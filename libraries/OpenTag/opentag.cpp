/**********************************************************
 * File: opentag.cpp
 * Description: source file opentag C++ library for Arduino 
 * Version: 2.0
 * Created: 18.05.2016
 * Author: Stanislav Kamenik
 **********************************************************/
#include "opentag.h"

COpenTag::COpenTag()
{
}

COpenTag::~COpenTag()
{
}

/**
  * @brief Confifures port pin and sets internal use timer control values
  * @param pin - desired IO pin to generate signal - must be one of Pin_PB3(default) or Pin_PD3
  * @retval none
  * @note pin_t is enumerated type in "opentag.h"
  */
void COpenTag::Init(pin_t pin)
{
  m_pin = pin;
  
  if(m_pin == Pin_PB3)                          // PB3/OC2A used for IR LED; configure it as output
  {
    DDRB  |= (1 << 3);

#if IRLED_ACTIVE_LEVEL                          // Set out non-active level (IR LED off)
    PORTB &= ~(1 << 3);
#else
    PORTB |= (1 << 3);
#endif
    
    m_TCCR2A_val = (1 << COM2A0 | 1 << WGM21);  // For Toggle OC2A Pin
  }
  
  if(m_pin == Pin_PD3)                          // PD3/OC2B used for IR LED; configure it as output
  {
    DDRD  |= (1 << 3);

#if IRLED_ACTIVE_LEVEL
    PORTD &= ~(1 << 3);
#else
    PORTD |= (1 << 3);
#endif 
    
    m_TCCR2A_val = (1 << COM2B0 | 1 << WGM21);  //For Toggle OC2B Pin
  }
}

/**
  * @brief Prepare Timer2 for transmission
  * @retval none
  */
void COpenTag::Timer_Init()
{
  TIMSK2 = 0;                   // Disable interrupts
  TIFR2 = (1 << TOV2) | (1 << OCF2A) | (1 << OCF2B);// Clear pending flags if any
  TCCR2B = 0;                   // Disable clock
  TCNT2 = 0;                    // Reset TCNT
  TCCR2A = (1 << WGM21);        // Select CTC mode
  OCR2A = IR_WFM_PULSE;
  OCR2B = IR_WFM_PULSE;
  TIMSK2 = (1 << OCIE2A);      //Enable Output compare A interrupt
}

/**
  * @brief Stop Timer2 and disable its interrupts
  * @retval none
  */
void COpenTag::Timer_DeInit()
{
  TCCR2B = 0;                                       // Stop clock
  TIMSK2 = 0;                                       // Disable interrupts
  TIFR2 = (1 << TOV2) | (1 << OCF2A) | (1<< OCF2B); // Clear pending flags if any
}

/**
  * @brief Set transmission flags for start bit transmission
  * @retval none
  */
void COpenTag::SetFlagsStart()
{
  m_flags.burstEnd = 0;
  m_flags.pauseEnd = 0;
  m_flags.carrierOn = 1; 
}

/**
  * @brief Sends IR packet of BREAK followed by data
  * @param cmd: command to send
  * @param nBits: number of data bits to send, default value 14
  * @retval none
  * @note transmission starts from most significant bit(15),
  *  so cmd should be left-aligned
  * @note if nBits is 0, only BREAK will be transmitted
  */
void COpenTag::SendTagIR(uint16_t cmd, uint8_t nBits)
{
  Timer_Init();
  interrupts();                 // Global interrupt enable
  
  m_nBurst = IR_PULSES_BREAK;   // Load Break burst counter
  SetFlagsStart();
  TCCR2B = (1 << CS20);         // Enable clock
  
  nBits += 1;                   // loop nBits + Break times
  while(nBits--)
  {
    SetFlagsStart();
    m_nPause = IR_PULSES_PAUSE; // Load pause counter
    
    while(!m_flags.burstEnd){}  // Wait burst time

    m_flags.carrierOn = 0;      // Carrier off, pause
    
    // Load next burst counter
    if(cmd & 0x8000) {m_nBurst = IR_PULSES_BURST_1;}
    else             {m_nBurst = IR_PULSES_BURST_0;}
    
    cmd <<= 1;                  // prepare next data bit
    while(!m_flags.pauseEnd){}  // wait pause time  
  }
  Timer_DeInit();
}

/**
  * @brief Sends IR packet of BREAK followed by data
  * @param pBuf: pointer to data buffer 
  * @param size: number of data bytes to send (max 255) 
  * @retval none
  */
void COpenTag::SendTagIR(uint8_t* pBuf, uint8_t size)
{
  Timer_Init();
  interrupts();                 // Global interrupt enable
  
  m_nBurst = IR_PULSES_BREAK;   // Load Break burst counter
  SetFlagsStart();
  TCCR2B = (1 << CS20);         // Enable clock
  

  uint8_t bit = 8;
  uint8_t bufIndex = 0;
  uint8_t byte = *pBuf;          // Load first byte

  for(uint16_t i = (size * 8 + 1); i != 0; i--)
  {
    SetFlagsStart();
    
    m_nPause = IR_PULSES_PAUSE; // Load pause counter
    
    while(!m_flags.burstEnd){}  // Wait burst time
    m_flags.carrierOn = 0;      // Carrier off, pause
    
    // Load next burst counter
    if(byte & 0x80) {m_nBurst = IR_PULSES_BURST_1;}
    else            {m_nBurst = IR_PULSES_BURST_0;}
    
    // prepare next data bit/byte
    byte <<= 1;                 
    if(--bit == 0)
    {
      bit = 8;
      if(++bufIndex < size){byte = *(pBuf + bufIndex);}
    }
    while(!m_flags.pauseEnd){}  // wait pause time 
  }
  Timer_DeInit();
}

/**
  * @brief Sends "shot" IR packet - 14 bit, left aligned
  * @param id: Player ID (range 0...127)
  * @param team: Team color (bitmask, ex. TEAM_RED)
  * @param damage: Damage (bitmask, ex. DMG_1)
  * @retval none
  * @note availible bitmasks defined in "opentag.h"
  */
void COpenTag::SendShot(uint8_t id, uint8_t team, uint8_t damage)
{
  uint16_t shot = (id << 8) | team | damage;
  shot &= 0x7FFF;
  SendTagIR(shot);
}

COpenTag opentag;


// Timer 2 Output compare match A ISR
// for IR carrier generation
// and burst timing control
#ifdef ARDUINO
ISR (TIMER2_COMPA_vect) 
#endif

#ifdef __IAR_SYSTEMS_ICC__
#pragma vector = TIMER2_COMPA_vect
__interrupt void TIMER2_COMPA_isr()
#endif
{
  if (opentag.m_flags.carrierOn)    
  {
    TCCR2A = opentag.m_TCCR2A_val; // Toggle OC2A or OC2B pin
    if(--opentag.m_nBurst == 0) {opentag.m_flags.burstEnd = 1;}
  }
  else
  {
    TCCR2A = (1 << WGM21);         // Disconnect OC2A & OC2B pins
    if(--opentag.m_nPause == 0) {opentag.m_flags.pauseEnd = 1;}
  }
}

