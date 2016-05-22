/**********************************************************
 * File: opentag.h
 * Description: header file opentag C++ library for Arduino 
 * Version: 2.0
 * Created: 18.05.2016
 * Author: Stanislav Kamenik
 **********************************************************/
#ifndef _OPENTAG_H_
#define _OPENTAG_H_

#include <opentag_config.h>

// Arduino library
#ifdef ARDUINO
  #if (ARDUINO >= 100)
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif
#endif /* ARDUINO */
 
// For IAR EWAVR compatibility
#ifdef __IAR_SYSTEMS_ICC__
  #define ENABLE_BIT_DEFINITIONS
  #include <ioavr.h>
  #include <intrinsics.h>
  #include <stdint.h>
  #define interrupts __enable_interrupt 
#endif 


#define IO_CLK_FREQ_DEFAULT     16000000.0      // Timer clock freq (without prescaler) 
#define IR_FREQ_DEFAULT         56000.0         // Required IR Carrier freq

#ifndef IO_CLK_FREQ                             // user may predefine frequencies 
#define IO_CLK_FREQ IO_CLK_FREQ_DEFAULT         // in "opentag_config.h"
#endif                                          // in order to override defaults 

#ifndef IR_FREQ
#define IR_FREQ IR_FREQ_DEFAULT
#endif

#define IR_WFM_PERIOD       (IO_CLK_FREQ / IR_FREQ)
#define IR_WFM_PULSE        (uint8_t)(IR_WFM_PERIOD / 2 + 0.5)

#define IR_BURST_BREAK_US   2400
#define IR_BURST_0_US       600
#define IR_BURST_1_US       1200
#define IR_PAUSE_US         600

#define IR_PULSES_BREAK     (uint16_t)(IR_BURST_BREAK_US * IO_CLK_FREQ /(1000000.0 * IR_WFM_PULSE) + 0.5)
#define IR_PULSES_BURST_0   (uint16_t)(IR_BURST_0_US * IO_CLK_FREQ /(1000000.0 * IR_WFM_PULSE) + 0.5)
#define IR_PULSES_BURST_1   (uint16_t)(IR_BURST_1_US * IO_CLK_FREQ /(1000000.0 * IR_WFM_PULSE) + 0.5)
#define IR_PULSES_PAUSE     (uint8_t)(IR_PAUSE_US * IO_CLK_FREQ /(1000000.0 * IR_WFM_PULSE) + 0.5)

typedef enum {Pin_PB3, Pin_PD3}pin_t;           // For output pin selection

// Protocol definitions
#define TEAM_RED        0
#define TEAM_BLUE    0x40
#define TEAM_YELLOW  0x80
#define TEAM_GREEN   0xC0

#define DMG_1           0
#define DMG_2        0x04
#define DMG_4        0x08
#define DMG_5        0x0C
#define DMG_7        0x10
#define DMG_10       0x14
#define DMG_15       0x18
#define DMG_17       0x1C
#define DMG_20       0x20
#define DMG_25       0x24
#define DMG_30       0x28
#define DMG_35       0x2C
#define DMG_40       0x30
#define DMG_50       0x34
#define DMG_75       0x38
#define DMG_100      0x3C

/**
  * @class COpenTag
  * Open Tag IR Transmission
  */
class COpenTag
{
public:
  COpenTag();
  ~COpenTag();
  void Init(pin_t pin = Pin_PB3);
  void SendTagIR(uint16_t cmd, uint8_t nBits = 14);
  void SendTagIR(uint8_t* buf, uint8_t size);
  void SendShot(uint8_t id, uint8_t team, uint8_t damage);
  
private:
  void Timer_Init();
  void Timer_DeInit();
  void SetFlagsStart();
  
public:  
  // Member variables
  uint8_t m_TCCR2A_val;
  volatile uint16_t m_nBurst;
  volatile uint8_t m_nPause;
  volatile struct
  {
    unsigned carrierOn : 1;            
    unsigned burstEnd  : 1;
    unsigned pauseEnd  : 1;
  }m_flags;
  pin_t m_pin;
};

extern COpenTag opentag;


#endif /* _OPENTAG_H_ */