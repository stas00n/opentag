#include <opentag.h>
#define buttonPin 2

uint8_t playerID = 10;

void setup() 
{
  opentag.Init(Pin_PD3);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(13,OUTPUT);
}

void loop()
{
  while(digitalRead(buttonPin) == HIGH){}
  digitalWrite(13, HIGH);
  opentag.SendShot(playerID, TEAM_BLUE, DMG_25);
  digitalWrite(13, LOW);
  while(digitalRead(buttonPin) == LOW){}
  delay(5);
}
