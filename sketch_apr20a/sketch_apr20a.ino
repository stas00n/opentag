
#include <opentag.h>

uint8_t buf[] = {0x83,0x05,0xE8};

void setup() {
  opentag.Init(Pin_PB3);
  // put your setup code here, to run once:
}

void loop() {
  opentag.SendTagIR(buf, sizeof(buf));
  delay(50);
  // put your main code here, to run repeatedly:
}
