#include "telemetry.h"

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(5);
}

void loop() {
  telem_fakedata(123, 4567);
  telem_morefakestuff(3.2, 1000000, 2.1673453459345);
  telem_strmessage("blahblah");
  //Serial.print("garbage");
  delay(1000);
}
