#include "telemetry.h"
#include "sensorcomm.h"
#include "inertial.h"

void setup() {
  setupAllComms();
}
int counter = 0;
void loop() {
  //telem_fakedata(123, 4567);
  /*telem_morefakestuff(3.2, 1000000, 2.1673453459345);
  telem_strmessage("blahblah");
  //Serial.print("garbage");
  delay(1000);*/
  //printImuData();


  int startTime = micros();

  readImu();

  int endTime = micros();
  // make 200hz loop
  delayMicroseconds(5000 - (endTime - startTime));
}
