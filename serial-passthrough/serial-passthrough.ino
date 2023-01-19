#define passthrough Serial5

void setup() {
  Serial.begin(115200);
  passthrough.begin(9600);
  while(!Serial || !passthrough) { delay(5); }

}

void loop() {
  
  if (Serial.available()) {      // If anything comes in Serial (USB),
    passthrough.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (Serial1.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    Serial.write(passthrough.read());   // read it and send it out Serial (USB)
  }
}
