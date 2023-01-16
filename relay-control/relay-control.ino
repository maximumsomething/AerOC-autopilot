//pins D0-5: receiver input
//pins A0-4: teensy input
//pins D6-10: servo output

void setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  Serial.begin(115200);
}

unsigned long autoSignalTimer = 0;
boolean onAutopilot = false;
boolean checkedStatus = false;
char outputState = 0;

void loop() {
  if((PIND & 0b10000000) && !checkedStatus){
    autoSignalTimer = micros();
    //Serial.println("w");
    //Serial.println(autoSignalTimer);
    checkedStatus = true;
  }else if(!(PIND & 0b10000000) && checkedStatus){
    if(micros() - autoSignalTimer > 1800){
      onAutopilot = true;
    }else{
      onAutopilot = false;
    }
    checkedStatus = false;
  }
  
  if(onAutopilot){
    PORTB = (PORTB & 0b11000000) | (PINC & 0b00111111);
  }else{
    PORTB = (PORTB & 0b11000000) | (PIND >> 2);
  }
}
