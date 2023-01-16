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
}

int sourceDutyCycle = 0;
boolean onAutopilot = false;
boolean checkedStatusThisCycle = false;

void loop() {
  if(digitalRead(6) == HIGH && sourceDutyCycle == 0){
    sourceDutyCycle = micros();
    checkedStatusThisCycle = false;
  }else if(digitalRead(5) == HIGH && micros() - sourceDutyCycle >= 1900){
    onAutopilot = true;
    checkedStatusThisCycle = true;
  }else if(digitalRead(5) == LOW && micros() - sourceDutyCycle < 1900){
    onAutopilot = false;
    checkedStatusThisCycle = true;
  }else if(digitalRead(5) == LOW && checkedStatusThisCycle){
    sourceDutyCycle = 0;
  }

  if(onAutopilot){
  }else{
    for(int i = 0; i++; i<6){
      digitalWrite(8 + i, digitalRead(2+ i));
    }
  }
}
