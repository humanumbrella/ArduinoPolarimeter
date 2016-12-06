//for looping
int i = 0;
//For the linear actuator [using PWM]
//params for accel
int apwmDelim = 5;
int motorMaxSpd = 200; // of 255
int apwmDelayTime = 40; //ms

//params for decel
int dpwmDelim = 25;
int motorMinSpd = 30;
int dpwmDelayTime = 80; //ms

//signal end of transmission on serial line
char delimChar = '+';

//LA needs dir,pwm, and 3 limits middle, left [V], right [I].
int LA_dir = 2; //YELLOW
int LA_pwm = 3; //BLUE
int LA_reset = 7;
int LA_FF1 = 8;
int LA_FF2 = 9;
int LA_V = 4; //ORANGEWHITE-ORANGE
int LA_HL = 5; //GREENWHITE-BLACK
int LA_I = 6; //BLUEWHITE-RED
unsigned long timing = 0;

int atV=0;
int atH=0;
int atI=0;

void setup() {
  
  // initialize serial communication:
  Serial.begin(57600); 
  //flush the line
  Serial.flush();
  
  //set up for the LA tray
  pinMode(LA_dir,OUTPUT);
  pinMode(LA_pwm,OUTPUT);
  pinMode(LA_reset,OUTPUT);
  analogWrite(LA_pwm,0);
  digitalWrite(LA_dir,0);

  
  pinMode(LA_V, INPUT);
  pinMode(LA_HL, INPUT);
  pinMode(LA_I, INPUT);
  pinMode(LA_FF1, INPUT);
  pinMode(LA_FF2, INPUT);
  
  digitalWrite(LA_reset,LOW);
  delay(100);
  digitalWrite(LA_reset,HIGH);
  digitalWrite(LA_V,HIGH);
  digitalWrite(LA_HL,HIGH);
  digitalWrite(LA_I,HIGH);
}

void loop() {
  //here is where we spend most of the time
  String fullInput = "";
  int strlen = 0;
  //read from serial
  if (Serial.available() > 0){
    //for this test, assume it's 'A=0+' where A is a cmd and 0 is an input
    //length will be 3 for all of these (not counting the delimChar)
    fullInput = Serial.readStringUntil(delimChar);
    strlen = fullInput.length();

    if (strlen == 1){
      if (fullInput.charAt(0) - '0' == 0){
        Serial.println("Going Left!");
        
        Serial.println(digitalRead(LA_FF1));
        Serial.println(digitalRead(LA_FF2));
        goLeft();
      }
      else if (fullInput.charAt(0)-'0' == 1){
        resetLA();
      }
      else{
        Serial.println("Going Right!");
        
        Serial.println(digitalRead(LA_FF1));
        Serial.println(digitalRead(LA_FF2));
        goRight();
      }
    }
    //longer string
    //expecting pwmDelim,motorMaxSpeed,delayTime
    else{
      apwmDelim = getValue(fullInput,',',0).toInt();
      motorMaxSpd = getValue(fullInput,',',1).toInt();
      apwmDelayTime = getValue(fullInput,',',2).toInt();
      dpwmDelim = getValue(fullInput,',',3).toInt();
      motorMinSpd = getValue(fullInput,',',4).toInt();
      dpwmDelayTime = getValue(fullInput,',',5).toInt();
      Serial.print("aPWMDelim: ");
      Serial.println(apwmDelim);
      Serial.print("MotorMaxS: ");
      Serial.println(motorMaxSpd);
      Serial.print("aDelayTime: ");
      Serial.println(apwmDelayTime);
      Serial.print("dPWMDelim: ");
      Serial.println(dpwmDelim);
      Serial.print("MotorMinS: ");
      Serial.println(motorMaxSpd);
      Serial.print("dDelayTime: ");
      Serial.println(dpwmDelayTime);
    }
  }
}

void resetLA(){
  digitalWrite(LA_reset,LOW);
  delay(100);
  digitalWrite(LA_reset,HIGH);
  
  Serial.println(digitalRead(LA_FF1));
  Serial.println(digitalRead(LA_FF2));
}

String getValue(String data, char separator, int index)
{
 int found = 0;
  int strIndex[] = {
0, -1  };
  int maxIndex = data.length()-1;
  for(int i=0; i<=maxIndex && found<=index; i++){
  if(data.charAt(i)==separator || i==maxIndex){
  found++;
  strIndex[0] = strIndex[1]+1;
  strIndex[1] = (i == maxIndex) ? i+1 : i;
  }
 }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void goLeft(){
  digitalWrite(LA_dir,0);
  pwmMove(0);
}
void goRight(){
  digitalWrite(LA_dir,1);
  pwmMove(1);
}
void pwmMove(int dir){
  
  timing = millis();
  for (int i=motorMinSpd; i<=motorMaxSpd; i+=apwmDelim){
    analogWrite(LA_pwm,i);
    delay(apwmDelayTime);
  }

  dir = digitalRead(LA_dir);
  
  while(true){
    atV = digitalRead(LA_V);
    atH = digitalRead(LA_HL);
    atI = digitalRead(LA_I);
    
    //check for limit
    if (dir == 0){
      if (atV == LOW or atH == LOW){
        break;
      }
    }
    else if (dir == 1){
      if (atI == LOW or atH == LOW){
        break;
      }
    }
    else{
      
    }
  }
  //timing = millis();
  for (int i=motorMaxSpd; i>=motorMinSpd; i-=dpwmDelim){
    analogWrite(LA_pwm,i);
    delay(dpwmDelayTime);
  }
  
  
  Serial.println(millis()-timing);
  analogWrite(LA_pwm,0);
  digitalWrite(LA_dir,0);
}
void printDelim(){
  Serial.print(delimChar);
}
