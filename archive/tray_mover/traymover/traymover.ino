
//for looping
int i = 0;
//signal end of transmission on serial line
char delimChar = '+';
//for the linear actuator
//these are for using PWM on the actuator.
int pwmDelim = 10;
int motorMaxSpd = 60;
int pwmDelayTime = 100;

//LA needs dir,pwm, and 3 limits middle, left, right.
int LA_dir = 2; //YELLOW
int LA_pwm = 3; //BLUE
int LA_HL_V = 4; //ORANGEWHITE
int LA_HL_O = 5; //GREENWHITE
int LA_HL_I = 6; //BLUEWHITE

//current positions
int LA_cp = 0;
double V_HWP_cp = 0.0;
double I_HWP_cp = 0.0;
double currentPos = 0.0;

void setup() {
  // initialize serial communication:
  Serial.begin(57600); 
  //flush the line
  Serial.flush();

  //set up for the LA tray
  pinMode(LA_dir,OUTPUT);
  pinMode(LA_pwm,OUTPUT);
  analogWrite(LA_pwm,0);
  digitalWrite(LA_dir,0);

  // initialize the motor limit pins:
  pinMode(LA_HL_V, INPUT);
  pinMode(LA_HL_O, INPUT);
  pinMode(LA_HL_I, INPUT);
  //turn on pullup resistors:
  digitalWrite(LA_HL_V,HIGH);
  digitalWrite(LA_HL_O,HIGH);
  digitalWrite(LA_HL_I,HIGH);

  //send the LA motor off looking for the home limit switch LA_HL_O
  //findHome('L');

}

void loop(){
  //here is where we spend most of our time
  String fullInput = "";
  char motor;
  char cmd;
  int cmdBreak = 0;
  int posVal = 0;
  int strlen = 0;
  String possibleMotors = "L";

  //read from serial
  if (Serial.available() > 0){
    //for this test, assume it's 'A=0+' where A is cmd and 0 is input
    //length will be 3 for all of these
    fullInput = Serial.readStringUntil(delimChar);
    strlen = fullInput.length();

    //if we have received a valid command...
    if (strlen > 1 && strlen < 4 && fullInput.indexOf('=') != -1){
      //which motor we're working on
      motor = fullInput.charAt(0);
      //subtract char '0' to get it to an int.
      posVal = fullInput.charAt(2) - '0';
      //make sure it's a valid motor.
      if (possibleMotors.indexOf(motor) != -1){
        //make sure it's a valid pos.
        if (posVal >= 0 && posVal <= 3){
          //moveTo(motor,posVal);
        }
      }
    }

    else{
      cmd = fullInput.charAt(0);
      switch(cmd){
      default:
        Serial.println("Command not recognized.  Type '?' for help.");
        printDelim();
        break;
      case 'L':
        //set the direction to left [arbitrary]
        digitalWrite(LA_dir,0);
        Serial.println("Set dir to left");
        break;
      case 'R':
        //set the direction to left [arbitrary]
        digitalWrite(LA_dir,1);
        Serial.println("Set dir to right");        
        break;
      case 'G':
        //set the direction to left [arbitrary]
         analogWrite(LA_pwm,motorMaxSpd);
        Serial.println("Set pwm to motorMaxSpd");
        break;
      case 'S':
        //set the direction to left [arbitrary]
        digitalWrite(LA_pwm,0);
        Serial.println("Set pwm to 0");
        break;
      case 'H':
        //rehome everyone
        findHome('L');
        break;
      case 'V':
        //move tray to V band polarimetric imaging
        moveTray(0);
        break;
      case 'O':
        //move tray to open imaging
        moveTray(1);
        break;
      case 'I':
        //move tray to I band polarimetric imaging
        moveTray(2);
        break;
      }
    }
  }

}

void printDelim(){
  Serial.println(delimChar);
}

void findHome(char motor){
  Serial.println("Searching for home on motor... ");
  Serial.println(motor);
  int areWeHome = 0;
  int homeLimitPin = 0;
  int areWeLeft = 0;
  int areWeRight = 0;

  switch (motor) {
  case 'L':
    //set dir and move til hit limit 1

    //int LA_HL_V = 4;
    //int LA_HL_O = 5;
    //int LA_HL_I = 6;

    //set the direction to left [arbitrary]
    digitalWrite(LA_dir,0);
    areWeHome = digitalRead(LA_HL_O);
    areWeLeft = digitalRead(LA_HL_V);
    areWeRight = digitalRead(LA_HL_I);

    //are we already sitting at a limit?
    if (areWeHome == HIGH && areWeLeft == HIGH && areWeRight == HIGH){
      //nope... we're in limbo, just start moving left.
      Serial.println("we dun got lost, moving...");
      //move slow [TODO pwm maybe]
      analogWrite(LA_pwm, motorMaxSpd);
    }
    else{
      //we are sitting at a limit, but which one?
      if (areWeHome == LOW){
        LA_cp = 1;
        Serial.println("We're home!  All done.");
        return;
      }
      else if (areWeLeft == LOW){
        LA_cp = 0;
        Serial.println("Left pos [V], Go middle.");
        moveTray(1);
      }
      else {
        LA_cp = 2;
        Serial.println("Right pos [I], Go middle");
        moveTray(1);

      }
    }

    //don't know where we are
    while (areWeHome != LOW && areWeLeft != LOW && areWeRight != LOW){  
      areWeLeft = digitalRead(LA_HL_V);
      areWeHome = digitalRead(LA_HL_O);
      areWeRight = digitalRead(LA_HL_I);
    }     
    //we hit a limit, stop moving.
    analogWrite(LA_pwm,0);
    Serial.println();
    Serial.println("Found where we are!");
    Serial.println(areWeLeft);
    Serial.println(areWeHome);
    Serial.println(areWeRight);

    //we are at a limit, but which one?
    if (areWeLeft == LOW){
      LA_cp = 0;
    }
    else if (areWeHome == LOW){
      LA_cp = 1;
    }
    else {
      LA_cp = 2;
    }
    //pause 500 ms
    delay(500);    
    Serial.println("Moving to middle if necessary.");
    //now we know where we are - move to the middle, aka OPEN position.
    moveTray(1);
    return;
  default:
    Serial.println("Unrecognized motor.");
    break;

  }
  Serial.println("Found home!");
  Serial.print("0");
  printDelim();
}


void moveTray(int posNum){
  //LA code to move (correct dir) to hit limit for posNum
  //0 = LEFTMOST = V
  //1 = MIDDLE = OPEN
  //2 = RIGHTTMOST = I
  int activeLimit = 0;
  int areWeHome = 0;
  if (LA_cp == posNum){
    //do nothing
    Serial.println("Already at that position!");
    return;
  }
  else{
    activeLimit = LA_HL_V + posNum; //active limit is on pin 4,5,6 (V is 4)
    if (LA_cp > posNum)
      digitalWrite(LA_dir,0);
    else
      digitalWrite(LA_dir,1);

    areWeHome = digitalRead(activeLimit);
    if (areWeHome != LOW){
      analogWrite(LA_pwm,motorMaxSpd);
    }
    while (areWeHome != LOW){  
      areWeHome = digitalRead(activeLimit);
    }   
    analogWrite(LA_pwm,0);
    LA_cp = posNum;
    Serial.println("Made it!");
  }

}

