//ARDUINO POLARIMETER MOTORS CONTROLLER FIRMWARE
//AUTHOR: JUSTIN MOORE
//
// Power On.
// Wait for further instructions... e.g. sit in the loop portion of the code
// waiting for Serial interaction

//includes for the motor shield [gives access to the stepper motors]
#include <Wire.h>
#include <Adafruit_MotorShield.h>

//date last worked on... MM-DD-YY
String versionNum = "Mar-23-16";
String motorUnhomed = "MOTOR_NOT_HOMED";

//debugging
bool debug = 0;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree resolution)
// to motor port #1 (M1 and M2) - calling this V-BAND
Adafruit_StepperMotor *vHWP = AFMS.getStepper(200, 1);
// to motor port #2 (M3 and M4) - calling this I-BAND
Adafruit_StepperMotor *iHWP = AFMS.getStepper(200, 2);

//Generic pointer for later code reuse
Adafruit_StepperMotor *currentMotor;

//for looping
int i = 0;

//signal end of transmission on serial line
char delimChar = '+';

//Step delimeter in degrees for each polarization measurement
double stepDelim = 22.5; 

/**
GEAR RATIOS > #actual steps
º     @1.8d-res 60mm    30mm    15mm
0     0		0	0	0
22.5  12.5	25	50	100
45    25	50	100	200
67.5  37.5	75	150	300
*/

//This is the # of steps to fire @ our particular gear ratio to get 22.5
//actual degree spacing between measurements
int actualSteps = 25;

//The following can be 15, 30, or 60 giving ratio of 1/4, 1/2, 1, respectively.
int numSmallGearTeeth = 30;

//Compute the number of steps to move between imaging positions
//since we get 1.8d per step resolution by default, we can calculate what we need
//in order to get to our desired positions: [0,22.5,45,67.5]
//this gives us #steps below. 
//60 is hard-coded in bc that's the max gearteeth I have - and every other will 
//be a ratio

//NOTE: I'm currently using INTERLEAVE as my method of rotation, so that gives 
//half resolution. [0.9d per step] - So I need to multiply the final computed 
//number of steps by 2.

int numStepsPerImagingMode = actualSteps * (60/numSmallGearTeeth) * 2;

//above gives #steps per 22.5deg.  Multiply by 16 to get 360
//in other words, if you go 360 deg without hitting the limit ... throw an error.
int maxStepsBeforeError = numStepsPerImagingMode * 16;
int stepCount = 0;

//For the linear actuator [using Pulse Width Modulation]
//params for accel
int apwmDelim = 5;
int motorMaxSpd = 200; // of 255
int apwmDelayTime = 40; //ms

//params for decel
int dpwmDelim = 25;
int motorMinSpd = 30;
int dpwmDelayTime = 80; //ms

//Pin declaration area abbreviations
//HWP = Half Wave Plate
//LA = Linear Actuator
//HL = Home Limit

//each HWP assembly needs a home limit [zero position]
int V_HWP_HL = 10; //GREENWHITE-WHITE
int I_HWP_HL = 11; //ORANGEWHITE-BLUE

//LA needs dir,pwm, and 3 limits middle, left [V], right [I].
int LA_dir = 2; //YELLOW
int LA_pwm = 3; //BLUE
int LA_reset = 7; //RED
int LA_V = 4; //ORANGEWHITE-ORANGE
int LA_HL = 5; //GREENWHITE-BLACK
int LA_I = 6; //BLUEWHITE-RED

//ALL GREEN = GROUND

//cp = current position
int LA_cp = -1;
double V_HWP_cp = -1.0;
double I_HWP_cp = -1.0;
double currentPos = 0.0;

//limits for LA
int areWeHome = 0;
int areWeLeft = 0;
int areWeRight = 0;
int areWeThere = 0;

//timeout for LA movement (to prevent case where we're outside of our limits)
unsigned long timeoutAmountMs = 15000;//ms

//timeout for release steppers
unsigned long timeoutStepperMs = 1800000; //30min in ms
unsigned long lastInteraction = 0;
bool idle = false;
bool steppersReleased = false;

//time ON for nudge
int nudgeLengthMs = 1000;//ms

//stepDelay if one at a time
int stepDelayMs = 20; //ms

unsigned long nextUpdate = 0;
bool timeOutTriggered = false;

/*%%%%%%%%%%%%%%%%%%%%%%

setup variables, 
start serial listen,
wait for commands

%%%%%%%%%%%%%%%%%%%%%%%*/

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
  
  //AFMS = Adafruit Motor Shield
  AFMS.begin();  // create with the default frequency 1.6KHz
  //This changes the frequency of the PWM portion on the motors.
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // setup the stepper speed
  vHWP->setSpeed(10);  // 10 rpm   
  iHWP->setSpeed(10);  // 10 rpm   
  
  // initialize the motor limit pins:
  pinMode(V_HWP_HL, INPUT);
  pinMode(I_HWP_HL, INPUT);
  pinMode(LA_V, INPUT);
  pinMode(LA_HL, INPUT);
  pinMode(LA_I, INPUT);
  //turn on pull-up resistors:
  digitalWrite(LA_reset,HIGH);
  digitalWrite(V_HWP_HL,HIGH);
  digitalWrite(I_HWP_HL,HIGH);
  digitalWrite(LA_V,HIGH);
  digitalWrite(LA_HL,HIGH);
  digitalWrite(LA_I,HIGH);

  if (debug){
    Serial.print("Setup");
    printDelim();
    pinMode(13,OUTPUT);
  }
  
  lastInteraction = millis();
  idle = false;
}

void loop(){
  //here is where we spend most of the time
  String fullInput = "";
  char motor;
  char cmd;
  int cmdBreak = 0;
  int posVal = 0;
  int strleng = 0;
  //Chars for available possible motors to control
  String possibleMotors = "VIL";
  int msMove = 0;
  //flicker lights to see if we're in this loop
  if (debug){
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);              // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(100);              // wait for a second  
  }

  //if been idle, release steppers.
  if (millis() >= lastInteraction+timeoutStepperMs && idle == false){
    //release steppers[stop heat generation]
    idle = true;
    Serial.print("Idle > Rlsing Stpprs (Prvnt Excess Heat)");
    printDelim();
    releaseSteppers();
  }
  
  //read from serial
  if (Serial.available() > 0){
    //for this test, assume it's 'A=0+' where A is a cmd and 0 is an input
    //length will be 3 for all of these (not counting the delimChar)
    fullInput = Serial.readStringUntil(delimChar);
    strleng = fullInput.length();
    
    //update last interaction time to now
    lastInteraction = millis();
    idle = false;
    
    //if we have received a "valid" command... [in the sense that it's defined]
    if (strleng > 1 && strleng < 4 && fullInput.indexOf('=') != -1){
      
      //let's figure out which motor we're working on
      motor = fullInput.charAt(0);
      //subtract char '0' to get it to an int.
      posVal = fullInput.charAt(2) - '0';
      //make sure it's a valid motor / position combo before doing anything
      if (possibleMotors.indexOf(motor) != -1){
        //make sure it's a valid pos.
        if (posVal >= 0 && posVal <= 4){
          moveTo(motor,posVal);
        }
      }
    }
    //L=3,200+
    else if (fullInput.indexOf(',') != -1){
      //which motor we're working on
      motor = fullInput.charAt(0);
      //subtract char '0' to get it to an int.
      posVal = fullInput.charAt(2) - '0';
      //3 for R, 4 for L
      msMove = fullInput.substring(4,strleng).toInt();
      if (debug){
        Serial.print("Mv ");
        Serial.print(motor);
        Serial.print(" 2 pos ");
        Serial.print(posVal);
        Serial.print(" 4 ");
        Serial.print(msMove);
        Serial.print(" ms");
        printDelim();
      }
      nudgeLengthMs = msMove;
      moveTray(posVal);
    }
    //not a move command - maybe a predefined command
    else{
      cmd = fullInput.charAt(0);
      switch(cmd){
        case '@':
          //motor status on one line
          printStatus();
          break;
        case '#':
          //everything on one line
          printStatusFull();
          break;
        case '?':
          //help
          Serial.println("Opts:");
          Serial.println("'?' help");
          Serial.println("'S' status");
          Serial.println("'@' csv status");
          Serial.println("'#' csv status w lims");
          Serial.println("'H' home all m's");
          Serial.println("'R' rls stpprs");
          Serial.println("'V' drvr ver");
          Serial.println("'U' uptime");
          Serial.println("'A=0' mv m A to pos 0 [V,I,L]");
          Serial.println("'L=3,X' mv tr R for X ms");
          Serial.println("'L=4,X' mv tr L for X ms");
          printDelim();
          break;
        case 'S':
          //print positions for everything
          Serial.print("VHWP: ");
          Serial.println(V_HWP_cp);
          Serial.print("IHWP: ");
          Serial.println(I_HWP_cp);
          Serial.print("LA: ");
          Serial.print(LA_cp);
		      printDelim();
          break;
       case 'H':
          //rehome everyone
          findHome('V');
          delay(100);
          findHome('I');
          delay(100);
          steppersReleased = false;
          findHome('L');
          delay(100);
          printStatusFull();
          break;
        case 'V':
          //print out version
          Serial.print("V: ");          
          Serial.print(versionNum);
          printDelim();
          break;
        case 'U':
          //print out version
          Serial.print("U: ");          
          Serial.print(millis());
          Serial.print(" ms");
          printDelim();
          break;
        case 'R':
          //release steppers [no heat]
          releaseSteppers();
          break;
        default:
          Serial.print("Unknwn. try '?'");
	  printDelim();
          break;
      }
    }
  }
   
}

void printDelim(){
  Serial.print(delimChar);
}

void printStatus(){
  Serial.print(V_HWP_cp);
  Serial.print(",");
  Serial.print(I_HWP_cp);
  Serial.print(",");
  Serial.print(LA_cp);
  printDelim();
}

void printStatusFull(){
  Serial.print(V_HWP_cp);
  Serial.print(",");
  Serial.print(I_HWP_cp);
  Serial.print(",");
  Serial.print(LA_cp);
  Serial.print(",");
  Serial.print(digitalRead(LA_V));
  Serial.print(digitalRead(LA_HL));
  Serial.print(digitalRead(LA_I));
  Serial.print(digitalRead(V_HWP_HL));
  Serial.print(digitalRead(I_HWP_HL));
  printDelim();
}

void releaseSteppers(){
  vHWP->release();
  delay(100);
  iHWP->release();
  V_HWP_cp = -1.0;
  I_HWP_cp = -1.0;
  steppersReleased = true;
  Serial.print("Stpprs relsd.");
  printDelim();
  printStatusFull();
}

/**
this moves the tray but makes sure we're inside the limits...
if the tray gets outside of our limits, and we move left we will
hit the physical stop limit... and then sit there indefinitely.
So we need a timeout on motion-without-hitting-a-limit
*/
void timeoutMoveTray(int areWeHome, int areWeLeft, int areWeRight){
  //millis gives time since startup
  nextUpdate = millis() + timeoutAmountMs;
  timeOutTriggered = false;
  Serial.print("Wait 4 timeout (tray outsde lmts)");
  printDelim();
  while (areWeHome == HIGH && areWeLeft == HIGH && areWeRight == HIGH){
    //currently not on limits... so check again
    areWeHome = digitalRead(LA_HL);
    areWeLeft = digitalRead(LA_V);
    areWeRight = digitalRead(LA_I);
    if (millis() >= nextUpdate && timeOutTriggered == false){
      Serial.print("Timed out, swtch dir");
      printDelim();
      //stuck outside limits...switch dir and re-go
      analogWrite(LA_pwm,0);  //doesn't matter abrupt because is already stationary
      //switch dirs
      digitalWrite(LA_dir,1);
      //crank back up
      accelTray();
      //reset timeout
      nextUpdate = millis() + timeoutAmountMs;	
      //now we've already turned around once...
      timeOutTriggered = true;	
    }
    else if (millis() >= nextUpdate && timeOutTriggered == true){
      Serial.print("Dbl time-out, lmt swtch prob!");
      printDelim();
      analogWrite(LA_pwm,0);
      digitalWrite(LA_dir,0);
      LA_cp = -2;
      return;
    }
    else{
      //do nothing
    }
  }
  /*
  //make sure we are fully triggering the limit...
  delay(100); //hack-ish debounce
  //stop movement.
  analogWrite(LA_pwm,0);
  //in order to get out of that previous loop, we had to hit a limit
  //but which one? read again...  
  areWeHome = digitalRead(LA_HL);
  areWeLeft = digitalRead(LA_V);
  areWeRight = digitalRead(LA_I);
  */
  
  if (areWeHome == LOW){
    LA_cp = 1;
    decelTray(LA_HL);
    if (debug){
      Serial.print("Homed");
      printDelim();
      printStatusFull();
    }
    return;
  }
  else if (areWeLeft == LOW){
    LA_cp = 0;
    decelTray(LA_V);
    Serial.print("L [V], Go mid");
    printDelim();
    moveTray(1);
  }
  else {
    LA_cp = 2;
    decelTray(LA_I);
    Serial.print("R [I], Go mid");
    printDelim();
    moveTray(1);
    
  }
  return;
}

void findHome(char motor){
  if (debug){
    Serial.print("Home mtr: ");
    Serial.print(motor);  
    printDelim();
  }
  
  int homeLimitPin = 0;
  
  //sets up variables for HWP's
  //or actually moves for the LA
  //HWP movement occurs after this switch ends
  switch (motor) {
    case 'V':
      areWeHome = digitalRead(V_HWP_HL);
      currentMotor = vHWP;
      homeLimitPin = V_HWP_HL;
      V_HWP_cp = -1.0;
      break;
    case 'I':
      areWeHome = digitalRead(I_HWP_HL);
      currentMotor = iHWP;      
      homeLimitPin = I_HWP_HL;
      I_HWP_cp = -1.0;
      break;
    case 'L':
      //set dir and move til we hit a limit
      //10s timeout if we somehow get past the leftmost limit
      //set the direction to left [arbitrary]
      digitalWrite(LA_dir,0);
      //check limits
      areWeHome = digitalRead(LA_HL);
      areWeLeft = digitalRead(LA_V);
      areWeRight = digitalRead(LA_I);
      
      //are we already sitting at a limit?
      if (areWeHome == HIGH && areWeLeft == HIGH && areWeRight == HIGH){
        //nope... we're in limbo, just start moving left [arbitrary,recall]
        Serial.print("Tray lost, mv left");
        printDelim();
        //move slow
        //analogWrite(LA_pwm, motorMaxSpd);
        //accelerate using PWM
        accelTray();
        timeoutMoveTray(areWeHome,areWeLeft,areWeRight);
      }
      else{
          //we are already sitting at a limit, but which one?
          if (areWeHome == LOW){
            LA_cp = 1;
            if (debug){
              Serial.print("Tray homed!");
              printDelim();
            }
            return;
          }
          else if (areWeLeft == LOW){
            LA_cp = 0;
            Serial.print("L [V], Go mid");
            printDelim();
            moveTray(1);
          }
          else {
            LA_cp = 2;
            Serial.print("R [I], Go mid");
            printDelim();
            moveTray(1);
          }
        }
        return;
     default:
       Serial.print("ERR: unknwn mtr");
       printDelim();
       break;
    
  }  

  //if here, it means a home command for one of the steppers was issued.

  //if we released while homed.
  if (areWeHome == LOW && steppersReleased == true){
    currentMotor->step(1,FORWARD,INTERLEAVE); 
    delay(stepDelayMs);
    currentMotor->step(1,BACKWARD,INTERLEAVE); 
    if (debug){
      Serial.print("Nudged2engage stppr hold");
    }    
  }
  //read limit
  //take a step
  //check step < max step
  //repeat.
  while (areWeHome != LOW){
    //THIS IS WHERE WE USE INTERLEAVE INSTEAD OF ANOTHER OPTION
    currentMotor->step(1,FORWARD,INTERLEAVE); 
    delay(stepDelayMs);
    stepCount+=1;
    if (stepCount>=maxStepsBeforeError){
      //throw Error e;
      Serial.print("ERR: chk lmt swtch");
      printDelim();
      //reset step count!
      stepCount=0;
      if (homeLimitPin == V_HWP_HL){
        V_HWP_cp = -2.0;
      }
      else if (homeLimitPin == I_HWP_HL){
        I_HWP_cp = -2.0;
      }
      return;
    }
    areWeHome = digitalRead(homeLimitPin);
  }
  if (debug){
    Serial.print("Homed:  ");
    Serial.print(stepCount);
    Serial.print(" steps");
    printDelim();
  }
  //reset step count!
  stepCount=0;

  if (idle){
    
  }
  if (homeLimitPin == V_HWP_HL){
    V_HWP_cp = 0;
  }
  else if (homeLimitPin == I_HWP_HL){
    I_HWP_cp = 0;
  }
}

void moveTo(char motor, int posNum) {  
  //this sets up generic variables for the HWP or sends cmd for the LA
  switch (motor){
    case 'V':
      currentMotor = AFMS.getStepper(200, 1);
      currentPos = V_HWP_cp;
      if (V_HWP_cp<0){
        Serial.print(motorUnhomed);
        printDelim();
        return;
      }
      break;
    case 'I':
      currentMotor = AFMS.getStepper(200, 2);      
      currentPos = I_HWP_cp;
      if (I_HWP_cp<0){
        Serial.print(motorUnhomed);
        printDelim();
        return;
      }
      break;
    case 'L':
    if (LA_cp<0){
        Serial.print(motorUnhomed);
        printDelim();
        return;
      }
      moveTray(posNum);
      return;
    default:
      Serial.print("Unknown mtr");
      printDelim();
      return;
  }

  if (posNum > 3){
    Serial.print("ERR: pos outside known vals");
    printDelim();
    printStatusFull();
    return;
  }
  
  //posNum requested multiplied by numSteps per posNum
  double desiredPos = stepDelim * posNum;
  
  if (currentPos == desiredPos) {
    //do nothing... already there...
    Serial.print("Already there!");
    printDelim();
    printStatusFull();
    return;
  }
  
  
  int multiplier = (desiredPos - currentPos) / stepDelim;
  
  if (multiplier < 0) {
    if (debug){
      Serial.print("Mving Fwd!");
      printDelim();
    }
    currentMotor->step((-1*multiplier)*numStepsPerImagingMode,FORWARD,INTERLEAVE); 
  }
  else {
    if (debug){
      Serial.print("Mving Bckwd!");      
      printDelim();
    }
    currentMotor->step(multiplier*numStepsPerImagingMode,BACKWARD,INTERLEAVE); 
  }  
  
  currentPos = desiredPos;
  
  if (motor == 'V'){
    V_HWP_cp = currentPos;
    //now V_HWP_HL should be activated.  If it's not, we need to adjust.
    if (desiredPos == 0){
      if (digitalRead(V_HWP_HL)!=LOW){
        delay(100);
        stepCount=0;
        while (digitalRead(V_HWP_HL)!=LOW){
          currentMotor->step(1,FORWARD,INTERLEAVE); 
          delay(stepDelayMs);
          stepCount+=1;
        }
        Serial.print("VHWP took ");
        Serial.print(stepCount);
        Serial.print(" xtra steps to home.");
        printDelim();
        stepCount=0;
      }
    }
  }
  else{
    I_HWP_cp = currentPos;
    //now I_HWP_HL should be activated.  If it's not, we need to adjust.
    if (desiredPos == 0){
      if (digitalRead(I_HWP_HL)!=LOW){
        delay(100);
        stepCount=0;
        while (digitalRead(I_HWP_HL)!=LOW){
          currentMotor->step(1,FORWARD,INTERLEAVE); 
          delay(stepDelayMs);
          stepCount+=1;
        }
        Serial.print("IHWP took ");
        Serial.print(stepCount);
        Serial.print(" xtra steps to home.");
        printDelim();
        stepCount=0;
      }
    }
  }
    
  printStatusFull();
}
/**
 * Accelerate PWM
 */
void accelTray(){
  //loop over pwm levels and accelerate the tray to full speed(or motorMaxSpd)
  for (int i=motorMinSpd; i<=motorMaxSpd; i+=apwmDelim){
    analogWrite(LA_pwm,i);
    delay(apwmDelayTime);
  }
  if (debug){
    Serial.print("Full spd ahead!");
    printDelim();
  }
}

/**
 * Decelerate PWM
 */
void decelTray(int specifiedLimit){
  areWeThere = 0;
  //should be moving at full speed at this point
  if (debug){
    Serial.print("Watch 4 lmt!");
    printDelim();
  }
  
  areWeThere = digitalRead(specifiedLimit);
  
  if (areWeThere == LOW){
    if (debug){
      Serial.print("Hit lmt, decel..");
      printDelim();
    }
  }
  else{
    nextUpdate = millis() + timeoutAmountMs;
    while (true){
      areWeThere = digitalRead(specifiedLimit);
      if (millis() <= nextUpdate){
        if (areWeThere == LOW){
          if (debug){
            Serial.print("Hit lmt, decel..");
            printDelim();
          }
          break;
        }
      }
      else{
        Serial.print("Timed out - chk limits!");
        printDelim();
        analogWrite(LA_pwm,0);
        digitalWrite(LA_dir,0);
        LA_cp = -2;
        printStatusFull();
        return;
      }
    }
  }
  
  //we hit the limit so actually start decelerating now!
  for (int i=motorMaxSpd; i>=motorMinSpd; i-=dpwmDelim){
    analogWrite(LA_pwm,i);
    delay(dpwmDelayTime);
  }

  //just to be sure pwm is actually off and reset the dir pin to its default (0)
  analogWrite(LA_pwm,0);
  digitalWrite(LA_dir,0);

  if (debug){
    Serial.println("Done decel..");
    printDelim();
  }

  printStatusFull();
}

/* 
  LA code to move (correct dir) to hit limit for posNum
  0 = LEFTMOST = V
  1 = MIDDLE = OPEN
  2 = RIGHTTMOST = I
  3 = NUDGE RIGHT [on for nudgeLengthMs]
  4 = NUDGE LEFT [on for nudgeLengthMs]
 */
void moveTray(int posNum){
  int activeLimit = 0;
  areWeThere = 0;
  if (LA_cp == posNum){
    //do nothing... already there...
    Serial.print("Already there!");
    printDelim();
    printStatusFull();
    return;
  }
  else{
    switch(posNum){
      case 0:
        activeLimit = LA_V;
        break;
      case 1:
        activeLimit = LA_HL;
        break;
      case 2:
        activeLimit = LA_I;
        break;
      case 3:
        //move right for nudgeLengthMs
        digitalWrite(LA_dir,1);
        analogWrite(LA_pwm, motorMaxSpd/2);
              
        delay(nudgeLengthMs);
        analogWrite(LA_pwm, 0);
        digitalWrite(LA_dir,0);
        LA_cp = -1;
        printStatusFull();
        return;
      case 4:
        //move left for nudgeLengthMs
        digitalWrite(LA_dir,0);
        analogWrite(LA_pwm, motorMaxSpd/2);       
        delay(nudgeLengthMs);
        analogWrite(LA_pwm, 0);
        LA_cp = -1;        
        printStatusFull();
        return;
      default:
        Serial.print("ERR: unknown motor pos");
        printDelim();
        return;
    }
    if (LA_cp > posNum)
      //go left
      digitalWrite(LA_dir,0);
    else
      //go right
      digitalWrite(LA_dir,1);

    //millis gives time since startup
    nextUpdate = millis() + timeoutAmountMs;
    timeOutTriggered = false;
    Serial.print("Wait for timeout (in normal motion)");
    printDelim();
    
    //actually waiting for a limit trigger below  
    areWeThere = digitalRead(activeLimit);
    
    //if for some reason we're already at that limit ... don't do anything
    if (areWeThere != LOW){
        //analogWrite(LA_pwm,motorMaxSpd);
        accelTray();
    }
    
    //wait for limit to trigger
    while (areWeThere != LOW){  
      if (millis()>= nextUpdate){
        Serial.print("Timed out going to expected position...");
        printDelim();
        //if we time out, abruptly stopping doesn't matter bc
        //it is already sitting at a hard limit
        analogWrite(LA_pwm,0);
        digitalWrite(LA_dir,0);
        LA_cp = -2;
        printStatusFull();
        return;
      }
      areWeThere = digitalRead(activeLimit);
    }
    /**   
    //make sure we are fully triggering the limit...
    delay(100); //hack-ish debounce
    analogWrite(LA_pwm,0);
    */
    decelTray(activeLimit);
    //shouldn't matter but lets turn this back off too.
    digitalWrite(LA_dir,0);
    LA_cp = posNum;    
    printStatusFull();
  }
}
