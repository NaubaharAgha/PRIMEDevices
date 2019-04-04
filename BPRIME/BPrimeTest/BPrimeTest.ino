//#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#define ENCODER_DO_NOT_USE_INTERRUPTS // SINCE OUR ENCODER DOESN'T USE INTERRUPTS, WE MUST DEFINE THIS BEFORE CALLING THE LIBRARY
#include <Encoder.h>
#include <Stepper.h>
#include "BPrimeTest.h"

void setup()
{
  // Set Encoder pins as input
  /*
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor
  */
  
  // Initial value of encoder pins
  PastA = (boolean)digitalRead(encoder0PinA); //initial value of channel A;
  PastB = (boolean)digitalRead(encoder0PinB); //and channel B
  
  // Attach servo, initialize LED pin, and LCD outputs
  //servo_0.attach(servoPin);
  //servo_0.write(angle);  // Initialize to front center angle

  myStepper.setSpeed(stepperSpeed);
  pinMode(boardLED, OUTPUT);
  pinMode(pulPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enblPin, OUTPUT);
  resetMotorPins(); // Initialize Motor Driver pins

  // Set IR sensor pins as input
  pinMode(sensorRT, INPUT);
  pinMode(sensorRB, INPUT);
  pinMode(sensorLT, INPUT);
  pinMode(sensorLB, INPUT);
  
  // Set IR inputs (all go into same pin) as an interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(int2Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), sensorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(int2Pin), sensorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hardDirPin), dirInterrupt, CHANGE);
  
  pinMode(startTrialTrigger, INPUT_PULLDOWN);
  //digitalRead(startTrialTrigger);

  //Set input button to move motor manually to a pulldown input
  pinMode(but, INPUT_PULLDOWN);
  
  // Initialize Cue Light Strip
  cueStrip.begin();
  cueStrip.show(); // Initialize all pixels to 'off'
  cueStrip.setBrightness(LEDBrightness);

  // Initialize Cue Light Strip
  barrelrollStrip.begin();
  barrelrollStrip.setBrightness(LEDBrightness);
  barrelrollStrip.setPixelColor(0, orange); // Left is Orange
  barrelrollStrip.setPixelColor(1, cyan);   // Right is Cyan
  barrelrollStrip.show();
  
  // Initialize and light up food wells, declared here since these dont change
  // 0 = Left Top (LT) = Green
  // 1 = Left Bottom (LB) = Pink
  // 2 = Right Bottom (RB) = Blue
  // 3 = Right Top (RT) = Yellow
  foodwellStrip.begin();
  foodwellStrip.setBrightness(LEDBrightness);
  foodwellStrip.setPixelColor(0, green);
  foodwellStrip.setPixelColor(1, pink);
  foodwellStrip.setPixelColor(2, blue);
  foodwellStrip.setPixelColor(3, yellow);
  foodwellStrip.show();

  Serial.begin(9600);
  if (Debug){
    Serial.println("Setup");
  }
  
  resetDevice();
  prepareTrial();
  
}

void loop()
{ 
  
  switch(runningMode){
  case 'S':
    while(digitalRead(startTrialTrigger)){
      if (Debug){
        Serial.println(" ");
        Serial.println("Triggered!");
      }
      startTrial(pickNewRandTarget());
    }
    break;
  case 'M':
    // Moroco wait for serial input

    if (Serial.available() > 0)
    {
      c = Serial.read();

//      switch (c){
//        case REWARD:
//        
//        break;
//        case MANUAL_REWARD:
//        
//        break;
//        case NEW_TRIAL:
//        
//        break;
//        case SHOW_CUE:
//        
//        break;
//        case 'R':
//        
//        break;
//      }
    }
  
    //handle reset request
    if (c == RESET)
    {
      if (Serial.read() == RESET)
      {
        if (Serial.read() == RESET)
        {
          reset = true;
        }
      }
    }
    
    break;
  case 'B':
    if (digitalRead(but)){
      spinMotor();
      if (Debug){
        Serial.println("Pushed the button");
      }
    }
    break;
  }
  
// Reset the motor angle immediately, as soon as it reaches 0 or 180
/*  
  if((angle == 0) || (angle == 180)){
    resetDevice();
  }
*/
  if (Debug){
    //Serial.print("In Loop");
  }
  
}

void rotateBarrel(int currTarget) {

  if (Debug){
    Serial.println("Ready to Rotate Barrel");
  }
  cueStrip.setPixelColor(0, green);
  cueStrip.show();
  digitalWrite(boardLED, HIGH);
  keepRunning = true; // This is overwritten by an interrupt
 // Keep allowing the barrel to move until the food well is accessed
  while(keepRunning){
    int potAngle = map(analogRead(potPin), 0, 1023, 0, 180); // Determine current motor position
    int motorDir = 1; // set default motor direction to increasing
    // Read from the encoder and determine which way it is turning
    // then write that angle into a motor, as long as it is between
    // 0 and 180 degrees.
    int aVal = digitalRead(encoder0PinA); // Read from Encoder Pin A
     if (aVal != PastA) { // Means the knob is rotating
      if (digitalRead(encoder0PinB) != aVal) { // Means pin A Changed first - We're Rotating Clockwise
        encoder0Pos--;
        if ((potAngle > 1) && (potAngle < 179)) {
          motorDir = 1*rotationDir;
        }
      } else {// Otherwise B changed first and we're moving CCW
        encoder0Pos++;
        if ((angle > 1) && (angle < 179)) {
          motorDir = -1*rotationDir;
        }
        if ((potAngle >= 179) || (potAngle <= 1)) {
          // HANDLE THE SITUATION IF ANGLE REACHES THE LIMIT!!!!!!!!!!!!!!<-------------------------------------------------------
          if (trialType == "offset"){ 
            angle = 90 + offsetAmount;
          } else {
            angle = 90; 
          }
          //writeAngle(angle); // DON'T DO ANYTHING
        }
      }
      //int newAngle = angle - oldAngle;
      int moveMotor = motorDir*stepsPerRev*rotationSpeed;
      myStepper.step(moveMotor);
      
      delay(5);  
       
     // DEBUG: Serial output of sensors and actuators
      if (Debug){
         Serial.print ("Motor Angle: ");
         Serial.print (potAngle);
         Serial.print (", Motor: ");
         Serial.println (moveMotor);
      }
    }
  }
}

int pickNewRandTarget(){
    if (Debug){
      Serial.println("Picking a new Target");
    }
    int a[4] = {sensorRT,sensorRB,sensorLT,sensorLB};
    // Pick current trial target
    int currTarget = a[rand() % 4];

    return currTarget;
}

void showCue(int targetID){
 if (Debug){
  Serial.println("Showing Cue");
 }
  // 0 = Left Top (LT) = Green
  // 1 = Left Bottom (LB) = Pink
  // 2 = Right Bottom (RB) = Blue
  // 3 = Right Top (RT) = Yellow
  switch(targetID) {
    case sensorRT : 
      cueStrip.setPixelColor(1, cyan);
      cueStrip.setPixelColor(2, yellow);
      break;      
    case sensorRB : 
      cueStrip.setPixelColor(1, cyan);
      cueStrip.setPixelColor(2, blue);
        break;
    case sensorLT : 
      cueStrip.setPixelColor(1, orange);
      cueStrip.setPixelColor(2, green);
        break;     
    case sensorLB : 
      cueStrip.setPixelColor(1, orange);
      cueStrip.setPixelColor(2, pink);
        break;
  }
  cueStrip.setPixelColor(0, red);
  cueStrip.show();
  
  if (trialType == "memory"){    
    delay(cueDisplayTime);
    cueStrip.setPixelColor(2, off);
    cueStrip.show();
  }
}

void prepareTrial() {

  // Begin Cue Strip ("start" mode)
  cueStrip.setPixelColor(0, red);
  cueStrip.setPixelColor(1, off);
  cueStrip.setPixelColor(2, off);
  cueStrip.show();
  if(Debug){
    Serial.println ("Device Ready");
  }
  delay(ITI);
  
  //digitalRead(startTrialTrigger)
}

void startTrial(int currTarget){
  depositReward(currTarget, numTreatstoDispense);
  showCue(currTarget);
  rotateBarrel(currTarget);
}

void resetDevice() {

  // Turn off Cue Strip ("off" mode)
  cueStrip.setPixelColor(0, off);
  cueStrip.setPixelColor(1, off);
  cueStrip.setPixelColor(2, off);
  cueStrip.show();

  if(Debug){
    // Reset servo position
    Serial.println ("Resetting Device...");
  }
  //delay(1000);
  if (trialType == "offset"){ 
    angle = 90 + offsetAmount;
  } else {
    angle = 90; 
  }
  writeAngle(angle);
  delay(1000);
  prepareTrial();
}

void depositReward(int targetNumber, int numSteps){
// targetNumber is which dispenser/target combo was chosen to receive reward. 
// 0 = Left Top (LT) = Green
// 1 = Left Bottom (LB) = Pink
// 2 = Right Bottom (RB) = Blue
// 3 = Right Top (RT) = Yellow
// numSteps is how many treats should be delivered (less than 1 has a probability of delivering treats. 0.25 is enough not to deliver a treat.

  writeAngle(angle); // Initalize main barrel motor to original position to drop reward in the correct position

  int totalSteps = numSteps * stepFactor;

  for(int x= 1; x<totalSteps; x++)  //Loop the forward stepping enough times for motion to be visible
  {
//    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
//    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  
}

void spinMotor() {
  while(digitalRead(but)){
    if (Debug){
      Serial.println("Button still pressed");
    }
    myStepper.step(stepsPerRev);
  }
}

void writeAngle(int setAngle){
  myStepper.setSpeed(stepperSpeed/30);
  int potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
  if(Debug){
    Serial.print("Motor position: ");
    Serial.println(potAngle);
  }
  while((setAngle - angleRange >= potAngle) || (potAngle >= setAngle + angleRange)){
    while((setAngle - angleRange > potAngle) && (potAngle > 0) && (potAngle < 180)){
       myStepper.step(stepsPerRev);
       potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
       if(Debug){
        cueStrip.setPixelColor(0, blue);
        cueStrip.show();
        Serial.print("Motor increasing position: ");
        Serial.println(potAngle);
       }
    }
    while((setAngle + angleRange < potAngle) && (potAngle > 0) && (potAngle < 180)){
      myStepper.step(-stepsPerRev);
      potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
      if(Debug){
        cueStrip.setPixelColor(0, green);
        cueStrip.show();
        Serial.print("Motor decreasing position: ");
        Serial.println(potAngle);
       }
    }
    if(potAngle >= 180){
      myStepper.step(-stepsPerRev);
    }
    if(potAngle <= 0){
      myStepper.step(stepsPerRev);
    }
    if(Debug){
      Serial.print("I'm stuck here: ");
      Serial.println(potAngle);
    }
    potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
  }
  myStepper.setSpeed(stepperSpeed);
}

//Reset Motor Driver pins to default states
void resetMotorPins(){
  digitalWrite(pulPin, LOW);
  digitalWrite(enblPin, LOW);
  digitalWrite(dirPin, LOW);

  if (Debug){
    Serial.println ("Initializing Motor Driver");
  }
  delay(300);
  digitalWrite(enblPin, HIGH);
  delay(100);
  digitalWrite(enblPin, LOW);
  
}

void fingerInside(char fingerState){
  switch(fingerState){
    case 'I':
      if (Debug){
        Serial.println ("Finger still inside. Motor detached, while loop.");
      }
      while(!digitalRead(interruptPin) || !digitalRead(int2Pin)){
        digitalWrite(enblPin, 1); //Disable motor driver
        cueStrip.setPixelColor(0, red);
        cueStrip.setPixelColor(1, off);
        cueStrip.setPixelColor(2, off);
        cueStrip.show();
      }
      break;
    case 'O':
      if (Debug){
        Serial.println ("Finger removed. Motor reattached.");
      }
      delay(timeToWaitAfterTrigger);
      digitalWrite(enblPin, 0); // Enable motor driver
      resetDevice();
      if (Debug){
        Serial.println ("Waiting for ready signal.");
      }
      break;
  }
}

void sensorInterrupt() {
  // DEBUG: Serial output if IR sensor tripped
  if (Debug){
    Serial.println ("IR sensor tripped!");
  }
  digitalWrite(enblPin, 1); //Disable motor driver
  cueStrip.setPixelColor(0, red);
  cueStrip.show();
  keepRunning = false;
  digitalWrite(boardLED, LOW);
  //int sensorValue = analogRead(interruptPin);
  if(!digitalRead(interruptPin) || !digitalRead(int2Pin)){
    fingerInside('I');
  }else{    
    fingerInside('O');
  }
}

void dirInterrupt() {

  if(digitalRead(hardDirPin)){
    rotationDir = 1;
  }else{    
    rotationDir = -1;
  }
  
}
