//#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <RotaryEncoder.h>
#include <Stepper.h>
#include "BPrimeTest.h"

void setup()
{
 
  // Setup Stepper 
  myStepper.setSpeed(stepperSpeed);
  treatStepper.setSpeed(stepperSpeed);
  digitalWrite(treatEnable, 1); //Disable treat motor driver
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

  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), encInt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), encInt, CHANGE);

  // Treat Button ALWAYS deposits a treat <------------------------------------HARD CODED INTO POSITION 0... CHANGE THIS!
  pinMode(treatBut, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(treatBut), treatDispense, RISING);

  // Magnetic Sensor always records pot position when it is tripped 
  //pinMode(magSensor, INPUT_PULLDOWN);
  //attachInterrupt(digitalPinToInterrupt(magSensor), magTripped, RISING);
  
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
  // 0 = Right Bottom (RB) = Blue
  // 1 = Right Top (RT) = Yellow
  // 2 = Left Top (LT) = Green
  // 3 = Left Bottom (LB) = Pink
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

  // If the pin on "but" is HIGH, then begin with the initialization phase and remember all array positions
  // If not, you are screwed anyway...
  // TODO: Add default array positions...
  //if(digitalRead(but)){
  //  initializePositions();
  //}
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
// TODO: Add case to deal with flipping the direction switch. What happens when increasing the encoder decreases the motor angle? (check hardcoded conditionals...)

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

    // Set default position as 0
    static int pos = 0;
    static int motorFlag = 0;
    int moveMotor = 0;
    
    // Read current encoder position
    int newPos = encoder.getPosition();
    if (pos != newPos) {
      if(newPos < pos){ //If new position is lower than original position, turn one direction
        if(potAngle > 1){
          motorFlag = 1;
        }
        motorDir = -1*rotationDir;
        if (Debug){
          cueStrip.setPixelColor(2, red);
          cueStrip.show();
        }
      }
      if(newPos > pos){ //If new position is higher than original position, turn the other direction
        if(potAngle < 179){
          motorFlag = 1;
        }
        motorDir = 1*rotationDir;
        if (Debug){
          cueStrip.setPixelColor(2, pink);
          cueStrip.show();
        }
      }
      pos = newPos;
      if (Debug){
        Serial.println(pos);
      }
    }else{
      motorFlag = 0;
    }
  
    potAngle = map(analogRead(potPin), 0, 1023, 0, 180);

      if(motorFlag){
        //moveMotor = motorDir*stepsPerRev*rotationSpeed;
        myStepper.setSpeed(stepperSpeed);
        moveMotor = motorDir*stepsPerRev;
        myStepper.step(moveMotor);
      }
      
      potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
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
  // 0 = Right Bottom (RB) = Blue
  // 1 = Right Top (RT) = Yellow
  // 2 = Left Top (LT) = Green
  // 3 = Left Bottom (LB) = Pink
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
  //depositReward(currTarget, numTreatstoDispense);
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
  if (Debug){
      Serial.println("Treat Requested");
  }
// targetNumber is which dispenser/target combo was chosen to receive reward. 
  // 0 = Right Bottom (RB) = Blue
  // 1 = Right Top (RT) = Yellow
  // 2 = Left Top (LT) = Green
  // 3 = Left Bottom (LB) = Pink
// numSteps is how many treats should be delivered (less than 1 has a probability of delivering treats. 0.25 is enough not to deliver a treat.

  magnetTestFlag = 0; // Prepare to flip the flag once the magnet in the correct position is detected
  
  //writeAngle(arrayPos[targetNumber]); // Turn main barrel motor to position of "targetNumber" reward position from the arrayPos array to align for treat deposition

//  if(magnetTestFlag){  
    digitalWrite(treatEnable, 0); //Enable treat motor driver
//  }

  if(treatCounter%intToSwitch){
    treatStepper.step(stepFactor/4*numSteps); // Step backwards a little bit to "shake up the dust/treats" a bit
    treatStepper.step((-stepFactor*(4/3))*numSteps); // stepFactor is empirically determined to be the stepsize required to deposit treats
    if (Debug){
        Serial.println("Treat Dispensed Backwards");
    }
  }else{
    treatStepper.step(-stepFactor/4*numSteps); // Step backwards a little bit to "shake up the dust/treats" a bit
    treatStepper.step((stepFactor*(4/3))*numSteps); // stepFactor is empirically determined to be the stepsize required to deposit treats
    if (Debug){
        Serial.println("Treat Dispensed");
    }
  }

  digitalWrite(treatEnable, 1); //Disable treat motor driver
  treatCounter++;
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
  myStepper.setSpeed(stepperSpeed);
  int potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
  if(Debug){
    Serial.print("Motor position: ");
    Serial.println(potAngle);
  }
  while((setAngle - angleRange >= potAngle) || (potAngle >= setAngle + angleRange)){
    potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
    delay(1000);
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
  //myStepper.setSpeed(stepperSpeed);
}

//Reset Motor Driver pins to default states
void resetMotorPins(){
  digitalWrite(pulPin, LOW);
  digitalWrite(enblPin, LOW);
  digitalWrite(dirPin, LOW);
  digitalWrite(treatDir, LOW);
  digitalWrite(treatPulse, LOW);
  digitalWrite(treatEnable, LOW);

  if (Debug){
    Serial.println ("Initializing Motor Driver");
  }
  delay(300);
  digitalWrite(enblPin, HIGH);
  delay(100);
  digitalWrite(enblPin, LOW);
  
}

void initializePositions(){
  myStepper.setSpeed(stepperSpeed/10);
  writeAngle(0);
  delay(100);
  magnetTestFlag = 0;
  for( int i = 0; i <= 5; i++){
    while(magnetTestFlag == 0){
      myStepper.step(stepsPerRev/5);
    }
    switch (i) {
      case 0: arrayPos[5] = magPotPosit; break;
      case 5: arrayPos[4] = magPotPosit; break;
      default: arrayPos[i-1] = magPotPosit; break;
    }
    magnetTestFlag = 0;
  }
  myStepper.setSpeed(stepperSpeed);
  writeAngle(angle);

  if (Debug){
    Serial.println ("Magnetic sensor tripped!");
  }
  
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

void treatDispense() {
   static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
   // If interrupts come faster than 200ms, assume it's a bounce and ignore
   if (interrupt_time - last_interrupt_time > 1000) 
   {
      depositReward(0,1);
   }
   last_interrupt_time = interrupt_time;
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

  if(!digitalRead(interruptPin) || !digitalRead(int2Pin)){
    fingerInside('I');
  }else{    
    fingerInside('O');
  }
}

// This routine will only be called on any signal change on encoder pins: exactly where we need to check.
void encInt(){
     encoder.tick(); // just call tick() to check the state.
}

void magTripped(){
  magPotPosit = map(analogRead(potPin), 0, 1023, 0, 180);
  magnetTestFlag = 1;
  if (Debug){
    Serial.println ("Magnetic sensor tripped!");
  }
}

void dirInterrupt() {

  if(digitalRead(hardDirPin)){
    rotationDir = 1;
  }else{    
    rotationDir = -1;
  }
  
}
