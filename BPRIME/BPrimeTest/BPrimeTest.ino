#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <RotaryEncoder.h>
#include <Stepper.h>
#include "BPrimeTest.h"
#include <SPI.h>

void setup()
{
 
  // Setup Stepper 
//  myStepper.setSpeed(stepperSpeed);
  treatStepper.setSpeed(treatStepperSpeed);
//  myServo.attach(servoControl);
  
  digitalWrite(treatEnable, 1); //Disable treat motor driver
  pinMode(boardLED, OUTPUT);
  pinMode(pulPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enblPin, OUTPUT);
  //resetMotorPins(); // Initialize Motor Driver pins

  // Set IR sensor pins as input
//  pinMode(sensorRT, INPUT);
//  pinMode(sensorRB, INPUT);
//  pinMode(sensorLT, INPUT);
//  pinMode(sensorLB, INPUT);
  
  // Set IR inputs (all go into same pin) as an interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(int2Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), sensorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(int2Pin), sensorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hardDirPin), dirInterrupt, CHANGE);

  // Attach interrupts to the encoder pins
  //attachInterrupt(digitalPinToInterrupt(encoder0PinA), encInt, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoder0PinB), encInt, CHANGE);
  int pos = encoder.getPosition(); // initalize encoder position

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

  pinMode(CS, OUTPUT);
  SPI.begin();
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

  digitalWrite(enblPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), encInt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), encInt, CHANGE);
  
  keepRunning = true; // This is overwritten by an interrupt
 // Keep allowing the barrel to move until the food well is accessed
  while(keepRunning){
    int potAngle = map(analogRead(potPin), 0, 1023, 0, 180); // Determine current motor position
    int motorDir = 1; // set default motor direction to increasing

    // Set default position as 0
    //static int pos = 0;
    static int motorFlag = 0;
    //int moveMotor = 0;
    long Time1 = 0;
    long Time2 = 0;
    long Time3 = 0;

    // Variable initialization
    Time1 = millis();
    Time3 = Time1 -= Time2;

    if (Time3 >= encRes)
    {
      Time2 = millis ();
      int PulsSpeed = Encoder_Count;
      Encoder_Count = 0;
      
      speednum = map(constrain(PulsSpeed,0,encTimeUpperLim), 0, encTimeUpperLim, 0, upperSpeedLim);
  
      smoothedVal = smooth(speednum, smoothedVal);

      if(digitalRead(dirPin)){
        potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
        if(potAngle > 0){
          motorFlag = 1;
        } else if(potAngle <= 0){
          motorFlag = 0;
        }
      }else if(!digitalRead(dirPin)){
        potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
        if(potAngle < 180){
          motorFlag = 1;
        } else if(potAngle >= 180){
          motorFlag = 0;
        }
      }
      
      if(motorFlag == 1){
          //if(smoothedVal <= minMoveStateLimit){
          //  moveMotor(speednum);
            //Serial.println("instant mode"); 
          //}
          //else {
            moveMotor(smoothedVal);
            //Serial.println("smoothed mode");
          //}
      }
    }

    /* OLD STEPPER MOTOR CONTROL VIA POSITION COMMAND
    // Read current encoder position
    int newPos = encoder.getPosition();
    if (pos != newPos) {
      if(newPos < pos){ //If new position is lower than original position, turn one direction
        potAngle = map(analogRead(potPin), 0, 1023, 0, 180); // Determine current motor position
        if(potAngle > 1){
          motorDir = -1*rotationDir;
          moveMotor = motorDir*stepsPerRev;
          myStepper.step(moveMotor);
          myServo.write(myServo.read() + motorDir*servoAngleStep);
        }
        if (Debug){
          cueStrip.setPixelColor(2, red);
          cueStrip.show();
        }
      }
      if(newPos > pos){ //If new position is higher than original position, turn the other direction
        potAngle = map(analogRead(potPin), 0, 1023, 0, 180); // Determine current motor position
        if(potAngle < 179){
          motorDir = 1*rotationDir;
          moveMotor = motorDir*stepsPerRev;
          myStepper.step(moveMotor);
          myServo.write(myServo.read() + motorDir*servoAngleStep);
        }
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
    */
  
      delay(5);  
       
     // DEBUG: Serial output of sensors and actuators
      if (Debug){
         Serial.print ("Motor Angle: ");
         Serial.print (potAngle);
         Serial.print (", Motor: ");
         Serial.println (speednum);
      }
      speednum = 0;
  }
  //digitalWrite(enblPin, LOW);
  detachInterrupt(digitalPinToInterrupt(encoder0PinA));
  detachInterrupt(digitalPinToInterrupt(encoder0PinB));
}

int pickNewRandTarget(){
    if (Debug){
      Serial.println("Picking a new Target");
    }
    int a[4] = {0,1,2,3}; // <--------------------------- HARD CODED NUMBERS IN AN ARRAY...
    // Pick current trial target
    int currTarget = a[rand() % 4];

    return currTarget;
}

void showCue(int targetID){ // <------------------------- Hard coded numbers in the case
 if (Debug){
  Serial.println("Showing Cue");
 }
  // 0 = Right Bottom (RB) = Blue
  // 1 = Right Top (RT) = Yellow
  // 2 = Left Top (LT) = Green
  // 3 = Left Bottom (LB) = Pink
  switch(targetID) {
    case 0 : 
      cueStrip.setPixelColor(1, cyan);
      cueStrip.setPixelColor(2, yellow);
      break;      
    case 1 : 
      cueStrip.setPixelColor(1, cyan);
      cueStrip.setPixelColor(2, blue);
        break;
    case 2 : 
      cueStrip.setPixelColor(1, orange);
      cueStrip.setPixelColor(2, green);
        break;     
    case 3 : 
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

  //magnetTestFlag = 0; // Prepare to flip the flag once the magnet in the correct position is detected
  
  writeAngle(arrayPos[targetNumber]); // Turn main barrel motor to position of "targetNumber" reward position from the arrayPos array to align for treat deposition

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
  if (trialType == "offset"){ 
    angle = 90 + offsetAmount;
  } else {
    angle = 90; 
  }
  writeAngle(angle);
}

void spinMotor() {
  while(digitalRead(but)){
    if (Debug){
      Serial.println("Button still pressed");
    }
    myStepper.step(stepsPerRev);
  }
}

// MOVE MOTOR BY A SPECIFIC SPEED (SPEED COMMAND)
// Output analog speed command via voltage over a digital potentiometer
int moveMotor(int value)
{
  digitalWrite(CS, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

// PLACE HOLDER FOR TESTING WRITING SPEED TO A MOTOR
void writeAngle(int setAngle){
  Serial.println("Hi, we're in writeAngle");
}

// MOVE MOTOR TO A SPECIFIC POT ANGLE (AT DEFAULT SPEED)
void writeAngle_BAK(int setAngle){
  digitalWrite(enblPin, HIGH);
  //myServo.write(setAngle);
  //myStepper.setSpeed(stepperSpeed);
  int potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
  if(Debug){
    Serial.print("Motor position: ");
    Serial.println(potAngle);
  }
  while((setAngle - angleRange >= potAngle) || (potAngle >= setAngle + angleRange)){
    potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
    delay(500);
    while((setAngle - angleRange >= potAngle) && (potAngle > 0) && (potAngle < 180)){ // If set angle is Higher than Pot Angle, move motor LOWER
       //myStepper.step(stepsPerRev); // MOVE THE ACTUAL MOTOR HERE (POSITIVE)
       digitalWrite(dirPin, LOW);
       moveMotor(defMotorMoveSpeed);
       potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
       if(Debug){
        cueStrip.setPixelColor(3, blue);
        cueStrip.show();
        Serial.print("Motor increasing position: ");
        Serial.println(potAngle);
       }
    }
    while((setAngle + angleRange <= potAngle) && (potAngle > 0) && (potAngle < 180)){ // If set angle is Lower than Pot Angle, move motor HIGHER
      //myStepper.step(-stepsPerRev); // MOVE THE ACTUAL MOTOR HERE (NEGATIVE)
      digitalWrite(dirPin, HIGH);
      moveMotor(defMotorMoveSpeed);
      potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
      if(Debug){
        cueStrip.setPixelColor(3, green);
        cueStrip.show();
        Serial.print("Motor decreasing position: ");
        Serial.println(potAngle);
       }
    }
    if(potAngle >= 180){
      digitalWrite(dirPin, HIGH);
      moveMotor(defMotorMoveSpeed);
    }
    if(potAngle <= 0){
      digitalWrite(dirPin, LOW);
      moveMotor(defMotorMoveSpeed);
    }
    if(Debug){
      Serial.print("I'm stuck here: ");
      Serial.println(potAngle);
      Serial.print(digitalRead(dirPin));
    }
    potAngle = map(analogRead(potPin), 0, 1023, 0, 180);
  }
  //myStepper.setSpeed(stepperSpeed);
  //digitalWrite(enblPin, LOW);
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
  //digitalWrite(enblPin, HIGH);
  delay(100);
  digitalWrite(enblPin, LOW);
  
}

// Initialize the original position of the inner barrel. Includes some magnet testing code, which is not currently used.
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
  if (trialType == "offset"){ 
    angle = 90 + offsetAmount;
  } else {
    angle = 90; 
  }
  writeAngle(angle);

  if (Debug){
    Serial.println ("Magnetic sensor tripped!");
  }
  
}

// If an IR interrupt is triggered, whether the finger is inside or outside, it runs this function and if its inside, detach the motor, if its outside, restart the trial.
void fingerInside(char fingerState){
  switch(fingerState){
    case 'I':
      if (Debug){
        Serial.println ("Finger still inside. Motor detached, while loop.");
      }
      while(!digitalRead(interruptPin) || !digitalRead(int2Pin)){
        digitalWrite(enblPin, 0); //Disable motor driver
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
      digitalWrite(enblPin, 1); // Enable motor driver
      resetDevice();
      if (Debug){
        Serial.println ("Waiting for ready signal.");
      }
      break;
  }
}

// ISR: Interrupt to trigger manual treat dispensing (INCORPORATES DEBOUNCING INTO THE ISR, NOT USING AN EXTERNAL DEBOUNCE FUNCTION)
void treatDispense() {
   static unsigned long last_interrupt_time = 0;
   unsigned long interrupt_time = millis();
   // If interrupts come faster than 200ms, assume it's a bounce and ignore
   if (interrupt_time - last_interrupt_time > 1000) 
   {
      depositReward(3,1);
   }
   last_interrupt_time = interrupt_time;
}

// ISR: IR finger sensors tripped will trigger this interrupt which declares whether a finger was removed or inserted into a reward pocket
void sensorInterrupt() {
  // DEBUG: Serial output if IR sensor tripped
  if (Debug){
    Serial.println ("IR sensor tripped!");
  }
  digitalWrite(enblPin, 0); //Disable motor driver
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

// ISR: This routine will only be called on any signal change on encoder pins.
void encInt(){
  encoder.tick(); // just call tick() to check the state.
  Encoder_Count++;
  if (Debug){
    //Serial.println("EncInt");
  }
  int newPos = encoder.getPosition();
  if (pos != newPos) {
      if(newPos < pos){
        rotationDir = -1;
        digitalWrite(dirPin, LOW);
      }
      else if(newPos > pos){
        rotationDir = 1;
        digitalWrite(dirPin, HIGH);
      }
  }
  pos = newPos;
}

// ISR: If the magnet is tripped, determine the potentiometer position when it was tripped <----------------------- CURRENTLY NOT USED
void magTripped(){
  magPotPosit = map(analogRead(potPin), 0, 1023, 0, 180);
  magnetTestFlag = 1;
  if (Debug){
    Serial.println ("Magnetic sensor tripped!");
  }
}

// ISR: Read the directional interrupt pin and assign a rotation direction based on the digital input pin
void dirInterrupt() {

  if(digitalRead(hardDirPin)){
    rotationDir = 1;
  }else{    
    rotationDir = -1;
  }
 
}

// Smoothing function to take a value and the previous smoothed value and then find a smoothed new value based on the strength of the variable "smoothStrength"
float smooth(int t_rawVal, float t_smoothedVal) {
    return  t_smoothedVal + ((t_rawVal - t_smoothedVal) + 0.5) / smoothStrength;  // +0.5 for rounding
}

// Debouncing for digital inputs <------------------- CURRENTLY NOT USED
bool debounce(int buttonName)
{
  byte count = 0;
  for(byte l = 0; l < 5; l++) {
    if (digitalRead(buttonName) == 0)
      count++;
    delay(10);
  }
  if(count > 2)  return 1;
  else           return 0;
}
