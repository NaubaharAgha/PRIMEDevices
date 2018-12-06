#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Encoder.h>
#include "BPrimeTest.h"

void setup()
{
  // Set Encoder pins as input
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor

  // Initial value of encoder pins
  PastA = (boolean)digitalRead(encoder0PinA); //initial value of channel A;
  PastB = (boolean)digitalRead(encoder0PinB); //and channel B
  
  // Attach servo, initialize LED pin, and LCD outputs
  servo_0.attach(servoPin);
  servo_0.write(angle);  // Initialize to front center angle
  delay(1000);
  pinMode(boardLED, OUTPUT);
  resetEDPins(); //Set step, direction, microstep and enable pins to default states
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(AENABLE, OUTPUT);

  // Set IR sensor pins as input
  pinMode(sensorRT, INPUT);
  pinMode(sensorRB, INPUT);
  pinMode(sensorLT, INPUT);
  pinMode(sensorLB, INPUT);
  
  // Set IR inputs (all go into same pin) as an interrupt
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), sensorInterrupt, FALLING);

  pinMode(startTrialTrigger, INPUT_PULLDOWN);
  //digitalRead(startTrialTrigger);

  //Set input button to move motor manually to a pulldown input
  pinMode(but, INPUT_PULLDOWN);
  
  // Initialize Cue Light Strip
  cueStrip.begin();
  cueStrip.show(); // Initialize all pixels to 'off'
  cueStrip.setBrightness(255);

  // Initialize Cue Light Strip
  barrelrollStrip.begin();
  barrelrollStrip.setBrightness(255);
  barrelrollStrip.setPixelColor(0, orange); // Left is Orange
  barrelrollStrip.setPixelColor(1, cyan);   // Right is Cyan
  barrelrollStrip.show();
  
  // Initialize and light up food wells, declared here since these dont change
  // 0 = Left Top (LT) = Green
  // 1 = Left Bottom (LB) = Pink
  // 2 = Right Bottom (RB) = Blue
  // 3 = Right Top (RT) = Yellow
  foodwellStrip.begin();
  foodwellStrip.setBrightness(255);
  foodwellStrip.setPixelColor(0, green);
  foodwellStrip.setPixelColor(1, pink);
  foodwellStrip.setPixelColor(2, blue);
  foodwellStrip.setPixelColor(3, yellow);
  foodwellStrip.show();

  Serial.begin(9600);
  Serial.print("Setup");
  
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
          Serial.println("Pushed the button");
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
      Serial.print("Ready to Rotate Barrel");
      digitalWrite(boardLED, HIGH);
   }

   keepRunning = true;
 // Keep allowing the barrel to move until the food well is accessed
 while(keepRunning){

  // Read from the encoder and determine which way it is turning
  // then write that angle into servo, as long as it is between
  // 0 and 180 degrees.
  int aVal = digitalRead(encoder0PinA); // Read from Encoder Pin A
   if (aVal != PastA) { // Means the knob is rotating
    if (digitalRead(encoder0PinB) != aVal) { // Means pin A Changed first - We're Rotating Clockwise
        encoder0Pos--;
        if ((angle > 0) && (angle < 180)) {
          angle += rotationSpeed;
        }
      } else {// Otherwise B changed first and we're moving CCW
        encoder0Pos++;
        if ((angle > 0) && (angle < 180)) {
          angle -= rotationSpeed;
        }
        if ((angle == 180) || (angle == 0)) {
          angle = 90;
        }
      }
      servo_0.write(angle);
      delay(15);  
     
   // DEBUG: Serial output of sensors and actuators
    if (Debug){
       Serial.print ("Angle: ");
       Serial.print (angle);
       Serial.print (", Enc: ");
       Serial.print (encoder0Pos);
       Serial.print (", RT:");
       Serial.print (digitalRead(sensorRT));
       Serial.print (", RB:");
       Serial.print (digitalRead(sensorRB));
       Serial.print (", LT:");
       Serial.print (digitalRead(sensorLT));
       Serial.print (", LB:");
       Serial.println (digitalRead(sensorLB));
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
    int currTarget = a[rand() % 3];

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
  cueStrip.show();
  
  if (trialType == "memory"){    
  delay(cueDisplayTime);
    cueStrip.setPixelColor(2, off);
    cueStrip.show();
  }
  
}

void prepareTrial() {

  // Begin Cue Strip ("start" mode)
  cueStrip.setPixelColor(0, white);
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
  servo_0.write(angle);
  delay(1000);
}

void dispenseTreat(int numSteps, bool rotDirection){
// numSteps is how many treats should be delivered (less than 1 has a probability of delivering treats. 0.25 is enough not to deliver a treat.
// rotDireciton is to go forward or backwards. Toggle this boolean to switch directions.

  int totalSteps = numSteps * 200;
  if(rotDirection)
  {
    digitalWrite(dir, LOW);
  }
  else
  {
    digitalWrite(dir,HIGH);
  }

  for(int x= 1; x<totalSteps; x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
  
}

void spinMotor() {
  Serial.println("Stepping at 1/8th microstep mode.");
  //digitalWrite(dir, LOW); //Pull direction pin low to move "forward" // ALREADY SET EARLIER 
  digitalWrite(MS1, HIGH); //Pull MS1, and MS2 high to set logic to 1/8th microstep resolution
  digitalWrite(MS2, HIGH);

  dispenseTreat(3,true);
}

//Reset Easy Driver pins to default states
void resetEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(AENABLE, HIGH);
}


void sensorInterrupt() {
     // DEBUG: Serial output if IR sensor tripped
  if (Debug){
    Serial.println ("IR sensor tripped!");
    digitalWrite(boardLED, LOW);
  }
  delay(timeToWaitAfterTrigger);
  keepRunning = false;
  servo_0.write(startAngle);
  resetDevice();
  if (Debug){
    Serial.println ("Waiting for ready signal.");
  }
}
