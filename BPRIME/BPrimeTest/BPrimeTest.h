//---------------------------------- Experimental Parameters

// DEBUG FLAG, set to 1 for Serial Output, 0 to mute output
bool Debug = 1;

// RUNNING MODE:
// S: "Standalone mode" doesn't require external input
// M: "Moroco mode" Waits for Moroco input to progress through the trial
// X: "XBI mode" communicates with the XBI <---- NOT IMPLEMENTED YET! ----
char runningMode = 'S';

// TRIAL TYPES:
// "normal": cue stays on
// "memory": cue turns off after "cueDisplayTime" milliseconds
// "offset": device starts off center by "offsetAmount"
String trialType = "normal";
int offsetAmount = 30; // Set offset amount from 90 (i.e. 30 means 90 +- 30) 
int cueDisplayTime = 1000; // Memory trial type, cue show time (in ms)

// BARREL MOTOR SECTION
int stepsPerRev = 20; // steps per revolution (set by switches on the driver DM542)
float motorResolution = stepsPerRev/360; // Determine Motor resolution
int stepperSpeed = 550; // stepper speed in RPM

int angleRange = 5; // Degrees of allowance for chosen angle (i.e. 90 becomes +/-5 so between 85 to 95)

int ITI = 1000; // Intertrial Interval in ms (time after resetting device and starting new trial)

int timeToWaitAfterTrigger = 3000; // Wait time after the finger sensor is triggered (in ms)

// TREAT MOTOR SECTION
int numTreatstoDispense = 1; // Number of treats to dispense per dispense request (whole number)
int stepFactor = 40*stepsPerRev; // Empirically deterimined number of motor steps to take to dispense a single treat
//int stepFactor = 1200;
int treatStepperSpeed = 2*stepperSpeed;

int rotationDir = 1; // Direction to rotate 1 or -1

int LEDBrightness = 64; // Brightness range from 0 - 255

int treatCounter = 0; // Keep track of how many treats have been dispensed (can be sent out at some point...)
int intToSwitch = 5; // Interval to switch direction on the motor (after ever x treats, turn the opposite direction once)

int magnetTestFlag = 0; // Flag initialized at 0, goes high whenever the magnetic sensor is tripped
int magPotPosit = 0; // Store the pot position every time the magnetic sensor is tripped

// Array of positions (a bit confusing, but as follows:)
// The order from the top down anticlockwise: 5, 0, 1, 2, 3, 4
// 0-3: the positions of the treat wells:
  // 0 = Right Bottom (RB) = Blue
  // 1 = Right Top (RT) = Yellow
  // 2 = Left Top (LT) = Green
  // 3 = Left Bottom (LB) = Pink
// 4: position of the LEFT limit (180 deg)
// 5: position of the RIGHT limit (0 deg)
// Detailed Explanation: When the LEFT treat wells are facing the user, the RIGHT treat wells are directly in the back (0 degrees) [position 5]
// As the user moves toward the center, the right treat wells move towards the right side of the user (the RIGHT treat wells move clockwise when looking top down).
// The first treat dispenser (RB) comes up first [position 0]. Second treat dispenser (RT) comes up next  [position 1], and so on...
// Finally, all treat dispensers pass and the RIGHT treat wells are facing the user, which means the LEFT treat wells are directly in the back (180 degrees) [position 4].
int arrayPos [6] = { 45, 71, 104, 128, 180, 0 };  

//---------------------------------- Serial Communication Protocol

#define RESET '$'
#define MANUAL_REWARD 'R'
#define REWARD 'r'

//---------------------------------- Pin Assignments

int unused10 = 24;
int unused9 = 25;
int unused8 = 26;
int unused7 = 27;
int unused6 = 28;
int unused5 = 29;
int unused4 = 30;
int unused3 = 31;
int unused2 = 32;
int unused1 = 33;

int hardDirPin = 0; // Switch to manually switch the rotation direction of the barrel; Default = 0;

int but = 1; // Button for treat deposition position initialization (hold during startup to begin with initialization phase)

const byte interruptPin = 2; // Interrupt for IR Sensors

int barrelrollLEDsPIN = 3; //Front face Left and Right indicators
int foodwellLEDsPIN = 4; //Foodwell LEDs
int cueLEDsPIN = 5; //Signal light cue LEDs

int startTrialTrigger = 6; // Input to trigger start of a trial

int treatBut = 7; // Button to manually dispense a treat

// Need to remain constants for the rest of the code (switch statement)
const int sensorLB = 8;
const int sensorLT = 9;
const int sensorRT = 10;
const int sensorRB = 11;

int magSensor = 12;

int boardLED = 13;

// Feeder Stepper motor pins
int treatDir = 14; //Treat motor direction
int treatPulse = 15; //Feeder pulse
int treatEnable = 16; // Treat motor enable

// Encoder Interrupt Pins (Need to be Analog... I think?)
int encoder0PinA = 17;
int encoder0PinB = 18;

// Main Barrel Motor pins
const int enblPin = 19;
const int pulPin = 20;
const int dirPin = 21;

const int int2Pin = 22; //Second IR interrupt pin

const int potPin = 23; //Pot to measure rotation of Main Barrel Motor

//---------------------------------- Initialization

// Initialize Stepper
int startAngle = 90;
int angle = startAngle;   // servo position in degrees 

Stepper myStepper(stepsPerRev, pulPin, dirPin); // MAIN INNER BARREL STEPPER MOTOR

Stepper treatStepper(stepsPerRev, treatPulse, treatDir); //SECONDARY TREAT DISPENSER STEPPER MOTOR

// Initialize Encoder
RotaryEncoder encoder(encoder0PinA, encoder0PinB);

// Initialize LED strips
Adafruit_NeoPixel cueStrip = Adafruit_NeoPixel(3, cueLEDsPIN);
Adafruit_NeoPixel foodwellStrip = Adafruit_NeoPixel(4, foodwellLEDsPIN);
Adafruit_NeoPixel barrelrollStrip = Adafruit_NeoPixel(2, barrelrollLEDsPIN);

byte c;
boolean reset;

//---------------------------------- Variables

// Placeholder variables for the encoder counting
volatile unsigned int encoder0Pos = 65536/2; // Start encoder count from the middle
volatile boolean PastA = 0;
volatile boolean PastB = 0;

bool keepRunning = false; // Boolean if the sensor -> motor loop should keep running or stop (this is set in the motor loop)

// Colors declarations
uint32_t off = cueStrip.Color(0, 0, 0);
uint32_t white = cueStrip.Color(255, 255, 255);
uint32_t red = cueStrip.Color(255, 0, 0);
uint32_t green = cueStrip.Color(0, 255, 0);
uint32_t blue = cueStrip.Color(0, 0, 255);
uint32_t yellow = cueStrip.Color(255, 255, 0);
uint32_t pink = cueStrip.Color(255, 0, 255);
uint32_t orange = cueStrip.Color(255, 128, 0);
uint32_t cyan = cueStrip.Color(0, 255, 255);
