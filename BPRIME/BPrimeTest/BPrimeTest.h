//---------------------------------- Experimental Parameters

// DEBUG FLAG, set to 1 for Serial Output, 0 to mute output
bool Debug = 1;

// Running Mode:
// S: "Standalone mode" doesn't require external input
// M: "Moroco mode" Waits for Moroco input to progress through the trial
// X: "XBI mode" communicates with the XBI <---- NOT IMPLEMENTED YET! ----
char runningMode = 'M';

// Trial Types:
// "normal": cue stays on
// "memory": cue turns off after "cueDisplayTime" milliseconds
// "offset": device starts off center by "offsetAmount"
String trialType = "normal";
int offsetAmount = 30; // Set offset amount from 90 (i.e. 30 means 90 +- 30) 
int cueDisplayTime = 1000; // Memory trial type, cue show time (in ms)

int rotationSpeed = 1; // speed of barrel rotation
int ITI = 1000; // Intertrial Interval in ms (time after resetting device and starting new trial)

int timeToWaitAfterTrigger = 3000; // Wait time after the food well sensor is triggered (in ms)

//---------------------------------- Serial Communication Protocol

#define RESET '$'
#define MANUAL_REWARD 'R'
#define REWARD 'r'

//---------------------------------- Pin Assignments 

const byte interruptPin = 2;

// Encoder and servo Pins
int encoder0PinB = 3;
int encoder0PinA = 7;

int foodwellLEDsPIN = 4; //Foodwell LEDs
int cueLEDsPIN = 5; //Signal light cue LEDs
int servoPin = 6;

// Need to remain constants for the rest of the code (switch statement)
const int sensorLB = 8;
const int sensorLT = 9;
const int sensorRT = 10;
const int sensorRB = 11;

int boardLED = 13;
int barrelrollLEDsPIN = 14; //Front face Left and Right indicators

int startTrialTrigger = 15; //Input to trigger start of a trial

const int stp = 23;
const int dir = 22;
const int MS1 = 21;
const int MS2 = 20;
const int AENABLE = 19;
const int debug = 18;

//---------------------------------- Initialization

// Initialize Servo
int startAngle = 90;
Servo servo_0;
int angle = startAngle;   // servo position in degrees 

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
