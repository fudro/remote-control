/*
 * This program accepts input from a wireless game controller to control HomeBot.
 * 
 * HARDWARE:
 * Sony PS2 Wireless Force 2 Controller
 * Cytron PS2 Shield: http://www.cytron.com.my/p-shield-ps2
 * MakeBlock Mega-Pi: http://learn.makeblock.com/en/megapi/
 * 
 * CONTROL FEATURES:
 * TODO:
 * a) Mode Switching using SELECT button:                 (Press to toggle between Drive Mode and Arm Control Mode)
 * b) Drive control using LEFT and RIGHT joysticks:       (LEFT-Y-Axis = left drive speed, RIGHT-Y-Axis = right drive speed, Sonar Sensors considered FRONT of robot)
 * c) Arm control using LEFT joystick:                    (Y-Axis: Shoulder Up/Down = stick forward/back, X-Axis: Turntable Rotate CCW/CW = stick Left/Right)
 * d) Arm control using RIGHT joystick:                   (Y-Axis: Elbow Up/Down = stick forward/back, X-Axis: Wrist Rotate: Left = CCW / Right = CW, looking "down" the arm toward the wrist)
 * DONE e) Activate Tail Gripper using L Buttons:              (L1 pressed = OPEN, L2 pressed = CLOSE)
 * DONE f) Activate Arm Gripper using R Buttons:               (R1 pressed = OPEN - Automatic Fully Open, R2 pressed = CLOSE - Manual, continuous closing while pressed)
 * g) D-Pad programmatic Arm movement                     (UP = arm to front toward Sonar, DOWN = arm to rear toward Tail Gripper)
 * h) D-Pad programmatic Chassis Movement                 (LEFT = rotate 90 degrees CCW based on IMU, RIGHT = rotate 90 degrees CW based on IMU)
 * EXTRA:
 * e) Pickup Item                    (TRIANGLE Button)
 * f) Sonar Scan                     (CIRCLE Button)
 * g) Collect Item                   (SQUARE Button)
 * h) Drop Item                      (CROSS Button)
*/

#include <SoftwareSerial.h>
#include "Cytron_PS2Shield.h"
#include "MeMegaPi.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//#define DEBUG_JOYSTICK

#define IMU   //Use flag when using IMU otherwhise an error will be thrown if the sensor is not connected.
//Set up MegoPi motor ports
MeMegaPiDCMotor armGrip(PORT1A); //Arm Gripper
MeMegaPiDCMotor armWrist(PORT1B); //Wrist
MeMegaPiDCMotor driveLeft(PORT2A); //Drive Right, based on the sonar unit being the "front" of the robot
MeMegaPiDCMotor driveRight(PORT2B); //Drive Left
MeMegaPiDCMotor tailGrip(PORT3A); //Tail Gripper
MeMegaPiDCMotor turnTable(PORT3B); //Turntable
MeMegaPiDCMotor elbow(PORT4A); //Elbow
MeMegaPiDCMotor shoulder(PORT4B); //Shoulder

//Set up IMU sensor (BNO055)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
 
//Create PS2Shield object
Cytron_PS2Shield ps2; // Create object without paramaters to use the MegaPi hardware Serial2 pins 17(RX) and 16(TX)

//Direction Constants
const int OPEN = 0;
const int CLOSE = 1;
const int STOP = 2;
const int H = 0;    //HORIZONTAL
const int V = 1;    //VERTICAL
const int CW = 0; //Rotation Direction based on "personified" view of the robot. For WRIST rotation: direction is visualized as "looking down the arm" as if it were your own. For BODY and TURNTABLE rotation: direction is visualized based on a TOP-DOWN view of the robot with the front (SONAR) facing the 12 o'clock direction. 
const int CCW = 1;
const int FW = 1; //Forward
const int BW = 0; //Backward
//TODO: Add Lidar Pins
//Pin Assignments
const int IMU_RESET = 39;   //This reset functionality does not work and requires additional coding. The sensor doesn't fully recover after reset because it also needs to be re-initialized.
const int WRIST_HALL = 30;
const int WRIST_SWITCH = 28;
const int TURNTABLE_HALL = 26;
const int TURNTABLE_SWITCH = 24;
const int SONAR_RIGHT = A14;
const int SONAR_CENTER = A13;
const int SONAR_LEFT = A12;
const int DRIVE_RIGHT_ENCODER = A11;
const int DRIVE_LEFT_ENCODER = A10;
const int ELBOW_POT = A9;
const int SHOULDER_POT = A8;
const int TURNTABLE_ENCODER = A7;
const int SONAR_TRIGGER = A6;
//Time Constants
const int ARMGRIPTIME = 1200; //Time for arm gripper at max speed 255 to go from fully open to fully closed and vice versa.
const int WRISTTIME = 1000; //Wrist rotation at max speed 255 for 1000 milliseconds is approximately 180 degres of rotation (or one-half turn)
const int TAILGRIPTIME = 400;  //Time for tail gripper at HALF SPEED (128) to go from fully open to fully closed and vice versa.
//Sonar Constants
int NUM_SAMPLES = 9;    //The number of times the sonar sensor is sampled. These samples are then sorted from low to high.
int SAMPLE_OFFSET = 3;  //The number of entries from the max (highest) entry in the sorted array of samples (i.e. the 7th item in a list of 10)
//Potentiometer Limits
const int ELBOW_MAX = 700;  //Elbow UPPER limit
const int ELBOW_GRIP = 250; //Best position where the arm gripper can transfer an object to the tail gripper.
const int ELBOW_MIN = 20; //Elbow LOWER limit
const int SHOULDER_MAX = 400; //Shoulder UPPER limit. REVERSED: sensor value DECREASES when the arm is moving UP.
const int SHOULDER_MIN = 600; //Shoulder LOWER limit. REVERSED: sensor value INCREASES when the arm is moving DOWN.
//Encoder Analog Limits
//The encoders use the analog value from the opto-coupler sensor board. 
//The Min/Max constants represent the "highest" and "lowest" analog values returned from the sensor. (These values can vary by sensor board and must be determined through testing.)
//When the sensor value goes above or below these values, it corresponds to the sensor states of "on" or "off" as the encoder disc passes through the opto-coupler.
//The resulting changes in state are counted as an encoder "ticks".
const int TURNTABLE_ANALOG_MAX = 875;   
const int TURNTABLE_ANALOG_MIN = 650;
const int TURNTABLE_LIMIT = 29; //maximum number of encoder ticks for arm to travel 180 degrees
const int DRIVE_LEFT_ANALOG_MAX = 700;
const int DRIVE_LEFT_ANALOG_MIN = 400;
const int DRIVE_RIGHT_ANALOG_MAX = 700;
const int DRIVE_RIGHT_ANALOG_MIN = 400;
//State Variables
int controlMode = 2;  //state to hold current mode of control (DRIVE or ARM). Defaults to 2 ("STOP")
int selectState = 0;  //state of Select Button (zero = "not activated", 1 = "activated")
int triangleState = 0;    //remote control button states
int circleState = 0;
int crossState = 0;
int squareState = 0;
int armGripState = 1; //Default to "closed"
int tailGripState = 1;  //Default to "closed"
int wristSwitch = 0;  //state of wrist switch
int lastWristDirection = 1; //store last rotation direction (used to reorient to nearest cardinal position)
int wristOrientation = H;   //target wrist orientation (H or V), horizontal direction as default
int wristState = 0;  //state of wrist motor (-1 = CCW, 0 = NOT MOVING, 1 = CW)
int elbowState = 0; //Initialize elbow as NOT MOVING
int elbowDirection = 0; //Initialize elbow as NOT MOVING
int shoulderState = 0; //Initialize shoulder as NOT MOVING
int shoulderDirection = 0;  //Track direction of motor to facilitate proper braking direction. Initialize shoulderDirection to "zero" for NEUTRAL
int turnTableState = 0; //Initialize shoulder as NOT MOVING
int turnTableEncoder = 1; //state of turntable encoder as high(1) or low(0). Set initial state to 1 since this input pin is pulled high
int turnTableSwitch = 0;  //state of turntable switch, default state is 0 (LOW)
int turnTableHall = 1;    //state of turntable hall effect sensor, default state is HIGH, sensor pulls pin LOW when active (magnet present). IMPORTANT: Activated (LOW) when arm gripper is aligned with tail gripper
int driveLeftEncoder = 1; //last state of encoder high(1) or low(0), for comparison with current state read from sensor
int driveRightEncoder = 1;
int driveLeftState = 0;   //store direction of left drive motor
int driveRightState = 0;  //store direction of right drive motor
//Tracking variables
int turnTableCount = 1;  //store encoder ticks for current amount of turntable travel based on encoder ticks. Default to 1 for easier calculations (15 ticks ~ 90 degrees)
int turnTableTarget = 0;   //track whether the a turnTable limit has been reached. (0 = FALSE, 1 = TRUE)
int turnTablePosition = 0;  //store current turn table position based on encoder ticks and turn table rotation limits
int turnTableAnalog = 0;  //last analog value of turntable encoder
int driveLeftCount = 0;  //store encoder tick value to track movement
int driveRightCount = 0;
int driveLeftAnalog = 0;  //last analog value from drive encoder, raw analog value is compared with the Min/Max constants to set current state of encoder
int driveRightAnalog = 0;
int driveLeftDistance = 0; //track tick count of encoder
int driveRightDistance = 0;  
int tailGripCount = 0;    //track the number of iterations for pulsing motor to close tail gripper
int sort_count = 0;       //track the last ten "selected" sonar values. Sonar values are "selected" using the NUM_SAMPLES and SAMPLE_OFFSET constants.
int left_sonar[10];       //arrays to store sonar readings
int center_sonar[10];
int right_sonar[10];
int left_sort_array[10]; //arrays to hold last 10 selected values from sorted sonar reading arrays
int center_sort_array[10];
int right_sort_array[10];
int range_left = 0;    //calculated sonar range for each sensor
int range_center = 0;
int range_right = 0;
//Test Variables
int count = 0;    //test variable
int runFlag = 1;  //test variable

//The runArray[] determines which motor functions are active (0 = inactive, 1 = active).
//IMPORTANT: The defaults are overridden with new values when switching modes between DRIVE and ARM 
//Even if code is called by the main loop, the flag array will prevent code execution if motor is not set to "active".
int runArray[] = {0,  //runArray[0]: armGripper
                  0,  //runArray[1]: wrist
                  0,  //runArray[2]: elbow
                  0,  //runArray[3]: shoulder
                  0,  //runArray[4]: turntable
                  0,  //runArray[5]: tailGripper
                  0,  //runArray[6]: drive
                  0   //runArray[7]: sonar
};

//Joystick Variables
int joystick_left_Y = 0;      //left joystick position in Y axis (FORWARD and BACKWARD)
int joystick_left_X = 0;      //left joystick position in X axis (LEFT/CCW and RIGHT/CW)
int joystick_right_Y = 0;      //left joystick position in Y axis (FORWARD and BACKWARD)
int joystick_right_X = 0;      //left joystick position in X axis (LEFT/CCW and RIGHT/CW)
int joystickLeftState = 0;   //left joystick button state (0 = Pressed, 1 = Released)
int joystickRightState = 0;   //right joystick button state (0 = Pressed, 1 = Released)
int shoulder_lift_speed = 0;  //speed at which the arm "shoulder" moves up and down
int turntable_speed = 0;
int elbow_lift_speed = 0;  //speed at which the arm "shoulder" moves up and down
int wrist_turn_speed = 0;
int drive_speed_left = 0;
int drive_speed_right = 0;
//int commandState = 0;         //Track if a command is currently being executed

/***********************************
 * FUNCTION PROTOTYPES - Function prototypes are required to allow function with parameters that can have deafult values.
***********************************/
void armGripper(int gripState, int gripTime = ARMGRIPTIME); //gripState is OPEN (0) or CLOSE (1)
void armGripperManual(int gripState, int gripTime = ARMGRIPTIME); //special version of arm gripper funtion for manual remote control operation
void wristRotate (int targetState, int wristDirection = CW, float wristRevolution = 0.0, int wristSpeed = 255);  //targetState is (V)ertical or (H)orizontal, wristRevolution is the number of full revolutions to be perfomed.
void wristManual(int wristDirection, int wristSpeed = 255);  //manual version of the wristRotate function. Requires a direction (CCW = -1, CW = 1)
void elbowMoveManual(int elbowPosition = 500, int elbowSpeed = 65);  //default parameter values are approximate center position and preferred default speed.
void shoulderMoveManual(int shoulderPosition = 550, int shoulderSpeed = 131); //default parameter values are approximate center position and preferred default speed.
void turnTableMove(int turnDegrees = 0, int turnDirection = CW, int turnSpeed = 65);    //turnDegrees is the degrees of angular rotation from the current position
void turnTableManual(int commandState = 0, int turnSpeed = 65);    //special version of turnTableMove for remote control operation
void tailGripper(int gripState, int gripTime = TAILGRIPTIME); //gripState is OPEN (0) or CLOSE (1)
void driveMove(int driveDistance = 20, int driveDirection = FW, int driveSpeed = 127);


void setup()
{
  //Set up MegaPi sensor pins
  pinMode(WRIST_HALL, INPUT_PULLUP);  //wrist hall effect, set as pullup since sensor goes LOW when activated
  pinMode(WRIST_SWITCH, INPUT); //wrist switch
  pinMode(DRIVE_LEFT_ENCODER, INPUT_PULLUP);
  pinMode(DRIVE_RIGHT_ENCODER, INPUT_PULLUP);
  pinMode(ELBOW_POT, INPUT_PULLUP); //elbow potentiomter
  pinMode(SHOULDER_POT, INPUT_PULLUP); //shoulder potentiometer
  pinMode(TURNTABLE_SWITCH, INPUT); //turntable switch
  pinMode(TURNTABLE_HALL, INPUT_PULLUP); //turntable hall effect, set as pullup since sensor goes LOW when activated
  pinMode(TURNTABLE_ENCODER, INPUT_PULLUP); //turntable encoder
  pinMode(IMU_RESET, OUTPUT);
  digitalWrite(IMU_RESET, HIGH);  //default pin state is HIGH. Pull IMU reset pin LOW to trigger reset
  pinMode(SONAR_TRIGGER, OUTPUT);
  digitalWrite(SONAR_TRIGGER, LOW);   //Default sonar trigger pin to LOW. Pull HIGH to trigger sensor.
  //Set up serial communications
  Serial.begin(115200);
  Serial.println("HomeBot Remote Control Test!");
  delay(1000);
  Serial.println("StartingTest in 3 seconds...");
  delay(3000);
  #ifdef IMU
  //IMU Sensor (BNO055) This sensor uses I2C so no additional pin assignments are necessary 
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* Check if there was a problem detecting the BNO055 ... display a warning */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);   //advanced setup (I don't know what this actually does)
  #endif
  
  ps2.begin(115200);          //Start remote control shield and set baud rate (baudrate must be the same as the jumper setting at PS2 shield)
  Serial.println("Remote Control Ready!");

  Serial2.begin(115200);       //Set MegaPi serial channel for remote control communication
}

void loop(){ 
  //SPECIAL BUTTONS
  //SET MODE WITH SELECT BUTTON
  //Select Pressed
  if(ps2.readButton(PS2_SELECT) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_SELECT) == 0 && selectState == 0) { //double check button to prevent false trigger
      selectState = 1;
      Serial.println("Select Pressed!");
     
      //Check Control Mode
      if(controlMode == STOP) {  //INACTIVE MODE
        controlMode = 0;  //set to DRIVE MODE
      }
      else {
        controlMode = !controlMode;   //toggle state variable for the mode (DRIVE or ARM)
      }
      if(controlMode == 0) {  //DRIVE MODE
        Serial.print("DRIVE MODE!");
        Serial.print("\n");
        //Reset runArray to only allow the drive motor commands
        runArray[0] = 0,  //runArray[0]: armGripper
        runArray[1] = 0,  //runArray[1]: wrist
        runArray[2] = 0,  //runArray[2]: elbow
        runArray[3] = 0,  //runArray[3]: shoulder
        runArray[4] = 0,  //runArray[4]: turntable
        runArray[5] = 0,  //runArray[5]: tailGripper
        runArray[6] = 1,  //runArray[6]: drive
        runArray[7] = 0;  //runArray[7]: sonar
        ps2.vibrate(PS2_MOTOR_1,255);   //Vibrate desired motor (1 or 2) at the max speed (255)
        delay(300);
        ps2.vibrate(PS2_MOTOR_1,0);     //Stop vibraton (set speed to zero)
        delay(200);
        ps2.vibrate(PS2_MOTOR_2,255);   //Vibrate desired motor (1 or 2) at the max speed (255)
        delay(300);
        ps2.vibrate(PS2_MOTOR_2,0);     //Stop vibraton (set speed to zero)
      }
      else if(controlMode == 1) { //ARM MODE
        Serial.print("ARM MODE!");
        Serial.print("\n");
        Serial.println("Remember to reset arm position (L3)!");
        //Reset runArray to only allow the arm and gripper motor commands
        runArray[0] = 1,  //runArray[0]: armGripper
        runArray[1] = 1,  //runArray[1]: wrist
        runArray[2] = 1,  //runArray[2]: elbow
        runArray[3] = 1,  //runArray[3]: shoulder
        runArray[4] = 1,  //runArray[4]: turntable
        runArray[5] = 1,  //runArray[5]: tailGripper
        runArray[6] = 0,  //runArray[6]: drive
        runArray[7] = 0;  //runArray[7]: sonar
        ps2.vibrate(PS2_MOTOR_1,255);   //Vibrate desired motor (1 or 2) at the max speed (255)
        delay(300);
        ps2.vibrate(PS2_MOTOR_1,0);
      }
    }
  }
  //Select Released
  else if (ps2.readButton(PS2_SELECT) == 1 && selectState == 1)
  {
    selectState = 0;
    Serial.println("Select Released!");
    ps2.vibrate(PS2_MOTOR_1,0);   //Stop all vibraton (set speed to zero)
    ps2.vibrate(PS2_MOTOR_2,0);
  }


  //CHECK CONTROL MODE:
  //DRIVE MODE
  if(controlMode == 0) {
    //Buttons
    //Left Joystick Press (L3)
    if(ps2.readButton(PS2_JOYSTICK_LEFT) == 0) // 0 = pressed, 1 = released
    {
      delay(10);
      if(ps2.readButton(PS2_JOYSTICK_LEFT) == 0 && joystickLeftState == 0) { //double check button to prevent false trigger
        joystickLeftState = 1;
        driveLeftDistance = 0;
        Serial.println("Left Joystick Pressed!");
        Serial.println("Left Encoder Reset!");
      }
    }
    else if (ps2.readButton(PS2_JOYSTICK_LEFT) == 1 && joystickLeftState == 1)
    {
      joystickLeftState = 0;
      Serial.println("Left Joystick Released!");
    }
  
    //Right Joystick Press (R3)
    if(ps2.readButton(PS2_JOYSTICK_RIGHT) == 0) // 0 = pressed, 1 = released
    {
      delay(10);
      if(ps2.readButton(PS2_JOYSTICK_RIGHT) == 0 && joystickRightState == 0) { //double check button to prevent false trigger
        joystickRightState = 1;
        driveRightDistance = 0;
        Serial.println("Right Joystick Pressed!");
        Serial.println("Right Encoder Reset!");
      }
    }
    else if (ps2.readButton(PS2_JOYSTICK_RIGHT) == 1 && joystickRightState == 1)
    {
      joystickRightState = 0;
      Serial.println("Right Joystick Released!");
    }
    
    //DRIVE MODE - JOYSTICKS
    //Find LEFT joystick distance from center position. Value of 128 is center in the Y axes.
    joystick_left_Y = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD/ShoulderDown is positive)
  
    //Get LEFT DRIVE speed. Check for joystick movement beyond UP/DOWN dead zone.
    if (joystick_left_Y > 20 || joystick_left_Y < -20) {      //Create "dead zone" for when joystick is centered (with independent adjustment values for DOWN and UP. Defaults to 20 for both.).
      drive_speed_left = map(joystick_left_Y, 0, 128, 0, 128);    //Map values to HALF power for better drivability.
    }
    else {
      drive_speed_left = 0;    //if no detectable joystick movement
    }

    //Check LEFT DRIVE direction
    if (drive_speed_left != 0) {
      Serial.print("Left Drive ");
      //MOVING FORWARD (Joystick Forward)
      if (drive_speed_left > 0) {   //Check FORWARD/BACK direction of joystick (FORWARD is greater than zero and moves the left wheels forward)
        driveLeftState = 1;
        driveLeftManual(drive_speed_left);   //Left Drive Motor Forward
        Serial.print("Forward! \n");
      }
      //MOVING BACKWARD (Joystick Backward)
      else if (drive_speed_left < 0) {
        driveLeftState = -1;
        driveLeftManual(drive_speed_left);;   //Left Drive Motor Backward
        Serial.print("Backward! \n");
      }
    }
    else if(drive_speed_left == 0 && driveLeftState != 0) {
      //Stop
      driveLeftManual(STOP);;   //Left Drive Motor Stop
      driveLeftState = 0;
      Serial.print("LEFT DRIVE NEUTRAL!!\n");
    }
  
    //Find RIGHT joystick distance from center position. Value of 128 is center in both X and Y axes.
    joystick_right_Y = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Get joystick difference from center position (FORWARD/ShoulderDown is positive)
  
    //Check for joystick movement beyond UP/DOWN dead zone
    if (joystick_right_Y > 20 || joystick_right_Y < -20) {      //Create "dead zone" for when joystick is centered (with independent adjustment values for DOWN and UP. Defaults to 20 for both.).
      drive_speed_right = map(joystick_right_Y, 0, 128, 0, 128);    //Map values to HALF power for better drivability
    }
    else {
      drive_speed_right = 0;    //if no detectable joystick movement
    }
    
    //Check RIGHT DRIVE direction
    if (drive_speed_right != 0) {
      Serial.print("Right Joystick: ");
      //MOVING FORWARD (Joystick Forward)
      if (drive_speed_right > 0) {   //Check FORWARD/BACK direction of joystick (FORWARD is greater than zero and moves the shoulder DOWN)
        driveRightState = 1;
        driveRightManual(drive_speed_right);   //Right Drive Motor Forward
        Serial.print("Right Drive Forward! \n");
      }
      //MOVING BACKWARD (Joystick Backward)
      else if (drive_speed_right < 0) {
        driveRightState = -1;
        driveRightManual(drive_speed_right);   //Right Drive Motor Backward
        Serial.print("Right Drive Backward! \n");
      }
    }
    else if(drive_speed_right == 0 && driveRightState != 0) {
      //Stop
      driveRightManual(STOP);   //Right Drive Motor Backward
      driveRightState = 0;
      Serial.print("RIGHT DRIVE NEUTRAL!!\n");
    }
  }

  //ARM CONTROL MODE
  else if(controlMode == 1) {   
    /***************************
      TRIGGERS - ARM MODE
     ***************************/
    //L1 and L2 Triggers (Used for Tail Gripper)
    if (ps2.readButton(PS2_LEFT_1) == 0) // 0 = pressed, 1 = released
    {
      delay(10);  //debounce button by pausing and then double checking buttonstate
      if(ps2.readButton(PS2_LEFT_1) == 0 && tailGripState == 1) {
        Serial.println("L1 Pressed!");
        tailGripper(OPEN);
      }
    }
    if(ps2.readButton(PS2_LEFT_2) == 0) // 0 = pressed, 1 = released
    {
      delay(10);  //debounce button by pausing and then double checking buttonstate
      if(ps2.readButton(PS2_LEFT_2) == 0 && tailGripState == 0) { 
        Serial.println("L2 Pressed!");
        tailGripper(CLOSE);
      }
    }
     
    //R1 and R2 Triggers (Used for Arm Gripper)
    if(ps2.readButton(PS2_RIGHT_1) == 0) // 0 = pressed, 1 = released
    {
      delay(10);  //debounce button by pausing and then double checking buttonstate
      if(ps2.readButton(PS2_RIGHT_1) == 0) {
        Serial.println("R1 Pressed!");
        armGripper(OPEN);   //automatically fully open gripper based on timer default constant
      }
    }
    if(ps2.readButton(PS2_RIGHT_2) == 0) // 0 = pressed, 1 = released
    {
      delay(10);  //debounce button by pausing and then double checking buttonstate
      if(ps2.readButton(PS2_RIGHT_2) == 0) {
        Serial.println("R2 Pressed!");
        armGripperManual(CLOSE);    //Continually close arm gripper while button is pressed.
      }
    }
    else if(ps2.readButton(PS2_RIGHT_2) == 1)
    {
      armGripperManual(STOP);
    }
  
    /****************************
        BUTTONS - ARM MODE
    ****************************/
    //Triangle
    if(ps2.readButton(PS2_TRIANGLE) == 0) // 0 = pressed, 1 = released
    {
      delay(10);
      if(ps2.readButton(PS2_TRIANGLE) == 0 && triangleState == 0) { //double check button to prevent false trigger
        triangleState = 1;
        Serial.println("Triangle Pressed");
      }
    }
    else if(ps2.readButton(PS2_TRIANGLE) == 1 && triangleState == 1)
    {
      triangleState = 0;
      Serial.println("Triangle Released");
    }
    
    //Circle
    if(ps2.readButton(PS2_CIRCLE) == 0) // 0 = pressed, 1 = released
    {
      delay(10);
      if(ps2.readButton(PS2_CIRCLE) == 0 && circleState == 0) { //double check button to prevent false trigger
        circleState = 1;
        Serial.println("Circle Pressed!");
      }
    }
    else if(ps2.readButton(PS2_CIRCLE) == 1 && circleState == 1)
    {
      circleState = 0;
      Serial.println("Circle Released!");
    }
    
    //Cross
    if(ps2.readButton(PS2_CROSS) == 0) // 0 = pressed, 1 = released
    {
      delay(10);
      if(ps2.readButton(PS2_CROSS) == 0 && crossState == 0) { //double check button to prevent false trigger
        crossState = 1;
        Serial.println("Cross Pressed!");
      }
    }
    else if(ps2.readButton(PS2_CROSS) == 1 && crossState == 1)
    {
      crossState = 0;
      Serial.println("Cross Released!");
    }
    
    //Square
    if(ps2.readButton(PS2_SQUARE) == 0) // 0 = pressed, 1 = released
    {
      delay(10);
      if(ps2.readButton(PS2_SQUARE) == 0 && squareState == 0) { //double check button to prevent false trigger
        squareState = 1;
        Serial.println("Square Pressed!");
      }
    }
    else if (ps2.readButton(PS2_SQUARE) == 1 && squareState == 1)
    {
      squareState = 0;
      Serial.println("Square Released!");
    }
  
    /****************************
        JOYSTICK BUTTONS - ARM MODE
    ****************************/
  
    //JOYSTICK BUTTONS
    //Left Joystick Press (L3)
    if(ps2.readButton(PS2_JOYSTICK_LEFT) == 0) // 0 = pressed, 1 = released
    {
      delay(10);
      if(ps2.readButton(PS2_JOYSTICK_LEFT) == 0 && joystickLeftState == 0) { //double check button to prevent false trigger
        joystickLeftState = 1;
        turnTableReset();
        Serial.println("Left Joystick Pressed!");
      }
    }
    else if (ps2.readButton(PS2_JOYSTICK_LEFT) == 1 && joystickLeftState == 1)
    {
      joystickLeftState = 0;
      Serial.println("Left Joystick Released!");
    }
  
    //Right Joystick Press (R3)
    if(ps2.readButton(PS2_JOYSTICK_RIGHT) == 0) // 0 = pressed, 1 = released
    {
      delay(10);
      if(ps2.readButton(PS2_JOYSTICK_RIGHT) == 0 && joystickRightState == 0) { //double check button to prevent false trigger
        wristReset();
        joystickRightState = 1;
        Serial.println("Right Joystick Pressed!");
      }
    }
    else if (ps2.readButton(PS2_JOYSTICK_RIGHT) == 1 && joystickRightState == 1)
    {
      joystickRightState = 0;
      Serial.println("Right Joystick Released!");
    }

    /*****************************
        JOYSTICKS - ARM MODE
    *****************************/
    //LEFT JOYSTICK
    //Find LEFT joystick distance from center position. Value of 128 is center in both X and Y axes.
    //NOTE: if Cytron remote control unit is not connected to a remote, the button value will be returned as 255.
    joystick_left_Y = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD/ShoulderDown is positive)
    joystick_left_X = ps2.readButton(PS2_JOYSTICK_LEFT_X_AXIS) - 128;  //Get joystick difference from center position (RIGHT/RotateCW is positive)
  
    //Get Shoulder LIFT speed. Check for joystick movement beyond UP/DOWN dead zone.
    if (joystick_left_Y > 50 || joystick_left_Y < -50) {      //Create "dead zone" for when joystick is centered (with independent adjustment values for DOWN and UP. Defaults to 50 for both.).
      shoulder_lift_speed = map(joystick_left_Y, 0, 128, 0, 255);    //Map values to get full power delivery using only half of joystick travel (from center to extremity)
    }
    else {
      shoulder_lift_speed = 0;    //if no detectable joystick movement
    }
    //Get Shoulder TURN speed. Check for joystick movement beyond LEFT/RIGHT dead zone.
    if (joystick_left_X > 100 || joystick_left_X < -100) {  //Add a wide buffer for truntable rotation to prevent accidental rotation
      turntable_speed = map(joystick_left_X, 0, 128, 0, 255); 
    }  
    else {
      turntable_speed = 0;    //if no detectable joystick movement
    }
    //Check LIFT direction
    if (shoulder_lift_speed != 0) {
      Serial.print("\n");
      Serial.print("left Y axis: ");
      Serial.println(joystick_left_Y);
      Serial.print("left X axis: ");
      Serial.println(joystick_left_X);
      Serial.print("Left Joystick Lift: ");
      //MOVING DOWN (Joystick Forward)
      if (shoulder_lift_speed > 0) {   //Check FORWARD/BACK direction of joystick (FORWARD is greater than zero and moves the shoulder DOWN)
        Serial.print("Down! \n");
        shoulderMoveManual(SHOULDER_MIN);   //Limit for the shoulder joint's lowest position
      }
      //MOVING UP (Joystick Backward)
      else if (shoulder_lift_speed < 0) {
        Serial.print("Up! \n");
        shoulderMoveManual(SHOULDER_MAX);   //Limit for the shoulder joint's highest position
      }
    }
    else if(shoulder_lift_speed == 0 && shoulderState != 0) {
      //Stop
      shoulderMoveManual(STOP);
      Serial.print("SHOULDER UP/DOWN NEUTRAL!!\n");
    }
  
    //Check LEFT/RIGHT direction
    if (turntable_speed != 0) {
      Serial.print("Turntable Speed: ");
      Serial.print(turntable_speed);
      Serial.print("\n");
      Serial.print("Left Joystick Rotation: ");
      //MOVING CW (Joystick Right)
      if (turntable_speed > 0) {   //Check LEFT/RIGHT direction of joystick (RIGHT is greater than zero and moves the shoulder CW)
        Serial.print("CW! \n");
        turnTableState = 1;
        turnTableManual(turnTableState);
      }
      //MOVING CCW (Joystick Left)
      else if (turntable_speed < 0) {
        Serial.print("CCW! \n");
        turnTableState = -1;
        turnTableManual(turnTableState);
      }
    }
    else if(turntable_speed == 0 && turnTableState != 0) {
      //Stop
      turnTableManual(STOP);
      turnTableState = 0;
      turnTableCount = 1; //Reset turnTableCount. Default to 1 for easier calculations (15 ticks ~ 90 degrees)
      Serial.print("TURNTABLE ROTATION NEUTRAL!!\n");
    }

    //RIGHT JOYSTICK
    //Find RIGHT joystick distance from center position. Value of 128 is center in both X and Y axes.
    joystick_right_Y = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Get joystick difference from center position (FORWARD/ShoulderDown is positive)
    joystick_right_X = ps2.readButton(PS2_JOYSTICK_RIGHT_X_AXIS) - 128;  //Get joystick difference from center position (RIGHT/RotateCW is positive)
  
    //Check for joystick movement beyond UP/DOWN dead zone
    if (joystick_right_Y > 50 || joystick_right_Y < -50) {      //Create "dead zone" for when joystick is centered (with independent adjustment values for DOWN and UP. Defaults to 50 for both.).
      elbow_lift_speed = map(joystick_right_Y, 0, 128, 0, 255);    //Map values to get full power delivery using only half of joystick travel (from center to extremity)
    }
    else {
      elbow_lift_speed = 0;    //if no detectable joystick movement
    }
    //Check for joystick movement beyond LEFT/RIGHT dead zone
    if (joystick_right_X > 100 || joystick_right_X < -100) { 
      wrist_turn_speed = map(joystick_right_X, 0, 128, 0, 255); 
    }  
    else {
      wrist_turn_speed = 0;    //if no detectable joystick movement
    }
    //Check UP/DOWN direction
    if (elbow_lift_speed != 0) {
      Serial.print("\n");
      Serial.print("Right Y axis: ");
      Serial.println(joystick_right_Y);
      Serial.print("Right X axis: ");
      Serial.println(joystick_right_X);
      Serial.print("Elbow Lift Speed: ");
      Serial.print(elbow_lift_speed);
      Serial.print("\n");
      Serial.print("Right Joystick: ");
      //MOVING DOWN (Joystick Forward)
      if (elbow_lift_speed > 0) {   //Check FORWARD/BACK direction of joystick (FORWARD is greater than zero and moves the shoulder DOWN)
        Serial.print("Down! \n");
        elbowMoveManual(ELBOW_MIN);
      }
      //MOVING UP (Joystick Backward)
      else if (elbow_lift_speed < 0) {
        Serial.print("Up! \n");
        elbowMoveManual(ELBOW_MAX);
      }
    }
    else if(elbow_lift_speed == 0 && elbowState != 0) {
      //Stop
        elbowMoveManual(STOP);
        elbowState = 0;
        Serial.print("ELBOW UP/DOWN NEUTRAL!!\n");
    }
    
    //Check CW/CCW direction
    if (wrist_turn_speed != 0) {
      Serial.print("Wrist Turn Speed: ");
      Serial.print(wrist_turn_speed);
      Serial.print("\n");
      Serial.print("Right Joystick: ");
      //MOVING CW (Joystick Right)
      if (wrist_turn_speed > 0) {   //Check FORWARD/BACK direction of joystick (FORWARD is greater than zero and moves the shoulder DOWN)
        wristState = 1;
        lastWristDirection = wristState;
        wristManual(wristState);
        Serial.print("CW! \n");
      }
      //MOVING CCW (Joystick Left)
      else if (wrist_turn_speed < 0) {
        wristState = -1;
        lastWristDirection = wristState;
        wristManual(wristState);
        Serial.print("CCW! \n");
      }
    }
    else if(wrist_turn_speed == 0 && wristState != 0) {
      //Stop
      wristManual(STOP);
      wristState = 0;
      Serial.print("WRIST NEUTRAL!!\n");
    }
  }
}


void driveLeftManual(int driveSpeed) {
  if(runArray[6] == 1) {    //Check flag to see if movement of this motor is allowed

    //if Stopped
    if(driveSpeed == STOP) {
      //check motor direction based on driveLeftState and brake by setting the motorspeed to the opposite motor direction
      if(driveLeftState == 1) {
        driveLeft.stop();
        driveLeft.run(-driveSpeed); //Reverse motor direction to brake briefly (Braking direction based on actual motor wire connection)
      }
      else if(driveLeftState == -1) {
        driveLeft.stop();
        driveLeft.run(driveSpeed); //Reverse motor direction to brake briefly (Braking direction based on actual motor wire connection)
      }
      delay(10);
      driveLeft.run(0);    //Release motor by setting speed to zero
      driveLeft.stop();
      Serial.print("Drive Left Encoder: ");   //Display stored global arm position
      Serial.println(driveLeftEncoder);
      Serial.println("DRIVE LEFT STOPPED!");
    }
    //if driving forward
    else if(driveSpeed > 0 && driveLeftState != 0) {   //The driveLeftState flag prevents motor from continuing to run if stopped by tick count and joystick is still held in a movement diection
      driveLeft.run(driveSpeed);    //No need to "negate" driveSpeed since it already comes from the controller as a positive OR negative value.
      Serial.println("Drive Left Forward");
    }
    //if driving backward
    else if(driveSpeed < 0 && driveLeftState != 0) {   //The driveLeftState check prevents motor from continuing to run if stopped by tick count and joystick is still held in a movement diection.
      driveLeft.run(driveSpeed);    //No need to "negate" driveSpeed since it already comes from the controller as a positive OR negative value.
      Serial.println("Drive Left Backward");
    }
    //Read left drive encoder
    driveLeftAnalog = analogRead(DRIVE_LEFT_ENCODER);
    //Check if encoder signal HIGH
    if(driveLeftAnalog > DRIVE_LEFT_ANALOG_MAX && driveLeftEncoder == 0) { //if encoder value passes the threshold for HIGH, and the current state of the sensor is LOW, set sensor state to HIGH and increment the tick count.
      driveLeftEncoder = 1;
      driveLeftDistance++;   //Increment encoder tick count each time the sensor reads HIGH
    }
    //Check if encoder signal LOW
    else if(driveLeftAnalog < DRIVE_LEFT_ANALOG_MIN && driveLeftEncoder == 1) {  //if encoder value goes below the threshold for LOW, and the current state of the sensor is HIGH, set the sensor state to LOW and wait for next trigger.
      driveLeftEncoder = 0;
    }
    Serial.print("Drive Left: ");
    Serial.print("A ");
    Serial.print(driveLeftAnalog);
    Serial.print("\t");
    Serial.print("E ");
    Serial.print(driveLeftEncoder);
    Serial.print("\t");
    Serial.print("D ");
    Serial.println(driveLeftDistance);
    delay (10);
  }
}


void driveRightManual(int driveSpeed) {
  if(runArray[6] == 1) {    //Check flag to see if movement of this motor is allowed

    //if Stopped
    if(driveSpeed == STOP) {
      //check motor direction based on driveLeftState and brake by setting the motorspeed to the opposite motor direction
      if(driveRightState == 1) {
        driveRight.stop();
        driveRight.run(-driveSpeed); //Reverse motor direction to brake briefly (Braking direction based on actual motor wire connection)
      }
      else if(driveRightState == -1) {
        driveRight.stop();
        driveRight.run(driveSpeed); //Reverse motor direction to brake briefly (Braking direction based on actual motor wire connection)
      }
      delay(10);
      driveRight.run(0);    //Release motor by setting speed to zero
      driveRight.stop();
      Serial.print("Drive Right Encoder: ");   //Display stored global arm position
      Serial.println(driveRightEncoder);
      Serial.println("DRIVE RIGHT STOPPED!");
    }
    //if driving forward
    else if(driveSpeed > 0 && driveRightState != 0) {   //The driveRightState flag prevents motor from continuing to run if stopped by tick count and joystick is still held in a movement diection
      driveRight.run(driveSpeed);   //No need to "negate" driveSpeed since it already comes from the controller as a positive OR negative value.
      Serial.println("Drive Right Forward");
    }
    //if driving backward
    else if(driveSpeed < 0 && driveRightState != 0) {   //The driveRightState check prevents motor from continuing to run if stopped by tick count and joystick is still held in a movement diection.
      driveRight.run(driveSpeed);    //No need to "negate" driveSpeed since it already comes from the controller as a positive OR negative value.
      Serial.println("Drive Right Backward");
    }
    //Read right drive encoder
    driveRightAnalog = analogRead(DRIVE_RIGHT_ENCODER);
    //Check if encoder signal HIGH
    if(driveRightAnalog > DRIVE_RIGHT_ANALOG_MAX && driveRightEncoder == 0) { //if encoder value passes the threshold for HIGH, and the current state of the sensor is LOW, set sensor state to HIGH and increment the tick count.
      driveRightEncoder = 1;
      driveRightDistance++;   //Increment encoder tick count each time the sensor reads HIGH
    }
    //Check if encoder signal LOW
    else if(driveRightAnalog < DRIVE_RIGHT_ANALOG_MIN && driveRightEncoder == 1) {  //if encoder value goes below the threshold for LOW, and the current state of the sensor is HIGH, set the sensor state to LOW and wait for next trigger.
      driveRightEncoder = 0;
    }
    Serial.print("Drive Right: ");
    Serial.print("A ");
    Serial.print(driveRightAnalog);
    Serial.print("\t");
    Serial.print("E ");
    Serial.print(driveRightEncoder);
    Serial.print("\t");
    Serial.print("D ");
    Serial.println(driveRightDistance);
    delay (10);
  }
}


void wristManual(int wristDirection, int wristSpeed = 255) {
  if(runArray[1] == 1) {    //Check flag to prevent unnecessary re-triggering of the function
    Serial.println("Wrist Manual Called!");
    //if Stopped
    if(wristDirection == STOP) {
      //check command direction based on wristState (brake by setting the motorspeed to the opposite motor direction)
      if(wristState == 1) {
        armWrist.stop();
        armWrist.run(-wristSpeed); //Reverse motor direction to brake briefly (Braking direction based on actual motor wire connection)
      }
      else if(wristState == -1) {
        armWrist.stop();
        armWrist.run(wristSpeed); //Reverse motor direction to brake briefly (Braking direction based on actual motor wire connection)
      }
      delay(10);
      armWrist.run(0);    //Release motor by setting speed to zero
      armWrist.stop();
      Serial.println("WRIST STOPPED!");
    }
    //if rotating CW
    else if(wristDirection > 0 && wristState != 0) {   //The wristState flag is probably unnecessary
      armWrist.run(wristSpeed);
      Serial.println("Wrist Clockwise!");
    }
    //if rotating CCW
    else if(wristDirection < 0 && wristState != 0) {   //The wristState check prevents motor from continuing to run if stopped by tick count and joystick is still held in a movement diection.
      armWrist.run(-wristSpeed);
      Serial.println("Wrist CounterClockwise!");
    }
  }
}


void wristReset() {
  if(runArray[1] == 1) {    //Check flag to prevent unnecessary re-triggering of the function
    int wristSpeed = 255;
    //Update wristOrientation
    if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == LOW) {  //If switch AND hall effect sensors are both "activated" (hall sensor is LOW when active), gripper is HORIZONTAL (H)
      wristOrientation = H;
    }
    else if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == HIGH) { //If switch is activated (HIGH) but hall effect sensor is NOT (hall sensor is LOW when active), gripper is VERTICAL (V)
      wristOrientation = V;
    }
    //Display wrist info
    Serial.print("\n");
    Serial.print("Wrist Reset!");
    Serial.print("\n");
    Serial.print("Start Orientation: ");    //Display current orientation at start
    if(wristOrientation == H) {
      Serial.print("H");
    }
    else if(wristOrientation == V) {
      Serial.print("V");
    }
    Serial.print("\n");
    Serial.print("Wrist Direction: ");    //display the desired direction
    
    //Initialize wrist postion to the nearest cardinal position (determined by switch activation by one of 4 hardware screws combined with state of hall effect sensor)
    if(digitalRead(WRIST_SWITCH) == LOW){     //If switch starts in UNPRESSED (LOW) state (switch is NOT currently positioned over a hardware screw)
      //Set rotation direction based on the last used direction of rotation
      if(lastWristDirection > 0) {
      armWrist.run(wristSpeed);  
      Serial.print("CW");
      }
      else if(lastWristDirection < 0) {
        armWrist.run(-wristSpeed);
        Serial.print("Wrist CCW");
      }              
      while(digitalRead(WRIST_SWITCH) == LOW) {   //Run motor until a switch is activated (which means wrist is in one of the cardinal directions)
        //Wait for switch to go HIGH (switch is PRESSED)
      }
      delay(50);  //Insert slight delay to allow switch roller to fully seat over the hardware screw.
      //Brake motor once switch is activated
      armWrist.stop();
      //Set braking direction based on the last used direction of rotation
      if(lastWristDirection > 0) {
      armWrist.run(-wristSpeed);  
      Serial.print("Braking CCW!");
      }
      else if(lastWristDirection < 0) {
        armWrist.run(wristSpeed);
        Serial.print("Braking CW!");
      } 
      delay(30);
      armWrist.run(0);    //Release motor by setting speed to zero
      armWrist.stop();

      //Update wristOrientation
      if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == LOW) {  //If switch AND hall effect sensors are both "activated" (hall sensor is LOW when active), gripper is HORIZONTAL (H)
        wristOrientation = H;
      }
      else if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == HIGH) { //If switch is activated (HIGH) but hall effect sensor is NOT (hall sensor is LOW when active), gripper is VERTICAL (V)
        wristOrientation = V;
      }
    }
    else if(digitalRead(WRIST_SWITCH) == HIGH){ //If switch starts in PRESSED state (switch IS currently positioned over a hardware screw)
      //Set rotation direction based on the last used direction of rotation
      if(lastWristDirection > 0) {
      armWrist.run(wristSpeed);  
      Serial.print("CW");
      }
      else if(lastWristDirection < 0) {
        armWrist.run(-wristSpeed);
        Serial.print("Wrist CCW");
      }  
      while(digitalRead(WRIST_SWITCH) == HIGH) {  //Run motor until switch is re-activated (which means wrist has rotated 90 degrees to a new cardinal direction but different from the last)
        //Wait for switch to deactivate
      }
      while(digitalRead(WRIST_SWITCH) == LOW) {
        //Wait for switch to re-activate
      }
      delay(50);  //Insert slight delay to allow switch roller to fully seat over the hardware screw.
      //Brake motor once switch is activated
      armWrist.stop();
      //Set braking direction based on the last used direction of rotation
      if(lastWristDirection > 0) {
      armWrist.run(-wristSpeed);  
      Serial.print("Braking CCW!");
      }
      else if(lastWristDirection < 0) {
        armWrist.run(wristSpeed);
        Serial.print("Braking CW!");
      } 
      delay(30);
      armWrist.run(0);    //Release motor by setting speed to zero
      armWrist.stop();

      //Update wristOrientation
      if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == LOW) {  //If switch AND hall effect are "activated" (hall effect sensor is LOW when activated), gripper is HORIZONTAL (H)
        wristOrientation = H;
      }
      else if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == HIGH) { //If switch is activated but hall effect sensor is NOT (hall effect sensor is LOW when activated), gripper is VERTICAL (V)
        wristOrientation = V;
      }
    }
  }
}


void turnTableReset() {
  if(runArray[4] == 1) {    //Check flag to prevent unnecessary re-triggering of the function
    Serial.print("\n");
    Serial.println("Turntable Reset!");
    Serial.print("\n");
    int turnSpeed = 65;   //Set to one quarter maximum speed
    
    Serial.print("Turntable Switch at Reset Start: ");
    Serial.print(turnTableSwitch);
    Serial.print("\n");

    Serial.print("Turntable Hall at Reset Start: ");
    Serial.print(turnTableHall);
    Serial.print("\n");
    
    turnTable.run(turnSpeed);   //Default direction is CCW which is what we want so no change necessary.
    while (turnTableSwitch < 1) {   //while turntable switch is not activated.
      turnTableSwitch = digitalRead(TURNTABLE_SWITCH);
      if(turnTableSwitch > 0) { //if turntable reaches physical limit (indicated by switch HIGH), then stop motor.
        turnTableHall = digitalRead(TURNTABLE_HALL);    //Read hall effect sensor which effectively resets it since it should NOT be active when the limit switch is active
        //Brake motor once limit switch is activated
        turnTable.stop();
        turnTable.run(-turnSpeed); //Reverse motor direction to brake briefly
        delay(30);
        turnTable.run(0);    //Release motor by setting speed to zero
        turnTable.stop();
      }
      delay (10);
    }
    Serial.print("Turntable Switch at Reset Limit: ");
    Serial.print(turnTableSwitch);
    Serial.print("\n");

    Serial.print("Turntable Hall at Reset Limit: ");
    Serial.print(turnTableHall);
    Serial.print("\n");
    
    delay(1000);

    Serial.print("\n");
    Serial.println("Turntable to: Zero Position");
    Serial.print("\n");
    turnTable.run(-turnSpeed);  //Rotate CW until both turntable switch and hall effect sensor are LOW
    while (turnTableHall > 0) {   //while turntable hall effect sensor is not activated.
      //update sensor states
      turnTableHall = digitalRead(TURNTABLE_HALL);
      turnTableSwitch = digitalRead(TURNTABLE_SWITCH);
      delay (10);
    }
    delay(100); //pause briefly to allow arm to truly center. extra time is required due to detection angle of sensor and arm turn speed.
    //Brake motor once limit switch is activated
    turnTable.stop();
    turnTable.run(turnSpeed); //Reverse motor direction to brake briefly
    delay(30);
    turnTable.run(0);    //Release motor by setting speed to zero
    turnTable.stop();
    turnTablePosition = 0;

    Serial.print("Turntable Hall at Reset End: ");
    Serial.print(turnTableHall);
    Serial.print("\n");

    Serial.print("Turntable Switch at Reset End: ");
    Serial.print(turnTableSwitch);
    Serial.print("\n");
    
    Serial.print("\n");
    Serial.println("Turntable and Arm at Zero Position!");
  }
}


void turnTableManual(int commandState = 0, int turnSpeed = 65) {
  if(runArray[4] == 1) {    //Check flag to see if movement of this joint is allowed
    float targetDistance = 0;   //store the distance to the limit of turntable travel measured in encoder ticks
 //   float conversionRate = 14.0;

    //if Stopped
    if(commandState == STOP) {
      //check motor direction based on turnTableState and brake by setting the motorspeed to the opposite motor direction. turnTableState is set by joystick input.
      if(turnTableState == 1) {
        turnTable.stop();
        turnTable.run(turnSpeed); //Reverse motor direction to brake briefly (Braking direction based on actual motor wire connection)
      }
      else if(turnTableState == -1) {
        turnTable.stop();
        turnTable.run(-turnSpeed); //Reverse motor direction to brake briefly (Braking direction based on actual motor wire connection)
      }
      delay(10);
      turnTable.run(0);    //Release motor by setting speed to zero
      turnTable.stop();
      Serial.print("Turntable Position: ");   //Display stored global arm position
      Serial.println(turnTablePosition);
      Serial.println("TURNTABLE STOPPED!");
    }
    //if rotating CW (commandState "positive")
    else if(commandState > 0 && turnTableState != 0) {   //The turnTableState flag prevents motor from continuing to run if stopped by tick count and joystick is still held in a movement diection. IMPORTANT: turnTableState is reset to zero when the limit is reached.
      targetDistance = TURNTABLE_LIMIT - turnTablePosition; //Since the joystick can be release at any time, set targetDistance to the maximum distance required to move the arm to the fully extended position (180 degrees from tail gripper). TURNTABLE_LIMIT is the number of encoder ticks to travel 180 degrees.
 //     conversionRate = 14.5;
      Serial.println("Turntable CW");

      //Check if LAST state of Turntable Hall Effect sensor was LOW (hall effect sensor is "activated" when LOW - arm gripper is aligned with tail gripper)
      if(turnTableHall == 0) {   //if hall effect sensor WAS activated...then double check current value until it is DEACTIVATED.
        //Read turntable hall effect sensor.
        turnTableHall = digitalRead(TURNTABLE_HALL);    //Read current state of hall effect sensor
        Serial.print("Turntable Hall: ");
        Serial.print(turnTableHall);
        Serial.print("\n");
        if(turnTableHall == 1) {  //if sensor is DEACTIVATED...
          turnTablePosition = 2;  //Reset turnTablePosition to the default encoder position at the point where the hall effect sensor becomse deactivated. Always resetting this value based on the hall effect sensor ensures better accuracy for turnTablePosition.
          Serial.println("HALL EFFECT RESET!");
          Serial.print("Turntable Position: ");
          Serial.println(turnTablePosition);
        }
      }
    }
    //if rotating CCW (commandState "negative")
    else if(commandState < 0 && turnTableState != 0) {   //The turnTableState check prevents motor from continuing to run if stopped by tick count and joystick is still held in a movement diection. IMPORTANT: turnTableState is reset to zero when the limit is reached.
      targetDistance = turnTablePosition;   //targetDistance is just equal to the current turnTablePosition since we are rotating arm towards "zero" (towards tail gripper)
//      conversionRate = 14.0;   //adjust tick target due to tension from the main cable.
      Serial.println("Turntable CCW");
      
      //Check if Hall Effect sensor state is NOT activated (hall effect sensor is "activated" when LOW)
      if(turnTableHall == 1) {   //if sensor is NOT activated
        //Read turntable hall effect sensor.
        turnTableHall = digitalRead(TURNTABLE_HALL);    //Read current state of hall effect sensor
        Serial.print("Turntable Hall: ");
        Serial.print(turnTableHall);
        Serial.print("\n");
        if(turnTableHall == 0) {  //if sensor IS activated AFTER reading, CCW turntable rotatin limit has been reached.
          turnTablePosition = 1;  //Set turnTablePosition to a default value based on how far the turntable still needs to travel at default speed to be at the true limit. (The hall effect sensor tends to throw early before the magnet is centered over the sensor.)
          Serial.println("HALL EFFECT LIMIT!");
          Serial.print("Turntable Position: ");
          Serial.println(turnTablePosition);
        }
      }
    }
    Serial.print("Target Distance: ");
    Serial.print(targetDistance);
    Serial.print("\n");

    if(targetDistance > 0) {    //Check if turnTablePosition is still some distance away from the limit
      //Activate motor in the desired direction
      turnTableTarget = 0;    //Reset turnTableTarget flag to FALSE (has NOT reached target) if there is distance between the turntable limit and turnTablePosition
      if(turnTableState == 1) {   //Check motor direction based on turnTabel State and set brake direction accordingly
        turnTable.run(-turnSpeed);    //Unintuitive negative motor speed for clockwise direction based on actual motor wire connections
      }
      else if(turnTableState == -1) {
        turnTable.run(turnSpeed);    //Unintuitive positive motor speed for counter-clockwise direction based on actual motor wire connections
      }
      
      //Read turntable encoder
      turnTableAnalog = analogRead(TURNTABLE_ENCODER);
      //Check if encoder signal HIGH
      if(turnTableAnalog > TURNTABLE_ANALOG_MAX && turnTableEncoder == 0) { //if encoder value passes the threshold for HIGH, and the current state of the sensor is LOW, set sensor state to HIGH and increment the tick count.
        turnTableEncoder = 1;
        turnTableCount++;   //Increment encoder tick count each time th sensor reads HIGH
        //Upstate turnTablePosition
        if(turnTableState > 0) {
          turnTablePosition++;
        }
        else if(turnTableState < 0) {
          turnTablePosition--;
        }
      }
      //Check if encoder signal LOW
      else if(turnTableAnalog < TURNTABLE_ANALOG_MIN && turnTableEncoder == 1) {  //if encoder value goes below the threshold for LOW, and the current state of the sensor is HIGH, set the sensor state to LOW and wait for next trigger.
        turnTableEncoder = 0;
      }
      Serial.print("Turntable: ");
      Serial.print("A ");
      Serial.print(turnTableAnalog);
      Serial.print("\t");
      Serial.print("E ");
      Serial.print(turnTableEncoder);
      Serial.print("\t");
      Serial.print("C ");
      Serial.println(turnTableCount);
      delay (10);
    }
    else if(targetDistance <= 0 && turnTableTarget != 1) {  //If target limit has been reached for the first time (as indicated by turnTableTarget flag)
      //check motor direction based on turnTableState and brake by setting the motorspeed to the opposite motor direction
      if(turnTableState == 1) {
        turnTable.stop();
        turnTable.run(turnSpeed); //Reverse motor direction to brake briefly
      }
      else if(turnTableState == -1) {
        turnTable.stop();
        turnTable.run(-turnSpeed); //Reverse motor direction to brake briefly
      }
      delay(10);
      turnTable.run(0);    //Release motor by setting speed to zero
      turnTable.stop();
      turnTableTarget = 1;   //Set turnTableState flag to TRUE once target value has been reached. This prevents this clause from running again if joystick is still held a movement direction after target value has been reached
      Serial.println("TARGET VALUE REACHED!");
      Serial.print("Turntable Position: ");
      Serial.println(turnTablePosition);
    }
  }
}


void elbowMoveManual(int elbowPosition = 500, int elbowSpeed = 65) { //Default values allow the function to be called without arguments to reset to a default position (at the default speed).
  if(runArray[2] == 1) {    //Check if movement is allowed
    int lastPosition = analogRead(ELBOW_POT);   //read encoder position
    
    if(elbowPosition == ELBOW_MIN) {   //Check if command is to move "down"
      if(lastPosition > ELBOW_MIN) {  //check if elbow has NOT reached lower limit
        elbow.run(-elbowSpeed);
        elbowDirection = -1;
        elbowState = -1;   //Set elbow as MOVING DOWN
        Serial.print("Elbow DOWN\t");
        Serial.print("Target Position: ");
        Serial.print(lastPosition);
        Serial.print("\n\n");
      }
      else if(lastPosition <= ELBOW_MIN && elbowState == -1) {    //Only stop motor if limit has been reached or exceeded AND elbowState is set to "moving down". This prevent the reverse braking method to cause unnecessary jitter in the motor when there is no change in state.
        //Brake motor once target position is reached
        elbow.stop();
        elbow.run(elbowSpeed); //Reverse motor direction to brake briefly
        delay(30);
        elbow.run(0);    //Release motor by setting speed to zero
        elbow.stop();
        elbowState = 0;   //Set elbow as NOT MOVING to prevent retrigger if joystick held in same position
        Serial.print("Stopped DOWN!\n\n");
      }
    }
    else if(elbowPosition == ELBOW_MAX) {
      if(lastPosition < ELBOW_MAX) {  //check if elbow has NOT reached upper limit
        elbow.run(elbowSpeed);
        elbowDirection = 1;
        elbowState = 1;   //Set elbow as MOVING UP
        Serial.print("\n");
        Serial.print("Elbow UP\t");
        Serial.print("Target Position: ");
        Serial.print(lastPosition);
        Serial.print("\n");
      }
      else if(lastPosition >= ELBOW_MAX && elbowState == 1) {    //Only stop motor if limit has been reached or exceeded AND elbowState is set to "moving up". This prevent the reverse braking method to cause unnecessary jitter in the motor when there is no change in state.
        //Brake motor once target position is reached
        elbow.stop();
        elbow.run(-elbowSpeed); //Reverse motor direction to brake briefly
        delay(10);
        elbow.run(0);    //Release motor by setting speed to zero
        elbow.stop();
        elbowState = 0;   //Set elbow as NOT MOVING to prevent retrigger if joystick held in same position
        Serial.print("Stopped UP!\n\n");
      }
    }
    else if(elbowPosition == STOP) {
      //Brake motor once target position is reached
      if(elbowDirection > 0) {
        elbow.stop();
        elbow.run(-elbowSpeed); //Reverse motor speed (based on elbowDirection) to brake briefly
        delay(10);
        elbow.run(0);    //Release motor by setting speed to zero
        elbow.stop();
      }
      else if(elbowDirection < 0) {
        elbow.stop();
        elbow.run(elbowSpeed); //Reverse motor speed (based on elbowDirection)
        delay(10);
        elbow.run(0);    //Release motor by setting speed to zero
        elbow.stop();
      }
      else {  //if elbowDirection cannot be determined, stop without braking
        elbow.stop();
        elbow.run(0);    //Release motor by setting speed to zero
        elbow.stop();
      }
      delay(30);
      elbowState = 0;   //Set elbow as NOT MOVING
      Serial.print("RIGHT JOYSTICK LIFT NEUTRAL!\n\n");
    }
  }
}


void shoulderMoveManual(int shoulderPosition = 550, int shoulderSpeed = 131) { //Default values allow the function to be called without arguments to reset to a default "center" position (at the default speed).
  if(runArray[3] == 1) {    //Check if movement is allowed
    int lastPosition = analogRead(SHOULDER_POT);   //read encoder position
    
    if(shoulderPosition == SHOULDER_MIN) {   //Check if command is to move "DOWN"
      if(lastPosition < SHOULDER_MIN) {  //check if shoulder has NOT reached lower limit. REVERSED: lastPosition INCREASES as shoulder LOWERS
        shoulder.run(shoulderSpeed);  //Positive motor speed when moving down
        shoulderDirection = 1;  //set to 1 for "positive" motor speed
        shoulderState = -1;   //Set shoulder as MOVING DOWN
        Serial.print("Shoulder DOWN\t");
        Serial.print("Target Position: ");
        Serial.print(lastPosition);
        Serial.print("\n\n");
      }
      else if(lastPosition >= SHOULDER_MIN && shoulderState == -1) {    //Only stop if limit has been reached or exceeded AND shoulderState has been set to "moving down". This prevent the reverse braking method to cause unnecessary jitter in the motor when there is no change in state.
        //Brake motor once target position is reached
        shoulder.stop();
        shoulder.run(-shoulderSpeed); //Reverse motor direction to brake briefly
        delay(15);          //pause for braking
        shoulder.run(0);    //Release motor by setting speed to zero
        shoulder.stop();
        delay(30);          //pause to make sure arm motion has fully stopped
        shoulderState = 0;  //Set shoulder as NOT MOVING to prevent retrigger if joystick is held in the same direction
        Serial.print("SHOULDER LIMIT DOWN!\n\n");    //Signal that the limit has be reached
      }
    }
    else if(shoulderPosition == SHOULDER_MAX) { //Check if command is to move "UP"
      if(lastPosition > SHOULDER_MAX) {  //check if shoulder has NOT reached upper limit. REVERSED: lastPosition DECREASES as shoulder RISES.
        shoulder.run(-shoulderSpeed); //Negative speed when moving upward
        shoulderDirection = -1;   //set to -1 for "negative" motor speed
        shoulderState = 1;   //Set shoulder as MOVING UP
        Serial.print("\n");
        Serial.print("Shoulder UP\t");
        Serial.print("Target Position: ");
        Serial.print(lastPosition);
        Serial.print("\n");
      }
      else if(lastPosition <= SHOULDER_MAX && shoulderState == 1) {    //Only stop if shoulderState has been reached or exceeded AND shouderState is set to "moving upward". This prevent the reverse braking method to cause unnecessary jitter in the motor when there is no change in state.
        //Brake motor once target position is reached
        shoulder.stop();
        shoulder.run(shoulderSpeed); //Reverse motor direction to brake briefly
        delay(15);
        shoulder.run(0);    //Release motor by setting speed to zero
        shoulder.stop();
        delay(30);
        shoulderState = 0;   //Set shoulder as NOT MOVING
        Serial.print("SHOULDER LIMIT UP!\n\n");
      }
    }
    else if(shoulderPosition == STOP) {
      //Brake motor once target position is reached
      //check motor direction
      if(shoulderDirection > 0) {
        shoulder.stop();
        shoulder.run(-shoulderSpeed); //Reverse motor speed (based on shoulderDirection) to brake briefly
        delay(10);
        shoulder.run(0);    //Release motor by setting speed to zero
        shoulder.stop();
      }
      else if(shoulderDirection < 0) {
        shoulder.stop();
        shoulder.run(shoulderSpeed); //Reverse motor speed (based on shoulderDirection)
        delay(10);
        shoulder.run(0);    //Release motor by setting speed to zero
        shoulder.stop();
      }
      else {  //if shoulderDirection cannot be determined, stop without braking
        shoulder.stop();
        shoulder.run(0);    //Release motor by setting speed to zero
        shoulder.stop();
      }
      delay(30);
      shoulderState = 0;   //Set shoulder as NOT MOVING
      Serial.print("LEFT JOYSTICK LIFT NEUTRAL!\n\n");
    }
  }
}


void armGripper(int targetState, int gripTime = ARMGRIPTIME) {
  if (runArray[0] == 1) {   //Check if action is enabled in runArray
//    runArray[0] = 0;    //Set runArray flag to prevent repeat operation
    int gripSpeed = 255;   // value: between -255 and 255. It is rarely necessary to change the gripper speed, so it is only a local variable
    Serial.print("\n");
    Serial.println("Arm Gripper...");
    if (targetState == OPEN && armGripState == CLOSE) {   //Check armGripState to prevent uncessary actuation of gripper motor if already fully open.
      Serial.println("Arm Open:");
      Serial.print("Speed: ");
      Serial.println(gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      armGrip.run(gripSpeed); 
      delay(gripTime);
      armGrip.stop();
      armGripState = OPEN;
      Serial.print("armGripState: ");
      Serial.println(armGripState);
      Serial.print("\n");
    }
    delay(1000);
  }
}


void armGripperManual(int targetState, int gripTime = ARMGRIPTIME) {
  if (runArray[0] == 1) {   //Check if action is enabled in runArray
    int gripSpeed = 255;   // value: between -255 and 255. It is rarely necessary to change the gripper speed, so it is only a local variable
    if (targetState == CLOSE) {
      armGrip.run(-gripSpeed); 
    }
    else if(targetState == STOP) {
      armGrip.stop();
      armGripState = CLOSE;
    }
    delay(10);
  }
}


void tailGripper(int targetState, int gripTime = TAILGRIPTIME) {
   if (runArray[5] == 1) {  //Check flag to prevent unnecessary re-triggering of the function
//    runArray[5] = 0;      //Set runArray flag to prevent repeat operation
    int gripSpeed = 128;   // value: between -255 and 255. It is rarely necessary to change the gripper speed, so it is only a local variable
    Serial.println("Tail Gripper...");
    if (targetState == OPEN && tailGripState == 1) {
      Serial.println("Tail Open:");
      Serial.print("Speed: ");
      Serial.println(gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      tailGrip.run(gripSpeed); 
      delay(gripTime);
      tailGripState = 0;
      Serial.print("tailGripState: ");
      Serial.println(tailGripState);
      Serial.print("\n");
      
      //Brake motor
      tailGrip.stop();
      tailGrip.run(-gripSpeed); //Reverse motor direction to brake briefly
      delay(10);
      tailGrip.run(0);    //Release motor by setting speed to zero
      tailGrip.stop();
      tailGripState = 0;
    }
    else if (targetState == CLOSE && tailGripState == 0) {

/**********************************************************
 * OLD CODE: Does not use steps for tail gripper movement
 * **********************************************************
//      Serial.println("Tail Close:");
//      Serial.print("Speed: ");
//      Serial.println(-gripSpeed);
//      Serial.print("Time: ");
//      Serial.println(gripTime);
//      tailGrip.run(-gripSpeed); 
//      delay(gripTime);
//      tailGripState = 1;
//      Serial.print("tailGripState: ");
//      Serial.println(tailGripState);
//      Serial.print("\n");
//      
//      //Brake motor
//      tailGrip.stop();
//      tailGrip.run(gripSpeed); //Reverse motor direction to brake briefly
//      delay(10);
//      tailGrip.run(0);    //Release motor by setting speed to zero
//      tailGrip.stop();
//      tailGripState = 1;
**************************************************************/
      Serial.println("Tail Close:");
      Serial.print("Speed: ");
      Serial.println(-gripSpeed);
      Serial.print("Grip Time: ");
      Serial.println(gripTime);
      int runDelay = 10;
      int gripIterations = floor(gripTime/runDelay);
      Serial.print("Grip Iterations: ");
      Serial.println(gripIterations);
      while(tailGripCount < (gripIterations * 2)) {    //divide gripTime by the amount of time the motor will run each iteration
        tailGrip.run(-gripSpeed); 
        delay(runDelay);
        tailGrip.run(0);
        tailGrip.stop();    //stop motor (without reverse braking) to slightly pause motor to slow it down
        tailGrip.run(gripSpeed); //Reverse motor direction to brake briefly
        delay(runDelay/2);
        tailGrip.run(0);    //Release motor by setting speed to zero
        tailGrip.stop();
        tailGripCount++;
        delay(runDelay);
      }
      //Fully brake motor once gripTime has elapsed (based on derived gripInterval)
      tailGrip.stop();
      tailGrip.run(gripSpeed); //Reverse motor direction to brake briefly
      delay(runDelay);
      tailGrip.run(0);    //Release motor by setting speed to zero
      tailGrip.stop();
      tailGripCount = 0;
      tailGripState = 1;    //update tailgripper state once fully closed
      Serial.print("tailGripState: ");
      Serial.println(tailGripState);
      Serial.print("\n");
    }
    delay(1000);
  }
}
