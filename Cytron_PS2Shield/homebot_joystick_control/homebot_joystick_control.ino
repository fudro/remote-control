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
 * e) Activate Tail Gripper using R Buttons:              (L1 pressed = OPEN, L2 pressed = CLOSE)
 * f) Activate Arm Gripper using R Buttons:               (R1 pressed = OPEN, R2 pressed = CLOSE)
 * g) D-Pad programmatic Arm movement                     (UP = arm to front toward Sonar, DOWN = arm to rear toward Tail Gripper)
 * h) D-Pad programmatic Chassis Movement                 (LEFT = rotate 90 degrees CCW based on IMU, RIGHT = rotate 90 degrees CW based on IMU)
 * EXTRA:
 * e) Target Item                       (TRIANGLE Button)
 * f) Sonar Scan                        (CIRCLE Button)
 * g) Pickup Item                       (SQUARE Button)
 * h) Release Item                      (CROSS Button)
*/

#include <SoftwareSerial.h>
#include "Cytron_PS2Shield.h"
#include "MeMegaPi.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define DEBUG_JOYSTICK

#define IMU   //Use flag when using IMU otherwhise an error will be thrown is the sensor is not connected.
//Set up MegoPi motor ports
MeMegaPiDCMotor armGrip(PORT1A); //Arm Gripper
MeMegaPiDCMotor armWrist(PORT1B); //Wrist
MeMegaPiDCMotor driveRight(PORT2A); //Drive Right, based on the sonar unit being the "front" of the robot
MeMegaPiDCMotor driveLeft(PORT2B); //Drive Left
MeMegaPiDCMotor tailGrip(PORT3A); //Tail Gripper
MeMegaPiDCMotor turnTable(PORT3B); //Turntable
MeMegaPiDCMotor elbow(PORT4A); //Elbow
MeMegaPiDCMotor shoulder(PORT4B); //Shoulder

//Set up IMU sensor (BNO055)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
 
//Create PS2Shield object
//Cytron_PS2Shield ps2(17, 16); // SoftwareSerial: assign Rx and Tx pin
Cytron_PS2Shield ps2; // Create object without paramaters to use the MegaPi hardware Serial2 pins 17(RX) and 16(TX)

//Direction Constants
const int OPEN = 0;
const int CLOSE = 1;
const int STOP = 2;
const int H = 0;    //HORIZONTAL
const int V = 1;    //VERTICAL
const int CW = 0;
const int CCW = 1;
const int FW = 1; //Forward
const int BW = 0; //Backward
//TODO: Add Lidar Pins
//Pin Assignments
const int IMU_RESET = 39;   //This reset functionality does not work yet. The sensor doesn't fully recover after reset because it also needs to be re-initialized.
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
int NUM_SAMPLES = 9;
int SAMPLE_OFFSET = 3;  //number of entries from the max entry of the sorted array
//Potentiometer Limits
const int ELBOW_MIN = 80; //Limit values for sensor
const int ELBOW_MAX = 700;
const int SHOULDER_MIN = 375;
const int SHOULDER_MAX = 600;
//Encoder Limits
const int TURNTABLE_ANALOG_MAX = 875;
const int TURNTABLE_ANALOG_MIN = 650;
const int DRIVE_LEFT_ANALOG_MAX = 700;
const int DRIVE_LEFT_ANALOG_MIN = 400;
const int DRIVE_RIGHT_ANALOG_MAX = 700;
const int DRIVE_RIGHT_ANALOG_MIN = 400;
//State Variables
int triangleState = 0;
int circleState = 0;
int crossState = 0;
int squareState = 0;
int armGripState = 1; //Default to "closed"
int tailGripState = 1;  //Default to "closed"
int wristSwitch = 0;  //store state of wrist switch
int wristState = H;   //current state of wrist (H or V), horizontal direction as default
int turnTableEncoder = 1; //state of turntable encoder. Set initial stae to 1 since this pin is pulled high
int turnTableSwitch = 0;  //state of turntable switch, default state is 0 (LOW)
int turnTableHall = 1;    //state of turntable hall effect sensor, default state is HIGH, sensor pulls pin LOW when active (magnet present)
int driveLeftEncoder = 1; //last state of encoder high(1) or low(0), for comparison
int driveRightEncoder = 1;
//Tracking variables
int turnTableCount = 1;  //store value of turntable position based on encoder ticks. Default to 1 for easy math (15 ticks ~ 90 degrees)
int driveLeftCount = 0;  //store encoder tick value to track movement
int driveRightCount = 0;
int driveLeftAnalog = 0;  //last analog value from drive encoder, raw analog value is used to set current state of encoder
int driveRightAnalog = 0;
int turnTableAnalog = 0;  //last analog value of turntable encoder
int sort_count = 0; //track the last ten selected sonar values
int left_sonar[10];    //arrays to store sonar readings
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

//the runArray[] determines which motor functions are active (0 = inactive, 1 = active). Event if code is called by the main loop, the flag array will prevent code execution if motor is not set to "active" (1)
int runArray[] = {1,  //runArray[0]: armGripper
                  0,  //runArray[1]: wrist
                  0,  //runArray[2]: elbow
                  0,  //runArray[3]: shoulder
                  0,  //runArray[4]: turntable
                  1,  //runArray[5]: tailGripper
                  0,  //runArray[6]: drive
                  0   //runArray[7]: sonar
};

//Joystick Variables
int joystick_left_Y = 0;       //left joystick position in Y axis (FORWARD and BACKWARD)
int joystick_left_X = 0;       //left joystick position in X axis (LEFT and RIGHT)
int joystick_arm = 0;         //right joystick position in Y axis (UP and DOWN). TODO:Direction is inverted - pull back to lift UP
int lift_speed = 0;
int turn_speed = 0;
int commandState = 0;         //Track if a command is currently being executed

/***********************************
 * FUNCTION PROTOTYPES
***********************************/
void armGripper(int gripState, int gripTime = ARMGRIPTIME); //gripState is OPEN or CLOSE
void armGripperManual(int gripState, int gripTime = ARMGRIPTIME); //for remote manual operation
void wristRotate (int targetState, int wristDirection = CW, float wristRevolution = 0.0, int wristSpeed = 255);  //targetState is (V)ertical or (H)orizontal, wristRevolution is the number of full revolutions to be perfomed.
void elbowMove(int elbowPosition = 500, int elbowSpeed = 65);  //approximate center position and preferred default speed.
void shoulderMove(int shoulderPosition = 575, int shoulderSpeed = 127); //approximate center position and preferred default speed.
void turnTableMove(int turnDegrees = 0, int turnDirection = CW, int turnSpeed = 65);    //turnDegrees is the degrees of angular rotation from the current position
void tailGripper(int gripState, int gripTime = TAILGRIPTIME); //gripState is OPEN or CLOSE
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
  digitalWrite(IMU_RESET, HIGH);  //pull IMU reset pin low to trigger reset
  pinMode(SONAR_TRIGGER, OUTPUT);
  digitalWrite(SONAR_TRIGGER, LOW);
  //Set up serial communications
  Serial.begin(115200);
  Serial.println("HomeBot Remote Control Test!");
  delay(1000);
  Serial.println("StartingTest in 3 seconds...");
  delay(3000);
  #ifdef IMU
  //IMU Sensor (BNO055) This sensor uses I2C so no pin additional assignments are necessary 
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* Check if there was a problem detecting the BNO055 ... display a warning */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);   //advanced setup (I don;t know what this actually does)
  #endif
  
  ps2.begin(115200);          //Start remote control shield and set baud rate (baudrate must be the same as the jumper setting at PS2 shield)
  Serial.println("Remote Control Ready!");

  Serial2.begin(115200);       //Set serial channel for remote control
}

void loop()
{ 
  /***************************
      TRIGGERS
   ***************************/
  //L1 and L2 Triggers (Used for Tail Gripper)
  if (ps2.readButton(PS2_LEFT_1) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_LEFT_1) == 0 && tailGripState == 1) { //debounce button by double checking buttonstate
      Serial.println("L1 Pressed!");
      tailGripper(OPEN);
    }
  }
  if(ps2.readButton(PS2_LEFT_2) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_LEFT_2) == 0 && tailGripState == 0) { //double check buttonstate after short delay to prevent false trigger. also check if trimmer is below lower threshold.
      Serial.println("L2 Pressed!");
      tailGripper(CLOSE);
    }
  }

   
  //R1 and R2 Triggers (Used for Arm Gripper)
  if(ps2.readButton(PS2_RIGHT_1) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_RIGHT_1) == 0 && armGripState == 1) { //double check buttonstate after short delay to prevent false trigger
      Serial.println("R1 Pressed!");
      armGripper(OPEN);
    }
  }
  if(ps2.readButton(PS2_RIGHT_2) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_RIGHT_2) == 0) { //double check buttonstate after short delay to prevent false trigger
      Serial.println("R2 Pressed!");
      armGripperManual(CLOSE);
    }
  }
  else if(ps2.readButton(PS2_RIGHT_2) == 1)
  {
    armGripperManual(STOP);
  }

  /****************************
      BUTTONS
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
      JOYSTICKS
  ****************************/
  //LEFT joystick position. Value of 128 is center.
  joystick_left_Y = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD is positive)
  joystick_left_X = ps2.readButton(PS2_JOYSTICK_LEFT_X_AXIS) - 128;  //Get joystick difference from center position (RIGHT is positive)
  
  if (joystick_left_Y > 50 || joystick_left_Y < -50) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD. Defaults to 10.).
    lift_speed = map(joystick_left_Y, 0, 128, 0, 255);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
  }
  else {
    lift_speed = 0;
  }
  if (joystick_left_X > 50 || joystick_left_X < -50) { 
    turn_speed = map(joystick_left_X, 0, 128, 0, 255); 
  }  
  else {
    turn_speed = 0;
  }

  if (lift_speed != 0 || turn_speed != 0) {
    Serial.print("Left Joystick: ");
    //MOVING FORWARD
    if (lift_speed > 0) {   //Check FORWARD/BACK direction of joystick (FORWARD greater than zero and moves that shoulder downward)
      Serial.print("Forward! ");
    }
    //MOVING BACKWARD
    else if (lift_speed < 0) {
      Serial.print("Backward! ");
    }
  
    //TURNING CW
    if (turn_speed > 0) {   //Check FORWARD/BACK direction of joystick (FORWARD greater than zero and moves that shoulder downward)
      Serial.print("Right! ");
    }
    //TURNING CCW
    else if (turn_speed < 0) {
      Serial.print("Left! ");
    }
    
    #ifdef DEBUG_JOYSTICK
    Serial.print("\t");
    Serial.print ("Lift: ");
    Serial.print (joystick_left_Y);
    Serial.print ("\t");
    Serial.print ("lift_speed: ");
    Serial.print (lift_speed);
    Serial.print ("\t");
    Serial.print ("Horizontal: ");
    Serial.print (joystick_left_X);
    Serial.print ("\t");
    Serial.print ("turn_speed: ");
    Serial.print (turn_speed);
    Serial.print ("\n");
    #endif
  }
}

  

void armGripper(int targetState, int gripTime = ARMGRIPTIME) {
  if (runArray[0] == 1) {   //Check if action is enabled in runArray
//    runArray[0] = 0;    //Set runArray flag to prevent repeat operation
    int gripSpeed = 255;   // value: between -255 and 255. It is rarely necessary to change the gripper speed, so it is only a local variable
    Serial.print("\n");
    Serial.println("Arm Gripper...");
    if (targetState == OPEN) {
      Serial.println("Arm Open:");
      Serial.print("Speed: ");
      Serial.println(gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      armGrip.run(gripSpeed); 
      delay(gripTime);
      armGrip.stop();
      Serial.print("armGripState: ");
      Serial.println(armGripState);
      Serial.print("\n");
    }
    else if (targetState == CLOSE && armGripState == 0) {
      Serial.println("Arm Close:");
      Serial.print("Speed: ");
      Serial.println(-gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      armGrip.run(-gripSpeed); 
      delay(gripTime);
      armGrip.stop();
      armGripState = 1;
      Serial.print("armGripState: ");
      Serial.println(armGripState);
      Serial.print("\n");
    }
    delay(1000);
  }
}


void armGripperManual(int targetState, int gripTime = ARMGRIPTIME) {
  if (runArray[0] == 1) {   //Check if action is enabled in runArray
//    runArray[0] = 0;    //Set runArray flag to prevent repeat operation
    int gripSpeed = 255;   // value: between -255 and 255. It is rarely necessary to change the gripper speed, so it is only a local variable
    if (targetState == CLOSE) {
      armGrip.run(-gripSpeed); 
    }
    else if(targetState == STOP) {
      armGrip.stop();
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
      Serial.println("Tail Close:");
      Serial.print("Speed: ");
      Serial.println(-gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      tailGrip.run(-gripSpeed); 
      delay(gripTime);
      tailGripState = 1;
      Serial.print("tailGripState: ");
      Serial.println(tailGripState);
      Serial.print("\n");
      
      //Brake motor
      tailGrip.stop();
      tailGrip.run(gripSpeed); //Reverse motor direction to brake briefly
      delay(10);
      tailGrip.run(0);    //Release motor by setting speed to zero
      tailGrip.stop();
      tailGripState = 1;
    }
    delay(1000);
  }
}
