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

//#define DEBUG_JOYSTICK

#define IMU   //Use flag when using IMU otherwhise an error will be thrown if the sensor is not connected.
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
const int ELBOW_MIN = 100; //Limit values for sensor
const int ELBOW_MAX = 700;
const int SHOULDER_MIN = 375;
const int SHOULDER_MAX = 600;
//Encoder Analog Limits
//The encoders use the analog value from the opto-coupler sensor board. 
//The Min/Max constants represent the "highest" and "lowest" analog values returned from the sensor. (These values can vary by sensor board and must be determined through testing.)
//When the sensor value goes above or below these values, it corresponds to the sensor states of "on" or "off" as the encoder disc passes through the opto-coupler.
//The resulting changes in state are counted as an encoder "ticks".
const int TURNTABLE_ANALOG_MAX = 875;   
const int TURNTABLE_ANALOG_MIN = 650;
const int DRIVE_LEFT_ANALOG_MAX = 700;
const int DRIVE_LEFT_ANALOG_MIN = 400;
const int DRIVE_RIGHT_ANALOG_MAX = 700;
const int DRIVE_RIGHT_ANALOG_MIN = 400;
//State Variables
int triangleState = 0;    //remote control button states
int circleState = 0;
int crossState = 0;
int squareState = 0;
int armGripState = 1; //Default to "closed"
int tailGripState = 1;  //Default to "closed"
int wristSwitch = 0;  //state of wrist switch
int wristState = H;   //state of wrist orientation (H or V), horizontal direction as default
int turnTableEncoder = 1; //state of turntable encoder as high(1) or low(0). Set initial state to 1 since this input pin is pulled high
int turnTableSwitch = 0;  //state of turntable switch, default state is 0 (LOW)
int turnTableHall = 1;    //state of turntable hall effect sensor, default state is HIGH, sensor pulls pin LOW when active (magnet present)
int driveLeftEncoder = 1; //last state of encoder high(1) or low(0), for comparison with current state read from sensor
int driveRightEncoder = 1;
//Tracking variables
int turnTableCount = 1;  //store encoder tick value of turntable position based on encoder ticks. Default to 1 for easier calculations (15 ticks ~ 90 degrees)
int driveLeftCount = 0;  //store encoder tick value to track movement
int driveRightCount = 0;
int driveLeftAnalog = 0;  //last analog value from drive encoder, raw analog value is compared with the Min/Max constants to set current state of encoder
int driveRightAnalog = 0;
int turnTableAnalog = 0;  //last analog value of turntable encoder
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
//Even if code is called by the main loop, the flag array will prevent code execution if motor is not set to "active".
int runArray[] = {1,  //runArray[0]: armGripper
                  0,  //runArray[1]: wrist
                  1,  //runArray[2]: elbow
                  0,  //runArray[3]: shoulder
                  0,  //runArray[4]: turntable
                  1,  //runArray[5]: tailGripper
                  0,  //runArray[6]: drive
                  0   //runArray[7]: sonar
};

//Joystick Variables
int joystick_left_Y = 0;      //left joystick position in Y axis (FORWARD and BACKWARD)
int joystick_left_X = 0;      //left joystick position in X axis (LEFT/CCW and RIGHT/CW)
//int joystick_arm = 0;         //right joystick position in Y axis (UP and DOWN). TODO:Direction is inverted - pull back to lift UP
int shoulder_lift_speed = 0;  //speed at which the arm "shoulder" moves up and down
int shoulder_turn_speed = 0;
//int commandState = 0;         //Track if a command is currently being executed

/***********************************
 * FUNCTION PROTOTYPES - Function prototypes are required to allow function with parameters that can have deafult values.
***********************************/
void armGripper(int gripState, int gripTime = ARMGRIPTIME); //gripState is OPEN (0) or CLOSE (1)
void armGripperManual(int gripState, int gripTime = ARMGRIPTIME); //special version of arm gripper funtion for manual remote control operation
void wristRotate (int targetState, int wristDirection = CW, float wristRevolution = 0.0, int wristSpeed = 255);  //targetState is (V)ertical or (H)orizontal, wristRevolution is the number of full revolutions to be perfomed.
void elbowMove(int elbowPosition = 500, int elbowSpeed = 65);  //default parameter values are approximate center position and preferred default speed.
void shoulderMove(int shoulderPosition = 575, int shoulderSpeed = 127); //default parameter values are approximate center position and preferred default speed.
void turnTableMove(int turnDegrees = 0, int turnDirection = CW, int turnSpeed = 65);    //turnDegrees is the degrees of angular rotation from the current position
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
  delay(1000);
  bno.setExtCrystalUse(true);   //advanced setup (I don't know what this actually does)
  #endif
  
  ps2.begin(115200);          //Start remote control shield and set baud rate (baudrate must be the same as the jumper setting at PS2 shield)
  Serial.println("Remote Control Ready!");

  Serial2.begin(115200);       //Set MegaPi serial channel for remote control communication
}

void loop()
{ 
  /***************************
      TRIGGERS
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
  joystick_left_Y = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD/ShoulderDown is positive)
  joystick_left_X = ps2.readButton(PS2_JOYSTICK_LEFT_X_AXIS) - 128;  //Get joystick difference from center position (RIGHT/RotateCW is positive)

  //Shoulder DOWN (joystick forward) and UP (joystick backward)
  if (joystick_left_Y > 50 || joystick_left_Y < -50) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for DOWN and UP. Defaults to 50 for both.).
    shoulder_lift_speed = map(joystick_left_Y, 0, 128, 0, 255);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
  }
  else {
    shoulder_lift_speed = 0;
  }
  //Shoulder Rotate CCW (joystick left) and CCW (joystick right)
  if (joystick_left_X > 50 || joystick_left_X < -50) { 
    shoulder_turn_speed = map(joystick_left_X, 0, 128, 0, 255); 
  }  
  else {
    shoulder_turn_speed = 0;
  }
  //Only activate shoulder motor if the LEFT joystick is outside of deadzone AND the motor speeds have been set to a non-zero value.
  if (shoulder_lift_speed != 0) {
    Serial.print("Left Joystick: ");
    //MOVING DOWN (Joystick Forward)
    if (shoulder_lift_speed > 0) {   //Check FORWARD/BACK direction of joystick (FORWARD is greater than zero and moves the shoulder DOWN)
      Serial.print("Down! \n");
      elbowMove();
    }
    //MOVING UP (Joystick Backward)
    else if (shoulder_lift_speed < 0) {
      Serial.print("Up! \n");
    }
    else {
      //Stop
      elbowMove(STOP);
      Serial.print("JOYSTICK NEUTRAL!!\n");
    }
  }
  else if(shoulder_turn_speed != 0) {
    //TURNING CW  (Joystick Right)
    if (shoulder_turn_speed > 0) {   //Check LEFT/RIGHT direction of joystick (RIGHT is greater than zero and moves the shoulder Clockwise)
      Serial.print("Right! ");
    }
    //TURNING CCW (Joystick Left)
    else if (shoulder_turn_speed < 0) {
      Serial.print("Left! ");
    }
    else {
      //stop
    }
    //Display additional information about the joystick input
    #ifdef DEBUG_JOYSTICK
    Serial.print("\t\t");
    Serial.print ("Lift: ");
    Serial.print (joystick_left_Y);
    Serial.print ("\t\t");
    Serial.print ("shoulder_lift_speed: ");
    Serial.print (shoulder_lift_speed);
    Serial.print ("\t\t");
    Serial.print ("Horizontal: ");
    Serial.print (joystick_left_X);
    Serial.print ("\t\t");
    Serial.print ("shoulder_turn_speed: ");
    Serial.print (shoulder_turn_speed);
    Serial.print ("\n");
    #endif
  }
}


void elbowMove(int elbowPosition = 500, int elbowSpeed = 65) { //Default values allow the function to be called without arguments to reset to a default position (at the default speed).
  if(runArray[2] == 1) {    //Check if movement is allowed
    if(elbowPosition >= ELBOW_MIN && elbowPosition <= ELBOW_MAX) {   //Check if command value is within allowed range
      int elbowState = 0; //Initialize elbow as NOT MOVING
      int lastPosition = analogRead(ELBOW_POT);   //read encoder position
      if(lastPosition - 1 > ELBOW_MIN) {  //check if elbow has reached lower limit
        elbowState = 1;   //Set elbow as MOVING
        Serial.print("\n");
        Serial.println("Elbow Down");
        Serial.print("Target Position: ");
        Serial.print(lastPosition + 1);
        Serial.print("\n\n");
//        while(elbowPosition < lastPosition) {   //while target positiion is still lower than the last read position
//          elbow.run(-elbowSpeed);   //set motor direction to move elbow "down"
//          delay(10);               //wait for small amount of elbow movement
//          lastPosition = analogRead(ELBOW_POT);   //get new position
//          Serial.print("Elbow Position: ");
//          Serial.println(lastPosition);
//        }
      }
      else if(lastPosition - 1 <= ELBOW_MIN && elbowState == 1) {    //only stop motor is running. This prevent the reverse braking method to cause unnecessary jitter in the motor when there is no change in state.
        //Brake motor once target position is reached
        elbow.stop();
        elbow.run(elbowSpeed); //Reverse motor direction to brake briefly
        delay(30);
        elbow.run(0);    //Release motor by setting speed to zero
        elbow.stop();
        elbowState = 0;   //Set elbow as NOT MOVING
        Serial.print("Stopped Inside!");
      }
    }
    else if(elbowPosition == STOP) {
      //Brake motor once target position is reached
      elbow.stop();
      elbow.run(elbowSpeed); //Reverse motor direction to brake briefly
      delay(30);
      elbow.run(0);    //Release motor by setting speed to zero
      elbow.stop();
      Serial.print("Stopped Outside!");
    }
//      else if(elbowPosition > lastPosition) {  //If the desired postion is physically HIGHER than the last read position.
//        Serial.print("\n");
//        Serial.println("Elbow Up");
//        Serial.print("Target Position: ");
//        Serial.print(elbowPosition);
//        Serial.print("\n\n");
//        while(elbowPosition > lastPosition) {
//          elbow.run(elbowSpeed);
//          delay(10);
//          lastPosition = analogRead(ELBOW_POT);
//          Serial.print("Elbow Position: ");
//          Serial.println(lastPosition);
//        }
//        //Brake motor once target position is reached
//        elbow.stop();
//        elbow.run(-elbowSpeed); //Reverse motor direction to brake briefly
//        delay(30);
//        elbow.run(0);    //Release motor by setting speed to zero
//        elbow.stop();
//      }
//      runArray[2] = 0;
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
//    runArray[0] = 0;    //Set runArray flag to prevent repeat operation
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
