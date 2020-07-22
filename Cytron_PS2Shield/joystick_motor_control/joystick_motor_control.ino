/*
 * This program accepts input from a game controller to control a robot.
 * 
 * HARDWARE:
 * Arduino Uno R3 Clone (ATMega328P)
 * Sony PS2 Wireless Force 2 Controller
 * Cytron PS2 Shield: http://www.cytron.com.my/p-shield-ps2
 * Adafruit Motor Shield V2.3: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/overview
 * Drive Motors: Pololu High Power 6V DC Motor 20.4:1 Gear Ratio
 * Stepper Motor: 28BYJ-48 unipolar
 * 
 * CONTROL FEATURES:
 * WORKING:
 * a) Drive control using LEFT joystick:        (Y-Axis = drive speed, X-Axis = turn speed)
 * b) Arm control using RIGHT joystick:         (Y-Axis = UP - pull back / DOWN - push forward)
 * c) Activate Weed Trimmer using R2:           (R2 pressed = ON, R2 released = OFF)
 * d) D-Pad programmatic drive test             (UP direction = drive forward for 5 seconds, motor adjustment values for "straight" travel. Left:200, Right:215)
 * TODO:
 * e) Pump Air                                  (TRIANGLE Button)
 * f) Open Scoop                                (CIRCLE Button)
 * g) Close Scoop                               (SQUARE Button)
 * h) Pickup Item (AUTOMATED)                   (CROSS Button)
 * NOTES:
 * Currently, for testing, the pump and valve can be manually activated using two button combinations:
 * Pump:  TRIANGLE, CIRCLE.
 * Valve: SQUARE, CROSS
*/

#include <SoftwareSerial.h>
#include "Cytron_PS2Shield.h"
#include <Adafruit_MotorShield.h>
#include <Stepper.h>
#include "TimerOne.h"

#define DRIVE_MOTORS
#define ARM_MOTOR
//#define DEBUG_DRIVE
#define DEBUG_ENCODER
#define DEBUG_ARM
#define DEBUG_DPAD
 
//Create PS2Shield object
Cytron_PS2Shield ps2(8, 9); // SoftwareSerial: assign Rx and Tx pin
//Cytron_PS2Shield ps2; // Create object without paramaters to use the default hardware serial pins 0(RX) and 1(TX)
// Create the motor shield object (with the default I2C address)
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Create DC motors on ports M1, M2, and M4 (refer to the motor shield PCB screenprint - choice of motors based on physical location due to connecting wire length.)
Adafruit_DCMotor *Motor_Left = AFMS.getMotor(1);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(2);
Adafruit_DCMotor *Motor_Arm = AFMS.getMotor(4);

int joystick_drive = 0;       //left joystick position in Y axis (FORWARD and BACKWARD)
int joystick_steer = 0;       //left joystick position in X axis (LEFT and RIGHT)
int last_drive_speed = 0;  //store last value for comparison - used to prevent huge jumps in motor current due to rapid joystick changes which can overload buck converter. 
int ramp_step = 7;            //maximum ammount the motor speed can change (up or down) per loop iteration
int joystick_arm = 0;         //right joystick position in Y axis (UP and DOWN). TODO:Direction is inverted - pull back to lift UP
int arm_scoop = 75;             //lowest mechanically safe position for the arm (may vary depending on current attachment
int arm_extend = 250;         //best position to extend arm enough for clean dump of object
int arm_up = 400;            //highest mechanically safe position for the arm (may vary depending on current attachment
float drive_speed = 0.0;     //base speed component in the FORWARD/BACKWARD direction - derived from "joystick_drive"
float turn_speed = 0.0;      //speed adjustment value for left and right wheels while turning
float arm_speed = 0.0;        //speed of arm motor
float motor_speed_left = 0.0;   //calculated speed for LEFT wheels of robot (composed of drive_speed and turn_speed)
float motor_speed_right = 0.0;  //calculated speed for RIGHT wheels of robot (composed of drive_speed and turn_speed)
float motor_speed_arm = 0.0;
int relay_1 = 10;             //pin connected to relay input 1 (IN1)
int relay_2 = 11;             //pin connected to relay input 2 (IN2)
int relay_3 = 12;             //pin connected to relay input 3 (IN3)
int relay_4 = 13;             //pin connected to relay input 4 (IN4)
int arm_pot = A0;             //sensor feedback for arm position
const float STEPS_PER_REV = 32;   //Number of steps for one revolution of the internal drive shaft
const float GEAR_REDUCTION = 64;  //Internal stepper motor gear reduction of 64 turns of internal shaft to 1 turn of output shaft
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_REDUCTION;   //Number of steps required to rotate the output shaft 1 revolution
int stepsRequired;        //Number of steps required for the current command
int stepperState = 0;     //Track current position of stepper motor with regards to teh switch it is activating
int commandState = 0;     //Track if a command is currently being executed
Stepper stepperMotor(STEPS_PER_REV, 4, 6, 5, 7);    //Define stepper motor and the pins used to control the coils

//Encoders
const byte MOTOR1 = 2;  //assign hardware interrupt pins to each motor
const byte MOTOR2 = 3;
//IMPORTANT: Variables that need to be modified within an ISR MUST be declared as "volatile". See Arduino reference listed above.
volatile int encoder1 = 0; //variables to count encoder disc "ticks" each time the encoder circuitry sends an interrupt pulse. 
volatile int encoder2 = 0;

/**************INTERRUPT SERVICE ROUTINES******************/
//Motor1 pulse count ISR
void ISR_encoder1() {
  encoder1++;
}

//Motor2 pulse count ISR
void ISR_encoder2() {
  encoder2++;
}

//TimerOne ISR
void ISR_timerone() {
  Timer1.detachInterrupt();   //Stop Timer1 to allow time for serial print out
  #ifdef DEBUG_ENCODER
  Serial.print("encoder1: ");
  Serial.print(encoder1);
  Serial.print("\t");
  Serial.print("encoder2: ");
  Serial.println(encoder2);
  #endif
  Timer1.attachInterrupt(ISR_timerone);
}

void setup()
{
  pinMode(relay_1, OUTPUT);   //Set all relay pins as outputs
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);

  pinMode(MOTOR1, INPUT);
  pinMode(MOTOR2, INPUT);
  
  ps2.begin(115200);          //Start remote control shield and set baud rate (baudrate must be the same as the jumper setting at PS2 shield)
  Serial.begin(115200);       //Set baudrate for serial monitor
  Serial.println("Robot Start!");
  
  AFMS.begin();               //Start motorshield object (with the default frequency of 1.6KHz)
  #ifdef DRIVE_MOTORS
  Motor_Left->setSpeed(255);  //Initialize Motors (max motor speed is 255)
  Motor_Left->run(FORWARD);
  Motor_Left->run(RELEASE);
  Motor_Right->setSpeed(255);
  Motor_Right->run(FORWARD);
  Motor_Right->run(RELEASE);
  Motor_Arm->setSpeed(255);
  Motor_Arm->run(FORWARD);
  Motor_Arm->run(RELEASE);
  #endif

  attachInterrupt(digitalPinToInterrupt(MOTOR1), ISR_encoder1, RISING);   //Attach interrupt service routines to hardware interrupt pins and set trigger mode.
  attachInterrupt(digitalPinToInterrupt(MOTOR2), ISR_encoder2, RISING);
  Timer1.initialize(200000);   //Timer1 accepts parameter in microseconds. Set to one million for 1 second.
  Timer1.attachInterrupt(ISR_timerone);   //Enable the timer
}

void loop()
{ 
  /***************************
            D-PAD
   **************************/
  #ifdef DEBUG_DPAD
  //UP
  if(ps2.readButton(PS2_UP) == 0)   // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_UP) == 0 && commandState == 0) { //Double check button press after delay
      Serial.println("UP Pressed!");
      encoder1 = 0;               //Reset encoders
      encoder2 = 0;
      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
      
      #ifdef DRIVE_MOTORS
      //Start motors
      Motor_Left->run(BACKWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(BACKWARD);
      int i=0;
      for (i=0; i<255; i++) {   //Ramp up power to max speed
        Motor_Left->setSpeed(i);
        Motor_Right->setSpeed(i);  
        delay(10);
      }
      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
      delay(5000);    //Drive for 5 seconds
      Motor_Left->setSpeed(0);
      Motor_Right->setSpeed(0);
      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(RELEASE);
      #endif
      
      commandState = 0;
    }
  }

  //RIGHT
  if(ps2.readButton(PS2_RIGHT) == 0)   // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_RIGHT) == 0 && commandState == 0) { //Double check button press after delay
      Serial.println("UP Pressed!");
      encoder1 = 0;               //Reset encoders
      encoder2 = 0;
      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
      
      #ifdef DRIVE_MOTORS
      //Start motors
      Motor_Left->run(BACKWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(FORWARD);
      int i=0;
      for (i=0; i<255; i++) {   //Ramp up power to max speed
        Motor_Left->setSpeed(i);
        Motor_Right->setSpeed(i);  
        delay(10);
      }
      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
      while(encoder1 < 80) {   //measure using the wheel on the outside of the turn (e.g. Left Wheel for righthand turns)
        //wait for encoder-based turn to complete 
        //Value of 80 is good for a 90 degree encoder-based turn
        //Value of 170 is good for a 180 degree encoder-based turn
      }
//      delay(500);    //Wait to complete 90 degree time-based turn
//      delay(2750);    //Wait to complete 180 degree time-based turn
      Motor_Left->setSpeed(0);
      Motor_Right->setSpeed(0);
      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(RELEASE);
      #endif
      
      commandState = 0;
    }
  }

  //DOWN
  if(ps2.readButton(PS2_DOWN) == 0)   // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_DOWN) == 0 && commandState == 0) { //Double check button press after delay
      Serial.println("DOWN Pressed!");
      encoder1 = 0;               //Reset encoders
      encoder2 = 0;
      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
      
      #ifdef DRIVE_MOTORS
      //Start motors
      Motor_Left->run(FORWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(FORWARD);
      int i=0;
      for (i=0; i<255; i++) {   //Ramp up power to max speed
        Motor_Left->setSpeed(i);
        Motor_Right->setSpeed(i);  
        delay(10);
      }
      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
      delay(5000);    //Drive for 5 seconds
      Motor_Left->setSpeed(0);
      Motor_Right->setSpeed(0);
      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(RELEASE);
      #endif
      
      commandState = 0;
    }
  }

  //LEFT
  if(ps2.readButton(PS2_LEFT) == 0)   // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_LEFT) == 0 && commandState == 0) { //Double check button press after delay
      Serial.println("LEFT Pressed!");
      encoder1 = 0;               //Reset encoders
      encoder2 = 0;
      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
      
      #ifdef DRIVE_MOTORS
      //Start motors
      Motor_Left->run(FORWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(BACKWARD);
      int i=0;
      for (i=0; i<255; i++) {   //Ramp up power to max speed
        Motor_Left->setSpeed(i);
        Motor_Right->setSpeed(i);  
        delay(10);
      }
      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
      while(encoder2 < 80) {   //measure using the wheel on the outside of the turn (e.g. Left Wheel for righthand turns)
        //wait for encoder-based turn to complete 
        //Value of 80 is good for a 90 degree encoder-based turn
        //Value of 170 is good for a 180 degree encoder-based turn
      }
//      delay(500);    //Wait to complete 90 degree time-based turn
//      delay(2750);    //Wait to complete 180 degree time-based turn
      Motor_Left->setSpeed(0);
      Motor_Right->setSpeed(0);
      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(RELEASE);
      #endif
      
      commandState = 0;
    }
  }
  #endif
  
  /***************************
      TRIGGERS - STEPPER MOTOR
   ***************************/
  //R2 Trigger
  if(ps2.readButton(PS2_RIGHT_2) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_RIGHT_2) == 0 && stepperState == 0) { //double check buttonstate after short delay to prevent false trigger
      //Rotate CW
      stepperState = 1;
      stepsRequired = STEPS_PER_OUT_REV/2;    //Rotate a fraction of a revolution. Change divisor value to adjust amount of rotation. Change sign to adjust direction of rotation.
      stepperMotor.setSpeed(700);
      stepperMotor.step(stepsRequired);
    }
  }
  else if (stepperState == 1) {
    //Rotate CCW
    stepsRequired = -STEPS_PER_OUT_REV/2;
    stepperMotor.setSpeed(700);
    stepperMotor.step(stepsRequired);
    stepperState = 0;
  }
  
  /****************************
      BUTTONS
  ****************************/
//  //Triangle
//  if(ps2.readButton(PS2_TRIANGLE) == 0) // 0 = pressed, 1 = released
//  {
//    delay(10);
//    if(ps2.readButton(PS2_TRIANGLE) == 0) { //double check button to prevent false trigger
//      digitalWrite(10, LOW);
//    }
//  }
//  else
//  {
//    digitalWrite(10, HIGH);
//  }
//  
//  //Circle
//  if(ps2.readButton(PS2_CIRCLE) == 0) // 0 = pressed, 1 = released
//  {
//    delay(10);
//    if(ps2.readButton(PS2_CIRCLE) == 0) { //double check button to prevent false trigger
//      digitalWrite(11, LOW);
//    }
//  }
//  else
//  {
//    digitalWrite(11, HIGH);
//  }
//  
//  //Cross
//  if(ps2.readButton(PS2_CROSS) == 0) // 0 = pressed, 1 = released
//  {
//    delay(10);
//    if(ps2.readButton(PS2_CROSS) == 0) { //double check button to prevent false trigger
//      digitalWrite(12, LOW);
//    }
//  }
//  else
//  {
//    digitalWrite(12, HIGH);
//  }
//  
//  //Square
//  if(ps2.readButton(PS2_SQUARE) == 0) // 0 = pressed, 1 = released
//  {
//    delay(10);
//    if(ps2.readButton(PS2_SQUARE) == 0) { //double check button to prevent false trigger
//      digitalWrite(13, LOW);
//    }
//  }
//  else
//  {
//    digitalWrite(13, HIGH);
//  }

  //TRIANGLE - Pick Up
  if(ps2.readButton(PS2_TRIANGLE) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_TRIANGLE) == 0) { //double check button to prevent false trigger
      Serial.println("Picking Up!");
      //Pump Air
      digitalWrite(10, LOW);    //Close both relays (+,-) required to activate pump
      digitalWrite(11, LOW);
      delay(5000);   //Wait for pressure to build up
      
      //Move Forward
      encoder1 = 0;       //Reset encoders to track movement
      encoder2 = 0;
      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
      //Start motors
      #ifdef DRIVE_MOTORS
      Motor_Left->run(BACKWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(BACKWARD);
      int i=0;
      for (i=0; i<255; i++) {   //Ramp up power to max speed
        Motor_Left->setSpeed(i);
        Motor_Right->setSpeed(i);  
        delay(10);
      }
      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
      delay(500);    //Wait for robot to drive forward to position scoop over object
      Motor_Left->setSpeed(0);
      Motor_Right->setSpeed(0);
      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(RELEASE);
      #endif
      
      //Lower Arm
      #ifdef ARM_MOTOR
      Motor_Arm->setSpeed(255);
      if(arm_pot < arm_scoop) {
        while(arm_pot < arm_scoop) {    //if arm is below midpoint...
          Motor_Arm->run(FORWARD);   //Run motor in direction to RAISE arm to midpoint
        }
      }
      else if(arm_pot > arm_scoop) {    //if arm is above midpoint...
        while(arm_pot > arm_scoop) {
          Motor_Arm->run(BACKWARD);   //Run motor in direction to LOWER arm to midpoint
        }
      }
      Motor_Arm->setSpeed(0);   //Stop arm at scooping position
      Motor_Arm->run(RELEASE);
      #endif

      //Release Valve
      digitalWrite(12, LOW);
      digitalWrite(13, LOW);
      delay(500);
      digitalWrite(12, HIGH);
      digitalWrite(13, HIGH);

      //Raise Arm
      #ifdef ARM_MOTOR
      Motor_Arm->setSpeed(255);
      if(arm_pot < arm_up) {
        while(arm_pot < arm_up) {    //if arm is below midpoint...
          Motor_Arm->run(FORWARD);   //Run motor in direction to RAISE arm to midpoint
        }
      }
      else if(arm_pot > arm_up) {    //if arm is above midpoint...
        while(arm_pot > arm_up) {
          Motor_Arm->run(BACKWARD);   //Run motor in direction to LOWER arm to midpoint
        }
      }
      Motor_Arm->setSpeed(0);   //Stop arm at scooping position
      Motor_Arm->run(RELEASE);
      #endif
      
      //Move Backward
      encoder1 = 0;       //Reset encoders to track movement
      encoder2 = 0;
      //Start motors
      #ifdef DRIVE_MOTORS
      Motor_Left->run(FORWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(FORWARD);
      i=0;    //Reuse previously declared iterator variable and reset it.
      for (i=0; i<255; i++) {   //Ramp up power to max speed
        Motor_Left->setSpeed(i);
        Motor_Right->setSpeed(i);  
        delay(10);
      }
      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
      delay(500);    //Wait for robot to drive backward to view if object was picked up
      Motor_Left->setSpeed(0);
      Motor_Right->setSpeed(0);
      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(RELEASE);
      #endif
      
      commandState = 0;   //Reset flag
    }
  }
  
  //CIRCLE - Pump Air
  if(ps2.readButton(PS2_CIRCLE) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_CIRCLE) == 0) { //double check button to prevent false trigger
      digitalWrite(10, LOW);
      digitalWrite(11, LOW);
    }
  }
  else
  {
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);
  }
  
  //CROSS - Dump Object
  if(ps2.readButton(PS2_CROSS) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_CROSS) == 0) { //double check button to prevent false trigger
      
      //Move Forward
      encoder1 = 0;       //Reset encoders to track movement
      encoder2 = 0;
      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
      //Start motors
      #ifdef DRIVE_MOTORS
      Motor_Left->run(BACKWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(BACKWARD);
      int i=0;
      for (i=0; i<255; i++) {   //Ramp up power to max speed
        Motor_Left->setSpeed(i);
        Motor_Right->setSpeed(i);  
        delay(10);
      }
      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
      delay(500);    //Wait for robot to drive forward to position scoop over object
      Motor_Left->setSpeed(0);
      Motor_Right->setSpeed(0);
      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(RELEASE);
      #endif

      //Lower Arm
      #ifdef ARM_MOTOR
      Motor_Arm->setSpeed(255);
      if(arm_pot < arm_extend) {
        while(arm_pot < arm_extend) {    //if arm is below midpoint...
          Motor_Arm->run(FORWARD);   //Run motor in direction to RAISE arm to midpoint
        }
      }
      else if(arm_pot > arm_extend) {    //if arm is above midpoint...
        while(arm_pot > arm_extend) {
          Motor_Arm->run(BACKWARD);   //Run motor in direction to LOWER arm to midpoint
        }
      }
      Motor_Arm->setSpeed(0);   //Stop arm at scooping position
      Motor_Arm->run(RELEASE);
      #endif

      //Pump Air
      digitalWrite(10, LOW);
      digitalWrite(11, LOW);
      delay(3000);    //Wait for enough pressure to open scoop
      digitalWrite(10, HIGH);
      digitalWrite(11, HIGH);

      //Raise Arm
      #ifdef ARM_MOTOR
      Motor_Arm->setSpeed(255);
      if(arm_pot < arm_up) {
        while(arm_pot < arm_up) {    //if arm is below midpoint...
          Motor_Arm->run(FORWARD);   //Run motor in direction to RAISE arm to midpoint
        }
      }
      else if(arm_pot > arm_up) {    //if arm is above midpoint...
        while(arm_pot > arm_up) {
          Motor_Arm->run(BACKWARD);   //Run motor in direction to LOWER arm to midpoint
        }
      }
      Motor_Arm->setSpeed(0);   //Stop arm at scooping position
      Motor_Arm->run(RELEASE);
      #endif
      
      //Move Backward
      encoder1 = 0;       //Reset encoders to track movement
      encoder2 = 0;
      //Start motors
      #ifdef DRIVE_MOTORS
      Motor_Left->run(FORWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(FORWARD);
      i=0;    //Reuse previously declared iterator variable and reset it
      for (i=0; i<255; i++) {   //Ramp up power to max speed
        Motor_Left->setSpeed(i);
        Motor_Right->setSpeed(i);  
        delay(10);
      }
      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
      delay(500);    //Wait for robot to drive backward to clear dumped object
      Motor_Left->setSpeed(0);
      Motor_Right->setSpeed(0);
      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(RELEASE);
      #endif
      
      commandState = 0;   //Reset flag
    }
  }
  
  //SQUARE - Release Valve
  if(ps2.readButton(PS2_SQUARE) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_SQUARE) == 0) { //double check button to prevent false trigger
      digitalWrite(12, LOW);
      digitalWrite(13, LOW);
    }
  }
  else
  {
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);
  }

  /****************************
      JOYSTICKS
  ****************************/
  //LEFT joystick position
  joystick_drive = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD is positive)
  joystick_steer = ps2.readButton(PS2_JOYSTICK_LEFT_X_AXIS) - 128;  //Get joystick difference from center position (RIGHT is positive)
  
  if (joystick_drive > 10 || joystick_drive < -10) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD.
    drive_speed = map(joystick_drive, 0, 128, 0, 255);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
    turn_speed = map(joystick_steer, 0, 128, 0, 255);   
 
    //MOVING FORWARD
    if (drive_speed > 0) {   //Check FORWARD/BACK direction of joystick (FORWARD is positive)
      //If turning LEFT:
      if (joystick_steer < 0) {   //Check LEFT/RIGHT direction of joystick
        motor_speed_left = drive_speed - (drive_speed * 0.3);   //drive_speed automatically decreases as the joystick is pushed left. 
                                                                //The rate is increased by subtracting an additional fraction of the current speed.)
        motor_speed_right = drive_speed + (255 - drive_speed);   //Adjust the right wheel speed (which is based on drive_speed) to a higher speed by adding the difference between
                                                                 //the maximum drive speed and the current drive speed. This essentially keeps the motor at maximum speed.
      }
      //If turning RIGHT:
      else if (joystick_steer > 0) {
        motor_speed_left = drive_speed + (255 - drive_speed);   //Same control logic as turning left...
        motor_speed_right = drive_speed - (drive_speed * 0.3);  //but applied to the opposite motors.
      }
      //if going STRAIGHT, keeps both motors at equal speed
      else {
        motor_speed_left = drive_speed;
        motor_speed_right = drive_speed;
      }
      #ifdef DRIVE_MOTORS
      Motor_Left->setSpeed(motor_speed_left);
      Motor_Right->setSpeed(motor_speed_right);
      Motor_Left->run(BACKWARD);
      Motor_Right->run(BACKWARD);
      #endif
    }
    //MOVING BACKWARD
    else if (drive_speed < 0) {
      //If turning LEFT (Robot BODY rotating RIGHT as it moves backward):
      if (joystick_steer < 0) {
        motor_speed_left = drive_speed - (drive_speed * 0.3);
        motor_speed_right = drive_speed - (255 + drive_speed);
        
      }
      //If turning RIGHT (Robot BODY rotating LEFT as it moves backward):
      else if (joystick_steer > 0) {
        motor_speed_left = drive_speed - (255 + drive_speed);
        motor_speed_right = drive_speed - (drive_speed * 0.3);
      }
      else {
        motor_speed_left = drive_speed;
        motor_speed_right = drive_speed;
      }
      
      #ifdef DRIVE_MOTORS
      Motor_Left->setSpeed(-motor_speed_left);    //IMPORTANT: Multiply motor speeds by -1 to convert to a positive value since direction is controlled by the "run" command.
      Motor_Right->setSpeed(-motor_speed_right);
      Motor_Left->run(FORWARD);
      Motor_Right->run(FORWARD);
      #endif
    }
    
    #ifdef DEBUG_DRIVE
    Serial.print ("joystick drive: ");
    Serial.print (joystick_drive);
    Serial.print ("\t");
    Serial.print ("drive speed: ");
    Serial.print (drive_speed);
    Serial.print ("\t");
    Serial.print ("turn_speed: ");
    Serial.print (turn_speed);
    Serial.print ("\t");
    Serial.print ("motor_left: ");
    Serial.print (motor_speed_left);
    Serial.print ("\t");
    Serial.print ("motor_right: ");
    Serial.println (motor_speed_right);
    #endif
    
  }
  else {
    motor_speed_left = 0;
    motor_speed_right = 0;
    last_drive_speed = 0;
    #ifdef DRIVE_MOTORS
    Motor_Left->setSpeed(0);
    Motor_Right->setSpeed(0);
    Motor_Left->run(RELEASE);
    Motor_Right->run(RELEASE);
    #endif
  }

  //RIGHT joystick position
  joystick_arm = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Get joystick difference from center position (UP is positive)

  if (joystick_arm > 15 || joystick_arm < -15) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD.
    delay(10);
    joystick_arm = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Double check after a short delay to prevent false activation
    if (joystick_arm > 15 || joystick_arm < -15) {
      arm_speed = map(joystick_arm, 0, 128, 0, 255);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
      motor_speed_arm = arm_speed;
  
      #ifdef ARM_MOTOR
      if (arm_speed > 0) {
        Motor_Arm->setSpeed(motor_speed_arm);
        Motor_Arm->run(BACKWARD);   //Run motor in direction to LOWER arm when joystick is "pushed forward"
      }
      else if (arm_speed < 0) {   //Check if negative..
        Motor_Arm->setSpeed(-motor_speed_arm);    //Always pass "setSpeed" a positive value. Multiply the passed argument by -1 to make the passed value positive.
        Motor_Arm->run(FORWARD);   //Run motor in direction to LIFT arm when joystick is "pulled back"
      }
      #endif
  
      #ifdef DEBUG_ARM
      Serial.print ("joystick arm: ");
      Serial.print (joystick_arm);
      Serial.print ("\t");
      Serial.print ("arm_speed: ");
      Serial.print (arm_speed);
      Serial.print ("\t");
      Serial.print("Arm Pot: ");
      Serial.println (analogRead(arm_pot));
      #endif
    }
  }
  else {
    motor_speed_arm = 0.0;
    arm_speed = 0.0;
    
    #ifdef ARM_MOTOR
    Motor_Arm->setSpeed(0);
    Motor_Arm->run(RELEASE);
    #endif
  }
  
  delay(50);    //Master delay between cycles
}
