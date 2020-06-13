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
 * d) Pump Air                                  (TRIANGLE Button)
 * e) Open Scoop                                (CIRCLE Button)
 * f) Close Scoop                               (SQUARE Button)
 * g) Pickup Item (AUTOMATED)                   (CROSS Button)
*/

#include <SoftwareSerial.h>
#include "Cytron_PS2Shield.h"
#include <Adafruit_MotorShield.h>
#include <Stepper.h>

//#define DEBUG_DRIVE
#define DEBUG_ARM
 
//Create PS2Shield object
Cytron_PS2Shield ps2(8, 9); // SoftwareSerial: assign Rx and Tx pin
//Cytron_PS2Shield ps2; // Create object without paramaters to use the default hardware serial pins 0(RX) and 1(TX)
// Create the motor shield object (with the default I2C address)
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Create DC motors on ports M1, M2, and M3 (refer to the motor shield PCB screenprint)
Adafruit_DCMotor *Motor_Left = AFMS.getMotor(1);
Adafruit_DCMotor *Motor_Right = AFMS.getMotor(2);
Adafruit_DCMotor *Motor_Arm = AFMS.getMotor(3);

int joystick_drive = 0;       //left joystick position in Y axis (FORWARD and BACKWARD)
int joystick_steer = 0;       //left joystick position in X axis (LEFT and RIGHT)
int joystick_arm = 0;         //right joystick position in Y axis (UP and DOWN). TODO:Direction is inverted - pull back to lift UP
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

Stepper stepperMotor(STEPS_PER_REV, 4, 6, 5, 7);    //Define stepper motor and the pins used to control the coils



void setup()
{
  pinMode(relay_1, OUTPUT);   //Set all relay pins as outputs
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);
  
  ps2.begin(115200);          //Start remote control shield and set baud rate (baudrate must be the same as the jumper setting at PS2 shield)
  Serial.begin(115200);       //Set baudrate for serial monitor
  Serial.println("Robot Start!");
  
  AFMS.begin();               //Start motorshield object (with the default frequency of 1.6KHz)
  Motor_Left->setSpeed(255);  //Initialize Motors (max motor speed is 255)
  Motor_Left->run(FORWARD);
  Motor_Left->run(RELEASE);
  Motor_Right->setSpeed(255);
  Motor_Right->run(FORWARD);
  Motor_Right->run(RELEASE);
  Motor_Arm->setSpeed(255);
  Motor_Arm->run(FORWARD);
  Motor_Arm->run(RELEASE);
}

void loop()
{ 
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
      stepsRequired = STEPS_PER_OUT_REV/5;    //Rotate one-fifth of a revolution
      stepperMotor.setSpeed(700);
      stepperMotor.step(stepsRequired);
    }
  }
  else if (stepperState == 1) {
    //Rotate CCW
    stepsRequired = - STEPS_PER_OUT_REV/5;
    stepperMotor.setSpeed(700);
    stepperMotor.step(stepsRequired);
    stepperState = 0;
  }
  
  /****************************
      BUTTONS
  ****************************/
  //Triangle
  if(ps2.readButton(PS2_TRIANGLE) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_TRIANGLE) == 0) { //double check button to prevent false trigger
      digitalWrite(10, LOW);
    }
  }
  else
  {
    digitalWrite(10, HIGH);
  }
  //Circle
  if(ps2.readButton(PS2_CIRCLE) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_CIRCLE) == 0) { //double check button to prevent false trigger
      digitalWrite(11, LOW);
    }
  }
  else
  {
    digitalWrite(11, HIGH);
  }
  //Cross
  if(ps2.readButton(PS2_CROSS) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_CROSS) == 0) { //double check button to prevent false trigger
      digitalWrite(12, LOW);
    }
  }
  else
  {
    digitalWrite(12, HIGH);
  }
  //Square
  if(ps2.readButton(PS2_SQUARE) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_SQUARE) == 0) { //double check button to prevent false trigger
      digitalWrite(13, LOW);
    }
  }
  else
  {
    digitalWrite(13, HIGH);
  }


  /****************************
      JOYSTICKS
  ****************************/
  //LEFT joystick position
  joystick_drive = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD is negative)
  joystick_steer = ps2.readButton(PS2_JOYSTICK_LEFT_X_AXIS) - 128;  //Get joystick difference from center position (BACKWARD is positive)
  
  if (joystick_drive > 5 || joystick_drive < -10) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD.
    drive_speed = map(joystick_drive, 0, 128, 0, 255);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
    turn_speed = map(joystick_steer, 0, 128, 0, 255);               

    //MOVING FORWARD
    if (drive_speed > 0) {
      
      //If turning LEFT:
      if (joystick_steer < 0) {
        motor_speed_left = drive_speed + turn_speed * 0.5;
        if(motor_speed_left < 0) {
          motor_speed_left = 0;
        }
        motor_speed_right = drive_speed; 
      }
      //If turning RIGHT:
      else if (joystick_steer > 0) {
        motor_speed_left = drive_speed;
        motor_speed_right = drive_speed - turn_speed * 0.5;
        if(motor_speed_right < 0) {
          motor_speed_right = 0;
        }
      }
      else {
        motor_speed_left = drive_speed;
        motor_speed_right = drive_speed;
      }
      Motor_Left->setSpeed(motor_speed_left);
      Motor_Right->setSpeed(motor_speed_right);
      Motor_Left->run(FORWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
      Motor_Right->run(FORWARD);
    }

    //MOVING BACKWARD
    else if (drive_speed < 0) {
      //If turning LEFT (Robot rotating RIGHT):
      if (joystick_steer < 0) {
        motor_speed_left = drive_speed - turn_speed * 0.5;
        if(motor_speed_left > 0) {
          motor_speed_left = 0;
        }
        motor_speed_right = drive_speed;
        
      }
      //If turning RIGHT:
      else if (joystick_steer > 0) {
        motor_speed_left = drive_speed;
        motor_speed_right = drive_speed + turn_speed * 0.5;
        if(motor_speed_right > 0) {
          motor_speed_right = 0;
        }
      }
      else {
        motor_speed_left = drive_speed;
        motor_speed_right = drive_speed;
      }
      Motor_Left->setSpeed(-motor_speed_left);    //IMPORTANT: Multiply motor speeds by -1 to convert to a positive value since direction is controlled by the "run" command.
      Motor_Right->setSpeed(-motor_speed_right);
      Motor_Left->run(BACKWARD);
      Motor_Right->run(BACKWARD);
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
    Motor_Left->setSpeed(0);
    Motor_Right->setSpeed(0);
    Motor_Left->run(RELEASE);
    Motor_Right->run(RELEASE);
  }

  //RIGHT joystick position
  joystick_arm = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Get joystick difference from center position (UP is positive)

  if (joystick_arm > 5 || joystick_arm < -5) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD.
    arm_speed = map(joystick_arm, 0, 128, 0, 255);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
    motor_speed_arm = arm_speed;

    if (arm_speed > 0) {
      Motor_Arm->setSpeed(motor_speed_arm);
      Motor_Arm->run(FORWARD);   //Run motor in direction to LOWER arm when joystick is "pushed forward"
    }
    else if (arm_speed < 0) {   //Check if negative..
      Motor_Arm->setSpeed(-motor_speed_arm);    //Always pass "setSpeed" a positive value. Multiply the passed argument by -1 to make the passed value positive.
      Motor_Arm->run(BACKWARD);   //Run motor in direction to LIFT arm when joystick is "pulled back"
    }

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
  else {
    motor_speed_arm = 0;
    arm_speed = 0;
    Motor_Arm->setSpeed(0);
    Motor_Arm->run(RELEASE);
  }
  
  delay(50);    //Master delay between cycles
}
