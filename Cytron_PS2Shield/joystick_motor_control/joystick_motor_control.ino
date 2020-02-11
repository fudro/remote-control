/*
 * This program accepts input from a game controller to control a robot.
 * 
 * HARDWARE:
 * Arduino Duemilanove (ATMega328P)
 * Sony PS2 Wireless Force 2 Controller
 * Cytron PS2 Shield: http://www.cytron.com.my/p-shield-ps2
 * Adafruit Motor Shield V2.3: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/overview
 * 
 * CONTROL FEATURES:
 * IN-PROGRESS
 * a) Drive control using LEFT joystick:        (Y-Axis = drive speed, X-Axis = turn speed)
 * TODO:
 * b) Shoulder control using RIGHT joystick:    (Y-Axis = arm shoulder UP/DOWN, X-Axis = arm turntable LEFT/RIGHT rotation)
 * c) Wrist control using D-Pad:                (Y-Axis = arm wrist UP/DOWN, X-Axis = rotation for HORIZONTAL/VERTICAL orientation)
 * d) Arm Gripper control using R1, R2:         (R1 = gripper OPEN, R2 = gripper CLOSED)
 * e) Body Gripper control using L1, L2:        (L1 = gripper OPEN, L2 = gripper CLOSED)
 * f) Pickup Item                               (TRIANGLE Button)
 * g) Scan Area                                 (CIRCLE Button)
 * h) Target Item                               (SQUARE Button)
 * i) Pickup Item                               (TRIANGLE Button)
 * j) Place Item                                (TRIANGLE Button)
 * k) Toggle Drive Mode                         (SELECT Button, toggle between single joystick drive and dual joystick tank drive)
 * l) Reset Robot                               (START Button)
*/

#include <SoftwareSerial.h>
#include "Cytron_PS2Shield.h"
#include <Adafruit_MotorShield.h>

#define DEBUG
 
//Create PS2Shield object
Cytron_PS2Shield ps2(2, 3); // SoftwareSerial: assign Rx and Tx pin
//Cytron_PS2Shield ps2; // HardwareSerial: use default pins 0(RX) and 1(TX)
// Create the motor shield object (with the default I2C address)
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Create DC motors on ports M3 and M4 (refer to the motor shield PCB screenprint)
Adafruit_DCMotor *Motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *Motor4 = AFMS.getMotor(4);

int joystick_drive = 0;       //amount of joystick travel in Y axis (UP and DOWN)
int joystick_steer = 0;       //amount of joystick travel in X axis (LEFT and RIGHT)
float drive_speed = 0.00;     //base speed in the FORWARD/BACKWARD direction
float turn_speed = 0.00;      //amount of speed adjustment for left and right wheels while turning
float motor_speed_left = 0;   //individual speed for LEFT wheels of robot
float motor_speed_right = 0;  //individual speed for RIGHT wheels of robot

void setup()
{
  ps2.begin(115200);          //Start motor shield: this baudrate must same with the jumper setting at PS2 shield
  Serial.begin(115200);
  Serial.println("Robot Start!");
  
  AFMS.begin();               //Start motorshield object (with the default frequency of 1.6KHz)
  Motor3->setSpeed(255);      //Initialize Motors (max motor speed is 255)
  Motor3->run(RELEASE);
  Motor4->setSpeed(255);
  Motor4->run(RELEASE);
}

void loop()
{ 
  //LEFT joystick position
  joystick_drive = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD is positive)
  joystick_steer = ps2.readButton(PS2_JOYSTICK_LEFT_X_AXIS) - 128;  //Get joystick difference from center position (RIGHT is positive)
  
  if (joystick_drive > 5 || joystick_drive < -10) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD.
    drive_speed = map(joystick_drive, 0, 128, 0, 255);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
    turn_speed = map(joystick_steer, 0, 128, 0, 255);               

    //MOVING FORWARD
    if (drive_speed > 0) {
      
      //If turning LEFT:
      if (joystick_steer < 0) {
        motor_speed_left = drive_speed + turn_speed * 0.5;
        motor_speed_right = drive_speed;
        
      }
      //If turning RIGHT:
      else if (joystick_steer > 0) {
        motor_speed_left = drive_speed;
        motor_speed_right = drive_speed - turn_speed * 0.5;
      }
      else {
        motor_speed_left = drive_speed;
        motor_speed_right = drive_speed;
      }
      Motor3->setSpeed(motor_speed_left);
      Motor4->setSpeed(motor_speed_right);
      Motor3->run(FORWARD);
      Motor4->run(FORWARD);
    }

    //MOVING BACKWARD
    else if (drive_speed < 0) {
      //If turning LEFT (Robot rotating RIGHT):
      if (joystick_steer < 0) {
        motor_speed_left = drive_speed - turn_speed * 0.5;
        motor_speed_right = drive_speed;
        
      }
      //If turning RIGHT:
      else if (joystick_steer > 0) {
        motor_speed_left = drive_speed;
        motor_speed_right = drive_speed + turn_speed * 0.5;
      }
      else {
        motor_speed_left = drive_speed;
        motor_speed_right = drive_speed;
      }
      Motor3->setSpeed(-motor_speed_left);    //IMPORTANT: Multiply motor speeds by -1 to convert to a positive value since direction is controlled by the "run" command.
      Motor4->setSpeed(-motor_speed_right);
      Motor3->run(BACKWARD);
      Motor4->run(BACKWARD);
    }

    /*
    if (drive_speed > 0) {
        Motor3->setSpeed(motor_speed_left);
        Motor4->setSpeed(motor_speed_right);
        Motor3->run(FORWARD);
        Motor4->run(FORWARD);
    }
    else if (drive_speed < 0) {
        Motor3->setSpeed(-motor_speed_left);
        Motor4->setSpeed(-motor_speed_left);
        Motor3->run(BACKWARD);
        Motor4->run(BACKWARD);
    }
    */
    #ifdef DEBUG
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
    Motor3->setSpeed(0);
    Motor4->setSpeed(0);
    Motor3->run(RELEASE);
    Motor4->run(RELEASE);
  }
  delay(50);
}
