/*
 DESCRIPTION:
 * This sketch provides simple "tank-style" drive control for a two-motor robot chassis.

 FUNTIONALITY:
 * Dual joystick drive control
 * Each joystick provides Forward/Backward movement for one side of the robot (LEFT Wheels/RIGHT Wheels)

 HARDWARE:
 * Arudino Uno / Uno Wifi Rev2
 * SparkFun Monster Moto Shield
 * Cystron PS2 Shield
 * Wireless PS2 Controller

 NOTES:
 * Be sure to disconnect motor power before removing Remote Control Receiver Cartridge.

 Main Controller Function:
  readButton(button); // Read button status of controller, it will return corresponding data
                      // Digital Buttons: 0 = pressed, 1 = released
                      // Analog Joysticks: return an integer value between 0-255

 CONTROLLER CONSTANTS:
 Digital Buttons:
  PS2_SELECT
  PS2_JOYSTICK_LEFT
  PS2_JOYSTICK_RIGHT
  PS2_START
  PS2_UP
  PS2_RIGHT
  PS2_DOWN
  PS2_LEFT
  PS2_LEFT_2
  PS2_RIGHT_2
  PS2_LEFT_1
  PS2_RIGHT_1
  PS2_TRIANGLE
  PS2_CIRCLE
  PS2_CROSS
  PS2_SQUARE

 Analog Joysticks:
  PS2_JOYSTICK_LEFT_X_AXIS
  PS2_JOYSTICK_LEFT_Y_AXIS
  PS2_JOYSTICK_RIGHT_X_AXIS
  PS2_JOYSTICK_RIGHT_Y_AXIS
  PS2_JOYSTICK_LEFT_UP
  PS2_JOYSTICK_LEFT_DOWN
  PS2_JOYSTICK_LEFT_LEFT
  PS2_JOYSTICK_LEFT_RIGHT
  PS2_JOYSTICK_RIGHT_UP
  PS2_JOYSTICK_RIGHT_DOWN
  PS2_JOYSTICK_RIGHT_LEFT
  PS2_JOYSTICK_RIGHT_RIGHT
*/

#include <SoftwareSerial.h>
#include "Cytron_PS2Shield.h"

Cytron_PS2Shield ps2(10, 11); //Set the Software Serial communication pins (should match the jumper settings on the PS2 shield)

//Joystick Variables
int joystick_left_Y = 0;      //Joystick positions in Y axis (FORWARD and BACKWARD)
int joystick_right_Y = 0;
int drive_speed_left = 0;     //Movement speeds for each side of the robot
int drive_speed_right = 0;

//Motor Controller Variables

 int out1_A = 7;  //Motor Port 1
 int out1_B = 8;
 int out2_A = 4;  //Motor Port 2
 int out2_B = 9;
/*
 int out1_A = 9;  //Motor Port 1
 int out1_B = 4;
 int out2_A = 8;  //Motor Port 2
 int out2_B = 7;
*/
 int pwm_1 = 5;   //pins 5 and 6 have the highest pwm frequency on the Uno (980Hz). For Uno Wifi Rev2, all PWM pins operate at 976Hz.
 int pwm_2 = 6;

 int errant_value = -64; //The remote control shield will sometimes provide errant values.


void setup()
{
  ps2.begin(9600);    // Set the Software Serial baudrate. This baudrate must match the jumper setting on the PS2 shield.
                      //IMPORTANT: 9600 is the most reliable baudrate to prevent errant values being received by the shield.
  Serial.begin(115200); // Set serial monitor baudrate. It is OK to set this to the maximum rate.
  Serial.println("BigDog Remote Control Test!");

  pinMode(out1_A, OUTPUT);
  pinMode(out1_B, OUTPUT);
  pinMode(out2_A, OUTPUT);
  pinMode(out2_B, OUTPUT);
  pinMode(pwm_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);

  //Initialize all motors as "braked"
  digitalWrite(out1_A, LOW);
  digitalWrite(out1_B, LOW);
  digitalWrite(out2_A, LOW);
  digitalWrite(out2_B, LOW);
}

void loop()
{
  //Find LEFT joystick distance from center position. Value of 128 is center of the Y axis.
  joystick_left_Y = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD is positive)

  //Get LEFT DRIVE speed. Check for joystick movement beyond UP/DOWN dead zone.
  if (joystick_left_Y > 20 || joystick_left_Y < -20) {      //Create "dead zone" for when joystick is centered (with independent adjustment values for DOWN and UP). Defaults to 20 for both.
    drive_speed_left = map(joystick_left_Y, 0, 128, 0, 128);    //Map values to HALF (255/2) of max power for better drivability.
  }
  else {
    drive_speed_left = 0;    //if no detectable joystick movement (beyond dead zone)
  }

  //Find RIGHT joystick distance from center position. Value of 128 is center of the Y axes.
  joystick_right_Y = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Get joystick difference from center position (FORWARD is positive)

  //Check for joystick movement beyond UP/DOWN dead zone
  if (joystick_right_Y > 20 || joystick_right_Y < -20) {      //Create "dead zone" for when joystick is centered (with independent adjustment values for DOWN and UP). Defaults to 20 for both.
    drive_speed_right = map(joystick_right_Y, 0, 128, 0, 128);    //Map values to HALF (255/2) of max power for better drivability.
  }
  else {
    drive_speed_right = 0;    //if no detectable joystick movement (beyond dead zone)
  }

  //Send command to motor controller
  moveRobot(drive_speed_left, drive_speed_right);
  Serial.print(drive_speed_left);
  Serial.print("\t");
  Serial.print(drive_speed_right);
  Serial.print("\n");
  delay(100);
}

 void moveRobot(int left_pwm, int right_pwm) {
  //Set Speeds
  analogWrite(pwm_1, left_pwm);
  analogWrite(pwm_2, right_pwm);
  //Set Left Wheel Direction
  if (left_pwm > 0) { //Forward
    digitalWrite(out2_A, HIGH);
    digitalWrite(out2_B, LOW);
  }
  else if (left_pwm < 0) {  //Backward (Ignore errant values of -64)
    left_pwm *= -1;         //Change negative value provided by joystick to a positive PWM value
    analogWrite(pwm_1, left_pwm); //Update pin to local PWM variable
    digitalWrite(out2_A, LOW);
    digitalWrite(out2_B, HIGH);
  }
  else if (left_pwm == 0) {  //Stopped
    digitalWrite(out2_A, LOW);
    digitalWrite(out2_B, LOW);
  }
  //set Right Wheel Direction
  if (right_pwm > 0) { //Forward
    digitalWrite(out1_A, HIGH);
    digitalWrite(out1_B, LOW);
  }
  else if (right_pwm < 0) {  //Backward (Ignore errant values of -64)
    right_pwm *= -1;         //Change negative value provided by joystick to a positive PWM value
    analogWrite(pwm_2, right_pwm); //Update pin to local PWM variable
    digitalWrite(out1_A, LOW);
    digitalWrite(out1_B, HIGH);
  }
  else if (right_pwm == 0) {  //Stopped
    digitalWrite(out1_A, LOW);
    digitalWrite(out1_B, LOW);
  }
 }
