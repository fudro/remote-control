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
/*IMPORTANT:
 * Make sure the pin assignments are as follows: 
 * out1_A = 7, out1_B = 8, out2_A = 4, out2_B = 9
 * Otherwise the wheel rotation directions may not work as expected.
 */
 //LEFT and RIGHT motor ports are based on the physical configuration of the Big Dog robot.
int out1_A = 7;  //Motor Port 1 (RIGHT)
int out1_B = 8;
int out2_A = 4;  //Motor Port 2 (LEFT)
int out2_B = 9;

//pins 5 and 6 have the highest pwm frequency on the Uno (980Hz). For Uno Wifi Rev2, all PWM pins operate at 976Hz.
int pwm_right = 5;   //RIGHT WHEELS
int pwm_left = 6;   //LEFT WHEELS


void setup()
{
  ps2.begin(9600);    // Set the Software Serial baudrate. This baudrate must match the jumper setting on the PS2 shield.
                      //IMPORTANT: 9600 is the most reliable baudrate to prevent errant values being received by the shield.
  Serial.begin(115200); // Set serial monitor baudrate. It is OK to set this to a rate higher than the PS2 shield software serial.
  Serial.println("BigDog Remote Control Test!");

  pinMode(out1_A, OUTPUT);
  pinMode(out1_B, OUTPUT);
  pinMode(out2_A, OUTPUT);
  pinMode(out2_B, OUTPUT);
  pinMode(pwm_right, OUTPUT);
  pinMode(pwm_left, OUTPUT);

  //Initialize all motors as "braked"
  digitalWrite(out1_A, LOW);
  digitalWrite(out1_B, LOW);
  digitalWrite(out2_A, LOW);
  digitalWrite(out2_B, LOW);
}

void loop()
{
  //Find LEFT joystick distance from center position. Value of 128 is center of the Y axis. Fully Forward is Zero. Fully Backward is 255.
  joystick_left_Y = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD is positive, BACKWARD is negative)

  //Get LEFT DRIVE speed. Check for joystick movement beyond FORWARD/BACKWARD dead zone.
  if (joystick_left_Y > 20 || joystick_left_Y < -20) {      //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD). Defaults to 20 for both.
    drive_speed_left = map(joystick_left_Y, 0, 128, 0, 255);    //Map values to HALF (255/2) of max power for better drivability.
  }
  else {
    drive_speed_left = 0;    //if no detectable joystick movement (beyond dead zone)
  }

  //Find RIGHT joystick distance from center position. Value of 128 is center of the Y axis. Fully Forward is Zero. Fully Backward is 255.
  joystick_right_Y = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Get joystick difference from center position (FORWARD is positive)

  //Check for joystick movement beyond UP/DOWN dead zone
  if (joystick_right_Y > 20 || joystick_right_Y < -20) {      //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD). Defaults to 20 for both.
    drive_speed_right = map(joystick_right_Y, 0, 128, 0, 255);    //Map values to HALF (255/2) of max power for better drivability.
  }
  else {
    drive_speed_right = 0;    //if no detectable joystick movement (beyond dead zone)
  }

  //Send command to motor controller
  moveRobot(drive_speed_left, drive_speed_right);
  Serial.print(ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS));
  Serial.print("\t");
  Serial.print(drive_speed_left);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS));
  Serial.print("\t");
  Serial.print(drive_speed_right);
  Serial.print("\n");
  delay(100);
}

 void moveRobot(int left_speed, int right_speed) {
  //Set Speeds
  analogWrite(pwm_left, left_speed);
  analogWrite(pwm_right, right_speed);
  //Set Left Wheel Direction
  if (left_speed > 0) { //Forward
    digitalWrite(out2_A, HIGH);
    digitalWrite(out2_B, LOW);
  }
  else if (left_speed < 0) {  //Backward
    left_speed *= -1;         //Change negative value provided by joystick to a positive PWM value
    analogWrite(pwm_left, left_speed); //Update pin to corrected PWM value
    digitalWrite(out2_A, LOW);
    digitalWrite(out2_B, HIGH);
  }
  else if (left_speed == 0) {  //Stopped
    digitalWrite(out2_A, LOW);
    digitalWrite(out2_B, LOW);
  }
  
  //set Right Wheel Direction
  if (right_speed > 0) { //Forward
    digitalWrite(out1_A, HIGH);
    digitalWrite(out1_B, LOW);
  }
  else if (right_speed < 0) {  //Backward
    right_speed *= -1;         //Change negative value provided by joystick to a positive PWM value
    analogWrite(pwm_right, right_speed); //Update pin to corrected PWM value
    digitalWrite(out1_A, LOW);
    digitalWrite(out1_B, HIGH);
  }
  else if (right_speed == 0) {  //Stopped
    digitalWrite(out1_A, LOW);
    digitalWrite(out1_B, LOW);
  }
 }
