/*
 * This program accepts input from a wireless game controller to control a robot.
 * 
 * HARDWARE:
 * Arduino Uno R3 Clone (ATMega328P)
 * Sony PS2 Wireless Force 2 Controller
 * Cytron PS2 Shield: http://www.cytron.com.my/p-shield-ps2
 * Seeedstudio 4A Motorshield
 * Drive Motors: Vex 393 DC Motor
 * 
 * FUNCTIONALITY:
 * "Speed pins" control the PWM of the corresponding OUT channel.
 * SPEED_LEFT controls: OUT1, OUT2
 * SPEED_RIGHT controls: OUT3, OUT4
 * 
 * "Out pins" control the logic (HIGH or LOW) of the output ports that control the motors.
 * By setting the logic of the OUT pins, the polarity and thus directionality of the motor 
 * is changed.
 * 
 * REFERENCE:
 * Logic Table (located in Schematic, Ports section): https://wiki.seeedstudio.com/4A_Motor_Shield/
 */

#include <SoftwareSerial.h>
#include "Cytron_PS2Shield.h"
 
//Create PS2Shield object
Cytron_PS2Shield ps2(2, 3); // SoftwareSerial: assign Rx and Tx pin
//Cytron_PS2Shield ps2; // Create object without paramaters to use the default hardware serial pins 0(RX) and 1(TX)

int joystick_left = 0;       //left joystick position in Y axis (FORWARD and BACKWARD)
int joystick_right = 0;       //left joystick position in X axis (LEFT and RIGHT) 

float left_speed = 0.0;     //base speed component in the FORWARD/BACKWARD direction - derived from "joystick_left"
float right_speed = 0.0;      //speed adjustment value for left and right wheels while turning

int commandState = 0;
int speedUp = 0;      //flag to track whether a speed increase button is currently pressed
int speedDown = 0;    //flag to track whether the speed decrease button is currently pressed

int OUT1 = 5;
int OUT2 = 6;
int OUT3 = 7;
int OUT4 = 8;
int SPEED_LEFT = 9;   //PWM pin that controls the speed of the left motor.
int SPEED_RIGHT = 10; //PWM pin that controls the speed of the left motor.
int max_speed = 50;   //default max speed value
int speed_increment = 10;

void setup()
{ 
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting 4A MotorShield Test!");
  Serial.println("Setting Speed!");
  Serial.println("");
  ps2.begin(115200);          //Start remote control shield and set baud rate (baudrate must be the same as the jumper setting at PS2 shield)
  delay(500);
  
  pinMode(SPEED_LEFT, OUTPUT);
  pinMode(SPEED_RIGHT, OUTPUT);
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);
  pinMode(OUT4, OUTPUT);

  Serial.println("Stopped");
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, LOW);
  delay(500);
  
  analogWrite(SPEED_LEFT, max_speed);  //Left Wheel
  analogWrite(SPEED_RIGHT, max_speed);  //Right Wheel 
  
}

void loop()
{ 
  /***************************
            D-PAD
   **************************/
  //UP
  if(ps2.readButton(PS2_UP) == 0)   // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_UP) == 0 && commandState == 0) { //Double check button press after delay
      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
      Serial.println("UP Pressed!");
      Serial.println("Forward");
      digitalWrite(OUT1, HIGH);
      digitalWrite(OUT2, LOW);
      digitalWrite(OUT3, HIGH);
      digitalWrite(OUT4, LOW);
      delay(5000);
      Serial.println("Stopped");
      digitalWrite(OUT1, LOW);
      digitalWrite(OUT2, LOW);
      digitalWrite(OUT3, LOW);
      digitalWrite(OUT4, LOW);
      delay(500);
      commandState = 0;
    }
  }

  //RIGHT
  if(ps2.readButton(PS2_RIGHT) == 0)   // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_RIGHT) == 0 && commandState == 0) { //Double check button press after delay
      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
      Serial.println("UP Pressed!");
      //Turn Right
      Serial.println("Turn Right");
      digitalWrite(OUT1, HIGH);
      digitalWrite(OUT2, LOW);
      digitalWrite(OUT3, LOW);
      digitalWrite(OUT4, HIGH);
      delay(3000);
      Serial.println("Stopped");
      digitalWrite(OUT1, LOW);
      digitalWrite(OUT2, LOW);
      digitalWrite(OUT3, LOW);
      digitalWrite(OUT4, LOW);
      delay(500);
      commandState = 0;
    }
  }

  //DOWN
  if(ps2.readButton(PS2_DOWN) == 0)   // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_DOWN) == 0 && commandState == 0) { //Double check button press after delay
      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
      Serial.println("DOWN Pressed!");
      //Move Backward
      Serial.println("Backward");
      digitalWrite(OUT1, LOW);
      digitalWrite(OUT2, HIGH);
      digitalWrite(OUT3, LOW);
      digitalWrite(OUT4, HIGH);
      delay(5000);
      Serial.println("Stopped");
      digitalWrite(OUT1, LOW);
      digitalWrite(OUT2, LOW);
      digitalWrite(OUT3, LOW);
      digitalWrite(OUT4, LOW);
      delay(500);
      commandState = 0;
    }
  }

  //LEFT
  if(ps2.readButton(PS2_LEFT) == 0)   // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_LEFT) == 0 && commandState == 0) { //Double check button press after delay
      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
      Serial.println("LEFT Pressed!");
      //Turn Left
      Serial.println("Turn Left");
      digitalWrite(OUT1, LOW);
      digitalWrite(OUT2, HIGH);
      digitalWrite(OUT3, HIGH);
      digitalWrite(OUT4, LOW);
      delay(3000);
    
      Serial.println("Stopped");
      digitalWrite(OUT1, LOW);
      digitalWrite(OUT2, LOW);
      digitalWrite(OUT3, LOW);
      digitalWrite(OUT4, LOW);
      delay(500);
      commandState = 0;
    }
  }


    /****************************
      JOYSTICKS
  ****************************/
  //LEFT joystick position
  joystick_left = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD is positive)
  if (joystick_left > 10 || joystick_left < -10) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD.
    delay(10);
    joystick_left = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);   //Double check after a short delay to prevent false activation
    if (joystick_left > 10 || joystick_left < -10) {
      left_speed = map(joystick_left, 0, 128, 0, max_speed);    //Map values to max speed using only half of joystick travel (center to extremity)
      if (left_speed > 0) {
         digitalWrite(OUT1, HIGH);
         digitalWrite(OUT2, LOW);
      }
      else if(left_speed < 0) {
        digitalWrite(OUT1, LOW);
        digitalWrite(OUT2, HIGH);
      }
      Serial.print ("max_speed: ");
      Serial.print (max_speed);
      Serial.print ("\t");
  //    Serial.print ("joystick left: ");
  //    Serial.print (joystick_left);
  //    Serial.print ("\t");
      Serial.print ("left speed: ");
      Serial.print (left_speed);
      Serial.print ("\t");
  //    Serial.print ("joystick right: ");
  //    Serial.print (joystick_right);
  //    Serial.print ("\t");
      Serial.print ("right speed: ");
      Serial.println (right_speed);
    }
  }
  else {
    //Stop Motor
    digitalWrite(OUT1, LOW);
    digitalWrite(OUT2, LOW);
    
    //Reset joystick values
    joystick_left = 0;
    left_speed = 0;
  }
 
  //RIGHT joystick position
  joystick_right = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Get joystick difference from center position (UP is positive)
  if (joystick_right > 10 || joystick_right < -10) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD.
    delay(10);
    joystick_right = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Double check after a short delay to prevent false activation
    if (joystick_right > 10 || joystick_right < -10) {
      right_speed = map(joystick_right, 0, 128, 0, max_speed);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
      if(right_speed > 0) {
        digitalWrite(OUT3, HIGH);
        digitalWrite(OUT4, LOW);
      }
      else if(right_speed < 0) {
        digitalWrite(OUT3, LOW);
        digitalWrite(OUT4, HIGH);
      }
      Serial.print ("max_speed: ");
      Serial.print (max_speed);
      Serial.print ("\t");
  //    Serial.print ("joystick left: ");
  //    Serial.print (joystick_left);
  //    Serial.print ("\t");
      Serial.print ("left speed: ");
      Serial.print (left_speed);
      Serial.print ("\t");
  //    Serial.print ("joystick right: ");
  //    Serial.print (joystick_right);
  //    Serial.print ("\t");
      Serial.print ("right speed: ");
      Serial.println (right_speed);
    }
  }
  else {
    //Stop Motor
    digitalWrite(OUT3, LOW);
    digitalWrite(OUT4, LOW);
    
    //Reset joystick values
    joystick_right = 0;
    right_speed = 0;
  }
   
  
  /***************************
      TRIGGERS - STEPPER MOTOR
   ***************************/
  //RIGHT Trigger 1
  if(ps2.readButton(PS2_RIGHT_1) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_RIGHT_1) == 0 && speedDown == 0) { //double check buttonstate after short delay to prevent false trigger
      //Increment Max_Speed
      speedDown = 1;
      if(max_speed - 10 >= 0) {
        max_speed -= 10;
        analogWrite(SPEED_LEFT, max_speed);  //Left Wheel
        analogWrite(SPEED_RIGHT, max_speed);  //Right Wheel 
      }
    }
  }
  else if (speedDown == 1) {
    //Rotate CCW
    speedDown = 0;
  }
  
  //RIGHT Trigger 2
  if(ps2.readButton(PS2_RIGHT_2) == 0) // 0 = pressed, 1 = released
  {
    delay(10);
    if(ps2.readButton(PS2_RIGHT_2) == 0 && speedUp == 0) { //double check buttonstate after short delay to prevent false trigger
      //Increment Max_Speed
      speedUp = 1;
      if(max_speed + 10 <= 100) {
        max_speed += 10;
        analogWrite(SPEED_LEFT, max_speed);  //Left Wheel
        analogWrite(SPEED_RIGHT, max_speed);  //Right Wheel 
      }
    }
  }
  else if (speedUp == 1) {
    //Rotate CCW
    speedUp = 0;
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
  
  delay(50);    //Master delay between cycles
}
