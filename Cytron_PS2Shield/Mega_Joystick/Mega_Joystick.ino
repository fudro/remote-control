/*
This example shows how to read a joystick value on PS2 Controller.

Function:
  readButton(button); // Read button status, it will return corresponding data
                      // Digital button: 0 = pressed, 1 = released
                      // Analog button: return a value

  Digital button:
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

  Analog button:
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

Product page:
  Cytron PS2 Shield: http://www.cytron.com.my/p-shield-ps2
  PS2 Controller: http://www.cytron.com.my/p-ps-gp-1
  CT-UNO: http://www.cytron.com.my/p-ct-uno

Original written by:
            Cytron Technologies

Modified:
  29/06/15  Idris, Cytron Technologies
*/

#include <SoftwareSerial.h>
#include "Cytron_PS2Shield.h"

Cytron_PS2Shield ps2(10, 11); // SoftwareSerial: Rx and Tx pin
//Cytron_PS2Shield ps2; // HardwareSerial, note: 

void setup()
{
  Serial2.begin(9600);
  ps2.begin(9600); // This baudrate must same with the jumper setting at PS2 shield
  Serial.begin(9600); // Set monitor baudrate to 9600
}

void loop()
{
  if (Serial2.available()) {
    Serial.print("Yeah Baby!!");
  }
  // Open monitor and move left joystick in x axis
  // Analog value will be displayed
  Serial.print(ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS));
  Serial.print("\t");
  Serial.print(ps2.readButton(PS2_JOYSTICK_LEFT_X_AXIS));
  Serial.print("\t");
  Serial.print(ps2.readButton(PS2_JOYSTICK_RIGHT_X_AXIS));
  Serial.print("\t");
  Serial.println(ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS));
  delay(100);
}
