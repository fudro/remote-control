



//
//  //RIGHT joystick position
//  joystick_arm = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Get joystick difference from center position (UP is positive)
//
//  if (joystick_arm > 15 || joystick_arm < -15) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD.
//    delay(10);
//    joystick_arm = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Double check after a short delay to prevent false activation
//    if (joystick_arm > 15 || joystick_arm < -15) {
//      arm_speed = map(joystick_arm, 0, 128, 0, 255);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
////      motor_speed_arm = arm_speed;    //Set arm to variable speed based on joystick.
//      motor_speed_arm = 70; //Set arm speed to a constant value
//  
//      #ifdef ARM_MOTOR
//      if (arm_speed > 0) {
//        if (analogRead(arm_pot) >= arm_trimmer_low_threshold) {
//          Motor_Arm->setSpeed(abs(motor_speed_arm));
//          Motor_Arm->run(BACKWARD);   //Run motor in direction to LOWER arm when joystick is "pushed forward"
//        }
//        else {  //Stop arm is lower limit reached.
//          Motor_Arm->setSpeed(0);
//          Motor_Arm->run(RELEASE);
//        }
//      }
//      else if (arm_speed < 0) {   //Check if negative..
//        if (analogRead(arm_pot) <= arm_trimmer_high_threshold) { 
//          Motor_Arm->setSpeed(abs(motor_speed_arm));    //Always pass "setSpeed" a positive value.
//          Motor_Arm->run(FORWARD);   //Run motor in direction to LIFT arm when joystick is "pulled back"
//        }
//        else {  //Stop arm if upper limit reached.
//          Motor_Arm->setSpeed(0);
//          Motor_Arm->run(RELEASE);
//        }
//      }
//      #endif
//  
//      #ifdef DEBUG_ARM
//      Serial.print ("joystick arm: ");
//      Serial.print (joystick_arm);
//      Serial.print ("\t");
//      Serial.print ("arm_speed: ");
//      Serial.print (arm_speed);
//      Serial.print ("\t");
//      Serial.print("Arm Pot: ");
//      Serial.println (analogRead(arm_pot));
//      #endif
//    }
//  }
//  else {
//    motor_speed_arm = 0.0;
//    arm_speed = 0.0;
//    
//    #ifdef ARM_MOTOR
//    Motor_Arm->setSpeed(0);
//    Motor_Arm->run(RELEASE);
//    #endif
//  }
//  
//  delay(50);    //Master delay between cycles
//}


  /***************************
            D-PAD
   **************************/
//  #ifdef DEBUG_DPAD
//  //UP
//  if(ps2.readButton(PS2_UP) == 0)   // 0 = pressed, 1 = released
//  {
//    delay(10);
//    if(ps2.readButton(PS2_UP) == 0 && commandState == 0) { //Double check button press after delay
//      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
//      Serial.println("UP Pressed!");
//      //Move Forward
//      encoder1 = 0;               //Reset encoders
//      encoder2 = 0;
//      #ifdef DRIVE_MOTORS
//      //Start motors
//      Motor_Left->run(BACKWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
//      Motor_Right->run(BACKWARD);
//      int i=0;
//      for (i=0; i<255; i++) {   //Ramp up power to max speed
//        Motor_Left->setSpeed(i);
//        Motor_Right->setSpeed(i);  
//        delay(10);
//      }
//      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
//      delay(5000);    //Drive for 5 seconds. As an alternative, encoder-based movement can be used.
//      Motor_Left->setSpeed(0);
//      Motor_Right->setSpeed(0);
//      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
//      Motor_Right->run(RELEASE);
//      #endif
//      commandState = 0;
//    }
//  }
//
//  //RIGHT
//  if(ps2.readButton(PS2_RIGHT) == 0)   // 0 = pressed, 1 = released
//  {
//    delay(10);
//    if(ps2.readButton(PS2_RIGHT) == 0 && commandState == 0) { //Double check button press after delay
//      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
//      Serial.println("UP Pressed!");
//      //Turn Right
//      encoder1 = 0;               //Reset encoders
//      encoder2 = 0;
//      #ifdef DRIVE_MOTORS
//      //Start motors
//      Motor_Left->run(BACKWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
//      Motor_Right->run(FORWARD);
//      int i=0;
//      for (i=0; i<255; i++) {   //Ramp up power to max speed
//        Motor_Left->setSpeed(i);
//        Motor_Right->setSpeed(i);  
//        delay(10);
//      }
//      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
//      while(encoder1 < 80) {   //measure using the wheel on the outside of the turn (e.g. Left Wheel for righthand turns)
//        //wait for encoder-based turn to complete 
//        //Value of 80 is good for a 90 degree encoder-based turn
//        //Value of 170 is good for a 180 degree encoder-based turn
//      }
//      //As an alternative, time-based movement can be used:
////      delay(500);    //Wait to complete 90 degree time-based turn
////      delay(2750);    //Wait to complete 180 degree time-based turn
//      Motor_Left->setSpeed(0);
//      Motor_Right->setSpeed(0);
//      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
//      Motor_Right->run(RELEASE);
//      #endif
//      commandState = 0;
//    }
//  }
//
//  //DOWN
//  if(ps2.readButton(PS2_DOWN) == 0)   // 0 = pressed, 1 = released
//  {
//    delay(10);
//    if(ps2.readButton(PS2_DOWN) == 0 && commandState == 0) { //Double check button press after delay
//      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
//      Serial.println("DOWN Pressed!");
//      //Move Backward
//      encoder1 = 0;               //Reset encoders
//      encoder2 = 0;
//      #ifdef DRIVE_MOTORS
//      //Start motors
//      Motor_Left->run(FORWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
//      Motor_Right->run(FORWARD);
//      int i=0;
//      for (i=0; i<255; i++) {   //Ramp up power to max speed
//        Motor_Left->setSpeed(i);
//        Motor_Right->setSpeed(i);  
//        delay(10);
//      }
//      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
//      delay(5000);    //Drive for 5 seconds. As an alternative, encoder-based movement can be used.
//      Motor_Left->setSpeed(0);
//      Motor_Right->setSpeed(0);
//      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
//      Motor_Right->run(RELEASE);
//      #endif
//      commandState = 0;
//    }
//  }
//
//  //LEFT
//  if(ps2.readButton(PS2_LEFT) == 0)   // 0 = pressed, 1 = released
//  {
//    delay(10);
//    if(ps2.readButton(PS2_LEFT) == 0 && commandState == 0) { //Double check button press after delay
//      commandState = 1;   //Set flag to prevent repeat activation while movement is executing.
//      Serial.println("LEFT Pressed!");
//      //Turn Left
//      encoder1 = 0;               //Reset encoders
//      encoder2 = 0;
//      #ifdef DRIVE_MOTORS
//      //Start motors
//      Motor_Left->run(FORWARD);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
//      Motor_Right->run(BACKWARD);
//      int i=0;
//      for (i=0; i<255; i++) {   //Ramp up power to max speed
//        Motor_Left->setSpeed(i);
//        Motor_Right->setSpeed(i);  
//        delay(10);
//      }
//      Motor_Right->setSpeed(240);   //Adjust (reduce) Right Wheel motor power to account for robot drift to the left.
//      while(encoder2 < 80) {   //measure using the wheel on the outside of the turn (e.g. Left Wheel for righthand turns)
//        //wait for encoder-based turn to complete 
//        //Value of 80 is good for a 90 degree encoder-based turn
//        //Value of 170 is good for a 180 degree encoder-based turn
//      }
//      //As an alternative, time-based movement can be used:
////      delay(500);    //Wait to complete 90 degree time-based turn
////      delay(2750);    //Wait to complete 180 degree time-based turn
//      Motor_Left->setSpeed(0);
//      Motor_Right->setSpeed(0);
//      Motor_Left->run(RELEASE);   //IMPORTANT: FORWARD and BACKWARD are intentionally reversed due to reverse directionality caused by the gearing of the robot.
//      Motor_Right->run(RELEASE);
//      #endif
//      commandState = 0;
//    }
//  }
//  #endif
