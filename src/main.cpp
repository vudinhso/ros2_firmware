#include <Arduino.h> // Arduino library for basic functions

#include "MyMotor.h"      // Library for the motor
#include "MyController.h" // Library for the controller
#include "MyIMU.h"        // Library for the IMU
#include "MySerial.h"     // Library for the serial

Motor motor1(MOT1_A, MOT1_B, ENC1_A, ENC1_B);    
Motor motor2(MOT2_A, MOT2_B, ENC2_A, ENC2_B);    
Controller controller1(&w1, &MOT1_cmd, &w1_ref); 
Controller controller2(&w2, &MOT2_cmd, &w2_ref); 

void setup()
{
  SerialBegin();
  motor1.begin();      
  motor2.begin();      
  controller1.begin(); 
  controller2.begin(); 
  IMUBegin();          
}

void loop()
{
  w1 = motor1.getVelocity();
  w2 = motor2.getVelocity();
  
  controller1.compute();       // Compute the PID control output
  controller2.compute();       
  motor1.send_pwm(MOT1_cmd);   // Send the PWM command to the motor
  motor2.send_pwm(MOT2_cmd);   
  IMUGetData();                // Get the data from the IMU
  SerialDataPrint();           // Print the data to the Serial Monitor
  // SerialDataRead();            // Read the data to the Serial Monitor // Required for ROS2
  SerialDataWrite();            // Write the data to the Serial Monitor // Only for testing
}
