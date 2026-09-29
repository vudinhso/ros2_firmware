#ifndef MY_MOTOR_H // Include guard to prevent multiple inclusions of this header file
#define MY_MOTOR_H

#include <Arduino.h>
#include <ESP32Encoder.h> // ESP32Encoder library for handling encoders

#define MOT1_A 26 
#define MOT1_B 27 
#define ENC1_A 14 
#define ENC1_B 13 

#define MOT2_A 18 
#define MOT2_B 17 
#define ENC2_A 16 
#define ENC2_B  4 

inline volatile long EncoderTick1, EncoderTick2;    

inline double w1, w2;   

class Motor
{
public:
    Motor(byte MOTA, byte MOTB, byte ENCA, byte ENCB);       // Constructor to initialize the motor object
    void begin();                    // Method to set the pin mode and enable the motors
    void send_pwm(double motor_cmd); // Method to send the PWM signal to the motors
    long getCount();
    double getVelocity();

private:
    byte _MOTA, _MOTB;     // Pin used on ESP32 for the MOT_A
    byte _ENCA, _ENCB;
    int _PWM_FREQ = 12000; // Frequency of the PWM signal
    int _PWM_RES = 12;      // Resolution of the PWM signal

    int _ENC_RES = 270*11;
    unsigned long w_time;
    double th, th_prev, w, w_prev, w_raw;
    double alpha = 0.091;
    ESP32Encoder _encoder;
};

#endif