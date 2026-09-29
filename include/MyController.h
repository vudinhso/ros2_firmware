#ifndef MY_CONTROLLER_H // Include guard to prevent multiple inclusions of this header file
#define MY_CONTROLLER_H

#include <Arduino.h> // Arduino library for basic functions
#include <PID_v1.h>  // PID library for PID control

inline double w1_ref, w2_ref;
inline double MOT1_cmd, MOT2_cmd;

class Controller
{
public:
    Controller(double *, double *, double *); // Constructor to initialize the PID controller
    void begin();                             // Method to initialize the PID controller
    void compute();                           // Method to compute the PID control output

private:
    double _kp = 1800., _ki = 8000., _kd = .00; // PID Parameters - to be tuned
    double *_input, *_output, *_ref;     // Input variable for the PID controller
    int _direct = 0;                     // Direction of the PID controller
    PID _PID;                            // PID object to handle the PID control
};

#endif