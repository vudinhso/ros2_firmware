#include "MyController.h"

Controller::Controller(double *input, double *output, double *ref)
    : _PID(input, output, ref, _kp, _ki, _kd, DIRECT) // Initialize the PID object with input, output, and reference
{
    _input = input;
    _output = output;
    _ref = ref;
}

void Controller::begin()
{
    _PID.SetMode(AUTOMATIC);         // Set the PID controller to automatic mode
    _PID.SetOutputLimits(-4096, 4096); // Set the output limits for the PID controller
    _PID.SetSampleTime(10);          // Set the sample time for the PID controller
}
void Controller::compute()
{
    _PID.SetTunings(_kp, _ki, _kd); // Set the PID tuning parameters
    _PID.Compute();                 // Compute the PID control output
}