#include "MyMotor.h"

Motor::Motor(byte MOTA, byte MOTB, byte ENCA, byte ENCB)

{
    _MOTA = MOTA;
    _MOTB = MOTB;
    _ENCA = ENCA;
    _ENCB = ENCB;
}

void Motor::begin()
{
    pinMode(_MOTA, OUTPUT);
    pinMode(_MOTB, OUTPUT);
    ledcAttach(_MOTA, _PWM_FREQ, _PWM_RES);
    ledcAttach(_MOTB, _PWM_FREQ, _PWM_RES);

    _encoder.attachFullQuad(_ENCA, _ENCB); 
    _encoder.setCount(0);
}

long Motor::getCount() // Get the current count from the encoder
{
    return _encoder.getCount();
}

double Motor::getVelocity() // Get the velocity of the encoder
{
    th = _encoder.getCount() / 4 * 2 * PI / _ENC_RES; // motor angular position in radians
    if (micros() - w_time >= 10 * 1e3)           // Velocity is calculated every 10ms
    {
        w_raw = (th - th_prev) / ((micros() - w_time) * 1e-6); // Calculate the unfiltered velocity
        w = alpha * w_raw + (1 - alpha) * w;                   // Calculate the filtered velocity
        w_time = micros();
        th_prev = th;
    }
    return w;
}

void Motor::send_pwm(double motor_cmd)
{
    if (motor_cmd < 0)
    {
        ledcWrite(_MOTA, 1);
        ledcWrite(_MOTB, abs(motor_cmd));
    }
    else
    {
        ledcWrite(_MOTB, 1);
        ledcWrite(_MOTA, abs(motor_cmd));
    }
}