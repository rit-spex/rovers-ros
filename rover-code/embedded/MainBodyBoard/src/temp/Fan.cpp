#include "../../include/Fan.h"

Fan::Fan(FAN_PINS pin) : m_pin(pin)
{
    pinMode(pin, OUTPUT);
}

void Fan::setPower(int pwm_signal)
{
    analogWrite(m_pin, pwm_signal);
    // four fans
}