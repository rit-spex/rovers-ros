#ifndef FAN_H
#define FAN_H

#include "Pinout.h"
#include <Arduino.h>

class Fan
{
    public:
    Fan(FAN_PINS pin);
    void setPower(int pwm_signal);
    //pwm pin to control fan speed
    // four fans
    private:
    FAN_PINS m_pin;
};

#endif