#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>
#include "Motor.h"
#include "Pinout.h"

class Motor {
    public:
        Motor();
        Motor(PWM_PINS pwm_pin);
        void setSpeed(float duty_cycle_us);
    private:
        PWM_PINS pwm_pin;
        Servo motor;
};
#endif
