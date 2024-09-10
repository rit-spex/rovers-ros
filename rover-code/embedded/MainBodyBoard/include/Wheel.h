/*
File: wheel.h
Author: Ryan Barry
Date Created: 1/23/2024

This file defines the wheel class for the rover.

This class is responsible for controlling each individual wheel of the rover 
based on its target speed.
*/

#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>
#include <Servo.h>
#include "Pinout.h"
#include "Wheel.h"
#include "Motor.h"
#include "QuadDecoder.h"
#include "PIDController.h"


#define FULL_FORWARD 2000
#define MIN_FORWARD 1525
#define NEUTRAL 1500
#define MIN_REVERSE 1475
#define FULL_REVERSE 1000

class Wheel {
    public:
        /*
        * Constructor for the wheel class
        * @param pwm_pin The PWM pin the wheel is connected to
        * @param enc_A_pin The A pin of the wheel's quadrature encoder
        * @param enc_B_pin The B pin of the wheel's quadrature encoder
        */
        Wheel(PWM_PINS pwm_pin, ENC_A_PINS enc_A_pin, ENC_B_PINS enc_B_pin, double kp, double ki, double kd);

        /*
        * Adjust's the PWM signal to the wheel to match the target speed
        * @param targetSpeed The target speed of the wheel
        */
        void setSpeed(float targetSpeed);

        /*
        * Gets the current speed of the wheel
        * @return The current speed of the wheel
        */
        int getSpeed();
    private:
        float currentSpeed;
        float targetSpeed;
        Motor motor;
        QuadratureDecoder encoder;
        PIDController pid;
        int pwm_duty_cycle;
};

#endif
