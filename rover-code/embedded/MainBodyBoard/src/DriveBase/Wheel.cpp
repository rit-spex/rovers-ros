#include "../../include/Wheel.h"


Wheel::Wheel(PWM_PINS pwm_pin, ENC_A_PINS enc_A_pin, ENC_B_PINS enc_B_pin, double kp, double ki, double kd) 
    : motor(pwm_pin), encoder(enc_A_pin, enc_B_pin), pid(kp, ki, kd){

    this->targetSpeed = 0;
    this->currentSpeed = 0;
}

void Wheel::setSpeed(float targetSpeed) {
    this->targetSpeed = targetSpeed;
    /*
    Do something here with controls for PWM
    https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces

    Full Reverse 1000 us pulse width
    Proportional Reverse 1000 to 1475 us pulse width
    Neutral 1475 to 1525 us pulse width
    Proportional Forward 1525 to 2000 us pulse width
    Full Forward 2000 us pulse width

    Maximum pulse range 500 to 2500 us
    Valid frequency 50 to 200 Hz
    */
    if(this->targetSpeed == 0){
        this->pwm_duty_cycle = NEUTRAL;
    }
    else{
        double pid_output = this->pid.update(abs(this->targetSpeed), abs(this->currentSpeed));

        if (this->targetSpeed < 0) {
            this->pwm_duty_cycle = (pid_output - OUTPUT_MIN) / (OUTPUT_MAX - OUTPUT_MIN) * (FULL_REVERSE - MIN_REVERSE);
        }

        else if (this->targetSpeed > 0) {
            this->pwm_duty_cycle = (pid_output - OUTPUT_MIN) / (OUTPUT_MAX - OUTPUT_MIN) * (FULL_FORWARD - MIN_FORWARD);
        }
    }
    
    motor.setSpeed(this->pwm_duty_cycle);
}
