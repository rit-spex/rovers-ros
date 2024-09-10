#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "Motor.h"
#include "Pinout.h"
#include "EnumList.h"
#include <Stepper.h>

template<typename T>
class StepperMotor : public Motor<T>
{
    private:
    T currPos;
    T targetPos;
    Stepper stepper;

    public:

    // Constructor
    StepperMotor(STEPPER_PINS type);
    ~StepperMotor(); // Virtual destructor to ensure proper cleanup


    void setPower(float power) override;
    void setPosition(T target) override;

    T getPosition(void) override; 
    float getPower(void) override;

    void stop(void);
};
#endif