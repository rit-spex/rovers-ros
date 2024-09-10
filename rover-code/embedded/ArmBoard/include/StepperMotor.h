#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "Motor.h"
#include "Pinout.h"
#include "EnumList.h"
#include <Tic.h>

template<typename T>
class StepperMotor : public Motor<T>
{
    private:
    TicI2C *stepper;
    
    public:

    // Constructor
    StepperMotor(CONTROLLER_ADDRESS type);
    ~StepperMotor(); // Virtual destructor to ensure proper cleanup

    void setVelocity(int velocity);
    void setPower(float power) override;
    void setPosition(T target) override;

    T getPosition(void) override; 
    float getPower(void) override;
    int getVelocity(void);

    void stop(void);
};
#endif