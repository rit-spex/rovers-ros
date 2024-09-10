#include "../include/StepperMotor.h"

template<typename T>
StepperMotor<T>::StepperMotor(CONTROLLER_ADDRESS type)
{
    TicI2C *stepper = new TicI2C(type);
}
    
template<typename T>
StepperMotor<T>::~StepperMotor() // Virtual destructor to ensure proper cleanup
{
    // Not implemented
}

template<typename T>
void StepperMotor<T>::setPower(float power)
{
    // if motor is in safe mode it will not move
    stepper->exitSafeStart();

    // get max speed of motor then muliply it times the power to make it a power
    int maxSpeed = stepper->getMaxSpeed();
    stepper->setTargetVelocity(maxSpeed * power);
    // need to resetCommandTimeout otherwise it will timeout 
}

template<typename T>
void StepperMotor<T>::setPosition(T target)
{
    // if motor is in safe mode it will not move
    stepper->exitSafeStart();

    // set the target position of the motor
    stepper->setTargetPosition(target);
    // need to resetCommandTimeout otherwise it will timeout 
}

template<typename T>
void StepperMotor<T>::setVelocity(int velocity)
{
    // if motor is in safe mode it will not move
    stepper->exitSafeStart();

    // set the target velocity of the motor
    stepper->setTargetVelocity(velocity);
    // need to resetCommandTimeout otherwise it will timeout 
}

template<typename T>
T StepperMotor<T>::getPosition(void)
{
    return stepper->getCurrentPosition()
}

template<typename T>
float StepperMotor<T>::getPower(void)
{
    return 0;
}

template<typename T>
int StepperMotor<T>::getVelocity(void)
{
    return stepper->getCurrentVelocity();
}

template<typename T>
void StepperMotor<T>::stop(void)
{
    // Not implemented
}