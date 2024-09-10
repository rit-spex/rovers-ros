#include "../include/StepperMotor.h"

template<typename T>
StepperMotor<T>::StepperMotor(STEPPER_PINS type)
{
    switch (type)
    {
        case SOILTRANSFER:
            this->stepper = Stepper(
            SOILTRANSFER_STEPPER_CONFIG::SOILTRANSFER_STEPPER_STEPS, 
            SOILTRANSFER_STEPPER_CONFIG::SOILTRANSFER_STEPPER_PIN0, 
            SOILTRANSFER_STEPPER_CONFIG::SOILTRANSFER_STEPPER_PIN1, 
            SOILTRANSFER_STEPPER_CONFIG::SOILTRANSFER_STEPPER_PIN2, 
            SOILTRANSFER_STEPPER_CONFIG::SOILTRANSFER_STEPPER_PIN3);
        break;
        case FLUROMETER:
            this->stepper = Stepper(
            FLUROMETER_STEPPER_CONFIG::FLUROMETER_STEPPER_STEPS, 
            FLUROMETER_STEPPER_CONFIG::FLUROMETER_STEPPER_PIN0, 
            FLUROMETER_STEPPER_CONFIG::FLUROMETER_STEPPER_PIN1, 
            FLUROMETER_STEPPER_CONFIG::FLUROMETER_STEPPER_PIN2, 
            FLUROMETER_STEPPER_CONFIG::FLUROMETER_STEPPER_PIN3);
        break;
        case SOILCOLLECTOR:
            this->stepper = Stepper(
            SOILCOLLECTOR_STEPPER_CONFIG::SOILCOLLECTOR_STEPPER_STEPS, 
            SOILCOLLECTOR_STEPPER_CONFIG::SOILCOLLECTOR_STEPPER_PIN0, 
            SOILCOLLECTOR_STEPPER_CONFIG::SOILCOLLECTOR_STEPPER_PIN1, 
            SOILCOLLECTOR_STEPPER_CONFIG::SOILCOLLECTOR_STEPPER_PIN2, 
            SOILCOLLECTOR_STEPPER_CONFIG::SOILCOLLECTOR_STEPPER_PIN3);
        break;
    }
}
    
template<typename T>
StepperMotor<T>::~StepperMotor() // Virtual destructor to ensure proper cleanup
{

}

template<typename T>
void StepperMotor<T>::setPower(float power)
{
    // Not implemented
}

template<typename T>
void StepperMotor<T>::setPosition(T target)
{
    this->targetPos = target;
    this->stepper.step(target - this->currPos);
    this->currPos = target;
}

template<typename T>
T StepperMotor<T>::getPosition(void)
{
    return this->currPos;
}

template<typename T>
float StepperMotor<T>::getPower(void)
{
    // Not implemented
    return 0;
}

template<typename T>
void StepperMotor<T>::stop(void)
{
    // Not implemented
}