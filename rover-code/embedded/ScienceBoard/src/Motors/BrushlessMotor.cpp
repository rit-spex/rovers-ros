#include "../include/BrushlessMotor.h"

template<typename T>
BrushlessMotor<T>::BrushlessMotor()
{
    // Not implemented
}

template<typename T>
BrushlessMotor<T>::~BrushlessMotor() // Virtual destructor to ensure proper cleanup
{
    // Not implemented
}

template<typename T>
void BrushlessMotor<T>::setPower(float power)
{
    // Not implemented
}

template<typename T>
void BrushlessMotor<T>::setPosition(T target)
{
    this->targetPos = target;
}

template<typename T>
T BrushlessMotor<T>::getPosition(void)
{
    return this->currPos;
}

template<typename T>
float BrushlessMotor<T>::getPower(void)
{
    // Not implemented
    return 0;
}

template<typename T>
void BrushlessMotor<T>::stop(void)
{
    // Not implemented
}