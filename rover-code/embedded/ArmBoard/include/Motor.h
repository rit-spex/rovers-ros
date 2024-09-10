#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>

class Encoder {
public:
    // Encoder class definition goes here
};

template<class T>
class Motor {
public:
    virtual ~Motor() {} // Virtual destructor to ensure proper cleanup

    // Constructor
    //Motor(int pin, Encoder encoder) {}

    // Helper methods as pure virtual functions
    virtual void setPower(float power) = 0;
    virtual float getPower(void) = 0;
    virtual void setPosition(T target) = 0;
    virtual T getPosition(void) = 0;
    virtual void stop() = 0;
};
#endif