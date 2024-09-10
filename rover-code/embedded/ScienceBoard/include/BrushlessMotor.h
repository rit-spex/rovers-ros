#ifndef BrushlessMotor_H
#define BrushlessMotor_H

#include "Motor.h"
#include "Pinout.h"
#include "EnumList.h"

template<typename T>
class BrushlessMotor : public Motor<T>
{
    private:
    T currPos;
    T targetPos;

    public:

    // Constructor
    BrushlessMotor();
    ~BrushlessMotor(); // Virtual destructor to ensure proper cleanup


    void setPower(float power) override;
    void setPosition(T target) override;

    T getPosition(void) override; 
    float getPower(void) override;

    void stop(void);
};
#endif