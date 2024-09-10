#ifndef MICROPUMP_H
#define MICROPUMP_H

#define PPSTONORMAL(pulsesPerSecond) pulsesPerSecond * 1000
#define NORMALTOPPS(normalizedSpeed) normalizedSpeed / 1000

#include "EnumList.h"
#include "CAN.h"
/*
Attributes
- pumping: bool
- mode: MICROPUMP_MODE
- pulsesPerSecond: int
- normalizedSpeed: float
*/

class MicroPump
{
private:
    bool pumping;
    int pulsesPerSecond;
    float normalizedSpeed;

    CAN *can;
    MICROPUMP_MODE mode;
public:
    MicroPump(CAN *can);

    float ppsToNormal(int pulses);
    int normalToPps(float normalVal);

    int setNormal(float normalVal);
    int setPps(int pulses);

    int getPps(void);
    float getNormal(void);
};

#endif