#include "../include/MicroPump.h"

MicroPump::MicroPump(CAN *can)
{
    this->can = can;
    this->pumping = false;
    this->mode = MICROPUMP_MODE::DISCRETE;
    this->pulsesPerSecond = 0;
    this->normalizedSpeed = 0;
}

float MicroPump::ppsToNormal(int pulses)
{
    return PPSTONORMAL(pulses);
}

int MicroPump::normalToPps(float normalVal)
{
    return NORMALTOPPS(normalVal);
}

int MicroPump::setNormal(float normalVal)
{
    this->normalizedSpeed = normalVal;
    this->pulsesPerSecond = normalToPps(normalVal);
    return 0;
}

int MicroPump::setPps(int pulses)
{
    this->pulsesPerSecond = pulses;
    this->normalizedSpeed = ppsToNormal(pulses);
    return 0;
}

int MicroPump::getPps()
{
    return this->pulsesPerSecond;
}

float MicroPump::getNormal()
{
    return this->normalizedSpeed;
}

