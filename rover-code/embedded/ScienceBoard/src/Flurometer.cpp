#include "../include/Flurometer.h"

Flurometer::Flurometer(CAN *can)
{
    this->can = can;
    this->pump = new MicroPump(can);
    this->position = FLURO_POS::FLURO_POS0;
    this->photoDiode = 0;
    this->diodeLEDPin = 0;
    this->pumping = false;
}

Flurometer::~Flurometer()
{
    delete this->can;
    //delete this->pump;
    //delete this->stepper;
}

bool Flurometer::toggleLED()
{
    return false;
}

bool Flurometer::togglePumping()
{
    return false;
}

bool Flurometer::foundLife()
{
    return false;
}

bool Flurometer::isPumping() {return this->pumping;}

FLURO_POS Flurometer::currPos() {return this->position;}

void Flurometer::update(void)
{
    //this->pump->update();
    //this->stepper->update();
}