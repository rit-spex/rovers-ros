#include "../include/SoilTransferer.h"

SoilTransferer::SoilTransferer( CAN *can)
{
    // Constructor
    this->can = can;         
    currPos = 0;
    targetPos = 0;
    this->vibrators[0] = new Vibrator(VIBRATOR_PINS::VIBRATOR_PIN0);    
    this->vibrators[1] = new Vibrator(VIBRATOR_PINS::VIBRATOR_PIN1);
    this->vibrators[2] = new Vibrator(VIBRATOR_PINS::VIBRATOR_PIN2);
    this->vibrators[3] = new Vibrator(VIBRATOR_PINS::VIBRATOR_PIN3);
    this->vibrators[4] = new Vibrator(VIBRATOR_PINS::VIBRATOR_PIN4);
    this->vibrators[5] = new Vibrator(VIBRATOR_PINS::VIBRATOR_PIN5);
}
int SoilTransferer::getPos() {return caroMotor.getPosition();}

int SoilTransferer::getTargetPos() {return targetPos;}

std::array<bool, 6> SoilTransferer::activeVibrators() 
{
    // Returns an array of vibrators that are currently active
    std::array<bool, 6> activeVibrators;
    for(int i = 0; i < 6; i++)
    {
        if(vibrators[i]->isActive())
        {
            activeVibrators[i] = 1;
        }
        else
        {
            activeVibrators[i] = 0;
        }
    }
    return activeVibrators;
}

void SoilTransferer::setVibrator(VIBRATOR_PINS pin, bool active)
{
    // Sets the vibrator to active or inactive
    vibrators[pin]->set(active);
}

void SoilTransferer::update(void)
{
    
}