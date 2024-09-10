#include "../include/SoilCollector.h"

SoilCollector::SoilCollector(CAN *can) 
{
    this->can = can;
  // Set the default values for the soil collector
    this->turnDirection = TURN_DIRECTION::CLOCKWISE;
    
    this->extended = false;
    this->turning = false;
}

bool SoilCollector::isExtended(){return extended;}

bool SoilCollector::isTurning(){return turning;}

TURN_DIRECTION SoilCollector::getTurnDirection(){return turnDirection;}

void SoilCollector::setTurnDirection(TURN_DIRECTION direction){turnDirection = direction;}