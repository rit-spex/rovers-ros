#ifndef SOIL_COLLECTOR_H
#define SOIL_COLLECTOR_H

#include "StepperMotor.h"
#include "BrushlessMotor.h"
#include "CAN.h"
#include "EnumList.h"

// Attributes
// - stepper: StepperMotor
// - brushless: BrushlessMotor
// - extended: bool
// - turning: bool
// - turnDirection: TurnDirection
// - can: *CAN

class SoilCollector
{
private:
    StepperMotor<double> stepper = StepperMotor<double>(STEPPER_PINS::SOILCOLLECTOR);
    //BrushlessMotor brushless = new BrushlessMotor();

    TURN_DIRECTION turnDirection;

    bool extended;
    bool turning;
    
    CAN *can;
public:
    SoilCollector(CAN *can);

    bool isExtended();
    bool isTurning();
    TURN_DIRECTION getTurnDirection();
    void setTurnDirection(TURN_DIRECTION direction);
    void update();
};

#endif