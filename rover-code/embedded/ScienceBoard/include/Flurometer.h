#ifndef FLUROMETER_H
#define FLUROMETER_H

#include "CAN.h"
#include "MicroPump.h"
#include "StepperMotor.h"
#include "EnumList.h"

class Flurometer
{
  public:
    Flurometer(CAN *can);
    ~Flurometer();

    bool toggleLED(); 
    bool togglePumping(); 
    bool foundLife(); 
    bool isPumping(); 
    FLURO_POS currPos(); 
    
    void update(void);

  private:
    CAN *can;
    MicroPump *pump;
    FLURO_POS position;
    StepperMotor<double> stepper = StepperMotor<double>(STEPPER_PINS::FLUROMETER); 

    float photoDiode;
    int diodeLEDPin;     
    bool pumping;
    
    

};




#endif // FLUROMETER_H