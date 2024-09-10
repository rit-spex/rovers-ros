#ifndef SoilTransferer_h
#define SoilTransferer_h

#include "SoilTransferer.h"
#include "StepperMotor.h"
#include "Vibrator.h"
#include "CAN.h"
#include "EnumList.h"

/*
Attributes
- positions: PosSoilTransfer
- currPos: float
- targetPos: float
- caroMotor: StepperMotor
- vibrators: Vibrator[6]
- can: *CAN
*/

class SoilTransferer 
{
    public:
        SoilTransferer(CAN *can);

        void transferSoil();

        int getPos();
        int getTargetPos();
        std::array<bool, 6> activeVibrators();  
        
        void setVibrator(VIBRATOR_PINS pin, bool active);
        void update(void);
        
    private:
 
        StepperMotor<double> caroMotor = StepperMotor<double>(STEPPER_PINS::SOILTRANSFER);
        CAN *can;
        Vibrator *vibrators[6];

        float currPos;
        float targetPos;
};

#endif