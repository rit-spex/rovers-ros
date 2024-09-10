#include "../include/MainBodyBoard.h"

MainBodyBoard::MainBodyBoard()
{
    #ifndef DISABLE_STATUS_LIGHT
    pinMode(STATUS_LIGHT_PIN, OUTPUT);
    #endif
}

MainBodyBoard::~MainBodyBoard()
{
    
}

void MainBodyBoard::updateSubsystems(void)
{
    #ifndef DISABLE_STATUS_LIGHT
    if(statusLightWait == 0)
    {
        if(statusLightOn)
        {
            digitalWrite(STATUS_LIGHT_PIN, LOW);
            statusLightOn = false;
        }
        else
        {
            digitalWrite(STATUS_LIGHT_PIN, HIGH);
            statusLightOn = true;
        }
        statusLightWait = 1000;
    }
    else
    {
        statusLightWait--;
    }

    #ifdef DEBUG_STATUS_LIGHT
    Serial.println("Status Light: " + String(statusLightOn));
    #endif

    #endif
    
    can.sendMessage(CAN::CAN_MB::JETSON,CAN::Message_ID::E_STOP, (uint8_t*)1);
    can.TEST();
    
    digitalWrite(STATUS_LIGHT_PIN, HIGH);

    // #ifndef DISABLE_CAN || DISABLE_STATUS_LIGHT
    // if(can.IsEStop(can.getMessage(CAN::Message_ID::E_STOP)))
    // {
    //     statusLightWait = -1;
    //     digitalWrite(STATUS_LIGHT_PIN, HIGH);
    // }
    // else if(statusLightWait < 0)
    // {
    //     statusLightWait = 0;
    // }
    // #endif

    #ifndef DISABLE_DRIVEBASE
    drive_base.updateVelocity();
    #endif

    #ifndef DISABLE_TEMP
    temp_subsystem.updateFans();
    #endif
}

