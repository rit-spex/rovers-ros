#include "../../include/TempSubsystem.h"


/*
* Constructor for the temp subsystem class.
* Initializes the thermistors.
*/
TempSubsystem::TempSubsystem(CAN *can) :
    m_thermistors
    {
        Thermistor(THERMISTOR_PINS::THERMISTOR_PIN_0), 
        Thermistor(THERMISTOR_PINS::THERMISTOR_PIN_1),
        Thermistor(THERMISTOR_PINS::THERMISTOR_PIN_2),
        Thermistor(THERMISTOR_PINS::THERMISTOR_PIN_3)
    },
    m_fans
    {
        Fan(FAN_PINS::FAN_PIN_0),
        Fan(FAN_PINS::FAN_PIN_1),
        Fan(FAN_PINS::FAN_PIN_2),
        Fan(FAN_PINS::FAN_PIN_3)
    },
    m_can(can)
{}


// @return An array of temperatures measured by each thermistor
float* TempSubsystem::getTemperature() 
{
    for(int i=0; i<NUM_THERMISTORS; i++)
    {
        m_temperature[i] = m_thermistors[i].getTemperature();
    }
    return m_temperature;
}

// Set the power of the fans
void TempSubsystem::setFansPower(int power)
{
    power = min(max(power, MIN_FAN_SPEED), MAX_FAN_SPEED); // clamp the power to the range (MIN_FAN_SPEED, MAX_FAN_SPEED)

    for (int i = 0; i < NUM_FANS; i++)
    {
        m_fans[i].setPower(power);
    }
}

void TempSubsystem::updateFans()
{
    getTemperature();
    float avgTemp = 0;
    for (int i = 0; i < NUM_THERMISTORS; i++)
    {
        avgTemp += m_temperature[i];
    }
    avgTemp /= NUM_THERMISTORS;

    if(avgTemp > MAX_TEMP)
    {
        setFansPower(MAX_FAN_SPEED);
    }
    else if(avgTemp < MIN_TEMP)
    {
        setFansPower(MIN_FAN_SPEED);
    }
    else
    {
        float deltaTemp = MAX_TEMP - MIN_TEMP;
        float deltaPower = MAX_FAN_SPEED - MIN_FAN_SPEED;
        
        int power = (avgTemp - MIN_TEMP) / deltaTemp * deltaPower + MIN_FAN_SPEED;
        setFansPower(0); // change later
    }
}