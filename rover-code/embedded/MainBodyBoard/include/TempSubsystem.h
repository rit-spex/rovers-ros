/*
File: temp_subsystem.h
Author: Ryan Barry
Date Created: 1/23/2024

This file defines the temp subsystem class for the rover.

This class is responsible for reading the temperature of the thermistors.
*/

#ifndef TEMP_SUBSYSTEM_H
#define TEMP_SUBSYSTEM_H

#define NUM_THERMISTORS 4
#define NUM_FANS 4

// this is in celsius
#define MAX_TEMP 44 //110 ferinheight
#define MIN_TEMP 15 // 60 ferinheight

//this is duty cycle pwm Signal
#define MAX_FAN_SPEED 255
#define MIN_FAN_SPEED 51

#include "Fan.h"
#include "Thermistor.h"
#include "Pinout.h"
#include "CAN.h"

using namespace std;

class TempSubsystem {
    public:
        TempSubsystem(CAN *can);

        float* getTemperature();
        void setFansPower(int power);
        /*
         * Update the temperature and fan power
        */
        void updateFans(void);
    private:
        // Array of Thermistor objects
        Thermistor m_thermistors[NUM_THERMISTORS];

        // Array of Fan objects
        Fan m_fans[NUM_FANS];

        // Array of temperature readings
        float m_temperature[NUM_THERMISTORS];

        // Pointer to the CAN object
        CAN *m_can;
};

#endif