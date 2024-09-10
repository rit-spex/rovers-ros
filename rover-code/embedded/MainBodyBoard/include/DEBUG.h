/*
File: DEBUG.h 
Author: Tyler Halifax
Date Created: 3/21/2024

This file is used to define the debug flags for the rover code.
If disable is defined, the corresponding subsystem will be disabled.
If debug is defined, the corresponding subsystem will be serial printed
*/

#define DISABLE_STATUS_LIGHT
// #define DISABLE_CAN
#define DISABLE_DRIVEBASE
#define DISABLE_TEMP

#ifndef DISABLE_STATUS_LIGHT
// #define DEBUG_STATUS_LIGHT
#endif
#ifndef DISABLE_CAN
// #define DEBUG_CAN
#endif
#ifndef DISABLE_DRIVEBASE
// #define DEBUG_DRIVEBASE
#endif
#ifndef DISABLE_TEMP
// #define DEBUG_TEMP
#endif