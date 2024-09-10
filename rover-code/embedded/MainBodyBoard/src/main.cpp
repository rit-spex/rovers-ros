#include <Arduino.h>
#include "../include/MainBodyBoard.h"

MainBodyBoard mbb;

void setup() 
{
  pinMode(STATUS_LIGHT_PIN, OUTPUT);
  mbb = MainBodyBoard();
  Serial.begin(9600);
  Serial.println("Main Body Board");
}

void loop() 
{
    mbb.updateSubsystems();
    delay(1);
}

