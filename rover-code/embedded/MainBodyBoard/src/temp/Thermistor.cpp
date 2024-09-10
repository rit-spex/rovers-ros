#include "../../include/Thermistor.h"

Thermistor::Thermistor(THERMISTOR_PINS thermistorPin) : m_thermistorPin(thermistorPin)
{
    pinMode(m_thermistorPin, INPUT);
}

float Thermistor::getTemperature() {
    
    float voltage = analogRead(m_thermistorPin) * (3.3 / 1023.0) * 1000 ; // convert to mV
    // Steinhart-Hart equation
    // https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
    // See if we want to use this or approximation scale from datasheet
    
    //float resistance = 10000.0 * voltage / (3.3 - voltage);  
    //m_temperature = 1.0 / (log(resistance / 10000.0) / 3950.0 + 1.0 / 298.15) - 273.15;
    
    // equation from the thermistor datasheet
    //https://www.ti.com/lit/ds/symlink/lmt87.pdf?ts=1707515767599
    m_temperature = (13.582 - sqrt(184.47 + 0.01732) * (2230.8-voltage))/(-0.00866) + 30;
    return m_temperature;
}
