#include "../../include/Vibrator.h"

Vibrator::Vibrator(VIBRATOR_PINS pin) {
    // Constructor
    this->pin = pin;
    this->active = false;
    pinMode(this->pin, OUTPUT);
}

bool Vibrator::isActive() {
    return this->active;
}

bool Vibrator::toggle() {
    this->active = !this->active;
	this->writePin();

	bool before_active = this->active;
    return this->active != before_active;
}

bool Vibrator::set(bool status) {
    this->active = status;
	this->writePin();

    return this->active = status;
}

void Vibrator::writePin(){
    if(this->active){
        digitalWrite(this->pin, LOW);
    } else {
        digitalWrite(this->pin, HIGH);
    }
}