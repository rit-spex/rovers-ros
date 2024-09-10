#ifndef VIBRATOR_H
#define VIBRATOR_H

#include "Pinout.h"
#include <Arduino.h>

class Vibrator {
	public:
		Vibrator(VIBRATOR_PINS pin);
		bool isActive(void);
		bool toggle(void);
		bool set(bool status);
		void writePin(void);
	private:
		VIBRATOR_PINS pin;
		bool active;
};

#endif