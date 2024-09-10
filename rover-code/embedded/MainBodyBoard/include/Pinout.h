#ifndef PINOUT_H
#define PINOUT_H

enum PWM_PINS {
    PWM_PIN_0 = 2,
    PWM_PIN_1 = 3,
    PWM_PIN_2 = 4,
    PWM_PIN_3 = 5,
    PWM_PIN_4 = 6,
    PWM_PIN_5 = 7
};

enum ENC_A_PINS {
    ENC_A_PIN_0 = 8,
    ENC_A_PIN_1 = 9,
    ENC_A_PIN_2 = 10,
    ENC_A_PIN_3 = 11,
    ENC_A_PIN_4 = 12,
    ENC_A_PIN_5 = 13
};

enum ENC_B_PINS {
    ENC_B_PIN_0 = 14,
    ENC_B_PIN_1 = 15,
    ENC_B_PIN_2 = 16,
    ENC_B_PIN_3 = 17,
    ENC_B_PIN_4 = 18,
    ENC_B_PIN_5 = 19
};

enum CAN_PINS {
    CAN_RX_PIN = 0,
    CAN_TX_PIN = 1
};

enum THERMISTOR_PINS {
    THERMISTOR_PIN_0 = 20,
    THERMISTOR_PIN_1 = 21,
    THERMISTOR_PIN_2 = 22,
    THERMISTOR_PIN_3 = 23
};

enum FAN_PINS {
    FAN_PIN_0 = 22,
    FAN_PIN_1 = 23,
    FAN_PIN_2 = 24,
    FAN_PIN_3 = 25
};

#define STATUS_LIGHT_PIN 26

#endif
