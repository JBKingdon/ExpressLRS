// Configuration for the UI co-processor

#ifndef COPRO_CONFIG_H
#define COPRO_CONFIG_H

// NB Remember that the TFT pins are defined in Setup_240x240_ST7789.h

// pins for aux switches

#define AUX1_LOW    PB12
#define AUX1_HIGH   PB13

#define AUX2_LOW    PB14
#define AUX2_HIGH   PB15

#define AUX3_LOW    PB6
#define AUX3_HIGH   PB7

#define AUX4_LOW    PB8
#define AUX4_HIGH   PB9

// pins for the rotary encoder
#define ROT_ENC_A   PA11
#define ROT_ENC_B   PA12

// UI buttons
#define UI_BUTTON1  PA15
#define UI_BUTTON2  PB3

// PIN for sampling the battery voltage
#define PIN_VBAT    A0

// pins used for the uart connection to the main processor

// main processor uses A2,A3

// #define RX          PA10    // XXX these don't seem to match reality, used for debug monitoring
// #define TX          PA9

#endif // ndef COPRO_CONFIG_H