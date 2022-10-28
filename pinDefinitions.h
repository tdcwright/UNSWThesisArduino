#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#include "Arduino.h"

// LIMIT SWITCH PINS
#define LS_PIN_YELLOW 50 // Top Yellow
#define LS_PIN_PINK 51 // Top Pink
#define LS_PIN_GREY 52 // Bottom Grey
#define LS_PIN_WHITE 53 // Bottom White


// COUNTER CLICK PINS
#define CC_SPI_MISO 12
#define CC_SPI_MOSI 11
#define CC_SPI_CLK 13
#define CC_BUS_1_SELECT 4   // TOP MOTOR
#define CC_BUS_2_SELECT 10  // BOTTOM MOTOR
#define CC_BUS_1_ENABLE A3  // TOP MOTOR
#define CC_BUS_2_ENABLE A4  // BOTTOM MOTOR

// MOTOR CONTROLLER PINS
#define MC_A_PWM 8          // TOP MOTOR
#define MC_B_PWM 6          // BOTTOM MOTOR
#define MC_FORWARD_DIR 7
#define MC_REVERSE_DIR 9

// ACCELEROMETER PINS
#define ACC_INT_PIN 22

// BLUETOOTH PINS
#define BT_EN_PIN 16
#define BT_STATE_PIN 17
#define BT_RX_PIN 18
#define BT_TX_PIN 19


#endif