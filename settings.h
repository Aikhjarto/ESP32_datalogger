#ifndef SETTINGS_H
#define SETTINGS_H

/*****************************************************************************/
/* board select */
/*****************************************************************************/
//#define FIREBEETLE
//#define HELTEC_ESP32_KIT
#define COMPILE_FOR_HEIZUNG

/*****************************************************************************/
/* includes */
/*****************************************************************************/
#include <esp32-hal-gpio.h>   // for pin definitions

/*****************************************************************************/
/* Network and time settings */
/*****************************************************************************/
#include "arduino_secrets.h"


// Note: Don't use 6-11 since they are used by ESP32 interally to communicate with onboard FLASH.

/*****************************************************************************/
/* Sleep and Wake Cycles */
/*****************************************************************************/
// initial values for wake-up timing (might get overwritten by MQTT messages)
#define SLEEP_SECONDS                 15
#define WIFI_CONNECT_EVERY_Nth_LOOP   10
#define DISPLAY_UPDATE_EVERY_Nth_LOOP  5 // save power


#ifdef COMPILE_FOR_HEIZUNG
#include "settings_heizung.h"
#else

#if defined(HELTEC_ESP32_KIT)
#include "settings_Heltec_ESP32_Kit.h"
#elif defined(FIREBEETLE)
#include "settings_Firebeetle.h"
#endif


#if defined(HELTEC) || defined(FIREBEETLE)
/*****************************************************************************/
/* Liquid Crystal Display */
/*****************************************************************************/
// // size of display
#define LCD_ROWS 2
#define LCD_COLS 16

// // I2C interface
// #define LCD_I2C_INTERFACE_NUM 1 //unused so far
#define LCD_I2C_ADDRESS 0x27
#define LCD_SDA_PIN GPIO_NUM_21
#define LCD_SCL_PIN GPIO_NUM_22


// // lower PIN count for operating HD44780 LCD driver
// #define LCD_RS_PIN GPIO_NUM_13
// #define LCD_EN_PIN GPIO_NUM_10
// #define LCD_D0_PIN GPIO_NUM_9
// #define LCD_D1_PIN GPIO_NUM_27
// #define LCD_D2_PIN GPIO_NUM_26
// #define LCD_D3_PIN GPIO_NUM_25

// // optional additional PIN definition for operation HD44780 LCD driver
// #define LCD_D4_PIN GPIO_NUM_13
// #define LCD_D5_PIN GPIO_NUM_14
// #define LCD_D6_PIN GPIO_NUM_15
// #define LCD_D7_PIN GPIO_NUM_16
// #define LCD_RW_PIN GPIO_NUM_18

#endif

#endif


#endif
