#ifndef SETTINGS_H
#define SETTINGS_H

/*****************************************************************************/
/* includes */
/*****************************************************************************/
#include <esp32-hal-gpio.h>   // for pin definitions

/*****************************************************************************/
/* Network and time settings */
/*****************************************************************************/
#include "arduino_secrets.h"

/*****************************************************************************/
/* Misc Settings */
/*****************************************************************************/
// #define STATUSLED_PIN         GPIO_NUM_19  // high when operating, low when in deep-sleep
// #define WAKEUP_PIN            GPIO_NUM_4   // when connected to 3.3V via pushbutton, ePaper refresh is performed
// /* if SENSOR_VALUES_AS_JSON is defined, {"epoch":13949503,"value":2.33} is published instead of only 2.33 */
// #define SENSOR_VALUES_AS_JSON 

// #define VBAT_PIN                GPIO_NUM_13  // analog input pin
// #define VBAT_DIV_RATIO          ((float)3.0)// Voltage divider ratio 


/*****************************************************************************/
/* Sleep and Wake Cycles */
/*****************************************************************************/
// initial values for wake-up timing (might get overwritten by MQTT messages)
#define SLEEP_MILLISECONDS            15000
#define WIFI_CONNECT_EVERY_Nth_LOOP   10
#define DISPLAY_UPDATE_EVERY_Nth_LOOP 5
#define HISTORY_LENGTH                5
#define SYSTEMINFO_PUBLISH_INTERVAL (int(600))

/*****************************************************************************/
/* DS18B20 OneWire Temperature Sensors */
/*****************************************************************************/
// Pins
#define ONE_WIRE_BUS_PIN      GPIO_NUM_33  // don't forget external pull-up (4k7 resistor between ONE_WIRE_BUS_PIN and ONE_WIRE_PWR_PIN )
#define ONE_WIRE_PWR_PIN      GPIO_NUM_32  // Output pin to enable/disable supply to the bus
#define ONE_WIRE_PWR_RTC_PIN  GPIO_NUM_9   // in ULP, Pin numbering is different, thus ONE_WIRE_PWR_PIN needs to be defined twice

// total number of OwnWire Temperature Sensors
const size_t OW_SERIALS_LEN = 5;

// serial numbers to enumerate for DS18B20 sensors
const uint8_t OW_SERIALS[][8] = {
  { 0x28, 0xB8, 0x6D, 0x91, 0x13, 0x21, 0x01, 0xDE },
  { 0x28, 0x72, 0xD3, 0x94, 0x13, 0x21, 0x01, 0xE2 },
  { 0x28, 0xCA, 0x19, 0x88, 0x13, 0x21, 0x01, 0xF1 },
  { 0x28, 0x4E, 0xAF, 0x89, 0x13, 0x21, 0x01, 0x9B },
  { 0x28, 0x85, 0x8A, 0x8E, 0x13, 0x21, 0x01, 0x50 },
};

// names for the MQTT topics for the temperature sensors in same order as OW_SERIALS
static const char* const TEMPERATURE_SENSOR_NAMES[] = {"Input Temperature 1", 
                                                       "Output Temperature 1",
                                                       "Input Temperature 2", 
                                                       "Output Temperature 2",
                                                       "Room Temperature",
                                                       };

/*****************************************************************************/
/* OLED settings for Heltec ESP32 Kit with integrated display */
/*****************************************************************************/
#define OLED_I2C_INTERFACE_NUM  1           // I2C interface number. Default pins: 0: 21/22 1: Pins 4/15
#define OLED_SCREEN_WIDTH       128         // OLED display width in pixels
#define OLED_SCREEN_HEIGHT      64          // OLED display height in pixels
#define OLED_RESET_PIN          GPIO_NUM_16 
#define OLED_SDA_PIN            GPIO_NUM_4  
#define OLED_SCL_PIN            GPIO_NUM_15

#define STATUSLED_PIN           GPIO_NUM_25

/*****************************************************************************/
/* Waveshare ePaper */
/*****************************************************************************/
// #define EPAPER_PWR_PIN        GPIO_NUM_23 // ESP->ePaper VCC pin
// #define EPAPER_BUSY_PIN       GPIO_NUM_25 // ePaper->ESPhigh when ePaper is busy
// #define EPAPER_RST_PIN        GPIO_NUM_26 // ESP->ePaper reset pin to wake display up
// #define EPAPER_DC_PIN         GPIO_NUM_27 // ESP->ePaper switch between data and command transfer
// #define EPAPER_SPI_CLK_PIN    GPIO_NUM_13
// #define EPAPER_SPI_MISO_PIN   GPIO_NUM_12 // unused by this project but required for SPI configuration
// #define EPAPER_SPI_MOSI_PIN   GPIO_NUM_14
// #define EPAPER_SPI_CS_PIN     GPIO_NUM_15
// #define DISPLAY_UPDATE_EVERY_Nth_LOOP  4

/*****************************************************************************/
/* digital inputs */
/*****************************************************************************/
#define DIs_LEN 1
const gpio_num_t DIs[DIs_LEN] = { GPIO_NUM_35, };

// names for digital input signals
static const char* const DI_SENSOR_NAMES[DIs_LEN] = {"TestDI",};

// boolean array whether or not the logic values should be inverted
const bool DI_INV[DIs_LEN] = {true,};

// input mode array
const uint8_t DI_MODE[DIs_LEN] = {INPUT}; // INPUT, INPUT_PULLUP, INPUT_PULLDOWN

/*****************************************************************************/
/* Analog input */
/*****************************************************************************/
#define AIs_LEN 1
const uint8_t AIs[AIs_LEN] = { (u_int8_t)35, };
// names for analog input signals
static const char* const AI_SENSOR_NAMES[AIs_LEN] = {"TestAI",};


/*****************************************************************************/
/* Liquid Crystal Display */
/*****************************************************************************/
// // size of display
// #define LCD_ROWS 2
// #define LCD_COLS 16

// // // I2C interface
// // #define LCD_I2C_INTERFACE_NUM 1 //unused so far
// #define LCD_I2C_ADDRESS 0x27
// #define LCD_SDA_PIN GPIO_NUM_21
// #define LCD_SCL_PIN GPIO_NUM_22


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


/*****************************************************************************/
/* I2C RGB sensors */
/*****************************************************************************/
#include <Adafruit_TCS34725.h>
// #define TCS34725_INTEGRATION_TIME TCS34725_INTEGRATIONTIME_614MS // optional
// #define TCS34725_GAIN TCS34725_GAIN_1X  // optional
// #define RGB_PWR_PIN GPIO_NUM_5  // optional


/*****************************************************************************/
/* I2C Humidity sensor */
/*****************************************************************************/
#include <Adafruit_BME280.h>

#endif