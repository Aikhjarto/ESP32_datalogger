#ifndef SETTINGS_HEIZUNG_H
#define SETTINGS_HEIZUNG_H

#include <esp32-hal-gpio.h>   // for pin definitions

/*****************************************************************************/
/* Misc Settings */
/*****************************************************************************/
#define STATUSLED_PIN         GPIO_NUM_19  // high when operating, low when in deep-sleep
#define WAKEUP_PIN            GPIO_NUM_4   // when connected to 3.3V via pushbutton, ePaper refresh is performed
/* if SENSOR_VALUES_AS_JSON is defined, {"epoch":13949503,"value":2.33} is published instead of only 2.33 */
#define SENSOR_VALUES_AS_JSON 

/*****************************************************************************/
/* ePaper */
/*****************************************************************************/
#define EPAPER_PWR_PIN        GPIO_NUM_23 // ESP->ePaper VCC pin
#define EPAPER_BUSY_PIN       GPIO_NUM_25 // ePaper->ESPhigh when ePaper is busy
#define EPAPER_RST_PIN        GPIO_NUM_26 // ESP->ePaper reset pin to wake display up
#define EPAPER_DC_PIN         GPIO_NUM_27 // ESP->ePaper switch between data and command transfer
#define EPAPER_SPI_CLK_PIN    GPIO_NUM_13
#define EPAPER_SPI_MISO_PIN   GPIO_NUM_12 // unused by this project but required for SPI configuration
#define EPAPER_SPI_MOSI_PIN   GPIO_NUM_14
#define EPAPER_SPI_CS_PIN     GPIO_NUM_15
#define DISPLAY_UPDATE_EVERY_Nth_LOOP  4

/*****************************************************************************/
/* History storage */
/*****************************************************************************/
#define HISTORY_LENGTH                100

/*****************************************************************************/
/* DS18B20 OneWire Temperature Sensors */
/*****************************************************************************/
// Pins
#define ONE_WIRE_BUS_PIN      GPIO_NUM_33  // don't forget external pull-up (4k7 resistor between ONE_WIRE_BUS_PIN and ONE_WIRE_PWR_PIN )
#define ONE_WIRE_PWR_PIN      GPIO_NUM_32  // Output pin to enable/disable supply to the bus
#define ONE_WIRE_PWR_RTC_PIN  GPIO_NUM_9   // in ULP, Pin numbering is different, thus ONE_WIRE_PWR_PIN needs to be defined twice

#include "settings_heizung_module_1.h"


#endif