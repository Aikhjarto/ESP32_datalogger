#ifndef SETTINGS_HEIZUNG_MODULE_1
#define SETTINGS_HEIZUNG_MODULE_1

#include <stddef.h>  // for size_t
#include <esp32-hal-gpio.h> // for pin definitions

/*
Konfiguration für das ESP Modul zur Temperaturmessung rund um die beiden Heizkreismischer
*/

/*****************************************************************************/
/* Misc Settings */
/*****************************************************************************/
//#define STATUS_LED_PIN         GPIO_NUM_19  // high when operating, low when in deep-sleep
#define WAKEUP_PIN            GPIO_NUM_25   // when connected to 3.3V via pushbutton, ePaper refresh is performed
#define HISTORY_LENGTH                90
/* if SENSOR_VALUES_AS_JSON is defined, {"epoch":13949503,"value":2.33} is published instead of only 2.33 */
#define SENSOR_VALUES_AS_JSON 

/*****************************************************************************/
/* ePaper */
/*****************************************************************************/
#define EPAPER_PWR_PIN        GPIO_NUM_21 // ESP->ePaper VCC pin
#define EPAPER_BUSY_PIN       GPIO_NUM_16 // ePaper->ESPhigh when ePaper is busy
#define EPAPER_RST_PIN        GPIO_NUM_4 // ESP->ePaper reset pin to wake display up
#define EPAPER_DC_PIN         GPIO_NUM_17 // ESP->ePaper switch between data and command transfer
#define EPAPER_SPI_CLK_PIN    GPIO_NUM_18
#define EPAPER_SPI_MISO_PIN   GPIO_NUM_19 // unused by this project but required for SPI configuration
#define EPAPER_SPI_MOSI_PIN   GPIO_NUM_23
#define EPAPER_SPI_CS_PIN     GPIO_NUM_5

/*****************************************************************************/
/* DS18B20 OneWire Temperature Sensors */
/*****************************************************************************/
// Pins
#define ONE_WIRE_BUS_PIN      GPIO_NUM_12  // don't forget external pull-up (4k7 resistor between ONE_WIRE_BUS_PIN and ONE_WIRE_PWR_PIN )
#define ONE_WIRE_PWR_PIN      GPIO_NUM_14  // Output pin to enable/disable supply to the bus
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
static const char* const TEMPERATURE_SENSOR_NAMES[] = {"Vorlauftemperatur Kreis 1", 
                                                       "Rücklauftemperatur Kreis 1",
                                                       "Vorlauftemperatur Kreis 2", 
                                                       "Rücklauftemperatur Kreis 2",
                                                       "Buffertemperatur",
                                                       };

#define EPAPER_LABEL_LINE1    "Kreis 1 (V/R):"
#define EPAPER_LABEL_LINE2    "Kreis 2 (V/R):"

/*****************************************************************************/
/* digital inputs */
/*****************************************************************************/
#define DIs_LEN 1
const gpio_num_t DIs[] = { GPIO_NUM_27, };

// names for digital input signals
static const char* const DI_SENSOR_NAMES[] = {"Vorlaufpumpe Kreis 1",};

// boolean array whether or not the logic values should be inversd
const bool DI_INV[] = {true,};

// input mode array
const uint8_t DI_MODE[] = {INPUT}; // INPUT, INPUT_PULLUP, INPUT_PULLDOWN

#endif