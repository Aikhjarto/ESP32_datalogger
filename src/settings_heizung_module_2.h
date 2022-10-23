#ifndef SETTINGS_HEIZUNG_MODULE_2
#define SETTINGS_HEIZUNG_MODULE_2

#include <stddef.h>         // for size_t
#include <esp32-hal-gpio.h> // for pin definitions

/*
Konfiguration für das ESP Modul zur Temperaturmessung der Gastherme und der Wärmepumpe
*/

/*****************************************************************************/
/* Misc Settings */
/*****************************************************************************/
//#define STATUS_LED_PIN GPIO_NUM_2 // high when operating, low when in deep-sleep
#define WAKEUP_PIN GPIO_NUM_25      // when connected to 3.3V via pushbutton, ePaper refresh is performed
#define VBAT_DIV_RATIO ((float)2.0) // 2.0 for firebeetle
#define VBAT_PIN A0                 // A0 for firebeetle, GPIO_NUM13 for Heltec WiFi Kit 32
#define VREF_PIN GPIO_NUM_34        // VBAT_DIV_RATIO voltage divider for 3.3 V to calibrate for offset
#define HISTORY_LENGTH 48
/* if SENSOR_VALUES_AS_JSON is defined, {"epoch":13949503,"value":2.33} is published instead of only 2.33 */
#define SENSOR_VALUES_AS_JSON

/*****************************************************************************/
/* Analog input */
/*****************************************************************************/
#define AIs_LEN 1
const uint8_t AIs[] = {
    (u_int8_t)35,
};
// names for analog input signals
static const char *const AI_SENSOR_NAMES[] = {
    "Photoresistor",
};

/*****************************************************************************/
/* ePaper */
/*****************************************************************************/
#define EPAPER_PWR_PIN GPIO_NUM_12  // ESP->ePaper VCC pin
#define EPAPER_BUSY_PIN GPIO_NUM_16 // ePaper->ESPhigh when ePaper is busy
#define EPAPER_RST_PIN GPIO_NUM_17  // ESP->ePaper reset pin to wake display up
#define EPAPER_DC_PIN GPIO_NUM_14   // ESP->ePaper switch between data and command transfer
#define EPAPER_SPI_CLK_PIN GPIO_NUM_18
#define EPAPER_SPI_MISO_PIN GPIO_NUM_19 // unused by this project but required for SPI configuration
#define EPAPER_SPI_MOSI_PIN GPIO_NUM_23
#define EPAPER_SPI_CS_PIN GPIO_NUM_5

/*****************************************************************************/
/* DS18B20 OneWire Temperature Sensors */
/*****************************************************************************/
// Pins
#define ONE_WIRE_BUS_PIN GPIO_NUM_2     // don't forget external pull-up (4k7 resistor between ONE_WIRE_BUS_PIN and ONE_WIRE_PWR_PIN )
#define ONE_WIRE_PWR_PIN GPIO_NUM_13    // Output pin to enable/disable supply to the bus
#define ONE_WIRE_PWR_RTC_PIN GPIO_NUM_9 // in ULP, Pin numbering is different, thus ONE_WIRE_PWR_PIN needs to be defined twice

// total number of OwnWire Temperature Sensors
const size_t OW_SERIALS_LEN = 9;

// serial numbers to enumerate for DS18B20 sensors
const uint8_t OW_SERIALS[][8] = {
    {0x28, 0x4C, 0x36, 0xA5, 0x2F, 0x21, 0x01, 0xB6},
    {0x28, 0xBC, 0xB6, 0x56, 0x2F, 0x21, 0x01, 0xF6},
    {0x28, 0xCA, 0x20, 0x83, 0x2F, 0x21, 0x01, 0x77},
    {0x28, 0xB3, 0xD0, 0x82, 0x2F, 0x21, 0x01, 0x50},
    {0x28, 0x0C, 0x1C, 0xA0, 0x2F, 0x21, 0x01, 0x07},
    {0x28, 0xEF, 0x48, 0x98, 0x2F, 0x21, 0x01, 0xA9},
    {0x28, 0xF5, 0x3A, 0x86, 0x2F, 0x21, 0x01, 0xFE},
    {0x28, 0xFF, 0x64, 0x02, 0xE9, 0x93, 0x9C, 0x2C},
    {0x28, 0xFF, 0x64, 0x02, 0xF7, 0x4A, 0xD9, 0x01},
};

// names for the MQTT topics for the temperature sensors in same order as OW_SERIALS
static const char *const TEMPERATURE_SENSOR_NAMES[] = {
    "Vorlauftemperatur Wärmepumpe",
    "Rücklauftemperatur Wärmepumpe",
    "Vorlauftemperatur Gastherme",
    "Rücklauftemperatur Gastherme",
    "Boilertemperatur oben",
    "Boilertemperatur mitte",
    "Boilertemperatur unten",
    "Ausgangstemperatur Wärmetauscher",
    "Eingangstemperatur Wärmetauscher"};

#define EPAPER_LABEL_LINE1 "Waermepumpe"
#define EPAPER_LABEL_LINE2 "Gas (V/R):"

/*****************************************************************************/
/* I2C RGB sensors */
/*****************************************************************************/
#include <Adafruit_TCS34725.h> // load constants for sensor config
//#define TCS34725_INTEGRATION_TIME TCS34725_INTEGRATIONTIME_614MS
//#define TCS34725_INTEGRATION_TIME TCS34725_INTEGRATIONTIME_120MS
#define TCS34725_GAIN TCS34725_GAIN_60X
// #define RGB_PWR_PIN GPIO_NUM_5

/*****************************************************************************/
/* digital inputs */
/*****************************************************************************/
#define DIs_LEN 11
#define PCFs_LEN 2
const uint8_t PCF_I2C_ADDRESSES[PCFs_LEN] = {0x20, 0x24};

const int DIs[DIs_LEN] = {
    -14, // K1a
    -16, // K1b
    -7,  // K2
    -8,  // K3
    -5,  // K4
    -15, // K5 Schütz defekt
    -6,  // K6
    -11, // K7
    -9,  // K8
    -4,  // K9
    -10, // K10
};

// names for digital input signals
static const char *const DI_SENSOR_NAMES[DIs_LEN] = {
    "Schütz Heizkreispumpe 1",                 // K1a
    "Schütz Heizkreispumpe 2",                 // K1b
    "Schütz Wärmepumpenausgang Buffer",        // K2
    "Schütz Wärmepumpenausgang Wärmetauscher", // K3
    "Schütz Brauchwasserumwälzpumpe",          // K4
    "Schütz Gastherme Wärmetauscher",          // K5
    "Schütz Gastherme Buffer",                 // K6
    "Schütz Umwälzpumpe",                      // K7
    "Schütz Anforderung Buffer",               // K8
    "Schütz Brauchwasserladepumpe",            // K9
    "Schütz Anforderung Boiler",               // K10
};

// boolean array whether or not the logic values should be inverted
const bool DI_INV[DIs_LEN] = {
    true,  // K1a
    true,  // K1b
    true,  // K2
    true,  // K3
    true,  // K4
    true,  // K5
    true,  // K6
    false, // K7
    false, // K8
    true,  // K9
    false, // K10
};

// input mode array
const uint8_t DI_MODE[DIs_LEN] = {
    INPUT_PULLUP, // K1a
    INPUT_PULLUP, // K1b
    INPUT_PULLUP, // K2
    INPUT_PULLUP, // K3
    INPUT_PULLUP, // K4
    INPUT_PULLUP, // K5
    INPUT_PULLUP, // K6
    INPUT_PULLUP, // K7
    INPUT_PULLUP, // K8
    INPUT_PULLUP, // K9
    INPUT_PULLUP, // K10
};                // INPUT, INPUT_PULLUP, INPUT_PULLDOWN

#endif
