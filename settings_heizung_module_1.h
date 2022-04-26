#ifndef SETTINGS_HEIZUNG_MODULE_1
#define SETTINGS_HEIZUNG_MODULE_1


/*****************************************************************************/
/* DS18B20 OneWire Temperature Sensors */
/*****************************************************************************/

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
#define DIs_LEN (size_t)1 // maximum of 8 since to save memory DI states are stored as bits in a uint_8 array.
const gpio_num_t DIs[] = { GPIO_NUM_35, };

// names for digital input signals
static const char* const DI_SENSOR_NAMES[] = {"Vorlaufpumpe Kreis 1",};

// boolean array whether or not the logic values should be inversd
const bool DI_INV[] = {true,};

#endif