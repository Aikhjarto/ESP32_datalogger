#pragma once

#include "arduino_secrets.h"

/*****************************************************************************/
/* Board select */
/*****************************************************************************/
#if __has_include("settings_board_select.h")
// settings_board_select.h can be generated with scripts/set_build_target
#include "settings_board_select.h"
#endif

#if defined(esp32_C7C50C) || defined(esp32_44BCD8)

#define SYSTEMINFO_PUBLISH_INTERVAL (int(600))

// initial values for wake-up timing (might get overwritten by MQTT messages)
#define SLEEP_MILLISECONDS 15000
#define WIFI_CONNECT_EVERY_Nth_LOOP 10
#define DISPLAY_UPDATE_EVERY_Nth_LOOP 5 // save power
#if defined(esp32_C7C50C)
// NodeMCU
#include "settings_heizung_module_1.h"
#elif defined(esp32_44BCD8)
// Firebeetle
#include "settings_heizung_module_2.h"
#elif defined(esp32_07EDE0)
// Heltec ESP32 Kit
#include "settings_Heltec_ESP32_Kit.h"
#endif
#else

#if defined(FIREBEETLE)
#include "settings_Firebeetle.h"
#endif

#define SLEEP_MILLISECONDS 15000
#define WIFI_CONNECT_EVERY_Nth_LOOP 10
// #define DISPLAY_UPDATE_EVERY_Nth_LOOP 30
#define HISTORY_LENGTH 50
#define HISTORY_FILE_PREFIX "/history_file_"
//#define STORE_BOOT_COUNT

#endif
