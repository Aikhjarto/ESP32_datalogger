#pragma once
#include "settings.h"
#include "myLib.h"
#include <FS.h>
#include <mqtt_client.h>

#ifdef HISTORY_LENGTH
struct history_t
{
  size_t next_store_index;

  size_t MAX_LENGTH = HISTORY_LENGTH;

  time_t epochs[HISTORY_LENGTH];

#ifdef ONE_WIRE_BUS_PIN
  float temperatures[OW_SERIALS_LEN][HISTORY_LENGTH];
#endif

#ifdef AIs_LEN
  uint16_t AIs[AIs_LEN][HISTORY_LENGTH];
#endif

#if DIs_LEN <= 8
  uint8_t DIs[HISTORY_LENGTH];
#elif DIs_LEN <= 16
  uint16_t DIs[HISTORY_LENGTH];
#else
  uint32_t DIs[HISTORY_LENGTH];
#endif

#ifdef _TCS34725_H_
  uint16_t RGBCs[4][HISTORY_LENGTH];
#endif

#ifdef _TCS34725_H_
  bme_readout_t bme[HISTORY_LENGTH];
#endif

#ifdef STORE_BOOT_COUNT
  size_t boot_count[HISTORY_LENGTH];
#endif
};

void offload_history(history_t *history, time_t epoch);
std::string read_history_file(File file);
std::string get_json_from_history(history_t *history);
size_t iterate_history_files(fs::FS &fs, const char *dirname,
                             esp_mqtt_client_handle_t my_esp_mqtt_client, const char *topic);

#endif