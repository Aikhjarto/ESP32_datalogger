#include "history.h"
#include <sstream>
#include <FS.h>
#include <SPIFFS.h>
#include <esp32-hal-log.h>
#include "fs.h"
#include <mqtt_client.h>

#ifdef HISTORY_LENGTH
/*
  {"len":44,"epoch":[16756006,16755006,...],"t1":[4.32,5.33,...], ...}
*/
std::string get_json_from_history(history_t *history)
{

  std::stringstream ss;
  ss << "{\"len\":" << history->next_store_index << ",\"epoch\":[";
  for (size_t i = 0; i < history->next_store_index; ++i)
  {
    ss << history->epochs[i] << ",";
  }
  if (history->next_store_index > 0)
    ss.seekp(-1, ss.cur);
  ss << "]";

#ifdef ONE_WIRE_BUS_PIN
  for (size_t j = 0; j < OW_SERIALS_LEN; j++)
  {
    ss << ",\"" << TEMPERATURE_SENSOR_NAMES[j] << "\":[";
    for (size_t i = 0; i < history->next_store_index; ++i)
    {
      ss << history->temperatures[j][i] << ",";
    }
    if (history->next_store_index > 0)
      ss.seekp(-1, ss.cur);
    ss << "]";
  }
#endif

#ifdef DIs_LEN
  for (size_t j = 0; j < (size_t)DIs_LEN; j++)
  {
    ss << ",\"" << DI_SENSOR_NAMES[j] << "\":[";
    for (size_t i = 0; i < history->next_store_index; ++i)
    {
      // Serial.printf("DIs %d: %d\r\n", i, history->DIs[i]);
      ss << ((history->DIs[i] & (0x01 << j)) > 0) << ",";
    }
    if (history->next_store_index > 0)
      ss.seekp(-1, ss.cur);
    ss << "]";
  }
#endif
#ifdef AIs_LEN
  for (size_t j = 0; j < AIs_LEN; j++)
  {
    ss << ",\"" << AI_SENSOR_NAMES[j] << "\":[";
    for (size_t i = 0; i < history->next_store_index; ++i)
    {
      ss << (int)history->AIs[j][i] << ",";
    }
    if (history->next_store_index > 0)
      ss.seekp(-1, ss.cur);
    ss << "]";
  }
#endif

#ifdef _TCS34725_H_
  ss << ",\"RGBC\":[";
  for (size_t i = 0; i < history->next_store_index; ++i)
  {
    ss << "["
       << history->RGBCs[0][i] << ","
       << history->RGBCs[1][i] << ","
       << history->RGBCs[2][i] << ","
       << history->RGBCs[3][i] << "],";
  }
  if (history->next_store_index > 0)
    ss.seekp(-1, ss.cur);
  ss << "]";
#endif

#ifdef __BME280_H__
  ss << ",\"BME280\":{";
  ss << "\"temperature\":[";
  for (size_t i = 0; i < history->next_store_index; ++i)
  {
    ss << history->bme[i].temperature << ",";
  }
  if (history->next_store_index > 0)
    ss.seekp(-1, ss.cur);
  ss << "],\"humidity\":[";
  for (size_t i = 0; i < history->next_store_index; ++i)
  {
    ss << history->bme[i].humidity << ",";
  }
  if (history->next_store_index > 0)
    ss.seekp(-1, ss.cur);
  ss << "],\"pressure\":";
  for (size_t i = 0; i < history->next_store_index; ++i)
  {
    ss << history->bme[i].pressure << ",";
  }
  if (history->next_store_index > 0)
    ss.seekp(-1, ss.cur);
  ss << "]}";
#endif

#ifdef STORE_BOOT_COUNT
  ss << ",\"boot_count\":[";
  for (size_t i = 0; i < history->next_store_index; ++i)
  {
    ss << history->boot_count[i] << ",";
  }
  if (history->next_store_index > 0)
  {
    ss.seekp(-1, ss.cur);
  }
  ss << "]";
#endif

  ss << "}";

  // Hint: don't pass a c_str around that was created from a temporal string as in ss.str().c_str(),
  // since the c_str() returns a pointer to a string object that might be freed right after this line.
  return ss.str();
}

#ifdef HISTORY_FILE_PREFIX
void offload_history(history_t *history, time_t epoch)
{
  if (SPIFFS.begin())
  {

    ESP_LOGI(LOGTAG_SPIFFS, "SPIFFS used %d/%d bytes", SPIFFS.usedBytes(), SPIFFS.totalBytes());

    File file = SPIFFS.open(String(HISTORY_FILE_PREFIX) + epoch, FILE_WRITE);
    if (!file)
    {
      ESP_LOGE(LOGTAG_SPIFFS, "failed to open file %s for writing", file.name());
      return;
    }
    else
    {
      ESP_LOGI(LOGTAG_SPIFFS, "opened file %s for writing", file.name());
    }

    multi_write_log_t log;
    write_helper(file, (uint8_t *)&history->next_store_index, sizeof(history->next_store_index), &log);
    write_helper(file, (uint8_t *)history->epochs, history->next_store_index * sizeof(time_t), &log);
#ifdef ONE_WIRE_BUS_PIN
    write_helper(file, (uint8_t *)history->temperatures,
                 history->next_store_index * OW_SERIALS_LEN * sizeof(history->temperatures[0]), &log);
#endif
#ifdef DIs_LEN
    write_helper(file, (uint8_t *)history->DIs, history->next_store_index * sizeof(history->DIs[0]), &log);
#endif
#ifdef AIs_LEN
    write_helper(file, (uint8_t *)history->AIs, history->next_store_index * AIs_LEN * sizeof(history->AIs[0]), &log);
#endif
#ifdef _TCS34725_H_
    write_helper(file, (uint8_t *)history->RGBCs, history->next_store_index * 4 * sizeof(history->RGBCs[0]), &log);
#endif
#ifdef __BME280_H__
    write_helper(file, (uint8_t *)history->bme, history->next_store_index * sizeof(history->bme[0]), &log);
#endif
#ifdef STORE_BOOT_COUNT
    write_helper(file, (uint8_t *)history->boot_count, history->next_store_index * sizeof(history->boot_count[0]), &log);
#endif
    esp_err_t err = check_multi_write_log(&log);
    if (err == ESP_OK)
    {
      ESP_LOGI(LOGTAG_SPIFFS, "%s written", file.path());
    }
    else
    {
      ESP_LOGE(LOGTAG_SPIFFS, "write to %s failed. Only %d/%d bytes written", file.path(), log.bytes_written, log.bytes_total);
    }

    file.close();

    ESP_LOGI(LOGTAG_SPIFFS, "SPIFFS used %d/%d bytes", SPIFFS.usedBytes(), SPIFFS.totalBytes());

    SPIFFS.end();
  }
}

std::string read_history_file(File file)
{

  std::string result;
  if (!file)
  {
    ESP_LOGE(LOGTAG_SPIFFS, "Failed to open file %s for reading", file.path());
  }
  else
  {
    ESP_LOGI(LOGTAG_SPIFFS, "Reading file %s", file.path());
    history_t buffer;

    file.read((uint8_t *)&(buffer.next_store_index), sizeof(size_t));
    // Serial.printf("%d timestamps\n", buffer.next_store_index);
    file.read((uint8_t *)&(buffer.epochs), buffer.next_store_index * sizeof(time_t));
    // Serial.print("Epochs: ");
    // for (size_t i = 0; i < buffer.next_store_index; i++)
    // {
    //   Serial.printf("%ld ", buffer.epochs[i]);
    // }
    // Serial.print("\n");

#ifdef ONE_WIRE_BUS_PIN
    file.read((uint8_t *)&(buffer.temperatures), buffer.next_store_index * OW_SERIALS_LEN * sizeof(buffer.temperatures[0]));
#endif

#ifdef DIs_LEN
    file.read((uint8_t *)&(buffer.DIs), buffer.next_store_index * sizeof(buffer.DIs[0]));
    // Serial.print("DIs: ");
    // for (size_t i = 0; i < buffer.next_store_index; i++)
    // {
    //   Serial.printf("%d ", buffer.DIs[i]);
    // }
    // Serial.print("\n");
#endif

#ifdef AIs_LEN
    file.read((uint8_t *)&(buffer.AIs), buffer.next_store_index * sizeof(buffer.AIs[0]));
    // Serial.print("AIs: ");
    // for (size_t j = 0; j < AIs_LEN; j++)
    // {
    //   for (size_t i = 0; i < buffer.next_store_index; i++)
    //   {
    //     Serial.printf("%d ", buffer.AIs[j][i]);
    //   }
    // }
    // Serial.print("\n");
#endif

#ifdef STORE_BOOT_COUNT
    file.read((uint8_t *)&buffer.boot_count, buffer.next_store_index * sizeof(buffer.boot_count[0]));
#endif

    ESP_LOGI(LOGTAG_SPIFFS, "Reading File done");
    result = get_json_from_history(&buffer);
    ESP_LOGD(LOGTAG_SPIFFS, "Read: %s", result.c_str());
  }

  return result;
}

size_t iterate_history_files(fs::FS &fs, const char *dirname,
                             esp_mqtt_client_handle_t my_esp_mqtt_client, const char *topic)
{
  size_t no_files = 0;

  File root = fs.open(dirname);
  if (!root)
  {
    ESP_LOGE(LOGTAG_SPIFFS, "failed to open %s", dirname);
    return 0;
  }
  if (!root.isDirectory())
  {
    ESP_LOGE(LOGTAG_SPIFFS, "%s is not a directory", dirname);
    return 0;
  }
  ESP_LOGI(LOGTAG_SPIFFS, "Listing directory: %s", dirname);

  File file = root.openNextFile();
  std::string json;
  while (file)
  {
    if (!file.isDirectory())
    {
      ESP_LOGI(LOGTAG_SPIFFS, "\tFILE: %s\t%d", file.path(), file.size());

      if (strncmp(file.path(), HISTORY_FILE_PREFIX, strlen(HISTORY_FILE_PREFIX)) == 0)
      {
        no_files++;

        if (my_esp_mqtt_client)
        {
          json = read_history_file(file);
          esp_mqtt_client_publish(my_esp_mqtt_client, topic, json.c_str(), json.length(), 1, 1);

          // todo maybe wait with deletion for QOS ack
          fs.remove(file.path());
        }
      }
    }
    file = root.openNextFile();
  }

  return no_files;
}

#endif
#endif
