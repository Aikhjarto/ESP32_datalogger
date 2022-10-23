#include <esp_spiffs.h>
#include <esp_err.h>
#include <esp32-hal-log.h>
#include <FS.h>
#include <SPIFFS.h>
#include "fs.h"

const char *LOGTAG_SPIFFS = "SPIFFS";

esp_err_t write_helper(fs::File file, uint8_t *data, size_t len, multi_write_log_t *log)
{
  log->bytes_written += file.write(data, len);
  log->bytes_total += len;
  return check_multi_write_log(log);
}

esp_err_t check_multi_write_log(multi_write_log_t *log)
{
  if (log->bytes_written != log->bytes_total)
  {
    return ESP_ERR_NOT_FINISHED;
  }
  else
  {
    return ESP_OK;
  }
}

/*
Reads file and optionally null-terminates string.
*/
char *readFile(fs::FS &fs, const char *path, bool null_terminated)
{
  File file = fs.open(path);
  if (!file)
  {
    ESP_LOGE(LOGTAG_SPIFFS, "Could not open %s", path);
    return NULL;
  }

  if (file.isDirectory())
  {
    file.close();
    ESP_LOGE(LOGTAG_SPIFFS, "%s is a directory", path);
    return NULL;
  }

  char *buffer = (char *)malloc((file.size() + null_terminated) * sizeof(char));
  if (!buffer)
  {
    ESP_LOGE(LOGTAG_SPIFFS, "Failed to allocate %d bytes for reading %s", (file.size() + 1) * sizeof(char), file.path());
  }
  else
  {
    ESP_LOGI(LOGTAG_SPIFFS, "Read %d bytes from %s", file.readBytes(buffer, file.size()), file.path());
    if (null_terminated) { 
    buffer[(file.size() + 1)] = '\0';
    }
  }

  file.close();
  return buffer;
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
  File root = fs.open(dirname);
  if (!root)
  {
    ESP_LOGE(LOGTAG_SPIFFS, "failed to open %s", dirname);
    return;
  }
  if (!root.isDirectory())
  {
    ESP_LOGE(LOGTAG_SPIFFS, "%s is not a directory", dirname);
    return;
  }

  ESP_LOGI(LOGTAG_SPIFFS, "Listing directory: %s", dirname);

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      ESP_LOGI(LOGTAG_SPIFFS, "  DIR: %s", file.name());
      if (levels)
      {
        listDir(fs, file.path(), levels - 1);
      }
    }
    else
    {
      ESP_LOGI(LOGTAG_SPIFFS, "  FILE: %s\t\t%d", file.name(), file.size());
    }
    file = root.openNextFile();
  }
  ESP_LOGI(LOGTAG_SPIFFS, "Directory listing done!");
}
