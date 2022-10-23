
/*****************************************************************************/
/* OTA */
/*****************************************************************************/
#if __has_include("settings.h")
#include "settings.h"
#endif
#include "arduino_secrets.h"

#include <HTTPClient.h>
#include <esp_http_client.h>
#include <esp_https_ota.h>
#include <esp_tls.h>
#include <sstream>
#include <esp32-hal-log.h> // for ESP_LOG*
#include <esp_ota_ops.h>   // to get version and timestamp of image
#include "myLib.h"
#include <SPIFFS.h>
#include "fs.h"

const char *LOGTAG_OTA = "OTA";

/*
https://github.com/espressif/esp-idf/blob/master/examples/protocols/esp_http_client/main/esp_http_client_example.c
*/
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
  static char *output_buffer; // Buffer to store response of http request from event handler
  static int output_len;      // Stores number of bytes read
  switch (evt->event_id)
  {
  case HTTP_EVENT_ERROR:
    ESP_LOGD(LOGTAG_OTA, "HTTP_EVENT_ERROR");
    break;
  case HTTP_EVENT_ON_CONNECTED:
    ESP_LOGD(LOGTAG_OTA, "HTTP_EVENT_ON_CONNECTED");
    break;
  case HTTP_EVENT_HEADER_SENT:
    ESP_LOGD(LOGTAG_OTA, "HTTP_EVENT_HEADER_SENT");
    break;
  case HTTP_EVENT_ON_HEADER:
    ESP_LOGD(LOGTAG_OTA, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
    break;
  case HTTP_EVENT_ON_DATA:
    ESP_LOGD(LOGTAG_OTA, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
    /*
     *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
     *  However, event handler can also be used in case chunked encoding is used.
     */
    if (!esp_http_client_is_chunked_response(evt->client))
    {
      // If user_data buffer is configured, copy the response into the buffer
      if (evt->user_data)
      {
        // TODO: might overflow in user_data
        memcpy(evt->user_data + output_len, evt->data, evt->data_len);
      }
      else
      {
        if (output_buffer == NULL)
        {
          output_buffer = (char *)malloc(esp_http_client_get_content_length(evt->client));
          output_len = 0;
          if (output_buffer == NULL)
          {
            ESP_LOGE(LOGTAG_OTA, "Failed to allocate memory for output buffer");
            return ESP_FAIL;
          }
        }
        memcpy(output_buffer + output_len, evt->data, evt->data_len);
      }
      output_len += evt->data_len;
    }

    break;
  case HTTP_EVENT_ON_FINISH:
    ESP_LOGD(LOGTAG_OTA, "HTTP_EVENT_ON_FINISH");
    if (output_buffer != NULL)
    {
      // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
      // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
      free(output_buffer);
      output_buffer = NULL;
    }
    output_len = 0;
    break;
  case HTTP_EVENT_DISCONNECTED:
    ESP_LOGI(LOGTAG_OTA, "HTTP_EVENT_DISCONNECTED");
    int mbedtls_err = 0;
    esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
    if (err != 0)
    {
      ESP_LOGI(LOGTAG_OTA, "Last esp error code: 0x%x", err);
      ESP_LOGI(LOGTAG_OTA, "Last mbedtls failure: 0x%x", mbedtls_err);
    }
    if (output_buffer != NULL)
    {
      free(output_buffer);
      output_buffer = NULL;
    }
    output_len = 0;
    break;
    // // HTTP_EVENT_REDIRECT is not available in all versions of ESP IDF
    // case HTTP_EVENT_REDIRECT:
    //     ESP_LOGD(LOGTAG_OTA, "HTTP_EVENT_REDIRECT");
    //     esp_http_client_set_header(evt->client, "From", "user@example.com");
    //     esp_http_client_set_header(evt->client, "Accept", "text/html");
    //     break;
  }
  return ESP_OK;
}

/*

*/
void ota_update(const char *update_url)
{
  // assemble URL
  std::string url_str;
  std::stringstream url_ss;
  url_ss << update_url << WiFi.getHostname() << ".date";
  url_str = url_ss.str();

  // buffer for response
  char response_buffer[1000] = {0};
  const char *cert_buffer = NULL;

  if (SPIFFS.begin())
  {
    cert_buffer = readFile(SPIFFS, "/server.crt", true);
    SPIFFS.end();
  }

  if (!cert_buffer) {
    ESP_LOGE(LOGTAG_OTA, "Could not read certificate, skipping update.");
    return;
  }
  esp_http_client_config_t config = {
      .url = url_str.c_str(),
      .cert_pem = cert_buffer,
      .event_handler = _http_event_handler,
      .user_data = response_buffer};

  ESP_LOGV(LOGTAG_OTA, "Downloading firmware date from %s", config.url);
  esp_http_client_handle_t client = esp_http_client_init(&config);
  esp_err_t err = esp_http_client_perform(client);

  if (err == ESP_OK)
  {
    int status_code = esp_http_client_get_status_code(client);
    if (status_code == 200)
    {
      int content_length = esp_http_client_get_content_length(client);

      ESP_LOGV(LOGTAG_OTA, "HTTP Status %d, content_length: %d", status_code, content_length);

      ESP_LOGV(LOGTAG_OTA, "HTTP Response: %s", response_buffer);

      time_t tm_curr, tm_avail;
      const esp_app_desc_t *esp_app_desc = esp_ota_get_app_description();

      tm_avail = cvt_TIME(response_buffer);
      tm_curr = cvt_TIME(esp_app_desc->date, esp_app_desc->time);

      ESP_LOGV(LOGTAG_OTA, "Timestamp of available FW: %ld", tm_avail);
      ESP_LOGV(LOGTAG_OTA, "Timestamp of installed FW: %ld", tm_curr);

      if (tm_avail > tm_curr)
      {
        ESP_LOGI(LOGTAG_OTA, "Update available");

        url_ss.seekp(-4, url_ss.cur);
        url_ss << "bin" << '\0';
        url_str = url_ss.str();

        esp_http_client_config_t config = {
            .url = url_str.c_str(),
            .cert_pem = cert_buffer,
        };

        ESP_LOGV(LOGTAG_OTA, "Downloading firmware from %s", config.url);
        esp_err_t ret = esp_https_ota(&config);
        if (ret == ESP_OK)
        {
          esp_restart();
        }
      }
      else
      {
        ESP_LOGV(LOGTAG_OTA, "No update available");
      }
    }
    else
    {
      ESP_LOGE(LOGTAG_OTA, "Received HTTP status code %d, use esp_log_level_set(\"HTTP_CLIENT\", ESP_LOG_VERBOSE); to see more details", status_code);
    }
  }
  else
  {
    ESP_LOGE(LOGTAG_OTA, "HTTPS connection failed with status code %d ", esp_http_client_get_status_code(client));
  }
  esp_http_client_cleanup(client);
}
