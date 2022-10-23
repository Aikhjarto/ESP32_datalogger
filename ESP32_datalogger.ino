/*****************************************************************************/
/* includes */
/*****************************************************************************/
#include "src/settings.h" // configuration
#include <Arduino.h>      // for compatiblity with native esp-idf installtion
#include <esp32-hal.h>    // GPIO pins, timers, ...
#ifdef WIFI_SSID
#include <WiFi.h> // Wireless Network
#endif
#ifdef NTP_SERVER
#include <esp_sntp.h> // erspressif for NTP timesync
#endif
#ifdef MQTT_URI
#include <mqtt_client.h> // esp mqtt client (compared to PubSubClient, esp mqtt client supports QoS 1)
#endif
#include <driver/rtc_io.h>  // to avoid floating PWR pins in sleep mode
#include <driver/adc.h>     // to power off ADCs after they are not needed any more
#include <HardwareSerial.h> // for object Serial
#include <string.h>         // for strndup
#include <sstream>          // string creating via streams
#include <iomanip>          // stream settings
#include <esp32-hal-gpio.h> // for pin definitions
#include <esp_ota_ops.h>    // to get version and timestamp of image
#include "src/fs.h"
#include "src/myLib.h"
#include <FS.h>
#include <SPIFFS.h>
#include <sys/time.h> // for gettimeofday
#ifdef EPAPER_SPI_CS_PIN
#include <SPI.h> // SPI library to communicate with ePaper
#include "src/ePaper.h"
#endif
#ifdef ONE_WIRE_BUS_PIN
#include <OneWire.h>           // for DS18B20 temperature sensors, tested with modified Dallas library ...
#include <DallasTemperature.h> // ... from https://github.com/milesburton/Arduino-Temperature-Control-Library
#endif
#ifdef PCFs_LEN
#include <Wire.h>
#include <Adafruit_PCF8574.h>
Adafruit_PCF8574 pcf[PCFs_LEN];
bool pcf_available[PCFs_LEN];
#endif
#ifdef OLED_I2C_INTERFACE_NUM
#include <Adafruit_SSD1306.h> // OLED controller SSD1306 is used in may display modules
#include <Wire.h>             // I2C library to communicate with SSD1306 controller
#endif
#ifdef LCD_I2C_ADDRESS
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#endif
#ifdef LCD_RS_PIN
#include <LiquidCrystal.h>
#endif
#ifdef CONFIG_FIRMWARE_UPGRADE_URL
#include "src/ota.h"
#endif

static const char *LOGTAG_MAIN = "MAIN";
#ifdef MQTT_URI
static const char *LOGTAG_MQTT = "MQTT";
#endif
static const char *LOGTAG_SENSOR_DATA = "SENSOR";
static const char *LOGTAG_INIT = "INIT";

RTC_DATA_ATTR size_t boot_count = 0;
RTC_DATA_ATTR time_t epoch;
RTC_DATA_ATTR uint32_t last_wake_duration_ms = 0;
#ifdef SLEEP_MILLISECONDS
RTC_DATA_ATTR uint32_t sleep_milliseconds = SLEEP_MILLISECONDS;
#else
RTC_DATA_ATTR uint32_t sleep_milliseconds = 15000;
#endif
RTC_DATA_ATTR time_t last_send_system_state_time = LONG_MIN;
#ifdef WIFI_CONNECT_EVERY_Nth_LOOP
RTC_DATA_ATTR size_t WiFi_connect_every_Nth_loop = WIFI_CONNECT_EVERY_Nth_LOOP;
#else
RTC_DATA_ATTR size_t WiFi_connect_every_Nth_loop = 1;
#endif
RTC_DATA_ATTR struct timeval sleep_enter_time;
RTC_DATA_ATTR time_t last_epoch = 0;

#if defined(EPAPER_SPI_CS_PIN) || defined(OLED_I2C_INTERFACE_NUM)
RTC_DATA_ATTR size_t display_update_every_Nth_loop = DISPLAY_UPDATE_EVERY_Nth_LOOP;
#endif

#ifdef WIFI_SSID
// network
int8_t RSSI = 0;
#endif

#ifdef MQTT_URI
// MQTT
size_t published_messages_count = 0; // counter to check if everything got published before going to sleep again
size_t no_history_files = 0;
#endif

#ifdef ONE_WIRE_BUS_PIN
// temperature sensors
float temperatures[OW_SERIALS_LEN];
#endif

#ifdef VBAT_PIN
// battery level
float v_bat;
#endif

#ifdef DIs_LEN
bool di_values[DIs_LEN];
#endif

#ifdef AIs_LEN
uint16_t analog_values[AIs_LEN];
#endif

#ifdef _TCS34725_H_
uint16_t RGBC_values[4];
SemaphoreHandle_t semaphore_RGB_sensor_done;
#endif

#ifdef __BME280_H__
bme_readout_t bme_values;
SemaphoreHandle_t semaphore_BME_sensor_done;
#endif

// multitasking for network communications
SemaphoreHandle_t semaphore_MQTT_done;
SemaphoreHandle_t semaphore_WIFI_connected;
SemaphoreHandle_t semaphore_NTP_synced;

// history storage
// TODO: it would be much nicer to have these in a struct, but I couldn't get a struct containing arrays on RTC memory
#ifdef HISTORY_LENGTH
#include "src/history.h"

RTC_DATA_ATTR history_t history;
bool need_to_store_this_loop;

#endif

/*****************************************************************************/
/* read analog inputs */
/*****************************************************************************/

#ifdef AIs_LEN
void readAIs()
{
  uint16_t value;
  for (size_t j = 0; j < AIs_LEN; j++)
  {
    value = analogRead(AIs[j]);
    ESP_LOGI(LOGTAG_SENSOR_DATA, "Analog value %s: %d", AI_SENSOR_NAMES[j], value);
    analog_values[j] = value;
  }
};
#endif

/*****************************************************************************/
/* read pin DIs[j] with optionally inverting according to DI_INV[j] */
/*****************************************************************************/
#ifdef DIs_LEN
uint8_t readDI(size_t j)
{
  uint8_t raw = 0;
#ifdef PCFs_LEN
  if (DIs[j] > 0)
  {
    raw = digitalRead(DIs[j]);
  }
  else
  {
    int pcf_index = ((-1 - DIs[j]) / 8);
    if (pcf_available[pcf_index]) {
      raw = pcf[pcf_index].digitalRead((-1 - DIs[j]) % 8);
    }
    else
    {
      ESP_LOGE(LOGTAG_MAIN, "Cannot connect to PCF on address %d", PCF_I2C_ADDRESSES[pcf_index]);
    }
  }
#else
  raw = digitalRead(DIs[j]);
#endif
  ESP_LOGI(LOGTAG_SENSOR_DATA, "Digital IN %2d, GPIO %3d,\traw: %d, %s,", j, DIs[j], raw, DI_SENSOR_NAMES[j]);

  if (DI_INV[j])
  {
    ESP_LOGD(LOGTAG_SENSOR_DATA, "Inverted to %d\n", !raw);
    return !raw;
  }
  else
  {
    return raw;
  }
}
#endif

/*****************************************************************************/
/* read RGB */
/*****************************************************************************/
#ifdef _TCS34725_H_
bool readRGB(uint16_t *RGBC_values)
{
  Adafruit_TCS34725 RGB_sensor;
#ifdef RGB_PWR_PIN
  digitalWrite(RGB_PWR_PIN, HIGH);
  delay(3);
#endif
#ifdef TCS34725_INTEGRATION_TIME
  RGB_sensor.setIntegrationTime(TCS34725_INTEGRATION_TIME);
#endif
#ifdef TCS34725_GAIN
  RGB_sensor.setGain(TCS34725_GAIN);
#endif
  if (RGB_sensor.begin(TCS34725_ADDRESS, &Wire))
  {
    delay(30); // wait for sensor to boot
    RGB_sensor.getRawData(&RGBC_values[0], &RGBC_values[1], &RGBC_values[2], &RGBC_values[3]);
    ESP_LOGI(LOGTAG_SENSOR_DATA, "RGBC: (%d,%d,%d,%d)", RGBC_values[0], RGBC_values[1], RGBC_values[2], RGBC_values[3]);
    RGB_sensor.disable(); // send to sleep
    return true;
  }
  else
  {
    ESP_LOGE(LOGTAG_MAIN, "RGB Sensor not found");
    return false;
  }
}

void readRGBTaskFunction(void *pvParameters)
{
  readRGB((uint16_t *)pvParameters);

  xSemaphoreGive(semaphore_RGB_sensor_done);
  for (;;)
  {
    delay(1000);
  };

  vTaskDelete(NULL);
}
#endif

/*****************************************************************************/
/* read BME */
/*****************************************************************************/
#ifdef __BME280_H__
void readBME280()
{
#ifdef BME_PWR_PIN
  pinMode(BME_PWR_PIN, OUTPUT);
  digitalWrite(BME_PWR_PIN, HIGH);
#endif
  Adafruit_BME280 bme; // use I2C interface
  bme.begin();
  bme_values.temperature = bme.readTemperature();
  bme_values.humidity = bme.readHumidity();
  bme_values.pressure = bme.readPressure();

#ifdef BME_PWR_PIN
  digitalWrite(BME_PWR_PIN, LOW);
#endif
}

void readBMETaskFunction(void *pvParameters)
{
  readBME280();

  xSemaphoreGive(semaphore_BME_sensor_done);
  for (;;)
  {
    delay(1000);
  };

  vTaskDelete(NULL);
}
#endif

/*****************************************************************************/
/* convert readouts to strings */
/*****************************************************************************/
#ifdef SENSOR_VALUES_AS_JSON
#ifdef __BME280_H__
std::string get_sensor_value_string(time_t epoch, bme_readout_t value)
{
  std::stringstream ss;
  ss << "{\"epoch\":" << epoch << ",\"value\":{";
  ss << "\"temperature\":" << value.temperature;
  ss << ",\"humidity\":" << value.humidity;
  ss << ",\"pressure\":" << value.pressure;
  ss << "}";
  return ss.str();
}
#endif

template <typename T>
std::string get_sensor_value_string(time_t epoch, T value)
{
  std::stringstream ss;
  ss << "{\"epoch\":" << epoch << ",\"value\":" << value << "}";
  return ss.str();
}

template <typename T>
std::string get_sensor_value_string(time_t epoch, T *values, size_t len)
{
  std::stringstream ss;
  ss << "{\"epoch\":" << epoch << ",\"value\":[";
  for (size_t i = 0; i < len; i++)
  {
    ss << values[i] << ",";
  }
  if (len > 0)
    ss.seekp(-1, ss.cur);
  ss << "]}";
  return ss.str();
}

#else
#ifdef __BME280_H__
std::string get_sensor_value_string(time_t epoch, bme_readout_t value)
{
  std::stringstream ss;
  ss << value.temperature << "," << value.humidity << "," << value.pressure;
  return ss.str();
}
#endif

template <typename T>
std::string get_sensor_value_string(time_t epoch, T value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template <typename T>
std::string get_sensor_value_string(time_t epoch, T *values, size_t len)
{
  std::stringstream ss;
  for (size_t i = 0; i < len; i++)
  {
    ss << values[i] << ",";
  }
  if (len > 0)
    ss.seekp(-1, ss.cur);
  return ss.str();
}

#endif

/*****************************************************************************/
/* History storage */
/*****************************************************************************/
#ifdef HISTORY_LENGTH

int append_to_history(history_t *history)
{
  if (history->next_store_index >= history->MAX_LENGTH)
  {
#ifdef HISTORY_FILE_PREFIX
    offload_history(history, epoch);
#endif
    history->next_store_index = 0; // wrap around
  }
#ifdef ONE_WIRE_BUS_PIN
  for (size_t j = 0; j < OW_SERIALS_LEN; j++)
  {
    history->temperatures[j][history->next_store_index] = temperatures[j];
  }
#endif

#ifdef DIs_LEN
  for (size_t j = 0; j < (size_t)DIs_LEN; j++)
  {
    // store as bytes
    if (di_values[j])
    {
      history->DIs[history->next_store_index] |= 0x01 << j;
    }
    else
    {
      history->DIs[history->next_store_index] &= ~(0x01 << j);
    }
  }
#endif

#ifdef AIs_LEN
  for (size_t j = 0; j < AIs_LEN; j++)
  {
    history->AIs[j][history->next_store_index] = analog_values[j];
  }
#endif

#ifdef _TCS34725_H_
  history->RGBCs[0][history->next_store_index] = RGBC_values[0];
  history->RGBCs[1][history->next_store_index] = RGBC_values[1];
  history->RGBCs[2][history->next_store_index] = RGBC_values[2];
  history->RGBCs[3][history->next_store_index] = RGBC_values[3];
#endif

#ifdef __BME280_H__
  history->bme[history->next_store_index] = bme_values;
#endif

#ifdef STORE_BOOT_COUNT
  history->boot_count[history->next_store_index] = boot_count;
#endif

  history->epochs[history->next_store_index] = epoch;
  return history->next_store_index++;
}

#endif

/*****************************************************************************/
/* I2C bus enumeration*/
/*****************************************************************************/

#if defined(OLED_I2C_INTERFACE_NUM) || defined(LCD_I2C_ADDRESS) || defined(_TCS34725_H_) || defined(PCFs_LEN)
/*
Copied from Arduino's examples regarding Wire.h, as including the ino file does not work
*/
int I2CSearch(TwoWire *twi)
{
  byte error, address;
  int nDevices = 0;

  ESP_LOGI(LOGTAG_INIT, "Scanning for I2C devices ...");
  for (address = 0x01; address < 0xFF; address++)
  {
    twi->beginTransmission(address);
    error = twi->endTransmission();
    if (error == 0)
    {
      ESP_LOGI(LOGTAG_INIT, "I2C device found at address 0x%02X", address);
      nDevices++;
    }
    else if (error != 2)
    {
      ESP_LOGD(LOGTAG_INIT, "Error %d at address 0x%02X", error, address);
    }
  }
  if (nDevices > 0)
  {
    ESP_LOGI(LOGTAG_INIT, "%d I2C devices found", nDevices);
  }
  else
  {
    ESP_LOGE(LOGTAG_INIT, "%d I2C devices found", nDevices);
  }
  return nDevices;
}

void findI2CDevices()
{
#ifdef LCD_I2C_ADDRESS
#ifdef LCD_SDA_PIN
  Wire.setPins(LCD_SDA_PIN, LCD_SCL_PIN);
#endif
  Wire.begin();
  I2CSearch(&Wire);
  Wire.end();
  Wire.setPins(-1, -1); // revert to default
#endif

#if defined(_TCS34725_H_) || defined(PCFs_LEN)
#ifdef CUSTOM_SDA_PIN
  Wire.setPins(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
#endif
  Wire.begin();
  I2CSearch(&Wire);
  Wire.end();
  Wire.setPins(-1, -1); // revert to default
#endif

#ifdef OLED_I2C_INTERFACE_NUM
#ifdef OLED_SDA_PIN
  Wire1.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  I2CSearch(&Wire1);
  Wire1.end();
  Wire1.setPins(-1, -1); // revert to default
#endif
#endif

}

#endif

/*****************************************************************************/
/* OneWire bus enumeration */
/*****************************************************************************/
#ifdef ONE_WIRE_BUS_PIN
/*
Enumerates the serials of devices connected to the OneWire bus and prints them
to the Serial interface in a formate suitable to copy-paste them into C-code.

This function is copies from the example "oneWireSearch.ino" from the Dallas
Temperature library, since  including the ino-file is not possible.
*/
uint8_t findOneWireDevices(int pin)
{
  OneWire ow(pin);

  uint8_t address[8];
  uint8_t count = 0;

  if (ow.search(address))
  {
    Serial.print("\nuint8_t pin");
    Serial.print(pin, DEC);
    Serial.println("[][8] = {");
    do
    {
      count++;
      Serial.println("  {");
      for (uint8_t i = 0; i < 8; i++)
      {
        Serial.print("0x");
        if (address[i] < 0x10)
          Serial.print("0");
        Serial.print(address[i], HEX);
        if (i < 7)
          Serial.print(", ");
      }
      Serial.println("},");
    } while (ow.search(address));

    Serial.println("};");
    Serial.print("// nr devices found: ");
    Serial.println(count);
  }

  return count;
}

/*****************************************************************************/
/* OneWire temperature sensors */
/*****************************************************************************/

/*
Get temperature from a DS18B20 sensor with a given serial.

In case of an error, a sensor is read-out again at most 10 times with a 10 ms
interval. Return NAN when no valid (-127.0 to 85.0 degree) value could be read.
*/
float getTemperature(DallasTemperature *DS18B20, const uint8_t *serial)
{
  int cnt = 10; // maximum tries
  float temp = 85.0;

  temp = DS18B20->getTempC(serial);
  while ((temp == 85.0 || temp == (-127.0)) && --cnt)
  {
    DS18B20->requestTemperatures();
    temp = DS18B20->getTempC(serial);
    delay(10);
  }
  if (cnt == 0)
  {
    return NAN;
  }
  else
  {
    return temp;
  }
}

void readTemperatures(int pin)
{
  OneWire oneWire(pin);
  DallasTemperature DS18B20(&oneWire);
#ifdef ONE_WIRE_PWR_PIN
  pinMode(ONE_WIRE_PWR_PIN, OUTPUT);
  digitalWrite(ONE_WIRE_PWR_PIN, HIGH);
#endif
  DS18B20.begin();
  DS18B20.requestTemperatures(); // send command to all devices to read temperatures
  for (uint8_t i = 0; i < OW_SERIALS_LEN; ++i)
  {
    temperatures[i] = getTemperature(&DS18B20, OW_SERIALS[i]);
    ESP_LOGI(LOGTAG_SENSOR_DATA, "Onewire sensor %d: %f", i, temperatures[i]);
  }

#ifdef ONE_WIRE_PWR_PIN
  digitalWrite(ONE_WIRE_PWR_PIN, LOW);
#endif
}

#endif

/*****************************************************************************/
/* MQTT */
/*****************************************************************************/
#ifdef MQTT_URI
/*
Callback for incoming MQTT messages of subscribed topics.
*/
void process_subscription(esp_mqtt_event_handle_t event)
{
  char *p = event->topic + (event->topic_len - 1); // move p to end of topic string
  int i = 0;

  // move p backwards until "/" is hit
  for (i = 1; i <= event->topic_len; i++)
  {
    if (*p == '/')
    {
      p++;
      i--;
      break;
    }
    else
      p--;
  }
  // assert: *p points to la>st '/' in topic name (or event->topic if it did not contain a '/')
  // assert: i holds len of topic name starting from *p

  // printf("TOPIC_SUFFIX=%.*s\n", i, p);

  // make a c-string out of data to be able to process it with strtol
  char *data = strndup(event->data, event->data_len);

  // temporary variable to catch error of strtol
  int tmp_convert;

  if (strncmp(p, "sleep_milliseconds", i) == 0)
  {
    if ((str2int(&tmp_convert, data, 10) == SUCCESS) && tmp_convert > 0)
    {
      sleep_milliseconds = tmp_convert;
      ESP_LOGI(LOGTAG_MQTT, "sleep_milliseconds=%d", sleep_milliseconds);
    }
    else
    {
      ESP_LOGI(LOGTAG_MQTT, "%s could not be interpreted as integer", data);
    }
  }

  if (strncmp(p, "WiFi_connect_every_Nth_loop", i) == 0)
  {
    if ((str2int(&tmp_convert, data, 10) == SUCCESS) && tmp_convert > 0)
    {
      WiFi_connect_every_Nth_loop = tmp_convert;
      ESP_LOGI(LOGTAG_MQTT, "WiFi_connect_every_Nth_loop=%d", WiFi_connect_every_Nth_loop);
    }
    else
    {
      ESP_LOGI(LOGTAG_MQTT, "%s could not be interpreted as integer", data);
    }
  }
#if defined(EPAPER_SPI_CS_PIN) || defined(OLED_I2C_INTERFACE_NUM)
  if (strncmp(p, "display_update_every_Nth_loop", i) == 0)
  {
    if ((str2int(&tmp_convert, data, 10) == SUCCESS) && tmp_convert > 0)
    {
      display_update_every_Nth_loop = tmp_convert;
      ESP_LOGI(LOGTAG_MQTT, "display_update_every_Nth_loop=%d", display_update_every_Nth_loop);
    }
    else
    {
      ESP_LOGI(LOGTAG_MQTT, "%s could not be interpreted as integer", data);
    }
  }
#endif

  free(data);
}

/*
Callback to be executed when the MQTT client has connected to the broker.
Here, subscription is done.
*/
static void mqtt_event_handler_connected(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  ESP_LOGD(LOGTAG_MQTT, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  esp_mqtt_client_handle_t client = event->client;

  // generate prefix with MAC address
  std::string sub_prefix = MQTT_PREFIX "/" + getMACasString();
  ESP_LOGI(LOGTAG_MQTT, "%s", sub_prefix.c_str());

  esp_mqtt_client_subscribe(client,
                            (sub_prefix + "/WiFi_connect_every_Nth_loop").c_str(),
                            1);
  esp_mqtt_client_subscribe(client,
                            (sub_prefix + "/sleep_milliseconds").c_str(),
                            1);
#if defined(EPAPER_SPI_CS_PIN) || defined(OLED_I2C_INTERFACE_NUM)
  esp_mqtt_client_subscribe(client,
                            (sub_prefix + "/display_update_every_Nth_loop").c_str(),
                            1);
#endif
}

void esp_mqtt_client_publish_wrapper(esp_mqtt_client_handle_t client, const char *topic, const char *data, int len, int qos, int retain)
{

  if (esp_mqtt_client_publish(client, topic, data, len, qos, retain) < 0)
  {
    ESP_LOGE(LOGTAG_MQTT, "Cannot publish");
  }
}

/*
Callback for MQTT events that are not the subscribe-event.
*/
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  ESP_LOGD(LOGTAG_MQTT, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  // esp_mqtt_client_handle_t client = event->client;
  int messages_to_wait_for_being_published = 4;

  ESP_LOGD(LOGTAG_MQTT, "event_id:%d, topic_len:%d, data_len:%d msg_id:%d",
           (esp_mqtt_event_id_t)event_id,
           event->topic_len,
           event->data_len,
           event->msg_id);
  // for (int i=0; i<event->topic_len; i++){
  //   Serial.println(event->topic[i]);
  // }

  //   for (int i=0; i<event->data_len; i++){
  //   Serial.println(event->data[i]);
  // }

  switch ((esp_mqtt_event_id_t)event_id)
  {
  case MQTT_EVENT_CONNECTED:
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(LOGTAG_MQTT, "MQTT_EVENT_DISCONNECTED");
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(LOGTAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(LOGTAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(LOGTAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
#ifdef VBAT_PIN
    messages_to_wait_for_being_published++;
#endif
#ifdef ONE_WIRE_BUS_PIN
    messages_to_wait_for_being_published += OW_SERIALS_LEN;
#endif
#ifdef DIs_LEN
    messages_to_wait_for_being_published += (size_t)DIs_LEN;
#endif
#ifdef AIs_LEN
    messages_to_wait_for_being_published += AIs_LEN;
#endif
#ifdef _TCS34725_H_
    messages_to_wait_for_being_published++;
#endif
#ifdef __BME280_H__
    messages_to_wait_for_being_published++;
#endif
#ifdef HISTORY_LENGTH
    messages_to_wait_for_being_published += (history.next_store_index > 0);
#endif
#ifdef SYSTEMINFO_PUBLISH_INTERVAL
    if ((last_send_system_state_time + SYSTEMINFO_PUBLISH_INTERVAL <= epoch) || last_send_system_state_time == epoch)
    {
      messages_to_wait_for_being_published++;
    }
#endif
#ifdef HISTORY_FILE_PREFIX
    messages_to_wait_for_being_published += no_history_files;
#endif

    published_messages_count++;
    ESP_LOGD(LOGTAG_MQTT, "Number of published Messages: %d", published_messages_count);
    if (published_messages_count == messages_to_wait_for_being_published)
    {
      xSemaphoreGive(semaphore_MQTT_done);
    }
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(LOGTAG_MQTT, "MQTT_EVENT_DATA");
    process_subscription(event);

    // printf("TOPIC=%.*s\n", event->topic_len, event->topic);
    // printf("DATA=%.*s\n", event->data_len, event->data);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(LOGTAG_MQTT, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
    {
      ESP_LOGI(LOGTAG_MQTT, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
    }
    break;
  default:
    ESP_LOGI(LOGTAG_MQTT, "Other event id:%d", event->event_id);
    break;
  }
}

void MQTT_communication()
{

  int out_buffer_size = 1024;
#ifdef HISTORY_LENGTH
  // get length of json encoded history, since it might be larger than the default
  // output buffer size of 1024 bytes used by the MQTT library
  // The MQTT library does not check the length properly and might produce a
  // buffer-overflow.
  std::string json;
  int json_len = 0;
  if (history.next_store_index > 0)
  {
    json = get_json_from_history(&history);
    json_len = json.length();
    if (json_len + 1 > out_buffer_size)
    {
      out_buffer_size = json_len + 1;
    }
  }
#endif

  // setup client
  esp_mqtt_client_config_t mqtt_cfg = {
      .uri = MQTT_URI,
#ifdef MQTT_USERNAME
      .username = MQTT_USERNAME,
#endif
#ifdef MQTT_PASSWORD
      .password = MQTT_PASSWORD,
#endif
      .out_buffer_size = out_buffer_size,
      .network_timeout_ms = 1000,
  };
  esp_mqtt_client_handle_t my_esp_mqtt_client;
  my_esp_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

  // TODO reduce timeout in case MQTT broker was down
  // connect to broker
  if (esp_mqtt_client_start(my_esp_mqtt_client) == ESP_OK)
  {
    ESP_LOGI(LOGTAG_MQTT, "MQTT connected");

    // register event handler to subscribe
    esp_mqtt_client_register_event(my_esp_mqtt_client, MQTT_EVENT_CONNECTED, mqtt_event_handler_connected, NULL);

    // register event handler to check for PUBACK messages.
    esp_mqtt_client_register_event(my_esp_mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);

#ifdef ONE_WIRE_BUS_PIN
    // publish temperatures
    for (uint8_t i = 0; i < OW_SERIALS_LEN; ++i)
    {
      // publish epoch and temperature to separate topics
      ESP_LOGI(LOGTAG_MQTT, "Publishing temperature: %s: %f", bytes2hex(OW_SERIALS[i], 8).c_str(), temperatures[i]);
      esp_mqtt_client_publish(my_esp_mqtt_client,
                              (MQTT_PREFIX "/sensors/" + String(TEMPERATURE_SENSOR_NAMES[i])).c_str(),
                              (get_sensor_value_string(epoch, temperatures[i])).c_str(), 0, 1, 0);
    }
#endif

#ifdef DIs_LEN
    // publish digital input states
    for (size_t j = 0; j < (size_t)DIs_LEN; j++)
    {

      esp_mqtt_client_publish(my_esp_mqtt_client,
                              (MQTT_PREFIX "/sensors/" + String(DI_SENSOR_NAMES[j])).c_str(),
                              (get_sensor_value_string(epoch, di_values[j])).c_str(),
                              0, 1, 0);
    }
#endif

#ifdef AIs_LEN
    // publish analog input values
    for (size_t j = 0; j < AIs_LEN; j++)
    {
      esp_mqtt_client_publish(my_esp_mqtt_client,
                              (MQTT_PREFIX "/sensors/" + String(AI_SENSOR_NAMES[j])).c_str(),
                              (get_sensor_value_string(epoch, analog_values[j])).c_str(),
                              0, 1, 0);
    }
#endif

#ifdef _TCS34725_H_
    // publish RGBC values
    esp_mqtt_client_publish(my_esp_mqtt_client,
                            MQTT_PREFIX "/sensors/RGBC",
                            (get_sensor_value_string(epoch, RGBC_values, 4)).c_str(),
                            0, 1, 0);
#endif

#ifdef __BME280_H__
    // publish values
    esp_mqtt_client_publish(my_esp_mqtt_client,
                            MQTT_PREFIX "/sensors/BME280",
                            (get_sensor_value_string(epoch, bme_values)).c_str(),
                            0, 1, 0);
#endif

    // publish system state
    std::string pub_prefix = MQTT_PREFIX "/" + getMACasString();

    esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/epoch").c_str(), String(epoch).c_str(), 0, 1, 0);
    esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/RSSI").c_str(), String(RSSI).c_str(), 0, 1, 0);
    esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/last_duration").c_str(), String(last_wake_duration_ms).c_str(), 0, 1, 0);
    esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/boot_count").c_str(), String(boot_count).c_str(), 0, 1, 0);

#ifdef SYSTEMINFO_PUBLISH_INTERVAL
    if (last_send_system_state_time + SYSTEMINFO_PUBLISH_INTERVAL <= epoch)
    {
      esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/systeminfo").c_str(), systeminfo_json().c_str(), 0, 1, 0);
      last_send_system_state_time = epoch;
    }
#endif
#ifdef VBAT_PIN
    esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/vbat").c_str(), String(v_bat).c_str(), 0, 1, 0);
#endif

#ifdef HISTORY_LENGTH
// published data that accumulated while no WiFi was available
#ifdef HISTORY_FILE_PREFIX
    if (SPIFFS.begin())
    {

      // count history files to know beforehand how many PUBACKs we need to expect
      no_history_files = iterate_history_files(SPIFFS, "/", NULL, NULL);

      // actually publish them
      iterate_history_files(SPIFFS, "/", my_esp_mqtt_client, (pub_prefix + "/history").c_str());
      ESP_LOGI(LOGTAG_SPIFFS, "SPIFFS done");
      SPIFFS.end();
    }
    else
    {
      ESP_LOGE(LOGTAG_SPIFFS, "mount of SPIFFS failed");
    }
#endif

    if (history.next_store_index > 0)
    {
      ESP_LOGI(LOGTAG_MQTT, "Publishing history from RTC memory");
      esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/history").c_str(), json.c_str(), json_len, 1, 1);
    }
#endif

    ESP_LOGI(LOGTAG_MQTT, "MQTT waiting for all PUBACKs");
    if (xSemaphoreTake(semaphore_MQTT_done, pdMS_TO_TICKS(10000)) != pdTRUE)
    {
      ESP_LOGW(LOGTAG_MQTT, "MQTT publishing took too long, aborting to save power!");
    }
    else
    {
#ifdef HISTORY_LENGTH
      // everything was published, thus purge history
      need_to_store_this_loop = false;
      history.next_store_index = 0;
#endif
      ESP_LOGI(LOGTAG_MQTT, "MQTT publishing finished, disconnecting");
    }
    esp_mqtt_client_stop(my_esp_mqtt_client);
  }
  else
  {
    ESP_LOGW(LOGTAG_MQTT, "MQTT connection failed");
  }
}
#endif

/*****************************************************************************/
/* Sleep and Wakeup */
/*****************************************************************************/
/*
Setup stuff that needs to be performed only after a hard-reset and not after a deep-sleep.
This included configuration of input and output pins and RTC-memory.
*/
void first_boot_init()
{
  // print important configurable (at compile time) hardware setting
  ESP_LOGI(LOGTAG_INIT, "WiFi MAC: %s", getMACasString().c_str());
  ESP_LOGI(LOGTAG_INIT, "CPU Frequency: %d MHz", getCpuFrequencyMhz());
  ESP_LOGI(LOGTAG_INIT, "XTAL Frequency: %d MHz", getXtalFrequencyMhz());
  ESP_LOGI(LOGTAG_INIT, "APB Frequency: %d MHz", getApbFrequency() / 1000000);
#ifdef EPAPER_SPI_CS_PIN
  ESP_LOGI(LOGTAG_INIT, "SPI Clock Divider: %d", SPI.getClockDivider());
#endif
  ESP_LOGI(LOGTAG_INIT, "Setup task started on core %d", xPortGetCoreID());
  ESP_LOGI(LOGTAG_INIT, "Compile Timestamp %s %s", __DATE__, __TIME__);

  const esp_app_desc_t *esp_app_desc = esp_ota_get_app_description();
  ESP_LOGI(LOGTAG_INIT, "FW name: %s; Project Name: %s; IDF ver: %s; Firmware Timestamp: %s, %s",
           esp_app_desc->version,
           esp_app_desc->project_name,
           esp_app_desc->idf_ver,
           esp_app_desc->time,
           esp_app_desc->date);

#if defined(OLED_I2C_INTERFACE_NUM) || defined(LCD_I2C_ADDRESS) || defined(_TCS34725_H_) || defined(PCFs_LEN)
  findI2CDevices();
#endif

#ifdef ONE_WIRE_BUS_PIN
  pinMode(ONE_WIRE_BUS_PIN, INPUT);
#ifdef ONE_WIRE_PWR_PIN
  pinMode(ONE_WIRE_PWR_PIN, OUTPUT);
  digitalWrite(ONE_WIRE_PWR_PIN, HIGH);
#endif
  ESP_LOGI(LOGTAG_INIT, "One wire bus enumeration");
  uint8_t tmp;
  tmp = findOneWireDevices(ONE_WIRE_BUS_PIN);
  if (tmp > 0)
  {
    ESP_LOGI(LOGTAG_INIT, "One wire bus enumeration found %d devices", tmp);
  }
  else
  {
    ESP_LOGE(LOGTAG_INIT, "One wire bus enumeration found no devices");
  }
#ifdef ONE_WIRE_PWR_PIN
  digitalWrite(ONE_WIRE_PWR_PIN, LOW);
#endif
#endif

  if (SPIFFS.begin())
  {
    listDir(SPIFFS, "/", 1);

    char *file_content = readFile(SPIFFS, "/server.crt");
    if (file_content)
    {
      ESP_LOGI(LOGTAG_INIT, "Server cert found: %s", file_content);
      free(file_content);
    }
  }
  // avoid floating PWR pin for onewire sensors in sleep mode
  // this reduce power consumption
  // https://geeks-r-us.de/2020/07/10/esp32-stromverbrauch-im-deep-sleep
  // rtc_gpio_init(ONE_WIRE_PWR_RTC_PIN);
  // rtc_gpio_hold_en(ONE_WIRE_PWR_RTC_PIN);
  // rtc_gpio_set_direction(ONE_WIRE_PWR_RTC_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
  // rtc_gpio_set_level(ONE_WIRE_PWR_RTC_PIN, 0);
  // rtc_gpio_isolate();

  // use a faster but more accurate frequency source for better clock stability
  // in sleep at the expense of ~5µA more current consumption
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html#config-esp32-rtc-clk-src
  // CONFIG_ESP32_RTC_CLK_SRC(ESP32_RTC_CLK_SRC_INT_8MD256);
  // TODO: this seems only to be working when compiling a new kernel

#ifdef EPAPER_SPI_CS_PIN
  epaper.init_background_image(EPAPER_LABEL_LINE1, EPAPER_LABEL_LINE2);
#endif

  ESP_LOGI(LOGTAG_INIT, "First boot init done");
}

/*****************************************************************************/
/* WiFi */
/*****************************************************************************/
#ifdef WIFI_SSID
void IRAM_ATTR WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    ESP_LOGI(LOGTAG_MAIN, "Connected to access point");
    xSemaphoreGive(semaphore_WIFI_connected);
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    ESP_LOGI(LOGTAG_MAIN, "Disconnected from WiFi access point");
    break;
  case SYSTEM_EVENT_AP_STADISCONNECTED:
    ESP_LOGI(LOGTAG_MAIN, "WiFi client disconnected");
    break;
  default:
    break;
  }
}
#endif

#ifdef NTP_SERVER
/*
Callback to give semaphore once NTP sync is completed
*/
void timesync_callback(struct timeval *tv)
{
  ESP_LOGI(LOGTAG_MAIN, "Timesync callback with status: %d ", sntp_get_sync_status());
  xSemaphoreGive(semaphore_NTP_synced);
}
#endif

/*****************************************************************************/
/* main */
/*****************************************************************************/

void wait_for_sensors()
{
#ifdef _TCS34725_H_
  // wait for RGB sensor
  if (!xSemaphoreTake(semaphore_RGB_sensor_done, pdMS_TO_TICKS(2000)))
  {
    ESP_LOGE(LOGTAG_MAIN, "RGB sensor task timed out");
  }
#endif

#ifdef __BME280_H__
  // wait for BME sensor
  if (!xSemaphoreTake(semaphore_BME_sensor_done, pdMS_TO_TICKS(1000)))
  {
    ESP_LOGE(LOGTAG_MAIN, "BME sensor tasked time out");
  }
#endif
}

void setup()
{
  struct tm timeinfo;
  struct timeval tv_now;
  char strftime_buf[64];

  bool connect_to_WiFi_this_loop;
#if defined(EPAPER_SPI_CS_PIN) || defined(OLED_I2C_INTERFACE_NUM)
  bool update_display_this_loop;
#endif
  size_t update_multiplier = 1;

  ++boot_count;

  esp_log_level_set(LOGTAG_SPIFFS, ESP_LOG_VERBOSE);

  if (digitalRead(GPIO_NUM_15))
  {
    esp_log_level_set(LOGTAG_MAIN, ESP_LOG_VERBOSE);
    esp_log_level_set(LOGTAG_SENSOR_DATA, ESP_LOG_INFO);
#ifdef EPAPER_SPI_CS_PIN
    esp_log_level_set(LOGTAG_EPAPER, ESP_LOG_VERBOSE);
#endif
    esp_log_level_set("HTTP_CLIENT", ESP_LOG_VERBOSE);
#ifdef CONFIG_FIRMWARE_UPGRADE_URL
    esp_log_level_set(LOGTAG_OTA, ESP_LOG_VERBOSE);
#endif
  }

#ifdef STATUS_LED_PIN
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
#endif
  Serial.begin(115200);

  // first-boot if wakeup not from time or wakeup button
  // Hint: don't check boot-count as i might overflow
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER &&
      esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0)
  {
    if (digitalRead(GPIO_NUM_15))
    {
      esp_log_level_set(LOGTAG_INIT, ESP_LOG_DEBUG);
    }
    first_boot_init();
  }

  print_wakeup_reason(LOGTAG_MAIN);
  localtime_r(&epoch, &timeinfo); // convert epoch to local time

  // print sleep duration
  gettimeofday(&tv_now, NULL);
  ESP_LOGI(LOGTAG_MAIN, "sleep_enter_time: %ld, %06ld", sleep_enter_time.tv_sec, sleep_enter_time.tv_usec);
  ESP_LOGI(LOGTAG_MAIN, "sleep_enter_time: %ld, %06ld", tv_now.tv_sec, tv_now.tv_usec);
  // print epoch, i.e time of internal clock
  strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  ESP_LOGI(LOGTAG_MAIN, "The current date/time is: %s", strftime_buf);

#ifdef HISTORY_LENGTH
  need_to_store_this_loop = true;
  ESP_LOGI(LOGTAG_MAIN, "History length: %d/%d", history.next_store_index, history.MAX_LENGTH);

  // don't update display and send data via WiFi as often during nighttime since probably no-one will check
  if (timeinfo.tm_year > 2016) // same check in arduino's NTP client to see if timesync was successful
  {
    if (timeinfo.tm_hour > 22 || timeinfo.tm_hour < 5)
    {
      // increase duration between history transfers as long as history length supports it
      if (WiFi_connect_every_Nth_loop * 4 < HISTORY_LENGTH)
      {
        update_multiplier *= 4;
      }
      else if (WiFi_connect_every_Nth_loop * 4 < HISTORY_LENGTH)
      {
        update_multiplier *= 2;
      }
    }
  }
#endif

#ifdef CUSTOM_SDA_PIN
  Wire.setPins(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
#endif

#ifdef WIFI_SSID
  // determine if WiFi connection is scheduled for this loop
  connect_to_WiFi_this_loop = (boot_count % (update_multiplier * WiFi_connect_every_Nth_loop)) == 1 ||
                              esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0;
#else
  connect_to_WiFi_this_loop = false;
#endif

  // if (!connect_to_WiFi_this_loop && !update_display_this_loop) {
  //   // reduce clock speed (and therefore power consumption), when only sensors are red
  //   // since most of the time we are waiting for temperature sensor conversion.
  //   //setCpuFrequencyMhz(10); // TODO: immediately causes reboot
  //   //setCpuFrequencyMhz(20); // TODO: immediately causes reboot
  //   //setCpuFrequencyMhz(40); // TODO: immediately causes reboot
  // }

  // greeting message
  ESP_LOGI(LOGTAG_MAIN, "BC: %d, connect to WiFi %d",
           boot_count,
           connect_to_WiFi_this_loop);

// determine if display update is scheduled for this loop
#if defined(EPAPER_SPI_CS_PIN) || defined(OLED_I2C_INTERFACE_NUM)
  update_display_this_loop = (boot_count % (update_multiplier * display_update_every_Nth_loop)) == 1 ||
                             esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0;
  ESP_LOGI(LOGTAG_MAIN, "update display: %d", update_display_this_loop);
#endif

#if defined(EPAPER_SPI_CS_PIN)
  if (update_display_this_loop)
  {
#ifdef EPAPER_PWR_PIN
    epaper.setPowerPin(EPAPER_PWR_PIN);
#endif
#ifdef VBAT_PIN
    epaper.setHasVBat(true);
#endif
    epaper.AssignControlPins(EPAPER_RST_PIN, EPAPER_DC_PIN, EPAPER_BUSY_PIN);
    epaper.AssignSPIPins(EPAPER_SPI_CLK_PIN, EPAPER_SPI_MISO_PIN, EPAPER_SPI_MOSI_PIN, EPAPER_SPI_CS_PIN);
  }

  // eInk display
#ifdef EPAPER_SPI_CS_PIN
  queue_content queue_struct;
  if (update_display_this_loop)
  {
    xTaskCreate(ePaperTaskFunction, /* Task function. */
                "ePaperTask",       /* name of task. */
                10000,              /* Stack size of task */
                NULL,               /* parameter of the task */
                1,                  /* priority of the task */
                &ePaperTaskHandle); /* Task handle to keep track of created task */
  }
#endif

#endif

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
  {
    esp_log_level_set(LOGTAG_INIT, ESP_LOG_DEBUG);
#ifdef ONE_WIRE_BUS_PIN
#ifdef ONE_WIRE_PWR_PIN
    pinMode(ONE_WIRE_PWR_PIN, OUTPUT);
    digitalWrite(ONE_WIRE_PWR_PIN, HIGH);
#endif
    ESP_LOGI(LOGTAG_INIT, "One wire bus enumeration");
    findOneWireDevices(ONE_WIRE_BUS_PIN);
#endif

#if defined(OLED_I2C_INTERFACE_NUM) || defined(LCD_I2C_ADDRESS) || defined(_TCS34725_H_)
    findI2CDevices();
#endif
  }

  semaphore_MQTT_done = xSemaphoreCreateBinary();
  semaphore_WIFI_connected = xSemaphoreCreateBinary();
  semaphore_NTP_synced = xSemaphoreCreateBinary();

#ifdef _TCS34725_H_
  // start read of RGB sensor early since most of the time spent is in the delay for waiting for the integration
  semaphore_RGB_sensor_done = xSemaphoreCreateBinary();
  TaskHandle_t RGBTaskHandle;
  xTaskCreate(readRGBTaskFunction, /* Task function. */
              "readRGBTask",       /* name of task. */
              10000,               /* Stack size of task */
              RGBC_values,         /* parameter of the task */
              1,                   /* priority of the task */
              &RGBTaskHandle);     /* Task handle to keep track of created task */
#endif

#ifdef __BME280_H__
  semaphore_BME_sensor_done = xSemaphoreCreateBinary();
  TaskHandle_t BMETaskHandle;
  xTaskCreate(readBMETaskFunction,
              "readBMETask",
              10000,
              NULL,
              1,
              &BMETaskHandle);
#endif

#if defined(TZ_INFO) && defined(NTPSERVER)
  configTzTime(TZ_INFO, NTPSERVER);
#endif

#ifdef WIFI_SSID
  // start WIFI connection (lasts a few seconds  to connect but runs in background)
  if (connect_to_WiFi_this_loop)
  {
    WiFi.mode(WIFI_STA);
    WiFi.config(((u32_t)0x0UL), ((u32_t)0x0UL), ((u32_t)0x0UL)); // Workaround for bug https://github.com/espressif/arduino-esp32/issues/4732

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.onEvent(WiFiEvent);
#ifdef NTP_SERVER
    sntp_set_time_sync_notification_cb(timesync_callback);
#endif
    // BlueTooth is not needed, maybe stopping it saves some power
    // https://www.mischianti.org/2021/03/10/esp32-power-saving-modem-and-light-sleep-2/
    // btStop(); //  causes crash and reboot for some reason
  }
#endif

#ifdef ONE_WIRE_BUS_PIN
  // read temperature sensors
  ESP_LOGI(LOGTAG_MAIN, "read temperatures");
  readTemperatures(ONE_WIRE_BUS_PIN);

#ifdef EPAPER_SPI_CS_PIN
  if (update_display_this_loop)
  {
    // send temperatures to ePaper
    ESP_LOGV(LOGTAG_MAIN, "send temperature updates to display");
    queue_struct.type = queue_content_type::temperature1;
    queue_struct.ptr = temperatures;
    xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
    queue_struct.type = queue_content_type::temperature2;
    queue_struct.ptr = temperatures + 1;
    xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
    queue_struct.type = queue_content_type::temperature3;
    queue_struct.ptr = temperatures + 2;
    xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
    queue_struct.type = queue_content_type::temperature4;
    queue_struct.ptr = temperatures + 3;
    xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
  }
#endif
#endif

#ifdef VBAT_PIN
  pinMode(VBAT_PIN, INPUT);
#ifdef VREF_PIN
  pinMode(VREF_PIN, INPUT);
  // get battery voltage of 3.3V supply with same voltage divider ratio
  v_bat = 3.3 * analogRead(VBAT_PIN) / analogRead(VREF_PIN);
#else
  // get battery voltage level
  v_bat = (VBAT_DIV_RATIO * 3.3 * analogRead(VBAT_PIN)) / 4095;
#endif
  ESP_LOGI(LOGTAG_SENSOR_DATA, "Power-supply voltage: %f V", v_bat);

#ifdef EPAPER_SPI_CS_PIN
  if (update_display_this_loop)
  {
    ESP_LOGV(LOGTAG_MAIN, "send battery voltage to display");
    queue_struct.type = queue_content_type::VBat;
    queue_struct.ptr = &v_bat;
    xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
  }
#endif
#endif

#ifdef AIs_LEN
  readAIs();
#endif

#ifdef PCFs_LEN
  for (size_t i = 0; i < PCFs_LEN; ++i)
  {
    pcf_available[i] = pcf[i].begin(PCF_I2C_ADDRESSES[i], &Wire); 
    if (!pcf_available)
    {
      ESP_LOGE(LOGTAG_MAIN, "Cannot connect to PCF on address %d", PCF_I2C_ADDRESSES[i]);
    }
  }
#endif

#ifdef DIs_LEN
  for (size_t j = 0; j < (size_t)DIs_LEN; j++)
  {
#ifdef PCFs_LEN
    if (DIs[j] > 0)
    {
      // ESP Pin
      pinMode(DIs[j], DI_MODE[j]);
    }
    else
    {
      int pcf_index=((-1 - DIs[j]) / 8);
      if (pcf_available[pcf_index]) {
        pcf[pcf_index].pinMode((-1 - DIs[j]) % 8, INPUT_PULLUP);
      }
    }

#else
    pinMode(DIs[j], DI_MODE[j]);
#endif

    di_values[j] = readDI(j);
  }
#endif

  if (!connect_to_WiFi_this_loop || xSemaphoreTake(semaphore_WIFI_connected, pdMS_TO_TICKS(5000)) != pdTRUE)
  {
    // either no WiFi connection scheduled, or WiFi connection timed out
    ESP_LOGI(LOGTAG_MAIN, "Not connected to WiFi");

    time(&epoch);                   // get epoch from RTC
    localtime_r(&epoch, &timeinfo); // convert epoch to local time
    ESP_LOGI(LOGTAG_MAIN, "Epoch: %ld", epoch);
    // Serial.println(&timeinfo, "; local time: %A, %B %d %Y %H:%M:%S");

#ifdef EPAPER_SPI_CS_PIN
    if (update_display_this_loop)
    {
      // update date and time
      ESP_LOGV(LOGTAG_MAIN, "send timeinfo updates to display");
      queue_struct.type = queue_content_type::Date;
      queue_struct.ptr = &timeinfo;
      xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);

      // notify ePaper that nothing more is to be printed
      ESP_LOGV(LOGTAG_MAIN, "send end of updates to display");
      queue_struct.type = queue_content_type::finish;
      xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
    }
#endif
    wait_for_sensors();
  }

#ifdef WIFI_SSID
  else
  { // WiFi is connected

    RSSI = WiFi.RSSI();
    ESP_LOGI(LOGTAG_MAIN, "WiFi connected with IP %s and RSSI %d and DNS %s as %s",
             WiFi.localIP().toString().c_str(), RSSI,
             WiFi.dnsIP().toString().c_str(),
             WiFi.getHostname());

#ifdef CONFIG_FIRMWARE_UPGRADE_URL
    ota_update(CONFIG_FIRMWARE_UPGRADE_URL);
#endif

#ifdef EPAPER_SPI_CS_PIN
    if (update_display_this_loop)
    {
      ESP_LOGV(LOGTAG_MAIN, "send WiFi info to display");
      queue_struct.type = queue_content_type::RSSI;
      queue_struct.ptr = &RSSI;
      xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
    }
#endif

#ifdef NTP_SERVER
    // wait for time update from an NTP server
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html
    // Caution: getLocalTime as used in may examples justs waits until the year is greater than 2016.
    if (xSemaphoreTake(semaphore_NTP_synced, pdMS_TO_TICKS(5000)) != pdTRUE)
    { // try for 5 seconds to sync with NTP server
      ESP_LOGE(LOGTAG_MAIN, "Failed to connect to NTP server");
    }
    else
    {
      ESP_LOGI(LOGTAG_MAIN, "Synced with NTP server.");
    }
#endif
    time(&epoch);                   // get epoch from RTC
    localtime_r(&epoch, &timeinfo); // convert epoch to local time
    ESP_LOGI(LOGTAG_MAIN, "Epoch: %ld", epoch);
    Serial.println(&timeinfo, "local time: %A, %B %d %Y %H:%M:%S");

#ifdef EPAPER_SPI_CS_PIN
    if (update_display_this_loop)
    {
      // update date and time
      ESP_LOGV(LOGTAG_MAIN, "send timeinfo updates to display");
      queue_struct.type = queue_content_type::Date;
      queue_struct.ptr = &timeinfo;
      xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);

      // notify ePaper that nothing more is to be printed
      ESP_LOGV(LOGTAG_MAIN, "send end of updates to display");
      queue_struct.type = queue_content_type::finish;
      xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
    }
#endif

    wait_for_sensors();

    MQTT_communication();

    // disable and power down WiFi
    ESP_LOGI(LOGTAG_MAIN, "Disabling WiFi");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.setSleep(true);
    // adc_power_release();  // WiFi used ADC2, now we don't need it any more. However, this command crashes and reboots.
    ESP_LOGI(LOGTAG_MAIN, "WiFi is OFF");
  }
#endif // end of #ifdef WIFI_SSID

#ifdef OLED_RESET_PIN
  if (update_display_this_loop)
  {
    ESP_LOGI(LOGTAG_MAIN, "OLED init");
    Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire1, OLED_RESET_PIN);
    Wire1.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.print(getMACasString().c_str());
    display.printf(" BC %d\n", boot_count);
    display.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
#ifdef VBAT_PIN
    display.printf("VBat: %0.1f V\n", v_bat);
#endif
    display.printf("RSSI: %d dBm\n", RSSI);
    display.display();
    ESP_LOGI(LOGTAG_MAIN, "OLED update done");
#ifndef OLED_KEEP_ON_IN_SLEEP
    // wait a bit until oled brightness up so users can see it
    delay(5000);
#endif
  }
#endif

#ifdef LCD_I2C_ADDRESS
  // #ifdef LCD_SDA_PIN
  // LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_ROWS, LCD_COLS);
  // #else
  LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_ROWS, LCD_COLS);
// #endif
#elif defined(LCD_RW_PIN)
  LiquidCrystal lcd(LCD_RS_PIN, LCD_RW_PIN, LCD_EN_PIN,
                    LCD_D0_PIN, LCD_D1_PIN, LCD_D2_PIN, LCD_D3_PIN);
#elif defined(LCD_RS_PIN)
  LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D0_PIN, LCD_D1_PIN, LCD_D2_PIN, LCD_D3_PIN);
#endif

#if defined(LCD_RS_PIN)
  lcd.begin(LCD_COLS, LCD_ROWS);

#elif defined(LCD_I2C_ADDRESS)
  lcd.begin();
  lcd.noBacklight();
#endif
#if defined(LCD_I2C_ADDRESS) || defined(LCD_RS_PIN)
  // Print a message to the LCD.
  lcd.print(&timeinfo, "%Y-%m-%d %H:%M");
  lcd.setCursor(0, 1);
  lcd.printf("BC: %d", boot_count);
  // TODO: temperature output
  ESP_LOGI(LOGTAG_MAIN, "LCD update done.");
#endif

#ifdef HISTORY_LENGTH
  // if temperature data was not transferred, e.g. no WiFi or error during MQTT session,
  // store it up for next MQTT session
  if (need_to_store_this_loop)
  {
    append_to_history(&history);
    ESP_LOGD(LOGTAG_MAIN, "%s", get_json_from_history(&history).c_str());
  }
#endif

  // reduce CPU speed while waiting for ePaper (WiFi needed at least 80 MHz)
  // setCpuFrequencyMhz(20); // this hangs forever for some reason

#ifdef EPAPER_SPI_CS_PIN
  if (update_display_this_loop)
  {
    // wait until ePaper is done
    ESP_LOGI(LOGTAG_MAIN, "Wait for ePaper");
    // wait for finish notification of ePaper
    if (xSemaphoreTake(semaphore_ePaper_done, pdMS_TO_TICKS(5000)))
    {
      ESP_LOGI(LOGTAG_MAIN, "ePaper notified main task");
    }
    else
    {
      ESP_LOGE(LOGTAG_MAIN, "ePaper task timed out");
    }
  }
#endif

#ifdef OLED_KEEP_ON_IN_SLEEP
  // Hint: int and enum gpio_num_t are incompatible in c++, but digitalPinToRtcPin returns an int.
  // thus force cast
  int OLED_RTC_PIN = digitalPinToRtcPin(OLED_RESET_PIN);
  if (OLED_RTC_PIN == -1)
  {
    ESP_LOGE(LOGTAG_MAIN, "OLED_RESET_PIN %d does not have RTC functionality)", OLED_RESET_PIN);
  }
  else
  {
    rtc_gpio_init(static_cast<gpio_num_t> digitalPinToRtcPin(OLED_RESET_PIN));
    rtc_gpio_hold_en(static_cast<gpio_num_t> digitalPinToRtcPin(OLED_RESET_PIN));
  }
#endif
#ifdef WAKEUP_PIN
  pinMode(WAKEUP_PIN, INPUT_PULLDOWN); // pulldown to avoid wakeup due to floating pin
  esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, HIGH);
#endif

  // store how long CPU was active
  last_wake_duration_ms = millis();
  ESP_LOGI(LOGTAG_MAIN, "Wake duration: %d ms", last_wake_duration_ms);
  Serial.flush();

// send to sleep
#ifdef STATUS_LED_PIN
  // turn led off after loop is done an hold it off during sleep
  digitalWrite(STATUS_LED_PIN, LOW);
  int STATUS_RTC_PIN_INT = digitalPinToRtcPin(STATUS_LED_PIN);
  // gpio_num_t STATUS_RTC_PIN = static_cast<gpio_num_t>(STATUS_RTC_PIN_INT); // C++ requires static cast for conversion of int to enum
  gpio_num_t STATUS_RTC_PIN = STATUS_LED_PIN;
  if (STATUS_RTC_PIN_INT == -1)
  {
    ESP_LOGV(LOGTAG_MAIN, "STATUS_LED_PIN %d does not have RTC functionality", STATUS_LED_PIN);
  }
  else
  {
    ESP_LOGV(LOGTAG_MAIN, "STATUS_LED_PIN %d is %d in RTC %d", STATUS_LED_PIN, STATUS_RTC_PIN_INT, STATUS_RTC_PIN);
    // rtc_gpio_init(STATUS_RTC_PIN); // only required if analog function is used in ULP
    rtc_gpio_set_direction(STATUS_RTC_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_direction_in_sleep(STATUS_RTC_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(STATUS_RTC_PIN, LOW);

    // rtc_gpio_hold_en(STATUS_RTC_PIN);
  }
#endif
  // reduce deep sleep current according to
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#_CPPv416rtc_gpio_isolate10gpio_num_t
  rtc_gpio_isolate(GPIO_NUM_12);

  esp_sleep_enable_timer_wakeup(sleep_milliseconds * 1000ULL);
  // gpio_deep_sleep_hold_en();

  gettimeofday(&sleep_enter_time, NULL);
  esp_deep_sleep_start();
}

/*
Dummy loop as this functions must be defined according to the Arduino Framework, even if its not used.
*/
void loop() {}
