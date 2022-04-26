#include "settings.h"

/*****************************************************************************/
/* includes */
/*****************************************************************************/
#include <Arduino.h>   // for compatiblity with native esp-idf installtion
#include <esp32-hal.h> // GPIO pins, timers, ...
#include <WiFi.h>
#include <esp_sntp.h>       // erspressif for NTP timesync
#include <mqtt_client.h>    // esp mqtt client (compared to PubSubClient, esp mqtt client supports QoS 1)
#include <driver/rtc_io.h>  // to avoid floating PWR pins in sleep mode
#include <driver/adc.h>     // to power off ADCs after they are not needed any more
#include <HardwareSerial.h> // for object Serial
#include <WString.h>        // for String() class
#include <sstream>          // string creating via streams
#include <iomanip>          // stream settings
#include "myLib.h"
#ifdef EPAPER_SPI_CS_PIN
#include <SPI.h> // SPI library to communicate with ePaper
#include "ePaper.h"
#endif
#ifdef ONE_WIRE_BUS_PIN
#include <OneWire.h>           // for DS18B20 temperature sensors, tested with modified Dallas library ...
#include <DallasTemperature.h> // ... from https://github.com/milesburton/Arduino-Temperature-Control-Library
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

/*****************************************************************************/
/* global variables (to be accessibly from multiple task) and RTC memory */
/*****************************************************************************/
static const char *TAG = "LOGTAG";
RTC_DATA_ATTR size_t bootCount = 0;
RTC_DATA_ATTR time_t last_epoch = 0;
RTC_DATA_ATTR time_t epoch;
RTC_DATA_ATTR uint32_t last_wake_duration_ms = 0;
RTC_DATA_ATTR uint32_t sleep_seconds = SLEEP_SECONDS;
RTC_DATA_ATTR size_t WiFi_connect_every_Nth_loop = WIFI_CONNECT_EVERY_Nth_LOOP;
#if defined(EPAPER_SPI_CS_PIN) || defined(OLED_I2C_INTERFACE_NUM)
RTC_DATA_ATTR size_t display_update_every_Nth_loop = DISPLAY_UPDATE_EVERY_Nth_LOOP;
#endif

// network
int8_t RSSI = 0;

// MQTT
size_t published_messages_count = 0; // counter to check if everything got published before going to sleep again

#ifdef ONE_WIRE_BUS_PIN
// temperature sensors
float temperatures[OW_SERIALS_LEN];
#endif

#ifdef VBAT_PIN
// battery level
float v_bat;
#endif

// multitasking
SemaphoreHandle_t semaphore_MQTT_done;
SemaphoreHandle_t semaphore_WIFI_connected;
SemaphoreHandle_t sempahore_NTP_synced;

// history storage
// TODO: it would be much nicer to have these in a struct, but I couldn't get a struct containing arrays on RTC memory
#ifdef HISTORY_LENGTH
RTC_DATA_ATTR time_t history_epochs[HISTORY_LENGTH];
RTC_DATA_ATTR size_t history_next_store_index = 0;
bool need_to_store_this_loop;
#ifdef ONE_WIRE_BUS_PIN
RTC_DATA_ATTR float history_temperatures[OW_SERIALS_LEN][HISTORY_LENGTH];
#endif
#ifdef DIs_LEN
RTC_DATA_ATTR uint8_t history_DIs[DIs_LEN][HISTORY_LENGTH];
#endif
#endif

/*****************************************************************************/
/* Helper functions */
/*****************************************************************************/

/*
read pin DIs[j] with optionally inverting according to DI_INV[j]
*/
#ifdef DIs_LEN
uint8_t readDI(size_t j)
{
  uint8_t raw = digitalRead(DIs[j]);
  Serial.printf("Sensor %d, GPIO %d, %d\n", j, DIs[j], raw);
  if (DI_INV[j])
  {
    // Serial.printf("Inverted to %d\n", !raw);
    return !raw;
  }
  else
  {
    return raw;
  }
}
#endif

#ifdef SENSOR_VALUES_AS_JSON
std::string get_sensor_value_string(time_t epoch, float value)
{
  std::stringstream ss;
  ss << "{\"epoch\":" << epoch << ",\"value\":" << value << "}";
  return ss.str();
}

std::string get_sensor_value_string(time_t epoch, int value)
{
  std::stringstream ss;
  ss << "{\"epoch\":" << epoch << ",\"value\":" << value << "}";
  return ss.str();
}
#else
std::string get_sensor_value_string(time_t epoch, float value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}
std::string get_sensor_value_string(time_t epoch, int value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}
#endif

/*****************************************************************************/
/* History storage */
/*****************************************************************************/
#ifdef HISTORY_LENGTH
int append_to_history()
{
  if (history_next_store_index >= HISTORY_LENGTH)
  {
    return -1;
  }
#ifdef ONE_WIRE_BUS_PIN
  for (size_t j = 0; j < OW_SERIALS_LEN; j++)
  {
    history_temperatures[j][history_next_store_index] = temperatures[j];
  }
#endif

#ifdef DIs_LEN
  for (size_t j = 0; j < DIs_LEN; j++)
  {
    // // store as bytes
    // uint8_t bit_mask = 0x01<<j;
    // if (readDI(j)) {
    //   history_DIs[history_next_store_index] |= bit_mask;
    // } else {
    //   history_DIs[history_next_store_index] &= ~bit_mask;
    // }
    history_DIs[j][history_next_store_index] = readDI(j);
  }
#endif

  history_epochs[history_next_store_index] = epoch;
  return history_next_store_index++;
}

/*
  {"len":44,"epoch":[16756006,16755006,...],"t1":[4.32,5.33,...], ...}
*/
std::string get_json_from_history(void)
{

  std::stringstream ss;
  ss << "{\"len\":" << history_next_store_index << ",\"epoch\":[";
  for (size_t i = 0; i < history_next_store_index; ++i)
  {
    ss << history_epochs[i] << ",";
  }
  ss.seekp(-1, ss.cur);
  ss << "]";

#ifdef ONE_WIRE_BUS_PIN
  for (size_t j = 0; j < OW_SERIALS_LEN; j++)
  {
    ss << ",\"" << TEMPERATURE_SENSOR_NAMES[j] << "\":[";
    for (size_t i = 0; i < history_next_store_index; ++i)
    {
      ss << history_temperatures[j][i] << ",";
    }
    ss.seekp(-1, ss.cur);
    ss << "]";
  }
#endif

#ifdef DIs_LEN
  for (size_t j = 0; j < DIs_LEN; j++)
  {
    ss << ",\"" << DI_SENSOR_NAMES[j] << "\":[";
    for (size_t i = 0; i < history_next_store_index; ++i)
    {
      ss << (int)history_DIs[j][i] << ",";
    }
    ss.seekp(-1, ss.cur);
    ss << "]";
  }
#endif

  ss << "}";

  // Hint: don't pass a c_str around that was created from a temporal string as in ss.str().c_str(),
  // since the c_str() returns a pointer to a string object that might be freed right after this line.
  return ss.str();
}

#endif

/*****************************************************************************/
/* I2C */
/*****************************************************************************/

#if defined(OLED_I2C_INTERFACE_NUM) || defined(LCD_I2C_ADDRESS)
/*
Copied from Arduino's examples regarding Wire.h, as inclding the ino file does not work
*/
int I2CSearch(TwoWire *twi)
{
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0xFF; address++)
  {
    twi->beginTransmission(address);
    // Serial.printf("Scanning address %d\n", address);
    error = twi->endTransmission();
    if (error == 0)
    {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  Serial.printf("%d I2C devices found\n", nDevices);
  return nDevices;
}
#endif

/*****************************************************************************/
/* OneWire temperature sensors */
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
      Serial.println("  },");
    } while (ow.search(address));

    Serial.println("};");
    Serial.print("// nr devices found: ");
    Serial.println(count);
  }

  return count;
}

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
  // temperature update
  OneWire oneWire(pin);
  DallasTemperature DS18B20(&oneWire);
  DS18B20.begin();
  DS18B20.requestTemperatures(); // send command to all devices to read temperatures
  for (uint8_t i = 0; i < OW_SERIALS_LEN; ++i)
  {
    temperatures[i] = getTemperature(&DS18B20, OW_SERIALS[i]);
    Serial.printf("Sensor %d: %f\n", i, temperatures[i]);
  }
}

#endif

/*****************************************************************************/
/* MQTT */
/*****************************************************************************/

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
  // assert: *p points to last '/' in topic name (or event->topic if it did not contain a '/')
  // assert: i holds len of topic name starting from *p

  // printf("TOPIC_SUFFIX=%.*s\n", i, p);

  // make a c-string out of data to be able to process it with strtol
  char *data = strndup(event->data, event->data_len);

  // temporary variable to catch error of strtol
  int tmp_convert;

  if (strncmp(p, "sleep_seconds", i) == 0)
  {
    if ((str2int(&tmp_convert, data, 10) == SUCCESS) && tmp_convert > 0)
    {
      sleep_seconds = tmp_convert;
      Serial.printf("sleep_seconds=%d\n", sleep_seconds);
    }
    else
    {
      Serial.printf("%s could not be interpreted as integer", data);
    }
  }

  if (strncmp(p, "WiFi_connect_every_Nth_loop", i) == 0)
  {
    if ((str2int(&tmp_convert, data, 10) == SUCCESS) && tmp_convert > 0)
    {
      WiFi_connect_every_Nth_loop = tmp_convert;
      Serial.printf("WiFi_connect_every_Nth_loop=%d\n", WiFi_connect_every_Nth_loop);
    }
    else
    {
      Serial.printf("%s could not be interpreted as integer", data);
    }
  }
#if defined(EPAPER_SPI_CS_PIN) || defined(OLED_I2C_INTERFACE_NUM)
  if (strncmp(p, "display_update_every_Nth_loop", i) == 0)
  {
    if ((str2int(&tmp_convert, data, 10) == SUCCESS) && tmp_convert > 0)
    {
      display_update_every_Nth_loop = tmp_convert;
      Serial.printf("display_update_every_Nth_loop=%d\n", display_update_every_Nth_loop);
    }
    else
    {
      Serial.printf("%s could not be interpreted as integer", data);
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
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  esp_mqtt_client_handle_t client = event->client;

  // generate prefix with MAC address
  std::string sub_prefix = MQTT_PREFIX "/" + getMACasString();
  Serial.println(sub_prefix.c_str());

  esp_mqtt_client_subscribe(client,
                            (sub_prefix + "/WiFi_connect_every_Nth_loop").c_str(),
                            1);
  esp_mqtt_client_subscribe(client,
                            (sub_prefix + "/sleep_seconds").c_str(),
                            1);
#if defined(EPAPER_SPI_CS_PIN) || defined(OLED_I2C_INTERFACE_NUM)
  esp_mqtt_client_subscribe(client,
                            (sub_prefix + "/display_update_every_Nth_loop").c_str(),
                            1);
#endif
}

/*
Callback for MQTT events that are not the subscribe-event.
*/
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  // esp_mqtt_client_handle_t client = event->client;
  // int msg_id;
  int messages_to_wait_for_being_published = 4;

  // Serial.print((esp_mqtt_event_id_t)event_id);
  // snprintf(BrokerMsg, 50, "EVENT %d %d %d", event->topic_len, event->data_len, event->msg_id);
  // Serial.println(BrokerMsg);
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
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
#ifdef VBAT_PIN
    messages_to_wait_for_being_published++;
#endif
#ifdef ONE_WIRE_BUS_PIN
    messages_to_wait_for_being_published += OW_SERIALS_LEN;
#endif
#ifdef DIs_LEN
    messages_to_wait_for_being_published += DIs_LEN;
#endif
#ifdef HISTORY_LENGTH
    messages_to_wait_for_being_published += (history_next_store_index > 0);
#endif

    published_messages_count++;
    // Serial.println("Number of published Messages:");
    // Serial.println(published_messages_count);
    if (published_messages_count == messages_to_wait_for_being_published)
    {
      xSemaphoreGive(semaphore_MQTT_done);
    }
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    process_subscription(event);

    // printf("TOPIC=%.*s\n", event->topic_len, event->topic);
    // printf("DATA=%.*s\n", event->data_len, event->data);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
    {
      ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
    }
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
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
  if (history_next_store_index > 0)
  {
    json = get_json_from_history();
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
      .out_buffer_size = out_buffer_size,
      // .username = MQTT_USERNAME,
      // .password = MQTT_PASSWORD,
      .network_timeout_ms = 1000,
      };
  esp_mqtt_client_handle_t my_esp_mqtt_client;
  my_esp_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

  // TODO reduce timeout in case MQTT broker was down
  // connect to broker
  if (esp_mqtt_client_start(my_esp_mqtt_client) == ESP_OK)
  {
    Serial.println("MQTT connected");

    // register event handler to subscribe
    esp_mqtt_client_register_event(my_esp_mqtt_client, MQTT_EVENT_CONNECTED, mqtt_event_handler_connected, NULL);

    // register event handler to check for PUBACK messages.
    esp_mqtt_client_register_event(my_esp_mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);

#ifdef ONE_WIRE_BUS_PIN
    // publish temperatures
    for (uint8_t i = 0; i < OW_SERIALS_LEN; ++i)
    {
      // publish epoch and temperature to separate topics
      Serial.printf("Publishing temperature: %s: %f\n", bytes2hex(OW_SERIALS[i], 8).c_str(), temperatures[i]);
      esp_mqtt_client_publish(my_esp_mqtt_client,
                              (MQTT_PREFIX "/sensors/" + String(TEMPERATURE_SENSOR_NAMES[i])).c_str(),
                              (get_sensor_value_string(epoch, temperatures[i])).c_str(), 0, 1, 0);
    }
#endif

#ifdef DIs_LEN
    // publish digital input states
    for (size_t j = 0; j < DIs_LEN; j++)
    {
      esp_mqtt_client_publish(my_esp_mqtt_client,
                              (MQTT_PREFIX "/sensors/" + String(DI_SENSOR_NAMES[j])).c_str(),
                              (get_sensor_value_string(epoch, readDI(j))).c_str(),
                              0, 1, 0);
    }
#endif

    // publish system state
    std::string pub_prefix = MQTT_PREFIX "/" + getMACasString();
    esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/epoch").c_str(), String(epoch).c_str(), 0, 1, 0);
    esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/RSSI").c_str(), String(RSSI).c_str(), 0, 1, 0);
    esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/last_duration").c_str(), String(last_wake_duration_ms).c_str(), 0, 1, 0);
    esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/boot_count").c_str(), String(bootCount).c_str(), 0, 1, 0);
#ifdef VBAT_PIN
    esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/vbat").c_str(), String(v_bat).c_str(), 0, 1, 0);
#endif

#ifdef HISTORY_LENGTH
    // published data that accumulated while no WiFi was available
    if (history_next_store_index > 0)
    {
      Serial.println("Publishing history");
      esp_mqtt_client_publish(my_esp_mqtt_client, (pub_prefix + "/history").c_str(), json.c_str(), json_len, 1, 1);
    }
#endif

    Serial.println("MQTT waiting for all PUBACKs");
    if (xSemaphoreTake(semaphore_MQTT_done, pdMS_TO_TICKS(10000)) != pdTRUE)
    {
      Serial.println("MQTT publishing took too long, aborting to save power!");
    }
    else
    {
#ifdef HISTORY_LENGTH
      // everything was published, thus purge history
      need_to_store_this_loop = false;
      history_next_store_index = 0;
#endif
      Serial.println("MQTT publishing finishd, disconnecting");
    }
    esp_mqtt_client_stop(my_esp_mqtt_client);
  }
  else
  {
    Serial.print("MQTT connection failed");
  }
}

/*****************************************************************************/
/* Sleep and Wakeup */
/*****************************************************************************/
/*
For debugging, write wakeup reason in human readable form to Serial.
*/
void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("Wakeup caused by ULP program");
    break;
  default:
    Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

/*
Setup stuff that needs to be performed only after a hard-reset and not after a deep-sleep.
This included configuration of input and output pins and RTC-memory.
*/
void first_boot_init()
{
  // print important configurable (at compile time) hardware setting
  Serial.printf("WiFi MAC: %s\n", getMACasString().c_str());
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  Serial.printf("XTAL Frequency: %d MHz\n", getXtalFrequencyMhz());
  Serial.printf("APB Frequency: %d MHz\n", getApbFrequency() / 1000000);
#ifdef EPAPER_SPI_CS_PIN
  Serial.printf("SPI Clock Divider: %d\n", SPI.getClockDivider());
#endif
  Serial.printf("Setup task startet on core %d\n", xPortGetCoreID());

#if defined(OLED_I2C_INTERFACE_NUM) || defined(LCD_I2C_ADDRESS)
  // I2C address search
  Wire.begin();
  I2CSearch(&Wire);
  Wire.end();

#ifdef OLED_SDA_PIN
  Wire1.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  I2CSearch(&Wire1);
  Wire1.end();
#endif
#endif

#ifdef ONE_WIRE_BUS_PIN
#ifdef ONE_WIRE_PWR_PIN
  pinMode(ONE_WIRE_PWR_PIN, OUTPUT);
  digitalWrite(ONE_WIRE_PWR_PIN, HIGH);
#endif
  Serial.print("One wire bus enumeration\n");
  findOneWireDevices(ONE_WIRE_BUS_PIN);
#ifdef ONE_WIRE_PWR_PIN
  digitalWrite(ONE_WIRE_PWR_PIN, LOW);
#endif
#endif

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

  Serial.println("First boot init done");
}

/*****************************************************************************/
/* WiFi */
/*****************************************************************************/
void IRAM_ATTR WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("Connected to access point");
    xSemaphoreGive(semaphore_WIFI_connected);
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("Disconnected from WiFi access point");
    break;
  case SYSTEM_EVENT_AP_STADISCONNECTED:
    Serial.println("WiFi client disconnected");
    break;
  default:
    break;
  }
}

/*
Callback to give semaphore once NTP sync is completed
*/
void timesync_callback(struct timeval *tv)
{
  // Serial.print("Timesync callback: ");
  // Serial.println(sntp_get_sync_status());
  xSemaphoreGive(sempahore_NTP_synced);
}

/*****************************************************************************/
/* main */
/*****************************************************************************/
void setup()
{
  struct tm timeinfo;
  bool connect_to_WiFi_this_loop;
#if defined(EPAPER_SPI_CS_PIN) || defined(OLED_I2C_INTERFACE_NUM)
  bool update_display_this_loop;
#endif

  ++bootCount;

#ifdef STATUSLED_PIN
  pinMode(STATUSLED_PIN, OUTPUT);
  digitalWrite(STATUSLED_PIN, HIGH);
#endif
  Serial.begin(115200);

  // greeting message
  Serial.printf("\nBoot count: %d\n", bootCount);
  print_wakeup_reason();
  localtime_r(&epoch, &timeinfo); // convert epoch to local time
  Serial.println(&timeinfo, "Last Boot: %Y-%m-%d %H:%M:%S");
#ifdef HISTORY_LENGTH
  need_to_store_this_loop = true;
  Serial.printf("History length: %d\n", history_next_store_index);
#endif

  connect_to_WiFi_this_loop = (bootCount % WiFi_connect_every_Nth_loop) == 1;
#if defined(EPAPER_SPI_CS_PIN) || defined(OLED_I2C_INTERFACE_NUM)
  update_display_this_loop = (bootCount % display_update_every_Nth_loop) == 1 || esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0;
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
#endif

  // first-boot if wakeup not from time or wakeup button
  // Hint: don't check boot-count as i might overflow
  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER &&
      esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_EXT0)
  {
    first_boot_init();
  }

#ifdef VBAT_PIN
  pinMode(VBAT_PIN, INPUT);
#endif

#ifdef DIs_LEN
  for (size_t j = 0; j < DIs_LEN; j++)
  {
    pinMode(DIs[j], INPUT);
  }
#endif


  // if (!connect_to_WiFi_this_loop && !update_display_this_loop) {
  //   // reduce clock speed (and therefore power consumption), when only sensors are red
  //   // since most of the time we are waiting for temperature sensor conversion.
  //   //setCpuFrequencyMhz(10); // TODO: immediately causes reboot
  //   //setCpuFrequencyMhz(20); // TODO: immediately causes reboot
  //   //setCpuFrequencyMhz(40); // TODO: immediately causes reboot
  // }

  semaphore_MQTT_done = xSemaphoreCreateBinary();
  semaphore_WIFI_connected = xSemaphoreCreateBinary();
  sempahore_NTP_synced = xSemaphoreCreateBinary();

  configTzTime(TZ_INFO, NTPSERVER);

  // start WIFI connection (lasts a few seconds  to connect but runs in background)
  if (connect_to_WiFi_this_loop)
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.onEvent(WiFiEvent);
    sntp_set_time_sync_notification_cb(timesync_callback);
    // BlueTooth is not needed, maybe stopping it saves some power
    // https://www.mischianti.org/2021/03/10/esp32-power-saving-modem-and-light-sleep-2/
    // btStop(); //  causes crash and reboot for some reason
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

#ifdef ONE_WIRE_BUS_PIN
// read temperature sensors
#ifdef ONE_WIRE_PWR_PIN
  pinMode(ONE_WIRE_PWR_PIN, OUTPUT);
  digitalWrite(ONE_WIRE_PWR_PIN, HIGH);
#endif
  Serial.print("read temperatures\n");
  readTemperatures(ONE_WIRE_BUS_PIN);
#ifdef ONE_WIRE_PWR_PIN
  digitalWrite(ONE_WIRE_PWR_PIN, LOW);
#endif
#endif

#ifdef EPAPER_SPI_CS_PIN
  if (update_display_this_loop)
  {
    // send temperatures to ePaper
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

#ifdef VBAT_PIN
  // get battery voltage level
  v_bat = (VBAT_DIV_RATIO * 3.3 * analogRead(VBAT_PIN)) / 4095;
  Serial.printf("Power-supply voltage: %f V\n", v_bat);
#endif

#if defined(EPAPER_SPI_CS_PIN) && defined(VBAT_PIN)
  if (update_display_this_loop)
  {
    queue_struct.type = queue_content_type::VBat;
    queue_struct.ptr = &v_bat;
    xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
  }
#endif

  if (!connect_to_WiFi_this_loop || xSemaphoreTake(semaphore_WIFI_connected, pdMS_TO_TICKS(5000)) != pdTRUE)
  {
    // either no WiFi connection scheduled, or WiFi connection timed out
    Serial.println("\nNot connected to WiFi");

    time(&epoch);                   // get epoch from RTC
    localtime_r(&epoch, &timeinfo); // convert epoch to local time
    Serial.printf("Epoch: %ld", epoch);
    Serial.println(&timeinfo, "; local time: %A, %B %d %Y %H:%M:%S");

#ifdef EPAPER_SPI_CS_PIN
    if (update_display_this_loop)
    {
      // update date and time
      queue_struct.type = queue_content_type::Date;
      queue_struct.ptr = &timeinfo;
      xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);

      // notify ePaper that nothing more is to be printed
      queue_struct.type = queue_content_type::finish;
      xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
    }
#endif
  }
  else
  { // WiFi is connected

    RSSI = WiFi.RSSI();
    Serial.printf("WiFi connected with IP %s and RSSI %d\n",
                  WiFi.localIP().toString().c_str(), RSSI);

    // wait for time update from an NTP server
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html
    // Caution: getLocalTime as used in may examples justs waits until the year is greater than 2016.
    if (xSemaphoreTake(sempahore_NTP_synced, pdMS_TO_TICKS(5000)) != pdTRUE)
    { // try for 5 seconds to sync with NTP server
      Serial.println("Failed to connect to NTP server");
    }
    else
    {
      Serial.println("Synced with NTP server.");
    }

    time(&epoch);                   // get epoch from RTC
    localtime_r(&epoch, &timeinfo); // convert epoch to local time
    Serial.printf("Epoch: %ld", epoch);
    Serial.println(&timeinfo, "; local time: %A, %B %d %Y %H:%M:%S");

#ifdef EPAPER_SPI_CS_PIN
    if (update_display_this_loop)
    {
      queue_struct.type = queue_content_type::RSSI;
      queue_struct.ptr = &RSSI;
      xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);

      // udpate date and time
      queue_struct.type = queue_content_type::Date;
      queue_struct.ptr = &timeinfo;
      xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);

      // notify ePaper that nothing more is to be printed
      queue_struct.type = queue_content_type::finish;
      xQueueSend(ePaper_update_queue, (void *)&queue_struct, portMAX_DELAY);
    }
#endif

    MQTT_communication();

    // disable and power down WiFi
    Serial.println("Disabling WiFi");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.setSleep(true);
    // adc_power_release();  // WiFi used ADC2, now we dont need it any more. However, this command crashes and reboots.
    Serial.println("WiFi is OFF");
  }

#ifdef OLED_RESET_PIN
  if (update_display_this_loop) {
    Serial.println("OLED init");
    Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire1, OLED_RESET_PIN);
    Wire1.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.print(getMACasString().c_str());
    display.printf(" BC %d\n", bootCount);
    display.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
  #ifdef VBAT_PIN
    display.printf("VBat: %0.1f V\n", v_bat);
  #endif
    display.printf("RSSI: %d dBm\n", RSSI);
    display.display();
    Serial.println("OLED update done");
    delay(5000);
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
  lcd.setCursor(0,1);
  lcd.printf("BC: %d", bootCount);
  // TODO: temperature output
  Serial.println("LCD update done.");
#endif

#ifdef HISTORY_LENGTH
  // if temperature data was not transferred, e.g. no WiFi or error during MQTT session,
  // store it up for next MQTT session
  if (need_to_store_this_loop)
  {
    append_to_history();
    Serial.println(get_json_from_history().c_str());
  }
#endif

  // reduce CPU speed while waiting for ePaper (WiFi needed at least 80 MHz)
  // setCpuFrequencyMhz(20); // this hangs forever for some reason

#ifdef EPAPER_SPI_CS_PIN
  if (update_display_this_loop)
  {
    // wait until ePaper is done
    Serial.println("Wait for ePaper");
    // wait for finish notification of ePaper
    if (xSemaphoreTake(semaphore_ePaper_done, pdMS_TO_TICKS(10000)))
    {
      Serial.println("ePaper notified main task");
    }
    else
    {
      Serial.println("ePaper task timed out");
    }
  }
#endif

#ifdef STATUSLED_PIN
  // turn led off after loop is done
  digitalWrite(STATUSLED_PIN, LOW);
#endif

  // store how long CPU was active
  last_wake_duration_ms = millis();
  Serial.printf("Back to sleep after: %d ms\n", last_wake_duration_ms);
  Serial.flush();

  // send to sleep
#ifdef WAKEUP_PIN
  esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, HIGH);
#endif
  esp_sleep_enable_timer_wakeup(sleep_seconds * 1000000ULL);
  gpio_deep_sleep_hold_en();
  esp_deep_sleep_start();
}

/*
Dummy loop as this functions must be defined according to the Arduino Framework, even if its not used.
*/
void loop() {}
