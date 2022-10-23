# ESP32 Datalogger
In this project, I realized a temperature measurement a heating circuit of a building using an ESP32 to read out DS18B20 temperature sensors and mains detectors, send data via MQTT, and optionally display the result on an ePaper, OLED or LCD display. 

As the system should be able to run from a battery, special thought on power consumption was given.

## Working Principle

### Upon first boot
* ePaper display is initialized with static image containing lines and labels, but no values.
* WiFi connection is established to get current time via SNTP.
* Onewire bus is enumerated and found serial numbers are printed via serial interface in a way that they can be copy-pasted to 'settings.h'.
* Onewire sensors, battery voltage and RSSI are read, sent via MQTT and display is updated with values.
* ESP32 is sent to deep-sleep with a wake-up-timer for `SLEEP_SECONDS` and a wake-up-interrupt when `WAKEUP_PIN` goes high.

### Wake-ups
* Upon each wake-up temperature measurements using the connected Onewire temperature sensors are conducted and the results are stored along with the corresponding epochs in RTC memory and deep-sleep is entered again.
* Upon each `WIFI_CONNECT_EVERY_Nth_LOOP`-th wake-up, a WiFi connection, NTP timesync and data transfer to an MQTT server is attempted.
* Upon each `DISPLAY_UPDATE_EVERY_Nth_LOOP`-th wake-up or upon wake-up via the `WAKEUP_PIN`, the values on the ePaper display are updated.

### Settings
All settings, e.g. WiFi and MQTT settings, pin assignments, and sensor names, are grouped in [`setting.h.example`](settings.h.example).
Some settings can be changed via MQTT messages. See [Subscribed MQTT messages](#Subscribed-MQTT-messages)


## MQTT Topics
### Subscribed MQTT messages
Some settings are updated via MQTT subscription to 

| Topic | format |
|-------|--------|
| MQTT_PREFIX/0123456789abc/WiFi_connect_every_Nth_loop | ASCII uint |
| MQTT_PREFIX/0123456789abc/ePaper_update_every_Nth_loop | ASCII uint |
| MQTT_PREFIX/0123456789abc/sleep_seconds | ASCII uint |

where '0123456789abc' is the hex-encoded (all lowercase) MAC address of the WiFi interface.


### Published MQTT messages

#### ESP32 System data

| Topic | format |
|-------|--------| 
| MQTT_PREFIX/0123456789abc/vbat | ASCII float | 
| MQTT_PREFIX/0123456789abc/epoch | ASCII time_t |
| MQTT_PREFIX/0123456789abc/RSSI | ASCII uint |
| MQTT_PREFIX/0123456789abc/last_duration | ASCII uint in ms |
| MQTT_PREFIX/0123456789abc/boot_count | ASCII uint |
| MQTT_PREFIX/0123456789abc/history | json* |

where '0123456789abc' is the hex-encoded (all lowercase) MAC address of the WiFi interface.

*e.g.: `{"len":44,"epoch":[16756006,16755006,...],"sensor1_name":[4.32,5.33,...],...}`

#### Sensors data

| Topic | format |
|-------|--------|
| MQTT_PREFIX/sensors/OW_SENSOR_NAMES[j] | json* or ASCII float |
| MQTT_PREFIX/sensors/DI_SENSOR_NAMES[k] | json* or ASCII [0,1] |

with `0<=j<OW_SERIALS_LENGTH` and `0<=k<DIs_LEN`

*e.g. `{"epoch":16756006,"value":33.4}`

## Implementation details
### Onewire temperature sensors
* The Onewire bus needs a 4.7k pull-up resistor for the data-line. The ESP32 provides internal pull-up/pull-down resistors. However, due to the large manufacturing tolerances of silicium-based resistors as used in the ESP32, the internal pull-up/pull-down resistors may vary between roughly 10k and 100k.

### ePaper
* ePaper update is slower the more pixel require an update. Thus only update values that have changed since the last update.
* The slowest part of the used ePaper display is the wake-up from sleep ($\approx$ 1.7 seconds) and the initialization phase required before any data transfer ($\approx$ 200 ms). Thus the number of calls to `EDP.setFrameMemory()` is kept to a minimum.
* The ePaper retains pixels but not its memory when sleeping/poweroff. Thus pixel values are stored bit-wise, i.e. 8 pixel per byte, in RTC memory of the ESP. For the 200x200px display 5000 bytes are required. Thus, not the whole image needs to be created when the ESP wakes from deep-sleep, but differential updates are used.

### History storage
To save power, ePaper display updates and WiFi connection are not made in the same rate as data acquisition.
In between WiFi connections, sensor data is stored in RTC memory during deep sleeps.
If space in RTC memory runs out, data is offloaded to flash using SPIFFS.

### Timesync
`getLocalTime()` is presented seen in many examples and tutorials about the ESP32 to be used to wait for the ESP32's NTP client to be synced with an NTP server. However, this function actually waits until the year of the system clock is above 2016. Thus, it effectively waits only for the first NTP sync, since for a subsequent NTP update, the year was already above 2016 before starting the NTP update.

### Multitasking
Connecting to WiFi and initializing ePaper display both require a significant amount of waiting. Thus, two tasks are run as background thread while data from sensors is acquired.

### Power consumption
Most cheap ESP32 boards have an LDO, e.g. ASM1117, voltage regulators, which have 6-10 mA idle power consumption, thus even when the ESP is deep-sleep, the LDO is a significant power sink.

### ADC for battery level detection
The ADC's in the ESP32 are quite nonlinear, especially outside 0.5-2.5V, and biased.
To get rid of the bias, next to `VBAT_PIN` a `VREF_PIN` can be defined which sports the same voltage divider ratio but dividing from the 3.3V bin.
Note that `VREF_PIN` and `VBAT_PIN` must be both connected to ADC1 since ADC2 is used internally for WiFi.

### Serial output
While outputting status over UART is nice during development, serial output of even a few characters is taking several ms at baud-rate 115200.
Thus reduce serial output to a minimum, but for debugging, I left it at first boot.

## OTA
EPSImageSrv is a simple Python HTTP server which serves binary images as `*.bin`, SPIFFS images as `*.spiffs` and `*.date` as ASCII ISO date string.
The `*.date` file is virtual (ESPImageSrv reads it from the `*.bin` file) and does not need to be created

Each time a WiFi connection is scheduled on the ESP32 boards, the check the `*.date` URL to see if an update is required and if so, download the `*.bin` file.

## Used Hardware
* [Waveshare 200x200px, 1.54" E-Ink display module, B/W, V2, SPI](https://www.waveshare.com/product/displays/e-paper/1.54inch-e-paper-module.htm)
    + low number of pixels so full framebuffer fits in EPS32's RTC memory
    - documentation and example library has potential for improvement
* ESP32 Dev Board. Tested with:
    * [DrRobot Firebeetle](https://www.dfrobot.com/product-1590.html)
        + low quiescent current (< 1 mA) of voltage regulator
        + 3.7V LiPo port with charge over USB possibility
        - SMD soldering skill required to fill in missing 0-resistors to enable VBat
    * [Heltec ESP32 Kit](https://heltec.org/project/wifi-kit-32/)
        + 3.7V LiPo port with charge over USB possibility
        - high quiescent current
        + OLED display integrated
        - OLED power is hardwired to GPIO_NUM_16, which cannot be enabled in deep-sleep
    * [AZ-Delivery ESP-32 Dev Kit C](https://www.az-delivery.de/products/esp-32-dev-kit-c-v4)
        + high availability
        + follows Espressif's reference design
        - high quiescent current of voltage regulator
* DollaTek PCF8574 I2C I/O-Expander 
* one push-button

## Build
For details about the layout of the project and compiling it, see <BUILD.md>
