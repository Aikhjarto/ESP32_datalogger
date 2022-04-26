# Sketch
In this project, I realized a temperature measurement for dual heating circuit of a building using an ESP32 to read out 4 DS18B20 temperature sensors, send temperature data via MQTT, and optionally display the result on an ePaper display. 

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
* Upon each `EPAPER_UPDATE_EVERY_Nth_LOOP`-th wake-up or upon wake-up via the `WAKEUP_PIN`, the values on the ePaper display are updated.

### Settings
All settings, e.g. WiFi and MQTT settings, pin assignments, and sensor names, are grouped in [`setting.h`](settings.h).
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
* The slowest part of the used ePaper display is the wake-up from sleep (~1.7 seconds) and the initialization phase required before any data transfer (~200 ms). Thus the number of calls to `EDP.setFrameMemory()` is kept to a minimum.
* The ePaper retains pixels but not its memory when sleeping/poweroff. Thus pixel values are stored bit-wise, i.e. 8 pixel per byte, in RTC memory of the ESP. For the 200x200px display 5000 bytes are required. Thus, not the whole image needs to be created when the ESP wakes from deep-sleep, but differential updates are used.

### History storage
* To save memory, digital inputs are stored bitwise, like the image for the ePaper display.

### Timesync
* `getLocalTime()` is presented seen in many examples and tutorials about the ESP32 to be used to wait for the ESP32's NTP client to be synced with an NTP server. However, this function actually waits until the year of the system clock is above 2016. Thus, it effectively waits only for the first NTP sync, since for a subsequent NTP update, the year was already above 2016 before starting the NTP update.

### Multitasking
* Connecting to WiFi and initializing ePaper display both require a significant amount of waiting. Thus, two tasks are run in parallel.
* TODO: describe semaphore and queue usage.

### Power consumption
Most cheap ESP32 boards have an LDO, e.g. ASM1117, voltage regulators, which have 6-10 mA idle power consumption, thus even when the ESP is deep-sleep, the LDO is a significant power sink.


LoLin D1 are seemingly the best voltage regulator, providing for 130µA board power consumption when ESP32 is in deep sleep.
However, the board is not produced any more https://www.esp32.com/viewtopic.php?p=21863#p21863

LDO regulators are good if board is mostly idle.

Best would be some buck/boost converter with  mode switch like this https://www.farnell.com/datasheets/43210.pdf
Mode can be switched by ULP mode of ESP32 before powering on.

Firebeetle ESP32 has pretty low idle consumption by using the RT9080-33GJ5 according to https://diyi0t.com/reduce-the-esp32-power-consumption/
However, newer versions of Firebeetle ESP32 boards also have the ASM1117 according to high-res product images on amazon.
Sparkfun ESP32 Thing is not available in Europe.
Adafruit HUZZAH32 has fewer pins.

## Used Hardware
* [Waveshare 200x200px, 1.54" E-Ink display module, B/W, V2, SPI](https://www.waveshare.com/product/displays/e-paper/1.54inch-e-paper-module.htm)
* ESP32 NodeMCU with an ESP32-WROOM https://www.az-delivery.de/products/esp32-developmentboard?_pos=2&_sid=c2e3160aa&_ss=r
* [DrRobot Firebeetle](https://www.dfrobot.com/product-1590.html)
* one push-button

## Used Software
* ESP for Arduino v4.4
* Modified version of Waveshare's [e-Paper](https://github.com/waveshare/e-Paper) library (since the library had many bugs and was cumbersome to use for EPD1in54_V2)
* [Arduino-Temperature-Control-Library](https://github.com/milesburton/Arduino-Temperature-Control-Library.git)

Caution: Names or files and variables concerning fonts in `./src/epd1in54_v2` is quite common among many EPD libraries. If you get errors during linking regarding multiple definitions of fonts, remove EPD libraries that were installed using the Arduino Library Manager.

