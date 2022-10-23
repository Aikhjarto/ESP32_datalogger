# TODO

## OTA update does not yet update server.crt
Update server.crt in SPIFFS if EOL of server is higher than EOL of current server.crt.

## Switch to MQTT5
MQTT 5, compared to 3, has TTL and a content type and metadata field for messages.
The metadata field could be used to store the time of data-acquisition.
ESP-IDF has a library for MQTT5 already: https://github.com/espressif/esp-idf/tree/master/examples/protocols/mqtt5

Also alias for topic are possible.

## Optimize partition sizes
Make a small image containing only OTA code to be able to reduce APP partition size.

## Arduino Build
### Custom partition scheme for Arduino build
Partitionscheme and clock settings, e.g. for firebeetles not set in arduino builds.

For partitions scheme, there is no selection for 16 MB with ota available.

### Check if data folder is flashed
Maybe add server.crt to `data` folder
`data` https://arduino.github.io/arduino-cli/0.26/sketch-specification/#data-subfolder

### Where to make custom partition tables selectable with arduino builds?
`boards.local.txt` from build folder is not yet read and must be copied to `~/.arduino15/packages/esp32/hardware/esp32/2.0.4/boards.local.txt` 

* https://github.com/arduino/arduino-cli/issues/1684
* https://arduino.github.io/arduino-cli/0.26/platform-specification/#boardslocaltxt


### sketch.yaml
Make use of arduino-cli https://arduino.github.io/arduino-cli/0.26/sketch-project-file/


## Add more sensors

* capture signal from thermostat with PPS connection
	* https://github.com/fredlcore/BSB-LAN
* capture signal from old ISTA 4-pin flowmeter (no documentation available)
* capture signal from powermeter
	* https://tasmota.github.io/docs/Smart-Meter-Interface/
	* https://www.photovoltaikforum.com/thread/153441-stromz%C3%A4hler-auslesen-mit-wemos-d1-mini-und-tasmota-home-assistant-low-effort-und/
	* https://wiki.volkszaehler.org/hardware/controllers/ir-schreib-lesekopf-ttl-ausgang
	* https://www.heise.de/tests/Ausprobiert-Guenstiger-IR-Lesekopf-fuer-Smart-Meter-mit-Tastmota-Firmware-7065559.html
	* https://www.youtube.com/watch?v=s6qQs4FN9B0


## Improve wakeup from reset and deep-sleep capture wakeup timing
	* external pull down on GPIO15 to suppress boot message (Note: only read once during initial boot from reset)
	* https://docs.espressif.com/projects/esptool/en/latest/esp32/advanced-topics/boot-mode-selection.html
	

Indicators:
Deep-sleep-wakeup pin
RST pin
Serial output on GPIO3/GPIO1 (RX/TX)
SPI-transfer from FLASH start
setup() by setting enable light
GPIO 3 (RXD) is HIGH on boot


RST to STATUS LED measured with logicanalyzer
```
I (29) boot: ESP-IDF v4.4.1-dirty 2nd stage bootloader
I (29) boot: compile time 13:31:15
I (29) boot: chip revision: 1
I (32) boot_comm: chip revision: 1, min. bootloader chip revision: 0
I (41) qio_mode: Enabling default flash chip QIO
I (44) boot.esp32: SPI Speed      : 80MHz
I (49) boot.esp32: SPI Mode       : QIO
I (53) boot.esp32: SPI Flash Size : 16MB
```


GPIO 0, 5, 14, and 15 output PWM during boot
GPIO 3 HIGH on boot

| Setting | Boot time | Notes |
| --- | --- | --- | 
| Arduino | 0.4 s | |
| ESP Default | 0.8s | | 
| ESP boot log from info to error | 0.7 s |
| ESP Flash mode to QIO | 0.65 s |
| System log from info to warning | 0.55 s |

Wakeup from deep sleep via pushbutton

| Setting | Boot time | Notes |
| --- | --- | --- | 
| Arduino | 392 ms | |
| ESP-IDF with CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP | 239 ms | |




## More power saving
* power saving by increasing WiFi-n-th loop during night when a) NTP is synced and b) RTC xtal is connected
* Power saving by scaling down CPU frequency using `setCpuFrequencyMhz(20)` does not work, but actually resets.
* Use pull-up transistor instead of resistor for less power consumption?
* G0 has already a pull-up on many boards for the boot process. This pin could possible be reused for ONEWIRE_PIN is the value is not too hight.
* Possible light-sleep periods
	* When not WiFi and ePaper updated is needed, sleep during `DS18B20.requestTemperatures();`, which takes ~0.5 seconds.
	* During the ~1.7 seconds waiting for waking up the ePaper display, light-sleep could be enabled.
* Faster wake-up from deep sleep by modifying boot-loader:
	* https://hackaday.io/project/174898-esp-now-weather-station/log/183782-bootloader-wake-time-improvements
* Correctly configure pin setting in GPIO (high/low/floating) for minimum power consumption.
* Use a voltage converter with an enable pin. Program ULP to measure voltage even in deep sleep and turn on voltage converter when capacitors can not drive ULP any more.
	* For full wake-up adhere ramp up time of voltage converter.
* Some sensors like digital in, ADC or I2C can possible be read during deep-sleep
	* https://stackoverflow.com/questions/65546232/esp32-reading-adc-in-active-mode-blocks-adc-reading-in-deep-sleep-mode-ulp-code
