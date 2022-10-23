# Build
## Project Layout
The layout of this project is designed to be compatible with the Arduino workflow (Arduino IDE and arduino-cli) as well as es Espressif IDF workflow (idf.py).

`main`, `components`, `partitions` and `sdkconfig*` are used by `idf.py`
`sketch.json`, `sketch.yaml`, `build_props.txt` and `arduino-cli.yaml` are used by `arduino-cli`

Placing non-modules `*.h` and `*.cpp` files in `src` is mandated by Arduino, but also used by `idf.py`.

<https://arduino.github.io/arduino-cli/0.28/sketch-specification/#sketch-folders-and-files>

<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html>

## Prerequisites
ESP-IDF and Arduino Libraries (for OwnWire temperature sensors and display control) are required for compilation.
In Arduino IDE one can add support for ESP32 boards and in ESP-IDF arduino-esp32 can be added as component.

URLs of Boardmanager for Arduino IDE :
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
http://download.dfrobot.top/FireBeetle/package_esp32_index.json
https://git.oschina.net/dfrobot/FireBeetle-ESP32/raw/master/package_esp32_index.json
https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/releases/download/0.0.5/package_heltec_esp32_index.json
```
* Arduino IDE with ESP for Arduino v4.4 or
* VSCode with ESP-IDF extension and Arduino as IDF component https://github.com/espressif/arduino-esp32

**Caution:** Names or files and variables concerning fonts in `./src/epd1in54_v2` is quite common among many EPD libraries. If you get errors during linking regarding multiple definitions of fonts, remove EPD libraries that were installed using the Arduino Library Manager.

Before OTA Images can be used, a certificate and a key for ESPImageSrv must be created.


## Select Module Configuration
In `settings.h`, different settings per board are possible using defines. 
However, Arduino IDE cannot be configured to pass defines as compiler arguments and `idf.py` does not handle `SDKCONFIG_DEFAULT` variable well (bootloader build fails if set).

Thus,
```
scripts/set_build_target TARGET
```
can be used to modify `main/CMakeLists.txt` and `settings_board_select.h`

## Build

### Build with ESP-IDF in VSCode
Tasks for board selection, building and serving OTA images are defined.

### Build with ESP-IDF from CLI
```
. ~/esp/esp-idf/export.sh
idf.py set-target esp32
idf.py build flash monitor
```

### Build with make/cmake
```
. ~/esp/esp-idf/export.sh
mkdir -p build
cd build
cmake ..
make
```

### Build with Arduino IDE
Install ESP32 board an libraries OneWire, Dallas Temperature from [here](https://github.com/milesburton/Arduino-Temperature-Control-Library), Adafruit SSD1306 and LiquidCrystal\_I2C.

Upload the data folder using [https://github.com/me-no-dev/arduino-esp32fs-plugin/releases/] https://microcontrollerslab.com/install-esp32-filesystem-uploader-in-arduino-ide-spiffs/

Drawback: Advanced board configuration, like disabling bootloader message, selecting an external crystal for the RTC or compiling for OTA is not possible.

* `scripts/set_build_target` to select build target and build with Arduino IDE

## Bulk Building OTA Images from CLI
```
scripts/build_ota_images
```


