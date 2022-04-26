
# Build

## Build with Arduino IDE
Install ESP32 board an libraries OneWire, Dallas Temperature from [here](https://github.com/milesburton/Arduino-Temperature-Control-Library), Adafruit SSD1306 and LiquidCrystal\_I2C.

Advantage: Easy to install, compile and upload.
Drawback: Advanced board configuration, like disabling bootloader message or selecting an extern clock for the RTC is not possible.

## Build using ESP-IDF
Config:
```
. $HOME/esp/esp-idf/export.sh
idf.py menuconfig
```
* Goto section `Arduino Configuration ---> ` and turn on `Turn on Autostart Arduino setup and loop on boot`
* Component config → FreeRTOS -> Tick Rate to 1000 since with default of 1000, the delay function has a granularity of 10 ms.
* Increase Flash size since default is 1MB, but most modules have 4MB nowadays.

```
idf.py set-target esp32
idf.py build flash monitor
```

### VSCode with idf.py
```
. $HOME/esp/esp-idf/export.sh
code
```

