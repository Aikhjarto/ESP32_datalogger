idf_component_register(
    REQUIRES arduino OneWireNg
    SRC_DIRS "Arduino-Temperature-Control-Library"
    "Adafruit_BusIO"
    "Adafruit_SSD1306"
    "Adafruit-GFX-Library"
    "Arduino-LiquidCrystal-I2C-library"
    "Adafruit_TCS34725"
    "Adafruit_BME280"
    "Adafruit_Sensor"
    "Adafruit_PCF8574"

    INCLUDE_DIRS "Arduino-Temperature-Control-Library"
    "Adafruit_BusIO"
    "Adafruit_SSD1306"
    "Adafruit-GFX-Library"
    "Arduino-LiquidCrystal-I2C-library"
    "Adafruit_TCS34725"
    "Adafruit_BME280"
    "Adafruit_Sensor"
    "Adafruit_PCF8574"
)

# Adafruit_TCS34725 1.4.1 uses narrowing which throws an error during compling instead of a warning in newer GCC versions.
# Thus, make the error a warning.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=narrowing)
