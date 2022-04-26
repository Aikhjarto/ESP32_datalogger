# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#cmake-component-register
idf_component_register(
	#REQUIRES arduino
	SRC_DIRS "." "../" 
	# pull-in some arduino libraries
	#"../../libraries/Adafruit_GFX_Library"
	#"../../libraries/Adafruit_SSD1306"
	#"../../libraries/Adafruit_BusIO"
	#"../../libraries/Arduino-Temperature-Control-Library"
	#"../../libraries/Arduino-LiquidCrystal-I2C-library/"
	"../src/epd1in54_v2"
	#"../../libraries/OneWire" original OneWire is not compaitble with ESP-IDF any more https://github.com/PaulStoffregen/OneWire/issues/112
	INCLUDE_DIRS "../" 
	# pull-in some arduino libraries
	#"../../libraries/Adafruit_GFX_Library"
	#"../../libraries/Adafruit_BusIO"
	#"../../libraries/Adafruit_SSD1306"
	#"../../libraries/Arduino-Temperature-Control-Library"
	#"../../libraries/Arduino-LiquidCrystal-I2C-library/"
	#"../../libraries/OneWire" original OneWire is not compaitble with ESP-IDF any more https://github.com/PaulStoffregen/OneWire/issues/112
	)

set(IDF_COMPONENT_MANAGER,1)