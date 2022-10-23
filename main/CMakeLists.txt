# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#cmake-component-register
idf_component_register(
	SRC_DIRS "." "../src" "../src/epd1in54_v2"
	INCLUDE_DIRS ".." "../src"
)
spiffs_create_partition_image(spiffs ../data FLASH_IN_PROJECT)
#idf_build_set_property(COMPILE_OPTIONS "-Desp32" APPEND)
#idf_build_set_property(C_COMPILE_OPTIONS "${c_compile_options}" APPEND)
idf_build_set_property(CXX_COMPILE_OPTIONS -Wno-missing-field-initializers APPEND)
#idf_build_set_property(COMPILE_DEFINITIONS "${compile_definitions}" APPEND)
#idf_build_set_property(LINK_OPTIONS "${link_options}" APPEND)

