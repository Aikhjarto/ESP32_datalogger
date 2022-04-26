
## TODO for more power saving
* get rid of bootloader message by using an external pull-down on GPIO to save some power
* power saving by increasing WiFi-n-th loop during night when a) NTP is synced and b) RTC xtal is connected
* Power saving by scaling down CPU frequency using `setCpuFrequencyMhz(20)` does not work, but actually resets.
* Use pull-up transistor instead of resistor for less power consumption?
* G0 has already a pull-up on many boards for the boot process. This pin could possible be reused for ONEWIRE_PIN is the value is not too hight.
* Possible light-sleep periods
	* When not WiFi and ePaper updated is needed, sleep during `DS18B20.requestTemperatures();`, which takes ~0.5 seconds.
	* During the ~1.7 seconds waiting for waking up the ePaper display, light-sleep could be enabled.
* Faster wake-up from deep sleep by modifying boot-loader:
	* https://community.platformio.org/t/esp32-firebeetle-fast-boot-after-deep-sleep/13206
	* https://hackaday.io/project/174898-esp-now-weather-station/log/183782-bootloader-wake-time-improvements
* Correctly configure pin setting in GPIO (high/low/floating) for minimum power consumption.
* Use a voltage converter with an enable pin. Program ULP to measure voltage even in deep sleep and turn on voltage converter when capacitors can not drive ULP any more.
	* For full wake-up adhere ramp up time of voltage converter.
https://stackoverflow.com/questions/65546232/esp32-reading-adc-in-active-mode-blocks-adc-reading-in-deep-sleep-mode-ulp-code
