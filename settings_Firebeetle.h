
// R10 and R11 are not soldered per default on the board, thus the voltage divider is disconnected from the ESP until
// R10 and R11 are soldered on.
#define VBAT_DIV_RATIO        ((float)2.0)      // 2.0 for firebeetle
#define VBAT_PIN              A0                // A0 for firebeetle, GPIO_NUM13 for Heltec WiFi Kit 32
