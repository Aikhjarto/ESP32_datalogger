// Settings for display an battery voltage pins on Heltec ESP32 Kit

/*****************************************************************************/
/* OLED settings for Heltec ESP32 Kit with integrated display */
/*****************************************************************************/
#define OLED_I2C_INTERFACE_NUM  1           // I2C interface number. Default pins: 0: 21/22 1: Pins 4/15
#define OLED_SCREEN_WIDTH       128         // OLED display width in pixels
#define OLED_SCREEN_HEIGHT      64          // OLED display height in pixels
#define OLED_RESET_PIN          GPIO_NUM_16 // HIGH when display is on
#define OLED_SDA_PIN            GPIO_NUM_4  
#define OLED_SCL_PIN            GPIO_NUM_15
//#define OLED_KEEP_ON_IN_SLEEP

// Note: Older Heltec ESP32 Kits have OLED_RESET_PIN on GPIO_NUM16, which does not have RTC functionality,
// thus preventing OLED_KEEP_ON_IN_SLEEP function from working

#define STATUSLED_PIN           GPIO_NUM_25

/*****************************************************************************/
/* Battery Voltage check */
/*****************************************************************************/
#define VBAT_PIN                GPIO_NUM_13  // analog input pin
#define VBAT_DIV_RATIO          ((float)3.0)// Voltage divider ratio 
