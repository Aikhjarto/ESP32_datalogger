/*****************************************************************************/
/* OLED settings for Heltec ESP32 Kit with integrated display */
/*****************************************************************************/
#define OLED_I2C_INTERFACE_NUM  1           // I2C interface number. Default pins: 0: 21/22 1: Pins 4/15
#define OLED_SCREEN_WIDTH       128         // OLED display width in pixels
#define OLED_SCREEN_HEIGHT      64          // OLED display height in pixels
#define OLED_RESET_PIN          GPIO_NUM_16 
#define OLED_SDA_PIN            GPIO_NUM_4  
#define OLED_SCL_PIN            GPIO_NUM_15

#define STATUSLED_PIN           GPIO_NUM_25

#define VBAT_PIN                GPIO_NUM_13  // analog input pin
#define VBAT_DIV_RATIO          ((float)3.0)// Voltage divider ratio 
