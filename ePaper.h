#ifndef EPAPER_H
#define EPAPER_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp32-hal.h>
#include "src/epd1in54_v2/epd1in54_V2.h" // control functions for ePaper
#include "src/epd1in54_v2/epdpaint.h"    // paint functions for ePaper
#include <inttypes.h>

enum class queue_content_type
{
    temperature1,
    temperature2,
    temperature3,
    temperature4,
    VBat,
    RSSI,
    Date,
    finish
};

struct queue_content
{
    queue_content_type type;
    void *ptr;
};

// inter-procress communication
extern TaskHandle_t ePaperTaskHandle;
extern QueueHandle_t ePaper_update_queue;
extern SemaphoreHandle_t semaphore_ePaper_done;
void ePaperTaskFunction(void *pvParameters);

class ePaper : private Epd
{
public:
    ePaper();
    ~ePaper();

    void setPowerPin(int pin) { powerPin = pin; };
    bool hasPowerPin() { return powerPin != GPIO_NUM_NC; };
    int getPowerPin() { return powerPin; };
    void setHasVBat(bool has_vbat) { hasVbat = has_vbat; };
    bool hasVBat() { return hasVbat; };

    void init(void);
    void init_background_image(const char *, const char *);
    void updateTemperature(float temperature, size_t position_index);
    void updateVBat(float v_bat);
    void updateDate(struct tm *timeinfo);
    void updateRSSI(int8_t RSSI);
    void finish();

    void AssignControlPins(unsigned int reset, unsigned int dc, unsigned int busy)
    {
        Epd::AssignControlPins(reset, dc, busy);
    };
    void AssignSPIPins(int8_t spi_clk, int8_t spi_miso, int8_t spi_mosi, int8_t spi_cs)
    {
        Epd::AssignSPIPins(spi_clk, spi_miso, spi_mosi, spi_cs);
    };

private:
    unsigned char image[1024];
    int powerPin = GPIO_NUM_NC;
    bool hasVbat = false;
    Paint *background_painter;
    Paint *painter;
};

extern ePaper epaper;

#endif