#include <stdint.h>
#include <time.h>
#include "src/epd1in54_v2/fonts.h" // fonts for ePaper display
#include <stdlib_noniso.h>         // for dtostrf
#include "ePaper.h"
#include "myLib.h"
#include <HardwareSerial.h>

#define COLORED 0
#define UNCOLORED 1

/*****************************************************************************/
/* RTC variables for partial updates */
/*****************************************************************************/

RTC_DATA_ATTR unsigned char background_image_buffer[EPD_WIDTH * EPD_HEIGHT / 8]; // 8 pixel per byte
RTC_DATA_ATTR char old_date_str[12];
RTC_DATA_ATTR char old_hour_str[9];
RTC_DATA_ATTR char old_temperature_strings[4 * 6];
RTC_DATA_ATTR char old_v_bat_str[4];
RTC_DATA_ATTR char old_RSSI_str[4];

/*****************************************************************************/
/* Multitasking */
/*****************************************************************************/
QueueHandle_t ePaper_update_queue = xQueueCreate(8, sizeof(struct queue_content));
TaskHandle_t ePaperTaskHandle;
SemaphoreHandle_t semaphore_ePaper_done = xSemaphoreCreateBinary();

void ePaperTaskFunction(void *pvParameters)
{

  uint32_t ulNotifiedValue = 0;
  queue_content content;

  Serial.printf("ePaper Task startet on Core %d\n", xPortGetCoreID());

  epaper.init();

  for (;;)
  {
    xQueueReceive(ePaper_update_queue, &(content), portMAX_DELAY);
    switch (content.type)
    {
    case queue_content_type::temperature1:
      epaper.updateTemperature(*((float *)content.ptr), (size_t)0);
      break;
    case queue_content_type::temperature2:
      epaper.updateTemperature(*((float *)content.ptr), (size_t)1);
      break;
    case queue_content_type::temperature3:
      epaper.updateTemperature(*((float *)content.ptr), (size_t)2);
      break;
    case queue_content_type::temperature4:
      epaper.updateTemperature(*((float *)content.ptr), (size_t)3);
      break;
    case queue_content_type::RSSI:
      epaper.updateRSSI(*((int8_t *)content.ptr));
      break;
    case queue_content_type::Date:
      epaper.updateDate((struct tm *)content.ptr);
      break;
    case queue_content_type::finish:
      epaper.finish();
      break;
    case queue_content_type::VBat:
      epaper.updateVBat(*((float *)content.ptr));
      break;

    default:
      Serial.printf("Got unknown notification value: %d\n", ulNotifiedValue);
      break;
    }
  }
  vTaskDelete(NULL);
}

/*****************************************************************************/
/* ePaper */
/*****************************************************************************/

void ePaper::init_background_image(const char* label_line1, const char* label_line2)
{

  background_painter->Clear(UNCOLORED);

  // draw frame
  background_painter->DrawFilledRectangle(0, 50, EPD_WIDTH, 54, COLORED);
  background_painter->DrawFilledRectangle(0, 118, EPD_WIDTH, 122, COLORED);
  background_painter->DrawFilledRectangle(0, 180, EPD_WIDTH, 200, COLORED);

  // draw labels
  background_painter->DrawStringAt(0, 58, label_line1, &Font20, COLORED);
  background_painter->DrawStringAt(0, 126, label_line2, &Font20, COLORED);

  if (hasVBat())
  {
    // battery voltage
    background_painter->DrawStringAt(2, 183, "VBat:", &Font16, UNCOLORED);
    background_painter->DrawStringAt(2 + 8 * Font16.Width, 183, "V", &Font16, UNCOLORED);
  }
  else
  {
    // MAC address
    background_painter->DrawStringAt(2, 185, getMACasString().c_str(), &Font12, UNCOLORED);
  }

  // RSSI
  background_painter->DrawStringAt(EPD_WIDTH - 2 - 3 * Font16.Width, 183, "dBm", &Font16, UNCOLORED);

  // value separators
  background_painter->DrawStringAt((EPD_WIDTH - Font32.Width) / 2, 82, "/", &Font32, COLORED);
  background_painter->DrawStringAt((EPD_WIDTH - Font32.Width) / 2, 145, "/", &Font32, COLORED);

  // time and date
  background_painter->DrawStringAt(32 + 2 * Font24.Width, 0, ":", &Font24, COLORED);
}

ePaper::ePaper()
{
  background_painter = new Paint(background_image_buffer, EPD_WIDTH, EPD_HEIGHT);
  painter = new Paint(image, 0, 0);
}

ePaper::~ePaper()
{
  delete background_painter;
  delete painter;
}

void ePaper::init(void)
{
  Serial.print("e-Paper init");

  // supply ePaper with power
  if (hasPowerPin())
  {
    pinMode(getPowerPin(), OUTPUT);
    digitalWrite(getPowerPin(), HIGH);
  }

  // wake up
  LDirInit();
  DisplayPartBaseImage(background_image_buffer);
  Serial.println("EPD init done");
}

void ePaper::updateTemperature(float temperature, size_t position_index)
{

  painter->SetHeight(Font32.Height);
  painter->SetWidth(4 * Font32.Width);
  char buff_10[6];
  int x_start, y_start;
  dtostrf(temperature, 4, 1, buff_10);
  if (strncmp(buff_10, old_temperature_strings + 6 * position_index, 4))
  {
    Serial.printf("EPD Update Temperature %d\n", position_index);
    strcpy(old_temperature_strings + 6 * position_index, buff_10);

    switch (position_index)
    {
    case 0:
      x_start = 0;
      y_start = 82;
      break;

    case 1:
      x_start = EPD_WIDTH - 4 * Font32.Width;
      y_start = 82;
      break;

    case 2:
      x_start = 0;
      y_start = 145;
      break;

    case 3:
      x_start = EPD_WIDTH - 4 * Font32.Width;
      y_start = 145;
      break;

    default:
      x_start = 0;
      y_start = 0;
      break;
    }

    painter->Clear(UNCOLORED);
    painter->DrawStringAt(0, 0, buff_10, &Font32, COLORED);
    SetFrameMemoryPartial(painter->GetImage(), x_start, y_start, painter->GetWidth(), painter->GetHeight());

    background_painter->DrawFilledRectangle(x_start, y_start, x_start + 4 * Font32.Width, y_start + Font32.Height, UNCOLORED);
    background_painter->DrawStringAt(x_start, y_start, buff_10, &Font32, COLORED);
  }
}

void ePaper::updateVBat(float v_bat)
{
  char tmpStr[4];
  dtostrf(v_bat, 1, 1, tmpStr);
  tmpStr[3] = '\0';
  if (strncmp(tmpStr, old_v_bat_str, 3))
  {
    Serial.println("EPD update vBat");
    strcpy(old_v_bat_str, tmpStr);
    painter->SetWidth(3 * Font16.Width);
    painter->SetHeight(16);
    painter->Clear(COLORED);
    painter->DrawStringAt(0, 0, tmpStr, &Font16, UNCOLORED);
    SetFrameMemoryPartial(painter->GetImage(), 2 + 5 * Font16.Width, 183, painter->GetWidth(), painter->GetHeight());

    background_painter->DrawFilledRectangle(2 + 5 * Font16.Width, 180, 2 + 8 * Font16.Width, 200, COLORED);
    background_painter->DrawStringAt(2 + 5 * Font16.Width, 183, tmpStr, &Font16, UNCOLORED);
  }
}

void ePaper::updateDate(struct tm *timeinfo)
{
  Serial.println("EPD update date");

  painter->SetHeight(24);
  painter->SetWidth(2 * Font24.Width);

  char timeString[4];
  strftime(timeString, 3, "%H", timeinfo);
  if (strncmp(old_hour_str, timeString, 2))
  {
    strcpy(old_hour_str, timeString);
    painter->Clear(UNCOLORED);
    painter->DrawStringAt(0, 0, timeString, &Font24, COLORED);
    SetFrameMemoryPartial(painter->GetImage(), 32, 0, painter->GetWidth(), painter->GetHeight());

    background_painter->DrawFilledRectangle(32, 0, 32 + 2 * Font24.Width, Font24.Height, UNCOLORED);
    background_painter->DrawStringAt(32, 0, timeString, &Font24, COLORED);
  }

  painter->SetWidth(5 * Font24.Width);
  painter->Clear(UNCOLORED);
  strftime(timeString, 6, "%M:%S", timeinfo);
  painter->DrawStringAt(0, 0, timeString, &Font24, COLORED);
  SetFrameMemoryPartial(painter->GetImage(), 32 + 3 * Font24.Width, 0, painter->GetWidth(), painter->GetHeight());

  char dateString[13];
  strftime(dateString, 12, "%Y-%m-%d", timeinfo);
  if (strncmp(dateString, old_date_str, 10))
  {
    strcpy(old_date_str, dateString);
    painter->SetWidth(10 * Font24.Width);
    painter->Clear(UNCOLORED);
    painter->DrawStringAt(0, 0, dateString, &Font24, COLORED);
    SetFrameMemoryPartial(painter->GetImage(), 15, 25, painter->GetWidth(), painter->GetHeight());

    background_painter->DrawFilledRectangle(15, 25, 15 + 10 * Font24.Width, 25 + Font24.Height, UNCOLORED);
    background_painter->DrawStringAt(15, 25, dateString, &Font24, COLORED);
  }
}

void ePaper::updateRSSI(int8_t RSSI)
{
  char RSSI_str[6];
  snprintf(RSSI_str, 6, "%d", RSSI);
  if (strncmp(RSSI_str, old_RSSI_str, 4))
  {
    strcpy(old_RSSI_str, RSSI_str);
    Serial.println("Update RSSI");
    painter->SetHeight(Font16.Height);
    painter->SetWidth(3 * Font16.Width);
    painter->Clear(COLORED);
    painter->DrawStringAt(0, 0, String(RSSI).c_str(), &Font16, UNCOLORED);
    SetFrameMemoryPartial(painter->GetImage(), EPD_WIDTH - 6 * Font16.Width, 183, painter->GetWidth(), painter->GetHeight());

    background_painter->DrawFilledRectangle(EPD_WIDTH - 2 - 6 * Font16.Width, 183, EPD_WIDTH - 2 - 3 * Font16.Width, 200, COLORED);
    background_painter->DrawStringAt(EPD_WIDTH - 2 - 6 * Font16.Width, 183, RSSI_str, &Font16, UNCOLORED);
  }
}

void ePaper::finish()
{

  // update view
  Serial.println("EPD memory set");
  DisplayPartFrame();

  // shut down ePaper
  Serial.println("e-Paper done; sending to sleep");
  Sleep();

  if (epaper.hasPowerPin())
  {
    // cut power
    digitalWrite(getPowerPin(), LOW);
  }
  xSemaphoreGive(semaphore_ePaper_done);
}

ePaper epaper;