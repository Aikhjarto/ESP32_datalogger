#include "myLib.h"
#include <sstream> // string creating via streams
#include <iomanip> // stream settings
#include <errno.h>
#include <limits.h> // for LONG_MAX,...
#include <WiFi.h>   // for getting the max address
#include <esp_ota_ops.h> // for getting firmware status

/*
Convert a sequence of bytes to it's hex representation (all-lowercase hex string
and omitting the prefix "0x") as std::string, suitable e.g. for displaying a
MAC address.
*/
std::string bytes2hex(const uint8_t *bytes, size_t len)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < len; ++i)
    {
        ss << std::setw(2) << static_cast<unsigned>(bytes[i]);
    }
    return ss.str();
}

const char hexlut[16] = {
    '0',
    '1',
    '2',
    '3',
    '4',
    '5',
    '6',
    '7',
    '8',
    '9',
    'a',
    'b',
    'c',
    'd',
    'e',
    'f',
};
/*
Convert a sequence of bytes to it's hex representation (all-lowercase hex string
and omitting the prefix "0x")
*hex must point to a allocated space of 2*byteslen.
*/
void bytes2hex(const unsigned char *bytes, size_t byteslen, char *hex)
{
    while (byteslen--)
    {
        unsigned char chr = *bytes++;
        *hex++ = hexlut[chr >> 4];
        *hex++ = hexlut[chr & 0xf];
    }
}

/*
MAC address of WiFi interface as all lower-case hex-string
*/
std::string getMACasString(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    return bytes2hex(mac, 6);
}

/*
String to int conversion with error checking.
https://stackoverflow.com/a/6154614
*/
STR2INT_ERROR str2int(int *i, char const *s, int base)
{
    char *end;
    long l;
    errno = 0;
    l = strtol(s, &end, base);
    if ((errno == ERANGE && l == LONG_MAX) || l > INT_MAX)
    {
        return OVERFLOW;
    }
    if ((errno == ERANGE && l == LONG_MIN) || l < INT_MIN)
    {
        return UNDERFLOW;
    }
    if (*s == '\0' || *end != '\0')
    {
        return INCONVERTIBLE;
    }
    *i = l;
    return SUCCESS;
}

time_t cvt_TIME(char const *datetime)
{
    return cvt_TIME(datetime, datetime + 12);
}

/*
    date: "Aug  3 2022"
    time: "12:34:56"
*/
time_t cvt_TIME(char const *date, const char *time)
{

    char s_month[5];
    int month, year;
    struct tm t = {0};
    static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

    sscanf(date, "%s %d %d", s_month, &t.tm_mday, &year);

    month = (strstr(month_names, s_month) - month_names) / 3;

    t.tm_mon = month;
    t.tm_year = year - 1900;

    sscanf(time, "%d:%d:%d", &t.tm_hour, &t.tm_min, &t.tm_sec);

    t.tm_isdst = -1; // information is not available.

    return mktime(&t);
}

std::string systeminfo_json()
{

    std::stringstream ss;

    ss << "'{\"WiFi MAC\":" << getMACasString().c_str();
    ss << ",\"CPU Frequency\":" << getCpuFrequencyMhz();
    ss << ",\"XTAL Frequency\":" << getXtalFrequencyMhz();
    ss << ",\"APB Frequency\":" << getApbFrequency() / 1000000;
#ifdef EPAPER_SPI_CS_PIN
    ss << "\"SPI Clock Divider\"" << SPI.getClockDivider();
#endif
    ss << ",\"Setup core\":\"" << xPortGetCoreID();
    ss << "\",\"Compile Timestamp\":\"" << __DATE__ << " " << __TIME__;

    const esp_app_desc_t *esp_app_desc = esp_ota_get_app_description();
    ss << "\",\"FW version\":\"" << esp_app_desc->version;
    ss << "\",\"FW project name\":\"" << esp_app_desc->project_name;
    ss << "\",\"FW IDF VER\":\"" << esp_app_desc->idf_ver;
    ss << "\",\"FW time\":\"" << esp_app_desc->time;
    ss << "\",\"FW date\":\"" << esp_app_desc->date;
    ss << "\"}";
    return ss.str();
}

/*
For debugging, write wakeup reason in human readable form to Serial.
*/
void print_wakeup_reason(const char* LOGTAG)
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    ESP_LOGI(LOGTAG, "Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    ESP_LOGI(LOGTAG, "Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    ESP_LOGI(LOGTAG, "Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    ESP_LOGI(LOGTAG, "Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    ESP_LOGI(LOGTAG, "Wakeup caused by ULP program");
    break;
  default:
    ESP_LOGI(LOGTAG, "Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}
