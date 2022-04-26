#include "myLib.h"
#include <sstream> // string creating via streams
#include <iomanip> // stream settings
#include <errno.h>
#include <limits.h> // for LONG_MAX,...
#include <WiFi.h>   // for getting the max address

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
