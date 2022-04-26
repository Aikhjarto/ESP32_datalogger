#include <string> // stream settings

/*
Convert a sequence of bytes to it's hex representation (all-lowercase hex string
and omitting the prefix "0x") as std::string, suitable e.g. for displaying a
MAC address.
*/
std::string bytes2hex(const uint8_t *bytes, size_t len);

/*
Convert a sequence of bytes to it's hex representation (all-lowercase hex string
and omitting the prefix "0x")
*hex must point to a allocated space of 2*byteslen.
*/
void bytes2hex(const unsigned char *bytes, size_t byteslen, char *hex);

/*
MAC address of WiFi interface as all lower-case hex-string
*/
std::string getMACasString(void);

enum STR2INT_ERROR
{
    SUCCESS,
    OVERFLOW,
    UNDERFLOW,
    INCONVERTIBLE
};
/*
String to int conversion with error checking.
https://stackoverflow.com/a/6154614
*/
STR2INT_ERROR str2int(int *i, char const *s, int base = 0);