#include <esp_http_client.h>
#include <esp_err.h>

/*
https://github.com/espressif/esp-idf/blob/master/examples/protocols/esp_http_client/main/esp_http_client_example.c
*/
esp_err_t _http_event_handler(esp_http_client_event_t *evt);

void ota_update(const char* update_url);

extern const char * LOGTAG_OTA;