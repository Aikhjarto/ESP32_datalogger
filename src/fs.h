#include <FS.h>

extern const char *LOGTAG_SPIFFS;

struct multi_write_log_t
{
    size_t bytes_written;
    size_t bytes_total;
};

char *readFile(fs::FS &fs, const char *path, bool null_terminated=false);
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
esp_err_t write_helper(fs::File file, uint8_t* data, size_t len, multi_write_log_t *log);
esp_err_t check_multi_write_log(multi_write_log_t *log);
