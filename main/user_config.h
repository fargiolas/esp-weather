#include "esp_system.h"
#include "esp_err.h"

void config_parse_payload(char *msg, size_t msg_len);
esp_err_t config_read(char *key, void *data);
