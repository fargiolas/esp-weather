#include "esp_system.h"
#include "bme280.h"

struct bme280_dev bme;

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_cleanup(void);
esp_err_t bme280_setup(void);
esp_err_t bme280_read_forced(double *T, double *RH, double *P);


