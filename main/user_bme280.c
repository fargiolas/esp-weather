#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "bme280.h"
#include "user_bme280.h"

#define I2C_SCL_IO           2                /*!< gpio number for I2C master clock */
#define I2C_SDA_IO           0                /*!< gpio number for I2C master data  */
#define I2C_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define BME280_ADDR                         0x76
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

static uint8_t bme280_i2c_addr;
static const char *TAG = "bme280";

esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300;
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

esp_err_t i2c_master_cleanup()
{
    i2c_driver_delete(I2C_NUM);
    return ESP_OK;
}

static int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_id = *((uint8_t *) intf_ptr);
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_id << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_id << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, reg_data, len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_id = *((uint8_t *) intf_ptr);
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, dev_id << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, (uint8_t *) reg_data, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;

}

void user_delay_us(uint32_t period, void *intf_ptr)
{
    ets_delay_us(period);
}

esp_err_t bme280_setup(void)
{
    int8_t rslt = BME280_OK;

    bme280_i2c_addr = BME280_I2C_ADDR_PRIM;
    bme.intf = BME280_I2C_INTF;
    bme.intf_ptr = &bme280_i2c_addr;
    bme.read = user_i2c_read;
    bme.write = user_i2c_write;
    bme.delay_us = user_delay_us;

    if (bme280_init(&bme) != BME280_OK) {
        ESP_LOGE(TAG, "device absent");
        return ESP_FAIL;
    }

    bme.settings.osr_h = BME280_OVERSAMPLING_1X;
    bme.settings.osr_p = BME280_OVERSAMPLING_1X;
    bme.settings.osr_t = BME280_OVERSAMPLING_1X;
    bme.settings.filter = BME280_FILTER_COEFF_OFF;

    int settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL |
        BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    rslt = bme280_set_sensor_settings(settings_sel, &bme);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "set sensor settings failed, (code %d).", rslt);
        return ESP_FAIL;
    }

    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme);
    if (rslt != BME280_OK){
        ESP_LOGE(TAG, "set forced mode failed, (code %d).", rslt);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t bme280_read_forced(double *T, double *RH, double *P)
{
    uint32_t req_delay;
    int8_t rslt = BME280_OK;

    rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme);
    if (rslt != BME280_OK){
        ESP_LOGE(TAG, "bme: set forced mode failed, (code %d).\n", rslt);
        return ESP_FAIL;
    }

    req_delay = bme280_cal_meas_delay(&bme.settings);
    bme.delay_us(req_delay, bme.intf_ptr);

    struct bme280_data comp_data;

    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme);
    if (rslt != BME280_OK) {
        ESP_LOGE(TAG, "Failed to get sensor data (code %d).\n", rslt);
        return ESP_FAIL;
    }

    *T = comp_data.temperature / 100.;
    *P = comp_data.pressure / 100.;
    *RH = comp_data.humidity / 1024.;

    return ESP_OK;
}
