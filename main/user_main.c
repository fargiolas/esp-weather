/*
 * Copyright (c) 2022 Filippo Argiolas <filippo.argiolas@gmail.com>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "FreeRTOSConfig.h"
#include "bme280_defs.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "projdefs.h"
#include "sdkconfig.h"
#include "user_wifi.h"
#include "user_bme280.h"

static const char *TAG = "main";

esp_mqtt_client_handle_t client;

struct bme280_dev bme;
static uint8_t bme280_i2c_addr = CONFIG_I2C_BME280_ADDRESS;
static uint32_t bme_serial;
static uint32_t sampling_delay = CONFIG_SAMPLING_DELAY;
static double humidity_correction = CONFIG_HUMIDITY_CORRECTION;
static TaskHandle_t publish_task_handle = NULL;

static void publish_sensor_data(void *params)
{
    double T, RH, P;
    char payload[16];
    char topic[64];

    while (1) {
        if (bme280_read_forced(&bme, &T, &RH, &P) == ESP_OK) {
            RH += humidity_correction;

            memset(topic, 0, sizeof(topic));
            memset(payload, 0, sizeof(payload));
            sprintf(topic, "%s/temperature", CONFIG_MQTT_TOPIC);
            sprintf(payload, "%d.%02u", (int32_t) T, (uint32_t) (T * 100) % 100);
            esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);
            ESP_LOGI(TAG, "bme280 (0x%X): temperature: %s C", bme_serial, payload);

            memset(topic, 0, sizeof(topic));
            memset(payload, 0, sizeof(payload));
            sprintf(topic, "%s/humidity", CONFIG_MQTT_TOPIC);
            sprintf(payload, "%d.%02u", (int32_t) RH, (uint32_t) (RH * 100) % 100);
            esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);
            ESP_LOGI(TAG, "bme280 (0x%X): humidity: %s%%", bme_serial, payload);

            memset(topic, 0, sizeof(topic));
            memset(payload, 0, sizeof(payload));
            sprintf(topic, "%s/pressure", CONFIG_MQTT_TOPIC);
            sprintf(payload, "%d.%02u", (int32_t) P, (uint32_t) (P * 100) % 100);
            esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);
            ESP_LOGI(TAG, "bme280 (0x%X): pressure: %s Pa", bme_serial, payload);
        } else {
            /* should we trigger a reset here or wait for the problem to solve itself? */
            ESP_LOGE(TAG, "Cannot read from BME280");
        }

        vTaskDelay(pdMS_TO_TICKS(sampling_delay));
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            if (publish_task_handle == NULL) {
                xTaskCreate(publish_sensor_data, "publish_sensor_data",
                            2048, NULL,
                            tskIDLE_PRIORITY+1, &publish_task_handle);
            }
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
        case MQTT_EVENT_UNSUBSCRIBED:
        case MQTT_EVENT_PUBLISHED:
        case MQTT_EVENT_DATA:
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

static void sensors_init(void)
{
    i2c_master_init();
    ESP_ERROR_CHECK(bme280_setup(&bme, &bme280_i2c_addr));
    bme_serial = bme280_get_serial(&bme);
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_LOGI(TAG, "Initializing WIFI in Station Mode");
    wifi_init_sta();
    ESP_LOGI(TAG, "Initializing MQTT Client");
    sensors_init();
    mqtt_app_start();
}
