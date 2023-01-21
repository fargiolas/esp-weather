/*
 * Copyright (c) 2023 Filippo Argiolas <filippo.argiolas@gmail.com>.
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
#include "esp_err.h"
#include "esp_system.h"
#include "esp_app_format.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "portmacro.h"
#include "projdefs.h"
#include "sdkconfig.h"
#include "user_wifi.h"
#include "user_bme280.h"
#include "user_config.h"
#include "math.h"
#include "util.h"

#define OTA_TOPIC CONFIG_MQTT_TOPIC "/ota"
#define CONF_TOPIC CONFIG_MQTT_TOPIC "/config"

#define STATUS_LED 16


static const char *TAG = "main";

esp_mqtt_client_handle_t client;

struct bme280_dev bme;
static uint8_t bme280_i2c_addr = CONFIG_I2C_BME280_ADDRESS;
static uint32_t bme_serial;
static uint32_t sampling_delay = CONFIG_SAMPLING_DELAY;
static double humidity_correction = CONFIG_HUMIDITY_CORRECTION;
static TaskHandle_t publish_task_handle = NULL;

char ota_url[2048];
static TaskHandle_t ota_task_handle = NULL;
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

static int mqtt_log(const char *topic, const char *fmt, ...)
{
    char payload[256];
    va_list l;
    va_start(l, fmt);
    vsprintf(payload, fmt, l);
    va_end(l);

    esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);

    return 0;
}

/* super basic http event handler for OTA updates debugging */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}


/* super simple OTA update task using esp_https_ota SDK API MQTT event
   handler sends a direct notification to this task when a new url is
   received on the ota topic. This task waits for the notification and
   proceeds with the upgrade once the new firmware url is received */
static void ota_task(void *params)
{
    while(1) {
        /* wait for OTA notification from the MQTT task */
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        vTaskSuspend(publish_task_handle);

        ESP_LOGI(TAG, "Starting OTA upgrade process from: %s", ota_url);
        esp_http_client_config_t config = {
            .url = ota_url,
            .cert_pem = (char *)server_cert_pem_start,
            .event_handler = http_event_handler,
        };
        esp_err_t ret = esp_https_ota(&config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "OTA upgrade successful, restarting the device");
            esp_restart();
        } else {
            ESP_LOGE(TAG, "Firmware Upgrades Failed");
        }

        /* clear ota requests we received while performing an upgrade */
        xTaskNotifyStateClear(ota_task_handle);
        vTaskResume(publish_task_handle);
    }
}

static void publish_sensor_data(void *params)
{
    double T, RH, P;

    while (1) {
        if (bme280_read_forced(&bme, &T, &RH, &P) == ESP_OK) {
            RH += humidity_correction;

            ESP_LOGI(TAG, "T: %d.%02d °C, RH: %d.%02d%%, P: %d.%02d hPa",
                     INT(T), DEC(T), INT(RH), DEC(RH), INT(P), DEC(P));

            mqtt_log(CONFIG_MQTT_TOPIC "/temperature", "%d.%02d", INT(T), DEC(T));
            mqtt_log(CONFIG_MQTT_TOPIC "/humidity", "%d.%02d", INT(RH), DEC(RH));
            mqtt_log(CONFIG_MQTT_TOPIC "/pressure", "%d.%02d", INT(P), DEC(P));
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
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

            /* log app description to mqtt */
            const esp_app_desc_t *app = esp_ota_get_app_description();
            mqtt_log(CONFIG_MQTT_TOPIC "/info", "%s v%s -- %s %s", app->project_name, app->version, app->date, app->time);
            mqtt_log(CONFIG_MQTT_TOPIC "/info", "bme280: 0x%X", bme_serial);
            mqtt_log(CONFIG_MQTT_TOPIC "/info", "humidity_correction: %d.%02d", INT(humidity_correction), DEC(humidity_correction));

            if (publish_task_handle == NULL) {
                xTaskCreate(publish_sensor_data, "publish_sensor_data",
                            2048, NULL,
                            tskIDLE_PRIORITY+1, &publish_task_handle);
            }

            if (ota_task_handle == NULL) {
                xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, &ota_task_handle);
            }

            ESP_LOGI(TAG, "Subscribing to %s for ota updates", OTA_TOPIC);
            esp_mqtt_client_subscribe(client, OTA_TOPIC, 1);
            ESP_LOGI(TAG, "Subscribing to %s for ota config", CONF_TOPIC);
            esp_mqtt_client_subscribe(client, CONF_TOPIC, 1);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT EVENT SUBSCRIBED");
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            break;
        case MQTT_EVENT_PUBLISHED:
            break;
        case MQTT_EVENT_DATA:
            /* Notify ota task when a new url is received on the OTA topic */
            if (!strncmp(event->topic, OTA_TOPIC, strlen(OTA_TOPIC))) {
                char topic_buf[64+1];
                int topic_len = event->topic_len < 64 ? event->topic_len : 64;
                int url_len = event->data_len < sizeof(ota_url) ? event->data_len : sizeof(ota_url);

                strlcpy(topic_buf, event->topic, topic_len+1);
                strlcpy(ota_url, event->data, url_len+1);
                topic_buf[topic_len+1] = '\0';
                ota_url[url_len+1] = '\0';
                ESP_LOGI(TAG, "DATA: %s: %s", topic_buf, ota_url);

                xTaskNotify(ota_task_handle, 0, eNoAction);
            } else if (!strncmp(event->topic, CONF_TOPIC, strlen(CONF_TOPIC))) {
                config_parse_payload(event->data, event->data_len);
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
            break;
        default:
            ESP_LOGE(TAG, "UNKWOWN MQTT ERROR");
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
    char *broker_url;
    config_read("mqtt_broker_url", &broker_url);
    ESP_LOGI(TAG, "Setting broker url: %s", broker_url);

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = broker_url,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);

    os_free(broker_url);
}

static void sensors_init(void)
{
    int32_t offset = 0;

    i2c_master_init();
    ESP_ERROR_CHECK(bme280_setup(&bme, &bme280_i2c_addr));
    bme_serial = bme280_get_serial(&bme);

    config_read("humidity_offset", &offset);
    humidity_correction = (double) (offset) / 1000.;
    ESP_LOGI(TAG, "Setting humidity offset: %d.%02d",
             INT(humidity_correction), DEC(humidity_correction));
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_LOGI(TAG, "Initializing WIFI in Station Mode");
    wifi_init_sta();
    ESP_LOGI(TAG, "I2C and sensors setup");
    sensors_init();
    ESP_LOGI(TAG, "Initializing MQTT Client");
    mqtt_app_start();
}
