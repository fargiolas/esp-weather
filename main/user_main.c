#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "user_wifi.h"
#include "user_bme280.h"

static const char *TAG = "main";

esp_mqtt_client_handle_t client;

static void publish_sensor_data(void *params)
{
    i2c_master_init();
    bme280_setup();

    while (1) {
        double T, RH, P;
        char T_buf[64];
        char RH_buf[64];
        char P_buf[64];
        char topic[64];

        if (bme280_read_forced(&T, &RH, &P) != ESP_OK) {
            continue;
        }

        sprintf(T_buf, "%d.%02u", (int32_t) T, (uint32_t) (T * 100) % 100);
        sprintf(RH_buf, "%d.%02u", (int32_t) RH, (uint32_t) (RH * 100) % 100);
        sprintf(P_buf, "%d.%02u", (int32_t) P, (uint32_t) (P * 100) % 100);

        ESP_LOGI(TAG, "%s C, %s%%, %s Pa", T_buf, RH_buf, P_buf);

        memset(topic, 0, sizeof(topic));
        sprintf(topic, "%s/temperature", CONFIG_MQTT_TOPIC);
        esp_mqtt_client_publish(client, topic, T_buf, 0, 1, 0);
        memset(topic, 0, sizeof(topic));
        sprintf(topic, "%s/humidity", CONFIG_MQTT_TOPIC);
        esp_mqtt_client_publish(client, topic, RH_buf, 0, 1, 0);
        memset(topic, 0, sizeof(topic));
        sprintf(topic, "%s/pressure", CONFIG_MQTT_TOPIC);
        esp_mqtt_client_publish(client, topic, P_buf, 0, 1, 0);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    i2c_master_cleanup();
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            xTaskCreate(publish_sensor_data, "publish_sensor_data", 2048, NULL, 10, NULL);
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

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_LOGI(TAG, "Initializing WIFI in Station Mode");
    wifi_init_sta();
    ESP_LOGI(TAG, "Initializing MQTT Client");
    mqtt_app_start();
}
