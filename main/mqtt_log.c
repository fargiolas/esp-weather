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

/* ESP8266 doesn't allow to overload vsprintf for custom logging, but
 * we can replace or extend putchar. The idea here is to accumulate
 * all chars from putchar in a buffer and send them through MQTT when
 * we get a new line */

#include <limits.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include "FreeRTOSConfig.h"
#include "esp_log.h"
#include "mqtt_client.h"
 #include "mqtt_log.h"
#include "portmacro.h"
#include "freertos/message_buffer.h"
#include "projdefs.h"

/* MQTT isn't available right from the start as it takes a while to
 * connect to wifi and to the broker. Accumulate some logs while
 * waiting for the connection to come up. Still have to tune how much
 * can we cache without to waste to much memory. It's not really
 * needed for normal operation but esp_https_ota wants most of the
 * memory for the update process */
#define MAX_MSG_LEN 150
#define MAX_MSGS 30
#define MSG_QUEUE_LEN (MAX_MSG_LEN + 1) * MAX_MSGS

#define MQTT_LOG_TOPIC CONFIG_MQTT_TOPIC "/info"

QueueHandle_t mqtt_log_client_queue = NULL;
QueueHandle_t mqtt_log_char_queue = NULL;

TaskHandle_t mqtt_log_pub_task_handle = NULL;

/* FIXME:

   - what happens when we get longer lines than MAX_MSG_LEN?

   - what happens if MQTT doesn't connect and we accumulate more lines
     than we can store? xQueueSend in mqtt_log_putchar will silently
     fail so we'll lose new messages, it will be nice to have some
     sort of ring buffer that clears older elements as soon as we
     overfill it.
*/
static void mqtt_log_pub_task(void *params)
{
    char c = 0;
    char buf[MAX_MSG_LEN+1];
    uint8_t cursor = 0;
    esp_mqtt_client_handle_t client;

    /* wait for the mqtt connection before publishing logs */
    xQueueReceive(mqtt_log_client_queue, &client, portMAX_DELAY);

    while (1) {
        /* receive a full line before publishing */
        while (c != '\n') {
            xQueueReceive(mqtt_log_char_queue, &c, portMAX_DELAY);
            buf[cursor] = c;
            cursor++;
        }

        c = '\0';
        buf[cursor-1] = c;

        esp_mqtt_client_publish(client, MQTT_LOG_TOPIC, buf, 0, 1, 0);

        cursor = 0;
    }
}

/* override default logging putchar with out method */
static int mqtt_log_putchar(int c) {
    xQueueSend(mqtt_log_char_queue, &c, 0);
    return putchar(c);
}

/* this should be set once we have a valid mqtt client, ideally with
 * an active connection to the broker */
void mqtt_log_set_mqtt_client(esp_mqtt_client_handle_t client)
{
    if (mqtt_log_client_queue != NULL) {
        xQueueSend(mqtt_log_client_queue, &client, pdMS_TO_TICKS(1000));
    }
}

void mqtt_log_init(void) {
    mqtt_log_char_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(char));
    mqtt_log_client_queue = xQueueCreate(1, sizeof(esp_mqtt_client_handle_t));
    esp_log_set_putchar(mqtt_log_putchar);
    xTaskCreate(mqtt_log_pub_task, "mqtt_log_pub_task", 1024, NULL, 2, &mqtt_log_pub_task_handle);
}
