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

#ifndef __MQTT_LOG_H__
#define __MQTT_LOG_H__


#include <stdint.h>
#include <stdarg.h>

#include "esp_log.h"
#include "mqtt_client.h"
#include "sdkconfig.h"

void mqtt_log_init(void);
void mqtt_log_set_mqtt_client(esp_mqtt_client_handle_t client);

void mqtt_log_pause(void);
void mqtt_log_resume(void);


#endif /* __MQTT_LOG_H__ */
