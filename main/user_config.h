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

#include "esp_system.h"
#include "esp_err.h"

#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

void config_parse_payload(char *msg, size_t msg_len);
esp_err_t config_read(char *key, void *data);

#endif /* __USER_CONFIG_H__ */
