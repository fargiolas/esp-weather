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

#ifndef __USER_BME280_H__
#define __USER_BME280_H__

#include "esp_system.h"
#include "bme280.h"
#include "bme280_defs.h"

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_cleanup(void);
esp_err_t bme280_setup(struct bme280_dev *dev, uint8_t *addr);
esp_err_t bme280_read_forced(struct bme280_dev *dev, double *T, double *RH, double *P);
uint32_t  bme280_get_serial(struct bme280_dev *dev);

#endif /* __USER_BME280_H__ */

