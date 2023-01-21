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

#ifndef _UTIL_H
#define _UTIL_H

#include "esp_system.h"
#include "math.h"

#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))
#define CLAMP(x,a,b) (MAX((a), MIN((x), (b))))
#define DEC(f) (uint32_t)(fabs(fmod(f * 100, 100.)))
#define INT(f) (int32_t)(f)

/* statically allocated arrays only */
#define N_ELEMENTS(arr) (sizeof (arr) / sizeof ((arr)[0]))

#endif /* _UTILS_H */
