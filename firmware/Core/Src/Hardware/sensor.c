/**
 ******************************************************************************
 *  QuadFS - quadcopter flight controller built from scratch on STM32
 *
 *  Copyright (C) 2026  Henry Odoemelem <huobuilds@gmail.com>
 *
 *  SPDX-License-Identifier: GPL-3.0-or-later
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <stdbool.h>
#include "sensor.h"
#include "sensor_def.h"
#include "stm32f4xx_hal.h"
#include "imu.h"

extern UART_HandleTypeDef huart2;

// Public members ========================================================
void sensor_update(sensor_t *out)
{
	out->imu_data = update_mpu_imu();
}
