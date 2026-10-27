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
#ifndef QUADFS_DEF_H
#define QUADFS_DEF_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "quadfs_config.h"


#define CONTROLLER_PRIORITY 3
#define FILTER_PRIORITY 3
#define SENSOR_PRIORITY 3
#define TELEMETRY_PRIORITY 2
#define MONITOR_PRIORITY 1

#define STACK_SIZE 128 * 8
#define QUEUE_LENGTH 5

#define  FIRMWARE_VERSION_STRING "QuadFS v1.1.2"


#define RADIO_MIN   1000U
#define RADIO_MAX   2000U

extern QueueHandle_t sensor_telemetry_q;
extern QueueHandle_t filter_telemetry_q;
extern QueueHandle_t controller_telemetry_q;

extern QueueHandle_t sensor_filter_q;
extern QueueHandle_t filter_controller_q;

extern TaskHandle_t Status_indicator_Handle;
extern TaskHandle_t controller_run_Handle;
extern TaskHandle_t telemetry_run_Handle;

enum control_states
{
    safe_start,
    start_stage,
    second_start_stage,
    controller_enabled,
    controller_disabled,
    fault,
    end_control
};
#endif // QUADFS_DEF_H
