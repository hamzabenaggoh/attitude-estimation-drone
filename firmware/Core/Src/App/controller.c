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
#include <stdbool.h>
#include "pid_controller.h"
#include "controller.h"
#include "controller_def.h"
#include "queue.h"
#include "radio.h"
#include "status_monitor.h"
#include "imu.h"
#include "board.h"

static enum control_states ctrl_state = controller_disabled;

static void controller_run(void *const params);
static void controller_process_data(const imu_t *imu, const filter_t *filter, controller_t *ctrl);
static void quadfs_status_indicator();

QueueHandle_t controller_telemetry_q;
TaskHandle_t controller_run_Handle;
BaseType_t controller_run_return_Handle;
static bool launch_once = true;

//ticks 1 tick -> 1ms, 3ticks -> 3ms -> 333Hz  loop freq
//------------------------------------------------------------
// control loop freq(set to 333Hz) < PWM freq (set to 400Hz)
// to allow for pwm/motor updating without interruption
//------------------------------------------------------------
static uint8_t tick_time = TASK_INTERVAL_MS;

static sensor_t sensor_data;
static filter_t filter_data;
static controller_t ctrl_data;

// Public members ========================================================
BaseType_t controller_launch_thread(void)
{
    if (launch_once)
    {
        controller_telemetry_q = xQueueCreate(QUEUE_LENGTH, sizeof(controller_t));
        controller_run_return_Handle = xTaskCreate(controller_run, "Controller Activity Task",
                                                  configMINIMAL_STACK_SIZE*8,
                                                   NULL, CONTROLLER_PRIORITY, &controller_run_Handle);
        launch_once = false;
    }
    return controller_run_return_Handle;
}

// Private members =======================================================

static void controller_run(void *const params)
{
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        update_channels();
        sensor_update(&sensor_data);
        filter_update(&sensor_data.imu_data, &filter_data);
        controller_process_data(&sensor_data.imu_data, &filter_data, &ctrl_data);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(tick_time));
    }
}

static void controller_process_data(const imu_t *imu, const filter_t *filter, controller_t *ctrl)
{
    //SWA 1000 is up(use for off or disarm), 2000 is down (for on or arm )
    uint16_t CH_MID = 1500;

    switch (ctrl_state)
    {
        case safe_start:
            ctrl->pwm_data = turn_motors_off();
            if (channels[CH_SWA] < CH_MID)
            {
                ctrl_state = start_stage;
            }
        case start_stage:
            ctrl->pwm_data = turn_motors_off();
            if (channels[CH_SWA] > CH_MID)
            {
                init_setpoints(filter->imu_f_data.yaw);
                init_pid_attitude_heading_controller((float)(tick_time*0.001f), board_get_tick());
                ctrl_state = controller_enabled;
            }
            break;
        case controller_enabled:
            if (channels[CH_SWA] < CH_MID)
            {
                ctrl_state = controller_disabled;
                break;
            }

            ctrl->setpoint_data = update_setpoints(channels, imu->gyroZnf,
                                                   filter->imu_f_data.yaw, 
                                                   tick_time);
            ctrl->pwm_data = pid_attitude_heading_controller(imu, 
                                                             &ctrl->setpoint_data,
                                                             &filter->imu_f_data);

            set_motors_pwm(ctrl->pwm_data);
            break;
        case controller_disabled:
            ctrl->pwm_data = turn_motors_off();
            init_setpoints(filter->imu_f_data.yaw);
            init_pid_attitude_heading_controller((float)(tick_time * 0.001f), board_get_tick());
            ctrl_state = safe_start;
            break;
        default:
            break;
    }


    quadfs_status(ctrl_state);  
}

const sensor_t *ctrl_sensor_data(void)
{
    return &sensor_data;
}

const filter_t *ctrl_filter_data(void)
{
    return &filter_data;
}

const controller_t *ctrl_output_data(void)
{
    return &ctrl_data;
}
