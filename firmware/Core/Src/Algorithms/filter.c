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
#include "filter.h"
#include "filter_def.h"


#define M_PI 3.14159265358979323846f /* pi */
#define IMU_ROLL_TILT_OFFSET -2.735f
#define IMU_PITCH_TILT_OFFSET -4.595f
static int tick_time = TASK_INTERVAL_MS;


float dt = 0.0f;
uint32_t last_cycle_count;

// Public members ========================================================
void filter_init(uint32_t cycle_count)
{
    last_cycle_count = cycle_count;
}

void filter_update(const imu_t *imu, filter_t *out)
{
    // //for loop measurement
    // uint32_t current_cycle_count = DWT->CYCCNT;
    // uint32_t elapsed_cycles = current_cycle_count - last_cycle_count;
    // last_cycle_count = current_cycle_count;
    // dt = (float)elapsed_cycles / (float)SystemCoreClock;

    float to_deg = TO_DEG;
    float to_rad = TO_RAD;
    float declination = DECLINATION;
    float new_angle[2];

    // comp_filter_angle(dt,self->imu_data, new_angle,2);
    // self->compf_data.roll = new_angle[0];
    // self->compf_data.pitch = new_angle[1];

    iekf_t iekf_angles = iekf_orientation_estimation(tick_time, *imu);
    out->imu_f_data = iekf_angles;
    out->imu_f_data.roll =  out->imu_f_data.roll - IMU_ROLL_TILT_OFFSET;//find and apply your own imu mounting offset using step 2 in the readme
    out->imu_f_data.pitch =  -1.0f*out->imu_f_data.pitch - IMU_PITCH_TILT_OFFSET;//find and apply your own imu mounting offset using step 2 in the readme
}
