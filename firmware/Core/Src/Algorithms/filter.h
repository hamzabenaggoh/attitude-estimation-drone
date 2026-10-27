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
#ifndef FILTER_H
#define FILTER_H
#include "quadfs_types.h"
#include "quadfs_config.h"
#include "iekf.h"
#include "complementary_filter.h"


typedef struct filter
{
	iekf_t imu_f_data;
	compf_t compf_data;
} filter_t;

void filter_init(uint32_t cycle_count);
void filter_update(const imu_t *imu, filter_t *out);


#endif // FILTER_H