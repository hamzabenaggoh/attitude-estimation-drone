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
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include "sensor.h"
#include "imu.h"
#include "filter.h"
#include "status_monitor.h"
#include "radio.h"
#include "quadfs_app.h"
#include "motor_pwm.h"
#include "controller.h"
#include "board.h"
#include "telemetry.h"
#include "i2c.h"

void init_hardware(){

  //   // turn on counter if doing  loop measurement
  // CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  // DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


  board_usb_init();
  init_mpu();
  motor_init();
  usart_radio_dma_start();

  board_delay_ms(200); // wait before task  startup

  filter_init(board_cycle_count());

  controller_launch_thread();
  status_monitor_launch();
  telemetry_launch_thread();

  static char startup_msg[200];

  int len = snprintf(
      startup_msg,
      sizeof(startup_msg),
      "QuadFS initialization complete, starting up...\r\n"
      "Firmware: %s\r\n",
      FIRMWARE_VERSION_STRING
  );

  board_usb_transmit((uint8_t *)startup_msg,(uint16_t)len);

  vTaskStartScheduler();

}