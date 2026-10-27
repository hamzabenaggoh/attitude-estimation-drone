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
#include "board.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

uint32_t board_get_tick(void)
{
    return HAL_GetTick();
}

void board_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

uint32_t board_cycle_count(void)
{
    return DWT->CYCCNT;
}

void board_status_led_on(void)
{
    HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_SET);
}

void board_status_led_toggle(void)
{
    HAL_GPIO_TogglePin(status_led_GPIO_Port, status_led_Pin);
}

void board_usb_init(void)
{
    MX_USB_DEVICE_Init();
}

uint8_t board_usb_transmit(uint8_t *buf, uint16_t len)
{
    return CDC_Transmit_FS(buf, len);
}
