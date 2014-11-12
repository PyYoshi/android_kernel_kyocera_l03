/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 *
 * drivers/input/touchscreen/board-8974-touch.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#ifndef BOARD_8974_TOUCH_H
#define BOARD_8974_TOUCH_H

#ifdef CONFIG_TOUCHSCREEN_PIXART_KC
#include <linux/kernel.h>
#include <linux/gpio.h>
//#include <linux/i2c/pixart.h>
#include "kc_ts.h"

#define PIXART_TS_RESET_GPIO	60
#define PIXART_TS_GPIO_IRQ		61

#define PIXART_PANEL_WIDTH		1920
#define PIXART_PANEL_HEIGHT		1080

#define PIXART_BOARD_INFO		I2C_BOARD_INFO(PIXART_I2C_NAME, 0x66 >> 1)

static inline int pixart_init_hw(void)
{
	int ret;

	ret = gpio_request(PIXART_TS_GPIO_IRQ, "pixart_ts_irq_gpio");
	if (ret) {
		printk("%s: unable to request pixart_ts_irq gpio [%d]\n"
						, __func__, PIXART_TS_GPIO_IRQ);
		goto err_irq_gpio_req;
	}
	ret = gpio_direction_input(PIXART_TS_GPIO_IRQ);
	if (ret) {
		printk("%s: unable to set_direction for pixart_ts_irq gpio [%d]\n"
						, __func__, PIXART_TS_GPIO_IRQ);
		goto err_irq_gpio_set;
	}
	ret = gpio_request(PIXART_TS_RESET_GPIO, "pixart_reset_gpio");
	if (ret) {
		printk("%s: unable to request pixart_reset gpio [%d]\n"
						, __func__, PIXART_TS_RESET_GPIO);
		goto err_reset_gpio_req;
	}
	ret = gpio_direction_output(PIXART_TS_RESET_GPIO, 0);
	if (ret) {
		printk("%s: unable to set_direction for pixart_reset gpio [%d]\n"
						, __func__, PIXART_TS_RESET_GPIO);
		goto err_reset_gpio_set;
	}

	return 0;

err_reset_gpio_set:
	gpio_free(PIXART_TS_RESET_GPIO);
err_reset_gpio_req:
err_irq_gpio_set:
	gpio_free(PIXART_TS_GPIO_IRQ);
err_irq_gpio_req:
	return ret;
}

static inline int pixart_reset_hw(void)
{
	KC_TS_DEV_DBG("%s: start\n",__func__);
	usleep_range(10000, 10000);
	gpio_set_value(PIXART_TS_RESET_GPIO, 0);
	usleep_range(1000, 1000);
	gpio_set_value(PIXART_TS_RESET_GPIO, 1);
	msleep(20);

	return 0;
}

static inline int pixart_shutdown_hw(void)
{
	KC_TS_DEV_DBG("%s: start\n",__func__);
	gpio_set_value(PIXART_TS_RESET_GPIO, 0);
	usleep_range(1000, 1000);

	return 0;
}

static u8 pixart_config_data[] = {
	0x05, 1, 0x17,		0x06, 1, 0x3F,
	0x07, 1, 0xCF,		0x08, 1, 0xC8,
	0x09, 1, 0x5A,		0x0F, 1, 0x00,
	0x15, 1, 0x03,		0x16, 1, 0x01,
	0x17, 1, 0x00,		0x20, 1, 0xA0,
	0x28, 1, 0x90,		0x29, 1, 0x01,
	0x2A, 1, 0x50,		0x2B, 1, 0x32,
	0x2C, 1, 0xB0,		0x2D, 1, 0x04,
	0x2E, 1, 0x95,		0x2F, 1, 0x02,
	0x41, 1, 0x0E,		0x42, 1, 0x16,
	0x43, 1, 0x14,		0x44, 1, 0x38,
	0x45, 1, 0x04,		0x46, 1, 0x80,
	0x47, 1, 0x07,		0x4A, 1, 0x0A,
	0x51, 1, 0x03,		0x53, 1, 0x14,
	0x56, 1, 0xF4,		0x57, 1, 0x01,
	0x58, 1, 0x2C,		0x59, 1, 0x01,
	0x5B, 1, 0x11,		0x5C, 1, 0x28,
	0x5D, 1, 0x0A,		0x60, 1, 0xB8,
	0x61, 1, 0x0B,		0x62, 1, 0x00,
	0x63, 1, 0x0A,		0x64, 1, 0x0A,
	0x65, 1, 0x0A,		0x66, 1, 0x0A,
	0x67, 1, 0x64,		0x75, 1, 0x44,
	0x77, 1, 0x01,
};

static u8 pixart_extended_data[] = {
	0x24, 1, 0x00,	0x25, 1, 0x3F,	0x26, 1, 0x09,
	0x24, 1, 0x00,	0x25, 1, 0x41,	0x26, 1, 0x02,
	0x24, 1, 0x00,	0x25, 1, 0x70,	0x26, 1, 0xF4,
	0x24, 1, 0x00,	0x25, 1, 0x71,	0x26, 1, 0x01,
	0x24, 1, 0x00,	0x25, 1, 0x78,	0x26, 1, 0xC8,
};

static u8 pixart_config_discharging[] = {
	0x10, 1, 0x0B,		0x11, 1, 0x0F,
	0x12, 1, 0x19,		0x13, 1, 0x23,
	0x18, 1, 0x0A,		0x19, 1, 0x4E,
	0x1A, 1, 0x27,		0x1B, 1, 0x05,
};

static u8 pixart_config_charging[] = {
	0x10, 1, 0x0B,		0x11, 1, 0x0F,
	0x12, 1, 0x19,		0x13, 1, 0x23,
	0x18, 1, 0x0A,		0x19, 1, 0x4E,
	0x1A, 1, 0x27,		0x1B, 1, 0x05,
};

static u8 pixart_config_charging_s1[] = {
	0x10, 1, 0x0B,		0x11, 1, 0x0F,
	0x12, 1, 0x19,		0x13, 1, 0x23,
	0x18, 1, 0x0A,		0x19, 1, 0x4E,
	0x1A, 1, 0x27,		0x1B, 1, 0x05,
};

static u8 pixart_config_charging_s2[] = {
	0x10, 1, 0x0B,		0x11, 1, 0x0F,
	0x12, 1, 0x19,		0x13, 1, 0x23,
	0x18, 1, 0x0A,		0x19, 1, 0x4E,
	0x1A, 1, 0x27,		0x1B, 1, 0x05,
};

static u8 pixart_config_charging_wireless[] = {
	0x10, 1, 0x0B,		0x11, 1, 0x0F,
	0x12, 1, 0x19,		0x13, 1, 0x23,
	0x18, 1, 0x0A,		0x19, 1, 0x4E,
	0x1A, 1, 0x27,		0x1B, 1, 0x05,
};

#endif /* CONFIG_TOUCHSCREEN_PIXART_KC */
#endif /* BOARD_8974_TOUCH_H */
