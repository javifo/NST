/*
 * HEAD:include/tps65921.h
 *
 * Copyright (C) 2010-2011 Barnes & Noble, Inc.
 * Intrinsyc Software International, Inc. on behalf of Barnes & Noble, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _TPS_65921_H_
#define _TPS_65921_H_

#include <common.h>
#include <twl4030.h>

/* Register Offsets */
#define KEYP_CTRL			0x00
#define KEYP_DEB			0x01
#define KEYP_LONG_KEY		0x02
#define KEYP_LK_PTV			0x03
#define KEYP_TIMEOUT_L		0x04
#define KEYP_TIMEOUT_H		0x05
#define KEYP_KBC			0x06
#define KEYP_KBR			0x07
#define KEYP_SMS			0x08

/* KEYP_CTRL_REG Fields */
#define KEYP_CTRL_SOFT_NRST		(1 << 0)
#define KEYP_CTRL_SOFTMODEN		(1 << 1)
#define KEYP_CTRL_LK_EN			(1 << 2)
#define KEYP_CTRL_TOE_EN		(1 << 3)
#define KEYP_CTRL_TOLE_EN		(1 << 4)
#define KEYP_CTRL_RP_EN			(1 << 5)
#define KEYP_CTRL_KBD_ON		(1 << 6)

#define TPS65921_USB_SW_CHRG_CTRL_EN	(1 << 1)
#define TPS65921_USB_HW_CHRG_DET_EN	(1 << 0)

#define TPS65921_VUSB3V1_DEV_GRP	(0xD2)
#define TPS65921_VUSB1V8_DEV_GRP	(0xCF)
#define TPS65921_VUSB1V5_DEV_GRP	(0xCC)

#define TPS65921_USB_FUNC_CTRL			(0x04)
#define TPS65921_USB_FUNC_CTRL2			(0xB8)
#define TPS65921_FUNC_CTRL_USB_NON_DRIVING	(1 << 3)
#define TPS65921_FUNC_CTRL_USB_NON_DRIVING_MASK	(0x18)

#define TPS65921_USB_DTCT_CTRL		(0x76)
#define TPS65921_USB_DET_STS_MASK	(0x0C)
#define TPS65921_USB_DET_STS_NONE	(0x00)
#define TPS65921_USB_DET_STS_100	(0x04)
#define TPS65921_USB_DET_STS_500	(0x08)
#define TPS65921_USB_DET_STS_1000	(TPS65921_USB_DET_STS_100 | \
					 TPS65921_USB_DET_STS_500)

#define TPS65921_I2C_BUS_NUM		(0x00)

#define TPS65921_CHIP_KEYPAD		0x4a

/* Register Offsets */
#define TPS65921_KEYP_CTRL		0x00
#define TPS65921_KEYP_DEB		0x01
#define TPS65921_KEYP_LONG_KEY		0x02
#define TPS65921_KEYP_LK_PTV		0x03
#define TPS65921_KEYP_TIMEOUT_L		0x04
#define TPS65921_KEYP_TIMEOUT_H		0x05
#define TPS65921_KEYP_KBC		0x06
#define TPS65921_KEYP_KBR		0x07
#define TPS65921_KEYP_SMS		0x08

/* KEYP_CTRL_REG Fields */
#define TPS65921_KEYP_CTRL_SOFT_NRST	(1 << 0)
#define TPS65921_KEYP_CTRL_SOFTMODEN	(1 << 1)
#define TPS65921_KEYP_CTRL_LK_EN	(1 << 2)
#define TPS65921_KEYP_CTRL_TOE_EN	(1 << 3)
#define TPS65921_KEYP_CTRL_TOLE_EN	(1 << 4)
#define TPS65921_KEYP_CTRL_RP_EN	(1 << 5)
#define TPS65921_KEYP_CTRL_KBD_ON	(1 << 6)
#define TPS65921_ACCISR1			(0x79)
#define TPS65921_ACCIMR1			(0x7A)
#define TPS65921_I2C_BUS_NUM		(0x00)

#define CHARGER_DETECT_DWELL_TIME_US 100000
#define CHARGER_IRQ_TIMEOUT_MS		 1000

#define TPS65921_CHIP_KEYPAD		0x4a

/* Register Offsets */
#define TPS65921_KEYP_CTRL		0x00
#define TPS65921_KEYP_DEB		0x01
#define TPS65921_KEYP_LONG_KEY		0x02
#define TPS65921_KEYP_LK_PTV		0x03
#define TPS65921_KEYP_TIMEOUT_L		0x04
#define TPS65921_KEYP_TIMEOUT_H		0x05
#define TPS65921_KEYP_KBC		0x06
#define TPS65921_KEYP_KBR		0x07
#define TPS65921_KEYP_SMS		0x08

/* KEYP_CTRL_REG Fields */
#define TPS65921_KEYP_CTRL_SOFT_NRST	(1 << 0)
#define TPS65921_KEYP_CTRL_SOFTMODEN	(1 << 1)
#define TPS65921_KEYP_CTRL_LK_EN	(1 << 2)
#define TPS65921_KEYP_CTRL_TOE_EN	(1 << 3)
#define TPS65921_KEYP_CTRL_TOLE_EN	(1 << 4)
#define TPS65921_KEYP_CTRL_RP_EN	(1 << 5)
#define TPS65921_KEYP_CTRL_KBD_ON	(1 << 6)

#define TPS65921_STOPON_ENABLE 1
#define TPS65921_STOPON_DISABLE 0

int tps65921_get_charger_amps(void);
int tps65921_enable_charger_detect(void);
int tps65921_enable_usb_power(void);
int tps65921_disable_usb_transceiver(void);
int tps65921_shutdown(char * reason);
int tps65921_i2c_read(u8 mod, u8 reg, u8 * data);
int tps65921_i2c_write(u8 mod, u8 reg, u8 * data);
int tps65921_is_rtc_running(void);
void tps65921_set_watchdog(int time);
void tps65921_stopon_pwr_button(int state);
int tps65921_get_watchdog();
#endif /* _TPS_65921_H_ */
