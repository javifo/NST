/*
 * bat_i2c_id.c
 *
 *  Created on: Feb 10, 2011
 *      Author: x0151843
 */
#include <common.h>
#include <command.h>
#include <i2c.h>
#include <asm/byteorder.h>
#include <asm/mach-types.h>
#include "bat_i2c_id.h"

int set_bus(uchar bus_ix, uchar bus_sp)
{
	int res = 0;
	res = select_bus(bus_ix, bus_sp);
	if (res)
		return res;
	else
		return 0;
}

int get_battery_id(void)
{
	uchar LSB = 0, MSB = 0;
	uchar flag = 1, bit = 0;
	ulong result = 0, volt = 0;

	if (set_bus(BAT_BUS_INDEX, CFG_I2C_SPEED) != 0)
		goto exit;
	if (do_bat_i2c_write(BAT_INTBR, BAT_GPBR1, 0x90) != 0)
		goto exit;
	if (do_bat_i2c_write(BAT_MADC, BAT_CTRL1, 0x01) != 0)
		goto exit;
	if (do_bat_i2c_write(BAT_MADC, BAT_SW1SELECT_LSB, 0x01) != 0)
		goto exit;
	if (do_bat_i2c_write(BAT_MADC, BAT_CTRL_SW1, 0xfe) != 0)
		goto exit;
	while (flag) {
		if (i2c_read(BAT_MADC, BAT_CTRL_SW1, 1, &bit, 1) != 0)
			goto exit;
		flag = bit & 0x01;
	}
	if (i2c_read(BAT_MADC, BAT_GPCH0_LSB, 1, &LSB, 1) != 0)
		goto exit;
	if (i2c_read(BAT_MADC, BAT_GPCH0_MSB, 1, &MSB, 1) != 0)
		goto exit;

	result |= MSB;
	result <<= 2;
	LSB >>= 6;
	LSB &= 0x03;
	result += LSB;
	volt = ((150000 * result) / 1024) * 10;

	if ((volt >= BAT_ID1_REF_VOLT
			- (BAT_ID1_REF_VOLT / 100 * BAT_DEVIATION)) && (volt
			<= BAT_ID1_REF_VOLT + (BAT_ID1_REF_VOLT / 100
					* BAT_DEVIATION)))
		return BAT_ID1;
	else if ((volt >= BAT_ID2_REF_VOLT - (BAT_ID2_REF_VOLT / 100
			* BAT_DEVIATION)) && (volt <= BAT_ID2_REF_VOLT
			+ (BAT_ID2_REF_VOLT / 100 * BAT_DEVIATION)))
		return BAT_ID2;
	else if ((volt >= BAT_ID3_REF_VOLT - (BAT_ID3_REF_VOLT / 100
			* BAT_DEVIATION)) && (volt <= BAT_ID3_REF_VOLT
			+ (BAT_ID3_REF_VOLT / 100 * BAT_DEVIATION)))
		return BAT_ID3;
	else
		return BAT_UNKNOWN;

exit: return -1;
}
