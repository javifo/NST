/*
 * board/omap3621_boxer/tps65921_keypad.c
 *
 * Copyright (C) 2010 Barnes & Noble, Inc.
 * Intrinsyc Software International, Inc. on behalf of Barnes & Noble, Inc.
 *
 * Keypad read for u-boot
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

#include <common.h>
#include <i2c.h>
#include <tps65921.h>
#include <tps65921_keypad.h>

#define MAX_ROW 4

/********************************************
 * Functions to read and write from tps65921
 ********************************************/
static inline int tps65921_i2c_write_u8(u8 chip_no, u8 val, u8 reg)
{
	return tps65921_i2c_write(chip_no, reg + 0xD2, &val);
}

static inline int tps65921_i2c_read_u8(u8 chip_no, u8 *val, u8 reg)
{
	int rc = tps65921_i2c_read(chip_no, reg + 0xD2, val);
	//printf("read %d\n", rc);
	return rc;
}

/**************
 * Keypad Init
 **************/
int tps65921_keypad_init(void)
{
	int ret = 0;
	u8 ctrl;
	ret = tps65921_i2c_read_u8(TPS65921_CHIP_KEYPAD, &ctrl, TPS65921_KEYP_CTRL);
	if (!ret) {
		ctrl |= TPS65921_KEYP_CTRL_KBD_ON | TPS65921_KEYP_CTRL_SOFT_NRST;
		ctrl &= ~TPS65921_KEYP_CTRL_SOFTMODEN;
		ret = tps65921_i2c_write_u8(TPS65921_CHIP_KEYPAD, ctrl, TPS65921_KEYP_CTRL);
	}
	return ret;
}

/***************
 * Keypad Reset
 ***************/
/*
int tps65921_keypad_reset(void)
{
	int ret = 0;
	u8 ctrl;
	ret = tps65921_i2c_read_u8(TPS65921_CHIP_KEYPAD, &ctrl, TPS65921_KEYP_CTRL);
	if (!ret) {
		ctrl &= ~TPS65921_KEYP_CTRL_SOFT_NRST;
		ret = tps65921_i2c_write_u8(TPS65921_CHIP_KEYPAD, ctrl, TPS65921_KEYP_CTRL);
	}
	return ret;
}
*/

/**************************************************************
 * Read the keypad and return the number of keys pressed while
 * setting the corresponding bit in the (key) argument
 **************************************************************/
int tps65921_keypad_keys_pressed(unsigned int *key)
{
	int count = 0;
	u8 col, row;

	*key = 0;
	u8 ridx;

	// All keys triggered by the same col (0)
	col = ~(1) & 0xff;
	tps65921_i2c_write_u8(TPS65921_CHIP_KEYPAD, col, TPS65921_KEYP_KBC);
	tps65921_i2c_read_u8(TPS65921_CHIP_KEYPAD, &row, TPS65921_KEYP_KBR);
	// All keys on their own row
	for (ridx = 0; ridx < MAX_ROW; ridx++) {
		if (!(row & (1 << ridx))) {
			switch (ridx)
			{
				case 0: *key |= MENU_KEY;
						count++;
						break;
				case 1: *key |= BACK_KEY;
						count++;
						break;
				case 2: *key |= PGFORWARD_KEY;
						count++;
						break;
				case 3: *key |= PGBACK_KEY;
						count++;
						break;
				default:
						break;
			}
		}
	}
	return count;
}

/*
#include <command.h>
int do_keypad(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int keys;
	tps65921_keypad_keys_pressed(&keys);
	printf("key 0x%x\n", keys);
        return 0;
}
U_BOOT_CMD(
        k,  1,      0,      do_keypad,
        "k   - read keypad\n",
        "    - read keypad\n"
);
*/
