/*
 * gg_update.c
 *
 *  Created on: Feb 3, 2011
 *      Author: x0151841
 */

#include <malloc.h>
#include <common.h>
#include <i2c.h>

#include "gg_update.h"
#include "gg_def.h"
#include "gg_command_parser.h"

extern int select_bus(int, int);

static unsigned char BQ27500_ENTER_ROM_CMD[] = { 0x00, 0x00, 0x0F };
static unsigned char BQ27500_CHECK_MODE[] = { 0x00, 0x00, 0x00 };

static int gauge_update_do_write(const unsigned char *buff,
		unsigned int buff_size)
{
	unsigned char chip = ((unsigned char *) buff)[0] / 2;
	unsigned int addr = ((unsigned char *) buff)[1];
	unsigned char *buffer = ((unsigned char *) buff) + 2;
	int len = buff_size - 2;

	if (i2c_write(chip, addr, 1, buffer, len) != 0)
		return I2C_WRITE_ERROR;
	return 0;
}

static int gauge_update_do_cmp(const unsigned char *buff,
		unsigned int buff_size)
{
	unsigned char chip = ((unsigned char *) buff)[0] / 2;
	unsigned int addr = ((unsigned char *) buff)[1];
	unsigned char *buffer = ((unsigned char *) buff) + 2;
	int len = buff_size - 2;
	unsigned char *tmp = (unsigned char *) malloc(len);

	memset(tmp, 0, len);

	if (i2c_read(chip, addr, 1, tmp, len) != 0) {
		free(tmp);
		return I2C_READ_ERROR;
	}

	if (memcmp(buffer, tmp, len) != 0) {
		free(tmp);
		return GG_CMP_FAIL;
	}
	free(tmp);
	return 0;
}

static int gauge_update_do_delay(const unsigned char *buff,
		unsigned int buff_size)
{
	long int delay;
	char *strend;
	strend = (char *) buff + buff_size;
	delay = simple_strtol((char *) buff, &strend, 10)+1;
	udelay(delay * 1000);
	return 0;
}

int gauge_single_command(char tag, unsigned char *buff, unsigned int buff_size)
{
	int res;

	switch (tag) {
	case 'W':
		res = gauge_update_do_write(buff, buff_size);
        udelay (800);
		CHECK_ERROR_EXIT(res != 0);
		break;
	case 'C':
		res = gauge_update_do_cmp(buff, buff_size);
        udelay (800);
		CHECK_ERROR_EXIT(res != 0);
		break;
	case 'X':
		res = gauge_update_do_delay(buff, buff_size);
		CHECK_ERROR_EXIT(res != 0);
		break;
	default:
		res = GG_WRONG_COMMAND;
		CHECK_ERROR_EXIT(res != 0);
	}

	EXIT_WITH_ERROR;
	return res;
}

static int gauge_check_mode(unsigned char chip)
{
	int res;
	unsigned char tmp;
	res = i2c_write(chip, BQ27500_CHECK_MODE[0], 1, BQ27500_CHECK_MODE + 1,
			sizeof(BQ27500_CHECK_MODE) - 1);
	CHECK_ERROR_EXIT(res != 0);
	udelay(20 * 1000 * 5);
	res = i2c_read(chip, 0x01, 1, &tmp, 1);
	CHECK_ERROR_EXIT(res != 0);
	if (tmp & 0x30) {
		res = -1;
		printf("chip is sealed %02X\n", tmp & 0x30);
	}
	EXIT_WITH_ERROR;
	if (res != 0)
		res = GG_WRONG_MODE;

	return res;
}

static int gauge_enter_rom_mode(unsigned char chip)
{
	int res;

	res = i2c_write(chip, BQ27500_ENTER_ROM_CMD[0], 1,
			BQ27500_ENTER_ROM_CMD + 1,
			sizeof(BQ27500_ENTER_ROM_CMD) - 1);

	CHECK_ERROR_EXIT(res != 0);
	udelay(BQ27500_DELAY_MODE * 1000 * 5);

	EXIT_WITH_ERROR;
	if (res != 0)
		res = GG_ENTER_ROM_FAIL;

	return res;
}

int gauge_update_firmware(char *buff, int len, const int bus_id,
		const int bus_speed)
{
	int res;

	res = select_bus(bus_id, bus_speed);
	if (res != 0)
		res = GG_ENTER_BUS_FAIL;
	CHECK_ERROR_EXIT(res != 0);

	res = i2c_probe(BQ27500_ID);
	if (res == 0) {
		res = gauge_check_mode(BQ27500_ID);
		CHECK_ERROR_EXIT(res != 0);

		res = gauge_enter_rom_mode(BQ27500_ID);
		CHECK_ERROR_EXIT(res != 0);
	} else {
		res = i2c_probe(BQ27500_ID_ROM);
		if (res != 0)
			res = GG_CHIP_NOT_FOUND;
		CHECK_ERROR_EXIT(res != 0);
		printf("Warning chip found in ROM mode!\n");
	}

	res = script_parser(buff, len);
	CHECK_ERROR_EXIT(res != 0);

	EXIT_WITH_ERROR;
	return res;
}

