/*
 * cmd_bat_i2c_id.c
 *
 *  Created on: Feb 10, 2011
 *      Author: x0151843
 */
#include <common.h>
#include <command.h>
#include <i2c.h>
#include <asm/byteorder.h>
#include <asm/mach-types.h>
#include <fat.h>
#include <mmc.h>

#include "bat_i2c_id.h"
#include "gg_command_parser.h"
#include "gg_update.h"

#define SCRIPT_MEM_ADR		(char *)0x80000000
#define MMC_DEVICE		"mmc"
#define MMC_DEVICE_NUM		0
#define MMC_DEVICE_PART		1

block_dev_desc_t *get_dev(char *ifname, int dev);

/*
 * Executes a test sequence reading the battery id
 * Syntax:
 * 	ibatck
 */
int do_bat_ck(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int res = 0;
	printf("Starting reading battery id.\n");
	if (argc > 1) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	res = get_battery_id();
	if (res == -1) {
		printf("Internal battery check error.\n");
		return 1;
	} else {
		printf("Battery id is %d\n", res);
	}

	return 0;
}

int gauge_flash(char *adr, int size, int bus, int bus_speed)
{
	int res;
	printf("Starting firmware update...\n");
	printf("Script at %p, size: %d, bus: %x, bus_speed %x\n", adr, size,
			bus, bus_speed);

	res = gauge_update_firmware(adr, size, bus, bus_speed);
	if (res == 0)
		printf("Success firmware update.\n");
	else
		printf("Firmware update fail with error: %d\n", res);

	return res;
}

int do_ggflash(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	char *adr;
	int size;
	int res;
	int bus, bus_speed;
	block_dev_desc_t *dev_desc = NULL;

	if (argc != 2 && argc != 4) {
		printf("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}
	res = mmc_init(MMC_DEVICE_NUM);

	if (res) {
		printf("No MMC card found!\n");
		return 1;
	}

	adr = SCRIPT_MEM_ADR;
	dev_desc = get_dev(MMC_DEVICE, MMC_DEVICE_NUM);

	if (fat_register_device(dev_desc, MMC_DEVICE_PART) != 0) {
		printf("\n** Unable to use  %s %d:%d for ggflash **\n",
				MMC_DEVICE, MMC_DEVICE_NUM, MMC_DEVICE_PART);
		return 1;
	}

	size = file_fat_read(argv[1], adr, 0);

	if (size == -1) {
		printf("\n** Unable to read \"%s\" from %s %d:%d **\n",
				argv[1], MMC_DEVICE, MMC_DEVICE_NUM,
				MMC_DEVICE_PART);
		return 1;
	}

	if (argc == 4) {
		bus = simple_strtoul(argv[2], NULL, 16);
		bus_speed = simple_strtoul(argv[3], NULL, 16);
	} else {
		bus = BQ27500_BUS_ID;
		bus_speed = BQ27500_BUS_SPEED;
	}
	res = gauge_flash(adr, size, bus, bus_speed);

	return 0;
}

U_BOOT_CMD(
	ggflash, 4, 1, do_ggflash,
	"ggflash - flash bq27500 from .dffs script\n",
	"filename [bus_index] [speed]\n\
filename (.dffs/.bqfs) must be on mmc\n\
bus index and the speed (0x64(ST),0x190(FS),0xD48(HS)) are optional\n"
);

U_BOOT_CMD(
		ibatck, 1, 1, do_bat_ck,
		"ibatck  - used to track battery id\n",
		"no additional arguments are required\n"
);
