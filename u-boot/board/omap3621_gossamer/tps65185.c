#include <common.h>
#include <asm/io.h>
#include <asm/arch/mem.h>	/* get mem tables */
#include <asm/arch/sys_proto.h>
#include <asm/arch/mux.h>
#include "tps65185.h"
#include <i2c.h>

/* Copied from common/power/gpio.h */
typedef enum {
    GPIO_OUTPUT = 0,
    GPIO_INPUT  = 1
} gpio_dir_t;

typedef enum {
    GPIO_LOW  = 0,
    GPIO_HIGH = 1
} gpio_level_t;

extern int gpio_pin_init(u32, gpio_dir_t, gpio_level_t);
extern int gpio_pin_write(u32, gpio_level_t);

extern int select_bus(int, int);

#define PAPYRUS2_1P0_BASE_ADDR			0x48
#define PAPYRUS2_1P1_BASE_ADDR			0x68

#define PAPYRUS2_REVID_REG			0x10
#define PAPYRUS2_EEPROM18_REG		0x32
#define PAPYRUS2_EEPROM_UNLOCK_REG	0x40
#define PAPYRUS2_EEPROM_PRG_REG		0x41
#define PAPYRUS2_VTEST_REG			0x48

#define PAPYRUS2_VERSION_MASK 0x0F
#define PAPYRUS2_VERSION      0x05
#define PAPYRUS2_UNLOCK_KEY   0xAA
#define PAPYRUS2_LOCK_KEY     0x00

#define VZERO_TEST_MASK		  0x80
#define EEPROM_PROG_DELAY_US  600000
#define EEPROM_PROG_MASK      0x10

#define PAPYRUS2_PATCH_ERR    -1
#define VDDH_TRIM_VAL_MASK    0x18

#define PAPYRUS2_I2C_BUS 1
#define PAPYRUS2_WAKEUP_GPIO 87
#define PAPYRUS2_WAKEUP_DELAY_US 10000

static inline int i2c_write_u8(u8 chip_no, u8 val, u8 reg)
{
	return i2c_write(chip_no, reg, 1, &val, 1);
}

static inline int i2c_read_u8(u8 chip_no, u8 *val, u8 reg)
{
	int retval = i2c_read(chip_no, reg, 1, val, 1);
	return retval;
}

int papyrus2_patch(papyrus_version_t papyrus2_version)
{
	u8 read_val, version;
	int err_value = 0;
	int papyrus2_base_addr;

	if (papyrus2_version == PAPYRUS2_1P0) {
		papyrus2_base_addr = PAPYRUS2_1P0_BASE_ADDR;
	}
	else {
		papyrus2_base_addr = PAPYRUS2_1P1_BASE_ADDR;
	}

	if	(select_bus(PAPYRUS2_I2C_BUS, CFG_I2C_SPEED)) {
		err_value = PAPYRUS2_PATCH_ERR;
		goto papyrus2_patch_err;
	}

	gpio_pin_init(PAPYRUS2_WAKEUP_GPIO, GPIO_OUTPUT, GPIO_LOW);
	gpio_pin_write(PAPYRUS2_WAKEUP_GPIO, 1);
	udelay(PAPYRUS2_WAKEUP_DELAY_US);

	if (!i2c_read_u8(papyrus2_base_addr, &version, PAPYRUS2_REVID_REG)) {
		if ((version & PAPYRUS2_VERSION_MASK) != PAPYRUS2_VERSION) {
			printf ("Papyrus does not seem to be a tps65185 (version ID: 0x%02x)\n", version);
			err_value = PAPYRUS2_PATCH_ERR;
			goto papyrus2_patch_err;
		}
	} else {
		err_value = PAPYRUS2_PATCH_ERR;
		goto papyrus2_patch_err;
	}

	if (i2c_write_u8(papyrus2_base_addr, PAPYRUS2_UNLOCK_KEY,    PAPYRUS2_EEPROM_UNLOCK_REG)) {
		err_value = PAPYRUS2_PATCH_ERR;
		goto papyrus2_patch_err;
	}

	if (!i2c_read_u8(papyrus2_base_addr, &read_val, PAPYRUS2_EEPROM18_REG)) {
		if ((read_val & VDDH_TRIM_VAL_MASK) == VDDH_TRIM_VAL_MASK) {
			printf ("Papyrus VDDH ILIM already set to the max current. We're all set.\n");
			err_value = 0;
			goto papyrus2_patch_err;
		} else {
			printf ("Setting VDDH ILIM EEPROM to maximum current: ");
			read_val |= VDDH_TRIM_VAL_MASK;
			if (i2c_write_u8(papyrus2_base_addr, read_val, PAPYRUS2_EEPROM18_REG)) {
				printf("KO\n");
				err_value = PAPYRUS2_PATCH_ERR;
				goto papyrus2_patch_err;
			}
			if (!i2c_read_u8(papyrus2_base_addr, &read_val, PAPYRUS2_VTEST_REG)) {
				read_val |= VZERO_TEST_MASK;
				if (i2c_write_u8(papyrus2_base_addr, read_val, PAPYRUS2_VTEST_REG)) {
					printf("KO\n");
					err_value = PAPYRUS2_PATCH_ERR;
					goto papyrus2_patch_err;
				}
			} else {
				printf("KO\n");
				err_value = PAPYRUS2_PATCH_ERR;
				goto papyrus2_patch_err;
			}
			if (!i2c_read_u8(papyrus2_base_addr, &read_val, PAPYRUS2_EEPROM_PRG_REG)) {
				read_val |= EEPROM_PROG_MASK;
				if (i2c_write_u8(papyrus2_base_addr, read_val, PAPYRUS2_EEPROM_PRG_REG)) {
					printf("KO\n");
					err_value = PAPYRUS2_PATCH_ERR;
					goto papyrus2_patch_err;
				}
				udelay(EEPROM_PROG_DELAY_US);
				read_val &= ~EEPROM_PROG_MASK;
				if (i2c_write_u8(papyrus2_base_addr, read_val, PAPYRUS2_EEPROM_PRG_REG)) {
					printf("KO\n");
					err_value = PAPYRUS2_PATCH_ERR;
					goto papyrus2_patch_err;
				}
			}	
		}
	} else {
		printf ("Unable to read Papyrus VDDH ILIM value\n");
		err_value = PAPYRUS2_PATCH_ERR;
	}
	
	if (i2c_write_u8(papyrus2_base_addr, PAPYRUS2_LOCK_KEY,    PAPYRUS2_EEPROM_UNLOCK_REG)) {
		printf("KO\n");
		err_value = PAPYRUS2_PATCH_ERR;
	} else {
		printf("OK\n");
	}

papyrus2_patch_err:
	gpio_pin_write(PAPYRUS2_WAKEUP_GPIO, 0);
	return err_value;
}

