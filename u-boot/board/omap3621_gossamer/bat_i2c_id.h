/*
 * bat_i2c_id.h
 *
 *  Created on: Feb 10, 2011
 *      Author: x0151843
 *	Description: contains macros and function declarations
 *	for reading MADC and determining the actual battery id
 */
#ifndef BAT_I2C_ID_H
#define BAT_I2C_ID_H
/*
 * Battery reference voltages macros are scaled by 1000000 (actual values in uV
 * in order for the MADC reading to be accurately calculated and compared) and
 * the standard deviation is set by BAT_DEVIATION macro in %
 */
#define BAT_BUS_INDEX		0x00
/*
 * macros for TPS65921 interface and auxiliary
 * registers of interest - prefixed with BAT_ for
 * clarity
 */
#define BAT_INTBR		0x49
#define BAT_MADC		0x4a
#define BAT_GPBR1		0x91
#define BAT_CTRL1		0x00
#define BAT_SW1SELECT_LSB	0x06
#define BAT_CTRL_SW1		0x12
#define BAT_GPCH0_LSB		0x37
#define BAT_GPCH0_MSB		0x38
/*macros for desired reference voltages identifying each battery id*/
#define BAT_ID1_REF_VOLT	346000
#define BAT_ID2_REF_VOLT	750000
#define BAT_ID3_REF_VOLT	1236000
#define BAT_DEVIATION		10

enum {
	BAT_ID1 = 0, BAT_ID2 = 1, BAT_ID3 = 2, BAT_UNKNOWN = 3
};

/*
 * set_bus is used for setting i2c bus channel and speed
 */
int set_bus(uchar bus_ix, uchar bus_sp);

/*
 * do_bat_i2c_write wraps the standard i2c_write method
 */
static inline int do_bat_i2c_write(uchar chip, uchar addr, uchar buffer)
{
	return i2c_write(chip, addr, 1, &buffer, 1);
}

/*
 * get_battery_id executes a series of commands related to MADC and
 * determines the estimated battery id - returns -1 in case of error
 */
int get_battery_id(void);

#endif /*BAT_I2C_ID_H*/
