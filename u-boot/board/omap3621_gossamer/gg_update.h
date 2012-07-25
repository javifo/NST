/*
 * gg_update.h
 *
 *  Created on: Feb 3, 2011
 *      Author: x0151841
 */

#ifndef GG_UPDATE_H_
#define GG_UPDATE_H_

#define BQ27500_ID		0x55
#define BQ27500_ID_ROM		0x0B
#define BQ27500_BUS_ID		0x01
#define BQ27500_BUS_SPEED	0x64

#define BQ27500_DELAY_MODE	250

int gauge_update_firmware(char *buff, int len, const int bus_id,
						  const int bus_speed);
int gauge_single_command(char tag, unsigned char *buff, unsigned int buff_size);

#define I2C_WRITE_ERROR		-10
#define I2C_READ_ERROR		-11
#define GG_CMP_FAIL		-12
#define GG_WRONG_MODE		-13
#define GG_ENTER_ROM_FAIL	-14
#define GG_ENTER_BUS_FAIL	-15
#define GG_CHIP_NOT_FOUND	-16
#define	GG_WRONG_COMMAND	-17

#endif /* GG_UPDATE_H_ */

