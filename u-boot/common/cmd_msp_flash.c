/*
 * cmd_msp_flash.c
 *
 *  Created on: Feb 16, 2011
 *       Authors: x0151846 and x0151843
 */

#include <common.h>
#include <command.h>
#include <serial.h>
#include <devices.h>
#include <exports.h>
#include <asm/io.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>
#include <asm/arch/mem.h>
#include <asm/mach-types.h>
#include <i2c.h>
#include <mmc.h>
#include <fat.h>
#include <linux/mtd/nand_ecc.h>
#include <ns16550.h>
#include <configs/omap3621_gossamer.h>
#include <linux/ctype.h>

/* gpio handling .h files in use*/
#include "../common/power/general.h"
#include "../common/power/gpio.h"
#include "../common/power/omap_cm_regs.h"
#include "../common/power/omap_gpio_regs.h"

#include "msp_flash.h"
extern block_dev_desc_t *get_dev (char* ifname, int dev);

/* msp default firmware password */
static unsigned char MSP_DEF_PASS[MSP_OPT_PASS_LEN] = "30FE30FE00FE04FE30FE08FE0CFE10FE14FE18FE1CFE30FE20FE24FE28FE2CFE";

/* initializes serial port connected to msp
 * the function does not expose any success status checks
 * and must be called at the beginning of msp communication
 */
void msp_serial_init(NS16550_t com_port, unsigned int baudrate)
{

	int baud_divisor = (CFG_NS16550_CLK / MODE_X_DIV / baudrate);
	com_port->ier = 0x00;
#if defined(CONFIG_OMAP) && !defined(CONFIG_3430ZOOM2)
	com_port->mdr1 = 0x7;
#endif

#if defined(CONFIG_3430ZOOM2)
	/* On Zoom2 board Set pre-scalar to 1
	 * CLKSEL is GND => MCR[7] is 1 => preslr is 4
	 * So change the prescl to 1
	 */
	com_port->lcr = 0xBF;
	com_port->fcr |= 0x10;
	com_port->mcr &= 0x7F;
#endif
	com_port->lcr = LCR_BKSE | LCR_8E1;
	com_port->dll = baud_divisor & 0xff;
	com_port->dlm = (baud_divisor >> 8) & 0xff;
	com_port->lcr = LCR_8E1;
	com_port->mcr = MSP_MCRVAL;
	com_port->fcr = MSP_FCRVAL;

#if defined(CONFIG_OMAP)
#if defined(CONFIG_APTIX)
	/* /13 mode so Aptix 6MHz can hit 115200 */
	com_port->mdr1 = 3;
#else
	/* /16 is proper to hit 115200 with 48MHz */
	com_port->mdr1 = 0;
#endif
#endif
	printf("MSP serial port initialized!\n");
}

/* wrapping functions for manipulating msp uart gpios */
void msp_set_reset_pin(int level)
{
	gpio_pin_write(MSP_MCBSP3_DR, level);
}

void msp_set_test_pin(int level)
{
	gpio_pin_write(MSP_MCBSP3_DX, level);
}

/* this function sets msp either in bootstrap loader mode or application
 * mode;
 * all delay arguments are adjusted experimentally to suit proper
 * msp initialization
 */
void msp_bsl_reset(int invoke_bsl)
{
	/* MSP430 Power enable - GPIO_37 */
	gpio_pin_init(37, GPIO_OUTPUT, GPIO_HIGH);
	gpio_pin_write(37, 1);
	udelay(250000);

	gpio_pin_init(MSP_MCBSP3_DX, GPIO_OUTPUT, 0);
	gpio_pin_init(MSP_MCBSP3_DR, GPIO_OUTPUT, 0);

	if (invoke_bsl) { /* starts BSL mode */
		msp_set_reset_pin(0);
		msp_set_test_pin(0);
		udelay(50000);

		msp_set_reset_pin(0);
		msp_set_test_pin(1);
		udelay(10000);

		msp_set_reset_pin(0);
		msp_set_test_pin(0);
		udelay(10000);

		msp_set_reset_pin(0);
		msp_set_test_pin(0);
		udelay(10000);

		msp_set_reset_pin(0);
		msp_set_test_pin(1);
		udelay(10000);

		msp_set_reset_pin(1);
		msp_set_test_pin(1);
		udelay(10000);

		msp_set_reset_pin(1);
		msp_set_test_pin(0);
		udelay(10000);
		printf("BSL mode activated!\n");

	} else { /* starts application mode */
		msp_set_reset_pin(0);
		msp_set_test_pin(0);
		udelay(50000);

		msp_set_reset_pin(1);
		msp_set_test_pin(0);
		udelay(10000);

		msp_set_reset_pin(0);
		msp_set_test_pin(0);
		udelay(10000);

		gpio_pin_write(37, 0);
		printf("User mode activated!\n");
	}
	udelay(250000);
}

/* msp command sync function*/
int msp_bsl_sync()
{
	int i;
	const int sync_retry = 3; /* max tries to get synchronization */
	unsigned char rx_message;

	for (i = 0; i < sync_retry; i++) {

		while (_serial_tstc(UART2))
			_serial_getc(UART2);

		rs_write_byte(MSP_SYNC, (NS16550_t) CFG_NS16550_COM2);

		if (rs_read_byte(&rx_message) == -1) {
			printf("\nMSP sync timed out!\n");
			return 0;
		}
		if (rx_message == MSP_DATA_ACK) {
			return 1;
		}
	}
	if (rx_message == MSP_DATA_NAK) {
		printf("MSP sync failed!\n");
		return -1;
	}
	return 0;
}

/* helping function for getting a byte through msp uart*/
int rs_read_byte(unsigned char* rx_byte)
{
	int count = 0;
	while (count < MSP_READ_RETRY) {
		udelay(MSP_RW_DELAY);
		if (_serial_tstc(UART2)) {
			*rx_byte = _serial_getc(UART2);
			return 0;
		}
		count++;
	}
	return -1;
}

/* used for writing a single data byte to msp uart */
static inline void rs_write_byte(unsigned char tx_byte, NS16550_t com_port)
{
	_serial_putc_raw(tx_byte, UART2);
	while (com_port->lsr & 0x40)
		;
}

/* used for validating msp command checksum */
void msp_cmd_chksum(unsigned char* buffer, int length,
		unsigned char *chk_sum_l, unsigned char *chk_sum_h)
{
	int i;
	*chk_sum_l = 0;
	*chk_sum_h = 0;

	for (i = 0; i < length; i = i + 2) {
		*chk_sum_l = *chk_sum_l ^ buffer[i];
		*chk_sum_h = *chk_sum_h ^ buffer[i + 1];
	}
	*chk_sum_l = ~*chk_sum_l;
	*chk_sum_h = ~*chk_sum_h;
}

/* obtaining and validating BSL Version */
int msp_read_bsl_version()
{
	unsigned char ckl, ckh;
	unsigned char buff;
	int i;
	int header_length = 8;
	unsigned char tx_message[10];
	unsigned char rx_message[22];
	int res = 0;

	if (msp_bsl_sync() != 1) {
		printf("MSP read bsl version sync failed!\n");
		return -1;
	}

	tx_message[0] = MSP_HDR;
	tx_message[1] = MSP_TX_BSL_VERSION;
	tx_message[2] = 0x04;
	tx_message[3] = 0x04;
	tx_message[4] = 0x01;
	tx_message[5] = 0x01;
	tx_message[6] = 0x01;
	tx_message[7] = 0x01;

	msp_cmd_chksum(tx_message, header_length, &ckl, &ckh);

	tx_message[header_length] = ckl;
	tx_message[header_length + 1] = ckh;

	for (i = 0; i < header_length + 2; i++)
		rs_write_byte(tx_message[i], (NS16550_t) CFG_NS16550_COM2);
	i = 0;

	while (i < BSL_RESPONSE_LEN && res != -1) {
		res = rs_read_byte(&buff);
		rx_message[i] = buff;
		i++;
	}
	if (res == -1 && i <= BSL_RESPONSE_LEN) {
		printf("BSL version response failed!\n");
		return -1;
	}

	msp_cmd_chksum(rx_message, BSL_RESPONSE_LEN - 2, &ckl, &ckh);
	if ((rx_message[BSL_RESPONSE_LEN - 2] != ckl)
			|| (rx_message[BSL_RESPONSE_LEN - 1] != ckh)) {
		printf("\nIncorrect BSL version response checksum!\n");
		return -1;
	}

	printf("Device family high byte is: %02X\n", rx_message[4]);
	printf("Device family low byte is: %02X\n", rx_message[5]);
	printf("BSL version high byte is: %02X\n", rx_message[15]);
	printf("BSL version low byte is: %02X\n", rx_message[16]);

	return 0;
}

/* function for sending a 32-byte password to msp and querying a response*/
int msp_send_pass(unsigned char* password)
{
	unsigned char ckl, ckh;
	unsigned char buff;
	unsigned char tx_message[42];
	int i;
	int header_length = 8;

	if (msp_bsl_sync() != 1) {
		printf("MSP send pass sync failed!\n");
		return -1;
	}

	tx_message[0] = MSP_HDR;
	tx_message[1] = MSP_RX_PASSWORD;
	tx_message[2] = 0x24;
	tx_message[3] = 0x24;
	tx_message[4] = 0x00;
	tx_message[5] = 0x00;
	tx_message[6] = 0x00;
	tx_message[7] = 0x00;

	for (i = 0; i < MSP_PASS_LEN; i++)
		tx_message[header_length + i] = *(password + i);

	msp_cmd_chksum(tx_message, 40, &ckl, &ckh);

	tx_message[40] = ckl;
	tx_message[41] = ckh;

	for (i = 0; i < MSP_PASS_LEN + 10; i++)
		rs_write_byte(tx_message[i], (NS16550_t) CFG_NS16550_COM2);

	if (rs_read_byte(&buff) == -1) {
		printf("Password query timed out!\n");
		return -1;
	}
	if (buff == MSP_DATA_ACK)
		printf("MSP password accepted!\n");
	else {
		printf("MSP password denied!\n");
		return -1;
	}

	return 0;
}

/* function for performing main memory erase only;
 * the function is password protected */
int msp_erase_main()
{
	unsigned char ckl, ckh;
	unsigned char buff;
	int header_length = 8;
	unsigned char tx_message[10];
	int i;

	if (msp_bsl_sync() != 1) {
		printf("MSP erase main sync failed!\n");
		return -1;
	}

	tx_message[0] = MSP_HDR;
	tx_message[1] = MSP_ERASE_MAIN;
	tx_message[2] = 0x04;
	tx_message[3] = 0x04;
	tx_message[4] = 0x00;
	tx_message[5] = 0x80;
	tx_message[6] = 0x04;
	tx_message[7] = 0xA5;

	msp_cmd_chksum(tx_message, header_length, &ckl, &ckh);

	tx_message[8] = ckl;
	tx_message[9] = ckh;

	for (i = 0; i < 10; i++)
		rs_write_byte(tx_message[i], (NS16550_t) CFG_NS16550_COM2);

	udelay(500000);

	if (rs_read_byte(&buff) == -1) {
		printf("MSP main memory erase read response error!\n");
		return -1;
	}

	if (buff == MSP_DATA_ACK)
		printf("MSP main memory erase executed!\n");
	else {
		printf("MSP main memory erase failed!\n");
		return -1;
	}

	return 0;
}
/* function for performing complete msp flash mass erase - it is not
 * password protected */
int msp_mass_erase()
{
	unsigned char ckl, ckh;
	int i;
	unsigned char buff;
	unsigned char tx_message[10];

	if(msp_bsl_sync() != 1) {
		printf("MSP mass erase sync failed!\n");
		return -1;
	}

	tx_message[0] = MSP_HDR;
	tx_message[1] = MSP_MASS_ERASE;
	tx_message[2] = 0x04;
	tx_message[3] = 0x04;
	tx_message[4] = 0x00;
	tx_message[5] = 0x00;
	tx_message[6] = 0x06;
	tx_message[7] = 0xA5;

	msp_cmd_chksum(tx_message, 8, &ckl, &ckh);

	tx_message[8] = ckl;
	tx_message[9] = ckh;

	for (i = 0; i < 10; i++)
		rs_write_byte(tx_message[i], (NS16550_t) CFG_NS16550_COM2);

	if(rs_read_byte(&buff) == -1) {
		printf("MSP mass erase timed out!\n");
		return -1;
	}

	if (buff == MSP_DATA_ACK)
		printf("MSP mass erase executed!\n");
	else {
		printf("MSP mass erase failed!\n");
		return -1;
	}

	return 0;
}

/* function for writing a block of data to msp flash */
int msp_send_block(unsigned char *tx_data, unsigned int addr,
		unsigned int length)
{
	unsigned char ckl, ckh;
	int i;
	unsigned char buff;
	int header_length = 8;
	unsigned char tx_message[MSP_ROW_MAXDLEN + 10];

	/* align to an even start address*/
	if ((addr % 2) != 0)
		addr--;
	if (length <= 0 || length > MSP_ROW_MAXDLEN) {
		printf("Incorrect block data length!\n");
		return -1;
	}
	/* determine if write to control registers is allowed */
	if (addr < MSP_INFO_START_ADDR || addr >= MSP_MAIN_END_ADDR || (addr
			>= MSP_INFO_END_ADDR && addr < MSP_MAIN_START_ADDR)) {
		printf("Incorrect address!\n");
		return -1;
	}
	if (((addr + length - 1) > MSP_INFO_END_ADDR && (addr + length - 1)
			< MSP_MAIN_START_ADDR) || (addr + length - 1)
			> MSP_MAIN_END_ADDR) {
		printf("Incorrect data positioning!\n");
		return -1;
	}

	if(msp_bsl_sync() != 1) {
		printf("MSP send block sync failed!\n");
		return -1;
	}
	tx_message[0] = MSP_HDR;
	tx_message[1] = MSP_RX_DATA_BLOCK;
	tx_message[2] = length + 4;
	tx_message[3] = length + 4;
	tx_message[4] = (unsigned char) (addr & 0x00FF);
	tx_message[5] = (unsigned char) ((addr & 0xFF00) >> 8);
	tx_message[6] = (unsigned char) length;
	tx_message[7] = 0x00;

	for (i = 0; i < length; i++)
		tx_message[header_length + i] = tx_data[i];
	/*data padding is with 0xFF in case of uneven data length*/
	if ((length % 2) != 0) {
		length++;
		tx_message[6] = length;
		tx_message[header_length + length - 1] = 0xFF;
	}

	if (((addr + length - 1) > MSP_INFO_END_ADDR && (addr + length - 1)
			< MSP_MAIN_START_ADDR) || (addr + length - 1)
			> MSP_MAIN_END_ADDR) {
		printf("Incorrect block data positioning!\n");
		return -1;
	}

	msp_cmd_chksum(tx_message, header_length + length, &ckl, &ckh);

	tx_message[header_length + length] = ckl;
	tx_message[header_length + length + 1] = ckh;

	for (i = 0; i < header_length + length + 2; i++)
		rs_write_byte(tx_message[i], (NS16550_t) CFG_NS16550_COM2);

	if (rs_read_byte(&buff) == -1) {
		printf("Write block ACK timed out!\n");
		return -1;
	}
	if (buff == MSP_DATA_NAK) {
		printf("MSP write block failed!\n");
		return -1;
	}

	return 0;
}

/* function for reading a data block from msp flash;
 * actual length of container rx_message should be with 6 bytes more than desired
 * length because of additional command response data (consult msp documentation)*/
int msp_read_block(unsigned char *rx_message, unsigned int addr,
		unsigned int length)
{
	unsigned char ckl, ckh;
	int i;
	int header_length = 8;
	unsigned char tx_message[10];

	if ((addr % 2) != 0)
		addr--;

	if ((length % 2) != 0) {
		length++;
	}

	if (length <= 0 || length > MSP_ROW_MAXDLEN) {
		printf("Incorrect block data length!\n");
		return -1;
	}

	if (addr < MSP_INFO_START_ADDR || addr >= MSP_MAIN_END_ADDR || (addr
			>= MSP_INFO_END_ADDR && addr < MSP_MAIN_START_ADDR)) {
		printf("Incorrect address!\n");
		return -1;
	}
	if (((addr + length - 1) > MSP_INFO_END_ADDR && (addr + length - 1)
			< MSP_MAIN_START_ADDR) || (addr + length - 1)
			> MSP_MAIN_END_ADDR) {
		printf("Incorrect data positioning!\n");
		return -1;
	}

	if(msp_bsl_sync() != 1) {
		printf("MSP read block sync failed!\n");
		return -1;
	}
	tx_message[0] = MSP_HDR;
	tx_message[1] = MSP_TX_DATA_BLOCK;
	tx_message[2] = 0x04;
	tx_message[3] = 0x04;
	tx_message[4] = (unsigned char) (addr & 0x00FF);
	tx_message[5] = (unsigned char) ((addr & 0xFF00) >> 8);
	tx_message[6] = (unsigned char) length;
	tx_message[7] = 0x00;

	msp_cmd_chksum(tx_message, header_length, &ckl, &ckh);

	tx_message[header_length] = ckl;
	tx_message[header_length + 1] = ckh;

	for (i = 0; i < header_length + 2; i++)
		rs_write_byte(tx_message[i], (NS16550_t) CFG_NS16550_COM2);
	for (i = 0; i < length + 6; i++) {
		if (rs_read_byte(rx_message + i) == -1) {
			printf("MSP flash read timed out!\n");
			return -1;
		}
	}

	return 0;
}

/* function for loading a data file in memory */
int do_msp_file_load(char* fname)
{
	ulong size;
	char buf[12];
	block_dev_desc_t *dev_desc = NULL;
	int ret = -1;

	ret = mmc_init(MSP_DEV_NUM);
	if (ret) {
		printf("No MMC card found!\n");
		return -1;
	} else
		printf("MMC initialization OK!\n");
	dev_desc = get_dev(MSP_DEVICE, MSP_DEV_NUM);

	if (dev_desc == NULL) {
		printf("Invalid boot device!\n");
		return -1;
	}
	if (fat_register_device(dev_desc, MSP_PARTITION) != 0) {
		printf("Unable to use %s %d:%d for fatload!", MSP_DEVICE, 0,
				MSP_PARTITION);
		return -1;
	}

	size = file_fat_read(fname, (unsigned char*) MSP_LOAD_ADDR, 0);

	if (size == -1) {
		printf("Unable to read file data or missing file!\n");
		return -1;
	}
	if (size < 1 || size > MSP_FLASH_SIZE) {
		printf("MSP file size: %ul. Maximum buffer size: %ul\n", size,
				MSP_FLASH_SIZE);
		return -1;
	}

	printf("\n%ld bytes read:\n", size);
	sprintf(buf, "%lX", size);
	setenv("filesize", buf);

	return 0;
}

unsigned char calc_row_cksum(unsigned char* buffer, unsigned int length)
{
	int i = 0;
	unsigned char res = 0;
	for (i = 0; i < length; i++)
		res += buffer[i];
	res = ~res;
	res += 1;
	return res;
}

int is_hex(unsigned char sym)
{
	if ((sym >= 48 && sym <= 57) || (sym >= 65 && sym <= 70))
		return 1;
	else
		return 0;
}

int ascii_to_char(unsigned char* sym, unsigned char* dest)
{
	if ((*sym >= 48 && *sym <= 57)) {
		*dest = *sym - '0';
		return 1;
	}
	if ((*sym >= 65 && *sym <= 70)) {
		*dest = *sym - 'A' + 10;
		return 1;
	}
	return 0;
}

int msp_flash_row(unsigned char* base, unsigned int* index)
{
	unsigned char buffer[MSP_ROW_MAXDLEN] = { };
	unsigned char buff_high = 0x00;
	unsigned char buff_low = 0x00;
	unsigned int data_len = 0;
	unsigned int buff_byte = 0;
	unsigned int addr = 0, tmp = 0;
	unsigned char* pos = base;
	int i = 0;

	pos += 1;
	ascii_to_char(pos, &buff_high);
	pos += 1;
	ascii_to_char(pos, &buff_low);
	buff_byte = (buff_high << 4) + buff_low;
	data_len = buff_byte;
	buffer[0] = buff_byte;

	if (data_len == 0) {
		pos += 1;
		for (i = 0; i < 8; i++, pos += 1) {
			ascii_to_char(pos, &buff_high);
			tmp += buff_high;
			if (i < 7)
				tmp <<= 4;
		}
		if ((tmp ^ MSP_EOF_STR) == 0)
			return 1;
		if (((tmp & 0x0000FF00) >> 8) == 0x01)
			return 1;
		return -1;
	} else {
		pos += 1;
		for (i = 0; i < 4; i++, pos += 1) {
			ascii_to_char(pos, &buff_high);
			addr += buff_high;
			if (i < 3)
				addr <<= 4;
		}
		buffer[1] = (addr & 0xFF00) >> 8;
		buffer[2] = (addr & 0x00FF);

		ascii_to_char(pos, &buff_high);
		pos += 1;
		ascii_to_char(pos, &buff_low);
		pos += 1;
		tmp = (buff_high << 4) + buff_low;
		buffer[3] = tmp;

		switch (tmp) {
		case MSP_DATA:
			for (i = 0; i < data_len; i++, pos += 2) {
				ascii_to_char(pos, &buff_high);
				ascii_to_char(pos + 1, &buff_low);
				buff_byte = (buff_high << 4) + buff_low;
				buffer[i + 4] = buff_byte;
			}
			ascii_to_char(pos, &buff_high);
			ascii_to_char(pos + 1, &buff_low);
			buff_byte = (buff_high << 4) + buff_low;
			pos += 2;

			if (msp_send_block(buffer + 4, addr, data_len) == -1)
				return -1;

			/* checking for correct row ending (either '/n' or '/r/n')*/
			if (*pos == '\r') {
				pos += 2;
			} else if (*pos == '\n')
				pos += 1;
			*index = pos - base;
			break;
		case MSP_EOF:
			return -1;
			break;
		case MSP_EXT:
			return -1;
			break;
		default:
			return -1;
		}
	}

	return 0;
}

/* used to extract password bytes from flash file if any found */
void msp_find_pass(unsigned int addr, unsigned char* buffer,
		unsigned int data_len, unsigned char* password)
{
	unsigned int length = 0;
	unsigned int offset = 0;
	unsigned int store_pos = 0;
	unsigned char* pos;
	int i = 0;

	if (addr == MSP_PASS_START_ADDR) {
		if (data_len > MSP_PASS_LEN)
			return;
		length = data_len;
		pos = buffer + 4;
	} else if ((addr < MSP_PASS_START_ADDR) && (addr + data_len
			> MSP_PASS_START_ADDR)) {
		offset = (MSP_PASS_START_ADDR - addr);
		if (data_len - offset >= MSP_PASS_LEN)
			length = MSP_PASS_LEN;
		else
			length = data_len - offset;
		pos = buffer + 4 + offset;
	} else if (addr > MSP_PASS_START_ADDR && (addr + data_len
			> (MSP_MAIN_END_ADDR + 1))) {
		return;
	} else if (addr > MSP_PASS_START_ADDR && (addr + data_len
			<= (MSP_MAIN_END_ADDR + 1))) {
		store_pos = addr - MSP_PASS_START_ADDR;
		length = data_len;
		pos = buffer + 4;
	} else {
		return;
	}
	for (i = 0; i < length; i++) {
		password[i + store_pos] = *(pos + i);
	}
	return;
}

/* used for per row flash file data validation */
int msp_row_validate(unsigned char* base, unsigned int* index,
		unsigned char* password)
{
	unsigned char buffer[MSP_ROW_MAXDLEN] = { };
	unsigned char buff_high = 0x00;
	unsigned char buff_low = 0x00;
	unsigned int data_len = 0;
	unsigned int buff_byte = 0;
	unsigned int addr = 0, tmp = 0;
	unsigned char* pos = base;
	int i = 0;

	if (*pos != ':')
		return -1;
	pos += 1;
	if (!is_hex(*pos) || !is_hex(*(pos + 1)))
		return -1;
	if (!ascii_to_char(pos, &buff_high))
		return -1;
	pos += 1;
	if (!ascii_to_char(pos, &buff_low))
		return -1;
	buff_byte = (buff_high << 4) + buff_low;
	data_len = buff_byte;
	/*first non-delimiter byte is read*/
	buffer[0] = buff_byte;

	if (data_len == 0) {
		pos += 1;
		for (i = 0; i < 8; i++, pos += 1) {
			if (!is_hex(*pos))
				return -1;
			if (!ascii_to_char(pos, &buff_high))
				return -1;
			tmp += buff_high;
			if (i < 7)
				tmp <<= 4;
		}
		if ((tmp ^ MSP_EOF_STR) == 0)
			/*End of flash file found*/
			return 1;
		if (((tmp & 0x0000FF00) >> 8) == 0x01)
			return 1;
		/*this simplified parser does not accept other cases*/
		return -1;
	} else {
		if ((data_len % 2) != 0) {
			printf(
					"Incorrect file format - data length is uneven!\n");
			return -1;
		}
		if (data_len > 32) {
			printf("Incorrect file format - data length > 32!\n");
			return -1;
		}
		pos += 1;
		if (*(pos + 6 + (data_len * 2) + 2) != '\n' && *(pos + 6
				+ (data_len * 2) + 2) != '\r') {
			printf("Error finding the delimiter!\n");
			return -1;
		}

		for (i = 0; i < 4; i++, pos += 1) {
			if (!is_hex(*pos))
				return -1;
			if (!ascii_to_char(pos, &buff_high))
				return -1;
			addr += buff_high;
			if (i < 3)
				addr <<= 4;
			/*addr now holds an address*/
		}
		if (addr < MSP_INFO_START_ADDR || addr >= MSP_MAIN_END_ADDR
				|| (addr >= MSP_INFO_END_ADDR && addr
						< MSP_MAIN_START_ADDR)) {
			printf("Incorrect address!\n");
			return -1;
		}
		if (addr % 2 != 0) {
			printf("Incorrect row address - uneven!\n");
			return -1;
		}

		buffer[1] = (addr & 0xFF00) >> 8;
		buffer[2] = (addr & 0x00FF);
		/*process record type bytes now*/
		if (!is_hex(*pos) || !is_hex(*(pos + 1)))
			return -1;
		if (!ascii_to_char(pos, &buff_high))
			return -1;
		pos += 1;
		if (!ascii_to_char(pos, &buff_low))
			return -1;
		pos += 1;
		tmp = (buff_high << 4) + buff_low;
		buffer[3] = tmp;

		switch (tmp) {
		case MSP_DATA:
			for (i = 0; i < data_len; i++, pos += 2) {
				if (!is_hex(*pos) || !is_hex(*(pos + 1)))
					return -1;

				if (!ascii_to_char(pos, &buff_high))
					return -1;
				if (!ascii_to_char(pos + 1, &buff_low))
					return -1;
				buff_byte = (buff_high << 4) + buff_low;
				/* store data at desired offset */
				buffer[i + 4] = buff_byte;
			}
			/* pointer is now at first checksum byte */
			/* add a checksum validation function */
			if (!is_hex(*pos) || !is_hex(*(pos + 1))) {
				printf("Invalid checksum row ending bytes!\n");
				return -1;
			}
			if (!ascii_to_char(pos, &buff_high))
				return -1;
			if (!ascii_to_char(pos + 1, &buff_low))
				return -1;
			buff_byte = (buff_high << 4) + buff_low;
			if (calc_row_cksum(buffer, data_len + 4) != buff_byte) {
				printf("Error validating row checksum!\n");
				return -1;
			}
			pos += 2;
			if (*pos != '\n' && *pos != '\r') {
				printf("Invalid data row ending!\n");
				return -1;
			}
			msp_find_pass(addr, buffer, data_len, password);
			/* checking for correct row ending (either '/n' or '/r/n')*/
			if (*pos == '\r') {
				pos += 2;
			} else if (*pos == '\n')
				pos += 1;
			*index = pos - base;
			break;
			/* MSP_EOF obviously is in conflict with the protocol in this case */
		case MSP_EOF:
			return -1;
			break;
			/* MSP_EXT is not handled by the parser */
		case MSP_EXT:
			return -1;
			break;
		default:
			return -1;
		}
	}

	return 0;
}

/* helping function for validating a firmware flash file */
int do_msp_file_validate(char* fname, unsigned char* password)
{
	unsigned char *filep = (unsigned char *)MSP_LOAD_ADDR;
	unsigned int i, j, num;

	if (do_msp_file_load(fname) == -1)
		return -1;

	i = msp_row_validate(filep, &num, password);
	j = num;
	while (i == 0) {
		i = msp_row_validate(filep + j, &num, password);
		j += num;
	}
	if (i == -1) {
		printf("Error validating file!\n");
		return -1;
	}
	if (i == 1) {
		printf("File validation passed!\n");
	}
	return 0;
}

/* helping function for flashing a file*/
int do_msp_flash()
{
	unsigned char *filep = (unsigned char *)MSP_LOAD_ADDR;
	int i = 0;
	unsigned int j = 0, num = 0;
	if (filep == NULL) {
		printf("Invalid starting address or failure loading file!\n");
		return -1;
	}
	if ((i = msp_flash_row(filep, &num)) == -1) {
		printf("MSP Flash error\n");
		return -1;
	}

	j = num;
	while (i == 0) {
		i = msp_flash_row(filep + j, &num);
		j += num;
	}
	if (i == -1) {
		printf("MSP Flash error\n");
		return -1;
	}
	printf("MSP Flashing passed!\n");
	return 0;
}

/* validates a password provided as an optional argument and converts it from ascii to a
 * 32-byte password suitable for msp query;
 * the optional password must be exactly 64 chars long because of the msp 32-byte
 * password limitation; the function returns -1 in case of error or 0 in case of
 * success*/
int opt_password_validate(unsigned char* o_password,
		unsigned char* dest_password)
{
	int i = 0;
	unsigned char buff_byte;
	unsigned char buff_high;
	unsigned char buff_low;

	if (strlen((char *)o_password) != 64) {
		printf("Password must be 64 chars!\n");
		return -1;
	}
	if (o_password != MSP_DEF_PASS) {
		for (i = 0; i < MSP_OPT_PASS_LEN; i++)
			o_password[i] = toupper(o_password[i]);
	}
	for (i = 0; i < MSP_OPT_PASS_LEN - 1; i++) {
		if (!is_hex(o_password[i])) {
			printf(
					"Invalid password - must contain valid hex chars!\n");
			return -1;
		}
	}
	for (i = 0; i < MSP_OPT_PASS_LEN - 1; i += 2) {
		if (!ascii_to_char(o_password + i, &buff_high))
			return -1;
		if (!ascii_to_char(o_password + i + 1, &buff_low))
			return -1;
		buff_byte = (buff_high << 4) + buff_low;
		dest_password[i / 2] = buff_byte;
	}
	return 0;
}
/*********************************************************************************/
/* U-Boot commands*/
/*********************************************************************************/

/*
 * Executes a msp file flashing sequence
 * Syntax:
 * 	mspflash {file_name} [{optional password}]
 * MSP main memory is erased first and then reflashed
 */
int do_msp_cmd_flash(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	unsigned char password[MSP_PASS_LEN];
	unsigned char opt_password[MSP_OPT_PASS_LEN];
	/* used to store a located into the file password if found*/
	unsigned char read_password[MSP_PASS_LEN];

	memset(password, '\0', MSP_PASS_LEN);
	memset(opt_password,'\0', MSP_OPT_PASS_LEN);
	memset(read_password, 0xFF, MSP_PASS_LEN);

	if (argc != 2 && argc != 3) {
		printf("Usage:\n%s\n", cmdtp->help);
		return 1;
	}
	if (argc == 2) {
		if (opt_password_validate(MSP_DEF_PASS, password) == -1) {
			return 1;
		}
	}
	if (argc == 3) {
		printf("Optional mode selected!\n");
		if (opt_password_validate((unsigned char *)argv[2], password) == -1) {
			return 1;
		}
	}
	if (do_msp_file_validate(argv[1], read_password) == -1)
		return 1;

	msp_bsl_reset(BSL_START);
	udelay(250000);

	msp_serial_init((NS16550_t) CFG_NS16550_COM2, MSP_B9600);
	udelay(250000);

	if(msp_send_pass(password) == -1)
		return 1;
	if(msp_erase_main() == -1)
		return 1;
	udelay(300000);

	memset(password, 0xFF, MSP_PASS_LEN);

	if (msp_send_pass(password) == -1)
		return 1;

	printf("Now flashing the msp! Please wait!\n");

	if (do_msp_flash() == -1)
		return 1;

	udelay(100000);
	msp_bsl_reset(USER_PROGRAM_START);

	return 0;
}

U_BOOT_CMD(
		mspflash, 3, 0, do_msp_cmd_flash,
		"mspflash- used to flash a new msp430 firmware file\n",
		"\nfilename [optional password]\n\
		filename(.hex) must be on mmc\n\
		password is optional and must be 64 chars long\n"
);
