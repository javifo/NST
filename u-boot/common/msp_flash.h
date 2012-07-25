/*
 * msp_flash.h
 *
 *  Created on: Feb 15, 2011
 *      Authors: x0151846 and x0151843
 */

#ifndef MSP_FLASH_H_
#define MSP_FLASH_H_

/* msp implemented command codes */
#define MSP_RX_DATA_BLOCK        0x12
#define MSP_RX_PASSWORD          0x10
#define MSP_MASS_ERASE           0x18
#define MSP_TX_DATA_BLOCK        0x14
#define MSP_TX_BSL_VERSION       0x1E
#define MSP_ERASE_MAIN		0x16
/* msp command fields in use*/
#define MSP_DATA_ACK	0x90
#define MSP_DATA_NAK	0xA0
#define MSP_SYNC	0x80
#define MSP_HDR		0x85
/* helping command formation macros*/
#define BSL_RESPONSE_LEN	22
#define BSL_START	1
#define USER_PROGRAM_START	0
/*delay macros for read/write oerations in us*/
#define MSP_RW_DELAY	1200
/* maximum number of uart 2 read retry attempts*/
#define MSP_READ_RETRY	500
/* uart 2 setup macros
 * 8 data, 1 stop, even parity */
#define LCR_8E1	0x1B
#define UART2	2
/* RTS/DTR */
#define MSP_MCRVAL	(MCR_DTR | MCR_RTS)
/* Clear & enable FIFOs */
#define MSP_FCRVAL	(FCR_FIFO_EN | FCR_RXSR | FCR_TXSR)

#ifdef CONFIG_APTIX
#define MODE_X_DIV 13
#else
#define MODE_X_DIV 16
#endif
/* msp gpio pins of interest */
#define MSP_MCBSP3_DX	140
#define MSP_MCBSP3_DR	141
/* msp file flash macros */
#define MSP_FLASH_SIZE	0x1D000 /*max 116k*/
#define MSP_DEV_NUM	0x00
#define MSP_DEVICE	"mmc"
#define MSP_PARTITION	0x01
/* msp address map bounds*/
#define MSP_INFO_START_ADDR	0x01000
#define MSP_INFO_END_ADDR	0x010FF
#define MSP_MAIN_START_ADDR	0x08000
#define MSP_MAIN_END_ADDR	0x0FFFF
#define MSP_PASS_START_ADDR	0x0FFE0
/* maximum data allowed in a flash file row*/
#define MSP_ROW_MAXDLEN	250
/* start address for loading msp flash file*/
#define MSP_LOAD_ADDR	0x80000000
/* typical intel hex end of file record */
#define MSP_EOF_STR	0x00000001FF
/*default uart2 baudrate*/
#define MSP_B9600	9600
/* msp password length macro */
#define MSP_PASS_LEN 32
/* length of scanned password */
#define MSP_OPT_PASS_LEN 65
/* intel hex file record type macros*/
enum {
	MSP_DATA = 0x00, MSP_EOF = 0x01, MSP_EXT = 0x04
};

/* initializes serial port connected to msp chip*/
void msp_serial_init(NS16550_t com_port, unsigned int baudrate);

/* wrapping functions for manipulating msp serial port gpios */
void msp_set_reset_pin(int level);
void msp_set_test_pin(int level);

/* resets msp and starts it either in bootstrap loader mode
 * or application
 */
void msp_bsl_reset(int invoke_bsl);

/* executes a msp sync operation and returns 1 in case of success (ACK) or
 * -1 in case of negative acknowledge (NACK) or 0 in case of timeout
 */
int msp_bsl_sync(void);

/* used for reading a single data byte from msp uart -
 * returns 0 in case of success or -1 in case of failure
 */
int rs_read_byte(unsigned char* rx_byte);

/* used for writing a single data byte to msp uart */
static inline void rs_write_byte(unsigned char tx_byte, NS16550_t com_port);

/* calculates a msp command checksum and
 * returns the desired high and low bytes through chk_sum_h and
 * chk_sum_l pointers respectively - consult the msp datasheet
 */
void msp_cmd_chksum(unsigned char* buffer, int length,
		unsigned char *chk_sum_l, unsigned char *chk_sum_h);

/* obtaining and validating BSL Version;
 * returns 0 if successful or -1 in case of error;
 * it is not password protected; prior to usage msp uart should be initialized
 * and msp started in bsl mode */
int msp_read_bsl_version(void);

/* function for writing a block of data (tx_data)
 * at address (addr) with length specified by the length parameter (max 250 bytes),
 * returns -1 in case of failure or 0 in case of success; data_padding with 0xFF is
 * ensured if the function is used separately with uneven length;
 * the function requires password unlocking via msp_send_pass function;
 * prior to usage msp uart should be initialized
 * and msp started in bsl mode
 */
int msp_send_block(unsigned char *tx_data, unsigned int addr,
		unsigned int length);

/* function for reading a data block starting at addr with the specified in length
 * data length; max length is limited to 250 bytes per reading and if this function is
 * to be used separately on its own it requires also password unlocking via msp_send_pass
 * function; returns -1 in case of error and 0 if successful;
 * destination array must have length + 7 size at least;
 * prior to usage msp uart should be initialized
 * and msp started in bsl mode
 */
int msp_read_block(unsigned char *rx_message, unsigned int addr,
		unsigned int length);

/* function for loading a firmware file in memory - the starting address is specified
 * by MSP_LOAD_ADDR as a macro - returns -1 in case of error and 0 if successful, a valid
 * filename is expected; storage device and partition are also internally provided as
 * macros; MSP_LOAD_ADDR typically should not be changed.
 */
int do_msp_file_load(char* fname);

/* used for proper checksum calculation for each data row in the firmware file*/
unsigned char calc_row_cksum(unsigned char* buffer, unsigned int length);

/* helping function to determine whether an ASCII symbol is a valid hex digit -
 * returns 1 if successful or 0 in case of error
 */
int is_hex(unsigned char sym);

/* returns 1 if conversion from ascii code to unsigned char is successful or 0
 * if failed
 */
int ascii_to_char(unsigned char* sym, unsigned char* dest);

/* querying msp for a particular 32-byte password - returns 0 if successful (password is accepted)
 * or -1 in case of error (password is denied);
 * prior to usage msp uart should be initialized
 * and msp started in bsl mode
 */
int msp_send_pass(unsigned char* password);

/* function for performin main memory erasing only - it is password protected;
 * returns -1 in case of error or 0 in case of success;
 * prior to usage msp uart should be initialized, msp started in bsl mode and
 * valid password accepted by msp */
int msp_erase_main(void);

/* function for performing complete msp flash mass erase  - it is not password protected
 * returns 0 if successful or -1 in case of error;
 * prior to usage msp uart should be initialized
 * and msp started in bsl mode*/
int msp_mass_erase(void);

/* helping function used to flash a row from the firmware file
 * into msp - returns 1 in case of end of file,
 * 0 - in case there is more to read and -1 in case of failure;
 * this function is to be invoked only after complete file validation.
 * moreover, the function invokes a password protected series of commands, so before calling it for
 * the first time only (for a particular session) msp_send_pass is to be called with a proper password
 * in advance (msp uart should be initialized and msp started in bsl mode of course)
 */
int msp_flash_row(unsigned char* base, unsigned int* index);

/* function for extracting password bytes at a given address when reading the file
 * if no password has been provided then the default stored in password array is to be used;
 * addr specifies the address read in the exmined row, buffer holds a file row data and data_len specifies
 * the length of the actual row data as it is in the intel hex format
 */
void msp_find_pass(unsigned int addr, unsigned char* buffer,
		unsigned int data_len, unsigned char* password);

/* function for flash file row validation - returns 1 in case correct end of file found,
 * 0 - in case the row is correct and there is more to read and -1 in case of wrong data row
 */
int msp_row_validate(unsigned char* base, unsigned int* index,
		unsigned char* password);

/* function for executing the sequence  of loading and validating a firmware file
 * in memory - returns 0 in case of success
 * or -1 in case of failure; a 32-byte password array is to be provided
 * to store the located password if one is found in the file
 */
int do_msp_file_validate(char* fname, unsigned char* password);

/* function which performs the required sequence of actions for a proper firmware
 * file flashing - returns -1 in case of error or 0 if successful
 */
int do_msp_flash(void);

/* validates a password provided as an optional argument and converts it from ascii to a
 * 32-byte password suitable for msp query;
 * the optional password is checked to be exactly 64 chars long because of the msp 32-byte
 * password limitation; the function returns -1 in case of error or 0 in case of
 * success*/
int opt_password_validate(unsigned char* o_password,
		unsigned char* dest_password);

#endif /* MSP_FLASH_H_ */
