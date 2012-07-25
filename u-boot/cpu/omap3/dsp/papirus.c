
// KNOWN TARGETS : CONFIG_OMAP3_BEAGLE. CONFIG_3630EDP1, CONFIG_3621EDP1, CONFIG_3621GOSSAMER.

#include <common.h>
#include <asm/io.h>
#include <asm/arch/mem.h>	/* get mem tables */
#include <asm/arch/sys_proto.h>
#include <asm/arch/mux.h>
#if defined(CONFIG_3621GOSSAMER)
	#include <asm/arch/sys_info.h>
#endif

#include <fat.h>
#include <linux/ctype.h>

#include "dsp_cmn.h"
#include "papirus.h"
#include "gpio.h"
#include "gossamer_splash_interface.h"

#if defined(CONFIG_3630EDP1)
	#define PAPYRUS_I2C_BUS 	2
#elif defined (CONFIG_3621EDP1)
	#define PAPYRUS_I2C_BUS 	1
#elif defined(CONFIG_3621GOSSAMER)
	#define PAPYRUS_I2C_BUS 	1
#elif defined(CONFIG_OMAP3_BEAGLE)
	#define PAPYRUS_I2C_BUS 	/// Beagle Board has no any PAPYRUS.
#else
	#error "PAPYRUS_I2C_BUS undefined"
#endif

/* Papyrus1 register map */
#define PAPYRUS1_TMST_VALUE_REG		0x00
#define PAPYRUS1_ENABLE_REG			0x01
#define PAPYRUS1_VP_ADJ_REG			0x02
#define PAPYRUS1_VN_ADJ_REG			0x03
#define PAPYRUS1_VCOM_ADJUST_REG	0x04
#define PAPYRUS1_INT_ENABLE1_REG	0x05
#define PAPYRUS1_INT_ENABLE2_REG	0x06
#define PAPYRUS1_INT_STATUS1_REG	0x07
#define PAPYRUS1_INT_STATUS2_REG	0x08
#define PAPYRUS1_PWRSEQ0_REG		0x09
#define PAPYRUS1_PWRSEQ1_REG		0x0a
#define PAPYRUS1_PWRSEQ2_REG		0x0b
#define PAPYRUS1_TMST_CONFIG_REG	0x0c
#define PAPYRUS1_TMST_OS_REG		0x0d
#define PAPYRUS1_TMST_HYST_REGi		0x0e
#define PAPYRUS1_PG_STATUS_REG		0x0f
#define PAPYRUS1_REVID_REG			0x10
#define PAPYRUS1_FIX_READ_PTR_REG	0x11

/* PAPYRUS2 register map */
#define PAPYRUS2_TMST_VALUE_REG		0x00
#define PAPYRUS2_ENABLE_REG			0x01
#define PAPYRUS2_VADJ_REG			0x02
#define PAPYRUS2_VCOM1_ADJUST_REG	0x03
#define PAPYRUS2_VCOM2_ADJUST_REG	0x04
#define PAPYRUS2_INT_ENABLE1_REG		0x05
#define PAPYRUS2_INT_ENABLE2_REG		0x06
#define PAPYRUS2_INT_STATUS1_REG		0x07
#define PAPYRUS2_INT_STATUS2_REG		0x08
#define PAPYRUS2_UPSEQ0_REG			0x09
#define PAPYRUS2_UPSEQ1_REG			0x0a
#define PAPYRUS2_DWNSEQ0_REG			0x0b
#define PAPYRUS2_DWNSEQ1_REG			0x0c
#define PAPYRUS2_TMST1_REG			0x0d
#define PAPYRUS2_TMST2_REG			0x0e
#define PAPYRUS2_PG_STATUS_REG		0x0f
#define PAPYRUS2_REVID_REG			0x10

/* I2C chip addresses */
#define PAPYRUS2_BASE_ADDR_EVT1C     0x48
#define PAPYRUS2_BASE_ADDR_EVT2      0x68
#define PAPYRUS1_BASE_ADDR			0x48

#define PAPYRUS1_VCOM_STEP 11
#define PAPYRUS2_VCOM_STEP 10

static unsigned int PAPYRUS_BASE_ADDR = 0x00;  //will be defined run-time in the init

static int papirus_power_on = 0;

#if defined(CONFIG_3621GOSSAMER)	// || defined(CONFIG_3630EDP1) || defined(CONFIG_3621EDP1)
/*
 * Convience functions to read and write from TWL4030
 *
 * chip_no is the i2c address, it must be one of the chip addresses
 *   defined at the top of this file with the prefix TWL4030_CHIP_
 *   examples are TWL4030_CHIP_PM_RECEIVER and TWL4030_CHIP_KEYPAD
 *
 * val is the data either written to or read from the twl4030
 *
 * reg is the register to act on, it must be one of the defines
 *   above and with the format TWL4030_<chip suffix>_<register name>
 *   examples are TWL4030_PM_RECEIVER_VMMC1_DEV_GRP and
 *   TWL4030_LED_LEDEN.
 */
static inline int papirus_i2c_write_u8(u8 chip_no, u8 val, u8 reg)
{
#if defined(SPLASH_SCREEN_DEBUG)
	printf("-I-papirus_i2c_write_u8(0x%.2X, 0x%.2X, 0x%.2X)\n",
		chip_no, val, reg);
#else
	udelay(10000); //Wait
#endif
	return i2c_write(chip_no, reg, 1, &val, 1);
}

static inline int papirus_i2c_read_u8(u8 chip_no, u8 *val, u8 reg)
{
	int retval = i2c_read(chip_no, reg, 1, val, 1);
#if defined (SPLASH_SCREEN_DEBUG)
	printf("-I-papirus_i2c_read_u8(0x%.2X, 0x%.2X, 0x%.2X)\n",
		chip_no, *val, reg);
#else
	udelay(10000); //Wait
#endif
	return retval;
}

//#define	MESS_TEMP			0x54		// Version with reading temperature (EDP1 with TPS65180/TPS65181)
//#define TEMP_MEASURE_DELAY	(60*1000)	// Version with reading temperature (EDP1 with TPS65180/TPS65181)

#define CPLD_RESET			(88)
#define EN_CPLD_POWER		(85)
#define EPD_WAKEUP			(87)
#define WAKEUP_DELAY		(50)
#define VCOM_DELAY			(100)
#define VCOM_DISCHARGE_DELAY_US			350000
#define PAPYRUS2_HIGH_VOL_PWRDN_DELAY_US 200000
#endif

#if defined(SPLASH_SCREEN_DEBUG)
	#define print_info(fmt,args...)	printf ("-I-PAPYRUS2-: " fmt "\n", ##args)
	#define print_info_fileop(fmt,args...)	printf ("-I-FILE-: " fmt "\n", ##args)
#else
	#define print_info(fmt,args...)
	#define print_info_fileop(fmt,args...)
#endif

// Splash screen related: loads file for use by DSP.
long splash_load_file2buffer (const char *fname, void * const buf, const int max_sz)
{
	long size;

	print_info_fileop ("   ** ******** OFFSET %lx ** **********", (unsigned long)buf);
	size = file_fat_read (fname, (unsigned char*)buf, max_sz);
	print_info_fileop("Downloading file '%s' through !!! file_fat_read !!! %ld\n", fname, size);
	if (size < 0) {
		printf("ERROR: Open file '%s'\n", fname);
		return -1;
	}
	if (size < 1 || size > max_sz) {
		printf("ERROR: File size (%d). Buffer size (%d bytes)\n", (int)size, max_sz);
		return -1;
	}
	return (size);
}

#if !defined(CONFIG_OMAP3_BEAGLE)
//------------------------------------------------------------------------------
int papyrus1_power_off(void)
{
	int stat = 0;

	if(!papirus_power_on)
		return stat;

	print_info("Power Off PAPYRUS2 by I2C bus\n");

	i2c_set_bus_num(PAPYRUS_I2C_BUS);
	stat |= omap_request_gpio(EN_CPLD_POWER);

	if (stat) {
		printf("PAPYRUS2: cannot reserve GPIOs\n");
		i2c_set_bus_num(0);
		stat = -1;
		return stat;
	}

	/* Disable VCOM */
	if ( papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0xaf, PAPYRUS1_ENABLE_REG) )
		printf("PAPYRUS1: cannot disable VCOM driver\n");
	udelay(VCOM_DISCHARGE_DELAY_US);

	/* Set powerdown sequence to VDDH->VPOS->VNEG->VEE */
	if ( papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0xe1, PAPYRUS1_PWRSEQ0_REG) )
		printf("PAPYRUS1: cannot switch Papyrus1 powerdown sequence\n");
	/* shutdown delays set to 6/6/15/3 ms between each strobe
       as close as possible to TPS65185 */
	if ( papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0xf3, PAPYRUS1_PWRSEQ1_REG) )
		printf("PAPYRUS1: cannot change Papyrus1 powerdown delay`s\n");
	if ( papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x66, PAPYRUS1_PWRSEQ2_REG) )
		printf("PAPYRUS1: cannot change Papyrus1 powerdown delay`s\n");

	/* PAPYRUS1 goes to standby -  keep 3.3V active */
	if ( papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x6f, PAPYRUS1_ENABLE_REG) )
		printf("PAPYRUS2: cannot send PAPYRUS1 to standby\n");

	/* 3.3V must be the last to come down */
	udelay(PAPYRUS2_HIGH_VOL_PWRDN_DELAY_US);
	if ( papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x4f, PAPYRUS1_ENABLE_REG) )
		printf("PAPYRUS2: cannot turn off 3.3V\n");

#if defined(CONFIG_3621GOSSAMER)
	/* CPLD reset */
	omap_set_gpio_dataout(CPLD_RESET, 0);
	omap_set_gpio_dataout(EN_CPLD_POWER, 0);
#endif

	i2c_set_bus_num(0);
	papirus_power_on = 0;

	return (stat);
}

//------------------------------------------------------------------------------
int papyrus2_power_off(void)
{
	int stat = 0;

	if(!papirus_power_on)
		return stat;
#if defined(CONFIG_3621GOSSAMER)
	print_info("Power Off PAPYRUS2 by I2C bus\n");

	i2c_set_bus_num(PAPYRUS_I2C_BUS);
	stat |= omap_request_gpio(EN_CPLD_POWER);

	if (stat) {
		printf("PAPYRUS2: cannot reserve GPIOs\n");
		i2c_set_bus_num(0);
		stat = -1;
		return stat;
	}

	/* Disable VCOM */
	if ( papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0xaf, PAPYRUS2_ENABLE_REG) )
		printf("PAPYRUS2: cannot send PAPYRUS2 to standby\n");
	udelay(VCOM_DISCHARGE_DELAY_US);

	/* PAPYRUS2 goes to standby -  keep 3.3V active */
	if ( papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x6f, PAPYRUS2_ENABLE_REG) )
		printf("PAPYRUS2: cannot send PAPYRUS2 to standby\n");

	/* 3.3V must be the last to come down */
	udelay(PAPYRUS2_HIGH_VOL_PWRDN_DELAY_US);
	if ( papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x4f, PAPYRUS2_ENABLE_REG) )
		printf("PAPYRUS2: cannot turn off 3.3V\n");

	omap_set_gpio_dataout(CPLD_RESET, 0);
	omap_set_gpio_dataout(EN_CPLD_POWER, 0);

	/* wait to reset PAPYRUS2 */
	udelay(WAKEUP_DELAY);

	i2c_set_bus_num(0);
#elif 	defined(CONFIG_3630EDP1) || defined(CONFIG_3621EDP1)
	// TODO: When necessary add code for TPS65180/TPS65181 (old PAPYRUS2)
#endif
	papirus_power_on = 0;

	return (stat);
}
#endif	///!defined(CONFIG_OMAP3_BEAGLE)

//------------------------------------------------------------------------------
static int err_deactivate(void)
{
#if !defined(CONFIG_OMAP3_BEAGLE)
	print_info("Error =?= PAPYRUS2 I2C bus\n");

// Send papyrus to sleep
	if (!omap_request_gpio(EPD_WAKEUP)) {
		omap_set_gpio_dataout(EPD_WAKEUP, 0);   //GPIO 87 -> 0
		omap_set_gpio_direction(EPD_WAKEUP, 0);
	}

//Disable CPLD power
	if (!omap_request_gpio(EN_CPLD_POWER)) {
		omap_set_gpio_dataout(EN_CPLD_POWER, 0); //GPIO 85 -> 0
		omap_set_gpio_direction(EN_CPLD_POWER, 0);
	}
#endif

	papirus_power_on = 0;

	return 1;
}

//------------------------------------------------------------------------------
int dis_power_off(void)
{
	int stat = 0;;

#if !defined(CONFIG_OMAP3_BEAGLE)
	DECLARE_GLOBAL_DATA_PTR;
#if defined(CONFIG_3621GOSSAMER)
	switch (gd->bd->bi_board_revision) {
		case BOARD_GOSSAMER_REV_EVT2:
        case BOARD_GOSSAMER_REV_EVT1C:
		case BOARD_GOSSAMER_REV_EVTPRE1C:
			stat = papyrus2_power_off();
			break;
		default: /* EVT1a and EVT3, DVT, PVT... */
			stat = papyrus1_power_off();
			break;
		}
#elif defined(CONFIG_3630EDP1) || defined (CONFIG_3621EDP1)
	stat = papyrus1_power_off();
#endif
#endif

	return stat;
}

// Support for reading VCOM from ROM token
//------------------------------------------------------------------------------
static int papyrus_get_vcom_from_boot_rom(int *vcom_out, int vcom_step_mv)
{
	char *vcom_str = NULL;
	char *ep = NULL;
	int vcom = 0;

	vcom_str = getenv("vcom");

	if (!vcom_str) {
		// No vcom available, probably on external
		return -1;
	}

	vcom = simple_strtol(vcom_str, &ep, 10);
	vcom = -vcom/vcom_step_mv;
	*vcom_out = vcom;

	return 0;
}

// Support for reading VCOM from boot.scr
//------------------------------------------------------------------------------
static void _tolower_buffer( char *buf, const int len)
{
	int i;
	for (i = 0; i < len; i++) {
		buf[i] = tolower(buf[i]);
	}
}

//------------------------------------------------------------------------------
static int _atodn( int *val, const char *buf, const int max_chars)
{
	int i=0;
	int sign = 0;
	int err = 0;

	*val = 0;
	if (buf[i] == '-') {
		i++;
		sign = 1;
	} else if (buf[i] == '+') {
		i++;
	}
	if (!isdigit(buf[i]))
		err = 1;
	for( ; i < max_chars && !err; i++) {
		if (!isdigit(buf[i]))
			break;
		*val = (*val)*10 + (buf[i] - '0');
	}
	if (sign)
		*val = -(*val);
	return (err);
}

// Support for reading VCOM from boot.scr
//------------------------------------------------------------------------------
static int papyrus_get_vcom_from_boot_scr(int vcom_step_mv)
{
	static char file_buf[5*1024+1];
	char *p;
	const char *fname = "boot.scr";
	long size;
	int vcom = 0;

	size = splash_load_file2buffer( fname, file_buf, sizeof(file_buf)-1);
	if (size < 0 || size > (sizeof(file_buf)-1)) {
		return (-1);
	}
	// Search VCOM argument.
	file_buf[size] = 0;
	_tolower_buffer( file_buf, size);
	for (p = file_buf; p < (file_buf + sizeof(file_buf) - 5); p++) {
		if (0 == strncmp( p, "vcom=", 5)) {
			if (0 == _atodn( &vcom, p+5, 7)) {
				vcom = -vcom/vcom_step_mv;
				return(vcom);// Success
			} else {
				printf("-W- Error parsing VCOM value in 'boot.scr'\n");
				return (-1);
			}
		}
	}
	printf("-W- Argument VCOM not found in 'boot.scr'\n");
	return (-1);
}

static int papyrus_get_vcom(int vcom_step_mv)
{
    int vcom = 0;

    if (!papyrus_get_vcom_from_boot_rom(&vcom, vcom_step_mv)) {
        // Successfully read vcom from ROM token
        return vcom;
    }

    printf("Failed reading VCOM from ROM token, trying boot.scr\n");
    // No rom token, try boot.scr
    vcom = papyrus_get_vcom_from_boot_scr(vcom_step_mv);

    if (vcom == -1) {
        printf("Failed to read VCOM from boot.src, returning default\n");
    }

    return vcom;
}

#if !defined(CONFIG_OMAP3_BEAGLE)
//------------------------------------------------------------------------------
int papyrus1_init(void)
{
	u8 version;
    int vcom;
	//Setup papirus via I2C
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0,    PAPYRUS1_FIX_READ_PTR_REG))
		return -1;
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0,    PAPYRUS1_ENABLE_REG))  //deactivate write in enable register -> 0
		return -1;

	//Check version
	if(!papirus_i2c_read_u8(PAPYRUS_BASE_ADDR, &version, PAPYRUS1_REVID_REG))
		print_info("Papirus version = %x (TPS6518%dr%dp%d)\n",version,
		version & 0xF, (version & 0xC0) >> 6, (version & 0x30) >> 4);
	else
		return -1;

	/* Powerup sequence : VNEG->VEE->VPOS->VDDH */
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0xE4, PAPYRUS1_PWRSEQ0_REG))
		return -1;
	/* 6 ms between each power rail startup */
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x66, PAPYRUS1_PWRSEQ1_REG))
		return -1;
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x66, PAPYRUS1_PWRSEQ2_REG))
		return -1;

	//Start temperature measure
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x84, PAPYRUS1_TMST_CONFIG_REG))
		return -1;

	vcom = papyrus_get_vcom(PAPYRUS1_VCOM_STEP);
	if (vcom < 28 || vcom > 227) {
		vcom = 0x74;// 0x74 keep at default
		printf("-W- WARNING: VCOM set to default -%dmV\n", vcom*PAPYRUS1_VCOM_STEP);
	} else {
		print_info("VCOM = -%dmV", vcom*PAPYRUS1_VCOM_STEP);
	}

	/* set VADJ : VCOM controlled by I2C */
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0xA3, PAPYRUS1_VN_ADJ_REG))
		return -1;
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x33, PAPYRUS1_VP_ADJ_REG))
		return -1;
	// set VCOM
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, (uint8)vcom, PAPYRUS1_VCOM_ADJUST_REG))
		return -1;

	//Turn On
	/* 3.3V switch first */
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x20, PAPYRUS1_ENABLE_REG))
		return -1;
	/* Then everyone else, but VCOM */
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0xaf, PAPYRUS1_ENABLE_REG))
		return -1;

	return 0;
}

//------------------------------------------------------------------------------
int papyrus1_pic_voltage_enable(volatile struct epd_splash_cfg_arm *pIf)
{
	int err_value = 0;
	u8 read_val;

	//Enable VCOM
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x3f, PAPYRUS1_ENABLE_REG))
		return -1;

	{//Read temperature
		int i;
		for (i = 0; i < 50;) {
			err_value = papirus_i2c_read_u8(PAPYRUS_BASE_ADDR, &read_val, PAPYRUS1_TMST_CONFIG_REG);
			i++;
			if (!err_value && !(read_val & 0x80))
				break;
		}
		if (!err_value && !(read_val & 0x80)) {
			if(!papirus_i2c_read_u8(PAPYRUS_BASE_ADDR, &read_val, PAPYRUS1_TMST_VALUE_REG))
			{
				pIf->cmn.temperature = read_val;
				print_info("Papirus temperature = %d, tries to read = %d\n", pIf->cmn.temperature, i);
			} else {
				return -1;
			}
		} else {
			return -1;
		}
	}
	{//Check PG
		int i;
		for (i = 0; i < 50;) {
			err_value = papirus_i2c_read_u8(PAPYRUS_BASE_ADDR, &read_val, PAPYRUS1_PG_STATUS_REG);
			i++;
			if (err_value)
				continue;
			print_info("Papirus PG status (after TURN ON & temp.measure) = %x, i = %d\n",read_val, i);
			if (read_val == 0xfa)
				break;
		}
		if (err_value || read_val != 0xfa) {
			return -1;
		}
	}
	return 0;
}

//------------------------------------------------------------------------------
int papyrus2_init(void)
{
	u8 version;
    int vcom;

	//Setup papirus via I2C
	//Init
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0,    PAPYRUS2_ENABLE_REG))  //deactivate write in enable register -> 0
		return -1;

	//Check version
	if(!papirus_i2c_read_u8(PAPYRUS_BASE_ADDR, &version, PAPYRUS2_REVID_REG))
		print_info("Papirus version = %x\n",version);
	else
		return -1;

	//Start temperature measure
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x83, PAPYRUS2_TMST1_REG))
		return -1;

	vcom = papyrus_get_vcom(PAPYRUS2_VCOM_STEP);
	if (vcom < 0 || vcom > 511) {
		vcom = 0x74;// 0x74 keep at default
		printf("-W- WARNING: VCOM set to default -%d0mV\n", vcom);
	} else {
		print_info("VCOM = -%d0mV", vcom);
	}
	// set VCOM1
        if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, (uint8)vcom, PAPYRUS2_VCOM1_ADJUST_REG))
		return -1;

	// set VCOM2
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x04 | ((uint8)((vcom>>8)&1)), PAPYRUS2_VCOM2_ADJUST_REG))// ??? keep other bits at default
		return -1;

	// set VADJ
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x23, PAPYRUS2_VADJ_REG))// 0x23 is the default +-15.000V
		return -1;

	//Turn On
	/* 3.3V switch first */
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x20, PAPYRUS2_ENABLE_REG))
		return -1;
	/* Then everyone else, but VCOM */
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0xaf, PAPYRUS2_ENABLE_REG))
		return -1;

	return 0;
}

//------------------------------------------------------------------------------
int papyrus2_pic_voltage_enable(volatile struct epd_splash_cfg_arm *pIf)
{
	int err_value = 0;
	u8 read_val;

	//Enable VCOM
	if(papirus_i2c_write_u8(PAPYRUS_BASE_ADDR, 0x3f, PAPYRUS2_ENABLE_REG))
		return -1;

	{//Read temperature
		int i;
		for (i = 0; i < 50;) {
			err_value = papirus_i2c_read_u8(PAPYRUS_BASE_ADDR, &read_val, PAPYRUS2_TMST1_REG);
			i++;
			if (!err_value && !(read_val & 0x80))
				break;
		}
		if (!err_value && !(read_val & 0x80)) {
			if(!papirus_i2c_read_u8(PAPYRUS_BASE_ADDR, &read_val, PAPYRUS2_TMST_VALUE_REG))
			{
				pIf->cmn.temperature = read_val;
				print_info("Papirus temperature = %d, tries to read = %d\n", pIf->cmn.temperature, i);
			} else {
				return -1;
			}
		} else {
			return -1;
		}
	}
	{//Check PG
		int i;
		for (i = 0; i < 50;) {
			err_value = papirus_i2c_read_u8(PAPYRUS_BASE_ADDR, &read_val, PAPYRUS2_PG_STATUS_REG);
			i++;
			if (err_value)
				continue;
			print_info("Papirus PG status (after TURN ON & temp.measure) = %x, i = %d\n",read_val, i);
			if (read_val == 0xfa)
				break;
		}
		if (err_value || read_val != 0xfa) {
			return -1;
		}
	}
	return 0;
}
#endif /// #if !defined(CONFIG_OMAP3_BEAGLE)

//------------------------------------------------------------------------------
int read_dsp_fw(const char *init_img_fname, volatile struct epd_splash_cfg_arm *pIf) {
	long size;

	// Read DSP code.
	size = splash_load_file2buffer( "flash_spl.bin", (void*)DSP_BOOT_ADDR, DSP_BOOT_SIZE);
	if (size < 1)
		return -1;
	// Read init tm[] image file.
	pIf->cmn.img.file = (uint8*)NULL;
	pIf->cmn.img.file_sz = 0;
	if (init_img_fname != NULL) {
		size = splash_load_file2buffer( init_img_fname, (void*)DSP_PIC_ADDR, MAX_IMAGE_FILE_SIZE);
		if (size > 0) {
			pIf->cmn.img.file = (uint8*)DSP_PIC_ADDR;
			pIf->cmn.img.file_sz = size;
		}
	}
	pIf->cmn.img.type = 0;	// Don't care
	// Read waveforms file.
	size = splash_load_file2buffer( "wvf.bin", (void*)DSP_WVF_ADDR, MAX_WAVEFORM_FILE_SIZE);
	if (size < 1)
		return -1;
	pIf->cmn.wvf.file = (uint8*)DSP_WVF_ADDR;
	pIf->cmn.wvf.file_sz = size;
	pIf->cmn.wvf.type = WVF_FILE_TYPE_EINK;
	// Read screen config file.
	size = splash_load_file2buffer( "cfg.bin", pIf->screen.a, sizeof(pIf->screen.a));
	if (size < 1)
		return -1;

	return 0;
}

/*
 * Power Reset
 */
int papirus_init(const char *init_img_fname)
{
	volatile struct epd_splash_cfg_arm* pIf = (struct epd_splash_cfg_arm*)DSP_IF_ADDR;
	int err_value = 0;

#if !defined(CONFIG_OMAP3_BEAGLE)
	DECLARE_GLOBAL_DATA_PTR;
#if defined(CONFIG_3621GOSSAMER)
	/* CPLD reset */
	if (!omap_request_gpio(CPLD_RESET)) {
		omap_set_gpio_dataout(CPLD_RESET, 0);
		omap_set_gpio_direction(CPLD_RESET, 0);
	}
	else {
		printf("%s: unable to get CPLD reset gpio (%d)\n", __func__, CPLD_RESET);
	}
#endif

	//Enable CPLD power
	if (!omap_request_gpio(EN_CPLD_POWER)) {
		omap_set_gpio_dataout(EN_CPLD_POWER, 1); //GPIO 85 -> 1
		omap_set_gpio_direction(EN_CPLD_POWER, 0);
	}
#if defined(CONFIG_3621GOSSAMER)
	/* CPLD reset */
		omap_set_gpio_dataout(CPLD_RESET, 1);
#endif
	//EPD wakeup
	if (!omap_request_gpio(EPD_WAKEUP)) {
		omap_set_gpio_dataout(EPD_WAKEUP, 0);   //GPIO 87 -> 0
		omap_set_gpio_direction(EPD_WAKEUP, 0);

		udelay(WAKEUP_DELAY); //Wait

		omap_set_gpio_dataout(EPD_WAKEUP, 1);   //GPIO 87 -> 1
	}
	else {
		printf("%s: unable to get Papyrus wakeup GPIO (%d)\n", __func__, EPD_WAKEUP);
	}

	//Setup current I2C bus
	if(i2c_set_bus_num(PAPYRUS_I2C_BUS))  //set current i2c bus to i2c_3 bus
		goto papirus_init_err;

    if (PAPYRUS_BASE_ADDR == 0x00) {
#if defined(CONFIG_3621GOSSAMER)
		switch (gd->bd->bi_board_revision) {
			case BOARD_GOSSAMER_REV_EVT2:
				PAPYRUS_BASE_ADDR = PAPYRUS2_BASE_ADDR_EVT2;
				break;
            case BOARD_GOSSAMER_REV_EVT1C:
			case BOARD_GOSSAMER_REV_EVTPRE1C:
               	PAPYRUS_BASE_ADDR = PAPYRUS2_BASE_ADDR_EVT1C;
				break;
			default: /* EVT1a and EVT3, DVT, PVT... */
               	PAPYRUS_BASE_ADDR = PAPYRUS1_BASE_ADDR;
				break;
		}
#elif defined(CONFIG_3630EDP1) || defined (CONFIG_3621EDP1)
	PAPYRUS_BASE_ADDR = PAPYRUS1_BASE_ADDR;
#endif
	}

#if defined(CONFIG_3621GOSSAMER)
		switch (gd->bd->bi_board_revision) {
			case BOARD_GOSSAMER_REV_EVT2:
           	case BOARD_GOSSAMER_REV_EVT1C:
			case BOARD_GOSSAMER_REV_EVTPRE1C:
				if (papyrus2_init())
					goto papirus_init_err;
				break;
			default: /* EVT1a and EVT3, DVT, PVT... */
				if (papyrus1_init())
					goto papirus_init_err;
				break;
		}
#elif defined(CONFIG_3630EDP1) || defined (CONFIG_3621EDP1)
	if (papyrus1_init()) {
		goto papirus_init_err;
	}
#endif

#endif	/// !defined(CONFIG_OMAP3_BEAGLE)
	if (read_dsp_fw(init_img_fname, pIf)) {
		goto papirus_init_err;
	}
	pIf->cmn.temperature = 0;	/// Set some defined value.

#if defined(CONFIG_3621GOSSAMER)
	switch (gd->bd->bi_board_revision) {
			case BOARD_GOSSAMER_REV_EVT2:
           	case BOARD_GOSSAMER_REV_EVT1C:
			case BOARD_GOSSAMER_REV_EVTPRE1C:
				if (papyrus2_pic_voltage_enable(pIf))
					goto papirus_init_err;
				break;
			default: /* EVT1a and EVT3, DVT, PVT... */
				if (papyrus1_pic_voltage_enable(pIf))
					goto papirus_init_err;
				break;
		}
#elif defined(CONFIG_3630EDP1) || defined (CONFIG_3621EDP1)
	if (papyrus1_pic_voltage_enable()) {
		goto papirus_init_err;
	}
#endif

papirus_init_exit:
	// Return to the default No.
	i2c_set_bus_num(0);
	papirus_power_on = 1;
	return err_value;

papirus_init_err:
	err_deactivate();
	err_value = 1;
	goto papirus_init_exit;
}
