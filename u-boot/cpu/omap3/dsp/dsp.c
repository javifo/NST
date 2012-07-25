/*
 * (C) Copyright 2008 - 2009 Texas Instruments.
 *
 * (C) Copyright 2010
 * MM solutions, <www.mm-sol.com>
 * Nikolay Nikolov, <nnikolov@mm-sol.com>
 *
 * Interface functions for fast splash screen drawing by means of the DSP.
 *
 */

// KNOWN TARGETS : CONFIG_OMAP3_BEAGLE. CONFIG_3630EDP1, CONFIG_3621EDP1, CONFIG_3621GOSSAMER.

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clocks.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mux.h>

#include <mmc.h>
#include <fat.h>

#include <twl4030.h>

#include "dsp_cmn.h"
#include "gossamer_splash_interface.h"
#include "dsp.h"
#include "papirus.h"
#include "epd_splash_mbx.h"

#if defined(SPLASH_SCREEN_DEBUG)
	#define print_info(fmt,args...)	printf ("-I-DSP-: " fmt "\n", ##args)
	#define print_warn(fmt,args...)	printf ("-W-DSP-: " fmt "\n", ##args)
#else
	#define print_info(fmt,args...)
	#define print_warn(fmt,args...)
#endif

#if !defined(DSP_RETRY_COUNT)
#define DSP_RETRY_COUNT 100
#endif

static void dsp_print_error_reason(void);

struct splash_prcm {
	u32 fclken_iva2;	/* 0x00 */
	u32 clken_pll_iva2;	/* 0x04 */
	u8 res1[0x18];
	u32 idlest_iva2;		/* 0x20 */
	u32 idlest_pll_iva2;	/* 0x24 */
	u32 res2a[3];
	u32 autoidle_pll_iva2;
	u32 res2b[2];
	u32 clksel1_pll_iva2 ;	/* 0x40 */
	u32 clksel2_pll_iva2;	/* 0x44 */
	u32 clkstctrl_iva2;		/* 0x48 */
	u32 clkstst_iva2;		/* 0x4C */
	u8 res3[0x8b4];
	u32 clken_pll_mpu;	/* 0x904 */
	u8 res4[0x1c];
	u32 idlest_pll_mpu;	/* 0x924 */
	u8 res5[0x18];
	u32 clksel1_pll_mpu;	/* 0x940 */
	u32 clksel2_pll_mpu;	/* 0x944 */
	u8 res6[0xb8];
	u32 fclken1_core;	/* 0xa00 */
	u8 res7[0xc];
	u32 iclken1_core;	/* 0xa10 */
	u32 iclken2_core;	/* 0xa14 */
	u8 res8[0x28];
	u32 clksel_core;	/* 0xa40 */
	u8 res9[0xbc];
	u32 fclken_gfx;		/* 0xb00 */
	u8 res10[0xc];
	u32 iclken_gfx;		/* 0xb10 */
	u8 res11[0x2c];
	u32 clksel_gfx;		/* 0xb40 */
	u8 res12[0xbc];
	u32 fclken_wkup;	/* 0xc00 */
	u8 res13[0xc];
	u32 iclken_wkup;	/* 0xc10 */
	u8 res14[0xc];
	u32 idlest_wkup;	/* 0xc20 */
	u8 res15[0x1c];
	u32 clksel_wkup;	/* 0xc40 */
	u8 res16[0xbc];
	u32 clken_pll;		/* 0xd00 */
	u8 res17[0x1c];
	u32 idlest_ckgen;	/* 0xd20 */
	u8 res18[0x1c];
	u32 clksel1_pll;	/* 0xd40 */
	u32 clksel2_pll;	/* 0xd44 */
	u32 clksel3_pll;	/* 0xd48 */
	u8 res19[0xb4];
	u32 fclken_dss;		/* 0xe00 */
	u8 res20[0xc];
	u32 iclken_dss;		/* 0xe10 */
	u8 res21[0x2c];
	u32 clksel_dss;		/* 0xe40 */
	u8 res22[0xbc];
	u32 fclken_cam;		/* 0xf00 */
	u8 res23[0xc];
	u32 iclken_cam;		/* 0xf10 */
	u8 res24[0x2c];
	u32 clksel_cam;		/* 0xf40 */
	u8 res25[0xbc];
	u32 fclken_per;		/* 0x1000 */
	u8 res26[0xc];
	u32 iclken_per;		/* 0x1010 */
	u8 res27[0x2c];
	u32 clksel_per;		/* 0x1040 */
	u8 res28[0xfc];
	u32 clksel1_emu;	/* 0x1140 */
};

struct splash_prm {
	u8	res1[0x50];
	u32	rm_rstctrl_iva2;	/* 0x050 */
	u8	res2[0x04];
	u32	rm_rstst_iva2;		/* 0x058 */
	u8	res3[0x6c];
	u32	pm_wkdep_iva2;		/* 0x0c8 */
	u8	res4[0x14];
	u32	pm_pwstctrl_iva2;	/* 0x0e0 */
	u32	pm_pwstst_iva2;		/* 0x0e4 */
	u32	pm_prepwstst_iva2;	/* 0x0e8 */
	u8	res5[0x0c];
	u32	prm_irqstatus_iva2;	/* 0x0f8 */
	u32 prm_irqenable_iva2;	/* 0x0fc */
};

#define IVA2_SW_RST1	(1<<8)
#define IVA2_SW_RST2	(1<<9)

#define PRCM_BASE	0x48004000
#define PRM_BASE	0x48306000

// Define registers needed

// Define bit-masks and bit-fields
#define SPLASH_FCK_IVA2_ON	(1<<0)
#define SPLASH_FCK_IVA2_OFF	(0<<0)
#define SPLASH_AUTO_IVA2_DPLL_ON	(1<<0)
#define SPLASH_AUTO_IVA2_DPLL_OFF	(0<<0)

typedef enum {
	EN_IVA2_DPLL_STOP	= 1,
	EN_IVA2_DPLL_BYPASS	= 5,
	EN_IVA2_DPLL_LOCK	= 7
} EN_IVA2_DPLL_enum_t;
#define SPLASH_ST_IVA2_CLK	(1<<0)

typedef struct {
	struct {
		u32	fclken_iva2;
		u32 autoidle_pll_iva2;
	} prcm;
} regs_restore_t;
static regs_restore_t regs_restore;
static int dsp_on = 0;	// It will guard actually access to the DSP int.mem.

/*****************************************************************
 * splash_sr32 - clear & set a value in a bit range for a 32 bit address
 *****************************************************************/
void splash_sr32(volatile u32 * const addr, const u32 start_bit, const u32 num_bits, const u32 value)
{
	u32 tmp, msk;

	msk = (1 << num_bits) - 1;
	tmp = (*addr) & ~(msk << start_bit);
	*addr = tmp | (value << start_bit);
}

/*********************************************************************
 * splash_wait_on_value() - common routine to allow waiting for changes in
 *   volatile regs.
 *********************************************************************/
u32 splash_wait_on_value(const u32 read_bit_mask, const u32 match_value, volatile u32 *read_addr, const u32 bound)
{
	u32 i = 0, val;
	do {
		++i;
		val = (*read_addr) & read_bit_mask;
		if (val == match_value)
			return(1);
		if (i==bound)
			return(0);
	} while (1);
}

#if defined(CONFIG_DSP_MEASURMENT)
	volatile u32* gpio_141_out_reg = 0x4905603C;
	int pulse;

void init_gio141(void)
{
volatile unsigned long buf;
///	////////////	__raw_writew(4, OMAP34XX_CTRL_BASE + CONTROL_PADCONF_McBSP3_DR);

	buf = *(volatile unsigned long*)0x48005000;		// CM_FCLKEN_PER
	buf |= (1<<16);
	__raw_writew(buf, 0x48005000);
	buf = *(volatile unsigned long*)0x48005010;		// CM_ICLKEN_PER
	buf |= (1<<16);
	__raw_writew(buf, 0x48005010);

	buf = *(volatile unsigned long*)0x49056034;		// gpio 141 OE reg
	buf &= (~(1<<(141-128)));
	__raw_writew(buf, 0x49056034);				// gpio 141
}
#endif

// Function to send command to the DSP. It's better encapsulated to add
// in the future mailbox signaling between ARM & DSP.
// -- Also keep it visible for tests. Then make stsic.
// Return	0: cmd not sent;
//			1: cmd sent;
#if defined(_SPLASH_SCREEN_TEST_)
int dsp_send_cmd(DSP_STATUS_t cmd)
#else
static int dsp_send_cmd(DSP_STATUS_t cmd)
#endif
{
	volatile struct epd_splash_cfg_arm* pIf = (struct epd_splash_cfg_arm*)DSP_IF_ADDR;

	if (!dsp_on) {
		printf("ERROR: dsp_send_cmd() not sent: DSP is OFF.\n");
		return (0);
	}
	// Just for clearness - DSP guards itself.
	if (cmd <= DSP_STCMD_base || cmd >= DSP_STCMD_end) {
		printf("ERROR: dsp_send_cmd() not sent: cmd = 0x%.8X is INVALID.\n", cmd);
		return (0);
	}
	if (pIf->cmn.dsp_err > DSP_ERROR_FATAL_2 && cmd != DSP_STCMD_STOP) {
		printf("ERROR: dsp_send_cmd() not sent: ");
		dsp_print_error_reason();
		return (0);
	}
	if (pIf->cmn.dsp_st != DSP_ST_ALL_DONE) {
		printf("ERROR: dsp_send_cmd() not sent: ");
		dsp_print_error_reason();
		return (0);
	}
	pIf->cmn.dsp_st = cmd;
	if (!epd_splash_mbx_write( MBX_MB_0, 0)) {
		printf("ERROR: dsp_send_cmd() not sent: Mailbox BUSY.\n");
		pIf->cmn.dsp_st = DSP_ST_ALL_DONE;	// Restore dsp_st so we can retry later.
		return (0);
	}
	return (1);
}

// Function to display a picture with given waveform id.
DISP_ERR_t dsp_display_pic_file_by_wvfid(const char * fname, const int timeout_ms, const WVFID_t wvfid)
{
	volatile struct epd_splash_cfg_arm* pIf = (struct epd_splash_cfg_arm*)DSP_IF_ADDR;
	long size;
	int i;

	if (dsp_on == 0) {
		printf("ERROR: dsp_display_pic_file() - DSP is OFF.\n");
		return (DISP_ERR_NOTSTARTED);///(-2);	/// e.g. -2
	}
	for (i = 0; pIf->cmn.dsp_st != DSP_ST_ALL_DONE && i < timeout_ms; i += 1) {
		udelay(UDELAY_1ms);
	}
	if(pIf->cmn.dsp_st == DSP_ST_ALL_DONE) { // Check DSP has comleted all previous job an is safely waiting.
		// Read picture file.
		size = splash_load_file2buffer( fname, (void*)DSP_PIC_ADDR, MAX_IMAGE_FILE_SIZE);
		if (size < 1)
			return (DISP_ERR_NOTSTARTED);///(-1);
		pIf->cmn.img.file = (uint8*)DSP_PIC_ADDR;
		pIf->cmn.img.file_sz = size;
		pIf->cmn.img.type = 0;	// Don't care
		pIf->cmn.vio_1 = wvfid;
		// Get the DSP to display the new picture.
		if (!dsp_send_cmd(DSP_STCMD_DRAW_PIC)) {
			return (DISP_ERR_NOTSTARTED);///(-3);	/// e.g. -3
		}
		for (i = 0; pIf->cmn.dsp_st == DSP_STCMD_DRAW_PIC && i < 10; i += 1) {
			udelay(UDELAY_1ms);
		}
		if (pIf->cmn.dsp_st == DSP_STCMD_DRAW_PIC) {
			printf("ERROR: DSP failed to run new image display\n");
			return (DISP_ERR_NOTSTARTED);/// ?? ///(-3);
		} else {
			;///return (0);	// Success to start.
		}
	} else {
		printf("ERROR: DSP not completed previous job: ");
		dsp_print_error_reason();
		return (DISP_ERR_NOTSTARTED);///(-2);
	}
	for (i = 0; pIf->cmn.dsp_st != DSP_ST_ALL_DONE && i < timeout_ms; i += 1) {
		udelay(UDELAY_1ms);
	}
	if(pIf->cmn.dsp_st == DSP_ST_ALL_DONE) {
		///(0);	// Success DSP completed.
		if (pIf->cmn.dsp_err == DSP_ERROR_OK) {
			print_info("dsp_display_pic_file() SUCCESS");
			return (DISP_ERR_OK);
		} else {
			printf("ERROR: DSP new image display: ");
			dsp_print_error_reason();
			return (DISP_ERR_NOTSTARTED);/// /// /// /// ///
		}
	} else {
		printf("ERROR: DSP failed to complete new image display within %dms.\n", timeout_ms);
		dsp_print_error_reason();
		return (DISP_ERR_NOTCOMPLETED);///(-3);
	}
}//	***/

/* Functions to read and write from TWL4030 */
static int splash_twl4030_i2c_write_u8(u8 chip_no, u8 val, u8 reg)
{
	print_info("i2c-splash_twl4030_i2c_write_u8(0x%.2X, 0x%.2X, 0x%.2X)",
		chip_no, val, reg);

	return i2c_write(chip_no, reg, 1, &val, 1);
}

static int splash_twl4030_i2c_read_u8(u8 chip_no, u8 *val, u8 reg)
{
	int retval = i2c_read(chip_no, reg, 1, val, 1);
	print_info("i2c-splash_twl4030_i2c_read_u8(0x%.2X, 0x%.2X, 0x%.2X)",
		chip_no, *val, reg);
	return retval;
}

// Set to ON VPLL2 power.
static int turn_on_twl4030_vpll2(void)
{
	int retval;
#define TWL4030_PM_RECEIVER_VPLL2_DEV_GRP		0x8E
	#define DEV_GRP_ALL		0xE0
#define TWL4030_PM_RECEIVER_VPLL2_DEDICATED		0x91
	#define VPLL2_VSEL_18	0x05
	unsigned char reg_VPLL2_DEV_GRP = TWL4030_PM_RECEIVER_VPLL2_DEV_GRP;//0x8E;//
	unsigned char reg_VPLL2_DEDICATED = TWL4030_PM_RECEIVER_VPLL2_DEDICATED;//0x91;//
	unsigned char byte;

//Setup current I2C bus
	i2c_set_bus_num(0);
	//
	print_info("i2c-set VPLL2 to 1.8V-start");
	byte = VPLL2_VSEL_18;
	retval = splash_twl4030_i2c_write_u8(TWL4030_CHIP_PM_RECEIVER, byte, reg_VPLL2_DEDICATED);
	byte = DEV_GRP_ALL;
	if (retval)
		return retval;
	retval = splash_twl4030_i2c_write_u8(TWL4030_CHIP_PM_RECEIVER, byte, reg_VPLL2_DEV_GRP);
	// Just to check
	if (retval)
		return retval;
	retval = splash_twl4030_i2c_read_u8(TWL4030_CHIP_PM_RECEIVER, &byte, reg_VPLL2_DEV_GRP);
	if (retval)
		return retval;
	retval = splash_twl4030_i2c_read_u8(TWL4030_CHIP_PM_RECEIVER, &byte, reg_VPLL2_DEDICATED);
	print_info("i2c-set VPLL2 to 1.8V-end");
	return 0;
}

// This will work only for OMAP36XX
static void dsp_clocks_setup(void)
{
	volatile struct splash_prcm *prcm_base  = (struct splash_prcm *)PRCM_BASE;

	splash_sr32(&prcm_base->clken_pll_iva2, 0, 3, EN_IVA2_DPLL_STOP);
	splash_wait_on_value(SPLASH_ST_IVA2_CLK, 0, &prcm_base->idlest_pll_iva2, LDELAY);

	/* IVA bypass clock set to CORECLK/2=(100) at OPP1 */
	splash_sr32(&prcm_base->clksel1_pll_iva2, 19, 3, 2);	/* set CLK_SRC: DPLL2_FCLK is CORE.CLK / 2 */
	splash_sr32(&prcm_base->clksel1_pll_iva2, 8, 11, 35/* iva->m : mul 35 times */);
	splash_sr32(&prcm_base->clksel1_pll_iva2, 0,  7, 0/* iva->n : div (0+1) times */);
	splash_sr32(&prcm_base->clksel2_pll_iva2, 0,  5, 1/*iva->m2 : DPLL2 CLKOUTX2 / 1 */);

	/* FREQSEL */
	splash_sr32(&prcm_base->clken_pll_iva2, 4, 4, 0xD);
	splash_sr32(&prcm_base->clken_pll_iva2, 0, 3, EN_IVA2_DPLL_LOCK);	/* lock mode */

	splash_wait_on_value(SPLASH_ST_IVA2_CLK, 1, &prcm_base->idlest_pll_iva2, LDELAY);
}

//------------------------------------------------------------------------------
static void dsp_regs_setup(void)
{
	volatile struct splash_prcm *prcm_base  = (struct splash_prcm *)PRCM_BASE;

//	print_info("Set DSP clock");
//	dsp_clocks_setup();		// Implement dsp_clocks_restore() if used.
	regs_restore.prcm.fclken_iva2 = prcm_base->fclken_iva2;
	if (SPLASH_FCK_IVA2_ON & regs_restore.prcm.fclken_iva2) {
		print_info("IVA2 SS clock already enabled by U-boot.");
	} else {
		print_info("Enable IVA2 SS clock.");
		splash_sr32( &prcm_base->fclken_iva2, 0, 32, SPLASH_FCK_IVA2_ON);
	}
	print_info("fclken_iva2: old = 0x%.8X, new = 0x%.8X", regs_restore.prcm.fclken_iva2, prcm_base->fclken_iva2);
	print_info("clken_pll_iva2 = 0x%.8X", prcm_base->clken_pll_iva2);
	print_info("idlest_pll_iva2 = 0x%.8X", prcm_base->idlest_pll_iva2);
	print_info("clksel1_pll_iva2 = 0x%.8X", prcm_base->clksel1_pll_iva2);
	print_info("clksel2_pll_iva2 = 0x%.8X", prcm_base->clksel2_pll_iva2);
	//
	regs_restore.prcm.autoidle_pll_iva2 = prcm_base->autoidle_pll_iva2;
	prcm_base->autoidle_pll_iva2 = SPLASH_AUTO_IVA2_DPLL_ON;
	print_info("autoidle_pll_iva2: old = 0x%.8X, new = 0x%.8X", regs_restore.prcm.autoidle_pll_iva2, prcm_base->autoidle_pll_iva2);
}

//------------------------------------------------------------------------------
static void dsp_regs_restore(void)
{
	volatile struct splash_prcm *prcm_base  = (struct splash_prcm *)PRCM_BASE;
	volatile u32 dummy_read_reg;

	prcm_base->autoidle_pll_iva2 = regs_restore.prcm.autoidle_pll_iva2;
	dummy_read_reg = prcm_base->autoidle_pll_iva2;
	print_info("autoidle_pll_iva2 = 0x%.8X", dummy_read_reg);
	prcm_base->fclken_iva2 = regs_restore.prcm.fclken_iva2;
	dummy_read_reg = prcm_base->fclken_iva2;
	print_info("fclken_iva2 = 0x%.8X", dummy_read_reg);
//	dsp_clocks_restore();	// Implement if used dsp_clocks_setup()
}

#define	_SPLASH_ARMFW_VERSION_STRING_	"splash arm fw v.1.03 (ANYWVF)"

//------------------------------------------------------------------------------
int dsp_start (const char *init_img_fname)
{
	int retval;
	volatile u32 *control_iva2_bootaddr = (volatile u32 *)0x48002400;
	volatile u32 *control_iva2_bootmod  = (volatile u32 *)0x48002404;
	volatile struct splash_prm *prm_base = (struct splash_prm *)PRM_BASE;
	volatile struct epd_splash_cfg_arm* pIf = (struct epd_splash_cfg_arm*)DSP_IF_ADDR;

	int dev =0;
	int part=1;

	if (dsp_on == 1)
		return (0);
	// Print version
	printf("=== " _SPLASH_ARMFW_VERSION_STRING_ " ===\n");
#if defined(CONFIG_DSP_MEASURMENT)
	for (pulse = 0; pulse < 10; pulse++) {
		(*gpio_141_out_reg) ^= (1<<(141-128));
		udelay(1000);
	}
	(*gpio_141_out_reg) &= (~(1<<(141-128)));
#endif

	/* -1. Change DSP clocks */
//
//	print_info("Set DSP clock");
//	dsp_clocks();
	/* 0. Software enables the IVA2.2 subsystem clock - 4.5.9.3 in the TRM */
	dsp_regs_setup();
	/* 1. Initialize MSG system if needed */
#if 1//Use MSG system
	epd_splash_mbx_init();	///print_info("Don't use MSG system");
#endif
	/* 2. Write Boot Address & Boot mode */
	print_info("Write Boot Address & Boot mode");
	{
		*control_iva2_bootaddr = DSP_BOOT_ADDR;///0x007E0000;///
		*control_iva2_bootmod = 0;
	}
	/* 3.a. Release MMU reset*/
	print_info("Release MMU reset");
	splash_sr32( &prm_base->rm_rstctrl_iva2, 1, 1, 0);	// RST2_IVA2 bit = 0
	/* 3.b. Wait for MMU reset deasserted */
	print_info("Wait for MMU reset deasserted");
	if (splash_wait_on_value(IVA2_SW_RST2, IVA2_SW_RST2, &prm_base->rm_rstst_iva2, LDELAY)) {
		print_info("MMU reset deasserted");
	} else {
		printf("ERROR: MMU reset timed out");
		return(1);
	}
	/* 3.c. Program and enable DSP MMU */
#if 1// Do not program DSP MMU.
	print_info("Do not program DSP MMU");
#endif
#if defined(CONFIG_3621GOSSAMER)
	extern int get_mmcbootdev(void);
	dev = get_mmcbootdev();
#endif
	/* 4. Downloading Code through DSP HPI port - this actualy done inside papirus_init() bellow */
#if defined(CONFIG_OMAP3_BEAGLE)
	retval = mmc_legacy_init(dev);
	if (retval) {
		printf ("\n** ******** Unable to use %s %d:%d for fatload ** **********\n", "mmc", dev, part);
	}
#else
	retval = mmc_init(dev);
	if (retval) {
		printf("ERROR: No MMC card found!\n");
		return (1);
	} else {
		print_info("MMC%d Initalization OK\n", dev+1);
	}
	block_dev_desc_t *dev_desc;
	dev_desc = get_dev("mmc", dev);
	if (dev_desc == NULL) {
		printf ("\n** ****** Invalid boot device ** *********\n");
	}
	if (fat_register_device(dev_desc, part) != 0 ) {
		printf ("\n** ******** Unable to use %s %d:%d for fatload ** **********\n", "mmc", dev, part);
	}
#endif
	/* -  Power-up EPD and logic for it. Also VPLL2 */
	retval = papirus_init(init_img_fname);
	print_info("papirus_init() returned [%d]\n", retval);
	if (retval)		return(1);
	retval = turn_on_twl4030_vpll2();
	print_info("turn_on_twl4030_vpll2() returned [%d]\n", retval);
	if (retval)		return(1);

	/* 5. Release DSP from reset */
	pIf->cmn.dsp_st = DSP_ST_RESET;
	pIf->cmn.dsp_err = DSP_ERROR_OK;
#if defined(V_SCLK)
	pIf->cmn.SYS_CLK_KHZ = V_SCLK/1000;
#else
	pIf->cmn.SYS_CLK_KHZ = 26*1000;
#endif
	print_info("Release DSP from reset");
	splash_sr32( &prm_base->rm_rstctrl_iva2, 0, 1, 0);	// RST1_IVA2 bit = 0
	print_info("Wait for DSP reset deasserted");
	if (splash_wait_on_value(IVA2_SW_RST1, IVA2_SW_RST1, &prm_base->rm_rstst_iva2, LDELAY)) {
		print_info("DSP reset deasserted");
	} else {
		printf("ERROR: DSP reset timed out");
		return(1);
	}
	dsp_on = 1;
	print_info("Params: SYSCLK[kHz] = %d, TEMPERATURE = %d", pIf->cmn.SYS_CLK_KHZ, pIf->cmn.temperature);
{	// Get DSP code version.
	int timeout = 200, timeout_step = 10;
	for ( ; timeout > 0; timeout -= timeout_step) {
		if (pIf->cmn.dsp_st != DSP_ST_RESET)
			break;
		udelay(timeout_step*1000);
	}
	if (timeout > 0) {
		printf("=== splash dsp fw v.%d.%.2d %c ===\n",
			(pIf->cmn.vio_1>>24)&0xff, (pIf->cmn.vio_1>>16)&0xff, (pIf->cmn.vio_1>>0)&0xff);
	} else {
		printf("splash dsp fw timed out\n");
	}
}
	return(0);	// Everything above OK
}

//------------------------------------------------------------------------------
#if defined(_SPLASH_SCREEN_TEST_)
void dsp_print_state(void)
#else
static void dsp_print_state(void)
#endif
{
	volatile struct epd_splash_cfg_arm* pIf = (struct epd_splash_cfg_arm*)DSP_IF_ADDR;
	char *msgst, *msgerr;
	int unknownst = 0, unknownerr = 0;

	if (!dsp_on) {
		printf("ERROR dsp_print_state() - DSP is OFF\n");
		return;
	}
	switch(pIf->cmn.dsp_st) {
	case DSP_ST_RESET:
		msgst = "STILL NOT RUNNING !!! ...";
		break;
	case DSP_ST_RUN:
		msgst = "RUN ...";
		break;
	case DSP_ST_MEMINIT:
		msgst = "INITIALIZING MEMORY ...";
		break;
	case DSP_ST_HWINIT:
		msgst = "INITIALIZING HW ...";
		break;
	case DSP_ST_CLEARSCREEN:
		msgst = "CLEARS SCREEN ...";
		break;
	case DSP_ST_PICTURE:
		msgst = "DRAWING IMAGE ...";
		break;
	case DSP_ST_PICTURE_DONE:
		msgst = "SPLASH SCREEN COMPLETED ...";
		break;
	case DSP_ST_PLAY_IDLES:
		msgst = "PLAYING IDLES ...";
		break;
	case DSP_ST_ALL_DONE:
		msgst = "COMPLETED";
		break;
	case DSP_ST_STOPPED:
		msgst = "STOPPED";
		break;
	default:
		unknownst = 1;
		msgst = "UNKNOWN !!! ...";
	}
	///
	switch(pIf->cmn.dsp_err) {
	case DSP_ERROR_OK:
		msgerr = "OK";
		break;
	case DSP_ERROR_CFG_SCREEN:
		msgerr = "CFG SCREEN";
		break;
	case DSP_ERROR_CFG_WVF:
		msgerr = "CFG WAVEFORM";
		break;
	case DSP_ERROR_MEMALLOC:
		msgerr = "MEMALLOC";
		break;
	case DSP_ERROR_DSS_RESET:
		msgerr = "DSS RESET";
		break;
	case DSP_ERROR_DISPC_RESET:
		msgerr = "DISPC RESET";
		break;
	case DSP_ERROR_DSI_RESET:
		msgerr = "DSI RESET";
		break;
	case DSP_ERROR_DSS_MALLOC:
		msgerr = "DSS MALLOC";
		break;
	case DSP_ERROR_DSI_PWR:
		msgerr = "DSI PWR";
		break;
	case DSP_ERROR_DSI_PLL_BYPASS:
		msgerr = "DSI PLL BYPASS";
		break;
	case DSP_ERROR_DSI_PLL_LOCK:
		msgerr = "DSI PLL LOCK";
		break;
	case DSP_ERROR_DSI_PLL_CLOCK:
		msgerr = "DSI PLL CLOCK";
		break;
	case DSP_ERROR_SUBFPROC:
		msgerr = "SUBF PROC";
		break;
	case DSP_ERROR_SUBFQ:
		msgerr = "SUBF QUEUE";
		break;
	case DSP_ERROR_IMGFMT:
		msgerr = "IMAGE FORMAT";
		break;
	case DSP_ERROR_WVFID:
		msgerr = "UNSUPPORTED WAVEFORM";
		break;
	case DSP_ERROR_BADARG:
		msgerr = "BAD ARG";
		break;
	default:
		unknownerr = 1;
		msgerr = "UNKNOWN !!! ...";
	}
	if (unknownst) {
		printf("DSP state: %s, addr = 0x%.8X, val = 0x%.8X; ", msgst, (uint32)(&pIf->cmn.dsp_st), pIf->cmn.dsp_st);
	} else {
		printf("DSP state: %s; ", msgst);
	}
	if (unknownerr) {
		printf("DSP err: %s, addr = 0x%.8X, val = 0x%.8X;\n", msgerr, (uint32)(&pIf->cmn.dsp_err), pIf->cmn.dsp_err);
	} else {
		printf("DSP result: %s%s;\n", (pIf->cmn.dsp_err != DSP_ERROR_OK) ? "ERROR " : "", msgerr);
	}
#if defined(SPLASH_SCREEN_DEBUG)
{
	volatile struct splash_prcm *prcm_base  = (struct splash_prcm *)PRCM_BASE;
	print_info("idlest_iva2, idlest_pll_iva2, clkstst_iva2 = (%d,%d,%d)",
		prcm_base->idlest_iva2, prcm_base->idlest_pll_iva2, prcm_base->clkstst_iva2);
}
#endif
}

//------------------------------------------------------------------------------
int dsp_stop(const int force_stop)
{
	volatile struct splash_prm *prm_base = (struct splash_prm *)PRM_BASE;
	volatile struct epd_splash_cfg_arm* pIf = (struct epd_splash_cfg_arm*)DSP_IF_ADDR;
	int i;

	// Check this 1st to guard access to IF since it is in DSP internal mem.now
	// Once DSP MMU is under reset any access to DSP int.mem.will lock !
	if (dsp_on == 0) {
		// It will always go here if dsp_start() fails to complete to the end or already stopped.
		// It should be safe to call dsp_start() again however.
		return (0);
	}
	//
	dsp_print_state();
	if (pIf->cmn.dsp_st != DSP_ST_ALL_DONE && pIf->cmn.dsp_st != DSP_ST_STOPPED && force_stop == 0)
		return (1);
	//
	for (i = 0; pIf->cmn.dsp_st != DSP_ST_ALL_DONE && pIf->cmn.dsp_st != DSP_ST_STOPPED; i++) {
		printf("dsp_stop() waits for DSP. n = %d: ", i);
		dsp_print_error_reason();
		udelay(200*1000);
        if (i>DSP_RETRY_COUNT) {
            break;
        }
	}
	if (pIf->cmn.dsp_st == DSP_ST_ALL_DONE) {
		dsp_send_cmd(DSP_STCMD_STOP);
		udelay(20);	// Give DSP time to start executing STOP command.
		for (i = 0; pIf->cmn.dsp_st != DSP_ST_STOPPED; i++) {
			printf("dsp_stop() waits for DSP. n = %d: ", i);
			dsp_print_error_reason();
			udelay(200*1000);
            if (i>DSP_RETRY_COUNT) {
                break;
            }
		}
	}
	udelay(200);	// Give DSP time to step onto IDLE instruction.
	// Set IVA2 to reset state
	splash_sr32(&prm_base->rm_rstctrl_iva2, 0, 1, 1);	//RST1_IVA2 bit = 1
	pIf->cmn.dsp_st = DSP_ST_RESET;
	splash_sr32(&prm_base->rm_rstctrl_iva2, 1, 1, 1);	//RST2_IVA2 bit = 1
	//
	dsp_on = 0;
	dsp_regs_restore();	// Restore modified registers (sets also fclken_iva2 to OFF if it was !).
	epd_splash_mbx_restore();
	//
	print_info("DSP Reset OK.");
	dis_power_off();
	return (0);
}

//------------------------------------------------------------------------------
static void dsp_print_error_reason(void)
{
	dsp_print_state();
}

// Function to display a picture
DISP_ERR_t dsp_display_pic_file(const char * fname, const int timeout_ms)
{
	return (dsp_display_pic_file_by_wvfid(fname, timeout_ms, WVFID_AUTO));
}

int is_dsp_on(void)
{
    return (dsp_on)?1:0;
}
