/*
 * (C) Copyright 
 * Texas Instruments, <www.ti.com>
 *
 * (C) Copyright 2010, MM solutions, <www.mm-sol.com>
 * Boris Todorov, <btodorov@mm-sol.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <asm/arch/cpu.h>
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
#include <twl4030.h>
#include <tps65921.h>
#include <bq27520.h>
#include <bq24073.h>
#include <tps65921_keypad.h>
#include <rtc.h>

#include "../cpu/omap3/dsp/dsp.h"
#include "tps65185.h"


#if defined(CONFIG_DSP_SPLASH_SCREEN) && !defined(CFG_TFT_DISPLAY)
#include "../cpu/omap3/dsp/dsp.h"
#endif

#include <ns16550.h>

static int power_button_boot;	// state of power button at boot
static int first_boot;

extern int select_bus(int, int);
extern block_dev_desc_t *get_dev (char* ifname, int dev);
extern int gpio_pin_read(u32);
extern int gpio_pin_write(u32, int);

int get_boot_type(void);
void v7_flush_dcache_all(int, int);
void l2cache_enable(void);
void setup_auxcr(int, int);
void eth_init(void *);
u32 get_board_rev(void);
void set_muxconf_regs_late(void);

#define MAKE_HW_BOARD_ID(id1, id2, id3) (id1 | (id2 << 1 ) | (id3 << 2 ))

/* charger defines */
#define GOSSAMER_BQ24074_CE_GPIO_EVTPRE1C	110
#define GOSSAMER_BQ24074_EN1_GPIO_EVTPRE1C	102
#define GOSSAMER_BQ24074_EN2_GPIO_EVTPRE1C	61

#define GOSSAMER_BQ24074_CE_GPIO_EVT1C	44
#define GOSSAMER_BQ24074_EN1_GPIO_EVT1C	45
#define GOSSAMER_BQ24074_EN2_GPIO_EVT1C	61

#define TPS65921_I2C_BUS 			0

/* charger related */
#define GOSSAMER_MIN_DEAD_SOC		(6) 

#define GOSSAMER_MIN_ZERO_SOC_UVOLTAGE (3000)
#define GOSSAMER_MIN_DEAD_UVOLTAGE	(3560)	/* 240 mV drop from papyrus1 3300mV min for touch */
#define GOSSAMER_MIN_DEAD_UVOLTAGE_PIC	(3400)	/* mV */
#define GOSSAMER_MIN_DEAD_UVOLTAGE_PIC_DSP_ON	(3000)	/* mV */
#define GOSSAMER_MIN_DEAD_UVOLTAGE_PIC_SAFE	(3650)	/* mV */
//#define GOSSAMER_MIN_DEAD_BOOT_SOC_NONE	(10)

// Note gossamer should use lower thresholds for DEAD_BOOT_SOC  USB and WALL, original test case spec 15,12
#define GOSSAMER_MIN_DEAD_BOOT_SOC_USB	(15)	/* % as per the test case spec */
#define GOSSAMER_MIN_DEAD_BOOT_SOC_WALL	(12)	/* % as per the test case spec */

// This voltage is set higher than soc threshold for booting, back door in case gg error
#if HAS_BQ27520_BATTERY_MODEL
#define GOSSAMER_MIN_ALWAYS_BOOT_UVOLTAGE (4050)  /* mV */
#else
#define GOSSAMER_MIN_ALWAYS_BOOT_UVOLTAGE (3700)  /* mV */
#endif

#define GOSSAMER_CHARGE_IMAGES		(4)
#define GOSSAMER_PWR_BUTTON_GPIO	(14)
#define GOSSAMER_DRAW_TIMEOUT		(10000) /* ms */
#define GOSSAMER_WALL_CHARGE_MAX    (1000) /* mA */
#define GOSSAMER_USB_CHARGE_MAX     (500) /* mA */
#define GOSSAMER_USB_CHARGE_MIN     (100) /* mA */
#define GOSSAMER_VBUS_WAIT_COUNT_MAX (10)

/* GOSSAMER_MIN_CHARGER_CONTROL is the minimum threshold for allowing system to change power supply current*/
#define GOSSAMER_MIN_CHARGER_CONTROL_UVOLTAGE	(3300)	/* triton allows omap to boot at 3.3v...   */
#define GOSSAMER_MIN_CHARGER_CONTROL_UVOLTAGE_CHARGING  (3200) /* b3400073 start worst case cc at 3.1v, so allow some reserve */
#define GOSSAMER_MIN_CHARGER_CONTROL_SOC        (1)


/* GOSSAMER_MIN_CHARGER_CONTROL is the minimum threshold for allowing system to display picture*/
#define GOSSAMER_MIN_DEAD_UVOLTAGE_CHARGE_PIC_WALL (3400)
#define GOSSAMER_MIN_DEAD_UVOLTAGE_CHARGE_PIC	(3500)	/* mV */
#define GOSSAMER_MIN_DEAD_SOC_CHARGE_PIC        (1)     /* % soc */
#define GOSSAMER_MIN_DEAD_SOC_PIC               (2)

/*******************************************************
 * Routine: delay
 * Description: spinning delay to use before udelay works
 ******************************************************/
static inline void delay(unsigned long loops)
{
	__asm__ volatile ("1:\n" "subs %0, %1, #1\n"
			  "bne 1b":"=r" (loops):"0"(loops));
}

static const char * const arch_string(u32 mtype)
{
    switch (mtype) {
    case MACH_TYPE_OMAP3621_GOSSAMER:
        return "GOSSAMER";
    default:
        return "Unknown";
    }
}

static const char * const rev_string(u32 btype)
{
	switch (btype) {
		case BOARD_GOSSAMER_REV_EVT0:
			return "EVT0";
		case BOARD_GOSSAMER_REV_EVT1A:
			return "EVT1A";
		case BOARD_GOSSAMER_REV_EVTPRE1C:
			return "EVTPRE1C";
		case BOARD_GOSSAMER_REV_EVT1C:
			return "EVT1C";
		case BOARD_GOSSAMER_REV_EVT2:
			return "EVT2";
		case BOARD_GOSSAMER_REV_EVT3:
			return "EVT3";
		case BOARD_GOSSAMER_REV_DVT:
			return "DVT";
		default:
			return "Unknown";
	}
}

static inline int read_board_id(void)
{
    int bid;

    bid = MAKE_HW_BOARD_ID(gpio_pin_read(40),
				gpio_pin_read(41),
				gpio_pin_read(42));
    return bid;
}

/*****************************************
 * Routine: board_init
 * Description: Early hardware init.
 *****************************************/
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;
	int bid;

	gpmc_init();		/* in SRAM or SDRAM, finish GPMC */

	gd->bd->bi_arch_number = MACH_TYPE_OMAP3621_GOSSAMER; /* Linux mach id */
	bid = read_board_id();

	switch(bid) {
		case BOARD_GOSSAMER_REV_EVT0:
		case BOARD_GOSSAMER_REV_EVT1A:
			gd->bd->bi_board_revision = bid;
			break;
		case BOARD_GOSSAMER_REV_EVT1C:
		case BOARD_GOSSAMER_REV_EVTPRE1C:
		case BOARD_GOSSAMER_REV_EVT2:
		case BOARD_GOSSAMER_REV_EVT3:
		case BOARD_GOSSAMER_REV_DVT:
			set_muxconf_regs_late();
			gd->bd->bi_board_revision = bid;
			break;
	default:
			gd->bd->bi_arch_number = BOARD_GOSSAMER_REV_UNKNOWN;
	}

	// Update muxes according to board type
	update_mux(get_board_type(), get_mem_type());

	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100); /* boot param addr */

	return 0;
}

/*****************************************
 * Routine: secure_unlock
 * Description: Setup security registers for access
 * (GP Device only)
 *****************************************/
void secure_unlock_mem(void)
{
	/* Permission values for registers -Full fledged permissions to all */
	#define UNLOCK_1 0xFFFFFFFF
	#define UNLOCK_2 0x00000000
	#define UNLOCK_3 0x0000FFFF

	/* Protection Module Register Target APE (PM_RT)*/
	__raw_writel(UNLOCK_1, RT_REQ_INFO_PERMISSION_1);
	__raw_writel(UNLOCK_1, RT_READ_PERMISSION_0);
	__raw_writel(UNLOCK_1, RT_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, RT_ADDR_MATCH_1);

	__raw_writel(UNLOCK_3, GPMC_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, GPMC_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_3, OCM_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, OCM_WRITE_PERMISSION_0);
	__raw_writel(UNLOCK_2, OCM_ADDR_MATCH_2);

	/* IVA Changes */
	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_0);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_0);

	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_1);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_1);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_1);

	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_2);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_2);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_2);

	__raw_writel(UNLOCK_3, IVA2_REQ_INFO_PERMISSION_3);
	__raw_writel(UNLOCK_3, IVA2_READ_PERMISSION_3);
	__raw_writel(UNLOCK_3, IVA2_WRITE_PERMISSION_3);

	__raw_writel(UNLOCK_1, SMS_RG_ATT0); /* SDRC region 0 public */
}


/**********************************************************
 * Routine: secureworld_exit()
 * Description: If chip is EMU and boot type is external
 *		configure secure registers and exit secure world
 *  general use.
 ***********************************************************/
void secureworld_exit(void)
{
	unsigned long i;

	/* configrue non-secure access control register */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c1, 2":"=r" (i));
	/* enabling co-processor CP10 and CP11 accesses in NS world */
	__asm__ __volatile__("orr %0, %0, #0xC00":"=r"(i));
	/* allow allocation of locked TLBs and L2 lines in NS world */
	/* allow use of PLE registers in NS world also */
	__asm__ __volatile__("orr %0, %0, #0x70000":"=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c1, 2":"=r" (i));

	/* Enable ASA and IBE in ACR register */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 1":"=r" (i));
	__asm__ __volatile__("orr %0, %0, #0x50":"=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 1":"=r" (i));

	/* Exiting secure world */
	__asm__ __volatile__("mrc p15, 0, %0, c1, c1, 0":"=r" (i));
	__asm__ __volatile__("orr %0, %0, #0x31":"=r"(i));
	__asm__ __volatile__("mcr p15, 0, %0, c1, c1, 0":"=r" (i));
}

/**********************************************************
 * Routine: try_unlock_sram()
 * Description: If chip is GP/EMU(special) type, unlock the SRAM for
 *  general use.
 ***********************************************************/
void try_unlock_memory(void)
{
	int mode;
	int in_sdram = running_in_sdram();

	/* if GP device unlock device SRAM for general use */
	/* secure code breaks for Secure/Emulation device - HS/E/T*/
	mode = get_device_type();
	if (mode == GP_DEVICE) {
		secure_unlock_mem();
	}
	/* If device is EMU and boot is XIP external booting
	 * Unlock firewalls and disable L2 and put chip
	 * out of secure world
	 */
	/* Assuming memories are unlocked by the demon who put us in SDRAM */
	if ((mode <= EMU_DEVICE) && (get_boot_type() == 0x1F)
		&& (!in_sdram)) {
		secure_unlock_mem();
		secureworld_exit();
	}

	return;
}

/**********************************************************
 * Routine: s_init
 * Description: Does early system init of muxing and clocks.
 * - Called path is with SRAM stack.
 **********************************************************/
void s_init(void)
{
	int i;
	int external_boot = 0;

	watchdog_init();

	external_boot = (get_boot_type() == 0x1F) ? 1 : 0;
	/* Right now flushing at low MPU speed. Need to move after clock init */
	v7_flush_dcache_all(get_device_type(), external_boot);

	try_unlock_memory();

	if (cpu_is_3410()) {
		/* Lock down 6-ways in L2 cache so that effective size of L2 is 64K */
		__asm__ __volatile__("mov %0, #0xFC":"=r" (i));
		__asm__ __volatile__("mcr p15, 1, %0, c9, c0, 0":"=r" (i));
	}

#ifndef CONFIG_ICACHE_OFF
	icache_enable();
#endif

	l2cache_enable();

	set_muxconf_regs();
	/*identify the HW REV and other IDs to do the board differentiation without
	  ifdefs */

	delay(100);

	/* Writing to AuxCR in U-boot using SMI for GP/EMU DEV */
	/* Currently SMI in Kernel on ES2 devices seems to have an isse
	 * Once that is resolved, we can postpone this config to kernel
	 */
	setup_auxcr(get_device_type(), external_boot);

	prcm_init();

	per_clocks_enable();
}

/*******************************************************
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 ********************************************************/
typedef enum {
    GPIO_OUTPUT = 0,
    GPIO_INPUT  = 1
} gpio_dir_t;

/* ========================================================================== */
/**
*  gpio_level_t    enum_description
*
*  @param  element_name
*/
/* ========================================================================== */
typedef enum {
    GPIO_LOW  = 0,
    GPIO_HIGH = 1
} gpio_level_t;

extern int gpio_pin_init(u32, gpio_dir_t, gpio_level_t);


typedef enum _button_state {
	BUTTON_PRESSED = 0,
	BUTTON_RELEASED,
	BUTTON_MAX
} button_state;

int get_mmcbootdev(void)
{
	char *boot_device = NULL;
	char *ep = NULL;
	int dev = 0;

	// We need this early
	get_boot_device();

	run_command("run autodetectmmc; run readtokens", 0 );

	boot_device = getenv("mmcbootdev");
	if (!boot_device) {
		puts("\n** No boot device **\n");
		return 0;
	}

	dev = simple_strtoul(boot_device, &ep, 16);
	return dev;
}

int gossamer_mmc_splash_reinit(void)
{
	int dev = 0;
	int part = 1;
	int stat;
	block_dev_desc_t *dev_desc = NULL;

	dev = get_mmcbootdev();
	stat = mmc_init(dev);
	if (stat) {
		printf("No MMC%d card found!\n", dev);
		return -1;
	}

	dev_desc = get_dev("mmc", dev);
	if (dev_desc == NULL) {
		printf ("\n** ****** Invalid boot device ** *********\n");
		return -1;
	}
	if (fat_register_device(dev_desc, part) != 0 ) {
		printf ("\n** ******** Unable to use %s %d:%d for fatload ** **********\n", "mmc", dev, part);
		return -1;
	}

	return 0;
}


static void set_epd_pmic_env(void)
{
    DECLARE_GLOBAL_DATA_PTR;

    switch (gd->bd->bi_board_revision) {
        case BOARD_GOSSAMER_REV_EVT0:
        case BOARD_GOSSAMER_REV_EVT1A:
        case BOARD_GOSSAMER_REV_EVT3:
        case BOARD_GOSSAMER_REV_DVT:
        default:
            setenv ("epd_pmic", "tps65180-1p2-i2c");
            break;
        case BOARD_GOSSAMER_REV_EVTPRE1C:
        case BOARD_GOSSAMER_REV_EVT1C:
        case BOARD_GOSSAMER_REV_EVT2:
            setenv ("epd_pmic", "tps65185-i2c");
            break;
    }
}

 
 
int board_dsp_display_pic_file(char *filename, int tout, int flash)
{
#if defined(CONFIG_DSP_SPLASH_SCREEN)
	int ret;
	printf("Display image %s. Flash=%d\n",filename, flash);
	if (!flash) {
		ret= dsp_display_pic_file_by_wvfid(filename,tout,WVFID_AUTO);
	}
	else {
		ret= dsp_display_pic_file_by_wvfid(filename,tout,WVFID_GC);
	}
	udelay(500 * 1000);
	return ret;
#else
	return 0;
#endif
}

/* power is ok and system is ready to boot */
int display_booting_logo(void)
{
	return board_dsp_display_pic_file("booting.pgm", GOSSAMER_DRAW_TIMEOUT, 1);
}

/* device is connected to external power */
int has_external_power(void)
{
	select_bus(TPS65921_I2C_BUS, CFG_I2C_SPEED);
	return twl4030_get_vbus_status() == USB_LINK_VBUS;
}

/* battery is not charged and no charger is plugged */
static int display_dead_battery(void)
{
	return board_dsp_display_pic_file("dead.pgm", GOSSAMER_DRAW_TIMEOUT,1);
}

/* charger is plugged and charging is started */
static int display_charging(int num, int flashing)
{
	char pic[32];

	sprintf(pic, "charging%d.pgm", num);

	board_dsp_display_pic_file(pic, GOSSAMER_DRAW_TIMEOUT, flashing);

	return 0;
}

#if 0
/* check if boot was done by the power button */
static int boot_from_button(void)
{
	u8 data;

	select_bus(TPS65921_I2C_BUS, CFG_I2C_SPEED);

	/* figure out why we have booted */
	i2c_read(TWL4030_CHIP_INT, TWL4030_BASEADD_INT, 1, &data, 1);

	/* PWRON is set, return success */
	if (data & 0x01) {
		printf("boot from button\n");
		return 0;
	}

	return -1;
}
#endif

/* get current state of the power button */
static button_state get_pwr_button_state(void)
{
	u8 value;

	value = gpio_pin_read(GOSSAMER_PWR_BUTTON_GPIO);
	if (value < 0)
		printf("Can't read PWR_BUTTON_GPIO\n");

	if (value)
		return BUTTON_PRESSED;

	return BUTTON_RELEASED;
}

int gossamer_set_usb_drive(void)
{
	u8 data = 0x2C;

	return (i2c_write(TWL4030_CHIP_USB, TPS65921_USB_FUNC_CTRL2, 1, &data, 1));
}

enum boot_action {
    SHUTDOWN, // shutdown
    SHUTDOWN_LOWBAT, // shutdown due to low battery
    BOOT, // normal boot
    CHARGE, // charge battery
    CHARGE_ENABLE, // enable the charger
    SAFE_TO_BOOT, // state after system determines safe to boot and attached to charger
    WAIT_FOR_CHARGE, // do the loop again
};

struct boot_state {
    enum boot_action current_action;
    u16 bat_voltage;
    u16 bat_charge;
    u16 nominal_capacity;
    int bat_present;
    int charger_present;
    int charger_enabled;
    int battery_was_charged;
    u8 charger_wait_count;
};

static enum boot_action gossamer_get_lowbat_nocharger(struct boot_state *state)
{
    enum boot_action  next_state = SHUTDOWN_LOWBAT;
    // Attempt to redetect charger before we shut down
    if (state->charger_wait_count < GOSSAMER_VBUS_WAIT_COUNT_MAX) {
        ++state->charger_wait_count;
        return WAIT_FOR_CHARGE;
    }

    // no charger, if voltage to low for e-ink just shutdown, else show lowbat image and shutdown
    if (state->bat_voltage < GOSSAMER_MIN_DEAD_UVOLTAGE_PIC || state->bat_charge < GOSSAMER_MIN_DEAD_SOC_PIC ) {
        if ( !is_dsp_on() || state->bat_voltage < GOSSAMER_MIN_DEAD_UVOLTAGE_PIC_DSP_ON) {
            // shutdown if dsp is off or the voltage is really low
            next_state = SHUTDOWN;
        }
    }
    return next_state;
}

/* isolate the decision to one point 
 */
static enum boot_action gossamer_get_boot_action(struct boot_state *state) 
{
#if HAS_BQ27520_BATTERY_MODEL
    u16 boot_limit;
#endif

    // Currently charging or trying to detect charger
    if (state->current_action == CHARGE ||
        state->current_action == WAIT_FOR_CHARGE) {

        // No charger present
        if (!state->charger_present) {
            if (state->bat_voltage >= GOSSAMER_MIN_ALWAYS_BOOT_UVOLTAGE) {
                return SAFE_TO_BOOT;
            }
            // Just do the normal low bat action
            // we know that we are in lowbat conditions
            // because we are trying to charge
            return gossamer_get_lowbat_nocharger(state);
        }

#if HAS_BQ27520_BATTERY_MODEL
        boot_limit = state->charger_enabled == GOSSAMER_WALL_CHARGE_MAX ? GOSSAMER_MIN_DEAD_BOOT_SOC_WALL : GOSSAMER_MIN_DEAD_BOOT_SOC_USB;
        return state->bat_charge < boot_limit ? CHARGE : SAFE_TO_BOOT;
#else
        return state->bat_voltage < GOSSAMER_MIN_ALWAYS_BOOT_UVOLTAGE ? CHARGE : SAFE_TO_BOOT;
#endif
    }

    // We are at dead battery voltage
    if (state->bat_voltage < GOSSAMER_MIN_DEAD_UVOLTAGE 
#if HAS_BQ27520_BATTERY_MODEL
        || state->bat_charge < GOSSAMER_MIN_DEAD_SOC
#endif
       ) {
        
        if (!state->charger_present) {
            return gossamer_get_lowbat_nocharger(state);
        }

        // Charger present lets try to charge
        state->charger_wait_count = 0;
        return state->charger_enabled ? CHARGE : CHARGE_ENABLE;
    }

    // good to go, if it is safe to change charge current 
    if (state->charger_enabled) {
        if (bq24073_get_current() > 0 ) {
            // safe to change charge current, so disable charger ???
            return SAFE_TO_BOOT;
        }
        return CHARGE;
    }
    
    return BOOT;
}

static int gossamer_charger_enable(void)
{
    select_bus(TPS65921_I2C_BUS, CFG_I2C_SPEED);
	udelay(10 * 1000);

    switch(tps65921_get_charger_amps()) {
    case TPS65921_USB_DET_STS_100:
        printf("usb detected\n");
        bq24073_enable_charger();
        bq24073_enable(GOSSAMER_USB_CHARGE_MAX);
        return GOSSAMER_USB_CHARGE_MAX;
    case TPS65921_USB_DET_STS_500:
        printf("charger detected\n");
        bq24073_enable_charger();
        bq24073_enable(GOSSAMER_WALL_CHARGE_MAX);
        return GOSSAMER_WALL_CHARGE_MAX;
    default:
        printf("unknown detected\n");
        return 0;
    }
}

static void gossamer_charger_disable(void)
{
    select_bus(TPS65921_I2C_BUS, CFG_I2C_SPEED);
    bq24073_disable_charger();
}

static void gossamer_do_charge_cycle(struct boot_state *state)
{
#if defined(DEBUG_CHARGE_LED_GPIO)
    gpio_pin_write(DEBUG_CHARGE_LED_GPIO, GPIO_HIGH);
#endif
	udelay(500 * 1000);
#if defined(DEBUG_CHARGE_LED_GPIO)
	gpio_pin_write(DEBUG_CHARGE_LED_GPIO, GPIO_LOW);
#endif
	udelay(500 * 1000);
	printf("Wait for charge to boot %d mV, %d %%\n", state->bat_voltage, state->bat_charge);
}


void gossamer_charge(void)
{
    int charger_pic = 0;
    struct boot_state state;
    int safe_display = 0;
    int safe_change_charge = 0;
    u16 na_cap,re_cap,imp;
    s16 current;
    struct rtc_time cur_time;
    unsigned int  timestamp;

    state.bat_present = bq27520_battery_present() < 0 ? 0 : 1;
	select_bus(TPS65921_I2C_BUS, CFG_I2C_SPEED);

    rtc_get(&cur_time);
    timestamp = mktime (cur_time.tm_year, cur_time.tm_mon,cur_time.tm_yday, cur_time.tm_hour,
                        cur_time.tm_min, cur_time.tm_sec);
    printf("Get DATE: %4d-%02d-%02d (wday=%d)  TIME: %2d:%02d:%02d  %u\n",
        cur_time.tm_year, cur_time.tm_mon, cur_time.tm_mday, cur_time.tm_wday,
        cur_time.tm_hour, cur_time.tm_min, cur_time.tm_sec , timestamp);

    if (!state.bat_present) {
        // No battery detected, just boot the device
        return;
    }

    state.charger_enabled = 0;
    state.charger_wait_count = 0;
    state.bat_charge = 0;
    state.bat_voltage = 0;
    state.battery_was_charged = 0;

 
    bq27520_get_voltage(&state.bat_voltage);
    bq27520_get_state_of_charge(&state.bat_charge);

    printf("Initial voltage %hu mV charge %hu %%\n", state.bat_voltage, state.bat_charge);

    // tmp fix for soc error when battery is dead...
    for (;;) {
        state.bat_voltage = 0;
        state.bat_charge = 0;

        // read charger detect
        select_bus(TPS65921_I2C_BUS, CFG_I2C_SPEED);

        bq27520_get_voltage(&state.bat_voltage);
	    bq27520_get_state_of_charge(&state.bat_charge);

        rtc_get(&cur_time);
        timestamp = mktime (cur_time.tm_year, cur_time.tm_mon,cur_time.tm_yday, cur_time.tm_hour,
                            cur_time.tm_min, cur_time.tm_sec);

        bq27520_get_current(&current);
        bq27520_get_nominal_available_capacity(&na_cap);
        bq27520_get_remaining_capacity(&re_cap);
        bq27520_get_normalized_impedance(&imp);

        printf("BBLOG: %u %d %d %d %d %d %d\n",timestamp, state.bat_voltage,current,state.bat_charge,na_cap,re_cap,imp);
        
        if (state.bat_voltage < GOSSAMER_MIN_ZERO_SOC_UVOLTAGE && state.bat_charge >= GOSSAMER_MIN_DEAD_SOC )
        {
            printf("error is gas gauge %d volts %d soc\n",state.bat_voltage,state.bat_charge);
            state.bat_charge = 0;
        }
        udelay(10 * 1000);
    
        state.charger_present = (twl4030_get_vbus_status() == USB_LINK_VBUS) ? 1:0;
        safe_display = (
                   (bq24073_get_current() > GOSSAMER_USB_CHARGE_MAX && (state.bat_voltage >= GOSSAMER_MIN_DEAD_UVOLTAGE_CHARGE_PIC_WALL))
                || (state.battery_was_charged == 1 && (state.bat_voltage >= GOSSAMER_MIN_DEAD_UVOLTAGE_CHARGE_PIC))
                || (state.battery_was_charged == 1 && (state.bat_charge >= GOSSAMER_MIN_DEAD_SOC_CHARGE_PIC))
                || (state.battery_was_charged == 0 && (state.bat_voltage >= GOSSAMER_MIN_DEAD_UVOLTAGE_PIC_SAFE || 
                            state.bat_charge >= GOSSAMER_MIN_DEAD_SOC_PIC ))
                ) ? 1:0;

        safe_change_charge = (state.bat_charge > GOSSAMER_MIN_CHARGER_CONTROL_SOC 
                 || (bq24073_get_current() == 0 && state.bat_voltage > GOSSAMER_MIN_CHARGER_CONTROL_UVOLTAGE )
                 || (bq24073_get_current() != 0 && state.bat_voltage > GOSSAMER_MIN_CHARGER_CONTROL_UVOLTAGE_CHARGING )
                 ) ? 1:0;
        if (safe_change_charge && (bq24073_get_current() <= 0) ) {
            bq24073_allow_current_change();
        }
                    
        state.current_action = gossamer_get_boot_action(&state);
#if 0
        printf("state: current_action: %d charger_present: %d bat_voltage: %hu\n\t\
                bat_charge: %hu charger_enabled: %u charger_wait_count: %d bat_present: %u\n"
               "safe_display: %d  safe_change_charge %d     bq24073_current %d\n",
                state.current_action, state.charger_present, state.bat_voltage,
                state.bat_charge, state.charger_enabled, state.charger_wait_count, state.bat_present,
               safe_display, safe_change_charge, bq24073_get_current() 
               );
#endif

        switch(state.current_action) {
        case SHUTDOWN:            
            // Just shutdown in this case, don't update the screen
            // just leave it blank
            dsp_stop(1);
            gossamer_charger_disable();
            tps65921_shutdown("shutdown");
            // tps65921_shutdown powers off the device
            break;
        case SHUTDOWN_LOWBAT:
            // Show lowbat screen and then shutdown
            if (safe_display) {
                dsp_start(NULL);
            }
            if (is_dsp_on()) {
                display_dead_battery();
            }
            dsp_stop(0);
            gossamer_charger_disable();
            tps65921_shutdown("low battery");
            // tps65921_shutdown powers off the device
            break;
        case CHARGE_ENABLE:
            state.charger_enabled = gossamer_charger_enable();
            state.battery_was_charged |= 1;
            break;
        case WAIT_FOR_CHARGE:
            udelay(100*1000);
            break;
        case CHARGE:
            if (safe_display) {
		        dsp_start(NULL);
		        //dsp_start("charging0.pgm");
		        // Show charge image and enable charging
            }
            if (is_dsp_on()) {
                display_charging(charger_pic,0);
            }
            gossamer_do_charge_cycle(&state);
            charger_pic = (charger_pic + 1) % GOSSAMER_CHARGE_IMAGES;
            break;
        case SAFE_TO_BOOT:
        case BOOT:
            if (bq24073_get_current() <= 0 ) {
                if (state.charger_present) {
                    gossamer_charger_enable();
                    bq24073_allow_current_change();
                }
                else {
                    // make sure pin mux is setup correctly by selecting default 100mA current, but not charging
                    bq24073_allow_current_change();
                    bq24073_enable(GOSSAMER_USB_CHARGE_MIN);
                    gossamer_charger_disable();
                }
            }
            // All good, continue to boot
            return;
        }
    }
}

/* this is the entry for the charging routine */
void gossamer_charger_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;
#if defined(DEBUG_CHARGE_LED_GPIO)
	gpio_pin_init(DEBUG_CHARGE_LED_GPIO, GPIO_OUTPUT, GPIO_LOW);
#endif
	printf("gossamer charger init\n");
	/*
	 * as stated by the charging procedure
	 *
	 * ERRATA: charger is detected as 100ma
	 * if it is 500ma.
	 */
	select_bus(TPS65921_I2C_BUS, CFG_I2C_SPEED);
	tps65921_enable_usb_power();
	tps65921_disable_usb_transceiver();
	tps65921_enable_charger_detect();

	gossamer_mmc_splash_reinit();

	if (gd->bd->bi_board_revision  ==  BOARD_GOSSAMER_REV_EVT0 ||
	    gd->bd->bi_board_revision ==  BOARD_GOSSAMER_REV_EVT1A ||
	    gd->bd->bi_board_revision == BOARD_GOSSAMER_REV_EVTPRE1C) {
	    bq24073_init(GOSSAMER_BQ24074_CE_GPIO_EVTPRE1C,
			 GOSSAMER_BQ24074_EN1_GPIO_EVTPRE1C,
			 GOSSAMER_BQ24074_EN2_GPIO_EVTPRE1C);
	}
	else {
		bq24073_init(GOSSAMER_BQ24074_CE_GPIO_EVT1C,
			     GOSSAMER_BQ24074_EN1_GPIO_EVT1C,
			     GOSSAMER_BQ24074_EN2_GPIO_EVT1C);
	}
}

void gossamer_button_rtc_ack(void)
{
#define RTC_STATUS_DUMP (RTC_STATUS_POWER_UP|RTC_STATUS_ALARM|RTC_STATUS_DAY_EVENT|RTC_STATUS_HOUR_EVENT|RTC_STATUS_MIN_EVENT|RTC_STATUS_SEC_EVENT)

#define TWL4030_DUMPRTC_REG(X) 	    \
	if (i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + X, 1, &val, 1))  \
	{	printf("%s: I/O error\n",#X);} else printf("%s=0x%x\n",#X,val);

	unsigned char val,data;
	int err;

	err=i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_INTERRUPTS_REG, 1, &data, 1);
	if (data) {
		printf("warning-rtc irq enabled = 0x%x\n",data);
	}
	err|=i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_STATUS_REG, 1, &val, 1);
	if (data||(val&RTC_STATUS_DUMP)) {
		printf("warning-rtc status = 0x%x\n",val);
		TWL4030_DUMPRTC_REG(RTC_ALARM_SECONDS_REG);
		TWL4030_DUMPRTC_REG(RTC_ALARM_MINUTES_REG);   
		TWL4030_DUMPRTC_REG(RTC_ALARM_HOURS_REG);     
		TWL4030_DUMPRTC_REG(RTC_ALARM_DAYS_REG);      
		TWL4030_DUMPRTC_REG(RTC_ALARM_MONTHS_REG);    
		TWL4030_DUMPRTC_REG(RTC_ALARM_YEARS_REG);     
		/* get the time*/
		i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);
		val|=0x40;
		i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);
		TWL4030_DUMPRTC_REG(RTC_SECONDS_REG);
		TWL4030_DUMPRTC_REG(RTC_MINUTES_REG);
		TWL4030_DUMPRTC_REG(RTC_HOURS_REG);
		TWL4030_DUMPRTC_REG(RTC_DAYS_REG);
		TWL4030_DUMPRTC_REG(RTC_MONTHS_REG);
		TWL4030_DUMPRTC_REG(RTC_YEARS_REG);
		TWL4030_DUMPRTC_REG(RTC_WEEKS_REG);

		/* acknowledge any alarm*/
		val = RTC_STATUS_RTC_ALARM_ACK;
		val = i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_STATUS_REG, 1, &val, 1);
	}
	if (data) {
		printf("warning-clearing rtc irq\n");
		data = 0;
		val = i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_INTERRUPTS_REG, 1, &val, 1);
	}
}

int misc_init_r(void)
{
	DECLARE_GLOBAL_DATA_PTR;
#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	unsigned char data;

	printf("Hardware arch: %s rev: %s\n",
            arch_string(gd->bd->bi_arch_number),
            rev_string(gd->bd->bi_board_revision));

	/* Store the state of the power button now. It will be checked later,
	   but can't be read then because that means it has to be held for
	   perilously close to the 8 seconds it takes to shut down again. */
	power_button_boot = get_pwr_button_state() == BUTTON_PRESSED;
	if (!power_button_boot)
		printf("Power button is not pressed\n");

	select_bus(TPS65921_I2C_BUS, CFG_I2C_SPEED);

    printf("pmic watchdog time %d\n",tps65921_get_watchdog());

    /* turn on long pwr button press reset*/
    data = 0x40;
    i2c_write(0x4b, 0x46, 1, &data, 1);
    printf("Power Button Active\n");
	
	(void) twl4030_usb_init();
	twl4030_power_reset_init();
	if (gossamer_set_usb_drive()) {
		printf("Unable to change USB drive - trying to go on\n");
	}

    first_boot= 0;
    if (!tps65921_is_rtc_running()) {
        // if needed one can use this to check to see if PMIC was 'cold booted', as RTC by default is not running
        // This indicates battery was just inserted or charger was inserted to cause system to boot
        rtc_reset();
        first_boot= 1;
    }

	gossamer_charger_init();
	select_bus(TPS65921_I2C_BUS, CFG_I2C_SPEED);
    printf("gossamer_charger_init complete\n");
    // board_charge();

	/* see if we need to activate the power button startup */
//#warning "TODO: Add charger for Gossamer"
	/* For charger from EVT1B */
	// Only in case of EVT1A, enable this workaround to prevent nRESPWRON line toggle

//#warning "MMS: Why is this? There is no info in the related patch... Remove it?"
#if 0
	printf("Forcing internal level shifter to VIO/VINTDIG\n");
	data_read = 0xB6;
	i2c_write(0x49, 0x97, 1,&data_read, 1);
	data_read = 0x4A;
	i2c_write(0x4b, 0x44, 1,&data_read, 1);
	data_read = 0xB3;
	i2c_write(0x4b, 0x44, 1,&data_read, 1);
	data_read = 0x04;
	i2c_write(0x4b, 0x6c, 1,&data_read, 1);
	data_read = 0xFC;
	i2c_write(0x4b, 0x44, 1,&data_read, 1);
	data_read = 0x96;
	i2c_write(0x4b, 0x44, 1,&data_read, 1);
#endif

	/* Currently no AUD-Codec */
	/*
	printf("Keep Audio codec under reset \n");
	gpio_pin_init(37, GPIO_OUTPUT, GPIO_LOW);

	printf("Power on Audio codec\n");
	gpio_pin_init(103, GPIO_OUTPUT, GPIO_HIGH);
	*/

	char *s = getenv("pbboot");
	if (s) {
		/* figure out why we have booted */
		i2c_read(0x4b, 0x3a, 1, &data, 1);

		/* if status is non-zero, we didn't transition
		 * from WAIT_ON state
		 */
		if (data) {
			printf("Transitioning to Wait State (%x)\n", data);

			/* clear status */
			data = 0;
			i2c_write(0x4b, 0x3a, 1, &data, 1);

			/* put PM into WAIT_ON state */
			data = 0x01;
			i2c_write(0x4b, 0x46, 1, &data, 1);

			/* no return - wait for power shutdown */
			while (1) {;}
		}
		printf("Transitioning to Active State (%x)\n", data);

		/* turn on long pwr button press reset*/
		data = 0x40;
		i2c_write(0x4b, 0x46, 1, &data, 1);
		printf("Power Button Active\n");
	}
#endif /* CONFIG_DRIVER_OMAP34XX_I2C */
	select_bus(TPS65921_I2C_BUS, CFG_I2C_SPEED);
	(void) twl4030_keypad_init();
	dieid_num_r();
#ifdef PAPYRUS2_VDDH_PATCH
	switch (gd->bd->bi_board_revision) {
		case BOARD_GOSSAMER_REV_EVTPRE1C:
		case BOARD_GOSSAMER_REV_EVT1C:
			papyrus2_patch(PAPYRUS2_1P0);
			break;
		case BOARD_GOSSAMER_REV_EVT2:
			papyrus2_patch(PAPYRUS2_1P1);
			break;
		default:
			break;
	}
#endif


	select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);
	/* For MSP430 TS programming */
	gpio_pin_init(140, GPIO_OUTPUT, GPIO_LOW);
	gpio_pin_init(141, GPIO_OUTPUT, GPIO_LOW);
	gpio_pin_init(37, GPIO_OUTPUT, GPIO_LOW);
    	/* Clear UART2 MSR register - modifying the pin
           muxing for MSP430 programming generates dummy
           events there */
	NS16550_t uart2 = (NS16550_t)CFG_NS16550_COM2;
	u8 volatile v = uart2->msr;
    v=v;

    tps65921_keypad_init();
    set_epd_pmic_env();
    return (0);
}


void board_early_console(void)
{
    /* Power down the device immediately UNLESS it's a warm boot, OR there's
       a command written in the BCB, OR the power button was pressed for
       a long time OR the device is plugged in to power */
    if (!strcmp(getenv("resettype"), "cold") && !get_bcb_boot_code() &&
	!get_power_button_boot_state() && !has_external_power() && !first_boot) {
        gossamer_button_rtc_ack();  
        tps65921_shutdown("short power press");
    }

    gossamer_mmc_splash_reinit();

	gossamer_charge();

#ifdef WATCHDOG_POWER_BUTTON_DISABLE
    printf("watchdog starting, power button inactive\n");
    tps65921_set_watchdog(30);
    tps65921_stopon_pwr_button(TPS65921_STOPON_DISABLE);
#endif

	bq27520_impedance_track_enable();
	if (bq27520_disable_batlspuen()) {
		printf("Something went wrong when trying to disable BATLSPUEN\n");
	}

    dsp_start(NULL);
    display_booting_logo();
    dsp_stop(0);

    check_boot_buttons();

#ifdef WATCHDOG_POWER_BUTTON_DISABLE
    if (get_pwr_button_state()==BUTTON_PRESSED) {
        int count,wcount=-1;
        printf("Power Button Inactive, waiting for user to release key\n");
        for (count=0;count<100 && (get_pwr_button_state()==BUTTON_PRESSED);count++) {
            if (wcount!=tps65921_get_watchdog()) {
               wcount = tps65921_get_watchdog();
               printf("watchdog %d\n",wcount);
            }
            udelay(100 * 1000);
        }
    }
 
    /* turn on long pwr button press reset*/
    tps65921_stopon_pwr_button(TPS65921_STOPON_ENABLE);
    tps65921_set_watchdog(0);
    printf("watchdog disabled %d\n",tps65921_get_watchdog());

#endif

    if (bq24073_get_current()<0) {
        bq24073_allow_current_change();
        bq24073_enable(GOSSAMER_USB_CHARGE_MAX);
        gossamer_charger_disable();
    }

}


/******************************************************
 * Routine: wait_for_command_complete
 * Description: Wait for posting to finish on watchdog
 ******************************************************/
void wait_for_command_complete(unsigned int wd_base)
{
	int pending = 1;
	do {
		pending = __raw_readl(wd_base + WWPS);
	} while (pending);
}

/****************************************
 * Routine: watchdog_init
 * Description: Shut down watch dogs
 *****************************************/
void watchdog_init(void)
{
	/* There are 3 watch dogs WD1=Secure, WD2=MPU, WD3=IVA. WD1 is
	 * either taken care of by ROM (HS/EMU) or not accessible (GP).
	 * We need to take care of WD2-MPU or take a PRCM reset.  WD3
	 * should not be running and does not generate a PRCM reset.
	 */

	sr32(CM_FCLKEN_WKUP, 5, 1, 1);
	sr32(CM_ICLKEN_WKUP, 5, 1, 1);
	wait_on_value(BIT5, 0x20, CM_IDLEST_WKUP, 5); /* some issue here */

	__raw_writel(WD_UNLOCK1, WD2_BASE + WSPR);
	wait_for_command_complete(WD2_BASE);
	__raw_writel(WD_UNLOCK2, WD2_BASE + WSPR);
}

/**********************************************
 * Routine: dram_init
 * Description: sets uboots idea of sdram size
 **********************************************/
int dram_init(void)
{
#define NOT_EARLY 0
	DECLARE_GLOBAL_DATA_PTR;
	unsigned int size0 = 0, size1 = 0;
	u32 mtype, btype;

	btype = get_board_type();
	mtype = get_mem_type();
#ifndef CONFIG_3430ZEBU
	/* fixme... dont know why this func is crashing in ZeBu */
	display_board_info(btype);
#endif
	/* If a second bank of DDR is attached to CS1 this is
	 * where it can be started.  Early init code will init
	 * memory on CS0.
	 */
	if ((mtype == DDR_COMBO) || (mtype == DDR_STACKED)) {
		do_sdrc_init(SDRC_CS1_OSET, NOT_EARLY);
	}
	size0 = get_sdr_cs_size(SDRC_CS0_OSET);
	size1 = get_sdr_cs_size(SDRC_CS1_OSET);

	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = size0;
	gd->bd->bi_dram[1].start = PHYS_SDRAM_1+size0;
	gd->bd->bi_dram[1].size = size1;

	return 0;
}

#define 	MUX_VAL(OFFSET,VALUE)\
		__raw_writew((VALUE), OMAP34XX_CTRL_BASE + (OFFSET));

#define		CP(x)	(CONTROL_PADCONF_##x)
/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */
#define MUX_GOSSAMER()\
	/*SDRC*/\
	MUX_VAL(CP(SDRC_D0),        (IEN  | PTD | DIS | M0)) /*SDRC_D0*/\
	MUX_VAL(CP(SDRC_D1),        (IEN  | PTD | DIS | M0)) /*SDRC_D1*/\
	MUX_VAL(CP(SDRC_D2),        (IEN  | PTD | DIS | M0)) /*SDRC_D2*/\
	MUX_VAL(CP(SDRC_D3),        (IEN  | PTD | DIS | M0)) /*SDRC_D3*/\
	MUX_VAL(CP(SDRC_D4),        (IEN  | PTD | DIS | M0)) /*SDRC_D4*/\
	MUX_VAL(CP(SDRC_D5),        (IEN  | PTD | DIS | M0)) /*SDRC_D5*/\
	MUX_VAL(CP(SDRC_D6),        (IEN  | PTD | DIS | M0)) /*SDRC_D6*/\
	MUX_VAL(CP(SDRC_D7),        (IEN  | PTD | DIS | M0)) /*SDRC_D7*/\
	MUX_VAL(CP(SDRC_D8),        (IEN  | PTD | DIS | M0)) /*SDRC_D8*/\
	MUX_VAL(CP(SDRC_D9),        (IEN  | PTD | DIS | M0)) /*SDRC_D9*/\
	MUX_VAL(CP(SDRC_D10),       (IEN  | PTD | DIS | M0)) /*SDRC_D10*/\
	MUX_VAL(CP(SDRC_D11),       (IEN  | PTD | DIS | M0)) /*SDRC_D11*/\
	MUX_VAL(CP(SDRC_D12),       (IEN  | PTD | DIS | M0)) /*SDRC_D12*/\
	MUX_VAL(CP(SDRC_D13),       (IEN  | PTD | DIS | M0)) /*SDRC_D13*/\
	MUX_VAL(CP(SDRC_D14),       (IEN  | PTD | DIS | M0)) /*SDRC_D14*/\
	MUX_VAL(CP(SDRC_D15),       (IEN  | PTD | DIS | M0)) /*SDRC_D15*/\
	MUX_VAL(CP(SDRC_D16),       (IEN  | PTD | DIS | M0)) /*SDRC_D16*/\
	MUX_VAL(CP(SDRC_D17),       (IEN  | PTD | DIS | M0)) /*SDRC_D17*/\
	MUX_VAL(CP(SDRC_D18),       (IEN  | PTD | DIS | M0)) /*SDRC_D18*/\
	MUX_VAL(CP(SDRC_D19),       (IEN  | PTD | DIS | M0)) /*SDRC_D19*/\
	MUX_VAL(CP(SDRC_D20),       (IEN  | PTD | DIS | M0)) /*SDRC_D20*/\
	MUX_VAL(CP(SDRC_D21),       (IEN  | PTD | DIS | M0)) /*SDRC_D21*/\
	MUX_VAL(CP(SDRC_D22),       (IEN  | PTD | DIS | M0)) /*SDRC_D22*/\
	MUX_VAL(CP(SDRC_D23),       (IEN  | PTD | DIS | M0)) /*SDRC_D23*/\
	MUX_VAL(CP(SDRC_D24),       (IEN  | PTD | DIS | M0)) /*SDRC_D24*/\
	MUX_VAL(CP(SDRC_D25),       (IEN  | PTD | DIS | M0)) /*SDRC_D25*/\
	MUX_VAL(CP(SDRC_D26),       (IEN  | PTD | DIS | M0)) /*SDRC_D26*/\
	MUX_VAL(CP(SDRC_D27),       (IEN  | PTD | DIS | M0)) /*SDRC_D27*/\
	MUX_VAL(CP(SDRC_D28),       (IEN  | PTD | DIS | M0)) /*SDRC_D28*/\
	MUX_VAL(CP(SDRC_D29),       (IEN  | PTD | DIS | M0)) /*SDRC_D29*/\
	MUX_VAL(CP(SDRC_D30),       (IEN  | PTD | DIS | M0)) /*SDRC_D30*/\
	MUX_VAL(CP(SDRC_D31),       (IEN  | PTD | DIS | M0)) /*SDRC_D31*/\
	MUX_VAL(CP(SDRC_CLK),       (IEN  | PTD | DIS | M0)) /*SDRC_CLK*/\
	MUX_VAL(CP(SDRC_DQS0),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS0*/\
	MUX_VAL(CP(SDRC_DQS1),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS1*/\
	MUX_VAL(CP(SDRC_DQS2),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS2*/\
	MUX_VAL(CP(SDRC_DQS3),      (IEN  | PTD | DIS | M0)) /*SDRC_DQS3*/\
	MUX_VAL(CP(SDRC_nCS0),      (IEN  | PTD | DIS | M0)) /*SDRC_nCS0*/\
	MUX_VAL(CP(SDRC_nCS1),      (IEN  | PTD | DIS | M0)) /*SDRC_nCS1*/\
	/* MUX_VAL(CP(SYS_NRESWARM),   (IDIS | PTD | DIS | M0)) */ /*SYS_NRESWARM*/\
	/*GPMC - NC*/\
	MUX_VAL(CP(GPMC_D0),        (IEN  | PTD | EN  | M4)) /*GPMC_D0 NC*/\
	MUX_VAL(CP(GPMC_D1),        (IEN  | PTD | EN  | M4)) /*GPMC_D1 NC*/\
	MUX_VAL(CP(GPMC_D2),        (IEN  | PTD | EN  | M4)) /*GPMC_D2 NC*/\
	MUX_VAL(CP(GPMC_D3),        (IEN  | PTD | EN  | M4)) /*GPMC_D3 NC*/\
	MUX_VAL(CP(GPMC_D4),        (IEN  | PTD | EN  | M4)) /*GPMC_D4 NC*/\
	MUX_VAL(CP(GPMC_D5),        (IEN  | PTD | EN  | M4)) /*GPMC_D5 NC*/\
	MUX_VAL(CP(GPMC_D6),        (IEN  | PTD | EN  | M4)) /*GPMC_D6 NC*/\
	MUX_VAL(CP(GPMC_D7),        (IEN  | PTD | EN  | M4)) /*GPMC_D7 NC*/\
	MUX_VAL(CP(GPMC_nCS0),      (IEN  | PTD | EN  | M4)) /*GPMC_nCS0 NC*/\
	MUX_VAL(CP(GPMC_nADV_ALE),  (IEN  | PTD | EN  | M4)) /*GPMC_nADV_ALE NC*/\
	MUX_VAL(CP(GPMC_nOE),       (IEN  | PTD | EN  | M4)) /*GPMC_nOE NC*/\
	MUX_VAL(CP(GPMC_nWE),       (IEN  | PTD | EN  | M4)) /*GPMC_nWE NC*/\
	MUX_VAL(CP(GPMC_nBE1),      (IDIS | PTD | DIS | M4)) /*GPMC_nBE1 DC-CHG-ILM*/\
	MUX_VAL(CP(GPMC_WAIT0),     (IEN  | PTD | EN  | M4)) /*GPMC_WAIT0 NC*/\
	/*GPIOs*/\
	MUX_VAL(CP(ETK_D0_ES2 ),    (IEN  | WAKEUP_EN  | PTD | DIS | M4)) /*GIO-14  SYS-PWR-ON-KEY*/\
	MUX_VAL(CP(ETK_D1_ES2 ),    (IEN  | PTD | DIS | M4)) /*GIO-15  WLAN-IRQ */\
	MUX_VAL(CP(ETK_D2_ES2 ),    (IDIS | PTD | DIS | M4)) /*GIO-16  EN_WIFI_POW */\
	MUX_VAL(CP(ETK_D7_ES2 ),    (IEN  | PTD | EN  | M4)) /*GIO-21  MODEM_EN -> NC*/\
	MUX_VAL(CP(ETK_D8_ES2 ),    (IDIS | PTD | DIS | M4)) /*GIO-22  WLAN-EN */\
	MUX_VAL(CP(ETK_D9_ES2 ),    (IEN  | PTD | EN  | M4)) /*GIO-23  HSUSB2_RSTn -> NC*/\
	MUX_VAL(CP(GPMC_A1),        (IEN  | PTD | EN  | M4)) /*GIO-34  MODEM_nON NOT PRESENT -> NC*/\
	MUX_VAL(CP(GPMC_A2),        (IEN  | PTD | DIS | M4)) /*GIO-35  HW-ID3  SKU*/\
	MUX_VAL(CP(GPMC_A3),        (IEN  | PTD | EN  | M4)) /*GIO-36  LCD_PWR_EN -> NC*/\
	MUX_VAL(CP(GPMC_A4),        (IDIS | PTD | DIS | M4)) /*GIO-37  AUD_nRESET -> OMAP-Hapt-Enable*/\
	MUX_VAL(CP(GPMC_A5),        (IEN  | PTD | DIS | M4)) /*GIO-38  HW-ID2  PRODUCT ID*/\
	MUX_VAL(CP(GPMC_A6),        (IEN  | PTD | EN  | M4)) /*GIO-39  CRADLE_DET -> NC*/\
	MUX_VAL(CP(GPMC_A7),        (IEN  | PTD | DIS | M4)) /*GIO-40  HW-ID7  HW-REV*/\
	MUX_VAL(CP(GPMC_A8),        (IEN  | PTD | DIS | M4)) /*GIO-41  HW-ID6  HW-REV*/\
	MUX_VAL(CP(GPMC_A9),        (IEN  | PTD | DIS | M4)) /*GIO-42  HW-ID5  HW-REV*/\
	MUX_VAL(CP(GPMC_A10),       (IEN  | PTD | DIS | M4)) /*GIO-43  HW-ID4  SKU*/\
	MUX_VAL(CP(GPMC_D10),       (IEN  | PTD | EN  | M4)) /*GPMC_D10 GPIO46 TP-nRESET -> NC*/\
	MUX_VAL(CP(GPMC_D11),       (IEN  | PTD | EN  | M4)) /*GPMC_D11 GPIO47 misc I/O*/\
	MUX_VAL(CP(GPMC_D12),       (IEN  | PTD | EN  | M4)) /*GPMC_D12 GPIO48 misc I/O*/\
	MUX_VAL(CP(GPMC_D13),       (IEN  | PTD | EN  | M4)) /*GPMC_D13 GPIO49 misc I/O*/\
	MUX_VAL(CP(GPMC_D14),       (IEN  | PTD | EN  | M4)) /*GPMC_D14 GPIO50 misc I/O*/\
	MUX_VAL(CP(GPMC_D15),       (IEN  | PTD | EN  | M4)) /*GPMC_D15 GPIO51 misc I/O*/\
	MUX_VAL(CP(GPMC_CLK),       (IEN  | PTD | EN  | M4)) /*GIO-59 AUD-CODEC-nINT -> NC*/\
	MUX_VAL(CP(GPMC_nBE0_CLE),  (IDIS  | PTD | DIS | M4)) /*GPIO60 (NAV-RESET) */\
	MUX_VAL(CP(GPMC_nWP),       (IDIS  | PTD | EN  | M4)) /*GPIO62 (NAV_nEN) */\
	MUX_VAL(CP(CAM_XCLKA),      (IEN  | PTD | DIS | M4)) /*GIO-96 HW-ID0  PRODUCT ID*/\
	MUX_VAL(CP(CAM_D0 ),        (IEN  | PTD | EN  | M4)) /*GIO-99  TS_INT -> NC*/\
	MUX_VAL(CP(CAM_D1 ),        (IEN  | PTD | DIS | M4)) /*GIO-100 GG_INT*/\
	MUX_VAL(CP(CAM_D2 ),        (IDIS  | PTD | DIS | M4)) /*GIO-101 (OMAP-Hapt-PWM)*/\
	MUX_VAL(CP(CAM_D3 ),        (IDIS | PTD | DIS | M4)) /*GIO-102 CHRG_EN1*/\
	MUX_VAL(CP(CAM_D4 ),        (IEN  | PTD | EN  | M4)) /*GIO-103 AUD-CODEC-EN -> NC*/\
	MUX_VAL(CP(CAM_D5 ),        (IEN  | PTD | DIS | M4)) /*GIO-104 CHRG_EN2 -> GG_BAT_LOW*/\
	MUX_VAL(CP(CAM_D10),        (IEN  | PTD | EN  | M4)) /*GIO-109 UART_GPIO (spare)*/\
	MUX_VAL(CP(CAM_D11),        (IEN  | PTD | DIS | M4)) /*GIO-110 CHRG_Cen*/\
	MUX_VAL(CP(CAM_XCLKB),      (IEN  | PTD | DIS | M4)) /*GIO-111 CHRG_STATUS*/\
	MUX_VAL(CP(CSI2_DX0),       (IEN  | PTD | DIS | M4)) /*GIO-112 HW-ID1 PRODUCT ID*/\
	MUX_VAL(CP(CSI2_DY0),       (IEN  | PTD | DIS | M4)) /*GIO-113 (NAV-nINT)*/\
	MUX_VAL(CP(CSI2_DX1),       (IEN  | PTD | EN  | M4)) /*GIO-114 (NAV-nPresence -> 09.03 NC) */\
	MUX_VAL(CP(CSI2_DY1),       (IEN  | PTD | DIS | M4)) /*GIO-115 nUSB-CHG-OK -> USB-PWR-OK*/\
	MUX_VAL(CP(McBSP1_FSR),     (IEN  | PTD | EN  | M4)) /*GIO-157 HS-nDETECT -> NC*/ \
	/* Backlight */ \
	MUX_VAL(CP(GPMC_nCS7),      (IEN  | PTD | EN  | M4)) /*GPT_8_PWM_EVT -> NC */ \
	/*DSS*/\
	MUX_VAL(CP(DSS_PCLK),       (IDIS | PTD | DIS | M0)) /*DSS_PCLK*/\
	MUX_VAL(CP(DSS_HSYNC),      (IDIS | PTD | DIS | M0)) /*DSS_HSYNC*/\
	MUX_VAL(CP(DSS_VSYNC),      (IDIS | PTD | DIS | M0)) /*DSS_VSYNC*/\
	MUX_VAL(CP(DSS_ACBIAS),     (IEN  | PTD | DIS | M0)) /*LCD_ENABLE */\
	MUX_VAL(CP(DSS_DATA0),      (IDIS | PTD | DIS | M0)) /*DSS_DATA0 */\
	MUX_VAL(CP(DSS_DATA1),      (IDIS | PTD | DIS | M0)) /*DSS_DATA1 */\
	MUX_VAL(CP(DSS_DATA2),      (IDIS | PTD | DIS | M0)) /*DSS_DATA2 */\
	MUX_VAL(CP(DSS_DATA3),      (IDIS | PTD | DIS | M0)) /*DSS_DATA3 */\
	MUX_VAL(CP(DSS_DATA4),      (IDIS | PTD | DIS | M0)) /*DSS_DATA4 */\
	MUX_VAL(CP(DSS_DATA5),      (IDIS | PTD | DIS | M0)) /*DSS_DATA5 */\
	MUX_VAL(CP(DSS_DATA6),      (IDIS | PTD | DIS | M0)) /*DSS_DATA6 */\
	MUX_VAL(CP(DSS_DATA7),      (IDIS | PTD | DIS | M0)) /*DSS_DATA7 */\
	MUX_VAL(CP(DSS_DATA8),      (IEN  | PTD | EN  | M4)) /*DSS_DATA8 -> NC*/\
	MUX_VAL(CP(DSS_DATA9),      (IEN  | PTD | EN  | M4)) /*DSS_DATA9 -> NC*/\
	MUX_VAL(CP(DSS_DATA10),     (IEN  | PTD | EN  | M4)) /*DSS_DATA10 -> NC*/\
	MUX_VAL(CP(DSS_DATA11),     (IEN  | PTD | EN  | M4)) /*DSS_DATA11 -> NC*/\
	MUX_VAL(CP(DSS_DATA12),     (IDIS | PTD | DIS | M4)) /*DSS_DATA12 -> EN_VEE */\
	MUX_VAL(CP(DSS_DATA13),     (IDIS | PTD | DIS | M4)) /*DSS_DATA13 -> EN_VNEG_VCOM */\
	MUX_VAL(CP(DSS_DATA14),     (IDIS | PTD | DIS | M4)) /*DSS_DATA14 -> COM_CTRL */\
	MUX_VAL(CP(DSS_DATA15),     (IDIS | PTD | DIS | M4)) /*DSS_DATA15 -> EN_CPLD_POW */\
	MUX_VAL(CP(DSS_DATA16),     (IEN  | PTD | DIS | M4)) /*DSS_DATA16 -> EPD_PWR_GOOD */\
	MUX_VAL(CP(DSS_DATA17),     (IDIS | PTD | DIS | M4)) /*DSS_DATA17 -> EPD-PM-CEN */\
	MUX_VAL(CP(DSS_DATA18),     (IEN  | PTD | EN  | M4)) /*DSS_DATA18 -> NC*/\
	MUX_VAL(CP(DSS_DATA19),     (IEN  | PTD | EN  | M4)) /*DSS_DATA19 -> NC*/\
	MUX_VAL(CP(DSS_DATA20),     (IEN  | PTD | EN  | M4)) /*DSS_DATA20 -> NC*/\
	MUX_VAL(CP(DSS_DATA21),     (IEN  | PTD | EN  | M4)) /*DSS_DATA21 -> NC*/\
	MUX_VAL(CP(DSS_DATA22),     (IDIS | PTD | DIS | M4)) /*DSS_DATA22 -> EN_VPOS */\
	MUX_VAL(CP(DSS_DATA23),     (IDIS | PTD | DIS | M4)) /*DSS_DATA23 -> EN_VDDH */\
	/*Audio Interface */\
	MUX_VAL(CP(McBSP2_FSX),     (IEN  | PTD | EN  | M4)) /*McBSP2_FSX -> NC*/\
	MUX_VAL(CP(McBSP2_CLKX),    (IEN  | PTD | EN  | M4)) /*McBSP2_CLKX -> NC*/\
	MUX_VAL(CP(McBSP2_DR),      (IEN  | PTD | EN  | M4)) /*McBSP2_DR -> NC*/\
	MUX_VAL(CP(McBSP2_DX),      (IEN  | PTD | EN  | M4)) /*McBSP2_DX -> NC*/\
	/*micro SD connector - expansion card  */\
	MUX_VAL(CP(MMC1_CLK),       (IEN  | PTD | DIS | M0)) /*MMC1_CLK */\
	MUX_VAL(CP(MMC1_CMD),       (IEN  | PTD | DIS | M0)) /*MMC1_CMD */\
	MUX_VAL(CP(MMC1_DAT0),      (IEN  | PTD | DIS | M0)) /*MMC1_DAT0*/\
	MUX_VAL(CP(MMC1_DAT1),      (IEN  | PTD | DIS | M0)) /*MMC1_DAT1*/\
	MUX_VAL(CP(MMC1_DAT2),      (IEN  | PTD | DIS | M0)) /*MMC1_DAT2*/\
	MUX_VAL(CP(MMC1_DAT3),      (IEN  | PTD | DIS | M0)) /*MMC1_DAT3*/\
	/* eMMC */\
	MUX_VAL(CP(MMC2_CLK),       (IEN  | PTD | DIS | M0)) /*MMC2_CLK */\
	MUX_VAL(CP(MMC2_CMD),       (IEN  | PTD | DIS | M0)) /*MMC2_CMD */\
	MUX_VAL(CP(MMC2_DAT0),      (IEN  | PTD | DIS | M0)) /*MMC2_DAT0*/\
	MUX_VAL(CP(MMC2_DAT1),      (IEN  | PTD | DIS | M0)) /*MMC2_DAT1*/\
	MUX_VAL(CP(MMC2_DAT2),      (IEN  | PTD | DIS | M0)) /*MMC2_DAT2*/\
	MUX_VAL(CP(MMC2_DAT3),      (IEN  | PTD | DIS | M0)) /*MMC2_DAT3*/\
	MUX_VAL(CP(MMC2_DAT4),      (IEN  | PTD | DIS | M0)) /*MMC2_DIR_DAT0*/\
	MUX_VAL(CP(MMC2_DAT5),      (IEN  | PTD | DIS | M0)) /*MMC2_DIR_DAT1*/\
	MUX_VAL(CP(MMC2_DAT6),      (IEN  | PTD | DIS | M0)) /*MMC2_DIR_CMD */\
	MUX_VAL(CP(MMC2_DAT7),      (IEN  | PTD | DIS | M0)) /*MMC2_CLKIN*/\
	/*MMC3 (WiFi)*/\
	MUX_VAL(CP(ETK_CTL_ES2),    (IEN  | PTU | EN  | M2)) /*MMC3_CMD */\
	MUX_VAL(CP(ETK_CLK_ES2),    (IEN  | PTU | EN  | M2)) /*MMC3_CLK */\
	MUX_VAL(CP(ETK_D4_ES2 ),    (IEN  | PTU | EN  | M2)) /*MMC3_DAT0 */\
	MUX_VAL(CP(ETK_D5_ES2 ),    (IEN  | PTU | EN  | M2)) /*MMC3_DAT1 */\
	MUX_VAL(CP(ETK_D6_ES2 ),    (IEN  | PTU | EN  | M2)) /*MMC3_DAT2 */\
	MUX_VAL(CP(ETK_D3_ES2 ),    (IEN  | PTU | EN  | M2)) /*MMC3_DAT3 */\
	/*Bluetooth*/\
	MUX_VAL(CP(McBSP3_DX),      (IEN  | PTD | EN  | M4)) /*UART2_CTS -> NC*/\
	MUX_VAL(CP(McBSP3_DR),      (IEN  | PTD | EN  | M4)) /*UART2_RTS -> NC*/\
	MUX_VAL(CP(McBSP3_CLKX),    (IDIS | PTD | DIS | M1)) /*UART2_TX -> NC*/\
	MUX_VAL(CP(McBSP3_FSX),     (IEN  | PTD | EN  | M1)) /*UART2_RX -> NC*/\
	/*Console Interface */\
	MUX_VAL(CP(UART1_TX),       (IDIS | PTD | DIS | M0)) /*UART1_TX*/\
	MUX_VAL(CP(UART1_RTS),      (IDIS | PTD | DIS | M0)) /*UART1_RTS*/ \
	MUX_VAL(CP(UART1_CTS),      (IEN  | PTU | EN  | M0)) /*UART1_CTS even if input for omap as pin is connected to test point*/ \
	MUX_VAL(CP(UART1_RX),       (WAKEUP_EN  | IEN  | PTU | EN  | M0)) /*UART1_RX*/\
	/* EPD-SPI */ \
	MUX_VAL(CP(McBSP1_CLKR),    (IEN  | PTD | EN  | M4)) /*McSPI4-CLK -> TP*/ \
	MUX_VAL(CP(McBSP1_DX),      (IEN  | PTD | EN  | M4)) /*McSPI4-SIMO -> TP*/ \
	MUX_VAL(CP(McBSP1_DR),      (IEN  | PTD | EN  | M4)) /*McSPI4-SOMI -> TP*/\
	MUX_VAL(CP(McBSP1_FSX),     (IEN  | PTD | EN  | M4)) /*McSPI4-CS0 -> TP*/ \
	/*Serial Interface*/\
	MUX_VAL(CP(HSUSB0_CLK),     (IEN  | PTD | DIS | M0)) /*HSUSB0_CLK*/\
	MUX_VAL(CP(HSUSB0_STP),     (IDIS | PTU | DIS | M0)) /*HSUSB0_STP*/\
	MUX_VAL(CP(HSUSB0_DIR),     (IEN  | PTD | DIS | M0)) /*HSUSB0_DIR*/\
	MUX_VAL(CP(HSUSB0_NXT),     (IEN  | PTD | DIS | M0)) /*HSUSB0_NXT*/\
	MUX_VAL(CP(HSUSB0_DATA0),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA0 */\
	MUX_VAL(CP(HSUSB0_DATA1),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA1 */\
	MUX_VAL(CP(HSUSB0_DATA2),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA2 */\
	MUX_VAL(CP(HSUSB0_DATA3),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA3 */\
	MUX_VAL(CP(HSUSB0_DATA4),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA4 */\
	MUX_VAL(CP(HSUSB0_DATA5),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA5 */\
	MUX_VAL(CP(HSUSB0_DATA6),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA6 */\
	MUX_VAL(CP(HSUSB0_DATA7),   (IEN  | PTD | DIS | M0)) /*HSUSB0_DATA7 */\
	MUX_VAL(CP(I2C1_SCL),       (IEN  | PTU | DIS | M0)) /*I2C1_SCL*/\
	MUX_VAL(CP(I2C1_SDA),       (IEN  | PTU | DIS | M0)) /*I2C1_SDA*/\
	MUX_VAL(CP(I2C2_SCL),       (IEN  | PTU | DIS | M0)) /*I2C2_SCL (NAV_SCL)*/\
	MUX_VAL(CP(I2C2_SDA),       (IEN  | PTU | DIS | M0)) /*I2C2_SDA (NAV_SDA)*/\
	MUX_VAL(CP(I2C4_SCL),       (IEN  | PTU | DIS | M0)) /*I2C4_SCL*/\
	MUX_VAL(CP(I2C4_SDA),       (IEN  | PTU | DIS | M0)) /*I2C4_SDA*/\
	MUX_VAL(CP(McSPI1_CS3),     (IEN  | PTD | EN  | M4)) /*HSUSB2_D2 -> NC*/\
	MUX_VAL(CP(McSPI2_CLK),     (IEN  | PTD | EN  | M4)) /*HSUSB2_D7 -> NC*/\
	MUX_VAL(CP(McSPI2_SIMO),    (IEN  | PTD | EN  | M4)) /*HSUSB2_D4 -> NC*/\
	MUX_VAL(CP(McSPI2_SOMI),    (IEN  | PTD | EN  | M4)) /*HSUSB2_D5 -> NC*/\
	MUX_VAL(CP(McSPI2_CS0),     (IEN  | PTD | EN  | M4)) /*HSUSB2_D6 -> NC*/\
	MUX_VAL(CP(McSPI2_CS1),     (IEN  | PTD | EN  | M4)) /*HSUSB2_D3 -> NC*/\
	/*Control and debug */\
	MUX_VAL(CP(SYS_32K),        (IEN  | PTD | DIS | M0)) /*SYS_32K*/\
	MUX_VAL(CP(SYS_CLKREQ),     (IEN  | PTD | DIS | M0)) /*SYS_CLKREQ*/\
	MUX_VAL(CP(SYS_nIRQ),       (WAKEUP_EN  | IEN  | PTU | EN  | M0)) /*SYS_nIRQ 65921 Int*/\
	MUX_VAL(CP(SYS_BOOT0),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT0*/\
	MUX_VAL(CP(SYS_BOOT1),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT1*/\
	MUX_VAL(CP(SYS_BOOT2),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT2*/\
	MUX_VAL(CP(SYS_BOOT3),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT3*/\
	MUX_VAL(CP(SYS_BOOT4),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT4*/\
	MUX_VAL(CP(SYS_BOOT5),  (IEN  | PTD | DIS | M0)) /*SYS_BOOT5*/\
	MUX_VAL(CP(SYS_BOOT6),      (IEN  | PTD | DIS | M0)) /*SYS_BOOT6*/\
	MUX_VAL(CP(SYS_OFF_MODE),   (IDIS | PTD | DIS | M0)) /*SYS_OFF_MODE */\
	MUX_VAL(CP(SYS_CLKOUT2),    (IDIS | PTD | DIS | M0)) /*SYS_CLKOUT2	 */\
	MUX_VAL(CP(JTAG_nTRST),     (IEN  | PTD | DIS | M0)) /*JTAG_nTRST*/\
	MUX_VAL(CP(JTAG_TCK),       (IEN  | PTD | DIS | M0)) /*JTAG_TCK*/\
	MUX_VAL(CP(JTAG_TMS),       (IEN  | PTD | DIS | M0)) /*JTAG_TMS*/\
	MUX_VAL(CP(JTAG_TDI),       (IEN  | PTD | DIS | M0)) /*JTAG_TDI*/\
	MUX_VAL(CP(JTAG_TDO),  	    (IDIS | PTD | DIS | M0)) /*JTAG_TDO*/\
	MUX_VAL(CP(JTAG_RTCK),      (IDIS | PTD | DIS | M0)) /*JTAG_RTCK*/\
	MUX_VAL(CP(JTAG_EMU0),      (IDIS | PTD | DIS | M0)) /*JTAG_EMU0*/\
	MUX_VAL(CP(JTAG_EMU1),      (IDIS | PTD | DIS | M0)) /*JTAG_EMU1*/\
	MUX_VAL(CP(ETK_D10_ES2),    (IEN  | PTD | EN  | M4)) /*HSUSB2_CLK -> NC*/\
	MUX_VAL(CP(ETK_D11_ES2),    (IEN  | PTU | EN  | M4)) /*HSUSB2_STP -> NC*/\
	MUX_VAL(CP(ETK_D12_ES2),    (IEN  | PTD | EN  | M4)) /*HSUSB2_DIR -> NC*/\
	MUX_VAL(CP(ETK_D13_ES2),    (IEN  | PTD | EN  | M4)) /*HSUSB2_NXT -> NC*/\
	MUX_VAL(CP(ETK_D14_ES2),    (IEN  | PTD | EN  | M4)) /*HSUSB2_D0 -> NC*/\
	MUX_VAL(CP(ETK_D15_ES2),    (IEN  | PTD | EN  | M4)) /*HSUSB2_D1 -> NC*/\
	/* IO not connected muxed to default value to prevent spurious interrupts*/\
	MUX_VAL(CP(GPMC_nCS1),      (IEN | PTD | EN | M4)) /*GPIO-52 Not balled*/\
	MUX_VAL(CP(GPMC_nCS2),      (IEN | PTD | EN | M4)) /*GPIO-53 Not balled*/\
	MUX_VAL(CP(GPMC_nCS3),      (IEN | PTD | EN | M4)) /*GPIO-54 Not balled*/\
	MUX_VAL(CP(GPMC_nCS4),      (IEN | PTD | EN | M4)) /*GPIO-55 Not balled*/\
	MUX_VAL(CP(GPMC_nCS5),      (IEN | PTD | EN | M4)) /*GPIO-56 Not balled*/\
	MUX_VAL(CP(GPMC_nCS6),      (IEN | PTD | EN | M4)) /*GPIO-57 Not balled*/\
	MUX_VAL(CP(GPMC_WAIT1),     (IEN | PTD | EN | M4)) /*GPI0-63 Not balled*/\
	MUX_VAL(CP(GPMC_WAIT2),     (IEN | PTD | EN | M4)) /*GPI0-64 Not balled*/\
	MUX_VAL(CP(GPMC_WAIT3),     (IEN | PTD | EN | M4)) /*GPI0-65 Not balled*/\
	MUX_VAL(CP(CAM_STROBE), (IEN | PTD | EN | M4)) /*GPI0-126 Not balled*/\
	MUX_VAL(CP(CAM_WEN),    (IEN | PTD | EN | M4)) /*GPI0-167 Not balled*/\
	MUX_VAL(CP(CAM_HS),         (IEN | PTD | EN | M4)) /*GPIO-94 Not balled*/\
	MUX_VAL(CP(CAM_VS),         (IEN | PTD | EN | M4)) /*GPIO-95 Not balled*/\
	MUX_VAL(CP(CAM_PCLK),       (IEN | PTD | EN | M4)) /*GPIO-97 Not balled*/\
	MUX_VAL(CP(CAM_FLD),        (IEN | PTD | EN | M4)) /*GPIO-98 Not balled*/\
	MUX_VAL(CP(CAM_D6 ),        (IEN | PTD | EN | M4)) /*GPIO-105 Not balled*/\
	MUX_VAL(CP(CAM_D7 ),        (IEN | PTD | EN | M4)) /*GPIO-106 Not balled*/\
	MUX_VAL(CP(CAM_D8 ),        (IEN | PTD | EN | M4)) /*GPIO-107 Not balled*/\
	MUX_VAL(CP(CAM_D9 ),        (IEN | PTD | EN | M4)) /*GPIO-108 Not balled*/\
	MUX_VAL(CP(UART2_CTS),      (IEN | PTD | EN | M4)) /*GPIO-144 Not balled*/\
	MUX_VAL(CP(UART2_RTS),      (IEN | PTD | EN | M4)) /*GPIO-145 Not balled*/\
	MUX_VAL(CP(UART2_TX),       (IEN | PTD | EN | M4)) /*UPIO-146 Not balled*/\
	MUX_VAL(CP(UART2_RX),       (IEN | PTD | EN | M4)) /*GPIO-147 Not balled*/\
	/*McBSP4 Interface */\
	MUX_VAL(CP(McBSP4_CLKX),    (IEN | PTD | EN | M4)) /*GPIO-152 Not balled*/\
	MUX_VAL(CP(McBSP4_DR),      (IEN | PTD | EN | M4)) /*GPIO-153 Not balled*/\
	MUX_VAL(CP(McBSP4_FSX),     (IEN | PTD | EN | M4)) /*GPIO-154 Not balled*/\
	MUX_VAL(CP(McBSP4_DX),      (IEN | PTD | EN | M4)) /*GPIO-155 Not balled*/\
	MUX_VAL(CP(McBSP_CLKS),     (IEN | PTD | EN | M4)) /*GPIO-160 Not balled*/\
	MUX_VAL(CP(McBSP1_CLKX),    (IEN | PTD | EN | M4)) /*GPIO-162 Not balled*/\
	MUX_VAL(CP(UART3_CTS_RCTX), (IEN | PTD | EN | M4)) /*GPIO-163 Not balled*/\
	MUX_VAL(CP(UART3_RX_IRRX),  (IEN | PTD | EN | M4)) /*GPIO-164 Not balled*/\
	MUX_VAL(CP(UART3_RTS_SD),   (IEN | PTD | EN | M4)) /*GPIO-165 Not balled*/\
	MUX_VAL(CP(UART3_TX_IRTX),  (IEN | PTD | EN | M4)) /*GPIO-165 Not balled*/\
	/*I2C3 */\
	MUX_VAL(CP(I2C3_SCL),       (IEN | PTD | EN | M4)) /*GPIO_184 Not balled*/\
	MUX_VAL(CP(I2C3_SDA),       (IEN | PTD | EN | M4)) /*GPIO_185 Not balled*/\
	/*McSPI1 */\
	MUX_VAL(CP(McSPI1_SIMO),    (IEN | PTD  | EN | M4)) /*GPIO_171 Not balled*/\
	MUX_VAL(CP(McSPI1_CLK),     (IEN | PTD  | EN | M4)) /*GPIO_172 Not balled*/\
	MUX_VAL(CP(McSPI1_CS0),     (IEN | PTD  | EN | M4)) /*GPIO_173 Not balled*/\
	MUX_VAL(CP(McSPI1_SOMI),    (IEN | PTD  | EN | M4)) /*GPIO_174 Not balled*/\
	MUX_VAL(CP(McSPI1_CS1),     (IEN | PTD  | EN | M4)) /*GPIO_175 Not balled*/\
	MUX_VAL(CP(McSPI1_CS2),     (IEN | PTD  | EN | M4)) /*GPIO_176 Not balled*/\
	/*Die to Die */\
	MUX_VAL(CP(sdrc_cke0),      (IEN  | PTD | DIS | M0)) /*sdrc_cke0 */\
	MUX_VAL(CP(sdrc_cke1),      (IEN  | PTD | DIS | M0)) /*sdrc_cke1 not used*/\
	MUX_VAL(CP(d2d_mcad0),      (IEN  | PTD | EN  | M0)) /*d2d_mcad0*/\
	MUX_VAL(CP(d2d_mcad1),      (IEN  | PTD | EN  | M0)) /*d2d_mcad1*/\
	MUX_VAL(CP(d2d_mcad2),      (IEN  | PTD | EN  | M0)) /*d2d_mcad2*/\
	MUX_VAL(CP(d2d_mcad3),      (IEN  | PTD | EN  | M0)) /*d2d_mcad3*/\
	MUX_VAL(CP(d2d_mcad4),      (IEN  | PTD | EN  | M0)) /*d2d_mcad4*/\
	MUX_VAL(CP(d2d_mcad5),      (IEN  | PTD | EN  | M0)) /*d2d_mcad5*/\
	MUX_VAL(CP(d2d_mcad6),      (IEN  | PTD | EN  | M0)) /*d2d_mcad6*/\
	MUX_VAL(CP(d2d_mcad7),      (IEN  | PTD | EN  | M0)) /*d2d_mcad7*/\
	MUX_VAL(CP(d2d_mcad8),      (IEN  | PTD | EN  | M0)) /*d2d_mcad8*/\
	MUX_VAL(CP(d2d_mcad8),      (IEN  | PTD | EN  | M0)) /*d2d_mcad9*/\
	MUX_VAL(CP(d2d_mcad10),     (IEN  | PTD | EN  | M0)) /*d2d_mcad10*/\
	MUX_VAL(CP(d2d_mcad11),     (IEN  | PTD | EN  | M0)) /*d2d_mcad11*/\
	MUX_VAL(CP(d2d_mcad12),     (IEN  | PTD | EN  | M0)) /*d2d_mcad12*/\
	MUX_VAL(CP(d2d_mcad13),     (IEN  | PTD | EN  | M0)) /*d2d_mcad13*/\
	MUX_VAL(CP(d2d_mcad14),     (IEN  | PTD | EN  | M0)) /*d2d_mcad14*/\
	MUX_VAL(CP(d2d_mcad15),     (IEN  | PTD | EN  | M0)) /*d2d_mcad15*/\
	MUX_VAL(CP(d2d_mcad16),     (IEN  | PTD | EN  | M0)) /*d2d_mcad16*/\
	MUX_VAL(CP(d2d_mcad17),     (IEN  | PTD | EN  | M0)) /*d2d_mcad17*/\
	MUX_VAL(CP(d2d_mcad18),     (IEN  | PTD | EN  | M0)) /*d2d_mcad18*/\
	MUX_VAL(CP(d2d_mcad19),     (IEN  | PTD | EN  | M0)) /*d2d_mcad19*/\
	MUX_VAL(CP(d2d_mcad20),     (IEN  | PTD | EN  | M0)) /*d2d_mcad20*/\
	MUX_VAL(CP(d2d_mcad21),     (IEN  | PTD | EN  | M0)) /*d2d_mcad21*/\
	MUX_VAL(CP(d2d_mcad22),     (IEN  | PTD | EN  | M0)) /*d2d_mcad22*/\
	MUX_VAL(CP(d2d_mcad23),     (IEN  | PTD | EN  | M0)) /*d2d_mcad23*/\
	MUX_VAL(CP(d2d_mcad24),     (IEN  | PTD | EN  | M0)) /*d2d_mcad24*/\
	MUX_VAL(CP(d2d_mcad25),     (IEN  | PTD | EN  | M0)) /*d2d_mcad25*/\
	MUX_VAL(CP(d2d_mcad26),     (IEN  | PTD | EN  | M0)) /*d2d_mcad26*/\
	MUX_VAL(CP(d2d_mcad27),     (IEN  | PTD | EN  | M0)) /*d2d_mcad27*/\
	MUX_VAL(CP(d2d_mcad28),     (IEN  | PTD | EN  | M0)) /*d2d_mcad28*/\
	MUX_VAL(CP(d2d_mcad29),     (IEN  | PTD | EN  | M0)) /*d2d_mcad29*/\
	MUX_VAL(CP(d2d_mcad30),     (IEN  | PTD | EN  | M0)) /*d2d_mcad30*/\
	MUX_VAL(CP(d2d_mcad31),     (IEN  | PTD | EN  | M0)) /*d2d_mcad31*/\
	MUX_VAL(CP(d2d_mcad32),     (IEN  | PTD | EN  | M0)) /*d2d_mcad32*/\
	MUX_VAL(CP(d2d_mcad33),     (IEN  | PTD | EN  | M0)) /*d2d_mcad33*/\
	MUX_VAL(CP(d2d_mcad34),     (IEN  | PTD | EN  | M0)) /*d2d_mcad34*/\
	MUX_VAL(CP(d2d_mcad35),     (IEN  | PTD | EN  | M0)) /*d2d_mcad35*/\
	MUX_VAL(CP(d2d_mcad36),     (IEN  | PTD | EN  | M0)) /*d2d_mcad36*/\
	MUX_VAL(CP(d2d_clk26mi),    (IEN  | PTD | EN  | M0)) /*d2d_clk26msi*/\
	MUX_VAL(CP(d2d_nrespwron),  (IEN  | PTD | DIS | M0)) /*d2d_nrespwron*/\
	MUX_VAL(CP(d2d_nreswarm),   (IEN  | PTU | EN  | M0)) /*d2d_nreswarm*/\
	MUX_VAL(CP(d2d_arm9nirq),   (IEN  | PTD | DIS | M0)) /*d2d_arm9nirq*/\
	MUX_VAL(CP(d2d_uma2p6fiq),  (IEN  | PTD | DIS | M0)) /*d2d_uma2p6fi*/\
	MUX_VAL(CP(d2d_spint),      (IEN  | PTD | EN  | M0)) /*d2d_spint*/\
	MUX_VAL(CP(d2d_frint),      (IEN  | PTD | EN  | M0)) /*d2d_clk26msi*/\
	MUX_VAL(CP(d2d_dmareq0),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq0*/\
	MUX_VAL(CP(d2d_dmareq1),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq1*/\
	MUX_VAL(CP(d2d_dmareq2),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq2*/\
	MUX_VAL(CP(d2d_dmareq3),    (IEN  | PTD | EN  | M0)) /*d2d_dmareq3*/\
	MUX_VAL(CP(d2d_n3gtrst),    (IEN  | PTD | EN  | M0)) /*d2d_n3gtrst*/\
	MUX_VAL(CP(d2d_n3gtdi),     (IEN  | PTD | EN  | M0)) /*d2d_n3gtdi*/\
	MUX_VAL(CP(d2d_n3gtdo),     (IEN  | PTD | EN  | M0)) /*d2d_n3gtdo*/\
	MUX_VAL(CP(d2d_n3gtms),     (IEN  | PTD | EN  | M0)) /*d2d_n3gtms*/\
	MUX_VAL(CP(d2d_n3gtck),     (IEN  | PTD | DIS | M0)) /*d2d_n3gtck*/\
	MUX_VAL(CP(d2d_n3grtck),    (IEN  | PTD | EN  | M0)) /*d2d_rtck*/\
	MUX_VAL(CP(d2d_mstdby),     (IEN  | PTU | EN  | M0)) /*d2d_mstdby*/\
	MUX_VAL(CP(d2d_idlereq),    (IEN  | PTD | EN  | M0)) /*d2d_idlereq*/\
	MUX_VAL(CP(d2d_idleack),    (IEN  | PTU | EN  | M0)) /*d2d_idleack*/\
	MUX_VAL(CP(d2d_mwrite),     (IEN  | PTD | EN  | M0)) /*d2d_mwrite*/\
	MUX_VAL(CP(d2d_swrite),     (IEN  | PTD | EN  | M0)) /*d2d_swrite*/\
	MUX_VAL(CP(d2d_mread),      (IEN  | PTD | EN  | M0)) /*d2d_mread*/\
	MUX_VAL(CP(d2d_sread),      (IEN  | PTD | EN  | M0)) /*d2d_sread*/\
	MUX_VAL(CP(d2d_mbusflag),   (IEN  | PTD | EN  | M0)) /*d2d_mbusflag*/\
	MUX_VAL(CP(d2d_sbusflag),   (IEN  | PTD | EN  | M0)) /*d2d_sbusflag*/

/* Changes for revision pre 1C and 1C */
#define MUX_GOSSAMER_EVT1C()\
	MUX_VAL(CP(GPMC_A2),        (IEN  | PTD | DIS | M4)) /*GIO-35  HW-ID3  SKU*/\
	MUX_VAL(CP(GPMC_A4),        (IDIS | PTD | EN  | M4)) /*GIO-37  TP_PWR_nOFF - externally pulled down*/\
	MUX_VAL(CP(GPMC_A5),        (IEN  | PTD | DIS | M4)) /*GIO-38  HW-ID2  PRODUCT ID*/\
	MUX_VAL(CP(GPMC_A6),        (IEN  | PTU | EN  | M4)) /*GIO-39  NC*/\
	MUX_VAL(CP(GPMC_A7),        (IEN  | PTU | DIS | M4)) /*GIO-40  HW-ID7  HW-REV*/\
	MUX_VAL(CP(GPMC_A8),        (IEN  | PTU | DIS | M4)) /*GIO-41  HW-ID6  HW-REV*/\
	MUX_VAL(CP(GPMC_A9),        (IEN  | PTU | DIS | M4)) /*GIO-42  HW-ID5  HW-REV*/\
	MUX_VAL(CP(GPMC_A10),       (IEN  | PTU | DIS | M4)) /*GIO-43  HW-ID4  SKU*/\
	MUX_VAL(CP(GPMC_D8),        (IDIS  | PTU | DIS | M4)) /*GIO-44  CHARGER_CEN (EVT1C) externally pulled up */\
	MUX_VAL(CP(GPMC_D9),        (IDIS  | PTU | DIS | M4)) /*GIO-45  CHARGER_EN2 (EVT1C) externally pulled up */\
	MUX_VAL(CP(GPMC_D12),       (IEN  | PTU | EN  | M4)) /*GPIO48 HOME buttoni externally pulled up */\
	MUX_VAL(CP(GPMC_nBE0_CLE),  (IDIS | PTD | EN  | M4)) /*GPIO60  -> NC */\
    MUX_VAL(CP(GPMC_nWP),       (IDIS | PTD | EN  | M4)) /*GPIO62  -> NC */\
	MUX_VAL(CP(CAM_D1 ),        (IEN | PTD | WAKEUP_EN | DIS | M4)) /*GIO-100 -> BAT-INTERRUPT  */\
	MUX_VAL(CP(CAM_D2 ),        (IDIS | PTD | EN  | M4)) /*GIO-101 -> NC */\
	MUX_VAL(CP(CAM_D11),        (IEN  | PTD | EN  | M4)) /*GIO-110 -> NC */\
	MUX_VAL(CP(CSI2_DY0),       (IEN  | PTD | DIS | M4)) /*GIO-113 IR-INT externally pulled up */\
	MUX_VAL(CP(CSI2_DX1),       (IDIS | PTD | EN  | M4)) /*GIO-114 -> NC */\
	MUX_VAL(CP(GPMC_nCS7),      (IDIS | PTU | EN  | M4)) /*GPIO58  -> NC */\
	MUX_VAL(CP(DSS_DATA10),     (IEN  | PTD | DIS | M4)) /*GPIO80 -> EPD_PWR_FLT_nINT externally pulled up */\
	MUX_VAL(CP(DSS_DATA12),     (IDIS | PTD | EN  | M4)) /*GPIO82 -> EPD_PWRUP externally pulled down */\
	MUX_VAL(CP(DSS_DATA13),     (IDIS | PTD | EN  | M4)) /*DSS_DATA13 -> NC */\
	MUX_VAL(CP(DSS_DATA17),     (IDIS | PTD | EN  | M4)) /*GPIO87 -> EPD-WAKEUP externally pulled down */\
	MUX_VAL(CP(DSS_DATA20),     (IEN  | PTU | EN  | M4)) /*GPIO90 -> debug LED */\
	MUX_VAL(CP(DSS_DATA22),     (IDIS | PTD | EN  | M4)) /*GPIO92 -> BRD_CTRL0 */\
	MUX_VAL(CP(DSS_DATA23),     (IDIS | PTD | EN  | M4)) /*GPIO93 -> BRD_CTRL1 */

/**********************************************************
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers
 *              specific to the hardware. Many pins need
 *              to be moved from protect to primary mode.
 *********************************************************/
void set_muxconf_regs(void)
{
	MUX_GOSSAMER();
}

void set_muxconf_regs_late(void)
{
	MUX_GOSSAMER_EVT1C();
}
/******************************************************************************
 * Routine: update_mux()
 * Description:Update balls which are different between boards.  All should be
 *             updated to match functionality.  However, I'm only updating ones
 *             which I'll be using for now.  When power comes into play they
 *             all need updating.
 *****************************************************************************/
void update_mux(u32 btype, u32 mtype)
{
	/* NOTHING as of now... */
}

/******************************************************************************
 * Routine: get_boot_device()
 * Description:Sets the "bootdevice" variable with the name of the device from
 *             which the device booted.
 *****************************************************************************/
void get_boot_device(void)
{
	u32 boot_device = __raw_readl(0x480029c0) & 0xff;
	const char * boot_dev;
	switch(boot_device)
	{
	  case 2: boot_dev = "NAND"; break;
	  case 3: boot_dev = "OneNAND"; break;
	  case 5: boot_dev = "eMMC"; break;
	  case 6: boot_dev = "SD"; break;
	  default:
		boot_dev = "unknown";
		printf("Error, no valid booting device found for board\n");
		break;
	}
	setenv ("bootdevice",(char *)boot_dev);
	printf("Booting from %s\n", boot_dev);
}

/******************************************************************************
 * Routine: get_boot_kind()
 * Description:Sets the "resettype" variable with "warm" or "cold" depending
 *             on whether the device just came up from power off or not
 *****************************************************************************/
void get_boot_kind(void)
{
	long boot_code= *((long *)OMAP_SCRATCHPAD_BOOT_CODE);
	int warm_reset = (boot_code & 0xffff0000) == (('B' << 24) | ('M' << 16));
	setenv("resettype", warm_reset ? "warm" : "cold");
	printf("%s reset\n", getenv("resettype"));
}

#define RESET_TICK (100000) // 100ms
#define RESET_SECOND (1000000 / RESET_TICK)

// Keys to press at boot to enter recovery mode
#define RECOVERY_KEYS (BACK_KEY | PGBACK_KEY)

// Seconds keys need to be held
#define FACTORY_RESET_DELAY 4
#define FACTORY_RESET_LOOP (RESET_SECOND * FACTORY_RESET_DELAY)

void check_boot_buttons(void)
{
	unsigned int key_pad = 0;
	int user_req = 0;

	// Read the keypad and store any keys pressed during start
	tps65921_keypad_keys_pressed(&key_pad);

	// Detect factory clear keypad combination
	user_req = (key_pad & RECOVERY_KEYS) == RECOVERY_KEYS;
	if (user_req) {
		int i;
		printf("Recovery magic key combination detected\n");
		for (i = 0; i < FACTORY_RESET_LOOP; i++)
		{
			udelay(RESET_TICK);
			tps65921_keypad_keys_pressed(&key_pad);
			user_req = (key_pad & RECOVERY_KEYS) == RECOVERY_KEYS;
			if (!user_req) {
				printf("Keys held for less than %d sec.\n", FACTORY_RESET_DELAY);
				break;
			}
			if ((i % RESET_SECOND) == 0)
			{
				printf("Factory reset count = %d\n", i / RESET_SECOND);
			}
		}

		if (user_req) {
			  setenv("forcerecovery", "2");
			  printf("Booting into Factory Reset Kernel\n");
		}
		else {
			setenv("forcerecovery", "0");
			printf("Booting into Normal Kernel\n");

		     /* note: this does not currently over-write what is in the bcb.
		      * Action on forcerecovery == 0 could read back the bcb and
		      * clear it if it is currently 'recovery -- charging.zip, but
		      * this will generally only happen in development when batteries
		      * are swapped.  In this case it'll just boot into recovery and
		      * reboot again into normal OS fairly quickly, so no special
		      * code to handle that case.
		      */
		}
	}
}

#if 0
#define POWER_TICK (100000) // 100ms

// Seconds key needs to be held
#define POWER_DELAY 1
#define POWER_LOOP (POWER_DELAY * (1000000 / POWER_TICK))

/* Make sure power button is pressed for a few seconds to power up */
int check_power_button(void)
{
	int count = POWER_LOOP;
	printf("Checking for power button press\n");
	while ((gpio_pin_read(GOSSAMER_PWR_BUTTON_GPIO) == 1) && --count) {
		udelay(POWER_TICK);
	}
	return !count;
}
#endif

int get_power_button_boot_state(void)
{
	return power_button_boot;
}


#define GET_BCB_BOOT_CODE       "run autodetectmmc;fatload mmc ${mmcromdev}:2 ${loadaddr} BCB 0x1000;"
int get_bcb_boot_code(void)
{
	int bootcode;
	u8 *bcbflag = ((u8*)simple_strtoul(getenv("loadaddr"), NULL, 16)) + 0x40;
	*bcbflag = 1;	// Defaults to looking like a BCB command is set
	run_command( GET_BCB_BOOT_CODE, 0 );
	bootcode = *bcbflag;
	return bootcode;
}

