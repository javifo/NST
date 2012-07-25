/*
 * (C) Copyright 2008 - 2009 Texas Instruments.
 *
 * (C) Copyright 2010
 * MM solutions, <www.mm-sol.com>
 * Boris Todorov, <btodorov@mm-sol.com>
 *
 * Configuration settings for the 3430 TI Zoom2 board.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * High Level Configuration Options
 */
#define CONFIG_ARMCORTEXA8	1    /* This is an ARM V7 CPU core */
#define CONFIG_OMAP		1    /* in a TI OMAP core */
#define CONFIG_OMAP36XX		1    /* which is a 36XX */
#define CONFIG_OMAP34XX		1    /* reuse the 34XX setup */
#define CONFIG_OMAP3621_MM	1
#define CONFIG_OMAP3430		1    /* which is in a 3430 */
#define CONFIG_3630ZOOM3	1    /* used for Zoom3 Hynix memory and zoom3 LED configuration on boot */
#define CONFIG_3430ZOOM2	1    /* reuse Zoom2 setup */
#define CONFIG_BOARD_REVISION	1    /* Board revision */
#define CONFIG_3621GOSSAMER	        1    /* for specific IEC configuration	*/
/* TODO: Check again memory */
#define CONFIG_SDRAM_MT46H64M32LF	1	/* 2Gb Micron Memory */
#define CONFIG_FASTBOOT         1    /* Enable Fastboot support for Android */
#define PAPYRUS2_VDDH_PATCH     1  /* Temp patch for Ppyrus2 VDDH startup issue */
#include <asm/arch/cpu.h>        /* get chip and board defs */

#define CONFIG_CMD_BQ27520  1
#define CONFIG_CMD_BQ24073  1
#define HAS_BQ27520_BATTERY_MODEL 1
#define CONFIG_CMD_OMAP_EPD 1
#define DEBUG_CHARGE_LED_GPIO   90
#define DSP_RETRY_COUNT 20
#define CONFIG_TIMESTAMP 1
#define TPS65921_RTC 1

/* Clock Defines */
#define V_OSCK                  26000000  /* Clock output from T2 */

#define V_SCLK			(V_OSCK >> 0) /* PRM_CLKSRC_CTRL[SYSCLKDIV] is divider 1 in cpu/omap3/clock.c */

#define PRCM_CLK_CFG2_332MHZ  1         /* VDD2=1.2v - 166MHz DDR */
#define PRCM_PCLK_OPP2        1        /* ARM=500MHz - VDD1=1.20v */

/* PER clock options, only uncomment 1 */
/* Uncomment to run PER M2 at 2x 96MHz */
// #define CONFIG_PER_M2_192 */
/* Uncomment to run PER SGX at 192MHz */
#define CONFIG_PER_SGX_192

/* By default, run PER at 96MHz */

#undef CONFIG_USE_IRQ                 /* no support for IRQs */
#define CONFIG_MISC_INIT_R
#define CONFIG_BOARD_EARLY_CONSOLE 1

#define CONFIG_CMDLINE_TAG       1    /* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS 1
#define CONFIG_INITRD_TAG        1
#define CONFIG_REVISION_TAG      1
#define CONFIG_SERIAL_TAG        1

/*
 * Size of malloc() pool
 */
#define CFG_ENV_SIZE             SZ_128K    /* Total Size Environment Sector */
#define CFG_MALLOC_LEN           (CFG_ENV_SIZE + SZ_128K)
#define CFG_GBL_DATA_SIZE        128  /* bytes reserved for initial data */

/*
 * Hardware drivers
 */

/*
 * NS16550 Configuration:
 */
#define V_NS16550_CLK            (48000000)  /* 48 Mhz */

#define CFG_NS16550
#define CFG_NS16550_SERIAL
#define CFG_NS16550_REG_SIZE     (-4)
#define CFG_NS16550_CLK          V_NS16550_CLK
#define CONFIG_BAUDRATE          115200
#define CFG_BAUDRATE_TABLE       {115200}
#define CFG_NS16550_COM1         OMAP34XX_UART1
#define CFG_NS16550_COM2         OMAP34XX_UART2
#define CFG_NS16550_COM3         OMAP34XX_UART3

/*
 * select serial console configuration
 */
#define CONFIG_SERIAL1           1    /* UART1 on board */
#define CONFIG_CONS_INDEX        1

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_MMC              1
#define CFG_MMC_BASE            0xF0000000
#define CONFIG_DOS_PARTITION    1

#define NET_CMDS                 (CFG_CMD_DHCP|CFG_CMD_NFS|CFG_CMD_NET)

#ifndef CONFIG_OPTIONAL_NOR_POPULATED
//#define C_MSK (CFG_CMD_FLASH | CFG_CMD_IMLS)
#define C_MSK (CFG_CMD_IMLS | CFG_CMD_FLASH | CFG_CMD_NET)
#endif

/* Config CMD*/
#define CONFIG_COMMANDS		((CFG_CMD_I2C | CONFIG_CMD_DFL | \
				  CFG_CMD_FAT | CFG_CMD_MMC | CFG_CMD_DATE  ) & ~(C_MSK))

#define CONFIG_BOOTP_MASK        CONFIG_BOOTP_DEFAULT

/* this must be included AFTER the definition of CONFIG_COMMANDS (if any) */
#include <cmd_confdefs.h>

#if (CONFIG_COMMANDS & CFG_CMD_I2C)
#define CFG_I2C_SPEED            100
#define CFG_I2C_SLAVE            1
#define CFG_I2C_BUS              0
#define CFG_I2C_BUS_SELECT       1
#define CONFIG_DRIVER_OMAP34XX_I2C 1
#endif

#define NAND_MAX_CHIPS           0

#define CONFIG_BOOTDELAY         1

/*
 * Miscellaneous configurable options
 */
#define V_PROMPT                 "OMAP36XX GOSSAMER # "

#define CFG_LONGHELP             /* undef to save memory */
#define CFG_PROMPT               V_PROMPT
#define CFG_CBSIZE               256  /* Console I/O Buffer Size */
/* Print Buffer Size */
#define CFG_PBSIZE               (CFG_CBSIZE+sizeof(CFG_PROMPT)+16)
#define CFG_MAXARGS              16          /* max number of command args */
#define CFG_BARGSIZE             CFG_CBSIZE  /* Boot Argument Buffer Size */

#define CFG_MEMTEST_START        (OMAP34XX_SDRC_CS0)  /* memtest works on */
#define CFG_MEMTEST_END          (OMAP34XX_SDRC_CS0+SZ_31M)

#undef	CFG_CLKS_IN_HZ           /* everything, incl board info, in Hz */

#define CFG_LOAD_ADDR            (OMAP34XX_SDRC_CS0) /* default load address */

#define CFG_64BIT_STRTOUL        1

/* 2430 has 12 GP timers, they can be driven by the SysClk (12/13/19.2) or by
 * 32KHz clk, or from external sig. This rate is divided by a local divisor.
 */
#define V_PVT                    7

#define CFG_TIMERBASE            OMAP34XX_GPT2
#define CFG_PVT                  V_PVT  /* 2^(pvt+1) */
#define CFG_HZ                   ((V_SCLK)/(2 << CFG_PVT))

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	SZ_128K /* regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	SZ_4K   /* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	SZ_4K   /* FIQ stack */
#endif

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	2       /* CS1 may or may not be populated */
#define PHYS_SDRAM_1		OMAP34XX_SDRC_CS0
#define PHYS_SDRAM_1_SIZE	SZ_32M            /* at least 32 meg */
#define PHYS_SDRAM_2		OMAP34XX_SDRC_CS1

/* SDRAM Bank Allocation method */
/*#define SDRC_B_R_C		1 */
/*#define SDRC_B1_R_B0_C	1 */
#define SDRC_R_B_C		1

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */

/* **** PISMO SUPPORT *** */

/* Configure the PISMO */
/** REMOVE ME ***/
#define PISMO1_NOR_SIZE_SDPV2	GPMC_SIZE_128M
#define PISMO1_NOR_SIZE		GPMC_SIZE_64M

#define PISMO1_NAND_SIZE	GPMC_SIZE_128M
#define PISMO1_ONEN_SIZE	GPMC_SIZE_128M
#define DBG_MPDB_SIZE		GPMC_SIZE_16M
#define PISMO2_SIZE		0
#define SERIAL_TL16CP754C_SIZE	GPMC_SIZE_16M

#define CFG_MAX_FLASH_SECT	(520)		/* max number of sectors on one chip */
#define CFG_MAX_FLASH_BANKS      2		/* max number of flash banks */
#define CFG_MONITOR_LEN		SZ_256K 	/* Reserve 2 sectors */

#define PHYS_FLASH_SIZE_SDPV2	SZ_128M
#define PHYS_FLASH_SIZE		SZ_32M

#define CFG_FLASH_BASE		boot_flash_base
#define PHYS_FLASH_SECT_SIZE	boot_flash_sec
/* Dummy declaration of flash banks to get compilation right */
#define CFG_FLASH_BANKS_LIST	{0, 0}

#define CFG_MONITOR_BASE	CFG_FLASH_BASE /* Monitor at start of flash */

#define CFG_ENV_IS_IN_EMMC	0
#ifdef CFG_ENV_IS_IN_EMMC
#define SMNAND_ENV_OFFSET	0x1c0000 /* environment starts here  */
#define CFG_ENV_SECT_SIZE	boot_flash_sec
#define CFG_ENV_OFFSET		boot_flash_off
#endif
#define CFG_ENV_MMC_CONT        1

#define CFG_ENV_ADDR		(0x700)

#define CFG_BOOTCMDBLK_ADDR     (0x480028E0)  /* scratchpad memmory */

#define OMAP_SCRATCHPAD (0x48002000 + 0x910 )
#define OMAP_SCRATCHPAD_BOOT_CODE (OMAP_SCRATCHPAD+4)

/*-----------------------------------------------------------------------
 * CFI FLASH driver setup
 */
#ifndef CONFIG_OPTIONAL_NOR_POPULATED
#define CFG_NO_FLASH	1            /* Disable NOR Flash support */
#else
#define CFG_FLASH_CFI		1    /* Flash memory is CFI compliant */
#define CFG_FLASH_CFI_DRIVER	1    /* Use drivers/cfi_flash.c */
#if (!ENV_IS_VARIABLE)
/* saveenv fails when this variable is defined. 
   If env is variable, do not use buffered writes */
#define CFG_FLASH_USE_BUFFER_WRITE 1    /* Use buffered writes (~10x faster) */
#endif
#define CFG_FLASH_PROTECTION	1    /* Use hardware sector protection */
#define CFG_FLASH_QUIET_TEST	1    /* Dont crib abt missing chips */
#define CFG_FLASH_CFI_WIDTH	0x02
/* timeout values are in ticks */
#define CFG_FLASH_ERASE_TOUT	(100*CFG_HZ) /* Timeout for Flash Erase */
#define CFG_FLASH_WRITE_TOUT	(100*CFG_HZ) /* Timeout for Flash Write */

/* Flash banks JFFS2 should use */
#define CFG_MAX_MTD_BANKS	(CFG_MAX_FLASH_BANKS+CFG_MAX_NAND_DEVICE)
#define CFG_JFFS2_MEM_NAND
#define CFG_JFFS2_FIRST_BANK	CFG_MAX_FLASH_BANKS /* use flash_info[2] */
#define CFG_JFFS2_NUM_BANKS	1
#define CONFIG_LED_INFOnand_read_buf16
#define CONFIG_LED_LEN		16
#endif  /* optional NOR flash */

#ifndef __ASSEMBLY__
extern unsigned int nand_cs_base;
extern unsigned int boot_flash_base;
extern volatile unsigned int boot_flash_env_addr;
extern unsigned int boot_flash_off;
extern unsigned int boot_flash_sec;
extern unsigned int boot_flash_type;
#endif

#define WRITE_NAND_COMMAND(d, adr) __raw_writew(d, (nand_cs_base + GPMC_NAND_CMD))
#define WRITE_NAND_ADDRESS(d, adr) __raw_writew(d, (nand_cs_base + GPMC_NAND_ADR))
#define WRITE_NAND(d, adr) __raw_writew(d, (nand_cs_base + GPMC_NAND_DAT))
#define READ_NAND(adr) __raw_readw((nand_cs_base + GPMC_NAND_DAT))

/* Other NAND Access APIs */
#define NAND_WP_OFF()  do {*(volatile u32 *)(GPMC_CONFIG) |= 0x00000010;} while(0)
#define NAND_WP_ON()  do {*(volatile u32 *)(GPMC_CONFIG) &= ~0x00000010;} while(0)
#define NAND_DISABLE_CE(nand)
#define NAND_ENABLE_CE(nand)
#define NAND_WAIT_READY(nand)	udelay(10)

/* Fastboot variables */
#define CFG_FASTBOOT_MMC_NO               1    /* Use eMMC */
#define CFG_FASTBOOT_TRANSFER_BUFFER      (PHYS_SDRAM_1 + SZ_16M)
#define CFG_FASTBOOT_TRANSFER_BUFFER_SIZE (SZ_256M - SZ_16M)
#define CFG_FASTBOOT_PREBOOT_KEYS         1
#define CFG_FASTBOOT_PREBOOT_KEY1         0x37 /* 'ok' */
#define CFG_FASTBOOT_PREBOOT_KEY2         0x00 /* unused */
#define CFG_FASTBOOT_PREBOOT_INITIAL_WAIT (0)
#define CFG_FASTBOOT_PREBOOT_LOOP_MAXIMUM (1)
#define CFG_FASTBOOT_PREBOOT_LOOP_WAIT    (0)

/* Command shell */
#define CFG_HUSH_PARSER 1
#define CFG_PROMPT_HUSH_PS2	V_PROMPT

/* Clock command */
#define CONFIG_CMD_CLOCK		1
#define CONFIG_CMD_CLOCK_INFO_CPU	1

#if CFG_HUSH_PARSER
  #define CONFIG_BOOTCOMMAND \
"run autodetectmmc;" \
"if run loadbootscript; then " \
 "run bootscript;" \
"else " \
 "run readtokens; run checkbootcount; run checkrom; run checkupdate; run checkbcb; run ${bootvar};" \
"fi"

  #define CONFIG_EXTRA_ENV_SETTINGS \
"loadaddr=0x81c00000" \
	"\0" \
\
"loadaddrrd=0x81f00000" \
	"\0" \
\
"loadbootscript=fatload mmc ${mmcbootdev} ${loadaddr} boot.scr" \
	"\0" \
\
"bootscript=echo Running bootscript from ${bootdevice}...; " \
	"autoscr ${loadaddr}" \
	"\0" \
\
"mmcbootdev=0" \
	"\0" \
\
"mmcromdev=0" \
	"\0" \
\
"forcerecovery=0" \
	"\0" \
\
"bootvar=normalboot" \
	"\0" \
\
"vcom=-1990" \
	"\0" \
\
"commonbootargs=console=ttyS0,115200n8 initrd rw init=/init vram=16M video=omap3epfb:mode=800x600x16x14x270x0,pmic=${epd_pmic},vcom=${vcom} androidboot.console=ttyS0" \
	"\0" \
\
"setbootargs=setenv indirectba setenv bootargs $commonbootargs;" \
"run indirectba" \
	"\0" \
\
"recoveryboot=\
echo Booting into recovery mode;\
run setbootargs;\
mmcinit $mmcbootdev; fatload mmc $mmcbootdev ${loadaddr} uRecImg; fatload mmc $mmcbootdev ${loadaddrrd} uRecRam; bootm ${loadaddr} ${loadaddrrd}" \
	"\0" \
\
"normalboot=\
echo Booting into Android;\
if fatload mmc ${mmcromdev}:2 0x81c00000 devconf/BootCnt 4; then\
 calc.l *0x81c00000 + 1 0x81c00000;\
 fatsave mmc ${mmcromdev}:2 0x81c00000 devconf/BootCnt 4;\
fi;\
run setbootargs;\
mmcinit $mmcbootdev; fatload mmc $mmcbootdev ${loadaddr} uImage; fatload mmc $mmcbootdev ${loadaddrrd} uRamdisk; bootm ${loadaddr} ${loadaddrrd}" \
	"\0" \
\
"autodetectmmc=if itest.s ${bootdevice} == \"SD\"; then\
 setenv mmcbootdev 0;\
 setenv mmcromdev 0;\
else\
 setenv mmcbootdev 1;\
 setenv mmcromdev 1;\
 setenv mmcrootdev 1;\
fi;\
mmcinit $mmcbootdev"\
	"\0" \
\
"readtokens=if fatload mmc ${mmcromdev}:2 0x81c00000 devconf/DeviceID 100; then\
 setenvmem serialnum 0x81c00000 0x$filesize;\
fi;\
if fatload mmc ${mmcromdev}:2 0x81c00000 devconf/EpdVcom 20; then\
 setenvmem vcom 0x81c00000 0x$filesize;\
fi" \
	"\0" \
\
"checkbootcount=\
 if fatload mmc ${mmcromdev}:2 0x81c00000 devconf/BootCnt 4; then\
  calc.l *0x81c00000 + 1 0x81c00000;\
  if itest.l *0x81c00000 < 0; then\
   echo Bad Boot count (Fixed);\
   mw.l 0x81c00000 0 1;\
   fatsave mmc ${mmcromdev}:2 0x81c00000 devconf/BootCnt 4;\
  fi;\
  if itest.l *0x81c00000 > 7; then\
   echo Boot count exceeded;\
   echo Forced to boot into recovery mode for Factory Fallback; \
   mw.b 0x81c00000 0 0x200;\
   cp.s \"boot-recovery\" 0x81c00000 0xd;\
   cp.s \"recovery --restore=factory --update_package=MISC:factory.zip\" 0x81c00040 0x3d;\
   mw.b 0x81c00048 0xa 1; mw.b 0x81c0005a 0xa 1;\
   fatsave mmc ${mmcromdev}:2 0x81c00000 BCB 0x200;\
   mw.l 0x81c00000 0 1;\
   fatsave mmc ${mmcromdev}:2 0x81c00000 devconf/BootCnt 4;\
  else\
   echo Boot count OK;\
  fi;\
fi" \
	"\0" \
\
"bcbcharging=if fatload mmc ${mmcromdev}:2 0x81c00000 BCB 0x1000; then\
 mw.b 0x81c00000 0 0x200;\
 cp.s \"boot-recovery\" 0x81c00000 0xd;\
 cp.s \"recovery --update_package=BOOT:charging.zip\" 0x81c00040 0x2c;\
 mw.b 0x81c00048 0xa 1;\
 fatsave mmc ${mmcromdev}:2 0x81c00000 BCB 0x200;\
fi" \
	"\0" \
 \
"bcbreset=if fatload mmc ${mmcromdev}:2 0x81c00000 BCB 0x1000; then\
 mw.b 0x81c00000 0 0x200;\
 cp.s \"boot-recovery\" 0x81c00000 0xd;\
 cp.s \"recovery --wipe_data_ui\" 0x81c00040 0x19;\
 mw.b 0x81c00048 0xa 1;\
 fatsave mmc ${mmcromdev}:2 0x81c00000 BCB 0x200;\
fi" \
	"\0" \
\
"checkbcb=mmcinit $mmcbootdev;\
if itest ${forcerecovery} -eq 1; then \
 echo Forced to boot into recovery mode for charging; \
 run bcbcharging; \
 setenv bootvar recoveryboot;\
fi;  \
if itest ${forcerecovery} -eq 2; then \
  echo Forced to boot into recovery mode for reset; \
  run bcbreset; \
  setenv bootvar recoveryboot;\
fi;  \
fatload mmc ${mmcromdev}:2 0x81c00000 BCB 0x1000;\
if itest.l $? -ne 0; then\
  echo Missing BCB forces recovery mode;\
  setenv bootvar recoveryboot;\
elif itest.l 0x$filesize -lt 0x200; then\
  echo Empty BCB forces recovery mode;\
  setenv bootvar recoveryboot;\
elif itest.b *0x81c00040 -ne 0 &&\
 itest.s *0x81c00040 == \"recovery\"; then\
  echo BCB forces recovery mode;\
  setenv bootvar recoveryboot;\
fi" \
	"\0" \
\
"checkupdate=mmcinit 0;\
if itest.s ${resettype} -eq \"cold\"; then\
 if fatload mmc 0:1 0x81c00000 gossamer_update.zip 1; then\
  echo Forced to boot into recovery mode for SD update; \
  mw.b 0x81c00000 0 0x200;\
  cp.s \"boot-recovery\" 0x81c00000 0xd;\
  cp.s \"recovery --update_package=SDCARD:gossamer_update.zip\" 0x81c00040 0x35;\
  mw.b 0x81c00048 0xa 1;\
  mmcinit ${mmcromdev};\
  fatsave mmc ${mmcromdev}:2 0x81c00000 BCB 0x200;\
 else\
  mmcinit ${mmcromdev};\
 fi;\
fi" \
	"\0" \
\
"checkrom=mmcinit ${mmcromdev};\
fatload mmc ${mmcromdev}:2 0x81c00000 devconf/DeviceID 1;\
setenv badrom $?;\
fatload mmc ${mmcromdev}:2 0x81c00000 devconf/BootCnt 4;\
if itest.l $? -ne 0 ||\
   itest.l 0x$filesize -ne 0x4 ||\
   itest.l $badrom -ne 0; then\
 echo Forced to boot into recovery mode for ROM restore; \
 mw.b 0x81c00000 0 0x200;\
 cp.s \"boot-recovery\" 0x81c00000 0xd;\
 cp.s \"recovery --restore=rom --update_package=BOOT:romrestore.zip\" 0x81c00040 0x3c;\
 mw.b 0x81c00048 0xa 1; mw.b 0x81c00056 0xa 1;\
 fatsave mmc ${mmcromdev}:2 0x81c00000 BCB 0x200;\
fi" \
	"\0" \


#else
   #define CONFIG_BOOTCOMMAND "setenv bootargs $bootargs boardtype=$boardtype; mmcinit 0; fatload mmc 0 0x81c00000 uImage; bootm 0x81c00000;"
   #define CONFIG_BOOTARGS "console=ttyO0,115200n8 androidboot.console=ttyO0 root=/dev/mmcblk0p2 rw rootwait mem=256M init=/init video=omap3epfb:mode=800x600x16x9x0,deferred_io=400,pmic_dwell=150,vcom=-1860"
#endif	 /* CFG_HUSH_PARSER */

#endif                           /* __CONFIG_H */
