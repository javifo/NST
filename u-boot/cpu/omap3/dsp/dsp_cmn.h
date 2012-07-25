
#ifndef DSP_CMN_H
#define DSP_CMN_H

// KNOWN TARGETS : CONFIG_OMAP3_BEAGLE. CONFIG_3630EDP1, CONFIG_3621EDP1, CONFIG_3621GOSSAMER.

#undef SPLASH_SCREEN_DEBUG

#define uint8	uint8_t
#define uint32	uint32_t

// Timeouts ...
#define	UDELAY_1ms	(1000)

/*******************************************/
#if defined(CONFIG_OMAP3_BEAGLE)
	#define	CFG_I2C_SPEED	100
#else ///----
	extern int select_bus(int bus, int speed);
	#define i2c_set_bus_num(bus)	select_bus((bus), CFG_I2C_SPEED)
#endif
/*******************************************/
extern block_dev_desc_t *get_dev (char* ifname, int dev);

#endif
