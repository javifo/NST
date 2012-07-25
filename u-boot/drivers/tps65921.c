#include <common.h>
#include <tps65921.h>
#if defined(TPS65921_RTC)
#include <rtc.h>
#endif

#ifdef DEBUG
#  define DPRINTF(x,args...)	printf("%s: " x , __FILE__, ##args)
#else
#  define DPRINTF(x,args...)
#endif

int tps65921_i2c_read(u8 mod, u8 reg, u8 * data)
{
	select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);

	return i2c_read(mod, reg, 1, data, 1);
}

int tps65921_i2c_write(u8 mod, u8 reg, u8 * data)
{
	select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);

	return i2c_write(mod, reg, 1, data, 1);
}

int tps65921_get_charger_amps(void)
{
	u8 data;
	int stat;
	int ret = -1;

	udelay(CHARGER_DETECT_DWELL_TIME_US);

	stat = tps65921_i2c_read(TWL4030_CHIP_MAIN_CHARGE, TPS65921_USB_DTCT_CTRL, &data);
	if (stat != 0) {
		printf("Can't read DTCT\n");
		return -1;
	}

	data &= TPS65921_USB_DET_STS_MASK;
	switch (data) {
	case TPS65921_USB_DET_STS_100:
		ret = TPS65921_USB_DET_STS_100;
		break;
	case TPS65921_USB_DET_STS_500:
		ret = TPS65921_USB_DET_STS_500;
		break;
	case TPS65921_USB_DET_STS_1000:
		ret = TPS65921_USB_DET_STS_1000;
		break;
	default:
		ret = TPS65921_USB_DET_STS_NONE;
	}

	return ret;
}

int tps65921_shutdown(char * reason)
{
	u8 data;

	printf("Shutting down (%s)...\n", reason);

	select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);
	/* put PM into WAIT_ON state */
	data = 0x01;
	i2c_write(TWL4030_CHIP_BACKUP, 0x46, 1, &data, 1);

	while(1);

	return 0;
}

int tps65921_enable_charger_detect(void)
{
	u8 data = 0;
	int stat;

	/* TPS65921 errata here */
	stat = tps65921_i2c_read(TWL4030_CHIP_MAIN_CHARGE, TPS65921_USB_DTCT_CTRL, &data);
	data |= TPS65921_USB_SW_CHRG_CTRL_EN;
	stat |= tps65921_i2c_write(TWL4030_CHIP_MAIN_CHARGE,
			TPS65921_USB_DTCT_CTRL, &data);

	data |= TPS65921_USB_HW_CHRG_DET_EN;
	data &= ~TPS65921_USB_SW_CHRG_CTRL_EN;
	stat |= tps65921_i2c_write(TWL4030_CHIP_MAIN_CHARGE,
			TPS65921_USB_DTCT_CTRL, &data);

	if (stat != 0)
		printf("Error detecting charger\n");

	return stat;
}

int tps65921_enable_usb_power(void)
{
	u8 data;
	int stat;

	stat = tps65921_i2c_read(TWL4030_CHIP_PM_MASTER, TPS65921_VUSB3V1_DEV_GRP, &data);
	data |= 0xE0;
	stat |= tps65921_i2c_write(TWL4030_CHIP_PM_MASTER, TPS65921_VUSB3V1_DEV_GRP, &data);

	stat |= tps65921_i2c_read(TWL4030_CHIP_PM_MASTER, TPS65921_VUSB1V8_DEV_GRP,&data);
	data |= 0xE0;
	stat |= tps65921_i2c_write(TWL4030_CHIP_PM_MASTER, TPS65921_VUSB1V8_DEV_GRP, &data);

	stat |= tps65921_i2c_read(TWL4030_CHIP_PM_MASTER, TPS65921_VUSB1V5_DEV_GRP, &data);
	data |= 0xE0;
	stat |= tps65921_i2c_write(TWL4030_CHIP_PM_MASTER, TPS65921_VUSB1V5_DEV_GRP, &data);

	if (stat != 0)
		printf("Error powering USB\n");

	return stat;
}

int tps65921_disable_usb_transceiver(void)
{
	u8 data;
	int stat;

	stat = tps65921_i2c_read(TWL4030_CHIP_USB, TPS65921_USB_FUNC_CTRL, &data);
	data &= ~TPS65921_FUNC_CTRL_USB_NON_DRIVING_MASK;
	data |= TPS65921_FUNC_CTRL_USB_NON_DRIVING;
	stat |= tps65921_i2c_write(TWL4030_CHIP_USB, TPS65921_USB_FUNC_CTRL, &data);

	if (stat != 0)
		printf("Error disabling USB transceiver\n");

	return stat;
}




int tps65921_is_rtc_running(void)
{
    unsigned char val;
    select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);
    /* get the time*/
    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);
    return (val&RTC_CTRL_RTC_ENABLE)?1:0;
}

void tps65921_set_watchdog(int time)
{
    unsigned char val;
    val = time;

    select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);
    i2c_write(TWL4030_CHIP_PM_MASTER, PM_MASTER_WATCHDOG_CFG, 1, &val, 1);
}

int tps65921_get_watchdog()
{
    unsigned char val;
    val = 0;

    select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);
    i2c_read(TWL4030_CHIP_PM_MASTER, PM_MASTER_WATCHDOG_CFG, 1, &val, 1);
    return val;
}


void tps65921_stopon_pwr_button(int state)
{
    unsigned char val;

    select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);
    /* Enable writing to power configuration registers */

#if 0
    val=0xC0; val=0xce;
    i2c_write(TWL4030_CHIP_PM_MASTER, PM_MASTER_PROTECT_KEY, 1, &val, 1);
    val=0x0c; val=0xec;
    i2c_write(TWL4030_CHIP_PM_MASTER, PM_MASTER_PROTECT_KEY, 1, &val, 1);
#endif
    val = (state)?SW_EVENTS_STOPON_PWRON:0;
    i2c_write(TWL4030_CHIP_PM_MASTER, PM_MASTER_P1_SW_EVENTS, 1, &val, 1);

#if 0
    /* disable access to power configuration registers */
    val=0x00;
    i2c_write(TWL4030_CHIP_PM_MASTER, PM_MASTER_PROTECT_KEY, 1, &val, 1);
#endif
}

#if defined(TPS65921_RTC)
void rtc_get(struct rtc_time *tmp)
{
    unsigned char val;
    unsigned char  seconds,minutes,hours,days,months,years,weeks;

    select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);
    /* get the time*/
    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);
    val|=RTC_CTRL_GET_TIME;
    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);

    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_SECONDS_REG , 1, &seconds , 1);
    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_MINUTES_REG , 1, &minutes , 1);
    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_HOURS_REG   , 1, &hours   , 1);
    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_DAYS_REG    , 1, &days    , 1);
    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_MONTHS_REG  , 1, &months  , 1);
    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_YEARS_REG   , 1, &years   , 1);
    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_WEEKS_REG   , 1, &weeks   , 1);

	tmp->tm_sec=seconds;
	tmp->tm_min=minutes;
	tmp->tm_hour=hours;
	tmp->tm_wday=weeks;
	tmp->tm_mday=days;
	tmp->tm_mon=months;
	tmp->tm_year=years;

	tmp->tm_yday = 0;
	tmp->tm_isdst= 0;

	DPRINTF("Get DATE: %4d-%02d-%02d (wday=%d)  TIME: %2d:%02d:%02d\n",
		tmp->tm_year, tmp->tm_mon, tmp->tm_mday, tmp->tm_wday,
		tmp->tm_hour, tmp->tm_min, tmp->tm_sec );

}

void rtc_reset (void)
{
    unsigned char val;

    select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);

    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);
    val|=RTC_CTRL_RTC_ENABLE;
    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);
}

void rtc_set (struct rtc_time *tmp)
{
    unsigned char val;
    unsigned char  seconds,minutes,hours,days,months,years,weeks;

    DPRINTF("rtc_set not tested\n");

    seconds  =  tmp->tm_sec ;
    minutes  =  tmp->tm_min ;
    hours    =  tmp->tm_hour;
    weeks    =  tmp->tm_wday;
    days     =  tmp->tm_mday;
    months   =  tmp->tm_mon ;
    years    =  tmp->tm_year;

    select_bus(TPS65921_I2C_BUS_NUM, CFG_I2C_SPEED);


    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);
    val&=RTC_CTRL_RTC_ENABLE;
    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);

    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_SECONDS_REG , 1, &seconds , 1);
    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_MINUTES_REG , 1, &minutes , 1);
    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_HOURS_REG   , 1, &hours   , 1);
    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_DAYS_REG    , 1, &days    , 1);
    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_MONTHS_REG  , 1, &months  , 1);
    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_YEARS_REG   , 1, &years   , 1);
    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_WEEKS_REG   , 1, &weeks   , 1);

    i2c_read(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);
    val|=RTC_CTRL_RTC_ENABLE;
    i2c_write(TWL4030_CHIP_RTC, TWL4030_BASEADD_RTC + RTC_CTRL_REG, 1, &val, 1);
}

#endif
