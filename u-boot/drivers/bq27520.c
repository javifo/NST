#include <bq27520.h>

#if defined(CONFIG_DRIVER_OMAP24XX_I2C) || defined(CONFIG_DRIVER_OMAP34XX_I2C)
int i2c_multidata_write(uchar chip, uint addr, int alen, uchar * buffer, int len);
int i2c_read_2_byte(u8 devaddr, u8 regoffset, u8 * value);
#endif

static u8 bq27520_compute_checksum(u8 *block, int length)
{
	u8 checksum = 0;
	int i;

	for (i=0; i < length; i++) {
		checksum += block[i];
	}

	return (~checksum & 0xFF);
}

static int bq27520_i2c_read(u8 reg, u16 * data)
{
	int status;

    *data = 0;
	select_bus(BQ27520_I2C_BUS_NUM, CFG_I2C_SPEED);

	status = i2c_read(BQ27520_I2C_ADDR, reg, 1, data, 2);
	if (status)
		printf("i2c_read error in %s\n", __FUNCTION__);

	return status;
}


static int bq27520_i2c_readn(u8 reg, u8 * data,int n)
{
	int status;

	select_bus(BQ27520_I2C_BUS_NUM, CFG_I2C_SPEED);

	status = i2c_read(BQ27520_I2C_ADDR, reg, 1, data, n);
	if (status)
		printf("i2c_read error in %s\n", __FUNCTION__);

	return status;
}

static int bq27520_i2c_write_reg(u8 reg, u8 data)
{
	int status;

	select_bus(BQ27520_I2C_BUS_NUM, CFG_I2C_SPEED);
	status = i2c_write(BQ27520_I2C_ADDR, reg, 1, &data, 1);
	if (status)
		printf("i2c_read error in %s\n", __FUNCTION__);

	return status;
}


int bq27520_get_state_of_charge(u16 * soc)
{
	int status;

	status = bq27520_i2c_read(BQ27520_REG_STATE_OF_CHARGE, soc);
	if (status)
		printf("i2c_read error in %s\n", __FUNCTION__);

	return status;
}

int bq27520_battery_present(void)
{
	u16 status;

	if (bq27520_i2c_read(BQ27520_REG_FLAGS, &status))
		return -1;

	printf("bq27520 status 0x%x\n",status);

	return (status & BQ27520_FLAG_BAT_DET) ? 1 : 0;
}

int bq27520_get_voltage(u16 * voltage)
{
	return bq27520_i2c_read(BQ27520_REG_VOLTAGE, voltage);
}

int bq27520_get_current(s16 * current)
{
	return bq27520_i2c_read(BQ27520_AVG_CURRENT, current);
}

int bq27520_get_nominal_available_capacity(u16 *nc)
{
	return bq27520_i2c_read(BQ27510_REG_NOMINAL_AVAILABLE_CAPACITY,(u16 *) nc);
}

int bq27520_get_remaining_capacity(u16 *nc)
{
	return bq27520_i2c_read(BQ27510_REMAINING_CAPACITY,(u16 *) nc);
}

int bq27520_get_normalized_impedance(u16 *nc)
{
	return bq27520_i2c_read(BQ27510_REG_NORMALIZED_IMPEDANCE_CAL,(u16 *) nc);
}

#if defined(CONFIG_DRIVER_OMAP24XX_I2C) || defined(CONFIG_DRIVER_OMAP34XX_I2C)

static int bq27520_i2c_control_read(u16 ctrl_reg, u16 * data)
{
	int err;
    char buffer[8];
    unsigned char data_in[8];

	select_bus(BQ27520_I2C_BUS_NUM, CFG_I2C_SPEED);

	if (data!=NULL) {
		data[0]=0;
		data[1]=0;
	}

    buffer[0]=(ctrl_reg&0x00ff);
    buffer[1]=(ctrl_reg&0xff00)>>8;

    err = i2c_multidata_write(BQ27520_I2C_ADDR, BQ27520_REG_CONTROL, 1, buffer, 2);
	if (err) {
		printf("i2c_read error in %s unable to write control\n", __FUNCTION__);
    }
    else {
		if (data!=NULL) {
			udelay(2 * 1000);
			// printf("i2c 0x%x device 0x%x muti byte write 0x%x 0x%x\n",BQ27520_I2C_ADDR,BQ27520_REG_CONTROL, buffer[0],buffer[1]);
			err = i2c_read_2_byte(BQ27520_I2C_ADDR, BQ27520_REG_CONTROL, data_in);
			if (!err) {
			//    printf("control data reg %d read %02x %02x\n",ctrl_reg,data_in[0],data_in[1]);
				*data=(u16) ((data_in[0])|(((unsigned int) data_in[1])<<8));
			}
			else {
				printf("i2c error reading control data for register 0x%x",ctrl_reg);
			}
		}
    }
    udelay(1 * 1000);

	return err;
}

int bq27520_get_hw_type(u16 *hwtype)
{
    return bq27520_i2c_control_read(BQ27520_CONTROL_DEVICE_TYPE, hwtype);
}


int bq27520_get_hw_version(u16 *hwversion)
{
    return bq27520_i2c_control_read(BQ27520_CONTROL_HW_VERSION, hwversion);
}

int bq27520_get_fw_version(u16 *fwversion)
{
    return bq27520_i2c_control_read(BQ27520_CONTROL_FW_VERSION, fwversion);
}

int bq27520_get_control_register(u16 control, u16 *data)
{
    return bq27520_i2c_control_read(control, data);
}

int bq27520_get_data_block(int dbclass,int block, u8 *data)
{
    int status = 0;
    status = bq27520_i2c_write_reg(BQ27520_REG_BLOCKDATACONTROL, 0);
    if (status) { printf("error %s line %d\n",__FUNCTION__,__LINE__); //return status; }
    }
    udelay(2 * 1000);
    status = bq27520_i2c_write_reg(BQ27520_DATAFLASHCLASS, dbclass);
    if (status) { printf("error %s line %d\n",__FUNCTION__,__LINE__); //return status; }
    }
    udelay(2 * 1000);
    status = bq27520_i2c_write_reg(BQ27520_DATAFLASHBLOCK, block);
    if (status) { printf("error %s line %d\n",__FUNCTION__,__LINE__); //return status; }
    }
    udelay(2 * 1000);
    status = bq27520_i2c_write_reg(BQ27520_REG_BLOCKDATACONTROL, 0);
    if (status) { printf("error %s line %d\n",__FUNCTION__,__LINE__); //return status; }
    }
    udelay(2 * 1000);
    status = bq27520_i2c_readn(BQ27520_BLOCKDATA_BASE,data,BQ27520_BLOCKDATA_LEN);
    if (status) { printf("error %s line %d\n",__FUNCTION__,__LINE__); //return status; }
    }
    udelay(2 * 1000);
    return status;
}

int bq27520_write_data_block(u8 *data)
{
	int status = 0;
	u8 checksum;

	checksum = bq27520_compute_checksum(data, BQ27520_BLOCKDATA_LEN);
	status = bq27520_i2c_write_reg(BQ27520_REG_BLOCKDATACHECKSUM, checksum);
	if (status) {
		printf("error %s line %d\n", __FUNCTION__, __LINE__);
	}
    udelay(BQ27520_BLOCKDATA_LEN * 1000);

	return status;
}

int bq27520_disable_batlspuen(void)
{
	u8 data[BQ27520_BLOCKDATA_LEN];

    if (bq27520_get_data_block(BQ27520_OP_CFG_CLASS, BQ27520_OP_CFG_BLOCK, data)) {
		printf("Error reading Operation configuration data block\n");
		return -1;
	}

	if (data[BQ27520_OP_CFG_C_OFF] & BQ27520_BATLSPUEN_MASK) {
		data[BQ27520_OP_CFG_C_OFF] &= ~BQ27520_BATLSPUEN_MASK;

		if (bq27520_i2c_write_reg(BQ27520_BLOCKDATA_BASE + BQ27520_OP_CFG_C_OFF, data[BQ27520_OP_CFG_C_OFF])) {
			printf("Error erasing BATLSPUEN bit from config area\n");
			return -1;
		}
        udelay(2 * 1000);
		if (bq27520_write_data_block(data)) {
			printf("Error writing back configuration data block to data flash\n");
			return -1;
		}
        udelay(500 * 1000);
		printf("battery lspuen disabled\n");
	}

	return 0;
}



int bq27520_enable_batlspuen(void)
{
	u8 data[BQ27520_BLOCKDATA_LEN];

    if (bq27520_get_data_block(BQ27520_OP_CFG_CLASS, BQ27520_OP_CFG_BLOCK, data)) {
		printf("Error reading Operation configuration data block\n");
		return -1;
	}

	if (!(data[BQ27520_OP_CFG_C_OFF] & BQ27520_BATLSPUEN_MASK)) {
		data[BQ27520_OP_CFG_C_OFF] |= BQ27520_BATLSPUEN_MASK;

		if (bq27520_i2c_write_reg(BQ27520_BLOCKDATA_BASE + BQ27520_OP_CFG_C_OFF, data[BQ27520_OP_CFG_C_OFF])) {
			printf("Error erasing BATLSPUEN bit from config area\n");
			return -1;
		}
        udelay(2 * 1000);
		if (bq27520_write_data_block(data)) {
			printf("Error writing back configuration data block to data flash\n");
			return -1;
		}
        udelay(600 * 1000);
		printf("battery lspuen enabled\n");
	}

	return 0;
}


int bq27520_set_configuration_b(unsigned int mask,unsigned int value)
{
	u8 data[BQ27520_BLOCKDATA_LEN];

    if (bq27520_get_data_block(BQ27520_OP_CFG_CLASS, BQ27520_OP_CFG_BLOCK, data)) {
		printf("Error reading Operation configuration data block\n");
		return -1;
	}

	if ((data[BQ27520_OP_CFG_B_OFF] & mask)!=value) {
		data[BQ27520_OP_CFG_B_OFF] &= ~mask;
        data[BQ27520_OP_CFG_B_OFF] |= value;

		if (bq27520_i2c_write_reg(BQ27520_BLOCKDATA_BASE + BQ27520_OP_CFG_B_OFF, data[BQ27520_OP_CFG_B_OFF])) {
			printf("Error erasing configuration b from config area\n");
			return -1;
		}
        udelay(2 * 1000);
		if (bq27520_write_data_block(data)) {
			printf("Error writing back configuration b data block to data flash\n");
			return -1;
		}
        udelay(500 * 1000);
		printf("battery configuration b set to 0x%x\n", data[BQ27520_OP_CFG_B_OFF]);
	}
    else {
        printf("battery configuration b was already set to 0x%x\n", data[BQ27520_OP_CFG_B_OFF]);
    }

	return 0;
}


int bq27520_impedance_track(int enable)
{
	int ret;
	if (enable) {
		ret = bq27520_i2c_control_read(BQ27520_CONTROL_IT_ENABLE,NULL);
	}
	else {
		ret = bq27520_i2c_control_read(BQ27520_CONTROL_IT_DISABLE,NULL);
	}
	return ret;
}

int bq27520_impedance_track_enable(void)
{
	u16 status=0;
	if (bq27520_get_control_register(BQ27520_CONTROL_STATUS, &status)==0) 
		{
		udelay(2 * 1000);
			if (!(status&BQ27520_CONTROL_STATUS_QEN)) {
				bq27520_impedance_track(1);
				udelay(300 * 1000);
				printf("Bq27520 impedance track enabled\n");
			}
			return 0;
		}
    return 1;
}

int bq27520_reset(void)
{
	int ret;
	ret = bq27520_i2c_control_read(BQ27520_CONTROL_RESET,NULL);
	return ret;
}

#endif
