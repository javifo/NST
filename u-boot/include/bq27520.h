#ifndef _BQ27520_H_
#define _BQ27520_H_

#include <common.h>

#define BQ27520_I2C_ADDR	(0x55)
#define BQ27520_I2C_BUS_NUM	(0x01)

#define BQ27520_REG_CONTROL     (0x00)
#define BQ27520_CONTROL_STATUS   (0x0000)

#define BQ27520_CONTROL_DEVICE_TYPE   (0x0001)
#define BQ27520_CONTROL_FW_VERSION   (0x0002)
#define BQ27520_CONTROL_HW_VERSION   (0x0003)
#define BQ27520_CONTROL_DFS_VERSION  (0x001f)
#define BQ27520_CONTROL_IT_ENABLE	 (0x0021)
#define BQ27520_CONTROL_IT_DISABLE	 (0x0023)
#define BQ27520_CONTROL_RESET		 (0x0041)

#define BQ27520_CONTROL_STATUS_QEN   (0x0001)


#define BQ27520_REG_STATE_OF_CHARGE	(0x2C)
#define BQ27520_REG_VOLTAGE		(0x08)
#define BQ27520_REG_FLAGS	(0x0a)
#define BQ27520_AVG_CURRENT  (0x14)
#define BQ27510_REG_NOMINAL_AVAILABLE_CAPACITY (0x0C)
#define BQ27510_REMAINING_CAPACITY (0x10)
#define BQ27510_REG_NORMALIZED_IMPEDANCE_CAL  (0x2e)
#define BQ27520_FLAG_BAT_DET   0x8

#define BQ27520_REG_BLOCKDATACHECKSUM (0x60)
#define BQ27520_REG_BLOCKDATACONTROL (0x61)
#define BQ27520_BLOCKDATACONTROL_ENABLE     0
#define BQ27520_BLOCKDATACONTROL_DISABLE    1

#define BQ27520_DATAFLASHBLOCK (0x3f)
#define BQ27520_DATAFLASHCLASS (0x3e)

#define BQ27520_BLOCKDATA_BASE (0x40)
#define BQ27520_BLOCKDATA_LEN  (0x20)

#define BQ27520_OP_CFG_CLASS   (0x40)
#define BQ27520_OP_CFG_BLOCK   (0x00)
#define BQ27520_OP_CFG_C_OFF   (0x0C)
#define BQ27520_BATLSPUEN_MASK (0x20)

#define BQ27520_OP_CFG_B_OFF    (0x09)


int bq27520_battery_present(void);
int bq27520_get_state_of_charge(u16 * soc);
int bq27520_get_voltage(u16 * voltage);
int bq27520_get_hw_type(u16 *hwtype);
int bq27520_get_hw_version(u16 *hwversion);
int bq27520_get_fw_version(u16 *fwversion);
int bq27520_get_current(s16 * current);
int bq27520_get_nominal_available_capacity(u16 *nc);
int bq27520_get_remaining_capacity(u16 *nc);
int bq27520_get_normalized_impedance(u16 *nc);

int bq27520_get_control_register(u16 control, u16 *data);
int bq27520_get_data_block(int dbclass,int block, u8 *data);
int bq27520_write_data_block(u8 *data);
int bq27520_disable_batlspuen(void);
int bq27520_enable_batlspuen(void);
int bq27520_impedance_track(int enable);
int bq27520_reset(void);
int bq27520_impedance_track_enable(void);


#endif /* _BQ27520_H_ */
