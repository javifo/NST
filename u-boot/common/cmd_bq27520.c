#include <common.h>
#include <command.h>

#if defined(CONFIG_CMD_BQ27520)
#include <bq27520.h>


extern void gossamer_usb_test(void);

static void battery_log(void)
{
    u16 volt,charge,na_cap,re_cap,imp;
    s16 current;
    int oc_voltage,sf_voltage;
    bq27520_get_voltage(&volt);
    bq27520_get_state_of_charge(&charge);


    bq27520_get_current(&current);
    bq27520_get_nominal_available_capacity(&na_cap);
    bq27520_get_remaining_capacity(&re_cap);
    bq27520_get_normalized_impedance(&imp);
    oc_voltage = volt - ((int) current * imp) / 1000000;
    sf_voltage = oc_voltage - 3 * imp;
    printf("BBLOG: %d %d %d %d %d %d %d %d %d\n", volt,current,charge,na_cap,re_cap,imp,oc_voltage, oc_voltage- imp,  oc_voltage- imp * 2 );

}

int do_battery (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	int ret = 1;
    int bitno;
    int blkclass,blockno;
    static u8 block_data[BQ27520_BLOCKDATA_LEN];
	static int block_load=0;
    int i,j;
 
    if (argc == 4) {
		if (0 == strcmp(argv[1], "block_modify")) {
			i=simple_strtoul(argv[2], NULL, 10);
			j=simple_strtoul(argv[3], NULL, 10);
			printf("[0x%0x] = 0x%x (was 0x%x)\n",i,j,block_data[i]);
			block_data[i]=j;
		}
		else
        if (0 == strcmp(argv[1], "block_read")) {
            blkclass=simple_strtoul(argv[2], NULL, 10);
            blockno=simple_strtoul(argv[3], NULL, 10);
            bq27520_get_data_block(blkclass,blockno, block_data);
            for (i=0;i<BQ27520_BLOCKDATA_LEN;i++) {
                if ((i&0x7)==0) {
                    printf("\n%04x:",i);
                }
                printf(" 0x%02x",block_data[i]);
            }
            printf("\n");
			block_load = 1;
        }
        else {
            printf("only command is block_read <class> <block #>\n");
        }
    }
	else if (argc == 2) {
		if (0 == strcmp(argv[1], "block_save")) {
			if (block_load) {
				bq27520_write_data_block(block_data);
				for (i=0;i<BQ27520_BLOCKDATA_LEN;i++) {
					if ((i&0x7)==0) {
						printf("\n%04x:",i);
					}
					printf(" 0x%02x",block_data[i]);
				}
				printf("\n");
			}
		}
		else
		if (0 == strcmp(argv[1], "block_show")) {
			for (i=0;i<BQ27520_BLOCKDATA_LEN;i++) {
				if ((i&0x7)==0) {
					printf("\n%04x:",i);
				}
				printf(" 0x%02x",block_data[i]);
			}
			printf("\n");
		}
		else
		if (0 == strcmp(argv[1],"ite")) {
			bq27520_impedance_track(1);
		} else
		if (0 == strcmp(argv[1],"itd")) {
			bq27520_impedance_track(0);
		} else
		if (0 == strcmp(argv[1],"reset")) {
			bq27520_reset();
		} else
        if (0 == strncmp(argv[1],"lpsuen",9)) {
            printf("enable lspuen\n");
            if (bq27520_enable_batlspuen()!=0) {
               printf("Something went wrong when trying to disable BATLSPUEN\n");
           }
        } else

        if (0 == strncmp(argv[1],"lpsudis",9)) {
            printf("disabling batlspuen\n");
            if (bq27520_disable_batlspuen()!=0) {
               printf("Something went wrong when trying to disable BATLSPUEN\n");
           }
        } else

        if (0 == strncmp(argv[1], "log", 3)) {
            battery_log();
        } else
        if (0 == strncmp(argv[1], "info", 4)) {
			u16 soc16,volt16,hwtype=0,hwversion=0,fwversion=0,status=0;
			u16 dfsversion=0;
			int volt=-1,soc=-1,bat;
			bat=bq27520_battery_present();
			printf("bat returned\n");

			if (bat>=0) {
#if defined(CONFIG_DRIVER_OMAP24XX_I2C) || defined(CONFIG_DRIVER_OMAP34XX_I2C)
                if (bq27520_get_hw_type(&hwtype)==0) {
                    printf("HW type = 0x%x\n",hwtype);
                }
                else {
                    printf("HW type unknown\n");
                }

                if (bq27520_get_hw_version(&hwversion)==0) {
                    printf("HW version = 0x%x\n",hwversion);
                }
                else {
                    printf("HW version unknown\n");
                }
                if (bq27520_get_fw_version(&fwversion)==0) {
                    printf("FW version = 0x%x\n",fwversion);
                }
                else {
                    printf("FW version unknown\n");
                }
                if (bq27520_get_control_register(BQ27520_CONTROL_STATUS, &status)==0) {
                    const char *bitno_r_str[16]={"DLOGEN","Full Access Sealed","Sealed","SCSV","SCCA","SBCA","SOCVCMDCOMP",
                        "SOCVFAIL","SINITCOMP","SHIBERNATE","SSNOOZE","SSLEEP","SLDMD","SRUP_DIS","SVOK","SQEN"};
                    printf("CONTROL_STATUS = 0x%04x\n",status);
                    for (bitno=0;bitno<16;bitno++) {
                        if (status&(1<<bitno)) {
                            printf("%s ",bitno_r_str[15-bitno]);
                        }
                    }
                    printf("\n");
                }
				if (bq27520_get_control_register(BQ27520_CONTROL_DFS_VERSION, &dfsversion)==0) {
					printf("DFS version = 0x%x\n",dfsversion);
				}
#endif
                printf("get voltage\n");
				if (bq27520_get_voltage(&volt16) == 0) volt=volt16;
				if (bq27520_get_state_of_charge(&soc16) == 0 ) soc=soc16;
				printf("Battery voltage : %d\nsoc : %d  present:%d\n",volt,soc,bat);

 		}
			else
			{
				printf("Error reading battery\n");
			}
		}
	}
	else {
		printf("Unsupported option:\n%s\n", argv[1]);
		printf("Usage:\n%s\n", cmdtp->usage);
	}

	return ret;
}

U_BOOT_CMD(
	battery,	4,	1,	do_battery,
	"battery   - gas gauge BQ27520 info\n",
	"  options : \n"
	"            info - display gas gauge information\n"
	"            block_read <block_num> <subblock> - display dataflash block\n"
    "            lpsudis - disable batlspuen\n"
    "            lpsuen - enable batlspuen\n"
	"			 ite -enable impedance track\n"
	"			 itd -disable impedance track\n"
	"			 reset\n"
	"            block_modify <offset> <value> - untested\n"
	"            block_save - untested\n"

);



#endif	/* CONFIG_CMD_CLOCK */
