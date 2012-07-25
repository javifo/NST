#include <common.h>
#include <command.h>

#if defined(CONFIG_CMD_BQ24073)
#include <bq24073.h>

int do_charger (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    int ret = 1;

    if (argc == 2) {
        if (0 == strncmp(argv[1], "enable", 6)) {
            bq24073_enable_charger();
        }
        else
            if (0 == strncmp(argv[1], "disable", 7)) {
            bq24073_disable_charger();
        }
        else
            if (0 == strncmp(argv[1], "allow", 5)) {
                bq24073_allow_current_change();
        }
        else
            if (0 == strncmp(argv[1], "get", 3)) {
                printf("charge current set to %d\nNote -1 indicates charger enabled, but current not set by software\n",bq24073_get_current());
        }
        else
            if (0 == strncmp(argv[1], "1000", 7)) {
                bq24073_enable(1000);
        }
        else
            if (0 == strncmp(argv[1], "500", 7)) {
                bq24073_enable(500);
        }
        else
            if (0 == strncmp(argv[1], "100", 7)) {
                bq24073_enable(100);
        }
        else
            if (0 == strncmp(argv[1], "0", 7)) {
                bq24073_enable(0);
        }
        else {
            printf("Error unkown command\n");
        }
    }
    else {
        printf("Usage:\n%s\n", cmdtp->usage);
    }

    return ret;
}

U_BOOT_CMD(
	charger,	2,	1,	do_charger,
	"charger   - charger BQ24073 info\n",
	"  options : \n"
	"            enable - enable charger\n"
    "            disable - disable charger\n"
    "            allow - allow current change\n"
    "            get   - display get charge value\n"
    "            0,100,500,1000 - set charger current\n"
);



#endif	/* CONFIG_CMD_CLOCK */
