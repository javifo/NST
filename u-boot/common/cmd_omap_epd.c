#include <common.h>
#include <command.h>
#include "../cpu/omap3/dsp/dsp.h"

#if defined(CONFIG_CMD_OMAP_EPD)


int do_omap_epd (cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    int ret = 1;

    if (argc == 2) {

        if (0 == strncmp(argv[1], "dspon", 5)) {
#if defined(CONFIG_DSP_SPLASH_SCREEN) && !defined(CFG_TFT_DISPLAY)
            if (dsp_start(NULL) != 0) {
                printf("\nDSP & Splash Screen doesn't start!\n");
            }
#endif
            dsp_display_pic_file("booting.pgm", 10000);
        }
        else if (0 == strncmp(argv[1], "dspoff", 6)) {
#if defined(CONFIG_DSP_SPLASH_SCREEN) && !defined(CFG_TFT_DISPLAY)
            dsp_stop(0);
#endif
        }
        else if (0 == strncmp(argv[1], "image1", 6)) {
#if defined(CONFIG_DSP_SPLASH_SCREEN) && !defined(CFG_TFT_DISPLAY)
            dsp_display_pic_file("charging0.pgm", 10000);    
#endif
        }
        else if (0 == strncmp(argv[1], "image2", 6)) {
#if defined(CONFIG_DSP_SPLASH_SCREEN) && !defined(CFG_TFT_DISPLAY)
            dsp_display_pic_file("dead.pgm", 10000);    
#endif
        }
        else {
            printf("Error reading battery\n");
        }
    }

    else {
        printf("Unsupported option:\n%s\n", argv[1]);
        printf("Usage:\n%s\n", cmdtp->usage);
    }

    return ret;
}

U_BOOT_CMD(
	epd,	2,	1,	do_omap_epd,
	"epd tests dspon dspoff image1 image2\n",
);



#endif	/* CONFIG_CMD_CLOCK */
