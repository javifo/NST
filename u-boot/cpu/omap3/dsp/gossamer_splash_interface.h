
/*******************************************************************************
 *
 * File name: epd_splash_interface.h
 *
 ******************************************************************************/

#ifndef EPD_SPLASH_INTERFACE_H
#define EPD_SPLASH_INTERFACE_H

// Next are got from DSP linker file (cmd)
#define MAX_WAVEFORM_FILE_SIZE	(0x0f0000)	// Got from linker file (cmd)
#define MAX_IMAGE_FILE_SIZE		(0x100000)	// Got from linker file (cmd)
#define DSP_BOOT_ADDR	0x5C7F8000
#define DSP_BOOT_SIZE	0x10000	// MAX 64k
#define DSP_IF_ADDR		(0x87000000)
#define DSP_IF_SIZE		(256)
#define DSP_PIC_ADDR	(0x87010000)
#define DSP_WVF_ADDR	(DSP_PIC_ADDR + MAX_IMAGE_FILE_SIZE)

// DSP status values and Operation result codes
typedef enum {
	// DSP execution state codes
	DSP_ST_RESET			= 0x645f0000,
	DSP_ST_RUN,
	DSP_ST_MEMINIT,
	DSP_ST_HWINIT,
	DSP_ST_CLEARSCREEN,
	DSP_ST_PICTURE,
	DSP_ST_PICTURE_DONE,
	DSP_ST_PLAY_IDLES,
	// DSP completion codes - ARM is allowed to change, i.e. send CMD.
	DSP_ST_ALL_DONE 		= 0x645f0700,	// DSP done current op., but can accept CMD.
	DSP_ST_STOPPED,			// DSP is no more respodable unless started again.
	// Command codes (basically ARM -> DSP, but DSP can
	// also put some code to self request an operation.
	DSP_STCMD_base			= 0x645f0800,
	DSP_STCMD_CLS_INIT,
	DSP_STCMD_CLS_DU,
	DSP_STCMD_CLS_GC,
	DSP_STCMD_DRAW_PIC,
	DSP_STCMD_IDLE_N,
	DSP_STCMD_STOP,
	DSP_STCMD_end,
	//
	DSP_ST_UNUSED			// Use just as last.
} DSP_STATUS_t;

typedef enum {
	//
	DSP_ERROR_OK				= 0x37310000,// not fatal
	DSP_ERROR_DSI_PLL_LOCK,
	//
	DSP_ERROR_FATAL_1			= 0x37310100,// fatal for current operation, could retry
	DSP_ERROR_IMGFMT,
	DSP_ERROR_WVFID,
	DSP_ERROR_BADARG,
	//
	DSP_ERROR_FATAL_2			= 0x37310200,// fatal at all - DSP can only stop.
	DSP_ERROR_CFG_SCREEN,
	DSP_ERROR_CFG_WVF,
	DSP_ERROR_MEMALLOC,
	DSP_ERROR_DSS_RESET,
	DSP_ERROR_DISPC_RESET,
	DSP_ERROR_DSI_RESET,
	DSP_ERROR_DSS_MALLOC,
	DSP_ERROR_DSI_PWR,
	DSP_ERROR_DSI_PLL_BYPASS,
	DSP_ERROR_DSI_PLL_CLOCK,
	DSP_ERROR_SUBFPROC,
	DSP_ERROR_SUBFQ,
	//
	DSP_ERROR_UNUSED			// Use just as last.
} DSP_ERROR_t;

typedef enum {
	WVF_FILE_TYPE_EINK = 1,
	WVF_FILE_TYPE_MMS = 2
} WVF_FILE_TYPE_t;

#if defined(_TMS320C6400_PLUS)
#define DSP_CONST	const	// Fields constant on the DSP side.
#else
#define DSP_CONST
#endif

typedef struct {
	DSP_STATUS_t	dsp_st;		// Exchange status between ARM & DSP
	DSP_ERROR_t		dsp_err;	// DSP operation result code.
	DSP_CONST int	temperature;
	DSP_CONST int	SYS_CLK_KHZ;
	int	vio_1;
	int	vio_2;
	int	vio_3;
	int	vio_4;
	DSP_CONST struct {
		DSP_CONST uint8 *file;
		DSP_CONST uint32 file_sz;
		DSP_CONST uint32 type;	// Don't care.
	} img;
	DSP_CONST struct {
		DSP_CONST uint8 *file;
		DSP_CONST uint32 file_sz;
		DSP_CONST WVF_FILE_TYPE_t type;
	} wvf;
} epd_splash_cfg_cmn_t;

typedef struct {
	uint8 a[DSP_IF_SIZE-sizeof(epd_splash_cfg_cmn_t)];
} screen_cfg_arm_t;

struct epd_splash_cfg_arm {
	epd_splash_cfg_cmn_t	cmn;
	screen_cfg_arm_t		screen;
};

#endif//EPD_SPLASH_INTERFACE_H
