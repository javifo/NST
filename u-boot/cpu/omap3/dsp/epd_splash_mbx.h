
#ifndef	_EPD_SPLASH_MBX_H_
#define	_EPD_SPLASH_MBX_H_

typedef enum {
	MBX_MB_0	= 0,
	MBX_MB_1	= 1
} MBX_MB_en_t;

typedef enum {
	MBX_USER_MPU	= 0,
	MBX_USER_IVA	= 1
} MBX_USER_en_t;

//---
void epd_splash_mbx_init(void);
void epd_splash_mbx_restore(void);

int epd_splash_mbx_write(const MBX_MB_en_t mbx, const u32 msg);

#endif
