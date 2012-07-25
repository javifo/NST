
#ifndef PAPIRUS_H
#define PAPIRUS_H

#include <common.h>
#include <i2c.h>

/*
 * Papirus
 */

/* For papirus init */
int papirus_init(const char *init_img_fname);
int dis_power_off(void);
long splash_load_file2buffer (const char *fname, void * const buf, const int max_sz);

#endif /* PAPIRUS_H */
