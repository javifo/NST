#ifndef _BQ24073_H_
#define _BQ24073_H_

int bq24073_init(int _ce_gpio, int _en1_gpio, int _en2_gpio);
int bq24073_enable(int mamps);
int bq24073_enable_charger(void);
int bq24073_disable_charger(void);
void bq24073_allow_current_change(void);
int bq24073_get_current(void);




#endif /* _BQ24073_H_ */
