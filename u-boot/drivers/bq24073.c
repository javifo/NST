#include <bq24073.h>
#include <asm/types.h>
#include "../common/power/gpio.h"

struct gpio_en_t
{
	int gpio;
	int is_init;
};

static struct gpio_en_t ce_gpio = {};
static struct gpio_en_t en1_gpio = {};
static struct gpio_en_t en2_gpio = {};
static int    last_mamps = -1;
static int    allow_current_change = 0;


static int bq24073_gpio_write(struct gpio_en_t gpio_pin, gpio_level_t gpio_level)
{
	int ret;
	if(gpio_pin.is_init == 0){
		ret = gpio_pin_init(gpio_pin.gpio, GPIO_OUTPUT, gpio_level);
		gpio_pin.is_init = 1;
	}else{
		ret = gpio_pin_write(gpio_pin.gpio, gpio_level);
	}
	return ret;
};

int bq24073_init(int _ce_gpio,
		int _en1_gpio,
		int _en2_gpio)
{
	ce_gpio.gpio = _ce_gpio;
	en1_gpio.gpio = _en1_gpio;
	en2_gpio.gpio = _en2_gpio;

	ce_gpio.is_init = 0;
	en1_gpio.is_init = 0;
	en2_gpio.is_init = 0;
    allow_current_change = 0;
    last_mamps = -1;
	printf("charger pins %d %d %d\n",ce_gpio.gpio,en1_gpio.gpio,en2_gpio.gpio);

	return 0;
}

int bq24073_enable_charger(void)
{
	if ((ce_gpio.gpio == 0) ||\
	   (en1_gpio.gpio == 0) ||\
	   (en2_gpio.gpio == 0))
		return -1;

	return bq24073_gpio_write(ce_gpio, GPIO_LOW);
}

int bq24073_disable_charger(void)
{
	if ((ce_gpio.gpio == 0) ||\
	   (en1_gpio.gpio == 0) ||\
	   (en2_gpio.gpio == 0))
		return -1;

	return bq24073_gpio_write(ce_gpio, GPIO_HIGH);
}

void  bq24073_allow_current_change(void)
{
    allow_current_change = 1;
    if (last_mamps >=0 ) {
        bq24073_enable(last_mamps);
    }
}


int bq24073_enable(int mamps)
{
	int stat = -1;
    int cc_same_bank;

    cc_same_bank = ((en1_gpio.gpio>>5)==(en2_gpio.gpio>>5))?1:0;
    last_mamps = mamps;

	if (ce_gpio.gpio == 0) {
            printf("bq24073: not initted\n");
            return stat;
    }
    if (allow_current_change == 0 ) {
        printf("bq24073: not safe to change current\n");
        return stat;
    }

	printf("charge current request %d\n",mamps);
    if (cc_same_bank) {
        switch (mamps) {
        case 0:
            stat = gpio_two_pin_init_write(en1_gpio.gpio, GPIO_HIGH, en2_gpio.gpio, GPIO_HIGH);
            stat |= bq24073_gpio_write(ce_gpio, GPIO_LOW);
            break;
        case 100:
            stat = gpio_two_pin_init_write(en1_gpio.gpio, GPIO_LOW, en2_gpio.gpio, GPIO_LOW);
            stat |= bq24073_gpio_write(ce_gpio, GPIO_LOW);
            break;
        case 500:
            stat = gpio_two_pin_init_write(en1_gpio.gpio, GPIO_HIGH, en2_gpio.gpio, GPIO_LOW);
            stat |= bq24073_gpio_write(ce_gpio, GPIO_LOW);
            break;
        case 1000:
            stat = gpio_two_pin_init_write(en1_gpio.gpio, GPIO_LOW, en2_gpio.gpio, GPIO_HIGH);
            stat |= bq24073_gpio_write(ce_gpio, GPIO_LOW);
            break;
        default:
            printf("bq24073: invalid current value: %d\n", mamps);
        }
        return stat;
    }

    // when switching current control pins individually, the low one should be done first, so that any
    // transitional state will still be charging instead of disabled
	switch (mamps) {
    case 0:
        stat = bq24073_gpio_write(en1_gpio, GPIO_HIGH);
        stat |= bq24073_gpio_write(en2_gpio, GPIO_HIGH);
        stat |= bq24073_gpio_write(ce_gpio, GPIO_LOW);
        break;
	case 100:
		stat = bq24073_gpio_write(en1_gpio, GPIO_LOW);
		stat |= bq24073_gpio_write(en2_gpio, GPIO_LOW);
		stat |= bq24073_gpio_write(ce_gpio, GPIO_LOW);
		break;
	case 500:
        stat = bq24073_gpio_write(en2_gpio, GPIO_LOW);
		stat |= bq24073_gpio_write(en1_gpio, GPIO_HIGH);
		stat |= bq24073_gpio_write(ce_gpio, GPIO_LOW);
		break;
	case 1000:
		stat = bq24073_gpio_write(en1_gpio, GPIO_LOW);
		stat |= bq24073_gpio_write(en2_gpio, GPIO_HIGH);
		stat |= bq24073_gpio_write(ce_gpio, GPIO_LOW);
		break;
	default:
		printf("bq24073: invalid current value: %d\n", mamps);
	}

	return stat;
}



int bq24073_get_current(void)
{
    // note current is current from charger, not charging current.
    // also  note, even though reporting 0, may not be zero.
    if (ce_gpio.gpio) {
        return (allow_current_change?last_mamps:-1);
    }
    return 0;
}

