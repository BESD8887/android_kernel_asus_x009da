/*
 * ktd2037.c - driver for KTD2037 led driver chip
 *
 * Copyright (C) 2016, Arima Communications Co. Ltd. All Rights Reserved.
 *
 * Contact: Poting Chang <potingchang@arimacomm.com.tw>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 */
#define  DEBUG
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/leds-ktd2037.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>

/* KTD2037 register map */
#define KTD2037_REG_EN_RST		0x00
#define KTD2037_REG_FLASH_PERIOD	0x01
#define KTD2037_REG_PWM1_TIMER		0x02
#define KTD2037_REG_PWM2_TIMER		0x03
#define KTD2037_REG_LED_EN		0x04
#define KTD2037_REG_TRISE_TFALL		0x05
#define KTD2037_REG_LED1		0x06
#define KTD2037_REG_LED2		0x07
#define KTD2037_REG_LED3		0x08
#define KTD2037_REG_AUTONLINK		0x09
#define KTD2037_REG_MAX			0x0a
#define KTD2037_TIME_UNIT		500
/* MASK */
#define CNT_TIMER_SLOT_MASK		0x07
#define CNT_ENABLE_MASK			0x18
#define CNT_RISEFALL_TSCALE_MASK	0x60

#define CNT_TIMER_SLOT_SHIFT		0x00
#define CNT_ENABLE_SHIFT		0x03
#define CNT_RISEFALL_TSCALE_SHIFT	0x05

#define LED_R_MASK		0x00ff0000
#define LED_G_MASK		0x0000ff00
#define LED_B_MASK		0x000000ff
#define LED_R_SHIFT		16
#define LED_G_SHIFT		8

//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 S
#define LED_R_OFF_REG4_MASK		0x03
#define LED_G_OFF_REG4_MASK		0x0C
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 E

#define KTD2037_RESET		0x07

#define LED_MAX_CURRENT		0x20
#define LED_DEFAULT_CURRENT	0x10
#define LED_LOW_CURRENT		0x02
#define LED_OFF			0x00

#define MAX_NUM_LEDS		3

#define KTD_VTG_MAX_UV		3300000
#define KTD_I2C_VTG_MIN_UV 	1800000
#define KTD_I2C_VTG_MAX_UV       1800000
#define KTD_I2C_LOAD_UA	10000

enum ktd2037_led_mode {
	LED_EN_OFF	= 0,
	LED_EN_ON	= 1,
	LED_EN_PWM1	= 2,
	LED_EN_PWM2	= 3,
};
enum ktd2037_pwm{
	PWM1 = 0,
	PWM2 = 1,
};
enum ktd2037_led_enum {
	LED_R = 0,
	LED_G = 2,
	LED_B = 4,
};

struct ktd2037_led {
	u8	channel;
	u8	brightness;
	unsigned long delay_on_time_ms;
	unsigned long delay_off_time_ms;
};

struct ktd2037_data {
	struct	i2c_client	*client;
	struct	mutex	mutex;
	struct	ktd2037_led	leds[MAX_NUM_LEDS];
	u8		shadow_reg[KTD2037_REG_MAX];
};

struct i2c_client *b_client;

#define SEC_LED_SPECIFIC

#ifdef SEC_LED_SPECIFIC
struct class *sec_class;
static struct device *led_dev;
/*path : /sys/class/secleds/led/led_r*/
/*path : /sys/class/secleds/led/led_g*/
/*path : /sys/class/secleds/led/led_blink*/
#endif

static void ktd2037_leds_on(enum ktd2037_led_enum led,
		enum ktd2037_led_mode mode, u8 bright);

//[5830][RGB Led]  I2C write one byte instead of write all to reduce fail rate in potentially PT Chang 20160315 S
static int ktd_write_byte(struct ktd2037_data *led, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(led->client, reg, val);
}
//[5830][RGB Led]  I2C write one byte instead of write all to reduce fail rate in potentially PT Chang 20160315 E

static int leds_i2c_write_all(struct i2c_client *client)
{
	struct ktd2037_data *data = i2c_get_clientdata(client);
	int ret,i;

	mutex_lock(&data->mutex);

	for(i=0;i<KTD2037_REG_MAX;i++)
	{
		pr_devel(" before write  [%d]. data->shadow_reg=%x \n", i,data->shadow_reg[i]   );
	}

	ret = i2c_smbus_write_i2c_block_data(client,
			KTD2037_REG_EN_RST, KTD2037_REG_MAX,
			&data->shadow_reg[KTD2037_REG_EN_RST]);

	if (ret < 0) {
		dev_err(&client->adapter->dev,
			"%s: failure on i2c block write\n",
			__func__);
		goto exit;
	}

	mutex_unlock(&data->mutex);

	return 0;

exit:
	mutex_unlock(&data->mutex);

	return ret;
}

static void ktd2037_leds_on(enum ktd2037_led_enum led,
		enum ktd2037_led_mode mode, u8 bright)
{
	struct ktd2037_data *data = i2c_get_clientdata(b_client);

	data->shadow_reg[KTD2037_REG_LED1 + led/2] = bright;

	if(mode == LED_EN_OFF)
	{
		data->shadow_reg[KTD2037_REG_LED_EN] &= ~(LED_EN_PWM2 << led);
//[5830][RGB Led]  Don't enable REG4[6] when no intend to Led light PT Chang 20160315 S
		data->shadow_reg[KTD2037_REG_LED_EN] &= ~(0xc0);
//[5830][RGB Led]  Don't enable REG4[6] when no intend to Led light PT Chang 20160315 E
	}
	else
	{
		data->shadow_reg[KTD2037_REG_LED_EN] |= mode << led;
		data->shadow_reg[KTD2037_REG_LED_EN] |= 0X40;
		pr_devel("  [%s]  data->shadow_reg[KTD2037_REG_LED_EN] = 0x%x \n",__func__,data->shadow_reg[KTD2037_REG_LED_EN]);
	}

}
void ktd2037_set_timerslot_control(int timer_slot)
{
	struct ktd2037_data *data = i2c_get_clientdata(b_client);

	data->shadow_reg[KTD2037_REG_EN_RST] &= ~(CNT_TIMER_SLOT_MASK);
	data->shadow_reg[KTD2037_REG_EN_RST]
		|= timer_slot << CNT_TIMER_SLOT_SHIFT;
}

/*  Flash period = period * 0.128 + 0.256
	exception  0 = 0.128s
	please refer to data sheet for detail */
void ktd2037_set_period(int period)
{
	struct ktd2037_data *data = i2c_get_clientdata(b_client);

	data->shadow_reg[KTD2037_REG_FLASH_PERIOD] = period;
}

/* MAX duty = 0xFF (99.6%) , min duty = 0x0 (0%) , 0.4% scale */
void ktd2037_set_pwm_duty(enum ktd2037_pwm pwm, int duty)
{
	struct ktd2037_data *data = i2c_get_clientdata(b_client);
	data->shadow_reg[KTD2037_REG_PWM1_TIMER + pwm] = duty;
}

/* Rise Ramp Time = trise * 96 (ms) */
/* minimum rise ramp time = 1.5ms when traise is set to 0 */
/* Tscale */
/* 0 = 1x      1 = 2x slower      2 = 4x slower    3 = 8x slower */

void ktd2037_set_trise_tfall(int trise, int tfall, int tscale)
{
	struct ktd2037_data *data = i2c_get_clientdata(b_client);

	data->shadow_reg[KTD2037_REG_TRISE_TFALL] = (tfall << 4) + trise;

	data->shadow_reg[KTD2037_REG_EN_RST] &= ~(CNT_RISEFALL_TSCALE_MASK);
	data->shadow_reg[KTD2037_REG_EN_RST]
			|= tscale << CNT_RISEFALL_TSCALE_SHIFT;
}

#ifdef SEC_LED_SPECIFIC
static void ktd2037_reset_register_work(struct work_struct *work)
{
	struct i2c_client *client;
	struct ktd2037_data *data;
	client = b_client;
	data = i2c_get_clientdata(client);

	pr_devel("  [%s]  \n",__func__);
	ktd2037_leds_on(LED_R, LED_EN_OFF, 0);
	ktd2037_leds_on(LED_G, LED_EN_OFF, 0);
	ktd2037_leds_on(LED_B, LED_EN_OFF, 0);

//[5830][RGB Led]  I2C write one byte instead of write all to reduce fail rate in potentially PT Chang 20160315 S
	ktd_write_byte(data, KTD2037_REG_LED_EN, data->shadow_reg[KTD2037_REG_LED_EN]);
	ktd_write_byte(data, KTD2037_REG_LED1, data->shadow_reg[KTD2037_REG_LED1]);
	ktd_write_byte(data, KTD2037_REG_LED2, data->shadow_reg[KTD2037_REG_LED2]);

	ktd2037_set_timerslot_control(0); /* Tslot1 */
	ktd2037_set_period(0);
	ktd2037_set_pwm_duty(PWM1, 0);
	ktd2037_set_trise_tfall(0, 0, 0);

	ktd_write_byte(data, KTD2037_REG_EN_RST, data->shadow_reg[KTD2037_REG_EN_RST]);
	ktd_write_byte(data, KTD2037_REG_FLASH_PERIOD, data->shadow_reg[KTD2037_REG_FLASH_PERIOD]);
	ktd_write_byte(data, KTD2037_REG_PWM1_TIMER, data->shadow_reg[KTD2037_REG_PWM1_TIMER]);
	ktd_write_byte(data, KTD2037_REG_TRISE_TFALL, data->shadow_reg[KTD2037_REG_TRISE_TFALL]);
	ktd_write_byte(data, KTD2037_REG_EN_RST, data->shadow_reg[KTD2037_REG_EN_RST]);
//[5830][RGB Led]  I2C write one byte instead of write all to reduce fail rate in potentially PT Chang 20160315 E

}

void ktd2037_led_off(enum ktd2037_pattern mode)
{

	struct i2c_client *client;
	struct work_struct *reset = 0;
	client = b_client;

	/* Set all LEDs Off */
	ktd2037_reset_register_work(reset);

	if (mode == LED_OFF)
		return;
}
EXPORT_SYMBOL(ktd2037_led_off);

static void ktd2037_leds_on_blink(enum ktd2037_led_enum led,
		enum ktd2037_led_mode mode, u8 bright)
{
	struct ktd2037_data *data = i2c_get_clientdata(b_client);
	data->shadow_reg[KTD2037_REG_LED1 + led/2] = bright;
	data->shadow_reg[KTD2037_REG_LED_EN] &= ~(LED_EN_ON<<led);
	data->shadow_reg[KTD2037_REG_LED_EN] |= mode << led;
	data->shadow_reg[KTD2037_REG_LED_EN] |= 0x40;
}

static void ktd2037_set_led_blink(enum ktd2037_led_enum led,
					 int Tflash,
					 int duty1,
					u8 brightness)
{
	ktd2037_leds_on_blink(led, LED_EN_PWM1, brightness);
	ktd2037_set_period(Tflash);
	ktd2037_set_pwm_duty(PWM1, duty1);
}

//[5830][RGB Led] Map britness level for 5mA Chang 20160408 S
 unsigned char convert_rgb(unsigned char bri)
{
	unsigned char newbri;
	newbri = bri*39/255;

	if(bri != 0)
		newbri++;
	if(newbri == 40)
		newbri--;

	pr_devel("[%s]  %d \n", __func__, newbri);

	return newbri;
}
//[5830][RGB Led] Map britness level for 5mA Chang 20160408 E

//[5830][RGB Led] Map britness level PT Chang 20160328 S
static void ktd2037_leds_map(u8 *brightness)
{
	unsigned char reg = 0;
//[5830][RGB Led] Map britness level for PCBAPT Chang 20160331 S
//	*brightness /= 6;
//[5830][RGB Led] Map britness level for PCBA PT Chang 20160331 E
//[5830][RGB Led] Map britness level for 5mA Chang 20160408 S
	reg = convert_rgb(*brightness);
	pr_devel("[%s] Input *brightness = %d,  reg = %d \n", __func__, *brightness, reg);
	*brightness = reg;
	pr_devel("[%s] After Input *brightness = %d\n\n", __func__,*brightness);
//[5830][RGB Led] Map britness level for 5mA Chang 20160408 E
}
//[5830][RGB Led] Map britness level PT Chang 20160328 E

	/* ex) echo 0x201000 30 3 > led_blink */ //4s 40ms
	/* brightness r=20 g=10 b=00, aa=0~127(s), bb=0~255(%) */
static ssize_t store_ktd2037_led_blink(struct device *dev,
					struct device_attribute *devattr,
					const char *buf, size_t count)
{
	int retval;
	unsigned int led_brightness = 0;
	unsigned int Tflash = 0;
	unsigned int duty1 = 0;
	struct ktd2037_data *data = dev_get_drvdata(dev);
	u8 led_r_brightness = 0;
	u8 led_g_brightness = 0;
	u8 led_b_brightness = 0;
	pr_devel("  [%s] START \n",__func__);
	retval = sscanf(buf, "0x%x %d %d", &led_brightness,
		&Tflash, &duty1);
	if (retval == 0) {
		dev_err(&data->client->dev, "fail to get led_blink value.\n");
		return count;
	}

	/*Set LED blink mode*/
	led_r_brightness = ((u32)led_brightness & LED_R_MASK) >> LED_R_SHIFT;
	led_g_brightness = ((u32)led_brightness & LED_G_MASK) >> LED_G_SHIFT;
	led_b_brightness = ((u32)led_brightness & LED_B_MASK);

//[5830][RGB Led] Map britness level PT Chang 20160328 S
	ktd2037_leds_map(&led_r_brightness);
	ktd2037_leds_map(&led_g_brightness);
//[5830][RGB Led] Map britness level PT Chang 20160328 E

	// Configure Reg before I2C write .
	ktd2037_set_led_blink(LED_R, Tflash , duty1, led_r_brightness);
	ktd2037_set_led_blink(LED_G, Tflash , duty1, led_g_brightness);
	ktd2037_set_led_blink(LED_B, Tflash , duty1, led_b_brightness);

//[5830][RGB Led] Turn off led clearly when no using  PT Chang 20160428 S
	if (led_r_brightness == 0)
		data->shadow_reg[KTD2037_REG_LED_EN] &= ~(LED_EN_PWM2<<LED_R);

	if (led_g_brightness == 0)
		data->shadow_reg[KTD2037_REG_LED_EN] &= ~(LED_EN_PWM2<<LED_G);

	if (led_b_brightness == 0)
		data->shadow_reg[KTD2037_REG_LED_EN] &= ~(LED_EN_PWM2<<LED_B);

	if ( led_brightness == 0x00)
		data->shadow_reg[KTD2037_REG_LED_EN] &= ~0x7f;// turn off red,green,blue. and shutdown IC.
//[5830][RGB Led] Turn off led clearly when no using  PT Chang 20160428 S

//[5830][RGB Led]  I2C write one byte instead of write all to reduce fail rate in potentially PT Chang 20160315 S
	ktd_write_byte(data, KTD2037_REG_LED_EN, data->shadow_reg[KTD2037_REG_LED_EN]);
	ktd_write_byte(data, KTD2037_REG_FLASH_PERIOD, data->shadow_reg[KTD2037_REG_FLASH_PERIOD]);
	ktd_write_byte(data, KTD2037_REG_PWM1_TIMER, data->shadow_reg[KTD2037_REG_PWM1_TIMER]);
	ktd_write_byte(data, KTD2037_REG_LED1, data->shadow_reg[KTD2037_REG_LED1]);
	ktd_write_byte(data, KTD2037_REG_LED2, data->shadow_reg[KTD2037_REG_LED2]);
//[5830][RGB Led]  I2C write one byte instead of write all to reduce fail rate in potentially PT Chang 20160315 E
	pr_devel("  led_blink is called, Color:0x%X \n",led_brightness   );
	pr_devel("  [%s] END \n",__func__);

	return count;
}

static void ktd2037_leds_on_britness(enum ktd2037_led_enum led,
		enum ktd2037_led_mode mode, u8 bright)
{

	struct ktd2037_data *data = i2c_get_clientdata(b_client);

	data->shadow_reg[KTD2037_REG_LED1 + led/2] = bright;
	data->shadow_reg[KTD2037_REG_LED_EN]  &=  ~(LED_EN_PWM1<< LED_R);
	data->shadow_reg[KTD2037_REG_LED_EN]  &=  ~(LED_EN_PWM1<< LED_G);

	if(mode == LED_EN_OFF)
	{
		data->shadow_reg[KTD2037_REG_LED_EN] &= ~(LED_EN_PWM2 << led);
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 S
//[5830][RGB Led]  Don't enable REG4[6] when no intend to Led light PT Chang 20160315 S
//			data->shadow_reg[KTD2037_REG_LED_EN] &= ~(0xc0);
//[5830][RGB Led]  Don't enable REG4[6] when no intend to Led light PT Chang 20160315 E
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 E
	}
	else
	{
		data->shadow_reg[KTD2037_REG_LED_EN] |= mode << led;
		data->shadow_reg[KTD2037_REG_LED_EN] |= 0X40;
	}

}

//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 S
static void ktd2037_leds_faucet_config(struct ktd2037_data *pdata )
{
	pr_devel("  [%s] START \n",__func__);
	pr_devel("  [%s] pdata->shadow_reg[KTD2037_REG_LED_EN]=0x%x \n",__func__,pdata->shadow_reg[KTD2037_REG_LED_EN]);
	if(((pdata->shadow_reg[KTD2037_REG_LED_EN]&LED_R_OFF_REG4_MASK) ==  LED_EN_OFF) && ((pdata->shadow_reg[KTD2037_REG_LED_EN]& LED_G_OFF_REG4_MASK) == LED_EN_OFF))
		{
			pr_devel("  [%s] enter if condition ,off mafor switch \n",__func__);
			pdata->shadow_reg[KTD2037_REG_LED_EN] &= ~(0xc0);//turn off major switch
		}
	pr_devel("  [%s] END \n",__func__);
}
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 E

static ssize_t store_led_r(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct ktd2037_data *data = dev_get_drvdata(dev);
	char buff[10] = {0,};
	int cnt, ret;
	u8 brightness;
	pr_devel("  [%s] START \n",__func__);
	cnt = count;
	cnt = (buf[cnt-1] == '\n') ? cnt-1 : cnt;
	memcpy(buff, buf, cnt);
	buff[cnt] = '\0';

	ret = kstrtou8(buff, 0, &brightness);
	if (ret != 0) {
		dev_err(&data->client->dev, "fail to get brightness.\n");
		goto out;
	}
	pr_devel("  [%s]  GREEN brightness=%d  \n",__func__,brightness);

//[5830][RGB Led] Map britness level PT Chang 20160328 S
	ktd2037_leds_map(&brightness);
//[5830][RGB Led] Map britness level PT Chang 20160328 E

//[5830][RGB Led]  I2C write one byte instead of write all to reduce fail rate in potentially PT Chang 20160315 S
	if (brightness == 0)
	{
		// If do not turn-off green , read green brightness would be wrong ,
		// The side-effect is that Led doesn't light in Vanilla when bat cap <15% , becuse HAL light Red only.
		// It would be normal in MMI build.
		ktd2037_leds_on_britness(LED_R, LED_EN_OFF, 0);
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 S
		//ktd2037_leds_on_britness(LED_G, LED_EN_OFF, 0);
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 E
	}
	else
	{
		ktd2037_leds_on_britness(LED_R, LED_EN_ON, brightness);
	}
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 S
	ktd2037_leds_faucet_config(data);
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 E
	ktd_write_byte(data, KTD2037_REG_LED_EN, data->shadow_reg[KTD2037_REG_LED_EN]);
	ktd_write_byte(data, KTD2037_REG_LED1, data->shadow_reg[KTD2037_REG_LED1]);
//[5830][RGB Led]  I2C write one byte instead of write all to reduce fail rate in potentially PT Chang 20160315 E
	pr_devel("  [%s] END \n",__func__);
out:
	return count;
}

static ssize_t store_led_g(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct ktd2037_data *data = dev_get_drvdata(dev);
	char buff[10] = {0,};
	int cnt, ret;
	u8 brightness;

	cnt = count;
	cnt = (buf[cnt-1] == '\n') ? cnt-1 : cnt;
	memcpy(buff, buf, cnt);
	buff[cnt] = '\0';

	pr_devel("  [%s]  START \n",__func__);
	ret = kstrtou8(buff, 0, &brightness);
	if (ret != 0) {
		dev_err(&data->client->dev, "fail to get brightness.\n");
		goto out;
	}
	pr_devel("  [%s]  GREEN brightness=%d  \n",__func__,brightness);

//[5830][RGB Led] Map britness level PT Chang 20160328 S
	ktd2037_leds_map(&brightness);
//[5830][RGB Led] Map britness level PT Chang 20160328 E

//[5830][RGB Led]  I2C write one byte instead of write all to reduce fail rate in potentially PT Chang 20160315 S
	if (brightness == 0)
	{
		// If do not turn-off green , read green brightness would be wrong ,
		// The side-effect is that Led doesn't light in Vanilla when bat cap <15% , becuse HAL light Red only.
		// It would be normal in MMI build.
		ktd2037_leds_on_britness(LED_G, LED_EN_OFF, 0);
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 S
		//ktd2037_leds_on_britness(LED_G, LED_EN_OFF, 0);
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 E
	}
	else
	{
		ktd2037_leds_on_britness(LED_G, LED_EN_ON, brightness);
	}
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 S
	ktd2037_leds_faucet_config(data);
//[5830][RGB Led]  fix LED light when charging at cap < 15%   PT Chang 20160419 E
	ktd_write_byte(data, KTD2037_REG_LED_EN, data->shadow_reg[KTD2037_REG_LED_EN]);
	ktd_write_byte(data, KTD2037_REG_LED2, data->shadow_reg[KTD2037_REG_LED2]);
	pr_devel("  [%s] END \n",__func__);
//[5830][RGB Led]  I2C write one byte instead of write all to reduce fail rate in potentially PT Chang 20160315 E
out:
	return count;
}

#endif

//[5830][RGB Led]  Read Led brightness  PT Chang 20160315 S
static ssize_t show_led_r(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ktd2037_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 200, "  Red Led 's brightness = %d \n" , data->shadow_reg[6]);
}

static ssize_t show_led_g(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ktd2037_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 200, "  Green Led 's brightness = %d \n" , data->shadow_reg[7]);
}
//[5830][RGB Led]  Read Led brightness  PT Chang 20160315 E

#ifdef SEC_LED_SPECIFIC
/* below nodes is SAMSUNG specific nodes */
//[5830][RGB Led]  Read Led brightness  PT Chang 20160315 S
static DEVICE_ATTR(led_r, 0664, show_led_r, store_led_r);
static DEVICE_ATTR(led_g, 0664, show_led_g, store_led_g);
//[5830][RGB Led]  Read Led brightness  PT Chang 20160315 E
#if 0
static DEVICE_ATTR(led_b, 0664, NULL, store_led_b);
#endif
/* led_pattern node permission is 664 */
/* To access sysfs node from other groups */
static DEVICE_ATTR(led_blink, 0664, NULL, store_ktd2037_led_blink);

#endif

#ifdef SEC_LED_SPECIFIC
static struct attribute *sec_led_attributes[] = {
	&dev_attr_led_r.attr,
	&dev_attr_led_g.attr,
#if 0
	&dev_attr_led_b.attr,
#endif
	&dev_attr_led_blink.attr,
	NULL,
};

static struct attribute_group sec_led_attr_group = {
	.attrs = sec_led_attributes,
};
#endif

static void ktd2037_set_AutoBlinkQ(void)
{
	struct ktd2037_data *data = i2c_get_clientdata(b_client);
	data->shadow_reg[KTD2037_REG_AUTONLINK] |=  0X06;
}

static int ktd_regulator_configure(struct regulator ** vcc_i2c)
{

	int ret;
	*vcc_i2c = regulator_get(&b_client->dev,
					"vcc_i2c");

	if (IS_ERR(*vcc_i2c)) {
		dev_err(&b_client->dev,
				"%s: Failed to get vdd regulator\n",
				__func__);
		return PTR_ERR(*vcc_i2c);
	}

	if (regulator_count_voltages( *vcc_i2c ) > 0) {
		ret = regulator_set_voltage(*vcc_i2c,
				KTD_I2C_VTG_MIN_UV,
				KTD_I2C_VTG_MAX_UV);
		if (ret) {
			dev_err(&b_client->dev,
				"Regulator set failed vdd ret=%d\n",
				ret);
			goto reg_vcc_put;
		}
	}

	return 0;

reg_vcc_put:
	regulator_put(*vcc_i2c);

	return ret;

}

static int reg_set_optimum_mode_check(struct regulator **reg, int load_uA)
{
	return (regulator_count_voltages(*reg) > 0) ?
		regulator_set_optimum_mode(*reg, load_uA) : 0;
}

static int  ktd_power_on( struct regulator * * vcc_i2c , bool on  )
{
	int retval;

	if (on == false)
		{
			goto power_off;
		}

	retval = reg_set_optimum_mode_check(vcc_i2c,
		KTD_I2C_LOAD_UA);
	if (retval < 0) {
		dev_err(&b_client->dev,
			"Regulator vcc_i2c set_opt failed rc=%d\n",
			retval);
		return retval;
	}

	retval = regulator_enable(*vcc_i2c);
	if (retval) {
		dev_err(&b_client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n",
			retval);
		goto error_reg_en_vcc_i2c;
	}

		return 0;

error_reg_en_vcc_i2c:
		reg_set_optimum_mode_check(vcc_i2c, 0);

power_off:
		reg_set_optimum_mode_check(vcc_i2c, 0);
		regulator_disable(*vcc_i2c);

	return 0;
}

static int  ktd2037_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct ktd2037_data *data;
	struct regulator *vcc;
	int ret , i2cret , retval;

	pr_devel(" [%s] \n",__func__ );
	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C.\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
		pr_err("%s: i2c functionality check error\n", __func__);
		dev_err(&client->dev, "need I2C_FUNC_SMBUS_WRITE_I2C_BLOCK.\n");
		return -EIO;
	}

	if (client->dev.of_node) {
		data = kzalloc(sizeof(*data), GFP_KERNEL);
		if (!data) {
			dev_err(&client->adapter->dev,
					"failed to allocate driver data.\n");
			return -ENOMEM;
		}
	} else
		data = client->dev.platform_data;

	i2c_set_clientdata(client, data);
	data->client = client;
	b_client = client;

	retval=ktd_regulator_configure(&vcc);
	if (retval < 0) {
		dev_err(&client->dev, "Failed to configure regulators\n");
	}

	retval = ktd_power_on(&vcc, true);
	if (retval < 0) {
		dev_err(&client->dev, "Failed to power on\n");
	}

	msleep(100);
	mutex_init(&data->mutex);

	/* initialize LED */
	/* turn off all leds */
	ktd2037_leds_on(LED_R, LED_EN_OFF, 0);
	ktd2037_leds_on(LED_G, LED_EN_OFF, 0);
	ktd2037_leds_on(LED_B, LED_EN_OFF, 0);
	ktd2037_set_timerslot_control(0);
	ktd2037_set_period(0);
	ktd2037_set_pwm_duty(PWM1, 0);
	ktd2037_set_pwm_duty(PWM2, 0);
	ktd2037_set_trise_tfall(0, 0, 0);
	ktd2037_set_AutoBlinkQ();

	i2cret=leds_i2c_write_all(client);

	if(i2cret < 0)
	{
		pr_devel("\n\n  #I2C  ktd2037_probe  ERROR###,,,    i2cret=%d# \n\n",i2cret);
	}

#ifdef SEC_LED_SPECIFIC
	sec_class = class_create(THIS_MODULE, "secleds");
	led_dev = device_create(sec_class, NULL, 0, data, "led");

	if (IS_ERR(led_dev)) {
		dev_err(&client->dev,
			"Failed to create device for samsung specific led\n");
		ret = -ENODEV;
		goto exit;
	}

	ret = sysfs_create_group(&led_dev->kobj, &sec_led_attr_group);
	if (ret) {
		dev_err(&client->dev,
			"Failed to create sysfs group for samsung specific led\n");
		goto exit;
	}
#endif
	return ret;
exit:
	mutex_destroy(&data->mutex);
	kfree(data);
	return ret;
}

static int  ktd2037_remove(struct i2c_client *client)
{
	struct ktd2037_data *data = i2c_get_clientdata(client);
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
#ifdef SEC_LED_SPECIFIC
	sysfs_remove_group(&led_dev->kobj, &sec_led_attr_group);
#endif
	mutex_destroy(&data->mutex);
	kfree(data);
	return 0;
}
//[5830][RGB Led]  Fix Red Led blink when phone turn-off PT Chang 20160314 S
static void ktd2037_shutdown(struct i2c_client *client )
{
	struct ktd2037_data *data = i2c_get_clientdata(client);
	int retval;
	pr_devel(" [%s] \n",__func__ );
	data->shadow_reg[KTD2037_REG_LED_EN] = 0x00 ;
	retval = ktd_write_byte(data, KTD2037_REG_LED_EN, data->shadow_reg[KTD2037_REG_LED_EN]);
	if (retval) {
		dev_err(&client->dev,
			"Failed to access ktd2037 by I2C to shutdown\n");
	}
}
//[5830][RGB Led]  Fix Red Led blink when phone turn-off PT Chang 20160314 E
static struct i2c_device_id ktd2037_id[] = {
	{"ktd2037", 0},
	{},
};

#ifdef CONFIG_OF
static struct of_device_id ktd2037_match_table[] = {
	{ .compatible = "leds,ktd2037",},
	{ },
};
#else
#define ktd2037_match_table NULL
#endif
MODULE_DEVICE_TABLE(i2c, ktd2037_id);

static struct i2c_driver ktd2037_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ktd2037",
		.of_match_table = ktd2037_match_table,
	},
	.id_table = ktd2037_id,
	.probe = ktd2037_probe,
	.remove = (ktd2037_remove),
//[5830][RGB Led]  Fix Red Led blink when phone turn-off PT Chang 20160314 S
	.shutdown = ktd2037_shutdown,
//[5830][RGB Led]  Fix Red Led blink when phone turn-off PT Chang 20160314 E
};

static int __init ktd2037_init(void)
{
	pr_devel("\n\n  #######  [%s]  ###### \n\n",__func__   );
	return i2c_add_driver(&ktd2037_i2c_driver);
}

static void __exit ktd2037_exit(void)
{
	i2c_del_driver(&ktd2037_i2c_driver);
}

module_init(ktd2037_init);
module_exit(ktd2037_exit);

MODULE_DESCRIPTION("KTD2037 LED driver");
MODULE_AUTHOR("Poting Chang <potingchang@arimacomm.com.tw>");
MODULE_LICENSE("GPL");
