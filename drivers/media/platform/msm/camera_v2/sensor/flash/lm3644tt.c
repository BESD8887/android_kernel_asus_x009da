/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <soc/qcom/camera2.h>
#include <media/msm_cam_sensor.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"
#include "../cci/msm_cci.h"

#define FLASH_NAME "ti,lm3644tt"

#define CONFIG_MSMB_LM3644TT_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_LM3644TT_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define IsEnableStateCheck	0

#define REG_ENABLE		0x01
#define REG_IVFM		0x02
#define REG_1_FLASH_CR	0x03
#define REG_2_FLASH_CR	0x04
#define REG_1_TORCH_CR	0x05
#define REG_2_TORCH_CR	0x06
#define REG_BOOST		0x07
#define REG_TIMING		0x08
#define REG_TEMP		0x09
#define REG_FLAG1		0x0A
#define REG_FLAG2		0x0B
#define REG_DEV_ID		0x0C
#define REG_LAST_FLASH	0x0D

#define LED_OFF_VAL	255
#define MAX_FLASH_CR		127 // 1500mA
#define MAX_TORCH_CR		127 // 360mA

#define DEFAULT_FLASH_CURR1		50
#define DEFAULT_FLASH_CURR2		84
#define DEFAULT_TORCH_CURR1		20
#define DEFAULT_TORCH_CURR2		28

#define DEFAULT_FLASH_VAL		0x0C
#define DEFAULT_TORCH_VAL		0x08
#define DEFAULT_FLASH_MODE		3
#define DEFAULT_TORCH_MODE		2

#define PROC_ENTRY_FLASH_ASUS  "driver/asus_flash_brightness"
#define PROC_ENTRY_FLASH_LED1  "driver/flash_led1"
#define PROC_ENTRY_FLASH_LED2  "driver/flash_led2"

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm3644tt_i2c_driver;

struct lm3644tt_ctrl_mode {
	u8 led1_en:1;
	u8 led2_en:1;
	u8 led_mode:2;
	u8 torch_en:1;
	u8 strobe_en:1;
	u8 strobe_type:1;
	u8 tx_en:1;
};
union u_lm3644tt_ctrl_mode {
	struct lm3644tt_ctrl_mode st;
	u8 value;
};

uint16_t *g_f1 = NULL;
uint16_t *g_f2 = NULL;
uint16_t *g_t1 = NULL;
uint16_t *g_t2 = NULL;
u8 g_chip_en = 0;
u8 g_stop_update_current = 0;
union u_lm3644tt_ctrl_mode g_ctrl_mode;

static struct msm_camera_i2c_reg_array lm3644tt_init_array[] = {
	{REG_ENABLE,	0x00},
};

static struct msm_camera_i2c_reg_array lm3644tt_off_array[] = {
	{REG_ENABLE,	0x00},
};

static struct msm_camera_i2c_reg_array lm3644tt_release_array[] = {
	{REG_ENABLE,	0x00},
};

static struct msm_camera_i2c_reg_array lm3644tt_low_array[] = {
	{REG_1_TORCH_CR,	DEFAULT_TORCH_CURR1},
	{REG_2_TORCH_CR,	DEFAULT_TORCH_CURR2},
	{REG_TIMING,		0x0B}, // Time of timeout, Torch no ramp, Flash=800ms
	{REG_ENABLE,		DEFAULT_TORCH_VAL + 0x03}, // Torch On
};
static struct msm_camera_i2c_reg_array lm3644tt_low_write_array[] = {
	{REG_1_TORCH_CR,	DEFAULT_TORCH_CURR1},
	{REG_2_TORCH_CR,	DEFAULT_TORCH_CURR2},
	{REG_TIMING,		0x0B}, // Time of timeout, Torch no ramp, Flash=800ms
	{REG_ENABLE,		DEFAULT_TORCH_VAL + 0x03}, // Torch On
};

static struct msm_camera_i2c_reg_array lm3644tt_high_array[] = {
	{REG_1_FLASH_CR,	DEFAULT_FLASH_CURR1},
	{REG_2_FLASH_CR,	DEFAULT_FLASH_CURR2},
	{REG_TIMING,		0x0B}, // Time of timeout, Torch no ramp, Flash=800ms
	{REG_ENABLE,		DEFAULT_FLASH_VAL + 0x03}, // Flash On
};
static struct msm_camera_i2c_reg_array lm3644tt_high_write_array[] = {
	{REG_1_FLASH_CR,	DEFAULT_FLASH_CURR1},
	{REG_2_FLASH_CR,	DEFAULT_FLASH_CURR2},
	{REG_TIMING,		0x0B}, // Time of timeout, Torch no ramp, Flash=800ms
	{REG_ENABLE,		DEFAULT_FLASH_VAL + 0x03}, // Flash On
};

#define FLASH_CURR_M	11725
#define FLASH_CURR_A	10900
#define TORCH_CURR_M	2800
#define TORCH_CURR_A	1954

// Current uA = reg * M + A
static u8 lm3644tt_convert_mA_to_reg(unsigned int mA, unsigned int M, unsigned int A) {
	unsigned int reg_val = 0;
	unsigned int uA = 1000 * mA;

	if (mA == 0) return LED_OFF_VAL;

	// uA = reg * M + A --> reg = (uA - A) / M
	if (uA <= A) uA = A;
	reg_val = (uA - A) / M;

	return (unsigned char)reg_val;
}

// ======== Sysfs Attributes regions BEGIN ========
static u8 DecideCurrent(u8 off_val, u8 max_val, u8 val) {
	if (val == off_val) return off_val;
	if (val >= max_val) return max_val;
	return val;
}

static ssize_t lm3644tt_show_f1(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("[%s] BEGIN g_f1=%d \n", __func__, *g_f1);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", *g_f1);

	CDBG("[%s] END \n", __func__);
	return rc;
}
static ssize_t lm3644tt_save_f1(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 input_value;
	CDBG("[%s] BEGIN Buff=%s \n", __func__, buf);
	if (sscanf(buf, "%hhu", &input_value) != 1) {
		dev_err(dev, "Input value error \n");
		return -EINVAL;
	}
	*g_f1 = DecideCurrent(LED_OFF_VAL ,MAX_FLASH_CR , input_value);

	CDBG("[%s] END \n", __func__);
	return count;
}
static DEVICE_ATTR(f1, 0664, lm3644tt_show_f1, lm3644tt_save_f1);

static ssize_t lm3644tt_show_f2(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("[%s] BEGIN g_f2=%d \n", __func__, *g_f2);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", *g_f2);

	CDBG("[%s] END \n", __func__);
	return rc;
}
static ssize_t lm3644tt_save_f2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 input_value;
	CDBG("[%s] BEGIN Buff=%s \n", __func__, buf);
	if (sscanf(buf, "%hhu", &input_value) != 1) {
		dev_err(dev, "Input value error \n");
		return -EINVAL;
	}
	*g_f2 =  DecideCurrent(LED_OFF_VAL ,MAX_FLASH_CR , input_value);

	CDBG("[%s] END \n", __func__);
	return count;
}
static DEVICE_ATTR(f2, 0664, lm3644tt_show_f2, lm3644tt_save_f2);

static ssize_t lm3644tt_show_t1(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("[%s] BEGIN g_t1=%d \n", __func__, *g_t1);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", *g_t1);

	CDBG("[%s] END \n", __func__);
	return rc;
}
static ssize_t lm3644tt_save_t1(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 input_value;
	CDBG("[%s] BEGIN Buff=%s \n", __func__, buf);
	if (sscanf(buf, "%hhu", &input_value) != 1) {
		dev_err(dev, "Input value error \n");
		return -EINVAL;
	}
	*g_t1 = DecideCurrent(LED_OFF_VAL, MAX_TORCH_CR, input_value);

	CDBG("[%s] END \n", __func__);
	return count;
}
static DEVICE_ATTR(t1, 0664, lm3644tt_show_t1, lm3644tt_save_t1);

static ssize_t lm3644tt_show_t2(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("[%s] BEGIN g_t2=%d \n", __func__, *g_t2);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", *g_t2);

	CDBG("[%s] END \n", __func__);
	return rc;
}
static ssize_t lm3644tt_save_t2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 input_value;
	CDBG("[%s] BEGIN Buff=%s \n", __func__, buf);
	if (sscanf(buf, "%hhu", &input_value) != 1) {
		dev_err(dev, "Input value error \n");
		return -EINVAL;
	}
	*g_t2 = DecideCurrent(LED_OFF_VAL, MAX_TORCH_CR, input_value);

	CDBG("[%s] END \n", __func__);
	return count;
}
static DEVICE_ATTR(t2, 0664, lm3644tt_show_t2, lm3644tt_save_t2);

static ssize_t lm3644tt_show_flash(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("[%s] BEGIN \n", __func__	);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", 0);

	fctrl.func_tbl->flash_led_init(&fctrl);
	fctrl.func_tbl->flash_led_high(&fctrl);

	CDBG("[%s] END \n", __func__);
	return rc;
}
static ssize_t lm3644tt_save_flash(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 input_value;
	CDBG("[%s] BEGIN Buff=%s \n", __func__, buf);
	if (sscanf(buf, "%hhu", &input_value) != 1) {
		dev_err(dev, "Input value error \n");
		return -EINVAL;
	}
	if (input_value >= 1) {
		fctrl.func_tbl->flash_led_init(&fctrl);
		fctrl.func_tbl->flash_led_high(&fctrl);
	} else {
		fctrl.func_tbl->flash_led_off(&fctrl);
	}

	CDBG("[%s] END \n", __func__);
	return count;
}
static DEVICE_ATTR(flash, 0664, lm3644tt_show_flash, lm3644tt_save_flash);

static ssize_t lm3644tt_show_torch(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	CDBG("[%s] BEGIN \n", __func__);
	rc = scnprintf(buf, PAGE_SIZE, "%d\n", 0);

	fctrl.func_tbl->flash_led_init(&fctrl);
	fctrl.func_tbl->flash_led_low(&fctrl);

	CDBG("[%s] END \n", __func__);
	return rc;
}
static ssize_t lm3644tt_save_torch(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 input_value;
	CDBG("[%s] BEGIN Buff=%s \n", __func__, buf);
	if (sscanf(buf, "%hhu", &input_value) != 1) {
		dev_err(dev, "Input value error \n");
		return -EINVAL;
	}
	if (input_value >= 1) {
		fctrl.func_tbl->flash_led_init(&fctrl);
		fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
		fctrl.func_tbl->flash_led_off(&fctrl);
	}

	CDBG("[%s] END \n", __func__);
	return count;
}
static DEVICE_ATTR(torch, 0664, lm3644tt_show_torch, lm3644tt_save_torch);

static ssize_t lm3644tt_show_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	int ret = 0;
	uint16_t read_data = 255;

	CDBG("[%s] BEGIN \n", __func__);

	fctrl.func_tbl->flash_led_init(&fctrl);
	ret = fctrl.flash_i2c_client->i2c_func_tbl->i2c_read(fctrl.flash_i2c_client, REG_DEV_ID, &read_data, MSM_CAMERA_I2C_BYTE_DATA);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", read_data);

	CDBG("[%s] END read_data=%d ret=%d \n", __func__, read_data, ret);
	return rc;
}
static ssize_t lm3644tt_save_id(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 input_value;
	CDBG("[%s] BEGIN Buff=%s \n", __func__, buf);
	if (sscanf(buf, "%hhu", &input_value) != 1) {
		dev_err(dev, "Input value error \n");
		return -EINVAL;
	}

	CDBG("[%s] END \n", __func__);
	return count;
}
static DEVICE_ATTR(id, 0664, lm3644tt_show_id, lm3644tt_save_id);

u8 g_addr = 0;
static ssize_t lm3644tt_show_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;

	CDBG("[%s] BEGIN \n", __func__);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", g_addr);

	CDBG("[%s] END read_data=%d ret=%d \n", __func__, g_addr, rc);
	return rc;
}
static ssize_t lm3644tt_save_addr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	CDBG("[%s] BEGIN Buff=%s \n", __func__, buf);
	if (sscanf(buf, "%hhu", &g_addr) != 1) {
		dev_err(dev, "Input value error \n");
		return -EINVAL;
	}

	CDBG("[%s] END Buff=%s Addr=%d \n", __func__, buf, g_addr);
	return count;
}
static DEVICE_ATTR(addr, 0664, lm3644tt_show_addr, lm3644tt_save_addr);

static ssize_t lm3644tt_show_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc = 0;
	int ret = 0;
	uint16_t read_data = 255;

	CDBG("[%s] BEGIN \n", __func__);

	fctrl.func_tbl->flash_led_init(&fctrl);
	ret = fctrl.flash_i2c_client->i2c_func_tbl->i2c_read(fctrl.flash_i2c_client, g_addr, &read_data, MSM_CAMERA_I2C_BYTE_DATA);

	rc = scnprintf(buf, PAGE_SIZE, "%d\n", read_data);

	CDBG("[%s] END read_data=%d ret=%d \n", __func__, read_data, ret);
	return rc;
}
static ssize_t lm3644tt_save_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 input_value;
	int ret = 0;
	CDBG("[%s] BEGIN Buff=%s \n", __func__, buf);
	if (sscanf(buf, "%hhu", &input_value) != 1) {
		dev_err(dev, "Input value error \n");
		return -EINVAL;
	}

	fctrl.func_tbl->flash_led_init(&fctrl);
	ret = fctrl.flash_i2c_client->i2c_func_tbl->i2c_write(fctrl.flash_i2c_client, g_addr, input_value, MSM_CAMERA_I2C_BYTE_DATA);

	CDBG("[%s] END Buff=%s, Data=%d Ret=%d \n", __func__, buf, input_value, ret);
	return count;
}
static DEVICE_ATTR(data, 0664, lm3644tt_show_data, lm3644tt_save_data);

static struct device_attribute *lm3644tt_class_attrs[] = {
	&dev_attr_f1,
	&dev_attr_f2,
	&dev_attr_flash,
	&dev_attr_t1,
	&dev_attr_t2,
	&dev_attr_torch,
	&dev_attr_id,
	&dev_attr_addr,
	&dev_attr_data,
};

static int lm3644tt_create_attr(struct device *dev)
{
	int idx, err = 0;
	int num = (int)(sizeof(lm3644tt_class_attrs)/sizeof(lm3644tt_class_attrs[0]));

	CDBG("[%s] BEGIN \n", __func__);
	if (!dev)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		if ((err = device_create_file(dev, lm3644tt_class_attrs[idx])))
		{
			//PK_DBG("device_create_file (%s) = %d\n", flashlight_class_attrs[idx]->attr.name, err);
			break;
		}
	}

	CDBG("[%s] END ERR=%d \n", __func__, err);
	return err;
}
// ======== Sysfs Attributes regions END ========

// ======== proc entries regions BEGIN ========
static struct proc_dir_entry *g_proc_asus; // For ASUS required
static struct proc_dir_entry *g_proc_led1; // For ATS LED1
static struct proc_dir_entry *g_proc_led2; // For ATS LED2

static u8 g_asus_input_value;
static int lm3644tt_asus_proc_read (struct seq_file *buf, void *v)
{
	CDBG("[%s] BEGIN \n", __func__);

	seq_printf(buf, "%d\n", g_asus_input_value);

	CDBG("[%s] END \n", __func__);
	return 0;
}
static ssize_t lm3644tt_asus_proc_write(struct file *filp, const char __user *buffer, size_t count, loff_t *data)
{
	char buf[] = "0x00000000";
	unsigned long len = min((unsigned long)sizeof(buf) - 1, count);
	u32 torch_cr;

	CDBG("[%s] BEGIN Buff=%s \n", __func__, buffer);
	if (copy_from_user(buf, buffer, len)) return count;
	buf[len] = 0;
	if (sscanf(buf, "%hhu", &g_asus_input_value) != 1) {
		return -EINVAL;
	}

	// input_value == 0: Turn off the LED
	if (g_asus_input_value <= 0) torch_cr = LED_OFF_VAL;
	else if (g_asus_input_value >= 200) torch_cr = MAX_TORCH_CR;
	else {
		torch_cr = (g_asus_input_value * 128) / 200;
	}

	if (torch_cr == LED_OFF_VAL) {
		fctrl.func_tbl->flash_led_init(&fctrl);
		fctrl.func_tbl->flash_led_off(&fctrl);
		fctrl.func_tbl->flash_led_release(&fctrl);
	} else {
		fctrl.reg_setting->low_setting->reg_setting[0].reg_data = torch_cr;
		fctrl.reg_setting->low_setting->reg_setting[1].reg_data = torch_cr;
		fctrl.func_tbl->flash_led_init(&fctrl);
		g_stop_update_current = 1;
		fctrl.func_tbl->flash_led_low(&fctrl);
	}

	CDBG("[%s] END input_value=%d, torch_cr=%d\n", __func__, g_asus_input_value, torch_cr);
	return strnlen(buf, count);;
}
static int lm3644tt_asus_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, lm3644tt_asus_proc_read, NULL);
}
static const struct file_operations g_asus_proc_fops = {
	.owner = THIS_MODULE,
	.open = lm3644tt_asus_proc_open,
	.write = lm3644tt_asus_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int lm3644tt_led1_proc_read (struct seq_file *buf, void *v)
{
	CDBG("[%s] BEGIN \n", __func__);

	CDBG("[%s] END \n", __func__);
	return 0;
}
static ssize_t lm3644tt_led1_proc_write(struct file *filp, const char __user *buffer, size_t count, loff_t *data)
{
	char buf[] = "0x00000000";
	unsigned long len = min((unsigned long)sizeof(buf) - 1, count);
	u8 input_value;

	CDBG("[%s] BEGIN Buff=%s \n", __func__, buffer);
	if (copy_from_user(buf, buffer, len)) return count;
	buf[len] = 0;
	if (sscanf(buf, "%hhu", &input_value) != 1) {
		return -EINVAL;
	}

	fctrl.reg_setting->low_setting->reg_setting[1].reg_data = LED_OFF_VAL;
	if (input_value <= 0) {
		fctrl.reg_setting->low_setting->reg_setting[0].reg_data = LED_OFF_VAL;
		fctrl.func_tbl->flash_led_off(&fctrl);
		fctrl.func_tbl->flash_led_release(&fctrl);
	} else {
		fctrl.reg_setting->low_setting->reg_setting[0].reg_data = DEFAULT_TORCH_CURR1;
		fctrl.func_tbl->flash_led_init(&fctrl);
		g_stop_update_current = 1;
		fctrl.func_tbl->flash_led_low(&fctrl);
	}

	CDBG("[%s] END input_value=%d \n", __func__, input_value);
	return strnlen(buf, count);;
}
static int lm3644tt_led1_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, lm3644tt_led1_proc_read, NULL);
}
static const struct file_operations g_led1_proc_fops = {
	.owner = THIS_MODULE,
	.open = lm3644tt_led1_proc_open,
	.write = lm3644tt_led1_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int lm3644tt_led2_proc_read (struct seq_file *buf, void *v)
{
	CDBG("[%s] BEGIN \n", __func__);

	CDBG("[%s] END \n", __func__);
	return 0;
}
static ssize_t lm3644tt_led2_proc_write(struct file *filp, const char __user *buffer, size_t count, loff_t *data)
{
	char buf[] = "0x00000000";
	unsigned long len = min((unsigned long)sizeof(buf) - 1, count);
	u8 input_value;

	CDBG("[%s] BEGIN Buff=%s \n", __func__, buffer);
	if (copy_from_user(buf, buffer, len)) return count;
	buf[len] = 0;
	if (sscanf(buf, "%hhu", &input_value) != 1) {
		return -EINVAL;
	}

	fctrl.reg_setting->low_setting->reg_setting[0].reg_data = LED_OFF_VAL;
	if (input_value <= 0) {
		fctrl.reg_setting->low_setting->reg_setting[1].reg_data = LED_OFF_VAL;
		fctrl.func_tbl->flash_led_off(&fctrl);
		fctrl.func_tbl->flash_led_release(&fctrl);
	} else {
		fctrl.reg_setting->low_setting->reg_setting[1].reg_data = DEFAULT_TORCH_CURR2;
		fctrl.func_tbl->flash_led_init(&fctrl);
		g_stop_update_current = 1;
		fctrl.func_tbl->flash_led_low(&fctrl);
	}

	CDBG("[%s] END input_value=%d \n", __func__, input_value);
	return strnlen(buf, count);;
}
static int lm3644tt_led2_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, lm3644tt_led2_proc_read, NULL);
}
static const struct file_operations g_led2_proc_fops = {
	.owner = THIS_MODULE,
	.open = lm3644tt_led2_proc_open,
	.write = lm3644tt_led2_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void lm3644tt_create_proc_file(void)
{
	g_proc_asus = proc_create(PROC_ENTRY_FLASH_ASUS, 0666, NULL, &g_asus_proc_fops);
	if (g_proc_asus) CDBG("[%s] proc entry %s created DONE! \n", __func__, PROC_ENTRY_FLASH_ASUS);
	else CDBG("[%s] proc entry %s created FAILed... \n", __func__, PROC_ENTRY_FLASH_ASUS);

	g_proc_led1 = proc_create(PROC_ENTRY_FLASH_LED1, 0666, NULL, &g_led1_proc_fops);
	if (g_proc_led1) CDBG("[%s] proc entry %s created DONE! \n", __func__, PROC_ENTRY_FLASH_LED1);
	else CDBG("[%s] proc entry %s created FAILed... \n", __func__, PROC_ENTRY_FLASH_LED1);

	g_proc_led2 = proc_create(PROC_ENTRY_FLASH_LED2, 0666, NULL, &g_led2_proc_fops);
	if (g_proc_led2) CDBG("[%s] proc entry %s created DONE! \n", __func__, PROC_ENTRY_FLASH_LED2);
	else CDBG("[%s] proc entry %s created FAILed... \n", __func__, PROC_ENTRY_FLASH_LED2);
}
// ======== proc entries regions END ========

static void lm3644tt_init_default_value(void)
{
	CDBG("[%s] BEGIN \n", __func__);

	g_f1 = &(lm3644tt_high_array[0].reg_data);
	g_f2 = &(lm3644tt_high_array[1].reg_data);
	g_t1 = &(lm3644tt_low_array[0].reg_data);
	g_t2 = &(lm3644tt_low_array[1].reg_data);

	g_ctrl_mode.value = 0;

	CDBG("[%s] END Flash=%d,%d Torch=%d,%d \n", __func__, *g_f1, *g_f2, *g_t1, *g_t2);
}

static void lm3644_switch_ic(u8 onoff)
{
	struct msm_camera_power_ctrl_t *power_info = &fctrl.flashdata->power_info;
	if (onoff == 0) {
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_HIGH);

		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_HIGH);

		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
	} else {
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);

		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_HIGH);

		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_HIGH);
	}
}

int lm3644tt_flash_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = &fctrl->flashdata->power_info;

	CDBG("[%s] BEGIN \n", __func__);

#if IsEnableStateCheck
	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
#endif

	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
	}

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("cci_init failed\n");
			return rc;
		}
	}

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		CDBG("[%s] request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		CDBG("%s:%d PC:: flash pins setting to active state \n",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
		}
	}
	msleep(20);

	lm3644_switch_ic(1);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
#if IsEnableStateCheck
	fctrl->led_state = MSM_CAMERA_LED_INIT;
#endif

	CDBG("[%s] END \n", __func__);
	return rc;
}

int lm3644tt_flash_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = &fctrl->flashdata->power_info;

	CDBG("[%s] BEGIN \n", __func__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	if (!power_info) {
		pr_err("%s:%d power_info NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
#if IsEnableStateCheck
	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
#endif

	fctrl->func_tbl->flash_led_off(fctrl);

	lm3644_switch_ic(0);

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
		}
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

#if IsEnableStateCheck
	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
#endif
	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}

	CDBG("[%s] END \n", __func__);
	return 0;
}

int lm3644tt_flash_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	CDBG("[%s] BEGIN \n", __func__);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

#if IsEnableStateCheck
	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
#endif

	lm3644_switch_ic(1);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	lm3644_switch_ic(0);

	CDBG("[%s] END rc=%d \n", __func__, rc);
	return rc;
}

int lm3644tt_flash_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	CDBG("[%s] BEGIN Curr=(%d, %d) \n", __func__, fctrl->torch_op_current[0], fctrl->torch_op_current[1]);
#if IsEnableStateCheck
	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
#endif

	// Stop auto update current when called by proc entries
	if (g_stop_update_current == 1) {
		g_stop_update_current = 0;
	} else {
		if (fctrl->torch_op_current[0] == 0 && fctrl->torch_op_current[1] == 0)
		{
			*g_t1 = DEFAULT_TORCH_CURR1;
			*g_t2 = DEFAULT_TORCH_CURR2;
		} else {
			*g_t1 = lm3644tt_convert_mA_to_reg(fctrl->torch_op_current[0], TORCH_CURR_M, TORCH_CURR_A);
			*g_t2 = lm3644tt_convert_mA_to_reg(fctrl->torch_op_current[1], TORCH_CURR_M, TORCH_CURR_A);
		}
	}

	// Turn Each LED on/off if it was set as LED_OFF_VAL
	g_ctrl_mode.st.led_mode = DEFAULT_TORCH_MODE;
	g_ctrl_mode.st.led1_en = *g_t1 == LED_OFF_VAL ? 0 : 1;
	g_ctrl_mode.st.led2_en = *g_t2 == LED_OFF_VAL ? 0 : 1;
	fctrl->reg_setting->low_setting->reg_setting[3].reg_data = g_ctrl_mode.value;

	lm3644tt_low_write_array[0].reg_data = *g_t1 == LED_OFF_VAL ? *g_t1 - 128 : *g_t1;
	lm3644tt_low_write_array[1].reg_data = *g_t2 == LED_OFF_VAL ? *g_t2 - 128 : *g_t2;
	lm3644tt_low_write_array[3].reg_data = g_ctrl_mode.value;

	lm3644_switch_ic(1);

	fctrl->reg_setting->low_setting->reg_setting = lm3644tt_low_write_array;
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	CDBG("[%s] Torch=%d,%d mode=0x%d \n", __func__, lm3644tt_low_write_array[0].reg_data, lm3644tt_low_write_array[1].reg_data, lm3644tt_low_write_array[3].reg_data);
	CDBG("[%s] Mode set 0x%x to 0x%x \n", __func__, fctrl->reg_setting->low_setting->reg_setting[2].reg_data, fctrl->reg_setting->low_setting->reg_setting[2].reg_addr);
	fctrl->reg_setting->low_setting->reg_setting = lm3644tt_low_array;

	CDBG("[%s] END rc=%d \n", __func__, rc);
	return rc;
}

int lm3644tt_flash_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;

	CDBG("[%s] BEGIN Curr=(%d, %d) \n", __func__, fctrl->flash_op_current[0], fctrl->flash_op_current[1]);

#if IsEnableStateCheck
	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
#endif

	*g_f1 = lm3644tt_convert_mA_to_reg(fctrl->flash_op_current[0], FLASH_CURR_M, FLASH_CURR_A);
	*g_f2 = lm3644tt_convert_mA_to_reg(fctrl->flash_op_current[1], FLASH_CURR_M, FLASH_CURR_A);

	// Turn Each LED on/off if it was set as LED_OFF_VAL
	g_ctrl_mode.st.led_mode = DEFAULT_FLASH_MODE;
	g_ctrl_mode.st.led1_en = *g_f1 == LED_OFF_VAL ? 0 : 1;
	g_ctrl_mode.st.led2_en = *g_f2 == LED_OFF_VAL ? 0 : 1;
	fctrl->reg_setting->high_setting->reg_setting[3].reg_data = g_ctrl_mode.value;

	lm3644tt_high_write_array[0].reg_data = *g_f1 == LED_OFF_VAL ? *g_f1 - 128 : *g_f1;
	lm3644tt_high_write_array[1].reg_data = *g_f2 == LED_OFF_VAL ? *g_f2 - 128 : *g_f2;
	lm3644tt_high_write_array[3].reg_data = g_ctrl_mode.value;

	lm3644_switch_ic(1);

	fctrl->reg_setting->high_setting->reg_setting = lm3644tt_high_write_array;
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	CDBG("[%s] Flash=%d,%d mode=0x%d \n", __func__, lm3644tt_high_write_array[0].reg_data, lm3644tt_high_write_array[1].reg_data, lm3644tt_high_write_array[3].reg_data);
	CDBG("[%s] Mode set 0x%x to 0x%x \n", __func__, fctrl->reg_setting->high_setting->reg_setting[2].reg_data, fctrl->reg_setting->high_setting->reg_setting[2].reg_addr);
	fctrl->reg_setting->high_setting->reg_setting = lm3644tt_high_array;

	CDBG("[%s] END rc=%d \n", __func__, rc);
	return rc;
}

static struct msm_camera_i2c_reg_setting lm3644tt_init_setting = {
	.reg_setting = lm3644tt_init_array,
	.size = ARRAY_SIZE(lm3644tt_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3644tt_off_setting = {
	.reg_setting = lm3644tt_off_array,
	.size = ARRAY_SIZE(lm3644tt_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3644tt_release_setting = {
	.reg_setting = lm3644tt_release_array,
	.size = ARRAY_SIZE(lm3644tt_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3644tt_low_setting = {
	.reg_setting = lm3644tt_low_array,
	.size = ARRAY_SIZE(lm3644tt_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3644tt_high_setting = {
	.reg_setting = lm3644tt_high_array,
	.size = ARRAY_SIZE(lm3644tt_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm3644tt_regs = {
	.init_setting = &lm3644tt_init_setting,
	.off_setting = &lm3644tt_off_setting,
	.low_setting = &lm3644tt_low_setting,
	.high_setting = &lm3644tt_high_setting,
	.release_setting = &lm3644tt_release_setting,
};

static struct msm_flash_fn_t lm3644tt_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = lm3644tt_flash_led_init,
	.flash_led_release = lm3644tt_flash_led_release,
	.flash_led_off = lm3644tt_flash_led_off,
	.flash_led_low = lm3644tt_flash_led_low,
	.flash_led_high = lm3644tt_flash_led_high,
};

static const struct of_device_id lm3644tt_i2c_trigger_dt_match[] = {
	{.compatible = FLASH_NAME},
	{}
};

MODULE_DEVICE_TABLE(of, lm3644tt_i2c_trigger_dt_match);

static const struct i2c_device_id lm3644tt_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int msm_flash_lm3644tt_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0;
	if (!id) {
		pr_err("msm_flash_lm3644tt_i2c_probe: id is NULL \n");
		id = lm3644tt_i2c_id;
	}
	CDBG("[%s] BEGIN \n", __func__);

	lm3644tt_init_default_value();

	lm3644tt_create_attr(&(client->dev));

	lm3644tt_create_proc_file();

	rc = msm_flash_i2c_probe(client, id);
	CDBG("[%s] END rc=%d \n", __func__, rc);
	return rc;
}

static int msm_flash_lm3644tt_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver lm3644tt_i2c_driver = {
	.id_table = lm3644tt_i2c_id,
	.probe  = msm_flash_lm3644tt_i2c_probe,
	.remove = msm_flash_lm3644tt_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm3644tt_i2c_trigger_dt_match,
	},
};

static int msm_flash_lm3644tt_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	match = of_match_device(lm3644tt_i2c_trigger_dt_match, &pdev->dev);
	if (!match)
		return -EFAULT;
	return msm_flash_probe(pdev, match->data);
}

static struct platform_driver lm3644tt_platform_driver = {
	.probe = msm_flash_lm3644tt_platform_probe,
	.driver = {
		.name = "qcom,led-flash",
		.owner = THIS_MODULE,
		.of_match_table = lm3644tt_i2c_trigger_dt_match,
	},
};

static int __init msm_flash_lm3644tt_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&lm3644tt_platform_driver);
	if (fctrl.pdev != NULL && rc == 0) {
		pr_err("lm3644tt platform_driver_register success");
		return rc;
	} else if (rc != 0) {
		pr_err("lm3644tt platform_driver_register failed");
		return rc;
	} else {
		rc = i2c_add_driver(&lm3644tt_i2c_driver);
		if (!rc)
			pr_err("lm3644tt i2c_add_driver success");
	}
	return rc;
}

static void __exit msm_flash_lm3644tt_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&lm3644tt_platform_driver);
	else
		i2c_del_driver(&lm3644tt_i2c_driver);
}

static struct msm_camera_i2c_client lm3644tt_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm3644tt_i2c_client,
	.reg_setting = &lm3644tt_regs,
	.func_tbl = &lm3644tt_func_tbl,
};

module_init(msm_flash_lm3644tt_init_module);
module_exit(msm_flash_lm3644tt_exit_module);
MODULE_DESCRIPTION("lm3644tt FLASH");
MODULE_LICENSE("GPL v2");
