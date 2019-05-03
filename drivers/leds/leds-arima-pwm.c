
/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define CONFIG_MSM_LEDS_ARIMA_PWM_DEBUG
#undef CDBG
#ifdef CONFIG_MSM_LEDS_ARIMA_PWM_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_info(fmt, ##args)
#endif

#define LED_DEV_NAME	"arima,leds-pwm"

#define MPP_MAX_LEVEL			LED_FULL
#define LED_MPP_MODE_CTRL(base)		(base + 0x40)
#define LED_MPP_EN_CTRL(base)		(base + 0x46)
#define LED_MPP_SINK_CTRL(base)		(base + 0x4C)

#define LED_MPP_EN_ENABLE		0x80
#define LED_MPP_EN_DISABLE		0x00
#define LED_MOO_EN_CURR_SNK		0x68

#define LED_RED_BASE	0xA100
#define LED_GREEN_BASE	0xA300

/**
 * struct qpnp_led_data - internal led data structure
 * @led_classdev - led class device
 * @work - workqueue for led
 * @id - led index
 * @base_reg - base register given in device tree
 * @lock - to protect the transactions
 * @reg - cached value of led register
 * @num_leds - number of leds in the module
 */
struct qpnp_led_data {
	struct led_classdev	cdev;
	struct spmi_device	*spmi_dev;
	struct work_struct	work;
	u16			base;
	u8			reg;
	u8			num_leds;
	struct mutex		lock;
};

static int arima_pwm_led_write(struct qpnp_led_data *led, u8 sid, u16 addr, u8 val)
{
	int rc = 0;

	rc = spmi_ext_register_writel(led->spmi_dev->ctrl, sid, addr, &val, 1);
	CDBG("%s Write SID=%d 0x%04X : 0x%02X (%d) \n", __func__, sid, addr, val, rc);
	if (rc) dev_err(&led->spmi_dev->dev, "Unable to write to addr=%x, rc(%d)\n", addr, rc);
	return rc;
}

static void arima_pwm_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct qpnp_led_data *led;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	CDBG("[%s] BEGIN LED_Name=%s, BRI=%d\n", __func__, led_cdev->name, value);

	if (value < LED_OFF) {
		dev_err(&led->spmi_dev->dev, "Invalid brightness value\n");
		return;
	}

	value = value > led->cdev.max_brightness ? led->cdev.max_brightness : value;

	schedule_work(&led->work);

	CDBG("[%s] END \n", __func__);
}

static void __arima_pwm_led_work(struct qpnp_led_data *led, u16 value)
{
	u8 value_lo = 0, value_hi = 0;

	CDBG("[%s] BEGIN LED_Name=%s Value=%d \n", __func__, led->cdev.name, value);
	mutex_lock(&led->lock);

	if (value > 0) {
		value_hi = (u8)(value * 2 / 256);
		value_lo = (u8)(value * 2 % 256);
		arima_pwm_led_write(led, 1, 0xbc41, 7);
		arima_pwm_led_write(led, 1, 0xbc42, 2);
		arima_pwm_led_write(led, 1, 0xbc44, value_lo);
		arima_pwm_led_write(led, 1, 0xbc45, value_hi);
		arima_pwm_led_write(led, 1, 0xbc47, 1);
		arima_pwm_led_write(led, 0, LED_MPP_EN_CTRL(led->base), LED_MPP_EN_ENABLE);
	} else {
		arima_pwm_led_write(led, 0, LED_MPP_EN_CTRL(led->base), LED_MPP_EN_DISABLE);
	}

	mutex_unlock(&led->lock);
	CDBG("[%s] END \n", __func__);
}

static void arima_pwm_led_work(struct work_struct *work)
{
	struct qpnp_led_data *led = container_of(work, struct qpnp_led_data, work);

	__arima_pwm_led_work(led, led->cdev.brightness);

	return;
}

static enum led_brightness arima_pwm_led_get(struct led_classdev *led_cdev)
{
	struct qpnp_led_data *led;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	return led->cdev.brightness;
}

static ssize_t arima_pwm_pattern_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u8 pattern;
	ssize_t ret = -EINVAL;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	CDBG("[%s] BEGIN buf(pattern)=%s \n", __func__, buf);

	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	ret = sscanf(buf, "%hhu", &pattern);
	if (1 != ret) return ret;

	switch (pattern) {
		case 1: // for Red, blink every 4000ms
			arima_pwm_led_write(led, 1, 0xbc41, 1);
			arima_pwm_led_write(led, 1, 0xbc42, 6);
			arima_pwm_led_write(led, 1, 0xbc44, 1);
			arima_pwm_led_write(led, 1, 0xbc45, 0);
			arima_pwm_led_write(led, 1, 0xbc47, 1);
			arima_pwm_led_write(led, 0, 0xa146, LED_MPP_EN_ENABLE);
			arima_pwm_led_write(led, 0, 0xa346, LED_MPP_EN_DISABLE);
			break;
		case 2: // for Green, blink every 2000ms
			arima_pwm_led_write(led, 1, 0xbc41, 1);
			arima_pwm_led_write(led, 1, 0xbc42, 5);
			arima_pwm_led_write(led, 1, 0xbc44, 1);
			arima_pwm_led_write(led, 1, 0xbc45, 0);
			arima_pwm_led_write(led, 1, 0xbc47, 1);
			arima_pwm_led_write(led, 0, 0xa146, LED_MPP_EN_DISABLE);
			arima_pwm_led_write(led, 0, 0xa346, LED_MPP_EN_ENABLE);
			break;
		case 3: // for Other, blink every 1000ms
			arima_pwm_led_write(led, 1, 0xbc41, 1);
			arima_pwm_led_write(led, 1, 0xbc42, 4);
			arima_pwm_led_write(led, 1, 0xbc44, 1);
			arima_pwm_led_write(led, 1, 0xbc45, 0);
			arima_pwm_led_write(led, 1, 0xbc47, 1);
			arima_pwm_led_write(led, 0, 0xa146, LED_MPP_EN_ENABLE);
			arima_pwm_led_write(led, 0, 0xa346, LED_MPP_EN_ENABLE);
			break;
		case 4: // for red ON
			arima_pwm_led_write(led, 1, 0xbc41, 7);
			arima_pwm_led_write(led, 1, 0xbc42, 2);
			arima_pwm_led_write(led, 1, 0xbc44, 0xff);
			arima_pwm_led_write(led, 1, 0xbc45, 1);
			arima_pwm_led_write(led, 1, 0xbc47, 1);
			arima_pwm_led_write(led, 0, 0xa146, LED_MPP_EN_ENABLE);
			arima_pwm_led_write(led, 0, 0xa346, LED_MPP_EN_DISABLE);
			break;
		case 5: // for green ON
			arima_pwm_led_write(led, 1, 0xbc41, 7);
			arima_pwm_led_write(led, 1, 0xbc42, 2);
			arima_pwm_led_write(led, 1, 0xbc44, 0xff);
			arima_pwm_led_write(led, 1, 0xbc45, 1);
			arima_pwm_led_write(led, 1, 0xbc47, 1);
			arima_pwm_led_write(led, 0, 0xa146, LED_MPP_EN_DISABLE);
			arima_pwm_led_write(led, 0, 0xa346, LED_MPP_EN_ENABLE);
			break;
		case 6: // for orange ON
			arima_pwm_led_write(led, 1, 0xbc41, 7);
			arima_pwm_led_write(led, 1, 0xbc42, 2);
			arima_pwm_led_write(led, 1, 0xbc44, 0xff);
			arima_pwm_led_write(led, 1, 0xbc45, 1);
			arima_pwm_led_write(led, 1, 0xbc47, 1);
			arima_pwm_led_write(led, 0, 0xa146, LED_MPP_EN_ENABLE);
			arima_pwm_led_write(led, 0, 0xa346, LED_MPP_EN_ENABLE);
			break;
		default: // No blinks
			arima_pwm_led_write(led, 1, 0xbc41, 7);
			arima_pwm_led_write(led, 1, 0xbc42, 2);
			arima_pwm_led_write(led, 1, 0xbc44, 0xff);
			arima_pwm_led_write(led, 1, 0xbc45, 1);
			arima_pwm_led_write(led, 1, 0xbc47, 1);
			arima_pwm_led_write(led, 0, 0xa146, LED_MPP_EN_DISABLE);
			arima_pwm_led_write(led, 0, 0xa346, LED_MPP_EN_DISABLE);
			break;
	}

	CDBG("[%s] END \n", __func__);
	return count;
}
static DEVICE_ATTR(pattern, 0664, NULL, arima_pwm_pattern_store);
static struct attribute *pattern_attrs[] = {
	&dev_attr_pattern.attr,
	NULL
};
static const struct attribute_group pattern_attr_group = {
	.attrs = pattern_attrs,
};

static int arima_pwm_mpp_init(struct qpnp_led_data *led)
{
	int rc;
	u8 led_curr_setting = 0;
	CDBG("[%s] BEGIN \n", __func__);

	if (LED_GREEN_BASE == led->base) led_curr_setting = 1;
	else led_curr_setting = 0;
	CDBG("[%s] LED base=%x, curr_setting=%d \n", __func__, led->base, led_curr_setting);

	rc = arima_pwm_led_write(led, 0, LED_MPP_SINK_CTRL(led->base), led_curr_setting);
	if (rc) {
		dev_err(&led->spmi_dev->dev, "Failed to write sink control reg\n");
		return rc;
	}

	rc = arima_pwm_led_write(led, 0, LED_MPP_MODE_CTRL(led->base), LED_MOO_EN_CURR_SNK);
	if (rc) {
		dev_err(&led->spmi_dev->dev, "Failed to write led vin control reg\n");
		return rc;
	}

	CDBG("[%s] END \n", __func__);
	return 0;
}

u8 g_is_pwm_inited = 0;
static int arima_pwm_channel_init(struct qpnp_led_data *led)
{
	int rc = 0;
	CDBG("[%s] BEGIN \n", __func__);

	rc = arima_pwm_led_write(led, 1, 0xbc41, 0x03);
	if (rc) {
		dev_err(&led->spmi_dev->dev, "Failed to write sink control reg\n");
		return rc;
	}

	rc = arima_pwm_led_write(led, 1, 0xbc42, 0x03);
	if (rc) {
		dev_err(&led->spmi_dev->dev, "Failed to write sink control reg\n");
		return rc;
	}

	rc = arima_pwm_led_write(led, 1, 0xbc43, 0x20);
	if (rc) {
		dev_err(&led->spmi_dev->dev, "Failed to write sink control reg\n");
		return rc;
	}

	rc = arima_pwm_led_write(led, 1, 0xbc44, 0xFF);
	if (rc) {
		dev_err(&led->spmi_dev->dev, "Failed to write sink control reg\n");
		return rc;
	}

	rc = arima_pwm_led_write(led, 1, 0xbc45, 0x01);
	if (rc) {
		dev_err(&led->spmi_dev->dev, "Failed to write sink control reg\n");
		return rc;
	}

	rc = arima_pwm_led_write(led, 1, 0xbc46, 0x80);
	if (rc) {
		dev_err(&led->spmi_dev->dev, "Failed to write sink control reg\n");
		return rc;
	}

	rc = arima_pwm_led_write(led, 1, 0xbc47, 0x01);
	if (rc) {
		dev_err(&led->spmi_dev->dev, "Failed to write sink control reg\n");
		return rc;
	}

	g_is_pwm_inited = 1;
	CDBG("[%s] END rc=%d \n", __func__, rc);
	return rc;
}

static int arima_pwm_leds_probe(struct spmi_device *spmi)
{
	struct qpnp_led_data *led, *led_array;
	struct resource *led_resource;
	struct device_node *node, *temp;
	int rc, i, num_leds = 0, parsed_leds = 0;

	CDBG("[%s] BEGIN \n", __func__);

	node = spmi->dev.of_node;
	if (node == NULL)
		return -ENODEV;

	temp = NULL;
	while ((temp = of_get_next_child(node, temp))) num_leds++;

	if (!num_leds)
		return -ECHILD;

	led_array = devm_kzalloc(&spmi->dev,
		(sizeof(struct qpnp_led_data) * num_leds), GFP_KERNEL);
	if (!led_array) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->num_leds = num_leds;
		led->spmi_dev = spmi;

		led_resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
		if (!led_resource) {
			dev_err(&spmi->dev, "Unable to get LED base address\n");
			rc = -ENXIO;
			goto fail_id_check;
		}
		led->base = led_resource->start;

		rc = of_property_read_string(temp, "linux,name", &led->cdev.name);
		if (rc < 0) {
			dev_err(&led->spmi_dev->dev, "Failure reading led name, rc = %d\n", rc);
			goto fail_id_check;
		}

		led->cdev.brightness_set    = arima_pwm_led_set;
		led->cdev.brightness_get    = arima_pwm_led_get;

		mutex_init(&led->lock);

		INIT_WORK(&led->work, arima_pwm_led_work);

		rc = arima_pwm_mpp_init(led);
		if (rc < 0) goto fail_id_check;

		if(1 != g_is_pwm_inited)
		{
			rc = arima_pwm_channel_init(led);
			if (rc < 0) goto fail_id_check;
		}

		led->cdev.max_brightness = MPP_MAX_LEVEL;

		rc = led_classdev_register(&spmi->dev, &led->cdev);
		if (rc) {
			dev_err(&spmi->dev, "unable to register led %s,rc=%d\n", led->cdev.name, rc);
			goto fail_id_check;
		}

		rc = sysfs_create_group(&led->cdev.dev->kobj, &pattern_attr_group);
		if (rc) goto fail_id_check;

		parsed_leds++;
	}

	dev_set_drvdata(&spmi->dev, led_array);
	CDBG("[%s] END \n", __func__);
	return 0;

fail_id_check:
	for (i = 0; i < parsed_leds; i++) {
		led_classdev_unregister(&led_array[i].cdev);
	}

	CDBG("[%s] END_XX rc=%d \n", __func__, rc);
	return rc;
}

static int arima_pwm_leds_remove(struct spmi_device *spmi)
{
	struct qpnp_led_data *led_array  = dev_get_drvdata(&spmi->dev);
	int i, parsed_leds = led_array->num_leds;

	for (i = 0; i < parsed_leds; i++) {
		cancel_work_sync(&led_array[i].work);
		mutex_destroy(&led_array[i].lock);

		led_classdev_unregister(&led_array[i].cdev);

		sysfs_remove_group(&led_array[i].cdev.dev->kobj, &pattern_attr_group);
	}

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id spmi_match_table[] = {
	{ .compatible = LED_DEV_NAME,},
	{ },
};
#else
#define spmi_match_table NULL
#endif

static struct spmi_driver arima_pwm_leds_driver = {
	.driver		= {
		.name	= LED_DEV_NAME,
		.of_match_table = spmi_match_table,
	},
	.probe		= arima_pwm_leds_probe,
	.remove		= arima_pwm_leds_remove,
};

static int __init arima_pwm_led_init(void)
{
	return spmi_driver_register(&arima_pwm_leds_driver);
}
module_init(arima_pwm_led_init);

static void __exit arima_pwm_led_exit(void)
{
	spmi_driver_unregister(&arima_pwm_leds_driver);
}
module_exit(arima_pwm_led_exit);

MODULE_DESCRIPTION("QPNP LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-qpnp");

