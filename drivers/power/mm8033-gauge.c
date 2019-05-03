/* Copyright (c) 2013-2015 The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt) "MM8033:%s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/completion.h>
#include <linux/pm_wakeup.h>

#define TRY								5
#define SKIP							10
#define MM8033_ID					0x0110
#define SYSTEM_LOWVOLTAGE_LIMIT		3400

#define MM8033_STATUS				0x00
#define MM8033_RemCapacity			0x05
#define MM8033_SOC					0x06
#define MM8033_Voltage				0x09
#define MM8033_Current				0x0A
#define MM8033_AverageCurrent		0x0B
#define MM8033_IC_Temperature		0x0C
#define MM8033_FullChargeCapacity	0x10
#define MM8033_FullVoltageThr		0x13
#define MM8033_Temperature			0x16
#define MM8033_CycleCount			0x17
#define MM8033_DesignCapacity		0x18
#define MM8033_AverageVoltage		0x19
#define MM8033_Config				0x1D
#define MM8033_ChargeTerminalCurrent	0x1E
#define MM8033_FGCondition			0x20
#define MM8033_Identify				0x21
#define MM8033_BatteryImpedance		0x22
#define MM8033_BatteryCapacity		0x23
#define MM8033_Project_Name			0x2C
#define MM8033_Pack_Cell_Vendor		0x2D
#define MM8033_Manufacture_Date		0x2E
#define MM8033_FGSTAT				0x3D

#define Config_I2CSH	0x40

#define FGCondition_Active	0x00
#define FGCondition_LowPowerActive	0x01
#define FGCondition_Shutdown	0x02
#define FGCondition_OCV_measured_vol	0x20
#define FGCondition_OCV_average_vol	0x21
#define FGCondition_FG_Restart	0x24
#define FGCondition_System_Reset	0x80

struct mm8033_batparams {
	u8 params[512];
	u8 en_i2c_sh;
	int fullvoltage_thr;
	int designcapacity;
	int charge_terminal_current;
	int paramrevision;
	int batterycode;
	int default_cyclecount;
	int default_batterycapacity;
	int default_batteryimpedance;
};

struct mm8033_chip {
	struct i2c_client		*client;
	struct device			*dev;
	u16				identify;
	unsigned short			default_i2c_addr;

	/* configuration data - charger */
	int				fake_battery_soc;

	/* configuration data - fg */
	struct mm8033_batparams batparams;
	u8				fgstat;
	char			*batt_id;
	int				bat_year;	
	int				bat_week_1;
	int				bat_week_2;
	int				invalid_bat;
	u16				parameter_version;
	u16				battery_ctrlcode;
	int				cyclecount;
	int				batterycapacity;
	int				batteryimpedance;
	int				voltage_max_mv;
/*[Arima_5830][bozhi_lin] implement battery end of charging and re-charge  20160317 begin*/
	int				skipcheck;
	bool			power_on_reset;
/*[Arima_5830][bozhi_lin] 20160317 end*/

	/* status tracking */
	int				voltage_now;
	int				current_now;
	int				resistance_now;
	int				temp_now;
	int				soc_now;
	int				fcc_mah;
/*[Arima_5830][bozhi_lin] implement read battery state-of-health 20160310 begin*/
	int				fcc_design_mah;
/*[Arima_5830][bozhi_lin] 20160310 end*/
	bool				usb_present;
	bool				batt_present;
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	bool				batt_full;
	bool				resume_completed;

	u32				peek_poke_address;
	struct dentry			*debug_root;

	struct power_supply		*usb_psy;
	struct power_supply		*batt_psy;
	struct power_supply		batt_gauge_mm8033_psy;
	struct mutex			irq_complete;
	struct mutex			read_write_lock;
/*[Arima_5830][bozhi_lin] implement battery end of charging and re-charge  20160317 begin*/
	int				eoc_reported;
/*[Arima_5830][bozhi_lin] 20160317 end*/
/*[Arima_5830][bozhi_lin] gauge add hw varient to check can supoort ocv correction 20160408 begin*/
	int				pcba_id0;
	int				pcba_id1;
	int				pcba_id2;
	int				pcba_id3;
	bool			can_support_ocv_correct;
/*[Arima_5830][bozhi_lin] 20160408 end*/
};

/*[Arima_5830][bozhi_lin] implement battery end of charging and re-charge  20160317 begin*/
static int mm8033_checkRamData(struct mm8033_chip *chip);
/*[Arima_5830][bozhi_lin] 20160317 end*/

static struct mm8033_chip *the_chip;
static ssize_t mm8033_gauge_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

/*[Arima_5830][bozhi_lin] re-format battery version from Asus request 20160408 begin*/
#if 1
	sprintf(buf, "%s%04x%04x%d%d%d", the_chip->batt_id, the_chip->parameter_version, the_chip->battery_ctrlcode, the_chip->bat_year, the_chip->bat_week_1, the_chip->bat_week_2);
#else
	sprintf(buf, "MM8033_%s_0x%04x%04x_%dW%d%d", the_chip->batt_id, the_chip->battery_ctrlcode, the_chip->parameter_version, the_chip->bat_year, the_chip->bat_week_1, the_chip->bat_week_2);
#endif
/*[Arima_5830][bozhi_lin] 20160408 end*/

	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, mm8033_gauge_vendor_show, NULL);

static struct kobject *android_battery_kobj;

static int mm8033_gauge_sysfs_init(void)
{
	int ret ;

	android_battery_kobj = kobject_create_and_add("android_battery", NULL) ;
	if (android_battery_kobj == NULL) {
		pr_err("[B]%s(%d): subsystem_register failed\n", __func__, __LINE__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_battery_kobj, &dev_attr_vendor.attr);
	if (ret) {
		pr_err("[B]%s(%d): sysfs_create_group failed\n", __func__, __LINE__);
		return ret;
	}
	return 0 ;
}

static void mm8033_gauge_sysfs_deinit(void)
{
	sysfs_remove_file(android_battery_kobj, &dev_attr_vendor.attr);
	kobject_del(android_battery_kobj);
}

static int bound(int val, int min, int max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;

	return val;
}

static int mm8033_read_bytes(struct mm8033_chip *chip, int reg,
						u8 *val, u8 bytes)
{
	s32 rc;

	mutex_lock(&chip->read_write_lock);
	rc = i2c_smbus_read_i2c_block_data(chip->client, reg, bytes, val);
	if (rc < 0)
		dev_err(chip->dev,
			"i2c read fail: can't read %d bytes from %02x: %d\n",
							bytes, reg, rc);
	mutex_unlock(&chip->read_write_lock);

	return (rc < 0) ? rc : 0;
}

static int mm8033_write_bytes(struct mm8033_chip *chip, int reg,
						u8 *val, u8 bytes, u8 ms)
{
	s32 rc;

	mutex_lock(&chip->read_write_lock);
	rc = i2c_smbus_write_i2c_block_data(chip->client, reg, bytes, val);
	if (rc < 0)
		dev_err(chip->dev,
			"i2c write fail: can't read %d bytes from %02x: %d\n",
							bytes, reg, rc);
	msleep(ms);
	mutex_unlock(&chip->read_write_lock);

	return (rc < 0) ? rc : 0;
}

#if 0
static int mm8033_masked_write(struct mm8033_chip *chip, int reg,
						u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	mutex_lock(&chip->read_write_lock);
	rc = __mm8033_read(chip, reg, &temp);
	if (rc < 0) {
		dev_err(chip->dev, "read failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __mm8033_write(chip, reg, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"write failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}
#endif
static inline bool is_device_suspended(struct mm8033_chip *chip)
{
	return !chip->resume_completed;
}

static enum power_supply_property mm8033_battery_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
/*[Arima_5830][bozhi_lin] implement read battery state-of-health 20160310 begin*/
	POWER_SUPPLY_PROP_CHARGE_FULL,
/*[Arima_5830][bozhi_lin] 20160310 end*/
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_GAUGE_OCV_CORRECT,
};

static int mm8033_get_prop_batt_present(struct mm8033_chip *chip)
{
	return chip->batt_present;
}

/*[Arima_5833][bozhi_lin] gauge do OCV correction when power-on 20160318 begin*/
static int mm8033_ocv_correction(struct mm8033_chip *chip)
{
	u8 reg[2];
	int rc = 0;

/*[Arima_5830][bozhi_lin] gauge add hw varient to check can supoort ocv correction 20160408 begin*/
/*[Arima_5830][bozhi_lin] temp disable gauge do OCV correction 20160321 begin*/
	if (!chip->can_support_ocv_correct) {
		pr_info("[B]%s(%d): SR or ER1 not support mm8033_ocv_correction\n", __func__, __LINE__);
		return rc;
	}
/*[Arima_5830][bozhi_lin] 20160321 end*/
/*[Arima_5830][bozhi_lin] 20160408 end*/

	if (is_device_suspended(chip))
		return rc;

	pr_debug("[B]%s(%d): \n", __func__, __LINE__);

	reg[1] = 0x00;
	reg[0] = FGCondition_OCV_measured_vol & 0xFF;

	rc = mm8033_write_bytes(chip, MM8033_FGCondition, reg, 2, 100);
	if (rc) {
		pr_err("Couldn't write MM8033_FGCondition with FGCondition_OCV_measured_vol rc=%d\n", rc);
		return rc;	
	}

	return rc;
}
/*[Arima_5833][bozhi_lin] 20160318 end*/

static int mm8033_get_prop_cycle_count(struct mm8033_chip *chip)
{
	u8 reg[2];
	int rc, cycle_count = 0;

	if (is_device_suspended(chip))
		return chip->cyclecount;

	rc = mm8033_read_bytes(chip, MM8033_CycleCount, reg, 2);
	if (rc) {
		pr_err("Failed to read MM8033_CycleCount rc=%d\n", rc);
		return rc;
	}
	
	cycle_count = (reg[1] << 8) | reg[0];

	pr_debug("reg[1]=0x%02x reg[0]=0x%02x cycle_count=%d\n",
				reg[1], reg[0], cycle_count);

	chip->cyclecount = cycle_count;

	return chip->cyclecount;
}

static int mm8033_get_prop_voltage_max_design(struct mm8033_chip *chip)
{
	u8 reg[2];
	int rc, voltage_max_mv = 0;

	if (is_device_suspended(chip))
		return chip->voltage_max_mv;

	rc = mm8033_read_bytes(chip, MM8033_FullVoltageThr, reg, 2);
	if (rc) {
		pr_err("Failed to read MM8033_FullVoltageThr rc=%d\n", rc);
		return rc;
	}
	
	voltage_max_mv = (reg[1] << 8) | reg[0];

	pr_debug("reg[1]=0x%02x reg[0]=0x%02x cycle_count=%d\n",
				reg[1], reg[0], voltage_max_mv);

	chip->voltage_max_mv = voltage_max_mv * 1000;

	return chip->voltage_max_mv;
}

/*[Arima_5830][bozhi_lin] fine tune mm8033 gauge charging algorithm 20160408 begin*/
/*[Arima_5830][bozhi_lin] implement battery end of charging and re-charge  20160317 begin*/
#if 0
static void check_recharge_condition(struct mm8033_chip *chip)
{
	int rc;
	union power_supply_propval ret = {0,};

	if (chip->soc_now > 99)
		return;

	if (chip->batt_psy == NULL)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (chip->batt_psy) {
		rc = chip->batt_psy->get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_STATUS, &ret);
		if (rc) {
			pr_err("Unable to get battery 'STATUS' rc=%d\n", rc);
			/* Report recharge to charger for SOC based resume of charging */
		} else if ((ret.intval == POWER_SUPPLY_STATUS_FULL) && chip->eoc_reported) {
			ret.intval = POWER_SUPPLY_STATUS_CHARGING;
			rc = chip->batt_psy->set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_STATUS, &ret);
			if (rc < 0) {
				pr_err("Unable to set battery property rc=%d\n", rc);
			} else {
				pr_info("soc dropped below resume_soc soc=%d, restart charging\n", chip->soc_now);
				chip->eoc_reported = false;

				/*[Arima_5833][bozhi_lin] gauge do OCV correction when power-on 20160318 begin*/
				rc = mm8033_ocv_correction(chip);
				if (rc) {
					pr_err("Failed to mm8033_ocv_correction rc=%d\n", rc);
				}
				/*[Arima_5833][bozhi_lin] 20160318 end*/
			}
		} else if ((ret.intval == POWER_SUPPLY_STATUS_DISCHARGING) && chip->eoc_reported) {
			chip->eoc_reported = false;
			
			/*[Arima_5833][bozhi_lin] gauge do OCV correction when power-on 20160318 begin*/
			rc = mm8033_ocv_correction(chip);
			if (rc) {
				pr_err("Failed to mm8033_ocv_correction rc=%d\n", rc);
			}
			/*[Arima_5833][bozhi_lin] 20160318 end*/
		}
	}
}

static void check_eoc_condition(struct mm8033_chip *chip)
{
	int rc = -EINVAL;
	union power_supply_propval ret = {0,};

	if (chip->batt_psy == NULL)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (chip->batt_psy) {
		rc = chip->batt_psy->get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_STATUS, &ret);
		if (rc) {
			pr_err("Unable to get battery 'STATUS' rc=%d\n", rc);
		} else if (ret.intval == POWER_SUPPLY_STATUS_CHARGING) {
			pr_debug("Report EOC to charger\n");
			ret.intval = POWER_SUPPLY_STATUS_FULL;
			rc = chip->batt_psy->set_property(chip->batt_psy,
					POWER_SUPPLY_PROP_STATUS, &ret);
			if (rc) {
				pr_err("Unable to set 'STATUS' rc=%d\n", rc);
			}
			chip->eoc_reported = true;
		}
	} else {
		pr_err("battery psy not registered\n");
	}
}
#endif
/*[Arima_5830][bozhi_lin] 20160317 end*/
/*[Arima_5830][bozhi_lin] 20160408 end*/

/*[Arima_5830][bozhi_lin] fine tune mm8033 gauge charging algorithm 20160408 begin*/
static int mm8033_do_gauge_ocv_correct(struct mm8033_chip *chip)
{
	int rc;

	rc = mm8033_ocv_correction(chip);
	if (rc) {
		pr_err("Failed to mm8033_ocv_correction rc=%d\n", rc);
	}

	return rc;
}
/*[Arima_5830][bozhi_lin] 20160408 end*/

static int mm8033_get_prop_batt_capacity(struct mm8033_chip *chip)
{
	u8 reg[2];
	u32 temp = 0;
	int rc, soc = 0;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (is_device_suspended(chip))
		return chip->soc_now;

	rc = mm8033_read_bytes(chip, MM8033_SOC, reg, 2);
	if (rc) {
		pr_err("Failed to read MM8033_SOC rc=%d\n", rc);
		return rc;
	}
	pr_debug("Reading reg[1]=0x%02x, reg[0]=0x%02x\n", reg[1], reg[0]);
	
	soc = reg[1];

	temp = reg[0];
#if 0
	if (temp > (256 / 2))
		soc += 1;
#endif

	pr_debug("reg[1]=0x%x, reg[0]=0x%x, fg_soc=%d batt_full = %d\n", reg[1], reg[0],
						soc, chip->batt_full);

	chip->soc_now = (chip->batt_full ? 100 : bound(soc, 0, 100));

/*[Arima_5830][bozhi_lin] fine tune mm8033 gauge charging algorithm 20160408 begin*/
/*[Arima_5830][bozhi_lin] implement battery end of charging and re-charge  20160317 begin*/
#if 0
	if (chip->soc_now == 100) {
		check_eoc_condition(chip);
	}

	if (chip->eoc_reported) {
		check_recharge_condition(chip);
		//chip->soc_now = 100;
	}
#endif
/*[Arima_5830][bozhi_lin] 20160317 end*/
/*[Arima_5830][bozhi_lin] 20160408 end*/

	return chip->soc_now;
}

/*[Arima_5830][bozhi_lin] implement read battery state-of-health 20160310 begin*/
static int mm8033_get_prop_chg_full_design(struct mm8033_chip *chip)
{
/*[Arima_5830][bozhi_lin] always show battery capacity to 3000mA 20160719 begin*/
#if 1
	int fcc_design_mah = 3000;

	chip->fcc_design_mah = fcc_design_mah * 1000;

	return chip->fcc_design_mah;
#else
	u8 reg[2];
	int rc, fcc_design_mah = 0;

	if (is_device_suspended(chip))
		return chip->fcc_design_mah;

	rc = mm8033_read_bytes(chip, MM8033_DesignCapacity, reg, 2);
	if (rc) {
		pr_err("Failed to read MM8033_DesignCapacity rc=%d\n", rc);
		return rc;
	}
	fcc_design_mah = (reg[1] << 8) | reg[0];

	pr_debug("reg[1]=0x%02x reg[0]=0x%02x fcc_mah=%d\n",
				reg[1], reg[0], fcc_design_mah);

	chip->fcc_design_mah = fcc_design_mah * 1000;

	return chip->fcc_design_mah;
#endif
/*[Arima_5830][bozhi_lin] 20160719 end*/
}

static int mm8033_get_prop_chg_full(struct mm8033_chip *chip)
{
/*[Arima_5830][bozhi_lin] always show battery capacity to 3000mA 20160719 begin*/
#if 1
	int fcc_mah = 3000;

	chip->fcc_mah = fcc_mah * 1000;

	return chip->fcc_mah;
#else
	u8 reg[2];
	int rc, fcc_mah = 0;

	if (is_device_suspended(chip))
		return chip->fcc_mah;

	rc = mm8033_read_bytes(chip, MM8033_FullChargeCapacity, reg, 2);
	if (rc) {
		pr_err("Failed to read MM8033_FullChargeCapacity rc=%d\n", rc);
		return rc;
	}
	fcc_mah = (reg[1] << 8) | reg[0];

	pr_debug("reg[1]=0x%02x reg[0]=0x%02x fcc_mah=%d\n",
				reg[1], reg[0], fcc_mah);

	chip->fcc_mah = fcc_mah * 1000;

	return chip->fcc_mah;
#endif
/*[Arima_5830][bozhi_lin] 20160719 end*/
}
/*[Arima_5830][bozhi_lin] 20160310 end*/

static int mm8033_get_prop_batt_temp(struct mm8033_chip *chip)
{
	u8 reg[2];
	int rc, temp = 0;

	if (is_device_suspended(chip))
		return chip->temp_now;

	rc = mm8033_read_bytes(chip, MM8033_IC_Temperature, reg, 2);
	if (rc) {
		pr_err("Failed to read MM8033_IC_Temperature rc=%d\n", rc);
		return rc;
	}

	temp = (reg[1] << 8) | reg[0];

	pr_debug("reg[1]=0x%02x reg[0]=0x%02x temperature=%d\n",
					reg[1], reg[0], temp);

	if (temp > 32767) {
		temp -= 65536;
	}

	chip->temp_now = temp;

	return chip->temp_now;
}

static int mm8033_get_prop_voltage_now(struct mm8033_chip *chip)
{
	u8 reg[2];
	int rc, temp = 0;

	if (is_device_suspended(chip))
		return chip->voltage_now;

	rc = mm8033_read_bytes(chip, MM8033_Voltage, reg, 2);
	if (rc) {
		pr_err("Failed to read MM8033_Voltage rc=%d\n", rc);
		return rc;
	}

	temp = ((reg[1] &0x1F) << 8) | reg[0];

	pr_debug("reg[1]=0x%02x reg[0]=0x%02x voltage=%d\n",
				reg[1], reg[0], temp);

	chip->voltage_now = temp * 1000;

	return chip->voltage_now;
}

static int mm8033_get_prop_batt_resistance(struct mm8033_chip *chip)
{
	u8 reg[2];
	u16 temp;
	int rc;
	int64_t resistance;

	if (is_device_suspended(chip))
		return chip->resistance_now;

	rc = mm8033_read_bytes(chip, MM8033_BatteryImpedance, reg, 2);
	if (rc) {
		pr_err("Failed to read MM8033_BatteryImpedance rc=%d\n", rc);
		return rc;
	}
	temp = (reg[1] << 8) | reg[0];

	resistance = temp;

	pr_debug("reg=0x%02x resistance=%lld\n", temp, resistance);

	/* resistance in uohms */
	chip->resistance_now = resistance;

	return chip->resistance_now;
}

static int mm8033_get_prop_current_now(struct mm8033_chip *chip)
{
	u8 reg[2];
	int rc, temp = 0;

	if (is_device_suspended(chip))
		return chip->current_now;

	rc = mm8033_read_bytes(chip, MM8033_Current, reg, 2);
	if (rc) {
		pr_err("Failed to read MM8033_Current rc=%d\n", rc);
		return rc;
	}

	temp = (reg[1] << 8) | reg[0];

	pr_debug("reg[1]=0x%02x reg[0]=0x%02x current=%d\n",
				reg[1], reg[0], temp);

	if (temp > 32767) {
		temp -= 65536;
	}

	chip->current_now = temp * 1000;

	return chip->current_now;
}


static int mm8033_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct mm8033_chip *chip = container_of(psy,
				struct mm8033_chip, batt_gauge_mm8033_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		pr_info("fake_soc set to %d\n", chip->fake_battery_soc);
		power_supply_changed(&chip->batt_gauge_mm8033_psy);
		break;
/*[Arima_5830][bozhi_lin] fine tune mm8033 gauge charging algorithm 20160711 begin*/
	case POWER_SUPPLY_PROP_GAUGE_OCV_CORRECT:
		mm8033_do_gauge_ocv_correct(chip);
		break;
/*[Arima_5830][bozhi_lin] 20160711 end*/
	default:
		return -EINVAL;
	}

	return 0;
}

static int mm8033_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int mm8033_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct mm8033_chip *chip = container_of(psy,
				struct mm8033_chip, batt_gauge_mm8033_psy);
/*[Arima_5830][bozhi_lin] implement battery end of charging and re-charge  20160317 begin*/
	int ret = 0;

	if (chip->skipcheck > SKIP) {
		chip->skipcheck = 0;
		ret = mm8033_checkRamData(chip);
	} else {
		chip->skipcheck++;
	}
/*[Arima_5830][bozhi_lin] 20160317 end*/

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = mm8033_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = mm8033_get_prop_cycle_count(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = mm8033_get_prop_voltage_max_design(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = mm8033_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = mm8033_get_prop_chg_full_design(chip);
		break;
/*[Arima_5830][bozhi_lin] implement read battery state-of-health 20160310 begin*/
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = mm8033_get_prop_chg_full(chip);
		break;
/*[Arima_5830][bozhi_lin] 20160310 end*/
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = mm8033_get_prop_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = mm8033_get_prop_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		val->intval = mm8033_get_prop_batt_resistance(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = mm8033_get_prop_batt_temp(chip);
		break;
/*[Arima_5830][bozhi_lin] fine tune mm8033 gauge charging algorithm 20160711 begin*/
/*[Arima_5830][bozhi_lin] fine tune mm8033 gauge charging algorithm 20160408 begin*/
	case POWER_SUPPLY_PROP_GAUGE_OCV_CORRECT:
		val->intval = 0;
		break;
/*[Arima_5830][bozhi_lin] 20160408 end*/
/*[Arima_5830][bozhi_lin] 20160711 end*/
	default:
		return -EINVAL;
	}
	return 0;
}

static int get_reg(void *data, u64 *val)
{
	struct mm8033_chip *chip = data;
	int rc;
	u8 temp[2];

	rc = mm8033_read_bytes(chip, chip->peek_poke_address, temp, 2);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = (temp[1] << 8) | temp[0];
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct mm8033_chip *chip = data;
	int rc;
	u8 temp[2];

	temp[1] = ((u8) (val & 0xFF00)) >> 8;
	temp[0] = (u8) (val & 0xFF);
	rc = mm8033_write_bytes(chip, chip->peek_poke_address, temp, 2, 100);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x 0x%02x rc= %d\n",
			chip->peek_poke_address, temp[1], temp[0], rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

#define LAST_CNFG_REG	0x3F
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct mm8033_chip *chip = m->private;
	int rc;
	u8 reg[2];
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = mm8033_read_bytes(chip, addr, reg, 2);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x 0x%02x\n", addr, reg[1], reg[0]);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct mm8033_chip *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x80
#define LAST_STATUS_REG		0xBF
static int show_fgpara_regs(struct seq_file *m, void *data)
{
	struct mm8033_chip *chip = m->private;
	int rc;
	u8 reg[8];
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = mm8033_read_bytes(chip, addr, reg, 8);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", 
				addr, reg[7], reg[6], reg[5], reg[4], reg[3], reg[2], reg[1], reg[0]);
	}

	return 0;
}

static int fgpara_debugfs_open(struct inode *inode, struct file *file)
{
	struct mm8033_chip *chip = inode->i_private;

	return single_open(file, show_fgpara_regs, chip);
}

static const struct file_operations fgpara_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= fgpara_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static bool is_charger_present(struct mm8033_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->usb_psy == NULL)
		chip->usb_psy = power_supply_get_by_name("usb");
	if (chip->usb_psy) {
		chip->usb_psy->get_property(chip->usb_psy,
					POWER_SUPPLY_PROP_ONLINE, &ret);
		return ret.intval;
	}

	return false;
}

static int determine_initial_status(struct mm8033_chip *chip)
{
/*[Arima_5830][bozhi_lin] gauge add hw varient to check can supoort ocv correction 20160408 begin*/
	int err = 0;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}
/*[Arima_5830][bozhi_lin] 20160408 end*/

	chip->batt_present = true;

	chip->usb_present = is_charger_present(chip);
	
/*[Arima_5830][bozhi_lin] gauge add hw varient to check can supoort ocv correction 20160408 begin*/
	chip->pcba_id0 = of_get_named_gpio(node, "mm8033,pcba-id0", 0);

	chip->pcba_id1 = of_get_named_gpio(node, "mm8033,pcba-id1", 0);

	chip->pcba_id2 = of_get_named_gpio(node, "mm8033,pcba-id2", 0);

	chip->pcba_id3 = of_get_named_gpio(node, "mm8033,pcba-id3", 0);

	if (gpio_is_valid(chip->pcba_id0)) {
		err = gpio_request(chip->pcba_id0, "pcba_id0");
		if (err) {
			dev_err(chip->dev, "unable to request gpio [%d]\n", chip->pcba_id0);
		}

		err = gpio_direction_input(chip->pcba_id0);
		if (err) {
			dev_err(chip->dev, "unable to set direction for gpio [%d]\n", chip->pcba_id0);
		}
	} else {
		dev_err(chip->dev, "gpio is not valid for gpio [%d]\n", chip->pcba_id0);
	}

	if (gpio_is_valid(chip->pcba_id1)) {
		err = gpio_request(chip->pcba_id1, "pcba_id1");
		if (err) {
			dev_err(chip->dev, "unable to request gpio [%d]\n", chip->pcba_id1);
		}

		err = gpio_direction_input(chip->pcba_id1);
		if (err) {
			dev_err(chip->dev, "unable to set direction for gpio [%d]\n", chip->pcba_id1);
		}
	} else {
		dev_err(chip->dev, "gpio is not valid for gpio [%d]\n", chip->pcba_id1);
	}

	if (gpio_is_valid(chip->pcba_id2)) {
		err = gpio_request(chip->pcba_id2, "pcba_id2");
		if (err) {
			dev_err(chip->dev, "unable to request gpio [%d]\n", chip->pcba_id2);
		}

		err = gpio_direction_input(chip->pcba_id2);
		if (err) {
			dev_err(chip->dev, "unable to set direction for gpio [%d]\n", chip->pcba_id2);
		}
	} else {
		dev_err(chip->dev, "gpio is not valid for gpio [%d]\n", chip->pcba_id2);
	}
	
	if (gpio_is_valid(chip->pcba_id3)) {
		err = gpio_request(chip->pcba_id3, "pcba_id3");
		if (err) {
			dev_err(chip->dev, "unable to request gpio [%d]\n", chip->pcba_id3);
		}

		err = gpio_direction_input(chip->pcba_id3);
		if (err) {
			dev_err(chip->dev, "unable to set direction for gpio [%d]\n", chip->pcba_id3);
		}
	} else {
		dev_err(chip->dev, "gpio is not valid for gpio [%d]\n", chip->pcba_id3);
	}

	if (!gpio_get_value(chip->pcba_id0) && !gpio_get_value(chip->pcba_id1) && !gpio_get_value(chip->pcba_id2) && !gpio_get_value(chip->pcba_id3)) {
		pr_info("[B]%s(%d): SR\n", __func__, __LINE__);
		chip->can_support_ocv_correct = false;
	} else if (!gpio_get_value(chip->pcba_id0) && !gpio_get_value(chip->pcba_id1) && !gpio_get_value(chip->pcba_id2) && gpio_get_value(chip->pcba_id3)) {
		pr_info("[B]%s(%d): ER1\n", __func__, __LINE__);
		chip->can_support_ocv_correct = false;
	} else if (!gpio_get_value(chip->pcba_id0) && !gpio_get_value(chip->pcba_id1) && gpio_get_value(chip->pcba_id2) && !gpio_get_value(chip->pcba_id3)) {
		pr_info("[B]%s(%d): ER2\n", __func__, __LINE__);
		chip->can_support_ocv_correct = true;
	} else if (!gpio_get_value(chip->pcba_id0) && !gpio_get_value(chip->pcba_id1) && gpio_get_value(chip->pcba_id2) && gpio_get_value(chip->pcba_id3)) {
		pr_info("[B]%s(%d): PR\n", __func__, __LINE__);
		chip->can_support_ocv_correct = true;
	} else if (!gpio_get_value(chip->pcba_id0) && gpio_get_value(chip->pcba_id1) && !gpio_get_value(chip->pcba_id2) && !gpio_get_value(chip->pcba_id3)) {
		pr_info("[B]%s(%d): RESERVE1\n", __func__, __LINE__);
		chip->can_support_ocv_correct = true;
	} else if (!gpio_get_value(chip->pcba_id0) && gpio_get_value(chip->pcba_id1) && !gpio_get_value(chip->pcba_id2) && gpio_get_value(chip->pcba_id3)) {
		pr_info("[B]%s(%d): RESERVE2\n", __func__, __LINE__);
		chip->can_support_ocv_correct = true;
	} else {
		pr_info("[B]%s(%d): Not support\n", __func__, __LINE__);
		chip->can_support_ocv_correct = true;
	}

	pr_info("[B]%s(%d): pcba_id0[GPIO_100]=%d, pcba_id1[GPIO_101]=%d, pcba_id2[GPIO_102]=%d, pcba_id2[GPIO_95]=%d\n", 
			__func__, __LINE__, gpio_get_value(chip->pcba_id0), gpio_get_value(chip->pcba_id1), gpio_get_value(chip->pcba_id2), gpio_get_value(chip->pcba_id3));

	pr_info("[B]%s(%d): can_support_ocv_correct=%d\n",__func__, __LINE__, chip->can_support_ocv_correct);

	if (gpio_is_valid(chip->pcba_id0))
		gpio_free(chip->pcba_id0);

	if (gpio_is_valid(chip->pcba_id1))
		gpio_free(chip->pcba_id1);

	if (gpio_is_valid(chip->pcba_id2))
		gpio_free(chip->pcba_id2);

	if (gpio_is_valid(chip->pcba_id3))
		gpio_free(chip->pcba_id3);
/*[Arima_5830][bozhi_lin] 20160408 end*/

	return 0;
}


static int mm8033_setFgParameter(struct mm8033_chip *chip)
{
	int i, j;
	int ret;
//	int val;
	u8 buf[8];
	u8 reg[2];

	for (i = 0; i < 0x40; i++) {
		if ((i != 0x9) && (i != 0xa)) {
			for (j = 0; j < 8; j++) {
				buf[j] = chip->batparams.params[i*8 + j];
			}
			
			ret = mm8033_write_bytes(chip, 0x80 + i, buf, 8, 5);
			if (ret) {
				pr_err("Couldn't write 0x%x ret=%d\n", 0x80 + i, ret);
				return ret;
			}
			
		}
	}

	if (chip->batparams.en_i2c_sh) {
		reg[1] = 0x00;
		reg[0] = Config_I2CSH & 0xFF;
		ret = mm8033_write_bytes(chip, MM8033_Config, reg, 2, 5);
		if (ret) {
			pr_err("Couldn't write MM8033_Config rc=%d\n", ret);
			return ret;	
		}	
	}

	reg[1] = (u8) ((chip->batparams.fullvoltage_thr & 0x1F00) >> 8);
	reg[0] = (u8) (chip->batparams.fullvoltage_thr & 0xFF);
	ret = mm8033_write_bytes(chip, MM8033_FullVoltageThr, reg, 2, 5);
	if (ret) {
		pr_err("Couldn't write MM8033_FullVoltageThr rc=%d\n", ret);
		return ret;	
	}	

	reg[1] = (u8) ((chip->batparams.designcapacity & 0xFF00) >> 8);
	reg[0] = (u8) (chip->batparams.designcapacity & 0xFF);
	ret = mm8033_write_bytes(chip, MM8033_DesignCapacity, reg, 2, 5);
	if (ret) {
		pr_err("Couldn't write MM8033_DesignCapacity rc=%d\n", ret);
		return ret;	
	}	

	reg[1] = 0x00;
	reg[0] = (u8) (chip->batparams.charge_terminal_current & 0xFF);
	ret = mm8033_write_bytes(chip, MM8033_ChargeTerminalCurrent, reg, 2, 5);
	if (ret) {
		pr_err("Couldn't write MM8033_ChargeTerminalCurrent rc=%d\n", ret);
		return ret;	
	}	

	reg[1] = (u8) ((chip->batteryimpedance & 0xFF00) >> 8);
	reg[0] = (u8) (chip->batteryimpedance & 0xFF);
	ret = mm8033_write_bytes(chip, MM8033_BatteryImpedance, reg, 2, 5);
	if (ret) {
		pr_err("Couldn't write MM8033_BatteryImpedance rc=%d\n", ret);
		return ret;	
	}	

	reg[1] = (u8) ((chip->batterycapacity & 0xFF00) >> 8);
	reg[0] = (u8) (chip->batterycapacity & 0xFF);
	ret = mm8033_write_bytes(chip, MM8033_BatteryCapacity, reg, 2, 5);
	if (ret) {
		pr_err("Couldn't write MM8033_BatteryCapacity rc=%d\n", ret);
		return ret;	
	}	

	reg[1] = (u8) ((chip->cyclecount & 0xFF00) >> 8);
	reg[0] = (u8) (chip->cyclecount & 0xFF);
	ret = mm8033_write_bytes(chip, MM8033_CycleCount, reg, 2, 5);
	if (ret) {
		pr_err("Couldn't write MM8033_CycleCount rc=%d\n", ret);
		return ret;	
	}	


	reg[1] = 0x00;
	reg[0] = FGCondition_FG_Restart & 0xFF;
	ret = mm8033_write_bytes(chip, MM8033_FGCondition, reg, 2, 100);
	if (ret) {
		pr_err("Couldn't write MM8033_FGCondition rc=%d\n", ret);
		return ret;	
	}	
	// wait 50ms

	do {
		ret = mm8033_read_bytes(chip, MM8033_FGSTAT, reg, 2);
		if (ret) {
			pr_err("Couldn't read MM8033_FGSTAT ret=%d", ret);
			return ret;
		}

		chip->fgstat = (reg[1] << 8) | reg[0];
		
		if ((chip->fgstat) & 0x0001) {
			pr_err("MM8033 Data not ready!, chip->fgstat=0x%x\n", chip->fgstat);
		}
	} while ((chip->fgstat) & 0x0001);

/*[Arima_5830][bozhi_lin] implement battery end of charging and re-charge  20160317 begin*/
	if (chip->power_on_reset) {
		reg[1] = 0x00;
		reg[0] = 0x02;
		ret = mm8033_write_bytes(chip, MM8033_STATUS, reg, 2, 5);
		if (ret) {
			pr_err("Couldn't write MM8033_STATUS rc=%d\n", ret);
			return ret;	
		}
		chip->power_on_reset = false;
	}
/*[Arima_5830][bozhi_lin] 20160317 end*/

	return 0;
}

static int mm8033_checkRamData(struct mm8033_chip *chip)
{
	int i, ret, val;
	u8 reg[2];
	u8 buf[8];

	pr_debug("[B]%s(%d): +++\n", __func__, __LINE__);

/*[Arima_5830][bozhi_lin] implement battery end of charging and re-charge  20160317 begin*/
	ret = mm8033_read_bytes(chip, MM8033_STATUS, reg, 2);
	if (ret) {
		pr_err("Failed to read MM8033_STATUS rc=%d\n", ret);
		return ret;
	}
	
	if (reg[0] & 0x02) {
		pr_info("[B]%s(%d): POR! goto SETPARAMETER\n", __func__, __LINE__);
		chip->power_on_reset = true;
		goto SETPARAMETER;
	} else {
		ret = mm8033_read_bytes(chip, 0xBE, buf, 8);
		if (ret) {
			pr_err("Failed to read 0xBE rc=%d\n", ret);
			return ret;
		}

		chip->parameter_version = ((buf[7] & 0xff) << 8) | (buf[6] & 0xff);
		chip->battery_ctrlcode = ((buf[5] & 0xff) << 8) | (buf[4] & 0xff);

		if ((chip->parameter_version < chip->batparams.paramrevision)
		|| (chip->battery_ctrlcode != chip->batparams.batterycode)) {
			pr_info("[B]%s(%d): chip->parameter_version=0x%04x, chip->battery_ctrlcode=0x%04x\n", __func__, __LINE__, chip->parameter_version, chip->battery_ctrlcode);
			pr_info("[B]%s(%d): batparams.paramrevision=0x%04x, batparams.batterycode =0x%04x\n", __func__, __LINE__, chip->batparams.paramrevision, chip->batparams.batterycode);
			goto SETPARAMETER;
		} else {
			ret = mm8033_read_bytes(chip, MM8033_DesignCapacity, reg, 2);
			if (ret) {
				pr_err("Failed to read MM8033_DesignCapacity rc=%d\n", ret);
				return ret;
			}
		
			val = (reg[1] << 8) | reg[0];
		
			if (val != chip->batparams.designcapacity) {
				pr_info("[B]%s(%d): capacity = %d, chip->batparams.designcapacity=%d, set parameter\n", __func__, __LINE__, val, chip->batparams.designcapacity);
				goto SETPARAMETER;
			} else {
				goto GETPARAMETER;
			}
		}
	}
/*[Arima_5830][bozhi_lin] 20160317 end*/

GETPARAMETER:
	pr_debug("[B]%s(%d): GETPARAMETER\n", __func__, __LINE__);
	ret = mm8033_read_bytes(chip, MM8033_CycleCount, reg, 2);
	if (ret) {
		pr_err("Failed to read MM8033_CycleCount rc=%d\n", ret);
		return ret;
	}
	
	chip->cyclecount = (reg[1] << 8) | reg[0];

	ret = mm8033_read_bytes(chip, MM8033_BatteryImpedance, reg, 2);
	if (ret) {
		pr_err("Failed to read MM8033_BatteryImpedance rc=%d\n", ret);
		return ret;
	}

	chip->batteryimpedance = (reg[1] << 8) | reg[0];

	ret = mm8033_read_bytes(chip, MM8033_BatteryCapacity, reg, 2);
	if (ret) {
		pr_err("Failed to read MM8033_BatteryCapacity rc=%d\n", ret);
		return ret;
	}

	chip->batterycapacity = (reg[1] << 8) | reg[0];

	goto EXIT;

SETPARAMETER:
	pr_debug("[B]%s(%d): SETPARAMETER\n", __func__, __LINE__);

	for (i = 0; i < TRY; i++) {

		ret = mm8033_setFgParameter(chip);
		if (ret) return ret;

		ret = mm8033_read_bytes(chip, 0xBE, buf, 8);
		if (ret) {
			pr_err("Failed to read 0xBE rc=%d\n", ret);
			return ret;
		}

		chip->parameter_version = ((buf[7] & 0xff) << 8) | (buf[6] & 0xff);
		chip->battery_ctrlcode = ((buf[5] & 0xff) << 8) | (buf[4] & 0xff);

		/* need to check after */
		if (chip->parameter_version != chip->batparams.paramrevision) continue;
		if (chip->battery_ctrlcode != chip->batparams.batterycode) continue;

		ret = mm8033_read_bytes(chip, MM8033_CycleCount, reg, 2);
		if (ret) {
			pr_err("Failed to read MM8033_CycleCount rc=%d\n", ret);
			return ret;
		}
	
		chip->cyclecount = (reg[1] << 8) | reg[0];
		
		if (chip->cyclecount != chip->batparams.default_cyclecount) continue;

		ret = mm8033_read_bytes(chip, MM8033_DesignCapacity, reg, 2);
		if (ret) {
			pr_err("Failed to read MM8033_DesignCapacity rc=%d\n", ret);
			return ret;
		}
		
		val = (reg[1] << 8) | reg[0];

		if (val != chip->batparams.designcapacity) continue;

		break;
	}
	if (i >= TRY) {
		pr_info("[B]%s(%d): ---, i=%d\n", __func__, __LINE__, i);		
		return -1;
	}

EXIT:
	pr_debug("[B]%s(%d): ---\n", __func__, __LINE__);
	return 0;
}

static int mm8033_checkdevice(struct mm8033_chip *chip)
{
	int i;
	int val;
	int ret;
	u8 projectname_low;
	u8 projectname_high;
	u8 packvendor;
	u8 cellvendor;
	u8 date_low;
	u8 date_high;
//	u8 buf[8];
	u8 reg[2];

	struct mm8033_batparams *batparams;
	// NVT ATL battery parameter
	struct mm8033_batparams params_nvt = {
	{
		0xAB, 0x17, 0x94, 0x1E, 0xED, 0x2A, 0xE4, 0x3F,
		0xA8, 0x5D, 0xE3, 0x5D, 0x22, 0x70, 0x6A, 0x78,
		0x5B, 0x11, 0x42, 0xC2, 0x0D, 0x45, 0xF6, 0x0A,
		0xDA, 0xE4, 0x48, 0x16, 0x48, 0x07, 0x40, 0xF4,
		0x2B, 0x06, 0x46, 0x05, 0x3D, 0xFA, 0xB4, 0x01,
		0x86, 0x03, 0xBD, 0xFD, 0xF3, 0xFF, 0x3F, 0x08,
		0x49, 0xF7, 0x27, 0x02, 0x60, 0xFD, 0x1C, 0x06,
		0x1A, 0xFD, 0xFE, 0xDE, 0xCA, 0x28, 0x34, 0xF3,
		0x00, 0x0E, 0x0D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x14, 0x2F, 0x01, 0x66, 0x30, 0x00, 0x72, 0x06,
		0x54, 0x0B, 0x48, 0x0D, 0x30, 0x11, 0x32, 0x41,
		0x1E, 0x05, 0x3C, 0x2D, 0x78, 0x01, 0x00, 0x01,
		0xB1, 0x0C, 0xED, 0x0C, 0x25, 0x0D, 0x97, 0x0D,
		0x1E, 0x0E, 0x41, 0x0E, 0x60, 0x0E, 0x6F, 0x0E,
		0x7A, 0x0E, 0x93, 0x0E, 0xB1, 0x0E, 0xC9, 0x0E,
		0xE3, 0x0E, 0x0A, 0x0F, 0x34, 0x0F, 0x71, 0x0F,
		0xAD, 0x0F, 0xFA, 0x0F, 0x72, 0x10, 0x1A, 0x11,
		0x00, 0x00, 0x05, 0x00, 0x0A, 0x00, 0x14, 0x00,
		0x26, 0x00, 0x32, 0x00, 0x4B, 0x00, 0x67, 0x00,
		0x84, 0x00, 0xB4, 0x00, 0xFA, 0x00, 0x36, 0x01,
		0x90, 0x01, 0xEA, 0x01, 0x2B, 0x02, 0x62, 0x02,
		0xA8, 0x02, 0xF8, 0x02, 0x66, 0x03, 0xFC, 0x03,
		0x00, 0xE1, 0x00, 0x4D, 0xE1, 0x00, 0xA6, 0xF0,
		0x00, 0xFF, 0xD9, 0x00, 0x0D, 0xF6, 0x00, 0x39,
		0xCB, 0x00, 0x5A, 0xF6, 0x00, 0x80, 0x3C, 0x00,
		0x14, 0x0A, 0x05, 0x0A, 0x5C, 0xFF, 0xE4, 0x00,
		0x9A, 0x0B, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x07, 0x08, 0xFF, 0x28, 0x00, 0x28, 0x00,
		0x2D, 0x00, 0x46, 0x00, 0x00, 0x07, 0x08, 0xFF,
		0xC8, 0x00, 0xC8, 0x00, 0x6E, 0x00, 0x82, 0x00,
		0x00, 0x07, 0x08, 0xFF, 0xFE, 0x01, 0xFE, 0x01,
		0xC8, 0x00, 0x18, 0x01, 0x00, 0x07, 0x08, 0x78,
		0xFF, 0xFF, 0x9E, 0x02, 0x9E, 0x02, 0x2C, 0x01,
		0x2C, 0x01, 0xA4, 0x01, 0xFF, 0xFF, 0x00, 0x07,
		0x08, 0xE6, 0xFF, 0xFF, 0x1A, 0x04, 0x1A, 0x04,
		0xF0, 0x00, 0x94, 0x02, 0x94, 0x02, 0xFF, 0xFF,
		0x56, 0x0E, 0x0A, 0x2C, 0xD8, 0x0E, 0x0A, 0x20,
		0xE6, 0x0C, 0x0C, 0xFE, 0x3A, 0xFC, 0x68, 0x03,
		0x82, 0x00, 0xA8, 0x94, 0x0F, 0x66, 0xFA, 0x1E,
		0xF4, 0x01, 0xE4, 0x00, 0x02, 0x02, 0x32, 0x0A,
		0xA4, 0x01, 0x1A, 0x01, 0xA0, 0x00, 0xFF, 0xFF,
		0xAA, 0x02, 0x35, 0x01, 0x20, 0x03, 0x60, 0x62,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x80, 0x9C, 0xCE, 0xFF, 0x01, 0x32, 0x64, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x80, 0xFA, 0xFD, 0xFF, 0x01, 0x03, 0x06, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x00, 0x00, 0x14, 0x14, 0x14, 0x05, 0x00, 0x00,
		0x06, 0x00, 0x19, 0x10, 0x17, 0x17, 0x3E, 0x17,
		0x17, 0x02, 0x1C, 0x03, 0x0D, 0x32, 0x0A, 0xFF,
		0x0A, 0x12, 0x66, 0xFF, 0x02, 0x05, 0x05, 0x01,
		0x70, 0xC2, 0x0A, 0x00, 0x00, 0x00, 0x03, 0x99,
		0x99, 0x96, 0xFF, 0xCC, 0x4B, 0x0F, 0x71, 0xC1,
		0x62, 0x05, 0x14, 0x0A, 0x2C, 0x01, 0x5F, 0x05,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
		0x7A, 0x0D, 0x48, 0x0D, 0x03, 0x00, 0x0A, 0x0A,
		0x64, 0x00, 0x54, 0x0B, 0x64, 0x02, 0x02, 0x02,
		0x7D, 0x73, 0xFA, 0xE6, 0xBC, 0xB7, 0x6C, 0x71,
		0x4B, 0x78, 0x82, 0x09, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x30, 0x74, 0x0F, 0xFF, 0x03, 0x00, 0x06, 0x01,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	},
	0,      // enable i2c_sh
	4350,   // full voltage threshold
	2970,   // design capacity
	130,    // charge terminal current
	0x0106, // parameter revision
	0x0003, // battery code
	0,      // default cycle count
	2970,   // default battery capacity
	130     // default battery impedance
	};

	// Simplo Coslight battery parameter
	struct mm8033_batparams params_simplo = {
	{
		0xEB, 0x17, 0xC0, 0x1F, 0x35, 0x2C, 0xBE, 0x3F,
		0xB0, 0x42, 0x36, 0x60, 0xB6, 0x71, 0x41, 0x79,
		0x73, 0x11, 0xB7, 0xC1, 0xC8, 0x45, 0x53, 0x0A,
		0xD8, 0xE7, 0xC5, 0x12, 0x14, 0x07, 0xF0, 0xF4,
		0x93, 0x05, 0x2D, 0x05, 0x72, 0xFA, 0x96, 0x01,
		0xB4, 0x04, 0x65, 0xFB, 0x1C, 0x01, 0x56, 0x03,
		0x05, 0xFE, 0xDA, 0xFF, 0x48, 0xFD, 0x12, 0x06,
		0x2C, 0xFD, 0xA4, 0xD1, 0x33, 0x37, 0x59, 0xEF,
		0x00, 0x0E, 0x0D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x14, 0x2F, 0x01, 0x66, 0x30, 0x00, 0x72, 0x06,
		0x54, 0x0B, 0x48, 0x0D, 0x30, 0x11, 0x32, 0x41,
		0x1E, 0x05, 0x3C, 0x2D, 0x78, 0x01, 0x00, 0x01,
		0xB8, 0x0C, 0xFC, 0x0C, 0x45, 0x0D, 0xAF, 0x0D,
		0x0F, 0x0E, 0x3C, 0x0E, 0x63, 0x0E, 0x70, 0x0E,
		0x7A, 0x0E, 0x91, 0x0E, 0xAF, 0x0E, 0xC7, 0x0E,
		0xE3, 0x0E, 0x08, 0x0F, 0x31, 0x0F, 0x6C, 0x0F,
		0xA9, 0x0F, 0xF9, 0x0F, 0x6E, 0x10, 0x15, 0x11,
		0x00, 0x00, 0x05, 0x00, 0x0A, 0x00, 0x14, 0x00,
		0x26, 0x00, 0x37, 0x00, 0x4D, 0x00, 0x67, 0x00,
		0x84, 0x00, 0xB4, 0x00, 0xFA, 0x00, 0x36, 0x01,
		0x90, 0x01, 0xEA, 0x01, 0x2B, 0x02, 0x62, 0x02,
		0xA8, 0x02, 0xF8, 0x02, 0x66, 0x03, 0xFC, 0x03,
		0x00, 0xE1, 0x00, 0x4D, 0xE1, 0x00, 0xA6, 0xF0,
		0x00, 0xFF, 0xD4, 0x00, 0x0D, 0xF6, 0x00, 0x39,
		0xCB, 0x00, 0x5A, 0xF6, 0x00, 0x80, 0x3C, 0x00,
		0x14, 0x0A, 0x05, 0x0A, 0x5C, 0xFF, 0xE4, 0x00,
		0x54, 0x0B, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x07, 0x08, 0xFF, 0x96, 0x00, 0x96, 0x00,
		0x58, 0x00, 0x5A, 0x00, 0x00, 0x07, 0x08, 0xFF,
		0xDC, 0x00, 0xDC, 0x00, 0x69, 0x00, 0x8C, 0x00,
		0x00, 0x07, 0x08, 0xFF, 0x1C, 0x02, 0x1C, 0x02,
		0x09, 0x01, 0x5E, 0x01, 0x00, 0x07, 0x08, 0x6E,
		0xFF, 0xFF, 0x80, 0x02, 0x80, 0x02, 0x7C, 0x01,
		0x68, 0x01, 0xF4, 0x01, 0xFF, 0xFF, 0x00, 0x07,
		0x08, 0x78, 0xFF, 0xFF, 0x84, 0x03, 0x84, 0x03,
		0x58, 0x02, 0xE0, 0x01, 0x8A, 0x02, 0xFF, 0xFF,
		0x42, 0x0E, 0x0A, 0x1E, 0xEC, 0x0E, 0x0A, 0x32,
		0xE6, 0x0C, 0xD4, 0xFE, 0x57, 0xFC, 0x92, 0x03,
		0x8C, 0x00, 0xA8, 0x94, 0x0F, 0x66, 0xFA, 0x1E,
		0xF4, 0x01, 0xD4, 0x00, 0x02, 0x02, 0x32, 0x0A,
		0x6D, 0x01, 0xFB, 0x00, 0x96, 0x00, 0xFF, 0xFF,
		0xAA, 0x02, 0x35, 0x01, 0x20, 0x03, 0x60, 0x62,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x80, 0x9C, 0xCE, 0xFF, 0x01, 0x32, 0x64, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x80, 0xFA, 0xFD, 0xFF, 0x01, 0x03, 0x06, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x00, 0x00, 0x14, 0x14, 0x14, 0x05, 0x00, 0x00,
		0x06, 0x00, 0x19, 0x10, 0x17, 0x17, 0x3E, 0x17,
		0x17, 0x02, 0x1C, 0x03, 0x0D, 0x32, 0x0A, 0xFF,
		0x0A, 0x12, 0x66, 0xFF, 0x02, 0x05, 0x05, 0x01,
		0x70, 0xC2, 0x0A, 0x00, 0x00, 0x00, 0x03, 0x99,
		0x99, 0x96, 0xFF, 0xCC, 0x4B, 0x0F, 0x71, 0xC1,
		0x62, 0x05, 0x14, 0x0A, 0x2C, 0x01, 0x5F, 0x05,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
		0x98, 0x0D, 0x48, 0x0D, 0x03, 0x00, 0x0A, 0x0A,
		0x64, 0x00, 0x54, 0x0B, 0x64, 0x02, 0x02, 0x02,
		0x7D, 0x73, 0xFA, 0xE6, 0xBC, 0xB7, 0x6C, 0x71,
		0x4B, 0x78, 0x82, 0x09, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x40, 0x7A, 0x0F, 0xFF, 0x04, 0x00, 0x06, 0x01,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	},
	0,      // enable i2c_sh
	4350,   // full voltage threshold
	2900,   // design capacity
	130,    // charge terminal current
	0x0106, // parameter revision
	0x0004, // battery code
	0,      // default cycle count
	2900,   // default battery capacity
	140     // default battery impedance
	};

	// Celxpert Coslight battery parameter
	struct mm8033_batparams params_celxpert = {
	{
		0xEB, 0x17, 0xC0, 0x1F, 0x35, 0x2C, 0xBE, 0x3F,
		0xB0, 0x42, 0x36, 0x60, 0xB6, 0x71, 0x41, 0x79,
		0x73, 0x11, 0xB7, 0xC1, 0xC8, 0x45, 0x53, 0x0A,
		0xD8, 0xE7, 0xC5, 0x12, 0x14, 0x07, 0xF0, 0xF4,
		0x93, 0x05, 0x2D, 0x05, 0x72, 0xFA, 0x96, 0x01,
		0xB4, 0x04, 0x65, 0xFB, 0x1C, 0x01, 0x56, 0x03,
		0x05, 0xFE, 0xDA, 0xFF, 0x48, 0xFD, 0x12, 0x06,
		0x2C, 0xFD, 0xA4, 0xD1, 0x33, 0x37, 0x59, 0xEF,
		0x00, 0x0E, 0x0D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x14, 0x2F, 0x01, 0x66, 0x30, 0x00, 0x72, 0x06,
		0x54, 0x0B, 0x48, 0x0D, 0x30, 0x11, 0x32, 0x41,
		0x1E, 0x05, 0x3C, 0x2D, 0x78, 0x01, 0x00, 0x01,
		0x9E, 0x0C, 0xF2, 0x0C, 0x39, 0x0D, 0x8E, 0x0D,
		0xF3, 0x0D, 0x2F, 0x0E, 0x5D, 0x0E, 0x69, 0x0E,
		0x74, 0x0E, 0x8D, 0x0E, 0xAB, 0x0E, 0xC4, 0x0E,
		0xE3, 0x0E, 0x09, 0x0F, 0x31, 0x0F, 0x62, 0x0F,
		0xA7, 0x0F, 0xF6, 0x0F, 0x6B, 0x10, 0x13, 0x11,
		0x00, 0x00, 0x05, 0x00, 0x0A, 0x00, 0x14, 0x00,
		0x26, 0x00, 0x32, 0x00, 0x41, 0x00, 0x67, 0x00,
		0x84, 0x00, 0xB4, 0x00, 0xFA, 0x00, 0x36, 0x01,
		0x90, 0x01, 0xEA, 0x01, 0x2B, 0x02, 0x62, 0x02,
		0xA8, 0x02, 0xF8, 0x02, 0x66, 0x03, 0xFC, 0x03,
		0x00, 0xE1, 0x00, 0x4D, 0xE1, 0x00, 0xA6, 0xF0,
		0x00, 0xFF, 0xDD, 0x00, 0x0D, 0xF6, 0x00, 0x39,
		0xCB, 0x00, 0x5A, 0xF6, 0x00, 0x80, 0x3C, 0x00,
		0x14, 0x0A, 0x05, 0x0A, 0x5C, 0xFF, 0xE4, 0x00,
		0x54, 0x0B, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x08, 0x09, 0xFF, 0x6E, 0x00, 0x6E, 0x00,
		0x50, 0x00, 0x5A, 0x00, 0x00, 0x08, 0x09, 0xFF,
		0xFA, 0x00, 0xFA, 0x00, 0xA0, 0x00, 0xAA, 0x00,
		0x00, 0x08, 0x09, 0xFF, 0x3A, 0x02, 0x3A, 0x02,
		0x2C, 0x01, 0x90, 0x01, 0x00, 0x08, 0x09, 0x8C,
		0xFF, 0xFF, 0x94, 0x02, 0x94, 0x02, 0x90, 0x01,
		0x90, 0x01, 0xE0, 0x01, 0xFF, 0xFF, 0x00, 0x08,
		0x09, 0xB4, 0xFF, 0xFF, 0x84, 0x03, 0x84, 0x03,
		0x58, 0x02, 0x3A, 0x02, 0xA8, 0x02, 0xFF, 0xFF,
		0x56, 0x0E, 0x0A, 0x1E, 0xEC, 0x0E, 0x0A, 0x10,
		0xE6, 0x0C, 0xD4, 0xFE, 0x94, 0xFB, 0x98, 0x03,
		0xAA, 0x00, 0xA8, 0x94, 0x0F, 0x66, 0xFA, 0x1E,
		0xF4, 0x01, 0xAE, 0x00, 0x02, 0x02, 0x32, 0x0A,
		0x72, 0x01, 0xFC, 0x00, 0x96, 0x00, 0xFF, 0xFF,
		0xAA, 0x02, 0x35, 0x01, 0x20, 0x03, 0x60, 0x62,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x80, 0x9C, 0xCE, 0xFF, 0x01, 0x32, 0x64, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x80, 0xFA, 0xFD, 0xFF, 0x01, 0x03, 0x06, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x00, 0x00, 0x14, 0x14, 0x14, 0x05, 0x00, 0x00,
		0x06, 0x00, 0x19, 0x10, 0x17, 0x17, 0x3E, 0x17,
		0x17, 0x02, 0x1C, 0x03, 0x0D, 0x32, 0x0A, 0xFF,
		0x0A, 0x12, 0x66, 0xFF, 0x02, 0x05, 0x05, 0x01,
		0x70, 0xC2, 0x0A, 0x00, 0x00, 0x00, 0x03, 0x99,
		0x99, 0x96, 0xFF, 0xCC, 0x4B, 0x0F, 0x71, 0xC1,
		0x62, 0x05, 0x14, 0x0A, 0x2C, 0x01, 0x5F, 0x05,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
		0xAC, 0x0D, 0x48, 0x0D, 0x03, 0x00, 0x0A, 0x0A,
		0x64, 0x00, 0x54, 0x0B, 0x64, 0x02, 0x02, 0x02,
		0x7D, 0x73, 0xFA, 0xE6, 0xBC, 0xB7, 0x6C, 0x71,
		0x4B, 0x78, 0x82, 0x09, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x30, 0x64, 0x0F, 0xFF, 0x05, 0x00, 0x06, 0x01,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	},
	0,      // enable i2c_sh
	4350,   // full voltage threshold
	2900,   // design capacity
	130,    // charge terminal current
	0x0106, // parameter revision
	0x0005, // battery code
	0,      // default cycle count
	2900,   // default battery capacity
	170     // default battery impedance
	};

	pr_info("[B]%s(%d): +++\n", __func__, __LINE__);

	ret = mm8033_read_bytes(chip, MM8033_Identify, reg, 2);
	if (ret) {
		pr_err("Couldn't read MM8033_Identify ret=%d", ret);
		return ret;
	}

	chip->identify = (reg[1] << 8) | reg[0];
	
	if ((chip->identify) != MM8033_ID) {
		pr_err("MM8033_ID Mismatch!! MM8033_ID=0x%x, chip->identify=0x%x\n", MM8033_ID, chip->fgstat);
		return -1;
	}

	do {
		ret = mm8033_read_bytes(chip, MM8033_FGSTAT, reg, 2);
		if (ret) {
			pr_err("Couldn't read MM8033_FGSTAT ret=%d", ret);
			return ret;
		}

		chip->fgstat = (reg[1] << 8) | reg[0];
		
		if ((chip->fgstat) & 0x0001) {
			pr_err("MM8033 Data not ready!, chip->fgstat=0x%x\n", chip->fgstat);
		}
	} while ((chip->fgstat) & 0x0001);

	ret = mm8033_read_bytes(chip, MM8033_Voltage, reg, 2);
	if (ret) {
		pr_err("Failed to read MM8033_Voltage rc=%d\n", ret);
		return ret;
	}

	val = ((reg[1] & 0x1F) << 8) | reg[0];

	chip->voltage_now = val * 1000;

	/*[Arima_5830][bozhi_lin] gauge add hw varient to check can supoort ocv correction 20160408 begin*/
	/*[Arima_5833][bozhi_lin] gauge do OCV correction when power-on 20160318 begin*/
	#if 0
	ret = mm8033_ocv_correction(chip);
	if (ret) {
		pr_err("Failed to mm8033_ocv_correction ret=%d\n", ret);
		return ret;
	}
	#endif
	/*[Arima_5833][bozhi_lin] 20160318 end*/
	/*[Arima_5830][bozhi_lin] 20160408 end*/
	
//	if (val < SYSTEM_LOWVOLTAGE_LIMIT) return -1;

	// get project ids
	ret = mm8033_read_bytes(chip, MM8033_Project_Name, reg, 2);
	if (ret) {
		pr_err("Failed to read MM8033_Project_Name rc=%d\n", ret);
		return ret;
	}

	projectname_low = reg[0];
	projectname_high = reg[1];

	//get vendor id
	ret = mm8033_read_bytes(chip, MM8033_Pack_Cell_Vendor, reg, 2);
	if (ret) {
		pr_err("Failed to read MM8033_Pack_Cell_Vendor rc=%d\n", ret);
		return ret;
	}
	packvendor = reg[0];
	cellvendor = reg[1];
	
	//get ic date
	ret = mm8033_read_bytes(chip, MM8033_Manufacture_Date, reg, 2);
	if (ret) {
		pr_err("Failed to read MM8033_Manufacture_Date rc=%d\n", ret);
		return ret;
	}
	date_low = reg[0];
	date_high = reg[1];
	//show bat parameter date[15:9]=year, date[8:5]=week_1, date[4:0]=week_2
	chip->bat_year = (int)(date_high >> 1);
	chip->bat_week_1 = (int)( ((date_high & 0x01) << 3) +((date_low & 0xe0) >> 5) );
	chip->bat_week_2 = (int)(date_low & 0x1f);

	// select params
	if ((projectname_low != (u8)'Z') || (projectname_high != (u8)'2')) {
		batparams = &params_nvt;
		pr_info("[B]%s(%d): PROJECT is %c%c, not Z2, use NVT as default\n", __func__, __LINE__, projectname_low, projectname_high);
		chip->batt_id = "----";
		chip->invalid_bat = 1;
	} else {
		if ((packvendor == (u8)'N') && (cellvendor == (u8)'3')) {
			batparams = &params_nvt;
			pr_info("[B]%s(%d): BATTERY is NVT\n", __func__, __LINE__);
			chip->batt_id = "Z2N3";
		} else if ((packvendor == (u8)'S') && (cellvendor == (u8)'3')) {
			batparams = &params_simplo;
			pr_info("[B]%s(%d): BATTERY is SIMPLO\n", __func__, __LINE__);
			chip->batt_id = "Z2S3";
		} else if ((packvendor == (u8)'C') && (cellvendor == (u8)'3')) {
			batparams = &params_celxpert;
			pr_info("[B]%s(%d): BATTERY is CELXPERT\n", __func__, __LINE__);
			chip->batt_id = "Z2C3";
		} else {
			batparams = &params_nvt;
			pr_info("[B]%s(%d): PROJECT is %c%c, not Z2, use NVT as default\n", __func__, __LINE__, projectname_low, projectname_high);
			chip->batt_id = "----";
			chip->invalid_bat = 1;
		}
	}

	// copy params
	for (i = 0; i < 512; i++) {
		chip->batparams.params[i] = batparams->params[i];
	}
	chip->batparams.en_i2c_sh = batparams->en_i2c_sh;
	chip->batparams.fullvoltage_thr = batparams->fullvoltage_thr;
	chip->batparams.designcapacity = batparams->designcapacity;
	chip->batparams.charge_terminal_current = batparams->charge_terminal_current;
	chip->batparams.paramrevision = batparams->paramrevision;
	chip->batparams.batterycode = batparams->batterycode;
	chip->batparams.default_cyclecount = batparams->default_cyclecount;
	chip->batparams.default_batterycapacity = batparams->default_batterycapacity;
	chip->batparams.default_batteryimpedance = batparams->default_batteryimpedance;
	chip->cyclecount = batparams->default_cyclecount;
	chip->batterycapacity = batparams->default_batterycapacity;
	chip->batteryimpedance = batparams->default_batteryimpedance;

	ret = mm8033_checkRamData(chip);
	if (ret) return ret;


	pr_info("[B]%s(%d): ---\n", __func__, __LINE__);

	return 0;
}

static int mm8033_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc;
	struct mm8033_chip *chip;
	struct power_supply *usb_psy;

	pr_debug("[B]%s(%d): \n", __func__, __LINE__);

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->resume_completed = true;
	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;
	mutex_init(&chip->read_write_lock);

	/* probe the device to check if its actually connected */
	rc = mm8033_checkdevice(chip);
	if (rc) {
		pr_err("Failed to detect MM8033, device may be absent\n");
		return -ENODEV;
	}

	i2c_set_clientdata(client, chip);
	mutex_init(&chip->irq_complete);
	chip->default_i2c_addr = client->addr;

	pr_debug("default_i2c_addr=%x\n", chip->default_i2c_addr);

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto fail_hw_init;
	}

	mm8033_gauge_sysfs_init();

	chip->batt_gauge_mm8033_psy.name		= "gauge_mm8033";
/*[Arima_5833][bozhi_lin] fix battery capacity will show 99% when battery is full 20160601 begin*/
	chip->batt_gauge_mm8033_psy.type		= POWER_SUPPLY_TYPE_UNKNOWN;
/*[Arima_5833][bozhi_lin] 20160601 end*/
	chip->batt_gauge_mm8033_psy.get_property	= mm8033_battery_get_property;
	chip->batt_gauge_mm8033_psy.set_property	= mm8033_battery_set_property;
	chip->batt_gauge_mm8033_psy.properties	= mm8033_battery_properties;
	chip->batt_gauge_mm8033_psy.num_properties  = ARRAY_SIZE(mm8033_battery_properties);
	chip->batt_gauge_mm8033_psy.property_is_writeable = mm8033_battery_is_writeable;

	rc = power_supply_register(chip->dev, &chip->batt_gauge_mm8033_psy);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to register batt_gauge_mm8033_psy rc = %d\n", rc);
		goto fail_hw_init;
	}

	chip->debug_root = debugfs_create_dir("mm8033", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cnfg debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("fg_parameters_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &fgpara_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create status debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

	}

	the_chip = chip;

	dev_info(chip->dev, "MM8033 identify=0x%x probe success! batt=%d usb=%d soc=%d\n",
			chip->identify,
			mm8033_get_prop_batt_present(chip),
			chip->usb_present,
			mm8033_get_prop_batt_capacity(chip));

	return 0;

fail_hw_init:
	return rc;
}

static int mm8033_remove(struct i2c_client *client)
{
	struct mm8033_chip *chip = i2c_get_clientdata(client);

	mm8033_gauge_sysfs_deinit();

	power_supply_unregister(&chip->batt_gauge_mm8033_psy);
	mutex_destroy(&chip->read_write_lock);
	mutex_destroy(&chip->irq_complete);
	debugfs_remove_recursive(chip->debug_root);

	the_chip = NULL;

	return 0;
}

static int mm8033_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mm8033_chip *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);

	return 0;
}

static int mm8033_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mm8033_chip *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	mutex_unlock(&chip->irq_complete);

	power_supply_changed(&chip->batt_gauge_mm8033_psy);

	return 0;
}


static const struct dev_pm_ops mm8033_pm_ops = {
	.resume		= mm8033_resume,
	.suspend	= mm8033_suspend,
};

static struct of_device_id mm8033_match_table[] = {
	{ .compatible = "qcom,mm8033-gauge",},
	{ },
};

static const struct i2c_device_id mm8033_id[] = {
	{"mm8033-gauge", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, mm8033_id);

static struct i2c_driver mm8033_driver = {
	.driver		= {
		.name		= "mm8033-gauge",
		.owner		= THIS_MODULE,
		.of_match_table	= mm8033_match_table,
		.pm		= &mm8033_pm_ops,
	},
	.probe		= mm8033_probe,
	.remove		= mm8033_remove,
	.id_table	= mm8033_id,
};

module_i2c_driver(mm8033_driver);

MODULE_DESCRIPTION("MM8033 Gauge IC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:mm8033-gauge");
