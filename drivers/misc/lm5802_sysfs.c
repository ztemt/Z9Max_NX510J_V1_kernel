/*
 * LM5802 Sysfs Driver
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_data/lm5802.h>
#include <linux/slab.h>

#define LM5802_RESULT_BUF_LEN		300

/*
 * Loading is done by the user-space.
 * The interrupt should be configured after user-space UEvent thread is ready.
 */
static ssize_t lm5802_load_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct lm5802 *lm5802 = dev_get_drvdata(dev);
	unsigned long load;
	int ret;

	if (kstrtoul(buf, 0, &load))
		return -EINVAL;

	dev_info(lm5802->dev, "Load request from the A2I service\n");

	ret = lm5802_load_by_user(lm5802);
	if (ret) {
		dev_err(dev, "Load err: %d\n", ret);
		return ret;
	}

	return len;
}

static bool lm5802_is_valid_enrollment_id(int id)
{
	if ((id < LM5802_ID_USER1) || (id > LM5802_ID_USER_INDEP))
		return false;

	return true;
}

static ssize_t lm5802_enrollment_id_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t len)
{
	struct lm5802 *lm5802 = dev_get_drvdata(dev);
	unsigned long id;

	if (kstrtoul(buf, 0, &id))
		return -EINVAL;

	if (!lm5802_is_valid_enrollment_id(id))
		return -EINVAL;

	lm5802->enrollment_id = id;

	return len;
}

static ssize_t lm5802_enable_detect_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t len)
{
	struct lm5802 *lm5802 = dev_get_drvdata(dev);
	unsigned long enable;
	int id = lm5802->enrollment_id;
	unsigned int val;
	int ret;

	if (kstrtoul(buf, 0, &enable))
		return -EINVAL;

	if (enable)
		val = BIT(id);
	else
		val = ~BIT(id);

	dev_info(lm5802->dev, "enable detection: id= %d, %s\n", id,
		 enable ? "enabled" : "disabled");

	ret = lm5802_reg_update_bit(lm5802, LM5802_REG_ENABLE_DETECT,
				    BIT(id), val);
	if (ret)
		return ret;

	return len;
}

static ssize_t lm5802_clear_enrollment_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t len)
{
	struct lm5802 *lm5802 = dev_get_drvdata(dev);
	unsigned long id;
	int ret;
	u8 val;

	if (kstrtoul(buf, 0, &id))
		return -EINVAL;

	dev_info(lm5802->dev, "clear enrollment: id= %ld\n", id);

	if (!lm5802_is_valid_enrollment_id(id))
		return -EINVAL;

	val = id;
	ret = lm5802_reg_write(lm5802, LM5802_REG_FLUSH_ENROLLMENT, &val, 1);
	if (ret)
		return ret;

	return len;
}

static ssize_t lm5802_validate_enrollment_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t len)
{
	struct lm5802 *lm5802 = dev_get_drvdata(dev);
	unsigned long id;
	int ret;
	u8 val;

	if (kstrtoul(buf, 0, &id))
		return -EINVAL;

	dev_info(lm5802->dev, "validate enrollment: id= %ld\n", id);

	if (!lm5802_is_valid_enrollment_id(id))
		return -EINVAL;

	ret = lm5802_reg_read(lm5802, LM5802_REG_VALIDATE_ENROLLMENT, &val, 1);
	if (ret)
		return ret;

	dev_info(lm5802->dev, "enrollment status = 0x%.2x\n", val);

	if (val & BIT(id))
		return len;
	else
		return -EINVAL;
}

static bool lm5802_is_valid_training_mode(int mode)
{
	if ((mode < LM5802_TRAINING_ABORT) || (mode > LM5802_TRAINING_USER4))
		return false;

	return true;
}

static bool lm5802_is_in_training(struct lm5802 *lm5802)
{
	int ret;
	u16 val = 0;

	ret = lm5802_reg_read(lm5802, LM5802_REG_APP_STATUS, (u8 *)&val, 2);
	if (ret)
		return false;

	return val & LM5802_IN_TRAINING;
}

static ssize_t lm5802_training_mode_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t len)
{
	struct lm5802 *lm5802 = dev_get_drvdata(dev);
	unsigned long mode;
	int ret;
	u8 val;

	if (kstrtoul(buf, 0, &mode))
		return -EINVAL;

	if (!lm5802_is_valid_training_mode(mode))
		return -EINVAL;

	lm5802->training_mode = mode;

	if (mode == LM5802_TRAINING_ABORT) {
		if (!lm5802_is_in_training(lm5802))
			return len;

		ret = lm5802_reg_write(lm5802, LM5802_REG_ABORT_TRAINING, NULL,
				       0);
		if (ret)
			return ret;

		return len;
	}

	val = (u8)mode;
	ret = lm5802_reg_write(lm5802, LM5802_REG_START_TRAINING, &val, 1);
	if (ret)
		return ret;

	return len;
}

static ssize_t lm5802_training_result_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct lm5802 *lm5802 = dev_get_drvdata(dev);
	char reason[LM5802_RESULT_BUF_LEN];

	memset(reason, 0, sizeof(reason));

	if (lm5802->training_result == LM5802_TRAINING_IS_GOOD)
		return scnprintf(buf, PAGE_SIZE, "%s\n", "Training is good");

	if (lm5802->training_result & LM5802_REASON_HIGH_VOLUME)
		strcat(reason, "Volume is too high\n");
	if (lm5802->training_result & LM5802_REASON_LOW_VOLUME)
		strcat(reason, "Volume is too low\n");
	if (lm5802->training_result & LM5802_REASON_LONG_CMD)
		strcat(reason, "Command is too long\n");
	if (lm5802->training_result & LM5802_REASON_SHORT_CMD)
		strcat(reason, "Command is too short\n");
	if (lm5802->training_result & LM5802_REASON_NOISY_ENV)
		strcat(reason, "Environment is too noisy\n");
	if (lm5802->training_result & LM5802_REASON_SIMPLE_CMD)
		strcat(reason, "Command is too simple\n");
	if (lm5802->training_result & LM5802_REASON_INCONSISTENT)
		strcat(reason, "All three commands are inconsistent\n");

	return scnprintf(buf, PAGE_SIZE, "%s", reason);
}

static u8 lm5802_compute_checksum(const u8 *val, int len)
{
	int i;
	long checksum = 0;

	for (i = 0; i < len; i++)
		checksum += val[i];

	return checksum &= 0xFF;
}

/*
 * Trigger mode switch(single/dual) through advanced settings register.
 *
 * Threshold, trigger mode and checksum should be updated.
 * Other values should be kept.
 */
static ssize_t lm5802_trigger_mode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t len)
{
	struct lm5802 *lm5802 = dev_get_drvdata(dev);
	unsigned long mode;
	int ret;
	u8 val[LM5802_ADV_SETTINGS_LENGTH] = { 0, };

	if (kstrtoul(buf, 0, &mode))
		return -EINVAL;

	/* Read advanced settings register */
	ret = lm5802_reg_read(lm5802, LM5802_REG_ADVANCED_SETTINGS, (u8 *)&val,
			      sizeof(val));
	if (ret)
		return ret;

	/* Change threshold key value */
	if (mode == LM5802_SINGLE_TRIGGER) {
		val[LM5802_THRES_KEY_LO_INDEX] = LM5802_SINGLE_THRES_KEY_LO;
		val[LM5802_THRES_KEY_HI_INDEX] = LM5802_SINGLE_THRES_KEY_HI;
	} else if (mode == LM5802_DUAL_TRIGGER) {
		val[LM5802_THRES_KEY_LO_INDEX] = LM5802_DUAL_THRES_KEY_LO;
		val[LM5802_THRES_KEY_HI_INDEX] = LM5802_DUAL_THRES_KEY_HI;
	} else {
		return -EINVAL;
	}

	/* Switch trigger mode */
	val[LM5802_TRIGGER_MODE_INDEX] = (u8)mode;

	/* Compute the checksum */
	val[LM5802_CHECKSUM_INDEX] =
		lm5802_compute_checksum(val, LM5802_ADV_SETTINGS_LENGTH - 1);

	/* Update advanced settings register */
	ret = lm5802_reg_write(lm5802, LM5802_REG_ADVANCED_SETTINGS,
			       (u8 *)&val, sizeof(val));
	if (ret)
		return ret;

	dev_info(lm5802->dev, "Trigger mode: %d\n", (u8)mode);

	return len;
}

/* Device control attributes */
static DEVICE_ATTR(load, S_IWUSR, NULL, lm5802_load_store);
static DEVICE_ATTR(enrollment_id, S_IWUSR, NULL, lm5802_enrollment_id_store);
static DEVICE_ATTR(enable_detect, S_IWUSR, NULL, lm5802_enable_detect_store);
static DEVICE_ATTR(validate_enrollment, S_IWUSR, NULL,
		   lm5802_validate_enrollment_store);
static DEVICE_ATTR(clear_enrollment, S_IWUSR, NULL,
		   lm5802_clear_enrollment_store);
static DEVICE_ATTR(training_mode, S_IWUSR, NULL, lm5802_training_mode_store);
static DEVICE_ATTR(training_result, S_IRUGO, lm5802_training_result_show, NULL);
static DEVICE_ATTR(trigger_mode, S_IWUSR, NULL, lm5802_trigger_mode_store);

static struct attribute *lm5802_attributes[] = {
	&dev_attr_load.attr,
	&dev_attr_enrollment_id.attr,
	&dev_attr_enable_detect.attr,
	&dev_attr_validate_enrollment.attr,
	&dev_attr_clear_enrollment.attr,
	&dev_attr_training_mode.attr,
	&dev_attr_training_result.attr,
	&dev_attr_trigger_mode.attr,
	NULL,
};

static struct attribute_group lm5802_attr_group = {
	.attrs = lm5802_attributes
};

static void lm5802_transfer_option_config(struct lm5802 *lm5802)
{
	int ret;
	u8 val;

	/*
	 * Transfer resume bit should be set when the platform has
	 * size limitation of I2C transfer data.
	 * Otherwise this bit should be cleared.
	 */

	if (lm5802->pdata->is_chunk_xfer)
		val = LM5802_APP_XFER_RESUME;
	else
		val = 0;

	ret = lm5802_reg_write(lm5802, LM5802_REG_XFER_CTRL, &val, 1);
	if (ret)
		dev_warn(lm5802->dev, "Transfer config err: %d\n", ret);
}

static ssize_t lm5802_sysfs_read_userdata(struct file *filp,
					  struct kobject *kobj,
					  struct bin_attribute *attr, char *buf,
					  loff_t offset, size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct lm5802 *lm5802 = dev_get_drvdata(dev);
	int reg_offset;
	int ret;

	if (!lm5802_is_valid_enrollment_id(lm5802->enrollment_id))
		return -EINVAL;

	lm5802_transfer_option_config(lm5802);

	reg_offset = lm5802->enrollment_id - 1;

	ret = lm5802_reg_read(lm5802, LM5802_REG_ENROLLMENT_BASE + reg_offset,
			      buf, lm5802->size_userdata);
	if (ret)
		return ret;

	return lm5802->size_userdata;
}

static ssize_t lm5802_sysfs_write_userdata(struct file *filp,
					   struct kobject *kobj,
					   struct bin_attribute *attr,
					   char *buf, loff_t offset,
					   size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct lm5802 *lm5802 = dev_get_drvdata(dev);
	int reg_offset;
	int ret;

	if (!lm5802_is_valid_enrollment_id(lm5802->enrollment_id))
		return -EINVAL;

	/* Requested file size and supported data size should be same */
	if (lm5802->size_userdata != size) {
		dev_err(lm5802->dev, "Size error\n");
		return -EINVAL;
	}

	lm5802_transfer_option_config(lm5802);

	reg_offset = lm5802->enrollment_id - 1;

	ret = lm5802_reg_write(lm5802, LM5802_REG_ENROLLMENT_BASE + reg_offset,
			       buf, size);
	if (ret)
		return ret;

	return size;
}

static struct bin_attribute lm5802_userdata_attr = {
	.attr  = {
		.name = "userdata",
		.mode = (S_IRUGO | S_IWUSR),
	},
	.size  = 0,
	.read  = lm5802_sysfs_read_userdata,
	.write = lm5802_sysfs_write_userdata,
};

int lm5802_add_sysfs(struct lm5802 *lm5802)
{
	int ret;

	ret = device_create_bin_file(lm5802->dev, &lm5802_userdata_attr);
	if (ret)
		return ret;

	return sysfs_create_group(&lm5802->dev->kobj, &lm5802_attr_group);
}
EXPORT_SYMBOL_GPL(lm5802_add_sysfs);

void lm5802_remove_sysfs(struct lm5802 *lm5802)
{
	sysfs_remove_group(&lm5802->dev->kobj, &lm5802_attr_group);
	device_remove_bin_file(lm5802->dev, &lm5802_userdata_attr);
}
EXPORT_SYMBOL_GPL(lm5802_remove_sysfs);

MODULE_DESCRIPTION("LM5802 Sysfs Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL v2");
