/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* Vibrator-LDO register definitions */
#define QPNP_VIB_LDO_REG_STATUS1 0x08
#define QPNP_VIB_LDO_VREG_READY BIT(7)

#define QPNP_VIB_LDO_REG_VSET_LB 0x40

#define QPNP_VIB_LDO_REG_EN_CTL 0x46
#define QPNP_VIB_LDO_EN BIT(7)

/* Vibrator-LDO voltage settings */
#define QPNP_VIB_LDO_VMIN_UV 1504000
#define QPNP_VIB_LDO_VMAX_UV 3544000
#define QPNP_VIB_LDO_VOLT_STEP_UV 8000

/*
 * Define vibration periods: default(5sec), min(50ms), max(15sec) and
 * overdrive(30ms).
 */
#define QPNP_VIB_MIN_PLAY_MS 50
#define QPNP_VIB_PLAY_MS 5000
#define QPNP_VIB_MAX_PLAY_MS 15000
#define QPNP_VIB_OVERDRIVE_PLAY_MS 30

struct vib_ldo_chip {
	struct led_classdev cdev;
	struct regmap *regmap;
	struct input_dev *input_dev;
	struct mutex lock;
	struct hrtimer stop_timer;
	struct hrtimer overdrive_timer;
	struct work_struct vib_work;
	struct work_struct overdrive_work;

	u16 base;
	int vmax_uV;
	int overdrive_volt_uV;
	int ldo_uV;
	int state;
	u64 vib_play_ms;
	bool vib_enabled;
	bool disable_overdrive;
};

static inline int qpnp_vib_ldo_poll_status(struct vib_ldo_chip *chip)
{
	unsigned int val;
	int ret;

	ret = regmap_read_poll_timeout(chip->regmap,
				       chip->base + QPNP_VIB_LDO_REG_STATUS1,
				       val, val & QPNP_VIB_LDO_VREG_READY, 100,
				       1000);
	if (ret < 0) {
		pr_err("Vibrator LDO vreg_ready timeout, status=0x%02x, ret=%d\n",
		       val, ret);

		/* Keep VIB_LDO disabled */
		regmap_update_bits(chip->regmap,
				   chip->base + QPNP_VIB_LDO_REG_EN_CTL,
				   QPNP_VIB_LDO_EN, 0);
	}

	return ret;
}

static int qpnp_vib_ldo_set_voltage(struct vib_ldo_chip *chip, int new_uV)
{
	u32 vlevel;
	u8 reg[2];
	int ret;

	if (chip->ldo_uV == new_uV)
		return 0;

	vlevel = roundup(new_uV, QPNP_VIB_LDO_VOLT_STEP_UV) / 1000;
	reg[0] = vlevel & 0xff;
	reg[1] = (vlevel & 0xff00) >> 8;
	ret = regmap_bulk_write(chip->regmap,
				chip->base + QPNP_VIB_LDO_REG_VSET_LB, reg, 2);
	if (ret < 0) {
		pr_err("regmap write failed, ret=%d\n", ret);
		return ret;
	}

	if (chip->vib_enabled) {
		ret = qpnp_vib_ldo_poll_status(chip);
		if (ret < 0) {
			pr_err("Vibrator LDO status polling timedout\n");
			return ret;
		}
	}

	chip->ldo_uV = new_uV;
	return ret;
}

static inline int qpnp_vib_ldo_enable(struct vib_ldo_chip *chip, bool enable)
{
	int ret;

	if (chip->vib_enabled == enable)
		return 0;

	ret = regmap_update_bits(chip->regmap,
				 chip->base + QPNP_VIB_LDO_REG_EN_CTL,
				 QPNP_VIB_LDO_EN, enable ? QPNP_VIB_LDO_EN : 0);
	if (ret < 0) {
		pr_err("Program Vibrator LDO %s is failed, ret=%d\n",
		       enable ? "enable" : "disable", ret);
		return ret;
	}

	if (enable) {
		ret = qpnp_vib_ldo_poll_status(chip);
		if (ret < 0) {
			pr_err("Vibrator LDO status polling timedout\n");
			return ret;
		}
	}

	chip->vib_enabled = enable;

	return ret;
}

static int qpnp_vibrator_play_on(struct vib_ldo_chip *chip)
{
	int volt_uV;
	int ret;

	volt_uV = chip->vmax_uV;
	if (!chip->disable_overdrive)
		volt_uV = chip->overdrive_volt_uV ?
				  chip->overdrive_volt_uV :
				  min(chip->vmax_uV * 2, QPNP_VIB_LDO_VMAX_UV);

	ret = qpnp_vib_ldo_set_voltage(chip, volt_uV);
	if (ret < 0) {
		pr_err("set voltage = %duV failed, ret=%d\n", volt_uV, ret);
		return ret;
	}
	pr_debug("voltage set to %d uV\n", volt_uV);

	ret = qpnp_vib_ldo_enable(chip, true);
	if (ret < 0) {
		pr_err("vibration enable failed, ret=%d\n", ret);
		return ret;
	}

	if (!chip->disable_overdrive)
		hrtimer_start(&chip->overdrive_timer,
			      ms_to_ktime(QPNP_VIB_OVERDRIVE_PLAY_MS),
			      HRTIMER_MODE_REL);

	return ret;
}

static void qpnp_vib_work(struct work_struct *work)
{
	struct vib_ldo_chip *chip =
		container_of(work, struct vib_ldo_chip, vib_work);
	int ret = 0;

	if (chip->state) {
		if (!chip->vib_enabled)
			ret = qpnp_vibrator_play_on(chip);

		if (ret == 0)
			hrtimer_start(&chip->stop_timer,
				      ms_to_ktime(chip->vib_play_ms),
				      HRTIMER_MODE_REL);
	} else {
		if (!chip->disable_overdrive) {
			hrtimer_cancel(&chip->overdrive_timer);
			cancel_work_sync(&chip->overdrive_work);
		}
		qpnp_vib_ldo_enable(chip, false);
	}
}

static enum hrtimer_restart vib_stop_timer(struct hrtimer *timer)
{
	struct vib_ldo_chip *chip =
		container_of(timer, struct vib_ldo_chip, stop_timer);

	chip->state = 0;
	schedule_work(&chip->vib_work);
	return HRTIMER_NORESTART;
}

static void qpnp_vib_overdrive_work(struct work_struct *work)
{
	struct vib_ldo_chip *chip =
		container_of(work, struct vib_ldo_chip, overdrive_work);
	int ret;

	mutex_lock(&chip->lock);

	/* LDO voltage update not required if Vibration disabled */
	if (!chip->vib_enabled)
		goto unlock;

	ret = qpnp_vib_ldo_set_voltage(chip, chip->vmax_uV);
	if (ret < 0) {
		pr_err("set vibration voltage = %duV failed, ret=%d\n",
		       chip->vmax_uV, ret);
		qpnp_vib_ldo_enable(chip, false);
		goto unlock;
	}
	pr_debug("voltage set to %d\n", chip->vmax_uV);

unlock:
	mutex_unlock(&chip->lock);
}

static enum hrtimer_restart vib_overdrive_timer(struct hrtimer *timer)
{
	struct vib_ldo_chip *chip =
		container_of(timer, struct vib_ldo_chip, overdrive_timer);
	schedule_work(&chip->overdrive_work);
	pr_debug("overdrive timer expired\n");
	return HRTIMER_NORESTART;
}

static int qpnp_vib_parse_dt(struct device *dev, struct vib_ldo_chip *chip)
{
	int ret;

	ret = of_property_read_u32(dev->of_node, "qcom,vib-ldo-volt-uv",
				   &chip->vmax_uV);
	if (ret < 0) {
		pr_err("qcom,vib-ldo-volt-uv property read failed, ret=%d\n",
		       ret);
		return ret;
	}

	chip->disable_overdrive =
		of_property_read_bool(dev->of_node, "qcom,disable-overdrive");

	if (of_find_property(dev->of_node, "qcom,vib-overdrive-volt-uv",
			     NULL)) {
		ret = of_property_read_u32(dev->of_node,
					   "qcom,vib-overdrive-volt-uv",
					   &chip->overdrive_volt_uV);
		if (ret < 0) {
			pr_err("qcom,vib-overdrive-volt-uv property read failed, ret=%d\n",
			       ret);
			return ret;
		}

		/* check against vibrator ldo min/max voltage limits */
		chip->overdrive_volt_uV =
			min(chip->overdrive_volt_uV, QPNP_VIB_LDO_VMAX_UV);
		chip->overdrive_volt_uV =
			max(chip->overdrive_volt_uV, QPNP_VIB_LDO_VMIN_UV);
	}

	return ret;
}

/* Dummy functions for brightness */
static enum led_brightness qpnp_vib_brightness_get(struct led_classdev *cdev)
{
	return 0;
}

static int qpnp_vib_play_effect(struct input_dev *dev, void *data,
				struct ff_effect *effect)
{
	struct vib_ldo_chip *vib = input_get_drvdata(dev);
	u16 speed;

	speed = effect->u.rumble.strong_magnitude >> 8;
	if (!speed)
		speed = effect->u.rumble.weak_magnitude >> 9;

	mutex_lock(&vib->lock);
	hrtimer_cancel(&vib->stop_timer);
	vib->state = !!speed;
	mutex_unlock(&vib->lock);
	schedule_work(&vib->vib_work);

	return 0;
}

static void qpnp_vib_brightness_set(struct led_classdev *cdev,
				    enum led_brightness level)
{
}

static int qpnp_vibrator_ldo_suspend(struct device *dev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	if (!chip->disable_overdrive) {
		hrtimer_cancel(&chip->overdrive_timer);
		cancel_work_sync(&chip->overdrive_work);
	}
	hrtimer_cancel(&chip->stop_timer);
	cancel_work_sync(&chip->vib_work);
	qpnp_vib_ldo_enable(chip, false);
	mutex_unlock(&chip->lock);

	return 0;
}
static SIMPLE_DEV_PM_OPS(qpnp_vibrator_ldo_pm_ops, qpnp_vibrator_ldo_suspend,
			 NULL);

static int qpnp_vibrator_ldo_probe(struct platform_device *pdev)
{
	struct device_node *of_node = pdev->dev.of_node;
	struct vib_ldo_chip *chip;
	int ret;
	u32 base;

	ret = of_property_read_u32(of_node, "reg", &base);
	if (ret < 0) {
		pr_err("reg property reading failed, ret=%d\n", ret);
		return ret;
	}

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		pr_err("couldn't get parent's regmap\n");
		return -EINVAL;
	}

	chip->input_dev = devm_input_allocate_device(&pdev->dev);
	if (!chip->input_dev)
		return -ENOMEM;

	ret = qpnp_vib_parse_dt(&pdev->dev, chip);
	if (ret < 0) {
		pr_err("couldn't parse device tree, ret=%d\n", ret);
		return ret;
	}

	chip->input_dev->name = "qpnp_vibrator_ldo";
	chip->input_dev->id.version = 1;
	chip->input_dev->close = NULL; // TODO
	input_set_drvdata(chip->input_dev, chip);
	input_set_capability(chip->input_dev, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(chip->input_dev, NULL,
				      qpnp_vib_play_effect);
	if (ret) {
		dev_err(&pdev->dev,
			"couldn't register vibrator as FF device\n");
		return ret;
	}

	ret = input_register_device(chip->input_dev);
	if (ret) {
		dev_err(&pdev->dev, "couldn't register input device\n");
		return ret;
	}
	chip->base = (uint16_t)base;
	chip->vib_play_ms = QPNP_VIB_PLAY_MS;
	mutex_init(&chip->lock);
	INIT_WORK(&chip->vib_work, qpnp_vib_work);
	INIT_WORK(&chip->overdrive_work, qpnp_vib_overdrive_work);

	hrtimer_init(&chip->stop_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->stop_timer.function = vib_stop_timer;
	hrtimer_init(&chip->overdrive_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->overdrive_timer.function = vib_overdrive_timer;
	dev_set_drvdata(&pdev->dev, chip);

	chip->cdev.name = "vibrator";
	chip->cdev.brightness_get = qpnp_vib_brightness_get;
	chip->cdev.brightness_set = qpnp_vib_brightness_set;
	chip->cdev.max_brightness = 100;
	ret = devm_led_classdev_register(&pdev->dev, &chip->cdev);
	if (ret < 0) {
		pr_err("Error in registering led class device, ret=%d\n", ret);
		goto fail;
	}

	return 0;

fail:
	mutex_destroy(&chip->lock);
	dev_set_drvdata(&pdev->dev, NULL);
	return ret;
}

static int qpnp_vibrator_ldo_remove(struct platform_device *pdev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(&pdev->dev);

	if (!chip->disable_overdrive) {
		hrtimer_cancel(&chip->overdrive_timer);
		cancel_work_sync(&chip->overdrive_work);
	}
	hrtimer_cancel(&chip->stop_timer);
	cancel_work_sync(&chip->vib_work);
	mutex_destroy(&chip->lock);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static const struct of_device_id vibrator_ldo_match_table[] = {
	{ .compatible = "qcom,qpnp-vibrator-ldo" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vibrator_ldo_match_table);

static struct platform_driver qpnp_vibrator_ldo_driver = {
	.driver	= {
		.name		= "qcom,qpnp-vibrator-ldo",
		.of_match_table	= vibrator_ldo_match_table,
		.pm		= &qpnp_vibrator_ldo_pm_ops,
	},
	.probe	= qpnp_vibrator_ldo_probe,
	.remove	= qpnp_vibrator_ldo_remove,
};
module_platform_driver(qpnp_vibrator_ldo_driver);

MODULE_DESCRIPTION("QCOM QPNP Vibrator-LDO driver");
MODULE_LICENSE("GPL v2");
