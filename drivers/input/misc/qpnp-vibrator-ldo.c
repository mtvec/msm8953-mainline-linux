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

#include <linux/atomic.h>
#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
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

/* We rely on having 255 levels of strength */
static_assert((QPNP_VIB_LDO_VMAX_UV - QPNP_VIB_LDO_VMIN_UV) /
		      QPNP_VIB_LDO_VOLT_STEP_UV ==
	      255);

struct vib_ldo_chip {
	struct regmap *regmap;
	struct input_dev *input_dev;
	struct mutex lock;
	struct work_struct vib_work;

	u16 base;
	int vmax_uV;
	int ldo_uV;
	bool vib_enabled;

	/* Atomic because shared between input_ff_create_memless callback
	 * (called from timer softirq) and workqueue
	 */
	atomic_t strength;
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

static void qpnp_vib_work(struct work_struct *work)
{
	struct vib_ldo_chip *chip =
		container_of(work, struct vib_ldo_chip, vib_work);
	u8 strength = atomic_read(&chip->strength);

	if (strength) {
		int uv = QPNP_VIB_LDO_VMIN_UV +
			 strength * QPNP_VIB_LDO_VOLT_STEP_UV;
		qpnp_vib_ldo_set_voltage(chip, uv);
	}

	qpnp_vib_ldo_enable(chip, !!strength);
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

	return ret;
}

static int qpnp_vib_play_effect(struct input_dev *dev, void *data,
				struct ff_effect *effect)
{
	struct vib_ldo_chip *vib = input_get_drvdata(dev);
	u8 strength;

	/* Map magnitude in the [0, 255] range */
	strength = effect->u.rumble.strong_magnitude >> 8;
	if (!strength)
		strength = effect->u.rumble.weak_magnitude >> 12;

	atomic_set(&vib->strength, strength);
	schedule_work(&vib->vib_work);

	return 0;
}

static int qpnp_vibrator_ldo_suspend(struct device *dev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
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
	mutex_init(&chip->lock);
	INIT_WORK(&chip->vib_work, qpnp_vib_work);

	dev_set_drvdata(&pdev->dev, chip);

	return 0;
}

static int qpnp_vibrator_ldo_remove(struct platform_device *pdev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(&pdev->dev);

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
