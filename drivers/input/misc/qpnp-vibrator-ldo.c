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
#define QPNP_VIB_LDO_REG_VSET_LB 0x40

#define QPNP_VIB_LDO_REG_EN_CTL 0x46
#define QPNP_VIB_LDO_EN BIT(7)

/* Vibrator-LDO voltage settings */
#define QPNP_VIB_LDO_VMIN_UV 1504000
#define QPNP_VIB_LDO_VMAX_UV 3544000
#define QPNP_VIB_LDO_VOLT_STEP_UV 8000
#define QPNP_VIB_LDO_STRENGTH_LEVELS 255

struct vib_ldo_chip {
	struct regmap *regmap;
	struct platform_device *pdev;
	struct input_dev *input_dev;
	struct work_struct vib_work;

	u32 base;
	int vmax_uV;
	int ldo_uV;
	bool vib_enabled;

	/* Atomic because shared between input_ff_create_memless callback
	 * (called from timer softirq) and workqueue
	 */
	atomic_t strength;
};

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
		dev_err(&chip->pdev->dev, "regmap write failed, ret=%d\n", ret);
		return ret;
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
		dev_err(&chip->pdev->dev,
			"Program Vibrator LDO %s is failed, ret=%d\n",
			enable ? "enable" : "disable", ret);
		return ret;
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
		int step = (chip->vmax_uV - QPNP_VIB_LDO_VMIN_UV) /
			   QPNP_VIB_LDO_STRENGTH_LEVELS;
		int uv = QPNP_VIB_LDO_VMIN_UV + strength * step;
		qpnp_vib_ldo_set_voltage(chip, uv);
	}

	qpnp_vib_ldo_enable(chip, !!strength);
}

static int qpnp_vib_parse_dt(struct device *dev, struct vib_ldo_chip *chip)
{
	int error;

	error = of_property_read_u32(dev->of_node, "reg", &chip->base);
	if (error) {
		return dev_err_probe(dev, error,
				     "Failed to read reg property from DT\n");
	}

	error = of_property_read_u32(dev->of_node, "max-uv", &chip->vmax_uV);
	if (error) {
		return dev_err_probe(
			dev, error, "Failed to read max-uv property from DT\n");
	}

	return error;
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

	cancel_work_sync(&chip->vib_work);
	qpnp_vib_ldo_enable(chip, false);

	return 0;
}
static SIMPLE_DEV_PM_OPS(qpnp_vibrator_ldo_pm_ops, qpnp_vibrator_ldo_suspend,
			 NULL);

static int qpnp_vibrator_ldo_probe(struct platform_device *pdev)
{
	struct vib_ldo_chip *chip;
	int ret;
	struct device *dev = &pdev->dev;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->pdev = pdev;

	chip->regmap = dev_get_regmap(dev->parent, NULL);
	if (!chip->regmap) {
		dev_err(dev, "Failed to get regmap\n");
		return -ENODEV;
	}

	chip->input_dev = devm_input_allocate_device(dev);
	if (!chip->input_dev) {
		dev_err(dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	ret = qpnp_vib_parse_dt(dev, chip);
	if (ret < 0)
		return ret;

	chip->input_dev->name = "qpnp_vibrator_ldo";
	chip->input_dev->id.version = 1;
	chip->input_dev->close = NULL; // TODO
	input_set_drvdata(chip->input_dev, chip);
	input_set_capability(chip->input_dev, EV_FF, FF_RUMBLE);

	ret = input_ff_create_memless(chip->input_dev, NULL,
				      qpnp_vib_play_effect);
	if (ret) {
		return dev_err_probe(
			dev, ret, "Failed to register force-feedback device\n");
	}

	ret = input_register_device(chip->input_dev);
	if (ret) {
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to register input device\n");
	}

	INIT_WORK(&chip->vib_work, qpnp_vib_work);
	dev_set_drvdata(&pdev->dev, chip);

	return 0;
}

static int qpnp_vibrator_ldo_remove(struct platform_device *pdev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(&pdev->dev);

	cancel_work_sync(&chip->vib_work);
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
