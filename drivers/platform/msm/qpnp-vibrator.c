/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
*/
/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>

#include <linux/qpnp/vibrator.h>
#include "../../staging/android/timed_output.h"

#define VIB_DEBUG 0

/* log */
#define VIB_LOG(md, fmt, ...) \
printk(md "[VIB]%s(%d): " fmt, __func__, __LINE__, ## __VA_ARGS__)
#ifdef VIB_DEBUG
#define VIB_DEBUG_LOG(md, fmt, ...) \
printk(md "[VIB]%s(%d): " fmt, __func__, __LINE__, ## __VA_ARGS__)
#else /* VIB_DEBUG */
#define VIB_DEBUG_LOG(md, fmt, ...)
#endif /* VIB_DEBUG */

#define QPNP_VIB_VTG_CTL(base)		(base + 0x41)
#define QPNP_VIB_EN_CTL(base)		(base + 0x46)

#define QPNP_VIB_MAX_LEVEL		31
#define QPNP_VIB_MIN_LEVEL		12

#define QPNP_VIB_DEFAULT_TIMEOUT	15000
#define QPNP_VIB_DEFAULT_VTG_LVL	3100

#define QPNP_VIB_EN			BIT(7)
#define QPNP_VIB_VTG_SET_MASK		0x1F
#define QPNP_VIB_LOGIC_SHIFT		4

struct qpnp_vib {
	struct spmi_device *spmi;
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;

	u8  reg_vtg_ctl;
	u8  reg_en_ctl;
	u16 base;
	int state;
	int vtg_level;
	int timeout;
	struct mutex lock;
};

static struct qpnp_vib *vib_dev;

static int qpnp_vib_read_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_readl(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
        VIB_LOG(KERN_ERR, "Error reading address: %X - ret %X\n", reg, rc);

    VIB_DEBUG_LOG(KERN_INFO, "Write addr=[0x%04x] val=[0x%02x] \n", reg, *data);

	return rc;
}

static int qpnp_vib_write_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_writel(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
        VIB_LOG(KERN_ERR, "Error writing address: %X - ret %X\n", reg, rc);

    VIB_DEBUG_LOG(KERN_INFO, "Read addr=[0x%04x] val=[0x%02x] \n", reg, *data);

	return rc;
}

int qpnp_vibrator_config(struct qpnp_vib_config *vib_cfg)
{
	u8 reg = 0;
	int rc = -EINVAL, level;

    VIB_DEBUG_LOG(KERN_INFO, "called. drive_mV=%d\n", vib_cfg->drive_mV);

	if (vib_dev == NULL) {
		pr_err("%s: vib_dev is NULL\n", __func__);
        VIB_LOG(KERN_ERR, "vib_dev is NULL\n");
		return -ENODEV;
	}

	level = vib_cfg->drive_mV / 100;
	if (level) {
		if ((level < QPNP_VIB_MIN_LEVEL) ||
				(level > QPNP_VIB_MAX_LEVEL)) {
            VIB_LOG(KERN_ERR, "Invalid voltage level\n");
			return -EINVAL;
		}
	} else {
        VIB_LOG(KERN_ERR, "Voltage level not specified\n");
		return -EINVAL;
	}

	/* Configure the VTG CTL regiser */
	reg = vib_dev->reg_vtg_ctl;
	reg &= ~QPNP_VIB_VTG_SET_MASK;
	reg |= (level & QPNP_VIB_VTG_SET_MASK);
	rc = qpnp_vib_write_u8(vib_dev, &reg, QPNP_VIB_VTG_CTL(vib_dev->base));
	if (rc){
        VIB_LOG(KERN_ERR, "Fail to qpnp_vib_write_u8(QPNP_VIB_VTG_CTL) rc=[%d] \n", rc);
		return rc;
	}
	vib_dev->reg_vtg_ctl = reg;

	/* Configure the VIB ENABLE regiser */
	reg = vib_dev->reg_en_ctl;
	reg |= (!!vib_cfg->active_low) << QPNP_VIB_LOGIC_SHIFT;
	if (vib_cfg->enable_mode == QPNP_VIB_MANUAL)
		reg |= QPNP_VIB_EN;
	else
		reg |= BIT(vib_cfg->enable_mode - 1);
	rc = qpnp_vib_write_u8(vib_dev, &reg, QPNP_VIB_EN_CTL(vib_dev->base));
	if (rc < 0){
        VIB_LOG(KERN_ERR, "Fail to qpnp_vib_write_u8(QPNP_VIB_EN_CTL) rc=[%d] \n", rc);
		return rc;
	}
	vib_dev->reg_en_ctl = reg;

    VIB_DEBUG_LOG(KERN_INFO, "end.rc=%d\n", rc);
	return rc;
}
EXPORT_SYMBOL(qpnp_vibrator_config);

static int qpnp_vib_set(struct qpnp_vib *vib, int on)
{
	int rc;
	u8 val;

    VIB_DEBUG_LOG(KERN_INFO, "called. on=%d\n", on);

    mutex_lock(&vib->lock);
	VIB_DEBUG_LOG(KERN_INFO, "mutex_lock \n");

	if (on) {
        VIB_DEBUG_LOG(KERN_INFO, "VIB ON.vib->reg_vtg_ctl=%d\n", vib->reg_vtg_ctl);
		val = vib->reg_vtg_ctl;
		val &= ~QPNP_VIB_VTG_SET_MASK;
		val |= (vib->vtg_level & QPNP_VIB_VTG_SET_MASK);
		rc = qpnp_vib_write_u8(vib, &val, QPNP_VIB_VTG_CTL(vib->base));
		if (rc < 0){
			VIB_LOG(KERN_ERR, "Fail to qpnp_vib_write_u8(QPNP_VIB_VTG_CTL) rc=[%d] \n", rc);
			mutex_unlock(&vib->lock);
			VIB_DEBUG_LOG(KERN_INFO, "mutex_unlock \n");
			return rc;
		}
		vib->reg_vtg_ctl = val;
		val = vib->reg_en_ctl;
		val |= QPNP_VIB_EN;
		rc = qpnp_vib_write_u8(vib, &val, QPNP_VIB_EN_CTL(vib->base));
		if (rc < 0){
			VIB_LOG(KERN_ERR, "Fail to qpnp_vib_write_u8(QPNP_VIB_EN_CTL) rc=[%d] \n", rc);
			mutex_unlock(&vib->lock);
			VIB_DEBUG_LOG(KERN_INFO, "mutex_unlock \n");
			return rc;
		}
		vib->reg_en_ctl = val;
	} else {
        VIB_DEBUG_LOG(KERN_INFO, "VIB OFF.vib->reg_vtg_ctl=%d\n", vib->reg_vtg_ctl);
		val = vib->reg_en_ctl;
		val &= ~QPNP_VIB_EN;
		rc = qpnp_vib_write_u8(vib, &val, QPNP_VIB_EN_CTL(vib->base));
		if (rc < 0){
			VIB_LOG(KERN_ERR, "Fail to qpnp_vib_write_u8(QPNP_VIB_EN_CTL) rc=[%d] \n", rc);
			mutex_unlock(&vib->lock);
			VIB_DEBUG_LOG(KERN_INFO, "mutex_unlock \n");
			return rc;
		}
		vib->reg_en_ctl = val;
	}

    mutex_unlock(&vib->lock);
	VIB_DEBUG_LOG(KERN_INFO, "mutex_unlock \n");

    VIB_DEBUG_LOG(KERN_INFO, "end.rc=%d\n", rc);

	return rc;
}

static void qpnp_vib_enable(struct timed_output_dev *dev, int value)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib,
					 timed_dev);

    VIB_DEBUG_LOG(KERN_INFO, "called. value=%d\n", value);

	mutex_lock(&vib->lock);
	VIB_DEBUG_LOG(KERN_INFO, "mutex_lock \n");
	hrtimer_cancel(&vib->vib_timer);

	if (value == 0)
		vib->state = 0;
	else {
        if(value < 50) {
            VIB_DEBUG_LOG(KERN_INFO, "set value to 55.\n");
            value = 55;
        } else if(value < 55) {
            VIB_DEBUG_LOG(KERN_INFO, "add value 5.\n");
            value += 5;
        } else if(value < 60) {
            VIB_DEBUG_LOG(KERN_INFO, "add value 3.\n");
            value += 3;
        }
		vib->state = 1;
		hrtimer_start(&vib->vib_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	mutex_unlock(&vib->lock);
	VIB_DEBUG_LOG(KERN_INFO, "mutex_unlock \n");
	schedule_work(&vib->work);

    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
}

static void qpnp_vib_update(struct work_struct *work)
{
	struct qpnp_vib *vib = container_of(work, struct qpnp_vib,
					 work);
    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	qpnp_vib_set(vib, vib->state);
    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
}

static int qpnp_vib_get_time(struct timed_output_dev *dev)
{
	struct qpnp_vib *vib = container_of(dev, struct qpnp_vib,
							 timed_dev);

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
        VIB_DEBUG_LOG(KERN_INFO, "end ret=%d.\n",(int)ktime_to_us(r));
		return (int)ktime_to_us(r);
	} else {
        VIB_DEBUG_LOG(KERN_INFO, "end. ret:0\n");
		return 0;
    }
}

static enum hrtimer_restart qpnp_vib_timer_func(struct hrtimer *timer)
{
	struct qpnp_vib *vib = container_of(timer, struct qpnp_vib,
							 vib_timer);

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	vib->state = 0;
	schedule_work(&vib->work);

    VIB_DEBUG_LOG(KERN_INFO, "end. ret=%d\n", HRTIMER_NORESTART);
	return HRTIMER_NORESTART;
}

#ifdef CONFIG_PM
static int qpnp_vibrator_suspend(struct device *dev)
{
	struct qpnp_vib *vib = dev_get_drvdata(dev);

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	hrtimer_cancel(&vib->vib_timer);
	cancel_work_sync(&vib->work);
	/* turn-off vibrator */
	qpnp_vib_set(vib, 0);

    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(qpnp_vibrator_pm_ops, qpnp_vibrator_suspend, NULL);

static int __devinit qpnp_vibrator_probe(struct spmi_device *spmi)
{
	struct qpnp_vib *vib;
	struct resource *vib_resource;
	int rc;
	u8 val;
	u32 temp_val;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");

	vib = devm_kzalloc(&spmi->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)
    {
        VIB_LOG(KERN_ERR, "devm_kzalloc error:size=0x%x\n",sizeof(*vib));
		return -ENOMEM;
    }

	vib->spmi = spmi;

	vib->timeout = QPNP_VIB_DEFAULT_TIMEOUT;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-timeout-ms", &temp_val);
	if (!rc) {
        VIB_DEBUG_LOG(KERN_INFO, "vib->timeout=%d\n", temp_val);
		vib->timeout = temp_val;
	} else if (rc != EINVAL) {
        VIB_LOG(KERN_ERR, "Unable to read vib timeout\n");
		return rc;
	}

	vib->vtg_level = QPNP_VIB_DEFAULT_VTG_LVL;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-vtg-level-mV", &temp_val);
	if (!rc) {
        VIB_DEBUG_LOG(KERN_INFO, "vib->vtg_level=%d\n", temp_val);
		vib->vtg_level = temp_val;
	} else if (rc != -EINVAL) {
        VIB_LOG(KERN_ERR, "Unable to read vtg level\n");
		return rc;
	}

	vib->vtg_level /= 100;

	vib_resource = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!vib_resource) {
        VIB_LOG(KERN_ERR, "Unable to get vibrator base address\n");
		return -EINVAL;
	}
	vib->base = vib_resource->start;

	/* save the control registers values */
	rc = qpnp_vib_read_u8(vib, &val, QPNP_VIB_VTG_CTL(vib->base));
	if (rc < 0)
    {
        VIB_LOG(KERN_ERR, "QPNP_VIB_VTG_CTL value read failed.(%d)\n", rc);
		return rc;
    }
	vib->reg_vtg_ctl = val;

	rc = qpnp_vib_read_u8(vib, &val, QPNP_VIB_EN_CTL(vib->base));
	if (rc < 0)
    {
        VIB_LOG(KERN_ERR, "QPNP_VIB_EN_CTL value read failed.(%d)\n", rc);
		return rc;
    }
	vib->reg_en_ctl = val;

	mutex_init(&vib->lock);
	INIT_WORK(&vib->work, qpnp_vib_update);

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = qpnp_vib_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = qpnp_vib_get_time;
	vib->timed_dev.enable = qpnp_vib_enable;

	dev_set_drvdata(&spmi->dev, vib);

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
    {
        VIB_LOG(KERN_ERR, "timed_output_dev_register failed.(%d)\n", rc);
		return rc;
    }

	vib_dev = vib;

    VIB_DEBUG_LOG(KERN_INFO, "end. rc=%d\n", rc);
	return rc;
}

static int  __devexit qpnp_vibrator_remove(struct spmi_device *spmi)
{
	struct qpnp_vib *vib = dev_get_drvdata(&spmi->dev);

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");

	cancel_work_sync(&vib->work);
	hrtimer_cancel(&vib->vib_timer);
	timed_output_dev_unregister(&vib->timed_dev);
	mutex_destroy(&vib->lock);

    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "qcom,qpnp-vibrator",
	},
	{}
};

static struct spmi_driver qpnp_vibrator_driver = {
	.driver		= {
		.name	= "qcom,qpnp-vibrator",
		.of_match_table = spmi_match_table,
		.pm	= &qpnp_vibrator_pm_ops,
	},
	.probe		= qpnp_vibrator_probe,
	.remove		= __devexit_p(qpnp_vibrator_remove),
};

static int __init qpnp_vibrator_init(void)
{
    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	return spmi_driver_register(&qpnp_vibrator_driver);
}
module_init(qpnp_vibrator_init);

static void __exit qpnp_vibrator_exit(void)
{
    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
	return spmi_driver_unregister(&qpnp_vibrator_driver);
}
module_exit(qpnp_vibrator_exit);

MODULE_DESCRIPTION("qpnp vibrator driver");
MODULE_LICENSE("GPL v2");
