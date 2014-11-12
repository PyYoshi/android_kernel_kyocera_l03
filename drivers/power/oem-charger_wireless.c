/* 
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"OEM_WLC %s: " fmt, __func__

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/qpnp/qpnp-adc.h>

#include <oem-charger_wlc_i2c.h>
#include <oem-charger_stand.h>
#include <oem-charger_wireless.h>
#include <oem-charger.h>
#include <oem-charger_parm.h>
#include <oem-hkadc.h>

#define CHG_WLC_ERR		pr_err
//#define FEATURE_CHG_WLC_DEBUG
#ifdef FEATURE_CHG_WLC_DEBUG
#define CHG_WLC_DEBUG	pr_err
#else
#define CHG_WLC_DEBUG	pr_debug
#endif

#define WLC_CHECK_PERIOD_MS	1000

#define WLC_I2C_BUSY_WAIT_MS	10
#define WLC_I2C_BUSY_GIVE_UP_COUNT	20

#define WLC_IDC_MAX_LOW		100
#define WLC_IDC_MAX_MIDDLE	400
#define WLC_IDC_MAX_HIGH	800
#define WLC_IDC_MAX_STAND	1800
#define WLC_IDC_MAX_DEFAULT	WLC_IDC_MAX_MIDDLE
#define WLC_IDC_MAX_STEP	100

#define WLC_GPIO_CHG_TBATT2	75

#define WPC_EN_N		83
#define WLC_USB_PATH_EN	132

#define WLC_IDC_MAX_VS_TEMP_LOW_TO_MIDDLE(x)		((x < 530)	? 1:0)
#define WLC_IDC_MAX_VS_TEMP_MIDDLE_TO_HIGH(x)		((40 <= x && x < 425)	? 1:0)
#define WLC_IDC_MAX_VS_TEMP_MIDDLE_TO_LOW(x)		((550 <= x)	? 1:0)
#define WLC_IDC_MAX_VS_TEMP_HIGH_TO_MIDDLE_COOL(x)	((x < 20)	? 1:0)
#define WLC_IDC_MAX_VS_TEMP_HIGH_TO_MIDDLE_WARM(x)	((445 <= x)	? 1:0)

#define DELTA_VDD_MAX_FULL		20

#define DRV_NAME "oem_wlc-driver"

typedef enum{
	OEM_DC_CONNECT_NOTHING,
	OEM_DC_CONNECT_WLC,
	OEM_DC_CONNECT_STAND,
	OEM_DC_CONNECT_WLC_FORCE
}oem_dc_connect_status;

struct oem_wlc_device {
	int gpio;
	int irq;
	struct work_struct irq_work;
};

static struct oem_wlc_device *oem_wlc_dev;

static void *the_chip;

static spinlock_t	oem_wlc_work_lock;
static spinlock_t	oem_wlc_stage_lock;

static struct delayed_work	wlc_work;
static struct delayed_work	wlc_i2c_2nd_work;
static struct delayed_work	wlc_i2c_3rd_work;

static struct power_supply *dc_psy = NULL;
static struct power_supply *wlc_psy = NULL;

static atomic_t oem_wlc_wpc_en_pri = ATOMIC_INIT(0);

static
int oem_get_prop_dc_online(void)
{
	union power_supply_propval ret = {0,};

	if (!dc_psy) {
		dc_psy = power_supply_get_by_name("qpnp-dc");
	}
	if (dc_psy) {
		dc_psy->get_property(dc_psy, POWER_SUPPLY_PROP_ONLINE, &ret);
	}
	else {
		CHG_WLC_ERR("qpnp-dc power supply is not registered\n");
	}
	return ret.intval;
}

static
void oem_set_prop_current_max(int idcmax_mA)
{
	union power_supply_propval ret = {idcmax_mA * 1000,};

	if (!dc_psy) {
		dc_psy = power_supply_get_by_name("qpnp-dc");
	}
	if (dc_psy) {
		dc_psy->set_property(dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
	}
	else {
		CHG_WLC_ERR("qpnp-dc power supply is not registered\n");
	}
}

static
int oem_get_prop_current_max(void)
{
	union power_supply_propval ret = {0,};

	if (!dc_psy) {
		dc_psy = power_supply_get_by_name("qpnp-dc");
	}
	if (dc_psy) {
		dc_psy->get_property(dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
	}
	else {
		CHG_WLC_ERR("qpnp-dc power supply is not registered\n");
	}
	return (ret.intval / 1000);
}

static
int oem_wlc_change_idcmax_wlc(oem_dc_connect_status status, int current_idcmax_mA)
{
	int batt_temp = 0;

	int voltage_now_mv = 0;
	unsigned int vdd_max_mv = 0;
	unsigned int resume_delta_mv = 0;

	int idcmax_mA = 0;
	static int target_idcmax_mA = WLC_IDC_MAX_DEFAULT;

	if(status != OEM_DC_CONNECT_WLC) {
		target_idcmax_mA = WLC_IDC_MAX_DEFAULT;
		idcmax_mA =  WLC_IDC_MAX_DEFAULT;
	}
	else {
		batt_temp = oem_chg_get_prop_batt_temp(the_chip);
		voltage_now_mv = oem_get_vbatt_value() / 1000;
		vdd_max_mv = oem_vddmax_get(the_chip);
		resume_delta_mv = oem_resume_delta_mv(the_chip);

		switch(target_idcmax_mA) {
		case WLC_IDC_MAX_LOW:
			if(WLC_IDC_MAX_VS_TEMP_LOW_TO_MIDDLE(batt_temp)) {
				target_idcmax_mA = WLC_IDC_MAX_MIDDLE;
			}
			break;

		case WLC_IDC_MAX_MIDDLE:
			if(WLC_IDC_MAX_VS_TEMP_MIDDLE_TO_HIGH(batt_temp)
				&& (voltage_now_mv < (vdd_max_mv - resume_delta_mv))) {
				target_idcmax_mA = WLC_IDC_MAX_HIGH;
			}
			else if(WLC_IDC_MAX_VS_TEMP_MIDDLE_TO_LOW(batt_temp)) {
				target_idcmax_mA = WLC_IDC_MAX_LOW;
			}
			break;

		case WLC_IDC_MAX_HIGH:
			if(WLC_IDC_MAX_VS_TEMP_HIGH_TO_MIDDLE_COOL(batt_temp)
				|| WLC_IDC_MAX_VS_TEMP_HIGH_TO_MIDDLE_WARM(batt_temp)
				|| ((vdd_max_mv - DELTA_VDD_MAX_FULL) <= voltage_now_mv)) {
				target_idcmax_mA = WLC_IDC_MAX_MIDDLE;
			}
			break;
		default:
			target_idcmax_mA = WLC_IDC_MAX_DEFAULT;
			break;
		}

		if(target_idcmax_mA < current_idcmax_mA) {
			idcmax_mA = current_idcmax_mA - WLC_IDC_MAX_STEP;
		}
		else if(current_idcmax_mA < target_idcmax_mA) {
			idcmax_mA = current_idcmax_mA + WLC_IDC_MAX_STEP;
		}
		else {
			idcmax_mA = current_idcmax_mA;
		}
	}
	if(current_idcmax_mA != idcmax_mA) {
		CHG_WLC_DEBUG("batt_temp[%d], voltage_now[%d], vdd_max[%d], delta[%d], target_idcmax[%d]\n"
						, batt_temp, voltage_now_mv, vdd_max_mv, resume_delta_mv, target_idcmax_mA);
	}
	return idcmax_mA;
}

static
void oem_wlc_change_idcmax(oem_dc_connect_status status)
{
	int idcmax_mA = 0;
	int current_idcmax_mA = oem_get_prop_current_max();

	idcmax_mA =  oem_wlc_change_idcmax_wlc(status, current_idcmax_mA);

	switch(status) {
	case OEM_DC_CONNECT_NOTHING:
		idcmax_mA =  WLC_IDC_MAX_DEFAULT;
		break;

	case OEM_DC_CONNECT_STAND:
		idcmax_mA =  WLC_IDC_MAX_STAND;
		break;

	case OEM_DC_CONNECT_WLC:
	case OEM_DC_CONNECT_WLC_FORCE:
		break;
	}
	if(current_idcmax_mA != idcmax_mA) {
		CHG_WLC_DEBUG("status[%d], idcmax_mA[%d], current_idcmax_mA[%d]\n"
						, status, idcmax_mA, current_idcmax_mA);
		oem_set_prop_current_max(idcmax_mA);
	}
	return;
}

void oem_wlc_wpc_en_ctrl_set_pri(int pri)
{
	atomic_set(&oem_wlc_wpc_en_pri, pri);
}

int oem_wlc_wpc_en_ctrl_get_pri(void)
{
	return atomic_read(&oem_wlc_wpc_en_pri);
}

void oem_wlc_set_wpc_en_n(unsigned int value)
{
	static unsigned int hold_value = 0;
	int pri = 0;

	pri = atomic_read(&oem_wlc_wpc_en_pri);

	if(hold_value != value && pri == 0) {
		CHG_WLC_DEBUG("WPC_EN_N Set = %d \n", value);
		gpio_set_value(WPC_EN_N, value);
		hold_value = value;
	}
}

oem_wlc_stage oem_wlc_stage_check(oem_wlc_stage_set_get judge, oem_wlc_stage current_stage)
{
	unsigned long flags;
	static oem_wlc_stage hold_stage = STAGE_NOTHING;

	spin_lock_irqsave(&oem_wlc_stage_lock, flags);
	if(judge == STAGE_SET) {
		if(hold_stage != current_stage) {
			CHG_WLC_DEBUG("hold_stage[%d], current_stage[%d]\n",
								hold_stage, current_stage);

			switch(current_stage) {
			case STAGE_NOTHING:
				if(hold_stage == STAGE_RESTRICTION_FAKE_CHARGE) {
					oem_chg_buck_ctrl(the_chip, 0);
				}
				break;

			case STAGE_RESTRICTION_FAKE_CHARGE:
				oem_chg_buck_ctrl(the_chip, 1);
				break;

			case STAGE_RESTRICTION:
				oem_chg_buck_ctrl(the_chip, 0);
				break;
			}
			hold_stage = current_stage;
		}
	}
	spin_unlock_irqrestore(&oem_wlc_stage_lock, flags);
	return hold_stage;
}

static
void __oem_wlc_main(int from_schedule)
{
	unsigned long flags;
	int dc_present = 0;
	int stand_detect = 0;
	static unsigned int isworking = 0;
	static unsigned int hold_log_judge = 0;
	unsigned int log_judge;

	if(from_schedule) {
		dc_present = oem_get_prop_dc_online();
		stand_detect = oem_chg_get_oem_stand_detect();

		if(dc_present && !stand_detect) {
			if(oem_wlc_stage_check(STAGE_GET, STAGE_NOTHING) == STAGE_NOTHING) {
				oem_wlc_change_idcmax(OEM_DC_CONNECT_WLC);
				oem_wlc_set_wpc_en_n(0);
			}
			else {
				oem_wlc_change_idcmax(OEM_DC_CONNECT_WLC_FORCE);
				oem_wlc_set_wpc_en_n(0);
			}
		}
		else {
			oem_wlc_stage_check(STAGE_SET, STAGE_NOTHING);

			if(stand_detect == 1) {
				oem_wlc_set_wpc_en_n(1);
				oem_wlc_change_idcmax(OEM_DC_CONNECT_STAND);
			}
			else {
				oem_wlc_set_wpc_en_n(0);
				oem_wlc_change_idcmax(OEM_DC_CONNECT_NOTHING);
			}
		}
	}
	log_judge = (((0x01 & dc_present) << 3) |
					((0x01 & stand_detect) << 2) |
					((0x01 & from_schedule) << 1) |
					(0x01 & isworking));
	if(hold_log_judge != log_judge) {
		CHG_WLC_DEBUG("dc:%d, stand:%d, from_sched:%d, isworking:%d\n",
						dc_present, stand_detect, from_schedule, isworking);
		hold_log_judge = log_judge;
	}
	spin_lock_irqsave(&oem_wlc_work_lock, flags);
	if(from_schedule) {
		isworking = 0;
		if(dc_present || stand_detect) {
			schedule_delayed_work(&wlc_work,
									msecs_to_jiffies(WLC_CHECK_PERIOD_MS));
			isworking = 1;
		}
	}
	else {
		if(isworking == 0) {
			schedule_delayed_work(&wlc_work, 0);
			isworking = 1;
		}
	}
	spin_unlock_irqrestore(&oem_wlc_work_lock, flags);
}

static
void oem_wlc_check_work(struct work_struct *work)
{
	__oem_wlc_main(1);
}

void oem_wlc_main(void)
{
	__oem_wlc_main(0);

	if (!wlc_psy) {
		wlc_psy = power_supply_get_by_name("wlc");
	}
	if (wlc_psy) {
		power_supply_changed(wlc_psy);
	}
	else {
		CHG_WLC_ERR("qpnp-dc power supply is not registered\n");
	}
}

static
void oem_wlc_init_gpio(void)
{
	if (gpio_request(WPC_EN_N, "oem_wlc-gpio")) {
		CHG_WLC_ERR("gpio_request[%d] failed.\n", WPC_EN_N);
	}
	gpio_tlmm_config(GPIO_CFG
						(WPC_EN_N,
						0,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
	gpio_set_value(WPC_EN_N, 0);

	if (gpio_request(WLC_USB_PATH_EN, "oem_wlc-gpio")) {
		CHG_WLC_ERR("gpio_request[%d] failed.\n", WLC_USB_PATH_EN);
	}
	gpio_tlmm_config(GPIO_CFG
						(WLC_USB_PATH_EN,
						0,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
	gpio_set_value(WLC_USB_PATH_EN, 0);
}

static void oem_wlc_i2c_3rd_work(struct work_struct *work)
{
	CHG_WLC_DEBUG("called\n");

	oem_wlc_stage_check(STAGE_SET, STAGE_NOTHING);
}

static void oem_wlc_i2c_2nd_work(struct work_struct *work)
{
	int gpio_value = gpio_get_value(oem_wlc_dev->gpio);
	int chg_tbatt2_value = gpio_get_value(WLC_GPIO_CHG_TBATT2);
	CHG_WLC_DEBUG("called GPIO : %s, TBATT2 : %s\n",
					gpio_value ? "H" : "L", chg_tbatt2_value ? "H" : "L");

	if(delayed_work_pending(&wlc_i2c_3rd_work)) {
		cancel_delayed_work_sync(&wlc_i2c_3rd_work);
	}
	if(gpio_value && chg_tbatt2_value) {
		oem_wlc_stage_check(STAGE_SET, STAGE_RESTRICTION);
		oem_chg_wlc_i2c_write_2nd();
		schedule_delayed_work(&wlc_i2c_3rd_work,
					msecs_to_jiffies(oem_param_charger.idc_wait_time));
	}
}

static void oem_wlc_isr_work(struct work_struct *work)
{
	int gpio_value = gpio_get_value(oem_wlc_dev->gpio);
	int chg_tbatt2_value = gpio_get_value(WLC_GPIO_CHG_TBATT2);
	int rc;
	static unsigned int retry_count = 0;
	CHG_WLC_DEBUG("called GPIO : %s, TBATT2 : %s, retry_count[%d]\n",
					gpio_value ? "H" : "L", chg_tbatt2_value ? "H" : "L",
					retry_count);

	if(delayed_work_pending(&wlc_i2c_3rd_work)) {
		cancel_delayed_work_sync(&wlc_i2c_3rd_work);
	}
	if(delayed_work_pending(&wlc_i2c_2nd_work)) {
		cancel_delayed_work_sync(&wlc_i2c_2nd_work);
	}
	if(gpio_value && chg_tbatt2_value) {
		rc = oem_chg_wlc_i2c_write_1st();
		if(rc == -EBUSY && retry_count < WLC_I2C_BUSY_GIVE_UP_COUNT) {
			msleep(WLC_I2C_BUSY_WAIT_MS);
			schedule_work(&oem_wlc_dev->irq_work);
			retry_count++;
		}
		else {
			schedule_delayed_work(&wlc_i2c_2nd_work,
					msecs_to_jiffies(oem_param_charger.buck_ctrl_wait_time));
			retry_count = 0;
		}
	}
	else {
		retry_count = 0;
	}
}

static irqreturn_t oem_wlc_isr(int irq, void *dev)
{
	struct oem_wlc_device *oem_wlc_dev = dev;
	int gpio_value = gpio_get_value(oem_wlc_dev->gpio);
	int chg_tbatt2_value = gpio_get_value(WLC_GPIO_CHG_TBATT2);
	CHG_WLC_DEBUG("called GPIO : %s, TBATT2 : %s\n",
					gpio_value ? "H" : "L", chg_tbatt2_value ? "H" : "L");
	if(gpio_value && chg_tbatt2_value) {
		oem_wlc_stage_check(STAGE_SET, STAGE_RESTRICTION_FAKE_CHARGE);

		schedule_work(&oem_wlc_dev->irq_work);
	}
	else {
		oem_wlc_stage_check(STAGE_SET, STAGE_NOTHING);
	}
	return IRQ_HANDLED;
}

static __devinit int oem_wlc_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (!pdev->dev.of_node) {
		CHG_WLC_ERR("No platform supplied from device tree.\n");
		rc = -EINVAL;
		goto err_arg;
	}

	oem_wlc_dev = kzalloc(sizeof(struct oem_wlc_device), GFP_KERNEL);
	if (!oem_wlc_dev) {
		CHG_WLC_ERR("kzalloc fail\n");
		rc = -ENOMEM;
		goto err_alloc;
	}

	oem_wlc_dev->gpio = of_get_named_gpio(pdev->dev.of_node, "oem_wlc-gpio", 0);
	if (oem_wlc_dev->gpio < 0) {
		CHG_WLC_ERR("of_get_named_gpio failed.\n");
		rc = -EINVAL;
		goto err_gpio;
	}

	rc = gpio_request(oem_wlc_dev->gpio, "oem_wlc-gpio");
	if (rc) {
		CHG_WLC_ERR("gpio_request failed.\n");
		goto err_gpio;
	}

	oem_wlc_dev->irq = gpio_to_irq(oem_wlc_dev->gpio);

	INIT_WORK(&oem_wlc_dev->irq_work, oem_wlc_isr_work);

	rc = request_irq(oem_wlc_dev->irq, oem_wlc_isr,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
							"oem_wlc-irq", oem_wlc_dev);
	if (rc) {
		CHG_WLC_ERR("failed request_irq.\n");
		goto err_irq;
	}

	enable_irq_wake(oem_wlc_dev->irq);

	return 0;


err_irq:
	gpio_free(oem_wlc_dev->gpio);

err_gpio:
	kfree(oem_wlc_dev);

err_alloc:
err_arg:
	CHG_WLC_ERR("failed.\n");

	return rc;
}

static int __devexit oem_wlc_remove(struct platform_device *pdev)
{
	free_irq(oem_wlc_dev->irq, oem_wlc_dev);
	gpio_free(oem_wlc_dev->gpio);
	kfree(oem_wlc_dev);

	return 0;
}

static const struct of_device_id oem_wlc_of_match[] = {
	{ .compatible = "kc,oem_wlc-driver", },
};

static struct platform_driver oem_wlc_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = oem_wlc_of_match,
	},
	.probe = oem_wlc_probe,
	.remove = __devexit_p(oem_wlc_remove),
};

void oem_wlc_init(void *chip)
{
	the_chip = chip;

	spin_lock_init(&oem_wlc_work_lock);
	spin_lock_init(&oem_wlc_stage_lock);

	INIT_DELAYED_WORK(&wlc_work, oem_wlc_check_work);
	INIT_DELAYED_WORK(&wlc_i2c_2nd_work, oem_wlc_i2c_2nd_work);
	INIT_DELAYED_WORK(&wlc_i2c_3rd_work, oem_wlc_i2c_3rd_work);

	oem_wlc_init_gpio();

	platform_driver_register(&oem_wlc_driver);
}

void oem_wlc_exit(void)
{
	platform_driver_unregister(&oem_wlc_driver);
}
