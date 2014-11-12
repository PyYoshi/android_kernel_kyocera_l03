/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
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
#define pr_fmt(fmt)	"OEM_BMS: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/power_supply.h>
#include <oem-charger_parm.h>
#include <oem-bms.h>

//#define FEATURE_OEM_BMS_DEBUG
#ifdef FEATURE_OEM_BMS_DEBUG
#define OEM_BMS_ERR       pr_err
#define OEM_BMS_DEBUG     pr_err
#else
#define OEM_BMS_ERR       pr_err
#define OEM_BMS_DEBUG     pr_debug
#endif

static int oem_bms_get_battery_prop(enum power_supply_property psp)
{
	union power_supply_propval ret = {0,};
	static struct power_supply *batatery_psy;

	if (batatery_psy == NULL)
		batatery_psy = power_supply_get_by_name("battery");
	if (batatery_psy) {
		/* if battery has been registered, use the status property */
		batatery_psy->get_property(batatery_psy, psp, &ret);
		return ret.intval;
	}

	/* Default to false if the battery power supply is not registered. */
	OEM_BMS_DEBUG("battery power supply is not registered\n");
	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static bool oem_bms_is_battery_charging(void)
{
	return oem_bms_get_battery_prop(POWER_SUPPLY_PROP_STATUS) == POWER_SUPPLY_STATUS_CHARGING;
}

static bool oem_bms_is_battery_full(void)
{
	return oem_bms_get_battery_prop(POWER_SUPPLY_PROP_STATUS) == POWER_SUPPLY_STATUS_FULL;
}

static int oem_bms_bound_soc_in_charging(int soc)
{
	soc = max(0, soc);
	soc = min(99, soc);
	return soc;
}

#define PERCENT_OF_AUTO_POWER_OFF	0
#define PERCENT_OF_LOW_BATTERY		1
#define AUTO_POWER_OFF_THRESHOLD	3300*1000
#define LOW_BATTERY_THRESHOLD		3350*1000
#define INIT_VBATT_MAX				4200*1000
static int save_vbatt = INIT_VBATT_MAX;

static int oem_bms_check_low_vol(int soc)
{
	int result_soc;

	if (save_vbatt <= AUTO_POWER_OFF_THRESHOLD) {
		result_soc = PERCENT_OF_AUTO_POWER_OFF;
	} else if (save_vbatt <= LOW_BATTERY_THRESHOLD
				&& !oem_bms_is_battery_charging()
				&& (soc <= 10)) {
		result_soc = PERCENT_OF_LOW_BATTERY;
	} else {
		result_soc = soc;
	}
	OEM_BMS_DEBUG("result soc = %d, save_vbatt = %d \n", result_soc, save_vbatt);
	return result_soc;
}

static int oem_real_last_bms_soc_backup;
int oem_bms_correct_soc(int in_soc)
{
	int result_soc = in_soc;

	oem_real_last_bms_soc_backup = in_soc;
	
	if(oem_bms_is_battery_charging()) {
		result_soc = oem_bms_bound_soc_in_charging(in_soc);
	}

	if(oem_bms_is_battery_full()) {
		result_soc = 100;
	} else if (oem_get_status_oem_after_warmcool_recharge_flag() == OEM_AFTER_WARMCOOL_RECHARGING) {
		result_soc = 100;
	}

	OEM_BMS_DEBUG("result soc = %d, in_soc = %d \n", result_soc, in_soc);

	result_soc = oem_bms_check_low_vol(result_soc);

	return result_soc;
}
EXPORT_SYMBOL(oem_bms_correct_soc);

int oem_get_real_last_bms_soc_backup(void)
{
	return oem_real_last_bms_soc_backup;
}
EXPORT_SYMBOL(oem_get_real_last_bms_soc_backup);


#define DETERIORATION_THRESH_GOOD_TO_NORMAL(x)	((x * 80)/ 100)
#define DETERIORATION_THRESH_NORMAL_TO_GOOD(x)	((x * 90)/ 100)
#define DETERIORATION_THRESH_NORMAL_TO_DEAD(x)	((x * 60)/ 100)
#define DETERIORATION_THRESH_DEAD_TO_NORMAL(x)	((x * 70)/ 100)

int oem_bms_get_deteriorationstatus(int *batt_deterioration_status, int new_fcc_mah, int default_fcc_mah)
{
	OEM_BMS_DEBUG("batt_deterioration_status[%d], new_fcc_mah[%d], default_fcc_mah[%d]\n",
					*batt_deterioration_status, new_fcc_mah, default_fcc_mah);
	switch( *batt_deterioration_status )
	{
		case BATT_DETERIORATION_GOOD:
			if (new_fcc_mah >= DETERIORATION_THRESH_GOOD_TO_NORMAL(default_fcc_mah)) {
				*batt_deterioration_status = BATT_DETERIORATION_GOOD;
			} else {
				*batt_deterioration_status = BATT_DETERIORATION_NORMAL;
			}
			break;
		case BATT_DETERIORATION_NORMAL:
			if (new_fcc_mah >= DETERIORATION_THRESH_NORMAL_TO_GOOD(default_fcc_mah)) {
				*batt_deterioration_status = BATT_DETERIORATION_GOOD;
			} else if (new_fcc_mah >= DETERIORATION_THRESH_NORMAL_TO_DEAD(default_fcc_mah)) {
				*batt_deterioration_status = BATT_DETERIORATION_NORMAL;
			} else {
				*batt_deterioration_status = BATT_DETERIORATION_DEAD;
			}
			break;
		case BATT_DETERIORATION_DEAD:
			if (new_fcc_mah >= DETERIORATION_THRESH_DEAD_TO_NORMAL(default_fcc_mah)) {
				*batt_deterioration_status = BATT_DETERIORATION_NORMAL;
			} else {
				*batt_deterioration_status = BATT_DETERIORATION_DEAD;
			}
			break;
		default:
			break;
	}
	oem_param_deterioration_status_backup(*batt_deterioration_status);

	return *batt_deterioration_status;
}

void oem_bms_low_vol_detect_active(int vbatt, int batt_temp, int init_state)
{
	pr_debug("in active vbatt = %d batt_temp = %d init_state = %d\n",
						vbatt, batt_temp, init_state);
	if (init_state) {
		pr_err("Uninitialized(vbatt batt_temp)\n");
		return;
	}
	save_vbatt = vbatt;

	return;
}
EXPORT_SYMBOL(oem_bms_low_vol_detect_active);
