#ifndef OEM_BMS_H
#define OEM_BMS_H
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

enum batt_deterioration_type {
	BATT_DETERIORATION_GOOD,
	BATT_DETERIORATION_NORMAL,
	BATT_DETERIORATION_DEAD,
};

/**
 * oem_get_status_oem_after_warmcool_recharge_flag - get status of oem_after_warmcool_recharge_flag
 *
 */
int oem_get_status_oem_after_warmcool_recharge_flag(void);

enum oem_after_warmcool_recharge_flag_type {
	OEM_AFTER_WARMCOOL_INIT = 0,
	OEM_AFTER_WARMCOOL_CHARGE_DONE_ON_NOT_NORMAL,
	OEM_AFTER_WARMCOOL_RECHARGING,
	OEM_AFTER_WARMCOOL_CHARGE_DONE,
};

/**
 * oem_bms_correct_soc - readjustment oem soc
 *
 */
int oem_bms_correct_soc(int in_soc);

/**
 * oem_get_real_last_bms_soc_backup - read real_last_bms_soc
 *
 */
int oem_get_real_last_bms_soc_backup(void);


/**
 * oem_bms_get_deteriorationstatus - battery cycle status
 *
 */
int oem_bms_get_deteriorationstatus(int *, int, int);

void oem_bms_low_vol_detect_active(int vbatt, int batt_temp, int init_state);

#endif

