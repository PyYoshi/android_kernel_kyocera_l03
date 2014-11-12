#ifndef OEM_CHARGER_H
#define OEM_CHARGER_H
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

/**
 * oem_chargermonit_init - oem hkadc initialize
 *
 */
void oem_chargermonit_init(void *chip);

/**
 * oem_chargermonit_exit - oem hkadc exit
 *
 */
void oem_chargermonit_exit(void);

int oem_chg_buck_ctrl(void *_chip, int disable);
unsigned int oem_vddmax_get(void *_chip);
unsigned int oem_resume_delta_mv(void *_chip);
int oem_chg_get_prop_batt_temp(void *_chip);

bool oem_qpnp_chg_set_appropriate_vbatdet(void *_chip);
bool oem_is_chg_vbatt_ov(void);
void oem_chg_vbatt_ov_work_init(void);
void chg_vbatt_ov_worker(struct work_struct *work);
void oem_chg_vbatt_ov_check_charger(void);
void oem_chg_check_vbatt_ov(void);
int oem_chg_is_dc_ov_check(void *_chip);

void oem_uim_smem_init(void);
bool oem_is_uim_dete(void);

/**
 * oem_charger_get_connect_state - update connect state
 *
 */
int oem_charger_get_connect_state(void);

/**
 * oem_charger_pm_batt_power_get_property - get oem_charger property
 *
 */
int oem_charger_pm_batt_power_get_property(enum power_supply_property psp, int *intval);

/**
 * oem_chg_set_temporary_chg_param - set temporary ibatmax & vin min
 *
 */
void oem_chg_set_temporary_chg_param(void *_chip);

/**
 * oem_chg_set_appropriate_chg_param - set appropriate ibatmax & vin min
 *
 */
void oem_chg_set_appropriate_chg_param(void *_chip);

/**
 * oem_chg_vadc_read - set appropriate ibatmax & vin min
 *
 */
int oem_chg_vadc_read(enum qpnp_vadc_channels channel, struct qpnp_vadc_result *result);


/**
 * oem_chg_failed_clear - failed charging status clear
 *
 */
void oem_chg_failed_clear(void *_chip);

/**
 * oem_chg_set_vbat_det - set vbat det low
 *
 */
void oem_chg_set_vbat_det_low(void *_chip);

/**
 * oem_chg_set_uim_status_fix_parm - when uim status is fixed, set ibat_max & vdd_max
 *
 */
void oem_chg_set_uim_status_fix_parm(void *_chip);

/**
 * oem_chg_uim_status_check - uim status check in hkadc_monitor
 *
 */
void oem_chg_uim_status_check(void);

/**
 * oem_chg_hvdcp_max_voltage_ctl - HVDCP Max Voltage control
 *
 */
void oem_chg_hvdcp_max_voltage_ctl(int dc_present);
#endif

