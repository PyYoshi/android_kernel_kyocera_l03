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

#ifndef __OEM_CHARGER_WIRELESS_H
#define __OEM_CHARGER_WIRELESS_H

typedef enum{
	STAGE_GET,
	STAGE_SET
} oem_wlc_stage_set_get;

typedef enum{
	STAGE_NOTHING,
	STAGE_RESTRICTION_FAKE_CHARGE,
	STAGE_RESTRICTION
} oem_wlc_stage;

oem_wlc_stage oem_wlc_stage_check(oem_wlc_stage_set_get, oem_wlc_stage);
void oem_wlc_set_wpc_en_n(unsigned int);
void oem_wlc_main(void);
void oem_wlc_init(void *);
void oem_wlc_wpc_en_ctrl_set_pri(int pri);
int oem_wlc_wpc_en_ctrl_get_pri(void);

#endif
