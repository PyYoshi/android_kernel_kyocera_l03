/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 */

#include <linux/batterydata-lib.h>

static struct single_row_lut fcc_temp = {
	.x		= {-20, -10, 0, 10, 20, 35, 45, 60},
	.y		= {919, 1924, 2502, 2589, 3011, 3016, 2777, 2684},
	.cols	= 8
};

static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 29,
	.cols		= 8,
	.temp		= {-20, -10, 0, 10, 20, 35, 45, 60},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40,
					35, 30, 25, 20, 15, 10, 9, 8, 7, 6, 5,
					4, 3, 2, 1, 0},
	.ocv		= {
				{4049, 4132, 4170, 4197, 4270, 4293, 4224, 4220},
				{3615, 3896, 4040, 4081, 4201, 4230, 4170, 4169},
				{3567, 3814, 3995, 4039, 4148, 4176, 4122, 4122},
				{3540, 3766, 3940, 3990, 4098, 4124, 4075, 4077},
				{3520, 3727, 3892, 3938, 4051, 4074, 4032, 4034},
				{3505, 3694, 3860, 3912, 4002, 4031, 3991, 3994},
				{3493, 3666, 3829, 3878, 3952, 3978, 3954, 3956},
				{3481, 3640, 3798, 3841, 3922, 3945, 3915, 3915},
				{3470, 3616, 3773, 3814, 3876, 3903, 3869, 3873},
				{3460, 3593, 3753, 3794, 3839, 3856, 3840, 3846},
				{3450, 3572, 3737, 3777, 3815, 3829, 3818, 3823},
				{3440, 3551, 3724, 3763, 3794, 3807, 3799, 3804},
				{3430, 3532, 3713, 3751, 3777, 3789, 3783, 3787},
				{3419, 3513, 3701, 3742, 3763, 3773, 3769, 3770},
				{3409, 3495, 3685, 3732, 3751, 3759, 3750, 3744},
				{3398, 3474, 3664, 3717, 3740, 3742, 3730, 3722},
				{3388, 3453, 3636, 3695, 3722, 3724, 3712, 3702},
				{3378, 3430, 3605, 3664, 3692, 3697, 3683, 3672},
				{3369, 3406, 3573, 3647, 3665, 3671, 3665, 3656},
				{3367, 3401, 3566, 3644, 3663, 3670, 3664, 3655},
				{3365, 3396, 3556, 3640, 3661, 3668, 3662, 3654},
				{3363, 3391, 3546, 3635, 3658, 3667, 3661, 3652},
				{3361, 3385, 3533, 3628, 3655, 3664, 3658, 3649},
				{3360, 3380, 3518, 3616, 3648, 3659, 3651, 3640},
				{3358, 3374, 3500, 3595, 3633, 3643, 3627, 3613},
				{3356, 3369, 3476, 3561, 3596, 3601, 3584, 3571},
				{3354, 3363, 3445, 3513, 3537, 3547, 3528, 3520},
				{3352, 3357, 3405, 3449, 3464, 3466, 3453, 3450},
				{3350, 3350, 3350, 3350, 3350, 3350, 3350, 3350}
	}
};

static struct sf_lut rbatt_sf = {
	.rows		= 29,
	.cols		= 5,
	/* row_entries are temperature */
	.row_entries	= {-20, 0, 25, 40, 65},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40,
					35, 30, 25, 20, 15, 10, 9, 8, 7, 6, 5,
					4, 3, 2, 1, 0},
	.sf		= {
				{822, 316, 100, 72, 72},
				{822, 316, 100, 72, 72},
				{819, 316, 100, 75, 72},
				{822, 312, 100, 75, 73},
				{822, 316,  98, 73, 72},
				{818, 315,  98, 76, 72},
				{818, 318,  93, 72, 72},
				{818, 320,  96, 73, 75},
				{815, 321,  97, 75, 73},
				{814, 324,  99, 75, 75},
				{815, 326,  97, 72, 73},
				{814, 334,  98, 77, 72},
				{815, 339, 100, 73, 72},
				{814, 342,  99, 73, 73},
				{815, 352, 103, 76, 75},
				{817, 354, 103, 73, 75},
				{819, 361, 105, 75, 71},
				{819, 366, 105, 78, 75},
				{820, 370, 108, 78, 75},
				{816, 368, 108, 78, 76},
				{820, 370, 108, 75, 73},
				{820, 371, 108, 78, 72},
				{819, 372, 110, 75, 76},
				{818, 372, 109, 77, 72},
				{819, 372, 113, 77, 72},
				{819, 373, 111, 81, 73},
				{820, 374, 116, 82, 75},
				{820, 375, 116, 81, 73},
				{822, 377, 119, 82, 77},
	}
};

struct bms_battery_data oem_batt_data = {
	.fcc			= 3000,
	.fcc_temp_lut		= &fcc_temp,
	.pc_temp_ocv_lut	= &pc_temp_ocv,
	.rbatt_sf_lut		= &rbatt_sf,
	.default_rbatt_mohm	= 116,
	.flat_ocv_threshold_uv	= 3800000,
};
