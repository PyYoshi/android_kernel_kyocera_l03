/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 *
 * drivers/video/msm/disp_ext.h
 *
 * Copyright (c) 2008-2010, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#ifndef DISP_EXT_H
#define DISP_EXT_H
#include <linux/msm_mdp.h>
#include "mdss_fb.h"
#include "mdss_dsi.h"

#define FEATURE_DISP_LOCAL_LOG
/* #undef FEATURE_DISP_LOCAL_LOG */
extern uint8_t fb_dbg_msg_level;
#ifdef FEATURE_DISP_LOCAL_LOG
#define DISP_LOCAL_LOG_EMERG(msg, ...) if (fb_dbg_msg_level>0) printk(KERN_EMERG msg, ## __VA_ARGS__);
#else
#define DISP_LOCAL_LOG_EMERG(msg, ...) (void)0;
#endif

#define REG_ERR_CHECK 0x01
#define IMG_ERR_CHECK 0x02
#define MIPI_ERR_CHECK 0x04

void disp_ext_set_dmflag(int dmflag);
int disp_ext_is_invalid(void);
bool disp_ext_util_get_shutdown_pending(void);
void disp_ext_util_set_shutdown_pending(bool enable);

#ifdef CONFIG_DISP_EXT_BOARD
int disp_ext_board_detect_board(struct mdss_dsi_ctrl_pdata *ctrl);
int disp_ext_board_get_panel_detect(void);
void disp_ext_board_set_panel_detect(int flag);
#endif /* CONFIG_DISP_EXT_BOARD */

#ifdef CONFIG_DISP_EXT_DIAG
void disp_ext_diag_count_err_status(u32 ack_err_status);
void disp_ext_diag_set_ack_err_stat(u32 ack_err_status);
u8 disp_ext_diag_event_flag_check(void);
void disp_ext_diag_set_mipi_err_chk(bool flag);
int disp_ext_diag_reg_write(struct fb_info *info, unsigned int cmd, unsigned long arg);
int disp_ext_diag_reg_read(struct fb_info *info, unsigned int cmd, unsigned long arg);
int disp_ext_diag_tx_rate(struct fb_info *info, unsigned int cmd, unsigned long arg);
int disp_ext_diag_err_check_start(void);
int disp_ext_diag_err_check_stop(struct fb_info *info, unsigned int cmd, unsigned long arg);
int disp_ext_diag_current_err_stat(struct fb_info *info, unsigned int cmd, unsigned long arg);
int disp_ext_diag_img_transfer_sw(struct fb_info *info, unsigned int cmd);
void disp_ext_diag_init(void);

uint32_t disp_ext_util_get_crc_error(void);
void disp_ext_util_set_crc_error(uint32_t count);
void disp_ext_util_crc_countup(void);
#endif /* CONFIG_DISP_EXT_DIAG */

#ifdef CONFIG_DISP_EXT_PROPERTY
void disp_ext_util_set_kcjprop(struct mdss_panel_data *pdata, struct fb_kcjprop_data* kcjprop_data);
#endif /* CONFIG_DISP_EXT_PROPERTY */

#ifdef CONFIG_DISP_EXT_UTIL
local_disp_state_enum disp_ext_util_get_disp_state(void);
void disp_ext_util_set_disp_state(local_disp_state_enum state);
struct local_disp_state_type *disp_ext_util_get_disp_info(void);
void disp_ext_util_disp_local_init( void );
void disp_ext_util_disp_local_lock( void );
void disp_ext_util_disp_local_unlock( void );
void disp_ext_util_mipitx_lock( void );
void disp_ext_util_mipitx_unlock( void );
#ifdef CONFIG_DISP_EXT_UTIL_GET_RATE
u32 disp_ext_util_get_refresh_rate( void );
#endif /*CONFIG_DISP_EXT_UTIL_GET_RATE*/
#ifdef CONFIG_DISP_UTIL_DSI_CLK_OFF
void disp_ext_util_dsi_clk_off( void );
#endif /*CONFIG_DISP_UTIL_DSI_CLK_OFF*/
#ifdef CONFIG_DISP_EXT_UTIL_VSYNC
u32 disp_ext_util_get_total_line( void );
u32 disp_ext_util_get_vsync_count( void );
u32 disp_ext_util_vsync_cal_start( u32 start_y );
#endif /*CONFIG_DISP_EXT_UTIL_VSYNC*/
#endif /* CONFIG_DISP_EXT_UTIL */

#ifdef CONFIG_DISP_EXT_REFRESH
void disp_ext_refresh_set_flag(uint8_t flag);
uint8_t disp_ext_refresh_get_flag(void);
void disp_ext_set_refresh_start_sw(uint8_t sw);
uint8_t disp_ext_get_refresh_start_sw(void);
int disp_ext_refresh_reg_read(struct mdss_panel_data *pdata);
int disp_ext_refresh_seq_exec(struct mdss_panel_data *pdata);
void disp_ext_refresh_ctrl_start(struct mdss_panel_data *pdata);
#endif /* CONFIG_DISP_EXT_REFRESH */

#endif /* DISP_EXT_H */
