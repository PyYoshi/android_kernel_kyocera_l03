/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 *
 * drivers/video/msm/disp_ext_util.c
 *
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/msm_mdp.h>
#include "disp_ext.h"
#include "mdss_dsi.h"

static uint8_t refresh_flag = 0;
static uint8_t refresh_start_sw = 0;

static char disp_stat_chk_cmd[1] = {0x0A};
static struct dsi_cmd_desc disp_stat_chk_cmds = {
	{DTYPE_DCS_READ, 1, 0, 1, 0, sizeof(disp_stat_chk_cmd)},
	disp_stat_chk_cmd
};

void disp_ext_refresh_set_flag(uint8_t flag)
{
	refresh_flag = flag;
}

uint8_t disp_ext_refresh_get_flag(void)
{
	return refresh_flag;
}

void disp_ext_set_refresh_start_sw(uint8_t sw)
{
	refresh_start_sw = sw;
}

uint8_t disp_ext_get_refresh_start_sw(void)
{
	return refresh_start_sw;
}

static void disp_ext_refresh_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

int disp_ext_refresh_reg_read(struct mdss_panel_data *pdata)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	char rbuf[25];
	int ret;

	/* Rx command send */
	memset(&cmdreq, 0, sizeof(cmdreq));
	
	cmdreq.cmds = &disp_stat_chk_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 1;
	cmdreq.cb = NULL;
	cmdreq.rbuf = rbuf;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	ret = mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	if(ret) {
		pr_err("%s:dcs read failed\n", __func__);
		return -1;
	}

	DISP_LOCAL_LOG_EMERG("%s:read data is = %x\n", __func__,
										*(ctrl_pdata->rx_buf.data));

	if(*(ctrl_pdata->rx_buf.data) != 0x1C) {
		ret = -1;
	} else {
		ret = 0;
	}
	
	return ret;
}

int disp_ext_refresh_seq_exec(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mipi_panel_info *mipi;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	mipi  = &pdata->panel_info.mipi;

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	disp_ext_util_set_disp_state(LOCAL_DISPLAY_OFF);

	if (ctrl->off_cmds.cmd_cnt)
		disp_ext_refresh_panel_cmds_send(ctrl, &ctrl->off_cmds);

	/* HS -> LP */
	mdss_set_tx_power_mode(1,pdata);

	usleep_range(15000, 16000);

	gpio_set_value((ctrl->disp_en_gpio), 0);

	usleep_range(15000, 16000);

	if (ctrl->off_post_cmds.cmd_cnt)
		disp_ext_refresh_panel_cmds_send(ctrl, &ctrl->off_post_cmds);

	/* RST -> HI after initialize of mipi dsi */
	/* Move from mdss_dsi_panel_reset() */
	if (!gpio_is_valid(ctrl->rst_gpio)) {
		pr_err("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return -EINVAL;
	}
	gpio_set_value((ctrl->rst_gpio), 0);

	usleep_range(10000, 11000);

	gpio_set_value((ctrl->disp_en_gpio), 1);

	usleep_range(15000, 16000);

	gpio_set_value((ctrl->rst_gpio), 1);

	usleep_range(10000, 11000);

	if (ctrl->on_cmds.cmd_cnt)
		disp_ext_refresh_panel_cmds_send(ctrl, &ctrl->on_cmds);

	/* LP -> HS */
	mdss_set_tx_power_mode(0,pdata);

	if (ctrl->on_post_cmds.cmd_cnt)
		disp_ext_refresh_panel_cmds_send(ctrl, &ctrl->on_post_cmds);

	disp_ext_util_set_disp_state(LOCAL_DISPLAY_ON);
	disp_ext_set_refresh_start_sw(0);

	return 0;
}

void disp_ext_refresh_ctrl_start(struct mdss_panel_data *pdata)
{
	int ret = -1;

	ret = disp_ext_refresh_reg_read(pdata);

	if(ret) {
		pr_err("%s:refresh reg check failed\n", __func__);
		disp_ext_set_refresh_start_sw(1);

	} else {
		disp_ext_set_refresh_start_sw(0);
	}
}
