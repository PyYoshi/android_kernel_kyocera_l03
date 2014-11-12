/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 */
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
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <linux/mfd/wcd9xxx/core.h>
#include <linux/slimbus/slimbus.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <sound/soc.h>
#include <sound/jack.h>

#include "wcd9xxx-mbhc.h"
#include "wcd9xxx-resmgr.h"
#include "wcd9xxx-gpio.h"
#include <linux/key_dm_driver.h>

#ifdef USEGPIO
#define TABLA_HS_DET_READ_CNT 4
#define TABLA_HS_PLUG_TYPE_CNT 4
#define TABLA_HS_POLLING_CNT 2

bool g_gpio_sw_polling_flg = false;

#define EXT_ACQUIRE_LOCK(x) do { \
	mutex_lock_nested(&x, SINGLE_DEPTH_NESTING); \
} while (0)
#define EXT_RELEASE_LOCK(x) do { mutex_unlock(&x); } while (0)

#define WCD9XXX_GPIO_JACK_MASK (SND_JACK_HEADSET | SND_JACK_OC_HPHL | \
			   SND_JACK_OC_HPHR | SND_JACK_LINEOUT | \
			   SND_JACK_UNSUPPORTED)

static struct gpio_hs_det_proc_state* gpio_set_hs_det_proc_state(struct wcd9xxx_mbhc *mbhc, struct gpio_hs_det_proc_state *state);
static void gpio_jack_detect_push_state(struct wcd9xxx_mbhc *mbhc, struct gpio_hs_det_proc_state *state);
static struct gpio_hs_det_proc_state* gpio_jack_detect_pop_state(struct wcd9xxx_mbhc *mbhc);
static void gpio_jack_detect_on_timer(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_det_proc_state_init_enter(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_det_proc_state_polling_enter(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_det_proc_state_polling_on_timeout(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_det_proc_state_remove_enter(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_det_proc_state_remove_on_gpio_irq(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_det_proc_state_insert_enter(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_det_proc_state_insert_exit(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_det_proc_state_insert_on_gpio_irq(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_btn_det_proc_state_init_on_gpio_irq(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_btn_det_proc_state_init_on_timeout(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_btn_det_proc_state_3pinouts_on_gpio_irq(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_btn_det_proc_state_3pinouts_on_timeout(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_btn_det_proc_state_4pinouts_enter(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_btn_det_proc_state_4pinouts_on_gpio_irq(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_btn_det_proc_state_4pinouts_on_sw_gpio_irq(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_btn_det_proc_state_sw_polling_enter(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_btn_det_proc_state_sw_polling_on_gpio_irq(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_btn_det_proc_state_sw_polling_on_timeout(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_det_set_timer(struct wcd9xxx_mbhc *mbhc);
static enum hrtimer_restart gpio_hs_det_timer_cb(struct hrtimer *hrtimer);
static enum hrtimer_restart gpio_hs_plug_type_rejudge_timer_cb(struct hrtimer *hrtimer);
static void gpio_hs_det_proc_work(struct work_struct *work);
static int gpio_hs_sw_gpio_get_value(struct wcd9xxx_mbhc *mbhc);
static void gpio_hs_polling_off(struct wcd9xxx_mbhc *mbhc);

static struct gpio_hs_det_proc_state hs_det_proc_state_init =
{
	.state_enter	= gpio_hs_det_proc_state_init_enter,
	.state_exit		= NULL,
	.on_gpio_irq	= NULL,
	.on_sw_gpio_irq	= NULL,
	.on_timeout		= NULL,
};

static struct gpio_hs_det_proc_state hs_det_proc_state_polling =
{
	.state_enter	= gpio_hs_det_proc_state_polling_enter,
	.state_exit		= NULL,
	.on_gpio_irq	= NULL,
	.on_sw_gpio_irq	= NULL,
	.on_timeout		= gpio_hs_det_proc_state_polling_on_timeout,
};

static struct gpio_hs_det_proc_state hs_det_proc_state_remove =
{
	.state_enter	= gpio_hs_det_proc_state_remove_enter,
	.state_exit		= NULL,
	.on_gpio_irq	= gpio_hs_det_proc_state_remove_on_gpio_irq,
	.on_sw_gpio_irq	= NULL,
	.on_timeout		= NULL,
};

static struct gpio_hs_det_proc_state hs_det_proc_state_insert =
{
	.state_enter	= gpio_hs_det_proc_state_insert_enter,
	.state_exit		= gpio_hs_det_proc_state_insert_exit,
	.on_gpio_irq	= gpio_hs_det_proc_state_insert_on_gpio_irq,
	.on_sw_gpio_irq	= NULL,
	.on_timeout		= NULL,
};

static struct gpio_hs_det_proc_state hs_btn_det_proc_state_init =
{
	.state_enter	= NULL,
	.state_exit		= NULL,
	.on_gpio_irq	= gpio_hs_btn_det_proc_state_init_on_gpio_irq,
	.on_sw_gpio_irq	= NULL,
	.on_timeout		= gpio_hs_btn_det_proc_state_init_on_timeout,
};

static struct gpio_hs_det_proc_state hs_btn_det_proc_state_3pinouts =
{
	.state_enter	= NULL,
	.state_exit		= NULL,
	.on_gpio_irq	= gpio_hs_btn_det_proc_state_3pinouts_on_gpio_irq,
	.on_sw_gpio_irq	= NULL,
	.on_timeout		= gpio_hs_btn_det_proc_state_3pinouts_on_timeout,
};

static struct gpio_hs_det_proc_state hs_btn_det_proc_state_4pinouts =
{
	.state_enter	= gpio_hs_btn_det_proc_state_4pinouts_enter,
	.state_exit		= NULL,
	.on_gpio_irq	= gpio_hs_btn_det_proc_state_4pinouts_on_gpio_irq,
	.on_sw_gpio_irq	= gpio_hs_btn_det_proc_state_4pinouts_on_sw_gpio_irq,
	.on_timeout		= NULL,
};

static struct gpio_hs_det_proc_state hs_btn_det_proc_state_sw_polling =
{
	.state_enter	= gpio_hs_btn_det_proc_state_sw_polling_enter,
	.state_exit		= NULL,
	.on_gpio_irq	= gpio_hs_btn_det_proc_state_sw_polling_on_gpio_irq,
	.on_sw_gpio_irq	= NULL,
	.on_timeout		= gpio_hs_btn_det_proc_state_sw_polling_on_timeout,
};

void gpio_set_hs_det_proc_state_init( 
				struct wcd9xxx_mbhc *mbhc)
{
	gpio_set_hs_det_proc_state(mbhc, &hs_det_proc_state_init);
}

bool gpio_sw_polling_state( void )
{
	return g_gpio_sw_polling_flg;
}

static struct gpio_hs_det_proc_state* gpio_set_hs_det_proc_state( 
				struct wcd9xxx_mbhc *mbhc, struct gpio_hs_det_proc_state *state)
{
	struct gpio_hs_det_proc_state *last = mbhc->hs_det_proc_state;

	mbhc->hs_det_proc_state = state;
	if(last != NULL && state != NULL)
	{
		state->next = last->next;
	}

	if(last != NULL && last->state_exit != NULL)
	{
		last->state_exit(mbhc);
	}

	if(state != NULL && state->state_enter != NULL)
	{
		state->state_enter(mbhc);
	}
	return last;
}

static void gpio_jack_detect_push_state(struct wcd9xxx_mbhc *mbhc,
				struct gpio_hs_det_proc_state *state)
{

	if(state != NULL) {
		state->next = mbhc->hs_det_proc_state;
		mbhc->hs_det_proc_state = state;

		if(state->state_enter != NULL)
			state->state_enter(mbhc);
	}
}

static struct gpio_hs_det_proc_state* gpio_jack_detect_pop_state(struct wcd9xxx_mbhc *mbhc)
{
	struct gpio_hs_det_proc_state *head = mbhc->hs_det_proc_state;

	if(head != NULL)
	{
		mbhc->hs_det_proc_state = head->next;

		if(head->state_exit != NULL)
			head->state_exit(mbhc);
	}
	return head;
}

irqreturn_t gpio_jack_detect_on_gpio_irq(int irq, void *data)
{
	struct wcd9xxx_mbhc *mbhc = data;

	struct gpio_hs_det_proc_state *state;
	struct gpio_hs_det_proc_state *next;
	unsigned long flags;

    if (mbhc->mbhc_cfg->gpio_irq != irq)
    {
        return IRQ_NONE;
    }

	spin_lock_irqsave(&mbhc->hs_spinlock,flags);

	for(state = mbhc->hs_det_proc_state; state != NULL; state = next) {
		next = state->next;
		if(state->on_gpio_irq != NULL)
			state->on_gpio_irq(mbhc);
	}

	spin_unlock_irqrestore(&mbhc->hs_spinlock,flags);

    return IRQ_HANDLED;
}

irqreturn_t gpio_btn_detect_on_gpio_irq(int irq, void *data)
{
	struct wcd9xxx_mbhc *mbhc = data;

	struct gpio_hs_det_proc_state *state;
	struct gpio_hs_det_proc_state *next;
	unsigned long flags;

    if (mbhc->mbhc_cfg->sw_gpio_irq != irq)
    {
        return IRQ_NONE;
    }

	spin_lock_irqsave(&mbhc->hs_spinlock,flags);

	for(state = mbhc->hs_det_proc_state; state != NULL; state = next) {
		next = state->next;
		if(state->on_sw_gpio_irq != NULL)
			state->on_sw_gpio_irq(mbhc);
	}

	spin_unlock_irqrestore(&mbhc->hs_spinlock,flags);

    return IRQ_HANDLED;
}

static void gpio_jack_detect_on_timer(struct wcd9xxx_mbhc *mbhc)
{
	struct gpio_hs_det_proc_state *state;
	struct gpio_hs_det_proc_state *next;

	EXT_ACQUIRE_LOCK(mbhc->codec_resource_lock);

	disable_irq_wake(mbhc->mbhc_cfg->gpio_irq);
	disable_irq_nosync(mbhc->mbhc_cfg->gpio_irq);

	for(state = mbhc->hs_det_proc_state; state != NULL; state = next) {
		next = state->next;
		if(state->on_timeout != NULL)
			state->on_timeout(mbhc);
	}

	enable_irq(mbhc->mbhc_cfg->gpio_irq);
	enable_irq_wake(mbhc->mbhc_cfg->gpio_irq);

	EXT_RELEASE_LOCK(mbhc->codec_resource_lock);

}

static void gpio_hs_det_proc_state_init_enter(struct wcd9xxx_mbhc *mbhc)
{

    mbhc->hs_det_gpio_read_cnt	= 0;
    mbhc->hs_sw_gpio_read_cnt	= 0;
    mbhc->hs_btn_press_flg		= false;
    mbhc->hs_jack_return_flg	= false;
    mbhc->hs_det_state			= HS_DET_STATE_REMOVE;
    mbhc->polling_off_state		= false;

	hrtimer_init(&mbhc->hs_det_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mbhc->hs_det_timer.function = gpio_hs_det_timer_cb;
	hrtimer_init(&mbhc->hs_plug_type_rejudge_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	mbhc->hs_plug_type_rejudge_timer.function = gpio_hs_plug_type_rejudge_timer_cb;

	INIT_WORK(&mbhc->hs_det_proc_work,gpio_hs_det_proc_work);

	gpio_set_hs_det_proc_state(mbhc, &hs_det_proc_state_polling);
}

static void gpio_hs_det_proc_state_polling_enter(struct wcd9xxx_mbhc *mbhc)
{
	wake_lock(&mbhc->hs_det_wlock);
	gpio_hs_det_set_timer(mbhc);
}

static void gpio_hs_det_proc_state_polling_on_timeout(struct wcd9xxx_mbhc *mbhc)
{
    static enum gpio_hs_det_state	 last_state = HS_DET_STATE_REMOVE;
    enum gpio_hs_det_state	 next_state;

	if( true == mbhc->polling_off_state )
	{
		gpio_hs_polling_off(mbhc);
		mbhc->polling_off_state = false;
	}

	if(!gpio_get_value(mbhc->mbhc_cfg->gpio))
	{
		next_state = HS_DET_STATE_INSERT;
	}
	else
	{
		next_state = HS_DET_STATE_REMOVE;
	}

    if(last_state == next_state)
    {
		mbhc->hs_det_gpio_read_cnt++;
    }
    else
    {
		last_state = next_state;
		mbhc->hs_det_gpio_read_cnt = 1;
	}

#if 0
    if(mbhc->hs_det_gpio_read_cnt == TABLA_HS_DET_READ_CNT)
#else
    if(mbhc->hs_det_gpio_read_cnt >= TABLA_HS_DET_READ_CNT)
#endif
	{
		if(next_state == HS_DET_STATE_INSERT)
		{
			gpio_set_hs_det_proc_state(mbhc, &hs_det_proc_state_insert);
		}
		else
		{
			gpio_set_hs_det_proc_state(mbhc, &hs_det_proc_state_remove);
			wake_unlock(&mbhc->hs_det_wlock);
		}
		
		mbhc->hs_det_gpio_read_cnt = 0;
	}
	else
	{
		gpio_hs_det_set_timer(mbhc);
	}
}

static void gpio_hs_det_proc_state_remove_enter(struct wcd9xxx_mbhc *mbhc)
{
	if(mbhc->hs_det_state == HS_DET_STATE_REMOVE)
	{
		return;
	}

	key_dm_driver_set_port(0x12);
	key_dm_driver_set_port(0x10);

	snd_soc_jack_report_no_dapm(&mbhc->headset_jack, 0, WCD9XXX_GPIO_JACK_MASK);
	snd_soc_jack_report_no_dapm(&mbhc->button_jack, 0, SND_JACK_BTN_0);

    mbhc->hs_sw_gpio_read_cnt	= 0;
    mbhc->hs_btn_press_flg		= false;
	mbhc->hs_det_state 		= HS_DET_STATE_REMOVE;
}

static void gpio_hs_det_proc_state_remove_on_gpio_irq(struct wcd9xxx_mbhc *mbhc)
{
	gpio_set_hs_det_proc_state(mbhc, &hs_det_proc_state_polling);
}

static void gpio_hs_det_proc_state_insert_enter(struct wcd9xxx_mbhc *mbhc)
{
	ktime_t debounce_time;

	struct wcd9xxx *wcd9xxx;
	struct slim_device *sb;
	int	msm_get_state_count;
	struct snd_soc_codec *codec = mbhc->codec;
	u16 reg;
#ifdef MSM8960_WCD9310
	int ret;
#endif

	wcd9xxx = codec->control_data;
	sb = wcd9xxx->slim;
	msm_get_state_count = 0;

	while(msm_get_state_count < 60)
	{
		if(msm_get_state(sb->ctrl) != 0)
		{
			msleep(50);
			msm_get_state_count++;
		} else {
			break;
		}
	}

#ifdef MSM8960_WCD9310
	if (!mbhc->mclk_enabled) {
		gpio_codec_disable_clock_block(codec);
		gpio_codec_enable_bandgap(codec, TAIKO_BANDGAP_MBHC_MODE);
		gpio_codec_enable_clock_block(codec, 1);

		ret = wcd9xxx_enable_mux_bias_block(codec, mbhc);
		if (ret) {
			pr_err("%s: Error returned, ret: %d\n", __func__, ret);
			return;
		}
	}
#endif

#ifdef MSM8960_WCD9310
	snd_soc_update_bits(codec, TAIKO_A_LDO_H_MODE_1, 0x80, 0x80);
	snd_soc_update_bits(codec, TAIKO_A_MICB_2_CTL, 0x80, 0x80);
	snd_soc_update_bits(codec, TAIKO_A_MICB_CFILT_2_CTL, 0x80, 0x80);
	snd_soc_update_bits(codec, TAIKO_A_MICB_2_INT_RBIAS, 0x80, 0x80);
#else
	snd_soc_update_bits(codec, WCD9XXX_A_MICB_CFILT_2_CTL,    0xe0, 0xe0);
	snd_soc_update_bits(codec, WCD9XXX_A_MICB_2_CTL,          0xa7, 0xa6);
	snd_soc_update_bits(codec, WCD9XXX_A_LDO_H_MODE_1,        0xcd, 0xcd);

	reg = snd_soc_read(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL);
	if(!( reg & 0x04 ))
	{
		snd_soc_update_bits(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL, 0x04, 0x04);
	}

	snd_soc_update_bits(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL, 0x01, 0x01);
	usleep_range(1000, 1000);
#endif

	g_gpio_sw_polling_flg = true;

	if(mbhc->hs_det_state == HS_DET_STATE_INSERT_3PIN)
	{
#if 0
		wake_lock(&mbhc->hs_det_wlock);
#endif
		debounce_time = ktime_set(3, 0);
		hrtimer_start(&mbhc->hs_plug_type_rejudge_timer, debounce_time, HRTIMER_MODE_REL);
		gpio_jack_detect_push_state(mbhc, &hs_btn_det_proc_state_3pinouts);
		return;
	}

	if(mbhc->hs_det_state == HS_DET_STATE_INSERT_4PIN)
	{
		mbhc->hs_jack_return_flg = true;
		gpio_jack_detect_push_state(mbhc, &hs_btn_det_proc_state_4pinouts);
		enable_irq(mbhc->mbhc_cfg->sw_gpio_irq);
		enable_irq_wake(mbhc->mbhc_cfg->sw_gpio_irq);
		wake_unlock(&mbhc->hs_det_wlock);
		return;
	}

	gpio_jack_detect_push_state(mbhc, &hs_btn_det_proc_state_init);

	gpio_hs_det_set_timer(mbhc);

#if 0
	wake_lock(&mbhc->hs_det_wlock);
#endif
	debounce_time = ktime_set(3, 0);

	hrtimer_start(&mbhc->hs_plug_type_rejudge_timer, debounce_time, HRTIMER_MODE_REL);

}

static void gpio_hs_det_proc_state_insert_exit(struct wcd9xxx_mbhc *mbhc)
{
	hrtimer_cancel(&mbhc->hs_det_timer);
	hrtimer_cancel(&mbhc->hs_plug_type_rejudge_timer);
	wake_unlock(&mbhc->hs_det_wlock);

#if 0
	gpio_hs_polling_off(mbhc);
#endif
	mbhc->polling_off_state = true;
}

static void gpio_hs_det_proc_state_insert_on_gpio_irq(struct wcd9xxx_mbhc *mbhc)
{
	gpio_set_hs_det_proc_state(mbhc, &hs_det_proc_state_polling);
}

static void gpio_hs_btn_det_proc_state_init_on_gpio_irq(struct wcd9xxx_mbhc *mbhc)
{
	gpio_jack_detect_pop_state(mbhc);
}

static void gpio_hs_btn_det_proc_state_init_on_timeout(struct wcd9xxx_mbhc *mbhc)
{
    static int last_value = 0;
    int next_value;

	next_value = gpio_hs_sw_gpio_get_value(mbhc);

	if(last_value == next_value)
	{
		mbhc->hs_sw_gpio_read_cnt++;
	}
	else
	{
		last_value = next_value;
		mbhc->hs_sw_gpio_read_cnt = 1;
	}

#if 0
	if(mbhc->hs_sw_gpio_read_cnt == TABLA_HS_PLUG_TYPE_CNT)
#else
	if(mbhc->hs_sw_gpio_read_cnt >= TABLA_HS_PLUG_TYPE_CNT)
#endif
	{
		if(next_value)
		{
			key_dm_driver_set_port(0xC0);
			snd_soc_jack_report_no_dapm(&mbhc->headset_jack, SND_JACK_HEADPHONE, WCD9XXX_GPIO_JACK_MASK);
#if 0
			mbhc->hs_det_state = HS_DET_STATE_INSERT;
#else
			mbhc->hs_det_state = HS_DET_STATE_INSERT_3PIN;
#endif
			gpio_set_hs_det_proc_state(mbhc, &hs_btn_det_proc_state_3pinouts);
		}
		else
		{
			key_dm_driver_set_port(0xC0);
			snd_soc_jack_report_no_dapm(&mbhc->headset_jack, SND_JACK_HEADSET, WCD9XXX_GPIO_JACK_MASK);
#if 0
			mbhc->hs_det_state = HS_DET_STATE_INSERT;
#else
			mbhc->hs_det_state = HS_DET_STATE_INSERT_4PIN;
#endif
			wake_unlock(&mbhc->hs_det_wlock);
			gpio_set_hs_det_proc_state(mbhc, &hs_btn_det_proc_state_4pinouts);
			enable_irq(mbhc->mbhc_cfg->sw_gpio_irq);
			enable_irq_wake(mbhc->mbhc_cfg->sw_gpio_irq);
		}
		mbhc->hs_sw_gpio_read_cnt = 0;
	}
	else
	{
		gpio_hs_det_set_timer(mbhc);
	}
}

static void gpio_hs_btn_det_proc_state_3pinouts_on_gpio_irq(struct wcd9xxx_mbhc *mbhc)
{
	gpio_jack_detect_pop_state(mbhc);
}

static void gpio_hs_btn_det_proc_state_3pinouts_on_timeout(struct wcd9xxx_mbhc *mbhc)
{
    static int last_value = 0;
    int next_value;

	next_value = gpio_hs_sw_gpio_get_value(mbhc);

	if(last_value == next_value)
	{
		mbhc->hs_sw_gpio_read_cnt++;
	}
	else
	{
		last_value = next_value;
		mbhc->hs_sw_gpio_read_cnt = 1;
	}

	if(mbhc->hs_sw_gpio_read_cnt >= TABLA_HS_PLUG_TYPE_CNT)
	{
		if(!next_value)
		{
			key_dm_driver_set_port(0xC0);
			snd_soc_jack_report_no_dapm(&mbhc->headset_jack, SND_JACK_HEADSET, WCD9XXX_GPIO_JACK_MASK);
			mbhc->hs_det_state = HS_DET_STATE_INSERT_4PIN;
			wake_unlock(&mbhc->hs_det_wlock);
			gpio_set_hs_det_proc_state(mbhc, &hs_btn_det_proc_state_4pinouts);
			enable_irq(mbhc->mbhc_cfg->sw_gpio_irq);
			enable_irq_wake(mbhc->mbhc_cfg->sw_gpio_irq);
		}
		else
		{
			wake_unlock(&mbhc->hs_det_wlock);
			gpio_hs_polling_off(mbhc);
		}

		mbhc->hs_sw_gpio_read_cnt = 0;
	}
	else
	{
		gpio_hs_det_set_timer(mbhc);
	}
}

static void gpio_hs_btn_det_proc_state_4pinouts_enter(struct wcd9xxx_mbhc *mbhc)
{
	hrtimer_cancel(&mbhc->hs_plug_type_rejudge_timer);
}

static void gpio_hs_btn_det_proc_state_4pinouts_on_gpio_irq(struct wcd9xxx_mbhc *mbhc)
{
	disable_irq_wake(mbhc->mbhc_cfg->sw_gpio_irq);
	disable_irq_nosync(mbhc->mbhc_cfg->sw_gpio_irq);
	gpio_jack_detect_pop_state(mbhc);
}

static void gpio_hs_btn_det_proc_state_4pinouts_on_sw_gpio_irq(struct wcd9xxx_mbhc *mbhc)
{
	gpio_set_hs_det_proc_state(mbhc, &hs_btn_det_proc_state_sw_polling);
}


static void gpio_hs_btn_det_proc_state_sw_polling_enter(struct wcd9xxx_mbhc *mbhc)
{
	wake_lock(&mbhc->hs_det_wlock);
	gpio_hs_det_set_timer(mbhc);
}

static void gpio_hs_btn_det_proc_state_sw_polling_on_gpio_irq(struct wcd9xxx_mbhc *mbhc)
{
	disable_irq_wake(mbhc->mbhc_cfg->sw_gpio_irq);
	disable_irq_nosync(mbhc->mbhc_cfg->sw_gpio_irq);
	gpio_jack_detect_pop_state(mbhc);
}

static void gpio_hs_btn_det_proc_state_sw_polling_on_timeout(struct wcd9xxx_mbhc *mbhc)
{
    static int last_value = 0;
    int next_value;

	next_value = gpio_hs_sw_gpio_get_value(mbhc);

	if(last_value == next_value)
	{
		mbhc->hs_sw_gpio_read_cnt++;
	}
	else
	{
		last_value = next_value;
		mbhc->hs_sw_gpio_read_cnt = 1;
	}

	if(mbhc->hs_sw_gpio_read_cnt >= TABLA_HS_POLLING_CNT)
	{
		if(next_value)
		{
			if((mbhc->hs_btn_press_flg == false) &&(mbhc->hs_jack_return_flg == false))
			{
				key_dm_driver_set_port(0x11);
				snd_soc_jack_report_no_dapm(&mbhc->button_jack, SND_JACK_BTN_0, SND_JACK_BTN_0);
				mbhc->hs_btn_press_flg = true;
			}
		}
		else
		{
			if(mbhc->hs_btn_press_flg == true)
			{
				key_dm_driver_set_port(0x10);
				snd_soc_jack_report_no_dapm(&mbhc->button_jack, 0, SND_JACK_BTN_0);
				mbhc->hs_btn_press_flg = false;
			}
		}
		mbhc->hs_sw_gpio_read_cnt = 0;
		gpio_set_hs_det_proc_state(mbhc, &hs_btn_det_proc_state_4pinouts);
		mbhc->hs_jack_return_flg = false;
		wake_unlock(&mbhc->hs_det_wlock);
	}
	else {
		gpio_hs_det_set_timer(mbhc);
	}
}

static void gpio_hs_det_set_timer(struct wcd9xxx_mbhc *mbhc)
{
	ktime_t debounce_time;

	debounce_time = ktime_set(0, 50000000);

	hrtimer_start(&mbhc->hs_det_timer, debounce_time, HRTIMER_MODE_REL);
}

static enum hrtimer_restart gpio_hs_det_timer_cb(struct hrtimer *hrtimer)
{

	struct wcd9xxx_mbhc *mbhc;

	mbhc = container_of(hrtimer, struct wcd9xxx_mbhc, hs_det_timer);

    schedule_work(&mbhc->hs_det_proc_work);

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart gpio_hs_plug_type_rejudge_timer_cb(struct hrtimer *hrtimer)
{
	struct wcd9xxx_mbhc *mbhc;

	mbhc = container_of(hrtimer, struct wcd9xxx_mbhc, hs_plug_type_rejudge_timer);

    schedule_work(&mbhc->hs_det_proc_work);

	return HRTIMER_NORESTART;
}

static void gpio_hs_det_proc_work(struct work_struct *work)
{
	struct wcd9xxx_mbhc *mbhc;

	mbhc = container_of(work, struct wcd9xxx_mbhc, hs_det_proc_work);

	gpio_jack_detect_on_timer(mbhc);
}

static int gpio_hs_sw_gpio_get_value(struct wcd9xxx_mbhc *mbhc)
{
	int value;
	value = gpio_get_value(86);
	return value;
}

static void gpio_hs_polling_off(struct wcd9xxx_mbhc *mbhc)
{
	struct snd_soc_codec *codec = mbhc->codec;
	struct wcd9xxx_resmgr *resmgr = mbhc->resmgr;
	u16 reg;
	bool tx_path_in_use = 0;
	u16 reg_value = 0;

	g_gpio_sw_polling_flg = false;

#ifdef MSM8960_WCD9310
	if (!mbhc->mclk_enabled) {
		gpio_codec_disable_clock_block(codec);
		gpio_codec_enable_bandgap(codec, TAIKO_BANDGAP_OFF);
	}
#endif
	
#ifdef MSM8960_WCD9310
	snd_soc_update_bits(codec, TAIKO_A_MICB_2_CTL, 0x80, 0x00);
	snd_soc_update_bits(codec, TAIKO_A_MICB_CFILT_2_CTL, 0x80, 0x00);
	snd_soc_update_bits(codec, TAIKO_A_BIAS_CONFIG_MODE_BG_CTL, 0x01, 0x00);
	snd_soc_update_bits(codec, TAIKO_A_CLK_BUFF_EN1, 0x08, 0x00);
	snd_soc_update_bits(codec, TAIKO_A_MICB_2_INT_RBIAS, 0x80, 0x00);

	reg_value |= snd_soc_read(codec, TAIKO_A_MICB_1_CTL);
	reg_value |= snd_soc_read(codec, TAIKO_A_MICB_2_CTL);
	reg_value |= snd_soc_read(codec, TAIKO_A_MICB_3_CTL);
	reg_value |= snd_soc_read(codec, mbhc->reg_addr.micb_4_ctl);
#else
	snd_soc_update_bits(codec, WCD9XXX_A_MICB_CFILT_2_CTL,    0xe0, 0x20);
	snd_soc_update_bits(codec, WCD9XXX_A_MICB_2_CTL,          0xa7, 0x27);

	reg_value |= snd_soc_read(codec, WCD9XXX_A_MICB_1_CTL);
	reg_value |= snd_soc_read(codec, WCD9XXX_A_MICB_2_CTL);
	reg_value |= snd_soc_read(codec, WCD9XXX_A_MICB_3_CTL);

	tx_path_in_use = ((reg_value & 0x80) != 0);
	if (! tx_path_in_use) {
		snd_soc_update_bits(codec, WCD9XXX_A_LDO_H_MODE_1,    0x80, 0x00);
	}

	reg = snd_soc_read(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL);
	if(( reg & 0x01 ) && (resmgr->bandgap_type == WCD9XXX_BANDGAP_OFF))
	{
		snd_soc_update_bits(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL, 0x01, 0x00);
		usleep_range(100, 100);
	}
#endif
}
#endif
