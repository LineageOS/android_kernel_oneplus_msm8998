/* Copyright (c) 2016 The Linux Foundation. All rights reserved.
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
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
#define pr_fmt(fmt) "SMBLIB: %s: " fmt, __func__
#endif

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/qpnp/power-on.h>
#include <linux/irq.h>
#include "smb-lib.h"
#include "smb-reg.h"
#include "storm-watch.h"
#include "pmic-voter.h"

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
#include "oem_external_fg.h"
#include <linux/gpio.h>
#include <linux/delay.h>

#define SOC_INVALID                   0x7E
#define SOC_DATA_REG_0                0x88D
#define HEARTBEAT_INTERVAL_MS         6000
#define CHG_TIMEOUT_COUNT             10 * 10 * 60 /* 10hr */
#define CHG_SOFT_OVP_MV               5800
#define BATT_SOFT_OVP_MV              4500
#define CHG_SOFT_UVP_MV               4300
#define CHG_VOLTAGE_NORMAL            5000
#define BATT_REMOVE_TEMP              -400
#define BATT_TEMP_HYST                20

struct smb_charger *g_chg;
static struct external_battery_gauge *fast_charger = NULL;
static int op_charging_en(struct smb_charger *chg, bool en);
bool op_set_fast_chg_allow(struct smb_charger *chg, bool enable);
bool get_prop_fast_chg_started(struct smb_charger *chg);
static bool set_prop_fast_switch_to_normal_false(struct smb_charger *chg);
extern void mcu_en_gpio_set(int value);
extern void usb_sw_gpio_set(int value);
extern void set_mcu_en_gpio_value(int value);
static void op_battery_temp_region_set(struct smb_charger *chg,
		temp_region_type batt_temp_region);
static void set_usb_switch(struct smb_charger *chg, bool enable);
static bool get_prop_fast_switch_to_normal(struct smb_charger *chg);
static int get_prop_batt_temp(struct smb_charger *chg);
static int get_prop_batt_capacity(struct smb_charger *chg);
static int get_prop_batt_current_now(struct smb_charger *chg);
static int get_prop_batt_voltage_now(struct smb_charger *chg);
static int set_property_on_fg(struct smb_charger *chg,
		enum power_supply_property prop, int val);
static temp_region_type
		op_battery_temp_region_get(struct smb_charger *chg);
#endif

#define smblib_err(chg, fmt, ...)		\
	pr_err("%s: %s: " fmt, chg->name,	\
		__func__, ##__VA_ARGS__)	\

#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

static bool is_secure(struct smb_charger *chg, int addr)
{
	/* assume everything above 0xA0 is secure */
	return (bool)((addr & 0xFF) >= 0xA0);
}

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val)
{
	unsigned int temp;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	return rc;
}

int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val)
{
	int rc = 0;

	mutex_lock(&chg->write_lock);
	if (is_secure(chg, addr)) {
		rc = regmap_write(chg->regmap, (addr & 0xFF00) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_update_bits(chg->regmap, addr, mask, val);

unlock:
	mutex_unlock(&chg->write_lock);
	return rc;
}

int smblib_write(struct smb_charger *chg, u16 addr, u8 val)
{
	int rc = 0;

	mutex_lock(&chg->write_lock);

	if (is_secure(chg, addr)) {
		rc = regmap_write(chg->regmap, (addr & ~(0xFF)) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_write(chg->regmap, addr, val);

unlock:
	mutex_unlock(&chg->write_lock);
	return rc;
}

static int smblib_get_step_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, step_state;
	u8 stat;

	if (!chg->step_chg_enabled) {
		*cc_delta_ua = 0;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	step_state = (stat & STEP_CHARGING_STATUS_MASK) >>
				STEP_CHARGING_STATUS_SHIFT;
	rc = smblib_get_charge_param(chg, &chg->param.step_cc_delta[step_state],
				     cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get step cc delta rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int smblib_get_jeita_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, cc_minus_ua;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}

	if (!(stat & BAT_TEMP_STATUS_SOFT_LIMIT_MASK)) {
		*cc_delta_ua = 0;
		return 0;
	}

	rc = smblib_get_charge_param(chg, &chg->param.jeita_cc_comp,
				     &cc_minus_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc minus rc=%d\n", rc);
		return rc;
	}

	*cc_delta_ua = -cc_minus_ua;
	return 0;
}

static void smblib_split_fcc(struct smb_charger *chg, int total_ua,
			     int *master_ua, int *slave_ua)
{
	int rc, jeita_cc_delta_ua, step_cc_delta_ua, effective_total_ua,
		slave_limited_ua, hw_cc_delta_ua = 0;

	rc = smblib_get_step_cc_delta(chg, &step_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get step cc delta rc=%d\n", rc);
		step_cc_delta_ua = 0;
	} else {
		hw_cc_delta_ua = step_cc_delta_ua;
	}

	rc = smblib_get_jeita_cc_delta(chg, &jeita_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc delta rc=%d\n", rc);
		jeita_cc_delta_ua = 0;
	} else if (jeita_cc_delta_ua < 0) {
		/* HW will take the min between JEITA and step charge */
		hw_cc_delta_ua = min(hw_cc_delta_ua, jeita_cc_delta_ua);
	}

	effective_total_ua = max(0, total_ua + hw_cc_delta_ua);
	slave_limited_ua = min(effective_total_ua, chg->input_limited_fcc_ua);
	*slave_ua = (slave_limited_ua * chg->pl.slave_pct) / 100;
	*slave_ua = (*slave_ua * chg->pl.taper_pct) / 100;
	*master_ua = max(0, total_ua - *slave_ua);
}

/********************
 * REGISTER GETTERS *
 ********************/

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u)
{
	int rc = 0;
	u8 val_raw;

	rc = smblib_read(chg, param->reg, &val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't read from 0x%04x rc=%d\n",
			param->name, param->reg, rc);
		return rc;
	}

	if (param->get_proc)
		*val_u = param->get_proc(param, val_raw);
	else
		*val_u = val_raw * param->step_u + param->min_u;
	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, *val_u, val_raw);

	return rc;
}

int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp;

	rc = smblib_read(chg, USBIN_CMD_IL_REG, &temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_CMD_IL rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_BIT;

	return rc;
}

#define FSW_600HZ_FOR_5V	600
#define FSW_800HZ_FOR_6V_8V	800
#define FSW_1MHZ_FOR_REMOVAL	1000
#define FSW_1MHZ_FOR_9V		1000
#define FSW_1P2MHZ_FOR_12V	1200
static int smblib_set_opt_freq_buck(struct smb_charger *chg, int fsw_khz)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_buck, fsw_khz);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_buck rc=%d\n", rc);

	if (chg->mode == PARALLEL_MASTER && chg->pl.psy) {
		pval.intval = fsw_khz;
		rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_BUCK_FREQ, &pval);
		if (rc < 0) {
			dev_err(chg->dev,
				"Could not set parallel buck_freq rc=%d\n", rc);
			return rc;
		}
	}

	return rc;
}

struct apsd_result {
	const char * const name;
	const u8 bit;
	const enum power_supply_type pst;
};

enum {
	UNKNOWN,
	SDP,
	CDP,
	DCP,
	OCP,
	FLOAT,
	HVDCP2,
	HVDCP3,
	MAX_TYPES
};

static const struct apsd_result const smblib_apsd_results[] = {
	[UNKNOWN] = {
		.name	= "UNKNOWN",
		.bit	= 0,
		.pst	= POWER_SUPPLY_TYPE_UNKNOWN
	},
	[SDP] = {
		.name	= "SDP",
		.bit	= SDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB
	},
	[CDP] = {
		.name	= "CDP",
		.bit	= CDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_CDP
	},
	[DCP] = {
		.name	= "DCP",
		.bit	= DCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[OCP] = {
		.name	= "OCP",
		.bit	= OCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[FLOAT] = {
		.name	= "FLOAT",
		.bit	= FLOAT_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[HVDCP2] = {
		.name	= "HVDCP2",
		.bit	= DCP_CHARGER_BIT | QC_2P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP
	},
	[HVDCP3] = {
		.name	= "HVDCP3",
		.bit	= DCP_CHARGER_BIT | QC_3P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP_3,
	},
};

static const struct apsd_result *smblib_get_apsd_result(struct smb_charger *chg)
{
	int rc, i;
	u8 apsd_stat, stat;
	const struct apsd_result *result = &smblib_apsd_results[UNKNOWN];

	rc = smblib_read(chg, APSD_STATUS_REG, &apsd_stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return result;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", apsd_stat);

	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT))
		return result;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_RESULT_STATUS rc=%d\n",
			rc);
		return result;
	}
	stat &= APSD_RESULT_STATUS_MASK;

	for (i = 0; i < ARRAY_SIZE(smblib_apsd_results); i++) {
		if (smblib_apsd_results[i].bit == stat)
			result = &smblib_apsd_results[i];
	}

	if (apsd_stat & QC_CHARGER_BIT) {
		/* since its a qc_charger, either return HVDCP3 or HVDCP2 */
		if (result != &smblib_apsd_results[HVDCP3])
			result = &smblib_apsd_results[HVDCP2];
	}

	return result;
}


/********************
 * REGISTER SETTERS *
 ********************/

int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u)
{
	int rc = 0;
	u8 val_raw;

	if (param->set_proc) {
		rc = param->set_proc(param, val_u, &val_raw);
		if (rc < 0)
			return -EINVAL;
	} else {
		if (val_u > param->max_u || val_u < param->min_u) {
			smblib_err(chg, "%s: %d is out of range [%d, %d]\n",
				param->name, val_u, param->min_u, param->max_u);
			return -EINVAL;
		}

		val_raw = (val_u - param->min_u) / param->step_u;
	}

	rc = smblib_write(chg, param->reg, val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, val_u, val_raw);

	return rc;
}

static int step_charge_soc_update(struct smb_charger *chg, int capacity)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.step_soc, capacity);
	if (rc < 0) {
		smblib_err(chg, "Error in updating soc, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_write(chg, STEP_CHG_SOC_VBATT_V_UPDATE_REG,
			STEP_CHG_SOC_VBATT_V_UPDATE_BIT);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't set STEP_CHG_SOC_VBATT_V_UPDATE_REG rc=%d\n",
			rc);
		return rc;
	}

	return rc;
}

int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				 suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to USBIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_SUSPEND_BIT,
				 suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to DCIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

#define MICRO_5V	5000000
#define MICRO_9V	9000000
#define MICRO_12V	12000000
static int smblib_set_usb_pd_allowed_voltage(struct smb_charger *chg,
					int min_allowed_uv, int max_allowed_uv)
{
	int rc;
	u8 allowed_voltage;

	if (min_allowed_uv == MICRO_5V && max_allowed_uv == MICRO_5V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V;
		smblib_set_opt_freq_buck(chg, FSW_600HZ_FOR_5V);
	} else if (min_allowed_uv == MICRO_9V && max_allowed_uv == MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V;
		smblib_set_opt_freq_buck(chg, FSW_1MHZ_FOR_9V);
	} else if (min_allowed_uv == MICRO_12V && max_allowed_uv == MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_12V;
		smblib_set_opt_freq_buck(chg, FSW_1P2MHZ_FOR_12V);
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_9V;
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_12V;
	} else if (min_allowed_uv < MICRO_12V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V_TO_12V;
	} else {
		smblib_err(chg, "invalid allowed voltage [%d, %d]\n",
			min_allowed_uv, max_allowed_uv);
		return -EINVAL;
	}

	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG, allowed_voltage);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to USBIN_ADAPTER_ALLOW_CFG rc=%d\n",
			allowed_voltage, rc);
		return rc;
	}

	return rc;
}

/********************
 * HELPER FUNCTIONS *
 ********************/

static int try_rerun_apsd_for_hvdcp(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result;

	/*
	 * PD_INACTIVE_VOTER on hvdcp_disable_votable indicates whether
	 * apsd rerun was tried earlier
	 */
	if (get_client_vote(chg->hvdcp_disable_votable, PD_INACTIVE_VOTER)) {
		vote(chg->hvdcp_disable_votable, PD_INACTIVE_VOTER, false, 0);
		/* ensure hvdcp is enabled */
		if (!get_effective_result(chg->hvdcp_disable_votable)) {
			apsd_result = smblib_get_apsd_result(chg);
			if (apsd_result->bit & (QC_2P0_BIT | QC_3P0_BIT)) {
				/* rerun APSD */
				smblib_dbg(chg, PR_MISC, "rerun APSD\n");
				smblib_masked_write(chg, CMD_APSD_REG,
						APSD_RERUN_BIT,
						APSD_RERUN_BIT);
			}
		}
	}
	return 0;
}

static const struct apsd_result *smblib_update_usb_type(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	/* if PD is active, APSD is disabled so won't have a valid result */
	if (chg->pd_active) {
		chg->usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
		return apsd_result;
	}
#ifdef VENDOR_EDIT //Yangfb@bsp add to fix fastcharge test not pass
	if(chg->dash_on)
		chg->usb_psy_desc.type = POWER_SUPPLY_TYPE_DASH;
	else
		chg->usb_psy_desc.type = apsd_result->pst;
#else
	chg->usb_psy_desc.type = apsd_result->pst;
#endif

	return apsd_result;
}

static int smblib_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct smb_charger *chg = container_of(nb, struct smb_charger, nb);

	if (!strcmp(psy->desc->name, "bms")) {
		if (!chg->bms_psy)
			chg->bms_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED && chg->batt_psy)
			schedule_work(&chg->bms_update_work);
	}

	if (!chg->pl.psy && !strcmp(psy->desc->name, "parallel")) {
		chg->pl.psy = psy;
		schedule_work(&chg->pl_detect_work);
	}

	return NOTIFY_OK;
}

static int smblib_register_notifier(struct smb_charger *chg)
{
	int rc;

	chg->nb.notifier_call = smblib_notifier_call;
	rc = power_supply_reg_notifier(&chg->nb);
	if (rc < 0) {
		smblib_err(chg, "Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	*val_raw = val_u << 1;

	return 0;
}

int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw)
{
	int val_u  = val_raw * param->step_u + param->min_u;

	if (val_u > param->max_u)
		val_u -= param->max_u * 2;

	return val_u;
}

int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u - param->max_u)
		return -EINVAL;

	val_u += param->max_u * 2 - param->min_u;
	val_u %= param->max_u * 2;
	*val_raw = val_u / param->step_u;

	return 0;
}

/*********************
 * VOTABLE CALLBACKS *
 *********************/

static int smblib_usb_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

#ifdef VENDOR_EDIT
	/* david.liu@bsp, 20161014 Add charging standard */
	pr_err("set usb suspend=%d\n", suspend);
#endif
	/* resume input if suspend is invalid */
	if (suspend < 0)
		suspend = 0;

	return smblib_set_usb_suspend(chg, (bool)suspend);
}

static int smblib_dc_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

	/* resume input if suspend is invalid */
	if (suspend < 0)
		suspend = 0;

	return smblib_set_dc_suspend(chg, (bool)suspend);
}

static int smblib_fcc_max_vote_callback(struct votable *votable, void *data,
			int fcc_ua, const char *client)
{
	struct smb_charger *chg = data;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("set fcc_ua=%d\n", fcc_ua);
#endif
	return vote(chg->fcc_votable, FCC_MAX_RESULT_VOTER, true, fcc_ua);
}

static int smblib_fcc_vote_callback(struct votable *votable, void *data,
			int total_fcc_ua, const char *client)
{
	struct smb_charger *chg = data;
	union power_supply_propval pval = {0, };
	int rc, master_fcc_ua = total_fcc_ua, slave_fcc_ua = 0;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("set fcc=%d, mode=%d\n", total_fcc_ua, chg->mode);
#endif
	if (total_fcc_ua < 0)
		return 0;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161109 Charging porting */
	if (chg->pl.psy && chg->mode == PARALLEL_MASTER
#else
	if (chg->mode == PARALLEL_MASTER
#endif
		&& !get_effective_result_locked(chg->pl_disable_votable)) {
		smblib_split_fcc(chg, total_fcc_ua, &master_fcc_ua,
				 &slave_fcc_ua);

		/*
		 * parallel charger is not disabled, implying that
		 * chg->pl.psy exists
		 */
		pval.intval = slave_fcc_ua;
		rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
		if (rc < 0) {
			smblib_err(chg, "Could not set parallel fcc, rc=%d\n",
				rc);
			return rc;
		}

		chg->pl.slave_fcc_ua = slave_fcc_ua;
	}

	rc = smblib_set_charge_param(chg, &chg->param.fcc, master_fcc_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set master fcc rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_PARALLEL, "master_fcc=%d slave_fcc=%d distribution=(%d/%d)\n",
		   master_fcc_ua, slave_fcc_ua,
		   (master_fcc_ua * 100) / total_fcc_ua,
		   (slave_fcc_ua * 100) / total_fcc_ua);

	return 0;
}

#define PARALLEL_FLOAT_VOLTAGE_DELTA_UV 50000
static int smblib_fv_vote_callback(struct votable *votable, void *data,
			int fv_uv, const char *client)
{
	struct smb_charger *chg = data;
	union power_supply_propval pval = {0, };
	int rc = 0;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("set fv_uv=%d, mode=%d, psy=%d\n", fv_uv, chg->mode,
			chg->pl.psy ? 1 : 0);
#endif
	if (fv_uv < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter\n");
		return 0;
	}

	rc = smblib_set_charge_param(chg, &chg->param.fv, fv_uv);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set floating voltage rc=%d\n", rc);
		return rc;
	}

	if (chg->mode == PARALLEL_MASTER && chg->pl.psy) {
		pval.intval = fv_uv + PARALLEL_FLOAT_VOLTAGE_DELTA_UV;
		rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX, &pval);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't set float on parallel rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

#define USBIN_25MA	25000
#define USBIN_100MA	100000
#define USBIN_150MA	150000
#define USBIN_500MA	500000
#define USBIN_900MA	900000
#ifndef VENDOR_EDIT
/* david.liu@bsp, 20161214 usb can't charge in power off charging */
static int smblib_usb_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool suspend = (icl_ua < USBIN_25MA);
	u8 icl_options = 0;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("set iusb_max=%d, type=%d\n", icl_ua,
			chg->usb_psy_desc.type);
#endif
	if (icl_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter hence suspending\n");
		icl_ua = 0;
	}

	if (suspend)
		goto out;

	if (chg->usb_psy_desc.type != POWER_SUPPLY_TYPE_USB) {
		rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set HC ICL rc=%d\n", rc);
			return rc;
		}

		goto out;
	}

	/* power source is SDP */
	switch (icl_ua) {
	case USBIN_100MA:
		/* USB 2.0 100mA */
		icl_options = 0;
		break;
	case USBIN_150MA:
		/* USB 3.0 150mA */
		icl_options = CFG_USB3P0_SEL_BIT;
		break;
	case USBIN_500MA:
		/* USB 2.0 500mA */
		icl_options = USB51_MODE_BIT;
		break;
	case USBIN_900MA:
		/* USB 3.0 900mA */
		icl_options = CFG_USB3P0_SEL_BIT | USB51_MODE_BIT;
		break;
	default:
		smblib_err(chg, "ICL %duA isn't supported for SDP\n", icl_ua);
		icl_options = 0;
		break;
	}

out:
	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
			CFG_USB3P0_SEL_BIT | USB51_MODE_BIT, icl_options);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL opetions rc=%d\n", rc);
		return rc;
	}

	rc = vote(chg->usb_suspend_votable, PD_VOTER, suspend, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s input rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}

	return rc;
}
#else
static int smblib_usb_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool suspend;

	if (icl_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter hence suspending\n");
		icl_ua = 0;
	}

	pr_info("set iusb_max=%d, type=%d\n", icl_ua,
			chg->usb_psy_desc.type);

	suspend = (icl_ua < USBIN_25MA);
	if (suspend)
		goto suspend;

	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB)
		rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
				USB51_MODE_BIT,
				(icl_ua > USBIN_100MA) ? USB51_MODE_BIT : 0);
	else
		rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);

	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't set USB input current limit rc=%d\n", rc);
		return rc;
	}

suspend:
	rc = vote(chg->usb_suspend_votable, PD_VOTER, suspend, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't %s input rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}

	return rc;
}
#endif

#define MICRO_250MA	250000
static int smblib_otg_cl_config(struct smb_charger *chg, int otg_cl_ua)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.otg_cl, otg_cl_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set otg current limit rc=%d\n", rc);
		return rc;
	}

	/* configure PFM/PWM mode for OTG regulator */
	rc = smblib_masked_write(chg, DC_ENG_SSUPPLY_CFG3_REG,
				 ENG_SSUPPLY_CFG_SKIP_TH_V0P2_BIT,
				 otg_cl_ua > MICRO_250MA ? 1 : 0);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't write DC_ENG_SSUPPLY_CFG3_REG rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int smblib_dc_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool suspend;

	if (icl_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter hence suspending\n");
		icl_ua = 0;
	}

	suspend = (icl_ua < USBIN_25MA);
	if (suspend)
		goto suspend;

	rc = smblib_set_charge_param(chg, &chg->param.dc_icl, icl_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set DC input current limit rc=%d\n",
			rc);
		return rc;
	}

suspend:
	rc = vote(chg->dc_suspend_votable, USER_VOTER, suspend, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}
	return rc;
}

static int smblib_pd_disallowed_votable_indirect_callback(
	struct votable *votable, void *data, int disallowed, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = vote(chg->pd_allowed_votable, PD_DISALLOWED_INDIRECT_VOTER,
		!disallowed, 0);

	return rc;
}

static int smblib_awake_vote_callback(struct votable *votable, void *data,
			int awake, const char *client)
{
	struct smb_charger *chg = data;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("set awake=%d\n", awake);
#endif
	if (awake)
		pm_stay_awake(chg->dev);
	else
		pm_relax(chg->dev);

	return 0;
}

static int smblib_pl_disable_vote_callback(struct votable *votable, void *data,
			int pl_disable, const char *client)
{
	struct smb_charger *chg = data;
	union power_supply_propval pval = {0, };
	int rc;

	if (chg->mode != PARALLEL_MASTER || !chg->pl.psy)
		return 0;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("set pl_disable=%d\n", pl_disable);
#endif
	chg->pl.taper_pct = 100;
	rerun_election(chg->fv_votable);
	rerun_election(chg->fcc_votable);

	pval.intval = pl_disable;
	rc = power_supply_set_property(chg->pl.psy,
			POWER_SUPPLY_PROP_INPUT_SUSPEND, &pval);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't change slave suspend state rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_PARALLEL, "parallel charging %s\n",
		   pl_disable ? "disabled" : "enabled");

	return 0;
}

static int smblib_chg_disable_vote_callback(struct votable *votable, void *data,
			int chg_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_err("set chg_disable=%d\n", chg_disable);
#endif
	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				 CHARGING_ENABLE_CMD_BIT,
				 chg_disable ? 0 : CHARGING_ENABLE_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s charging rc=%d\n",
			chg_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}

static int smblib_pl_enable_indirect_vote_callback(struct votable *votable,
			void *data, int chg_enable, const char *client)
{
	struct smb_charger *chg = data;

	vote(chg->pl_disable_votable, PL_INDIRECT_VOTER, !chg_enable, 0);

	return 0;
}

static int smblib_hvdcp_disable_vote_callback(struct votable *votable,
			void *data,
			int hvdcp_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;
	u8 val = HVDCP_AUTH_ALG_EN_CFG_BIT
		| HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT | HVDCP_EN_BIT;

	/*
	 * Disable the autonomous bit and auth bit for disabling hvdcp.
	 * This ensures only qc 2.0 detection runs but no vbus
	 * negotiation happens.
	 */
	if (hvdcp_disable)
		val = HVDCP_EN_BIT;

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				 HVDCP_EN_BIT
				 | HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT
				 | HVDCP_AUTH_ALG_EN_CFG_BIT,
				 val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s hvdcp rc=%d\n",
			hvdcp_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}

static int smblib_apsd_disable_vote_callback(struct votable *votable,
			void *data,
			int apsd_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				 AUTO_SRC_DETECT_BIT,
				 apsd_disable ? 0 : AUTO_SRC_DETECT_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s APSD rc=%d\n",
			apsd_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}
/*****************
 * OTG REGULATOR *
 *****************/

#define MAX_SOFTSTART_TRIES	2
int smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	u8 stat;
	int rc = 0;
	int tries = MAX_SOFTSTART_TRIES;

	rc = smblib_masked_write(chg, OTG_ENG_OTG_CFG_REG,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set OTG_ENG_OTG_CFG_REG rc=%d\n",
			rc);
		return rc;
	}

	rc = smblib_write(chg, CMD_OTG_REG, OTG_EN_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable OTG regulator rc=%d\n", rc);
		return rc;
	}

	/* waiting for boost readiness, usually ~1ms, 2ms in worst case */
	do {
		usleep_range(1000, 1100);

		rc = smblib_read(chg, OTG_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read OTG_STATUS_REG rc=%d\n",
				rc);
			return rc;
		}
		if (stat & BOOST_SOFTSTART_DONE_BIT) {
			smblib_otg_cl_config(chg, chg->otg_cl_ua);
			break;
		}
	} while (--tries);

	if (tries == 0)
		smblib_err(chg, "Timeout waiting for boost softstart rc=%d\n",
				rc);

	return rc;
}

int smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = smblib_write(chg, CMD_OTG_REG, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable OTG regulator rc=%d\n", rc);
		return rc;
	}

	smblib_otg_cl_config(chg, MICRO_250MA);

	rc = smblib_masked_write(chg, OTG_ENG_OTG_CFG_REG,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set OTG_ENG_OTG_CFG_REG rc=%d\n",
			rc);
		return rc;
	}


	return rc;
}

int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 cmd;

	rc = smblib_read(chg, CMD_OTG_REG, &cmd);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CMD_OTG rc=%d", rc);
		return rc;
	}

	return (cmd & OTG_EN_BIT) ? 1 : 0;
}

/*******************
 * VCONN REGULATOR *
 * *****************/

int smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	u8 stat;
	int rc = 0;

	/*
	 * VCONN_EN_ORIENTATION is overloaded with overriding the CC pin used
	 * for Vconn, and it should be set with reverse polarity of CC_OUT.
	 */
	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}
	stat = stat & CC_ORIENTATION_BIT ? 0 : VCONN_EN_ORIENTATION_BIT;
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT | VCONN_EN_ORIENTATION_BIT,
				 VCONN_EN_VALUE_BIT | stat);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable vconn setting rc=%d\n", rc);

	return rc;
}

int smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable vconn regulator rc=%d\n",
			rc);

	return rc;
}

int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 cmd;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &cmd);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			rc);
		return rc;
	}

	return (cmd & VCONN_EN_VALUE_BIT) ? 1 : 0;
}

/********************
 * BATT PSY GETTERS *
 ********************/

int smblib_get_prop_input_suspend(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	val->intval = get_client_vote(chg->usb_suspend_votable, USER_VOTER) &&
			get_client_vote(chg->dc_suspend_votable, USER_VOTER);
	return 0;
}

int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATIF_INT_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = !(stat & (BAT_THERM_OR_ID_MISSING_RT_STS_BIT
					| BAT_TERMINAL_MISSING_RT_STS_BIT));

	return rc;
}

int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (chg->fake_capacity >= 0) {
		val->intval = chg->fake_capacity;
		return 0;
	}

	if (chg->bms_psy)
		rc = power_supply_get_property(chg->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, val);
	return rc;
}

int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	bool usb_online, dc_online;
	u8 stat;
	int rc;

	rc = smblib_get_prop_usb_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb online property rc=%d\n",
			rc);
		return rc;
	}
	usb_online = (bool)pval.intval;

	rc = smblib_get_prop_dc_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get dc online property rc=%d\n",
			rc);
		return rc;
	}
	dc_online = (bool)pval.intval;

	if (!usb_online && !dc_online) {
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return rc;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	switch (stat) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FAST_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case DISABLE_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	switch (stat & BATTERY_CHARGER_STATUS_MASK) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case FAST_CHARGE:
	case FULLON_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TAPER;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return rc;
}

int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval;
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_2 = 0x%02x\n",
		   stat);

	if (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT) {
		rc = smblib_get_prop_batt_voltage_now(chg, &pval);
		if (!rc) {
			/*
			 * If Vbatt is within 40mV above Vfloat, then don't
			 * treat it as overvoltage.
			 */
			if (pval.intval >=
				get_effective_result(chg->fv_votable) + 40000) {
				val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
				smblib_err(chg, "battery over-voltage\n");
				goto done;
			}
		}
	}

	if (stat & BAT_TEMP_STATUS_TOO_COLD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_STATUS_TOO_HOT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_STATUS_COLD_SOFT_LIMIT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_STATUS_HOT_SOFT_LIMIT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_WARM;
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

done:
	return rc;
}

int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->system_temp_level;
	return 0;
}

int smblib_get_prop_input_current_limited(struct smb_charger *chg,
				union power_supply_propval *val)
{
	u8 stat;
	int rc;

	rc = smblib_read(chg, AICL_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n", rc);
		return rc;
	}
	val->intval = (stat & SOFT_ILIMIT_BIT) || chg->is_hdc;
	return 0;
}

int smblib_get_prop_batt_voltage_now(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	return rc;
}

int smblib_get_prop_batt_current_now(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_CURRENT_NOW, val);
	return rc;
}

int smblib_get_prop_batt_temp(struct smb_charger *chg,
			      union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	if (chg->use_fake_temp) {
		val->intval = chg->fake_temp;
		return 0;
	}
#endif

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_TEMP, val);
	return rc;
}

int smblib_get_prop_step_chg_step(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	if (!chg->step_chg_enabled) {
		val->intval = -1;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	val->intval = (stat & STEP_CHARGING_STATUS_MASK) >>
				STEP_CHARGING_STATUS_SHIFT;

	return rc;
}

int smblib_get_prop_batt_charge_done(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	val->intval = (stat == TERMINATE_CHARGE);
	return 0;
}

/***********************
 * BATTERY PSY SETTERS *
 ***********************/

int smblib_set_prop_input_suspend(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc;

	rc = vote(chg->usb_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);
	return rc;
}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
static int op_check_battery_temp(struct smb_charger *chg);

int smblib_set_prop_chg_voltage(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_chgvol = val->intval;
	chg->use_fake_chgvol = true;
	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_batt_temp(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_temp = val->intval;
	chg->use_fake_temp = true;
	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_chg_protect_status(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_protect_sts = val->intval;
	chg->use_fake_protect_sts = true;
	power_supply_changed(chg->batt_psy);

	return 0;
}
int smblib_set_prop_charge_parameter_set(struct smb_charger *chg)
{
	chg->is_power_changed = true;
	op_check_battery_temp(chg);
	return 0;
}

#endif

int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_capacity = val->intval;

	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels)
		return -EINVAL;

	chg->system_temp_level = val->intval;
	if (chg->system_temp_level == chg->thermal_levels)
		return vote(chg->chg_disable_votable,
			THERMAL_DAEMON_VOTER, true, 0);

	vote(chg->chg_disable_votable, THERMAL_DAEMON_VOTER, false, 0);
	if (chg->system_temp_level == 0)
		return vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, false, 0);

	vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, true,
			chg->thermal_mitigation[chg->system_temp_level]);
	return 0;
}

/*******************
 * DC PSY GETTERS *
 *******************/

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DCIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_dc_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote(chg->dc_suspend_votable, USER_VOTER)) {
		val->intval = false;
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_DCIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_BIT);

	return rc;
}

int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_effective_result_locked(chg->dc_icl_votable);
	return 0;
}

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	rc = vote(chg->dc_icl_votable, USER_VOTER, true, val->intval);
	return rc;
}

/*******************
 * USB PSY GETTERS *
 *******************/

int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_usb_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote(chg->usb_suspend_votable, USER_VOTER)) {
		val->intval = false;
		return rc;
	}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161122 Fix power off charging loop */
	if (chg->vbus_present) {
		val->intval = true;
		return rc;
	}
#endif

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_USBIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_BIT);
	return rc;
}

int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_get_prop_usb_present(chg, val);
	if (rc < 0 || !val->intval)
		return rc;

	if (!chg->iio.usbin_v_chan ||
		PTR_ERR(chg->iio.usbin_v_chan) == -EPROBE_DEFER)
		chg->iio.usbin_v_chan = iio_channel_get(chg->dev, "usbin_v");

	if (IS_ERR(chg->iio.usbin_v_chan))
		return PTR_ERR(chg->iio.usbin_v_chan);
#ifdef VENDOR_EDIT
		/* yangfb@bsp, 20161229 Vbus switch uV to mV */
	rc = iio_read_channel_processed(chg->iio.usbin_v_chan, &val->intval);
	val->intval = val->intval/1000;
	return rc;
#else
	return iio_read_channel_processed(chg->iio.usbin_v_chan, &val->intval);
#endif
}

int smblib_get_prop_pd_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_client_vote_locked(chg->usb_icl_votable, PD_VOTER);
	return 0;
}

int smblib_get_prop_usb_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_client_vote_locked(chg->usb_icl_votable,
			USB_PSY_VOTER);
	return 0;
}

int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_get_prop_usb_present(chg, val);
	if (rc < 0 || !val->intval)
		return rc;

	if (!chg->iio.usbin_i_chan ||
		PTR_ERR(chg->iio.usbin_i_chan) == -EPROBE_DEFER)
		chg->iio.usbin_i_chan = iio_channel_get(chg->dev, "usbin_i");

	if (IS_ERR(chg->iio.usbin_i_chan))
		return PTR_ERR(chg->iio.usbin_i_chan);

	return iio_read_channel_processed(chg->iio.usbin_i_chan, &val->intval);
}

int smblib_get_prop_charger_temp(struct smb_charger *chg,
				 union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.temp_chan ||
		PTR_ERR(chg->iio.temp_chan) == -EPROBE_DEFER)
		chg->iio.temp_chan = iio_channel_get(chg->dev, "charger_temp");

	if (IS_ERR(chg->iio.temp_chan))
		return PTR_ERR(chg->iio.temp_chan);

	rc = iio_read_channel_processed(chg->iio.temp_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_charger_temp_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.temp_max_chan ||
		PTR_ERR(chg->iio.temp_max_chan) == -EPROBE_DEFER)
		chg->iio.temp_max_chan = iio_channel_get(chg->dev,
							 "charger_temp_max");
	if (IS_ERR(chg->iio.temp_max_chan))
		return PTR_ERR(chg->iio.temp_max_chan);

	rc = iio_read_channel_processed(chg->iio.temp_max_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
					 union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n",
		   stat);

	if (stat & CC_ATTACHED_BIT)
		val->intval = (bool)(stat & CC_ORIENTATION_BIT) + 1;
	else
		val->intval = 0;

	return rc;
}

static const char * const smblib_typec_mode_name[] = {
	[POWER_SUPPLY_TYPEC_NONE]		  = "NONE",
	[POWER_SUPPLY_TYPEC_SOURCE_DEFAULT]	  = "SOURCE_DEFAULT",
	[POWER_SUPPLY_TYPEC_SOURCE_MEDIUM]	  = "SOURCE_MEDIUM",
	[POWER_SUPPLY_TYPEC_SOURCE_HIGH]	  = "SOURCE_HIGH",
	[POWER_SUPPLY_TYPEC_NON_COMPLIANT]	  = "NON_COMPLIANT",
	[POWER_SUPPLY_TYPEC_SINK]		  = "SINK",
	[POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE]   = "SINK_POWERED_CABLE",
	[POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY] = "SINK_DEBUG_ACCESSORY",
	[POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER]   = "SINK_AUDIO_ADAPTER",
	[POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY]   = "POWERED_CABLE_ONLY",
};

static int smblib_get_prop_ufp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_1 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_1 = 0x%02x\n", stat);

	switch (stat) {
	case 0:
		return POWER_SUPPLY_TYPEC_NONE;
	case UFP_TYPEC_RDSTD_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	case UFP_TYPEC_RD1P5_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
	case UFP_TYPEC_RD3P0_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_HIGH;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NON_COMPLIANT;
}

static int smblib_get_prop_dfp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_2 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_2 = 0x%02x\n", stat);

	switch (stat & DFP_TYPEC_MASK) {
	case DFP_RA_RA_BIT:
		return POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
	case DFP_RD_RD_BIT:
		return POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
	case DFP_RD_RA_VCONN_BIT:
		return POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE;
	case DFP_RD_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_SINK;
	case DFP_RA_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

int smblib_get_prop_typec_mode(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		val->intval = POWER_SUPPLY_TYPEC_NONE;
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat);

	if (!(stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT)) {
		val->intval = POWER_SUPPLY_TYPEC_NONE;
		return rc;
	}

	if (stat & UFP_DFP_MODE_STATUS_BIT)
		val->intval = smblib_get_prop_dfp_mode(chg);
	else
		val->intval = smblib_get_prop_ufp_mode(chg);

	return rc;
}

int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc = 0;
	u8 ctrl;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_INTRPT_ENB_SOFTWARE_CTRL = 0x%02x\n",
		   ctrl);

	if (ctrl & TYPEC_DISABLE_CMD_BIT) {
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		return rc;
	}

	switch (ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT)) {
	case 0:
		val->intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		break;
	case DFP_EN_CMD_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
		break;
	case UFP_EN_CMD_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SINK;
		break;
	default:
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		smblib_err(chg, "unsupported power role 0x%02lx\n",
			ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT));
		return -EINVAL;
	}

	return rc;
}

int smblib_get_prop_pd_allowed(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = get_effective_result(chg->pd_allowed_votable);
	return 0;
}

int smblib_get_prop_input_current_settled(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.icl_stat, &val->intval);
}

int smblib_get_prop_pd_in_hard_reset(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc;
	u8 ctrl;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG rc=%d\n",
			rc);
		return rc;
	}
	val->intval = ctrl & EXIT_SNK_BASED_ON_CC_BIT;
	return 0;
}

int smblib_get_pe_start(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	/*
	 * hvdcp timeout voter is the last one to allow pd. Use its vote
	 * to indicate start of pe engine
	 */
	val->intval
		= !get_client_vote_locked(chg->pd_disallowed_votable_indirect,
			HVDCP_TIMEOUT_VOTER);
	return 0;
}

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_pd_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	if (chg->pd_active)
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, val->intval);
	else
		rc = -EPERM;

	return rc;
}

int smblib_set_prop_usb_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_err("set usb current_max=%d\n", val->intval);
#endif
	if (!chg->pd_active) {
		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
				true, val->intval);
	} else if (chg->system_suspend_supported) {
		if (val->intval <= USBIN_25MA)
			rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
					true, val->intval);
		else
			rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
					false, 0);
	}
	return rc;
}

int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				     const union power_supply_propval *val)
{
	int rc = 0;
	u8 power_role;

	switch (val->intval) {
	case POWER_SUPPLY_TYPEC_PR_NONE:
		power_role = TYPEC_DISABLE_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_DUAL:
		power_role = 0;
		break;
	case POWER_SUPPLY_TYPEC_PR_SINK:
		power_role = UFP_EN_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_SOURCE:
		power_role = DFP_EN_CMD_BIT;
		break;
	default:
		smblib_err(chg, "power role %d not supported\n", val->intval);
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, power_role);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			power_role, rc);
		return rc;
	}

	return rc;
}

int smblib_set_prop_usb_voltage_min(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, min_uv;

	min_uv = min(val->intval, chg->voltage_max_uv);
	rc = smblib_set_usb_pd_allowed_voltage(chg, min_uv,
					       chg->voltage_max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid max voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER,
		     min_uv > MICRO_5V, 0);

	chg->voltage_min_uv = min_uv;
	return rc;
}

int smblib_set_prop_usb_voltage_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, max_uv;

	max_uv = max(val->intval, chg->voltage_min_uv);
	rc = smblib_set_usb_pd_allowed_voltage(chg, chg->voltage_min_uv,
					       max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid min voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_max_uv = max_uv;
	return rc;
}

int smblib_set_prop_pd_active(struct smb_charger *chg,
			      const union power_supply_propval *val)
{
	int rc;
	u8 stat = 0;
	bool cc_debounced;
	bool orientation;
	bool pd_active = val->intval;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	pr_info("set pd_active=%d\n", val->intval);
#endif
	if (!get_effective_result(chg->pd_allowed_votable)) {
		smblib_err(chg, "PD is not allowed\n");
		return -EINVAL;
	}

	vote(chg->apsd_disable_votable, PD_VOTER, pd_active, 0);
	vote(chg->pd_allowed_votable, PD_VOTER, pd_active, 0);

	/*
	 * VCONN_EN_ORIENTATION_BIT controls whether to use CC1 or CC2 line
	 * when TYPEC_SPARE_CFG_BIT (CC pin selection s/w override) is set
	 * or when VCONN_EN_VALUE_BIT is set.
	 */
	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}

	if (pd_active) {
		orientation = stat & CC_ORIENTATION_BIT;
		rc = smblib_masked_write(chg,
				TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				VCONN_EN_ORIENTATION_BIT,
				orientation ? 0 : VCONN_EN_ORIENTATION_BIT);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't enable vconn on CC line rc=%d\n", rc);
			return rc;
		}

		rc = vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_500MA);
		if (rc < 0) {
			smblib_err(chg, "Couldn't vote for USB ICL rc=%d\n",
					rc);
			return rc;
		}

		rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't vote for USB ICL rc=%d\n",
					rc);
			return rc;
		}

		rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
				USBIN_MODE_CHG_BIT, USBIN_MODE_CHG_BIT);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't change USB mode rc=%d\n", rc);
			return rc;
		}

		rc = smblib_masked_write(chg, CMD_APSD_REG,
				ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't override APSD rc=%d\n", rc);
			return rc;
		}
	} else {
		rc = vote(chg->usb_icl_votable, DCP_VOTER, true,
				chg->dcp_icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't vote for USB ICL rc=%d\n",
					rc);
			return rc;
		}

		rc = smblib_masked_write(chg, CMD_APSD_REG,
				ICL_OVERRIDE_BIT, 0);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't override APSD rc=%d\n", rc);
			return rc;
		}

		rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
				USBIN_MODE_CHG_BIT, 0);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't change USB mode rc=%d\n", rc);
			return rc;
		}
	}

	/* CC pin selection s/w override in PD session; h/w otherwise. */
	rc = smblib_masked_write(chg, TAPER_TIMER_SEL_CFG_REG,
				 TYPEC_SPARE_CFG_BIT,
				 pd_active ? TYPEC_SPARE_CFG_BIT : 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't change cc_out ctrl to %s rc=%d\n",
			pd_active ? "SW" : "HW", rc);
		return rc;
	}

	cc_debounced = (bool)(stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT);
	if (!pd_active && cc_debounced)
		try_rerun_apsd_for_hvdcp(chg);

	chg->pd_active = pd_active;
	smblib_update_usb_type(chg);
	power_supply_changed(chg->usb_psy);

	return rc;
}

int smblib_reg_block_update(struct smb_charger *chg,
				struct reg_info *entry)
{
	int rc = 0;

	while (entry && entry->reg) {
		rc = smblib_read(chg, entry->reg, &entry->bak);
		if (rc < 0) {
			dev_err(chg->dev, "Error in reading %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry->bak &= entry->mask;

		rc = smblib_masked_write(chg, entry->reg,
					 entry->mask, entry->val);
		if (rc < 0) {
			dev_err(chg->dev, "Error in writing %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry++;
	}

	return rc;
}

int smblib_reg_block_restore(struct smb_charger *chg,
				struct reg_info *entry)
{
	int rc = 0;

	while (entry && entry->reg) {
		rc = smblib_masked_write(chg, entry->reg,
					 entry->mask, entry->bak);
		if (rc < 0) {
			dev_err(chg->dev, "Error in writing %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry++;
	}

	return rc;
}

static struct reg_info cc2_detach_settings[] = {
	{
		.reg	= TYPE_C_CFG_2_REG,
		.mask	= TYPE_C_UFP_MODE_BIT | EN_TRY_SOURCE_MODE_BIT,
		.val	= TYPE_C_UFP_MODE_BIT,
		.desc	= "TYPE_C_CFG_2_REG",
	},
	{
		.reg	= TYPE_C_CFG_3_REG,
		.mask	= EN_TRYSINK_MODE_BIT,
		.val	= 0,
		.desc	= "TYPE_C_CFG_3_REG",
	},
	{
		.reg	= TAPER_TIMER_SEL_CFG_REG,
		.mask	= TYPEC_SPARE_CFG_BIT,
		.val	= TYPEC_SPARE_CFG_BIT,
		.desc	= "TAPER_TIMER_SEL_CFG_REG",
	},
	{
		.reg	= TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
		.mask	= VCONN_EN_ORIENTATION_BIT,
		.val	= 0,
		.desc	= "TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG",
	},
	{
		.reg	= MISC_CFG_REG,
		.mask	= TCC_DEBOUNCE_20MS_BIT,
		.val	= TCC_DEBOUNCE_20MS_BIT,
		.desc	= "Tccdebounce time"
	},
	{
	},
};

static int smblib_cc2_sink_removal_enter(struct smb_charger *chg)
{
	int rc = 0;
	union power_supply_propval cc2_val = {0, };

	if ((chg->wa_flags & TYPEC_CC2_REMOVAL_WA_BIT) == 0)
		return rc;

	if (chg->cc2_sink_detach_flag != CC2_SINK_NONE)
		return rc;

	rc = smblib_get_prop_typec_cc_orientation(chg, &cc2_val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get cc orientation rc=%d\n", rc);
		return rc;
	}
	if (cc2_val.intval == 1)
		return rc;

	rc = smblib_get_prop_typec_mode(chg, &cc2_val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get prop typec mode rc=%d\n", rc);
		return rc;
	}

	switch (cc2_val.intval) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		smblib_reg_block_update(chg, cc2_detach_settings);
		chg->cc2_sink_detach_flag = CC2_SINK_STD;
		schedule_work(&chg->rdstd_cc2_detach_work);
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		chg->cc2_sink_detach_flag = CC2_SINK_MEDIUM_HIGH;
		break;
	default:
		break;
	}

	return rc;
}

static int smblib_cc2_sink_removal_exit(struct smb_charger *chg)
{
	int rc = 0;

	if ((chg->wa_flags & TYPEC_CC2_REMOVAL_WA_BIT) == 0)
		return rc;

	if (chg->cc2_sink_detach_flag == CC2_SINK_STD) {
		cancel_work_sync(&chg->rdstd_cc2_detach_work);
		smblib_reg_block_restore(chg, cc2_detach_settings);
	}

	chg->cc2_sink_detach_flag = CC2_SINK_NONE;

	return rc;
}

int smblib_set_prop_pd_in_hard_reset(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 EXIT_SNK_BASED_ON_CC_BIT,
				 (val->intval) ? EXIT_SNK_BASED_ON_CC_BIT : 0);
	if (rc < 0) {
		smblib_err(chg, "Could not set EXIT_SNK_BASED_ON_CC rc=%d\n",
				rc);
		return rc;
	}

	vote(chg->apsd_disable_votable, PD_HARD_RESET_VOTER, val->intval, 0);

	if (val->intval)
		rc = smblib_cc2_sink_removal_enter(chg);
	else
		rc = smblib_cc2_sink_removal_exit(chg);

	if (rc < 0) {
		smblib_err(chg, "Could not detect cc2 removal rc=%d\n", rc);
		return rc;
	}

	return rc;
}

/************************
 * PARALLEL PSY GETTERS *
 ************************/

int smblib_get_prop_slave_current_now(struct smb_charger *chg,
				      union power_supply_propval *pval)
{
	if (IS_ERR_OR_NULL(chg->iio.batt_i_chan))
		chg->iio.batt_i_chan = iio_channel_get(chg->dev, "batt_i");

	if (IS_ERR(chg->iio.batt_i_chan))
		return PTR_ERR(chg->iio.batt_i_chan);

	return iio_read_channel_processed(chg->iio.batt_i_chan, &pval->intval);
}

/**********************
 * INTERRUPT HANDLERS *
 **********************/

irqreturn_t smblib_handle_debug(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	return IRQ_HANDLED;
}

static void smblib_pl_handle_chg_state_change(struct smb_charger *chg, u8 stat)
{
	bool pl_enabled;

	if (chg->mode != PARALLEL_MASTER)
		return;

	pl_enabled = !get_effective_result_locked(chg->pl_disable_votable);
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("IRQ: stat=%d, pl_enabled=%d\n", stat, pl_enabled);
#endif
	switch (stat) {
	case FAST_CHARGE:
	case FULLON_CHARGE:
		vote(chg->pl_disable_votable, CHG_STATE_VOTER, false, 0);
		break;
	case TAPER_CHARGE:
		if (pl_enabled) {
			cancel_delayed_work_sync(&chg->pl_taper_work);
			schedule_delayed_work(&chg->pl_taper_work, 0);
		}
		break;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
	case DISABLE_CHARGE:
		vote(chg->pl_disable_votable, TAPER_END_VOTER, false, 0);
		break;
	default:
		break;
	}
}

irqreturn_t smblib_handle_chg_state_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return IRQ_HANDLED;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	smblib_pl_handle_chg_state_change(chg, stat);
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161109 Charging porting */
	if (stat == TERMINATE_CHARGE) {
		/* charge done, disable charge in software also */
		pr_err("TERMINATE_CHARGE: chg_done:temp:%d,soc_calib:%d,VOLT:%d,current:%d\n",
			get_prop_batt_temp(chg), get_prop_batt_capacity(chg),
			get_prop_batt_voltage_now(chg) / 1000,
			get_prop_batt_current_now(chg) / 1000);
		op_charging_en(chg, false);
	}
#endif
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_step_chg_state_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->step_chg_enabled)
		rerun_election(chg->fcc_votable);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_step_chg_soc_update_fail(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->step_chg_enabled)
		rerun_election(chg->fcc_votable);

	return IRQ_HANDLED;
}

#define STEP_SOC_REQ_MS	3000
irqreturn_t smblib_handle_step_chg_soc_update_request(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	union power_supply_propval pval = {0, };

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (!chg->bms_psy) {
		schedule_delayed_work(&chg->step_soc_req_work,
				      msecs_to_jiffies(STEP_SOC_REQ_MS));
		return IRQ_HANDLED;
	}

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "Couldn't get batt capacity rc=%d\n", rc);
	else
		step_charge_soc_update(chg, pval.intval);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_batt_temp_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	rerun_election(chg->fcc_votable);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_batt_psy_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usb_psy_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->usb_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usb_plugin(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat;
	bool vbus_rising;
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	bool last_vbus_present;
	int is_usb_supend;
	last_vbus_present = chg->vbus_present;
#endif
	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	smblib_set_opt_freq_buck(chg,
		vbus_rising ? FSW_600HZ_FOR_5V : FSW_1MHZ_FOR_REMOVAL);

	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
					      "dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			smblib_err(chg, "Couldn't get dpdm regulator rc=%ld\n",
				PTR_ERR(chg->dpdm_reg));
			chg->dpdm_reg = NULL;
		}
	}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	chg->vbus_present = vbus_rising;
	if (last_vbus_present != chg->vbus_present) {
		if (chg->vbus_present) {
			pr_info("acquire chg_wake_lock\n");
			wake_lock(&chg->chg_wake_lock);
			smblib_get_usb_suspend(chg,&is_usb_supend);
			if(is_usb_supend)
				vote(chg->usb_suspend_votable,
						BOOST_BACK_VOTER, false, 0);
		} else {
			pr_info("release chg_wake_lock\n");
			wake_unlock(&chg->chg_wake_lock);
		}
	}

	chg->dash_on = get_prop_fast_chg_started(chg);
	if (chg->dash_on) {
		pr_err("return directly because dash is online\n");
		return IRQ_HANDLED;
	}
#endif

	if (vbus_rising) {
		if (chg->dpdm_reg && !regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg, "Couldn't enable dpdm regulator rc=%d\n",
					rc);
		}
	} else {
		if (chg->wa_flags & BOOST_BACK_WA)
			vote(chg->usb_suspend_votable,
						BOOST_BACK_VOTER, false, 0);

		if (chg->dpdm_reg && regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg, "Couldn't disable dpdm regulator rc=%d\n",
					rc);
		}
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
		if (last_vbus_present != chg->vbus_present) {
			op_set_fast_chg_allow(chg, false);
			set_prop_fast_switch_to_normal_false(chg);
			set_usb_switch(chg, false);

			chg->dash_on = false;
			chg->chg_done = false;
			chg->time_out = false;
			chg->recharge_status = false;
			chg->usb_enum_status = false;
			chg->non_std_chg_present = false;
			op_battery_temp_region_set(chg, BATT_TEMP_INVALID);
		}
#endif
	}

	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s %s\n",
		   irq_data->name, vbus_rising ? "attached" : "detached");
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	pr_err("IRQ: %s %s\n",
		   irq_data->name, chg->vbus_present ? "attached" : "detached");
#endif
	return IRQ_HANDLED;
}

#define USB_WEAK_INPUT_UA	1400000
#define EFFICIENCY_PCT		80
irqreturn_t smblib_handle_icl_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc, icl_ua;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_get_charge_param(chg, &chg->param.icl_stat, &icl_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if (chg->mode != PARALLEL_MASTER)
		return IRQ_HANDLED;

	chg->input_limited_fcc_ua = div64_s64(
			(s64)icl_ua * MICRO_5V * EFFICIENCY_PCT,
			(s64)get_effective_result(chg->fv_votable) * 100);

	if (!get_effective_result(chg->pl_disable_votable))
		rerun_election(chg->fcc_votable);

	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER,
		icl_ua >= USB_WEAK_INPUT_UA, 0);

	return IRQ_HANDLED;
}

static void smblib_handle_slow_plugin_timeout(struct smb_charger *chg,
					      bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: slow-plugin-timeout %s\n",
		   rising ? "rising" : "falling");
}

static void smblib_handle_sdp_enumeration_done(struct smb_charger *chg,
					       bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: sdp-enumeration-done %s\n",
		   rising ? "rising" : "falling");
}

#define QC3_PULSES_FOR_6V	5
#define QC3_PULSES_FOR_9V	20
#define QC3_PULSES_FOR_12V	35
static void smblib_hvdcp_adaptive_voltage_change(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	int pulses;

	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_CHANGE_STATUS rc=%d\n", rc);
			return;
		}

		switch (stat & QC_2P0_STATUS_MASK) {
		case QC_5V_BIT:
			smblib_set_opt_freq_buck(chg, FSW_600HZ_FOR_5V);
			break;
		case QC_9V_BIT:
			smblib_set_opt_freq_buck(chg, FSW_1MHZ_FOR_9V);
			break;
		case QC_12V_BIT:
			smblib_set_opt_freq_buck(chg, FSW_1P2MHZ_FOR_12V);
			break;
		default:
			smblib_set_opt_freq_buck(chg, FSW_1MHZ_FOR_REMOVAL);
			break;
		}
	}

	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_HVDCP_3) {
		rc = smblib_read(chg, QC_PULSE_COUNT_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
			return;
		}
		pulses = (stat & QC_PULSE_COUNT_MASK);

		if (pulses < QC3_PULSES_FOR_6V)
			smblib_set_opt_freq_buck(chg, FSW_600HZ_FOR_5V);
		else if (pulses < QC3_PULSES_FOR_9V)
			smblib_set_opt_freq_buck(chg, FSW_800HZ_FOR_6V_8V);
		else if (pulses < QC3_PULSES_FOR_12V)
			smblib_set_opt_freq_buck(chg, FSW_1MHZ_FOR_9V);
		else
			smblib_set_opt_freq_buck(chg, FSW_1P2MHZ_FOR_12V);

	}
}

static void smblib_handle_adaptive_voltage_done(struct smb_charger *chg,
						bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: adaptive-voltage-done %s\n",
		   rising ? "rising" : "falling");
}

/* triggers when HVDCP 3.0 authentication has finished */
static void smblib_handle_hvdcp_3p0_auth_done(struct smb_charger *chg,
					      bool rising)
{
	const struct apsd_result *apsd_result;
	int rc;

	if (!rising)
		return;

	/*
	 * Disable AUTH_IRQ_EN_CFG_BIT to receive adapter voltage
	 * change interrupt.
	 */
	rc = smblib_masked_write(chg, USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				 AUTH_IRQ_EN_CFG_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable QC auth setting rc=%d\n", rc);

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, true, 0);

	/* the APSD done handler will set the USB supply type */
	apsd_result = smblib_get_apsd_result(chg);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-3p0-auth-done rising; %s detected\n",
		   apsd_result->name);
}

static void smblib_handle_hvdcp_check_timeout(struct smb_charger *chg,
					      bool rising, bool qc_charger)
{
	/* Hold off PD only until hvdcp 2.0 detection timeout */
	if (rising) {
		vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
								false, 0);
		if (get_effective_result(chg->pd_disallowed_votable_indirect))
			/* could be a legacy cable, try doing hvdcp */
			try_rerun_apsd_for_hvdcp(chg);
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: smblib_handle_hvdcp_check_timeout %s\n",
		   rising ? "rising" : "falling");
}

/* triggers when HVDCP is detected */
static void smblib_handle_hvdcp_detect_done(struct smb_charger *chg,
					    bool rising)
{
	if (!rising)
		return;

	/* the APSD done handler will set the USB supply type */
	cancel_delayed_work_sync(&chg->hvdcp_detect_work);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-detect-done %s\n",
		   rising ? "rising" : "falling");
}

#define HVDCP_DET_MS 2500
static void smblib_handle_apsd_done(struct smb_charger *chg, bool rising)
{
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161109 Charging porting */
	int temp_region;
#endif
	const struct apsd_result *apsd_result;

	if (!rising)
		return;

	apsd_result = smblib_update_usb_type(chg);
	switch (apsd_result->bit) {
	case SDP_CHARGER_BIT:
	case CDP_CHARGER_BIT:
	case OCP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		/* if not DCP then no hvdcp timeout happens. Enable pd here */
		vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
				false, 0);
		break;
	case DCP_CHARGER_BIT:
		if (chg->wa_flags & QC_CHARGER_DETECTION_WA_BIT)
			schedule_delayed_work(&chg->hvdcp_detect_work,
					      msecs_to_jiffies(HVDCP_DET_MS));
		break;
	default:
		break;
	}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	temp_region = op_battery_temp_region_get(chg);
	if (temp_region != BATT_TEMP_COLD
		&& temp_region != BATT_TEMP_HOT) {
		op_charging_en(chg, true);
	}

	pr_info("apsd result=0x%x, name=%s, psy_type=%d\n",
		apsd_result->bit, apsd_result->name, apsd_result->pst);
	if (apsd_result->bit != SDP_CHARGER_BIT) {
		schedule_delayed_work(&chg->check_switch_dash_work,
					msecs_to_jiffies(500));
	}

	/* set allow read extern fg IIC */
	set_property_on_fg(chg,
		POWER_SUPPLY_PROP_SET_ALLOW_READ_EXTERN_FG_IIC, true);
#endif
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: apsd-done rising; %s detected\n",
		   apsd_result->name);
}

irqreturn_t smblib_handle_usb_source_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", stat);
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	pr_info("APSD_STATUS=0x%02x\n", stat);
#endif

	smblib_handle_apsd_done(chg,
		(bool)(stat & APSD_DTC_STATUS_DONE_BIT));

	smblib_handle_hvdcp_detect_done(chg,
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_check_timeout(chg,
		(bool)(stat & HVDCP_CHECK_TIMEOUT_BIT),
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_3p0_auth_done(chg,
		(bool)(stat & QC_AUTH_DONE_STATUS_BIT));

	smblib_handle_adaptive_voltage_done(chg,
		(bool)(stat & VADP_CHANGE_DONE_AFTER_AUTH_BIT));

	smblib_handle_sdp_enumeration_done(chg,
		(bool)(stat & ENUMERATION_DONE_BIT));

	smblib_handle_slow_plugin_timeout(chg,
		(bool)(stat & SLOW_PLUGIN_TIMEOUT_BIT));

	smblib_hvdcp_adaptive_voltage_change(chg);

	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

static void typec_source_removal(struct smb_charger *chg)
{
	int rc;

	vote(chg->pl_disable_votable, TYPEC_SRC_VOTER, true, 0);
	/* reset both usbin current and voltage votes */
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	/* reset taper_end voter here */
	vote(chg->pl_disable_votable, TAPER_END_VOTER, false, 0);

	cancel_delayed_work_sync(&chg->hvdcp_detect_work);

	/* reset AUTH_IRQ_EN_CFG_BIT */
	rc = smblib_masked_write(chg, USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				 AUTH_IRQ_EN_CFG_BIT, AUTH_IRQ_EN_CFG_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable QC auth setting rc=%d\n", rc);

	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG,
			  USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V rc=%d\n",
			rc);

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;

	/* clear USB ICL vote for PD_VOTER */
	rc = vote(chg->usb_icl_votable, PD_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't un-vote for USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for USB_PSY_VOTER */
	rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't un-vote for USB ICL rc=%d\n", rc);
}

static void typec_source_insertion(struct smb_charger *chg)
{
	vote(chg->pl_disable_votable, TYPEC_SRC_VOTER, false, 0);
}

static void typec_sink_insertion(struct smb_charger *chg)
{
	/* when a sink is inserted we should not wait on hvdcp timeout to
	 * enable pd
	 */
	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
			false, 0);
}

static void smblib_handle_typec_removal(struct smb_charger *chg)
{
	vote(chg->pd_disallowed_votable_indirect, CC_DETACHED_VOTER, true, 0);
	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER, true, 0);
	vote(chg->pd_disallowed_votable_indirect, LEGACY_CABLE_VOTER, true, 0);
	vote(chg->pd_disallowed_votable_indirect, VBUS_CC_SHORT_VOTER, true, 0);

	/* reset votes from vbus_cc_short */
	vote(chg->hvdcp_disable_votable, VBUS_CC_SHORT_VOTER, true, 0);

	vote(chg->hvdcp_disable_votable, PD_INACTIVE_VOTER, true, 0);

	/*
	 * cable could be removed during hard reset, remove its vote to
	 * disable apsd
	 */
	vote(chg->apsd_disable_votable, PD_HARD_RESET_VOTER, false, 0);

	typec_source_removal(chg);

	smblib_update_usb_type(chg);
}

static void smblib_handle_typec_insertion(struct smb_charger *chg,
		bool sink_attached, bool legacy_cable)
{
	int rp;
	bool vbus_cc_short = false;

	vote(chg->pd_disallowed_votable_indirect, CC_DETACHED_VOTER, false, 0);

	if (sink_attached) {
		typec_source_removal(chg);
		typec_sink_insertion(chg);
	} else {
		typec_source_insertion(chg);
	}

	vote(chg->pd_disallowed_votable_indirect, LEGACY_CABLE_VOTER,
			legacy_cable, 0);

	if (legacy_cable) {
		rp = smblib_get_prop_ufp_mode(chg);
		if (rp == POWER_SUPPLY_TYPEC_SOURCE_HIGH
				|| rp == POWER_SUPPLY_TYPEC_NON_COMPLIANT) {
			vbus_cc_short = true;
			smblib_err(chg, "Disabling PD and HVDCP, VBUS-CC shorted, rp = %d found\n",
					rp);
		}
	}

	vote(chg->hvdcp_disable_votable, VBUS_CC_SHORT_VOTER, vbus_cc_short, 0);
	vote(chg->pd_disallowed_votable_indirect, VBUS_CC_SHORT_VOTER,
			vbus_cc_short, 0);
}

static void smblib_handle_typec_debounce_done(struct smb_charger *chg,
			bool rising, bool sink_attached, bool legacy_cable)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (rising)
		smblib_handle_typec_insertion(chg, sink_attached, legacy_cable);
	else
		smblib_handle_typec_removal(chg);

	rc = smblib_get_prop_typec_mode(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "Couldn't get prop typec mode rc=%d\n", rc);

	/*
	 * HW BUG - after cable is removed, medium or high rd reading
	 * falls to std. Use it for signal of typec cc detachment in
	 * software WA.
	 */
	if (chg->cc2_sink_detach_flag == CC2_SINK_MEDIUM_HIGH
		&& pval.intval == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT) {

		chg->cc2_sink_detach_flag = CC2_SINK_WA_DONE;

		rc = smblib_masked_write(chg,
				TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				EXIT_SNK_BASED_ON_CC_BIT, 0);
		if (rc < 0)
			smblib_err(chg, "Couldn't get prop typec mode rc=%d\n",
				rc);
	}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("IRQ: debounce-done %s; Type-C %s detected\n",
#else
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: debounce-done %s; Type-C %s detected\n",
#endif
		   rising ? "rising" : "falling",
		   smblib_typec_mode_name[pval.intval]);
}

irqreturn_t smblib_handle_usb_typec_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat4, stat5;
	bool debounce_done, sink_attached, legacy_cable;

	/* WA - not when PD hard_reset WIP on cc2 in sink mode */
	if (chg->cc2_sink_detach_flag == CC2_SINK_STD)
		return IRQ_HANDLED;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat4);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat5);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_5 rc=%d\n", rc);
		return IRQ_HANDLED;
	}

#ifdef VENDOR_EDIT
	/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("TYPE_C_STATUS_4=0x%02x, TYPE_C_STATUS_5=0x%02x\n",
			stat4, stat5);
#endif

	debounce_done = (bool)(stat4 & TYPEC_DEBOUNCE_DONE_STATUS_BIT);
	sink_attached = (bool)(stat4 & UFP_DFP_MODE_STATUS_BIT);
	legacy_cable = (bool)(stat5 & TYPEC_LEGACY_CABLE_STATUS_BIT);

	smblib_handle_typec_debounce_done(chg,
			debounce_done, sink_attached, legacy_cable);

	if (stat4 & TYPEC_VBUS_ERROR_STATUS_BIT)
		smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s vbus-error\n",
			   irq_data->name);

	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat4);
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_5 = 0x%02x\n", stat5);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_dc_plugin(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	power_supply_changed(chg->dc_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_high_duty_cycle(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	chg->is_hdc = true;
	schedule_delayed_work(&chg->clear_hdc_work, msecs_to_jiffies(60));

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_switcher_power_ok(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat;

	if (!(chg->wa_flags & BOOST_BACK_WA))
		return IRQ_HANDLED;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if ((stat & USE_USBIN_BIT) &&
				get_effective_result(chg->usb_suspend_votable))
		return IRQ_HANDLED;

	if (stat & USE_DCIN_BIT)
		return IRQ_HANDLED;

	if (is_storming(&irq_data->storm_data)) {
		smblib_err(chg, "Reverse boost detected: suspending input\n");
		vote(chg->usb_suspend_votable, BOOST_BACK_VOTER, true, 0);
	}

	return IRQ_HANDLED;
}

/***************
 * Work Queues *
 ***************/

static void smblib_hvdcp_detect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
					       hvdcp_detect_work.work);

	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
				false, 0);
	if (get_effective_result(chg->pd_disallowed_votable_indirect))
		/* pd is still disabled, try hvdcp */
		try_rerun_apsd_for_hvdcp(chg);
	else
		/* notify pd now that pd is allowed */
		power_supply_changed(chg->usb_psy);
}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
irqreturn_t smblib_handle_aicl_done(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int icl_ma, rc;

	rc = smblib_get_charge_param(chg,
				&chg->param.icl_stat, &icl_ma);
	if (rc < 0) {
		pr_err("Couldn't get ICL status rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	pr_info("IRQ: %s AICL result=%d\n", irq_data->name, icl_ma);
	return IRQ_HANDLED;
}

static int get_property_from_fg(struct smb_charger *chg,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chg->bms_psy)
		chg->bms_psy = power_supply_get_by_name("bms");

	if (chg->bms_psy) {
		rc = power_supply_get_property(chg->bms_psy, prop, &ret);
		if (rc) {
			pr_err("bms psy doesn't support reading prop %d rc = %d\n",
				prop, rc);
			return rc;
		}
		*val = ret.intval;
	} else {
		pr_err("no bms psy found\n");
		return -EINVAL;
	}

	return rc;
}

static int set_property_on_fg(struct smb_charger *chg,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chg->bms_psy)
		chg->bms_psy = power_supply_get_by_name("bms");

	if (chg->bms_psy) {
		ret.intval = val;
		rc = power_supply_set_property(chg->bms_psy, prop, &ret);
		if (rc)
			pr_err("bms psy does not allow updating prop %d rc = %d\n",
				prop, rc);
	} else {
		pr_err("no bms psy found\n");
		return -EINVAL;
	}

	return rc;
}

static int op_charging_en(struct smb_charger *chg, bool en)
{
	int rc;

	pr_err("enable=%d\n", en);
	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				 CHARGING_ENABLE_CMD_BIT,
				 en ? CHARGING_ENABLE_CMD_BIT : 0);
	if (rc < 0) {
		pr_err("Couldn't %s charging rc=%d\n",
			en ? "enable" : "disable", rc);
		return rc;
	}

	return 0;
}

static bool is_usb_present(struct smb_charger *chg)
{
	int rc = 0;
	u8 stat;

//	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		pr_err("Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}
	pr_debug("TYPE_C_STATUS_4 = 0x%02x\n", stat);

//	return (bool)(stat & CC_ATTACHED_BIT);
	return (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
}

static bool op_get_fast_low_temp_full(struct smb_charger *chg)
{
	if (fast_charger && fast_charger->get_fast_low_temp_full)
		return fast_charger->get_fast_low_temp_full();
	else {
		pr_err("no fast_charger register found\n");
		return false;
	}
}

static bool get_fastchg_firmware_updated_status(struct smb_charger *chg)
{
	if (fast_charger && fast_charger->get_fastchg_firmware_already_updated)
		return fast_charger->get_fastchg_firmware_already_updated();
	else {
		pr_err("no fast_charger register found\n");
		return false;
	}
}

static bool get_prop_fast_switch_to_normal(struct smb_charger *chg)
{
	if (fast_charger && fast_charger->fast_switch_to_normal)
		return fast_charger->fast_switch_to_normal();
	else {
		pr_err("no fast_charger register found\n");
		return false;
	}
}


bool is_fastchg_allowed(struct smb_charger *chg)
{
	int temp;
	static int pre_temp = 0;
	bool low_temp_full, switch_to_normal, fw_updated;

	temp = get_prop_batt_temp(chg);
	low_temp_full = op_get_fast_low_temp_full(chg);
	fw_updated = get_fastchg_firmware_updated_status(chg);

	if (!fw_updated)
		return false;

	if (temp < 165 || temp > 430) {
		if (temp != pre_temp) {
			pr_err("temp=%d is not allow to swith fastchg\n", temp);
		}
		pre_temp = temp;
		return false;
	}

	switch_to_normal = get_prop_fast_switch_to_normal(chg);
	if (switch_to_normal)
		return false;

	return true;
}

bool get_oem_charge_done_status(void)
{
	if (g_chg)
		return g_chg->chg_done;
	else
		return false;
}

int update_dash_unplug_status(void)
{
	/* TODO: check if vbus > 2.5v */
	pr_info("\n");

	return 0;
}


bool get_prop_fast_chg_started(struct smb_charger *chg)
{
	if (fast_charger && fast_charger->fast_chg_started)
		return fast_charger->fast_chg_started();
	else
		pr_err("no fast_charger register found\n");

	return false;
}

static bool set_prop_fast_switch_to_normal_false(struct smb_charger *chg)
{
	if (fast_charger && fast_charger->set_switch_to_noraml_false)
		return fast_charger->set_switch_to_noraml_false();
	else
		pr_err("no fast_charger register found\n");

	return false;
}

bool op_get_fastchg_ing(struct smb_charger *chg)
{
	if (fast_charger && fast_charger->get_fast_chg_ing)
		return fast_charger->get_fast_chg_ing();
	else
		pr_err("no fast_charger register found\n");

	return false;
}

bool op_set_fast_chg_allow(struct smb_charger *chg, bool enable)
{
	if (fast_charger && fast_charger->set_fast_chg_allow)
		return fast_charger->set_fast_chg_allow(enable);
	else
		pr_err("no fast_charger register found\n");

	return false;
}

static bool op_get_fast_chg_allow(struct smb_charger *chg)
{
	if (fast_charger && fast_charger->get_fast_chg_allow)
		return fast_charger->get_fast_chg_allow();
	else
		pr_err("no fast_charger register found\n");

	return false;
}

static bool op_is_usb_switch_on(struct smb_charger *chg)
{
	if (fast_charger && fast_charger->is_usb_switch_on)
		return fast_charger->is_usb_switch_on();
	else
		pr_err("no fast_charger register found\n");

	return false;
}

static enum batt_status_type op_battery_status_get(struct smb_charger *chg)
{
	return chg->battery_status;
}

static temp_region_type op_battery_temp_region_get(struct smb_charger *chg)
{
	return chg->mBattTempRegion;
}

int fuelgauge_battery_temp_region_get(void)
{
	if (!g_chg)
		return BATT_TEMP_NORMAL;

	return op_battery_temp_region_get(g_chg);
}

static void op_battery_status_set(struct smb_charger *chg,
		enum batt_status_type battery_status)
{
	chg->battery_status = battery_status;
}

static void op_battery_temp_region_set(struct smb_charger *chg,
		temp_region_type batt_temp_region)
{
	chg->mBattTempRegion = batt_temp_region;
	pr_err("set temp_region=%d\n", chg->mBattTempRegion);
}

static void set_prop_batt_health(struct smb_charger *chg, int batt_health)
{
	chg->batt_health = batt_health;
}

static void set_usb_switch(struct smb_charger *chg, bool enable)
{
	if (!fast_charger) {
		pr_err("no fast_charger register found\n");
		return;
	}

	if (enable) {
		pr_err("switch on fastchg\n");
		set_mcu_en_gpio_value(1);
		msleep(10);
		usb_sw_gpio_set(1);
		msleep(10);
		mcu_en_gpio_set(0);
	} else {
		pr_err("switch off fastchg\n");
		usb_sw_gpio_set(0);
		mcu_en_gpio_set(1);
	}
}

static void switch_fast_chg(struct smb_charger *chg)
{
	bool fastchg_allowed, is_allowed;

	if (op_is_usb_switch_on(chg))
		return;
	if (!is_usb_present(chg))
		return;

	fastchg_allowed = op_get_fast_chg_allow(chg);
	if (!fastchg_allowed) {
		is_allowed = is_fastchg_allowed(chg);
		if (is_allowed) {
			set_usb_switch(chg, true);
			op_set_fast_chg_allow(chg, true);
		}
	}
}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161117 Fix dash in power off charging mode */
static void op_re_kick_allowed_voltage(struct smb_charger  *chg)
{
	const struct apsd_result *apsd_result;

	if (!is_usb_present(chg))
		return;

	apsd_result = smblib_get_apsd_result(chg);
	if (apsd_result->bit == SDP_CHARGER_BIT)
		return;

	pr_info("re-kick allowed voltage\n");
	smblib_set_usb_pd_allowed_voltage(chg, MICRO_9V, MICRO_9V);
	msleep(500);
	smblib_set_usb_pd_allowed_voltage(chg, MICRO_5V, MICRO_5V);
}

static void op_re_kick_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work,
			struct smb_charger,
			re_kick_work.work);

    if (chg->vbus_present) {
		op_re_kick_allowed_voltage(chg);
		schedule_delayed_work(&chg->check_switch_dash_work,
				msecs_to_jiffies(500));
	}
}
#endif

static void op_check_allow_switch_dash_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb_charger *chg = container_of(dwork,
			struct smb_charger, check_switch_dash_work);
	const struct apsd_result *apsd_result;

	if (!is_usb_present(chg))
		return;

	apsd_result = smblib_get_apsd_result(chg);
	if ((apsd_result->bit != SDP_CHARGER_BIT && apsd_result->bit != CDP_CHARGER_BIT) && apsd_result->bit)
		switch_fast_chg(chg);
}

int check_allow_switch_dash(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	schedule_delayed_work(&chg->check_switch_dash_work,
				msecs_to_jiffies(500));
	return 0;
}

#define DEFAULT_WALL_CHG_MA	1800
static int set_dash_charger_present(int status)
{
	int charger_present;
	bool pre_dash_present;

	if (g_chg) {
		pre_dash_present = g_chg->dash_present;
		charger_present = is_usb_present(g_chg);
		g_chg->dash_present = status && charger_present;
		if (g_chg->dash_present && !pre_dash_present) {
			pr_err("set dash online\n");
			g_chg->usb_psy_desc.type = POWER_SUPPLY_TYPE_DASH;
			vote(g_chg->usb_icl_votable, PD_VOTER, true,
					DEFAULT_WALL_CHG_MA * 1000);
		}
		power_supply_changed(g_chg->batt_psy);
	} else {
		pr_err("set_dash_charger_present error\n");
	}

	return 0;
}

static void op_check_charge_timeout(struct smb_charger *chg)
{
	static int batt_status, count = 0;

	if (chg->chg_done)
		return;

	batt_status = get_prop_batt_status(chg);
	if (chg->vbus_present
			&& batt_status == POWER_SUPPLY_STATUS_CHARGING)
		count++;
	else
		count = 0;

	if (count > CHG_TIMEOUT_COUNT) {
		pr_err("chg timeout! stop chaging now\n");
		op_charging_en(chg, false);
		chg->time_out = true;
	}
}

static int get_prop_charger_voltage_now(struct smb_charger *chg)
{
	int vchg_uv = 0;

	if(!is_usb_present(chg))
		return 0;

	if (chg->fake_chgvol)
		return chg->fake_chgvol;

	if (!chg->iio.usbin_v_chan ||
		PTR_ERR(chg->iio.usbin_v_chan) == -EPROBE_DEFER)
		chg->iio.usbin_v_chan = iio_channel_get(chg->dev, "usbin_v");

	if (IS_ERR(chg->iio.usbin_v_chan)) {
		pr_err("failed to get usbin_v iio_channel");
		return 0;
	}

	iio_read_channel_processed(chg->iio.usbin_v_chan, &vchg_uv);
	if (chg->vbus_present && vchg_uv == 0)
		return 5000;

	return vchg_uv / 1000;
}

static int get_prop_batt_present(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		pr_err("Couldn't read BATIF_INT_RT_STS rc=%d\n", rc);
		return rc;
	}

	return !(stat & (BAT_THERM_OR_ID_MISSING_RT_STS_BIT
					| BAT_TERMINAL_MISSING_RT_STS_BIT));
}

#define DEFAULT_BATT_CAPACITY	50
static int get_prop_batt_capacity(struct smb_charger *chg)
{
	int capacity, rc;

	if (chg->fake_capacity >= 0)
		return chg->fake_capacity;

	rc = get_property_from_fg(chg, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (rc) {
		pr_err("Couldn't get capacity rc=%d\n", rc);
		capacity = DEFAULT_BATT_CAPACITY;
	}

	return capacity;
}

#define DEFAULT_BATT_TEMP		200
static int get_prop_batt_temp(struct smb_charger *chg)
{
	int temp, rc;

	if (chg->use_fake_temp)
		return chg->fake_temp;

	rc = get_property_from_fg(chg, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_err("Couldn't get temperature rc=%d\n", rc);
		temp = DEFAULT_BATT_TEMP;
	}

	return temp;
}

#define DEFAULT_BATT_CURRENT_NOW	0
static int get_prop_batt_current_now(struct smb_charger *chg)
{
	int ua, rc;

	rc = get_property_from_fg(chg, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc) {
		pr_err("Couldn't get current rc=%d\n", rc);
		ua = DEFAULT_BATT_CURRENT_NOW;
	}

	return ua;
}

#define DEFAULT_BATT_VOLTAGE_NOW	0
static int get_prop_batt_voltage_now(struct smb_charger *chg)
{
	int uv, rc;

	rc = get_property_from_fg(chg, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc) {
		pr_err("Couldn't get voltage rc=%d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_NOW;
	}

	return uv;
}

int get_prop_batt_status(struct smb_charger *chg)
{
	int capacity, batt_status, rc;
	temp_region_type temp_region;
	union power_supply_propval pval = {0, };

	temp_region = op_battery_temp_region_get(chg);
	capacity = get_prop_batt_capacity(chg);
	chg->dash_on = get_prop_fast_chg_started(chg);
	if ((chg->chg_done || chg->recharge_status)
			&& (temp_region == BATT_TEMP_COOL
			|| temp_region == BATT_TEMP_LITTLE_COOL
			|| temp_region == BATT_TEMP_PRE_NORMAL
			|| temp_region == BATT_TEMP_NORMAL)
			&& capacity > 90) {
		return POWER_SUPPLY_STATUS_FULL;
	} else if (chg->dash_on) {
		return POWER_SUPPLY_STATUS_CHARGING;
	}

	rc = smblib_get_prop_batt_status(chg, &pval);
	if (rc)
		batt_status = 0;
	else
		batt_status = pval.intval;

	return batt_status;
}

int get_charging_status(void)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (!g_chg)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	rc = smblib_get_prop_batt_status(g_chg, &pval);
	if (rc)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	return pval.intval;
}

void set_chg_ibat_vbat_max(struct smb_charger *chg, int ibat, int vfloat )
{
	pr_err("set ibatmax=%d and set vbatmax=%d\n",
			ibat, vfloat);

	vote(chg->fcc_votable,
		DEFAULT_VOTER, true, ibat * 1000);
	vote(chg->fv_votable,
		DEFAULT_VOTER, true, vfloat * 1000);

	/* set cc to cv 100mv lower than vfloat */
	set_property_on_fg(chg, POWER_SUPPLY_PROP_CC_TO_CV_POINT, vfloat - 100);
}

/* Tbatt < -3C */
static int handle_batt_temp_cold(struct smb_charger *chg)
{
	temp_region_type temp_region;

	temp_region = op_battery_temp_region_get(chg);
	if (temp_region != BATT_TEMP_COLD || chg->is_power_changed) {
		pr_err("triggered\n");
		chg->is_power_changed = false;

		op_charging_en(chg, false);
		op_battery_temp_region_set(chg, BATT_TEMP_COLD);

		/* Update the temperature boundaries */
		chg->mBattTempBoundT0 = chg->BATT_TEMP_T0 + BATT_TEMP_HYST;
		chg->mBattTempBoundT1 = chg->BATT_TEMP_T1;
		chg->mBattTempBoundT2 = chg->BATT_TEMP_T2;
		chg->mBattTempBoundT3 = chg->BATT_TEMP_T3;
		chg->mBattTempBoundT4 = chg->BATT_TEMP_T4;
		chg->mBattTempBoundT5 = chg->BATT_TEMP_T5;
		chg->mBattTempBoundT6 = chg->BATT_TEMP_T6;
		set_prop_batt_health(chg, POWER_SUPPLY_HEALTH_COLD);
	}

	return 0;
}

/* -3C <= Tbatt <= 0C */
static int handle_batt_temp_little_cold(struct smb_charger *chg)
{
	temp_region_type temp_region;

	if (chg->chg_ovp)
		return 0;

	temp_region = op_battery_temp_region_get(chg);
	if (temp_region != BATT_TEMP_LITTLE_COLD
			|| chg->is_power_changed || chg->recharge_pending) {
		pr_err("triggered\n");
		chg->recharge_pending = false;
		chg->is_power_changed = false;

		if (temp_region == BATT_TEMP_HOT ||
				temp_region == BATT_TEMP_COLD)
			op_charging_en(chg, true);

		set_chg_ibat_vbat_max(chg,
				chg->ibatmax[BATT_TEMP_LITTLE_COLD],
				chg->vbatmax[BATT_TEMP_LITTLE_COLD]);
		op_battery_temp_region_set(chg, BATT_TEMP_LITTLE_COLD);

		/* Update the temperature boundaries */
		chg->mBattTempBoundT0 = chg->BATT_TEMP_T0;
		chg->mBattTempBoundT1 = chg->BATT_TEMP_T1 + BATT_TEMP_HYST;
		chg->mBattTempBoundT2 = chg->BATT_TEMP_T2;
		chg->mBattTempBoundT3 = chg->BATT_TEMP_T3;
		chg->mBattTempBoundT4 = chg->BATT_TEMP_T4;
		chg->mBattTempBoundT5 = chg->BATT_TEMP_T5;
		chg->mBattTempBoundT6 = chg->BATT_TEMP_T6;
		set_prop_batt_health(chg, POWER_SUPPLY_HEALTH_GOOD);
	}

	return 0;
}

/* 0C < Tbatt <= 5C*/
static int handle_batt_temp_cool(struct smb_charger *chg)
{
	temp_region_type temp_region;

	if (chg->chg_ovp)
		return 0;

	temp_region = op_battery_temp_region_get(chg);
	if (temp_region != BATT_TEMP_COOL
			|| chg->is_power_changed || chg->recharge_pending) {
		pr_err("triggered\n");
		chg->recharge_pending = false;
		chg->is_power_changed = false;

		if (temp_region == BATT_TEMP_HOT ||
				temp_region == BATT_TEMP_COLD)
			op_charging_en(chg, true);

		set_chg_ibat_vbat_max(chg,
				chg->ibatmax[BATT_TEMP_COOL],
				chg->vbatmax[BATT_TEMP_COOL]);
		op_battery_temp_region_set(chg, BATT_TEMP_COOL);

		/* Update the temperature boundaries */
		chg->mBattTempBoundT0 = chg->BATT_TEMP_T0;
		chg->mBattTempBoundT1 = chg->BATT_TEMP_T1 ;
		chg->mBattTempBoundT2 = chg->BATT_TEMP_T2 + BATT_TEMP_HYST;
		chg->mBattTempBoundT3 = chg->BATT_TEMP_T3;
		chg->mBattTempBoundT4 = chg->BATT_TEMP_T4;
		chg->mBattTempBoundT5 = chg->BATT_TEMP_T5;
		chg->mBattTempBoundT6 = chg->BATT_TEMP_T6;
		set_prop_batt_health(chg, POWER_SUPPLY_HEALTH_GOOD);
	}

	return 0;
}
/* 5C < Tbatt <= 12C */
static int handle_batt_temp_little_cool(struct smb_charger *chg)
{
	int temp_region, vbat_mv;

	if (chg->chg_ovp)
		return 0;

	temp_region = op_battery_temp_region_get(chg);
	if (temp_region != BATT_TEMP_LITTLE_COOL
			|| chg->is_power_changed || chg->recharge_pending) {
		pr_err("triggered\n");
		chg->recharge_pending = false;
		chg->is_power_changed = false;

		if (temp_region == BATT_TEMP_HOT ||
				temp_region == BATT_TEMP_COLD)
			op_charging_en(chg, true);

		vbat_mv = get_prop_batt_voltage_now(chg) / 1000;
		if (vbat_mv > 4180) {
			set_chg_ibat_vbat_max(chg, 450,
					chg->vbatmax[BATT_TEMP_LITTLE_COOL]);
			chg->temp_littel_cool_set_current_0_point_25c = false;
		} else {
			set_chg_ibat_vbat_max(chg,
					chg->ibatmax[BATT_TEMP_LITTLE_COOL],
					chg->vbatmax[BATT_TEMP_LITTLE_COOL]);
			chg->temp_littel_cool_set_current_0_point_25c = true;
		}
		op_battery_temp_region_set(chg, BATT_TEMP_LITTLE_COOL);

		/* Update the temperature boundaries */
		chg->mBattTempBoundT0 = chg->BATT_TEMP_T0;
		chg->mBattTempBoundT1 = chg->BATT_TEMP_T1;
		chg->mBattTempBoundT2 = chg->BATT_TEMP_T2;
		chg->mBattTempBoundT3 = chg->BATT_TEMP_T3 + BATT_TEMP_HYST;
		chg->mBattTempBoundT4 = chg->BATT_TEMP_T4;
		chg->mBattTempBoundT5 = chg->BATT_TEMP_T5;
		chg->mBattTempBoundT6 = chg->BATT_TEMP_T6;
		set_prop_batt_health(chg, POWER_SUPPLY_HEALTH_GOOD);
	}

	return 0;
}

/* 12C < Tbatt < 22C */
static int handle_batt_temp_prenormal(struct smb_charger *chg)
{
	temp_region_type temp_region;

	if (chg->chg_ovp)
		return 0;

	temp_region = op_battery_temp_region_get(chg);
	if (temp_region != BATT_TEMP_PRE_NORMAL
			|| chg->is_power_changed || chg->recharge_pending) {
		pr_err("triggered\n");
		chg->recharge_pending = false;
		chg->is_power_changed = false;

		if (temp_region == BATT_TEMP_HOT ||
				temp_region == BATT_TEMP_COLD)
			op_charging_en(chg, true);

		set_chg_ibat_vbat_max(chg,
				chg->ibatmax[BATT_TEMP_PRE_NORMAL],
				chg->vbatmax[BATT_TEMP_PRE_NORMAL]);
		op_battery_temp_region_set(chg, BATT_TEMP_PRE_NORMAL);

		/* Update the temperature boundaries */
		chg->mBattTempBoundT0 = chg->BATT_TEMP_T0;
		chg->mBattTempBoundT1 = chg->BATT_TEMP_T1;
		chg->mBattTempBoundT2 = chg->BATT_TEMP_T2;
		chg->mBattTempBoundT3 = chg->BATT_TEMP_T3;
		chg->mBattTempBoundT4 = chg->BATT_TEMP_T4 + BATT_TEMP_HYST;
		chg->mBattTempBoundT5 = chg->BATT_TEMP_T5;
		chg->mBattTempBoundT6 = chg->BATT_TEMP_T6;
		set_prop_batt_health(chg, POWER_SUPPLY_HEALTH_GOOD);
	}

	return 0;
}

/* 15C < Tbatt < 45C */
static int handle_batt_temp_normal(struct smb_charger *chg)
{
	temp_region_type temp_region;

	if (chg->chg_ovp)
		return 0;

	temp_region = op_battery_temp_region_get(chg);
	if ((temp_region != BATT_TEMP_NORMAL)
			|| chg->is_power_changed || chg->recharge_pending) {
		pr_err("triggered\n");
		chg->recharge_pending = false;
		chg->is_power_changed = false;

		if (temp_region == BATT_TEMP_HOT ||
				temp_region == BATT_TEMP_COLD)
			op_charging_en(chg, true);

		set_chg_ibat_vbat_max(chg,
				chg->ibatmax[BATT_TEMP_NORMAL],
				chg->vbatmax[BATT_TEMP_NORMAL]);
		op_battery_temp_region_set(chg, BATT_TEMP_NORMAL);

		/* Update the temperature boundaries */
		chg->mBattTempBoundT0 = chg->BATT_TEMP_T0;
		chg->mBattTempBoundT1 = chg->BATT_TEMP_T1;
		chg->mBattTempBoundT2 = chg->BATT_TEMP_T2;
		chg->mBattTempBoundT3 = chg->BATT_TEMP_T3;
		chg->mBattTempBoundT4 = chg->BATT_TEMP_T4;
		chg->mBattTempBoundT5 = chg->BATT_TEMP_T5;
		chg->mBattTempBoundT6 = chg->BATT_TEMP_T6;
		set_prop_batt_health(chg, POWER_SUPPLY_HEALTH_GOOD);
	}

	return 0;
}

/* 45C <= Tbatt <= 55C */
static int handle_batt_temp_warm(struct smb_charger *chg)
{
	temp_region_type temp_region;

	if (chg->chg_ovp)
		return 0;

	temp_region = op_battery_temp_region_get(chg);
	if ((temp_region != BATT_TEMP_WARM)
			|| chg->is_power_changed || chg->recharge_pending) {
		pr_err("triggered\n");
		chg->is_power_changed = false;
		chg->recharge_pending = false;

		if (temp_region == BATT_TEMP_HOT ||
				temp_region == BATT_TEMP_COLD)
			op_charging_en(chg, true);

		set_chg_ibat_vbat_max(chg,
				chg->ibatmax[BATT_TEMP_WARM],
				chg->vbatmax[BATT_TEMP_WARM]);
		op_battery_temp_region_set(chg, BATT_TEMP_WARM);

		/* Update the temperature boundaries */
		chg->mBattTempBoundT0 = chg->BATT_TEMP_T0;
		chg->mBattTempBoundT1 = chg->BATT_TEMP_T1;
		chg->mBattTempBoundT2 = chg->BATT_TEMP_T2;
		chg->mBattTempBoundT3 = chg->BATT_TEMP_T3;
		chg->mBattTempBoundT4 = chg->BATT_TEMP_T4;
		chg->mBattTempBoundT5 = chg->BATT_TEMP_T5 - BATT_TEMP_HYST;
		chg->mBattTempBoundT6 = chg->BATT_TEMP_T6;
		set_prop_batt_health(chg, POWER_SUPPLY_HEALTH_GOOD);
	}

	return 0;
}

/* 55C < Tbatt */
static int handle_batt_temp_hot(struct smb_charger *chg)
{
	temp_region_type temp_region;

	temp_region = op_battery_temp_region_get(chg);
	if ((temp_region != BATT_TEMP_HOT)
			|| chg->is_power_changed) {
		pr_err("triggered\n");
		chg->is_power_changed = false;

		op_charging_en(chg, false);
		op_battery_temp_region_set(chg, BATT_TEMP_HOT);

		/* Update the temperature boundaries */
		chg->mBattTempBoundT0 = chg->BATT_TEMP_T0;
		chg->mBattTempBoundT1 = chg->BATT_TEMP_T1;
		chg->mBattTempBoundT2 = chg->BATT_TEMP_T2;
		chg->mBattTempBoundT3 = chg->BATT_TEMP_T3;
		chg->mBattTempBoundT4 = chg->BATT_TEMP_T4;
		chg->mBattTempBoundT5 = chg->BATT_TEMP_T5;
		chg->mBattTempBoundT6 = chg->BATT_TEMP_T6 - BATT_TEMP_HYST; /* from hot to warm */
		set_prop_batt_health(chg, POWER_SUPPLY_HEALTH_OVERHEAT);
	}

	return 0;
}

static int op_check_battery_temp(struct smb_charger *chg)
{
	int temp, rc = -1;

	if(!chg->vbus_present)
		return rc;

	temp = get_prop_batt_temp(chg);
	if (temp < chg->mBattTempBoundT0) /* COLD */
		rc = handle_batt_temp_cold(chg);
	else if (temp >=  chg->mBattTempBoundT0 &&
			temp < chg->mBattTempBoundT1) /* LITTLE_COLD */
		rc = handle_batt_temp_little_cold(chg);
	else if (temp >=  chg->mBattTempBoundT1 &&
			temp < chg->mBattTempBoundT2) /* COOL */
		rc = handle_batt_temp_cool(chg);
	else if (temp >= chg->mBattTempBoundT2 &&
			temp < chg->mBattTempBoundT3) /* LITTLE_COOL */
		rc = handle_batt_temp_little_cool(chg);
	else if (temp >= chg->mBattTempBoundT3 &&
			temp < chg->mBattTempBoundT4) /* PRE_NORMAL */
		rc = handle_batt_temp_prenormal(chg);
	else if (temp >= chg->mBattTempBoundT4 &&
			temp < chg->mBattTempBoundT5) /* NORMAL */
		rc = handle_batt_temp_normal(chg);
	else if (temp >= chg->mBattTempBoundT5 &&
			temp <=  chg->mBattTempBoundT6) /* WARM */
		rc = handle_batt_temp_warm(chg);
	else if (temp > chg->mBattTempBoundT6) /* HOT */
		rc = handle_batt_temp_hot(chg);

	return rc;
}

void op_charge_info_init(struct smb_charger *chg)
{
	op_battery_temp_region_set(chg, BATT_TEMP_NORMAL);

	chg->mBattTempBoundT0 = chg->BATT_TEMP_T0;
	chg->mBattTempBoundT1 = chg->BATT_TEMP_T1;
	chg->mBattTempBoundT2 = chg->BATT_TEMP_T2;
	chg->mBattTempBoundT3 = chg->BATT_TEMP_T3;
	chg->mBattTempBoundT4 = chg->BATT_TEMP_T4;
	chg->mBattTempBoundT5 = chg->BATT_TEMP_T5;
	chg->mBattTempBoundT6 = chg->BATT_TEMP_T6;
	chg->chg_ovp = false;
	chg->is_power_changed = false;
	chg->chg_done = false;
	chg->recharge_pending = false;
	chg->recharge_status = false;
	chg->temp_littel_cool_set_current_0_point_25c = false;
	chg->oem_lcd_is_on = false;
	chg->time_out = false;
	chg->battery_status = BATT_STATUS_GOOD;
	chg->disable_normal_chg_for_dash = false;
	chg->usb_enum_status = false;
	chg->non_std_chg_present = false;
}

static int op_handle_battery_uovp(struct smb_charger *chg)
{
	pr_err("vbat is over voltage, stop charging\n");
	set_prop_batt_health(chg, POWER_SUPPLY_HEALTH_OVERVOLTAGE);
	op_charging_en(chg, false);

	return 0;
}

static int op_handle_battery_restore_from_uovp(struct smb_charger *chg)
{
	pr_err("vbat is back to normal, start charging\n");
	/* restore charging form battery ovp */
	op_charging_en(chg, true);
	set_prop_batt_health(chg, POWER_SUPPLY_HEALTH_GOOD);

	return 0;
}

static void op_check_battery_uovp(struct smb_charger *chg)
{
	int vbat_mv = 0;
	enum batt_status_type battery_status_pre;

	if (!chg->vbus_present)
		return;

	battery_status_pre = op_battery_status_get(chg);
	vbat_mv = get_prop_batt_voltage_now(chg) / 1000;
	pr_debug("bat vol:%d\n", vbat_mv);
	if (vbat_mv > BATT_SOFT_OVP_MV) {
		if (battery_status_pre == BATT_STATUS_GOOD) {
			pr_err("BATTERY_SOFT_OVP_VOLTAGE\n");
			op_battery_status_set(chg, BATT_STATUS_BAD);
			op_handle_battery_uovp(chg);
		}
	}
	else {
		if (battery_status_pre == BATT_STATUS_BAD) {
			pr_err("battery_restore_from_uovp\n");
			op_battery_status_set(chg, BATT_STATUS_GOOD);
			op_handle_battery_restore_from_uovp(chg);
			//smbchg_rerun_aicl(chip);
		}
	}

	return;
}

static void op_check_charger_uovp(struct smb_charger *chg)
{
	static int over_volt_count = 0, not_over_volt_count = 0;
	static bool uovp_satus, pre_uovp_satus;
	int vchg_mv = CHG_VOLTAGE_NORMAL;
	int detect_time = 3; /* 3 x 6s = 18s */

	if (!chg->vbus_present)
		return;

	vchg_mv = get_prop_charger_voltage_now(chg);
	pr_debug("charger_voltage=%d charger_ovp=%d\n", vchg_mv, chg->chg_ovp);

	if (!chg->chg_ovp) {
		if (vchg_mv > CHG_SOFT_OVP_MV || vchg_mv <= CHG_SOFT_UVP_MV) {
			pr_err("charger is over voltage, count=%d\n", over_volt_count);
			uovp_satus = true;
			if (pre_uovp_satus)
				over_volt_count++;
			else
				over_volt_count = 0;

			pr_err("uovp_satus=%d, pre_uovp_satus=%d, over_volt_count=%d\n",
					uovp_satus, pre_uovp_satus, over_volt_count);
			if (detect_time <= over_volt_count) {
				/* vchg continuous higher than 5.8v */
				pr_err("charger is over voltage, stop charging\n");
				op_charging_en(chg, false);
				chg->chg_ovp = true;
			}
		}
	} else {
		if (vchg_mv < CHG_SOFT_OVP_MV - 100
				&& vchg_mv > CHG_SOFT_UVP_MV + 100) {
			uovp_satus = false;
			if (!pre_uovp_satus)
				not_over_volt_count++;
			else
				not_over_volt_count = 0;

			pr_err("uovp_satus=%d, pre_uovp_satus=%d,not_over_volt_count=%d\n",
					uovp_satus, pre_uovp_satus, not_over_volt_count);
			if (detect_time <= not_over_volt_count) {
				/* vchg continuous lower than 5.7v */
				pr_err("charger voltage is back to normal\n");
				op_charging_en(chg, true);
				chg->chg_ovp = false;
				op_check_battery_temp(chg);
				//smbchg_rerun_aicl(chg);
			}
		}
	}
	pre_uovp_satus = uovp_satus;
	return;
}

#define SOFT_CHG_TERM_CURRENT 100 /* 100MA */
void checkout_term_current(struct smb_charger *chg, int batt_temp)
{
	static int term_current_reached = 0;
	int current_ma, voltage_mv, temp_region, batt_status;

	batt_status = get_prop_batt_status(chg);
	if (batt_status != POWER_SUPPLY_STATUS_CHARGING)
		return;

	current_ma = get_prop_batt_current_now(chg) / 1000;
	if (!(current_ma >= -SOFT_CHG_TERM_CURRENT
			&& current_ma <= SOFT_CHG_TERM_CURRENT)) {
		/* soft charge term set to 100mA */
		term_current_reached = 0;
		return;
	}

	voltage_mv = get_prop_batt_voltage_now(chg) / 1000;
	temp_region = op_battery_temp_region_get(chg);
	if (voltage_mv >= chg->vbatmax[temp_region]) {
		term_current_reached++;
	} else {
		term_current_reached = 0;
		return;
	}

	if (term_current_reached >= 5) {
		//smbchg_charging_status_change(chg);
		//set_property_on_fg(chg, POWER_SUPPLY_PROP_CHARGE_DONE, 1);
		chg->chg_done = true;
		term_current_reached = 0;
		pr_err("chg_done: temp=%d, soc_calib=%d, VOLT=%d, current:%d\n",
				get_prop_batt_temp(chg), get_prop_batt_capacity(chg),
				get_prop_batt_voltage_now(chg) / 1000,
				get_prop_batt_current_now(chg) / 1000);
		op_charging_en(chg, false);
	}
}

static void op_heartbeat_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb_charger *chg = container_of(dwork,
			struct smb_charger, heartbeat_work);
	temp_region_type temp_region;
	bool charger_present;
	bool fast_charging;
	static int batt_temp = 0, vbat_mv = 0;

	op_check_charge_timeout(chg);

	charger_present = is_usb_present(chg);
	if (!charger_present)
		goto out;

	/* charger present */
	power_supply_changed(chg->batt_psy);
	chg->dash_on = get_prop_fast_chg_started(chg);
	if (chg->dash_on) {
		switch_fast_chg(chg);
		pr_info("fast chg started, usb_switch=%d\n",
				op_is_usb_switch_on(chg));
		/* add for disable normal charge */
		fast_charging = op_get_fastchg_ing(chg);
		if (fast_charging) {
			if (!chg->disable_normal_chg_for_dash)
				op_charging_en(chg, false);
			chg->disable_normal_chg_for_dash = true;
		}
		goto out;
	} else {
		if (chg->disable_normal_chg_for_dash) {
			chg->disable_normal_chg_for_dash = false;
			op_charging_en(chg, true);
		}
		schedule_delayed_work(&chg->check_switch_dash_work,
							msecs_to_jiffies(100));
	}

	op_check_charger_uovp(chg);
	op_check_battery_uovp(chg);

	vbat_mv = get_prop_batt_voltage_now(chg) / 1000;
	temp_region = op_battery_temp_region_get(chg);
	if (temp_region == BATT_TEMP_LITTLE_COOL) {
		if (vbat_mv > 4180 + 20
				&& chg->temp_littel_cool_set_current_0_point_25c) {
			chg->is_power_changed = true;
		} else if (vbat_mv < 4180 - 10
				&& !chg->temp_littel_cool_set_current_0_point_25c) {
			chg->is_power_changed = true;
		}
	}

	batt_temp = get_prop_batt_temp(chg);
	checkout_term_current(chg, batt_temp);
	if (!chg->chg_ovp && chg->chg_done
			&& temp_region > BATT_TEMP_COLD
			&& temp_region < BATT_TEMP_HOT
			&& chg->vbatdet[temp_region] >= vbat_mv) {
		chg->chg_done = false;
		chg->recharge_pending = true;
		chg->recharge_status = true;

		op_charging_en(chg, true);
		pr_debug("temp_region=%d, recharge_pending\n", temp_region);
	}

	if (!chg->chg_ovp && chg->battery_status == BATT_STATUS_GOOD
			&& !chg->time_out) {
		op_check_battery_temp(chg);
	}

out:
	if (charger_present) {
		pr_info("CAP=%d, VBAT=%d, IBAT=%d, BAT_TEMP=%d, CHG_TYPE=%d, VBUS=%d\n",
				get_prop_batt_capacity(chg),
				get_prop_batt_voltage_now(chg) / 1000,
				get_prop_batt_current_now(chg) / 1000,
				get_prop_batt_temp(chg),
				chg->usb_psy_desc.type,
				get_prop_charger_voltage_now(chg));
	}

	/*update time 6s*/
	schedule_delayed_work(&chg->heartbeat_work,
			round_jiffies_relative(msecs_to_jiffies
				(HEARTBEAT_INTERVAL_MS)));
}

static int load_data(struct smb_charger *chg)
{
	u8 stored_soc = 0;
	int rc = 0, shutdown_soc = 0;

	if (!chg) {
		pr_err("chg is NULL !\n");
		return SOC_INVALID;
	}

	rc = smblib_read(chg, SOC_DATA_REG_0, &stored_soc);
	if (rc) {
		pr_err("failed to read addr[0x%x], rc=%d\n", SOC_DATA_REG_0, rc);
		return SOC_INVALID;
	}

	/* the fist time connect battery, the reg 0x88d is 0x0, we do not need load this data.*/
	if ((stored_soc % 2) == 1)
		shutdown_soc = (stored_soc >> 1 ); /* get data from bit1~bit7 */
	else
		shutdown_soc = SOC_INVALID;

	pr_info("stored_soc[0x%x], shutdown_soc[%d]\n", stored_soc, shutdown_soc);
	return shutdown_soc;
}

int load_soc(void)
{
	int soc = 0;

	soc = load_data(g_chg);
	if (soc == SOC_INVALID || soc < 0 || soc > 100)
		return -1;
	return soc;
}

static void clear_backup_soc(struct smb_charger *chg)
{
	int rc = 0;
	u8 soc_temp = 0;

	rc = smblib_write(chg, SOC_DATA_REG_0, soc_temp);
	if (rc)
		pr_err("failed to clean addr[0x%x], rc=%d\n",
				SOC_DATA_REG_0, rc);
}

void clean_backup_soc_ex(void)
{
	if(g_chg)
		clear_backup_soc(g_chg);
}

static void backup_soc(struct smb_charger *chg, int soc)
{
	int rc = 0;
	u8 invalid_soc = SOC_INVALID;
	u8 soc_temp = (soc << 1) + 1; /* store data in bit1~bit7 */
	if (!chg || soc < 0 || soc > 100) {
		pr_err("chg or soc invalid, store an invalid soc\n");
		if (chg) {
			rc = smblib_write(chg, SOC_DATA_REG_0, invalid_soc);
			if (rc)
				pr_err("failed to write addr[0x%x], rc=%d\n",
						SOC_DATA_REG_0, rc);
		}
		return;
	}

	pr_err("backup_soc[%d]\n", soc);
	rc = smblib_write(chg, SOC_DATA_REG_0, soc_temp);
	if (rc)
		pr_err("failed to write addr[0x%x], rc=%d\n",
				SOC_DATA_REG_0, rc);
}

void backup_soc_ex(int soc)
{
	if (g_chg)
		backup_soc(g_chg, soc);
}

enum chg_protect_status_type {
    PROTECT_CHG_OVP = 1,                  /* 1: VCHG > 5.8V     */
    PROTECT_BATT_MISSING,                 /* 2: battery missing */
    PROTECT_CHG_OVERTIME,                 /* 3: charge overtime */
    PROTECT_BATT_OVP,                     /* 4: vbat >= 4.5     */
    PROTECT_BATT_TEMP_REGION__HOT,        /* 5: 55 < t          */
    PROTECT_BATT_TEMP_REGION_COLD,        /* 6:      t <= -3    */
    PROTECT_BATT_TEMP_REGION_LITTLE_COLD, /* 7: -3 < t <=  0    */
    PROTECT_BATT_TEMP_REGION_COOL,        /* 8:  0 < t <=  5    */
    PROTECT_BATT_TEMP_REGION_WARM         /* 9: 45 < t <= 55    */
};

int get_prop_chg_protect_status(struct smb_charger *chg)
{
	int temp, vbus_mv;
	bool batt_present;
	temp_region_type temp_region;

	if (chg->use_fake_protect_sts)
		return chg->fake_protect_sts;

	if (!is_usb_present(chg))
		return 0;

	temp = get_prop_batt_temp(chg);
	vbus_mv = get_prop_charger_voltage_now(chg);
	batt_present = get_prop_batt_present(chg);
	temp_region = op_battery_temp_region_get(chg);
	if (chg->chg_ovp && vbus_mv >= CHG_SOFT_OVP_MV - 100)
		return PROTECT_CHG_OVP;
	else if (BATT_REMOVE_TEMP > temp || !batt_present)
		return  PROTECT_BATT_MISSING;
	else if (BATT_STATUS_BAD == chg->battery_status)
		return PROTECT_BATT_OVP;
	else if (true == chg->time_out)
		return PROTECT_CHG_OVERTIME;
	else if (temp_region == BATT_TEMP_HOT)
		return PROTECT_BATT_TEMP_REGION__HOT;
	else if (temp_region == BATT_TEMP_COLD)
		return PROTECT_BATT_TEMP_REGION_COLD;
	else if (temp_region == BATT_TEMP_LITTLE_COLD
			&& (chg->chg_done || chg->recharge_status))
		return PROTECT_BATT_TEMP_REGION_LITTLE_COLD;
	else if (temp_region == BATT_TEMP_WARM
			&& (chg->chg_done || chg->recharge_status))
		return PROTECT_BATT_TEMP_REGION_WARM;
	else
		return 0;
}

bool get_prop_fastchg_status(struct smb_charger *chg)
{
	int capacity;

	if (chg->dash_present)
		return true;

	if (chg->hvdcp_present) {
		capacity = get_prop_batt_capacity(chg);
		if (capacity >= 1 && capacity <= 85)
			return true;
	}

	return false;
}

static struct notify_dash_event notify_unplug_event  = {
	.notify_event					= update_dash_unplug_status,
	.notify_dash_charger_present	= set_dash_charger_present,
};

void fastcharge_information_register(struct external_battery_gauge *fast_chg)
{
	if (fast_charger) {
		fast_charger = fast_chg;
		pr_err("multiple battery gauge called\n");
	} else {
		fast_charger = fast_chg;
	}
}
EXPORT_SYMBOL(fastcharge_information_register);

void fastcharge_information_unregister(struct external_battery_gauge *fast_chg)
{
	fast_charger = NULL;
}
EXPORT_SYMBOL(fastcharge_information_unregister);
#endif

static void bms_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bms_update_work);
	power_supply_changed(chg->batt_psy);
}

static void step_soc_req_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						step_soc_req_work.work);
	union power_supply_propval pval = {0, };
	int rc;

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get batt capacity rc=%d\n", rc);
		return;
	}

	step_charge_soc_update(chg, pval.intval);
}

static void smblib_pl_detect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pl_detect_work);

	vote(chg->pl_disable_votable, PARALLEL_PSY_VOTER, false, 0);
}

#define MINIMUM_PARALLEL_FCC_UA		500000
#define PL_TAPER_WORK_DELAY_MS		100
#define TAPER_RESIDUAL_PCT		75
static void smblib_pl_taper_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pl_taper_work.work);
	union power_supply_propval pval = {0, };
	int rc;

	smblib_dbg(chg, PR_PARALLEL, "starting parallel taper work\n");
	if (chg->pl.slave_fcc_ua < MINIMUM_PARALLEL_FCC_UA) {
		smblib_dbg(chg, PR_PARALLEL, "parallel taper is done\n");
		vote(chg->pl_disable_votable, TAPER_END_VOTER, true, 0);
		goto done;
	}

	rc = smblib_get_prop_batt_charge_type(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get batt charge type rc=%d\n", rc);
		goto done;
	}

	if (pval.intval == POWER_SUPPLY_CHARGE_TYPE_TAPER) {
		smblib_dbg(chg, PR_PARALLEL, "master is taper charging; reducing slave FCC\n");
		vote(chg->awake_votable, PL_TAPER_WORK_RUNNING_VOTER, true, 0);
		/* Reduce the taper percent by 25 percent */
		chg->pl.taper_pct = chg->pl.taper_pct
					* TAPER_RESIDUAL_PCT / 100;
		rerun_election(chg->fcc_votable);
		schedule_delayed_work(&chg->pl_taper_work,
				msecs_to_jiffies(PL_TAPER_WORK_DELAY_MS));
		return;
	}

	/*
	 * Master back to Fast Charge, get out of this round of taper reduction
	 */
	smblib_dbg(chg, PR_PARALLEL, "master is fast charging; waiting for next taper\n");

done:
	vote(chg->awake_votable, PL_TAPER_WORK_RUNNING_VOTER, false, 0);
}

static void clear_hdc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						clear_hdc_work.work);

	chg->is_hdc = 0;
}

static void rdstd_cc2_detach_work(struct work_struct *work)
{
	int rc;
	u8 stat;
	struct smb_irq_data irq_data = {NULL, "cc2-removal-workaround"};
	struct smb_charger *chg = container_of(work, struct smb_charger,
						rdstd_cc2_detach_work);

	/*
	 * WA steps -
	 * 1. Enable both UFP and DFP, wait for 10ms.
	 * 2. Disable DFP, wait for 30ms.
	 * 3. Removal detected if both TYPEC_DEBOUNCE_DONE_STATUS
	 *    and TIMER_STAGE bits are gone, otherwise repeat all by
	 *    work rescheduling.
	 * Note, work will be cancelled when pd_hard_reset is 0.
	 */

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write TYPE_C_CTRL_REG rc=%d\n", rc);
		return;
	}

	usleep_range(10000, 11000);

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT,
				 UFP_EN_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write TYPE_C_CTRL_REG rc=%d\n", rc);
		return;
	}

	usleep_range(30000, 31000);

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
			rc);
		return;
	}
	if (stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT)
		goto rerun;

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't read TYPE_C_STATUS_5_REG rc=%d\n", rc);
		return;
	}
	if (stat & TIMER_STAGE_2_BIT)
		goto rerun;

	/* Bingo, cc2 removal detected */
	smblib_reg_block_restore(chg, cc2_detach_settings);
	chg->cc2_sink_detach_flag = CC2_SINK_WA_DONE;
	irq_data.parent_data = chg;
	smblib_handle_usb_typec_change(0, &irq_data);

	return;

rerun:
	schedule_work(&chg->rdstd_cc2_detach_work);
}

static int smblib_create_votables(struct smb_charger *chg)
{
	int rc = 0;

	chg->usb_suspend_votable = create_votable("USB_SUSPEND", VOTE_SET_ANY,
					smblib_usb_suspend_vote_callback,
					chg);
	if (IS_ERR(chg->usb_suspend_votable)) {
		rc = PTR_ERR(chg->usb_suspend_votable);
		return rc;
	}

	chg->dc_suspend_votable = create_votable("DC_SUSPEND", VOTE_SET_ANY,
					smblib_dc_suspend_vote_callback,
					chg);
	if (IS_ERR(chg->dc_suspend_votable)) {
		rc = PTR_ERR(chg->dc_suspend_votable);
		return rc;
	}

	chg->fcc_max_votable = create_votable("FCC_MAX", VOTE_MAX,
					smblib_fcc_max_vote_callback,
					chg);
	if (IS_ERR(chg->fcc_max_votable)) {
		rc = PTR_ERR(chg->fcc_max_votable);
		return rc;
	}

	chg->fcc_votable = create_votable("FCC", VOTE_MIN,
					smblib_fcc_vote_callback,
					chg);
	if (IS_ERR(chg->fcc_votable)) {
		rc = PTR_ERR(chg->fcc_votable);
		return rc;
	}

	chg->fv_votable = create_votable("FV", VOTE_MAX,
					smblib_fv_vote_callback,
					chg);
	if (IS_ERR(chg->fv_votable)) {
		rc = PTR_ERR(chg->fv_votable);
		return rc;
	}

	chg->usb_icl_votable = create_votable("USB_ICL", VOTE_MIN,
					smblib_usb_icl_vote_callback,
					chg);
	if (IS_ERR(chg->usb_icl_votable)) {
		rc = PTR_ERR(chg->usb_icl_votable);
		return rc;
	}

	chg->dc_icl_votable = create_votable("DC_ICL", VOTE_MIN,
					smblib_dc_icl_vote_callback,
					chg);
	if (IS_ERR(chg->dc_icl_votable)) {
		rc = PTR_ERR(chg->dc_icl_votable);
		return rc;
	}

	chg->pd_disallowed_votable_indirect
		= create_votable("PD_DISALLOWED_INDIRECT", VOTE_SET_ANY,
			smblib_pd_disallowed_votable_indirect_callback, chg);
	if (IS_ERR(chg->pd_disallowed_votable_indirect)) {
		rc = PTR_ERR(chg->pd_disallowed_votable_indirect);
		return rc;
	}

	chg->pd_allowed_votable = create_votable("PD_ALLOWED",
					VOTE_SET_ANY, NULL, NULL);
	if (IS_ERR(chg->pd_allowed_votable)) {
		rc = PTR_ERR(chg->pd_allowed_votable);
		return rc;
	}

	chg->awake_votable = create_votable("AWAKE", VOTE_SET_ANY,
					smblib_awake_vote_callback,
					chg);
	if (IS_ERR(chg->awake_votable)) {
		rc = PTR_ERR(chg->awake_votable);
		return rc;
	}

	chg->pl_disable_votable = create_votable("PL_DISABLE", VOTE_SET_ANY,
					smblib_pl_disable_vote_callback,
					chg);
	if (IS_ERR(chg->pl_disable_votable)) {
		rc = PTR_ERR(chg->pl_disable_votable);
		return rc;
	}

	chg->chg_disable_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					smblib_chg_disable_vote_callback,
					chg);
	if (IS_ERR(chg->chg_disable_votable)) {
		rc = PTR_ERR(chg->chg_disable_votable);
		return rc;
	}

	chg->pl_enable_votable_indirect = create_votable("PL_ENABLE_INDIRECT",
					VOTE_SET_ANY,
					smblib_pl_enable_indirect_vote_callback,
					chg);
	if (IS_ERR(chg->pl_enable_votable_indirect)) {
		rc = PTR_ERR(chg->pl_enable_votable_indirect);
		return rc;
	}

	chg->hvdcp_disable_votable = create_votable("HVDCP_DISABLE",
					VOTE_SET_ANY,
					smblib_hvdcp_disable_vote_callback,
					chg);
	if (IS_ERR(chg->hvdcp_disable_votable)) {
		rc = PTR_ERR(chg->hvdcp_disable_votable);
		return rc;
	}

	chg->apsd_disable_votable = create_votable("APSD_DISABLE",
					VOTE_SET_ANY,
					smblib_apsd_disable_vote_callback,
					chg);
	if (IS_ERR(chg->apsd_disable_votable)) {
		rc = PTR_ERR(chg->apsd_disable_votable);
		return rc;
	}

	return rc;
}

static void smblib_destroy_votables(struct smb_charger *chg)
{
	if (chg->usb_suspend_votable)
		destroy_votable(chg->usb_suspend_votable);
	if (chg->dc_suspend_votable)
		destroy_votable(chg->dc_suspend_votable);
	if (chg->fcc_max_votable)
		destroy_votable(chg->fcc_max_votable);
	if (chg->fcc_votable)
		destroy_votable(chg->fcc_votable);
	if (chg->fv_votable)
		destroy_votable(chg->fv_votable);
	if (chg->usb_icl_votable)
		destroy_votable(chg->usb_icl_votable);
	if (chg->dc_icl_votable)
		destroy_votable(chg->dc_icl_votable);
	if (chg->pd_disallowed_votable_indirect)
		destroy_votable(chg->pd_disallowed_votable_indirect);
	if (chg->pd_allowed_votable)
		destroy_votable(chg->pd_allowed_votable);
	if (chg->awake_votable)
		destroy_votable(chg->awake_votable);
	if (chg->pl_disable_votable)
		destroy_votable(chg->pl_disable_votable);
	if (chg->chg_disable_votable)
		destroy_votable(chg->chg_disable_votable);
	if (chg->pl_enable_votable_indirect)
		destroy_votable(chg->pl_enable_votable_indirect);
	if (chg->apsd_disable_votable)
		destroy_votable(chg->apsd_disable_votable);
}

static void smblib_iio_deinit(struct smb_charger *chg)
{
	if (!IS_ERR_OR_NULL(chg->iio.temp_chan))
		iio_channel_release(chg->iio.temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.temp_max_chan))
		iio_channel_release(chg->iio.temp_max_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_i_chan))
		iio_channel_release(chg->iio.usbin_i_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_v_chan))
		iio_channel_release(chg->iio.usbin_v_chan);
	if (!IS_ERR_OR_NULL(chg->iio.batt_i_chan))
		iio_channel_release(chg->iio.batt_i_chan);
}

int smblib_init(struct smb_charger *chg)
{
	int rc = 0;

	mutex_init(&chg->write_lock);
	INIT_WORK(&chg->bms_update_work, bms_update_work);
	INIT_WORK(&chg->pl_detect_work, smblib_pl_detect_work);
	INIT_WORK(&chg->rdstd_cc2_detach_work, rdstd_cc2_detach_work);
	INIT_DELAYED_WORK(&chg->hvdcp_detect_work, smblib_hvdcp_detect_work);
	INIT_DELAYED_WORK(&chg->pl_taper_work, smblib_pl_taper_work);
	INIT_DELAYED_WORK(&chg->step_soc_req_work, step_soc_req_work);
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	INIT_DELAYED_WORK(&chg->re_kick_work, op_re_kick_work);
	INIT_DELAYED_WORK(&chg->check_switch_dash_work,
			op_check_allow_switch_dash_work);
	INIT_DELAYED_WORK(&chg->heartbeat_work,
			op_heartbeat_work);
	schedule_delayed_work(&chg->heartbeat_work,
			msecs_to_jiffies(HEARTBEAT_INTERVAL_MS));
	notify_dash_unplug_register(&notify_unplug_event);
	wake_lock_init(&chg->chg_wake_lock,
			WAKE_LOCK_SUSPEND, "chg_wake_lock");
	g_chg = chg;
#endif
	INIT_DELAYED_WORK(&chg->clear_hdc_work, clear_hdc_work);
	chg->fake_capacity = -EINVAL;

	switch (chg->mode) {
	case PARALLEL_MASTER:
		rc = smblib_create_votables(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't create votables rc=%d\n",
				rc);
			return rc;
		}

		rc = smblib_register_notifier(chg);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't register notifier rc=%d\n", rc);
			return rc;
		}

		chg->bms_psy = power_supply_get_by_name("bms");
		chg->pl.psy = power_supply_get_by_name("parallel");
		if (chg->pl.psy)
			vote(chg->pl_disable_votable, PARALLEL_PSY_VOTER,
			     false, 0);

		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	return rc;
}

int smblib_deinit(struct smb_charger *chg)
{
	switch (chg->mode) {
	case PARALLEL_MASTER:
		power_supply_unreg_notifier(&chg->nb);
		smblib_destroy_votables(chg);
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	smblib_iio_deinit(chg);

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	notify_dash_unplug_unregister(&notify_unplug_event);
#endif

	return 0;
}

int smblib_validate_initial_typec_legacy_status(struct smb_charger *chg)
{
	int rc;
	u8 stat;


	if (qpnp_pon_is_warm_reset())
		return 0;

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_5 rc=%d\n", rc);
		return rc;
	}

	if ((stat & TYPEC_LEGACY_CABLE_STATUS_BIT) == 0)
		return 0;

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_DISABLE_CMD_BIT, TYPEC_DISABLE_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable typec rc=%d\n", rc);
		return rc;
	}

	usleep_range(150000, 151000);

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_DISABLE_CMD_BIT, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable typec rc=%d\n", rc);
		return rc;
	}

	return 0;
}
