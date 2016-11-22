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
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
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
static bool op_set_fast_chg_allow(struct smb_charger *chg, bool enable);
static bool get_prop_fast_chg_started(struct smb_charger *chg);
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

#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			dev_info(chg->dev, fmt, ##__VA_ARGS__);	\
		else						\
			dev_dbg(chg->dev, fmt, ##__VA_ARGS__);	\
	} while (0)

static bool is_secure(struct smb_charger *chg, int addr)
{
	/* assume everything above 0xC0 is secure */
	return (bool)((addr & 0xFF) >= 0xC0);
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

static int smblib_get_step_charging_adjustment(struct smb_charger *chg,
					       int *cc_offset)
{
	int step_state;
	int rc;
	u8 stat;

	if (!chg->step_chg_enabled) {
		*cc_offset = 0;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	step_state = (stat & STEP_CHARGING_STATUS_MASK) >>
				STEP_CHARGING_STATUS_SHIFT;
	rc = smblib_get_charge_param(chg, &chg->param.step_cc_delta[step_state],
				     cc_offset);

	return rc;
}

static void smblib_fcc_split_ua(struct smb_charger *chg, int total_fcc,
			int *master_ua, int *slave_ua)
{
	int rc, cc_reduction_ua = 0;
	int step_cc_delta;
	int master_percent = min(max(*chg->pl.master_percent, 0), 100);
	union power_supply_propval pval = {0, };
	int effective_fcc;

	/*
	 * if master_percent is 0, s/w will configure master's fcc to zero and
	 * slave's fcc to the max. However since master's fcc is zero it
	 * disables its own charging and as a result the slave's charging is
	 * disabled via the fault line.
	 */

	rc = smblib_get_prop_batt_health(chg, &pval);
	if (rc == 0) {
		if (pval.intval == POWER_SUPPLY_HEALTH_WARM
			|| pval.intval == POWER_SUPPLY_HEALTH_COOL) {
			rc = smblib_get_charge_param(chg,
					&chg->param.jeita_cc_comp,
					&cc_reduction_ua);
			if (rc < 0) {
				dev_err(chg->dev, "Could not get jeita comp, rc=%d\n",
					rc);
				cc_reduction_ua = 0;
			}
		}
	}

	rc = smblib_get_step_charging_adjustment(chg, &step_cc_delta);
	if (rc < 0)
		step_cc_delta = 0;

	/*
	 * During JEITA condition and with step_charging enabled, PMI will
	 * pick the lower of the two value: (FCC - JEITA current compensation)
	 * or (FCC + step_charging current delta)
	 */

	effective_fcc = min(max(0, total_fcc - cc_reduction_ua),
			    max(0, total_fcc + step_cc_delta));
	*master_ua = (effective_fcc * master_percent) / 100;
	*slave_ua = (effective_fcc - *master_ua) * chg->pl.taper_percent / 100;
	*master_ua = max(0, *master_ua + total_fcc - effective_fcc);
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
		dev_err(chg->dev, "%s: Couldn't read from 0x%04x rc=%d\n",
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
		dev_err(chg->dev, "Couldn't read USBIN_CMD_IL rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_BIT;

	return rc;
}

struct apsd_result {
	const char * const name;
	const u8 bit;
	const enum power_supply_type pst;
};

static const struct apsd_result const smblib_apsd_results[] = {
	{"UNKNOWN", 0, POWER_SUPPLY_TYPE_UNKNOWN},
	{"SDP", SDP_CHARGER_BIT, POWER_SUPPLY_TYPE_USB},
	{"CDP", CDP_CHARGER_BIT, POWER_SUPPLY_TYPE_USB_CDP},
	{"DCP", DCP_CHARGER_BIT, POWER_SUPPLY_TYPE_USB_DCP},
	{"OCP", OCP_CHARGER_BIT, POWER_SUPPLY_TYPE_USB_DCP},
	{"FLOAT", FLOAT_CHARGER_BIT, POWER_SUPPLY_TYPE_USB_DCP},
	{"HVDCP2", DCP_CHARGER_BIT | QC_2P0_BIT, POWER_SUPPLY_TYPE_USB_HVDCP},
	{"HVDCP3", DCP_CHARGER_BIT | QC_3P0_BIT, POWER_SUPPLY_TYPE_USB_HVDCP_3},
};

static const struct apsd_result *smblib_get_apsd_result(struct smb_charger *chg)
{
	int rc, i;
	u8 stat;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return &smblib_apsd_results[0];
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", stat);

	if (!(stat & APSD_DTC_STATUS_DONE_BIT))
		return &smblib_apsd_results[0];

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read APSD_RESULT_STATUS rc=%d\n",
			rc);
		return &smblib_apsd_results[0];
	}
	stat &= APSD_RESULT_STATUS_MASK;

	for (i = 0; i < ARRAY_SIZE(smblib_apsd_results); i++) {
		if (smblib_apsd_results[i].bit == stat)
			return &smblib_apsd_results[i];
	}

	dev_err(chg->dev, "Couldn't find an APSD result for 0x%02x\n", stat);
	return &smblib_apsd_results[0];
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
			dev_err(chg->dev, "%s: %d is out of range [%d, %d]\n",
				param->name, val_u, param->min_u, param->max_u);
			return -EINVAL;
		}

		val_raw = (val_u - param->min_u) / param->step_u;
	}

	rc = smblib_write(chg, param->reg, val_raw);
	if (rc < 0) {
		dev_err(chg->dev, "%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
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
		dev_err(chg->dev, "Error in updating soc, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_write(chg, STEP_CHG_SOC_VBATT_V_UPDATE_REG,
			STEP_CHG_SOC_VBATT_V_UPDATE_BIT);
	if (rc < 0) {
		dev_err(chg->dev,
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
		dev_err(chg->dev, "Couldn't write %s to USBIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_SUSPEND_BIT,
				 suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't write %s to DCIN_SUSPEND_BIT rc=%d\n",
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
	} else if (min_allowed_uv == MICRO_9V && max_allowed_uv == MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V;
	} else if (min_allowed_uv == MICRO_12V && max_allowed_uv == MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_12V;
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_9V;
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_12V;
	} else if (min_allowed_uv < MICRO_12V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V_TO_12V;
	} else {
		dev_err(chg->dev, "invalid allowed voltage [%d, %d]\n",
			min_allowed_uv, max_allowed_uv);
		return -EINVAL;
	}

	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG, allowed_voltage);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't write 0x%02x to USBIN_ADAPTER_ALLOW_CFG rc=%d\n",
			allowed_voltage, rc);
		return rc;
	}

	return rc;
}

/********************
 * HELPER FUNCTIONS *
 ********************/

static int smblib_update_usb_type(struct smb_charger *chg)
{
	int rc = 0;
	const struct apsd_result *apsd_result;

	/* if PD is active, APSD is disabled so won't have a valid result */
	if (chg->pd_active) {
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
		pr_err("pd is active! return directly\n");
#endif
		return rc;
	}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	if (!get_prop_fast_switch_to_normal(chg)) {
		if (chg->dash_on) {
			chg->usb_psy_desc.type = POWER_SUPPLY_TYPE_DASH;
		} else {
			apsd_result = smblib_get_apsd_result(chg);
			chg->usb_psy_desc.type = apsd_result->pst;
		}
	} else {
		chg->usb_psy_desc.type = POWER_SUPPLY_TYPE_DASH;
	}
	pr_info("type=%d\n", chg->usb_psy_desc.type);
#else
	apsd_result = smblib_get_apsd_result(chg);
	chg->usb_psy_desc.type = apsd_result->pst;
#endif

	return rc;
}

static int smblib_detach_usb(struct smb_charger *chg)
{
	int rc;

	cancel_delayed_work_sync(&chg->hvdcp_detect_work);
	chg->usb_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;

	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG,
			  USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V rc=%d\n",
			rc);
		return rc;
	}

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;

	/* clear USB ICL vote for PD_VOTER */
	rc = vote(chg->usb_icl_votable, PD_VOTER, false, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't vote for USB ICL rc=%d\n",
			rc);
		return rc;
	}

	vote(chg->pd_allowed_votable, DEFAULT_VOTER, false, 0);

	return rc;
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
		pr_err("Couldn't register psy notifier rc = %d\n", rc);
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
	return smblib_set_usb_suspend(chg, suspend);
}

static int smblib_dc_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

	if (suspend < 0)
		suspend = false;

	return smblib_set_dc_suspend(chg, suspend);
}

static int smblib_fcc_max_vote_callback(struct votable *votable, void *data,
			int fcc_ua, const char *client)
{
	struct smb_charger *chg = data;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("set fcc_ua=%d\n", fcc_ua);
#endif
	return vote(chg->fcc_votable, FCC_MAX_RESULT, true, fcc_ua);
}

static int smblib_fcc_vote_callback(struct votable *votable, void *data,
			int fcc_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	union power_supply_propval pval = {0, };
	int master_ua = fcc_ua, slave_ua;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("set fcc=%d, mode=%d\n", fcc_ua, chg->mode);
#endif
	if (fcc_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter\n");
		return 0;
	}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161109 Charging porting */
	if (chg->pl.psy && chg->mode == PARALLEL_MASTER
#else
	if (chg->mode == PARALLEL_MASTER
#endif
		&& !get_effective_result_locked(chg->pl_disable_votable)) {
		smblib_fcc_split_ua(chg, fcc_ua, &master_ua, &slave_ua);

		/*
		 * parallel charger is not disabled, implying that
		 * chg->pl.psy exists
		 */
		pval.intval = slave_ua;
		rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
		if (rc < 0) {
			dev_err(chg->dev, "Could not set parallel fcc, rc=%d\n",
				rc);
			return rc;
		}

		chg->pl.slave_fcc = slave_ua;
	}

	rc = smblib_set_charge_param(chg, &chg->param.fcc, master_ua);
	if (rc < 0) {
		dev_err(chg->dev, "Error in setting fcc, rc=%d\n", rc);
		return rc;
	}

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
		dev_err(chg->dev,
			"Couldn't set floating voltage rc=%d\n", rc);
		return rc;
	}

	if (chg->mode == PARALLEL_MASTER && chg->pl.psy) {
		pval.intval = fv_uv + PARALLEL_FLOAT_VOLTAGE_DELTA_UV;
		rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX, &pval);
		if (rc < 0) {
			dev_err(chg->dev,
				"Couldn't set float on parallel rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

#define USBIN_25MA 25000
#define USBIN_100MA 100000
static int smblib_usb_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool suspend;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("set iusb_max=%d, type=%d\n", icl_ua,
			chg->usb_psy_desc.type);
#endif
	if (icl_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter hence suspending\n");
		icl_ua = 0;
	}

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
		dev_err(chg->dev, "Couldn't set DC input current limit rc=%d\n",
			rc);
		return rc;
	}

suspend:
	rc = vote(chg->dc_suspend_votable, USER_VOTER, suspend, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't vote to %s DC rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}
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
	chg->pl.taper_percent = 100;
	rerun_election(chg->fv_votable);
	rerun_election(chg->fcc_votable);

	pval.intval = pl_disable;
	rc = power_supply_set_property(chg->pl.psy,
			POWER_SUPPLY_PROP_INPUT_SUSPEND, &pval);
	if (rc < 0) {
		dev_err(chg->dev,
			"Couldn't change slave suspend state rc=%d\n", rc);
		return rc;
	}

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
		dev_err(chg->dev, "Couldn't %s charging rc=%d\n",
			chg_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}
/*****************
 * OTG REGULATOR *
 *****************/

int smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = regmap_write(chg->regmap, CMD_OTG_REG, OTG_EN_BIT);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't enable OTG regulator rc=%d\n", rc);

	return rc;
}

int smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = regmap_write(chg->regmap, CMD_OTG_REG, 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't disable OTG regulator rc=%d\n", rc);

	return rc;
}

int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 cmd;

	rc = smblib_read(chg, CMD_OTG_REG, &cmd);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read CMD_OTG rc=%d", rc);
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
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}
	stat = stat & CC_ORIENTATION_BIT ? 0 : VCONN_EN_ORIENTATION_BIT;
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT | VCONN_EN_ORIENTATION_BIT,
				 VCONN_EN_VALUE_BIT | stat);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't enable vconn setting rc=%d\n", rc);

	return rc;
}

int smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT, 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't disable vconn regulator rc=%d\n",
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
		dev_err(chg->dev, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
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
		dev_err(chg->dev, "Couldn't read BATIF_INT_RT_STS rc=%d\n",
			rc);
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
		dev_err(chg->dev, "Couldn't get usb online property rc=%d\n",
			rc);
		return rc;
	}
	usb_online = (bool)pval.intval;

	rc = smblib_get_prop_dc_online(chg, &pval);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get dc online property rc=%d\n",
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
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
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
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
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
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_2 = 0x%02x\n",
		   stat);

	if (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT) {
		dev_err(chg->dev, "battery over-voltage\n");
		val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		goto done;
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
		dev_err(chg->dev, "Couldn't read AICL_STATUS rc=%d\n", rc);
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
	if (chg->use_fake_temp)
		return chg->fake_temp;
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
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	val->intval = (stat & STEP_CHARGING_STATUS_MASK) >>
				STEP_CHARGING_STATUS_SHIFT;

	return rc;
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
		dev_err(chg->dev, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't vote to %s DC rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);
	return rc;
}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
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
		return vote(chg->chg_disable_votable, THERMAL_DAEMON, true, 0);

	vote(chg->chg_disable_votable, THERMAL_DAEMON, false, 0);
	if (chg->system_temp_level == 0)
		return vote(chg->fcc_votable, THERMAL_DAEMON, false, 0);

	vote(chg->fcc_votable, THERMAL_DAEMON, true,
			chg->thermal_mitigation[chg->system_temp_level]);
	return 0;
}

/*******************
 * DC PSY GETTERS *
 *******************/

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, DC_INT_RT_STS_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read DC_INT_RT_STS_REG rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "DC_INT_RT_STS_REG = 0x%02x\n",
		   stat);

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);

	return rc;
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
		dev_err(chg->dev, "Couldn't read POWER_PATH_STATUS rc=%d\n",
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
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n",
		   stat);

	val->intval = (bool)(stat & CC_ATTACHED_BIT);

	return rc;
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
		dev_err(chg->dev, "Couldn't read POWER_PATH_STATUS rc=%d\n",
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

	return iio_read_channel_processed(chg->iio.usbin_v_chan, &val->intval);
}

int smblib_get_prop_usb_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_effective_result_locked(chg->usb_icl_votable);
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
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
			rc);
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
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_1 rc=%d\n", rc);
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
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_2 rc=%d\n", rc);
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
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
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
		dev_err(chg->dev, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
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
		dev_err(chg->dev, "unsupported power role 0x%02lx\n",
			ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT));
		return -EINVAL;
	}

	return rc;
}

int smblib_get_prop_pd_allowed(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = get_effective_result_locked(chg->pd_allowed_votable);
	return 0;
}

int smblib_get_prop_input_current_settled(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.icl_stat, &val->intval);
}

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_usb_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_err("USB current_max_ma=%d\n", val->intval);
#endif
	rc = vote(chg->usb_icl_votable, PD_VOTER, true, val->intval);
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
		dev_err(chg->dev, "power role %d not supported\n", val->intval);
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, power_role);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
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
		dev_err(chg->dev, "invalid max voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_min_uv = val->intval;
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
		dev_err(chg->dev, "invalid min voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_max_uv = val->intval;
	return rc;
}

int smblib_set_prop_pd_active(struct smb_charger *chg,
			      const union power_supply_propval *val)
{
	int rc;
	u8 stat;

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	pr_info("set pd_active=%d\n", val->intval);
#endif
	if (!get_effective_result(chg->pd_allowed_votable)) {
		dev_err(chg->dev, "PD is not allowed\n");
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				 AUTO_SRC_DETECT_BIT,
				 val->intval ? 0 : AUTO_SRC_DETECT_BIT);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't %s APSD rc=%d\n",
			val->intval ? "disable" : "enable", rc);
		return rc;
	}

	vote(chg->pd_allowed_votable, PD_VOTER, val->intval, 0);

	/*
	 * VCONN_EN_ORIENTATION_BIT controls whether to use CC1 or CC2 line
	 * when TYPEC_SPARE_CFG_BIT (CC pin selection s/w override) is set
	 * or when VCONN_EN_VALUE_BIT is set.
	 */
	if (val->intval) {
		rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
			if (rc < 0) {
				dev_err(chg->dev,
					"Couldn't read TYPE_C_STATUS_4 rc=%d\n",
					rc);
				return rc;
		}

		stat &= CC_ORIENTATION_BIT;
		rc = smblib_masked_write(chg,
					TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
					VCONN_EN_ORIENTATION_BIT,
					stat ? 0 : VCONN_EN_ORIENTATION_BIT);
		if (rc < 0)
			dev_err(chg->dev,
				"Couldn't enable vconn on CC line rc=%d\n", rc);
	}

	/* CC pin selection s/w override in PD session; h/w otherwise. */
	rc = smblib_masked_write(chg, TAPER_TIMER_SEL_CFG_REG,
				 TYPEC_SPARE_CFG_BIT,
				 val->intval ? TYPEC_SPARE_CFG_BIT : 0);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't change cc_out ctrl to %s rc=%d\n",
			val->intval ? "SW" : "HW", rc);
		return rc;
	}

	chg->pd_active = (bool)val->intval;
	smblib_update_usb_type(chg);
	return rc;
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
	union power_supply_propval pval = {0, };
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return IRQ_HANDLED;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	smblib_pl_handle_chg_state_change(chg, stat);
	pval.intval = (stat == TERMINATE_CHARGE);
	power_supply_set_property(chg->batt_psy, POWER_SUPPLY_PROP_CHARGE_DONE,
		&pval);
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161109 Charging porting */
	if (pval.intval) {
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
		dev_err(chg->dev, "Couldn't get batt capacity rc=%d\n", rc);
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
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	bool last_vbus_present;

	last_vbus_present = chg->vbus_present;
#endif
	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
					      "dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			dev_err(chg->dev, "Couldn't get dpdm regulator rc=%ld\n",
				PTR_ERR(chg->dpdm_reg));
			chg->dpdm_reg = NULL;
		}
	}

	if (!chg->dpdm_reg) {
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
		pr_err("skip_dpdm_float\n");
#endif
		goto skip_dpdm_float;
	}

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	chg->vbus_present = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	if (last_vbus_present != chg->vbus_present) {
		if (chg->vbus_present) {
			pr_info("release chg_wake_lock\n");
			wake_lock(&chg->chg_wake_lock);
		} else {
			pr_info("acquire chg_wake_lock\n");
			wake_unlock(&chg->chg_wake_lock);
		}
	}

	chg->dash_on = get_prop_fast_chg_started(chg);
	if (chg->dash_on) {
		pr_err("return directly because dash is online\n");
		return IRQ_HANDLED;
	}
#endif

	if (chg->vbus_present) {
		if (!regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't enable dpdm regulator rc=%d\n",
					rc);
		}
	} else {
		if (regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't disable dpdm regulator rc=%d\n",
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

skip_dpdm_float:
	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s %s\n",
		   irq_data->name, chg->vbus_present ? "attached" : "detached");
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	pr_err("IRQ: %s %s\n",
		   irq_data->name, chg->vbus_present ? "attached" : "detached");
#endif
	return IRQ_HANDLED;
}

#define MICRO_5P5V		5500000
#define USB_WEAK_INPUT_MA	1500000
static bool is_icl_pl_ready(struct smb_charger *chg)
{
	union power_supply_propval pval = {0, };
	int icl_ma;
	int rc;

	rc = smblib_get_prop_usb_voltage_now(chg, &pval);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get prop usb voltage rc=%d\n", rc);
		return false;
	}

	if (pval.intval <= MICRO_5P5V) {
		rc = smblib_get_charge_param(chg,
					&chg->param.icl_stat, &icl_ma);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't get ICL status rc=%d\n",
				rc);
			return false;
		}

		if (icl_ma < USB_WEAK_INPUT_MA)
			return false;
	}

	/*
	 * Always enable parallel charging when USB INPUT is higher than 5V
	 * regardless of the AICL results. Assume chargers above 5V are strong
	 */

	return true;
}

irqreturn_t smblib_handle_icl_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_disable_votable, USBIN_ICL_VOTER,
					!is_icl_pl_ready(chg), 0);

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

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

	if (!rising)
		return;

	/* the APSD done handler will set the USB supply type */
	apsd_result = smblib_get_apsd_result(chg);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-3p0-auth-done rising; %s detected\n",
		   apsd_result->name);
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
	int rc, temp_region;
#else
	int rc;
#endif
	const struct apsd_result *apsd_result;

	if (!rising)
		return;

	apsd_result = smblib_get_apsd_result(chg);
	switch (apsd_result->bit) {
	case SDP_CHARGER_BIT:
	case CDP_CHARGER_BIT:
	case OCP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		vote(chg->pd_allowed_votable, DEFAULT_VOTER, true, 0);
		break;
	case DCP_CHARGER_BIT:
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
		|| temp_region != BATT_TEMP_HOT) {
		op_charging_en(chg, true);
	}

	pr_info("apsd result=0x%x, name=%s, psy_type=%d\n",
		apsd_result->bit, apsd_result->name, apsd_result->pst);
	if (apsd_result->bit == OCP_CHARGER_BIT) {
		schedule_delayed_work(&chg->check_switch_dash_work,
					msecs_to_jiffies(500));
	}

	/* set allow read extern fg IIC */
	set_property_on_fg(chg,
		POWER_SUPPLY_PROP_SET_ALLOW_READ_EXTERN_FG_IIC, true);
#endif
	rc = smblib_update_usb_type(chg);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't update usb type rc=%d\n", rc);

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
		dev_err(chg->dev, "Couldn't read APSD_STATUS rc=%d\n", rc);
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

	smblib_handle_hvdcp_3p0_auth_done(chg,
		(bool)(stat & QC_AUTH_DONE_STATUS_BIT));

	smblib_handle_adaptive_voltage_done(chg,
		(bool)(stat & VADP_CHANGE_DONE_AFTER_AUTH_BIT));

	smblib_handle_sdp_enumeration_done(chg,
		(bool)(stat & ENUMERATION_DONE_BIT));

	smblib_handle_slow_plugin_timeout(chg,
		(bool)(stat & SLOW_PLUGIN_TIMEOUT_BIT));

	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

static void smblib_handle_typec_cc(struct smb_charger *chg, bool attached)
{
	int rc;

	if (!attached) {
		rc = smblib_detach_usb(chg);
		if (rc < 0)
			dev_err(chg->dev, "Couldn't detach USB rc=%d\n", rc);
	}

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_err("IRQ: CC %s\n",
#else
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: CC %s\n",
#endif
		   attached ? "attached" : "detached");
}

static void smblib_handle_typec_debounce_done(struct smb_charger *chg,
					      bool rising, bool sink_attached)
{
	int rc;
	union power_supply_propval pval = {0, };

	/* allow PD for attached sinks */
	if (rising && sink_attached)
		vote(chg->pd_allowed_votable, DEFAULT_VOTER, true, 0);

	rc = smblib_get_prop_typec_mode(chg, &pval);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get prop typec mode rc=%d\n", rc);

	/*
	 * vote to enable parallel charging if a source is attached, and disable
	 * otherwise
	 */
	vote(chg->pl_disable_votable, TYPEC_SRC_VOTER,
					!rising || sink_attached, 0);

	if (!rising || sink_attached) {
		/* icl votes to disable parallel charging */
		vote(chg->pl_disable_votable, USBIN_ICL_VOTER, true, 0);
		/* reset taper_end voter here */
		vote(chg->pl_disable_votable, TAPER_END_VOTER, false, 0);
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
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
			rc);
		return IRQ_HANDLED;
	}
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
	pr_info("TYPE_C_STATUS_4=0x%02x\n", stat);
#endif
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat);

	if (stat & TYPEC_VBUS_ERROR_STATUS_BIT) {
		dev_err(chg->dev, "IRQ: vbus-error rising\n");
		return IRQ_HANDLED;
	}

	smblib_handle_typec_cc(chg,
			(bool)(stat & CC_ATTACHED_BIT));
	smblib_handle_typec_debounce_done(chg,
			(bool)(stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT),
			(bool)(stat & UFP_DFP_MODE_STATUS_BIT));

	power_supply_changed(chg->usb_psy);

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

/***************
 * Work Queues *
 ***************/

static void smblib_hvdcp_detect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
					       hvdcp_detect_work.work);
	const struct apsd_result *apsd_result;

	apsd_result = smblib_get_apsd_result(chg);
	if (apsd_result->bit &&
			!(apsd_result->bit & (QC_2P0_BIT | QC_3P0_BIT))) {
		vote(chg->pd_allowed_votable, DEFAULT_VOTER, true, 0);
		power_supply_changed(chg->usb_psy);
	}
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

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}
	pr_debug("TYPE_C_STATUS_4 = 0x%02x\n", stat);

	return (bool)(stat & CC_ATTACHED_BIT);
}

static bool is_dc_present(struct smb_charger *chg)
{
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, DC_INT_RT_STS_REG, &stat);
	if (rc < 0) {
		pr_err("Couldn't read DC_INT_RT_STS_REG rc=%d\n", rc);
		return rc;
	}
	pr_debug("DC_INT_RT_STS_REG = 0x%02x\n", stat);

	return (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);
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


static bool get_prop_fast_chg_started(struct smb_charger *chg)
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

static bool op_set_fast_chg_allow(struct smb_charger *chg, bool enable)
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

	if (gpio_get_value(15)) /* usb-sw-gpio */
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

	if (!is_usb_present(chg) && !is_dc_present(chg))
		return;

	apsd_result = smblib_get_apsd_result(chg);
	if (apsd_result->bit != OCP_CHARGER_BIT)
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
	bool charger_present;

	charger_present = is_usb_present(chg) | is_dc_present(chg);
	if (!charger_present)
		return;

	apsd_result = smblib_get_apsd_result(chg);
	if (apsd_result->bit == OCP_CHARGER_BIT)
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
		charger_present = is_usb_present(g_chg) | is_dc_present(g_chg);
		g_chg->dash_present = status && charger_present;
		if (g_chg->dash_present && !pre_dash_present) {
			pr_err("set dash online\n");
			op_charging_en(g_chg, false);
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

	if(!is_usb_present(chg) && !is_dc_present(chg))
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

	charger_present = is_usb_present(chg) | is_dc_present(chg);
	if (!charger_present)
		goto out;

	/* charger present */
	power_supply_changed(chg->batt_psy);
	chg->dash_on = get_prop_fast_chg_started(chg);
	if (chg->dash_on) {
		switch_fast_chg(chg);
		pr_info("fast chg started, GPIO15=%d\n",
				gpio_get_value(15));
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
	int temp, vbus_mv, charger_present = 0;
	bool batt_present;
	temp_region_type temp_region;

	if (chg->use_fake_protect_sts)
		return chg->fake_protect_sts;

	charger_present = is_usb_present(chg) | is_dc_present(chg);
	if (!charger_present)
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
		dev_err(chg->dev, "Couldn't get batt capacity rc=%d\n", rc);
		return;
	}

	step_charge_soc_update(chg, pval.intval);
}

static void smblib_pl_detect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pl_detect_work);

	if (!get_effective_result_locked(chg->pl_disable_votable))
		rerun_election(chg->pl_disable_votable);
}

#define MINIMUM_PARALLEL_FCC_UA		500000
#define PL_TAPER_WORK_DELAY_MS		100
#define TAPER_RESIDUAL_PERCENT		75
static void smblib_pl_taper_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pl_taper_work.work);
	union power_supply_propval pval = {0, };
	int rc;

	if (chg->pl.slave_fcc < MINIMUM_PARALLEL_FCC_UA) {
		vote(chg->pl_disable_votable, TAPER_END_VOTER, true, 0);
		goto done;
	}

	rc = smblib_get_prop_batt_charge_type(chg, &pval);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get batt charge type rc=%d\n", rc);
		goto done;
	}

	if (pval.intval == POWER_SUPPLY_CHARGE_TYPE_TAPER) {
		vote(chg->awake_votable, PL_VOTER, true, 0);
		/* Reduce the taper percent by 25 percent */
		chg->pl.taper_percent = chg->pl.taper_percent
					* TAPER_RESIDUAL_PERCENT / 100;
		rerun_election(chg->fcc_votable);
		schedule_delayed_work(&chg->pl_taper_work,
				msecs_to_jiffies(PL_TAPER_WORK_DELAY_MS));
		return;
	}

	/*
	 * Master back to Fast Charge, get out of this round of taper reduction
	 */
done:
	vote(chg->awake_votable, PL_VOTER, false, 0);
}

static void clear_hdc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						clear_hdc_work.work);

	chg->is_hdc = 0;
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

	chg->pd_allowed_votable = create_votable("PD_ALLOWED", VOTE_SET_ANY,
					NULL, NULL);
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
	if (chg->pd_allowed_votable)
		destroy_votable(chg->pd_allowed_votable);
	if (chg->awake_votable)
		destroy_votable(chg->awake_votable);
	if (chg->pl_disable_votable)
		destroy_votable(chg->pl_disable_votable);
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
}

int smblib_init(struct smb_charger *chg)
{
	int rc = 0;

	mutex_init(&chg->write_lock);
	INIT_WORK(&chg->bms_update_work, bms_update_work);
	INIT_WORK(&chg->pl_detect_work, smblib_pl_detect_work);
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
			dev_err(chg->dev, "Couldn't create votables rc=%d\n",
				rc);
			return rc;
		}

		chg->bms_psy = power_supply_get_by_name("bms");
		chg->pl.psy = power_supply_get_by_name("parallel");

		rc = smblib_register_notifier(chg);
		if (rc < 0) {
			dev_err(chg->dev,
				"Couldn't register notifier rc=%d\n", rc);
			return rc;
		}

		break;
	case PARALLEL_SLAVE:
		break;
	default:
		dev_err(chg->dev, "Unsupported mode %d\n", chg->mode);
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
		dev_err(chg->dev, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	smblib_iio_deinit(chg);

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	notify_dash_unplug_unregister(&notify_unplug_event);
#endif

	return 0;
}
