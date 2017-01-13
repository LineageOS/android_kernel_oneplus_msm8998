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

#ifndef __SMB2_CHARGER_H
#define __SMB2_CHARGER_H
#include <linux/types.h>
#include <linux/irqreturn.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
#include "oem_external_fg.h"
#include <linux/wakelock.h>
#endif
#include "storm-watch.h"

enum print_reason {
	PR_INTERRUPT	= BIT(0),
	PR_REGISTER	= BIT(1),
	PR_MISC		= BIT(2),
	PR_PARALLEL	= BIT(3),
};

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20161014 Add charging standard */
#define BATT_TYPE_FCC_VOTER "BATT_TYPE_FCC_VOTER"
#define PSY_ICL_VOTER		"PSY_ICL_VOTER"
#define TEMP_REGION_MAX               9
#endif
#define DEFAULT_VOTER			"DEFAULT_VOTER"
#define USER_VOTER			"USER_VOTER"
#define PD_VOTER			"PD_VOTER"
#define DCP_VOTER			"DCP_VOTER"
#define USB_PSY_VOTER			"USB_PSY_VOTER"
#define PL_TAPER_WORK_RUNNING_VOTER	"PL_TAPER_WORK_RUNNING_VOTER"
#define PARALLEL_PSY_VOTER		"PARALLEL_PSY_VOTER"
#define PL_INDIRECT_VOTER		"PL_INDIRECT_VOTER"
#define USBIN_I_VOTER			"USBIN_I_VOTER"
#define USBIN_V_VOTER			"USBIN_V_VOTER"
#define CHG_STATE_VOTER			"CHG_STATE_VOTER"
#define TYPEC_SRC_VOTER			"TYPEC_SRC_VOTER"
#define TAPER_END_VOTER			"TAPER_END_VOTER"
#define FCC_MAX_RESULT_VOTER		"FCC_MAX_RESULT_VOTER"
#define THERMAL_DAEMON_VOTER		"THERMAL_DAEMON_VOTER"
#define CC_DETACHED_VOTER		"CC_DETACHED_VOTER"
#define HVDCP_TIMEOUT_VOTER		"HVDCP_TIMEOUT_VOTER"
#define PD_DISALLOWED_INDIRECT_VOTER	"PD_DISALLOWED_INDIRECT_VOTER"
#define PD_HARD_RESET_VOTER		"PD_HARD_RESET_VOTER"
#define VBUS_CC_SHORT_VOTER		"VBUS_CC_SHORT_VOTER"
#define LEGACY_CABLE_VOTER		"LEGACY_CABLE_VOTER"
#define PD_INACTIVE_VOTER		"PD_INACTIVE_VOTER"
#define BOOST_BACK_VOTER		"BOOST_BACK_VOTER"

enum smb_mode {
	PARALLEL_MASTER = 0,
	PARALLEL_SLAVE,
	NUM_MODES,
};

enum cc2_sink_type {
	CC2_SINK_NONE = 0,
	CC2_SINK_STD,
	CC2_SINK_MEDIUM_HIGH,
	CC2_SINK_WA_DONE,
};

enum {
	QC_CHARGER_DETECTION_WA_BIT	= BIT(0),
	BOOST_BACK_WA			= BIT(1),
	TYPEC_CC2_REMOVAL_WA_BIT	= BIT(2),
};

struct smb_regulator {
	struct regulator_dev	*rdev;
	struct regulator_desc	rdesc;
};

struct smb_irq_data {
	void			*parent_data;
	const char		*name;
	struct storm_watch	storm_data;
};

struct smb_chg_param {
	const char	*name;
	u16		reg;
	int		min_u;
	int		max_u;
	int		step_u;
	int		(*get_proc)(struct smb_chg_param *param,
				    u8 val_raw);
	int		(*set_proc)(struct smb_chg_param *param,
				    int val_u,
				    u8 *val_raw);
};

struct smb_params {
	struct smb_chg_param	fcc;
	struct smb_chg_param	fv;
	struct smb_chg_param	usb_icl;
	struct smb_chg_param	icl_stat;
	struct smb_chg_param	otg_cl;
	struct smb_chg_param	dc_icl;
	struct smb_chg_param	dc_icl_pt_lv;
	struct smb_chg_param	dc_icl_pt_hv;
	struct smb_chg_param	dc_icl_div2_lv;
	struct smb_chg_param	dc_icl_div2_mid_lv;
	struct smb_chg_param	dc_icl_div2_mid_hv;
	struct smb_chg_param	dc_icl_div2_hv;
	struct smb_chg_param	jeita_cc_comp;
	struct smb_chg_param	step_soc_threshold[4];
	struct smb_chg_param	step_soc;
	struct smb_chg_param	step_cc_delta[5];
	struct smb_chg_param	freq_buck;
};

struct parallel_params {
	struct power_supply	*psy;
	int			slave_pct;
	int			taper_pct;
	int			slave_fcc_ua;
};

struct smb_iio {
	struct iio_channel	*temp_chan;
	struct iio_channel	*temp_max_chan;
	struct iio_channel	*usbin_i_chan;
	struct iio_channel	*usbin_v_chan;
	struct iio_channel	*batt_i_chan;
};

struct reg_info {
	u16		reg;
	u8		mask;
	u8		val;
	u8		bak;
	const char	*desc;
};

struct smb_charger {
	struct device		*dev;
	char			*name;
	struct regmap		*regmap;
	struct smb_params	param;
	struct smb_iio		iio;
	int			*debug_mask;
	enum smb_mode		mode;

	/* locks */
	struct mutex		write_lock;
	struct mutex		ps_change_lock;

	/* power supplies */
	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*dc_psy;
	struct power_supply		*bms_psy;
	struct power_supply_desc	usb_psy_desc;

	/* notifiers */
	struct notifier_block	nb;

	/* parallel charging */
	struct parallel_params	pl;

	/* regulators */
	struct smb_regulator	*vbus_vreg;
	struct smb_regulator	*vconn_vreg;
	struct regulator	*dpdm_reg;

	/* votables */
	struct votable		*usb_suspend_votable;
	struct votable		*dc_suspend_votable;
	struct votable		*fcc_max_votable;
	struct votable		*fcc_votable;
	struct votable		*fv_votable;
	struct votable		*usb_icl_votable;
	struct votable		*dc_icl_votable;
	struct votable		*pd_disallowed_votable_indirect;
	struct votable		*pd_allowed_votable;
	struct votable		*awake_votable;
	struct votable		*pl_disable_votable;
	struct votable		*chg_disable_votable;
	struct votable		*pl_enable_votable_indirect;
	struct votable		*hvdcp_disable_votable;
	struct votable		*apsd_disable_votable;

	/* work */
	struct work_struct	bms_update_work;
	struct work_struct	pl_detect_work;
	struct work_struct	rdstd_cc2_detach_work;
	struct delayed_work	hvdcp_detect_work;
	struct delayed_work	ps_change_timeout_work;
	struct delayed_work	pl_taper_work;
	struct delayed_work	step_soc_req_work;
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	struct delayed_work	re_kick_work;
	struct delayed_work	check_switch_dash_work;
	struct delayed_work heartbeat_work;
	struct wake_lock	chg_wake_lock;
#endif
	struct delayed_work	clear_hdc_work;

	/* cached status */
#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
	int				BATT_TEMP_T0;
	int				BATT_TEMP_T1;
	int				BATT_TEMP_T2;
	int				BATT_TEMP_T3;
	int				BATT_TEMP_T4;
	int				BATT_TEMP_T5;
	int				BATT_TEMP_T6;
	int				batt_health;
	int				ibatmax[TEMP_REGION_MAX];
	int				vbatmax[TEMP_REGION_MAX];
	int				vbatdet[TEMP_REGION_MAX];
	int				fake_chgvol;
	int				fake_temp;
	int				fake_protect_sts;

	bool				use_fake_chgvol;
	bool				use_fake_temp;
	bool				use_fake_protect_sts;
	bool				vbus_present;
	bool				hvdcp_present;
	bool				dash_present;
	bool				usb_enum_status;
	bool				non_std_chg_present;
	bool				time_out;
	bool				disable_normal_chg_for_dash;
	bool				dash_on;
	bool				chg_ovp;
	bool				is_power_changed;
	bool				recharge_pending;
	bool				recharge_status;
	bool				temp_littel_cool_set_current_0_point_25c;
	bool				oem_lcd_is_on;
	bool				chg_enabled;

	temp_region_type		mBattTempRegion;
	enum batt_status_type		battery_status;
	short				mBattTempBoundT0;
	short				mBattTempBoundT1;
	short				mBattTempBoundT2;
	short				mBattTempBoundT3;
	short				mBattTempBoundT4;
	short				mBattTempBoundT5;
	short				mBattTempBoundT6;
#endif
	int			voltage_min_uv;
	int			voltage_max_uv;
	int			pd_active;
	bool			system_suspend_supported;

	int			system_temp_level;
	int			thermal_levels;
	int			*thermal_mitigation;

	int			otg_cl_ua;
	int			dcp_icl_ua;

	int			fake_capacity;

	bool			step_chg_enabled;
	bool			is_hdc;
	bool			chg_done;
	int			input_limited_fcc_ua;

	/* workaround flag */
	u32			wa_flags;
	enum cc2_sink_type	cc2_sink_detach_flag;
};

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val);
int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val);
int smblib_write(struct smb_charger *chg, u16 addr, u8 val);

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u);
int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend);

int smblib_enable_charging(struct smb_charger *chg, bool enable);
int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u);
int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend);
int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend);

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw);
int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw);
int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw);

int smblib_vbus_regulator_enable(struct regulator_dev *rdev);
int smblib_vbus_regulator_disable(struct regulator_dev *rdev);
int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev);

int smblib_vconn_regulator_enable(struct regulator_dev *rdev);
int smblib_vconn_regulator_disable(struct regulator_dev *rdev);
int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev);

irqreturn_t smblib_handle_debug(int irq, void *data);
irqreturn_t smblib_handle_chg_state_change(int irq, void *data);
irqreturn_t smblib_handle_step_chg_state_change(int irq, void *data);
irqreturn_t smblib_handle_step_chg_soc_update_fail(int irq, void *data);
irqreturn_t smblib_handle_step_chg_soc_update_request(int irq, void *data);
irqreturn_t smblib_handle_batt_temp_changed(int irq, void *data);
irqreturn_t smblib_handle_batt_psy_changed(int irq, void *data);
irqreturn_t smblib_handle_usb_psy_changed(int irq, void *data);
irqreturn_t smblib_handle_usb_plugin(int irq, void *data);
irqreturn_t smblib_handle_usb_source_change(int irq, void *data);
irqreturn_t smblib_handle_icl_change(int irq, void *data);
irqreturn_t smblib_handle_usb_typec_change(int irq, void *data);
irqreturn_t smblib_handle_dc_plugin(int irq, void *data);
irqreturn_t smblib_handle_high_duty_cycle(int irq, void *data);
irqreturn_t smblib_handle_switcher_power_ok(int irq, void *data);

int smblib_get_prop_input_suspend(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_charge_done(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_input_current_limited(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_voltage_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_current_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_batt_temp(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_step_chg_step(struct smb_charger *chg,
				union power_supply_propval *val);

int smblib_set_prop_input_suspend(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val);

#ifdef VENDOR_EDIT
/* david.liu@bsp, 20160926 Add dash charging */
irqreturn_t smblib_handle_aicl_done(int irq, void *data);
void op_charge_info_init(struct smb_charger *chg);
int update_dash_unplug_status(void);
int get_prop_batt_status(struct smb_charger *chg);
int get_prop_chg_protect_status(struct smb_charger *chg);
int check_allow_switch_dash(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_chg_voltage(struct smb_charger *chg,
				  const union power_supply_propval *val);
int smblib_set_prop_batt_temp(struct smb_charger *chg,
				  const union power_supply_propval *val);
int smblib_set_prop_chg_protect_status(struct smb_charger *chg,
				  const union power_supply_propval *val);
bool op_get_fastchg_ing(struct smb_charger *chg);
bool get_prop_fastchg_status(struct smb_charger *chg);
#endif
int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_dc_online(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				const union power_supply_propval *val);

int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_online(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_suspend(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_pd_current_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_current_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_typec_mode(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_pd_allowed(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_input_current_settled(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_pd_in_hard_reset(struct smb_charger *chg,
			       union power_supply_propval *val);
int smblib_get_pe_start(struct smb_charger *chg,
			       union power_supply_propval *val);
int smblib_get_prop_charger_temp(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_get_prop_charger_temp_max(struct smb_charger *chg,
				union power_supply_propval *val);
int smblib_set_prop_pd_current_max(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_usb_current_max(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_usb_voltage_min(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_usb_voltage_max(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_pd_active(struct smb_charger *chg,
				const union power_supply_propval *val);
int smblib_set_prop_pd_in_hard_reset(struct smb_charger *chg,
				const union power_supply_propval *val);

int smblib_get_prop_slave_current_now(struct smb_charger *chg,
				union power_supply_propval *val);

int smblib_validate_initial_typec_legacy_status(struct smb_charger *chg);

int smblib_init(struct smb_charger *chg);
int smblib_deinit(struct smb_charger *chg);
#endif /* __SMB2_CHARGER_H */
