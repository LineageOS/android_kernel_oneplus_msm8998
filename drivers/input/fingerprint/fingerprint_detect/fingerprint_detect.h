#ifndef __FINGERPRINT_DETETC_H_
#define __FINGERPRINT_DETETC_H_

struct fingerprint_detect_data {
	struct device *dev;
	int id0_gpio;
	int id1_gpio;
	int id2_gpio;
	struct pinctrl         *fp_pinctrl;
	struct pinctrl_state   *id_state_init;
	struct pinctrl_state   *fp_enable;
	int sensor_version;
	int project_version;
};
extern int fp_version;
#endif

