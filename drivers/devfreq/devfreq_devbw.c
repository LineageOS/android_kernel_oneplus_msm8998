/*
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "devbw: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/devfreq.h>
#include <linux/of.h>
#include <trace/events/power.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>

/* Has to be ULL to prevent overflow where this macro is used. */
#define MBYTE (1ULL << 20)
#define MAX_PATHS	2
#define DBL_BUF		2

#ifdef CONFIG_VENDOR_ONEPLUS
#include <linux/pm_qos.h>
struct qos_request_v {
	int max_state;
	int max_devfreq;
	int min_devfreq;
};

static bool cpubw_flag = false;
static struct qos_request_v qos_request_value = {
	.max_state = 0,
	.max_devfreq = INT_MAX,
	.min_devfreq = 0,
};
#endif

struct dev_data {
	struct msm_bus_vectors vectors[MAX_PATHS * DBL_BUF];
	struct msm_bus_paths bw_levels[DBL_BUF];
	struct msm_bus_scale_pdata bw_data;
	int num_paths;
	u32 bus_client;
	int cur_idx;
	int cur_ab;
	int cur_ib;
	long gov_ab;
	unsigned int ab_percent;
	struct devfreq *df;
	struct devfreq_dev_profile dp;
};

static int set_bw(struct device *dev, int new_ib, int new_ab)
{
	struct dev_data *d = dev_get_drvdata(dev);
	int i, ret;

	if (d->cur_ib == new_ib && d->cur_ab == new_ab)
		return 0;

	i = (d->cur_idx + 1) % DBL_BUF;

	d->bw_levels[i].vectors[0].ib = new_ib * MBYTE;
	d->bw_levels[i].vectors[0].ab = new_ab / d->num_paths * MBYTE;
	d->bw_levels[i].vectors[1].ib = new_ib * MBYTE;
	d->bw_levels[i].vectors[1].ab = new_ab / d->num_paths * MBYTE;

	dev_dbg(dev, "BW MBps: AB: %d IB: %d\n", new_ab, new_ib);

	ret = msm_bus_scale_client_update_request(d->bus_client, i);
	if (ret) {
		dev_err(dev, "bandwidth request failed (%d)\n", ret);
	} else {
		d->cur_idx = i;
		d->cur_ib = new_ib;
		d->cur_ab = new_ab;
	}

	return ret;
}

static unsigned int find_ab(struct dev_data *d, unsigned long *freq)
{
	return (d->ab_percent * (*freq)) / 100;
}

static void find_freq(struct devfreq_dev_profile *p, unsigned long *freq,
			u32 flags)
{
	int i;
	unsigned long atmost, atleast, f;

	atmost = p->freq_table[0];
	atleast = p->freq_table[p->max_state-1];
	for (i = 0; i < p->max_state; i++) {
		f = p->freq_table[i];
		if (f <= *freq)
			atmost = max(f, atmost);
		if (f >= *freq)
			atleast = min(f, atleast);
	}

	if (flags & DEVFREQ_FLAG_LEAST_UPPER_BOUND)
		*freq = atmost;
	else
		*freq = atleast;
}

#ifdef CONFIG_VENDOR_ONEPLUS
static void find_freq_cpubw(struct devfreq_dev_profile *p, unsigned long *freq,
                        u32 flags)
{
        int i;
        unsigned long atmost, atleast, f;
        int min_index, max_index;

        if (cpubw_flag) {
                min_index = qos_request_value.min_devfreq;
                if (p->max_state > qos_request_value.max_devfreq)
                        max_index = qos_request_value.max_devfreq;
                else
                        max_index = p->max_state;
        } else {
                min_index = 0;
                max_index =  p->max_state;
        }

        atmost = p->freq_table[min_index];
        atleast = p->freq_table[max_index-1];

        for (i = min_index; i < max_index; i++) {
                f = p->freq_table[i];
                if (f <= *freq)
                        atmost = max(f, atmost);
                if (f >= *freq)
                        atleast = min(f, atleast);
        }

        if (flags & DEVFREQ_FLAG_LEAST_UPPER_BOUND)
                *freq = atmost;
        else
                *freq = atleast;
}

static int devbw_target_cpubw(struct device *dev, unsigned long *freq, u32 flags)
{
	struct dev_data *d = dev_get_drvdata(dev);

	find_freq_cpubw(&d->dp, freq, flags);

	if (!d->gov_ab)
		return set_bw(dev, *freq, find_ab(d, freq));
	else
		return set_bw(dev, *freq, d->gov_ab);
}
#endif

static int devbw_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct dev_data *d = dev_get_drvdata(dev);

	find_freq(&d->dp, freq, flags);

	if (!d->gov_ab)
		return set_bw(dev, *freq, find_ab(d, freq));
	else
		return set_bw(dev, *freq, d->gov_ab);
}

static int devbw_get_dev_status(struct device *dev,
				struct devfreq_dev_status *stat)
{
	struct dev_data *d = dev_get_drvdata(dev);

	stat->private_data = &d->gov_ab;
	return 0;
}

#ifdef CONFIG_VENDOR_ONEPLUS
static int devfreq_qos_handler(struct notifier_block *b, unsigned long val, void *v)
{
	unsigned int max_devfreq_index, min_devfreq_index;
	unsigned int index_max, index_min;

	max_devfreq_index = (unsigned int)pm_qos_request(PM_QOS_DEVFREQ_MAX);
	min_devfreq_index = (unsigned int)pm_qos_request(PM_QOS_DEVFREQ_MIN);

	/* add limit */
	if (max_devfreq_index & MASK_CPUFREQ) {
		index_max = MAX_CPUFREQ - max_devfreq_index;
		if (index_max > qos_request_value.max_state)
			index_max = 0;
		index_max = qos_request_value.max_state - index_max;
	} else {
		if (max_devfreq_index > qos_request_value.max_state)
			index_max = qos_request_value.max_state;
	}
	if (min_devfreq_index & MASK_CPUFREQ) {
		index_min = MAX_CPUFREQ - min_devfreq_index;
		if (index_min > (qos_request_value.max_state-1))
			index_min = 0;
		index_min = qos_request_value.max_state -1 - index_min;
	} else {
		if (min_devfreq_index > qos_request_value.max_state)
			index_min = qos_request_value.max_state -1;
	}

	qos_request_value.min_devfreq = index_min;
	qos_request_value.max_devfreq = index_max;

	return NOTIFY_OK;
}
static struct notifier_block devfreq_qos_notifier = {
        .notifier_call = devfreq_qos_handler,
};
#endif

#define PROP_PORTS "qcom,src-dst-ports"
#define PROP_TBL "qcom,bw-tbl"
#define PROP_AB_PER "qcom,ab-percent"
#define PROP_ACTIVE "qcom,active-only"

int devfreq_add_devbw(struct device *dev)
{
	struct dev_data *d;
	struct devfreq_dev_profile *p;
	u32 *data, ports[MAX_PATHS * 2];
	const char *gov_name;
	int ret, len, i, num_paths;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	dev_set_drvdata(dev, d);

	if (of_find_property(dev->of_node, PROP_PORTS, &len)) {
		len /= sizeof(ports[0]);
		if (len % 2 || len > ARRAY_SIZE(ports)) {
			dev_err(dev, "Unexpected number of ports\n");
			return -EINVAL;
		}

		ret = of_property_read_u32_array(dev->of_node, PROP_PORTS,
						 ports, len);
		if (ret)
			return ret;

		num_paths = len / 2;
	} else {
		return -EINVAL;
	}

	d->bw_levels[0].vectors = &d->vectors[0];
	d->bw_levels[1].vectors = &d->vectors[MAX_PATHS];
	d->bw_data.usecase = d->bw_levels;
	d->bw_data.num_usecases = ARRAY_SIZE(d->bw_levels);
	d->bw_data.name = dev_name(dev);
	d->bw_data.active_only = of_property_read_bool(dev->of_node,
							PROP_ACTIVE);

	for (i = 0; i < num_paths; i++) {
		d->bw_levels[0].vectors[i].src = ports[2 * i];
		d->bw_levels[0].vectors[i].dst = ports[2 * i + 1];
		d->bw_levels[1].vectors[i].src = ports[2 * i];
		d->bw_levels[1].vectors[i].dst = ports[2 * i + 1];
	}
	d->bw_levels[0].num_paths = num_paths;
	d->bw_levels[1].num_paths = num_paths;
	d->num_paths = num_paths;

	p = &d->dp;
	p->polling_ms = 50;
#ifdef CONFIG_VENDOR_ONEPLUS
	if (strstr(d->bw_data.name, "soc:qcom,cpubw") != NULL) {
		p->target = devbw_target_cpubw;
		cpubw_flag = true;
	} else
		p->target = devbw_target;
#else
	p->target = devbw_target;
#endif
	p->get_dev_status = devbw_get_dev_status;

	if (of_find_property(dev->of_node, PROP_TBL, &len)) {
		len /= sizeof(*data);
		data = devm_kzalloc(dev, len * sizeof(*data), GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		p->freq_table = devm_kzalloc(dev,
					     len * sizeof(*p->freq_table),
					     GFP_KERNEL);
		if (!p->freq_table)
			return -ENOMEM;

		ret = of_property_read_u32_array(dev->of_node, PROP_TBL,
						 data, len);
		if (ret)
			return ret;

		for (i = 0; i < len; i++)
			p->freq_table[i] = data[i];
		p->max_state = len;
	}

	if (of_find_property(dev->of_node, PROP_AB_PER, &len)) {
		ret = of_property_read_u32(dev->of_node, PROP_AB_PER,
							&d->ab_percent);
		if (ret)
			return ret;

		dev_dbg(dev, "ab-percent used %u\n", d->ab_percent);
	}

	d->bus_client = msm_bus_scale_register_client(&d->bw_data);
	if (!d->bus_client) {
		dev_err(dev, "Unable to register bus client\n");
		return -ENODEV;
	}

	if (of_property_read_string(dev->of_node, "governor", &gov_name))
		gov_name = "performance";

	d->df = devfreq_add_device(dev, p, gov_name, NULL);
	if (IS_ERR(d->df)) {
		msm_bus_scale_unregister_client(d->bus_client);
		return PTR_ERR(d->df);
	}

#ifdef CONFIG_VENDOR_ONEPLUS
	if (cpubw_flag) {
		qos_request_value.max_state = len;
		qos_request_value.min_devfreq = 0;
		qos_request_value.max_devfreq = len;
	}
#endif
	return 0;
}

int devfreq_remove_devbw(struct device *dev)
{
	struct dev_data *d = dev_get_drvdata(dev);
	msm_bus_scale_unregister_client(d->bus_client);
	devfreq_remove_device(d->df);
	return 0;
}

int devfreq_suspend_devbw(struct device *dev)
{
	struct dev_data *d = dev_get_drvdata(dev);
	return devfreq_suspend_device(d->df);
}

int devfreq_resume_devbw(struct device *dev)
{
	struct dev_data *d = dev_get_drvdata(dev);
	return devfreq_resume_device(d->df);
}

static int devfreq_devbw_probe(struct platform_device *pdev)
{
	return devfreq_add_devbw(&pdev->dev);
}

static int devfreq_devbw_remove(struct platform_device *pdev)
{
	return devfreq_remove_devbw(&pdev->dev);
}

static struct of_device_id match_table[] = {
	{ .compatible = "qcom,devbw" },
	{}
};

static struct platform_driver devbw_driver = {
	.probe = devfreq_devbw_probe,
	.remove = devfreq_devbw_remove,
	.driver = {
		.name = "devbw",
		.of_match_table = match_table,
		.owner = THIS_MODULE,
	},
};

static int __init devbw_init(void)
{
#ifdef CONFIG_VENDOR_ONEPLUS
	/* add cpufreq qos notify */
	cpubw_flag = false;
	pm_qos_add_notifier(PM_QOS_DEVFREQ_MAX, &devfreq_qos_notifier);
	pm_qos_add_notifier(PM_QOS_DEVFREQ_MIN, &devfreq_qos_notifier);
#endif
	platform_driver_register(&devbw_driver);
	return 0;
}
device_initcall(devbw_init);

MODULE_DESCRIPTION("Device DDR bandwidth voting driver MSM SoCs");
MODULE_LICENSE("GPL v2");
