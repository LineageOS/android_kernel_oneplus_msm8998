/*
 * drivers/input/misc/tri_state_key.c
 *
 * Copyright (C) 2018, Sultanxda <sultanxda@gmail.com>
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

#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "tri-state-key"
#define NR_STATES (3)

enum key_vals {
	KEYCODE_TOTAL_SILENCE = 600,
	KEYCODE_MODE_ALARMS_ONLY,
	KEYCODE_MODE_PRIORITY_ONLY,
	KEYCODE_MODE_NONE,
	KEYCODE_MODE_MAX
};

static const char *const proc_names[] = {
	"keyCode_top", "keyCode_middle", "keyCode_bottom"
};

struct tristate_data {
	struct device *dev;
	struct input_dev *input;
	struct mutex irq_lock;
	enum key_vals key_codes[NR_STATES];
	int key_gpios[NR_STATES];
	uint8_t curr_state;
};

static struct tristate_data *tdata_g;

static void send_input(struct tristate_data *t, int keycode)
{
	input_report_key(t->input, keycode, 1);
	input_sync(t->input);
	input_report_key(t->input, keycode, 0);
	input_sync(t->input);
}

static void tristate_process_state(struct tristate_data *t)
{
	unsigned long key_state = 0;
	int i;

	/* Store the GPIO values of the keys as a bitmap */
	for (i = 0; i < NR_STATES; i++)
		key_state |= !!gpio_get_value(t->key_gpios[i]) << i;

	/* Spurious interrupt; at least one of the pins should be zero */
	if (key_state == 0x7)
		return;

	/* Redundant interrupt */
	if (key_state == t->curr_state)
		return;

	t->curr_state = key_state;

	/*
	 * Find the first bit set to zero; this is the current position. Bit 0
	 * corresponds to the top position, bit 1 corresponds to the middle
	 * position, and bit 2 corresponds to the bottom position.
	 */
	i = ffz(key_state);
	send_input(t, t->key_codes[i]);
}

static irqreturn_t tristate_irq_handler(int irq, void *dev_id)
{
	struct tristate_data *t = dev_id;

	/* This handler is shared between three interrupt lines */
	mutex_lock(&t->irq_lock);
	tristate_process_state(t);
	mutex_unlock(&t->irq_lock);

	return IRQ_HANDLED;
}

static int keycode_get_idx(const char *filename)
{
	int i;

	for (i = 0; i < NR_STATES; i++) {
		if (!strcmp(proc_names[i], filename))
			break;
	}

	return i;
}

static int keycode_show(struct seq_file *seq, void *offset)
{
	struct tristate_data *t = tdata_g;
	int idx = (int)(long)seq->private;

	seq_printf(seq, "%d\n", t->key_codes[idx]);
	return 0;
}

static ssize_t tristate_keycode_write(struct file *file,
	const char __user *page, size_t count, loff_t *offset)
{
	struct tristate_data *t = tdata_g;
	char buf[sizeof("600")];
	int data, idx;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, page, min_t(int, count, 3))) {
		dev_err(t->dev, "Failed to read procfs input\n");
		return count;
	}

	if (kstrtoint(buf, 10, &data) < 0)
		return count;

	if (data < KEYCODE_TOTAL_SILENCE || data >= KEYCODE_MODE_MAX)
		return count;

	idx = keycode_get_idx(file->f_path.dentry->d_iname);

	mutex_lock(&t->irq_lock);
	t->key_codes[idx] = data;
	if (!(t->curr_state & BIT(idx)))
		send_input(t, data);
	mutex_unlock(&t->irq_lock);

	return count;
}

static int tristate_keycode_open(struct inode *inode, struct file *file)
{
	int idx = keycode_get_idx(file->f_path.dentry->d_iname);

	/* Pass the index to keycode_show */
	return single_open(file, keycode_show, (void *)(long)idx);
}

static const struct file_operations tristate_keycode_proc = {
	.owner		= THIS_MODULE,
	.open		= tristate_keycode_open,
	.read		= seq_read,
	.write		= tristate_keycode_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int tristate_parse_dt(struct tristate_data *t)
{
	struct device_node *np;
	char prop_name[sizeof("tristate,gpio_key#")];
	int i;

	np = t->dev->of_node;
	if (!np)
		return -EINVAL;

	for (i = 0; i < NR_STATES; i++) {
		sprintf(prop_name, "tristate,gpio_key%d", i + 1);

		t->key_gpios[i] = of_get_named_gpio(np, prop_name, 0);
		if (!gpio_is_valid(t->key_gpios[i])) {
			dev_err(t->dev, "Invalid %s property\n", prop_name);
			return -EINVAL;
		}
	}

	return 0;
}

static int tristate_register_irqs(struct tristate_data *t)
{
	char label[sizeof("tristate_key#-int")];
	int i, j, key_irqs[NR_STATES], ret;

	/* Get the IRQ numbers corresponding to the GPIOs */
	for (i = 0; i < NR_STATES; i++) {
		key_irqs[i] = gpio_to_irq(t->key_gpios[i]);
		if (key_irqs[i] < 0) {
			dev_err(t->dev, "Invalid IRQ (%d) for GPIO%d\n",
				key_irqs[i], i + 1);
			return -EINVAL;
		}
	}

	for (i = 0; i < NR_STATES; i++) {
		sprintf(label, "tristate_key%d-int", i + 1);

		ret = gpio_request(t->key_gpios[i], label);
		if (ret < 0) {
			dev_err(t->dev, "Failed to request GPIO%d, ret: %d\n",
				i + 1, ret);
			goto free_gpios;
		}

		ret = gpio_direction_input(t->key_gpios[i]);
		if (ret < 0) {
			dev_err(t->dev, "Failed to set GPIO%d dir, ret: %d\n",
				i + 1, ret);
			goto free_gpios;
		}

		sprintf(label, "tristate_key%d", i + 1);

		ret = request_threaded_irq(key_irqs[i], NULL,
				tristate_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT, label, t);
		if (ret < 0) {
			dev_err(t->dev, "Failed to register %s IRQ, ret: %d\n",
				label, ret);
			goto free_irqs;
		}
	}

	for (i = 0; i < NR_STATES; i++)
		enable_irq_wake(key_irqs[i]);

	return 0;

free_irqs:
	for (j = i; j--;)
		free_irq(key_irqs[j], t);
	i = NR_STATES;
free_gpios:
	for (j = i; j--;)
		gpio_free(t->key_gpios[j]);
	return -EINVAL;
}

static void tristate_create_procfs(void)
{
	struct proc_dir_entry *proc_dir;
	int i;

	proc_dir = proc_mkdir("tri-state-key", NULL);
	if (!proc_dir)
		return;

	for (i = 0; i < NR_STATES; i++)
		proc_create_data(proc_names[i], 0644, proc_dir,
			&tristate_keycode_proc, NULL);
}

static int tristate_probe(struct platform_device *pdev)
{
	struct tristate_data *t;
	int i, ret;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->dev = &pdev->dev;
	tdata_g = t;

	ret = tristate_parse_dt(t);
	if (ret)
		goto free_t;

	t->input = input_allocate_device();
	if (!t->input) {
		dev_err(t->dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto free_t;
	}

	t->input->name = DRIVER_NAME;
	t->input->dev.parent = t->dev;
	input_set_drvdata(t->input, t);

	set_bit(EV_KEY, t->input->evbit);
	set_bit(KEYCODE_TOTAL_SILENCE, t->input->keybit);
	set_bit(KEYCODE_MODE_ALARMS_ONLY, t->input->keybit);
	set_bit(KEYCODE_MODE_PRIORITY_ONLY, t->input->keybit);
	set_bit(KEYCODE_MODE_NONE, t->input->keybit);

	ret = input_register_device(t->input);
	if (ret)
		goto free_input;

	/* Initialize with default keycodes */
	for (i = 0; i < NR_STATES; i++)
		t->key_codes[i] = KEYCODE_TOTAL_SILENCE + i;

	/* Process initial state */
	tristate_process_state(t);

	mutex_init(&t->irq_lock);
	ret = tristate_register_irqs(t);
	if (ret)
		goto input_unregister;

	tristate_create_procfs();
	return 0;

input_unregister:
	input_unregister_device(t->input);
free_input:
	input_free_device(t->input);
free_t:
	kfree(t);
	return ret;
}

static const struct of_device_id tristate_of_match[] = {
	{ .compatible = "oneplus,tri-state-key", },
	{ },
};

static struct platform_driver tristate_driver = {
	.probe	= tristate_probe,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = tristate_of_match,
	},
};

static int __init tristate_init(void)
{
	return platform_driver_register(&tristate_driver);
}
device_initcall(tristate_init);
