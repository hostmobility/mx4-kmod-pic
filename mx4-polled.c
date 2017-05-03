/*
 * Copyright (C) 2017 Host Mobility AB. All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "mx4-core.h"

#include <linux/irq.h>

/* This is offset of the first input of gpio base */
#define INPUT_EVENT_OFFSET 12

static inline unsigned long ms_to_jiffies_raw(unsigned long ms)
{
	return (ms * ((unsigned long)HZ)) / 1000;
}

static inline unsigned long ms_to_jiffies(unsigned long ms)
{
	unsigned long val = ms_to_jiffies_raw (ms);
	return (val == 0) ? 1 : val;
}

static inline unsigned long jiffies_to_ms(unsigned long jiffies)
{
	return (jiffies * 1000) / HZ;
}

/* is to be called with the mutex locked */
static int is_value_polling_active(struct mx4_spi_device* mx4)
{
	return delayed_work_pending(&mx4->polled.polled_work);
}

static void notify_periodic_polled_value_changes (
	struct spi_device *spi,
	struct kobject* obj,
	u32 previous_value,
	u32 current_value)
{
	unsigned i = 0;
	struct device *dev = &spi->dev;
	struct mx4_spi_device* mx4 = dev_get_drvdata(dev);
	struct mx4_irq *irq = &mx4->irq;
	struct gpio_chip *chip = &mx4->chip;
	u32 rising, falling, rising_mask, falling_mask;

	rising_mask = (irq->irq_rise >> INPUT_EVENT_OFFSET);
	falling_mask = (irq->irq_fall >> INPUT_EVENT_OFFSET);

	rising = (current_value >> 16) & 0xffff;
	falling = (current_value & 0xffff);

	rising = rising & rising_mask;
	falling = falling & falling_mask;

	dev_dbg(dev, "input event change: rising: 0x%x, falling 0x%x,"
		"rising mask: 0x%x, falling mask: 0x%x\n",
		rising, falling, rising_mask, falling_mask);

	for (; i < mx4_polled_inputs_count; ++i) {
		int irq_nr = chip->to_irq(chip, i + INPUT_EVENT_OFFSET);
		/* rising edge */
		if (MX4_READ_INT_BIT (rising, i)) {
			dev_dbg(dev, "rising event sent to gpio: %d\n",
				(chip->base + i + INPUT_EVENT_OFFSET));
			handle_nested_irq(irq_nr);
		}

		/* falling edge */
		if (MX4_READ_INT_BIT (falling, i)) {
			dev_dbg(dev, "falling event sent to gpio: %d\n",
				(chip->base + i + INPUT_EVENT_OFFSET));
			handle_nested_irq(irq_nr);
		}
	}
}

static void cancel_event_work_que(struct mx4_spi_device* mx4)
{
	if (mx4->event_rdy_irq) {
		cancel_work_sync(&mx4->polled.event_work);
		flush_workqueue(mx4->polled.event_wq);
	}
}

int mx4_event_stop(struct mx4_spi_device* mx4)
{
	cancel_event_work_que(mx4);
	return 0;
}

/* is to be called with the mutex locked */
int mx4_polled_stop(struct mx4_spi_device* mx4)
{
	struct device *dev = &mx4->spi->dev;
	if (is_value_polling_active(mx4)) {
		/* Abort all on-going communication immediately otherwise our stop
		*  off work-queue could fail due to long timeout in spi protocol.
		*/
		mx4->spi_response_sync.has_data = 0;
		wake_up_interruptible(&mx4->spi_response_sync.queue);

		cancel_delayed_work_sync(&mx4->polled.polled_work);
		flush_workqueue(mx4->polled.polled_wq);

		dev_dbg(dev, "canceling spi periodical value polling\n");
		return 1;
	} else {
		dev_dbg(dev, "canceling spi periodical value polling: "
			"nothing to cancel\n");
		return 0;
	}
}

/* is to be called with a locked mutex */
int mx4_polled_start(struct mx4_spi_device* mx4)
{
	int val = 0;
	struct device *dev = &mx4->spi->dev;

	if (ms_to_jiffies_raw(mx4->polled.period_ms) == 0) {
		dev_warn(dev, "the polling period was tried to set to (%lu ms), "
			"which is below the minimum jiffie resolution in this machine: "
			"setting value to 1 jiffie = %lu ms", mx4->polled.period_ms,
			jiffies_to_ms (1));
	}

	if (!is_value_polling_active(mx4)) {
		dev_dbg(dev, "adding delayed work -> spi value polling\n");
		val = queue_delayed_work (mx4->polled.polled_wq, &mx4->polled.polled_work,
			ms_to_jiffies (mx4->polled.period_ms));
	}

	dev_dbg(dev, "adding delayed work -> result: %d\n", val);
	return val;
}

static int mx4_updated_event_inputs(struct mx4_spi_device *mx4)
{
	ssize_t op_result;
	struct device *dev = &mx4->spi->dev;
	mx4_spi_data_field_t current_value = 0;
	op_result = mx4_spi_read_value(mx4->spi, &current_value,
		PROT_TYPE_EVENT_INPUTS);

	if (op_result != SUCCESSFULL_MX4_RW) {
		dev_err(dev, "failed to update event inputs: %d", op_result);
		return -1;
	}

	if (current_value != mx4->polled.last_input_value) {
		notify_periodic_polled_value_changes(
			mx4->spi,
			&mx4->spi->dev.kobj,
			mx4->polled.last_input_value,
			current_value
		);

		mx4->polled.last_input_value = current_value;
	}

	return 0;
}

void mx4_event_work_callback(struct work_struct *work)
{
	struct mx4_spi_device* mx4 = container_of(work, struct mx4_spi_device,
		polled.event_work);

	mutex_lock (&mx4->lock);
	mx4_updated_event_inputs(mx4);
	mutex_unlock(&mx4->lock);
}

void mx4_ping_co_cpu(struct mx4_spi_device* mx4)
{
	struct device *dev = &mx4->spi->dev;
	ssize_t op_result;
	mx4_spi_data_field_t current_value = 0;

	op_result = mx4_spi_read_value(mx4->spi, &current_value,
		PROT_TYPE_POLLED);

	if (op_result != SUCCESSFULL_MX4_RW) {
		dev_err(dev, "Failed to ping co-cpu");
	}

}

void mx4_polled_work_callback(struct work_struct* work)
{
	struct mx4_spi_device* mx4 = container_of(work, struct mx4_spi_device,
		polled.polled_work.work);

	mutex_lock (&mx4->lock);

	/* 	If there is no event_rdy signal then the update of event inputs
		acts as an ping to co-cpu to keep the watchdog happy
	*/
	if (!mx4->event_rdy_present)
		mx4_updated_event_inputs(mx4);
	else
		mx4_ping_co_cpu(mx4);

	mx4_polled_start(mx4);
	mutex_unlock(&mx4->lock);
}
