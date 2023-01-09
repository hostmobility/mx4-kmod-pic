/*
+++                                                              +++
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+++                                                              +++
+++   COPYRIGHT (c)  HostMobility AB                             +++
+++                                                              +++
+++ The copyright to the computer Program(s) herein is the       +++
+++ property of HostMobility, Sweden. The program(s) may be      +++
+++ used and or copied only with the written permission of       +++
+++ HostMobility, or in accordance with the terms and            +++
+++ conditions stipulated in the agreement contract under        +++
+++ which the program(s) have been supplied                      +++
+++                                                              +++
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+++                                                              +++
*/

#include <linux/delay.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/version.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#endif /* CONFIG_OF */

#include "mx4-core.h"

#ifdef DEBUG
#define MX4_POLLED_FREQ_MS 50000
#else
#define MX4_POLLED_FREQ_MS 500
#endif /* DEBUG */

#define MX4_SYNC_SLEEP_TIME_MS 500

static void mx4_spi_sync (struct spi_device *spi)
{
	dev_err(&spi->dev, "Sleeping %d ms to sync up after parse error",
		MX4_SYNC_SLEEP_TIME_MS);

	msleep(MX4_SYNC_SLEEP_TIME_MS);
}

int mx4_spi_communication (struct spi_device *spi, int length)
{
	int val;
	struct mx4_spi_device* mx4 = dev_get_drvdata(&spi->dev);
	struct device *dev = &spi->dev;
	int rc;

#ifdef DEBUG
	ktime_t start, end;
	s64 actual_time;
	start = ktime_get();
#endif

	dev_dbg(&spi->dev, "communication: read request bytes: %02x %02x\n", mx4->dma_safe_buffer[0], mx4->dma_safe_buffer[1]);

	while (1) 
	{

		val = mx4_spi_write (spi, mx4->dma_safe_buffer, length);

		if (val < 0) {
			dev_err(dev, "communication request transfer failed: %d cmd = 0x%02x\n",
				val, mx4->dma_safe_buffer[1]);
			return val;
		}

		//# mx4_wait_to_receive_response refactoring
		rc = wait_event_interruptible_timeout(mx4->spi_response_sync.queue,
					(mx4->spi_response_sync.has_data == 1), HZ/8); //100/8= 32milisec

		if (rc > 0 && mx4->spi_response_sync.has_data) {
			// ok
			val = mx4->spi_response_sync.has_data;
			// clear if we gets data otherwise counter++ each spi wait timeout and let it rollover.
			mx4->pic_wait_error_counter = 0;
			break;
		} else if (!rc) {
			// timeout and resend until more than 10 tries has been performed.
			mx4->pic_wait_error_counter++;
			dev_dbg(&mx4->spi->dev, "spi wait timeout, count: %u\n", mx4->pic_wait_error_counter);
			val = 0;
			if(mx4->pic_wait_error_counter < 10)
			{
				mx4->spi_response_sync.has_data = 0;
			} else {
				dev_err(&mx4->spi->dev, "spi wait timeout, count: %u\n", mx4->pic_wait_error_counter);
				break;
			}
		} else {
			dev_err(&mx4->spi->dev, "spi wait error rc=%d has_data=%d\n",
				rc, mx4->spi_response_sync.has_data);
			val = 0;
			break;
		}
	}
	mx4->spi_response_sync.has_data = 0;

#ifdef DEBUG
	end = ktime_get();
	actual_time = ktime_to_ns(ktime_sub(end, start));
	if (val == 0) {
		dev_err(dev, "communication: co-cpu: no response received after %lld nano seconds.",(long long)actual_time);
	}
	else
		dev_err(dev, "communication: co-cpu: OK received after %lld nano seconds.",(long long)actual_time);
#endif

	if (val == 0) {
		dev_err(dev, "communication timeout: no response received. cmd = 0x%02x\n",
			mx4->dma_safe_buffer[1]);
		return -ETIMEDOUT;
	}

	val = mx4_spi_read (spi, mx4->dma_safe_buffer, MX4_SPI_READ_RESPONSE_SIZE);

	if (val < 0) {
		dev_err(dev, "communication response transfer failed: %d cmd = 0x%02x\n\n",
			val, mx4->dma_safe_buffer[1]);
		mx4_spi_sync (spi);
		return val;
	}

	return SUCCESSFULL_MX4_RW;
}

ssize_t mx4_spi_read_value (struct spi_device *spi, u32* value, u8 type)
{
	struct mx4_spi_device* mx4 = dev_get_drvdata(&spi->dev);
	struct device *dev = &spi->dev;
	u8 primitive, type_received, status, checksum;

	if(mx4->suspended){
		dev_err(dev, "mx4_spi_read_value: Mx4 is in suspended");
		return -EBUSY;
	}

	dev_dbg(dev, "request to read type: 0x%02x\n", type);
	memset( mx4->dma_safe_buffer, '\0', sizeof(char)*BUFFER_ARRAY_LENGTH );

	mx4->dma_safe_buffer[MX4_SPI_READ_REQUEST_SERVICE_PRIMITIVE_OFFSET] = MX4_SPI_READ_REQUEST;
	mx4->dma_safe_buffer[MX4_SPI_READ_REQUEST_RELATED_VALUE_OFFSET] = type;
	
	dev_dbg(&spi->dev, "read request bytes: %02x %02x\n", mx4->dma_safe_buffer[0], mx4->dma_safe_buffer[1]);

	if (mx4_spi_communication(spi, MX4_SPI_READ_REQUEST_SIZE) != SUCCESSFULL_MX4_RW)
	{
		dev_err(dev, "mx4_spi_read_value failed\n");
		/*Input/output error.*/
		return -EIO;
	}

	// Read response
	primitive 	= *((u8*)(mx4->dma_safe_buffer +
		MX4_SPI_READ_RESPONSE_SERVICE_PRIMITIVE_OFFSET));
	type_received 		= *((u8*) (mx4->dma_safe_buffer +
		MX4_SPI_READ_RESPONSE_RELATED_VALUE_OFFSET));

	status 		= *((u8*) (mx4->dma_safe_buffer + MX4_SPI_READ_RESPONSE_STATUS_OFFSET));
	*value = *((u32*) (mx4->dma_safe_buffer + MX4_SPI_READ_RESPONSE_DATA_OFFSET));
	checksum 	= *((u8*) (mx4->dma_safe_buffer + MX4_SPI_READ_RESPONSE_CHECKSUM_OFFSET));

	dev_dbg(dev, "read response bytes: %02x %02x %02x %02x%02x%02x%02x %02x \n",
		 mx4->dma_safe_buffer[0],  mx4->dma_safe_buffer[1],  mx4->dma_safe_buffer[2],  mx4->dma_safe_buffer[3],
		 mx4->dma_safe_buffer[4],  mx4->dma_safe_buffer[5],  mx4->dma_safe_buffer[6],  mx4->dma_safe_buffer[7]);

	if (primitive !=  MX4_SPI_READ_RESPONSE) {
		dev_err(dev, "read service doesn't match: expected: 0x%02x, "
			"received: 0x%02x", MX4_SPI_READ_RESPONSE, primitive);
		dev_err(dev, "read response bytes: %02x %02x %02x %02x%02x%02x%02x %02x \n",
		 	mx4->dma_safe_buffer[0],  mx4->dma_safe_buffer[1],  mx4->dma_safe_buffer[2],  mx4->dma_safe_buffer[3],
		 	mx4->dma_safe_buffer[4],  mx4->dma_safe_buffer[5],  mx4->dma_safe_buffer[6],  mx4->dma_safe_buffer[7]);
		dev_err(dev, "read response parse error. cmd = 0x%02x\n", type);
		mx4_spi_sync (spi);
		return -EINVAL;
	}

	if (type_received !=  type) {
		dev_err(dev, "read type doesn't match: expected: 0x%02x, "
			"received: 0x%02x\n", type, type_received);
		dev_err(dev, "read response bytes: %02x %02x %02x %02x%02x%02x%02x %02x \n",
		 	mx4->dma_safe_buffer[0],  mx4->dma_safe_buffer[1],  mx4->dma_safe_buffer[2],  mx4->dma_safe_buffer[3],
		 	mx4->dma_safe_buffer[4],  mx4->dma_safe_buffer[5],  mx4->dma_safe_buffer[6],  mx4->dma_safe_buffer[7]);
		dev_err(dev, "read response parse error. cmd = 0x%02x\n", type);
		mx4_spi_sync (spi);
		return -EINVAL;
	}

	if (status !=  MX4_SPI_OK) {
		dev_err(dev, "read status doesn't match: expected: 0x%02x, "
			"received: 0x%02x\n", MX4_SPI_OK, status);
		return -EIO;
	}

	if (mx4_checksum_fail(
		 mx4->dma_safe_buffer + MX4_SPI_READ_RESPONSE_CHECKSUM_FIRST_BYTE_OFFSET,
		MX4_SPI_READ_RESPONSE_CHECKSUM_BYTE_COUNT,
		checksum
		)) {
		dev_err(dev, "checksum failed, try again\n");
		dev_err(dev, "read response bytes: %02x %02x %02x %02x%02x%02x%02x %02x \n",
		 	mx4->dma_safe_buffer[0],  mx4->dma_safe_buffer[1],  mx4->dma_safe_buffer[2],  mx4->dma_safe_buffer[3],
		 	mx4->dma_safe_buffer[4],  mx4->dma_safe_buffer[5],  mx4->dma_safe_buffer[6],  mx4->dma_safe_buffer[7]);
		dev_err(dev, "read response parse error. cmd = 0x%02x\n", type);
		mx4_spi_sync (spi);
		return -EINVAL;
	}

	return  SUCCESSFULL_MX4_RW;
}

ssize_t mx4_spi_write_value(struct spi_device *spi, u32 value, u8 type)
{
#ifndef FOUND_WRITE_DELAY_ISSUE
	int val;
	int rc;
#endif

	struct mx4_spi_device* mx4 = dev_get_drvdata(&spi->dev);
	struct device *dev = &spi->dev;
#ifdef DEBUG
	ktime_t start, end;
	s64 actual_time;
	start = ktime_get();
#endif
	if(mx4->suspended){
		dev_err(dev, "mx4_spi_write_value: Mx4 is in suspended");
		return -EBUSY;
	}

	dev_dbg(dev, "request to write type: 0x%02x\n", type);
	memset( mx4->dma_safe_buffer, '\0', sizeof(char)*BUFFER_ARRAY_LENGTH );

	*((u8*) (mx4->dma_safe_buffer +
		MX4_SPI_WRITE_REQUEST_SERVICE_PRIMITIVE_OFFSET)) = MX4_SPI_WRITE_REQUEST;

	*((u8*) (mx4->dma_safe_buffer + MX4_SPI_WRITE_REQUEST_RELATED_VALUE_OFFSET)) = type;
	*((u32*) (mx4->dma_safe_buffer + MX4_SPI_WRITE_REQUEST_DATA_OFFSET)) = value;
	*((u8*) (mx4->dma_safe_buffer + MX4_SPI_WRITE_REQUEST_CHECKSUM_OFFSET)) =
		mx4_calculate_checksum(
			mx4->dma_safe_buffer + MX4_SPI_WRITE_REQUEST_CHECKSUM_FIRST_BYTE_OFFSET,
			MX4_SPI_WRITE_REQUEST_CHECKSUM_BYTE_COUNT
		);

	dev_dbg(&spi->dev, "write request bytes: %02x %02x %02x%02x%02x%02x %02x\n",
		mx4->dma_safe_buffer[0], mx4->dma_safe_buffer[1], mx4->dma_safe_buffer[2], mx4->dma_safe_buffer[3],
		mx4->dma_safe_buffer[4], mx4->dma_safe_buffer[5], mx4->dma_safe_buffer[6]);
#warning "There is an delay issue when runing mx4_spi_communication for write that is of a factor ten slower. For mx4_spi_read_value this function do work the same as before the refactoration."
#ifdef FOUND_WRITE_DELAY_ISSUE
	if (mx4_spi_communication(spi, MX4_SPI_WRITE_REQUEST_SIZE) != SUCCESSFULL_MX4_RW)
	{
		dev_err(dev, "mx4_spi_write_value failed\n");
		return -EIO;
#else

	while (1) 
	{

		val = mx4_spi_write(spi, mx4->dma_safe_buffer, MX4_SPI_WRITE_REQUEST_SIZE);

		if (val < 0) {
			dev_err(dev, "write request transfer failed: %d cmd = 0x%02x\n",
				val, mx4->dma_safe_buffer[1]);
			return val;
		}

		//# # mx4_wait_to_receive_response refactoring
		rc = wait_event_interruptible_timeout(mx4->spi_response_sync.queue,
					(mx4->spi_response_sync.has_data == 1), HZ/8); //100/8= 32milisec

		if (rc > 0 && mx4->spi_response_sync.has_data) {
			// ok
			val = mx4->spi_response_sync.has_data;
			// clear if we gets data otherwise counter++ each spi wait timeout and let it rollover.
			mx4->pic_wait_error_counter = 0;
			break;
		} else if (!rc) {
			// timeout and resend until more than 10 tries has been performed.
			mx4->pic_wait_error_counter++;
			dev_dbg(&mx4->spi->dev, "spi wait timeout, count: %u\n", mx4->pic_wait_error_counter);
			val = 0;
			if(mx4->pic_wait_error_counter < 10)
			{
				mx4->spi_response_sync.has_data = 0;
			} else {
				dev_err(&mx4->spi->dev, "spi wait timeout, count: %u\n", mx4->pic_wait_error_counter);
				break;
			}
		} else {
			dev_err(&mx4->spi->dev, "spi wait error rc=%d has_data=%d\n",
				rc, mx4->spi_response_sync.has_data);
			val = 0;
			break;
		}
	}
	mx4->spi_response_sync.has_data = 0;

#ifdef DEBUG
	end = ktime_get();
	actual_time = ktime_to_ns(ktime_sub(end, start));
	if (val == 0) {
		dev_err(dev, "write: co-cpu: no response received after %lld nano seconds.",(long long)actual_time);
	}else
		dev_err(dev, "write: co-cpu: OK received after %lld nano seconds.",(long long)actual_time);
#endif

	if (val == 0) {
		dev_err(dev, "write timeout: no response received. cmd = 0x%02x\n",
			type);
		return -ETIMEDOUT;
	}

	val = mx4_spi_read (spi, mx4->dma_safe_buffer, MX4_SPI_WRITE_RESPONSE_SIZE);

	if (val < 0) {
		dev_err(dev, "write response transfer failed: %d cmd = 0x%02x\n",
			val, type);
		return -ETIMEDOUT;
	}
#endif
	// Read response
	dev_dbg(dev, "write response bytes: %02x %02x %02x\n",
		mx4->dma_safe_buffer[0], mx4->dma_safe_buffer[1], mx4->dma_safe_buffer[2]);

	if (mx4->dma_safe_buffer[MX4_SPI_WRITE_RESPONSE_SERVICE_PRIMITIVE_OFFSET] !=  MX4_SPI_WRITE_RESPONSE) {
		dev_err(dev, "write service doesn't match: expected: 0x%02x, "
			"received: 0x%02x\n", MX4_SPI_WRITE_RESPONSE, mx4->dma_safe_buffer[MX4_SPI_WRITE_RESPONSE_SERVICE_PRIMITIVE_OFFSET]);
		dev_err(dev, "write response bytes: %02x %02x %02x\n",
		mx4->dma_safe_buffer[0], mx4->dma_safe_buffer[1], mx4->dma_safe_buffer[2]);
		return -EINVAL;
	}

	if (mx4->dma_safe_buffer[MX4_SPI_WRITE_RESPONSE_RELATED_VALUE_OFFSET] !=  type) {
		dev_err(dev, "write type doesn't match: expected: 0x%02x, "
			"received: 0x%02x\n", type, mx4->dma_safe_buffer[MX4_SPI_WRITE_RESPONSE_RELATED_VALUE_OFFSET]);
		dev_err(dev, "write response bytes: %02x %02x %02x\n",
		mx4->dma_safe_buffer[0], mx4->dma_safe_buffer[1], mx4->dma_safe_buffer[2]);
		return -EINVAL;
	}

	if (mx4->dma_safe_buffer[MX4_SPI_WRITE_RESPONSE_STATUS_OFFSET] !=  MX4_SPI_OK) {
		dev_err(dev, "write status doesn't match: expected: 0x%02x, "
			"received: 0x%02x\n", MX4_SPI_OK, mx4->dma_safe_buffer[MX4_SPI_WRITE_RESPONSE_STATUS_OFFSET]);
		return -EIO;
	}

	return SUCCESSFULL_MX4_RW;
}

static int mx4_spi_wakup_pic(struct mx4_spi_device *mx4)
{
	int val;
	int i;
	struct device *dev = &mx4->spi->dev;
	struct spi_device *spi = mx4->spi;

	for(i = 0; i < 8; ++i)
	{
		mx4->dma_safe_buffer[i] = 0xff;
	}

	/* We send 8 bytes to make sure protocol overflows */
	val = mx4_spi_communication(spi, (MX4_SPI_WRITE_REQUEST_SIZE + 1) );

	if (val < 0) {
		dev_err(dev, "wakeup pic request transfer failed: %d\n", val);
		return val;
	}
	if (val == 0) {
		dev_err(dev, "wakeup pic no sync received\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static irqreturn_t spi_transfer_irq_handler (int irq, void *dev_id)
{
	struct spi_device *spi = (struct spi_device*)dev_id;
	struct device *dev = &spi->dev;
	struct mx4_spi_device* mx4 = dev_get_drvdata(&spi->dev);

	if (mx4->suspended) {
		mx4->pic_wake_up = 1;
		dev_info(dev, "wakeup source was pic\n");
		return IRQ_HANDLED;
	}

	dev_dbg(dev, "spi transfer interrupt received\n");

	mx4->spi_response_sync.has_data = 1;
	wake_up_interruptible(&mx4->spi_response_sync.queue);

	return IRQ_HANDLED;
}

static irqreturn_t spi_event_irq_handler(int irq, void *dev_id)
{
	struct spi_device *spi = (struct spi_device*)dev_id;
	struct mx4_spi_device* mx4 = dev_get_drvdata(&spi->dev);
	struct device *dev = &spi->dev;
	struct mx4_polling *poll = &mx4->polled;

	dev_dbg(dev, "spi event interrupt received\n");

	if (mx4->suspended) {
		mx4->pic_wake_up = 1;
		dev_info(dev, "wakeup source was pic\n");
		return IRQ_HANDLED;
	}

	/* Event queue might still be uninitialized, double check*/
	if (!poll->event_wq || mx4->pic_suspended)
		return IRQ_HANDLED;

	queue_work(poll->event_wq, &poll->event_work);
	return IRQ_HANDLED;
}


static int mx4_setup_event_interrupt(struct spi_device* spi,
	struct mx4_spi_device* mx4)
{
	int ret;
	struct device *dev = &spi->dev;

#ifdef CONFIG_OF
	mx4->event_rdy = devm_gpiod_get(dev, NULL, GPIOD_ASIS);
	mx4->event_rdy_irq = (!IS_ERR(mx4->event_rdy)) ?
		gpiod_to_irq(mx4->event_rdy) : 0;
#else
	struct mx4_io_platform_data *pdata = spi->dev.platform_data;
	mx4->event_rdy_irq = (pdata->event_rdy > 0) ? pdata->event_rdy : 0;
#endif /* CONFIG_OF*/

	if (mx4->event_rdy_irq) {
		ret = request_irq (mx4->event_rdy_irq, spi_event_irq_handler,
				IRQF_TRIGGER_RISING, dev_name (&spi->dev), spi);

		if (ret) {
			dev_err(dev, "request irq for irq number=%d failed: %d\n",
				mx4->event_rdy_irq, ret);
			return -1;
		}

		mx4->event_rdy_present = 1;
	} else {
		dev_info(dev, "No event RDY signal specified");
	}

	return 0;
}

static int mx4_io_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct mx4_spi_device *mx4 = container_of(chip, struct mx4_spi_device, chip);

#ifdef CONFIG_OF
	return irq_find_mapping(mx4->irq.irq_domain, offset);
#else
	return mx4->irq.base + offset;
#endif /* CONFIG_OF */
}

static void mx4_io_irq_mask(struct irq_data *data)
{
	struct mx4_spi_device *mx4 = irq_data_get_irq_chip_data(data);
	unsigned int pos = data->hwirq;

	mx4->irq.cache &= ~BIT(pos);
}

static void mx4_io_irq_unmask(struct irq_data *data)
{
	struct mx4_spi_device *mx4 = irq_data_get_irq_chip_data(data);
	unsigned int pos = data->hwirq;

	mx4->irq.cache |= BIT(pos);
}

static int mx4_io_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct mx4_spi_device *mx4 = irq_data_get_irq_chip_data(data);
	struct mx4_irq *irq = &mx4->irq;
#ifdef CONFIG_OF
	unsigned int pos = data->hwirq;
#else
	unsigned int pos = data->irq - mx4->irq.base;
#endif /* CONFIG_OF */

	if ((type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH) {
		irq->cache &= ~BIT(pos);
		irq->irq_rise |= BIT(pos);
		irq->irq_fall |= BIT(pos);
	} else if (type & IRQ_TYPE_EDGE_RISING) {
		irq->cache &= ~BIT(pos);
		irq->irq_rise |= BIT(pos);
		irq->irq_fall &= ~BIT(pos);
	} else if (type & IRQ_TYPE_EDGE_FALLING) {
		irq->cache &= ~BIT(pos);
		irq->irq_rise &= ~BIT(pos);
		irq->irq_fall |= BIT(pos);
	} else
		return -EINVAL;

	return 0;
}

static struct irq_chip mx4_io_irq_chip = {
	.name = "gpio-mx4",
	.irq_mask = mx4_io_irq_mask,
	.irq_unmask = mx4_io_irq_unmask,
	.irq_set_type = mx4_io_irq_set_type,
	.flags		= IRQCHIP_MASK_ON_SUSPEND,
};

static int mx4_io_irq_setup(struct mx4_spi_device *mx4)
{
	struct gpio_chip *chip = &mx4->chip;
	int irq, j;
#ifdef CONFIG_OF
	struct mx4_irq *irq_priv = &mx4->irq;
#endif

	mutex_init(&mx4->irq.irq_lock);

#ifdef CONFIG_OF
	irq_priv->irq_domain = irq_domain_add_linear(mx4->dev->of_node, chip->ngpio,
					&irq_domain_simple_ops, mx4);

	if (!irq_priv->irq_domain)
		return -ENODEV;
#endif

	chip->to_irq = mx4_io_gpio_to_irq;

	for (j = 0; j < mx4->chip.ngpio; j++) {
#ifdef CONFIG_OF
		irq = irq_create_mapping(irq_priv->irq_domain, j);
#else
		irq = mx4_io_gpio_to_irq(chip, j);
#endif /* CONFIG_OF */
		irq_set_chip_data(irq, mx4);
		irq_set_chip_and_handler(irq, &mx4_io_irq_chip, handle_simple_irq);
		irq_set_nested_thread(irq, 1);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,0,0)
		set_irq_flags(irq, IRQF_VALID);
#else
		irq_set_noprobe(irq);
#endif
	}
	return 0;
}

static void mx4_io_gpio_irq_remove(struct mx4_spi_device *mx4)
{
	int irq;

#ifdef CONFIG_OF
	unsigned int i;
	for (i = 0; i < mx4->chip.ngpio; i++) {
		irq = irq_find_mapping(mx4->irq.irq_domain, i);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(mx4->irq.irq_domain);
#else
	int base = mx4->irq.base;
	for (irq = base; irq < base + mx4->chip.ngpio; irq++) {
#ifdef CONFIG_ARM
		set_irq_flags(irq, 0);
#endif
		irq_set_chip_and_handler(irq, NULL, NULL);
		irq_set_chip_data(irq, NULL);
	}
#endif /* CONFIG_OF */
}

static int mx4_io_probe (struct spi_device *spi)
{
	struct mx4_spi_device *mx4 = NULL;
	struct device *dev = &spi->dev;
	int ret;

	dev_info(dev, "probing spi pic io driver\n");

	mx4 = kzalloc (sizeof (struct mx4_spi_device), GFP_KERNEL);

	if (!mx4) {
		ret = -ENOMEM;
		dev_err(dev, "failed to allocate memory: %d\n", ret);
		goto print_and_exit;
	}

	mutex_init(&mx4->lock);

	mx4_gpio_init(&mx4->chip);

#ifndef CONFIG_OF
	mx4->irq.base = 500;
#endif /* CONFIG_OF */

	init_waitqueue_head(&mx4->spi_response_sync.queue);

	mx4->dev = dev;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,5,0)
	mx4->chip.dev = dev;
#else
	mx4->chip.parent = dev;
#endif
	mx4->spi = spi;
	mx4->event_rdy_present = 0;
	mx4->bootloader.active = 0;
	mx4->spi_response_sync.has_data = 0;
	mx4->polled.period_ms = MX4_POLLED_FREQ_MS;
	mx4->polled.last_input_value = 0;
	mx4->suspended = 0;
	mx4->pic_suspended = 0;
	mx4->pic_wake_up = 0;
	mx4->pic_wait_error_counter = 0;

	dev_set_drvdata (&spi->dev, mx4);

	ret = sysfs_create_group(&spi->dev.kobj, mx4_attributes_get());

	if (ret < 0 ) {
		dev_err(dev, "Failed to create attributes");
		goto exit_destroy_analog_group;
	}

	if (mx4_io_irq_setup(mx4) < 0) {
		dev_err(dev, "failed to setup irq domain");
		goto exit_destroy_gpio;
	}

	ret = gpiochip_add(&mx4->chip);

	if (ret < 0) {
		dev_err(dev, "failed to add the gpio chip: %d\n", ret);
		goto exit_destroy_gpio;
	}

	ret = mx4_gpio_configure(mx4->chip.base);

	if (ret < 0) {
		dev_err(dev,
			"failed to configure gpio´s: %d\n", ret);
		goto exit_destroy_irq;
	}

	ret = request_irq (spi->irq, spi_transfer_irq_handler, IRQF_TRIGGER_RISING,
			   dev_name (&spi->dev), spi);

	if (ret) {
		dev_err(dev,
			"request irq for irq number=%d failed: %d\n", spi->irq, ret);
		goto exit_destroy_irq;
	}

	ret = mx4_setup_event_interrupt(spi, mx4);
	if (ret) {
		goto exit_destroy_post_irq;
	}

	mx4->polled.polled_wq = create_singlethread_workqueue ("mx4_polled_inputs");
	if (mx4->polled.polled_wq == NULL) {
		dev_err(dev, "unable to create polling work queue:%d\n", ret);
		goto exit_destroy_post_irq;
	}

	INIT_DELAYED_WORK (&mx4->polled.polled_work, mx4_polled_work_callback);

	if (mx4->event_rdy_present) {
		mx4->polled.event_wq = create_singlethread_workqueue ("mx4_input_events");
		if (mx4->polled.event_wq == NULL) {
			dev_err(dev, "unable to create event work queue:%d\n", ret);
			goto exit_destroy_post_irq;
		}
		INIT_WORK(&mx4->polled.event_work,mx4_event_work_callback);
	}

	if (!mx4_polled_start(mx4))
	{
		dev_err(dev, "unable to queue work in the queue:%d\n", ret);
		goto exit_destroy_all;
	}

	return ret;


exit_destroy_all:
	if (mx4->event_rdy_present)
		free_irq(mx4->event_rdy_irq, spi);

exit_destroy_post_irq:

	free_irq (spi->irq, spi);

exit_destroy_irq:
	mx4_io_gpio_irq_remove(mx4);
	mx4_gpio_clear(mx4->chip.base);
	gpiochip_remove (&mx4->chip);

exit_destroy_gpio:
	sysfs_remove_group(&spi->dev.kobj, mx4_attributes_get());

exit_destroy_analog_group:

	dev_set_drvdata (&spi->dev, NULL);
	mutex_destroy (&mx4->lock);
	kfree (mx4);

print_and_exit:

	dev_err(dev, "failed to probe the mx4 gpio driver: %s: %d\n", MX4_IO_DRV_NAME, ret);
	return ret;
}

static int mx4_io_remove (struct spi_device *spi)
{
	struct mx4_spi_device *mx4  = dev_get_drvdata(&spi->dev);
	struct device *dev = &spi->dev;

	dev_info(dev, "removing (un-probing) spi pic io driver\n");

	mx4_polled_stop(mx4);
	destroy_workqueue(mx4->polled.polled_wq);

	if (mx4->event_rdy_irq) {
		cancel_work_sync(&mx4->polled.event_work);
		flush_workqueue(mx4->polled.event_wq);
		destroy_workqueue(mx4->polled.event_wq);

		free_irq(mx4->event_rdy_irq, spi);
	}

	free_irq(spi->irq, spi);

	sysfs_remove_group(&spi->dev.kobj, mx4_attributes_get());

	mx4_io_gpio_irq_remove(mx4);

	dev_set_drvdata(&spi->dev, NULL);
	gpiochip_remove(&mx4->chip);

	mutex_destroy (&mx4->lock);
	kfree (mx4);

	return 0;
}

#ifdef CONFIG_PM
static int mx4_spi_suspend(struct device *dev)
{
	struct mx4_spi_device* mx4  = dev_get_drvdata (dev);
	struct spi_device *spi = mx4->spi;
	int ret = 0, cnt = 0;

	dev_info(dev, "Entering suspend state.");

	mutex_lock (&mx4->lock);

	mx4_polled_stop(mx4);
	mx4_event_stop(mx4);

	/* Tell pic to go to sleep.*/
	do {
		ret = mx4_spi_write_value (mx4->spi, (u32)0x01,
			PROT_TYPE_CTRL_GO_TO_SLEEP);

		if(ret != SUCCESSFULL_MX4_RW)
			dev_err(dev, "Failed to set PIC to sleep mode: %d", ret);
		else
			break;

	} while(++cnt != MX4_IO_SUSPEND_RESUME_MAX_TRY);

	if(cnt == MX4_IO_SUSPEND_RESUME_MAX_TRY)
		goto error;

	if (ret != SUCCESSFULL_MX4_RW) {
		dev_err(dev, "timeout waiting for pic sleep sync\n");
		return -ETIMEDOUT;
	}

	/*log ktime when enter suspend*/
	mx4->time_start_suspend = ktime_get();

	dev_info(dev, "Suspend succeeded after %d retries. Enter suspend at %lld nano second\n ", cnt, ktime_to_ns(mx4->time_start_suspend));

	mx4->suspended = 1;
	mx4->pic_suspended = 1;
	mx4->pic_wake_up = 0;

	enable_irq_wake(spi->irq);

	if (mx4->event_rdy_irq)
		enable_irq_wake(mx4->event_rdy_irq);

	mutex_unlock (&mx4->lock);

	return 0;

error:
	mutex_unlock (&mx4->lock);
	return ret;
}

static int mx4_spi_resume(struct device *dev)
{
	struct mx4_spi_device* mx4  = dev_get_drvdata (dev);
	struct spi_device *spi = mx4->spi;
	int ret = 0, cnt = 0;
	ktime_t end;
	s64 actual_time;

	dev_info(dev, "Leaving suspend state.");

	mutex_lock (&mx4->lock);

	disable_irq_wake(spi->irq);

	if (mx4->event_rdy_irq)
		disable_irq_wake(mx4->event_rdy_irq);

	mx4->suspended = 0;

	if (!mx4->pic_wake_up) {
		/* Wake up PIC */
		do {
			ret = mx4_spi_wakup_pic(mx4);

			if(ret)
				dev_err(dev, "Failed to wake up PIC: %d", ret);
			else
				break;

		} while(++cnt != MX4_IO_SUSPEND_RESUME_MAX_TRY);

		if(cnt == MX4_IO_SUSPEND_RESUME_MAX_TRY)
			goto error;
		end = ktime_get();
		actual_time = ktime_to_ns(ktime_sub(end, mx4->time_start_suspend));
		dev_info(dev, "Wake-up succeeded after %d retries\n. Resume system at %lld nano second.\n Suspend last for %lld nano seconds", cnt, ktime_to_ns(end), (long long)actual_time);
	}

	mx4->pic_suspended = 0;
	mx4_polled_start(mx4);
	mutex_unlock (&mx4->lock);

	return 0;

error:
	/* We are still suspended */
	mx4->suspended = 1;
	mutex_unlock (&mx4->lock);
	return ret;
}
#endif

static const struct dev_pm_ops mx4_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(mx4_spi_suspend, mx4_spi_resume)
};

#ifdef CONFIG_OF
static const struct of_device_id mx4_io_spi_dt_ids[] = {
	{ .compatible = "mx4, mx4_pic" },
	{},
};

MODULE_DEVICE_TABLE(of, mx4_io_spi_dt_ids);
#endif /* CONFIG_OF */

static struct spi_driver mx4_io_driver =
{
	.driver =
	{
		.name	= MX4_IO_DRV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(mx4_io_spi_dt_ids),
#endif /* CONFIG_OF */
		.pm = &mx4_pmops,
	},
	.probe	= mx4_io_probe,
	.remove	= mx4_io_remove,
};

static int __init mx4_io_init (void)
{
	printk ("loading spi pic io driver\n");
	return spi_register_driver (&mx4_io_driver);
}
subsys_initcall (mx4_io_init);

static void __exit mx4_io_exit (void)
{
	printk ("unloading spi pic io driver\n");
	spi_unregister_driver (&mx4_io_driver);
}
module_exit(mx4_io_exit);

MODULE_AUTHOR ("Host Mobility");
MODULE_DESCRIPTION ("mx4 io");
MODULE_LICENSE ("GPL");
MODULE_VERSION (DRIVER_VERSION);
