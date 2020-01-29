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

#ifndef __MX4_CORE_H__
#define __MX4_CORE_H__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>


#define MX4_LINUX_KERNEL_SPACE 1 //to *_spi_protocol_defs.h

#include "mx4_spi_protocol_defs.h"

struct mx4_polling
{
	struct workqueue_struct* 	polled_wq;
	struct workqueue_struct* 	event_wq;
	struct delayed_work 		polled_work;
	struct work_struct	 		event_work;
	mx4_spi_data_field_t 		last_input_value;
	unsigned long			period_ms;
};

struct spi_response_sync
{
	wait_queue_head_t queue;
	u8 has_data;
};

struct bootloader
{
	u8	active;
};

struct mx4_io_platform_data {
	int event_rdy;
};

struct mx4_irq {
	u32 cache;

	u32	irq_rise;
	u32 irq_fall;

	int	irq;

	int base;

	struct mutex		irq_lock;
	struct irq_domain	*irq_domain;
};

struct mx4_spi_device
{
	/* locks this struct data access and a r/w operation. */
	struct mutex lock;

	/* Contains gpio chip structure */
	struct gpio_chip chip;

	struct mx4_irq irq;

	/* WAKE-UP-CPU signal from CO-CPU. Is used to signal event RDY */
	struct gpio_desc *event_rdy;
	int event_rdy_irq;
	u8 event_rdy_present;

	/**/
	struct spi_device* spi;
	struct device* dev;

	/**/
	struct spi_response_sync spi_response_sync;

	/**/
	struct bootloader bootloader;

	/**/
	struct mx4_polling polled;

 	/* Enable if suspended */
	u8 suspended;
	u8 pic_suspended;
	u8 pic_wake_up;
	u8 pic_wait_error_counter;
};

#define MX4_IO_DRV_NAME	"mx4_io_spi"
#define DRIVER_VERSION	"2.3"

#define MX4_IO_SUSPEND_RESUME_MAX_TRY	3

#define SUCCESSFULL_MX4_RW 4

#define mx4_spi_write spi_write
#define mx4_spi_read spi_read

extern void mx4_gpio_init(struct gpio_chip *chip);
extern void mx4_gpio_clear(int gpio_base);
extern int mx4_gpio_configure(int gpio_base);

extern void mx4_polled_work_callback(struct work_struct *work);
extern void mx4_event_work_callback(struct work_struct *work);
extern int mx4_polled_start(struct mx4_spi_device *mx4);
extern int mx4_polled_stop(struct mx4_spi_device *mx4);
extern int mx4_event_stop(struct mx4_spi_device *mx4);

extern const struct attribute_group *mx4_attributes_get(void);

ssize_t mx4_spi_read_value (struct spi_device *spi, u32 *value, u8 type);
ssize_t mx4_spi_write_value (struct spi_device *spi, u32 value, u8 type);

#endif /* __MX4_CORE_H */
