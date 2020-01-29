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

#include <linux/gpio.h>

#include "mx4-core.h"

struct mx4_gpio {
	u8 type;
	u8 offset;
	const char *name;
	unsigned long flags;
};

static const struct mx4_gpio mx4_gpios[] = {
	{PROT_TYPE_DIGITAL_OUT_1, 0, "digital-out-1-source", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_2, 1, "digital-out-2-source", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_3, 2, "digital-out-3-source", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_4, 3, "digital-out-4-source", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_5, 4, "digital-out-5-source", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_6, 5, "digital-out-6-sink", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_7, 6, "digital-out-7-sink", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_S_DIGITAL_OUT_1, 7, "digital-out-1-sink", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_S_DIGITAL_OUT_2, 8, "digital-out-2-sink", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_S_DIGITAL_OUT_3, 9, "digital-out-3-sink", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_S_DIGITAL_OUT_4, 10, "digital-out-4-sink", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_S_DIGITAL_OUT_5, 11, "digital-out-5-sink", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_S_DIGITAL_OUT_6, 12, "digital-out-6-source", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_S_DIGITAL_OUT_7, 13, "digital-out-7-source", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIG_OUT_EN, 14, "digital-out-en", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_1, 15, "digital-out-1", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_2, 16, "digital-out-2", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_3, 17, "digital-out-3", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_4, 18, "digital-out-4", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_5, 19, "digital-out-5 / 4-20mA", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_6, 20, "digital-out-6", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_POWER_ON_VREF, 21, "vref", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_POWER_ON_5V, 22, "5V", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_POWER_ON_3_3V, 23, "3.3V", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_LIN_SLEEP, 24, "LIN sleep", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_LIN2_SLEEP, 25, "LIN2 sleep", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_OUT_5V_ON, 26, "5V out", GPIOF_OUT_INIT_LOW},

	/* Inputs need to here for event generation/signaling*/
	{PROT_TYPE_DIGITAL_IN_1, 27, "digital-in-1 / sc", GPIOF_IN},
	{PROT_TYPE_DIGITAL_IN_2, 28, "digital-in-2 / sc", GPIOF_IN},
	{PROT_TYPE_DIGITAL_IN_3, 29, "digital-in-3 / sc", GPIOF_IN},
	{PROT_TYPE_DIGITAL_IN_4, 30, "digital-in-4 / sc", GPIOF_IN},
	{PROT_TYPE_DIGITAL_IN_5, 31, "digital-in-5 / sc", GPIOF_IN},
	{PROT_TYPE_DIGITAL_IN_6, 32, "digital-in-6", GPIOF_IN},
	{PROT_TYPE_MODEM_SYNC, 33, "modem sync", GPIOF_IN},
	{PROT_TYPE_MODEM_RING, 34, "modem ring", GPIOF_IN},
	{PROT_TYPE_MODEM_CURRENT_IND, 35, "modem current ind", GPIOF_IN},
	{PROT_TYPE_MODEM_POWER_IND, 36, "modem power ind", GPIOF_IN},
	{PROT_TYPE_START_SW, 37, "start switch", GPIOF_IN},
	{PROT_TYPE_WAKE_ON_CAN, 38, "CAN-WAKEUP", GPIOF_IN},
	{PROT_TYPE_MUX_IN_1, 39, "MUX-IN-1", GPIOF_IN},
	{PROT_TYPE_MUX_IN_2, 40, "MUX-IN-2", GPIOF_IN},
	{PROT_TYPE_WAKE_ON_CAN_1, 41, "CAN0-WAKEUP", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_WAKE_ON_CAN_2, 42, "CAN1-WAKEUP", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_WAKE_ON_CAN_3, 43, "CAN2-WAKEUP", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_WAKE_ON_CAN_4, 44, "CAN3-WAKEUP", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_WAKE_ON_CAN_5, 45, "CAN4-WAKEUP", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_WAKE_ON_CAN_6, 46, "CAN5-WAKEUP", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_MUX_OUT_1, 47, "MUX-OUT-1", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_MUX_OUT_2, 48, "MUX-OUT-2", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_LIN_ENABLED, 49, "LIN enabled", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_LIN2_ENABLED, 50, "LIN2 enabled", GPIOF_OUT_INIT_HIGH},
	{PROT_TYPE_DIGITAL_IN_7, 51, "digital-in-7", GPIOF_IN},
	{PROT_TYPE_DIGITAL_IN_8, 52, "digital-in-8", GPIOF_IN},
	{PROT_TYPE_DIGITAL_OUT_7, 53, "digital-out-7", GPIOF_OUT_INIT_LOW},
	{PROT_TYPE_DIGITAL_OUT_8, 54, "digital-out-8", GPIOF_OUT_INIT_LOW},
};

static inline u8 mx4_gpio_get_type(unsigned gpio)
{
	if (gpio >= ARRAY_SIZE(mx4_gpios))
	{
		return PROT_TYPE_INVALID;
	}

	return mx4_gpios[gpio].type;
}

static int mx4_gpio_get(struct gpio_chip *gc, unsigned gpio_num)
{
	u8 type_code = mx4_gpio_get_type (gpio_num);

	if (type_code != PROT_TYPE_INVALID) {
		struct mx4_spi_device* mx4 = container_of(gc,struct mx4_spi_device,
			chip);
		ssize_t ret;
		u32 val;

		mutex_lock (&mx4->lock);
		ret = mx4_spi_read_value(mx4->spi, &val, type_code);
		mutex_unlock(&mx4->lock);
		return (ret == SUCCESSFULL_MX4_RW) ? (int) val : -EIO;
	}

	return -EINVAL;
}

static void mx4_gpio_set(struct gpio_chip *gc, unsigned gpio_num, int val)
{
	u8 type_code = mx4_gpio_get_type (gpio_num);

	if (type_code != PROT_TYPE_INVALID) {
		struct mx4_spi_device* mx4 = container_of(gc, struct mx4_spi_device,
			chip);

		mutex_lock (&mx4->lock);
		mx4_spi_write_value(mx4->spi, (u32) val, type_code);
		mutex_unlock(&mx4->lock);
	}
}

static int mx4_gpio_direction_in(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static int mx4_gpio_direction_out(struct gpio_chip *chip,
				     unsigned offset, int value)
{
	return 0;
}

int mx4_gpio_configure(int gpio_base)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(mx4_gpios); ++i) {
		err = gpio_request_one(gpio_base + i, mx4_gpios[i].flags,
			mx4_gpios[i].name);

		if (err) {
			pr_warning("mx4_gpio_configure (%s) failed, err = %d",
				   mx4_gpios[i].name, err);

			return -1;
		}

		gpio_export (gpio_base + i, false);
	}

	return 0;
}

void mx4_gpio_clear(int gpio_base)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mx4_gpios); ++i) {
		gpio_unexport (gpio_base + i);
		gpio_free (gpio_base + i);
	}
}

void mx4_gpio_init(struct gpio_chip* chip)
{
	chip->label 			= "mx4_digitals",
	chip->owner				= THIS_MODULE,
	chip->get				= mx4_gpio_get,
	chip->set				= mx4_gpio_set,
	chip->direction_input	= mx4_gpio_direction_in,
	chip->direction_output	= mx4_gpio_direction_out,
	chip->base 				= -1,
	chip->ngpio 			= ARRAY_SIZE(mx4_gpios);
}
