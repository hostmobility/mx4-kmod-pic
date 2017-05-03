/*
 * Copyright (C) 2017 Host Mobility AB. All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "mx4-core.h"

#define MX4_STRINGIFY_EXPANDED(string) #string
#define MX4_STRINGIFY(string) MX4_STRINGIFY_EXPANDED(string)

static ssize_t write_remove_exported_gpios (struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mx4_spi_device* mx4 = dev_get_drvdata(&spi->dev);

	mx4_gpio_clear(mx4->chip.base);

	return count;
}

static DEVICE_ATTR (drv_remove_exported_gpios, S_IWUSR | S_IWGRP, NULL,
	write_remove_exported_gpios);

#define START_BOOTLOADER_ATTRIBUTE_NAME start_pic_bootloader_mode
static ssize_t write_pic_bootloader_mode (struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
	ssize_t ret;
	struct spi_device *spi = to_spi_device(dev);
	struct mx4_spi_device* mx4 = dev_get_drvdata(&spi->dev);

	if (kstrtoul (buf, 10, &value) < 0) {
		return -EINVAL;
	}

	mutex_lock (&mx4->lock);

	if (mx4->bootloader.active || !value) {
		mutex_unlock (&mx4->lock);
		return -EINVAL;
	}

	dev_dbg(dev, "will try to set pic in bootloader mode\n");
	mx4_polled_stop(mx4);

	ret = mx4_spi_write_value (mx4->spi, 1, PROT_TYPE_CTRL_ENTER_BL_MODE);
	mx4->bootloader.active = (ret == SUCCESSFULL_MX4_RW );

	if (mx4->bootloader.active) {
		dev_dbg(dev, "pic bootloader mode success, notifying this file\n");
	}
	else {
		mx4->bootloader.active = 0;
		dev_err(dev,
			"pic failed to enter in bootloader mode, notifying this file\n");
	}

	sysfs_notify (&mx4->spi->dev.kobj, NULL,
		MX4_STRINGIFY (START_BOOTLOADER_ATTRIBUTE_NAME));
	mutex_unlock (&mx4->lock);

	return ret;
}

static ssize_t read_pic_bootloader_mode (struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct spi_device *spi = to_spi_device(dev);
	struct mx4_spi_device* mx4 = dev_get_drvdata(&spi->dev);

	mutex_lock (&mx4->lock);
	ret = sprintf (buf, "%d\n", mx4->bootloader.active);
	mutex_unlock (&mx4->lock);

	return ret;
}

static DEVICE_ATTR(
	start_pic_bootloader_mode,
	S_IRUGO | S_IWUSR | S_IWGRP,
	read_pic_bootloader_mode,
	write_pic_bootloader_mode
	);

#define def_read_integer_func_call(read_attribute_function_name, type)\
\
static ssize_t read_attribute_function_name (struct device *dev,\
	struct device_attribute *attr, char *buf)\
{\
	mx4_spi_data_field_t value;\
	ssize_t ret;\
	struct spi_device *spi = to_spi_device(dev);\
	struct mx4_spi_device* mx4 = dev_get_drvdata(&spi->dev);\
\
	mutex_lock (&mx4->lock);\
	ret = mx4_spi_read_value (mx4->spi, &value, type);\
	mutex_unlock (&mx4->lock);\
\
	if (ret == SUCCESSFULL_MX4_RW)\
	{\
		ret = snprintf (buf, mx4_spi_data_field_char_digits + 2, "%d\n", value);\
	}\
\
	return ret;\
\
}

#define def_write_integer_func_call(write_attribute_function_name, type)\
\
static ssize_t write_attribute_function_name (struct device *dev,\
	struct device_attribute *attr, const char *buf, size_t count)\
{\
	unsigned long value;\
	ssize_t ret;\
	struct spi_device *spi = to_spi_device(dev);\
	struct mx4_spi_device* mx4 = dev_get_drvdata(&spi->dev);\
\
	if (kstrtoul (buf, 10, &value) < 0)\
	{\
		return -EINVAL;\
	}\
\
	mutex_lock (&mx4->lock);\
	ret = mx4_spi_write_value (mx4->spi, value, type);\
	mutex_unlock (&mx4->lock);\
\
	return (ret == SUCCESSFULL_MX4_RW) ? count : -EPERM;\
}


/* attribute macros */

#define def_read_integer_attr(name, type_code)\
def_read_integer_func_call (read_##name, type_code);\
static DEVICE_ATTR (name, S_IRUGO, read_##name, NULL);


#define def_write_integer_attr(name, type_code)\
def_write_integer_func_call (write_##name, type_code);\
static DEVICE_ATTR (name, S_IWUSR | S_IWGRP, NULL, write_##name);


#define def_read_write_integer_attr(name, type_code)\
def_read_integer_func_call (read_##name, type_code);\
def_write_integer_func_call (write_##name, type_code);\
static DEVICE_ATTR (name, S_IRUGO | S_IWUSR | S_IWGRP, read_##name, write_##name);

def_write_integer_attr(debug_k_line, PROT_TYPE_DEBUG_K_LINE);

def_read_integer_attr (analog_1, PROT_TYPE_INPUT_ANALOG_1);
def_read_integer_attr (analog_2, PROT_TYPE_INPUT_ANALOG_2);
def_read_integer_attr (analog_3, PROT_TYPE_INPUT_ANALOG_3);
def_read_integer_attr (analog_4, PROT_TYPE_INPUT_ANALOG_4);
def_read_integer_attr (analog_5, PROT_TYPE_INPUT_ANALOG_5);

def_read_write_integer_attr (analog_out_1, PROT_TYPE_ANALOG_OUT_1);
def_read_write_integer_attr (analog_out_2, PROT_TYPE_ANALOG_OUT_2);
def_read_write_integer_attr (analog_out_3, PROT_TYPE_ANALOG_OUT_3);
def_read_write_integer_attr (analog_out_4, PROT_TYPE_ANALOG_OUT_4);

def_read_integer_attr (input_voltage, PROT_TYPE_INPUT_VOLTAGE);
def_read_integer_attr (input_voltage_2, PROT_TYPE_INPUT_VOLTAGE_2);
def_read_integer_attr (input_battery, PROT_TYPE_INPUT_BATTERY);
def_read_integer_attr (input_temperature_mC, PROT_TYPE_INPUT_TEMPERATURE);
def_read_integer_attr (start_signal, PROT_TYPE_INPUT_START_SIGNAL);
def_read_integer_attr (super_cap, PROT_TYPE_INPUT_SUPER_CAP);
def_read_integer_attr (fuse, PROT_TYPE_INPUT_FUSE);

def_read_write_integer_attr (analog_1_calibration_u,
	PROT_TYPE_CALIBRATION_ANALOG_1);
def_read_write_integer_attr (analog_2_calibration_u,
	PROT_TYPE_CALIBRATION_ANALOG_2);
def_read_write_integer_attr (analog_3_calibration_u,
	PROT_TYPE_CALIBRATION_ANALOG_3);
def_read_write_integer_attr (analog_4_calibration_u,
	PROT_TYPE_CALIBRATION_ANALOG_4);
def_read_write_integer_attr (analog_5_calibration_u,
	PROT_TYPE_CALIBRATION_ANALOG_5);

def_read_write_integer_attr (analog_out_1_calibration_u,
	PROT_TYPE_CALIBRATION_OUT_1);
def_read_write_integer_attr (analog_out_2_calibration_u,
	PROT_TYPE_CALIBRATION_OUT_2);
def_read_write_integer_attr (analog_out_3_calibration_u,
	PROT_TYPE_CALIBRATION_OUT_3);
def_read_write_integer_attr (analog_out_4_calibration_u,
	PROT_TYPE_CALIBRATION_OUT_4);


def_read_write_integer_attr (legacy_input_voltage_a_calibration_u,
	PROT_TYPE_LEGACY_INPUT_VOLTAGE_CALIBRATION_A);
def_read_write_integer_attr (legacy_input_voltage_b_calibration_u,
	PROT_TYPE_LEGACY_INPUT_VOLTAGE_CALIBRATION_B);

def_read_write_integer_attr (input_voltage_calibration_u,
	PROT_TYPE_INPUT_VOLTAGE_CALIBRATION);
def_read_write_integer_attr (input_voltage_2_calibration_u,
	PROT_TYPE_INPUT_VOLTAGE_2_CALIBRATION);
def_read_write_integer_attr (input_temperature_calibration_u,
	PROT_TYPE_TEMP_CALIBRATION);
def_read_write_integer_attr (input_battery_calibration_u,
	PROT_TYPE_INPUT_BATTERY_CALIBRATION);
def_read_write_integer_attr (start_signal_calibration_u,
	PROT_TYPE_CALIBRATION_START_SIGNAL);
def_read_write_integer_attr (super_cap_calibration_u,
	PROT_TYPE_CALIBRATION_SUPER_CAP);
def_read_write_integer_attr (fuse_calibration_u,
	PROT_TYPE_CALIBRATION_FUSE);

def_read_write_integer_attr (analog_1_offset, PROT_TYPE_OFFSET_ANALOG1);
def_read_write_integer_attr (analog_2_offset, PROT_TYPE_OFFSET_ANALOG2);
def_read_write_integer_attr (analog_3_offset, PROT_TYPE_OFFSET_ANALOG3);
def_read_write_integer_attr (analog_4_offset, PROT_TYPE_OFFSET_ANALOG4);
def_read_write_integer_attr (analog_5_offset, PROT_TYPE_OFFSET_ANALOG5);

def_read_write_integer_attr (analog_out_1_offset, PROT_TYPE_OFFSET_OUT_1);
def_read_write_integer_attr (analog_out_2_offset, PROT_TYPE_OFFSET_OUT_2);
def_read_write_integer_attr (analog_out_3_offset, PROT_TYPE_OFFSET_OUT_3);
def_read_write_integer_attr (analog_out_4_offset, PROT_TYPE_OFFSET_OUT_4);

def_read_write_integer_attr (start_signal_offset,
	PROT_TYPE_OFFSET_START_SIGNAL);
def_read_write_integer_attr (input_voltage_offset,
	PROT_TYPE_INPUT_VOLTAGE_OFFSET);
def_read_write_integer_attr (input_voltage_2_offset,
	PROT_TYPE_INPUT_VOLTAGE_OFFSET_2);
def_read_write_integer_attr (input_battery_offset,
	PROT_TYPE_INPUT_BATTERY_OFFSET);
def_read_write_integer_attr (super_cap_offset, PROT_TYPE_SUPER_CAP_OFFSET);
def_read_write_integer_attr (fuse_offset, PROT_TYPE_FUSE_OFFSET);

def_read_write_integer_attr (led_2, PROT_TYPE_LED_2);
def_read_write_integer_attr (led_3, PROT_TYPE_LED_3);
def_read_write_integer_attr (led_4, PROT_TYPE_LED_4);
def_read_write_integer_attr (buzzer, PROT_TYPE_BUZZER);

def_read_write_integer_attr (c3_counter, PROT_TYPE_C3_COUNTER);

def_read_write_integer_attr(pin_select_can2_h, PROT_TYPE_PS_CAN2H);
def_read_write_integer_attr(pin_select_can2_l, PROT_TYPE_PS_CAN2L);
def_read_write_integer_attr(pin_select_can3_h, PROT_TYPE_PS_CAN3H);
def_read_write_integer_attr(pin_select_can3_l, PROT_TYPE_PS_CAN3L);
def_read_write_integer_attr(pin_select_k_line, PROT_TYPE_PS_K_LINE);
def_read_write_integer_attr(pin_select_l_line, PROT_TYPE_PS_L_LINE);
def_read_write_integer_attr(pin_select_register, PROT_TYPE_PS_REGISTER);

def_read_write_integer_attr (ctrl_wakeup_sources_register,
	PROT_TYPE_WAKEUP_SOURCE_REGISTER);
def_read_write_integer_attr (ctrl_gpio_suspend_state,
	PROT_TYPE_DIGITAL_OUTPUT_SUSPEND_STATE);

def_read_write_integer_attr (input_voltage_threshold_high,
	PROT_TYPE_INPUT_VOLTAGE_THRESHOLD_HIGH);
def_read_write_integer_attr (input_voltage_threshold_low,
	PROT_TYPE_INPUT_VOLTAGE_THRESHOLD_LOW);
def_read_write_integer_attr (input_battery_threshold_high,
	PROT_TYPE_INPUT_BATTERY_THRESHOLD_HIGH);
def_read_write_integer_attr (input_battery_threshold_low,
	PROT_TYPE_INPUT_BATTERY_THRESHOLD_LOW);
def_read_write_integer_attr (analog_1_threshold_high,
	PROT_TYPE_INPUT_ANALOG1_THRESHOLD_HIGH);
def_read_write_integer_attr (analog_1_threshold_low,
	PROT_TYPE_INPUT_ANALOG1_THRESHOLD_LOW);
def_read_write_integer_attr (analog_2_threshold_high,
	PROT_TYPE_INPUT_ANALOG2_THRESHOLD_HIGH);
def_read_write_integer_attr (analog_2_threshold_low,
	PROT_TYPE_INPUT_ANALOG2_THRESHOLD_LOW);
def_read_write_integer_attr (analog_3_threshold_high,
	PROT_TYPE_INPUT_ANALOG3_THRESHOLD_HIGH);
def_read_write_integer_attr (analog_3_threshold_low,
	PROT_TYPE_INPUT_ANALOG3_THRESHOLD_LOW);
def_read_write_integer_attr (analog_4_threshold_high,
	PROT_TYPE_INPUT_ANALOG4_THRESHOLD_HIGH);
def_read_write_integer_attr (analog_4_threshold_low,
	PROT_TYPE_INPUT_ANALOG4_THRESHOLD_LOW);
def_read_write_integer_attr (analog_5_threshold_high,
	PROT_TYPE_INPUT_ANALOG5_THRESHOLD_HIGH);
def_read_write_integer_attr (analog_5_threshold_low,
	PROT_TYPE_INPUT_ANALOG5_THRESHOLD_LOW);
def_read_write_integer_attr (start_signal_threshold_high,
	PROT_TYPE_INPUT_START_SIGNAL_THRESHOLD_HIGH);
def_read_write_integer_attr (start_signal_threshold_low,
	PROT_TYPE_INPUT_START_SIGNAL_THRESHOLD_LOW);
def_read_write_integer_attr (fuse_threshold_high,
	PROT_TYPE_INPUT_FUSE_THRESHOLD_HIGH);
def_read_write_integer_attr (fuse_threshold_low,
	PROT_TYPE_INPUT_FUSE_THRESHOLD_LOW);

def_write_integer_attr (ctrl_goto_sleep, PROT_TYPE_CTRL_GO_TO_SLEEP);

def_read_integer_attr (ctrl_version, PROT_TYPE_CTRL_VERSION);
def_read_integer_attr (ctrl_bl_version, PROT_TYPE_CTRL_BL_VERSION);
def_read_integer_attr (ctrl_wakeup_cause, PROT_TYPE_WAKEUP_CAUSE);
def_read_write_integer_attr(ctrl_system_state, PROT_TYPE_CTRL_SYSTEM_STATE);
def_read_write_integer_attr(ctrl_power_state, PROT_TYPE_CTRL_POWER_STATE);
def_read_write_integer_attr(ctrl_led_power_off, PROT_TYPE_CTRL_LED_POWER_OFF);
def_read_write_integer_attr (ctrl_pic_reset_cause, PROT_TYPE_PIC_RESET_CAUSE);

def_read_write_integer_attr (ctrl_modem_on, PROT_TYPE_MODEM_ON);
def_read_integer_attr (ctrl_modem_status, PROT_TYPE_MODEM_STATUS);
def_read_write_integer_attr (ctrl_modem_ign, PROT_TYPE_MODEM_IGN);
def_read_write_integer_attr (ctrl_modem_emg, PROT_TYPE_MODEM_EMG);

def_read_write_integer_attr (ctrl_j1708_enable, PROT_TYPE_CTRL_J1708_ENABLE);
def_read_write_integer_attr (ctrl_rs485_enable, PROT_TYPE_CTRL_RS485_ENABLE);
def_read_write_integer_attr (ctrl_rs422_mode, PROT_TYPE_CTRL_RS422_MODE);
def_read_write_integer_attr (ctrl_rs422_loopback, PROT_TYPE_CTRL_RS422_LOOPBACK);

def_read_write_integer_attr (ctrl_serial_nr, PROT_TYPE_CTRL_SERIAL);
def_read_write_integer_attr (ctrl_hw_rev, PROT_TYPE_CTRL_HW_REV);
def_read_write_integer_attr (ctrl_product_id, PROT_TYPE_CTRL_PRODUCT_ID);
def_read_write_integer_attr(trap_location, PROT_TYPE_TRAP_LOCATION);
def_read_write_integer_attr(trap_location_prev, PROT_TYPE_TRAP_LOCATION_PREV);
def_read_write_integer_attr(trap_iec_trace, PROT_TYPE_TRAP_IEC_TRACE);
def_read_write_integer_attr(trap_ifs_trace, PROT_TYPE_TRAP_IFS_TRACE);

def_read_write_integer_attr(ctrl_on_4v, PROT_TYPE_CTRL_ON_4V);
def_read_write_integer_attr(ctrl_on_battery, PROT_TYPE_CTRL_ON_BATTERY);
def_read_write_integer_attr(usb_port_otg, PROT_TYPE_USB_OTG);
def_read_write_integer_attr (rtc_year, PROT_TYPE_RTC_YEAR);
def_read_write_integer_attr (rtc_month, PROT_TYPE_RTC_MONTH);
def_read_write_integer_attr (rtc_day, PROT_TYPE_RTC_DAY);
def_read_write_integer_attr (rtc_dow, PROT_TYPE_RTC_DOW);
def_read_write_integer_attr (rtc_hour, PROT_TYPE_RTC_HOURS);
def_read_write_integer_attr (rtc_minute, PROT_TYPE_RTC_MINUTES);
def_read_write_integer_attr (rtc_second, PROT_TYPE_RTC_SECONDS);

#define MX4_CONCAT_IMPL(a, b) a##b
#define MX4_CONCAT(a, b) MX4_CONCAT_IMPL(a, b)

static struct attribute *mx4_attributes[] = {
	&dev_attr_debug_k_line.attr,
	&dev_attr_analog_1.attr,
	&dev_attr_analog_2.attr,
	&dev_attr_analog_3.attr,
	&dev_attr_analog_4.attr,
	&dev_attr_analog_5.attr,
	&dev_attr_analog_out_1.attr,
	&dev_attr_analog_out_2.attr,
	&dev_attr_analog_out_3.attr,
	&dev_attr_analog_out_4.attr,
	&dev_attr_input_temperature_mC.attr,
	&dev_attr_input_voltage.attr,
	&dev_attr_input_voltage_2.attr,
	&dev_attr_input_battery.attr,
	&dev_attr_start_signal.attr,
	&dev_attr_super_cap.attr,
	&dev_attr_fuse.attr,

	&dev_attr_legacy_input_voltage_a_calibration_u.attr,
	&dev_attr_legacy_input_voltage_b_calibration_u.attr,

	&dev_attr_analog_1_calibration_u.attr,
	&dev_attr_analog_2_calibration_u.attr,
	&dev_attr_analog_3_calibration_u.attr,
	&dev_attr_analog_4_calibration_u.attr,
	&dev_attr_analog_5_calibration_u.attr,
	&dev_attr_analog_out_1_calibration_u.attr,
	&dev_attr_analog_out_2_calibration_u.attr,
	&dev_attr_analog_out_3_calibration_u.attr,
	&dev_attr_analog_out_4_calibration_u.attr,
	&dev_attr_input_temperature_calibration_u.attr,
	&dev_attr_input_voltage_calibration_u.attr,
	&dev_attr_input_voltage_2_calibration_u.attr,
	&dev_attr_input_battery_calibration_u.attr,
	&dev_attr_start_signal_calibration_u.attr,
	&dev_attr_super_cap_calibration_u.attr,
	&dev_attr_fuse_calibration_u.attr,

	&dev_attr_analog_1_offset.attr,
	&dev_attr_analog_2_offset.attr,
	&dev_attr_analog_3_offset.attr,
	&dev_attr_analog_4_offset.attr,
	&dev_attr_analog_5_offset.attr,
	&dev_attr_analog_out_1_offset.attr,
	&dev_attr_analog_out_2_offset.attr,
	&dev_attr_analog_out_3_offset.attr,
	&dev_attr_analog_out_4_offset.attr,
	&dev_attr_input_voltage_offset.attr,
	&dev_attr_input_voltage_2_offset.attr,
	&dev_attr_start_signal_offset.attr,
	&dev_attr_input_battery_offset.attr,
	&dev_attr_super_cap_offset.attr,
	&dev_attr_fuse_offset.attr,

	&dev_attr_led_2.attr,
	&dev_attr_led_3.attr,
	&dev_attr_led_4.attr,

	&dev_attr_buzzer.attr,
	&dev_attr_c3_counter.attr,

	&dev_attr_ctrl_wakeup_sources_register.attr,

	&dev_attr_input_voltage_threshold_high.attr,
	&dev_attr_input_voltage_threshold_low.attr,
	&dev_attr_input_battery_threshold_high.attr,
	&dev_attr_input_battery_threshold_low.attr,
	&dev_attr_analog_1_threshold_high.attr,
	&dev_attr_analog_1_threshold_low.attr,
	&dev_attr_analog_2_threshold_high.attr,
	&dev_attr_analog_2_threshold_low.attr,
	&dev_attr_analog_3_threshold_high.attr,
	&dev_attr_analog_3_threshold_low.attr,
	&dev_attr_analog_4_threshold_high.attr,
	&dev_attr_analog_4_threshold_low.attr,
	&dev_attr_analog_5_threshold_high.attr,
	&dev_attr_analog_5_threshold_low.attr,
	&dev_attr_start_signal_threshold_high.attr,
	&dev_attr_start_signal_threshold_low.attr,
	&dev_attr_fuse_threshold_high.attr,
	&dev_attr_fuse_threshold_low.attr,

	&dev_attr_ctrl_gpio_suspend_state.attr,
	&dev_attr_ctrl_goto_sleep.attr,
	&dev_attr_ctrl_wakeup_cause.attr,
	&dev_attr_ctrl_pic_reset_cause.attr,
	&dev_attr_ctrl_modem_on.attr,
	&dev_attr_ctrl_modem_status.attr,
	&dev_attr_ctrl_modem_ign.attr,
	&dev_attr_ctrl_modem_emg.attr,
	&dev_attr_ctrl_j1708_enable.attr,
	&dev_attr_ctrl_rs485_enable.attr,
	&dev_attr_ctrl_rs422_mode.attr,
	&dev_attr_ctrl_rs422_loopback.attr,
	&dev_attr_ctrl_version.attr,
	&dev_attr_ctrl_bl_version.attr,
	&dev_attr_ctrl_system_state.attr,
	&dev_attr_ctrl_power_state.attr,
	&dev_attr_ctrl_led_power_off.attr,

	&dev_attr_ctrl_serial_nr.attr,
	&dev_attr_ctrl_hw_rev.attr,
	&dev_attr_ctrl_product_id.attr,

	&dev_attr_trap_location.attr,
	&dev_attr_trap_location_prev.attr,
	&dev_attr_trap_iec_trace.attr,
	&dev_attr_trap_ifs_trace.attr,

	&dev_attr_pin_select_can2_h.attr,
	&dev_attr_pin_select_can2_l.attr,
	&dev_attr_pin_select_can3_h.attr,
	&dev_attr_pin_select_can3_l.attr,
	&dev_attr_pin_select_k_line.attr,
	&dev_attr_pin_select_l_line.attr,
	&dev_attr_pin_select_register.attr,

	&dev_attr_ctrl_on_4v.attr,
	&dev_attr_ctrl_on_battery.attr,
	&dev_attr_usb_port_otg.attr,
	&dev_attr_rtc_year.attr,
	&dev_attr_rtc_month.attr,
	&dev_attr_rtc_day.attr,
	&dev_attr_rtc_dow.attr,
	&dev_attr_rtc_hour.attr,
	&dev_attr_rtc_minute.attr,
	&dev_attr_rtc_second.attr,
		//functions
	&dev_attr_drv_remove_exported_gpios.attr,

		//bootloader
	&MX4_CONCAT (dev_attr_, START_BOOTLOADER_ATTRIBUTE_NAME).attr,
	NULL
};

static const struct attribute_group mx4_attr_group =
{
	.attrs = mx4_attributes,
};

const struct attribute_group *mx4_attributes_get(void)
{
	return &mx4_attr_group;
}
