/*
 * Copyright (C) 2017 Host Mobility AB. All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef MX4_SPI_PROTOCOL_DEFS_H_
#define MX4_SPI_PROTOCOL_DEFS_H_

#ifdef MX4_LINUX_KERNEL_SPACE

    #include <asm/types.h>
    typedef u8 mx4_u8_t;
    typedef u32 mx4_u32_t;

#else //the microcontroller side

    #include <stdint.h>
    typedef uint8_t mx4_u8_t;
    typedef uint32_t mx4_u32_t;

#endif

/* Frame field types*/
typedef mx4_u8_t mx4_spi_service_primitive_t;
typedef mx4_u8_t mx4_spi_related_value_t;
typedef mx4_u8_t mx4_spi_status_field_t;
typedef mx4_u32_t mx4_spi_data_field_t;
typedef mx4_u8_t mx4_spi_checksum_t;

/* Checksum calc*/
static inline mx4_spi_checksum_t mx4_checksum_accumulate (const void* buffer, unsigned length)
{
    mx4_spi_checksum_t result = 0;
    const mx4_spi_checksum_t* buff = (const mx4_spi_checksum_t*) buffer;
    unsigned i;

    for (i = 0; i < length; ++i, ++buff) {	result += *buff; }

    return result;
}

static inline mx4_spi_checksum_t mx4_calculate_checksum (const void* buffer, unsigned length)
{
    return 0 - mx4_checksum_accumulate (buffer, length);
}

static inline mx4_spi_checksum_t mx4_checksum_fail (const void* buffer, unsigned length, mx4_spi_checksum_t checksum)
{
    return checksum + mx4_checksum_accumulate (buffer, length);
}


static const int mx4_spi_data_field_char_digits = 11; /*an integer string is always maximum 11 chars long without null.*/

/* Frame positioning*/


typedef enum spi_read_request_frame_e
{
    MX4_SPI_READ_REQUEST_SERVICE_PRIMITIVE_OFFSET,
    MX4_SPI_READ_REQUEST_RELATED_VALUE_OFFSET = MX4_SPI_READ_REQUEST_SERVICE_PRIMITIVE_OFFSET + sizeof (mx4_spi_service_primitive_t),
    MX4_SPI_READ_REQUEST_SIZE = MX4_SPI_READ_REQUEST_RELATED_VALUE_OFFSET + sizeof (mx4_spi_related_value_t),
}
spi_read_request_frame;

typedef enum spi_read_response_frame_e
{
    MX4_SPI_READ_RESPONSE_SERVICE_PRIMITIVE_OFFSET,
    MX4_SPI_READ_RESPONSE_RELATED_VALUE_OFFSET = MX4_SPI_READ_RESPONSE_SERVICE_PRIMITIVE_OFFSET + sizeof (mx4_spi_service_primitive_t),
    MX4_SPI_READ_RESPONSE_STATUS_OFFSET = MX4_SPI_READ_RESPONSE_RELATED_VALUE_OFFSET + sizeof (mx4_spi_related_value_t),
    MX4_SPI_READ_RESPONSE_DATA_OFFSET = MX4_SPI_READ_RESPONSE_STATUS_OFFSET + sizeof (mx4_spi_status_field_t),
    MX4_SPI_READ_RESPONSE_CHECKSUM_OFFSET = MX4_SPI_READ_RESPONSE_DATA_OFFSET + sizeof (mx4_spi_data_field_t),
    MX4_SPI_READ_RESPONSE_SIZE = MX4_SPI_READ_RESPONSE_CHECKSUM_OFFSET + sizeof (mx4_spi_checksum_t),

    MX4_SPI_READ_RESPONSE_CHECKSUM_FIRST_BYTE_OFFSET = MX4_SPI_READ_RESPONSE_STATUS_OFFSET,
    MX4_SPI_READ_RESPONSE_CHECKSUM_BYTE_COUNT = MX4_SPI_READ_RESPONSE_CHECKSUM_OFFSET - MX4_SPI_READ_RESPONSE_CHECKSUM_FIRST_BYTE_OFFSET,
}
spi_read_response_frame;

typedef enum spi_write_request_frame_e
{
    MX4_SPI_WRITE_REQUEST_SERVICE_PRIMITIVE_OFFSET,
    MX4_SPI_WRITE_REQUEST_RELATED_VALUE_OFFSET = MX4_SPI_WRITE_REQUEST_SERVICE_PRIMITIVE_OFFSET + sizeof (mx4_spi_service_primitive_t),
    MX4_SPI_WRITE_REQUEST_DATA_OFFSET = MX4_SPI_WRITE_REQUEST_RELATED_VALUE_OFFSET + sizeof (mx4_spi_related_value_t),
    MX4_SPI_WRITE_REQUEST_CHECKSUM_OFFSET = MX4_SPI_WRITE_REQUEST_DATA_OFFSET + sizeof (mx4_spi_data_field_t),
    MX4_SPI_WRITE_REQUEST_SIZE = MX4_SPI_WRITE_REQUEST_CHECKSUM_OFFSET + sizeof (mx4_spi_checksum_t),

    MX4_SPI_WRITE_REQUEST_CHECKSUM_FIRST_BYTE_OFFSET = MX4_SPI_WRITE_REQUEST_DATA_OFFSET,
    MX4_SPI_WRITE_REQUEST_CHECKSUM_BYTE_COUNT = MX4_SPI_WRITE_REQUEST_CHECKSUM_OFFSET - MX4_SPI_WRITE_REQUEST_CHECKSUM_FIRST_BYTE_OFFSET,
}
spi_write_request_frame;

typedef enum spi_write_response_frame_e
{
    MX4_SPI_WRITE_RESPONSE_SERVICE_PRIMITIVE_OFFSET,
    MX4_SPI_WRITE_RESPONSE_RELATED_VALUE_OFFSET = MX4_SPI_WRITE_RESPONSE_SERVICE_PRIMITIVE_OFFSET + sizeof (mx4_spi_service_primitive_t),
    MX4_SPI_WRITE_RESPONSE_STATUS_OFFSET = MX4_SPI_WRITE_RESPONSE_RELATED_VALUE_OFFSET + sizeof (mx4_spi_related_value_t),
    MX4_SPI_WRITE_RESPONSE_SIZE = MX4_SPI_WRITE_RESPONSE_STATUS_OFFSET + sizeof (mx4_spi_status_field_t)
}
spi_write_response_frame;

typedef enum mx4_spi_service_primitive_e
{
    MX4_SPI_READ_REQUEST = 1,
    MX4_SPI_WRITE_REQUEST,
    MX4_SPI_READ_RESPONSE = 0x81,
    MX4_SPI_WRITE_RESPONSE
}
mx4_spi_service_primitive;

typedef enum mx4_spi_status_e
{
    MX4_SPI_OK = 1,
    MX4_SPI_UNKNOWN_COMMAND,
    MX4_SPI_UNKNOWN_TYPE,
    MX4_SPI_BAD_CHECKSUM,
    MX4_SPI_INVALID_DATA,
}
mx4_spi_status;

#define MX4_SPI_SYNC_BYTE 0xff

static const char mx4_spi_sync_frame [] =
{
    MX4_SPI_SYNC_BYTE, MX4_SPI_SYNC_BYTE, MX4_SPI_SYNC_BYTE, MX4_SPI_SYNC_BYTE, MX4_SPI_SYNC_BYTE
};

static const mx4_u32_t mx4_spi_sync_frame_size = sizeof (mx4_spi_sync_frame);

/* Data types*/
#define PROT_TYPE_SW_RESET                      0x01

#define PROT_TYPE_DEBUG_K_LINE                  0x02

#define PROT_TYPE_DIGITAL_OUT_7                 0x03
#define PROT_TYPE_DIGITAL_OUT_8                 0x04

#define PROT_TYPE_INPUT_ANALOG_1                0x10
#define PROT_TYPE_INPUT_ANALOG_2                0x11

#define PROT_TYPE_ANALOG_OUT_1                  0x14
#define PROT_TYPE_ANALOG_OUT_2                  0x15
#define PROT_TYPE_ANALOG_OUT_3                  0x16
#define PROT_TYPE_ANALOG_OUT_4                  0x17

#define PROT_TYPE_INPUT_ANALOG_3                0x20
#define PROT_TYPE_INPUT_ANALOG_4                0x21
#define PROT_TYPE_INPUT_ANALOG_5                0x22
#define PROT_TYPE_INPUT_TEMPERATURE             0x25
#define PROT_TYPE_INPUT_VOLTAGE                 0x26
#define PROT_TYPE_INPUT_BATTERY                 0x27
#define PROT_TYPE_INPUT_START_SIGNAL            0x28
#define PROT_TYPE_INPUT_SUPER_CAP               0x29
#define PROT_TYPE_INPUT_FUSE                    0x2a
#define PROT_TYPE_INPUT_VOLTAGE_2               0x2b

#define PROT_TYPE_DIGITAL_OUT_1                 0x30
#define PROT_TYPE_DIGITAL_OUT_2                 0x31
#define PROT_TYPE_DIGITAL_OUT_3                 0x32
#define PROT_TYPE_DIGITAL_OUT_4                 0x33
#define PROT_TYPE_OUT_5V_ON                     0x34
#define PROT_TYPE_DIGITAL_OUT_5                 0x35
#define PROT_TYPE_DIGITAL_OUT_6                 0x36

/* NOTE! Order here, because we added 7-8 later and had to use number that are
   available.
*/
#define PROT_TYPE_DIGITAL_IN_7                  0x37
#define PROT_TYPE_DIGITAL_IN_1                  0x38
#define PROT_TYPE_DIGITAL_IN_2                  0x39
#define PROT_TYPE_DIGITAL_IN_3                  0x3a
#define PROT_TYPE_DIGITAL_IN_4                  0x3b
#define PROT_TYPE_DIGITAL_IN_5                  0x3c
#define PROT_TYPE_DIGITAL_IN_6                  0x3d
#define PROT_TYPE_START_SW                      0x3e
#define PROT_TYPE_DIGITAL_IN_8                  0x3f

#define PROT_TYPE_LED_1                         0x40
#define PROT_TYPE_LED_2                         0x41
#define PROT_TYPE_LED_3                         0x42
#define PROT_TYPE_LED_4                         0x43
#define PROT_TYPE_BUZZER                        0x44
#define PROT_TYPE_MUX_OUT_1                     0x45
#define PROT_TYPE_MUX_OUT_2                     0x46
#define PROT_TYPE_MUX_IN_1                      0x47
#define PROT_TYPE_MUX_IN_2                      0x48
#define PROT_TYPE_LIN_ENABLED                   0x49
#define PROT_TYPE_LIN2_ENABLED                  0x4a

#define PROT_TYPE_POWER_ON_VREF                 0x51
#define PROT_TYPE_POWER_ON_5V                   0x52
#define PROT_TYPE_POWER_ON_3_3V                 0x53

#define PROT_TYPE_LIN_SLEEP                     0x54
#define PROT_TYPE_LIN2_SLEEP                    0x55
#define PROT_TYPE_INPUT_ANALOG3_THRESHOLD_HIGH  0x56
#define PROT_TYPE_INPUT_ANALOG3_THRESHOLD_LOW   0x57
#define PROT_TYPE_INPUT_ANALOG4_THRESHOLD_HIGH  0x58
#define PROT_TYPE_INPUT_ANALOG4_THRESHOLD_LOW   0x59
#define PROT_TYPE_INPUT_START_SIGNAL_THRESHOLD_HIGH 0x5a
#define PROT_TYPE_INPUT_START_SIGNAL_THRESHOLD_LOW  0x5b
#define PROT_TYPE_INPUT_FUSE_THRESHOLD_HIGH     0x5c
#define PROT_TYPE_INPUT_FUSE_THRESHOLD_LOW      0x5d
#define PROT_TYPE_INPUT_ANALOG5_THRESHOLD_HIGH  0x5e
#define PROT_TYPE_INPUT_ANALOG5_THRESHOLD_LOW   0x5f

#define PROT_TYPE_MODEM_SYNC                    0x60
#define PROT_TYPE_MODEM_RING                    0x61
#define PROT_TYPE_MODEM_CURRENT_IND             0x62
#define PROT_TYPE_MODEM_POWER_IND               0x63
#define PROT_TYPE_MODEM_ON                      0x64
#define PROT_TYPE_WAKE_ON_CAN                   0x65
#define PROT_TYPE_WAKE_ON_CAN_1                 0x66
#define PROT_TYPE_WAKE_ON_CAN_2                 0x67
#define PROT_TYPE_WAKE_ON_CAN_3                 0x68
#define PROT_TYPE_WAKE_ON_CAN_4                 0x69
#define PROT_TYPE_WAKE_ON_CAN_5                 0x6a
#define PROT_TYPE_WAKE_ON_CAN_6                 0x6b
#define PROT_TYPE_MODEM_STATUS                  0x6c
#define PROT_TYPE_MODEM_IGN                     0x6d
#define PROT_TYPE_MODEM_EMG                     0x6e

#define PROT_TYPE_C3_COUNTER                    0x70

#define PROT_TYPE_PS_K_LINE                     0x71
#define PROT_TYPE_PS_L_LINE                     0x72
#define PROT_TYPE_PS_CAN2H                      0x73
#define PROT_TYPE_PS_CAN2L                      0x74
#define PROT_TYPE_PS_REGISTER                   0x75
#define PROT_TYPE_PS_CAN3H                      0x76
#define PROT_TYPE_PS_CAN3L                      0x77

#define PROT_TYPE_WAKEUP_SOURCE_REGISTER        0x78
#define PROT_TYPE_INPUT_VOLTAGE_THRESHOLD_HIGH  0x79
#define PROT_TYPE_INPUT_VOLTAGE_THRESHOLD_LOW   0x80
#define PROT_TYPE_DIGITAL_OUTPUT_SUSPEND_STATE  0x81

#define PROT_TYPE_WAKEUP_CAUSE                  0x82
#define PROT_TYPE_PIC_RESET_CAUSE               0x83
#define PROT_TYPE_INPUT_BATTERY_THRESHOLD_HIGH  0x84
#define PROT_TYPE_INPUT_BATTERY_THRESHOLD_LOW   0x85
#define PROT_TYPE_INPUT_ANALOG1_THRESHOLD_HIGH  0x86
#define PROT_TYPE_INPUT_ANALOG1_THRESHOLD_LOW   0x87
#define PROT_TYPE_INPUT_ANALOG2_THRESHOLD_HIGH  0x88
#define PROT_TYPE_INPUT_ANALOG2_THRESHOLD_LOW   0x89

#define PROT_TYPE_CTRL_GO_TO_SLEEP		        0x90
#define PROT_TYPE_CTRL_ENTER_BL_MODE		    0x91
#define PROT_TYPE_CTRL_VERSION			        0x92
#define PROT_TYPE_CTRL_BL_VERSION		        0x93
#define PROT_TYPE_CTRL_POWER_STATE		        0x94
#define PROT_TYPE_CTRL_SYSTEM_STATE             0x95
#define PROT_TYPE_CTRL_LED_POWER_OFF            0x96
#define PROT_TYPE_CTRL_SERIAL                   0x97
#define PROT_TYPE_CTRL_HW_REV                   0x98
#define PROT_TYPE_CTRL_PRODUCT_ID               0x99
#define PROT_TYPE_CTRL_ON_4V                    0x9a
#define PROT_TYPE_CTRL_ON_BATTERY               0x9b
#define PROT_TYPE_CTRL_J1708_ENABLE             0x9c
#define PROT_TYPE_CTRL_RS485_ENABLE             0x9d
#define PROT_TYPE_CTRL_RS422_MODE               0x9e
#define PROT_TYPE_CTRL_RS422_LOOPBACK           0x9f

#define PROT_TYPE_TRAP_IEC_TRACE                0xa0
#define PROT_TYPE_TRAP_IFS_TRACE                0xa1
#define PROT_TYPE_TRAP_LOCATION                 0xa2
#define PROT_TYPE_TRAP_LOCATION_PREV            0xa3

#define PROT_TYPE_LEGACY_INPUT_VOLTAGE_CALIBRATION_A   0xaa
#define PROT_TYPE_LEGACY_INPUT_VOLTAGE_CALIBRATION_B   0xab

#define PROT_TYPE_USB_OTG                       0xb0
#define PROT_TYPE_RTC_YEAR                      0xb1
#define PROT_TYPE_RTC_MONTH                     0xb2
#define PROT_TYPE_RTC_DAY                       0xb3
#define PROT_TYPE_RTC_DOW                       0xb4
#define PROT_TYPE_RTC_HOURS                     0xb5
#define PROT_TYPE_RTC_MINUTES                   0xb6
#define PROT_TYPE_RTC_SECONDS                   0xb7

#define PROT_TYPE_OFFSET_ANALOG1                0xc0
#define PROT_TYPE_OFFSET_ANALOG2                0xc1
#define PROT_TYPE_OFFSET_ANALOG3                0xc2
#define PROT_TYPE_OFFSET_ANALOG4                0xc3
#define PROT_TYPE_OFFSET_OUT_1                  0xc4
#define PROT_TYPE_OFFSET_OUT_2                  0xc5
#define PROT_TYPE_OFFSET_OUT_3                  0xc6
#define PROT_TYPE_OFFSET_OUT_4                  0xc7
#define PROT_TYPE_OFFSET_START_SIGNAL           0xc8
#define PROT_TYPE_INPUT_VOLTAGE_OFFSET          0xc9
#define PROT_TYPE_INPUT_VOLTAGE_OFFSET_2        0xca
#define PROT_TYPE_INPUT_BATTERY_OFFSET          0xcb
#define PROT_TYPE_SUPER_CAP_OFFSET              0xcc
#define PROT_TYPE_FUSE_OFFSET                   0xcd
#define PROT_TYPE_OFFSET_ANALOG5                0xce

#define PROT_TYPE_CALIBRATION_ANALOG_1          0xe0
#define PROT_TYPE_CALIBRATION_ANALOG_2          0xe1
#define PROT_TYPE_CALIBRATION_ANALOG_3          0xe2
#define PROT_TYPE_CALIBRATION_ANALOG_4          0xe3
#define PROT_TYPE_TEMP_CALIBRATION              0xe4
#define PROT_TYPE_INPUT_VOLTAGE_CALIBRATION     0xe5
#define PROT_TYPE_INPUT_BATTERY_CALIBRATION     0xe6
#define PROT_TYPE_CALIBRATION_START_SIGNAL      0xe7
#define PROT_TYPE_CALIBRATION_SUPER_CAP         0xe8
#define PROT_TYPE_CALIBRATION_FUSE              0xe9
#define PROT_TYPE_INPUT_VOLTAGE_2_CALIBRATION   0xea
#define PROT_TYPE_CALIBRATION_OUT_1             0xeb
#define PROT_TYPE_CALIBRATION_OUT_2             0xec
#define PROT_TYPE_CALIBRATION_OUT_3             0xed
#define PROT_TYPE_CALIBRATION_OUT_4             0xee
#define PROT_TYPE_CALIBRATION_ANALOG_5          0xef


#define PROT_TYPE_EVENT_INPUTS                  0xf6
#define PROT_TYPE_POLLED                        0xf7
#define PROT_TYPE_INVALID                       0xff

//Generic bit access convenience macros
#define MX4_READ_INT_BIT(integer, index)\
	 ((integer >> index) & 1)

#define MX4_CLEAR_INT_BIT(integer, index)\
	(integer & (~(1 << index)))

#define MX4_WRITE_INT_BIT(integer, index, write_bit)\
	 integer =  MX4_CLEAR_INT_BIT(integer, index) | ((write_bit & 1) << index)

enum mx4_polled_inputs_ids
{
    digital_input_1_id,
    digital_input_2_id,
    digital_input_3_id,
    digital_input_4_id,
    digital_input_5_id,
    digital_input_6_id,
    modem_sync_id,
    modem_ring_id,
    modem_current_ind_id,
    modem_power_ind_id,
    start_switch_id,
    wake_on_can_id,
    mux_in_1_id,
    mux_in_2_id,
    digital_input_7_id,
    digital_input_8_id,
    mx4_polled_inputs_count
};

//Use this macros for accessing the periodically polled short circuit inputs bits instead of direct accessing.
//Read
#define MX4_READ_EVENT_DIGITAL_IN_1(polled_inputs)\
	MX4_READ_INT_BIT (polled_inputs, digital_input_1_id)

#define MX4_READ_EVENT_DIGITAL_IN_2(polled_inputs)\
	MX4_READ_INT_BIT (polled_inputs, digital_input_2_id)

#define MX4_READ_EVENT_DIGITAL_IN_3(polled_inputs)\
	MX4_READ_INT_BIT (polled_inputs, digital_input_3_id)

#define MX4_READ_EVENT_DIGITAL_IN_4(polled_inputs)\
	MX4_READ_INT_BIT (polled_inputs, digital_input_4_id)

#define MX4_READ_EVENT_DIGITAL_IN_5(polled_inputs)\
    MX4_READ_INT_BIT (polled_inputs, digital_input_5_id)

#define MX4_READ_EVENT_DIGITAL_IN_6(polled_inputs)\
    MX4_READ_INT_BIT (polled_inputs, digital_input_6_id)

#define MX4_READ_EVENT_MODEM_SYNC(polled_inputs)\
	MX4_READ_INT_BIT (polled_inputs, modem_sync_id)

#define MX4_READ_EVENT_MODEM_RING(polled_inputs)\
	MX4_READ_INT_BIT (polled_inputs, modem_ring_id)

#define MX4_READ_EVENT_MODEM_CURRENT_IND(polled_inputs)\
	MX4_READ_INT_BIT (polled_inputs, modem_current_ind_id)

#define MX4_READ_EVENT_MODEM_POWER_IND(polled_inputs)\
	MX4_READ_INT_BIT (polled_inputs, modem_power_ind_id)

#define MX4_READ_EVENT_START_SWITCH(polled_inputs)\
    MX4_READ_INT_BIT (polled_inputs, start_switch_id)

#define MX4_READ_EVENT_WAKE_ON_CAN(polled_inputs)\
    MX4_READ_INT_BIT (polled_inputs, wake_on_can_id)

#define MX4_READ_EVENT_MUX_IN_1(polled_inputs)\
    MX4_READ_INT_BIT (polled_inputs, mux_in_1_id)

#define MX4_READ_EVENT_MUX_IN_2(polled_inputs)\
    MX4_READ_INT_BIT (polled_inputs, mux_in_2_id)

#define MX4_READ_EVENT_DIGITAL_IN_7(polled_inputs)\
    MX4_READ_INT_BIT (polled_inputs, digital_input_7_id)

#define MX4_READ_EVENT_DIGITAL_IN_8(polled_inputs)\
    MX4_READ_INT_BIT (polled_inputs, digital_input_8_id)

//Write
#define MX4_WRITE_EVENT_DIGITAL_IN_1(polled_inputs, bool_value)\
	MX4_WRITE_INT_BIT (polled_inputs, digital_input_1_id, bool_value)

#define MX4_WRITE_EVENT_DIGITAL_IN_2(polled_inputs, bool_value)\
	MX4_WRITE_INT_BIT (polled_inputs, digital_input_2_id, bool_value)

#define MX4_WRITE_EVENT_DIGITAL_IN_3(polled_inputs, bool_value)\
	MX4_WRITE_INT_BIT (polled_inputs, digital_input_3_id, bool_value)

#define MX4_WRITE_EVENT_DIGITAL_IN_4(polled_inputs, bool_value)\
	MX4_WRITE_INT_BIT (polled_inputs, digital_input_4_id, bool_value)

#define MX4_WRITE_EVENT_DIGITAL_IN_5(polled_inputs, bool_value)\
    MX4_WRITE_INT_BIT (polled_inputs, digital_input_5_id, bool_value)

#define MX4_WRITE_EVENT_DIGITAL_IN_6(polled_inputs, bool_value)\
    MX4_WRITE_INT_BIT (polled_inputs, digital_input_6_id, bool_value)

#define MX4_WRITE_EVENT_MODEM_SYNC(polled_inputs, bool_value)\
	MX4_WRITE_INT_BIT (polled_inputs, modem_sync_id, bool_value)

#define MX4_WRITE_EVENT_MODEM_RING(polled_inputs, bool_value)\
	MX4_WRITE_INT_BIT (polled_inputs, modem_ring_id, bool_value)

#define MX4_WRITE_EVENT_MODEM_CURRENT_IND(polled_inputs, bool_value)\
	MX4_WRITE_INT_BIT (polled_inputs, modem_current_ind_id, bool_value)

#define MX4_WRITE_EVENT_MODEM_POWER_IND(polled_inputs, bool_value)\
	MX4_WRITE_INT_BIT (polled_inputs, modem_power_ind_id, bool_value)

#define MX4_WRITE_EVENT_START_SWITCH(polled_inputs, bool_value)\
    MX4_WRITE_INT_BIT (polled_inputs, start_switch_id, bool_value)

#define MX4_WRITE_EVENT_WAKE_ON_CAN(polled_inputs, bool_value)\
    MX4_WRITE_INT_BIT (polled_inputs, wake_on_can_id, bool_value)

#define MX4_WRITE_EVENT_MUX_IN_1(polled_inputs, bool_value)\
    MX4_WRITE_INT_BIT (polled_inputs, mux_in_1_id, bool_value)

#define MX4_WRITE_EVENT_MUX_IN_2(polled_inputs, bool_value)\
    MX4_WRITE_INT_BIT (polled_inputs, mux_in_2_id, bool_value)

#define MX4_WRITE_EVENT_DIGITAL_IN_7(polled_inputs, bool_value)\
    MX4_WRITE_INT_BIT (polled_inputs, digital_input_7_id, bool_value)

#define MX4_WRITE_EVENT_DIGITAL_IN_8(polled_inputs, bool_value)\
    MX4_WRITE_INT_BIT (polled_inputs, digital_input_8_id, bool_value)

typedef struct sReadReplyMsg {
    mx4_spi_status_field_t status;
    mx4_spi_data_field_t data;
    //mx4_spi_checksum_t checksum; //
} read_reply_msg_t;

typedef struct sWriteMsg {
    mx4_spi_data_field_t data;
    mx4_spi_checksum_t checksum_fail;
} write_msg_t;

typedef struct sWriteReplyMsg {
    mx4_spi_status_field_t status;
} write_reply_msg_t;

typedef struct sComboMessage{
    read_reply_msg_t read_reply_msg;
    write_msg_t write_msg;
    write_reply_msg_t write_reply_msg;
} combo_message_t;

typedef struct sMessage {
//mx4_spi_msg_length_t lom;
    mx4_spi_service_primitive_t cmd;
    mx4_spi_related_value_t type;

    combo_message_t msg;
} Message;

#endif	/* PROTOCOL_DEFS_H */


