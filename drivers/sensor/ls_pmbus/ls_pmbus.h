#include <zephyr/device.h>
#include <zephyr/drivers/smbus.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

/* PMBus Command Definitions */
#define PMBUS_CMD_PAGE                       0x00
#define PMBUS_CMD_OPERATION                  0x01
#define PMBUS_CMD_ON_OFF_CONFIG              0x02
#define PMBUS_CMD_CLEAR_FAULTS               0x03
#define PMBUS_CMD_PHASE                      0x04
#define PMBUS_CMD_PAGE_PLUS_WRITE            0x05
#define PMBUS_CMD_PAGE_PLUS_READ             0x06
#define PMBUS_CMD_WRITE_PROTECT              0x10
#define PMBUS_CMD_STORE_USER_ALL             0x15
#define PMBUS_CMD_RESTORE_USER_ALL           0x16
#define PMBUS_CMD_CAPABILITY                 0x19
#define PMBUS_CMD_QUERY                      0x1A
#define PMBUS_CMD_SMBALERT_MASK              0x1B
#define PMBUS_CMD_VOUT_MODE			         0x20
#define PMBUS_CMD_VOUT_COMMAND		         0x21
#define PMBUS_CMD_VOUT_TRIM			         0x22
#define PMBUS_CMD_VOUT_CAL_OFFSET		     0x23
#define PMBUS_CMD_VOUT_MAX			         0x24
#define PMBUS_CMD_VOUT_MARGIN_HIGH		     0x25
#define PMBUS_CMD_VOUT_MARGIN_LOW		     0x26
#define PMBUS_CMD_VOUT_TRANSITION_RATE	     0x27
#define PMBUS_CMD_VOUT_DROOP		         0x28
#define PMBUS_CMD_VOUT_SCALE_LOOP		     0x29
#define PMBUS_CMD_VOUT_SCALE_MONITOR		 0x2A
#define PMBUS_CMD_COEFFICIENTS		         0x30
#define PMBUS_CMD_POUT_MAX			         0x31
#define PMBUS_CMD_IOUT_CAL_GAIN              0x38
#define PMBUS_CMD_IOUT_CAL_OFFSET            0x39
#define PMBUS_CMD_FAN_CONFIG_12			     0x3A
#define PMBUS_CMD_FAN_COMMAND_1		    	 0x3B
#define PMBUS_CMD_FAN_COMMAND_2		    	 0x3C
#define PMBUS_CMD_FAN_CONFIG_34			     0x3D
#define PMBUS_CMD_FAN_COMMAND_3		    	 0x3E
#define PMBUS_CMD_FAN_COMMAND_4			     0x3F
#define PMBUS_CMD_VOUT_OV_FAULT_LIMIT        0x40
#define PMBUS_CMD_VOUT_OV_FAULT_RESPONSE     0x41
#define PMBUS_CMD_VOUT_OV_WARN_LIMIT         0x42
#define PMBUS_CMD_VOUT_UV_WARN_LIMIT         0x43
#define PMBUS_CMD_VOUT_UV_FAULT_LIMIT        0x44
#define PMBUS_CMD_VOUT_UV_FAULT_RESPONSE     0x45
#define PMBUS_CMD_IOUT_OC_FAULT_LIMIT        0x46
#define PMBUS_CMD_IOUT_OC_FAULT_RESPONSE     0x47
#define PMBUS_CMD_IOUT_OC_LV_FAULT_LIMIT     0x48
#define PMBUS_CMD_IOUT_OC_LV_FAULT_RESPONSE  0x49
#define PMBUS_CMD_IOUT_OC_WARN_LIMIT         0x4A
#define PMBUS_CMD_IOUT_UC_FAULT_LIMIT        0x4B
#define PMBUS_CMD_IOUT_UC_FAULT_RESPONSE     0x4C
#define PMBUS_CMD_OT_FAULT_LIMIT             0x4F
#define PMBUS_CMD_OT_FAULT_RESPONSE          0x50
#define PMBUS_CMD_OT_WARN_LIMIT              0x51
#define PMBUS_CMD_UT_WARN_LIMIT              0x52
#define PMBUS_CMD_UT_FAULT_LIMIT             0x53
#define PMBUS_CMD_UT_FAULT_RESPONSE          0x54
#define PMBUS_CMD_VIN_OV_FAULT_LIMIT         0x55
#define PMBUS_CMD_VIN_OV_FAULT_RESPONSE      0x56
#define PMBUS_CMD_VIN_OV_WARN_LIMIT          0x57
#define PMBUS_CMD_VIN_UV_WARN_LIMIT          0x58
#define PMBUS_CMD_VIN_UV_FAULT_LIMIT         0x59
#define PMBUS_CMD_IIN_OC_FAULT_LIMIT         0x5B
#define PMBUS_CMD_IIN_OC_WARN_LIMIT          0x5D
#define PMBUS_CMD_POWER_GOOD_ON              0x5E
#define PMBUS_CMD_POWER_GOOD_OFF             0x5F
#define PMBUS_CMD_TON_DELAY                  0x60
#define PMBUS_CMD_POUT_OP_FAULT_LIMIT        0x68
#define PMBUS_CMD_POUT_OP_WARN_LIMIT         0x6A
#define PMBUS_CMD_PIN_OP_WARN_LIMIT          0x6B
#define PMBUS_CMD_READ_VIN1                  0x75
#define PMBUS_CMD_STATUS_BYTE                0x78
#define PMBUS_CMD_STATUS_WORD                0x79
#define PMBUS_CMD_STATUS_VOUT		         0x7A
#define PMBUS_CMD_STATUS_IOUT	             0x7B
#define PMBUS_CMD_STATUS_INPUT               0x7C
#define PMBUS_CMD_STATUS_TEMP                0x7D
#define PMBUS_CMD_STATUS_CML                 0x7E
#define PMBUS_CMD_STATUS_OTHER	             0x7F
#define PMBUS_CMD_STATUS_MFR_SPECIFIC	     0x80
#define PMBUS_CMD_STATUS_FAN_12		         0x81
#define PMBUS_CMD_STATUS_FAN_34		         0x82
#define PMBUS_CMD_READ_KWH_OUT		         0x84
#define PMBUS_CMD_READ_EIN                   0x86
#define PMBUS_CMD_READ_EOUT			         0x87
#define PMBUS_CMD_READ_VIN                   0x88
#define PMBUS_CMD_READ_IIN		 	         0x89
#define PMBUS_CMD_READ_VCAP			         0x8A
#define PMBUS_CMD_READ_VOUT                  0x8B
#define PMBUS_CMD_READ_IOUT                  0x8C
#define PMBUS_CMD_READ_TEMPERATURE_1         0x8D
#define PMBUS_CMD_READ_TEMPERATURE_2	     0x8E
#define PMBUS_CMD_READ_TEMPERATURE_3	     0x8F
#define PMBUS_CMD_READ_FAN_SPEED_1		     0x90
#define PMBUS_CMD_READ_FAN_SPEED_2	     	 0x91
#define PMBUS_CMD_READ_FAN_SPEED_3		     0x92
#define PMBUS_CMD_READ_FAN_SPEED_4		     0x93
#define PMBUS_CMD_READ_DUTY_CYCLE		     0x94
#define PMBUS_CMD_READ_FREQUENCY		     0x95
#define PMBUS_CMD_READ_POUT			         0x96
#define PMBUS_CMD_READ_PIN                   0x97
#define PMBUS_CMD_PMBUS_REVISION             0x98
#define PMBUS_CMD_MFR_ID                     0x99
#define PMBUS_CMD_MFR_MODEL                  0x9A
#define PMBUS_CMD_MFR_REVISION               0x9B
#define PMBUS_CMD_MFR_LOCATION		         0x9C
#define PMBUS_CMD_MFR_DATE                   0x9D
#define PMBUS_CMD_MFR_SERIAL		         0x9E

#define PMBUS_CMD_MFR_VIN_MIN		         0xA0
#define PMBUS_CMD_MFR_VIN_MAX		         0xA1
#define PMBUS_CMD_MFR_IIN_MAX		         0xA2
#define PMBUS_CMD_MFR_PIN_MAX		         0xA3
#define PMBUS_CMD_MFR_VOUT_MIN		         0xA4
#define PMBUS_CMD_MFR_VOUT_MAX		         0xA5
#define PMBUS_CMD_MFR_IOUT_MAX		         0xA6
#define PMBUS_CMD_MFR_POUT_MAX		         0xA7

#define PMBUS_CMD_IC_DEVICE_ID		         0xAD
#define PMBUS_CMD_IC_DEVICE_REV		         0xAE

#define PMBUS_CMD_MFR_MAX_TEMP_1		     0xC0
#define PMBUS_CMD_MFR_MAX_TEMP_2		     0xC1
#define PMBUS_CMD_MFR_MAX_TEMP_3		     0xC2

#define PMBUS_VIRT_BASE	    		         0x100


/* PMBus Data Format Definitions */
#define PMBUS_LINEAR11_EXPONENT_MASK         0xE000
#define PMBUS_LINEAR11_EXPONENT_SHIFT        13
#define PMBUS_LINEAR11_MANTISSA_MASK         0x1FFF
#define PMBUS_LINEAR16_EXPONENT_MASK         0xFF00
#define PMBUS_LINEAR16_EXPONENT_SHIFT        8
#define PMBUS_LINEAR16_MANTISSA_MASK         0x00FF

/* PMBus constants */
#define PMBUS_MAX_READ_RETRIES               3
#define PMBUS_MAX_BLOCK_DATA_SIZE            32

/* 自定义的2的幂函数实现 */
float my_powf(float base, int exponent);

/* PMBUS API */
int ls_pmbus_read_word(const struct smbus_dt_spec *smbus, uint8_t cmd, uint16_t *value);
int ls_pmbus_write_word(const struct smbus_dt_spec *smbus, uint8_t cmd, uint16_t value);
int ls_pmbus_write_byte(const struct smbus_dt_spec *smbus, uint8_t cmd);
int ls_pmbus_write_byte_data(const struct smbus_dt_spec *smbus, uint8_t cmd, uint8_t value);
int ls_pmbus_read_byte(const struct smbus_dt_spec *smbus, uint8_t cmd, uint8_t *value);
int ls_pmbus_read_block(const struct smbus_dt_spec *smbus, uint8_t cmd, uint8_t *len, uint8_t *data);
int ls_pmbus_write_block(const struct smbus_dt_spec *smbus, uint8_t cmd, uint8_t len, const uint8_t *data);
int ls_pmbus_process_call(const struct smbus_dt_spec *smbus, uint8_t cmd, uint16_t send_word, uint16_t *recv_word);
int ls_pmbus_select_page(const struct smbus_dt_spec *smbus, uint8_t page);
int ls_pmbus_clear_faults(const struct smbus_dt_spec *smbus);
int ls_pmbus_verify_device(const struct smbus_dt_spec *smbus);
float ls_pmbus_parse_linear11(uint16_t value);
float ls_pmbus_parse_linear16(uint16_t value);
int ls_pmbus_configure_pec(const struct smbus_dt_spec *smbus, bool enable);
