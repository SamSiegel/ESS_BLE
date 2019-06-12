#include <stdio.h>
#include "sparcommon.h"
#include "wiced_bt_trace.h"
#include "wiced_platform.h"
#include "wiced_hal_puart.h"
#include "wiced_hal_i2c.h"
#include "wiced_timer.h"
#include "wiced_rtos.h"

//SHT Defines
#define STATUS_OK 0
#define STATUS_ERR_BAD_DATA (-1)
#define STATUS_CRC_FAIL (-2)
#define STATUS_UNKNOWN_DEVICE (-3)
#define STATUS_FAIL (-1)

//CRC_Common
#define be16_to_cpu(s) (s)
#define be32_to_cpu(s) (s)
#define be64_to_cpu(s) (s)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))
#define CRC8_POLYNOMIAL             0x31
#define CRC8_INIT                   0xFF
#define CRC8_LEN                    1
#define SHT_MEAS_DURATION_MSEC      15
#define MEASUREMENT_DURATION_USEC   14400

//SHT consts
static const uint8_t CMD_MEASURE_HPM[]     = { 0x78, 0x66 };
static const uint8_t CMD_READ_ID_REG[]     = { 0xef, 0xc8 };
static const uint8_t COMMAND_SIZE = sizeof(CMD_MEASURE_HPM);
static const uint8_t SHTC1_ADDRESS = 0x70;
static const uint8_t ID_REG_CONTENT    = 0x07;
static const uint8_t ID_REG_MASK       = 0x1f;
//SGP Defines
#define SGP30_DEVICE_ADDRESS        0x58

//Sensirion Common
int8_t sensirion_common_check_crc(uint8_t *data, uint16_t count, uint8_t checksum);
uint8_t sensirion_common_generate_crc(uint8_t *data, uint16_t count);
//SHT Common functions
int8_t sht_common_read_ticks(uint8_t address, int32_t *temperature_ticks, int32_t *humidity_ticks);
int8_t sht_common_read_measurement(uint8_t address, int32_t *temperature, int32_t *humidity);
//SHTC1 Functions
int8_t sht_probe(void);
int8_t sht_measure_blocking_read(int32_t *temperature, int32_t *humidity);
int8_t sht_measure(void);
int8_t sht_read(int32_t *temperature, int32_t *humidity);
//sgp30 functions
int8_t sgp_init(void);
int8_t sgp_read_iaq(uint16_t *eCO2, uint16_t *tvoc);
