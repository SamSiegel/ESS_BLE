#include "sensirion.h"

//for SHT reads
int8_t sensirion_common_check_crc(uint8_t *data, uint16_t count, uint8_t checksum)
{
    if (sensirion_common_generate_crc(data, count) != checksum)
        return STATUS_FAIL;
    return STATUS_OK;
}

uint8_t sensirion_common_generate_crc(uint8_t *data, uint16_t count)
{
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

int8_t sht_common_read_ticks(uint8_t address, int32_t *temperature_ticks, int32_t *humidity_ticks)
{
    uint8_t data[6];
    int8_t ret = wiced_hal_i2c_read(data,sizeof(data),address);
    if (ret)
        return ret;
    if (sensirion_common_check_crc(data, 2, data[2]) ||
            sensirion_common_check_crc(data + 3, 2, data[5])) {
        return STATUS_CRC_FAIL;
    }

    *temperature_ticks = (data[1] & 0xff) | ((int32_t)data[0] << 8);
    *humidity_ticks = (data[4] & 0xff) | ((int32_t)data[3] << 8);

    return STATUS_OK;
}

int8_t sht_common_read_measurement(uint8_t address, int32_t *temperature, int32_t *humidity)
{
    int8_t ret = sht_common_read_ticks(address, temperature, humidity);
     /**
     * formulas for conversion of the sensor signals, optimized for fixed point algebra:
     * Temperature       = 175 * S_T / 2^16 - 45
     * Relative Humidity = 100 * S_RH / 2^16
     */
    *temperature = ((21875 * *temperature) >> 13) - 45000;
    *humidity = ((12500 * *humidity) >> 13);

    return ret;
}

int8_t sht_probe(void)
{
    uint8_t data[3];
    wiced_hal_i2c_init();
    int8_t ret = wiced_hal_i2c_write(CMD_READ_ID_REG, COMMAND_SIZE, SHTC1_ADDRESS);
    if (ret)
        return ret;

    ret = wiced_hal_i2c_read(data, sizeof(data),SHTC1_ADDRESS);
    if (ret)
        return ret;

    ret = sensirion_common_check_crc(data, 2, data[2]);
    if (ret)
        return ret;

    if ((data[1] & ID_REG_MASK) != ID_REG_CONTENT)
        return STATUS_UNKNOWN_DEVICE;
    return STATUS_OK;
}

int8_t sht_measure_blocking_read(int32_t *temperature, int32_t *humidity)
{
    int8_t ret = sht_measure();
    if (ret == STATUS_OK) {
        wiced_rtos_delay_microseconds(MEASUREMENT_DURATION_USEC);
        ret = sht_read(temperature, humidity);
    }
    return ret;
}

int8_t sht_measure(void)
{
    return wiced_hal_i2c_write(CMD_MEASURE_HPM,COMMAND_SIZE,SHTC1_ADDRESS);
}

int8_t sht_read(int32_t *temperature, int32_t *humidity)
{
    return sht_common_read_measurement(SHTC1_ADDRESS, temperature, humidity);
}

int8_t sgp_init(void){
    uint8_t ret;
    uint8_t cmd[] = {0x20, 0x03};
    ret = wiced_hal_i2c_write(cmd, sizeof(cmd), SGP30_DEVICE_ADDRESS);
    return ret;
}

int8_t sgp_read_iaq(uint16_t *eCO2, uint16_t *tvoc){
    uint8_t ret;
    uint8_t cmd[] = {0x20, 0x08};
    uint8_t iaq_data[6];

    ret = wiced_hal_i2c_write(cmd, sizeof(cmd), SGP30_DEVICE_ADDRESS);
    if(ret){
        return ret;
    }
    wiced_rtos_delay_milliseconds(12, ALLOW_THREAD_TO_SLEEP);
    ret = wiced_hal_i2c_read(iaq_data, sizeof(iaq_data), SGP30_DEVICE_ADDRESS);
    if (ret)
        return ret;
    if (sensirion_common_check_crc(iaq_data, 2, iaq_data[2]) ||
            sensirion_common_check_crc(iaq_data + 3, 2, iaq_data[5])) {
        return STATUS_CRC_FAIL;
    }
    *eCO2 = (iaq_data[1] & 0xff) | ((int16_t)iaq_data[0] << 8);
    *tvoc = (iaq_data[4] & 0xff) | ((int16_t)iaq_data[3] << 8);

    return ret;
}
