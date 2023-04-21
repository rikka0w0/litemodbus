#ifndef MODBUS_INCLUDE_MODBUS_UTILS_H_
#define MODBUS_INCLUDE_MODBUS_UTILS_H_

#include <stdint.h>

/**
 * This function performs one iteration of CRC16 hashing. The starting crc16 value should be 0xFFFF.
 * This function assuming the input byte stream is big-endian, as stated in Modbus specification,
 * regardless of the host processor architecture.
 * See: https://crccalc.com/
 */
uint16_t modbus_crc16_hash_byte(uint16_t crc16, uint_least8_t val);

#endif /* MODBUS_INCLUDE_MODBUS_UTILS_H_ */
