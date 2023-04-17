#ifndef INCLUDE_MODBUS_PROTOCOL_H_
#define INCLUDE_MODBUS_PROTOCOL_H_

#include "modbus_common.h"

/**
 * @return the length of the response, in bytes, without CRC
 */
modbus_size_t parse_query_packet(const modbus_dev_def_t* dev_def, const void* rx_buf, modbus_size_t rx_len, void* tx_buf);

#endif /* INCLUDE_MODBUS_PROTOCOL_H_ */
