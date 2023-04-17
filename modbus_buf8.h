#ifndef INCLUDE_MODBUS_BUF8_H_
#define INCLUDE_MODBUS_BUF8_H_

#include <stdint.h>

static inline uint_least8_t modbus_get_buf_byte(const void* buf_in, uint16_t i) {
    const uint16_t* buf = (const uint16_t*) buf_in;
    return buf[i];
}

static inline void modbus_set_buf_byte(void* buf_io, uint16_t i, uint_least8_t val) {
    uint16_t* buf = (uint16_t*) buf_io;
    buf[i] = val;
}

#endif /* INCLUDE_MODBUS_BUF8_H_ */
