#ifndef INCLUDE_MODBUS_BUF16_H_
#define INCLUDE_MODBUS_BUF16_H_

#include <stdint.h>

static inline uint_least8_t modbus_get_buf_byte(const void* buf_in, uint16_t i) {
    const uint16_t* buf = (const uint16_t*) buf_in;
    uint16_t tmp = buf[i>>1];
    tmp = (i & 1) ? (tmp >> 8) : tmp;
    return tmp & 0xFF;
}

static inline void modbus_set_buf_byte(void* buf_io, uint16_t i, uint_least8_t val) {
    uint16_t* buf = (uint16_t*) buf_io;
    if (i & 1) {
        // Set higher byte
        val = (val << 8);
        val |= buf[i>>1] & 0x00FF;
    } else {
        // Set lower byte
        val = val & 0xFF;
        val |= buf[i>>1] & 0xFF00;
    }
    buf[i>>1] = val;
}

#endif /* INCLUDE_MODBUS_BUF16_H_ */
