#ifndef MODBUS_INCLUDE_MODBUS_C2000_PRIV_H_
#define MODBUS_INCLUDE_MODBUS_C2000_PRIV_H_

#include <stdint.h>
#include "modbus_config.h"

/**
 * Need to implement:
 *  c2000_sci_rx_buf: get 8-bit data from Rx FIFO
 *  c2000_sci_tx_buf: send 8-bit data to Tx FIFO
 *  c2000_sci_tx_fifo_reset: controls SCIFFTX.bit.TXFIFORESET
 */

#ifdef MODBUS_PLATFORM_TMS320F2806x
// F28062, F28069

#define MODBUS_PLATFORM_C2000
#include "F2806x_Device.h"

// Should be half of the FIFO depth
#define SCI_TXFIFO_THRESHOLD 2

static inline uint16_t c2000_sci_rx_buf(volatile struct SCI_REGS* const sci) {
    return sci->SCIRXBUF.bit.RXDT;
}

static inline void c2000_sci_tx_buf(volatile struct SCI_REGS* const sci, uint16_t u8Data) {
    sci->SCITXBUF = u8Data & 0xFF;
}

static inline void c2000_sci_tx_fifo_reset(volatile struct SCI_REGS* const sci, uint16_t bit_status) {
    sci->SCIFFTX.bit.TXFIFOXRESET = bit_status;
}

#elif defined(MODBUS_PLATFORM_TMS320F2837x)
// F28379D

#define MODBUS_PLATFORM_C2000
#include "F2837xD_device.h"

// Should be half of the FIFO depth
#define SCI_TXFIFO_THRESHOLD 8

static inline uint16_t c2000_sci_rx_buf(volatile struct SCI_REGS* const sci) {
    return sci->SCIRXBUF.bit.SAR;
}

static inline void c2000_sci_tx_buf(volatile struct SCI_REGS* const sci, uint16_t u8Data) {
    sci->SCITXBUF.bit.TXDT = u8Data & 0xFF;
}

static inline void c2000_sci_tx_fifo_reset(volatile struct SCI_REGS* const sci, uint16_t bit_status) {
    sci->SCIFFTX.bit.TXFIFORESET = bit_status;
}

#else
#error "No C2000 platform defined or unsupported platform!"
#endif

#endif /* MODBUS_INCLUDE_MODBUS_C2000_PRIV_H_ */
