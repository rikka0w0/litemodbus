#ifndef INCLUDE_MODBUS_C2000_H_
#define INCLUDE_MODBUS_C2000_H_

#include "modbus_config.h"
#include <stdint.h>
#include "modbus_c2000_priv.h"

#include "timestamp.h"
#include "modbus_common.h"

enum modbus_c2000_state {
    mbc2000_idle,
    mbc2000_rx_addressed,
    mbc2000_rx_skipping,
    mbc2000_interframe_gap_rx,
    mbc2000_interframe_gap_tx,
    mbc2000_tx_push_fifo,
    mbc2000_tx_wait_done,
    mbc2000_tx_de_tail,
    mbc2000_debug
};

typedef void (*gpio_action)(void);

typedef struct _modbus_c2000_t {
    volatile struct SCI_REGS* const sci;
    const modbus_dev_def_t modbus_dev_def;
    gpio_action const de_set;
    gpio_action const de_clr;
    int_least8_t de_tail_us;
    int_least8_t listening_addr;

    // Internal states
    enum modbus_c2000_state state;
    timestamp_t next_timeout_timestamp;

    uint16_t rx_len;
    uint16_t rx_buf[RX_BUFLEN_BYTE>>1];
    uint16_t tx_len;
    uint16_t tx_ptr;
    uint16_t tx_buf[TX_BUFLEN_BYTE>>1];
} modbus_c2000;

/**
 * Platform-specific SCI init.
 * You should set fields other than "Internal states" and
 * manually configure baud rate registers before calling
 * this function.
 */
void modbus_sci_init(modbus_c2000* mb);

void modbus_sci_loop(modbus_c2000* mb);

#endif /* INCLUDE_MODBUS_C2000_H_ */
