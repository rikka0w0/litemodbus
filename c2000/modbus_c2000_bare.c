#include "modbus_config.h"
#include "modbus_c2000_priv.h"
#if defined(MODBUS_PLATFORM_C2000) && !defined(MODBUS_FREERTOS)

#include "modbus_c2000.h"
#include "modbus_protocol.h"
#include "modbus_buf16.h"
#include "modbus_utils.h"

static uint16_t modbus_crc16(const uint16_t* buf, uint16_t len_byte) {
    uint16_t word;
    uint16_t crc16 = 0xFFFF;

    for (;len_byte > 1; len_byte -= 2) {
        word = *buf++;

        // Lower byte
        crc16 = modbus_crc16_hash_byte(crc16, word & 0xFF);

        // Higher byte
        crc16 = modbus_crc16_hash_byte(crc16, (word >> 8) & 0xFF);
    }

    if (len_byte) {
        word = *buf;
        crc16 = modbus_crc16_hash_byte(crc16, word & 0xFF);
    }
    return crc16;
}

static inline void sci_reset(volatile struct SCI_REGS* sci) {
    // To clear all errors a sw reset of the module is required
    sci->SCICTL1.bit.SWRESET = 0;
    sci->SCICTL1.bit.SWRESET = 1;
}

static inline void sci_enable_rx(volatile struct SCI_REGS* sci) {
    sci->SCIFFRX.bit.RXFIFORESET = 0;   // Start FIFO reset
    sci->SCIFFRX.bit.RXFFINTCLR = 1;    // Clear RXFFINT
    sci->SCIFFRX.bit.RXFFOVRCLR = 1;    // Clear RXFFOVF
    sci->SCIFFRX.bit.RXFIFORESET = 1;   // End FIFO reset
    sci->SCICTL1.bit.RXENA = 1;         // Enable receiver
}

static inline void sci_disable_rx(volatile struct SCI_REGS* sci) {
    sci->SCICTL1.bit.RXENA = 0;
}

void modbus_sci_init(modbus_c2000* mb) {
    volatile struct SCI_REGS* const sci = mb->sci;
    // 1 stop bit,  No parity, 8 char bits, No loopback, idle-line protocol
    sci->SCICCR.all = 0x0007;

    // Enable interrupt flags
//    sci->SCICTL2.bit.RXBKINTENA = 1;    // SCI_INT_RXRDY_BRKDT
//    sci->SCICTL2.bit.TXRDY = 1;         // SCI_INT_TXRDY

    // Disable TX RX (for now), internal SCICLK,
    // Disable RX ERR, SLEEP, TXWAKE
    sci->SCICTL1.bit.RXERRINTENA = 0;
    sci->SCICTL1.all = 0x0020;

    sci_reset(sci);

    sci->SCIFFTX.all = 0xC040;      // SCIRST, SCIFFENA, TXFFINTCLR (Clear TXFFINT)
    sci->SCIFFRX.all = 0x204f;      // RXFIFORESET, RXFFINTCLR (Clear RXFFINT), RXFFIL = 0xF
    sci->SCIFFCT.all = 0x0;

    sci->SCIFFRX.bit.RXFFIL = 1;    // Set Rx FIFO threshold to 1
    // Set Tx FIFO threshold
    sci->SCIFFTX.bit.TXFFIL = SCI_TXFIFO_THRESHOLD;

    mb->state = mbc2000_idle;

    sci_enable_rx(sci);
}

void modbus_sci_loop(modbus_c2000* mb) {
    uint16_t tmp16;
    timestamp_t now = timestamp_get();
    switch(mb->state) {
        case mbc2000_idle:
            if (mb->sci->SCIFFRX.bit.RXFFINT) {
                mb->sci->SCIFFRX.bit.RXFFINTCLR = 1; // Clear RXFFINT

                tmp16 = c2000_sci_rx_buf(mb->sci);

                // Check if we are addressed
                mb->state = (tmp16 == mb->listening_addr || tmp16 == 0) ? mbc2000_rx_addressed : mbc2000_rx_skipping;

                // Put the address byte to the buffer
                mb->rx_buf[0] = tmp16;
                mb->rx_len = 1;
            } else if (mb->sci->SCIRXST.bit.RXERROR) {
                sci_reset(mb->sci);
            }
            break;

        case mbc2000_rx_addressed:
            if (mb->sci->SCIRXST.bit.RXWAKE) {
                sci_disable_rx(mb->sci);

                mb->next_timeout_timestamp = timestamp_schedule(MODBUS_INTERFRAME_GAP_US, now);
                if (mb->rx_len > 4) {
                    // Extract CRC16 from the request
                    tmp16 = modbus_get_buf_byte(mb->rx_buf, mb->rx_len-2);
                    tmp16 |= modbus_get_buf_byte(mb->rx_buf, mb->rx_len-1) << 8;

                    // Calc and check CRC16
                    if (tmp16 == modbus_crc16(mb->rx_buf, mb->rx_len-2)) {
                        // Process the request and generate response
                        mb->tx_len = parse_query_packet(&mb->modbus_dev_def, mb->rx_buf, mb->rx_len-2, mb->tx_buf);

                        if (modbus_get_buf_byte(mb->rx_buf, 0) == 0) {
                            // Do not respond to broadcasting message
                            mb->state = mbc2000_interframe_gap_rx;
                        } else {
                            // Append CRC16
                            tmp16 = modbus_crc16(mb->tx_buf, mb->tx_len);
                            modbus_set_buf_byte(mb->tx_buf, mb->tx_len++, tmp16 & 0xFF);    // Lower byte
                            modbus_set_buf_byte(mb->tx_buf, mb->tx_len++, tmp16 >> 8);      // Higher byte
                            mb->state = mbc2000_interframe_gap_tx;
                        }
                        break;
                    }
                }

                // Invalid packet, resume listening later.
                mb->state = mbc2000_interframe_gap_rx;
                break;
            }

            while(mb->sci->SCIFFRX.bit.RXFFST) {
                tmp16 = c2000_sci_rx_buf(mb->sci);
                if (mb->rx_len & 1) {
                    // Odd byte
                    tmp16 <<= 8; // Little endian
                    mb->rx_buf[mb->rx_len >> 1] |= tmp16;
                } else {
                    // Even byte
                    mb->rx_buf[mb->rx_len >> 1] = tmp16;
                }

                if (mb->rx_len >= RX_BUFLEN_BYTE) {
                    // Buffer is full, discard the rest
                    mb->state = mbc2000_rx_skipping;
                } else {
                    mb->rx_len++;
                }
            }
            break;

        case mbc2000_rx_skipping:
            if (mb->sci->SCIRXST.bit.RXWAKE) {
                // After skip this frame, resume listening immediately
                sci_disable_rx(mb->sci);
                sci_enable_rx(mb->sci);
                mb->state = mbc2000_idle;
            }
            break;

        case mbc2000_interframe_gap_rx:
            if (timestamp_reached(now, mb->next_timeout_timestamp)) {
                sci_enable_rx(mb->sci);
                mb->state = mbc2000_idle;
            }
            break;

        case mbc2000_interframe_gap_tx:
            if (timestamp_reached(now, mb->next_timeout_timestamp)) {
                c2000_sci_tx_fifo_reset(mb->sci, 0);    // Start FIFO reset
                mb->sci->SCIFFTX.bit.TXFFINTCLR = 1;    // Clear TXFFINT
                c2000_sci_tx_fifo_reset(mb->sci, 1);    // End FIFO reset
                mb->sci->SCICTL1.bit.TXENA = 1;
                mb->tx_ptr = 0;
                mb->state = mbc2000_tx_push_fifo;
            }
            break;

        case mbc2000_tx_push_fifo:
            if (mb->sci->SCIFFTX.bit.TXFFINT) {
                mb->sci->SCIFFTX.bit.TXFFINTCLR = 1;    // Clear TXFFINT

                mb->de_set();

                tmp16 = mb->tx_buf[mb->tx_ptr >> 1];
                if (mb->tx_ptr & 1) {
                    // Odd byte
                    tmp16 >>= 8;
                }
                c2000_sci_tx_buf(mb->sci, tmp16);

                mb->tx_ptr++;
                if (mb->tx_ptr == mb->tx_len) {
                    mb->state = mbc2000_tx_wait_done;
                }
            }
            break;

        case mbc2000_tx_wait_done:
            if (mb->sci->SCICTL2.bit.TXEMPTY && !mb->sci->SCIFFTX.bit.TXFFST) {
                // Disable Tx
                mb->sci->SCICTL1.bit.TXENA = 0;

                mb->next_timeout_timestamp = timestamp_schedule(mb->de_tail_us, now);
                mb->state = mbc2000_tx_de_tail;
            }
            break;

        case mbc2000_tx_de_tail:
            if (timestamp_reached(now, mb->next_timeout_timestamp)) {
                mb->de_clr();
                mb->next_timeout_timestamp = timestamp_schedule(MODBUS_INTERFRAME_GAP_US, now);
                mb->state = mbc2000_interframe_gap_rx;
            }
            break;

        default:
            break;
    }
}

#endif
