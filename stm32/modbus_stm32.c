#include "modbus_stm32_priv.h"

#ifdef MODBUS_PLATFORM_STM32
#include "modbus_protocol.h"
#include "modbus_stm32.h"
#include "modbus_utils.h"
#include "timestamp.h"

static uint16_t modbus_crc16(const uint8_t *data, uint32_t dat_len) {
    uint16_t crc16 = 0xFFFF;

    while (dat_len--) {
        crc16 = modbus_crc16_hash_byte(crc16, *data++);
    }

    return crc16;
}

void modbus_enable_listening(const modbus_stm32* const mb) {
    // LL_DMA_SetDataLength
    mb->rx_dma->CNDTR = UART_BUFLEN;

    // LL_DMA_EnableChannel
    SET_BIT(mb->rx_dma->CCR, DMA_CCR_EN);

    LL_USART_EnableDirectionRx(mb->uart);
}

/*
 * Called by main() periodically
 */
void modbus_uart_poll(modbus_stm32* mb) {
    uint16_t crc16_calc;
    union {
        uint16_t crc16_recv;
        uint16_t tx_len;
    } var16;
    timestamp_t now = timestamp_get();

    switch(mb->state) {
        case mbcstm32_listening:
            if (LL_USART_IsActiveFlag_RTO(mb->uart)) {
                // Silent interval detected!
                // Clear the Interrupt Flag (Receiver timeout flag)
                LL_USART_ClearFlag_RTO(mb->uart);

                // Disable Rx until we finish processing this request, if it is valid
                LL_USART_DisableDirectionRx(mb->uart);

                // LL_DMA_DisableChannel
                CLEAR_BIT(mb->rx_dma->CCR, DMA_CCR_EN);

                mb->next_timeout_timestamp = timestamp_schedule(MODBUS_INTERFRAME_GAP_US, now);

                // The length with CRC16
                uint16_t rx_len = RX_BUFLEN_BYTE - mb->rx_dma->CNDTR;

                // Discard invalid message (too short) or if we are not the recipient
                if (rx_len > 5 && (mb->rx_buf[0] == mb->listening_addr || mb->rx_buf[0] == 0)) {
                    // We have a new incoming request
                    crc16_calc = modbus_crc16((uint8_t*) mb->rx_buf, rx_len - 2);
                    var16.crc16_recv = mb->rx_buf[rx_len-1];
                    var16.crc16_recv <<= 8;
                    var16.crc16_recv |= mb->rx_buf[rx_len-2];

                    // Check CRC
                    if (crc16_calc == var16.crc16_recv) {
                        var16.tx_len = parse_query_packet(&mb->modbus_dev_def, mb->rx_buf, rx_len - 2, mb->tx_buf);

                        crc16_calc = modbus_crc16((uint8_t*) mb->tx_buf, var16.tx_len);
                        mb->tx_buf[var16.tx_len] = crc16_calc & 0xFF;   // Lower Byte
                        var16.tx_len++;
                        crc16_calc>>=8;
                        mb->tx_buf[var16.tx_len] = crc16_calc & 0xFF;   // Higher Byte
                        var16.tx_len++;

                        // LL_DMA_DisableChannel
                        CLEAR_BIT(mb->tx_dma->CCR, DMA_CCR_EN);

                        // LL_DMA_SetDataLength
                        mb->tx_dma->CNDTR = var16.tx_len;

                        // Do not answer to broadcast messages
                        mb->state = mb->rx_buf[0] == 0 ? mbcstm32_resume_rx : mbcstm32_send_response;
                    } else {
                        mb->state = mbcstm32_resume_rx;
                    }
                } else {
                    // Resume listening if we don't need to process the message
                    mb->state = mbcstm32_resume_rx;
                }
            }
            break;

        case mbcstm32_send_response:
            if (timestamp_reached(now, mb->next_timeout_timestamp)) {
                // Start sending response
                mb->de_set();
                LL_USART_ClearFlag_TC(mb->uart);
                LL_USART_EnableDirectionTx(mb->uart);

                // LL_DMA_EnableChannel
                SET_BIT(mb->tx_dma->CCR, DMA_CCR_EN);

                mb->state = mbcstm32_sending;
            }
            break;

        case mbcstm32_sending:
            if (LL_USART_IsActiveFlag_TC(mb->uart)) {
                // Transmission completed

                // Clear the Interrupt Flag and disable the TC interrupt
                LL_USART_ClearFlag_TC(mb->uart);

                // LL_DMA_DisableChannel
                CLEAR_BIT(mb->tx_dma->CCR, DMA_CCR_EN);

                LL_USART_DisableDirectionTx(mb->uart);
                mb->de_clr();

                // Resume listening
                mb->next_timeout_timestamp = timestamp_schedule(MODBUS_INTERFRAME_GAP_US, now);
                mb->state = mbcstm32_resume_rx;
            }
            break;

        case mbcstm32_resume_rx:
            if (timestamp_reached(now, mb->next_timeout_timestamp)) {
                modbus_enable_listening(mb);
                mb->state = mbcstm32_listening;
            }
            break;

        default:
            break;
    }
}
#endif // #ifdef MODBUS_PLATFORM_STM32
