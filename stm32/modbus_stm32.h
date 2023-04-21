#ifndef MODBUS_STM32_H_
#define MODBUS_STM32_H_

#include <stdint.h>
#include "modbus_common.h"
#include "modbus_stm32_priv.h"

#include "timestamp.h"

enum modbus_stm32_state {
    mbcstm32_listening,
    mbcstm32_send_response,
    mbcstm32_sending,
    mbcstm32_resume_rx,
    mbcstm32_debug
};

typedef void (*gpio_action)(void);

typedef struct _modbus_stm32_t {
    USART_TypeDef* const uart;
    DMA_Channel_TypeDef* const tx_dma;
    DMA_Channel_TypeDef* const rx_dma;
    const modbus_dev_def_t modbus_dev_def;
    gpio_action const de_set;
    gpio_action const de_clr;
    int_least8_t listening_addr;

    // Internal states
    enum modbus_stm32_state state;
    timestamp_t next_timeout_timestamp;

    uint8_t rx_buf[RX_BUFLEN_BYTE];
    uint8_t tx_buf[TX_BUFLEN_BYTE];
} modbus_stm32;

/**
 * Platform-specific SCI init.
 * You should set fields other than "Internal states" and
 * manually configure baud rate registers before calling
 * this function.
 */
void modbus_uart_init(modbus_stm32* mb);

#define UART_BUFLEN 255
#define MODBUS_PDU_MAXLEN 252

/**
 * Initialize the USART peripheral and configure DMA channels for Rx and Tx.
 */
void modbus_enable_listening(const modbus_stm32* const mb);

#ifdef MODBUS_FREERTOS
void modbus_freertos_init(void);
#else
void modbus_uart_poll(modbus_stm32* mb);
#endif

#endif /* MODBUS_STM32_H_ */
