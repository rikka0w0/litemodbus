#ifndef MODBUS_STM32F0_H_
#define MODBUS_STM32F0_H_

#include <stdint.h>
#include "modbus_common.h"

#define UART_BUFLEN 255
#define MODBUS_PDU_MAXLEN 252

/**
 * Initialize the USART peripheral and configure DMA channels for Rx and Tx.
 */
void modbus_init(void);
void modbus_set_dev_addr(modbus_addr_t addr);
modbus_addr_t modbus_get_dev_addr();
void modbus_enable_listening(void);

#ifdef MODBUS_FREERTOS
void modbus_freertos_init(void);
#else
void modbus_task(void);
#endif

#endif /* MODBUS_STM32_H_ */
