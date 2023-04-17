#ifndef MODBUS_H_
#define MODBUS_H_

#include <stdint.h>
#include "modbus_common.h"

// GPIO
#define RS485_DE_HIGH() GPIOF->BSRR = LL_GPIO_PIN_0
#define RS485_DE_LOW()  GPIOF->BRR = LL_GPIO_PIN_0
#define RS485_PV_HIGH() GPIOF->BSRR = LL_GPIO_PIN_1
#define RS485_PV_LOW()  GPIOF->BRR = LL_GPIO_PIN_1

// Configure the baud rate
#define MODBUS_BAUD 115200

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

#endif /* MODBUS_H_ */
