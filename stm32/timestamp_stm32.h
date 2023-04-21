#ifndef TIMESTAMP_STM32_H_
#define TIMESTAMP_STM32_H_

#include "timestamp.h"

/*
 * Initialize the timestamp generator.
 */
void timestamp_config();

/*
 * This should be called in SysTick_Handler or the tick function of the RTOS
 */
void timestamp_systick(void);

#endif
