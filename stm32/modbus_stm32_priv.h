#ifndef MODBUD_STM32_PRIV_H_
#define MODBUS_STM32_PRIV_H_

#include "modbus_config.h"

#ifdef MODBUS_PLATFORM_STM32F0
#define MODBUS_PLATFORM_STM32

#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_utils.h"
#elif (defined(MODBUS_PLATFORM_STM32F3))
#define MODBUS_PLATFORM_STM32

#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_utils.h"
#else
#error "No STM32 platform has been selected!"
#endif

#endif /* MODBUS_STM32_PRIV_H_ */
