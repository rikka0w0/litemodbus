#include "timestamp_stm32.h"
#include "modbus_stm32_priv.h"

#ifndef TIMESTAMP_TICK2US
#error "Please define macro TIMESTAMP_TICK2US in modbus_config.h"
#endif

/*
 * Resources used:
 * SysTick
 */

/*
 * Assume SysTick is running at 48MHz and
 * the SysTick ISR is generated every 1mS.
 */

static volatile uint32_t timestamp_tick_ms;

void timestamp_systick(void) {
    timestamp_tick_ms++;
}

void timestamp_config() {
    LL_Init1msTick(SystemCoreClock);
    LL_SYSTICK_EnableIT();
}

timestamp_t timestamp_get(void) {
    uint32_t ms, systick;

    do {
        ms = timestamp_tick_ms;
        systick = SysTick->LOAD - SysTick->VAL;
    } while (ms != timestamp_tick_ms);

    //return ms * 1000 + systick / SYSTICK_PER_US;
    systick = TIMESTAMP_TICK2US(systick);
    return ((timestamp_t)ms * 1000) + systick;
}

