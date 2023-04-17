#include "modbus_config.h"
#ifdef MODBUS_STM32F0
#include "modbus_stm32f0.h"

#if MODBUS_FREERTOS
#include <limits.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include "timestamp.h"
#endif

#include "stm32f0xx_it.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_gpio.h"

#define MODBUS_INTERFRAME_GAP 3300      // uS
#define MODBUS_MODE_TX() RS485_DE_HIGH()
#define MODBUS_MODE_RX() RS485_DE_LOW()

static uint8_t modbus_addr;

static volatile uint32_t uart_rx_len;           // The length without CRC16
volatile uint8_t uart_rx_buf[UART_BUFLEN];      // Populated by DMA

static volatile uint32_t uart_tx_len;
volatile uint8_t uart_tx_buf[UART_BUFLEN];      // Consumed by DMA

// Modbus State Machine
enum modbus_pending_tasks {
    modbus_no_pending = 0,
    modbus_enable_rx = 1,
    modbus_send_txbuf = 2
};

#if(MODBUS_FREERTOS)
// number of words allocated to the modbus task
#define MODBUS_STACK_SIZE 128
#define MODBUS_TASK_PRIORITY (configMAX_PRIORITIES-2)
#define MODBUS_INTERFRAME_GAP_TICKS (pdUS_TO_TICKS(MODBUS_INTERFRAME_GAP))
static TaskHandle_t modbus_rtostask;
#else
static enum modbus_pending_tasks modbus_next_task = modbus_no_pending;
static timestamp_t modbus_last_frame_timestamp = 0;
static timestamp_t modbus_next_timeout_timestamp = 0;
static void modbus_schedule_task(enum modbus_pending_tasks task, uint32_t timeout) {
    modbus_next_timeout_timestamp = modbus_last_frame_timestamp + timeout;
    modbus_next_task = task;
}
#endif

uint16_t modbus_crc16(const uint8_t *data, uint32_t dat_len)
{
    static const uint16_t crc16_table[] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };

    uint8_t temp;
    uint16_t crc16 = 0xFFFF;

    while (dat_len--)
    {
        temp = *data++ ^ crc16;
        crc16 >>= 8;
        crc16 ^= crc16_table[temp];
    }
    return crc16;
} // End: CRC16


/**
 * The Interrupt Handler for USART1
 */
void USART1_IRQHandler(void) {
#if(MODBUS_FREERTOS)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#endif

    if (USART1->ISR & USART_ISR_RTOF) {
        // Silent interval detected!
        // Clear the Interrupt Flag (Receiver timeout flag)
        LL_USART_ClearFlag_RTO(USART1);

        // Disable Rx until we finish processing this request, if it is valid
        LL_USART_DisableDirectionRx(USART1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);

#if(!MODBUS_FREERTOS)
        modbus_last_frame_timestamp = timestamp_get();
#endif

        uint32_t rx_len = UART_BUFLEN - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
        // Discard invalid message (too short)
        if (rx_len > 4) {
            // Discard the message if we are not the recipient
            if (uart_rx_buf[0] == modbus_addr || uart_rx_buf[0] == 0) {
                rx_len -= 2;
                uint16_t crc16_calc = modbus_crc16((uint8_t*) uart_rx_buf, rx_len);
                uint16_t crc16_recv = uart_rx_buf[rx_len+1];
                crc16_recv <<= 8;
                crc16_recv |= uart_rx_buf[rx_len];

                // Check CRC
                if (crc16_calc == crc16_recv) {
                    uart_rx_len = rx_len;

#if(MODBUS_FREERTOS)
                    xTaskNotifyFromISR(modbus_rtostask, modbus_send_txbuf, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif

                    // Return so that we will not resume listening immediately
                    return;
                }
            }
        }

        // Resume listening if we don't need to process the message
#if(MODBUS_FREERTOS)
        xTaskNotifyFromISR(modbus_rtostask, modbus_enable_rx, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
        modbus_schedule_task(modbus_enable_rx, MODBUS_INTERFRAME_GAP);
#endif
    } else if (USART1->ISR & USART_ISR_TC) {
        // Transmission completed

        // Clear the Interrupt Flag and disable the TC interrupt
        LL_USART_ClearFlag_TC(USART1);
        LL_USART_DisableIT_TC(USART1);

        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        LL_USART_DisableDirectionTx(USART1);
        uart_tx_len = 0;
        MODBUS_MODE_RX();

        // Resume listening
#if(MODBUS_FREERTOS)
        xTaskNotifyFromISR(modbus_rtostask, modbus_enable_rx, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
        modbus_last_frame_timestamp = timestamp_get();
        modbus_schedule_task(modbus_enable_rx, MODBUS_INTERFRAME_GAP);
#endif
    }
}

void modbus_set_dev_addr(uint8_t addr) {
    modbus_addr = addr;
}

uint8_t modbus_get_dev_addr() {
    return modbus_addr;
}

void modbus_enable_listening(void) {
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, UART_BUFLEN);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_USART_EnableDirectionRx(USART1);
}

typedef uint8_t (*field_action)(const modbus_reg_map_def_t* cur_reg_range, uint16_t cur_addr, uint16_t i, void* param);

static uint8_t collect_mutex_flag(const modbus_reg_map_def_t* cur_reg_range, uint16_t cur_addr, uint16_t i, void* param) {
    uint32_t* tx_buf_index = (uint32_t*) param;
    *tx_buf_index |= (1 << cur_reg_range->reg_mutex_flag_bit);
    return MODBUS_ERR_NOERR;
}

static uint8_t process_read(const modbus_reg_map_def_t* cur_reg_range, uint16_t cur_addr, uint16_t i, void* param) {
    uint16_t* tx_buf_index = (uint16_t*) param;
    uint8_t error_code = MODBUS_ERR_NOERR;
    uint16_t reg_val;
    switch (cur_reg_range->reg_data_type) {
    case MODBUS_DATATYPE_INT16_GAP:
        error_code = MODBUS_ERR_ILLEGAL_ADDR;
        break;
    case MODBUS_DATATYPE_INT16_DYN:
        error_code = ((modbus_dyn_reg_func)cur_reg_range->mem)(cur_addr, &reg_val, 0);
        break;
    case MODBUS_DATATYPE_INT16:
        reg_val = ((uint16_t*)(cur_reg_range->mem))[i];
        break;
    case MODBUS_DATATYPE_INT32_LH:
        break;
    case MODBUS_DATATYPE_INT32_HL:
        break;
    default:
        break;
    }

    if (error_code == MODBUS_ERR_NOERR) {
        uart_tx_buf[*tx_buf_index] = (reg_val >> 8) & 0xFF; // Higher byte
        *tx_buf_index = *tx_buf_index + 1;
        uart_tx_buf[*tx_buf_index] = reg_val & 0xFF;        // Lower byte
        *tx_buf_index = *tx_buf_index + 1;
    }

    return error_code;
}

static uint8_t process_write(const modbus_reg_map_def_t* cur_reg_range, uint16_t cur_addr, uint16_t i, void* param) {
    uint16_t* rx_buf_index = (uint16_t*) param;
    uint8_t error_code = MODBUS_ERR_NOERR;
    uint16_t reg_val = uart_rx_buf[*rx_buf_index];        // Higher byte
    reg_val <<= 8;
    *rx_buf_index = *rx_buf_index + 1;
    reg_val |= uart_rx_buf[*rx_buf_index];   // Lower byte
    *rx_buf_index = *rx_buf_index + 1;

    switch (cur_reg_range->reg_data_type) {
    case MODBUS_DATATYPE_INT16_GAP:
        error_code = MODBUS_ERR_ILLEGAL_ADDR;
        break;
    case MODBUS_DATATYPE_INT16_DYN:
        error_code = ((modbus_dyn_reg_func)cur_reg_range->mem)(cur_addr, &reg_val, 1);
        break;
    case MODBUS_DATATYPE_INT16:
        ((uint16_t*)(cur_reg_range->mem))[i] = reg_val;
        break;
    case MODBUS_DATATYPE_INT32_LH:
        break;
    case MODBUS_DATATYPE_INT32_HL:
        break;
    default:
        break;
    }

    return error_code;
}

static uint8_t process_iteration(uint16_t start_addr, uint16_t count, void* param, field_action action) {
    const modbus_reg_map_def_t* reg_def_start = uart_rx_buf[1] == MODBUS_FUNC_READ_INPUT
            ? modbus_get_input_regdef() : modbus_get_holding_regdef();
    uint8_t error_code = MODBUS_ERR_NOERR;

    uint16_t cur_range_index = 0;
    uint16_t cur_range_addr_offset = 0;

    const uint16_t end_addr = start_addr + count;
    uint16_t cur_addr = start_addr;

    while (cur_addr < end_addr) {
        modbus_reg_map_def_t const* cur_reg_range = reg_def_start + cur_range_index;

        if (cur_reg_range->reg_count == 0) {
            // Reached the end marker, out of defined range
            error_code = MODBUS_ERR_ILLEGAL_ADDR;
            break;
        } else if (cur_addr >= cur_range_addr_offset + cur_reg_range->reg_count) {
            // Skip current range
            cur_range_addr_offset += cur_reg_range->reg_count;
            cur_range_index++;
        } else {
            uint16_t i = cur_addr - cur_range_addr_offset;

            error_code = action(cur_reg_range, cur_addr, i, param);
            if (error_code != MODBUS_ERR_NOERR) {
                break;
            }

            cur_addr++;
            if (cur_addr == cur_range_addr_offset + cur_reg_range->reg_count) {
                // Move to the next range
                cur_range_addr_offset = cur_addr;
                cur_range_index++;
            }
        }
    }

    return error_code;
}

/*
 * Parse the query in uart_rx_buf and generate a response in uart_tx_buf without sending it.
 */
static void parse_query_packet(void) {
    // Prepare response
    // If error_code != MODBUS_ERR_NOERR, then response_len will be ignored
    uint8_t error_code = MODBUS_ERR_NOERR;
    uint16_t response_len = 2;
    uart_tx_buf[0] = uart_rx_buf[0];    // Our device address
    uart_tx_buf[1] = uart_rx_buf[1];    // Copy function code

    // Parse the query
    uint32_t mutex_flag;
    uint16_t reg_addr, reg_num, var16;
    switch(uart_rx_buf[1]) {
    case MODBUS_FUNC_READ_REG:          // Read analog output / Read holding registers
    case MODBUS_FUNC_READ_INPUT:        // Read analog input
        if (uart_rx_len < 6) {
            error_code = MODBUS_ERR_ILLEGAL_FUNC;
            break;
        }

        reg_addr = uart_rx_buf[2];
        reg_addr <<= 8;
        reg_addr |= uart_rx_buf[3];

        reg_num = uart_rx_buf[4];
        reg_num <<= 8;
        reg_num |= uart_rx_buf[5];

        if (reg_num > 125) {
            error_code = MODBUS_ERR_ILLEGAL_FUNC;
            break;
        }

#if(MODBUS_FREERTOS)
        process_iteration(reg_addr, reg_num, &mutex_flag, collect_mutex_flag);
        modbus_mutex_take(mutex_flag);
#endif
        response_len = 3;
        error_code = process_iteration(reg_addr, reg_num, &response_len, &process_read);
#if(MODBUS_FREERTOS)
        modbus_mutex_give(mutex_flag);
#endif

        // Find out byte number of all fields.
        uart_tx_buf[2] = (response_len-3);
        break;
    case MODBUS_FUNC_WRITE_1REG:        // Set single analog output / Write a holding register
        if (uart_rx_len < 6) {
            error_code = MODBUS_ERR_ILLEGAL_FUNC;
            break;
        }

        reg_addr = uart_rx_buf[2];
        reg_addr <<= 8;
        reg_addr |= uart_rx_buf[3];

#if(MODBUS_FREERTOS)
        process_iteration(reg_addr, 1, &mutex_flag, collect_mutex_flag);
        modbus_mutex_take(mutex_flag);
#endif
        var16 = 4;
        error_code = process_iteration(reg_addr, 1, &var16, process_write);
#if(MODBUS_FREERTOS)
        modbus_mutex_give(mutex_flag);
#endif

        if (error_code == MODBUS_ERR_NOERR) {
            // Address
            uart_tx_buf[2] = uart_rx_buf[2];
            uart_tx_buf[3] = uart_rx_buf[3];
            // Value
            uart_tx_buf[4] = uart_rx_buf[4];
            uart_tx_buf[5] = uart_rx_buf[5];

            response_len = 6;
        } else {
            response_len = 3;
        }

        break;
    case MODBUS_FUNC_WRITE_REGS:        // Set multiple analog output / Write multiple holding registers
        if (uart_rx_len < 9) {
            error_code = MODBUS_ERR_ILLEGAL_FUNC;
            break;
        }

        reg_addr = uart_rx_buf[2];
        reg_addr <<= 8;
        reg_addr |= uart_rx_buf[3];

        reg_num = uart_rx_buf[4];
        reg_num <<= 8;
        reg_num |= uart_rx_buf[5];

        // Number of value field
        var16 = uart_rx_buf[6] >> 1;
        if (var16 < ((uart_rx_len-7)>>1) || var16 < reg_num) {
            // We don't have enough value for the operation
            error_code = MODBUS_ERR_ILLEGAL_VAL;
            break;
        }

#if(MODBUS_FREERTOS)
        process_iteration(reg_addr, reg_num, &mutex_flag, collect_mutex_flag);
        modbus_mutex_take(mutex_flag);
#endif
        var16 = 7;
        error_code = process_iteration(reg_addr, reg_num, &var16, process_write);
#if(MODBUS_FREERTOS)
        modbus_mutex_give(mutex_flag);
#endif

        if (error_code == MODBUS_ERR_NOERR) {
            // Address
            uart_tx_buf[2] = uart_rx_buf[2];
            uart_tx_buf[3] = uart_rx_buf[3];
            // Value
            uart_tx_buf[4] = uart_rx_buf[4];
            uart_tx_buf[5] = uart_rx_buf[5];
            // Length
            response_len = 6;
        }
        break;
    default:
        error_code = MODBUS_ERR_ILLEGAL_FUNC;
        break;
    }

    if (error_code != MODBUS_ERR_NOERR) {
        uart_tx_buf[1] = uart_rx_buf[1] | 0x80;
        uart_tx_buf[2] = error_code;
        response_len = 3;
    }

    var16 = modbus_crc16((uint8_t*) uart_tx_buf, response_len);
    uart_tx_buf[response_len] = var16 & 0xFF;       // Lower Byte
    response_len++;
    var16>>=8;
    uart_tx_buf[response_len] = var16 & 0xFF;   // Higher Byte
    response_len++;
    uart_tx_len = response_len;

    // Clear previous states
    uart_rx_len = 0;
}

static void start_tx(void) {
    MODBUS_MODE_TX();
    LL_USART_ClearFlag_TC(USART1);
    LL_USART_EnableIT_TC(USART1);   // Fire interrupt when transmission is done
    LL_USART_EnableDirectionTx(USART1);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, uart_tx_len);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}

#if(MODBUS_FREERTOS)
static void modbus_rtos_task(void *pvParameters) {
    uint32_t event;

    modbus_enable_listening();

    while (1) {
        xTaskNotifyWait(
                0x00,           // Don't clear any bits on entry.
                ULONG_MAX,      // Clear all bits on exit.
                &event,         // Receives the notification value.
                portMAX_DELAY); // Block indefinitely.

        if (event == modbus_enable_rx) {
            vTaskDelay(MODBUS_INTERFRAME_GAP_TICKS);
            modbus_enable_listening();
        } else {
            parse_query_packet();

            if (uart_tx_buf[0] == 0) {
                // Broadcasting message, do not send response
                uart_tx_len = 0;
                MODBUS_MODE_RX();

                vTaskDelay(MODBUS_INTERFRAME_GAP_TICKS);

                // Resume listening
                modbus_enable_listening();
            } else {
                vTaskDelay(MODBUS_INTERFRAME_GAP_TICKS);

                // Ready to send response
                start_tx();
            }
        }
    }
}

void modbus_freertos_init(void) {
    static StackType_t stack[MODBUS_STACK_SIZE];
    static StaticTask_t task_buffer;
    modbus_rtostask = xTaskCreateStatic(&modbus_rtos_task,
            "Modbus",
            MODBUS_STACK_SIZE,
            (void*)0,
            MODBUS_TASK_PRIORITY,
            stack,
            &task_buffer
            );
}
#else
/*
 * Called by main() periodically
 */
void modbus_task(void) {
    timestamp_t now = timestamp_get();
    if (modbus_next_task != modbus_no_pending && timestamp_reached(modbus_next_timeout_timestamp, now)) {
        // If we have a pending task to execute & it is the time
        if (modbus_next_task == modbus_enable_rx) {
            modbus_next_task = modbus_no_pending;
            modbus_enable_listening();
        } else if (modbus_next_task == modbus_send_txbuf) {
            modbus_next_task = modbus_no_pending;
            start_tx();
        }
    }

    if (uart_rx_len == 0)
        return;

    // We have a new incoming request
    parse_query_packet();

    if (uart_tx_buf[0] == 0) {
        // Broadcasting message, do not send response
        uart_tx_len = 0;
        MODBUS_MODE_RX();

        // Resume listening
        modbus_last_frame_timestamp = timestamp_get();
        modbus_schedule_task(modbus_enable_rx, MODBUS_INTERFRAME_GAP);
    } else {
        // Ready to send response
        modbus_schedule_task(modbus_send_txbuf, MODBUS_INTERFRAME_GAP);
    }
}
#endif
#endif
