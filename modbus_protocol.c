#include "modbus_buf.h"
#include "modbus_protocol.h"

typedef struct {
    void* ptr;              // Holds a pointer to the actual tx or rx buffer
    uint16_t ptr_offset;    // The current offset within ptr

    uint32_t tmp_u32;
} iteration_context_t;

/*
 * Perform an action on a Modbus register.
 * @param cb_context    the callback context
 * @param context       holds some information between each iteration
 * @return error code
 */
typedef modbus_err_t (*field_action)(modbus_cb_context_t* cb_context, iteration_context_t* context);

#ifdef MODBUS_FREERTOS
static modbus_err_t collect_mutex_flag(modbus_cb_context_t* cb_context, iteration_context_t* context) {
    context->tmp_u32 |= (1 << cb_context>reg_def->reg_mutex_flag_bit);
    return MODBUS_ERR_NOERR;
}
#endif

static modbus_err_t process_read(modbus_cb_context_t* cb_context, iteration_context_t* context) {
    modbus_err_t error_code = MODBUS_ERR_NOERR;
    uint16_t reg_val;
    switch (cb_context->reg_def->data_type & MODBUS_DATATYPE_MASK) {
    case MODBUS_DATATYPE_INT16_GAP:
        reg_val = 0;
        break;

    case MODBUS_DATATYPE_INT16_DYN:
        error_code = cb_context->reg_def->callback(modbus_cb_read, cb_context, &reg_val);
        break;

    case MODBUS_DATATYPE_INT16:
        reg_val = MODBUS_MEM_U16(cb_context->reg_def, cb_context->reg_index);
        break;

    case MODBUS_DATATYPE_INT32:
        context->tmp_u32 = MODBUS_MEM_U32(cb_context->reg_def, cb_context->reg_index);
        context->tmp_u32 = (cb_context->reg_index & 1) ? (context->tmp_u32 & 0xFFFF) : (context->tmp_u32 >> 16) & 0xFFFF;
        reg_val = context->tmp_u32;
        break;

    default:
        while(1);   // Invalid data_type
    }

    if (error_code == MODBUS_ERR_NOERR) {
        modbus_set_buf_byte(context->ptr, context->ptr_offset, (reg_val >> 8) & 0xFF);    // Higher byte
        context->ptr_offset++;
        modbus_set_buf_byte(context->ptr, context->ptr_offset, reg_val & 0xFF);           // Lower byte
        context->ptr_offset++;
    }

    return error_code;
}

static inline modbus_dyn_val_t cur_reg_val(iteration_context_t* context) {
    modbus_dyn_val_t reg = {.changed = 0};

    reg.val = modbus_get_buf_byte(context->ptr, context->ptr_offset);   // Higher byte
    context->ptr_offset++;
    reg.val <<= 8;
    reg.val |= modbus_get_buf_byte(context->ptr, context->ptr_offset);  // Lower byte
    context->ptr_offset++;

    return reg;
}

static modbus_err_t process_write_check(modbus_cb_context_t* cb_context, iteration_context_t* context) {
    modbus_err_t error_code = MODBUS_ERR_NOERR;
    modbus_dyn_val_t reg = cur_reg_val(context);

    switch (cb_context->reg_def->data_type & MODBUS_DATATYPE_MASK) {
    case MODBUS_DATATYPE_INT16_GAP:
        break;

    case MODBUS_DATATYPE_INT16_DYN:
    case MODBUS_DATATYPE_INT16:
        // Check if the value is valid
        if (cb_context->reg_def->data_type & MODBUS_DATAFLAG_WRITE_CHECK) {
            error_code = cb_context->reg_def->callback(modbus_cb_write_check, cb_context, &reg.val);
        }
        break;

    case MODBUS_DATATYPE_INT32:
        if (cb_context->reg_index & 1) {
            // Lower word
            context->tmp_u32 |= (uint32_t)reg.val;

            if (cb_context->reg_addr == cb_context->start_addr) {
                // Partial write at the beginning
                error_code = MODBUS_ERR_ILLEGAL_VAL;
                break;
            }

            // Check if the value is valid
            if (cb_context->reg_def->data_type & MODBUS_DATAFLAG_WRITE_CHECK) {
                error_code = cb_context->reg_def->callback(modbus_cb_write_check, cb_context, &context->tmp_u32);
            }
        } else {
            // Higher word
            context->tmp_u32 = ((uint32_t)reg.val) << 16;

            if (cb_context->end_addr - cb_context->reg_addr < 2) {
                // Partial write at the end
                error_code = MODBUS_ERR_ILLEGAL_VAL;
            }
        }
        break;

    default:
        while(1);   // Invalid data_type
    }

    return error_code;
}

static modbus_err_t process_write(modbus_cb_context_t* cb_context, iteration_context_t* context) {
    modbus_err_t error_code = MODBUS_ERR_NOERR;
    modbus_dyn_val_t reg = cur_reg_val(context);

    switch (cb_context->reg_def->data_type & MODBUS_DATATYPE_MASK) {
    case MODBUS_DATATYPE_INT16_GAP:
        // Ignore writes to gap area
        break;

    case MODBUS_DATATYPE_INT16_DYN:
        error_code = cb_context->reg_def->callback(modbus_cb_write, cb_context, &reg);
        break;

    case MODBUS_DATATYPE_INT16:
        // Detect changes
        if (cb_context->reg_def->data_type & MODBUS_DATAFLAG_EQUAL_CHECK) {
            if (cb_context->reg_def->callback(modbus_cb_is_changed, cb_context, &reg.val)) {
                reg.changed = 1;
            }
        } else if (MODBUS_MEM_U16(cb_context->reg_def, cb_context->reg_index) != reg.val) {
            reg.changed = 1;
        }
        break;

    case MODBUS_DATATYPE_INT32:
        if (cb_context->reg_index & 1) {
            // Lower word
            context->tmp_u32 |= (uint32_t)reg.val;

            // Detect changes
            if (cb_context->reg_def->data_type & MODBUS_DATAFLAG_EQUAL_CHECK) {
                if (cb_context->reg_def->callback(modbus_cb_is_changed, cb_context, &context->tmp_u32)) {
                    reg.changed = 1;
                }
            } else if (MODBUS_MEM_U32(cb_context->reg_def, cb_context->reg_index) != context->tmp_u32) {
                reg.changed = 1;
            }
        } else {
            // Higher word
            context->tmp_u32 = ((uint32_t)reg.val) << 16;
        }
        break;

    default:
        break;
    }

    if (reg.changed) {
        // Write to the actual buffer
        switch (cb_context->reg_def->data_type & MODBUS_DATATYPE_MASK) {
        case MODBUS_DATATYPE_INT16:
            MODBUS_MEM_U16(cb_context->reg_def, cb_context->reg_index) = reg.val;
            break;

        case MODBUS_DATATYPE_INT32:
            MODBUS_MEM_U32(cb_context->reg_def, cb_context->reg_index) = context->tmp_u32;
            break;

        default:
            break;
        }

        if (cb_context->reg_def->data_type & MODBUS_DATAFLAG_NOTICE_NEW) {
            cb_context->reg_def->callback(modbus_cb_modified, cb_context, 0);
        }
    }

    return error_code;
}

static modbus_err_t process_iteration(modbus_cb_context_t* cb_context, uint16_t count, field_action action, iteration_context_t* context) {
    const modbus_reg_map_def_t* const first_reg_range = modbus_get_reg_def(cb_context->dev_def, cb_context->func_code);
    modbus_err_t error_code = MODBUS_ERR_NOERR;

    uint16_t cur_range_index = 0;
    uint16_t cur_range_addr_offset = 0;

    cb_context->end_addr = cb_context->start_addr + count;
    cb_context->reg_addr = cb_context->start_addr;
    
    while (cb_context->reg_addr < cb_context->end_addr) {
        cb_context->reg_def = first_reg_range + cur_range_index;

        if (cb_context->reg_def->reg_count == 0) {
            // Reached the end marker, out of defined range
            error_code = MODBUS_ERR_ILLEGAL_ADDR;
            break;
        } else if (cb_context->reg_addr >= cur_range_addr_offset + cb_context->reg_def->reg_count) {
            // Skip current range
            cur_range_addr_offset += cb_context->reg_def->reg_count;
            cur_range_index++;
        } else {
            cb_context->reg_index = cb_context->reg_addr - cur_range_addr_offset;

            error_code = action(cb_context, context);
            if (error_code != MODBUS_ERR_NOERR) {
                break;
            }

            cb_context->reg_addr++;
            if (cb_context->reg_addr == cur_range_addr_offset + cb_context->reg_def->reg_count) {
                // Move to the next range
                cur_range_addr_offset = cb_context->reg_addr;
                cur_range_index++;
            }
        }
    }

    return error_code;
}

modbus_size_t parse_query_packet(const modbus_dev_def_t* dev_def, const void* rx_buf, modbus_size_t rx_len, void* tx_buf) {
    // Prepare response
    // If error_code != MODBUS_ERR_NOERR, then response_len will be ignored
    modbus_err_t error_code = MODBUS_ERR_NOERR;

    modbus_cb_context_t cb_context = {
        .dev_def = dev_def,
        .func_code = modbus_get_buf_byte(rx_buf, 1),
    };

    iteration_context_t context;

#ifdef MODBUS_FREERTOS
    uint32_t mutex_flag = 0;
#endif

    // Parse the query
    uint16_t reg_num;
    switch(cb_context.func_code) {
    case MODBUS_FUNC_READ_REG:          // Read analog output / Read holding registers
    case MODBUS_FUNC_READ_INPUT:        // Read analog input
        if (rx_len < 6) {
            error_code = MODBUS_ERR_ILLEGAL_FUNC;
            break;
        }

        cb_context.start_addr = modbus_get_buf_byte(rx_buf, 2);
        cb_context.start_addr <<= 8;
        cb_context.start_addr |= modbus_get_buf_byte(rx_buf, 3);

        reg_num = modbus_get_buf_byte(rx_buf, 4);
        reg_num <<= 8;
        reg_num |= modbus_get_buf_byte(rx_buf, 5);

        if (reg_num > 125) {
            error_code = MODBUS_ERR_ILLEGAL_FUNC;
            break;
        }

#ifdef MODBUS_FREERTOS
        context.tmp_u32 = 0;
        process_iteration(&cb_context, reg_num, collect_mutex_flag, &context);
        mutex_flag = context.tmp_u32;
        dev_def->mutex_take(mutex_flag);
#endif
        context.ptr = tx_buf;
        context.ptr_offset = 3; // Store value offset
        error_code = process_iteration(&cb_context, reg_num, process_read, &context);
#ifdef MODBUS_FREERTOS
        dev_def->mutex_give(mutex_flag);
#endif

        context.ptr_offset = 2 * reg_num;
        modbus_set_buf_byte(tx_buf, 2, context.ptr_offset); // Byte number field
        context.ptr_offset += 3;
        break;

    case MODBUS_FUNC_WRITE_1REG:        // Set single analog output / Write a holding register
        if (rx_len < 6) {
            error_code = MODBUS_ERR_ILLEGAL_FUNC;
            break;
        }

        cb_context.start_addr = modbus_get_buf_byte(rx_buf, 2);
        cb_context.start_addr <<= 8;
        cb_context.start_addr |= modbus_get_buf_byte(rx_buf, 3);

#ifdef MODBUS_FREERTOS
        context.tmp_u32 = 0;
        process_iteration(&cb_context, reg_num, collect_mutex_flag, &context);
        mutex_flag = context.tmp_u32;
        dev_def->mutex_take(mutex_flag);
#endif
        context.ptr = (void*)rx_buf;
        // Validate the write request before apply it
        context.ptr_offset = 4; // Store value offset
        error_code = process_iteration(&cb_context, 1, process_write_check, &context);

        if (error_code == MODBUS_ERR_NOERR) {
            context.ptr_offset = 4; // Store value offset
            error_code = process_iteration(&cb_context, 1, process_write, &context);
        }
#ifdef MODBUS_FREERTOS
        dev_def->mutex_give(mutex_flag);
#endif

        // Address
        modbus_set_buf_byte(tx_buf, 2, modbus_get_buf_byte(rx_buf, 2));
        modbus_set_buf_byte(tx_buf, 3, modbus_get_buf_byte(rx_buf, 3));
        // Value
        modbus_set_buf_byte(tx_buf, 4, modbus_get_buf_byte(rx_buf, 4));
        modbus_set_buf_byte(tx_buf, 5, modbus_get_buf_byte(rx_buf, 5));
        // Length
        context.ptr_offset = 6;

        break;

    case MODBUS_FUNC_WRITE_REGS:        // Set multiple analog output / Write multiple holding registers
        if (rx_len < 9) {
            error_code = MODBUS_ERR_ILLEGAL_FUNC;
            break;
        }

        reg_num = modbus_get_buf_byte(rx_buf, 4);
        reg_num <<= 8;
        reg_num |= modbus_get_buf_byte(rx_buf, 5);

        // Number of value field
        cb_context.start_addr = modbus_get_buf_byte(rx_buf, 6) >> 1;
        if (cb_context.start_addr < ((rx_len-7)>>1) || cb_context.start_addr < reg_num) {
            // We don't have enough value for the operation
            error_code = MODBUS_ERR_ILLEGAL_VAL;
            break;
        }

        cb_context.start_addr = modbus_get_buf_byte(rx_buf, 2);
        cb_context.start_addr <<= 8;
        cb_context.start_addr |= modbus_get_buf_byte(rx_buf, 3);

#ifdef MODBUS_FREERTOS
        context.tmp_u32 = 0;
        process_iteration(&cb_context, reg_num, collect_mutex_flag, &context);
        mutex_flag = context.tmp_u32;
        dev_def->mutex_take(mutex_flag);
#endif
        context.ptr = (void*)rx_buf;
        // Validate all writes before apply them
        context.ptr_offset = 7; // Store value offset
        error_code = process_iteration(&cb_context, reg_num, process_write_check, &context);

        if (error_code == MODBUS_ERR_NOERR) {
            context.ptr_offset = 7; // Store value offset
            error_code = process_iteration(&cb_context, reg_num, process_write, &context);
        }
#ifdef MODBUS_FREERTOS
        dev_def->mutex_give(mutex_flag);
#endif

        // Address
        modbus_set_buf_byte(tx_buf, 2, modbus_get_buf_byte(rx_buf, 2));
        modbus_set_buf_byte(tx_buf, 3, modbus_get_buf_byte(rx_buf, 3));
        // Value
        modbus_set_buf_byte(tx_buf, 4, modbus_get_buf_byte(rx_buf, 4));
        modbus_set_buf_byte(tx_buf, 5, modbus_get_buf_byte(rx_buf, 5));
        // Length
        context.ptr_offset = 6;
        break;

    default:
        error_code = MODBUS_ERR_ILLEGAL_FUNC;
        break;
    }

    // Set header (3 byte)
    modbus_set_buf_byte(tx_buf, 0, modbus_get_buf_byte(rx_buf, 0)); // Our device address
    if (error_code == MODBUS_ERR_NOERR) {
        modbus_set_buf_byte(tx_buf, 1, cb_context.func_code); // Copy function code
        return context.ptr_offset;
    } else {
        modbus_set_buf_byte(tx_buf, 1, cb_context.func_code | 0x80);
        modbus_set_buf_byte(tx_buf, 2, error_code);
        return 3;
    }
}
