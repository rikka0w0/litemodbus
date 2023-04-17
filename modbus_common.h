#ifndef MODBUS_COMMON_H_
#define MODBUS_COMMON_H_

#include <stdint.h>
#include "modbus_config.h"

#define MODBUS_INTERFRAME_GAP_US 3300      // uS
#define MODBUS_DEV_ADDR_CANSET(addr) (addr > 0 && addr <= 247)
#define MODBUS_DEV_ADDR_VALID(addr) (addr >= 0 && addr <= 247)

// Error Code
// No error, all good
#define MODBUS_ERR_NOERR        0x00    // No Error
// Function code received in the query is not recognized or allowed by slave
#define MODBUS_ERR_ILLEGAL_FUNC 0x01    // Illegal Function Code
// Data address of some or all the required entities are not allowed or do not exist in slave
#define MODBUS_ERR_ILLEGAL_ADDR 0x02    // Illegal Data Address
// Value is not accepted by slave
#define MODBUS_ERR_ILLEGAL_VAL  0x03    // Illegal Data Value
// Unrecoverable error occurred while slave was attempting to perform requested action
#define MODBUS_ERR_DEV_FAILURE  0x04    // Slave Device Failure

// Function codes
#define MODBUS_FUNC_READ_REG    03      // Read holding registers
#define MODBUS_FUNC_READ_INPUT  04      // Read (analog) inputs
#define MODBUS_FUNC_WRITE_1REG  06      // Write a holding register
#define MODBUS_FUNC_WRITE_REGS  16      // Write holding registers

// 16位空寄存器，读为0，写忽略
#define MODBUS_DATATYPE_INT16_GAP   0x00
// 16位寄存器，读写由指定函数处理，在main函数中执行读写操作
#define MODBUS_DATATYPE_INT16_DYN   0x01
// 16位寄存器，读写操作直接映射到内存，在main函数中执行读写操作
#define MODBUS_DATATYPE_INT16       0x02
// 32位寄存器，大端顺序，读写操作直接映射到内存，在main函数中执行读写操作
#define MODBUS_DATATYPE_INT32       0x03

#define MODBUS_DATATYPE_MASK        0x00FF

// Check and/or clamp the value before write
#define MODBUS_DATAFLAG_WRITE_CHECK 0x0100
// Use custom change detection, required if float or double is used.
#define MODBUS_DATAFLAG_EQUAL_CHECK 0x0200
// Notice when register has been updated
#define MODBUS_DATAFLAG_NOTICE_NEW  0x0400

// 标志modbus_reg_map_def_t数组的结尾，必须为最后一项
// Marks the end of a modbus_reg_map_def_t array, must be the last item.
#define MODBUS_DECLARE_END_MARK {.callback = 0, .mem = 0, .reg_count = 0, .data_type = MODBUS_DATATYPE_INT16_GAP}

// 一个用于计算指定类型占多少16位寄存器的宏
// A helper macro to determine the equivalent number of 16-bit registers of a given type
#define MODBUS_REG_COUNT(myType) (sizeof(myType)/sizeof(uint16_t))

// Access to "modbus_reg_map_def_t.mem"
#define MODBUS_MEM_U16(reg_def, index)      (((uint16_t*)reg_def->mem)[index])
#define MODBUS_MEM_U32(reg_def, index)      (((uint32_t*)reg_def->mem)[index >> 1])

typedef uint_least8_t modbus_cbret_t;
typedef uint_least8_t modbus_addr_t;
typedef uint_least8_t modbus_bool_t;
typedef uint_least8_t modbus_err_t;
typedef uint_least8_t modbus_size_t;

enum modbus_cb_action {
    /**
     * Handle read in MODBUS_DATATYPE_INT16_DYN.
     * Applies to MODBUS_DATATYPE_INT16_DYN only.
     * Set the result to the memory (uint16_t*) pointed by "val_ptr".
     * Return the error code if the read operation cannot be performed.
     */
    modbus_cb_read = 0,

    /**
     * Handle write in MODBUS_DATATYPE_INT16_DYN.
     * Applies to MODBUS_DATATYPE_INT16_DYN only.
     * "val_ptr" points to a "modbus_dyn_val_t", where its "val" field contains the new value.
     * If modification detection is used, set the "changed" field if there is an update.
     * Return the error code if the write operation cannot be performed.
     */
    modbus_cb_write,

    /**
     * Check if the new value is valid, optionally do clamping.
     * Applies to all.
     * Get called if MODBUS_DATAFLAG_WRITE_CHECK is specified in "reg_data_type".
     * "val_ptr" points to the new value. In case of clamping, set the result here.
     * Return MODBUS_ERR_NOERR if the write operation can be performed.
     * Return MODBUS_ERR_ILLEGAL_VAL if the new value is invalid.
     */
    modbus_cb_write_check,

    /**
     * Check if the new value is different from the current one.
     * Required for float and double fields, optional otherwise.
     * Get called if MODBUS_DATAFLAG_EQUAL_CHECK is specified in "reg_data_type".
     * "val_ptr" points to the new value.
     * Return non-zero if there is a change.
     *
     * An example for float comparison:
     *
     * modbus_cbret_t float_changed(enum modbus_cb_action action,
     *     modbus_cb_context_ptr context, void *val_ptr) {
     *
     *     float old = MODBUS_MEM_FLOAT(context->reg_def, context->reg_index);
     *     float new = *((float*)val_ptr);
     *     return fabsf(old - new) > 1e-6F;
     * }
     */
    modbus_cb_is_changed,

    /**
     * The register content has changed.
     * Applies to all.
     * "val_ptr" is not used.
     * Return value is ignored.
     */
    modbus_cb_modified,
};

typedef struct _modbus_cb_context_t const *modbus_cb_context_ptr;

typedef struct {
    uint16_t val;
    modbus_bool_t changed;
} modbus_dyn_val_t;

/*
 * Function prototype of "modbus_reg_map_def_t.callback".
 *
 * @param action    Specify the purpose of this function call.
 * @param context   Contains some necessary information, e.g. current Modbus address.
 * @param val_ptr   Point to a action-dependent memory location.
 * @return          error code, MODBUS_ERR_NOERR for no error.
 *                  In the case of "modbus_cb_is_changed", MODBUS_ERR_NOERR stands for no change.
 */
typedef modbus_cbret_t (*modbus_cb_func)(enum modbus_cb_action action, modbus_cb_context_ptr context, void *val_ptr);
// Give or take mutex
typedef void (*modbus_mutex_operation_t)(uint32_t mutex_mask);

/**
 * modbus_reg_map_def_t 为Modbus寄存器组定义。每个modbus_reg_map_def_t定义一类寄存器（例如
 * 保持寄存器和只读输入寄存器是两个modbus_reg_map_def_t定义）。一个modbus_reg_map_def_t可以
 * 定义多个寄存器，整个寄存器类型用一个modbus_reg_map_def_t数组定义，由一个{.mem=0, .reg_count=0}
 * 的条目作为结尾标志(reg_data_type的值不限)。Modbus的定义里面的modbus_dev_def需要指向相应的
 * modbus_reg_map_def_t数组。
 *
 * modbus_reg_map_def_t包含4个成员，其中3个有用。
 *
 * reg_data_type表示数据类型以及单个单元长度，MODBUS_DATATYPE_INT16_DYN和MODBUS_DATATYPE_INT16表示
 * 16位标准Modbus单元，区别是前者被读写时会调用用户函数，后者直接映射到内存。MODBUS_DATATYPE_INT32表示
 * 32位大端的Modbus单元，读写操作直接映射到内存。
 *
 * mem为一个指向被调用的用户函数（使用MODBUS_DATATYPE_INT16_DYN时）或者储存着单元数据的内存地址。
 * 当mem为用户函数时，函数声明必须符合modbus_cb_func。
 *
 * reg_count为modbus_reg_map_def_t所定义的单元占用16位标准单元的个数，MODBUS_DATATYPE_INT16_DYN和
 * MODBUS_DATATYPE_INT16时为单元数，MODBUS_DATATYPE_INT32时为单元数的2倍。
 */

typedef struct {
    /**
     * 1. MODBUS_DATATYPE_INT16_DYN is chosen: point to the callback function
     *    that handles the read and write operation.
     * 2. Not used in MODBUS_DATATYPE_INT16_GAP.
     * 3. Otherwise, this callback allows the user to validate the value to be written
     */
    modbus_cb_func callback;

    // The memory/ROM region that holds the data to be read
    void* mem;

    // Number of 16-bit registers
    uint16_t reg_count;

    /**
     * A combination of MODBUS_DATATYPE_XX and MODBUS_DATAFLAG_XX
     */
    uint16_t data_type;
#ifdef MODBUS_FREERTOS
    // Specify the bit in the mutex flag (31-bits available), 0 = no mutex
    uint_least8_t reg_mutex_flag_bit;
#endif
} modbus_reg_map_def_t ;

typedef struct {
#ifdef MODBUS_FREERTOS
    const modbus_mutex_operation_t mutex_take;
    const modbus_mutex_operation_t mutex_give;
#endif

    const modbus_reg_map_def_t* const holding_regs;  // 03(r) 06(w) 16(w)
    const modbus_reg_map_def_t* const input_regs;    // 04(r)
} modbus_dev_def_t ;

typedef struct _modbus_cb_context_t {
    const modbus_dev_def_t *const dev_def;

    // Current register definition
    const modbus_reg_map_def_t* reg_def;

    // The offset of the current register within the definition
    uint16_t reg_index;

    // Current Modbus address
    uint16_t reg_addr;

    // The first addressed Modbus cell in this request.
    uint16_t start_addr;

    // The Modbus address after the last addressed cell in this request.
    uint16_t end_addr;

    // Current Modbus function code
    const uint_least8_t func_code;
} modbus_cb_context_t;

const modbus_reg_map_def_t* modbus_get_reg_def(const modbus_dev_def_t* dev_def, uint_least8_t func_code);

#endif /* MODBUS_COMMON_H_ */
