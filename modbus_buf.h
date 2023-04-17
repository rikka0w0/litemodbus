#ifndef INCLUDE_MODBUS_BUF_H_
#define INCLUDE_MODBUS_BUF_H_

#include <stdint.h>

#if defined(__TMS320C2000__) && !defined(_UINT8_T_DECLARED)
// TI's C2000 platform, 16-bit char...
#include "modbus_buf16.h"
#else
#include "modbus_buf8.h"
#endif

#endif /* INCLUDE_MODBUS_BUF_H_ */
