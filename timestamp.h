#ifndef TIMESTAMP_H_
#define TIMESTAMP_H_

#include <stdint.h>

typedef uint32_t timestamp_t;

#define timestamp_schedule(duration, ref) ((timestamp_t)duration + (timestamp_t)ref)

/*
 * Check if a certain time has been reached, timestamp overflow is handled.
 * Behavior is guaranteed if the difference is less than 2^(bits(timestamp_t)-1) uS.
 */
#define timestamp_reached(ts, ref) ((timestamp_t)ts - (timestamp_t)ref < 0x7FFFFFFF)

/*
 * Return timestamp in microsecond (us)
 */
timestamp_t timestamp_get(void);

/*
 * Block the CPU for a given amount of time, in microsecond (us)
 */
void timestamp_block_us(timestamp_t duration);

#endif /* TIMESTAMP_H_ */
