#include "timestamp.h"

void timestamp_block_us(timestamp_t duration) {
    timestamp_t end = timestamp_get() + duration;
    while (timestamp_reached(end, timestamp_get()));
}

