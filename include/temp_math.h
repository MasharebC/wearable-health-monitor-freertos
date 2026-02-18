#ifndef TEMP_MATH_H_
#define TEMP_MATH_H_

#include <stdint.h>

static inline float mlx_raw_to_celsius(uint16_t raw)
{
    return (raw * 0.02f) - 273.15f;
}

#endif // TEMP_MATH_H_
