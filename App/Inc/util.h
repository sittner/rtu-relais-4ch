#ifndef UTIL_H
#define UTIL_H

#define MS_PER_S  1000u
#define US_PER_MS 1000u
#define US_PER_S  1000000u

/* Integer division rounding up, for non-negative values */
#define DIV_CEIL(a, b) (((a) + (b) - 1) / (b))

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#endif
