#ifndef _UTIL_H
#define _UTIL_H

#include "esp_system.h"
#include "math.h"

#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))
#define CLAMP(x,a,b) (MAX((a), MIN((x), (b))))
#define DEC(f) (uint32_t)(fabs(fmod(f * 100, 100.)))
#define INT(f) (int32_t)(f)

/* statically allocated arrays only */
#define N_ELEMENTS(arr) (sizeof (arr) / sizeof ((arr)[0]))

#endif /* _UTILS_H */
