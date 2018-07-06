/**
@defgroup detour Detour

Members in this module are wrappers around the standard math library
*/

#ifndef DETOURMATH_H
#define DETOURMATH_H

#include <fix16.hpp>
#include <math.h>

inline Fix16 dtMathFabsf(Fix16 x) { return fabsf(x); }
inline Fix16 dtMathSqrtf(Fix16 x) { return sqrtf(x); }
inline Fix16 dtMathFloorf(Fix16 x) { return floorf(x); }
inline Fix16 dtMathCeilf(Fix16 x) { return ceilf(x); }
inline Fix16 dtMathCosf(Fix16 x) { return cosf(x); }
inline Fix16 dtMathSinf(Fix16 x) { return sinf(x); }
inline Fix16 dtMathAtan2f(Fix16 y, Fix16 x) { return atan2f(y, x); }

#endif
