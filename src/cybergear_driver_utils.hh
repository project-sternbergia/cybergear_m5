#ifndef CYBERGEAR_DRIVER_UTILS_HH
#define CYBERGEAR_DRIVER_UTILS_HH

// #define CG_DEBUG
#ifndef CG_DEBUG
#define CG_DEBUG_FUNC
#define CG_DEBUG_PRINTF(fmt, ...)
#define CG_DEBUG_PRINTLN(msg)
#else
#include <M5Stack.h>
#undef min
#undef max
#define CG_DEBUG_FUNC Serial.printf("l%d %s\n", __LINE__, __func__);
#define CG_DEBUG_PRINTF(fmt, ...) Serial.printf(fmt, __VA_ARGS__);
#define CG_DEBUG_PRINTLN(msg) Serial.println(msg);
#endif

#endif  // CYBERGEAR_DRIVER_UTILS_HH
