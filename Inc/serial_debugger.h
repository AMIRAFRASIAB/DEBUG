
#ifndef __DEBUGGER_H_INCLUDED__
#define __DEBUGGER_H_INCLUDED__

#ifdef __cplusplus 
  extern "C"{
#endif //__cplusplus 

#include <stdint.h>
#include <stdbool.h>
#include "serial_debugger.h"
#include "serial_config.h"

//--------------------------------------------------------------------------------------
/* Debug configuration structure */
typedef struct {
  uint8_t       level;
} DebugConfig_t;
extern DebugConfig_t debugConf;
//--------------------------------------------------------------------------------------
/* Public APIs */
bool debug_init (void);
bool debug_transmit (uint8_t level, uint8_t argLen, const char* FORMAT, ...);
//--------------------------------------------------------------------------------------
/* Helper macros */
#define __COUNT(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, N, ...)\
                                    N 
#define COUNT(...)                  __COUNT(__VA_ARGS__ ,20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1)
#define __LOG(level, argLen, ...)   debug_transmit (level, argLen, __VA_ARGS__)
//--------------------------------------------------------------------------------------
/* Main interface */
#define LOG_TRACE(...)              debug_transmit(0, COUNT(__VA_ARGS__), __VA_ARGS__)
#define LOG_INFO(...)               debug_transmit(1, COUNT(__VA_ARGS__), __VA_ARGS__)
#define LOG_WARNING(...)            debug_transmit(2, COUNT(__VA_ARGS__), __VA_ARGS__)
#define LOG_ERROR(...)              debug_transmit(3, COUNT(__VA_ARGS__), __VA_ARGS__)
#define LOG_FATAL(...)              debug_transmit(4, COUNT(__VA_ARGS__), __VA_ARGS__)

#ifdef __cplusplus 
  };
#endif //__cplusplus 
#endif
