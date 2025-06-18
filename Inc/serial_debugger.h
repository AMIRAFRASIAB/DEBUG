
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
void debug_getTimestamp14 (uint8_t* dst);
//--------------------------------------------------------------------------------------
/* Helper macros */
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
