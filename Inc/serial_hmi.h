
#ifndef __SERIAL_HMI_H_INCLUDED__
#define __SERIAL_HMI_H_INCLUDED__

#ifdef __cplusplus 
  extern "C"{
#endif //__cplusplus 

#include <stdint.h>
#include <stdbool.h>


void hmi_decoder (void* stream, uint16_t streamLen);

#ifdef __cplusplus 
  };
#endif //__cplusplus 
#endif