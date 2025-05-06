

#ifndef __SERIAL_CONFIG_TEMPELATE_H_INCLUDED__
#define __SERIAL_CONFIG_TEMPELATE_H_INCLUDED__

#ifdef __cplusplus 
  extern "C"{
#endif //__cplusplus 

#include <stdint.h>
#include <stdbool.h>


/* Default RAM usage */
#define DEBUG_TX_BUF_LEN        64
#define DEBUG_RX_PACKET_LEN_MAX 32
/* Task options */
#define DEBUG_TX_TASK_PRIORITY  1
#define DEBUG_RX_TASK_PRIORITY  1
#define DEBUG_TX_STACK_SIZE     128
#define DEBUG_RX_STACK_SIZE     128
/* IRQ Priorities */
#define DEBUG_DMA_IRQ_PRIORITY  6
#define DEBUG_UART_IRQ_PRIORITY 6
/* IRQ Definitions */
#define DEBUG_UART_IRQHandler   USART1_IRQHandler
#define DEBUG_DMA_IRQHandler    DMA2_Stream7_IRQHandler





#ifdef __cplusplus 
  };
#endif //__cplusplus 
#endif