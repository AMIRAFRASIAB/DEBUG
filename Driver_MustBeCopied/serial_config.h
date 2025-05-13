

#ifndef __SERIAL_CONFIG_TEMPELATE_H_INCLUDED__
#define __SERIAL_CONFIG_TEMPELATE_H_INCLUDED__

#ifdef __cplusplus 
  extern "C"{
#endif //__cplusplus 

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup DebugConfig Debug Configuration
 * @brief Configuration settings for the debug interface.
 * @{
 */




/* DEBUG baud rate */
#define DEBUG_BAUD_RATE         230400 /**< Baud rate for the debug UART */

/* Default RAM usage */
#define DEBUG_TX_TOTAL_RAM      4096   /**< Total available RAM for debug transmission (in bytes) */
#define DEBUG_RX_TOTAL_RAM      32     /**< Total available RAM for debug reception (in bytes) */
                                       
/* Task options */                     
#define DEBUG_RX_TASK_PRIORITY  1      /**< Priority of the debug RX task (lower value = higher priority) */
#define DEBUG_RX_STACK_SIZE     512    /**< Stack size for the debug RX task (in bytes) */
                                       
/* IRQ Priorities */                   
#define DEBUG_UART_IRQ_PRIORITY 6      /**< Priority of the UART interrupt */

/* IRQ Definitions */
#define DEBUG_UART_IRQHandler   USART1_IRQHandler /**< UART IRQ handler for debug UART */
#define DEBUG_DMA_IRQHandler    DMA2_Stream7_IRQHandler /**< DMA IRQ handler for debug DMA */

/** @} */





#ifdef __cplusplus 
  };
#endif //__cplusplus 
#endif