

#ifndef __SERIAL_CONFIG_TEMPELATE_H_INCLUDED__
#define __SERIAL_CONFIG_TEMPELATE_H_INCLUDED__

#ifdef __cplusplus 
  extern "C"{
#endif //__cplusplus 

#include <stdint.h>
#include <stdbool.h>
#include "swo.h"

/**
 * @defgroup DebugConfig Debug Configuration
 * @brief Configuration settings for the debug interface.
 * @{
 */
  



/* DEBUG baud rate */
#define DEBUG_BAUD_RATE                 115200                  /**< Baud rate for the debug UART */
                
/* Formatter */               
#define DEBUG_USE_SPRINTF_FORMATTER     NO                      /**< Formatter usage permission */

/* Default RAM usage */
#define DEBUG_TX_TOTAL_RAM              512                     /**< Total available RAM for debug transmission (in bytes) */
#define DEBUG_RX_TOTAL_RAM              64                      /**< Total available RAM for debug reception (in bytes) */
                                                      
/* Task options */                                    
#define DEBUG_RX_TASK_PRIORITY          1                       /**< Priority of the debug RX task (lower value = higher priority) */
#define DEBUG_RX_STACK_SIZE             512                     /**< Stack size for the debug RX task (in bytes) */
                                                      
/* IRQ Priorities */                                  
#define DEBUG_UART_IRQ_PRIORITY         6                       /**< Priority of the UART interrupt */
                
/* STM32 Part Number */               
#define DEBUG_STM32_SERIES              f4                      /**< Select target STM32 device */
                
/* UART Tx Pin Configuration */               
#define DEBUG_UART_TX_GPIO              B                       /**< GPIO of the tx pin */
#define DEBUG_UART_TX_PIN               6                       /**< Pin number of tx */
#define DEBUG_UART_TX_AF                7                       /**< Alternate function map of tx pin */
#define DEBUG_TX_PIN_LOCK               YES                     /**< Lock the tx pin after initialization */
                
/* UART Rx Pin Configuration */               
#define DEBUG_UART_RX_GPIO              B                       /**< GPIO of the rx pin */
#define DEBUG_UART_RX_PIN               7                       /**< Pin number of rx */
#define DEBUG_UART_RX_AF                7                       /**< Alternate function map of rx pin */
#define DEBUG_RX_PIN_LOCK               YES                     /**< Lock the rx pin after initialization */
                
/* DMA Setting */               
#define DEBUG_DMA                       2                       /**< DMA module number */
#define DEBUG_DMA_USE_STREAM            YES                     /**< Does DMA use stream mechanism? */
                
#if(DEBUG_DMA_USE_STREAM == YES)                
  #define DEBUG_DMA_STREAM              7                       /**< DMA stream number */
#endif //(DEBUG_DMA_USE_STREAM == YES)

#define DEBUG_DMA_CHANNEL               4                       /**< DMA channel number */
#define DEBUG_DMA_PRIORITY              LL_DMA_PRIORITY_LOW     /**< DMA line priority */

#define DEBUG_USART_TYPE                USART                   /**< USART or UART peripheral is using */
#define DEBUG_USART_NUM                 1                       /**< USART Instance */
#define DEBUG_USART_CLOCK_DOMIN_Hz      84000000                /**< USART peripheral main clock */



/** @} */





#ifdef __cplusplus 
  };
#endif //__cplusplus 
#endif