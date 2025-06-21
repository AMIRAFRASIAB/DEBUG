

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
  
/*-----------------------------------------------------------------*/
/* Main Setting */
/*-----------------------------------------------------------------*/
#define debug_USE_SPRINTF_FORMATTER     YES     
#define debug_USE_LABLE                 YES     
#define debug_USE_TIMESTAMP             YES     
#define debug_TX_TOTAL_RAM              512     
#define debug_RX_ENGINE_ENABLE          YES     
#if (debug_RX_ENGINE_ENABLE==YES)
  #define debug_RX_TOTAL_RAM            32     
  #define debug_RX_TASK_PRIORITY        1    
  #define debug_RX_STACK_SIZE           512
#endif

/*-----------------------------------------------------------------*/
/* GPIO Setting */
/*-----------------------------------------------------------------*/
#define debug_TX_GPIO                   A                       
#define debug_TX_PIN                    2                       
#define debug_TX_AF                     7                       
#define debug_TX_PIN_LOCK               YES                     
               
#define debug_RX_GPIO                   A                       
#define debug_RX_PIN                    3                      
#define debug_RX_AF                     7                       
#define debug_RX_PIN_LOCK               YES                     

/*-----------------------------------------------------------------*/
/* USART Setting */
/*-----------------------------------------------------------------*/
#define debug_BAUD_RATE                 115200                  
#define debug_USART_CLOCK_DOMIN_Hz      42000000                
#define debug_USART_TYPE                USART                   
#define debug_USART_NUM                 2                       
#define debug_USART_IRQn                USART2_IRQn
#define debug_USART_IRQHandler          USART2_IRQHandler 
#define debug_UART_IRQ_PRIORITY         6

/*-----------------------------------------------------------------*/
/* DMA Setting */
/*-----------------------------------------------------------------*/
#define debug_DMA                      1                       
#define debug_DMA_STREAM               6                       
#define debug_DMA_CHANNEL              4                       
#define debug_DMA_IRQn                 DMA1_Stream6_IRQn
#define debug_DMA_IRQHandler           DMA1_Stream6_IRQHandler
#define debug_DMA_PRIORITY             LL_DMA_PRIORITY_LOW     



/** @} */



#ifdef __cplusplus 
  };
#endif //__cplusplus 
#endif