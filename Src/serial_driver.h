
/**
 * @file serial_drv.h
 * @brief Driver functions for initializing and handling UART and DMA for serial communication.
 *
 * This file contains inline functions to initialize the UART and DMA for serial communication on an STM32
 * microcontroller. These functions provide low-level access to hardware peripherals for debugging and data
 * transmission.
 *
 * The functions configure the UART peripheral for transmitting and receiving data, configure DMA for 
 * optimized data transfer, and manage error flags. These operations are critical for setting up a serial
 * debugger or communication interface using UART and DMA.
 */
 
#ifndef __SERIAL_DRV_H_INCLUDED__
#define __SERIAL_DRV_H_INCLUDED__

#include "serial_debugger.h"
#include "serial_config.h"


#include TO_STRING(CONCAT(stm32, DEBUG_STM32_SERIES, xx_ll_bus.h))
#include TO_STRING(CONCAT(stm32, DEBUG_STM32_SERIES, xx_ll_gpio.h))
#include TO_STRING(CONCAT(stm32, DEBUG_STM32_SERIES, xx_ll_usart.h))
#include TO_STRING(CONCAT(stm32, DEBUG_STM32_SERIES, xx_ll_dma.h))
#include TO_STRING(CONCAT(stm32, DEBUG_STM32_SERIES, xx_hal_rcc.h))

#define __DEBUG_UART_GPIO_TX_CLK_ENABLE()         CONCAT(CONCAT(__HAL_RCC_GPIO, DEBUG_UART_TX_GPIO), _CLK_ENABLE)()
#define __DEBUG_UART_GPIO_RX_CLK_ENABLE()         CONCAT(CONCAT(__HAL_RCC_GPIO, DEBUG_UART_RX_GPIO), _CLK_ENABLE)()
#define __DEBUG_DMA_CLK_ENABLE()                  CONCAT(CONCAT(__HAL_RCC_DMA, DEBUG_DMA), _CLK_ENABLE)()
     
#define __DEBUG_TX_GPIO                           CONCAT(GPIO, DEBUG_UART_TX_GPIO)
#define __DEBUG_RX_GPIO                           CONCAT(GPIO, DEBUG_UART_RX_GPIO)
     
#define __DEBUG_TX_PIN                            CONCAT(LL_GPIO_PIN_, DEBUG_UART_TX_PIN)
#define __DEBUG_RX_PIN                            CONCAT(LL_GPIO_PIN_, DEBUG_UART_RX_PIN)
     
#define __DEBUG_TX_AF                             CONCAT(LL_GPIO_AF_, DEBUG_UART_TX_AF)
#define __DEBUG_RX_AF                             CONCAT(LL_GPIO_AF_, DEBUG_UART_RX_AF)
     
#define __DEBUG_DMAx                              CONCAT(DMA, DEBUG_DMA)

#if (DEBUG_DMA_USE_STREAM == YES)
  #define __DEBUG_DMA_LINE                        CONCAT(LL_DMA_STREAM_, DEBUG_DMA_STREAM)
  #define __DEBUG_DMA_CHANNEL                     CONCAT(LL_DMA_CHANNEL_, DEBUG_DMA_CHANNEL)
  #define __DEBUG_DMA_DISABLE_LINE()              LL_DMA_DisableStream(__DEBUG_DMAx, __DEBUG_DMA_LINE)
  #define __DEBUG_DMA_ENABLE_LINE()               LL_DMA_EnableStream(__DEBUG_DMAx, __DEBUG_DMA_LINE)
  #define __DEBUG_NVIC_IRQn                       CONCAT(DMA, CONCAT(CONCAT(CONCAT(DEBUG_DMA, _Stream), DEBUG_DMA_STREAM), _IRQn))
  #define __DEBUG_DMA_ClearFlag_DME()             CONCAT(LL_DMA_ClearFlag_DME, DEBUG_DMA_STREAM)(__DEBUG_DMAx)
  #define __DEBUG_DMA_ClearFlag_FE()              CONCAT(LL_DMA_ClearFlag_FE,  DEBUG_DMA_STREAM)(__DEBUG_DMAx)
  #define __DEBUG_DMA_ClearFlag_TC()              CONCAT(LL_DMA_ClearFlag_TC,  DEBUG_DMA_STREAM)(__DEBUG_DMAx)
  #define __DEBUG_DMA_ClearFlag_HT()              CONCAT(LL_DMA_ClearFlag_HT,  DEBUG_DMA_STREAM)(__DEBUG_DMAx)
  #define __DEBUG_DMA_ClearFlag_TE()              CONCAT(LL_DMA_ClearFlag_TE,  DEBUG_DMA_STREAM)(__DEBUG_DMAx)
  #define __DEBUG_DMA_SetLinePriorityLevel(...)   LL_DMA_SetStreamPriorityLevel(__VA_ARGS__)
  #define __DEBUG_DMA_IsActiveFlag_TE()           CONCAT(LL_DMA_IsActiveFlag_TE, DEBUG_DMA_STREAM)(__DEBUG_DMAx)
  #define __DEBUG_DMA_IsActiveFlag_TC()           CONCAT(LL_DMA_IsActiveFlag_TC, DEBUG_DMA_STREAM)(__DEBUG_DMAx)
  #define DEBUG_DMA_IRQHandler                    CONCAT(DMA, CONCAT(DEBUG_DMA, _Stream, DEBUG_DMA_STREAM, _IRQHandler))
#else
  #define __DEBUG_DMA_LINE                        CONCAT(LL_DMA_CHANNEL_, DEBUG_DMA_CHANNEL)
  #define __DEBUG_DMA_DISABLE_LINE()              LL_DMA_DisableChannel(__DEBUG_DMAx, __DEBUG_DMA_LINE)
  #define __DEBUG_DMA_ENABLE_LINE()               LL_DMA_EnableChannel(__DEBUG_DMAx, __DEBUG_DMA_LINE)
  #define __DEBUG_NVIC_IRQn                       CONCAT(DMA, CONCAT(CONCAT(CONCAT(DEBUG_DMA, _Channel), DEBUG_DMA_CHANNEL), _IRQn))
  #define __DEBUG_DMA_ClearFlag_DME()             CONCAT(LL_DMA_ClearFlag_DME, DEBUG_DMA_CHANNEL)(__DEBUG_DMAx)
  #define __DEBUG_DMA_ClearFlag_FE()              CONCAT(LL_DMA_ClearFlag_FE,  DEBUG_DMA_CHANNEL)(__DEBUG_DMAx)
  #define __DEBUG_DMA_ClearFlag_TC()              CONCAT(LL_DMA_ClearFlag_TC,  DEBUG_DMA_CHANNEL)(__DEBUG_DMAx)
  #define __DEBUG_DMA_ClearFlag_HT()              CONCAT(LL_DMA_ClearFlag_HT,  DEBUG_DMA_CHANNEL)(__DEBUG_DMAx)
  #define __DEBUG_DMA_ClearFlag_TE()              CONCAT(LL_DMA_ClearFlag_TE,  DEBUG_DMA_CHANNEL)(__DEBUG_DMAx)
  #define __DEBUG_DMA_SetLinePriorityLevel(...)   LL_DMA_SetChannelPriorityLevel(__VA_ARGS__)
  #define __DEBUG_DMA_IsActiveFlag_TE()           CONCAT(LL_DMA_IsActiveFlag_TE, DEBUG_DMA_CHANNEL)(__DEBUG_DMAx)
  #define __DEBUG_DMA_IsActiveFlag_TC()           CONCAT(LL_DMA_IsActiveFlag_TC, DEBUG_DMA_CHANNEL)(__DEBUG_DMAx)
  #define DEBUG_DMA_IRQHandler                    CONCAT(DMA, CONCAT(DEBUG_DMA, _Stream, DEBUG_DMA_CHANNEL, _IRQHandler)
#endif //(DEBUG_DMA_USE_STREAM == YES)

#define DEBUG_UART_IRQHandler                     CONCAT(DEBUG_USART_TYPE, CONCAT(DEBUG_USART_NUM, _IRQHandler))
#define __DEBUG_USARTx                            CONCAT(DEBUG_USART_TYPE, DEBUG_USART_NUM)
#define __DEBUG_USARTx_CLK_ENABLE()               CONCAT(CONCAT(CONCAT(__HAL_RCC_, DEBUG_USART_TYPE), DEBUG_USART_NUM), _CLK_ENABLE)()
#define __DEBUG_USARTx_IRQn                       CONCAT(DEBUG_USART_TYPE, CONCAT(DEBUG_USART_NUM, _IRQn))


__STATIC_INLINE bool drv_hw_driver_init (void) {
  /* ------------------------------------------------------------------------*/
  /* GPIO :: Clock Enable */
  __DEBUG_UART_GPIO_RX_CLK_ENABLE();
  __DEBUG_UART_GPIO_TX_CLK_ENABLE();
  __DSB();
  /* GPIO :: Mode Alternate */
  LL_GPIO_SetPinMode(__DEBUG_TX_GPIO, __DEBUG_TX_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinMode(__DEBUG_RX_GPIO, __DEBUG_RX_PIN, LL_GPIO_MODE_ALTERNATE);
  /* GPIO :: AF Mapping */
  #if DEBUG_UART_TX_AF <= 7
  LL_GPIO_SetAFPin_0_7(__DEBUG_TX_GPIO, __DEBUG_TX_PIN, __DEBUG_TX_AF);
  #else 
  LL_GPIO_SetAFPin_8_15(__DEBUG_TX_GPIO, __DEBUG_TX_PIN, __DEBUG_TX_AF);
  #endif
  #if DEBUG_UART_RX_AF <= 7
  LL_GPIO_SetAFPin_0_7(__DEBUG_RX_GPIO, __DEBUG_RX_PIN, __DEBUG_RX_AF);
  #else 
  LL_GPIO_SetAFPin_8_15(__DEBUG_RX_GPIO, __DEBUG_RX_PIN, __DEBUG_RX_AF);
  #endif
  /* GPIO :: Pin Lock Setting */
  #if DEBUG_TX_PIN_LOCK == YES
  LL_GPIO_LockPin(__DEBUG_TX_GPIO, __DEBUG_TX_PIN);
  #endif
  #if DEBUG_RX_PIN_LOCK == YES
  LL_GPIO_LockPin(__DEBUG_RX_GPIO, __DEBUG_RX_PIN);
  #endif
  /* ------------------------------------------------------------------------*/
  /* DMA :: Clock Enable */
  __DEBUG_DMA_CLK_ENABLE();
  __DSB();
  /* DMA :: Disable Line */
  __DEBUG_DMA_DISABLE_LINE();
  NVIC_DisableIRQ(__DEBUG_NVIC_IRQn);
  /* DMA :: Clear All Flags */
  __DEBUG_DMA_ClearFlag_DME();
  __DEBUG_DMA_ClearFlag_FE();
  __DEBUG_DMA_ClearFlag_TC();
  __DEBUG_DMA_ClearFlag_HT();
  __DEBUG_DMA_ClearFlag_TE();
  /* DMA :: Channel Selection */
  LL_DMA_SetChannelSelection(__DEBUG_DMAx, __DEBUG_DMA_LINE, __DEBUG_DMA_CHANNEL);
  /* DMA :: Line Priority */
  __DEBUG_DMA_SetLinePriorityLevel(__DEBUG_DMAx, __DEBUG_DMA_LINE, DEBUG_DMA_PRIORITY);
  /* DMA :: Memory and Periph Sizes */
  LL_DMA_SetMemorySize(__DEBUG_DMAx, __DEBUG_DMA_LINE, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_SetPeriphSize(__DEBUG_DMAx, __DEBUG_DMA_LINE, LL_DMA_PDATAALIGN_BYTE);
  /* DMA :: Memory and Periph Inc Mode */
  LL_DMA_SetMemoryIncMode(__DEBUG_DMAx, __DEBUG_DMA_LINE, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphIncMode(__DEBUG_DMAx, __DEBUG_DMA_LINE, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMode(__DEBUG_DMAx, __DEBUG_DMA_LINE, LL_DMA_MODE_NORMAL);
  /* DMA :: Transfer Direction */
  LL_DMA_SetDataTransferDirection(__DEBUG_DMAx, __DEBUG_DMA_LINE, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetDataLength(__DEBUG_DMAx, __DEBUG_DMA_LINE, 0x0000);
  LL_DMA_SetPeriphAddress(__DEBUG_DMAx, __DEBUG_DMA_LINE, LL_USART_DMA_GetRegAddr(__DEBUG_USARTx));
  /* ------------------------------------------------------------------------*/
  /* USART :: Clock Enable */
  __DEBUG_USARTx_CLK_ENABLE();
  __DSB();
  LL_USART_Disable(__DEBUG_USARTx);
  LL_USART_SetTransferDirection(__DEBUG_USARTx, LL_USART_DIRECTION_TX_RX);
  LL_USART_SetBaudRate(__DEBUG_USARTx, DEBUG_USART_CLOCK_DOMIN_Hz, LL_USART_OVERSAMPLING_16, DEBUG_BAUD_RATE);
  LL_USART_SetDataWidth(__DEBUG_USARTx, LL_USART_DATAWIDTH_8B);
  LL_USART_SetParity(__DEBUG_USARTx, LL_USART_PARITY_NONE);
  LL_USART_SetStopBitsLength(__DEBUG_USARTx, LL_USART_STOPBITS_1);
  LL_USART_ConfigAsyncMode(__DEBUG_USARTx);
  LL_USART_EnableDMAReq_TX(__DEBUG_USARTx);
  LL_USART_DisableIT_TXE(__DEBUG_USARTx);
  LL_USART_EnableIT_RXNE(__DEBUG_USARTx);
  LL_USART_ClearFlag_RXNE(__DEBUG_USARTx);
  NVIC_ClearPendingIRQ(__DEBUG_USARTx_IRQn);
  NVIC_SetPriority(__DEBUG_USARTx_IRQn, DEBUG_UART_IRQ_PRIORITY);
  NVIC_EnableIRQ(__DEBUG_USARTx_IRQn);
  LL_USART_Enable(__DEBUG_USARTx);
  __DSB();
  return true;
}
//-----------------------------------------------------------------------
/**
 * @brief Disables the DMA stream for UART transmission.
 * 
 * This function disables the DMA stream used for UART transmission, 
 * halting any ongoing data transfer.
 * 
 * @note This function is typically used to stop data transmission via DMA and 
 *       should be called before reconfiguring the DMA settings or deactivating the DMA controller.
 */
__STATIC_INLINE void drv_hw_dma_disable (void) {
  __DEBUG_DMA_DISABLE_LINE();
}
//-----------------------------------------------------------------------
/**
 * @brief Enables the DMA stream for UART transmission.
 * 
 * This function enables the DMA stream used for UART transmission, 
 * allowing data to be transferred from memory to the UART peripheral.
 * 
 * @note This function is typically called after configuring the DMA settings for UART 
 *       to start the data transfer via DMA.
 */
__STATIC_INLINE void drv_hw_dma_enable (void) {
  __DEBUG_DMA_ENABLE_LINE();
}
//-----------------------------------------------------------------------
/**
 * @brief Clears DMA error flags
 * 
 * This function checks if any error flags are set for DMA Stream (used for UART 
 * transmission) and clears the corresponding flags to reset the DMA error state. 
 * 
 * @note This function should be called when an error occurs during DMA data transfer 
 *       to clear the error flags and prepare the DMA for the next transfer attempt.
 */
__STATIC_INLINE void drv_hw_dma_clearErrorFlags (void) {
  if (__DEBUG_DMA_IsActiveFlag_TE()) {
    __DEBUG_DMA_ClearFlag_TE();
    __DEBUG_DMA_ClearFlag_DME();
    __DEBUG_DMA_ClearFlag_FE();
  }
}
//-----------------------------------------------------------------------
/**
 * @brief Retrieves the current data length for DMA Stream
 * 
 * This function returns the number of remaining data items to be transferred 
 * by DMA Stream. The NDTR (Number of Data to Transfer) register holds this value, 
 * indicating how many data items are left in the DMA transfer queue.
 * 
 * @return uint16_t The number of data items remaining in the DMA transfer.
 * 
 * @note This value is updated during the DMA transfer and is useful for tracking 
 *       progress or checking the transfer status.
 */
__STATIC_INLINE uint16_t drv_hw_dma_get_ndtr (void) {
  return LL_DMA_GetDataLength(__DEBUG_DMAx, __DEBUG_DMA_LINE);
}
//-----------------------------------------------------------------------
/**
 * @brief Sets the data length for DMA Stream
 * 
 * This function sets the NDTR (Number of Data to Transfer) register for DMA 
 * Stream. The NDTR register specifies the number of data items to be transferred 
 * in the current DMA operation. This value should be set before enabling the DMA 
 * stream to control how many data items will be transferred.
 * 
 * @param ndtr The number of data items to be transferred by DMA
 * 
 * @note It is important to configure the NDTR register correctly before starting 
 *       the DMA transfer to avoid transfer errors or misbehaviors.
 */
__STATIC_INLINE void drv_hw_dma_set_ndtr (uint16_t ndtr) {
  LL_DMA_SetDataLength(__DEBUG_DMAx, __DEBUG_DMA_LINE, ndtr);
}
//-----------------------------------------------------------------------
/**
 * @brief Checks if the transfer complete flag for DMA is set.
 * 
 * This function checks if the Transfer Complete (TC) flag is set for DMA Stream.
 * The TC flag is set when the DMA transfer has completed successfully.
 * 
 * @return Returns a non-zero value if the TC flag is set, 0 otherwise.
 */
__STATIC_INLINE uint32_t drv_hw_dma_get_tc_flag (void) {
  return __DEBUG_DMA_IsActiveFlag_TC();
}
//-----------------------------------------------------------------------
/**
 * @brief Sets the memory address for DMA.
 * 
 * This function sets the memory address for the DMA data transfer. 
 * The address is where the data will be read from or written to during the DMA transfer.
 * 
 * @param address The memory address to set for the DMA transfer.
 */
__STATIC_INLINE void drv_hw_dma_set_memory_address (uint32_t address) {
  LL_DMA_SetMemoryAddress(__DEBUG_DMAx, __DEBUG_DMA_LINE, address);
}
//-----------------------------------------------------------------------
/**
 * @brief Clears the transfer complete flag for DMA.
 * 
 * This function clears the transfer complete (TC) flag for DMA.
 * indicating that the current transfer is complete and ready for the next one.
 */
__STATIC_INLINE void drv_hw_dma_clear_tc_flag (void) {
  __DEBUG_DMA_ClearFlag_TC();
}
//-----------------------------------------------------------------------
/**
 * @brief Checks if the UART receive data register is not empty (RXNE flag).
 * 
 * This function checks whether the UART receive data register is not empty 
 * by evaluating the RXNE flag. When this flag is set, data is available in 
 * the receive register and can be read.
 * 
 * @return 1 if the RXNE flag is set, indicating that the receive data register 
 *         is not empty; otherwise, 0.
 */
__STATIC_INLINE uint32_t drv_hw_uart_get_rxne_flag (void) {
  return LL_USART_IsActiveFlag_RXNE(__DEBUG_USARTx);
}
//-----------------------------------------------------------------------
/**
 * @brief Clears the UART receive data register not empty (RXNE) flag.
 * 
 * This function clears the RXNE flag in the USART status register. The RXNE 
 * flag is set when data is received in the UART data register. Clearing this 
 * flag indicates that the data has been read and is ready for the next 
 * reception.
 */
__STATIC_INLINE void drv_hw_uart_clear_rxne_flag (void) {
  LL_USART_ClearFlag_RXNE(__DEBUG_USARTx);
}
//-----------------------------------------------------------------------
/**
 * @brief Reads one byte from the UART receive data register.
 *
 * This function returns the next received byte from the __DEBUG_USARTx data register.
 * It should be called only when the RXNE flag is set, indicating that data is available.
 *
 * @return The received byte.
 */
__STATIC_INLINE uint8_t drv_hw_uart_read_byte (void) {
  return LL_USART_ReceiveData8(__DEBUG_USARTx);
}

#endif //__SERIAL_DRV_H_INCLUDED__