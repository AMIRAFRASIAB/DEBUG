
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

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"

/**
 * @brief Initializes the GPIO, DMA, and UART peripherals for serial communication.
 * 
 * This function configures the necessary hardware for serial communication using USART and DMA:
 * - Enables the clock for GPIO, DM, and USART.
 * - Configures GPIO pins
 * - Configures DMA for data transfer from memory to USART.
 * - Sets up the UART peripheral for asynchronous communication, including configuring baud rate, data width, parity, and stop bits.
 * - Configures DMA for UART transmission, enabling the DMA request for transmission.
 * - Enables the UART interrupts for receive data available (RXNE) and clears any pending interrupt flags.
 * 
 * @note This function also configures interrupt priorities for USART and DMA, ensuring proper interrupt handling during data transmission and reception.
 * 
 * @return true if the driver initialization was successful, false otherwise.
 */
__STATIC_INLINE bool drv_hw_driver_init (void) {
  /* GPIO */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  __DSB();
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_7);
  LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_7);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_MEDIUM);
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_MEDIUM);
  /* DMA  */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
  NVIC_DisableIRQ(DMA2_Stream7_IRQn);
  LL_DMA_ClearFlag_DME7(DMA2);
  LL_DMA_ClearFlag_FE7(DMA2);
  LL_DMA_ClearFlag_TC7(DMA2);
  LL_DMA_ClearFlag_HT7(DMA2);
  LL_DMA_ClearFlag_TE7(DMA2);
  WRITE_REG(DMA2_Stream7->CR, 0UL);
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4);
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_MEDIUM);
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, 0x0000);
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7, LL_USART_DMA_GetRegAddr(USART1));
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_7);
  /* UART */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  __DSB();
  LL_USART_Disable(USART1);
  extern uint32_t RCC_GetPCLK2ClockFreq(uint32_t HCLK_Frequency);
  LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
  LL_USART_SetBaudRate(USART1, RCC_GetPCLK2ClockFreq(SystemCoreClock), LL_USART_OVERSAMPLING_16, DEBUG_BAUD_RATE);
  LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
  LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
  LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_EnableDMAReq_TX(USART1);
  LL_USART_DisableIT_TXE(USART1);
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_ClearFlag_RXNE(USART1);
  NVIC_ClearPendingIRQ(USART1_IRQn);
  NVIC_SetPriority(USART1_IRQn, DEBUG_UART_IRQ_PRIORITY);
  NVIC_EnableIRQ(USART1_IRQn);
  LL_USART_Enable(USART1);
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
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
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
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
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
  if (LL_DMA_IsActiveFlag_TE7(DMA2)) {
    LL_DMA_ClearFlag_TE7(DMA2);
    LL_DMA_ClearFlag_DME7(DMA2);
    LL_DMA_ClearFlag_FE7(DMA2);
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
  return LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_7);
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
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, ndtr);
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
  return LL_DMA_IsActiveFlag_TC7(DMA2);
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
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, address);
}
//-----------------------------------------------------------------------
/**
 * @brief Clears the transfer complete flag for DMA.
 * 
 * This function clears the transfer complete (TC) flag for DMA.
 * indicating that the current transfer is complete and ready for the next one.
 */
__STATIC_INLINE void drv_hw_dma_clear_tc_flag (void) {
  LL_DMA_ClearFlag_TC7(DMA2);
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
  return LL_USART_IsActiveFlag_RXNE(USART1);
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
  LL_USART_ClearFlag_RXNE(USART1);
}
//-----------------------------------------------------------------------
/**
 * @brief Reads one byte from the UART receive data register.
 *
 * This function returns the next received byte from the USART1 data register.
 * It should be called only when the RXNE flag is set, indicating that data is available.
 *
 * @return The received byte.
 */
__STATIC_INLINE uint8_t drv_hw_uart_read_byte (void) {
  return LL_USART_ReceiveData8(USART1);
}

#endif //__SERIAL_DRV_H_INCLUDED__