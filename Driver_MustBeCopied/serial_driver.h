
#ifndef __SERIAL_DRV_H_INCLUDED__
#define __SERIAL_DRV_H_INCLUDED__

#include "serial_debugger.h"
#include "serial_config.h"

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"

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
__STATIC_INLINE void drv_hw_dma_disable (void) {
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_enable (void) {
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_clearErrorFlags (void) {
  if (LL_DMA_IsActiveFlag_TE7(DMA2)) {
    LL_DMA_ClearFlag_TE7(DMA2);
    LL_DMA_ClearFlag_DME7(DMA2);
    LL_DMA_ClearFlag_FE7(DMA2);
  }
}
//-----------------------------------------------------------------------
__STATIC_INLINE uint16_t drv_hw_dma_get_ndtr (void) {
  return LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_7);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_set_ndtr (uint16_t ndtr) {
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, ndtr);
}
//-----------------------------------------------------------------------
__STATIC_INLINE uint32_t drv_hw_dma_get_tc_flag (void) {
  return LL_DMA_IsActiveFlag_TC7(DMA2);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_set_memory_address (uint32_t address) {
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, address);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_clear_tc_flag (void) {
  LL_DMA_ClearFlag_TC7(DMA2);
}
//-----------------------------------------------------------------------
__STATIC_INLINE uint32_t drv_hw_uart_get_rxne_flag (void) {
  return LL_USART_IsActiveFlag_RXNE(USART1);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_uart_clear_rxne_flag (void) {
  LL_USART_ClearFlag_RXNE(USART1);
}
//-----------------------------------------------------------------------
__STATIC_INLINE uint8_t drv_hw_uart_read_byte (void) {
  return LL_USART_ReceiveData8(USART1);
}

#endif //__SERIAL_DRV_H_INCLUDED__