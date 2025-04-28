
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
  
  /* DMA  */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);
  NVIC_DisableIRQ(DMA1_Stream4_IRQn);
  LL_DMA_ClearFlag_DME4(DMA1);
  LL_DMA_ClearFlag_FE4(DMA1);
  LL_DMA_ClearFlag_TC4(DMA1);
  LL_DMA_ClearFlag_HT4(DMA1);
  LL_DMA_ClearFlag_TE4(DMA1);
  WRITE_REG(DMA1_Stream4->CR, 0UL);
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_4, LL_DMA_CHANNEL_4);
  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_4, LL_DMA_PRIORITY_MEDIUM);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_4, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_4, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_4, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_4, LL_DMA_MODE_NORMAL);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_4);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_4);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, 0x0000);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_4, LL_USART_DMA_GetRegAddr(UART4));
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_4);
  NVIC_ClearPendingIRQ(DMA1_Stream4_IRQn);
  NVIC_SetPriority(DMA1_Stream4_IRQn, DEBUG_DMA_IRQ_PRIORITY);
  NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* UART */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
  __DSB();
  LL_USART_Disable(UART4);
  extern uint32_t RCC_GetPCLK1ClockFreq(uint32_t HCLK_Frequency);
  LL_USART_SetTransferDirection(UART4, LL_USART_DIRECTION_TX_RX);
  LL_USART_SetBaudRate(UART4, RCC_GetPCLK1ClockFreq(SystemCoreClock), LL_USART_OVERSAMPLING_16, 115200);
  LL_USART_SetDataWidth(UART4, LL_USART_DATAWIDTH_8B);
  LL_USART_SetParity(UART4, LL_USART_PARITY_NONE);
  LL_USART_SetStopBitsLength(UART4, LL_USART_STOPBITS_1);
  LL_USART_ConfigAsyncMode(UART4);
  LL_USART_EnableDMAReq_TX(UART4);
  LL_USART_DisableIT_TXE(UART4);
  LL_USART_EnableIT_RXNE(UART4);
  LL_USART_ClearFlag_RXNE(UART4);
  NVIC_ClearPendingIRQ(UART4_IRQn);
  NVIC_SetPriority(UART4_IRQn, DEBUG_UART_IRQ_PRIORITY);
  NVIC_EnableIRQ(UART4_IRQn);
  LL_USART_Enable(UART4);
  __DSB();
  return true;
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_disable (void) {
  LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_enable (void) {
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_clearErrorFlags (void) {
  if (LL_DMA_IsActiveFlag_TE4(DMA1)) {
    LL_DMA_ClearFlag_TE4(DMA1);
    LL_DMA_ClearFlag_DME4(DMA1);
    LL_DMA_ClearFlag_FE4(DMA1);
  }
}
//-----------------------------------------------------------------------
__STATIC_INLINE uint16_t drv_hw_dma_get_ndtr (void) {
  return LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_4);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_set_ndtr (uint16_t ndtr) {
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, ndtr);
}
//-----------------------------------------------------------------------
__STATIC_INLINE uint32_t drv_hw_dma_get_tc_flag (void) {
  return LL_DMA_IsActiveFlag_TC4(DMA1);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_set_memory_address (uint32_t address) {
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_4, address);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_dma_clear_tc_flag (void) {
  LL_DMA_ClearFlag_TC4(DMA1);
}
//-----------------------------------------------------------------------
__STATIC_INLINE uint32_t drv_hw_dma_isStreamEnabled (void) {
  return LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_4);
}
//-----------------------------------------------------------------------
__STATIC_INLINE uint32_t drv_hw_uart_get_rxne_flag (void) {
  return LL_USART_IsActiveFlag_RXNE(UART4);
}
//-----------------------------------------------------------------------
__STATIC_INLINE void drv_hw_uart_clear_rxne_flag (void) {
  LL_USART_ClearFlag_RXNE(UART4);
}
//-----------------------------------------------------------------------
__STATIC_INLINE uint8_t drv_hw_uart_read_byte (void) {
  return LL_USART_ReceiveData8(UART4);
}

#endif //__SERIAL_DRV_H_INCLUDED__