
/* وَ نُرِیدُ أَنْ نَمُنَّ عَلَی الَّذِینَ اسْتُضْعِفُوا فِی الْأَرْضِ وَ نَجْعَلَهُمْ أَئِمَّةً وَ نَجْعَلَهُمُ الْوارِثِینَ */

#include "serial_debugger.h"
#include "serial_config.h"
#include "serial_driver.h"
#include "serial_hmi.h"
#include "swo.h"

#include "stdint.h"
#include "stdbool.h"
#include  "string.h"
#include "stdarg.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "stream_buffer.h"


//------------------------------------------------------------------------
DebugConfig_t debugConf = {
  .level = 0,
};
//------------------------------------------------------------------------
#define ms1             (12 - 1)
#define ms10            (11 - 1)
#define ms100           (10 - 1)
#define S1              (8  - 1)
#define S10             (7  - 1)
#define M1              (5  - 1)
#define M10             (4  - 1)
#define H1              (2  - 1)
#define H10             (1  - 1)
//------------------------------------------------------------------------
typedef struct {
  uint32_t           index[2];
  SemaphoreHandle_t  mutex;
  uint8_t            buf[2][debug_TX_TOTAL_RAM / 2];
  uint8_t            active;
} Tx_s;
//------------------------------------------------------------------------
static const uint8_t  __FOOTER[]          = "\n"     ;
#if debug_USE_LABLE == YES
static const uint8_t  __TRACE_LABEL[]     = "<TRC> " ;
static const uint8_t  __INFO_LABEL[]      = "<INF> " ;
static const uint8_t  __WARNING_LABEL[]   = "<WRN> " ;
static const uint8_t  __ERROR_LABEL[]     = "<ERR> " ;
static const uint8_t  __FATAL_LABEL[]     = "<FTL> " ;
static const uint8_t* LOG_LEVEL_STRING[]  = {__TRACE_LABEL, __INFO_LABEL, __WARNING_LABEL, __ERROR_LABEL, __FATAL_LABEL};
#endif //debug_USE_LABLE
//------------------------------------------------------------------------
#if debug_USE_TIMESTAMP == YES
static char                  ts[]        = "00:00:00:000 ";
static TimerHandle_t         hTsTimer    = NULL; 
static QueueHandle_t         tsMailBox   = NULL;
#endif //debug_USE_TIMESTAMP
static TaskHandle_t          hTaskRx     = NULL;
static StreamBufferHandle_t  streamRx    = NULL;
static Tx_s                  tx          = {0};
//------------------------------------------------------------------------
static void vTimerCallback (TimerHandle_t xTimer);
static void serviceDebugRx (void* const pvParameters);
//------------------------------------------------------------------------
/**
 * @brief Copies data into the debug transmission buffer.
 *
 * This function copies `len` bytes of data into the active transmission buffer
 * (`tx.buf[tx.active]`) at the current position (`tx.index[tx.active]`).
 * It ensures that the copy operation does not exceed the remaining available space
 * in the buffer. The buffer index is updated after the copy.
 *
 * @param DATA Pointer to the data to be copied into the buffer.
 * @param len  Number of bytes to copy. The actual number of bytes copied may be
 *             less if there is not enough free space in the buffer.
 */
void __debug_copyFrom (const void* DATA, uint32_t len) {
  uint32_t freeSpace = (debug_TX_TOTAL_RAM / 2) - tx.index[tx.active];
  len = (freeSpace >= len)? len : freeSpace;
  memcpy(tx.buf[tx.active] + tx.index[tx.active], DATA, len);
  tx.index[tx.active] += len;
}
//------------------------------------------------------------------------
/**
 * @brief Callback function to update the timestamp on each timer tick.
 *
 * This function increments the timestamp value stored in the `ts` array,
 * which tracks time in a format resembling `HH:MM:SS:ms100:ms10:ms1`.
 * The function updates the timestamp and rolls over each component as necessary:
 * - ms1 (milliseconds)
 * - ms10 (tens of milliseconds)
 * - ms100 (hundreds of milliseconds)
 * - S1, S10 (seconds)
 * - M1, M10 (minutes)
 * - H1, H10 (hours)
 *
 * After updating the timestamp, the new value is written to the `tsMailBox` queue.
 *
 * @param xTimer The timer handle (not used in the current implementation).
 */
static void vTimerCallback (TimerHandle_t xTimer) {
#if debug_USE_TIMESTAMP == YES
  ts[ms1]++;
  if (ts[ms1] == ':') {
    ts[ms1] = '0';
    ts[ms10]++;
    if (ts[ms10] == ':') {
      ts[ms10] = '0';
      ts[ms100]++;
      if (ts[ms100] == ':') {
        ts[ms100] = '0';
        ts[S1]++;
        if (ts[S1] == ':') {
          ts[S1] = '0';
          ts[S10]++;
          if (ts[S10] == '6') {
            ts[S10] = '0';
            ts[M1]++;
            if (ts[M1] == ':') {
              ts[M1] = '0';
              ts[M10]++;
              if (ts[M10] == '6') {
                ts[M10] = '0';
                ts[H1]++;
                if (ts[H1] == ':') {
                  ts[H1] = '0';
                  ts[H10]++;
                }
                else if (ts[H10] == '2' && ts[H1] == '4') {
                  ts[H10] = '0';
                  ts[H1]  = '0';
                }
              }
            }
          }
        }
      }
    }
  }
  xQueueOverwrite(tsMailBox, ts);
#endif //debug_USE_TIMESTAMP  
}
//------------------------------------------------------------------------
/**
 * @brief Triggers the DMA transfer for debug transmission if idle, and swaps the active buffer.
 *
 * This function checks if the debug DMA is idle (NDTR == 0), then:
 * - Disables the DMA,
 * - Clears any pending error or transfer complete flags,
 * - Sets the memory address and transfer size from the active buffer,
 * - Swaps to the next buffer and resets its index,
 * - Re-enables the DMA to start transmission.
 *
 * @return true if a DMA transfer was triggered, false if DMA was still busy.
 */
static bool __debug_dma_trig_and_swap (void) {
  bool trigged = false;
  if (drv_hw_dma_get_ndtr() == 0) {
    drv_hw_dma_disable();  
    drv_hw_dma_clearErrorFlags();
    if (drv_hw_dma_get_tc_flag()) {
      drv_hw_dma_clear_tc_flag();
    }
    drv_hw_dma_set_memory_address((uint32_t)tx.buf[tx.active]);
    drv_hw_dma_set_ndtr(tx.index[tx.active]);
    tx.active ^= 1;
    tx.index[tx.active] = 0;
    trigged = true;
    drv_hw_dma_enable();
  }
  return trigged;
}
//------------------------------------------------------------------------
/**
 * @brief Task that processes incoming debug UART data from the stream buffer.
 *
 * This task waits for incoming bytes from the debug UART stream buffer.
 * It detects packets that start and end with a carriage return (`\r`),
 * and passes complete packets to the `hmi_decoder()` for processing.
 *
 * It also includes protection against buffer overflows by resetting
 * the packet assembly state if the buffer limit is reached.
 *
 * @param pvParameters Unused task parameter (can be NULL).
 */
#if debug_RX_ENGINE_ENABLE == YES
static void serviceDebugRx(void* const pvParameters) {
  uint8_t rxBuf[debug_RX_TOTAL_RAM];
  uint16_t cnt = 0;
  bool headerFlag = false;
  while (1) {
    xStreamBufferReceive(streamRx, rxBuf + cnt, 1, portMAX_DELAY);
    if (rxBuf[cnt] == '\r') {
      if (headerFlag) {
        /* Packet complete */
        if (cnt > 0) {
          /* Handle the packet here */
          rxBuf[cnt % debug_RX_TOTAL_RAM] = '\0';
          hmi_decoder(rxBuf, cnt);
        }
        cnt = 0;
        headerFlag = false;
      } 
      else {
        /* Start of a new packet */
        headerFlag = true;
        cnt = 0;
      }
    } 
    else if (headerFlag) {
      cnt++;
      if (cnt >= sizeof(rxBuf)) {
        /* Buffer overflow protection */
        cnt = 0;
        headerFlag = false;
      }
    }
  } 
  /* Never reaches here */
}
#endif //debug_RX_ENGINE_ENABLE
//------------------------------------------------------------------------
/**
 * @brief UART interrupt handler for receiving debug data.
 *
 * This ISR handles the RXNE (Receive Not Empty) interrupt for the debug UART.
 * When a byte is received, it is read from the UART data register and pushed
 * into the debug receive stream buffer.
 *
 * @note This function should be linked to the UART IRQ used for the debug interface (e.g., USART2).
 */
void debug_USART_IRQHandler (void) {
  if (drv_hw_uart_get_rxne_flag() != 0) {
    drv_hw_uart_clear_rxne_flag();
    uint8_t receivedByte = drv_hw_uart_read_byte();
    xStreamBufferSendFromISR(streamRx, &receivedByte, sizeof(receivedByte), NULL);
  }
}
//------------------------------------------------------------------------
/**
 * @brief DMA interrupt handler for the debug transmit channel.
 *
 * This ISR clears DMA error and transfer complete flags related to the debug
 * interface. If a transfer complete (TC) event is detected, it requests a
 * context switch if needed.
 *
 * @note This function should be linked to the actual DMA IRQ used for debug transmission.
 */
void debug_DMA_IRQHandler (void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  drv_hw_dma_clearErrorFlags();
  if (drv_hw_dma_get_tc_flag()) {
    drv_hw_dma_clear_tc_flag();
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
//------------------------------------------------------------------------
/**
 * @brief Initializes the debug system.
 *
 * Sets up all required components for the debug interface, including:
 * - Timestamp queue and timer
 * - Debug transmission mutex
 * - Receive stream buffer
 * - Hardware driver
 * - Debug RX service task
 *
 * This function must be called before using any debug transmission functions
 * like @ref debug_transmit().
 *
 * @return true if all components were initialized successfully, false otherwise.
 */
bool debug_init (void) {
  bool status = true;
  #if debug_USE_TIMESTAMP == YES
  status = status && (tsMailBox = xQueueCreate(1, sizeof(ts))) != 0;
  status = status && (hTsTimer = xTimerCreate("Timestamp Timer", pdMS_TO_TICKS(1), pdTRUE, NULL, &vTimerCallback)) != 0;
  status = status && (xQueueSend(tsMailBox, ts, 0)) == pdPASS;
  status = status && (xTimerStart(hTsTimer, 0)) == pdPASS;
  #endif //debug_USE_TIMESTAMP
  status = status && (tx.mutex = xSemaphoreCreateMutex()) != NULL;
  #if debug_RX_ENGINE_ENABLE == YES
  status = status && (streamRx = xStreamBufferCreate(debug_RX_TOTAL_RAM, 1)) != NULL;
  status = status && xTaskCreate(&serviceDebugRx, "DBG_RX",   debug_RX_STACK_SIZE / 4,   NULL, debug_RX_TASK_PRIORITY,   &hTaskRx)  == pdTRUE;
  #endif //debug_RX_ENGINE_ENABLE
  status = status && drv_hw_driver_init();
  if (status == true) {
    LOG_TRACE("Serial debugger engine initialized successfully");
  }
  return status;
}
//------------------------------------------------------------------------
static BaseType_t __debug_xSemaphoreTakeFromTask (SemaphoreHandle_t obj) {
  return xSemaphoreTake(obj, 0);
}
static BaseType_t __debug_xSemaphoreTakeFromISR (SemaphoreHandle_t obj) {
  return xSemaphoreTakeFromISR(obj, NULL);
}
static BaseType_t (* const semaphoreTake_Fn[2]) (SemaphoreHandle_t) = {
  __debug_xSemaphoreTakeFromTask,
  __debug_xSemaphoreTakeFromISR,
};
//------------------------------------------------------------------------
#if debug_USE_TIMESTAMP == YES
static void __debug_getTimestampFromTask (uint8_t* buf) {
  xQueuePeek(tsMailBox, buf, 0);
}
static void __debug_getTimestampFromISR (uint8_t* buf) {
  xQueuePeekFromISR(tsMailBox, buf);
}
static void (* const getTimestamp_Fn[2]) (uint8_t*) = {
  __debug_getTimestampFromTask,
  __debug_getTimestampFromISR,
};
#endif //debug_USE_TIMESTAMP
//------------------------------------------------------------------------
static void __debug_xSemaphoreGiveFromTask (SemaphoreHandle_t obj) {
  xSemaphoreGive(obj);
}
static void __debug_xSemaphoreGiveFromISR (SemaphoreHandle_t obj) {
  xSemaphoreGiveFromISR(obj, NULL);
}
static void (* const semaphoreGive_Fn[2]) (SemaphoreHandle_t) = {
  __debug_xSemaphoreGiveFromTask,
  __debug_xSemaphoreGiveFromISR,
};
//------------------------------------------------------------------------
/**
 * @brief Sends a formatted debug message over the debug interface.
 *
 * This function acts like `printf`. It formats and queues a debug message
 * based on the specified log level and arguments. Messages below the configured
 * debug level are ignored.
 *
 * It works in both thread and interrupt context, using the appropriate FreeRTOS APIs.
 *
 * @param level   Debug message level (e.g., info, warning, error).
 * @param argLen  Number of arguments after FORMAT (1 means plain string).
 * @param FORMAT  `printf`-style format string or plain message.
 * @param ...     Optional arguments matching the FORMAT string.
 *
 * @return true if the message was accepted for transmission, false otherwise.
 */
bool debug_transmit (uint8_t level, uint8_t argLen, const char* FORMAT, ...) {
  uint32_t freeSpace;
  uint32_t __len;
  #if debug_USE_TIMESTAMP == YES
  uint8_t timestamp[sizeof(ts)];
  #endif //debug_USE_TIMESTAMP
  bool result = 0;
  if (debugConf.level > level) {
    return result;
  }
  uint8_t context = IS_INSIDE_INTERRUPT();
  if (semaphoreTake_Fn[context](tx.mutex) == pdTRUE) {
    #if debug_USE_LABLE == YES
    __debug_copyFrom(LOG_LEVEL_STRING[level], sizeof(__TRACE_LABEL) - 1);
    #endif //debug_USE_LABLE
    #if debug_USE_TIMESTAMP == YES
    getTimestamp_Fn[context](timestamp);
    __debug_copyFrom(timestamp, sizeof(ts) - 1);
    #endif //debug_USE_TIMESTAMP
    #if debug_USE_SPRINTF_FORMATTER == YES
    if (argLen == 1) {
      __debug_copyFrom((uint8_t*)FORMAT, strlen(FORMAT));
    }
    else { 
      va_list args;
      va_start(args, FORMAT);
      freeSpace = (debug_TX_TOTAL_RAM / 2) - tx.index[tx.active];
      __len = vsnprintf((char*)(tx.buf[tx.active] + tx.index[tx.active]), freeSpace, FORMAT, args);
      tx.index[tx.active] += (freeSpace > __len)? __len : freeSpace;
      va_end(args);
    }
    #else 
    __debug_copyFrom((uint8_t*)FORMAT, strlen(FORMAT));
    #endif //debug_USE_SPRINTF_FORMATTER
    freeSpace = (debug_TX_TOTAL_RAM / 2) - tx.index[tx.active];
    if (freeSpace >= sizeof(__FOOTER) - 1) {
      memcpy(tx.buf[tx.active] + tx.index[tx.active], __FOOTER, sizeof(__FOOTER) - 1);
      tx.index[tx.active] += (sizeof(__FOOTER) - 1);
    }
    else {
      memcpy(tx.buf[tx.active] + (debug_TX_TOTAL_RAM / 2) - (sizeof(__FOOTER) - 1), __FOOTER, sizeof(__FOOTER) - 1);
      tx.index[tx.active] = (debug_TX_TOTAL_RAM / 2);
    }
    __debug_dma_trig_and_swap();
    result = true;
    semaphoreGive_Fn[context](tx.mutex);
  }
  return result;
}
//------------------------------------------------------------------------
void debug_getTimestamp14 (uint8_t* dst) {
  #if debug_USE_TIMESTAMP == YES 
  getTimestamp_Fn[IS_INSIDE_INTERRUPT()](dst);
  #endif //debug_USE_TIMESTAMP
}
//------------------------------------------------------------------------
void debug_setTimestamp (uint8_t hh,uint8_t mm, uint8_t ss) {
  #if debug_USE_TIMESTAMP == YES 
  vPortEnterCritical();
  snprintf(ts, sizeof(ts), "%02d:%02d:%02d:000 ", hh % 24, mm % 60, ss % 60);
  vPortExitCritical();
  #endif //debug_USE_TIMESTAMP
}
//------------------------------------------------------------------------
