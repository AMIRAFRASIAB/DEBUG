
#include "stdint.h"
#include "cmsis_armclang.h"
#include "serial_debugger.h"
#include "serial_config.h"
#include "serial_driver.h"
#include "serial_hmi.h"
#include "stdint.h"
#include "stdbool.h"
#include  "string.h"
#include "stdarg.h"
#include "stdio.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "stream_buffer.h"

//------------------------------------------------------------------------
__STATIC_INLINE BaseType_t __xPortIsInsideInterrupt (void) {
  return (__get_IPSR() == 0)? pdFALSE : pdTRUE;
}
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
  uint8_t            buf[2][DEBUG_TX_TOTAL_RAM / 2];
  uint8_t            active;
} Tx_s;
//------------------------------------------------------------------------
static const uint8_t  __FOOTER[]          = "\n"     ;
static const uint8_t  __TRACE_LABEL[]     = "<TRC> " ;
static const uint8_t  __INFO_LABEL[]      = "<INF> " ;
static const uint8_t  __WARNING_LABEL[]   = "<WRN> " ;
static const uint8_t  __ERROR_LABEL[]     = "<ERR> " ;
static const uint8_t  __FATAL_LABEL[]     = "<FTL> " ;
static const uint8_t* LOG_LEVEL_STRING[]  = {__TRACE_LABEL, __INFO_LABEL, __WARNING_LABEL, __ERROR_LABEL, __FATAL_LABEL};
//------------------------------------------------------------------------
static char                  ts[]        = "00:00:00:000 ";
static TimerHandle_t         hTsTimer    = NULL; 
static QueueHandle_t         tsMailBox   = NULL;
static TaskHandle_t          hTaskRx     = NULL;
static StreamBufferHandle_t  streamRx    = NULL;
static Tx_s                  tx          = {0};
//------------------------------------------------------------------------
static uint32_t __debug_getTimeStamp (void* pvDstBuf, uint32_t dstSizeInByte);
static void vTimerCallback (TimerHandle_t xTimer);
static void serviceDebugRx (void* const pvParameters);
//------------------------------------------------------------------------
void __debug_copyFrom (const void* DATA, uint32_t len) {
  uint32_t freeSpace = (DEBUG_TX_TOTAL_RAM / 2) - tx.index[tx.active];
  len = (freeSpace >= len)? len : freeSpace;
  memcpy(tx.buf[tx.active] + tx.index[tx.active], DATA, len);
  tx.index[tx.active] += len;
}
//------------------------------------------------------------------------
static void vTimerCallback (TimerHandle_t xTimer) {
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
}
//------------------------------------------------------------------------
static uint32_t __debug_getTimeStamp (void* pvDstBuf, uint32_t dstSizeInByte) {
  if (dstSizeInByte < sizeof(ts)) {
    return 0;
  }
  if (xPortIsInsideInterrupt() == pdTRUE) {
    xQueuePeekFromISR(tsMailBox, pvDstBuf);
  }
  else {
    xQueuePeek(tsMailBox, pvDstBuf, 0);
  }
  return sizeof(ts) - 1;
}
//------------------------------------------------------------------------
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
    drv_hw_dma_enable();
  }
  return trigged;
}
//------------------------------------------------------------------------
static void serviceDebugRx(void* const pvParameters) {
  uint8_t rxBuf[DEBUG_RX_TOTAL_RAM];
  uint16_t cnt = 0;
  bool headerFlag = false;
  while (1) {
    xStreamBufferReceive(streamRx, rxBuf + cnt, 1, portMAX_DELAY);
    if (rxBuf[cnt] == '\r') {
      if (headerFlag) {
        /* Packet complete */
        if (cnt > 0) {
          /* Handle the packet here */
          rxBuf[cnt % DEBUG_RX_TOTAL_RAM] = '\0';
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
//------------------------------------------------------------------------
void DEBUG_UART_IRQHandler (void) {
  if (drv_hw_uart_get_rxne_flag() != 0) {
    drv_hw_uart_clear_rxne_flag();
    uint8_t receivedByte = drv_hw_uart_read_byte();
    xStreamBufferSendFromISR(streamRx, &receivedByte, sizeof(receivedByte), NULL);
  }
}
//------------------------------------------------------------------------
void DEBUG_DMA_IRQHandler (void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  drv_hw_dma_clearErrorFlags();
  if (drv_hw_dma_get_tc_flag()) {
    drv_hw_dma_clear_tc_flag();
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
//------------------------------------------------------------------------
bool debug_init (void) {
  bool status = true;
  status = status && (tsMailBox = xQueueCreate(1, sizeof(ts))) != 0;
  status = status && (hTsTimer = xTimerCreate("Timestamp Timer", pdMS_TO_TICKS(1), pdTRUE, NULL, &vTimerCallback)) != 0;
  status = status && (xQueueSend(tsMailBox, ts, 0)) == pdPASS;
  status = status && (xTimerStart(hTsTimer, 0)) == pdPASS;
  status = status && (tx.mutex = xSemaphoreCreateMutex()) != NULL;
  status = status && (streamRx = xStreamBufferCreate(DEBUG_RX_TOTAL_RAM, 1)) != NULL;
  status = status && drv_hw_driver_init();
  status = status && xTaskCreate(&serviceDebugRx, "DBG_RX",   DEBUG_RX_STACK_SIZE / 4,   NULL, DEBUG_RX_TASK_PRIORITY,   &hTaskRx)  == pdTRUE;
  if (status == true) {
    LOG_TRACE("Serial debugger engine initialized successfully");
  }
  return status;
}
//------------------------------------------------------------------------
bool debug_transmit (uint8_t level, uint8_t argLen, const char* FORMAT, ...) {
  uint32_t freeSpace;
  uint32_t __len;
  char timeStamp[sizeof(ts)];
  bool result = 0;
  if (debugConf.level > level) {
    return result;
  }
  if (__xPortIsInsideInterrupt() == pdTRUE) {
    /* Handler mode */
    if (xSemaphoreTakeFromISR(tx.mutex, NULL) == pdTRUE) {
      __debug_copyFrom(LOG_LEVEL_STRING[level], sizeof(__TRACE_LABEL) - 1);
      xQueuePeekFromISR(tsMailBox, timeStamp);
      __debug_copyFrom(timeStamp, sizeof(ts) - 1);
      if (argLen == 1) {
        __debug_copyFrom((uint8_t*)FORMAT, strlen(FORMAT));
      }
      else { 
        va_list args;
        va_start(args, FORMAT);
        freeSpace = (DEBUG_TX_TOTAL_RAM / 2) - tx.index[tx.active];
        __len = vsnprintf((char*)(tx.buf[tx.active] + tx.index[tx.active]), freeSpace, FORMAT, args);
        tx.index[tx.active] += (freeSpace > __len)? __len : freeSpace;
        va_end(args);
      }
      freeSpace = (DEBUG_TX_TOTAL_RAM / 2) - tx.index[tx.active];
      if (freeSpace >= sizeof(__FOOTER) - 1) {
        memcpy(tx.buf[tx.active] + tx.index[tx.active], __FOOTER, sizeof(__FOOTER) - 1);
        tx.index[tx.active] += (sizeof(__FOOTER) - 1);
      }
      else {
        memcpy(tx.buf[tx.active] + (DEBUG_TX_TOTAL_RAM / 2) - (sizeof(__FOOTER) - 1), __FOOTER, sizeof(__FOOTER) - 1);
        tx.index[tx.active] = (DEBUG_TX_TOTAL_RAM / 2);
      }
      __debug_dma_trig_and_swap();
      result = true;
      xSemaphoreGiveFromISR(tx.mutex, NULL);
    }
  }
  else {
    /* Thread mode  */
    if (xSemaphoreTake(tx.mutex, pdMS_TO_TICKS(0)) == pdTRUE) {
      __debug_copyFrom(LOG_LEVEL_STRING[level], sizeof(__TRACE_LABEL) - 1);
      xQueuePeek(tsMailBox, timeStamp, 0);
      __debug_copyFrom(timeStamp, sizeof(ts) - 1);
      if (argLen == 1) {
        __debug_copyFrom((uint8_t*)FORMAT, strlen(FORMAT));
      }
      else {
        va_list args;
        va_start(args, FORMAT);
        freeSpace = (DEBUG_TX_TOTAL_RAM / 2) - tx.index[tx.active];
        __len = vsnprintf((char*)(tx.buf[tx.active] + tx.index[tx.active]), freeSpace, FORMAT, args);
        tx.index[tx.active] += (freeSpace > __len)? __len : freeSpace;
        va_end(args);
      }
      freeSpace = (DEBUG_TX_TOTAL_RAM / 2) - tx.index[tx.active];
      if (freeSpace >= sizeof(__FOOTER) - 1) {
        memcpy(tx.buf[tx.active] + tx.index[tx.active], __FOOTER, sizeof(__FOOTER) - 1);
        tx.index[tx.active] += (sizeof(__FOOTER) - 1);
      }
      else {
        memcpy(tx.buf[tx.active] + (DEBUG_TX_TOTAL_RAM / 2) - (sizeof(__FOOTER) - 1), __FOOTER, sizeof(__FOOTER) - 1);
        tx.index[tx.active] = (DEBUG_TX_TOTAL_RAM / 2);
      }
      __debug_dma_trig_and_swap();
      result = true;
      xSemaphoreGive(tx.mutex);
    }
  }
  return result;
}
//------------------------------------------------------------------------
