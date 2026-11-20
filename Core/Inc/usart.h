/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#ifndef UART1_RX_DMA_BUFFER_SIZE
#define UART1_RX_DMA_BUFFER_SIZE 256U
#endif
/* UART1-backed shell output queue. Data is copied here before DMA starts. */
#ifndef SHELL_TX_DMA_BUFFER_SIZE
#define SHELL_TX_DMA_BUFFER_SIZE 512U
#endif

#if (SHELL_TX_DMA_BUFFER_SIZE < 2U) || (SHELL_TX_DMA_BUFFER_SIZE > 65535U)
#error "SHELL_TX_DMA_BUFFER_SIZE must be between 2 and 65535 bytes"
#endif

/* Maximum length of one formatted shell response, including its trailing NUL. */
#ifndef SHELL_PRINTF_BUFFER_SIZE
#define SHELL_PRINTF_BUFFER_SIZE 256U
#endif

#if (SHELL_PRINTF_BUFFER_SIZE < 2U) || (SHELL_PRINTF_BUFFER_SIZE > 65535U)
#error "SHELL_PRINTF_BUFFER_SIZE must be between 2 and 65535 bytes"
#endif
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
HAL_StatusTypeDef UART_DMA_Init(void);
HAL_StatusTypeDef uart1_msg_send(const uint8_t *data, uint16_t length);
HAL_StatusTypeDef shell_send_raw(const uint8_t *data, uint16_t length);
int shell_send(const char *format, ...) __attribute__((format(printf, 1, 2)));
const uint8_t *uart1_msg_data(void);
uint16_t uart1_msg_length(void);
void uart1_msg_rev(uint8_t *buf, uint16_t len);

/** Return non-zero while the corresponding UART DMA transmission is active. */
uint8_t uart1_tx_busy(void);

typedef enum
{
  UART1_STREAM_IDLE,
  UART1_STREAM_BUSY,
  UART1_STREAM_DONE,
  UART1_STREAM_ABORTING,
  UART1_STREAM_ERROR
} UART1_STREAM_STATE;

/** 主循环取得二进制流独占权，直到重新初始化；shell 发送返回忙，避免混入文本。 */
HAL_StatusTypeDef uart1_stream_enable(void);
/** 仅主循环调用。零拷贝，数据须保持有效直到状态变为 DONE 或 ERROR。 */
HAL_StatusTypeDef uart1_stream_send(const uint8_t *data, uint16_t length);
UART1_STREAM_STATE uart1_stream_state(void);
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

