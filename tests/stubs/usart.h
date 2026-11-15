#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef int HAL_StatusTypeDef;
typedef struct { uint32_t SR, DR, CR1, CR3; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
typedef struct { uint32_t unused; } DMA_HandleTypeDef;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern uint32_t test_uart_idle, test_dma_remaining;
#define UART_FLAG_IDLE 1u
#define UART_IT_IDLE 1u
#define RESET 0
#define __HAL_UART_GET_FLAG(uart, flag) test_uart_idle
#define __HAL_UART_CLEAR_IDLEFLAG(uart) ((void)(test_uart_idle = 0))
#define __HAL_UART_ENABLE_IT(uart, flag) ((void)0)
#define __HAL_DMA_GET_COUNTER(dma) test_dma_remaining
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *);
void HAL_UART_IRQHandler(UART_HandleTypeDef *);
#endif
