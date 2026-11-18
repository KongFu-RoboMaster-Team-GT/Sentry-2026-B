#ifndef TEST_CAN_H
#define TEST_CAN_H
#include <stdint.h>
typedef struct { volatile uint32_t ESR, MCR, MSR, TSR; } CAN_TypeDef;
typedef struct { CAN_TypeDef *Instance; uint32_t ErrorCode, State; } CAN_HandleTypeDef;
typedef struct { uint32_t StdId, IDE, RTR, DLC; } CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
extern CAN_TypeDef test_can1, test_can2;
#define CAN1 (&test_can1)
#define CAN2 (&test_can2)
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
#define CAN_RX_FIFO0 0
#define HAL_OK 0
#define HAL_CAN_STATE_READY 1u
#define HAL_CAN_STATE_LISTENING 2u
#define HAL_CAN_STATE_ERROR 5u
#define HAL_CAN_ERROR_NONE 0u
#define HAL_CAN_ERROR_TIMEOUT 0x00020000u
#define CAN_ESR_BOFF 0x00000004u
#define CAN_MCR_INRQ 0x00000001u
#define CAN_MCR_SLEEP 0x00000002u
#define CAN_MSR_INAK 0x00000001u
#define CAN_TSR_TME 0x1C000000u
#define CAN_TSR_TME0 0x04000000u
#define CAN_TX_MAILBOX0 1u
#define CAN_TX_MAILBOX1 2u
#define CAN_TX_MAILBOX2 4u
#define CAN_IT_RX_FIFO0_MSG_PENDING 2u
extern CAN_HandleTypeDef hcan1, hcan2;
uint32_t HAL_GetTick(void);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can);
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, uint8_t *, uint32_t *);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t);
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *);
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *, uint32_t);
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
#endif
