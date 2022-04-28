/*
 * stm32_can_libv1.h
 *
 *  Created on: 9 Kas 2019
 *      Author: yemrelaydin
 */

#ifndef STM32F1XX_HAL_DRIVER_INC_STM32_CAN_LIBV1_H_
#define STM32F1XX_HAL_DRIVER_INC_STM32_CAN_LIBV1_H_
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
#include "AESK_Ring_Buffer.h"
#define CAN_IDE_32				0x04


 typedef enum
 {
	 CAN_BUFFER_MOD = -1,
	 CAN_Error = 0,
	 CAN_OK = 1,
	 CAN_TransmitError = 2,
	 CAN_ReceiveError = 3,
	 CAN_WRONG_ID = 4

 }CAN_ErrorState;
 typedef struct {
 	 CAN_HandleTypeDef* hcan;
 	 CAN_TxHeaderTypeDef txMsg;
 	 CAN_RxHeaderTypeDef rxMsg;
 	 CAN_FilterTypeDef sConfig;
 	 uint32_t pTxMailbox;
 	 uint8_t receivedData[8];
 	 uint8_t rx_buf[255];
    uint8_t* rx_ptr;
  }AESK_CAN_Struct;
  AESK_CAN_Struct aesk_can;

 CAN_ErrorState AESK_CAN_Send_RingBuffer(AESK_CAN_Struct *can_struct, AESK_Ring_Buffer* ring_buf, uint32_t address);

 CAN_ErrorState AESK_CAN_Init( AESK_CAN_Struct *can_struct, uint32_t activateInterrupt);

 CAN_ErrorState AESK_CAN_ExtIDListFilterConfiguration(AESK_CAN_Struct *can_struct, uint32_t filterAddress,
 									   uint32_t FIFOSelect, uint32_t filterBank);


 CAN_ErrorState AESK_CAN_StdtIDListFilterConfiguration(AESK_CAN_Struct *can_struct, uint32_t filterAddress,
 									   uint32_t FIFOSelect, uint32_t filterBank);

 CAN_ErrorState AESK_CAN_ExtIDMaskFilterConfiguration(AESK_CAN_Struct *can_struct, uint32_t filterAddress,
 									   uint32_t filterMaskAddress, uint32_t FIFOSelect, uint32_t filterBank);

 CAN_ErrorState AESK_CAN_StdIDMaskFilterConfiguration(AESK_CAN_Struct *can_struct, uint32_t filterAddress,
 									   uint32_t filterMaskAddress, uint32_t FIFOSelect, uint32_t filterBank);

 CAN_ErrorState AESK_CAN_SendExtIDMessage(AESK_CAN_Struct *can_struct, uint32_t address, uint8_t *transmitBuf, uint32_t size);

 CAN_ErrorState AESK_CAN_SendStdIDMessage(AESK_CAN_Struct *can_struct, uint32_t address, uint8_t *transmitBuf, uint32_t size);

 CAN_ErrorState AESK_CAN_ReadExtIDMessage(AESK_CAN_Struct *can_struct, uint32_t FIFOSelect);

 CAN_ErrorState AESK_CAN_ReadStdIDMessage(AESK_CAN_Struct *can_struct, uint32_t address, uint32_t FIFOSelect,
 							   uint8_t *receiveBuf);
#ifdef __cplusplus
}
#endif
#endif /* STM32F1XX_HAL_DRIVER_INC_STM32_CAN_LIBV1_H_ */
