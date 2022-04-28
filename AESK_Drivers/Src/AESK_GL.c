/*
 * AESK_GL.c
 *
 *  Created on: Dec 12, 2020
 *      Author: basri
 */


#include "AESK_GL.h"

void AESK_GL_Init(AESK_GL* aesk_gl)
{
	for(uint32_t i = 0; i < sizeof(AESK_GL); i++)
	{
		((uint8_t*)aesk_gl)[i] = 0;
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
 {
		if(hcan == aesk_gl.aesk_can_2.hcan)
		{
			AESK_CAN_Send_RingBuffer(&aesk_gl.aesk_can_2, &CAN2_Rng_Buf_Tx, aesk_gl.aesk_can_2.txMsg.ExtId);
		}
		else
		{
			AESK_CAN_Send_RingBuffer(&aesk_gl.aesk_can_1, &CAN1_Rng_Buf_Tx, aesk_gl.aesk_can_1.txMsg.ExtId);
		}
 }

 void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
 {
		if(hcan == aesk_gl.aesk_can_2.hcan)
		{
			AESK_CAN_Send_RingBuffer(&aesk_gl.aesk_can_2, &CAN2_Rng_Buf_Tx, aesk_gl.aesk_can_2.txMsg.ExtId);
		}
		else
		{
			AESK_CAN_Send_RingBuffer(&aesk_gl.aesk_can_1, &CAN1_Rng_Buf_Tx, aesk_gl.aesk_can_1.txMsg.ExtId);
		}
 }
 void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
 {
		if(hcan == aesk_gl.aesk_can_2.hcan)
		{
			AESK_CAN_Send_RingBuffer(&aesk_gl.aesk_can_2, &CAN2_Rng_Buf_Tx, aesk_gl.aesk_can_2.txMsg.ExtId);
		}
		else
		{
			AESK_CAN_Send_RingBuffer(&aesk_gl.aesk_can_1, &CAN1_Rng_Buf_Tx, aesk_gl.aesk_can_1.txMsg.ExtId);
		}
 }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
#if COM_MODE == COMPRO
	if(hcan == aesk_gl.aesk_can_1.hcan)
	{
		AESK_CAN_ReadExtIDMessage(&aesk_gl.aesk_can_1, FIFO_0);
		//AESK_CAN_Rx_Handler(aesk_can.rxMsg.ExtId);

		switch (aesk_gl.aesk_can_1.rxMsg.ExtId)
		{
			case 0x1555DDD0 :
			{
				Write_Data_Ring_Buffer(&(aesk_gl.CAN_Rng_Buf_mcu), aesk_gl.aesk_can_1.receivedData, aesk_gl.aesk_can_1.rxMsg.DLC);
				break;
			}
			case 0x15550011 :
			{
				Write_Data_Ring_Buffer(&(CAN_Rng_Buf_telemetry[FIFO_0]), aesk_gl.aesk_can_1.receivedData, aesk_gl.aesk_can_1.rxMsg.DLC);
				break;
			}
			case 0x1555BBB0 :
			{
				Write_Data_Ring_Buffer(&(aesk_gl.CAN_Rng_Buf_bms), aesk_gl.aesk_can_1.receivedData, aesk_gl.aesk_can_1.rxMsg.DLC);
				break;
			}
			default:
			{
				break;
			}
		}
		//Write_Data_Ring_Buffer(&(aesk_gl.CAN_Rng_Buf_1[FIFO_0]), aesk_gl.aesk_can_1.receivedData, aesk_gl.aesk_can_1.rxMsg.DLC);
	}

	else
	{
		AESK_CAN_ReadExtIDMessage(&aesk_gl.aesk_can_2, FIFO_0);
		//AESK_CAN_Rx_Handler(aesk_can.rxMsg.ExtId);
		Write_Data_Ring_Buffer(&CAN2_Rng_Buf_Rx[FIFO_0], aesk_gl.aesk_can_2.receivedData, aesk_gl.aesk_can_2.rxMsg.DLC);
	}

#elif COM_MODE == MCU_OLD
	if(hcan == aesk_gl.aesk_can_1.hcan)
	{
		AESK_CAN_ReadExtIDMessage(&aesk_gl.aesk_can_1, FIFO_0);
		//AESK_CAN_Rx_Handler(aesk_can.rxMsg.ExtId);
		if(aesk_gl.aesk_can_1.rxMsg.ExtId != MCU_PACK_ID_1 &&
		   aesk_gl.aesk_can_1.rxMsg.ExtId != MCU_PACK_ID_2 &&
		   aesk_gl.aesk_can_1.rxMsg.ExtId != MCU_PACK_ID_3)
		{
			Write_Data_Ring_Buffer(&(aesk_gl.CAN_Rng_Buf_1[FIFO_0]), aesk_gl.aesk_can_1.receivedData, aesk_gl.aesk_can_1.rxMsg.DLC);
		}

		else
		{
			VCU_Can_Rx_Handler(aesk_gl.aesk_can_1.rxMsg.ExtId);
		}
	}
	else
	{
		AESK_CAN_ReadExtIDMessage(&aesk_gl.aesk_can_2, FIFO_0);
		if(aesk_gl.aesk_can_1.rxMsg.ExtId != MCU_PACK_ID_1 &&
		   aesk_gl.aesk_can_1.rxMsg.ExtId != MCU_PACK_ID_2 &&
		   aesk_gl.aesk_can_1.rxMsg.ExtId != MCU_PACK_ID_3)
		{
			Write_Data_Ring_Buffer(&(aesk_gl.CAN_Rng_Buf_2[FIFO_0]), aesk_gl.aesk_can_2.receivedData, aesk_gl.aesk_can_2.rxMsg.DLC);
		}

		else
		{
			VCU_Can_Rx_Handler(aesk_gl.aesk_can_2.rxMsg.ExtId);
		}
	}


#elif COM_MODE == OLD_COM
	AESK_CAN_ReadExtIDMessage(&aesk_gl.aesk_can_1, FIFO_0);
	VCU_Can_Rx_Handler(aesk_gl.aesk_can_1.rxMsg.ExtId);

#endif
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == aesk_gl.aesk_can_1.hcan)
	{
		AESK_CAN_ReadExtIDMessage(&aesk_gl.aesk_can_1, FIFO_1);
		Write_Data_Ring_Buffer(&CAN1_Rng_Buf_Rx[FIFO_1], aesk_gl.aesk_can_1.receivedData, aesk_gl.aesk_can_1.rxMsg.DLC);
	}
	else
	{
		AESK_CAN_ReadExtIDMessage(&aesk_gl.aesk_can_2, FIFO_1);
		Write_Data_Ring_Buffer(&CAN2_Rng_Buf_Rx[FIFO_1], aesk_gl.aesk_can_2.receivedData, aesk_gl.aesk_can_2.rxMsg.DLC);
	}
}


uint16_t aeskCRCCalculator(uint8_t *frame, size_t framesize)
{
	uint16_t crc16_data = 0;
	uint8_t data = 0;
	for (uint8_t mlen = 0; mlen < framesize; mlen++)
	{
		data = frame[mlen] ^ ((uint8_t) (crc16_data) & (uint8_t) (0xFF));
		data ^= data << 4;
		crc16_data = ((((uint16_t) data << 8) | ((crc16_data & 0xFF00) >> 8)) ^ (uint8_t) (data >> 4) ^ ((uint16_t) data << 3));
	}
	return (crc16_data);
}




