/*
 * CAN_Handle.c
 *
 *  Created on: Dec 22, 2023
 *      Author: win 10
 */
#include "main.h"
#include "CAN_Handle.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
extern CAN_HandleTypeDef hcan;
CAN_RxHeaderTypeDef Rxheader;
uint8_t rcdata[8];
uint8_t Can_RecFlag = 0;
extern osMutexId_t CAN_MutexHandle;
HAL_StatusTypeDef CAN_HandleSendData(const uint32_t arbitration_id,
		const uint8_t *data, const uint8_t size) {
	if (osMutexAcquire(CAN_MutexHandle, 100) != osOK) {
		CAN_TxHeaderTypeDef Txheader;
		uint32_t TxMailbox;
		Txheader.DLC = size;
		Txheader.IDE = CAN_ID_STD;
		Txheader.RTR = CAN_RTR_DATA;
		Txheader.StdId = arbitration_id;
		if (HAL_CAN_AddTxMessage(&hcan, &Txheader, data, &TxMailbox)
				!= HAL_OK) {
			return HAL_ERROR;
		} else {
			return HAL_OK;
		}
	}
}
void CAN_Config_filtering(uint8_t FilterBank, uint16_t NodeID) {
	CAN_FilterTypeDef Can_filter_init;
	Can_filter_init.FilterActivation = ENABLE;
	Can_filter_init.FilterBank = FilterBank;
	Can_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	Can_filter_init.FilterIdHigh = (NodeID << 5) & 0xFFFF;
	;
	Can_filter_init.FilterIdLow = 0x0000;
	Can_filter_init.FilterMaskIdHigh = 0xFFF8;
	Can_filter_init.FilterMaskIdLow = 0x0000;
	Can_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	Can_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;
	if (HAL_CAN_ConfigFilter(&hcan, &Can_filter_init) != HAL_OK) {
		Error_Handler();
	}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rxheader, rcdata) == HAL_OK) {
		Can_RecFlag = 1;
	} else {
		Can_RecFlag = 0;
	}
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Rxheader, rcdata) == HAL_OK) {
		Can_RecFlag = 1;
	} else {
		Can_RecFlag = 0;
	}
}
float bytes2Float(uint8_t bytes[4]) {
	union {
		uint8_t bytes[4];
		float a;
	} thing;
	memcpy(thing.bytes, bytes, 4);
	return thing.a;
}
void float2Bytes(uint8_t bytes_temp[4], float float_variable) {
	union {
		float a;
		unsigned char bytes[4];
	} thing;
	thing.a = float_variable;
	memcpy(bytes_temp, thing.bytes, 4);
}
