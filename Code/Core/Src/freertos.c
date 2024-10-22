/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GPIO/GPIOHandler.h"
#include "EngineControl/EngineControl.h"
#include "Encoder/Encoder.h"
#include "MotorController/SpeedController.h"
#include "CAN_Handle.h"
#include <stdbool.h>
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern CAN_HandleTypeDef hcan;
extern GPIO_HandlerStruct Blinker;
extern GPIO_HandlerStruct BrakePin;
extern GPIO_HandlerStruct GasEnPin;
extern GPIO_HandlerStruct DirPin;

extern Encoder_HandlerStruct Encoder;
extern Engine_HandlerStruct Engine;
extern SpeedControler_HandlerStruct SpeedController;
extern float Speed;
extern bool SpeedOk;
extern uint8_t Break;
extern bool braking;
extern float curSpeed;
extern uint8_t rcdata[8];
extern uint8_t Can_RecFlag;
extern float speeding;
/* USER CODE END Variables */
/* Definitions for tDefault */
osThreadId_t tDefaultHandle;
const osThreadAttr_t tDefault_attributes = {
  .name = "tDefault",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myCAN_handl */
osThreadId_t myCAN_handlHandle;
const osThreadAttr_t myCAN_handl_attributes = {
  .name = "myCAN_handl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CAN_Mutex */
osMutexId_t CAN_MutexHandle;
const osMutexAttr_t CAN_Mutex_attributes = {
  .name = "CAN_Mutex"
};
/* Definitions for data_semaphore */
osSemaphoreId_t data_semaphoreHandle;
const osSemaphoreAttr_t data_semaphore_attributes = {
  .name = "data_semaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void CAN_SendData(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of CAN_Mutex */
  CAN_MutexHandle = osMutexNew(&CAN_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of data_semaphore */
  data_semaphoreHandle = osSemaphoreNew(1, 0, &data_semaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of tDefault */
  tDefaultHandle = osThreadNew(StartDefaultTask, NULL, &tDefault_attributes);

  /* creation of myCAN_handl */
  myCAN_handlHandle = osThreadNew(CAN_SendData, NULL, &myCAN_handl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the tDefault thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint8_t Data[8];
	/* Infinite loop */
	for (;;) {
		if (Can_RecFlag == 1) {
			memcpy(Data, rcdata, 8);
			switch (Data[0]) {
			case 0x23:
				switch (Data[1]) {
				case 1:
					stopEngineAndBrake(&SpeedController, &Engine);
					braking = false;
					HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
					break;
				default:
					braking = true;
					HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,
							GPIO_PIN_RESET);
				}
				break;
			case 0x34:
				if (braking) {
					Speed = bytes2Float(Data + 1);
					speeding = SpeedController_GetSpeed(&SpeedController);
					if (Speed == 0) {
						stopEngineAndBrake(&SpeedController, &Engine);
					} else if (!SpeedOk && Speed == SpeedController.SetPoint) {
						SpeedOk = true;
					} else {
						SpeedOk = false;
					}
					if (!SpeedOk) {
						if (Speed > 0) {
							setSpeedAndClearBrake(&SpeedController, &Engine,
									Speed);
							if (!SpeedController.isEnable) {
								SpeedController_Start(&SpeedController, 1);
							}
						} else {
							SpeedController_Start(&SpeedController, 0);
							Engine_SetSpeed(&Engine,
									Speed * 4 * Engine_TIM_MaxPulse / 100);
						}
					}
				}
				break;
			}
		}
		osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_CAN_SendData */
/**
 * @brief Function implementing the myCAN_handl thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CAN_SendData */
void CAN_SendData(void *argument)
{
  /* USER CODE BEGIN CAN_SendData */
	uint8_t sendData[8];
	memset(sendData, 0x55, 8);
	/* Infinite loop */
	for (;;) {
		speeding = SpeedController_GetSpeed(&SpeedController);
		float2Bytes(sendData, speeding);
		CAN_HandleSendData(0x203, sendData, 8);
		osDelay(10);
	}
  /* USER CODE END CAN_SendData */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END DataProcessing */

/* USER CODE END Application */

