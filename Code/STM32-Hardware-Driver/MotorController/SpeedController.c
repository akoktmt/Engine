/*
 * PIDController.c
 *
 *  Created on: Jul 5, 2023
 *      Author: vanti
 */

#include "SpeedController.h"

const osThreadAttr_t SpeedControlTask_attributes = { .name = "SpdCtrl",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityHigh3, };

void SpeedController_RunTh(void *arg) {
	SpeedControler_HandlerStruct *Controller =
			(SpeedControler_HandlerStruct*) arg;
	uint32_t startTime;
	int delayTime;
	while (1) {
		if (!Controller->isEnable) {
			osThreadSuspend(Controller->RunTh);
		}
		startTime = osKernelGetTickCount();
		SpeedController_Run(Controller);
		delayTime = Controller->dt - (osKernelGetTickCount() - startTime);
		osDelay((delayTime > 0) ? delayTime : 0);
	}
}

void SpeedController_Init(SpeedControler_HandlerStruct *Controller,
		Encoder_HandlerStruct *Encoder, Engine_HandlerStruct *Engine,
		uint32_t interval) {
	memset(Controller, 0, sizeof(SpeedControler_HandlerStruct));
	Controller->Engine = Engine;
	Controller->Encoder = Encoder;

	Controller->RunTh = osThreadNew(SpeedController_RunTh, Controller,
			&SpeedControlTask_attributes);
	Controller->dt = interval;
	Controller->PulsePerMeter = 2927;

}

void SpeedController_Start(SpeedControler_HandlerStruct *Controller,
		uint8_t State) {
	if (State != Controller->isEnable) {
		SpeedController_ClearErr(Controller);
		Controller->isEnable = State;
	}
	if (Controller->isEnable) {
		osThreadResume(Controller->RunTh);
	}
}
void setSpeedAndClearBrake(SpeedControler_HandlerStruct *SpeedController,Engine_HandlerStruct *Engine , float Speed){
	SpeedController_SetSpeed(SpeedController, Speed);
	SpeedController_Start(SpeedController, 0);
	Engine_Brake(Engine, ENGINE_BRAKE_CLEAR);
}
void  stopEngineAndBrake(SpeedControler_HandlerStruct *SpeedController,Engine_HandlerStruct *Engine){
	SpeedController_SetSpeed(SpeedController, 0);
	SpeedController_Start(SpeedController, 0);
	Engine_Brake(Engine, ENGINE_BRAKE_SET);
}
__weak void SpeedController_Run(SpeedControler_HandlerStruct *Controller) {
//	if (Controller->SetPoint == 0) {
//		Engine_SetSpeed(Controller->Engine, 0);
//	}
	float Cur_Error = (Controller->SetPoint
			- SpeedController_GetSpeed(Controller)) * Controller->dt;
	Controller->Integration_Error += Cur_Error;
	float duty = Controller->Kp * Cur_Error
			+ Controller->Ki * Controller->Integration_Error
			+ Controller->Kd * (Cur_Error - Controller->Prev_Error)
					/ Controller->dt;

	Controller->Prev_Error = Cur_Error;
	if (duty < 0)
		duty = 0;
//	if (duty * Controller->SetPoint < 0) {
//		Engine_Brake(Controller->Engine, ENGINE_BRAKE_SET);
//		Encoder_Start(Controller->Encoder, 0);
//		osDelay(2000);
//		Encoder_Start(Controller->Encoder, 1);
////		Engine_Brake(Controller->Engine, ENGINE_BRAKE_CLEAR);
//	}
	if (!Engine_inRange(Controller->Engine, duty)) {
		Controller->Integration_Error -= Cur_Error;
	}
	if (duty > Controller->Engine->PosLimit) {
		duty = Controller->Engine->PosLimit;
	} else if (duty < Controller->Engine->NegLimit) {
		duty = Controller->Engine->NegLimit;
	}
	Engine_SetSpeed(Controller->Engine, (int16_t) duty);
} 

void SpeedController_ClearErr(SpeedControler_HandlerStruct *Controller) {
	Controller->Integration_Error = 0;
	Controller->Prev_Error = 0;
}

void SpeedController_SetSpeed(SpeedControler_HandlerStruct *Controller,
		float newSpeed) {
	if (newSpeed == Controller->SetPoint)
		return;
	Controller->SetPoint = newSpeed;
//	SyncPrintf("Setpoint %f \r\n", newSpeed);
	SpeedController_ClearErr(Controller);
}

float SpeedController_GetSpeed(SpeedControler_HandlerStruct *Controller) {
	return Encoder_GetSpeed_PPS(Controller->Encoder) / Controller->PulsePerMeter;
}

void SpeedController_SetTunings(SpeedControler_HandlerStruct *Controller,
		float Kp, float Ki, float Kd) {
	Controller->Kp = Kp;
	Controller->Ki = Ki;
	Controller->Kd = Kd;
	SpeedController_ClearErr(Controller);

}
