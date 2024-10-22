/*
 * DataProcess.h
 *
 *  Created on: Jun 29, 2023
 *      Author: vanti
 */

#ifndef DATAPROCESS_DATAPROCESS_H_
#define DATAPROCESS_DATAPROCESS_H_

#include "Utility/Utility.h"

int DataPack_Split(char* str, uint8_t* ID, char *pValue);
int DataPack_GetLEDState(char* str, int* LEDLeft, int* LEDRight, int* LEDFront, int* LEDStop);
int DataPack_GetSpeakerState(char* str, int* SpeakerState);
int DataPack_GetSteerAngle(char* str, float* Angle);
int DataPack_GetSpeed(char* str, float* Speed);

#endif /* DATAPROCESS_DATAPROCESS_H_ */
