/*
 * DataProcess.c
 *
 *  Created on: Jun 29, 2023
 *      Author: vanti
 */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "DataProcess.h"

int DataPack_Split(char* str, uint8_t* ID, char *pValue)
{

	if (str[0] != '#') return 1;
	*ID = atoi(str+1);
	char* tok = strtok(str, "=");
	if(tok == NULL) return 1;
	tok = strtok(NULL,"=");
	if(tok == NULL) return 1;
	strcpy(pValue, tok);
	tok = strtok(pValue,"\r");
	return 0;
}

int DataPack_GetLEDState(char* str, int* LEDLeft, int* LEDRight, int* LEDFront, int* LEDStop)
{
	char* tok = strtok(str, ";");
	if(tok == NULL) return 1;
	*LEDLeft = atoi(tok);

	tok = strtok(NULL, ";");
	if(tok == NULL) return 1;
	*LEDRight = atoi(tok);

	tok = strtok(NULL, ";");
	if(tok == NULL) return 1;
	*LEDFront = atoi(tok);

	tok = strtok(NULL, ";");
	if(tok == NULL) return 1;
	*LEDStop = atoi(tok);
	return 0;
}
int DataPack_GetSpeakerState(char* str, int* SpeakerState)
{
	*SpeakerState = atoi(str);
	return 0;
}

int DataPack_GetSteerAngle(char* str, float* Angle)
{

	*Angle = StrToFloat(str);
	return 0;
}

int DataPack_GetSpeed(char* str, float* Speed)
{

	*Speed = StrToFloat(str);
	return 0;
}
