/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */



/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

extern CAN_RxHeaderTypeDef CAN_RxMsg;
extern uint8_t RXmessage[8];
extern int16_t value;
extern int16_t value2;
	
typedef struct
{
    uint32_t RawValue;          //���α�����ԭʼֵ
    uint32_t PreRawValue;       //��һ�α�����ԭʼֵ
    int32_t Diff;              //���α�����ԭʼֵ��ֵ
    int32_t Number;            //����������
    float SumValue;        //�������ۼ�ֵ
	
}ST_ENCODER;

typedef struct
{
    float fpDes;    //���Ʊ���Ŀ��ֵ
    float fpFB;     //���Ʊ�������ֵ
	
    float fpKp;     //����ϵ��
    float fpKi;     //�������
    float fpKd;     //΢��ϵ��
	
	  float fpUp;      //�������
  	float fpUi;      //�������
  	float fpUd;      //΢�����
	
		float fpE;       //����ƫ��
		float fpPreE;    //�ϴ�ƫ��
		float fpSumE;    //��ƫ��
		float fpU;       //����PID������
		
		float fpUMax;    //�������������ֵ
		float fpEpMax;   //������������ֵ
		float fpEiMax;   //������������ֵ
		float fpEdMax;   //΢����������ֵ
		float fpEMin;    //��������
		float fpSumEMax;    //��ƫ������
		
		float fpDt;      //��������
	
}ST_PID;

uint32_t GetEncoderNumber(uint8_t DATA[8]);
uint16_t GetSpeed (CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef CanRxMsg, uint8_t DATA[8]);
uint8_t GetTemperature (CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef CanRxMsg, uint8_t DATA[8]);

void Abs_Encoder_Process(ST_ENCODER*pEncoder,uint32_t value);


/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */



/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
