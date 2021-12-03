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
    uint32_t RawValue;          //本次编码器原始值
    uint32_t PreRawValue;       //上一次编码器原始值
    int32_t Diff;              //两次编码器原始值差值
    int32_t Number;            //编码器线数
    float SumValue;        //编码器累加值
	
}ST_ENCODER;

typedef struct
{
    float fpDes;    //控制变量目标值
    float fpFB;     //控制变量反馈值
	
    float fpKp;     //比例系数
    float fpKi;     //积分输出
    float fpKd;     //微分系数
	
	  float fpUp;      //比例输出
  	float fpUi;      //积分输出
  	float fpUd;      //微分输出
	
		float fpE;       //本次偏差
		float fpPreE;    //上次偏差
		float fpSumE;    //总偏差
		float fpU;       //本次PID运算结果
		
		float fpUMax;    //运算后输入上限值
		float fpEpMax;   //比例项输出最大值
		float fpEiMax;   //积分项输出最大值
		float fpEdMax;   //微分项输出最大值
		float fpEMin;    //积分上限
		float fpSumEMax;    //总偏差上限
		
		float fpDt;      //控制周期
	
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
