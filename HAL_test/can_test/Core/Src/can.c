/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

//----------------pitch6020----------------------------------
ST_ENCODER encoder = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_PID PID_pitch_6020POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 100 , .fpKp = 5, .fpKi = 0.09};
ST_PID PID_pitch_6020Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 15000, .fpKp = 400};

//----------------yaw6020----------------------------------
ST_ENCODER encoder2 = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_PID PID_yaw_6020POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 80 , .fpKp = 3 , .fpKi = 0.008};
ST_PID PID_yaw_6020Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 15000, .fpKp = 420};




//----------------classis1_6020_1号舵机----------------------------------
ST_ENCODER encoder3 = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_PID PID_classis1_6020POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 80 , .fpKp = 4 , .fpKi = 0.05};
ST_PID PID_classis1_6020Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 15000, .fpKp = 290};

//----------------classis2_6020_2号舵机----------------------------------
ST_ENCODER encoder4 = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_PID PID_classis2_6020POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 80 , .fpKp = 4 , .fpKi = 0.05};
ST_PID PID_classis2_6020Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 15000, .fpKp = 290};

//----------------classis3_6020_3号舵机----------------------------------
ST_ENCODER encoder5 = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_PID PID_classis3_6020POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 80 , .fpKp = 4 , .fpKi = 0.05};
ST_PID PID_classis3_6020Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 15000, .fpKp = 260};

//----------------classis4_6020_4号舵机----------------------------------
ST_ENCODER encoder6 = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_PID PID_classis4_6020POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 80 , .fpKp = 4 , .fpKi = 0.05};
ST_PID PID_classis4_6020Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 15000, .fpKp = 260};




//----------------classis1_3508_1号底盘---------------------------------
ST_ENCODER encoder7 = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_PID PID_classis1_3508POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 80 , .fpKp = 0 , .fpKi = 0};
ST_PID PID_classis1_3508Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 15000, .fpKp = 300};

//----------------classis2_3508_2号底盘---------------------------------
ST_ENCODER encoder8 = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_PID PID_classis2_3508POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 80 , .fpKp = 0 , .fpKi = 0};
ST_PID PID_classis2_3508Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 15000, .fpKp = 300};

//----------------classis3_3508_3号底盘---------------------------------
ST_ENCODER encoder9 = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_PID PID_classis3_3508POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 80 , .fpKp = 0 , .fpKi = 0};
ST_PID PID_classis3_3508Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 15000, .fpKp = 300};

//----------------classis4_3508_4号底盘---------------------------------
ST_ENCODER encoder10 = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_PID PID_classis4_3508POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 80 , .fpKp = 0 , .fpKi = 0};
ST_PID PID_classis4_3508Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 15000, .fpKp = 300};



//----------------classis底盘跟随PID---------------------------------
ST_PID PID_classis = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpUMax = 16, .fpKp = 0.2};




//----------------云台外参----------------------------------
int32_t Target_pitch_6020_Position = 0;
int32_t Target_pitch_6020_Speed = 0;
int32_t Target_yaw_6020_Position = 0;
int32_t Target_yaw_6020_Speed = 0;
int32_t Target_pitch_6020_current = -3350;//前馈电流

//----------------底盘外参---------------------------------------------------------------

//----------------底盘舵机----------------------------------
float Target_classis1_6020_Position = 0;
float Target_classis1_6020_Speed = 0;
float Target_classis2_6020_Position = 0;
float Target_classis2_6020_Speed = 0;
float Target_classis3_6020_Position = 0;
float Target_classis3_6020_Speed = 0;
float Target_classis4_6020_Position = 0;
float Target_classis4_6020_Speed = 0;

//----------------底盘电机---------------------------------
int32_t Target_classis1_3508_Position = 0;
int32_t Target_classis1_3508_Speed = 0;
int32_t Target_classis2_3508_Position = 0;
int32_t Target_classis2_3508_Speed = 0;
int32_t Target_classis3_3508_Position = 0;
int32_t Target_classis3_3508_Speed = 0;
int32_t Target_classis4_3508_Position = 0;
int32_t Target_classis4_3508_Speed = 0;

//--------------------------------------------------------电机-------------------------------------------------------------------	

//-----pitch6020------
int16_t value = 0;
int16_t speed = 0;
int16_t Temperature = 0;
int16_t real_value = 900;

//------yaw6020-------
int16_t value2 = 0;
int16_t speed2 = 0;
int16_t Temperature2 = 0;
int16_t real_value2 = 0;


//---classis1_6020_1号舵机---0x208
int16_t value3 = 0;
int16_t speed3 = 0;
int16_t Temperature3 = 0;
int16_t real_value3 = 3435;

//---classis2_6020_2号舵机---0x206
int16_t value4 = 0;
int16_t speed4 = 0;
int16_t Temperature4 = 0;
int16_t real_value4 = 4778;

//---classis3_6020_3号舵机---0x207
int16_t value5 = 0;
int16_t speed5 = 0;
int16_t Temperature5 = 0;
int16_t real_value5 = 1357;

//---classis4_6020_4号舵机---0x209
int16_t value6 = 0;
int16_t speed6 = 0;
int16_t Temperature6 = 0;
int16_t real_value6 = 6115;



//---classis1_3508_1号底盘电机---
int16_t value7 = 0;
int16_t speed7 = 0;
int16_t Temperature7 = 0;
int16_t real_value7 = 0;

//---classis2_3508_2号底盘电机---
int16_t value8 = 0;
int16_t speed8 = 0;
int16_t Temperature8 = 0;
int16_t real_value8 = 0;

//---classis3_3508_3号底盘电机---
int16_t value9 = 0;
int16_t speed9 = 0;
int16_t Temperature9 = 0;
int16_t real_value9 = 0;

//---classis4_3508_4号底盘电机---
int16_t value10 = 0;
int16_t speed10 = 0;
int16_t Temperature10 = 0;
int16_t real_value10 = 0;


/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

	CAN_FilterTypeDef filter1;
	filter1.FilterBank=14;//滤波器编号
	filter1.FilterMode=CAN_FILTERMODE_IDMASK;//掩码模式
	filter1.FilterScale=CAN_FILTERSCALE_32BIT;
	filter1.FilterIdHigh=0x0000;
	filter1.FilterIdLow=0x0000;
	filter1.FilterMaskIdHigh=0x0000;
	filter1.FilterMaskIdLow=0x0000;
	filter1.FilterFIFOAssignment=CAN_FILTER_FIFO0;//FIFO0
	filter1.FilterActivation=ENABLE;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&filter1)!=HAL_OK)
	{
		Error_Handler();
	}
	HAL_CAN_Start(&hcan1);		
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);		

//	 if(HAL_CAN_Start(&hcan1) != HAL_OK)//打开can
//	 {
//		Error_Handler();
//	 }
//	 if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)//开启CAN1接受邮箱0挂起中断
//	 {
//		Error_Handler();
//	 }
	
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	
	CAN_FilterTypeDef filter2;
	filter2.FilterBank=1;//滤波器编号
	filter2.FilterMode=CAN_FILTERMODE_IDMASK;//掩码模式
	filter2.FilterScale=CAN_FILTERSCALE_32BIT;
	filter2.FilterIdHigh=0x0000;
	filter2.FilterIdLow=0x0000;
	filter2.FilterMaskIdHigh=0x0000;
	filter2.FilterMaskIdLow=0x0000;
	filter2.FilterFIFOAssignment=CAN_FILTER_FIFO0;//FIFO0
	filter2.FilterActivation=ENABLE;
	
	if(HAL_CAN_ConfigFilter(&hcan2,&filter2)!=HAL_OK)
	{
		Error_Handler();
	}
	HAL_CAN_Start(&hcan2);		
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);	
	
//	 if(HAL_CAN_Start(&hcan2) != HAL_OK)//打开can
//	 {
//		Error_Handler();
//	 }
//	 if(HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)//开启CAN2接受邮箱0挂起中断
//	 {
//		Error_Handler();
//	 }	
	
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_SCE_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */




uint32_t GetEncoderNumber(uint8_t DATA[8])//
{
   
	 uint32_t EncoderNumber = 0;
   EncoderNumber = (( DATA[0]<<8) | DATA[1] );
	
	
   return EncoderNumber;
}

uint16_t GetSpeed (CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef CanRxMsg, uint8_t DATA[8])//
{
   
	 HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0,&CanRxMsg, DATA);
	 uint16_t Speed = 0;
   Speed = ((DATA[2]<<8) | DATA[3]);
	
	
   return Speed;
}


uint8_t GetTemperature (CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef CanRxMsg, uint8_t DATA[8])//
{
   
	 HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0,&CanRxMsg, DATA);
	 uint8_t Temperature = 0;
   Temperature = DATA[6];
	
	
   return Temperature;
}



void Abs_Encoder_Process(ST_ENCODER*pEncoder,uint32_t value)
{
    pEncoder->PreRawValue = pEncoder->RawValue;
    pEncoder->RawValue = value;
    pEncoder->Diff = pEncoder->RawValue - pEncoder->PreRawValue;
	if(pEncoder->Diff < -pEncoder->Number/2)       //正转过0
      pEncoder->Diff += pEncoder->Number;
	else if(pEncoder->Diff > pEncoder->Number/2)   //反转过0
      pEncoder->Diff -= pEncoder->Number;
	pEncoder->SumValue += pEncoder->Diff;
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
