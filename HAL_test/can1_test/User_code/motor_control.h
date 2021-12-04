#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "PID.h"
typedef struct
{
    uint32_t  RawValue;          //本次编码器原始值
    uint32_t  PreRawValue;       //上一次编码器原始值
    int32_t   Diff;              //两次编码器原始值差值
    int32_t   Number;            //编码器线数
    float     SumValue;          //编码器累加值
	  float     GearRatio;         //电机减速器减速比
	  float     fpSpeed;           //电机减速器输出轴转速，单位：r/min
}ST_ENCODER;

uint32_t GetEncoderNumber(uint8_t DATA[8]);
int16_t GetSpeed (uint8_t DATA[8]);
int8_t GetTemperature (uint8_t DATA[8]);

void Abs_Encoder_Process(ST_ENCODER*pEncoder,uint32_t value);
void Inc_Encoder_Process(volatile ST_ENCODER* encoder, uint32_t value);

void Angle_pitch(float target_pitch_angle);
void Angle_yaw(float target_yaw_angle);

#endif

