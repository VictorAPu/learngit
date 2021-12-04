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
    uint32_t  RawValue;          //���α�����ԭʼֵ
    uint32_t  PreRawValue;       //��һ�α�����ԭʼֵ
    int32_t   Diff;              //���α�����ԭʼֵ��ֵ
    int32_t   Number;            //����������
    float     SumValue;          //�������ۼ�ֵ
	  float     GearRatio;         //������������ٱ�
	  float     fpSpeed;           //��������������ת�٣���λ��r/min
}ST_ENCODER;

uint32_t GetEncoderNumber(uint8_t DATA[8]);
int16_t GetSpeed (uint8_t DATA[8]);
int8_t GetTemperature (uint8_t DATA[8]);

void Abs_Encoder_Process(ST_ENCODER*pEncoder,uint32_t value);
void Inc_Encoder_Process(volatile ST_ENCODER* encoder, uint32_t value);

void Angle_pitch(float target_pitch_angle);
void Angle_yaw(float target_yaw_angle);

#endif

