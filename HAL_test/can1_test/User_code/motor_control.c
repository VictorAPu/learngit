#include "motor_control.h"

extern ST_PID PID_pitch_6020POS;
extern ST_PID PID_pitch_6020Vel;
extern ST_PID PID_yaw_6020POS;
extern ST_PID PID_yaw_6020Vel;
ST_ENCODER encoder_pitch = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_ENCODER encoder_yaw = {.Number = 8192, .SumValue = 0, .RawValue = 0,};


/*************************************************************************
函 数 名：Abs_Encoder_Process(volatile ST_ENCODER* encoder, SINT32 value)
函数功能：RM3510电机绝对式编码器数据处理，得到转速
备    注:
*************************************************************************/
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


/*************************************************************************
函 数 名：Inc_Encoder_Process
函数功能：有刷电机增量式编码器数据处理，得到转速
备    注:
*************************************************************************/
void Inc_Encoder_Process(volatile ST_ENCODER* encoder, uint32_t value)
{
    static float fpVeltCoff;
    encoder->PreRawValue = encoder->RawValue;
    encoder->RawValue = value;
    encoder->Diff = encoder->RawValue - encoder->PreRawValue;
    if(encoder->Diff < -4000)//两次编码器的反馈值差别太大,表示绝对式编码器圈数发生了改变或增量式编码器的定时器计数器向上溢出
    {
        encoder->Diff += 65536;//定时器16位计数器计数范围0-65535共65536个数
    }
    else if(encoder->Diff > 4000)//两次编码器的反馈值差别太大,表示绝对式编码器圈数发生了改变或增量式编码器的定时器计数器向下溢出
    {
        encoder->Diff -= 65536;
    }

    fpVeltCoff = 60.0f/encoder->GearRatio/encoder->Number/0.001f/4;//0.001是指两次采样间隔1ms,4是指定时器编码器模式的4倍频

    encoder->fpSpeed = fpVeltCoff*encoder->Diff;//单位：r/min
    encoder->SumValue += encoder->Diff;//记录编码器的总数，位置闭环用
}


/*************************************************************************
函 数 名：读取3508电机编码器can通讯内容
函数功能：有刷电机增量式编码器数据处理，得到转速
备    注:
*************************************************************************/

uint32_t GetEncoderNumber(uint8_t DATA[8])//
{
   
	 uint32_t EncoderNumber = 0;
   EncoderNumber = (( DATA[0]<<8) | DATA[1] );
		
   return EncoderNumber;
}

int16_t GetSpeed (uint8_t DATA[8])//
{
   
	 uint16_t Speed = 0;
   Speed = ((DATA[2]<<8) | DATA[3]);
		
   return Speed;
}


int8_t GetTemperature (uint8_t DATA[8])//
{

	 uint8_t Temperature = 0;
   Temperature = DATA[6];
	
   return Temperature;
}

/*************************************************************************
函 数 名：角度控制，位控
函数功能：控制电机角度
备    注:
*************************************************************************/
void Angle_pitch(float target_pitch_angle)
{
  PID_pitch_6020POS.fpDes = target_pitch_angle;
	CalPID(&PID_pitch_6020POS);
	PID_pitch_6020Vel.fpDes = PID_pitch_6020POS.fpU;
	CalPID(&PID_pitch_6020Vel);

}

void Angle_yaw(float target_yaw_angle)
{
  PID_yaw_6020POS.fpDes = target_yaw_angle;
	CalPID(&PID_yaw_6020POS);
	PID_yaw_6020Vel.fpDes = PID_yaw_6020POS.fpU;
	CalPID(&PID_yaw_6020Vel);

}
