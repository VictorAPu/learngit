#include "motor_control.h"

extern ST_PID PID_pitch_6020POS;
extern ST_PID PID_pitch_6020Vel;
extern ST_PID PID_yaw_6020POS;
extern ST_PID PID_yaw_6020Vel;
ST_ENCODER encoder_pitch = {.Number = 8192, .SumValue = 0, .RawValue = 0,};
ST_ENCODER encoder_yaw = {.Number = 8192, .SumValue = 0, .RawValue = 0,};


/*************************************************************************
�� �� ����Abs_Encoder_Process(volatile ST_ENCODER* encoder, SINT32 value)
�������ܣ�RM3510�������ʽ���������ݴ����õ�ת��
��    ע:
*************************************************************************/
void Abs_Encoder_Process(ST_ENCODER*pEncoder,uint32_t value)
{
    pEncoder->PreRawValue = pEncoder->RawValue;
    pEncoder->RawValue = value;
    pEncoder->Diff = pEncoder->RawValue - pEncoder->PreRawValue;
	if(pEncoder->Diff < -pEncoder->Number/2)       //��ת��0
      pEncoder->Diff += pEncoder->Number;
	else if(pEncoder->Diff > pEncoder->Number/2)   //��ת��0
      pEncoder->Diff -= pEncoder->Number;
	pEncoder->SumValue += pEncoder->Diff;
}


/*************************************************************************
�� �� ����Inc_Encoder_Process
�������ܣ���ˢ�������ʽ���������ݴ����õ�ת��
��    ע:
*************************************************************************/
void Inc_Encoder_Process(volatile ST_ENCODER* encoder, uint32_t value)
{
    static float fpVeltCoff;
    encoder->PreRawValue = encoder->RawValue;
    encoder->RawValue = value;
    encoder->Diff = encoder->RawValue - encoder->PreRawValue;
    if(encoder->Diff < -4000)//���α������ķ���ֵ���̫��,��ʾ����ʽ������Ȧ�������˸ı������ʽ�������Ķ�ʱ���������������
    {
        encoder->Diff += 65536;//��ʱ��16λ������������Χ0-65535��65536����
    }
    else if(encoder->Diff > 4000)//���α������ķ���ֵ���̫��,��ʾ����ʽ������Ȧ�������˸ı������ʽ�������Ķ�ʱ���������������
    {
        encoder->Diff -= 65536;
    }

    fpVeltCoff = 60.0f/encoder->GearRatio/encoder->Number/0.001f/4;//0.001��ָ���β������1ms,4��ָ��ʱ��������ģʽ��4��Ƶ

    encoder->fpSpeed = fpVeltCoff*encoder->Diff;//��λ��r/min
    encoder->SumValue += encoder->Diff;//��¼��������������λ�ñջ���
}


/*************************************************************************
�� �� ������ȡ3508���������canͨѶ����
�������ܣ���ˢ�������ʽ���������ݴ����õ�ת��
��    ע:
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
�� �� �����Ƕȿ��ƣ�λ��
�������ܣ����Ƶ���Ƕ�
��    ע:
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
