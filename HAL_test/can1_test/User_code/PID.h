#ifndef __PID_H__
#define __PID_H__


#include <stdbool.h>
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"



typedef struct
{
    float x1;
    float x2;
    float x;
    float r;
    float h;
    float T;
    float aim;
} TD;

typedef struct
{
    float preout;
    float out;
    float in;
    float off_freq;
    float samp_tim;
} ST_LPF;

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

extern ST_PID PID_pitch_6020POS;
extern ST_PID PID_pitch_6020Vel;
extern ST_PID PID_yaw_6020POS;
extern ST_PID PID_yaw_6020Vel;


void CalPID(ST_PID *pstPid);
void CalISeparatedPID(ST_PID *pstPid);
void CalIResistedPID(ST_PID *pstPid);
void CalIWeakenPID(ST_PID *pstPid);
void PowWeakenPID(ST_PID *pstPid);
void ChassisFollowPID(ST_PID *pstPid);
void Chassis_AutoFetchPelletPID(ST_PID *pstPid);
float Clip(float fpValue, float fpMin, float fpMax);
int Sign_Judge(float fp_Judge_Number);
void LpFilter(ST_LPF* lpf);
void TD_Function(TD *ptd);



#endif
