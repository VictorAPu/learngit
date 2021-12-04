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
