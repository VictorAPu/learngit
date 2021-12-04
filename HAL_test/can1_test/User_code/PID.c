#include <stdbool.h>
#include "PID.h"
#include "global_variate.h"

bool TRUE = 1;
bool FALSE = 0;

ST_PID PID_pitch_6020POS = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 100 ,.fpUMax = 15000,.fpEMin = -2000, .fpKp = 4, .fpKi = 0};
ST_PID PID_pitch_6020Vel = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 100 ,.fpUMax = 15000,.fpEMin = -2000, .fpKp = 400, .fpKi = 0};

ST_PID PID_yaw_6020POS   = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 100 ,.fpUMax = 15000,.fpEMin = -2000, .fpKp = 0, .fpKi = 0};
ST_PID PID_yaw_6020Vel   = {.fpE = 0 , .fpPreE = 0, .fpSumE = 0 ,.fpSumEMax = 100 ,.fpUMax = 15000,.fpEMin = -2000, .fpKp = 0, .fpKi = 0};


/*λ��ʽPID�㷨 u(k)=Kp*E(K)+Ki*sum(E(K))+Kd*(E(K)-E(K-1))*/

/*-------------------------------------------------------------------
�������ܣ���ͨ��PID�㷨����PID��
-------------------------------------------------------------------*/
void CalPID(ST_PID *pstPid)
{
    pstPid->fpE = pstPid->fpDes - pstPid->fpFB;//���㵱ǰƫ��
    if( pstPid->fpE <= pstPid->fpEMin )//ƫ����������
    {
        pstPid->fpE = 0;
    }
    pstPid->fpSumE += pstPid->fpE;
    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpKp * pstPid->fpE
                  + pstPid->fpKi * pstPid->fpSumE
                  + pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE);
    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��
    /*PID��������޷�*/
    if(pstPid->fpU > pstPid->fpUMax)
    {
        pstPid->fpU = pstPid->fpUMax;
    }
    else if(pstPid->fpU < -pstPid->fpUMax)
    {
        pstPid->fpU = -pstPid->fpUMax;
    }
}

/*-------------------------------------------------------------------
�������ܣ����ַ���ʽPID�㷨����PID��
��    ע�����ַ���ʽPID�Ľ��㷨�ɼ�С������ֹͣ����������ʱ�ϴ�ƫ��
          �Ի�����Ļ��ۣ��Ӷ�������ֽϴ�ĳ�����������
-------------------------------------------------------------------*/
void CalISeparatedPID(ST_PID *pstPid)
{
    uint8_t uck=1;

    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;//���㵱ǰƫ��
    if( pstPid->fpE <= pstPid->fpEMin)//ƫ����������
    {
        pstPid->fpE = 0;
    }
    pstPid->fpSumE += pstPid->fpE;//����ƫ���ۻ�
    /*��ƫ������������ۻ�ƫ��*/
    if( pstPid->fpE > pstPid->fpEiMax )//�ж��Ƿ�������ַ���
    {
        uck=0;
    }
    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpKp * pstPid->fpE
                  + pstPid->fpKi * pstPid->fpSumE * uck
                  + pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE);
    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��

    /*PID��������޷�*/
    pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}

/*-------------------------------------------------------------------
�������ܣ������ֱ���PID�㷨
��    ע��ϵͳ��һ�������˶�������ϴ���������ڼ��������ڲ����񵴻򳬵�
-------------------------------------------------------------------*/
void CalIResistedPID(ST_PID *pstPid)
{
    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;   //���㵱ǰƫ��
    pstPid->fpSumE += pstPid->fpE;   //����ƫ���ۻ�
	

    pstPid->fpSumE = Clip(pstPid->fpSumE, -pstPid->fpEiMax, pstPid->fpEiMax);
    pstPid->fpUi = pstPid->fpKi * pstPid->fpSumE;

    pstPid->fpUp = Clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);
    pstPid->fpUd = Clip(pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE), -pstPid->fpEdMax, pstPid->fpEdMax);


    /*��ƫ��������֮�ڣ�����������ۼ���*/
    if( pstPid->fpE < pstPid->fpEMin)   //�ж��Ƿ�������ֱ�������
    {
        pstPid->fpSumE = 0;   //���ƫ���ۻ�
    }
    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpUp + pstPid->fpUi + pstPid->fpUd;

    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��
    /*PID��������޷�*/
    pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}

/*-------------------------------------------------------------------
�������ܣ�������������PID�Ľ��㷨����PID��
-------------------------------------------------------------------*/
void CalIWeakenPID(ST_PID *pstPid)
{
    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;//���㵱ǰƫ��

	if(pstPid->fpE > pstPid->fpEMin)
		pstPid->fpSumE = pstPid->fpSumE + pstPid->fpEMin;
	else if(pstPid->fpE < -pstPid->fpEMin)
		pstPid->fpSumE = pstPid->fpSumE - pstPid->fpEMin;
    else
    {
        pstPid->fpSumE = pstPid->fpSumE + pstPid->fpE;//����ƫ���ۻ�
    }
	if(pstPid->fpKi!=0)
	pstPid->fpSumE = Clip(pstPid->fpSumE, -pstPid->fpEiMax/pstPid->fpKi, pstPid->fpEiMax/pstPid->fpKi);
	else pstPid->fpSumE = 0;
	pstPid->fpUi = pstPid->fpKi * pstPid->fpSumE;

    pstPid->fpUp = Clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);
    pstPid->fpUd = Clip(pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE), -pstPid->fpEdMax, pstPid->fpEdMax);

    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpUp + pstPid->fpUi + pstPid->fpUd;

    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��


    /*PID��������޷�*/
    pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}
void PowWeakenPID(ST_PID *pstPid)
{
    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;//���㵱ǰƫ��

	if(pstPid->fpE > pstPid->fpEMin)
		pstPid->fpSumE = pstPid->fpSumE + pstPid->fpEMin;
	else if(pstPid->fpE < -pstPid->fpEMin)
		pstPid->fpSumE = pstPid->fpSumE - pstPid->fpEMin;
    else
    {
        pstPid->fpSumE = pstPid->fpSumE + pstPid->fpE;//����ƫ���ۻ�
    }
	pstPid->fpSumE = Clip(pstPid->fpSumE, -pstPid->fpEiMax, pstPid->fpEiMax);
	pstPid->fpUi = pstPid->fpKi * pstPid->fpSumE;

    pstPid->fpUp = Clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);
    pstPid->fpUd = Clip(pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE), -pstPid->fpEdMax, pstPid->fpEdMax);

    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpUp + pstPid->fpUi + pstPid->fpUd;

    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��


    /*PID��������޷�*/
    pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}

/*-------------------------------------------------------------------
�������ܣ�������������PID�Ľ��㷨����PID��
-------------------------------------------------------------------*/
void ChassisFollowPID(ST_PID *pstPid)
{
    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;//���㵱ǰƫ��

    if(pstPid->fpE > pstPid->fpEMin)
    {
        pstPid->fpSumE += pstPid->fpE;//����ƫ���ۻ�
    }
	else pstPid->fpSumE = 0;
	pstPid->fpSumE = Clip(pstPid->fpSumE, -pstPid->fpEiMax/pstPid->fpKi, pstPid->fpEiMax/pstPid->fpKi);
    pstPid->fpUi = Clip(pstPid->fpKi * pstPid->fpSumE, -pstPid->fpEiMax, pstPid->fpEiMax);
    pstPid->fpUp = Clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);
    pstPid->fpUd = Clip(pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE), -pstPid->fpEdMax, pstPid->fpEdMax);

    if(pstPid->fpUd<0.02f) pstPid->fpUd = 0;

    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpUp + pstPid->fpUi + pstPid->fpUd;

    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��

    /*PID��������޷�*/
    pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);
}

float data = 0.0f;
void Chassis_AutoFetchPelletPID(ST_PID *pstPid)
{
    pstPid->fpE=pstPid->fpDes-pstPid->fpFB;//���㵱ǰƫ��

    if(pstPid->fpE < pstPid->fpEMin)
    {
        pstPid->fpSumE += pstPid->fpE;//����ƫ���ۻ�
    }

    pstPid->fpSumE = Clip(pstPid->fpSumE, -pstPid->fpEiMax, pstPid->fpEiMax);
    pstPid->fpUi = pstPid->fpKi * pstPid->fpSumE;

    pstPid->fpUp = Clip(pstPid->fpKp * pstPid->fpE, -pstPid->fpEpMax, pstPid->fpEpMax);
    pstPid->fpUd = Clip(pstPid->fpKd * (pstPid->fpE - pstPid->fpPreE), -pstPid->fpEdMax, pstPid->fpEdMax);

    /*λ��ʽPID���㹫ʽ*/
    pstPid->fpU = pstPid->fpUp + pstPid->fpUi + pstPid->fpUd;

    pstPid->fpPreE = pstPid->fpE;//���汾��ƫ��

    /*PID��������޷�*/
    pstPid->fpU = Clip(pstPid->fpU, -pstPid->fpUMax, pstPid->fpUMax);

    if(fabs(pstPid->fpE)<10.0f) pstPid->fpU = 0.0f;
}


///*--------------------------------------------------------------------------------------------------
//�������ƣ�Clip()
//�������ܣ�����������ȥ���������ֵ����Сֵ֮���ֵ����֮��������Сֵ
//--------------------------------------------------------------------------------------------------*/
float Clip(float fpValue, float fpMin, float fpMax)
{
	if(fpValue <= fpMin)
	{
		return fpMin;
	}
	else if(fpValue >= fpMax)
	{
		return fpMax;
	}
	else
	{
		return fpValue;
	}
}


///*******************************************************************
//�������ƣ�Sign_Judge(FP32 fp_Any_Number)
//�������ܣ��ж�����
//��    ע������ֵΪ1��-1�����ı����ķ���
//********************************************************************/
int Sign_Judge(float fp_Judge_Number)
{
	if(fp_Judge_Number > 0)
		return 1;
	else if(fp_Judge_Number < 0)
		return -1;
	else
		return 0;
}


bool RampSignal_uint16_t(uint16_t* p_Output, uint16_t DesValue, uint16_t Step)  //����ָ������Ŀ��ֵ��Step��������������DesValue
{
    if(abs(*p_Output-DesValue) <= Step)
    {
        *p_Output = DesValue;
        return TRUE;
    }
    else
    {
        if(*p_Output < DesValue)
            *p_Output += Step;
        else
            *p_Output -= Step;

        return FALSE;
    }
}

void TD_Function(TD *ptd)
{
    /*���ַ����������ã��ڶ�����Զ������߸���ƽ��*/
//	int fh=0;
//	float d,a0,y,a1,a2,sa,sy,a=0;
//	ptd->x = ptd->x1-ptd->aim;
//	d = ptd->r*ptd->h*ptd->h;	a0=ptd->h*ptd->x2;	y = ptd->x + a0;	a1 = sqrt(d*d+8*d*abs(y));
//	a2 = a0 + SIGN(y)*(a1-d)/2;
//	sy = (SIGN(y+d)-SIGN(y-d))/2;
//	a  = (a0+y-a2)*sy+a2;
//	sa = (SIGN(a+d)-SIGN(a-d))/2;
//	fh = -ptd->r*(a/d-SIGN(a))*sa - ptd->r*SIGN(a);
//	ptd->x1 +=  0.001*ptd->x2;
//	ptd->x2 +=  0.001*fh;
    /***/
    float d,d0,y,a0,a=0;
    ptd->x = ptd->x1 - ptd->aim;
    d = ptd->r*ptd->h;
    d0=ptd->h * d;
    y = ptd->x + ptd->h*ptd->x2;
    a0 = sqrt(d*d+8*ptd->r*fabs(y));

    if(fabs(y)>d0)
        a = ptd->x2+(a0-d)*Sign_Judge(y)/2;
    else
        a = ptd->x2 + y/ptd->h;

    if(fabs(a)>d)
        y=-1*ptd->r*Sign_Judge(a);
    else
        y=-1*ptd->r*a/d;

    ptd->x1 +=  0.001f*ptd->x2;
    ptd->x2 +=  0.001f*y;
}

