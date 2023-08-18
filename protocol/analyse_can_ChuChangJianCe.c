/*
�ļ���analyse_can_fv.c
���ܣ�ȼ�ͳ�CAN�������
���ڣ�2019/7/10
��˾���麣Ŧ�����Զ����������޹�˾
���ߣ�chenshaojian
*/

#include "fun_can.h"
#include "bsp_rtc.h"
#include "Fun_Net.h"
#include "protocol_GB.h"

#include "cmsis_os2.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stdint.h"

#include "bsp_storage.h"
#include "algo_verify.h"


//static const char* carType = "F4_V1.3_CCCX_V1.0";
static const char* carType = "F4_FNE200V1_3_EV_V1.0";
//static const char* carType = "F4_FNE200V103_STD_Test";

static const uint8_t CAN1_USED_ANA = 1;								/*�Ƿ�ʹ��CAN1*/
static const uint8_t CAN2_USED_ANA = 1;								/*�Ƿ�ʹ��CAN1*/
static const uint8_t CAN3_USED_ANA = 0;								/*�Ƿ�ʹ��CAN1*/

static const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1������*/
static const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2������*/
static const uint32_t CAN3_BAUDRATE		=		250000;							/*CAN2������*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;		//���ݸߵ�λ
const static CANANA_MODEL CAN_MODE = INTEL;					//CAN����ģʽ����ΪIntel��ʽ��Motorola��ʽ
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;	//CAN����ƫ������ϵ������ģʽ



void iniEvCanData(void)
{
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can3_baudrate = CAN3_BAUDRATE;
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	gSysPara.can3_used = CAN3_USED_ANA;
	CAR_TYPE = carType;
}

/*
*********************************************************************************************************
*	�� �� ��: calcExtremum
*	����˵��: CAN���ݽ�������λ��ǰ
*	��    ��:  CAN_msg  	CAN�ṹ��
*	�� �� ֵ: ��
*********************************************************************************************************/
void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{

	switch(msg->id)
	{
		case 0x166:
		{
			if(msg->data[0] == 1)
			{
				gRealData.carState = CARSTA_START;
			}
			else 
			{
				gRealData.carState = CARSTA_STOP;
			}
		
		}	
		break;
		case 0x177:
		{
			switch(msg->data[0])
			{
				case 0:
					gRealData.chargeState = NO_CHARGE;			//δ���
					break;
				case 1:
					gRealData.chargeState = STOP_CHARGE;		//ͣ�����
					break;
				case 2:
					gRealData.chargeState = RUN_CHARGE;			//��ʻ���
					break;
				case 3:
					gRealData.chargeState = CHARGE_FINISH;	//������
					break;
			}
		}
		break;
		case 0x0CFE6CEE :
		{
			gRealData.speed  = calcRealValue(48,16,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
	}
	
}


