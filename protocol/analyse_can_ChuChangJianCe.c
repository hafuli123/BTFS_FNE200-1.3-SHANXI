/*
文件：analyse_can_fv.c
功能：燃油车CAN处理程序
日期：2019/7/10
公司：珠海纽安特自动化技术有限公司
作者：chenshaojian
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

static const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN3_USED_ANA = 0;								/*是否使用CAN1*/

static const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1波特率*/
static const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2波特率*/
static const uint32_t CAN3_BAUDRATE		=		250000;							/*CAN2波特率*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;		//数据高地位
const static CANANA_MODEL CAN_MODE = INTEL;					//CAN接收模式，分为Intel格式和Motorola格式
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;	//CAN数据偏移量，系数计算模式



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
*	函 数 名: calcExtremum
*	功能说明: CAN数据解析，高位在前
*	形    参:  CAN_msg  	CAN结构体
*	返 回 值: 无
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
					gRealData.chargeState = NO_CHARGE;			//未充电
					break;
				case 1:
					gRealData.chargeState = STOP_CHARGE;		//停车充电
					break;
				case 2:
					gRealData.chargeState = RUN_CHARGE;			//行驶充电
					break;
				case 3:
					gRealData.chargeState = CHARGE_FINISH;	//充电完成
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


