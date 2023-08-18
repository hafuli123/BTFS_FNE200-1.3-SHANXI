/*
文件：analyse_Bms_EV_STD.c
功能：出厂检测程序
日期：2021/07/15->2022/06/10
公司：佛山新源
作者：csj->LGC
*/

#include "fun_can.h"
#include "protocol_GB.h"
#include "bsp_gps.h"
#include "string.h"
#include "cmsis_os2.h"
#include "bsp_rtc.h"

static const char* carType = "F4_EV_STD_TEST";


static const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN3_USED_ANA = 0;								/*是否使用CAN1*/

static const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1波特率*/
static const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2波特率*/
static const uint32_t CAN3_BAUDRATE		=		250000;							/*CAN2波特率*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/

const uint8_t CALC_EXTREMUN  = 0;											/*是否计算极值 0:关闭 1:开启*/

void iniEvCanData(void)
{
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can3_baudrate = CAN3_BAUDRATE;
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	gSysPara.can3_used = CAN3_USED_ANA;
	CAR_TYPE = carType;
	gRealData.rechargeSysCodeLen = 0;
}

void sendSendCycleData(uint8_t ch);

/*****************************************************************
*	函 数 名: calcExtremum
*	功能说明: CAN数据解析
*	返 回 值: 无
*****************************************************************/
void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{
	gRealData.carState = CARSTA_START;					//测试有CAN就启动
	
	switch(msg->id)
	{
		case 0x235:
		{

		}
		break;
		default:break;
	}
}

/*
功能：获取车型程序版本
描述：
*/

const char* getCarType(void)
{
	return carType;
}





