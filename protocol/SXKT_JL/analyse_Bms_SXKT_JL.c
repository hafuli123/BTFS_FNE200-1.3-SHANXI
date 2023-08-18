/*
文件：analyse_Bms_HCXY_STD.c
功能：华晨鑫源
日期：2021/07/15
公司：佛山新源
作者：csj
*/

#include "fun_can.h"
#include "protocol_GB.h"
#include "bsp_gps.h"
#include "string.h"
#include "cmsis_os2.h"
#include "bsp_rtc.h"
//#include "analyse_Bms_UDS.h"
#include "SXKT_JL/protocol_GB_EX_SXKT_JL.h"

static const char* carType = "F4_DST_HCXY_TS_V1.00";
#define DEBUG
/*
版本说明：
地上铁，不需要终端开启极值运算
TS 			测试版本
S 			指标准版本

*/


static const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN3_USED_ANA = 0;								/*是否使用CAN1*/

static const uint32_t CAN1_BAUDRATE		=		500000;							/*CAN1波特率*/
static const uint32_t CAN2_BAUDRATE		=		500000;							/*CAN2波特率*/
static const uint32_t CAN3_BAUDRATE		=		500000;							/*CAN2波特率*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/

const uint8_t CALC_EXTREMUN  = 0;											/*是否计算极值 0:关闭 1:开启*/
static char realVin[18] = {0};
static void analysisTemper(uint32_t canID,const uint8_t *data);


void iniEvCanData(void)
{
//	if(fileTagSave.newFile_Flag == 0x55AA || gSysPara.store_flag != 0xAA)
//	{
//		//华晨鑫源初始化
//		strcpy(gSysPara.domain[0],"zijian.shineraynewenergy.com");
//		gSysPara.port[0] = 19006;
//		gSysPara.linkSwitch = 0x01;
//		if(!CheckVin(gSysPara.vinCode))
//		{
//			memset(gSysPara.vinCode,0,sizeof(gSysPara.vinCode));
//		}
//		System_Pare_Save();
//	}
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can3_baudrate = CAN3_BAUDRATE;
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	gSysPara.can3_used = CAN3_USED_ANA;
	CAR_TYPE = carType;
	gRealData.rechargeSysCodeLen = 0;
//	udsInit();
}

void sendSendCycleData(uint8_t ch);

/*****************************************************************
*	函 数 名: calcExtremum
*	功能说明: CAN数据解析
*	返 回 值: 无
*****************************************************************/
void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{
	uint16_t n16Val = 0;
	uint8_t n8Val = 0,bValTmp = 0;
//	udsRecvData(ch,msg);
					gRealData.carState = CARSTA_START;
	switch(msg->id)
	{
		case 0x18FAB027:
		{
				//车辆状态
//				n8Val = calcCanValue(42,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
//				if(2 == n8Val || 3 == n8Val) 
//					gRealData.carState = CARSTA_START;
//				else if(0 == n8Val) 
//					gRealData.carState = CARSTA_STOP;
//				else if(1 == n8Val) 
//					gRealData.carState = CARSTA_OTHER;
				//运行模式
				gRealData.operationState = EV;
				//档位
				n8Val = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0x7D == n8Val) 
					gRealData.stall = (bValTmp | NEUTRAL);
				else if(0xFB == n8Val) 
					gRealData.stall = (bValTmp | PART);
				else if(0xDF == n8Val) 
					gRealData.stall = (bValTmp | REVERSE);
				else if(0xFC == n8Val || 0xFA == n8Val) 
					gRealData.stall = (bValTmp | DIRVE);
				//设置DCDC状态
				n8Val = calcCanValue(10,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.dc2dcState = DCDC_WORK;
				else if(0 == n8Val) 
					gRealData.dc2dcState = DCDC_BREAK;
				else if(2 == n8Val) 
					gRealData.dc2dcState = DCDC_ABNORMAL;
				else 
					gRealData.dc2dcState = DCDC_INVALID; 
		}
		break;
		case 0x18FA3EF4:
		{
				//充电状态
				n8Val = calcCanValue(4,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;
				else if(2 == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;
				else
					gRealData.chargeState = NO_CHARGE;
		}
		break;
		case 0x0CFAB127:
		{
				//车速
				gRealData.speed = calcRealValue(8,16,1.0/256,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//加速踏板
				gRealData.acceleratorVal = (uint8_t)calcRealValue(32,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//制动踏板
				n8Val = calcRealValue(40,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,-1);
				//驱动制动状态
				SET_ACCE_BREAK();
		}
		break;
		case 0x0CFA01EF:
		{
				//驱动电机控制器温度
				gRealData.motorData[0].motorCtrTemp = (int16_t)calcRealValue(56,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机温度
				gRealData.motorData[0].motorTemp = (int16_t)calcRealValue(48,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器输入电压
				gRealData.motorData[0].motorVol = calcRealValue(0,16,0.05,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器直流母线电流
				gRealData.motorData[0].motorCur = calcRealValue(16,16,0.05,-1600,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x0CFA00EF:
		{
//				//驱动电机状态
//				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
//				if( == n8Val) 
//					gRealData.motorData[0].motorState = MOTOR_CONSUME;
//				else if( == n8Val) 
//					gRealData.motorData[0].motorState = MOTOR_GENERATION;
//				else if( == n8Val) 
//					gRealData.motorData[0].motorState = MOTOR_OFF;
//				else if( == n8Val) 
//					gRealData.motorData[0].motorState = MOTOR_READY;
				//驱动电机总数
				gRealData.motorCnt = 1;
				//驱动电机序号
				gRealData.motorData[0].motorIdx = 1;
				//驱动电机转速
				gRealData.motorData[0].motorSpeed = (int16_t)calcRealValue(8,16,0.5,-16000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩
				gRealData.motorData[0].motorTorsion = calcRealValue(24,16,1,-32000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FEE017:
		{
				//总里程
				gRealData.totalMileage = calcRealValue(32,32,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FA40F4:
		{
				//总电压
				gRealData.total_volt = calcRealValue(0,16,0.05,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysVol = gRealData.total_volt;
				//总电流
				gRealData.total_current = calcRealValue(16,16,0.05,-1600,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysCur = gRealData.total_current;
				//SOC
				gRealData.soc = (uint8_t)calcRealValue(32,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FA41F4:
		{
				//绝缘电阻
				gRealData.mohm = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FA64D5:
		{
				//可充电储能子系统个数 不大于MAX_BAT_SUBSYS_NUM
				gRealData.subBatSysCnt = 1;
				//可充电储能子系统号
				gRealData.subSysData[0].subSysIdx = 1;
				//子系统单体电池总数
				gRealData.subSysData[0].singleVolCnt = calcRealValue(8,12,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FA4AF4:
		{
				//最高电压电池子系统号
				gRealData.maxVolPack_index = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最高值
				gRealData.max_singleVol = calcRealValue(32,16,0.01,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池子系统号 
				gRealData.minVolPack_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FA48F4:
		{
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度探针序号
				gRealData.maxTemper_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度值
				gRealData.max_singleTemper = (int16_t)calcRealValue(24,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度子系统号
				gRealData.minTemperPack_index = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度探针序号
				gRealData.minTemper_index = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低单温度
				gRealData.min_singleTemper = (int16_t)calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FA61D5:
		{
			//燃料电池电压
			gRealData.fuelBatVol = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//燃料电池电流
			gRealData.fuelBatCur = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//氢燃料氢电转化率
			gRealData.batFuelConsumption = calcRealValue(32,16,0.01,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//高压DCDC工作状态
			n8Val = calcCanValue(42,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			if(0 == n8Val)
				gRealData.dc2dcState_highVol = DCDC_BREAK;
			else if(1 == n8Val)
				gRealData.dc2dcState_highVol = DCDC_WORK;
		}
		break;
		case 0x18FA62D5:
		{
			//燃料电池温度探针总数
			gRealData.fuelBatTemCnt = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//燃料电池探针温度值1
			gRealData.fuelBatTem[0] = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//燃料电池探针温度值2
			gRealData.fuelBatTem[1] = calcRealValue(40,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA6389:
		{
			//氢系统最高压力
			gRealData.maxHydrPressure = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//氢泄露最高浓度
			gRealData.maxHydrThickness = calcRealValue(24,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA4FF4:
		{
				//子系统温度探针个数
				gRealData.subSysData[0].singleTemCnt = 8;
				//解析单体温度
				analysisTemper(msg->id,msg->data);
		}
		break;
		case 0x18FA02EF:
		{
				//驱动电机控制器温度报警
				gRealData.motorCtrTemAlert = calcCanValue(12,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机温度报警	
				gRealData.motorTempAlert = calcCanValue(10,2,msg->data,HIGHTLOW_MODE,CAN_MODE);

		}
		break;
#ifdef DEBUG
		case 0x111:
		{
			gRealData.tempDiffAlert = 3;
		}
		break;
#endif
	}
//	if(NcmTxSuppoted() && osKernelGetTickCount() - recvCarActicTime < 5000)
// 		sendSendCycleData(ch);
	unpackYZTcan(ch,msg);
}

/*
功能：获取车型程序版本
描述：
*/

const char* getCarType(void)
{
	return carType;
}

/*
*********************************************************************************************************
*	函 数 名: calcExtremum
*	功能说明: 解析单体电压数据
*	形    参: canID  	 CANID
*			 			uint8_t  CAN数据
*	返 回 值: 无
*********************************************************************************************************
*/
static void analysisVol(uint32_t canID,const uint8_t *data)
{
	int i;
	//根据协议计算得到电压位置             Byte0~Byte1 本帧起始单体电压序号
	uint16_t volStartIndex = (data[0] - 1) * 3;
	
	/******根据协议计算*******/
	for(i=0;i<3;++i)
	{
		if(volStartIndex < gRealData.subSysData[0].singleVolCnt)
		{
			 gRealData.single_vol[volStartIndex++] = calcRealValue(16+i*16,16,0.001,0,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);		
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: calcExtremum
*	功能说明: 解析单体温度CAN数据
*	形    参:  canID  	CANID
*			 			 uint8_t  CAN数据
*	返 回 值: 无
*********************************************************************************************************
*/
static void analysisTemper(uint32_t canID,const uint8_t *data)
{
	int i;
	//计算电池温度序号												
	
	/******根据协议计算*******/
	for(i=0;i<8;++i)
	{
		if(i < gRealData.subSysData[0].singleTemCnt)
		{
			gRealData.single_temper[i] = calcRealValue(i*8,8,1,-40,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
		}
	}
}



