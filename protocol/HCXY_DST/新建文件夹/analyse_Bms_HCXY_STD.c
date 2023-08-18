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
#include "HCXY_DST/protocol_GB_EX_HCXY.h"

static const char* carType = "F4_DST_HCXY_TS_V1.00";

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
	
	switch(msg->id)
	{
		case 0x235:
		{
			memcpy(realVin,msg->data,8);
		}
		break;
		case 0x236:
		{
			memcpy(&realVin[8],msg->data,8);
		}
		break;
		case 0x237:
		{
			realVin[16] = msg->data[0];
			if(memcmp(realVin,gSysPara.vinCode,17) != 0 && strlen(realVin) == 17 && !CheckVin(gSysPara.vinCode))
			{
				memcpy(gSysPara.vinCode,realVin,17);
				System_Pare_Save();
				memcpy(&getVINTime,&g_system_dt,sizeof(g_system_dt));				//首次获取VIN时间
				glongd = gRealData.longd;																		//经度
				glatd = gRealData.latd;																			//纬度
				fisrtGetVIN = 1;
			}
		}
		break;
		case 0x444:
		case 0x445:
		case 0x446:
			{
				uint8_t idx = msg->id - 0x444;
				//储能系统编码17-24
				memcpy(&gRealData.subSysData[0].rechargeSysCode[idx * 8],msg->data,8);
				if(idx * 8 == gRealData.rechargeSysCodeLen)
				{
					gRealData.rechargeSysCodeLen = idx * 8 + 8;
				}
			}
			break;
		case 0x01A:
		{
			//车辆电源状态
			//recvCarActicTime = osKernelGetTickCount();
		}
		break;
		case 0x112:
			{
				//SOC
				gRealData.soc = calcRealValue(48,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最高值
				gRealData.max_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x11D:
			{
				//驱动电机温度报警
				gRealData.motorTempAlert = calcCanValue(39,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x121:
			{
				//电动机状态
				n8Val = calcCanValue(59,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val  )
					gRealData.motorData[0].motorState = MOTOR_CONSUME;
				else if(1 == n8Val  )
					gRealData.motorData[0].motorState = MOTOR_GENERATION;
				else if(2 == n8Val  )
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if(3 == n8Val  )
					gRealData.motorData[0].motorState = MOTOR_READY;
				
				//电机控制器输入电压
				gRealData.motorData[0].motorVol = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x123:
			{
				//驱动电机个数
				//gRealData.motorCnt = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.motorCnt = 1;
				//驱动电机序号
				gRealData.motorData[0].motorIdx= calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器直流母线电流
				gRealData.motorData[0].motorCur = calcRealValue(32,14,0.1,-500,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x138:
			{
				//recvCarActicTime = osKernelGetTickCount();
				//驱动电机控制器温度
				gRealData.motorData[0].motorCtrTemp = calcRealValue(40,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转速
				gRealData.motorData[0].motorSpeed = calcRealValue(16,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩
				gRealData.motorData[0].motorTorsion = calcRealValue(48,13,0.1,-400,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机温度
				gRealData.motorData[0].motorTemp = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机控制器温度报警
				gRealData.motorCtrTemAlert = calcCanValue(61,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}	
			break;
		case 0x139:
			{
				//车辆状态
				n8Val = calcCanValue(8,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val )
					gRealData.carState = CARSTA_START;
				else if(0 == n8Val )
					gRealData.carState = CARSTA_STOP;
				//运行模式
				n8Val = calcCanValue(38,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val  )
					gRealData.operationState = EV;
				else if(1 == n8Val  )
					gRealData.operationState = EV_FV;		
					//档位
				n8Val = calcCanValue(56,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val  )
					gRealData.stall = (bValTmp | NEUTRAL);
				else if(1 == n8Val || 4 ==  n8Val)
					gRealData.stall = (bValTmp | DIRVE);
				else if(2 == n8Val  )
					gRealData.stall = (bValTmp | REVERSE);
				else if(3 == n8Val  )
					gRealData.stall = (bValTmp | PART);
			
				//加速踏板行程值
				gRealData.acceleratorVal = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//制动踏板状态
				n8Val= calcRealValue(13,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.brakingVal = (n8Val == 1) ? 0x65 : 0;
				//驱动制动状态
				SET_ACCE_BREAK();
			}
			break;
		case 0x140:
			{
				//制动系统报警
				gRealData.brakingAlert = calcCanValue(9,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x164:
			{
				//最高报警等级
				n8Val = calcCanValue(8,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val<4)
					gRealData.alarmLevel = n8Val;					
				//高压互锁状态报警
				gRealData.highPressInterlockStateAlert = calcCanValue(48,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x175:
			{
				//总电压
				gRealData.total_volt = calcRealValue(24,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//总电流
				gRealData.total_current = calcRealValue(8,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能装置电压
				gRealData.subSysData[0].subSysVol = gRealData.total_volt;
				//可充电储能装置电流
				gRealData.subSysData[0].subSysCur = gRealData.total_current;
			}
			break;
		case 0x177:
			{
				//充电状态
				uint8_t n8Val1 = calcCanValue(45,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				uint8_t n8Val2 = calcCanValue(47,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//快速或慢速充电
				if(1 == n8Val1 || 1 == n8Val2)
					gRealData.chargeState = STOP_CHARGE;
				//快充或慢充充电完成，SOC等于100
				else if((2 == n8Val1 || 2 == n8Val2) && gRealData.soc == 100)
					gRealData.chargeState = CHARGE_FINISH;
				else
					gRealData.chargeState = NO_CHARGE;
				//绝缘电阻
				gRealData.mohm = calcRealValue(24,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x193:
			{
				//DCDC状态
				n8Val = calcCanValue(8,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val  )
					gRealData.dc2dcState = DCDC_WORK;
				else if(0 == n8Val  )
					gRealData.dc2dcState = DCDC_BREAK;
				//DC-DC温度报警
				gRealData.dc2dcTemAlert = calcCanValue(18,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//DC-DC状态报警
				n8Val= calcCanValue(11,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.dc2dcStateAlert = (n8Val == 0 ? 0:1);
			}
			break;
		case 0x241:
			{
				//车速
				gRealData.speed = calcRealValue(8,13,0.05625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x260:
			{
				//累计里程
				gRealData.totalMileage = calcRealValue(16,20,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x440:
			{
				//最高温度值
				gRealData.max_singleTemper = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度值
				gRealData.min_singleTemper = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
		case 0x449:
			{
				//最高电压电池子系统号
				gRealData.maxVolPack_index = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池子系统号
				gRealData.minVolPack_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度探针单体代号
				gRealData.maxTemper_index = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x44A:
			{
				//最低温度子系统号
				gRealData.minTemperPack_index = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度探针子系统代号
				gRealData.minTemper_index = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//电池单体总数
				n16Val= calcCanValue(48,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val <= 600)
					gRealData.subSysData[0].singleVolCnt = n16Val;
				//电池系统温度探针总数量
				n16Val = calcCanValue(32,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val <= 150)
					gRealData.subSysData[0].singleTemCnt = n16Val;
				//可充电储能系统数量
				gRealData.subBatSysCnt = 1;
			}
			break;
		case 0x474:
			{
				//温度差异报警
				gRealData.tempDiffAlert = calcCanValue(12,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池高温报警
				gRealData.batHighTempAlert = calcCanValue(1,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型过压报警
				gRealData.batHighVolAlert = calcCanValue(18,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型欠压报警
				gRealData.batLowVolAlert = calcCanValue(19,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC低报警
				gRealData.socLowAlert = calcCanValue(17,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池过压报警
				gRealData.singleBatHighVolAlert = calcCanValue(7,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池欠压报警
				gRealData.singleBattLowVolAlert = calcCanValue(8,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC过高报警
				gRealData.socHighAlert = calcCanValue(16,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC跳变报警
				gRealData.socHopAlert = calcCanValue(13,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能系统不匹配报警
				gRealData.batNotMatchAlert = calcCanValue(14,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体一致性差报警
				gRealData.singleBatPoorConsisAlert = calcCanValue(20,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//绝缘报警
				gRealData.insulationFailtAlert = calcCanValue(11,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型过充
				gRealData.batOverCharge = calcCanValue(15,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x441:
			{
				//单体电压
				int i;
				uint16_t volStartIndex = 0;
				volStartIndex = msg->data[0]-1;
				for(i=0;i<3;++i)
				{
					if(volStartIndex < gRealData.subSysData[0].singleVolCnt)
					{
						gRealData.single_vol[volStartIndex++] = calcRealValue((i*16)+8,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
					}
				}
				//recvCarActicTime = osKernelGetTickCount();
			}
			break;
		case 0x442:
			{
				//探针温度
				int i;
				uint16_t temperStartIndex = 0;
				temperStartIndex = msg->data[0]-1;
				for(i=0;i<7;++i)
				{
					if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
					{
						gRealData.single_temper[temperStartIndex++] = calcRealValue((i*8)+8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
					}	
				}
			}
			break;
	}
//	if(NcmTxSuppoted() && osKernelGetTickCount() - recvCarActicTime < 5000)
		sendSendCycleData(ch);
		unpackDSTcan(ch,msg);									//企标功能及企标数据
}

/*******************************************************************************	
*	函 数 名: sendTimeToCAN
*	功能说明: 发送时间到CAN总线		发送间隔200ms
*	形    参: 无
*	返 回 值: 无
*******************************************************************************/
static uint8_t sendTimeToCAN(uint8_t ch,CAN_msg *msg)
{
	static uint32_t sendCanMinTime = 0;
	static uint8_t sendTimeLife = 0;
	if(isEnableSendCANData() && osKernelGetTickCount() - sendCanMinTime >= 200)
	{
		sendCanMinTime = osKernelGetTickCount();
		sendTimeLife = (sendTimeLife + 1) % 0xFF;
		msg->id = 0x346;
		msg->data[0] = sendTimeLife;
		msg->data[1] = (uint8_t)(g_system_dt.year >> 0);
		msg->data[2] = (uint8_t)(g_system_dt.year >> 8);
		msg->data[3] = g_system_dt.month;
		msg->data[4] = g_system_dt.day;
		msg->data[5] = g_system_dt.hour;
		msg->data[6] = g_system_dt.minute;
		msg->data[7] = g_system_dt.second;
		startSendCANData(ch,msg);
		return 1;
	}
	return 0;
}


/*******************************************************************************	
*	函 数 名: sendLocalToCAN
*	功能说明: 发送位置信息到CAN总线		发送间隔200ms
*	形    参: 无
*	返 回 值: 无
*******************************************************************************/
static uint8_t sendLocalToCAN(uint8_t ch,CAN_msg *msg)
{
	static uint32_t sendCanMinTime = 0;
	if(isEnableSendCANData() && osKernelGetTickCount() - sendCanMinTime >= 200)
	{
		sendCanMinTime = osKernelGetTickCount();
		msg->id = 0x347;
		msg->data[0] = g_tGPS.PositionOk;
		msg->data[0] |= (g_tGPS.EW == 'E' ? 1 : (g_tGPS.EW == 'W' ? 2 : 0)) << 1;
		msg->data[0] |= (g_tGPS.NS == 'S' ? 1 : (g_tGPS.EW == 'N' ? 2 : 0)) << 3;
		msg->data[1] = (uint16_t)(gRealData.longd * 100) >> 0;
		msg->data[2] = (uint16_t)(gRealData.longd * 100) >> 8;
		msg->data[3] = (uint16_t)(gRealData.latd * 100) >> 0;
		msg->data[4] = (uint16_t)(gRealData.latd * 100) >> 8;
		msg->data[5] = g_tGPS.SpeedKnots * 0.1852f;
		msg->data[6] = 0;
		msg->data[6] = 0 << 0;																													//是否需要限速行车
		msg->data[6] |= 4;																		//请求VIN
		msg->data[6] |= 8;							//电池组编号请求信号
		startSendCANData(ch,msg);
		return 1;
	}
	return 0;
}

static CAN_msg msg_buf = {0,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,STANDARD_TYPE,DATA_FRAME};
void sendSendCycleData(uint8_t ch)
{
	sendTimeToCAN(ch,&msg_buf);
	sendLocalToCAN(ch,&msg_buf);
}

/*
功能：发送数据到CAN总线
描述：
*/
static uint32_t sendInterVal = 0;
const uint8_t MAX_INTERVAL = 30;
void startSendCANData(uint8_t ch,CAN_msg *msg)
{
	if(osKernelGetTickCount()-sendInterVal >= MAX_INTERVAL)
	{
		sendInterVal = osKernelGetTickCount();
		CAN_send(1,msg,100);				
	}
}

/*
功能：查询当前是否允许发送
描述：
*/
uint8_t isEnableSendCANData(void)
{
	if(osKernelGetTickCount() - sendInterVal >= MAX_INTERVAL)
	{
		return 1;
	}
	return 0;
}

/*
功能：获取车型程序版本
描述：
*/

const char* getCarType(void)
{
	return carType;
}





