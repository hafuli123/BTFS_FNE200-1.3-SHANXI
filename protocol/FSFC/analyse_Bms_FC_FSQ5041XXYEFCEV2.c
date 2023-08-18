/*
文件：analyse_Bms_Battery.c
功能：扩展CAN协议
日期：2021/12/28
公司：佛山新源
作者：HYY
*/

#include "fun_can.h"
#include "protocol_GB.h"
#include "bsp_gps.h"
#include "string.h"
#include "cmsis_os2.h"
#include "bsp_rtc.h"
#include "FSFC/protocol_GB_EX_FSFC.h"

const char* carType = "FSFC_5180_GH_FV2.00";
const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN2*/

const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1波特率*/
const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2波特率*/
#define FAULTCOUNT 15																		//故障码个数
#define MAX_SUBSYS_CODE_LEN 24													//电池编码长度
#define SENDINTERVAL		1000														//1s间隔

static uint8_t isTiming = 0;
static uint32_t sendStamp[6] = {0};												//发送节拍
static uint8_t heartBeats[2] = {0};																//心跳计数

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/

const uint8_t CALC_EXTREMUN  = 0;											/*是否计算极值 0:关闭 1:开启*/
static void analysisVol(uint32_t canID,const uint8_t *data);
static void analysisTemper(uint32_t canID,const uint8_t *data);
void send_RechargeSysCode(uint32_t canID,const uint8_t *data);
static void execute_CustomFun(uint8_t ctrl,CAN_msg *msg);
static void send_HeartToCANCh(uint8_t canCh,unsigned int canid);
static void SenVin_To_Can(uint8_t canCh,unsigned int canid);
typedef struct 
{
	uint8_t interval;
	uint32_t faultCode;	
}FAULT;



static FAULT fault[3][FAULTCOUNT];
//更新故障码列表
static void updateFault(FAULT fau[],uint32_t code)
{
	uint8_t i,j;
	for(i=0;i<FAULTCOUNT-1;i++)
	{
		if(fau[i].interval == 0)
		{
			//出现无效故障码，后面故障码往前靠
			for(j=i;j<FAULTCOUNT-1;j++)
			{
				fau[j] = fau[j+1];
			}
			fau[FAULTCOUNT-1].interval = 0;
		}
	}
	if(code != 0)
	{
		for(i=0;i<FAULTCOUNT;i++)
		{
			if(fau[i].faultCode == code && fau[i].interval > 0)
			{
				//故障码已存在，刷新计时
				fau[i].interval = 12;
				return;
			}
			//前面已经对故障码进行了排序，此判定为新故障码
			else if(fau[i].interval == 0)
			{
				//新故障码
				fau[i].faultCode = code;
				fau[i].interval = 12;
				return;
			}
		}
	}
}

static void getFault(void)
{
	uint8_t i;
	//故障码超时控制
	uint8_t batFaultCnt = 0;
	uint8_t motorFaultCnt = 0;
	uint8_t otherFaultCnt = 0;
	uint32_t batFault[FAULTCOUNT]={0};
	uint32_t motorFault[FAULTCOUNT]={0};
	uint32_t otherFault[FAULTCOUNT]={0};
	for(i=0;i<FAULTCOUNT;i++)
	{
		if(fault[0][i].interval > 0)
		{
			fault[0][i].interval--;
			batFaultCnt++;
			batFault[i] = fault[0][i].faultCode;
		}
		if(fault[1][i].interval > 0)
		{
			fault[1][i].interval--;
			motorFaultCnt++;
			motorFault[i] = fault[1][i].faultCode;
		}
		if(fault[2][i].interval > 0)
		{
			fault[2][i].interval--;
			otherFaultCnt++;
			otherFault[i] = fault[2][i].faultCode;
		}
	}
	gRealData.batFaultCnt = batFaultCnt;
	memcpy(gRealData.batFault,batFault,sizeof(gRealData.batFault));
	gRealData.motorFaultCnt = motorFaultCnt;
	memcpy(gRealData.motorFault,motorFault,sizeof(gRealData.motorFault));
	gRealData.otherFaultCnt = otherFaultCnt;
	memcpy(gRealData.otherFault,otherFault,sizeof(gRealData.otherFault));
}

void iniEvCanData(void)
{
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	CAR_TYPE = carType;
	gRealData.rechargeSysCodeLen = 0;
	
	gRealData.carState = CARSTA_START;
}

const static uint8_t SEND_CAN_MIN_INTERVAL = 50;			                //CAN发送最小间隔

//static void sendCanData(uint8_t ch);										//发送CAN接口
//static void sendLockCMD(uint8_t ch,CAN_msg *msg);							//发送远程/心跳锁  指令
//static void sendIntervalData(unsigned char canCh);						    //发送周期数据

static CAN_msg msg_buf = {0,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,EXTENDED_TYPE,DATA_FRAME};
static uint32_t minInterl = 0;
static uint8_t isSendCan = 0;
static uint8_t canCh = 0xFF;
static uint16_t tempVal;
static uint16_t Voltageofsystem;
static uint32_t  BatteryNum;
static int Vnum=1;
uint8_t authResultNum;
uint8_t authResult;
/*	nstartPos:起始位
	nlen:数据长度
	factor:系数
	offset:偏移量
	pcanVal:can数据域
	hightLowMode:是否高位在前，如果高位在前需要转换为低位在前
	canMode:can传输格式（英特尔格式，摩托罗格格式）*/
void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{
	uint8_t n8Val = 0;
	uint16_t n16Val = 0;
	uint32_t n32Val = 0;
	
	uint8_t MaxFault = 0;
	uint8_t MaxBatFault = 0;
	uint8_t MaxMotFault = 0;
	uint8_t MaxOthFault = 0;
	uint8_t bValTmp;
//		gRealData.carState = CARSTA_START;
	switch(msg->id)
	{
		case 0x18EF4AEF:
		{
				//车辆状态
				n8Val = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.carState = CARSTA_START;
				else if(2 == n8Val) 
					gRealData.carState = CARSTA_STOP;
				else if(3 == n8Val) 
					gRealData.carState = CARSTA_OTHER;
				//制动踏板
				n8Val = calcCanValue(12,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,1);
				//驱动制动状态
				SET_ACCE_BREAK();
		}
		break;
		case 0x18FFACF3:
		{
				//充电状态
				n8Val = calcCanValue(3,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;
				else if(2 == n8Val) 
					gRealData.chargeState = RUN_CHARGE;
				else if(0 == n8Val) 
					gRealData.chargeState = NO_CHARGE;
				else if(3 == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;
				else if(4 == n8Val) 
					gRealData.chargeState = CHARGE_ABNORMAL;
		}
		break;
		case 0x18FF5327:
		{
				//运行模式
				n8Val = calcCanValue(3,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.operationState = EV;
				else if(2 == n8Val) 
					gRealData.operationState = EV_FV;
		}
		break;
		case 0x18FF1127:
		{
				//车速
				gRealData.speed = calcRealValue(20,12,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//档位
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val) 
					gRealData.stall = (bValTmp | NEUTRAL);
				else if(2 == n8Val) 
					gRealData.stall = (bValTmp | REVERSE);
				else if(1 == n8Val) 
					gRealData.stall = (bValTmp | DIRVE);
				else 
					gRealData.stall = (bValTmp | PART);
				//加速踏板
				gRealData.acceleratorVal = (uint8_t)calcRealValue(40,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0xC02EF22:
		{
				//总里程
				gRealData.totalMileage = calcRealValue(0,32,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			
		}
		break;
		case 0x18FFA2F3:
		{
				//总电压
				gRealData.total_volt = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysVol = gRealData.total_volt;
				//总电流
				gRealData.total_current = calcRealValue(32,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysCur = gRealData.total_current;
		}
		break;
		case 0x18FFA1F3:
		{
				//SOC
				gRealData.soc = (uint8_t)calcRealValue(16,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FF1519:
		{
				//设置DCDC状态
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(2 == n8Val) {
					gRealData.dc2dcState  = DCDC_WORK;
					gRealData.dc2dcState_highVol = DCDC_WORK;
					gRealData.dc2dcStateAlert = 0;
				} 
				else if(0 == n8Val) {
					gRealData.dc2dcState = DCDC_BREAK;
					gRealData.dc2dcState_highVol = DCDC_BREAK;	
					gRealData.dc2dcStateAlert = 0;
				}
				else if(3 == n8Val) {
					gRealData.dc2dcState = DCDC_ABNORMAL;
					gRealData.dc2dcState_highVol = DCDC_ABNORMAL;
					gRealData.dc2dcStateAlert = 1;
				}
				//DC-DC温度报警
				n8Val = calcCanValue(50,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(4 == n8Val)
					gRealData.dc2dcTemAlert = 1;
				else
					gRealData.dc2dcTemAlert = 0;
		}
		break;
		case 0x18FFA7F3:
		{
				//绝缘电阻
				gRealData.mohm = calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		case 0x18EF4EEF:
		{
				//驱动电机状态
				n8Val = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;
				else if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if(4 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;
				//驱动电机总数
				gRealData.motorCnt = 1;
				//驱动电机序号
				gRealData.motorData[0].motorIdx = 1;
				//驱动电机控制器温度
				gRealData.motorData[0].motorCtrTemp = (int16_t)calcRealValue(48,8,0.1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转速
				gRealData.motorData[0].motorSpeed = (int16_t)calcRealValue(8,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩
				gRealData.motorData[0].motorTorsion = calcRealValue(24,16,1,-2000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机温度
				gRealData.motorData[0].motorTemp = (int16_t)calcRealValue(56,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器输入电压
				gRealData.motorData[0].motorVol = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器直流母线电流
				gRealData.motorData[0].motorCur = calcRealValue(32,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18C1E928:
		{
				//燃料电池1电压
				n16Val = calcRealValue(0,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val > 0)
					gRealData.fuelBatVol = n16Val;
				else
					gRealData.fuelBatVol = 0;
				//燃料电池1电流
				n16Val = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val > 0)
					gRealData.fuelBatVol = n16Val;
				else
					gRealData.fuelBatVol = 0;
		}
		break;
		case 0x1802F0F5:
		{
				//燃料消耗率
				gRealData.batFuelConsumption = calcRealValue(48,8,0.2,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18C8E928:
		{
				//燃料电池1温度探针总数
				gRealData.fuelBatTemCnt = 2;
				//水入温度
				gRealData.fuelBatTem[0] = calcRealValue(0,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//水出温度
				gRealData.fuelBatTem[1] = calcRealValue(16,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x1805F0F5:
		{
				//氢系统中最高温度
				gRealData.maxHydrSysTem = calcRealValue(20,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//氢系统中最高温度探针代号
				gRealData.maxHydrSysTemIdx = calcRealValue(28,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//氢气最高浓度
				gRealData.maxHydrThickness = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//氢气最高浓度传感器代号
				gRealData.maxHydrThicknessIdx = calcRealValue(16,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//氢气最高压力
				gRealData.maxHydrPressure = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//氢气最高压力传感器代号
				gRealData.maxHydrPressureIdx = calcRealValue(48,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FF3327:
		{
				//最高报警等级
				gRealData.alarmLevel = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//制动系统报警
				n8Val = calcCanValue(24,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.brakingAlert = (n8Val == 0? 0:1);
		}
		break;
		case 0x18FFAAF3:
		{
				//温度差异报警
				n8Val = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.tempDiffAlert = (n8Val == 0? 0:1);
				//电池高温报警
				gRealData.batHighTempAlert = calcCanValue(16,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型过压报警
				gRealData.batHighVolAlert = calcCanValue(12,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型欠压报警
				gRealData.batLowVolAlert = calcCanValue(14,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC低报警
				gRealData.socLowAlert = calcCanValue(20,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池过压报警
				gRealData.singleBatHighVolAlert = calcCanValue(8,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池欠压报警
				gRealData.singleBattLowVolAlert = calcCanValue(10,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC过高报警
				n8Val = calcCanValue(52,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHighAlert = (n8Val == 0? 0:1);
				//SOC跳变报警
				n8Val = calcCanValue(53,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHopAlert = (n8Val == 0? 0:1);
				//可充电储能系统不匹配报警
				n8Val = calcCanValue(54,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batNotMatchAlert = (n8Val == 0? 0:1);
				//电池单体一致性差报警
				n8Val = calcCanValue(56,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.singleBatPoorConsisAlert = (n8Val == 0? 0:1);
				//绝缘报警
				gRealData.insulationFailtAlert = calcCanValue(22,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//高压互锁状态报警
				n8Val = calcCanValue(49,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.highPressInterlockStateAlert = (n8Val == 0? 0:1);
				//车载储能装置类型过充	
				gRealData.batOverCharge = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FF81F0:
		{
				//驱动电机控制器温度报警
				gRealData.motorCtrTemAlert = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//驱动电机温度报警	
				gRealData.motorTempAlert = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机故障代码列表
				n16Val = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val > 0){
					updateFault(fault[1],n16Val);
					MaxBatFault = 2;
				}
		}
		break;
		case 0x18FFA5F3:
		{
				//可充电储能故障代码列表
				n8Val = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val > 0 && n8Val <= 50)
				{
					MaxBatFault = 1;
				}
				else if(n8Val > 50 && n8Val <= 100)
				{
					MaxBatFault = 2;
				}
				else if(n8Val > 100 && n8Val <= 150)
				{
					MaxBatFault = 3;
				}
				updateFault(fault[0],n8Val);
		}
		break;
		case 0x18FF1427:
		{
				for(int i=0;i<4;i++)
				{
					n16Val = calcCanValue(i*16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(n16Val>0)
						updateFault(fault[2],n16Val);
				}
		}
		break;
		case 0x18FF1527:
		{
				for(int i=0;i<4;i++)
				{
					n16Val = calcCanValue(i*16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(n16Val>0)
						updateFault(fault[2],n16Val);
				}
		}
		break;
		case 0x18FFABF3:
		{
				//可充电储能子系统个数 不大于MAX_BAT_SUBSYS_NUM
				gRealData.subBatSysCnt = 1;
				//可充电储能子系统号
				gRealData.subSysData[0].subSysIdx = 1;
				//子系统单体电池总数
				gRealData.subSysData[0].singleVolCnt = calcCanValue(48,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//子系统温度探针个数
				gRealData.subSysData[0].singleTemCnt = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FFB1F3:
		{
			//解析单体电压
			analysisVol(msg->id,msg->data);
		}
		break;
		case 0x18FFB2F3:
		{
			//解析温度探针
			analysisTemper(msg->id,msg->data);
		}
		break;
		default:break;
	}
	
//		execute_CustomFun(ch,msg);
	
	
	
	MaxFault = (MaxFault > MaxBatFault ? MaxFault : MaxBatFault);
	MaxFault = (MaxFault > MaxMotFault ? MaxFault : MaxMotFault);
	MaxFault = (MaxFault > MaxOthFault ? MaxFault : MaxOthFault);
	
	MaxFault = (MaxFault > gRealData.tempDiffAlert ? MaxFault : gRealData.tempDiffAlert);
	MaxFault = (MaxFault > gRealData.batHighTempAlert ? MaxFault : gRealData.batHighTempAlert);
	MaxFault = (MaxFault > gRealData.batHighVolAlert ? MaxFault : gRealData.batHighVolAlert);
	MaxFault = (MaxFault > gRealData.batLowVolAlert ? MaxFault : gRealData.batLowVolAlert);
	MaxFault = (MaxFault > gRealData.socLowAlert ? MaxFault : gRealData.socLowAlert);
	MaxFault = (MaxFault > gRealData.singleBatHighVolAlert ? MaxFault : gRealData.singleBatHighVolAlert);
	MaxFault = (MaxFault > gRealData.singleBattLowVolAlert ? MaxFault : gRealData.singleBattLowVolAlert);
	MaxFault = (MaxFault > gRealData.socHighAlert ? MaxFault : gRealData.socHighAlert);
	MaxFault = (MaxFault > gRealData.socHopAlert ? MaxFault : gRealData.socHopAlert);
	MaxFault = (MaxFault > gRealData.batNotMatchAlert ? MaxFault : gRealData.batNotMatchAlert);
	MaxFault = (MaxFault > gRealData.singleBatPoorConsisAlert ? MaxFault : gRealData.singleBatPoorConsisAlert);
	MaxFault = (MaxFault > gRealData.insulationFailtAlert ? MaxFault : gRealData.insulationFailtAlert);
	MaxFault = (MaxFault > gRealData.batOverCharge ? MaxFault : gRealData.batOverCharge);
	MaxFault = (MaxFault > gRealData.highPressInterlockStateAlert ? MaxFault : gRealData.highPressInterlockStateAlert);
	MaxFault = (MaxFault > gRealData.motorCtrTemAlert ? MaxFault : gRealData.motorCtrTemAlert);
	MaxFault = (MaxFault > gRealData.motorTempAlert ? MaxFault : gRealData.motorTempAlert);
	MaxFault = (MaxFault > gRealData.dc2dcStateAlert ? MaxFault : gRealData.dc2dcStateAlert);
	MaxFault = (MaxFault > gRealData.dc2dcTemAlert ? MaxFault : gRealData.dc2dcTemAlert);
	MaxFault = (MaxFault > gRealData.brakingAlert ? MaxFault : gRealData.brakingAlert);
	
	gRealData.alarmLevel = (MaxFault > gRealData.alarmLevel ? MaxFault:gRealData.alarmLevel);
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
void analysisVol(uint32_t canID,const uint8_t *data)
{
	uint8_t volStartIndex,i;
	volStartIndex = (data[0]-1)*3;
	for(i = 0;i<3;i++){
		gRealData.single_vol[volStartIndex++]  = calcRealValue(16+i*16,16,0.001,0,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
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
void analysisTemper(uint32_t canID,const uint8_t *data)
{
	uint8_t temperStartIndex,i;
	temperStartIndex = (data[0]-1)*6;
	for(i = 0;i<6;i++){
		gRealData.single_temper[temperStartIndex++] = calcRealValue(i*8+16,8,1,-40,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
	}
}

static void execute_CustomFun(uint8_t ctrl,CAN_msg *msg)
{
	uint16_t n16Val = 0;
	uint32_t chTimes = osKernelGetTickCount();
	if(isTiming == 0)									//发送间隔校时保证上送间隔尽量相等
	{
		isTiming = 1;
		sendStamp[0] = chTimes - 900;
		
		sendStamp[1] = chTimes - 800;
		sendStamp[2] = chTimes - 600;
		sendStamp[3] = chTimes - 400;
		sendStamp[4] = chTimes - 200;
		sendStamp[5] = chTimes - 100;
	}
	if((chTimes - sendStamp[0]) >= SENDINTERVAL && ctrl == 1)
	{
		sendStamp[0] = chTimes;
		send_HeartToCANCh(1,0x1864F0D1);
	}
	else if((chTimes - sendStamp[1]) >= SENDINTERVAL && ctrl == 2)
	{
		sendStamp[1] = chTimes;
		send_HeartToCANCh(2,0x1865F0D1);
	}
	else if((chTimes - sendStamp[2]) >= SENDINTERVAL && ctrl == 2)
	{
		sendStamp[2] = chTimes;
		SenVin_To_Can(2,0x1873F0E1);
	}
	else if((chTimes - sendStamp[3]) >= SENDINTERVAL && ctrl == 2)
	{
		sendStamp[3] = chTimes;
		SenVin_To_Can(2,0x1874F0E1);
	}
	else if((chTimes - sendStamp[4]) >= SENDINTERVAL && ctrl == 2)
	{
		sendStamp[4] = chTimes;
		SenVin_To_Can(2,0x1875F0E1);
	}
	else if((chTimes - sendStamp[5]) >= SENDINTERVAL)
	{
		sendStamp[5] = chTimes;
		getFault();												//故障码超时控制		
	}
}
/*******************************************************************************
*	函 数 名: send_To_CAN
*	功能说明: 发送数据到CAN总线		发送间隔500ms
*	形    参:  无
*	返 回 值: 无
*******************************************************************************/
static void sendData_To_CAN(uint8_t canCh,unsigned int canid,unsigned char *canbuf){
	CAN_msg	msg_buf;

	msg_buf.id = canid;
	msg_buf.len = 0x08;
	msg_buf.format =  EXTENDED_FORMAT;
	msg_buf.type = DATA_FRAME;
	
	memset(&msg_buf.data[0],0xFF,8);	
	memcpy(&msg_buf.data[0],&canbuf[0],8);
		
	
	if(canCh==1 && fun_can_Get_recvCnt(1) > 0)
	{
		CAN_send(canCh,&msg_buf,0x64);
	}
	if(canCh == 2 && fun_can_Get_recvCnt(2) > 0)
	{
		CAN_send(canCh,&msg_buf,0x64);
	}
	if(canCh == 3 && fun_can_Get_recvCnt(2) > 0)
	{
		CAN_send(canCh,&msg_buf,0x64);
	}
}

/********************************************************************************************************
*	函 数 名: SenVin_To_Can
*	功能说明: 终端发送VIN
*	形    参:  canID  	CANID
*			 			 vinCode  车架号VIN
*	返 回 值: 无
*********************************************************************************************************/
static void SenVin_To_Can(uint8_t canCh,unsigned int canid){	
	uint8_t sendBuf[8] = {0};	
	switch(canid){
		case 0x1873F0E1:
			memcpy(&sendBuf[0],&gSysPara.vinCode[0],8);
			break;
		case 0x1874F0E1:
			memcpy(&sendBuf[0],&gSysPara.vinCode[8],8);
			break;
		case 0x1875F0E1:
			sendBuf[0] = gSysPara.vinCode[16];
			break;	
	}
	sendData_To_CAN(canCh,canid,sendBuf);
}

/*******************************************************************************
*	函 数 名: send_LockHeart
*	功能说明: 终端  发送心跳锁车信息/保持连接信息		发送间隔500ms
*	形    参:  canID  	CANID
*			 			 canCh	: 	BSP_CAN1 or BSP_CAN2 or BSP_CAN3
*	返 回 值: 无
*******************************************************************************/	
static void send_HeartToCANCh(uint8_t canCh,unsigned int canid)
{
	uint8_t sendBuf[8] = {0};
	
	if(fun_can_Get_recvCnt(1) > 0 && canCh==1)
	{
		heartBeats[0]++;
		sendBuf[0] = heartBeats[0];
		if(heartBeats[0]>15)
			heartBeats[0] = 0;
	}
	else if(fun_can_Get_recvCnt(2) > 0 && canCh==2)
	{
		heartBeats[1]++;
		sendBuf[0] = heartBeats[1];
		if(heartBeats[1]>15)
			heartBeats[1] = 0;
	}	
	sendData_To_CAN(canCh,canid,sendBuf);
}

void send_RechargeSysCode(uint32_t canID,const uint8_t *data){
	int i;
	//根据协议计算储能系统编码序号
	uint16_t codeIndex = (data[0]-1)*6;

	for(i=0;i<6;i++){
		if(codeIndex<MAX_SUBSYS_CODE_LEN){
				gRealData.subSysData[0].rechargeSysCode[codeIndex++] = data[i+2];	
		}
	}	
}