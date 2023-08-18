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
const char* carType = "FSFC_5041XLCEFCEV2_GH70_FV2.00";
const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN2*/

const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1波特率*/
const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2波特率*/
#define FAULTCOUNT 15																		//故障码个数
#define MAX_SUBSYS_CODE_LEN 24													//电池编码长度


const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/

const uint8_t CALC_EXTREMUN  = 0;											/*是否计算极值 0:关闭 1:开启*/
static void analysisVol(uint32_t canID,const uint8_t *data);
static void analysisTemper(uint32_t canID,const uint8_t *data);
void send_RechargeSysCode(uint32_t canID,const uint8_t *data);


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
	uint8_t i;
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
			}
			break;		
		case 0x18FF31F0:	
		    {		
				//制动踏板
				n8Val = calcCanValue(12,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,1);//行程值是-1，状态值为1
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
				gRealData.speed = calcRealValue(20,12,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//档位
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val) 
					gRealData.stall = (bValTmp | NEUTRAL);
				else if(2 == n8Val) 
					gRealData.stall = (bValTmp | REVERSE);
				else if(1 == n8Val) 
					gRealData.stall = (bValTmp | DIRVE);
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
				gRealData.soc = calcRealValue(16,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
 					 
		case 0x18FF1519:
			{			
				//设置DCDC状态
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val ||2 == n8Val) 
					gRealData.dc2dcState = DCDC_WORK;
				else if(0 == n8Val) 
					gRealData.dc2dcState = DCDC_BREAK;
				else if(3 == n8Val) 
					gRealData.dc2dcState = DCDC_ABNORMAL;
				else 
					gRealData.dc2dcState = DCDC_INVALID;	
				 //高压DC/DC1状态
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(2 == n8Val) 
					gRealData.dc2dcState_highVol = DCDC_WORK;
				else
					gRealData.dc2dcState_highVol = DCDC_BREAK;

			    //DC-DC温度报警
				n8Val = calcCanValue(50,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(4 == n8Val)
					gRealData.dc2dcTemAlert = 1;
			    else 
					gRealData.dc2dcTemAlert = 0;
			    //DC-DC状态报警
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(3 == n8Val && gRealData.carState == CARSTA_START)
					gRealData.dc2dcStateAlert = 1;
			    else 
					gRealData.dc2dcStateAlert = 0;

			}
		    break;
		case 0x18FFA7F3:
			{
				//绝缘电阻
				gRealData.mohm = calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
		case 0x18EF4EEF:
			{
				//驱动电机总数
				gRealData.motorCnt = 1;
				//驱动电机转速
				gRealData.motorData[0].motorSpeed = (int16_t)calcRealValue(8,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩
				gRealData.motorData[0].motorTorsion = calcRealValue(24,16,1,-2000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;

		case 0x18EF4FEF:
			{
				//驱动电机序号
				gRealData.motorData[0].motorIdx = 1;
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
				//驱动电机控制器温度
				gRealData.motorData[0].motorCtrTemp = calcRealValue(48,8,0.1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机温度
				gRealData.motorData[0].motorTemp = (int16_t)calcRealValue(56,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器输入电压
				gRealData.motorData[0].motorVol = calcRealValue(16,16,0.1	,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器直流母线电流
				gRealData.motorData[0].motorCur = calcRealValue(32,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
	
			}
		    break;
		case 0x18C1E928:
			{
               //燃料电池1电压
               gRealData.fuelBatVol = calcRealValue(0,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
               //燃料电池1电流
			   gRealData.fuelBatCur = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			   
			}
		    break;
		case 0x1802F0F5:
			{
               //燃料消耗率
               gRealData.batFuelConsumption = calcRealValue(48,8,0.2,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //燃料电池1温度探针总数
               gRealData.fuelBatTemCnt = 2;
			}
		    break;
		case 0x1805F0F5:
			{ 
			   //燃料电池温度探针值
			   gRealData.maxHydrSysTem = calcRealValue(20,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //氢系统中最高温度探针代号
			   gRealData.maxHydrSysTemIdx = calcCanValue(28,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //氢气最高浓度
			   gRealData.maxHydrThickness = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //氢气最高浓度传感器代号
			   gRealData.maxHydrThicknessIdx = calcCanValue(16,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //氢气最高压力
			   gRealData.maxHydrPressure = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //氢气最高压力传感器代号
			   gRealData.maxHydrPressureIdx = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
			
		case 0x18FFA4F3:
			{
				
				gRealData.maxVol_index = calcCanValue(40,12,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.minVol_index = calcCanValue(52,12,msg->data,HIGHTLOW_MODE,CAN_MODE);

				//电池单体电压最高值
				gRealData.max_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
		case 0x18FFA5F3:
			{
				//最高温度探针序号
				gRealData.maxTemper_index = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度探针子系统代号
				gRealData.minTemper_index = calcRealValue(48,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度值
				gRealData.max_singleTemper = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低单温度
				gRealData.min_singleTemper = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能装置故障代码列表
				n8Val = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val || 0xFF == n8Val){
					MaxBatFault = 0;
				}else{
					if(n8Val > 0 && n8Val < 51){								//1-50		一级报警
						MaxBatFault =1;
					}
					else if(n8Val > 50 && n8Val < 101){					//51-100	二级报警
						MaxBatFault =2;
					}
					else if(n8Val > 100 && n8Val < 151){				//101-150	三级报警
						MaxBatFault =3;
					}						
					updateFault(fault[0],n8Val);
				}					
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
                if(1 == n8Val)
					gRealData.motorCtrTemAlert = 3;
			    else if(0 == n8Val)
			        gRealData.motorCtrTemAlert = 0;
			    //车载储能装置类型过充
				gRealData.batOverCharge = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
		case 0x18ff81f0:
			{
				//驱动电机控制器温度报警 && 驱动电机温度报警
				n8Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.motorCtrTemAlert = (n8Val == 0? 0:1);
				gRealData.motorTempAlert = (n8Val == 0? 0:1);
				//驱动电机故障代码列表
				n16Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n16Val){
					MaxMotFault = 0;
				}else{
					MaxMotFault = 2;											//有驱动电机故障代码， 为二级报警
					updateFault(fault[1],n16Val);				
				}		
			
			}
		    break;
		case 0x18ff1427:
			{
				for(i=0;i<4;i++)
				{
					n16Val = calcCanValue(i*16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					updateFault(fault[2],n16Val);
				}
			}
		    break;
		case 0x18ff1527:
			{
				for(i=0;i<4;i++)
				{
					n16Val = calcCanValue(i*16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					updateFault(fault[2],n16Val);
				}
			}
		    break;
		case 0x18FFABF3:
			{
				//最高电压电池子系统号
				//gRealData.maxVolPack_index = 1;
				gRealData.maxVolPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池子系统号
				gRealData.minVolPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//gRealData.minVolPack_index = 1;
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//gRealData.maxTemperPack_index = 1;
				//最低温度子系统号
				gRealData.minTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//gRealData.minTemperPack_index = 1;
				//可充电储能子系统个数 不大于MAX_BAT_SUBSYS_NUM
				gRealData.subBatSysCnt = 1;
				//可充电储能子系统号
				gRealData.subSysData[0].subSysIdx = 1;
			    //单体电池总数
				gRealData.subSysData[0].singleVolCnt = calcCanValue(48,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
			    //温度探针总数
				gRealData.subSysData[0].singleTemCnt = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
		case 0x18FFB1F3:
			{
				analysisVol(msg->id,msg->data);
			}
		    break;
		case 0x18FFB2F3:
			{
				analysisTemper(msg->id,msg->data);
			}
		    break;
		case 0x18C8E928:
			{ 
			   //燃料电池温度探针值
			   gRealData.fuelBatTem[0] = calcRealValue(0,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   gRealData.fuelBatTem[1] = calcRealValue(16,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
//				gSelfData80->InWaterTem = calcRealValue(0,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
//				gSelfData80->OutWaterTem = calcRealValue(16,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
		    break;
//		case 0x18D2E928:			//自定义数据
//			{
//				//空气压缩机电压
//				gSelfData80->AirComVol = calcRealValue(32,12,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);

//				//空气压缩机电流
//				 gSelfData80->AirComCur = calcRealValue(20,12,0.5,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//	
//			}
//			break;
//		case 0x18D3E928:			//自定义数据
//			{
//				//氢气循环泵电压
//				gSelfData80->HyCyclePumpVol = calcRealValue(24,12,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//				//氢气循环泵电流
//				gSelfData80->HyCyclePumpCur = calcRealValue(16,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			}
//			break;
//		case 0x1801F0F5:		//自定义数据
//			{
//				//氢气剩余量
//				gSelfData80->HySurplus = calcRealValue(14,10,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//				gSelfData80->WarmRiskControlCommand = 0;
//				gSelfData80->AirConControlCommand = 0;
//			}
//			break;
			default: break;
	}	
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
	int i;
	//根据协议计算得到电压位置             Byte0~Byte1 本帧起始单体电压序号
	uint16_t volStartIndex = (data[0]-1)*3;
	uint16_t volTmp = 0;
	
	/******根据协议计算*******/
	for(i=0;i<3;++i)
	{
		if(volStartIndex < gRealData.subSysData[0].singleVolCnt)
		{
			 volTmp = calcCanValue(16+i*16,16,data,HIGHTLOW_MODE,CAN_MODE);
			 gRealData.single_vol[volStartIndex++] = volTmp * 0.001; 
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
void analysisTemper(uint32_t canID,const uint8_t *data)
{
	int i;
	//计算电池温度序号												Byte0~Byte1 本帧起始单体温度序号
	uint16_t temperStartIndex = (data[0]-1)*6;
	
	/******根据协议计算*******/
	for(i=0;i<6;++i)
	{
		if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
		{
			gRealData.single_temper[temperStartIndex++] = (int16_t)calcRealValue(16+i*8,8,1,-40,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
		}
	}
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