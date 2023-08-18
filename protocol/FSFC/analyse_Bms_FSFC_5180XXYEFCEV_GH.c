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
const char* carType = "FSFC_5180_GH_FV2.00";
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
	
//		gRealData.carState = CARSTA_START;
	
	switch(msg->id)
	{
		case 0x18019A17:
			{
				//制动系统报警
				n8Val = calcCanValue(30,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
 				if(1 == n8Val)
					gRealData.brakingAlert = 1;							//1  报警等级一
				else
					gRealData.brakingAlert = 0;							//其他	无报警										
			}
			break;	
		case 0x1805F0F5:
			{
				//氢系统中最高温度									
				gRealData.maxHydrSysTem = calcRealValue(20,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//氢系统中最高温度探针代号					
				gRealData.maxHydrSysTemIdx = calcCanValue(28,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//氢气最高浓度
				gRealData.maxHydrThickness = calcCanValue(0,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//氢气最高浓度传感器代号
				gRealData.maxHydrThicknessIdx = calcCanValue(16,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//氢气最高压力
				gRealData.maxHydrPressure = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//氢气最高压力传感器代号
				gRealData.maxHydrPressureIdx = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;

		case 0x180CA5A6:
			{
				//车辆状态
				n8Val = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val)
					gRealData.carState = CARSTA_START;					//1		启动
				else if(0 == n8Val || 2 == n8Val)
					gRealData.carState = CARSTA_STOP;					//0或2	熄火					
				//充电状态
				n8Val = calcCanValue(4,2,msg->data,HIGHTLOW_MODE,CAN_MODE);							
				if(0 == n8Val) 
					gRealData.chargeState = NO_CHARGE;					//0 未充电
				else if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;				//1 停车充电
				else if(2 == n8Val) 
					gRealData.chargeState = RUN_CHARGE;					//2	行车充电
				else if(3 == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;  		//3 充电完成				
				//运行模式
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val)
					gRealData.operationState = EV;					//0	纯电
				else if(1 == n8Val)
					gRealData.operationState = EV_FV;				//1	混动
				else if(3 == n8Val)
					gRealData.operationState = FV;					//2	燃油	
				//车速
				gRealData.speed = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//累计里程
				gRealData.totalMileage = calcRealValue(32,24,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//总电流
				gRealData.total_current = calcRealValue(8,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//驱动电机状态
				n8Val = calcCanValue(6,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;							//0 关闭
				else if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;					//1	耗电
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;				//2	发电
				else if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;						//3	准备				
				//燃料消耗率
				gRealData.batFuelConsumption = calcRealValue(24,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				
			}
			break;
		case 0x18FF1127:
			{
				//档位	
				uint8_t bValTmp; 				
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val  )		//或上0，运算时高位用0补齐
					gRealData.stall = (bValTmp | NEUTRAL); 					//0	N
				else if(1 == n8Val  )	
					gRealData.stall = (bValTmp | DIRVE);						//1	D
				else if(2 == n8Val  )
					gRealData.stall = (bValTmp | REVERSE);					//2	R
				else if(3 == n8Val  )
					gRealData.stall = (bValTmp | PART);							//3	P
				//加速踏板行程值		
				gRealData.acceleratorVal = calcRealValue(40,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//制动踏板状态			
				n8Val = calcRealValue(32,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,-1);//TODO...如果制动踏板为行程值，则第二个参数为-1。如果为状态值，则第二个参数为1				
				//驱动制动状态
				SET_ACCE_BREAK();
			}
			break;
		case 0x18FF1518:
			{	
				//DCDC状态	
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(2 == n8Val  )
					gRealData.dc2dcState = DCDC_WORK;							//2		工作；
				else
					gRealData.dc2dcState = DCDC_BREAK;						//其他	不工作	
				
				//DC-DC状态报警
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				if(3 == n8Val && gRealData.carState == CARSTA_START){
					gRealData.dc2dcStateAlert = 1;					
				}else{
					gRealData.dc2dcStateAlert = 0;	
				}
			
				//DC-DC温度报警
				n8Val = calcCanValue(50,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
 				if(4 == n8Val)
					gRealData.dc2dcTemAlert = 1;									//4: 1级报警；其他：正常
				else
					gRealData.dc2dcTemAlert = 0;			
			}
			break;
		case 0x18FF2027:
			{
				//燃料电池电压							数值小于0时，置0，其他转发
				n16Val = calcRealValue(0,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				if(n16Val < 0)
					gRealData.fuelBatVol = 0;
				else
					gRealData.fuelBatVol = n16Val;
				//燃料电池电流							数值小于0时，置0，其他转发
				n16Val = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val < 0)
					gRealData.fuelBatCur = 0;
				else
					gRealData.fuelBatCur = n16Val;
			}
			break;
		case 0x18FF2227:
			{
				//高压DCDC状态
				n8Val = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.dc2dcState_highVol = DCDC_WORK;
				else 
					gRealData.dc2dcState_highVol = DCDC_BREAK;
				
				//燃料电池温度探针总数	
				gRealData.fuelBatTemCnt = 2;
				gRealData.fuelBatTem[0]=calcRealValue(0,16,0.1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.fuelBatTem[1]=calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FF41F0:
			{
				//驱动电机转速							
				gRealData.motorData[0].motorSpeed = calcRealValue(0,16,1,-6000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩						
				gRealData.motorData[0].motorTorsion = calcRealValue(16,16,1,-3200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器直流母线电流	
				gRealData.motorData[0].motorCur = calcRealValue(32,16,1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FF51F0:
			{
				//驱动电机控制器温度			
				gRealData.motorData[0].motorCtrTemp = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//驱动电机温度						
				gRealData.motorData[0].motorTemp = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器输入电压			
				gRealData.motorData[0].motorVol = calcRealValue(16,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
 		case 0x18FF81F0:
			{				
				//驱动电机控制器温度报警		
				n8Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(3 == n8Val)
					gRealData.motorCtrTemAlert = 2;					//3 ：2级
				else
					gRealData.motorCtrTemAlert = 0;					//其他	无故障
				
				//驱动电机温度报警				
				n8Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(4 == n8Val)
					gRealData.motorTempAlert = 2;						//4 : 2级
				else
					gRealData.motorTempAlert = 0;						//其他	无故障
										
				//驱动电机故障代码列表
				n16Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n16Val){
					gRealData.alarmLevel = 0;
				}else{
					gRealData.alarmLevel = 2;								//有驱动电机故障代码， 为二级报警
					updateFault(fault[1],n16Val);										
				}										
			}
			break;
		case 0x18FFA1F3:
			{
				//SOC
				gRealData.soc = calcRealValue(16,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FFA2F3:
			{
				//总电压
				gRealData.total_volt = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//可充电储能装置电压
				gRealData.subSysData[0].subSysVol = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能装置电流
				gRealData.subSysData[0].subSysCur = calcRealValue(32,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
		case 0x18FFA4F3:
			{
				//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(40,12,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最高值
				gRealData.max_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(52,12,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;		
		case 0x18FFA5F3:
			{
				//最高温度探针单体代号
				gRealData.maxTemper_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度值
				gRealData.max_singleTemper = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度探针子系统代号
				gRealData.minTemper_index = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度值
				gRealData.min_singleTemper = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				
				//可充电储能装置故障代码列表
				n8Val = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[0],n8Val);
				if(0!=n8Val && 0xFF!=n8Val){
					if(n8Val>=1 && n8Val<=50){								//1-50		一级报警
						gRealData.alarmLevel =1;
					}
					else if(n8Val>=51 && n8Val<=100){					//51-100	二级报警
						gRealData.alarmLevel =2;
					}
					else if(n8Val>=101 && n8Val<=150){				//101-150	三级报警
						gRealData.alarmLevel =3;
					}	
					updateFault(fault[0],n8Val);	
											
				}else{
					gRealData.alarmLevel = 0;
				}	
			}
			break;	
		case 0x18FFA7F3:
			{		
				//绝缘电阻
				gRealData.mohm = calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
		case 0x18FFAAF3:			
			{			
				//温度差异报警								
				n8Val = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.tempDiffAlert = (n8Val == 0 ? 0:1 );
				
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
 				
				//车载储能装置类型过充报警
				gRealData.batOverCharge = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				//高压互锁状态报警
				n8Val = calcCanValue(49,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val)
					gRealData.highPressInterlockStateAlert = 0;		//0	无故障
				else if(1 == n8Val)
					gRealData.highPressInterlockStateAlert = 3;		//1	故障等级三						
 			}
			break;		
		case 0x18FFABF3:	 
			{
				//最高电压电池子系统号
				gRealData.maxVolPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池子系统号
				gRealData.minVolPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度子系统号
				gRealData.minTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				//可充电储能子系统个数			纯电车为1
				gRealData.subBatSysCnt = 1;
				//可充电储能子系统号			直接发1
				gRealData.subSysData[0].subSysIdx = 1;

				//单体总数
				gRealData.subSysData[0].singleVolCnt= calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//温度探针总数
				gRealData.subSysData[0].singleTemCnt = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
			break;
		case 0x18FFB3F3:	
			{				
				//可充电储能系统编码长度
				gRealData.rechargeSysCodeLen = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能系统编码
				send_RechargeSysCode(msg->id,msg->data);		
			}
			break;	
		case 0x00FF3327:
			{
				//最高报警等级
				gRealData.alarmLevel = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);		
			}
			break;

		case 0x18FF1427:
		case 0x18FF1627:
			{	
				for(i = 0;i<8;i++){
					n8Val = calcCanValue(i*8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(0 != n8Val)
						updateFault(fault[2],n8Val);
				}	
			}
			break;
		case 0x18FF1527:
			{
				for(i = 0;i<6;i++){
					n8Val = calcCanValue(i*8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(0 != n8Val)
						updateFault(fault[2],n8Val);
				}	
				n16Val = calcCanValue(48,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 != n16Val)
					updateFault(fault[2],n16Val);
			}
			break;
		case 0x18FF2427:
			{
				for(i = 0;i<4;i++){
					n16Val = calcCanValue(i*16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(0 != n16Val)
						updateFault(fault[2],n16Val);
				}		
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
				//解析单体温度
				analysisTemper(msg->id,msg->data);
			}		
			break;
//			case 0x04FF3312:
//			{			
//         //水入温度
//				gSelfData80->InWaterTem = calcRealValue(0,16,0.1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			    //水出温度
//				gSelfData80->OutWaterTem = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
//			}
//			break;
//			
//		case 0x04FF3712:			//自定义数据
//			{
//          //空气压缩机电压
//				gSelfData80->AirComVol = calcRealValue(24,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			    //空气压缩机电流
//				gSelfData80->AirComCur = calcRealValue(40,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			}
//			break;
//		case 0x04FF3912:			//自定义数据
//			{
//           //氢气循环泵电压
//				gSelfData80->HyCyclePumpVol = calcRealValue(8,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//           //氢气循环泵电流
//				gSelfData80->HyCyclePumpCur = calcRealValue(16,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			}
//			break;
//		case 0x1801F0F5:		//自定义数据
//			{
//                //氢气剩余量
//				gSelfData80->HySurplus = calcRealValue(14,10,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			    gSelfData80->AirConControlCommand = 0;
//			    gSelfData80->WarmRiskControlCommand = 0;
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