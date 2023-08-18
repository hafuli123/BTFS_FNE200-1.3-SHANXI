/*
文 件：analyse_Bms_NJJL_KC_STD.c
功 能：南京金龙卡车通信协议 - 解析国标数据
日 期: 2021/12/30
公 司：北理新源(佛山)信息科技有限公司
作 者: CZJ -> LGC
*/

#include "fun_can.h"
#include "protocol_GB.h"
#include "bsp_gps.h"
#include "string.h"
#include "cmsis_os2.h"
#include "bsp_rtc.h"
#include "NJJL_KC/protocol_GB_EX_NJJL_KC.h"

#define MAX_VOL_CNT 600										//最大单体总数
#define MAX_TMP_CNT 150										//最大温度探针数
#define MAX_SUBSYS_CODE_LEN 24						//电池编码长度
#define FAULTCOUNT 15											//故障码个数

const char* carType = "F4_NJJLKC_EVFV103_T1.00";

static const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN3_USED_ANA = 0;								/*是否使用CAN1*/
static const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1波特率*/
static const uint32_t CAN2_BAUDRATE		=		500000;							/*CAN2波特率*/
static const uint32_t CAN3_BAUDRATE		=		250000;							/*CAN2波特率*/
const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/
const uint8_t CALC_EXTREMUN  = 1;											/*是否计算极值 0:关闭 1:开启*/

static void analysisVol(uint32_t canID,const uint8_t *data);
static void analysisTemper(uint32_t canID,const uint8_t *data);
static void send_RechargeSysCode(uint32_t canID,const uint8_t *data);

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
	gSysPara.can3_baudrate = CAN3_BAUDRATE;
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	gSysPara.can3_used = CAN3_USED_ANA;
	CAR_TYPE = carType;
	gRealData.rechargeSysCodeLen = 0;
	unpackSelfInit();										//企标数据初始化
}
/*****************************************************************
*	函 数 名: calcExtremum
*	功能说明: CAN数据解析
*	返 回 值: 无
*****************************************************************/
void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{
	uint8_t n8Val = 0;
	uint16_t n16Val = 0;
	
	uint8_t MaxFault = 0;
	uint8_t MaxBatFault = 0;
	uint8_t MaxMotFault = 0;
	uint8_t MaxOthFault = 0;
	
	switch(msg->id)
	{
		case 0x18FFAAF3:
			{	
				//可充电储能系统编码长度
				gRealData.rechargeSysCodeLen = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能系统编码
				send_RechargeSysCode(msg->id,msg->data);
			}
			break;
		case 0x08F010A0:
			{		
				uint8_t bValTmp = 0;
				//车辆状态
				n8Val = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0x02 == n8Val)
					gRealData.carState = CARSTA_STOP;									//2		熄火
				else if(0x01 == n8Val)
					gRealData.carState = CARSTA_START;									//1		启动
				//车速
				gRealData.speed = calcRealValue(48,16,0.1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//档位					
				n8Val = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val  )		//或上0，运算时高位用0补齐
					gRealData.stall = (bValTmp | NEUTRAL); 													//00		N	
				else if(14 == n8Val ||(n8Val>=1 && n8Val <= 12))	
					gRealData.stall = (bValTmp | DIRVE);													//14		D	
				else if(13 == n8Val  )
					gRealData.stall = (bValTmp | REVERSE);													//13		R	
				else if(15 == n8Val  )
					gRealData.stall = (bValTmp | PART);														//15		P
				//加速踏板行程值		
				gRealData.acceleratorVal = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//制动踏板状态			
				n8Val = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,-1);//TODO...如果制动踏板为行程值，则第二个参数为-1。如果为状态值，则第二个参数为1				
				//驱动制动状态
				SET_ACCE_BREAK();			
			}
			break;
		case 0x0C20A0A6:
			{
				//累计里程
				gRealData.totalMileage = calcRealValue(16,24,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//制动系统报警
				gRealData.brakingAlert = calcCanValue(10,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;			
		case 0x18FEA0A5:
			{				
				//DCDC状态	
				n8Val = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val  )
					gRealData.dc2dcState = DCDC_WORK;																	//1		工作；
				else if(0 == n8Val)
					gRealData.dc2dcState = DCDC_BREAK;																	//其他	不工作	
				//DC-DC温度报警
				gRealData.dc2dcTemAlert = calcCanValue(15,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//DC-DC状态报警
				n8Val = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				if(2 == n8Val)
					gRealData.dc2dcStateAlert = 1;	
				else
					gRealData.dc2dcStateAlert = 0;	
			}
			break;
		case 0x18FECBF3:
			{
				//温度差异报警								
				gRealData.tempDiffAlert = calcCanValue(22,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池高温报警						
				gRealData.batHighTempAlert= calcCanValue(18,2,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//车载储能装置类型过压报警					
				gRealData.batHighVolAlert = calcCanValue(8,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型欠压报警					
				gRealData.batLowVolAlert = calcCanValue(10,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//SOC低报警
				gRealData.socLowAlert = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//单体电池过压报警
				gRealData.singleBatHighVolAlert = calcCanValue(12,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池欠压报警					 
				gRealData.singleBattLowVolAlert = calcCanValue(14,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC过高报警						
				gRealData.socHighAlert = calcCanValue(24,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC跳变报警
				gRealData.socHopAlert = calcCanValue(28,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能系统不匹配报警
				gRealData.batNotMatchAlert = calcCanValue(30,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体一致性差报警				 
				gRealData.singleBatPoorConsisAlert = calcCanValue(16,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//绝缘报警							 
				gRealData.insulationFailtAlert= calcCanValue(6,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//高压互锁状态报警
				gRealData.highPressInterlockStateAlert = calcCanValue(32,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型过充报警
				gRealData.batOverCharge = calcCanValue(8,2,msg->data,HIGHTLOW_MODE,CAN_MODE);			
			}
			break;
		case 0x18FF41F0:
			{
				//驱动电机状态
				n8Val = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;						//3 关闭
				else if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;					//1	耗电
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;				//2	发电
				else if(4 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;					//4	准备	
				//驱动电机转速							
				gRealData.motorData[0].motorSpeed = calcRealValue(0,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩						
				gRealData.motorData[0].motorTorsion = calcRealValue(16,16,1,-5000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FF51F0:
			{
				//电机控制器输入电压			
				gRealData.motorData[0].motorVol = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器直流母线电流	
				gRealData.motorData[0].motorCur = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
		case 0x18FF61F0:
			{
				//驱动电机温度						
				gRealData.motorData[0].motorTemp = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机控制器温度			
				gRealData.motorData[0].motorCtrTemp = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);								
			}
			break;		
		case 0x18FFA2F3:
			{
				//总电压
				gRealData.total_volt = calcRealValue(0,12,0.2,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//总电流
				gRealData.total_current = calcRealValue(24,16,0.035,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//可充电储能装置电压
				gRealData.subSysData[0].subSysVol = gRealData.total_volt;
				//可充电储能装置电流
				gRealData.subSysData[0].subSysCur = gRealData.total_current;		
				//SOC
				gRealData.soc = calcRealValue(40,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;	
		case 0x18FFA8F3:
			{
				//绝缘电阻
				gRealData.mohm = calcRealValue(32,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);									
			}
			break;
		case 0x18FFA9F3:
			{
				//充电状态
				n8Val = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);							
				if(3 == n8Val) 
					gRealData.chargeState = NO_CHARGE;						//3 未充电
				else if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;					//1 停车充电
				else if(2 == n8Val) 
					gRealData.chargeState = RUN_CHARGE;						//2	行车充电
				else if(4 == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;  				//4 充电完成
				else if(5 == n8Val)
					gRealData.chargeState = 5;								//5 换电站充电
				//单体电池总数
				n16Val = calcCanValue(36,16,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				if(n16Val<MAX_VOL_CNT)
					gRealData.subSysData[0].singleVolCnt = n16Val;
			}
			break;			
		case 0x18FFC1F0:
			{
				//驱动电机控制器温度报警		
				n8Val = calcCanValue(40,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val < 2)
					gRealData.motorCtrTemAlert = n8Val;	
				//驱动电机温度报警				
				n8Val = calcCanValue(42,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val < 2)
					gRealData.motorTempAlert = n8Val;
				//驱动电机故障代码列表
				n16Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val != 0)
					updateFault(fault[1],n16Val);
			}
			break;
		case 0x18FECAF3:
			{
				//可充电储能装置故障代码列表
				n16Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val != 0)
					updateFault(fault[0],n16Val);			
			}
			break;
		case 0x18FFC1F3:
			{
				//最高电压电池子系统号	
				gRealData.maxVolPack_index = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(24,16,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//电池单体电压最高值
				gRealData.max_singleVol = calcRealValue(8,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FFC3F3:
			{
				//最低电压电池子系统号	 
				gRealData.minVolPack_index = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(24,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(8,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;			
		case 0x18FFC5F3:
			{
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//最高温度探针单体代号
				gRealData.maxTemper_index = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度值
				gRealData.max_singleTemper = calcRealValue(8,8,1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//最低温度子系统号
				gRealData.minTemperPack_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);					
				//最低温度探针子系统代号
				gRealData.minTemper_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);								
				//最低温度值
				gRealData.min_singleTemper = calcRealValue(32,8,1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				//可充电储能温度探针个数
				n16Val = calcCanValue(48,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val<MAX_TMP_CNT)
					gRealData.subSysData[0].singleTemCnt = n16Val;
			}
			break;
			
		default: 
			if((msg->id & 0xFF00FFFF) == 0x1800F8F4){
			
				if(((msg->id & 0x00FF0000) >>16) >= 0xE0 && ((msg->id & 0x00FF0000) >> 16) <= 0xFF){
					//解析单体温度
					analysisTemper(msg->id,msg->data);
				} 
			
				if(((msg->id & 0x00FF0000) >>16) >= 0x20 && ((msg->id & 0x00FF0000) >> 16) <= 0xDF){
					//解析单体电压
					analysisVol(msg->id,msg->data);									
				}			
			}	
			break;
	}	

	//(n8Val == 0 ? 0:1 )
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
	
	gRealData.alarmLevel = MaxFault;
	
	unpackSelfcan(ch,msg);						//企标数据解析
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
	uint16_t volStartIndex = (((canID & 0x00FF0000)>>16)-0x20)*4;
	
	/******根据协议计算*******/
	for(i=0;i<4;++i)
	{
		if(volStartIndex < gRealData.subSysData[0].singleVolCnt)
		{
			 gRealData.single_vol[volStartIndex++] = calcRealValue(i*16,16,0.001,0,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);		
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
	//计算电池温度序号												Byte0~Byte1 本帧起始单体温度序号
	uint16_t temperStartIndex = (((canID & 0x00FF0000)>>16) - 0xE0)*8;
	
	/******根据协议计算*******/
	for(i=0;i<8;++i)
	{
		if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
		{
			gRealData.single_temper[temperStartIndex++] = calcRealValue(i*8,8,1,-50,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
		}
	}
}
/*******************************************************************************
*	函 数 名: send_RechargeSysCode
*	功能说明: 解析可充电储能系统编码
*	形    参:  canID  	CANID
*			 			 uint8_t  CAN数据
*	返 回 值: 无
*******************************************************************************/
static void send_RechargeSysCode(uint32_t canID,const uint8_t *data){
	int i;
	//根据协议计算储能系统编码序号
	uint16_t codeIndex = (calcCanValue(0,4,data,HIGHTLOW_MODE,CAN_MODE)-1)*6;

	for(i=0;i<6;i++){
		if(codeIndex<MAX_SUBSYS_CODE_LEN){
				gRealData.subSysData[0].rechargeSysCode[codeIndex++] = data[i+2];	
		}
	}	
}


