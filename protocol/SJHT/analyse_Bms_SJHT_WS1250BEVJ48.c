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
#include "bsp_storage.h"


#define FAULTCOUNT 15																		//故障码个数

const char* carType = "SJHT_WS1250_FV4.03";
const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN2*/

const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1波特率*/
const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2波特率*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/

const uint8_t CALC_EXTREMUN  = 0;											/*是否计算极值 0:关闭 1:开启*/
static void analysisVol(uint32_t canID,const uint8_t *data);
static void analysisTemper(uint32_t canID,const uint8_t *data);
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
static char realVin[18] = {0};


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
	uint8_t bValTmp = 0;
//	gRealData.carState = CARSTA_START;
	switch(msg->id)
	{
		case 0x18E1F3D0:
		{
			if(msg->data[0] == 1)
				memcpy(&realVin[0],&msg->data[1],7);
			else if(msg->data[0] == 2)
				memcpy(&realVin[7],&msg->data[1],7);
			else if(msg->data[0] == 3)
				memcpy(&realVin[14],&msg->data[1],3);
			
//			if(memcmp(realVin,gSysPara.vinCode,17) != 0 && strlen(realVin) == 17 )
//			{
//				memcpy(gSysPara.vinCode,realVin,17);
//				System_Pare_Save();
//			}

		}
		break;
		case 0x18FF43A8:
			{
          //车辆状态
				n8Val = calcCanValue(0,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.carState = CARSTA_START;
				else if(2 == n8Val) 
					gRealData.carState = CARSTA_STOP;
				//运行模式
				n8Val = calcCanValue(4,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val) 
					gRealData.operationState = EV;
				else if(1 == n8Val) 
					gRealData.operationState = EV_FV;
				else if(2 == n8Val) 
					gRealData.operationState = FV;
				//驱动电机状态
				n8Val = calcCanValue(12,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;
				else if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if(4 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;
				else if(5 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_ABNORMAL;
				//发动机状态
				n8Val = calcCanValue(16,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.engineState = ENGINE_START;
				else if(2 == n8Val) 
					gRealData.engineState = ENGINE_STOP;
				else if(3 == n8Val) 
					gRealData.engineState = ENGINE_ABNORMAL;
				//DCDC 状态
				n8Val = calcCanValue(22,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.dc2dcState = DCDC_WORK;
				else if(0 == n8Val) 
					gRealData.dc2dcState = DCDC_BREAK;
				
			}
			break;
		case 0x1882D0F3:
		{
			//总电流
			gRealData.total_current = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18E5D0F3:
		{
			//总电压
			gRealData.total_volt = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
			
		case 0x18FF44A8:
			{
				//车速
				gRealData.speed = calcRealValue(0,16,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
         //油门开度
				gRealData.acceleratorVal = calcRealValue(24,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//制动开度
				n8Val = calcRealValue(32,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,-1);
				//驱动制动状态
				SET_ACCE_BREAK();
			}
			break;
		case 0x18FEDA17:
			{
				//总行驶里程
				gRealData.totalMileage = calcRealValue(0,32,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
    case 0x18FF46A8:
			{
				//驱动电机总数
				gRealData.motorCnt = 1;
				//驱动电机序号
				gRealData.motorData[0].motorIdx = 1;
				//电机转速
				gRealData.motorData[0].motorSpeed = calcRealValue(0,16,0.25,-8000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机扭矩
				gRealData.motorData[0].motorTorsion = calcRealValue(16,16,0.1,-3200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机温度
				gRealData.motorData[0].motorTemp = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//直流母线电压
				gRealData.motorData[0].motorVol = calcRealValue(56,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机控制器温度
				gRealData.motorData[0].motorCtrTemp = calcRealValue(40,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FF32F1:
			{			
				//直流母线电流
				gRealData.motorData[0].motorCur = calcRealValue(24,12,0.4,-800,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;			
		case 0x18FF47A8:
			{
				//可充电储能装置电压
				gRealData.subSysData[0].subSysVol = calcRealValue(0,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC
				gRealData.soc = calcRealValue(8,16,0.0025,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能装置电流
				gRealData.subSysData[0].subSysCur = calcRealValue(24,16,0.025,-800,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x18FF48A8:
			{
				//档位
				n8Val = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(2 == n8Val) 
					gRealData.stall = (bValTmp | NEUTRAL);
				else if(5 == n8Val) 
					gRealData.stall = (bValTmp | PART);
				else if(3 == n8Val) 
					gRealData.stall = (bValTmp | REVERSE);
				else if(1 == n8Val) 
					gRealData.stall = (bValTmp | DIRVE);
			}
			break;
		case  0x18E6D0F3:
			{
				//子系统单体电池总数
				gRealData.subSysData[0].singleVolCnt = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//子系统温度探针个数
				gRealData.subSysData[0].singleTemCnt = calcCanValue(24,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能子系统个数 不大于MAX_BAT_SUBSYS_NUM
				//gRealData.subBatSysCnt = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x1884D0F3:
			{
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度探针序号
				gRealData.maxTemper_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度值
				gRealData.max_singleTemper = calcRealValue(0,8,1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度子系统号
				gRealData.minTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度探针序号
				gRealData.minTemper_index = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低单温度
				gRealData.min_singleTemper = calcRealValue(8,8,1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x1885D0F3:
			{
				//最高电压电池子系统号
				gRealData.maxVolPack_index = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最高值
				gRealData.max_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x1886D0F3:
			{
				//最低电压电池子系统号 
				gRealData.minVolPack_index = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x18C2D0F3:
			{
         //解析温度探针
			   analysisTemper(msg->id,msg->data);
			}
			break;
		case  0x18C1D0F3:
			{
         //解析单体电压
			   analysisVol(msg->id,msg->data);
			}
			break;
		case  0x18FF35F1:
			{
				 //当前电机系统故障码总数
				 gRealData.motorFaultCnt = calcCanValue(8,6,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //驱动电机控制器温度报警
			   gRealData.motorCtrTemAlert = calcCanValue(14,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //驱动电机温度报警
			   gRealData.motorTempAlert = calcCanValue(15,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x18FF33F1:
			{
				 //绝缘电阻
				 gRealData.mohm = calcRealValue(48,8,8,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FF99A8:
			{
				//驱动电机故障代码列表
				n8Val = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[1],n8Val);
				//可充电储能装置故障代码列表
				n8Val = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[0],n8Val);
				//其它故障代码列表
				n8Val = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[2],n8Val);
				n8Val = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[2],n8Val);
				n8Val = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[2],n8Val);
				getFault();												//故障码超时控制		
			}
			break;
		case  0x1881D0F3:
			{
					 //最高报警等级
					 gRealData.alarmLevel = calcCanValue(21,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
					 //充电状态
					 n8Val = calcCanValue(16,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
					 if(0 == n8Val)
				  gRealData.chargeState = NO_CHARGE;
			   else if(1 == n8Val)
			   {
				   n8Val = calcCanValue(19,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				   if(0 == n8Val)
						gRealData.chargeState = NO_CHARGE;
			       else if(1 == n8Val)
						gRealData.chargeState = STOP_CHARGE;
			       else if(2 == n8Val)
						gRealData.chargeState = CHARGE_FINISH;
			       else if(3 == n8Val)
						gRealData.chargeState = CHARGE_ABNORMAL;
			   }
			   //电芯温差异常报警
			   gRealData.tempDiffAlert = calcCanValue(24,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //电芯温度过高报警
			   gRealData.batHighTempAlert = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
               //PACK过压报警
			   gRealData.batHighVolAlert = calcCanValue(28,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //PACK欠压报警
			   gRealData.batLowVolAlert =  calcCanValue(30,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //绝缘报警
			   gRealData.insulationFailtAlert = calcCanValue(38,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //单体电压欠压报警
			   gRealData.singleBattLowVolAlert = calcCanValue(36,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //单体电压过高报警
			   gRealData.singleBatHighVolAlert = calcCanValue(34,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
               //SOC过低报警
			   gRealData.socLowAlert = calcCanValue(32,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //单体压差过大
			   gRealData.singleBatPoorConsisAlert = calcCanValue(40,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //SOC过高报警
			   gRealData.socHighAlert = calcCanValue(52,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //SOC跳变报警
			   gRealData.socHopAlert = calcCanValue(53,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //BMS系统不匹配报警
			   gRealData.batNotMatchAlert = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //高压互锁报警
			   gRealData.highPressInterlockStateAlert = calcCanValue(56,1,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			}
			break;			
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
static void analysisVol(uint32_t canID,const uint8_t *data)
{
	int i;
	//根据协议计算得到电压位置             Byte0~Byte1 本帧起始单体电压序号
	uint16_t volStartIndex = (data[0]-1)*3;
	
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
	uint16_t temperStartIndex = 0;
	if(data[0]%2 != 0)
	{
		temperStartIndex = (data[0]*4-4);			//计算电池温度序号												
		for(i=0;i<6;++i)
		{
			if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
			{
				gRealData.single_temper[temperStartIndex++] = calcRealValue(16+i*8,8,1,-50,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
			}
		}
	}
	else {
		temperStartIndex = (data[0]*4-2);			//计算电池温度序号												
		for(i=0;i<2;++i)
		{
			if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
			{
				gRealData.single_temper[temperStartIndex++] = calcRealValue(16+i*8,8,1,-50,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
			}
		}
	}

}