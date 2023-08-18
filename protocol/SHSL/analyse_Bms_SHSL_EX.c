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
#include "analyse_Bms_UDS.h"
#include "HCXY_DST/protocol_GB_EX_HCXY.h"

static const char* carType = "F4_DST_HCXY_TS_V1.11";
extern void unpackDSTcan(uint8_t ch,CAN_msg *msg);			/* 地上铁CAN参数解析 */
extern RTC_INFOR getVINTime;														/* 首次获取VIN的时间 */
extern double glongd;									            	 		//经度
extern double glatd;									              		//纬度
extern uint8_t fisrtGetVIN;															/* 是否时首次获取VIN 1是，0否 */
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
char realVin[18] = {0};
extern uint8_t isChgVIN;
static void analysisVol(uint32_t canID,const uint8_t *data);
static void analysisTemper(uint32_t canID,const uint8_t *data);

void iniEvCanData(void)
{
	if(bsp_first_start() == 1 || gSysPara.store_flag != 0xAA)
	{
		//华晨鑫源初始化
		strcpy(gSysPara.domain[0],"depot32960.flybees.com.cn");
		gSysPara.port[0] = 6658;		
		strcpy(gSysPara.domain[1],"zijian.shineraynewenergy.com");
		gSysPara.port[1] = 19006;
		gSysPara.linkSwitch = 0x03;
		if(!CheckVin(gSysPara.vinCode))
		{
			memset(gSysPara.vinCode,0,sizeof(gSysPara.vinCode));
		}
		System_Pare_Save();
	}
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can3_baudrate = CAN3_BAUDRATE;
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	gSysPara.can3_used = CAN3_USED_ANA;
	CAR_TYPE = carType;
	gRealData.rechargeSysCodeLen = 0;
	//udsInit(1);
}

void sendSendCycleData(uint8_t ch);

/*****************************************************************
*	函 数 名: calcExtremum
*	功能说明: CAN数据解析
*	返 回 值: 无
*****************************************************************/
void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{   
	uint8_t n8Val = 0;
	uint16_t n16Val = 0;
	uint8_t bValTmp = 0;
	uint8_t MaxAlarmLevel;
	gRealData.carState = CARSTA_START;
	
	gRealData.subSysData[0].singleTemCnt = 24;
	switch(msg->id)
	{
		case 0x180889D5:
		{
			memcpy(realVin,msg->data,8);
		}
		break;
		case 0x180989D5:
		{
			memcpy(&realVin[8],msg->data,8);
		}
		break;
		case 0x181089D5:
		{
			realVin[16] = msg->data[0];
			if(memcmp(realVin,gSysPara.vinCode,17) != 0 && strlen(realVin) == 17 )   //&& CheckVin(realVin)
			{
				memcpy(gSysPara.vinCode,realVin,17);
				System_Pare_Save();
			}
				memcpy(&getVINTime,&g_system_dt,sizeof(g_system_dt));				//首次获取VIN时间
				fisrtGetVIN = 1;
				
				isChgVIN = 1;																								//检测到VIN变更
		}
    case 0x0C35D5F4:
			{
				//总里程
	            gRealData.totalMileage = calcRealValue(0,32,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		break;
    case 0x1811D5D0:
			{
				//车辆信息
	           n8Val  = calcRealValue(0,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
					gRealData.operationState = EV;
				else if(n8Val == 2)
					gRealData.operationState = EV_FV;
				else if(3 == n8Val)
					gRealData.operationState = FV;
//			    //车辆状态
//				n8Val  = calcRealValue(4,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//				if( 1 == n8Val) 
//					gRealData.carState = CARSTA_START;
//				else if(2 == n8Val) 
//					gRealData.carState = CARSTA_STOP;
//				else if(3 == n8Val) 
//					gRealData.carState = CARSTA_OTHER;
			}
		break; 
    case 0x183DF4D0:
	        {
						//车速
		        gRealData.speed = calcRealValue(0,15,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
						//dcdc状态
						n8Val = calcRealValue(48,1,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
						if(n8Val == 0)
						gRealData.dc2dcState  = DCDC_BREAK;
						if(n8Val == 1)
						gRealData.dc2dcState  = DCDC_WORK;
							n8Val = calcRealValue(49,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
						if(n8Val == 0)
								gRealData.stall = NEUTRAL;
						if(n8Val == 1)
								gRealData.stall = 0x0E;
						if(n8Val == 2)
								gRealData.stall = 0X0D;	
						gRealData.alarmLevel = calcRealValue(53,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
						//加速踏板行程值
						gRealData.acceleratorVal = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		break;	
    case 0x1842F4D0:
	        {		
				gRealData.motorCnt = 1;				
				//驱动电机序号
				gRealData.motorData[0].motorIdx= calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩	
				gRealData.motorData[0].motorTorsion = calcRealValue(8,16,1,-5000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//驱动电机温度
				gRealData.motorData[0].motorTemp = calcRealValue(40,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			    
				//驱动电机控制器温度
				gRealData.motorData[0].motorCtrTemp = calcRealValue(48,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机状态
				n8Val = calcCanValue(56,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;
				else if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;
				//制动力	
				n8Val = calcCanValue(58,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
                if(n8Val == 1)
				    gRealData.stall	|= 	1<<4;
				//驱动力		
			    n8Val = calcCanValue(59,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
				    gRealData.stall	|= 1<<5;
			}
			break;
	case 0x1841F4D0:
	        {	
				//驱动电机转速(低字节)
				n16Val = calcRealValue(0,16,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		        gRealData.motorData[0].motorSpeed |= n16Val;
		        //驱动电机转速(高字节)
				n16Val = calcRealValue(16,16,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			    gRealData.motorData[0].motorSpeed |= n16Val<<8;
				//电机控制器输入电压
				gRealData.motorData[0].motorVol = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			    //电机控制器直流母线电流
				gRealData.motorData[0].motorCur = calcRealValue(48,16,0.05,-1600,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
	case 0x18F7F4D4:
	        {
				//报警级别	
                n8Val = calcRealValue(4,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
				    gRealData.alarmLevel = 2;
				if(n8Val == 2)
				    gRealData.alarmLevel = 3;
				if(n8Val == 3)
				    gRealData.alarmLevel = 1;		
				//绝缘电阻
				gRealData.mohm = calcRealValue(8,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
	case 0x1806F489:
	        {
			  //总线电压(总电压)
				gRealData.total_volt = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//充放电电流(总电流)
				gRealData.total_current = calcRealValue(16,16,0.1,-3200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC
				gRealData.soc = calcRealValue(32,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
	case 0x1809D589:
	        {
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(0,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度探针单体代号
				gRealData.maxTemper_index = calcCanValue(4,12,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//最低温度子系统号
				gRealData.minTemperPack_index = calcCanValue(16,4,msg->data,HIGHTLOW_MODE,CAN_MODE);               
			    //最低温度探针子系统代号
				gRealData.minTemper_index = calcCanValue(20,12,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//最低温度值
				gRealData.min_singleTemper = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(40,12,msg->data,HIGHTLOW_MODE,CAN_MODE);				          
				//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(52,12,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
	case 0x1810D589:
			{
				//可充电储能子系统号
				gRealData.subSysData[0].subSysIdx = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				analysisTemper(msg->id,msg->data);
			}
			break;	
    case 0x1811D589:
			{
				//单体电压
				analysisVol(msg->id,msg->data);
			}													
			break;
	case 0x0CF00400:
			{
				//曲率转数
				gRealData.crankshaftSpeed = calcRealValue(24,16,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
	case 0x18FEF200:
			{
				//燃料消耗率
				gRealData.fuelConsumption = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);  
			}
			break;
/**********************************************************************************************/		
	//非监控平台接收
	case 0x182BD589:
			{
			//电池单体最高电压
			gRealData.max_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//电池单体电压最低值
			gRealData.min_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//单体电池总数
			gRealData.subSysData[0].singleVolCnt = calcCanValue(48,12,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
			break;
	case 0x1807D5D7:
			{		
				//燃料电池温度探针总数
				gRealData.fuelBatTemCnt = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	 	
			}
			break;
	case 0x1807F489:
			{
				//电池单体电压最高值
				gRealData.min_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.max_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度值
				gRealData.max_singleTemper = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}	
			break;
	case 0x186DD589:
	        {
				//最低电压电池子系统号
				gRealData.minVolPack_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高电压电池单体代号
				gRealData.maxVolPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;		
			
				
	}
	sendSendCycleData(ch);
}
/***********************************************************************	
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
		memset(msg->data,0,sizeof(msg->data));
		msg->id = 0x1808F489;
		msg->data[0] = 1;
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
	//计算电池温度序号												
	uint16_t temperStartIndex = (data[0]-1)*6;
	
	/******根据协议计算*******/
	for(i=0;i<6;++i)
	{
		if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
		{
			gRealData.single_temper[temperStartIndex++] = calcRealValue(16+i*8,8,1,-40,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
		}
	}
}


