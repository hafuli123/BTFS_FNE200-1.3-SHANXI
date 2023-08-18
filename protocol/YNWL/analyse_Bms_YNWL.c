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
const char* carType = "GXSL_HQK5032XXYGBEVU1_FV2.00";
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
	uint8_t bValTmp = 0;
	uint8_t MaxAlarmLevel;
	gRealData.carState = CARSTA_START;
	
	gRealData.subSysData[0].singleTemCnt = 24;
	switch(msg->id)
	{
    case 0x0CF60A27:
			{
				//车辆信息
	           n8Val  = calcRealValue(33,1,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
					gRealData.operationState = EV;
				else if(n8Val == 2)
					gRealData.operationState = EV_FV;
				else if(3 == n8Val)
					gRealData.operationState = FV;
//			    //车辆状态
//				n8Val  = calcRealValue(31,1,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//				if( 1 == n8Val) 
//					gRealData.carState = CARSTA_START;
//				else if(0 == n8Val) 
//					gRealData.carState = CARSTA_STOP;
						//加速踏板行程值
				gRealData.acceleratorVal = calcRealValue(49,1,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				uint8_t bValTmp; 		 
				//档位
				n8Val = calcCanValue(46,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val  )		//或上0，运算时高位用0补齐
					gRealData.stall = (bValTmp | NEUTRAL); 													//0	N
				else if(1 == n8Val || 2 == n8Val || 3 == n8Val ||4 == n8Val)	
					gRealData.stall = (bValTmp | DIRVE);													//1	D
				else if(13 == n8Val  )
					gRealData.stall = (bValTmp | REVERSE);													//3	R
				else if(15 == n8Val  )
					gRealData.stall = (bValTmp | PART);														//3	P
				//制动踏板状态			
				n8Val = calcRealValue(50,1,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,1);//TODO...如果制动踏板为行程值，则第二个参数为-1。如果为状态值，则第二个参数为1	
			}
		break; 
			
		case 0x1881EFF3:
		{
							//充电状态
				n8Val = calcCanValue(32,1,msg->data,HIGHTLOW_MODE,CAN_MODE);							
				if(0 == n8Val) 
					gRealData.chargeState = NO_CHARGE;						//0 未充电
				else if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;					//1 停车充电
				else if(2 == n8Val) 
					gRealData.chargeState = RUN_CHARGE;						//2	行车充电
				else if(3 == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;  				//3 充电完成		
				//最高报警等级
				gRealData.alarmLevel = calcRealValue(21,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					//温度差异报警								
				gRealData.tempDiffAlert = calcCanValue(23,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//电池高温报警						
				gRealData.batHighTempAlert = calcCanValue(25,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型过压报警					
				gRealData.batHighVolAlert = calcCanValue(27,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//车载储能装置类型欠压报警					
				gRealData.batLowVolAlert = calcCanValue(29,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//SOC低报警
				gRealData.socLowAlert = calcCanValue(31,2,msg->data,HIGHTLOW_MODE,CAN_MODE); 				
				//单体电池过压报警
				gRealData.singleBatHighVolAlert = calcCanValue(33,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池欠压报警					 
				gRealData.singleBattLowVolAlert = calcCanValue(35,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC过高报警						
				n8Val = calcCanValue(51,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHighAlert = (n8Val == 0? 0:1);
				//SOC跳变报警
				n8Val = calcCanValue(52,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHopAlert = (n8Val == 0? 0:1);				
				//可充电储能系统不匹配报警
				n8Val = calcCanValue(54,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batNotMatchAlert = (n8Val == 0? 0:1);			
				//电池单体一致性差报警				 
				n8Val = calcCanValue(39,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.singleBatPoorConsisAlert = (n8Val == 0? 0:1);	
				//绝缘报警							 
				gRealData.insulationFailtAlert = calcCanValue(37,2,msg->data,HIGHTLOW_MODE,CAN_MODE);	
//				//车载储能装置类型过充报警
//				gRealData.batOverCharge = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				//高压互锁状态报警
				n8Val = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val)
					gRealData.highPressInterlockStateAlert = 0;		//0	无故障
				else if(1 == n8Val)
					gRealData.highPressInterlockStateAlert = 3;		//1	故障等级三	

		}
		break;
		
    case 0x18F80A17:
	        {
						//车速
		        gRealData.speed = calcRealValue(34,2,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
						//总里程
	           gRealData.totalMileage = calcRealValue(36,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		break;	
			
		case 0x18A127F7:
		{
						//dcdc状态
						n8Val = calcRealValue(45,1,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
						if(n8Val == 3 || n8Val == 4)
						gRealData.dc2dcState  = DCDC_BREAK;
						if(n8Val == 5)
						gRealData.dc2dcState  = DCDC_WORK;
		}
		break;
		
    case 0x18F60D27:
	        {		
				gRealData.motorCnt = 1;				
				//驱动电机序号
				gRealData.motorData[0].motorIdx= 1;
		
				//驱动电机温度
				gRealData.motorData[0].motorTemp = calcRealValue(60,1,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			    
				//驱动电机控制器温度
				gRealData.motorData[0].motorCtrTemp = calcRealValue(55,1,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机状态
				n8Val = calcCanValue(54,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;
				else if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;
				else if(0 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;
			}
			break;
	case 0x18F60C27:
	        {	
				//驱动电机转速
			    gRealData.motorData[0].motorSpeed  = calcRealValue(56,2,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩	
				gRealData.motorData[0].motorTorsion = calcRealValue(58,2,1,-2000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//电机控制器输入电压
				gRealData.motorData[0].motorVol = calcRealValue(61,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			    //电机控制器直流母线电流
				gRealData.motorData[0].motorCur = calcRealValue(63,2,1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
			
			
	case 0x1883EFF3:
	{
						//绝缘电阻
				gRealData.mohm = calcRealValue(47,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
	}
			
	case 0x18E5EFE3:
	{
				//总线电压(总电压)
				gRealData.total_volt = calcRealValue(40,2,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
	}
	break;
	
	case 0x1882EFF3:
	    {
				//充放电电流(总电流)
				gRealData.total_current = calcRealValue(42,2,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC
				gRealData.soc = calcRealValue(44,1,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
	case 0x1884EFF3:
	        {
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(24,6,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度探针单体代号
				gRealData.maxTemper_index = calcCanValue(30,5,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//最低温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(35,6,msg->data,HIGHTLOW_MODE,CAN_MODE); 
				//最低温度探针单体代号
				gRealData.maxTemper_index = calcCanValue(41,5,msg->data,HIGHTLOW_MODE,CAN_MODE);						
				//最高温度值
				gRealData.max_singleTemper = calcRealValue(0,8,1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);							
				//最低温度值
				gRealData.min_singleTemper = calcRealValue(8,8,1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
	
	case 0x1885EFF3:
	{
				//电池单体最高电压
				gRealData.max_singleVol = calcRealValue(0,13,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(13,13,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高电压电池子系统号
				gRealData.maxVolPack_index = calcCanValue(39,5,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池子系统号
				gRealData.minVolPack_index = calcCanValue(51,5,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(44,7,msg->data,HIGHTLOW_MODE,CAN_MODE);				          
				//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(56,7,msg->data,HIGHTLOW_MODE,CAN_MODE);
	}
			
	case 0x18E6EFF3:
	{
				//可充电储能子系统号
				gRealData.subSysData[0].subSysIdx = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//子系统单体电池总数
				gRealData.subSysData[0].singleVolCnt =  calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					//子系统温度探针个数
				gRealData.subSysData[0].singleTemCnt = calcCanValue(24,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
	}
	break;
	case 0x1880EFF3:
	{
		gRealData.subSysData[0].subSysIdx = calcRealValue(0,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
	}
	break;
    case 0x18C1EFF3:
			{
				//单体电压
				analysisVol(msg->id,msg->data);
			}													
			break;
		case 0x18C2EFF3:
		{
			analysisTemper(msg->id,msg->data);
		}
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



