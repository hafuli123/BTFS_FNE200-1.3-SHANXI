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
const char* carType = "HBSH_FV2.02";
const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN2*/

const uint32_t CAN1_BAUDRATE		=		500000;							/*CAN1波特率*/
const uint32_t CAN2_BAUDRATE		=		500000;							/*CAN2波特率*/

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
//	gRealData.carState = CARSTA_START;
	
	switch(msg->id)
	{	
		case 0x348:
		{
				n8Val = calcRealValue(0,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,1);//TODO...如果制动踏板为行程值，则第二个参数为-1。如果为状态值，则第二个参数为1
		}
		break;
			
		case 0x355:
		{
					//车辆信息
	           n8Val  = calcRealValue(8,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
					gRealData.operationState = EV;
				else if(n8Val == 2)
					gRealData.operationState = EV_FV;
			    //车辆状态
				n8Val  = calcRealValue(16,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( 1 == n8Val) 
					gRealData.carState = CARSTA_START;
				else if(2 == n8Val) 
					gRealData.carState = CARSTA_STOP;
				else if(3 == n8Val) 
					gRealData.carState = CARSTA_OTHER;
                //dcdc状态
				n8Val = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 2)
				gRealData.dc2dcState  = DCDC_BREAK;
				if(n8Val == 1)
				gRealData.dc2dcState  = DCDC_WORK;
                //档位
					n8Val = calcRealValue(32,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
					if(n8Val == 0)
							gRealData.stall = NEUTRAL;
					if(n8Val == 15)
							gRealData.stall = PART;
					if(n8Val == 13)
							gRealData.stall = REVERSE;	
					if(n8Val == 14 || (n8Val>=1 && n8Val<=6))
							gRealData.stall = DIRVE;
                gRealData.motorCnt = 1;				
				//驱动电机序号
				gRealData.motorData[0].motorIdx= 1;
				//电机控制器直流母线电流
				gRealData.motorData[0].motorCur = calcRealValue(48,8,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x42D:
		{
				//充电状态
				n8Val = calcCanValue(10,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( 0xB== n8Val) 
					gRealData.chargeState = STOP_CHARGE;
				else if( 0xA== n8Val) 
					gRealData.chargeState = RUN_CHARGE;
				else if( 0 == n8Val) 
					gRealData.chargeState = NO_CHARGE;
				else if(0xC == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;
		}
		break;
		

		case 0x456:
		{
						//加速踏板行程值
						gRealData.acceleratorVal = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
						//驱动踏板行程值
//						gRealData.brakingVal 
                        n8Val= calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
            			SET_BREAK_STATE(n8Val,-1);//行程值是-1，状态值为1
						//车速
		        gRealData.speed = calcRealValue(24,13,0.05625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x26C:
		{
			//DCDC温度报警
			 n8Val = calcRealValue(8,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			if(n8Val == 1)
			gRealData.dc2dcTemAlert = 1;
			else 
			gRealData.dc2dcTemAlert = 0;
			//DCDC状态报警
			n8Val = calcRealValue(26,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			if(n8Val == 1)
			gRealData.dc2dcStateAlert = 1;
			else 
			gRealData.dc2dcStateAlert = 0;
			
		}
		break;
		
		case 0x2A:
		{
				//驱动电机转矩	
				gRealData.motorData[0].motorTorsion = calcRealValue(16,16,0.1,-400,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//驱动电机状态
				n8Val = calcCanValue(48,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;
		}
		break;
		
		case 0x2B:
		{
			//驱动电机转速
			gRealData.motorData[0].motorSpeed = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机温度
				gRealData.motorData[0].motorTemp = calcRealValue(24,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			    
				//驱动电机控制器温度
				gRealData.motorData[0].motorCtrTemp = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		

		
		case 0x090:
		{
			//发动机转速
				n8Val = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
				gRealData.crankshaftSpeed = ENGINE_STOP;
				else if(n8Val == 2)
				gRealData.crankshaftSpeed = ENGINE_START;		
		}
		break;
		
		case 0x091:
		{
			//发动机状态
			gRealData.engineState = calcRealValue(48,3,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x096:
		{
			//发动机故障个数
			gRealData.engineFaultCnt = calcRealValue(0,6,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x25C:
		{
			//总里程
	    gRealData.totalMileage = calcRealValue(0,27,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x416:
		{
				//总线电压(总电压)
				gRealData.total_volt = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//充放电电流(总电流)
				gRealData.total_current = calcRealValue(0,16,0.1,-500,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC
				gRealData.soc = calcRealValue(42,10,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x427:
		{
				//绝缘电阻
				gRealData.mohm = calcRealValue(32,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x452:
		{
			  //单体电池总数
				gRealData.subSysData[0].singleVolCnt = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//子系统温度探针个数
				gRealData.subSysData[0].singleTemCnt = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能子系统个数 
				gRealData.subBatSysCnt = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能子系统号
				gRealData.subSysData[0].subSysIdx = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
    case 0x41E:
		{
							//最高电压电池子系统号
				gRealData.maxVolPack_index = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
							//最高电压电池单体代号
				gRealData.maxVol_index = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
						//电池单体最高电压
				gRealData.max_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
							//最低电压电池子系统号 
				gRealData.minVolPack_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
							//最低电压电池单体代号
				gRealData.minVol_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(48,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x432:
		{
				//最高温度子系统号
				gRealData.maxTemperPack_index = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度值
				gRealData.min_singleTemper = calcRealValue(8,8,0.5,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//最高温度探针序号
				gRealData.maxTemper_index = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最高温度值
				gRealData.max_singleTemper = calcRealValue(24,8,0.5,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度子系统号
				gRealData.minTemperPack_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度探针单体代号
				gRealData.minTemper_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);	

		}
		break;
		
        case 0x2c:
        {
            //输入电压
            gRealData.motorData[0].motorVol = calcCanValue(0,12,msg->data,HIGHTLOW_MODE,CAN_MODE);
        }
        break;
        
		case 0x43A:
			{
				//温度差异报警
				gRealData.tempDiffAlert = calcCanValue(32,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池高温报警
				gRealData.batHighTempAlert = calcCanValue(33,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型过压报警
				gRealData.batHighVolAlert = calcCanValue(34,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型欠压报警
				gRealData.batLowVolAlert = calcCanValue(35,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC低报警
				gRealData.socLowAlert = calcCanValue(36,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池过压报警
				gRealData.singleBatHighVolAlert = calcCanValue(37,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池欠压报警
				gRealData.singleBattLowVolAlert = calcCanValue(38,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC过高报警
				gRealData.socHighAlert = calcCanValue(42,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC跳变报警
				gRealData.socHopAlert = calcCanValue(43,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能系统不匹配报警
				gRealData.batNotMatchAlert = calcCanValue(44,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体一致性差报警
				gRealData.singleBatPoorConsisAlert = calcCanValue(40,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//绝缘报警
				gRealData.insulationFailtAlert = calcCanValue(39,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//高压互锁状态报警
				gRealData.motorCtrTemAlert = calcCanValue(45,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型过充
				gRealData.batOverCharge = calcCanValue(41,1,msg->data,HIGHTLOW_MODE,CAN_MODE);

			}
			break;
	
	case 0x434:
			{
                    analysisTemper(msg->id,msg->data);
			}
			break;	
    case 0x433:
			{
				//单体电压
				analysisVol(msg->id,msg->data);
			}													
			break;

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
	uint16_t volStartIndex = (data[0]);
	
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
	uint16_t temperStartIndex = (data[0]-1);
	
	/******根据协议计算*******/
	for(i=0;i<7;++i)
	{
		if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
		{
			gRealData.single_temper[temperStartIndex++] = calcRealValue(8+i*8,8,0.5,-40,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
		}
	}
}
