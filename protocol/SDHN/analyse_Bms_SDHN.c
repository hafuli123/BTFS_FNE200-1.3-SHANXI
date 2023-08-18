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
#include "math.h"

const char* carType = "SDHN_FV2.04";
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

int datato(uint8_t data)
{
    int ret = 0 ;
    int i=1;
    while(i<8)
    {  
        ret += (data/(int)(pow(16,i)))*pow(10,i);
        i++;
    }
    ret = ret + data % 16;
    return ret;
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
	uint8_t bValTmp = 0;
    int mile1 = 0;
    int mile2 =0;
    int mile3 =0;
//	gRealData.carState = CARSTA_START;
	switch(msg->id)
	{
		case 0x1028FF00:
			{
				//高压DC/DC状态
				n8Val = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
					gRealData.dc2dcState_highVol = DCDC_WORK;
				else
					gRealData.dc2dcState_highVol = DCDC_BREAK;
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

		case 0x18FF43A8:
			{
				//车辆状态
				n8Val = calcCanValue(0,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val)
					gRealData.carState = CARSTA_START;							//1		启动
				else if(2 == n8Val)
					gRealData.carState = CARSTA_STOP;							//2		熄火	
				//运行模式
				n8Val = calcCanValue(4,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val)
				{
					gRealData.operationState = EV;								//0	纯电
				}
				else if(1 == n8Val)
				{
					gRealData.operationState = EV_FV;							//1	混动
				}
				else if(2 == n8Val)
					gRealData.operationState = FV;								//2	燃油	
				//制动踏板状态			
				n8Val = calcRealValue(8,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,1);//TODO...如果制动踏板为行程值，则第二个参数为-1。如果为状态值，则第二个参数为1						
				//驱动电机状态
				n8Val = calcCanValue(12,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;							//0 关闭
				else if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;						//1	耗电
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;					//2	发电
				else if(4 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;						//3	准备
				//燃料消耗率
				gRealData.batFuelConsumption = calcRealValue(24,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//DCDC状态	
				n8Val = calcCanValue(22,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val)
					gRealData.dc2dcState = DCDC_WORK;																	//2	工作；
				else
					gRealData.dc2dcState = DCDC_BREAK;																//else		不工作	                
			}
			break;
			
		case 0x18FF47A8:
		{
				//总电流
				gRealData.total_current = calcRealValue(24,16,0.025,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
					//SOC
				gRealData.soc = calcRealValue(8,16,0.0025,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//总电压
				gRealData.total_volt = calcRealValue(0,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
							//电机控制器直流母线电流	
				gRealData.motorData[0].motorCur = calcRealValue(24,16,0.025,-800,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
			
		case 0x18FEDA17:
		{
				//累计里程
				gRealData.totalMileage  = calcRealValue(0,32,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//                mile1 = datato(msg->data[0]);
//                mile2 = datato(msg->data[1]);
//                mile3 = datato(msg->data[2]);

//                gRealData.totalMileage = mile1 * 10000;
//                gRealData.totalMileage += mile2 * 100;
//                gRealData.totalMileage += mile3;
            
		}
			
			case 0x1883EFF3:
			{
				//充电状态
				n8Val = calcCanValue(41,2,msg->data,HIGHTLOW_MODE,CAN_MODE);							
				if(0 == n8Val) 
					gRealData.chargeState = NO_CHARGE;						//0 未充电
				else if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;					//1 停车充电
				else if(2 == n8Val) 
					gRealData.chargeState = RUN_CHARGE;						//2	行车充电
				else if(3 == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;  				//3 充电完成		
			}
			break;
			
			case 0x18FF44A8:
			{
				//车速
				gRealData.speed = calcRealValue(0,16,0.0003906,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//加速踏板行程值		
				gRealData.acceleratorVal = calcRealValue(24,8,0.005,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动制动状态
				SET_ACCE_BREAK();				
			}
			break;
			
		case 0x1881EFF3:
			{
				//温度差异报警								
				gRealData.tempDiffAlert = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池高温报警						
				gRealData.batHighTempAlert = calcCanValue(4,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型过压报警					
				gRealData.batHighVolAlert = calcCanValue(6,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//车载储能装置类型欠压报警					
				gRealData.batLowVolAlert = calcCanValue(8,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//SOC低报警
				gRealData.socLowAlert = calcCanValue(10,1,msg->data,HIGHTLOW_MODE,CAN_MODE); 				
				//单体电池过压报警
				gRealData.singleBatHighVolAlert = calcCanValue(12,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池欠压报警					 
				gRealData.singleBattLowVolAlert = calcCanValue(14,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC过高报警						
				gRealData.socHighAlert = calcCanValue(22,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC跳变报警
				n8Val = calcCanValue(24,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHopAlert = (n8Val == 0? 0:1);				
				//可充电储能系统不匹配报警
				n8Val = calcCanValue(25,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batNotMatchAlert = (n8Val == 0? 0:1);			
				//电池单体一致性差报警				 
				gRealData.singleBatPoorConsisAlert = calcCanValue(18,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//绝缘报警							 
				gRealData.insulationFailtAlert= calcCanValue(16,2,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//高压互锁状态报警
				n8Val = calcCanValue(26,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val)
					gRealData.highPressInterlockStateAlert = 0;		//0	无故障
				else if(1 == n8Val)
					gRealData.highPressInterlockStateAlert = 3;		//1	故障等级三						
				//车载储能装置类型过充报警
				n8Val = calcCanValue(20,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batOverCharge = (n8Val == 0? 0:1);	
			}
			break;
			
		
		case 0x1882EFF3:
		{
			//可充电故障总数
			gRealData.batFaultCnt = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x18FF35F1:
		{
			//驱动电机故障总数
			gRealData.motorFaultCnt = calcCanValue(8,6,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x1884EFF3:
			{				
				//可充电储能装置电压
				gRealData.subSysData[0].subSysVol = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能装置电流
				gRealData.subSysData[0].subSysCur = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
			
		case 0x1885EFF3:
			{
				//绝缘电阻
				gRealData.mohm = calcRealValue(32,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;

		case 0x1888EFF3:
			{
				//最高电压电池子系统号
				gRealData.maxVolPack_index = 1;
				//最低电压电池子系统号
				gRealData.minVolPack_index = 1;
				//最高温度子系统号
				gRealData.maxTemperPack_index = 1;
				//最低温度子系统号
				gRealData.minTemperPack_index = 1;
				//最高电压单体代号
				gRealData.maxVol_index = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压单体代号
				gRealData.minVol_index = calcCanValue(32,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				// 可充电储能子系统个数			
				gRealData.subBatSysCnt = 1;
				// 可充电储能子系统号			直接发1
				gRealData.subSysData[0].subSysIdx = 1;	
			}
			break;
			
		case 0x18E3EFF3:
		{
				//单体电池总数
				gRealData.subSysData[0].singleVolCnt = 324;

				gRealData.subSysData[0].singleTemCnt = 	calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
			
		case 0x1887EFF3:
		{
			//单体电压最高值
			gRealData.max_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
			//单体电压最低值
			gRealData.min_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
		}
		break;
		
		case 0x1886EFF3:
		{
			//最高温度探针单体代号
			gRealData.maxTemper_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//最高温度值
			gRealData.max_singleTemper = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//最低温度探针序号
			gRealData.minTemper_index = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//最低单温度
			gRealData.min_singleTemper = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x18FF48A8:
			{					
				uint8_t bValTmp; 		 
				//档位
				n8Val = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(2 == n8Val  )		//或上0，运算时高位用0补齐
					gRealData.stall = (bValTmp | NEUTRAL); 													//0	N
				else if(1 == n8Val  )	
					gRealData.stall = (bValTmp | DIRVE);													//1	D
				else if(3 == n8Val  )
					gRealData.stall = (bValTmp | REVERSE);													//3	R
				else if(5 == n8Val  )
					gRealData.stall = (bValTmp | PART);														//3	P

			}
			break;
		

		case 0x18FF99A8:
			{
				//最高报警等级
				gRealData.alarmLevel = calcCanValue(0,4,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//DC-DC温度报警
				n8Val = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
 				if(4 == n8Val)
					gRealData.dc2dcTemAlert = 1;																	//4: 1级报警；其他：正常
				else
					gRealData.dc2dcTemAlert = 0;				
				//DC-DC状态报警
				n8Val = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				if( 21 == n8Val){
					gRealData.dc2dcStateAlert = 1;					
				}else{
					gRealData.dc2dcStateAlert = 0;
				}

			}
			break;
			
		case 0x18f0010b:
		{
				//制动系统报警
				n8Val = calcCanValue(44,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val)
					gRealData.brakingAlert = 1;							//1  报警等级一
				else if(0 == n8Val)
					gRealData.brakingAlert = 0;							//其他	无报警			
		}
		break;
			
		case 0x18FF46A8:
			{
				//驱动电机控制器温度			
				gRealData.motorData[0].motorCtrTemp = calcRealValue(40,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//驱动电机温度						
				gRealData.motorData[0].motorTemp = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电机控制器输入电压			
				gRealData.motorData[0].motorVol = calcRealValue(56,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转速							
				gRealData.motorData[0].motorSpeed = calcRealValue(0,16,1,-6000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//驱动电机转矩						
				gRealData.motorData[0].motorTorsion = calcRealValue(0,16,0.25,-8000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18ff81f0:
			{
				//驱动电机控制器温度报警		
				n16Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(3 == n16Val)
					gRealData.motorCtrTemAlert = 2;					//3 ：2级
				else
					gRealData.motorCtrTemAlert = 0;					//其他	无故障
				
				//驱动电机温度报警				
				n16Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(4 == n16Val)
					gRealData.motorTempAlert = 2;						//4 : 2级
				else
					gRealData.motorTempAlert = 0;						//其他	无故障
						
			}
			break;
			
		case 0x00FF020C:
			{
								//燃料电池电压					数值小于0时，置0，其他转发
			n16Val = calcRealValue(0,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val < 0)
					gRealData.fuelBatVol = 0;
				else
					gRealData.fuelBatVol = n16Val;
				//燃料电池电流					数值小于0时，置0，其他转发
				n16Val = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val < 0)
					gRealData.fuelBatCur = 0;
				else
					gRealData.fuelBatCur = n16Val;
			}
			break;
	
		case 0x18E1EFF3:	
			{				
				//可充电储能系统编码长度
				gRealData.rechargeSysCodeLen = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能系统编码
//				send_RechargeSysCode(msg->id,msg->data);
			}
			break;
			
		case 0x18C1EFF3:
			{
				//解析单体电压
				analysisVol(msg->id,msg->data);	
			}
			break;		
		case 0x18C2EFF3:											
			{
				//解析单体温度
				analysisTemper(msg->id,msg->data);
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
	uint16_t volStartIndex = (calcCanValue(0,16,data,HIGHTLOW_MODE,CAN_MODE) - 1);
	
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
	for(i=0;i<6;++i)
	{
		if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
		{
			gRealData.single_temper[temperStartIndex++] = calcRealValue(16+i*8,8,1,-40,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
		}
	}
}