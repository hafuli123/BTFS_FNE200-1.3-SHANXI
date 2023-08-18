/*
文件：analyse_Bms_XMQ4250BEVL.c
功能：航天凌河-新能源车型终端程序
日期：2021/10/13
公司：北理新源(佛山)信息科技有限公司
作者：LGC
*/

//#include "newant_bms.h"
#include "protocol_GB.h"
#include "fun_can.h"
#include "bsp_gps.h"
#include "string.h"
#include "cmsis_os2.h"
#include "bsp_rtc.h"
static const char* carType = "HTLH_XMQ4250BEVL_FV2.00";
//static const char* carType = "F4_HTLH_XMQ4250BEVL_Test3";

uint8_t alertTest = 0;							//模拟报警测试	1： 开启模拟	0：关闭模拟


static const uint8_t CAN1_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN2_USED_ANA = 1;								/*是否使用CAN1*/
static const uint8_t CAN3_USED_ANA = 0;								/*是否使用CAN1*/

static const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1波特率*/
static const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2波特率*/
static const uint32_t CAN3_BAUDRATE		=		250000;							/*CAN2波特率*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/

const uint8_t CALC_EXTREMUN  = 1;											/*是否计算极值 0:关闭 1:开启*/

void iniEvCanData(void)
{
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can3_baudrate = CAN3_BAUDRATE;
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	gSysPara.can3_used = CAN3_USED_ANA;
	CAR_TYPE = carType;
}

/*****************************************************************
*	函 数 名: calcExtremum
*	功能说明: CAN数据解析
*	返 回 值: 无
*****************************************************************/
uint32_t motorCanId[4] = {0};				//电机状态CAN ID
uint8_t byPassVolIdxMax = 0;				//最大单体电压支路个数
uint8_t byPassTemIdxMax = 0;				//最大温度探针支路个数
uint8_t motorTempAlert[5] = {0};		//电机温度报警
uint8_t motorCtrTemAlert[5] = {0};	//电机控制器温度报警
uint32_t totalMileage = 0;
char realVin[18] = {0};							//实车VIN码

uint32_t gpsStateOkCnt = 0;					//GPS天线状态
uint32_t checkStamp = 0;

void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{
	uint16_t i,n16Val = 0;
	uint8_t n8Val = 0,bValTmp = 0,motorIdx = 0,MaxFault;

	switch(msg->id)
	{
		case 0x18FE0297:
		{
			//电池单体总数
			gRealData.subSysData[0].singleVolCnt = (msg->data[5] | ((msg->data[6] & 0x0F) << 8));
			if(gRealData.subSysData[0].singleVolCnt > sizeof(gRealData.single_vol)/sizeof(gRealData.single_vol[0]))
			{
				gRealData.subSysData[0].singleVolCnt = sizeof(gRealData.single_vol) / sizeof(gRealData.single_vol[0]);
			}
			//温度探针总数
			gRealData.subSysData[0].singleTemCnt = (msg->data[7] | ((msg->data[6] & 0xF0) << 4));
			if(gRealData.subSysData[0].singleTemCnt > sizeof(gRealData.single_temper)/sizeof(gRealData.single_temper[0]))
			{
				gRealData.subSysData[0].singleTemCnt = sizeof(gRealData.single_temper) / sizeof(gRealData.single_temper[0]);
			}
		}
		break;
		case 0x18FE1F97:
		case 0x18FE2197:
		case 0x18FE2397:
		case 0x18FE2597:
		case 0x18FE2797:
		{
			for(motorIdx = 0;motorIdx < 4;motorIdx++)
			{
				if(msg->id == motorCanId[motorIdx])
				{
					//找到电机序号
					break;
				}
				if(motorCanId[motorIdx] == 0)
				{
					//新电机
					for(n8Val = 0;n8Val < motorIdx;n8Val++)
					{
						//排序
						if(msg->id < motorCanId[n8Val])
						{
							//找到插入位置
							motorIdx = n8Val;
							//插入位置后的ID需要往后移一位
							for(i = 0;i < 3 - motorIdx;i++)
							{
								motorCanId[3 - i] = motorCanId[2 - i];
							}
							break;
						}
					}
					motorCanId[motorIdx] = msg->id;
					break;
				}
			}
			if(motorIdx >= (sizeof(gRealData.motorData) / sizeof(MotorData)))
			{
				break;
			}
			if(gRealData.motorCnt < motorIdx + 1)
			{
				gRealData.motorCnt = motorIdx + 1;
			}
			//驱动电机状态
			n8Val = msg->data[0];
			if(1 == n8Val) 
				gRealData.motorData[motorIdx].motorState = MOTOR_CONSUME;
			else if(2 == n8Val) 
				gRealData.motorData[motorIdx].motorState = MOTOR_GENERATION;
			else if(3 == n8Val) 
				gRealData.motorData[motorIdx].motorState = MOTOR_OFF;
			else if(4 == n8Val) 
				gRealData.motorData[motorIdx].motorState = MOTOR_READY;
			else if(0xFE == n8Val) 
				gRealData.motorData[motorIdx].motorState = MOTOR_ABNORMAL;
			else if(0xFF)
				gRealData.motorData[motorIdx].motorState = MOTOR_NVALID;
			//电机控制器输入电压
			gRealData.motorData[0].motorVol = calcRealValue(8,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//电机控制器直流母线电流
			gRealData.motorData[motorIdx].motorCur = calcRealValue(24,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//驱动电机控制器温度
			gRealData.motorData[motorIdx].motorCtrTemp = (int16_t)calcRealValue(40,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//驱动电机温度
			gRealData.motorData[motorIdx].motorTemp = (int16_t)calcRealValue(48,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FE2097:
		case 0x18FE2297:
		case 0x18FE2497:
		case 0x18FE2697:
		case 0x18FE2897:
		{
			for(motorIdx = 0;motorIdx < 4;motorIdx++)
			{
				if(msg->id == (motorCanId[motorIdx] + 0x100))
				{
					//找到电机序号
					break;
				}
			}
			if(motorIdx >= (sizeof(gRealData.motorData) / sizeof(MotorData)))
			{
				break;
			}
			//驱动电机转速
			gRealData.motorData[motorIdx].motorSpeed = (int16_t)calcRealValue(0,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//驱动电机转矩
			gRealData.motorData[motorIdx].motorTorsion = calcRealValue(16,16,0.1,-2000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FE2A97:
		case 0x18FE2D97:
		case 0x18FE3097:
		case 0x18FE3397:
		case 0x18FE3697:
		{
			uint8_t i;
			uint16_t mIdx;
			uint8_t byPassIdx = ((msg->id - 0x18FE2A97) >> 8) / 3;//支路序号
			if(byPassIdx > byPassVolIdxMax)
			{
				byPassVolIdxMax = byPassIdx;//保存最大支路序号
			}
			//支路单体电压
			mIdx = ((msg->data[0]-1) * 3) + (gRealData.subSysData[0].singleVolCnt / (byPassVolIdxMax + 1) * byPassIdx);
			for(i = 0;i < 3 && mIdx < gRealData.subSysData[0].singleVolCnt;i++)
			{
				gRealData.single_vol[mIdx++] = calcRealValue(i * 14 + 8,14,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		}
		break;
		case 0x18FE2B97:
		case 0x18FE2E97:
		case 0x18FE3197:
		case 0x18FE3497:
		case 0x18FE3797:
		{
			uint8_t i,mIdx;
			uint8_t byPassIdx = ((msg->id - 0x18FE2B97) >> 8) / 3;//支路序号
			if(byPassIdx > byPassTemIdxMax)
			{
				byPassTemIdxMax = byPassIdx;//保存最大支路序号
			}
			//支路温度探针
//			mIdx = (msg->data[0] * 7) + (gRealData.subSysData[0].singleTemCnt / (byPassTemIdxMax + 1) * byPassIdx);
			if((msg->data[0]%2) == 1)
			{
				mIdx = (msg->data[0]-1)/2*8;
				for(i = 0;i < 6 && mIdx < gRealData.subSysData[0].singleTemCnt;i++)
				{
					gRealData.single_temper[mIdx++] = msg->data[i+1] - 40;
				}			
			}
			else
			{
				mIdx = (msg->data[0]/2-1)*8+6;
				for(i = 0;i < 2 && mIdx < gRealData.subSysData[0].singleTemCnt;i++)
				{
					gRealData.single_temper[mIdx++] = msg->data[i+1] - 40;
				}	
			}

		}
		break;
		case 0x18FE3897://500ms
		{
			//燃料电池电压
			gRealData.fuelBatVol = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//燃料电池电流
			gRealData.fuelBatCur = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//燃料电池消耗率
			gRealData.batFuelConsumption = calcRealValue(32,16,0.01,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//燃料电池温度探针总数
			gRealData.fuelBatTemCnt = calcCanValue(48,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
			if(gRealData.fuelBatTemCnt > sizeof(gRealData.fuelBatTem)/sizeof(gRealData.fuelBatTem[0]))
			{
				gRealData.fuelBatTemCnt = sizeof(gRealData.fuelBatTem) / sizeof(gRealData.fuelBatTem[0]);
			}
		}
		break;
		case 0x18FE3997://500ms
		{
			//氢系统中最高温度
			gRealData.maxHydrSysTem = calcRealValue(0,16,0.1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//氢系统中最高温度探针代号
			gRealData.maxHydrSysTemIdx = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//氢气最高浓度
			gRealData.maxHydrThickness = calcRealValue(24,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//氢气最高浓度传感器代号
			gRealData.maxHydrThicknessIdx = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FE3A97://500ms
		{
			//氢气最高压力
			gRealData.maxHydrPressure = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//氢气最高压力传感器代号
			gRealData.maxHydrPressureIdx = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//高压 DC/DC 状态
			gRealData.dc2dcState_highVol = calcRealValue(24,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FE3B97://查询/200ms
		{
			//燃料电池探针温度
			uint8_t i,mIdx = msg->data[0] - 1;
			for(i = 0;i < 7 && mIdx < gRealData.fuelBatTemCnt;i++)
			{
				gRealData.fuelBatTem[mIdx++] = msg->data[i+1] - 40;
			}
		}
		break;
		case 0x18FE4097://500ms
		{
				uint8_t bValTmp;
				//车辆状态
				n8Val = msg->data[0];
				if(1 == n8Val) 
					gRealData.carState = CARSTA_START;
				else if(2 == n8Val) 
					gRealData.carState = CARSTA_STOP;
				else if(3 == n8Val) 
					gRealData.carState = CARSTA_OTHER;
				else if(0xFE == n8Val) 
					gRealData.carState = CARSTA_ABNORMAL;
				else if(0xFF)
					gRealData.carState = CARSTA_NVALID;				
				//充电状态
				if(gRealData.total_current < 0)
				{
					if(gRealData.soc<100 && gRealData.speed==0)
						gRealData.chargeState = STOP_CHARGE;
					else if(gRealData.soc<100 && gRealData.speed>0)
						gRealData.chargeState = RUN_CHARGE;
					else
						gRealData.chargeState = CHARGE_FINISH;						
				}
				else
				{
					gRealData.chargeState = NO_CHARGE;
				}
//				n8Val = msg->data[1];
//				if(1 == n8Val) 
//					gRealData.chargeState = STOP_CHARGE;
//				else if(2 == n8Val) 
//					gRealData.chargeState = RUN_CHARGE;
//				else if(3 == n8Val) 
//					gRealData.chargeState = NO_CHARGE;
//				else if(4 == n8Val) 
//					gRealData.chargeState = CHARGE_FINISH;
//				else if(0xFE == n8Val) 
//					gRealData.chargeState = CHARGE_ABNORMAL;
//				else if(0xFF)
//					gRealData.chargeState = CHARGE_NVALID;
				//运行模式
				n8Val = msg->data[2];
				if(1 == n8Val) 
					gRealData.operationState = EV;
				else if(2 == n8Val) 
					gRealData.operationState = FV;
				else if(3 == n8Val) 
					gRealData.operationState = EV_FV;
				//档位
				n8Val = msg->data[3] & 0x0F;
				bValTmp = (gRealData.stall & 0xF0);
				gRealData.stall = (bValTmp | n8Val);
		}
		break;
		case 0x18FE4197://500ms
		{
				//总电压
				gRealData.total_volt = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysVol = gRealData.total_volt;
				//总电流
				gRealData.total_current = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysCur = gRealData.total_current;
				//SOC
				gRealData.soc = (uint8_t)calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//绝缘电阻
				gRealData.mohm = calcRealValue(40,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FE4297://500ms
		{
			//byte0:DC-DC温度
			//设置DCDC状态
			n8Val = msg->data[1];
			if(1 == n8Val) 
				gRealData.dc2dcState = DCDC_WORK;
			else if(2 == n8Val) 
				gRealData.dc2dcState = DCDC_BREAK;
			else if(0xFE == n8Val) 
				gRealData.dc2dcState = DCDC_ABNORMAL;
			else if(0xFF)
				gRealData.dc2dcState = DCDC_INVALID; 
			//加速踏板
			gRealData.acceleratorVal = (uint8_t)calcRealValue(16,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//制动踏板
			gRealData.brakingVal = calcCanValue(24,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//驱动制动状态
			SET_ACCE_BREAK();
		}
		break;
		case 0x18FE4A97://500ms
		{
				//最高电压电池子系统号
				gRealData.maxVolPack_index = 1;
				//最高电压电池单体代号
				gRealData.maxVol_index = msg->data[0];
				//电池单体电压最高值
				gRealData.max_singleVol = calcRealValue(8,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低电压电池子系统号 
				gRealData.minVolPack_index = 1;
				//最低电压电池单体代号
				gRealData.minVol_index = msg->data[3];
				//电池单体电压最低值
				gRealData.min_singleVol = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FE4B97://500ms
		{
				//最高温度子系统号
				gRealData.maxTemperPack_index = 1;
				//最高温度探针序号
				gRealData.maxTemper_index = msg->data[0];
				//最高温度值
				gRealData.max_singleTemper = (int16_t)calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//最低温度子系统号
				gRealData.minTemperPack_index = 1;
				//最低温度探针序号
				gRealData.minTemper_index = msg->data[2];
				//最低单温度
				gRealData.min_singleTemper = (int16_t)calcRealValue(24,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FE4C97:
		{
				//温度差异报警
				gRealData.tempDiffAlert = calcCanValue(0,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池高温报警
				gRealData.batHighTempAlert = calcCanValue(3,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型过压报警
				gRealData.batHighVolAlert = calcCanValue(6,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型欠压报警
				gRealData.batLowVolAlert = calcCanValue(9,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC低报警
				gRealData.socLowAlert = calcCanValue(12,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池过压报警
				gRealData.singleBatHighVolAlert = calcCanValue(15,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//单体电池欠压报警
				gRealData.singleBattLowVolAlert = calcCanValue(18,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC过高报警
				gRealData.socHighAlert = calcCanValue(21,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC跳变报警
				gRealData.socHopAlert = calcCanValue(24,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//可充电储能系统不匹配报警
				gRealData.batNotMatchAlert = calcCanValue(27,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//电池单体一致性差报警
				gRealData.singleBatPoorConsisAlert = calcCanValue(30,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//绝缘报警
				gRealData.insulationFailtAlert = calcCanValue(33,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//车载储能装置类型过充	
				gRealData.batOverCharge = calcCanValue(36,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FE5997://500ms,有故障时发
		case 0x18FE5A97:
		case 0x18FE5B97:
		case 0x18FE5C97:
		case 0x18FE5D97:
		{
			uint8_t maxMotorTempAlert = 0;
			uint8_t maxMotorCtrTemAlert = 0;
			motorIdx = (msg->id - 0x18FE5997) >> 8;
			//电机温度过高报警
			motorTempAlert[motorIdx] = calcCanValue(0,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//电机控制器温度过高报警
			motorCtrTemAlert[motorIdx] = calcCanValue(3,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
			for(motorIdx = 0;motorIdx < 5;motorIdx++)
			{
				if(motorTempAlert[motorIdx] > maxMotorTempAlert && motorTempAlert[motorIdx] <= 3)
				{
					maxMotorTempAlert = motorTempAlert[motorIdx];
				}
				if(motorCtrTemAlert[motorIdx] > maxMotorCtrTemAlert && motorCtrTemAlert[motorIdx] <= 3)
				{
					maxMotorCtrTemAlert = motorCtrTemAlert[motorIdx];
				}
			}
			gRealData.motorTempAlert = maxMotorTempAlert;
			gRealData.motorCtrTemAlert = maxMotorCtrTemAlert;
		}
		break;
		case 0x18FE6097://500ms,有故障时发
		{
				//DCDC温度报警
				gRealData.dc2dcTemAlert = calcCanValue(0,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//制动系统报警
				gRealData.brakingAlert = calcCanValue(3,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//DCDC状态报警
				gRealData.dc2dcStateAlert = calcCanValue(6,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//高压互锁状态报警
				gRealData.highPressInterlockStateAlert = calcCanValue(9,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x0CFE6CEE://50ms
		{
			//车速
			gRealData.speed = calcRealValue(48,16,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FEE017://500ms
		{
				//总里程
				gRealData.totalMileage = calcRealValue(32,32,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				totalMileage = gRealData.totalMileage;
		}
		break;
		case 0x18FF3C2B://1000ms
		{
			//车架号
			if(msg->data[1] == 0x01)
				memcpy(&realVin[0],&msg->data[2],6);
			else if(msg->data[1] == 0x07)
				memcpy(&realVin[6],&msg->data[2],6);
			else if(msg->data[1] == 0x0D)
				memcpy(&realVin[12],&msg->data[2],5);
			realVin[17] = 0;
			if(realVin[0] == 'L' && strlen(realVin) == 17 && memcmp(realVin,gSysPara.vinCode,17) != 0)
			{
				strcpy(gSysPara.vinCode,realVin);
				System_Pare_Save();
			}
		}
		break;
	}

//	if(alertTest == 1)
//	{
//		//模拟三级报警
//		if(gTerminalState.gpsState != 1)
//			gRealData.dc2dcStateAlert = 3;	
//		else
//			gRealData.dc2dcStateAlert = 0;
//	}

	
	MaxFault = 0;
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

}


/*******************************************************************************
*	函 数 名: send_To_CAN
*	功能说明: 发送数据到CAN总线		发送间隔500ms
*	形    参:  无
*	返 回 值: 无
*******************************************************************************/
//static void sendData_To_CAN(uint8_t canCh,unsigned int canid,unsigned char *canbuf){
//	CAN_msg	msg_buf;

//	msg_buf.id = canid;
//	msg_buf.len = 0x08;
//	msg_buf.format = STANDARD_FORMAT;
//	msg_buf.type = DATA_FRAME;
//	
//	memset(&msg_buf.data[0],0xFF,8);	
//	memcpy(&msg_buf.data[0],&canbuf[0],8);
//		
//	
//	if(canCh==1 && gTerminalState.can1State == 1)
//	{
//		CAN_send(canCh,&msg_buf,0x64);
//	}
//	if(canCh == 2 && gTerminalState.can2State == 1)
//	{
//		CAN_send(canCh,&msg_buf,0x64);
//	}
//	if(canCh == 3 && gTerminalState.can2State == 1)
//	{
//		CAN_send(canCh,&msg_buf,0x64);
//	}
//}

///*******************************************************************************	
//*	函 数 名: sendTimeToCAN
//*	功能说明: 发送时间到CAN总线		发送间隔200ms
//*	形    参: 无
//*	返 回 值: 无
//*******************************************************************************/
//static void sendTimeToCAN(void)
//{
//	static uint8_t sendTimeLife = 0;
//	uint8_t sendbuf[8] = {0};
//	
//	sendTimeLife++;	
//	if(sendTimeLife>255)
//		sendTimeLife = 0;

//	sendbuf[0] = sendTimeLife;
//	sendbuf[1] = (uint8_t)(g_system_dt.year >> 8);
//	sendbuf[2] = (uint8_t)(g_system_dt.year >> 0);
//	sendbuf[3] = g_system_dt.month;
//	sendbuf[4] = g_system_dt.day;
//	sendbuf[5] = g_system_dt.hour;
//	sendbuf[6] = g_system_dt.minute;
//	sendbuf[7] = g_system_dt.second;
//	sendData_To_CAN(1,0x346,sendbuf);
//}

///*******************************************************************************	
//*	函 数 名: sendLocalToCAN
//*	功能说明: 发送位置信息到CAN总线		发送间隔200ms
//*	形    参: 无
//*	返 回 值: 无
//*******************************************************************************/
//uint8_t islimitVehicle = 0;								//限速
//uint8_t isReqVIN = 0;											//请求VIN
//uint8_t isReqBatNum = 0;									//电池组编号请求信号
//static void sendLocalToCAN(void)
//{
//	uint8_t sendbuf[8] = {0};
//	uint8_t n8Val = 0;
//	uint16_t n16Val = 0;
//	
//	n8Val |= g_tGPS.PositionOk<<0;
//	if(g_tGPS.EW == 'E')
//		n8Val |= 1<<1;
//	else
//		n8Val |= 2<<1;
//	
//	if(g_tGPS.NS == 'N')
//		n8Val |= 1<<3;
//	else
//		n8Val |= 2<<3;
//	//位置状态信息
//	sendbuf[0] = n8Val;
//	
//	//经度
//	n16Val = gRealData.longd *100;
//	sendbuf[1] = (uint8_t)(n16Val >> 8);
//	sendbuf[2] = (uint8_t)(n16Val >> 0);

//	//纬度
//	n16Val = gRealData.latd *100;
//	sendbuf[3] = (uint8_t)(n16Val >> 8);
//	sendbuf[4] = (uint8_t)(n16Val >> 0);
//	
//	//GPS速度
//	sendbuf[5] = g_tGPS.SpeedKnots;
//	
//	//限速，请求VIN，请求电池组编号
//	n8Val |= islimitVehicle<<0;
//	n8Val |= isReqVIN << 2;
//	n8Val |= isReqBatNum<<3;
//	sendbuf[6] = n8Val;
//	
//	sendData_To_CAN(1,0x347,sendbuf);
//}

///*******************************************************************************	
//*	函 数 名: getVIN
//*	功能说明: 从CAN总线上获取VIN		发送间隔1000ms
//*	形    参: 无
//*	返 回 值: 无
//*******************************************************************************/
//static void getVIN(uint8_t ch,CAN_msg *msg)
//{	
//	switch(msg->id)
//	{
//		case 0x235:
//			memcpy(&gSysPara.vinCode[0],msg->data,8);
//			break;
//		case 0x236:
//			memcpy(&gSysPara.vinCode[8],msg->data,8);
//			break;
//		case 0x237:
//			gSysPara.vinCode[16] = msg->data[0];
//			break;	
//	}
//}

