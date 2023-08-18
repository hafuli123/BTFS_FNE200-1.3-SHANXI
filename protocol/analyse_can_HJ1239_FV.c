/*
文件：analyse_can_HJ1239_FV.c
功能：HJ1239 燃油车CAN处理程序
日期：2022/5/20
公司：北理新源(佛山)信息科技有限公司
作者：LGC
*/

#include "fun_can.h"
#include "Fun_Net.h"
#include "bsp_rtc.h"
#include "protocol_GB.h"
#include "protocol_GB17691.h"

#include "cmsis_os2.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stdint.h"

#include "bsp_storage.h"
#include "algo_verify.h"


static const char* carType = "F4_FNE200V103_FV_V1.0";

static const uint8_t CAN1_USED_ANA = 1;												/*是否使用CAN1*/
static const uint8_t CAN2_USED_ANA = 0;												/*是否使用CAN1*/
static const uint8_t CAN3_USED_ANA = 0;												/*是否使用CAN1*/

static const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1波特率*/
static const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2波特率*/
static const uint32_t CAN3_BAUDRATE		=		250000;							/*CAN2波特率*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;							//数据高地位
const static CANANA_MODEL CAN_MODE = INTEL;										//CAN接收模式，分为Intel格式和Motorola格式
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;						//CAN数据偏移量，系数计算模式

static void Udstask(void);																		//OBD请求发送
static void udsAnalyse(CAN_msg *msg);													//OBD数据接收
static uint8_t emissionLevel = 0;							//判断国五和国六 1为国六
//static uint8_t catype = 5;																		//判断国五和国六 6为国六
static uint8_t sendStaSecond;																	//发送金龙CAN数据

static uint8_t scrInReady = 0;																//SCR入口ready,数据质量优化
static uint8_t scrOutReady = 0;																//SCR出口ready,数据质量优化


static uint32_t sendCanTimeStamp = 0;													//发送CAN数据时间戳
static uint32_t sendAuxStamp = 0;															//发送辅助数据时间戳

static uint32_t engTotalHoursOfOperationStamp = 0;						//发动机总运行时长
static uint32_t actAreaInjectValStamp = 0;										//实际尿素喷射量
static uint32_t totalUreaUsedStamp1 = 0;											//总尿素消耗率,ID1
static uint32_t totalUreaUsedStamp2 = 0;											//总尿素消耗率,ID2
static uint32_t engReferenceTorqueStamp = 0;									//发动机参考扭矩

static uint32_t sendDateTimeStamp;														//发送时间日期
static uint32_t sendOxygenStamp;															//27145发送氧输出发送时间日期
static uint32_t sendUdsId = 0;																//发送UDS命令ID，分标准帧和扩展帧
static uint8_t afterOtherCnt = 0;															//其他占用命令，3秒后终端才能发送

static uint32_t getObdDiagProtIdx = 2;												//测探OBD诊断协议顺序 ISO27145 ISO15765 SAE J1939

static uint32_t speedTime = 0;																//车速停止超时判断
static uint32_t engSpeedTime = 0;															//发动机转速停止超时判断
static uint8_t getOxygenFlag = 0;															//
uint8_t gUploadCan = 0;																				/*上传CAN日志 0:不上传 1:接收到上传命令 2:正在采集 3:开始上传*/

void iniFvCanData(void)
{
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can3_baudrate = CAN3_BAUDRATE;
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	gSysPara.can3_used = CAN3_USED_ANA;
	CAR_TYPE = carType;
	gSysPara.carType = 1;			//初始化默认国六
}

/*
*********************************************************************************************************
*	函 数 名: calcExtremum
*	功能说明: CAN数据解析，高位在前
*	形    参:  CAN_msg  	CAN结构体
*	返 回 值: 无
*********************************************************************************************************
源地址标识	装置
Hex	Dec	
0x00	0	发动机控制单元
0x03	3	传动系统控制单元(变速箱)
0x0B	11	电控刹车系统 (EBS)
0x0F	15	发动机缓速器
0x10	16	驱动系统缓速器
0x17	23	仪表
0x21	33	车身
0x24	36	PTO
0x27	39	车辆智能中心
0x29	41	排气缓速器
0x2B	43	车载诊断系统
0x3D	61	后处理控制器
0xEE	238	转速表
0xF9	249	故障诊断－售后服务工具
*/
void unpackFvCAN(uint8_t ch,CAN_msg *msg)
{
	float fval;
	uint32_t pgn = (msg->id & 0x00FFFF00) >> 8;
	uint8_t sourseAddr = msg->id & 0x000000FF;
	if((pgn & 0xF000) != 0xF000)
	{
		pgn &= 0xFF00;
	}
	Udstask();
	udsAnalyse(msg);
	//重型车混合动力车数据
	switch(msg->id)
	{
		case 0x18FFFB97:
			{
				gRealData.motorData[0].motorLoad = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FE2097:
			{
				gRealData.motorData[0].motorSpeed = calcRealValue(0,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FE4197:
			{
				gRealData.total_volt = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.total_current = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.soc = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
	}
	switch(pgn)
	{
		case 0xFE6C://0x0CFE6CEE 0xEE:测速计或里程表
			{
				//车速 优先采用里程表
				if(gTerminalState.speed <= 1 || gTerminalState.speed == msg->id || sourseAddr == 0xEE)
				{
					speedTime = osKernelGetTickCount();
					fval = calcRealValue(48,16,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(fval >= 0 && fval < 250.0f)
					{
						gRealData.speed = fval;
						if(fval > 0)
							gTerminalState.speed = msg->id;
					}
				}
				if(gTerminalState.speed == 0)
					gTerminalState.speed = 1;
			}
			break;
		case 0xFEF1:
			{
				//车速 襄阳检测要求只保留1位小数
				if(gTerminalState.speed <= 1 || gTerminalState.speed == msg->id)
				{
					speedTime = osKernelGetTickCount();
					fval = calcRealValue(8,16,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(fval >= 0 && fval < 250.0f)
					{
						gRealData.speed = fval;
						if(fval > 0)
							gTerminalState.speed = msg->id;
					}
				}
				if(gTerminalState.speed == 0)
					gTerminalState.speed = 1;
			}
			break;
		case 0xFEF5://18FEF500
			{
				//大气压力
				gRealData.barometric = calcRealValue(0,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gTerminalState.barometric = msg->id;
			}
			break;	
		case 0xF003:
			{
				//加速踏板行程值
				gRealData.acceleratorVal = calcRealValue(8,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xFEE9:
			{
				//累计油耗
				gRealData.EngTotalFuelUsed = calcRealValue(32,32,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xF023:
			{
				actAreaInjectValStamp = osKernelGetTickCount();
				//实际尿素喷射量//待确定
				gRealData.actAreaInjectVal = calcRealValue(0,16,0.3,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xF024:
			{
				//累计尿素消耗//待确定
				gRealData.totalUreaUsed = calcRealValue(24,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				totalUreaUsedStamp1 = 0xFFFFFFFF;
			}
			break;
		case 0xFCBD://待确定
			{
				//累计尿素消耗//待确定 精度0.5L 尿素密度取1.092kg/L
				gRealData.totalUreaUsed = calcRealValue(0,32,540,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				totalUreaUsedStamp2 = 0xFFFFFFFF;
			}
			break;
		case 0xFEE5:
			{
				engTotalHoursOfOperationStamp = 0xFFFFFFFF;
				//发动机总运行时长
				gRealData.engTotalHoursOfOperation = calcRealValue(0,32,0.05,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
//		case 0x0000:
//			{
//				if(sourseAddr == 0x0B || sourseAddr == 0x00 || sourseAddr == 0x03)
//				{
//					//发动机扭矩模式00:Override disabled 01:Speed control 10:Torque control 11:Speed/torque limit control
//					gRealData.engineTorqueMode = calcRealValue(0,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//				}
//			}
//			break;				
		case 0xF004://0CF00400
			{
				if(sourseAddr == 0)
				{
					engSpeedTime = osKernelGetTickCount();
					//发动机净输出扭矩
					gRealData.engineTorque = calcRealValue(16,8,1,-125,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					//发动机转速
					gRealData.engineSpeed = calcRealValue(24,16,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					gTerminalState.engineTorque = msg->id;
					gTerminalState.engineSpeed = msg->id;
					gRealData.engineTorqueMode = calcRealValue(0,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				}
			}
			break;
		case 0xFEDF://18FEDF00
			{
				//摩擦扭矩
				if(sourseAddr == 0x00)
				{
					gRealData.frictionTorque = calcRealValue(0,8,1,-125,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					gTerminalState.frictionTorque = msg->id;
				}
			}
			break;
		case 0xFEF2://18FEF200
			{
				//发动机燃料流量
				gRealData.engineFuelFlow = calcRealValue(0,16,0.05,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gTerminalState.engineFuelFlow = msg->id;
				//瞬时油耗
				gRealData.engFuelRate = calcRealValue(16,16,0.001953125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xF00E://0x18F00E51 襄阳检测小于等于0值需要置无效 18F00E00
			{
				//SCR 上游 NOx 传感器输出值
				gRealData.scrUpperNOxSensor = calcRealValue(0,16,0.05,-200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.engineSpeed <= 0 || gRealData.engineSpeed >= 0xFFFE)
				{
					//发动机熄火，无无废气排放
					gRealData.scrUpperNOxSensor = 0;
				}
				else if(gRealData.scrUpperNOxSensor > 0 && gRealData.scrUpperNOxSensor < 3012.0f && scrInReady == 0 && gRealData.engineSpeed >= 10 && gRealData.engineSpeed < 0xFFFE)//SCR已经工作
				{
					scrInReady = 1;
				}
				if(scrInReady == 0 && gRealData.engineSpeed >= 10 && gRealData.engineSpeed < 0xFFFE)//SCR未工作，无效
				{
					gRealData.scrUpperNOxSensor = 0xFFFF;
				}
				else if(scrInReady == 1 && gRealData.scrUpperNOxSensor < 0 && gRealData.scrUpperNOxSensor >= -20)//正常工作过滤
				{
					gRealData.scrUpperNOxSensor = -gRealData.scrUpperNOxSensor;
				}
			}
			break;
		case 0xF00F://0x18F00F52 襄阳检测小于等于0值需要置无效 18F00F00
			{
				//SCR 下游 NOx 传感器输出值
				gRealData.scrLowerNOxSensor = calcRealValue(0,16,0.05,-200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.engineSpeed <= 0 || gRealData.engineSpeed >= 0xFFFE)
				{
					//发动机熄火，无无废气排放
					gRealData.scrLowerNOxSensor = 0;
				}
				else if(gRealData.scrLowerNOxSensor > 0 && gRealData.scrLowerNOxSensor < 3012.0f && scrOutReady == 0 && gRealData.engineSpeed >= 10 && gRealData.engineSpeed < 0xFFFE)//SCR已经工作
				{
					scrOutReady = 1;
				}
				if(scrOutReady == 0 && gRealData.engineSpeed >= 10 && gRealData.engineSpeed < 0xFFFE)//SCR未工作，无效
				{
					gRealData.scrLowerNOxSensor = 0xFFFF;
				}
				else if(scrOutReady == 1 && gRealData.scrLowerNOxSensor < 0 && gRealData.scrLowerNOxSensor >= -20)//正常工作过滤
				{
					gRealData.scrLowerNOxSensor = -gRealData.scrLowerNOxSensor;
				}
			}
			break;			
		case 0xFD3E://0x14FD3E3D 18FD3E00
			{
				if(gRealData.engineSpeed >= 10 && gRealData.engineSpeed < 0xFFFE)
				{
					//SCR入口温度
					gRealData.scrInletTemp = calcRealValue(0,16,0.03125,-273,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					//SCR出口温度
					gRealData.scrOutletTemp = calcRealValue(24,16,0.03125,-273,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				}
			}
			break;
		case 0xFE56://0x18FE56A3 0x18FE563D 18FE5600
			{
				//反应剂余量
				if(gTerminalState.reagentSurplus <= 1 || gTerminalState.reagentSurplus == msg->id)
				{
					gRealData.reagentSurplus = calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(gRealData.reagentSurplus > 0 && gRealData.reagentSurplus <= 100)
					{
						gTerminalState.reagentSurplus = msg->id;
					}
				}
				if(gTerminalState.reagentSurplus == 0)
					gTerminalState.reagentSurplus = 1;
				//尿素箱温度
				gRealData.ureaTankTemp = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xFF4C://18FF4C00 得意TD
			{
				//反应剂余量
				if(gTerminalState.reagentSurplus <= 1 || gTerminalState.reagentSurplus == msg->id)
				{
					gRealData.reagentSurplus = calcRealValue(16,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(gRealData.reagentSurplus > 0 && gRealData.reagentSurplus <= 100)
					{
						gTerminalState.reagentSurplus = msg->id;
					}
				}
				if(gTerminalState.reagentSurplus == 0)
					gTerminalState.reagentSurplus = 1;
			}
			break;
		case 0xFDB2://18FDB200
			{
				//dpf压差
				gRealData.dpfPressDiff = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//dpf排气温度
				gRealData.dpfExhaustTemp = calcRealValue(0,16,0.03125,-273,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xFEEE://18FEEE00
			{
				//发动机冷却液温度
				gRealData.engineCoolantTemp = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gTerminalState.engineCoolantTemp = msg->id;
			}
			break;
		case 0xFEFC://油箱液位 J1939 0x18FEFC17 18FEFC21
			{
				//判断是否
				if(gTerminalState.tankLevel <= 1 || gTerminalState.tankLevel == msg->id || 
					(sourseAddr == 17 && gTerminalState.tankLevel != 0x18A70017 && gTerminalState.tankLevel != 0x18FA0317))
				{
					gRealData.tankLevel = calcRealValue(8,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(gRealData.tankLevel > 0 && gRealData.tankLevel <= 100)
					{
						gTerminalState.tankLevel = msg->id;
					}
				}
			}
			break;
		case 0xFA03://油箱液位 金龙
			{
				if(gTerminalState.tankLevel <= 1 || 0x18FA0317 == msg->id)
				{
					gRealData.tankLevel = calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(gRealData.tankLevel > 0 && gRealData.tankLevel <= 100)
					{
							gTerminalState.tankLevel = msg->id;
					}
				}
			}
			break;
		case 0xA700://油箱液位 金旅
			{
				if(gTerminalState.tankLevel <= 1 || 0x18A70017 == msg->id)
				{
					gRealData.tankLevel = calcRealValue(56,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(gRealData.tankLevel > 0 && gRealData.tankLevel <= 100)
					{
							gTerminalState.tankLevel = msg->id;
					}
				}
			}
			break;
		case 0xFEC1://0x18FEC1EE
			{
				float totalMileage;
				//高精度累计里程,优先使用0x18fee017或0x18fee0ee
				if(msg->id == 0x18FEC1EE && gTerminalState.totalMileage != 0x18fee017 && gTerminalState.totalMileage != 0x18fee0ee)
				{
					gTerminalState.totalMileage = msg->id;
				}
				if(gTerminalState.totalMileage <= 1 || gTerminalState.totalMileage == msg->id)
				{
					totalMileage = calcRealValue(0,32,0.005,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(totalMileage > 0 && totalMileage < 9999999.9f)
					{
						gRealData.totalMileage = totalMileage;
					}
					gTerminalState.totalMileage = msg->id;
				}
			}
			break;
		case 0xFEE0:
			{
				float totalMileage;
				if(msg->id == 0x18fee0ee)//优先使用
				{
					gTerminalState.totalMileage = msg->id;
				}
				if(msg->id == 0x18fee017 && gTerminalState.totalMileage != 0x18fee0ee)//第二使用
				{
					gTerminalState.totalMileage = msg->id;
				}
				//累计里程
				if(gTerminalState.totalMileage <= 1 || gTerminalState.totalMileage == msg->id)
				{
					totalMileage = calcRealValue(32,32,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(totalMileage > 0 && totalMileage < 9999999.9f)
					{
						gRealData.totalMileage = totalMileage;
						gTerminalState.totalMileage = msg->id;
					}
				}
			}
			break;
		case 0xF00A://0F00A00
			{
				//进气量
				gRealData.intakeFlowrate = calcRealValue(16,16,0.05,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gTerminalState.intakeFlowrate = msg->id;
			}
			break;
			//玉柴
		case 0xFFCC://0CFFCC00
			{
				//TWC 上游氧传感器输出值 
				gRealData.twcUpperOxySensor = calcRealValue(0,16,1/16384,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
			}
			break;
			//
		case 0xFD46://0CFD4600
			{
				//TWC 出口温度
				gRealData.twcTemp = calcRealValue(16,16,0.03125,-273,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
			}
			break;
			//潍柴
		case 0xFFAE://1CFFAE00
			{
				//TWC 上游氧传感器输出值 空燃比
				gRealData.twcUpperOxySensor = calcRealValue(0,16,0.0000305,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
			}
			break;
		case 0xFFC1://0x1CFFC100
			{
				//TWC催化器下游氧传感器输出值(电压)
				gRealData.twcUpperOxySensor = calcRealValue(8,8,0.01,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
				getOxygenFlag = 1;
			}
			break;
		case 0xFD22://0x18FD2200
			{
				//TWC 出口温度
				gRealData.twcTemp = calcRealValue(0,16,0.03125,-273,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
			}
			break;
		default:
			{
				
			}
			break;
	}
	if(osKernelGetTickCount() - speedTime >= 5000)
	{
		gRealData.speed = 0;
	}
	if(osKernelGetTickCount() - engSpeedTime >= 5000)
	{
		gRealData.engineSpeed = 0;
	}
	if(osKernelGetTickCount() - sendCanTimeStamp >= 10 && fun_can_Get_State(BSP_CAN1) > 0)
	{
		uint8_t isSend = 0;
		CAN_msg msg_buf = {0x18FA122B,{ 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,EXTENDED_TYPE,DATA_FRAME};
		sendCanTimeStamp = osKernelGetTickCount();
		if(osKernelGetTickCount() - sendStaSecond >= 5000)
		{
			uint32_t devId = 0;
			uint8_t vinSta = 0;
			sendStaSecond = osKernelGetTickCount();	
			//CAN心跳
			sscanf(&gFrimPara.terminalId[6],"%u",&devId);
			devId = 0xEF5F4C00 + devId;
			msg_buf.data[0] = 0x22;
			msg_buf.data[1] = devId >> 24;
			msg_buf.data[2] = devId >> 16;
			msg_buf.data[3] = devId >> 8;
			msg_buf.data[4] = devId >> 0;
			if(strlen(gRealData.vin) == 17 && CheckVin(gRealData.vin) == 1)
			{
				vinSta = 1;
			}
			msg_buf.data[5] = vinSta << 7;
			msg_buf.data[5] |= fun_can_Get_State(BSP_CAN1) << 6;
			msg_buf.data[5] |= gb17691GetSta(0) << 5;
			msg_buf.data[5] |= (!gRealData.locationState) << 4;
			msg_buf.data[5] |= (Fun_Gprs_GetSta() >= FUN_GPRS_GET_SIM) << 3;
			msg_buf.data[5] |= (1 & 0x07);
			
			msg_buf.data[6] = (gTerminalState.reagentSurplus > 0) << 7;
			msg_buf.data[6] |= (gTerminalState.intakeFlowrate > 0) << 6;
			msg_buf.data[6] |= (gTerminalState.engineFuelFlow > 0) << 5;
			msg_buf.data[6] |= (gTerminalState.engineSpeed > 0) << 4;
			msg_buf.data[6] |= (gTerminalState.frictionTorque > 0)<< 3;
			msg_buf.data[6] |= (gTerminalState.engineTorque > 0) << 2;
			msg_buf.data[6] |= (gTerminalState.barometric > 0) << 1;
			msg_buf.data[6] |= (gTerminalState.speed > 0) << 0;
			msg_buf.data[7] = (gTerminalState.engineCoolantTemp > 0) << 0;
			isSend = 1;
		}
		else if(osKernelGetTickCount() - sendDateTimeStamp >= 1000)
		{
			/*//CAN日志
			if(gUploadCan == 1)
			{
				gUploadCanSize = 0;
				canLogSecCnt = 0;
				canlogBufLen = 0;
				memset(printfBuf,0,sizeof(printfBuf));
				canlogAddr = ADDR_FLASH_SECTOR_4;
				gUploadCan = 2;
			}
			canLogSecCnt++;
			*/
			//湖北三环要求
			sendDateTimeStamp = osKernelGetTickCount();
			msg_buf.id = 0x18fee6fa;
			msg_buf.data[0] = g_system_dt.second * 4;
			msg_buf.data[1] = g_system_dt.minute;
			msg_buf.data[2] = g_system_dt.hour;
			msg_buf.data[3] = g_system_dt.month;
			msg_buf.data[4] = g_system_dt.day * 4;
			msg_buf.data[5] = g_system_dt.year - 1985;
			msg_buf.data[6] = 125;
			msg_buf.data[7] = 125 + 8;
			isSend = 1;
		}
		//SUP_DATA_BIT 6  //补充数据流格式，占二位 0-不上报 1-正常开启 2：金旅版补充数据流 3备用
		else if(((gSysPara.linkSwitch >> 6) & 0x03) > 0 && gRealData.engineSpeed >= 100 && gRealData.engineSpeed < 0xFFFE && osKernelGetTickCount() - sendAuxStamp >= 1000)
		{
			sendAuxStamp = osKernelGetTickCount();
			msg_buf.id = 0x18eafff9;
			if(osKernelGetTickCount() - actAreaInjectValStamp >= 6000)
			{
				actAreaInjectValStamp = osKernelGetTickCount();
				msg_buf.data[0] = (uint8_t)(0x00F023 >> 0);
				msg_buf.data[1] = (uint8_t)(0x00F023 >> 8);
				msg_buf.data[2] = (uint8_t)(0x00F023 >> 16);
				isSend = 1;
			}
			else if(osKernelGetTickCount() - totalUreaUsedStamp1 >= 6000 && totalUreaUsedStamp2 != 0xFFFFFFFF)
			{
				//取其中一个
				totalUreaUsedStamp1 = osKernelGetTickCount();
				msg_buf.data[0] = (uint8_t)(0x00F024 >> 0);
				msg_buf.data[1] = (uint8_t)(0x00F024 >> 8);
				msg_buf.data[2] = (uint8_t)(0x00F024 >> 16);
				isSend = 1;
			}
			else if(osKernelGetTickCount() - totalUreaUsedStamp2 >= 6000 && totalUreaUsedStamp1 != 0xFFFFFFFF)
			{
				//取其中一个
				totalUreaUsedStamp2 = osKernelGetTickCount();
				msg_buf.data[0] = (uint8_t)(0x00F024 >> 0);
				msg_buf.data[1] = (uint8_t)(0x00F024 >> 8);
				msg_buf.data[2] = (uint8_t)(0x00F024 >> 16);
				isSend = 1;
			}
			else if(osKernelGetTickCount() - engReferenceTorqueStamp >= 6000)
			{
				engReferenceTorqueStamp = osKernelGetTickCount();
				msg_buf.data[0] = (uint8_t)(0x00FEE9 >> 0);
				msg_buf.data[1] = (uint8_t)(0x00FEE9 >> 8);
				msg_buf.data[2] = (uint8_t)(0x00FEE9 >> 16);
				isSend = 1;
			}
			else if(osKernelGetTickCount() - engTotalHoursOfOperationStamp >= 6000)
			{
				engTotalHoursOfOperationStamp = osKernelGetTickCount();
				msg_buf.data[0] = (uint8_t)(0x00FEE5 >> 0);
				msg_buf.data[1] = (uint8_t)(0x00FEE5 >> 8);
				msg_buf.data[2] = (uint8_t)(0x00FEE5 >> 16);
				isSend = 1;
			}
		}
		//ISO 27145 请求
		else if(osKernelGetTickCount() - sendOxygenStamp >= 2000 && gRealData.obdDiagProt == 1 && afterOtherCnt == 0 && gTerminalState.obdState == 1 && gRealData.technology >= 1 && getOxygenFlag == 0)
		{
			sendOxygenStamp = osKernelGetTickCount();
			msg_buf.id = sendUdsId;
			if(sendUdsId <= 0x7FF)
			{
				msg_buf.format = STANDARD_TYPE;
			}
			memset(msg_buf.data,0,8);
			msg_buf.data[0] = 0x03;
			msg_buf.data[1] = 0x22;
			msg_buf.data[2] = 0xF4;
			msg_buf.data[3] = 0x15;
			isSend = 1;
		}
		if(isSend == 1)
			CAN_send (1, &msg_buf,100);
	}
	/*	//获取CAN日志
	if(gUploadCan == 2)
	{
		if(canlogAddr < ADDR_FLASH_SECTOR_4 + 65536)
		{
			if(obdLog == 0  || (msg->id & 0xFFF00000) == 0x18d00000 || (msg->id & 0xFFFFFFF0) == 0x000007e0 || (msg->id & 0xF0F800FF) == 0x10e80000)
			{
				sprintf((char*)&printfBuf[canlogBufLen],"%02d%02d\t%08X\t%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx\r\n",g_system_dt.minute,g_system_dt.second,msg->id,msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
				canlogBufLen += 32;
			}
			if(canlogBufLen >= 2048 || (((obdLog == 0 && canLogSecCnt > 3) || canLogSecCnt > 10) && canlogBufLen > 0))//64帧数据存一次
			{
				Flash_Write(canlogAddr,printfBuf,canlogBufLen);
				memset(printfBuf,0,sizeof(printfBuf));
				canlogAddr += canlogBufLen;
				gUploadCanSize += canlogBufLen;
				canlogBufLen = 0;
			}
		}
		else
		{
			gUploadCan = 3;
		}
		if((obdLog == 0 && canLogSecCnt > 3) || canLogSecCnt > 10)
		{
			obdLog = 0;
			gUploadCan = 3;
		}
	}
	*/
}

static uint8_t recvJ1939Len = 0;				//J1939数据接收长度
static uint8_t recvUdsLen = 0;					//UDS数据接收长度

static uint8_t udsGetIdx = 0;						//数据接收位置
static uint8_t j1939GetIdx = 0;					//数据接收位置

static uint8_t j1939recvDataFLag = 0;		//数据接收标志
static uint8_t udsrecvDataFLag = 0;			//数据接收标志

static uint8_t recvUdsData[100];				//UDS数据接收缓冲区
static uint8_t recvJ1939Data[100];			//1939数据接收缓冲区

static uint8_t sendCmdSec = 0;					//发送命令计时秒计时
static uint8_t sendCmdIdx = 0;					//发送命令索引号
static uint32_t startSecCnt = 0;				//发动机启动时间计时
static uint8_t sendCmdTimeout = 0;			//发送命令超时计时

static uint8_t getJ1939Flag[8] = {0};		//已获取OBD数据序号，不用重复获取
static uint8_t get27145Flag[8] = {0};		//已获取OBD数据序号，不用重复获取
static uint8_t get15765Flag[8] = {0};		//已获取OBD数据序号，不用重复获取

static uint8_t uds15765Cmd[5][8]={{0x02,0x09,0x02,0x00,0x00,0x00,0x00,0x00},	//VIN
																	{0x02,0x09,0x04,0x00,0x00,0x00,0x00,0x00},	//标定识别码
																	{0x02,0x09,0x06,0x00,0x00,0x00,0x00,0x00},	//标定验证码
																	{0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00},	//诊断支持就绪状态
																	{0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00},};//故障码（排放相关）

static uint8_t uds27145Cmd[8][8]={{0x03,0x22,0xf8,0x02,0x00,0x00,0x00,0x00},	//VIN
																	{0x03,0x22,0xf8,0x04,0x00,0x00,0x00,0x00},	//标准标准符
																	{0x03,0x22,0xf8,0x06,0x00,0x00,0x00,0x00},	//标定验证码
																	{0x03,0x22,0xf4,0x01,0x00,0x00,0x00,0x00},	//诊断支持状态和诊断就绪状态
																	{0x05,0x19,0x42,0x33,0x0C,0x1E,0x00,0x00},	//读取故障代码（排放相关）05 19 42 33 0C 1E FF FF 注释：19读故障码 42：掩码过滤方式读取 33：排放故障 0C：pendingDTC confirmedDTC 1E：所有等级
																	{0x03,0x22,0xf4,0x91,0x00,0x00,0x00,0x00},	//读取MIL状态：玉柴 超时：跳过
																	{0x03,0x22,0xf8,0x0B,0x00,0x00,0x00,0x00},	//柴油机IUPR值MODE0:0x03,0x22,0xf8,0x0B
																	{0x03,0x22,0xf8,0x08,0x00,0x00,0x00,0x00},};//气体机IUPR值MODE0:0x03,0x22,0xf8,0x08


static uint32_t j1939Cmd[5] =			{0xfeec,0xd300,0xfece,0xfeca,0xc200};				//VIN,标定识别码,诊断支持就绪状态,故障码,IUPR
/*
static void udsAnalyse(CAN_msg *msg)
{
	uint32_t pgn = (msg->id & 0x00FFFF00) >> 8;
	if((pgn & 0xF000) != 0xF000)
	{
		pgn &= 0xFF00;
	}
	//测探协议,放在此处是排除干扰，同时支持多个协议时确定协议
	if(gRealData.obdDiagProt == 0xFF && sendCmdTimeout > 0)
	{
		if(msg->id == 0x18daf100 || msg->id == 0x18daf110 || msg->id == 0x7E8)
		{
			if((msg->data[2] == 0x62 && msg->data[3]== 0xF8 && msg->data[4]== 0x02) && getObdDiagProtIdx == 0)
			{
				gRealData.obdDiagProt = 1;//确定是ISO27145协议
			}
			else if(msg->data[2] == 0x49 && msg->data[3]== 0x02 && msg->data[4]== 0x01 && getObdDiagProtIdx == 1)
			{
				gRealData.obdDiagProt = 0;//确定是ISO15765协议
			}
		}
		else if(msg->id == 0x1cecff00 || msg->id == 0x18ecff00)
		{
			uint32_t png = (msg->data[5] | (msg->data[6] << 8) | (msg->data[7] << 16));
			if(msg->data[0] == 0x20 && png == 0xfeec && getObdDiagProtIdx == 2)
			{
				gRealData.obdDiagProt = 2;//确定是SAE J1939协议,SAE J1939最后采用
			}
		}
	}
	if(msg->id == 0x18da00f1 || msg->id == 0x18da10f1 || msg->id == 0x7E0 || msg->id == 0x18db33f1 || msg->id == 0x7df)
	{
		//其他系统发送的诊断命令
		afterOtherCnt = 0;
	}
	if(msg->id == 0x18daf100 || msg->id == 0x18daf110 || msg->id == 0x7E8)
	{
		if(msg->id == 0x18daf100)
		{
			sendUdsId = 0x18da00f1;
		}
		else if(msg->id == 0x7E8)
		{
			sendUdsId = 0x7E0;
		}
		else if(msg->id == 0x18daf110 && sendCmdTimeout > 0 && sendUdsId == 0)//终端发出请求
		{
			sendUdsId = 0x18da10f1;
		}
		//接收单帧数据
		if((msg->data[0] & 0xF0) == 0x00)
		{
			memset(recvUdsData,0,sizeof(recvUdsData));
			recvUdsLen = msg->data[0] & 0x0F;				//获取接收数据长度
			memcpy(&recvUdsData[0],&msg->data[1],7);				//填充接收数据
			udsrecvDataFLag = 0;																//接收数据完成
		}
		//接收多帧数据首帧
		else if((msg->data[0] & 0xF0) == 0x10)
		{
			CAN_msg msg_buf = {0x18da00f1,{0x30, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,EXTENDED_TYPE,DATA_FRAME};
			memset(recvUdsData,0,sizeof(recvUdsData));
			//标准帧
			if(msg->id == 0x18daf100)
			{
				msg_buf.format = EXTENDED_TYPE;
				msg_buf.id = 0x18da00f1;
			}
			else if(msg->id == 0x18daf110)
			{
				msg_buf.format = EXTENDED_TYPE;
				msg_buf.id = 0x18da10f1;
			}
			if(msg->id == 0x7E8)
			{
				msg_buf.id = 0x7E0;
				msg_buf.format = STANDARD_TYPE;
			}
			udsGetIdx = 0x21;
			recvUdsLen = ((msg->data[0] & 0x0F) << 8) | msg->data[1];
			memcpy(&recvUdsData[0],&msg->data[2],6);
			if(sendCmdTimeout > 0)//终端发出请求，发送请求帧
			{
				osDelay(20);
				CAN_send (1, &msg_buf,0x0F00);
			}
			udsrecvDataFLag = 1;	//非终端发出请求，变更接收数据标志
		}
		//接收连续帧,按照顺序
		else if((msg->data[0] & 0xF0) == 0x20 && msg->data[0] == udsGetIdx && udsrecvDataFLag == 1)
		{
			memcpy(&recvUdsData[(udsGetIdx - 0x21) * 7 + 6],&msg->data[1],7);
			//判断接收数据完成
			if(((udsGetIdx - 0x21) * 7 + 13) >= recvUdsLen)
			{
				udsrecvDataFLag = 0;										
			}
			udsGetIdx++;
		}
		else
		{
			udsGetIdx = 0;
			udsrecvDataFLag = 0;
			return;//接收错误，返回
		}
		if(udsrecvDataFLag == 0)//接收数据完成
		{
			uint8_t codeLen;
			uint8_t head[3];;
			head[0] = recvUdsData[0]- 0x40;head[1] = recvUdsData[1];head[2] = recvUdsData[2];
			if(sendCmdTimeout > 0 && ((gRealData.obdDiagProt == 1 && memcmp(head,&uds27145Cmd[sendCmdIdx][1],3) == 0) || (gRealData.obdDiagProt == 0 && memcmp(head,&uds15765Cmd[sendCmdIdx][1],strlen((char*)&uds15765Cmd[sendCmdIdx][1]) - 1) == 0)))
			{
				sendCmdIdx++;
				sendCmdTimeout = 0;
			}
			//VIN
			if((recvUdsData[0] == 0x49 && recvUdsData[1]== 0x02 && recvUdsData[2]== 0x01) || (recvUdsData[0] == 0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x02))
			{
				if(recvUdsData[0] == 0x49)
					gRealData.obdDiagProt = 0;
				else if(recvUdsData[0] == 0x62)
					gRealData.obdDiagProt = 1;
				memcpy(gRealData.vin,&recvUdsData[3],17);
				gRealData.vin[17] = 0;
				//终端未备案激活，可以更换终端
				if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gFrimPara.setUpFlag != 0xAA && CheckVin(gRealData.vin) == 1)
				{
					//获取到车架号，重连网络
					//gprsRedial();
					Fun_Gprs_Tcp_disconnect(3);
					Fun_Gprs_Tcp_disconnect(4);
					strcpy(gSysPara.vinCode,gRealData.vin);
					System_Pare_Save();
				}
				get15765Flag[0] = 1;
				get27145Flag[0] = 1;
			}
			//校准标识符
			else if((recvUdsData[0]==0x49 && recvUdsData[1]== 0x04) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x04))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 18)
					codeLen = 18;
				memcpy(gRealData.softCalId,&recvUdsData[3],codeLen);
				get15765Flag[1] = 1;
				get27145Flag[1] = 1;
			}
			//标定验证码
			else if((recvUdsData[0]==0x49 && recvUdsData[1]== 0x06) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x06))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 18)
					codeLen = 18;
				memcpy(gRealData.cvn,&recvUdsData[3],codeLen);
				get15765Flag[2] = 1;
				get27145Flag[2] = 1;
			}
			//诊断支持状态和诊断就绪状态 ISO27145 和 ISO15765
			else if((recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x01) || (recvUdsData[0] == 0x41 && recvUdsData[1]== 0x01))
			{
				uint8_t A,B,C,D;
				//MIL状态
				gRealData.milState =  calcCanValue(7,1,&A,HIGHTLOW_MODE,CAN_MODE);
				if(recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x01)
				{
					A = recvUdsData[3];B = recvUdsData[4];C = recvUdsData[5];D = recvUdsData[6];
				}
				else
				{
					A = recvUdsData[2];B = recvUdsData[3];C = recvUdsData[4];D = recvUdsData[5];					
				}
				//燃气
				if(0)
				{
					//催化转化器
					gRealData.diagSpState.catalyst = calcCanValue(0,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//加热催化转化器
					gRealData.diagSpState.heatedCatalyst = calcCanValue(1,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//蒸发系统
					gRealData.diagSpState.evaporativeSys = calcCanValue(2,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//二次空气系统
					gRealData.diagSpState.secondaryAirSys = calcCanValue(3,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//排气传感器加热器
					gRealData.diagSpState.exhaustGasSensorHeater = calcCanValue(6,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//催化转化器
					gRealData.diagRdyState.catalyst = calcCanValue(0,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//加热催化转化器
					gRealData.diagRdyState.heatedCatalyst = calcCanValue(1,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//蒸发系统
					gRealData.diagRdyState.evaporativeSys = calcCanValue(2,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//二次空气系统
					gRealData.diagRdyState.secondaryAirSys = calcCanValue(3,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//排气传感器加热器
					gRealData.diagRdyState.exhaustGasSensorHeater = calcCanValue(6,1,&D,HIGHTLOW_MODE,CAN_MODE);
				}
				
				//柴油
				else
				{
					//增压压力控制系统
					gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(3,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//DPF监控
					gRealData.diagSpState.dpf = calcCanValue(6,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//选择性催化还原系统（SCR）或NOx吸附器
					gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(1,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//NMHC氧化催化器
					gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(0,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//增压压力控制系统
					gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(3,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//DPF监控
					gRealData.diagSpState.dpf = calcCanValue(6,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//选择性催化还原系统（SCR）或NOx吸附器
					gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(1,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//NMHC氧化催化器
					gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(0,1,&D,HIGHTLOW_MODE,CAN_MODE);					
				}
				//排气传感器
				gRealData.diagSpState.exhaustGasSensor = calcCanValue(5,1,&C,HIGHTLOW_MODE,CAN_MODE);
				//EGR和Vvt系统
				gRealData.diagSpState.egrAndVvtSys = calcCanValue(7,1,&C,HIGHTLOW_MODE,CAN_MODE);
				//失火
				gRealData.diagSpState.misfire = calcCanValue(0,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//燃油系统
				gRealData.diagSpState.fuelSys = calcCanValue(1,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//综合成分
				gRealData.diagSpState.comprehensiveComponent = calcCanValue(2,1,&B,HIGHTLOW_MODE,CAN_MODE);
				
				
				//排气传感器
				gRealData.diagRdyState.exhaustGasSensor = calcCanValue(5,1,&D,HIGHTLOW_MODE,CAN_MODE);
				//EGR和Vvt系统
				gRealData.diagRdyState.egrAndVvtSys = calcCanValue(7,1,&D,HIGHTLOW_MODE,CAN_MODE);				
				//失火
				gRealData.diagRdyState.misfire = calcCanValue(4,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//燃油系统
				gRealData.diagRdyState.fuelSys = calcCanValue(5,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//综合成分
				gRealData.diagRdyState.comprehensiveComponent = calcCanValue(6,1,&B,HIGHTLOW_MODE,CAN_MODE);
				
				//AC系统制冷剂
				//gRealData.diagSpState.acSysRefrigerant = calcCanValue(4,1,&C,HIGHTLOW_MODE,CAN_MODE);//待定
				//AC系统制冷剂
				//gRealData.diagRdyState.acSysRefrigerant = calcCanValue(4,1,&D,HIGHTLOW_MODE,CAN_MODE);//待定
				//冷启动辅助系统
				//gRealData.diagSpState.coldStartAidSys = 0;
				//冷启动辅助系统
				//gRealData.diagRdyState.coldStartAidSys = 0;
				
				//get15765Flag[3] = 1;
				//get27145Flag[3] = 1;
			}
			//ISO15765故障码
			else if(recvUdsData[0]==0x43)
			{
				uint8_t i;
				uint8_t cnt = 0;
				for(i=0;i < recvUdsData[1];i++)
				{
					cnt++;
					//gRealData.faultCode[i] = (recvUdsData[i*2+2] << 8) | (recvUdsData[i*2+3] << 0);
					gRealData.faultCode[i] = (recvUdsData[i*2+2] << 8) | (recvUdsData[i*2+3] << 0);//与27145协议故障码对齐
				}
				gRealData.faultCodeCnt = cnt;
				//get15765Flag[4] = 1;
			}
			//ISO27145故障码
			else if(recvUdsData[0]==0x59 && recvUdsData[1]== 0x42 && recvUdsData[2]== 0x33)
			{
				uint8_t i;
				uint8_t cnt = 0;
				for(i=6;i < recvUdsLen;i += 5)
				{
					gRealData.faultCode[cnt++] = (recvUdsData[i+1] << 24) | (recvUdsData[i+2] << 16) | (recvUdsData[i+3] << 8) | (recvUdsData[i+4] << 0);
				}
				gRealData.faultCodeCnt = cnt;
				//get27145Flag[4] = 1;
			}
			//ISO 27145MIL灯状态
			else if(recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x91)
			{
				uint8_t A;
				A = recvUdsData[3];
				//MIL状态
				gRealData.milState =  calcCanValue(0,4,&A,HIGHTLOW_MODE,CAN_MODE);
				//get27145Flag[5] = 1;
			}
			//IUPR值
			else if((recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x0B) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x08))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 36)
					codeLen = 36;
				memcpy(gRealData.iuprVal,&recvUdsData[3],codeLen);
				catype = 6;
				//get27145Flag[6] = 1;
				if(sendCmdIdx == 7)
				{
					//当前未柴油机，跳过气体机
					sendCmdIdx++;
				}
			}
		}
		return;//UDS解析完成，提前离开
	}
	else if(msg->id == 0x1cecff00 || msg->id == 0x18ecff00)
	{
		//J1939诊断 byte0:ControlByte byte1~byte2:TotalMessageSizeBAM byte3:TotalNumberOfPacketsBAM byte5~7:PGNumber
		if(msg->data[0] == 0x20)//ControlByte 0x20:BAM
		{
			memset(recvJ1939Data,0,sizeof(recvJ1939Data));
			memcpy(recvJ1939Data,msg->data,8);
			j1939GetIdx = 8;
			recvJ1939Len = msg->data[1] | (msg->data[2] << 8);
			j1939recvDataFLag = 1;
		}
		return;//收到预告帧，提前离开
	}
	else if(msg->id == 0x1cebff00 || msg->id == 0x18ebff00)
	{
		//UDS协议不通，走J1939协议
		if((j1939GetIdx  - 8) != (msg->data[0] - 1) * 7 || j1939recvDataFLag == 0)
		{
			memset(recvJ1939Data,0,sizeof(recvJ1939Data));
			j1939GetIdx = 0;
			j1939recvDataFLag = 0;
			return;//接收顺序错误，提前离开
		}
		if((j1939GetIdx  - 8) == (msg->data[0] - 1) * 7 && j1939recvDataFLag == 1)//按顺序接收
		{
			memcpy(&recvJ1939Data[j1939GetIdx],&msg->data[1],7);
			j1939GetIdx += 7;
			if(msg->data[0] == recvJ1939Data[3] && recvJ1939Data[0] == 0x20 && recvJ1939Len > 0)	//接收完成，解析
			{
				uint8_t* pJ1939Data = &recvJ1939Data[8];
				pgn = (recvJ1939Data[5] | (recvJ1939Data[6] << 8) | (recvJ1939Data[7] << 16));
				j1939recvDataFLag = 0;
				if(gRealData.obdDiagProt == 2)
				{
					uint8_t dataLen;
					if(pgn == 0xfeec)//车架号
					{
						memcpy(gRealData.vin,pJ1939Data,17);
						gRealData.vin[17] = 0;
						//if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gSysPara.vinCode[0] != 'L')
						//终端未备案激活，可以更换终端
						if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gFrimPara.setUpFlag != 0xAA && CheckVin(gRealData.vin) == 1)
						{
							//获取到车架号,重连网络
							//gprsRedial();
							Fun_Gprs_Tcp_disconnect(3);
							Fun_Gprs_Tcp_disconnect(4);
							strcpy(gSysPara.vinCode,gRealData.vin);
							System_Pare_Save();
						}
						getJ1939Flag[0] = 1;
					}
					else if(pgn == 0xd300)//标定识别号，标定验证码
					{
						memcpy(gRealData.cvn,pJ1939Data,4);
						memcpy(gRealData.softCalId,&pJ1939Data[4],16);
						getJ1939Flag[1] = 1;
					}
					else if(pgn == 0xfeca)//故障码
					{
						uint8_t jj;
						//MIL状态
						gRealData.milState = calcRealValue(6,2,1,0,CALC_MODE,pJ1939Data,HIGHTLOW_MODE,CAN_MODE);
						gRealData.faultCodeCnt = ((recvJ1939Len - 2)/4) % 20;//最大只存20个故障码
						for(jj = 0;jj < gRealData.faultCodeCnt;jj++)
						{
							gRealData.faultCode[0] = (pJ1939Data[8 + jj*4] << 0) |  (pJ1939Data[3+jj*4] << 8) | (pJ1939Data[4 + jj*4] << 16) | (pJ1939Data[5 +jj*4] << 24 );
						}
						getJ1939Flag[3] = 1;
					}
					else if(pgn == 0xc200)//IUPR
					{
						if( recvJ1939Data[1] > 36)
							dataLen = 36;
						memcpy(gRealData.iuprVal,pJ1939Data,dataLen);
						if(gRealData.obdDiagProt == 2)
						{
							catype = 6;
						}
						getJ1939Flag[4] = 1;
					}
				}
				if(pgn == 0xFEE3)//EC1
				{
					//发动机最大参考扭矩 1
					gRealData.engReferenceTorque = pJ1939Data[19] | (pJ1939Data[20] << 8);
				}
			}
		}
	}
	else if(pgn == 0x00FECA && gRealData.obdDiagProt == 2)//MIL状态和故障码
	{
		//getJ1939Flag[3] = 1;
		//MIL状态
		gRealData.milState = calcRealValue(6,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		if(msg->data[2] != 0 || msg->data[3] != 0  || msg->data[4] != 0 || msg->data[5] != 0)
		{
			gRealData.faultCodeCnt = 1;
			gRealData.faultCode[0] = (msg->data[2] << 0) |  (msg->data[3] << 8) | (msg->data[4] << 16) | (msg->data[5] << 24 );
		}
	}
	else if(pgn == 0x00FECE && gRealData.obdDiagProt == 2)//就绪状态
	{
		//getJ1939Flag[2] = 1;
		// 诊断支持状态 
		//催化转化器
		gRealData.diagSpState.catalyst = calcCanValue(32,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//加热催化转化器
		gRealData.diagSpState.heatedCatalyst = calcCanValue(33,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//蒸发系统
		gRealData.diagSpState.evaporativeSys = calcCanValue(34,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//二次空气系统
		gRealData.diagSpState.secondaryAirSys = calcCanValue(35,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//AC系统制冷剂
		gRealData.diagSpState.acSysRefrigerant = calcCanValue(36,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//排气传感器
		gRealData.diagSpState.exhaustGasSensor = calcCanValue(37,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//排气传感器加热器
		gRealData.diagSpState.exhaustGasSensorHeater = calcCanValue(38,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//EGR和Vvt系统
		gRealData.diagSpState.egrAndVvtSys = calcCanValue(39,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//冷启动辅助系统
		gRealData.diagSpState.coldStartAidSys = calcCanValue(40,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		
		//诊断就绪状态
		//催化转化器
		gRealData.diagRdyState.catalyst = calcCanValue(48,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//加热催化转化器
		gRealData.diagRdyState.heatedCatalyst = calcCanValue(49,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//蒸发系统
		gRealData.diagRdyState.evaporativeSys = calcCanValue(50,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//二次空气系统
		gRealData.diagRdyState.secondaryAirSys = calcCanValue(51,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//AC系统制冷剂
		gRealData.diagRdyState.acSysRefrigerant = calcCanValue(52,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//排气传感器
		gRealData.diagRdyState.exhaustGasSensor = calcCanValue(53,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//排气传感器加热器
		gRealData.diagRdyState.exhaustGasSensorHeater = calcCanValue(54,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//EGR和Vvt系统
		gRealData.diagRdyState.egrAndVvtSys = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//冷启动辅助系统
		gRealData.diagRdyState.coldStartAidSys = calcCanValue(56,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		
		
		
		//增压压力控制系统
		gRealData.diagRdyState.boostPressureCtrlSys = calcCanValue(57,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//DPF监控
		gRealData.diagRdyState.dpf = calcCanValue(58,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//选择性催化还原系统（SCR）或NOx吸附器
		gRealData.diagRdyState.scrOrNOxAdsorber = calcCanValue(59,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//NMHC氧化催化器
		gRealData.diagRdyState.nmhcConvertingCatalyst = calcCanValue(60,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		
		
		//增压压力控制系统
		gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(41,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//DPF监控
		gRealData.diagSpState.dpf = calcCanValue(42,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//选择性催化还原系统（SCR）或NOx吸附器
		gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(43,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//NMHC氧化催化器
		gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(44,1,msg->data,HIGHTLOW_MODE,CAN_MODE);


		//失火
		gRealData.diagSpState.misfire = calcCanValue(24,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//燃油系统
		gRealData.diagSpState.fuelSys = calcCanValue(25,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//综合成分
		gRealData.diagSpState.comprehensiveComponent = calcCanValue(26,1,msg->data,HIGHTLOW_MODE,CAN_MODE);

		//失火
		gRealData.diagRdyState.misfire = calcCanValue(28,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//燃油系统
		gRealData.diagRdyState.fuelSys = calcCanValue(29,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//综合成分
		gRealData.diagRdyState.comprehensiveComponent = calcCanValue(30,1,msg->data,HIGHTLOW_MODE,CAN_MODE);			
	}
	if(sendCmdTimeout > 0 && gRealData.obdDiagProt == 2 && j1939Cmd[sendCmdIdx] == pgn)
	{
		sendCmdIdx++;
		sendCmdTimeout = 0;
	}
}
											
static void Udstask(void)
{
	//发动机启动，请求诊断数据
	if(g_system_dt.second != sendCmdSec)
	{
		sendCmdSec = g_system_dt.second;
		//应答超时
		if(sendCmdTimeout > 0)
		{
			sendCmdTimeout--;								//请求超时控制
			if(sendCmdTimeout == 1)					//3秒超时
			{
				//J1939不支持IUPR：超时跳过
				if(gRealData.obdDiagProt == 2 && sendCmdIdx == 4)
				{
					sendCmdIdx++;
				}
				//ISO27145不支持此种读MIL状态 或IUPR状态：超时跳过
				else if(gRealData.obdDiagProt == 1 && (sendCmdIdx == 5 || sendCmdIdx == 6 || sendCmdIdx == 7))
				{
					sendCmdIdx++;
				}
				sendCmdTimeout = 0;
			}
		}
		afterOtherCnt = (afterOtherCnt < 0xFF) ? (afterOtherCnt + 1) : afterOtherCnt;
		if(afterOtherCnt >= 4)
			startSecCnt = (startSecCnt < 0xFF) ? (startSecCnt + 1) : startSecCnt;//启动发动机48秒后2分30秒内
		if(startSecCnt >= 45 && startSecCnt <= 150 && sendCmdTimeout == 0 && gRealData.engineSpeed >= 100 && gRealData.engineSpeed < 0xFFFE && afterOtherCnt >= 4)
		{
			//测探协议顺序控制
			getObdDiagProtIdx++;
			getObdDiagProtIdx = getObdDiagProtIdx % 3;
			if((gRealData.obdDiagProt == 0 && sendCmdIdx < (sizeof(uds15765Cmd) / 8)) || (gRealData.obdDiagProt == 1 && sendCmdIdx < (sizeof(uds27145Cmd) / 8)) || (gRealData.obdDiagProt == 2 && sendCmdIdx < (sizeof(j1939Cmd) / 4)) || gRealData.obdDiagProt == 0xFF)
			{
				CAN_msg msg_buf = {0x18eafff9,{ 0x30, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,EXTENDED_TYPE,DATA_FRAME};
				osDelay(100);
				if(gRealData.obdDiagProt == 1 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 0))//ISO27145协议或未知
				{
					if(get15765Flag[sendCmdIdx] == 1)//不需要重复获取
					{
						sendCmdIdx++;
						if(sendCmdIdx >= sizeof(uds27145Cmd) / 8)
						{
							return;
						}
					}
					memcpy(msg_buf.data,uds27145Cmd[sendCmdIdx],8);
					sendCmdTimeout = 4;
				}
				if(gRealData.obdDiagProt == 0 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 1))//ISO15765协议或未知
				{
					if(get15765Flag[sendCmdIdx] == 1)//不需要重复获取
					{
						sendCmdIdx++;
						if(sendCmdIdx >= sizeof(uds15765Cmd) / 8)
						{
							return;
						}
					}
					memcpy(msg_buf.data,uds15765Cmd[sendCmdIdx],8);
					sendCmdTimeout = 4;
				}
				if(sendCmdTimeout > 0)
				{
					//扩展帧
					if(sendUdsId == 0 || sendUdsId == 0x18DA00F1 || sendUdsId == 0x18DA10F1)
					{
						msg_buf.format = EXTENDED_TYPE;	
						//功能寻址：0x18DB33F1，物理寻址：0x18DA00F1 0号ECU响应：0x18DAF100
						if(gRealData.obdDiagProt == 0 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 1))
						{
							msg_buf.id = 0x18DB33F1;
							osDelay(10);					
							CAN_send (1, &msg_buf,0x0F00);							
						}
						else
						{
							if(sendUdsId == 0 || sendUdsId == 0x18DA00F1)
							{
								msg_buf.id = 0x18DA00F1;
								osDelay(10);					
								CAN_send (1, &msg_buf,0x0F00);
							}
							if(sendUdsId == 0 || sendUdsId == 0x18DA10F1)
							{
								msg_buf.id = 0x18DA10F1;
								osDelay(10);					
								CAN_send (1, &msg_buf,0x0F00);							
							}
						}
					}
					//标准帧
					if(sendUdsId == 0 || sendUdsId == 0x7E0)
					{
						//功能寻址：0x7DF，物理寻址：0x7E0 0号ECU响应：7E8
						msg_buf.id = 0x7E0;
						msg_buf.format = STANDARD_TYPE;
						osDelay(10);
						CAN_send (1, &msg_buf,0x0F00);
					}
				}
				if(gRealData.obdDiagProt == 2 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 2))//SAE J1939协议或未知
				{
					msg_buf.data[0] = j1939Cmd[sendCmdIdx] >> 0;
					msg_buf.data[1] = j1939Cmd[sendCmdIdx] >> 8;
					msg_buf.data[2] = j1939Cmd[sendCmdIdx] >> 16;
					msg_buf.id = 0x18eafff9;
					msg_buf.format = EXTENDED_TYPE;
					osDelay(10);
					CAN_send (1, &msg_buf,0x0F00);
					sendCmdTimeout = 4;
				}
			}
			else
			{
				//OBD诊断完成
				gTerminalState.obdState = 1;
				if(gSysPara.carType != catype)
				{
					gSysPara.carType = catype;
					System_Pare_Save();
				}
			}
		}
	}
	
	//发动机熄火
	if(gRealData.engineSpeed <= 10 || gRealData.engineSpeed >= 0xFFFE)
	{
		scrInReady = 0;
		scrOutReady = 0;
		memset(getJ1939Flag,0,sizeof(getJ1939Flag));
		memset(get27145Flag,0,sizeof(get27145Flag));
		memset(get15765Flag,0,sizeof(get15765Flag));
		startSecCnt = 0;
		sendCmdIdx = 0;
		gTerminalState.obdState = 0;
	}
	gTerminalState.getCarData = gTerminalState.obdState;
}
*/
static void udsAnalyse(CAN_msg *msg)
{
	uint32_t pgn = (msg->id & 0x00FFFF00) >> 8;
	uint8_t source = msg->id & 0xFF;
	if((pgn & 0xF000) != 0xF000)
	{
		pgn &= 0xFF00;
	}
	//测探协议,放在此处是排除干扰，同时支持多个协议时确定协议
	if(gRealData.obdDiagProt == 0xFF && sendCmdTimeout > 0)
	{
		if(msg->id == 0x18daf100 || msg->id == 0x18daf110 || msg->id == 0x7E8)
		{
			if((msg->data[2] == 0x62 && msg->data[3]== 0xF8 && msg->data[4]== 0x02) && getObdDiagProtIdx == 0)
			{
				gRealData.obdDiagProt = 1;//确定是ISO27145协议
			}
			else if(msg->data[2] == 0x49 && msg->data[3]== 0x02 && msg->data[4]== 0x01 && getObdDiagProtIdx == 1)
			{
				gRealData.obdDiagProt = 0;//确定是ISO15765协议
			}
		}
		else if(msg->id == 0x1cecff00 || msg->id == 0x18ecff00)
		{
			uint32_t png = (msg->data[5] | (msg->data[6] << 8) | (msg->data[7] << 16));
			if(msg->data[0] == 0x20 && png == 0xfeec && getObdDiagProtIdx == 2)
			{
				gRealData.obdDiagProt = 2;//确定是SAE J1939协议,SAE J1939最后采用
			}
		}
	}
	if(msg->id == 0x18da00f1 || msg->id == 0x18da10f1 || msg->id == 0x7E0 || msg->id == 0x18db33f1 || msg->id == 0x7df)
	{
		//其他系统发送的诊断命令，终端先不发送，3秒后才能发送
		afterOtherCnt = 0;
	}
	if(msg->id == 0x18daf100 || msg->id == 0x18daf110 || msg->id == 0x7E8)
	{
		if(msg->id == 0x18daf100)
		{
			sendUdsId = 0x18da00f1;
		}
		else if(msg->id == 0x7E8)
		{
			sendUdsId = 0x7E0;
		}
		else if(msg->id == 0x18daf110 && sendCmdTimeout > 0 && sendUdsId == 0)//终端发出请求
		{
			sendUdsId = 0x18da10f1;
		}
		//接收单帧数据
		if((msg->data[0] & 0xF0) == 0x00)
		{
			memset(recvUdsData,0,sizeof(recvUdsData));
			recvUdsLen = msg->data[0] & 0x0F;								//获取接收数据长度
			memcpy(&recvUdsData[0],&msg->data[1],7);				//填充接收数据
			udsrecvDataFLag = 0;														//接收数据完成
		}
		//接收多帧数据首帧
		else if((msg->data[0] & 0xF0) == 0x10)
		{
			CAN_msg msg_buf = {0x18da00f1,{0x30, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,EXTENDED_TYPE,DATA_FRAME};
			memset(recvUdsData,0,sizeof(recvUdsData));
			//标准帧
			if(msg->id == 0x18daf100)
			{
				msg_buf.format = EXTENDED_TYPE;
				msg_buf.id = 0x18da00f1;
			}
			else if(msg->id == 0x18daf110)
			{
				msg_buf.format = EXTENDED_TYPE;
				msg_buf.id = 0x18da10f1;
			}
			if(msg->id == 0x7E8)
			{
				msg_buf.id = 0x7E0;
				msg_buf.format = STANDARD_TYPE;
			}
			//终端发出请求，发送请求帧
			udsGetIdx = 0x21;
			recvUdsLen = ((msg->data[0] & 0x0F) << 8) | msg->data[1];
			memcpy(&recvUdsData[0],&msg->data[2],6);
			//确保是终端发出的命令
			if(sendCmdTimeout > 0)
			{
				osDelay(20);
				CAN_send (1, &msg_buf,0x0F00);
			}
			//UDS接收数据标志
			udsrecvDataFLag = 1;
		}
		//接收连续帧,按照顺序
		else if((msg->data[0] & 0xF0) == 0x20 && msg->data[0] == udsGetIdx && udsrecvDataFLag == 1)
		{
			memcpy(&recvUdsData[(udsGetIdx - 0x21) * 7 + 6],&msg->data[1],7);
			//判断接收数据完成
			if(((udsGetIdx - 0x21) * 7 + 13) >= recvUdsLen)
			{
				udsrecvDataFLag = 0;										
			}
			udsGetIdx++;
		}
		else
		{
			udsGetIdx = 0;
			udsrecvDataFLag = 0;
			return;//接收错误，返回
		}
		if(udsrecvDataFLag == 0)//接收数据完成
		{
			uint8_t codeLen;
			uint8_t head[3];;
			head[0] = recvUdsData[0]- 0x40;head[1] = recvUdsData[1];head[2] = recvUdsData[2];
			if(sendCmdTimeout > 0 && ((gRealData.obdDiagProt == 1 && memcmp(head,&uds27145Cmd[sendCmdIdx][1],3) == 0) || (gRealData.obdDiagProt == 0 && memcmp(head,&uds15765Cmd[sendCmdIdx][1],strlen((char*)&uds15765Cmd[sendCmdIdx][1]) - 1) == 0)))
			{
				sendCmdIdx++;
				sendCmdTimeout = 0;
			}
			//VIN
			if((recvUdsData[0] == 0x49 && recvUdsData[1]== 0x02 && recvUdsData[2]== 0x01) || (recvUdsData[0] == 0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x02))
			{
				if(recvUdsData[0] == 0x49)
					gRealData.obdDiagProt = 0;
				else if(recvUdsData[0] == 0x62)
					gRealData.obdDiagProt = 1;
				memcpy(gRealData.vin,&recvUdsData[3],17);
				gRealData.vin[17] = 0;
				//终端未备案激活，可以更换终端
				if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gFrimPara.setUpFlag != 0xAA && CheckVin(gRealData.vin) == 1)
				{
					//获取到车架号，重连网络
					//gprsRedial();
					Fun_Gprs_Tcp_disconnect(3);
					Fun_Gprs_Tcp_disconnect(4);
					strcpy(gSysPara.vinCode,gRealData.vin);
					System_Pare_Save();
				}
				get15765Flag[0] = 1;
				get27145Flag[0] = 1;
			}
			//校准标识符
			else if((recvUdsData[0]==0x49 && recvUdsData[1]== 0x04) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x04))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 18)
					codeLen = 18;
				memcpy(gRealData.softCalId,&recvUdsData[3],codeLen);
				get15765Flag[1] = 1;
				get27145Flag[1] = 1;
			}
			//标定验证码
			else if((recvUdsData[0]==0x49 && recvUdsData[1]== 0x06) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x06))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 18)
					codeLen = 18;
				memcpy(gRealData.cvn,&recvUdsData[3],codeLen);
				get15765Flag[2] = 1;
				get27145Flag[2] = 1;
			}
			//诊断支持状态和诊断就绪状态 ISO27145 和 ISO15765
			else if((recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x01) || (recvUdsData[0] == 0x41 && recvUdsData[1]== 0x01))
			{
				uint8_t A,B,C,D;
				//MIL状态
				gRealData.milState =  calcCanValue(7,1,&A,HIGHTLOW_MODE,CAN_MODE);
				if(recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x01)
				{
					A = recvUdsData[3];B = recvUdsData[4];C = recvUdsData[5];D = recvUdsData[6];
				}
				else
				{
					A = recvUdsData[2];B = recvUdsData[3];C = recvUdsData[4];D = recvUdsData[5];					
				}
				//燃气
				if(0)
				{
					//催化转化器
					gRealData.diagSpState.catalyst = calcCanValue(0,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//加热催化转化器
					gRealData.diagSpState.heatedCatalyst = calcCanValue(1,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//蒸发系统
					gRealData.diagSpState.evaporativeSys = calcCanValue(2,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//二次空气系统
					gRealData.diagSpState.secondaryAirSys = calcCanValue(3,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//排气传感器加热器
					gRealData.diagSpState.exhaustGasSensorHeater = calcCanValue(6,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//催化转化器
					gRealData.diagRdyState.catalyst = calcCanValue(0,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//加热催化转化器
					gRealData.diagRdyState.heatedCatalyst = calcCanValue(1,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//蒸发系统
					gRealData.diagRdyState.evaporativeSys = calcCanValue(2,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//二次空气系统
					gRealData.diagRdyState.secondaryAirSys = calcCanValue(3,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//排气传感器加热器
					gRealData.diagRdyState.exhaustGasSensorHeater = calcCanValue(6,1,&D,HIGHTLOW_MODE,CAN_MODE);
				}
				
				//柴油
				else
				{
					//增压压力控制系统
					gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(3,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//DPF监控
					gRealData.diagSpState.dpf = calcCanValue(6,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//选择性催化还原系统（SCR）或NOx吸附器
					gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(1,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//NMHC氧化催化器
					gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(0,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//增压压力控制系统
					gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(3,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//DPF监控
					gRealData.diagSpState.dpf = calcCanValue(6,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//选择性催化还原系统（SCR）或NOx吸附器
					gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(1,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//NMHC氧化催化器
					gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(0,1,&D,HIGHTLOW_MODE,CAN_MODE);					
				}
				//排气传感器
				gRealData.diagSpState.exhaustGasSensor = calcCanValue(5,1,&C,HIGHTLOW_MODE,CAN_MODE);
				//EGR和Vvt系统
				gRealData.diagSpState.egrAndVvtSys = calcCanValue(7,1,&C,HIGHTLOW_MODE,CAN_MODE);
				//失火
				gRealData.diagSpState.misfire = calcCanValue(0,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//燃油系统
				gRealData.diagSpState.fuelSys = calcCanValue(1,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//综合成分
				gRealData.diagSpState.comprehensiveComponent = calcCanValue(2,1,&B,HIGHTLOW_MODE,CAN_MODE);
				
				
				//排气传感器
				gRealData.diagRdyState.exhaustGasSensor = calcCanValue(5,1,&D,HIGHTLOW_MODE,CAN_MODE);
				//EGR和Vvt系统
				gRealData.diagRdyState.egrAndVvtSys = calcCanValue(7,1,&D,HIGHTLOW_MODE,CAN_MODE);				
				//失火
				gRealData.diagRdyState.misfire = calcCanValue(4,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//燃油系统
				gRealData.diagRdyState.fuelSys = calcCanValue(5,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//综合成分
				gRealData.diagRdyState.comprehensiveComponent = calcCanValue(6,1,&B,HIGHTLOW_MODE,CAN_MODE);
				
				//AC系统制冷剂
				//gRealData.diagSpState.acSysRefrigerant = calcCanValue(4,1,&C,HIGHTLOW_MODE,CAN_MODE);//待定
				//AC系统制冷剂
				//gRealData.diagRdyState.acSysRefrigerant = calcCanValue(4,1,&D,HIGHTLOW_MODE,CAN_MODE);//待定
				//冷启动辅助系统
				//gRealData.diagSpState.coldStartAidSys = 0;
				//冷启动辅助系统
				//gRealData.diagRdyState.coldStartAidSys = 0;
				
				//get15765Flag[3] = 1;
				//get27145Flag[3] = 1;
			}
			//ISO15765故障码
			else if(recvUdsData[0]==0x43)
			{
				uint8_t i;
				uint8_t cnt = 0;
				for(i=0;i < recvUdsData[1];i++)
				{
					cnt++;
					//gRealData.faultCode[i] = (recvUdsData[i*2+2] << 8) | (recvUdsData[i*2+3] << 0);
					gRealData.faultCode[i] = (recvUdsData[i*2+2] << 8) | (recvUdsData[i*2+3] << 0);//与27145协议故障码对齐
				}
				gRealData.faultCodeCnt = cnt;
				//需要读最新数据
				//get15765Flag[4] = 1;
			}
			//ISO27145故障码
			else if(recvUdsData[0]==0x59 && recvUdsData[1]== 0x42 && recvUdsData[2]== 0x33)
			{
				uint8_t i;
				uint8_t cnt = 0;
				for(i=6;i < recvUdsLen;i += 5)
				{
					gRealData.faultCode[cnt++] = (recvUdsData[i+1] << 24) | (recvUdsData[i+2] << 16) | (recvUdsData[i+3] << 8) | (recvUdsData[i+4] << 0);
				}
				gRealData.faultCodeCnt = cnt;
				//需要读最新数据
				//get27145Flag[4] = 1;
			}
			//ISO 27145MIL灯状态
			else if(recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x91)
			{
				uint8_t A;
				A = recvUdsData[3];
				//MIL状态
				gRealData.milState =  calcCanValue(0,4,&A,HIGHTLOW_MODE,CAN_MODE);
				//需要读最新数据
				//get27145Flag[5] = 1;
			}
			//IUPR值
			else if((recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x0B) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x08))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 36)
					codeLen = 36;
				memcpy(gRealData.iuprVal,&recvUdsData[3],codeLen);
				emissionLevel = 1;
				if(sendCmdIdx == 7)
				{
					//当前未柴油机，跳过气体机
					sendCmdIdx++;
				}
				//需要读最新数据
				//get27145Flag[6] = 1;
			}
			//ISO 27145  Sensor 2 Oxygen sensor Output Voltage
			else if(recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x15)
			{
				uint8_t A;
				A = recvUdsData[3];
				//TWC催化器下游氧传感器输出值(电压)
				gRealData.twcUpperOxySensor = A * 0.005;
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
			}
		}
		return;//UDS解析完成，提前离开
	}
	else if(msg->id == 0x1cecff00 || msg->id == 0x18ecff00)
	{
		//J1939诊断 byte0:ControlByte byte1~byte2:TotalMessageSizeBAM byte3:TotalNumberOfPacketsBAM byte5~7:PGNumber
		if(msg->data[0] == 0x20)//ControlByte 0x20:BAM
		{
			memset(recvJ1939Data,0,sizeof(recvJ1939Data));
			memcpy(recvJ1939Data,msg->data,8);
			j1939GetIdx = 8;
			recvJ1939Len = msg->data[1] | (msg->data[2] << 8);
			j1939recvDataFLag = 1;
		}
		return;//收到预告帧，提前离开
	}
	else if(msg->id == 0x1cebff00 || msg->id == 0x18ebff00)
	{
		//UDS协议不通，走J1939协议
		if((j1939GetIdx  - 8) != (msg->data[0] - 1) * 7 || j1939recvDataFLag == 0)
		{
			memset(recvJ1939Data,0,sizeof(recvJ1939Data));
			j1939GetIdx = 0;
			j1939recvDataFLag = 0;
			return;//接收顺序错误，提前离开
		}
		if((j1939GetIdx  - 8) == (msg->data[0] - 1) * 7 && j1939recvDataFLag == 1)//按顺序接收
		{
			memcpy(&recvJ1939Data[j1939GetIdx],&msg->data[1],7);
			j1939GetIdx += 7;
			if(msg->data[0] == recvJ1939Data[3] && recvJ1939Data[0] == 0x20 && recvJ1939Len > 0)	//接收完成，解析
			{
				uint8_t* pJ1939Data = &recvJ1939Data[8];
				pgn = (recvJ1939Data[5] | (recvJ1939Data[6] << 8) | (recvJ1939Data[7] << 16));
				j1939recvDataFLag = 0;
				if(gRealData.obdDiagProt == 2)
				{
					uint8_t dataLen;
					if(pgn == 0xfeec)//车架号
					{
						memcpy(gRealData.vin,pJ1939Data,17);
						gRealData.vin[17] = 0;
						//if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gSysPara.vinCode[0] != 'L')
						//终端未备案激活，可以更换终端
						if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gFrimPara.setUpFlag != 0xAA && CheckVin(gRealData.vin) == 1)
						{
							//获取到车架号,重连网络
							//gprsRedial();
							Fun_Gprs_Tcp_disconnect(3);
							Fun_Gprs_Tcp_disconnect(4);
							strcpy(gSysPara.vinCode,gRealData.vin);
							System_Pare_Save();
						}
						getJ1939Flag[0] = 1;
					}
					else if(pgn == 0xd300)//标定识别号，标定验证码
					{
						memcpy(gRealData.cvn,pJ1939Data,4);
						memcpy(gRealData.softCalId,&pJ1939Data[4],16);
						getJ1939Flag[1] = 1;
					}
					else if(pgn == 0xfeca)//故障码
					{
						uint8_t jj;
						//MIL状态
						gRealData.milState = calcRealValue(6,2,1,0,CALC_MODE,pJ1939Data,HIGHTLOW_MODE,CAN_MODE);
						gRealData.faultCodeCnt = ((recvJ1939Len - 2)/4) % 20;//最大只存20个故障码
						for(jj = 0;jj < gRealData.faultCodeCnt;jj++)
						{
							gRealData.faultCode[0] = (pJ1939Data[8 + jj*4] << 0) |  (pJ1939Data[3+jj*4] << 8) | (pJ1939Data[4 + jj*4] << 16) | (pJ1939Data[5 +jj*4] << 24 );
						}
						getJ1939Flag[3] = 1;
					}
					else if(pgn == 0xc200)//IUPR
					{
						if( recvJ1939Data[1] > 36)
							dataLen = 36;
						memcpy(gRealData.iuprVal,pJ1939Data,dataLen);
						if(gRealData.obdDiagProt == 2)
						{
							emissionLevel = 1;
						}
						getJ1939Flag[4] = 1;
					}
				}
				if(pgn == 0xFEE3)//EC1
				{
					//发动机最大参考扭矩 1
					gRealData.engReferenceTorque = pJ1939Data[19] | (pJ1939Data[20] << 8);
					engReferenceTorqueStamp = 0xFFFFFFFF;		
				}
			}
		}
	}
	else if(source == 0)
	{
		if(pgn == 0x00FECA && gRealData.obdDiagProt == 2)//MIL状态和故障码
		{
			//getJ1939Flag[3] = 1;
			//MIL状态
			//gRealData.milState = calcRealValue(6,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			if(msg->data[2] != 0 || msg->data[3] != 0  || msg->data[4] != 0 || msg->data[5] != 0)
			{
				gRealData.faultCodeCnt = 1;
				gRealData.faultCode[0] = (msg->data[2] << 0) |  (msg->data[3] << 8) | (msg->data[4] << 16) | (msg->data[5] << 24 );
			}
		}
		else if(pgn == 0x00FD07 && gRealData.obdDiagProt == 2)//MIL状态
		{
			//MIL状态
			gRealData.milState = calcRealValue(6,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		else if(pgn == 0x00FECE && gRealData.obdDiagProt == 2)//就绪状态
		{
			//getJ1939Flag[2] = 1;
			/* 诊断支持状态 */
			//催化转化器
			gRealData.diagSpState.catalyst = calcCanValue(32,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//加热催化转化器
			gRealData.diagSpState.heatedCatalyst = calcCanValue(33,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//蒸发系统
			gRealData.diagSpState.evaporativeSys = calcCanValue(34,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//二次空气系统
			gRealData.diagSpState.secondaryAirSys = calcCanValue(35,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//AC系统制冷剂
			gRealData.diagSpState.acSysRefrigerant = calcCanValue(36,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//排气传感器
			gRealData.diagSpState.exhaustGasSensor = calcCanValue(37,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//排气传感器加热器
			gRealData.diagSpState.exhaustGasSensorHeater = calcCanValue(38,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//EGR和Vvt系统
			gRealData.diagSpState.egrAndVvtSys = calcCanValue(39,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//冷启动辅助系统
			gRealData.diagSpState.coldStartAidSys = calcCanValue(40,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			
			/*诊断就绪状态*/
			//催化转化器
			gRealData.diagRdyState.catalyst = calcCanValue(48,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//加热催化转化器
			gRealData.diagRdyState.heatedCatalyst = calcCanValue(49,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//蒸发系统
			gRealData.diagRdyState.evaporativeSys = calcCanValue(50,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//二次空气系统
			gRealData.diagRdyState.secondaryAirSys = calcCanValue(51,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//AC系统制冷剂
			gRealData.diagRdyState.acSysRefrigerant = calcCanValue(52,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//排气传感器
			gRealData.diagRdyState.exhaustGasSensor = calcCanValue(53,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//排气传感器加热器
			gRealData.diagRdyState.exhaustGasSensorHeater = calcCanValue(54,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//EGR和Vvt系统
			gRealData.diagRdyState.egrAndVvtSys = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//冷启动辅助系统
			gRealData.diagRdyState.coldStartAidSys = calcCanValue(56,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			
			
			
			//增压压力控制系统
			gRealData.diagRdyState.boostPressureCtrlSys = calcCanValue(57,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//DPF监控
			gRealData.diagRdyState.dpf = calcCanValue(58,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//选择性催化还原系统（SCR）或NOx吸附器
			gRealData.diagRdyState.scrOrNOxAdsorber = calcCanValue(59,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//NMHC氧化催化器
			gRealData.diagRdyState.nmhcConvertingCatalyst = calcCanValue(60,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			
			
			//增压压力控制系统
			gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(41,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//DPF监控
			gRealData.diagSpState.dpf = calcCanValue(42,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//选择性催化还原系统（SCR）或NOx吸附器
			gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(43,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//NMHC氧化催化器
			gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(44,1,msg->data,HIGHTLOW_MODE,CAN_MODE);


			//失火
			gRealData.diagSpState.misfire = calcCanValue(24,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//燃油系统
			gRealData.diagSpState.fuelSys = calcCanValue(25,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//综合成分
			gRealData.diagSpState.comprehensiveComponent = calcCanValue(26,1,msg->data,HIGHTLOW_MODE,CAN_MODE);

			//失火
			gRealData.diagRdyState.misfire = calcCanValue(28,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//燃油系统
			gRealData.diagRdyState.fuelSys = calcCanValue(29,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//综合成分
			gRealData.diagRdyState.comprehensiveComponent = calcCanValue(30,1,msg->data,HIGHTLOW_MODE,CAN_MODE);			
		}
	}
	if(source == 0 && sendCmdTimeout > 0 && gRealData.obdDiagProt == 2 && j1939Cmd[sendCmdIdx] == pgn)
	{
		sendCmdIdx++;
		sendCmdTimeout = 0;
	}
}
											
static void Udstask(void)
{
	//发动机启动，请求诊断数据
	if(g_system_dt.second != sendCmdSec)
	{
		sendCmdSec = g_system_dt.second;
		//应答超时
		if(sendCmdTimeout > 0)
		{
			sendCmdTimeout--;								//请求超时控制
			if(sendCmdTimeout == 1)					//3秒超时
			{
				if(gRealData.obdDiagProt == 2 && sendCmdIdx == 4)//J1939不支持IUPR：超时跳过
				{
					sendCmdIdx++;
				}
				else if(gRealData.obdDiagProt == 1 && (sendCmdIdx == 5 || sendCmdIdx == 6 || sendCmdIdx == 7))//ISO27145不支持此种读MIL状态 或IUPR状态：超时跳过
				{
					sendCmdIdx++;
				}
				sendCmdTimeout = 0;
			}
		}
		//其他诊断系统命令占用
		afterOtherCnt = (afterOtherCnt < 0xFF) ? (afterOtherCnt + 1) : afterOtherCnt;
		if(afterOtherCnt >= 4)
			startSecCnt = (startSecCnt < 0xFF) ? (startSecCnt + 1) : startSecCnt;//启动发动机48秒后2分30秒内
		/*/上传OBD诊断数据
		if(gUploadCan == 0xAA)
		{
			gTerminalState.obdState = 0;
			memset(getJ1939Flag,0,sizeof(getJ1939Flag));
			memset(get27145Flag,0,sizeof(get27145Flag));
			memset(get15765Flag,0,sizeof(get15765Flag));
			sendCmdIdx = 0;
			startSecCnt = 45;
			obdLog = 1;
			gUploadCan = 1;
		}*/
		if(startSecCnt >= 45 && startSecCnt <= 150 && sendCmdTimeout == 0 && gRealData.engineSpeed >= 100 && gRealData.engineSpeed < 0xFFFE && afterOtherCnt >= 4)
		{
			//测探协议顺序控制
			getObdDiagProtIdx++;
			getObdDiagProtIdx = getObdDiagProtIdx % 3;
			if((gRealData.obdDiagProt == 0 && sendCmdIdx < (sizeof(uds15765Cmd) / 8)) || (gRealData.obdDiagProt == 1 && sendCmdIdx < (sizeof(uds27145Cmd) / 8)) || (gRealData.obdDiagProt == 2 && sendCmdIdx < (sizeof(j1939Cmd) / 4)) || gRealData.obdDiagProt == 0xFF)
			{
				CAN_msg msg_buf = {0x18eafff9,{ 0x30, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,EXTENDED_TYPE,DATA_FRAME};
				osDelay(100);
				if(gRealData.obdDiagProt == 1 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 0))//ISO27145协议或未知
				{
					if(get27145Flag[sendCmdIdx] == 1)//不需要重复获取
					{
						sendCmdIdx++;
						if(sendCmdIdx >= sizeof(uds27145Cmd) / 8)
						{
							return;
						}
					}
					memcpy(msg_buf.data,uds27145Cmd[sendCmdIdx],8);
					sendCmdTimeout = 4;
				}
				if(gRealData.obdDiagProt == 0 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 1))//ISO15765协议或未知
				{
					if(get15765Flag[sendCmdIdx] == 1)//不需要重复获取
					{
						sendCmdIdx++;
						if(sendCmdIdx >= sizeof(uds15765Cmd) / 8)
						{
							return;
						}
					}
					memcpy(msg_buf.data,uds15765Cmd[sendCmdIdx],8);
					sendCmdTimeout = 4;
				}
				if(sendCmdTimeout > 0)
				{
					//扩展帧
					if(sendUdsId == 0 || sendUdsId == 0x18DA00F1 || sendUdsId == 0x18DA10F1)
					{
						msg_buf.format = EXTENDED_TYPE;	
						//功能寻址：0x18DB33F1，物理寻址：0x18DA00F1 0号ECU响应：0x18DAF100
						if(gRealData.obdDiagProt == 0 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 1))
						{
							msg_buf.id = 0x18DB33F1;
							osDelay(10);					
							CAN_send (1, &msg_buf,0x0F00);							
						}
						else
						{
							if(sendUdsId == 0 || sendUdsId == 0x18DA00F1)
							{
								msg_buf.id = 0x18DA00F1;
								osDelay(10);					
								CAN_send (1, &msg_buf,0x0F00);
							}
							if(sendUdsId == 0 || sendUdsId == 0x18DA10F1)
							{
								msg_buf.id = 0x18DA10F1;
								osDelay(10);					
								CAN_send (1, &msg_buf,0x0F00);							
							}
						}
					}
					//标准帧
					if(sendUdsId == 0 || sendUdsId == 0x7E0)
					{
						//功能寻址：0x7DF，物理寻址：0x7E0 0号ECU响应：7E8
						msg_buf.id = 0x7E0;
						msg_buf.format = STANDARD_TYPE;
						osDelay(10);
						CAN_send (1, &msg_buf,0x0F00);
					}
				}
				if(gRealData.obdDiagProt == 2 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 2))//SAE J1939协议或未知
				{
					msg_buf.data[0] = j1939Cmd[sendCmdIdx] >> 0;
					msg_buf.data[1] = j1939Cmd[sendCmdIdx] >> 8;
					msg_buf.data[2] = j1939Cmd[sendCmdIdx] >> 16;
					msg_buf.id = 0x18eafff9;
					msg_buf.format = EXTENDED_TYPE;
					osDelay(10);
					CAN_send (1, &msg_buf,0x0F00);
					sendCmdTimeout = 4;
				}
			}
			else
			{
				//OBD诊断完成
				gTerminalState.obdState = 1;
				if(gSysPara.carType != emissionLevel)
				{
					gSysPara.carType = emissionLevel;
					System_Pare_Save();
				}
			}
		}
	}
	//发动机熄火
	if(gRealData.engineSpeed <= 10 || gRealData.engineSpeed >= 0xFFFE)
	{
		scrInReady = 0;
		scrOutReady = 0;
		memset(getJ1939Flag,0,sizeof(getJ1939Flag));
		memset(get27145Flag,0,sizeof(get27145Flag));
		memset(get15765Flag,0,sizeof(get15765Flag));
		startSecCnt = 0;
		sendCmdIdx = 0;
		gTerminalState.obdState = 0;
		actAreaInjectValStamp = 0;		
		totalUreaUsedStamp1 = 0;
		totalUreaUsedStamp2 = 0;	
		engReferenceTorqueStamp = 0;
		engTotalHoursOfOperationStamp = 0;
	}
	gTerminalState.getCarData = gTerminalState.obdState;
}


