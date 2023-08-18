/*
文 件：analyse_Bms_NJJL_DK_EX.c
功 能：南京金龙通信协议 - 解析自定数据
日 期: 2021/12/9
公 司：北理新源(佛山)信息科技有限公司
作 者: CZJ -> LGC
*/

#include "protocol_GB.h"
#include "NJJL_DK/protocol_GB_EX_NJJL_DK.h"
#include <stdlib.h>

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/
const static uint8_t SEND_CAN_MIN_INTERVAL = 30;			//CAN发送最小间隔
	
static void sendCanData(uint8_t ch);													//发送CAN接口
static void sendLockCMD(uint8_t ch,CAN_msg *msg);										//发送远程/心跳锁  指令
static void sendIntervalData(unsigned char canCh);										//发送周期数据
static void saveCANData(uint8_t ch,uint32_t canId,uint8_t *canData);					
static uint32_t saveInterl = 0;

static CAN_msg msg_buf = {0,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,EXTENDED_TYPE,DATA_FRAME};
static uint32_t minInterl = 0;
static uint8_t isSendCan = 0;

uint8_t authResultNum;
uint8_t authResult;

void unpackYZTcan(uint8_t ch,CAN_msg *msg)
{
	uint8_t n8Val;
	uint8_t AllCarState = 0;
	if(pSelfData80 == NULL || pSelfData81 == NULL)
	{
		return;
	}
	switch(msg->id)
	{
		case 0x18F101D0:
			{
				//自定义80数据 - 车辆运行模式
				n8Val= calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val)
					pSelfData80->sRunModel = EV;											//1 纯电
				else if(2 == n8Val)	
					pSelfData80->sRunModel = EV_FV;									//2 混动
				else if(3 == n8Val)
					pSelfData80->sRunModel = FV;											//3 燃油
				//自定义80数据 - 车辆状态1
				pSelfData80->sCarState1 = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义80数据 - 车辆故障状态1
				pSelfData80->sCarFaultState1 = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义80数据 - VCU故障
				pSelfData80->sVCUFault = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义80数据 - LIFE信号
				pSelfData80->sLifeSign = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义80数据 - 整车状态
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				AllCarState |= n8Val << 0;
				//自定义80数据 - 整车状态		【】
				n8Val = calcCanValue(4,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				AllCarState |= n8Val << 4;
				//自定义80数据 - 整车状态
				n8Val = calcCanValue(1,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				AllCarState |= n8Val << 6;
			}
			break;
		case 0x18F503F0:
			{
				//自定义80数据 - 相电流
				pSelfData80->sPhaseCurr = calcCanValue(16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F40217:
			{
				//累计里程
				gRealData.totalMileage = calcRealValue(0,32,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//自定义83数据 - 总里程
				pSelfData83->sAllMileage = calcCanValue(0,32,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义83数据 - 仪表程序版本
				pSelfData83->sInstrumentSoftwareVersion = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
			break;
		case 0x18F201F3:
			{
				//自定义81数据 - 电池组总串数
				pSelfData81->sBatsCount = calcCanValue(40,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - BMS life信号
				pSelfData81->sBMSLifeSignal = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F103D0:
			{
				//自定义80数据 - 续航里程
				pSelfData80->sRunMileage = calcRealValue(8,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义80数据 - 百公里平均电耗
				pSelfData80->sPowerAvgConsumption = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义80数据 - VCU扭矩请求
				pSelfData80->sVCUTorsion = calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义80数据 - VCU版本信息
				pSelfData80->sVCUVersionInfo = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F104D0:
			{
				//自定义80数据 - 整车加速度
				pSelfData80->sCarAcceleratedSpeed = calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F105D0:
			{
				//自定义80数据 - 高压附件使能状态	
				uint8_t TempVal;
				TempVal = 0;
				n8Val = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
					TempVal |= 1<<0;	
				n8Val = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
					TempVal |= 1<<1;
				n8Val = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
					TempVal |= 1<<2;				
				n8Val = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
					TempVal |= 1<<3;
				pSelfData80->sHighDevState = TempVal;
			}
			break;
		case 0x18F106D0:
			{
				//自定义80数据 - VCU接触器控制指令	
				pSelfData80->sVCUTouchControlCmd = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义80数据 - VCU接触器闭合状态	
				pSelfData80->sVCUTouchCloseState = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FF2B49:		//【MOTOROLA格式】
			{
				//自定义81数据 - 绝缘监测报警状态		 
				n8Val = calcCanValue(4,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 2)
					pSelfData81->sInsulationAlarmState = 3;
				else if(n8Val == 1)
					pSelfData81->sInsulationAlarmState = 2;
				else if(n8Val == 0)
					pSelfData81->sInsulationAlarmState = 0;
				//自定义81数据 - 绝缘检测电池总压
				pSelfData81->sInsulationCheckAllVolt = calcRealValue(32,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,MOTOROLA);			
			}
			break;
		case 0x18F214F3:
			{
				//自定义81数据 - 充电抢1正温度
				pSelfData81->sCHGPlusesTem1 = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,MOTOROLA);			
				//自定义81数据 - 充电抢1正温度
				pSelfData81->sCHGMinusTem1 = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,MOTOROLA);			
				//自定义81数据 - 充电抢1正温度
				pSelfData81->sCHGPlusesTem2 = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,MOTOROLA);			
				//自定义81数据 - 充电抢1正温度
				pSelfData81->sCHGMinusTem2 = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,MOTOROLA);			
			}
			break;
		case 0x18F501F0:
			{
				//自定义80数据 - MCU故障			
				pSelfData80->sMCUFault = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义80数据 - 整车状态
				n8Val = calcCanValue(42,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val < 4)
					AllCarState |= n8Val << 2;
			}
			break;
		case 0x18F205F3:
			{
				//温度差异报警								
				n8Val = calcCanValue(16,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 温度差异报警1
				pSelfData81->sTemDiffAlarm1 = n8Val;
				//电池高温报警						
				n8Val = calcCanValue(6,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 电池高温报警1
				pSelfData81->sBatHighTemAlarm1 = n8Val;
				//车载储能装置类型过压报警					
				n8Val = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 电池组过压报警
				pSelfData81->sBatsOverVlotAlarm = n8Val;
				//车载储能装置类型欠压报警					
				n8Val = calcCanValue(14,2,msg->data,HIGHTLOW_MODE,CAN_MODE);						
				//自定义81数据 - 电池组欠压报警
				pSelfData81->sBatsLowVlotAlarm = n8Val;
				//单体电池过压报警
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 单体过压报警
				pSelfData81->sSingleOverVlotAlarm = n8Val;
				//单体电池欠压报警					 
				n8Val = calcCanValue(12,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 单体欠压报警
				pSelfData81->sSingleLowVlotAlarm = n8Val;				
				//电池单体一致性差报警				 
				n8Val = calcCanValue(18,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//自定义81数据 - 单体电压差异报警
				pSelfData81->sSingleDifVlotAlarm = n8Val;
				//自定义81数据 - SOC低报警1
				pSelfData81->sSOCLowAlarm1 = calcCanValue(30,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - SOC高报警
				pSelfData81->sSOCHighAlarm = calcCanValue(28,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 充电插头连接状态
				n8Val= calcCanValue(51,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				pSelfData81->sCHGLinkState  = (n8Val == 1) ? 0x08:0x00;
				//自定义81数据 - 电池冷却状态
				n8Val= calcCanValue(50,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				pSelfData81->sBatCoolState  = (n8Val == 1) ? 0x04:0x00;
				//自定义81数据 - 加热状态
				n8Val= calcCanValue(49,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				pSelfData81->sHeatState  = (n8Val == 1) ? 0x02:0x00;
				//自定义81数据 - 均衡状态
				n8Val= calcCanValue(48,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				pSelfData81->sBalanceState  = (n8Val == 1) ? 0x02:0x00;	
				//自定义81数据 - BMS故障等级(动力电池故障状态)
				pSelfData81->sBMSFaultLevel = calcCanValue(8,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 极柱高温报警
				pSelfData81->sPoleColumnHighTemAlarm = calcCanValue(4,2,msg->data,HIGHTLOW_MODE,CAN_MODE);							
				//自定义81数据 - 充电枪高温报警
				pSelfData81->sCHGGunHighTemAlarm = calcCanValue(10,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 放电电流报警
				pSelfData81->sOutCurrentAlarm = calcCanValue(22,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 充电电流报警
				pSelfData81->sCHGCurrentAlarm = calcCanValue(20,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - SOC差异报警
				pSelfData81->sSOCDiffenceAlarm = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 电池低温报警
				pSelfData81->sBatLowTemAlarm = calcCanValue(24,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - BMS通讯故障（针对can硬件故障）
				pSelfData81->sBMSCommunicationFault = calcCanValue(47,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 电池系统其他故障
				pSelfData81->sBatSystemOtherFault = calcCanValue(46,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 电池冷却系统故障
				pSelfData81->sBatCoolSystemFault = calcCanValue(45,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 加热故障报警状态
				pSelfData81->sHeatFaultAlarmState = calcCanValue(44,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 均衡报警状态
				pSelfData81->sBalanceAlarmState = calcCanValue(43,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 预充电报警
				pSelfData81->sPreCHGAlarm = calcCanValue(42,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 与充电机通信报警
				pSelfData81->sCHGDevInfoAlarm = calcCanValue(41,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - BMS从控掉线报警（针对某箱数据丢失）
				pSelfData81->sBMSControlOffLinAlarm = calcCanValue(40,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 接触器粘连故障
				pSelfData81->sToucherLinkFault = calcCanValue(63,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 支路压差报警
				pSelfData81->sBranchPreAlarm = calcCanValue(62,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 火灾极限故障报警
				pSelfData81->sFireLimitFaultAlarm = calcCanValue(61,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 电池放电保护（用于持续小于10A放电2小时切断总负）
				pSelfData81->sBatProtectOut = calcCanValue(58,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 单体电压采集掉线状态
				pSelfData81->sSingleBatOffLinState = calcCanValue(57,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 温度采集掉线状态
				pSelfData81->sSingleTemOffLinState = calcCanValue(56,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 动力电池故障码(BMS故障码)
				pSelfData81->sBatFaultNum = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F202F3:
			{
				//自定义81数据 - BMS异常状态下请求切断高压
				pSelfData81->sBMSFaultShutHighPower = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 最大可用持续充电电流（5min）
				pSelfData81->sMaxLongInCurrent = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 最大可用短时充电电流（30s）
				pSelfData81->sMaxShortInCurrent = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 最大可用持续放电电流（5min）
				pSelfData81->sMaxLongOutCurrent = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 最大可用短时放电电流（30s）
				pSelfData81->sMaxShortOutCurrent = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - BMS接触器控制命令
				pSelfData81->sBMSTouchControlCMD = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - BMS接触器闭合状态
				pSelfData81->sBMSTouchControlCloseState = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
		case 0x18F20CF3:
			{
				//自定义81数据 - 充电次数
				pSelfData81->sCHGCounts = calcRealValue(24,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 电池组生产日期（月）
				pSelfData81->sBatsProductDate_Month = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 电池组生产日期（年）
				pSelfData81->sBatsProductDate_Year = calcRealValue(48,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 动力电池生产厂家
				pSelfData81->sBatsProducer = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - BMS程序版本
				pSelfData81->sBMSSoftwareVersion = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F222F3:
			{
				//自定义81数据 - 电池组累计输出能量
				pSelfData81->sBMSAddUpOutPower = calcRealValue(0,24,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义81数据 - 电池组累计充电（不含制动回馈）能量
				pSelfData81->sBMSAddUpChgPower = calcRealValue(24,24,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x0CF602A1:
			{
				//DCDC状态	
				n8Val = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val  )
				{
					pSelfData82->sDCDCWorkState = DCDC_WORK;		//【DCDC工作状态
				}
				else if(0 == n8Val)
				{
					pSelfData82->sDCDCWorkState = DCDC_BREAK;	//【DCDC工作状态	
				}
				//自定义82数据 - DC/DC实时输出电流
				pSelfData82->sDCDC_RealTimeOutCur = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - DC/DC本体温度
				pSelfData82->sDCDCTem = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - DCDC Life信号
				pSelfData82->sDCDCLifeSignal = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;

		case 0x185F17F3:
			{
				//自定义81数据 - 电池组温度点数
				pSelfData81->sBatsTemsCount = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x0CF606A6:
			{
				//自定义82数据 - 4合1状态
				pSelfData82->sFourInOne_State = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义82数据 - 4合1故
				pSelfData82->sFourInOne_FaultNum = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义82数据 - 4合1 - BMS接触器状态反馈
				pSelfData82->sFourInOne_BMSToucherState = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//自定义82数据 - 4合1 - VCU接触器状态反馈
				pSelfData82->sFourInOne_VCUToucherState = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义82数据 - 4合1 - BMS接触器故障状态
				pSelfData82->sFourInOne_BMSToucherFaultState = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义82数据 - 4合1 - VCU接触器故障状态
				pSelfData82->sFourInOne_VCUToucherFaultState = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x0CF601A0:
			{
				//自定义82数据 - 高压油泵输出电压		
				pSelfData82->sHighOilPump_OutVolt = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 高压油泵DC/AC输出电流	
				pSelfData82->sHighOilPump_DCACOutCur = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 高压油泵电机温度
				pSelfData82->sHighOilPump_MoterTem = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 高压油泵DC/AC状态及故障
				pSelfData82->sHighOilPump_DCACStateAndFault = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 高压油泵变频器故障码
				pSelfData82->sHighOilPump_ConverterFaultNum = calcRealValue(48,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 高压油泵转速
				pSelfData82->sHighOilPump_motorSpeed = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 高压油泵DC/AC life信号
				pSelfData82->sHighOilPump_DCACLifeSignal = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
			break;
		case 0x0CF603A2:
			{
				//自定义82数据 - 气泵DC/AC输出电压
				pSelfData82->sAirPump_DCACOutVolt = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 气泵DC/AC输出电流
				pSelfData82->sAirPump_DCACOutCur = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 气泵DC/AC状态及故障
				pSelfData82->sAirPump_DCACStateAndFault = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 气泵温度
				pSelfData82->sAirPump_Tem = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 气泵变频器故障码
				pSelfData82->sAirPump_ConverterFaultNum = calcRealValue(48,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 气泵转速
				pSelfData82->sAirPump_motorSpeed = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 气泵DC/AC Life信号	
				pSelfData82->sAirPump_DCACLifeSignal = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
			break;
		case 0x0CF605A5:
			{
				//自定义82数据 - 低压油泵输出电压	
				pSelfData82->sLowOilPump_OutVolt = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 低压油泵DC/AC输出电流	
				pSelfData82->sLowOilPump_DCACOutCur = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 低压油泵DC/AC状态及故障	
				pSelfData82->sLowOilPump_DCACStateAndFault = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 低压油泵变频器故障码
				pSelfData82->sLowOilPump_ConverterFaultNum = calcRealValue(48,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 低压油泵转速
				pSelfData82->sLowOilPump_motorSpeed = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义82数据 - 低压油泵DC/AC life信号
				pSelfData82->sLowOilPump_DCACLifeSignal = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
			break;
		case 0x18F40117:
			{
				//自定义83数据 - 低压电池电压
				pSelfData83->sLowBatVolt = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义83数据 - 前制动储气筒气压
				pSelfData83->sFrontBrakeAirPressure = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义83数据 - 后制动储气筒气压
				pSelfData83->sRearBrakeAirPressure = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义83数据 - 车辆状态1
				pSelfData83->sCarState1 = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义83数据 - 车辆状态2
				pSelfData83->sCarState2 = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义83数据 - 车辆状态3
				pSelfData83->sCarState3 = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义83数据 - 仪表报警状态1
				pSelfData83->sInstrumentAlarmState1 = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义83数据 - 仪表报警状态2
				pSelfData83->sInstrumentAlarmState2 = calcRealValue(48,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
			break;
		case 0x18F20EF3:
			{
				//自定义84数据 - BMS冷却请求工作模式
				pSelfData84->sBMSCoolWorkMode = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义84数据 - BMS水冷机组出水口（电池入水口）设定温度
				pSelfData84->sBMSsetoutWaterTem = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义84数据 - 电池组最高温度
				pSelfData84->sBatsHighestTem = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义84数据 - 电池组最低温度
				pSelfData84->sBatsLowestTem = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//自定义84数据 - 电池组请求life值
				pSelfData84->sBatsQueLifeValue = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x0CF609A9:
			{
				//自定义84数据 - 热管理系统工作模式汇总
				pSelfData84->sHotContrlMode = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义84数据 - 进水温度
				pSelfData84->sInWaterTem = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义84数据 - 出水温度
				pSelfData84->sOutWaterTem = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义84数据 - 压缩机负载比率
				pSelfData84->sCompressorPower = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义84数据 - 热管理系统故障码
				pSelfData84->sHotContrlFaultNum = calcRealValue(48,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义84数据 - 热管理工作life值
				pSelfData84->sHotContrlLifeValue = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x0CF601A7:
			{
				//自定义85数据 - 空调开关机命令/状态
				pSelfData85->sAirConditionerOpenCMD = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 空调设定温度
				pSelfData85->sAirConditionerSetTem = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 空调风机运行档位
				pSelfData85->sAirConditionerRunStall = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 车内环境温度
				pSelfData85->sInCarTem = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 空调整机运行模式
				pSelfData85->sAirConditionerRunMode = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 车外环境温度
				pSelfData85->sOutCarTem = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 空调接触器控制以及状态
				pSelfData85->sAirToucherContrlAndState = calcRealValue(48,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 空调life
				pSelfData85->sAirLife = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x0CF603A7:
			{
				//自定义85数据 - 空调系统母线电压
				pSelfData85->sAirSystemVolt = calcRealValue(32,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 空调系统一部件运行状态
				pSelfData85->sAirSystem_PartsRunState = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 空调系统一系统运行状态
				pSelfData85->sAirSystem_SystemRunState = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 压缩机目标频率
				pSelfData85->sAirRunTargetHz = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 压缩机运行频率
				pSelfData85->sAirRunHz = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义85数据 - 空调故障码
				pSelfData85->sAirFaultNum = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
			}
			break;
		case 0x18FEF433:
			{
				//自定义86数据 - 轮胎位置
				pSelfData86->sTirePosition = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//自定义86数据 - 轮胎压力
				pSelfData86->sTirePressure = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//自定义86数据 - 轮胎温度
				pSelfData86->sTireTem = calcRealValue(16,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//自定义86数据 - 状态（轮胎状态）
				pSelfData86->sTireState = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//自定义86数据 - 压力阀检测
				pSelfData86->sPressureValveCheck = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
			}
			break;
		case 0x18F701F5:
			{
				//自定义87数据 - 灭火器系统状态
				pSelfData87->sFlameArrester_SystemState = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//自定义87数据 -电池组号
				pSelfData87->sBatsNum = calcRealValue(8,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//自定义87数据 - 箱体内传感器工作状态
				pSelfData87->sSensorBoxState = calcRealValue(16,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//自定义87数据 - 箱体内故障状态等级
				pSelfData87->sSensorBoxFault = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//自定义87数据 - 箱体内温度
				pSelfData87->sSensorBoxTem = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//自定义87数据 - 箱体内灭火器启动状态
				pSelfData87->sSensorBoxStartState = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//自定义87数据 - LIFE
				pSelfData87->sSensorBoxLife = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
			}
			break;
		case 0x18FF09E9:
			{
				//自定义88数据 - 前车碰撞警告
				pSelfData88->sAheadCarWarning = calcCanValue(9,1,msg->data,HIGHTLOW_MODE,MOTOROLA);		
				//自定义88数据 - 前车距离
				pSelfData88->sAheadCarDistance = calcRealValue(12,10,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,MOTOROLA);		
				//自定义88数据 - 车道偏离警告
				pSelfData88->sLaneWarning = calcCanValue(35,1,msg->data,HIGHTLOW_MODE,MOTOROLA);		
				//自定义88数据 - 车道偏离方向
				pSelfData88->sLaneDirection = calcCanValue(36,1,msg->data,HIGHTLOW_MODE,MOTOROLA);	
			}
			break;
		case 0x18FF0EE9:
			{
				//自定义88数据 - 相对速度
				pSelfData88->sAheadCar_RelativeSpeed = calcRealValue(7,12,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,MOTOROLA);		
			}
			break;
		case 0x0CF02F2A:
			{
				//自定义88数据 - 启动碰撞预警功能
				pSelfData88->sActivateCollisionWarning = calcCanValue(4,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义88数据 - 启动紧急制动功能
				pSelfData88->sActivateEmergencyBraking = calcCanValue(4,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//自定义88数据 - AEB系统状态
				pSelfData88->sABESystemState = calcRealValue(0,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
			}
			break;
	}
	//自定义80数据 - 整车状态
	pSelfData80->sCarState = AllCarState;
	sendIntervalData(ch);
//	saveCANData(ch,msg->id,msg->data);
}

/*******************************************************************************
*	函 数 名: SeedtoKey
*	功能说明: 安全进入算法（TBD）
*	形    参:  
*	返 回 值: 无
*******************************************************************************/
static const uint32_t seedMask = 0x7543C1A1;
static uint8_t SeedtoKey (uint32_t seed, uint32_t mask)
{
    uint8_t i = 0 ;
    uint8_t key = 0 ;
    uint8_t A = 0 ;
    uint8_t B = 0 ;
    uint8_t C = 0 ;
    if ( seed != 0 )
    {
        for (i=0; i<4; i++)
        {
            A = (uint8_t)(seed >> 8*i & 255U);
            B = (uint8_t)(mask >> 8*i & 255U);
            C = (uint8_t)(A + B);
            key = (uint8_t)(C+key);
        }
        return key;
    }
	return 0;
}

static uint32_t upFirstSendLockTime = 0;				//升级后第一次发锁车指令时间计时

static uint32_t startTime = 0;								//启动时间

/*******************************************************************************
*	函 数 名: sendLockCMD
*	功能说明: 终端  发送远程(心跳)锁车/解锁指令		发送5次，每次间隔500ms
*	形    参: canCh :CAN 通道
*						canid	:CAN ID 		0x1864A0F0(远程)		0x1866A0F0(心跳)
*						cmd		:远程锁/心跳锁，指令
*			 			 canCh	: 	BSP_CAN1 or BSP_CAN2 or BSP_CAN3
*	返 回 值: 无
*******************************************************************************/

static void sendLockCMD(uint8_t ch,CAN_msg *msg)
{
	static uint32_t sendLockCmdTime = 0;
	uint8_t n8Val = 0;
	uint32_t n32Val = 0;
	switch(msg->id)
	{
		case 0x1862F0A0:																			//VCU 反馈锁车状态																																											
		{
			gRemoteLockPara.remoteLockState = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);						
			gRemoteLockPara.heartLockState = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);							
		}
		break;
		case 0x1865F0A0:																			//VCU应答远程锁车指令																																							
		{				
			n8Val = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);									
			if(n8Val==1 || n8Val == 2 || n8Val == 4)
			{
				gRemoteLockPara.carDoorCtrRspCode = 1;					
			}
			else
			{
				gRemoteLockPara.carDoorCtrRspCode = 0;
			}
			gRemoteLockPara.isReturnLockState = 0xA5;	
		}
		break;
		case 0x1867F0A0:																			//VCU应答心跳锁车指令
		{
			n8Val= calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);									
			if(n8Val==1 || n8Val == 4)
			{
				gRemoteLockPara.carDoorCtrRspCode = 1;
			}
			else
			{
				gRemoteLockPara.carDoorCtrRspCode = 0;
			}
			gRemoteLockPara.isReturnLockState = 0xA5;	
		}
		break;
					//锁车认证反馈
		case 0x18F104D0:		//整车状态信息
			{
				//认证结果
				authResult = calcCanValue(40,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//锁车状态
				gRemoteLockPara.remoteLockState = calcCanValue(44,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18F110D0:		//整车认证随机数
			{
				n32Val = calcCanValue(0,32,msg->data,HIGHTLOW_MODE,CAN_MODE);
				authResultNum = SeedtoKey(n32Val,seedMask);
			}
			break;
	}
	if(upFirstSendLockTime >= 600 && upFirstSendLockTime <= 2400 && gRemoteLockPara.isHeartLock == 1)											//每次升级更新程序，就发5次心跳锁车
	{
		gRemoteLockPara.isHeartLock = 0;
		System_Pare_Save();
		gRemoteLockPara.carDoorCtrCode = 2;																//心跳锁指令
		sendLockCMDSign = 5;
		upFirstSendLockTime = 2500;
	}
	//计时
	if(osKernelGetTickCount() - startTime >= 500)
	{
		startTime = osKernelGetTickCount();	
		upFirstSendLockTime++;																			//升级后第一次发送锁车指令的计时
		if(upFirstSendLockTime > 240)
			upFirstSendLockTime = 250;
	}
	if(sendLockCMDSign > 0)
	{
		if(osKernelGetTickCount() -  sendLockCmdTime >= 500)
		{
			memset(msg_buf.data,0,sizeof(msg_buf.data));
			switch(gRemoteLockPara.carDoorCtrCode)
			{
				case 0: msg_buf.id = 0x1864A0F0; msg_buf.data[0] = 4; break;				//远程 - 解锁
				case 1: msg_buf.id = 0x1866A0F0; msg_buf.data[0] = 4; break;				//心跳 - 解锁
				case 2: msg_buf.id = 0x1866A0F0; msg_buf.data[0] = 1; break;				//心跳 - 心跳锁
				case 3: msg_buf.id = 0x1864A0F0; msg_buf.data[0] = 1; break;				//远程 - 一级锁
				case 4: msg_buf.id = 0x1864A0F0; msg_buf.data[0] = 2; break;				//远程 - 二级锁
			}
			msg_buf.data[2] = rand() % 256;	
			msg_buf.data[7] = getBccCode(msg_buf.data,0,7);
			sendLockCMDSign--;
			isSendCan = 1;
			sendCanData(ch);
		}
	}
}


/*******************************************************************************
*	函 数 名: SelfFunction
*	功能说明: 轻客自定义功能 入口函数
*	形    参:  

*	返 回 值: 无
*******************************************************************************/
void sendIntervalData(unsigned char canCh)
{
	static uint32_t sendStaTime,sendDevTimTime = 0,sendHeartBeatTime = 0,sendConnectTime = 0,sendVinTime = 0;
	if(osKernelGetTickCount() - minInterl >= SEND_CAN_MIN_INTERVAL)//发送锁车心跳信息 		0x1860A0F0			500ms 执行一次			
	{
		if(osKernelGetTickCount() -  sendHeartBeatTime >= 500)
		{
			static uint8_t heartNum;
			uint32_t termID;			
			sendHeartBeatTime = osKernelGetTickCount();
			srand(sendHeartBeatTime);
			msg_buf.data[0] = 0xF0;													
			msg_buf.data[1] = heartNum++;										
			msg_buf.data[2] = rand() % 256;												
			sscanf(&gFrimPara.terminalId[6],"%u",&termID);
			msg_buf.data[3] = (uint8_t)(termID>>24);
			msg_buf.data[4] = (uint8_t)(termID>>16);
			msg_buf.data[5] = (uint8_t)(termID>>8);
			msg_buf.data[6] = (uint8_t)(termID>>0);
			msg_buf.data[7] = getBccCode(msg_buf.data,0,7);
			msg_buf.id = 0x1860A0F0;
			isSendCan = 1;
		}
		else if(osKernelGetTickCount() -  sendConnectTime >= 500)//发送保持连接信息			0x1861A0F0		0xFF 不带锁车功能，0xEE 带锁车功能	
		{
			static uint8_t heartNum;
			uint32_t termID;												
			sendConnectTime = osKernelGetTickCount();
			srand(sendConnectTime);
//			if(gTerminalState.gpsState == 0)					//未检测到GPS天线，生命值发送0，实现防拆
//			{
//				heartNum = 0;						
//			}
			msg_buf.data[0] = 0xEE;													
			msg_buf.data[1] = heartNum++;										
			msg_buf.data[2] = rand() % 256;								//随机密钥
			sscanf(&gFrimPara.terminalId[6],"%u",&termID);
			msg_buf.data[3] = (uint8_t)(termID>>24);
			msg_buf.data[4] = (uint8_t)(termID>>16);
			msg_buf.data[5] = (uint8_t)(termID>>8);
			msg_buf.data[6] = (uint8_t)(termID>>0);
			msg_buf.data[7] = getBccCode(msg_buf.data,0,7);
			msg_buf.id = 0x1861A0F0;
			isSendCan = 1;
		}
		else if(osKernelGetTickCount() -  sendVinTime >= 500)//发送VIN到CAN总线
		{
			static uint8_t sendVINSeril = 0;
			sendVinTime = osKernelGetTickCount();
			sendVINSeril = (sendVINSeril + 1) % 3;
			switch(sendVINSeril)
			{
				case 0:
					memcpy(msg_buf.data,&gSysPara.vinCode[0],8);
					msg_buf.id = 0x18F301F8;
					break;
				case 1:
					memcpy(msg_buf.data,&gSysPara.vinCode[8],8);
					msg_buf.id = 0x18F302F8;
					break;
				case 2:
					msg_buf.data[0] = gSysPara.vinCode[16];
					msg_buf.id = 0x18F303F8;
					break;
			}
			isSendCan = 1;
		}
		else if(osKernelGetTickCount() -  sendStaTime >= 200)//自身状态参数						0x18F304F8
		{
			uint8_t temp = 0;
			static uint8_t lifeCount = 0;
			sendStaTime = osKernelGetTickCount();
			msg_buf.data[0] = 1;																						//远程终端运行状态
//			temp |=  (can_State == 0 ? 1:0) << 0;														//终端CAN通信掉线故障
//			temp |=  (gTerminalState.simCardState == 0 ? 1:0) << 1;					//GPRS故障
//			temp |=  (gTerminalState.gpsState != 1 ? 1:0) << 2;							//GPS 故障
//			temp |=  (gTerminalState.sdState == 0 ? 1:0) << 3;							//SD卡故障
			//temp |=  (0 > 0 ? 1:0) << 4;																	//其他故障
			//temp |=  (0 > 0 ? 1:0) << 5;																	//里程超差
			msg_buf.data[1] = temp;
			msg_buf.data[2] = 0x02;
			msg_buf.data[3] = authResultNum;																//认证反馈数
			msg_buf.data[6] = 0x0A;																					//版本1
			msg_buf.data[7] = lifeCount++;
			msg_buf.id = 0x18F304F8;
			isSendCan = 1;
		}
		else if(osKernelGetTickCount() -  sendDevTimTime >= 200)//终端时间								0x18FEE6F8
		{
			sendDevTimTime = osKernelGetTickCount();
			uint8_t index = 0;
			msg_buf.data[index++] = g_system_dt.second;												//秒
			msg_buf.data[index++] = g_system_dt.minute;												//分
			msg_buf.data[index++] = g_system_dt.hour;													//时
			msg_buf.data[index++] = g_system_dt.day;													//日
			msg_buf.data[index++] = g_system_dt.month;												//月
			msg_buf.data[index++] = (uint8_t)(g_system_dt.year - 1985);				//年（偏移+1985，2000-1985 = 15）
			msg_buf.id = 0x18FEE6F8;
			isSendCan = 1;
		}
	}
	sendCanData(canCh);
}

static void sendCanData(uint8_t ch)
{
	if(isSendCan == 1)
	{
		isSendCan = 0;
		minInterl = osKernelGetTickCount();
		CAN_send(ch,&msg_buf,100);//无动作
	}
}

#define SDFREE_SPACE (60*1024*1024)			//SD容量最小 空间 60M

char saveBuff[6144] = {0};
extern uint32_t chanefileTime;						//切换文件时间
uint8_t startSave = 0;										//每次休眠启动后重置
uint32_t tempTime = 0;										//与起始时间相差时间戳
static uint16_t dIndex = 0;								//存储区下标ID
static void saveCANData(uint8_t ch,uint32_t canId,uint8_t *canData)
{
//	if(gTerminalState.sdState > 0 && gTerminalState.sdFreeCap > SDFREE_SPACE)			//SD卡有插入且SD卡剩余容量大于60M
	{
		if(startSave == 0)					//第一次启动或者时间变更
		{
			startSave = 1;
			chanefileTime = osKernelGetTickCount();							//记录起始时间戳
			memset(saveBuff,0,sizeof(saveBuff));
			dIndex = 0;					
		}
	
		tempTime = osKernelGetTickCount() - chanefileTime;		//更新时间戳差值
		dIndex = strlen((char*)saveBuff);
		sprintf((char*)&saveBuff[dIndex],"%d.%d %x %02x%02x%02x%02x%02x%02x%02x%02x\n",tempTime/1000,tempTime%1000,canId,canData[0],canData[1],canData[2],canData[3],canData[4],canData[5],canData[6],canData[7]);
		if(dIndex + 120 > sizeof(saveBuff))
		{
//			saveHistory(11,1,(uint8_t*)saveBuff,dIndex,0,LOG);
			dIndex = 0;
			memset(saveBuff,0,sizeof(saveBuff));
		}
	}
}


#if 0
static void saveCANData(uint8_t ch,uint32_t canId,uint8_t *canData)
{
	if(startSave == 0)								//初始启动
	{	
		chanefileTime = osKernelGetTickCount();
		memset(saveBuff,0,sizeof(saveBuff));
		dIndex = 0;
		startSave = 1;
	}

	tempTime = osKernelGetTickCount() - chanefileTime;
	saveBuff[dIndex++] = (uint8_t)(tempTime>>24);
	saveBuff[dIndex++] = (uint8_t)(tempTime>>16);
	saveBuff[dIndex++] = (uint8_t)(tempTime>>8);
	saveBuff[dIndex++] = (uint8_t)(tempTime>>0);
	
	saveBuff[dIndex++] = (uint8_t)(canId>>24);
	saveBuff[dIndex++] = (uint8_t)(canId>>16);
	saveBuff[dIndex++] = (uint8_t)(canId>>8);
	saveBuff[dIndex++] = (uint8_t)(canId>>0);

	saveBuff[dIndex++] = canData[0];
	saveBuff[dIndex++] = canData[1];
	saveBuff[dIndex++] = canData[2];
	saveBuff[dIndex++] = canData[3];
	saveBuff[dIndex++] = canData[4];
	saveBuff[dIndex++] = canData[5];
	saveBuff[dIndex++] = canData[6];
	saveBuff[dIndex++] = canData[7];
	
	if(dIndex + 64 > sizeof(saveBuff))
	{
		saveHistory(11,1,(uint8_t*)saveBuff,dIndex,0,LOG);
		dIndex = 0;
		memset(saveBuff,0,sizeof(saveBuff));
	}
}
#endif










