/*
文 件：analyse_Bms_SXKT_JL_EX.c
功 能：山西课题吉利通信协议 - 解析自定数据
日 期: 2023/5/24
公 司：北理新源(佛山)信息科技有限公司
作 者: HYQ
*/
#include "SXKT_JL/protocol_GB_EX_SXKT_JL.h"
#include "protocol_GB.h"
#include <stdlib.h>

/*
//强相关风险3级报警
#define STRONGLYRISK	(gRealData.tempDiffAlert == 3 || gRealData.batHighTempAlert == 3 || gRealData.batHighVolAlert == 3 || gRealData.insulationFailtAlert == 3 || gRealData.batOverCharge == 3)	
//热失控风险感知-1 极差T≥20℃且Tmax≥70℃且连续触发5帧
#define THERMALRUNWAYRISK1	(gRealData.max_singleTemper - gRealData.min_singleTemper >= 20 && gRealData.max_singleTemper > 70)	
//热失控风险感知-2 极差T≥15℃且Tmax≥60℃且充放电过程中，极差V＞200mV连续触发2帧
#define THERMALRUNWAYRISK2 	(gRealData.max_singleTemper - gRealData.min_singleTemper >= 15 && (gRealData.speed > 0 || gRealData.chargeState == STOP_CHARGE) && gRealData.max_singleVol - gRealData.min_singleVol > 0.2)
//氢系统安全分析预警 氢气气浓度>=1%，氢瓶标称压力>35MPa，气瓶过温报警>70°C，整车绝缘报警值>600欧/伏
#define HYDSYSSECURISK	(gRealData.maxHydrThickness >= 1 || gRealData.maxHydrPressure > 35 || gRealData.maxHydrSysTem > 70 || gRealData.maxHydrSysTem < -50 || gRealData.mohm > 600)

extern uint32_t gRealDataSendCnt;	//记录实时数据帧数量
//风险预警模型1
static uint32_t ariseFaultItemWarningCnt;		//出现动力电池热失控预警时的实时数据的帧数
static uint8_t FaultItemWarningStep = 0;

//风险预警模型2
static uint32_t ariseRiskearlyOutputThermalRunawayWarningCnt1;		//热失控风险感知-1出现时的实时数据的帧数
static uint32_t ariseRiskearlyOutputThermalRunawayWarningCnt2;		//热失控风险感知-2出现时的实时数据的帧数
static uint32_t ariseRiskearlyOutputThermalRunawayWarningCnt3;		//热失控风险感知-3出现时的实时数据的帧数
static uint8_t RiskearlyOutputThermalRunawayWarningStep1 = 0;	//热失控风险感知-1
static uint8_t RiskearlyOutputThermalRunawayWarningStep2 = 0;	//热失控风险感知-2
static uint8_t RiskearlyOutputThermalRunawayWarningStep3 = 0;	//热失控风险感知-3

//风险预警模型3
static uint32_t ariseHydrogenSystemWarningCnt;		//出现动力电池热失控预警时的实时数据的帧数
static uint8_t HydrogenSystemWarningStep = 0;
*/
#define RISKWARNING 1
#define RISKNOWARNING 0


const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*数据高地位*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN接收模式，分为INTEL格式和 MOTOROLA格式*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN数据偏移量，系数计算模式FACTOR_OFFSET  OFFSET_FACTOR*/

static uint32_t RiskWarningBuzzesTime = 0;
void RiskWarningBuzzes(void)	//风险预警蜂鸣
{
//		if(pSelfData81->RiskearlyOutputThermalRunawayWarning> 0 || pSelfData81->HydrogenSystemWarning > 0 || pSelfData81->FaultItemWarning > 0)
		{
			if(osKernelGetTickCount() - RiskWarningBuzzesTime > 2000)
			{
				RiskWarningBuzzesTime  = osKernelGetTickCount();
				whistle(50,50);
				osDelay(10);
				whistle(50,50);
			}
		}
}

//风险预警模型
uint8_t RiskEarlyWarningModel(void)
{

		if(pSelfData81->Risk->value != 0)
			return RISKWARNING;
		else 
			return RISKNOWARNING;
}

void unpackYZTcan(uint8_t ch,CAN_msg *msg)
{
	uint8_t RiskWarningFlag = 0;
	if(pSelfData81 == NULL)
	{
		return;
	}
	
	switch(msg->id)
	{
		case 0x18FA64D5:
		{
			//燃料电池电堆个数
			pSelfData81->FuelCellElectricPileNumber = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FA60D5:
		{
			//燃料电池系统状态
			pSelfData81->FuelCellSystemStatus = calcCanValue(0,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FA65D5:
		{
			//燃料电池电堆氢气入口压力
			pSelfData81->FuelCellStackHydrogenInletPressure = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//燃料电池空气入口流量
			pSelfData81->FuelCellAirInletFlow = calcRealValue(8,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//燃料电池输出功率
			pSelfData81->FuelCellOutputPower = calcRealValue(8,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//燃料电池系统氢气入口压力
			pSelfData81->FuelCellSystemHydrogenInletPressure = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA62D5:
		{
			//燃料电池电堆水温度
			pSelfData81->FuelCellReactorWaterTemperature = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA67D5:
		{
			//空气压缩机电压
			pSelfData81->AirCompressorVoltage = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//空气压缩机电流
			pSelfData81->AirCompressorCurrent = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//氢气循环泵电压
			pSelfData81->HydrogenCirculatingPumpVoltage = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//氢气循环泵电流
			pSelfData81->HydrogenCirculatingPumpCurrent = calcRealValue(48,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA66D5:
		{
			//PTC电压
			pSelfData81->PTC_Voltage = calcRealValue(40,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//空压机状态
			pSelfData81->AirCompressorStatus = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//氢回流泵状态
			pSelfData81->HydrogenRefluxPumpStatus = calcCanValue(8,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//水泵状态
			pSelfData81->HydrogenRefluxPumpStatus = calcCanValue(12,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//水泵电压
			pSelfData81->PumpVoltage = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//水泵电流
			pSelfData81->PumpCurrent = calcRealValue(32,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//PTC电流
			pSelfData81->PTC_Current = calcRealValue(56,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA63D5:
		{
			//高压DC/DC输出电压
			pSelfData81->HighVoltage_DCDC_OutputVoltage = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//高压DC/DC输出电流
			pSelfData81->HighVoltage_DCDC_OutputCurrent = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA61D5:
		{
			//高压DC/DC温度
			pSelfData81->HighVoltage_DCDC_OutputTemperature = calcRealValue(56,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA68D5:
		{
			//高压DC/DC输入电压
			pSelfData81->HighVoltage_DCDC_InputVoltage = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//高压DC/DC输入电流
			pSelfData81->HighVoltage_DCDC_InputCurrent = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
	}
	
	
/*第一版预警模型
	//国标3级报警综合研判预警模型
	if(STRONGLYRISK)	//强相关风险3级报警
	{
		if(FaultItemWarningStep == 0)
		{
			ariseFaultItemWarningCnt = gRealDataSendCnt;	//记录出现强故障项预警时的实时数据帧数
			FaultItemWarningStep = 1;			
		}
		else if(FaultItemWarningStep == 1)
		{
			if(gRealDataSendCnt - ariseFaultItemWarningCnt >= 3)	//连续3帧
			{
				pSelfData81->FaultItemWarning = 1;	//故障项预警
			}
		}
	}
	else
	{
		FaultItemWarningStep = 0;
		pSelfData81->FaultItemWarning = 0;	//故障项预警
	}
	
	//动力电池热失控风险预警模型
	if(THERMALRUNWAYRISK1)	//热失控风险感知-1 极差T≥20℃且Tmax≥70℃且连续触发5帧
	{
		if(RiskearlyOutputThermalRunawayWarningStep1 == 0)
		{
			ariseRiskearlyOutputThermalRunawayWarningCnt1 = gRealDataSendCnt;	//记录出现动力电池热失控预警-1时的实时数据帧数
			RiskearlyOutputThermalRunawayWarningStep1 = 1;
			RiskearlyOutputThermalRunawayWarningStep2 = 0;
			RiskearlyOutputThermalRunawayWarningStep3 = 0;
		}
		else if(RiskearlyOutputThermalRunawayWarningStep1 == 1)
		{
			if(gRealDataSendCnt - ariseRiskearlyOutputThermalRunawayWarningCnt1 >= 5)	//连续触发5帧
			{
				pSelfData81->RiskearlyOutputThermalRunawayWarning = 2;	//输出热失控风险预警
			}
		}
	}
	else if(THERMALRUNWAYRISK2)		//热失控风险感知-2 极差T≥15℃且Tmax≥60℃且充放电过程中，极差V＞200mV连续触发2帧
	{
		if(RiskearlyOutputThermalRunawayWarningStep2 == 0)
		{
			ariseRiskearlyOutputThermalRunawayWarningCnt2 = gRealDataSendCnt;	//记录出现动力电池热失控预警-2时的实时数据帧数
			RiskearlyOutputThermalRunawayWarningStep1 = 0;
			RiskearlyOutputThermalRunawayWarningStep2 = 1;
			RiskearlyOutputThermalRunawayWarningStep3 = 0;
		}
		else if(RiskearlyOutputThermalRunawayWarningStep2 == 1)
		{
			if(gRealDataSendCnt - ariseRiskearlyOutputThermalRunawayWarningCnt2 >= 2)
			{
				pSelfData81->RiskearlyOutputThermalRunawayWarning = 2;	//输出热失控风险预警
			}
		}
	}
	else if(gRealData.max_singleTemper > 65)	//热失控风险感知-3 Tmax＞65℃且温度报警持续5帧
	{
		if(RiskearlyOutputThermalRunawayWarningStep3 == 0)
		{
			ariseRiskearlyOutputThermalRunawayWarningCnt3 = gRealDataSendCnt;	//记录出现动力电池热失控预警-3时的实时数据帧数
			RiskearlyOutputThermalRunawayWarningStep1 = 0;
			RiskearlyOutputThermalRunawayWarningStep2 = 0;
			RiskearlyOutputThermalRunawayWarningStep3 = 1;
		}
		else if(RiskearlyOutputThermalRunawayWarningStep3 == 1)
		{
			if(gRealDataSendCnt - ariseRiskearlyOutputThermalRunawayWarningCnt3 >= 5)
			{
				pSelfData81->RiskearlyOutputThermalRunawayWarning = 1;	//输出热失控风险预警
			}
		}
	}
	else
	{
		RiskearlyOutputThermalRunawayWarningStep1 = 0;
		RiskearlyOutputThermalRunawayWarningStep2 = 0;
		RiskearlyOutputThermalRunawayWarningStep3 = 0;
		pSelfData81->RiskearlyOutputThermalRunawayWarning = 0;	//输出热失控风险预警
	}
	
	//氢系统安全分析预警模型
	if(HYDSYSSECURISK)
	{
		pSelfData81->HydrogenSystemWarning = 1;	//氢系统安全预警
	}
	else
	{
		pSelfData81->HydrogenSystemWarning = 0;	//氢系统安全预警
	}
*/
	//风险预警模型
	RiskWarningFlag = RiskEarlyWarningModel();
	if(RiskWarningFlag == RISKWARNING)
		RiskWarningBuzzes();//蜂鸣
	
}
