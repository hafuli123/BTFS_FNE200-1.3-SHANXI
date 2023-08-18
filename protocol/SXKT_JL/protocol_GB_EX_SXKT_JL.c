/*
文 件：protocol_GB_EX_SXKT_JL.c
功 能：山西课题吉利风险预警通信协议 - 解析上送自定义扩展数据
日 期: 2023/5/24
公 司：北理新源(佛山)信息科技有限公司
作 者: HYQ
*/

#include "SXKT_JL/protocol_GB_EX_SXKT_JL.h"


SelfData81* pSelfData81;
SelfData81*  p81Data;
static uint16_t p81offset;

typedef struct _GBSTA
{
	//外部控制参数
	uint8_t bLink;									//链路号
	uint8_t bUse;										//是否使用
	char *vin;											//车架号
	uint8_t sendOverTimeCnt;				//发送次数
	//内部时间戳
	uint32_t lockStaStamp;					//锁状态时间戳
	//其他参数
	uint8_t* buff;									//发送数据缓冲区
	uint16_t buffLen;								//发送数据缓冲区长度
}GBSTA;


#define MAX_YZT_LINK 1

static GBSTA gbSta[MAX_YZT_LINK] = {0};

//扩展协议初始化，返回配置状态参数
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	for(i = 0;i < MAX_YZT_LINK;i++)
	{
		if(gbSta[i].bUse == 1 && gbSta[i].bLink == link)
		{
			oldLinkIdx = i;//重新初始化
		}
		if(gbSta[i].bUse == 0 && objLinkIdx == 0xFF)
		{
			objLinkIdx = i;
		}
	}
	if(oldLinkIdx != 0xFF)
	{
		objLinkIdx = oldLinkIdx;
	}
	if(objLinkIdx == 0xFF)
		return NULL;
	//外部参数
	gbSta[objLinkIdx].bUse = 1;
	gbSta[objLinkIdx].bLink = link;
	gbSta[objLinkIdx].vin = vin;
	gbSta[objLinkIdx].buff = buff;
	gbSta[objLinkIdx].buffLen = buffLen;
	//内部
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
	gbSta[objLinkIdx].lockStaStamp = 0;
	//自定义数据分配内存，设置偏移量
	
	p81offset = 0;
	pSelfData81 = (SelfData81*)&gRealData.externData[p81offset];
	
	return &gbSta[objLinkIdx];
}

uint16_t Pack81Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3,usVal = 0;
	bPackBuf[0] = 0x81;
	p81Data = (SelfData81*)&(pRealData->externData[p81offset]);		
	
	//填充自定义数据
//	bPackBuf[index++] = p81Data->FaultItemWarning;															//故障项预警				
//	bPackBuf[index++] = p81Data->RiskearlyOutputThermalRunawayWarning;					//输出热失控风险预警		
//	bPackBuf[index++] = p81Data->HydrogenSystemWarning;													//氢系统预警

	bPackBuf[index++] = p81Data->Risk->TotalVoltageOvervoltageLevel3Flag;					//总电压过压3级故障	
	bPackBuf[index++] = p81Data->Risk->TotalVoltageUndervoltageLevel3Flag;							//总电压欠压3级故障	
	bPackBuf[index++] = p81Data->Risk->MonomerVoltageOvervoltageLevel3Flag;							//单体电压过压3级故障
	bPackBuf[index++] = p81Data->Risk->MonomerVoltageeUndervoltageLevel3Flag;						//单体电压欠压3级故障
	bPackBuf[index++] = p81Data->Risk->BatteryVoltageRangeLargeLevel3Flag;							//电池电压极差过大3级故障
	bPackBuf[index++] = p81Data->Risk->FuelCellOvertemperatureLevel3Flag;								//燃料电池过温3级故障
	bPackBuf[index++] = p81Data->Risk->FuelCellTemperatureExcessiveDifLevel3Flag;				//燃料电池温差过大3级故障
	bPackBuf[index++] = p81Data->Risk->HydrogenPressureLevel3Flag;											//氢气压力3级故障
	bPackBuf[index++] = p81Data->Risk->HydrogenSystemTemperatureLevel3Flag;							//氢系统温度3级故障
	bPackBuf[index++] = p81Data->Risk->HydrogenBottleTemperatureLevel3Flag;							//氢瓶温度3级故障
	bPackBuf[index++] = p81Data->Risk->HydrogenLeakLevel3Flag;													//氢气泄露3级故障
	bPackBuf[index++] = p81Data->Risk->VehicleHighVoltageInsulationLevel3Flag;					//整车高压绝缘3级故障
	
	//燃料电池电堆个数
	bPackBuf[index++] = (uint8_t)(p81Data->FuelCellElectricPileNumber);
	
	//燃料电池系统状态
	bPackBuf[index++] = (uint8_t)(p81Data->FuelCellSystemStatus);
	
	//燃料电池电堆氢气入口压力
	usVal = (uint16_t)(p81Data->FuelCellStackHydrogenInletPressure);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->FuelCellStackHydrogenInletPressure + 100) * 10);//usVal * 100;
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);

	//燃料电池空气入口流量
	usVal = (uint16_t)(p81Data->FuelCellAirInletFlow);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)(p81Data->FuelCellAirInletFlow* 10);//usVal * 100;
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);

	//燃料电池电堆水温度（出水口温度）
	usVal = (uint8_t)(p81Data->FuelCellReactorWaterTemperature);
	if(usVal != 0xFF && usVal != 0xFE)
		usVal = (uint8_t)((p81Data->FuelCellReactorWaterTemperature+40)*1);
	bPackBuf[index++] = (uint8_t)usVal;

	//燃料电池输出功率（电堆功率）
	bPackBuf[index++] = (uint8_t)(p81Data->FuelCellOutputPower);
	
	//空气压缩机电压
	usVal = (uint16_t)(p81Data->AirCompressorVoltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->AirCompressorVoltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//空气压缩机电流
	usVal = (uint16_t)(p81Data->AirCompressorCurrent);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->AirCompressorCurrent+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);

	//氢气循环泵电压
	usVal = (uint16_t)(p81Data->HydrogenCirculatingPumpVoltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HydrogenCirculatingPumpVoltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);

	//氢气循环泵电流
	usVal = (uint16_t)(p81Data->HydrogenCirculatingPumpCurrent);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HydrogenCirculatingPumpCurrent+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//PTC电压
	usVal = (uint16_t)(p81Data->PTC_Voltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->PTC_Voltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//高压DC/DC输出电压
	usVal = (uint16_t)(p81Data->HighVoltage_DCDC_OutputVoltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HighVoltage_DCDC_OutputVoltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);

	//高压DC/DC输出电流
	usVal = (uint16_t)(p81Data->HighVoltage_DCDC_OutputCurrent);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HighVoltage_DCDC_OutputCurrent+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//高压DC/DC温度
	usVal = (uint8_t)(p81Data->HighVoltage_DCDC_OutputTemperature);
	if(usVal != 0xFF && usVal != 0xFE)
		usVal = (uint8_t)((p81Data->HighVoltage_DCDC_OutputTemperature+40)*1);
	bPackBuf[index++] = (uint8_t)usVal;
	
	//燃料电池系统氢气入口压力
	usVal = (uint16_t)(p81Data->FuelCellSystemHydrogenInletPressure);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->FuelCellSystemHydrogenInletPressure + 100) * 10);//usVal * 100;
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//空压机状态
	bPackBuf[index++] = (uint8_t)(p81Data->AirCompressorStatus);
	
	//氢回流泵状态
	bPackBuf[index++] = (uint8_t)(p81Data->HydrogenRefluxPumpStatus);
	
	//水泵状态
	bPackBuf[index++] = (uint8_t)(p81Data->PumpCondition);
	
	//水泵电压
	usVal = (uint16_t)(p81Data->PumpVoltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->PumpVoltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//水泵电流
	usVal = (uint16_t)(p81Data->PumpCurrent);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->PumpCurrent+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//PTC电流
	usVal = (uint16_t)(p81Data->PTC_Current);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->PTC_Current+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//高压DC/DC输入电压
	usVal = (uint16_t)(p81Data->HighVoltage_DCDC_InputVoltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HighVoltage_DCDC_InputVoltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//高压DC/DC输入电流
	usVal = (uint16_t)(p81Data->HighVoltage_DCDC_InputCurrent);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HighVoltage_DCDC_InputCurrent+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	//填充长度
	return index;	
}


//扩展实时数据，实时数据增加自定义数据
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t index = 0;
	index += Pack81Data(pRealData,&bPackBuf[index]);
	return index;
}
