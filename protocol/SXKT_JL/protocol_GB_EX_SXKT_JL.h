//云智通定义文件
#ifndef __YZT_DEF_H
#define __YZT_DEF_H

#include "fun_can.h"
#include "bsp_rtc.h"
#include "stdint.h"
#include "cmsis_os2.h"
#include "bsp_io.h"

//3级报警标志
typedef	union{
		struct{
				uint8_t TotalVoltageOvervoltageLevel3Flag:1;					//总电压过压3级
				uint8_t TotalVoltageUndervoltageLevel3Flag:1;					//总电压欠压3级	
				uint8_t MonomerVoltageOvervoltageLevel3Flag:1;				//单体电压过压3级
				uint8_t MonomerVoltageeUndervoltageLevel3Flag:1;			//单体电压欠压3级
				uint8_t BatteryVoltageRangeLargeLevel3Flag:1;					//电池电压极差过大3级
				uint8_t FuelCellOvertemperatureLevel3Flag:1;					//燃料电池过温3级
				uint8_t FuelCellTemperatureExcessiveDifLevel3Flag:1;	//燃料电池温差过大3级
				uint8_t HydrogenPressureLevel3Flag:1;									//氢气压力3级
				uint8_t HydrogenSystemTemperatureLevel3Flag:1;				//氢系统温度3级
				uint8_t HydrogenBottleTemperatureLevel3Flag:1;				//氢瓶温度3级
				uint8_t HydrogenLeakLevel3Flag:1;											//氢气泄露3级
				uint8_t VehicleHighVoltageInsulationLevel3Flag:1;			//整车高压绝缘3级
		};
		uint16_t value;
}RiskFlag;

//自定义 81数据
typedef struct _selfData81{
//	uint8_t FaultItemWarning:1;													//故障项预警
//	uint8_t RiskearlyOutputThermalRunawayWarning;			//输出热失控风险预警
//	uint8_t HydrogenSystemWarning:1;										//氢系统预警
	RiskFlag *Risk;																//3级报警
	uint8_t FuelCellElectricPileNumber;						//燃料电池电堆个数
	uint8_t FuelCellSystemStatus;									//燃料电池系统状态
	float FuelCellStackHydrogenInletPressure;			//燃料电池电堆氢气入口压力
	float FuelCellAirInletFlow;										//燃料电池空气入口流量
	int FuelCellReactorWaterTemperature;					//燃料电池电堆水温度（出水口温度）
	float	FuelCellOutputPower;										//燃料电池输出功率（电堆功率）
	float	AirCompressorVoltage;										//空气压缩机电压
	float AirCompressorCurrent;										//空气压缩机电流
	float HydrogenCirculatingPumpVoltage;					//氢气循环泵电压
	float HydrogenCirculatingPumpCurrent;					//氢气循环泵电流
	float PTC_Voltage;														//PTC电压
	float HighVoltage_DCDC_OutputVoltage;					//高压DC/DC输出电压
	float HighVoltage_DCDC_OutputCurrent;					//高压DC/DC输出电流
	int HighVoltage_DCDC_OutputTemperature;				//高压DC/DC温度
	float FuelCellSystemHydrogenInletPressure;		//燃料电池系统氢气入口压力
	uint8_t AirCompressorStatus;									//空压机状态
	uint8_t HydrogenRefluxPumpStatus;							//氢回流泵状态
	uint8_t PumpCondition;												//水泵状态
	float PumpVoltage;														//水泵电压
	float PumpCurrent;														//水泵电流
	float PTC_Current;														//PTC电流
	float HighVoltage_DCDC_InputVoltage;					//高压DC/DC输入电压
	float HighVoltage_DCDC_InputCurrent;					//高压DC/DC输入电流
	
}SelfData81;

extern SelfData81* pSelfData81;
void unpackYZTcan(uint8_t ch,CAN_msg *msg);

#endif
