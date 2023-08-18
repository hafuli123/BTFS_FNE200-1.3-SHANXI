//����ͨ�����ļ�
#ifndef __YZT_DEF_H
#define __YZT_DEF_H

#include "fun_can.h"
#include "bsp_rtc.h"
#include "stdint.h"
#include "cmsis_os2.h"
#include "bsp_io.h"

//3��������־
typedef	union{
		struct{
				uint8_t TotalVoltageOvervoltageLevel3Flag:1;					//�ܵ�ѹ��ѹ3��
				uint8_t TotalVoltageUndervoltageLevel3Flag:1;					//�ܵ�ѹǷѹ3��	
				uint8_t MonomerVoltageOvervoltageLevel3Flag:1;				//�����ѹ��ѹ3��
				uint8_t MonomerVoltageeUndervoltageLevel3Flag:1;			//�����ѹǷѹ3��
				uint8_t BatteryVoltageRangeLargeLevel3Flag:1;					//��ص�ѹ�������3��
				uint8_t FuelCellOvertemperatureLevel3Flag:1;					//ȼ�ϵ�ع���3��
				uint8_t FuelCellTemperatureExcessiveDifLevel3Flag:1;	//ȼ�ϵ���²����3��
				uint8_t HydrogenPressureLevel3Flag:1;									//����ѹ��3��
				uint8_t HydrogenSystemTemperatureLevel3Flag:1;				//��ϵͳ�¶�3��
				uint8_t HydrogenBottleTemperatureLevel3Flag:1;				//��ƿ�¶�3��
				uint8_t HydrogenLeakLevel3Flag:1;											//����й¶3��
				uint8_t VehicleHighVoltageInsulationLevel3Flag:1;			//������ѹ��Ե3��
		};
		uint16_t value;
}RiskFlag;

//�Զ��� 81����
typedef struct _selfData81{
//	uint8_t FaultItemWarning:1;													//������Ԥ��
//	uint8_t RiskearlyOutputThermalRunawayWarning;			//�����ʧ�ط���Ԥ��
//	uint8_t HydrogenSystemWarning:1;										//��ϵͳԤ��
	RiskFlag *Risk;																//3������
	uint8_t FuelCellElectricPileNumber;						//ȼ�ϵ�ص�Ѹ���
	uint8_t FuelCellSystemStatus;									//ȼ�ϵ��ϵͳ״̬
	float FuelCellStackHydrogenInletPressure;			//ȼ�ϵ�ص���������ѹ��
	float FuelCellAirInletFlow;										//ȼ�ϵ�ؿ����������
	int FuelCellReactorWaterTemperature;					//ȼ�ϵ�ص��ˮ�¶ȣ���ˮ���¶ȣ�
	float	FuelCellOutputPower;										//ȼ�ϵ��������ʣ���ѹ��ʣ�
	float	AirCompressorVoltage;										//����ѹ������ѹ
	float AirCompressorCurrent;										//����ѹ��������
	float HydrogenCirculatingPumpVoltage;					//����ѭ���õ�ѹ
	float HydrogenCirculatingPumpCurrent;					//����ѭ���õ���
	float PTC_Voltage;														//PTC��ѹ
	float HighVoltage_DCDC_OutputVoltage;					//��ѹDC/DC�����ѹ
	float HighVoltage_DCDC_OutputCurrent;					//��ѹDC/DC�������
	int HighVoltage_DCDC_OutputTemperature;				//��ѹDC/DC�¶�
	float FuelCellSystemHydrogenInletPressure;		//ȼ�ϵ��ϵͳ�������ѹ��
	uint8_t AirCompressorStatus;									//��ѹ��״̬
	uint8_t HydrogenRefluxPumpStatus;							//�������״̬
	uint8_t PumpCondition;												//ˮ��״̬
	float PumpVoltage;														//ˮ�õ�ѹ
	float PumpCurrent;														//ˮ�õ���
	float PTC_Current;														//PTC����
	float HighVoltage_DCDC_InputVoltage;					//��ѹDC/DC�����ѹ
	float HighVoltage_DCDC_InputCurrent;					//��ѹDC/DC�������
	
}SelfData81;

extern SelfData81* pSelfData81;
void unpackYZTcan(uint8_t ch,CAN_msg *msg);

#endif
