/*
�� ����protocol_GB_EX_SXKT_JL.c
�� �ܣ�ɽ�����⼪������Ԥ��ͨ��Э�� - ���������Զ�����չ����
�� ��: 2023/5/24
�� ˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
�� ��: HYQ
*/

#include "SXKT_JL/protocol_GB_EX_SXKT_JL.h"


SelfData81* pSelfData81;
SelfData81*  p81Data;
static uint16_t p81offset;

typedef struct _GBSTA
{
	//�ⲿ���Ʋ���
	uint8_t bLink;									//��·��
	uint8_t bUse;										//�Ƿ�ʹ��
	char *vin;											//���ܺ�
	uint8_t sendOverTimeCnt;				//���ʹ���
	//�ڲ�ʱ���
	uint32_t lockStaStamp;					//��״̬ʱ���
	//��������
	uint8_t* buff;									//�������ݻ�����
	uint16_t buffLen;								//�������ݻ���������
}GBSTA;


#define MAX_YZT_LINK 1

static GBSTA gbSta[MAX_YZT_LINK] = {0};

//��չЭ���ʼ������������״̬����
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen)
{
	uint8_t i,oldLinkIdx = 0xFF,objLinkIdx = 0xFF;
	for(i = 0;i < MAX_YZT_LINK;i++)
	{
		if(gbSta[i].bUse == 1 && gbSta[i].bLink == link)
		{
			oldLinkIdx = i;//���³�ʼ��
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
	//�ⲿ����
	gbSta[objLinkIdx].bUse = 1;
	gbSta[objLinkIdx].bLink = link;
	gbSta[objLinkIdx].vin = vin;
	gbSta[objLinkIdx].buff = buff;
	gbSta[objLinkIdx].buffLen = buffLen;
	//�ڲ�
	gbSta[objLinkIdx].sendOverTimeCnt = 0;
	gbSta[objLinkIdx].lockStaStamp = 0;
	//�Զ������ݷ����ڴ棬����ƫ����
	
	p81offset = 0;
	pSelfData81 = (SelfData81*)&gRealData.externData[p81offset];
	
	return &gbSta[objLinkIdx];
}

uint16_t Pack81Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t index = 3,usVal = 0;
	bPackBuf[0] = 0x81;
	p81Data = (SelfData81*)&(pRealData->externData[p81offset]);		
	
	//����Զ�������
//	bPackBuf[index++] = p81Data->FaultItemWarning;															//������Ԥ��				
//	bPackBuf[index++] = p81Data->RiskearlyOutputThermalRunawayWarning;					//�����ʧ�ط���Ԥ��		
//	bPackBuf[index++] = p81Data->HydrogenSystemWarning;													//��ϵͳԤ��

	bPackBuf[index++] = p81Data->Risk->TotalVoltageOvervoltageLevel3Flag;					//�ܵ�ѹ��ѹ3������	
	bPackBuf[index++] = p81Data->Risk->TotalVoltageUndervoltageLevel3Flag;							//�ܵ�ѹǷѹ3������	
	bPackBuf[index++] = p81Data->Risk->MonomerVoltageOvervoltageLevel3Flag;							//�����ѹ��ѹ3������
	bPackBuf[index++] = p81Data->Risk->MonomerVoltageeUndervoltageLevel3Flag;						//�����ѹǷѹ3������
	bPackBuf[index++] = p81Data->Risk->BatteryVoltageRangeLargeLevel3Flag;							//��ص�ѹ�������3������
	bPackBuf[index++] = p81Data->Risk->FuelCellOvertemperatureLevel3Flag;								//ȼ�ϵ�ع���3������
	bPackBuf[index++] = p81Data->Risk->FuelCellTemperatureExcessiveDifLevel3Flag;				//ȼ�ϵ���²����3������
	bPackBuf[index++] = p81Data->Risk->HydrogenPressureLevel3Flag;											//����ѹ��3������
	bPackBuf[index++] = p81Data->Risk->HydrogenSystemTemperatureLevel3Flag;							//��ϵͳ�¶�3������
	bPackBuf[index++] = p81Data->Risk->HydrogenBottleTemperatureLevel3Flag;							//��ƿ�¶�3������
	bPackBuf[index++] = p81Data->Risk->HydrogenLeakLevel3Flag;													//����й¶3������
	bPackBuf[index++] = p81Data->Risk->VehicleHighVoltageInsulationLevel3Flag;					//������ѹ��Ե3������
	
	//ȼ�ϵ�ص�Ѹ���
	bPackBuf[index++] = (uint8_t)(p81Data->FuelCellElectricPileNumber);
	
	//ȼ�ϵ��ϵͳ״̬
	bPackBuf[index++] = (uint8_t)(p81Data->FuelCellSystemStatus);
	
	//ȼ�ϵ�ص���������ѹ��
	usVal = (uint16_t)(p81Data->FuelCellStackHydrogenInletPressure);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->FuelCellStackHydrogenInletPressure + 100) * 10);//usVal * 100;
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);

	//ȼ�ϵ�ؿ����������
	usVal = (uint16_t)(p81Data->FuelCellAirInletFlow);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)(p81Data->FuelCellAirInletFlow* 10);//usVal * 100;
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);

	//ȼ�ϵ�ص��ˮ�¶ȣ���ˮ���¶ȣ�
	usVal = (uint8_t)(p81Data->FuelCellReactorWaterTemperature);
	if(usVal != 0xFF && usVal != 0xFE)
		usVal = (uint8_t)((p81Data->FuelCellReactorWaterTemperature+40)*1);
	bPackBuf[index++] = (uint8_t)usVal;

	//ȼ�ϵ��������ʣ���ѹ��ʣ�
	bPackBuf[index++] = (uint8_t)(p81Data->FuelCellOutputPower);
	
	//����ѹ������ѹ
	usVal = (uint16_t)(p81Data->AirCompressorVoltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->AirCompressorVoltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//����ѹ��������
	usVal = (uint16_t)(p81Data->AirCompressorCurrent);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->AirCompressorCurrent+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);

	//����ѭ���õ�ѹ
	usVal = (uint16_t)(p81Data->HydrogenCirculatingPumpVoltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HydrogenCirculatingPumpVoltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);

	//����ѭ���õ���
	usVal = (uint16_t)(p81Data->HydrogenCirculatingPumpCurrent);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HydrogenCirculatingPumpCurrent+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//PTC��ѹ
	usVal = (uint16_t)(p81Data->PTC_Voltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->PTC_Voltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//��ѹDC/DC�����ѹ
	usVal = (uint16_t)(p81Data->HighVoltage_DCDC_OutputVoltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HighVoltage_DCDC_OutputVoltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);

	//��ѹDC/DC�������
	usVal = (uint16_t)(p81Data->HighVoltage_DCDC_OutputCurrent);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HighVoltage_DCDC_OutputCurrent+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//��ѹDC/DC�¶�
	usVal = (uint8_t)(p81Data->HighVoltage_DCDC_OutputTemperature);
	if(usVal != 0xFF && usVal != 0xFE)
		usVal = (uint8_t)((p81Data->HighVoltage_DCDC_OutputTemperature+40)*1);
	bPackBuf[index++] = (uint8_t)usVal;
	
	//ȼ�ϵ��ϵͳ�������ѹ��
	usVal = (uint16_t)(p81Data->FuelCellSystemHydrogenInletPressure);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->FuelCellSystemHydrogenInletPressure + 100) * 10);//usVal * 100;
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//��ѹ��״̬
	bPackBuf[index++] = (uint8_t)(p81Data->AirCompressorStatus);
	
	//�������״̬
	bPackBuf[index++] = (uint8_t)(p81Data->HydrogenRefluxPumpStatus);
	
	//ˮ��״̬
	bPackBuf[index++] = (uint8_t)(p81Data->PumpCondition);
	
	//ˮ�õ�ѹ
	usVal = (uint16_t)(p81Data->PumpVoltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->PumpVoltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//ˮ�õ���
	usVal = (uint16_t)(p81Data->PumpCurrent);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->PumpCurrent+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//PTC����
	usVal = (uint16_t)(p81Data->PTC_Current);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->PTC_Current+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//��ѹDC/DC�����ѹ
	usVal = (uint16_t)(p81Data->HighVoltage_DCDC_InputVoltage);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HighVoltage_DCDC_InputVoltage+0)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	//��ѹDC/DC�������
	usVal = (uint16_t)(p81Data->HighVoltage_DCDC_InputCurrent);
	if(usVal != 0xFFFF && usVal != 0xFFFE)
		usVal = (uint16_t)((p81Data->HighVoltage_DCDC_InputCurrent+1000)*10);
	bPackBuf[index++] = (uint8_t)(usVal/256);
	bPackBuf[index++] = (uint8_t)(usVal%256);
	
	bPackBuf[1] = (index - 3) >> 8;
	bPackBuf[2] = (index - 3) >> 0;
	//��䳤��
	return index;	
}


//��չʵʱ���ݣ�ʵʱ���������Զ�������
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t index = 0;
	index += Pack81Data(pRealData,&bPackBuf[index]);
	return index;
}
