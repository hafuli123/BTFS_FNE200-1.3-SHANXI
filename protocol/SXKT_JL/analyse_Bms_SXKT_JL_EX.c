/*
�� ����analyse_Bms_SXKT_JL_EX.c
�� �ܣ�ɽ�����⼪��ͨ��Э�� - �����Զ�����
�� ��: 2023/5/24
�� ˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
�� ��: HYQ
*/
#include "SXKT_JL/protocol_GB_EX_SXKT_JL.h"
#include "protocol_GB.h"
#include <stdlib.h>

/*
//ǿ��ط���3������
#define STRONGLYRISK	(gRealData.tempDiffAlert == 3 || gRealData.batHighTempAlert == 3 || gRealData.batHighVolAlert == 3 || gRealData.insulationFailtAlert == 3 || gRealData.batOverCharge == 3)	
//��ʧ�ط��ո�֪-1 ����T��20����Tmax��70������������5֡
#define THERMALRUNWAYRISK1	(gRealData.max_singleTemper - gRealData.min_singleTemper >= 20 && gRealData.max_singleTemper > 70)	
//��ʧ�ط��ո�֪-2 ����T��15����Tmax��60���ҳ�ŵ�����У�����V��200mV��������2֡
#define THERMALRUNWAYRISK2 	(gRealData.max_singleTemper - gRealData.min_singleTemper >= 15 && (gRealData.speed > 0 || gRealData.chargeState == STOP_CHARGE) && gRealData.max_singleVol - gRealData.min_singleVol > 0.2)
//��ϵͳ��ȫ����Ԥ�� ������Ũ��>=1%����ƿ���ѹ��>35MPa����ƿ���±���>70��C��������Ե����ֵ>600ŷ/��
#define HYDSYSSECURISK	(gRealData.maxHydrThickness >= 1 || gRealData.maxHydrPressure > 35 || gRealData.maxHydrSysTem > 70 || gRealData.maxHydrSysTem < -50 || gRealData.mohm > 600)

extern uint32_t gRealDataSendCnt;	//��¼ʵʱ����֡����
//����Ԥ��ģ��1
static uint32_t ariseFaultItemWarningCnt;		//���ֶ��������ʧ��Ԥ��ʱ��ʵʱ���ݵ�֡��
static uint8_t FaultItemWarningStep = 0;

//����Ԥ��ģ��2
static uint32_t ariseRiskearlyOutputThermalRunawayWarningCnt1;		//��ʧ�ط��ո�֪-1����ʱ��ʵʱ���ݵ�֡��
static uint32_t ariseRiskearlyOutputThermalRunawayWarningCnt2;		//��ʧ�ط��ո�֪-2����ʱ��ʵʱ���ݵ�֡��
static uint32_t ariseRiskearlyOutputThermalRunawayWarningCnt3;		//��ʧ�ط��ո�֪-3����ʱ��ʵʱ���ݵ�֡��
static uint8_t RiskearlyOutputThermalRunawayWarningStep1 = 0;	//��ʧ�ط��ո�֪-1
static uint8_t RiskearlyOutputThermalRunawayWarningStep2 = 0;	//��ʧ�ط��ո�֪-2
static uint8_t RiskearlyOutputThermalRunawayWarningStep3 = 0;	//��ʧ�ط��ո�֪-3

//����Ԥ��ģ��3
static uint32_t ariseHydrogenSystemWarningCnt;		//���ֶ��������ʧ��Ԥ��ʱ��ʵʱ���ݵ�֡��
static uint8_t HydrogenSystemWarningStep = 0;
*/
#define RISKWARNING 1
#define RISKNOWARNING 0


const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*���ݸߵ�λ*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN����ģʽ����ΪINTEL��ʽ�� MOTOROLA��ʽ*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN����ƫ������ϵ������ģʽFACTOR_OFFSET  OFFSET_FACTOR*/

static uint32_t RiskWarningBuzzesTime = 0;
void RiskWarningBuzzes(void)	//����Ԥ������
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

//����Ԥ��ģ��
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
			//ȼ�ϵ�ص�Ѹ���
			pSelfData81->FuelCellElectricPileNumber = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FA60D5:
		{
			//ȼ�ϵ��ϵͳ״̬
			pSelfData81->FuelCellSystemStatus = calcCanValue(0,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FA65D5:
		{
			//ȼ�ϵ�ص���������ѹ��
			pSelfData81->FuelCellStackHydrogenInletPressure = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//ȼ�ϵ�ؿ����������
			pSelfData81->FuelCellAirInletFlow = calcRealValue(8,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//ȼ�ϵ���������
			pSelfData81->FuelCellOutputPower = calcRealValue(8,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//ȼ�ϵ��ϵͳ�������ѹ��
			pSelfData81->FuelCellSystemHydrogenInletPressure = calcRealValue(24,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA62D5:
		{
			//ȼ�ϵ�ص��ˮ�¶�
			pSelfData81->FuelCellReactorWaterTemperature = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA67D5:
		{
			//����ѹ������ѹ
			pSelfData81->AirCompressorVoltage = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//����ѹ��������
			pSelfData81->AirCompressorCurrent = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//����ѭ���õ�ѹ
			pSelfData81->HydrogenCirculatingPumpVoltage = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//����ѭ���õ���
			pSelfData81->HydrogenCirculatingPumpCurrent = calcRealValue(48,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA66D5:
		{
			//PTC��ѹ
			pSelfData81->PTC_Voltage = calcRealValue(40,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//��ѹ��״̬
			pSelfData81->AirCompressorStatus = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//�������״̬
			pSelfData81->HydrogenRefluxPumpStatus = calcCanValue(8,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//ˮ��״̬
			pSelfData81->HydrogenRefluxPumpStatus = calcCanValue(12,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//ˮ�õ�ѹ
			pSelfData81->PumpVoltage = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//ˮ�õ���
			pSelfData81->PumpCurrent = calcRealValue(32,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//PTC����
			pSelfData81->PTC_Current = calcRealValue(56,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA63D5:
		{
			//��ѹDC/DC�����ѹ
			pSelfData81->HighVoltage_DCDC_OutputVoltage = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//��ѹDC/DC�������
			pSelfData81->HighVoltage_DCDC_OutputCurrent = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA61D5:
		{
			//��ѹDC/DC�¶�
			pSelfData81->HighVoltage_DCDC_OutputTemperature = calcRealValue(56,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
		case 0x18FA68D5:
		{
			//��ѹDC/DC�����ѹ
			pSelfData81->HighVoltage_DCDC_InputVoltage = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			//��ѹDC/DC�������
			pSelfData81->HighVoltage_DCDC_InputCurrent = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE); 
		}
		break;
	}
	
	
/*��һ��Ԥ��ģ��
	//����3�������ۺ�����Ԥ��ģ��
	if(STRONGLYRISK)	//ǿ��ط���3������
	{
		if(FaultItemWarningStep == 0)
		{
			ariseFaultItemWarningCnt = gRealDataSendCnt;	//��¼����ǿ������Ԥ��ʱ��ʵʱ����֡��
			FaultItemWarningStep = 1;			
		}
		else if(FaultItemWarningStep == 1)
		{
			if(gRealDataSendCnt - ariseFaultItemWarningCnt >= 3)	//����3֡
			{
				pSelfData81->FaultItemWarning = 1;	//������Ԥ��
			}
		}
	}
	else
	{
		FaultItemWarningStep = 0;
		pSelfData81->FaultItemWarning = 0;	//������Ԥ��
	}
	
	//���������ʧ�ط���Ԥ��ģ��
	if(THERMALRUNWAYRISK1)	//��ʧ�ط��ո�֪-1 ����T��20����Tmax��70������������5֡
	{
		if(RiskearlyOutputThermalRunawayWarningStep1 == 0)
		{
			ariseRiskearlyOutputThermalRunawayWarningCnt1 = gRealDataSendCnt;	//��¼���ֶ��������ʧ��Ԥ��-1ʱ��ʵʱ����֡��
			RiskearlyOutputThermalRunawayWarningStep1 = 1;
			RiskearlyOutputThermalRunawayWarningStep2 = 0;
			RiskearlyOutputThermalRunawayWarningStep3 = 0;
		}
		else if(RiskearlyOutputThermalRunawayWarningStep1 == 1)
		{
			if(gRealDataSendCnt - ariseRiskearlyOutputThermalRunawayWarningCnt1 >= 5)	//��������5֡
			{
				pSelfData81->RiskearlyOutputThermalRunawayWarning = 2;	//�����ʧ�ط���Ԥ��
			}
		}
	}
	else if(THERMALRUNWAYRISK2)		//��ʧ�ط��ո�֪-2 ����T��15����Tmax��60���ҳ�ŵ�����У�����V��200mV��������2֡
	{
		if(RiskearlyOutputThermalRunawayWarningStep2 == 0)
		{
			ariseRiskearlyOutputThermalRunawayWarningCnt2 = gRealDataSendCnt;	//��¼���ֶ��������ʧ��Ԥ��-2ʱ��ʵʱ����֡��
			RiskearlyOutputThermalRunawayWarningStep1 = 0;
			RiskearlyOutputThermalRunawayWarningStep2 = 1;
			RiskearlyOutputThermalRunawayWarningStep3 = 0;
		}
		else if(RiskearlyOutputThermalRunawayWarningStep2 == 1)
		{
			if(gRealDataSendCnt - ariseRiskearlyOutputThermalRunawayWarningCnt2 >= 2)
			{
				pSelfData81->RiskearlyOutputThermalRunawayWarning = 2;	//�����ʧ�ط���Ԥ��
			}
		}
	}
	else if(gRealData.max_singleTemper > 65)	//��ʧ�ط��ո�֪-3 Tmax��65�����¶ȱ�������5֡
	{
		if(RiskearlyOutputThermalRunawayWarningStep3 == 0)
		{
			ariseRiskearlyOutputThermalRunawayWarningCnt3 = gRealDataSendCnt;	//��¼���ֶ��������ʧ��Ԥ��-3ʱ��ʵʱ����֡��
			RiskearlyOutputThermalRunawayWarningStep1 = 0;
			RiskearlyOutputThermalRunawayWarningStep2 = 0;
			RiskearlyOutputThermalRunawayWarningStep3 = 1;
		}
		else if(RiskearlyOutputThermalRunawayWarningStep3 == 1)
		{
			if(gRealDataSendCnt - ariseRiskearlyOutputThermalRunawayWarningCnt3 >= 5)
			{
				pSelfData81->RiskearlyOutputThermalRunawayWarning = 1;	//�����ʧ�ط���Ԥ��
			}
		}
	}
	else
	{
		RiskearlyOutputThermalRunawayWarningStep1 = 0;
		RiskearlyOutputThermalRunawayWarningStep2 = 0;
		RiskearlyOutputThermalRunawayWarningStep3 = 0;
		pSelfData81->RiskearlyOutputThermalRunawayWarning = 0;	//�����ʧ�ط���Ԥ��
	}
	
	//��ϵͳ��ȫ����Ԥ��ģ��
	if(HYDSYSSECURISK)
	{
		pSelfData81->HydrogenSystemWarning = 1;	//��ϵͳ��ȫԤ��
	}
	else
	{
		pSelfData81->HydrogenSystemWarning = 0;	//��ϵͳ��ȫԤ��
	}
*/
	//����Ԥ��ģ��
	RiskWarningFlag = RiskEarlyWarningModel();
	if(RiskWarningFlag == RISKWARNING)
		RiskWarningBuzzes();//����
	
}
