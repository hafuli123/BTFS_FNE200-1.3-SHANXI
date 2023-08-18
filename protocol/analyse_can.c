#include "fun_can.h"
#include "stm32f4xx_can.h"
#include "cmsis_os2.h"
#include "string.h"

void iniCanData(void)
{
	uint16_t i;
	memset(&gRealData,0,sizeof(gRealData));
	gRealData.obdDiagProt = 0xFF;
	gRealData.milState = 0xFF;
	gRealData.speed = 0;
	gRealData.barometric = 0xFF;
	gRealData.engineTorque = 0xFF;
	gRealData.frictionTorque = 0xFF;
	gRealData.engineSpeed = 0;
	gRealData.engineFuelFlow = 0xFFFF;
	gRealData.scrUpperNOxSensor = 0xFFFF;
	gRealData.scrLowerNOxSensor = 0xFFFF;
	gRealData.scrInletTemp = 0xFFFF;
	gRealData.scrOutletTemp = 0xFFFF;
	gRealData.dpfPressDiff = 0xFFFF;
	gRealData.intakeFlowrate = 0xFFFF;
	gRealData.reagentSurplus = 0xFF;
	gRealData.engineCoolantTemp = 0xFF;
	gRealData.tankLevel = 0xFF;
	gRealData.totalMileage= 0xFFFFFFFF;
	gRealData.engineTorqueMode = 0xFF;
	gRealData.acceleratorVal =0xFF;
	gRealData.EngTotalFuelUsed = 0xFFFFFFFF;
	gRealData.ureaTankTemp = 0xFF;
	gRealData.actAreaInjectVal = 0xFFFFFFFF;
	gRealData.totalUreaUsed = 0xFFFFFFFF;
	gRealData.dpfExhaustTemp = 0xFFFF;
	gRealData.engFuelRate = 0xFFFF;
	gRealData.engTotalHoursOfOperation = 0xFFFFFFFF;
	gRealData.engReferenceTorque = 0xFFFF;
	gRealData.rechargeSysCodeLen = 0;
	//���������ֵֻ��FE
	gRealData.crankshaftSpeed = 0xFFFE;
	gRealData.fuelConsumption = 0xFFFE;
	gRealData.fuelBatVol = 0xFFFE;
	gRealData.maxHydrPressure = 0xFFFE;
	gRealData.totalMileage = 0xFFFFFFFE;
	gRealData.total_volt = 0xFFFE;
	gRealData.total_current = 0xFFFE;
	gRealData.max_singleVol = 0xFFFE;
	gRealData.min_singleVol = 0xFFFE;

    for(i = 0;i < sizeof(gRealData.single_vol) /4;++i)
		gRealData.single_vol[i] = 0xFFFE;	
	gRealData.stall = 0;
	gRealData.motorCnt = 1;
	gRealData.subBatSysCnt = 1;
	gRealData.operationState = EV;
	/*����ÿ���������ĵ����ѹ�͵����¶�����*/
	for(i = 0;i < sizeof(gRealData.subSysData) / sizeof(gRealData.subSysData[0]);++i)
	{
		gRealData.subSysData[i].subSysIdx = (i+1);
		gRealData.subSysData[i].singleVolCnt = 0;
		gRealData.subSysData[i].singleTemCnt = 0;
		gRealData.subSysData[i].singleVolStartIdx = 0;
		gRealData.subSysData[i].singleTemStartIdx = 0;
	}
	for(i=0 ;i < sizeof(gRealData.motorData) / sizeof(gRealData.motorData[0]);++i)
	{
		gRealData.motorData[i].motorIdx = (i+1);
	}
	iniFvCanData();
	iniEvCanData();
}

/*
*********************************************************************************************************
*	�� �� ��: calcExtremum
*	����˵��: ���㼫ֵ���ݣ����������͵����ѹ���¶ȡ���㵥���ѹ���¶Ȱ���ţ�������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void CalcExtremum(void)
{
	int i,j,k;
	int volIndex=0,temperIndex=0;
	//��ߵ�ѹ�����Ϣ
	uint8_t maxVolPackIndex = 1;
	uint16_t maxVolIndex = 1;
	float maxVol = gRealData.single_vol[0];
	//��͵�ѹ�����Ϣ
	uint8_t minVolPackIndex = 1;
	uint16_t minVolIndex = 1;
	float minVol = gRealData.single_vol[0];
	//����¶������Ϣ
	uint8_t maxTemperPackIndex = 1;
	uint16_t maxTemperIndex = 1;
	int16_t maxTemper = gRealData.single_temper[0];
	//����¶������Ϣ
	uint8_t minTemperPackIndex = 1;
	uint16_t minTemperIndex = 1;
	int16_t minTemper = gRealData.single_temper[0];
	
	for(i=0;i<gRealData.subBatSysCnt && gRealData.subBatSysCnt < 0xFE;++i)
	{
		for(j=0;j<gRealData.subSysData[i].singleVolCnt;++j)
		{
			if(gRealData.single_vol[volIndex] != 0xFFFF && gRealData.single_vol[volIndex] != 0xFFFE && gRealData.single_vol[volIndex] >= 0)
			{
				if(gRealData.single_vol[volIndex] > maxVol)
				{
					maxVolPackIndex = i+1;
					maxVolIndex = j+1;
					maxVol = gRealData.single_vol[volIndex];
				}
				
				if(gRealData.single_vol[volIndex] < minVol)
				{
					minVolPackIndex = i+1;
					minVolIndex = j+1;
					minVol = gRealData.single_vol[volIndex];
				}
			}
			++volIndex;
		}
		
		for(k=0;k<gRealData.subSysData[i].singleTemCnt;++k)
		{
			if(gRealData.single_temper[temperIndex] > maxTemper)
			{
				maxTemperPackIndex = i+1;
				maxTemperIndex = k+1;
				maxTemper = gRealData.single_temper[temperIndex];
			}
			
			if(gRealData.single_temper[temperIndex] < minTemper)
			{
				minTemperPackIndex = i+1;
				minTemperIndex = k+1;
				minTemper = gRealData.single_temper[temperIndex];
			}
			++temperIndex;
		}
	}
	
	gRealData.maxVolPack_index = maxVolPackIndex;
	gRealData.maxVol_index = maxVolIndex;
	gRealData.max_singleVol = maxVol;
	
	gRealData.minVolPack_index = minVolPackIndex;
	gRealData.minVol_index = minVolIndex;
	gRealData.min_singleVol = minVol;
	
	gRealData.maxTemperPack_index = maxTemperPackIndex;
	gRealData.maxTemper_index = maxTemperIndex;
	gRealData.max_singleTemper = maxTemper;
	
	gRealData.minTemperPack_index = minTemperPackIndex;
	gRealData.minTemper_index = minTemperIndex;
	gRealData.min_singleTemper = minTemper;
}

void unpackCAN(uint8_t ch,CAN_msg *msg)
{
	//ȼ�ͳ�CAN����
	unpackFvCAN(ch,msg);
	//�綯��CAN����
	unpackEvCAN(ch,msg);
	if(CALC_EXTREMUN == 1)
	{
		CalcExtremum();
	}
}

//Ĭ�ϳ���
__attribute__((weak)) const char* CAR_TYPE = "DEFAULT";

//Ĭ�ϼ�ֵ���㷽ʽ
__attribute__((weak)) const uint8_t CALC_EXTREMUN = 0;

//Ĭ��ȼ�ͳ���ʼ��
__attribute__((weak)) void iniFvCanData(void)
{
	
}

//Ĭ�ϵ綯����ʼ��
__attribute__((weak)) void iniEvCanData(void)
{
	
}

//Ĭ��ȼ�ͳ�������
__attribute__((weak))  void unpackFvCAN(uint8_t ch,CAN_msg *msg)
{
	
}

//Ĭ�ϵ綯������
__attribute__((weak))  void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{
	
}

__attribute__((weak))  void udsInit(uint8_t ch)
{
	
}
__attribute__((weak))  void udsProc (void)
{
	
}
