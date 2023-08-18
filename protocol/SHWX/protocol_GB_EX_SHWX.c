/*
�� ����protocol_GB_EX_SHWX.c
�� �ܣ��Ϻ����� - ���������Զ�����չ����
�� ��: 2022/12/7
�� ˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
�� ��: HYQ
*/

#include "SHWX/protocol_GB_EX_SHWX.h"
#define MAX_YZT_LINK 1

uint8_t isChgVIN = 0;

//�Ϻ������Զ�������
SelfData80* pSelfData80 = NULL;					
SelfData81* pSelfData81 = NULL;
SelfData82* pSelfData82 = NULL;
SelfData83 pSelfData83;
SelfData84 pSelfData84;
SelfData85 pSelfData85 ;
SelfData86 pSelfData86 ;
SelfData87 pSelfData87 ;
SelfData88 pSelfData88 ;
SelfData89 pSelfData89 ;
SelfData8A pSelfData8A ;
SelfData8B pSelfData8B ;
SelfData8C pSelfData8C ;
SelfData8D pSelfData8D ;
SelfData8E pSelfData8E ;
SelfData8F pSelfData8F ;
SelfData90 pSelfData90 ;
SelfData91 pSelfData91 ;
data83 data_83;
float SOH;

static uint16_t p80offset;
static uint16_t p81offset;
static uint16_t p82offset;
static uint16_t p83offset;
static uint16_t p84offset;
static uint16_t p85offset;
static uint16_t p86offset;
static uint16_t p87offset;
static uint16_t p88offset;
static uint16_t p89offset;
static uint16_t p8Aoffset;
static uint16_t p8Boffset;
static uint16_t p8Coffset;
static uint16_t p8Doffset;
static uint16_t p8Eoffset;
static uint16_t p8Foffset;
static uint16_t p90offset;
static uint16_t p91offset;


static uint16_t PackVehicleState(RealData *pRealData,uint8_t *bPackBuf);	/*�������״̬����*/
static uint16_t PackOnBoardHydSys(RealData *pRealData,uint8_t *bPackBuf);	/*���������ϵͳ����*/
static uint16_t PackFuelCellSys(RealData *pRealData,uint8_t *bPackBuf);		/*���ȼ�ϵ��ϵͳ����*/
static uint16_t PackVehicleAlarm(RealData *pRealData,uint8_t *bPackBuf);	/*��������쳣��������*/
static uint16_t PackDirectionSlope(RealData *pRealData,uint8_t *bPackBuf);	/*��������¶�����*/
static uint16_t PackHydrogenFuelCell(RealData *pRealData,uint8_t *bPackBuf); /*��ȼ������*/


static uint16_t PackVehicleData(RealData *pRealData,uint8_t *bPackBuf);	/*��չЭ�顪��������*/


typedef struct _GBSTA
{
	//�ⲿ���Ʋ���
	uint8_t bLink;									//��·��
	uint8_t bUse;										//�Ƿ�ʹ��
	char *vin;											//���ܺ�
	uint8_t bLogouted;							//�ǳ���־λ 0:δ�ǳ� 1:�ѵǳ�
	uint8_t sendOverTimeCnt;				//���ʹ���
	//�ڲ�ʱ���
	uint32_t chgPowerStamp;					//�������ϱ�ʱ���
	//��������
	uint8_t* buff;									//�������ݻ�����
	uint16_t buffLen;								//�������ݻ���������
}GBSTA;

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
    
    pSelfData83.AccumulatedChargeCapacity = 0xFFFFFFFE;
	//�Զ������ݷ����ڴ棬����ƫ����
	p80offset = 0;
	pSelfData80 = (SelfData80*)&gRealData.externData[p80offset];
    
	p81offset = p80offset + sizeof(SelfData80);
	pSelfData81 = (SelfData81*)&gRealData.externData[p81offset]; 

	p82offset = p81offset + sizeof(SelfData81);
	pSelfData82 = (SelfData82*)&gRealData.externData[p82offset];
	
//    p83offset = p82offset + sizeof(SelfData82);
////	pSelfData83 = (SelfData83*)&gRealData.externData[p83offset];
//	
//	p84offset = p83offset + sizeof(SelfData83);
////	pSelfData84 = (SelfData84*)&gRealData.externData[p84offset];
//	
//	p85offset = p84offset + sizeof(SelfData84);
////	pSelfData85 = (SelfData85*)&gRealData.externData[p85offset];

//	p86offset = p85offset + sizeof(SelfData85);
////	pSelfData86 = (SelfData86*)&gRealData.externData[p86offset];
//	
//	p87offset = p86offset + sizeof(SelfData86);
////	pSelfData87 = (SelfData87*)&gRealData.externData[p87offset];
//	
//	p88offset = p87offset + sizeof(SelfData87);
////	pSelfData88 = (SelfData88*)&gRealData.externData[p88offset];
//	
//	p89offset = p88offset + sizeof(SelfData88);
////	pSelfData89 = (SelfData89*)&gRealData.externData[p89offset];
//	
//	p8Aoffset = p89offset + sizeof(SelfData89);
////	pSelfData8A = (SelfData8A*)&gRealData.externData[p8Aoffset];
//	
//	p8Boffset = p8Aoffset + sizeof(SelfData8A);
////	pSelfData8B = (SelfData8B*)&gRealData.externData[p8Boffset];
//	
//	p8Coffset = p8Boffset + sizeof(SelfData8B);
////	pSelfData8C = (SelfData8C*)&gRealData.externData[p8Coffset];
//	
//	p8Doffset = p8Coffset + sizeof(SelfData8C);
////	pSelfData8D = (SelfData8D*)&gRealData.externData[p8Doffset];

//	p8Eoffset = p8Doffset + sizeof(SelfData8D);
////	pSelfData8E = (SelfData8E*)&gRealData.externData[p8Eoffset];
//	
//	p8Foffset = p8Eoffset + sizeof(SelfData8E);
////	pSelfData8F = (SelfData8F*)&gRealData.externData[p8Foffset];
//	
//	p90offset = p8Foffset + sizeof(SelfData8F);
////	pSelfData90 = (SelfData90*)&gRealData.externData[p90offset];
//	
//	p91offset = p90offset + sizeof(SelfData90);
//	pSelfData91 = (SelfData91*)&gRealData.externData[p91offset];
	return &gbSta[objLinkIdx];
}

/*******************************************�Ϻ��ر�����****************************************��*/
/*�������״̬����*/
static uint16_t PackVehicleState(RealData *pRealData,uint8_t *bPackBuf)
{
	float data;
	uint16_t PackLen = 0, Idx = 0, usVal = 0;
	int16_t sVal = 0;
	uint8_t len = 7;
	if(bPackBuf != 0)
	{
	    bPackBuf[Idx++] = 0x21;
		
		//˲ʱ��� 
		usVal = (uint16_t)(gRealData.vehiclestatedata.InstantaneousHydrogenConsumption);
		if(usVal != 0xFFFF && usVal != 0xFFFE && gRealData.fuelcellsysdata[0].FuelCellSysWorkStatus == 2)
			usVal = (uint16_t)(gRealData.vehiclestatedata.InstantaneousHydrogenConsumption * 100);//usVal * 100;
		else
			usVal = 0;
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
	    
		//����ϵͳ״̬
	  bPackBuf[Idx++] = gRealData.vehiclestatedata.PowerSystemState;
	
    //�յ�ϵͳʵʱ����
		usVal = (uint16_t)gRealData.vehiclestatedata.PowerAirConditioningSystem; 
		if(usVal != 0xFF && usVal != 0xFE)
			usVal = (uint16_t)(gRealData.vehiclestatedata.PowerAirConditioningSystem * 10);
	    bPackBuf[Idx++] = (uint8_t)usVal;
	
	    //���ٶȣ���ײ�����������
		bPackBuf[Idx++] = gRealData.vehiclestatedata.AccelerationSensorOutput;
	
		//���⻷���¶�
		usVal = (int16_t)(gRealData.vehiclestatedata.AmbientTemperatureOutsideVehicle); 
		if(usVal != 0xFF && usVal != 0xFE)
			usVal = gRealData.vehiclestatedata.AmbientTemperatureOutsideVehicle + 40;
	    bPackBuf[Idx++] = (uint8_t)usVal;
	
		//����ѹ��
		usVal = (uint16_t)(gRealData.vehiclestatedata.AtmosphericPressure); 
		if(usVal != 0xFF && usVal != 0xFE)
			usVal = (uint16_t)(gRealData.vehiclestatedata.AtmosphericPressure);
	    bPackBuf[Idx++] = (uint8_t)usVal;
	
	    PackLen = Idx;
	}
	return PackLen;
}
/*���������ϵͳ����*/
static uint16_t PackOnBoardHydSys(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, usVal = 0;
	int16_t sVal = 0;
	uint8_t len = 6;
	uint32_t UsVal=0;
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x22;
		
		//������ϵͳʣ��ѹ��
		usVal = (uint16_t)(gRealData.onboardhydsysdata.ResidualPressure);
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(gRealData.onboardhydsysdata.ResidualPressure * 10);//usVal * 100;
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
	
		//������ϵͳʣ����������
		UsVal = (uint32_t)(gRealData.onboardhydsysdata.OnboardHydrogenSysSurplusQuality);
		if(UsVal != 0xFFFFFFFF && UsVal != 0xFFFFFFFE)
			UsVal = (uint32_t)(gRealData.onboardhydsysdata.OnboardHydrogenSysSurplusQuality * 100);//usVal * 100;
		bPackBuf[Idx++] = (uint8_t)(UsVal>>24);
		bPackBuf[Idx++] = (uint8_t)(UsVal>>16);
		bPackBuf[Idx++] = (uint8_t)(UsVal>>8);
		bPackBuf[Idx++] = (uint8_t)(UsVal%256);
	
	    PackLen = Idx;
	}
	return PackLen;
}

/*���ȼ�ϵ��ϵͳ����*/
static uint16_t PackFuelCellSys(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, usVal = 0;
	uint8_t FuelBatSysCnt = 0, i = 0;
	int16_t sVal = 0;
	uint32_t UsVal = 0;
	FuelBatSysCnt = ((gRealData.FuelBatSysCnt != 0xFF && gRealData.FuelBatSysCnt != 0xFE) ? gRealData.FuelBatSysCnt : 0);	
	
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x23;
	    bPackBuf[Idx++] = gRealData.FuelBatSysCnt;
			for(i = 0; i < FuelBatSysCnt; i++)
			{
				//ȼ�ϵ��ϵͳ���
				usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellSysNum);
				if(usVal != 0xFF && usVal != 0xFE)
					usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellSysNum);
				bPackBuf[Idx++] = (uint8_t)usVal;
				
				//ȼ�ϵ��ϵͳ����״̬
				usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellSysWorkStatus) ;
				if(usVal != 0xFF && usVal != 0xFE)
					usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellSysWorkStatus);
				bPackBuf[Idx++] = (uint8_t)usVal;
	
				//ȼ�ϵ�ض��ܵ�ѹ
				usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellStackTotalVoltage) ;
				if(usVal != 0xFFFF && usVal != 0xFFFE)
					usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellStackTotalVoltage * 10);
				bPackBuf[Idx++] = (uint8_t)(usVal/256);
				bPackBuf[Idx++] = (uint8_t)(usVal%256);
	
				//ȼ�ϵ�ض��ܵ���
				usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellStackTotalCurrent) ;
				if(usVal != 0xFFFF && usVal != 0xFFFE)
					usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellStackTotalCurrent * 10);
				bPackBuf[Idx++] = (uint8_t)(usVal/256);
				bPackBuf[Idx++] = (uint8_t)(usVal%256);
	
				//�����ȴҺ�����¶�
				usVal = (uint16_t)(gRealData.fuelcellsysdata[i].ReactorCoolantOutletTemperature) ;
				if(usVal != 0xFFFF && usVal != 0xFFFE)
					usVal = (uint16_t)(gRealData.fuelcellsysdata[i].ReactorCoolantOutletTemperature + 40);
				bPackBuf[Idx++] = (uint8_t)(usVal/256);
				bPackBuf[Idx++] = (uint8_t)(usVal%256);
	
				//��С��ز�����Ԫ�ķ���
				UsVal = (uint32_t)(gRealData.fuelcellsysdata[i].MinCellMeasuredVarianceCells);
				if(UsVal != 0xFFFFFFFF && UsVal != 0xFFFFFFFE)
					UsVal = (uint32_t)(gRealData.fuelcellsysdata[i].MinCellMeasuredVarianceCells * 10);
				bPackBuf[Idx++] = (uint8_t)(UsVal>>24);
				bPackBuf[Idx++] = (uint8_t)(UsVal>>16);
				bPackBuf[Idx++] = (uint8_t)(UsVal>>8);
				bPackBuf[Idx++] = (uint8_t)(UsVal%256);

				//��С��ز�����Ԫ����͵�ѹ
				UsVal = (uint32_t)(gRealData.fuelcellsysdata[i].MinBatteryMeasuringVoltage);
				if(UsVal != 0xFFFFFFFF && UsVal != 0xFFFFFFFE)
					UsVal = (uint32_t)(gRealData.fuelcellsysdata[i].MinBatteryMeasuringVoltage * 1000);
				bPackBuf[Idx++] = (uint8_t)(UsVal>>24);
				bPackBuf[Idx++] = (uint8_t)(UsVal>>16);
				bPackBuf[Idx++] = (uint8_t)(UsVal>>8);
				bPackBuf[Idx++] = (uint8_t)(UsVal%256);

        //��С��ز�����Ԫ�ĵ�ѹ��ƽ��ֵ
				UsVal = (uint32_t)(gRealData.fuelcellsysdata[i].MinBatteryMeasurPressureAverage);
				if(UsVal != 0xFFFFFFFF && UsVal != 0xFFFFFFFE)
					UsVal = (uint32_t)(gRealData.fuelcellsysdata[i].MinBatteryMeasurPressureAverage * 1000);
				bPackBuf[Idx++] = (uint8_t)(UsVal>>24);
				bPackBuf[Idx++] = (uint8_t)(UsVal>>16);
				bPackBuf[Idx++] = (uint8_t)(UsVal>>8);
				bPackBuf[Idx++] = (uint8_t)(UsVal%256);

        //����ѹ��������������������ѹ
				usVal = (uint16_t)(gRealData.fuelcellsysdata[i].AirCompMotorContrInputVoltage);
				if(usVal != 0xFFFF && usVal != 0xFFFE)
					usVal = (uint16_t)(gRealData.fuelcellsysdata[i].AirCompMotorContrInputVoltage *10);
				bPackBuf[Idx++] = (uint8_t)(usVal/256);
				bPackBuf[Idx++] = (uint8_t)(usVal%256);

        //����ѹ����������������������
				usVal = (uint16_t)(gRealData.fuelcellsysdata[i].AirCompMotorContrInputCurrent);
				if(usVal != 0xFFFF && usVal != 0xFFFE)
					usVal = (uint16_t)(gRealData.fuelcellsysdata[i].AirCompMotorContrInputCurrent *10);
				bPackBuf[Idx++] = (uint8_t)(usVal/256);
				bPackBuf[Idx++] = (uint8_t)(usVal%256);

        //ȼ�ϵ��ϵͳ�������ĸ�ߵ�ѹ
				usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellSystemVoltage);
				if(usVal != 0xFFFF && usVal != 0xFFFE)
					usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellSystemVoltage *10);
				bPackBuf[Idx++] = (uint8_t)(usVal/256);
				bPackBuf[Idx++] = (uint8_t)(usVal%256);
				
				//ȼ�ϵ��ϵͳ�������ĸ�ߵ���
				usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellSystemCurrent);
				if(usVal != 0xFFFF && usVal != 0xFFFE)
					usVal = (uint16_t)(gRealData.fuelcellsysdata[i].FuelCellSystemCurrent *10);
				bPackBuf[Idx++] = (uint8_t)(usVal/256);
				bPackBuf[Idx++] = (uint8_t)(usVal%256);
			}
	     PackLen = Idx;
	}
	return PackLen;
}

//��������쳣��������
static uint16_t PackVehicleAlarm(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, usVal = 0;
	uint8_t FuelBatSysFaultCnt = 0, i = 0, Tmp = 0;
	int16_t sVal = 0;
	uint32_t UsVal = 0;
	
	
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x24;

		//��߱����ȼ�
		usVal = (uint16_t)(gRealData.vehiclealarmdata.MaxSlarmLevel); 
		if(usVal != 0xFF && usVal != 0xFE)
			usVal = (uint16_t)(gRealData.vehiclealarmdata.MaxSlarmLevel);
		bPackBuf[Idx++] = (uint8_t)usVal;

		
		//ͨ�ñ�����־			
		UsVal |= (gRealData.vehiclealarmdata.CollisionSignalState > 0 ? 0x01 : 0x00);
		UsVal |= (gRealData.vehiclealarmdata.HydrogenBottleLowPressureAlarm > 0 ? 0x02 : 0x00);
		UsVal |= (gRealData.vehiclealarmdata.HydrogenBottleHighPressureAlarm > 0 ? 0x04 : 0x00);
		UsVal |= (gRealData.vehiclealarmdata.HydrogenBottleHighTemperatureAlarm > 0 ? 0x08 : 0x00);
		UsVal |= (gRealData.vehiclealarmdata.HydrogenBottleLowTemperatureAlarm > 0 ? 0x10 : 0x00);
		UsVal |= (gRealData.vehiclealarmdata.HighHydrogenConcentrationAlarm > 0 ? 0x20: 0x00);
		UsVal |= (gRealData.vehiclealarmdata.FuelCellStackOvertemperatureAlarm > 0 ? 0x40 : 0x00);
		UsVal |= (gRealData.vehiclealarmdata.FuelCellOutputPowerLimitAlarm > 0 ? 0x80 : 0x00);
		UsVal |= (gRealData.vehiclealarmdata.FuelCellSystemFailureAlarm > 0 ? 0x100 : 0x00);
		
        bPackBuf[Idx++] = (uint8_t)(UsVal >>24);
        bPackBuf[Idx++] = (uint8_t)(UsVal >>16);
        bPackBuf[Idx++] = (uint8_t)(UsVal >>8);
        bPackBuf[Idx++] = (uint8_t)(UsVal >>0);

	
		FuelBatSysFaultCnt	= ((gRealData.vehiclealarmdata.fuelBatSysFaultCnt != 0xFF && gRealData.FuelBatSysCnt != 0xFE) ? gRealData.vehiclealarmdata.fuelBatSysFaultCnt : 0);
       	bPackBuf[Idx++] = gRealData.vehiclealarmdata.fuelBatSysFaultCnt;

		for(i = 0;i<FuelBatSysFaultCnt;i++)
		{
			UsVal = (uint32_t)(gRealData.vehiclealarmdata.FuelCellSysFaultList[i]);
			if(UsVal != 0xFFFFFFFF && UsVal != 0xFFFFFFFE)
				UsVal = (uint32_t)(gRealData.vehiclealarmdata.FuelCellSysFaultList[i]);
			bPackBuf[Idx++] = (uint8_t)(UsVal & 0xFF000000)>>24;
			bPackBuf[Idx++] = (uint8_t)(UsVal & 0x00FF0000)>>16;
			bPackBuf[Idx++] = (uint8_t)(UsVal & 0x0000FF00)>>8;
			bPackBuf[Idx++] = (uint8_t)UsVal;
		}
		PackLen = Idx;
	}
	return PackLen;
}

//��������¶�����
static uint16_t PackDirectionSlope(RealData *pRealData,uint8_t *bPackBuf)
{   
	uint16_t PackLen = 0, Idx = 0, usVal = 0;
	int16_t sVal = 0;
	if(bPackBuf != 0)
	{
    bPackBuf[Idx++] = 0x1f;
		//����
		usVal = (uint16_t)(gRealData.directionslope.direction); 
//		usVal = (uint16_t)(g_tGPS.TrackDegTrue);
		if(usVal != 0xFFFF && usVal != 0xFFFE)
		  usVal = (uint16_t)(g_tGPS.TrackDegTrue * 0.1);
		bPackBuf[Idx++] = (uint8_t)(usVal >>8);
		bPackBuf[Idx++] = (uint8_t)(usVal >>8);
		
		//�¶�
		usVal = (uint16_t)(gRealData.directionslope.slope);
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)(gRealData.directionslope.slope+30);
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
	
		PackLen = Idx;
	}		
	return PackLen;
}
/*******************************************�Ϻ��ر�����****************************************��*/

/*��*****************************************��ȼ�ϵ������******************************************/
static uint16_t PackHydrogenFuelCell(RealData *pRealData,uint8_t *bPackBuf)
{	
	uint16_t PackLen = 0, Idx = 0, usVal = 0;
	int16_t sVal = 0;
	if(bPackBuf != 0)
	{
    bPackBuf[Idx++] = 0x30;
    bPackBuf[Idx++] = 0;
    bPackBuf[Idx++] = 0x09;

		//����¶�
		usVal = (uint8_t)(gRealData.hydrogenfuelcelldata.ElectricTem);
		if(usVal != 0xFF && usVal != 0xFE)
			usVal = (uint8_t)((gRealData.hydrogenfuelcelldata.ElectricTem+40)*1);
		bPackBuf[Idx++] = (uint8_t)usVal;
			
		//����ѹ������ѹ
		usVal = (uint16_t)(gRealData.hydrogenfuelcelldata.AirComVol);
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)((gRealData.hydrogenfuelcelldata.AirComVol+0)*10);
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
		
		//����ѹ��������
		usVal = (uint16_t)(gRealData.hydrogenfuelcelldata.AirComCur);
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)((gRealData.hydrogenfuelcelldata.AirComCur+1000)*10);
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);

		//����ѭ���õ�ѹ
		usVal = (uint16_t)(gRealData.hydrogenfuelcelldata.HyCyclePumpVol);
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)((gRealData.hydrogenfuelcelldata.HyCyclePumpVol+0)*10);
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);

		//����ѭ���õ���
		usVal = (uint16_t)(gRealData.hydrogenfuelcelldata.HyCyclePumpCur);
		if(usVal != 0xFFFF && usVal != 0xFFFE)
			usVal = (uint16_t)((gRealData.hydrogenfuelcelldata.HyCyclePumpCur+1000)*10);
		bPackBuf[Idx++] = (uint8_t)(usVal/256);
		bPackBuf[Idx++] = (uint8_t)(usVal%256);
		
		PackLen = Idx;
	}		
	return PackLen;
}
/******************************************��ȼ�ϵ������*****************************************��*/

/*�����������*/
uint16_t Pack81Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	int16_t sVal = 0;
	
	SelfData81*  p81Data = (SelfData81*)&pRealData->externData[p81offset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x81;
        bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x07;

		
		//��ѹ1
		u8Val = (uint8_t)(p81Data->AirPressure1);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)((p81Data->AirPressure1)/4);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//��ѹ2
		u8Val = (uint8_t)(p81Data->AirPressure2);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)((p81Data->AirPressure2)/4);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//���ص�ѹ
		u8Val = (uint8_t)(p81Data->BatteryVoltage);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)((p81Data->BatteryVoltage)*2);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//ǰ��״̬
		u8Val |= (uint8_t)((p81Data->FrontDoorStatus)<<0);
		//����״̬
		u8Val |= (uint8_t)((p81Data->MiddleDoorStatus)<<2);
		//����״̬
		u8Val |= (uint8_t)((p81Data->RearDoorStatus)<<4);
		//�����״̬
		u8Val |= (uint8_t)((p81Data->RearHatchStatus)<<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;
		
		//��ɲ�ź�
		u8Val |= (uint8_t)((p81Data->HandBrakeSignal)<<0);
		//��ɲ�ź�
		u8Val |= (uint8_t)((p81Data->FootBrakeSignal)<<2);
		//Կ��λ��
		u8Val |= (uint8_t)((p81Data->KeyLocation)<<5);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	
			u8Val = 0;	//��ձ���
		
		//��ת���
		u8Val |= (uint8_t)((p81Data->LeftTurnSignal)<<0);
		//��ת���
		u8Val |= (uint8_t)((p81Data->RightTurnSignal)<<2);
		//λ�õ�
		u8Val |= (uint8_t)((p81Data->PositionLight)<<4);
		//�����
		u8Val |= (uint8_t)((p81Data->NearLightLamp)<<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	
			u8Val = 0;	//��ձ���
		
		//Զ���
		u8Val |= (uint8_t)((p81Data->HighBeamLamp)<<0);
		//ǰ���
		u8Val |= (uint8_t)((p81Data->FrontFogLight)<<2);
		//�����
		u8Val |= (uint8_t)((p81Data->RearFogLamp)<<4);
		//���
		u8Val |= (uint8_t)((p81Data->Wiper)<<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	
			u8Val = 0;	//��ձ���
		
		PackLen = Idx;
	}		
	return PackLen;
}

/*�����������������*/
uint16_t Pack82Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	int16_t sVal = 0;
	
	SelfData82*  p82Data = (SelfData82*)&pRealData->externData[p82offset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x82;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x05;
		
		//Ԥ������ź�
		u8Val |= (uint8_t)((p82Data->PrechargeControlSignal)<<0);
		//���Ӵ����������ź�
		u8Val |= (uint8_t)((p82Data->MainContactorPositiveControlSignal)<<2);
		//���Ӵ��������ź�
		u8Val |= (uint8_t)((p82Data->MainContactorFeedbackSignal)<<4);
		//���Ӵ����������ź�
		u8Val |= (uint8_t)((p82Data->MainContactorNegativeControlSignal)<<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;
		
		//���Ӵ����������ź�
		u8Val |= (uint8_t)((p82Data->MainContactorNegativeFeedbackSignal)<<0);
		//�����µ�����
		u8Val |= (uint8_t)((p82Data->EmergencyPoweroffRequest)<<2);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	
			u8Val = 0;	//��ձ���
		
		//������
		bPackBuf[Idx++] = (uint8_t)(p82Data->FaultCode);
		
		//������
		bPackBuf[Idx++] = (uint8_t)(p82Data->WarningCode);
		
		//����ϵͳ������
		bPackBuf[Idx++] = (uint8_t)(p82Data->DriveSystemFaultCode);
		
		PackLen = Idx;
	}		
	return PackLen;
}


/*��������������*/
uint16_t Pack83Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;

    int8_t a = 0;
	uint32_t u32Val = 0;
//    SelfData83  p83Data_temp;
//    = &p83Data_temp;
//	memcpy(&p83Data, &pRealData->externData[p83offset],sizeof(SelfData83));
//	 SelfData83  *p83Data = (SelfData83*)&pRealData->externData[p83offset];	
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x83;
		bPackBuf[Idx++] = 0;
        bPackBuf[Idx++] = 0x1C;
		//SOH
        
		u8Val = (uint8_t)(pSelfData83.SOH);
//        u8Val = (uint8_t)(SOH);
//        u8Val = data_83.SOH;
   
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)((pSelfData83.SOH)*2.5);
		bPackBuf[Idx++] = (uint8_t)u8Val;

		//����������ŵ����
		u16Val = (uint16_t)(pSelfData83.batteryMaximumAllowableDischargeCurrent);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (uint16_t)((pSelfData83.batteryMaximumAllowableDischargeCurrent)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//���������������
		u16Val = (uint16_t)(pSelfData83.batteryMaximumAllowableChargeCurrent);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (uint16_t)((pSelfData83.batteryMaximumAllowableChargeCurrent)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//��ذ�������Եֵ
		u16Val = (uint16_t)(pSelfData83.batteryPackPositiveInsulation);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (uint16_t)(pSelfData83.batteryPackPositiveInsulation);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//��ذ�������Եֵ
		u16Val = (uint16_t)(pSelfData83.batteryPackNegativeInsulation);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (uint16_t)(pSelfData83.batteryPackNegativeInsulation);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//��ص�ѹ�����̵����ڲࣩ
		u16Val = (uint16_t)(pSelfData83.BatteryVoltageInsideMainRelay);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (uint16_t)((pSelfData83.BatteryVoltageInsideMainRelay)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
		
		//��ص�ѹ�����̵�����ࣩ
		u16Val = (uint16_t)(pSelfData83.BatteryVoltageOutsideMainRelay);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (uint16_t)((pSelfData83.BatteryVoltageOutsideMainRelay)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
		
		//�ۻ�������
		u32Val = (uint32_t)(pSelfData83.AccumulatedChargeCapacity);
		if(u32Val != 0xFFFFFFFF && u32Val != 0xFFFFFFFE)
			u32Val = (uint32_t)((pSelfData83.AccumulatedChargeCapacity)*10);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>24);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>16);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>8);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>0);
		
		//�ۻ��ŵ����
		u32Val = (uint32_t)(pSelfData83.AccumulatedDischargeCapacity);
		if(u32Val != 0xFFFFFFFF && u32Val != 0xFFFFFFFE)
	    u32Val = (uint32_t)((pSelfData83.AccumulatedDischargeCapacity)*10);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>24);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>16);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>8);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>0);
		
		//�����������
		u32Val = (uint32_t)(pSelfData83.MotorFeedbackQuantity);
		if(u32Val != 0xFFFFFFFF && u32Val != 0xFFFFFFFE)
			u32Val = (uint32_t)((pSelfData83.MotorFeedbackQuantity)*10);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>24);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>16);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>8);
		bPackBuf[Idx++] = (uint8_t)(u32Val>>0);
		
		//�������
		u16Val = (uint16_t)(pSelfData83.RangeDriving);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (uint16_t)((pSelfData83.RangeDriving)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
		
		//�����µ�����
		u8Val |= (uint8_t)((pSelfData83.EmergencyPoweroffRequest)<<0);
		//��ؾ���״̬
		u8Val |= (uint8_t)((pSelfData83.BatteryEquilibriumState)<<2);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;
		
		PackLen = Idx;
	}		
	return PackLen;
}

/*��� DCDC ����*/
uint16_t Pack84Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData84*  p84Data = (SelfData84*)&pRealData->externData[p84offset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x84;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x09;


		//DC/DC �������
		u16Val = (pSelfData84.DCDC_OutputCurrent);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData84.DCDC_OutputCurrent)*10)+1000);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//DC/DC �����ѹ
		u16Val = (pSelfData84.DCDC_OutputVoltage);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData84.DCDC_OutputVoltage)*10)+1000);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//DC/DC �����ѹ
		u16Val = (pSelfData84.DCDC_InputVoltage);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData84.DCDC_InputVoltage)*10)+1000);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//DC/DC ɢ�����¶�
		u16Val = (pSelfData84.DCDC_HeatSinkTemperature);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (pSelfData84.DCDC_HeatSinkTemperature+40);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//DC/DC ʹ���ź�
		bPackBuf[Idx++] = (uint8_t)(pSelfData84.DCDC_EnableSignal);

		PackLen = Idx;
	}		
	return PackLen;
}

/*������� DC/AC ����*/
uint16_t Pack85Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData85*  p85Data = (SelfData85*)&pRealData->externData[p85offset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x85;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x09;


		//���� DC/AC U ���������
		u16Val = (pSelfData85.AirPumpDCAC_U_PhaseOutputCurrent);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData85.AirPumpDCAC_U_PhaseOutputCurrent)*10)+1000);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//���� DC/AC V ���������
		u16Val = (pSelfData85.AirPumpDCAC_V_PhaseOutputCurrent);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData85.AirPumpDCAC_V_PhaseOutputCurrent)*10)+1000);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//����DC/AC W ���������
		u16Val = (pSelfData85.AirPumpDCAC_W_PhaseOutputCurrent);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData85.AirPumpDCAC_W_PhaseOutputCurrent)*10)+1000);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//���� DC/AC ɢ�����¶�
		u16Val = (pSelfData85.AirPump_DCAC_HeatSinkTemperature);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (pSelfData85.AirPump_DCAC_HeatSinkTemperature+40);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//���� DC/AC ʹ���ź�
		bPackBuf[Idx++] = (uint8_t)(pSelfData85.AirPump_DCAC_EnableSignal);

		PackLen = Idx;
	}		
	return PackLen;
}

/*����ͱ� DC/AC ����*/
uint16_t Pack86Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData86*  p86Data = (SelfData86*)&pRealData.externData[p86offset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x86;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x09;


		//�ͱ� DC/AC U ���������
		u16Val = (pSelfData86.OilPumpDCAC_U_PhaseOutputCurrent);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData86.OilPumpDCAC_U_PhaseOutputCurrent)*10)+1000);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//�ͱ� DC/AC V ���������
		u16Val = (pSelfData86.OilPumpDCAC_V_PhaseOutputCurrent);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData86.OilPumpDCAC_V_PhaseOutputCurrent)*10)+1000);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//�ͱ� DC/AC W ���������
		u16Val = (pSelfData86.OilPumpDCAC_W_PhaseOutputCurrent);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData86.OilPumpDCAC_W_PhaseOutputCurrent)*10)+1000);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//�ͱ� DC/AC ɢ�����¶�
		u16Val = (pSelfData86.OilPump_DCAC_HeatSinkTemperature);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (pSelfData86.OilPump_DCAC_HeatSinkTemperature+40);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//�ͱ� DC/AC ʹ���ź�
		bPackBuf[Idx++] = (uint8_t)(pSelfData86.OilPump_DCAC_EnableSignal);

		PackLen = Idx;
	}		
	return PackLen;
}

/*����յ�����*/
uint16_t Pack87Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
////	SelfData87*  p87Data = (SelfData87*)&pRealData.externData[p87offset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x87;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x04;
        
		//�յ�����
		u8Val |= (uint8_t)((pSelfData87.AirConditionerLowSpeed)<<0);
		//�յ�����
		u8Val |= (uint8_t)((pSelfData87.AirConditioningMediumSpeed)<<1);
		//�յ�����
		u8Val |= (uint8_t)((pSelfData87.AirConditionerHighSpeed)<<2);
		//�յ�����
		u8Val |= (uint8_t)((pSelfData87.AirConditioningHeating)<<3);
		//�յ����� 1 ��˪
		u8Val |= (uint8_t)((pSelfData87.AirConditioningRefrigerationDefrosting1)<<4);
		//�յ��·�
		u8Val |= (uint8_t)((pSelfData87.AirConditioningFreshAir)<<5);
		//�յ�ɱ��
		u8Val |= (uint8_t)((pSelfData87.AirConditioningSterilization)<<6);
		//�յ����� 2 ��˪
		u8Val |= (uint8_t)((pSelfData87.AirConditioningRefrigerationDefrosting2)<<7);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;
		
		//�յ����� 2
		u8Val |= (uint8_t)((pSelfData87.AirConditioningRefrigeration2)<<0);
		//�յ����� 1
		u8Val |= (uint8_t)((pSelfData87.AirConditioningRefrigeration1)<<1);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;
		
		//�����¶�
		u8Val = (uint8_t)(pSelfData87.InsideCarTemperature);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData87.InsideCarTemperature+30);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//�����¶�
		u8Val = (uint8_t)(pSelfData87.OutsideCarTemperature);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData87.OutsideCarTemperature+30);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		PackLen = Idx;
	}		
	return PackLen;
}

/*�������������*/
uint16_t Pack88Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData88*  p88Data = (SelfData88*)&pRealData.externData[p88offset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x88;
        bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x01;
		
		//��ѹ��״̬
		u8Val |= (uint8_t)((pSelfData88.LubricationPressureState)<<0);
		//����λ״̬
		u8Val |= (uint8_t)((pSelfData88.LubricatingOilLevelState)<<2);
		//�󻬵��״̬
		u8Val |= (uint8_t)((pSelfData88.LubricatingMotorState)<<4);
		//��ϵͳ״̬
		u8Val |= (uint8_t)((pSelfData88.LubricationSystemStatus)<<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;
		
		
		PackLen = Idx;
	}		
	return PackLen;
}

/*�����������������*/
uint16_t Pack89Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData89*  p89Data = (SelfData89*)&pRealData.externData[p89offset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x89;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x04;
        
		//������ˮ��
		u8Val = (uint8_t)(pSelfData89.EngineWaterTemperature);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData89.EngineWaterTemperature+40);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//����������ѹ��
		u8Val = (uint8_t)(pSelfData89.EngineOilPressure);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData89.EngineOilPressure/4);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//ʣ������
		u8Val = (uint8_t)(pSelfData89.RemainingOilAmount);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData89.RemainingOilAmount*25);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//����Һλ
		u8Val = (uint8_t)(pSelfData89.UreaLevel);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData89.UreaLevel*25);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		
		PackLen = Idx;
	}		
	return PackLen;
}

/*���̥ѹ�������*/
uint16_t Pack8AData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData8A*  p8AData = (SelfData8A*)&pRealData.externData[p8Aoffset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x8A;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x12;
        
		//��ǰ��̥ѹ��
		u8Val = (uint8_t)(pSelfData8A.LeftFrontTirePressure);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8A.LeftFrontTirePressure/4);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		// ��ǰ��̥�¶�
		u16Val = (pSelfData8A.LeftFrontTireTemperature);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8A.LeftFrontTireTemperature)+40);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//��ǰ��̥ѹ��
		u8Val = (uint8_t)(pSelfData8A.RightFrontTirePressure);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8A.RightFrontTirePressure/4);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//��ǰ��̥�¶�
		u16Val = (pSelfData8A.RightFrontTireTemperature);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (pSelfData8A.RightFrontTireTemperature+40);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//��� 1 ��̥ѹ��
		u8Val = (uint8_t)(pSelfData8A.LeftRear1TirePressure);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8A.LeftRear1TirePressure/4);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//��� 1 ��̥�¶�
		u16Val = (pSelfData8A.LeftRear1TireTemperature);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8A.LeftRear1TireTemperature)+40);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//��� 2 ��̥ѹ��
		u8Val = (uint8_t)(pSelfData8A.LeftRear2TirePressure);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8A.LeftRear2TirePressure/4);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//��� 2 ��̥�¶�
		u16Val = (pSelfData8A.LeftRear2TireTemperature);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8A.LeftRear2TireTemperature)+40);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//�Һ� 1 ��̥ѹ��
		u8Val = (uint8_t)(pSelfData8A.RightRear1TirePressure);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8A.RightRear1TirePressure/4);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//�Һ� 1 ��̥�¶�
		u16Val = (pSelfData8A.RightRear1TireTemperature);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8A.RightRear1TireTemperature)+40);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//�Һ� 2 ��̥ѹ��
		u8Val = (uint8_t)(pSelfData8A.RightRear2TirePressure);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8A.RightRear2TirePressure/4);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//�Һ� 2 ��̥�¶�
		u16Val = (pSelfData8A.RightRear2TireTemperature);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8A.RightRear2TireTemperature)+40);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		
		PackLen = Idx;
	}		
	return PackLen;
}

/*��������ȴϵͳ TMS ����*/
uint16_t Pack8BData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData8B*  p8BData = (SelfData8B*)&pRealData.externData[p8Boffset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x8B;
        bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x04;

		
		//TMS ����״̬
		u8Val |= (uint8_t)((pSelfData8B.TMS_OperatingStatus)<<0);
		//TMS ��ѹ�̵���״̬
		u8Val |= (uint8_t)((pSelfData8B.TMS_HighVoltageRelayStatus)<<2);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;

		//��ˮ�¶�
		u8Val = (uint8_t)(pSelfData8B.WaterOutletTemperature);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8B.WaterOutletTemperature+40);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//��ˮ�¶�
		u8Val = (uint8_t)(pSelfData8B.WaterInletTemperature);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8B.WaterInletTemperature+40);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//TMS ������
		bPackBuf[Idx++] = (uint8_t)(pSelfData8B.TMS_FaultCode);
		
		PackLen = Idx;
	}		
	return PackLen;
}

/*����������*/
uint16_t Pack8CData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData8C*  p8CData = (SelfData8C*)&pRealData.externData[p8Coffset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x8C;
        bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x11;

		
		//DC/DC ˲ʱ����
		u8Val = (uint8_t)(pSelfData8C.DCDC_InstantaneousPower);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8C.DCDC_InstantaneousPower*10);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//ת���ͱ�˲ʱ����
		u8Val = (uint8_t)(pSelfData8C.SteeringOilPumpInstantaneousPower);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8C.SteeringOilPumpInstantaneousPower*10);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//����˲ʱ����
		u8Val = (uint8_t)(pSelfData8C.AirPumpInstantaneousPower);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8C.AirPumpInstantaneousPower*10);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//����ϵͳ˲ʱ����
		u16Val = (pSelfData8C.DriveSystemInstantaneousPower);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData8C.DriveSystemInstantaneousPower)*10)-1000);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
		
		//����ϵͳʣ�๦��
		u16Val = (pSelfData8C.DriveSystemRemainingPower);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8C.DriveSystemRemainingPower)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
		
		// DC/DC ���
		u16Val = (pSelfData8C.DCDC_Power_Consumption);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8C.DCDC_Power_Consumption)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
		
		//ת���ͱõ��
		u16Val = (pSelfData8C.PowerConsumptionSteeringOilPump);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8C.PowerConsumptionSteeringOilPump)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
		
		//���õ��
		u16Val = (pSelfData8C.AirPumpPowerConsumption);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8C.AirPumpPowerConsumption)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
	
		//����ϵͳ���
		u16Val = (pSelfData8C.DriveSystemPowerConsumption);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8C.DriveSystemPowerConsumption)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
		
		//��ʻ���
		u16Val = (pSelfData8C.RangeDriving);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData8C.RangeDriving)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
		
		PackLen = Idx;
	}		
	return PackLen;
}

/*������˪����*/
uint16_t Pack8DData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData8D*  p8DData = (SelfData8D*)&pRealData.externData[p8Doffset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x8D;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x06;

		//��˪���ڸ�ѹ�Ӵ���״̬
		u8Val |= (uint8_t)((pSelfData8D.DefrosterHighPressureContactorStatus)<<0);
		//���״̬
		u8Val |= (uint8_t)((pSelfData8D.FanStatus)<<2);
		//���˪ʹ��
		u8Val |= (uint8_t)((pSelfData8D.DefrostEnable)<<4);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;
		
		// �������¶�
		u8Val = (uint8_t)(pSelfData8D.HeatingBodyTemperature);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8D.HeatingBodyTemperature-40);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//��˪������¶�
		u8Val = (uint8_t)(pSelfData8D.DefrosterEnclosureTemperature);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8D.DefrosterEnclosureTemperature-40);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//��˪�������¶�
		u8Val = (uint8_t)(pSelfData8D.DefrosterOutletTemperature);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8D.DefrosterOutletTemperature-40);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//���������
		u8Val = (uint8_t)(pSelfData8D.HeatingBodyCurrent);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8D.HeatingBodyCurrent*10);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//��˪����������
		u8Val = (uint8_t)(pSelfData8D.DefrosterWorkingPower);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8D.DefrosterWorkingPower*10);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		PackLen = Idx;
	}		
	return PackLen;
}

/*������Ӳ���������*/
uint16_t Pack8EData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData8E*  p8EData = (SelfData8E*)&pRealData.externData[p8Eoffset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x8E;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x02;
        
		//EDC ת���־
		bPackBuf[Idx++] = (uint8_t)(pSelfData8E.EDC_TurnSign);
		
		//EDC ����ģʽ
		bPackBuf[Idx++] = (uint8_t)(pSelfData8E.EDC_WorkingMode);
		
		PackLen = Idx;
	}		
	return PackLen;
}

/*���������������*/
uint16_t Pack8FData(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData8F*  p8FData = (SelfData8F*)&pRealData.externData[p8Foffset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x8F;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x0E;
		
		//����ۼƴ���
		bPackBuf[Idx++] = (uint8_t)((pSelfData8F.ChargeCumulativeNumber)/256);
		bPackBuf[Idx++] = (uint8_t)((pSelfData8F.ChargeCumulativeNumber)%256);
		
		//�ͱ� DCAC �ߵ�ѹ
		bPackBuf[Idx++] = (uint8_t)((pSelfData8F.OilPumpDCAC_LineVoltage)/256);
		bPackBuf[Idx++] = (uint8_t)((pSelfData8F.OilPumpDCAC_LineVoltage)%256);
		
		//���� DCAC �ߵ�ѹ
		bPackBuf[Idx++] = (uint8_t)((pSelfData8F.AirPumpDCAC_LineVoltage)/256);
		bPackBuf[Idx++] = (uint8_t)((pSelfData8F.AirPumpDCAC_LineVoltage)%256);
		
		//�յ�˲ʱ����
		u8Val = (uint8_t)(pSelfData8F.AirConditioningInstantaneousPower);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8F.AirConditioningInstantaneousPower*10);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//��ǰ����̥װ��״̬
		u8Val |= (uint8_t)((pSelfData8F.LeftFrontExplosionproofTireDeviceStatus)<<0);
		//��ǰ����̥װ��״̬
		u8Val |= (uint8_t)((pSelfData8F.RightFrontExplosionproofTireDeviceStatus)<<1);
		//��� 1 ����̥װ��״̬
		u8Val |= (uint8_t)((pSelfData8F.LeftRear1ExplosionproofTireDeviceStatus)<<2);
		//�Һ� 1 ����̥װ��״̬
		u8Val |= (uint8_t)((pSelfData8F.LeftRear2ExplosionproofTireDeviceStatus)<<3);
		//�Һ� 1 ����̥װ��״̬
		u8Val |= (uint8_t)((pSelfData8F.RightRear1ExplosionproofTireDeviceStatus)<<4);
		//�Һ� 2 ����̥װ��״̬
		u8Val |= (uint8_t)((pSelfData8F.RightRear2ExplosionproofTireDeviceStatus)<<5);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;
		
		//ǰ�����ƶ���Ƭʣ����
		u8Val = (uint8_t)(pSelfData8F.DrontAxleLeftBrakeShoeResidualAmount);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8F.DrontAxleLeftBrakeShoeResidualAmount*25);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//ǰ�����ƶ���Ƭʣ����
		u8Val = (uint8_t)(pSelfData8F.DrontAxleRightBrakeShoeResidualAmount);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8F.DrontAxleRightBrakeShoeResidualAmount*25);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//���� 1 ���ƶ���Ƭʣ����
		u8Val = (uint8_t)(pSelfData8F.RearAxle1LeftBrakeShoeRemainingAmount);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8F.RearAxle1LeftBrakeShoeRemainingAmount*25);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//���� 1 ���ƶ���Ƭʣ����
		u8Val = (uint8_t)(pSelfData8F.RearAxle1RightBrakeShoeRemainingAmount);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)(pSelfData8F.RearAxle1RightBrakeShoeRemainingAmount*25);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		
		//��ذ�����
		bPackBuf[Idx++] = (uint8_t)(pSelfData8F.TotalBatteryPacks);
		
		//�յ��趨�¶�
		u8Val = (uint8_t)(pSelfData8F.AirConditioningSettingTemperature);
		if(u8Val != 0xFF && u8Val != 0xFE)
			u8Val = (uint8_t)((pSelfData8F.AirConditioningSettingTemperature+45)*20);
		bPackBuf[Idx++] = (uint8_t)u8Val;

		PackLen = Idx;
	}		
	return PackLen;
}

/*�������������ϵͳ����*/
uint16_t Pack90Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData90*  p90Data = (SelfData90*)&pRealData.externData[p90offset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x90;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x04;
        
		//̽������ţ������ţ�
		bPackBuf[Idx++] = (uint8_t)(pSelfData90.DetectorNum);
		//�𾯼���
		bPackBuf[Idx++] = (uint8_t)(pSelfData90.FireLevel);
		//ϵͳ״̬
		bPackBuf[Idx++] = (uint8_t)(pSelfData90.SystemStatus);
		//�ӷ���������״̬
		bPackBuf[Idx++] = (uint8_t)(pSelfData90.SubvalveControlCommandStatus);
				
		PackLen = Idx;
	}		
	return PackLen;
}

/*���ADAS����*/
uint16_t Pack91Data(RealData *pRealData,uint8_t *bPackBuf)
{
	uint16_t PackLen = 0, Idx = 0, u16Val = 0;
	uint8_t u8Val = 0;
	uint32_t u32Val = 0;
	
//	SelfData91*  p91Data = (SelfData91*)&pRealData.externData[p91offset];		
	//����Զ�������
	if(bPackBuf != 0)
	{
		bPackBuf[Idx++] = 0x91;
		bPackBuf[Idx++] = 0;
		bPackBuf[Idx++] = 0x0D;
		
		//����ƫ��Ԥ��״̬
		u8Val |= (uint8_t)((pSelfData91.LaneDepartureWarningStatus)<<0);
		//ǰײԤ��״̬
		u8Val |= (uint8_t)((pSelfData91.ForwardCollisionWarningStatus)<<2);
		//AEB ����״̬��ʾ
		u8Val |= (uint8_t)((pSelfData91.AEB_WorkingStatusAlert)<<4);
		//����Ԥ����־
		u8Val |= (uint8_t)((pSelfData91.VehicleWarningSign)<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;

		//��ǰ������ٶ�
		u16Val = (pSelfData91.FrontVehicleRelativeSpeed);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = (((pSelfData91.FrontVehicleRelativeSpeed)+250)*128);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);
		
		//��ǰ������
		u16Val = (pSelfData91.FrontVehicleRelativeDistance);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData91.FrontVehicleRelativeDistance)*10);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//AEBS ����״̬
		u8Val |= (uint8_t)((pSelfData91.AEBS_FaultStatus)<<0);
		//������������
		u8Val |= (uint8_t)((pSelfData91.PrimaryControllerFailure)<<2);
		//ͨѶ���ϣ��복��ͨѶ��
		u8Val |= (uint8_t)((pSelfData91.CommunicationFailure)<<4);
		//����Ԥ����־
		u8Val |= (uint8_t)((pSelfData91.PedestrianWarningSigns)<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;

		//ʱ����ֵ
		u16Val = (pSelfData91.TimeIntervalValue);
		if(u16Val != 0xFFFF && u16Val != 0xFFFE)
			u16Val = ((pSelfData91.TimeIntervalValue)*100);
		bPackBuf[Idx++] = (uint8_t)(u16Val/256);
		bPackBuf[Idx++] = (uint8_t)(u16Val%256);

		//����ƫ��ϵͳ����״̬
		u8Val |= (uint8_t)((pSelfData91.LaneDepartureSystemWorkingStatus)<<0);
		//��������ϵͳ����״̬
		u8Val |= (uint8_t)((pSelfData91.LaneKeepingSystemWorkingStatus)<<3);
		//ͼ�񴫸�������
		u8Val |= (uint8_t)((pSelfData91.ImageSensorFault)<<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;

		//����ӦѲ��ϵͳ����״̬
		u8Val |= (uint8_t)((pSelfData91.AdaptiveCruiseSystemOperatingState)<<0);
		//ǰ����ײԤ��ϵͳ����״̬
		u8Val |= (uint8_t)((pSelfData91.ForwardCollisionWarningSystemOperatingStatus)<<3);
		//ͼ�񴫸���ͨѶ����
		u8Val |= (uint8_t)((pSelfData91.ImageSensorCommunicationFaulty)<<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;

		//��ײ����ϵͳ����״̬
		u8Val |= (uint8_t)((pSelfData91.CollisionMitigationSystemWorkingStatus)<<0);
		//���ŷ����ϵͳ����״̬
		u8Val |= (uint8_t)((pSelfData91.ThrottleAntimisstepSystemWorkingStatus)<<3);
		//��������������
		u8Val |= (uint8_t)((pSelfData91.AuxiliaryControllerFailure)<<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;

		//ͼ�񴫸�������״̬
		u8Val |= (uint8_t)((pSelfData91.ImageSensorWorkingStatus)<<0);
		//���ײ��״﹤��״̬
		u8Val |= (uint8_t)((pSelfData91.Millimeter_waveRadarOperatingStatus)<<3);

		//����������ͨѶ����
		u8Val |= (uint8_t)((pSelfData91.AuxiliaryControllerCommunicationFaulty)<<0);
		//���ײ��״����
		u8Val |= (uint8_t)((pSelfData91.Millimeter_waveRadarFaulty)<<3);
		//���ײ��״�ͨѶ����
		u8Val |= (uint8_t)((pSelfData91.Millimeter_waveRadarCommunicationFaulty)<<6);
		bPackBuf[Idx++] = (uint8_t)u8Val;
		if(u8Val!=0)	//��ձ���
			u8Val = 0;

		//CSVU �Լ�״̬
		bPackBuf[Idx++] = (uint8_t)(pSelfData91.CSVU_self_CheckStatus);
		
		PackLen = Idx;
	}		
	return PackLen;
}



//�����Ϻ��ر�����
uint16_t extSHDBReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
		GBSTA *pSta = obj;
		uint16_t index = 0;
		
		//�����¶�����
		index += PackDirectionSlope(pRealData,&bPackBuf[index]);		
		//����״̬
		index += PackVehicleState(pRealData,&bPackBuf[index]);
		//������ϵͳ����
		index += PackOnBoardHydSys(pRealData,&bPackBuf[index]);
		//ȼ�ϵ��ϵͳ����
		index += PackFuelCellSys(pRealData,&bPackBuf[index]);
		//�����쳣��������
		index += PackVehicleAlarm(pRealData,&bPackBuf[index]);
		
		return index;
}

//������ȼ������
uint16_t extHyFuelCellReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
		GBSTA *pSta = obj;
		uint16_t index = 0;
		
		//��ȼ������
		index += PackHydrogenFuelCell(pRealData,&bPackBuf[index]);		
		
		return index;
}

//��չʵʱ���ݣ�ʵʱ���������Զ�������
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen)
{
	uint16_t index = 0;;
	index += Pack81Data(pRealData,&bPackBuf[index]);
	index += Pack82Data(pRealData,&bPackBuf[index]);
	index += Pack83Data(pRealData,&bPackBuf[index]);
	index += Pack84Data(pRealData,&bPackBuf[index]);
	index += Pack85Data(pRealData,&bPackBuf[index]);
	index += Pack86Data(pRealData,&bPackBuf[index]);
	index += Pack87Data(pRealData,&bPackBuf[index]);
	index += Pack88Data(pRealData,&bPackBuf[index]);
	index += Pack89Data(pRealData,&bPackBuf[index]);
	index += Pack8AData(pRealData,&bPackBuf[index]);
	index += Pack8BData(pRealData,&bPackBuf[index]);
	index += Pack8CData(pRealData,&bPackBuf[index]);
	index += Pack8DData(pRealData,&bPackBuf[index]);
	index += Pack8EData(pRealData,&bPackBuf[index]);
	index += Pack8FData(pRealData,&bPackBuf[index]);
	index += Pack90Data(pRealData,&bPackBuf[index]);
	index += Pack91Data(pRealData,&bPackBuf[index]);

	return index;
}
