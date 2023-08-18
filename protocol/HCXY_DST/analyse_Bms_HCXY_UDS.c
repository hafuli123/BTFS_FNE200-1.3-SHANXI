/*
�ļ���analyse_Bms_HCXY_STD.c
���ܣ�������Դ
���ڣ�2021/07/15
��˾����ɽ��Դ
���ߣ�csj
*/
#include "cmsis_os2.h" 
#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"

#include "Fun_Net.h"

#include "bsp_rtc.h"
#include "bsp_sys.h"
#include "bsp_gps.h"
#include "bsp_can.h"
#include "bsp_storage.h"


#include "analyse_Bms_UDS.h"
#include "diagnostic.h"

static uint8_t udsCanch = 1;			//UDS CANͨ��
static uint32_t recvCanCnt = 0;		//����CAN����
static uint8_t Commbit = 0;				//ͨ�ſ���λ
#define NcmTxSet(bit) Commbit &= 0xFE;Commbit |= (!bit << 0)
#define NcmRxSet(bit) Commbit &= 0xFD;Commbit |= (!bit << 1)
#define NmcmTxSet(bit) Commbit &= 0xFB;Commbit |= (!bit << 2)
#define NmcmRxSet(bit) Commbit &= 0xF7;Commbit |= (!bit << 3)

static uint32_t getUdsTime = 0;
static uint32_t tcu188Time = 0;
static uint32_t abc140Time = 0;
static uint32_t obc172Time = 0;
static uint32_t vcu01aTime = 0;
static uint32_t ic260Time = 0;
static uint32_t tpms294Time = 0;
static uint32_t bms441Time = 0;
static uint32_t dcdc193Time = 0;
static uint32_t mcu138Time = 0;
static uint32_t slowChargeTime = 0;

static uint8_t SnapshotIgnitionStatus;
static uint8_t SnapshotPowerVol;
static uint8_t SnapshotSpeed[2];
static uint8_t SnapshotTolMil[3];
static uint8_t SnapshotBcdTime[6];

static uint8_t gprsSignal[2];
static uint8_t vehCfg[4] = {0xFF,0xFF,0xFF,0xFF};
static uint8_t funCfg[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static uint8_t ECUManufacturingDate[4] = {0x20,0x21,0x09,0x30};
static char ecuId[24] = {"                        "};
static uint8_t vin1[18] = {0};
static uint8_t vin2[18] = {0};
static char ecuHw[16] = {"V01.00.00       "};
static char ecuSw[16] = {"V01.00.00       "};
static char ecuName[10] = {"TBOX-FNE  "};
static uint8_t UnintUds = 0;

TP_T types_T;
uint8_t tempVals[50] = {0};

//��ѹ�߼��
DTCTestResult checkVolHight(void)
{
	static uint32_t faultTime = 0,okTime = 0;
	if(osKernelGetTickCount() < 10000) {
		faultTime = 0;
		okTime = 0;
		return IN_TESTING;
	}
	else if(gTerminalState.pwrVolt > 16.5f){
		okTime = 0;
		if(faultTime == 0){
			faultTime = osKernelGetTickCount();
		}
		if(osKernelGetTickCount() - faultTime >= 1000){
			return FAILEDD;
		}
		return IN_TESTING;
	}
	else{
		faultTime = 0;
		if(okTime == 0){
			okTime = osKernelGetTickCount();
		}
		if(osKernelGetTickCount() - okTime >= 1000){
			return PASSED;
		}
		return IN_TESTING;
	}
}

//��ѹ�ͼ��
DTCTestResult checkVolLow(void)
{
	static uint32_t faultTime = 0,okTime = 0;
	if(osKernelGetTickCount() < 10000) {
		faultTime = 0;
		okTime = 0;
		return IN_TESTING;
	}
	else if(gTerminalState.pwrVolt < 8.5f){
		okTime = 0;
		if(faultTime == 0){
			faultTime = osKernelGetTickCount();
		}
		if(osKernelGetTickCount() - faultTime >= 1000){
			return FAILEDD;
		}
		return IN_TESTING;
	}
	else{
		faultTime = 0;
		if(okTime == 0){
			okTime = osKernelGetTickCount();
		}
		if(osKernelGetTickCount() - okTime >= 1000){
			return PASSED;
		}
		return IN_TESTING;
	}
}

//busOff���
DTCTestResult checkBusOff(void)
{
	if(osKernelGetTickCount() < 10000) {
		return IN_TESTING;
	}
	else if(CAN1->ESR & 0x04){
		return FAILEDD;
	}
	else{
		return PASSED;
	}
}

//TCU188���ļ��
DTCTestResult checkTcu188(void)
{
	if(fun_can_Get_State(BSP_CAN) == 0 || recvCanCnt < 2000) {
		return IN_TESTING;
	}
	else if(osKernelGetTickCount() - tcu188Time >= 5000){
		return FAILEDD;
	}
	else{ 
		return PASSED;
	}
}

//ABC140���ļ��
DTCTestResult checkAbc140(void)
{
	if(fun_can_Get_State(BSP_CAN) == 0 || recvCanCnt < 2000) {
		return IN_TESTING;
	}
	else if(osKernelGetTickCount() - abc140Time >= 5000){
		return FAILEDD;
	}
	else{ 
		return PASSED;
	}
}

//OBC172���ļ��
DTCTestResult checkObc172(void)
{
	if(fun_can_Get_State(BSP_CAN) == 0 || recvCanCnt < 2000 || osKernelGetTickCount() - slowChargeTime >= 1000) {
		return IN_TESTING;
	}
	else if(osKernelGetTickCount() - obc172Time >= 5000){
		return FAILEDD;
	}
	else{ 
		return PASSED;
	}
}

//VCU01A���ļ��
DTCTestResult checkVcu01a(void)
{
	if(fun_can_Get_State(BSP_CAN) == 0 || recvCanCnt < 2000) {
		return IN_TESTING;
	}
	else if(osKernelGetTickCount() - vcu01aTime >= 5000){
		return FAILEDD;
	}
	else{ 
		return PASSED;
	}
}

//IC260���ļ��
DTCTestResult checkIc260(void)
{
	if(fun_can_Get_State(BSP_CAN) == 0 || recvCanCnt < 2000) {
		return IN_TESTING;
	}
	else if(osKernelGetTickCount() - ic260Time >= 5000){
		return FAILEDD;
	}
	else{ 
		return PASSED;
	}
}

//TPMS194���ļ��
DTCTestResult checkTpms294(void)
{
	if(fun_can_Get_State(BSP_CAN) == 0 || recvCanCnt < 2000) {
		return IN_TESTING;
	}
	else if(osKernelGetTickCount() - tpms294Time >= 5000){
		return FAILEDD;
	}
	else{ 
		return PASSED;
	}
}

//BMS441���ļ��
DTCTestResult checkBms441(void)
{
	if(fun_can_Get_State(BSP_CAN) == 0 || recvCanCnt < 2000) {
		return IN_TESTING;
	}
	else if(osKernelGetTickCount() - bms441Time >= 5000){
		return FAILEDD;
	}
	else{ 
		return PASSED;
	}
}

//DCDC193���ļ��
DTCTestResult checkDcdc193(void)
{
	if(fun_can_Get_State(BSP_CAN) == 0 || recvCanCnt < 2000) {
		return IN_TESTING;
	}
	else if(osKernelGetTickCount() - dcdc193Time >= 5000){
		return FAILEDD;
	}
	else{ 
		return PASSED;
	}
}

//MCU138���ļ��
DTCTestResult checkMcu138(void)
{
	if(fun_can_Get_State(BSP_CAN) == 0 || recvCanCnt < 2000) {
		return IN_TESTING;
	}
	else if(osKernelGetTickCount() - mcu138Time >= 5000){
		return FAILEDD;
	}
	else{ 
		return PASSED;
	}
}

//GPRS���߿�·��⣬ģ�鲻֧�֣���SIM��״̬����
DTCTestResult checkGprsAntOpen(void)
{
	static uint32_t faultTime = 0;
	if(osKernelGetTickCount() < 15000 || gTerminalState.pwrVolt < 8.5f || gTerminalState.pwrVolt > 16.5f) {
		return IN_TESTING;
	}
	else if(Fun_Gprs_GetSta()< FUN_GPRS_GET_SIM){			//��SIM��
		if(faultTime == 0)
		{
			faultTime = osKernelGetTickCount();
		}
		if(osKernelGetTickCount() - faultTime >= 30000)
		{
			return FAILEDD;
		}
		else
		{
			return IN_TESTING;
		}
	}
	else{
		faultTime = 0;
		return PASSED;
	}
}

//GSP���߶�·���
DTCTestResult checkGpsAntShort(void)
{
	if(osKernelGetTickCount() < 15000 || gTerminalState.pwrVolt < 8.5f || gTerminalState.pwrVolt > 16.5f) {
		return IN_TESTING;
	}
	else if(g_tGPS.antSta == 2){
		return FAILEDD;
	}
	else{
		return PASSED;
	}
}

//GPS���߿�·���
DTCTestResult checkGpsAntOpen(void)
{
	if(osKernelGetTickCount() < 15000 || gTerminalState.pwrVolt < 8.5f || gTerminalState.pwrVolt > 16.5f) {
		return IN_TESTING;
	}
	else if(g_tGPS.antSta == 3){
		return FAILEDD;
	}
	else{
		return PASSED;
	}
}

//TBOX����������У������
DTCTestResult checkTboxLimphome(void)
{
	if(osKernelGetTickCount() < 15000 || gTerminalState.pwrVolt < 8.5f || gTerminalState.pwrVolt > 16.5f) {
		return IN_TESTING;
	}
	else if(0){
		return FAILEDD;
	}
	else{
		return PASSED;
	}
}

//GPRSͨ�Ź���,�汾����ʶ��
DTCTestResult checkGprsFault(void)
{
	if(osKernelGetTickCount() < 60000 || gTerminalState.pwrVolt < 8.5f || gTerminalState.pwrVolt > 16.5f) {
		return IN_TESTING;
	}
	else if(!Fun_Gprs_getVer((char*)&tempVals,sizeof(tempVals))){				//ģ���ȡ����
		return FAILEDD;
	}
	else{
		return PASSED;
	}
}

uint8_t NcmTxSuppoted(void)
{
	if(osKernelGetTickCount() - getUdsTime < 500)
	{
		return 0;
	}
	return ((Commbit & 0x01) == 0);
}

uint8_t NcmRxSuppoted(void)
{
	return ((Commbit & 0x02) == 0);
}

uint8_t NmcmTxSuppoted(void)
{
	return ((Commbit & 0x04) == 0);
}

uint8_t NmcmRxSuppoted(void)
{
	return ((Commbit & 0x08) == 0);
}

//UDSCAN����
static uint8_t SendFrame(uint32_t ID, uint8_t *array, uint8_t length, uint8_t priority, uint8_t rtr, uint8_t ide)
{
	CAN_msg msg_buf = {0x346,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,STANDARD_TYPE,DATA_FRAME};
	msg_buf.id = ID;
	msg_buf.len = length > 8 ? 8 : length;
	memcpy(msg_buf.data,array,msg_buf.len);
	msg_buf.format = ide;
	msg_buf.type = rtr;
	return CAN_send(udsCanch,&msg_buf,10);
}

//������Կ
static uint32_t SeedToKeyLevel1(uint32_t seed)
{
	uint8_t xor_data[4] = {0x92,0xB0,0xD5,0x11};
	uint8_t cal_data[4];
	uint8_t key_data[4];
	uint8_t *pSeed = (uint8_t*)&seed;
	uint8_t i;
	for(i=0;i<4;i++)
	{
		cal_data[i] = pSeed[i] ^ xor_data[i];
	}
	key_data[0] = ((cal_data[3] & 0x0F) << 4) | (cal_data[0] & 0xFF); 
	key_data[1] = (cal_data[1] & 0xD8) | ((cal_data[2] & 0xF0) >> 4); 
	key_data[2] = (cal_data[2] & 0xCF) | (cal_data[0] & 0xF0); 
	key_data[3] = (cal_data[3] & 0xF0) | ((cal_data[1] & 0xD0) >> 4); 
	return *((uint32_t*)key_data);
}

//UDS��λ
static void fneboardReset(EcuResetType type)
{
	osDelay(100);
	BoardReset();
}

//UDS�ٶ�����
static uint8_t speedCondition(void)
{
	if(gRealData.speed > 3 && gRealData.speed < 0xFFFE)
	{
		return 0;
	}
	return 1;
}

//UDS������ƻص�
static void CommCtrl(CommulicationType type, communicationParam para)
{
	switch(type)
	{
		case ERXTX:
		{
			if(para == NWMCM_NCM)
			{
				//ȫ������������շ���
				NcmTxSet(1);
				NcmRxSet(1);
				NmcmTxSet(1);
				NmcmRxSet(1);
				setCANmask(0,0,0,0);
			}
			else if(para == NCM)
			{
				//���汨��������շ���
				NcmTxSet(1);
				NcmRxSet(1);
				setCANmask(0,0,0,0);
			}
			else if(para == NWMCM)
			{
				//������������շ���
				NmcmTxSet(1);
				NmcmRxSet(1);
			}
		}
		break;
		case ERXDTX://��֧��
		case DRXETX://��֧��
		break;
		case DRXTX:
		{
			if(para == NWMCM_NCM)
			{
				//ȫ����ֹ
				setCANmask(0,0,0xFFFFFFFF,0);
				setCANmask(1,0x682,0xFFFFFFFF,0);
				NcmTxSet(0);
				NcmRxSet(0);
				NmcmTxSet(0);
				NmcmRxSet(0);
			}
			else if(para == NCM)
			{
				//��ֹ����ͨ�ű���
				setCANmask(0,0x7df,0xFFFFFFFF,0);
				setCANmask(1,0x682,0xFFFFFFFF,0);
				NcmTxSet(0);
				NcmRxSet(0);
			}
			else if(para == NWMCM)
			{
				//��ֹ���������
				NmcmTxSet(0);
				NmcmRxSet(0);
			}
		}
		break;
	}
}

//UDS����
void udsProc(void)
{
	uint16_t speed = gRealData.speed;
	if(gRealData.speed >= 0xFFFE)
	{
		speed = 0;
	}
	gprsSignal[0] = 0;
	gprsSignal[1] = Fun_Gprs_Csq();
	SnapshotPowerVol = gTerminalState.pwrVolt * 10;
  SnapshotSpeed[0] = (uint16_t)speed >> 8;
	SnapshotSpeed[1] = (uint16_t)speed >> 0;
  SnapshotTolMil[0] = (uint32_t)gRealData.totalMileage >> 16;
	SnapshotTolMil[1] = (uint32_t)gRealData.totalMileage >> 8;
	SnapshotTolMil[2] = (uint32_t)gRealData.totalMileage >> 0;
  SnapshotBcdTime[0] = 0x20;
	SnapshotBcdTime[1] = (((g_system_dt.year - 2000) % 100 / 10) << 4) | ((g_system_dt.year - 2000) % 10);
	SnapshotBcdTime[2] = ((g_system_dt.month / 10) << 4) | (g_system_dt.month % 10);
	SnapshotBcdTime[3] = ((g_system_dt.day / 10) << 4) | (g_system_dt.day % 10);
	SnapshotBcdTime[4] = ((g_system_dt.hour / 10) << 4) | (g_system_dt.hour % 10);
	SnapshotBcdTime[5] = ((g_system_dt.minute / 10) << 4) | (g_system_dt.minute % 10);
	if(fun_can_Get_State(BSP_CAN) == 0)
	{
		recvCanCnt = 0;
	}
	if(memcmp(vin1,vin2,17) != 0)
	{
		memcpy(vin2,vin1,17);
		memcpy(gSysPara.vinCode,vin1,17);
		System_Pare_Save();
	}
	Diagnostic_Proc(&types_T);
	if(fun_can_Get_State(BSP_CAN) == 0)
	{
		if(UnintUds == 0)
		{
			UnintUds = 1;
			Diagnostic_DelInit();
		}
	}
	else
	{
		if(UnintUds == 1)
		{
			
		}
		UnintUds = 0;
	}
}

#include "stm32f4xx_eeprom.h"

//UDS��ʼ��
void udsInit(uint8_t ch)
{
	FLASH_Unlock();
	EE_Init();
	
	if(gFrimPara.factoryDate[0] == 0x20 && (gFrimPara.factoryDate[2] > 0 && gFrimPara.factoryDate[2] < 0x12))
	{
		memcpy(ECUManufacturingDate,gFrimPara.factoryDate,4);
	}
	
	memcpy(ecuId,gFrimPara.qbId,strlen(gFrimPara.qbId));
	memcpy(vin1,gSysPara.vinCode,17);
	memcpy(vin2,gSysPara.vinCode,17);
	
	Diagnostic_Init(&types_T,0x682, 0x602 , 0x7DF, 0 , 1024 , SendFrame,50,2000);
	
	//********************************** service 10*****************************************//
	InitSetSessionSupportAndSecurityAccess(TRUE,0x10,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO);
	InitSetSessionControlParam(TRUE , TRUE , TRUE , FALSE , FALSE , TRUE); 
	  //********************************** service 11*****************************************//
	InitSetSessionSupportAndSecurityAccess(TRUE,0x11,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO);
	InitSetSysResetParam(TRUE , FALSE , TRUE , FALSE , FALSE ,fneboardReset , TRUE,speedCondition);	 	
	//********************************** service 14*****************************************//
	InitSetSessionSupportAndSecurityAccess(TRUE,0x14,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO);
	InitAddDTCGroup(0x00100000); //����
	InitAddDTCGroup(0x00400000); //����
	InitAddDTCGroup(0x00800000); //����
	InitAddDTCGroup(0x00C00000); //ͨ��
	InitAddDTCGroup(0x00FFFFFF); //ȫ��
	//********************************** service 19*****************************************//
 	InitSetSessionSupportAndSecurityAccess(TRUE,0x19,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO);
	InitSetDTCAvailiableMask(0x09);
	//********************************** service 27*****************************************//
	InitSetSessionSupportAndSecurityAccess(TRUE,0x27,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_UNSUPPORT,LEVEL_UNSUPPORT);
	InitAddSecurityAlgorithm(LEVEL_ONE,SeedToKeyLevel1,0x01,0x02, NULL ,3 , 10000, SUB_EXTENDED,4);
	//********************************** service 28*****************************************//
	InitSetSessionSupportAndSecurityAccess(TRUE,0x28,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO);
	InitSetCommControlParam(TRUE , FALSE , FALSE , TRUE , TRUE , TRUE , TRUE , CommCtrl , TRUE);  
	//********************************** service 2E*****************************************//
	InitSetSessionSupportAndSecurityAccess(TRUE,0x2E,LEVEL_UNSUPPORT,LEVEL_ONE,LEVEL_ONE,LEVEL_UNSUPPORT,LEVEL_UNSUPPORT,LEVEL_UNSUPPORT);
	//********************************** service 3E*****************************************//
	InitSetSessionSupportAndSecurityAccess(TRUE,0x3E,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO);
	InitSetTesterPresentSupress(TRUE);
	//********************************** service 85*****************************************//
	InitSetSessionSupportAndSecurityAccess(TRUE,0x85,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO,LEVEL_UNSUPPORT,LEVEL_ZERO,LEVEL_ZERO);
	InitSetDTCControlSupress(TRUE);
	
	InitAddDTC(0xD00017,checkVolHight,10, 1 ,LEVEL_C);		//ECU�����ѹ��
	InitAddDTC(0xD00016,checkVolLow,10,1 ,LEVEL_C);				//ECU�����ѹ��
	InitAddDTC(0xC07388,checkBusOff,10, 2 ,LEVEL_C);			//������ BusOff
	//InitAddDTC(0xC10187,checkTcu188,10, 1 ,LEVEL_C);		//���ղ���TCU��0x188�����籨��,����
	InitAddDTC(0xC12187,checkAbc140,10, 1 ,LEVEL_C);			//���ղ���ABS��0x140��CAN����
	InitAddDTC(0xD10387,checkObc172,10, 1 ,LEVEL_C);			//���ղ���OBC��0x172��CAN����//���䱨��
	InitAddDTC(0xD10287,checkVcu01a,10, 1 ,LEVEL_C);			//���ղ���VCU��0x01A��CAN����
	InitAddDTC(0xC15587,checkIc260,10, 1 ,LEVEL_C);				//���ղ���IC��0x260��CAN����
	//InitAddDTC(0xC12787,checkTpms294,10, 1 ,LEVEL_C);		//���ղ���TPMS��0x294��CAN����//̥ѹ���ģ�����
	InitAddDTC(0xD10087,checkBms441,10, 1 ,LEVEL_C);			//���ղ���BMS��0x441��CAN����
	InitAddDTC(0xC29887,checkDcdc193,10, 1 ,LEVEL_C);			//���ղ���DCDC��0x193��CAN����
	InitAddDTC(0xD10187,checkMcu138,10, 1 ,LEVEL_C);			//���ղ���MCU��0x138��CAN����
	InitAddDTC(0x960013,checkGprsAntOpen,10, 1 ,LEVEL_C);	//��⵽4G���߿�·
	InitAddDTC(0x960111,checkGpsAntShort,10, 1 ,LEVEL_C);	//��⵽GPS���߶�·
	InitAddDTC(0x960113,checkGpsAntOpen,10, 1 ,LEVEL_C);	//��⵽GPS���߿�·
	InitAddDTC(0xD20100,checkTboxLimphome,10, 1 ,LEVEL_C);//TBOX����limphomeģʽ
	InitAddDTC(0x961A65,checkGprsFault,10, 1 ,LEVEL_C);		//MCU��⵽4Gģ���޷�ͨѶ
	 //********************************** service 22  2E  2F*****************************************//
	InitAddDID(0xF187, 21 , (uint8_t*)"2136100-BC010-A000000",  REALTIME_DID , NULL , READONLY , 0 ,TRUE);//������������ BOM ��
	InitAddDID(0xF18A, 10 , (uint8_t*)"BLXYFS    ",  REALTIME_DID , NULL , READONLY , 0 ,TRUE);						//��Ӧ��ʶ���
	InitAddDID(0xF18B, 4 , (uint8_t*)ECUManufacturingDate,  REALTIME_DID , NULL , READONLY , 0 ,TRUE);		//ECU ��������
	InitAddDID(0xF18C, 24 , (uint8_t*)ecuId ,  REALTIME_DID , NULL , READONLY , 0 ,TRUE);									//ECU ����
	InitAddDID(0xF190, 17 , (uint8_t*)vin1 ,  REALTIME_DID , NULL , READWRITE , 0 ,TRUE);									//���ܺ�
	InitAddDID(0xF192, 16 , (uint8_t*)ecuHw ,  REALTIME_DID , NULL , READONLY , 0 ,TRUE);									//��Ӧ�� ECU Ӳ���汾��
	InitAddDID(0xF193, 16 , (uint8_t*)ecuHw ,  REALTIME_DID , NULL , READONLY , 0 ,TRUE);									//��Ӧ�� ECU Ӳ���汾��
	InitAddDID(0xF189, 16 , (uint8_t*)ecuSw ,  REALTIME_DID , NULL , READONLY , 0 ,TRUE);									//��Ӧ��ECU����汾��
	InitAddDID(0xF195, 16 , (uint8_t*)ecuSw ,  REALTIME_DID , NULL , READONLY , 0 ,TRUE);									//��Ӧ��ECU����汾��
	InitAddDID(0xF197, 10 , (uint8_t*)ecuName,  REALTIME_DID , NULL , READONLY , 0 ,TRUE);								//ECU����������
	InitAddDID(0xF010, 4 , vehCfg ,  REALTIME_DID , NULL , READWRITE , 0 ,TRUE);													//��������
	InitAddDID(0xF011, 8, funCfg ,  REALTIME_DID , NULL , READWRITE , 0 ,TRUE);														//��������
	InitAddDID(0x6003, 2 , gprsSignal ,  REALTIME_DID , NULL , READONLY , 0 ,TRUE);												//4G�ź�ǿ��
	memset(tempVals,0,sizeof(tempVals));
	Fun_Gprs_getICCID((char*)&tempVals,sizeof(tempVals));
	InitAddDID(0xE001, 20, tempVals ,  REALTIME_DID , NULL , READONLY , 0 ,TRUE);							//ICCID
	memset(tempVals,0,sizeof(tempVals));
	Fun_Gprs_getIMEI((char*)&tempVals,sizeof(tempVals));
	InitAddDID(0xE002, 15, tempVals ,  REALTIME_DID , NULL , READONLY , 0 ,TRUE);							//IMEI
	#if 1
	InitSetCanDriverVersionDID(0x0A01);
	InitSetCanNMVersionDID(0x0A02);
	InitSetCanDiagnosticVersionDID(0x0A03);
	InitSetCanDataBaseVersionDID(0x0A04);
	InitSetCurrentSessionDID(0xF186);
	#endif
	InitSetSessionSupportAndSecurityAccess(TRUE,0x22,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO,LEVEL_ZERO);
	InitSetSessionSupportAndSecurityAccess(TRUE,0x2F,LEVEL_UNSUPPORT,LEVEL_UNSUPPORT,LEVEL_ONE,LEVEL_UNSUPPORT,LEVEL_UNSUPPORT,LEVEL_UNSUPPORT);
	//**********************************snaptshot*****************************************//
	InitAddDTCSnapShot(0x01 , 0xDF00 , &SnapshotIgnitionStatus ,1);															//���״̬
	InitAddDTCSnapShot(0x01 , 0xDF01 , &SnapshotPowerVol ,1);																		//�����ѹ
	InitAddDTCSnapShot(0x01 , 0xDF02 , SnapshotSpeed ,2);																				//����
	InitAddDTCSnapShot(0x01 , 0xDF03 , SnapshotTolMil ,3);																			//�����
	InitAddDTCSnapShot(0x01 , 0xDF04 , SnapshotBcdTime ,6);																			//����ʱ��
	InitSetAgingCounterRecordNumber(3);
	InitSetAgedCounterRecordNumber(2);
	InitSetOccurrenceCounterRecordNumber(1);
	InitSetPendingCounterRecordNumber(0);
	Diagnostic_LoadAllData();
	Diagnostic_SetNLParam(&types_T,70, 150, 150, 70, 20, 70, 0, 20, 0);
	udsCanch = ch;
}

/*****************************************************************
*	�� �� ��: calcExtremum
*	����˵��: CAN���ݽ���
*	�� �� ֵ: ��
*****************************************************************/
void udsRecvData(uint8_t ch,CAN_msg *msg)
{
	uint32_t ticks = osKernelGetTickCount();
	recvCanCnt++;
	switch(msg->id)
	{
		case 0x188:tcu188Time = ticks;break;
		case 0x140:abc140Time = ticks;break;
		case 0x172:obc172Time = ticks;break;
		case 0x01a:vcu01aTime = ticks;
			{
				ticks = calcCanValue(15,3,msg->data,LOWHIGHT,INTEL);
				if(ticks == 0)
				{
					SnapshotIgnitionStatus = 0;
				}
				else if(ticks == 1)
				{
					SnapshotIgnitionStatus = 1 << 1;
				}
				else if(ticks == 1)
				{
					SnapshotIgnitionStatus = 1 << 1;
					SnapshotIgnitionStatus |= 1 << 2;
				}
			}
			break;
		case 0x260:ic260Time = ticks;break;
		case 0x294:tpms294Time = ticks;break;
		case 0x441:bms441Time = ticks;break;
		case 0x193:dcdc193Time = ticks;break;
		case 0x138:mcu138Time = ticks;break;
		case 0x7df:getUdsTime = ticks;break;
		case 0x682:getUdsTime = ticks;break;
		case 0x177:
			{
				if((msg->data[5] & 0x60) == 0x20)
				{
					slowChargeTime = ticks;
				}
			}
			break;
		default:break;
	}
	if(NmcmRxSuppoted() || msg->id == 0x7DF || msg->id == 0x682)
	{
		Diagnostic_RxFrame(&types_T,msg->id, msg->data , msg->format , msg->len , msg->type);	
	}
}

