/*
�ļ���analyse_Bms_Battery.c
���ܣ���չCANЭ��
���ڣ�2021/12/28
��˾����ɽ��Դ
���ߣ�HYY
*/

#include "fun_can.h"
#include "protocol_GB.h"
#include "bsp_gps.h"
#include "string.h"
#include "cmsis_os2.h"
#include "bsp_rtc.h"
#include "FSFC/protocol_GB_EX_FSFC.h"

const char* carType = "FSFC_5180_GH_FV2.00";
const uint8_t CAN1_USED_ANA = 1;								/*�Ƿ�ʹ��CAN1*/
const uint8_t CAN2_USED_ANA = 1;								/*�Ƿ�ʹ��CAN2*/

const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1������*/
const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2������*/
#define FAULTCOUNT 15																		//���������
#define MAX_SUBSYS_CODE_LEN 24													//��ر��볤��
#define SENDINTERVAL		1000														//1s���

static uint8_t isTiming = 0;
static uint32_t sendStamp[6] = {0};												//���ͽ���
static uint8_t heartBeats[2] = {0};																//��������

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*���ݸߵ�λ*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN����ģʽ����ΪINTEL��ʽ�� MOTOROLA��ʽ*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN����ƫ������ϵ������ģʽFACTOR_OFFSET  OFFSET_FACTOR*/

const uint8_t CALC_EXTREMUN  = 0;											/*�Ƿ���㼫ֵ 0:�ر� 1:����*/
static void analysisVol(uint32_t canID,const uint8_t *data);
static void analysisTemper(uint32_t canID,const uint8_t *data);
void send_RechargeSysCode(uint32_t canID,const uint8_t *data);
static void execute_CustomFun(uint8_t ctrl,CAN_msg *msg);
static void send_HeartToCANCh(uint8_t canCh,unsigned int canid);
static void SenVin_To_Can(uint8_t canCh,unsigned int canid);
typedef struct 
{
	uint8_t interval;
	uint32_t faultCode;	
}FAULT;



static FAULT fault[3][FAULTCOUNT];
//���¹������б�
static void updateFault(FAULT fau[],uint32_t code)
{
	uint8_t i,j;
	for(i=0;i<FAULTCOUNT-1;i++)
	{
		if(fau[i].interval == 0)
		{
			//������Ч�����룬�����������ǰ��
			for(j=i;j<FAULTCOUNT-1;j++)
			{
				fau[j] = fau[j+1];
			}
			fau[FAULTCOUNT-1].interval = 0;
		}
	}
	if(code != 0)
	{
		for(i=0;i<FAULTCOUNT;i++)
		{
			if(fau[i].faultCode == code && fau[i].interval > 0)
			{
				//�������Ѵ��ڣ�ˢ�¼�ʱ
				fau[i].interval = 12;
				return;
			}
			//ǰ���Ѿ��Թ�������������򣬴��ж�Ϊ�¹�����
			else if(fau[i].interval == 0)
			{
				//�¹�����
				fau[i].faultCode = code;
				fau[i].interval = 12;
				return;
			}
		}
	}
}

static void getFault(void)
{
	uint8_t i;
	//�����볬ʱ����
	uint8_t batFaultCnt = 0;
	uint8_t motorFaultCnt = 0;
	uint8_t otherFaultCnt = 0;
	uint32_t batFault[FAULTCOUNT]={0};
	uint32_t motorFault[FAULTCOUNT]={0};
	uint32_t otherFault[FAULTCOUNT]={0};
	for(i=0;i<FAULTCOUNT;i++)
	{
		if(fault[0][i].interval > 0)
		{
			fault[0][i].interval--;
			batFaultCnt++;
			batFault[i] = fault[0][i].faultCode;
		}
		if(fault[1][i].interval > 0)
		{
			fault[1][i].interval--;
			motorFaultCnt++;
			motorFault[i] = fault[1][i].faultCode;
		}
		if(fault[2][i].interval > 0)
		{
			fault[2][i].interval--;
			otherFaultCnt++;
			otherFault[i] = fault[2][i].faultCode;
		}
	}
	gRealData.batFaultCnt = batFaultCnt;
	memcpy(gRealData.batFault,batFault,sizeof(gRealData.batFault));
	gRealData.motorFaultCnt = motorFaultCnt;
	memcpy(gRealData.motorFault,motorFault,sizeof(gRealData.motorFault));
	gRealData.otherFaultCnt = otherFaultCnt;
	memcpy(gRealData.otherFault,otherFault,sizeof(gRealData.otherFault));
}

void iniEvCanData(void)
{
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	CAR_TYPE = carType;
	gRealData.rechargeSysCodeLen = 0;
	
	gRealData.carState = CARSTA_START;
}

const static uint8_t SEND_CAN_MIN_INTERVAL = 50;			                //CAN������С���

//static void sendCanData(uint8_t ch);										//����CAN�ӿ�
//static void sendLockCMD(uint8_t ch,CAN_msg *msg);							//����Զ��/������  ָ��
//static void sendIntervalData(unsigned char canCh);						    //������������

static CAN_msg msg_buf = {0,{ 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF },8,0,EXTENDED_TYPE,DATA_FRAME};
static uint32_t minInterl = 0;
static uint8_t isSendCan = 0;
static uint8_t canCh = 0xFF;
static uint16_t tempVal;
static uint16_t Voltageofsystem;
static uint32_t  BatteryNum;
static int Vnum=1;
uint8_t authResultNum;
uint8_t authResult;
/*	nstartPos:��ʼλ
	nlen:���ݳ���
	factor:ϵ��
	offset:ƫ����
	pcanVal:can������
	hightLowMode:�Ƿ��λ��ǰ�������λ��ǰ��Ҫת��Ϊ��λ��ǰ
	canMode:can�����ʽ��Ӣ�ض���ʽ��Ħ���޸��ʽ��*/
void unpackEvCAN(uint8_t ch,CAN_msg *msg)
{
	uint8_t n8Val = 0;
	uint16_t n16Val = 0;
	uint32_t n32Val = 0;
	
	uint8_t MaxFault = 0;
	uint8_t MaxBatFault = 0;
	uint8_t MaxMotFault = 0;
	uint8_t MaxOthFault = 0;
	uint8_t bValTmp;
//		gRealData.carState = CARSTA_START;
	switch(msg->id)
	{
		case 0x18EF4AEF:
		{
				//����״̬
				n8Val = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.carState = CARSTA_START;
				else if(2 == n8Val) 
					gRealData.carState = CARSTA_STOP;
				else if(3 == n8Val) 
					gRealData.carState = CARSTA_OTHER;
				//�ƶ�̤��
				n8Val = calcCanValue(12,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,1);
				//�����ƶ�״̬
				SET_ACCE_BREAK();
		}
		break;
		case 0x18FFACF3:
		{
				//���״̬
				n8Val = calcCanValue(3,3,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;
				else if(2 == n8Val) 
					gRealData.chargeState = RUN_CHARGE;
				else if(0 == n8Val) 
					gRealData.chargeState = NO_CHARGE;
				else if(3 == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;
				else if(4 == n8Val) 
					gRealData.chargeState = CHARGE_ABNORMAL;
		}
		break;
		case 0x18FF5327:
		{
				//����ģʽ
				n8Val = calcCanValue(3,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.operationState = EV;
				else if(2 == n8Val) 
					gRealData.operationState = EV_FV;
		}
		break;
		case 0x18FF1127:
		{
				//����
				gRealData.speed = calcRealValue(20,12,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��λ
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val) 
					gRealData.stall = (bValTmp | NEUTRAL);
				else if(2 == n8Val) 
					gRealData.stall = (bValTmp | REVERSE);
				else if(1 == n8Val) 
					gRealData.stall = (bValTmp | DIRVE);
				else 
					gRealData.stall = (bValTmp | PART);
				//����̤��
				gRealData.acceleratorVal = (uint8_t)calcRealValue(40,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0xC02EF22:
		{
				//�����
				gRealData.totalMileage = calcRealValue(0,32,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			
		}
		break;
		case 0x18FFA2F3:
		{
				//�ܵ�ѹ
				gRealData.total_volt = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysVol = gRealData.total_volt;
				//�ܵ���
				gRealData.total_current = calcRealValue(32,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysCur = gRealData.total_current;
		}
		break;
		case 0x18FFA1F3:
		{
				//SOC
				gRealData.soc = (uint8_t)calcRealValue(16,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FF1519:
		{
				//����DCDC״̬
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(2 == n8Val) {
					gRealData.dc2dcState  = DCDC_WORK;
					gRealData.dc2dcState_highVol = DCDC_WORK;
					gRealData.dc2dcStateAlert = 0;
				} 
				else if(0 == n8Val) {
					gRealData.dc2dcState = DCDC_BREAK;
					gRealData.dc2dcState_highVol = DCDC_BREAK;	
					gRealData.dc2dcStateAlert = 0;
				}
				else if(3 == n8Val) {
					gRealData.dc2dcState = DCDC_ABNORMAL;
					gRealData.dc2dcState_highVol = DCDC_ABNORMAL;
					gRealData.dc2dcStateAlert = 1;
				}
				//DC-DC�¶ȱ���
				n8Val = calcCanValue(50,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(4 == n8Val)
					gRealData.dc2dcTemAlert = 1;
				else
					gRealData.dc2dcTemAlert = 0;
		}
		break;
		case 0x18FFA7F3:
		{
				//��Ե����
				gRealData.mohm = calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		case 0x18EF4EEF:
		{
				//�������״̬
				n8Val = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;
				else if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if(4 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;
				//�����������
				gRealData.motorCnt = 1;
				//����������
				gRealData.motorData[0].motorIdx = 1;
				//��������������¶�
				gRealData.motorData[0].motorCtrTemp = (int16_t)calcRealValue(48,8,0.1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ת��
				gRealData.motorData[0].motorSpeed = (int16_t)calcRealValue(8,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ת��
				gRealData.motorData[0].motorTorsion = calcRealValue(24,16,1,-2000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��������¶�
				gRealData.motorData[0].motorTemp = (int16_t)calcRealValue(56,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��������������ѹ
				gRealData.motorData[0].motorVol = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//���������ֱ��ĸ�ߵ���
				gRealData.motorData[0].motorCur = calcRealValue(32,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18C1E928:
		{
				//ȼ�ϵ��1��ѹ
				n16Val = calcRealValue(0,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val > 0)
					gRealData.fuelBatVol = n16Val;
				else
					gRealData.fuelBatVol = 0;
				//ȼ�ϵ��1����
				n16Val = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val > 0)
					gRealData.fuelBatVol = n16Val;
				else
					gRealData.fuelBatVol = 0;
		}
		break;
		case 0x1802F0F5:
		{
				//ȼ��������
				gRealData.batFuelConsumption = calcRealValue(48,8,0.2,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18C8E928:
		{
				//ȼ�ϵ��1�¶�̽������
				gRealData.fuelBatTemCnt = 2;
				//ˮ���¶�
				gRealData.fuelBatTem[0] = calcRealValue(0,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//ˮ���¶�
				gRealData.fuelBatTem[1] = calcRealValue(16,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x1805F0F5:
		{
				//��ϵͳ������¶�
				gRealData.maxHydrSysTem = calcRealValue(20,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ϵͳ������¶�̽�����
				gRealData.maxHydrSysTemIdx = calcRealValue(28,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������Ũ��
				gRealData.maxHydrThickness = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������Ũ�ȴ���������
				gRealData.maxHydrThicknessIdx = calcRealValue(16,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ѹ��
				gRealData.maxHydrPressure = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ѹ������������
				gRealData.maxHydrPressureIdx = calcRealValue(48,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FF3327:
		{
				//��߱����ȼ�
				gRealData.alarmLevel = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ƶ�ϵͳ����
				n8Val = calcCanValue(24,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.brakingAlert = (n8Val == 0? 0:1);
		}
		break;
		case 0x18FFAAF3:
		{
				//�¶Ȳ��챨��
				n8Val = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.tempDiffAlert = (n8Val == 0? 0:1);
				//��ظ��±���
				gRealData.batHighTempAlert = calcCanValue(16,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//���ش���װ�����͹�ѹ����
				gRealData.batHighVolAlert = calcCanValue(12,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//���ش���װ������Ƿѹ����
				gRealData.batLowVolAlert = calcCanValue(14,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC�ͱ���
				gRealData.socLowAlert = calcCanValue(20,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�����ع�ѹ����
				gRealData.singleBatHighVolAlert = calcCanValue(8,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//������Ƿѹ����
				gRealData.singleBattLowVolAlert = calcCanValue(10,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC���߱���
				n8Val = calcCanValue(52,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHighAlert = (n8Val == 0? 0:1);
				//SOC���䱨��
				n8Val = calcCanValue(53,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHopAlert = (n8Val == 0? 0:1);
				//�ɳ�索��ϵͳ��ƥ�䱨��
				n8Val = calcCanValue(54,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batNotMatchAlert = (n8Val == 0? 0:1);
				//��ص���һ���Բ��
				n8Val = calcCanValue(56,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.singleBatPoorConsisAlert = (n8Val == 0? 0:1);
				//��Ե����
				gRealData.insulationFailtAlert = calcCanValue(22,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ѹ����״̬����
				n8Val = calcCanValue(49,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.highPressInterlockStateAlert = (n8Val == 0? 0:1);
				//���ش���װ�����͹���	
				gRealData.batOverCharge = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FF81F0:
		{
				//��������������¶ȱ���
				gRealData.motorCtrTemAlert = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//��������¶ȱ���	
				gRealData.motorTempAlert = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����������ϴ����б�
				n16Val = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val > 0){
					updateFault(fault[1],n16Val);
					MaxBatFault = 2;
				}
		}
		break;
		case 0x18FFA5F3:
		{
				//�ɳ�索�ܹ��ϴ����б�
				n8Val = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val > 0 && n8Val <= 50)
				{
					MaxBatFault = 1;
				}
				else if(n8Val > 50 && n8Val <= 100)
				{
					MaxBatFault = 2;
				}
				else if(n8Val > 100 && n8Val <= 150)
				{
					MaxBatFault = 3;
				}
				updateFault(fault[0],n8Val);
		}
		break;
		case 0x18FF1427:
		{
				for(int i=0;i<4;i++)
				{
					n16Val = calcCanValue(i*16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(n16Val>0)
						updateFault(fault[2],n16Val);
				}
		}
		break;
		case 0x18FF1527:
		{
				for(int i=0;i<4;i++)
				{
					n16Val = calcCanValue(i*16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(n16Val>0)
						updateFault(fault[2],n16Val);
				}
		}
		break;
		case 0x18FFABF3:
		{
				//�ɳ�索����ϵͳ���� ������MAX_BAT_SUBSYS_NUM
				gRealData.subBatSysCnt = 1;
				//�ɳ�索����ϵͳ��
				gRealData.subSysData[0].subSysIdx = 1;
				//��ϵͳ����������
				gRealData.subSysData[0].singleVolCnt = calcCanValue(48,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ϵͳ�¶�̽�����
				gRealData.subSysData[0].singleTemCnt = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18FFB1F3:
		{
			//���������ѹ
			analysisVol(msg->id,msg->data);
		}
		break;
		case 0x18FFB2F3:
		{
			//�����¶�̽��
			analysisTemper(msg->id,msg->data);
		}
		break;
		default:break;
	}
	
//		execute_CustomFun(ch,msg);
	
	
	
	MaxFault = (MaxFault > MaxBatFault ? MaxFault : MaxBatFault);
	MaxFault = (MaxFault > MaxMotFault ? MaxFault : MaxMotFault);
	MaxFault = (MaxFault > MaxOthFault ? MaxFault : MaxOthFault);
	
	MaxFault = (MaxFault > gRealData.tempDiffAlert ? MaxFault : gRealData.tempDiffAlert);
	MaxFault = (MaxFault > gRealData.batHighTempAlert ? MaxFault : gRealData.batHighTempAlert);
	MaxFault = (MaxFault > gRealData.batHighVolAlert ? MaxFault : gRealData.batHighVolAlert);
	MaxFault = (MaxFault > gRealData.batLowVolAlert ? MaxFault : gRealData.batLowVolAlert);
	MaxFault = (MaxFault > gRealData.socLowAlert ? MaxFault : gRealData.socLowAlert);
	MaxFault = (MaxFault > gRealData.singleBatHighVolAlert ? MaxFault : gRealData.singleBatHighVolAlert);
	MaxFault = (MaxFault > gRealData.singleBattLowVolAlert ? MaxFault : gRealData.singleBattLowVolAlert);
	MaxFault = (MaxFault > gRealData.socHighAlert ? MaxFault : gRealData.socHighAlert);
	MaxFault = (MaxFault > gRealData.socHopAlert ? MaxFault : gRealData.socHopAlert);
	MaxFault = (MaxFault > gRealData.batNotMatchAlert ? MaxFault : gRealData.batNotMatchAlert);
	MaxFault = (MaxFault > gRealData.singleBatPoorConsisAlert ? MaxFault : gRealData.singleBatPoorConsisAlert);
	MaxFault = (MaxFault > gRealData.insulationFailtAlert ? MaxFault : gRealData.insulationFailtAlert);
	MaxFault = (MaxFault > gRealData.batOverCharge ? MaxFault : gRealData.batOverCharge);
	MaxFault = (MaxFault > gRealData.highPressInterlockStateAlert ? MaxFault : gRealData.highPressInterlockStateAlert);
	MaxFault = (MaxFault > gRealData.motorCtrTemAlert ? MaxFault : gRealData.motorCtrTemAlert);
	MaxFault = (MaxFault > gRealData.motorTempAlert ? MaxFault : gRealData.motorTempAlert);
	MaxFault = (MaxFault > gRealData.dc2dcStateAlert ? MaxFault : gRealData.dc2dcStateAlert);
	MaxFault = (MaxFault > gRealData.dc2dcTemAlert ? MaxFault : gRealData.dc2dcTemAlert);
	MaxFault = (MaxFault > gRealData.brakingAlert ? MaxFault : gRealData.brakingAlert);
	
	gRealData.alarmLevel = (MaxFault > gRealData.alarmLevel ? MaxFault:gRealData.alarmLevel);
}

/*
*********************************************************************************************************
*	�� �� ��: calcExtremum
*	����˵��: ���������ѹ����
*	��    ��: canID  	 CANID
*			 			uint8_t  CAN����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void analysisVol(uint32_t canID,const uint8_t *data)
{
	uint8_t volStartIndex,i;
	volStartIndex = (data[0]-1)*3;
	for(i = 0;i<3;i++){
		gRealData.single_vol[volStartIndex++]  = calcRealValue(16+i*16,16,0.001,0,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
	}	
}

/*
*********************************************************************************************************
*	�� �� ��: calcExtremum
*	����˵��: ���������¶�CAN����
*	��    ��:  canID  	CANID
*			 			 uint8_t  CAN����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void analysisTemper(uint32_t canID,const uint8_t *data)
{
	uint8_t temperStartIndex,i;
	temperStartIndex = (data[0]-1)*6;
	for(i = 0;i<6;i++){
		gRealData.single_temper[temperStartIndex++] = calcRealValue(i*8+16,8,1,-40,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
	}
}

static void execute_CustomFun(uint8_t ctrl,CAN_msg *msg)
{
	uint16_t n16Val = 0;
	uint32_t chTimes = osKernelGetTickCount();
	if(isTiming == 0)									//���ͼ��Уʱ��֤���ͼ���������
	{
		isTiming = 1;
		sendStamp[0] = chTimes - 900;
		
		sendStamp[1] = chTimes - 800;
		sendStamp[2] = chTimes - 600;
		sendStamp[3] = chTimes - 400;
		sendStamp[4] = chTimes - 200;
		sendStamp[5] = chTimes - 100;
	}
	if((chTimes - sendStamp[0]) >= SENDINTERVAL && ctrl == 1)
	{
		sendStamp[0] = chTimes;
		send_HeartToCANCh(1,0x1864F0D1);
	}
	else if((chTimes - sendStamp[1]) >= SENDINTERVAL && ctrl == 2)
	{
		sendStamp[1] = chTimes;
		send_HeartToCANCh(2,0x1865F0D1);
	}
	else if((chTimes - sendStamp[2]) >= SENDINTERVAL && ctrl == 2)
	{
		sendStamp[2] = chTimes;
		SenVin_To_Can(2,0x1873F0E1);
	}
	else if((chTimes - sendStamp[3]) >= SENDINTERVAL && ctrl == 2)
	{
		sendStamp[3] = chTimes;
		SenVin_To_Can(2,0x1874F0E1);
	}
	else if((chTimes - sendStamp[4]) >= SENDINTERVAL && ctrl == 2)
	{
		sendStamp[4] = chTimes;
		SenVin_To_Can(2,0x1875F0E1);
	}
	else if((chTimes - sendStamp[5]) >= SENDINTERVAL)
	{
		sendStamp[5] = chTimes;
		getFault();												//�����볬ʱ����		
	}
}
/*******************************************************************************
*	�� �� ��: send_To_CAN
*	����˵��: �������ݵ�CAN����		���ͼ��500ms
*	��    ��:  ��
*	�� �� ֵ: ��
*******************************************************************************/
static void sendData_To_CAN(uint8_t canCh,unsigned int canid,unsigned char *canbuf){
	CAN_msg	msg_buf;

	msg_buf.id = canid;
	msg_buf.len = 0x08;
	msg_buf.format =  EXTENDED_FORMAT;
	msg_buf.type = DATA_FRAME;
	
	memset(&msg_buf.data[0],0xFF,8);	
	memcpy(&msg_buf.data[0],&canbuf[0],8);
		
	
	if(canCh==1 && fun_can_Get_recvCnt(1) > 0)
	{
		CAN_send(canCh,&msg_buf,0x64);
	}
	if(canCh == 2 && fun_can_Get_recvCnt(2) > 0)
	{
		CAN_send(canCh,&msg_buf,0x64);
	}
	if(canCh == 3 && fun_can_Get_recvCnt(2) > 0)
	{
		CAN_send(canCh,&msg_buf,0x64);
	}
}

/********************************************************************************************************
*	�� �� ��: SenVin_To_Can
*	����˵��: �ն˷���VIN
*	��    ��:  canID  	CANID
*			 			 vinCode  ���ܺ�VIN
*	�� �� ֵ: ��
*********************************************************************************************************/
static void SenVin_To_Can(uint8_t canCh,unsigned int canid){	
	uint8_t sendBuf[8] = {0};	
	switch(canid){
		case 0x1873F0E1:
			memcpy(&sendBuf[0],&gSysPara.vinCode[0],8);
			break;
		case 0x1874F0E1:
			memcpy(&sendBuf[0],&gSysPara.vinCode[8],8);
			break;
		case 0x1875F0E1:
			sendBuf[0] = gSysPara.vinCode[16];
			break;	
	}
	sendData_To_CAN(canCh,canid,sendBuf);
}

/*******************************************************************************
*	�� �� ��: send_LockHeart
*	����˵��: �ն�  ��������������Ϣ/����������Ϣ		���ͼ��500ms
*	��    ��:  canID  	CANID
*			 			 canCh	: 	BSP_CAN1 or BSP_CAN2 or BSP_CAN3
*	�� �� ֵ: ��
*******************************************************************************/	
static void send_HeartToCANCh(uint8_t canCh,unsigned int canid)
{
	uint8_t sendBuf[8] = {0};
	
	if(fun_can_Get_recvCnt(1) > 0 && canCh==1)
	{
		heartBeats[0]++;
		sendBuf[0] = heartBeats[0];
		if(heartBeats[0]>15)
			heartBeats[0] = 0;
	}
	else if(fun_can_Get_recvCnt(2) > 0 && canCh==2)
	{
		heartBeats[1]++;
		sendBuf[0] = heartBeats[1];
		if(heartBeats[1]>15)
			heartBeats[1] = 0;
	}	
	sendData_To_CAN(canCh,canid,sendBuf);
}

void send_RechargeSysCode(uint32_t canID,const uint8_t *data){
	int i;
	//����Э����㴢��ϵͳ�������
	uint16_t codeIndex = (data[0]-1)*6;

	for(i=0;i<6;i++){
		if(codeIndex<MAX_SUBSYS_CODE_LEN){
				gRealData.subSysData[0].rechargeSysCode[codeIndex++] = data[i+2];	
		}
	}	
}