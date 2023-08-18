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
const char* carType = "FSFC_5041XLCEFCEV2_GH70_FV2.00";
const uint8_t CAN1_USED_ANA = 1;								/*�Ƿ�ʹ��CAN1*/
const uint8_t CAN2_USED_ANA = 1;								/*�Ƿ�ʹ��CAN2*/

const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1������*/
const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2������*/
#define FAULTCOUNT 15																		//���������
#define MAX_SUBSYS_CODE_LEN 24													//��ر��볤��


const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*���ݸߵ�λ*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN����ģʽ����ΪINTEL��ʽ�� MOTOROLA��ʽ*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN����ƫ������ϵ������ģʽFACTOR_OFFSET  OFFSET_FACTOR*/

const uint8_t CALC_EXTREMUN  = 0;											/*�Ƿ���㼫ֵ 0:�ر� 1:����*/
static void analysisVol(uint32_t canID,const uint8_t *data);
static void analysisTemper(uint32_t canID,const uint8_t *data);
void send_RechargeSysCode(uint32_t canID,const uint8_t *data);


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
	uint8_t i;
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
			}
			break;		
		case 0x18FF31F0:	
		    {		
				//�ƶ�̤��
				n8Val = calcCanValue(12,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,1);//�г�ֵ��-1��״ֵ̬Ϊ1
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
				gRealData.speed = calcRealValue(20,12,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��λ
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val) 
					gRealData.stall = (bValTmp | NEUTRAL);
				else if(2 == n8Val) 
					gRealData.stall = (bValTmp | REVERSE);
				else if(1 == n8Val) 
					gRealData.stall = (bValTmp | DIRVE);
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
				gRealData.soc = calcRealValue(16,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
 					 
		case 0x18FF1519:
			{			
				//����DCDC״̬
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val ||2 == n8Val) 
					gRealData.dc2dcState = DCDC_WORK;
				else if(0 == n8Val) 
					gRealData.dc2dcState = DCDC_BREAK;
				else if(3 == n8Val) 
					gRealData.dc2dcState = DCDC_ABNORMAL;
				else 
					gRealData.dc2dcState = DCDC_INVALID;	
				 //��ѹDC/DC1״̬
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(2 == n8Val) 
					gRealData.dc2dcState_highVol = DCDC_WORK;
				else
					gRealData.dc2dcState_highVol = DCDC_BREAK;

			    //DC-DC�¶ȱ���
				n8Val = calcCanValue(50,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(4 == n8Val)
					gRealData.dc2dcTemAlert = 1;
			    else 
					gRealData.dc2dcTemAlert = 0;
			    //DC-DC״̬����
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(3 == n8Val && gRealData.carState == CARSTA_START)
					gRealData.dc2dcStateAlert = 1;
			    else 
					gRealData.dc2dcStateAlert = 0;

			}
		    break;
		case 0x18FFA7F3:
			{
				//��Ե����
				gRealData.mohm = calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
		case 0x18EF4EEF:
			{
				//�����������
				gRealData.motorCnt = 1;
				//�������ת��
				gRealData.motorData[0].motorSpeed = (int16_t)calcRealValue(8,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ת��
				gRealData.motorData[0].motorTorsion = calcRealValue(24,16,1,-2000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;

		case 0x18EF4FEF:
			{
				//����������
				gRealData.motorData[0].motorIdx = 1;
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
				//��������������¶�
				gRealData.motorData[0].motorCtrTemp = calcRealValue(48,8,0.1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��������¶�
				gRealData.motorData[0].motorTemp = (int16_t)calcRealValue(56,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��������������ѹ
				gRealData.motorData[0].motorVol = calcRealValue(16,16,0.1	,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//���������ֱ��ĸ�ߵ���
				gRealData.motorData[0].motorCur = calcRealValue(32,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
	
			}
		    break;
		case 0x18C1E928:
			{
               //ȼ�ϵ��1��ѹ
               gRealData.fuelBatVol = calcRealValue(0,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
               //ȼ�ϵ��1����
			   gRealData.fuelBatCur = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			   
			}
		    break;
		case 0x1802F0F5:
			{
               //ȼ��������
               gRealData.batFuelConsumption = calcRealValue(48,8,0.2,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //ȼ�ϵ��1�¶�̽������
               gRealData.fuelBatTemCnt = 2;
			}
		    break;
		case 0x1805F0F5:
			{ 
			   //ȼ�ϵ���¶�̽��ֵ
			   gRealData.maxHydrSysTem = calcRealValue(20,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //��ϵͳ������¶�̽�����
			   gRealData.maxHydrSysTemIdx = calcCanValue(28,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //�������Ũ��
			   gRealData.maxHydrThickness = calcRealValue(0,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //�������Ũ�ȴ���������
			   gRealData.maxHydrThicknessIdx = calcCanValue(16,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //�������ѹ��
			   gRealData.maxHydrPressure = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //�������ѹ������������
			   gRealData.maxHydrPressureIdx = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
			
		case 0x18FFA4F3:
			{
				
				gRealData.maxVol_index = calcCanValue(40,12,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.minVol_index = calcCanValue(52,12,msg->data,HIGHTLOW_MODE,CAN_MODE);

				//��ص����ѹ���ֵ
				gRealData.max_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ص����ѹ���ֵ
				gRealData.min_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
		case 0x18FFA5F3:
			{
				//����¶�̽�����
				gRealData.maxTemper_index = calcRealValue(40,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�̽����ϵͳ����
				gRealData.minTemper_index = calcRealValue(48,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�ֵ
				gRealData.max_singleTemper = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��͵��¶�
				gRealData.min_singleTemper = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ɳ�索��װ�ù��ϴ����б�
				n8Val = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val || 0xFF == n8Val){
					MaxBatFault = 0;
				}else{
					if(n8Val > 0 && n8Val < 51){								//1-50		һ������
						MaxBatFault =1;
					}
					else if(n8Val > 50 && n8Val < 101){					//51-100	��������
						MaxBatFault =2;
					}
					else if(n8Val > 100 && n8Val < 151){				//101-150	��������
						MaxBatFault =3;
					}						
					updateFault(fault[0],n8Val);
				}					
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
                if(1 == n8Val)
					gRealData.motorCtrTemAlert = 3;
			    else if(0 == n8Val)
			        gRealData.motorCtrTemAlert = 0;
			    //���ش���װ�����͹���
				gRealData.batOverCharge = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
		case 0x18ff81f0:
			{
				//��������������¶ȱ��� && ��������¶ȱ���
				n8Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.motorCtrTemAlert = (n8Val == 0? 0:1);
				gRealData.motorTempAlert = (n8Val == 0? 0:1);
				//����������ϴ����б�
				n16Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n16Val){
					MaxMotFault = 0;
				}else{
					MaxMotFault = 2;											//������������ϴ��룬 Ϊ��������
					updateFault(fault[1],n16Val);				
				}		
			
			}
		    break;
		case 0x18ff1427:
			{
				for(i=0;i<4;i++)
				{
					n16Val = calcCanValue(i*16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					updateFault(fault[2],n16Val);
				}
			}
		    break;
		case 0x18ff1527:
			{
				for(i=0;i<4;i++)
				{
					n16Val = calcCanValue(i*16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					updateFault(fault[2],n16Val);
				}
			}
		    break;
		case 0x18FFABF3:
			{
				//��ߵ�ѹ�����ϵͳ��
				//gRealData.maxVolPack_index = 1;
				gRealData.maxVolPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��͵�ѹ�����ϵͳ��
				gRealData.minVolPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//gRealData.minVolPack_index = 1;
				//����¶���ϵͳ��
				gRealData.maxTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//gRealData.maxTemperPack_index = 1;
				//����¶���ϵͳ��
				gRealData.minTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//gRealData.minTemperPack_index = 1;
				//�ɳ�索����ϵͳ���� ������MAX_BAT_SUBSYS_NUM
				gRealData.subBatSysCnt = 1;
				//�ɳ�索����ϵͳ��
				gRealData.subSysData[0].subSysIdx = 1;
			    //����������
				gRealData.subSysData[0].singleVolCnt = calcCanValue(48,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
			    //�¶�̽������
				gRealData.subSysData[0].singleTemCnt = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
		    break;
		case 0x18FFB1F3:
			{
				analysisVol(msg->id,msg->data);
			}
		    break;
		case 0x18FFB2F3:
			{
				analysisTemper(msg->id,msg->data);
			}
		    break;
		case 0x18C8E928:
			{ 
			   //ȼ�ϵ���¶�̽��ֵ
			   gRealData.fuelBatTem[0] = calcRealValue(0,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   gRealData.fuelBatTem[1] = calcRealValue(16,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
//				gSelfData80->InWaterTem = calcRealValue(0,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
//				gSelfData80->OutWaterTem = calcRealValue(16,16,0.1,-60,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
		    break;
//		case 0x18D2E928:			//�Զ�������
//			{
//				//����ѹ������ѹ
//				gSelfData80->AirComVol = calcRealValue(32,12,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);

//				//����ѹ��������
//				 gSelfData80->AirComCur = calcRealValue(20,12,0.5,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//	
//			}
//			break;
//		case 0x18D3E928:			//�Զ�������
//			{
//				//����ѭ���õ�ѹ
//				gSelfData80->HyCyclePumpVol = calcRealValue(24,12,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//				//����ѭ���õ���
//				gSelfData80->HyCyclePumpCur = calcRealValue(16,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			}
//			break;
//		case 0x1801F0F5:		//�Զ�������
//			{
//				//����ʣ����
//				gSelfData80->HySurplus = calcRealValue(14,10,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//				gSelfData80->WarmRiskControlCommand = 0;
//				gSelfData80->AirConControlCommand = 0;
//			}
//			break;
			default: break;
	}	
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
	int i;
	//����Э�����õ���ѹλ��             Byte0~Byte1 ��֡��ʼ�����ѹ���
	uint16_t volStartIndex = (data[0]-1)*3;
	uint16_t volTmp = 0;
	
	/******����Э�����*******/
	for(i=0;i<3;++i)
	{
		if(volStartIndex < gRealData.subSysData[0].singleVolCnt)
		{
			 volTmp = calcCanValue(16+i*16,16,data,HIGHTLOW_MODE,CAN_MODE);
			 gRealData.single_vol[volStartIndex++] = volTmp * 0.001; 
		}
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
	int i;
	//�������¶����												Byte0~Byte1 ��֡��ʼ�����¶����
	uint16_t temperStartIndex = (data[0]-1)*6;
	
	/******����Э�����*******/
	for(i=0;i<6;++i)
	{
		if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
		{
			gRealData.single_temper[temperStartIndex++] = (int16_t)calcRealValue(16+i*8,8,1,-40,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
		}
	}
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