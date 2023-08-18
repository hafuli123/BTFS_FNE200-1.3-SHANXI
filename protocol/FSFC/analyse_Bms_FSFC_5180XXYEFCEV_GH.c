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
const char* carType = "FSFC_5180_GH_FV2.00";
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
	
//		gRealData.carState = CARSTA_START;
	
	switch(msg->id)
	{
		case 0x18019A17:
			{
				//�ƶ�ϵͳ����
				n8Val = calcCanValue(30,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
 				if(1 == n8Val)
					gRealData.brakingAlert = 1;							//1  �����ȼ�һ
				else
					gRealData.brakingAlert = 0;							//����	�ޱ���										
			}
			break;	
		case 0x1805F0F5:
			{
				//��ϵͳ������¶�									
				gRealData.maxHydrSysTem = calcRealValue(20,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ϵͳ������¶�̽�����					
				gRealData.maxHydrSysTemIdx = calcCanValue(28,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������Ũ��
				gRealData.maxHydrThickness = calcCanValue(0,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������Ũ�ȴ���������
				gRealData.maxHydrThicknessIdx = calcCanValue(16,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ѹ��
				gRealData.maxHydrPressure = calcRealValue(32,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ѹ������������
				gRealData.maxHydrPressureIdx = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;

		case 0x180CA5A6:
			{
				//����״̬
				n8Val = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val)
					gRealData.carState = CARSTA_START;					//1		����
				else if(0 == n8Val || 2 == n8Val)
					gRealData.carState = CARSTA_STOP;					//0��2	Ϩ��					
				//���״̬
				n8Val = calcCanValue(4,2,msg->data,HIGHTLOW_MODE,CAN_MODE);							
				if(0 == n8Val) 
					gRealData.chargeState = NO_CHARGE;					//0 δ���
				else if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;				//1 ͣ�����
				else if(2 == n8Val) 
					gRealData.chargeState = RUN_CHARGE;					//2	�г����
				else if(3 == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;  		//3 ������				
				//����ģʽ
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val)
					gRealData.operationState = EV;					//0	����
				else if(1 == n8Val)
					gRealData.operationState = EV_FV;				//1	�춯
				else if(3 == n8Val)
					gRealData.operationState = FV;					//2	ȼ��	
				//����
				gRealData.speed = calcRealValue(56,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//�ۼ����
				gRealData.totalMileage = calcRealValue(32,24,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//�ܵ���
				gRealData.total_current = calcRealValue(8,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//�������״̬
				n8Val = calcCanValue(6,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;							//0 �ر�
				else if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;					//1	�ĵ�
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;				//2	����
				else if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;						//3	׼��				
				//ȼ��������
				gRealData.batFuelConsumption = calcRealValue(24,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				
			}
			break;
		case 0x18FF1127:
			{
				//��λ	
				uint8_t bValTmp; 				
				n8Val = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(0 == n8Val  )		//����0������ʱ��λ��0����
					gRealData.stall = (bValTmp | NEUTRAL); 					//0	N
				else if(1 == n8Val  )	
					gRealData.stall = (bValTmp | DIRVE);						//1	D
				else if(2 == n8Val  )
					gRealData.stall = (bValTmp | REVERSE);					//2	R
				else if(3 == n8Val  )
					gRealData.stall = (bValTmp | PART);							//3	P
				//����̤���г�ֵ		
				gRealData.acceleratorVal = calcRealValue(40,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ƶ�̤��״̬			
				n8Val = calcRealValue(32,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,-1);//TODO...����ƶ�̤��Ϊ�г�ֵ����ڶ�������Ϊ-1�����Ϊ״ֵ̬����ڶ�������Ϊ1				
				//�����ƶ�״̬
				SET_ACCE_BREAK();
			}
			break;
		case 0x18FF1518:
			{	
				//DCDC״̬	
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(2 == n8Val  )
					gRealData.dc2dcState = DCDC_WORK;							//2		������
				else
					gRealData.dc2dcState = DCDC_BREAK;						//����	������	
				
				//DC-DC״̬����
				n8Val = calcCanValue(48,2,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				if(3 == n8Val && gRealData.carState == CARSTA_START){
					gRealData.dc2dcStateAlert = 1;					
				}else{
					gRealData.dc2dcStateAlert = 0;	
				}
			
				//DC-DC�¶ȱ���
				n8Val = calcCanValue(50,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
 				if(4 == n8Val)
					gRealData.dc2dcTemAlert = 1;									//4: 1������������������
				else
					gRealData.dc2dcTemAlert = 0;			
			}
			break;
		case 0x18FF2027:
			{
				//ȼ�ϵ�ص�ѹ							��ֵС��0ʱ����0������ת��
				n16Val = calcRealValue(0,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				if(n16Val < 0)
					gRealData.fuelBatVol = 0;
				else
					gRealData.fuelBatVol = n16Val;
				//ȼ�ϵ�ص���							��ֵС��0ʱ����0������ת��
				n16Val = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val < 0)
					gRealData.fuelBatCur = 0;
				else
					gRealData.fuelBatCur = n16Val;
			}
			break;
		case 0x18FF2227:
			{
				//��ѹDCDC״̬
				n8Val = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.dc2dcState_highVol = DCDC_WORK;
				else 
					gRealData.dc2dcState_highVol = DCDC_BREAK;
				
				//ȼ�ϵ���¶�̽������	
				gRealData.fuelBatTemCnt = 2;
				gRealData.fuelBatTem[0]=calcRealValue(0,16,0.1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.fuelBatTem[1]=calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FF41F0:
			{
				//�������ת��							
				gRealData.motorData[0].motorSpeed = calcRealValue(0,16,1,-6000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ת��						
				gRealData.motorData[0].motorTorsion = calcRealValue(16,16,1,-3200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//���������ֱ��ĸ�ߵ���	
				gRealData.motorData[0].motorCur = calcRealValue(32,16,1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FF51F0:
			{
				//��������������¶�			
				gRealData.motorData[0].motorCtrTemp = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//��������¶�						
				gRealData.motorData[0].motorTemp = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��������������ѹ			
				gRealData.motorData[0].motorVol = calcRealValue(16,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
 		case 0x18FF81F0:
			{				
				//��������������¶ȱ���		
				n8Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(3 == n8Val)
					gRealData.motorCtrTemAlert = 2;					//3 ��2��
				else
					gRealData.motorCtrTemAlert = 0;					//����	�޹���
				
				//��������¶ȱ���				
				n8Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(4 == n8Val)
					gRealData.motorTempAlert = 2;						//4 : 2��
				else
					gRealData.motorTempAlert = 0;						//����	�޹���
										
				//����������ϴ����б�
				n16Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n16Val){
					gRealData.alarmLevel = 0;
				}else{
					gRealData.alarmLevel = 2;								//������������ϴ��룬 Ϊ��������
					updateFault(fault[1],n16Val);										
				}										
			}
			break;
		case 0x18FFA1F3:
			{
				//SOC
				gRealData.soc = calcRealValue(16,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FFA2F3:
			{
				//�ܵ�ѹ
				gRealData.total_volt = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//�ɳ�索��װ�õ�ѹ
				gRealData.subSysData[0].subSysVol = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ɳ�索��װ�õ���
				gRealData.subSysData[0].subSysCur = calcRealValue(32,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
		case 0x18FFA4F3:
			{
				//��ߵ�ѹ��ص������
				gRealData.maxVol_index = calcCanValue(40,12,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ص����ѹ���ֵ
				gRealData.max_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��͵�ѹ��ص������
				gRealData.minVol_index = calcCanValue(52,12,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ص����ѹ���ֵ
				gRealData.min_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;		
		case 0x18FFA5F3:
			{
				//����¶�̽�뵥�����
				gRealData.maxTemper_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�ֵ
				gRealData.max_singleTemper = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�̽����ϵͳ����
				gRealData.minTemper_index = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�ֵ
				gRealData.min_singleTemper = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				
				//�ɳ�索��װ�ù��ϴ����б�
				n8Val = calcCanValue(56,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[0],n8Val);
				if(0!=n8Val && 0xFF!=n8Val){
					if(n8Val>=1 && n8Val<=50){								//1-50		һ������
						gRealData.alarmLevel =1;
					}
					else if(n8Val>=51 && n8Val<=100){					//51-100	��������
						gRealData.alarmLevel =2;
					}
					else if(n8Val>=101 && n8Val<=150){				//101-150	��������
						gRealData.alarmLevel =3;
					}	
					updateFault(fault[0],n8Val);	
											
				}else{
					gRealData.alarmLevel = 0;
				}	
			}
			break;	
		case 0x18FFA7F3:
			{		
				//��Ե����
				gRealData.mohm = calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
		case 0x18FFAAF3:			
			{			
				//�¶Ȳ��챨��								
				n8Val = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.tempDiffAlert = (n8Val == 0 ? 0:1 );
				
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
 				
				//���ش���װ�����͹��䱨��
				gRealData.batOverCharge = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				//��ѹ����״̬����
				n8Val = calcCanValue(49,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val)
					gRealData.highPressInterlockStateAlert = 0;		//0	�޹���
				else if(1 == n8Val)
					gRealData.highPressInterlockStateAlert = 3;		//1	���ϵȼ���						
 			}
			break;		
		case 0x18FFABF3:	 
			{
				//��ߵ�ѹ�����ϵͳ��
				gRealData.maxVolPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��͵�ѹ�����ϵͳ��
				gRealData.minVolPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶���ϵͳ��
				gRealData.maxTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶���ϵͳ��
				gRealData.minTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				//�ɳ�索����ϵͳ����			���糵Ϊ1
				gRealData.subBatSysCnt = 1;
				//�ɳ�索����ϵͳ��			ֱ�ӷ�1
				gRealData.subSysData[0].subSysIdx = 1;

				//��������
				gRealData.subSysData[0].singleVolCnt= calcRealValue(48,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�¶�̽������
				gRealData.subSysData[0].singleTemCnt = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
			}
			break;
		case 0x18FFB3F3:	
			{				
				//�ɳ�索��ϵͳ���볤��
				gRealData.rechargeSysCodeLen = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ɳ�索��ϵͳ����
				send_RechargeSysCode(msg->id,msg->data);		
			}
			break;	
		case 0x00FF3327:
			{
				//��߱����ȼ�
				gRealData.alarmLevel = calcCanValue(0,2,msg->data,HIGHTLOW_MODE,CAN_MODE);		
			}
			break;

		case 0x18FF1427:
		case 0x18FF1627:
			{	
				for(i = 0;i<8;i++){
					n8Val = calcCanValue(i*8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(0 != n8Val)
						updateFault(fault[2],n8Val);
				}	
			}
			break;
		case 0x18FF1527:
			{
				for(i = 0;i<6;i++){
					n8Val = calcCanValue(i*8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(0 != n8Val)
						updateFault(fault[2],n8Val);
				}	
				n16Val = calcCanValue(48,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 != n16Val)
					updateFault(fault[2],n16Val);
			}
			break;
		case 0x18FF2427:
			{
				for(i = 0;i<4;i++){
					n16Val = calcCanValue(i*16,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(0 != n16Val)
						updateFault(fault[2],n16Val);
				}		
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
				//���������¶�
				analysisTemper(msg->id,msg->data);
			}		
			break;
//			case 0x04FF3312:
//			{			
//         //ˮ���¶�
//				gSelfData80->InWaterTem = calcRealValue(0,16,0.1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			    //ˮ���¶�
//				gSelfData80->OutWaterTem = calcRealValue(16,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
//			}
//			break;
//			
//		case 0x04FF3712:			//�Զ�������
//			{
//          //����ѹ������ѹ
//				gSelfData80->AirComVol = calcRealValue(24,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			    //����ѹ��������
//				gSelfData80->AirComCur = calcRealValue(40,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			}
//			break;
//		case 0x04FF3912:			//�Զ�������
//			{
//           //����ѭ���õ�ѹ
//				gSelfData80->HyCyclePumpVol = calcRealValue(8,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//           //����ѭ���õ���
//				gSelfData80->HyCyclePumpCur = calcRealValue(16,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			}
//			break;
//		case 0x1801F0F5:		//�Զ�������
//			{
//                //����ʣ����
//				gSelfData80->HySurplus = calcRealValue(14,10,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//			    gSelfData80->AirConControlCommand = 0;
//			    gSelfData80->WarmRiskControlCommand = 0;
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