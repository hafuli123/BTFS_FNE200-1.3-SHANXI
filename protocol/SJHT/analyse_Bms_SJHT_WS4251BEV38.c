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
#include "bsp_storage.h"


#define FAULTCOUNT 15																		//���������

const char* carType = "SJHT_WS1250_FV4.03";
const uint8_t CAN1_USED_ANA = 1;								/*�Ƿ�ʹ��CAN1*/
const uint8_t CAN2_USED_ANA = 1;								/*�Ƿ�ʹ��CAN2*/

const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1������*/
const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2������*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;			/*���ݸߵ�λ*/
const static CANANA_MODEL CAN_MODE = INTEL;						/*CAN����ģʽ����ΪINTEL��ʽ�� MOTOROLA��ʽ*/
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;		/*CAN����ƫ������ϵ������ģʽFACTOR_OFFSET  OFFSET_FACTOR*/

const uint8_t CALC_EXTREMUN  = 0;											/*�Ƿ���㼫ֵ 0:�ر� 1:����*/
static void analysisVol(uint32_t canID,const uint8_t *data);
static void analysisTemper(uint32_t canID,const uint8_t *data);
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
static char realVin[18] = {0};


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
	uint8_t bValTmp = 0;
//	gRealData.carState = CARSTA_START;
	switch(msg->id)
	{
		case 0x18E1F3D0:
		{
			if(msg->data[0] == 1)
				memcpy(&realVin[0],&msg->data[1],7);
			else if(msg->data[0] == 2)
				memcpy(&realVin[7],&msg->data[1],7);
			else if(msg->data[0] == 3)
				memcpy(&realVin[14],&msg->data[1],3);
			
//			if(memcmp(realVin,gSysPara.vinCode,17) != 0 && strlen(realVin) == 17 )
//			{
//				memcpy(gSysPara.vinCode,realVin,17);
//				System_Pare_Save();
//			}

		}
		break;
		case 0x18FF43A8:
			{
          //����״̬
				n8Val = calcCanValue(0,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.carState = CARSTA_START;
				else if(2 == n8Val) 
					gRealData.carState = CARSTA_STOP;
				//����ģʽ
				n8Val = calcCanValue(4,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val) 
					gRealData.operationState = EV;
				else if(1 == n8Val) 
					gRealData.operationState = EV_FV;
				else if(2 == n8Val) 
					gRealData.operationState = FV;
				//�������״̬
				n8Val = calcCanValue(12,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;
				else if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if(4 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;
				else if(5 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_ABNORMAL;
				//������״̬
				n8Val = calcCanValue(16,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.engineState = ENGINE_START;
				else if(2 == n8Val) 
					gRealData.engineState = ENGINE_STOP;
				else if(3 == n8Val) 
					gRealData.engineState = ENGINE_ABNORMAL;
				//DCDC ״̬
				n8Val = calcCanValue(22,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val) 
					gRealData.dc2dcState = DCDC_WORK;
				else if(0 == n8Val) 
					gRealData.dc2dcState = DCDC_BREAK;
				
			}
			break;
		case 0x1882D0F3:
		{
			//�ܵ���
			gRealData.total_current = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		case 0x18E5D0F3:
		{
			//�ܵ�ѹ
			gRealData.total_volt = calcRealValue(16,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
			
		case 0x18FF44A8:
			{
				//����
				gRealData.speed = calcRealValue(0,16,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
         //���ſ���
				gRealData.acceleratorVal = calcRealValue(24,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ƶ�����
				n8Val = calcRealValue(32,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,-1);
				//�����ƶ�״̬
				SET_ACCE_BREAK();
			}
			break;
		case 0x18FEDA17:
			{
				//����ʻ���
				gRealData.totalMileage = calcRealValue(0,32,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
    case 0x18FF46A8:
			{
				//�����������
				gRealData.motorCnt = 1;
				//����������
				gRealData.motorData[0].motorIdx = 1;
				//���ת��
				gRealData.motorData[0].motorSpeed = calcRealValue(0,16,0.25,-8000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//���Ť��
				gRealData.motorData[0].motorTorsion = calcRealValue(16,16,0.1,-3200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�
				gRealData.motorData[0].motorTemp = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//ֱ��ĸ�ߵ�ѹ
				gRealData.motorData[0].motorVol = calcRealValue(56,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��������������¶�
				gRealData.motorData[0].motorCtrTemp = calcRealValue(40,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FF32F1:
			{			
				//ֱ��ĸ�ߵ���
				gRealData.motorData[0].motorCur = calcRealValue(24,12,0.4,-800,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;			
		case 0x18FF47A8:
			{
				//�ɳ�索��װ�õ�ѹ
				gRealData.subSysData[0].subSysVol = calcRealValue(0,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC
				gRealData.soc = calcRealValue(8,16,0.0025,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ɳ�索��װ�õ���
				gRealData.subSysData[0].subSysCur = calcRealValue(24,16,0.025,-800,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x18FF48A8:
			{
				//��λ
				n8Val = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(2 == n8Val) 
					gRealData.stall = (bValTmp | NEUTRAL);
				else if(5 == n8Val) 
					gRealData.stall = (bValTmp | PART);
				else if(3 == n8Val) 
					gRealData.stall = (bValTmp | REVERSE);
				else if(1 == n8Val) 
					gRealData.stall = (bValTmp | DIRVE);
			}
			break;
		case  0x18E6D0F3:
			{
				//��ϵͳ����������
				gRealData.subSysData[0].singleVolCnt = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ϵͳ�¶�̽�����
				gRealData.subSysData[0].singleTemCnt = calcCanValue(24,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ɳ�索����ϵͳ���� ������MAX_BAT_SUBSYS_NUM
				//gRealData.subBatSysCnt = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x1884D0F3:
			{
				//����¶���ϵͳ��
				gRealData.maxTemperPack_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�̽�����
				gRealData.maxTemper_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�ֵ
				gRealData.max_singleTemper = calcRealValue(0,8,1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶���ϵͳ��
				gRealData.minTemperPack_index = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�̽�����
				gRealData.minTemper_index = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��͵��¶�
				gRealData.min_singleTemper = calcRealValue(8,8,1,-50,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x1885D0F3:
			{
				//��ߵ�ѹ�����ϵͳ��
				gRealData.maxVolPack_index = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ߵ�ѹ��ص������
				gRealData.maxVol_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ص����ѹ���ֵ
				gRealData.max_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x1886D0F3:
			{
				//��͵�ѹ�����ϵͳ�� 
				gRealData.minVolPack_index = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��͵�ѹ��ص������
				gRealData.minVol_index = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ص����ѹ���ֵ
				gRealData.min_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x18C2D0F3:
			{
         //�����¶�̽��
			   analysisTemper(msg->id,msg->data);
			}
			break;
		case  0x18C1D0F3:
			{
         //���������ѹ
			   analysisVol(msg->id,msg->data);
			}
			break;
		case  0x18FF35F1:
			{
				 //��ǰ���ϵͳ����������
				 gRealData.motorFaultCnt = calcCanValue(8,6,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //��������������¶ȱ���
			   gRealData.motorCtrTemAlert = calcCanValue(14,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //��������¶ȱ���
			   gRealData.motorTempAlert = calcCanValue(15,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case  0x18FF33F1:
			{
				 //��Ե����
				 gRealData.mohm = calcRealValue(48,8,8,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FF99A8:
			{
				//����������ϴ����б�
				n8Val = calcCanValue(16,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[1],n8Val);
				//�ɳ�索��װ�ù��ϴ����б�
				n8Val = calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[0],n8Val);
				//�������ϴ����б�
				n8Val = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[2],n8Val);
				n8Val = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[2],n8Val);
				n8Val = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				updateFault(fault[2],n8Val);
				getFault();												//�����볬ʱ����		
			}
			break;
		case  0x1881D0F3:
			{
					 //��߱����ȼ�
					 gRealData.alarmLevel = calcCanValue(21,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
					 //���״̬
					 n8Val = calcCanValue(16,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
					 if(0 == n8Val)
				  gRealData.chargeState = NO_CHARGE;
			   else if(1 == n8Val)
			   {
				   n8Val = calcCanValue(19,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				   if(0 == n8Val)
						gRealData.chargeState = NO_CHARGE;
			       else if(1 == n8Val)
						gRealData.chargeState = STOP_CHARGE;
			       else if(2 == n8Val)
						gRealData.chargeState = CHARGE_FINISH;
			       else if(3 == n8Val)
						gRealData.chargeState = CHARGE_ABNORMAL;
			   }
			   //��о�²��쳣����
			   gRealData.tempDiffAlert = calcCanValue(24,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //��о�¶ȹ��߱���
			   gRealData.batHighTempAlert = calcCanValue(26,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
               //PACK��ѹ����
			   gRealData.batHighVolAlert = calcCanValue(28,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //PACKǷѹ����
			   gRealData.batLowVolAlert =  calcCanValue(30,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //��Ե����
			   gRealData.insulationFailtAlert = calcCanValue(38,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //�����ѹǷѹ����
			   gRealData.singleBattLowVolAlert = calcCanValue(36,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //�����ѹ���߱���
			   gRealData.singleBatHighVolAlert = calcCanValue(34,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
               //SOC���ͱ���
			   gRealData.socLowAlert = calcCanValue(32,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //����ѹ�����
			   gRealData.singleBatPoorConsisAlert = calcCanValue(40,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //SOC���߱���
			   gRealData.socHighAlert = calcCanValue(52,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //SOC���䱨��
			   gRealData.socHopAlert = calcCanValue(53,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //BMSϵͳ��ƥ�䱨��
			   gRealData.batNotMatchAlert = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			   //��ѹ��������
			   gRealData.highPressInterlockStateAlert = calcCanValue(56,1,msg->data,HIGHTLOW_MODE,CAN_MODE); 
			}
			break;			
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
static void analysisVol(uint32_t canID,const uint8_t *data)
{
	int i;
	//����Э�����õ���ѹλ��             Byte0~Byte1 ��֡��ʼ�����ѹ���
	uint16_t volStartIndex = (data[0]-1)*3;
	
	/******����Э�����*******/
	for(i=0;i<3;++i)
	{
		if(volStartIndex < gRealData.subSysData[0].singleVolCnt)
		{
			 gRealData.single_vol[volStartIndex++] = calcRealValue(16+i*16,16,0.001,0,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);		
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
static void analysisTemper(uint32_t canID,const uint8_t *data)
{
	int i;
	uint16_t temperStartIndex = 0;
	if(data[0]%2 != 0)
	{
		temperStartIndex = (data[0]*4-4);			//�������¶����												
		for(i=0;i<6;++i)
		{
			if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
			{
				gRealData.single_temper[temperStartIndex++] = calcRealValue(16+i*8,8,1,-50,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
			}
		}
	}
	else {
		temperStartIndex = (data[0]*4-2);			//�������¶����												
		for(i=0;i<2;++i)
		{
			if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
			{
				gRealData.single_temper[temperStartIndex++] = calcRealValue(16+i*8,8,1,-50,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
			}
		}
	}

}