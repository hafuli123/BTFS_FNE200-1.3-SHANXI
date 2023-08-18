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
#include "math.h"

const char* carType = "SDHN_FV2.04";
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

int datato(uint8_t data)
{
    int ret = 0 ;
    int i=1;
    while(i<8)
    {  
        ret += (data/(int)(pow(16,i)))*pow(10,i);
        i++;
    }
    ret = ret + data % 16;
    return ret;
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
	uint8_t bValTmp = 0;
    int mile1 = 0;
    int mile2 =0;
    int mile3 =0;
//	gRealData.carState = CARSTA_START;
	switch(msg->id)
	{
		case 0x1028FF00:
			{
				//��ѹDC/DC״̬
				n8Val = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n8Val == 1)
					gRealData.dc2dcState_highVol = DCDC_WORK;
				else
					gRealData.dc2dcState_highVol = DCDC_BREAK;
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

		case 0x18FF43A8:
			{
				//����״̬
				n8Val = calcCanValue(0,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val)
					gRealData.carState = CARSTA_START;							//1		����
				else if(2 == n8Val)
					gRealData.carState = CARSTA_STOP;							//2		Ϩ��	
				//����ģʽ
				n8Val = calcCanValue(4,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val)
				{
					gRealData.operationState = EV;								//0	����
				}
				else if(1 == n8Val)
				{
					gRealData.operationState = EV_FV;							//1	�춯
				}
				else if(2 == n8Val)
					gRealData.operationState = FV;								//2	ȼ��	
				//�ƶ�̤��״̬			
				n8Val = calcRealValue(8,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,1);//TODO...����ƶ�̤��Ϊ�г�ֵ����ڶ�������Ϊ-1�����Ϊ״ֵ̬����ڶ�������Ϊ1						
				//�������״̬
				n8Val = calcCanValue(12,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(3 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;							//0 �ر�
				else if(1 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;						//1	�ĵ�
				else if(2 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;					//2	����
				else if(4 == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;						//3	׼��
				//ȼ��������
				gRealData.batFuelConsumption = calcRealValue(24,8,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//DCDC״̬	
				n8Val = calcCanValue(22,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val)
					gRealData.dc2dcState = DCDC_WORK;																	//2	������
				else
					gRealData.dc2dcState = DCDC_BREAK;																//else		������	                
			}
			break;
			
		case 0x18FF47A8:
		{
				//�ܵ���
				gRealData.total_current = calcRealValue(24,16,0.025,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
					//SOC
				gRealData.soc = calcRealValue(8,16,0.0025,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ܵ�ѹ
				gRealData.total_volt = calcRealValue(0,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
							//���������ֱ��ĸ�ߵ���	
				gRealData.motorData[0].motorCur = calcRealValue(24,16,0.025,-800,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
			
		case 0x18FEDA17:
		{
				//�ۼ����
				gRealData.totalMileage  = calcRealValue(0,32,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//                mile1 = datato(msg->data[0]);
//                mile2 = datato(msg->data[1]);
//                mile3 = datato(msg->data[2]);

//                gRealData.totalMileage = mile1 * 10000;
//                gRealData.totalMileage += mile2 * 100;
//                gRealData.totalMileage += mile3;
            
		}
			
			case 0x1883EFF3:
			{
				//���״̬
				n8Val = calcCanValue(41,2,msg->data,HIGHTLOW_MODE,CAN_MODE);							
				if(0 == n8Val) 
					gRealData.chargeState = NO_CHARGE;						//0 δ���
				else if(1 == n8Val) 
					gRealData.chargeState = STOP_CHARGE;					//1 ͣ�����
				else if(2 == n8Val) 
					gRealData.chargeState = RUN_CHARGE;						//2	�г����
				else if(3 == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;  				//3 ������		
			}
			break;
			
			case 0x18FF44A8:
			{
				//����
				gRealData.speed = calcRealValue(0,16,0.0003906,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//����̤���г�ֵ		
				gRealData.acceleratorVal = calcRealValue(24,8,0.005,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�����ƶ�״̬
				SET_ACCE_BREAK();				
			}
			break;
			
		case 0x1881EFF3:
			{
				//�¶Ȳ��챨��								
				gRealData.tempDiffAlert = calcCanValue(2,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ظ��±���						
				gRealData.batHighTempAlert = calcCanValue(4,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//���ش���װ�����͹�ѹ����					
				gRealData.batHighVolAlert = calcCanValue(6,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//���ش���װ������Ƿѹ����					
				gRealData.batLowVolAlert = calcCanValue(8,2,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				//SOC�ͱ���
				gRealData.socLowAlert = calcCanValue(10,1,msg->data,HIGHTLOW_MODE,CAN_MODE); 				
				//�����ع�ѹ����
				gRealData.singleBatHighVolAlert = calcCanValue(12,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//������Ƿѹ����					 
				gRealData.singleBattLowVolAlert = calcCanValue(14,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC���߱���						
				gRealData.socHighAlert = calcCanValue(22,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//SOC���䱨��
				n8Val = calcCanValue(24,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHopAlert = (n8Val == 0? 0:1);				
				//�ɳ�索��ϵͳ��ƥ�䱨��
				n8Val = calcCanValue(25,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batNotMatchAlert = (n8Val == 0? 0:1);			
				//��ص���һ���Բ��				 
				gRealData.singleBatPoorConsisAlert = calcCanValue(18,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��Ե����							 
				gRealData.insulationFailtAlert= calcCanValue(16,2,msg->data,HIGHTLOW_MODE,CAN_MODE);	
				//��ѹ����״̬����
				n8Val = calcCanValue(26,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(0 == n8Val)
					gRealData.highPressInterlockStateAlert = 0;		//0	�޹���
				else if(1 == n8Val)
					gRealData.highPressInterlockStateAlert = 3;		//1	���ϵȼ���						
				//���ش���װ�����͹��䱨��
				n8Val = calcCanValue(20,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batOverCharge = (n8Val == 0? 0:1);	
			}
			break;
			
		
		case 0x1882EFF3:
		{
			//�ɳ���������
			gRealData.batFaultCnt = calcCanValue(0,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x18FF35F1:
		{
			//���������������
			gRealData.motorFaultCnt = calcCanValue(8,6,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x1884EFF3:
			{				
				//�ɳ�索��װ�õ�ѹ
				gRealData.subSysData[0].subSysVol = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ɳ�索��װ�õ���
				gRealData.subSysData[0].subSysCur = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;
			
		case 0x1885EFF3:
			{
				//��Ե����
				gRealData.mohm = calcRealValue(32,16,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);				
			}
			break;

		case 0x1888EFF3:
			{
				//��ߵ�ѹ�����ϵͳ��
				gRealData.maxVolPack_index = 1;
				//��͵�ѹ�����ϵͳ��
				gRealData.minVolPack_index = 1;
				//����¶���ϵͳ��
				gRealData.maxTemperPack_index = 1;
				//����¶���ϵͳ��
				gRealData.minTemperPack_index = 1;
				//��ߵ�ѹ�������
				gRealData.maxVol_index = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��͵�ѹ�������
				gRealData.minVol_index = calcCanValue(32,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				// �ɳ�索����ϵͳ����			
				gRealData.subBatSysCnt = 1;
				// �ɳ�索����ϵͳ��			ֱ�ӷ�1
				gRealData.subSysData[0].subSysIdx = 1;	
			}
			break;
			
		case 0x18E3EFF3:
		{
				//����������
				gRealData.subSysData[0].singleVolCnt = 324;

				gRealData.subSysData[0].singleTemCnt = 	calcCanValue(24,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
			
		case 0x1887EFF3:
		{
			//�����ѹ���ֵ
			gRealData.max_singleVol = calcRealValue(0,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);		
			//�����ѹ���ֵ
			gRealData.min_singleVol = calcRealValue(16,16,0.001,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);	
		}
		break;
		
		case 0x1886EFF3:
		{
			//����¶�̽�뵥�����
			gRealData.maxTemper_index = calcCanValue(32,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//����¶�ֵ
			gRealData.max_singleTemper = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//����¶�̽�����
			gRealData.minTemper_index = calcCanValue(48,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//��͵��¶�
			gRealData.min_singleTemper = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		break;
		
		case 0x18FF48A8:
			{					
				uint8_t bValTmp; 		 
				//��λ
				n8Val = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);
				bValTmp = (gRealData.stall & 0xF0);
				if(2 == n8Val  )		//����0������ʱ��λ��0����
					gRealData.stall = (bValTmp | NEUTRAL); 													//0	N
				else if(1 == n8Val  )	
					gRealData.stall = (bValTmp | DIRVE);													//1	D
				else if(3 == n8Val  )
					gRealData.stall = (bValTmp | REVERSE);													//3	R
				else if(5 == n8Val  )
					gRealData.stall = (bValTmp | PART);														//3	P

			}
			break;
		

		case 0x18FF99A8:
			{
				//��߱����ȼ�
				gRealData.alarmLevel = calcCanValue(0,4,msg->data,HIGHTLOW_MODE,CAN_MODE);		
				//DC-DC�¶ȱ���
				n8Val = calcCanValue(40,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
 				if(4 == n8Val)
					gRealData.dc2dcTemAlert = 1;																	//4: 1������������������
				else
					gRealData.dc2dcTemAlert = 0;				
				//DC-DC״̬����
				n8Val = calcCanValue(48,4,msg->data,HIGHTLOW_MODE,CAN_MODE);				
				if( 21 == n8Val){
					gRealData.dc2dcStateAlert = 1;					
				}else{
					gRealData.dc2dcStateAlert = 0;
				}

			}
			break;
			
		case 0x18f0010b:
		{
				//�ƶ�ϵͳ����
				n8Val = calcCanValue(44,2,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(1 == n8Val)
					gRealData.brakingAlert = 1;							//1  �����ȼ�һ
				else if(0 == n8Val)
					gRealData.brakingAlert = 0;							//����	�ޱ���			
		}
		break;
			
		case 0x18FF46A8:
			{
				//��������������¶�			
				gRealData.motorData[0].motorCtrTemp = calcRealValue(40,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);			
				//��������¶�						
				gRealData.motorData[0].motorTemp = calcRealValue(32,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��������������ѹ			
				gRealData.motorData[0].motorVol = calcRealValue(56,8,4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ת��							
				gRealData.motorData[0].motorSpeed = calcRealValue(0,16,1,-6000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ת��						
				gRealData.motorData[0].motorTorsion = calcRealValue(0,16,0.25,-8000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18ff81f0:
			{
				//��������������¶ȱ���		
				n16Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(3 == n16Val)
					gRealData.motorCtrTemAlert = 2;					//3 ��2��
				else
					gRealData.motorCtrTemAlert = 0;					//����	�޹���
				
				//��������¶ȱ���				
				n16Val = calcCanValue(8,16,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(4 == n16Val)
					gRealData.motorTempAlert = 2;						//4 : 2��
				else
					gRealData.motorTempAlert = 0;						//����	�޹���
						
			}
			break;
			
		case 0x00FF020C:
			{
								//ȼ�ϵ�ص�ѹ					��ֵС��0ʱ����0������ת��
			n16Val = calcRealValue(0,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val < 0)
					gRealData.fuelBatVol = 0;
				else
					gRealData.fuelBatVol = n16Val;
				//ȼ�ϵ�ص���					��ֵС��0ʱ����0������ת��
				n16Val = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(n16Val < 0)
					gRealData.fuelBatCur = 0;
				else
					gRealData.fuelBatCur = n16Val;
			}
			break;
	
		case 0x18E1EFF3:	
			{				
				//�ɳ�索��ϵͳ���볤��
				gRealData.rechargeSysCodeLen = calcCanValue(8,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ɳ�索��ϵͳ����
//				send_RechargeSysCode(msg->id,msg->data);
			}
			break;
			
		case 0x18C1EFF3:
			{
				//���������ѹ
				analysisVol(msg->id,msg->data);	
			}
			break;		
		case 0x18C2EFF3:											
			{
				//���������¶�
				analysisTemper(msg->id,msg->data);
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
	uint16_t volStartIndex = (calcCanValue(0,16,data,HIGHTLOW_MODE,CAN_MODE) - 1);
	
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
	//�������¶����												
	uint16_t temperStartIndex = (data[0]-1);
	
	/******����Э�����*******/
	for(i=0;i<6;++i)
	{
		if(temperStartIndex < gRealData.subSysData[0].singleTemCnt)
		{
			gRealData.single_temper[temperStartIndex++] = calcRealValue(16+i*8,8,1,-40,CALC_MODE,data,HIGHTLOW_MODE,CAN_MODE);
		}
	}
}