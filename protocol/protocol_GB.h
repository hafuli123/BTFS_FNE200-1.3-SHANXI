/*
�ļ���protocol_GB.h
���ܣ�����Э���һЩ����
���ڣ�2013-06-24
*/
#ifndef __PROTOCOL_GB_H__ 
#define __PROTOCOL_GB_H__

#include "pdu.h"

//CAN����ģʽ����ͬ�ĸ�ʽ����ͬ�Ľ�����ʽ
typedef enum
{
    INTEL = 0x01,			//Ӣ�ض���ʽ
    MOTOROLA = 0x02		//Ħ��������ʽ
}CANANA_MODEL;

//�ֽڸߵ�λ
typedef enum
{
    HIGHTLOW = 0x01,	//��λ��ǰ��λ�ں�
    LOWHIGHT = 0x02		//��λ��ǰ��λ�ں�
}BYTE_MODEL;

//ƫ����ϵ������ģʽ
typedef enum
{
    FACTOR_OFFSET = 0x01,		//�ȳ���ϵ���ټ���ƫ����
    OFFSET_FACTOR = 0x02		//�ȼ���ƫ�����ٳ���ϵ��
}CALC_MODEL;

/*��ŵ�״̬*/
typedef enum{
	ABNORMAL = 0xFE,		/*�쳣*/
	INVALID = 0xFF			/*��Ч*/
}DataState;

/*����ģʽ*/
typedef enum{
	EV = 0x01,		/*����*/
	EV_FV = 0x02,			/*�춯*/
	FV = 0x03,			/*ȼ��*/
}RunModle;

/*����ģʽ*/
typedef enum{
	NEUTRAL = 0x00,		/*�յ�*/
	REVERSE = 0x0D,		/*����*/
	DIRVE = 0x0E,			/*ǰ����*/
	PART = 0x0F				/*P��*/
}Stall;

/*�������״̬*/
typedef enum{
	MOTOR_CONSUME = 0x01,				/*�ĵ�*/
	MOTOR_GENERATION = 0x02,		/*����*/
	MOTOR_OFF = 0x03,						/*�ر�*/
	MOTOR_READY = 0x04,					/*׼��״̬*/
	MOTOR_ABNORMAL = 0xFE,			/*�쳣*/
	MOTOR_NVALID = 0xFF					/*��Ч*/
}MotorState;

typedef enum{
	CARSTA_START = 0x01,				/*����*/
	CARSTA_STOP = 0x02,					/*Ϩ��*/
	CARSTA_OTHER = 0x03,				/*����*/
	CARSTA_ABNORMAL = 0xFE,			/*�쳣*/
	CARSTA_NVALID = 0xFF				/*��Ч*/
}CarState;

typedef enum{
	STOP_CHARGE = 0x01,				/*ͣ�����*/
	RUN_CHARGE = 0x02,				/*�г����*/
	NO_CHARGE = 0x03,					/*δ���*/
	CHARGE_FINISH = 0x04,			/*������*/
	CHARGE_ABNORMAL = 0xFE,		/*�쳣*/
	CHARGE_NVALID = 0xFF			/*��Ч*/
}ChargeState;

typedef enum{
	DCDC_WORK = 0x01,				/*DCDC����*/
	DCDC_BREAK = 0x02,			/*DCDC�Ͽ�*/
	DCDC_ABNORMAL = 0xFE,		/*�쳣*/
	DCDC_INVALID = 0xFF			/*��Ч*/
}DCDCState;

typedef enum{
	ENGINE_START = 0x01,				/*����*/
	ENGINE_STOP = 0x02,					/*Ϩ��*/
	ENGINE_ABNORMAL = 0xFE,			/*�쳣*/
	ENGINE_NVALID = 0xFF				/*��Ч*/
}ENGINESTATE;

/********�����ϱ���Ϣ������*******
				//����״̬
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( == n8Val) 
					gRealData.carState = CARSTA_START;
				else if( == n8Val) 
					gRealData.carState = CARSTA_STOP;
				else if( == n8Val) 
					gRealData.carState = CARSTA_OTHER;
				else if( == n8Val) 
					gRealData.carState = CARSTA_ABNORMAL;
				else 
					gRealData.carState = CARSTA_NVALID;
				//���״̬
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( == n8Val) 
					gRealData.chargeState = STOP_CHARGE;
				else if( == n8Val) 
					gRealData.chargeState = RUN_CHARGE;
				else if( == n8Val) 
					gRealData.chargeState = NO_CHARGE;
				else if( == n8Val) 
					gRealData.chargeState = CHARGE_FINISH;
				else if( == n8Val) 
					gRealData.chargeState = CHARGE_ABNORMAL;
				else 
					gRealData.chargeState = CHARGE_NVALID
				//����ģʽ
				gRealData.operationState = EV;
				//����
				gRealData.speed = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�����
				gRealData.totalMileage = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ܵ�ѹ
				gRealData.total_volt = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysVol = gRealData.total_volt;
				//�ܵ���
				gRealData.total_current = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.subSysData[0].subSysCur = gRealData.total_current;
				//SOC
				gRealData.soc = (uint8_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����DCDC״̬
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( == n8Val) 
					gRealData.dc2dcState = DCDC_WORK;
				else if( == n8Val) 
					gRealData.dc2dcState = DCDC_BREAK;
				else if( == n8Val) 
					gRealData.dc2dcState = DCDC_ABNORMAL;
				else 
					gRealData.dc2dcState = DCDC_INVALID; 
				//��λ
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				uint8_t bValTmp = (gRealData.stall & 0xF0);
				if( == n8Val) 
					gRealData.stall = (bValTmp | NEUTRAL);
				else if( == n8Val) 
					gRealData.stall = (bValTmp | PART);
				else if( == n8Val) 
					gRealData.stall = (bValTmp | REVERSE);
				else if( == n8Val) 
					gRealData.stall = (bValTmp | DIRVE);
				else {
					if(n8Val>=1 && n8Val<=6) 
						gRealData.stall = n8Val;
					else 
						gRealData.stall = INVALID;
				}
				//��Ե����
				gRealData.mohm = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����̤��
				gRealData.acceleratorVal = (uint8_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ƶ�̤��
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				SET_BREAK_STATE(n8Val,);
				//�����ƶ�״̬
				SET_ACCE_BREAK();


				//�������״̬
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_CONSUME;
				else if( == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_GENERATION;
				else if( == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_OFF;
				else if( == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_READY;
				else if( == n8Val) 
					gRealData.motorData[0].motorState = MOTOR_ABNORMAL;
				else 
					gRealData.motorData[0].motorState = MOTOR_NVALID; 
				//�����������
				gRealData.motorCnt = MOTORCOUNT;
				//����������
				gRealData.motorData[0].motorIdx = 1;
				//��������������¶�
				gRealData.motorData[0].motorCtrTemp = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ת��
				gRealData.motorData[0].motorSpeed = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�������ת��
				gRealData.motorData[0].motorTorsion = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��������¶�
				gRealData.motorData[0].motorTemp = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��������������ѹ
				gRealData.motorData[0].motorVol = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//���������ֱ��ĸ�ߵ���
				gRealData.motorData[0].motorCur = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);


				//������״̬
				n8Val = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if( == n8Val) 
					gRealData.engineState = ENGINE_START;
				else if( == n8Val) 
					gRealData.engineState = ENGINE_STOP;
				else if( == n8Val) 
					gRealData.engineState = ENGINE_ABNORMAL;
				else 
					gRealData.engineState = ENGINE_NVALID;
				//����ת��
				gRealData.crankshaftSpeed = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//ȼ��������
				gRealData.fuelConsumption = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				
				//��ߵ�ѹ�����ϵͳ��
				gRealData.maxVolPack_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ߵ�ѹ��ص������
				gRealData.maxVol_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ص����ѹ���ֵ
				gRealData.max_singleVol = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��͵�ѹ�����ϵͳ�� 
				gRealData.minVolPack_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��͵�ѹ��ص������
				gRealData.minVol_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ص����ѹ���ֵ
				gRealData.min_singleVol = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶���ϵͳ��
				gRealData.maxTemperPack_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�̽�����
				gRealData.maxTemper_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�ֵ
				gRealData.max_singleTemper = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶���ϵͳ��
				gRealData.minTemperPack_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//����¶�̽�����
				gRealData.minTemper_index = calcCanValue(,8,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��͵��¶�
				gRealData.min_singleTemper = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);

				//�ɳ�索����ϵͳ���� ������MAX_BAT_SUBSYS_NUM
				gRealData.subBatSysCnt = BATT_PACK_COUNT;
				//�ɳ�索����ϵͳ��
				gRealData.subSysData[0].subSysIdx = 1;
				//�ɳ�索��װ�õ�ѹ
				gRealData.subSysData[0].subSysVol = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�ɳ�索��װ�õ���
				gRealData.subSysData[0].subSysCur = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//��ϵͳ����������
				gRealData.subSysData[0].singleVolCnt = BATTERY_COUNT;
				//�����ʼ���(�ܵ���б�, ��0��ʼ)
				gRealData.subSysData[0].singleVolStartIdx = 0;
				//��ϵͳ�¶�̽�����
				gRealData.subSysData[0].singleTemCnt = TEMPER_PROBE_COUNT;
				//�¶���ʼ���(���¶��б���0��ʼ)
				gRealData.subSysData[0].singleTemStartIdx = 0;

				//��߱����ȼ�
				gRealData.alarmLevel = calcCanValue(,,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�¶Ȳ��챨��
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.tempDiffAlert = (n8Val == 0? 0:1);
				//��ظ��±���
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batHighTempAlert = (n8Val == 0? 0:1);
				//���ش���װ�����͹�ѹ����
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batHighVolAlert = (n8Val == 0? 0:1);
				//���ش���װ������Ƿѹ����
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batLowVolAlert = (n8Val == 0? 0:1);
				//SOC�ͱ���
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socLowAlert = (n8Val == 0? 0:1);
				//�����ع�ѹ����
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.singleBatHighVolAlert = (n8Val == 0? 0:1);
				//������Ƿѹ����
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.singleBattLowVolAlert = (n8Val == 0? 0:1);
				//SOC���߱���
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHighAlert = (n8Val == 0? 0:1);
				//SOC���䱨��
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.socHopAlert = (n8Val == 0? 0:1);
				//�ɳ�索��ϵͳ��ƥ�䱨��
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batNotMatchAlert = (n8Val == 0? 0:1);
				//��ص���һ���Բ��
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.singleBatPoorConsisAlert = (n8Val == 0? 0:1);
				//��Ե����
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.insulationFailtAlert = (n8Val == 0? 0:1);
				//DC-DC�¶ȱ���
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.dc2dcTemAlert = (n8Val == 0? 0:1);
				//�ƶ�ϵͳ����
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.brakingAlert = (n8Val == 0? 0:1);
				//DC-DC״̬����
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.dc2dcStateAlert = (n8Val == 0? 0:1);
				//��������������¶ȱ���
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.motorCtrTemAlert = (n8Val == 0? 0:1);
				//��ѹ����״̬����
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.highPressInterlockStateAlert = (n8Val == 0? 0:1);
				//��������¶ȱ���	
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.motorTempAlert = (n8Val == 0? 0:1);
				//���ش���װ�����͹���	
				n8Val = calcCanValue(,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.batOverCharge = (n8Val == 0? 0:1);

				//�����ѹ
				gRealData.single_vol[] = calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//�����¶�
				gRealData.single_temper[] = (int16_t)calcRealValue(,,,,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//������� ����
				if(msg->id >=TEMPER_1 && msg->id <=TEMPER_3)
				{
					analysisTemper(msg->id,msg->data);
				}
				else if(msg->id >=VOLGAGE_1 && msg->id <=VOLGAGE_13)
				{
					analysisVol(msg->id,msg->data);
				}
*********************************/

/******����״̬******/
/*˵�� 
	val ����CAN���ݵõ���ֵ
	start	��������	�б�����ֵ
	stop	Ϩ��	�б�����ֵ
	other   ����״̬	�б�����ֵ
	abnormal �쳣
****************/
#define SET_CAR_STATE(val,start,stop,other,abnormal) {\
	if(val == start) gRealData.carState = 0x01;\
	else if(val == stop) gRealData.carState = 0x02;\
	else if(val == other) gRealData.carState = 0x03;\
	else if(val == abnormal) gRealData.carState = 0x0FE;\
	else gRealData.carState = INVALID; }
	
#define SET_CAR_STATE1(val,start1,start2,stop,other,abnormal) {\
	if(val == start1 || val == start2) gRealData.carState = 0x01;\
	else if(val == stop) gRealData.carState = 0x02;\
	else if(val == other) gRealData.carState = 0x03;\
	else if(val == abnormal) gRealData.carState = 0x0FE;\
	else gRealData.carState = INVALID; }
	
	
/*******��ŵ�״̬*******/
/*˵�� 
	val ����CAN���ݵõ���ֵ
	stopChange	ͣ������� ������ֵ
	runChange		��ʻ����� ������ֵ
	noChange		δ��� �б�����ֵ
	finish 			������ �б�����ֵ
	abnormal �쳣
*************************/
#define SET_CHARGE_STATE(val,stopChange,runChange,noChange,finish,abnormal) {\
	if(val == stopChange) gRealData.chargeState = 0x01;\
	else if(val == runChange) gRealData.chargeState = 0x02;\
	else if(val == noChange) gRealData.chargeState = 0x03;\
	else if(val == finish) gRealData.chargeState = 0x04;\
	else if(val == abnormal) gRealData.chargeState = 0x0FE;\
	else gRealData.chargeState = INVALID; }

/*˵�� 
	val ����CAN���ݵõ���ֵ
	stopChange1��stopChange2	ͣ������� ������ֵ
	runChange	��ʻ����� ������ֵ
	noChange	δ��� �б�����ֵ
	finish 	������ �б�����ֵ
	abnormal �쳣
*************************/
#define SET_CHARGE_STATE1(val,stopChange1,stopChange2,runChange,noChange,finish,abnormal) {\
	if(val == stopChange1 || val == stopChange2) gRealData.chargeState = 0x01;\
	else if(val == runChange) gRealData.chargeState = 0x02;\
	else if(val == noChange) gRealData.chargeState = 0x03;\
	else if(val == finish) gRealData.chargeState = 0x04;\
	else if(val == abnormal) gRealData.chargeState = 0x0FE;\
	else gRealData.chargeState = INVALID; }

	/*˵�� 
	val ����CAN���ݵõ���ֵ
	stopChange1��stopChange2	ͣ������� ������ֵ
	runChange	��ʻ����� ������ֵ
	noChange1 noChange2	δ��� �б�����ֵ
	finish 	������ �б�����ֵ
	abnormal �쳣
*************************/
#define SET_CHARGE_STATE2(val,stopChange1,stopChange2,runChange,noChange1,noChange2,finish,abnormal) {\
	if(val == stopChange1 || val == stopChange2) gRealData.chargeState = 0x01;\
	else if(val == runChange) gRealData.chargeState = 0x02;\
	else if(val == noChange1 || val == noChange2) gRealData.chargeState = 0x03;\
	else if(val == finish) gRealData.chargeState = 0x04;\
	else if(val == abnormal) gRealData.chargeState = 0x0FE;\
	else gRealData.chargeState = INVALID; }

/*******��λ����*******/
/*˵����
	val ����CAN���ݵõ���ֵ
	n1,n2,n3	�յ��ж�����
	r1,r2,r3	�����ж�����
	d1,d2,d3 	ǰ�������ж�����
	û���ж�����ֵΪ-1
  ���ֻ����һ���ж����ݣ��������ظ���д������˵�յ��ж�������ֵΪ0����n1,n2,n3����0 
	�����λ��1-6��������Щ��������ֵΪ-1*/
#define SET_STALL_3(val,n1,n2,n3,p1,p2,p3,r1,r2,r3,d1,d2,d3) {\
	unsigned char bValTmp = (gRealData.stall & 0xF0);\
	if(val == n1||val == n2||val == n3) gRealData.stall = (bValTmp | NEUTRAL);\
	else if(val == p1||val == p2||val == p3) gRealData.stall = (bValTmp | PART);\
	else if(val == r1||val == r2||val == r3) gRealData.stall = (bValTmp | REVERSE);\
	else if(val == d1||val == d2||val == d3) gRealData.stall = (bValTmp | DIRVE);\
	else {if(val>=1 && val<=6) gRealData.stall = val;else gRealData.stall = INVALID;}}
	
#define SET_STALL_2(val,n1,n2,p1,p2,r1,r2,d1,d2) {\
	unsigned char bValTmp = (gRealData.stall & 0xF0);\
	if(val == n1||val == n2) gRealData.stall = (bValTmp | NEUTRAL);\
	else if(val == p1||val == p1) gRealData.stall = (bValTmp | PART);\
	else if(val == r1||val == r2) gRealData.stall = (bValTmp | REVERSE);\
	else if(val == d1||val == d2) gRealData.stall = (bValTmp | DIRVE);\
	else {if(val>=1 && val<=6) gRealData.stall = val;else gRealData.stall = INVALID;}}
	
#define SET_STALL_1(val,n1,p1,r1,d1) {\
	unsigned char bValTmp = (gRealData.stall & 0xF0);\
	if(val == n1) gRealData.stall = (bValTmp | NEUTRAL);\
	else if(val == p1) gRealData.stall = (bValTmp | PART);\
	else if(val == r1) gRealData.stall = (bValTmp | REVERSE);\
	else if(val == d1) gRealData.stall = (bValTmp | DIRVE);\
	else {if(val>=1 && val<=6) gRealData.stall = val;else gRealData.stall = INVALID;}}
	
/*******�����ƶ�̤��ֵ*******/
/*˵����
	val ����CAN���ݵõ���ֵ
	bValid �޾����г�ֵʱ���ƶ���Ч״̬���ж�����ֵ��������г�ֵ����bValid = -1
***************************/
#define SET_BREAK_STATE(val,bValid) {\
	if(bValid == -1) {if(val>=0 && val<=101) gRealData.brakingVal = val; \
		else gRealData.brakingVal=INVALID;}\
	else {if(val == bValid) gRealData.brakingVal = 0x65; \
				else gRealData.brakingVal = 0;}}
	
/********�����������ƶ���Ч��Ч**********/
#define SET_ACCE_BREAK() {\
	if(gRealData.brakingVal > 0 && 0xFF != gRealData.brakingVal) gRealData.stall |= 0x10;\
	else gRealData.stall &= 0x2F;\
	if(gRealData.acceleratorVal > 0 && 0xFF != gRealData.acceleratorVal) gRealData.stall |= 0x20;\
	else gRealData.stall &= 0x1F;}
	
/*******����DCDC״̬*******/
/*˵����
	val ����CAN���ݵõ���ֵ
	work1,work2ֵΪ����	�б�����ֵ
	break1,break2ֵΪ�Ͽ� �б�����ֵ
	abnormal �쳣
***************************/
#define SET_DCDC_STATE(val,work1,work2,break1,break2,abnormal) {\
	if(val == work1 || val == work2) gRealData.dc2dcState = 0x01;\
	else if(val == break1||val == break2) gRealData.dc2dcState = 0x02;\
	else if(val == abnormal) gRealData.dc2dcState = 0xFE;\
	else gRealData.dc2dcState = INVALID; }
	
/******�����������״̬*****/
/*˵����
	val ����CAN���ݵõ���ֵ
	motoridx��ʾ�ǵڼ������,��0��ʼ
	consu1,consu2		�ĵ�	�б�����ֵ
	gene1 gene2			����	�б�����ֵ
	off1		�ر�״̬	�б�����ֵ
	ready1	׼��״̬	�б�����ֵ	
	abnormal �쳣
****************************/
#define SET_MOTOR_STATE(val,motoridx,consu1,consu2,gene1,gene2,off1,ready1,abnormal) {\
	if(val == consu1 || val == consu2) gRealData.motorData[motoridx].motorState = 0x01;\
	else if(val == gene1 || val == gene2) gRealData.motorData[motoridx].motorState = 0x02;\
	else if(val == off1) gRealData.motorData[motoridx].motorState = 0x03;\
	else if(val == ready1) gRealData.motorData[motoridx].motorState = 0x04;\
	else if(val == abnormal) gRealData.motorData[motoridx].motorState = 0xFE;\
	else gRealData.motorData[motoridx].motorState = INVALID; }
	
#endif
	
