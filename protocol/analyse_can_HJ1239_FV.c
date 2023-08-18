/*
�ļ���analyse_can_HJ1239_FV.c
���ܣ�HJ1239 ȼ�ͳ�CAN�������
���ڣ�2022/5/20
��˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
���ߣ�LGC
*/

#include "fun_can.h"
#include "Fun_Net.h"
#include "bsp_rtc.h"
#include "protocol_GB.h"
#include "protocol_GB17691.h"

#include "cmsis_os2.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stdint.h"

#include "bsp_storage.h"
#include "algo_verify.h"


static const char* carType = "F4_FNE200V103_FV_V1.0";

static const uint8_t CAN1_USED_ANA = 1;												/*�Ƿ�ʹ��CAN1*/
static const uint8_t CAN2_USED_ANA = 0;												/*�Ƿ�ʹ��CAN1*/
static const uint8_t CAN3_USED_ANA = 0;												/*�Ƿ�ʹ��CAN1*/

static const uint32_t CAN1_BAUDRATE		=		250000;							/*CAN1������*/
static const uint32_t CAN2_BAUDRATE		=		250000;							/*CAN2������*/
static const uint32_t CAN3_BAUDRATE		=		250000;							/*CAN2������*/

const static BYTE_MODEL HIGHTLOW_MODE = LOWHIGHT;							//���ݸߵ�λ
const static CANANA_MODEL CAN_MODE = INTEL;										//CAN����ģʽ����ΪIntel��ʽ��Motorola��ʽ
const static CALC_MODEL CALC_MODE = FACTOR_OFFSET;						//CAN����ƫ������ϵ������ģʽ

static void Udstask(void);																		//OBD������
static void udsAnalyse(CAN_msg *msg);													//OBD���ݽ���
static uint8_t emissionLevel = 0;							//�жϹ���͹��� 1Ϊ����
//static uint8_t catype = 5;																		//�жϹ���͹��� 6Ϊ����
static uint8_t sendStaSecond;																	//���ͽ���CAN����

static uint8_t scrInReady = 0;																//SCR���ready,���������Ż�
static uint8_t scrOutReady = 0;																//SCR����ready,���������Ż�


static uint32_t sendCanTimeStamp = 0;													//����CAN����ʱ���
static uint32_t sendAuxStamp = 0;															//���͸�������ʱ���

static uint32_t engTotalHoursOfOperationStamp = 0;						//������������ʱ��
static uint32_t actAreaInjectValStamp = 0;										//ʵ������������
static uint32_t totalUreaUsedStamp1 = 0;											//������������,ID1
static uint32_t totalUreaUsedStamp2 = 0;											//������������,ID2
static uint32_t engReferenceTorqueStamp = 0;									//�������ο�Ť��

static uint32_t sendDateTimeStamp;														//����ʱ������
static uint32_t sendOxygenStamp;															//27145�������������ʱ������
static uint32_t sendUdsId = 0;																//����UDS����ID���ֱ�׼֡����չ֡
static uint8_t afterOtherCnt = 0;															//����ռ�����3����ն˲��ܷ���

static uint32_t getObdDiagProtIdx = 2;												//��̽OBD���Э��˳�� ISO27145 ISO15765 SAE J1939

static uint32_t speedTime = 0;																//����ֹͣ��ʱ�ж�
static uint32_t engSpeedTime = 0;															//������ת��ֹͣ��ʱ�ж�
static uint8_t getOxygenFlag = 0;															//
uint8_t gUploadCan = 0;																				/*�ϴ�CAN��־ 0:���ϴ� 1:���յ��ϴ����� 2:���ڲɼ� 3:��ʼ�ϴ�*/

void iniFvCanData(void)
{
	gSysPara.can1_baudrate = CAN1_BAUDRATE;
	gSysPara.can2_baudrate = CAN2_BAUDRATE;	
	gSysPara.can3_baudrate = CAN3_BAUDRATE;
	gSysPara.can1_used = CAN1_USED_ANA;
	gSysPara.can2_used = CAN2_USED_ANA;
	gSysPara.can3_used = CAN3_USED_ANA;
	CAR_TYPE = carType;
	gSysPara.carType = 1;			//��ʼ��Ĭ�Ϲ���
}

/*
*********************************************************************************************************
*	�� �� ��: calcExtremum
*	����˵��: CAN���ݽ�������λ��ǰ
*	��    ��:  CAN_msg  	CAN�ṹ��
*	�� �� ֵ: ��
*********************************************************************************************************
Դ��ַ��ʶ	װ��
Hex	Dec	
0x00	0	���������Ƶ�Ԫ
0x03	3	����ϵͳ���Ƶ�Ԫ(������)
0x0B	11	���ɲ��ϵͳ (EBS)
0x0F	15	������������
0x10	16	����ϵͳ������
0x17	23	�Ǳ�
0x21	33	����
0x24	36	PTO
0x27	39	������������
0x29	41	����������
0x2B	43	�������ϵͳ
0x3D	61	���������
0xEE	238	ת�ٱ�
0xF9	249	������ϣ��ۺ���񹤾�
*/
void unpackFvCAN(uint8_t ch,CAN_msg *msg)
{
	float fval;
	uint32_t pgn = (msg->id & 0x00FFFF00) >> 8;
	uint8_t sourseAddr = msg->id & 0x000000FF;
	if((pgn & 0xF000) != 0xF000)
	{
		pgn &= 0xFF00;
	}
	Udstask();
	udsAnalyse(msg);
	//���ͳ���϶���������
	switch(msg->id)
	{
		case 0x18FFFB97:
			{
				gRealData.motorData[0].motorLoad = calcRealValue(0,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FE2097:
			{
				gRealData.motorData[0].motorSpeed = calcRealValue(0,16,1,-20000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0x18FE4197:
			{
				gRealData.total_volt = calcRealValue(0,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.total_current = calcRealValue(16,16,0.1,-1000,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gRealData.soc = calcRealValue(32,8,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
	}
	switch(pgn)
	{
		case 0xFE6C://0x0CFE6CEE 0xEE:���ټƻ���̱�
			{
				//���� ���Ȳ�����̱�
				if(gTerminalState.speed <= 1 || gTerminalState.speed == msg->id || sourseAddr == 0xEE)
				{
					speedTime = osKernelGetTickCount();
					fval = calcRealValue(48,16,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(fval >= 0 && fval < 250.0f)
					{
						gRealData.speed = fval;
						if(fval > 0)
							gTerminalState.speed = msg->id;
					}
				}
				if(gTerminalState.speed == 0)
					gTerminalState.speed = 1;
			}
			break;
		case 0xFEF1:
			{
				//���� �������Ҫ��ֻ����1λС��
				if(gTerminalState.speed <= 1 || gTerminalState.speed == msg->id)
				{
					speedTime = osKernelGetTickCount();
					fval = calcRealValue(8,16,0.00390625,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(fval >= 0 && fval < 250.0f)
					{
						gRealData.speed = fval;
						if(fval > 0)
							gTerminalState.speed = msg->id;
					}
				}
				if(gTerminalState.speed == 0)
					gTerminalState.speed = 1;
			}
			break;
		case 0xFEF5://18FEF500
			{
				//����ѹ��
				gRealData.barometric = calcRealValue(0,8,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gTerminalState.barometric = msg->id;
			}
			break;	
		case 0xF003:
			{
				//����̤���г�ֵ
				gRealData.acceleratorVal = calcRealValue(8,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xFEE9:
			{
				//�ۼ��ͺ�
				gRealData.EngTotalFuelUsed = calcRealValue(32,32,0.5,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xF023:
			{
				actAreaInjectValStamp = osKernelGetTickCount();
				//ʵ������������//��ȷ��
				gRealData.actAreaInjectVal = calcRealValue(0,16,0.3,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xF024:
			{
				//�ۼ���������//��ȷ��
				gRealData.totalUreaUsed = calcRealValue(24,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				totalUreaUsedStamp1 = 0xFFFFFFFF;
			}
			break;
		case 0xFCBD://��ȷ��
			{
				//�ۼ���������//��ȷ�� ����0.5L �����ܶ�ȡ1.092kg/L
				gRealData.totalUreaUsed = calcRealValue(0,32,540,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				totalUreaUsedStamp2 = 0xFFFFFFFF;
			}
			break;
		case 0xFEE5:
			{
				engTotalHoursOfOperationStamp = 0xFFFFFFFF;
				//������������ʱ��
				gRealData.engTotalHoursOfOperation = calcRealValue(0,32,0.05,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
//		case 0x0000:
//			{
//				if(sourseAddr == 0x0B || sourseAddr == 0x00 || sourseAddr == 0x03)
//				{
//					//������Ť��ģʽ00:Override disabled 01:Speed control 10:Torque control 11:Speed/torque limit control
//					gRealData.engineTorqueMode = calcRealValue(0,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
//				}
//			}
//			break;				
		case 0xF004://0CF00400
			{
				if(sourseAddr == 0)
				{
					engSpeedTime = osKernelGetTickCount();
					//�����������Ť��
					gRealData.engineTorque = calcRealValue(16,8,1,-125,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					//������ת��
					gRealData.engineSpeed = calcRealValue(24,16,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					gTerminalState.engineTorque = msg->id;
					gTerminalState.engineSpeed = msg->id;
					gRealData.engineTorqueMode = calcRealValue(0,4,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				}
			}
			break;
		case 0xFEDF://18FEDF00
			{
				//Ħ��Ť��
				if(sourseAddr == 0x00)
				{
					gRealData.frictionTorque = calcRealValue(0,8,1,-125,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					gTerminalState.frictionTorque = msg->id;
				}
			}
			break;
		case 0xFEF2://18FEF200
			{
				//������ȼ������
				gRealData.engineFuelFlow = calcRealValue(0,16,0.05,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gTerminalState.engineFuelFlow = msg->id;
				//˲ʱ�ͺ�
				gRealData.engFuelRate = calcRealValue(16,16,0.001953125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xF00E://0x18F00E51 �������С�ڵ���0ֵ��Ҫ����Ч 18F00E00
			{
				//SCR ���� NOx ���������ֵ
				gRealData.scrUpperNOxSensor = calcRealValue(0,16,0.05,-200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.engineSpeed <= 0 || gRealData.engineSpeed >= 0xFFFE)
				{
					//������Ϩ�����޷����ŷ�
					gRealData.scrUpperNOxSensor = 0;
				}
				else if(gRealData.scrUpperNOxSensor > 0 && gRealData.scrUpperNOxSensor < 3012.0f && scrInReady == 0 && gRealData.engineSpeed >= 10 && gRealData.engineSpeed < 0xFFFE)//SCR�Ѿ�����
				{
					scrInReady = 1;
				}
				if(scrInReady == 0 && gRealData.engineSpeed >= 10 && gRealData.engineSpeed < 0xFFFE)//SCRδ��������Ч
				{
					gRealData.scrUpperNOxSensor = 0xFFFF;
				}
				else if(scrInReady == 1 && gRealData.scrUpperNOxSensor < 0 && gRealData.scrUpperNOxSensor >= -20)//������������
				{
					gRealData.scrUpperNOxSensor = -gRealData.scrUpperNOxSensor;
				}
			}
			break;
		case 0xF00F://0x18F00F52 �������С�ڵ���0ֵ��Ҫ����Ч 18F00F00
			{
				//SCR ���� NOx ���������ֵ
				gRealData.scrLowerNOxSensor = calcRealValue(0,16,0.05,-200,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.engineSpeed <= 0 || gRealData.engineSpeed >= 0xFFFE)
				{
					//������Ϩ�����޷����ŷ�
					gRealData.scrLowerNOxSensor = 0;
				}
				else if(gRealData.scrLowerNOxSensor > 0 && gRealData.scrLowerNOxSensor < 3012.0f && scrOutReady == 0 && gRealData.engineSpeed >= 10 && gRealData.engineSpeed < 0xFFFE)//SCR�Ѿ�����
				{
					scrOutReady = 1;
				}
				if(scrOutReady == 0 && gRealData.engineSpeed >= 10 && gRealData.engineSpeed < 0xFFFE)//SCRδ��������Ч
				{
					gRealData.scrLowerNOxSensor = 0xFFFF;
				}
				else if(scrOutReady == 1 && gRealData.scrLowerNOxSensor < 0 && gRealData.scrLowerNOxSensor >= -20)//������������
				{
					gRealData.scrLowerNOxSensor = -gRealData.scrLowerNOxSensor;
				}
			}
			break;			
		case 0xFD3E://0x14FD3E3D 18FD3E00
			{
				if(gRealData.engineSpeed >= 10 && gRealData.engineSpeed < 0xFFFE)
				{
					//SCR����¶�
					gRealData.scrInletTemp = calcRealValue(0,16,0.03125,-273,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					//SCR�����¶�
					gRealData.scrOutletTemp = calcRealValue(24,16,0.03125,-273,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				}
			}
			break;
		case 0xFE56://0x18FE56A3 0x18FE563D 18FE5600
			{
				//��Ӧ������
				if(gTerminalState.reagentSurplus <= 1 || gTerminalState.reagentSurplus == msg->id)
				{
					gRealData.reagentSurplus = calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(gRealData.reagentSurplus > 0 && gRealData.reagentSurplus <= 100)
					{
						gTerminalState.reagentSurplus = msg->id;
					}
				}
				if(gTerminalState.reagentSurplus == 0)
					gTerminalState.reagentSurplus = 1;
				//�������¶�
				gRealData.ureaTankTemp = calcRealValue(8,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xFF4C://18FF4C00 ����TD
			{
				//��Ӧ������
				if(gTerminalState.reagentSurplus <= 1 || gTerminalState.reagentSurplus == msg->id)
				{
					gRealData.reagentSurplus = calcRealValue(16,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(gRealData.reagentSurplus > 0 && gRealData.reagentSurplus <= 100)
					{
						gTerminalState.reagentSurplus = msg->id;
					}
				}
				if(gTerminalState.reagentSurplus == 0)
					gTerminalState.reagentSurplus = 1;
			}
			break;
		case 0xFDB2://18FDB200
			{
				//dpfѹ��
				gRealData.dpfPressDiff = calcRealValue(32,16,0.1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				//dpf�����¶�
				gRealData.dpfExhaustTemp = calcRealValue(0,16,0.03125,-273,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			}
			break;
		case 0xFEEE://18FEEE00
			{
				//��������ȴҺ�¶�
				gRealData.engineCoolantTemp = calcRealValue(0,8,1,-40,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gTerminalState.engineCoolantTemp = msg->id;
			}
			break;
		case 0xFEFC://����Һλ J1939 0x18FEFC17 18FEFC21
			{
				//�ж��Ƿ�
				if(gTerminalState.tankLevel <= 1 || gTerminalState.tankLevel == msg->id || 
					(sourseAddr == 17 && gTerminalState.tankLevel != 0x18A70017 && gTerminalState.tankLevel != 0x18FA0317))
				{
					gRealData.tankLevel = calcRealValue(8,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(gRealData.tankLevel > 0 && gRealData.tankLevel <= 100)
					{
						gTerminalState.tankLevel = msg->id;
					}
				}
			}
			break;
		case 0xFA03://����Һλ ����
			{
				if(gTerminalState.tankLevel <= 1 || 0x18FA0317 == msg->id)
				{
					gRealData.tankLevel = calcRealValue(0,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(gRealData.tankLevel > 0 && gRealData.tankLevel <= 100)
					{
							gTerminalState.tankLevel = msg->id;
					}
				}
			}
			break;
		case 0xA700://����Һλ ����
			{
				if(gTerminalState.tankLevel <= 1 || 0x18A70017 == msg->id)
				{
					gRealData.tankLevel = calcRealValue(56,8,0.4,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(gRealData.tankLevel > 0 && gRealData.tankLevel <= 100)
					{
							gTerminalState.tankLevel = msg->id;
					}
				}
			}
			break;
		case 0xFEC1://0x18FEC1EE
			{
				float totalMileage;
				//�߾����ۼ����,����ʹ��0x18fee017��0x18fee0ee
				if(msg->id == 0x18FEC1EE && gTerminalState.totalMileage != 0x18fee017 && gTerminalState.totalMileage != 0x18fee0ee)
				{
					gTerminalState.totalMileage = msg->id;
				}
				if(gTerminalState.totalMileage <= 1 || gTerminalState.totalMileage == msg->id)
				{
					totalMileage = calcRealValue(0,32,0.005,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(totalMileage > 0 && totalMileage < 9999999.9f)
					{
						gRealData.totalMileage = totalMileage;
					}
					gTerminalState.totalMileage = msg->id;
				}
			}
			break;
		case 0xFEE0:
			{
				float totalMileage;
				if(msg->id == 0x18fee0ee)//����ʹ��
				{
					gTerminalState.totalMileage = msg->id;
				}
				if(msg->id == 0x18fee017 && gTerminalState.totalMileage != 0x18fee0ee)//�ڶ�ʹ��
				{
					gTerminalState.totalMileage = msg->id;
				}
				//�ۼ����
				if(gTerminalState.totalMileage <= 1 || gTerminalState.totalMileage == msg->id)
				{
					totalMileage = calcRealValue(32,32,0.125,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
					if(totalMileage > 0 && totalMileage < 9999999.9f)
					{
						gRealData.totalMileage = totalMileage;
						gTerminalState.totalMileage = msg->id;
					}
				}
			}
			break;
		case 0xF00A://0F00A00
			{
				//������
				gRealData.intakeFlowrate = calcRealValue(16,16,0.05,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				gTerminalState.intakeFlowrate = msg->id;
			}
			break;
			//���
		case 0xFFCC://0CFFCC00
			{
				//TWC ���������������ֵ 
				gRealData.twcUpperOxySensor = calcRealValue(0,16,1/16384,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
			}
			break;
			//
		case 0xFD46://0CFD4600
			{
				//TWC �����¶�
				gRealData.twcTemp = calcRealValue(16,16,0.03125,-273,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
			}
			break;
			//Ϋ��
		case 0xFFAE://1CFFAE00
			{
				//TWC ���������������ֵ ��ȼ��
				gRealData.twcUpperOxySensor = calcRealValue(0,16,0.0000305,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
			}
			break;
		case 0xFFC1://0x1CFFC100
			{
				//TWC�߻������������������ֵ(��ѹ)
				gRealData.twcUpperOxySensor = calcRealValue(8,8,0.01,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
				getOxygenFlag = 1;
			}
			break;
		case 0xFD22://0x18FD2200
			{
				//TWC �����¶�
				gRealData.twcTemp = calcRealValue(0,16,0.03125,-273,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
			}
			break;
		default:
			{
				
			}
			break;
	}
	if(osKernelGetTickCount() - speedTime >= 5000)
	{
		gRealData.speed = 0;
	}
	if(osKernelGetTickCount() - engSpeedTime >= 5000)
	{
		gRealData.engineSpeed = 0;
	}
	if(osKernelGetTickCount() - sendCanTimeStamp >= 10 && fun_can_Get_State(BSP_CAN1) > 0)
	{
		uint8_t isSend = 0;
		CAN_msg msg_buf = {0x18FA122B,{ 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,EXTENDED_TYPE,DATA_FRAME};
		sendCanTimeStamp = osKernelGetTickCount();
		if(osKernelGetTickCount() - sendStaSecond >= 5000)
		{
			uint32_t devId = 0;
			uint8_t vinSta = 0;
			sendStaSecond = osKernelGetTickCount();	
			//CAN����
			sscanf(&gFrimPara.terminalId[6],"%u",&devId);
			devId = 0xEF5F4C00 + devId;
			msg_buf.data[0] = 0x22;
			msg_buf.data[1] = devId >> 24;
			msg_buf.data[2] = devId >> 16;
			msg_buf.data[3] = devId >> 8;
			msg_buf.data[4] = devId >> 0;
			if(strlen(gRealData.vin) == 17 && CheckVin(gRealData.vin) == 1)
			{
				vinSta = 1;
			}
			msg_buf.data[5] = vinSta << 7;
			msg_buf.data[5] |= fun_can_Get_State(BSP_CAN1) << 6;
			msg_buf.data[5] |= gb17691GetSta(0) << 5;
			msg_buf.data[5] |= (!gRealData.locationState) << 4;
			msg_buf.data[5] |= (Fun_Gprs_GetSta() >= FUN_GPRS_GET_SIM) << 3;
			msg_buf.data[5] |= (1 & 0x07);
			
			msg_buf.data[6] = (gTerminalState.reagentSurplus > 0) << 7;
			msg_buf.data[6] |= (gTerminalState.intakeFlowrate > 0) << 6;
			msg_buf.data[6] |= (gTerminalState.engineFuelFlow > 0) << 5;
			msg_buf.data[6] |= (gTerminalState.engineSpeed > 0) << 4;
			msg_buf.data[6] |= (gTerminalState.frictionTorque > 0)<< 3;
			msg_buf.data[6] |= (gTerminalState.engineTorque > 0) << 2;
			msg_buf.data[6] |= (gTerminalState.barometric > 0) << 1;
			msg_buf.data[6] |= (gTerminalState.speed > 0) << 0;
			msg_buf.data[7] = (gTerminalState.engineCoolantTemp > 0) << 0;
			isSend = 1;
		}
		else if(osKernelGetTickCount() - sendDateTimeStamp >= 1000)
		{
			/*//CAN��־
			if(gUploadCan == 1)
			{
				gUploadCanSize = 0;
				canLogSecCnt = 0;
				canlogBufLen = 0;
				memset(printfBuf,0,sizeof(printfBuf));
				canlogAddr = ADDR_FLASH_SECTOR_4;
				gUploadCan = 2;
			}
			canLogSecCnt++;
			*/
			//��������Ҫ��
			sendDateTimeStamp = osKernelGetTickCount();
			msg_buf.id = 0x18fee6fa;
			msg_buf.data[0] = g_system_dt.second * 4;
			msg_buf.data[1] = g_system_dt.minute;
			msg_buf.data[2] = g_system_dt.hour;
			msg_buf.data[3] = g_system_dt.month;
			msg_buf.data[4] = g_system_dt.day * 4;
			msg_buf.data[5] = g_system_dt.year - 1985;
			msg_buf.data[6] = 125;
			msg_buf.data[7] = 125 + 8;
			isSend = 1;
		}
		//SUP_DATA_BIT 6  //������������ʽ��ռ��λ 0-���ϱ� 1-�������� 2�����ð油�������� 3����
		else if(((gSysPara.linkSwitch >> 6) & 0x03) > 0 && gRealData.engineSpeed >= 100 && gRealData.engineSpeed < 0xFFFE && osKernelGetTickCount() - sendAuxStamp >= 1000)
		{
			sendAuxStamp = osKernelGetTickCount();
			msg_buf.id = 0x18eafff9;
			if(osKernelGetTickCount() - actAreaInjectValStamp >= 6000)
			{
				actAreaInjectValStamp = osKernelGetTickCount();
				msg_buf.data[0] = (uint8_t)(0x00F023 >> 0);
				msg_buf.data[1] = (uint8_t)(0x00F023 >> 8);
				msg_buf.data[2] = (uint8_t)(0x00F023 >> 16);
				isSend = 1;
			}
			else if(osKernelGetTickCount() - totalUreaUsedStamp1 >= 6000 && totalUreaUsedStamp2 != 0xFFFFFFFF)
			{
				//ȡ����һ��
				totalUreaUsedStamp1 = osKernelGetTickCount();
				msg_buf.data[0] = (uint8_t)(0x00F024 >> 0);
				msg_buf.data[1] = (uint8_t)(0x00F024 >> 8);
				msg_buf.data[2] = (uint8_t)(0x00F024 >> 16);
				isSend = 1;
			}
			else if(osKernelGetTickCount() - totalUreaUsedStamp2 >= 6000 && totalUreaUsedStamp1 != 0xFFFFFFFF)
			{
				//ȡ����һ��
				totalUreaUsedStamp2 = osKernelGetTickCount();
				msg_buf.data[0] = (uint8_t)(0x00F024 >> 0);
				msg_buf.data[1] = (uint8_t)(0x00F024 >> 8);
				msg_buf.data[2] = (uint8_t)(0x00F024 >> 16);
				isSend = 1;
			}
			else if(osKernelGetTickCount() - engReferenceTorqueStamp >= 6000)
			{
				engReferenceTorqueStamp = osKernelGetTickCount();
				msg_buf.data[0] = (uint8_t)(0x00FEE9 >> 0);
				msg_buf.data[1] = (uint8_t)(0x00FEE9 >> 8);
				msg_buf.data[2] = (uint8_t)(0x00FEE9 >> 16);
				isSend = 1;
			}
			else if(osKernelGetTickCount() - engTotalHoursOfOperationStamp >= 6000)
			{
				engTotalHoursOfOperationStamp = osKernelGetTickCount();
				msg_buf.data[0] = (uint8_t)(0x00FEE5 >> 0);
				msg_buf.data[1] = (uint8_t)(0x00FEE5 >> 8);
				msg_buf.data[2] = (uint8_t)(0x00FEE5 >> 16);
				isSend = 1;
			}
		}
		//ISO 27145 ����
		else if(osKernelGetTickCount() - sendOxygenStamp >= 2000 && gRealData.obdDiagProt == 1 && afterOtherCnt == 0 && gTerminalState.obdState == 1 && gRealData.technology >= 1 && getOxygenFlag == 0)
		{
			sendOxygenStamp = osKernelGetTickCount();
			msg_buf.id = sendUdsId;
			if(sendUdsId <= 0x7FF)
			{
				msg_buf.format = STANDARD_TYPE;
			}
			memset(msg_buf.data,0,8);
			msg_buf.data[0] = 0x03;
			msg_buf.data[1] = 0x22;
			msg_buf.data[2] = 0xF4;
			msg_buf.data[3] = 0x15;
			isSend = 1;
		}
		if(isSend == 1)
			CAN_send (1, &msg_buf,100);
	}
	/*	//��ȡCAN��־
	if(gUploadCan == 2)
	{
		if(canlogAddr < ADDR_FLASH_SECTOR_4 + 65536)
		{
			if(obdLog == 0  || (msg->id & 0xFFF00000) == 0x18d00000 || (msg->id & 0xFFFFFFF0) == 0x000007e0 || (msg->id & 0xF0F800FF) == 0x10e80000)
			{
				sprintf((char*)&printfBuf[canlogBufLen],"%02d%02d\t%08X\t%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx\r\n",g_system_dt.minute,g_system_dt.second,msg->id,msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
				canlogBufLen += 32;
			}
			if(canlogBufLen >= 2048 || (((obdLog == 0 && canLogSecCnt > 3) || canLogSecCnt > 10) && canlogBufLen > 0))//64֡���ݴ�һ��
			{
				Flash_Write(canlogAddr,printfBuf,canlogBufLen);
				memset(printfBuf,0,sizeof(printfBuf));
				canlogAddr += canlogBufLen;
				gUploadCanSize += canlogBufLen;
				canlogBufLen = 0;
			}
		}
		else
		{
			gUploadCan = 3;
		}
		if((obdLog == 0 && canLogSecCnt > 3) || canLogSecCnt > 10)
		{
			obdLog = 0;
			gUploadCan = 3;
		}
	}
	*/
}

static uint8_t recvJ1939Len = 0;				//J1939���ݽ��ճ���
static uint8_t recvUdsLen = 0;					//UDS���ݽ��ճ���

static uint8_t udsGetIdx = 0;						//���ݽ���λ��
static uint8_t j1939GetIdx = 0;					//���ݽ���λ��

static uint8_t j1939recvDataFLag = 0;		//���ݽ��ձ�־
static uint8_t udsrecvDataFLag = 0;			//���ݽ��ձ�־

static uint8_t recvUdsData[100];				//UDS���ݽ��ջ�����
static uint8_t recvJ1939Data[100];			//1939���ݽ��ջ�����

static uint8_t sendCmdSec = 0;					//���������ʱ���ʱ
static uint8_t sendCmdIdx = 0;					//��������������
static uint32_t startSecCnt = 0;				//����������ʱ���ʱ
static uint8_t sendCmdTimeout = 0;			//�������ʱ��ʱ

static uint8_t getJ1939Flag[8] = {0};		//�ѻ�ȡOBD������ţ������ظ���ȡ
static uint8_t get27145Flag[8] = {0};		//�ѻ�ȡOBD������ţ������ظ���ȡ
static uint8_t get15765Flag[8] = {0};		//�ѻ�ȡOBD������ţ������ظ���ȡ

static uint8_t uds15765Cmd[5][8]={{0x02,0x09,0x02,0x00,0x00,0x00,0x00,0x00},	//VIN
																	{0x02,0x09,0x04,0x00,0x00,0x00,0x00,0x00},	//�궨ʶ����
																	{0x02,0x09,0x06,0x00,0x00,0x00,0x00,0x00},	//�궨��֤��
																	{0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00},	//���֧�־���״̬
																	{0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00},};//�����루�ŷ���أ�

static uint8_t uds27145Cmd[8][8]={{0x03,0x22,0xf8,0x02,0x00,0x00,0x00,0x00},	//VIN
																	{0x03,0x22,0xf8,0x04,0x00,0x00,0x00,0x00},	//��׼��׼��
																	{0x03,0x22,0xf8,0x06,0x00,0x00,0x00,0x00},	//�궨��֤��
																	{0x03,0x22,0xf4,0x01,0x00,0x00,0x00,0x00},	//���֧��״̬����Ͼ���״̬
																	{0x05,0x19,0x42,0x33,0x0C,0x1E,0x00,0x00},	//��ȡ���ϴ��루�ŷ���أ�05 19 42 33 0C 1E FF FF ע�ͣ�19�������� 42��������˷�ʽ��ȡ 33���ŷŹ��� 0C��pendingDTC confirmedDTC 1E�����еȼ�
																	{0x03,0x22,0xf4,0x91,0x00,0x00,0x00,0x00},	//��ȡMIL״̬����� ��ʱ������
																	{0x03,0x22,0xf8,0x0B,0x00,0x00,0x00,0x00},	//���ͻ�IUPRֵMODE0:0x03,0x22,0xf8,0x0B
																	{0x03,0x22,0xf8,0x08,0x00,0x00,0x00,0x00},};//�����IUPRֵMODE0:0x03,0x22,0xf8,0x08


static uint32_t j1939Cmd[5] =			{0xfeec,0xd300,0xfece,0xfeca,0xc200};				//VIN,�궨ʶ����,���֧�־���״̬,������,IUPR
/*
static void udsAnalyse(CAN_msg *msg)
{
	uint32_t pgn = (msg->id & 0x00FFFF00) >> 8;
	if((pgn & 0xF000) != 0xF000)
	{
		pgn &= 0xFF00;
	}
	//��̽Э��,���ڴ˴����ų����ţ�ͬʱ֧�ֶ��Э��ʱȷ��Э��
	if(gRealData.obdDiagProt == 0xFF && sendCmdTimeout > 0)
	{
		if(msg->id == 0x18daf100 || msg->id == 0x18daf110 || msg->id == 0x7E8)
		{
			if((msg->data[2] == 0x62 && msg->data[3]== 0xF8 && msg->data[4]== 0x02) && getObdDiagProtIdx == 0)
			{
				gRealData.obdDiagProt = 1;//ȷ����ISO27145Э��
			}
			else if(msg->data[2] == 0x49 && msg->data[3]== 0x02 && msg->data[4]== 0x01 && getObdDiagProtIdx == 1)
			{
				gRealData.obdDiagProt = 0;//ȷ����ISO15765Э��
			}
		}
		else if(msg->id == 0x1cecff00 || msg->id == 0x18ecff00)
		{
			uint32_t png = (msg->data[5] | (msg->data[6] << 8) | (msg->data[7] << 16));
			if(msg->data[0] == 0x20 && png == 0xfeec && getObdDiagProtIdx == 2)
			{
				gRealData.obdDiagProt = 2;//ȷ����SAE J1939Э��,SAE J1939������
			}
		}
	}
	if(msg->id == 0x18da00f1 || msg->id == 0x18da10f1 || msg->id == 0x7E0 || msg->id == 0x18db33f1 || msg->id == 0x7df)
	{
		//����ϵͳ���͵��������
		afterOtherCnt = 0;
	}
	if(msg->id == 0x18daf100 || msg->id == 0x18daf110 || msg->id == 0x7E8)
	{
		if(msg->id == 0x18daf100)
		{
			sendUdsId = 0x18da00f1;
		}
		else if(msg->id == 0x7E8)
		{
			sendUdsId = 0x7E0;
		}
		else if(msg->id == 0x18daf110 && sendCmdTimeout > 0 && sendUdsId == 0)//�ն˷�������
		{
			sendUdsId = 0x18da10f1;
		}
		//���յ�֡����
		if((msg->data[0] & 0xF0) == 0x00)
		{
			memset(recvUdsData,0,sizeof(recvUdsData));
			recvUdsLen = msg->data[0] & 0x0F;				//��ȡ�������ݳ���
			memcpy(&recvUdsData[0],&msg->data[1],7);				//����������
			udsrecvDataFLag = 0;																//�����������
		}
		//���ն�֡������֡
		else if((msg->data[0] & 0xF0) == 0x10)
		{
			CAN_msg msg_buf = {0x18da00f1,{0x30, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,EXTENDED_TYPE,DATA_FRAME};
			memset(recvUdsData,0,sizeof(recvUdsData));
			//��׼֡
			if(msg->id == 0x18daf100)
			{
				msg_buf.format = EXTENDED_TYPE;
				msg_buf.id = 0x18da00f1;
			}
			else if(msg->id == 0x18daf110)
			{
				msg_buf.format = EXTENDED_TYPE;
				msg_buf.id = 0x18da10f1;
			}
			if(msg->id == 0x7E8)
			{
				msg_buf.id = 0x7E0;
				msg_buf.format = STANDARD_TYPE;
			}
			udsGetIdx = 0x21;
			recvUdsLen = ((msg->data[0] & 0x0F) << 8) | msg->data[1];
			memcpy(&recvUdsData[0],&msg->data[2],6);
			if(sendCmdTimeout > 0)//�ն˷������󣬷�������֡
			{
				osDelay(20);
				CAN_send (1, &msg_buf,0x0F00);
			}
			udsrecvDataFLag = 1;	//���ն˷������󣬱���������ݱ�־
		}
		//��������֡,����˳��
		else if((msg->data[0] & 0xF0) == 0x20 && msg->data[0] == udsGetIdx && udsrecvDataFLag == 1)
		{
			memcpy(&recvUdsData[(udsGetIdx - 0x21) * 7 + 6],&msg->data[1],7);
			//�жϽ����������
			if(((udsGetIdx - 0x21) * 7 + 13) >= recvUdsLen)
			{
				udsrecvDataFLag = 0;										
			}
			udsGetIdx++;
		}
		else
		{
			udsGetIdx = 0;
			udsrecvDataFLag = 0;
			return;//���մ��󣬷���
		}
		if(udsrecvDataFLag == 0)//�����������
		{
			uint8_t codeLen;
			uint8_t head[3];;
			head[0] = recvUdsData[0]- 0x40;head[1] = recvUdsData[1];head[2] = recvUdsData[2];
			if(sendCmdTimeout > 0 && ((gRealData.obdDiagProt == 1 && memcmp(head,&uds27145Cmd[sendCmdIdx][1],3) == 0) || (gRealData.obdDiagProt == 0 && memcmp(head,&uds15765Cmd[sendCmdIdx][1],strlen((char*)&uds15765Cmd[sendCmdIdx][1]) - 1) == 0)))
			{
				sendCmdIdx++;
				sendCmdTimeout = 0;
			}
			//VIN
			if((recvUdsData[0] == 0x49 && recvUdsData[1]== 0x02 && recvUdsData[2]== 0x01) || (recvUdsData[0] == 0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x02))
			{
				if(recvUdsData[0] == 0x49)
					gRealData.obdDiagProt = 0;
				else if(recvUdsData[0] == 0x62)
					gRealData.obdDiagProt = 1;
				memcpy(gRealData.vin,&recvUdsData[3],17);
				gRealData.vin[17] = 0;
				//�ն�δ����������Ը����ն�
				if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gFrimPara.setUpFlag != 0xAA && CheckVin(gRealData.vin) == 1)
				{
					//��ȡ�����ܺţ���������
					//gprsRedial();
					Fun_Gprs_Tcp_disconnect(3);
					Fun_Gprs_Tcp_disconnect(4);
					strcpy(gSysPara.vinCode,gRealData.vin);
					System_Pare_Save();
				}
				get15765Flag[0] = 1;
				get27145Flag[0] = 1;
			}
			//У׼��ʶ��
			else if((recvUdsData[0]==0x49 && recvUdsData[1]== 0x04) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x04))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 18)
					codeLen = 18;
				memcpy(gRealData.softCalId,&recvUdsData[3],codeLen);
				get15765Flag[1] = 1;
				get27145Flag[1] = 1;
			}
			//�궨��֤��
			else if((recvUdsData[0]==0x49 && recvUdsData[1]== 0x06) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x06))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 18)
					codeLen = 18;
				memcpy(gRealData.cvn,&recvUdsData[3],codeLen);
				get15765Flag[2] = 1;
				get27145Flag[2] = 1;
			}
			//���֧��״̬����Ͼ���״̬ ISO27145 �� ISO15765
			else if((recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x01) || (recvUdsData[0] == 0x41 && recvUdsData[1]== 0x01))
			{
				uint8_t A,B,C,D;
				//MIL״̬
				gRealData.milState =  calcCanValue(7,1,&A,HIGHTLOW_MODE,CAN_MODE);
				if(recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x01)
				{
					A = recvUdsData[3];B = recvUdsData[4];C = recvUdsData[5];D = recvUdsData[6];
				}
				else
				{
					A = recvUdsData[2];B = recvUdsData[3];C = recvUdsData[4];D = recvUdsData[5];					
				}
				//ȼ��
				if(0)
				{
					//�߻�ת����
					gRealData.diagSpState.catalyst = calcCanValue(0,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//���ȴ߻�ת����
					gRealData.diagSpState.heatedCatalyst = calcCanValue(1,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//����ϵͳ
					gRealData.diagSpState.evaporativeSys = calcCanValue(2,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//���ο���ϵͳ
					gRealData.diagSpState.secondaryAirSys = calcCanValue(3,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//����������������
					gRealData.diagSpState.exhaustGasSensorHeater = calcCanValue(6,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//�߻�ת����
					gRealData.diagRdyState.catalyst = calcCanValue(0,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//���ȴ߻�ת����
					gRealData.diagRdyState.heatedCatalyst = calcCanValue(1,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//����ϵͳ
					gRealData.diagRdyState.evaporativeSys = calcCanValue(2,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//���ο���ϵͳ
					gRealData.diagRdyState.secondaryAirSys = calcCanValue(3,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//����������������
					gRealData.diagRdyState.exhaustGasSensorHeater = calcCanValue(6,1,&D,HIGHTLOW_MODE,CAN_MODE);
				}
				
				//����
				else
				{
					//��ѹѹ������ϵͳ
					gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(3,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//DPF���
					gRealData.diagSpState.dpf = calcCanValue(6,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//ѡ���Դ߻���ԭϵͳ��SCR����NOx������
					gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(1,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//NMHC�����߻���
					gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(0,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//��ѹѹ������ϵͳ
					gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(3,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//DPF���
					gRealData.diagSpState.dpf = calcCanValue(6,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//ѡ���Դ߻���ԭϵͳ��SCR����NOx������
					gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(1,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//NMHC�����߻���
					gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(0,1,&D,HIGHTLOW_MODE,CAN_MODE);					
				}
				//����������
				gRealData.diagSpState.exhaustGasSensor = calcCanValue(5,1,&C,HIGHTLOW_MODE,CAN_MODE);
				//EGR��Vvtϵͳ
				gRealData.diagSpState.egrAndVvtSys = calcCanValue(7,1,&C,HIGHTLOW_MODE,CAN_MODE);
				//ʧ��
				gRealData.diagSpState.misfire = calcCanValue(0,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//ȼ��ϵͳ
				gRealData.diagSpState.fuelSys = calcCanValue(1,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//�ۺϳɷ�
				gRealData.diagSpState.comprehensiveComponent = calcCanValue(2,1,&B,HIGHTLOW_MODE,CAN_MODE);
				
				
				//����������
				gRealData.diagRdyState.exhaustGasSensor = calcCanValue(5,1,&D,HIGHTLOW_MODE,CAN_MODE);
				//EGR��Vvtϵͳ
				gRealData.diagRdyState.egrAndVvtSys = calcCanValue(7,1,&D,HIGHTLOW_MODE,CAN_MODE);				
				//ʧ��
				gRealData.diagRdyState.misfire = calcCanValue(4,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//ȼ��ϵͳ
				gRealData.diagRdyState.fuelSys = calcCanValue(5,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//�ۺϳɷ�
				gRealData.diagRdyState.comprehensiveComponent = calcCanValue(6,1,&B,HIGHTLOW_MODE,CAN_MODE);
				
				//ACϵͳ�����
				//gRealData.diagSpState.acSysRefrigerant = calcCanValue(4,1,&C,HIGHTLOW_MODE,CAN_MODE);//����
				//ACϵͳ�����
				//gRealData.diagRdyState.acSysRefrigerant = calcCanValue(4,1,&D,HIGHTLOW_MODE,CAN_MODE);//����
				//����������ϵͳ
				//gRealData.diagSpState.coldStartAidSys = 0;
				//����������ϵͳ
				//gRealData.diagRdyState.coldStartAidSys = 0;
				
				//get15765Flag[3] = 1;
				//get27145Flag[3] = 1;
			}
			//ISO15765������
			else if(recvUdsData[0]==0x43)
			{
				uint8_t i;
				uint8_t cnt = 0;
				for(i=0;i < recvUdsData[1];i++)
				{
					cnt++;
					//gRealData.faultCode[i] = (recvUdsData[i*2+2] << 8) | (recvUdsData[i*2+3] << 0);
					gRealData.faultCode[i] = (recvUdsData[i*2+2] << 8) | (recvUdsData[i*2+3] << 0);//��27145Э����������
				}
				gRealData.faultCodeCnt = cnt;
				//get15765Flag[4] = 1;
			}
			//ISO27145������
			else if(recvUdsData[0]==0x59 && recvUdsData[1]== 0x42 && recvUdsData[2]== 0x33)
			{
				uint8_t i;
				uint8_t cnt = 0;
				for(i=6;i < recvUdsLen;i += 5)
				{
					gRealData.faultCode[cnt++] = (recvUdsData[i+1] << 24) | (recvUdsData[i+2] << 16) | (recvUdsData[i+3] << 8) | (recvUdsData[i+4] << 0);
				}
				gRealData.faultCodeCnt = cnt;
				//get27145Flag[4] = 1;
			}
			//ISO 27145MIL��״̬
			else if(recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x91)
			{
				uint8_t A;
				A = recvUdsData[3];
				//MIL״̬
				gRealData.milState =  calcCanValue(0,4,&A,HIGHTLOW_MODE,CAN_MODE);
				//get27145Flag[5] = 1;
			}
			//IUPRֵ
			else if((recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x0B) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x08))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 36)
					codeLen = 36;
				memcpy(gRealData.iuprVal,&recvUdsData[3],codeLen);
				catype = 6;
				//get27145Flag[6] = 1;
				if(sendCmdIdx == 7)
				{
					//��ǰδ���ͻ������������
					sendCmdIdx++;
				}
			}
		}
		return;//UDS������ɣ���ǰ�뿪
	}
	else if(msg->id == 0x1cecff00 || msg->id == 0x18ecff00)
	{
		//J1939��� byte0:ControlByte byte1~byte2:TotalMessageSizeBAM byte3:TotalNumberOfPacketsBAM byte5~7:PGNumber
		if(msg->data[0] == 0x20)//ControlByte 0x20:BAM
		{
			memset(recvJ1939Data,0,sizeof(recvJ1939Data));
			memcpy(recvJ1939Data,msg->data,8);
			j1939GetIdx = 8;
			recvJ1939Len = msg->data[1] | (msg->data[2] << 8);
			j1939recvDataFLag = 1;
		}
		return;//�յ�Ԥ��֡����ǰ�뿪
	}
	else if(msg->id == 0x1cebff00 || msg->id == 0x18ebff00)
	{
		//UDSЭ�鲻ͨ����J1939Э��
		if((j1939GetIdx  - 8) != (msg->data[0] - 1) * 7 || j1939recvDataFLag == 0)
		{
			memset(recvJ1939Data,0,sizeof(recvJ1939Data));
			j1939GetIdx = 0;
			j1939recvDataFLag = 0;
			return;//����˳�������ǰ�뿪
		}
		if((j1939GetIdx  - 8) == (msg->data[0] - 1) * 7 && j1939recvDataFLag == 1)//��˳�����
		{
			memcpy(&recvJ1939Data[j1939GetIdx],&msg->data[1],7);
			j1939GetIdx += 7;
			if(msg->data[0] == recvJ1939Data[3] && recvJ1939Data[0] == 0x20 && recvJ1939Len > 0)	//������ɣ�����
			{
				uint8_t* pJ1939Data = &recvJ1939Data[8];
				pgn = (recvJ1939Data[5] | (recvJ1939Data[6] << 8) | (recvJ1939Data[7] << 16));
				j1939recvDataFLag = 0;
				if(gRealData.obdDiagProt == 2)
				{
					uint8_t dataLen;
					if(pgn == 0xfeec)//���ܺ�
					{
						memcpy(gRealData.vin,pJ1939Data,17);
						gRealData.vin[17] = 0;
						//if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gSysPara.vinCode[0] != 'L')
						//�ն�δ����������Ը����ն�
						if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gFrimPara.setUpFlag != 0xAA && CheckVin(gRealData.vin) == 1)
						{
							//��ȡ�����ܺ�,��������
							//gprsRedial();
							Fun_Gprs_Tcp_disconnect(3);
							Fun_Gprs_Tcp_disconnect(4);
							strcpy(gSysPara.vinCode,gRealData.vin);
							System_Pare_Save();
						}
						getJ1939Flag[0] = 1;
					}
					else if(pgn == 0xd300)//�궨ʶ��ţ��궨��֤��
					{
						memcpy(gRealData.cvn,pJ1939Data,4);
						memcpy(gRealData.softCalId,&pJ1939Data[4],16);
						getJ1939Flag[1] = 1;
					}
					else if(pgn == 0xfeca)//������
					{
						uint8_t jj;
						//MIL״̬
						gRealData.milState = calcRealValue(6,2,1,0,CALC_MODE,pJ1939Data,HIGHTLOW_MODE,CAN_MODE);
						gRealData.faultCodeCnt = ((recvJ1939Len - 2)/4) % 20;//���ֻ��20��������
						for(jj = 0;jj < gRealData.faultCodeCnt;jj++)
						{
							gRealData.faultCode[0] = (pJ1939Data[8 + jj*4] << 0) |  (pJ1939Data[3+jj*4] << 8) | (pJ1939Data[4 + jj*4] << 16) | (pJ1939Data[5 +jj*4] << 24 );
						}
						getJ1939Flag[3] = 1;
					}
					else if(pgn == 0xc200)//IUPR
					{
						if( recvJ1939Data[1] > 36)
							dataLen = 36;
						memcpy(gRealData.iuprVal,pJ1939Data,dataLen);
						if(gRealData.obdDiagProt == 2)
						{
							catype = 6;
						}
						getJ1939Flag[4] = 1;
					}
				}
				if(pgn == 0xFEE3)//EC1
				{
					//���������ο�Ť�� 1
					gRealData.engReferenceTorque = pJ1939Data[19] | (pJ1939Data[20] << 8);
				}
			}
		}
	}
	else if(pgn == 0x00FECA && gRealData.obdDiagProt == 2)//MIL״̬�͹�����
	{
		//getJ1939Flag[3] = 1;
		//MIL״̬
		gRealData.milState = calcRealValue(6,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		if(msg->data[2] != 0 || msg->data[3] != 0  || msg->data[4] != 0 || msg->data[5] != 0)
		{
			gRealData.faultCodeCnt = 1;
			gRealData.faultCode[0] = (msg->data[2] << 0) |  (msg->data[3] << 8) | (msg->data[4] << 16) | (msg->data[5] << 24 );
		}
	}
	else if(pgn == 0x00FECE && gRealData.obdDiagProt == 2)//����״̬
	{
		//getJ1939Flag[2] = 1;
		// ���֧��״̬ 
		//�߻�ת����
		gRealData.diagSpState.catalyst = calcCanValue(32,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//���ȴ߻�ת����
		gRealData.diagSpState.heatedCatalyst = calcCanValue(33,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//����ϵͳ
		gRealData.diagSpState.evaporativeSys = calcCanValue(34,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//���ο���ϵͳ
		gRealData.diagSpState.secondaryAirSys = calcCanValue(35,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//ACϵͳ�����
		gRealData.diagSpState.acSysRefrigerant = calcCanValue(36,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//����������
		gRealData.diagSpState.exhaustGasSensor = calcCanValue(37,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//����������������
		gRealData.diagSpState.exhaustGasSensorHeater = calcCanValue(38,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//EGR��Vvtϵͳ
		gRealData.diagSpState.egrAndVvtSys = calcCanValue(39,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//����������ϵͳ
		gRealData.diagSpState.coldStartAidSys = calcCanValue(40,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		
		//��Ͼ���״̬
		//�߻�ת����
		gRealData.diagRdyState.catalyst = calcCanValue(48,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//���ȴ߻�ת����
		gRealData.diagRdyState.heatedCatalyst = calcCanValue(49,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//����ϵͳ
		gRealData.diagRdyState.evaporativeSys = calcCanValue(50,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//���ο���ϵͳ
		gRealData.diagRdyState.secondaryAirSys = calcCanValue(51,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//ACϵͳ�����
		gRealData.diagRdyState.acSysRefrigerant = calcCanValue(52,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//����������
		gRealData.diagRdyState.exhaustGasSensor = calcCanValue(53,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//����������������
		gRealData.diagRdyState.exhaustGasSensorHeater = calcCanValue(54,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//EGR��Vvtϵͳ
		gRealData.diagRdyState.egrAndVvtSys = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//����������ϵͳ
		gRealData.diagRdyState.coldStartAidSys = calcCanValue(56,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		
		
		
		//��ѹѹ������ϵͳ
		gRealData.diagRdyState.boostPressureCtrlSys = calcCanValue(57,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//DPF���
		gRealData.diagRdyState.dpf = calcCanValue(58,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//ѡ���Դ߻���ԭϵͳ��SCR����NOx������
		gRealData.diagRdyState.scrOrNOxAdsorber = calcCanValue(59,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//NMHC�����߻���
		gRealData.diagRdyState.nmhcConvertingCatalyst = calcCanValue(60,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		
		
		//��ѹѹ������ϵͳ
		gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(41,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//DPF���
		gRealData.diagSpState.dpf = calcCanValue(42,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//ѡ���Դ߻���ԭϵͳ��SCR����NOx������
		gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(43,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//NMHC�����߻���
		gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(44,1,msg->data,HIGHTLOW_MODE,CAN_MODE);


		//ʧ��
		gRealData.diagSpState.misfire = calcCanValue(24,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//ȼ��ϵͳ
		gRealData.diagSpState.fuelSys = calcCanValue(25,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//�ۺϳɷ�
		gRealData.diagSpState.comprehensiveComponent = calcCanValue(26,1,msg->data,HIGHTLOW_MODE,CAN_MODE);

		//ʧ��
		gRealData.diagRdyState.misfire = calcCanValue(28,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//ȼ��ϵͳ
		gRealData.diagRdyState.fuelSys = calcCanValue(29,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
		//�ۺϳɷ�
		gRealData.diagRdyState.comprehensiveComponent = calcCanValue(30,1,msg->data,HIGHTLOW_MODE,CAN_MODE);			
	}
	if(sendCmdTimeout > 0 && gRealData.obdDiagProt == 2 && j1939Cmd[sendCmdIdx] == pgn)
	{
		sendCmdIdx++;
		sendCmdTimeout = 0;
	}
}
											
static void Udstask(void)
{
	//�����������������������
	if(g_system_dt.second != sendCmdSec)
	{
		sendCmdSec = g_system_dt.second;
		//Ӧ��ʱ
		if(sendCmdTimeout > 0)
		{
			sendCmdTimeout--;								//����ʱ����
			if(sendCmdTimeout == 1)					//3�볬ʱ
			{
				//J1939��֧��IUPR����ʱ����
				if(gRealData.obdDiagProt == 2 && sendCmdIdx == 4)
				{
					sendCmdIdx++;
				}
				//ISO27145��֧�ִ��ֶ�MIL״̬ ��IUPR״̬����ʱ����
				else if(gRealData.obdDiagProt == 1 && (sendCmdIdx == 5 || sendCmdIdx == 6 || sendCmdIdx == 7))
				{
					sendCmdIdx++;
				}
				sendCmdTimeout = 0;
			}
		}
		afterOtherCnt = (afterOtherCnt < 0xFF) ? (afterOtherCnt + 1) : afterOtherCnt;
		if(afterOtherCnt >= 4)
			startSecCnt = (startSecCnt < 0xFF) ? (startSecCnt + 1) : startSecCnt;//����������48���2��30����
		if(startSecCnt >= 45 && startSecCnt <= 150 && sendCmdTimeout == 0 && gRealData.engineSpeed >= 100 && gRealData.engineSpeed < 0xFFFE && afterOtherCnt >= 4)
		{
			//��̽Э��˳�����
			getObdDiagProtIdx++;
			getObdDiagProtIdx = getObdDiagProtIdx % 3;
			if((gRealData.obdDiagProt == 0 && sendCmdIdx < (sizeof(uds15765Cmd) / 8)) || (gRealData.obdDiagProt == 1 && sendCmdIdx < (sizeof(uds27145Cmd) / 8)) || (gRealData.obdDiagProt == 2 && sendCmdIdx < (sizeof(j1939Cmd) / 4)) || gRealData.obdDiagProt == 0xFF)
			{
				CAN_msg msg_buf = {0x18eafff9,{ 0x30, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,EXTENDED_TYPE,DATA_FRAME};
				osDelay(100);
				if(gRealData.obdDiagProt == 1 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 0))//ISO27145Э���δ֪
				{
					if(get15765Flag[sendCmdIdx] == 1)//����Ҫ�ظ���ȡ
					{
						sendCmdIdx++;
						if(sendCmdIdx >= sizeof(uds27145Cmd) / 8)
						{
							return;
						}
					}
					memcpy(msg_buf.data,uds27145Cmd[sendCmdIdx],8);
					sendCmdTimeout = 4;
				}
				if(gRealData.obdDiagProt == 0 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 1))//ISO15765Э���δ֪
				{
					if(get15765Flag[sendCmdIdx] == 1)//����Ҫ�ظ���ȡ
					{
						sendCmdIdx++;
						if(sendCmdIdx >= sizeof(uds15765Cmd) / 8)
						{
							return;
						}
					}
					memcpy(msg_buf.data,uds15765Cmd[sendCmdIdx],8);
					sendCmdTimeout = 4;
				}
				if(sendCmdTimeout > 0)
				{
					//��չ֡
					if(sendUdsId == 0 || sendUdsId == 0x18DA00F1 || sendUdsId == 0x18DA10F1)
					{
						msg_buf.format = EXTENDED_TYPE;	
						//����Ѱַ��0x18DB33F1������Ѱַ��0x18DA00F1 0��ECU��Ӧ��0x18DAF100
						if(gRealData.obdDiagProt == 0 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 1))
						{
							msg_buf.id = 0x18DB33F1;
							osDelay(10);					
							CAN_send (1, &msg_buf,0x0F00);							
						}
						else
						{
							if(sendUdsId == 0 || sendUdsId == 0x18DA00F1)
							{
								msg_buf.id = 0x18DA00F1;
								osDelay(10);					
								CAN_send (1, &msg_buf,0x0F00);
							}
							if(sendUdsId == 0 || sendUdsId == 0x18DA10F1)
							{
								msg_buf.id = 0x18DA10F1;
								osDelay(10);					
								CAN_send (1, &msg_buf,0x0F00);							
							}
						}
					}
					//��׼֡
					if(sendUdsId == 0 || sendUdsId == 0x7E0)
					{
						//����Ѱַ��0x7DF������Ѱַ��0x7E0 0��ECU��Ӧ��7E8
						msg_buf.id = 0x7E0;
						msg_buf.format = STANDARD_TYPE;
						osDelay(10);
						CAN_send (1, &msg_buf,0x0F00);
					}
				}
				if(gRealData.obdDiagProt == 2 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 2))//SAE J1939Э���δ֪
				{
					msg_buf.data[0] = j1939Cmd[sendCmdIdx] >> 0;
					msg_buf.data[1] = j1939Cmd[sendCmdIdx] >> 8;
					msg_buf.data[2] = j1939Cmd[sendCmdIdx] >> 16;
					msg_buf.id = 0x18eafff9;
					msg_buf.format = EXTENDED_TYPE;
					osDelay(10);
					CAN_send (1, &msg_buf,0x0F00);
					sendCmdTimeout = 4;
				}
			}
			else
			{
				//OBD������
				gTerminalState.obdState = 1;
				if(gSysPara.carType != catype)
				{
					gSysPara.carType = catype;
					System_Pare_Save();
				}
			}
		}
	}
	
	//������Ϩ��
	if(gRealData.engineSpeed <= 10 || gRealData.engineSpeed >= 0xFFFE)
	{
		scrInReady = 0;
		scrOutReady = 0;
		memset(getJ1939Flag,0,sizeof(getJ1939Flag));
		memset(get27145Flag,0,sizeof(get27145Flag));
		memset(get15765Flag,0,sizeof(get15765Flag));
		startSecCnt = 0;
		sendCmdIdx = 0;
		gTerminalState.obdState = 0;
	}
	gTerminalState.getCarData = gTerminalState.obdState;
}
*/
static void udsAnalyse(CAN_msg *msg)
{
	uint32_t pgn = (msg->id & 0x00FFFF00) >> 8;
	uint8_t source = msg->id & 0xFF;
	if((pgn & 0xF000) != 0xF000)
	{
		pgn &= 0xFF00;
	}
	//��̽Э��,���ڴ˴����ų����ţ�ͬʱ֧�ֶ��Э��ʱȷ��Э��
	if(gRealData.obdDiagProt == 0xFF && sendCmdTimeout > 0)
	{
		if(msg->id == 0x18daf100 || msg->id == 0x18daf110 || msg->id == 0x7E8)
		{
			if((msg->data[2] == 0x62 && msg->data[3]== 0xF8 && msg->data[4]== 0x02) && getObdDiagProtIdx == 0)
			{
				gRealData.obdDiagProt = 1;//ȷ����ISO27145Э��
			}
			else if(msg->data[2] == 0x49 && msg->data[3]== 0x02 && msg->data[4]== 0x01 && getObdDiagProtIdx == 1)
			{
				gRealData.obdDiagProt = 0;//ȷ����ISO15765Э��
			}
		}
		else if(msg->id == 0x1cecff00 || msg->id == 0x18ecff00)
		{
			uint32_t png = (msg->data[5] | (msg->data[6] << 8) | (msg->data[7] << 16));
			if(msg->data[0] == 0x20 && png == 0xfeec && getObdDiagProtIdx == 2)
			{
				gRealData.obdDiagProt = 2;//ȷ����SAE J1939Э��,SAE J1939������
			}
		}
	}
	if(msg->id == 0x18da00f1 || msg->id == 0x18da10f1 || msg->id == 0x7E0 || msg->id == 0x18db33f1 || msg->id == 0x7df)
	{
		//����ϵͳ���͵��������ն��Ȳ����ͣ�3�����ܷ���
		afterOtherCnt = 0;
	}
	if(msg->id == 0x18daf100 || msg->id == 0x18daf110 || msg->id == 0x7E8)
	{
		if(msg->id == 0x18daf100)
		{
			sendUdsId = 0x18da00f1;
		}
		else if(msg->id == 0x7E8)
		{
			sendUdsId = 0x7E0;
		}
		else if(msg->id == 0x18daf110 && sendCmdTimeout > 0 && sendUdsId == 0)//�ն˷�������
		{
			sendUdsId = 0x18da10f1;
		}
		//���յ�֡����
		if((msg->data[0] & 0xF0) == 0x00)
		{
			memset(recvUdsData,0,sizeof(recvUdsData));
			recvUdsLen = msg->data[0] & 0x0F;								//��ȡ�������ݳ���
			memcpy(&recvUdsData[0],&msg->data[1],7);				//����������
			udsrecvDataFLag = 0;														//�����������
		}
		//���ն�֡������֡
		else if((msg->data[0] & 0xF0) == 0x10)
		{
			CAN_msg msg_buf = {0x18da00f1,{0x30, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,EXTENDED_TYPE,DATA_FRAME};
			memset(recvUdsData,0,sizeof(recvUdsData));
			//��׼֡
			if(msg->id == 0x18daf100)
			{
				msg_buf.format = EXTENDED_TYPE;
				msg_buf.id = 0x18da00f1;
			}
			else if(msg->id == 0x18daf110)
			{
				msg_buf.format = EXTENDED_TYPE;
				msg_buf.id = 0x18da10f1;
			}
			if(msg->id == 0x7E8)
			{
				msg_buf.id = 0x7E0;
				msg_buf.format = STANDARD_TYPE;
			}
			//�ն˷������󣬷�������֡
			udsGetIdx = 0x21;
			recvUdsLen = ((msg->data[0] & 0x0F) << 8) | msg->data[1];
			memcpy(&recvUdsData[0],&msg->data[2],6);
			//ȷ�����ն˷���������
			if(sendCmdTimeout > 0)
			{
				osDelay(20);
				CAN_send (1, &msg_buf,0x0F00);
			}
			//UDS�������ݱ�־
			udsrecvDataFLag = 1;
		}
		//��������֡,����˳��
		else if((msg->data[0] & 0xF0) == 0x20 && msg->data[0] == udsGetIdx && udsrecvDataFLag == 1)
		{
			memcpy(&recvUdsData[(udsGetIdx - 0x21) * 7 + 6],&msg->data[1],7);
			//�жϽ����������
			if(((udsGetIdx - 0x21) * 7 + 13) >= recvUdsLen)
			{
				udsrecvDataFLag = 0;										
			}
			udsGetIdx++;
		}
		else
		{
			udsGetIdx = 0;
			udsrecvDataFLag = 0;
			return;//���մ��󣬷���
		}
		if(udsrecvDataFLag == 0)//�����������
		{
			uint8_t codeLen;
			uint8_t head[3];;
			head[0] = recvUdsData[0]- 0x40;head[1] = recvUdsData[1];head[2] = recvUdsData[2];
			if(sendCmdTimeout > 0 && ((gRealData.obdDiagProt == 1 && memcmp(head,&uds27145Cmd[sendCmdIdx][1],3) == 0) || (gRealData.obdDiagProt == 0 && memcmp(head,&uds15765Cmd[sendCmdIdx][1],strlen((char*)&uds15765Cmd[sendCmdIdx][1]) - 1) == 0)))
			{
				sendCmdIdx++;
				sendCmdTimeout = 0;
			}
			//VIN
			if((recvUdsData[0] == 0x49 && recvUdsData[1]== 0x02 && recvUdsData[2]== 0x01) || (recvUdsData[0] == 0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x02))
			{
				if(recvUdsData[0] == 0x49)
					gRealData.obdDiagProt = 0;
				else if(recvUdsData[0] == 0x62)
					gRealData.obdDiagProt = 1;
				memcpy(gRealData.vin,&recvUdsData[3],17);
				gRealData.vin[17] = 0;
				//�ն�δ����������Ը����ն�
				if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gFrimPara.setUpFlag != 0xAA && CheckVin(gRealData.vin) == 1)
				{
					//��ȡ�����ܺţ���������
					//gprsRedial();
					Fun_Gprs_Tcp_disconnect(3);
					Fun_Gprs_Tcp_disconnect(4);
					strcpy(gSysPara.vinCode,gRealData.vin);
					System_Pare_Save();
				}
				get15765Flag[0] = 1;
				get27145Flag[0] = 1;
			}
			//У׼��ʶ��
			else if((recvUdsData[0]==0x49 && recvUdsData[1]== 0x04) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x04))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 18)
					codeLen = 18;
				memcpy(gRealData.softCalId,&recvUdsData[3],codeLen);
				get15765Flag[1] = 1;
				get27145Flag[1] = 1;
			}
			//�궨��֤��
			else if((recvUdsData[0]==0x49 && recvUdsData[1]== 0x06) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x06))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 18)
					codeLen = 18;
				memcpy(gRealData.cvn,&recvUdsData[3],codeLen);
				get15765Flag[2] = 1;
				get27145Flag[2] = 1;
			}
			//���֧��״̬����Ͼ���״̬ ISO27145 �� ISO15765
			else if((recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x01) || (recvUdsData[0] == 0x41 && recvUdsData[1]== 0x01))
			{
				uint8_t A,B,C,D;
				//MIL״̬
				gRealData.milState =  calcCanValue(7,1,&A,HIGHTLOW_MODE,CAN_MODE);
				if(recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x01)
				{
					A = recvUdsData[3];B = recvUdsData[4];C = recvUdsData[5];D = recvUdsData[6];
				}
				else
				{
					A = recvUdsData[2];B = recvUdsData[3];C = recvUdsData[4];D = recvUdsData[5];					
				}
				//ȼ��
				if(0)
				{
					//�߻�ת����
					gRealData.diagSpState.catalyst = calcCanValue(0,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//���ȴ߻�ת����
					gRealData.diagSpState.heatedCatalyst = calcCanValue(1,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//����ϵͳ
					gRealData.diagSpState.evaporativeSys = calcCanValue(2,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//���ο���ϵͳ
					gRealData.diagSpState.secondaryAirSys = calcCanValue(3,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//����������������
					gRealData.diagSpState.exhaustGasSensorHeater = calcCanValue(6,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//�߻�ת����
					gRealData.diagRdyState.catalyst = calcCanValue(0,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//���ȴ߻�ת����
					gRealData.diagRdyState.heatedCatalyst = calcCanValue(1,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//����ϵͳ
					gRealData.diagRdyState.evaporativeSys = calcCanValue(2,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//���ο���ϵͳ
					gRealData.diagRdyState.secondaryAirSys = calcCanValue(3,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//����������������
					gRealData.diagRdyState.exhaustGasSensorHeater = calcCanValue(6,1,&D,HIGHTLOW_MODE,CAN_MODE);
				}
				
				//����
				else
				{
					//��ѹѹ������ϵͳ
					gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(3,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//DPF���
					gRealData.diagSpState.dpf = calcCanValue(6,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//ѡ���Դ߻���ԭϵͳ��SCR����NOx������
					gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(1,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//NMHC�����߻���
					gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(0,1,&C,HIGHTLOW_MODE,CAN_MODE);
					//��ѹѹ������ϵͳ
					gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(3,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//DPF���
					gRealData.diagSpState.dpf = calcCanValue(6,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//ѡ���Դ߻���ԭϵͳ��SCR����NOx������
					gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(1,1,&D,HIGHTLOW_MODE,CAN_MODE);
					//NMHC�����߻���
					gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(0,1,&D,HIGHTLOW_MODE,CAN_MODE);					
				}
				//����������
				gRealData.diagSpState.exhaustGasSensor = calcCanValue(5,1,&C,HIGHTLOW_MODE,CAN_MODE);
				//EGR��Vvtϵͳ
				gRealData.diagSpState.egrAndVvtSys = calcCanValue(7,1,&C,HIGHTLOW_MODE,CAN_MODE);
				//ʧ��
				gRealData.diagSpState.misfire = calcCanValue(0,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//ȼ��ϵͳ
				gRealData.diagSpState.fuelSys = calcCanValue(1,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//�ۺϳɷ�
				gRealData.diagSpState.comprehensiveComponent = calcCanValue(2,1,&B,HIGHTLOW_MODE,CAN_MODE);
				
				
				//����������
				gRealData.diagRdyState.exhaustGasSensor = calcCanValue(5,1,&D,HIGHTLOW_MODE,CAN_MODE);
				//EGR��Vvtϵͳ
				gRealData.diagRdyState.egrAndVvtSys = calcCanValue(7,1,&D,HIGHTLOW_MODE,CAN_MODE);				
				//ʧ��
				gRealData.diagRdyState.misfire = calcCanValue(4,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//ȼ��ϵͳ
				gRealData.diagRdyState.fuelSys = calcCanValue(5,1,&B,HIGHTLOW_MODE,CAN_MODE);
				//�ۺϳɷ�
				gRealData.diagRdyState.comprehensiveComponent = calcCanValue(6,1,&B,HIGHTLOW_MODE,CAN_MODE);
				
				//ACϵͳ�����
				//gRealData.diagSpState.acSysRefrigerant = calcCanValue(4,1,&C,HIGHTLOW_MODE,CAN_MODE);//����
				//ACϵͳ�����
				//gRealData.diagRdyState.acSysRefrigerant = calcCanValue(4,1,&D,HIGHTLOW_MODE,CAN_MODE);//����
				//����������ϵͳ
				//gRealData.diagSpState.coldStartAidSys = 0;
				//����������ϵͳ
				//gRealData.diagRdyState.coldStartAidSys = 0;
				
				//get15765Flag[3] = 1;
				//get27145Flag[3] = 1;
			}
			//ISO15765������
			else if(recvUdsData[0]==0x43)
			{
				uint8_t i;
				uint8_t cnt = 0;
				for(i=0;i < recvUdsData[1];i++)
				{
					cnt++;
					//gRealData.faultCode[i] = (recvUdsData[i*2+2] << 8) | (recvUdsData[i*2+3] << 0);
					gRealData.faultCode[i] = (recvUdsData[i*2+2] << 8) | (recvUdsData[i*2+3] << 0);//��27145Э����������
				}
				gRealData.faultCodeCnt = cnt;
				//��Ҫ����������
				//get15765Flag[4] = 1;
			}
			//ISO27145������
			else if(recvUdsData[0]==0x59 && recvUdsData[1]== 0x42 && recvUdsData[2]== 0x33)
			{
				uint8_t i;
				uint8_t cnt = 0;
				for(i=6;i < recvUdsLen;i += 5)
				{
					gRealData.faultCode[cnt++] = (recvUdsData[i+1] << 24) | (recvUdsData[i+2] << 16) | (recvUdsData[i+3] << 8) | (recvUdsData[i+4] << 0);
				}
				gRealData.faultCodeCnt = cnt;
				//��Ҫ����������
				//get27145Flag[4] = 1;
			}
			//ISO 27145MIL��״̬
			else if(recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x91)
			{
				uint8_t A;
				A = recvUdsData[3];
				//MIL״̬
				gRealData.milState =  calcCanValue(0,4,&A,HIGHTLOW_MODE,CAN_MODE);
				//��Ҫ����������
				//get27145Flag[5] = 1;
			}
			//IUPRֵ
			else if((recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x0B) || (recvUdsData[0]==0x62 && recvUdsData[1]== 0xF8 && recvUdsData[2]== 0x08))
			{
				codeLen  = recvUdsLen - 3;
				if(codeLen > 36)
					codeLen = 36;
				memcpy(gRealData.iuprVal,&recvUdsData[3],codeLen);
				emissionLevel = 1;
				if(sendCmdIdx == 7)
				{
					//��ǰδ���ͻ������������
					sendCmdIdx++;
				}
				//��Ҫ����������
				//get27145Flag[6] = 1;
			}
			//ISO 27145  Sensor 2 Oxygen sensor Output Voltage
			else if(recvUdsData[0]==0x62 && recvUdsData[1]== 0xF4 && recvUdsData[2]== 0x15)
			{
				uint8_t A;
				A = recvUdsData[3];
				//TWC�߻������������������ֵ(��ѹ)
				gRealData.twcUpperOxySensor = A * 0.005;
				if(gRealData.technology == 0)
				{
					gRealData.technology = 1;
				}
			}
		}
		return;//UDS������ɣ���ǰ�뿪
	}
	else if(msg->id == 0x1cecff00 || msg->id == 0x18ecff00)
	{
		//J1939��� byte0:ControlByte byte1~byte2:TotalMessageSizeBAM byte3:TotalNumberOfPacketsBAM byte5~7:PGNumber
		if(msg->data[0] == 0x20)//ControlByte 0x20:BAM
		{
			memset(recvJ1939Data,0,sizeof(recvJ1939Data));
			memcpy(recvJ1939Data,msg->data,8);
			j1939GetIdx = 8;
			recvJ1939Len = msg->data[1] | (msg->data[2] << 8);
			j1939recvDataFLag = 1;
		}
		return;//�յ�Ԥ��֡����ǰ�뿪
	}
	else if(msg->id == 0x1cebff00 || msg->id == 0x18ebff00)
	{
		//UDSЭ�鲻ͨ����J1939Э��
		if((j1939GetIdx  - 8) != (msg->data[0] - 1) * 7 || j1939recvDataFLag == 0)
		{
			memset(recvJ1939Data,0,sizeof(recvJ1939Data));
			j1939GetIdx = 0;
			j1939recvDataFLag = 0;
			return;//����˳�������ǰ�뿪
		}
		if((j1939GetIdx  - 8) == (msg->data[0] - 1) * 7 && j1939recvDataFLag == 1)//��˳�����
		{
			memcpy(&recvJ1939Data[j1939GetIdx],&msg->data[1],7);
			j1939GetIdx += 7;
			if(msg->data[0] == recvJ1939Data[3] && recvJ1939Data[0] == 0x20 && recvJ1939Len > 0)	//������ɣ�����
			{
				uint8_t* pJ1939Data = &recvJ1939Data[8];
				pgn = (recvJ1939Data[5] | (recvJ1939Data[6] << 8) | (recvJ1939Data[7] << 16));
				j1939recvDataFLag = 0;
				if(gRealData.obdDiagProt == 2)
				{
					uint8_t dataLen;
					if(pgn == 0xfeec)//���ܺ�
					{
						memcpy(gRealData.vin,pJ1939Data,17);
						gRealData.vin[17] = 0;
						//if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gSysPara.vinCode[0] != 'L')
						//�ն�δ����������Ը����ն�
						if(strcmp(gSysPara.vinCode,gRealData.vin) != 0 && gFrimPara.setUpFlag != 0xAA && CheckVin(gRealData.vin) == 1)
						{
							//��ȡ�����ܺ�,��������
							//gprsRedial();
							Fun_Gprs_Tcp_disconnect(3);
							Fun_Gprs_Tcp_disconnect(4);
							strcpy(gSysPara.vinCode,gRealData.vin);
							System_Pare_Save();
						}
						getJ1939Flag[0] = 1;
					}
					else if(pgn == 0xd300)//�궨ʶ��ţ��궨��֤��
					{
						memcpy(gRealData.cvn,pJ1939Data,4);
						memcpy(gRealData.softCalId,&pJ1939Data[4],16);
						getJ1939Flag[1] = 1;
					}
					else if(pgn == 0xfeca)//������
					{
						uint8_t jj;
						//MIL״̬
						gRealData.milState = calcRealValue(6,2,1,0,CALC_MODE,pJ1939Data,HIGHTLOW_MODE,CAN_MODE);
						gRealData.faultCodeCnt = ((recvJ1939Len - 2)/4) % 20;//���ֻ��20��������
						for(jj = 0;jj < gRealData.faultCodeCnt;jj++)
						{
							gRealData.faultCode[0] = (pJ1939Data[8 + jj*4] << 0) |  (pJ1939Data[3+jj*4] << 8) | (pJ1939Data[4 + jj*4] << 16) | (pJ1939Data[5 +jj*4] << 24 );
						}
						getJ1939Flag[3] = 1;
					}
					else if(pgn == 0xc200)//IUPR
					{
						if( recvJ1939Data[1] > 36)
							dataLen = 36;
						memcpy(gRealData.iuprVal,pJ1939Data,dataLen);
						if(gRealData.obdDiagProt == 2)
						{
							emissionLevel = 1;
						}
						getJ1939Flag[4] = 1;
					}
				}
				if(pgn == 0xFEE3)//EC1
				{
					//���������ο�Ť�� 1
					gRealData.engReferenceTorque = pJ1939Data[19] | (pJ1939Data[20] << 8);
					engReferenceTorqueStamp = 0xFFFFFFFF;		
				}
			}
		}
	}
	else if(source == 0)
	{
		if(pgn == 0x00FECA && gRealData.obdDiagProt == 2)//MIL״̬�͹�����
		{
			//getJ1939Flag[3] = 1;
			//MIL״̬
			//gRealData.milState = calcRealValue(6,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
			if(msg->data[2] != 0 || msg->data[3] != 0  || msg->data[4] != 0 || msg->data[5] != 0)
			{
				gRealData.faultCodeCnt = 1;
				gRealData.faultCode[0] = (msg->data[2] << 0) |  (msg->data[3] << 8) | (msg->data[4] << 16) | (msg->data[5] << 24 );
			}
		}
		else if(pgn == 0x00FD07 && gRealData.obdDiagProt == 2)//MIL״̬
		{
			//MIL״̬
			gRealData.milState = calcRealValue(6,2,1,0,CALC_MODE,msg->data,HIGHTLOW_MODE,CAN_MODE);
		}
		else if(pgn == 0x00FECE && gRealData.obdDiagProt == 2)//����״̬
		{
			//getJ1939Flag[2] = 1;
			/* ���֧��״̬ */
			//�߻�ת����
			gRealData.diagSpState.catalyst = calcCanValue(32,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//���ȴ߻�ת����
			gRealData.diagSpState.heatedCatalyst = calcCanValue(33,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//����ϵͳ
			gRealData.diagSpState.evaporativeSys = calcCanValue(34,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//���ο���ϵͳ
			gRealData.diagSpState.secondaryAirSys = calcCanValue(35,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//ACϵͳ�����
			gRealData.diagSpState.acSysRefrigerant = calcCanValue(36,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//����������
			gRealData.diagSpState.exhaustGasSensor = calcCanValue(37,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//����������������
			gRealData.diagSpState.exhaustGasSensorHeater = calcCanValue(38,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//EGR��Vvtϵͳ
			gRealData.diagSpState.egrAndVvtSys = calcCanValue(39,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//����������ϵͳ
			gRealData.diagSpState.coldStartAidSys = calcCanValue(40,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			
			/*��Ͼ���״̬*/
			//�߻�ת����
			gRealData.diagRdyState.catalyst = calcCanValue(48,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//���ȴ߻�ת����
			gRealData.diagRdyState.heatedCatalyst = calcCanValue(49,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//����ϵͳ
			gRealData.diagRdyState.evaporativeSys = calcCanValue(50,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//���ο���ϵͳ
			gRealData.diagRdyState.secondaryAirSys = calcCanValue(51,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//ACϵͳ�����
			gRealData.diagRdyState.acSysRefrigerant = calcCanValue(52,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//����������
			gRealData.diagRdyState.exhaustGasSensor = calcCanValue(53,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//����������������
			gRealData.diagRdyState.exhaustGasSensorHeater = calcCanValue(54,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//EGR��Vvtϵͳ
			gRealData.diagRdyState.egrAndVvtSys = calcCanValue(55,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//����������ϵͳ
			gRealData.diagRdyState.coldStartAidSys = calcCanValue(56,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			
			
			
			//��ѹѹ������ϵͳ
			gRealData.diagRdyState.boostPressureCtrlSys = calcCanValue(57,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//DPF���
			gRealData.diagRdyState.dpf = calcCanValue(58,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//ѡ���Դ߻���ԭϵͳ��SCR����NOx������
			gRealData.diagRdyState.scrOrNOxAdsorber = calcCanValue(59,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//NMHC�����߻���
			gRealData.diagRdyState.nmhcConvertingCatalyst = calcCanValue(60,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			
			
			//��ѹѹ������ϵͳ
			gRealData.diagSpState.boostPressureCtrlSys = calcCanValue(41,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//DPF���
			gRealData.diagSpState.dpf = calcCanValue(42,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//ѡ���Դ߻���ԭϵͳ��SCR����NOx������
			gRealData.diagSpState.scrOrNOxAdsorber = calcCanValue(43,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//NMHC�����߻���
			gRealData.diagSpState.nmhcConvertingCatalyst = calcCanValue(44,1,msg->data,HIGHTLOW_MODE,CAN_MODE);


			//ʧ��
			gRealData.diagSpState.misfire = calcCanValue(24,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//ȼ��ϵͳ
			gRealData.diagSpState.fuelSys = calcCanValue(25,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//�ۺϳɷ�
			gRealData.diagSpState.comprehensiveComponent = calcCanValue(26,1,msg->data,HIGHTLOW_MODE,CAN_MODE);

			//ʧ��
			gRealData.diagRdyState.misfire = calcCanValue(28,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//ȼ��ϵͳ
			gRealData.diagRdyState.fuelSys = calcCanValue(29,1,msg->data,HIGHTLOW_MODE,CAN_MODE);
			//�ۺϳɷ�
			gRealData.diagRdyState.comprehensiveComponent = calcCanValue(30,1,msg->data,HIGHTLOW_MODE,CAN_MODE);			
		}
	}
	if(source == 0 && sendCmdTimeout > 0 && gRealData.obdDiagProt == 2 && j1939Cmd[sendCmdIdx] == pgn)
	{
		sendCmdIdx++;
		sendCmdTimeout = 0;
	}
}
											
static void Udstask(void)
{
	//�����������������������
	if(g_system_dt.second != sendCmdSec)
	{
		sendCmdSec = g_system_dt.second;
		//Ӧ��ʱ
		if(sendCmdTimeout > 0)
		{
			sendCmdTimeout--;								//����ʱ����
			if(sendCmdTimeout == 1)					//3�볬ʱ
			{
				if(gRealData.obdDiagProt == 2 && sendCmdIdx == 4)//J1939��֧��IUPR����ʱ����
				{
					sendCmdIdx++;
				}
				else if(gRealData.obdDiagProt == 1 && (sendCmdIdx == 5 || sendCmdIdx == 6 || sendCmdIdx == 7))//ISO27145��֧�ִ��ֶ�MIL״̬ ��IUPR״̬����ʱ����
				{
					sendCmdIdx++;
				}
				sendCmdTimeout = 0;
			}
		}
		//�������ϵͳ����ռ��
		afterOtherCnt = (afterOtherCnt < 0xFF) ? (afterOtherCnt + 1) : afterOtherCnt;
		if(afterOtherCnt >= 4)
			startSecCnt = (startSecCnt < 0xFF) ? (startSecCnt + 1) : startSecCnt;//����������48���2��30����
		/*/�ϴ�OBD�������
		if(gUploadCan == 0xAA)
		{
			gTerminalState.obdState = 0;
			memset(getJ1939Flag,0,sizeof(getJ1939Flag));
			memset(get27145Flag,0,sizeof(get27145Flag));
			memset(get15765Flag,0,sizeof(get15765Flag));
			sendCmdIdx = 0;
			startSecCnt = 45;
			obdLog = 1;
			gUploadCan = 1;
		}*/
		if(startSecCnt >= 45 && startSecCnt <= 150 && sendCmdTimeout == 0 && gRealData.engineSpeed >= 100 && gRealData.engineSpeed < 0xFFFE && afterOtherCnt >= 4)
		{
			//��̽Э��˳�����
			getObdDiagProtIdx++;
			getObdDiagProtIdx = getObdDiagProtIdx % 3;
			if((gRealData.obdDiagProt == 0 && sendCmdIdx < (sizeof(uds15765Cmd) / 8)) || (gRealData.obdDiagProt == 1 && sendCmdIdx < (sizeof(uds27145Cmd) / 8)) || (gRealData.obdDiagProt == 2 && sendCmdIdx < (sizeof(j1939Cmd) / 4)) || gRealData.obdDiagProt == 0xFF)
			{
				CAN_msg msg_buf = {0x18eafff9,{ 0x30, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 },8,0,EXTENDED_TYPE,DATA_FRAME};
				osDelay(100);
				if(gRealData.obdDiagProt == 1 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 0))//ISO27145Э���δ֪
				{
					if(get27145Flag[sendCmdIdx] == 1)//����Ҫ�ظ���ȡ
					{
						sendCmdIdx++;
						if(sendCmdIdx >= sizeof(uds27145Cmd) / 8)
						{
							return;
						}
					}
					memcpy(msg_buf.data,uds27145Cmd[sendCmdIdx],8);
					sendCmdTimeout = 4;
				}
				if(gRealData.obdDiagProt == 0 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 1))//ISO15765Э���δ֪
				{
					if(get15765Flag[sendCmdIdx] == 1)//����Ҫ�ظ���ȡ
					{
						sendCmdIdx++;
						if(sendCmdIdx >= sizeof(uds15765Cmd) / 8)
						{
							return;
						}
					}
					memcpy(msg_buf.data,uds15765Cmd[sendCmdIdx],8);
					sendCmdTimeout = 4;
				}
				if(sendCmdTimeout > 0)
				{
					//��չ֡
					if(sendUdsId == 0 || sendUdsId == 0x18DA00F1 || sendUdsId == 0x18DA10F1)
					{
						msg_buf.format = EXTENDED_TYPE;	
						//����Ѱַ��0x18DB33F1������Ѱַ��0x18DA00F1 0��ECU��Ӧ��0x18DAF100
						if(gRealData.obdDiagProt == 0 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 1))
						{
							msg_buf.id = 0x18DB33F1;
							osDelay(10);					
							CAN_send (1, &msg_buf,0x0F00);							
						}
						else
						{
							if(sendUdsId == 0 || sendUdsId == 0x18DA00F1)
							{
								msg_buf.id = 0x18DA00F1;
								osDelay(10);					
								CAN_send (1, &msg_buf,0x0F00);
							}
							if(sendUdsId == 0 || sendUdsId == 0x18DA10F1)
							{
								msg_buf.id = 0x18DA10F1;
								osDelay(10);					
								CAN_send (1, &msg_buf,0x0F00);							
							}
						}
					}
					//��׼֡
					if(sendUdsId == 0 || sendUdsId == 0x7E0)
					{
						//����Ѱַ��0x7DF������Ѱַ��0x7E0 0��ECU��Ӧ��7E8
						msg_buf.id = 0x7E0;
						msg_buf.format = STANDARD_TYPE;
						osDelay(10);
						CAN_send (1, &msg_buf,0x0F00);
					}
				}
				if(gRealData.obdDiagProt == 2 || (gRealData.obdDiagProt == 0xFF && getObdDiagProtIdx == 2))//SAE J1939Э���δ֪
				{
					msg_buf.data[0] = j1939Cmd[sendCmdIdx] >> 0;
					msg_buf.data[1] = j1939Cmd[sendCmdIdx] >> 8;
					msg_buf.data[2] = j1939Cmd[sendCmdIdx] >> 16;
					msg_buf.id = 0x18eafff9;
					msg_buf.format = EXTENDED_TYPE;
					osDelay(10);
					CAN_send (1, &msg_buf,0x0F00);
					sendCmdTimeout = 4;
				}
			}
			else
			{
				//OBD������
				gTerminalState.obdState = 1;
				if(gSysPara.carType != emissionLevel)
				{
					gSysPara.carType = emissionLevel;
					System_Pare_Save();
				}
			}
		}
	}
	//������Ϩ��
	if(gRealData.engineSpeed <= 10 || gRealData.engineSpeed >= 0xFFFE)
	{
		scrInReady = 0;
		scrOutReady = 0;
		memset(getJ1939Flag,0,sizeof(getJ1939Flag));
		memset(get27145Flag,0,sizeof(get27145Flag));
		memset(get15765Flag,0,sizeof(get15765Flag));
		startSecCnt = 0;
		sendCmdIdx = 0;
		gTerminalState.obdState = 0;
		actAreaInjectValStamp = 0;		
		totalUreaUsedStamp1 = 0;
		totalUreaUsedStamp2 = 0;	
		engReferenceTorqueStamp = 0;
		engTotalHoursOfOperationStamp = 0;
	}
	gTerminalState.getCarData = gTerminalState.obdState;
}


