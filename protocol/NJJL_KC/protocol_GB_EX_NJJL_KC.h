/*
�� ����protocol_GB_EX_NJJL_KC.c
�� �ܣ��Ͼ���ͨ������Э�� - ���������Զ�����չ����
�� ��: 2021/12/30
�� ˾��������Դ(��ɽ)��Ϣ�Ƽ����޹�˾
�� ��: CZJ -> LGC
*/
#ifndef __NJLL_KC_DEF_H
#define __NJLL_KC_DEF_H

#include "cmsis_os2.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stdint.h"
#include "fun_can.h"
#include "Fun_Net.h"
#include "bsp_rtc.h"
#include "bsp_ec20.h"
#include "algo_verify.h"
#include "protocol_GB.h"

#include "fun_mon.h"
#include "bsp_gps.h"
#include "bsp_storage.h"
#include "bsp_io.h"
#include "bsp_sys.h"
#include "bsp_power.h"
#include "protocol_GB32960.h"
#include "bsp_storage.h"
#include "rl_fs.h"
#include "bsp_uart_fifo.h"


enum NJJL_GB_CODE
{
	CMD_LOCKCAR = 0x83,								//����ָ��	
	CMD_CARDOORRSP = 0xC0,						//����Ӧ��
	CMD_UPAPP = 0xC4,									//��������Ӧ��
	CMD_LOCKCARRRSP = 0xC5,						//����Ӧ��(��������ϱ�)
};

typedef struct{
	uint8_t store_flag;									//�洢��־

	uint8_t carDoorCtrCode;							//�������ƴ���  1:����ָ�� 2:����ָ�� ����:�޲���
	uint8_t isReturnLockState;					//0xA5:����Ӧ�� 0xAA����Чָ��
	uint16_t carDoorCtrCount;						//Զ��������ˮ�� 
	uint8_t upDataHeartLock;						//������º�Ĭ������һ�� ������ָ��

	uint16_t carDoorCtrRspCode;					//Զ������Ӧ�� 0x00:�ս��յ�ƽָ̨�� 0xFF:�ȵ�
	uint8_t remoteLockState;						//Զ������״̬
	uint8_t heartLockState;							//������״̬
}USER_DATA;


//�Զ���B0���� - �������״̬
typedef struct{
	uint32_t sBatsOutTotalEnergy;				//�ۼ����ĵ���
	uint32_t sBatsChgTotalEnergy;				//�ۼƳ������
	uint8_t sBatsBreakOnceEnergy;				//�����ƶ���������
	uint8_t sSOH;												//SOH
	uint16_t sBatsPower;								//��ع���
	
}SelfDataB0,*pSelfDataB0;

//�Զ���B2���� - ������ر���
typedef struct{
	uint8_t sBMSDTC_CODE1;							//sBMSDTC_CODE1
	uint8_t sBMSDTC_CODE2;							//sBMSDTC_CODE2
	uint8_t sBMSDTC_CODE3;							//sBMSDTC_CODE3
	uint8_t sBMSDTC_CODE4;							//sBMSDTC_CODE4
	uint8_t sBMSDTC_CODE5;							//sBMSDTC_CODE5
	uint8_t sBMSDTC_CODE6;							//sBMSDTC_CODE6
	uint8_t sBMSDTC_CODE7;							//sBMSDTC_CODE7
	uint8_t sBMSDTC_CODE8;							//sBMSDTC_CODE8
	
	uint8_t sBMSCode1;									//BSM����CODE1
	uint8_t sBMSCode2;									//BSM����CODE2
	
	uint8_t sDTC_Code;									//DTC_CODE
	
}SelfDataB2,*pSelfDataB2;

//�Զ���B3���� - �����������ӿ�����
typedef struct{
	uint8_t sSlaveControlOnLine_1_8;		//1-8	�ӿ�����״̬
	uint8_t sSlaveControlOnLine_9_16;		//9-16	�ӿ�����״̬
	uint8_t sSlaveControlOnLine_17_24;	//17-24	�ӿ�����״̬
	uint8_t sSlaveControlOnLine_25_32;	//25_32	�ӿ�����״̬
	
	uint8_t sBatsCount;									//���������
	uint8_t sBatSlaveControlCount;			//�ӿ���

}SelfDataB3,*pSelfDataB3;

//�Զ���B4���� - ����������������
typedef struct{
	uint16_t sTCU_FaultCode;						//TCU���ϴ���
	uint8_t sTCU_OtherFaultCode;				//TCU�������ϴ���
	uint8_t sTCU_FaultLevel;						//TCU���ϵȼ�
	uint8_t sMoter_FaultLevel;					//����������ϵȼ�
	uint16_t sMoter_DTCCode;						//�������DTC_CODE
	uint16_t sMotor_FaultCount;					//��������������� 
	
	uint8_t sMCU_TempAlarm;							//MCU�¶ȱ���
	uint8_t sMotor_TempAlarm;						//��������¶ȱ���

}SelfDataB4,*pSelfDataB4;

//�Զ���B5���� - ��������������Ϣ
typedef struct{
	uint8_t sCarFaultLevel;							//�������ϵȼ�
	uint8_t sCarFaultNum1;							//����������1
	uint8_t sCarFaultNum2;							//����������2
	uint8_t sCarFaultNum3;							//����������3
	uint8_t sCarFaultNum4;							//����������4
	uint8_t sCarFaultNum5;							//����������5
	
}SelfDataB5,*pSelfDataB5;

//�Զ���BF���� - ������
typedef struct{
	uint8_t sCE_LockST;									//������״̬
	uint8_t sCE_DropLockST;							//����������״̬
	
}SelfDataBF,*pSelfDataBF;


extern SelfDataB0 *gSelfDataB0;							/*�Ͼ����������Զ�������*/
extern SelfDataB2 *gSelfDataB2;
extern SelfDataB3 *gSelfDataB3;
extern SelfDataB4 *gSelfDataB4;
extern SelfDataB5 *gSelfDataB5;
extern SelfDataBF *gSelfDataBF;

extern uint16_t selfDataB0Pos;
extern uint16_t selfDataB2Pos;
extern uint16_t selfDataB3Pos;
extern uint16_t selfDataB4Pos;
extern uint16_t selfDataB5Pos;
extern uint16_t selfDataBFPos;

extern uint8_t sendLockCMDSign;					//����ָ��ͱ�־

extern uint16_t upFirstSendLockTime;		//ÿ�γ�����·���������ָ��
extern uint32_t upDataFirstTime;				//ÿ�γ������ ���ͼ�ʱ

extern USER_DATA gUserDara;							//�û�����

extern void unpackSelfInit(void);												//������ݳ�ʼ��
extern void unpackSelfcan(uint8_t ch,CAN_msg *msg);			//������ݽ���
extern void saveUserData(void);													//�����û�����

#endif

