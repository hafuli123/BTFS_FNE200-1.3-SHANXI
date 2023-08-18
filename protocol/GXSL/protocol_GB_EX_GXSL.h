//����ͨ�����ļ�
#ifndef __DST_DEF_H
#define __DST_DEF_H

//#include "newant_bms.h"
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


//�û����ݴ洢
typedef struct{
	uint8_t store_flag;         //�����־	
	uint32_t wk_Interval;				//���Ѽ��
	uint8_t wk_SendInterval;		//���Ѻ���������ʱ���� (��λ��10s)
	uint16_t wk_Year;						//��������
	uint8_t wk_Month;	
	uint8_t wk_Day	;
	uint8_t wk_Hour;
	uint8_t wk_Min;
	uint8_t wk_Sec;		
	
	uint8_t runCMD;							//����ָ�����λ
	uint8_t lockCMD;						//��������:0x55,һ����0x56,������0x57,������0x58,�ļ���0x59,�弶��0x60,����һ����0xAA,������
	uint8_t changeCMD;					//����汾�л����ƣ�0x55�������汾��0xAA���������汾
	uint8_t	bindingCMD;					//�󶨿��ƣ�0x55��ִ�а󶨣�0x00��Ĭ��
	uint8_t doorCMD;						//���ſ��ƣ�����:0x55, ����:0xAA,  Ĭ��:0x00;
	uint8_t maintCMD;						//������ʾ
	uint8_t payCMD;							//�ɷ���ʾ
	uint8_t yearCheckCMD;				//������ʾ
	uint32_t outLineMile;				//������ʱ�ۼ����
	uint32_t outSiteMile;				//�޶�λ�ۼ����
	
	float autoChgVolt;					//�貹���ѹ			
	float autoChgOverVolt;			//ֹͣ�����ѹ			
	float underVolt;						//Ƿѹֵ								
	
	char httpURL[160];								//URL
	char httpUserName[20];						//�û���
	char httpPassWord[20];						//����
	
	uint32_t outLineTime;				//�����쳣��ʱ
	uint8_t isSiteExcute;
}USER_DATA;


/* ����������ָ�� */
enum DST_GB_CODE
{
	CMD_LOCKCMD = 0x84,						//��������ָ��(����)
	CMD_SOFTVER_CHANGE = 0x85,		//����汾�л����ƣ�0x55�������汾��0xAA���������汾
	CMD_BINDINGCTR = 0x86,				//�󶨿��ƣ�0x55��ִ�а󶨣�0x00��Ĭ��
	CMD_DOORCTR	= 0x87,						//���ſ��ƣ�����:0x55, ����:0xAA, 
	CMD_MAINTTIPS = 0x88,					//������ʾ
	CMD_PAYTIPS = 0x89,						//�ɷ���ʾ
	CMD_CHECKTIPS = 0x8A,					//������ʾ
	
	CMD_SENDVINCMD = 0x0B,				//�����״λ�ȡVIN
	CMD_CHGCMD = 0x0C,						//���ͳ�����
};

/* �������Զ��� 0A���� */
typedef struct _selfData0A{
	uint8_t sTboxST				:3;					//Tbox����״̬
	uint8_t sTermBindingST:1;					//�ն˰�״̬
	uint8_t sQorS_CHGSign	:2;					//�����������־
	uint8_t sAirWorkST		:2;					//�յ�����״̬

	uint8_t sVCUVerST			:1;					//�����������汾״̬
	uint8_t sPTCST				:1;					//PTC ����״̬
	uint8_t sPumpFault		:1;					//ˮ�ù���
	uint8_t sVacuumPumpST	:1;					//��ձ�״̬
	uint8_t sPTCRelayST		:1;					//PTC�̵���ʵ��״̬
	uint8_t sBatRelayST		:1;					//������ؼ��ȼ̵���״̬
	uint8_t sQ_CHG_Fault	:1;					//���̵���ճ������
	uint8_t sBreakST			:1;					//��ɲ״̬

	uint8_t sRelayST			:2;					//�̵���״̬
	
	uint8_t sLockCarST;								//����״̬
	uint8_t sLockCarSign;							//������־
	uint8_t sFaultCT;									//������
	uint16_t sCarType;								//����ʶ��	
	uint8_t sTboxNetCSQ;							//TBOX �����ź�
	uint8_t sAirCurr;									//�յ���������
	uint8_t sVacuumValue;							//��ն�
	uint8_t sTboxSoftVer[5];					//Tbox�̼��汾��
	uint8_t sVCUVerNum[8];						//VCU�汾��
}SelfData0A;

extern USER_DATA gUserDara;									//�û����ݴ洢
extern const char* getCarType(void);
extern SelfData0A* pSelfData0A;					
extern void unpackDSTcan(uint8_t ch,CAN_msg *msg);			/* ������CAN�������� */
extern uint8_t autoChgState;														//���� ����״̬λ 			1���ն��������𲹵�����		 0�������𲹵�����
extern uint8_t sAllowAutoChg;														//���� ����ָ�����λ  1��VCU��������ն˽�ֹ���� 0�������Զ�����
extern RTC_INFOR getVINTime;														/* �״λ�ȡVIN��ʱ�� */
extern double glongd;									            	 		//����
extern double glatd;									              		//γ��
extern uint8_t fisrtGetVIN;															/* �Ƿ�ʱ�״λ�ȡVIN 1�ǣ�0�� */
extern uint8_t getCanLogSwitch;													/* ��ȡCAN��־���� Ĭ��ֵ 0 ָ����1��ִ�гɹ����2 */


extern void saveUserData(void);													/* �����û����� */
extern void startSendCANData(uint8_t ch,CAN_msg *msg);	/* ����CAN���ݽӿ� */
extern uint8_t isEnableSendCANData(void);								/* ��ѯCAN�����Ƿ������� */	
extern void sendIntervalData(unsigned char canCh);			//������������



#endif

