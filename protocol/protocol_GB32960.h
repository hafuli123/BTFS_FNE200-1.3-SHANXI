#ifndef __PROTOCOL_GB32960_H__ 
#define __PROTOCOL_GB32960_H__ 

#include "pdu.h"

#define EV1_SERVER 	0							//����Դ�����·,
#define EV2_SERVER 	1							//����Դ�㲥��·
#define EV3_SERVER  2							//����Դѡ����·

typedef enum
{
	GB32960_TEST_LOGIN = 1,
	GB32960_TEST_LOGOUT = 2,
	GB32960_TEST_ONLINE = 3,
	GB32960_TEST_OFFLINE = 4,
	GB32960_TEST_UNALARM = 5,
	GB32960_TEST_ALARM = 6,
	GB32960_TEST_UNPRINT = 7,
	GB32960_TEST_PRINT = 8,
}GB32960_CRTL_CODE;

//����ʾ�� ��������·�ţ��������˿ڣ�ʵʱ���ڣ���������
void* gb32960Init(uint8_t link,uint8_t* pBuff,uint16_t pBuffSize,char*domain,uint32_t* port,char* vin,uint8_t realHz,uint8_t heartbeatHz);
uint8_t gb32960Run(void* obj);
uint8_t gb32960GetSta(uint8_t id);
uint8_t gb32960Ctrl(uint8_t ctrl);
void cleanPara(void* obj);							//�����·����

//��չЭ���ʼ�� protЭ��ţ�����Ҫ��Э��
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen);
//��չЭ������   ctrl:0 �������� 1:�������� 2:˯�� sta:0 ����δ���� 1:�����ѵ��� ����ֵ:0 ��չЭ��δ���� 1:��չЭ���ѵ���
uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta);
//��չЭ��������
uint8_t extUnpack(uint8_t link,const uint8_t *szRecvBuf,uint16_t rLen);
//��չʵʱ���ݣ�ʵʱ���������Զ�������
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen);
//��չ���ܺ�ʶ�𷽷������������ն˱��
uint8_t extGetVin(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *resvBuff);
//�������ƽ̨����
uint8_t extCtrlNetSta(void* obj);																
//��ȡ���ݻ���
RealData* getRealCache(uint8_t dataIdx,uint8_t* outDataIdx);
#endif
