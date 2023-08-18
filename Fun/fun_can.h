#ifndef __FUN_CAN_H
#define __FUN_CAN_H

#include "protocol_GB.h"
#include "bsp_can.h"

#define MAX_CANFAIL 				50							/*CAN���ʧ�ܴ�����������������ΪCAN�Ͽ������5������*/

extern const char* CAR_TYPE;								/*���� */
extern const uint8_t CALC_EXTREMUN;					/*�Ƿ���㼫ֵ*/

extern void funCanInit(void);
extern void funCanrun(void);
extern uint8_t fun_can_Get_State(uint8_t ch);
extern uint32_t fun_can_Get_recvCnt(uint8_t ch);

/*����CAN��ֵ*/
/*˵����
	nstartPos:��ʼλ
	nlen:���ݳ���
	pcanVal:can������
	hightLowMode:�Ƿ��λ��ǰ�������λ��ǰ��Ҫת��Ϊ��λ��ǰ
	canMode:can�����ʽ��Ӣ�ض���ʽ��Ħ���޸��ʽ��*/
extern unsigned int calcCanValue(unsigned char nstartPos,unsigned char nlen,const unsigned char* pcanVal,
															BYTE_MODEL hightLowMode,CANANA_MODEL canMode);

/*����CANʵʱ��ֵ����ƫ������ϵ������*/
/*˵����
	nstartPos:��ʼλ
	nlen:���ݳ���
	factor:ϵ��
	offset:ƫ����
	pcanVal:can������
	hightLowMode:�Ƿ��λ��ǰ�������λ��ǰ��Ҫת��Ϊ��λ��ǰ
	canMode:can�����ʽ��Ӣ�ض���ʽ��Ħ���޸��ʽ��*/
extern float calcRealValue(unsigned char nstartPos,unsigned char nlen,float factor,float offset,CALC_MODEL calcModel,
														const unsigned char* pcanVal,BYTE_MODEL hightLowMode,CANANA_MODEL canMode);	


typedef struct
{
	uint32_t code;																																									//����
	uint32_t timeStamp;																																							//ʱ���
}ROTACODE;

extern void updateFaultCode(ROTACODE* roataCode,uint8_t maxCnt,uint32_t newCode,uint32_t timeout);		//���¹�����
extern uint8_t checkFaultCode(ROTACODE* roataCode,uint8_t maxCnt,uint32_t code);											//����Ƿ���ڹ�����

extern void setCANmask(uint8_t idx,uint32_t id,uint32_t mask,uint8_t format);													//����CAN���ˣ��ܹ�27��
extern void iniCanData(void);																																					//��ʼ��CAN����
extern void iniFvCanData(void);																																				//��ʼ����Դ����
extern void iniEvCanData(void);																																				//��ʼ����Դ����
extern void unpackCAN(uint8_t ch,CAN_msg *msg);																												//����CAN����
extern void unpackFvCAN(uint8_t ch,CAN_msg *msg);																											//����ȼ�ͳ�����
extern void unpackEvCAN(uint8_t ch,CAN_msg *msg);																											//�����綯������
extern void udsInit(uint8_t ch);																																						//UDS��ϳ�ʼ��
extern void udsProc(void);																																						//UDS��ϴ���

#endif
