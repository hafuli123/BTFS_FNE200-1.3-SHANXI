#ifndef __FUN_MON_H
#define __FUN_MON_H

#include "stdint.h"

extern void funMonInit(void);
extern void funMonrun(void);
/*
�ص���������˵��
type:0-����������parameter��uint8_t* ��������
type:1-�����ɹ���parameter��NULL
type:2-����ʧ�ܣ�parameter��������
*/
extern void funMonSetFtpParas(char *hostname,uint16_t hostport,char *username,char *password,char *filePach,char* fileName,void (*callback)(uint8_t type,void* parameter));

extern uint8_t funGet_ACCState(void);					//��ȡACC����״̬
extern uint8_t funGet_ChgState(void);					//��ȡ�������״̬



#endif
