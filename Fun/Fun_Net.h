#ifndef __Fun_NET_H
#define __Fun_NET_H

#include "stdint.h"

/* GPRSMģ��״̬ */
typedef enum
{
   FUN_GPRS_GET_MOD  = 0,				//��ȡģ��
   FUN_GPRS_GET_SIM = 1,				//��ȡSIM��
   FUN_GPRS_GET_NET = 2,				//��ȡ����
   FUN_GPRS_DIAL = 3,						//����
   FUN_GPRS_MON = 4,						//����
}FUN_GPRS_STA;

/* GPRSMģ�鷴����� */
typedef enum
{
   FUN_GPRS_NONE = 0,						//�޲���
   FUN_GPRS_CONNECTED = 1,			//������
	 FUN_GPRS_CONNECTING = 2,			//������
   FUN_GPRS_LOSING = 3,					//���Ӷ�ʧ
   FUN_GPRS_CLOSING = 4,				//�ر���
   FUN_GPRS_COLSED = 5,					//�ѹر�
   FUN_GPRS_TIMEOUT = 6,				//�ѳ�ʱ
	 FUN_GPRS_FAILED = 7,					//��ʧ��
	 FUN_GPRS_SUCCEED = 8,				//�ѳɹ�
}FUN_GPRS_RET;

//GPRSģ���ʼ��
void Fun_Net_Init(void);
//GPRSģ������
void Fun_Net_Run(void);
//GPRSģ���ȡ״̬
FUN_GPRS_STA Fun_Gprs_GetSta(void);
//GPRSģ���ź�
uint8_t Fun_Gprs_Csq(void);
//GPRSģ��˯��
void Fun_Gprs_Sleep(void);
//GPRSģ�黽��
void Fun_Gprs_WakeUp(void);

//GPRSģ���ȡICCID
uint8_t Fun_Gprs_getICCID(char *buf, uint16_t len);
//GPRSģ���ȡIMEI
uint8_t Fun_Gprs_getIMEI(char *buf, uint16_t len);
//GPRSģ���ȡIMSI
uint8_t Fun_Gprs_getIMSI(char *buf, uint16_t len);
//GPRSģ���ȡ�汾
uint8_t Fun_Gprs_getVer(char *buf, uint16_t len);

//GPRSģ��TCP����
uint8_t Fun_Gprs_Tcp_connect(uint8_t socket_fd,char *hostname,uint16_t hostport);
//GPRSģ��TCP�Ͽ�
uint8_t Fun_Gprs_Tcp_disconnect(uint8_t socket_fd);
//GPRSģ��TCP���ջص�
FUN_GPRS_RET Fun_Gprs_Tcp_Set_RecvCallBack(uint8_t socket_fd,void (*unpackCallBack)(uint8_t,uint8_t*,uint16_t len));
//GPRSģ��TCP״̬
FUN_GPRS_RET Fun_Gprs_Tcp_Sta(uint8_t socket_fd);
//GPRSģ��TCP����
FUN_GPRS_RET Fun_Gprs_Tcp_send(uint8_t socket_fd, uint8_t *buf, uint16_t len);
//GPRSģ��TCP����
uint16_t Fun_Gprs_Tcp_recv(uint8_t socket_fd, uint8_t *buf, uint16_t len);

//GPRSģ��FTP����
uint8_t Fun_Gprs_Ftp_connect(char *hostname,uint16_t hostport,char *username,char *password);
//GPRSģ��FTP�Ͽ�
FUN_GPRS_RET Fun_Gprs_Ftp_disconnect(void);
//GPRS��ȡ�ļ���С,Ŀ¼�á�/������
uint32_t Fun_Gprs_Ftp_GetFileSize(char *filePach,char* fileName);
//GPRS��ȡ�ļ�����
uint32_t Fun_Gprs_Ftp_ReadFile(char* fileName,uint32_t filePos,uint32_t getSize,uint8_t* buff);
//HTTP�ϴ��ļ�����������ֵ0������ʧ�� 1�������ɹ�
uint8_t Fun_Gprs_Http_Post_File_Start(char *url,char *username,char *password,char*fileName,uint32_t fileSize);
//POST�ϴ��ļ��������룬����ֵ 0������ ����0�����볤��
uint32_t Fun_Gprs_Http_Post_File_Input(void* buff,uint32_t len);
#endif

