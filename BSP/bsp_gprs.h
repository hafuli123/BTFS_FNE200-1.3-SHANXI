#ifndef __BSP_GPRS_H
#define __BSP_GPRS_H

#include "pdu.h"

#define MAX_LINK 6

typedef uint8_t (*tcpUnpackDataCallBack)(uint8_t link,const uint8_t *pbuf, uint16_t rLen);

#define MAX_LEN_FTP_URL 50        //FTP��URL��󳤶�
#define MAX_LEN_FTP_USER 20       //FTP���û�����󳤶�
#define MAX_LEN_FTP_PWD 20        //FTP��������󳤶�
#define MAX_LEN_FTP_DIR 100				//FTPĿ¼��󳤶�
#define MAX_LEN_FTP_NAME 40				//FTP�ļ�����󳤶�

typedef struct Ftp_Pars
{
	char ftpUrl[MAX_LEN_FTP_URL];		//FTP��������ַ
	char ftpUser[MAX_LEN_FTP_USER];	//FTP�û���
	char ftpPwd[MAX_LEN_FTP_PWD];		//FTP����
	uint16_t ftpPort;								//FTP�������˿�
	char ftpName[MAX_LEN_FTP_NAME];	//FTP�ļ���
	char ftpDir[MAX_LEN_FTP_DIR];		//FTP�ļ���
	uint8_t parts;									//�����㲿��
}FTP_PARS;

extern FTP_PARS gFtpParas;

typedef struct
{
	uint8_t used;
	uint8_t linkState;
	char *address;
	uint16_t port;
	uint16_t connectCnt;	//������ʱ
	uint16_t connectHZ;
	uint8_t ack;
	uint8_t nAckTm;
	tcpUnpackDataCallBack UnpackData;
}TCP_CLIENT_PARA;

extern uint8_t gftp_Mode;        /* FTP״̬ */

/* SIMģ���ʼ�� */
void bsp_InitGprs(void);
/* ��λģ��,�Զ��ָ� */
void simReset(void);
/* SIM��״̬ */
uint8_t simGetSIMCardStatus(void);
/* GPRS���ţ����Զ�����Ŀ������� */
uint8_t simPppdCallGPRS(void);
/* ��ѯTCP/IP�������,�˺�����Ҫһֱ����,���TCP��������,�ſ�����������������ݰ� */
void simQueryConnectionStatus(void);
/*��ѯ��·״̬*/
uint8_t simQueryLinkStatus(uint8_t link);
/*��ѯ�����ź�ǿ��*/
uint8_t simGetSignalQuality (void);
/* SIM800ģ��ftp���� */
uint8_t simFtpUpdate(void);
/* GPRS�������� */
uint8_t simSendData(uint8_t linkIndex,uint8_t* wbuf,uint16_t len);
/* �������Ӳ��� */
uint8_t simSetUnpackdataFun(uint8_t link,char *address,uint16_t port,tcpUnpackDataCallBack UnpackDataFunc,uint8_t used);
/* ��ȡICCID */
uint8_t simGetCCID(uint8_t* rbuf, uint8_t rLen);
/* �ر�TCP���� */
void simIPClose(void);
/*�ر�TCP��·*/
void simLinkClose(uint8_t link);
/*TCP��·�Ͽ�����*/
void reConnect(uint8_t link);
/* ����TCP���� */
void simTcpConnect(uint8_t linkIndex,char *domain,uint16_t port);
/*��ȡ����ע��״̬*/
uint8_t simGetGPRSNetStatus(void);
/*��������*/
void gprsPppdask(void);
/*�ز�*/
void gprsRedial(void);
/*FTP��������*/
void ftpUpdateApp(void);
uint8_t getNAckTm(uint8_t link);
uint8_t simOpenSleepModel(void);				//����˯��ģʽ

extern uint8_t gprs_State;     	        				/* GPRS״ָ̬ʾ�� */
extern uint8_t gFtp_Mode;               				/* FTP״̬ 1-FTP���� 2-���ݵ��� 3-SD������ */

#endif

