/*
�ļ���bsp_gprs_EC20.c
���ܣ���ԶEC20 gprs��gps����һģ��GPRS��������
���ڣ�2017/11/22
��˾���麣Ŧ�����Զ����������޹�˾
���ߣ�chenshaojian
*/

#ifndef __BSP_GPRS_EC20_H
#define __BSP_GPRS_EC20_H

#include "stdint.h"

/* ����������仰, �����յ����ַ����͵����Դ���1 */
//#define SIM800_TO_COM1_EN

extern uint8_t gprsRecvBuf[2048];
extern char ipbuf[100];

/* ��λģ��,�Զ��ָ� */
void EC20_Reset(void);
/* SIM��״̬��� */
uint8_t EC20_SIMCardStatus(void);
/* ����״̬��ѯ */
uint8_t EC20_GetGPRSNetStatus(void);
/*PPP����*/
uint8_t EC20_PppdCallGPRS(void);
/* ��ѯTCP/IP�������,�˺�����Ҫһֱ����,���TCP��������,�ſ�����������������ݰ� */
void EC20_QueryConnectionStatus(void);
/*FTP����*/
uint8_t EC20_ftpDownload(void);
/* GPRS�������� */
uint8_t EC20_SendData(uint8_t linkIndex,uint8_t* wbuf,uint16_t len);
/* GPRS�������� */
uint16_t EC20_ReadData(void);
/* ��ȡCCID */
uint8_t EC20_GetCCID(uint8_t* rbuf, uint8_t rLen);
uint8_t EC20_GetIMEI(uint8_t* rbuf, uint8_t rLen);
uint8_t EC20_GetIMSI(uint8_t* rbuf, uint8_t rLen);
uint8_t EC20_GetNetworkMode(void);
/* ��ѯ�ź�ǿ�� */
uint8_t EC20_GetSignalQuality (void);
/* �ر�TCP���� */
void EC20_IPClose(void);
/*�ر�TCP��·*/
void EC20_linkClose(uint8_t link);
/* ����TCP���� */
void EC20_TCP_Connect(uint8_t linkIndex,char *domain,uint16_t port);
uint8_t EC20_OpenGPS(void);
uint8_t EC20_CloseGPS(void);

uint8_t EC20_OpenSleepModel(void);				//����˯��ģʽ
#endif

