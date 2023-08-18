/*
文件：bsp_gprs_EC20.c
功能：移远EC20 gprs、gps二合一模块GPRS解析函数
日期：2017/11/22
公司：珠海纽安特自动化技术有限公司
作者：chenshaojian
*/

#ifndef __BSP_GPRS_EC20_H
#define __BSP_GPRS_EC20_H

#include "stdint.h"

/* 定义下面这句话, 将把收到的字符发送到调试串口1 */
//#define SIM800_TO_COM1_EN

extern uint8_t gprsRecvBuf[2048];
extern char ipbuf[100];

/* 复位模块,自动恢复 */
void EC20_Reset(void);
/* SIM卡状态检查 */
uint8_t EC20_SIMCardStatus(void);
/* 网络状态查询 */
uint8_t EC20_GetGPRSNetStatus(void);
/*PPP拨号*/
uint8_t EC20_PppdCallGPRS(void);
/* 查询TCP/IP连接情况,此函数需要一直调用,如果TCP连接正常,才可以向服务器发送数据包 */
void EC20_QueryConnectionStatus(void);
/*FTP升级*/
uint8_t EC20_ftpDownload(void);
/* GPRS发送数据 */
uint8_t EC20_SendData(uint8_t linkIndex,uint8_t* wbuf,uint16_t len);
/* GPRS接收数据 */
uint16_t EC20_ReadData(void);
/* 获取CCID */
uint8_t EC20_GetCCID(uint8_t* rbuf, uint8_t rLen);
uint8_t EC20_GetIMEI(uint8_t* rbuf, uint8_t rLen);
uint8_t EC20_GetIMSI(uint8_t* rbuf, uint8_t rLen);
uint8_t EC20_GetNetworkMode(void);
/* 查询信号强度 */
uint8_t EC20_GetSignalQuality (void);
/* 关闭TCP连接 */
void EC20_IPClose(void);
/*关闭TCP链路*/
void EC20_linkClose(uint8_t link);
/* 创建TCP连接 */
void EC20_TCP_Connect(uint8_t linkIndex,char *domain,uint16_t port);
uint8_t EC20_OpenGPS(void);
uint8_t EC20_CloseGPS(void);

uint8_t EC20_OpenSleepModel(void);				//开启睡眠模式
#endif

