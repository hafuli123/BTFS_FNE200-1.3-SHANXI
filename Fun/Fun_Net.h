#ifndef __Fun_NET_H
#define __Fun_NET_H

#include "stdint.h"

/* GPRSM模块状态 */
typedef enum
{
   FUN_GPRS_GET_MOD  = 0,				//获取模块
   FUN_GPRS_GET_SIM = 1,				//获取SIM卡
   FUN_GPRS_GET_NET = 2,				//获取网络
   FUN_GPRS_DIAL = 3,						//拨号
   FUN_GPRS_MON = 4,						//监视
}FUN_GPRS_STA;

/* GPRSM模块反馈结果 */
typedef enum
{
   FUN_GPRS_NONE = 0,						//无操作
   FUN_GPRS_CONNECTED = 1,			//已连接
	 FUN_GPRS_CONNECTING = 2,			//连接中
   FUN_GPRS_LOSING = 3,					//连接丢失
   FUN_GPRS_CLOSING = 4,				//关闭中
   FUN_GPRS_COLSED = 5,					//已关闭
   FUN_GPRS_TIMEOUT = 6,				//已超时
	 FUN_GPRS_FAILED = 7,					//已失败
	 FUN_GPRS_SUCCEED = 8,				//已成功
}FUN_GPRS_RET;

//GPRS模块初始化
void Fun_Net_Init(void);
//GPRS模块运行
void Fun_Net_Run(void);
//GPRS模块获取状态
FUN_GPRS_STA Fun_Gprs_GetSta(void);
//GPRS模块信号
uint8_t Fun_Gprs_Csq(void);
//GPRS模块睡眠
void Fun_Gprs_Sleep(void);
//GPRS模块唤醒
void Fun_Gprs_WakeUp(void);

//GPRS模块获取ICCID
uint8_t Fun_Gprs_getICCID(char *buf, uint16_t len);
//GPRS模块获取IMEI
uint8_t Fun_Gprs_getIMEI(char *buf, uint16_t len);
//GPRS模块获取IMSI
uint8_t Fun_Gprs_getIMSI(char *buf, uint16_t len);
//GPRS模块获取版本
uint8_t Fun_Gprs_getVer(char *buf, uint16_t len);

//GPRS模块TCP连接
uint8_t Fun_Gprs_Tcp_connect(uint8_t socket_fd,char *hostname,uint16_t hostport);
//GPRS模块TCP断开
uint8_t Fun_Gprs_Tcp_disconnect(uint8_t socket_fd);
//GPRS模块TCP接收回调
FUN_GPRS_RET Fun_Gprs_Tcp_Set_RecvCallBack(uint8_t socket_fd,void (*unpackCallBack)(uint8_t,uint8_t*,uint16_t len));
//GPRS模块TCP状态
FUN_GPRS_RET Fun_Gprs_Tcp_Sta(uint8_t socket_fd);
//GPRS模块TCP发送
FUN_GPRS_RET Fun_Gprs_Tcp_send(uint8_t socket_fd, uint8_t *buf, uint16_t len);
//GPRS模块TCP接收
uint16_t Fun_Gprs_Tcp_recv(uint8_t socket_fd, uint8_t *buf, uint16_t len);

//GPRS模块FTP连接
uint8_t Fun_Gprs_Ftp_connect(char *hostname,uint16_t hostport,char *username,char *password);
//GPRS模块FTP断开
FUN_GPRS_RET Fun_Gprs_Ftp_disconnect(void);
//GPRS获取文件大小,目录用‘/’隔开
uint32_t Fun_Gprs_Ftp_GetFileSize(char *filePach,char* fileName);
//GPRS获取文件内容
uint32_t Fun_Gprs_Ftp_ReadFile(char* fileName,uint32_t filePos,uint32_t getSize,uint8_t* buff);
//HTTP上传文件启动，返回值0：启动失败 1：启动成功
uint8_t Fun_Gprs_Http_Post_File_Start(char *url,char *username,char *password,char*fileName,uint32_t fileSize);
//POST上传文件数据输入，返回值 0：结束 大于0：输入长度
uint32_t Fun_Gprs_Http_Post_File_Input(void* buff,uint32_t len);
#endif

