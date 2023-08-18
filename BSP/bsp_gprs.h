#ifndef __BSP_GPRS_H
#define __BSP_GPRS_H

#include "pdu.h"

#define MAX_LINK 6

typedef uint8_t (*tcpUnpackDataCallBack)(uint8_t link,const uint8_t *pbuf, uint16_t rLen);

#define MAX_LEN_FTP_URL 50        //FTP的URL最大长度
#define MAX_LEN_FTP_USER 20       //FTP的用户名最大长度
#define MAX_LEN_FTP_PWD 20        //FTP的密码最大长度
#define MAX_LEN_FTP_DIR 100				//FTP目录最大长度
#define MAX_LEN_FTP_NAME 40				//FTP文件名最大长度

typedef struct Ftp_Pars
{
	char ftpUrl[MAX_LEN_FTP_URL];		//FTP服务器地址
	char ftpUser[MAX_LEN_FTP_USER];	//FTP用户名
	char ftpPwd[MAX_LEN_FTP_PWD];		//FTP密码
	uint16_t ftpPort;								//FTP服务器端口
	char ftpName[MAX_LEN_FTP_NAME];	//FTP文件名
	char ftpDir[MAX_LEN_FTP_DIR];		//FTP文件夹
	uint8_t parts;									//升级零部件
}FTP_PARS;

extern FTP_PARS gFtpParas;

typedef struct
{
	uint8_t used;
	uint8_t linkState;
	char *address;
	uint16_t port;
	uint16_t connectCnt;	//重连计时
	uint16_t connectHZ;
	uint8_t ack;
	uint8_t nAckTm;
	tcpUnpackDataCallBack UnpackData;
}TCP_CLIENT_PARA;

extern uint8_t gftp_Mode;        /* FTP状态 */

/* SIM模块初始化 */
void bsp_InitGprs(void);
/* 复位模块,自动恢复 */
void simReset(void);
/* SIM卡状态 */
uint8_t simGetSIMCardStatus(void);
/* GPRS拨号，并自动连接目标服务器 */
uint8_t simPppdCallGPRS(void);
/* 查询TCP/IP连接情况,此函数需要一直调用,如果TCP连接正常,才可以向服务器发送数据包 */
void simQueryConnectionStatus(void);
/*查询链路状态*/
uint8_t simQueryLinkStatus(uint8_t link);
/*查询网络信号强度*/
uint8_t simGetSignalQuality (void);
/* SIM800模块ftp升级 */
uint8_t simFtpUpdate(void);
/* GPRS发送数据 */
uint8_t simSendData(uint8_t linkIndex,uint8_t* wbuf,uint16_t len);
/* 设置连接参数 */
uint8_t simSetUnpackdataFun(uint8_t link,char *address,uint16_t port,tcpUnpackDataCallBack UnpackDataFunc,uint8_t used);
/* 获取ICCID */
uint8_t simGetCCID(uint8_t* rbuf, uint8_t rLen);
/* 关闭TCP连接 */
void simIPClose(void);
/*关闭TCP链路*/
void simLinkClose(uint8_t link);
/*TCP链路断开重连*/
void reConnect(uint8_t link);
/* 创建TCP连接 */
void simTcpConnect(uint8_t linkIndex,char *domain,uint16_t port);
/*获取网络注册状态*/
uint8_t simGetGPRSNetStatus(void);
/*拨号任务*/
void gprsPppdask(void);
/*重播*/
void gprsRedial(void);
/*FTP升级处理*/
void ftpUpdateApp(void);
uint8_t getNAckTm(uint8_t link);
uint8_t simOpenSleepModel(void);				//开启睡眠模式

extern uint8_t gprs_State;     	        				/* GPRS状态指示灯 */
extern uint8_t gFtp_Mode;               				/* FTP状态 1-FTP升级 2-数据导出 3-SD卡升级 */

#endif

