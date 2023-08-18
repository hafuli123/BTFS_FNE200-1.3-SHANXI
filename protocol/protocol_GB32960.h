#ifndef __PROTOCOL_GB32960_H__ 
#define __PROTOCOL_GB32960_H__ 

#include "pdu.h"

#define EV1_SERVER 	0							//新能源企标链路,
#define EV2_SERVER 	1							//新能源点播链路
#define EV3_SERVER  2							//新能源选用链路

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

//返回示例 参数：链路号，域名，端口，实时周期，心跳周期
void* gb32960Init(uint8_t link,uint8_t* pBuff,uint16_t pBuffSize,char*domain,uint32_t* port,char* vin,uint8_t realHz,uint8_t heartbeatHz);
uint8_t gb32960Run(void* obj);
uint8_t gb32960GetSta(uint8_t id);
uint8_t gb32960Ctrl(uint8_t ctrl);
void cleanPara(void* obj);							//清除链路参数

//扩展协议初始化 prot协议号，如需要多协议
void* extInit(uint8_t link,char* vin,uint8_t prot,uint8_t* buff,uint16_t buffLen);
//扩展协议运行   ctrl:0 网络离线 1:网络在线 2:睡眠 sta:0 国标未登入 1:国标已登入 返回值:0 扩展协议未登入 1:扩展协议已登入
uint8_t extRun(void* obj,uint8_t ctrl,uint8_t sta);
//扩展协议解包程序
uint8_t extUnpack(uint8_t link,const uint8_t *szRecvBuf,uint16_t rLen);
//扩展实时数据，实时数据增加自定义数据
uint16_t extReal(void* obj,RealData *pRealData,uint8_t *bPackBuf, uint16_t RemainLen);
//扩展车架号识别方法，例如试用终端编号
uint8_t extGetVin(uint8_t cmd,uint8_t rsp,char * vin,uint8_t *resvBuff);
//企标连接平台控制
uint8_t extCtrlNetSta(void* obj);																
//获取数据缓存
RealData* getRealCache(uint8_t dataIdx,uint8_t* outDataIdx);
#endif
