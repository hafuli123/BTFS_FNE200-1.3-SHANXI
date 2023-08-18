#ifndef __FUN_MON_H
#define __FUN_MON_H

#include "stdint.h"

extern void funMonInit(void);
extern void funMonrun(void);
/*
回调函数参数说明
type:0-正在升级，parameter：uint8_t* 升级进度
type:1-升级成功，parameter：NULL
type:2-升级失败，parameter：待定义
*/
extern void funMonSetFtpParas(char *hostname,uint16_t hostport,char *username,char *password,char *filePach,char* fileName,void (*callback)(uint8_t type,void* parameter));

extern uint8_t funGet_ACCState(void);					//获取ACC连接状态
extern uint8_t funGet_ChgState(void);					//获取充电连接状态



#endif
