#ifndef __SHWX_GBEX_H
#define __SHWX_GBEX_H

#include "cmsis_os2.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stdint.h"
#include "fun_can.h"
#include "Fun_Net.h"
#include "bsp_rtc.h"
#include "bsp_ec20.h"
#include "algo_verify.h"
#include "protocol_GB.h"

#include "fun_mon.h"
#include "bsp_gps.h"
#include "bsp_storage.h"
#include "bsp_io.h"
#include "bsp_sys.h"
#include "bsp_power.h"
#include "protocol_GB32960.h"
#include "bsp_storage.h"
#include "rl_fs.h"
#include "bsp_uart_fifo.h"

//自定义 80数据
typedef struct _selfData80{
    float  InWaterTem;       				//水入温度
	float   OutWaterTem;     				//水出温度
    float  AirComVol;    				//空气压缩机电压
    float  AirComCur;	 				//空气压缩机电流
    float   HyCyclePumpVol;  			//氢气循环泵电压
    float   HyCyclePumpCur;  			//氢气循环泵电流
    float  HySurplus;             			//氢气剩余量
    uint8_t   AirConControlCommand;     //空调控制指令
    uint8_t   WarmRiskControlCommand;              //暖风控制指令
}SelfData80;

//自定义 81数据 整车数据
typedef struct _selfData81{
    float  AirComVol;    				//空气压缩机电压
    float  AirComPow;	 				//空气压缩机功率
    float   HyCyclePumpVol;  			//氢气循环泵电压
    float   HyCyclePumpCur;  			//氢气循环泵电流
    float  InWaterTem;       				//水入温度
		float   OutWaterTem;     				//水出温度
    float  HySurplus;             			//氢气剩余量	
}SelfData81;


extern SelfData80* pSelfData80;					
extern SelfData81* pSelfData81;



#endif
