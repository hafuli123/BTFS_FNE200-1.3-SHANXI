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

//�Զ��� 80����
typedef struct _selfData80{
    float  InWaterTem;       				//ˮ���¶�
	float   OutWaterTem;     				//ˮ���¶�
    float  AirComVol;    				//����ѹ������ѹ
    float  AirComCur;	 				//����ѹ��������
    float   HyCyclePumpVol;  			//����ѭ���õ�ѹ
    float   HyCyclePumpCur;  			//����ѭ���õ���
    float  HySurplus;             			//����ʣ����
    uint8_t   AirConControlCommand;     //�յ�����ָ��
    uint8_t   WarmRiskControlCommand;              //ů�����ָ��
}SelfData80;

//�Զ��� 81���� ��������
typedef struct _selfData81{
    float  AirComVol;    				//����ѹ������ѹ
    float  AirComPow;	 				//����ѹ��������
    float   HyCyclePumpVol;  			//����ѭ���õ�ѹ
    float   HyCyclePumpCur;  			//����ѭ���õ���
    float  InWaterTem;       				//ˮ���¶�
		float   OutWaterTem;     				//ˮ���¶�
    float  HySurplus;             			//����ʣ����	
}SelfData81;


extern SelfData80* pSelfData80;					
extern SelfData81* pSelfData81;



#endif
