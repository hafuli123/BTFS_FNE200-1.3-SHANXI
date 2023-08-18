#include "fun_gps.h"
#include "cmsis_os2.h"
#include "string.h"
#include "bsp_gps.h"
#include "bsp_uart_fifo.h"
# include <stdio.h>

/* 
	RTK模块传回的NMEA语句中，在一轮语句情况下，GGA和RMC都是一条而已；
	但获取卫星数目的GSV语句，数目不定，可能存在很多条的情况；
	RTK发送语句的频率测试了大概为1秒传10到15条之间
	
*/
void funGpsrun(void)
{
	rtk_analysis();
	
}
