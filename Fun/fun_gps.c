#include "fun_gps.h"
#include "cmsis_os2.h"
#include "string.h"
#include "bsp_gps.h"
#include "bsp_uart_fifo.h"
# include <stdio.h>

/* 
	RTKģ�鴫�ص�NMEA����У���һ���������£�GGA��RMC����һ�����ѣ�
	����ȡ������Ŀ��GSV��䣬��Ŀ���������ܴ��ںܶ����������
	RTK��������Ƶ�ʲ����˴��Ϊ1�봫10��15��֮��
	
*/
void funGpsrun(void)
{
	rtk_analysis();
	
}
