/*
 * This file is part of the EasyLogger Library.
 *
 * Copyright (c) 2015, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-04-28
 */
 
#include <elog.h>
#include "bsp_sys.h"
#include "string.h"
#include "stdio.h"
#include "bsp_uart_fifo.h"

/**
 * EasyLogger port initialize							初始化
 *
 * @return result
 */
ElogErrCode elog_port_init(void) {
    ElogErrCode result = ELOG_NO_ERR;
//    ELOG_FMT_LVL    = 1 << 0, /**< level */
//    ELOG_FMT_TAG    = 1 << 1, /**< tag */
//    ELOG_FMT_TIME   = 1 << 2, /**< current time */
//    ELOG_FMT_P_INFO = 1 << 3, /**< process info */
//    ELOG_FMT_T_INFO = 1 << 4, /**< thread info */
//    ELOG_FMT_DIR    = 1 << 5, /**< file directory and name */
//    ELOG_FMT_FUNC   = 1 << 6, /**< function name */
//    ELOG_FMT_LINE   = 1 << 7, /**< line number */
    /* add your code here */
		elog_set_fmt(ELOG_LVL_ASSERT, ELOG_FMT_ALL);
	
		elog_set_fmt(ELOG_LVL_ERROR, ELOG_FMT_ALL);
		elog_set_fmt(ELOG_LVL_WARN, ELOG_FMT_ALL);
		elog_set_fmt(ELOG_LVL_INFO, ELOG_FMT_ALL);
		elog_set_fmt(ELOG_LVL_DEBUG, ELOG_FMT_ALL);
	
		elog_set_fmt(ELOG_LVL_VERBOSE, ELOG_FMT_ALL);    
    return result;
}

/**
 * EasyLogger port deinitialize						取消初始化
 *
 */
void elog_port_deinit(void) {

    /* add your code here */

}

/**
 * output log port interface							输出至串口
 *
 * @param log output of log
 * @param size log size
 */
void elog_port_output(const char *log, size_t size) {
    
    /* add your code here */
   comSendBuf(RS1_COM,(uint8_t*)log,size);
}

/**
 * output lock														输出上锁
 */
void elog_port_output_lock(void) {
    
    /* add your code here */
    
}

/**
 * output unlock													输出解锁
 */
void elog_port_output_unlock(void) {
    
    /* add your code here */
    
}

/**
 * get current time interface							获取当前时间
 *
 * @return current time
 */
const char *elog_port_get_time(void) {
    
    /* add your code here */
		static char cur_system_time[24] = {0};
		sprintf(cur_system_time,"%d-%02d-%02d %02d:%02d:%02d",2022,1,1,0,0,0);
		return cur_system_time;
}

/**
 * get current process name interface			获取当前进程名
 *
 * @return current process name
 */
const char *elog_port_get_p_info(void) {
    /* add your code here */
		return "";
}

/**
 * get current thread name interface			获取当前线程号
 *
 * @return current thread name
 */
const char *elog_port_get_t_info(void) {
    
    /* add your code here */
		const char* name = NULL;
		osThreadId_t tempThreadID = osThreadGetId();
		if(tempThreadID != NULL)
		{
			name = osThreadGetName( osThreadGetId());
		}
		if(name != NULL)
		{
			return name;
		}
		return "";
}
