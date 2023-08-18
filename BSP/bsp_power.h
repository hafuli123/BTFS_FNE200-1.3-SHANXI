#ifndef __BSP_POWER_H
#define __BSP_POWER_H

#include "stdint.h"

#define BSP_WAKELOCK_CNT	10

typedef enum
{
	BSP_SLEEP_SUCCESS,
	BSP_SLEEP_INVALID_PARAM,   			/*  invalid input param  	   		 */
	BSP_SLEEP_LOCK_CREATE_FAIL,   		/*  failed to create wake lock  		 */
	BSP_SLEEP_LOCK_DELETE_FAIL,   		/*  failed to delete wake lock		 */
	BSP_SLEEP_LOCK_LOCK_FAIL,   			/*  failed to lock the wake lock 	 */
	BSP_SLEEP_LOCK_UNLOCK_FAIL,   		/*  failed to unlock the wake lock 	 */
	BSP_SLEEP_LOCK_AUTOSLEEP_FAIL,   /*  failed to enter auto sleep	 */
	BSP_SLEEP_PARAM_SAVE_FAIL,   		/*  failed to save sleep param	 */
	BSP_SLEEP_EXECUTE_FAIL,   				/*  failed to execute	 */
}bsp_errcode_sleep;

/**
 * Enable auto sleep
 */
typedef enum 
{
	BSP_NOT_ALLOW_SLEEP = 0,
	BSP_ALLOW_SLEEP,
}BSP_SLEEP_FLAG_E;

/**
 * Power up reason
 */
typedef enum
{
    BSP_PWRUP_UNKNOWN,         // unknown reason
    BSP_PWRUP_PWRKEY,          // power up by power key
    BSP_PWRUP_PIN_RESET,       // power up by pin reset
    BSP_PWRUP_ALARM,           // power up by alarm
    BSP_PWRUP_CHARGE,          // power up by charge in
    BSP_PWRUP_WDG,             // power up by watchdog
    BSP_PWRUP_PSM_WAKEUP,      // power up from PSM wakeup
    BSP_PWRUP_PANIC            // power up by panic reset
}bsp_pwrup_reason;

void bsp_sleep_enable(BSP_SLEEP_FLAG_E sleep_flag);																																	//睡眠使能
bsp_errcode_sleep bsp_enter_sleep(void);																																						//进入睡眠
bsp_errcode_sleep bsp_get_powerup_reason(uint8_t *pwrup_reason);																										//获取唤醒原因
uint8_t bsp_sleep_state(void);																																											//获取睡眠状态
bsp_errcode_sleep bsp_sleep_wakeup_volt(float voltLowVal);																													//低电压唤醒

int8_t bsp_wakelock_create(void);																																										//唤醒锁创建
bsp_errcode_sleep bsp_wakelock_delete(int8_t wakelock_fd);																													//唤醒锁删除
bsp_errcode_sleep bsp_wakelock_lock(int8_t wakelock_fd);																														//唤醒锁激活
bsp_errcode_sleep bsp_wakelock_unlock(int8_t wakelock_fd);																													//唤醒锁解锁
bsp_errcode_sleep bsp_wakelock_para(int8_t wakelock_fd,uint32_t timestamp,uint32_t interval,uint8_t (*callback)(uint8_t type),int8_t socket_fd,uint8_t keep_socket_flag);	//唤醒参数，时间戳，周期，回调函数(返回值0:继续睡眠 1:允许唤醒 参数0:定时唤醒 1:IO唤醒)，socket_fd

#endif

