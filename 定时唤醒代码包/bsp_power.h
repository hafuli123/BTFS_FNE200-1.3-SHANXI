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

void bsp_sleep_enable(BSP_SLEEP_FLAG_E sleep_flag);																																	//˯��ʹ��
bsp_errcode_sleep bsp_enter_sleep(void);																																						//����˯��
bsp_errcode_sleep bsp_get_powerup_reason(uint8_t *pwrup_reason);																										//��ȡ����ԭ��
uint8_t bsp_sleep_state(void);																																											//��ȡ˯��״̬
bsp_errcode_sleep bsp_sleep_wakeup_volt(float voltLowVal);																													//�͵�ѹ����

int8_t bsp_wakelock_create(void);																																										//����������
bsp_errcode_sleep bsp_wakelock_delete(int8_t wakelock_fd);																													//������ɾ��
bsp_errcode_sleep bsp_wakelock_lock(int8_t wakelock_fd);																														//����������
bsp_errcode_sleep bsp_wakelock_unlock(int8_t wakelock_fd);																													//����������
bsp_errcode_sleep bsp_wakelock_para(int8_t wakelock_fd,uint32_t timestamp,uint32_t interval,uint8_t (*callback)(uint8_t type),int8_t socket_fd,uint8_t keep_socket_flag);	//���Ѳ�����ʱ��������ڣ��ص�����(����ֵ0:����˯�� 1:������ ����0:��ʱ���� 1:IO����)��socket_fd

#endif

