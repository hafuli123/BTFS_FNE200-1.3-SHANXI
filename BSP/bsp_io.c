#include "bsp_io.h"
#include "stm32f4xx_gpio.h"

#include "cmsis_os2.h"

static uint8_t boarderVer = 0;		//0：新版 1：旧版兼容

/* IO口初始化 
   输入：无
   输出：无
*/
void BSP_IoInit(void)
{
	uint32_t StartUpCounter = 0;
  GPIO_InitTypeDef gpio_init;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
	#if(1)
	DBGMCU_Config(DBGMCU_STOP,ENABLE);
	#endif
	//MCU烧录口权限、调试口权限控制，设置为模拟量输入
	#if(0)
	gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_OType = GPIO_OType_OD;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_init.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
	GPIO_Init(GPIOA, &gpio_init);	
	#endif
	//涉及最高保护级别，无法下载程序，无法调试，不可逆
	#if(0)
	if(FLASH_OB_GetRDP() != SET)
	{
		FLASH_Unlock();
		FLASH_OB_Unlock();
		FLASH_OB_RDPConfig(OB_RDP_Level_2);
		FLASH_OB_Launch();
		FLASH_OB_Lock();
		FLASH_Lock();
	}
	#endif
	
	//上拉 输入
	gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_IN;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	
	/* GPIOA*/
	gpio_init.GPIO_Pin  =  GPIO_Pin_11 ;
  GPIO_Init(GPIOA, &gpio_init);	

	/* GPIOB */
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_init.GPIO_Pin  = GPIO_Pin_4;
  GPIO_Init(GPIOB, &gpio_init);	
	
	/* GPIOC*/
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_init.GPIO_Pin  =  GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
  GPIO_Init(GPIOC, &gpio_init);
	
  gpio_init.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOC, &gpio_init);
	
	//新版旧版检测
	gpio_init.GPIO_Mode  = GPIO_Mode_OUT;
	gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
	
	/*	兼容旧版终端
	osDelay(400);
//  do
//  {
//    StartUpCounter++;
//  }while(boarderVer == 0 && StartUpCounter <= 0x1000);
	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) || GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) || GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3))
	{
		boarderVer = 1;
	}
	*/	
	//上拉输出
  gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
  gpio_init.GPIO_Mode  = GPIO_Mode_OUT;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
  /* GPIOA*/
  gpio_init.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_8;
  GPIO_Init(GPIOA, &gpio_init);
  /* GPIOB */
  gpio_init.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_7;
  GPIO_Init(GPIOB, &gpio_init);
  /* GPIOC */
  gpio_init.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_7|GPIO_Pin_13;
  GPIO_Init(GPIOC, &gpio_init);
	
	/* 外部中断唤醒 */
	gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_IN;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_init.GPIO_Pin  =  GPIO_Pin_0;
  GPIO_Init(GPIOA, &gpio_init);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_EXTIT,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);		/* 连接 EXTI 中断源到CAN终端检测引脚 */
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;											/* 选择 EXTI 中断源 */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;							/* 中断模式 */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;					/* 下降沿触发 */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;												/* 使能中断/事件线 */
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;								/* 配置中断源：PA0 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;							/* 配置子优先级：1 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;									/* 使能中断通道 */
	NVIC_Init(&NVIC_InitStructure);
	
  //开启电源、睡眠禁能
	BSP_Ioctl(PWR_3V3_ID,ON);
	BSP_Ioctl(PWR_MCU_IO,ON);
	BSP_Ioctl(PWR_GPRS_IO,ON);
	BSP_Ioctl(PWR_VCC_ID,ON);
	BSP_Ioctl(CAN_STB_ID,ON);
	BSP_Ioctl(GPRS_DTR_ID,ON);
	//关闭指示
	BSP_Ioctl(ST_BEEP_ID,OFF);
	BSP_Ioctl(ST_DATA_ID,OFF);
	BSP_Ioctl(ST_GPRS_ID,OFF);
	BSP_Ioctl(ST_GPS_ID,OFF);
	BSP_Ioctl(ST_CAN_ID,OFF);
	//兼容旧版
	if(boarderVer == 1)
	{
	//上拉输出
		gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
		gpio_init.GPIO_Mode  = GPIO_Mode_OUT;
		gpio_init.GPIO_OType = GPIO_OType_PP;
		gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
		//旧版4G-RESET
		gpio_init.GPIO_Pin = GPIO_Pin_0;
		GPIO_Init(GPIOA, &gpio_init);
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);
		//旧版CAN_EN
		gpio_init.GPIO_Pin = GPIO_Pin_1;
		GPIO_Init(GPIOB, &gpio_init);
		GPIO_SetBits(GPIOB, GPIO_Pin_1);
		//旧版PWR_CLEAN,同蜂鸣器一起处理
		gpio_init.GPIO_Pin = GPIO_Pin_13;
		GPIO_Init(GPIOB, &gpio_init);
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	}
}
/* ADC _IO口初始化 
   输入：无
   输出：无
*/
void BSP_ADC_IoInit(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
  GPIO_InitTypeDef gpio_init;

  //ADC
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
  gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_5;
  gpio_init.GPIO_Mode = GPIO_Mode_AN;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOC, &gpio_init);
	
  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);

}

uint8_t BSP_GetVer(void)
{
	return boarderVer;
}

void  BSP_IoDeInit (void)
{
	GPIO_InitTypeDef  gpio_init;
	//IO设置为模拟量输入
	gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_OType = GPIO_OType_OD;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	gpio_init.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12/*|GPIO_Pin_13|GPIO_Pin_14*/|GPIO_Pin_15;
	GPIO_Init(GPIOA, &gpio_init);
	//旧版关闭4G相关IO
	if(boarderVer == 1)
	{
		gpio_init.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
		GPIO_Init(GPIOA, &gpio_init);
	}
	
	gpio_init.GPIO_Pin =  GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4/*|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9*/|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	
	GPIO_Init(GPIOB, &gpio_init);
	//新版关闭CAN睡眠控制引脚
	if(boarderVer == 0)
	{
		gpio_init.GPIO_Pin = GPIO_Pin_0;
		GPIO_Init(GPIOB, &gpio_init);
	}
	
	gpio_init.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7/*|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12*/;
	GPIO_Init(GPIOC, &gpio_init);
	
//	gpio_init.GPIO_Pin = GPIO_Pin_2;
//	GPIO_Init(GPIOD, &gpio_init);	
	
		
	USART_Cmd(USART3, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE);
	USART_Cmd(USART1, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
	
	SPI_Cmd(SPI1, DISABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, DISABLE);
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, DISABLE);
}

/*
ADC _IO 反初始化
*/
void  BSP_ADC_IoDeInit (void)
{
	ADC_Cmd(ADC1, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
}

void BSP_Ioctl(uint8_t id,uint8_t toggle)
{
	switch(id)
	{
		case PWR_GPRS_IO:
			if(toggle == ON)
				GPIO_SetBits(GPIOA, GPIO_Pin_1);
			else
				GPIO_ResetBits(GPIOA, GPIO_Pin_1);	
			break;
		case PWR_BAT_ID:
			if(toggle == ON)
				GPIO_SetBits(GPIOA, GPIO_Pin_8);
			else
				GPIO_ResetBits(GPIOA, GPIO_Pin_8);	
			break;
		case CAN_STB_ID:
			if(toggle == ON)
				GPIO_SetBits(GPIOB, GPIO_Pin_0);
			else
				GPIO_ResetBits(GPIOB,GPIO_Pin_0);
			break;
		case PWR_VCC_ID:
			//旧版不能操作
			if(boarderVer == 0)
			{
				if(toggle == OFF)
					GPIO_SetBits(GPIOB, GPIO_Pin_1);
				else
					GPIO_ResetBits(GPIOB, GPIO_Pin_1);
			}
			break;
		case ST_BEEP_ID:
			//旧版和蜂鸣器一起清除PWR_CLEAN
			if(boarderVer == 1)
			{
				if(toggle == ON)
					GPIO_SetBits(GPIOC, GPIO_Pin_13);
				else
					GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			}
			if(toggle == ON)
				GPIO_SetBits(GPIOB, GPIO_Pin_3);
			else
				GPIO_ResetBits(GPIOB, GPIO_Pin_3);
			break;
		case PWR_MCU_IO:
			if(toggle == ON)
				GPIO_SetBits(GPIOB, GPIO_Pin_7);
			else
				GPIO_ResetBits(GPIOB, GPIO_Pin_7);
			break;
		case ST_GPRS_ID:
			if(toggle == OFF)
				GPIO_SetBits(GPIOC, GPIO_Pin_1);
			else
				GPIO_ResetBits(GPIOC, GPIO_Pin_1);
			break;
		case ST_GPS_ID:
			if(toggle == OFF)
				GPIO_SetBits(GPIOC, GPIO_Pin_2);
			else
				GPIO_ResetBits(GPIOC, GPIO_Pin_2);
			break;
		case ST_CAN_ID:
			if(toggle == OFF)
				GPIO_SetBits(GPIOC, GPIO_Pin_3);
			else
				GPIO_ResetBits(GPIOC, GPIO_Pin_3);
			break;
		case PWR_3V3_ID:
			if(toggle == ON)
				GPIO_SetBits(GPIOC, GPIO_Pin_7);
			else
				GPIO_ResetBits(GPIOC, GPIO_Pin_7);			
			break;
		case GPRS_DTR_ID:
			//旧版不能操作
			if(boarderVer == 0)
			{
				if(toggle == ON)
					GPIO_SetBits(GPIOC, GPIO_Pin_13);
				else
					GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			}
			break;
		case AUTO_CHG_ID:			//自动补电
			{
				if(toggle == ON)
					GPIO_SetBits(GPIOC, GPIO_Pin_6);
				else
					GPIO_ResetBits(GPIOC, GPIO_Pin_6);
			}
			break;
	}
}

/* IO读
   输入：id
   输出：1->按下，0->无触发
 */
uint8_t BSP_Iostatus (uint8_t pb)
{
	uint8_t status = OFF;
	switch (pb)
	{
		/* ACC输入 */
		case IN_ACC_ID:
			if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4))
			{
				status = ON;
			}
			break;
		case IN_WAKEUP_ID:
			if(boarderVer == 0)
			{
				if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
				{
					status = ON;
				}
			}
			else
			{
				if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) || !GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8))
				{
					status = ON;
				}
			}
			break;
		case IN_USB_ID:
			if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11))
			{
				status = ON;
			}
			break;
	}
	return (status);
}

void ledFlash(uint8_t id,uint16_t intervel)
{
	BSP_Ioctl(id,ON);
	osDelay(intervel);
	BSP_Ioctl(id,OFF);
	osDelay(intervel);
}

/*********************************************************************************************************
*	函 数 名: whistle
*	功能说明: 鸣笛控制
*	形    参: time:鸣笛时间，intervel：鸣笛间隔
*	返 回 值: 
*********************************************************************************************************
*/
void whistle(uint16_t time,uint16_t intervel)
{
	BSP_Ioctl(ST_BEEP_ID,ON);
	osDelay(time);
	BSP_Ioctl(ST_BEEP_ID,OFF);
	osDelay(intervel);
}

static uint8_t WakeUpSeq;
/*
	EXTI0 中断服务函数
*/
void EXTI0_IRQHandler(void)
{
	WakeUpSeq++;
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) 
	{
		//清除中断标志位
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

uint8_t BSP_WakeUpSeq(void)
{
	return WakeUpSeq;
}

float BSP_Get_ADC_Value(uint8_t id)
{
	const uint16_t cnt = 200;
	uint32_t temp_val = 0;
	uint16_t t;
	switch (id)
	{
		case ADC_VCC_ID:
			ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_480Cycles);
			ADC_SoftwareStartConv(ADC1);
			for(t = 0; t < cnt;t++)
			{
				while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
				temp_val += ADC_GetConversionValue(ADC1);
			}
			return (temp_val / cnt* 3.3 / 4096* 11) * 1.003 + 0.537;	
		case ADC_BAT_ID:
			ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
			ADC_SoftwareStartConv(ADC1);
			for(t = 0; t < cnt;t++)
			{
				while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
				temp_val += ADC_GetConversionValue(ADC1);
			}
			return temp_val / cnt * 6 * 3.3 / 4096;	
	}
	return 0;
}
