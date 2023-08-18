#include "bsp_uart_fifo.h"
#include "stm32f4xx_usart.h"
#include "string.h"
#include "cmsis_os2.h" 
#include "stdio.h"

extern uint32_t apple;
extern osSemaphoreId_t sem_gps_id;

/* 串口设备结构体 */
typedef struct
{
	USART_TypeDef *uart;		/* STM32内部串口设备指针 */
	uint8_t *pTxBuf;			/* 发送缓冲区 */
	uint8_t *pRxBuf;			/* 接收缓冲区 */
	uint16_t usTxBufSize;		/* 发送缓冲区大小 */
	uint16_t usRxBufSize;		/* 接收缓冲区大小 */
	uint16_t usTxWrite;			/* 发送缓冲区写指针 */
	uint16_t usTxRead;			/* 发送缓冲区读指针 */
	uint16_t usTxCount;			/* 等待发送的数据个数 */

	uint16_t usRxWrite;			/* 接收缓冲区写指针 */
	uint16_t usRxRead;			/* 接收缓冲区读指针 */
	uint16_t usRxCount;			/* 还未读取的新数据个数 */

	void (*SendBefor)(void); 	/* 开始发送之前的回调函数指针（主要用于RS485切换到发送模式） */
	void (*SendOver)(void); 	/* 发送完毕的回调函数指针（主要用于RS485将发送模式切换为接收模式） */
	void (*ReciveNew)(uint8_t);	/* 串口收到数据的回调函数指针 */
}UART_T;

/* 定义每个串口结构体变量 */
#if UART1_FIFO_EN == 1
	static UART_T g_tUart1;
	static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART2_FIFO_EN == 1
	static UART_T g_tUart2;
	static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART3_FIFO_EN == 1
	static UART_T g_tUart3;
	static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE];		/* 接收缓冲区 */

#endif

volatile uint8_t u8_rxBuf[RX_MAX_LEN];	/* 串口接收缓冲区 */

volatile uint8_t u8_rxBuf_gga[RX_MAX_LEN];
volatile uint8_t u8_rxBuf_rmc[RX_MAX_LEN];
volatile uint8_t u8_rxBuf_gsv[RX_MAX_LEN];
volatile uint8_t u8_rxBufTyp;						/* 目前在接收的这条NMEA语句的类型 */
volatile uint16_t u16_bufLen_gga;
volatile uint16_t u16_bufLen_rmc;
volatile uint16_t u16_bufLen_gsv;

volatile uint8_t u8_rxBit;							/* 标记串口接收数组的元素号 */
volatile uint16_t u16_bufLen;						/* 串口接收到语句的长度 */
volatile uint8_t u8_rxFr;								/* 当接收到rtk模块'$'的NMEA语句开头字符，置为1；当接收到/n结束字符，置回0 */

volatile uint8_t u8_vuNum[9];						/* GSV获取到的卫星数量数据 */



static void UartVarInit(void);
static void InitHardUart(void);
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte);
static void UartIRQ(UART_T *_pUart);
static void ConfigUartNVIC(void);

/*
*********************************************************************************************************
*	函 数 名: bsp_InitUart
*	功能说明: 初始化串口硬件，并对全局变量赋初值.
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitUart(void)
{
	UartVarInit();		/* 必须先初始化全局变量,再配置硬件 */

	InitUsart3Baud();	/*初始化串口3波特率*/
	
	InitHardUart();		/* 配置串口的硬件参数(波特率等) */

	ConfigUartNVIC();	/* 配置串口中断 */
}

/*
*********************************************************************************************************
*	函 数 名: ComToUart
*	功能说明: 将COM端口号转换为UART指针
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: uart指针
*********************************************************************************************************
*/
UART_T *ComToUart(COM_PORT _ucPort)
{
	if (_ucPort == RS1_COM)
	{
		#if UART1_FIFO_EN == 1
			return &g_tUart1;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == RS2_COM)
	{
		#if UART2_FIFO_EN == 1
			return &g_tUart2;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == RS3_COM)
	{
		#if UART3_FIFO_EN == 1
			return &g_tUart3;
		#else
			return 0;
		#endif
	}
	else
	{
		/* 不做任何处理 */
		return 0;
	}
}

/*
*********************************************************************************************************
*	函 数 名: comSendBuf
*	功能说明: 向串口发送一组数据。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*	形    参: _ucPort: 端口号(COM1 - COM6)
*			  _ucaBuf: 待发送的数据缓冲区
*			  _usLen : 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void comSendBuf(COM_PORT _ucPort, uint8_t *_ucaBuf, uint16_t _usLen)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}
	while(_usLen)
    {
        USART_SendData(pUart->uart, *_ucaBuf++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(pUart->uart, USART_FLAG_TXE) == RESET);
        _usLen--;
    }
}

/*
*********************************************************************************************************
*	函 数 名: comSendChar
*	功能说明: 向串口发送1个字节。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*	形    参: _ucPort: 端口号(COM1 - COM6)
*			  _ucByte: 待发送的数据
*	返 回 值: 无
*********************************************************************************************************
*/
void comSendChar(COM_PORT _ucPort, uint8_t _ucByte)
{
	comSendBuf(_ucPort, &_ucByte, 1); //向串口中断发送一组数据。
}


//打印数据
void printfData(const char *_ch)
{
	uint8_t len = strlen(_ch);
	if(len<200)
		comSendBuf(RS1_COM,(uint8_t*)_ch,len);
}

/*
*********************************************************************************************************
*	函 数 名: comGetChar
*	功能说明: 从串口缓冲区读取1字节，非阻塞。无论有无数据均立即返回
*	形    参: _ucPort: 端口号(COM1 - COM6)
*			  _pByte: 接收到的数据存放在这个地址
*	返 回 值: 0 表示无数据, 1 表示读取到有效字节
*********************************************************************************************************
*/
uint16_t comGetBufDelay(COM_PORT _ucPort, uint16_t bufMaxLen,uint8_t *_pByte,uint16_t _usTimeOut)
{
	uint16_t retLen = 0;
	uint8_t ucData;
	uint16_t timeOutCnt = _usTimeOut;
	UART_T *pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return 0;
	}
	while (1)
	{
		if(UartGetChar(pUart,&ucData))
		{
			timeOutCnt = UART_CHAT_TIME;
			_pByte[retLen++] = ucData;
			if (retLen >= bufMaxLen)
			{
				break;
			}
		}
		else
		{
			timeOutCnt--;
			if(timeOutCnt == 0)
			{
				break;
			}
			osDelay(1);
		}
	}
	return retLen;
}

/*
*********************************************************************************************************
*	函 数 名: comChkBufDelay
*	功能说明: 从串口缓冲区检查数据
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: 0 表示无数据, 1 表示读取到有效字节
*********************************************************************************************************
*/
uint8_t comChkBufDelay(COM_PORT _ucPort,char *ack,uint16_t _usTimeOut)
{
	uint16_t pos;
	uint8_t i;
	uint16_t timeOutCnt = _usTimeOut;
	uint16_t StatComBufPos;								//开始获取缓冲区数据的位置指针
	uint16_t EndComBufPos;								//结束获取缓冲区数据的位置指针
	UART_T *pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return 0;
	}
	StatComBufPos  = pUart->usRxWrite;
	EndComBufPos = pUart->usRxWrite;
	while (1)
	{
		if(EndComBufPos  != pUart->usRxWrite)
		{
			i = 0;
			pos = StatComBufPos;
			EndComBufPos = pUart->usRxWrite;
			while(pos != EndComBufPos)
			{
				if(pUart->pRxBuf[pos++] == ack[i++])
				{
					if(i >= strlen(ack))
					{
						return 1;
					}
				}
				else
				{
					i = 0;
				}
				pos %= pUart->usRxBufSize;
			}
		}
		else
		{
			if(timeOutCnt == 0)
			{
				break;
			}
			timeOutCnt--;
			osDelay(1);
		}
	}
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: comGetChar
*	功能说明: 从串口缓冲区读取1字节，非阻塞。无论有无数据均立即返回
*	形    参: _ucPort: 端口号(COM1 - COM6)
*			  _pByte: 接收到的数据存放在这个地址
*	返 回 值: 0 表示无数据, 1 表示读取到有效字节
*********************************************************************************************************
*/
uint8_t comGetChar(COM_PORT _ucPort, uint8_t *_pByte)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return 0;
	}

	return UartGetChar(pUart, _pByte);
}

/*
*********************************************************************************************************
*	函 数 名: comClearTxFifo
*	功能说明: 清零串口发送缓冲区
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: 无
*********************************************************************************************************
*/
void comClearTxFifo(COM_PORT _ucPort)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}
	pUart->usTxWrite = 0;
	pUart->usTxRead = 0;
	pUart->usTxCount = 0;
}

/*
*********************************************************************************************************
*	函 数 名: comClearRxFifo
*	功能说明: 清零串口接收缓冲区
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: 无
*********************************************************************************************************
*/
void comClearRxFifo(COM_PORT _ucPort)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}
	USART_ITConfig(pUart->uart, USART_IT_RXNE, DISABLE);/* 禁能接收中断 */
	pUart->usRxWrite = 0;
	pUart->usRxRead = 0;
	pUart->usRxCount = 0;
	memset(pUart->pRxBuf,0,pUart->usRxBufSize);
	USART_ITConfig(pUart->uart, USART_IT_RXNE, ENABLE);/* 使能接收中断 */
}

/*
*********************************************************************************************************
*	函 数 名: UartVarInit
*	功能说明: 初始化串口相关的变量
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void UartVarInit(void)
{
#if UART1_FIFO_EN == 1
	g_tUart1.uart = USART1;						/* STM32 串口设备 */
	g_tUart1.pTxBuf = g_TxBuf1;					/* 发送缓冲区指针 */
	g_tUart1.pRxBuf = g_RxBuf1;					/* 接收缓冲区指针 */
	g_tUart1.usTxBufSize = UART1_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart1.usRxBufSize = UART1_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart1.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart1.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart1.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart1.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart1.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart1.usTxCount = 0;						/* 待发送的数据个数 */
	g_tUart1.SendBefor = 0;						/* 发送数据前的回调函数 */
	g_tUart1.SendOver = 0;						/* 发送完毕后的回调函数 */
	g_tUart1.ReciveNew = 0;						/* 接收到新数据后的回调函数 */
#endif

#if UART2_FIFO_EN == 1
	g_tUart2.uart = USART2;						/* STM32 串口设备 */
	g_tUart2.pTxBuf = g_TxBuf2;					/* 发送缓冲区指针 */
	g_tUart2.pRxBuf = g_RxBuf2;					/* 接收缓冲区指针 */
	g_tUart2.usTxBufSize = UART2_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart2.usRxBufSize = UART2_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart2.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart2.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart2.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart2.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart2.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart2.usTxCount = 0;						/* 待发送的数据个数 */	
	g_tUart2.SendBefor = 0;						/* 发送数据前的回调函数 */
	g_tUart2.SendOver = 0;						/* 发送完毕后的回调函数 */
	g_tUart2.ReciveNew = 0;						/* 接收到新数据后的回调函数 */
#endif

#if UART3_FIFO_EN == 1
	g_tUart3.uart = USART3;						/* STM32 串口设备 */
	g_tUart3.pTxBuf = g_TxBuf3;					/* 发送缓冲区指针 */
	g_tUart3.pRxBuf = g_RxBuf3;					/* 接收缓冲区指针 */
	g_tUart3.usTxBufSize = UART3_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart3.usRxBufSize = UART3_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart3.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart3.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart3.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart3.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart3.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart3.usTxCount = 0;						/* 待发送的数据个数 */	
	g_tUart3.SendBefor = 0;						/* 发送数据前的回调函数 */
	g_tUart3.SendOver = 0;						/* 发送完毕后的回调函数 */
	g_tUart3.ReciveNew = 0;						/* 接收到新数据后的回调函数 */
#endif
}

/*
*********************************************************************************************************
*	函 数 名: InitHardUart
*	功能说明: 配置串口的硬件参数（波特率，数据位，停止位，起始位，校验位，中断使能）适合于STM32-F4开发板
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void InitHardUart(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
#if UART1_FIFO_EN == 1		/* 串口1 TX = PA9   RX = PA10 或 TX = PB6   RX = PB7*/
	/* 打开 GPIO 时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* 打开 UART 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* 将 PA9 映射为 USART1_TX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

	/* 将 PA10 映射为 USART1_RX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/* 配置 USART Tx 为复用功能 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 配置 USART Rx 为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 第2步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = UART1_BAUD;
	USART_InitStructure.USART_WordLength = UART1_WORD_LENGTH;
	USART_InitStructure.USART_StopBits = UART1_STOP_BIT;
	USART_InitStructure.USART_Parity = UART1_PARITY;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;

	USART_ClockInit(USART1, &USART_ClockInitStructure);
    USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		注意: 不要在此处打开发送中断
		发送中断使能在SendUart()函数打开
	*/
	USART_Cmd(USART1, ENABLE);		/* 使能串口 */

	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if UART2_FIFO_EN == 1
		/* 打开 GPIO 时钟 */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		/* 打开 UART 时钟 */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		GPIO_StructInit(&GPIO_InitStructure); 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		/* 将 PA2 映射为 USART2_TX */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		/* 将 PA3 映射为 USART2_RX */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
		
		/* 第2步： 配置串口硬件参数 */
		USART_InitStructure.USART_BaudRate = UART2_BAUD;
		USART_InitStructure.USART_WordLength = UART2_WORD_LENGTH;
		USART_InitStructure.USART_StopBits = UART2_STOP_BIT;
		USART_InitStructure.USART_Parity = UART2_PARITY;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		
		USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
		USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
		USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
		USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;

		USART_ClockInit(USART2, &USART_ClockInitStructure);
    USART_Init(USART2, &USART_InitStructure);

		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
		/*
			USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
			注意: 不要在此处打开发送中断
			发送中断使能在SendUart()函数打开
		*/
		USART_Cmd(USART2, ENABLE);		/* 使能串口 */

		/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
			如下语句解决第1个字节无法正确发送出去的问题 */
		USART_ClearFlag(USART2, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if UART3_FIFO_EN == 1
	/* 第1步： 配置GPIO */

	/* 打开 GPIO 时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* 打开 UART 时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* 将 PB10 映射为 UART3_TX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

	/* 将 PB11 映射为 UART3_RX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	/* 配置 USART Tx 为复用功能 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 配置 USART Rx 为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 第2步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = UART3_BAUD;
	USART_InitStructure.USART_WordLength = UART3_WORD_LENGTH;
	USART_InitStructure.USART_StopBits = UART3_STOP_BIT;
	USART_InitStructure.USART_Parity = UART3_PARITY;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;

	USART_ClockInit(USART3, &USART_ClockInitStructure);
    USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
//	USART_ITConfig(USART3, USART_IT_ERR, ENABLE);		/* gps的串口开启错误中断，防卡死 */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		注意: 不要在此处打开发送中断
		发送中断使能在SendUart()函数打开
	*/
	USART_Cmd(USART3, ENABLE);		/* 使能串口 */

	/* CPU的小缺陷：串口配置好，如果直接Send，则第1个字节发送不出去
		如下语句解决第1个字节无法正确发送出去的问题 */
	USART_ClearFlag(USART3, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif
}

/*
*********************************************************************************************************
*	函 数 名: ConfigUartNVIC
*	功能说明: 配置串口硬件中断.
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ConfigUartNVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

#if UART1_FIFO_EN == 1
	/* 使能串口0中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#if UART2_FIFO_EN == 1
	/* 使能串口1中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#if UART3_FIFO_EN == 1
	/* 使能串口2中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

/*
*********************************************************************************************************
*	函 数 名: UartGetChar
*	功能说明: 从串口接收缓冲区读取1字节数据 （用于主程序调用）
*	形    参: _pUart : 串口设备
*			  _pByte : 存放读取数据的指针
*	返 回 值: 0 表示无数据  1表示读取到数据
*********************************************************************************************************
*/
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte)
{
	uint8_t ret = 0;
	if(_pUart->usRxRead != _pUart->usRxWrite)
	{
		*_pByte = _pUart->pRxBuf[_pUart->usRxRead++];		/* 从串口接收FIFO取1个数据 */
		_pUart->usRxRead %= _pUart->usRxBufSize;
		ret = 1;
	}
	return ret;
}

/* 获取串口缓冲区数据长度 */
uint16_t getBufLength(COM_PORT _ucPort)
{
	UART_T *pUart;
	uint16_t Len = 0;
	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return 0;
	}
	if(pUart->usRxRead < pUart->usRxWrite)
	{
		Len = pUart->usRxWrite - pUart->usRxRead;
	}
	else if(pUart->usRxRead > pUart->usRxWrite)
	{
		Len = pUart->usRxBufSize + pUart->usRxWrite - pUart->usRxRead;
	}
	else
	{
		Len = 0;
	}
	return Len;
}

void comSetReciveCallBack(COM_PORT _ucPort,void (*ReciveCallBack)(uint8_t))
{
	UART_T *pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}
	pUart->ReciveNew = ReciveCallBack;
}

/*
*********************************************************************************************************
*	函 数 名: UartIRQ
*	功能说明: 供中断服务程序调用，通用串口中断处理函数
*	形    参: _pUart : 串口设备
*	返 回 值: 无
*********************************************************************************************************
*/
static uint8_t logCom = 0;
static uint8_t logGps = 1;
static void UartIRQ(UART_T *_pUart)
{
	uint8_t uchar;
	/* 处理接收中断  */
	if (USART_GetITStatus(_pUart->uart, USART_IT_RXNE) != RESET || USART_GetFlagStatus(_pUart->uart, USART_FLAG_ORE) != RESET)
	{
		uchar = USART_ReceiveData(_pUart->uart);
		/* 处理溢出中断 */
		if(USART_GetFlagStatus(_pUart->uart, USART_FLAG_ORE) != RESET)
		{
			/* 清除溢出中断 */
			USART_ClearFlag(_pUart->uart, USART_FLAG_ORE);
		}
		/* 从串口接收数据寄存器读取数据存放到接收FIFO */
		_pUart->pRxBuf[_pUart->usRxWrite++] = uchar;
		_pUart->usRxWrite %= _pUart->usRxBufSize;
		if(logCom == 1)
		{
			if(USART2 == _pUart->uart)
			{
					USART_SendData(USART1, uchar);
					/* Loop until the end of transmission */
					while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
			}
		}
		if(logGps == 1)
		{
			if(USART3 == _pUart->uart)
			{
					USART_SendData(USART1, uchar);
					/* Loop until the end of transmission */
					while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
			}
		}
		/* 回调函数,通知应用程序收到新数据,一般是发送1个消息或者设置一个标记 */
		if (_pUart->ReciveNew)
		{
			_pUart->ReciveNew(uchar);
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: USART1_IRQHandler  USART2_IRQHandler USART3_IRQHandler UART4_IRQHandler UART5_IRQHandler
*	功能说明: USART中断服务程序
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
#if UART1_FIFO_EN == 1
void USART1_IRQHandler(void)
{
	UartIRQ(&g_tUart1);
}
#endif

#if UART2_FIFO_EN == 1
void USART2_IRQHandler(void)
{
	UartIRQ(&g_tUart2);
}
#endif

#if UART3_FIFO_EN == 1
void USART3_IRQHandler(void)
{
//	UartIRQ(&g_tUart3);
	uint8_t uchar ; 
	const char *p;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET || USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
	{
		uchar = USART_ReceiveData(USART3);
		/* 处理溢出中断 */
		if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
		{
			/* 清除溢出中断 */
			USART_ClearFlag(USART3, USART_FLAG_ORE);
		}
		
		/* 串口调试用，直接把收到的字节发去串口1 */
		USART_SendData(USART1, uchar);        //通过库函数  发送数据

		/* 收到$表示一条NMEA语句开始了 */
		if(uchar == '$'){
			u8_rxFr=1;
			u8_rxBuf[0]=uchar;
			u8_rxBit=1;
			u16_bufLen=1;
		}
		else{
			if(u8_rxFr==1){		
				u8_rxBuf[u8_rxBit]=uchar;
				u8_rxBit++;
				u16_bufLen++;	

				if(uchar=='\n'){
					u8_rxFr=0; 
					u8_rxBit=0;
					osSemaphoreRelease(sem_gps_id);
				}		
				return;
			}
			return;
			
			
			
			
			
			/* u8_rxFr为1时候，表明收到的是一条NMEA语句内的字节 */
			if(u8_rxFr==1){				
				u8_rxBuf[u8_rxBit]=uchar;
				if(u8_rxBufTyp!=0){
					/* 根据u8_rxBufTyp不同，把接收到的字节记录入不同的数组 */
					switch(u8_rxBufTyp){
						case u8_rxTyp_gga:{
							u8_rxBuf_gga[u8_rxBit] = uchar;
							break;
						}
						case u8_rxTyp_rmc:{
							u8_rxBuf_rmc[u8_rxBit] = uchar;
							break;
						}
//						case u8_rxTyp_gsv:{
//							u8_rxBuf_gqgsv[u8_rxBit] = uchar;
//							break;
//						}
					}

				}
				u8_rxBit++;
				u16_bufLen++;
				
				/* 收到第6字节后，可以往前判断语句是什么ID，除了GGA、RMC、GSV ， 其他ID的NMEA语句可不需要 */
				/* 根据GGA RMC GSV的不同，把u8_rxBufTyp置为相对应的值 */
				/* 根据GGA RMC语句的种类不同分开两个不同的数组存数据 */
				if(u8_rxBit==6){
					if((u8_rxBuf[3]=='G')&&(u8_rxBuf[4]=='G')&&(u8_rxBuf[5]=='A')){
						u8_rxBufTyp = u8_rxTyp_gga;			
						u8_rxBuf_gga[1]=u8_rxBuf[1];
						u8_rxBuf_gga[2]=u8_rxBuf[2];
						u8_rxBuf_gga[3]=u8_rxBuf[3];
						u8_rxBuf_gga[4]=u8_rxBuf[4];
						u8_rxBuf_gga[5]=u8_rxBuf[5];
					}
					else if((u8_rxBuf[3]=='R')&&(u8_rxBuf[4]=='M')&&(u8_rxBuf[5]=='C')){
						u8_rxBufTyp = u8_rxTyp_rmc;
						u8_rxBuf_rmc[1]=u8_rxBuf[1];
						u8_rxBuf_rmc[2]=u8_rxBuf[2];
						u8_rxBuf_rmc[3]=u8_rxBuf[3];
						u8_rxBuf_rmc[4]=u8_rxBuf[4];
						u8_rxBuf_rmc[5]=u8_rxBuf[5];
					}
					else if((u8_rxBuf[3]=='G')&&(u8_rxBuf[4]=='S')&&(u8_rxBuf[5]=='V')){				
						u8_rxBufTyp = u8_rxTyp_gsv; 
					}
					else{
						/* 若把u8_rxFr置为0，则把GGA GSV RMC以外的语句全部不接收 */
						u8_rxFr=0; 						
						return;
					}		
				}
				
				/* 收到\n表示一条NMEA语句结束了 */
				if(uchar=='\n'){
					/* 串口调试用 */
//					comSendBuf(RS1_COM,(uint8_t*)u8_rxBuf,u16_bufLen);
					
					switch(u8_rxBufTyp){				
						case u8_rxTyp_gga:{
							u16_bufLen_gga = u16_bufLen;
							break;
						}
						case u8_rxTyp_rmc:{
							u16_bufLen_rmc = u16_bufLen;
							break;
						}
						
						case u8_rxTyp_gsv:{	
							/* GSV语句由于句子数量，卫星种类都有很多差异，语句有很多不确定性 */
							p=(const char*)&u8_rxBuf[11];
							switch(u8_rxBuf[2]){
								case 'A':{
									if(u8_rxBuf[u16_bufLen - 6] == '1'){
										sscanf(p,"%2hhu",&u8_vuNum[0]); 
									}
									else if(u8_rxBuf[u16_bufLen - 6] == '7'){
										sscanf(p,"%2hhu",&u8_vuNum[1]);
									}									
									break;
								}
								case 'B':{
									if(u8_rxBuf[u16_bufLen - 6] == '1'){
										sscanf(p,"%2hhu",&u8_vuNum[2]);
									}
									else if(u8_rxBuf[u16_bufLen - 6] == '5'){
										sscanf(p,"%2hhu",&u8_vuNum[3]);
									}									
									break;
								}
								case 'P':{
									if(u8_rxBuf[u16_bufLen - 6] == '1'){
										sscanf(p,"%2hhu",&u8_vuNum[4]);
									}
									else if(u8_rxBuf[u16_bufLen - 6] == '8'){
										sscanf(p,"%2hhu",&u8_vuNum[5]);
									}									
									break;
								}
								case 'Q':{
									if(u8_rxBuf[u16_bufLen - 6] == '1'){
										sscanf(p,"%2hhu",&u8_vuNum[6]);
									}
									else if(u8_rxBuf[u16_bufLen - 6] == '8'){
										sscanf(p,"%2hhu",&u8_vuNum[7]);
									}									
									break;
								}
								case 'L':{
									sscanf(p,"%2hhu",&u8_vuNum[8]);									
									break;
								}
							}		
							break;
						}
					}
					u8_rxFr=0; 
					u8_rxBit=0;
					u8_rxBufTyp=0;
				}
			}
			return;
		}
	}	

	

}


#endif


//UART3默认模特率
//__attribute__((weak))  uint32_t USART3_BAUD = 9600;
__attribute__((weak))  uint32_t USART3_BAUD = 115200;

//默认串口3波特率初始化
__attribute__((weak))  void InitUsart3Baud(void)
{
	
}


