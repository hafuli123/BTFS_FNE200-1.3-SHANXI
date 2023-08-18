#include "bsp_uart_fifo.h"
#include "stm32f4xx_usart.h"
#include "string.h"
#include "cmsis_os2.h" 
#include "stdio.h"

extern uint32_t apple;
extern osSemaphoreId_t sem_gps_id;

/* �����豸�ṹ�� */
typedef struct
{
	USART_TypeDef *uart;		/* STM32�ڲ������豸ָ�� */
	uint8_t *pTxBuf;			/* ���ͻ����� */
	uint8_t *pRxBuf;			/* ���ջ����� */
	uint16_t usTxBufSize;		/* ���ͻ�������С */
	uint16_t usRxBufSize;		/* ���ջ�������С */
	uint16_t usTxWrite;			/* ���ͻ�����дָ�� */
	uint16_t usTxRead;			/* ���ͻ�������ָ�� */
	uint16_t usTxCount;			/* �ȴ����͵����ݸ��� */

	uint16_t usRxWrite;			/* ���ջ�����дָ�� */
	uint16_t usRxRead;			/* ���ջ�������ָ�� */
	uint16_t usRxCount;			/* ��δ��ȡ�������ݸ��� */

	void (*SendBefor)(void); 	/* ��ʼ����֮ǰ�Ļص�����ָ�루��Ҫ����RS485�л�������ģʽ�� */
	void (*SendOver)(void); 	/* ������ϵĻص�����ָ�루��Ҫ����RS485������ģʽ�л�Ϊ����ģʽ�� */
	void (*ReciveNew)(uint8_t);	/* �����յ����ݵĻص�����ָ�� */
}UART_T;

/* ����ÿ�����ڽṹ����� */
#if UART1_FIFO_EN == 1
	static UART_T g_tUart1;
	static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART2_FIFO_EN == 1
	static UART_T g_tUart2;
	static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE];		/* ���ջ����� */
#endif

#if UART3_FIFO_EN == 1
	static UART_T g_tUart3;
	static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE];		/* ���ջ����� */

#endif

volatile uint8_t u8_rxBuf[RX_MAX_LEN];	/* ���ڽ��ջ����� */

volatile uint8_t u8_rxBuf_gga[RX_MAX_LEN];
volatile uint8_t u8_rxBuf_rmc[RX_MAX_LEN];
volatile uint8_t u8_rxBuf_gsv[RX_MAX_LEN];
volatile uint8_t u8_rxBufTyp;						/* Ŀǰ�ڽ��յ�����NMEA�������� */
volatile uint16_t u16_bufLen_gga;
volatile uint16_t u16_bufLen_rmc;
volatile uint16_t u16_bufLen_gsv;

volatile uint8_t u8_rxBit;							/* ��Ǵ��ڽ��������Ԫ�غ� */
volatile uint16_t u16_bufLen;						/* ���ڽ��յ����ĳ��� */
volatile uint8_t u8_rxFr;								/* �����յ�rtkģ��'$'��NMEA��俪ͷ�ַ�����Ϊ1�������յ�/n�����ַ����û�0 */

volatile uint8_t u8_vuNum[9];						/* GSV��ȡ���������������� */



static void UartVarInit(void);
static void InitHardUart(void);
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte);
static void UartIRQ(UART_T *_pUart);
static void ConfigUartNVIC(void);

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitUart
*	����˵��: ��ʼ������Ӳ��������ȫ�ֱ�������ֵ.
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitUart(void)
{
	UartVarInit();		/* �����ȳ�ʼ��ȫ�ֱ���,������Ӳ�� */

	InitUsart3Baud();	/*��ʼ������3������*/
	
	InitHardUart();		/* ���ô��ڵ�Ӳ������(�����ʵ�) */

	ConfigUartNVIC();	/* ���ô����ж� */
}

/*
*********************************************************************************************************
*	�� �� ��: ComToUart
*	����˵��: ��COM�˿ں�ת��ΪUARTָ��
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*	�� �� ֵ: uartָ��
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
		/* �����κδ��� */
		return 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: comSendBuf
*	����˵��: �򴮿ڷ���һ�����ݡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*			  _ucaBuf: �����͵����ݻ�����
*			  _usLen : ���ݳ���
*	�� �� ֵ: ��
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
*	�� �� ��: comSendChar
*	����˵��: �򴮿ڷ���1���ֽڡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*			  _ucByte: �����͵�����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comSendChar(COM_PORT _ucPort, uint8_t _ucByte)
{
	comSendBuf(_ucPort, &_ucByte, 1); //�򴮿��жϷ���һ�����ݡ�
}


//��ӡ����
void printfData(const char *_ch)
{
	uint8_t len = strlen(_ch);
	if(len<200)
		comSendBuf(RS1_COM,(uint8_t*)_ch,len);
}

/*
*********************************************************************************************************
*	�� �� ��: comGetChar
*	����˵��: �Ӵ��ڻ�������ȡ1�ֽڣ��������������������ݾ���������
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*			  _pByte: ���յ������ݴ���������ַ
*	�� �� ֵ: 0 ��ʾ������, 1 ��ʾ��ȡ����Ч�ֽ�
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
*	�� �� ��: comChkBufDelay
*	����˵��: �Ӵ��ڻ������������
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*	�� �� ֵ: 0 ��ʾ������, 1 ��ʾ��ȡ����Ч�ֽ�
*********************************************************************************************************
*/
uint8_t comChkBufDelay(COM_PORT _ucPort,char *ack,uint16_t _usTimeOut)
{
	uint16_t pos;
	uint8_t i;
	uint16_t timeOutCnt = _usTimeOut;
	uint16_t StatComBufPos;								//��ʼ��ȡ���������ݵ�λ��ָ��
	uint16_t EndComBufPos;								//������ȡ���������ݵ�λ��ָ��
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
*	�� �� ��: comGetChar
*	����˵��: �Ӵ��ڻ�������ȡ1�ֽڣ��������������������ݾ���������
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*			  _pByte: ���յ������ݴ���������ַ
*	�� �� ֵ: 0 ��ʾ������, 1 ��ʾ��ȡ����Ч�ֽ�
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
*	�� �� ��: comClearTxFifo
*	����˵��: ���㴮�ڷ��ͻ�����
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*	�� �� ֵ: ��
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
*	�� �� ��: comClearRxFifo
*	����˵��: ���㴮�ڽ��ջ�����
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*	�� �� ֵ: ��
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
	USART_ITConfig(pUart->uart, USART_IT_RXNE, DISABLE);/* ���ܽ����ж� */
	pUart->usRxWrite = 0;
	pUart->usRxRead = 0;
	pUart->usRxCount = 0;
	memset(pUart->pRxBuf,0,pUart->usRxBufSize);
	USART_ITConfig(pUart->uart, USART_IT_RXNE, ENABLE);/* ʹ�ܽ����ж� */
}

/*
*********************************************************************************************************
*	�� �� ��: UartVarInit
*	����˵��: ��ʼ��������صı���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void UartVarInit(void)
{
#if UART1_FIFO_EN == 1
	g_tUart1.uart = USART1;						/* STM32 �����豸 */
	g_tUart1.pTxBuf = g_TxBuf1;					/* ���ͻ�����ָ�� */
	g_tUart1.pRxBuf = g_RxBuf1;					/* ���ջ�����ָ�� */
	g_tUart1.usTxBufSize = UART1_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart1.usRxBufSize = UART1_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart1.usTxWrite = 0;						/* ����FIFOд���� */
	g_tUart1.usTxRead = 0;						/* ����FIFO������ */
	g_tUart1.usRxWrite = 0;						/* ����FIFOд���� */
	g_tUart1.usRxRead = 0;						/* ����FIFO������ */
	g_tUart1.usRxCount = 0;						/* ���յ��������ݸ��� */
	g_tUart1.usTxCount = 0;						/* �����͵����ݸ��� */
	g_tUart1.SendBefor = 0;						/* ��������ǰ�Ļص����� */
	g_tUart1.SendOver = 0;						/* ������Ϻ�Ļص����� */
	g_tUart1.ReciveNew = 0;						/* ���յ������ݺ�Ļص����� */
#endif

#if UART2_FIFO_EN == 1
	g_tUart2.uart = USART2;						/* STM32 �����豸 */
	g_tUart2.pTxBuf = g_TxBuf2;					/* ���ͻ�����ָ�� */
	g_tUart2.pRxBuf = g_RxBuf2;					/* ���ջ�����ָ�� */
	g_tUart2.usTxBufSize = UART2_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart2.usRxBufSize = UART2_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart2.usTxWrite = 0;						/* ����FIFOд���� */
	g_tUart2.usTxRead = 0;						/* ����FIFO������ */
	g_tUart2.usRxWrite = 0;						/* ����FIFOд���� */
	g_tUart2.usRxRead = 0;						/* ����FIFO������ */
	g_tUart2.usRxCount = 0;						/* ���յ��������ݸ��� */
	g_tUart2.usTxCount = 0;						/* �����͵����ݸ��� */	
	g_tUart2.SendBefor = 0;						/* ��������ǰ�Ļص����� */
	g_tUart2.SendOver = 0;						/* ������Ϻ�Ļص����� */
	g_tUart2.ReciveNew = 0;						/* ���յ������ݺ�Ļص����� */
#endif

#if UART3_FIFO_EN == 1
	g_tUart3.uart = USART3;						/* STM32 �����豸 */
	g_tUart3.pTxBuf = g_TxBuf3;					/* ���ͻ�����ָ�� */
	g_tUart3.pRxBuf = g_RxBuf3;					/* ���ջ�����ָ�� */
	g_tUart3.usTxBufSize = UART3_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart3.usRxBufSize = UART3_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart3.usTxWrite = 0;						/* ����FIFOд���� */
	g_tUart3.usTxRead = 0;						/* ����FIFO������ */
	g_tUart3.usRxWrite = 0;						/* ����FIFOд���� */
	g_tUart3.usRxRead = 0;						/* ����FIFO������ */
	g_tUart3.usRxCount = 0;						/* ���յ��������ݸ��� */
	g_tUart3.usTxCount = 0;						/* �����͵����ݸ��� */	
	g_tUart3.SendBefor = 0;						/* ��������ǰ�Ļص����� */
	g_tUart3.SendOver = 0;						/* ������Ϻ�Ļص����� */
	g_tUart3.ReciveNew = 0;						/* ���յ������ݺ�Ļص����� */
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: InitHardUart
*	����˵��: ���ô��ڵ�Ӳ�������������ʣ�����λ��ֹͣλ����ʼλ��У��λ���ж�ʹ�ܣ��ʺ���STM32-F4������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void InitHardUart(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
#if UART1_FIFO_EN == 1		/* ����1 TX = PA9   RX = PA10 �� TX = PB6   RX = PB7*/
	/* �� GPIO ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* �� UART ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* �� PA9 ӳ��Ϊ USART1_TX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);

	/* �� PA10 ӳ��Ϊ USART1_RX */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/* ���� USART Tx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* �������Ϊ���� */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* �ڲ���������ʹ�� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* ����ģʽ */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ���� USART Rx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ��2���� ���ô���Ӳ������ */
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

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		ע��: ��Ҫ�ڴ˴��򿪷����ж�
		�����ж�ʹ����SendUart()������
	*/
	USART_Cmd(USART1, ENABLE);		/* ʹ�ܴ��� */

	/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
		�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART1, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

#if UART2_FIFO_EN == 1
		/* �� GPIO ʱ�� */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		/* �� UART ʱ�� */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

		GPIO_StructInit(&GPIO_InitStructure); 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* ����ģʽ */
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		/* �� PA2 ӳ��Ϊ USART2_TX */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		/* �� PA3 ӳ��Ϊ USART2_RX */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
		
		/* ��2���� ���ô���Ӳ������ */
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

		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
		/*
			USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
			ע��: ��Ҫ�ڴ˴��򿪷����ж�
			�����ж�ʹ����SendUart()������
		*/
		USART_Cmd(USART2, ENABLE);		/* ʹ�ܴ��� */

		/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
			�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
		USART_ClearFlag(USART2, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif

#if UART3_FIFO_EN == 1
	/* ��1���� ����GPIO */

	/* �� GPIO ʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* �� UART ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* �� PB10 ӳ��Ϊ UART3_TX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);

	/* �� PB11 ӳ��Ϊ UART3_RX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	/* ���� USART Tx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* �������Ϊ���� */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* �ڲ���������ʹ�� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* ����ģʽ */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ���� USART Rx Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* ��2���� ���ô���Ӳ������ */
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

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
//	USART_ITConfig(USART3, USART_IT_ERR, ENABLE);		/* gps�Ĵ��ڿ��������жϣ������� */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		ע��: ��Ҫ�ڴ˴��򿪷����ж�
		�����ж�ʹ����SendUart()������
	*/
	USART_Cmd(USART3, ENABLE);		/* ʹ�ܴ��� */

	/* CPU��Сȱ�ݣ��������úã����ֱ��Send�����1���ֽڷ��Ͳ���ȥ
		�����������1���ֽ��޷���ȷ���ͳ�ȥ������ */
	USART_ClearFlag(USART3, USART_FLAG_TC);     /* �巢����ɱ�־��Transmission Complete flag */
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: ConfigUartNVIC
*	����˵��: ���ô���Ӳ���ж�.
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ConfigUartNVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

#if UART1_FIFO_EN == 1
	/* ʹ�ܴ���0�ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#if UART2_FIFO_EN == 1
	/* ʹ�ܴ���1�ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#if UART3_FIFO_EN == 1
	/* ʹ�ܴ���2�ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

/*
*********************************************************************************************************
*	�� �� ��: UartGetChar
*	����˵��: �Ӵ��ڽ��ջ�������ȡ1�ֽ����� ��������������ã�
*	��    ��: _pUart : �����豸
*			  _pByte : ��Ŷ�ȡ���ݵ�ָ��
*	�� �� ֵ: 0 ��ʾ������  1��ʾ��ȡ������
*********************************************************************************************************
*/
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte)
{
	uint8_t ret = 0;
	if(_pUart->usRxRead != _pUart->usRxWrite)
	{
		*_pByte = _pUart->pRxBuf[_pUart->usRxRead++];		/* �Ӵ��ڽ���FIFOȡ1������ */
		_pUart->usRxRead %= _pUart->usRxBufSize;
		ret = 1;
	}
	return ret;
}

/* ��ȡ���ڻ��������ݳ��� */
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
*	�� �� ��: UartIRQ
*	����˵��: ���жϷ��������ã�ͨ�ô����жϴ�����
*	��    ��: _pUart : �����豸
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static uint8_t logCom = 0;
static uint8_t logGps = 1;
static void UartIRQ(UART_T *_pUart)
{
	uint8_t uchar;
	/* ��������ж�  */
	if (USART_GetITStatus(_pUart->uart, USART_IT_RXNE) != RESET || USART_GetFlagStatus(_pUart->uart, USART_FLAG_ORE) != RESET)
	{
		uchar = USART_ReceiveData(_pUart->uart);
		/* ��������ж� */
		if(USART_GetFlagStatus(_pUart->uart, USART_FLAG_ORE) != RESET)
		{
			/* �������ж� */
			USART_ClearFlag(_pUart->uart, USART_FLAG_ORE);
		}
		/* �Ӵ��ڽ������ݼĴ�����ȡ���ݴ�ŵ�����FIFO */
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
		/* �ص�����,֪ͨӦ�ó����յ�������,һ���Ƿ���1����Ϣ��������һ����� */
		if (_pUart->ReciveNew)
		{
			_pUart->ReciveNew(uchar);
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: USART1_IRQHandler  USART2_IRQHandler USART3_IRQHandler UART4_IRQHandler UART5_IRQHandler
*	����˵��: USART�жϷ������
*	��    ��: ��
*	�� �� ֵ: ��
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
		/* ��������ж� */
		if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
		{
			/* �������ж� */
			USART_ClearFlag(USART3, USART_FLAG_ORE);
		}
		
		/* ���ڵ����ã�ֱ�Ӱ��յ����ֽڷ�ȥ����1 */
		USART_SendData(USART1, uchar);        //ͨ���⺯��  ��������

		/* �յ�$��ʾһ��NMEA��俪ʼ�� */
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
			
			
			
			
			
			/* u8_rxFrΪ1ʱ�򣬱����յ�����һ��NMEA����ڵ��ֽ� */
			if(u8_rxFr==1){				
				u8_rxBuf[u8_rxBit]=uchar;
				if(u8_rxBufTyp!=0){
					/* ����u8_rxBufTyp��ͬ���ѽ��յ����ֽڼ�¼�벻ͬ������ */
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
				
				/* �յ���6�ֽں󣬿�����ǰ�ж������ʲôID������GGA��RMC��GSV �� ����ID��NMEA���ɲ���Ҫ */
				/* ����GGA RMC GSV�Ĳ�ͬ����u8_rxBufTyp��Ϊ���Ӧ��ֵ */
				/* ����GGA RMC�������಻ͬ�ֿ�������ͬ����������� */
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
						/* ����u8_rxFr��Ϊ0�����GGA GSV RMC��������ȫ�������� */
						u8_rxFr=0; 						
						return;
					}		
				}
				
				/* �յ�\n��ʾһ��NMEA�������� */
				if(uchar=='\n'){
					/* ���ڵ����� */
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
							/* GSV������ھ����������������඼�кܶ���죬����кܶ಻ȷ���� */
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


//UART3Ĭ��ģ����
//__attribute__((weak))  uint32_t USART3_BAUD = 9600;
__attribute__((weak))  uint32_t USART3_BAUD = 115200;

//Ĭ�ϴ���3�����ʳ�ʼ��
__attribute__((weak))  void InitUsart3Baud(void)
{
	
}


