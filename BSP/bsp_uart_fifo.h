#ifndef _BSP_USART_FIFO_H_
#define _BSP_USART_FIFO_H_

#include "stdint.h"
#define	UART1_FIFO_EN	1
#define	UART2_FIFO_EN	1
#define	UART3_FIFO_EN	1

#define PARITY_NONE     0
#define PARITY_ODD      1
#define PARITY_EVEN     2

#define UART_CHAT_TIME  5  /* ms */
/* ����˿ں� */
typedef enum
{
	RS1_COM = 0,	/* USART0 */
	RS2_COM = 1,	/* USART1 */
	RS3_COM = 2,     /* USART2 */
}COM_PORT;

/* ���崮�ڲ����ʺ�FIFO��������С����Ϊ���ͻ������ͽ��ջ�����, ֧��ȫ˫�� */
#if UART1_FIFO_EN == 1
	#define UART1_BAUD			115200
	#define UART1_WORD_LENGTH   8
	#define UART1_STOP_BIT      1
	#define UART1_PARITY        PARITY_NONE
	#define UART1_TX_BUF_SIZE	1*1
	#define UART1_RX_BUF_SIZE	2*512+64
	#define UART1_TIMER         0
#endif

#if UART2_FIFO_EN == 1
	#define UART2_BAUD			115200
	#define UART2_WORD_LENGTH   8
	#define UART2_STOP_BIT      1
	#define UART2_PARITY        PARITY_NONE
	#define UART2_TX_BUF_SIZE	1*1
	#define UART2_RX_BUF_SIZE	4*512+64
	#define UART2_TIMER         1
#endif

#if UART3_FIFO_EN == 1
	#define UART3_BAUD			USART3_BAUD
	#define UART3_WORD_LENGTH   8
	#define UART3_STOP_BIT      1
	#define UART3_PARITY        PARITY_NONE
	#define UART3_TX_BUF_SIZE	1*1
	#define UART3_RX_BUF_SIZE	1*16
	#define UART3_TIMER         3
	#define RX_MAX_LEN	100				/* ���ڽ��ջ������Ĵ�С */
#endif
#define RX_MAX_VEC	100					/* һ�ַ������������NMEA��� */
enum{
	u8_rxTyp_gga =1, 
	u8_rxTyp_rmc ,
	u8_rxTyp_gsv,
	u8_rxTyp_gpgsv,
	u8_rxTyp_glgsv,
	u8_rxTyp_gagsv,
	u8_rxTyp_gbgsv,
	u8_rxTyp_gqgsv,
};

/* ���ڳ�ʼ�� */
void bsp_InitUart(void);
/* �ַ������� */
void comSendBuf(COM_PORT _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
/* ��������֡���� */
uint16_t comGetBufDelay(COM_PORT _ucPort,uint16_t bufMaxLen,uint8_t *_pByte,uint16_t _usTimeOut);
/* ��������֡��� */
uint8_t comChkBufDelay(COM_PORT _ucPort,char *ack,uint16_t _usTimeOut);
/* ���ڵ��ֽڽ��� */
uint8_t comGetChar(COM_PORT _ucPort, uint8_t *_pByte);
/* д���ֽ� */
void comSendChar(COM_PORT _ucPort, uint8_t _ucByte);
void comClearTxFifo(COM_PORT _ucPort);
void comClearRxFifo(COM_PORT _ucPort);
uint16_t getBufLength(COM_PORT _ucPort);
void comSetReciveCallBack(COM_PORT _ucPort,void (*ReciveCallBack)(uint8_t));

void printfData(const char *_ch);
extern void InitUsart3Baud(void);
extern uint32_t USART3_BAUD;

extern volatile uint8_t u8_rxBuf_gga[RX_MAX_LEN];
extern volatile uint8_t u8_rxBuf_rmc[RX_MAX_LEN];

extern volatile uint16_t u16_bufLen_gga;
extern volatile uint16_t u16_bufLen_rmc;


extern volatile uint8_t u8_vuNum[9];
#endif

