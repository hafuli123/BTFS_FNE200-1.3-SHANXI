#include "driver.h"
#include "stm32f4xx_spi.h"
#include "cmsis_os2.h" 
#include "string.h"

#define TMC_SEND_MAX 261
#define TMC_RECE_MAX 258
#define TMC_SPI_MAX 4

#define FRAME_TAG        0xAA
#define UNDEFINE_FRAME   0x99
#define WRONG_TAG        0xBB
#define WRONG_LEN        0xCC
#define WRONG_CRC        0xDD

#define TMC_SPI					SPI2

/* 片选GPIO端口  */
#define TMC_CS_GPIO			GPIOA
#define TMC_CS_PIN			GPIO_Pin_15

#define TMC_SPI_DISABLE() GPIO_SetBits(TMC_CS_GPIO,TMC_CS_PIN)
#define TMC_SPI_ENABLE() GPIO_ResetBits(TMC_CS_GPIO,TMC_CS_PIN)

static uint8_t outBuf[TMC_RECE_MAX + TMC_SPI_MAX];
static int g_SPI_Fd = 0;
static TMS_DRIVER_OPS spi_ops;
static TMS_DRIVER spi_driver = {
    "SPI",
    &spi_ops
};

static void delayT(int ns)
{
	osDelay(1);
}

static uint8_t tmc_spi_crc(uint8_t *InBuf, int len)
{
    uint8_t ret = 0;
    int i;

    for (i=0;i<len;i++) {
        ret = ret ^ InBuf[i];
    }
    return ret;
}

/*
*********************************************************************************************************
*	函 数 名: sf_SendByte
*	功能说明: 向器件发送一个字节，同时从MISO口线采样器件返回的数据
*	形    参:  _ucByte : 发送的字节值
*	返 回 值: 从MISO口线采样器件返回的数据
*********************************************************************************************************
*/
static uint8_t sf_SendByte(uint8_t _ucValue)
{
	/* 等待上个数据未发送完毕 */
	while (SPI_I2S_GetFlagStatus(TMC_SPI, SPI_I2S_FLAG_TXE) == RESET);

	/* 通过SPI硬件发送1个字节 */
	SPI_I2S_SendData(TMC_SPI, _ucValue);

	/* 等待接收一个字节任务完成 */
	while (SPI_I2S_GetFlagStatus(TMC_SPI, SPI_I2S_FLAG_RXNE) == RESET);

	/* 返回从SPI总线读到的数据 */
	return SPI_I2S_ReceiveData(TMC_SPI);
}

static int SPI_Write(uint8_t *TxBuf, int len)
{
		int i;
		TMC_SPI_ENABLE();
		for(i= 0;i<len;i++)
		{
			sf_SendByte(TxBuf[i]);
		}
		TMC_SPI_DISABLE();
    return 0;
}

static int SPI_Read(uint8_t *RxBuf, int len)
{
		int i;
		TMC_SPI_ENABLE();
		for(i=0;i<len;i++)
		{
			RxBuf[i] = sf_SendByte(0xFF);
		}
		TMC_SPI_DISABLE();
    return 0;
}

static int tmc_spi_send(uint8_t *TxBuf, int len)
{
    int outlen = len + TMC_SPI_MAX;
    if (!g_SPI_Fd) {
        return -1;
    }

    if (len > TMC_SEND_MAX) {
        return -1;
    }

    memset(outBuf,0,sizeof(outBuf));

    outBuf[0] = FRAME_TAG;
    outBuf[1] = len >> 8;
    outBuf[2] = len >> 0;
    memcpy(outBuf + 3, TxBuf, len);
    outBuf[outlen -1] = tmc_spi_crc(TxBuf, len);

    return SPI_Write(outBuf,outlen);
}

static int tmc_spi_receive(uint8_t *RxBuf, int* len, int max_wait_time)
{
    int flag = 0;
    int findAA = 0;
    int count = DELAY_1_SEC * max_wait_time;
    int inlen;
    uint8_t crc ;

    if (!g_SPI_Fd) {
        return -1;
    }

    memset(outBuf,0,sizeof(outBuf));

    delayT(1);//200us

    while(count--) {
        if(SPI_Read(outBuf, 1) < 0) {
            return -1;
        }

        if (outBuf[0] == FRAME_TAG) {
            findAA = 1;
            break;
        }
        else if (outBuf[0] == UNDEFINE_FRAME) {
            delayT(3); 
            return -1;
        }
        else if (outBuf[0] == WRONG_TAG) {
            delayT(3); 
            return -1;
        }
        else if (outBuf[0] == WRONG_LEN) {
            delayT(3); 
            return -1;
        }
        else if (outBuf[0] == WRONG_CRC) {
            delayT(3);
            return -1;
        }
        else if (outBuf[0] == 0xFF) {
            //busy
        }
        else {
            if (flag <= 10) {
                flag++;
            }
        }

        delayT(1);//30us
    }

    if (findAA) {
        if (SPI_Read(outBuf + 1, 2) < 0) {
            return -1;
        }
        inlen = (outBuf[1] << 8) | (outBuf[2]);

        if (SPI_Read(outBuf + 3, inlen + 1) < 0) {
            return -1;
        }

        crc = tmc_spi_crc(outBuf + 3, inlen);
        if (crc != outBuf[inlen + 3]) {
            return -1;
        }

        if (RxBuf) {
            memcpy(RxBuf, outBuf + 3, inlen);
        }
        if (len) {
            *len = inlen;
        }

        delayT(1); //receive to send, delay 100us
        return 0;
    }
    else {
        return -1;
    }
}

int spi_init(char *device)
{		
		SPI_InitTypeDef  SPI_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;

    if (g_SPI_Fd) {
        return 1;
    }
		g_SPI_Fd = 1;
		
		/*  Enable SCK, MOSI, MISO and NSS GPIO clocks using RCC_AHB1PeriphClockCmd()*/
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
		
		/* Connect PB13 to SPI2_SCK */
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
		/* Connect PB14 to SPI2_MISO */
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
		/* Connect PB15 to SPI2_MOSI */
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
		
		GPIO_InitStructure.GPIO_Pin = TMC_CS_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(TMC_CS_GPIO, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
				
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;

		/* 打开SPI时钟 */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		SPI_Cmd(TMC_SPI, DISABLE); 
		SPI_Init(TMC_SPI, &SPI_InitStructure);
		SPI_Cmd(TMC_SPI, ENABLE);
		
    return 0;
}

int spi_finish()
{
	GPIO_InitTypeDef  gpio_init;	
	/* 关闭	PB13，PB14，PB15 */
	gpio_init.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	gpio_init.GPIO_Mode = GPIO_Mode_AN;
	gpio_init.GPIO_OType = GPIO_OType_OD;
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;				
	GPIO_Init(GPIOB, &gpio_init);
	/* 关闭 PA15，*/
	gpio_init.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &gpio_init);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);		//SPI1_SS					//加密芯片
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);		//SPI2_SCK
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);		//SPI2_MISO
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);		//SPI2_MOSI
	SPI_Cmd(TMC_SPI, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, DISABLE); 	
  g_SPI_Fd = 0;
  return SUCCEED;
}

int spi_reset()
{
    return SUCCEED;
}

int spi_wakeup(void)
{
   int ret = 0;
   uint8_t wakeup = 0x55;

   if (!g_SPI_Fd) {
       return -1;
   }


   ret = SPI_Write(&wakeup, 1);
   if (ret < 0) {
       return ret;
   }

   delayT(1);//200us

   return SUCCEED;
}

int spi_sleep(void)
{
		uint8_t sendbuf[2] = {0x02,0x02};
		uint8_t recebuf[2];
		int len;
		int ret = 0;

		if (!g_SPI_Fd) {
				return -1;
		}

		ret = tmc_spi_send(sendbuf, 2);
		if (ret != 0) {
				return ret;
		}

		ret = tmc_spi_receive(recebuf, &len, 1);
		if (ret != 0) {
				return ret;
		}

		if ((len != 2) || ((recebuf[0] != 0x90) || (recebuf[1] != 0x00))) {
				return -1;
		}

    return 0;
}

int spi_transmit(TRANSMIT_DATA *pData)
{
    int i,err,RETRY_COUNT=3;

    for (i = 0; i < RETRY_COUNT; i++) {
        err = tmc_spi_send(pData->tx, pData->tx_len);
        if (err) {
            continue;
        }

        err = tmc_spi_receive(pData->rx, (int*)&pData->rx_len, pData->max_wait_time);
        if (err) {
            continue;
        }

        return SUCCEED;
    }
    return FAILED;
}

TMS_DRIVER * tms_get_spi_driver(void)
{
    spi_ops.init = spi_init;
    spi_ops.finish = spi_finish;
    spi_ops.reset = spi_reset;
    spi_ops.wakeup = spi_wakeup;
    spi_ops.sleep = NULL;
    spi_ops.transmit = spi_transmit;

    return &spi_driver;
}

