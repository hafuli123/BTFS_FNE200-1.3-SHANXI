#include "MCP2515.h"
#include "stm32f4xx_spi.h"

/* 需要根据引脚设置进行修改. Modify below items for your SPI configurations */

#define SPI_CAN                 SPI1
#define SPI_CAN_CS_PIN					GPIO_Pin_4
#define SPI_TIMEOUT             10
#define MCP2515_CS_HIGH()   GPIO_SetBits(GPIOA,SPI_CAN_CS_PIN)
#define MCP2515_CS_LOW()    GPIO_ResetBits(GPIOA,SPI_CAN_CS_PIN)

/* Prototypes */
static void SPI_Tx(uint8_t data);
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length);
static uint8_t SPI_Rx(void);
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length);

/* MCP2515 初始化 */
uint8_t MCP2515_Initialize(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	
	/*  Enable SCK, MOSI, MISO and NSS GPIO clocks using RCC_AHB1PeriphClockCmd()*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* Connect PA5 to SPI1_SCK */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	/* Connect PA6 to SPI1_MISO */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	/* Connect PA7 to SPI1_MOSI */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_CAN_CS_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	/* 打开SPI时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_Cmd(SPI1, DISABLE); 
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
	
	/* 使能 SYSCFG 时钟，使用GPIO外部中断时必须使能SYSCFG时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	/* 使能 RCC_AHB1Periph_GPIOC 时钟*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	/* 选择震动检测的引脚 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	
	/* 设置引脚为输入模式 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	
	/* 设置引脚不上拉也不下拉 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	/*初始化IO*/
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* 连接 EXTI 中断源到CAN终端检测引脚 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource4);
	
	/* 选择 EXTI 中断源 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	
	/* 中断模式 */
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	
	/* 下降沿触发 */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	
	/* 使能中断/事件线 */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	
	EXTI_Init(&EXTI_InitStructure);
	
//	/* 配置中断源：按键1 */
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
//	
//	/* 配置子优先级：1 */
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	
//	/* 使能中断通道 */
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	
//	NVIC_Init(&NVIC_InitStructure);
	
  return MCP2515_TRUE;
}

/* MCP2515 切换到配置模式 */
uint8_t MCP2515_SetConfigMode(void)
{
  uint16_t loop = 10000;
  
  do {
		/* CANCTRL Register Configuration 模式设置 */  
		MCP2515_WriteByte(MCP2515_CANCTRL, 0x80);
    /*  确认模式切换 */    
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x80)
      return MCP2515_TRUE;
    
    loop--;
  } while(loop > 0); 
  
  return MCP2515_FALSE;
}

/* MCP2515 切换到标准模式 */
uint8_t MCP2515_SetNormalMode(void)
{
  uint16_t loop = 30000;
  
  do {
		/* CANCTRL Register Normal 模式设置 */  
		MCP2515_WriteByte(MCP2515_CANCTRL, 0x00);		
    /* 确认模式切换 */    
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x00)
      return MCP2515_TRUE;
    
    loop--;
  } while(loop > 0);
  
  return MCP2515_FALSE;
}

/* MCP2515 切换到Sleep模式 */
uint8_t MCP2515_SetSleepMode(void)
{
  uint16_t loop = 30000;
  
  do {
		/* CANCTRL Register Sleep 模式设置 */  
		MCP2515_WriteByte(MCP2515_CANCTRL, 0x20);		
    /*  确认模式切换  */    
    if((MCP2515_ReadByte(MCP2515_CANSTAT) & 0xE0) == 0x20)
      return MCP2515_TRUE;
    
    loop--;
  } while(loop > 0);
  
  return MCP2515_FALSE;
}

/* MCP2515 SPI-Reset */
void MCP2515_Reset(void)
{
  MCP2515_CS_LOW();
      
  SPI_Tx(MCP2515_RESET);
      
  MCP2515_CS_HIGH();
}

/* 读取1字节 */
uint8_t MCP2515_ReadByte (uint8_t address)
{
  uint8_t retVal;
  
  MCP2515_CS_LOW();
  
  SPI_Tx(MCP2515_READ);
  SPI_Tx(address);
  retVal = SPI_Rx();
      
  MCP2515_CS_HIGH();
  
  return retVal;
}

/* Sequential Bytes 读取 */
void MCP2515_ReadRxSequence(uint8_t instruction, uint8_t *data, uint8_t length)
{
  MCP2515_CS_LOW();
  
  SPI_Tx(instruction);        
  SPI_RxBuffer(data, length);
    
  MCP2515_CS_HIGH();
}

/* 写入1字节 */
void MCP2515_WriteByte(uint8_t address, uint8_t data)
{
  MCP2515_CS_LOW();  
  
  SPI_Tx(MCP2515_WRITE);
  SPI_Tx(address);
  SPI_Tx(data);  
    
  MCP2515_CS_HIGH();
}

/* Sequential Bytes 写入 */
void MCP2515_WriteByteSequence(uint8_t startAddress, uint8_t endAddress, uint8_t *data)
{  
  MCP2515_CS_LOW();
  
  SPI_Tx(MCP2515_WRITE);
  SPI_Tx(startAddress);
  SPI_TxBuffer(data, (endAddress - startAddress + 1));
  
  MCP2515_CS_HIGH();
}

/* TxBuffer Sequential Bytes 写入 */
void MCP2515_LoadTxSequence(uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data)
{    
  MCP2515_CS_LOW();
  
  SPI_Tx(instruction);
  SPI_TxBuffer(idReg, 4);
  SPI_Tx(dlc);
  SPI_TxBuffer(data, dlc);
       
  MCP2515_CS_HIGH();
}

/* TxBuffer 1 Bytes 写入 */
void MCP2515_LoadTxBuffer(uint8_t instruction, uint8_t data)
{
  MCP2515_CS_LOW();
  
  SPI_Tx(instruction);
  SPI_Tx(data);
        
  MCP2515_CS_HIGH();
}

/* RTS 通过命令 TxBuffer 发送 */
void MCP2515_RequestToSend(uint8_t instruction)
{
  MCP2515_CS_LOW();
  
  SPI_Tx(instruction);
      
  MCP2515_CS_HIGH();
}

/* MCP2515 Status 确认 */
uint8_t MCP2515_ReadStatus(void)
{
  uint8_t retVal;
  
  MCP2515_CS_LOW();
  
  SPI_Tx(MCP2515_READ_STATUS);
  retVal = SPI_Rx();
        
  MCP2515_CS_HIGH();
  
  return retVal;
}

/* MCP2515 RxStatus 确认寄存器 */
uint8_t MCP2515_GetRxStatus(void)
{
  uint8_t retVal;
  
  MCP2515_CS_LOW();
  
  SPI_Tx(MCP2515_RX_STATUS);
  retVal = SPI_Rx();
        
  MCP2515_CS_HIGH();
  
  return retVal;
}

/*  更改寄存器值  */
void MCP2515_BitModify(uint8_t address, uint8_t mask, uint8_t data)
{
  MCP2515_CS_LOW();
  
  SPI_Tx(MCP2515_BIT_MOD);
  SPI_Tx(address);
  SPI_Tx(mask);
  SPI_Tx(data);
        
  MCP2515_CS_HIGH();
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
	while (SPI_I2S_GetFlagStatus(SPI_CAN, SPI_I2S_FLAG_TXE) == RESET);

	/* 通过SPI硬件发送1个字节 */
	SPI_I2S_SendData(SPI_CAN, _ucValue);

	/* 等待接收一个字节任务完成 */
	while (SPI_I2S_GetFlagStatus(SPI_CAN, SPI_I2S_FLAG_RXNE) == RESET);

	/* 返回从SPI总线读到的数据 */
	return SPI_I2S_ReceiveData(SPI_CAN);
}

/* SPI Tx Wrapper */
static void SPI_Tx(uint8_t data)
{
	sf_SendByte(data);
}

/* SPI Tx Wrapper */
static void SPI_TxBuffer(uint8_t *buffer, uint8_t length)
{
	uint8_t i;
	for(i = 0;i<length;i++)
	{
		sf_SendByte(buffer[i]);
	}
}

/* SPI Rx Wrapper */
static uint8_t SPI_Rx(void)
{
  return sf_SendByte(0x00);
}

/* SPI Rx Wrapper */
static void SPI_RxBuffer(uint8_t *buffer, uint8_t length)
{
	uint8_t i;
	for(i = 0;i<length;i++)
	{
		buffer[i] = sf_SendByte(0x00);
	}
}
