#include "bsp_can.h"

#include "cmsis_os2.h" 

#include "stm32f4xx_can.h"
#include "CANSPI.h"
#include "MCP2515.h"

#define __CTRL1                 1
#define __CTRL2                 2
#define __CTRL3                 3

#define CAN_CLK               42000000

static const osMessageQueueAttr_t queueAttr_Q_CAN = {.name = "CAN1_QUEUE",};					//队列
static osMessageQueueId_t Q_CAN;																											//队列ID
static uint8_t irqSeq;

CAN_TypeDef *CAN_CTRL[] = { CAN1, CAN2 };																							//CAN结构体

/* 设置位时序寄存器：波特率预分配器 */
static void CAN_set_timing (uint32_t ctrl, uint32_t tseg1, uint32_t tseg2, uint32_t sjw, uint32_t brp) {
  CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];

  CANx->BTR &= ~(((          0x03) << 24) | ((            0x07) << 20) | ((            0x0F) << 16) | (          0x3FF));
  CANx->BTR |=  ((((sjw-1) & 0x03) << 24) | (((tseg2-1) & 0x07) << 20) | (((tseg1-1) & 0x0F) << 16) | ((brp-1) & 0x3FF));
}


/*--------------------------- CAN_set_baudrate ------------------------------
 *
 *  Setup the requested baudrate		设置波特率
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/
static void CAN_hw_set_baudrate (uint32_t ctrl, uint32_t baudrate)  {
  uint32_t brp;
  /* Note: this calculations fit for CAN_CLK = 42MHz                         */
  /* Determine which nominal time to use for requested baudrate and set
     appropriate bit timing                                                  */
  if (baudrate <= 1000000)  {
    brp  = (CAN_CLK / 7) / baudrate;

    /* Load the baudrate registers BTR                                       */
    /* so that sample point is at about 72% bit time from bit start          */
    /* TSEG1 = 4, TSEG2 = 2, SJW = 3 => 1 CAN bit = 7 TQ, sample at 71%      */
    CAN_set_timing(ctrl,  4, 2, 3, brp);
  }
}


/*************************** Module Functions ********************************/

/*--------------------------- CAN_hw_setup ----------------------------------
 *											初始化 CAN引脚及中断向量表
 *  Setup CAN transmit and receive PINs and interrupt vectors
 *									
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_setup (uint32_t ctrl)  {
  switch (ctrl) {
    case 1: 
        /* Enable clock for CAN1, GPIOB */
        RCC->APB1ENR   |= (1 << 25);
        RCC->AHB1ENR   |= (1 <<  1);
        /* CAN1, use PB8, PB9 */
        GPIOB->MODER   &= ~(( 3 << ( 8 << 1)) | ( 3 << (9 << 1)));
        GPIOB->MODER   |=  (( 2 << ( 8 << 1)) | ( 2 << (9 << 1)));
        GPIOB->OTYPER  &= ~(( 1 <<   8)       | ( 1 <<   9      ));
        GPIOB->OSPEEDR &= ~(( 3 << ( 8 << 1)) | ( 3 << (9 << 1)));
        GPIOB->PUPDR   &= ~(( 3 << ( 8 << 1)) | ( 3 << (9 << 1)));
        GPIOB->AFR[1]  &= ~((15 << ( 0 << 2)) | (15 << ( 1 << 2)));
        GPIOB->AFR[1]  |=  (( 9 << ( 0 << 2)) | ( 9 << ( 1 << 2)));
        /* Enable CAN1 interrupts */
        NVIC_SetPriority (CAN1_TX_IRQn,  1);
        NVIC_SetPriority (CAN1_RX0_IRQn, 1);
        NVIC_EnableIRQ   (CAN1_TX_IRQn);
        NVIC_EnableIRQ   (CAN1_RX0_IRQn);
      break;
    case 2: 
        /* Enable clock for CAN2 and GPIOB */
        RCC->APB1ENR   |= (1 << 25) | (1 << 26);
        RCC->AHB1ENR   |= (1 <<  1);
        /* CAN2, use PB5, PB6 */
        GPIOB->MODER   &= ~(( 3 << ( 5 << 1)) | ( 3 << (6 << 1)));
        GPIOB->MODER   |=  (( 2 << ( 5 << 1)) | ( 2 << (6 << 1)));
        GPIOB->OTYPER  &= ~(( 1 <<   5)       | ( 1 <<  6      ));
        GPIOB->OSPEEDR &= ~(( 3 << ( 5 << 1)) | ( 3 << (6 << 1)));
        GPIOB->PUPDR   &= ~(( 3 << ( 5 << 1)) | ( 3 << (6 << 1)));
        GPIOB->AFR[0]  &= ~((15 << ( 5 << 2)) | (15 << ( 6 << 2)));
        GPIOB->AFR[0]  |=  (( 9 << ( 5 << 2)) | ( 9 << ( 6 << 2)));
        /* Enable CAN2 interrupts */
        NVIC_SetPriority (CAN2_TX_IRQn,  1);
        NVIC_SetPriority (CAN2_RX0_IRQn, 1);
        NVIC_EnableIRQ   (CAN2_TX_IRQn);
        NVIC_EnableIRQ   (CAN2_RX0_IRQn);
      break;
		case 3:MCP2515_Initialize();
			break;
  }
	return CAN_OK;
}

/*--------------------------- CAN_hw_init -----------------------------------
 *									 CAN硬件初始化 开启终端 设置波特率
 *  Initialize the CAN hardware
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_init (uint32_t ctrl, uint32_t baudrate)  {
	if(ctrl == 3)
	{
		if(CANSPI_Initialize(baudrate) == MCP2515_FALSE)			//CAN3初始化
		{
			return CAN_UNEXIST_CTRL_ERROR;
		}
	}
	else																										//CAN1-2初始化
	{
		CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];

		CANx->MCR = (1 << 0);               /* Init mode, with enabled automatic   */
																				/* retransmission only FIFO 0,         */
																				/* transmit mailbox 0 used             */
		while (!(CANx->MSR & (1 << 0)));
		CANx->IER = ((1 << 1) | (1 << 0));  /* FIFO 0 msg pending,                 */
																				/* Transmit mbx empty                  */

		CAN_hw_set_baudrate(ctrl, baudrate);         /* Set baudrate */
	}
	return CAN_OK;
}

/*--------------------------- CAN_hw_start ----------------------------------
 *													重置CAN初始化模式
 *  reset CAN initialisation mode
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_start (uint32_t ctrl)  {
	if(ctrl	 == 3)						//CAN3 开启外部中断
	{
			NVIC_InitTypeDef NVIC_InitStructure;
			/* 设置为普通模式 */
			if(!MCP2515_SetNormalMode())
				return CAN_NOT_IMPLEMENTED_ERROR;
			NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
			/* 配置子优先级：1 */
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			/* 使能中断通道 */
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			/* 清空中断标志 */
			MCP2515_BitModify(MCP2515_CANINTF, 0xFF, 0x00); 
			/* 接收数据时产生中断 */
			MCP2515_WriteByte(MCP2515_CANINTE, 0xE3);	
	}
	else
	{
		CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];
		uint32_t loop = 1000000;
		CANx->MCR |= (1 << 6);             	/* 一旦监测到 128 次连续 11 个隐性位，即通过硬件自动退出总线关闭状态   */
		CANx->MCR &= ~(1 << 0);             /* normal operating mode, reset INRQ   */
		while ((CANx->MSR & (1 << 0)) && loop--);
	}
  return CAN_OK;
	
}

/*--------------------------- CAN_hw_tx_empty -------------------------------
 *													检测发送邮箱0是否可用
 *  Check if transmit mailbox 0 is available for usage (empty)
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_hw_tx_empty (uint32_t ctrl)  {
	if(ctrl == 3)
	{
		ctrl_status_t ctrlStatus;
		ctrlStatus.ctrl_status = MCP2515_ReadStatus();
		if (ctrlStatus.TXB0REQ != 1 || ctrlStatus.TXB1REQ != 1 || ctrlStatus.TXB2REQ != 1) /* Transmit mailbox 0 is empty         */
			return CAN_OK;		
	}
	else
	{
		CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];
		if ((CANx->TSR & (1 << 26)) != 0) /* Transmit mailbox 0 is empty         */
			 return CAN_OK;
	}
  return CAN_TX_BUSY_ERROR;
}

/*--------------------------- CAN_hw_wr -------------------------------------
 *										将CAN信息写入硬件发送寄存器中
 *  Write CAN_msg to the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              msg:        Pointer to CAN message to be written to hardware
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

void CAN_hw_wr (uint32_t ctrl, CAN_msg *msg)  {
	if(ctrl == 3)
	{
			uCAN_MSG tempCanMsg;
			tempCanMsg.frame.id = msg->id;
			tempCanMsg.frame.data0 = msg->data[0];
			tempCanMsg.frame.data1 = msg->data[1];
			tempCanMsg.frame.data2 = msg->data[2];
			tempCanMsg.frame.data3 = msg->data[3];
			tempCanMsg.frame.data4 = msg->data[4];
			tempCanMsg.frame.data5 = msg->data[5];
			tempCanMsg.frame.data6 = msg->data[6];
			tempCanMsg.frame.data7 = msg->data[7];
			tempCanMsg.frame.dlc = msg->len;
			tempCanMsg.frame.idType = msg->format + 1;
			CANSPI_Transmit(&tempCanMsg);		
	}
	else
	{
		CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];

		/* Reset TIR register                                                      */
		CANx->sTxMailBox[0].TIR  = (uint32_t)0;                       /* reset TXRQ bit */

		/* Setup the identifier information                                        */
		if (msg->format == STANDARD_FORMAT)  {                   /*    Standard ID */
			CANx->sTxMailBox[0].TIR |= (uint32_t)(msg->id << 21);
		}  else  {                                               /* Extended ID    */
			CANx->sTxMailBox[0].TIR |= (uint32_t)(msg->id <<  3) | (1 << 2);
		}

		/* Setup type information                                                  */
		if (msg->type == REMOTE_FRAME)  {                        /* REMOTE FRAME   */
			CANx->sTxMailBox[0].TIR |= (1 << 1);
		}

		/* Setup data bytes                                                        */
		CANx->sTxMailBox[0].TDLR = (((uint32_t)msg->data[3] << 24) |
																((uint32_t)msg->data[2] << 16) |
																((uint32_t)msg->data[1] <<  8) |
																((uint32_t)msg->data[0])       );
		CANx->sTxMailBox[0].TDHR = (((uint32_t)msg->data[7] << 24) |
																((uint32_t)msg->data[6] << 16) |
																((uint32_t)msg->data[5] <<  8) |
																((uint32_t)msg->data[4])       );

		/* Setup length                                                            */
		CANx->sTxMailBox[0].TDTR &= ~0x0000000F;
		CANx->sTxMailBox[0].TDTR |=  (msg->len & 0x0000000F);

		CANx->IER |= (1 << 0);                           /*  enable  TME interrupt */ 

		/*  transmit message                                                       */
		CANx->sTxMailBox[0].TIR  |=  (1 << 0);                   /*   set TXRQ bit */
	}
}

/*--------------------------- CAN_hw_rd -------------------------------------
 *											从寄存器中	读取CAN数据
 *  Read CAN_msg from the hardware registers of the requested controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              ch:         Ignored
 *              msg:        Pointer where CAN message will be read
 *
 *  Return:     none
 *---------------------------------------------------------------------------*/

static void CAN_hw_rd (uint32_t ctrl, uint32_t ch, CAN_msg *msg)  {
	if(ctrl == 3)
	{
			uCAN_MSG tempCanMsg;
			CANSPI_Receive(&tempCanMsg);
			msg->id = tempCanMsg.frame.id;
			msg->data[0] = tempCanMsg.frame.data0;
			msg->data[1] = tempCanMsg.frame.data1;
			msg->data[2] = tempCanMsg.frame.data2;
			msg->data[3] = tempCanMsg.frame.data3;
			msg->data[4] = tempCanMsg.frame.data4;
			msg->data[5] = tempCanMsg.frame.data5;
			msg->data[6] = tempCanMsg.frame.data6;
			msg->data[7] = tempCanMsg.frame.data7;
			msg->type =  DATA_FRAME;
			msg->len = tempCanMsg.frame.dlc;		
	}
	else
	{
		CAN_TypeDef *CANx = CAN_CTRL[ctrl-1];

		/* Read identifier information                                             */
		if ((CANx->sFIFOMailBox[0].RIR & (1 << 2)) == 0) {        /* Standard ID   */
			msg->format = STANDARD_FORMAT;
			msg->id     = 0x000007FFUL & (CANx->sFIFOMailBox[0].RIR >> 21);
		}  else  {                                                /* Extended ID   */
			msg->format = EXTENDED_FORMAT;
			msg->id     = 0x1FFFFFFFUL & (CANx->sFIFOMailBox[0].RIR >> 3);
		}

		/* Read type information                                                   */
		if ((CANx->sFIFOMailBox[0].RIR & (1 << 1)) == 0) {
			msg->type =   DATA_FRAME;                               /* DATA   FRAME  */
		}  else  {
			msg->type = REMOTE_FRAME;                               /* REMOTE FRAME  */
		}

		/* Read length (number of received bytes)                                  */
		msg->len = (uint32_t)0x0000000F & CANx->sFIFOMailBox[0].RDTR;

		/* Read data bytes                                                         */
		msg->data[0] = (uint32_t)0x000000FF & (CANx->sFIFOMailBox[0].RDLR);
		msg->data[1] = (uint32_t)0x000000FF & (CANx->sFIFOMailBox[0].RDLR >> 8);
		msg->data[2] = (uint32_t)0x000000FF & (CANx->sFIFOMailBox[0].RDLR >> 16);
		msg->data[3] = (uint32_t)0x000000FF & (CANx->sFIFOMailBox[0].RDLR >> 24);

		msg->data[4] = (uint32_t)0x000000FF & (CANx->sFIFOMailBox[0].RDHR);
		msg->data[5] = (uint32_t)0x000000FF & (CANx->sFIFOMailBox[0].RDHR >> 8);
		msg->data[6] = (uint32_t)0x000000FF & (CANx->sFIFOMailBox[0].RDHR >> 16);
		msg->data[7] = (uint32_t)0x000000FF & (CANx->sFIFOMailBox[0].RDHR >> 24);
	}
}

/************************* Interrupt Functions *******************************/

/*--------------------------- CAN_IRQ_Handler -------------------------------
 *															  CAN中断
 *  CAN interrupt function 
 *  If transmit interrupt occured and there are messages in mailbox for 
 *  transmit it writes it to hardware and starts the transmission
 *  If receive interrupt occured it reads message from hardware registers 
 *  and puts it into receive mailbox
 *---------------------------------------------------------------------------*/

void CAN1_TX_IRQHandler (void) {
  if (CAN1->TSR & (1 << 0)) {                     /* request completed mbx 0 */
    CAN1->TSR |= (1 << 0);                   /* reset request complete mbx 0 */

    /* If there is a message in the mailbox ready for send, read the 
       message from the mailbox and send it                                  */
      CAN1->IER &= ~(1 << 0);                      /* disable  TME interrupt */ 
  }
}

void CAN1_RX0_IRQHandler (void) {
  CAN_msg ptrmsg;
  if (CAN1->RF0R & 3) {                                 /* message pending ? */
    /* If the mailbox isn't full read the message from the hardware and
       send it to the message queue                                          */
    CAN_hw_rd (__CTRL1, 0, &ptrmsg);             /* Read received message */
		ptrmsg.ch = 1;
    osMessageQueuePut(Q_CAN,&ptrmsg,osPriorityNormal,0);
    CAN1->RF0R |= (1 << 5);                 /* Release FIFO 0 output mailbox */
		irqSeq++;
  }
}

void CAN2_TX_IRQHandler (void) {
  if (CAN2->TSR & (1 << 0)) {                     /* request completed mbx 0 */
    CAN2->TSR |= (1 << 0);                   /* reset request complete mbx 0 */

    /* If there is a message in the mailbox ready for send, read the 
       message from the mailbox and send it                                  */
      CAN2->IER &= ~(1 << 0);                      /* disable  TME interrupt */ 
  }
}

void CAN2_RX0_IRQHandler (void) {
  CAN_msg ptrmsg;

  if (CAN2->RF0R & 3) {                                 /* message pending ? */
    /* If the mailbox isn't full read the message from the hardware and
       send it to the message queue                                          */
		CAN_hw_rd (__CTRL2, 0, &ptrmsg);             /* Read received message */
		ptrmsg.ch = 2;
		osMessageQueuePut(Q_CAN,&ptrmsg,osPriorityNormal,0);
    CAN2->RF0R |= (1 << 5);                 /* Release FIFO 0 output mailbox */
		irqSeq++;
  }
}

#define ISR_CAN3_STK_SZ (600)
uint64_t isr_can3_stk[ISR_CAN3_STK_SZ / 8];
const osThreadAttr_t isr_can3_attr = {.stack_mem  = &isr_can3_stk[0],.stack_size = sizeof(isr_can3_stk),.priority = osPriorityRealtime};
osThreadId_t isr_can3_id;
/* Forward references */
__NO_RETURN void isr_can3(void *arg);

__NO_RETURN void isr_can3(void *arg)
{
	CAN_msg ptrmsg;
	ctrl_rx_status_t rxStatus;
	uint8_t i;
	while(1)
	{
		osThreadFlagsWait(0x0001,osFlagsWaitAny,osWaitForever);//等待ISR触发一个事件
		rxStatus.ctrl_rx_status = MCP2515_GetRxStatus();
		i = 10;
		while(i-- > 0 && rxStatus.rxBuffer > 0)
		{
			CAN_hw_rd (__CTRL3, 0, &ptrmsg);             /* Read received message */
			if(ptrmsg.len > 0 && ptrmsg.len <= 8)
			{
				ptrmsg.ch = 3;
				osMessageQueuePut(Q_CAN,&ptrmsg,osPriorityNormal,0);
			}
			rxStatus.ctrl_rx_status = MCP2515_GetRxStatus();
		}
		MCP2515_BitModify(MCP2515_CANINTF, 0xFF, 0x00);
	}
}

void EXTI4_IRQHandler(void)
{
	//确保是否产生了EXTI Line中断
	if (EXTI_GetITStatus(EXTI_Line4) != RESET) 
	{
		osThreadFlagsSet(isr_can3_id,0x0001);
		//清除中断标志位
		EXTI_ClearITPendingBit(EXTI_Line4);
		irqSeq++;
	}
}

/*************************************************************************
* 函 数 名: setCANmask
* 功能说明: 设置CAN接收屏蔽码
* 参数说明：idx-过滤器组起始编号，id-CANID，mask-CANID屏蔽码 位定义0：不过滤 1：过滤，format-帧格式 0:标准帧 1:扩展帧
* 返 回 值: 无
**************************************************************************/
void setCANmask(uint8_t idx,uint32_t id,uint32_t mask,uint8_t format)
{
 CAN1->FMR |= 1 << 0;                          //过滤器组设置在初始化模式（记得必须设置初始化模式）
 CAN1->FA1R &= ~(0x01 << idx);                 //过滤器组禁用
 CAN1->FM1R &= ~(0x01<< idx);                  //过滤器组工作在标识符屏蔽位模式
 CAN1->FS1R |= 0x01<<idx;                      //过滤器组位宽为32位
 CAN1->FFA1R &= ~(0x01 << idx);                //关联到fifo0
 //全部接收
 if(id == 0 && mask == 0 && idx == 0)
 {
		CAN1->sFilterRegister[idx].FR1 = 0x00000000;
		CAN1->sFilterRegister[idx].FR2 = 0x00000000;	 
 }
 //全部禁止
 else if(id == 0 && mask == 0 && idx == 0)
 {
		CAN1->sFilterRegister[idx].FR1 = 0x00000000;
		CAN1->sFilterRegister[idx].FR2 = 0xFFFFFFFF;	 
 }
 //扩展帧         示例：setCANmask(idx++,0x18F88F9E,0xFFFF00FF,1);
 else if(format==1)
 {
  CAN1->sFilterRegister[idx].FR1 = id <<3   |0x04;      //CANID<<3|扩展帧，数据帧
  CAN1->sFilterRegister[idx].FR2 = mask <<3 |0x06;      //00表示不关心
 }
 //标准帧
 else
 {
  CAN1->sFilterRegister[idx].FR1 = id <<21   |0x00;     //CANID<<3|扩展帧，数据帧
  CAN1->sFilterRegister[idx].FR2 = mask <<21 |0x06;     //00表示不关心  
 }
 CAN1->FA1R |= 0x01 << idx;                      //激活过滤器组
 CAN1->FMR  = 0;
}

/*--------------------------- CAN_init --------------------------------------
 *
 *  The first time this function is called initialize the memory pool for 
 *  CAN messages and setup CAN controllers hardware
 *
 *  Initialize mailboxes for CAN messages and initialize CAN controller
 *
 *  Parameter:  ctrl:       Index of the hardware CAN controller (1 .. x)
 *              baudrate:   Baudrate
 *
 *  Return:     CAN_ERROR:  Error code
 *---------------------------------------------------------------------------*/

CAN_ERROR CAN_init (uint32_t ctrl, uint32_t baudrate)  {
  CAN_ERROR error_code;
  error_code = CAN_hw_setup (ctrl);
  if (error_code != CAN_OK) 
    return error_code;
  return (CAN_hw_init (ctrl, baudrate));
}

CAN_ERROR CAN_start (uint32_t ctrl)
{
	static uint8_t first_run_flag = 0;
  if (first_run_flag == 0)  {
    first_run_flag = 1;
		Q_CAN = osMessageQueueNew(100,sizeof(CAN_msg),&queueAttr_Q_CAN );					//create the message queue
		isr_can3_id = osThreadNew(isr_can3, NULL, &isr_can3_attr);
		//接收目标
		CAN1->FMR |= 1 << 0;
		CAN1->FA1R &= ~(1 << 0);
		CAN1->FM1R &= ~(1<< 0);
		CAN1->FS1R |= 1<<0;
		CAN1->FFA1R &= ~(1 << 0);
		CAN1->sFilterRegister[0].FR1 = 0X00000000;
		CAN1->sFilterRegister[0].FR2 = 0X00000000;
		CAN1->FA1R |= 1 << 0;
		CAN1->FMR &= 0 << 0;
  }
	return CAN_hw_start(ctrl);
}

CAN_ERROR CAN_send (uint32_t ctrl,CAN_msg* msg,uint32_t timeout)  {
	if(CAN_hw_tx_empty(ctrl) == CAN_OK)
		CAN_hw_wr(ctrl,msg);
	return CAN_OK;
}

CAN_ERROR CAN_receive (CAN_msg *msg, uint32_t timeout)
{
	uint8_t priority;
	if(osMessageQueueGet(Q_CAN,msg,&priority,timeout) == osOK)
	{
		return CAN_OK;
	}
	return CAN_TIMEOUT_ERROR;
}

uint8_t CAN_IrqSeq(void)
{
	return irqSeq;
}
