#include <stdint.h>

//#include "stm32f4xx_hal.h"

#include "Driver_USBH.h"
#include "Driver_USBD.h"

#include "OTG_FS_STM32F4xx.h"


uint8_t otg_fs_role = ARM_USB_ROLE_NONE;


// Local Functions *************************************************************


// Common IRQ Routine **********************************************************

/**
  \fn          void OTG_FS_IRQHandler (void)
  \brief       USB Interrupt Routine (IRQ).
*/
void OTG_FS_IRQHandler (void) {
  uint32_t gintsts;
  gintsts = USB_OTG_FS->GINTSTS & USB_OTG_FS->GINTMSK;
  USBD_FS_IRQ (gintsts);
}


// Public Functions ************************************************************

/**						 初始化
  \fn          void OTG_FS_PinsConfigure (uint8_t pins_mask)
  \brief       Configure single or multiple USB Pin(s).
  \param[in]   Mask of pins to be configured (possible masking values:
               ARM_USB_PIN_DP, ARM_USB_PIN_DM, ARM_USB_PIN_VBUS,
               ARM_USB_PIN_OC, ARM_USB_PIN_ID)
*/
void OTG_FS_PinsConfigure (uint8_t pins_mask) {
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);			//使能GPIOA时钟
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE);		//使能USB OTG时钟	钟
  //GPIOA11,A12设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;	//PA11/12复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;							//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;				//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);										//初始化
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_OTG_FS);	//PA11,AF10(USB)
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_OTG_FS);	//PA12,AF10(USB)
}

/**						 反初始化
  \fn          void OTG_FS_PinsUnconfigure (uint8_t pins_mask)
  \brief       De-configure to reset settings single or multiple USB Pin(s).
  \param[in]   Mask of pins to be de-configured (possible masking values:
               ARM_USB_PIN_DP, ARM_USB_PIN_DM, ARM_USB_PIN_VBUS,
               ARM_USB_PIN_OC, ARM_USB_PIN_ID)
*/
void OTG_FS_PinsUnconfigure (uint8_t pins_mask) {
	;
}

/**
  \fn          void OTG_FS_PinVbusOnOff (bool state)
  \brief       Drive VBUS Pin On/Off.
  \param[in]   state    State On/Off (true = On, false = Off)
*/
void OTG_FS_PinVbusOnOff (bool state) {
  (void)state;
}

/**
  \fn          bool OTG_FS_PinGetOC (void)
  \brief       Get state of Overcurrent Pin.
  \return      overcurrent state (true = Overcurrent active, false = No overcurrent)
*/
bool OTG_FS_PinGetOC (void) {
  return false;
}
