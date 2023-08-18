#ifndef STM32F4XX_EEPROM_H
#define STM32F4XX_EEPROM_H
#include "stm32f4xx.h" 

/* EEPROM emulation firmware error codes */
#define EE_OK      (uint32_t)FLASH_COMPLETE
#define EE_BUSY    (uint32_t)FLASH_BUSY

#define TYPEERASE_SECTORS    0x00000000U  /*!< Sectors erase only          */
#define FLASH_SECTOR_10       10U  /*!< Sector Number 10   */  
#define FLASH_SECTOR_11       11U  /*!< Sector Number 11   */

#define FLASH_VOLTAGE_RANGE_1        0x00000000U  /*!< Device operating range: 1.8V to 2.1V                */
#define FLASH_VOLTAGE_RANGE_2        0x00000001U  /*!< Device operating range: 2.1V to 2.7V                */
#define FLASH_VOLTAGE_RANGE_3        0x00000002U  /*!< Device operating range: 2.7V to 3.6V                */
#define VOLTAGE_RANGE_3      0x00000002U  /*!< Device operating range: 2.7V to 3.6V */

#define TYPEPROGRAM_HALFWORD 0x00000001U  /*!< Program a half-word (16-bit) at a specified address   */
#define FLASH_SAVE_ADDR  0X0800C004 	//

/* Define the size of the sectors to be used */
#define PAGE_SIZE               (uint32_t)0x20000  /* Page size = 128KByte */

/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
   be done by word  */
#define VOLTAGE_RANGE           (uint8_t)VOLTAGE_RANGE_3

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS  ((uint32_t)0x080C0000) /* EEPROM emulation start address:
                                                  from sector10 : after 128KByte of used
                                                  Flash memory */

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0000))
#define PAGE0_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))
#define PAGE0_ID               FLASH_SECTOR_10

#define PAGE1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x4000))
#define PAGE1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))
#define PAGE1_ID               FLASH_SECTOR_11

/* Used Flash pages for EEPROM emulation */
#define PAGE0                 ((uint16_t)0x0000)
#define PAGE1                 ((uint16_t)0x0001) /* Page nb between PAGE0_BASE_ADDRESS & PAGE1_BASE_ADDRESS*/

/* No valid page define */
#define NO_VALID_PAGE         ((uint16_t)0x00AB)

/* Page status definitions */
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty */
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data */
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL             ((uint8_t)0x80)

/**
  * @brief  HAL Lock structures definition
  */
typedef enum
{
    HAL_UNLOCKED = 0x00U,
    HAL_LOCKED = 0x01U
}HAL_LockTypeDef;

typedef struct
{
    uint32_t TypeErase;   /*!< Mass erase or sector Erase.
                               This parameter can be a value of @ref FLASHEx_Type_Erase */

    uint32_t Banks;       /*!< Select banks to erase when Mass erase is enabled.
                               This parameter must be a value of @ref FLASHEx_Banks */

    uint32_t Sector;      /*!< Initial FLASH sector to erase when Mass erase is disabled
                               This parameter must be a value of @ref FLASHEx_Sectors */

    uint32_t NbSectors;   /*!< Number of sectors to be erased.
                               This parameter must be a value between 1 and (max number of sectors - value of Initial sector)*/

    uint32_t VoltageRange;/*!< The device voltage range which defines the erase parallelism
                               This parameter must be a value of @ref FLASHEx_Voltage_Range */

}FLASH_EraseInitTypeDef;

/**
  * @brief  FLASH Procedure structure definition
  */
typedef enum
{
    FLASH_PROC_NONE = 0U,
    FLASH_PROC_SECTERASE,
    FLASH_PROC_MASSERASE,
    FLASH_PROC_PROGRAM
}FLASH_ProcedureTypeDef;

/**
  * @brief  FLASH handle Structure definition
  */
typedef struct
{
    __IO FLASH_ProcedureTypeDef ProcedureOnGoing;   /*Internal variable to indicate which procedure is ongoing or not in IT context*/

    __IO uint32_t               NbSectorsToErase;   /*Internal variable to save the remaining sectors to erase in IT context*/

    __IO uint8_t                VoltageForErase;    /*Internal variable to provide voltage range selected by user in IT context*/

    __IO uint32_t               Sector;             /*Internal variable to define the current sector which is erasing*/

    __IO uint32_t               Bank;               /*Internal variable to save current bank selected during mass erase*/

    __IO uint32_t               Address;            /*Internal variable to save address selected for program*/

    HAL_LockTypeDef             Lock;               /* FLASH locking object                */
    __IO uint32_t               ErrorCode;          /* FLASH error code                    */
}FLASH_ProcessTypeDef;
#define __HAL_LOCK(__HANDLE__)                                             \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       return FLASH_BUSY ;                 \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                }while (0U)
#define __HAL_UNLOCK(__HANDLE__)                                           \
                                  do{                                      \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;   \
                                  }while (0U)
/**
* @brief  Disable the FLASH instruction cache.
* @retval none
*/
#define __HAL_FLASH_INSTRUCTION_CACHE_DISABLE()   (FLASH->ACR &= (~FLASH_ACR_ICEN))
/**
  * @brief  Resets the FLASH instruction Cache.
  * @note   This function must be used only when the Instruction Cache is disabled.
  * @retval None
  */
#define __HAL_FLASH_INSTRUCTION_CACHE_RESET() do {FLASH->ACR |= FLASH_ACR_ICRST;  \
                                                  FLASH->ACR &= ~FLASH_ACR_ICRST; \
                                                 }while(0U)
/**
  * @brief  Enable the FLASH instruction cache.
  * @retval none
  */
#define __HAL_FLASH_INSTRUCTION_CACHE_ENABLE()  (FLASH->ACR |= FLASH_ACR_ICEN)

/**
  * @brief  Disable the FLASH data cache.
  * @retval none
  */
#define __HAL_FLASH_DATA_CACHE_DISABLE()   (FLASH->ACR &= (~FLASH_ACR_DCEN))
/**
* @brief  Resets the FLASH data Cache.
* @note   This function must be used only when the data Cache is disabled.
* @retval None
*/
#define __HAL_FLASH_DATA_CACHE_RESET() do {FLASH->ACR |= FLASH_ACR_DCRST;  \
                                           FLASH->ACR &= ~FLASH_ACR_DCRST; \
                                          }while(0U)
/**
  * @brief  Enable the FLASH data cache.
  * @retval none
  */
#define __HAL_FLASH_DATA_CACHE_ENABLE()  (FLASH->ACR |= FLASH_ACR_DCEN)

#define FLASH_TYPEERASE_SECTORS         0x00000000U  /*!< Sectors erase only          */
#define FLASH_TYPEERASE_MASSERASE       0x00000001U  /*!< Flash Mass erase activation */
#define FLASH_TYPEPROGRAM_BYTE          0x00000000U  /*!< Program byte (8-bit) at a specified address           */     
#define FLASH_TYPEPROGRAM_HALFWORD      0x00000001U  /*!< Program a half-word (16-bit) at a specified address   */
#define FLASH_TYPEPROGRAM_WORD          0x00000002U  /*!< Program a word (32-bit) at a specified address        */
#define FLASH_TYPEPROGRAM_DOUBLEWORD    0x00000003U  /*!< Program a double word (64-bit) at a specified address */
#define FLASH_MER_BIT                   (FLASH_CR_MER) /*!< only 1 MER Bit */
#define POSITION_VAL(VAL)               (__CLZ(__RBIT(VAL))) 
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void FLASH_FlushCaches(void);
FLASH_Status HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
FLASH_Status HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);

#define EEPROM_CELL_SZ          32                                                //数据空间分配的大小, Bytes
#define EEPROM_CELL_ADDR_LEN    2                                                 //2 Bytes
#define EEPROM_CELL_DATA_LEN    (EEPROM_CELL_SZ-EEPROM_CELL_ADDR_LEN)             //30 Bytes
#define EEPROM_SIZE             PAGE_SIZE                                         //eeprom容量: 128KB
#define EEPROM_CELL_NUM         (EEPROM_SIZE/EEPROM_CELL_SZ-1)                    //可存入数据单元的数量512-1, 第一个cell留作Page status definition
#define EEPROM_ADDR_MIN         0
#define EEPROM_ADDR_MAX         2046
#define NUM_OF_VAL_CELL         ((uint8_t)(EEPROM_ADDR_MAX-EEPROM_ADDR_MIN+1))//最多使用255个地址，每个地址对应一个有效CELL，最多NUM_OF_VAL_CELL个有效cell

//注释(20170723 by axia)：
//16KB容量EEPROM需要2个16KB flash sector模拟。模拟EEPROM的基本单元CELL为32Bytes(2B存索引地址+30B存数据)。
//EEPROM CELL个数为16K/32 = 512, 其中编号为0的CELL保留作为EEPROM操作标志，编号1~511共511个CELL作为存储单元使用
//EEPROM可用索引地址限制在0x00~0xFE共255个，每个地址存储的数据量不超过30Bytes。

uint16_t EE_Init(void);
FLASH_Status EE_Format(void);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t len, void *data);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t len, void *data);

#endif






















