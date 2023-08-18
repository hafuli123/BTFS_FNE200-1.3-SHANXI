#include "stm32f4xx_eeprom.h"

/**
 *  @file : stmflash
 *  @brief: stm32 flash 模拟 EEPROM
 *  @attention:
 *               （1）添加stmflash.h与stmflash.c
 *                (2) 将stm32f4xx_flash.h于stm32f4xx_flash.c替换
 *
 *  @process  : (1)  FLASH_Unlock();           //解除flash限制
 *              (2)  EE_Init()                  //初始化flash
 *              (3)  EE_WriteVariable(uint16_t VirtAddress, uint16_t Data)   //写操作
 *                   EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data)   //读操作
 *              (4)  FLASH_Lock();             //记得锁住，以免flash随意改动
 */
/* Global variable and functions */
extern void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);

/* Variable used for Erase sectors under interruption */
static FLASH_ProcessTypeDef pFlash;
static void   FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data);
static void   FLASH_Program_Word(uint32_t Address, uint32_t Data);
static void   FLASH_Program_HalfWord(uint32_t Address, uint16_t Data);
static void   FLASH_Program_Byte(uint32_t Address, uint8_t Data);

/**
  * @brief  Mass erase of FLASH memory
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.
  *          This parameter can be one of the following values:
  *            @arg FLASH_VOLTAGE_RANGE_1: when the device voltage range is 1.8V to 2.1V,
  *                                  the operation will be done by byte (8-bit)
  *            @arg FLASH_VOLTAGE_RANGE_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg FLASH_VOLTAGE_RANGE_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg FLASH_VOLTAGE_RANGE_4: when the device voltage range is 2.7V to 3.6V + External Vpp,
  *                                  the operation will be done by double word (64-bit)
  *
  * @param  Banks: Banks to be erased
  *          This parameter can be one of the following values:
  *            @arg FLASH_BANK_1: Bank1 to be erased
  *
  * @retval None
  */
static void FLASH_MassErase(uint8_t VoltageRange, uint32_t Banks)
{
    /* Check the parameters */
    assert_param(IS_VOLTAGERANGE(VoltageRange));
    assert_param(IS_FLASH_BANK(Banks));

    /* If the previous operation is completed, proceed to erase all sectors */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_CR_MER;
    FLASH->CR |= FLASH_CR_STRT | ((uint32_t)VoltageRange << 8U);
}
/**
 * @brief  Perform a mass erase or erase the specified FLASH memory sectors
 * @param[in]  pEraseInit: pointer to an FLASH_EraseInitTypeDef structure that
 *         contains the configuration information for the erasing.
 *
 * @param[out]  SectorError: pointer to variable  that
 *         contains the configuration information on faulty sector in case of error
 *         (0xFFFFFFFFU means that all the sectors have been correctly erased)
 *
 * @retval HAL Status
 */
FLASH_Status HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError)
{
    FLASH_Status status = FLASH_ERROR_RD;

    uint32_t index = 0U;

    /* Process Locked */
    __HAL_LOCK(&pFlash);

    /* Check the parameters */
    assert_param(IS_FLASH_TYPEERASE(pEraseInit->TypeErase));

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();  //这个采用标准库的写法

    if (status == FLASH_COMPLETE)
    {
        /*Initialization of SectorError variable*/
        *SectorError = 0xFFFFFFFFU;

        if (pEraseInit->TypeErase == FLASH_TYPEERASE_MASSERASE)
        {
            /*Mass erase to be done*/
            FLASH_MassErase((uint8_t)pEraseInit->VoltageRange, pEraseInit->Banks);

            /* Wait for last operation to be completed */
            status = FLASH_WaitForLastOperation(); //这个采用标准库的写法

            /* if the erase operation is completed, disable the MER Bit */
            FLASH->CR &= (~FLASH_MER_BIT);
        }
        else
        {
            /* Check the parameters */
            assert_param(IS_FLASH_NBSECTORS(pEraseInit->NbSectors + pEraseInit->Sector));

            /* Erase by sector by sector to be done*/
            for (index = pEraseInit->Sector; index < (pEraseInit->NbSectors + pEraseInit->Sector); index++)
            {
                FLASH_Erase_Sector(index, (uint8_t)pEraseInit->VoltageRange);

                /* Wait for last operation to be completed */
                status = FLASH_WaitForLastOperation();  //这个采用标准库的写法

                /* If the erase operation is completed, disable the SER and SNB Bits */
                CLEAR_BIT(FLASH->CR, (FLASH_CR_SER | FLASH_CR_SNB));

                if (status != FLASH_COMPLETE)
                {
                    /* In case of error, stop erase procedure and return the faulty sector*/
                    *SectorError = index;
                    break;
                }
            }
        }
        /* Flush the caches to be sure of the data consistency */
        FLASH_FlushCaches();
    }

    /* Process Unlocked */
    __HAL_UNLOCK(&pFlash);

    return status;
}

/**
  * @brief  Program byte, halfword, word or double word at a specified address
  * @param  TypeProgram:  Indicate the way to program at a specified address.
  *                           This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address:  specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed
  *
  * @retval HAL_StatusTypeDef HAL Status
  */
FLASH_Status HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
{
    FLASH_Status status = FLASH_ERROR_RD;
    //HAL_StatusTypeDef statu = FLASH_COMPLETE;
    /* Process Locked */
    __HAL_LOCK(&pFlash);

    /* Check the parameters */
    assert_param(IS_FLASH_TYPEPROGRAM(TypeProgram));

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    if (status == FLASH_COMPLETE)
    {
        if (TypeProgram == FLASH_TYPEPROGRAM_BYTE)
        {
            /*Program byte (8-bit) at a specified address.*/
            FLASH_Program_Byte(Address, (uint8_t)Data);
        }
        else if (TypeProgram == FLASH_TYPEPROGRAM_HALFWORD)
        {
            /*Program halfword (16-bit) at a specified address.*/
            FLASH_Program_HalfWord(Address, (uint16_t)Data);
        }
        else if (TypeProgram == FLASH_TYPEPROGRAM_WORD)
        {
            /*Program word (32-bit) at a specified address.*/
            FLASH_Program_Word(Address, (uint32_t)Data);
        }
        else
        {
            /*Program double word (64-bit) at a specified address.*/
            FLASH_Program_DoubleWord(Address, Data);
        }

        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation();

        /* If the program operation is completed, disable the PG Bit */
        FLASH->CR &= (~FLASH_CR_PG);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(&pFlash);

    return status;
}
/**
  * @brief  Flush the instruction and data caches
  * @retval None
  */
void FLASH_FlushCaches(void)
{
    /* Flush instruction cache  */
    if (READ_BIT(FLASH->ACR, FLASH_ACR_ICEN) != RESET)
    {
        /* Disable instruction cache  */
        __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
        /* Reset instruction cache */
        __HAL_FLASH_INSTRUCTION_CACHE_RESET();
        /* Enable instruction cache */
        __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    }
    /* Flush data cache */
    if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != RESET)
    {
        /* Disable data cache  */
        __HAL_FLASH_DATA_CACHE_DISABLE();
        /* Reset data cache */
        __HAL_FLASH_DATA_CACHE_RESET();
        /* Enable data cache */
        __HAL_FLASH_DATA_CACHE_ENABLE();
    }
}

/**
  * @brief  Erase the specified FLASH memory sector
  * @param  Sector: FLASH sector to erase
  *         The value of this parameter depend on device used within the same series
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.
  *          This parameter can be one of the following values:
  *            @arg FLASH_VOLTAGE_RANGE_1: when the device voltage range is 1.8V to 2.1V,
  *                                  the operation will be done by byte (8-bit)
  *            @arg FLASH_VOLTAGE_RANGE_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg FLASH_VOLTAGE_RANGE_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg FLASH_VOLTAGE_RANGE_4: when the device voltage range is 2.7V to 3.6V + External Vpp,
  *                                  the operation will be done by double word (64-bit)
  *
  * @retval None
  */
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange)
{
    uint32_t tmp_psize = 0U;

    /* Check the parameters */
    assert_param(IS_FLASH_SECTOR(Sector));
    assert_param(IS_VOLTAGERANGE(VoltageRange));

    if (VoltageRange == FLASH_VOLTAGE_RANGE_1)
    {
        tmp_psize = FLASH_PSIZE_BYTE;
    }
    else if (VoltageRange == FLASH_VOLTAGE_RANGE_2)
    {
        tmp_psize = FLASH_PSIZE_HALF_WORD;
    }
    else if (VoltageRange == FLASH_VOLTAGE_RANGE_3)
    {
        tmp_psize = FLASH_PSIZE_WORD;
    }
    else
    {
        tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
    }

    /* If the previous operation is completed, proceed to erase the sector */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
    FLASH->CR |= tmp_psize;
    CLEAR_BIT(FLASH->CR, FLASH_CR_SNB);
    FLASH->CR |= FLASH_CR_SER | (Sector << POSITION_VAL(FLASH_CR_SNB));
    FLASH->CR |= FLASH_CR_STRT;
}

/**
  * @brief  Program a double word (64-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V and Vpp in the range 7V to 9V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data)
{
    /* Check the parameters */
    assert_param(IS_FLASH_ADDRESS(Address));

    /* If the previous operation is completed, proceed to program the new data */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_PSIZE_DOUBLE_WORD;
    FLASH->CR |= FLASH_CR_PG;

    /* Program the double-word */
    *(__IO uint32_t*)Address = (uint32_t)Data;
    *(__IO uint32_t*)(Address + 4) = (uint32_t)(Data >> 32);
}


/**
  * @brief  Program word (32-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_Word(uint32_t Address, uint32_t Data)
{
    /* Check the parameters */
    assert_param(IS_FLASH_ADDRESS(Address));

    /* If the previous operation is completed, proceed to program the new data */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_PSIZE_WORD;
    FLASH->CR |= FLASH_CR_PG;

    *(__IO uint32_t*)Address = Data;
}

/**
  * @brief  Program a half-word (16-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.1V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_HalfWord(uint32_t Address, uint16_t Data)
{
    /* Check the parameters */
    assert_param(IS_FLASH_ADDRESS(Address));

    /* If the previous operation is completed, proceed to program the new data */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_PSIZE_HALF_WORD;
    FLASH->CR |= FLASH_CR_PG;

    *(__IO uint16_t*)Address = Data;
}

/**
  * @brief  Program byte (8-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         1.8V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_Byte(uint32_t Address, uint8_t Data)
{
    /* Check the parameters */
    assert_param(IS_FLASH_ADDRESS(Address));

    /* If the previous operation is completed, proceed to program the new data */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_PSIZE_BYTE;
    FLASH->CR |= FLASH_CR_PG;

    *(__IO uint8_t*)Address = Data;
}




