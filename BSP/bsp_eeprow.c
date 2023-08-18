#include "bsp_eeprow.h"
#include "stm32f4xx_flash.h"

/* Global variable used to store variable value in read sequence */
static uint8_t DatWorkBuf[EEPROM_CELL_DATA_LEN];//data work buffer

/* Variable used for Erase sectors under interruption */
static uint16_t EE_FindValidPage(uint8_t Operation);
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, void *data, uint16_t len);
static uint16_t EE_PageTransfer(uint16_t VirtAddress, void *data, uint16_t Len);
static uint16_t EE_VerifyPageFullyErased(uint32_t Address);
static FLASH_Status EE_Format(void);

/**
 * @brief  Restore the pages to a known good state in case of page's status
 *   corruption after a power loss.
 * @param  None.
 * @retval - Flash error code: on write Flash error
 *         - FLASH_COMPLETE: on success
 */
uint16_t EE_Init1(void)
{
    uint16_t PageStatus0 = 6, PageStatus1 = 6;
    uint16_t VarIdx = 0;
    uint16_t EepromStatus = 0, ReadStatus = 0;
    int16_t x0 = -1;
    FLASH_Status  FLASHStatus;
		uint32_t FLASH_Sector = PAGE0_ID;

    /* Get Page0 status */
    PageStatus0 = (*(__IO uint16_t*)PAGE0_BASE_ADDRESS);
    /* Get Page1 status */
    PageStatus1 = (*(__IO uint16_t*)PAGE1_BASE_ADDRESS);
	
    /* Check for invalid header states and repair if necessary */
    switch (PageStatus0)
    {
        case ERASED:
            if (PageStatus1 == VALID_PAGE) /* Page0 erased, Page1 valid */
            {
								FLASH_Sector = PAGE0_ID;
                /* Erase Page0 */
                if (!EE_VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
                {
                    FLASHStatus = FLASH_EraseSector(FLASH_Sector,VoltageRange_3);
                    /* If erase operation was failed, a Flash error code is returned */
                    if (FLASHStatus != FLASH_COMPLETE)
                    {
                        return FLASHStatus;
                    }
                }
            }
            else if (PageStatus1 == RECEIVE_DATA) /* Page0 erased, Page1 receive */
            {
								FLASH_Sector = PAGE0_ID;
                /* Erase Page0 */
                if (!EE_VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
                {
                    FLASHStatus = FLASH_EraseSector(FLASH_Sector,VoltageRange_3);
                    /* If erase operation was failed, a Flash error code is returned */
                    if (FLASHStatus != FLASH_COMPLETE)
                    {
                        return FLASHStatus;
                    }
                }
                /* Mark Page1 as valid */
								FLASHStatus = FLASH_ProgramHalfWord(PAGE1_BASE_ADDRESS, VALID_PAGE);
                /* If program operation was failed, a Flash error code is returned */
                if (FLASHStatus != FLASH_COMPLETE)
                {
                    return FLASHStatus;
                }
            }
            else /* First EEPROM access (Page0&1 are erased) or invalid state -> format EEPROM */
            {
                /* Erase both Page0 and Page1 and set Page0 as valid page */
                FLASHStatus = EE_Format();
                /* If erase/program operation was failed, a Flash error code is returned */
                if (FLASHStatus != FLASH_COMPLETE)
                {
                    return FLASHStatus;
                }
            }
            break;
        case RECEIVE_DATA:
            if (PageStatus1 == VALID_PAGE) /* Page0 receive, Page1 valid */
            {
                /* Transfer data from Page1 to Page0 */
                //only data in 2nd cell of page0 is neweset
                x0 = (*(__IO uint16_t*)(PAGE0_BASE_ADDRESS + EEPROM_CELL_SZ * 2 - EEPROM_CELL_ADDR_LEN));
                for (VarIdx = 0; VarIdx < NUM_OF_VAL_CELL; VarIdx++)
                {
                    if (VarIdx != x0)
                    {
                        /* Read the last variables' updates */
                        ReadStatus = EE_ReadVariable1(VarIdx, EEPROM_CELL_DATA_LEN, DatWorkBuf);
                        /* In case variable corresponding to the virtual address was found */
                        if (ReadStatus != 0x1)
                        {
                            /* Transfer the variable to the Page0 */
                            EepromStatus = EE_VerifyPageFullWriteVariable(VarIdx, DatWorkBuf, EEPROM_CELL_DATA_LEN);
                            /* If program operation was failed, a Flash error code is returned */
                            if (EepromStatus != FLASH_COMPLETE)
                            {
                                return EepromStatus;
                            }
                        }
                    }
                }
                /* Mark Page0 as valid */
								FLASHStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
                /* If program operation was failed, a Flash error code is returned */
                if (FLASHStatus != FLASH_COMPLETE)
                {
                    return FLASHStatus;
                }
                FLASH_Sector = PAGE1_ID;
                /* Erase Page1 */
                if (!EE_VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
                {
										FLASHStatus = FLASH_EraseSector(FLASH_Sector,VoltageRange_3);									
                    /* If erase operation was failed, a Flash error code is returned */
                    if (FLASHStatus != FLASH_COMPLETE)
                    {
                        return FLASHStatus;
                    }
                }
            }
            else if (PageStatus1 == ERASED) /* Page0 receive, Page1 erased */
            {
								FLASH_Sector = PAGE1_ID;
                /* Erase Page1 */
                if (!EE_VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
                {
                    FLASHStatus = FLASH_EraseSector(FLASH_Sector,VoltageRange_3);				
                    /* If erase operation was failed, a Flash error code is returned */
                    if (FLASHStatus != FLASH_COMPLETE)
                    {
                        return FLASHStatus;
                    }
                }
                /* Mark Page0 as valid */
                FLASHStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
                /* If program operation was failed, a Flash error code is returned */
                if (FLASHStatus != FLASH_COMPLETE)
                {
                    return FLASHStatus;
                }
            }
            else /* Invalid state -> format eeprom */
            {
                /* Erase both Page0 and Page1 and set Page0 as valid page */
                FLASHStatus = EE_Format();
                /* If erase/program operation was failed, a Flash error code is returned */
                if (FLASHStatus != FLASH_COMPLETE)
                {
                    return FLASHStatus;
                }
            }
            break;
        case VALID_PAGE:
            if (PageStatus1 == VALID_PAGE) /* Invalid state -> format eeprom */
            {
                /* Erase both Page0 and Page1 and set Page0 as valid page */
                FLASHStatus = EE_Format();
                /* If erase/program operation was failed, a Flash error code is returned */
                if (FLASHStatus != FLASH_COMPLETE)
                {
                    return FLASHStatus;
                }
            }
            else if (PageStatus1 == ERASED) /* Page0 valid, Page1 erased */
            {
                FLASH_Sector = PAGE1_ID;
                /* Erase Page1 */
                if (!EE_VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
                {
                    FLASHStatus = FLASH_EraseSector(FLASH_Sector,VoltageRange_3);	
                    /* If erase operation was failed, a Flash error code is returned */
                    if (FLASHStatus != FLASH_COMPLETE)
                    {
                        return FLASHStatus;
                    }
                }
            }
            else /* Page0 valid, Page1 receive */
            {
                /* Transfer data from Page0 to Page1 */
                //only data in 2nd cell of page1 is neweset
                x0 = (*(__IO uint16_t*)(PAGE1_BASE_ADDRESS + EEPROM_CELL_SZ * 2 - EEPROM_CELL_ADDR_LEN));
                for (VarIdx = 0; VarIdx < NUM_OF_VAL_CELL; VarIdx++)
                {
                    if (VarIdx != x0)
                    {
                        /* Read the last variables' updates */
                        ReadStatus = EE_ReadVariable1(VarIdx, EEPROM_CELL_DATA_LEN, DatWorkBuf);
                        /* In case variable corresponding to the virtual address was found */
                        if (ReadStatus != 0x1)
                        {
                            /* Transfer the variable to the Page1 */
                            EepromStatus = EE_VerifyPageFullWriteVariable(VarIdx, DatWorkBuf, EEPROM_CELL_DATA_LEN);
                            /* If program operation was failed, a Flash error code is returned */
                            if (EepromStatus != FLASH_COMPLETE)
                            {
                                return EepromStatus;
                            }
                        }
                    }
                }
                /* Mark Page1 as valid */
								FLASHStatus = FLASH_ProgramHalfWord(PAGE1_BASE_ADDRESS, VALID_PAGE);
                /* If program operation was failed, a Flash error code is returned */
                if (FLASHStatus != FLASH_COMPLETE)
                {
                    return FLASHStatus;
                }
                FLASH_Sector = PAGE0_ID;
                /* Erase Page0 */
                if (!EE_VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
                {
                    FLASHStatus = FLASH_EraseSector(FLASH_Sector,VoltageRange_3);
                    /* If erase operation was failed, a Flash error code is returned */
                    if (FLASHStatus != FLASH_COMPLETE)
                    {
                        return FLASHStatus;
                    }
                }
            }
            break;
        default:  /* Any other state -> format eeprom */
            /* Erase both Page0 and Page1 and set Page0 as valid page */
            FLASHStatus = EE_Format();
            /* If erase/program operation was failed, a Flash error code is returned */
            if (FLASHStatus != FLASH_COMPLETE)
            {
                return FLASHStatus;
            }
            break;
    }
    return FLASH_COMPLETE;
}
/**

  * @brief  Verify if specified page is fully erased.
  * @param  Address: page address
  *   This parameter can be one of the following values:
  *     @arg PAGE0_BASE_ADDRESS: Page0 base address
  *     @arg PAGE1_BASE_ADDRESS: Page1 base address
  * @retval page fully erased status:
  *           - 0: if Page not erased
  *           - 1: if Page erased
  */
uint16_t EE_VerifyPageFullyErased(uint32_t Address)
{
    uint32_t ReadStatus = 1;
    uint16_t AddressValue = 0x5555;

    /* Check each active page address starting from end */
    if (Address < PAGE0_END_ADDRESS)
    {
        while (Address <= PAGE0_END_ADDRESS)
        {
            /* Get the current location content to be compared with virtual address */
            AddressValue = (*(__IO uint16_t*)Address);

            /* Compare the read address with the virtual address */
            if (AddressValue != ERASED)
            {
                /* In case variable value is read, reset ReadStatus flag */
                ReadStatus = 0;
                break;
            }
            /* Next address location */
            Address = Address + 2;//verify every bytes
        }
    }
    else
    {
        while (Address <= PAGE1_END_ADDRESS)
        {
            /* Get the current location content to be compared with virtual address */
            AddressValue = (*(__IO uint16_t*)Address);

            /* Compare the read address with the virtual address */
            if (AddressValue != ERASED)
            {
                /* In case variable value is read, reset ReadStatus flag */
                ReadStatus = 0;
                break;
            }
            /* Next address location */
            Address = Address + 2;//verify every bytes
        }
    }

    /* Return ReadStatus value: (0: Page not erased, 1: Sector erased) */
    return ReadStatus;
}

/**
  * @brief  Returns the last stored variable data, if found, which correspond to
  *   the passed virtual address
  * @param  VirtAddress: Variable virtual address
  * @param  Data: Global variable contains the read variable value
  * @retval Success or error status:
  *           - 0: if variable was found
  *           - 1: if the variable was not found
  *           - NO_VALID_PAGE: if no valid page was found.
  */
uint16_t EE_ReadVariable1(uint16_t VirtAddress, uint16_t len, void *data)
{
    uint16_t ValidPage = PAGE0;
    uint16_t i = 0;
    uint16_t AddressValue = 0x5555, ReadStatus = FLASH_ERROR_RD;
    uint32_t Address = EEPROM_START_ADDRESS, PageStartAddress = EEPROM_START_ADDRESS;
    uint16_t *Data = (uint16_t *)data;
    len = (len + 1) >> 1;
    /* Get active Page for read operation */
    ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

    /* Check if there is no valid page */
    if (ValidPage == NO_VALID_PAGE)
    {
        return  NO_VALID_PAGE;
    }

    /* Get the valid Page start Address */
    PageStartAddress = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(ValidPage * PAGE_SIZE));

    /* Get the valid Page end Address */
    Address = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)((1 + ValidPage) * PAGE_SIZE) - EEPROM_CELL_ADDR_LEN);   //页最后两个字节为地址

    /* Check each active page address starting from end */
    while (Address >= (PageStartAddress + EEPROM_CELL_SZ))                   //第一个cell的flash地址空间不算
    {
        /* Get the current location content to be compared with virtual address */
        AddressValue = (*(__IO uint16_t*)Address);

        /* Compare the read address with the virtual address */
        if (AddressValue == VirtAddress)
        {
            //从cell的起始地址读数据
            for (i = 0; i < len; i++)
            {
                *Data++ = (*(__IO uint16_t*)(Address - EEPROM_CELL_DATA_LEN + i * 2));  //一个Data 两个字节，应该乘以2
            }
            /* In case variable value is read, reset ReadStatus flag */
            ReadStatus = FLASH_COMPLETE;
            break;
        }
        else
        {
            /* Next address location */
            Address = Address - EEPROM_CELL_SZ;
        }
    }

    /* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
    return ReadStatus;
}

/**
  * @brief  Writes/upadtes variable data in EEPROM.
  * @param  VirtAddress: Variable virtual address
  * @param  Data: 16 bit data to be written
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
uint16_t EE_WriteVariable1(uint16_t VirtAddress, uint16_t len, void *Data)
{
    uint16_t Status = 0;

    /* Write the variable virtual address and value in the EEPROM */
    Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data, len);

    /* In case the EEPROM active page is full */
    if (Status == PAGE_FULL)
    {
        /* Perform Page transfer */
        Status = EE_PageTransfer(VirtAddress, Data, len);        //替换成多字节转移
    }

    /* Return last operation status */
    return Status;

}

/**
  * @brief  Erases PAGE and PAGE1 and writes VALID_PAGE header to PAGE
  * @param  None
  * @retval Status of the last operation (Flash write or erase) done during
  *         EEPROM formating
  */
FLASH_Status EE_Format(void)
{
    FLASH_Status FLASHStatus = FLASH_COMPLETE;
		uint32_t FLASH_Sector = PAGE0_ID;
    /* Erase Page0 */
    if (!EE_VerifyPageFullyErased(PAGE0_BASE_ADDRESS))
    {
        FLASHStatus = FLASH_EraseSector(FLASH_Sector,VoltageRange_3);
        /* If erase operation was failed, a Flash error code is returned */
        if (FLASHStatus != FLASH_COMPLETE)
        {
            return FLASHStatus;
        }
    }

    /* Set Page0 as valid page: Write VALID_PAGE at Page0 base address */
		FLASHStatus = FLASH_ProgramHalfWord(PAGE0_BASE_ADDRESS, VALID_PAGE);
    /* If program operation was failed, a Flash error code is returned */
    if (FLASHStatus != FLASH_COMPLETE)
    {
        return FLASHStatus;
    }

    FLASH_Sector = PAGE1_ID;
    /* Erase Page1 */
    if (!EE_VerifyPageFullyErased(PAGE1_BASE_ADDRESS))
    {
        FLASHStatus = FLASH_EraseSector(FLASH_Sector,VoltageRange_3);
        /* If erase operation was failed, a Flash error code is returned */
        if (FLASHStatus != FLASH_COMPLETE)
        {
            return FLASHStatus;
        }
    }
    return FLASH_COMPLETE;
}


/**
  * @brief  Find valid Page for write or read operation
  * @param  Operation: operation to achieve on the valid page.
  *   This parameter can be one of the following values:
  *     @arg READ_FROM_VALID_PAGE: read operation from valid page
  *     @arg WRITE_IN_VALID_PAGE: write operation from valid page
  * @retval Valid page number (PAGE or PAGE1) or NO_VALID_PAGE in case
  *   of no valid page was found
  */
static uint16_t EE_FindValidPage(uint8_t Operation)
{
    uint16_t PageStatus0 = 6, PageStatus1 = 6;

    /* Get Page0 actual status */
    PageStatus0 = (*(__IO uint16_t*)PAGE0_BASE_ADDRESS);

    /* Get Page1 actual status */
    PageStatus1 = (*(__IO uint16_t*)PAGE1_BASE_ADDRESS);

    /* Write or read operation */
    switch (Operation)
    {
        case WRITE_IN_VALID_PAGE:   /* ---- Write operation ---- */
            if (PageStatus1 == VALID_PAGE)
            {
                /* Page0 receiving data */
                if (PageStatus0 == RECEIVE_DATA)
                {
                    return PAGE0;         /* Page0 valid */
                }
                else
                {
                    return PAGE1;         /* Page1 valid */
                }
            }
            else if (PageStatus0 == VALID_PAGE)
            {
                /* Page1 receiving data */
                if (PageStatus1 == RECEIVE_DATA)
                {
                    return PAGE1;         /* Page1 valid */
                }
                else
                {
                    return PAGE0;         /* Page0 valid */
                }
            }
            else
            {
                return NO_VALID_PAGE;   /* No valid Page */
            }

        case READ_FROM_VALID_PAGE:  /* ---- Read operation ---- */
            if (PageStatus0 == VALID_PAGE)
            {
                return PAGE0;           /* Page0 valid */
            }
            else if (PageStatus1 == VALID_PAGE)
            {
                return PAGE1;           /* Page1 valid */
            }
            else
            {
                return NO_VALID_PAGE;  /* No valid Page */
            }

        default:
            return PAGE0;             /* Page0 valid */
    }
}

/**
  * @brief  Verify if active page is full and Writes variable in EEPROM.
  * @param  VirtAddress: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, void *data, uint16_t len)
{
    FLASH_Status FLASHStatus = FLASH_COMPLETE;
    uint16_t ValidPage = PAGE0;
    uint32_t Address = EEPROM_START_ADDRESS, PageEndAddress = EEPROM_START_ADDRESS + PAGE_SIZE;
    uint16_t   i = 0;
    uint16_t *Data = data;
    len = (len + 1) >> 1;

    /* Get valid Page for write operation */
    ValidPage = EE_FindValidPage(WRITE_IN_VALID_PAGE);

    /* Check if there is no valid page */
    if (ValidPage == NO_VALID_PAGE)
    {
        return  NO_VALID_PAGE;
    }

    /* Get the valid Page start Address */
    Address = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(ValidPage * PAGE_SIZE));

    /* Get the valid Page end Address */
    PageEndAddress = (uint32_t)((EEPROM_START_ADDRESS - 1) + (uint32_t)((ValidPage + 1) * PAGE_SIZE));

    /* Check each active page address starting from begining */
    while (Address < PageEndAddress)
    {
        /* Verify if first and last 4 Bytes contents of a cell are 0xFFFFFFFF */
        if (((*(__IO uint32_t*)Address) == 0xFFFFFFFF)
            && ((*(__IO uint32_t*)(Address + EEPROM_CELL_SZ-4)) == 0xFFFFFFFF))
        {
            /* Set variable data */
            for (i = 0; i < len; i++)                                                                  //改成循环写入，总共写入len字节
            {
								FLASHStatus = FLASH_ProgramHalfWord(Address + i * 2, Data[i]);
                /* If program operation was failed, a Flash error code is returned */
                if (FLASHStatus != FLASH_COMPLETE)
                {
                    return FLASHStatus;
                }
            }
            /* Set variable virtual address */
						FLASHStatus = FLASH_ProgramHalfWord(Address + EEPROM_CELL_DATA_LEN, VirtAddress);
            /* Return program operation status */
            if (FLASHStatus != FLASH_COMPLETE)
            {
                return FLASHStatus;
            }
            else
            {
                return FLASH_COMPLETE;
            }
        }
        else
        {
            /* Next address location */
            Address = Address + EEPROM_CELL_SZ;                         //*******************************************
        }
    }
    /* Return PAGE_FULL in case the valid page is full */
    return PAGE_FULL;
}

/**
  * @brief  Transfers last updated variables data from the full Page to
  *   an empty one.
  * @param  VirtAddress: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
static uint16_t EE_PageTransfer(uint16_t VirtAddress, void *Data, uint16_t len)
{
    FLASH_Status FLASHStatus = FLASH_COMPLETE;
    uint32_t NewPageAddress = EEPROM_START_ADDRESS;
    uint16_t OldPageId = 0;
    uint16_t ValidPage = PAGE0, VarIdx = 0;
    uint16_t EepromStatus = FLASH_ERROR_RD, ReadStatus = FLASH_ERROR_RD;
		uint32_t FLASH_Sector = PAGE0_ID;

    /* Get active Page for read operation */
    ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

    if (ValidPage == PAGE1)       /* Page1 valid */
    {
        /* New page address where variable will be moved to */
        NewPageAddress = PAGE0_BASE_ADDRESS;

        /* Old page ID where variable will be taken from */
        OldPageId = PAGE1_ID;
    }
    else if (ValidPage == PAGE0)  /* Page0 valid */
    {
        /* New page address  where variable will be moved to */
        NewPageAddress = PAGE1_BASE_ADDRESS;

        /* Old page ID where variable will be taken from */
        OldPageId = PAGE0_ID;
    }
    else
    {
        return NO_VALID_PAGE;       /* No valid Page */
    }

    /* Set the new Page status to RECEIVE_DATA status */
		FLASHStatus = FLASH_ProgramHalfWord(NewPageAddress, RECEIVE_DATA);
    /* If program operation was failed, a Flash error code is returned */
    if (FLASHStatus != FLASH_COMPLETE)
    {
        return FLASHStatus;
    }

    /* Write the variable passed as parameter in the new active page */
    EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data, len); 
    /* If program operation was failed, a Flash error code is returned */
    if (EepromStatus != FLASH_COMPLETE)
    {
        return EepromStatus;
    }

    /* Transfer process: transfer variables from old to the new active page */
    for (VarIdx = 0; VarIdx < NUM_OF_VAL_CELL; VarIdx++)
    {
        if (VarIdx != VirtAddress)  /* Check each variable except the one passed as parameter */
        {
            /* Read the other last variable updates */
            ReadStatus = EE_ReadVariable1(VarIdx, len, DatWorkBuf);
            /* In case variable corresponding to the virtual address was found */
            if (ReadStatus == FLASH_COMPLETE)                                    //改成只有读取成功才写入
            {
                /* Transfer the variable to the new active page */
                EepromStatus = EE_VerifyPageFullWriteVariable(VarIdx, DatWorkBuf, len);  //替换成多字节写入
                /* If program operation was failed, a Flash error code is returned */
                if (EepromStatus != FLASH_COMPLETE)
                {
                    return EepromStatus;
                }
            }
        }
    }

    FLASH_Sector = OldPageId;

    /* Erase the old Page: Set old Page status to ERASED status */
    FLASHStatus = FLASH_EraseSector(FLASH_Sector,VoltageRange_3);
    /* If erase operation was failed, a Flash error code is returned */
    if (FLASHStatus != FLASH_COMPLETE)
    {
        return FLASHStatus;
    }

    /* Set new Page status to VALID_PAGE status */
		FLASHStatus = FLASH_ProgramHalfWord(NewPageAddress, VALID_PAGE);
    /* If program operation was failed, a Flash error code is returned */
    if (FLASHStatus != FLASH_COMPLETE)
    {
        return FLASHStatus;
    }

    /* Return last operation flash status */
    return FLASHStatus;
}
