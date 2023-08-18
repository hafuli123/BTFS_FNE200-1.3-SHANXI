#ifndef _BSP_EEPROW_H
#define _BSP_EEPROW_H

#include "stdint.h"

/* EEPROM emulation firmware error codes */
#define EE_OK      (uint32_t)FLASH_COMPLETE
#define EE_BUSY    (uint32_t)FLASH_BUSY

/* Define the size of the sectors to be used */
#define PAGE_SIZE               (uint32_t)0x20000  		/* Page size = 128KByte */
/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS  ((uint32_t)0x080C0000) 	/* EEPROM emulation start address:from sector10 : after 128KByte of used Flash memory */

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x0000))
#define PAGE0_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))
#define PAGE0_ID               10U

#define PAGE1_BASE_ADDRESS    ((uint32_t)(EEPROM_START_ADDRESS + 0x20000))
#define PAGE1_END_ADDRESS     ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))
#define PAGE1_ID               11U

/* Used Flash pages for EEPROM emulation */
#define PAGE0                 ((uint16_t)0x0000)
#define PAGE1                 ((uint16_t)0x0001) /* Page nb between PAGE0_BASE_ADDRESS & PAGE1_BASE_ADDRESS*/


/* Page status definitions */
#define ERASED                ((uint16_t)0xFFFF)     /* Page is empty */
#define RECEIVE_DATA          ((uint16_t)0xEEEE)     /* Page is marked to receive data */
#define VALID_PAGE            ((uint16_t)0x0000)     /* Page containing valid data */
#define NO_VALID_PAGE         ((uint16_t)0x00AB)		 /* No valid page define */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE  ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE   ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL             ((uint8_t)0x80)
																																				

#define EEPROM_CELL_SZ          32                                                	//数据空间分配的大小, Bytes
#define EEPROM_CELL_ADDR_LEN    2                                                 	//2 Bytes
#define EEPROM_CELL_DATA_LEN    (EEPROM_CELL_SZ - EEPROM_CELL_ADDR_LEN)             //30 Bytes
#define EEPROM_SIZE             PAGE_SIZE                                         	//eeprom容量: 128KB
#define EEPROM_CELL_NUM         (EEPROM_SIZE / EEPROM_CELL_SZ - 1)                  //可存入数据单元的数量512-1, 第一个cell留作Page status definition

#define EEPROM_ADDR_MIN         0
#define EEPROM_ADDR_MAX         2046

#define NUM_OF_VAL_CELL         ((uint8_t)(EEPROM_ADDR_MAX - EEPROM_ADDR_MIN + 1))	//最多使用2045个地址，每个地址对应一个有效CELL，最多NUM_OF_VAL_CELL个有效cell

uint16_t EE_Init1(void);
uint16_t EE_ReadVariable1(uint16_t VirtAddress, uint16_t len, void *data);
uint16_t EE_WriteVariable1(uint16_t VirtAddress, uint16_t len, void *data);

#endif
