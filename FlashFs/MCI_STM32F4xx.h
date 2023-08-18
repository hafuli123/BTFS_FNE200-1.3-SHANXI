/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2018 Arm Limited (or its affiliates). All 
 * rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * $Date:        05. February 2018
 * $Revision:    V2.6
 *
 * Project:      MCI Driver Definitions for ST STM32F4xx
 * -------------------------------------------------------------------------- */

#ifndef __MCI_STM32F4XX_H
#define __MCI_STM32F4XX_H

#include "Driver_MCI.h"
#include <string.h>
#include "stm32f4xx.h"

#define MCI_BUS_MODE_HS		0U
/* Define 4-bit data bus width */
#define MCI_BUS_WIDTH_4   1U

/* Define 8-bit data bus width */
#define MCI_BUS_WIDTH_8   0U

/* Define Card Detect pin existence */
#define MCI_CD_PIN        0U

/* Define Write Protect pin existence */
#define MCI_WP_PIN        0U

/* SDIO Adapter Clock definition */
#define SDIOCLK             48000000U    /* SDIO adapter clock */
#define __CPUCLK 					 168000000U

#ifndef SDIO_MASK_STBITERRIE
#define SDIO_MASK_STBITERRIE    0U
#endif
#ifndef SDIO_STA_STBITERR
#define SDIO_STA_STBITERR       0U
#endif
#ifndef SDIO_ICR_STBITERRC
#define SDIO_ICR_STBITERRC      0U
#endif

/* Interrupt clear Mask */
#define SDIO_ICR_BIT_Msk       (SDIO_ICR_CCRCFAILC | \
                                SDIO_ICR_DCRCFAILC | \
                                SDIO_ICR_CTIMEOUTC | \
                                SDIO_ICR_DTIMEOUTC | \
                                SDIO_ICR_TXUNDERRC | \
                                SDIO_ICR_RXOVERRC  | \
                                SDIO_ICR_CMDRENDC  | \
                                SDIO_ICR_CMDSENTC  | \
                                SDIO_ICR_DATAENDC  | \
                                SDIO_ICR_STBITERRC | \
                                SDIO_ICR_DBCKENDC  | \
                                SDIO_ICR_SDIOITC)

/* Error interrupt mask */
#define SDIO_STA_ERR_BIT_Msk   (SDIO_STA_CCRCFAIL | \
                                SDIO_STA_DCRCFAIL | \
                                SDIO_STA_CTIMEOUT | \
                                SDIO_STA_DTIMEOUT | \
                                SDIO_STA_STBITERR)

/* Driver flag definitions */
#define MCI_INIT      ((uint8_t)0x01)   /* MCI initialized           */
#define MCI_POWER     ((uint8_t)0x02)   /* MCI powered on            */
#define MCI_SETUP     ((uint8_t)0x04)   /* MCI configured            */
#define MCI_RESP_LONG ((uint8_t)0x08)   /* Long response expected    */
#define MCI_RESP_CRC  ((uint8_t)0x10)   /* Check response CRC error  */
#define MCI_DATA_XFER ((uint8_t)0x20)   /* Transfer data             */
#define MCI_DATA_READ ((uint8_t)0x40)   /* Read transfer             */
#define MCI_READ_WAIT ((uint8_t)0x80)   /* Read wait operation start */

#define MCI_RESPONSE_EXPECTED_Msk (ARM_MCI_RESPONSE_SHORT      | \
                                   ARM_MCI_RESPONSE_SHORT_BUSY | \
                                   ARM_MCI_RESPONSE_LONG)

/* MCI Transfer Information Definition */
typedef struct _MCI_XFER {
  uint8_t *buf;                         /* Data buffer                        */
  uint32_t cnt;                         /* Data bytes to transfer             */
} MCI_XFER;

/* MCI Driver State Definition */
typedef struct _MCI_INFO {
  ARM_MCI_SignalEvent_t cb_event;       /* Driver event callback function     */
  ARM_MCI_STATUS        status;         /* Driver status                      */
  uint32_t             *response;       /* Pointer to response buffer         */
  MCI_XFER              xfer;           /* Data transfer description          */
  uint32_t              dctrl;          /* Data control register value        */
  uint32_t              dlen;           /* Data length register value         */
  uint16_t volatile     flags;          /* Driver state flags                 */
  uint16_t              reserved;       /* Reserved                           */
} MCI_INFO;

extern ARM_DRIVER_MCI Driver_MCI0;

#endif /* __MCI_STM32F4XX_H */
