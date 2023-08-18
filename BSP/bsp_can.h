#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "stdint.h"

typedef enum {BSP_CAN1 = 1, BSP_CAN2 = 2, BSP_CAN3 = 3 ,BSP_CAN = 0xFF} BSP_CAN_IDX;

typedef struct {
  uint32_t id;                 /* 29 bit identifier                               */
  uint8_t  data[8];            /* Data field                                      */
  uint8_t  len;                /* Length of data field in bytes                   */
  uint8_t  ch;                 /* Object channel                                  */
  uint8_t  format;             /* 0 - STANDARD,   1 - EXTENDED IDENTIFIER         */
  uint8_t  type;               /* 0 - DATA FRAME, 1 - REMOTE FRAME                */
} CAN_msg;

/* Symbolic names for formats of CAN message                                 */
typedef enum {STANDARD_FORMAT = 0, EXTENDED_FORMAT} CAN_FORMAT;

/* Symbolic names for type of CAN message                                    */
typedef enum {DATA_FRAME      = 0, REMOTE_FRAME   } CAN_FRAME;

/* Definitions for filter_type                                               */
#define FORMAT_TYPE     (1 << 0)
#define FRAME_TYPE      (1 << 1)
#define STANDARD_TYPE   (0 << 0)
#define EXTENDED_TYPE   (1 << 0)
#define DATA_TYPE       (0 << 1)
#define REMOTE_TYPE     (1 << 1)

/* Error values that functions can return                                    */
typedef enum   
{ CAN_OK = 0,                       /* No error                              */
  CAN_NOT_IMPLEMENTED_ERROR,        /* Function has not been implemented     */
  CAN_MEM_POOL_INIT_ERROR,          /* Memory pool initialization error      */
  CAN_BAUDRATE_ERROR,               /* Baudrate was not set                  */
  CAN_TX_BUSY_ERROR,                /* Transmitting hardware busy            */
  CAN_OBJECTS_FULL_ERROR,           /* No more rx or tx objects available    */
  CAN_ALLOC_MEM_ERROR,              /* Unable to allocate memory from pool   */
  CAN_DEALLOC_MEM_ERROR,            /* Unable to deallocate memory           */
  CAN_TIMEOUT_ERROR,                /* Timeout expired                       */
  CAN_UNEXIST_CTRL_ERROR,           /* Controller does not exist             */
  CAN_UNEXIST_CH_ERROR,             /* Channel does not exist                */
} CAN_ERROR;

extern CAN_ERROR CAN_init (uint32_t ctrl, uint32_t baudrate);
extern CAN_ERROR CAN_start (uint32_t ctrl);
extern CAN_ERROR CAN_send (uint32_t ctrl,CAN_msg* msg,uint32_t timeout);
extern CAN_ERROR CAN_receive (CAN_msg *msg, uint32_t timeout);
extern void setCANmask(uint8_t idx,uint32_t id,uint32_t mask,uint8_t format);
extern uint8_t CAN_IrqSeq(void);

#endif
