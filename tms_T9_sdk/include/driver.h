#ifndef HEADER_DRIVER_H
#define HEADER_DRIVER_H

#include "config.h"
#include "types.h"

#define DELAY_1_SEC     1000    //can NOT to modify!!

typedef struct _TRANSMIT_DATA {
    unsigned char            *tx;
    int                        tx_len;
    unsigned char            *rx;
    int                        rx_len;
    int                         max_wait_time;
}TRANSMIT_DATA;

typedef struct _TMS_DRIVER_OPS {
    int (*init)(char *);
    int (*finish)(void);
    int (*reset)(void);
    int (*wakeup)(void);
    int (*sleep)(void);
    int (*transmit)(TRANSMIT_DATA *pData);
}TMS_DRIVER_OPS;

typedef struct _TMS_DRIVER {
    char                *name;
    TMS_DRIVER_OPS        *ops;
}TMS_DRIVER;

#ifdef ENABLE_SPI
TMS_DRIVER* tms_get_spi_driver(void);
#elif defined(ENABLE_I2C)
TMS_DRIVER* tms_get_i2c_driver(void);
#endif

int Initialize_Driver(char *dev_name);
int Finalize_Driver(void);
int Driver_Transmit(TRANSMIT_DATA *pData);
int Driver_wakeup(void);

#endif    /*!HEADER_DRIVER_H*/
