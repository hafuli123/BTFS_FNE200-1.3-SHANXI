#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "driver.h"
#include "types.h"

TMS_DRIVER *driver = NULL;

TMS_DRIVER* tms_get_driver(void)
{
    return tms_get_spi_driver();
}


int Initialize_Driver(char *dev_name)
{
    int rv;

    if (driver != NULL) {
        return SUCCEED;
    }

    driver = tms_get_driver();

    if(driver->ops->init != NULL) {
        rv = driver->ops->init(dev_name);
        if(rv == FAILED) {
            return FAILED;
        }
    }
    else {
        return FAILED;
    }

    if(driver->ops->wakeup != NULL) {
        rv = driver->ops->wakeup();
        if(rv == FAILED) {
            return FAILED;
        }
    }

    return SUCCEED;
}


int Finalize_Driver(void)
{
    int rv;

    if(driver->ops->sleep != NULL) {
        rv = driver->ops->sleep();
        if(rv == FAILED) {
            return FAILED;
        }
    }
    if(driver->ops->finish != NULL) {
        rv = driver->ops->finish();
        if(rv == FAILED) {
            return FAILED;
        }
    }
    else {
        return FAILED;
    }

    driver = NULL;

    return SUCCEED;
}

int Driver_Transmit(TRANSMIT_DATA *pData)
{
    int rv;

    if(pData == NULL) {
        return FAILED;
    }

    if(driver->ops->transmit != NULL) {
        rv = driver->ops->transmit(pData);
        if(rv == FAILED) {
            return FAILED;
        }
    }
    else {
        return FAILED;
    }
    
    return SUCCEED;
}


int Driver_wakeup(void)
{
    int rv = SUCCEED;
    
    if(driver->ops->wakeup != NULL) {
        rv = driver->ops->wakeup();
        if(rv == FAILED) {
            return FAILED;
        }
    }
    return SUCCEED;
}
