#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "se.h"
#include "types.h"
#include "driver.h"
#include "common.h"


int patch(void)
{
    int rv;
    unsigned char patchBegin[] = {"FCFF000008544d434f53342e30"};
    unsigned char patchFinish[] = {"FCFF510000"};

    unsigned char cmd[150];
    int length;

    length = sizeof(patchBegin) - 1;
    hexCharArray2Char(cmd, patchBegin, length);
    length = length >> 1;//¡Â2
    rv = sendApdu(cmd, length);
    if(rv != SUCCEED)    {
        return FAILED;
    }

    length = sizeof(patchFinish) - 1;
    hexCharArray2Char(cmd, patchFinish, length);
    length = length >> 1;//¡Â2
    rv = sendApdu(cmd, length);
    if(rv != SUCCEED)    {
        return FAILED;
    }
    osDelay(1);
    
    Driver_wakeup();

    osDelay(1);

    return SUCCEED;
}




