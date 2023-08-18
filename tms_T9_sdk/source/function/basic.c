#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "se.h"
#include "types.h"
#include "driver.h"
#include "common.h"

const char *software_ver = "V2.0.1_";

unsigned char sdk_init_flag = 0;

int getLifecycle(unsigned char *LifeCyc)
{
    int rv = SUCCEED;
    unsigned char getLCS[] = {0x80,0xB0,0x00,0x00,0x03,0x00,0x01,0x01};

    rv = sendApdu(getLCS, sizeof(getLCS));
    if (rv == ERR_SE_FILE_NOT_EXIST) {
        *LifeCyc = 0x00;
        return SUCCEED;
    }
    else if(rv != SUCCEED)    {
        rv = FAILED;
        goto end;
    }

    *LifeCyc = (RecvBuf[0] == 0x01) ? 0x01 : 0x00;
    
end:
    return rv;
}

int buildFilesys(void) 
{
    int rv = FAILED;
    TRANSMIT_DATA apdu;

    unsigned char createFilesys[] = {0x80,0xE0,0x00,0x00,0x3F,
        0x80,0x00,0x01,0x00,0x01,0x00,0x10,0x00,0x00,// 1 binary file for system config (16 bytes)
        0x80,0x00,0x02,0x00,0x02,0x40,0x00,0x00,0x00,// 1 binary file for general storage (16K bytes)
        0x81,0x00,0x01,0x00,0x03,0x00,0x60,0x0F,0x00,// 3 sm2 file
        0x82,0x00,0x01,0x00,0x03,0x00,0x60,0x0F,0x00,// 3 ecc file
        0x83,0x00,0x01,0x00,0x03,0x03,0x80,0x0F,0x00,// 3 rsa file
        0x84,0x00,0x01,0x00,0x03,0x00,0x10,0x0F,0x00,// 3 sm4 file
        0x85,0x00,0x01,0x00,0x03,0x00,0x20,0x0F,0x00,// 3 aes file
        };
    unsigned char switchLifeCyc[] = {0x80,0xD6,0x00,0x00,0x03,0x00,0x01,0x01};

    memcpy(apduBuf, createFilesys, sizeof(createFilesys));
    apdu.max_wait_time = 3;
    apdu.tx = apduBuf;
    apdu.tx_len = sizeof(createFilesys);
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    rv = transmit_apdu(&apdu);
    if(rv != SUCCEED)    {
        return FAILED;
    }
    rv = sendApdu(switchLifeCyc, sizeof(switchLifeCyc));
    if(rv != SUCCEED)    {
        return FAILED;
    }

    return SUCCEED;
}

int personal(void)
{
    int rv;
    unsigned char deleteFilesys[] = {0x80,0xE4,0x00,0x00,0x00};
    unsigned char lifecyc;

    rv = getLifecycle(&lifecyc);
    if(rv != SUCCEED)    {
        return FAILED;
    }
    if (lifecyc == 0x00) {//上次初始化未成功建立文件结构
        sendApdu(deleteFilesys, sizeof(deleteFilesys));
    }
    else {
        return SUCCEED;
    }

    return buildFilesys();
}

int isInitialize(void)
{
    if (!sdk_init_flag) {
        return FAILED;
    }
    return SUCCEED;
}

int tms_initialize(char *dev_name)
{
    int rv;


    if (!sdk_init_flag) {

        rv = Initialize_Driver(dev_name);
        if (rv != SUCCEED) {
            return FAILED;
        }
        sdk_init_flag = 1;
    }
		
		rv = patch();
		if(rv != SUCCEED) {
				rv = Finalize_Driver();
				sdk_init_flag = 0;
				return FAILED;
		}
			
		rv = personal();
		if(rv != SUCCEED) {
				return FAILED;
		}

    return SUCCEED;
}

int tms_finalize(void)
{
    int rv;
    
    if (sdk_init_flag) {
        rv = Finalize_Driver();
        if (rv != SUCCEED) {
            return FAILED;
        }
        sdk_init_flag = 0;
    }

    return SUCCEED;
}

