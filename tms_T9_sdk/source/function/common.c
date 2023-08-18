#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "se.h"
#include "types.h"
#include "driver.h"
#include "common.h"

unsigned char apduBuf[SC_MAX_CACHE_SIZE];
unsigned char RecvBuf[SC_MAX_CACHE_SIZE];

static int proc_6CXX(TRANSMIT_DATA *pData, unsigned char *pBuf, unsigned int *BufLen, unsigned char sw2)
{
    pData->tx[OFF_LE] = sw2;

    pData->rx = pBuf;
    pData->rx_len = *BufLen;
    if (SUCCEED != Driver_Transmit(pData))
    {
        return FAILED;
    }
    *BufLen = pData->rx_len;
    return SUCCEED;
}

static int proc_61XX(unsigned char *pBuf, unsigned int *BufLen, unsigned char sw2)
{
		const unsigned char get_resp[APDU_HEADER_SIZE] = {0x00,0xC0,0x00,0x00,0x00};
    unsigned char cmd[APDU_HEADER_SIZE] = {0};
    TRANSMIT_DATA apdu;
    
    memcpy(cmd, get_resp, APDU_HEADER_SIZE);
    cmd[OFF_LE] = sw2;

    apdu.max_wait_time = 1;
    apdu.tx = cmd;
    apdu.tx_len = APDU_HEADER_SIZE;
    apdu.rx = pBuf;
    apdu.rx_len = *BufLen;
    if (SUCCEED != Driver_Transmit(&apdu))
    {
        return FAILED;
    }
    *BufLen = apdu.rx_len;
    return SUCCEED;
}

static int proc_95XX(unsigned char *pBuf, unsigned int *BufLen, unsigned char sw2)
{
		const unsigned char get_resp[APDU_HEADER_SIZE] = {0x80,0xC6,0x00,0x00,0xFF};
    unsigned char cmd[APDU_HEADER_SIZE] = {0};
    TRANSMIT_DATA apdu;
    
    memcpy(cmd, get_resp, APDU_HEADER_SIZE);
    cmd[OFF_LE] = sw2;

    apdu.max_wait_time = 1;
    apdu.tx = cmd;
    apdu.tx_len = APDU_HEADER_SIZE;
    apdu.rx = pBuf;
    apdu.rx_len = *BufLen;
    if (SUCCEED != Driver_Transmit(&apdu))
    {
        return FAILED;
    }
    *BufLen = apdu.rx_len;
    return SUCCEED;
}

static int get_sw(unsigned char *rx, unsigned int rx_len, unsigned char *sw1, unsigned char *sw2)
{
    if(rx_len < 2) {
        return FAILED;
    }

    *sw1 = rx[rx_len - 2];
    *sw2 = rx[rx_len - 1];

    return SUCCEED;
}

static int check_sw(unsigned char sw1, unsigned char sw2)
{
    if(sw1 == 0x90 && sw2 == 0x00) {
        return SUCCEED;
    }
    else {
        return (int)((sw1 << 8) | sw2);
    }
}

unsigned char buf[APDU_BUFFER_MAX_SIZE] = {0};
int transmit_apdu(TRANSMIT_DATA *pAPDU)
{
    int rv;
    unsigned int len;
    unsigned char sw1,sw2;
    int rx_len = 0;//记录接收数据总长度

    rv = Driver_Transmit(pAPDU);
    if(rv != SUCCEED) {
        return ERR_COMM_FAILED;
    }

    rv = get_sw(pAPDU->rx, pAPDU->rx_len, &sw1, &sw2);
    if(rv != SUCCEED) {
        return ERR_TRANSMIT_DATA_ERR;
    }
    rx_len = pAPDU->rx_len - 2;

    while(sw1 == 0x6C||sw1 == 0x61 || sw1 == 0x95) {
        switch(sw1) {
            case        0x6C:
                        len = sizeof(buf);
                        rv = proc_6CXX(pAPDU, buf, &len, sw2);
                        if(rv != SUCCEED) {
                            return ERR_COMM_FAILED;
                        }
                        break;
            case        0x61:
                        len = sizeof(buf);
                        rv = proc_61XX(buf, &len, sw2);
                        if(rv != SUCCEED) {
                            return ERR_COMM_FAILED;
                        }
                        break;
            case        0x95:
                        len = sizeof(buf);
                        rv = proc_95XX(buf, &len, sw2);
                        if(rv != SUCCEED) {
                            return ERR_COMM_FAILED;
                        }
                        break;
        }

        rv = get_sw(buf, len, &sw1, &sw2);
        if(rv != SUCCEED) {
            return ERR_TRANSMIT_DATA_ERR;
        }
        
        memcpy(pAPDU->rx + rx_len, buf, len - 2);
        rx_len += len - 2;
    }

    rv = check_sw(sw1, sw2);
    if(rv != SUCCEED) {
        return rv;
    }
    
    pAPDU->rx_len = rx_len;
    
    return SUCCEED;
}

int sendApdu(unsigned char *data, unsigned int data_len)
{
    TRANSMIT_DATA apdu;

    if ((data == NULL) || (data_len == 0)) {
        return ERR_PARAM_FAILED;
    }

    apdu.tx_len = 0;

    memcpy(apduBuf + apdu.tx_len, data, data_len);
    apdu.tx_len += data_len;

    apdu.max_wait_time = 1;
    apdu.tx = apduBuf;
    apdu.rx = RecvBuf;
    apdu.rx_len = sizeof(RecvBuf);
    return transmit_apdu(&apdu);
}

void hexCharArray2Char(unsigned char* targ_c, unsigned char* orig_c, int orig_len)
{
    int i =0;
    int num = 0;
    for(i=0;i<orig_len;i++)
    {
        if (*orig_c>=48 && *orig_c<=57)
            num = num+(*orig_c)-48;
        else if (*orig_c>=65 && *orig_c<=70)
            num = num+(*orig_c)+10-65;
        else if (*orig_c>=97 && *orig_c<=102)
            num = num+(*orig_c)+10 - 97;
        orig_c++;
        if (i%2 == 1)
        {
            *targ_c++ = num;
            num = 0;
        }
        else
            num = num*16;
    }
}


void hex2Char(unsigned char* targ_c, unsigned char* orig_c, int orig_len)
{
    int i =0;
    int ch1 = 0;
    int ch2 = 0;
    for(i=0;i<orig_len;i++)
    {
        ch1 = (*orig_c >> 4) + 0x30;
        ch2 = (*orig_c & 0x0F) + 0x30;
        orig_c++;
        *targ_c++ = ch1;
        *targ_c++ = ch2;
    }
}

