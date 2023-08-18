//
// Created by liuchen on 20-9-16.
//

#ifndef HEADER_COMMON_H
#define HEADER_COMMON_H

#include "driver.h"
#include "bsp_sys.h"

#define SC_MAX_CACHE_SIZE            300

extern unsigned char apduBuf[SC_MAX_CACHE_SIZE];
extern unsigned char RecvBuf[SC_MAX_CACHE_SIZE];


#define RSA_PKCS1_PADDING_SIZE  11


#define OFF_CLA                        0
#define OFF_INS                        1
#define OFF_P1                        2
#define OFF_P2                        3
#define OFF_LC                        4
#define OFF_LE                        4
#define APDU_HEADER_SIZE        5
#define APDU_BODY_MAX_SIZE            255
#define APDU_BODY_LE_SIZE            1
#define APDU_BUFFER_MAX_SIZE         (APDU_HEADER_SIZE+APDU_BODY_MAX_SIZE+APDU_BODY_LE_SIZE)
#define SYM_BODY_MAX_SIZE            240

#define BINARY_MAX_SIZE             253

/*  Error Code*/
#define ERR_PARAM_FAILED            1
#define ERR_COMM_FAILED                2
#define ERR_TRANSMIT_DATA_ERR        3
#define ERR_SE_USE_NOT_SATISFIED    27013//6985
#define ERR_SE_FILE_NOT_EXIST        27266//6A82

#define BIN_FILE_TAG    0x80
#define SM2_FILE_TAG    0x81
#define ECC_FILE_TAG    0x82
#define RSA_FILE_TAG    0x83
#define SM4_FILE_TAG    0x84
#define AES_FILE_TAG    0x85

#define PKG_UNIQUE          0x00
#define PKG_FIRST           0x01
#define PKG_MID              0x02
#define PKG_LAST            0x03

#define SM3_HASH_SIZE   32
#define SHA1_HASH_SIZE 20
#define SHA256_HASH_SIZE 32
#define SHA384_HASH_SIZE 48
#define SHA512_HASH_SIZE 64

#define RSA_BIT_LENGTH 2048
#define SM2_BIT_LENGTH 256
#define ECC_BIT_LENGTH 256

#define SM2_PUBKEY_SIZE     64
#define SM2_PRIKEY_SIZE     32
#define SM2_USERID_SIZE     16

#define ECC_PUBKEY_SIZE     64
#define ECC_PRIKEY_SIZE     32

#define SYM_BLOCK_LEN       16

#define SM2_SIGNATURE_LEN   64
#define ECC_SIGNATURE_LEN   64
#define RSA_MOD_LEN         256
#define SM2_PLAIN_MAX    2048
#define SM2_CIPHER_ATTACHMENT     96
#define SM2_CIPHER_MAX    (SM2_PLAIN_MAX + SM2_CIPHER_ATTACHMENT)

#define FIRMWARE_VER        4

#define DATA_STORAGE_ID 0x0002
#define DATA_LEN                16*1024

#define HASH_SM3            0x00
#define HASH_SHA256     0x01

int transmit_apdu(TRANSMIT_DATA *pAPDU);
int sendApdu(unsigned char *data, unsigned int data_len);
void hexCharArray2Char(unsigned char* targ_c, unsigned char* orig_c, int orig_len);
void hex2Char(unsigned char* targ_c, unsigned char* orig_c, int orig_len);

int patch(void);
int isInitialize(void);


#endif //!HEADER_COMMON_H
