#ifndef HEADER_SE_H
#define HEADER_SE_H

#if defined(__cplusplus)
extern "C" {
#endif

/*SE function*/
#define _T9_DECLARE_FUNCTION(name, args)    \
typedef int (*t9_ ## name) args;        \
int name args

_T9_DECLARE_FUNCTION (tms_initialize, (char *dev_name));
_T9_DECLARE_FUNCTION (tms_finalize, (void));

_T9_DECLARE_FUNCTION (tms_sm2_genkeypair, (unsigned short sm2ID));
_T9_DECLARE_FUNCTION (tms_sm2_import_pubkey, (unsigned short sm2ID, unsigned char *pub_key, unsigned int pub_key_len));
_T9_DECLARE_FUNCTION (tms_sm2_import_prikey, (unsigned short sm2ID, unsigned char *pri_key, unsigned int pri_key_len));
_T9_DECLARE_FUNCTION (tms_sm2_export_pubkey, (unsigned short sm2ID, unsigned char *pub_key, unsigned int *pub_key_len));
_T9_DECLARE_FUNCTION (tms_sm2_encrypt, (unsigned char *data, unsigned int data_len, unsigned char *encrypted_data, unsigned int *encrypted_data_len, unsigned short sm2ID));
_T9_DECLARE_FUNCTION (tms_sm2_decrypt, (unsigned char *data, unsigned int data_len, unsigned char *decrypted_data, unsigned int *decrypted_data_len, unsigned short sm2ID));
_T9_DECLARE_FUNCTION (tms_sm2_with_sm3_signature, (unsigned char *data, unsigned int data_len, unsigned char *sign_data, unsigned int *sign_data_len, unsigned short sm2ID, unsigned char userid[16]));
_T9_DECLARE_FUNCTION (tms_sm2_with_sm3_verify, (unsigned char *data, unsigned int data_len, unsigned char *sign_data, unsigned int sign_data_len, unsigned short sm2ID, unsigned char userid[16]));
_T9_DECLARE_FUNCTION (tms_sm2_signature, (unsigned char *digest, unsigned int digest_len, unsigned char *sign_data, unsigned int *sign_data_len, unsigned short sm2ID));
_T9_DECLARE_FUNCTION (tms_sm2_verify, (unsigned char *digest, unsigned int digest_len, unsigned char *sign_data, unsigned int sign_data_len, unsigned short sm2ID));
_T9_DECLARE_FUNCTION (tms_sm2_ext_encrypt, (unsigned char *data, unsigned int data_len, unsigned char *encrypted_data, unsigned int *encrypted_data_len, unsigned char *pub_key, unsigned int pub_key_len));
_T9_DECLARE_FUNCTION (tms_sm2_ext_decrypt, (unsigned char *data, unsigned int data_len, unsigned char *decrypted_data, unsigned int *decrypted_data_len, unsigned char *pri_key, unsigned int pri_key_len));
_T9_DECLARE_FUNCTION (tms_sm2_ext_signature, (unsigned char *digest, unsigned int digest_len, unsigned char *sign_data, unsigned int *sign_data_len, unsigned char *pri_key, unsigned int pri_key_len));
_T9_DECLARE_FUNCTION (tms_sm2_ext_verify, (unsigned char *digest, unsigned int digest_len, unsigned char *sign_data, unsigned int sign_data_len, unsigned char *pub_key, unsigned int pub_key_len));

_T9_DECLARE_FUNCTION (tmscmpt_sm2_genkeypair,(unsigned short sm2PubID,unsigned short sm2PriID));
_T9_DECLARE_FUNCTION (tmscmpt_sm2_export_pubkey,(unsigned short sm2ID, unsigned char *pub_key, unsigned int *pub_key_len));
_T9_DECLARE_FUNCTION (tmscmpt_sm2_with_sm3_signature,(unsigned char *data, unsigned int data_len,unsigned char *sign_data, unsigned int *sign_data_len,unsigned short sm2ID, unsigned char userid[16]));
_T9_DECLARE_FUNCTION (tmscmpt_sm2_import_pubkey,(unsigned short sm2ID, unsigned char *pub_key, unsigned int pub_key_len));
_T9_DECLARE_FUNCTION (tmscmpt_sm2_encrypt,(unsigned char *data, unsigned int data_len,unsigned char *encrypted_data, unsigned int *encrypted_data_len,unsigned short sm2ID));
_T9_DECLARE_FUNCTION (tmscmpt_sm2_decrypt,(unsigned char *data, unsigned int data_len,unsigned char *decrypted_data, unsigned int *decrypted_data_len, unsigned short sm2ID));


#if defined(__cplusplus)
}
#endif

#endif        /*!HEADER_SE_H*/

