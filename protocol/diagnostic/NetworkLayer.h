#ifndef _NETWORKLAYER_H
#define _NETWORKLAYER_H

#include "NetworkLayerTypeDefines.h"

#define MAX_BUFF_NUMBER													3
#define MAX_INDICATION_NUMBER										3
#define MAX_DOWNLOADING_BUF											100

typedef struct
{
 NetworkNotification IndicationList[MAX_INDICATION_NUMBER];		//ָʾ�б�
 uint8_t IndicationInIndex;																		//ָʾ����λ��
 uint8_t IndicationOutIndex;																	//ָʾ��ȡλ��
	//����CAN���ݻ��δ���ȡ��
 NetworkFrame RxFrameBuff[MAX_BUFF_NUMBER];										//rx֡
 uint8_t RxInIndex;																						//rx����λ��
 uint8_t RxOutIndex;																					//rx��ȡλ��
	//����CAN���ݻ��Ѵ���ȡ��
 NetworkFrame TxFrameBuff[MAX_BUFF_NUMBER];										//tx֡
 uint8_t TxInIndex;																						//tx����λ��
 uint8_t TxOutIndex;																					//tx��ȡλ��
 //�������
 N_Result m_N_Result;																				//������
 NWL_Status m_NetworkStatus;																//����״̬�����У����գ����䣬�ȴ�
 DuplexMode m_DuplexMode;																		//˫��ģʽ ȫ˫��������ʱ���л�������ģʽ�� ��˫��
 TimePeriodParam m_TimePeriod;															//ʱ��ο���
 TransmissionStep m_TxStep;																	//����״̬��
 RecivingStep m_RxStep;																			//����״̬��
 AddressFormat m_AddressFormat;															//��ַ��ʽ
 uint8_t* CFDataPionter;																		//������������ָ֡��
 CommuParam TxParam;																				//���Ͳ���
 CommuParam RxParam;																				//���ղ���
 uint8_t NetworkDataBufRx[MAX_DOWNLOADING_BUF];							//�������ݵĻ���
 uint8_t FrameFillData;																			//CAN֡�������
 //CAN����
 uint32_t m_PyhReqID;																				//����ID
 uint32_t m_FunReqID;																				//����ID
 uint32_t m_ResponseID;																			//��ӦID
 uint8_t RxDataBuff[7];																			//RX���ݻ���
 //��ʱ��
 DiagTimer SendTimer;																				//���Ͷ�ʱ��
 DiagTimer ReciveTimer;																			//���ն�ʱ��
 DiagTimer CFRequestTimer;																	//����֡����ʱ��
 SendCANFun NetworkSend;																		//CAN���ͺ���
}TP_T;

//��ʼ������
extern void NetworkLayer_InitParam(TP_T* pTp,uint32_t PyhReqID,uint32_t FunReqID, uint32_t ResponseID,SendCANFun sendFun,GetMsTickCountFun GetMsTickCntFun);
//��������
extern void NetworkLayer_SetParam(TP_T* pTp,uint8_t TimeAs, uint8_t TimeBs, uint8_t TimeCr, uint8_t TimeAr, uint8_t TimeBr, uint8_t TimeCs, 	uint8_t Bs, uint8_t m_STmin, DuplexMode nDuplex ,  MType Mtype , uint8_t N_SA , uint8_t N_TA , N_TAtype N_TAtype , uint8_t N_AE , uint8_t FillData);
//����㴦��
extern void NetworkLayer_Proc(TP_T* pTp);
//���ݷ���
extern void N_USData_request(TP_T* pTp,MType Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtype N_TAtype, uint8_t N_AE, uint8_t* MessageData, uint16_t Length);
//���ݽ���
extern void NetworkLayer_RxFrame(TP_T* pTp,uint32_t ID,uint8_t* data,uint8_t IDE,uint8_t DLC,uint8_t RTR);
//ָʾ�Ƿ�Ϊ��
extern bool IsIndicationListEmpty(TP_T* pTp);
//��ȡָʾ
extern NetworkNotification PullIndication(TP_T* pTp);
//��Ϊ��������
extern void N_ChangeParameter_request(TP_T* pTp,MType Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtype N_TAtype, uint8_t N_AE, Parameter Parameter, uint8_t Parameter_Value);
extern void DiagTimer_Set(DiagTimer *STimer, uint32_t TimeLength);
extern bool DiagTimer_HasExpired(DiagTimer *STimer);
#endif
