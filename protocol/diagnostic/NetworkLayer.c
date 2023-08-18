#include "NetworkLayer.h"

//����������ܣ����̿���
static void NetworkLayer_MainProc(TP_T* pTp);																	//����㴦��
//���͹��ܣ����ݲ��
static void NetworkLayer_TxProc(TP_T* pTp);																		//���ʹ���
static void NetworkLayer_SendSF(TP_T* pTp,uint8_t length, uint8_t *data);			//���͵�֡
static void NetworkLayer_SendFF(TP_T* pTp,uint16_t length, uint8_t *data);		//������֡
static void NetworkLayer_SendCF(TP_T* pTp);																		//��������֡
static void NetworkLayer_SendFC(TP_T* pTp);																		//��������֡
static void NetworkLayer_TxEnd(TP_T* pTp);																		//���ͽ���
//���չ��ܣ��������
static void NetworkLayer_RxProc(TP_T* pTp);																		//���մ���
static void NetworkLayer_RxSF(TP_T* pTp,NetworkFrame RxFrame);								//���յ�֡
static void NetworkLayer_RxFF(TP_T* pTp,NetworkFrame RxFrame);								//������֡
static void NetworkLayer_RxCF(TP_T* pTp,NetworkFrame RxFrame);								//��������֡
static void NetworkLayer_RxFC(TP_T* pTp,NetworkFrame RxFrame);								//��������֡
static void NetworkLayer_RxEnd(TP_T* pTp);																		//���ս���
//�ύ����ָʾ
static void N_USData_indication(TP_T* pTp,N_PCIType PciType,MType Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtype N_TAtype, uint8_t N_AE, uint8_t* MessageData, uint16_t Length, N_Result N_Result);
//��������ȷ��
static void N_USData_confirm(TP_T* pTp,N_PCIType PciType,MType Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtype N_TAtype, uint8_t N_AE, N_Result N_Result);
//���λ�����ָʾ
static bool IsRxBuffEmpty(TP_T* pTp);																					//���տ�
static bool IsRxBuffFull(TP_T* pTp);																					//������
static bool IsTxBuffEmpty(TP_T* pTp);																					//���Ϳ�
static bool IsTxBuffFull(TP_T* pTp);																					//������
static bool IsIndicationListFull(TP_T* pTp);																	//ָʾ��
static GetMsTickCountFun DiagTimerGetMsTickCntFun = NULL;											//��ȡϵͳ����tickָ��

void DiagTimer_Set(DiagTimer *STimer, uint32_t TimeLength)
{
	if(DiagTimerGetMsTickCntFun != NULL)
	{
		STimer->TimerCounter = DiagTimerGetMsTickCntFun() + TimeLength;
		STimer->valid = true;
	}
}

bool DiagTimer_HasExpired(DiagTimer *STimer)
{
	if(STimer->valid == true && DiagTimerGetMsTickCntFun != NULL)
	{
		if(STimer->TimerCounter == 0)
		{
			STimer->valid = false;
			return true;
		}
		else if((DiagTimerGetMsTickCntFun() - STimer->TimerCounter) <= 0x7fffffff)
		{
			STimer->TimerCounter = 0;	//set timer to stop
			STimer->valid = false;
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

/*���տ�*/
bool IsRxBuffEmpty(TP_T* pTp)
{
	return (pTp->RxInIndex == pTp->RxOutIndex);
}

/*������*/
bool IsRxBuffFull(TP_T* pTp)
{
	return ((pTp->RxInIndex ==  (MAX_BUFF_NUMBER - 1) && pTp->RxOutIndex == 0) || ((pTp->RxInIndex + 1) == pTp->RxOutIndex));
}

/*���Ϳ�*/
bool IsTxBuffEmpty(TP_T* pTp)
{
	return (pTp->TxInIndex == pTp->TxOutIndex);
}

/*������*/
bool IsTxBuffFull(TP_T* pTp)
{
	return ((pTp->TxInIndex ==  (MAX_BUFF_NUMBER - 1) && pTp->TxOutIndex == 0) || ((pTp->TxInIndex + 1) == pTp->TxOutIndex));
}

/*����㷢��CAN����*/
static void NetworkLayer_TxFrame(TP_T* pTp,NetworkFrame txFrame)
{
	uint8_t TxDataBuff[8];	
	TxDataBuff[0] = txFrame.CanData.data0;
	TxDataBuff[1] = txFrame.CanData.data1;
	TxDataBuff[2] = txFrame.CanData.data2;
	TxDataBuff[3] = txFrame.CanData.data3;
	TxDataBuff[4] = txFrame.CanData.data4;
	TxDataBuff[5] = txFrame.CanData.data5;
	TxDataBuff[6] = txFrame.CanData.data6;
	TxDataBuff[7] = txFrame.CanData.data7;
	if(pTp->NetworkSend != NULL)
	{
		pTp->NetworkSend(pTp->m_ResponseID,TxDataBuff,8,0,0,pTp->m_ResponseID > 0x7FF);
		pTp->m_N_Result = N_OK;
	}
	else
	{
		pTp->m_N_Result = N_ERROR;
	}
	N_USData_confirm(pTp,txFrame.N_PDU.N_PciType , pTp->m_AddressFormat.Mtype , pTp->m_AddressFormat.N_SA , pTp->m_AddressFormat.N_TA , pTp->m_AddressFormat.N_TAtype , pTp->m_AddressFormat.N_AE , pTp->m_N_Result);
}

/*CAN����*/
void NetworkLayer_TxProc(TP_T* pTp)
{
	if(!IsTxBuffEmpty(pTp))
	{
		NetworkLayer_TxFrame(pTp,pTp->TxFrameBuff[pTp->TxOutIndex]);
		(pTp->TxOutIndex >= (MAX_BUFF_NUMBER - 1)) ? (pTp->TxOutIndex = 0) : (pTp->TxOutIndex++);
	}
}

/*�ѷ�����֡��ʼ��������*/
void NetworkLayer_TxStart(TP_T* pTp)
{
	DiagTimer_Set(&pTp->SendTimer,pTp->m_TimePeriod.N_As);
	pTp->m_NetworkStatus = NWL_TRANSMITTING;
	pTp->m_TxStep = TX_WAIT_FF_CONF;
}

/*��������*/
void NetworkLayer_TxEnd(TP_T* pTp)
{
	pTp->m_TxStep = TX_IDLE;
	pTp->m_NetworkStatus = NWL_IDLE;
}

/*���͵�֡*/
void NetworkLayer_SendSF(TP_T* pTp,uint8_t length, uint8_t *data)
{
	uint8_t i;
	if(length  <= 7)//SF length must <= 7
	{
		if(!IsTxBuffFull(pTp))
		{
			for(i = 0; i < 7; i++)
			{
				if(i >= length)
				{
					*(&pTp->TxFrameBuff[pTp->TxInIndex].CanData.data7 + (6-i)) = pTp->FrameFillData;
				}
				else
				{
					*(&pTp->TxFrameBuff[pTp->TxInIndex].CanData.data7 + (6-i)) = *(data + i);
				}
			}
			pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.N_PciType = SF;
			pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.SF_DL = length;
			(pTp->TxInIndex >= MAX_BUFF_NUMBER - 1) ? (pTp->TxInIndex = 0) : (pTp->TxInIndex++);
		}
		else
		{
			//printf("send SF but tx buf full\r\n");
		}
	}
}

/*������֡*/
void NetworkLayer_SendFF(TP_T* pTp,uint16_t length, uint8_t *data)
{
	if(length > 7 && length < 0xFFF)
	{
		pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.N_PciType = FF;
		pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.SF_DL = length >> 8;			//length high 4 bits
		pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.FF_DL_LOW = length & 0xFF;	//length low nibble
		
		pTp->TxFrameBuff[pTp->TxInIndex].CanData.data2 = *data;
		pTp->TxFrameBuff[pTp->TxInIndex].CanData.data3 = *(data + 1);
		pTp->TxFrameBuff[pTp->TxInIndex].CanData.data4 = *(data + 2);
		pTp->TxFrameBuff[pTp->TxInIndex].CanData.data5 = *(data + 3);
		pTp->TxFrameBuff[pTp->TxInIndex].CanData.data6 = *(data + 4);
		pTp->TxFrameBuff[pTp->TxInIndex].CanData.data7 = *(data + 5);
		
		pTp->TxParam.SN = 0;
		pTp->TxParam.TotalDataNumber = length;
		pTp->TxParam.CompletedDataNumber = 6;
		pTp->CFDataPionter = data + 6;
		(pTp->TxInIndex >= MAX_BUFF_NUMBER - 1) ? (pTp->TxInIndex = 0) : (pTp->TxInIndex++);
	}
}

/*��������֡*/
void NetworkLayer_SendCF(TP_T* pTp)
{
	uint8_t i;
	uint8_t length;
	
	if(pTp->TxParam.CompletedDataNumber + 7 <= pTp->TxParam.TotalDataNumber)
	{
		length = 7;
	}
	else
	{
		length = pTp->TxParam.TotalDataNumber - pTp->TxParam.CompletedDataNumber;						
	}
	
	pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.N_PciType = CF;
	pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.SF_DL = 	++pTp->TxParam.SN;			//length high 4 bits	
	
	for(i = 0; i < 7; i++)
	{
		if(i < length)
		{
			*(&(pTp->TxFrameBuff[pTp->TxInIndex].CanData.data7) + (6-i)) = *(pTp->CFDataPionter + i);
		}
		else
		{
			*(&(pTp->TxFrameBuff[pTp->TxInIndex].CanData.data7) + (6-i)) = pTp->FrameFillData;
		}
	}

	pTp->TxParam.CompletedDataNumber += length;
	pTp->CFDataPionter += length;
	(pTp->TxInIndex >= MAX_BUFF_NUMBER - 1) ? (pTp->TxInIndex = 0) : (pTp->TxInIndex++);
}

/*��������֡*/
void NetworkLayer_SendFC(TP_T* pTp)
{
	pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.N_PciType = FC;
	pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.SF_DL = pTp->RxParam.FS_Type;
	pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.FF_DL_LOW = pTp->RxParam.BlockSize;
	pTp->TxFrameBuff[pTp->TxInIndex].N_PDU.STmin = pTp->RxParam.STmin;
	pTp->TxFrameBuff[pTp->TxInIndex].CanData.data7= pTp->FrameFillData;
	pTp->TxFrameBuff[pTp->TxInIndex].CanData.data6= pTp->FrameFillData;
	pTp->TxFrameBuff[pTp->TxInIndex].CanData.data5= pTp->FrameFillData;
	pTp->TxFrameBuff[pTp->TxInIndex].CanData.data4= pTp->FrameFillData;
	pTp->TxFrameBuff[pTp->TxInIndex].CanData.data3= pTp->FrameFillData;
	(pTp->TxInIndex >= MAX_BUFF_NUMBER - 1) ? (pTp->TxInIndex = 0) : (pTp->TxInIndex++);
}

/*���յ�֡����*/
void NetworkLayer_RxSF(TP_T* pTp,NetworkFrame RxFrame)
{
	if((RxFrame.N_PDU.SF_DL >= 1) && (RxFrame.N_PDU.SF_DL <= 7))//data length filter
	{
		if(RxFrame.N_PDU.DLC == 8)//DLC filter
		{
			if(pTp->m_DuplexMode == FULL_DUPLEX)
			{
				if(pTp->m_NetworkStatus == NWL_RECIVING)
				{
					pTp->m_N_Result = N_UNEXP_PDU;
					pTp->m_NetworkStatus = NWL_IDLE;//new start of reception
				}
				else if(pTp->m_NetworkStatus == NWL_TRANSMITTING || pTp->m_NetworkStatus == NWL_IDLE)
				{
					pTp->m_N_Result = N_OK;
					pTp->m_NetworkStatus = NWL_IDLE;//only when Full-duplex
				}
				
				pTp->RxDataBuff[0] = RxFrame.CanData.data1;
				pTp->RxDataBuff[1] = RxFrame.CanData.data2;
				pTp->RxDataBuff[2] = RxFrame.CanData.data3;
				pTp->RxDataBuff[3] = RxFrame.CanData.data4;
				pTp->RxDataBuff[4] = RxFrame.CanData.data5;
				pTp->RxDataBuff[5] = RxFrame.CanData.data6;
				pTp->RxDataBuff[6] = RxFrame.CanData.data7;
				N_USData_indication(pTp,SF,  RxFrame.CanData.Mtype , RxFrame.CanData.N_SA , RxFrame.CanData.N_TA , RxFrame.CanData.N_TAtype , RxFrame.CanData.N_AE , pTp->RxDataBuff, RxFrame.N_PDU.SF_DL ,  pTp->m_N_Result);
			}
			else if(pTp->m_DuplexMode == HALF_DUPLEX)//p504 use half duplex
			{
				if(pTp->m_NetworkStatus == NWL_RECIVING)
				{
					pTp->m_N_Result = N_UNEXP_PDU;
					pTp->m_NetworkStatus = NWL_IDLE;//new start of reception
					pTp->RxDataBuff[0] = RxFrame.CanData.data1;
					pTp->RxDataBuff[1] = RxFrame.CanData.data2;
					pTp->RxDataBuff[2] = RxFrame.CanData.data3;
					pTp->RxDataBuff[3] = RxFrame.CanData.data4;
					pTp->RxDataBuff[4] = RxFrame.CanData.data5;
					pTp->RxDataBuff[5] = RxFrame.CanData.data6;
					pTp->RxDataBuff[6] = RxFrame.CanData.data7;
					N_USData_indication(pTp,SF ,  RxFrame.CanData.Mtype , RxFrame.CanData.N_SA , RxFrame.CanData.N_TA , RxFrame.CanData.N_TAtype , RxFrame.CanData.N_AE , pTp->RxDataBuff, RxFrame.N_PDU.SF_DL ,  pTp->m_N_Result);
				}
				else if(pTp->m_NetworkStatus == NWL_IDLE)
				{
					pTp->m_N_Result = N_OK;
					pTp->RxDataBuff[0] = RxFrame.CanData.data1;
					pTp->RxDataBuff[1] = RxFrame.CanData.data2;
					pTp->RxDataBuff[2] = RxFrame.CanData.data3;
					pTp->RxDataBuff[3] = RxFrame.CanData.data4;
					pTp->RxDataBuff[4] = RxFrame.CanData.data5;
					pTp->RxDataBuff[5] = RxFrame.CanData.data6;
					pTp->RxDataBuff[6] = RxFrame.CanData.data7;
					N_USData_indication(pTp,SF ,  RxFrame.CanData.Mtype , RxFrame.CanData.N_SA , RxFrame.CanData.N_TA , RxFrame.CanData.N_TAtype , RxFrame.CanData.N_AE , pTp->RxDataBuff, RxFrame.N_PDU.SF_DL ,  pTp->m_N_Result);
				}
				else if(pTp->m_NetworkStatus == NWL_TRANSMITTING)
				{
					//printf("half duplex mode,ignore SF when transmiting\r\n");
				}
			}
		}
		else
		{
			//printk("SF invalid DLC:%d\r\n", RxFrame.N_PDU.DLC);
		}
	}
	else
	{
		//printk("SF invalid len %d\r\n",RxFrame.N_PDU.SF_DL);
	}
}

/*
������֡����*/
void NetworkLayer_RxFF(TP_T* pTp,NetworkFrame RxFrame)
{
	if(RxFrame.CanData.N_TAtype == PHYSICAL)
	{
		uint16_t FF_DL = (RxFrame.N_PDU.SF_DL << 8) | (RxFrame.N_PDU.FF_DL_LOW);
		
		if(FF_DL >= 8)
		{
			if(RxFrame.N_PDU.DLC == 8)
			{
				pTp->RxDataBuff[0] = RxFrame.CanData.data2;
				pTp->RxDataBuff[1] = RxFrame.CanData.data3;
				pTp->RxDataBuff[2] = RxFrame.CanData.data4;
				pTp->RxDataBuff[3] = RxFrame.CanData.data5;
				pTp->RxDataBuff[4] = RxFrame.CanData.data6;
				pTp->RxDataBuff[5] = RxFrame.CanData.data7;
				if(pTp->m_DuplexMode == FULL_DUPLEX)
				{
					if(pTp->m_NetworkStatus == NWL_IDLE)
					{
						pTp->m_N_Result = N_OK;
						pTp->m_NetworkStatus = NWL_RECIVING;
					}
					else if(pTp->m_NetworkStatus == NWL_RECIVING)
					{
						pTp->m_N_Result = N_UNEXP_PDU;
						pTp->m_NetworkStatus = NWL_RECIVING;//new start of reception
					}
					else if(pTp->m_NetworkStatus == NWL_TRANSMITTING)
					{
						pTp->m_N_Result = N_OK;
						pTp->m_NetworkStatus = NWL_RECIVING;//only when Full-duplex
					}

					if(pTp->RxParam.BuffSize < FF_DL)
					{
						//printk("Rx FF error size overflow FF_DL:%d,buffsize:%d\r\n",FF_DL,RxParam.BuffSize);
						pTp->RxParam.FS_Type = OVFLW;
						NetworkLayer_SendFC(pTp);
					}
					else
					{
						pTp->RxParam.SN = 0x0;
						pTp->RxParam.FS_Type = CTS;
						N_USData_indication(pTp,FF , RxFrame.N_PDU.Mtype , RxFrame.N_PDU.N_SA , RxFrame.N_PDU.N_TA , RxFrame.N_PDU.N_TAtype , RxFrame.N_PDU.N_AE , pTp->RxDataBuff , FF_DL ,  pTp->m_N_Result);
					}
				}
				else if(pTp->m_DuplexMode == HALF_DUPLEX)//p504 use half duplex
				{
					if(pTp->m_NetworkStatus == NWL_IDLE)
					{
						pTp->m_N_Result = N_OK;
						pTp->m_NetworkStatus = NWL_RECIVING;
						if(pTp->RxParam.BuffSize < FF_DL)
						{
							//printk("Rx FF error size overflow FF_DL:%d,buffsize:%d\r\n",FF_DL,RxParam.BuffSize);
							pTp->RxParam.FS_Type = OVFLW;
							NetworkLayer_SendFC(pTp);
						}
						else
						{
							pTp->RxParam.SN = 0x0;
							pTp->RxParam.FS_Type = CTS;
							N_USData_indication(pTp,FF , RxFrame.N_PDU.Mtype , RxFrame.N_PDU.N_SA , RxFrame.N_PDU.N_TA , RxFrame.N_PDU.N_TAtype , RxFrame.N_PDU.N_AE , pTp->RxDataBuff , FF_DL ,  pTp->m_N_Result);
						}
					}
					else if(pTp->m_NetworkStatus == NWL_RECIVING)
					{
						pTp->m_N_Result = N_UNEXP_PDU;
						pTp->m_NetworkStatus = NWL_RECIVING;//new start of reception
						if(pTp->RxParam.BuffSize < FF_DL)
						{
							//printf("FF size overflow :%d expect:%d\r\n",FF_DL,RxParam.BuffSize);
							pTp->RxParam.FS_Type = OVFLW;
							NetworkLayer_SendFC(pTp);
						}
						else
						{
							pTp->RxParam.SN = 0x0;
							pTp->RxParam.FS_Type = CTS;
							N_USData_indication(pTp,FF , RxFrame.N_PDU.Mtype , RxFrame.N_PDU.N_SA , RxFrame.N_PDU.N_TA , RxFrame.N_PDU.N_TAtype , RxFrame.N_PDU.N_AE , pTp->RxDataBuff , FF_DL ,  pTp->m_N_Result);
						}
					}
					else if(pTp->m_NetworkStatus == NWL_TRANSMITTING)
					{
						//printf("half duplex ,ignore FF when TX\r\n");
					}
				}
			}
			else
			{
				//printf("FF invalid DLC : %d\r\n",RxFrame.N_PDU.DLC);
			}
		}
		else
		{
			//printf("FF invalid len %d\r\n",FF_DL);
		}
	}
	else if(RxFrame.CanData.N_TAtype == FUNCTIONAL)
	{
		//printf("ignore functional  FF\r\n");
	}
}

/*��������֡����*/
void NetworkLayer_RxCF(TP_T* pTp,NetworkFrame RxFrame)
{
	if(RxFrame.N_PDU.DLC == 8)
	{
		if(pTp->m_NetworkStatus == NWL_RECIVING)
		{
			uint8_t CurrFrameLength = 0;
			if(((pTp->RxParam.SN + 1) & 0x0F) == (RxFrame.N_PDU.SF_DL & 0x0F))
			{
				pTp->m_N_Result = N_OK;
				if(pTp->RxParam.CompletedDataNumber < pTp->RxParam.TotalDataNumber)
				{
					pTp->RxParam.SN = RxFrame.N_PDU.SF_DL;
					pTp->RxDataBuff[0] = RxFrame.CanData.data1;
					pTp->RxDataBuff[1] = RxFrame.CanData.data2;
					pTp->RxDataBuff[2] = RxFrame.CanData.data3;
					pTp->RxDataBuff[3] = RxFrame.CanData.data4;
					pTp->RxDataBuff[4] = RxFrame.CanData.data5;
					pTp->RxDataBuff[5] = RxFrame.CanData.data6;
					pTp->RxDataBuff[6] = RxFrame.CanData.data7;
					
					if(pTp->RxParam.CompletedDataNumber + 7 < pTp->RxParam.TotalDataNumber)
					{
						CurrFrameLength = 7;
					}
					else
					{
						CurrFrameLength = pTp->RxParam.TotalDataNumber - pTp->RxParam.CompletedDataNumber;
					}

					N_USData_indication(pTp,CF, RxFrame.N_PDU.Mtype , RxFrame.N_PDU.N_SA , RxFrame.N_PDU.N_TA , RxFrame.N_PDU.N_TAtype , RxFrame.N_PDU.N_AE , pTp->RxDataBuff , CurrFrameLength ,  pTp->m_N_Result);
				}
				else 
				{
					//printf("RX end,ignore CF\r\n");
				}
			}
			else
			{
				//printf("CF error SN expect %d but %d\r\n", RxParam.SN + 1,RxFrame.N_PDU.SF_DL);
				pTp->m_N_Result = N_WRONG_SN;
				N_USData_indication(pTp,CF, RxFrame.N_PDU.Mtype , RxFrame.N_PDU.N_SA , RxFrame.N_PDU.N_TA , RxFrame.N_PDU.N_TAtype , RxFrame.N_PDU.N_AE , pTp->RxDataBuff , CurrFrameLength ,  pTp->m_N_Result);
			}
		}
		else
		{
			//printf("not in RX ,ignore CF : %d\r\n");
			//printf("not in RX ,ignore CF : %d\r\n",RxFrame.N_PDU.SF_DL);
		}
	}
	else
	{
		//printf("CF invalid DLC : %d\r\n",RxFrame.N_PDU.DLC);
	}
}

/*������������*/
void NetworkLayer_RxFC(TP_T* pTp,NetworkFrame RxFrame)
{
	if(RxFrame.CanData.N_TAtype == PHYSICAL)
	{
		if(pTp->m_NetworkStatus == NWL_TRANSMITTING || pTp->m_NetworkStatus == NWL_WAIT)
		{
			if(RxFrame.N_PDU.DLC == 8)//iso 15765 7.4.2
			{
				if(RxFrame.N_PDU.SF_DL <= OVFLW)
				{
					pTp->TxParam.FS_Type = (FS_Type)RxFrame.N_PDU.SF_DL;
				}
				else
				{
					pTp->m_N_Result = N_INVALID_FS;
					//printf("FC wrong FS : %d\r\n",RxFrame.N_PDU.SF_DL);
				}

				pTp->TxParam.BlockSize =RxFrame.N_PDU.FF_DL_LOW;
				
				if((RxFrame.N_PDU.STmin >= 0x80 && RxFrame.N_PDU.STmin <= 0xF0) || (RxFrame.N_PDU.STmin >= 0xFA))
				{
					pTp->TxParam.STmin = 0x7F;
				}
				else if(RxFrame.N_PDU.STmin >= 0xF1 && RxFrame.N_PDU.STmin <= 0xF9)
				{
					pTp->TxParam.STmin = 0x1;
				}
				else
				{
					pTp->TxParam.STmin = RxFrame.N_PDU.STmin;
				}

				N_USData_indication(pTp,FC , pTp->m_AddressFormat.Mtype , pTp->m_AddressFormat.N_SA , pTp->m_AddressFormat.N_TA , pTp->m_AddressFormat.N_TAtype , pTp->m_AddressFormat.N_AE , (void*)0, 0 ,  pTp->m_N_Result);
			
			}
			else
			{
				//printf("FC invalid DLC:%d\r\n",RxFrame.N_PDU.DLC);
			}
		}
		else 
		{
			pTp->m_TimePeriod.FC_RxBeforeFFOnSender = true;
			pTp->m_N_Result = N_TIMEOUT_Bs;
			N_USData_confirm(pTp,FF , pTp->m_AddressFormat.Mtype , pTp->m_AddressFormat.N_SA , pTp->m_AddressFormat.N_TA , pTp->m_AddressFormat.N_TAtype , pTp->m_AddressFormat.N_AE , pTp->m_N_Result);
			//printf("not in TX or wait ,ignore FC \r\n");
		}
	}
	else if(RxFrame.CanData.N_TAtype == FUNCTIONAL)
	{
		//printf("ignore functional  FC\r\n");
	}
}

/*
���մ���
*/
void NetworkLayer_RxProc(TP_T* pTp)
{
	if(!IsRxBuffEmpty(pTp))
	{
		N_PCIType PciType = pTp->RxFrameBuff[pTp->RxOutIndex].N_PDU.N_PciType;
		if(PciType == SF)//SF frame
		{
			NetworkLayer_RxSF(pTp,pTp->RxFrameBuff[pTp->RxOutIndex]);			
		}
		else if(PciType == FF)
		{
			NetworkLayer_RxFF(pTp,pTp->RxFrameBuff[pTp->RxOutIndex]);
		}
		else if(PciType == CF)
		{
			NetworkLayer_RxCF(pTp,pTp->RxFrameBuff[pTp->RxOutIndex]);
		}
		else if(PciType == FC)
		{
			NetworkLayer_RxFC(pTp,pTp->RxFrameBuff[pTp->RxOutIndex]);
		}
		else
		{
			pTp->m_N_Result = N_UNEXP_PDU;
		}
		(pTp->RxOutIndex == MAX_BUFF_NUMBER - 1) ? (pTp->RxOutIndex = 0) : (pTp->RxOutIndex++);
	}
}

/*
��������CAN����
ID��can id
data:can���ݳ�
IDE:�Ƿ���չID
RTR:�Ƿ�Զ��֡
*/
void NetworkLayer_RxFrame(TP_T* pTp,uint32_t ID,uint8_t* data,uint8_t IDE,uint8_t DLC,uint8_t RTR)
{
	if(pTp == NULL)
	{
		return;
	}
	if(ID == pTp->m_PyhReqID || ID == pTp->m_FunReqID)
	{
		uint8_t i;
		NetworkFrame TempFrame;
		for(i = 0 ; i < DLC; i ++) 
		{
			*(&(TempFrame.CanData.data7) + (7 -i)) = *(data + i);
		}
		
		TempFrame.CanData.ID = ID;
		TempFrame.CanData.DLC = DLC;
		TempFrame.CanData.RTR = RTR;
		TempFrame.CanData.IDE = IDE;
		TempFrame.CanData.N_SA = (uint8_t)(ID >> 8);
		TempFrame.CanData.N_TA = (uint8_t)ID;
		(ID == pTp->m_FunReqID) ? (TempFrame.CanData.N_TAtype = FUNCTIONAL) : (TempFrame.CanData.N_TAtype = PHYSICAL);
		//�������
		if(IsRxBuffFull(pTp))
		{
			pTp->RxOutIndex++;
		}
		pTp->RxFrameBuff[pTp->RxInIndex] = TempFrame;
		(pTp->RxInIndex >= MAX_BUFF_NUMBER - 1) ? (pTp->RxInIndex = 0) : (pTp->RxInIndex++);
	}
}

/*��ʼ��������֡����*/
void NetworkLayer_RxStart(TP_T* pTp)
{
	DiagTimer_Set(&pTp->ReciveTimer,pTp->m_TimePeriod.N_Br);
	pTp->m_NetworkStatus = NWL_RECIVING;
	pTp->m_RxStep = RX_WAIT_FC_REQ;
	pTp->RxParam.CompletedDataNumber = 0;
	pTp->RxParam.CompletedNumberInBlock = 0;
}

/*��������*/
void NetworkLayer_RxEnd(TP_T* pTp)
{
	//printk("RX end\r\n");
	pTp->m_RxStep = RX_IDLE;
	pTp->m_NetworkStatus = NWL_IDLE;
	pTp->RxParam.TotalDataNumber = 0;
	pTp->RxParam.CompletedDataNumber = 0;
	pTp->RxParam.CompletedNumberInBlock = 0;
}

/*֪ͨ�ϲ�*/
void NetworkLayer_NotifyToUpperLayer(TP_T* pTp,NetworkNotification notify)
{
	if(!IsIndicationListFull(pTp))
	{
		pTp->IndicationList[pTp->IndicationInIndex] = notify;
		(pTp->IndicationInIndex >= MAX_INDICATION_NUMBER -1) ? (pTp->IndicationInIndex = 0) : pTp->IndicationInIndex++;
	}
}

/*
��������ȷ��
PciType��
*/
void N_USData_confirm(TP_T* pTp,N_PCIType PciType,MType Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtype N_TAtype, uint8_t N_AE, N_Result N_Result)//interface confirm to upper layer
{
	if(PciType == FF)
	{
		//��֡����ȷ��
		pTp->m_TimePeriod.FF_ConfirmedOnSender = true;
	}
	else if(PciType == CF)
	{
		//����֡����ȷ��
		pTp->TxParam.CompletedNumberInBlock++;
		pTp->m_TimePeriod.CF_ConfirmedOnSender = true;
		if(pTp->TxParam.TotalDataNumber == pTp->TxParam.CompletedDataNumber)
		{
			NetworkNotification notify;
			notify.NotificationType = CONFIRM;
			notify.Mtype = Mtype;
			notify.N_SA = N_SA;
			notify.N_TA = N_TA;
			notify.N_TAtype = N_TAtype;
			notify.N_AE = N_AE;
			notify.N_Resut = N_Result;
			notify.valid = true;
			NetworkLayer_NotifyToUpperLayer(pTp,notify);
		}
	}
	else if(PciType == FC)
	{
		//����֡����ȷ��
		if(pTp->m_RxStep == RX_WAIT_FC_CONF)
		{
			pTp->m_TimePeriod.FC_ConfirmedOnReciver = true;
		}
		else
		{
			//printk("send FC on wrong time\r\n");
		}
	}
	else if(PciType == SF)
	{
		//��֡����ȷ��
		NetworkNotification notify;
		notify.NotificationType = CONFIRM;
		notify.Mtype = Mtype;
		notify.N_SA = N_SA;
		notify.N_TA = N_TA;
		notify.N_TAtype = N_TAtype;
		notify.N_AE = N_AE;
		notify.N_Resut = N_Result;
		notify.valid = true;
		NetworkLayer_NotifyToUpperLayer(pTp,notify);
	}
}

void N_USData_indication(TP_T* pTp,N_PCIType PciType,MType Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtype N_TAtype, uint8_t N_AE, uint8_t* MessageData, uint16_t Length, N_Result N_Result)
{
	uint8_t i;
	NetworkNotification notify;
	if(PciType == FC)
	{
		if(N_Result == N_OK)
		{
			if(pTp->m_TxStep == TX_WAIT_FC)
			{
				////printk("Rx FC type : %d\r\n",TxParam.FS_Type);
				pTp->TxParam.CompletedNumberInBlock = 0;
				
				if(pTp->TxParam.FS_Type == CTS)
				{
					pTp->m_NetworkStatus = NWL_TRANSMITTING;
					pTp->m_TimePeriod.FC_RecivedOnSender = true;
				}
				else if(pTp->TxParam.FS_Type == WT)
				{
					pTp->m_NetworkStatus = NWL_WAIT;
					DiagTimer_Set(&pTp->SendTimer,pTp->m_TimePeriod.N_Bs);
					//printk("Rx Fs wait\r\n");
				}
				else if(pTp->TxParam.FS_Type == OVFLW)
				{
					//printk("rx fs overflow,terminal transmitr\n");
					NetworkLayer_TxEnd(pTp);
				}
			}
			else
			{
				
			}
		}
		else
		{
			NetworkLayer_TxEnd(pTp);
		}
	}
	else if(PciType == CF)
	{
		if(N_Result == N_OK)
		{
			pTp->RxParam.CompletedNumberInBlock++;
			pTp->m_TimePeriod.CF_RecivedOnReciver = true;
			for(i = 0 ; i < Length ; i ++)
			{
				pTp->NetworkDataBufRx[pTp->RxParam.CompletedDataNumber++] = *(MessageData + i);
			}

			if(pTp->RxParam.CompletedDataNumber == pTp->RxParam.TotalDataNumber)
			{
				notify.NotificationType = INDICATION;
				notify.Mtype = Mtype;
				notify.N_SA = N_SA;
				notify.N_TA = N_TA;
				notify.N_TAtype = N_TAtype;
				notify.N_AE = N_AE;
				notify.N_Resut = N_Result;
				notify.MessageData = pTp->NetworkDataBufRx;
				notify.length = pTp->RxParam.TotalDataNumber;
				notify.valid = true;
				NetworkLayer_NotifyToUpperLayer(pTp,notify);
				NetworkLayer_RxEnd(pTp);
			}
		}
		else
		{
			NetworkLayer_RxEnd(pTp);
		}
	}
	else if(PciType == FF)
	{
		notify.NotificationType = FF_INDICATION;
		notify.Mtype = Mtype;
		notify.N_SA = N_SA;
		notify.N_TA = N_TA;
		notify.N_TAtype = N_TAtype;
		notify.N_AE = N_AE;
		notify.valid = true;
		notify.length = Length;
		NetworkLayer_NotifyToUpperLayer(pTp,notify);
		pTp->RxParam.TotalDataNumber = Length;
		NetworkLayer_RxStart(pTp);
		for(i = 0 ; i < 6 ; i ++)
		{
			pTp->NetworkDataBufRx[pTp->RxParam.CompletedDataNumber++] = *(MessageData + i);
		}
	}
	else if(PciType == SF)
	{
		if(pTp->m_N_Result == N_UNEXP_PDU)//Request-interrupt-by-SF
		{
			NetworkLayer_RxEnd(pTp);
		}
		notify.NotificationType = INDICATION;
		notify.Mtype = Mtype;
		notify.N_SA = N_SA;
		notify.N_TA = N_TA;
		notify.N_TAtype = N_TAtype;
		notify.N_AE = N_AE;
		notify.N_Resut = N_Result;
		notify.MessageData = MessageData;
		notify.length = Length;
		notify.valid = true;
		NetworkLayer_NotifyToUpperLayer(pTp,notify);
	}
}

void N_ChangeParameter_request(TP_T* pTp,MType Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtype N_TAtype, uint8_t N_AE, Parameter Parameter, uint8_t Parameter_Value)
{
	if(pTp == NULL)
	{
		return;
	}
	Result_ChangeParameter result;
	if(Parameter == STmin)
	{
		pTp->RxParam.STmin = Parameter_Value;
		result = N_CHANGE_OK;
	}
	else if(Parameter == BS)
	{
		pTp->RxParam.BlockSize = Parameter_Value;
		result = N_CHANGE_OK; 
	}
	else
	{
		result = N_WRONG_PARAMETER;
	}
}

/*�����������*/
void NetworkLayer_MainProc(TP_T* pTp)
{
	//����������
	if(pTp->m_NetworkStatus == NWL_TRANSMITTING)
	{
		//��֡����ȷ��
		if(pTp->m_TxStep == TX_WAIT_FF_CONF)
		{
			//��֡�������
			if(pTp->m_TimePeriod.FF_ConfirmedOnSender == true)
			{
				pTp->m_TxStep  = TX_WAIT_FC;
				DiagTimer_Set(&pTp->SendTimer,pTp->m_TimePeriod.N_Bs);
				pTp->m_TimePeriod.FF_ConfirmedOnSender = false;
			}
			else
			{
				//��֡���ͳ�ʱ�ж�
				if(DiagTimer_HasExpired(&pTp->SendTimer))
				{
					NetworkLayer_TxEnd(pTp);
					pTp->m_N_Result = N_TIMEOUT_A;
					N_USData_confirm(pTp,FF , pTp->m_AddressFormat.Mtype , pTp->m_AddressFormat.N_SA , pTp->m_AddressFormat.N_TA , pTp->m_AddressFormat.N_TAtype , pTp->m_AddressFormat.N_AE , pTp->m_N_Result);
				}
			}
		}
		//����֡����ȷ��
		else if(pTp->m_TxStep == TX_WAIT_FC)
		{
			//��������֡���
			if(pTp->m_TimePeriod.FC_RecivedOnSender == true)
			{
				pTp->m_TxStep  = TX_WAIT_CF_REQ;
				DiagTimer_Set(&pTp->SendTimer,pTp->m_TimePeriod.N_Cs);
				DiagTimer_Set(&pTp->CFRequestTimer , 0);
				pTp->m_TimePeriod.FC_RecivedOnSender = false;
			}
			else
			{
				//��������֡��ʱ�ж�
				if(DiagTimer_HasExpired(&pTp->SendTimer))
				{
					NetworkLayer_TxEnd(pTp);
					pTp->m_N_Result = N_TIMEOUT_Bs;
					N_USData_confirm(pTp,FF , pTp->m_AddressFormat.Mtype , pTp->m_AddressFormat.N_SA , pTp->m_AddressFormat.N_TA , pTp->m_AddressFormat.N_TAtype , pTp->m_AddressFormat.N_AE , pTp->m_N_Result);
				}
			}
		}
		//��������֮֡ǰ�ȴ�ʱ��
		else if(pTp->m_TxStep == TX_WAIT_CF_REQ)
		{
			if(DiagTimer_HasExpired(&pTp->CFRequestTimer))//when first CF or wait STmin end
			{
				NetworkLayer_SendCF(pTp);
				pTp->m_TxStep = TX_WAIT_CF_CONF;
				DiagTimer_Set(&pTp->SendTimer,pTp->m_TimePeriod.N_As);
			}
		}
		//����֡����
		else if(pTp->m_TxStep == TX_WAIT_CF_CONF)
		{
			if(pTp->m_TimePeriod.CF_ConfirmedOnSender == true)
			{
				DiagTimer_Set(&pTp->CFRequestTimer , pTp->TxParam.STmin + 1);
				pTp->m_TimePeriod.CF_ConfirmedOnSender  = false;
				if(pTp->TxParam.CompletedDataNumber < pTp->TxParam.TotalDataNumber)
				{
					if(pTp->TxParam.CompletedNumberInBlock < pTp->TxParam.BlockSize || pTp->TxParam.BlockSize == 0)
					{
						//�ȴ�������֡
						pTp->m_TxStep  = TX_WAIT_CF_REQ;
						DiagTimer_Set(&pTp->SendTimer,pTp->m_TimePeriod.N_Cs);
					}
					else
					{
						//�ȴ�����֡
						pTp->m_TxStep = TX_WAIT_FC;
						DiagTimer_Set(&pTp->SendTimer,pTp->m_TimePeriod.N_Bs);
					}
				}
				else
				{
					//�������
					NetworkLayer_TxEnd(pTp);//normal end
				}		
			}
			else
			{
				//��ʱ����
				if(DiagTimer_HasExpired(&pTp->SendTimer))
				{
					NetworkLayer_TxEnd(pTp);//time out end
				}
			}
		}
	}
	//��������
	else if(pTp->m_NetworkStatus == NWL_RECIVING)
	{
		//�ȴ���������֡
		if(pTp->m_RxStep == RX_WAIT_FC_REQ)
		{
			if(DiagTimer_HasExpired(&pTp->ReciveTimer))
			{
				NetworkLayer_SendFC(pTp);
				pTp->m_RxStep = RX_WAIT_FC_CONF;
				DiagTimer_Set(&pTp->ReciveTimer,pTp->m_TimePeriod.N_Ar);
			}
		}
		//����֡����ȷ��
		else if(pTp->m_RxStep == RX_WAIT_FC_CONF)
		{
			if(pTp->m_TimePeriod.FC_ConfirmedOnReciver == true)
			{
				pTp->m_RxStep  = RX_WAIT_CF;
				DiagTimer_Set(&pTp->ReciveTimer,pTp->m_TimePeriod.N_Cr);
				pTp->m_TimePeriod.FC_ConfirmedOnReciver = false;
			}
			else
			{
				if(DiagTimer_HasExpired(&pTp->ReciveTimer))
				{
					//printf("timeout wait FC conf\r\n");
					NetworkLayer_RxEnd(pTp);//time out end 
					pTp->m_N_Result = N_TIMEOUT_A;
					N_USData_indication(pTp,FC , pTp->m_AddressFormat.Mtype , pTp->m_AddressFormat.N_SA , pTp->m_AddressFormat.N_TA , pTp->m_AddressFormat.N_TAtype , pTp->m_AddressFormat.N_AE , (void*)0, 0 ,  pTp->m_N_Result);
				}
			}
		}
		//��������֡
		else if(pTp->m_RxStep == RX_WAIT_CF)
		{
			if(pTp->m_TimePeriod.CF_RecivedOnReciver == true)
			{
				pTp->m_TimePeriod.CF_RecivedOnReciver = false;
				if(pTp->RxParam.CompletedDataNumber < pTp->RxParam.TotalDataNumber)
				{
					if(pTp->RxParam.CompletedNumberInBlock < pTp->RxParam.BlockSize || pTp->RxParam.BlockSize == 0)
					{
						pTp->m_RxStep  = RX_WAIT_CF;
						DiagTimer_Set(&pTp->ReciveTimer,pTp->m_TimePeriod.N_Cr);
					}
					else 
					{
						//printf("block end %d >= %d\r\n",RxParam.CompletedNumberInBlock,RxParam.BlockSize);
						pTp->RxParam.CompletedNumberInBlock = 0;
						//RxParam.SN = 0;
						pTp->m_RxStep  = RX_WAIT_FC_REQ;
						DiagTimer_Set(&pTp->ReciveTimer,pTp->m_TimePeriod.N_Br);
					}
				}
				else
				{
					NetworkLayer_RxEnd(pTp);//normal end
				}
			}
			else
			{
				if(DiagTimer_HasExpired(&pTp->ReciveTimer))
				{
					//printf("timeout wait CF\r\n");
					NetworkLayer_RxEnd(pTp);//time out end
					pTp->m_N_Result = N_TIMEOUT_Cr;
					N_USData_indication(pTp,CF , pTp->m_AddressFormat.Mtype , pTp->m_AddressFormat.N_SA , pTp->m_AddressFormat.N_TA , pTp->m_AddressFormat.N_TAtype , pTp->m_AddressFormat.N_AE , (void*)0, 0 ,  pTp->m_N_Result);
				}
			}
		}
	}
	//�ȴ�״̬
	else if(pTp->m_NetworkStatus == NWL_WAIT)
	{
		//�ȴ�����
		if(pTp->m_TxStep == TX_WAIT_FC)
		{
			//�ȴ����س�ʱ
			if(DiagTimer_HasExpired(&pTp->SendTimer))
			{
				//printf("FC Wait time out : FC with FS = 0 not recive\r\n");
				NetworkLayer_TxEnd(pTp);//time out end
				pTp->m_N_Result = N_TIMEOUT_Bs;
				N_USData_confirm(pTp,FF , pTp->m_AddressFormat.Mtype , pTp->m_AddressFormat.N_SA , pTp->m_AddressFormat.N_TA , pTp->m_AddressFormat.N_TAtype , pTp->m_AddressFormat.N_AE , pTp->m_N_Result);
			}
		}
	}
}

/*
����������ʼ��
��������ID����������ID�����ͺ���
*/
void NetworkLayer_InitParam(TP_T* pTp,uint32_t PyhReqID,uint32_t FunReqID, uint32_t ResponseID,SendCANFun sendFun,GetMsTickCountFun GetMsTickCntFun)
{
	if(pTp == NULL)
	{
		return;
	}
	//set the default value to 0xAA
	pTp->FrameFillData = 0xAA;
	pTp->m_PyhReqID = PyhReqID;
	pTp->m_FunReqID = FunReqID;
	pTp->m_ResponseID = ResponseID;
	pTp->RxInIndex = 0;
	pTp->RxOutIndex = 0;
	pTp->TxInIndex = 0;
	pTp->TxOutIndex = 0;
	pTp->IndicationInIndex = 0;
	pTp->IndicationOutIndex = 0;
	
	pTp->m_TimePeriod.N_As = 25;
	pTp->m_TimePeriod.N_Ar = 25;
	pTp->m_TimePeriod.N_Br = 50;
	pTp->m_TimePeriod.N_Bs = 75;
	pTp->m_TimePeriod.N_Cs = 25;
	pTp->m_TimePeriod.N_Cr = 150;

	pTp->TxParam.FS_Type = CTS;
	pTp->TxParam.BlockSize = 20;
	pTp->TxParam.CompletedNumberInBlock = 0;
	pTp->TxParam.STmin = 100;
	pTp->TxParam.SN = 0;
	pTp->TxParam.TotalDataNumber = 100;
	pTp->TxParam.CompletedDataNumber = 0;

	pTp->RxParam.FS_Type = CTS;
	pTp->RxParam.BlockSize = 0;
	pTp->RxParam.CompletedNumberInBlock = 0;
	pTp->RxParam.STmin = 0x0A;
	pTp->RxParam.SN = 0;
	pTp->RxParam.TotalDataNumber = 0;
	pTp->RxParam.CompletedDataNumber = 0;	
	pTp->RxParam.BuffSize = MAX_DOWNLOADING_BUF;

	pTp->m_DuplexMode = HALF_DUPLEX;
	pTp->NetworkSend= sendFun;
	DiagTimerGetMsTickCntFun = GetMsTickCntFun;
}

/*��������ö�ʱ����*/
void NetworkLayer_SetParam(TP_T* pTp,uint8_t TimeAs, uint8_t TimeBs, uint8_t TimeCr, uint8_t TimeAr, uint8_t TimeBr, uint8_t TimeCs, 
	uint8_t Bs, uint8_t m_STmin, DuplexMode nDuplex ,  MType Mtype , uint8_t N_SA , uint8_t N_TA , N_TAtype N_TAtype , uint8_t N_AE , uint8_t FillData)
{
	if(pTp == NULL)
	{
		return;
	}
	pTp->m_TimePeriod.N_As = TimeAs;
	pTp->m_TimePeriod.N_Ar = TimeAr;
	pTp->m_TimePeriod.N_Br = TimeBr;
	pTp->m_TimePeriod.N_Bs = TimeBs;
	pTp->m_TimePeriod.N_Cs = TimeCs;
	pTp->m_TimePeriod.N_Cr = TimeCr;
	pTp->m_DuplexMode = nDuplex;
	pTp->RxParam.STmin = m_STmin;
	pTp->RxParam.BlockSize = Bs;
	pTp->m_AddressFormat.Mtype = Mtype;
	pTp->m_AddressFormat.N_SA = N_SA;
	pTp->m_AddressFormat.N_TA = N_TA;
	pTp->m_AddressFormat.N_TAtype = N_TAtype;
	pTp->m_AddressFormat.N_AE = N_AE;
	pTp->FrameFillData = FillData;
}

/*��������*/
void N_USData_request(TP_T* pTp,MType Mtype, uint8_t N_SA, uint8_t N_TA, N_TAtype N_TAtype, uint8_t N_AE, uint8_t* MessageData, uint16_t Length)//interface request for upper layer
{
	if(pTp == NULL)
	{
		return;
	}
	pTp->m_AddressFormat.Mtype = Mtype;
	pTp->m_AddressFormat.N_SA = N_SA;
	pTp->m_AddressFormat.N_TA = N_TA;
	pTp->m_AddressFormat.N_TAtype = N_TAtype;
	pTp->m_AddressFormat.N_AE = N_AE;
	//printf("diag req SA = %x,TA = %x,length = %x,TAtype = %d\r\n",N_SA,N_TA,Length,N_TAtype);
	if(Mtype == DIAGNOSTIC)
	{
		if(Length <= 7)
		{
			NetworkLayer_SendSF(pTp,(uint8_t)Length,MessageData);
		}
		else
		{
			NetworkLayer_SendFF(pTp,Length,MessageData);
			NetworkLayer_TxStart(pTp);
		}
	}
}

/*ָʾ�б��*/
bool IsIndicationListEmpty(TP_T* pTp)
{
	if(pTp == NULL)
	{
		return true;
	}
	return (pTp->IndicationInIndex == pTp->IndicationOutIndex);
}

/*ָʾ�б���*/
bool IsIndicationListFull(TP_T* pTp)
{
	return (pTp->IndicationInIndex == (MAX_INDICATION_NUMBER - 1) &&  pTp->IndicationOutIndex == 0) || (pTp->IndicationOutIndex == pTp->IndicationInIndex+1);
}

/*��ȡָʾ*/
NetworkNotification PullIndication(TP_T* pTp)
{
	uint8_t tempOutIndex = pTp->IndicationOutIndex;
	(pTp->IndicationOutIndex >= MAX_INDICATION_NUMBER - 1) ? (pTp->IndicationOutIndex = 0) : (pTp->IndicationOutIndex++);
	return pTp->IndicationList[tempOutIndex];
}

//����㴦��
void NetworkLayer_Proc(TP_T* pTp)
{
	if(pTp == NULL)
	{
		return;
	}
	NetworkLayer_RxProc(pTp);
	NetworkLayer_MainProc(pTp);
	NetworkLayer_TxProc(pTp);
}
