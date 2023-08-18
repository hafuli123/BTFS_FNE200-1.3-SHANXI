/***************************************************  ��ذ�ȫԤ���㷨���������ӿ�ͷ�ļ����� *******************************************************/
/* ��������������ʼ������initialFunction������1����battsafetyFunction1������2����battsafetyFunction2������3����battsafetyFunction3 */
/* �����ӿڽṹ��YJ_MESSAGE���壺
	(1) �������룺UTC�뼶ʱ�������ذ�����������������ذ����״̬����ذ��ܵ�ѹ����ذ�SOC����ذ���������ذ������ѹ���顣
	(2) �������͵ļ����� */
/* �������ù�̣�
	(1) ��ʼ������initialFunction�����ն��ϵ�ʱ����һ�Σ����ڳ�ʼ�������ӿڵĽṹ�壬��������0�����󷵻ط�0��
	(2) ��ʼ���ɹ��󣬶�ȡFlash�������ݣ�Flash�ṹ������˵�����£�
	      1���ṹ��Flash_DATA1��������2������3���ն�ÿ���µ�ʱ��Ҫ�洢�ṹ��Flash_DATA1�����е����ݣ�Ȼ���ն�ÿ���ϵ�ʱ��Ҫͨ���ṹ��Flash_DATA1�Ѵ洢�����ݶ�ȡ����;
	      2�� �ṹ��Flash_DATA2��������2��Flash_DATA2��д������һ�������������ǳ������״̬��0Ϊ�ŵ�/1Ϊ��磩��ת�䣬��д����μ�����PDF��
	(3) Flash��ȡ��ɺ����ε�������1~����3�������߼�Ϊ���ȵ�������1����battsafetyFunction1������1����������ɺ��������2����battsafetyFunction2������2����������ɺ��������3����battsafetyFunction3��
	(4) ��������1~3���ü�����ɺ�ͳһ�ϱ������ļ�������
	(5) ���������ĵ���ʱ������Ϊ5�롣  */
/**********************************************   Edison Wei,  March 2022 in Shenzhen      **************************************************************/

#include "stdint.h"

// ���ļ��汾�ţ����ڲ���flash���ݣ�ǰ������ֵ��һ�¶���Ϊ����Flash���ϴγ���Ϊ0����ǰ����Ϊ0
#define Cache_Version	0 										

// ����̻���������������޸�
#define MAX_CELL_CNT 104      							
#define MAX_Fault_CELL_CNT 5   						
#define MAX_diffocv_CNT 40 								
#define Char_CNT 5 		
#define Panduan_base_CNT 5 
#define Panduan_yujing_error_TimeCNT 40
#define UTC_CNT_PanduanBase 3
#define Max_Data_Save_CNT 250

// �����ӿڽṹ��
typedef struct {
	uint32_t UTC_TIME;														// �������룺UTCʱ������ֱ��ʣ���
	uint8_t Nbat_s;																// �������룺��ذ�����������������Чֵ��1~MAX_CELL_CNT
	uint8_t Charge_state;													// �������룺���״̬��־λ����Чֵ0��1��1��ʾ����У�0��ʾΪ�ǳ��״̬
	uint8_t SOC;																	// �������룺��ذ�SOC����λΪ%����Чֵ��0~110
	float Current;																// �������룺��ذ���������λΪA���ŵ�Ϊ��
	float PackVoltage;															// �������룺��ذ���ѹ����λΪV
	float* cellV;																	// �������룺�����ѹ���飬��λΪV��ά��������MAX_CELL_CNT
	uint8_t SWD_VERSION;													// �����Ƴ����㷨�汾�ţ���Чֵ��100~250����1.0.0~2.5.0
	uint8_t life1;   																// �����Ƴ������̲�������Чֵ��0~250
	uint8_t life2;   																// �����Ƴ������̲�������Чֵ��0~250
	uint8_t life3;   																// �����Ƴ������̲�������Чֵ��0~250
	uint8_t fault_OCVID;   													// �����Ƴ������̲�������Чֵ��0~100
	uint16_t mean_OCV;														// �����Ƴ������̲�������Чֵ��0~5000
	uint16_t OCV_interval_CNT;											// �����Ƴ������̲�������Чֵ��0~65500
	uint8_t fault_state1;   													// �����Ƴ��������������Чֵ��0����1��0��ʾ����,1��ʾ�쳣
	uint8_t fault_state2;   													// �����Ƴ��������������Чֵ��0����1��0��ʾ����,1��ʾ�쳣
	uint8_t fault_state3;   													// �����Ƴ��������������Чֵ��0����1��0��ʾ����,1��ʾ�쳣
	uint8_t fault_state4;   													// �����Ƴ��������������Чֵ��0����1��0��ʾ����,1��ʾ�쳣
	uint8_t fault_state5;   													// �����Ƴ��������������Чֵ��0����1��0��ʾ����,1��ʾ�쳣
	uint8_t fault_cell_number;											// �����Ƴ��������������Чֵ��0~5
	uint8_t fault_cell_ID[MAX_Fault_CELL_CNT];			// �����Ƴ������������1��5�����飬������ÿһ��Ԫ�ص���Чֵ��0~250
	uint8_t fault_cell_Type[MAX_Fault_CELL_CNT];		// �����Ƴ������������1��5�����飬������ÿһ��Ԫ�ص���Чֵ��0~250
	uint8_t PF_Total[MAX_CELL_CNT];							// �����Ƴ������������1��104�����飬������ÿһ��Ԫ�ص���Чֵ��0~250
}YJ_MESSAGE;

// �洢��flash�Ľṹ�壺�������µ���ж�д
typedef struct {
	uint8_t Cache_flag;																																			// ����Flash����
	uint8_t Cache_Flag_SOC_Highin;																													// ����Flash����
	uint8_t Cache_Flag_SOC_Middlehold;																											// ����Flash����
	uint8_t Cache_Flag_SOC_Lowout;																													// ����Flash����
	uint8_t Cache_Flag_SOC_RangeOK;																												// ����Flash����
	uint8_t Cache_New_charge_flag;																													// ����Flash����
	uint8_t Cache_EKF_TimeCNT;																														// ����Flash����
	uint8_t Cache_ocv_interval_jump_flag;																										// ����Flash����
	uint8_t Cache_OCV_ID_k;																																// ����Flash����
	uint8_t Cache_Panduan_yujing_error_TimeCNT_Disch;																			// ����Flash����
	uint8_t Cache_Panduan_yujing_error_TimeCNT_Char; 																			// ����Flash����
	uint8_t CC_Charge_Flag;																																	// ����Flash����
	uint16_t Cache_ocv_ID_CNT;																															// ����Flash����
	uint16_t Cache_Charge_CNT;																														// ����Flash����
	uint8_t Cache_EarlyCharge_CNT;																													// ����Flash����
	int16_t Cache_mean_OCV_k0;																														// ����Flash����
	uint8_t Cache_Fault_CNT;																																// ����Flash����
	uint8_t FaultCell_CNT_New;																															// ����Flash����
	uint8_t FaultCellOk_CNT_New;																														// ����Flash����
	uint8_t Disch_FaultCellOk_CNT_Last;																											// ����Flash����
	uint8_t Char_FaultCellOk_CNT_Last;																												// ����Flash����
	uint8_t FaultCell_CNT_Last;																															// ����Flash����
	uint8_t FaultOK_CNT;																																		// ����Flash����
	uint8_t Fault_OCV_ID;																																		// ����Flash����
	uint8_t Fault_Yes_Flag;																																	// ����Flash����
	uint8_t Disch_Fault_Yes_Flag;																														// ����Flash����
	uint8_t Char_Fault_Yes_Flag;																															// ����Flash����
	uint32_t dOCV_SOC_CNT;																																// ����Flash����
	float Cache_panduan_yujing_disch[MAX_CELL_CNT];																				// ����Flash��ά����
	float Cache_X_celiang_M[MAX_CELL_CNT + 1][2];																					// ����Flash��ά����
	float Cache_P_celiang_M[MAX_CELL_CNT + 1][4];																					// ����Flash��ά����
	float Cache_R_charge[MAX_CELL_CNT + 1][Char_CNT];																			// ����Flash��ά����
	uint8_t Cache_FaultCell_ID_SameProcess[MAX_Fault_CELL_CNT][4];													// ����Flash��ά����
	uint8_t Cache_Disch_FaultCell_ID_DiffProcess_Last[MAX_Fault_CELL_CNT];										// ����Flash��ά����
	uint8_t Cache_Char_FaultCell_ID_DiffProcess_Last[MAX_Fault_CELL_CNT];											// ����Flash��ά����
	uint8_t Cache_FaultCell_ID_DiffProcess_New[MAX_Fault_CELL_CNT];													// ����Flash��ά����
	float Cache_dOCV_SOC[MAX_CELL_CNT];																									// ����Flash��ά����
	uint8_t Suanli2_ThresholdFlag_Disch[MAX_CELL_CNT];																			// ����Flash��ά����
	uint8_t Suanli2_ThresholdUpdata_Disch[MAX_CELL_CNT][3];																// ����Flash��ά����
	uint8_t Suanli2_ThresholdFlag_Char[MAX_CELL_CNT];																			// ����Flash��ά����
	uint8_t Suanli2_ThresholdUpdata_Char[MAX_CELL_CNT][3];																    // ����Flash��ά����
	uint8_t Cache_dSOC_Cycles[MAX_CELL_CNT][Panduan_yujing_error_TimeCNT];								// ����Flash��ά����
	uint8_t Cache_dQ_Cycles[MAX_CELL_CNT][Panduan_yujing_error_TimeCNT];									// ����Flash��ά����
	float Cache_dSOC_Cal_Time[Panduan_yujing_error_TimeCNT];																// ����Flash��ά����
	float Cache_dQ_Cal_Time[Panduan_yujing_error_TimeCNT];																	// ����Flash��ά����
	uint32_t Cache_dSOC_Cal_Time_Last;																											// ����Flash��ά����
	uint32_t Cache_dQ_Cal_Time_Last;																												// ����Flash��ά����
	uint16_t Cache_dSOC_Cycles_CNT;																												// ����Flash����
	uint16_t Cache_dQ_Cycles_CNT;																													// ����Flash����
}Flash_DATA1;

// �洢��flash�Ľṹ�壺�������µ�ͳ�ŵ�״̬ת����ж�д
typedef struct {
	int16_t Cache_panduan_base[MAX_CELL_CNT][MAX_diffocv_CNT][Panduan_base_CNT];				// ����2Flash��ά����
	int16_t Cache_panduan_yujing_error_Time[MAX_CELL_CNT][Panduan_yujing_error_TimeCNT];		// ����2Flash��ά����
	uint32_t Cache_UTCtimeOF_panduan_base[MAX_diffocv_CNT][UTC_CNT_PanduanBase];				// ����2Flash��ά����
	uint8_t Cache_panduan_base_CNT[MAX_diffocv_CNT];																	// ����2Flash��ά����
}Flash_DATA2;

/*********   �ն˵����û��Զ��庯��    *************/
//extern const char* SWD_VERSION;				//  �㷨�汾��

// ��ʼ��������ֻ���ϵ�ʱ����һ�Σ���������0�����󷵻ط�0  
extern int initialFunction(Flash_DATA1* Cache_CAL_DATA, Flash_DATA2* Cache_Base_DATA , YJ_MESSAGE* Cal_Suanli);						// ������ʼ������

// ����1����
extern void battsafetyFunction1(YJ_MESSAGE* Cal_Suanli1);																																	// ����1����

// ����2����
extern void battsafetyFunction2(Flash_DATA1* Cache_CAL_DATA, Flash_DATA2* Cache_Base_DATA, YJ_MESSAGE* Cal_Suanli2);		// ����2����

// ����3����
extern void battsafetyFunction3(Flash_DATA1* Cache_CAL_DATA, YJ_MESSAGE* Cal_Suanli3);																	// ����3����

