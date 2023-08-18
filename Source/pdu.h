#ifndef __PDU_H__ 
#define __PDU_H__

#include "stdint.h"
#include "versionManager.h"

#define countof(a) (sizeof(a) / sizeof(*(a)))

#define MAX_URL_LEN 50            //������󳤶�

#define MAX_FAULT_NUM 20   				//���ϴ����������
#define MAX_EXTERN_DATALEN 500    //��չ������󳤶�

/* �ն˲��� */
typedef struct{
	uint8_t store_flag;               //�����־
	char terminalId[13];              //�ն˱��
	char scyId[17];										//��ȫоƬID
	uint8_t chipKey[65];							//��ȫоƬ��Կ
	uint8_t setUpKey[65];							//�����ù�Կ
	uint8_t realKey[65];							//ʵʱ�����ù�Կ
	uint8_t setUpFlag;								//�����־ 0xAA���Ѽ��� ��0xAA �ն�δ����
	uint8_t qbActicFlag;							//��꼤���־
	char qbId[21];										//����豸���
	uint16_t gpsCorpcode;							//GPS���̴���
	uint8_t subDevCode;								//�豸����
	uint8_t qbAuthSta;								//������Ȩ״̬
	uint8_t factoryDate[4];						//����������BCD��
}TERMINAL_PARA;

/* Ӧ�ò������� */
typedef struct{
	uint8_t store_flag;               //�����־
	//�汾��Ϣ
	char hardWareVer[6];              //Ӳ���汾
	char firmWareVer[6];              //�̼��汾
	//����������Ϣ
	char vinCode[18];                 //���ܺ�VIN 
	char plateNumber[12];             //���ƺ�	
	//�ն˻�������
	uint8_t can1_used;          			//can1 ����
	uint32_t can1_baudrate;           //can1 ������
	uint8_t can2_used;                //can2 ����
	uint32_t can2_baudrate;           //can2 ������
	uint8_t can3_used;                //can3 ����
	uint32_t can3_baudrate;           //can3 ������
	char apn[32];                     //APN
	uint8_t iccid[21];								//����ICCID
	uint8_t smsNo[16];								//�������ĺ���
	//ƽ̨Ӧ�ò���
	char domain[5][MAX_URL_LEN];  		//��������ƽ̨����
	uint32_t port[5];             		//��������ƽ̨�˿ں�
	uint8_t linkSwitch;								//��·���ƣ�һ��Ϊ��־һ����·
	uint8_t isDebug;           				//���ڴ�ӡ����	
	uint8_t carType;									//�������� 1:����Դ 2:��ȼ�� 3:�춯 4:���� 5:���� 6:����
	//ƽ̨���ڿ��ƣ���ʼƽ̨���ڿ��ƣ���������·�ɵ�������
	uint16_t localSaveInterval;       //���ش洢ʱ������(ms)
	uint16_t realDataInterval;        //ʵʱ��Ϣ�ϱ�����(s)
	uint16_t warnRealDataInterval;    //����ʱ��Ϣ�ϱ�����(ms)
	uint16_t heartInterval;           //��������(s)
	uint16_t terminalRespTimeOut;			//�ն���Ӧ��ʱʱ��
	uint16_t platformRespTimeOut;			//ƽ̨��Ӧ��ʱʱ��
	uint16_t nextLoginInterval;				//��һ�ֵ���ʱ����	
	//��չ����
	uint8_t externPar1;               //��չ����1
	uint8_t externPar2;               //��չ����2
}APP_PARA;


//��������ܳ���Ϣ ͣ����������Ч���������
typedef struct _motorData{
	uint8_t motorIdx;                           //����������
	uint8_t motorState;                         //�������״̬
	int16_t motorCtrTemp;					              //��������������¶�
	uint16_t motorSpeed;					              //�������ת��
	uint8_t motorLoad;													//����������ذٷֱ�	
	float motorTorsion;                         //�������ת��
	int16_t motorTemp;						              //��������¶�
	float motorVol;								              //��������������ѹ
	float motorCur;								              //���������ֱ��ĸ�ߵ���
}MotorData, *pMotorData;

//�ɳ�索����ϵͳ��Ϣ
typedef struct _subSysData{
	char rechargeSysCode[50];  //�ɳ�索��ϵͳ����
	//��ѹ��Ϣ
	uint8_t subSysIdx;                          //�ɳ�索����ϵͳ��
	float subSysVol;                            //�ɳ�索��װ�õ�ѹ
	float subSysCur;                            //�ɳ�索��װ�õ���
	uint16_t singleVolCnt;                      //��ϵͳ����������
	uint16_t singleVolStartIdx;                 //�����ʼ���(�ܵ���б�, ��0��ʼ)
	//�¶���Ϣ
	uint16_t singleTemCnt;                      //��ϵͳ�¶�̽�����
	uint16_t singleTemStartIdx;                 //�¶���ʼ���(���¶��б���0��ʼ)
}SubSysData, *pSubSysData;

#pragma anon_unions
//���״̬
typedef union{
	struct{
		uint8_t catalyst:1;														//�߻�ת����
		uint8_t heatedCatalyst:1;											//���ȴ߻�ת����
		uint8_t evaporativeSys:1;											//����ϵͳ
		uint8_t secondaryAirSys:1;										//���ο���ϵͳ
		uint8_t acSysRefrigerant:1;										//ACϵͳ�����
		uint8_t exhaustGasSensor:1;										//����������
		uint8_t exhaustGasSensorHeater:1;							//����������������
		uint8_t egrAndVvtSys:1;												//EGR��Vvtϵͳ
		uint8_t coldStartAidSys:1;										//����������ϵͳ
		uint8_t boostPressureCtrlSys:1;								//��ѹѹ������ϵͳ
		uint8_t dpf:1;																//DPF���
		uint8_t scrOrNOxAdsorber:1;										//ѡ���Դ߻���ԭϵͳ��SCR����NOx������
		uint8_t nmhcConvertingCatalyst:1;							//NMHC�����߻���
		uint8_t misfire:1;														//ʧ��
		uint8_t fuelSys:1;														//ȼ��ϵͳ
		uint8_t comprehensiveComponent:1;							//�ۺϳɷ�
	};
	uint16_t value;
}DiagMonitorState;

//����Э��ʵʱ����
typedef struct _realData{
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	//OBD����
	uint8_t obdDiagProt;                        //OBD���Э�� 0-IOS15765,1-IOS27145,2-SAEJ1939,0xFE-��Ч
	uint8_t milState;                        		//MIL״̬ 0-δ����,1-����,3δ���,4������,0xFE�쳣,0xFF��Ч
	DiagMonitorState diagSpState;               //���֧��״̬
	DiagMonitorState diagRdyState;							//��Ͼ���״̬
	char vin[18];																//���ܺţ����Ǹ������Э���ȡ��
	char softCalId[19];													//����궨ʶ���
	char cvn[19];																//�궨��֤��
	char iuprVal[37];														//IUPRֵ"һ���ĸ������-2�ֽ�
/*
���ѭ��������-2�ֽ�
NMHC���Ӽ�����-2�ֽ�
NMHC��ĸ������-2�ֽ�
NOx/SCR���Ӽ�����-2�ֽ�
NOx/SCR��ĸ������-2�ֽ�
NOx���������Ӽ�����-2�ֽ�
NOx��������ĸ������-2�ֽ�
PM���Ӽ�����-2�ֽ�
PM��ĸ������-2�ֽ�
�������������Ӽ�����-2�ֽ�
������������ĸ������-2�ֽ�
EGR and/or VVT���Ӽ�����-2�ֽ�
EGR and/or VVT��ĸ������-2�ֽ�
Boost Pressure���Ӽ�����-2�ֽ�
Boost Pressure��ĸ������-2�ֽ�"	
*/
	
	uint8_t faultCodeCnt;												//����������
	uint32_t faultCode[MAX_FAULT_NUM];       		//�ɳ�索��װ�ù��ϴ����б�
	
	//��������Ϣ
//	float speed;									              //����
	float barometric;														//����ѹ��
	int16_t engineTorque;												//���������Ť��/ʵ��Ť��
	int16_t frictionTorque;											//Ħ��Ť��
	float engineSpeed;													//������ת��
	float engineFuelFlow;												//������ȼ������
	float scrUpperNOxSensor;										//SCR����NOx���������ֵ
	float scrLowerNOxSensor;										//SCR����NOx���������ֵ
	float reagentSurplus;												//��Ӧ������
	float intakeFlowrate;												//������
	float scrInletTemp;													//SCR����¶�
	float scrOutletTemp;												//SCR�����¶�
	float dpfPressDiff;													//dpfѹ��
	int16_t engineCoolantTemp;									//��������ȴҺ�¶�
	float tankLevel;														//����Һλ
//	float totalMileage;						            //�ۼ����
	
	uint8_t technology;													//����·��:0:DPF��SCR������1:TWC������NOx��2:TWC��NOx
	//��Ԫ�߻�����������
	float twcLowerNOxSensor;										//TWC����NOx���������ֵ
	float twcUpperOxySensor;										//TWC����Oxy���������ֵ
	float twcLowerOxySensor;										//TWC����Oxy���������ֵ
	float twcTemp;															//TWC�����Ρ����Ρ�ģ�⣩�¶�
	
	//����������
	uint8_t engineTorqueMode;									  //������Ť��ģʽ 0000 �㲥
	float acceleratorVal;				                //����̤���г�ֵ F003 �㲥 ȷ��
	float EngTotalFuelUsed;											//�ۼ��ͺ� FEE9 ���󣨲��ֹ㲥�� ȷ��
	int16_t ureaTankTemp;												//�������¶� FE56 �㲥 ȷ��
	float actAreaInjectVal;											//ʵ������������ �㲥 ��δʵ��������֤
	uint32_t  totalUreaUsed;										//�ۼ���������  �㲥 ��δʵ��������֤
	float dpfExhaustTemp;												//DPF�����¶� ȷ��
	//���ð�������
	float engFuelRate;													//˲ʱ�ͺ� �����Ƿ�����ȼ�;����� FEF2 �㲥 ȷ��
	float engTotalHoursOfOperation;							//������������ʱ��, FEE5 ���󣨲��ֹ㲥�� ȷ��
	uint16_t	engReferenceTorque;								//���������ο�Ť�� FEE3 �㲥 data[19] data[20] ȷ��	
	
	//��������Ϣ
	float speed;									              //����
	float totalMileage;						              //�ۼ����
	//����λ������
	uint8_t locationState;				              //��λ״̬
	double longd;									              //����
	double latd;									              //γ��
	//��������
	uint8_t carState;                           //����״̬ 1����״̬,2Ϩ��,3����״̬,0xFE�쳣,0xFF��Ч
	uint8_t chargeState;                        //���״̬ 1ͣ�����,2��ʻ���,3δ���,4������,0xFE�쳣,0xFF��Ч
	uint8_t operationState;                     //����ģʽ 1����,2�춯,3ȼ��,0xFE�쳣,0xFF��Ч
	float total_volt;       			              //�ܵ�ѹ
	float total_current;    			              //�ܵ���
	uint8_t soc;									              //soc
	uint8_t dc2dcState;                         //DC-DC״̬ 1����,2�Ͽ�,0xFE�쳣,0xFF��Ч
	float brakingVal;						                //�ƶ�̤���г�ֵ
	uint8_t stall;                              //��λ Bit0-Bit3:0-�յ�, 1-1��, 2-2�� ... 6-6��, 0xOD-����, 0x0E-�Զ�D��, 0x0F-ͣ��P��; Bit4:1���ƶ���,0���ƶ���; Bit5:1��������,0��������
	uint16_t mohm;       					              //��Ե����
	//�����������
	uint8_t motorCnt;                           //�����������
	MotorData motorData[2];         						//��������ܳ���Ϣ
	//ȼ�ϵ������
	float fuelBatVol;                           //ȼ�ϵ�ص�ѹ
	float fuelBatCur;                           //ȼ�ϵ�ص���
	float batFuelConsumption;                   //���ȼ��������
	uint16_t fuelBatTemCnt;                     //ȼ�ϵ���¶�̽������
	int16_t fuelBatTem[120];    								//ȼ�ϵ��̽���¶�ֵ
	int16_t maxHydrSysTem;                      //��ϵͳ������¶�
	float maxHydrSysTemIdx;                   	//��ϵͳ������¶�̽�����
	uint16_t maxHydrThickness;                  //�������Ũ��
	uint8_t maxHydrThicknessIdx;                //�������Ũ�ȴ���������
	float maxHydrPressure;                      //�������ѹ��
	uint8_t maxHydrPressureIdx;                 //�������ѹ������������
	uint8_t dc2dcState_highVol;                 //��ѹDC/DC״̬
	//����������
	uint8_t engineState;                        //������״̬
	uint16_t crankshaftSpeed;                   //����ת��
	float fuelConsumption;                      //ȼ��������
	//��ֵ����
	uint8_t maxVolPack_index; 		              //��ߵ�ѹ�����ϵͳ��
	uint16_t maxVol_index; 				              //��ߵ�ѹ��ص������
	float max_singleVol;					              //��ص����ѹ���ֵ
	uint8_t minVolPack_index;    	              //��͵�ѹ�����ϵͳ�� 
	uint16_t minVol_index; 				              //��͵�ѹ��ص������
	float min_singleVol;     			              //��ص����ѹ���ֵ
	uint8_t maxTemperPack_index; 	              //����¶���ϵͳ��
	uint16_t maxTemper_index; 			            //����¶�̽�����
	int16_t max_singleTemper;			              //����¶�ֵ
	uint8_t minTemperPack_index;                //����¶���ϵͳ��
	uint16_t minTemper_index; 			            //����¶�̽�����
	int16_t min_singleTemper;                   //��͵��¶�
	//��������
	uint8_t alarmLevel;                         //�����ȼ�
	uint8_t tempDiffAlert;				              //�¶Ȳ��챨��
	uint8_t batHighTempAlert;		                //��ظ��±���
	uint8_t batHighVolAlert;	                  //���ش���װ�����͹�ѹ����
	uint8_t batLowVolAlert;	                    //���ش���װ������Ƿѹ����
	uint8_t socLowAlert;					              //SOC�ͱ���
	uint8_t singleBatHighVolAlert;			        //�����ع�ѹ����
	uint8_t singleBattLowVolAlert;			        //������Ƿѹ����
	uint8_t socHighAlert;					              //SOC���߱���
	
	uint8_t socHopAlert;					              //SOC���䱨��
	uint8_t batNotMatchAlert;                   //�ɳ�索��ϵͳ��ƥ�䱨��
	uint8_t singleBatPoorConsisAlert;           //��ص���һ���Բ��
	uint8_t insulationFailtAlert;	              //��Ե����
	uint8_t dc2dcTemAlert;                      //DC-DC�¶ȱ���
	uint8_t brakingAlert;                       //�ƶ�ϵͳ����
	uint8_t dc2dcStateAlert;                    //DC-DC״̬����
	uint8_t motorCtrTemAlert;                   //��������������¶ȱ���
	
	uint8_t highPressInterlockStateAlert;       //��ѹ����״̬����
	uint8_t motorTempAlert;                     //��������¶ȱ���
	uint8_t batOverCharge;                      //���ش���װ�����͹���
	
	uint8_t batFaultCnt;                        //�ɳ�索��װ�ù�������N1  ������MAX_BAT_FAULT_NUM
	uint32_t batFault[MAX_FAULT_NUM];       								//�ɳ�索��װ�ù��ϴ����б�
	uint8_t motorFaultCnt;                      //���������������N2  ������MAX_MOTOR_FAULT_NUM
	uint32_t motorFault[MAX_FAULT_NUM];   									//����������ϴ����б�
	uint8_t engineFaultCnt;                     //��������������N3  ������MAX_ENGINE_FAULT_NUM
	uint32_t engineFault[MAX_FAULT_NUM]; 									//���������ϴ����б�
	uint8_t otherFaultCnt;                      //������������N4  ������MAX_OTHER_FAULT_NUM
	uint32_t otherFault[MAX_FAULT_NUM];   									//�������ϴ����б�
	
	//�ɳ�索��װ�õ�ѹ����
	uint8_t subBatSysCnt;                      //�ɳ�索����ϵͳ���� ������MAX_BAT_SUBSYS_NUM
	SubSysData subSysData[2];                  //�ɳ�索����ϵͳ��ѹ/�¶���Ϣ��
	//�ɳ�索��ϵͳ����
	uint8_t rechargeSysCodeLen;                //�ɳ�索��ϵͳ���볤��
	float single_vol[600];				             //�����ѹ�ܱ�
	int16_t single_temper[160];		             //̽���¶�ֵ�ܱ�
	//�Զ�������
	uint16_t externDataLen;                     //��չ��Ч���ݳ���
	uint8_t externData[MAX_EXTERN_DATALEN];     //�û��Զ�������
}RealData,*pRealData;

typedef struct _terminalState{
	float batVolt;																//��ص�ѹ
	float pwrVolt;																//��Դ��ѹ
	uint8_t powerState:1;													//�ⲿ��Դ״̬
	uint8_t obdState:1;														//OBD״̬
	uint8_t gbDebug;															//���ƴ�ӡ���걨��
	uint8_t gpsState:3;					//GPS״̬ bit0:ģ��״̬ bit1:���߶�· bit2:���Ŷ�·

	uint32_t speed;																//��������ID
	uint32_t barometric;													//��������ѹ��ID
	uint32_t engineTorque;												//����������ת��ID
	uint32_t engineSpeed;													//����������ת��ID
	uint32_t frictionTorque;											//����Ħ��Ť��ID
	uint32_t engineFuelFlow;											//����ȼ������ID
	uint32_t reagentSurplus;											//������Ӧ������ID
	uint32_t engineCoolantTemp;										//������������ȴҺ�¶�ID
	uint32_t tankLevel;														//��������ҺλID
	uint32_t totalMileage;												//�����ۼ����ID
	uint32_t intakeFlowrate;											//����������ID
	uint8_t getCarData;														//������ȡOBD״̬
}TerminalState,*pTerminalState;

extern TERMINAL_PARA gFrimPara;         				/* �ն˹̼����� */
extern APP_PARA gSysPara;               				/* Ӧ�ò��� */
extern RealData gRealData;      	      				/* ʵʱ���� */
extern TerminalState gTerminalState;						/* �ն�״̬����*/
extern uint8_t szMainBuf[2048];

#endif
