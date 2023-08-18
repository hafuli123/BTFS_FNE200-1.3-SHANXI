#include "pdu.h"
#include "NetworkLayer.h"
#include "diagnostic.h"
#include "J1939TP.h"
#include "stm32f4xx_eeprom.h"
#include "bsp_sys.h"
#define TRUE 1
#define FALSE 0
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define SERVICE_NUMBER 				26
typedef void (*ServiceHandler)(TP_T* pTp,uint8_t N_TAType,uint16_t length, uint8_t *MessageData);

typedef enum{
	//华晨鑫源基本服务
	SESSION_CONTROL = 0x10,									//诊断会话控制
	RESET_ECU = 0x11,												//电控单元复位
	CLEAR_DTC_INFO = 0x14,									//清除诊断信息
	READ_DTC_INFO = 0X19,										//读取DTC信息
	READ_DATA_BY_ID = 0x22,									//读取数据
	READ_MEMORY_BY_ADDRESS = 0x23,					//读取内存
	SECURITY_ACCESS = 0x27,									//安全访问
	COMMUNICATION_CONTROL = 0x28,						//通信控制
	READ_DATA_PERIOD_ID = 0x2A,							//读取数据（周期标识符）
	DYNAMICALLY_DEFINE_DATA_ID = 0x2C,			//动态定义数据标识符
	WRITE_DATA_BY_ID = 0x2E,								//写入数据
	IO_CONTROL_BY_ID = 0x2F,								//输入输出控制
	ROUTINE_CONTROL = 0x31,									//例程控制	
	TESTER_PRESENT = 0x3E,									//诊断设备在线
	WRITE_MEMORY_BY_ADDRESS = 0x3D,					//写入内存
	CONTROL_DTC_SETTING = 0x85,							//控制DTC设置
	//华晨鑫源传输数据
	REQUEST_DOWNLOAD = 0x34,								//请求下载
	TRANSMIT_DATAD = 0x36,									//数据传输
	REQUEST_TRANSFER_EXIT = 0x37,						//请求退出传输
	//扩展
	GET_TIME_PARAM = 0x83,									//访问时间参数
	SECURITY_DATA_TRANSMISSION = 0x84,			//安全数据传输
	RESPONSE_ON_EVENT = 0x86,								//事件响应
	LINK_CONTROL = 0x87,										//链路控制
	READ_SCALING_DATA_BY_ID = 0x24,					//通过ID读比例数据
	REQUEST_UPLOAD = 0x35,									//请求上传
}ServiceName;

typedef enum{
	PR = 0x00,															//positive response
	GR = 0x10,															//general reject
	SNS = 0x11,															//service not supported
	SFNS = 0x12,														//sub-function not supported
	IMLOIF = 0x13,													//incorrect message length or invalid format
	RTL = 0x14,															//response too long
	BRR = 0x21,															//busy repeat request
	CNC = 0x22,															//condifitons not correct
	RSE = 0x24,															//request sequence error
	NRFSC = 0x25,
	FPEORA = 0x26,
	ROOR = 0x31,														//reqeust out of range
	SAD = 0x33,															//security access denied
	IK = 0x35,															//invalid key
	ENOA = 0x36,														//exceed number of attempts
	RTDNE = 0x37,														//required time delay not expired
	UDNA = 0x70,														//upload download not accepted
	TDS = 0x71,															//transfer data suspended
	GPF = 0x72,															//general programming failure
	WBSC = 0x73,														//wrong block sequence coutner
	RCRRP = 0x78,														//request correctly received-respone pending
	SFNSIAS = 0x7e,													//sub-function not supported in active session
	SNSIAS  = 0x7F,													//service not supported in active session
	VTH = 0x92,															//voltage too high
	VTL = 0x93,															//voltage too low
}NegativeResposeCode;

typedef enum{
	ECU_DEFAULT_SESSION = 1,								//默认会话
	ECU_PAOGRAM_SESSION = 2,								//编程会话
	ECU_EXTENED_SESSION = 3,								//扩展会话
	ECU_FACTORY_SESSION = 0x71,							//供应商session，用于下线配置
}SessionType;

typedef enum{
	WAIT_SEED_REQ,													//等待SEED
	WAIT_KEY,																//等待KEY
	WAIT_DELAY,															//等待
	UNLOCKED,																//解锁
}SecurityUnlockStep;

typedef enum{
	REPORT_DTCNUMBER_BY_MASK = 1,						//通过状态掩码去查找与其相匹配的故障个数
	REPORT_DTCCODE_BY_MASK = 2,							//按照定义的状态掩码的形式去查找匹配的故障
	REPORT_DTCSNAPSHOT_BY_ID = 3,						//报告DBC快照标识
	REPORT_DTCSNAPSHOT_BY_DTCNUMBER = 4,		//请求指定故障码（DTC）的快照信息
	REPORT_DTCSNAPSHOT_BY_RECORDNUMBER = 5,	//按记录编号报告DBC存储数据
	REPORT_DTCEXTEND_DATA_BY_DTCNUMBER = 6,	//请求指定故障码（DTC）的扩展信息
	REPORT_DTCNUMBER_BY_SEVERITYMASK,				//按照严重性掩码报告DTC编号
	REPORT_DTC_BY_SEVERITYMASK,							//按照严重性掩码报告DTC编号
	REPORT_SEVERITYID_OF_DTC,								//报告DTC的严重信息
	REPORT_SUPPORTED_DTC = 0x0A,						//请求所有支持的DTC信息
	REPORT_FIRST_FAILED_DTC,								//报告首个测试失败DTC
	REPORT_FIRST_CONFIRMED_DTC,							//报告首个确认DTC
	REPORT_MOST_FAILED_DTC,									//报告最新测试失败DTC
	REPORT_MOST_CONFIRMED_DTC,							//报告最新确认的DTC
	REPORT_MIRRORDTC_BY_MASK,								//按照状态掩码报告镜像内存DTC
	REPORT_MIRRORDTC_EXTENDED_BY_DTC_NUMBER,//按照DTC编号搞过镜像内存DTC扩展数据记录
	REPORT_MIRRORDTC_NUMBER_BY_MASK,				//按照状态掩码报告镜像内存DTC数量
	REPORT_OBDDTC_NUMBER_BY_MASK,						//按照状态掩码报告排放OBDDTC数据
	REPORT_OBDDTC_BY_MASK,									//按报告掩码报告排放OBD DTC
}DTCSubFunction;


typedef enum{
	POWERTRAIN,															//动力
	CHASSIS,																//底盘
	BODY,																		//车身
	NETWORK,																//网络
}DTCAlphanumeric ;


typedef union{
	struct{
		uint8_t TestFailed:1;													//测试失败
		uint8_t TestFailedThisMonitoringCycle:1;			//本检测周期测试失败
		uint8_t PendingDTC:1;													//待定DTC
		uint8_t ConfirmedDTC:1;												//确认DTC
		uint8_t TestNotCompleteSinceLastClear:1;			//上次清零后测试未完成
		uint8_t TestFailedSinceLastClear:1;						//上次清零后测试失败
		uint8_t TestNotCompleteThisMonitoringCycle:1;	//本检测周期测试未完成
		uint8_t WarningIndicatorRequested:1;					//警告指示位请求
	}DTCbit;
	uint8_t DTCStatusByte;
}DTCStatusType;

typedef struct _DtcNode{
	uint32_t DTCCode;																//DTC代码
	DetectFun DetectFunction;												//故障检测函数
	uint16_t EEpromAddr;														//存储地址
	uint8_t TripLimitTimes;													//DTC确认次数值
	DTCStatusType DTCStatus;												//DTC状态
	uint8_t OldCounter;															//已老去计数器，转老化完成
	uint8_t GoneCounter;														//老化计数器，累计老化完成计数
	uint8_t TripCounter;														//故障待定计数器，转确认
	uint8_t FaultOccurrences;												//故障出现计数器，累计
	uint16_t SnapShotEEpromAddr;										//快照信息地址
}DTCNode;

typedef struct _DidNode{
	uint16_t ID;																			//ID
	uint8_t dataLength;																//数据长度
	uint8_t* dataPointer;															//数据指针
	IoControl Callback;																//IO输入输出回调
	DIDType didType;																	//DID类型
	ReadWriteAttr RWAttr;															//读写属性
	uint16_t EEpromAddr;															//存储地址
	bool SupportWriteInFactoryMode;										//支持出厂模式写入
}DIDNode;

typedef struct _GroupNode{
	uint32_t GroupID;																	//组ID
}DTCGroupNode;

typedef struct _Snapshot{
	uint8_t snapshotRecord;														//快照记录
	uint16_t snapshotID;															//快照ID
	uint8_t* dataPointer;															//数据指针
	uint8_t dataLength;																//数据长度
}Snapshot;

typedef struct{
	bool support;
	ServiceName serviceName;
	uint8_t PHYDefaultSession_Security:4;							//security suppport in default session physical address
	uint8_t PHYProgramSeesion_Security:4;							//security suppport in program session physical address
	uint8_t PHYExtendedSession_Security:4;						//security suppport in extened session physical address
	uint8_t FUNDefaultSession_Security:4;							//security suppport in default session function address
	uint8_t FUNProgramSeesion_Security:4;							//security suppport in program session function address
	uint8_t FUNExtendedSession_Security:4;						//security suppport in extened session function address
	ServiceHandler serviceHandle;
}SessionService;

typedef struct{
	bool valid;
	SecurityLevel level;										//安全等级
	SecurityFun UnlockFunction;							//解锁函数，计算KEY
	uint8_t seedID;													//seedID，奇数位
	uint8_t keyID;													//keyId，偶数位
	uint8_t FaultCounter;										//解锁错误计数
	uint16_t FaultCounterAddr;							//故障次数地址
	uint8_t FaultLimitCounter;							//解锁失败最大次数
	uint32_t UnlockFailedDelayTime;					//解锁失败延迟时间
	uint8_t subFunctionSupported;						//子功能支持
	DiagTimer SecurityLockTimer;						//解锁失败3次后再次解锁延时定时器
	uint8_t KeySize;												//KEY字节数
}SecurityUnlock;

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

void ServiceNegReponse(TP_T* pTp,uint8_t serviceName,uint8_t RejectCode);														//否定响应
void Diagnostic_ReadDTCPositiveResponse(uint8_t DTCSubFunction,uint8_t DTCStatausMask);		//读取故障码肯定响应
void Service10Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//诊断会话处理
void Service11Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//电控单元复位处理
void Service27Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//安全访问处理
void Service28Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//通信控制处理
void Service3EHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//诊断设备在线处理
void Service83Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//
void Service84Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//
void Service85Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//控制DTC设置处理
void Service86Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//
void Service87Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//
void Service22Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//读取数据处理
void Service23Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//读取内存处理
void Service24Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//
void Service2AHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//数据数据（周期标识符）处理
void Service2CHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//动态定义数据标识符处理
void Service2EHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//写入数据处理
void Service3DHandle(TP_T* pTp,uint8_t N_TAType,uint16_t length, uint8_t *MessageData);							//写入内存处理
void Service14Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//清除诊断信息处理
void Service19Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//读取DTC信息处理
void Service2FHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//输入输出控制处理
void Service31Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//例程控制处理
void Service34Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//请求下载处理
void Service35Handle(TP_T* pTp,uint8_t N_TAType,uint16_t length, uint8_t *MessageData);							//请求上传处理
void Service36Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//数据传输处理
void Service37Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData);						//请求退出传输处理
DIDNode* SearchDidNode(uint16_t DID);																											//查找DID
void Diagnostic_ClearDTC(uint32_t Group);																									//清除DTC
void Diagnostic_SaveAllDTC(void);																													//保存DTC
void Diagnostic_LoadAllDTC(void);																													//加载DTC
DTCNode* GetDTCNodeByCode(uint32_t dtcCode);

/* Private variables ---------------------------------------------------------*/

/*========================diagnositc version================================*/
const char DiagnosticDriverVersion[] = {1,0,1};																				//诊断服务版本
const char NMVersion[] = {0,0,0};																											//标准网络版本
const char DiagnosticVersion[] = {1,0,1};																							//诊断仪版本
const char DatabaseVersion[] = {0,0};																									//数据库版本
/*========================about security acces================================*/


/*========================about security acces================================*/
static uint8_t Seed[5];																																		//保存解锁种子
static uint8_t key[4];																																		//保存解锁密匙
static SecurityLevel m_SecurityLevel = LEVEL_ZERO;																				//当前解锁安全等级
static SecurityUnlockStep m_UnlockStep = WAIT_SEED_REQ;																		//当前解锁步骤
static SecurityUnlock UnlockList[3];																															//锁列表
/*========================about security acces================================*/

/*========================sesion , id , buf and so on================================*/
static NegativeResposeCode m_NRC;																													//否定响应码
static uint8_t m_CurrSessionType;																													//当前回话类型
static bool ResponsePending;																															//是否发送过78负响应
static bool suppressResponse;																															//是否是抑制响应
static uint16_t ResponseLength;																														//响应长度
static uint8_t CurrentService;																														//当前服务
static uint8_t DiagnosticBuffTX[200];																											//发送数据的缓存
static DiagTimer S3serverTimer;							 																							//诊断会话计时器,从非默认会话自动跳转到默认会话计时，默认5000ms。
static uint16_t P2CanServerMax = 0x32;																										//会话参数，响应超时时间
static uint16_t P2ECanServerMax = 0x1F4;																									//会话参数，否定消息响应超时时间
static uint32_t TesterPhyID;																															//物理ID
static uint32_t TesterFunID;																															//功能ID																																		//ECU ID1
static uint32_t EcuID;																																		//ECU ID
static uint8_t N_Ta;																																			//目标地址
static uint8_t N_Sa;																																			//源地址

static uint8_t SessionSupport;																														//会话支持位
#define Service10Sub01Supported() ((SessionSupport & 0x01) != 0)													//bit0: default session 01 support
#define Service10Sub02Supported() ((SessionSupport & 0x02) != 0)													//bit1: program session 02 support
#define Service10Sub03Supported() ((SessionSupport & 0x04) != 0)													//bit2: extended session 03 support
#define Service10Sub01To02OK() ((SessionSupport & 0x08) != 0)															//bit3: sub02 supported in defaultsession
#define Service10Sub02To03OK() ((SessionSupport & 0x10) != 0)															//bit4: sub03 supported in program
#define Service10SupressSupproted() ((SessionSupport & 0x20) != 0)												//bit5: supressPosRespnse support
/*========================sesion , id , buf and so on================================*/

/*========================about program================================*/
static uint8_t m_BlockIndex = 0;
static uint32_t ProgramAddress = 0;
static uint32_t ProgramLength = 0;
static uint32_t ProgramLengthComplete = 0;
static bool IsUpdating = FALSE;
static bool WaitConfirmBeforeJump = FALSE;				//
static bool WaitConfirmBeforeErase = FALSE;				//擦除前等待78负反馈的确认信息
/*========================about program================================*/

/*========================about DTC and DIDs================================*/	
static DTCNode DTCS[MAX_DTC_NUMBER];																			//DTC链表
static uint8_t DTCAdded;																									//DTC加入位置
static DIDNode DIDS[MAX_DID_NUMBER];																			//DID链表
static uint8_t DIDAdded;																									//DID加入位置
static Snapshot SnapShots[MAX_SNAPSHOT_NUMBER];														//快照链表
static uint8_t SnapShotAdded = 0;																					//快照加入位置
static uint8_t AgedCounterRecord = 0;																			//过期计数器计数记录
static uint8_t AgingCounterRecord = 0;																		//老化计数器计数记录
static uint8_t OccurenceCounterRecord = 0;																//发生计数器计数记录
static uint8_t PendingCounterRecord = 0;																	//待决计数器计数记录

static DTCGroupNode DTCGROUPS[MAX_GROUP_NUMBER];													//DTC组链表
static uint8_t DTCGroupAdded;																							//DTC组加入位置
#if USE_J1939_DTC
static bool DiagDM1Enable = TRUE;
#endif
static uint8_t DtcAvailibaleMask;																					//DTC可用掩码
static uint32_t DtcTimer;																								//DTC定时器
static uint16_t ModuleEEpromStartAddr;																		//模块E2PROW地址
static uint16_t EEpromSizeForUse;																					//E2PROW使用大小
static uint16_t EEpromUsed;																								//EEPROW使用
#define DTC_BYTE_NUMBER_TO_SAVE	5																					//DTC保存占用字节
bool EnableDTCDetect;																											//使能故障码检测													
bool HighVoltage;																													//过电压标志
bool LowVoltage;																													//欠电压标志
/*========================about DTC and DIDs================================*/

/*========================about reset================================*/
static uint8_t ResetTypeSupport;
//bit0: 01 sub function supported
//bit1: 02 sub function supported
//bit2:03 sub function supported
//bit3:04 sub function supported
//bit4:05 sub function supported
//bit5:posresonpse supress supported
#define Service11SupressSupported()  ((ResetTypeSupport & 0x20) != 0)
#define Service11Sub01Supported()  ((ResetTypeSupport & 0x01) != 0)
#define Service11Sub02Supported()  ((ResetTypeSupport & 0x02) != 0)
#define Service11Sub03Supported()  ((ResetTypeSupport & 0x04) != 0)
#define Service11Sub04Supported()  ((ResetTypeSupport & 0x08) != 0)
#define Service11Sub05Supported()  ((ResetTypeSupport & 0x10) != 0)
static ResetCallBack ResetCallBackFun;
static ConditionCallBack conditionCallBackFun;
static EcuResetType m_EcuResetType;																				//ECU复位类型
static bool WaitConfimBeforeReset = FALSE;																//等待确认复位
/*========================about reset===============================*/

/*========================about tester present===============================*/
static uint8_t TesterPresentSuppport;//posresonpse supress supported
#define Service3ESupressSupported()   ((TesterPresentSuppport & 0x01) != 0)
#define Service85SupressSupported()   ((TesterPresentSuppport & 0x02) != 0)
/*========================about tester present===============================*/

/*========================about commulication control===============================*/	

static uint8_t CommTypeSupport;//bit0:00 sub function supported
							//bit1:01 sub function supported
							//bit2:02 sub function supported
							//bit3:03 sub function supported
							//bit4:01 type supported
							//bit5:02 type supported
							//bit6:03 type supported
							//bit6:posresponse supress supported
#define Service28Sub00Suppoted() ((CommTypeSupport & 0x01) != 0)
#define Service28Sub01Suppoted() ((CommTypeSupport & 0x02) != 0)
#define Service28Sub02Suppoted() ((CommTypeSupport & 0x04) != 0)
#define Service28Sub03Suppoted() ((CommTypeSupport & 0x08) != 0)
#define Service28Type01Suppoted() ((CommTypeSupport & 0x10) != 0)
#define Service28Type02Suppoted() ((CommTypeSupport & 0x20) != 0)
#define Service28Type03Suppoted() ((CommTypeSupport & 0x40) != 0)
#define Service28SupressSupported() ((CommTypeSupport & 0x80) != 0)
static CommCallBack commCallBack;
/*========================about commulication control===============================*/	

/*========================about factory mode use ===============================*/

/*========================about factory mode use===============================*/

SessionService ServiceList[SERVICE_NUMBER] = {
	{FALSE, SESSION_CONTROL,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			Service10Handle},//0X10
	{FALSE, RESET_ECU,					LEVEL_UNSUPPORT,	LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_ZERO,			LEVEL_ZERO,			Service11Handle},//0X11
	{FALSE, SECURITY_ACCESS,			LEVEL_UNSUPPORT,	LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service27Handle},//0X27
	{FALSE, COMMUNICATION_CONTROL,		LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_ZERO,			Service28Handle},//0X28
	{FALSE, TESTER_PRESENT,				LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_ZERO,			Service3EHandle},//0X3E
	{FALSE, GET_TIME_PARAM,				LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service83Handle},//0X83
	{FALSE, SECURITY_DATA_TRANSMISSION,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service84Handle},//0X84
	{FALSE, CONTROL_DTC_SETTING,		LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_ZERO,			Service85Handle},//0X85
	{FALSE, RESPONSE_ON_EVENT,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service86Handle},//0X86
	{FALSE, LINK_CONTROL,				LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service87Handle},//0X87
	{FALSE, READ_DATA_BY_ID,			LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service22Handle},//0X22
	{FALSE, READ_MEMORY_BY_ADDRESS,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service23Handle},//0X23
	{FALSE, READ_SCALING_DATA_BY_ID,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service24Handle},//0X24
	{FALSE, READ_DATA_PERIOD_ID,		LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service2AHandle},//0X2A
	{FALSE, DYNAMICALLY_DEFINE_DATA_ID,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service2CHandle},//0X2C
	{FALSE, WRITE_MEMORY_BY_ADDRESS,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service3DHandle},//0X3D
	{FALSE, WRITE_DATA_BY_ID,			LEVEL_UNSUPPORT,	LEVEL_ONE,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service2EHandle},//0X2E
	{FALSE, CLEAR_DTC_INFO,				LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_ZERO,			LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_ZERO,			Service14Handle},//0X14
	{FALSE, READ_DTC_INFO,				LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_ZERO,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service19Handle},//0X19
	{FALSE, IO_CONTROL_BY_ID,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_ONE,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service2FHandle},//0X2F
	{FALSE, ROUTINE_CONTROL,			LEVEL_UNSUPPORT,	LEVEL_ONE,			LEVEL_ONE,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service31Handle},//0X31
	{FALSE, REQUEST_DOWNLOAD,			LEVEL_UNSUPPORT,	LEVEL_ONE,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service34Handle},//0X34
	{FALSE, REQUEST_UPLOAD,				LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service35Handle},//0X35
	{FALSE, TRANSMIT_DATAD,				LEVEL_UNSUPPORT,	LEVEL_ONE,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service36Handle},//0X36
	{FALSE, REQUEST_TRANSFER_EXIT,		LEVEL_UNSUPPORT,	LEVEL_ONE,			LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	LEVEL_UNSUPPORT,	Service37Handle},//0X37
};

/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/*========interface for application layer setting diagnostic parameters==============*/
/*
*/
bool InitAddSecurityAlgorithm(SecurityLevel level, SecurityFun AlgoritthmFun,byte SeedSubFunctionNum,byte KeySubFunctionNum , uint8_t* FaultCounter,uint8_t FaultLimitCounter , uint32_t UnlockFailedDelayTimeMS, SubFunSuppInSession SubFuntioncSupportedInSession,uint8_t KeySize)
{
	uint8_t i;
	for(i = 0 ; i < 3 ; i++)
	{
		if(UnlockList[i].valid == FALSE)
		{
			UnlockList[i].UnlockFunction = AlgoritthmFun;
			UnlockList[i].seedID = SeedSubFunctionNum;
			UnlockList[i].keyID = KeySubFunctionNum;
			UnlockList[i].level = level;
			UnlockList[i].valid = TRUE;
			UnlockList[i].FaultCounterAddr = (ModuleEEpromStartAddr + EEpromUsed);
			EEpromUsed += 1;
			UnlockList[i].FaultLimitCounter = FaultLimitCounter;
			UnlockList[i].UnlockFailedDelayTime = UnlockFailedDelayTimeMS;
			UnlockList[i].subFunctionSupported = SubFuntioncSupportedInSession;
			UnlockList[i].KeySize = KeySize;
			return TRUE;
		}
	}
	return FALSE;
}

bool InitSetSessionSupportAndSecurityAccess(bool support ,uint8_t service,uint8_t PHYDefaultSession_Security,	uint8_t PHYProgramSeesion_Security,	uint8_t PHYExtendedSession_Security,	uint8_t FUNDefaultSession_Security,	uint8_t FUNProgramSeesion_Security,	uint8_t FUNExtendedSession_Security)
{
	uint8_t i;
	if(support == FALSE)
	{
		//warning ,we suggest set service supported when init,or we can not access this service
	}
	for(i = 0 ; i < SERVICE_NUMBER ; i++)
	{
		if(ServiceList[i].serviceName ==  service)
		{
			ServiceList[i].FUNDefaultSession_Security = FUNDefaultSession_Security;
			ServiceList[i].FUNExtendedSession_Security = FUNExtendedSession_Security;
			ServiceList[i].FUNProgramSeesion_Security = FUNProgramSeesion_Security;
			ServiceList[i].PHYDefaultSession_Security = PHYDefaultSession_Security;
			ServiceList[i].PHYExtendedSession_Security = PHYExtendedSession_Security;
			ServiceList[i].PHYProgramSeesion_Security = PHYProgramSeesion_Security;
			ServiceList[i].support = support;
			return TRUE;	
		}
	}
	return FALSE;
}

void InitAddDID(uint16_t DID , uint8_t DataLength , uint8_t* DataPointer , DIDType DidType , IoControl ControlFun , ReadWriteAttr RWAttr,uint16_t EEaddr, bool SupportWriteInFactoryMode)
{
	if(DIDAdded < MAX_DID_NUMBER)
	{
		DIDS[DIDAdded].ID = DID;
		DIDS[DIDAdded].dataLength = DataLength;
		DIDS[DIDAdded].Callback = ControlFun;
		DIDS[DIDAdded].didType = DidType;
		DIDS[DIDAdded].RWAttr = RWAttr;
		DIDS[DIDAdded].SupportWriteInFactoryMode = SupportWriteInFactoryMode;
		if(DidType == EEPROM_DID)
		{
			if(EEaddr == 0)
			{
				DIDS[DIDAdded].EEpromAddr = ModuleEEpromStartAddr + EEpromUsed;
				EEpromUsed += DataLength;
			}
			else
			{
				DIDS[DIDAdded].EEpromAddr = EEaddr;
			}
		}
		else
		{
			DIDS[DIDAdded].dataPointer = DataPointer;
		}

		DIDAdded++;
	}
}

bool InitAddDTC(uint32_t DTCCode,DetectFun MonitorFun,byte DectecPeroid, byte ValidTimes,DTCLevel dtcLevel)
{
	if(DTCAdded < MAX_DTC_NUMBER)
	{
		DTCS[DTCAdded].DTCCode = DTCCode;
		DTCS[DTCAdded].DetectFunction = MonitorFun;
		DTCS[DTCAdded].TripLimitTimes = ValidTimes;
		DTCS[DTCAdded].EEpromAddr = ModuleEEpromStartAddr + EEpromUsed;
		EEpromUsed += DTC_BYTE_NUMBER_TO_SAVE;
		DTCAdded++;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void InitAddDTCSnapShot(uint8_t recordNumber , uint16_t ID , uint8_t* datap , uint8_t size)
{
	if(SnapShotAdded < MAX_SNAPSHOT_NUMBER)
	{
		SnapShots[SnapShotAdded].snapshotRecord = recordNumber;
		SnapShots[SnapShotAdded].snapshotID = ID;
		SnapShots[SnapShotAdded].dataPointer = datap;
		SnapShots[SnapShotAdded].dataLength = size;
		SnapShotAdded++;
	}
}

void InitSetAgingCounterRecordNumber(uint8_t RecordNumer)
{
	AgingCounterRecord = RecordNumer;
}

void InitSetAgedCounterRecordNumber(uint8_t RecordNumer)
{
	AgedCounterRecord = RecordNumer;
}

void InitSetOccurrenceCounterRecordNumber(uint8_t RecordNumer)
{
	OccurenceCounterRecord = RecordNumer;
}

void InitSetPendingCounterRecordNumber(uint8_t RecordNumer)
{
	PendingCounterRecord = RecordNumer;
}

void InitSetDTCAvailiableMask(uint8_t AvailiableMask)
{
	DtcAvailibaleMask = AvailiableMask;
}

void InitAddDTCGroup(uint32_t Group)
{
	if(DTCGroupAdded < MAX_GROUP_NUMBER)
	{
		DTCGROUPS[DTCGroupAdded].GroupID = Group;
		DTCGroupAdded++;
	}
}

void InitSetSysResetParam(bool support01 , bool support02 , bool support03 , bool support04 , bool support05 , ResetCallBack callback,bool supressPosResponse,ConditionCallBack conditFun)
{
	ResetTypeSupport = support01 | (support02<<1) | (support03 << 2) | (support04 << 3) | (support05 << 4) | (supressPosResponse << 5);
	ResetCallBackFun = callback;
	conditionCallBackFun = conditFun;
}

void InitSetCommControlParam(bool supportSubFun00, bool supportSubFun01 , bool supportSubFun02 , bool supportSubFun03 , bool supportType01, bool supportType02, bool supportType03, CommCallBack callback, bool supressPosResponse)
{
	CommTypeSupport = supportSubFun00 | (supportSubFun01 << 1) | (supportSubFun02 << 2)  | (supportSubFun03 << 3) | (supportType01 << 4) | (supportType02 << 5) | (supportType03 << 6) | (supressPosResponse << 7);
	commCallBack = callback;
}

void InitSetSessionControlParam(bool supportSub01, bool supportSub02,bool supportSub03, bool sub02SupportedInDefaultSession, bool sub03SupportedInProgramSession, bool supressPosResponse)
{
	SessionSupport = supportSub01 | (supportSub02 << 1 ) | (supportSub03 << 2) | (sub02SupportedInDefaultSession << 3) | (sub03SupportedInProgramSession << 4) | (supressPosResponse << 5);
}

void InitSetTesterPresentSupress(bool supressPosResponse)
{
	TesterPresentSuppport = (supressPosResponse & 0x01);
}

void InitSetDTCControlSupress(bool supressPosResponse)
{
	TesterPresentSuppport |= (supressPosResponse << 1);
}

void InitSetCanDriverVersionDID(uint16_t m_DID)
{
	InitAddDID(m_DID,3 , (uint8_t*)DiagnosticDriverVersion , REALTIME_DID , NULL , READONLY , 0 , FALSE);
}

void InitSetCanNMVersionDID(uint16_t m_DID)
{
	InitAddDID(m_DID,3 , (uint8_t*)NMVersion , REALTIME_DID , NULL , READONLY , 0 , FALSE);
}

void InitSetCanDiagnosticVersionDID(uint16_t m_DID)
{
	InitAddDID(m_DID,3 , (uint8_t*)DiagnosticVersion , REALTIME_DID , NULL , READONLY , 0 , FALSE);
}

void InitSetCanDataBaseVersionDID(uint16_t m_DID)
{
	InitAddDID(m_DID,2 , (uint8_t*)DatabaseVersion , REALTIME_DID , NULL , READONLY , 0 , FALSE);
}

void InitSetCurrentSessionDID(uint16_t m_DID)
{
	InitAddDID(m_DID,1 , &m_CurrSessionType , REALTIME_DID , NULL , READONLY , 0 , FALSE);
}

/************set netwrok layer parameters********/
void Diagnostic_SetNLParam(TP_T* pTp,uint8_t TimeAs, uint8_t TimeBs, uint8_t TimeCr, uint8_t TimeAr, uint8_t TimeBr, uint8_t TimeCs, 
	uint8_t BlockSize, uint8_t m_STmin, uint8_t FillData)
{
	NetworkLayer_SetParam(pTp,TimeAs , TimeBs , TimeCr , TimeAr , TimeBr , TimeCs , BlockSize , m_STmin , HALF_DUPLEX , DIAGNOSTIC , N_Sa ,  N_Ta , PHYSICAL , 0 , FillData);
}

/*
统一诊断服务初始化
requestId：物理请求ID
responseId：响应ID
funRequestId：功能请求ID
EEPromStartAddr：E2PROW起始地址
EEpromSize：E2PROW大小
sendFun：CAN发送函数
p2CanServerMax：
p2ECanServerMax：
*/
void Diagnostic_Init(TP_T* pTp,uint32_t requestId, uint32_t responseId, uint32_t funRequestId,uint16_t EEPromStartAddr, uint16_t EEpromSize, SendCANFun sendFun,uint16_t p2CanServerMax, uint16_t p2ECanServerMax)
{	
	TesterPhyID = requestId;
	EcuID = responseId;
	TesterFunID = funRequestId;
	
	N_Ta = (uint8_t)EcuID;
	N_Sa = (uint8_t)(EcuID >> 8);
	NetworkLayer_InitParam(pTp,TesterPhyID, TesterFunID , EcuID , sendFun,osKernelGetTickCount);

	P2CanServerMax = p2CanServerMax;
	P2ECanServerMax = p2ECanServerMax;
	
	EnableDTCDetect = TRUE;
	HighVoltage = FALSE;
	ResponsePending = FALSE;
	m_CurrSessionType = ECU_DEFAULT_SESSION;
	
	WaitConfirmBeforeJump = FALSE;
	WaitConfimBeforeReset = FALSE;
	SessionSupport = 0;

	DTCAdded = 0;
	DIDAdded = 0;
	DTCGroupAdded = 0;
	ResetTypeSupport = 0;
	ResetCallBackFun = NULL;
	CommTypeSupport = 0;
	commCallBack = NULL;
	ModuleEEpromStartAddr = EEPromStartAddr;
	EEpromSizeForUse = EEpromSize;
	EEpromUsed = 0;
	memset(UnlockList, 0 ,sizeof(UnlockList));
}
/*========interface for application layer setting diagnostic parameters==============*/
void GenerateSeed(uint8_t *seed, uint32_t length)
{
	uint32_t SystemTick = osKernelGetTickCount();
	seed[0] = (uint8_t)SystemTick ^ (uint8_t)(SystemTick >> 3);
	seed[1] = (uint8_t)SystemTick ^ (uint8_t)(SystemTick >> 7);
	seed[2] = (uint8_t)SystemTick ^ (uint8_t)(SystemTick >> 11);
	seed[3] = (uint8_t)(SystemTick>>3) ^ (uint8_t)(SystemTick >> 11);
}

void GotoSession(SessionType session)
{
	if(session != ECU_DEFAULT_SESSION)
	{
		DiagTimer_Set(&S3serverTimer, 5000);
	}
	else
	{
		/***********when s3 timeout,session change, comunication recover********/
		if(commCallBack != NULL)
		{
			commCallBack(ERXTX, NWMCM_NCM);
		}
		/***********when s3 timeout,session change, DTC enable***********/
		EnableDTCDetect = TRUE;
	}
	m_CurrSessionType = session;
	m_SecurityLevel = LEVEL_ZERO;//session change ECU lock even if from extended session to extended session
	if(m_UnlockStep != WAIT_DELAY)
	{
		m_UnlockStep = WAIT_SEED_REQ;//by ukign 2016.04.01
	}

	if(m_CurrSessionType == ECU_PAOGRAM_SESSION)
	{
		WaitConfirmBeforeJump = TRUE;
	}
}


void Service10PosResponse(SessionType session)
{
	DiagnosticBuffTX[0] = 0x50;
	DiagnosticBuffTX[1] = session;
	DiagnosticBuffTX[2] = (uint8_t)(P2CanServerMax >> 8);
	DiagnosticBuffTX[3] = (uint8_t)P2CanServerMax;
	DiagnosticBuffTX[4] = (uint8_t)(P2ECanServerMax >> 8);
	DiagnosticBuffTX[5] = (uint8_t)P2ECanServerMax;
	ResponseLength = 6;
}

void Service10Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	uint8_t SubFunction;
	uint8_t suppressPosRspMsgIndicationBit;
	m_NRC = PR;
	//printf("service 10 handler\r\n");
	if(length == 2)
	{
		//服务响应支持
		if(Service10SupressSupproted())
		{
			SubFunction = *(MessageData + 1) & 0x7F;
			suppressPosRspMsgIndicationBit = *(MessageData + 1) & 0x80;
		}
		else
		{
			SubFunction = *(MessageData + 1);
			suppressPosRspMsgIndicationBit = 0;
		}
		
		switch(SubFunction)/* get sub-function parameter value without bit 7 */
		{
			case ECU_DEFAULT_SESSION: /* test if sub-function parameter value is supported */
				if(!Service10Sub01Supported())
				{
					m_NRC = SFNS;
				}
				break;
			case ECU_EXTENED_SESSION: /* test if sub-function parameter value is supported */
				if(!Service10Sub03Supported())
				{
					m_NRC = SFNS;
				}
				else
				{
					if(m_CurrSessionType == ECU_PAOGRAM_SESSION && !Service10Sub02To03OK())
					{
						m_NRC = SFNSIAS;
					}
				}
				break;
			case ECU_PAOGRAM_SESSION: /* test if sub-function parameter value is supported */
				if(!Service10Sub02Supported())
				{
					m_NRC = SFNS;
				}
				else
				{
					if(m_CurrSessionType == ECU_DEFAULT_SESSION && !Service10Sub01To02OK())
					{
						m_NRC = SFNSIAS;
					}
				}
				break;
			case ECU_FACTORY_SESSION:
				if(N_TAType == PHYSICAL)
				{
					if(m_CurrSessionType == ECU_EXTENED_SESSION)
					{
						
					}
					else
					{
						m_NRC = SFNS;
					}
				}
				else
				{
					m_NRC = SFNS;
				}
				break;
			default:
				m_NRC = SFNS; /* NRC 0x12: sub-functionNotSupported *///
		}
	}
	else
	{
		m_NRC = IMLOIF;
	}
	
	if ((suppressPosRspMsgIndicationBit) && (m_NRC == 0x00) && (ResponsePending == FALSE))
	{
		suppressResponse = TRUE; /* flag to NOT send a positive response message */
	}
	else
	{
		suppressResponse = FALSE; /* flag to send the response message */
		Service10PosResponse((SessionType)SubFunction);
	}

	if(m_NRC == PR)
	{
		GotoSession((SessionType)SubFunction);
	}
}

void Service11Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	uint8_t SubFunction;
	uint8_t suppressPosRspMsgIndicationBit;
	m_NRC = PR;
	if(length == 2)
	{
		if(Service11SupressSupported())
		{
			SubFunction = *(MessageData + 1) & 0x7F;
			m_EcuResetType = (EcuResetType)SubFunction;
			suppressPosRspMsgIndicationBit = *(MessageData + 1) & 0x80;
		}
		else
		{
			SubFunction = *(MessageData + 1);
			m_EcuResetType = (EcuResetType)SubFunction;
			suppressPosRspMsgIndicationBit = 0;
		}
		
		switch(SubFunction)/* get sub-function parameter value without bit 7 */
		{
			case HARD_RESET:
				{
					if(!Service11Sub01Supported())
					{
						m_NRC = SFNS;
					}
				}
				break;
			case KEY_OFF_ON_RESET: /* test if sub-function parameter value is supported */
				{
					if(!Service11Sub02Supported())
					{
						m_NRC = SFNS;
					}
				}
				break;
			case SOFT_RESET: /* test if sub-function parameter value is supported */
				{
					if(!Service11Sub03Supported())
					{
						m_NRC = SFNS;
					}
				}
				break;
			case ENABLE_RAPID_POWER_SHUTDOWN:
				{
					if(!Service11Sub04Supported())
					{
						m_NRC = SFNS;
					}
				}
				break;
			case DISABLE_RAPID_POWER_SHUTDOWN:
				{
					if(!Service11Sub05Supported())
					{
						m_NRC = SFNS;
					}
				}
				break;
			default:
				m_NRC = SFNS; /* NRC 0x12: sub-functionNotSupported */
		}
		if(m_NRC == PR && !conditionCallBackFun())
		{
			m_NRC = CNC;
		}
	}
	else
	{
		m_NRC = IMLOIF;
	}
	
	if ( (suppressPosRspMsgIndicationBit) && (m_NRC == 0x00) && (ResponsePending == FALSE))
	{
		suppressResponse = TRUE; /* flag to NOT send a positive response message */
	}
	else
	{
		suppressResponse = FALSE; /* flag to send the response message */		
		DiagnosticBuffTX[0] = CurrentService + 0x40;
		DiagnosticBuffTX[1] = SubFunction;
		ResponseLength = 2;
	}

	if(m_NRC == PR)
	{
		if(suppressResponse == FALSE)//需要正响应时，等级响应结束
		{
			WaitConfimBeforeReset = TRUE;
		}
		else//不需要正响应时直接复位
		{
			if(ResetCallBackFun != NULL)
			{
				ResetCallBackFun(m_EcuResetType);
			}
		}
	}
}

void Service27Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	uint8_t SubFunction ;
	uint8_t suppressPosRspMsgIndicationBit;
	m_NRC = PR;
	if(length >= 2)//min length check
	{
		uint8_t index = 0;
		bool subFunctionExist = FALSE;
		bool subFunctionSupInSession = FALSE;
		SubFunction = *(MessageData + 1) & 0x7F;
		suppressPosRspMsgIndicationBit = *(MessageData + 1) & 0x80;
		while(index < 3 && (!subFunctionExist))
		{
			if(UnlockList[index].valid == TRUE && (UnlockList[index].seedID == SubFunction || UnlockList[index].keyID == SubFunction))
			{
				subFunctionExist = TRUE;
				if(m_CurrSessionType == ECU_DEFAULT_SESSION)
				{
					if(UnlockList[index].subFunctionSupported & SUB_DEFAULT)
					{
						subFunctionSupInSession = TRUE;
					}
					else
					{
						subFunctionSupInSession = FALSE;
					}
				}
				else if(m_CurrSessionType == ECU_PAOGRAM_SESSION)
				{
					if(UnlockList[index].subFunctionSupported & SUB_PROGRAM)
					{
						subFunctionSupInSession = TRUE;
					}
					else
					{
						subFunctionSupInSession = FALSE;
					}
				}
				else if(m_CurrSessionType == ECU_EXTENED_SESSION)
				{
					if(UnlockList[index].subFunctionSupported & SUB_EXTENDED)
					{
						subFunctionSupInSession = TRUE;
					}
					else
					{
						subFunctionSupInSession = FALSE;
					}
				}
				else if(m_CurrSessionType == ECU_FACTORY_SESSION)
				{
					if(UnlockList[index].subFunctionSupported & SUB_FACTORY)
					{
						subFunctionSupInSession = TRUE;
					}
					else
					{
						subFunctionSupInSession = FALSE;
					}
				}
			}
			else
			{
				index++;
			}
		}
		
		if(subFunctionExist && subFunctionSupInSession)//sub function check ok
		{
			if(UnlockList[index].seedID == SubFunction)//request seed
			{
				if(length == 2)//length check again
				{
					if(m_UnlockStep == WAIT_DELAY)
					{
						m_NRC = RTDNE;
					}
					else if(m_UnlockStep == UNLOCKED && m_SecurityLevel == UnlockList[index].level)//by ukign 20160401,when ECU unlocked,retrun seed is all zero
					{
						DiagnosticBuffTX[0] = 0x67;
						DiagnosticBuffTX[1] = SubFunction;
						DiagnosticBuffTX[2] = 0;
						DiagnosticBuffTX[3] = 0;
						DiagnosticBuffTX[4] = 0;
						DiagnosticBuffTX[5] = 0;
						ResponseLength = UnlockList[index].KeySize + 2;
					}
					else
					{
						GenerateSeed(Seed,4);
						DiagnosticBuffTX[0] = 0x67;
						DiagnosticBuffTX[1] = SubFunction;
						DiagnosticBuffTX[2] = Seed[0];
						DiagnosticBuffTX[3] = Seed[1];
						if(UnlockList[index].KeySize == 2)
						{
							
						}
						else if(UnlockList[index].KeySize == 4)
						{
							DiagnosticBuffTX[4] = Seed[2];
							DiagnosticBuffTX[5] = Seed[3];
						}
						m_UnlockStep = WAIT_KEY;
						ResponseLength = UnlockList[index].KeySize + 2;
					}
				}
				else
				{
					m_NRC = IMLOIF;
				}
			}
			else if(SubFunction ==  UnlockList[index].keyID)//send key
			{
				if(length == UnlockList[index].KeySize + 2)
				{
					if(m_UnlockStep == WAIT_KEY)
					{
						*((uint32_t*)key) = UnlockList[index].UnlockFunction(*(uint32_t*)Seed);
						if(((key[0] == *(MessageData + 2) && key[1] == *(MessageData + 3)) && UnlockList[index].KeySize == 2) ||
							((key[0] == *(MessageData + 2) && key[1] == *(MessageData + 3) && key[2] == *(MessageData + 4) && key[3] == *(MessageData + 5)) && UnlockList[index].KeySize == 4))
						{
							m_UnlockStep = UNLOCKED;
							UnlockList[index].FaultCounter = 0;
							m_SecurityLevel = UnlockList[index].level;
							DiagnosticBuffTX[0] = 0x67;
							DiagnosticBuffTX[1] = SubFunction;
							ResponseLength = 2;
						}
						else 
						{
							UnlockList[index].FaultCounter++;
							if(UnlockList[index].FaultCounter >= 3)
							{
								m_NRC = ENOA;
								m_UnlockStep = WAIT_DELAY;
								DiagTimer_Set(&UnlockList[index].SecurityLockTimer , UnlockList[index].UnlockFailedDelayTime);
							}
							else
							{
								m_NRC = IK;
								m_UnlockStep = WAIT_SEED_REQ;
							}
						}
						EE_WriteVariable(UnlockList[index].FaultCounterAddr, 1 ,&UnlockList[index].FaultCounter);					
					}
					else if(m_UnlockStep == WAIT_DELAY)
					{
						m_NRC = RTDNE;
					}
					else if(m_UnlockStep == WAIT_SEED_REQ)//send key before request seed order error
					{
						m_NRC = RSE;
					}
					else//unlocked condition error
					{
						m_NRC = RSE;
					}
				}
				else
				{
					m_NRC = IMLOIF;
				}
			}
		}
		else
		{
			if(!subFunctionExist)
			{
				m_NRC = SFNS;
			}
			else if(!subFunctionSupInSession)
			{
				m_NRC = SFNSIAS;
			}
		}
	}
	else
	{
		m_NRC = IMLOIF;
	}
	if((suppressPosRspMsgIndicationBit) && (m_NRC == 0x00) && (ResponsePending == FALSE))
	{
		suppressResponse = TRUE; /* flag to NOT send a positive response message */
	}
	else
	{
		suppressResponse = FALSE; /* flag to send the response message */
	}
}

void Service28Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	uint8_t SubFunction;
	uint8_t ControlParam;
	uint8_t suppressPosRspMsgIndicationBit;
	m_NRC = PR;
	if(length == 3)
	{
		if(Service28SupressSupported())
		{
			SubFunction = *(MessageData + 1) & 0x7F;
			suppressPosRspMsgIndicationBit = *(MessageData + 1) & 0x80;
		}
		else
		{
			SubFunction = *(MessageData + 1);
			suppressPosRspMsgIndicationBit = 0;
		}
		ControlParam = *(MessageData + 2);
		
		switch(SubFunction)/* get sub-function parameter value without bit 7 */
		{
			case ERXTX:
				{
					if(!Service28Sub00Suppoted())
					{
						m_NRC = SFNS;
					}
				}
				break;
			case ERXDTX: /* test if sub-function parameter value is supported */
				{
					if(!Service28Sub01Suppoted())
					{
						m_NRC = SFNS;
					}
				}
				break;
			case DRXETX: /* test if sub-function parameter value is supported */
				{
					if(!Service28Sub02Suppoted())
					{
						m_NRC = SFNS;
					}
				}
				break;
			case DRXTX: /* test if sub-function parameter value is supported */
				{
					if(!Service28Sub03Suppoted())
					{
						m_NRC = SFNS;
					}
				}
				break;
			default:
				m_NRC = SFNS; /* NRC 0x12: sub-functionNotSupported */
		}
		
		if(m_NRC == 0)
		{
			switch(ControlParam)
			{
				case NCM:
					{
						if(!Service28Type01Suppoted())
						{
							m_NRC = ROOR;
						}
					}
					break;
				case NWMCM: /* test if sub-function parameter value is supported */
					{
						if(!Service28Type02Suppoted())
						{
							m_NRC = ROOR;
						}
					}
					break;
				case NWMCM_NCM: /* test if sub-function parameter value is supported */
					{
						if(!Service28Type03Suppoted())
						{
							m_NRC = ROOR;
						}
					}
					break;
				default:
					m_NRC = ROOR; /* NRC 0x12: sub-functionNotSupported */
			}
		}
		
	}
	else
	{
		m_NRC = IMLOIF;
	}
	
	if((suppressPosRspMsgIndicationBit) && (m_NRC == 0x00) && (ResponsePending == FALSE))
	{
		suppressResponse = TRUE; /* flag to NOT send a positive response message */
	}
	else
	{
		suppressResponse = FALSE; /* flag to send the response message */
	}

	if(m_NRC == 0x00)
	{
		if(commCallBack != NULL)
		{
			commCallBack((CommulicationType)SubFunction, (communicationParam)ControlParam);
		}
		DiagnosticBuffTX[0] = CurrentService + 0x40;
		DiagnosticBuffTX[1] = SubFunction;
		ResponseLength = 2;
	}
}

void Service3EHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	uint8_t SubFunction;
	uint8_t suppressPosRspMsgIndicationBit;
	m_NRC = PR;
	if(length == 2)
	{
		if(Service3ESupressSupported())
		{
			SubFunction = *(MessageData + 1) & 0x7F;
			suppressPosRspMsgIndicationBit = *(MessageData + 1) & 0x80;
		}
		else
		{
			SubFunction = *(MessageData + 1);
			suppressPosRspMsgIndicationBit = 0;
		}
		
		if(SubFunction != 0)
		{
			m_NRC = SFNS;
		}
		else
		{
			DiagnosticBuffTX[0] = CurrentService+ 0x40;
			DiagnosticBuffTX[1] = SubFunction;
			ResponseLength = 2;
		}
	}
	else
	{
		m_NRC = IMLOIF;
	}
	
	if ((suppressPosRspMsgIndicationBit) && (m_NRC == 0x00) && (ResponsePending == FALSE))
	{
		suppressResponse = TRUE; /* flag to NOT send a positive response message */
	}
	else
	{
		suppressResponse = FALSE; /* flag to send the response message */
	}
}

void Service83Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	
}

void Service84Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	
}

void Service85Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	uint8_t SubFunction;
	uint8_t suppressPosRspMsgIndicationBit;
	m_NRC = PR;
	if(length == 2)
	{
		if(Service85SupressSupported())
		{
			SubFunction = *(MessageData + 1) & 0x7F;
			suppressPosRspMsgIndicationBit = *(MessageData + 1) & 0x80;
		}
		else
		{
			SubFunction = *(MessageData + 1);
			suppressPosRspMsgIndicationBit = 0;
		}
		
		switch(SubFunction)/* get sub-function parameter value without bit 7 */
		{
			case 0x01:
				{
					EnableDTCDetect = TRUE;
					DiagnosticBuffTX[0] = CurrentService + 0x40;
					DiagnosticBuffTX[1] = SubFunction;
					ResponseLength = 2;
				}
				break;
			case 0x02: /* test if sub-function parameter value is supported */
				{
					EnableDTCDetect = FALSE;
					DiagnosticBuffTX[0] = CurrentService + 0x40;
					DiagnosticBuffTX[1] = SubFunction;
					ResponseLength = 2;
				}
				break;
			default:
				m_NRC = SFNS; /* NRC 0x12: sub-functionNotSupported */
		}
	}
	else
	{
		m_NRC = IMLOIF;
	}
	
	if ((suppressPosRspMsgIndicationBit) && (m_NRC == 0x00) && (ResponsePending == FALSE))
	{
		suppressResponse = TRUE; /* flag to NOT send a positive response message */
	}
	else
	{
		suppressResponse = FALSE; /* flag to send the response message */
	}
}

void Service86Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	
}

void Service87Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	
}

DIDNode* SearchDidNode(uint16_t DID)
{
	uint8_t i;
	for(i = 0; i < DIDAdded ; i++)
	{
		if(DIDS[i].ID == DID)
		{
			return DIDS + i;
		}
	}
	return NULL;
}

void Service22Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	bool suppressPosRspMsgIndicationBit = FALSE;
	m_NRC = PR;
	if(length == 3)
	{
		uint16_t DID = (*(MessageData + 1) << 8) + *(MessageData + 2);
		DIDNode* didNode = SearchDidNode(DID);
		if(didNode == NULL)
		{
			m_NRC = ROOR;//PID not in our PID list
		}
		else if(didNode->didType == IO_DID)
		{
			if(didNode->RWAttr == WRITEONLY)
			{
				m_NRC = ROOR;//mabe a IO Control DID
			}
			else
			{
				DiagnosticBuffTX[0] = 0x62;
				DiagnosticBuffTX[1] = *(MessageData + 1);
				DiagnosticBuffTX[2] = *(MessageData + 2);
				memcpy(DiagnosticBuffTX + 3 , didNode->dataPointer ,didNode->dataLength);
				ResponseLength = didNode->dataLength + 3;
			}
		}
		else
		{
			if(didNode->RWAttr == WRITEONLY)
			{
				m_NRC = ROOR;//this DID maybe supported by 2E service but not supported by 22 service
			}
			else
			{
				DiagnosticBuffTX[0] = 0x62;
				DiagnosticBuffTX[1] = *(MessageData + 1);
				DiagnosticBuffTX[2] = *(MessageData + 2);
				if(didNode->didType == EEPROM_DID)
				{
					EE_ReadVariable(didNode->EEpromAddr , didNode->dataLength , DiagnosticBuffTX+3);
				}
				else
				{
					memcpy(DiagnosticBuffTX + 3 , didNode->dataPointer ,didNode->dataLength);
				}
				ResponseLength = didNode->dataLength + 3;
			}
		}
	}
	else
	{
		m_NRC = IMLOIF;
	}

	if ((suppressPosRspMsgIndicationBit) && (m_NRC == 0x00) && (ResponsePending == FALSE))
	{
		suppressResponse = TRUE; /* flag to NOT send a positive response message */
	}
	else
	{
		suppressResponse = FALSE; /* flag to send the response message */
	}
}

void Service23Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	
}

void Service24Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	
}

void Service2AHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	
}

void Service2CHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	
}


void Service2EHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	bool suppressPosRspMsgIndicationBit = FALSE;
	m_NRC = PR;
	if(length >= 3)
	{
		uint16_t DID = (*(MessageData + 1) << 8) + *(MessageData + 2);
		DIDNode* didNode = SearchDidNode(DID);
		if(didNode == NULL)
		{
			m_NRC = ROOR;//PID not in our PID list
		}
		else if(didNode->didType == IO_DID)
		{
			m_NRC = ROOR;//mabe a IO Control DID
		}
		else if(didNode->didType == REALTIME_DID)
		{
			if(didNode->dataLength + 3 == length)
			{
				if(m_CurrSessionType != ECU_FACTORY_SESSION)
				{
					memcpy(didNode->dataPointer , MessageData + 3, didNode->dataLength);
					DiagnosticBuffTX[0] = 0x6E;
					DiagnosticBuffTX[1] = *(MessageData + 1);
					DiagnosticBuffTX[2] = *(MessageData + 2);
					ResponseLength = 3;
				}
				else
				{
					m_NRC = ROOR;//fatcory mode not support realtime DID write
				}
			}
			else
			{
				m_NRC = IMLOIF;
			}
		}
		else if(didNode->didType == EEPROM_DID)
		{
			if(didNode->RWAttr == READONLY)
			{
				if(didNode->SupportWriteInFactoryMode == TRUE && m_CurrSessionType == ECU_FACTORY_SESSION)
				{
					if(m_SecurityLevel == LEVEL_FOUR)
					{
						if(didNode->dataLength + 3 == length)
						{
							EE_WriteVariable(didNode->EEpromAddr, didNode->dataLength , MessageData + 3);
							DiagnosticBuffTX[0] = 0x6E;
							DiagnosticBuffTX[1] = *(MessageData + 1);
							DiagnosticBuffTX[2] = *(MessageData + 2);
							ResponseLength = 3;
						}
						else
						{
							m_NRC = IMLOIF;
						}
					}
					else
					{
						m_NRC = SAD;
					}
				}
				else
				{
					m_NRC = ROOR;//this DID maybe supported by 22 service but not supported by 2E service
				}
			}
			else 
			{
				if(didNode->dataLength + 3 == length)
				{
					if(m_CurrSessionType == ECU_FACTORY_SESSION)
					{
						m_NRC = ROOR;
					}
					else
					{
						EE_WriteVariable(didNode->EEpromAddr, didNode->dataLength , MessageData + 3);
						DiagnosticBuffTX[0] = 0x6E;
						DiagnosticBuffTX[1] = *(MessageData + 1);
						DiagnosticBuffTX[2] = *(MessageData + 2);
						ResponseLength = 3;
					}
				}
				else
				{
					m_NRC = IMLOIF;
				}
			}
		}
	}
	else
	{
		m_NRC = IMLOIF;
	}
	
	if ( (suppressPosRspMsgIndicationBit) && (m_NRC == 0x00) && (ResponsePending == FALSE))
	{
		suppressResponse = TRUE; /* flag to NOT send a positive response message */
	}
	else
	{
		suppressResponse = FALSE; /* flag to send the response message */
	}
}


void Service3DHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	
}

bool SearchDTCGroup(uint32_t group)
{
	uint8_t i;
	for(i = 0; i < DTCGroupAdded; i++)
	{
		if(DTCGROUPS[i].GroupID == group)
		{
			return TRUE;
		}
	}
	return FALSE;
}

void Service14Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	m_NRC = PR;
	suppressResponse = FALSE;
	if(length == 4)
	{
		uint32_t Group = ((*(MessageData + 1) << 16) + (*(MessageData + 2) << 8) + *(MessageData + 3)) & 0xFFFFFF;
		if(SearchDTCGroup(Group))//14229-1 page 183,send pos even if no DTCS are stored
		{
			DiagnosticBuffTX[0] = CurrentService + 0x40;
			ResponseLength = 1;
			Diagnostic_ClearDTC(Group);
		}
		else
		{
			m_NRC = ROOR;						//范围越界
		}
	}
	else
	{
		m_NRC = IMLOIF;						//长度错误
	}
}

void SearchDTCByMaskAndFillResponse(uint8_t mask)
{
	uint8_t i;
	for(i = 0 ; i < DTCAdded;i++)
	{
		if((DTCS[i].DTCStatus.DTCStatusByte & mask & DtcAvailibaleMask) != 0)
		{
			DiagnosticBuffTX[ResponseLength++] =  (uint8_t)(DTCS[i].DTCCode >> 16);
			DiagnosticBuffTX[ResponseLength++] =  (uint8_t)(DTCS[i].DTCCode >> 8);
			DiagnosticBuffTX[ResponseLength++] =  (uint8_t)(DTCS[i].DTCCode);
			DiagnosticBuffTX[ResponseLength++] =  (uint8_t)(DTCS[i].DTCStatus.DTCStatusByte & DtcAvailibaleMask);
		}
	}
}

void FillAllDTCResponse()
{
	uint8_t i;
	for(i = 0 ; i < DTCAdded;i++)
	{
		DiagnosticBuffTX[ResponseLength++] =  (uint8_t)(DTCS[i].DTCCode >> 16);
		DiagnosticBuffTX[ResponseLength++] =  (uint8_t)(DTCS[i].DTCCode >> 8);
		DiagnosticBuffTX[ResponseLength++] =  (uint8_t)(DTCS[i].DTCCode);
		DiagnosticBuffTX[ResponseLength++] =  (uint8_t)(DTCS[i].DTCStatus.DTCStatusByte & DtcAvailibaleMask);
	}
}

uint16_t GetDTCNumberByMask(uint8_t mask)
{
	uint8_t i;
	uint16_t number = 0;
	for(i = 0; i < DTCAdded ; i++)
	{

		if((DTCS[i].DTCStatus.DTCStatusByte & mask & DtcAvailibaleMask) != 0)
		{
			number++;
		}
	}
	return number;
}

DTCNode* GetDTCNodeByCode(uint32_t dtcCode)
{
	uint8_t i;
	for(i = 0 ; i < DTCAdded;i++)
	{	
		if((DTCS[i].DTCCode & 0xFFFFFF) == (dtcCode & 0xFFFFFF))
		{
			return DTCS+i;
		}
	}
	return NULL;
}

/*
功能：读取快照数据信息
参数：

*/
void FillDTCSnapshotReponse(uint16_t EEPromAddr, uint8_t ReordMask)
{
	uint8_t i;
	uint8_t snapshotRecordNumber = 0;
	uint8_t SnapshotDataLength = 0;
	uint8_t SnapShotsBuff[30];
	ResponseLength = 8;
	if(ReordMask == 0xFF || ReordMask == 1)
	{
		for(i = 0 ;  i < SnapShotAdded  && SnapshotDataLength < sizeof(SnapShotsBuff); i ++)
		{
			SnapshotDataLength += SnapShots[i].dataLength;
		}
		EE_ReadVariable(EEPromAddr,SnapshotDataLength,SnapShotsBuff);
		SnapshotDataLength = 0;
		for(i = 0 ; i < SnapShotAdded ; i++)
		{
			if(SnapShots[i].snapshotRecord == ReordMask)
			{
				snapshotRecordNumber++;
				DiagnosticBuffTX[ResponseLength] = (uint8_t)(SnapShots[i].snapshotID >> 8);
				DiagnosticBuffTX[ResponseLength + 1] = (uint8_t)(SnapShots[i].snapshotID);
				ResponseLength += 2;
				memcpy(&DiagnosticBuffTX[ResponseLength],&SnapShotsBuff[SnapshotDataLength],SnapShots[i].dataLength);
				SnapshotDataLength += SnapShots[i].dataLength;
				ResponseLength += SnapShots[i].dataLength;
			}
		}
		DiagnosticBuffTX[7]  = snapshotRecordNumber;//个数
	}
//	else
//	{
//		uint8_t currentRecord = SnapShots[0].snapshotRecord;
//		uint8_t currentRecordNumber = 0;
//		uint8_t currentRecordSizeIndex = 7;
//		for(i = 0 ; i < SnapShotAdded ; i++)
//		{
//			if(SnapShots[i].snapshotRecord == currentRecord)
//			{
//				currentRecordNumber++;
//				DiagnosticBuffTX[ResponseLength] = (uint8_t)(SnapShots[i].snapshotID >> 8);
//				DiagnosticBuffTX[ResponseLength + 1] = (uint8_t)(SnapShots[i].snapshotID);
//				ResponseLength += 2;
//				EE_ReadVariable(EEPromAddr, SnapShots[i].dataLength , DiagnosticBuffTX + ResponseLength);
//				ResponseLength += SnapShots[i].dataLength;
//			}
//			else
//			{
//				currentRecord = SnapShots[i].snapshotRecord;
//				DiagnosticBuffTX[currentRecordSizeIndex] = currentRecordNumber;
//				currentRecordSizeIndex = ResponseLength;
//				ResponseLength++;
//				currentRecordNumber = 1;
//				DiagnosticBuffTX[ResponseLength] = (uint8_t)(SnapShots[i].snapshotID >> 8);
//				DiagnosticBuffTX[ResponseLength + 1] = (uint8_t)(SnapShots[i].snapshotID);
//				ResponseLength += 2;
//				EE_ReadVariable(EEPromAddr, SnapShots[i].dataLength , DiagnosticBuffTX + ResponseLength);
//				ResponseLength += SnapShots[i].dataLength;
//			}
//		}
//		DiagnosticBuffTX[currentRecordSizeIndex]  = currentRecordNumber;
//	}
}

void Service19Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	uint8_t SubFunction;
	uint8_t suppressPosRspMsgIndicationBit;
	uint8_t mask = *(MessageData + 2);
	m_NRC = PR;
	if(length >= 2)
	{
		SubFunction = *(MessageData + 1);
		suppressPosRspMsgIndicationBit = FALSE;
		switch(SubFunction)
		{
			case REPORT_DTCNUMBER_BY_MASK:				//0x01 通过状态掩码去查找与其相匹配的故障个数
				{
					if(length == 3)
					{
						uint16_t number= GetDTCNumberByMask(mask);
						DiagnosticBuffTX[0]  = 0x59;
						DiagnosticBuffTX[1]  = 0x01;
						DiagnosticBuffTX[2]  = DtcAvailibaleMask;
						DiagnosticBuffTX[3]  = 0x00;//ISO15031 DTCFormat
						DiagnosticBuffTX[4]  = (uint8_t)(number >> 8);
						DiagnosticBuffTX[5]  = (uint8_t)(number);
						ResponseLength = 6;
					}
					else
					{
						m_NRC = IMLOIF;
					}
				}
				break;											//0x02  按照定义的状态掩码的形式去查找匹配的故障
			case REPORT_DTCCODE_BY_MASK: /* test if sub-function parameter value is supported */
				{
					if(length == 3)
					{
						DiagnosticBuffTX[0]  = CurrentService  + 0x40;
						DiagnosticBuffTX[1]  = SubFunction;
						DiagnosticBuffTX[2]  = DtcAvailibaleMask;
						ResponseLength = 3;
						SearchDTCByMaskAndFillResponse(mask);
					}
					else
					{
						m_NRC = IMLOIF;
					}
				}
				break;											// 0x03 报告DBC快照标识
			case REPORT_DTCSNAPSHOT_BY_ID: /* test if sub-function parameter value is supported */
				{
					m_NRC = SFNS;
				}
				break;
			case REPORT_DTCSNAPSHOT_BY_DTCNUMBER:		//0x04  请求指定故障码（DTC）的快照信息
				{
					if(length == 6)
					{
						uint32_t dtcCode = (MessageData[2] << 16 | MessageData[3] << 8 | MessageData[4] << 0);
						DTCNode* tempNode = GetDTCNodeByCode(dtcCode);
						if(tempNode != NULL)
						{
							DiagnosticBuffTX[0]  = CurrentService  + 0x40;
							DiagnosticBuffTX[1]  = SubFunction;
							DiagnosticBuffTX[2]  = (uint8_t)(tempNode->DTCCode >> 16);
							DiagnosticBuffTX[3]  = (uint8_t)(tempNode->DTCCode >> 8);
							DiagnosticBuffTX[4]  = (uint8_t)(tempNode->DTCCode);
							DiagnosticBuffTX[5]  = (tempNode->DTCStatus.DTCStatusByte) & DtcAvailibaleMask;
							DiagnosticBuffTX[6]  = *(MessageData + 5);
							ResponseLength = 7;
							FillDTCSnapshotReponse(tempNode->SnapShotEEpromAddr , *(MessageData + 5));
							if(DiagnosticBuffTX[7] == 0)
							{
								m_NRC = ROOR;
							}
						}
						else
						{
							m_NRC = ROOR;
						}
					}
					else
					{
						m_NRC = IMLOIF;
					}
				}
				break;
			case REPORT_DTCEXTEND_DATA_BY_DTCNUMBER:		//0x06  请求所有支持的DTC信息
				{
					if(length == 6)
					{
						uint32_t dtcCode = (MessageData[2] << 16 | MessageData[3] << 8 | MessageData[4] << 0);
						DTCNode* tempNode = GetDTCNodeByCode(dtcCode);
						if(tempNode != NULL)
						{
							uint8_t DataRecordNumber = *(MessageData + 5);
							DiagnosticBuffTX[0]  = CurrentService  + 0x40;
							DiagnosticBuffTX[1]  = SubFunction;
							DiagnosticBuffTX[2]  = (uint8_t)(tempNode->DTCCode >> 16);
							DiagnosticBuffTX[3]  = (uint8_t)(tempNode->DTCCode >> 8);
							DiagnosticBuffTX[4]  = (uint8_t)(tempNode->DTCCode);
							DiagnosticBuffTX[5]  = (tempNode->DTCStatus.DTCStatusByte) & DtcAvailibaleMask;
							ResponseLength = 6;
							
							if(DataRecordNumber == 0xFF)
							{
								if(AgingCounterRecord != 0)
								{
									DiagnosticBuffTX[ResponseLength++] = AgingCounterRecord;
									DiagnosticBuffTX[ResponseLength++] = tempNode->OldCounter;
								}
								
								if(AgedCounterRecord != 0)
								{
									DiagnosticBuffTX[ResponseLength++] = AgedCounterRecord;
									DiagnosticBuffTX[ResponseLength++] = tempNode->GoneCounter;
								}

								if(OccurenceCounterRecord != 0)
								{
									DiagnosticBuffTX[ResponseLength++] = OccurenceCounterRecord;
									DiagnosticBuffTX[ResponseLength++] = tempNode->FaultOccurrences;
								}

								if(PendingCounterRecord != 0)
								{
									DiagnosticBuffTX[ResponseLength++] = PendingCounterRecord;
									DiagnosticBuffTX[ResponseLength++] = tempNode->TripCounter;
								}
							}
							else if(DataRecordNumber == 0)
							{
								m_NRC = ROOR;
							}
							else if(DataRecordNumber == AgingCounterRecord)
							{
								DiagnosticBuffTX[ResponseLength++] = AgingCounterRecord;
								DiagnosticBuffTX[ResponseLength++] = tempNode->OldCounter;
							}
							else if(DataRecordNumber == AgedCounterRecord)
							{
								DiagnosticBuffTX[ResponseLength++] = AgedCounterRecord;
								DiagnosticBuffTX[ResponseLength++] = tempNode->GoneCounter;
							}
							else if(DataRecordNumber == OccurenceCounterRecord)
							{
								DiagnosticBuffTX[ResponseLength++] = OccurenceCounterRecord;
								DiagnosticBuffTX[ResponseLength++] = tempNode->FaultOccurrences;
							}
							else if(DataRecordNumber == PendingCounterRecord)
							{
								DiagnosticBuffTX[ResponseLength++] = PendingCounterRecord;
								DiagnosticBuffTX[ResponseLength++] = tempNode->TripCounter;
							}
							else
							{
								m_NRC = ROOR;
							}
						}
						else
						{
							m_NRC = ROOR;
						}
					}
					else
					{
						m_NRC = IMLOIF;
					}
				}
				break;
			case REPORT_SUPPORTED_DTC:
				{
					if(length == 2)
					{
						DiagnosticBuffTX[0]  = CurrentService + 0x40;
						DiagnosticBuffTX[1]  = SubFunction;
						DiagnosticBuffTX[2]  = DtcAvailibaleMask;
						ResponseLength = 3;
						FillAllDTCResponse();
					}
					else
					{
						m_NRC = IMLOIF;
					}
				}
				break;
			default:
				m_NRC = SFNS; /* NRC 0x12: sub-functionNotSupported */
		}
	}
	else
	{
		m_NRC = IMLOIF;
	}
	
	if ( (suppressPosRspMsgIndicationBit) && (m_NRC == 0x00) && (ResponsePending == FALSE))
	{
		suppressResponse = TRUE; /* flag to NOT send a positive response message */
	}
	else
	{
		suppressResponse = FALSE; /* flag to send the response message */
	}
}


void Service2FHandle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	uint8_t suppressPosRspMsgIndicationBit = FALSE;
	m_NRC = PR;
	if(length >= 4)
	{
		uint16_t PID = (*(MessageData + 1) << 8) | *(MessageData + 2);
		uint8_t control = *(MessageData + 3);
		DIDNode* didNode = SearchDidNode(PID);
		if(didNode == NULL)
		{
			m_NRC = ROOR;//PID not in our PID list
		}
		else if(didNode->didType != IO_DID)
		{
			m_NRC = ROOR;//mabe a IO Control DID
		}
		else
		{
			if(length == 4)
			{
				if(control == 0 || control == 1 || control == 2)
				{
					DiagnosticBuffTX[0] = 0x6F;
					DiagnosticBuffTX[1] = *(MessageData + 1);
					DiagnosticBuffTX[2] = *(MessageData + 2);
					DiagnosticBuffTX[3] = control;
					DiagnosticBuffTX[4] = didNode->Callback(control , 0);
					ResponseLength = 5;
				}
				else
				{
					m_NRC = IMLOIF;
				}
			}
			else if(length == 5)
			{
				if(control == 3)
				{
					uint8_t param = *(MessageData + 4);
					DiagnosticBuffTX[0] = 0x6F;
					DiagnosticBuffTX[1] = *(MessageData + 1);
					DiagnosticBuffTX[2] = *(MessageData + 2);
					DiagnosticBuffTX[3] = control;
					DiagnosticBuffTX[4] = didNode->Callback(control , param);
					ResponseLength = 5;
				}
				else
				{
					m_NRC = IMLOIF;
				}
			}
			else
			{
				m_NRC = IMLOIF;
			}
		}
	}
	else
	{
		m_NRC = IMLOIF;
	}

	if((suppressPosRspMsgIndicationBit) && (m_NRC == 0x00) && (ResponsePending == FALSE))
	{
		suppressResponse = TRUE; /* flag to NOT send a positive response message */
	}
	else
	{
		suppressResponse = FALSE; /* flag to send the response message */
	}
}

void Service31Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	#if 0
	uint8_t ServiceName = *MessageData;
	uint8_t SubFunction =  *(MessageData + 1);
	uint16_t Routine = (*(MessageData + 2) << 8) | *(MessageData + 3);
	if(length == 4)
	{
		if(SubFunction == 1)
		{
			//Printf("routine = %x\r\n",Routine);
			if(Routine == 0xFF00)
			{
				if(0)//if(GlobalDTCControlParam.DataBits.SystemState == SYSTEM_BOOT)
				{
					WaitConfirmBeforeErase = TRUE;
					IsUpdating = TRUE;
					ServiceNegReponse(pTp,ServiceName,RCRRP);
				}
				else
				{
					ServiceNegReponse(pTp,ServiceName,CNC);//not in bootloader 
				}
			}
			else
			{
				ServiceNegReponse(pTp,ServiceName,ROOR);//no more routine except 0xFF00
			}
		}
		else if(SubFunction == 3)
		{
			ServiceNegReponse(pTp,ServiceName,CNC);
		}
		else
		{
			ServiceNegReponse(pTp,ServiceName,SFNS);
		}
	}
	else if( length == 8)
	{
		if(SubFunction == 1)
		{
			if(Routine == 0xFF01)
			{
				uint32_t RxCRC = 0;
				uint32_t AppCRC;
				DiagnosticBuffTX[0] = 0x71;
				DiagnosticBuffTX[1] = 0x01;
				DiagnosticBuffTX[2] = 0xFF;
				DiagnosticBuffTX[3] = 0x01;
				
				#ifdef BOOTLOADER
				AppCRC = CalcAppCanCRC();//CalcAppCRC();
				RxCRC = (*(MessageData + 4) << 24) + (*(MessageData + 5) << 16) + (*(MessageData + 6) << 8) + *(MessageData + 7);
				if(RxCRC == AppCRC)
				{
					WriteCanCRC(RxCRC);//WriteCRC(RxCRC);
					//Printf("app CRC  %.8x = RXCRC %.8x\r\n",AppCRC,RxCRC );
					DiagnosticBuffTX[4] = 0x00;
					Diagnostic_UpdatePromgramTimes();
				}
				else
				{
					//Printf("app CRC  %.8x != RXCRC %.8x\r\n",AppCRC,RxCRC );
					DiagnosticBuffTX[4] = 0x01;
				}
				N_USData_request(pTp,DIAGNOSTIC , N_Sa ,  N_Ta , PHYSICAL , 0 , DiagnosticBuffTX , 5);
				#else
					ServiceNegReponse(pTp,ServiceName,CNC);
				#endif
				//TODO:verify app
			}
			else
			{
				ServiceNegReponse(pTp,ServiceName,ROOR);//no more routine except 0xFF00
			}
		}
		else if(SubFunction == 3)
		{
			ServiceNegReponse(pTp,ServiceName,CNC);
		}
		else
		{
			ServiceNegReponse(pTp,ServiceName,SFNS);
		}
	}
	else
	{
		ServiceNegReponse(pTp,ServiceName,IMLOIF);
	}
	#endif
}

void Service34Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	#if 0
	uint8_t ServiceName = *MessageData;
	uint8_t SubFunction =  *(MessageData + 1);
	uint8_t DataLengthID = *(MessageData + 2);
	if(length == 11)
	{
		if(SubFunction == 0x00)
		{
			if(DataLengthID == 0x44)
			{
				if(0)//if(GlobalDTCControlParam.DataBits.SystemState == SYSTEM_BOOT)
				{
					ProgramAddress = (*(MessageData + 3) << 24 ) + (*(MessageData + 4) << 16)  + (*(MessageData + 5) << 8) + *(MessageData + 6);
					ProgramLength = (*(MessageData + 7) << 24) + (*(MessageData + 8) << 16)  + (*(MessageData + 9) << 8) + *(MessageData + 10);
					//Printf("request program addr:%.8x,len:%.8x\r\n",ProgramAddress,ProgramLength);
					#ifdef BOOTLOADER
					if(ProgramAddress == APP_START_ADDR && ProgramLength < APP_MAX_SIZE)
					{
						WriteAppLength(ProgramLength);
						GlobalDTCControlParam.DataBits.Downloading = TRUE;
						m_BlockIndex = 0;
						
						DiagnosticBuffTX[0] = 0x74;
						DiagnosticBuffTX[1] = 0x20;
						DiagnosticBuffTX[2] =  (MAX_DOWNLOADING_BUF >> 8) & 0xFF;
						DiagnosticBuffTX[3] = MAX_DOWNLOADING_BUF & 0xFF;
						N_USData_request(pTp,DIAGNOSTIC , N_Sa ,  N_Ta , PHYSICAL , 0 , DiagnosticBuffTX , 4);
					}
					else
					{
						ServiceNegReponse(pTp,ServiceName,UDNA);
					}
					#endif
				}
				else
				{
					ServiceNegReponse(pTp,ServiceName,UDNA);//not in bootloader 
				}
			}
			else
			{
				ServiceNegReponse(pTp,ServiceName,ROOR);
			}
		}
		else
		{
			ServiceNegReponse(pTp,ServiceName,SFNS);
		}
	}
	else
	{
		ServiceNegReponse(pTp,ServiceName,IMLOIF);
	}
	#endif
}

void Service35Handle(TP_T* pTp,uint8_t N_TAType,uint16_t length, uint8_t *MessageData)
{
	
}

void Service36Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	#if 0
	uint8_t ServiceName = *MessageData;
	uint8_t BLockIndex = *(MessageData + 1);
	if(length <= MAX_DTCDATA_BUF)
	{
		if(GlobalDTCControlParam.DataBits.Downloading == TRUE || ProgramAddress < 0x800A000)
		{
			//printf("rx block %2x != %2x\r\n",BLockIndex,m_BlockIndex);
			if(BLockIndex == ((m_BlockIndex + 1) & 0xFF))
			{
				m_BlockIndex = BLockIndex;
				if(HighVoltage == TRUE)
				{
					ServiceNegReponse(pTp,ServiceName,VTH);
				}
				else if(LowVoltage == TRUE)
				{
					ServiceNegReponse(pTp,ServiceName,VTL);
				}
				else
				{
					#ifdef BOOTLOADER
					//if(ProgramLength > ProgramLengthComplete)
					//{
						//printf("36 service will program %d bytes , address:%.8x \r\n",length-2,ProgramAddress + ProgramLengthComplete);
						if(Update_WriteData(ProgramAddress + ProgramLengthComplete, MessageData + 2 , length - 2) == 0)
						{
							ProgramLengthComplete +=  length - 2;
							DiagnosticBuffTX[0] = 0x76;
							DiagnosticBuffTX[1] = BLockIndex;
							N_USData_request(pTp,DIAGNOSTIC , N_Sa ,  N_Ta , PHYSICAL , 0 , DiagnosticBuffTX , 2);
						}
						else
						{
							m_BlockIndex = 0;
							ServiceNegReponse(pTp,ServiceName,GPF);//program error
						}
					//}
					//else
					//{
						//m_BlockIndex = 0;
						//ServiceNegReponse(pTp,ServiceName,TDS);//program complete
					//}
					#endif
				}
			}
			else
			{
				ServiceNegReponse(pTp,ServiceName,WBSC);
			}
		}
		else
		{
			ServiceNegReponse(pTp,ServiceName,RSE);//not downloading not accept or address incorrect
		}
	}
	else
	{
		ServiceNegReponse(pTp,ServiceName,IMLOIF);//length error buff will overflow
	}
	#endif
}

void Service37Handle(TP_T* pTp,uint8_t N_TAType, uint16_t length, uint8_t *MessageData)
{
	#if 0
	uint8_t ServiceName = *MessageData;
	if(length == 1)
	{
		if(GlobalDTCControlParam.DataBits.Downloading == TRUE)
		{
			GlobalDTCControlParam.DataBits.Downloading = FALSE;
			if(0)//if(GlobalDTCControlParam.DataBits.SystemState == SYSTEM_BOOT)
			{
				ProgramAddress = 0;
				ProgramLength = 0;
				ProgramLengthComplete = 0;
				
				m_BlockIndex = 0;
				DiagnosticBuffTX[0] = 0x77;
				N_USData_request(pTp,DIAGNOSTIC , N_Sa ,  N_Ta , PHYSICAL , 0 , DiagnosticBuffTX , 1);
			}
			else
			{
				ServiceNegReponse(pTp,ServiceName,CNC);//not in bootloader 
			}
		}
		else
		{
			ServiceNegReponse(pTp,ServiceName,RSE);
		}
	}
	else
	{
		ServiceNegReponse(pTp,ServiceName,IMLOIF);
	}
	#endif
}

void ServiceNegReponse(TP_T* pTp,uint8_t serviceName,uint8_t RejectCode)
{
	DiagnosticBuffTX[0] = 0x7F;
	DiagnosticBuffTX[1] = serviceName;
	DiagnosticBuffTX[2] = RejectCode;
	N_USData_request(pTp,DIAGNOSTIC , N_Sa ,  N_Ta , PHYSICAL , 0 , DiagnosticBuffTX , 3);
}

void Diagnostic_ServiceHandle(TP_T* pTp,uint8_t N_SA , uint8_t N_TA , uint8_t N_TAtype , uint16_t length , uint8_t *MessageData)
{
	bool ValidSid;
	uint16_t ServiceIndex;
	ValidSid = FALSE;
	ServiceIndex = 0;
	CurrentService = MessageData[0];	
	//查找服务
	while((ServiceIndex < SERVICE_NUMBER) && (!ValidSid))
	{
		if(ServiceList[ServiceIndex].serviceName == CurrentService)
		{
			if(ServiceList[ServiceIndex].support == TRUE)
			{
				ValidSid = TRUE;
			}
			else//found service but service not enable by application layer
			{
				ValidSid = FALSE;
				break;
			}
		}
		else
		{
			ServiceIndex++;
		}
	}
	if(ValidSid == TRUE)
	{
		//物理寻址
		if(N_TAtype == PHYSICAL)
		{
			suppressResponse = FALSE;
			//默认会话
			if(ECU_DEFAULT_SESSION == m_CurrSessionType)
			{
				//不支持的安全等级
				if(ServiceList[ServiceIndex].PHYDefaultSession_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].PHYDefaultSession_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(pTp,N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			//扩展会话
			else if(ECU_EXTENED_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].PHYExtendedSession_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].PHYExtendedSession_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(pTp,N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			//编程会话
			else if(ECU_PAOGRAM_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].PHYProgramSeesion_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].PHYProgramSeesion_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(pTp,N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			//供应商会话，用于下线配置
			else if(ECU_FACTORY_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].serviceName== SESSION_CONTROL
					|| ServiceList[ServiceIndex].serviceName== SECURITY_ACCESS
					|| ServiceList[ServiceIndex].serviceName== READ_DATA_BY_ID
					|| ServiceList[ServiceIndex].serviceName== WRITE_DATA_BY_ID
					|| ServiceList[ServiceIndex].serviceName== RESET_ECU)
				{
					ServiceList[ServiceIndex].serviceHandle(pTp,N_TAtype,length,MessageData);
				}
				else
				{
					m_NRC = SNSIAS;
				}
			}
		}
		//功能寻址
		else if(N_TAtype == FUNCTIONAL)
		{
			if(ECU_DEFAULT_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].FUNDefaultSession_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].FUNDefaultSession_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(pTp,N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			else if(ECU_EXTENED_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].FUNExtendedSession_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].FUNExtendedSession_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(pTp,N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			else if(ECU_PAOGRAM_SESSION == m_CurrSessionType)
			{
				if(ServiceList[ServiceIndex].FUNProgramSeesion_Security == LEVEL_UNSUPPORT)
				{
					m_NRC = SNSIAS;
				}
				else
				{
					if((ServiceList[ServiceIndex].FUNProgramSeesion_Security & m_SecurityLevel) == m_SecurityLevel)
					{
						ServiceList[ServiceIndex].serviceHandle(pTp,N_TAtype,length,MessageData);
					}
					else
					{
						m_NRC = SAD;
					}
				}
			}
			else if(ECU_FACTORY_SESSION == m_CurrSessionType)
			{
				m_NRC = SNS;
			}
		}
	}
	else
	{
		//无效服务
		if(N_TAtype == PHYSICAL)//功能寻址无效SED不响应
		{
			m_NRC = SNS;
			suppressResponse = FALSE;
		}
		else
		{
			suppressResponse = TRUE;
		}
	}
}


void Diagnostic_MainProc(TP_T* pTp)
{
	uint32_t rxId = 0;
	if(!IsIndicationListEmpty(pTp))
	{
		NetworkNotification temp = PullIndication(pTp);
		rxId = ((temp.N_SA << 8) + temp.N_TA);
		
		if(temp.NotificationType == INDICATION)
		{
			uint8_t RequestEquipment = 0xFF;
			if((rxId & 0xFFFF) == (TesterPhyID & 0xFFFF) || (rxId & 0xFFFF) == (TesterFunID & 0xFFFF))
			{
				RequestEquipment = 0;
				N_Ta = (uint8_t)EcuID;
				N_Sa = (uint8_t)(EcuID >> 8);
			}
	
			if(RequestEquipment == 0)
			{
				if(temp.N_Resut == N_OK ||temp.N_Resut == N_UNEXP_PDU)
				{
					Diagnostic_ServiceHandle(pTp,temp.N_SA,temp.N_TA,temp.N_TAtype,temp.length, temp.MessageData);
					if((temp.N_TAtype == FUNCTIONAL) && ((m_NRC == SNS) || (m_NRC == SFNS) || (m_NRC == SFNSIAS) || (m_NRC == ROOR)) && (ResponsePending == FALSE))
					{
						//printf("res supress,pending =  %d\r\n",ResponsePending);
						/* suppress negative response message */
					}
					else if (suppressResponse == TRUE)
					{
						//printf("res supress bit is TRUE\r\n");
						/* suppress positive response message */
					}
					else
					{
						if(m_NRC == PR)
						{
							N_USData_request(pTp,DIAGNOSTIC , N_Sa ,  N_Ta , PHYSICAL , 0 , DiagnosticBuffTX , ResponseLength);
						}
						else
						{
							ServiceNegReponse(pTp,CurrentService,m_NRC);
						}
					}
					if(m_CurrSessionType  != ECU_DEFAULT_SESSION)
					{
						DiagTimer_Set(&S3serverTimer, 5000);
					}
				}
			}
		}
		else if(temp.NotificationType == CONFIRM)
		{
			if((rxId & 0xFFFF) == (EcuID & 0xFFFF))
			{
				if(WaitConfirmBeforeJump == TRUE)
				{
					
				}
				else if(WaitConfirmBeforeErase == TRUE)
				{
					WaitConfirmBeforeErase = FALSE;
					#ifdef BOOTLOADER
					//Printf("31 service will erase app\r\n");
					//Update_EraseFlash();
					DiagnosticBuffTX[0] = 0x71;
					DiagnosticBuffTX[1] = 0x01;
					DiagnosticBuffTX[2] = 0xFF;
					DiagnosticBuffTX[3] = 0x00;
					DiagnosticBuffTX[4] = 0x00;
					N_USData_request(pTp,DIAGNOSTIC , N_Sa ,  N_Ta , PHYSICAL , 0 , DiagnosticBuffTX , 5);
					#endif
				}
				else if(WaitConfimBeforeReset == TRUE)
				{
					WaitConfimBeforeReset = FALSE;
					if(ResetCallBackFun != NULL)
					{
						ResetCallBackFun(m_EcuResetType);
					}
				}
			}
		}
		else if(temp.NotificationType == FF_INDICATION)
		{
			//Printf("RX FF ind\r\n");
		}
	}
}

void Diagnostic_TimeProc(void)
{
	if(m_CurrSessionType != ECU_DEFAULT_SESSION)
	{
		if(DiagTimer_HasExpired(&S3serverTimer))
		{
			//printk("[time out]S3 server timeout\r\n");
			GotoSession(ECU_DEFAULT_SESSION);
		}
	}
	if(m_UnlockStep == WAIT_DELAY)
	{
		uint8_t i;
		for(i = 0 ; i < 3 ; i ++)
		{
			if(UnlockList[i].valid == TRUE)
			{
				if(DiagTimer_HasExpired(&UnlockList[i].SecurityLockTimer))
				{
					UnlockList[i].FaultCounter--;
					EE_ReadVariable(UnlockList[i].FaultCounterAddr, 1 ,&UnlockList[i].FaultCounter);
					m_UnlockStep = WAIT_SEED_REQ;
				}
			}
		}
	}
	if(ResponsePending == TRUE && WaitConfirmBeforeJump == FALSE)
	{
		ResponsePending = FALSE;
		Service10PosResponse((SessionType)m_CurrSessionType);
	}
}

/*
功能：保存快照数据信息
参数：EEPromAddr  需保存的地址
*/
void SaveSnapShotData(uint16_t EEPromAddr)
{
	uint8_t length = 0;
	uint8_t i;
	for(i = 0 ; i < SnapShotAdded; i++)
	{
		memcpy(DiagnosticBuffTX + length,SnapShots[i].dataPointer,SnapShots[i].dataLength);
		length += SnapShots[i].dataLength;
	}
	EE_WriteVariable(EEPromAddr , length , DiagnosticBuffTX);
}

void DtcHandle(DTCNode* DtcNode)
{
	uint8_t CurrentResult;
	if(DtcNode!= NULL && DtcNode->DetectFunction != NULL)
	{
		CurrentResult = DtcNode->DetectFunction();
		if(CurrentResult == PASSED || CurrentResult == FAILEDD)
		{
			DtcNode->DTCStatus.DTCbit.TestNotCompleteThisMonitoringCycle = 0;	//bit7:  本检测周期测试未完成
			DtcNode->DTCStatus.DTCbit.TestNotCompleteSinceLastClear = 0;			//bit5:  上次清零后测试未完成
		}
		//测试通过
		if(CurrentResult == PASSED)
		{
			DtcNode->DTCStatus.DTCbit.TestFailed = 0;													//bit1:  测试失败
		}
		//测试失败
		else if(CurrentResult == FAILEDD)
		{
			DtcNode->DTCStatus.DTCbit.TestFailed = 1;												
			if(DtcNode->DTCStatus.DTCbit.TestFailedThisMonitoringCycle == 0)	//bit2:		本次操作循环测试失败
			{
				DtcNode->DTCStatus.DTCbit.TestFailedThisMonitoringCycle = 1;			
				DtcNode->TripCounter++;
				if(DtcNode->TripCounter >= DtcNode->TripLimitTimes)									
				{
					if(DtcNode->DTCStatus.DTCbit.ConfirmedDTC != 1)								//bit4:		确认DTC
					{
						DtcNode->TripCounter = 0;
						DtcNode->DTCStatus.DTCbit.ConfirmedDTC = 1;
						SaveSnapShotData(DtcNode->SnapShotEEpromAddr);
					}
				}
				if(DtcNode->FaultOccurrences < 0xff)
				{
					DtcNode->FaultOccurrences++;
				}
			}
			DtcNode->DTCStatus.DTCbit.PendingDTC = 1;													//bit3:    未确认的诊断故障码
			DtcNode->DTCStatus.DTCbit.TestFailedSinceLastClear = 1;						//bit6:    自上次清除后测试失败
			DtcNode->OldCounter = 0;
		}
		if(DtcNode->DTCStatus.DTCbit.TestFailedThisMonitoringCycle == 0 && DtcNode->DTCStatus.DTCbit.TestNotCompleteThisMonitoringCycle == 0)//iso14229-1 Frigure D.4
		{
			DtcNode->TripCounter = 0;
		}
	}
}

void Diagnostic_DTCProc(void)
{
	if(EnableDTCDetect != FALSE && osKernelGetTickCount() >= 1000)
	{
		if( osKernelGetTickCount() - DtcTimer >= 50)
		{
			uint8_t i;
			for(i = 0 ; i < DTCAdded ; i++)
			{
				DtcHandle(DTCS + i);
			}
			DtcTimer = osKernelGetTickCount();
		}
	}
}

void Diagnostic_Proc(TP_T* pTp)
{
	NetworkLayer_Proc(pTp);
	Diagnostic_MainProc(pTp);
	Diagnostic_TimeProc();
	Diagnostic_DTCProc();
}

void Diagnostic_RxFrame(TP_T* pTp,uint32_t ID,uint8_t* data,uint8_t IDE,uint8_t DLC,uint8_t RTR)
{
	//UDS诊断数据接收
	NetworkLayer_RxFrame(pTp,ID,data,IDE,DLC,RTR);
}

void Diagnostic_DelInit(void)
{
	Diagnostic_SaveAllDTC();
}


void Diagnostic_ClearDTC(uint32_t Group)
{
	uint8_t i;
	for(i = 0 ; i < DTCAdded ; i++)
	{
		if((DTCS[i].DTCCode & Group) == DTCS[i].DTCCode)
		{
			DTCS[i].DTCStatus.DTCStatusByte = 0x50;//when clear bit4 and bit6 must be setted to 1
			DTCS[i].FaultOccurrences = 0;
			DTCS[i].OldCounter = 0;
			DTCS[i].GoneCounter = 0;
			DTCS[i].TripCounter = 0;
		}
	}
	Diagnostic_SaveAllDTC();
}

void Diagnostic_SaveAllDTC(void)
{
	uint8_t i;
	for(i = 0 ; i < DTCAdded ; i++)
	{															//上次清零后测试未完成																						//本检测周期测试失败	
		if(DTCS[i].DTCStatus.DTCbit.TestNotCompleteSinceLastClear == 0 && DTCS[i].DTCStatus.DTCbit.TestFailedThisMonitoringCycle == 0)
		{
			DTCS[i].DTCStatus.DTCbit.PendingDTC = 0;
		}														//本检测周期测试失败																							//确认DTC
		if(DTCS[i].DTCStatus.DTCbit.TestFailedThisMonitoringCycle == 0  && DTCS[i].DTCStatus.DTCbit.ConfirmedDTC == 1)
		{	//已老去计数器，转老化完成
			if(DTCS[i].OldCounter >= 40)//故障码老化机制
			{
				DTCS[i].DTCStatus.DTCStatusByte = 0x50;//when clear bit4 and bit6 must be setted to 1 and others 0
				DTCS[i].OldCounter = 0;
				DTCS[i].GoneCounter++;									//老化计数器，累计老化完成计数
				DTCS[i].FaultOccurrences = 0;
				DTCS[i].TripCounter = 0;
			}
			else
			{
				DTCS[i].OldCounter++;
			}
		}
		else
		{
			DTCS[i].OldCounter = 0;
		}
		DTCS[i].DTCStatus.DTCStatusByte &= 0xBD;//bit1 and bit6 can not be saved
		EE_WriteVariable(DTCS[i].EEpromAddr, DTC_BYTE_NUMBER_TO_SAVE , &(DTCS[i].DTCStatus));//&(DTCS[i].DTCStatus.DTCStatusByte));
	}
}

void Diagnostic_LoadAllDTC(void)
{
	uint8_t i;
	for(i = 0 ; i < DTCAdded ; i++)
	{
		EE_ReadVariable(DTCS[i].EEpromAddr, DTC_BYTE_NUMBER_TO_SAVE , &(DTCS[i].DTCStatus));
		DTCS[i].DTCStatus.DTCStatusByte |= 0x40;//when IGN ON, bit6 default 1,
		DTCS[i].DTCStatus.DTCStatusByte &= 0xFD;//when IGN ON, bit1 default 0
	}
}

void Diagnostic_LoadSecuriyFaultCounter(void)
{
	uint8_t i;
	for(i = 0 ; i < 3 ; i++)
	{
		if(UnlockList[i].valid != FALSE)
		{
			EE_ReadVariable(UnlockList[i].FaultCounterAddr, 1 ,&UnlockList[i].FaultCounter);
			if(UnlockList[i].FaultCounter == 0xFF)
			{
				UnlockList[i].FaultCounter = 0;
			}
			else if(UnlockList[i].FaultCounter >= UnlockList[i].FaultLimitCounter)
			{
				UnlockList[i].FaultCounter = 0;
				m_UnlockStep = WAIT_DELAY;
				DiagTimer_Set(&UnlockList[i].SecurityLockTimer , UnlockList[i].UnlockFailedDelayTime);
			}
			EE_WriteVariable(UnlockList[i].FaultCounterAddr, 1 ,&UnlockList[i].FaultCounter);
		}
	}
}

void Diagnostic_BindingSnapshot(void)
{
	uint8_t i;
	static uint16_t SnapshotDataLength = 0;
	if(SnapshotDataLength == 0)
	{
		for(i = 0 ;  i < SnapShotAdded ; i ++)
		{
			SnapshotDataLength += SnapShots[i].dataLength;
		}
		for(i = 0 ;  i < MAX_DTC_NUMBER ; i ++)
		{
			DTCS[i].SnapShotEEpromAddr = ModuleEEpromStartAddr + EEpromUsed + SnapshotDataLength * i;
		}
	}
}

void Diagnostic_LoadAllData(void)
{
	Diagnostic_LoadAllDTC();
	Diagnostic_LoadSecuriyFaultCounter();
	Diagnostic_BindingSnapshot();
}
