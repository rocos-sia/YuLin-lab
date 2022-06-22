 /*
 *  file : xnetdriver.h
 *  note : xnet driver interface
 */

#ifndef _H_XNETDRIVER
#define _H_XNETDRIVER

#include <memory.h>

/* NULL pointer */
#ifndef NULL
#ifdef __cplusplus
#define NULL    0
#else
#define NULL    ((void *)0)
#endif
#endif

#ifdef _WINDOWS
/* exact-width signed integer types */
typedef   signed char 				int8_t;
typedef   signed short				int16_t;
typedef   signed int					int32_t;
typedef   long  long					int64_t;

/* exact-width unsigned integer types */
typedef unsigned char				uint8_t;
typedef unsigned short				uint16_t;
typedef unsigned int					uint32_t;
typedef  unsigned long long		uint64_t;

#else
#include <stdint.h>
#endif


#ifdef _WINDOWS
#ifdef _DLL_XNET_DRIVER
#define XNET_DRIVER_EXPORT __declspec(dllexport)
#else
#define XNET_DRIVER_EXPORT __declspec(dllimport)
#endif
#else
#define XNET_DRIVER_EXPORT
#include <time.h>
#endif

#ifdef __cplusplus
namespace xNETDRIVER
{
#endif

	// UDP端口ID定义
#define XNET_UDP_PORT_ID_BEGIN		0x10
#define XNET_UDP_PORT_ID_END			0x1F

#define XNET_TCP_PORT_ID_BEGIN			0x20
#define XNET_TCP_PORT_ID_END			0x2F

#define XNET_MAC_PORT_ID_BEGIN		0x40
#define XNET_MAC_PORT_ID_END			0x47

#define XNET_USB_PORT_ID_BEGIN		0x50
#define XNET_USB_PORT_ID_END			0x5F

#define XNET_PACKET_DATA_LEN 512

	// 链路定义
	enum LOCAL_PORT_TYPE
	{
		LOCAL_PORT_NULL=0,
		LOCAL_PORT_UDP=0x0001,
		LOCAL_PORT_TCP=0x0002,
		LOCAL_PORT_MAC=0x0004,
		LOCAL_PORT_USB=0x0008
	};

	// 端口地址
	class PORT_ADDRESS
	{
	public:
		PORT_ADDRESS()
		{
			memset(data,0,8);
		}
		PORT_ADDRESS(const PORT_ADDRESS &addr)
		{
			memcpy(data, addr.data, 8);
		}
		PORT_ADDRESS &operator=(const PORT_ADDRESS &addr)
		{
			memcpy(data, addr.data, 8);
			return *this;
		}
		PORT_ADDRESS &operator=(const int addr)
		{
			memcpy(data, &addr, 4);
			return *this;
		}

		bool operator==(const PORT_ADDRESS &addr)
		{
			return 0==memcmp(data, addr.data,8);
		}

		bool IsSelf() const 
		{
			uint8_t	temp[8];
			memset(temp,0xFF,8);
			
			return 0==memcmp(data, temp,8);
		}

		void SetSelf() { memset(data, 0xFF, 8); }

		uint32_t GetIP(){	return *(uint32_t *)data; }
		void SetIP(uint32_t addr){ *(uint32_t *)data=addr; }

		uint8_t	data[8];
	};

	// 路径单元
	class LINK_UNIT
	{
	public:
		LINK_UNIT(){}
		LINK_UNIT(uint8_t src_link, PORT_ADDRESS dst_addr ) : bySrcLinkID(src_link), DstAddr(dst_addr){}
		LINK_UNIT & operator=(const LINK_UNIT &unit)
		{
			bySrcLinkID=unit.bySrcLinkID;
			byDstLinkID=unit.byDstLinkID;
			DstAddr=unit.DstAddr;
			return *this;
		}
		uint8_t			bySrcLinkID;
		uint8_t			byDstLinkID;
		PORT_ADDRESS			DstAddr;
	};

	class LINK_PATH				// 设备间通信的路径
	{
	public:
		LINK_PATH()
		{
			size=0;
			bRedun=0;
			memset(data, 0, sizeof(LINK_UNIT)*16);
		}

		LINK_PATH & operator=(const LINK_PATH &path)
		{
			size=path.size;
			bRedun=path.bRedun;
			memcpy(data, path.data, sizeof(LINK_UNIT)*path.size);
			return *this;
		}
		LINK_PATH & operator+=(const LINK_PATH &path)
		{
			memcpy(data+size, path.data, sizeof(LINK_UNIT)*path.size);
			size+=path.size;
			return *this;
		}
		LINK_PATH & operator+=(const LINK_UNIT &unit)
		{
			data[size]=unit;
			size++;
			return *this;
		}

		bool operator==(const LINK_PATH &path)
		{
			return ( size==path.size && bRedun == path.bRedun && 0==memcmp(data, path.data, sizeof(LINK_UNIT)*path.size) );
		}
		uint8_t			size;
		bool			bRedun;//是否冗余
		LINK_UNIT		data[16];
		
	};

	// 计算机连接设备的通信参数
	class  LINK_PARAM  
	{
	public:
		LINK_PARAM()
		{
			LocalPort=LOCAL_PORT_NULL;
			srcObjID=-1;
			dstObjID=-1;
			bySrcLinkID=0;
			byDstLinkID=0;
			SrcPort = 0;
			DstPort = 0;
		}
		LINK_PARAM(LOCAL_PORT_TYPE type, uint8_t src_link_id, uint8_t dst_link_id, PORT_ADDRESS src, PORT_ADDRESS dst,uint8_t src_id, uint8_t dst_id, const LINK_PATH &path)
		{
			LocalPort=type;
			bySrcLinkID=src_link_id;
			byDstLinkID=dst_link_id;
			SrcAddr=src;
			DstAddr=dst;
			SubPath=path;
			srcObjID=src_id;
			dstObjID=dst_id;
			SrcPort = 0;
			DstPort = 0;
		}
		bool equal(const LINK_PARAM &right)
		{
			return (LocalPort==right.LocalPort 
				&& byDstLinkID==right.byDstLinkID 
				&& bySrcLinkID==right.bySrcLinkID 
				&& SrcAddr==right.SrcAddr 
				&& DstAddr==right.DstAddr 
				&& SrcPort == right.SrcPort
				&& DstPort == right.DstPort
				&& SubPath==right.SubPath 
				&& srcObjID==right.srcObjID 
				&& dstObjID==right.dstObjID);
		}

		LOCAL_PORT_TYPE	LocalPort;				// 本地端口类型
		PORT_ADDRESS		SrcAddr;				// 源地址（当端口类型是UDP、TCP，可以不用设置源地址）
		uint8_t					bySrcLinkID;			// 源端口ID
		uint8_t					byDstLinkID;			// 目标端口ID
		PORT_ADDRESS		DstAddr;				// 目的地址

		uint16_t					SrcPort;
		uint16_t					DstPort;

		LINK_PATH			SubPath;				// 后续路径

		uint8_t					srcObjID;				// 源对象ID
		uint8_t					dstObjID;				// 目的对象ID
	};


	// 将路径装换成网络层路径
	typedef struct
	{
		uint8_t  byVersion;   
		uint8_t  byLinkPrio;   //0x80: 常规报文，0x00:快速报文
		uint8_t  byLinkType;   //格式: yxxx0000: y代表设备冗余，1b：该报文最终发给冗余设备 0b：该报文最终发给单个设备；xxx代表类型,000:1字节帧; 001:2字节帧; 002:6字节帧
		uint8_t  byLinkJump;   //链接总跳转数
		uint8_t  byLinkID[16];
		uint8_t  byLinkAddr[32];
	}xNET_LINK;

	// 管理服务服务码，命令码定义
	typedef unsigned int SERVICE;
	const SERVICE		DATA_MANAGE=0x00;			// 数据管理服务
	const SERVICE		FILE_MANAGE=0x01;				// 文件管理服务
	const SERVICE		DEVICE_OPERATE=0x02;			// 设备操作服务
	const SERVICE		DEVICE_MANAGE=0x03;			// 设备管理服务
	
	typedef unsigned int COMMAND;
		// 订阅管理服务命令码
	const COMMAND	CREATE_SUBSCRIBE=0x00;		// 创建订阅
	const COMMAND	DELETE_SUBSCRIBE=0x01;
	const COMMAND	UPDATE_SUBSCRIBE=0x02;
	const COMMAND	READ_DATA=0x03;
	const COMMAND	WRITE_DATA=0x04;


		// 文件管理服务命令码
	const COMMAND	CREATE_FILE=0x00;
	const COMMAND	DELETE_FILE=0x01;
	const COMMAND	CHECK_FILE=0x02;
	const COMMAND	READ_FILE=0x03;
	const COMMAND	WRITE_FILE=0x04;
	const COMMAND	ENUM_FILE=0x05;
	const COMMAND	READ_FILE_LENGTH=0x06;

		// 设备操作命令码
	const COMMAND	START_DOWNLOAD_CFG=0x00;			// 启动组态下载
	const COMMAND	ABORT_DOWNLOAD_CFG=0x01;			// 放弃组态下载
	const COMMAND	COMPLETE_DOWNLOAD_CFG=0x02;			// 完成组态下载
	const COMMAND	READ_CFG_ID=0x03;					// 读取组态特征码
	const COMMAND	START_FB_DOWNLOAD=0x04;				//启动功能块库下载
	const COMMAND	QUIT_FB_DOWNLOAD=0x05;				//放弃功能块库下载
	const COMMAND	COMPLETE_FB_DOWNLOAD=0x06;				//完成功能块库下载
	const COMMAND	READ_FB_UUID=0x07;				//读取功能块库特征码
	const COMMAND	START_FIRM=0x08;					//启动固件下载
	const COMMAND	QUIT_FIRM=0x09;						//放弃固件下载
	const COMMAND	COMPLETE_FIRM=0x0A;					//完成固件下载
	const COMMAND	RESTART_DEVICE=0x0B;				//重启设备
	const COMMAND	REDU_SWITCH=0x0C;					//冗余切换
	const COMMAND	CLEAR_CFG=0x0D;						//清除组态
	const COMMAND	READ_CFG_INFO = 0x0E;				//读取组态信息
	const COMMAND	START_UPLOAD_CFG=0x0F;				//开始上载组态
	const COMMAND	ABORT_UPLOAD_CFG=0x10;				//放弃上载组态
	const COMMAND	COMPLETE_UPLOAD_CFG = 0x11;			//结束上载组态

		// 设备管理命令码
	const COMMAND	READ_DEVICE_ID=0x00;						// 读取设备ID
	const COMMAND	READ_DEVICE_STATUS=0x01;				// 读取设备状态
	const COMMAND	ENUM_DEVICE_LINK_ID=0x02	;			// 枚举链接ID
	const COMMAND	QUERY_ONLINE_NODE_COUNT=0x03;	// 查询在线节点数量
	const COMMAND	QUERY_ONLINE_NODE=0x04;				// 查询在线节点
	const COMMAND	READ_HARD_VERSION=0x05;			// 读取设硬件版本
	const COMMAND	WRITE_HARD_VERSION=0x06;			// 设置硬件版本
	const COMMAND	READ_DEVICE_CODE=0x07;					// 读取设备编码
	const COMMAND	WRITE_DEVICE_CODE=0x08;				// 设置设备编码
	const COMMAND	READ_DEVICE_MANU_INFO=0x09;		// 读取设备制造信息
	const COMMAND	WRITE_DEVICE_MANU_INFO=0x0A;		// 设置设备制造信息
	const COMMAND	READ_LINK_PARAM=0x0B;					// 读取设备端口参数
	const COMMAND	WRITE_LINK_PARAM=0x0C;					// 读取设备端口参数
	const COMMAND	READ_SOFT_VERSION=0x0D;					//读取软件版本
	const COMMAND	READ_FIRM_INFO=0x0E;					//读取固件信息
	const COMMAND	READ_PORT_MODE=0x0F;							//读取端口工作模式
	const COMMAND	SET_PORT_MODE=0x10;							//设置端口工作模式
	const COMMAND	READ_IO_MODE=0x11;						//读取IO工作模式
	const COMMAND	SET_IO_MODE=0x12;						//设置IO工作模式
	const COMMAND	READ_IO_SIGNAL_TYPE=0x13;				//读取IO标定信号类型
	const COMMAND	SET_IO_SIGNAL_TYPE=0x14;				//设置IO标定信号类型
	const COMMAND	READ_DEVICE_TIME=0x15;					//读取当前时间
	const COMMAND	SET_DEVICE_TIME=0x16;					//设置当前时间
	const COMMAND	OPEN_PORT=0x17;							//打开端口
	const COMMAND	CLOSE_PORT=0x18;						//关闭端口
	const COMMAND	LED_TEST=0x19;							//LED指示灯测试
	const COMMAND	READ_NTP_ADDRESS=0X1A;					//读取NTP服务器地址
	const COMMAND	WRITE_NTP_ADDRESS=0X1B;					//设置NTP服务器地址



	// 管理服务响应码
	typedef unsigned int RESPONSE;

		//全局定义
	const RESPONSE	XNET_SUCCESS=0x00;							// 成功响应

	const RESPONSE	XNET_UNKNOWN_ERROR=0x01;					// 未知错误
	const RESPONSE	XNET_SERVICE_NOT_SUPPORTED=0x02;			// 服务码不支持
	const RESPONSE	XNET_CMD_NOT_SUPPORTED=0x03;				// 命令码不支持
		//数据管理服务
	const RESPONSE	XNET_SUBSCRIBE_NUMBER_OVER=0x10;				//订阅数已达上限		
	const RESPONSE	XNET_DATA_OBJECT_ID_ILLEGAL=0x11;			//数据对象ID非法
	const RESPONSE	XNET_OFFSET_OR_LENGTH_ILLEGAL=0x12;			//偏移、长度非法
	const RESPONSE	XNET_SUBSCRIBE_MISS=0x13;					//订阅号不存在
	const RESPONSE	XNET_DATA_NOT_READ=0x14;					//指定数据不支持读取
	const RESPONSE	XNET_DATA_NOT_WRITE=0x15;					//指定数据不支持写入
	const RESPONSE	XNET_SUBSCRIBE_VERSION_MISSMATCH=0x16;		//订阅版本不匹配
		//文件管理服务
	const RESPONSE	XNET_FILE_NUMBER_OVER=0x10;					//文件数已达上限
	const RESPONSE	XNET_FILE_ID_ILLEGAL=0x11;					//文件ID非法
		
	const RESPONSE	XNET_FILE_NOT_OPEN=0x13;					//文件未打开
	const RESPONSE	XNET_FILE_NOT_READ=0x14;					//文件不支持读取
	const RESPONSE	XNET_FILE_NOT_WRITE=0x15;					//文件不支持写入
		//设备操作服务
	const RESPONSE	XNET_CONFIG_DOING=0x10;						//正在进行组态操作，无法进行组态下载
	const RESPONSE	XNET_FIRMWARE_DOING=0x11;					//正在进行固件操作，无法进行固件下载
		//设备管理服务
	const RESPONSE	XNET_LINK_ID_ILLEGAL=0x10;					//链接ID非法
	const RESPONSE	XNET_HARDWARE_INVALID=0x11;					//硬件版本无效
	const RESPONSE	XNET_SOFTWARE_INVALID=0x12;					//软件版本无效
	const RESPONSE	XNET_DEVICE_CODE_INVALID=0x13;				//设备编码无效
		// 以下响应码不在协议中定义，为防止与协议重复，因此 值>0xFF
	const RESPONSE	XNET_SEND_FAILED=0x0100;					// 发送失败
	const RESPONSE	XNET_RECV_TIMEOUT=0x101;					// 接收超时
	const RESPONSE	XNET_RECV_PACKET_ERROR=0x102;			// 数据包接收超时


	// 管理服务数据包类型
	enum	PACKET_TYPE
	{
		PACKET_TYPE_REQUEST,
		PACKET_TYPE_RESPONSE
	};

#define DEFINE_STRUCT_AND_INIT(_struct, _var) _struct _var; memset(&_var, 0, sizeof(_struct));

	typedef struct  
	{
		PACKET_TYPE		PacketType;				// 数据包类型
		SERVICE			Service;						// 服务码
		COMMAND		Cmd;							// 命令码
		RESPONSE			Response;					// 应答码
		uint16_t				wSeq;						// 数据包序号
		uint16_t				wLen;
		uint8_t				byData[1024];			// 数据包最大1024字节
	}MANAGE_SERVICE_DATA;
	
	
	// 订阅服务
	typedef struct 
	{
		uint16_t		wLowSubsID;
		uint8_t		byRsv[2];
		uint32_t		dwOffset; 
		uint16_t		wLen;
		uint8_t		byMode;//0 周期 1变化 2 周期且变化
		uint8_t		byCount;
		uint16_t		wCycle;
		uint16_t		wUUIDCycle;
		uint8_t		bySubsUUID[16];
	}CREATE_SUBSCRIBE_SEND_FMT;

	typedef struct 
	{
		uint32_t		dwSubsID;
	}CREATE_SUBSCRIBE_RECV_FMT,DELETE_SUBSCRIBE_SEND_FMT;

	typedef struct  
	{
		uint32_t		dwSubsID;
		uint8_t		bySubsUUID[16];
		uint8_t		byCount;
		uint8_t		byRsv[3];
	}UPDATE_SUBSCRIBE_SEND_FMT;

	typedef struct  
	{
		uint8_t		byFlag;			// 0：全体下载；1：增量下载
		uint8_t		byRsv[3];
	}START_DOWNLAD_CFG_SEND_FMT;

	typedef struct 
	{
		uint8_t		byFlag;
		uint8_t		byRsv[3];
		uint8_t		byCfgUUID[16];
		uint32_t	dwCfgVersion;
		uint8_t		byDownArchive;
		uint8_t		byRsv2[3];
		uint8_t		byDownTime[32];
		uint8_t		byDownDesc[256];
	}COMPLETE_DOWNLOAD_CFG_SEND_FMT;

	typedef struct  
	{
		uint16_t		wDeviceID;
		uint8_t		byRsv[2];
		uint8_t		byCfgUUID[16];
	}READ_CFG_ID_RECV_FMT;

	typedef struct  
	{
		uint8_t		byFlag;			// 0：全体下载；1：增量下载
		uint8_t		byRsv[3];
		uint8_t		byCfgUUID[16];
		uint32_t	dwCfgVersion;
		uint8_t		byIsDownloadFile;// 0：未下载；1：已下载
		uint8_t		byRsv2[3];
		uint8_t		byDownloadTime[32];
		uint8_t		byDownloadDesc[256];
	}READ_CFG_INFO_RECV_FMT;

	typedef struct
	{
		uint16_t		wFileID;
		uint8_t		byFileType;
		uint8_t		byRsv;
		uint32_t		dwFileLen;
	}CREATE_FILE_SEND_FMT;


	typedef struct
	{
		uint16_t		wFileID;
		//uint8_t		byFileType;
		uint8_t		byRsv;
	}DELETE_FILE_SEND_FMT,CHECK_FILE_SEND_FMT;

	typedef struct
	{
		uint16_t		wFileID;
		uint8_t		byRsv[2];
		uint32_t		dwOffset;
		uint16_t		wLen;
		uint8_t		byRsv2[2];
	}READ_FILE_SEND_FMT;

	typedef struct
	{
		uint16_t		wFileID;
		uint8_t			byRsv[2];
	}READ_LENGTH_FILE_SEND_FMT,ENUM_FILE_SEND_FMT;

	typedef struct
	{
		uint16_t		wFileID;
		uint32_t		wLen;
	}READ_LENGTH_FILE_RECV_FMT;

	typedef struct
	{
		uint16_t		wFileID;
		uint8_t		byRsv[2];
		uint32_t		dwOffset;
		uint16_t		wLen;
		uint8_t		byRsv2[2];
		uint8_t		byData[XNET_PACKET_DATA_LEN];
	}READ_FILE_RECV_FMT, WRITE_FILE_SEND_FMT;

	typedef struct
	{
		uint16_t		wBaseFileID;
		uint16_t		wFileCount;
		uint16_t		wFileID[256];
	}ENUM_FILE_RECV_FMT;

	typedef struct
	{
		uint16_t		wFileID;
		uint16_t		wCheck;
	}CHECK_FILE_RECV_FMT;

	typedef struct  
	{
		uint32_t		dwOffset;
		uint16_t		wLen;
		uint8_t		byRsv[2];
	}READ_DATA_SEND_FMT;

	typedef struct  
	{
		uint32_t		dwOffset;
		uint16_t		wLen;
		uint8_t		byRsv[2];
		uint8_t		byData[512];
	}READ_DATA_RECV_FMT;

	typedef READ_DATA_RECV_FMT WRITE_DATA_SEND_FMT;




	typedef struct  
	{
		uint16_t		wDeviceID;
		uint8_t		byRsv[2];
	}READ_DEVICE_ID_RECV_FMT;

	typedef struct  
	{
		uint16_t		wDeviceStatus;
		uint8_t		byRsv[2];
	}READ_DEVICE_STATUS_RECV_FMT;

	typedef struct  
	{
		uint32_t		dwHdwVer;
		uint32_t		dwSoftVer;
	}READ_DEVICE_VERSION_RECV_FMT;

	typedef READ_DEVICE_VERSION_RECV_FMT WRITE_DEVICE_VERSION_SEND_FMT;

	typedef struct  
	{
	//	uint32_t		dwHdwVer;
		uint8_t		dwHdwVer[4];
	}READ_HARDWARE_RECV_FMT;

	typedef READ_HARDWARE_RECV_FMT WRITE_HARDWARE_SEND_FMT;

	typedef struct  
	{
		//uint32_t		dwSoftVer;
		uint8_t		dwSoftVer[4];
	}READ_SOFTWARE_RECV_FMT;

	typedef struct  
	{
		uint8_t		byFirmware[256];
	}READ_FIRMWARE_RECV_FMT;

	typedef struct
	{
		uint8_t	byCode[32];
	}READ_DEVICE_CODE_RECV_FMT;

	typedef READ_DEVICE_CODE_RECV_FMT WRITE_DEVICE_CODE_SEND_FMT;

	typedef struct
	{
		uint8_t	byManuInfo[256];
	}READ_DEVICE_MANU_INFO_RECV_FMT;

	typedef READ_DEVICE_MANU_INFO_RECV_FMT WRITE_DEVICE_MANU_INFO_SEND_FMT;

	typedef struct  
	{
		uint8_t		byLinkIDCount;
		uint8_t		byRsv[3];
		uint8_t		byLinkID[256];
	}ENUM_DEVICE_LINK_ID_RECV_FMT;

	typedef struct  
	{
		uint8_t		byLinkID;
		uint8_t		byRsv[3];
	}QUERY_ONLINE_NODE_COUNT_SEND_FMT;

	typedef struct  
	{
		uint8_t		byLinkID;
		uint8_t		byRsv[3];
		uint16_t		wOnlineNodeCount;
		uint16_t		wAddrLen;
	}QUERY_ONLINE_NODE_COUNT_RECV_FMT;

	typedef struct  
	{
		uint8_t		byLinkID;
		uint8_t		byRsv[3];
		uint16_t		wStartNO;
		uint16_t		wCount;
		uint16_t		wAddrLen;
		uint8_t			byRsv2[2];
	}QUERY_ONLINE_NODE_SEND_FMT;

	typedef struct  
	{
		uint8_t		byLinkID;
		uint8_t		byRsv[3];
		uint16_t		wStartNO;
		uint16_t		wCount;
		uint16_t		wAddrLen;
		uint8_t			byRsv2[2];
		uint8_t		byOnlineNode[512];
	}QUERY_ONLINE_NODE_RECV_FMT;

	typedef struct  
	{
		uint8_t		byLinkID;
		uint8_t		byRsv[3];
	}READ_LINK_PARAM_SEND_FMT;

	typedef struct  
	{
		uint8_t		byLinkID;
		uint8_t		byRsv[3];
		uint8_t		byLinkParam[256];
	}READ_LINK_PARAM_RECV_FMT;

	typedef READ_LINK_PARAM_RECV_FMT WRITE_LINK_PARAM_SEND_FMT;

	typedef struct
	{
		uint8_t		startPara;
		uint8_t		byRsv[3];
	}RESTART_DEVICE_SEND_FMT;

	typedef struct 
	{
		uint8_t byPortID;
		uint8_t byRsv[3];
	}READ_PORT_MODE_SEND_FMT;

	typedef struct
	{
		uint8_t bEnable;
		uint8_t	byRsv[3];
		uint8_t byIP[4];
		uint8_t	bEnable2;
		uint8_t byRsv2[3];
		uint8_t byIP2[4];
	}READ_DEVICE_SNDP_CFG_FMT;

	typedef struct 
	{
		uint8_t byPortID;
		uint8_t byMode;
		uint8_t byRsv[2];
	}READ_PORT_MODE_RECV_FMT;

	typedef READ_PORT_MODE_RECV_FMT SET_PORT_MODE_SEND_FMT;

	typedef struct
	{
		uint8_t byMode;
		uint8_t byRsv[3];

	} READ_IO_MODE_RECV_FMT;

	typedef READ_IO_MODE_RECV_FMT SET_IO_MODE_SEND_FMT;

	typedef struct
	{
		uint8_t byChannel;
		uint8_t byRsv[3];
	}READ_IO_SIGNAL_TYPE_RECV_FMT;

	typedef struct
	{
		uint8_t byChannel;
		uint8_t bySignalType;
		uint8_t byRsv[2];
	} SET_IO_SIGNAL_TYPE_SEND_FMT;


	typedef struct
	{
		float fZone;

	}READ_DEVICE_TIME_SEND_FMT;

	//打开端口
	typedef struct
	{
		uint8_t bytMode;
		uint8_t bytBaud;
		uint8_t bytDataBits;
		uint8_t bytParity;
		uint8_t bytStopBit;
		uint8_t byRsv[3];
	}PORT_PARAM;

	typedef struct
	{
		uint8_t bytPortID;
		uint8_t byRsv[3];
		PORT_PARAM paramPort;
	}OPEN_PORT_SEND_FMT;

	extern "C" {

	RESPONSE XNET_DRIVER_EXPORT SendRequestAndWaitForRecv(const LINK_PARAM* pLinkParam, SERVICE byService, COMMAND byCmd, const void *pSendData, const uint16_t wSendDataLen, void *pRecvData, const uint16_t wRecvDataLen, uint16_t timeout=2000);
}

#ifdef __cplusplus
}
#endif

#endif
