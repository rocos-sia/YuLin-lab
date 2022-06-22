/*
*  file : xdriver.h
*  note : read/write tag from controller
*/

#ifndef _H_XNETROBOTACS
#define _H_XNETROBOTACS
#pragma once
#include "xNetDriver.h"
//////////////////////////////////////////////////////////////////////////
//机器人通讯
#ifdef __cplusplus
namespace xNETDRIVER
{
#endif
	const RESPONSE	XNET_ROBOT_AUTO_MODE = 0x80;				// 机器人处于自动模式
	const RESPONSE	XNET_MAP_LOADING = 0x81;					// 正在更新地图
	const RESPONSE	XNET_LOCATION_NOT_SUCCESS = 0x82;			// 机器人定位未成功
	const RESPONSE	XNET_PATH_POINT_COUNT_EXCEED = 0x83;		// 路径点数量超过最大值
	const RESPONSE	XNET_FORBIDDEN_PATH_COUNT_EXCEED = 0x84;	// 禁行路径数量超过最大值
	const RESPONSE	XNET_FORBIDDEN_AREA_COUNT_EXCEED = 0x85;	// 禁行区域数量超过最大值
	const RESPONSE  XNET_ROBOT_MANUL_MODE = 0x86;				// 机器人处于手动模式
	const RESPONSE	XNET_INVALID_GOAL_ID = 0x88;				// 导航目标点ID不存在
	const RESPONSE	XNET_GOAL_POSE_NOT_IN_PATH = 0x89;			// 目标点不在路径上
	const RESPONSE	XNET_PATH_PLAN_FAILED = 0x8A;				// 路径规划失败
	const RESPONSE	XNET_ROBOT_OFFSET_PATH = 0x8B;				// 机器人偏离路径，需要手动遥控到路径上
	const RESPONSE	XNET_ROBOT_IN_SONOR_DISABLE_AREA = 0x8C;	// 机器人再超声屏蔽区域

	const SERVICE	ROBOT_COMMUNICATE = 0x10;		//机器人通信服务

													//机器人通讯命令码
	const COMMAND	READ_VALUE = 0x00;						//读变量值
	const COMMAND	WRITE_VALUE = 0x01;						//写变量值

	const COMMAND	AUTO_MANUAL_CHANGE = 0x11;				//手自动切换
	const COMMAND	ROBOT_CONTROL = 0x13;						//机器人控制
	const COMMAND	ROBOT_LOCATE = 0x14;						//机器人定位
	const COMMAND	ROBOT_POSITION = 0x15;					//获取机器人当前位置
	const COMMAND	SEND_WAY_POINT = 0x16;					//下发导航点
	const COMMAND	ROBOT_STATE = 0x17;						//获取机器人状态
	const COMMAND	ROBOT_TASK_STATE = 0x18;					//获取机器人任务状态

	const COMMAND	ROBOT_INFO_SET = 0x1C;					//机器人配置
	const COMMAND	ROBOT_POINT_ARRIVE = 0x1D;				//查询机器人到达导航点状态
	const COMMAND	ROBOT_LASER_DATA = 0x1E;					//获取激光数据
	const COMMAND	ROBOT_RELOCATE_COMPLETE = 0x1F;			//确认重定位完成


	const COMMAND	READ_NEW_ROBOT_SET = 0x38;
	const COMMAND	WRITE_NEW_ROBOT_SET = 0x39;
	const COMMAND	READ_ROBOT_BASE_INFO = 0x3A;			// 读取机器人基本信息
	const COMMAND	SET_FORBIDDEN_AREA = 0x3B;					// 设置禁止导航区域
	const COMMAND	READ_LASER_COUNT = 0x3C;					// 读取激光设备数量
	const COMMAND	UPDATE_ROBOT_PARAM = 0x3D;			// 更新机器人参数
	const COMMAND	LASER_POSITION_ADJUST = 0x3E;			// 标定机器人安装位置
	const COMMAND	READ_REFLECOR_INFO = 0x3F;					// 读取反光柱信息


	const COMMAND	QUERY_TRAFFIC_RESOURCE_REQUEST_INFO = 0x70;		// 查询交通管理请求信息
	const COMMAND	SET_TRAFFIC_RESOURCE_ALLOC_INFO = 0x71;					// 设置交通管理分配结果

	const COMMAND	READ_WIZARD_PARAM = 0x7E;				//读取向导生成的配置
	const COMMAND	ROBOT_MANUL_CONTROL = 0x80;				//手动控制机器人
	const COMMAND	ROBOT_MANUL_STATUS = 0x81;				//手动控制机器人
	const COMMAND	ROBOT_SET_CONTROL_MODE = 0x82;		//切换控制方式
	const COMMAND	ROBOT_GET_CONTROL_MODE = 0x83;		//获取控制方式
	const COMMAND	ROBOT_SET_WORK_MODE = 0x84;			// 设置工作模式
	const COMMAND	ROBOT_GET_WORK_MODE = 0x85;			// 读取工作模式

	const COMMAND	ROBOT_MAKE_MAP_RAWFILE_INDEX = 0x90;	// 制图原始数据文件序号

	typedef struct
	{
		uint8_t byValueName[16];
	}READ_VALUE_SEND_FMT;

	typedef struct
	{
		uint8_t byValueName[16];
		uint8_t byValue[256];

	}READ_VALUE_RECV_FMT;

	//写变量值
	typedef READ_VALUE_RECV_FMT WRITE_VALUE_SEND_FMT;

	//底盘控制
	typedef struct
	{
		double dLineSpeed;
		double dAngularSpeed;

	}BASE_CONTROL_SEND_FMT;

	//手自动切换
	typedef struct
	{
                uint8_t byMode;//0：切换到手动模式，1：切换到自动模式
		uint8_t byRes[3];

	}AUTO_MANUAL_CHANGE_SEND_FMT;

	//机器人控制
	typedef struct
	{
		uint8_t byMode;
		uint8_t byRes[3];
	}ROBOT_CONTROL_SEND_FMT;

	//机器人定位
	typedef struct
	{
		double dPointX;
		double dPointY;
		double dPointA;
	}ROBOT_LOCATE_SEND_FMT;

	//获取机器人当前位置
	typedef struct
	{
		double dPosX;
		double dPosY;
		double dPosTheta;
		double dReliability;
		uint16_t	wLaserCount;
		uint16_t wValidLaserCount;
		uint16_t wMatchedLaserCount;
		uint16_t wReflectorCount;
		uint8_t rsv[8];
		double dOdomX;
		double dOdomY;
		double dOdomTheta;
		uint8_t rsv1[8];
	}ROBOT_POSITION_RECV_FMT;

	//导航点控制
	typedef struct
	{
		uint8_t bytMode;		// 0：前往导航点	1：取消前往导航点	2：暂停导航	3：继续导航   4：下发导航任务但暂停导航
                uint8_t byWay;			// 导航方式，0：导航到路径点	1：导航到路径上的点	2：自由导航到地图上的点
                uint8_t bySetPath;		// 是否指定导航路径，0：不指定导航路径	1：指定导航路径
                uint8_t byTrafficControl;	// 0：不启用交通管理，1：启用交通管理，所有点和路径需要申请后才能使用
                uint8_t PointName[8];           //路径点id，字符串格式，导航方式为0时候使用
                uint16_t	byPathStartID;  //目标路径起点ID，导航方式为1使用
                uint16_t	byPathEndID;    //目标路径终点ID，导航方式为1使用
                float		goal_x;         //导航方式为1时候使用
                float		goal_y;         //导航方式为1时候使用
		float		goal_theta;
		uint8_t bytRes2[2];

                uint8_t byPathSize;				// 指定路径的数量
		uint8_t bytRes3;
                uint16_t	wPath[128];			// 指定的路径的所有路径点id，导航指定路径时候使用

		uint8_t bytRes4[12];

		uint8_t byForbiddenPathSize;				// 禁行的路径数量
		uint8_t bytRes5[3];		
		uint16_t	wForbiddenPath[64];				// 禁行的路径
	}SEND_WAY_POINT_SEND_FMT;

	//获取机器人状态
	typedef struct
	{
		double		dTemp;
                double		dPointX;//x坐标
                double		dPointY;//y坐标
                double		dTheta;//方向角
                double		dBattery;//电池电量，0--1
                uint8_t		byBlocked;
		uint8_t		byCharge;
                uint8_t		byRunMode;//运行模式，0手动，1自动
                uint8_t		byLoadMapState;//地图载入状态，0成功，1失败
                uint32_t	dwGoalID;//当前目标点id
                double		dForwardSpeed;//前进速度
                double		dRotateSpeed;//转弯速度
		double		dChargeVoltage;
		double		dChargeCurrent;
                uint8_t		byTaskState;//当前导航状态，0：无到导航点任务，1：等待，2：正在前往导航点，3：暂停，4：完成，5：失败，6：退出
		uint8_t		byRes;
		uint16_t	nMapVersion;	
		uint8_t		byRes2[4];
		double		dOdometer;
		double		dRunTime;
		double		dAddRunTime;
		uint8_t		byRelocState;
		uint8_t		byRes1[3];
		uint32_t	dwMapNum;
		uint8_t		MapName[64];
                float		fConfidence;//置信度
	}ROBOT_STATE_RECV_FMT;

	//查询是否到达目标点
	typedef struct
	{
                uint8_t		byState;//导航状态，0：无到导航点任务，1：等待，2：正在前往导航点，3：暂停，4：完成，5：失败，6：退出
		uint8_t		byRsv[3];
                uint16_t		nGoalID;//导航目标点
		uint8_t		byRsv2[2];
                uint16_t		FinishID[126];//已经经过的路径点ID，0表示无效路径点
                uint16_t		UnfinishID[126];//未经过的路径点ID，0表示无效路径点
	}ROBOT_POINT_ARRIVE_STATE_RECV_FMT;

	typedef struct
	{
		double		dLaserX;
		double		dLaserY;
		uint8_t		bytInversion;
		uint8_t		bytLaserType;
		uint8_t		byRes[2];
		uint32_t	dwLaserIP;
		double		dCarLen;
		double		dCarWidth;
		uint32_t	dwControlerIP;	
		uint8_t		byRes1[4];
		double		addLineSpeed;
		double		addAngleSpeed;
		double		maxLineSpeed;
		double		maxAngleSpeed;
		double		DecelerationDistance;
		double		DistanceAccuracy;
		double		AngleAccuracy;
		uint32_t	LostTime;
		uint8_t		byRes2[4];

	}SET_ROBOT_INFO_SEND_FMT;
	//查询机器人配置
	typedef struct
	{
		double		dLaserX;
		double		dLaserY;
		uint8_t		bytInversion;
		uint8_t		bytLaserType;
		uint8_t		byRes[2];
		uint32_t	dwLaserIP;
		double		dCarLen;
		double		dCarWidth;
		uint32_t	dwControlerIP;
		double		addLineSpeed;
		double		addAngleSpeed;
		double		maxLineSpeed;
		double		maxAngleSpeed;
		double		DecelerationDistance;
		double		DistanceAccuracy;
		double		AngleAccuracy;
		uint32_t	LostTime;
		uint8_t		robotType[64];
		uint8_t		robotVersion[64];
	}READ_ROBOT_INFO_SET_READ_FMT;

	typedef struct  
	{
		float	left;
		float	bottom;
		float	right;
		float	top;
	}FORBIDDEN_AREA;

	typedef struct 
	{
		uint16_t	area_cnt;
		uint8_t	rsv[2];
		FORBIDDEN_AREA	area[16];
	}SET_FORBIDDEN_AREA_SEND_FMT;

	typedef struct
	{
                uint16_t			line_id;
                uint16_t			point_id;
	}TRAFFIC_RESOURCE_PATH;

	typedef struct
	{
		uint8_t			request_cmd;		// 0：无请求；1：请求占据资源
		uint8_t			path_count;
		uint8_t			rsv[2];
		TRAFFIC_RESOURCE_PATH pathes[16];
	}QUERY_TRAFFIC_RESOURCE_RECV_FMT;

	typedef struct
	{
		uint8_t			alloc_result;			// 0：占据失败；1：占据成功
		uint8_t			path_count;
		uint8_t			rsv[2];
		TRAFFIC_RESOURCE_PATH pathes[16];
	}SET_TRAFFIC_RESOURCE_SEND_FMT;

	struct ROBOT_MANUL_CONTROL_FMT
	{
		float velocity_x;		// x方向线速度
		float velocity_y;		// y方向线速度
		float velocity_angle;	// 角速度
	};

	struct ROBOT_CHASSIS_PARAM_FMT
	{
		float fMaxLineVelocity;		// 最大线速度
		float fMaxAngleVelocity;	// 最大角速度
		float fMaxLineAcc;			// 最大线加速度
		float fMaxAngleAcc;			// 最大角加速度
		float fMinLineVelocity;		// 最小线速度
		float fMinAngleVelocity;	// 最小角速度
		float fMaxLineDec;			// 最大线减速度
		float fMaxAngleDec;			// 最大角减速度
		uint8_t byRsv[64];			// 保留
	};


	//////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
	extern "C" {
#endif  /* __cplusplus */

		//////////////////////////////////////////////////////////////////////////
		//机器人通讯
		//读变量值
		RESPONSE XNET_DRIVER_EXPORT ReadValue(const LINK_PARAM*pLinkParam, READ_VALUE_SEND_FMT *pSend, READ_VALUE_RECV_FMT *pRecv, uint16_t timeout = 2000);

		//写变量值
		RESPONSE XNET_DRIVER_EXPORT WriteValue(const LINK_PARAM*pLinkParam, WRITE_VALUE_SEND_FMT *pSend, uint16_t timeout = 2000);

		//底盘控制
		RESPONSE XNET_DRIVER_EXPORT BaseControl(const LINK_PARAM*pLinkParam, BASE_CONTROL_SEND_FMT *pSend, uint16_t timeout = 2000);

		//手自动切换
		RESPONSE XNET_DRIVER_EXPORT AutoManualChange(const LINK_PARAM*pLinkParam, AUTO_MANUAL_CHANGE_SEND_FMT *pSend, uint16_t timeout = 2000);

		//机器人控制
		RESPONSE XNET_DRIVER_EXPORT RobotControl(const LINK_PARAM*pLinkParam, ROBOT_CONTROL_SEND_FMT *pSend, uint16_t timeout = 2000);

		//机器人定位
		RESPONSE XNET_DRIVER_EXPORT RobotLocate(const LINK_PARAM*pLinkParam, ROBOT_LOCATE_SEND_FMT *pSend, uint16_t timeout = 2000);

		//获取机器人当前位置
		RESPONSE XNET_DRIVER_EXPORT GetRobotPosition(const LINK_PARAM*pLinkParam, ROBOT_POSITION_RECV_FMT *pRecv, uint16_t timeout = 2000);

		//导航点控制
		RESPONSE XNET_DRIVER_EXPORT SendWayPoint(const LINK_PARAM*pLinkParam, SEND_WAY_POINT_SEND_FMT *pSend, uint16_t timeout = 2000);

		//获取机器人状态
		RESPONSE XNET_DRIVER_EXPORT GetRobotState(const LINK_PARAM*pLinkParam, ROBOT_STATE_RECV_FMT *pRecv, uint16_t timeout = 2000);


		//查询是否到达目标点
		RESPONSE XNET_DRIVER_EXPORT GetPointArriveState(const LINK_PARAM*pLinkParam, xNETDRIVER::ROBOT_POINT_ARRIVE_STATE_RECV_FMT*pRecv, uint16_t timeout = 2000);

		//完成重定位
                RESPONSE XNET_DRIVER_EXPORT RelacateComplete(const LINK_PARAM*pLinkParam, uint16_t timeout = 2000);//机器人手动定位后需要确认机器人位置才可以正常导航，机器人开机后也需要确认机器人位置才可以正常导航。
		
		// 设置禁止导航区域
		RESPONSE XNET_DRIVER_EXPORT SetForbiddenArea( const LINK_PARAM *pLinkParam, const xNETDRIVER::SET_FORBIDDEN_AREA_SEND_FMT *pSend, uint16_t timeout = 2000);
		

		//查询机器人配置
		RESPONSE XNET_DRIVER_EXPORT ReadRobotInfoSet(const LINK_PARAM *pLinkParam, xNETDRIVER::READ_ROBOT_INFO_SET_READ_FMT*pRecv, uint16_t timeout = 2000);
		
		// 查询交通管理资源请求信息
		RESPONSE XNET_DRIVER_EXPORT QueryTrafficResourceRequestInfo(const LINK_PARAM * pLinkParam, QUERY_TRAFFIC_RESOURCE_RECV_FMT* pRecv, uint16_t timeout = 2000);

		// 设置交通管理资源占据结果
		RESPONSE XNET_DRIVER_EXPORT SetTrafficResourceOccupiedInfo(const LINK_PARAM * pLinkParam, SET_TRAFFIC_RESOURCE_SEND_FMT* pSend, uint16_t timeout = 2000);

#ifdef __cplusplus
	}
#endif /* __cplusplus */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
