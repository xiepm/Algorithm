#ifndef _REALTIMESERVOJOINT_H_
#define _REALTIMESERVOJOINT_H_
#include <queue>
#include "typeDef.h"

class realTimeServoJoint
{
#define bufferCounts		3

public:

	realTimeServoJoint();

	// Function: 初始化ServoJ功能；
	// Input: currentJointPosition, 当期的关节位置（rad）
	//		   realTimeCycle, 实时更新周期（0.001s);
	//		   servoTime, 用户伺服更新周期（0.008s);
	void initializeRealTimeServoJoint
		(
		const EcRealVector& currentJointPosition,
		const EcReal realTimeCycle,
		const EcReal servoTime
		);

	// Function: 设置ServoJ的时间参数；
	void setTimeParameters
		(
		const EcReal realTimeCycle,
		const EcReal servoTime
		);
	
	// 判断缓存是否已满，
	// InOut: isOverTime，判断起始阶段是否存在超时；
	// 返回true时，说明缓存为未满状态，可通过setServoJointVelocityParameters继续加入参数；
	EcBoolean checkBufferState(EcBoolean& isOverTime);

	// 判断是否需要请求DCS再发送一个点位过来，
	// return true, 则给DCS发送一个请求信号；
	EcBoolean checkRequestSupplyBuffer();

	// Function: 设置从DCS中获取计算得到的速度和加速度参数；
	// Input: jointVelocoty, 关节速度（rad/s）
	//		  jointAcceleration, 关节加速度（rad/s^2）
	void setServoJointVelocityParameters
		(
		EcRealVector& jointVelocity,
		EcRealVector& jointAcceleration
		);


	// Function: 按实时周期调用计算实时命令位置；
	// Output: isFinish, 如果为true，则运动完成；
	//		   jointPosition, 关节命令位置（rad）
	EcBoolean calculateJointPosition
		(
		EcBoolean& isFinish,
		EcRealVector& jointPosition
		);
	
private:
	EcBoolean							b_isInitialize;
	EcBoolean							b_isFinishStartBufferState;
	EcReal								m_timeCycle,m_servoTime;
	EcReal								m_currentTime, m_totalTime;
	EcReal								m_startTime;
	EcU32								m_numJoints;

	std::queue<EcRealVector>			m_velQueue, m_accQueue;
	std::queue<EcReal>					m_timeQueue;

	EcRealVector						m_lastNodePosition;
	EcReal								m_lastNodeTime;
	EcRealVector						m_currentVel, m_currentAcc;
	EcRealVector						m_preJointPosition;

};
#endif