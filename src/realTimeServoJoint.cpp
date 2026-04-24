#include "realTimeServoJoint.h"
#include <iostream>

// RTOS通过队列机制缓存；
realTimeServoJoint::realTimeServoJoint()
{
	m_totalTime = 0.;
	m_timeCycle = 0.001;
	m_servoTime = 0.008;
	b_isInitialize = false;
}

void realTimeServoJoint::initializeRealTimeServoJoint
(
const EcRealVector& currentJointPosition,
const EcReal realTimeCycle,
const EcReal servoTime
)
{
	m_timeCycle = realTimeCycle;
	m_servoTime = servoTime;
	m_currentTime = 0.;
	m_startTime = 0.;
	m_lastNodeTime = 0.;
	m_totalTime = 0.;

	m_lastNodePosition = currentJointPosition;
	m_numJoints = currentJointPosition.size();

	m_currentVel.assign(m_numJoints, 0.0);
	m_currentAcc.assign(m_numJoints, 0.0);

	m_velQueue = std::queue<EcRealVector>();
	m_accQueue = std::queue<EcRealVector>();
	m_timeQueue = std::queue<EcReal>();

	//m_velQueue.push(m_currentVel);
	//m_accQueue.push(m_currentAcc);
	m_timeQueue.push(0.0);

	m_preJointPosition = currentJointPosition;
	b_isFinishStartBufferState = false;
	b_isInitialize = true;
}

void realTimeServoJoint::setTimeParameters
(
const EcReal realTimeCycle,
const EcReal servoTime
)
{
	m_timeCycle = realTimeCycle;
	m_servoTime = servoTime;
}

EcBoolean realTimeServoJoint::checkBufferState(EcBoolean& isOverTime)
{
	m_startTime += m_timeCycle;
	isOverTime = false;
	if (m_startTime > 10 * m_servoTime)
	{
		isOverTime = true;
	}
	if (m_velQueue.size() > bufferCounts)
	{
		b_isFinishStartBufferState = true;
		return false;
	}
	else{
		return true;
	}
}

EcBoolean realTimeServoJoint::checkRequestSupplyBuffer()
{
	if (b_isFinishStartBufferState)
	{
		if (m_velQueue.size() < bufferCounts)
		{
			std::cout << "****************Request Supply Buffer*****************" << std::endl;
			return true;
		}
	}

	return false;
}


void realTimeServoJoint::setServoJointVelocityParameters
(
EcRealVector& jointVelocity,
EcRealVector& jointAcceleration
)
{
	m_startTime = 0.;		//用于避免启动阶段一直没有数据点插入，导致卡死的状态；
	m_velQueue.push(jointVelocity);
	m_accQueue.push(jointAcceleration);
	m_totalTime += m_servoTime;
	m_timeQueue.push(m_totalTime);
}

EcBoolean realTimeServoJoint::calculateJointPosition
(
EcBoolean& isFinish,
EcRealVector& jointPosition
)
{
	if (!b_isInitialize)
		return false;
	m_currentTime += m_timeCycle;
	if (m_currentTime > m_totalTime)	// 最后节点的前一个；
	{
		isFinish = true;
		std::cout << "The servoJ is Finished." << std::endl;
		m_currentTime -= m_timeCycle;
		jointPosition = m_preJointPosition;
		return true;
	}
	else
		isFinish = false;
	std::cout << "current time: " << m_currentTime <<", "<< "queue size: " << m_velQueue.size() << std::endl;
	if (m_currentTime > m_timeQueue.front())
	{
		for (EcU32 i = 0; i < m_numJoints; i++)
		{
			m_lastNodePosition[i] += m_currentVel[i] * m_servoTime + m_currentAcc[i] * m_servoTime*m_servoTime;
		}
		m_lastNodeTime = m_timeQueue.front();
		m_currentVel = m_velQueue.front();
		m_currentAcc = m_accQueue.front();
		m_timeQueue.pop();
		m_velQueue.pop();
		m_accQueue.pop();
		//std::cout << "pop queue" << std::endl;
	}



	EcReal t = m_currentTime - m_lastNodeTime;
	for (EcU32 i = 0; i < m_numJoints; i++)
	{
		jointPosition[i] = m_lastNodePosition[i] + m_currentVel[i] * t + 0.5*m_currentAcc[i] * t*t;
	}
	m_preJointPosition = jointPosition;
	//std::cout << "joint1 position: " << jointPosition[0] << std::endl;
	return true;
}

