#include "sevendofDynamics.h"

sevendofDynamics::sevendofDynamics()
{
	m_NumJoints = 7;
	m_kinematicParams.assign(m_NumJoints, 0.0);
	m_gx = 0.0;
	m_gy = 0.0;
	m_gz = -9.81;
	m_gravity = KDL::Vector(m_gx, m_gy, m_gz);
	m_zeros.assign(m_NumJoints, 0.0);
}

sevendofDynamics::~sevendofDynamics()
{
}

void sevendofDynamics::setRobotDHParameters
(
	const EcRealVector& kinematcisParam
)
{
	m_kinematicParams = kinematcisParam;
}

void sevendofDynamics::setGravityVector
(
	const EcReal gx, const EcReal gy, const EcReal gz
)
{
	m_gx = gx;
	m_gy = gy;
	m_gz = gz;
	m_gravity = KDL::Vector(m_gx, m_gy, m_gz);
}

void sevendofDynamics::calculateGravityJointTorques
(
	const EcRealVector& q,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	(void)parms;

	if (q.size() != m_NumJoints)
	{
		tau.assign(m_NumJoints, 0.0);
		return;
	}

	// TODO: 补入七轴机器人的重力项公式。
	tau.assign(m_NumJoints, 0.0);
}

EcBoolean sevendofDynamics::calculateEstimateJointToqrues
(
	const EcRealVector& q,
	const EcRealVector& dq,
	const EcRealVector& ddq,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	(void)parms;

	if (q.size() != m_NumJoints ||
		dq.size() != m_NumJoints ||
		ddq.size() != m_NumJoints)
	{
		tau.assign(m_NumJoints, 0.0);
		return false;
	}

	// TODO:
	// 1. 在这里填入七轴完整逆动力学公式；
	// 2. 若继续沿用项目现有参数组织，注意参数长度会从 6*13 变成 7*13 = 91；
	// 3. 温度摩擦、电机惯量、库仑/粘性摩擦是否保留，也需要一起定。
	tau.assign(m_NumJoints, 0.0);
	return false;
}

void sevendofDynamics::calculateMomentumEstimatedJointTorques
(
	const EcRealVector& q,
	const EcRealVector& dq,
	const EcRealVector& ddq,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	(void)q;
	(void)dq;
	(void)ddq;
	(void)parms;

	// TODO: 补入七轴动量观测用模型力矩。
	tau.assign(m_NumJoints, 0.0);
}
