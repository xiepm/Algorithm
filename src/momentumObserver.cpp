#include "momentumObserver.h"
#include <iostream>

std::shared_ptr<momentumObserver> momentumObserver::create
(
	const EcReal updateTimePeriod,
	const EcRealVector& high_b, const EcRealVector& high_a
)
{
	std::shared_ptr<momentumObserver> pMomentumObserver;

	pMomentumObserver.reset(new momentumObserver(updateTimePeriod, high_b, high_a));

	return pMomentumObserver;
}



momentumObserver::momentumObserver(const EcReal updateTimePeriod, const EcRealVector& high_b, const EcRealVector& high_a) :
	m_highPassFilter(high_b, high_a)
{
	m_SamplePeriod = updateTimePeriod;
	b_isInitialize = false;
}


momentumObserver::~momentumObserver()
{

}

EcBoolean momentumObserver::initializeStates
(
	const EcRealVector& jointPositions,
	const int robotType
)
{
	if (robotType == 1 || robotType == 3 || robotType == 8 || robotType == 10)
		m_dynBase.reset(new urDynamics);
	else if (robotType == 20)
		m_dynBase.reset(new anthorDynamics);
	else
		m_dynBase.reset(new elfinDynamics);
	m_NumJoints = jointPositions.size();
	m_KObserverCoeff = 50.0;			//	降低敏感度
	m_jointPosition = jointPositions;
	m_zeros.assign(m_NumJoints, 0.0);
	m_linkSideTorques.assign(m_NumJoints, 0.0);
	
	// Pre-allocate vectors for mass matrix calculation
	m_v1.assign(m_NumJoints, 0.0); if (m_NumJoints > 0) m_v1[0] = 1;
	m_v2.assign(m_NumJoints, 0.0); if (m_NumJoints > 1) m_v2[1] = 1;
	m_v3.assign(m_NumJoints, 0.0); if (m_NumJoints > 2) m_v3[2] = 1;
	m_v4.assign(m_NumJoints, 0.0); if (m_NumJoints > 3) m_v4[3] = 1;
	m_v5.assign(m_NumJoints, 0.0); if (m_NumJoints > 4) m_v5[4] = 1;
	m_v6.assign(m_NumJoints, 0.0); if (m_NumJoints > 5) m_v6[5] = 1;

	m_M1.assign(m_NumJoints, 0.0);
	m_M2.assign(m_NumJoints, 0.0);
	m_M3.assign(m_NumJoints, 0.0);
	m_M4.assign(m_NumJoints, 0.0);
	m_M5.assign(m_NumJoints, 0.0);
	m_M6.assign(m_NumJoints, 0.0);

	m_beta.assign(m_NumJoints, 0.0);
	m_NE0.assign(m_NumJoints, 0.0);
	m_gravityTau.assign(m_NumJoints, 0.0);
	m_gx = 0;	m_gy = 0;	m_gz = -9.81;
	m_collisionThreshold.assign(m_NumJoints, 0.0);
	m_dynamicsFactorThresholds.assign(m_NumJoints, 0.0);
	m_dynamicCollisionThreshold.assign(m_NumJoints, 0.0);
	m_coulombFriction.assign(m_NumJoints, 0.0);

	m_diffMassMatrix.assign(m_NumJoints, m_zeros);
	m_massMatrix.assign(m_NumJoints, m_zeros);
	m_massMatrixPre.assign(m_NumJoints, m_zeros);


	m_diffObserver.assign(m_NumJoints, 0.0);
	m_diffObserverSum.assign(m_NumJoints, 0.0);
	m_preDisturbancsTorques.assign(m_NumJoints, 0.0);
	m_jointMomentum.assign(m_NumJoints, 0.0);
	m_ObserverTorque.assign(m_NumJoints, 0.0);
	m_dynamicsParam.assign(10 * m_NumJoints, 0.);
	m_dynamicsFullParam.assign(13 * m_NumJoints, 0.);
	m_moterSideParams.assign(3 * m_NumJoints, 0.);
	b_is15066Strategy = false;

	m_highPassFilter.setOldData(m_zeros);
	m_filtedObserverTorques = m_zeros;
	if (m_SamplePeriod < 0.00000001)
	{
		m_SamplePeriod = 0.001;
		return false;
	}
	b_isInitialize = true;
	return true;
}




EcBoolean momentumObserver::setKinematicsParameters(const EcRealVector& dhParameters)
{
	m_dynBase->setRobotDHParameters(dhParameters);
	return true;
}

EcBoolean momentumObserver::setDynamicsParameters(const EcRealVector& dynamicsParam)
{
	if (dynamicsParam.size() != 13 * m_NumJoints)
	{
		return false;
	}
	m_dynamicsFullParam = dynamicsParam;

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		for (EcSizeT j = 0; j < 10; j++)
		{
			m_dynamicsParam[i * 10 + j] = dynamicsParam[i * 13 + j];		// dynamics parameters of link side
		}
		for (EcSizeT j = 0; j < 3; j++)
		{
			m_moterSideParams[i * 3 + j] = dynamicsParam[i * 13 + 10 + j];	// motor inertia, viscous and coulomb friction;
		}
	}
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		m_coulombFriction[i] = m_moterSideParams[i * 3 + 2];
	}
	
	// Adjust coulomb friction for pairs of joints if applicable
	for (EcSizeT i = 0; i < m_NumJoints / 2; i++)
	{
		m_coulombFriction[i * 2] = (m_moterSideParams[i * 2] > m_moterSideParams[i * 2 + 1]) ? m_moterSideParams[i * 2] : m_moterSideParams[i * 2 + 1];
		m_coulombFriction[i * 2 + 1] = m_coulombFriction[i * 2];
	}

	// 设置质量矩阵的 初值
	calculateMassMatrix(m_jointPosition, m_massMatrix);
	m_massMatrixPre = m_massMatrix;
	return true;
}

void momentumObserver::setGravityAcceleration
(
	const EcReal gx, const EcReal gy, const EcReal gz
)
{
	m_gx = gx;
	m_gy = gy;
	m_gz = gz;

	m_dynBase->setGravityVector(gx, gy, gz);
}

void momentumObserver::setCollisionThreshold
(
	const EcRealVector threshold
)
{
	m_collisionThreshold = threshold;
	for (int i = 0; i < m_collisionThreshold.size(); i++)
		m_collisionThreshold[i] *= 1.2;
}

void momentumObserver::setDynamicsFactorThreshold
(
	const EcRealVector threshold
)
{
	m_dynamicsFactorThresholds = threshold;
}

EcBoolean momentumObserver::updateStateEstimates
(
	const EcRealVector& jointPositions,
	const EcRealVector& jointVelocities,
	const EcRealVector& jointAccelerations,
	const EcRealVector& sensedTorque
)
{
	if (!b_isInitialize)
	{
		return false;
	}
	m_jointPosition = jointPositions;
	m_jointVel = jointVelocities;
	m_jointAcc = jointAccelerations;
	m_sensedTorques = sensedTorque;
	reduceMotorSideTorques(sensedTorque, jointVelocities, jointAccelerations, m_linkSideTorques);
	calculateDynamicsThreshold();
	return calculateMomentumObserverTorque(jointPositions, jointVelocities, m_linkSideTorques, m_ObserverTorque);
}

void momentumObserver::calculateDynamicsThreshold
(
)
{
	m_dynamicCollisionThreshold = m_collisionThreshold;
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (fabs(m_jointVel[i]) < LOWLEVELVELOCITY)
		{
			if (fabs(m_jointVel[i]) < 0.087)		//速度从零开始跳变时，扰动误差大；
				m_dynamicCollisionThreshold[i] = m_collisionThreshold[i] + m_coulombFriction[i] * 4.0;
			else
				m_dynamicCollisionThreshold[i] = m_collisionThreshold[i] + m_coulombFriction[i] * 3.0;			//todo：这里的计算会导致后面的速度依然会增加这个数值；
		}
		m_dynamicCollisionThreshold[i] += m_dynamicsFactorThresholds[i];		// 加上动态负载的影响
	}

}

void momentumObserver::reduceMotorSideTorques
(
	const EcRealVector& sensedTorque,
	const EcRealVector& dq,
	const EcRealVector& ddq,
	EcRealVector& linkSideTorques
)
{
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		linkSideTorques[i] = sensedTorque[i] - ddq[i] * m_moterSideParams[i * 3] - dq[i] * m_moterSideParams[i * 3 + 1] - sign(dq[i]) * m_moterSideParams[i * 3 + 2];
	}
}


void momentumObserver::getObserverTorqueDisturbances
(
	EcRealVector& disturbanceTorque
)
{
	//disturbanceTorque = m_ObserverTorque;
	disturbanceTorque = m_filtedObserverTorques;
}

void momentumObserver::getRawObserverTorqueDisturbances
(
	EcRealVector& disturbanceTorque
)
{
	disturbanceTorque = m_ObserverTorque;
}

void momentumObserver::getJointCollisionState
(
	EcBooleanVector& jointCollisionState
)
{
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (fabs(m_filtedObserverTorques[i]) > m_dynamicCollisionThreshold[i])
		{
			if (fabs(m_ObserverTorque[i]) > m_collisionThreshold[i])
			{
				jointCollisionState[i] = true;
			}
			else {
				jointCollisionState[i] = false;
			}
		}
	}
}

void momentumObserver::getJointGeneralizeMomentum
(
	EcRealVector& jointMomentum
)
{
	jointMomentum = m_jointMomentum;
}

void momentumObserver::calculateMomentum(const EcRealVector& jointPosition, const EcRealVector& jointVelocity, EcReal& momentum)
{
	EcRealVector jointMomentum(numofJoints);
	calculateGeneralizeMomentum(jointPosition, jointVelocity, jointMomentum);
	momentum = 0;
	for (int i = 0; i < jointMomentum.size(); i++)
	{
		momentum += jointMomentum[i];
	}
}


EcBoolean momentumObserver::calculateMomentumObserverTorque
(
	const EcRealVector& q,
	const EcRealVector& dq,
	const EcRealVector& sensedTorque,
	EcRealVector& observerTorque
)
{
	calculateGeneralizeMomentum(q, dq, m_jointMomentum);
	diffObserver(sensedTorque, m_preDisturbancsTorques, q, dq, m_diffObserver);

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		m_diffObserverSum[i] += m_diffObserver[i] * m_SamplePeriod;
		observerTorque[i] = m_KObserverCoeff * (m_jointMomentum[i] - m_diffObserverSum[i]);
	}
	m_preDisturbancsTorques = observerTorque;
	m_filtedObserverTorques = m_highPassFilter.updateOutput(observerTorque);
	return true;
}


void momentumObserver::calculateGeneralizeMomentum
(
	const EcRealVector& q,
	const EcRealVector& dq,
	EcRealVector& jointMomentum
)
{
	m_dynBase->calculateMomentumEstimatedJointTorques(q, m_zeros, dq, m_dynamicsParam, jointMomentum);
}

void momentumObserver::calculateMassMatrix
(
	const EcRealVector& q,
	EcRealVectorVector& massMatrix
)
{
	massMatrix.assign(m_NumJoints, EcRealVector(m_NumJoints, 0.0));
	EcRealVector ei(m_NumJoints, 0.0);
	for (size_t i = 0; i < m_NumJoints; ++i) {
		ei.assign(m_NumJoints, 0.0);
		ei[i] = 1.0;
		EcRealVector Mi(m_NumJoints, 0.0);
		m_dynBase->calculateMomentumEstimatedJointTorques(q, m_zeros, ei, m_dynamicsParam, Mi);
		massMatrix[i] = Mi;
	}
}

void momentumObserver::calculateBetaQuantity
(
	const EcRealVector& q,
	const EcRealVector& dq,
	EcRealVector& beta
)
{
	beta = m_zeros;
	m_dynBase->calculateMomentumEstimatedJointTorques(q, dq, m_zeros, m_dynamicsParam, m_NE0);
	m_dynBase->calculateGravityJointTorques(q, m_dynamicsFullParam, m_gravityTau);		// 计算重力

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		for (EcSizeT j = 0; j < m_NumJoints; j++)
		{
			m_diffMassMatrix[j][i] = m_massMatrix[j][i] - m_massMatrixPre[j][i];
			beta[i] += m_diffMassMatrix[j][i] * dq[j];
		}
		beta[i] /= m_SamplePeriod;
		beta[i] = m_NE0[i] + m_gravityTau[i] - beta[i];
	}

}

void momentumObserver::diffObserver
(
	const EcRealVector& sensedTorque,
	const EcRealVector& disturbanceTorque,
	const EcRealVector& q,
	const EcRealVector& dq,
	EcRealVector& diffObser
)
{
	m_massMatrixPre = m_massMatrix;
	calculateMassMatrix(q, m_massMatrix);
	calculateBetaQuantity(q, dq, m_beta);
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		diffObser[i] = sensedTorque[i] - m_beta[i] + disturbanceTorque[i];
	}
}


EcReal momentumObserver::sign(const EcReal x)
{
	if (fabs(x) < 0.00001)
		return 0.;

	if (x > 0)
		return 1.0;
	else
		return -1.0;
}
