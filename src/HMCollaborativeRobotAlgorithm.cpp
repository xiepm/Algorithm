#include "HMCollaborativeRobotAlgorithm.h"
//#include "EcOs.h"


#define				NUMofMeanVelFactorConst				200

std::shared_ptr<CHansCollaborativeAlgorithm> CHansCollaborativeAlgorithm::create
(
	const EcRealVector& jointPositions,
	const EcReal updateTimePeriod,
	const int robotType
)
{
	std::shared_ptr<CHansCollaborativeAlgorithm> pCollaborativeTool;

	pCollaborativeTool.reset(new CHansCollaborativeAlgorithm(updateTimePeriod));
	pCollaborativeTool->m_NumJoints = jointPositions.size();

	if (pCollaborativeTool)
	{
		pCollaborativeTool->initializeStates(jointPositions, robotType);
	}

	return pCollaborativeTool;
}

CHansCollaborativeAlgorithm::CHansCollaborativeAlgorithm
(
	const EcReal updateTimePeriod
) :
	m_StateEstimator(updateTimePeriod),
	m_commandStateEstimator(updateTimePeriod)
	//m_hmAlgorithm(0)
{
	m_updateTimePeriod = updateTimePeriod;
	m_stopGenerateFrictCounts = 2.0 * 60.0;
	m_stopStateTimeCounts = 0;
	//m_sensorlessAdmitControl = CAdmittancePositionController::create(updateTimePeriod);
	EcRealVector b = { 0.9922, -0.9922 };
	EcRealVector a = { 1, -0.9844 };
	m_momentumObserver = momentumObserver::create(updateTimePeriod, b, a);

	m_prePayloadMass = 0.0;
	b_newPayLoadStatus = false;
	m_payloadCount = 1;
}

CHansCollaborativeAlgorithm::~CHansCollaborativeAlgorithm()
{
}

EcBoolean CHansCollaborativeAlgorithm::initializeStates
(
	const EcRealVector& jointPositions,
	const int robotType
)
{
	if (jointPositions.size() != m_NumJoints)
	{
		m_IsInitialized = EcFalse;
		return EcFalse;
	}
	// 机型约定：
	// 1 -> UR 六轴
	// 2 -> 七轴机型（当前动力学为占位实现）
	// other -> Elfin 六轴
	if (robotType == 1)
		m_dynBase.reset(new urDynamics);
	else if (robotType == 2)
		m_dynBase.reset(new sevendofDynamics);
	else
		m_dynBase.reset(new elfinDynamics);

	m_robotType = robotType;

	m_logStatus = true;

	m_currentTime = 0.0;
	b_isUsingMomentumObserver = EcFalse;
	m_momentumObserver->initializeStates(jointPositions, robotType);
	m_frictionModel.initialize(jointPositions, m_updateTimePeriod);
	m_startCompensateFrictionFactor.assign(m_NumJoints, 1.0);

	m_driveBackTorques.assign(m_NumJoints, 0.);
	m_driveBackStatus.assign(m_NumJoints, EcFalse);
	m_driveBackBrakingStatus.assign(m_NumJoints, EcFalse);
	m_collisionStopStatus.assign(m_NumJoints, EcFalse);
	m_AdjustedTorqueConstants.assign(m_NumJoints, 1.0);
	m_MaxActutorTorques.assign(m_NumJoints, 0.0);
	m_MaxActutorCurrents.assign(m_NumJoints, 1.0);

	//m_sensorlessAdmitControl->initializeStates();
	b_isSensorlessAdmittanceMode = EcFalse;
	m_FilteredJointPositions = jointPositions;
	m_FilteredJointVelocities.assign(m_NumJoints, 0.0);
	m_FilteredJointAccelerations.assign(m_NumJoints, 0.0);

	m_FilteredCommandJointPositions = jointPositions;
	m_FilteredCommandJointVelocities.assign(m_NumJoints, 0.0);
	m_FilteredCommandJointAccelerations.assign(m_NumJoints, 0.0);

	m_FilteredMotorCurrents.assign(m_NumJoints, 0.0);

	m_SensedJointTorques.assign(m_NumJoints, 0.0);
	m_EstimatedJointTorques.assign(m_NumJoints, 0.0);
	m_DisturbanceJointTorques.assign(m_NumJoints, 0.0);
	m_meanDisturbanceJointTorques.assign(m_NumJoints, 0.0);
	m_FilteredSensedEeForces.assign(6, 0.0);
	m_FilteredAdmittanceDeviatePose.assign(6, 0.0);
	m_filteredAdmittanceVelocity.assign(6, 0.0);

	m_StateEstimator.initialize(jointPositions);
	m_commandStateEstimator.initialize(jointPositions);
	EcRealVector filterTimeConstant;
	filterTimeConstant.assign(m_NumJoints, 0.012);			// 设置滤波的时间常数；
	setFilterTimeConstant(jointPositions, filterTimeConstant, filterTimeConstant);

	m_ViscousFrictionCoefficient.assign(m_NumJoints, 0.0);
	m_CoulombFriction.assign(m_NumJoints, 0.0);
	m_vibrationPeriod.assign(7, 0.0);
	m_JacobianDeterminant = 1.0;

	m_jointLimitAgainstForceEquivalent.assign(m_NumJoints, 100.0);

	m_timeStep = m_StateEstimator.samplingPeriod();

	m_selectedPositionCoeff = { 1, 1, 0, 0, 0, 0 };
	m_viscousVelocityCoeff = { 1, 1, 1, 1, 1, 1 };
	m_endEffectorInertia = 0.5;
	m_endEffectorMass = 5;
	m_eeCalcVelocity.assign(6, 0.0);
	m_eeCalcPosition.assign(6, 0.0);
	m_eeCurrentPosition.assign(6, 0.0);

	m_payloadMass = 0.0;				//负载配置；
	m_centerofMass = { 0.0, 0.0, 0.0 };

	m_d1 = 0.220;		//默认设置为ElfinV5的杆长参数；
	m_d4 = 0.420;
	m_d6 = 0.180;
	m_a2 = 0.380;
	m_kinParams.assign(10, 0.0);

	m_gx = 0;
	m_gy = 0;
	m_gz = -9.81;
	m_gravity = { 0, 0, -9.81 };

	m_maxEfficiency.assign(m_NumJoints, 1.0);
	m_dampEfficiency.assign(m_NumJoints, 0.0);
	m_dampSetting.assign(m_NumJoints, 1.0);


	EcRealVector para;
	para.assign(78, 0.0);

	EcReal m = 0;
	EcRealVector cop = { 0, 0, 0 };
	m_DynamicsLinearParameters = para;
	m_DynamicsParameters = para;
	m_baseMountingRotation = 0.0;
	m_baseMountingTilt = 0.0;

	m_friTemperaturesParams.assign(m_NumJoints, 1.0);
	m_jointTemperatures.assign(m_NumJoints, 45.0);
	m_jointVoltages.assign(m_NumJoints, 48.0);


	EcReal rotation = 0;
	EcReal tilt = 0;
	setRobotMountingAngles(rotation, tilt);

	m_frictionCompensatoryFactor.assign(m_NumJoints, 0.7);
	m_UpperJointLimits.assign(m_NumJoints, 3.14);
	m_LowerJointLimits.assign(m_NumJoints, -3.14);
	m_jointRange.assign(m_NumJoints, 6.24);
	m_slowDownBoundary.assign(m_NumJoints, 0.2);
	m_stopBoundary.assign(m_NumJoints, 0.005);

	m_maxJointVelocitiesInAssistiveMode = { 1.05, 1.05, 1.05, 1.75, 1.75, 2.35 };    //设置关节默认最大速度
	m_frictionModel.setMaxJointVelocitiesInAssistiveMode(m_maxJointVelocitiesInAssistiveMode);
	m_nonZeroVelocity = 0.001;
	m_assistiveCheckTime = 0.08;

	m_zeroVelocityThresholds = { 0.06,0.06,0.06,0.06,0.06,0.06 };
	m_lowVelocityThresholds = { 0.25, 0.25, 0.25, 0.25, 0.25, 0.25 };
	m_CollisionStopDynamicsThresholds.assign(m_NumJoints, 0.0);
	setVibrationPeriod();
	m_VibrationAmplitude = { 0.9, 0.9, 0.9, 1.0, 1.0, 1.2 };

	m_maxJointVeloctiy.assign(m_NumJoints, 1.5);
	m_maxJointAccelerations.assign(m_NumJoints, 6.0);


	m_IsInitialized = EcTrue;
	m_IsAdmittanceInitialized = EcFalse;

	m_currentAssistiveTime = 10.;
	m_observerTorques.assign(m_NumJoints, 0.0);
	m_AssistiveStartJointPosition.assign(m_NumJoints, 0.0);
	m_AssistiveStartSensedTorques.assign(m_NumJoints, 0.0);

	m_AssistiveSafeAccumulateCount = { 50, 50, 50, 50, 50, 150 };
	m_AssistiveSafeStartAccel = { 0.8, 0.8, 0.8, 0.8, 0.8, 1.5 };
	m_AssistiveStartSumAccel.assign(m_NumJoints, 0.0);
	m_zeroVector.assign(m_NumJoints, 0.0);
	b_falseVector.assign(m_NumJoints, 0.0);

	m_AssistiveState = assistive_normal;
	m_motionConstraintScale = 1.0;
	b_previousAssisteErrorState = false;



	m_driveBackStatusVector.resize(6);
	m_driveBackStartPosition = jointPositions;
	m_overZeroVelCount.assign(m_NumJoints, 0);
	m_actualPreviousJointVel.assign(m_NumJoints, 0.0);
	m_actualJointPosition.assign(m_NumJoints, 0.0);
	m_sensedRawTorque.assign(m_NumJoints, 0.0);
	m_startCollisionJointVel.assign(m_NumJoints, 0.0);


	m_driveBackDisturbancesThreshold.assign(m_NumJoints, 100.0);
	m_driveBackCollisionStopStatus.assign(m_NumJoints, EcFalse);
	m_massThresholdTorques.assign(m_NumJoints, 0.0);
	m_MomentumCollisionThresholds.assign(m_NumJoints, 0.0);
	m_maxJointTorques.assign(m_NumJoints, 0.0);
	m_AssistiveModeCollisionStopThresholds.assign(m_NumJoints, 200.0);
	m_startDriveBackJointPosition.assign(m_NumJoints, 0.0);

	// 弹性摩擦相关参数；
	m_alpha.assign(m_NumJoints, 20.0);
	m_omegaK.assign(m_NumJoints, 0.0);
	m_frictionRatio2.assign(m_NumJoints, 0.7);
	m_calcCoulombFriction.assign(m_NumJoints, 0.0);
	m_alphaL = 20;
	m_alphaH = 500;
	m_Rw = 0.01;
	m_velThreshold = 0.005;
	m_KVector = { 3000, 3000, 2000, 2000, 2500, 500 };
	// 弹性摩擦相关参数
	m_jointMPosition = jointPositions;
	m_jointSidePosition = jointPositions;
	m_CollisionStopThresholds.assign(m_NumJoints, 100);

	m_allowBackDistance = 5 * KDL::PI / 180.0;
	m_allowMotionDuration = 0.5;

	m_maxPower = 300;
	m_maxMomentum = 25;
	m_maxConstraintPower = 300;
	m_maxConstraintMomentum = 25;
	m_maxJointPowers.assign(6, 1000);
	m_velFactorConstraint = 1.0;
	m_preVelFactorConstraint = 1.0;
	m_accFactorConstraint = 1.0;

	// mean in NUMofMeanVelFactorConst cycle;
	m_meanFilteredVelConstraint.assign(NUMofMeanVelFactorConst, 1.0);
	m_constraintsFrictionCompensatoryFactor = 1.0;


	m_dropDataCount = 0;
	m_actualJointPositions = jointPositions;
	m_actualJointVel.assign(m_NumJoints, 0.0);


	m_loopJointPosition.assign(21, jointPositions);
	m_loopSensedCurrent.assign(21, jointPositions);
	//setDynamicsLinearParameters(para);
	//setPayloadMassProperties(m, cop);

	b_isReadyToCloseMode = false;
	return EcTrue;
}

EcBoolean CHansCollaborativeAlgorithm::updateStateEstimates
(
	const EcRealVector& jointPositions,
	const EcRealVector& motorCurrents,
	const EcReal& currentTime
)
{
	m_currentTime += m_updateTimePeriod;
	if (!m_IsInitialized)
	{
		return EcFalse;
	}

	EcBoolean retVal = m_StateEstimator.estimateStates(
		jointPositions,
		m_FilteredJointPositions,
		m_FilteredJointVelocities,
		m_FilteredJointAccelerations
	);

	retVal &= m_StateEstimator.filterMotorCurrents(motorCurrents, m_FilteredMotorCurrents);

	return retVal;
}

EcBoolean CHansCollaborativeAlgorithm::updateJointTemperatures
(
	const EcRealVector& jointTemperatures
)
{
	m_jointTemperatures = jointTemperatures;
	return true;
}

EcBoolean CHansCollaborativeAlgorithm::updateStateEstimates
(
	const EcRealVector& commandJointPositions,
	const EcRealVector& jointPositions,
	const EcRealVector& motorCurrents,
	const EcReal& currentTime
)
{
	m_currentTime += m_updateTimePeriod;
	if (!m_IsInitialized)
	{
		return EcFalse;
	}


	m_loopJointPosition[20] = jointPositions;
	m_loopSensedCurrent[20] = motorCurrents;
	//循环保存更新的位置；
	for (int i = 0; i < 20; i++)
	{
		m_loopJointPosition[i] = m_loopJointPosition[i + 1];
		m_loopSensedCurrent[i] = m_loopSensedCurrent[i + 1];
	}


	// log: 当速度非零时，若出现实际位置和前一周期的数值相同，则打印语句；			后续应该增加约束，只允许有两个周期是这样；
	bool warningStatus = false;
	/**/
	for (int i = 0; i < m_NumJoints; i++)
	{
		if (fabs(m_FilteredJointVelocities[i]) > 0.087							// 5°/s;
			&& fabs(m_actualJointPositions[i] - jointPositions[i]) < 0.0000001)
		{
			warningStatus = true;
			m_dropDataCount++;
			break;
		}
	}

	if (m_dropDataCount > 8)
	{
		std::cout << "warning: lost too many data." << std::endl;
		m_StateEstimator.initialize(jointPositions);
		m_dropDataCount = 0;
	}

	// 如果存在实际位置没有更新的情况，使用差分的方法；    有可能存在连续多个周期丢失；
	EcRealVector tempJointPositions(m_NumJoints);
	if (warningStatus)
	{
		EcReal t2 = m_updateTimePeriod * m_updateTimePeriod;
		for (int i = 0; i < m_NumJoints; i++)
			tempJointPositions[i] = m_actualJointPositions[i] + m_FilteredJointVelocities[i] * m_updateTimePeriod * m_dropDataCount + m_FilteredJointAccelerations[i] * t2 * m_dropDataCount;
	}
	else
	{
		tempJointPositions = jointPositions;
		m_dropDataCount = 0;
	}

	m_actualJointPosition = jointPositions;
	// 计算关节的关节差分速度
	for (int i = 0; i < m_NumJoints; i++)
		m_actualJointVel[i] = (tempJointPositions[i] - m_actualJointPositions[i]) / m_updateTimePeriod;

	//std::cout << "update:"<<m_actualJointVel[0]<<"," << tempJointPositions[0] << "," << m_actualJointPositions[0] << "," << m_updateTimePeriod << std::endl;
	m_actualJointPositions = jointPositions;

	EcBoolean retVal = m_StateEstimator.estimateStates(
		tempJointPositions,
		m_FilteredJointPositions,
		m_FilteredJointVelocities,
		m_FilteredJointAccelerations
	);

	// 以命令位置作为状态估计；
	retVal = m_commandStateEstimator.estimateStates(
		commandJointPositions,
		m_FilteredCommandJointPositions,
		m_FilteredCommandJointVelocities,
		m_FilteredCommandJointAccelerations
	);

	retVal &= m_StateEstimator.filterMotorCurrents(motorCurrents, m_FilteredMotorCurrents);
	getSensedTorques(motorCurrents, m_sensedRawTorque);

	return retVal;
}


EcBoolean CHansCollaborativeAlgorithm::checkForCollision
(
	EcBooleanVector& jointCollisionStatus,
	ENCollisionType& type
)
{
	if (!m_IsInitialized)
	{
		return EcFalse;
	}

	/*
	// 修改为命令参数；可以减少滤波参数，减少加速度的相位滞后；
	EcBoolean retVal = calculateDisturbanceTorques(m_FilteredMotorCurrents,
		m_FilteredCommandJointPositions,
		m_FilteredCommandJointVelocities,
		m_FilteredCommandJointAccelerations,
		m_SensedJointTorques,
		m_EstimatedJointTorques,
		m_DisturbanceJointTorques);
	*/
	// 修改为命令参数；可以减少滤波参数，减少加速度的相位滞后；
	EcBoolean retVal = calculateDisturbanceTorques(m_FilteredMotorCurrents,
		m_FilteredJointPositions,
		m_FilteredJointVelocities,
		m_FilteredJointAccelerations,
		m_SensedJointTorques,
		m_EstimatedJointTorques,
		m_DisturbanceJointTorques);

	//动量观测器  || 使用命令速度和加速度有一些超前，使用实际速度和加速度试试
	m_momentumObserver->updateStateEstimates(m_actualJointPositions,
		m_FilteredJointVelocities,
		m_FilteredJointAccelerations,
		m_SensedJointTorques);

	retVal &= m_StateEstimator.filterDisturbanceTorques(m_DisturbanceJointTorques, m_DisturbanceJointTorques);
	updateDynamicsCollisionStopThreshold(m_CollisionStopDynamicsThresholds);

	retVal &= updateCollisionStopStatus(m_CollisionStopDynamicsThresholds, m_DisturbanceJointTorques, jointCollisionStatus);

	bool normalStatus = true;
	// 力矩偏差判断和动量观测器
	if (b_isUsingMomentumObserver)
	{
		EcBooleanVector momentumCollisionStatus(6);
		m_momentumObserver->getObserverTorqueDisturbances(m_observerTorques);
		m_momentumObserver->getJointCollisionState(momentumCollisionStatus);
		for (EcSizeT i = 0; i < m_NumJoints; i++)
		{
			if (momentumCollisionStatus[i])
				type = collision_Observer;
			if (jointCollisionStatus[i])
				type = collision_deviation;

			jointCollisionStatus[i] = jointCollisionStatus[i] || momentumCollisionStatus[i];
		}
	}

	/**/
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (jointCollisionStatus[i] && m_currentTime > 20)
		{
			normalStatus = false;

			std::cout << "Collision Joint:" << i + 1 << ",pvased,th,type:" << m_FilteredJointPositions[i] << "," << m_FilteredJointVelocities[i] << "," <<
				m_FilteredJointAccelerations[i] << "," << m_SensedJointTorques[i] << "," << m_EstimatedJointTorques[i] << "," << m_DisturbanceJointTorques[i] <<
				"," << m_CollisionStopDynamicsThresholds[i] << "," << m_CollisionStopThresholds[i] << "," << type << std::endl;

			std::cout << "update jointPosition:";
			for (int k = 0; k < 20; k++)
			{
				std::cout << m_loopJointPosition[k][i] << ",";
			}
			std::cout << std::endl;
			std::cout << "update current:";
			for (int k = 0; k < 20; k++)
			{
				std::cout << m_loopSensedCurrent[k][i] << ",";
			}
			std::cout << std::endl;
			break;
		}
	}

	if (!normalStatus)
		m_currentTime = 0;			// 发生碰撞后，将时间清零；

	m_collisionStopStatus = jointCollisionStatus;
	return retVal;
}


void CHansCollaborativeAlgorithm::updateDynamicsCollisionStopThreshold
(
	EcRealVector& enchanceDisturbJointTorques
)
{
	// 1. 减小摩擦力换向带来的影响
	// 2. 考虑负载质量+运行速度带来的阈值变化
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (fabs(m_FilteredJointVelocities[i]) < 0.03)
		{
			enchanceDisturbJointTorques[i] = m_CollisionStopThresholds[i] + m_CoulombFriction[i];
		}
		else {
			enchanceDisturbJointTorques[i] = m_CollisionStopThresholds[i];
		}
		enchanceDisturbJointTorques[i] += m_massThresholdTorques[i];
	}
}



EcBoolean CHansCollaborativeAlgorithm::setFilterTimeConstant
(
	const EcRealVector& jointsPosition,
	const EcRealVector& filterTimeConstant,
	const EcRealVector& disturbFilterTimeConstant
)
{
	m_commandStateEstimator.setUniteFilterTimeConstants(jointsPosition, filterTimeConstant, disturbFilterTimeConstant);
	return m_StateEstimator.setUniteFilterTimeConstants(jointsPosition, filterTimeConstant, disturbFilterTimeConstant);
}

void CHansCollaborativeAlgorithm::setRobotMountingAngles
(
	const EcReal& rotation,
	const EcReal& tilt
)
{
	std::cout << "mounting:" << rotation << "," << tilt << std::endl;
	m_baseMountingRotation = rotation;
	m_baseMountingTilt = tilt;							//Todo:用于计算等价重力时，是否应当是针对重力的旋转，而非基座的旋转；

	KDL::Rotation gRotation, R1, R2;
	/*
	R1 = KDL::Rotation::RPY(0, 0, m_baseMountingRotation);
	R2 = KDL::Rotation::RPY(0, m_baseMountingTilt, 0);
	gRotation = R1 * R2	; //  右乘，绕旋转后的轴旋转
	*/

	// need to be inversed.
	R1 = KDL::Rotation::RPY(0, 0, m_baseMountingRotation);
	R2 = KDL::Rotation::RPY(0, m_baseMountingTilt, 0);
	gRotation = (R2 * R1).Inverse();

	KDL::Vector gVector = gRotation * m_gravity;
	m_gx = gVector[0];
	m_gy = gVector[1];
	m_gz = gVector[2];
	//std::cout << "MountingRad:" << m_baseMountingRotation << "," << m_baseMountingTilt << std::endl;

	m_dynBase->setGravityVector(m_gx, m_gy, m_gz);
	m_momentumObserver->setGravityAcceleration(m_gx, m_gy, m_gz);
}

EcBoolean CHansCollaborativeAlgorithm::setActutorMaxCurrentLimits
(
	const EcRealVector& maxCurrents
)
{
	if (m_NumJoints != maxCurrents.size())
	{
		return false;
	}
	std::cout << "(cobot)Max Actuator Current:" << maxCurrents[0] << "," << maxCurrents[1] << "," << maxCurrents[2] << "," << maxCurrents[3] << "," << maxCurrents[4] << "," << maxCurrents[5] << "," << std::endl;
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (maxCurrents[i] < 0.1 || maxCurrents[i]>100) {
			std::cout << "************** The setting of actutor max current parameters occurs ERROR! over limit." << std::endl;
			return false;
		}
	}
	m_MaxActutorCurrents = maxCurrents;
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		m_MaxActutorTorques[i] = m_AdjustedTorqueConstants[i] * m_MaxActutorCurrents[i] * 1.5;		// 放大1.5倍；
	}
	return true;
}

void CHansCollaborativeAlgorithm::setGravityVector
(
	const EcRealVector& gravity
)
{
	m_gx = gravity[0];
	m_gy = gravity[1];
	m_gz = gravity[2];
	m_momentumObserver->setGravityAcceleration(m_gx, m_gy, m_gz);
	std::cout << "Gravity Vector:" << m_gx << "," << m_gy << "," << m_gz << std::endl;
}

void CHansCollaborativeAlgorithm::setDynamicsLinearParameters
(
	const EcRealVector& params
)
{
	m_DynamicsParameters.assign(params.begin(), params.end());
	m_DynamicsLinearParameters = m_DynamicsParameters;

	//std::cout << "dynamicsParams:" <<params.size() << std::endl;
	//for (int i = 0; i < params.size(); i++)
	//{
	//	std::cout << params[i] << std::endl;
	//}

	setPayloadMassProperties(m_payloadMass, m_centerofMass);
	setFrictionModel(params);
}

EcBoolean CHansCollaborativeAlgorithm::setPayloadMassProperties
(
	const EcReal& mass,
	const EcRealVector& centerofMass
)
{
	if (m_NumJoints < 1)
	{
		return EcFalse;
	}
	std::cout << "payload :" << mass << "," << centerofMass[0] << "," << centerofMass[1] << "," << centerofMass[2] << std::endl;

	m_prePayloadMass = m_payloadMass;
	b_newPayLoadStatus = true;
	m_payloadCount = 1;

	m_payloadMass = mass;
	m_centerofMass = { centerofMass[0], centerofMass[1], centerofMass[2] };
	EcReal x, y, z, lx, ly, lz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
	x = centerofMass[0];
	y = centerofMass[1];
	z = centerofMass[2];

	EcReal length = sqrt(x * x + y * y + z * z);

	Ixx = 0;	Iyy = 0;	Izz = 0;	Ixy = 0;	Ixz = 0;	Iyz = 0;
	lx = mass * x;	ly = mass * y;	lz = mass * z;
	EcRealVector payloadPara = { Ixx, Ixy, Ixz, Iyy, Iyz, Izz, lx, ly, lz, mass };

	for (EcU32 ii = 0; ii < 10; ii++)
	{
		m_DynamicsLinearParameters[13 * (m_NumJoints - 1) + ii] = m_DynamicsParameters[13 * (m_NumJoints - 1) + ii] + payloadPara[ii];
	}

	m_momentumObserver->setDynamicsParameters(m_DynamicsLinearParameters);

	// 增加质量导致的阈值变化
	m_massThresholdTorques[0] = mass * (1.0 + length * 4);
	m_massThresholdTorques[1] = mass * (1.2 + length * 6);
	m_massThresholdTorques[2] = mass * (0.7 + length * 3);
	m_massThresholdTorques[3] = mass * (0.4 + length * 3);
	m_massThresholdTorques[4] = mass * (0.3 + length * 3);
	m_massThresholdTorques[5] = mass * (0.3 + length * 3);
	m_momentumObserver->setDynamicsFactorThreshold(m_massThresholdTorques);

	EcBoolean ret = calculateMaxRectifyEstimateJointTorques();
	return EcTrue;
}

void CHansCollaborativeAlgorithm::setFrictionModel
(
	const EcRealVector& params
)
{
	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		m_ViscousFrictionCoefficient[ii] = params[ii * 13 + 11];
		m_CoulombFriction[ii] = params[ii * 13 + 12];
	}
	m_frictionModel.setFrictionParams(m_CoulombFriction, m_ViscousFrictionCoefficient);
	std::cout << "friction:" << m_CoulombFriction[0] << "," << m_CoulombFriction[1] << "," << m_CoulombFriction[2] << "," << m_CoulombFriction[3] << "," << m_CoulombFriction[4] << "," << m_CoulombFriction[5] << std::endl;

}


EcBoolean CHansCollaborativeAlgorithm::updateCollisionStopStatus
(
	const EcRealVector& torqueThresholds,
	const EcRealVector& disturbanceTorques,
	EcBooleanVector& collisionStopStatus
)
{
	const EcU32 numJoints = disturbanceTorques.size();

	if (
		!m_IsInitialized ||
		torqueThresholds.size() != numJoints
		)
	{
		return EcFalse;
	}


	for (EcU32 ii = 0; ii < numJoints; ++ii)
	{
		if (std::fabs(disturbanceTorques[ii]) >= torqueThresholds[ii])
		{
			collisionStopStatus[ii] = EcTrue;
		}
		else {
			collisionStopStatus[ii] = EcFalse;
		}
	}
	return EcTrue;
}

//------------------------------------------------------------------------------
EcBoolean CHansCollaborativeAlgorithm::setCollisionStopThresholds
(
	const EcRealVector& collisionStopThresholds
)
{
	if (collisionStopThresholds.size() != m_NumJoints)
		return false;

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (collisionStopThresholds[i] < 1.0 || collisionStopThresholds[i] > 10000000000.0)
		{
			std::cout << "************** The setting of collision stop thresholds occurs ERROR!" << std::endl;
			return false;
		}
	}
	m_CollisionStopThresholds = collisionStopThresholds;
	std::cout << "dTh:" << m_CollisionStopThresholds[0] << "," << m_CollisionStopThresholds[1] << "," << m_CollisionStopThresholds[2] << "," << m_CollisionStopThresholds[3] << "," << m_CollisionStopThresholds[4] << "," << m_CollisionStopThresholds[5] << std::endl;
	return true;
}

EcBoolean CHansCollaborativeAlgorithm::setCollisionStopInMomentumThresholds
(
	const EcRealVector& collisionStopThresholds
)
{
	if (collisionStopThresholds.size() != m_NumJoints)
		return false;

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (collisionStopThresholds[i] < 1.0 || collisionStopThresholds[i] > 10000000000.0)
		{
			std::cout << "************** The setting of collision stop thresholds occurs ERROR!" << std::endl;
			return false;
		}
	}

	/**/
	b_isUsingMomentumObserver = EcTrue;
	m_MomentumCollisionThresholds = collisionStopThresholds;
	m_momentumObserver->setCollisionThreshold(collisionStopThresholds);
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		m_driveBackDisturbancesThreshold[i] = 0.5 * collisionStopThresholds[i];
	}

	std::cout << "mTh:" << collisionStopThresholds[0] << "," << collisionStopThresholds[1] << "," << collisionStopThresholds[2] << "," << collisionStopThresholds[3] << "," << collisionStopThresholds[4] << "," << collisionStopThresholds[5] << std::endl;


	return true;
	// 测试弹性摩擦模型，调整弹性系数；
	//m_KVector = collisionStopThresholds;
	//std::cout<<"KVector(Collision Threshold):" << m_KVector[0] << "," << m_KVector[1] << "," << m_KVector[2] << ","
	//	<< m_KVector[3] << "," << m_KVector[4] << "," << m_KVector[5] << std::endl;
}

EcBoolean CHansCollaborativeAlgorithm::setActutorTorqueConstants
(
	const EcRealVector& torqueConstant,
	const EcRealVector& gearRatio,
	const EcRealVector& maxEfficiency
)
{
	if (torqueConstant.size() != m_NumJoints || gearRatio.size() != m_NumJoints || maxEfficiency.size() != m_NumJoints)
	{
		std::cout << "************** The setting of actutor toqrue constants parameters occurs ERROR!" << "the number of size is not equal" << std::endl;
		return false;
	}

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if ((torqueConstant[i] < 0.05 || torqueConstant[i] >0.5) ||
			(gearRatio[i] < 50 || gearRatio[i]>300) ||
			(maxEfficiency[i] < 0.1 || maxEfficiency[i] >1.1)
			)
		{
			std::cout << "************** The setting of actutor toqrue constants parameters occurs ERROR!" << std::endl;
			std::cout << "index: " << i << " ," << torqueConstant[i] << "," << gearRatio[i] << "," << maxEfficiency[i] << std::endl;
			return false;
		}
	}

	m_torqueConstant = torqueConstant;
	m_gearRatio = gearRatio;
	m_maxEfficiency = maxEfficiency;

	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		m_AdjustedTorqueConstants[ii] = m_torqueConstant[ii] * m_gearRatio[ii] * m_maxEfficiency[ii];
	}

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		m_MaxActutorTorques[i] = m_AdjustedTorqueConstants[i] * m_MaxActutorCurrents[i];
	}
	std::cout << "torqueConstants:" << m_AdjustedTorqueConstants[0] << "," << m_AdjustedTorqueConstants[1] << "," << m_AdjustedTorqueConstants[2] << "," << m_AdjustedTorqueConstants[3] << "," << m_AdjustedTorqueConstants[4] << "," << m_AdjustedTorqueConstants[5] << std::endl;
	return true;
}

void CHansCollaborativeAlgorithm::setActutorDampConstants
(
	const EcRealVector& damp
)
{
	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		m_dampEfficiency[ii] = 1.0 - damp[ii];
	}
	m_dampSetting = damp;
}


EcBoolean CHansCollaborativeAlgorithm::setRobotDHParameters
(
	const EcRealVector& kinematcisParam
)
{
	m_kinParams = kinematcisParam;
	m_dynBase->setRobotDHParameters(kinematcisParam);
	m_momentumObserver->setKinematicsParameters(kinematcisParam);

	for (EcSizeT i = 0; i < 4; i++) {
		if (kinematcisParam[i] < 0.01 || kinematcisParam[i]>1000.0)
		{
			std::cout << "************** The setting of kinematics parameters occurs ERROR!" << std::endl;
			std::cout << "index: " << i << " : " << kinematcisParam[i] << std::endl;
			return false;
		}
	}
	m_d1 = kinematcisParam[0];
	m_d4 = kinematcisParam[1];
	m_d6 = kinematcisParam[2];
	m_a2 = kinematcisParam[3];

	//Calculate joint limit against force equivalent
	m_jointLimitAgainstForceEquivalent[0] = fabs(1000 * m_a2);
	m_jointLimitAgainstForceEquivalent[1] = fabs(1000 * (m_a2 + m_d4));
	m_jointLimitAgainstForceEquivalent[2] = fabs(1000 * (m_d4 + m_a2));
	m_jointLimitAgainstForceEquivalent[3] = fabs(1000 * 0.20);
	m_jointLimitAgainstForceEquivalent[4] = fabs(1000 * m_d6);
	m_jointLimitAgainstForceEquivalent[5] = fabs(1000 * 0.15);

	return false;
}


EcBoolean CHansCollaborativeAlgorithm::setCollaborativeJointSpaceLimits
(
	const EcRealVector& upperJointLimits,
	const EcRealVector& lowerJointLimits
)
{
	if (upperJointLimits.size() != m_NumJoints || lowerJointLimits.size() != m_NumJoints)
		return false;
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (upperJointLimits[i] < lowerJointLimits[i])
			return false;
	}

	m_UpperJointLimits = upperJointLimits;
	m_LowerJointLimits = lowerJointLimits;

	for (EcU32 ii = 0; ii < m_UpperJointLimits.size(); ++ii)
	{
		m_jointRange[ii] = fabs(m_UpperJointLimits[ii] - m_LowerJointLimits[ii]);
		if (m_jointRange[ii] > 2 * KDL::PI)
		{
			m_slowDownBoundary[ii] = 0.025 * m_jointRange[ii];
		}
		else {
			m_slowDownBoundary[ii] = 0.05 * m_jointRange[ii];
		}
		m_slowDownBoundary[ii] = (m_slowDownBoundary[ii] < 0.14) ? m_slowDownBoundary[ii] : 0.14;		//当关节转角存在超多圈时，确保边界不过大；
		m_stopBoundary[ii] = 0.05 * m_slowDownBoundary[ii];
	}
	return true;
}


void CHansCollaborativeAlgorithm::setAssistiveModeCollisionStopThresholds
(
	const EcRealVector& assistiveModeCollisionStopThresholds
)
{
	if (assistiveModeCollisionStopThresholds.size() != m_NumJoints)
		return;
	m_AssistiveModeCollisionStopThresholds = assistiveModeCollisionStopThresholds;
}

void CHansCollaborativeAlgorithm::setFrictionCompensatoryFactor
(
	const EcRealVector& frictionCompensatoryFactor
)
{
	m_frictionCompensatoryFactor = frictionCompensatoryFactor;
	std::cout << "CompensateFrictionCoeff:" << frictionCompensatoryFactor[0] << ", " << frictionCompensatoryFactor[1] << ", " << frictionCompensatoryFactor[2] << ", " << frictionCompensatoryFactor[3] << ", " << frictionCompensatoryFactor[4] << ", " << frictionCompensatoryFactor[5] << std::endl;
	m_frictionModel.setCompensateFactor(frictionCompensatoryFactor);
}

void CHansCollaborativeAlgorithm::setStartCompensateFrictionFactor(
	const EcRealVector& compensateFactor)
{
	std::cout << "startCompensateFrictionFactor:" << compensateFactor[0] << "," << compensateFactor[1] << "," << compensateFactor[2] << "," << compensateFactor[3] << "," << compensateFactor[4] << "," << compensateFactor[5] << std::endl;

	m_startCompensateFrictionFactor = compensateFactor;
	m_frictionModel.setStartCompensateFrictionFactor(compensateFactor);
}

void CHansCollaborativeAlgorithm::setFrictionParamsWithTemperature(
	const EcRealVector& factor)
{
	if (factor.size() != m_NumJoints)
		return;
	std::cout << "set temperature params:" << m_friTemperaturesParams[0] << "," << m_friTemperaturesParams[1] << "," << m_friTemperaturesParams[2] << "," <<
		m_friTemperaturesParams[3] << "," << m_friTemperaturesParams[4] << "," << m_friTemperaturesParams[5] << std::endl;

	m_friTemperaturesParams = factor;
}


EcBoolean CHansCollaborativeAlgorithm::getEstimatedState
(
	EcRealVector& jointPostions,
	EcRealVector& jointVelocities,
	EcRealVector& jointAccelerations,
	EcRealVector& motorCurrents,
	EcRealVector& sensedTorques,
	EcRealVector& estimatedTorques,
	EcRealVector& disturbanceTorques
)
{
	if (
		!m_IsInitialized ||
		m_FilteredJointPositions.size() != m_NumJoints ||
		m_FilteredJointVelocities.size() != m_NumJoints ||
		m_FilteredJointAccelerations.size() != m_NumJoints ||
		m_FilteredMotorCurrents.size() != m_NumJoints ||
		m_SensedJointTorques.size() != m_NumJoints ||
		m_EstimatedJointTorques.size() != m_NumJoints ||
		m_DisturbanceJointTorques.size() != m_NumJoints
		)
	{
		return EcFalse;
	}

	jointPostions = m_FilteredJointPositions;
	jointVelocities = m_FilteredJointVelocities;
	jointAccelerations = m_FilteredJointAccelerations;
	motorCurrents = m_FilteredMotorCurrents;

	/*
	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		sensedTorques[ii] = motorCurrents[ii] * m_AdjustedTorqueConstants[ii];			// 接口获取的检测力矩不进行damp消除，避免影响辨识；
	}
	*/
	//sensedTorques = m_SensedJointTorques;
	getSensedTorques(motorCurrents, sensedTorques);				// 修改为校正后的力矩数据；

	estimatedTorques = m_EstimatedJointTorques;
	//disturbanceTorques = m_DisturbanceJointTorques;
	//用于调试动量观测
	disturbanceTorques = m_observerTorques;
	//Todo: 将motorcurrent用于保存高通滤波前的动量观测值；
	//m_momentumObserver->getRawObserverTorqueDisturbances(motorCurrents);
	return EcTrue;
}

EcBoolean CHansCollaborativeAlgorithm::getSensedTorques
(
	const EcRealVector& motorCurrents,
	EcRealVector& sensedTorques
)
{
	if (
		!m_IsInitialized ||
		motorCurrents.size() != m_NumJoints
		)
	{
		return EcFalse;
	}

	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		sensedTorques[ii] = motorCurrents[ii] * m_AdjustedTorqueConstants[ii];
		sensedTorques[ii] -= fabs(sensedTorques[ii]) * sign(m_FilteredJointVelocities[ii]) * m_dampEfficiency[ii];
	}

	return EcTrue;
}

void CHansCollaborativeAlgorithm::getFlexibleCompensateGravityTorque
(
	const EcRealVector& jointPosition,
	EcRealVector& gravTorque
)
{
	EcRealVector gravitationalTorques(m_NumJoints);
	m_dynBase->calculateGravityJointTorques(jointPosition, m_DynamicsLinearParameters, gravitationalTorques);
	gravTorque = gravitationalTorques;
}

EcBoolean CHansCollaborativeAlgorithm::calculateEstimateTorques
(
	const EcRealVector& jointPositions,
	const EcRealVector& jointVelocities,
	const EcRealVector& jointAccelerations,
	EcRealVector& estimatedTorques
)
{
	if (!m_IsInitialized)
	{
		return EcFalse;
	}

	m_dynBase->calculateEstimateJointToqrues(jointPositions, jointVelocities, jointAccelerations, m_DynamicsLinearParameters, estimatedTorques);
	return EcTrue;
}


EcBoolean CHansCollaborativeAlgorithm::calculateDisturbanceTorques
(
	const EcRealVector& motorCurrents,
	const EcRealVector& jointPositions,
	const EcRealVector& jointVelocities,
	const EcRealVector& jointAccelerations,
	EcRealVector& sensedTorques,
	EcRealVector& estimatedTorques,
	EcRealVector& disturbanceTorques
)
{
	if (
		!m_IsInitialized ||
		jointPositions.size() != m_NumJoints ||
		jointVelocities.size() != m_NumJoints ||
		jointAccelerations.size() != m_NumJoints ||
		motorCurrents.size() != m_NumJoints
		)
	{
		std::cout << "Error: Initialization is false" << std::endl;
		return EcFalse;
	}

	getSensedTorques(motorCurrents, sensedTorques);
	EcBoolean retVal = m_dynBase->calculateEstimateJointToqrues(jointPositions, jointVelocities, jointAccelerations, m_DynamicsLinearParameters, estimatedTorques);


	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		disturbanceTorques[ii] = sensedTorques[ii] - estimatedTorques[ii];
	}

	return retVal;
}

EcReal CHansCollaborativeAlgorithm::calculateJointFriction
(
	const EcU32 jointIndex,
	const EcReal jointVelocity
)
{
	return m_CoulombFriction[jointIndex] * sign(jointVelocity) + m_ViscousFrictionCoefficient[jointIndex] * jointVelocity;
}

EcBoolean CHansCollaborativeAlgorithm::calculateCompensateCoulombFriction
(
	const EcRealVector& jointVelocity,
	EcRealVector& coulombFriction
)
{
	// 计算弹性摩擦力，用于补偿超低速下的摩擦力
	calculateCoulombFriction(m_actualJointPositions, m_calcCoulombFriction);		//未启用

	EcReal sumVel = 0;
	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		sumVel += fabs(jointVelocity[ii]);
		if (fabs(jointVelocity[ii]) < m_zeroVelocityThresholds[ii])			//1.11°
		{
			if (m_stopStateTimeCounts >= m_stopGenerateFrictCounts || b_isDriveBackMode)
			{
				coulombFriction[ii] = 0;
			}
			else {
				// 关节产生实际的位置抖动：一段时间内（3s)的运动位置远大于位移；
				coulombFriction[ii] = generateVibrationSignal(ii, m_CoulombFriction[ii]) * 1.0;

			}
			//coulombFriction[ii] += m_calcCoulombFriction[ii] * 1.2;
		}
		else if (fabs(jointVelocity[ii]) < m_lowVelocityThresholds[ii])
		{
			coulombFriction[ii] = pow(fabs(jointVelocity[ii]) / m_lowVelocityThresholds[ii], 0.30) * sign(jointVelocity[ii]) * m_CoulombFriction[ii];
		}
		else
		{
			coulombFriction[ii] = sign(jointVelocity[ii]) * m_CoulombFriction[ii];
		}

		coulombFriction[ii] *= (1 + fabs(jointVelocity[ii]) / m_maxJointVelocitiesInAssistiveMode[ii]);		// 按速度相应提升静摩擦力的补偿比例；
	}
	if (sumVel < m_zeroVelocityThresholds[0])
	{
		m_stopStateTimeCounts += m_updateTimePeriod;
	}
	else {
		m_stopStateTimeCounts = 0;
	}

	return EcTrue;
}

EcReal CHansCollaborativeAlgorithm::generateVibrationSignal
(
	const EcU32 jointIndex,
	const EcReal& friction
)
{
	return m_VibrationAmplitude[jointIndex] * friction * (sin(m_vibrationPeriod[jointIndex] * m_currentTime));
}

void CHansCollaborativeAlgorithm::setCollisionDriveBackMode
(
	const ENDriveBackMode drivebackMode
)
{
	std::cout << "Start Collision Drive Back Mode2:"<< drivebackMode<<","<< m_actualJointPositions[1] << std::endl;
	m_drivebackMode = drivebackMode;
	m_currentDriveBackTime = 0.0;
	m_driveBackDuranceTime = 0.0;


	m_driveBackStatusVector.assign(6, status_statePosition);
	m_overZeroVelCount.assign(m_NumJoints, 0);

	m_driveBackTorques = m_zeroVector;
	b_isDriveBackAssistiveMode = EcTrue;
	b_isDriveBackMode = EcTrue;
	b_isReadyToCloseMode = false;

	m_driveBackStartPosition = m_actualJointPosition;
	m_startCollisionJointVel = m_FilteredJointVelocities;				// 有可能存在位置丢帧的情况，导致速度估计为零；所以使用滤波速度
	m_actualPreviousJointVel = m_actualJointVel;

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (fabs(m_DisturbanceJointTorques[i]) > m_driveBackDisturbancesThreshold[i] && fabs(m_startCollisionJointVel[i]) > 0.02)				// 0.5倍的动量碰撞检测阈值；
			m_driveBackCollisionStopStatus[i] = EcTrue;
		else
			m_driveBackCollisionStopStatus[i] = EcFalse;
	}

	m_driveBackStatus = m_driveBackCollisionStopStatus;			// step1: driveback status for joints
	for (EcSizeT i = 0; i < m_NumJoints; i++)
		if (m_driveBackCollisionStopStatus[i])
			m_driveBackStatusVector[i] = status_strongDrive;

	std::cout << "startdbmode:" << m_driveBackStatusVector[0] << "," << m_driveBackStatusVector[1] << "," << m_driveBackStatusVector[2] << "," << m_driveBackStatusVector[3] << "," << m_driveBackStatusVector[4] << "," << m_driveBackStatusVector[5] << std::endl;


	m_startDriveBackJointPosition = m_actualJointPositions;

	m_currentAssistiveTime = AssistiveModeStartTime * 10;		//reset safe start assistive mode
	// 弹性摩擦相关参数
	m_jointMPosition = m_actualJointPositions;
	m_jointSidePosition = m_actualJointPositions;
}


void CHansCollaborativeAlgorithm::setMotionLimitInDriveBack(const EcReal distanceLimit, const EcReal timeDuration)
{
	m_allowBackDistance = distanceLimit;
	if (m_allowBackDistance < 0.087)
		m_allowBackDistance = 0.087;
	m_allowMotionDuration = timeDuration;
	if (m_allowMotionDuration < 0.2)
		m_allowMotionDuration = 0.2;
	std::cout << "set allow driveback in motion limit:" << m_allowBackDistance << ", " << m_allowMotionDuration << std::endl;
}

EcBoolean CHansCollaborativeAlgorithm::checkCloseDriveBackMode
(
)
{
	//step：1. 反向驱动； 2. 摩擦力制动；3. 零力示教；4. 结束零力示教；
	if (b_isDriveBackAssistiveMode)
	{
		for (int i = 0; i < numofJoints; i++)
		{
			if (m_collisionStopStatus[i])
			{
				m_collisionStopStatus[i] = false;
			}
		}
		//std::cout << "time of drivebakce mode:" << m_currentDriveBackTime << std::endl;
		m_currentDriveBackTime += m_updateTimePeriod;
		if (m_drivebackMode == mode_ForceFree)
		{
			if (m_currentDriveBackTime > DriveBackClosedTime)	//大于设定时间后，停止反向驱动；
			{
				b_isDriveBackMode = EcFalse;						// 是否补偿摩擦力的标志位；
				m_driveBackStatus = b_falseVector;				  //1.结束反向驱动；

				// 若当前关节速度接近于零，则退出零力示教模式；
				EcReal sumVel = 0;
				for (EcSizeT i = 0; i < m_NumJoints; i++)
					sumVel += fabs(m_FilteredJointVelocities[i]);

				if (sumVel < DriveBackThresholdVelocity)
				{
					m_driveBackDuranceTime += m_updateTimePeriod;
				}
				else
					m_driveBackDuranceTime = 0.0;

				if (m_driveBackDuranceTime > MaxDriveBackEnduranceTime)	// 如果速度接近于零持续一段时间；
				{
					b_isDriveBackAssistiveMode = EcFalse;
					std::cout << "Close driveback mode.." << std::endl;
					return EcTrue;										// 4.结束零力示教
				}
			}
		}

		// 6个关节，需要每个关节都要判断是否完成这这三个阶段；
		EcBoolean driveBackFlag = false;
		EcBoolean errorStatus = false;
		EcReal coeff = 0;
		EcReal coeffCoulomb = 0;
		EcReal coeffViscous = 0;

		if (m_drivebackMode == mode_LimitMotion)
		{
			// 反弹模式，处理三个阶段：1. 反弹至速度接近于零； 2. 离开速度为零的位置小于0.5°； 3. 制动，使得关节位置小于1.0°；4. 稳定停止，取消摩擦力补偿；
			// 通过switch来控制，还需要一个时间变量，避免陷入一个状态无法离开；
			for (EcSizeT i = 0; i < m_NumJoints; i++)
			{
				if (m_driveBackStatus[i])
				{
					switch (m_driveBackStatusVector[i])
					{
					case status_strongDrive:
						if (m_actualJointVel[i] * m_startCollisionJointVel[i] <= 0 || m_currentDriveBackTime > 0.06)
						{
							m_driveBackStatusVector[i] = status_awayPosition;
							m_driveBackStartPosition[i] = m_actualJointPosition[i];
						}
						else																 // 速度换向这个周期，不更新反弹力，避免受到速度换向干扰；
						{
							m_driveBackTorques[i] = 15.0 * m_frictionModel.calculateFrictionTorque(i, m_actualJointVel[i]);				// 制动，表现为反向补偿摩擦力，10倍；
						}

						break;
					case status_awayPosition:				// 确保驱动力和碰撞起始速度反向；   // 反弹力度和反弹距离成反比（开始大，后逐渐衰减）
						coeff = (driveBackJointPosition_away - fabs(m_actualJointPosition[i] - m_driveBackStartPosition[i])) / driveBackJointPosition_away;
						if (fabs(m_actualJointVel[i]) > 0.3)		// 限制关节速度不大于20°/s
							m_driveBackTorques[i] = m_frictionModel.calculateFrictionTorque(i, m_actualJointVel[i], 1, 30);
						else
							m_driveBackTorques[i] = (5.0 + coeff * 15) * sign(m_startCollisionJointVel[i]) * fabs(m_frictionModel.calculateFrictionTorque(i, m_actualJointVel[i], 1, -6));// 驱动，正向补偿库伦摩擦力,反向粘性摩擦力矩，限制速度过快，逐渐衰减；

						if (fabs(m_actualJointPosition[i] - m_driveBackStartPosition[i]) > driveBackJointPosition_away || m_currentDriveBackTime > 0.12)			// 0.5°；
						{
							m_driveBackStatusVector[i] = status_controlPosition;
							if (m_actualJointVel[i] * m_startCollisionJointVel[i] > 0)		// 若远离阶段，速度仍然没有反向，则属于异常状态，中断反弹模式；
								errorStatus = true;
						}

						break;
					case status_controlPosition:
						m_driveBackTorques[i] = 1.0 * m_frictionModel.calculateFrictionTorque(i, m_actualJointVel[i], 1.2, 30);			// 制动，反向补偿摩擦力，特别是粘性；
						if (fabs(m_actualJointPosition[i] - m_driveBackStartPosition[i]) > driveBackJointPosition_control
							|| m_currentDriveBackTime > 0.2											// 1.0°；
							|| m_actualJointVel[i] * m_startCollisionJointVel[i] > 0)				// 已经制动为零	
							m_driveBackStatusVector[i] = status_stopMoving;


						break;
					case status_stopMoving:
						coeffCoulomb = fabs(m_actualJointVel[i]) > 0.1 ? 2.0 : 0;		// 速度在接近零（6°/s）时，取消摩擦力矩；避免过冲产生震荡；
						coeffViscous = fabs(m_actualJointVel[i]) > 0.1 ? 50.0 : 5;		// 速度在接近零（6°/s）时，取消摩擦力矩；避免过冲产生震荡；
						m_driveBackTorques[i] = 1.0 * m_frictionModel.calculateFrictionTorque(i, m_actualJointVel[i], coeffCoulomb, coeffViscous);			// 稳定，仅依赖粘性摩擦力矩，滑行停止；
						if (m_actualJointVel[i] * m_actualPreviousJointVel[i] <= 0)
							m_overZeroVelCount[i]++;


						if (fabs(m_actualJointVel[i]) < 0.005 || m_currentDriveBackTime > 0.8 || m_overZeroVelCount[i] >= 3)
							m_driveBackStatusVector[i] = status_statePosition;
						break;

					default:
					case status_statePosition:
						m_driveBackTorques[i] = 0;
						break;
					}

					//std::cout <<m_currentDriveBackTime << "," << i << "," << m_driveBackStatusVector[i] << ", " << m_driveBackTorques[i]<<","<<m_sensedRawTorque[i]
					//	<< ","<< m_actualJointPosition[i]<<","<< m_actualJointPosition[i] - m_driveBackStartPosition[i] << "," << m_actualJointVel[i] << std::endl;
				}

				if (m_driveBackStatusVector[i] != status_statePosition)		// 若还有关节不处于state状态，那么就是反弹模式；
				{
					driveBackFlag = true;
				}
			}
			//std::cout << "joint2:"<<m_currentDriveBackTime<<","<<m_sensedRawTorque[1]<<","<< m_driveBackStatusVector[1]<<"," << (m_actualJointPosition[1] - m_driveBackStartPosition[1]) * EcDEG2RAD << "," << m_actualJointVel[1] * EcDEG2RAD << std::endl;
			bool isOverJointMotionLimit = false;
			for (int i = 0; i < numofJoints; i++)
			{
				if (fabs(m_actualJointPosition[i] - m_driveBackStartPosition[i]) > m_allowBackDistance)
					isOverJointMotionLimit = true;
			}

			if (m_currentDriveBackTime > m_allowMotionDuration || isOverJointMotionLimit || (!driveBackFlag && m_currentDriveBackTime > 0.05) || errorStatus)
			{
				std::cout << "Close driveback mode2.0. Last time: " << m_currentDriveBackTime << ", driveBack:" << driveBackFlag << "," << errorStatus << "," << (m_actualJointPosition[0] - m_driveBackStartPosition[0]) * EcDEG2RAD << "," << (m_actualJointPosition[1] - m_driveBackStartPosition[1]) * EcDEG2RAD << "," << (m_actualJointPosition[2] - m_driveBackStartPosition[2]) * EcDEG2RAD << std::endl;
				b_isDriveBackAssistiveMode = EcFalse;
				return EcTrue;
			}
			m_actualPreviousJointVel = m_actualJointVel;


			/*
			for (EcSizeT i = 0; i < m_NumJoints; i++)
			{
				if (fabs(m_actualJointPositions[i] - m_startDriveBackJointPosition[i]) > m_allowBackDistance)
				{
					std::cout << "Close driveback mode. position: " << i << "," << fabs(m_actualJointPositions[i] - m_startDriveBackJointPosition[i]) << std::endl;
					b_isDriveBackAssistiveMode = EcFalse;
					return EcTrue;
				}
			}
			*/
		}

	}
	return EcFalse;
}

void CHansCollaborativeAlgorithm::setReadyToCloseAssistiveMode()
{
	std::cout << "ready to close assisitive mode. CurrentTime:"<< m_currentAssistiveTime <<","<<m_actualJointVel[0] << std::endl;
	b_isReadyToCloseMode = true;
	m_currentReadyToCloseAssistiveTime = 0.;
	m_viscousRatioForCloseAssistive = 10;
	m_startCloseAssistiveJointVel = m_actualJointVel;
	b_havedReverseJointVel.assign(6, false);
}



EcBoolean CHansCollaborativeAlgorithm::checkCompleteReadyToCloseAssisitiveMode()
{
	bool isCompleteStatus = true;
	if (b_isReadyToCloseMode)
	{
		for (EcSizeT i = 0; i < m_NumJoints; i++)
		{
			EcReal scale = 2;
			EcReal ratio = fabs(m_actualJointVel[i]) * 10;
			scale = 2.0*ratio;

			if (m_currentReadyToCloseAssistiveTime < 0.02)
				scale *= 4.0;

			if (m_startCloseAssistiveJointVel[i] * m_actualJointVel[i] < 0 || b_havedReverseJointVel[i])
			{
				scale = 0.0;
				b_havedReverseJointVel[i] = true;
			}

			if (fabs(m_actualJointVel[i]) > 0.1 && !b_havedReverseJointVel[i])		// 限制关节速度不大于20°/s
				m_driveBackTorques[i] = m_frictionModel.calculateFrictionTorqueForCloseAssistive(i, m_actualJointVel[i], fabs(m_actualJointVel[i]) * scale, 10);
			else
				m_driveBackTorques[i] = m_frictionModel.calculateFrictionTorqueForCloseAssistive(i, m_actualJointVel[i], 0.2, 10);

			isCompleteStatus &= (fabs(m_actualJointVel[i]) < 0.05) || b_havedReverseJointVel[i];
			/*
			if (i == 0)
			{
				std::cout << "close:" << m_currentReadyToCloseAssistiveTime << "," << m_actualJointVel[i] << "," << m_driveBackTorques[i] << "," << scale << std::endl;
			}
			*/
		}

		//std::cout << "checkCompleteReadToClose:" << m_currentReadyToCloseAssistiveTime << "," << m_driveBackTorques[0] << "," <<  m_actualJointVel[0] << ";  " << std::endl;

		if (isCompleteStatus && m_currentReadyToCloseAssistiveTime < 0.150)		// 所有关节的速度如果都接近于0，那么就在50ms后关闭； 当前直接差分计算的关节速度会快于实际关节速度；
		{
			m_currentReadyToCloseAssistiveTime = 0.150;
		}

		if ( m_currentReadyToCloseAssistiveTime > 0.2)// 200ms
		{
			b_isReadyToCloseMode = false;
			return true;
		}
		
		m_currentReadyToCloseAssistiveTime += m_updateTimePeriod;
		//m_viscousRatioForCloseAssistive = m_currentReadyToCloseAssistiveTime * 50.0;
	}

	return false;
}


void CHansCollaborativeAlgorithm::setStartTimeForCheckAssisitiveMode(EcReal startTime)
{
	m_assistiveCheckTime = startTime;
}


void CHansCollaborativeAlgorithm::setStartAssistiveMode
(
)
{
	std::cout << "Start Assistive Mode. V3" << std::endl;

	m_currentAssistiveTime = 0.;
	m_AssistiveStartCount = 0;
	m_AssistiveStartSumAccel = m_zeroVector;

	// driveback mode retset
	b_isDriveBackAssistiveMode = EcFalse;
	b_isDriveBackMode = EcFalse;
	m_driveBackTorques = m_zeroVector;
	m_driveBackStatus = b_falseVector;

	b_isReadyToCloseMode = false;

	m_AssistiveStartJointPosition = m_actualJointPositions;
	m_AssistiveStartSensedTorques = m_SensedJointTorques;

	// 弹性摩擦相关参数
	m_jointMPosition = m_actualJointPositions;
	m_jointSidePosition = m_actualJointPositions;

	m_frictionModel.resetStatus();
}




EcBoolean CHansCollaborativeAlgorithm::checkSafeStartAssistiveMode
(
)
{
	m_currentAssistiveTime += m_updateTimePeriod;
	//  在开启零力示教后的0.05s开始判断负载是否准确；
	if ((m_currentAssistiveTime > m_assistiveCheckTime)
		&& (m_currentAssistiveTime < m_assistiveCheckTime + 0.2))
	{
		m_AssistiveStartCount++;
		for (EcU32 i = 0; i < m_NumJoints; i++)
		{
			// 增加平均值滤波，在整个观测时间内的平均值大于设定值才报错；
			m_AssistiveStartSumAccel[i] += m_FilteredJointAccelerations[i];	// 必须是实际的关节加速度；
			if (m_AssistiveStartCount >= m_AssistiveSafeAccumulateCount[i])
			{
				// acceleration
				EcReal mean = fabs(m_AssistiveStartSumAccel[i] / m_AssistiveStartCount);
				if (mean > m_AssistiveSafeStartAccel[i])
				{
					std::cout << " Assistive start error(ACC), joint" << i + 1 << "(acc): " << mean << " time:" << m_currentAssistiveTime << std::endl;
					return EcTrue;
				}

				// position
				if (fabs(m_actualJointPositions[i] - m_AssistiveStartJointPosition[i]) > AssistiveSafeStartJointMotion)
				{
					std::cout << "Assistive Start Error(Position), joint" << i + 1 << ": " << (m_actualJointPositions[i] - m_AssistiveStartJointPosition[i])
						<< "time:" << m_currentAssistiveTime << std::endl;
					return EcTrue;
				}

				//sensedTorque
				if (fabs(m_SensedJointTorques[i] - m_AssistiveStartSensedTorques[i]) > 12 * m_CoulombFriction[i])
				{
					std::cout << "Assistive Start Error(sensedTorque), joint" << i + 1 << ": " << (m_SensedJointTorques[i] - m_AssistiveStartSensedTorques[i])
						<< ",time:" << m_currentAssistiveTime << std::endl;
					return EcTrue;
				}
			}
		}
	}

	if (b_previousAssisteErrorState)
	{
		m_motionConstraintScale = 0.5;
		if (m_currentAssistiveTime > 3.0)			// 3s后取消约束状态；
		{
			b_previousAssisteErrorState = false;
		}
	}
	else
	{
		m_motionConstraintScale = 1.0;
	}

	return EcFalse;
}


void CHansCollaborativeAlgorithm::resetCobotStatus()
{
	if (!(m_AssistiveState == assistive_normal || m_AssistiveState == assistive_steadyStatus))
	{
		// TODO: 若前一个状态为异常状态，那在这个状态下，增加对速度的限制；
		b_previousAssisteErrorState = true;
	}
	m_AssistiveState = assistive_normal;
}

ENAssistiveState CHansCollaborativeAlgorithm::getAssistiveModeMotorCurrentCommands
(
	EcRealVector& motorCurrentCommands,
	EcBooleanVector& jointCollisionStatus
)
{
	motorCurrentCommands.resize(m_NumJoints);

	// 在开环电流控制模式下，这个碰撞的检测没有意义；
	EcBoolean retVal = calculateDisturbanceTorques(
		m_FilteredMotorCurrents,
		m_FilteredJointPositions,
		m_FilteredJointVelocities,
		m_FilteredJointAccelerations,
		m_SensedJointTorques,
		m_EstimatedJointTorques,
		m_DisturbanceJointTorques
	);

	if (checkCloseDriveBackMode())				// 若在碰撞状态下，退出反向驱动的辅助函数；
	{
		m_AssistiveState = assistive_steadyStatus;
		return assistive_steadyStatus;
	}


	if (checkSafeStartAssistiveMode())			// 判断负载等参数是否正常；
	{
		m_AssistiveState = assistive_errorRobotState;
		return assistive_errorRobotState;
	}

	/*
	if (fabs(m_currentAssistiveTime - 3.0) < 0.001)
		setReadyToCloseAssistiveMode();
	*/

	if (checkCompleteReadyToCloseAssisitiveMode())
	{
		m_AssistiveState = assistive_finshedReadyToClose;
		std::cout << "ready to close assistiveMode." << std::endl;
		m_currentAssistiveTime = 0;
		return assistive_finshedReadyToClose;
		//return assistive_steadyStatus;
	}

	EcRealVector coulombFriction(m_NumJoints), viscousFriction(m_NumJoints);			// 静摩擦力计算
	//retVal &= calculateCompensateCoulombFriction(m_FilteredJointVelocities, coulombFriction);
	m_frictionModel.calculateCompensateFriction(m_actualJointPositions, m_FilteredJointVelocities, m_FilteredJointAccelerations, m_SensedJointTorques,
		b_isDriveBackMode, coulombFriction, viscousFriction);


	EcRealVector gravitationalTorques(m_NumJoints);
	m_dynBase->calculateGravityJointTorques(m_FilteredJointPositions, m_DynamicsLinearParameters, gravitationalTorques);

	retVal &= m_StateEstimator.filterDisturbanceTorques(m_DisturbanceJointTorques, m_DisturbanceJointTorques);
	retVal &= updateCollisionStopStatus(m_AssistiveModeCollisionStopThresholds, m_DisturbanceJointTorques, jointCollisionStatus);

	if (!retVal)
	{
		std::cout << "cobot: errorInitParams." << std::endl;
		m_AssistiveState = assistive_errorInitParams;
		return assistive_errorInitParams;
	}


	EcReal cartFrictionScale = 1.0;
	// 加入对末端笛卡尔速度的限制；(已屏蔽)  
	if (calculateCartesianVelocityScale(cartFrictionScale))
	{
		m_AssistiveState = assistive_overJointsRangeLimit;
		return assistive_overJointsRangeLimit;
	}

	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		const EcReal currentPosition = m_FilteredJointPositions[ii];
		const EcReal currentVelocity = m_FilteredJointVelocities[ii];

		const EcReal jointSpeed = fabs(currentVelocity);
		const EcReal viscousFrictionPercentage =
			(jointSpeed < m_maxJointVelocitiesInAssistiveMode[ii])
			? pow(fabs(m_maxJointVelocitiesInAssistiveMode[ii] - jointSpeed) / m_maxJointVelocitiesInAssistiveMode[ii], 0.1)
			: 0.0;

		//加入速度阻尼
		EcReal coulombDampCoeff = 1.0;
		EcReal absVelRatio = jointSpeed / (m_maxJointVelocitiesInAssistiveMode[ii] * m_motionConstraintScale);

		if (absVelRatio > 0.9 && absVelRatio <= 1.0)
		{
			coulombDampCoeff = 1 - absVelRatio;				// 线性约束
		}
		else if (absVelRatio > 1.0)
		{
			coulombDampCoeff = -std::pow(6.0, 10 * absVelRatio - 10.0);		// 指数约束；
		}

		EcReal frictionCompensationTorque = 0.0;
		frictionCompensationTorque = coulombDampCoeff * coulombFriction[ii] + viscousFrictionPercentage * viscousFriction[ii];

		frictionCompensationTorque *= cartFrictionScale * m_constraintsFrictionCompensatoryFactor;			// 当速度超过250mm/s时，提供限制；

		const EcReal futurePosition = currentPosition + currentVelocity * m_timeStep;
		EcReal jointLimitZoneFriction = 0;

		if (
			futurePosition >= m_UpperJointLimits[ii] - m_stopBoundary[ii] ||
			futurePosition <= m_LowerJointLimits[ii] + m_stopBoundary[ii]
			)
		{
			std::cout << "Error: closing to Joint Limit" << std::endl;
			std::cout << "upperLimits =" << m_UpperJointLimits[ii] << "," << "lowerLimits =" << m_LowerJointLimits[ii] << std::endl;
			jointCollisionStatus[ii] = EcTrue;
			m_AssistiveState = assistive_overJointsRangeLimit;
			return assistive_overJointsRangeLimit;
		}
		else if
			(
				(futurePosition >= m_UpperJointLimits[ii] - m_slowDownBoundary[ii] && currentVelocity > 0) ||
				(futurePosition <= m_LowerJointLimits[ii] + m_slowDownBoundary[ii] && currentVelocity < 0)
				)
		{
			EcReal distanceToLimit = 0;

			if (currentVelocity > 0)
				distanceToLimit = m_UpperJointLimits[ii] - futurePosition;
			else
				distanceToLimit = futurePosition - m_LowerJointLimits[ii];

			const EcReal jointLimitViscousFrictionCoefficient = m_jointLimitAgainstForceEquivalent[ii] * pow((1.0 - fabs(distanceToLimit / m_slowDownBoundary[ii])), 4);
			jointLimitZoneFriction = currentVelocity * jointLimitViscousFrictionCoefficient   // 在关节边界上提供反向力，同时减弱摩擦力的补偿；
				+ pow((1.0 - fabs(distanceToLimit / m_slowDownBoundary[ii])), 0.7) * frictionCompensationTorque;

		}

		if (m_AdjustedTorqueConstants[ii] == 0)
		{
			std::cout << "Error: adjustedTorqueConstants = 0" << std::endl;
			m_AssistiveState = assistive_errorInitParams;
			return assistive_errorInitParams;
		}

		EcReal compensationTorque = jointLimitZoneFriction
			- 1.0 * (frictionCompensationTorque + gravitationalTorques[ii]);

		// 如果是反向驱动模式，在反向驱动阶段，所有关节都不补偿摩擦力；
		if (b_isDriveBackMode) {
			compensationTorque += 1.0 * frictionCompensationTorque;
		}

		if (m_driveBackStatus[ii] || b_isReadyToCloseMode)						//反向驱动
		{
			compensationTorque += m_driveBackTorques[ii];// drive back torque; only in mode_ForceFree;
		}


		if (fabs(compensationTorque) > m_MaxActutorTorques[ii])
			motorCurrentCommands[ii] = sign(compensationTorque) * m_MaxActutorCurrents[ii];	// max actuator torques
		else
			motorCurrentCommands[ii] = compensationTorque / m_AdjustedTorqueConstants[ii];

		// 如果超过最大力矩（电流），就以最大电流为限；
		//加入对最大允许电流的限制
		if (std::fabs(motorCurrentCommands[ii]) > m_MaxActutorCurrents[ii] * 1.1)
		{
			std::cout << "Error: over current" << "(" << m_MaxActutorCurrents[ii] << ")" << std::endl;
			std::cout << "Joint" << ii + 1 << ": " << motorCurrentCommands[ii] << std::endl;

			motorCurrentCommands[ii] = sign(compensationTorque) * m_MaxActutorCurrents[ii];
			jointCollisionStatus[ii] = EcTrue;
			m_AssistiveState = assistive_overJointsCurrent;
			return assistive_overJointsCurrent;
		}
		//if(ii==1 || jointCollisionStatus[ii])
		//	std::cout << "cobot assistive:"<<ii<<"," << jointCollisionStatus[1] << "," << motorCurrentCommands[1] << "," << m_EstimatedJointTorques[1] << "," << m_DisturbanceJointTorques[1] << std::endl;;
	}
	m_AssistiveState = assistive_normal;
	return assistive_normal;
}

EcBoolean CHansCollaborativeAlgorithm::getGravityCompensationCurrentCommands
(
	EcRealVector& motorCurrentCommands,
	EcBooleanVector& jointCollisionStatus
)
{
	EcBoolean retVal = EcTrue;

	EcRealVector gravitationalTorques(m_NumJoints);
	motorCurrentCommands.resize(m_NumJoints);

	EcRealVector coulombFriction(6);
	m_dynBase->calculateGravityJointTorques(m_FilteredJointPositions, m_DynamicsLinearParameters, gravitationalTorques);
	retVal &= calculateCompensateCoulombFriction(m_FilteredJointVelocities, coulombFriction);
	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		if (m_AdjustedTorqueConstants[ii] == 0)
		{
			retVal = EcFalse;
			continue;
		}
		jointCollisionStatus[ii] = EcFalse;
		EcReal friction = 0.;
		if (b_isSensorlessAdmittanceMode) {
			std::cout << "Sensorless admittance friction compensation" << std::endl;

			friction = coulombFriction[ii];
			b_isSensorlessAdmittanceMode = EcFalse;
		}
		else {
			friction = m_ViscousFrictionCoefficient[ii] * m_FilteredJointVelocities[ii];
		}
		motorCurrentCommands[ii] = (gravitationalTorques[ii] + friction) / m_AdjustedTorqueConstants[ii];
	}

	return retVal;
}

void CHansCollaborativeAlgorithm::getComputeTorqueCurrentCommands
(
	const EcReal compensateRatio,
	EcRealVector& motorCurrentCommands
)
{
	EcRealVector computeTorque(m_NumJoints);
	motorCurrentCommands.resize(m_NumJoints);


	m_dynBase->calculateEstimateJointToqrues(m_FilteredCommandJointPositions,
		m_FilteredCommandJointVelocities,
		m_FilteredCommandJointAccelerations,
		m_DynamicsLinearParameters,
		computeTorque);

	//std::cout << "m_FilteredCommandVelocity:  ";
	for (EcU32 i = 0; i < m_NumJoints; i++)
	{
		//std::cout << m_FilteredCommandJointVelocities[i] << ",";
		//computeTorque[i] *= computeTorque[i] - m_FilteredCommandJointVelocities[i] * m_CoulombFriction[i];
		computeTorque[i] *= compensateRatio;
	}
	//std::cout << std::endl;

	//std::cout << "computTorque: ";
	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		//std::cout << computeTorque[ii] << ",";
		if (m_AdjustedTorqueConstants[ii] == 0)
		{
			continue;
		}
		EcReal viscousFriction = 0.;
		motorCurrentCommands[ii] = (computeTorque[ii] + viscousFriction) / m_AdjustedTorqueConstants[ii];
	}

	//std::cout << std::endl;
}

void CHansCollaborativeAlgorithm::getComputeTorqueCurrentCommands
(
	const EcReal compensateRatio,
	const EcRealVector& jointPosition,
	const EcRealVector& jointVelocity,
	const EcRealVector& jointAcceleration,
	EcRealVector& motorCurrentCommands
)
{
	EcRealVector computeTorque(m_NumJoints);
	motorCurrentCommands.resize(m_NumJoints);

	EcRealVector dynLinearParams = m_DynamicsLinearParameters;

	// 避免设置负载时，前馈电流出现阶跃，导致位置发生运动
	if (b_newPayLoadStatus)
	{
		EcReal mass = m_prePayloadMass + (m_payloadMass - m_prePayloadMass) * m_payloadCount / 50.0;
		m_payloadCount++;
		EcReal x, y, z, lx, ly, lz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
		x = m_centerofMass[0];
		y = m_centerofMass[1];
		z = m_centerofMass[2];

		EcReal length = sqrt(x * x + y * y + z * z);

		Ixx = 0;	Iyy = 0;	Izz = 0;	Ixy = 0;	Ixz = 0;	Iyz = 0;
		lx = mass * x;	ly = mass * y;	lz = mass * z;
		EcRealVector payloadPara = { Ixx, Ixy, Ixz, Iyy, Iyz, Izz, lx, ly, lz, mass };
		for (EcU32 ii = 0; ii < 10; ii++)
		{
			dynLinearParams[13 * (m_NumJoints - 1) + ii] = m_DynamicsParameters[13 * (m_NumJoints - 1) + ii] + payloadPara[ii];
		}

		if (m_payloadCount >= 50)
			b_newPayLoadStatus = false;

	}

	m_dynBase->calculateEstimateJointToqrues(jointPosition,
		jointVelocity,
		jointAcceleration,
		dynLinearParams,
		computeTorque);

	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		if (m_AdjustedTorqueConstants[ii] == 0)
		{
			std::cout << "torque constants == 0" << std::endl;
			continue;
		}
		motorCurrentCommands[ii] = (computeTorque[ii]) / m_AdjustedTorqueConstants[ii];
	}
}

void CHansCollaborativeAlgorithm::getGravityTorqueCurrentCommands
(
	const EcReal compensateRatio,
	EcRealVector& motorCurrentCommands
)
{
	EcBoolean retVal = EcTrue;

	EcRealVector gravitationalTorques(m_NumJoints);
	motorCurrentCommands.resize(m_NumJoints);

	m_dynBase->calculateGravityJointTorques(m_FilteredCommandJointPositions, m_DynamicsLinearParameters, gravitationalTorques);


	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		if (m_AdjustedTorqueConstants[ii] == 0)
		{
			retVal = EcFalse;
			continue;
		}
		EcReal viscousFriction = m_ViscousFrictionCoefficient[ii] * m_FilteredCommandJointVelocities[ii];
		motorCurrentCommands[ii] = compensateRatio * (gravitationalTorques[ii] + viscousFriction) / m_AdjustedTorqueConstants[ii];
	}

}

void CHansCollaborativeAlgorithm::getGravityTorqueCurrentCommands
(
	const EcReal compensateRatio,
	const EcRealVector& jointPosition,
	EcRealVector& motorCurrentCommands
)
{
	EcBoolean retVal = EcTrue;

	EcRealVector gravitationalTorques(m_NumJoints);
	motorCurrentCommands.resize(m_NumJoints);
	m_dynBase->calculateGravityJointTorques(jointPosition, m_DynamicsLinearParameters, gravitationalTorques);

	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		if (m_AdjustedTorqueConstants[ii] == 0)
		{
			retVal = EcFalse;
			continue;
		}
		EcReal viscousFriction = m_ViscousFrictionCoefficient[ii] * m_FilteredCommandJointVelocities[ii];
		motorCurrentCommands[ii] = compensateRatio * (gravitationalTorques[ii] + viscousFriction) / m_AdjustedTorqueConstants[ii];
	}

}

void CHansCollaborativeAlgorithm::setMaxJointVelocity
(
	const EcRealVector& maxJointVelocity
)
{
	EcU32 numJoints = maxJointVelocity.size();
	for (EcU32 ii = 0; ii < numJoints; ii++)
	{
		m_maxJointVeloctiy[ii] = maxJointVelocity[ii];
	}
	std::cout << "(cobot)max joint velociy: " << m_maxJointVeloctiy[0] << "," << m_maxJointVeloctiy[1] << "," << m_maxJointVeloctiy[2] << "," << m_maxJointVeloctiy[3] << "," << m_maxJointVeloctiy[4] << "," << m_maxJointVeloctiy[5] << std::endl;
}

void CHansCollaborativeAlgorithm::setMaxJointAccelerations
(
	const EcRealVector& maxJointAccelerations
)
{
	EcU32 numJoints = maxJointAccelerations.size();
	for (EcU32 ii = 0; ii < numJoints; ii++)
	{
		m_maxJointAccelerations[ii] = maxJointAccelerations[ii];
	}
}

void CHansCollaborativeAlgorithm::setLowVelocityThreshold
(
	const EcRealVector& velocityThreshold
)
{
	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		if (velocityThreshold[ii] < KDL::epsilon)
		{
			m_lowVelocityThresholds[ii] = KDL::epsilon;
		}
		else
		{
			m_lowVelocityThresholds[ii] = fabs(velocityThreshold[ii]);
		}
	}
	std::cout << "Low Vel Threshold:" << velocityThreshold[0] << "," << velocityThreshold[1] << "," << velocityThreshold[2] << "," << velocityThreshold[3] << "," <<
		velocityThreshold[4] << "," << velocityThreshold[5] << std::endl;
}

void CHansCollaborativeAlgorithm::setVibrationPeriod()
{
	m_vibrationPeriod[0] = 2 * 3.1415926 * 6;
	m_vibrationPeriod[1] = 2 * 3.1415926 * 5;
	m_vibrationPeriod[2] = 2 * 3.1415926 * 5.6;
	m_vibrationPeriod[3] = 2 * 3.1415926 * 7;
	m_vibrationPeriod[4] = 2 * 3.1415926 * 7.5;
	m_vibrationPeriod[5] = 2 * 3.1415926 * 8;
	m_vibrationPeriod[6] = 2 * 3.1415926 * 6.5;
}



/*
// 选择开放哪个自由度，对于无力传感器的方案，将其转换为位置/姿态；
void CHansCollaborativeAlgorithm::selectAdmittanceControlFreedom
(
const EcBooleanVector selectedFreedom
)
{
	if (selectedFreedom[0])
	{
		m_sensorlessAdmitControl->setOrientationAdmittance(!selectedFreedom[0]);
	}
	else{
		m_sensorlessAdmitControl->setOrientationAdmittance(selectedFreedom[0]);
	}
}

void CHansCollaborativeAlgorithm::setAdmitValidWrenchThreshold
(
const EcReal validForce,
const EcReal validTorque
)
{
	m_sensorlessAdmitControl->setValidWrenchThreshold(validForce, validTorque);
}

// 计算当前周期中最占优的末端力/矩；
void CHansCollaborativeAlgorithm::calculateSensorlessEndEffectorForces
(
EcRealVector& calcEEForces
)
{
	b_isSensorlessAdmittanceMode = EcTrue;
	EcRealVector eeOutputForces(6);
	EcBoolean ret = calculateEeOutputForces(m_FilteredJointPositions, m_DisturbanceJointTorques, eeOutputForces);
	m_sensorlessAdmitControl->votingSystem(eeOutputForces, calcEEForces);
	for (EcSizeT i = 0; i < calcEEForces.size(); i++)
	{
		calcEEForces[i] = -calcEEForces[i];
	}
	std::cout <<"calculate EE forces:" << calcEEForces[0] << "," << calcEEForces[1] << "," << calcEEForces[2] << std::endl;
}
*/

EcBoolean CHansCollaborativeAlgorithm::calculateMaxRectifyEstimateJointTorques
(
)
{
	EcRealVector q = { 0, 1.57, 0, 0, 0, 0 };
	EcRealVector dq = { 0, -0.1, 0, 0, 0, 0 };
	EcRealVector ddq = { 0, -m_maxJointAccelerations[1], 0, 0, m_maxJointAccelerations[4], 0 };
	EcReal gx = m_gx;
	EcReal gy = m_gy;
	EcReal gz = m_gz;
	m_gx = 0;
	m_gy = 0;
	m_gz = -9.81;

	EcBoolean ret = m_dynBase->calculateEstimateJointToqrues(q, dq, ddq, m_DynamicsLinearParameters, m_maxJointTorques);		// be used to calculate drive back torque
	m_maxJointTorques[0] = fabs(m_maxJointTorques[1]);	// joint2
	m_maxJointTorques[1] = fabs(m_maxJointTorques[1]);
	m_maxJointTorques[2] = fabs(m_maxJointTorques[2]);	// joint3
	m_maxJointTorques[3] = fabs(m_maxJointTorques[2]);
	m_maxJointTorques[4] = fabs(m_maxJointTorques[4]);	// joint5
	m_maxJointTorques[5] = fabs(m_maxJointTorques[4]);


	m_gx = gx;
	m_gy = gy;
	m_gz = gz;
	return ret;
}


EcReal CHansCollaborativeAlgorithm::sign(const EcReal& x)
{
	if (fabs(x) < m_nonZeroVelocity)
	{
		return 0;
	}

	if (x > 0)
		return 1.0;
	else
		return -1.0;
}



void CHansCollaborativeAlgorithm::saturationFunction(EcReal& value, EcSizeT index)
{
	if (fabs(value) > m_CoulombFriction[index])
		value = KDL::sign(value) * m_CoulombFriction[index];
}

void CHansCollaborativeAlgorithm::calculateCoulombFriction(const EcRealVector& jointPosition, EcRealVector& friction)
{
	// 更新 alpha, 改变滞后参数
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (fabs(jointPosition[i] - m_jointMPosition[i]) > m_velThreshold)
			m_omegaK[i] = std::max(m_omegaK[i] + m_Rw, 1.0);
		else
			m_omegaK[i] = std::max(m_omegaK[i] - m_Rw, 0.0);

		m_alpha[i] = (1 - m_omegaK[i]) * m_alphaL + m_omegaK[i] * m_alphaH;

		m_jointMPosition[i] = jointPosition[i];

		friction[i] = m_KVector[i] * (m_jointMPosition[i] - m_jointSidePosition[i]) / (1 + m_updateTimePeriod * m_alpha[i]);
		saturationFunction(friction[i], i);

		m_jointSidePosition[i] = m_jointMPosition[i] - friction[i] / m_KVector[i];
		friction[i] = m_KVector[i] * (m_jointMPosition[i] - m_jointSidePosition[i]);
	}

}

EcBoolean CHansCollaborativeAlgorithm::calculateCartesianVelocityScale(EcReal& cartScale)
{
	// 正常出货的屏蔽这个功能 
	cartScale = 1.0;
	return false;


	EcRealVector eeVelocity(6);
	//m_hmAlgorithm.calculateTCPVelocity(m_FilteredJointPositions, m_FilteredJointVelocities, eeVelocity);
	EcReal vel = KDL::sqrt(eeVelocity[0] * eeVelocity[0] + eeVelocity[1] * eeVelocity[1] + eeVelocity[2] * eeVelocity[2]);
	if (vel < 0.225)
	{
		cartScale = 1.0;
	}
	else {
		cartScale = (vel < 0.25)
			? pow(fabs(0.25 - vel) / 0.25, 0.5)
			: 0.0;
		if (vel > 0.25)
			return EcTrue;
	}

	return EcFalse;
}

void CHansCollaborativeAlgorithm::setMaxPowerAndMomentumConstraints
(
	const EcReal maxPower,
	const EcReal maxMomentum,
	const EcRealVector& maxJointPowers
)
{
	b_activeConstraintPower = false;
	b_constraintPowerStatus = false;
	m_velFactorConstraint = 1.0;
	m_preVelFactorConstraint = 1.0;
	m_accFactorConstraint = 1.0;
	m_maxPower = maxPower;
	m_maxMomentum = maxMomentum;
	m_maxJointPowers = maxJointPowers;
	m_maxConstraintPower = maxPower * 0.95;
	m_maxConstraintMomentum = maxMomentum * 0.95;
	std::cout << "power and momentum constraints:" << maxPower << "," << maxMomentum << std::endl;
	std::cout << "joint power constraints:" << maxJointPowers[0] << "," << maxJointPowers[1] << "," << maxJointPowers[2] << "," << maxJointPowers[3] << "," << maxJointPowers[4] << "," << maxJointPowers[5] << std::endl;
}

bool CHansCollaborativeAlgorithm::calculatePowerAndMomentum(
	const EcRealVector& jointVoltages,
	EcReal& electircPower, // P = U * I;
	EcReal& physicsPower,  // P = Torque * omega;
	EcReal& momentum,	   // momemtum = mass * omega;
	EcRealVector& jointPowers
)
{
	EcRealVector jointMomentum;
	EcRealVector jointPhysicsPower(6), jointSpeedRatio(6), jointPowerRatio(6), jointMomentumRatio(6);
	m_momentumObserver->getJointGeneralizeMomentum(jointMomentum);
	momentum = 0.0;
	physicsPower = 0.0;
	electircPower = 0.0;

	for (int i = 0; i < 6; i++)
	{
		momentum += jointMomentum[i];
		//jointPhysicsPower[i] = m_SensedJointTorques[i] * m_FilteredJointVelocities[i];

		// 在这里，将摩擦力的影响去掉，不属于对外部的输出力矩；
		jointPhysicsPower[i] = (m_SensedJointTorques[i] - (m_FilteredJointVelocities[i] * m_ViscousFrictionCoefficient[i] + sign(m_FilteredJointVelocities[i]) * m_CoulombFriction[i])) * m_FilteredJointVelocities[i];


		physicsPower += jointPhysicsPower[i];
		//electircPower += jointVoltages[i] * m_FilteredMotorCurrents[i];
	}
	jointPowers = jointPhysicsPower;
	return true;
}



bool CHansCollaborativeAlgorithm::calculatePowerAndMomentumConstraints
(
	const EcReal userDefineOverride,
	const EcRealVector& jointVoltages,
	EcBoolean& constraintActive,						// need to re-plan the motion at rising edge;
	EcReal& velFactorConstraint,					// constraint factor for joint velocities;
	EcReal& accFactorConstraint,					// constraint factor for joint accelerations;
	EcReal& electircPower,						// P = U * I;
	EcReal& physicsPower,							// P = Torque * omega;
	EcReal& momentum								// momemtum = mass * omega;
)
{
	accFactorConstraint = 1.0;
	velFactorConstraint = 1.0;

	constraintActive = false;
	if (userDefineOverride < 0.0001)
		return false;

	EcRealVector jointPowers(6);
	m_jointVoltages = jointVoltages;
	calculatePowerAndMomentum(jointVoltages, electircPower, physicsPower, momentum, jointPowers);
	m_momentumObserver->calculateMomentum(m_FilteredJointPositions,m_FilteredCommandJointVelocities,momentum);


	if (physicsPower > m_maxConstraintPower || fabs(momentum) > m_maxConstraintMomentum)
	{
		EcReal phyRatio = fabs(m_maxConstraintPower / (physicsPower + 0.0000001));
		EcReal momRatio = fabs(m_maxConstraintMomentum / (momentum + 0.00001));
		velFactorConstraint = (phyRatio < momRatio) ? phyRatio : momRatio;

		constraintActive = true;
	}
	else if ((physicsPower > 0.90 * m_maxConstraintPower || momentum > 0.90 * m_maxConstraintMomentum) && b_constraintPowerStatus)
	{   // hysteresis
		velFactorConstraint = m_velFactorConstraint;
		constraintActive = true;
	}

	// 关节的功率约束
	for (int i = 0; i < 6; i++)
	{
		if (jointPowers[i] > m_maxJointPowers[i])
		{
			velFactorConstraint = std::min(velFactorConstraint, m_maxJointPowers[i] / jointPowers[i]);
		}
	}


	// avoid fluctuation when robot in constraint status;
	if (b_constraintPowerStatus && m_velFactorConstraint < velFactorConstraint)
	{
		velFactorConstraint = m_velFactorConstraint - 0.01;
	}


	// low filter
	if (!constraintActive)
	{
		velFactorConstraint = (velFactorConstraint * m_updateTimePeriod + m_velFactorConstraint * 0.2) / (m_updateTimePeriod + 0.2);
	}
	else
	{
		velFactorConstraint = (velFactorConstraint * m_updateTimePeriod + m_velFactorConstraint * 0.1) / (m_updateTimePeriod + 0.1);
	}


	m_velFactorConstraint = velFactorConstraint;
	b_constraintPowerStatus = constraintActive;



	/*
		// add mean filter, because the setOverride func of codesys need smooth;
		m_meanFilteredVelConstraint.push_front(velFactorConstraint);
		EcReal tempSum = 0.0;
		for(EcSizeT i =0; i<NUMofMeanVelFactorConst;i++)
		{
			tempSum+=m_meanFilteredVelConstraint[i];
		}
		velFactorConstraint = tempSum/NUMofMeanVelFactorConst;


		// low freq to change velConstraint;
		if((int(m_currentTime/m_updateTimePeriod)%20 == 0))
		{
			m_preVelFactorConstraint = velFactorConstraint;
		}
		velFactorConstraint = m_preVelFactorConstraint;
	*/

	return true;
}

ENMotionConstraintStatus CHansCollaborativeAlgorithm::monitorMotionConstraintsStatus(
	EcBoolean isAssistiveMode)
{

	// add:运动状态的约束
	EcReal momentum, physicalPower, electricPower;
	EcRealVector jointPowers(6);
	calculatePowerAndMomentum(m_jointVoltages, electricPower, physicalPower, momentum, jointPowers);


	if (isAssistiveMode)
	{
		if (!(m_AssistiveState == assistive_normal || m_AssistiveState == assistive_steadyStatus || m_AssistiveState == assistive_finshedReadyToClose))
		{
			if (m_AssistiveState == assistive_steadyStatus)
			{	// 如果过一个实时周期没有处理碰撞反弹后零力示教回复正常的信号，则将零力示教状态切换为严重碰撞类型，让控制器报错；
				m_AssistiveState = assistive_severCollision;
				return constraint_normal;
			}
			std::cout << "constraint_errorStatus." << std::endl;
			return constraint_errorStatus;
		}

		for (EcSizeT ii = 0; ii < m_NumJoints; ii++)
		{
			EcReal tempVel = 0.0;
			if (b_isDriveBackMode)
				tempVel = m_maxJointVeloctiy[ii];
			else
				tempVel = m_maxJointVelocitiesInAssistiveMode[ii];

			if (fabs(m_FilteredJointVelocities[ii]) > tempVel * 1.25)
			{
				std::cout << "safe thread(assistiveMode, over speed limit): index, currentVel, maxVel" << ii + 1 << "," << m_FilteredJointVelocities[ii] << "," << m_maxJointVelocitiesInAssistiveMode[ii] << std::endl;
				return constraint_overJointVelocitiesLimit;
			}
		}
		if (fabs(momentum) > MaxAllowMomentumInAssistiveMode * 5 && !b_isDriveBackMode)
		{
			std::cout << "safe thread(assistiveMode, over momentum limit):current,maxAllow " << momentum << "," << MaxAllowMomentumInAssistiveMode << std::endl;
			return constraint_overMomentumLimit;
		}


		if (physicalPower > MaxAllowPowerInAssistiveMode * 5 && !b_isDriveBackMode)
		{
			std::cout << "safe thread(assistiveMode, over power limit):physical,maxAllow " << physicalPower << "," << MaxAllowPowerInAssistiveMode << std::endl;
			return constraint_overPowerLImit;
		}

		m_constraintsFrictionCompensatoryFactor = 1.0;
		// 如果超过阈值75%，则开始降低摩擦力的补偿比例；
		if (fabs(momentum) > MaxAllowMomentumInAssistiveMode * 0.75 && !b_isDriveBackMode)
		{
			m_constraintsFrictionCompensatoryFactor = 3.1 - 3.0 * fabs(momentum) / MaxAllowMomentumInAssistiveMode;
		}

		if (physicalPower > MaxAllowPowerInAssistiveMode * 0.75 && !b_isDriveBackMode)
		{
			m_constraintsFrictionCompensatoryFactor = 3.1 - 3.0 * fabs(physicalPower) / MaxAllowPowerInAssistiveMode;
		}


	}
	else
	{
		for (EcSizeT ii = 0; ii < m_NumJoints; ii++)
		{
			if (fabs(m_FilteredJointVelocities[ii]) > m_maxJointVeloctiy[ii] * 1.25)
			{
				std::cout << "safe thread(positionMode,over speed limit):index, current, max " << ii + 1 << "," << m_FilteredJointVelocities[ii] << "," << m_maxJointVeloctiy[ii] << std::endl;
				return constraint_overJointVelocitiesLimit;
			}

		}

		if (fabs(momentum) > (m_maxConstraintMomentum + m_payloadMass) * 6)
		{
			std::cout << "safe thread(positionMode, over momentum limit):velRatio,current,maxAllow " << m_velFactorConstraint << "," << momentum << "," << m_maxConstraintMomentum << std::endl;
			return constraint_overMomentumLimit;
		}


		if (physicalPower > m_maxConstraintPower * 6)
		{
			std::cout << "safe thread(positionMode, over power limit):electric,physical,maxAllow " << electricPower << "," << physicalPower << "," << m_maxConstraintPower << std::endl;
			return constraint_overPowerLImit;
		}

	}


	return constraint_normal;
}

void CHansCollaborativeAlgorithm::getSettingParameters(
	EcRealVector& kinParams,
	EcRealVector& actuatorDamp,
	EcRealVector& torqueConstant,
	EcRealVector& gearRatio,
	EcRealVector& maxEfficiency,
	EcRealVector& maxActuatorCurrents,
	EcReal& rotationofMounting, /* rotate robot base mounting angles(deg) */
	EcReal& tiltofMounting,		/* tilt robot base mounting angles(deg) */
	EcRealVector& upperJointLimits,
	EcRealVector& lowerJointLimits,
	EcRealVector& dynamicsParams,
	EcRealVector& collisionStopThresholds,			 /* joint collisiton threshold, */
	EcRealVector& collisionStopInMomentumThresholds, /* joint collisiton threshold, */
	EcRealVector& assistiveModeCollisionStopThresholds,
	EcRealVector& frictionCompensatoryFactor,	/* default setting: J1="0.5" J2="0.5" J3="0.4" J4="0.6" J5="0.7" J6="0.7" */
	EcRealVector& frictionCompensatoryFactorII, /* default seting [1.0], */
	EcRealVector& lowVelocityThreshold,			/* decide whether the joints are in low velocity status, */
	EcRealVector& maxJointVelocity,
	EcReal& maxPowerConstraint,		/* power */
	EcReal& maxMomentumConstraint,	/* momentum */
	EcReal& distanceLimitInDriveBack, /* allowing joint motion(degree,  °) in drive back mode, */
	EcReal& timeDurationInDriveBack,	/* allowing time duration(second, s) in drive back mode, */
	EcReal& mass,
	EcRealVector& centerofMass, /* (millimeter) */
	bool& isAssistiveMode,
	int& drivebackMode,
	EcReal& assistiveCheckTime /* the start time for check robot status in assistive mode, second ,s; */
)
{
	kinParams = m_kinParams;
	for (int i = 0; i < kinParams.size(); i++)
		kinParams[i] *= 1000.0;
	actuatorDamp = m_dampSetting;
	torqueConstant = m_torqueConstant;
	gearRatio = m_gearRatio;
	maxEfficiency = m_maxEfficiency;
	maxActuatorCurrents = m_MaxActutorCurrents;
	rotationofMounting = m_baseMountingRotation * KDL::rad2deg;
	tiltofMounting = m_baseMountingTilt * KDL::rad2deg;

	upperJointLimits = m_UpperJointLimits;
	lowerJointLimits = m_LowerJointLimits;
	maxJointVelocity = m_maxJointVeloctiy;
	lowVelocityThreshold = m_lowVelocityThresholds;

	for (int i = 0; i < numofJoints; i++)
	{
		upperJointLimits[i] *= KDL::rad2deg;
		lowerJointLimits[i] *= KDL::rad2deg;
		maxJointVelocity[i] *= KDL::rad2deg;
		lowVelocityThreshold[i] *= KDL::rad2deg;
	}


	dynamicsParams = m_DynamicsParameters;
	collisionStopThresholds = m_CollisionStopThresholds;
	collisionStopInMomentumThresholds = m_MomentumCollisionThresholds;
	assistiveModeCollisionStopThresholds = m_AssistiveModeCollisionStopThresholds;
	frictionCompensatoryFactor = m_frictionCompensatoryFactor;
	frictionCompensatoryFactorII = m_startCompensateFrictionFactor;

	maxPowerConstraint = m_maxPower;
	maxMomentumConstraint = m_maxMomentum;
	distanceLimitInDriveBack = m_allowBackDistance * KDL::rad2deg;
	timeDurationInDriveBack = m_allowMotionDuration;

	mass = m_payloadMass;
	centerofMass = m_centerofMass;
	centerofMass = { centerofMass[0] * 1000.0,centerofMass[1] * 1000.0,centerofMass[2] * 1000.0 };
	isAssistiveMode = !b_isDriveBackAssistiveMode;
	drivebackMode = m_drivebackMode;
	assistiveCheckTime = m_assistiveCheckTime;
}
