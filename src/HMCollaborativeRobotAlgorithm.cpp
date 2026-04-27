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
{
	m_updateTimePeriod = updateTimePeriod;
	m_stopGenerateFrictCounts = 2.0 * 60.0;
	m_stopStateTimeCounts = 0;
	
	EcRealVector b = { 0.9922, -0.9922 };
	EcRealVector a = { 1, -0.9844 };
	m_momentumObserver = momentumObserver::create(updateTimePeriod, b, a);
	m_momentumObserverActual = momentumObserver::create(updateTimePeriod, b, a);

	m_prePayloadMass = 0.0;
	m_centerofMass.assign(3, 0.0);
	b_newPayLoadStatus = false;
	m_payloadCount = 1;
	b_logFlag = true;
	b_enableEEWrenchFeedForward = false;
	m_NumJoints = 0; 
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
	m_NumJoints = jointPositions.size();
	if (m_NumJoints == 0)
	{
		m_IsInitialized = EcFalse;
		return EcFalse;
	}

	switch (robotType)
	{
	default:
	case 0:
	case 2:
	case 7:
	case 9:
		m_dynBase.reset(new elfinDynamics);
		m_kinBase.reset(new elfinKinematics);
		break;
	case 1:
	case 3:
	case 8:
	case 10:
		m_dynBase.reset(new urDynamics);
		m_kinBase.reset(new URKinematics);
		break;
	case 5:
		m_dynBase.reset(new palletDynamics);
		m_kinBase.reset(new palletKinematics);
		break;
	case 12:
		m_dynBase.reset(new dsDynamics);
		m_kinBase.reset(new DSKinematics);
		break;
	case 20:
		m_dynBase.reset(new anthorDynamics);
		m_kinBase.reset(new DSKinematics);
		break;
	}
	m_robotType = robotType;

	m_logStatus = true;

	m_currentTime = 0.0;
	b_isUsingMomentumObserver = EcFalse;
	m_momentumObserver->initializeStates(jointPositions, robotType);
	m_momentumObserverActual->initializeStates(jointPositions, robotType);
	m_frictionModel.initialize(jointPositions, m_updateTimePeriod);
	
	m_feedForwardAccTorqueRatio.assign(m_NumJoints, 1.0);
	m_FeedAccTorque.assign(m_NumJoints, 0.0);
	m_jointTorqueFromEEForce.assign(m_NumJoints, 0.0);
	m_startCompensateFrictionFactor.assign(m_NumJoints, 1.0);
	m_torqueConstant.assign(m_NumJoints, 1.0);

	m_savedCollisionStopStatus.assign(m_NumJoints, EcFalse);
	m_driveBackTorques.assign(m_NumJoints, 0.);
	m_driveBackStatus.assign(m_NumJoints, EcFalse);
	m_driveBackBrakingStatus.assign(m_NumJoints, EcFalse);
	m_collisionStopStatus.assign(m_NumJoints, EcFalse);
	m_AdjustedTorqueConstants.assign(m_NumJoints, 1.0);
	m_MaxActutorTorques.assign(m_NumJoints, 0.0);
	m_MaxActutorCurrents.assign(m_NumJoints, 1.0);
	m_previousFeedforwardCurrent.assign(m_NumJoints, 0.0);

	m_FilteredJointPositions = jointPositions;
	m_FilteredJointVelocities.assign(m_NumJoints, 0.0);
	m_FilteredActualJointVelocities.assign(m_NumJoints, 0.0);
	m_FilteredJointAccelerations.assign(m_NumJoints, 0.0);

	m_lockJointEstimateStatus = 30;
	m_FilteredCommandJointPositions = jointPositions;
	m_FilteredCommandJointVelocities.assign(m_NumJoints, 0.0);
	m_FilteredCommandJointAccelerations.assign(m_NumJoints, 0.0);

	m_FilteredMotorCurrents.assign(m_NumJoints, 0.0);
	b_is15066Strategy = false;

	m_previousJointVelocity.assign(m_NumJoints, 0.0);

	m_SensedJointTorques.assign(m_NumJoints, 0.0);
	m_EstimatedJointTorques.assign(m_NumJoints, 0.0);
	m_DisturbanceJointTorques.assign(m_NumJoints, 0.0);
	m_meanDisturbanceJointTorques.assign(m_NumJoints, 0.0);
	m_FilteredSensedEeForces.assign(m_NumJoints, 0.0);
	m_FilteredAdmittanceDeviatePose.assign(m_NumJoints, 0.0);
	m_filteredAdmittanceVelocity.assign(m_NumJoints, 0.0);
	m_EstimatedGravityTorques.assign(m_NumJoints, 0.0);
	m_StateEstimator.initialize(jointPositions);
	m_commandStateEstimator.initialize(jointPositions);
	EcRealVector filterTimeConstant;
	filterTimeConstant.assign(m_NumJoints, 0.012);			
	setFilterTimeConstant(jointPositions, filterTimeConstant, filterTimeConstant);

	m_ViscousFrictionCoefficient.assign(m_NumJoints, 0.0);
	m_CoulombFriction.assign(m_NumJoints, 0.0);
	m_vibrationPeriod.assign(m_NumJoints, 0.0);
	m_JacobianDeterminant = 1.0;

	m_jointLimitAgainstForceEquivalent.assign(m_NumJoints, 100.0);

	m_timeStep = m_StateEstimator.samplingPeriod();

	m_selectedPositionCoeff.assign(m_NumJoints, 0.0);
	if (m_NumJoints >= 2) {
		m_selectedPositionCoeff[0] = 1.0;
		m_selectedPositionCoeff[1] = 1.0;
	}
	m_viscousVelocityCoeff.assign(m_NumJoints, 1.0);
	m_endEffectorInertia = 0.5;
	m_endEffectorMass = 5;
	m_eeCalcVelocity.assign(m_NumJoints, 0.0);
	m_eeCalcPosition.assign(m_NumJoints, 0.0);
	m_eeCurrentPosition.assign(m_NumJoints, 0.0);

	m_payloadMass = 0.0;				
	m_centerofMass.assign(3, 0.0);

	m_d1 = 0.220;		
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
	m_gearRatio.assign(m_NumJoints, 1.0);

	EcRealVector para;
	para.assign(m_NumJoints * 13, 0.0);

	m_DynamicsLinearParameters = para;
	m_DynamicsParameters = para;
	m_compenasteDynParams = para;
	m_baseMountingRotation = 0.0;
	m_baseMountingTilt = 0.0;

	m_friTemperaturesParams.assign(m_NumJoints, 1.0);
	m_jointTemperatures.assign(m_NumJoints, 45.0);
	m_jointVoltages.assign(m_NumJoints, 48.0);
	m_forceCollisionThreshold = { 80,80,30 };

	EcReal rotation = 0;
	EcReal tilt = 0;
	setRobotMountingAngles(rotation, tilt);

	m_frictionCompensatoryFactor.assign(m_NumJoints, 0.7);
	m_UpperJointLimits.assign(m_NumJoints, 3.14);
	m_LowerJointLimits.assign(m_NumJoints, -3.14);
	m_jointRange.assign(m_NumJoints, 6.24);
	m_slowDownBoundary.assign(m_NumJoints, 0.2);
	m_stopBoundary.assign(m_NumJoints, 0.005);

	m_meanDriveBackTorque.clear();
	for (int i = 0; i < m_NumJoints; i++) {
		MovingAverage ma;
		ma.setDataPoolLength(5);
		m_meanDriveBackTorque.push_back(ma);
	}
	b_initEnterDriveBackFlag.assign(m_NumJoints, false);

	m_maxJointVelocitiesInAssistiveMode.assign(m_NumJoints, 1.75);
	if (m_NumJoints >= 3) {
		m_maxJointVelocitiesInAssistiveMode[0] = 1.05;
		m_maxJointVelocitiesInAssistiveMode[1] = 1.05;
		m_maxJointVelocitiesInAssistiveMode[2] = 1.05;
	}
	
	m_maxJointVelocitiesInAssistiveModeForErr.assign(m_NumJoints, 2.35);
	if (m_NumJoints >= 3) {
		m_maxJointVelocitiesInAssistiveModeForErr[0] = 1.05;
		m_maxJointVelocitiesInAssistiveModeForErr[1] = 1.05;
		m_maxJointVelocitiesInAssistiveModeForErr[2] = 1.05;
	}

	m_frictionModel.setMaxJointVelocitiesInAssistiveMode(m_maxJointVelocitiesInAssistiveMode);
	m_nonZeroVelocity = 0.001;
	m_assistiveCheckTime = 0.08;

	m_zeroVelocityThresholds.assign(m_NumJoints, 0.06);
	m_lowVelocityThresholds.assign(m_NumJoints, 0.25);
	m_CollisionStopDynamicsThresholds.assign(m_NumJoints, 0.0);
	setVibrationPeriod();
	m_VibrationAmplitude.assign(m_NumJoints, 1.0);
	if (m_NumJoints >= 6) {
		m_VibrationAmplitude[0] = 0.9;
		m_VibrationAmplitude[1] = 0.9;
		m_VibrationAmplitude[2] = 0.9;
		m_VibrationAmplitude[5] = 1.2;
	}

	m_maxJointVeloctiy.assign(m_NumJoints, 1.5);
	m_maxJointAccelerations.assign(m_NumJoints, 6.0);


	m_IsInitialized = EcTrue;
	m_IsAdmittanceInitialized = EcFalse;

	m_currentAssistiveTime = 10.;
	m_observerTorques.assign(m_NumJoints, 0.0);
	m_AssistiveStartJointPosition.assign(m_NumJoints, 0.0);
	m_AssistiveStartSensedTorques.assign(m_NumJoints, 0.0);

	m_AssistiveSafeAccumulateCount.assign(m_NumJoints, 50);
	if (m_NumJoints >= 6) m_AssistiveSafeAccumulateCount[5] = 150;

	m_AssistiveSafeStartAccel.assign(m_NumJoints, 0.8);
	if (m_NumJoints >= 6) m_AssistiveSafeStartAccel[5] = 1.5;

	m_AssistiveStartSumAccel.assign(m_NumJoints, 0.0);
	m_zeroVector.assign(m_NumJoints, 0.0);
	b_falseVector.assign(m_NumJoints, false);

	m_AssistiveState = assistive_normal;
	m_motionConstraintScale = 1.0;
	b_previousAssisteErrorState = false;

	m_maxAllowControlBoxCurrent = 200;			

	m_driveBackStatusVector.assign(m_NumJoints, status_statePosition);
	m_driveBackStartPosition = jointPositions;
	m_collisionDetectivePosition = jointPositions;
	m_overZeroVelCount.assign(m_NumJoints, 0);
	m_actualPreviousJointVel.assign(m_NumJoints, 0.0);
	m_actualJointPosition.assign(m_NumJoints, 0.0);
	m_sensedRawTorque.assign(m_NumJoints, 0.0);
	m_startCollisionJointVel.assign(m_NumJoints, 0.0);
	m_driveBackDirctionForJ5.assign(m_NumJoints, 1.0);

	m_accumuActualJointCurrent.assign(m_NumJoints, 0);
	m_accumuActualXYZPosition.assign(3, 0);
	m_startActualXYXVel.assign(3, 0);
	m_accumuTime = 0;

	m_driveBackDisturbancesThreshold.assign(m_NumJoints, 100.0);
	m_driveBackCollisionStopStatus.assign(m_NumJoints, EcFalse);
	m_massThresholdTorques.assign(m_NumJoints, 0.0);
	m_MomentumCollisionThresholds.assign(m_NumJoints, 0.0);
	m_maxJointTorques.assign(m_NumJoints, 0.0);
	m_AssistiveModeCollisionStopThresholds.assign(m_NumJoints, 200.0);
	m_startDriveBackJointPosition.assign(m_NumJoints, 0.0);

	m_alpha.assign(m_NumJoints, 20.0);
	m_omegaK.assign(m_NumJoints, 0.0);
	m_frictionRatio2.assign(m_NumJoints, 0.7);
	m_calcCoulombFriction.assign(m_NumJoints, 0.0);
	m_alphaL = 20;
	m_alphaH = 500;
	m_Rw = 0.01;
	m_velThreshold = 0.005;
	m_KVector.assign(m_NumJoints, 2000.0);
	if (m_NumJoints >= 3) {
		m_KVector[0] = 3000.0;
		m_KVector[1] = 3000.0;
		m_KVector[2] = 2000.0;
	}
	if (m_NumJoints >= 6) {
		m_KVector[3] = 2000.0;
		m_KVector[4] = 2500.0;
		m_KVector[5] = 500.0;
	}

	m_jointMPosition = jointPositions;
	m_jointSidePosition = jointPositions;
	m_CollisionStopThresholds.assign(m_NumJoints, 100);

	m_allowBackDistance = 5 * KDL::PI / 180.0;
	m_allowMotionDuration = 0.5;

	m_maxPower = 300;
	m_maxMomentum = 25;
	m_maxConstraintPower = 300;
	m_maxConstraintMomentum = 25;
	m_maxJointPowers.assign(m_NumJoints, 1000);
	m_velFactorConstraint = 1.0;
	m_preVelFactorConstraint = 1.0;
	m_accFactorConstraint = 1.0;

	m_meanFilteredVelConstraint.assign(NUMofMeanVelFactorConst, 1.0);
	m_constraintsFrictionCompensatoryFactor = 1.0;


	m_dropDataCount = 0;
	m_actualJointPositions = jointPositions;
	m_actualJointVel.assign(m_NumJoints, 0.0);


	m_loopJointPosition.assign(21, jointPositions);
	m_loopCommandJointPosition.assign(21, jointPositions);
	m_loopSensedCurrent.assign(21, EcRealVector(m_NumJoints, 0.0));

	m_movMeanLenForDistrubanceTorque = int(0.1 / m_updateTimePeriod);

	m_diffTorqueMovMean.clear();
	for (int i = 0; i < m_NumJoints; i++) {
		MovingAverage ma;
		ma.setDataPoolLength(m_movMeanLenForDistrubanceTorque);
		m_diffTorqueMovMean.push_back(ma);
	}

	m_forceMovMean.clear();
	for (int i = 0; i < 3; i++) {
		MovingAverage ma;
		ma.setDataPoolLength(100);
		m_forceMovMean.push_back(ma);
	}

	m_eeAccMovMean.clear();
	for (int i = 0; i < 3; i++) {
		MovingAverage ma;
		ma.setDataPoolLength(5);
		m_eeAccMovMean.push_back(ma);
	}

	m_diffTorqueTriggerCollisionRatio.assign(m_NumJoints, 1.4);
	if (m_NumJoints >= 6) {
		m_diffTorqueTriggerCollisionRatio[m_NumJoints-2] = 1.7;
		m_diffTorqueTriggerCollisionRatio[m_NumJoints-1] = 1.7;
	}
	
	b_isReadyToCloseMode = false;

	m_previousEnchanceDisturbJointTorques.assign(m_NumJoints, 0);
	
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

	for (int i = 0; i < m_NumJoints; i++)
		if (fabs(commandJointPositions[i] - m_loopCommandJointPosition[20][i]) > 0.03)  
			m_lockJointEstimateStatus = 0;

	m_loopJointPosition[20] = jointPositions;
	m_loopCommandJointPosition[20] = commandJointPositions;
	m_loopSensedCurrent[20] = motorCurrents;


	EcRealVector delayCommandJointPositon = commandJointPositions;
	if (!b_is15066Strategy)
	{
		int delayCount = int(0.03 / m_updateTimePeriod);
		delayCount = 20 - delayCount;
		delayCount = (delayCount > 0) ? delayCount : 0;
		delayCommandJointPositon = m_loopCommandJointPosition[delayCount];

	}

	for (int i = 0; i < 20; i++)
	{
		m_loopJointPosition[i] = m_loopJointPosition[i + 1];
		m_loopCommandJointPosition[i] = m_loopCommandJointPosition[i + 1];
		m_loopSensedCurrent[i] = m_loopSensedCurrent[i + 1];
	}


	bool warningStatus = false;
	for (int i = 0; i < m_NumJoints; i++)
	{
		if (fabs(m_FilteredJointVelocities[i]) > 0.087							
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

	EcRealVector tempJointPositions(m_NumJoints);
	if (warningStatus)
	{
		EcReal t2 = m_updateTimePeriod * m_updateTimePeriod;
		for (int i = 0; i < m_NumJoints; i++)
			tempJointPositions[i] = m_actualJointPositions[i] + m_FilteredActualJointVelocities[i] * m_updateTimePeriod * m_dropDataCount + m_FilteredActualJointAcc[i] * t2 * m_dropDataCount;
	}
	else
	{
		tempJointPositions = jointPositions;
		m_dropDataCount = 0;
	}

<<<<<<< HEAD
	m_actualJointPosition = jointPositions;
	// 计算关节的关节差分速度
	for (int i = 0; i < m_NumJoints; i++)
		m_actualJointVel[i] = (tempJointPositions[i] - m_actualJointPositions[i]) / m_updateTimePeriod;

	//std::cout << "update:"<<m_actualJointVel[0]<<"," << tempJointPositions[0] << "," << m_actualJointPositions[0] << "," << m_updateTimePeriod << std::endl;
	m_actualJointPositions = jointPositions;
=======
	m_actualJointPosition = tempJointPositions;
	for (int i = 0; i < m_NumJoints; i++)
		m_actualJointVel[i] = (tempJointPositions[i] - m_actualJointPositions[i]) / m_updateTimePeriod;

	m_actualJointPositions = tempJointPositions;
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117

	EcBoolean retVal = m_StateEstimator.estimateStates(
		tempJointPositions,
		m_FilteredJointPositions,
		m_FilteredJointVelocities,
		m_FilteredJointAccelerations
	);

	retVal = m_commandStateEstimator.estimateStates(
		delayCommandJointPositon,
		m_FilteredCommandJointPositions,
		m_FilteredCommandJointVelocities,
		m_FilteredCommandJointAccelerations
	);

	m_FilteredActualJointVelocities = m_FilteredJointVelocities;
	m_FilteredActualJointAcc = m_FilteredJointAccelerations;

	if (m_lockJointEstimateStatus > 30)
	{
		m_FilteredJointVelocities = m_FilteredCommandJointVelocities;
		m_FilteredJointAccelerations = m_FilteredCommandJointAccelerations;
	}
	
	if(m_lockJointEstimateStatus<40)
		m_lockJointEstimateStatus++;

	retVal &= m_StateEstimator.filterMotorCurrents(motorCurrents, m_FilteredMotorCurrents);
	getSensedTorques(motorCurrents, m_sensedRawTorque);
	return retVal;
}

void CHansCollaborativeAlgorithm::updateForceSensorData(
	const EcBoolean enable,
	const EcVector force
)
{
	if (enable)
	{
		EcVector tempForce = force;
		for (int i = 0; i < 3; i++)
		{
			m_forceMovMean[i].updateFilteredData(force[i], tempForce[i]);
			m_sensedForce[i] = force[i] - tempForce[i];
		}
	}
	b_enableForceCollision = enable;
	m_rawForce = force;
}

void CHansCollaborativeAlgorithm::updateForceSensorForFeedForward(
	const EcBoolean enable,
	const EcRealVector& force
)
{
	if (enable)
	{
		EcFrame ee = m_kinBase->forwardKinematics(m_loopCommandJointPosition[20]);
		EcVector F = { force[0],force[1],force[2] };
		EcVector T = { force[3],force[4],force[5] };

		F = ee.M * F;
		T = ee.M * T;
		EcRealVector baseForce = { F[0], F[1], F[2], T[0], T[1], T[2] };

		m_kinBase->calcJacobianJointTorque(m_loopCommandJointPosition[20], baseForce, m_jointTorqueFromEEForce);
		for (int i = 0; i < m_jointTorqueFromEEForce.size(); i++)
			m_jointTorqueFromEEForce[i] *= 0.0;
	}
	else
	{
		m_jointTorqueFromEEForce.assign(m_NumJoints, 0);
	}
	b_enableEEWrenchFeedForward = enable;
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
	EcBoolean retVal = calculateDisturbanceTorques(m_FilteredMotorCurrents,
		m_FilteredJointPositions,
		m_FilteredJointVelocities,
		m_FilteredJointAccelerations,
		m_SensedJointTorques,
		m_EstimatedJointTorques,
		m_DisturbanceJointTorques);

	EcRealVector meanDisturbanceTorque(m_NumJoints);
	for (int i = 0; i < m_NumJoints; i++)
	{
		m_diffTorqueMovMean[i].updateFilteredData(m_DisturbanceJointTorques[i], meanDisturbanceTorque[i]);
		m_meanDisturbanceJointTorques[i] = m_DisturbanceJointTorques[i] - meanDisturbanceTorque[i];
	}

	m_momentumObserver->updateStateEstimates(m_actualJointPositions,
		m_FilteredJointVelocities,
		m_FilteredJointAccelerations,
		m_SensedJointTorques);

	m_momentumObserverActual->updateStateEstimates(m_actualJointPositions,
		m_FilteredActualJointVelocities,
		m_FilteredActualJointAcc,
		m_SensedJointTorques);

	updateDynamicsCollisionStopThreshold(m_DisturbanceJointTorques,m_CollisionStopDynamicsThresholds);

	retVal &= updateCollisionStopStatus(m_CollisionStopDynamicsThresholds, m_DisturbanceJointTorques, jointCollisionStatus);

	bool normalStatus = true;
	if (b_isUsingMomentumObserver)
	{
		EcBooleanVector momentumCollisionStatus(m_NumJoints), momentumCollisionActualStatus(m_NumJoints);
		m_momentumObserver->getObserverTorqueDisturbances(m_observerTorques);

		m_momentumObserver->getJointCollisionState(momentumCollisionStatus);
		m_momentumObserverActual->getJointCollisionState(momentumCollisionActualStatus);

		for (EcSizeT i = 0; i < m_NumJoints; i++)
		{
			momentumCollisionStatus[i] = momentumCollisionStatus[i] && momentumCollisionActualStatus[i];
			if (momentumCollisionStatus[i])
				type = collision_Observer;
			if (jointCollisionStatus[i])
				type = collision_deviation;

			jointCollisionStatus[i] = jointCollisionStatus[i] || momentumCollisionStatus[i];
		}
	}
	else
		m_observerTorques.assign(m_NumJoints, 0.0);

	if (b_enableForceCollision && m_eeLinearVel > 0.180)
	{
		
		if (fabs(m_sensedForce[2]) > m_forceCollisionThreshold[2])
		{
			type = collision_forceSensor;
			jointCollisionStatus[2] = true;
			std::cout << "force collision_:" <<m_currentTime<<",  " << m_sensedForce[2] << "," << m_previousEndEffectorVel[2] << "," << m_meanEEAcc[2] << "," << m_loopSensedCurrent[20][2] << std::endl;
		}
		EcReal planeForce = std::sqrt(m_sensedForce[0] * m_sensedForce[0] + m_sensedForce[1] * m_sensedForce[1]);
		if (planeForce > m_forceCollisionThreshold[1])
		{
			type = collision_forceSensor;
			jointCollisionStatus[1] = true;
			std::cout << "force collision_:" << m_currentTime << ",  " << m_sensedForce[0] << "," << m_sensedForce[1] << "," << m_loopSensedCurrent[20][1] << std::endl;
		}
	}

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (jointCollisionStatus[i] && m_currentTime > 5 && fabs(m_SensedJointTorques[i]) >= fabs(m_EstimatedJointTorques[i]) * 0.2 && fabs(m_FilteredJointVelocities[i]) >= 0.005)
		{
			normalStatus = false;
			std::cout << "ft:" << b_enableForceCollision << "," << m_eeLinearVel << "," << m_sensedForce[2]<<","<<m_rawForce[2] << std::endl;
			std::cout << "collisionJoint:";
			for (int k = 0; k < m_NumJoints; k++) std::cout << jointCollisionStatus[k] << (k == m_NumJoints - 1 ? "" : ",");
			std::cout << std::endl;

			std::cout << "Collision Joint:" << i + 1 << ",pvased,th,type:" << m_FilteredJointPositions[i] << "," << m_FilteredJointVelocities[i] << "," <<
				m_FilteredJointAccelerations[i] << "," << m_SensedJointTorques[i] << "," << m_EstimatedJointTorques[i] << "," << m_DisturbanceJointTorques[i] <<
				"," << m_CollisionStopDynamicsThresholds[i] << "," << m_observerTorques[i] << "," << type << std::endl;
			
			std::cout << "actual jointVel:";
			for (int k = 0; k < m_NumJoints; k++) std::cout << m_FilteredActualJointVelocities[k] << (k == m_NumJoints - 1 ? "" : ",");
			std::cout << std::endl;

			std::cout << "actual jointAcc:";
			for (int k = 0; k < m_NumJoints; k++) std::cout << m_FilteredActualJointAcc[k] << (k == m_NumJoints - 1 ? "" : ",");
			std::cout << std::endl;

			std::cout << "updateJointPosition=[";
			for (int k = 0; k < 20; k++) std::cout << m_loopJointPosition[k][i] << ",";
			std::cout<<"];" << std::endl;

			std::cout << "updateCommandJointPosition=[";
			for (int k = 0; k < 20; k++) std::cout << m_loopCommandJointPosition[k][i] << ",";
			std::cout<<"];" << std::endl;
			std::cout << "updateCurrent=[";
			for (int k = 0; k < 20; k++) std::cout << m_loopSensedCurrent[k][i] << ",";
			std::cout <<"];"<< "\n jointStatus:" << std::endl;
			
			std::cout << "dynth: ";
			for (int k = 0; k < m_NumJoints; k++) std::cout << m_CollisionStopDynamicsThresholds[k] << (k == m_NumJoints - 1 ? "" : ",");
			std::cout << std::endl;

			std::cout <<"est: ";
			for (int k = 0; k < m_NumJoints; k++) std::cout << m_EstimatedJointTorques[k] << (k == m_NumJoints - 1 ? "" : ",");
			std::cout << std::endl;

			std::cout <<"meanD: ";
			for (int k = 0; k < m_NumJoints; k++) std::cout << m_meanDisturbanceJointTorques[k] << (k == m_NumJoints - 1 ? "" : ",");
			std::cout << std::endl;

			std::cout << "diff: ";
			for (int k = 0; k < m_NumJoints; k++) std::cout << m_DisturbanceJointTorques[k] << (k == m_NumJoints - 1 ? "" : ",");
			std::cout << std::endl;

			std::cout <<"mObs: ";
			for (int k = 0; k < m_NumJoints; k++) std::cout << m_observerTorques[k] << (k == m_NumJoints - 1 ? "" : ",");
			std::cout << std::endl;

			std::cout << std::endl;
			std::cout << "\n update current:\n";
			for (int j = 0; j < m_NumJoints; j++)
			{
				std::cout << "joint" << j + 1 << "=[";
				for (int kk = 0; kk < 19; kk++) std::cout << m_loopSensedCurrent[kk][j] << ",";
				std::cout << m_loopSensedCurrent[19][j] <<"];" << std::endl;
			}
			break;
		}
	}

	if (m_currentTime < 0.2)
		jointCollisionStatus = m_savedCollisionStopStatus;

	if (!normalStatus)
	{
		m_currentTime = 0;			
		m_savedCollisionStopStatus = jointCollisionStatus;
	}


	m_collisionStopStatus = jointCollisionStatus;
	return retVal;
}


void CHansCollaborativeAlgorithm::updateDynamicsCollisionStopThreshold
(
	const EcRealVector& currentDisturbTorque,
	EcRealVector& enchanceDisturbJointTorques
)
{
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (fabs(m_FilteredJointVelocities[i]) <  0.01 && fabs(m_FilteredJointAccelerations[i]) > 1)		
		{
			enchanceDisturbJointTorques[i] = m_CollisionStopThresholds[i] + 3.0 * m_CoulombFriction[i];
		}
		if (fabs(m_FilteredJointVelocities[i]) < 0.07)		
		{
			enchanceDisturbJointTorques[i] = m_CollisionStopThresholds[i] + 3.0 * m_CoulombFriction[i];
		}
		else if (fabs(m_FilteredJointVelocities[i]) < 0.09)
		{
			enchanceDisturbJointTorques[i] = m_CollisionStopThresholds[i] + 1.0 * m_CoulombFriction[i];
		}
		else if(fabs(m_FilteredJointAccelerations[i])>2.5){
			enchanceDisturbJointTorques[i] = m_CollisionStopThresholds[i]+ (fabs(m_FilteredJointAccelerations[i])/2.5 -0.3)* m_CoulombFriction[i];
		}
		else {
			enchanceDisturbJointTorques[i] = m_CollisionStopThresholds[i];
		}

		EcReal enhanceThreshold = fabs(m_EstimatedJointTorques[i]) *0.1 ;
		enchanceDisturbJointTorques[i] += enhanceThreshold;
	}

	for (int i = m_NumJoints - 2; i < m_NumJoints; i++) {
		EcReal ratio = 0.0;
		EcReal enchance = fabs(m_EstimatedJointTorques[i]) * (0.3 + ratio);
		if (fabs(m_FilteredJointVelocities[i]) > 1 || fabs(m_FilteredJointAccelerations[i])>6)		
		{
			ratio = fabs(m_FilteredJointVelocities[i]) / 10.0;
			if (fabs(m_FilteredJointVelocities[i]) > 2)		
				ratio *= 2;

			enchance = fabs(m_EstimatedJointTorques[i]) * (0.3 + ratio);
			enchance = (enchance > 2) ? enchance : 2;
		}
		enchanceDisturbJointTorques[i] = enchanceDisturbJointTorques[i] * 0.5 +enchance;
	}

	EcFrame T = m_kinBase->forwardKinematics(m_FilteredJointPositions);
	EcVector v1 = T.M.UnitZ();			

	EcRealVector endEffectorVel(6), endVelJ5(6);
	m_kinBase->calcJacobianEndEffectorVelocity(m_actualJointPosition, m_FilteredJointVelocities, endEffectorVel);
	EcVector v2 = { endEffectorVel[0],endEffectorVel[1],endEffectorVel[2] };			
	EcVector eeAcc = (v2 - m_previousEndEffectorVel) / m_updateTimePeriod;		
	for (int i = 0; i < 3; i++)
	{
		m_eeAccMovMean[i].updateFilteredData(eeAcc[i], m_meanEEAcc[i]);			
	}

	m_previousEndEffectorVel = v2;
	m_eeLinearVel = v2.Normalize();


	EcReal t = (v1.Norm() * v2.Norm());   
	if (t > 0)
	{
		EcReal temp = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / t;			
		temp = (fabs(temp) > 1.0) ? KDL::sign(temp) : temp;			
		enchanceDisturbJointTorques[m_NumJoints - 2] += (1-fabs(temp)) * 3 - 0.1* enchanceDisturbJointTorques[m_NumJoints - 2];
	}
	if(m_CollisionStopThresholds[m_NumJoints - 2]<10)
	{ 
		if (m_eeLinearVel > 0.280)				
		{
			EcReal ratio = m_eeLinearVel / 0.280;
			ratio = (ratio < 2) ? ratio : 2;
			for (int i = 0; i < 3; i++)
			{
				enchanceDisturbJointTorques[i] *= ratio;
			}
		}
	}

	m_previousEnchanceDisturbJointTorques = enchanceDisturbJointTorques;
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
	m_baseMountingTilt = tilt;							

	KDL::Rotation gRotation, R1, R2;

	R1 = KDL::Rotation::RPY(0, 0, m_baseMountingRotation);
	R2 = KDL::Rotation::RPY(0, m_baseMountingTilt, 0);
	gRotation = (R2 * R1).Inverse();

	KDL::Vector gVector = gRotation * m_gravity;
	m_gx = gVector[0];
	m_gy = gVector[1];
	m_gz = gVector[2];

	m_dynBase->setGravityVector(m_gx, m_gy, m_gz);
	m_momentumObserver->setGravityAcceleration(m_gx, m_gy, m_gz);
	m_momentumObserverActual->setGravityAcceleration(m_gx, m_gy, m_gz);
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
	std::cout << "(cobot)Max Actuator Current:";
	for(int i=0; i<m_NumJoints; i++) std::cout << maxCurrents[i] << (i==m_NumJoints-1?"":",");
	std::cout << std::endl;

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if (maxCurrents[i] < 0.1 || maxCurrents[i]>1000) {
			std::cout << "************** ERROR! over limit." <<maxCurrents[i]<< std::endl;
			return false;
		}
	}
	m_MaxActutorCurrents = maxCurrents;
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		m_MaxActutorTorques[i] = m_AdjustedTorqueConstants[i] * m_MaxActutorCurrents[i] * 1.0;		
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
	m_momentumObserverActual->setGravityAcceleration(m_gx, m_gy, m_gz);
	std::cout << "Gravity Vector:" << m_gx << "," << m_gy << "," << m_gz << std::endl;
}

void CHansCollaborativeAlgorithm::setDynamicsLinearParameters
(
	const EcRealVector& params
)
{
	m_DynamicsParameters.assign(params.begin(), params.end());
	m_DynamicsLinearParameters = m_DynamicsParameters;
	m_compenasteDynParams = m_DynamicsLinearParameters;

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
	std::cout << "payload:" << mass << "," << centerofMass[0] << "," << centerofMass[1] << "," << centerofMass[2] << std::endl;
	
	int startDualFlag = static_cast<int>(centerofMass[0] * 1000000) % 1000;				
	if (fabs(startDualFlag) == 813)
	{
		std::cout << "start dual encoder."<<centerofMass[1] * 1000 <<"\n\n\n";
		m_frictionModel.setAssistDualEncoderFlag(true);
	}



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

	Ixx = mass * x * x;	Iyy = mass*y*y;	Izz = mass*z*z;	Ixy = mass*x*y;	Ixz = mass*x*z;	Iyz = mass*y*z;
	lx = mass * x;	ly = mass * y;	lz = mass * z;
	EcRealVector payloadPara = { Ixx, Ixy, Ixz, Iyy, Iyz, Izz, lx, ly, lz, mass };

	int index = (m_robotType == 5) ? 2 : 1;

	for (EcU32 ii = 0; ii < 10; ii++)
	{
		m_DynamicsLinearParameters[13 * (m_NumJoints - index) + ii] = m_DynamicsParameters[13 * (m_NumJoints - index) + ii] + payloadPara[ii];
	}

	m_momentumObserver->setDynamicsParameters(m_DynamicsLinearParameters);
	m_momentumObserverActual->setDynamicsParameters(m_DynamicsLinearParameters);
	
	m_massThresholdTorques[0] = mass * (0.8 + length * 4);
	m_massThresholdTorques[1] = mass * (1.0 + length * 6);
	m_massThresholdTorques[2] = mass * (0.4 + length * 3);
	if (m_NumJoints > 3) {
		m_massThresholdTorques[3] = mass * (0.3 + length * 3);
	}
	for (int i = 4; i < m_NumJoints; i++) {
		m_massThresholdTorques[i] = mass * (0.25 + length * 3);
	}

	EcRealVector momemtumDynamicsThreshold(m_NumJoints);
	momemtumDynamicsThreshold[0] = mass * (0.5 + length * 4);
	momemtumDynamicsThreshold[1] = mass * (0.5 + length * 6);
	momemtumDynamicsThreshold[2] = mass * (0.3 + length * 2);
	for (int i = 3; i < m_NumJoints; i++) {
		momemtumDynamicsThreshold[i] = mass * (0.2 + length * 2);
	}

	for (int i = 0; i < m_NumJoints; i++)
	{
		if (fabs(m_FilteredJointVelocities[i]) > m_maxJointVeloctiy[i] * 0.8)		
		{
			m_massThresholdTorques[i] += m_CollisionStopThresholds[i] * 0.1;
			momemtumDynamicsThreshold[i] += m_MomentumCollisionThresholds[i] * 0.2;

		}
		m_massThresholdTorques[i] += m_EstimatedJointTorques[i] * 0.02;
	}

	m_momentumObserver->setDynamicsFactorThreshold(momemtumDynamicsThreshold);
	m_momentumObserverActual->setDynamicsFactorThreshold(momemtumDynamicsThreshold);

	calculateMaxRectifyEstimateJointTorques();
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
	
	std::cout << "friction:";
	for(int i=0; i<m_NumJoints; i++) std::cout << m_CoulombFriction[i] << (i==m_NumJoints-1?"":",");
	std::cout << std::endl;

}


EcBoolean CHansCollaborativeAlgorithm::updateCollisionStopStatus
(
	const EcRealVector& torqueThresholds,
	const EcRealVector& disturbanceTorques,
	EcBooleanVector& collisionStopStatus
)
{
	const EcU32 numJoints = disturbanceTorques.size();

	for (EcU32 ii = 0; ii < numJoints; ++ii)
	{
		if ((std::fabs(disturbanceTorques[ii]) >= torqueThresholds[ii] && std::fabs(m_meanDisturbanceJointTorques[ii]) >= torqueThresholds[ii])
			|| std::fabs(disturbanceTorques[ii])> torqueThresholds[ii] * m_diffTorqueTriggerCollisionRatio[ii])		
		{
			collisionStopStatus[ii] = EcTrue;
		}
		else {
			collisionStopStatus[ii] = EcFalse;
		}
	}
	return EcTrue;
}

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
			std::cout << "************** ERROR!" << std::endl;
			return false;
		}
	}
	m_CollisionStopThresholds = collisionStopThresholds;
	
	std::cout << "dTh:";
	for(int i=0; i<m_NumJoints; i++) std::cout << m_CollisionStopThresholds[i] << (i==m_NumJoints-1?"":",");
	std::cout << std::endl;

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
			std::cout << "************** ERROR!" << std::endl;
			return false;
		}
	}

	if (m_robotType == 0 || b_is15066Strategy)
		b_isUsingMomentumObserver = EcTrue;
	m_MomentumCollisionThresholds = collisionStopThresholds;
	m_momentumObserver->setCollisionThreshold(collisionStopThresholds);
	m_momentumObserverActual->setCollisionThreshold(collisionStopThresholds);
	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		m_driveBackDisturbancesThreshold[i] = 0.5 * collisionStopThresholds[i];
	}

	std::cout << "mTh:";
	for(int i=0; i<m_NumJoints; i++) std::cout << collisionStopThresholds[i] << (i==m_NumJoints-1?"":",");
	std::cout << std::endl;


	return true;
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
		std::cout << "************** ERROR! the number of size is not equal" << std::endl;
		return false;
	}

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if ((torqueConstant[i] < 0.05 || torqueConstant[i] >0.5) ||
			(gearRatio[i] < 50 || gearRatio[i]>300) ||
			(maxEfficiency[i] < 0.1 || maxEfficiency[i] >1.1)
			)
		{
			std::cout << "************** ERROR!" << std::endl;
			std::cout << "(index,torque constant,gearratio,efficiency): " << i << " ," << torqueConstant[i] << "," << gearRatio[i] << "," << maxEfficiency[i] << std::endl;
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
	
	std::cout << "torqueConstants:";
	for(int i=0; i<m_NumJoints; i++) std::cout << m_AdjustedTorqueConstants[i] << (i==m_NumJoints-1?"":",");
	std::cout << std::endl;

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
	m_momentumObserverActual->setKinematicsParameters(kinematcisParam);

	EcRealVector dh = kinematcisParam;
	m_kinBase->setRobotDHParameters(dh);

	for (EcSizeT i = 0; i < 4; i++) {
		if (kinematcisParam[i] < 0.01 || kinematcisParam[i]>1000.0)
		{
			std::cout << "************** ERROR!" << std::endl;
			std::cout << "index: " << i << " : " << kinematcisParam[i] << std::endl;
			return false;
		}
	}
	m_d1 = kinematcisParam[0];
	m_d4 = kinematcisParam[1];
	m_d6 = kinematcisParam[2];
	m_a2 = kinematcisParam[3];

	m_jointLimitAgainstForceEquivalent[0] = fabs(1000 * m_a2);
	m_jointLimitAgainstForceEquivalent[1] = fabs(1000 * (m_a2 + m_d4));
	m_jointLimitAgainstForceEquivalent[2] = fabs(1000 * (m_d4 + m_a2));
	m_jointLimitAgainstForceEquivalent[3] = fabs(1000 * 0.20);
	if (m_NumJoints >= 6) {
		m_jointLimitAgainstForceEquivalent[m_NumJoints - 2] = fabs(1000 * m_d6);
		m_jointLimitAgainstForceEquivalent[m_NumJoints - 1] = fabs(1000 * 0.15);
	}
	if (m_NumJoints == 7) {
		m_jointLimitAgainstForceEquivalent[4] = fabs(1000 * m_d6);
	}

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
		m_slowDownBoundary[ii] = (m_slowDownBoundary[ii] < 0.14) ? m_slowDownBoundary[ii] : 0.14;		
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

	for (int i = 0; i < 3; i++)
	{
		if (assistiveModeCollisionStopThresholds[i] != 200)
		{
			m_forceCollisionThreshold[i] = assistiveModeCollisionStopThresholds[i];
			std::cout << "setForceCollision：" << i << ": " << m_forceCollisionThreshold[i];
		}
	}
}

void CHansCollaborativeAlgorithm::setFrictionCompensatoryFactor
(
	const EcRealVector& frictionCompensatoryFactor
)
{
	m_frictionCompensatoryFactor = frictionCompensatoryFactor;
	m_frictionModel.setCompensateFactor(frictionCompensatoryFactor);
}

void CHansCollaborativeAlgorithm::setDynFrictionCompensatoryFactor
(
	const EcRealVector& frictionCompensatoryFactor
) {
	m_frictionModel.setDynFrictionCompensateFactor(frictionCompensatoryFactor);

}

void CHansCollaborativeAlgorithm::setStartCompensateFrictionFactor(
	const EcRealVector& compensateFactor)
{
	m_startCompensateFrictionFactor = compensateFactor;
}

void CHansCollaborativeAlgorithm::setFrictionParamsWithTemperature(
	const EcRealVector& factor)
{
	if (factor.size() != m_NumJoints)
		return;
	
	std::cout << "set temperature params:";
	for(int i=0; i<m_NumJoints; i++) std::cout << factor[i] << (i==m_NumJoints-1?"":",");
	std::cout << std::endl;

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
		std::cout << "not init.1" << std::endl;
		return EcFalse;
	}

	jointPostions = m_FilteredJointPositions;
	jointVelocities = m_FilteredJointVelocities;
	jointAccelerations = m_FilteredJointAccelerations;
	motorCurrents = m_FilteredMotorCurrents;

	getSensedTorques(motorCurrents, sensedTorques);				

	estimatedTorques = m_EstimatedJointTorques;
	disturbanceTorques = m_meanDisturbanceJointTorques;

	if (m_NumJoints >= 6 && m_AssistiveModeCollisionStopThresholds[m_NumJoints-1] != 200)
	{
		disturbanceTorques = m_FeedAccTorque;
	}

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
	m_dynBase->calculateEstimateJointToqrues(jointPositions, jointVelocities, jointAccelerations, m_DynamicsLinearParameters, estimatedTorques);
	m_dynBase->calculateGravityJointTorques(jointPositions, m_DynamicsLinearParameters, m_EstimatedGravityTorques);

	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		disturbanceTorques[ii] = sensedTorques[ii] - estimatedTorques[ii];
	}

	return EcTrue;
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
	calculateCoulombFriction(m_actualJointPositions, m_calcCoulombFriction);		

	EcReal sumVel = 0;
	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		sumVel += fabs(jointVelocity[ii]);
		if (fabs(jointVelocity[ii]) < m_zeroVelocityThresholds[ii])			
		{
			if (m_stopStateTimeCounts >= m_stopGenerateFrictCounts || b_isDriveBackMode)
			{
				coulombFriction[ii] = 0;
		}
			else {
				coulombFriction[ii] = generateVibrationSignal(ii, m_CoulombFriction[ii]) * 1.0;

			}
		}
		else if (fabs(jointVelocity[ii]) < m_lowVelocityThresholds[ii])
		{
			coulombFriction[ii] = pow(fabs(jointVelocity[ii]) / m_lowVelocityThresholds[ii], 0.30) * sign(jointVelocity[ii]) * m_CoulombFriction[ii];
		}
		else
		{
			coulombFriction[ii] = sign(jointVelocity[ii]) * m_CoulombFriction[ii];
		}

		coulombFriction[ii] *= (1 + fabs(jointVelocity[ii]) / m_maxJointVelocitiesInAssistiveMode[ii]);		
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
	std::cout << "Start Collision Drive Back Mode:"<<m_currentTime<<", " << drivebackMode << std::endl;
	m_drivebackMode = drivebackMode;
	m_currentDriveBackTime = 0.0;
	m_driveBackDuranceTime = 0.0;
	m_accumuActualJointCurrent.assign(m_NumJoints, 0);
	m_accumuActualXYZPosition.assign(3, 0);
	b_reachZeroVelFlag = false;
	m_accumuTime = 0;

	m_driveBackStatusVector.assign(m_NumJoints, status_statePosition);
	m_overZeroVelCount.assign(m_NumJoints, 0);

	m_driveBackTorques = m_zeroVector;
	b_isDriveBackAssistiveMode = EcTrue;
	b_isDriveBackMode = EcTrue;
	b_isReadyToCloseMode = false;

	m_driveBackStartPosition = m_actualJointPosition;
	m_collisionDetectivePosition = m_actualJointPosition;
	m_startCollisionJointVel = m_FilteredJointVelocities;				
	m_actualPreviousJointVel = m_actualJointVel;

	for (EcSizeT i = 0; i < m_NumJoints; i++)
	{
		if ((fabs(m_DisturbanceJointTorques[i]) > 0.6 * m_driveBackDisturbancesThreshold[i] && fabs(m_startCollisionJointVel[i]) > 0.07) || fabs(m_startCollisionJointVel[i]) > 0.12)				
		{
			m_driveBackCollisionStopStatus[i] = EcTrue;
			b_initEnterDriveBackFlag[i] = true;
		}
		else
			m_driveBackCollisionStopStatus[i] = EcFalse;
	}

	m_driveBackStatus = m_driveBackCollisionStopStatus;			
	for (EcSizeT i = 0; i < m_NumJoints; i++)
		if (m_driveBackCollisionStopStatus[i])
			m_driveBackStatusVector[i] = status_strongDrive;

	std::cout << "startVel:";
	for (int i = 0; i < m_NumJoints; i++) std::cout << m_startCollisionJointVel[i] << (i == m_NumJoints - 1 ? "" : ",");
	std::cout << std::endl;

	m_driveBackDirctionForJ5[m_NumJoints - 2] = 1.0;
	EcRealVector endEffectorVel(6),endVelJ5(6);
	m_kinBase->calcJacobianEndEffectorVelocity(m_actualJointPosition, m_FilteredJointVelocities, endEffectorVel);
	EcVector velEnd = { endEffectorVel[0],endEffectorVel[1],endEffectorVel[2] };
	m_startActualXYXVel = { endEffectorVel[0],endEffectorVel[1],endEffectorVel[2] };
	
	EcRealVector jointWristVel(m_NumJoints, 0.0);
	jointWristVel[m_NumJoints - 2] = m_FilteredJointVelocities[m_NumJoints - 2];
	m_kinBase->calcJacobianEndEffectorVelocity(m_actualJointPosition, jointWristVel, endVelJ5);
	EcVector velEndJ5 = { endVelJ5[0],endVelJ5[1],endVelJ5[2] };

	EcReal cartVel = velEnd.Normalize();
	EcReal projVel = KDL::dot(velEndJ5, velEnd);		
	if (projVel <  -fabs(cartVel * 0.05))
	{
		m_driveBackDirctionForJ5[m_NumJoints - 2] = -1.0;
	}

	m_startDriveBackJointPosition = m_actualJointPositions;
	m_currentAssistiveTime = AssistiveModeStartTime * 10;		
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
}

EcBoolean CHansCollaborativeAlgorithm::checkCloseDriveBackMode
(
)
{
	if (b_isDriveBackAssistiveMode)
	{
		for (int i = 0; i < m_NumJoints; i++)
		{
			if (m_collisionStopStatus[i])
			{
				m_collisionStopStatus[i] = false;
			}
		}
		m_currentDriveBackTime += m_updateTimePeriod;
		if (m_drivebackMode == mode_ForceFree)
		{
			if (m_currentDriveBackTime > DriveBackClosedTime)	
			{
				b_isDriveBackMode = EcFalse;						
				m_driveBackStatus = b_falseVector;				  

				EcReal sumVel = 0;
				for (EcSizeT i = 0; i < m_NumJoints; i++)
					sumVel += fabs(m_FilteredJointVelocities[i]);

				if (sumVel < DriveBackThresholdVelocity)
				{
					m_driveBackDuranceTime += m_updateTimePeriod;
				}
				else
					m_driveBackDuranceTime = 0.0;

				if (m_driveBackDuranceTime > MaxDriveBackEnduranceTime)	
				{
					b_isDriveBackAssistiveMode = EcFalse;
					return EcTrue;										
				}
			}
		}

		EcBoolean driveBackFlag = false;
		EcBoolean errorStatus = false;
		EcReal coeff = 0;
		EcReal coeffCoulomb = 0;
		EcReal coeffViscous = 0;

		if (m_drivebackMode == mode_LimitMotion)
		{
			for (EcSizeT i = 0; i < m_NumJoints; i++)
			{
				if (m_driveBackStatus[i])
				{
					switch (m_driveBackStatusVector[i])
					{
					case status_strongDrive:
<<<<<<< HEAD
						if (m_actualJointVel[i] * m_startCollisionJointVel[i] <= 0 || m_currentDriveBackTime > 0.06)
=======
						if ((m_actualJointVel[i] * m_startCollisionJointVel[i]*m_driveBackDirctionForJ5[i] <= 0 && m_currentDriveBackTime>0.018 )|| m_currentDriveBackTime > 0.05)
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117
						{
							m_driveBackStatusVector[i] = status_awayPosition;
							m_driveBackStartPosition[i] = m_actualJointPosition[i];
						}
						else																 
						{
							m_driveBackTorques[i] = 100.0 * m_frictionModel.calculateFrictionTorque(i, KDL::sign(m_startCollisionJointVel[i])*fabs(m_actualJointVel[i]));				
							if (m_NumJoints >= 6 && i == m_NumJoints - 2)
							{
								m_driveBackTorques[i] = m_driveBackTorques[i] * m_driveBackDirctionForJ5[i];
							}
						}
						if (b_initEnterDriveBackFlag[i])
						{
							b_initEnterDriveBackFlag[i] = false;
							m_meanDriveBackTorque[i].setDataPoolLength(5, m_driveBackTorques[i]);	
						}

						break;
					case status_awayPosition:				
						coeff = (driveBackJointPosition_away - fabs(m_actualJointPosition[i] - m_driveBackStartPosition[i])) / driveBackJointPosition_away;
<<<<<<< HEAD
						if (fabs(m_actualJointVel[i]) > 0.3)		// 限制关节速度不大于20°/s
							m_driveBackTorques[i] = m_frictionModel.calculateFrictionTorque(i, m_actualJointVel[i], 1, 30);
=======
						if (fabs(m_actualJointVel[i]) > 0.2)		
							m_driveBackTorques[i] = m_frictionModel.calculateFrictionTorque(i, m_actualJointVel[i], 1, 40);
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117
						else
							m_driveBackTorques[i] =m_frictionModel.calculateFrictionTorque(i, KDL::sign(m_startCollisionJointVel[i]) * fabs(m_actualJointVel[i]), 4, 1);

<<<<<<< HEAD
						if (fabs(m_actualJointPosition[i] - m_driveBackStartPosition[i]) > driveBackJointPosition_away || m_currentDriveBackTime > 0.12)			// 0.5°；
=======
						if (fabs(m_actualJointPosition[i] - m_driveBackStartPosition[i]) > driveBackJointPosition_away || m_currentDriveBackTime > 0.14)			
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117
						{
							m_driveBackStatusVector[i] = status_controlPosition;
							if (m_actualJointVel[i] * m_startCollisionJointVel[i] * m_driveBackDirctionForJ5[i] > 0)		
							{
								m_driveBackStatusVector[i] = status_stopMoving;
							}
						}

						break;
					case status_controlPosition:
						m_driveBackTorques[i] = 1.0 * m_frictionModel.calculateFrictionTorque(i, m_actualJointVel[i], 1.2, 30);			
						if (fabs(m_actualJointPosition[i] - m_driveBackStartPosition[i]) > driveBackJointPosition_control
<<<<<<< HEAD
							|| m_currentDriveBackTime > 0.2											// 1.0°；
							|| m_actualJointVel[i] * m_startCollisionJointVel[i] > 0)				// 已经制动为零	
=======
							|| m_currentDriveBackTime > 0.2											
							|| (m_actualJointVel[i] * m_startCollisionJointVel[i] > 0 && fabs(m_actualJointVel[i])<0.5))				
						{
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117
							m_driveBackStatusVector[i] = status_stopMoving;
						}

						break;
					case status_stopMoving:
<<<<<<< HEAD
						coeffCoulomb = fabs(m_actualJointVel[i]) > 0.1 ? 2.0 : 0;		// 速度在接近零（6°/s）时，取消摩擦力矩；避免过冲产生震荡；
						coeffViscous = fabs(m_actualJointVel[i]) > 0.1 ? 50.0 : 5;		// 速度在接近零（6°/s）时，取消摩擦力矩；避免过冲产生震荡；
						m_driveBackTorques[i] = 1.0 * m_frictionModel.calculateFrictionTorque(i, m_actualJointVel[i], coeffCoulomb, coeffViscous);			// 稳定，仅依赖粘性摩擦力矩，滑行停止；
						if (m_actualJointVel[i] * m_actualPreviousJointVel[i] <= 0)
=======
						coeffCoulomb = fabs(m_actualJointVel[i]) > 0.1 ? 2.0 : 0;		
						coeffViscous = fabs(m_actualJointVel[i]) > 0.1 ? 10.0 : 2.5;		
						m_driveBackTorques[i] = 1.0 * m_frictionModel.calculateFrictionTorque(i, m_actualJointVel[i], coeffCoulomb, coeffViscous);			
						if (m_actualJointVel[i] * m_actualPreviousJointVel[i] < 0)
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117
							m_overZeroVelCount[i]++;


						if (fabs(m_actualJointVel[i]) < 0.005 || m_currentDriveBackTime > 0.25 || m_overZeroVelCount[i] * m_timeStep >= 0.05)
						{
							m_driveBackStatusVector[i] = status_statePosition;
						}
						break;

					default:
					case status_statePosition:
						m_driveBackTorques[i] = 0;
						m_meanDriveBackTorque[i].reset();
						break;
					}


					m_meanDriveBackTorque[i].updateFilteredData(m_driveBackTorques[i], m_driveBackTorques[i]);
				}

				if (m_driveBackStatusVector[i] != status_statePosition)		
				{
					driveBackFlag = true;
				}
			}
<<<<<<< HEAD
			//std::cout << "joint2:"<<m_currentDriveBackTime<<","<<m_sensedRawTorque[1]<<","<< m_driveBackStatusVector[1]<<"," << (m_actualJointPosition[1] - m_driveBackStartPosition[1]) * EcDEG2RAD << "," << m_actualJointVel[1] * EcDEG2RAD << std::endl;
			bool isOverJointMotionLimit = false;
			for (int i = 0; i < numofJoints; i++)
			{
				if (fabs(m_actualJointPosition[i] - m_driveBackStartPosition[i]) > m_allowBackDistance)
					isOverJointMotionLimit = true;
			}

			if (m_currentDriveBackTime > m_allowMotionDuration || isOverJointMotionLimit || (!driveBackFlag && m_currentDriveBackTime > 0.05) || errorStatus)
=======
			bool isOverJointMotionLimit = false;
			for (int i = 0; i < m_NumJoints; i++)
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117
			{
				if (fabs(m_actualJointPosition[i] - m_driveBackStartPosition[i]) > m_allowBackDistance)
					isOverJointMotionLimit = true;
			}

			EcRealVector endEffectorVel(6);
			m_kinBase->calcJacobianEndEffectorVelocity(m_actualJointPosition, m_actualJointVel, endEffectorVel);
			if (m_startActualXYXVel[2]*endEffectorVel[2]>0 && !b_reachZeroVelFlag)
			{
				m_accumuTime += m_updateTimePeriod;
				for (int i = 0; i < m_NumJoints; i++)
					m_accumuActualJointCurrent[i] += m_loopSensedCurrent[20][i];
				for (int i = 0; i < 3; i++)
					m_accumuActualXYZPosition[i] += endEffectorVel[i] * m_updateTimePeriod;
			}
			else
			{
				if (!b_reachZeroVelFlag)
				{
					EcFrame T1 = m_kinBase->forwardKinematics(m_collisionDetectivePosition);
					EcFrame T2 = m_kinBase->forwardKinematics(m_actualJointPosition);
					EcVector v = T2.p - T1.p;
					m_accumuActualXYZPosition = { v[0],v[1],v[2] };
				}
				b_reachZeroVelFlag = true;
			}
			

			if (m_currentDriveBackTime > m_allowMotionDuration || isOverJointMotionLimit || (!driveBackFlag && m_currentDriveBackTime > 0.1) || errorStatus)
			{
				b_isDriveBackAssistiveMode = EcFalse;
				return EcTrue;
			}
			m_actualPreviousJointVel = m_actualJointVel;
		}
	}
	return EcFalse;
}

void CHansCollaborativeAlgorithm::setReadyToCloseAssistiveMode()
{
	b_isReadyToCloseMode = true;
	m_currentReadyToCloseAssistiveTime = 0.;
	m_viscousRatioForCloseAssistive = 10;
	m_startCloseAssistiveJointVel = m_actualJointVel;
	b_havedReverseJointVel.assign(m_NumJoints, false);
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

			if (fabs(m_actualJointVel[i]) > 0.1 && !b_havedReverseJointVel[i])		
				m_driveBackTorques[i] = m_frictionModel.calculateFrictionTorqueForCloseAssistive(i, m_actualJointVel[i], fabs(m_actualJointVel[i]) * scale, 10);
			else
				m_driveBackTorques[i] = m_frictionModel.calculateFrictionTorqueForCloseAssistive(i, m_actualJointVel[i], 0.2, 10);

			isCompleteStatus &= (fabs(m_actualJointVel[i]) < 0.05) || b_havedReverseJointVel[i];
		}

		if (isCompleteStatus && m_currentReadyToCloseAssistiveTime < 0.150)		
		{
			m_currentReadyToCloseAssistiveTime = 0.150;
		}

		if ( m_currentReadyToCloseAssistiveTime > 0.2)
		{
			b_isReadyToCloseMode = false;
			return true;
		}
		
		m_currentReadyToCloseAssistiveTime += m_updateTimePeriod;
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
	m_currentTime = 0.0;
	m_currentAssistiveTime = 0.;
	m_AssistiveStartCount = 0;
	m_AssistiveStartSumAccel.assign(m_NumJoints, 0.0);

	b_isDriveBackAssistiveMode = EcFalse;
	b_isDriveBackMode = EcFalse;
	m_driveBackTorques = m_zeroVector;
	m_driveBackStatus = b_falseVector;

	b_isReadyToCloseMode = false;

	m_AssistiveStartJointPosition = m_actualJointPositions;
	m_AssistiveStartSensedTorques = m_SensedJointTorques;

	m_jointMPosition = m_actualJointPositions;
	m_jointSidePosition = m_actualJointPositions;

	m_frictionModel.resetStatus();
}




EcBoolean CHansCollaborativeAlgorithm::checkSafeStartAssistiveMode
(
)
{
	m_currentAssistiveTime += m_updateTimePeriod;
	if ((m_currentAssistiveTime > m_assistiveCheckTime-0.02)
		&& (m_currentAssistiveTime < m_assistiveCheckTime + 0.2))
	{
		m_AssistiveStartCount++;
		for (EcU32 i = 0; i < m_NumJoints; i++)
		{
			m_AssistiveStartSumAccel[i] += m_FilteredActualJointAcc[i];	
			if (m_AssistiveStartCount >= m_AssistiveSafeAccumulateCount[i])
			{
				EcReal mean = fabs(m_AssistiveStartSumAccel[i] / m_AssistiveStartCount);
				if (mean > m_AssistiveSafeStartAccel[i])
				{
					std::cout << " Assistive start error(ACC), joint" << i + 1 << "(acc): " << mean << std::endl;
					return EcTrue;
				}
			}
			if (fabs(m_actualJointPositions[i] - m_AssistiveStartJointPosition[i]) > AssistiveSafeStartJointMotion)
			{
				std::cout << "Assistive Start Error(Position), joint" << i + 1 << std::endl;
				return EcTrue;
			}

			if (fabs(m_SensedJointTorques[i] - m_AssistiveStartSensedTorques[i]) > 30 * m_CoulombFriction[i] && fabs(m_SensedJointTorques[i] - m_AssistiveStartSensedTorques[i]) > 20)
			{
				std::cout << "Assistive Start Error(sensedTorque), joint" << i + 1 << std::endl;
				return EcTrue;
			}
		}
	}

	if (b_previousAssisteErrorState)
	{
		m_motionConstraintScale = 0.5;
		if (m_currentAssistiveTime > 3.0)			
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
		b_previousAssisteErrorState = true;
	}
	m_savedCollisionStopStatus.assign(m_NumJoints, EcFalse);
	m_AssistiveState = assistive_normal;
}

ENAssistiveState CHansCollaborativeAlgorithm::getAssistiveModeMotorCurrentCommands
(
	EcRealVector& motorCurrentCommands,
	EcBooleanVector& jointCollisionStatus
)
{
	motorCurrentCommands.assign(m_NumJoints, 0.0);

	calculateDisturbanceTorques(
		m_FilteredMotorCurrents,
		m_FilteredJointPositions,
		m_FilteredJointVelocities,
		m_FilteredJointAccelerations,
		m_SensedJointTorques,
		m_EstimatedJointTorques,
		m_DisturbanceJointTorques
	);

	if (checkCloseDriveBackMode())				
	{
		m_AssistiveState = assistive_steadyStatus;
		return assistive_steadyStatus;
	}

	if (checkSafeStartAssistiveMode())			
	{
		m_AssistiveState = assistive_errorRobotState;
		return assistive_errorRobotState;
	}

	if (checkCompleteReadyToCloseAssisitiveMode())
	{
		m_AssistiveState = assistive_finshedReadyToClose;
		m_currentAssistiveTime = 0;
		return assistive_finshedReadyToClose;
	}

	EcRealVector coulombFriction(m_NumJoints), viscousFriction(m_NumJoints);			
	m_frictionModel.calculateCompensateFriction(m_actualJointPositions, m_FilteredActualJointVelocities, m_FilteredActualJointAcc, m_SensedJointTorques,
		b_isDriveBackMode, coulombFriction, viscousFriction);


	EcRealVector gravitationalTorques(m_NumJoints);
	m_dynBase->calculateGravityJointTorques(m_FilteredJointPositions, m_DynamicsLinearParameters, gravitationalTorques);

	m_StateEstimator.filterDisturbanceTorques(m_DisturbanceJointTorques, m_DisturbanceJointTorques);
	updateCollisionStopStatus(m_AssistiveModeCollisionStopThresholds, m_DisturbanceJointTorques, jointCollisionStatus);
	jointCollisionStatus.assign(jointCollisionStatus.size(), EcFalse);


	EcReal cartFrictionScale = 1.0;
	if (calculateCartesianVelocityScale(cartFrictionScale))
	{
		m_AssistiveState = assistive_overJointsRangeLimit;
		return assistive_overJointsRangeLimit;
	}

	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		const EcReal currentPosition = m_FilteredJointPositions[ii];
		const EcReal currentVelocity = m_FilteredActualJointVelocities[ii];

		const EcReal jointSpeed = fabs(currentVelocity);
		const EcReal viscousFrictionPercentage =
			(jointSpeed < m_maxJointVelocitiesInAssistiveMode[ii])
			? pow(fabs(m_maxJointVelocitiesInAssistiveMode[ii] - jointSpeed) / m_maxJointVelocitiesInAssistiveMode[ii], 0.1)
			: 0.0;
		
		EcReal coulombDampCoeff = 1.0;
		EcReal absVelRatio = jointSpeed / (m_maxJointVelocitiesInAssistiveMode[ii] * m_motionConstraintScale);

		if (absVelRatio > 0.9 && absVelRatio <= 1.0)
		{
			coulombDampCoeff = 1 - absVelRatio;				
		}
		else if (absVelRatio > 1.0)
		{
			if (absVelRatio > 2)
				absVelRatio = 2;
			coulombDampCoeff = -std::pow(6.0, 10 * absVelRatio - 10.0);		
		}

		EcReal frictionCompensationTorque = 0.0;
		frictionCompensationTorque = coulombDampCoeff * coulombFriction[ii] + viscousFrictionPercentage * viscousFriction[ii];

		frictionCompensationTorque *= cartFrictionScale * m_constraintsFrictionCompensatoryFactor;			

		const EcReal futurePosition = currentPosition + currentVelocity * m_timeStep;
		EcReal jointLimitZoneFriction = 0;

		if (
			futurePosition >= m_UpperJointLimits[ii] - m_stopBoundary[ii] ||
			futurePosition <= m_LowerJointLimits[ii] + m_stopBoundary[ii]
			)
		{
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
			jointLimitZoneFriction = currentVelocity * jointLimitViscousFrictionCoefficient   
				+ pow((1.0 - fabs(distanceToLimit / m_slowDownBoundary[ii])), 0.7) * frictionCompensationTorque;

		}

		if (m_AdjustedTorqueConstants[ii] == 0)
		{
			m_AssistiveState = assistive_errorInitParams;
			return assistive_errorInitParams;
		}

		EcReal compensationTorque = jointLimitZoneFriction
			- 1.0 * (frictionCompensationTorque + gravitationalTorques[ii]);

		if (b_isDriveBackMode) {
			compensationTorque += 1.0 * frictionCompensationTorque;
		}

		if (m_driveBackStatus[ii] || b_isReadyToCloseMode)						
		{
			compensationTorque += m_driveBackTorques[ii];
		}

		EcReal driveBackRatio = 0.8;
		if (b_isDriveBackAssistiveMode && m_driveBackStatusVector[ii]==status_strongDrive)
			driveBackRatio = 0.8;

		
		if (fabs(compensationTorque) > m_MaxActutorTorques[ii] * driveBackRatio)
			motorCurrentCommands[ii] = sign(compensationTorque) * m_MaxActutorCurrents[ii]* driveBackRatio;	
		else
			motorCurrentCommands[ii] = compensationTorque / m_AdjustedTorqueConstants[ii];



		if (std::fabs(motorCurrentCommands[ii]) > m_MaxActutorCurrents[ii] * 1.1* driveBackRatio)
		{
			motorCurrentCommands[ii] = sign(compensationTorque) * m_MaxActutorCurrents[ii];
			jointCollisionStatus[ii] = EcTrue;
			m_AssistiveState = assistive_overJointsCurrent;
			return assistive_overJointsCurrent;
		}
	}
	m_AssistiveState = assistive_normal;

	EcReal wholeCurrent = 0;
	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
		wholeCurrent += fabs(motorCurrentCommands[ii]);

	EcReal currentRatio = wholeCurrent/ m_maxAllowControlBoxCurrent;
	if (currentRatio > 1.0)
		for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
			motorCurrentCommands[ii] = motorCurrentCommands[ii] * (1.0 / currentRatio) * 0.99;			

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
	motorCurrentCommands.assign(m_NumJoints, 0.0);

	EcRealVector coulombFriction(m_NumJoints);
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
		friction = m_ViscousFrictionCoefficient[ii] * m_FilteredJointVelocities[ii];

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
	motorCurrentCommands.assign(m_NumJoints, 0.0);


	m_dynBase->calculateEstimateJointToqrues(m_FilteredCommandJointPositions,
		m_FilteredCommandJointVelocities,
		m_FilteredCommandJointAccelerations,
		m_DynamicsLinearParameters,
		computeTorque);

	for (EcU32 i = 0; i < m_NumJoints; i++)
	{
		computeTorque[i] *= compensateRatio;
	}

	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		if (m_AdjustedTorqueConstants[ii] == 0)
		{
			continue;
		}
		EcReal viscousFriction = 0.;
		motorCurrentCommands[ii] = (computeTorque[ii] + viscousFriction) / m_AdjustedTorqueConstants[ii];
	}
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
	motorCurrentCommands.assign(m_NumJoints, 0.0);

	m_compenasteDynParams = m_DynamicsLinearParameters;

	if (b_newPayLoadStatus)
	{
		EcReal mass = m_prePayloadMass + (m_payloadMass - m_prePayloadMass) * m_payloadCount / 50.0;
		m_payloadCount++;
		EcReal x, y, z, lx, ly, lz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
		x = m_centerofMass[0];
		y = m_centerofMass[1];
		z = m_centerofMass[2];

		lx = mass * x;	ly = mass * y;	lz = mass * z;
		EcRealVector payloadPara = { 0, 0, 0, 0, 0, 0, lx, ly, lz, mass };
		for (EcU32 ii = 0; ii < 10; ii++)
		{
			m_compenasteDynParams[13 * (m_NumJoints - 1) + ii] = m_DynamicsParameters[13 * (m_NumJoints - 1) + ii] + payloadPara[ii];
		}

		if (m_payloadCount >= 50)
			b_newPayLoadStatus = false;

	}

	EcRealVector jointAcc = jointAcceleration;

	if (m_NumJoints >= 6 && m_AssistiveModeCollisionStopThresholds[m_NumJoints-1] != 200)
	{
		jointAcc = m_FilteredCommandJointAccelerations;

		m_dynBase->calculateEstimateJointToqrues(jointPosition, jointVelocity, jointAcc, m_compenasteDynParams, computeTorque);
		EcRealVector noAccTorque(m_NumJoints);
		m_dynBase->calculateEstimateJointToqrues(jointPosition,	jointVelocity,m_zeroVector, m_compenasteDynParams,noAccTorque);

		for (int i = 0; i < m_NumJoints; i++)
		{
			computeTorque[i] = computeTorque[i] + (computeTorque[i] - noAccTorque[i]) * (m_feedForwardAccTorqueRatio[i] - 1.0);		
			m_FeedAccTorque[i] = (computeTorque[i] - noAccTorque[i]) * m_feedForwardAccTorqueRatio[i];		
		}

	}
	else
	{
		for(int i=0;i<m_NumJoints;i++)
			if (fabs(jointVelocity[i]) < 0.018)
				jointAcc[i] = 0;		
		
		m_dynBase->calculateEstimateJointToqrues(jointPosition,
			jointVelocity,
			jointAcc,
			m_compenasteDynParams,
			computeTorque);

	}

	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		if (m_AdjustedTorqueConstants[ii] == 0)
		{
			continue;
		}
		computeTorque[ii] += m_jointTorqueFromEEForce[ii];
		motorCurrentCommands[ii] = (computeTorque[ii]) / m_AdjustedTorqueConstants[ii];

		if (fabs(motorCurrentCommands[ii] - m_previousFeedforwardCurrent[ii]) > 0.8)
		{
			motorCurrentCommands[ii] = m_previousFeedforwardCurrent[ii] + sign(motorCurrentCommands[ii] - m_previousFeedforwardCurrent[ii]) * 0.8;
		}

		if (fabs(motorCurrentCommands[ii]) > m_MaxActutorCurrents[ii])
		{
			motorCurrentCommands[ii] = sign(motorCurrentCommands[ii]) * m_MaxActutorCurrents[ii];
		}
	}

	m_previousFeedforwardCurrent = motorCurrentCommands;
	m_previousJointVelocity = jointVelocity;
}

void CHansCollaborativeAlgorithm::getGravityTorqueCurrentCommands
(
	const EcReal compensateRatio,
	EcRealVector& motorCurrentCommands
)
{
	EcRealVector gravitationalTorques(m_NumJoints);
	motorCurrentCommands.assign(m_NumJoints, 0.0);

	m_dynBase->calculateGravityJointTorques(m_FilteredCommandJointPositions, m_DynamicsLinearParameters, gravitationalTorques);


	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		if (m_AdjustedTorqueConstants[ii] == 0)
		{
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
	EcRealVector gravitationalTorques(m_NumJoints);
	motorCurrentCommands.assign(m_NumJoints, 0.0);
	m_dynBase->calculateGravityJointTorques(jointPosition, m_DynamicsLinearParameters, gravitationalTorques);

	for (EcU32 ii = 0; ii < m_NumJoints; ++ii)
	{
		if (m_AdjustedTorqueConstants[ii] == 0)
		{
			continue;
		}
		EcReal viscousFriction = m_ViscousFrictionCoefficient[ii] * m_FilteredCommandJointVelocities[ii];
		motorCurrentCommands[ii] = compensateRatio * (gravitationalTorques[ii] + viscousFriction) / m_AdjustedTorqueConstants[ii];
	}

}

void CHansCollaborativeAlgorithm::getFlexibleCompensateGravityTorque
(
	const EcRealVector& jointPosition,
	EcRealVector& gravTorque
)
{
	gravTorque = m_EstimatedGravityTorques;
}

void CHansCollaborativeAlgorithm::getGravityAndCoriolisCentrifugalTorque
(
	const EcRealVector& jointPosition,
	const EcRealVector& jointVelocity,
	EcRealVector& torque
)
{
	EcRealVector jointAcc(m_NumJoints, 0.0);
	m_dynBase->calculateEstimateJointToqrues(jointPosition, jointVelocity, jointAcc, m_compenasteDynParams, torque);
	for (int i = 0;i < m_NumJoints;i++)
	{
		torque[i] -= m_CoulombFriction[i] * dynamicsBase::sign(jointVelocity[i]) + m_ViscousFrictionCoefficient[i] * jointVelocity[i];
	}

}

void CHansCollaborativeAlgorithm::getJointInertia
(
	const EcRealVector& jointPosition,
	EcRealVector& inertia
) {
	EcRealVector jointAcc(m_NumJoints, 0.0),jointVel(m_NumJoints,0.0);
	EcRealVector gravitationalTorques(m_NumJoints),tau(m_NumJoints);
	m_dynBase->calculateGravityJointTorques(jointPosition, m_DynamicsLinearParameters, gravitationalTorques);
	for (int i = 0; i < m_NumJoints; i++)
	{
		jointAcc.assign(m_NumJoints, 0);
		jointAcc[i] = 1;
		m_dynBase->calculateEstimateJointToqrues(jointPosition, jointVel, jointAcc, m_DynamicsLinearParameters, tau);
		inertia[i] = tau[i] - gravitationalTorques[i];
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
			if (fabs(velocityThreshold[ii] - 0.00436332) > 0.000001)
			{
				m_lowVelocityThresholds[ii] = 0.00436332;
				m_feedForwardAccTorqueRatio[ii] = velocityThreshold[ii]*rad2deg;
				m_AssistiveModeCollisionStopThresholds[m_NumJoints - 1] = 199;
			}
			else
			{
				m_lowVelocityThresholds[ii] = fabs(velocityThreshold[ii]);
				m_feedForwardAccTorqueRatio[ii] = 1.0;

			}
		}
	}
}

void CHansCollaborativeAlgorithm::setVibrationPeriod()
{
	m_vibrationPeriod.assign(m_NumJoints, 0.0);
	if (m_NumJoints > 0) m_vibrationPeriod[0] = 2 * 3.1415926 * 6;
	if (m_NumJoints > 1) m_vibrationPeriod[1] = 2 * 3.1415926 * 5;
	if (m_NumJoints > 2) m_vibrationPeriod[2] = 2 * 3.1415926 * 5.6;
	if (m_NumJoints > 3) m_vibrationPeriod[3] = 2 * 3.1415926 * 7;
	if (m_NumJoints > 4) m_vibrationPeriod[4] = 2 * 3.1415926 * 7.5;
	if (m_NumJoints > 5) m_vibrationPeriod[5] = 2 * 3.1415926 * 8;
	if (m_NumJoints > 6) m_vibrationPeriod[6] = 2 * 3.1415926 * 6.5;
}


EcBoolean CHansCollaborativeAlgorithm::calculateMaxRectifyEstimateJointTorques
(
)
{
	EcRealVector q(m_NumJoints, 0.0);
	if (m_NumJoints >= 2) q[1] = 1.57;
	EcRealVector dq(m_NumJoints, 0.0);
	if (m_NumJoints >= 2) dq[1] = -0.1;
	EcRealVector ddq(m_NumJoints, 0.0);
	if (m_NumJoints >= 2) ddq[1] = -m_maxJointAccelerations[1];
	if (m_NumJoints >= 5) ddq[m_NumJoints - 2] = m_maxJointAccelerations[m_NumJoints - 2];

	EcReal gx = m_gx;
	EcReal gy = m_gy;
	EcReal gz = m_gz;
	m_gx = 0;
	m_gy = 0;
	m_gz = -9.81;

	EcBoolean ret = m_dynBase->calculateEstimateJointToqrues(q, dq, ddq, m_DynamicsLinearParameters, m_maxJointTorques);		
	m_maxJointTorques[0] = fabs(m_maxJointTorques[1]);	
	m_maxJointTorques[1] = fabs(m_maxJointTorques[1]);
	m_maxJointTorques[2] = fabs(m_maxJointTorques[2]);	
	if (m_NumJoints > 3) m_maxJointTorques[3] = fabs(m_maxJointTorques[2]);
	if (m_NumJoints >= 6) {
		m_maxJointTorques[m_NumJoints - 2] = fabs(m_maxJointTorques[m_NumJoints - 2]);	
		m_maxJointTorques[m_NumJoints - 1] = fabs(m_maxJointTorques[m_NumJoints - 2]);
	}


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
	cartScale = 1.0;
	return false;
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
}

bool CHansCollaborativeAlgorithm::calculatePowerAndMomentum(
	const EcRealVector& jointVoltages,
	EcReal& electircPower, 
	EcReal& physicsPower,  
	EcReal& momentum,	   
	EcRealVector& jointPowers,
	EcBoolean isUsingCommandVel
)
{
	EcRealVector jointMomentum(m_NumJoints);
	if(isUsingCommandVel)
		m_momentumObserver->calculateGeneralizeMomentum(m_FilteredJointPositions, m_FilteredJointVelocities, jointMomentum);
	else
		m_momentumObserver->calculateGeneralizeMomentum(m_FilteredJointPositions, m_FilteredActualJointVelocities, jointMomentum);

	momentum = 0.0;
	physicsPower = 0.0;
	electircPower = 0.0;

	for (int i = 0; i < m_NumJoints; i++)
	{
		momentum += jointMomentum[i];
		jointPowers[i] = (m_SensedJointTorques[i] - (m_FilteredActualJointVelocities[i] * m_ViscousFrictionCoefficient[i] + sign(m_FilteredActualJointVelocities[i]) * m_CoulombFriction[i])) * m_FilteredActualJointVelocities[i];
		physicsPower += jointPowers[i];
	}
	return true;
}



bool CHansCollaborativeAlgorithm::calculatePowerAndMomentumConstraints
(
	const EcReal userDefineOverride,
	const EcRealVector& jointVoltages,
	EcBoolean& constraintActive,						
	EcReal& velFactorConstraint,					
	EcReal& accFactorConstraint,					
	EcReal& electircPower,						
	EcReal& physicsPower,							
	EcReal& momentum								
)
{
	accFactorConstraint = 1.0;
	velFactorConstraint = 1.0;

	constraintActive = false;
	if (userDefineOverride < 0.0001)
		return false;

	EcRealVector jointPowers(m_NumJoints);
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
	{   
		velFactorConstraint = m_velFactorConstraint;
		constraintActive = true;
	}

	for (int i = 0; i < m_NumJoints; i++)
	{
		if (jointPowers[i] > m_maxJointPowers[i])
		{
			velFactorConstraint = std::min(velFactorConstraint, m_maxJointPowers[i] / jointPowers[i]);
		}
	}


	if (b_constraintPowerStatus && m_velFactorConstraint < velFactorConstraint)
	{
		velFactorConstraint = m_velFactorConstraint - 0.01;
	}


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

	return true;
}

ENMotionConstraintStatus CHansCollaborativeAlgorithm::monitorMotionConstraintsStatus(
	EcBoolean isAssistiveMode)
{
	if (m_currentTime < 0.1)
		return constraint_normal;

	EcReal momentum, physicalPower, electricPower;
	EcRealVector jointPowers(m_NumJoints);
	calculatePowerAndMomentum(m_jointVoltages, electricPower, physicalPower, momentum, jointPowers);


	if (isAssistiveMode)
	{
		if (!(m_AssistiveState == assistive_normal || m_AssistiveState == assistive_steadyStatus || m_AssistiveState == assistive_finshedReadyToClose))
		{
			if (m_AssistiveState == assistive_steadyStatus)
			{	
				m_AssistiveState = assistive_severCollision;
				return constraint_normal;
			}
			return constraint_errorStatus;
		}

		for (EcSizeT ii = 0; ii < m_NumJoints; ii++)
		{
			EcReal tempVel = 0.0;
			if (b_isDriveBackMode)
				tempVel = m_maxJointVeloctiy[ii];
			else
				tempVel = m_maxJointVelocitiesInAssistiveModeForErr[ii];

			if (fabs(m_FilteredActualJointVelocities[ii]) > tempVel * 1.25)
			{
				return constraint_overJointVelocitiesLimit;
			}
		}
		if (fabs(momentum) > MaxAllowMomentumInAssistiveMode * 5 && !b_isDriveBackMode)
		{
			return constraint_overMomentumLimit;
		}


		if (physicalPower > MaxAllowPowerInAssistiveMode * 5 && !b_isDriveBackMode)
		{
			return constraint_overPowerLImit;
		}

		m_constraintsFrictionCompensatoryFactor = 1.0;
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
			if (fabs(m_FilteredActualJointVelocities[ii]) > m_maxJointVeloctiy[ii] * 1.25)
			{
				b_logFlag = false;
				return constraint_overJointVelocitiesLimit;
			}

		}

		if (fabs(momentum) > (m_maxConstraintMomentum) * 6)
		{
			return constraint_overMomentumLimit;
		}


		if (physicalPower > m_maxConstraintPower * 6)
		{
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
	EcReal& rotationofMounting, 
	EcReal& tiltofMounting,		
	EcRealVector& upperJointLimits,
	EcRealVector& lowerJointLimits,
	EcRealVector& dynamicsParams,
	EcRealVector& collisionStopThresholds,			 
	EcRealVector& collisionStopInMomentumThresholds, 
	EcRealVector& assistiveModeCollisionStopThresholds,
	EcRealVector& frictionCompensatoryFactor,	
	EcRealVector& frictionCompensatoryFactorII, 
	EcRealVector& lowVelocityThreshold,			
	EcRealVector& maxJointVelocity,
	EcReal& maxPowerConstraint,		
	EcReal& maxMomentumConstraint,	
	EcReal& distanceLimitInDriveBack, 
	EcReal& timeDurationInDriveBack,	
	EcReal& mass,
	EcRealVector& centerofMass, 
	bool& isAssistiveMode,
	int& drivebackMode,
	EcReal& assistiveCheckTime 
)
{
	if (m_kinParams.size() > 0)
	{
		kinParams = m_kinParams;
		for (int i = 0; i < kinParams.size(); i++)
			kinParams[i] *= 1000.0;
	}
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

	for (int i = 0; i < upperJointLimits.size(); i++)
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
