#include "cobotInterface.h"
#include "../src/HMCollaborativeRobotAlgorithm.h"
//#include "algorithmInterface.h"

#ifdef __cplusplus
extern "C" {

	std::shared_ptr<CHansCollaborativeAlgorithm> m_cobotAlgorithm;
	int m_robotType = 0;
	int m_numJoints = 6;

	bool b_enableDualAugentedAssistiveMode = false;

	EcRealVector m_jointSidePosition, m_motorSidePosition;

	int getCobotAlgorithmVersion()
	{
		return CHansCollaborativeAlgorithm::getVersion();
	}



	void transREALDegToVectorRad(RTS_IEC_LREAL* realValue, EcRealVector& realVector)
	{
		realVector.assign(m_numJoints, 0.0);
		for (int i = 0; i < m_numJoints; i++)
		{
			realVector[i] = realValue[i] * KDL::deg2rad;
		}
	}

	void transVectorRadToREALDeg(EcRealVector& realVector, RTS_IEC_LREAL* realValue, double maxLimit = 10000)
	{
		for (int i = 0; i < m_numJoints; i++)
		{
			realValue[i] = realVector[i] * KDL::rad2deg;

			if (fabs(realValue[i]) > maxLimit)
				realValue[i] = sign(realValue[i]) * maxLimit;
		}
	}

	void transREALToVector(RTS_IEC_LREAL* realValue, EcRealVector& realVector)
	{
		realVector.assign(m_numJoints, 0.0);
		for (int i = 0; i < m_numJoints; i++)
		{
			realVector[i] = realValue[i];
		}
	}

	void transVectorToREAL(EcRealVector& realVector, RTS_IEC_LREAL* realValue, int size, double maxLimit = 10000)
	{
		for (int i = 0; i < size; i++)
		{
			realValue[i] = realVector[i];

			if (fabs(realValue[i]) > maxLimit)
				realValue[i] = sign(realValue[i]) * maxLimit;
		}
	}

	void transVectorToINT(EcU32Vector& intVector, RTS_IEC_INT* intValue, int size)
	{
		for (int i = 0; i < size; i++)
		{
			intValue[i] = intVector[i];
		}
	}

	void transUI32ToVector(RTS_IEC_INT* UI32Value, EcU32Vector& U32Vector)
	{
		U32Vector.assign(m_numJoints, 0);
		for (int i = 0; i < m_numJoints; i++)
		{
			U32Vector[i] = UI32Value[i];
		}
	}

	void transBoolVectorToRTSBool(EcBooleanVector boolVector, RTS_IEC_BOOL* boolValue)
	{
		for (int i = 0; i < m_numJoints; i++)
		{
			boolValue[i] = boolVector[i];
		}
	}

	void transBoolVectorToRTSBoolPointer(EcBooleanVector boolVector, RTS_IEC_BOOL* boolValue)
	{
		for (int i = 0; i < m_numJoints; i++)
		{
			boolValue[i] = boolVector[i];
		}
	}

	void transVectorToREALPointer(EcRealVector vector, RTS_IEC_LREAL* realValue)
	{
		for (int i = 0; i < m_numJoints; i++)
		{
			realValue[i] = vector[i];
		}
	}


	void initializeCollaborativeAlgorithm(RTS_IEC_LREAL* jointPosition,
		RTS_IEC_LREAL updatePeriod,
		RTS_IEC_INT robotType,				/* VAR_INPUT */	/* 0:elfin; 1:UR; */
		RTS_IEC_LREAL* kinParams		/* VAR_INPUT */	/* elfin[d1,d4,d6,a2]; UR[d1,d4,d5,d6,a2,a3]; */)
	{
		m_robotType = robotType;
		m_numJoints = (m_robotType == 20) ? 7 : 6;
		EcRealVector jointPos;
		transREALDegToVectorRad(jointPosition, jointPos);
		m_cobotAlgorithm = CHansCollaborativeAlgorithm::create(jointPos, updatePeriod, robotType);

		m_jointSidePosition.assign(m_numJoints, 0.0);
		m_motorSidePosition.assign(m_numJoints, 0.0);

		EcRealVector KineParams = { kinParams[0] * 0.001, kinParams[1] * 0.001, kinParams[2] * 0.001, kinParams[3] * 0.001, kinParams[4] * 0.001, kinParams[5] * 0.001, kinParams[6] * 0.001, kinParams[7] * 0.001,0,0 };
		std::cout << "(cobotKinDual1) " << CHansCollaborativeAlgorithm::getVersion() << ", " << robotType << "," << updatePeriod << "  ,  " << KineParams[0] << "," << KineParams[1] << "," << KineParams[2] << "," << KineParams[3] << "," << KineParams[4] << "," << KineParams[5] << "," << KineParams[6] << "," << KineParams[7] << std::endl;
		m_cobotAlgorithm->setRobotDHParameters(KineParams);
	}

	void set15066StrategyInterface(RTS_IEC_BOOL enable)
	{
		m_cobotAlgorithm->set15066Startegy((bool)enable);
	}



	RTS_IEC_BOOL updateStateEstimates
	(
		RTS_IEC_LREAL* commandJointPositions,
		RTS_IEC_LREAL* jointPositions,
		RTS_IEC_LREAL* motorCurrents,
		RTS_IEC_LREAL currentTime
	)
	{
		EcRealVector commandJointPos, jointPos, motorCurr;
		transREALDegToVectorRad(commandJointPositions, commandJointPos);
		transREALDegToVectorRad(jointPositions, jointPos);
		transREALToVector(motorCurrents, motorCurr);
		return m_cobotAlgorithm->updateStateEstimates(commandJointPos, jointPos, motorCurr, currentTime);
	}

	void updateForceSensorData(
		RTS_IEC_BOOL enable,				/* VAR_INPUT */	/* 是否开启末端力传感器的碰撞检测功能； */
		RTS_IEC_LREAL calibedForce[6]		/* VAR_INPUT */	/* 获取标定后的力传感器数据，当前为：GetFTValueInSelectedFrame */
	)
	{
		EcVector calibed = { calibedForce[0],calibedForce[1],calibedForce[2] };
		m_cobotAlgorithm->updateForceSensorData(enable, calibed);
	}

	void updateForceSensorForFoceControl(
		RTS_IEC_BOOL enable,				/* VAR_INPUT */	/* 是否开启末端力传感器的对关节电流前馈的作用； */
		RTS_IEC_LREAL calibedForce[6]		/* VAR_INPUT */	/* 获取标定后的力传感器数据，工具坐标系 */
	)
	{
		EcRealVector force = { calibedForce[0],calibedForce[1],calibedForce[2],calibedForce[3],calibedForce[4],calibedForce[5] };

		m_cobotAlgorithm->updateForceSensorForFeedForward(enable, force);

	}



	RTS_IEC_BOOL checkForCollision
	(
		RTS_IEC_BOOL* jointCollisionStatus
	)
	{
		EcBooleanVector collisionStatus(m_numJoints);
		ENCollisionType type;
		EcBoolean ret = m_cobotAlgorithm->checkForCollision(collisionStatus, type);
		transBoolVectorToRTSBoolPointer(collisionStatus, jointCollisionStatus);
		return ret;
	}

	RTS_IEC_BOOL getEstimatedState
	(
		RTS_IEC_LREAL* jointPostions,
		RTS_IEC_LREAL* jointVelocities,
		RTS_IEC_LREAL* jointAccelerations,
		RTS_IEC_LREAL* motorCurrents,
		RTS_IEC_LREAL* sensedTorques,
		RTS_IEC_LREAL* estimatedTorques,
		RTS_IEC_LREAL* disturbanceTorques
	)
	{
		EcRealVector jointPos(m_numJoints), jointVel(m_numJoints), jointAccel(m_numJoints),
			motorCurr(m_numJoints), sensedTor(m_numJoints), estimatedTor(m_numJoints), disturbTor(m_numJoints);
		EcBoolean ret = m_cobotAlgorithm->getEstimatedState(
			jointPos, jointVel, jointAccel, motorCurr, sensedTor, estimatedTor, disturbTor);

		transVectorRadToREALDeg(jointPos, jointPostions);
		transVectorRadToREALDeg(jointVel, jointVelocities);
		transVectorRadToREALDeg(jointAccel, jointAccelerations);

		transVectorToREAL(motorCurr, motorCurrents, m_numJoints);
		transVectorToREAL(sensedTor, sensedTorques, m_numJoints);
		transVectorToREAL(estimatedTor, estimatedTorques, m_numJoints);
		transVectorToREALPointer(disturbTor, disturbanceTorques);
		return ret;
	}

	void setAssistDualEncoderFlag(
		RTS_IEC_BOOL flag
	)
	{
		b_enableDualAugentedAssistiveMode = flag;
		m_cobotAlgorithm->setAssistDualEncoderFlag(flag);
	}

	void setDualDiffEncoderThd(RTS_IEC_LREAL thd[]) {
		std::cout << "set dual thd:";
		for (int i = 0; i < m_numJoints; i++) {
			std::cout << thd[i] << (i == m_numJoints - 1 ? "" : ",");
		}
		std::cout << std::endl;

		EcRealVector vThd(m_numJoints);
		for (int i = 0; i < m_numJoints; i++) {
			vThd[i] = thd[i] * 0.001;
		}
		m_cobotAlgorithm->setDualDiffEncoderThd(vThd);
	}

	void updateDualEncoderJointPosition(
		RTS_IEC_LREAL motorSideJointPosition[],	/* VAR_INPUT */	/* actual motor side  joint position(deg) */
		RTS_IEC_LREAL jointSideJointPosition[]	/* VAR_INPUT */	/* actual joint side joint position(deg) */
	)
	{
		EcRealVector motorSide, jointSide, vMotorSide, vJointSide;
		transREALDegToVectorRad(motorSideJointPosition, motorSide);
		transREALDegToVectorRad(jointSideJointPosition, jointSide);
		m_motorSidePosition = motorSide;
		m_jointSidePosition = jointSide;

		transREALToVector(motorSideJointPosition, vMotorSide);
		transREALToVector(jointSideJointPosition, vJointSide);
		m_cobotAlgorithm->updateDualEncoderPosition(vJointSide, vMotorSide);
	}


	RTS_IEC_INT monitorMotionConstraintStatus
	(
		RTS_IEC_BOOL isAssistiveMode
	)
	{
		return m_cobotAlgorithm->monitorMotionConstraintsStatus(isAssistiveMode);
	}

	void resetCobotStatus()
	{
		m_cobotAlgorithm->resetCobotStatus();
	}



	void setDynamicsLinearParametersCobot
	(
		RTS_IEC_LREAL* params
	)
	{
		int totalParams = m_numJoints * 13;
		EcRealVector dynParams(totalParams);
		for (int i = 0; i < totalParams; ++i)
		{
			dynParams[i] = params[i];
		}
		m_cobotAlgorithm->setDynamicsLinearParameters(dynParams);
	}

	RTS_IEC_BOOL setFilterTimeConstant
	(
		RTS_IEC_LREAL* jointsPosition,
		RTS_IEC_LREAL* filterTimeConstant
	)
	{
		EcRealVector jointPos, timeConstant;
		transREALDegToVectorRad(jointsPosition, jointPos);
		transREALToVector(filterTimeConstant, timeConstant);
		return m_cobotAlgorithm->setFilterTimeConstant(jointPos, timeConstant, timeConstant);
	}

	void setRobotMountingAnglesCobot
	(
		const RTS_IEC_LREAL rotation,
		const RTS_IEC_LREAL tilt
	)
	{
		m_cobotAlgorithm->setRobotMountingAngles(rotation * KDL::deg2rad, tilt * KDL::deg2rad);
	}

	void setActutorMaxCurrentLimits
	(
		RTS_IEC_LREAL* maxCurrents
	)
	{
		EcRealVector maxCurr;
		transREALToVector(maxCurrents, maxCurr);
		m_cobotAlgorithm->setActutorMaxCurrentLimits(maxCurr);
	}

	void setControlBoxAllowCurrentLimit
	(
		RTS_IEC_LREAL maxCurrent
	)
	{
		m_cobotAlgorithm->setControlBoxAllowCurrentLimit(maxCurrent);
	}

	RTS_IEC_BOOL setPayloadMassPropertiesCobot
	(
		const RTS_IEC_LREAL mass,
		RTS_IEC_LREAL* centerofMass
	)
	{
		EcRealVector centerMass = { centerofMass[0] * 0.001, centerofMass[1] * 0.001, centerofMass[2] * 0.001 };
		return m_cobotAlgorithm->setPayloadMassProperties(mass, centerMass);
	}

	void setActutorTorqueConstants
	(
		RTS_IEC_LREAL* torqueConstant,
		RTS_IEC_LREAL* gearRatio,
		RTS_IEC_LREAL* maxEfficiency
	)
	{
		EcRealVector torConst, maxEff;
		EcRealVector gearRatios;
		transREALToVector(torqueConstant, torConst);
		transREALToVector(maxEfficiency, maxEff);
		transREALToVector(gearRatio, gearRatios);
		m_cobotAlgorithm->setActutorTorqueConstants(torConst, gearRatios, maxEff);
	}


	void setCollisionStopThresholds
	(
		RTS_IEC_LREAL* collisionStopThresholds
	)
	{
		EcRealVector stopThresholds;
		transREALToVector(collisionStopThresholds, stopThresholds);
		m_cobotAlgorithm->setCollisionStopThresholds(stopThresholds);
	}

	void setAssistiveModeCollisionStopThresholds
	(
		RTS_IEC_LREAL* assistiveModeCollisionStopThresholds
	)
	{
		EcRealVector stopThresholds;
		transREALToVector(assistiveModeCollisionStopThresholds, stopThresholds);
		m_cobotAlgorithm->setAssistiveModeCollisionStopThresholds(stopThresholds);
	}

	void setCollaborativeJointSpaceLimits
	(
		RTS_IEC_LREAL* upperJointLimits,
		RTS_IEC_LREAL* lowerJointLimits
	)
	{
		std::cout << "Upper joint's Limit:";
		for (int i = 0; i < m_numJoints; i++) std::cout << upperJointLimits[i] << (i == m_numJoints - 1 ? "" : ",");
		std::cout << std::endl;

		std::cout << "Lower joint's Limit:";
		for (int i = 0; i < m_numJoints; i++) std::cout << lowerJointLimits[i] << (i == m_numJoints - 1 ? "" : ",");
		std::cout << std::endl;

		EcRealVector upper, lower;
		transREALDegToVectorRad(upperJointLimits, upper);
		transREALDegToVectorRad(lowerJointLimits, lower);
		m_cobotAlgorithm->setCollaborativeJointSpaceLimits(upper, lower);
	}

	void setFrictionCompensatoryFactor
	(
		RTS_IEC_LREAL* frictionCompensatoryFactor
	)
	{
		EcRealVector tempValue;
		transREALToVector(frictionCompensatoryFactor, tempValue);
		std::cout << "CompensateFrictionCoeff:";
		for (int i = 0; i < m_numJoints; i++) std::cout << frictionCompensatoryFactor[i] << (i == m_numJoints - 1 ? "" : ",");
		std::cout << std::endl;
		m_cobotAlgorithm->setFrictionCompensatoryFactor(tempValue);
	}

	void setFrictionCompensatoryFactorII
	(
		RTS_IEC_LREAL* factor
	)
	{
		std::cout << "DynFrictionCoeff:";
		for (int i = 0; i < m_numJoints; i++) std::cout << factor[i] << (i == m_numJoints - 1 ? "" : ",");
		std::cout << std::endl;
		EcRealVector temp(m_numJoints);
		transREALToVector(factor, temp);
		m_cobotAlgorithm->setStartCompensateFrictionFactor(temp);
		m_cobotAlgorithm->setDynFrictionCompensatoryFactor(temp);
	}



	void setLowVelocityThreshold
	(
		RTS_IEC_LREAL* velocityThreshold
	)
	{
		EcRealVector tempValue;
		transREALDegToVectorRad(velocityThreshold, tempValue);
		m_cobotAlgorithm->setLowVelocityThreshold(tempValue);
	}

	RTS_IEC_BOOL setMaxJointVelocity
	(
		RTS_IEC_LREAL* maxJointVelocity
	)
	{
		EcRealVector tempValue;
		transREALDegToVectorRad(maxJointVelocity, tempValue);
		m_cobotAlgorithm->setMaxJointVelocity(tempValue);
		return 1;
	}

	void setStartAssistiveMode
	(
	)
	{
		m_cobotAlgorithm->setStartAssistiveMode();
	}

	void setReadyToCloseAssistiveMode
	(
	)
	{
		m_cobotAlgorithm->setReadyToCloseAssistiveMode();
	}


	RTS_IEC_INT getAssistiveModeMotorCurrentCommands
	(
		RTS_IEC_LREAL* motorCurrentCommands,
		RTS_IEC_BOOL* jointCollisionStatus
	)
	{
		EcRealVector tempValue(m_numJoints);
		EcBooleanVector tempValue2(m_numJoints);

		RTS_IEC_INT ret = m_cobotAlgorithm->getAssistiveModeMotorCurrentCommands(tempValue, tempValue2);
		transVectorToREALPointer(tempValue, motorCurrentCommands);
		transBoolVectorToRTSBoolPointer(tempValue2, jointCollisionStatus);
		return ret;
	}

	void getComputeTorqueCurrentCommands
	(
		const RTS_IEC_LREAL compensateRatio,
		RTS_IEC_LREAL* jointPosition,
		RTS_IEC_LREAL* jointVelocity,
		RTS_IEC_LREAL* jointAcceleration,
		RTS_IEC_LREAL* motorCurrentCommands
	)
	{
		EcRealVector tempValue(m_numJoints), position(m_numJoints), vel(m_numJoints), acc(m_numJoints);
		transREALDegToVectorRad(jointPosition, position);
		transREALDegToVectorRad(jointVelocity, vel);
		m_cobotAlgorithm->getComputeTorqueCurrentCommands(compensateRatio, position, vel, acc, tempValue);
		transVectorToREALPointer(tempValue, motorCurrentCommands);
	}

	void getGravityTorqueCurrentCommands
	(
		const RTS_IEC_LREAL compensateRatio,
		RTS_IEC_LREAL* jointPosition,
		RTS_IEC_LREAL* motorCurrentCommands
	)
	{
		EcRealVector joint(m_numJoints), tempValue(m_numJoints);
		transREALDegToVectorRad(jointPosition, joint);
		m_cobotAlgorithm->getGravityTorqueCurrentCommands(compensateRatio, joint, tempValue);
		transVectorToREALPointer(tempValue, motorCurrentCommands);
	}

	void getFlexibleCompensateTorque
	(
		RTS_IEC_LREAL* jointPosition,
		RTS_IEC_LREAL* gravTorque
	)
	{
		EcRealVector joint(m_numJoints), tempValue(m_numJoints);
		transREALDegToVectorRad(jointPosition, joint);
		m_cobotAlgorithm->getFlexibleCompensateGravityTorque(joint, tempValue);
		transVectorToREALPointer(tempValue, gravTorque);
	}

	void getJointInertia
	(
		RTS_IEC_LREAL* jointPosition,
		RTS_IEC_LREAL* inertia
	)
	{
		EcRealVector joint(m_numJoints), tempValue(m_numJoints);
		transREALDegToVectorRad(jointPosition, joint);
		m_cobotAlgorithm->getJointInertia(joint, tempValue);
		transVectorToREALPointer(tempValue, inertia);
	}

	void getGravityAndCoriolisCentrifugalTorque(RTS_IEC_LREAL* jointPosition, RTS_IEC_LREAL* jointVelocity, RTS_IEC_LREAL* torque)
	{
		EcRealVector joint(m_numJoints), jointVel(m_numJoints), tempValue(m_numJoints);
		transREALDegToVectorRad(jointPosition, joint);
		transREALDegToVectorRad(jointVelocity, jointVel);
		m_cobotAlgorithm->getGravityAndCoriolisCentrifugalTorque(joint, jointVel, tempValue);
		transVectorToREALPointer(tempValue, torque);
	}


	void setActutorDampConstants
	(
		RTS_IEC_LREAL* damp
	)
	{
		EcRealVector temp;
		transREALToVector(damp, temp);
		m_cobotAlgorithm->setActutorDampConstants(temp);
	}

	void setCollisionDriveBackMode(RTS_IEC_INT driveBackMode)
	{

		ENDriveBackMode temp = ENDriveBackMode::mode_PowerOff;
		switch (driveBackMode)
		{
		default:
		case 0:
			temp = ENDriveBackMode::mode_PowerOff; break;
		case 1:
			temp = ENDriveBackMode::mode_ForceFree; break;
		case 2:
			temp = ENDriveBackMode::mode_LimitMotion; break;
		}
		m_cobotAlgorithm->setCollisionDriveBackMode(temp);
	}

	void setCollisionStopInMomentumThresholds
	(
		RTS_IEC_LREAL* thresholds
	)
	{
		EcRealVector temp;
		transREALToVector(thresholds, temp);
		m_cobotAlgorithm->setCollisionStopInMomentumThresholds(temp);
	}


	void setMaxPowerAndMomentumConstraintsCobot(
		const RTS_IEC_LREAL maxPower,
		const RTS_IEC_LREAL maxMomentum,
		RTS_IEC_LREAL* jointPowers)
	{
		std::cout << "Constraints, maxPower: " << maxPower << ", maxMomentum: " << maxMomentum << std::endl;
		EcRealVector temp;
		transREALToVector(jointPowers, temp);
		m_cobotAlgorithm->setMaxPowerAndMomentumConstraints(maxPower, maxMomentum, temp);
	}

	RTS_IEC_BOOL calculatePowerAndMomentumConstraints(
		RTS_IEC_LREAL userDefineOverride,
		RTS_IEC_LREAL* jointVoltages,
		RTS_IEC_BOOL* constraintActive,
		RTS_IEC_LREAL* jointVelFactorConstraint,
		RTS_IEC_LREAL* jointAccFactorConstraint,
		RTS_IEC_LREAL* electircPower,
		RTS_IEC_LREAL* physicsPower,
		RTS_IEC_LREAL* momentum
	)
	{
		EcRealVector volage;
		transREALToVector(jointVoltages, volage);
		bool temp;
		bool ret = m_cobotAlgorithm->calculatePowerAndMomentumConstraints(
			userDefineOverride,
			volage,
			temp,
			*jointVelFactorConstraint,
			*jointAccFactorConstraint,
			*electircPower,
			*physicsPower,
			*momentum);
		*constraintActive = (RTS_IEC_BOOL)temp;
		return ret;
	}

	RTS_IEC_BOOL setStartTimeForCheckAssistiveMode
	(
		RTS_IEC_LREAL assistiveCheckTime
	)
	{
		m_cobotAlgorithm->setStartTimeForCheckAssisitiveMode(assistiveCheckTime);
		return 1;
	}

	RTS_IEC_BOOL setMotionLimitInDriveback
	(
		RTS_IEC_LREAL distanceLimit,
		RTS_IEC_LREAL timeDuration
	)
	{
		distanceLimit *= KDL::deg2rad;
		m_cobotAlgorithm->setMotionLimitInDriveBack(distanceLimit, timeDuration);
		return 1;
	}

	RTS_IEC_BOOL getSettingParams
	(
		cobotParams* params
	)
	{
		EcRealVector kinParams(10);
		EcRealVector actuatorDamp(m_numJoints);
		EcRealVector torqueConstant(m_numJoints);
		EcRealVector gearRatio(m_numJoints);
		EcRealVector maxEfficiency(m_numJoints);
		EcRealVector maxActuatorCurrents(m_numJoints);
		EcReal rotationofMounting;
		EcReal tiltofMounting;
		EcRealVector upperJointLimits(m_numJoints);
		EcRealVector lowerJointLimits(m_numJoints);
		EcRealVector dynamicsParams(m_numJoints * 13);
		EcRealVector collisionStopThresholds(m_numJoints);
		EcRealVector collisionStopInMomentumThresholds(m_numJoints);
		EcRealVector assistiveModeCollisionStopThresholds(m_numJoints);
		EcRealVector frictionCompensatoryFactor(m_numJoints);
		EcRealVector lowVelocityThreshold(m_numJoints);
		EcRealVector frictionCompensatoryFactorII(m_numJoints);
		EcRealVector maxJointVelocity(m_numJoints);
		EcReal maxPowerConstraint;
		EcReal maxMomentumConstraint;
		EcReal distanceLimitInDriveBack;
		EcReal timeDurationInDriveBack;
		EcReal mass;
		EcRealVector centerofMass(3);
		bool isAssistiveMode;
		int drivebackMode;
		EcReal assistiveCheckTime;

		m_cobotAlgorithm->getSettingParameters(kinParams,
			actuatorDamp,
			torqueConstant,
			gearRatio,
			maxEfficiency,
			maxActuatorCurrents,
			rotationofMounting,
			tiltofMounting,
			upperJointLimits,
			lowerJointLimits,
			dynamicsParams,
			collisionStopThresholds,
			collisionStopInMomentumThresholds,
			assistiveModeCollisionStopThresholds,
			frictionCompensatoryFactor,
			frictionCompensatoryFactorII,
			lowVelocityThreshold,
			maxJointVelocity,
			maxPowerConstraint,
			maxMomentumConstraint,
			distanceLimitInDriveBack,
			timeDurationInDriveBack,
			mass,
			centerofMass,
			isAssistiveMode,
			drivebackMode,
			assistiveCheckTime);

		transVectorToREAL(kinParams, params->kinParams, kinParams.size());
		transVectorToREAL(actuatorDamp, params->actuatorDamp, m_numJoints);
		transVectorToREAL(torqueConstant, params->torqueConstant, m_numJoints);
		transVectorToREAL(gearRatio, params->gearRatio, m_numJoints);
		transVectorToREAL(maxEfficiency, params->maxEfficiency, m_numJoints);
		transVectorToREAL(maxActuatorCurrents, params->maxActuatorCurrents, m_numJoints);

		params->rotationofMounting = rotationofMounting;
		params->tiltofMounting = tiltofMounting;

		transVectorToREAL(upperJointLimits, params->upperJointLimits, m_numJoints);
		transVectorToREAL(lowerJointLimits, params->lowerJointLimits, m_numJoints);

		transVectorToREAL(dynamicsParams, params->dynamicsParams, m_numJoints * 13);
		transVectorToREAL(collisionStopThresholds, params->collisionStopThresholds, m_numJoints);
		transVectorToREAL(collisionStopInMomentumThresholds, params->collisionStopInMomentumThresholds, m_numJoints);
		transVectorToREAL(assistiveModeCollisionStopThresholds, params->assistiveModeCollisionStopThresholds, m_numJoints);

		transVectorToREAL(frictionCompensatoryFactor, params->frictionCompensatoryFactor, m_numJoints);
		transVectorToREAL(frictionCompensatoryFactorII, params->frictionCompensatoryFactorII, m_numJoints);
		transVectorToREAL(lowVelocityThreshold, params->lowVelocityThreshold, m_numJoints);
		transVectorToREAL(maxJointVelocity, params->maxJointVelocity, m_numJoints);
		transVectorToREAL(centerofMass, params->centerofMass, 3);

		params->maxPowerConstraint = maxPowerConstraint;
		params->maxMomentumConstraint = maxMomentumConstraint;
		params->distanceLimitInDriveBack = distanceLimitInDriveBack;
		params->timeDurationInDriveBack = timeDurationInDriveBack;
		params->mass = mass;
		params->isAssistiveMode = (RTS_IEC_BOOL)isAssistiveMode;
		params->drivebackMode = drivebackMode;
		params->assistiveCheckTime = assistiveCheckTime;
		return 1;

	}

};
#endif
