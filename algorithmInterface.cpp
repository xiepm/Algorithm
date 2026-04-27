#include "src/HMCollaborativeRobotAlgorithm.h"
#include "CmpStd.h"
#include "ElfinCollaborativeLibItf.h"

//#include "algorithmInterface.h"

#ifdef __cplusplus
extern "C" {

#define   JOINTNUM			6

	std::shared_ptr<CHansCollaborativeAlgorithm> m_cobotAlgorithm;
	int m_robotType = 0;

	bool b_enableDualAugentedAssistiveMode = false;
	//EcRealVector	temp(JOINTNUM);
	//m_cobotAlgorithm = CHansCollaborativeAlgorithm::create(temp, 0.001);

	EcRealVector m_jointSidePosition(6), m_motorSidePosition(6);


	void transREALDegToVectorRad(RTS_IEC_LREAL* realValue, EcRealVector& realVector)
	{
		realVector.assign(JOINTNUM, 0.0);
		for (int i = 0; i < JOINTNUM; i++)
		{
			realVector[i] = realValue[i] * KDL::deg2rad;
		}
	}

	void transVectorRadToREALDeg(EcRealVector& realVector, RTS_IEC_LREAL* realValue,double maxLimit = 10000)
	{
		for (int i = 0; i < JOINTNUM; i++)
		{
			realValue[i] = realVector[i] * KDL::rad2deg;
			
			if (fabs(realValue[i]) > maxLimit)
				realValue[i] = sign(realValue[i]) * maxLimit;
		}
	}

	void transREALToVector(RTS_IEC_LREAL* realValue, EcRealVector& realVector)
	{
		realVector.assign(JOINTNUM, 0.0);
		for (int i = 0; i < JOINTNUM; i++)
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
		if (m_robotType == 7 || m_robotType == 8)
			for (int i = 6; i < 10; i++)
				realValue[i] = 0;
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
		U32Vector.assign(JOINTNUM, 0);
		for (int i = 0; i < JOINTNUM; i++)
		{
			U32Vector[i] = UI32Value[i];
		}
	}

	void transBoolVectorToRTSBool(EcBooleanVector boolVector, RTS_IEC_BOOL* boolValue)
	{
		for (int i = 0; i < JOINTNUM; i++)
		{
			boolValue[i] = boolVector[i];
		}
	}

	void transBoolVectorToRTSBoolPointer(EcBooleanVector boolVector, RTS_IEC_BOOL* boolValue)
	{
		for (int i = 0; i < JOINTNUM; i++)
		{
			boolValue[i] = boolVector[i];
		}
	}

	void transVectorToREALPointer(EcRealVector vector, RTS_IEC_LREAL* realValue)
	{
		for (int i = 0; i < JOINTNUM; i++)
		{
			realValue[i] = vector[i];
		}
		if (m_robotType == 7 || m_robotType == 8)
			for (int i = 6; i < 10; i++)
				realValue[i] = 0;
	}


	void initializeCollaborativeAlgorithm(RTS_IEC_LREAL* jointPosition,
		RTS_IEC_LREAL updatePeriod,
		RTS_IEC_INT robotType,				/* VAR_INPUT */	/* 0:elfin; 1:UR; */
		RTS_IEC_LREAL* kinParams		/* VAR_INPUT */	/* elfin[d1,d4,d6,a2]; UR[d1,d4,d5,d6,a2,a3]; */)
	{
		EcRealVector jointPos;
		transREALDegToVectorRad(jointPosition, jointPos);
		m_cobotAlgorithm = CHansCollaborativeAlgorithm::create(jointPos, updatePeriod, robotType);
<<<<<<< HEAD

		EcRealVector KineParams = { kinParams[0] * 0.001, kinParams[1] * 0.001, kinParams[2] * 0.001, kinParams[3] * 0.001, kinParams[4] * 0.001, kinParams[5] * 0.001, kinParams[6] * 0.001, kinParams[7] * 0.001 };
		std::cout << "(cobotKin.b) " << robotType << "  ,  " << KineParams[0] << "," << KineParams[1] << "," << KineParams[2] << "," << KineParams[3] << "," << KineParams[4] << "," << KineParams[5] << "," << KineParams[6] << "," << KineParams[7] << std::endl;
=======
		m_robotType = robotType;
		EcRealVector KineParams = { kinParams[0] * 0.001, kinParams[1] * 0.001, kinParams[2] * 0.001, kinParams[3] * 0.001, kinParams[4] * 0.001, kinParams[5] * 0.001, kinParams[6] * 0.001, kinParams[7] * 0.001,0,0 };
		//std::cout << "(cobotKin.11F.NOF.) " << robotType << "  ,  " << KineParams[0] << "," << KineParams[1] << "," << KineParams[2] << "," << KineParams[3] << "," << KineParams[4] << "," << KineParams[5] << "," << KineParams[6] << "," << KineParams[7] << std::endl;
		//std::cout << "(cobotKin.13NOF) " << robotType << "  ,  " << KineParams[0] << "," << KineParams[1] << "," << KineParams[2] << "," << KineParams[3] << "," << KineParams[4] << "," << KineParams[5] << "," << KineParams[6] << "," << KineParams[7] << std::endl;
		std::cout << "(cobotKinDual acc.+..+) "<<CHansCollaborativeAlgorithm::getVersion()<<"," << robotType << "," << updatePeriod << "  ,  " << KineParams[0] << "," << KineParams[1] << "," << KineParams[2] << "," << KineParams[3] << "," << KineParams[4] << "," << KineParams[5] << "," << KineParams[6] << "," << KineParams[7] << std::endl;
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117
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

		//std::cout << "input:" << commandJointPositions[5] << "," << jointPositions[5] << "," << motorCurrents[5] << std::endl;
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
		EcBooleanVector collisionStatus(6);
		ENCollisionType type;
		EcBoolean ret = m_cobotAlgorithm->checkForCollision(collisionStatus, type);
		transBoolVectorToRTSBoolPointer(collisionStatus, jointCollisionStatus);
		if (m_robotType == 7 || m_robotType == 8)
		{
			for (int i = 6; i < 10; i++)
				jointCollisionStatus[i] = false;
			//std::cout << "collision status:" << bool(jointCollisionStatus[6]) << "," << bool(jointCollisionStatus[7]) << "," << bool(jointCollisionStatus[8]) << "," << bool(jointCollisionStatus[9]) << std::endl;
			/*
			for (int i = 0; i < 10; i++)
			{
				if (bool(jointCollisionStatus[i]))
				{
					std::cout << "collision status:" << i << std::endl;
				}
			}
			*/

		}
		/*
		else
		{
			for (int i = 0; i < 6; i++)
			{
				if (bool(jointCollisionStatus[i]))
				{
					std::cout << "collision status:" << i << std::endl;
				}
			}
		}
		*/
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
		EcRealVector jointPos(JOINTNUM), jointVel(JOINTNUM), jointAccel(JOINTNUM),
			motorCurr(JOINTNUM), sensedTor(JOINTNUM), estimatedTor(JOINTNUM), disturbTor(JOINTNUM);
		EcBoolean ret = m_cobotAlgorithm->getEstimatedState(
			jointPos, jointVel, jointAccel, motorCurr, sensedTor, estimatedTor, disturbTor);

		transVectorRadToREALDeg(jointPos, jointPostions);
		transVectorRadToREALDeg(jointVel, jointVelocities);
		transVectorRadToREALDeg(jointAccel, jointAccelerations);

		/**/
		if (b_enableDualAugentedAssistiveMode)
		{
			transVectorRadToREALDeg(m_motorSidePosition, jointVelocities);
			transVectorRadToREALDeg(m_jointSidePosition, jointAccelerations);
		}
		
		//transVectorRadToREALDeg(m_motorSidePosition, jointVelocities);
		//transVectorRadToREALDeg(m_jointSidePosition, jointAccelerations);

		transVectorToREAL(motorCurr, motorCurrents, JOINTNUM);
		transVectorToREAL(sensedTor, sensedTorques, JOINTNUM);
		transVectorToREAL(estimatedTor, estimatedTorques, JOINTNUM);
		transVectorToREALPointer(disturbTor, disturbanceTorques);

		//std::cout << "output:" << jointPostions[5] << "," << jointVelocities[5] << "," << jointAccelerations[5] << "," << sensedTorques[5] << "," << estimatedTorques[5] << std::endl;
		return ret;
	}

	void setAssistDualEncoderFlag(
		RTS_IEC_BOOL flag
	)
	{
		b_enableDualAugentedAssistiveMode = flag;
		m_cobotAlgorithm->setAssistDualEncoderFlag(flag);
	}

	void setDualDiffEncoderThd(RTS_IEC_LREAL thd[6]) {

		std::cout << "set dual thd:" << thd[0] << "," << thd[1] << "," << thd[2] << "," << thd[3] << "," << thd[4] << "," << thd[5] << std::endl;
		EcRealVector vThd = { thd[0]*0.001, thd[1] * 0.001, thd[2] * 0.001, thd[3] * 0.001, thd[4] * 0.001, thd[5] * 0.001 };
		m_cobotAlgorithm->setDualDiffEncoderThd(vThd);
	}

	void updateDualEncoderJointPosition(
		RTS_IEC_LREAL motorSideJointPosition[6],	/* VAR_INPUT */	/* actual motor side  joint position(deg) */
		RTS_IEC_LREAL jointSideJointPosition[6]	/* VAR_INPUT */	/* actual joint side joint position(deg) */
	)
	{
		EcRealVector motorSide, jointSide,vMotorSide,vJointSide;
		transREALDegToVectorRad(motorSideJointPosition, motorSide);
		transREALDegToVectorRad(jointSideJointPosition, jointSide);
		m_motorSidePosition = motorSide;
		m_jointSidePosition = jointSide;

		transREALToVector(motorSideJointPosition, vMotorSide);
		transREALToVector(jointSideJointPosition, vJointSide);
		m_cobotAlgorithm->updateDualEncoderPosition(vJointSide, vMotorSide);

		/*
		std::cout << "motor,joint:" << motorSideJointPosition[0] << "," << motorSideJointPosition[1] << "," << motorSideJointPosition[2] << "," << motorSideJointPosition[3] << "," << motorSideJointPosition[4] << "," << motorSideJointPosition[5] << ",   " <<
			jointSideJointPosition[0] << "," << jointSideJointPosition[1] << "," << jointSideJointPosition[2] << "," << jointSideJointPosition[3] << "," << jointSideJointPosition[4] << "," << jointSideJointPosition[5] << std::endl;
		*/
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



	void setDynamicsLinearParameters
	(
		RTS_IEC_LREAL* params
	)
	{
		EcRealVector dynParams(78);
		for (int i = 0; i < 78; ++i)
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

	void setRobotMountingAngles
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

	RTS_IEC_BOOL setPayloadMassProperties
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

	void setRobotDHParameters
	(
		RTS_IEC_LREAL* kinematcisParam
	)
	{
		EcRealVector KineParams = { kinematcisParam[0] * 0.001, kinematcisParam[1] * 0.001, kinematcisParam[2] * 0.001, kinematcisParam[3] * 0.001 };
		std::cout << "(cobotKin2)" << KineParams[0] << "," << KineParams[1] << "," << KineParams[2] << "," << KineParams[3] << std::endl;
		m_cobotAlgorithm->setRobotDHParameters(KineParams);
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
		std::cout << "assistiveMode thd:" << assistiveModeCollisionStopThresholds[0] << "," << assistiveModeCollisionStopThresholds[1] << "," << assistiveModeCollisionStopThresholds[2] << "," << assistiveModeCollisionStopThresholds[3] << "," << assistiveModeCollisionStopThresholds[4] << "," << assistiveModeCollisionStopThresholds[5] << std::endl;
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
		std::cout << "Upper joint's Limit:" << upperJointLimits[0] << "," << upperJointLimits[1] << "," << upperJointLimits[2] << "," << upperJointLimits[3] << "," << upperJointLimits[4] << "," << upperJointLimits[5] << std::endl;
		std::cout << "Lower joint's Limit:" << lowerJointLimits[0] << "," << lowerJointLimits[1] << "," << lowerJointLimits[2] << "," << lowerJointLimits[3] << "," << lowerJointLimits[4] << "," << lowerJointLimits[5] << std::endl;

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
		std::cout << "CompensateFrictionCoeff:" << frictionCompensatoryFactor[0] << ", " << frictionCompensatoryFactor[1] << ", " << frictionCompensatoryFactor[2] << ", " << frictionCompensatoryFactor[3] << ", " << frictionCompensatoryFactor[4] << ", " << frictionCompensatoryFactor[5] << std::endl;
		m_cobotAlgorithm->setFrictionCompensatoryFactor(tempValue);
	}

	void setFrictionCompensatoryFactorII
	(
		RTS_IEC_LREAL* factor
	)
	{
		std::cout << "DynFrictionCoeff:" << factor[0] << ", " << factor[1] << ", " << factor[2] << ", " << factor[3] << ", " << factor[4] << ", " << factor[5] << std::endl;
		EcRealVector temp(6);
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
		EcRealVector tempValue(6);
		EcBooleanVector tempValue2(6);

		RTS_IEC_INT ret = m_cobotAlgorithm->getAssistiveModeMotorCurrentCommands(tempValue, tempValue2);
		//std::cout<<"return assistiveMode = "<<ret<<std::endl;
		transVectorToREALPointer(tempValue, motorCurrentCommands);
		transBoolVectorToRTSBoolPointer(tempValue2, jointCollisionStatus);

		if (m_robotType == 7 || m_robotType == 8)
		{
			motorCurrentCommands[6] = 0;
			for (int i = 6; i < 10; i++)
				jointCollisionStatus[i] = false;
		}
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
		EcRealVector tempValue(6), position(6), vel(6), acc(6);
		transREALDegToVectorRad(jointPosition, position);
		transREALDegToVectorRad(jointVelocity, vel);
		transREALDegToVectorRad(jointAcceleration, acc);
		m_cobotAlgorithm->getComputeTorqueCurrentCommands(compensateRatio, position, vel, acc, tempValue);
		transVectorToREALPointer(tempValue, motorCurrentCommands);
		if (m_robotType == 7 || m_robotType == 8)
			motorCurrentCommands[6] = 0;
	}

	void getGravityTorqueCurrentCommands
	(
		const RTS_IEC_LREAL compensateRatio,
		RTS_IEC_LREAL* jointPosition,
		RTS_IEC_LREAL* motorCurrentCommands
	)
	{
		EcRealVector joint(6), tempValue(6);
		transREALDegToVectorRad(jointPosition, joint);
		m_cobotAlgorithm->getGravityTorqueCurrentCommands(compensateRatio, joint, tempValue);
		transVectorToREALPointer(tempValue, motorCurrentCommands);
	}

	/**/
	void getGravityTorqueForFlexibleCompensate
	(
		RTS_IEC_LREAL* jointPosition,
		RTS_IEC_LREAL* gravTorque
	)
	{
		EcRealVector joint(6), tempValue(6);
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
		EcRealVector joint(6), tempValue(6);
		transREALDegToVectorRad(jointPosition, joint);
		m_cobotAlgorithm->getJointInertia(joint, tempValue);
		transVectorToREALPointer(tempValue, inertia);
	}




	void setActutorDampConstants
	(
		RTS_IEC_LREAL* damp
	)
	{
		EcRealVector temp(6);
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
		EcRealVector temp(6);
		transREALToVector(thresholds, temp);
		m_cobotAlgorithm->setCollisionStopInMomentumThresholds(temp);
	}

	
	void setMaxPowerAndMomentumConstraints(
		const RTS_IEC_LREAL maxPower,
		const RTS_IEC_LREAL maxMomentum,
		RTS_IEC_LREAL* jointPowers)
	{
		std::cout << "Constraints, maxPower: " << maxPower << ", maxMomentum: " << maxMomentum << std::endl;
		EcRealVector temp(6);
		transREALToVector(jointPowers, temp);
		m_cobotAlgorithm->setMaxPowerAndMomentumConstraints(maxPower, maxMomentum, temp);
	}

	RTS_IEC_BOOL calculatePowerAndMomentumConstraints(
		RTS_IEC_LREAL userDefineOverride,
		RTS_IEC_LREAL* jointVoltages,
		RTS_IEC_BOOL* constraintActive,	 // need to re-plan the motion at rising edge;
		RTS_IEC_LREAL* jointVelFactorConstraint,					// constraint factor for joint velocities;
		RTS_IEC_LREAL* jointAccFactorConstraint,					// constraint factor for joint accelerations;
		RTS_IEC_LREAL* electircPower,	   // P = U * I;
		RTS_IEC_LREAL* physicsPower,		   // P = Torque * omega;
		RTS_IEC_LREAL* momentum			   // momemtum = mass * omega;
	)
	{
		EcRealVector volage(6);
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
		*constraintActive = temp;
		return ret;
	}

	RTS_IEC_BOOL setStartTimeForCheckAssistiveMode
	(
		RTS_IEC_LREAL assistiveCheckTime	/* VAR_INPUT */	/* the start time for check robot status in assistive mode, second ,s; */
	)
	{
		m_cobotAlgorithm->setStartTimeForCheckAssisitiveMode(assistiveCheckTime);
	}

	void getGravityTorqueForFlexibleCompensate
	(
		RTS_IEC_LREAL* jointPosition,
		RTS_IEC_LREAL* gravTorque
	)
	{
		EcRealVector joint(6), tempValue(6);
		transREALDegToVectorRad(jointPosition, joint);
		m_cobotAlgorithm->getFlexibleCompensateGravityTorque(joint, tempValue);
		transVectorToREALPointer(tempValue, gravTorque);
	}

	RTS_IEC_BOOL setMotionLimitInDriveback
	(
		RTS_IEC_LREAL distanceLimit,		/* VAR_INPUT */	/* allowing joint motion(degree,  °) in drive back mode; */
		RTS_IEC_LREAL timeDuration			/* VAR_INPUT */	/* allowing time duration(second, s) in drive back mode; */
	)
	{
		distanceLimit *= KDL::deg2rad;
		m_cobotAlgorithm->setMotionLimitInDriveBack(distanceLimit, timeDuration);
	}

	RTS_IEC_BOOL getSettingParams
	(
		cobotParams* params
	)
	{
		EcRealVector kinParams(4);
		EcRealVector actuatorDamp(numofJoints);
		EcRealVector torqueConstant(numofJoints);
		EcRealVector gearRatio(numofJoints);
		EcRealVector maxEfficiency(numofJoints);
		EcRealVector maxActuatorCurrents(numofJoints);
		EcReal rotationofMounting;		/* rotate robot base mounting angles(deg) */
		EcReal tiltofMounting;		/* tilt robot base mounting angles(deg) */
		EcRealVector upperJointLimits(numofJoints);
		EcRealVector lowerJointLimits(numofJoints);
		EcRealVector dynamicsParams(numofJoints * 13);
		EcRealVector collisionStopThresholds(numofJoints);			/* joint collisiton threshold, */
		EcRealVector collisionStopInMomentumThresholds(numofJoints);			/* joint collisiton threshold, */
		EcRealVector assistiveModeCollisionStopThresholds(numofJoints);
		EcRealVector frictionCompensatoryFactor(numofJoints);			/* default setting: J1="0.5" J2="0.5" J3="0.4" J4="0.6" J5="0.7" J6="0.7" */
		EcRealVector frictionCompensatoryFactorII(numofJoints);			/* default seting [1.0], */
		EcRealVector lowVelocityThreshold(numofJoints);			/* decide whether the joints are in low velocity status, */
		EcRealVector maxJointVelocity(numofJoints);
		EcReal maxPowerConstraint;		/* power */
		EcReal maxMomentumConstraint;		/* momentum */
		EcReal distanceLimitInDriveBack;		/* allowing joint motion(degree,  °) in drive back mode, */
		EcReal timeDurationInDriveBack;		/* allowing time duration(second, s) in drive back mode, */
		EcReal mass;
		EcRealVector centerofMass(3);		/* (millimeter) */
		bool isAssistiveMode;
		int drivebackMode;
		EcReal assistiveCheckTime;		/* the start time for check robot status in assistive mode, second ,s; */

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
		transVectorToREAL(actuatorDamp, params->actuatorDamp, 6);
		transVectorToREAL(torqueConstant, params->torqueConstant, 6);
		transVectorToREAL(gearRatio, params->gearRatio, 6);
		transVectorToREAL(maxEfficiency, params->maxEfficiency, 6);
		transVectorToREAL(maxActuatorCurrents, params->maxActuatorCurrents, 6);

		params->rotationofMounting = rotationofMounting;
		params->tiltofMounting = tiltofMounting;

		transVectorToREAL(upperJointLimits, params->upperJointLimits, 6);
		transVectorToREAL(lowerJointLimits, params->lowerJointLimits, 6);

		transVectorToREAL(dynamicsParams, params->dynamicsParams, 78);
		transVectorToREAL(collisionStopThresholds, params->collisionStopThresholds, 6);
		transVectorToREAL(collisionStopInMomentumThresholds, params->collisionStopInMomentumThresholds, 6);
		transVectorToREAL(assistiveModeCollisionStopThresholds, params->assistiveModeCollisionStopThresholds, 6);

		transVectorToREAL(frictionCompensatoryFactor, params->frictionCompensatoryFactor, 6);
		transVectorToREAL(frictionCompensatoryFactorII, params->frictionCompensatoryFactorII, 6);
		transVectorToREAL(lowVelocityThreshold, params->lowVelocityThreshold, 6);
		transVectorToREAL(maxJointVelocity, params->maxJointVelocity, 6);
		transVectorToREAL(centerofMass, params->centerofMass, 3);

		params->maxPowerConstraint = maxPowerConstraint;
		params->maxMomentumConstraint = maxMomentumConstraint;
		params->distanceLimitInDriveBack = distanceLimitInDriveBack;
		params->timeDurationInDriveBack = timeDurationInDriveBack;
		params->mass = mass;
		params->isAssistiveMode = isAssistiveMode;
		params->drivebackMode = drivebackMode;
		params->assistiveCheckTime = assistiveCheckTime;

	}



};
#endif