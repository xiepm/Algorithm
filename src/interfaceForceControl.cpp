#include "interfaceForceControl.h"


	EcBoolean interfaceForceControl::initialize(const EcRealVector& commandEEPosition, const EcReal timeCycle)
	{
		if (commandEEPosition.size() != 6)
			return false;
		m_commandToolPosition.p = { commandEEPosition[0], commandEEPosition[1], commandEEPosition[2] };
		m_commandToolPosition.M = KDL::Rotation::RPY(commandEEPosition[3], commandEEPosition[4], commandEEPosition[5]);

		m_preCommandToolPosition = m_commandToolPosition;
		m_preCommandToolPosition = m_commandToolPosition;
		m_preUpdateToolPosition = m_commandToolPosition;
		b_isToolCoordinate = true;
		m_timeCycle = timeCycle;
		m_commandVel.assign(6, 0.0);
		return true;
	}


	EcBoolean interfaceForceControl::setControlFreedom(const EcU32Vector& selectedDOF)
	{
		if (selectedDOF.size() != 8)
			return false;
		m_selectedDOF = selectedDOF;
		return true;
	}


	EcBoolean interfaceForceControl::setCoordinateSystem(const EcU32 frameType, const EcRealVector& userCoord, EcRealVector& userPoseMatrix)
	{
		if (userCoord.size() != 6 || userPoseMatrix.size() != 9)
			return false;
		m_FrameType = frameType;
		m_userRotation.M = KDL::Rotation::RPY(userCoord[3], userCoord[4], userCoord[5]);
		for (EcSizeT i = 0; i < 9; i++)
			userPoseMatrix[i] = m_userRotation.M.data[i];
		return true;
	}

	EcBoolean interfaceForceControl::updateRobotState(const EcRealVector& actualEEPosition, const EcRealVector& commandEEPosition, EcRealVector& actualEEPoseMatrix)
	{
		if (actualEEPosition.size() != 6 || commandEEPosition.size() != 6 || actualEEPoseMatrix.size() != 9)
			return false;

		m_commandToolPosition.p = { commandEEPosition[0], commandEEPosition[1], commandEEPosition[2] };
		m_commandToolPosition.M = KDL::Rotation::RPY(commandEEPosition[3], commandEEPosition[4], commandEEPosition[5]);

		KDL::Rotation R = KDL::Rotation::RPY(actualEEPosition[3], actualEEPosition[4], actualEEPosition[5]);
		for (EcSizeT i = 0; i < 9; i++)
			actualEEPoseMatrix[i] = R.data[i];
		return true;
	}

	EcBoolean interfaceForceControl::calculateCommandForcePosition(const EcRealVector& forceControlVel, EcRealVector& forceCommandEEPosition)
	{
		//1. update force controller velocity command;
		KDL::Vector velPID = { forceControlVel[0], forceControlVel[1], forceControlVel[2] };					// force control 更新的位置命令
		KDL::Vector omegaPID = { forceControlVel[3], forceControlVel[4], forceControlVel[5] };


		// todo: 在力控方向上不进行位置更新
		//2. convert to Robot Position command;
		KDL::Frame rectifyFrame;
		KDL::Vector rectifyAngle, actualVel, actualOmega;
		if (m_FrameType == 1){
			// tool coordinate
			KDL::Frame deltaFrame = m_preCommandToolPosition.Inverse() * m_commandToolPosition;	// update data	//SDK更新的位置命令
			actualVel = deltaFrame.p / m_timeCycle;
			actualOmega = deltaFrame.M.GetRot() / m_timeCycle;

			actualVel = actualVel + velPID;
			rectifyFrame.p = actualVel * m_timeCycle;
			actualOmega = actualOmega + omegaPID;
			rectifyAngle = actualOmega * m_timeCycle;
			EcReal angle = rectifyAngle.Normalize();
			rectifyFrame.M = KDL::Rotation::Rot2(rectifyAngle, angle);
			m_updateToolPosition = m_preUpdateToolPosition * rectifyFrame;
		}
		else{
			// user coordinate
			KDL::Frame deltaFrame = m_commandToolPosition*m_preCommandToolPosition.Inverse();	// update data
			actualVel = m_userRotation.Inverse() * deltaFrame.p / m_timeCycle;
			actualOmega = m_userRotation.Inverse() * deltaFrame.M.GetRot() / m_timeCycle;

			actualVel = actualVel + velPID;
			rectifyFrame.p = m_userRotation * actualVel * m_timeCycle;
			actualOmega = actualOmega + omegaPID;
			rectifyAngle = m_userRotation * actualOmega * m_timeCycle;
			EcReal angle = rectifyAngle.Normalize();
			rectifyFrame.M = KDL::Rotation::Rot2(rectifyAngle, angle);
			m_updateToolPosition = rectifyFrame * m_preUpdateToolPosition;
		}

		// todo:TCP工具和逆解的处理还需要考虑；
		m_updateEEPosition = m_updateToolPosition;
		forceCommandEEPosition[0] = m_updateEEPosition.p[0];
		forceCommandEEPosition[1] = m_updateEEPosition.p[1];
		forceCommandEEPosition[2] = m_updateEEPosition.p[2];
		m_updateEEPosition.M.GetRPY(forceCommandEEPosition[3], forceCommandEEPosition[4], forceCommandEEPosition[5]);

		// 3. refresh command; 
		m_preCommandToolPosition = m_commandToolPosition;
		m_preUpdateToolPosition = m_updateToolPosition;			// integral force control command;

		// 4. command velocity
		for (EcSizeT i = 0; i < 3; i++){
			m_commandVel[i] = actualVel[i];
			m_commandVel[i + 3] = actualOmega[i];
		}
		return true;
	}


	void interfaceForceControl::getCommandVelocity(EcRealVector& commandVel)
	{
		commandVel = m_commandVel;
	}


