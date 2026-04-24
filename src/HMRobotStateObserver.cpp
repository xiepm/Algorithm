#include "HMRobotStateObserver.h"
/*
#include <manipulator/ecDh.h>
#include <foundCore/ecMacros.h>
#include <foundCore/ecMath.h>
#include <manipulator/ecIndManipulator.h>
#include <manipulator/ecStatedSystem.h>
#include <manipulator/ecStatedSystemStruct.h>
*/

/////////////////////////////////////////////////////////////////////////
// Functions:    Big four and other header functions
// Description:  Common functions
/////////////////////////////////////////////////////////////////////////

CHansRobotStateObserver::CHansRobotStateObserver
(
const EcReal updateTimePeriod,
EcU32  sampleSpacingFactor // = 8
) :
m_IsInitialized(EcFalse),
m_SamplePeriod(updateTimePeriod),
m_SampleSpacingFactor(sampleSpacingFactor > 0 ? sampleSpacingFactor : 1),
m_StencilSpacing(updateTimePeriod * m_SampleSpacingFactor),
m_Index0(0),
m_Index1(1 * m_SampleSpacingFactor),
m_Index2(2 * m_SampleSpacingFactor),
m_StencileSize(3),    // currently using f(x), f(x-1h), and f(x-2h) for acceleration and velocity given a spacing of h
m_PositionLowPassFilters(),
m_VelocityLowPassFilters(),
m_AccelerationLowPassFilters(),
m_JerkLowPassFilters(),
m_MotorCurrentLowPassFilters(),
m_DisturbanceTorqueLowPassFilters(),
m_SensedEeForceLowPassFilters(),
m_AdmittancePoseLowPassFilters()
{
}

// destructor
CHansRobotStateObserver::~CHansRobotStateObserver
(
)
{
}

// copy constructor
CHansRobotStateObserver::CHansRobotStateObserver
(
const CHansRobotStateObserver& orig
) :
m_IsInitialized(orig.m_IsInitialized),
m_SamplePeriod(orig.m_SamplePeriod),
m_StencilSpacing(orig.m_StencilSpacing),
m_SampleSpacingFactor(orig.m_SampleSpacingFactor),
m_Index0(orig.m_Index0),
m_Index1(orig.m_Index1),
m_Index2(orig.m_Index2),
m_StencileSize(orig.m_StencileSize),
m_PositionLowPassFilters(orig.m_PositionLowPassFilters),
m_VelocityLowPassFilters(orig.m_VelocityLowPassFilters),
m_AccelerationLowPassFilters(orig.m_AccelerationLowPassFilters),
m_JerkLowPassFilters(orig.m_JerkLowPassFilters),
m_MotorCurrentLowPassFilters(orig.m_MotorCurrentLowPassFilters),
m_DisturbanceTorqueLowPassFilters(orig.m_DisturbanceTorqueLowPassFilters),
m_SensedEeForceLowPassFilters(orig.m_SensedEeForceLowPassFilters),
m_AdmittancePoseLowPassFilters(orig.m_AdmittancePoseLowPassFilters)
{
}

// assignment operator
CHansRobotStateObserver& CHansRobotStateObserver::operator=
(
const CHansRobotStateObserver& orig
)
{
	// self assignment.
	if (this == &orig)
	{
		return *this;
	}

	m_IsInitialized = orig.m_IsInitialized;
	m_SamplePeriod = orig.m_SamplePeriod;
	m_StencilSpacing = orig.m_StencilSpacing;
	m_SampleSpacingFactor = orig.m_SampleSpacingFactor;
	m_Index0 = orig.m_Index0;
	m_Index1 = orig.m_Index1;
	m_Index2 = orig.m_Index2;
	m_StencileSize = orig.m_StencileSize;
	m_PositionLowPassFilters = orig.m_PositionLowPassFilters;
	m_VelocityLowPassFilters = orig.m_VelocityLowPassFilters;
	m_AccelerationLowPassFilters = orig.m_AccelerationLowPassFilters;
	m_JerkLowPassFilters = orig.m_JerkLowPassFilters;
	m_MotorCurrentLowPassFilters = orig.m_MotorCurrentLowPassFilters;
	m_DisturbanceTorqueLowPassFilters = orig.m_DisturbanceTorqueLowPassFilters;
	m_SensedEeForceLowPassFilters = orig.m_SensedEeForceLowPassFilters;
	m_AdmittancePoseLowPassFilters = orig.m_AdmittancePoseLowPassFilters;
	return *this;
}

/////////////////////////////////////////////////////////////////////////
//End of header functions
/////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
void CHansRobotStateObserver::setSamplingPeriod
(
const EcReal samplePeriod
)
{
	// The sampling time period. This is the period at which the estimate method is called
	m_SamplePeriod = samplePeriod;
	m_IsInitialized = EcFalse;
}

//------------------------------------------------------------------------------
EcReal CHansRobotStateObserver::samplingPeriod
(
)
{
	// The sampling time period. This is the period at which the estimate method is called
	return m_SamplePeriod;
}


//------------------------------------------------------------------------------
void CHansRobotStateObserver::setSampleSpacingFactor
(
const EcU32 sampleSpacingFactor
)
{
	// The sampling time period. This is the period at which the estimate method is called
	m_SampleSpacingFactor = sampleSpacingFactor > 0 ? sampleSpacingFactor : 1;
	m_IsInitialized = EcFalse;
}

//------------------------------------------------------------------------------
EcU32 CHansRobotStateObserver::sampleSpacingFactor
(
)
{
	// The sampling time period. This is the period at which the estimate method is called
	return m_SampleSpacingFactor;
}

//------------------------------------------------------------------------------
// initialize observer
EcBoolean CHansRobotStateObserver::initialize
(
const EcRealVector& jointPositions
)
{
	// The stencil spacing is the constant time difference (h) over which the finite differences are computed.
	// It defines the spacing in time between the values used in the finite difference approximation ie f(x), f(x-h), f(x-2h)

	m_StencilSpacing = m_SamplePeriod * m_SampleSpacingFactor;

	//const EcReal defaultFilterTimeConstant = 0.016; // 16 milliseconds
	const EcReal defaultFilterTimeConstant = 0.012; // 16 milliseconds

	// Initialize low-pass filters
	m_PositionLowPassFilters.resize(jointPositions.size(), hansLowPassFilter(defaultFilterTimeConstant, m_SamplePeriod));
	m_VelocityLowPassFilters.resize(jointPositions.size(), hansLowPassFilter(defaultFilterTimeConstant, m_SamplePeriod));
	m_AccelerationLowPassFilters.resize(jointPositions.size(), hansLowPassFilter(defaultFilterTimeConstant, m_SamplePeriod));
	m_JerkLowPassFilters.resize(jointPositions.size(), hansLowPassFilter(defaultFilterTimeConstant, m_SamplePeriod));
	m_MotorCurrentLowPassFilters.resize(jointPositions.size(), hansLowPassFilter(defaultFilterTimeConstant, m_SamplePeriod));
	m_DisturbanceTorqueLowPassFilters.resize(jointPositions.size(), hansLowPassFilter(defaultFilterTimeConstant, m_SamplePeriod));
	m_SensedEeForceLowPassFilters.resize(jointPositions.size(), hansLowPassFilter(defaultFilterTimeConstant, m_SamplePeriod));
	m_AdmittancePoseLowPassFilters.resize(jointPositions.size(), hansLowPassFilter(defaultFilterTimeConstant, m_SamplePeriod));
	m_AdmittanceVelocityLowPassFilters.resize(jointPositions.size(), hansLowPassFilter(defaultFilterTimeConstant, m_SamplePeriod));
	// reset the low-pass filters
	for (EcSizeT ii = 0; ii < jointPositions.size(); ++ii)
	{
		m_PositionLowPassFilters[ii].setOldData(jointPositions[ii]);
		m_VelocityLowPassFilters[ii].setOldData(0);
		m_AccelerationLowPassFilters[ii].setOldData(0);
		m_JerkLowPassFilters[ii].setOldData(0);
		m_MotorCurrentLowPassFilters[ii].setOldData(0);
		m_DisturbanceTorqueLowPassFilters[ii].setOldData(0);
		m_SensedEeForceLowPassFilters[ii].setOldData(0);
		m_AdmittancePoseLowPassFilters[ii].setOldData(0);
		m_AdmittanceVelocityLowPassFilters[ii].setOldData(0);
	}

	// Initialize values
	const EcSizeT bufferSize = m_SampleSpacingFactor * m_StencileSize;
	EcRealVector zeros;
	zeros.assign(6, 0.0);
	m_OldJointPositions.assign(bufferSize, jointPositions);
	m_OldJointVelocity.assign(bufferSize, zeros);

	m_OldAdmittanceDeviatePose.assign(bufferSize, zeros);

	m_OldDisturbanceTorques.assign(10, zeros);
	m_OldJointCurrents.assign(currentMedianFilterCount, zeros);

	m_Index0 = 0;
	m_Index1 = 1 * m_SampleSpacingFactor;
	m_Index2 = 2 * m_SampleSpacingFactor;

	m_IsInitialized = EcTrue;

	return m_IsInitialized;
}

//------------------------------------------------------------------------------
// set filter time constants
EcBoolean CHansRobotStateObserver::setFilterTimeConstants
(
const EcRealVector& jointPositions,
const EcRealVector& positionTimeConstants,
const EcRealVector& velocityTimeConstants,
const EcRealVector& accelerationTimeConstants,
const EcRealVector& motorCurrentTimeConstants,
const EcRealVector& disturbanceTorqueTimeConstants
)
{
	const EcSizeT numJoints = jointPositions.size();

	// check the input data sizes
	if (
		!m_IsInitialized ||
		positionTimeConstants.size() != numJoints ||
		velocityTimeConstants.size() != numJoints ||
		accelerationTimeConstants.size() != numJoints ||
		motorCurrentTimeConstants.size() != numJoints ||
		disturbanceTorqueTimeConstants.size() != numJoints
		)
	{
		return EcFalse;
	}

	// check the filter sizes
	if (
		m_PositionLowPassFilters.size() != numJoints ||
		m_VelocityLowPassFilters.size() != numJoints ||
		m_AccelerationLowPassFilters.size() != numJoints ||
		m_JerkLowPassFilters.size() != numJoints ||
		m_MotorCurrentLowPassFilters.size() != numJoints ||
		m_DisturbanceTorqueLowPassFilters.size() != numJoints
		)
	{
		return EcFalse;
	}

	// Filters
	for (EcSizeT ii = 0; ii < m_PositionLowPassFilters.size(); ++ii)
	{
		m_PositionLowPassFilters[ii].setOldData(jointPositions[ii]);
		m_PositionLowPassFilters[ii].setTimeConstant(positionTimeConstants[ii]);

		m_VelocityLowPassFilters[ii].setTimeConstant(velocityTimeConstants[ii]);
		m_AccelerationLowPassFilters[ii].setTimeConstant(accelerationTimeConstants[ii]);
		m_JerkLowPassFilters[ii].setTimeConstant(accelerationTimeConstants[ii]);
		m_MotorCurrentLowPassFilters[ii].setTimeConstant(motorCurrentTimeConstants[ii]);
		m_DisturbanceTorqueLowPassFilters[ii].setTimeConstant(disturbanceTorqueTimeConstants[ii]);
	}

	return EcTrue;
}

EcBoolean CHansRobotStateObserver::setUniteFilterTimeConstants
(
const EcRealVector& jointPositions,
const EcRealVector& filterTimeConstants,
const EcRealVector& disturbFilterTimeConstants
)
{
	return setFilterTimeConstants(jointPositions, filterTimeConstants, filterTimeConstants, filterTimeConstants, filterTimeConstants, 
		disturbFilterTimeConstants);
}

//------------------------------------------------------------------------------
EcBoolean CHansRobotStateObserver::estimateStates
(
const EcRealVector& jointPositions,
EcRealVector& estJointPositions,
EcRealVector& estJointVelocities,
EcRealVector& estJointAccelerations
)
{
	if (!m_IsInitialized)
	{
		return EcFalse;
	}

	EcBoolean retVal = EcTrue;

	// calculate positions
	retVal &= estimateJointPositions(jointPositions, estJointPositions);

	// calculate velocity
	retVal &= estimateVelocities(estJointVelocities);

	// calculate acceleration
	retVal &= estimateAccelerations(estJointAccelerations);

	return retVal;
}

//------------------------------------------------------------------------------
EcBoolean CHansRobotStateObserver::estimateJointPositions
(
const EcRealVector& jointPositions,
EcRealVector& estJointPositions
)
{
	if (jointPositions.size() != m_PositionLowPassFilters.size())
	{
		return EcFalse;
	}

	for (EcSizeT ii = 0; ii < jointPositions.size(); ++ii)
	{
		estJointPositions[ii] = m_PositionLowPassFilters[ii].updateOutput(jointPositions[ii]);
	}

	m_OldJointPositions.push_front(estJointPositions);

	return EcTrue;
}

//------------------------------------------------------------------------------
EcBoolean CHansRobotStateObserver::estimateVelocities
(
EcRealVector& estJointVelocities
)
{
	if (m_Index2 >= m_OldJointPositions.size())
	{
		return EcFalse;
	}

	const EcRealVector& positions2 = m_OldJointPositions[m_Index2];
	const EcRealVector& positions1 = m_OldJointPositions[m_Index1];
	const EcRealVector& positions0 = m_OldJointPositions[m_Index0];

	const EcSizeT numJoints = positions0.size();
	if (
		positions1.size() != numJoints ||
		positions2.size() != numJoints ||
		m_VelocityLowPassFilters.size() != numJoints
		)
	{
		return EcFalse;
	}

	estJointVelocities.resize(numJoints);

	// 2nd order backward finite difference for velocity
	//	f_x = ( 1*f[i-2] - 4*f[i-1] + 3*f[i+0] ) / (2*h)

	for (EcSizeT ii = 0; ii < numJoints; ++ii)
	{
		estJointVelocities[ii] = (
			1.0 * positions2[ii]
			- 4.0 * positions1[ii]
			+ 3.0 * positions0[ii]
			)
			/ (2.0 * m_StencilSpacing);

		estJointVelocities[ii] = m_VelocityLowPassFilters[ii].updateOutput(estJointVelocities[ii]);
	}
	m_OldJointVelocity.push_front(estJointVelocities);
	return EcTrue;
}

//------------------------------------------------------------------------------
EcBoolean CHansRobotStateObserver::estimateAccelerations
(
EcRealVector& estJointAccelerations
)
{
	if (m_Index2 >= m_OldJointPositions.size())
	{
		return EcFalse;
	}

	const EcRealVector& positions2 = m_OldJointPositions[m_Index2];
	const EcRealVector& positions1 = m_OldJointPositions[m_Index1];
	const EcRealVector& positions0 = m_OldJointPositions[m_Index0];

	const EcSizeT numJoints = positions0.size();
	if (
		positions1.size() != numJoints ||
		positions2.size() != numJoints ||
		m_AccelerationLowPassFilters.size() != numJoints
		)
	{
		return EcFalse;
	}

	estJointAccelerations.resize(numJoints);

	//  2nd order backward finite difference for acceleration
	//  	f_xx = ( 1*f[i-2] - 2*f[i-1] + 1*f[i+0] ) / (1*h^2)

	for (EcSizeT ii = 0; ii < numJoints; ++ii)
	{
		estJointAccelerations[ii] = (
			1.0 * positions2[ii]
			- 2.0 * positions1[ii]
			+ 1.0 * positions0[ii]
			)
			/ (m_StencilSpacing * m_StencilSpacing);

		estJointAccelerations[ii] = m_AccelerationLowPassFilters[ii].updateOutput(estJointAccelerations[ii]);
	}

	return EcTrue;
}

EcBoolean CHansRobotStateObserver::estimateJerks
(
EcRealVector& estJointJerks
)
{
	if (m_Index2 >= m_OldJointVelocity.size())
	{
		return EcFalse;
	}

	const EcRealVector& positions2 = m_OldJointVelocity[m_Index2];
	const EcRealVector& positions1 = m_OldJointVelocity[m_Index1];
	const EcRealVector& positions0 = m_OldJointVelocity[m_Index0];

	const EcSizeT numJoints = positions0.size();
	if (
		positions1.size() != numJoints ||
		positions2.size() != numJoints ||
		m_JerkLowPassFilters.size() != numJoints
		)
	{
		return EcFalse;
	}

	estJointJerks.resize(numJoints);

	//  2nd order backward finite difference for acceleration
	//  	f_xx = ( 1*f[i-2] - 2*f[i-1] + 1*f[i+0] ) / (1*h^2)

	for (EcSizeT ii = 0; ii < numJoints; ++ii)
	{
		estJointJerks[ii] = (
			1.0 * positions2[ii]
			- 2.0 * positions1[ii]
			+ 1.0 * positions0[ii]
			)
			/ (m_StencilSpacing * m_StencilSpacing);

		estJointJerks[ii] = m_JerkLowPassFilters[ii].updateOutput(estJointJerks[ii]);
	}

	return EcTrue;

}

void CHansRobotStateObserver::sortFunction
(
EcRealVectorVector& sortArray
)
{
	//std::sort(sortArray.begin(), sortArray.end());
	const EcSizeT numCircle = sortArray.size();
	EcU32 numJoints = sortArray[0].size();
	for (EcSizeT ii = 0; ii < numJoints; ii++)
	{
		EcReal temp = 0.0;
		for (int j = 0; j < numCircle - 1; j++){
			for (int i = 0; i < numCircle - 1 - j; i++){
				if (sortArray[i][ii]>sortArray[i + 1][ii]){
					temp = sortArray[i][ii];
					sortArray[i][ii] = sortArray[i + 1][ii];
					sortArray[i + 1][ii] = temp;
				}
			}
		}
	}
}

//------------------------------------------------------------------------------
EcBoolean CHansRobotStateObserver::filterMotorCurrents
(
const EcRealVector& motorCurrents,
EcRealVector& filteredMotorCurrents
)
{
	const EcSizeT numFiltered = m_MotorCurrentLowPassFilters.size();
	if (numFiltered != motorCurrents.size())
	{
		return EcFalse;
	}
	// 加入一个中值滤波法（medianFilterCount个周期中，去掉最大和最小，剩下的取平均值）
	m_OldJointCurrents.push_front(motorCurrents);
	EcRealVectorVector tempArray;
	EcU32 numCircle = m_OldJointCurrents.size();
	for (int i = 0; i < numCircle; i++)
	{
		tempArray.push_back(m_OldJointCurrents[i]);
	}
	sortFunction(tempArray);

	EcRealVector tempCurrent;
	
	tempCurrent.assign(numFiltered, 0.);
	for (EcSizeT i = 0; i < numFiltered; i++)
	{
		EcReal temp = 0.;
		for (EcSizeT j = 2; j < numCircle - 2; j++)
		{
			temp += tempArray[j][i];
		}
		tempCurrent[i] = temp / (numCircle - 4);
	}

	filteredMotorCurrents.resize(numFiltered);

	for (EcSizeT ii = 0; ii < numFiltered; ++ii)
	{
		filteredMotorCurrents[ii] = m_MotorCurrentLowPassFilters[ii].updateOutput(tempCurrent[ii]);
	}

	return EcFalse;
}

//------------------------------------------------------------------------------
EcBoolean CHansRobotStateObserver::filterDisturbanceTorques
(
const EcRealVector& disturbanceTorques,
EcRealVector& filteredDisturbanceTorques
)
{
	const EcSizeT numFiltered = m_DisturbanceTorqueLowPassFilters.size();
	if (numFiltered != disturbanceTorques.size())
	{
		return EcFalse;
	}

	filteredDisturbanceTorques.resize(numFiltered);

	for (EcSizeT ii = 0; ii < numFiltered; ++ii)
	{
		filteredDisturbanceTorques[ii] = m_DisturbanceTorqueLowPassFilters[ii].updateOutput(disturbanceTorques[ii]);
	}

	return EcTrue;
}

EcBoolean CHansRobotStateObserver::filterEeSensedForces
(
const EcRealVector& sensedEeForce,
EcRealVector& filteredSensedEeForce
)
{
	const EcSizeT numFiltered = m_SensedEeForceLowPassFilters.size();
	if (numFiltered != sensedEeForce.size())
	{
		return EcFalse;
	}

	filteredSensedEeForce.resize(numFiltered);

	for (EcSizeT ii = 0; ii < numFiltered; ++ii)
	{
		filteredSensedEeForce[ii] = m_SensedEeForceLowPassFilters[ii].updateOutput(sensedEeForce[ii]);
	}

	return EcTrue;
}

EcBoolean CHansRobotStateObserver::filterAdmittancePoseDeviation
(
const EcRealVector& deviatePose,
EcRealVector& filteredDeviatePose
)
{
	const EcSizeT numFiltered = m_AdmittancePoseLowPassFilters.size();
	if (numFiltered != deviatePose.size())
	{
		return EcFalse;
	}

	filteredDeviatePose.resize(numFiltered);

	for (EcSizeT ii = 0; ii < numFiltered; ++ii)
	{
		filteredDeviatePose[ii] = m_AdmittancePoseLowPassFilters[ii].updateOutput(deviatePose[ii]);
	}

	m_OldAdmittanceDeviatePose.push_front(filteredDeviatePose);

	return EcTrue;
}

EcBoolean CHansRobotStateObserver::calculateMeanDisturbanceTorques
(
const EcRealVector& disturbanceTorques,
EcRealVector& meanDisturbanceTorques
)
{
	//std::cout << "disturb torque: " << disturbanceTorques[0]<<",";
	const EcSizeT numJoints = m_DisturbanceTorqueLowPassFilters.size();
	if (numJoints != disturbanceTorques.size())
	{
		return EcFalse;
	}
	meanDisturbanceTorques.resize(numJoints);
	
	const EcSizeT numCircle = m_OldDisturbanceTorques.size();
	if (numCircle == 0)
	{
		meanDisturbanceTorques.assign(numJoints, 0.0);
		return EcFalse;
	}
	EcRealVectorVector tempVector;
	for (int i = 0; i < numCircle; i++)
	{
		tempVector.push_back(m_OldDisturbanceTorques[i]);
	}


	for (EcSizeT ii = 0; ii < numJoints; ii++)
	{
		EcReal temp = 0.0;
		for (int j = 0; j < numCircle - 1; j++){
			for (int i = 0; i < numCircle - 1-j; i++){
				if (tempVector[i][ii]>tempVector[i + 1][ii]){
					temp = tempVector[i][ii];
					tempVector[i][ii] = tempVector[i + 1][ii];
					tempVector[i + 1][ii] = temp;
				}
			}
		}
		temp = 0.;
		for (EcSizeT jj = 2; jj < numCircle-2; jj++)
		{
			temp += tempVector[jj][ii];
		}  
		meanDisturbanceTorques[ii] = temp/(numCircle-4);
	}
	//std::cout << meanDisturbanceTorques[0] << std::endl;
	m_OldDisturbanceTorques.push_front(disturbanceTorques);
	//m_OldDisturbanceTorques.front();
	return EcTrue;
}

EcBoolean CHansRobotStateObserver::estimateAdmittanceVelocities
(
EcRealVector& estAdmittanceVelocity
)
{
	if (m_Index2 >= m_OldAdmittanceDeviatePose.size())
	{
		return EcFalse;
	}

	const EcRealVector& positions2 = m_OldAdmittanceDeviatePose[m_Index2];
	const EcRealVector& positions1 = m_OldAdmittanceDeviatePose[m_Index1];
	const EcRealVector& positions0 = m_OldAdmittanceDeviatePose[m_Index0];

	const EcSizeT numJoints = positions0.size();
	if (
		positions1.size() != numJoints ||
		positions2.size() != numJoints ||
		m_AdmittanceVelocityLowPassFilters.size() != numJoints
		)
	{
		return EcFalse;
	}

	estAdmittanceVelocity.resize(numJoints);

	// 2nd order backward finite difference for velocity
	//	f_x = ( 1*f[i-2] - 4*f[i-1] + 3*f[i+0] ) / (2*h)

	for (EcSizeT ii = 0; ii < numJoints; ++ii)
	{
		estAdmittanceVelocity[ii] = (
			1.0 * positions2[ii]
			- 4.0 * positions1[ii]
			+ 3.0 * positions0[ii]
			)
			/ (2.0 * m_StencilSpacing);

		estAdmittanceVelocity[ii] = m_AdmittanceVelocityLowPassFilters[ii].updateOutput(estAdmittanceVelocity[ii]);
	}

	return EcTrue;
}
