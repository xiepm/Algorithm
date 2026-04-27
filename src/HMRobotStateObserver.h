#ifndef HMRobotStateObserver_H_
#define HMRobotStateObserver_H_
/*********************************************************************
* ������Hans Э���������㷨��
* ���ߣ�����
* ���ڣ�2017-05-14
* ��飺�ṩ��������ر������ƺ��˲�����ȹ��ܣ�
* �汾��V1.0.0
*********************************************************************/

//#include <manipulator/ecManipActiveState.h>
//#include <manipulator/ecMassMatrixTool.h>
#include "hansLowPassFilter.h"

//#include <boost/circular_buffer.hpp>
#include "circular_buffer.hpp"

/*
/// forward declarations
class EcIndividualManipulator;
class EcManipulatorLink;
class EcPositionState;
class EcStatedSystemStruct;
*/

class CHansRobotStateObserver
{
#define			currentMedianFilterCount				10
public:
	/// specialized constructor
	/// @param [in] updateTimePeriod The sampling time period in seconds
	/// @param [in] sampleSpacingFactor The sampling factor used to by the finite differencing algorithm to estimate velocity and acceleration from position
	CHansRobotStateObserver
		(
		const EcReal updateTimePeriod,
		const EcU32  sampleSpacingFactor = 4
		);

	/// destructor
	virtual ~CHansRobotStateObserver
		(
		);

	/// copy constructor
	CHansRobotStateObserver
		(
		const CHansRobotStateObserver& orig
		);

	/// assignment operator
	CHansRobotStateObserver& operator=
		(
		const CHansRobotStateObserver& orig
		);

	/// Set the sampling period in seconds
	/// @param [in] samplePeriod the sampling period in seconds
	void setSamplingPeriod
		(
		const EcReal samplePeriod
		);

	/// Get the sampling period in seconds
	/// @return the sampling period in seconds
	EcReal samplingPeriod
		(
		);

	/// @brief Set the sample spacing factor used for calcuating velocity and acceleration using a three point finite difference stencil
	/// @details Set the finite difference spacing factor used to calculate velocity and acceleration. This factor is an unsigned integer multiplier
	///          of the sampling period. See EcMkManipulatorStateObserver::samplingPeriod.
	/// @param [in] sampleSpacingFactor An unsigned integer multipler of the sampling period
	void setSampleSpacingFactor
		(
		const EcU32 sampleSpacingFactor
		);

	/// @brief Get the sample spacing factor used for calcuating velocity and acceleration using a three point finite difference stencil
	/// @details Get the finite difference spacing factor used to calculate velocity and acceleration. This factor is an unsigned integer multiplier
	///          of the sampling period. See EcMkManipulatorStateObserver::samplingPeriod.
	/// @return An unsigned integer multipler of the sampling period
	EcU32 sampleSpacingFactor
		(
		);

	/// @brief Initialize observer
	/// @param [in] jointPositions Current joint positions.
	/// @return True if successful
	virtual EcBoolean initialize
		(
		const EcRealVector& jointPositions
		);

	/// @brief Set low-pass filter time constants
	/// @param [in] positionTimeConstants Time constants for the position low-pass filter.
	/// @param [in] velocityTimeConstants Time constants for the velocity low-pass filter.
	/// @param [in] accelerationTimeConstants Time constants for the acceleration low-pass filter.
	/// @param [in] motorCurrentTimeConstants Time constants for the motor current low-pass filter.
	/// @param [in] disturbanceTorqueTimeConstants Time constants for the disturbance torque low-pass filter.
	virtual EcBoolean setFilterTimeConstants
		(
		const EcRealVector& jointPositions,
		const EcRealVector& positionTimeConstants,
		const EcRealVector& velocityTimeConstants,
		const EcRealVector& accelerationTimeConstants,
		const EcRealVector& motorCurrentTimeConstants,
		const EcRealVector& disturbanceTorqueTimeConstants
		);

	virtual EcBoolean setUniteFilterTimeConstants
		(
		const EcRealVector& jointPositions,
		const EcRealVector& filterTimeConstants,
		const EcRealVector& disturbFilterTimeConstants
		);

	/// @brief Estimate joint states. For a manipulator, the velocity, and accelerations of a joint are estimated.
	/// @param [in] jointPositions Current joint state
	/// @param [out] estJointPositions The estimated (filtered) joint positions
	/// @param [out] estJointVelocities The estimated (filtered) joint velocities
	/// @param [out] estJointAccelerations The estimated (filtered) joint accelerations
	/// @return True if joint states were estimated.
	virtual EcBoolean estimateStates
		(
		const EcRealVector& jointPositions,
		EcRealVector& estJointPositions,
		EcRealVector& estJointVelocities,
		EcRealVector& estJointAccelerations
		);


	virtual void set15066Strategy(bool enable);

	/// @brief Filter motor currents
	/// @param [in] motorCurrents The motor currents at each joint
	/// @param [out] filteredMotorCurrents The low pass filtered motor currents
	/// @return True if the method succeeds
	virtual EcBoolean filterMotorCurrents
		(
		const EcRealVector& motorCurrents,
		EcRealVector& filteredMotorCurrents
		);

	/// @brief Filter Disturbance Torques
	/// @param [in] disturbanceTorques The disturbance torques at each joint
	/// @param [out] filteredDisturbanceTorques The low pass filtered disturbance torques
	/// @return True if the method succeeds
	virtual EcBoolean filterDisturbanceTorques
		(
		const EcRealVector& disturbanceTorques,
		EcRealVector& filteredDisturbanceTorques
		);

	/// @brief Filter Sensed Forces
	/// @param [in] sensed end effector forces
	/// @param [out] filtered The low pass filtered EE forces
	/// @return True if the method succeeds
	virtual EcBoolean filterEeSensedForces
		(
		const EcRealVector& sensedEeForces,
		EcRealVector& filteredSensedEeForces
		);

	/// @brief Filter admittance pose deviation
	/// @param [in] deviate pose
	/// @param [out] filtered the low pass filtered deviate pose
	/// @return True if the method succeeds
	virtual EcBoolean filterAdmittancePoseDeviation
		(
		const EcRealVector& deviatePose,
		EcRealVector& filteredDeviatePose
		);

	virtual EcBoolean estimateAdmittanceVelocities
		(
		EcRealVector& estAdmittanceVelocity
		);

	virtual EcBoolean calculateMeanDisturbanceTorques
		(
		const EcRealVector& disturbanceTorques,
		EcRealVector& meanDisturbanceTorques
		);

protected:

	/// @brief Estimate joint positions
	/// @param [in] jointPositions The current joint positions
	/// @param [out] estJointPositions The estimated (filtered) joint positions
	/// @return True if successful, False otherwise
	virtual EcBoolean estimateJointPositions
		(
		const EcRealVector& jointPositions,
		EcRealVector& estJointPositions
		);


	/// @brief Estimate joint velocities
	/// @param [out] estJointVelocities The estimated (filtered) joint velocities
	/// @return True if successful, False otherwise
	virtual EcBoolean estimateVelocities
		(
		EcRealVector& estJointVelocities
		);

	/// @brief Estimate accelerations
	/// @param [out] estJointAccelerations The estimated (filtered) joint accelerations
	/// @return True if successful, False otherwise
	virtual EcBoolean estimateAccelerations
		(
		EcRealVector& estJointAccelerations
		);

	virtual EcBoolean estimateJerks
		(
		EcRealVector& estJointJerks
		);

	virtual void sortFunction
		(
		EcRealVectorVector& sortArray
		);


protected:

	EcBoolean                               m_IsInitialized;    ///< Flag set if the class is initialized
	EcReal                                  m_SamplePeriod;     ///< Sampling time of the process
	EcReal                                  m_StencilSpacing;   ///< Spacing for the finite difference stencil
	EcU32                                   m_StencileSize;     ///< Size of the finite difference stencil (currently 3 for velocity and acceleration approximations)
	EcU32                                   m_SampleSpacingFactor; ///< The sample spacing factor used in forming the finite difference stencils.


	EcU32                                   m_Index0;    ///< Index to the first stencil point (i) for finite differencing.
	EcU32                                   m_Index1;    ///< Index to the second stencil point (i-h) for finite differencing.
	EcU32                                   m_Index2;    ///< Index to the third stencil point (i-2h) for finite differencing.

	hansLowPassFilterVector                  m_PositionLowPassFilters;  ///< Low pass filter for joint positions
	hansLowPassFilterVector                  m_VelocityLowPassFilters;  ///< Low pass filter for joint velocities
	hansLowPassFilterVector                  m_AccelerationLowPassFilters;  ///< Low pass filter for joint accelerations
	hansLowPassFilterVector                  m_JerkLowPassFilters;  ///< Low pass filter for joint accelerations
	hansLowPassFilterVector                  m_MotorCurrentLowPassFilters;  ///< Low pass filter for joint motor currents
	hansLowPassFilterVector                  m_DisturbanceTorqueLowPassFilters;  ///< Low pass filter for disturbance torques
	hansLowPassFilterVector                  m_SensedEeForceLowPassFilters;  ///< Low pass filter for sensed ee forces
	hansLowPassFilterVector					 m_AdmittancePoseLowPassFilters; ///< Low pass filter for admittance control pose deviation;
	hansLowPassFilterVector					 m_AdmittanceVelocityLowPassFilters; ///< Low pass filter for admittance control pose deviation;



	circular_buffer<EcRealVector>	m_OldJointPositions;     ///< A circular buffer for time series joint position data
	circular_buffer<EcRealVector>	m_OldJointVelocity;     ///< A circular buffer for time series joint position data
	circular_buffer<EcRealVector>	m_OldAdmittanceDeviatePose;     ///< A circular buffer for time series admittance deviate pose data
	circular_buffer<EcRealVector>	m_OldDisturbanceTorques;		/// A circular buffer for disturbance joint torques
	circular_buffer<EcRealVector>    m_OldJointCurrents;
	EcRealVector							m_previousMotorCurrent;		// 上一周期的电机电流；
	EcBoolean								b_previousUnNomalCurrentStatus;
	EcU32Vector								m_lastUnNomalCount;
	EcRealVector							m_errorDeltaThresholdCurrent;		// 关节电流每周期异常判断的电流变化值；



	EcBoolean								b_is15066Stategy;					// 是否为15066策略，电流是否需要滤波；
};

#endif