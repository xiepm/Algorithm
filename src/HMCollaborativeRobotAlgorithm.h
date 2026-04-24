#ifndef HMCollaborativeRobotAlgorithm_H_
#define HMCollaborativeRobotAlgorithm_H_

/*********************************************************************
* Name    :   Collaborative algorithm class
* Author  :   zhangp
* Time    :   2017-03-27
* Function：  Estimate robot states and calculate dynamcis model.
* Version ：  V1.0.0
*********************************************************************/
#include <vector>
#include <string>
#include "HMRobotStateObserver.h"
//#include "HMAlgorithm.h"
//#include "admittanceController.h" 
#include "momentumObserver.h"
#include "frictionModel.h"
#include "frames.hpp"
#include "dynamics/dynamicsBase.h"
#include "dynamics/elfinDynamics.h"
#include "dynamics/urDynamics.h"
#include <memory>

typedef enum EN_motionConstraintStatus {
	constraint_normal = 0,
	constraint_errorStatus,
	constraint_overJointVelocitiesLimit,
	constraint_overLinearVelocitiesLimit,
	constraint_overMomentumLimit,
	constraint_overPowerLImit
}ENMotionConstraintStatus;

typedef enum EN_AssistiveState {
	assistive_normal = 0,
	assistive_errorInitParams,
	assistive_overJointsRangeLimit,
	assistive_overJointsCurrent,
	assistive_errorRobotState,
	assistive_steadyStatus,
	assistive_severCollision,
	assistive_finshedReadyToClose
	/*Return:
	0，正常；
	1，参数初始化错误；
	2，超边界，根据输出jointCollisionStatus 确定那个关节；
	3，过电流，根据输出jointCollisionStatus 确定那个关节；
	4. 机器人状态错误，提醒用户检测当前的负载、关节零点是否准确；
	5. 机器人稳定状态，在碰撞检测状态下，可以切换为位置模式
	6. 机器人处于严重碰撞模式，应当直接去使能
	7. overMotionConstraints, 超出机器人所允许的运动约束限制，包括速度、功率、动量等

	*/
}ENAssistiveState;

typedef enum EN_CollisionType {
	collision_none = 0,
	collision_deviation,
	collision_Observer
}ENCollisionType;

typedef enum EN_DriveBackMode {
	mode_PowerOff = 0,
	mode_ForceFree,
	mode_LimitMotion
}ENDriveBackMode;

typedef enum EN_DriveBackStatus {
	status_statePosition = 0,
	status_strongDrive,
	status_awayPosition,
	status_controlPosition,
	status_stopMoving
}ENDriveBackStatus;


class CHansCollaborativeAlgorithm
{
#define				NonZeroVelocity						 0.001
#define				AssistiveModeStartMonitoringTime	 0.05
#define				AssistiveModeStartTime				 0.30
#define				AssistiveSafeStartJointMotion		 0.052				// 80ms开始检测，280ms结束，中间的运动位置于启动时的位置不能超过3°

#define				MaxDriveBackPosition				0.0088		
#define				DriveBackClosedTime					0.20
#define				MaxDriveBackEnduranceTime			0.5
#define				DriveBackThresholdVelocity			0.1			
#define				LimitDrivebackFrictionScale			5

#define				MaxAllowPowerInAssistiveMode		250
#define				MaxAllowMomentumInAssistiveMode		25					// 从10修改为20

#define				driveBackJointPosition_away			0.0087				// 反弹到判定为脱离的关节位置0.5°	
#define				driveBackJointPosition_control		0.0175				// 反弹到期望目标的关节位置1°；



public:
	static std::shared_ptr<CHansCollaborativeAlgorithm> create
	(
		const EcRealVector& jointPositions,
		const EcReal updateTimePeriod,
		const int robotType
	);

	/// destructor
	~CHansCollaborativeAlgorithm();

protected:
	CHansCollaborativeAlgorithm
	(
		const EcReal updateTimePeriod
	);

public:
	EcBoolean initializeStates
	(
		const EcRealVector& jointPositions,
		const int robotType
	);

	EcBoolean updateStateEstimates
	(
		const EcRealVector& jointPositions,
		const EcRealVector& motorCurrents,
		const EcReal& currentTime
	);

	EcBoolean updateJointTemperatures
	(
		const EcRealVector& jointTemperatures
	);

	EcBoolean updateStateEstimates
	(
		const EcRealVector& commandJointPositions,
		const EcRealVector& jointPositions,
		const EcRealVector& motorCurrents,
		const EcReal& currentTime
	);


	EcBoolean checkForCollision
	(
		EcBooleanVector& jointCollisionStatus,
		ENCollisionType& type
	);


	EcBoolean getEstimatedState
	(
		EcRealVector& jointPostions,
		EcRealVector& jointVelocities,
		EcRealVector& jointAccelerations,
		EcRealVector& motorCurrents,
		EcRealVector& sensedTorques,
		EcRealVector& estimatedTorques,
		EcRealVector& disturbanceTorques
	);



	void setDynamicsLinearParameters
	(
		const EcRealVector& params
	);

	EcBoolean setFilterTimeConstant
	(
		const EcRealVector& jointsPosition,
		const EcRealVector& filterTimeConstant,
		const EcRealVector& disturbFilterTimeConstant
	);

	void setRobotMountingAngles
	(
		const EcReal& rotation,
		const EcReal& tilt
	);

	EcBoolean setActutorMaxCurrentLimits
	(
		const EcRealVector& maxCurrents
	);

	void setGravityVector
	(
		const EcRealVector& gravity
	);

	EcBoolean setPayloadMassProperties
	(
		const EcReal& mass,
		const EcRealVector& centerofMass
	);

	EcBoolean setActutorTorqueConstants
	(
		const EcRealVector& torqueConstant,
		const EcRealVector& gearRatio,
		const EcRealVector& maxEfficiency
	);

	void setActutorDampConstants
	(
		const EcRealVector& damp
	);

	EcBoolean setRobotDHParameters
	(
		const EcRealVector& kinematcisParam
	);

	EcBoolean setCollaborativeJointSpaceLimits
	(
		const EcRealVector& upperJointLimits,
		const EcRealVector& lowerJointLimits
	);

	EcBoolean setCollisionStopThresholds
	(
		const EcRealVector& collisionStopThresholds
	);

	// 动量检测的预置
	EcBoolean setCollisionStopInMomentumThresholds
	(
		const EcRealVector& collisionStopThresholds
	);

	void setAssistiveModeCollisionStopThresholds
	(
		const EcRealVector& assistiveModeCollisionStopThresholds
	);

	void setFrictionCompensatoryFactor
	(
		const EcRealVector& frictionCompensatoryFactor
	);

	void setStartCompensateFrictionFactor
	(
		const EcRealVector& compensateFactor
	);

	// add new parameters for modeling temperature;
	// default: factor={1.0,1.0,1.0,1.0,1.0,1.0};
	void setFrictionParamsWithTemperature
	(
		const EcRealVector& factor
	);



	void setLowVelocityThreshold
	(
		const EcRealVector& velocityThreshold
	);


	void setMaxJointVelocity
	(
		const EcRealVector& maxJointVelocity
	);

	void setStartTimeForCheckAssisitiveMode
	(
		EcReal assistiveCheckTime
	);


	// 启动零力示教时，调用启动函数，触发异常状态的保护判断
	void setStartAssistiveMode
	(
	);

	// 预关闭零力示教，让关节的速度尽快接近于0；
	void setReadyToCloseAssistiveMode();


	// 在位置运动模式下检测到碰撞后，在启动零力示教之前，调用此函数后触发回弹模式；
	void setCollisionDriveBackMode
	(
		const ENDriveBackMode drivebackMode
	);

	// 限制碰撞后，关节运动的距离和运动时间；
	// distanceLimit: 最大转角运动限制(rad)  0.087 rad = 5°;
	// timeDuration: 最大运动时间（s) 0.5s;
	void setMotionLimitInDriveBack
	(
		const EcReal distanceLimit,
		const EcReal timeDuration
	);

	void setMaxPowerAndMomentumConstraints
	(
		const EcReal maxPower,
		const EcReal maxMomentum,
		const EcRealVector& maxJointPowers
	);

	bool calculatePowerAndMomentumConstraints
	(
		const EcReal userDefineOverride,
		const EcRealVector& jointVoltages,
		EcBoolean& constraintActive,						// need to re-plan the motion at rising edge;
		EcReal& velFactorConstraint,					// constraint factor for joint velocities;
		EcReal& accFactorConstraint,					// constraint factor for joint accelerations;
		EcReal& electircPower,							// P = U * I;
		EcReal& physicsPower,							// P = Torque * omega;
		EcReal& momentum								// momemtum = mass * omega;
	);

	bool calculatePowerAndMomentum(
		const EcRealVector& jointVoltages,
		EcReal& electircPower, // P = U * I;
		EcReal& physicsPower,  // P = Torque * omega;
		EcReal& momentum,	   // momemtum = mass * omega;
		EcRealVector& jointPowers
	);


	/*Return: 0，正常；
			  1，参数初始化错误；
			  2，超边界，根据输出jointCollisionStatus 确定那个关节；
			  3，过电流，根据输出jointCollisionStatus 确定那个关节；
	*/
	ENAssistiveState getAssistiveModeMotorCurrentCommands
	(
		EcRealVector& motorCurrentCommands,
		EcBooleanVector& jointCollisionStatus
	);

	// gravity + ( coulomb + viscous)
	EcBoolean getGravityCompensationCurrentCommands
	(
		EcRealVector& motorCurrentCommands,
		EcBooleanVector& jointCollisionStatus
	);

	// 力矩前馈：补偿重力矩 
	// gravity + (viscous)
	void getGravityTorqueCurrentCommands
	(
		const EcReal compensateRatio,
		EcRealVector& motorCurrentCommands
	);

	// codesys version
	// gravity + (viscous)
	void getGravityTorqueCurrentCommands
	(
		const EcReal compensateRatio,
		const EcRealVector& jointPosition,
		EcRealVector& motorCurrentCommands
	);

	// 力矩前馈：补偿动力学驱动力矩（Todo: 应当以命令位置为计算参数，避免滞后，实际位置已经有比较大的滞后。）
	void getComputeTorqueCurrentCommands
	(
		const EcReal compensateRatio,
		EcRealVector& motorCurrentCommands
	);

	void getComputeTorqueCurrentCommands
	(
		const EcReal compensateRatio,
		const EcRealVector& jointPosition,
		const EcRealVector& jointVelocity,
		const EcRealVector& jointAcceleration,
		EcRealVector& motorCurrentCommands
	);

	void getFlexibleCompensateGravityTorque
	(
		const EcRealVector& jointPosition,
		EcRealVector& gravTorque
	);



	/// @brief 监控机器人的运动状态是否超出安全限制；
	/// @param [in] isAssistiveMode 当前的状态是否为零力示教模式（最好从模组PDO控制字中获取这个状态，而非从主线程的状态机中）
	/// @return 违反运动约束的类型；
	ENMotionConstraintStatus monitorMotionConstraintsStatus
	(
		EcBoolean isAssistiveMode
	);

	/// @brief 清空零力示教下触发的异常状态；
	void resetCobotStatus();

	void getSettingParameters
	(
		EcRealVector& kinParams,
		EcRealVector& actuatorDamp,
		EcRealVector& torqueConstant,
		EcRealVector& gearRatio,
		EcRealVector& maxEfficiency,
		EcRealVector& maxActuatorCurrents,
		EcReal& rotationofMounting,		/* rotate robot base mounting angles(deg) */
		EcReal& tiltofMounting,		/* tilt robot base mounting angles(deg) */
		EcRealVector& upperJointLimits,
		EcRealVector& lowerJointLimits,
		EcRealVector& dynamicsParams,
		EcRealVector& collisionStopThresholds,		/* joint collisiton threshold, */
		EcRealVector& collisionStopInMomentumThresholds,		/* joint collisiton threshold, */
		EcRealVector& assistiveModeCollisionStopThresholds,
		EcRealVector& frictionCompensatoryFactor,		/* default setting: J1="0.5" J2="0.5" J3="0.4" J4="0.6" J5="0.7" J6="0.7" */
		EcRealVector& frictionCompensatoryFactorII,		/* default seting [1.0], */
		EcRealVector& lowVelocityThreshold,		/* decide whether the joints are in low velocity status, */
		EcRealVector& maxJointVelocity,
		EcReal& maxPowerConstraint,		/* power */
		EcReal& maxMomentumConstraint,		/* momentum */
		EcReal& distanceLimitInDriveBack,		/* allowing joint motion(degree,  °) in drive back mode, */
		EcReal& timeDurationInDriveBack,		/* allowing time duration(second, s) in drive back mode, */
		EcReal& mass,
		EcRealVector& centerofMass,		/* (millimeter) */
		bool& isAssistiveMode,
		int& drivebackMode,
		EcReal& assistiveCheckTime		/* the start time for check robot status in assistive mode, second ,s; */
	);

private:
	EcReal sign(const EcReal& x);

	void setVibrationPeriod();

	void setFrictionModel
	(
		const EcRealVector& params
	);

	EcBoolean checkSafeStartAssistiveMode
	(
	);
	EcBoolean checkCloseDriveBackMode
	(
	);
	EcBoolean checkCompleteReadyToCloseAssisitiveMode();

	EcBoolean calculateCompensateCoulombFriction
	(
		const EcRealVector& jointVelocity,
		EcRealVector& coulombFriction
	);

	EcReal generateVibrationSignal
	(
		const EcU32 jointIndex,
		const EcReal& friction
	);

	EcBoolean calculateMaxRectifyEstimateJointTorques
	(
	);

	EcReal calculateJointFriction
	(
		const EcU32 jointIndex,
		const EcReal jointVelocity
	);

	EcBoolean calculateDisturbanceTorques
	(
		const EcRealVector& motorCurrents,
		const EcRealVector& jointPositions,
		const EcRealVector& jointVelocities,
		const EcRealVector& jointAccelerations,
		EcRealVector& sensedTorques,
		EcRealVector& estimatedTorques,
		EcRealVector& disturbanceTorques
	);

	EcBoolean calculateEstimateTorques
	(
		const EcRealVector& jointPositions,
		const EcRealVector& jointVelocities,
		const EcRealVector& jointAccelerations,
		EcRealVector& estimatedTorques
	);




	EcBoolean getSensedTorques
	(
		const EcRealVector& motorCurrents,
		EcRealVector& sensedTorques
	);

	EcBoolean updateCollisionStopStatus
	(
		const EcRealVector& torqueThresholds,
		const EcRealVector& disturbanceTorques,
		EcBooleanVector& collisionStopStatus
	);

	// not using yet.
	void setMaxJointAccelerations
	(
		const EcRealVector& maxJointAccelerations
	);

	void updateDynamicsCollisionStopThreshold(EcRealVector& enchanceDisturbJointTorques);

	void saturationFunction(EcReal& value, EcSizeT Index);

	void calculateCoulombFriction(const EcRealVector& jointPosition, EcRealVector& friction);

	EcBoolean calculateCartesianVelocityScale(EcReal& cartScale);





private:
	dynBasePtr								m_dynBase;						// 动力学基类；
	int										m_robotType;

	EcBoolean								m_logStatus;
	EcBoolean                               m_IsInitialized;                ///< flag set to true if the class instance is initialized
	EcU32									m_NumJoints;			    	//Number of Robot
	EcReal									m_d1, m_d4, m_d6, m_a2; 		//Link length of Elfin
	EcRealVector							m_kinParams;
	//EcRealVector							m_actualJointPosition;				//Current Joint Position
	CHansRobotStateObserver					m_StateEstimator;				//Robot state estimator；
	CHansRobotStateObserver					m_commandStateEstimator;
	frictionModel							m_frictionModel;


	EcBooleanVector							m_collisionStopStatus;
	EcBooleanVector							m_driveBackCollisionStopStatus;
	EcBooleanVector							m_driveBackStatus;
	EcBooleanVector							m_driveBackBrakingStatus;
	EcRealVector							m_driveBackTorques;
	EcRealVector							m_driveBackStartPosition;

	ENDriveBackMode							m_drivebackMode;				// 碰撞后的反应策略选择
	EcReal									m_allowBackDistance;			// 允许碰撞后运动的距离；
	EcReal									m_allowMotionDuration;			// 允许碰撞后运动的时间；
	EcRealVector							m_startDriveBackJointPosition;	// 碰撞发生时的关节位置；



	EcReal									m_payloadMass;					//the mass of payload (kg);
	EcReal									m_prePayloadMass;					//the mass of payload (kg);
	EcBoolean								b_newPayLoadStatus;				// 判断是否设置新的负载；
	EcU32									m_payloadCount;
	EcRealVector							m_centerofMass;					// the mass centor of payload (m);

	EcReal									m_baseMountingRotation;			//Base mounting rotation rad；
	EcReal									m_baseMountingTilt;				//Base mounting tilt rad；

	EcRealVector                            m_FilteredJointPositions;     ///< Filtered joint positions (rad)
	EcRealVector                            m_FilteredJointVelocities;    ///< Filtered joint velocities (rad/s)
	EcRealVector                            m_FilteredJointAccelerations; ///< Filtered joint accelerations (rad/s^2)
	EcRealVector                            m_FilteredMotorCurrents;      ///< Filtered motor currents (amps)
	EcRealVector							m_jointVoltages;			  /// motor voltages(V);	

	EcRealVector                            m_FilteredCommandJointPositions;     ///< Filtered joint positions (rad)
	EcRealVector                            m_FilteredCommandJointVelocities;    ///< Filtered joint velocities (rad/s)
	EcRealVector                            m_FilteredCommandJointAccelerations; ///< Filtered joint accelerations (rad/s^2)


	EcBoolean								m_EeCollisiontStopStatus;		//End effector collision stop status； 
	EcReal									m_EeCollisiontStopThreshold;	//End effector collision stop threshold； 
	EcReal									m_EeResultantForce;				//End effector resultant force；
	EcRealVector							m_EeOutputForces;				//End effector output force vector；

	EcRealVector                            m_CollisionStopThresholds;    ///< Collision stop threshold 
	EcRealVector                            m_CollisionStopDynamicsThresholds;
	EcRealVector							m_AssistiveModeCollisionStopThresholds;

	EcRealVector							m_DynamicsLinearParameters;		//Dynamics linear parameters；
	EcRealVector							m_DynamicsParameters;			//Dynamics parameters；

	EcReal									m_nonZeroVelocity;				//Non-zero velocity threshold
	EcRealVector							m_CoulombFriction;				//Joint coulomb friction
	EcRealVector							m_ViscousFrictionCoefficient;	//Joint viscous friction
	EcRealVector							m_jointLimitAgainstForceEquivalent;//Joint limit against force equivalent
	EcReal									m_timeStep;						//Time step
	EcRealVector							m_maxJointVelocitiesInAssistiveMode;					//Max velocity in assistive mode
	EcReal									m_constraintsFrictionCompensatoryFactor;		// try to limit the friction compensatory factor;

	EcRealVector							m_MaxActutorCurrents;
	EcRealVector							m_MaxActutorTorques;

	EcReal									m_gx;							//Gravity component
	EcReal									m_gy;
	EcReal									m_gz;
	KDL::Vector								m_gravity;						//Gravity vector

	EcReal									m_currentTime;
	EcRealVector							m_vibrationPeriod;
	EcRealVector							m_VibrationAmplitude;			//the amplitude of vibration
	//EcRealVector							m_coulombCoefficient;			//the amplitude of vibration
	EcRealVector							m_maxJointVeloctiy;
	EcRealVector							m_maxJointAccelerations;
	EcU32									m_dropDataCount;				// 连续丢帧的个数；

	//need to read form config files
	EcRealVector							m_frictionCompensatoryFactor;
	EcRealVector							m_startCompensateFrictionFactor;
	//EcRealVector							m_viscousCompensatoryFactor;
	EcRealVector							m_UpperJointLimits;				//Upper joint limits
	EcRealVector							m_LowerJointLimits;				//Lower joint limits
	EcRealVector							m_jointRange;
	EcRealVector							m_slowDownBoundary;
	EcRealVector							m_stopBoundary;
	EcRealVector							m_lowVelocityThresholds;		//low velocity thresholds
	EcRealVector							m_zeroVelocityThresholds;		//zero velocity thresholds
	EcRealVector							m_maxDitherPosition;
	EcRealVector							m_minDitherPosition;
	EcRealVector							m_ditherAmpScale;
	EcU32									m_ditherLoopCount;

	EcRealVector							m_torqueConstant;				//motor torque constant
	EcRealVector								m_gearRatio;					//Module gear ratio
	EcRealVector							m_maxEfficiency;				//Module translate efficiency
	EcRealVector							m_AdjustedTorqueConstants;		//Module torque constants


	EcRealVector                            m_DisturbanceJointTorques;		  ///< Disturbance joint torques (Nm)
	EcRealVector							m_meanDisturbanceJointTorques;	///mean disturbance torques in 10 circle times(Nm)
	EcRealVector                            m_SensedJointTorques;			    ///< Sensed joint torques (Nm)
	EcRealVector                            m_EstimatedJointTorques;		    ///< Esimtated joint torques (Nm)
	EcRealVector							m_FilteredSensedEeForces;		  ///< Sensed end effector force(N,Nm)
	EcRealVector							m_FilteredAdmittanceDeviatePose;	///< Admittance deviate pose(m,rad)
	EcRealVector							m_filteredAdmittanceVelocity;

	EcReal									m_endEffectorMass;				//
	EcReal									m_endEffectorInertia;			//
	EcRealVector							m_selectedPositionCoeff;
	EcRealVector							m_viscousVelocityCoeff;

	EcRealVector							m_eeCalcVelocity;
	EcRealVector							m_eeCalcPosition;
	EcRealVector							m_eeCurrentPosition;
	EcReal									m_updateTimePeriod;
	EcReal									m_stopGenerateFrictCounts;
	EcReal									m_stopStateTimeCounts;

	EcReal									m_JacobianDeterminant;			//The determinant of Jacobian matrix
	EcBoolean								m_IsAdmittanceInitialized;

	EcReal									m_currentAssistiveTime;
	EcU32									m_AssistiveStartCount;
	EcU32Vector								m_AssistiveSafeAccumulateCount;
	EcRealVector							m_AssistiveStartSumAccel;
	EcRealVector							m_AssistiveSafeStartAccel;				// 各关节允许的安全启动加速度；
	EcRealVector							m_AssistiveStartJointPosition;			// 启动零力示教时关节的位置；
	EcRealVector							m_AssistiveStartSensedTorques;			// 启动零力示教时的关节检测力矩；

	EcBoolean								b_isDriveBackAssistiveMode;
	EcBoolean								b_isDriveBackMode;
	EcReal									m_currentDriveBackTime;
	EcReal									m_driveBackDuranceTime;
	EcRealVector							m_driveBackDisturbancesThreshold;

	EcReal									m_assistiveCheckTime;

	EcU32Vector								m_overZeroVelCount;						// 在碰撞反弹的stopping状态下，通过零点的次数；
	EcRealVector							m_actualJointPosition;					// 无滤波关节位置；
	EcRealVector							m_actualJointVel;						// 无滤波关节速度；
	EcRealVector							m_actualPreviousJointVel;				// 无滤波上一周期的关节速度；
	EcRealVector							m_startCollisionJointVel;				// 碰撞检测瞬间的关节速度；
	std::vector< ENDriveBackStatus>			m_driveBackStatusVector;				// 各关节的反弹状态；（控制状态迁移）
	EcRealVector							m_sensedRawTorque;

	// 关闭零力示教
	EcBoolean								b_isReadyToCloseMode;
	EcReal									m_currentReadyToCloseAssistiveTime;
	EcReal									m_viscousRatioForCloseAssistive;
	EcRealVector							m_startCloseAssistiveJointVel;			// 保存反弹开始时的线速度，避免速度振荡；
	EcBooleanVector							b_havedReverseJointVel;


	// Admitance control
	//CAdmittancePositionControllerSharedPtr				m_sensorlessAdmitControl;
	EcBoolean								b_isSensorlessAdmittanceMode;

	// momentum
	EcBoolean								b_isUsingMomentumObserver;
	CMomentumSharedPtr						m_momentumObserver;
	EcRealVector							m_observerTorques;
	EcRealVector							m_actualJointPositions;
	EcRealVector							m_dampEfficiency;
	EcRealVector							m_dampSetting;
	EcRealVector							m_MomentumCollisionThresholds;
	EcRealVector							m_massThresholdTorques;
	EcRealVector							m_maxJointTorques;
private:
	EcRealVector							m_zeroVector;
	EcBooleanVector							b_falseVector;
	static EcReal							vibrationPeriod;				//

private:
	EcReal									m_alphaL, m_alphaH;
	EcReal									m_velThreshold;
	EcReal									m_Rw;


	EcReal									m_maxConstraintPower;
	EcReal									m_maxConstraintMomentum;
	EcReal									m_maxPower;
	EcReal									m_maxMomentum;
	EcRealVector							m_maxJointPowers;
	EcBoolean								b_activeConstraintPower;
	EcReal									m_velFactorConstraint;
	EcReal									m_preVelFactorConstraint;
	EcReal									m_accFactorConstraint;
	EcBoolean								b_constraintPowerStatus;
	boost::circular_buffer<EcReal> 			m_meanFilteredVelConstraint;



	EcRealVector							m_alpha;
	EcRealVector							m_omegaK;
	EcRealVector							m_KVector;
	EcRealVector							m_frictionRatio2;
	EcRealVector							m_calcCoulombFriction;
	EcRealVector							m_jointMPosition;
	EcRealVector							m_jointSidePosition;


	EcRealVector							m_jointTemperatures;
	EcRealVector							m_friTemperaturesParams;
	//KDL::HMAlgorithm						m_hmAlgorithm;


	ENAssistiveState						m_AssistiveState;
	EcBoolean								b_previousAssisteErrorState;			// 上次启动的零力示教模式，是否存在异常退出；
	EcReal									m_motionConstraintScale;				//  若上次启动的零力示教存在异常，则讲最大允许速度降低，持续3s；

	EcRealVectorVector						m_loopJointPosition;
	EcRealVectorVector						m_loopSensedCurrent;
public:
};

typedef std::shared_ptr<CHansCollaborativeAlgorithm> CHansCollaborativeAlgorithmSharedPtr;
#endif