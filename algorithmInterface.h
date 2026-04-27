
#include "CmpStd.h"

// 初始化协作算法；
extern void initializeCollaborativeAlgorithm
(
	RTS_IEC_LREAL* jointPosition,
	RTS_IEC_LREAL updatePeriod,
	RTS_IEC_INT robotType,				/* VAR_INPUT */	/* 0:elfin; 1:UR; */
	RTS_IEC_LREAL* kinParams		/* VAR_INPUT */	/* elfin[d1,d4,d6,a2]; UR[d1,d4,d5,d6,a2,a3]; */
);

// 更新当前的机器人状态，包括命令位置、实际位置、电流和系统时间；
extern RTS_IEC_BOOL updateStateEstimates
(
	RTS_IEC_LREAL* commandJointPositions,
	RTS_IEC_LREAL* jointPositions,
	RTS_IEC_LREAL* motorCurrents,
	RTS_IEC_LREAL currentTime
);

/// @brief 是否启用15066碰撞检测策略（取消位置、电流的滤波，响应速度更快，但对本体的平滑性要求跟高）
/// @param enable 
extern void set15066StrategyInterface(RTS_IEC_BOOL enable);

// 获取机器人当前是否触发碰撞；各关节的碰撞状态
extern RTS_IEC_BOOL checkForCollision
(
	RTS_IEC_BOOL* jointCollisionStatus
);

// 获取当前机器人滤波后的状态，位置速度加速度、电流、力矩；
extern RTS_IEC_BOOL getEstimatedState
(
	RTS_IEC_LREAL* jointPostions,
	RTS_IEC_LREAL* jointVelocities,
	RTS_IEC_LREAL* jointAccelerations,
	RTS_IEC_LREAL* motorCurrents,
	RTS_IEC_LREAL* sensedTorques,
	RTS_IEC_LREAL* estimatedTorques,
	RTS_IEC_LREAL* disturbanceTorques
);

// 监控机器人的运动状态是否已超过受限制，需区分零力示教和位置运动模式
extern RTS_IEC_INT monitorMotionConstraintStatus
(
	RTS_IEC_BOOL isAssistiveMode
);

// 重置机器人状态
extern void resetCobotStatus();


// 设置DH参数；
extern void setRobotDHParameters
(
	RTS_IEC_LREAL* kinematcisParam
);


// 设置动力学参数；78个
extern void setDynamicsLinearParameters
(
	RTS_IEC_LREAL* params
);

// 设置低通滤波的时间常数，如不设置，则为默认值；（暂不用设置）
extern RTS_IEC_BOOL setFilterTimeConstant
(
	RTS_IEC_LREAL* jointsPosition,
	RTS_IEC_LREAL* filterTimeConstant
);

// 设置基座的安装角度；
extern void setRobotMountingAngles
(
	const RTS_IEC_LREAL rotation,
	const RTS_IEC_LREAL tilt
);

// 设置允许的最大电流，零力示教状态下，若超过此置，则会报错；
extern void setActutorMaxCurrentLimits
(
	RTS_IEC_LREAL* maxCurrents
);

// 设置末端的负载；
extern RTS_IEC_BOOL setPayloadMassProperties
(
	const RTS_IEC_LREAL mass,
	RTS_IEC_LREAL* centerofMass
);

// 设置驱动器的参数，包括力矩常数和减速比、传动效率；
extern void setActutorTorqueConstants
(
	RTS_IEC_LREAL* torqueConstant,
	RTS_IEC_LREAL* gearRatio,
	RTS_IEC_LREAL* maxEfficiency
);


// 设置判断碰撞的关节阈值；
extern void setCollisionStopThresholds
(
	RTS_IEC_LREAL* collisionStopThresholds
);

// 设置基于动量的关节碰撞检测阈值（第二组参数）
extern void setCollisionStopInMomentumThresholds
(
	RTS_IEC_LREAL* thresholds
);


// 设置零力示教状态下的判断碰撞阈值；
extern void setAssistiveModeCollisionStopThresholds
(
	RTS_IEC_LREAL* assistiveModeCollisionStopThresholds
);

/// @brief 设置关节的运动边界
extern void setCollaborativeJointSpaceLimits
(
	RTS_IEC_LREAL* upperJointLimits,
	RTS_IEC_LREAL* lowerJointLimits
);

// 设置摩擦力的补偿系数；
extern void setFrictionCompensatoryFactor
(
	RTS_IEC_LREAL* frictionCompensatoryFactor
);

// 设置判断为低速的阈值；
extern void setLowVelocityThreshold
(
	RTS_IEC_LREAL* velocityThreshold
);

// 设置关节允许的最大速度；
extern RTS_IEC_BOOL setMaxJointVelocity
(
	RTS_IEC_LREAL* maxJointVelocity
);

// 开启零力示教时，需要调用1次；
extern void setStartAssistiveMode
(
);

// 预关闭零力示教，让关节的速度尽快接近于0；
extern void setReadyToCloseAssistiveMode
(
);

// 设置碰撞反弹的约束（已弃用）
extern RTS_IEC_BOOL setMotionLimitInDriveback
(
	RTS_IEC_LREAL distanceLimit,		/* VAR_INPUT */	/* allowing joint motion(degree,  °) in drive back mode; */
	RTS_IEC_LREAL timeDuration			/* VAR_INPUT */	/* allowing time duration(second, s) in drive back mode; */
);


// 设置开启零力示教进行启动检测的时间，默认为0.08s
extern RTS_IEC_BOOL setStartTimeForCheckAssistiveMode
(
	RTS_IEC_LREAL assistiveCheckTime	/* VAR_INPUT */	/* the start time for check robot status in assistive mode, second ,s; */
);




// 获取零力示教时所需的关节电流和关节的碰撞状态，返回值预示机器人当前的状态；
/*
0，正常；
1，参数初始化错误；
2，超边界，根据输出jointCollisionStatus 确定那个关节；
3，过电流，根据输出jointCollisionStatus 确定那个关节；
4. 机器人状态错误，提醒用户检测当前的负载、关节零点是否准确；
5. 机器人稳定状态，在碰撞检测状态下，可以切换为位置模式
6. 机器人处于严重碰撞模式，应当直接去使能
7. 关节速度已接近为0，允许关闭零力示教；
*/

extern RTS_IEC_INT getAssistiveModeMotorCurrentCommands
(
	RTS_IEC_LREAL* motorCurrentCommands,
	RTS_IEC_BOOL* jointCollisionStatus
);

// 按照设定的百分比，补偿驱动力矩；
extern void getComputeTorqueCurrentCommands
(
	const RTS_IEC_LREAL compensateRatio,
	RTS_IEC_LREAL* jointPosition,
	RTS_IEC_LREAL* jointVelocity,
	RTS_IEC_LREAL* jointAcceleration,
	RTS_IEC_LREAL* motorCurrentCommands
);

// 按照设定的百分比，补偿重力矩；
extern void getGravityTorqueCurrentCommands
(
	const RTS_IEC_LREAL compensateRatio,
	RTS_IEC_LREAL* jointPosition,
	RTS_IEC_LREAL* motorCurrentCommands
);

extern void getGravityTorqueForFlexibleCompensate
(
	RTS_IEC_LREAL* jointPosition,
	RTS_IEC_LREAL* gravTorque
);


// 获取关节惯量
extern void getJointInertia
(
	RTS_IEC_LREAL* jointPosition,
	RTS_IEC_LREAL* inertia
);

// 设置damp参数；
extern void setActutorDampConstants
(
	RTS_IEC_LREAL* damp
);

// 设置碰撞后零力示教的处理模式；
//	/* mode_PowerOff = 0,	servo off; mode_ForceFree = 1,  switch to force free control mode;	mode_LimitMotion =2, drive back mode, with limit joint motion and time duration; */
extern void setCollisionDriveBackMode(RTS_IEC_INT driveBackMode);


//*动态摩擦力矩的补偿系数，用于调节关节在动态运行时的程度；
extern void setFrictionCompensatoryFactorII
(
	RTS_IEC_LREAL* factor
);

/// @brief 设置允许的最大功率和动量；
extern void setMaxPowerAndMomentumConstraints
(
	const RTS_IEC_LREAL maxPower,
	const RTS_IEC_LREAL maxMomentum,
	RTS_IEC_LREAL* jointPowers
);

/// @brief 输出速比约束（已弃用）,计算当前功率和动量
extern RTS_IEC_BOOL calculatePowerAndMomentumConstraints
(
	RTS_IEC_LREAL userDefineOverride,
	RTS_IEC_LREAL* jointVoltages,
	RTS_IEC_BOOL* constraintActive,						// need to re-plan the motion at rising edge;
	RTS_IEC_LREAL* jointVelFactorConstraint,					// constraint factor for joint velocities;
	RTS_IEC_LREAL* jointAccFactorConstraint,					// constraint factor for joint accelerations;
	RTS_IEC_LREAL* electircPower,						// P = U * I;
	RTS_IEC_LREAL* physicsPower,							// P = Torque * omega;
	RTS_IEC_LREAL* momentum								// momemtum = mass * omega;
);

extern void getGravityTorqueForFlexibleCompensate
(
	RTS_IEC_LREAL* jointPosition,
	RTS_IEC_LREAL* gravTorque
);


// 获取已经设置的参数，用于对比验证；
extern RTS_IEC_BOOL getSettingParams
(
	cobotParams* params
);

// 更新力传感器数据，是否使用末端力传感器辅助碰撞
extern void updateForceSensorData(
	RTS_IEC_BOOL enable,				/* VAR_INPUT */	/* 是否开启末端力传感器的碰撞检测功能； */
	RTS_IEC_LREAL calibedForce[6]		/* VAR_INPUT */	/* 获取标定后的力传感器数据，当前为：GetFTValueInSelectedFrame */
);

// 将末端力传感器的受力引入到关节电流前馈中
extern void updateForceSensorForFoceControl(
	RTS_IEC_BOOL enable,				/* VAR_INPUT */	/* 是否开启末端力传感器的对关节电流前馈的作用； */
	RTS_IEC_LREAL calibedForce[6]		/* VAR_INPUT */	/* 获取标定后的力传感器数据，工具坐标系 */
);

// 更新双编位置给控制器（辅助零力示教）
extern void updateDualEncoderJointPosition(
	RTS_IEC_LREAL motorSideJointPosition[6],	/* VAR_INPUT */	/* actual motor side  joint position(deg) */
	RTS_IEC_LREAL jointSideJointPosition[6]	/* VAR_INPUT */	/* actual joint side joint position(deg) */
);

// 是否开启双编辅助零力示教的功能；
extern void setAssistDualEncoderFlag(
	RTS_IEC_BOOL enable
);

// 设置双编辅助零力示教的阈值
extern void setDualDiffEncoderThd(RTS_IEC_LREAL thd[6]);