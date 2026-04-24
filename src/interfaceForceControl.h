#pragma once
#include "frames.hpp"
#include "typeDef.h"
/*
//		对接北大刘吴月的力控产品接口
*/


class interfaceForceControl
{
public:
	// 初始化力控接口
	EcBoolean initialize(const EcRealVector& commandEEPosition, const EcReal timeCycle);

	// 选择力控的自由度，1为被选择，0是未选择；
	// 输入： selectedDOF(8)
	EcBoolean setControlFreedom(const EcU32Vector& selectedDOF);

	// 力控坐标系的选择
	// 输入： 
	//	frameType:坐标系类型，
	//				1为工具坐标系，2为用户坐标系；
	// 输出：
	//	userPoseMatrix:用户坐标系旋转矩阵向量
	EcBoolean setCoordinateSystem(const EcU32 frameType, const EcRealVector& userCoord, EcRealVector& userPoseMatrix);

	// 更新机器人的运动状态，并得到旋转矩阵（9*1）：
	// 输入：
	//		actualEEPosition: 机器人TCP末端的实际位置；
	//		commandEEPosition：机器人TCP末端的命令位置；
	// 输出：
	//		actualEEPoseMatrix(9): 机器人TCP末端旋转矩阵向量；
	EcBoolean updateRobotState(const EcRealVector& actualEEPosition, const EcRealVector& commandEEPosition, EcRealVector& actualEEPoseMatrix);

	// 更新力控模块计算的速度，返回力控位置命令
	// 输入：
	//		forceControlVel:力控模块的速度；
	// 输出：
	//		forceCommandEEPosition: 发送给关节的力控混合位置；
	EcBoolean calculateCommandForcePosition(const EcRealVector& forceControlVel, EcRealVector& forceCommandEEPosition);


	// 获取机器人的命令速度
	// 输出： commandVel, 在设定坐标系下的
	void getCommandVelocity(EcRealVector& commandVel);



private:

	EcReal				m_timeCycle;
	EcBoolean			b_isToolCoordinate;
	KDL::Frame			m_preCommandToolPosition;
	KDL::Frame			m_commandToolPosition;

	KDL::Frame			m_updateToolPosition;

	KDL::Frame			m_userRotation;
	KDL::Frame			m_toolFrame;
	KDL::Frame			m_preUpdateToolPosition;

	KDL::Frame			m_updateEEPosition;
	EcU32Vector			m_selectedDOF;

	EcU32				m_FrameType;
	EcRealVector		m_commandVel;
};