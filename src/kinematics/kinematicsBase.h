#pragma once
#include "hansTypes.h"
#include <memory>


using namespace KDL;
using namespace Eigen;

const double            EPSILON1 = 1.0e-6;
const double            EPSILON2 = 1.0e-12;
const double            INFINITE = 9.0e9;  // 类比ABB的充分大值

class kinematicsBase
{
public:
	kinematicsBase();

	virtual void setRobotDHParameters
	(
		const EcRealVector& kinematcisParam
	) = 0;

	virtual void setJointMotionLimit
	(
		const EcRealVector& upperJointLimit,
		const EcRealVector& lowerJointLimit
	) = 0;

	virtual void setToolCoordinateSystem
	(
		const EcFrame& tool
	) = 0;

	virtual EcFrame forwardKinematics
	(
		const EcRealVector& jointPositions
	) = 0;

	virtual void forwardKinematics2
	(
		const EcRealVector& jointPositions,
		EcFrame& fkFrame
	) = 0;

	virtual int getRobotType() = 0;


	virtual ENInverseKineState inverseKinematics
	(
		const EcFrame& target,
		const EcRealVector& refJoint,
		EcRealVector& acs
	) = 0;

	// 根据臂型选择逆解; refJoint作用: 指定臂型，且奇异时提供q1/q4的参考值
	virtual ENInverseKineState inverseKinematicsWithConfigHolding(const Frame& targetPose, const EcRealVector& refJoint, EcRealVector& targetJoint) { return ikState_normal; }


	//@brief 设置机械臂DH模型的误差辨识结果
	virtual void setIdentityErrorOfDHParams(const DHParams& errorDHParams) { return; };

	//@brief 设置减速器的刚度特性
	virtual void setReducerParams(const EcRealVectorVector& reducerParamsOfJoints, const EcReal& rigidityRatio) { return; };

	// @brief 设置全参补偿策略
	virtual void setCompensationStrategy(const int& strategy = 0) { return; };

	//更新关节力矩
	virtual void updateFlexibilityError(const EcRealVector& torque) { return; };


	virtual EcReal getJacobianMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian) = 0;


	// 计算Tool Point的Jacobian；
	virtual void getJacobianWithToolPointMatrix(const VectorXd& jointPosition, Eigen::MatrixXd& Jacobian) = 0;

	// 计算Tool Point的JacobianDot；
	virtual void getJacobianDotWithToolMatrix(const VectorXd& jointPosition, const VectorXd& jointVel, Eigen::MatrixXd& JacobianDot) = 0;

	virtual EcReal calcJacobiandeterminant(const EcRealVector& jointPosition) {
		EcRealMatrixX Jacobian(6, 6);
		getJacobianMatrix(jointPosition, Jacobian);
		return Jacobian.determinant();
	};

	virtual EcReal getJacobianConditionNum(const EcRealVector& jointPosition) {
		EcRealMatrixX Jacobian(6, 6);
		getJacobianMatrix(jointPosition, Jacobian);
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jacobian);
		EcReal minSingularValue = svd.singularValues()(svd.singularValues().size() - 1);
		return  (minSingularValue == 0) ? INFINITE : (svd.singularValues()(0) / minSingularValue);
	};

	EcBoolean checkSingularity(const EcRealVector& jointPosition);

	EcReal calcJacobianJointsVelocity(EcRealVector jointPosition, EcRealVector EndEffectorVelocity, EcRealVector& jointsVelocity);
	void calcJacobianEndEffectorVelocity(const EcRealVector& jointPosition, const EcRealVector& jointVelocity, EcRealVector& endEffectorVelocity);

	// 基于末端受力，计算关节力矩
	void calcJacobianJointTorque(const EcRealVector& jointPosition, const EcRealVector& endEffectorForce, EcRealVector& jointTorque);



	// 计算笛卡尔空间速度、加速度、跃度和关节速度、加速度、跃度的关系；
	void calcSpatialVel2JointVel(const VectorXd& jointPosition, const VectorXd& spatialVel, VectorXd& jointVel);
	void calcSpatialAcc2JointAcc(const VectorXd& jointPosition, const VectorXd& spatialVel, const VectorXd& spatialAcc, VectorXd& jointVel, VectorXd& jointAcc);


	virtual void calcSpatialJerk2JointJerk(const EcReal deltaT, const VectorXd& jointPosition, const VectorXd& spatialVel, const VectorXd& spatialAcc, const VectorXd& spatialJerk, VectorXd& jointVel, VectorXd& jointAcc, VectorXd& jointJerk)
	{
		MatrixXd J(NUMOFJOINTS6, NUMOFJOINTS6), Jdot(NUMOFJOINTS6, NUMOFJOINTS6), Jdotdot(NUMOFJOINTS6, NUMOFJOINTS6);
		getJacobianWithToolPointMatrix(jointPosition, J);
		MatrixXd Jinv = J.inverse();
		jointVel = Jinv * spatialVel;

		getJacobianDotWithToolMatrix(jointPosition, jointVel, Jdot);
		jointAcc = Jinv * (spatialAcc - Jdot * jointVel);

		// 如果是样条曲线的控制参数u，那么 dt = |p'|du / v(u)
		Jdotdot = (Jdot - m_preJdot) / deltaT;				// 在第一次计算周期，m_preJdot是不准确的，可能会带来很大的误差；

		jointJerk = Jinv * (spatialJerk - Jdotdot * jointVel - 2 * Jdot * jointAcc);


		//EcReal diff = Jdotdot.sum();
		m_preJdot = Jdot;
	}

	void calcJointVel2SpatialVel(const VectorXd& jointPosition, const VectorXd& jointVel, VectorXd& spatialVel);
	void calcJointAcc2SpatialAcc(const VectorXd& jointPosition, const VectorXd& jointVel, const VectorXd& jointAcc, VectorXd& spatialVel, VectorXd& spatialAcc);
	void calcJointJerk2SpatialJerk(const EcReal deltaT, const VectorXd& jointPosition, const VectorXd& jointVel, const VectorXd& jointAcc, const VectorXd& jointJerk, VectorXd& spatialVel, VectorXd& spatialAcc, VectorXd& spatialJerk);

	/**
	 * @brief 计算雅克比矩阵的伪逆
	 *
	 * @param jacobianMatrix 雅克比矩阵
	 * @param tolerance 容差值，默认为1e-6
	 * @return 伪逆矩阵
	 */
	Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& Jacobian, double tolerance = 1e-6)
	{
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);  // 使用Full得到6×6的矩阵V，使用Thin得到6×3的矩阵V
		Eigen::MatrixXd singularValuesInv = Eigen::MatrixXd::Zero(Jacobian.cols(), Jacobian.rows());
		for (int i = 0; i < min(Jacobian.rows(), Jacobian.cols()); ++i)
		{
			singularValuesInv(i, i) = (svd.singularValues()(i) > tolerance) ? (1.0 / svd.singularValues()(i)) : 0.0;  // 奇异值小于tolerance时置为零
		}
		return svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
	}

private:
	Eigen::MatrixXd			m_jacobian;
	Eigen::MatrixXd			m_preJdot;				// 保存前一周期的Jdot，用于查分计算Jdotdot;
};

typedef				std::shared_ptr<kinematicsBase>				kinBasePtr;
