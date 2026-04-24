#pragma once
#include "hansTypes.h"
#include <memory>


using namespace KDL;

/// @brief 动力学抽象基类。
///
/// 这个类只定义“动力学模型应该提供哪些能力”，真正的实现由
/// `elfinDynamics`、`urDynamics` 等派生类完成。
///
/// 对刚接手项目的人，可以先把它理解成：
/// 1. 根据机器人型号创建一个具体的动力学对象；
/// 2. 设置几何参数和重力方向；
/// 3. 输入关节位置/速度/加速度，输出估计力矩；
/// 4. 由上层算法统一调用，而不关心底层到底是哪种机器人。
///
/// 典型使用方式：
/// @code
/// dynBasePtr dyn = std::make_shared<elfinDynamics>();
/// dyn->setRobotDHParameters(kinematicParams);
/// dyn->setGravityVector(0.0, 0.0, -9.81);
///
/// EcRealVector tau(6, 0.0);
/// EcBoolean ok = dyn->calculateEstimateJointToqrues(q, dq, ddq, parms, tau);
/// if (!ok)
/// {
///     // 一般说明输入维度不正确
/// }
/// @endcode
class dynamicsBase
{
public:

	/// @brief 设置机器人几何参数（DH/等效连杆参数）。
	///
	/// 不同机器人型号对 `kinematcisParam` 的长度和顺序要求不同：
	/// - `elfinDynamics` 需要 4 个参数
	/// - `urDynamics` 需要 8 个参数
	///
	/// 注意事项：
	/// - 这个接口本身不做长度约束，参数解释由派生类决定
	/// - 新人使用前，先看具体派生类头文件里的参数说明
	virtual void setRobotDHParameters
	(
		const EcRealVector& kinematcisParam
	) = 0;

	/// @brief 设置重力向量分量。
	///
	/// 默认通常是 `(0, 0, -9.81)`。
	/// 如果机器人安装方向变化，或者世界坐标系定义不同，需要重新设置。
	virtual void setGravityVector
	(
		const EcReal gx, const EcReal gy, const EcReal gz
	) = 0;


	/// @brief 仅计算重力补偿力矩。
	///
	/// 典型用途：
	/// - 静态姿态下的重力补偿
	/// - 调试不同姿态带来的负载变化
	///
	/// 使用示例：
	/// @code
	/// EcRealVector tau(6, 0.0);
	/// dyn->calculateGravityJointTorques(q, parms, tau);
	/// @endcode
	virtual void calculateGravityJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& parms,
		EcRealVector& tau
	) = 0;


	/// @brief 计算完整的关节估计力矩。
	///
	/// 名字虽然历史上叫 `EstimateJointToqrues`，但从用途上看更接近
	/// “逆动力学力矩估计”：输入 `q/dq/ddq`，输出关节力矩 `tau`。
	///
	/// 一般会包含：
	/// - 惯性项
	/// - 科氏力/离心力项
	/// - 重力项
	/// - 摩擦项
	/// - 某些实现中的温度修正项
	///
	/// 注意事项：
	/// - `q/dq/ddq` 必须是正确维度
	/// - `tau` 最好由调用方预分配
	/// - `parms` 的含义由具体建模方式决定，通常来自参数辨识
	virtual EcBoolean calculateEstimateJointToqrues
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	) = 0;


	/// @brief 计算给动量观测器使用的估计力矩。
	///
	/// 与完整力矩相比，通常会排除：
	/// - 重力项
	/// - 摩擦项
	/// - 电机惯量项
	///
	/// 常见用途：
	/// - 扰动力矩估计
	/// - 碰撞检测
	/// - 动量观测器内部模型
	virtual void calculateMomentumEstimatedJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	) = 0;

	/// @brief 带死区的符号函数。
	///
	/// 当速度非常接近 0 时，直接返回 0，避免摩擦项发生数值抖动。
	///
	/// 示例：
	/// @code
	/// sign( 0.5)   ->  1
	/// sign(-0.5)   -> -1
	/// sign( 1e-4)  ->  0
	/// @endcode
	EcReal sign(const EcReal x) {
		if (fabs(x) < 0.001)
		{
			return 0;
		}

		if (x > 0)
			return 1.0;
		else
			return -1.0;
	}


private:


};

typedef				std::shared_ptr<dynamicsBase>				dynBasePtr;
