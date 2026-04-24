#pragma once
#include "hansTypes.h"
#include "dynamicsBase.h"


using namespace KDL;

/// @brief Elfin 六轴协作机器人动力学模型。
///
/// 这个类封装了 Elfin 机器人的动力学计算，包括：
/// - 重力力矩计算
/// - 完整关节估计力矩计算
/// - 给动量观测器使用的简化力矩计算
///
/// 对新人来说，可以把它理解成“输入关节状态，输出关节力矩”的工具类。
///
/// 最小使用示例：
/// @code
/// elfinDynamics dyn;
/// dyn.setRobotDHParameters({0.22, 0.42, 0.18, 0.38});
/// dyn.setGravityVector(0.0, 0.0, -9.81);
///
/// EcRealVector tau(6, 0.0);
/// EcBoolean ok = dyn.calculateEstimateJointToqrues(q, dq, ddq, parms, tau);
/// @endcode
///
/// 注意：
/// - `q/dq/ddq/tau` 一般都应为 6 维
/// - `parms` 需要与辨识参数维度匹配
class elfinDynamics : public dynamicsBase
{
public:
	/// @brief 构造函数，填入默认几何参数、重力和温度相关初值。
	elfinDynamics();
	virtual ~elfinDynamics();

	/// @brief 设置 Elfin 的几何参数。
	///
	/// 参数顺序固定为：
	/// 1. `d1`
	/// 2. `d4`
	/// 3. `d6`
	/// 4. `a2`
	///
	/// 使用示例：
	/// @code
	/// dyn.setRobotDHParameters({0.22, 0.42, 0.18, 0.38});
	/// @endcode
	///
	/// 注意：
	/// - 调用方必须保证传入至少 4 个元素
	/// - 这里没有额外做越界保护
	virtual void setRobotDHParameters
	(
		const EcRealVector& kinematcisParam
	);

	/// @brief 设置重力分量。
	///
	/// 示例：
	/// @code
	/// dyn.setGravityVector(0.0, 0.0, -9.81);
	/// @endcode
	virtual void setGravityVector
	(
		const EcReal gx, const EcReal gy, const EcReal gz
	);


	/// @brief 计算 Elfin 当前姿态下的重力补偿力矩。
	///
	/// 它内部会把速度和加速度都视为 0，只保留姿态相关的重力项。
	virtual void calculateGravityJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& parms,
		EcRealVector& tau
	);


	/// @brief 计算 Elfin 的完整关节估计力矩。
	///
	/// 输入：
	/// - `q`   : 6 维关节角
	/// - `dq`  : 6 维关节角速度
	/// - `ddq` : 6 维关节角加速度
	/// - `parms` : 动力学辨识参数
	///
	/// 输出：
	/// - `tau` : 6 维关节力矩
	///
	/// 返回值：
	/// - `true`  : 计算完成
	/// - `false` : `q/dq/ddq` 维度错误
	///
	/// 注意：
	/// - `tau` 最好预先分配为 6 维
	/// - `parms` 需要有足够长度
	/// - 函数名里的 `Toqrues` 是历史拼写问题
	virtual EcBoolean calculateEstimateJointToqrues
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	);


	/// @brief 计算给动量观测器使用的估计力矩。
	///
	/// 与完整模型不同，这里会排除：
	/// - 重力项
	/// - 摩擦项
	/// - 电机惯量项
	///
	/// 适合做：
	/// - 扰动力矩估计
	/// - 碰撞检测
	/// - 动量观测器内部建模
	virtual void calculateMomentumEstimatedJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	);





private:
	/// 机器人关节数，Elfin 固定为 6。
	EcSizeT									m_NumJoints;
	/// Elfin 几何参数。
	EcReal									m_d1, m_d4, m_d6, m_a2;

	/// 重力向量分量。
	EcReal									m_gx;
	EcReal									m_gy;
	EcReal									m_gz;
	KDL::Vector								m_gravity;

	/// 温度对摩擦补偿的放大系数。
	EcRealVector							m_friTemperaturesParams;
	/// 当前关节温度。
	EcRealVector							m_jointTemperatures;

	/// 全零向量，常用于只计算重力项时复用。
	EcRealVector							m_zeros;

};
