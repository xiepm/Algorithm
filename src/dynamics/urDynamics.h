#pragma once
#include "hansTypes.h"
#include "dynamicsBase.h"


using namespace KDL;

/// @brief UR 系列六轴机器人动力学模型。
///
/// 它和 `elfinDynamics` 一样，实现了统一的 `dynamicsBase` 接口，
/// 但内部使用的是 UR 结构对应的几何参数和动力学公式。
///
/// 最小使用示例：
/// @code
/// urDynamics dyn;
/// dyn.setRobotDHParameters({d1, d2, d3, d4, d5, d6, a2, a3});
/// dyn.setGravityVector(0.0, 0.0, -9.81);
///
/// EcRealVector tau(6, 0.0);
/// dyn.calculateEstimateJointToqrues(q, dq, ddq, parms, tau);
/// @endcode
class urDynamics : public dynamicsBase
{
public:
	/// @brief 构造函数，填入默认参数。
	urDynamics();
	virtual ~urDynamics();

	/// @brief 设置 UR 机器人的几何参数。
	///
	/// 参数顺序为：
	/// 1. `d1`
	/// 2. `d2`
	/// 3. `d3`
	/// 4. `d4`
	/// 5. `d5`
	/// 6. `d6`
	/// 7. `a2`
	/// 8. `a3`
	///
	/// 注意：
	/// - 当前实现内部会把 `a2/a3` 取负保存
	/// - 外部调用时传正常几何值即可，不要自己先取负
	virtual void setRobotDHParameters
	(
		const EcRealVector& kinematcisParam
	);

	/// @brief 设置重力向量分量。
	virtual void setGravityVector
	(
		const EcReal gx, const EcReal gy, const EcReal gz
	);


	/// @brief 计算 UR 在当前姿态下的重力力矩。
	virtual void calculateGravityJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& parms,
		EcRealVector& tau
	);


	/// @brief 计算 UR 的完整关节估计力矩。
	///
	/// 返回 `false` 时，通常说明 `q/dq/ddq` 长度不是 6。
	virtual EcBoolean calculateEstimateJointToqrues
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	);


	/// @brief 计算给动量观测器使用的简化力矩。
	///
	/// 一般不含重力、摩擦、电机惯量等项，更适合观测外界扰动。
	virtual void calculateMomentumEstimatedJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	);





private:
	/// 机器人关节数，UR 固定为 6。
	EcSizeT									m_NumJoints;
	/// UR 几何参数。
	EcReal									m_d1, m_d2, m_d3, m_d4, m_d5, m_d6, m_a2, m_a3;

	/// 重力向量分量。
	EcReal									m_gx;
	EcReal									m_gy;
	EcReal									m_gz;
	KDL::Vector								m_gravity;

	/// 温度摩擦补偿系数。
	EcRealVector							m_friTemperaturesParams;
	/// 当前关节温度。
	EcRealVector							m_jointTemperatures;

	/// 全零向量。
	EcRealVector							m_zeros;

};
