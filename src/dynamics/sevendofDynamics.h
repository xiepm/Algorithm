#pragma once
#include "hansTypes.h"
#include "dynamicsBase.h"

using namespace KDL;

/// @brief 七轴机器人动力学模型脚手架。
///
/// 这个类是给新机型 `sevendofDynamics` 预留的正式接入点。
/// 当前版本先把工程结构接通，让上层算法能够按机型选择到这里；
/// 但完整动力学公式（逆动力学、动量观测用力矩）还需要你后续补入。
///
/// 对接步骤建议：
/// 1. 明确七轴 DH/连杆参数的顺序；
/// 2. 把辨识得到的动力学公式填进 `calculateEstimateJointToqrues()`；
/// 3. 再补 `calculateMomentumEstimatedJointTorques()` 给动量观测器使用；
/// 4. 最后联调总控层和对外接口。
///
/// 当前类的行为：
/// - 能保存 7 轴几何参数和重力；
/// - 重力/完整力矩/动量力矩接口都会先返回 0 向量；
/// - `calculateEstimateJointToqrues()` 会返回 `false`，提醒上层“公式还没实现”。
class sevendofDynamics : public dynamicsBase
{
public:
	sevendofDynamics();
	virtual ~sevendofDynamics();

	/// @brief 设置七轴机器人的几何参数。
	///
	/// 当前先约定为 7 个长度参数占位，后续你可以按真实机型改成：
	/// - 标准 DH: [d1, d2, ..., d7, a1, ...]
	/// - 或项目里常用的等效参数顺序
	///
	/// 注意：
	/// - 当前只做“保存参数”，还没有参与公式计算；
	/// - 调用方至少传入 7 个元素更稳妥。
	virtual void setRobotDHParameters
	(
		const EcRealVector& kinematcisParam
	);

	virtual void setGravityVector
	(
		const EcReal gx, const EcReal gy, const EcReal gz
	);

	virtual void calculateGravityJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& parms,
		EcRealVector& tau
	);

	/// @brief 计算七轴机器人完整补偿力矩。
	///
	/// 当前仅做输入尺寸检查，并返回 false。
	/// 等你把七轴动力学公式补进来之后，这里就是主入口。
	virtual EcBoolean calculateEstimateJointToqrues
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	);

	/// @brief 计算给动量观测器使用的力矩。
	///
	/// 当前先返回 7 维零向量，避免未实现时出现野值。
	virtual void calculateMomentumEstimatedJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	);

private:
	EcSizeT     m_NumJoints;
	EcRealVector m_kinematicParams;
	EcReal      m_gx;
	EcReal      m_gy;
	EcReal      m_gz;
	KDL::Vector m_gravity;
	EcRealVector m_zeros;
};
