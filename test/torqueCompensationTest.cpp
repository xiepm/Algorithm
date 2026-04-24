#include <cstdlib>
#include <iostream>
#include <string>

#include "src/dynamics/dynamicsBase.h"
#include "src/dynamics/elfinDynamics.h"
#include "src/dynamics/urDynamics.h"

namespace
{
void printVector(const std::string& name, const EcRealVector& values)
{
	std::cout << name << " = [";
	for (EcU32 i = 0; i < values.size(); ++i)
	{
		if (i != 0)
		{
			std::cout << ", ";
		}
		std::cout << values[i];
	}
	std::cout << "]" << std::endl;
}

std::shared_ptr<dynamicsBase> createDynamics(const std::string& robotName)
{
	if (robotName == "elfin")
	{
		return std::shared_ptr<dynamicsBase>(new elfinDynamics());
	}

	return std::shared_ptr<dynamicsBase>(new urDynamics());
}
}

int main(int argc, char* argv[])
{
	// 这个测试程序的目的很单一：
	// 给定关节位置 q、速度 dq、加速度 ddq，直接调用动力学类计算补偿力矩 tau。
	//
	// 注意：
	// 1. 这里的角度单位是弧度 rad，不是度。
	// 2. dynamics 参数如果全填 1.0，只能验证“接口是否通、结果是否能算出来”，
	//    不能代表真实机器人的物理结果。
	// 3. 想得到可信的补偿力矩，请换成你们标定后的 78 个动力学参数。
	// 4. 运行方式：
	//    ./build/torqueCompensationTest
	//    ./build/torqueCompensationTest ur
	//    ./build/torqueCompensationTest elfin

	const std::string robotName = (argc > 1) ? argv[1] : "ur";
	std::shared_ptr<dynamicsBase> dynamics = createDynamics(robotName);

	// 设置机器人 DH 参数。
	// Elfin: [d1, d4, d6, a2]
	// UR:    [d1, d4, d5, d6, a2, a3]
	EcRealVector kinParams;
	if (robotName == "elfin")
	{
		kinParams = { 0.22, 0.42, 0.18, 0.38 };
	}
	else
	{
		kinParams = { 0.22, 0.42, 0.18, 0.12, 0.38, 0.30 };
	}
	dynamics->setRobotDHParameters(kinParams);
	dynamics->setGravityVector(0.0, 0.0, -9.81);

	// 设置动力学线性参数。
	// 实际工程里应当替换成真实标定值；这里用 1.0 只是为了把测试流程跑通。
	EcRealVector dynParams(78, 1.0);

	// 待测试输入：关节位置、速度、加速度
	EcRealVector q   = { 0.10, -0.35, 0.45, -0.20, 0.15, 0.05 };
	EcRealVector dq  = { 0.20, -0.10, 0.15, 0.00, 0.05, -0.02 };
	EcRealVector ddq = { 0.50,  0.20, -0.10, 0.08, 0.03,  0.01 };

	EcRealVector tau(6, 0.0);
	EcBoolean ok = dynamics->calculateEstimateJointToqrues(q, dq, ddq, dynParams, tau);

	if (!ok)
	{
		std::cerr << "calculateEstimateJointToqrues failed." << std::endl;
		return 1;
	}

	std::cout << "robot = " << robotName << std::endl;
	printVector("q(rad)", q);
	printVector("dq(rad/s)", dq);
	printVector("ddq(rad/s^2)", ddq);
	printVector("tau(Nm)", tau);

	return 0;
}
