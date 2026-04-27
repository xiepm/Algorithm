#pragma once
#include "hansTypes.h"
#include <memory>


using namespace KDL;

// 两个配置文件：dh， gravity
class dynamicsBase
{
public:

	virtual void setRobotDHParameters
	(
		const EcRealVector& kinematcisParam
	) = 0;

	virtual void setGravityVector
	(
		const EcReal gx, const EcReal gy, const EcReal gz
	) = 0;


	virtual void calculateGravityJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& parms,
		EcRealVector& tau
	) = 0;


	// calculate forward dynamics of elfin
	virtual EcBoolean calculateEstimateJointToqrues
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	) = 0;

	virtual void calculateJointFricition(const EcRealVector& dq, EcReal coeffColomb, EcReal coeffViscous, EcRealVector& tau)
	{
		return;
	}


	// exclude gravity, friction, motor inertia;
	virtual void calculateMomentumEstimatedJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	) = 0;

	static EcReal sign(const EcReal x) {
		if (fabs(x) < 0.0113)
		{
			return x / 0.0113;
		}

		if (x > 0)
			return 1.0;
		else
			return -1.0;
	}


private:


};

typedef				std::shared_ptr<dynamicsBase>				dynBasePtr;
