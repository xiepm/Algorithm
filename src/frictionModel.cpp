#include "frictionModel.h"
#include <iostream>


void predictiveStartTorque::initialize(EcReal timePeriod)
{
	m_enterCount = 0;
	m_updateTimePeriod = timePeriod;
	m_currentTime = 1.0;
	m_predictiveCompensateTorque = 0.0;
	m_OldSensedTorques.assign(buffSize, 0.0);
}

// 如果进入补偿时间，则不应该对参数进行修改；
// 当力矩处于换向的临界点，力矩达到最大时，将此时的力矩更新给这个接口；
void predictiveStartTorque::updateTorques(int ii, const EcReal sensedTorque)
{
	// 达成一次预测补偿后，会把m_currentTime时间清零，让生成的预测补偿力矩维持compensateLastTime 时间；
	if (m_currentTime < compensateLastTime)
	{
		m_enterCount = 0;
		return;
	}

	if (m_enterCount <= buffSize)
	{
		m_predictiveCompensateTorque = 0.0;
		m_enterCount++;
	}
	else
	{
		m_averageTorque = 0;
		for (EcU32 ii = 0; ii < buffSize; ii++)
		{
			m_averageTorque += m_OldSensedTorques[ii];
		}
		m_averageTorque /= buffSize;

		m_amplitudeTorque = 0.0;
		for (EcU32 ii = 0; ii < buffSize; ii++)
		{
			m_amplitudeTorque += fabs(m_OldSensedTorques[ii] - m_averageTorque);
		}
		m_amplitudeTorque /= buffSize * 1.0;

		// 给予补偿力矩的情况1：峰值检测力矩小于0.7倍的幅值； 与sensedTorque方向相同；
		if (fabs(sensedTorque - m_averageTorque) < 0.5 * m_amplitudeTorque)
		{
			m_currentTime = 0.0;								// 当前时间为零，则进入补偿通道；
			m_predictiveCompensateTorque = m_averageTorque + 1.0 * KDL::sign(sensedTorque - m_averageTorque) * m_amplitudeTorque;
		}

		// 给予补偿力矩的情况2： 峰值检测力矩大于1.3倍的幅值； 与sensedTorque方向相反；
		if (fabs(sensedTorque - m_averageTorque) > 1.6 * m_amplitudeTorque)
		{
			m_currentTime = 0.0;								// 当前时间为零，则进入补偿通道；
			// 补偿力矩的计算：检测力矩的反方向，反向力矩为偏离平均力矩约0.75倍的幅值，持续时间为compensateLastTime = 0.05s；
			//风险：有可能力矩补偿不对，阻碍了运动；
			m_predictiveCompensateTorque = m_averageTorque - 1.0 * KDL::sign(sensedTorque - m_averageTorque) * m_amplitudeTorque;

		}
	}

	m_OldSensedTorques.push_front(sensedTorque);

}

EcBoolean predictiveStartTorque::getPredictiveCompensateTorque(EcReal& compensateTorque)
{
	// 如果触发预测补偿力矩，m_currentTime会被置零。
	m_currentTime += m_updateTimePeriod;
	if (m_currentTime < compensateLastTime && m_enterCount > buffSize)
	{
		//std::cout << "predictive torque:" << m_currentTime << "," << m_predictiveCompensateTorque << std::endl;;
		compensateTorque = m_predictiveCompensateTorque;
		return EcTrue;
	}
	else
	{
		m_predictiveCompensateTorque = 0;
		return false;
	}
}


// 给予补偿力矩的情况2： 实时检测力矩是否大于1.3倍的幅值；若是，则反向提供预测力矩；
void predictiveStartTorque::checkSensedTorque(const EcReal& sensedTorque)
{
	if (m_enterCount < buffSize)
	{
		return;
	}
	else
	{
		if (fabs(sensedTorque - m_averageTorque) > 1.5 * m_amplitudeTorque)
		{
			/**/
			std::cout << sensedTorque << "," << m_averageTorque << "," << m_amplitudeTorque << "," << "bufferData:" << m_enterCount << ",";
			for (EcU32 ii = 0; ii < buffSize; ii++)
			{
				std::cout << fabs(m_OldSensedTorques[ii] - m_averageTorque) << ",";
			}
			std::cout << std::endl;



			m_currentTime = 0.0;								// 当前时间为零，则进入补偿通道；
			m_predictiveCompensateTorque = m_averageTorque - 1.0 * KDL::sign(sensedTorque - m_averageTorque) * m_amplitudeTorque;
		}
	}
}


void predictiveStartTorque::reset()
{
	m_currentTime = 1.0;		// 重置时，让m_currentTime>compensateLastTime，避免进入补偿通道；
	m_predictiveCompensateTorque = 0.0;
	m_enterCount = 0;
}


frictionModel::frictionModel()
{

}

void frictionModel::initialize(const EcRealVector& currentJointPosition, const EcReal updateTimePeriod)
{
	m_updateTimePeriod = updateTimePeriod;
	m_currentTime = 0.0;
	m_NumJoints = currentJointPosition.size();
	m_stopGenerateFrictTime = 2 * 60.0;				// 2 min;
	//m_stopGenerateFrictTime = 0.0;				// 2 min;

	m_coulombFricton.assign(m_NumJoints, 0.0);
	m_viscousFrictionCoeff.assign(m_NumJoints, 0.0);
	m_calcCoulombFriction.assign(m_NumJoints, 0.0);
	m_calcViscousFriction.assign(m_NumJoints, 0.0);
	m_compensateConfigFactor.assign(m_NumJoints, 0.0);
	m_dynCompensateFactor.assign(m_NumJoints, 1.0);
	m_dynViscousCompensateFactor.assign(m_NumJoints, 1.0);
	m_constraintDynCompensateFactor.assign(m_NumJoints, 0.5);

	m_maxJointVelocities.assign(m_NumJoints, 0.0);



	// 生成抖动力矩
	b_activeDitherStatus.assign(m_NumJoints, true);
	m_ditherStartPosition.assign(m_NumJoints, 0.0);
	m_ditherPositionErr.assign(m_NumJoints, 0.0);
	m_ditherPrePositionErr.assign(m_NumJoints, 0.0);

	m_deltaFriction.assign(m_NumJoints, 0.0);
	m_ditherDirection.assign(m_NumJoints, 1.0);

	m_dynConstraintTime.assign(m_NumJoints, 0.0);
	m_jointMeanVel.assign(m_NumJoints, 0.0);
	m_jointPreMeanVel.assign(m_NumJoints, 0.0);

	m_compensateStartFrictionFactor.assign(m_NumJoints, 0.2);

	// 预判启动
	predictiveStartTorque temp;
	m_predictiveTorque.resize(numofJoints);
	for (size_t i = 0; i < numofJoints; i++)
	{
		m_predictiveTorque[i].initialize(updateTimePeriod);
	}
}


void frictionModel::setFrictionParams
(
	const EcRealVector& coulombFriction,
	const EcRealVector& viscousFrictionCoeff
)
{
	m_coulombFricton = coulombFriction;
	m_viscousFrictionCoeff = viscousFrictionCoeff;


	for (int ii = 0; ii < m_NumJoints; ii++)
	{
		m_deltaFriction[ii] = m_coulombFricton[ii] * m_updateTimePeriod / risingTime;
	}
}

void frictionModel::setCompensateFactor
(
	const EcRealVector& compensateFactor
)
{
	m_compensateConfigFactor = compensateFactor;
	for (int ii = 0; ii < m_NumJoints; ii++)
	{
		if (compensateFactor[ii] < 0.01)
		{
			m_compensateConfigFactor[ii] = 0.01;			// 避免出现为零的参数
		}
		if (compensateFactor[ii] > 1.0)
		{
			m_constraintDynCompensateFactor[ii] = 0.5 / compensateFactor[ii];
		}
	}
}

void frictionModel::setStartCompensateFrictionFactor(
	const EcRealVector& compensateFactorII)
{
	m_compensateStartFrictionFactor = compensateFactorII;
	for (int ii = 0; ii < m_NumJoints; ii++)
	{
		if (m_compensateStartFrictionFactor[ii] < 0.01)
		{
			m_compensateStartFrictionFactor[ii] = 0.01;			// 避免出现为零的参数
		}
		if (m_compensateStartFrictionFactor[ii] > 2.0)
		{
			m_constraintDynCompensateFactor[ii] = 2.0;
		}
		m_compensateStartFrictionFactor[ii] *= 0.3;
	}
}

void frictionModel::setMaxJointVelocitiesInAssistiveMode
(
	const EcRealVector& maxJointVelocities
)
{
	m_maxJointVelocities = maxJointVelocities;
}

void frictionModel::calculateCompensateFriction
(
	const EcRealVector& currentJointPosition,
	const EcRealVector& currentJointVel,
	const EcRealVector& currentJointAcc,
	const EcRealVector& sensedTorque,
	const EcBoolean& driveBackMode,
	EcRealVector& compensateCoulombFriction,
	EcRealVector& compensateViscousFriction
)
{
	m_currentTime += m_updateTimePeriod;
	// 库伦摩擦力：
	//			1） 预滑动状态下，生成抖动信号，确保位置的变化不大于0.003°；
	//          2） 低速状态下（0-5°/s)，为什么低速下要降低补偿值？
	//			3） 高速状态下(5-60°/s)，
	EcReal sumVel = 0;
	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		sumVel += fabs(currentJointVel[ii]);
		if (fabs(currentJointVel[ii]) < zeroVelocityThresholds)			//1.7°			  预滑动状态；
		{
			if (m_stopStateTime >= m_stopGenerateFrictTime || driveBackMode)
			{
				m_calcCoulombFriction[ii] = 0;		// 处于碰撞反弹模式，或者持续抖动2min后，不提供库伦摩擦力；
			}
			else {
				generateDitherSignal(ii, currentJointPosition, currentJointVel, sensedTorque);
			}
		}
		else if (fabs(currentJointVel[ii]) < lowVelocityThresholds)							 // 低速隔离带，降低补偿阈值，避免抖动引发运动；
		{
			b_activeDitherStatus[ii] = true;
			m_calcCoulombFriction[ii] = m_compensateConfigFactor[ii] * 0.75 * pow(fabs(currentJointVel[ii]) / lowVelocityThresholds, 0.70) * KDL::sign(currentJointVel[ii]) * m_coulombFricton[ii];
			//std::cout << "dither(p,F,dynC,V,mV,acc)," << currentJointPosition[ii] << "," << m_calcCoulombFriction[ii] << "," << m_dynCompensateFactor[ii]
			//	<< "," << currentJointVel[ii] << "," << m_jointPreMeanVel[ii] << "," << currentJointAcc[ii] << std::endl;
		}
		else
		{																					// 高速状态；
			b_activeDitherStatus[ii] = true;
			m_calcCoulombFriction[ii] = m_compensateConfigFactor[ii] * m_dynCompensateFactor[ii] * KDL::sign(currentJointVel[ii]) * m_coulombFricton[ii];
			//std::cout << "dither(p,F,dynC,V,mV,acc)," << currentJointPosition[ii] << "," << m_calcCoulombFriction[ii] << "," << m_dynCompensateFactor[ii]
			//	<< "," << currentJointVel[ii] << "," << m_jointPreMeanVel[ii]<<","<<currentJointAcc[ii] << std::endl;
		}

		compensateViscousFriction[ii] = m_dynViscousCompensateFactor[ii] * currentJointVel[ii] * m_viscousFrictionCoeff[ii];
	}
	if (sumVel < zeroVelocityThresholds)
	{
		m_stopStateTime += m_updateTimePeriod;
	}
	else {
		m_stopStateTime = 0;
	}
	dynamicConstrainCompensateFriction(currentJointVel, currentJointAcc);

	compensateCoulombFriction = m_calcCoulombFriction;
}


void frictionModel::generateDitherSignal
(
	const EcSizeT& ii,
	const EcRealVector& currentJointPosition,
	const EcRealVector& currentJointVel,
	const EcRealVector& sensedTorques
)
{
	// 如果是首次进入静置状态，获取位置、力矩渐变方向；
	if (b_activeDitherStatus[ii])
	{
		m_ditherStartPosition[ii] = currentJointPosition[ii];
		m_ditherDirection[ii] = -KDL::sign(currentJointVel[ii]);			// 生成力矩的增长方向与速度相反；
		m_ditherPrePositionErr[ii] = KDL::sign(currentJointVel[ii]);		// 当前位置误差与速度方向相同；

		m_calcCoulombFriction[ii] = 0.0;
		b_activeDitherStatus[ii] = false;
		m_predictiveTorque[ii].reset();
	}
	else
	{
		m_ditherPositionErr[ii] = currentJointPosition[ii] - m_ditherStartPosition[ii];

		// 当抖动位置运动大于0.002°时，且当前位置偏差于上一次抖动生成的位置偏差方向相反时，才对抖动力矩增长方向取反；
		// 同时，若库伦力矩过大，也会对方向取反；
		if (
			(fabs(m_ditherPositionErr[ii]) > 0.00001
				&& m_ditherPositionErr[ii] * m_ditherPrePositionErr[ii] < 0)
			|| fabs(m_calcCoulombFriction[ii]) > m_compensateStartFrictionFactor[ii] * m_coulombFricton[ii]
			)
		{
			//std::cout << "index:" << ii << ",";
			m_predictiveTorque[ii].updateTorques(ii, m_calcCoulombFriction[ii]);
			m_calcCoulombFriction[ii] *= 0.25;
			m_ditherDirection[ii] *= -1.0;			// 当前位置偏差于前一轮的位置偏差方向相反时，才对力矩变化方向取反，确保力矩和位置的运动约束在阈值内；
			m_ditherPrePositionErr[ii] = m_ditherPositionErr[ii];
			m_ditherStartPosition[ii] = currentJointPosition[ii];
		}

		m_calcCoulombFriction[ii] += m_deltaFriction[ii] * m_ditherDirection[ii];

		EcReal predictiveCompensateTorque = 0;


		/*
		if (ii == 3 || ii == 5)
		{
			//m_predictiveTorque[ii].checkSensedTorque(m_calcCoulombFriction[ii]);
			if (m_predictiveTorque[ii].getPredictiveCompensateTorque(predictiveCompensateTorque))
			{
				//b_activeDitherStatus[ii] = true;
				m_calcCoulombFriction[ii] = predictiveCompensateTorque;
			}
		}
		*/


		/*
		if (ii == 0 || ii == 3 || ii == 5 || ii == 1 || ii == 2 || ii == 4)
		{
			std::cout << ii << ",";
			m_predictiveTorque[ii].checkSensedTorque(m_calcCoulombFriction[ii]);
			if (m_predictiveTorque[ii].getPredictiveCompensateTorque(predictiveCompensateTorque))
			{
				//b_activeDitherStatus[ii] = true;
				m_calcCoulombFriction[ii] = predictiveCompensateTorque;
			}
			std::cout << std::endl;
		}
		*/

	}



}


// 计算粘性摩擦力矩的动态补偿系数
void frictionModel::dynamicConstrainCompensateFriction
(
	const EcRealVector& currentJointVel,
	const EcRealVector& currentJointAcc
)
{
	for (int ii = 0; ii < m_NumJoints; ii++)
	{
		m_dynCompensateFactor[ii] = 0.9;
		m_dynViscousCompensateFactor[ii] = 0.8;
	}
	//m_dynCompensateFactor[4] = 0.3;
	//m_dynViscousCompensateFactor[4] = 0.5;
	/*
	// 1. 100ms为一个检测周期，太长可能会导致关节滑动距离过大，太短，影响体验。
	// 2. 以加速度作为观测对象，在降低补偿力矩的时间内，是否会观察到加速度（平均值）变大的情况，
	// 3. 规则：
	//		1） 只在加速度为正的时候，才补偿100%的静摩擦力，否则为50%；


	// 比较前后50ms时间段内的平均速度，如果速度是上升的，才进行100%补偿；
	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
		if (fabs(currentJointVel[ii])>lowVelocityThresholds)
		{
			if (m_dynConstraintTime[ii] < 0.1)				// 100 ms 为一个判断检测周期；
			{
				m_jointMeanVel[ii] += fabs(currentJointVel[ii]);
			}
			else
			{
				// 在第100ms这个周期，与前一个时间段进行比较；
				if (m_jointMeanVel[ii] > 1.01 * m_jointPreMeanVel[ii])			// 一定要比前一个周期的速度快
				{
					m_dynCompensateFactor[ii] = 1.0;
					m_dynViscousCompensateFactor[ii] = 1.25;
				}
				else
				{
					//m_dynCompensateFactor[ii] = m_constraintDynCompensateFactor[ii];
					m_dynCompensateFactor[ii] = 0.5;
					m_dynViscousCompensateFactor[ii] = 0.75;
				}
				m_jointPreMeanVel[ii] = m_jointMeanVel[ii];
				m_jointMeanVel[ii] = 0;
				m_dynConstraintTime[ii] = 0.0;
			}

			m_dynConstraintTime[ii] += m_updateTimePeriod;
		}
		else
		{
			m_jointPreMeanVel[ii] = 0.0;		// 将上一时间段的统计速度归零，更容易在下一阶段达到大的启动补偿力矩；
			m_dynConstraintTime[ii] = 0.09;		// 只需要10ms，就能进入大幅度的补偿？
		}

	}
	*/

	/*
	for (EcU32 ii = 0; ii < m_NumJoints; ii++)
	{
	if (m_dynConstraintTime[ii] < 0.15)
	m_dynCompensateFactor[ii] = 1.0;
	else if (m_dynConstraintTime[ii] < 0.2)
	m_dynCompensateFactor[ii] = 0.5;
	else
	m_dynConstraintTime[ii] = 0;
	m_dynConstraintTime[ii] += m_updateTimePeriod;
	}
	*/
}



void frictionModel::resetStatus()
{
	m_currentTime = 0.0;
	m_stopStateTime = 0.0;
}
