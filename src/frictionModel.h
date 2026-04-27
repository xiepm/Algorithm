#pragma once
#include "frames.hpp"
#include "typeDef.h"
//#include <boost/circular_buffer.hpp>
#include "circular_buffer.hpp"
#include <iostream>
#include <deque>
//#define				dualDiffEncoderThd								 0.001					// 0.001°，使用双编位置差值，作为开启静摩擦力矩补偿的依据；

class predictiveStartTorque
{
	// 在启动阶段，如果检测到抖动力矩突然有大的变化，可能已经被施加外力，增加对应方向的启动力矩补偿；

	// 对单独的一个关节建模，描述如下行为：
	//	1. 收集最新buffSize的关节抖动换向时的力矩；
	//  2. 当缓存数满足buffSize个时，每个进入周期完成如下动作
	//		1) 计算平均值 averageTorque，计算幅值amplitudeTorque;
	//		2) 如果 (currentTorque - averageTorque)/amplitudeTorque > 150% ，开启补偿预判力矩，持续200ms；
	//      3) 预判力矩的计算, predictiveCompensateTorque = -sign(currentTorque) * 0.5 * coulombFriction;
	//		4) 第一次进入抖动时，将buffSize清空；
	//	3. 目标：对外提供predictiveCompensateTorqueCoeff；
	//	4. 如果进入补偿通道，则不应再修改补偿系数

	// 补充：
	//		1. 实时输入当前的检测力矩，如果当前力矩比前面3个峰值周期的平均值还要大30%，立即开始反向补偿力矩；

#define				buffSize									 10						// 预判启动的循环缓存大小；
#define				compensateLastTime							 0.02

public:
	void initialize(EcReal timePeriod);

	// 当检测到抖动位置达到换向边界时，更新当前的检测力矩，完成启动库伦摩擦力补偿系数的计算；
	void updateTorques(int ii,const EcReal sensedTorque);

	// 每个实时周期进行调用，输出预判启动摩擦力补偿系数；
	EcBoolean getPredictiveCompensateTorque(EcReal& compensateTorque);

	// 实时判断检测力矩，是否在抖动期间，出现异常的比较大的值；若是，则反向补偿力矩；
	void checkSensedTorque(const EcReal& sensedTorque);


	// 第一次进入抖动时，需要将前面的缓存清空；
	void reset();
	


private:
	EcU32							m_enterCount;
	EcReal							m_updateTimePeriod;
	EcReal							m_averageTorque;
	EcReal							m_amplitudeTorque;
	EcReal							m_currentTime;
	EcReal							m_predictiveCompensateTorque;
	circular_buffer<EcReal>	m_OldSensedTorques;     ///< A circular buffer for time series joint position data
};

// 滑动窗口计算定周期的差值；
class SlidingWindow {
public:
	SlidingWindow(){
		window_size_ = 1;
		window_.resize(window_size_);
		diffWindow_.resize(window_size_);
	}

	void setWindowSize(const size_t new_size, const size_t middleSize) {
		window_size_ = new_size;
		window_.resize(new_size);
		diffWindow_.resize(new_size);

		//计算200ms对应的整数周期；
		middle_size_ = middleSize;

	}

	void addValue(double value) {
		window_.pop_front();
		window_.push_back(value);
	}

	bool checkOverZeroCrossingsCount(const std::deque<double>& data, size_t window_size) const {
		bool isOver = false;
		int zero_crossings = 0;
		size_t start_index = data.size() - window_size;
		for (size_t i = start_index + 1; i < data.size(); ++i) {
			if (data[i - 1] * data[i] < 0) {
				++zero_crossings;
				if (zero_crossings > 2)
				{
					isOver = true;
					break;
				}
			}
		}
		//std::cout << "crossCount:" << isOver << "," << zero_crossings<<","<<data.size()<<","<<window_size<<","<<data[middle_size_]<<","<<data.back() << std::endl;
		return isOver;
	}

	double getDifference(const double thd) {
		// 500ms和200ms两个时间间距双编差值的变化量，取其中最大数值；
		double middleDelta = window_.back() - window_[middle_size_];
		double lastDelta = window_.back() - window_.front();

		// 加大500ms的权重；
		lastDelta = fabs(lastDelta) > thd ? lastDelta * 2.0 : lastDelta;

		EcReal diff = fabs(middleDelta) > fabs(lastDelta) ? middleDelta : lastDelta;
		diffWindow_.pop_front();
		diffWindow_.push_back(diff);

		bool isVibration = checkOverZeroCrossingsCount(diffWindow_, middle_size_);
		if (isVibration)
			return 0;
		else
		{
			return diff;
		}
		
	}

private:
	std::deque<double> window_;
	std::deque<double> diffWindow_;
	size_t window_size_;
	size_t middle_size_;
};




class frictionModel
{
#define				zeroVelocityThresholds						 0.03					// 1.7°/s
#define				lowVelocityThresholds						 0.08					// 4.5°/s
#define				risingTime									 0.1					// 0.1s的时间从0上升到库伦摩擦力数值；


public:
	frictionModel();


	void initialize
		(
		const EcRealVector& currentJointPosition, 
		const EcReal updateTimePeriod
		);


	void setFrictionParams
		(
		const EcRealVector& coulombFriction,
		const EcRealVector& viscousFriction
		);

	// 静摩擦力矩的补偿；
	void setCompensateFactor
		(
		const EcRealVector& compensateFactor
		);

	// 粘性摩擦力矩的补偿系数；
	void setDynFrictionCompensateFactor
	(
		const EcRealVector& compensateFactor
	);

	void setStartCompensateFrictionFactor
		(
			const EcRealVector& compensateFactorII
		);

	void setMaxJointVelocitiesInAssistiveMode
		(
		const EcRealVector& maxJointVelocities
		);

	// 是否使用双编的状态；
	void setAssistDualEncoderFlag(const EcBoolean flag) {
		b_dualEncoderAssistFlag = flag;
	}

	// 基于双编差值处理的启动补偿力矩，未使用；
	void setDualEncoderStateCompensateFactor(const EcRealVector& dualFactor) {
		m_dualCompensateFactor = dualFactor;
		//m_compensateConfigFactor = dualFactor;
	}
	
	// 实时更新双编码器的数值；
	void updateDualEncoder(const EcRealVector& jointSidePosition, const EcRealVector& motorSidePosition);

	void setDualDiffEncoderThd(const EcRealVector& thd) {
		m_dualDiffEncoderThd = thd;
	}

	void calculateCompensateFriction
		(
		const EcRealVector& currentJointPosition,
		const EcRealVector& currentJointVel,
		const EcRealVector& currentJointAcc,
		const EcRealVector&	sensedTorque,
		const EcBoolean&	driveBackMode,
		EcRealVector&		compensateCoulombFriction,
		EcRealVector&		compensateViscousFriction
		);

	EcReal calculateFrictionTorque(const EcU32 index, const EcReal jointVel, const EcReal coulombFactor = 1.0, const EcReal viscousFactor = 1.0) {
		return m_coulombFricton[index] * KDL::sign(jointVel) * coulombFactor + m_viscousFrictionCoeff[index] * jointVel * viscousFactor;
	}

	EcReal calculateFrictionTorqueForCloseAssistive(const EcU32 index, const EcReal jointVel, const EcReal coulombFactor = 1.0, const EcReal viscousFactor = 1.0) {
		EcReal viscousJointVel = (fabs(jointVel)<0.015)? 0.0:jointVel;
		return m_coulombFricton[index] * sign(viscousJointVel) * coulombFactor + m_viscousFrictionCoeff[index] * viscousJointVel * viscousFactor;
	}



	void resetStatus();

private:

	// 生成抖动的力矩信号，使得关节在预滑动阶段，运动范围不超过0.002°
	void generateDitherSignal
		(
		const EcSizeT&		index,
		const EcRealVector& currentJointPosition,
		const EcRealVector& currentJointVel,
		const EcRealVector& sensedTorques
		);

	// 当关节有一定速度时，定周期下降补偿数值，观察关节速度和加速度，判断是否处于无外力状态；
	void dynamicConstrainCompensateFriction
		(
		const EcRealVector& currentJointVel,
		const EcRealVector& currentJointAcc
		);
	EcReal sign(const EcReal v) {
		if (v > 0.01)
			return 1;
		else if (v < -0.01)
			return -1;
		else
			return 0;
	}


private:
	EcU32								m_NumJoints;
	EcReal								m_updateTimePeriod;
	EcReal								m_currentTime;									// 启动零力示教后的运行时间；

	EcRealVector						m_coulombFricton, m_viscousFrictionCoeff;		// 本体的库伦和粘性摩擦力矩；
	EcRealVector						m_calcCoulombFriction, m_calcViscousFriction;   // 计算得到的输出库伦和粘性摩擦力矩；
	EcRealVector						m_compensateStartFrictionFactor;				// 配置文件设置的最大抖动允许力矩；
	EcRealVector						m_compensateConfigFactor;						// 配置文件设置的（库伦）补偿比例；
	EcRealVector						m_dynCompensateFactor;							// 动态约束的库伦摩擦力补偿比例；
	EcRealVector						m_constraintDynCompensateFactor;				// 被约束时的库伦摩擦力补偿百分比；
	EcRealVector						m_dynViscousCompensateFactor;					// 动态约束的粘性摩擦力补偿比例；

	EcRealVector						m_maxJointVelocities;							// 允许的最大关节速度（60°/s）



	// 生成预滑动时抖动力矩
	EcBooleanVector						b_activeDitherStatus;							//是否是首次激活抖动信号；
	EcRealVector						m_ditherStartPosition;							// 抖动开始或换向时的起始位置；
	EcRealVector						m_ditherPositionErr;							// 当前抖动位置和换向时的位置偏差；
	EcRealVector						m_ditherPrePositionErr;							// 保存上一换向时的位置偏差；
	
	EcRealVector						m_deltaFriction;								// 每个实时周期力矩增加的数值；
	EcRealVector						m_ditherDirection;								// 抖动力矩变化的方向；
	EcReal								m_stopGenerateFrictTime;						// 允许生成抖动力矩的最大时间；
	EcReal								m_stopStateTime;								// 已生成抖动力矩的时间；


	// 生成运动时动态减少的力矩
	EcRealVector						m_dynConstraintTime;
	EcRealVector						m_jointMeanVel;
	EcRealVector						m_jointPreMeanVel;

	// 预测启动时的外部力矩
	std::vector<predictiveStartTorque>	m_predictiveTorque;
	EcRealVector						m_meanJointTorques;								// 在dither状态下一段时间内（5个周期）最大值和最小值的移动平均值；
	EcRealVector						m_amplitudeDitherTorques;						// 在dither状态下一段时间内的幅值（平均值）；

	// 双编信息
	EcBoolean							b_dualEncoderAssistFlag;						// 是否使用双编码器辅助；
	std::vector<SlidingWindow>			m_dualEncoderDiffWindows;						// 双编差值处理滑动窗口；
	EcRealVector						m_dualCompensateFactor;							// 双编状态估计下的补偿系数； 

	EcRealVector						m_dualDiffEncoderThd;							// 使用双编位置差值，作为开启静摩擦力矩补偿的依据；
};