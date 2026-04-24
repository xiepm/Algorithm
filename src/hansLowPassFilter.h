#ifndef hansLowPassFilter_H_
#define hansLowPassFilter_H_
//------------------------------------------------------------------------------
// Copyright (c) 22016 Energid Technologies. All rights reserved.
//
/// @file ecLowPassFilter.h
/// @class EcLowPassFilter
/// @brief Low pass filter for online and offline applications.
/// @details This class implements low pass filters.
//
//------------------------------------------------------------------------------
//#include <actinCore/actinCore_config.h>  // Required to be first header.
//#include <foundCore/ecVector.h>
#include "typeDef.h"
#include <queue>

class hansLowPassFilter
{

public:

	/// constructor
	hansLowPassFilter
		(
		);

	hansLowPassFilter
		(
		const EcReal& timeConstant,
		const EcReal& samplingTime
		);

	hansLowPassFilter
		(
		const EcReal& timeConstant,
		const EcReal& samplingTime,
		const EcReal& oldData
		);

	virtual ~hansLowPassFilter();

	/// @brief Get the time constant
	virtual EcReal& timeConstant
		(
		);

	/// @brief Set time constant
	/// @param timeConstant Time constant
	virtual void setTimeConstant
		(
		const EcReal& timeConstant
		);

	/// @brief Get the sampling time
	virtual EcReal& samplingTime
		(
		);

	/// @brief Set the sampling time
	/// @param timeConstant Time constant
	virtual void setSamplingTime
		(
		const EcReal& samplingTime
		);

	/// @brief Set old data
	virtual void setOldData
		(
		const EcReal& oldData
		);

	/// Use this when dealing with on-line signal, one datum at a time.
	virtual EcReal updateOutput
		(
		const EcReal& newData
		);

	/// static implementation
	static EcReal output
		(
		EcReal newData,
		EcReal oldData,
		EcReal samplingTime,
		EcReal timeConstant
		);

	/// reset the current and old values
	virtual void reset
		(
		);

protected:
	/// Filter time constant
	EcReal         m_TimeConstant;

	/// Filter sampling time
	EcReal         m_SamplingTime;

	/// Data from previous cycle
	EcReal         m_OldData;
};

typedef std::vector<hansLowPassFilter> hansLowPassFilterVector;

#endif // ecRunningAverageFilter.h
