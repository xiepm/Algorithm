#include "hansLowPassFilter.h"



hansLowPassFilter::hansLowPassFilter
(
const EcReal& timeConstant,
const EcReal& samplingTime
)
{
	m_TimeConstant = timeConstant;
	m_SamplingTime = samplingTime;
	m_OldData = 0.0;
}


hansLowPassFilter::hansLowPassFilter
(
const EcReal& timeConstant,
const EcReal& samplingTime,
const EcReal& oldData
)
{
	m_TimeConstant = timeConstant;
	m_SamplingTime = samplingTime;
	m_OldData = oldData;
}


hansLowPassFilter::~hansLowPassFilter()
{

}

/// @brief Get the time constant
EcReal& hansLowPassFilter::timeConstant
(
)
{
	return m_TimeConstant;
}

/// @brief Set time constant
/// @param timeConstant Time constant
void hansLowPassFilter::setTimeConstant
(
const EcReal& timeConstant
)
{
	m_TimeConstant = timeConstant;
}

/// @brief Get the sampling time
EcReal& hansLowPassFilter::samplingTime
(
)
{
	return m_SamplingTime;
}

/// @brief Set the sampling time
/// @param timeConstant Time constant
void hansLowPassFilter::setSamplingTime
(
const EcReal& samplingTime
)
{
	m_SamplingTime = samplingTime;
}

/// @brief Set old data
void hansLowPassFilter::setOldData
(
const EcReal& oldData
)
{
	m_OldData = oldData;
}

/// Use this when dealing with on-line signal, one datum at a time.
EcReal hansLowPassFilter::updateOutput
(
const EcReal& newData
)
{
	if (fabs(m_SamplingTime + m_TimeConstant) < 0.00001)
	{
		return 0;
	}

	EcReal retVal = (newData*m_SamplingTime + m_OldData*m_TimeConstant) / (m_SamplingTime + m_TimeConstant);
	
	m_OldData = retVal;
	return retVal;
}

/// static implementation
EcReal hansLowPassFilter::output
(
EcReal newData,
EcReal oldData,
EcReal samplingTime,
EcReal timeConstant
)
{
	if (fabs(samplingTime + timeConstant) < 0.00001)
	{
		return 0;
	}
	return (newData*samplingTime+oldData*timeConstant)/(samplingTime+timeConstant);
}

/// reset the current and old values
void hansLowPassFilter::reset
(
)
{
	m_OldData = 0.0;
}