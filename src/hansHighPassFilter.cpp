#include "hansHighPassFilter.h"


hansHighPassFilter::hansHighPassFilter
(
const EcRealVector& b,
const EcRealVector& a
)
{
	m_b = b;
	m_a = a;
}

void hansHighPassFilter::setOldData
(
const EcRealVector& oldData
)
{
	m_num = oldData.size();
	m_oldData = oldData;
	m_outputData = oldData;
}

EcRealVector hansHighPassFilter::updateOutput
(
const EcRealVector& data
)
{
	for (EcSizeT i = 0; i < m_num; i++)
	{
		m_outputData[i] = m_b[0] * data[i] + m_b[1] * m_oldData[i] - m_a[1] * m_outputData[i];
	}
	m_oldData = data;
	return  m_outputData;
}