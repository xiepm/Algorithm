#ifndef hansHighPassFilter_H_
#define hansHighPassFilter_H_
//#include <hansTypes.h>
#include "typeDef.h"

class hansHighPassFilter
{
public:
	hansHighPassFilter
		(
		const EcRealVector& b,
		const EcRealVector& a
		);

	void setOldData
		(
		const EcRealVector& oldData
		);

	EcRealVector updateOutput
		(
		const EcRealVector& data
		);

private:
	EcRealVector m_b, m_a;
	EcRealVector m_oldData,m_outputData;
	EcU32   m_num;


};



#endif