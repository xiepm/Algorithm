#pragma once
#include <string.h>
#include "hansTypes.h"

class MovingAverage
{
public:
	MovingAverage()
	{
		DataPool.assign(1, 0.0);
		DataPoolLength = 1;
		CountingIndex = 0;
		Mean = 0.0;
	}

	~MovingAverage() {}

	void reset()
	{
		DataPool.assign(1, 0.0);
		CountingIndex = 0;
		Mean = 0.0;
	}

	void setDataPoolLength(const int& size,const double initValue=0)
	{
		if (size < 1)
		{
			DataPoolLength = 1;
		}
		else
		{
			DataPoolLength = size;
		}

		DataPool.assign(DataPoolLength,initValue);
		Mean = initValue;
	}

	void updateFilteredData(const double& InVar, double& OutVar)
	{
		Mean = Mean - DataPool[CountingIndex] / DataPoolLength + InVar / DataPoolLength;
		DataPool[CountingIndex] = InVar;
		OutVar = Mean;

		CountingIndex++;
		if (CountingIndex >= DataPoolLength)
		{
			CountingIndex = 0;
		}
	}

private:
	EcRealVector DataPool;


	int DataPoolLength;
	int CountingIndex;
	double Mean;
};
typedef std::vector<MovingAverage> hansMovingAverageVector;
