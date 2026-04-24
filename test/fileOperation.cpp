#include "fileOperation.h"


bool fileDataRead(string filePath, int columnCounts, EcRealVectorVector& out)
{
	EcRealVector rowData(columnCounts);

	ifstream iFile(filePath);

	string row;
	vector<string> data;
	int typeChoose;

	bool ret = true;

	if (!iFile.good())
	{
		std::cout<<"file did not import."<<std::endl;
		ret = false;
	}

	while (iFile.good())
	{
		getline(iFile, row);
		splitCSVDataLine(row, ",", data);

		if (data.size() != columnCounts)
			continue;
		for (int ii = 0; ii < columnCounts; ii++)
		{
			rowData[ii] = atof(data[ii].c_str());
		}
		out.push_back(rowData);
	}

	return ret;
}


bool fileOperation(string filePath, EcRealVector& time, EcRealVectorVector& q)
{
	EcRealVector zeros, q1(6), dq1(6), ddq1(6), sensedTorques1(6), estimateTorques1(6);
	zeros.assign(6,0.0);
	ifstream iFile(filePath);

	string row;
	vector<string> data;
	int typeChoose;

	bool ret = true;
	
	if (!iFile.good())
	{
		ret = false;
	}
	
	while (iFile.good())
	{
		getline(iFile,row);
		splitCSVDataLine(row, ",", data);

		if (data.size() < 5)
			continue;
		for (int ii = 0; ii < 6; ii++)
		{
			q1[ii] = atof(data[ii+1].c_str());
		}
		time.push_back(atof(data[0].c_str()));
		q.push_back(q1);
	}

	return ret;
}

bool fileOperation2(string filePath, EcRealVector& time, EcRealVectorVector& q)
{
	EcRealVector zeros, q1(6), dq1(6), ddq1(6), sensedTorques1(6), estimateTorques1(6);
	zeros.assign(6, 0.0);
	ifstream iFile(filePath);

	string row;
	vector<string> data;
	int typeChoose;

	bool ret = true;

	if (!iFile.good())
	{
		ret = false;
	}

	while (iFile.good())
	{
		getline(iFile, row);
		splitCSVDataLine(row, ",", data);

		if (data.size() < 5)
			continue;
		for (int ii = 0; ii < 6; ii++)
		{
			q1[ii] = atof(data[ii].c_str());
		}
		time.push_back(atof(data[0].c_str()));
		q.push_back(q1);
	}

	return ret;
}


bool fileOperationMomentumObserver(string filePath, EcRealVectorVector& q, EcRealVectorVector& dq, EcRealVectorVector& ddq,
	EcRealVectorVector& sensedTorque)
{

	EcRealVector zeros, q1(6), dq1(6), ddq1(6), sensedTorques1(6);
	zeros.assign(6, 0.0);
	ifstream iFile(filePath);

	string row;
	vector<string> data;

	bool ret = true;

	if (!iFile.good())
	{
		ret = false;
	}

	while (iFile.good())
	{
		getline(iFile, row);
		splitCSVDataLine(row, ",", data);

		if (data.size() < 5)
			continue;
		for (int ii = 0; ii < 6; ii++)
		{
			q1[ii] = atof(data[ii].c_str());
			dq1[ii] = atof(data[ii + 6].c_str());
			ddq1[ii] = atof(data[ii + 12].c_str());
			sensedTorques1[ii] = atof(data[ii + 18].c_str());
		}
		q.push_back(q1);
		dq.push_back(dq1);
		ddq.push_back(ddq1);
		sensedTorque.push_back(sensedTorques1);
	}

	return ret;


}


void splitCSVDataLine(string str, string separator, vector<string>& result)
{
	result.clear();
	int cutAt;
	while ((cutAt = str.find_first_of(separator)) != str.npos)
	{
		if (cutAt > 0)
		{
			result.push_back(str.substr(0, cutAt));
		}
		str = str.substr(cutAt + 1);
	}
	if (str.length() > 0)
	{
		result.push_back(str);
	}
}
