#include "fileOperation.h"
#include <iostream>
#include <string>
#include "src/HMCollaborativeRobotAlgorithm.h"

namespace
{
std::string testFilePath(const std::string& fileName)
{
	std::string sourcePath = __FILE__;
	std::string::size_type slashPos = sourcePath.find_last_of("/\\");
	std::string baseDir = (slashPos == std::string::npos) ? "." : sourcePath.substr(0, slashPos);
	return baseDir + "/" + fileName;
}
}

void testLostData()
{
	std::shared_ptr<CHansCollaborativeAlgorithm> m_cobotAlgorithm;
	EcRealVector jointPosition(6);

	std::string fileRead = testFilePath("verifyData.csv");
	EcRealVectorVector data;
	fileDataRead(fileRead, 25, data);           // 这个例子中没有使用；

	// write data
	std::string filePath = testFilePath("output.csv");
	ofstream oFile;
	oFile.open(filePath, ios::out | ios::trunc);


	EcRealVector commandJointPosition(6), actualJointPosition(6), current(6);

	EcRealVector jointPostions;
	EcRealVector jointVelocities;
	EcRealVector jointAccelerations;
	EcRealVector motorCurrents;
	EcRealVector sensedTorques(6);
	EcRealVector estimatedTorques(6);
	EcRealVector disturbanceTorques(6);


	m_cobotAlgorithm = CHansCollaborativeAlgorithm::create(actualJointPosition, 0.004, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);

	for (int i = 0; i < 100; i++)
	{
		// 设计一个丢帧场景
		actualJointPosition[1] = actualJointPosition[1] + 0.1;
		if (i == 50 || i == 51)
			actualJointPosition[1] = actualJointPosition[1] - 0.1;          // 缺一个帧
		if (i == 52)
			actualJointPosition[1] = actualJointPosition[1] + 0.2;          // 下个周期补回

		m_cobotAlgorithm->updateStateEstimates(actualJointPosition, actualJointPosition, current, 0);



		m_cobotAlgorithm->getEstimatedState
		(
			jointPostions,
			jointVelocities,
			jointAccelerations,
			motorCurrents,
			sensedTorques,
			estimatedTorques,
			disturbanceTorques
		);
		oFile << actualJointPosition[1] << "," << jointPostions[1] << "," << jointVelocities[1] << "," << jointAccelerations[1] << std::endl;
		//std::cout << actualJointPosition[1]*KDL::rad2deg<<","<< actualJointPosition[1] << "," << jointPostions[1] << "," << jointVelocities[1] << "," << jointAccelerations[1] << std::endl;

	}

	std::cout << "exit." << std::endl;
}

void testElfinDyn()
{
	std::shared_ptr<CHansCollaborativeAlgorithm> m_cobotAlgorithm;
	EcRealVector jointPosition(6);

	std::string fileRead = testFilePath("verifyData.csv");
	EcRealVectorVector data;
	fileDataRead(fileRead, 25, data);           // 这个例子中没有使用；

	// write data
	std::string filePath = testFilePath("output.csv");
	ofstream oFile;
	oFile.open(filePath, ios::out | ios::trunc);


	EcRealVector commandJointPosition(6), actualJointPosition(6), current(6);

	EcRealVector jointPostions;
	EcRealVector jointVelocities;
	EcRealVector jointAccelerations;
	EcRealVector motorCurrents;
	EcRealVector sensedTorques(6);
	EcRealVector estimatedTorques(6);
	EcRealVector disturbanceTorques(6);
	EcBooleanVector collisionType(6);
	ENCollisionType type;



	m_cobotAlgorithm = CHansCollaborativeAlgorithm::create(actualJointPosition, 0.004, 0);
	EcRealVector kinParams = { 0.22,0.42,0.185,0.38 };
	m_cobotAlgorithm->setRobotDHParameters(kinParams);

	EcRealVector params;
	params.assign(78, 1.0);
	m_cobotAlgorithm->setDynamicsLinearParameters(params);
	EcRealVector threshold;
	threshold.assign(6, 100);
	m_cobotAlgorithm->setCollisionStopInMomentumThresholds(threshold);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);

	for (int i = 0; i < data.size() - 1; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			actualJointPosition[j] = data[i][j * 4 + 2] * deg2rad;
			current[j] = data[i][j * 4 + 4];
		}

		m_cobotAlgorithm->updateStateEstimates(actualJointPosition, actualJointPosition, current, 0);
		m_cobotAlgorithm->checkForCollision(collisionType, type);


		m_cobotAlgorithm->getEstimatedState
		(
			jointPostions,
			jointVelocities,
			jointAccelerations,
			motorCurrents,
			sensedTorques,
			estimatedTorques,
			disturbanceTorques
		);
		oFile << actualJointPosition[1] << "," << jointVelocities[1] << "," << jointAccelerations[1] << "," << sensedTorques[1] << "," << estimatedTorques[1] << "," << disturbanceTorques[1] << std::endl;
		std::cout << actualJointPosition[1] << "," << sensedTorques[1] << "," << estimatedTorques[1] << "," << disturbanceTorques[1] << std::endl;
		//std::cout << actualJointPosition[1]*KDL::rad2deg<<","<< actualJointPosition[1] << "," << jointPostions[1] << "," << jointVelocities[1] << "," << jointAccelerations[1] << std::endl;

	}

	std::cout << "exit." << std::endl;
}


int main(int argc, char* argv[])
{
	std::shared_ptr<CHansCollaborativeAlgorithm> m_cobotAlgorithm;
	EcRealVector jointPosition(6);

	std::string fileRead = testFilePath("verifyData.csv");
	EcRealVectorVector data;
	fileDataRead(fileRead, 25, data);           // 这个例子中没有使用；

	// write data
	std::string filePath = testFilePath("output.csv");
	ofstream oFile;
	oFile.open(filePath, ios::out | ios::trunc);


	EcRealVector commandJointPosition(6), actualJointPosition(6), current(6);

	EcRealVector jointPostions;
	EcRealVector jointVelocities;
	EcRealVector jointAccelerations;
	EcRealVector motorCurrents;
	EcRealVector sensedTorques(6);
	EcRealVector estimatedTorques(6);
	EcRealVector disturbanceTorques(6);
	EcBooleanVector collisionType(6);
	ENCollisionType type;

	actualJointPosition.assign(6, 1);


	m_cobotAlgorithm = CHansCollaborativeAlgorithm::create(actualJointPosition, 0.004, 1);
	EcRealVector kinParams = { 0.22,0.42,0.180,0.38,0.3,0.5 };
	m_cobotAlgorithm->setRobotDHParameters(kinParams);

	EcRealVector params;
	params.assign(78, 1.0);
	m_cobotAlgorithm->setDynamicsLinearParameters(params);
	EcRealVector threshold;
	threshold.assign(6, 100);
	m_cobotAlgorithm->setCollisionStopInMomentumThresholds(threshold);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);
	m_cobotAlgorithm->updateStateEstimates(commandJointPosition, actualJointPosition, current, 0);

	for (int i = 0; i < data.size(); i++)
	{
		for (int j = 0; j < 6; j++)
		{
			actualJointPosition[j] = data[i][j * 4 + 2] * deg2rad;
			current[j] = data[i][j * 4 + 4];
		}

		m_cobotAlgorithm->updateStateEstimates(actualJointPosition, actualJointPosition, current, 0);
		m_cobotAlgorithm->checkForCollision(collisionType, type);


		m_cobotAlgorithm->getEstimatedState
		(
			jointPostions,
			jointVelocities,
			jointAccelerations,
			motorCurrents,
			sensedTorques,
			estimatedTorques,
			disturbanceTorques
		);
		oFile << actualJointPosition[1] << "," << jointVelocities[1] << "," << jointAccelerations[1] << "," << sensedTorques[1] << "," << estimatedTorques[1] << "," << disturbanceTorques[1] << std::endl;
		std::cout << actualJointPosition[1] << "," << sensedTorques[1] << "," << estimatedTorques[1] << "," << disturbanceTorques[1] << std::endl;
		//std::cout << actualJointPosition[1]*KDL::rad2deg<<","<< actualJointPosition[1] << "," << jointPostions[1] << "," << jointVelocities[1] << "," << jointAccelerations[1] << std::endl;

	}
	EcRealVector motorCommand(6);
	EcBooleanVector vbool(6);
	m_cobotAlgorithm->getAssistiveModeMotorCurrentCommands(motorCommand, vbool);
	std::cout << motorCommand[0] << "," << motorCommand[1] << "," << motorCommand[2] << "," << motorCommand[4] << "," << motorCommand[4] << "," << motorCommand[5] << std::endl;

	std::cout << "exit." << std::endl;
	return 0;
}
