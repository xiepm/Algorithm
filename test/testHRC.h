/**/
#include "cobotInterface.h"
#include "hansTypeDef.h"
#include <iostream>
#include <vector>
#include <cmath>

void printVector(const std::string& name, RTS_IEC_LREAL* vec, int n)
{
    std::cout << name << ": ";
    for (int i = 0; i < n; i++)
    {
        std::cout << vec[i] << (i == n - 1 ? "" : ", ");
    }
    std::cout << std::endl;
}

void runSingleTest(int robotType, int joints)
{
    std::cout << "\n>>> Testing Robot Type: " << robotType << " (" << joints << " DOF) <<<" << std::endl;

    RTS_IEC_LREAL jointPosition[10] = { 0 };
    RTS_IEC_LREAL period = 0.004;
    RTS_IEC_LREAL kinParams[10] = { 220, 420, 185, 380, 0, 0, 0, 0, 0, 0 };

    // 1. Initialize
    initializeCollaborativeAlgorithm(jointPosition, period, robotType, kinParams);
    std::cout << "Initialization successful." << std::endl;

    // 2. Set Dynamics Parameters
    int totalDynParams = joints * 13;
    std::vector<RTS_IEC_LREAL> dynParams(130, 0.0);
    for (int i = 0; i < totalDynParams; i++) dynParams[i] = 0.1 * (i % 10 + 1); 
    setDynamicsLinearParametersCobot(dynParams.data());
    std::cout << "Dynamics parameters set." << std::endl;

    // 3. Set Robot Mounting
    setRobotMountingAnglesCobot(45.0, 90.0);
    std::cout << "Mounting angles set." << std::endl;

    // 4. Set Actuator Limits
    RTS_IEC_LREAL maxCurrents[10] = { 10, 10, 10, 5, 5, 5, 5, 0, 0, 0 };
    setActutorMaxCurrentLimits(maxCurrents);
    setControlBoxAllowCurrentLimit(150.0);
    std::cout << "Actuator limits set." << std::endl;

    // 5. Set Friction and Velocity Thresholds
    RTS_IEC_LREAL frictionFactor[10] = { 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0, 0, 0 };
    setFrictionCompensatoryFactor(frictionFactor);
    
    RTS_IEC_LREAL velThreshold[10] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0, 0, 0 };
    setLowVelocityThreshold(velThreshold);
    std::cout << "Friction and velocity thresholds set." << std::endl;

    // 6. Basic Calculations
    RTS_IEC_LREAL jointPosIn[10] = { 0 };
    RTS_IEC_LREAL jointVelIn[10] = { 0 };
    RTS_IEC_LREAL torqueOut[10] = { 0 };
    for(int i=0; i<joints; i++) {
        jointPosIn[i] = 10.0 * (i+1); 
        jointVelIn[i] = 1.0;      
    }

    getGravityAndCoriolisCentrifugalTorque(jointPosIn, jointVelIn, torqueOut);
    printVector("Grav & Coriolis Torque", torqueOut, joints);

    RTS_IEC_LREAL inertia[10] = { 0 };
    getJointInertia(jointPosIn, inertia);
    printVector("Inertia (Initial)", inertia, joints);

    // 7. Payload and updated calculations
    RTS_IEC_LREAL centerOfMass[3] = { 10, 20, 30 };
    setPayloadMassPropertiesCobot(3.5, centerOfMass);
    getJointInertia(jointPosIn, inertia);
    printVector("Inertia (With Payload)", inertia, joints);

    // 8. State Updates
    RTS_IEC_LREAL motorCurrents[10] = { 1, 1, 1, 1, 1, 1, 1, 0, 0, 0 };
    updateStateEstimates(jointPosIn, jointPosIn, motorCurrents, 0.004);
    std::cout << "State estimates updated." << std::endl;

    // 9. Collision Check
    RTS_IEC_BOOL collisionStatus[10] = { 0 };
    RTS_IEC_BOOL hasCollision = checkForCollision(collisionStatus);
    std::cout << "Collision status: " << (int)hasCollision << std::endl;

    // 10. Get Estimated State
    RTS_IEC_LREAL q[10], dq[10], ddq[10], cur[10], sTor[10], eTor[10], dTor[10];
    getEstimatedState(q, dq, ddq, cur, sTor, eTor, dTor);
    printVector("Estimated Disturb Torque", dTor, joints);

    // 11. Assistive Mode Simulation
    setStartAssistiveMode();
    RTS_IEC_LREAL assistiveCurrents[10] = { 0 };
    getAssistiveModeMotorCurrentCommands(assistiveCurrents, collisionStatus);
    printVector("Assistive Mode Currents", assistiveCurrents, joints);
    
    setReadyToCloseAssistiveMode();
    std::cout << "Ready to close assistive mode set." << std::endl;

    // 12. Param retrieval check
    cobotParams params;
    if (getSettingParams(&params)) {
        std::cout << "Params retrieved. Mass check: " << params.mass << " kg" << std::endl;
    }

    std::cout << ">>> Test finished for type " << robotType << " <<<" << std::endl;
}

void testHRC()
{
    int v = getCobotAlgorithmVersion();
    std::cout << "Cobot Algorithm Version: " << v << std::endl;

    // 6DOF Test
    runSingleTest(0, 6);

    // 7DOF Test
    runSingleTest(20, 7);

    std::cout << "\nAll test suites executed!" << std::endl;
}
