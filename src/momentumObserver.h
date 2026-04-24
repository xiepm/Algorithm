#ifndef	MOMENTUMOBSERVER_H_
#define MOMENTUMOBSERVER_H_

//#include<vld.h>
//#include "hansTypes.h"
#include "hansHighPassFilter.h"
#include "dynamics/dynamicsBase.h"
#include "dynamics/elfinDynamics.h"
#include "dynamics/sevendofDynamics.h"
#include "dynamics/urDynamics.h"
#include <memory>
#include <iostream>

class momentumObserver
{
#define						LOWLEVELVELOCITY						0.157
public:
	static std::shared_ptr<momentumObserver> create
	(
		const EcReal updateTimePeriod,
		const EcRealVector& high_b, const EcRealVector& high_a
	);

	/// destructor
	~momentumObserver();

	momentumObserver(const EcReal updateTimePeriod, const EcRealVector& high_b, const EcRealVector& high_a);

	EcBoolean initializeStates
	(
		const EcRealVector& jointPositions,
		const int robotType
	);

	// set kinematics and dynamics parameters
	EcBoolean setKinematicsParameters(const EcRealVector& dhParameters);
	EcBoolean setDynamicsParameters(const EcRealVector& dynamicsParam);

	void setGravityAcceleration
	(
		const EcReal gx, const EcReal gy, const EcReal gz
	);

	void setCollisionThreshold
	(
		const EcRealVector threshold
	);

	void setDynamicsFactorThreshold
	(
		const EcRealVector threshold
	);

	EcBoolean updateStateEstimates
	(
		const EcRealVector& jointPositions,
		const EcRealVector& jointVelocities,
		const EcRealVector& jointAccelerations,
		const EcRealVector& sensedTorque
	);


	void getObserverTorqueDisturbances
	(
		EcRealVector& disturbanceTorque
	);

	void getRawObserverTorqueDisturbances
	(
		EcRealVector& disturbanceTorque
	);

	void getJointCollisionState
	(
		EcBooleanVector& jointCollisionState
	);

	void getJointGeneralizeMomentum
	(
		EcRealVector& jointMomentum
	);

	void calculateMomentum(const EcRealVector& jointPosition, const EcRealVector& jointVelocity, EcReal& momentum);


protected:
	EcBoolean calculateMomentumObserverTorque
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& sensedTorque,
		EcRealVector& observerTorque
	);

	void calculateDynamicsThreshold
	(
	);

	void reduceMotorSideTorques
	(
		const EcRealVector& sensedTorque,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		EcRealVector& linkSideTorques
	);

	void calculateGeneralizeMomentum
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		EcRealVector& jointMomentum
	);

	void calculateMassMatrix
	(
		const EcRealVector& q,
		EcRealVectorVector& massMatrix
	);

	void calculateBetaQuantity
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		EcRealVector& beta
	);

	void diffObserver
	(
		const EcRealVector& sensedTorque,
		const EcRealVector& disturbanceTorque,
		const EcRealVector& q,
		const EcRealVector& dq,
		EcRealVector& diffObser
	);




	EcReal sign(const EcReal x);


private:
	dynBasePtr								m_dynBase;
	EcReal                                  m_SamplePeriod;     ///< Sampling time of the process
	EcBoolean								b_isInitialize;
	EcReal									m_KObserverCoeff;
	EcRealVector							m_dynamicsParam;
	EcRealVector							m_dynamicsFullParam;
	EcRealVector							m_moterSideParams;
	EcRealVector							m_coulombFriction;
	EcRealVector							m_jointMomentum;
	EcU32									m_NumJoints;			    	//Number of Robot

	EcRealVector							m_jointPosition, m_jointVel, m_jointAcc;
	EcRealVector							m_linkSideTorques, m_sensedTorques;
	EcRealVector							m_ObserverTorque;
	EcRealVector							m_filtedObserverTorques;
	EcRealVector							m_dynamicsFactorThresholds;

	EcReal									m_gx, m_gy, m_gz;
	EcRealVector							m_collisionThreshold;
	EcRealVector							m_dynamicCollisionThreshold;
	EcRealVector							b_jointCollisionStatus;
	hansHighPassFilter						m_highPassFilter;
	EcRealVector							high_a, high_b;

private:
	EcRealVector				m_zeros;
	EcRealVector				m_v1, m_v2, m_v3, m_v4, m_v5, m_v6;
	EcRealVector				m_M1, m_M2, m_M3, m_M4, m_M5, m_M6;
	EcRealVector				m_NE0, m_gravityTau;
	EcRealVector				m_beta;
	EcRealVectorVector			m_diffMassMatrix;
	EcRealVectorVector			m_massMatrix;
	EcRealVectorVector			m_massMatrixPre;
	EcRealVector				m_diffObserver;
	EcRealVector				m_diffObserverSum;
	EcRealVector				m_preDisturbancsTorques;
	/**/
	EcReal x0, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14,
		x15, x16, x17, x18, x19, x20, x21, x22, x23, x24, x25, x26, x27, x28,
		x29, x30, x31, x32, x33, x34, x35, x36, x37, x38, x39, x40, x41, x42,
		x43, x44, x45, x46, x47, x48, x49, x50, x51, x52, x53, x54, x55, x56,
		x57, x58, x59, x60, x61, x62, x63, x64, x65, x66, x67, x68, x69, x70,
		x71, x72, x73, x74, x75, x76, x77, x78, x79, x80, x81, x82, x83, x84,
		x85, x86, x87, x88, x89, x90, x91, x92, x93, x94, x95, x96, x97, x98,
		x99, x100, x101, x102, x103, x104, x105, x106, x107, x108, x109, x110,
		x111, x112, x113, x114, x115, x116, x117, x118, x119, x120, x121, x122,
		x123, x124, x125, x126, x127, x128, x129, x130, x131, x132, x133, x134,
		x135, x136, x137, x138, x139, x140, x141, x142, x143, x144, x145, x146,
		x147, x148, x149, x150, x151, x152, x153, x154, x155, x156, x157, x158,
		x159, x160, x161, x162, x163, x164, x165, x166, x167, x168, x169, x170,
		x171, x172, x173, x174;

	EcReal xx0, xx1, xx2, xx3, xx4, xx5, xx6, xx7, xx8, xx9, xx10, xx11, xx12, xx13,
		xx14, xx15, xx16, xx17, xx18, xx19, xx20, xx21, xx22, xx23, xx24, xx25, xx26,
		xx27, xx28, xx29, xx30, xx31, xx32, xx33, xx34, xx35, xx36, xx37, xx38, xx39,
		xx40, xx41, xx42, xx43, xx44, xx45, xx46, xx47, xx48, xx49, xx50, xx51, xx52,
		xx53, xx54, xx55, xx56;

};

typedef std::shared_ptr<momentumObserver> CMomentumSharedPtr;
#endif


