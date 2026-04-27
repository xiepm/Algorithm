/**
* <interfacename>ElfinCollaborativeLib</interfacename>
* <description></description>
*
* <copyright></copyright>
*/




#ifndef _ELFINCOLLABORATIVELIBITF_H_
#define _ELFINCOLLABORATIVELIBITF_H_

#include "CmpStd.h"








/** EXTERN LIB SECTION BEGIN **/
/*  Comments are ignored for m4 compiler so restructured text can be used.  */

#ifdef __cplusplus
extern "C" {
#endif

	/**
	 * <description>cobotParams</description>
	 */
	typedef struct tagcobotParams
	{
		RTS_IEC_LREAL kinParams[10];		/* kinrmatics parameters. */
		RTS_IEC_LREAL actuatorDamp[6];
		RTS_IEC_LREAL torqueConstant[6];
		RTS_IEC_LREAL gearRatio[6];
		RTS_IEC_LREAL maxEfficiency[6];
		RTS_IEC_LREAL maxActuatorCurrents[6];
		RTS_IEC_LREAL rotationofMounting;		/* rotate robot base mounting angles(deg) */
		RTS_IEC_LREAL tiltofMounting;		/* tilt robot base mounting angles(deg) */
		RTS_IEC_LREAL upperJointLimits[6];
		RTS_IEC_LREAL lowerJointLimits[6];
		RTS_IEC_LREAL dynamicsParams[78];
		RTS_IEC_LREAL collisionStopThresholds[6];		/* joint collisiton threshold; */
		RTS_IEC_LREAL collisionStopInMomentumThresholds[6];		/* joint collisiton threshold; */
		RTS_IEC_LREAL assistiveModeCollisionStopThresholds[6];
		RTS_IEC_LREAL frictionCompensatoryFactor[6];		/* default setting: J1="0.5" J2="0.5" J3="0.4" J4="0.6" J5="0.7" J6="0.7" */
		RTS_IEC_LREAL frictionCompensatoryFactorII[6];		/* default seting [1.0]; */
		RTS_IEC_LREAL lowVelocityThreshold[6];		/* decide whether the joints are in low velocity status; */
		RTS_IEC_LREAL maxJointVelocity[6];		/*  */
		RTS_IEC_LREAL maxPowerConstraint;		/* power */
		RTS_IEC_LREAL maxMomentumConstraint;		/* momentum */
		RTS_IEC_LREAL jointPowerConstraint[6];		/* power of joints; */
		RTS_IEC_LREAL distanceLimitInDriveBack;		/* allowing joint motion(degree,  °) in drive back mode; */
		RTS_IEC_LREAL timeDurationInDriveBack;		/* allowing time duration(second, s) in drive back mode; */
		RTS_IEC_LREAL mass;
		RTS_IEC_LREAL centerofMass[3];		/* (millimeter) */
		RTS_IEC_BOOL isAssistiveMode;
		RTS_IEC_INT drivebackMode;
		RTS_IEC_LREAL assistiveCheckTime;		/* the start time for check robot status in assistive mode, second ,s; */
	} cobotParams;

	/**
	 * <description>calculatepowerandmomentumconstraints</description>
	 */
	typedef struct tagcalculatepowerandmomentumconstraints_struct
	{
		RTS_IEC_LREAL userDefineOverride;	/* VAR_INPUT */	/* user define override; */
		RTS_IEC_LREAL jointVoltages[6];		/* VAR_INPUT */	/* actual joint voltage(v); */
		RTS_IEC_BOOL calculatePowerAndMomentumConstraints;	/* VAR_OUTPUT */
		RTS_IEC_BOOL constraintActive;		/* VAR_OUTPUT */	/* the status OF constraint; */
		RTS_IEC_LREAL jointVelFactorConstraint;	/* VAR_OUTPUT */	/* constraint factor FOR joint velocities; */
		RTS_IEC_LREAL jointAccFactorConstraint;	/* VAR_OUTPUT */	/* constraint factor FOR joint accelerations; */
		RTS_IEC_LREAL electircPower;		/* VAR_OUTPUT */	/* P = U * I; */
		RTS_IEC_LREAL physicsPower;			/* VAR_OUTPUT */	/* P = Torque * omega; */
		RTS_IEC_LREAL momentum;				/* VAR_OUTPUT */	/* momemtum = mass * omega; */
	} calculatepowerandmomentumconstraints_struct;

	void CDECL CDECL_EXT calculatepowerandmomentumconstraints(calculatepowerandmomentumconstraints_struct* p);
	typedef void (CDECL CDECL_EXT* PFCALCULATEPOWERANDMOMENTUMCONSTRAINTS_IEC) (calculatepowerandmomentumconstraints_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(CALCULATEPOWERANDMOMENTUMCONSTRAINTS_NOTIMPLEMENTED)
#define USE_calculatepowerandmomentumconstraints
#define EXT_calculatepowerandmomentumconstraints
#define GET_calculatepowerandmomentumconstraints(fl)  ERR_NOTIMPLEMENTED
#define CAL_calculatepowerandmomentumconstraints(p0) 
#define CHK_calculatepowerandmomentumconstraints  FALSE
#define EXP_calculatepowerandmomentumconstraints  ERR_OK
#elif defined(STATIC_LINK)
#define USE_calculatepowerandmomentumconstraints
#define EXT_calculatepowerandmomentumconstraints
#define GET_calculatepowerandmomentumconstraints(fl)  CAL_CMGETAPI( "calculatepowerandmomentumconstraints" ) 
#define CAL_calculatepowerandmomentumconstraints  calculatepowerandmomentumconstraints
#define CHK_calculatepowerandmomentumconstraints  TRUE
#define EXP_calculatepowerandmomentumconstraints  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"calculatepowerandmomentumconstraints", (RTS_UINTPTR)calculatepowerandmomentumconstraints, 1, 0x4FA4BE9E, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_calculatepowerandmomentumconstraints
#define EXT_calculatepowerandmomentumconstraints
#define GET_calculatepowerandmomentumconstraints(fl)  CAL_CMGETAPI( "calculatepowerandmomentumconstraints" ) 
#define CAL_calculatepowerandmomentumconstraints  calculatepowerandmomentumconstraints
#define CHK_calculatepowerandmomentumconstraints  TRUE
#define EXP_calculatepowerandmomentumconstraints  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"calculatepowerandmomentumconstraints", (RTS_UINTPTR)calculatepowerandmomentumconstraints, 1, 0x4FA4BE9E, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibcalculatepowerandmomentumconstraints
#define EXT_ElfinCollaborativeLibcalculatepowerandmomentumconstraints
#define GET_ElfinCollaborativeLibcalculatepowerandmomentumconstraints  ERR_OK
#define CAL_ElfinCollaborativeLibcalculatepowerandmomentumconstraints  calculatepowerandmomentumconstraints
#define CHK_ElfinCollaborativeLibcalculatepowerandmomentumconstraints  TRUE
#define EXP_ElfinCollaborativeLibcalculatepowerandmomentumconstraints  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"calculatepowerandmomentumconstraints", (RTS_UINTPTR)calculatepowerandmomentumconstraints, 1, 0x4FA4BE9E, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_calculatepowerandmomentumconstraints
#define EXT_calculatepowerandmomentumconstraints
#define GET_calculatepowerandmomentumconstraints(fl)  CAL_CMGETAPI( "calculatepowerandmomentumconstraints" ) 
#define CAL_calculatepowerandmomentumconstraints  calculatepowerandmomentumconstraints
#define CHK_calculatepowerandmomentumconstraints  TRUE
#define EXP_calculatepowerandmomentumconstraints  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"calculatepowerandmomentumconstraints", (RTS_UINTPTR)calculatepowerandmomentumconstraints, 1, 0x4FA4BE9E, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_calculatepowerandmomentumconstraints  PFCALCULATEPOWERANDMOMENTUMCONSTRAINTS_IEC pfcalculatepowerandmomentumconstraints;
#define EXT_calculatepowerandmomentumconstraints  extern PFCALCULATEPOWERANDMOMENTUMCONSTRAINTS_IEC pfcalculatepowerandmomentumconstraints;
#define GET_calculatepowerandmomentumconstraints(fl)  s_pfCMGetAPI2( "calculatepowerandmomentumconstraints", (RTS_VOID_FCTPTR *)&pfcalculatepowerandmomentumconstraints, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x4FA4BE9E, 0x01000000)
#define CAL_calculatepowerandmomentumconstraints  pfcalculatepowerandmomentumconstraints
#define CHK_calculatepowerandmomentumconstraints  (pfcalculatepowerandmomentumconstraints != NULL)
#define EXP_calculatepowerandmomentumconstraints   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"calculatepowerandmomentumconstraints", (RTS_UINTPTR)calculatepowerandmomentumconstraints, 1, 0x4FA4BE9E, 0x01000000) 
#endif


	/**
	 * Checks for collisions based on the estimated manipulator state and collision stop thesholds for joint disturbance torque.
	 * This method calculates the current disturbance torques on the joints and returns the status of collision.
	 *          Call this method to get the collision status for each joint.
	 *          If this method returns False or any of the collision stop flags are True then the robot should be stopped.
	 * VAR_IN_OUT [out] jointCollisionStatus, A vector of boolean values which will be True if the joint at that index has a disturbance torque exceeding the collision threshold.
	 * return True if the collision check is successful and False if the class has not been properly intialized or the check could not be completed.
	 */
	typedef struct tagcheckforcollision_struct
	{
		RTS_IEC_BOOL checkForCollision;		/* VAR_OUTPUT */
		RTS_IEC_BOOL jointCollisionStatus[6];	/* VAR_OUTPUT */
	} checkforcollision_struct;

	void CDECL CDECL_EXT checkforcollision(checkforcollision_struct* p);
	typedef void (CDECL CDECL_EXT* PFCHECKFORCOLLISION_IEC) (checkforcollision_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(CHECKFORCOLLISION_NOTIMPLEMENTED)
#define USE_checkforcollision
#define EXT_checkforcollision
#define GET_checkforcollision(fl)  ERR_NOTIMPLEMENTED
#define CAL_checkforcollision(p0) 
#define CHK_checkforcollision  FALSE
#define EXP_checkforcollision  ERR_OK
#elif defined(STATIC_LINK)
#define USE_checkforcollision
#define EXT_checkforcollision
#define GET_checkforcollision(fl)  CAL_CMGETAPI( "checkforcollision" ) 
#define CAL_checkforcollision  checkforcollision
#define CHK_checkforcollision  TRUE
#define EXP_checkforcollision  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"checkforcollision", (RTS_UINTPTR)checkforcollision, 1, 0xE9568F15, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_checkforcollision
#define EXT_checkforcollision
#define GET_checkforcollision(fl)  CAL_CMGETAPI( "checkforcollision" ) 
#define CAL_checkforcollision  checkforcollision
#define CHK_checkforcollision  TRUE
#define EXP_checkforcollision  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"checkforcollision", (RTS_UINTPTR)checkforcollision, 1, 0xE9568F15, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibcheckforcollision
#define EXT_ElfinCollaborativeLibcheckforcollision
#define GET_ElfinCollaborativeLibcheckforcollision  ERR_OK
#define CAL_ElfinCollaborativeLibcheckforcollision  checkforcollision
#define CHK_ElfinCollaborativeLibcheckforcollision  TRUE
#define EXP_ElfinCollaborativeLibcheckforcollision  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"checkforcollision", (RTS_UINTPTR)checkforcollision, 1, 0xE9568F15, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_checkforcollision
#define EXT_checkforcollision
#define GET_checkforcollision(fl)  CAL_CMGETAPI( "checkforcollision" ) 
#define CAL_checkforcollision  checkforcollision
#define CHK_checkforcollision  TRUE
#define EXP_checkforcollision  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"checkforcollision", (RTS_UINTPTR)checkforcollision, 1, 0xE9568F15, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_checkforcollision  PFCHECKFORCOLLISION_IEC pfcheckforcollision;
#define EXT_checkforcollision  extern PFCHECKFORCOLLISION_IEC pfcheckforcollision;
#define GET_checkforcollision(fl)  s_pfCMGetAPI2( "checkforcollision", (RTS_VOID_FCTPTR *)&pfcheckforcollision, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xE9568F15, 0x01000000)
#define CAL_checkforcollision  pfcheckforcollision
#define CHK_checkforcollision  (pfcheckforcollision != NULL)
#define EXP_checkforcollision   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"checkforcollision", (RTS_UINTPTR)checkforcollision, 1, 0xE9568F15, 0x01000000) 
#endif


<<<<<<< HEAD
/**
 * and check for excessive joint disturbance torques.
 *          along with the results from a collision check against the assitive mode collision stop thresholds.
 *          Call this method when in assistive mode to retrieve the proper joint motor current (amps) commands.
 *          If this method returns False or any of the collision stop flags are True then the robot should be stopped.
 *          and the assistive mode collision stop threholds are defined by “assistiveModeCollisionStopThresholds”.
 * VAR_IN_OUT [out] motorCurrentCommands, the joint motor current commands (amps)
 * VAR_IN_OUT [out] jointCollisionStatus, the collision status flags for the joints based on the assistive mode collision stop thresholds 
 * or joints' allow limit, motors' allow currents .
 * @return 0. the velocities and collision check has no problem;
 *  		   1. the class has not been properly intialized or the check could not be completed.;
 *  		   2. over the joints' allow limit error which is setting by "setCollaborativeJointSpaceLimits";
 *			   3. over the motors' allow currents error which is setting by "setActutorMaxCurrentLimits";
 *			   4. robot state error which might caused by wrong payload parameters or encoder home setting; To activate this
 *          		 feature, call "setStartAssistiveMode" only once before start Assistive mode every times.
 *
 *			   5. robot is in steady state, which is only used in collision drive back mode. and it means the robot can be switch to 
 *					position control mode.
 *			   6. in sever collision state , which is only used in collision drive back mode. and it means robot must to switch to 
 *					disable status;  
 *			   7. finshedReadyToClose state, it means the robot can be switch to position control mode.
 */
typedef struct taggetassistivemodemotorcurrentcommands_struct
{
	RTS_IEC_INT getAssistiveModeMotorCurrentCommands;	/* VAR_OUTPUT */	
	RTS_IEC_LREAL motorCurrentCommands[6];	/* VAR_OUTPUT */	
	RTS_IEC_BOOL jointCollisionStatus[6];	/* VAR_OUTPUT */	
} getassistivemodemotorcurrentcommands_struct;
=======
	/**
	 * 是否使用双编增强零力示教性能
	 */
	typedef struct tagenableaugmentedassistivemodeindualencoder_struct
	{
		RTS_IEC_BOOL enable;				/* VAR_INPUT */	/* true:开启,需要模组具备输入和输出侧编码器； */
		RTS_IEC_BOOL enableAugmentedAssistiveModeInDualEncoder;	/* VAR_OUTPUT */
	} enableaugmentedassistivemodeindualencoder_struct;

	void CDECL CDECL_EXT enableaugmentedassistivemodeindualencoder(enableaugmentedassistivemodeindualencoder_struct* p);
	typedef void (CDECL CDECL_EXT* PFENABLEAUGMENTEDASSISTIVEMODEINDUALENCODER_IEC) (enableaugmentedassistivemodeindualencoder_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(ENABLEAUGMENTEDASSISTIVEMODEINDUALENCODER_NOTIMPLEMENTED)
#define USE_enableaugmentedassistivemodeindualencoder
#define EXT_enableaugmentedassistivemodeindualencoder
#define GET_enableaugmentedassistivemodeindualencoder(fl)  ERR_NOTIMPLEMENTED
#define CAL_enableaugmentedassistivemodeindualencoder(p0) 
#define CHK_enableaugmentedassistivemodeindualencoder  FALSE
#define EXP_enableaugmentedassistivemodeindualencoder  ERR_OK
#elif defined(STATIC_LINK)
#define USE_enableaugmentedassistivemodeindualencoder
#define EXT_enableaugmentedassistivemodeindualencoder
#define GET_enableaugmentedassistivemodeindualencoder(fl)  CAL_CMGETAPI( "enableaugmentedassistivemodeindualencoder" ) 
#define CAL_enableaugmentedassistivemodeindualencoder  enableaugmentedassistivemodeindualencoder
#define CHK_enableaugmentedassistivemodeindualencoder  TRUE
#define EXP_enableaugmentedassistivemodeindualencoder  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"enableaugmentedassistivemodeindualencoder", (RTS_UINTPTR)enableaugmentedassistivemodeindualencoder, 1, 0x374FB9CC, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_enableaugmentedassistivemodeindualencoder
#define EXT_enableaugmentedassistivemodeindualencoder
#define GET_enableaugmentedassistivemodeindualencoder(fl)  CAL_CMGETAPI( "enableaugmentedassistivemodeindualencoder" ) 
#define CAL_enableaugmentedassistivemodeindualencoder  enableaugmentedassistivemodeindualencoder
#define CHK_enableaugmentedassistivemodeindualencoder  TRUE
#define EXP_enableaugmentedassistivemodeindualencoder  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"enableaugmentedassistivemodeindualencoder", (RTS_UINTPTR)enableaugmentedassistivemodeindualencoder, 1, 0x374FB9CC, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibenableaugmentedassistivemodeindualencoder
#define EXT_ElfinCollaborativeLibenableaugmentedassistivemodeindualencoder
#define GET_ElfinCollaborativeLibenableaugmentedassistivemodeindualencoder  ERR_OK
#define CAL_ElfinCollaborativeLibenableaugmentedassistivemodeindualencoder  enableaugmentedassistivemodeindualencoder
#define CHK_ElfinCollaborativeLibenableaugmentedassistivemodeindualencoder  TRUE
#define EXP_ElfinCollaborativeLibenableaugmentedassistivemodeindualencoder  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"enableaugmentedassistivemodeindualencoder", (RTS_UINTPTR)enableaugmentedassistivemodeindualencoder, 1, 0x374FB9CC, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_enableaugmentedassistivemodeindualencoder
#define EXT_enableaugmentedassistivemodeindualencoder
#define GET_enableaugmentedassistivemodeindualencoder(fl)  CAL_CMGETAPI( "enableaugmentedassistivemodeindualencoder" ) 
#define CAL_enableaugmentedassistivemodeindualencoder  enableaugmentedassistivemodeindualencoder
#define CHK_enableaugmentedassistivemodeindualencoder  TRUE
#define EXP_enableaugmentedassistivemodeindualencoder  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"enableaugmentedassistivemodeindualencoder", (RTS_UINTPTR)enableaugmentedassistivemodeindualencoder, 1, 0x374FB9CC, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_enableaugmentedassistivemodeindualencoder  PFENABLEAUGMENTEDASSISTIVEMODEINDUALENCODER_IEC pfenableaugmentedassistivemodeindualencoder;
#define EXT_enableaugmentedassistivemodeindualencoder  extern PFENABLEAUGMENTEDASSISTIVEMODEINDUALENCODER_IEC pfenableaugmentedassistivemodeindualencoder;
#define GET_enableaugmentedassistivemodeindualencoder(fl)  s_pfCMGetAPI2( "enableaugmentedassistivemodeindualencoder", (RTS_VOID_FCTPTR *)&pfenableaugmentedassistivemodeindualencoder, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x374FB9CC, 0x01000000)
#define CAL_enableaugmentedassistivemodeindualencoder  pfenableaugmentedassistivemodeindualencoder
#define CHK_enableaugmentedassistivemodeindualencoder  (pfenableaugmentedassistivemodeindualencoder != NULL)
#define EXP_enableaugmentedassistivemodeindualencoder   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"enableaugmentedassistivemodeindualencoder", (RTS_UINTPTR)enableaugmentedassistivemodeindualencoder, 1, 0x374FB9CC, 0x01000000) 
#endif

>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117

	/**
	 * and check for excessive joint disturbance torques.
	 *          along with the results from a collision check against the assitive mode collision stop thresholds.
	 *          Call this method when in assistive mode to retrieve the proper joint motor current (amps) commands.
	 *          If this method returns False or any of the collision stop flags are True then the robot should be stopped.
	 *          and the assistive mode collision stop threholds are defined by “assistiveModeCollisionStopThresholds”.
	 * VAR_IN_OUT [out] motorCurrentCommands, the joint motor current commands (amps)
	 * VAR_IN_OUT [out] jointCollisionStatus, the collision status flags for the joints based on the assistive mode collision stop thresholds
	 * or joints' allow limit, motors' allow currents .
	 * @return 0. the velocities and collision check has no problem;
	 *  		   1. the class has not been properly intialized or the check could not be completed.;
	 *  		   2. over the joints' allow limit error which is setting by "setCollaborativeJointSpaceLimits";
	 *			   3. over the motors' allow currents error which is setting by "setActutorMaxCurrentLimits";
	 *			   4. robot state error which might caused by wrong payload parameters or encoder home setting; To activate this
	 *          		 feature, call "setStartAssistiveMode" only once before start Assistive mode every times.
	 *
	 *			   5. robot is in steady state, which is only used in collision drive back mode. and it means the robot can be switch to
	 *					position control mode.
	 *			   6. in sever collision state , which is only used in collision drive back mode. and it means robot must to switch to
	 *					disable status;
	 *			   7. finshedReadyToClose state, it means the robot can be switch to position control mode.
	 */
	typedef struct taggetassistivemodemotorcurrentcommands_struct
	{
		RTS_IEC_INT getAssistiveModeMotorCurrentCommands;	/* VAR_OUTPUT */
		RTS_IEC_LREAL motorCurrentCommands[6];	/* VAR_OUTPUT */
		RTS_IEC_BOOL jointCollisionStatus[6];	/* VAR_OUTPUT */
	} getassistivemodemotorcurrentcommands_struct;

	void CDECL CDECL_EXT getassistivemodemotorcurrentcommands(getassistivemodemotorcurrentcommands_struct* p);
	typedef void (CDECL CDECL_EXT* PFGETASSISTIVEMODEMOTORCURRENTCOMMANDS_IEC) (getassistivemodemotorcurrentcommands_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(GETASSISTIVEMODEMOTORCURRENTCOMMANDS_NOTIMPLEMENTED)
#define USE_getassistivemodemotorcurrentcommands
#define EXT_getassistivemodemotorcurrentcommands
#define GET_getassistivemodemotorcurrentcommands(fl)  ERR_NOTIMPLEMENTED
#define CAL_getassistivemodemotorcurrentcommands(p0) 
#define CHK_getassistivemodemotorcurrentcommands  FALSE
#define EXP_getassistivemodemotorcurrentcommands  ERR_OK
#elif defined(STATIC_LINK)
#define USE_getassistivemodemotorcurrentcommands
#define EXT_getassistivemodemotorcurrentcommands
#define GET_getassistivemodemotorcurrentcommands(fl)  CAL_CMGETAPI( "getassistivemodemotorcurrentcommands" ) 
#define CAL_getassistivemodemotorcurrentcommands  getassistivemodemotorcurrentcommands
#define CHK_getassistivemodemotorcurrentcommands  TRUE
#define EXP_getassistivemodemotorcurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getassistivemodemotorcurrentcommands", (RTS_UINTPTR)getassistivemodemotorcurrentcommands, 1, 0x35D3E6EC, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_getassistivemodemotorcurrentcommands
#define EXT_getassistivemodemotorcurrentcommands
#define GET_getassistivemodemotorcurrentcommands(fl)  CAL_CMGETAPI( "getassistivemodemotorcurrentcommands" ) 
#define CAL_getassistivemodemotorcurrentcommands  getassistivemodemotorcurrentcommands
#define CHK_getassistivemodemotorcurrentcommands  TRUE
#define EXP_getassistivemodemotorcurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getassistivemodemotorcurrentcommands", (RTS_UINTPTR)getassistivemodemotorcurrentcommands, 1, 0x35D3E6EC, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibgetassistivemodemotorcurrentcommands
#define EXT_ElfinCollaborativeLibgetassistivemodemotorcurrentcommands
#define GET_ElfinCollaborativeLibgetassistivemodemotorcurrentcommands  ERR_OK
#define CAL_ElfinCollaborativeLibgetassistivemodemotorcurrentcommands  getassistivemodemotorcurrentcommands
#define CHK_ElfinCollaborativeLibgetassistivemodemotorcurrentcommands  TRUE
#define EXP_ElfinCollaborativeLibgetassistivemodemotorcurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getassistivemodemotorcurrentcommands", (RTS_UINTPTR)getassistivemodemotorcurrentcommands, 1, 0x35D3E6EC, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_getassistivemodemotorcurrentcommands
#define EXT_getassistivemodemotorcurrentcommands
#define GET_getassistivemodemotorcurrentcommands(fl)  CAL_CMGETAPI( "getassistivemodemotorcurrentcommands" ) 
#define CAL_getassistivemodemotorcurrentcommands  getassistivemodemotorcurrentcommands
#define CHK_getassistivemodemotorcurrentcommands  TRUE
#define EXP_getassistivemodemotorcurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getassistivemodemotorcurrentcommands", (RTS_UINTPTR)getassistivemodemotorcurrentcommands, 1, 0x35D3E6EC, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_getassistivemodemotorcurrentcommands  PFGETASSISTIVEMODEMOTORCURRENTCOMMANDS_IEC pfgetassistivemodemotorcurrentcommands;
#define EXT_getassistivemodemotorcurrentcommands  extern PFGETASSISTIVEMODEMOTORCURRENTCOMMANDS_IEC pfgetassistivemodemotorcurrentcommands;
#define GET_getassistivemodemotorcurrentcommands(fl)  s_pfCMGetAPI2( "getassistivemodemotorcurrentcommands", (RTS_VOID_FCTPTR *)&pfgetassistivemodemotorcurrentcommands, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x35D3E6EC, 0x01000000)
#define CAL_getassistivemodemotorcurrentcommands  pfgetassistivemodemotorcurrentcommands
#define CHK_getassistivemodemotorcurrentcommands  (pfgetassistivemodemotorcurrentcommands != NULL)
#define EXP_getassistivemodemotorcurrentcommands   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getassistivemodemotorcurrentcommands", (RTS_UINTPTR)getassistivemodemotorcurrentcommands, 1, 0x35D3E6EC, 0x01000000) 
#endif


	/**
	 * get feed forward joint current command；
	 */
	typedef struct taggetcomputetorquecurrentcommands_struct
	{
		RTS_IEC_LREAL compensateRatio;		/* VAR_INPUT */	/* compensate percentage. */
		RTS_IEC_LREAL jointPosition[6];		/* VAR_INPUT */	/* joint command position */
		RTS_IEC_LREAL jointVelocity[6];		/* VAR_INPUT */	/* joint command velocity */
		RTS_IEC_LREAL jointAcceleratrion[6];	/* VAR_INPUT */	/* joint command acceleration */
		RTS_IEC_BOOL getComputeTorqueCurrentCommands;	/* VAR_OUTPUT */
		RTS_IEC_LREAL motorCurrentCommands[6];	/* VAR_OUTPUT */	/* feedford currents(A). */
	} getcomputetorquecurrentcommands_struct;

	void CDECL CDECL_EXT getcomputetorquecurrentcommands(getcomputetorquecurrentcommands_struct* p);
	typedef void (CDECL CDECL_EXT* PFGETCOMPUTETORQUECURRENTCOMMANDS_IEC) (getcomputetorquecurrentcommands_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(GETCOMPUTETORQUECURRENTCOMMANDS_NOTIMPLEMENTED)
#define USE_getcomputetorquecurrentcommands
#define EXT_getcomputetorquecurrentcommands
#define GET_getcomputetorquecurrentcommands(fl)  ERR_NOTIMPLEMENTED
#define CAL_getcomputetorquecurrentcommands(p0) 
#define CHK_getcomputetorquecurrentcommands  FALSE
#define EXP_getcomputetorquecurrentcommands  ERR_OK
#elif defined(STATIC_LINK)
#define USE_getcomputetorquecurrentcommands
#define EXT_getcomputetorquecurrentcommands
#define GET_getcomputetorquecurrentcommands(fl)  CAL_CMGETAPI( "getcomputetorquecurrentcommands" ) 
#define CAL_getcomputetorquecurrentcommands  getcomputetorquecurrentcommands
#define CHK_getcomputetorquecurrentcommands  TRUE
#define EXP_getcomputetorquecurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getcomputetorquecurrentcommands", (RTS_UINTPTR)getcomputetorquecurrentcommands, 1, 0x047F1D78, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_getcomputetorquecurrentcommands
#define EXT_getcomputetorquecurrentcommands
#define GET_getcomputetorquecurrentcommands(fl)  CAL_CMGETAPI( "getcomputetorquecurrentcommands" ) 
#define CAL_getcomputetorquecurrentcommands  getcomputetorquecurrentcommands
#define CHK_getcomputetorquecurrentcommands  TRUE
#define EXP_getcomputetorquecurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getcomputetorquecurrentcommands", (RTS_UINTPTR)getcomputetorquecurrentcommands, 1, 0x047F1D78, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibgetcomputetorquecurrentcommands
#define EXT_ElfinCollaborativeLibgetcomputetorquecurrentcommands
#define GET_ElfinCollaborativeLibgetcomputetorquecurrentcommands  ERR_OK
#define CAL_ElfinCollaborativeLibgetcomputetorquecurrentcommands  getcomputetorquecurrentcommands
#define CHK_ElfinCollaborativeLibgetcomputetorquecurrentcommands  TRUE
#define EXP_ElfinCollaborativeLibgetcomputetorquecurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getcomputetorquecurrentcommands", (RTS_UINTPTR)getcomputetorquecurrentcommands, 1, 0x047F1D78, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_getcomputetorquecurrentcommands
#define EXT_getcomputetorquecurrentcommands
#define GET_getcomputetorquecurrentcommands(fl)  CAL_CMGETAPI( "getcomputetorquecurrentcommands" ) 
#define CAL_getcomputetorquecurrentcommands  getcomputetorquecurrentcommands
#define CHK_getcomputetorquecurrentcommands  TRUE
#define EXP_getcomputetorquecurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getcomputetorquecurrentcommands", (RTS_UINTPTR)getcomputetorquecurrentcommands, 1, 0x047F1D78, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_getcomputetorquecurrentcommands  PFGETCOMPUTETORQUECURRENTCOMMANDS_IEC pfgetcomputetorquecurrentcommands;
#define EXT_getcomputetorquecurrentcommands  extern PFGETCOMPUTETORQUECURRENTCOMMANDS_IEC pfgetcomputetorquecurrentcommands;
#define GET_getcomputetorquecurrentcommands(fl)  s_pfCMGetAPI2( "getcomputetorquecurrentcommands", (RTS_VOID_FCTPTR *)&pfgetcomputetorquecurrentcommands, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x047F1D78, 0x01000000)
#define CAL_getcomputetorquecurrentcommands  pfgetcomputetorquecurrentcommands
#define CHK_getcomputetorquecurrentcommands  (pfgetcomputetorquecurrentcommands != NULL)
#define EXP_getcomputetorquecurrentcommands   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getcomputetorquecurrentcommands", (RTS_UINTPTR)getcomputetorquecurrentcommands, 1, 0x047F1D78, 0x01000000) 
#endif


	/**
	 * @brief Get the estimated state of the manipulator joints
	 * VAR_IN_OUT [out] jointPositions The estimated joint positions (radians)
	 * VAR_IN_OUT [out] jointVelocities The estimated joint velocities (radians/second)
	 * VAR_IN_OUT [out] jointAccelerations The estimated joint accelerations (radians/second^2)
	 * VAR_IN_OUT[out] motorCurrents The filtered joint motor currents (amperes)
	 * VAR_IN_OUT [out] sensedTorques The sensed joint torques (Newton * meters) calculaed by (motor current * torque constant * gear ratio * max efficiency)
	 * VAR_IN_OUT [out] estimatedTorques The estimated joint torques (Newton * meters) based on models of the manipulator and joints.
	 * VAR_IN_OUT [out] disturbanceTorques The distrubance joint torques (Newton * meters) calculated by the difference between sensed and estimated joint torques.
	 * @return True if successful and False if the class has not been properly intialized
	 */
	typedef struct taggetestimatedstate_struct
	{
		RTS_IEC_BOOL getEstimatedState;		/* VAR_OUTPUT */
		RTS_IEC_LREAL jointPostions[6];		/* VAR_OUTPUT */
		RTS_IEC_LREAL jointVelocities[6];	/* VAR_OUTPUT */
		RTS_IEC_LREAL jointAccelerations[6];	/* VAR_OUTPUT */
		RTS_IEC_LREAL motorCurrents[6];		/* VAR_OUTPUT */
		RTS_IEC_LREAL sensedTorques[6];		/* VAR_OUTPUT */
		RTS_IEC_LREAL estimatedTorques[6];	/* VAR_OUTPUT */
		RTS_IEC_LREAL disturbanceTorque[6];	/* VAR_OUTPUT */
	} getestimatedstate_struct;

	void CDECL CDECL_EXT getestimatedstate(getestimatedstate_struct* p);
	typedef void (CDECL CDECL_EXT* PFGETESTIMATEDSTATE_IEC) (getestimatedstate_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(GETESTIMATEDSTATE_NOTIMPLEMENTED)
#define USE_getestimatedstate
#define EXT_getestimatedstate
#define GET_getestimatedstate(fl)  ERR_NOTIMPLEMENTED
#define CAL_getestimatedstate(p0) 
#define CHK_getestimatedstate  FALSE
#define EXP_getestimatedstate  ERR_OK
#elif defined(STATIC_LINK)
#define USE_getestimatedstate
#define EXT_getestimatedstate
#define GET_getestimatedstate(fl)  CAL_CMGETAPI( "getestimatedstate" ) 
#define CAL_getestimatedstate  getestimatedstate
#define CHK_getestimatedstate  TRUE
#define EXP_getestimatedstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getestimatedstate", (RTS_UINTPTR)getestimatedstate, 1, 0x49A2CB2A, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_getestimatedstate
#define EXT_getestimatedstate
#define GET_getestimatedstate(fl)  CAL_CMGETAPI( "getestimatedstate" ) 
#define CAL_getestimatedstate  getestimatedstate
#define CHK_getestimatedstate  TRUE
#define EXP_getestimatedstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getestimatedstate", (RTS_UINTPTR)getestimatedstate, 1, 0x49A2CB2A, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibgetestimatedstate
#define EXT_ElfinCollaborativeLibgetestimatedstate
#define GET_ElfinCollaborativeLibgetestimatedstate  ERR_OK
#define CAL_ElfinCollaborativeLibgetestimatedstate  getestimatedstate
#define CHK_ElfinCollaborativeLibgetestimatedstate  TRUE
#define EXP_ElfinCollaborativeLibgetestimatedstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getestimatedstate", (RTS_UINTPTR)getestimatedstate, 1, 0x49A2CB2A, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_getestimatedstate
#define EXT_getestimatedstate
#define GET_getestimatedstate(fl)  CAL_CMGETAPI( "getestimatedstate" ) 
#define CAL_getestimatedstate  getestimatedstate
#define CHK_getestimatedstate  TRUE
#define EXP_getestimatedstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getestimatedstate", (RTS_UINTPTR)getestimatedstate, 1, 0x49A2CB2A, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_getestimatedstate  PFGETESTIMATEDSTATE_IEC pfgetestimatedstate;
#define EXT_getestimatedstate  extern PFGETESTIMATEDSTATE_IEC pfgetestimatedstate;
#define GET_getestimatedstate(fl)  s_pfCMGetAPI2( "getestimatedstate", (RTS_VOID_FCTPTR *)&pfgetestimatedstate, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x49A2CB2A, 0x01000000)
#define CAL_getestimatedstate  pfgetestimatedstate
#define CHK_getestimatedstate  (pfgetestimatedstate != NULL)
#define EXP_getestimatedstate   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getestimatedstate", (RTS_UINTPTR)getestimatedstate, 1, 0x49A2CB2A, 0x01000000) 
#endif


<<<<<<< HEAD
/**
 * get flexible compensate Torque for motion precision
 */
typedef struct taggetflexiblecompensatetorque_struct
{
	RTS_IEC_LREAL jointPosition[6];		/* VAR_INPUT */	/* joint command position */
	RTS_IEC_LREAL jointVelocity[6];		/* VAR_INPUT */	/* joint command velocity */
	RTS_IEC_LREAL jointAcceleratrion[6];	/* VAR_INPUT */	/* joint command acceleration */
	RTS_IEC_BOOL getFlexibleCompensateTorque;	/* VAR_OUTPUT */	
	RTS_IEC_LREAL flexibleCompensateTorque[6];	/* VAR_OUTPUT */	/* torque(Nm). */
} getflexiblecompensatetorque_struct;

void CDECL CDECL_EXT getflexiblecompensatetorque(getflexiblecompensatetorque_struct *p);
typedef void (CDECL CDECL_EXT* PFGETFLEXIBLECOMPENSATETORQUE_IEC) (getflexiblecompensatetorque_struct *p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(GETFLEXIBLECOMPENSATETORQUE_NOTIMPLEMENTED)
	#define USE_getflexiblecompensatetorque
	#define EXT_getflexiblecompensatetorque
	#define GET_getflexiblecompensatetorque(fl)  ERR_NOTIMPLEMENTED
	#define CAL_getflexiblecompensatetorque(p0) 
	#define CHK_getflexiblecompensatetorque  FALSE
	#define EXP_getflexiblecompensatetorque  ERR_OK
#elif defined(STATIC_LINK)
	#define USE_getflexiblecompensatetorque
	#define EXT_getflexiblecompensatetorque
	#define GET_getflexiblecompensatetorque(fl)  CAL_CMGETAPI( "getflexiblecompensatetorque" ) 
	#define CAL_getflexiblecompensatetorque  getflexiblecompensatetorque
	#define CHK_getflexiblecompensatetorque  TRUE
	#define EXP_getflexiblecompensatetorque  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getflexiblecompensatetorque", (RTS_UINTPTR)getflexiblecompensatetorque, 1, 0x27D3DE2B, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
	#define USE_getflexiblecompensatetorque
	#define EXT_getflexiblecompensatetorque
	#define GET_getflexiblecompensatetorque(fl)  CAL_CMGETAPI( "getflexiblecompensatetorque" ) 
	#define CAL_getflexiblecompensatetorque  getflexiblecompensatetorque
	#define CHK_getflexiblecompensatetorque  TRUE
	#define EXP_getflexiblecompensatetorque  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getflexiblecompensatetorque", (RTS_UINTPTR)getflexiblecompensatetorque, 1, 0x27D3DE2B, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
	#define USE_ElfinCollaborativeLibgetflexiblecompensatetorque
	#define EXT_ElfinCollaborativeLibgetflexiblecompensatetorque
	#define GET_ElfinCollaborativeLibgetflexiblecompensatetorque  ERR_OK
	#define CAL_ElfinCollaborativeLibgetflexiblecompensatetorque  getflexiblecompensatetorque
	#define CHK_ElfinCollaborativeLibgetflexiblecompensatetorque  TRUE
	#define EXP_ElfinCollaborativeLibgetflexiblecompensatetorque  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getflexiblecompensatetorque", (RTS_UINTPTR)getflexiblecompensatetorque, 1, 0x27D3DE2B, 0x01000000) 
#elif defined(CPLUSPLUS)
	#define USE_getflexiblecompensatetorque
	#define EXT_getflexiblecompensatetorque
	#define GET_getflexiblecompensatetorque(fl)  CAL_CMGETAPI( "getflexiblecompensatetorque" ) 
	#define CAL_getflexiblecompensatetorque  getflexiblecompensatetorque
	#define CHK_getflexiblecompensatetorque  TRUE
	#define EXP_getflexiblecompensatetorque  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getflexiblecompensatetorque", (RTS_UINTPTR)getflexiblecompensatetorque, 1, 0x27D3DE2B, 0x01000000) 
#else /* DYNAMIC_LINK */
	#define USE_getflexiblecompensatetorque  PFGETFLEXIBLECOMPENSATETORQUE_IEC pfgetflexiblecompensatetorque;
	#define EXT_getflexiblecompensatetorque  extern PFGETFLEXIBLECOMPENSATETORQUE_IEC pfgetflexiblecompensatetorque;
	#define GET_getflexiblecompensatetorque(fl)  s_pfCMGetAPI2( "getflexiblecompensatetorque", (RTS_VOID_FCTPTR *)&pfgetflexiblecompensatetorque, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x27D3DE2B, 0x01000000)
	#define CAL_getflexiblecompensatetorque  pfgetflexiblecompensatetorque
	#define CHK_getflexiblecompensatetorque  (pfgetflexiblecompensatetorque != NULL)
	#define EXP_getflexiblecompensatetorque   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getflexiblecompensatetorque", (RTS_UINTPTR)getflexiblecompensatetorque, 1, 0x27D3DE2B, 0x01000000) 
#endif


/**
 * get feedforward current command of gravity;
 */
typedef struct taggetgravitytorquecurrentcommands_struct
{
	RTS_IEC_LREAL compensateRatio;		/* VAR_INPUT */	/* compensate percentage; */
	RTS_IEC_LREAL jointPosition[6];		/* VAR_INPUT */	
	RTS_IEC_BOOL getGravityTorqueCurrentCommands;	/* VAR_OUTPUT */	
	RTS_IEC_LREAL motorCurrentCommands[6];	/* VAR_OUTPUT */	/* joint current (A) */
} getgravitytorquecurrentcommands_struct;

void CDECL CDECL_EXT getgravitytorquecurrentcommands(getgravitytorquecurrentcommands_struct *p);
typedef void (CDECL CDECL_EXT* PFGETGRAVITYTORQUECURRENTCOMMANDS_IEC) (getgravitytorquecurrentcommands_struct *p);
=======
	/**
	 * get flexible compensate Torque for motion precision
	 */
	typedef struct taggetflexiblecompensatetorque_struct
	{
		RTS_IEC_LREAL jointPosition[6];		/* VAR_INPUT */	/* joint command position */
		RTS_IEC_LREAL jointVelocity[6];		/* VAR_INPUT */	/* joint command velocity */
		RTS_IEC_LREAL jointAcceleratrion[6];	/* VAR_INPUT */	/* joint command acceleration */
		RTS_IEC_BOOL getFlexibleCompensateTorque;	/* VAR_OUTPUT */
		RTS_IEC_LREAL flexibleCompensateTorque[6];	/* VAR_OUTPUT */	/* torque(Nm). */
	} getflexiblecompensatetorque_struct;

	void CDECL CDECL_EXT getflexiblecompensatetorque(getflexiblecompensatetorque_struct* p);
	typedef void (CDECL CDECL_EXT* PFGETFLEXIBLECOMPENSATETORQUE_IEC) (getflexiblecompensatetorque_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(GETFLEXIBLECOMPENSATETORQUE_NOTIMPLEMENTED)
#define USE_getflexiblecompensatetorque
#define EXT_getflexiblecompensatetorque
#define GET_getflexiblecompensatetorque(fl)  ERR_NOTIMPLEMENTED
#define CAL_getflexiblecompensatetorque(p0) 
#define CHK_getflexiblecompensatetorque  FALSE
#define EXP_getflexiblecompensatetorque  ERR_OK
#elif defined(STATIC_LINK)
#define USE_getflexiblecompensatetorque
#define EXT_getflexiblecompensatetorque
#define GET_getflexiblecompensatetorque(fl)  CAL_CMGETAPI( "getflexiblecompensatetorque" ) 
#define CAL_getflexiblecompensatetorque  getflexiblecompensatetorque
#define CHK_getflexiblecompensatetorque  TRUE
#define EXP_getflexiblecompensatetorque  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getflexiblecompensatetorque", (RTS_UINTPTR)getflexiblecompensatetorque, 1, 0x27D3DE2B, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_getflexiblecompensatetorque
#define EXT_getflexiblecompensatetorque
#define GET_getflexiblecompensatetorque(fl)  CAL_CMGETAPI( "getflexiblecompensatetorque" ) 
#define CAL_getflexiblecompensatetorque  getflexiblecompensatetorque
#define CHK_getflexiblecompensatetorque  TRUE
#define EXP_getflexiblecompensatetorque  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getflexiblecompensatetorque", (RTS_UINTPTR)getflexiblecompensatetorque, 1, 0x27D3DE2B, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibgetflexiblecompensatetorque
#define EXT_ElfinCollaborativeLibgetflexiblecompensatetorque
#define GET_ElfinCollaborativeLibgetflexiblecompensatetorque  ERR_OK
#define CAL_ElfinCollaborativeLibgetflexiblecompensatetorque  getflexiblecompensatetorque
#define CHK_ElfinCollaborativeLibgetflexiblecompensatetorque  TRUE
#define EXP_ElfinCollaborativeLibgetflexiblecompensatetorque  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getflexiblecompensatetorque", (RTS_UINTPTR)getflexiblecompensatetorque, 1, 0x27D3DE2B, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_getflexiblecompensatetorque
#define EXT_getflexiblecompensatetorque
#define GET_getflexiblecompensatetorque(fl)  CAL_CMGETAPI( "getflexiblecompensatetorque" ) 
#define CAL_getflexiblecompensatetorque  getflexiblecompensatetorque
#define CHK_getflexiblecompensatetorque  TRUE
#define EXP_getflexiblecompensatetorque  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getflexiblecompensatetorque", (RTS_UINTPTR)getflexiblecompensatetorque, 1, 0x27D3DE2B, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_getflexiblecompensatetorque  PFGETFLEXIBLECOMPENSATETORQUE_IEC pfgetflexiblecompensatetorque;
#define EXT_getflexiblecompensatetorque  extern PFGETFLEXIBLECOMPENSATETORQUE_IEC pfgetflexiblecompensatetorque;
#define GET_getflexiblecompensatetorque(fl)  s_pfCMGetAPI2( "getflexiblecompensatetorque", (RTS_VOID_FCTPTR *)&pfgetflexiblecompensatetorque, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x27D3DE2B, 0x01000000)
#define CAL_getflexiblecompensatetorque  pfgetflexiblecompensatetorque
#define CHK_getflexiblecompensatetorque  (pfgetflexiblecompensatetorque != NULL)
#define EXP_getflexiblecompensatetorque   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getflexiblecompensatetorque", (RTS_UINTPTR)getflexiblecompensatetorque, 1, 0x27D3DE2B, 0x01000000) 
#endif


	/**
	 * get feedforward current command of gravity;
	 */
	typedef struct taggetgravitytorquecurrentcommands_struct
	{
		RTS_IEC_LREAL compensateRatio;		/* VAR_INPUT */	/* compensate percentage; */
		RTS_IEC_LREAL jointPosition[6];		/* VAR_INPUT */
		RTS_IEC_BOOL getGravityTorqueCurrentCommands;	/* VAR_OUTPUT */
		RTS_IEC_LREAL motorCurrentCommands[6];	/* VAR_OUTPUT */	/* joint current (A) */
	} getgravitytorquecurrentcommands_struct;

	void CDECL CDECL_EXT getgravitytorquecurrentcommands(getgravitytorquecurrentcommands_struct* p);
	typedef void (CDECL CDECL_EXT* PFGETGRAVITYTORQUECURRENTCOMMANDS_IEC) (getgravitytorquecurrentcommands_struct* p);
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(GETGRAVITYTORQUECURRENTCOMMANDS_NOTIMPLEMENTED)
#define USE_getgravitytorquecurrentcommands
#define EXT_getgravitytorquecurrentcommands
#define GET_getgravitytorquecurrentcommands(fl)  ERR_NOTIMPLEMENTED
#define CAL_getgravitytorquecurrentcommands(p0) 
#define CHK_getgravitytorquecurrentcommands  FALSE
#define EXP_getgravitytorquecurrentcommands  ERR_OK
#elif defined(STATIC_LINK)
#define USE_getgravitytorquecurrentcommands
#define EXT_getgravitytorquecurrentcommands
#define GET_getgravitytorquecurrentcommands(fl)  CAL_CMGETAPI( "getgravitytorquecurrentcommands" ) 
#define CAL_getgravitytorquecurrentcommands  getgravitytorquecurrentcommands
#define CHK_getgravitytorquecurrentcommands  TRUE
#define EXP_getgravitytorquecurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getgravitytorquecurrentcommands", (RTS_UINTPTR)getgravitytorquecurrentcommands, 1, 0x30D8A81C, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_getgravitytorquecurrentcommands
#define EXT_getgravitytorquecurrentcommands
#define GET_getgravitytorquecurrentcommands(fl)  CAL_CMGETAPI( "getgravitytorquecurrentcommands" ) 
#define CAL_getgravitytorquecurrentcommands  getgravitytorquecurrentcommands
#define CHK_getgravitytorquecurrentcommands  TRUE
#define EXP_getgravitytorquecurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getgravitytorquecurrentcommands", (RTS_UINTPTR)getgravitytorquecurrentcommands, 1, 0x30D8A81C, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibgetgravitytorquecurrentcommands
#define EXT_ElfinCollaborativeLibgetgravitytorquecurrentcommands
#define GET_ElfinCollaborativeLibgetgravitytorquecurrentcommands  ERR_OK
#define CAL_ElfinCollaborativeLibgetgravitytorquecurrentcommands  getgravitytorquecurrentcommands
#define CHK_ElfinCollaborativeLibgetgravitytorquecurrentcommands  TRUE
#define EXP_ElfinCollaborativeLibgetgravitytorquecurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getgravitytorquecurrentcommands", (RTS_UINTPTR)getgravitytorquecurrentcommands, 1, 0x30D8A81C, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_getgravitytorquecurrentcommands
#define EXT_getgravitytorquecurrentcommands
#define GET_getgravitytorquecurrentcommands(fl)  CAL_CMGETAPI( "getgravitytorquecurrentcommands" ) 
#define CAL_getgravitytorquecurrentcommands  getgravitytorquecurrentcommands
#define CHK_getgravitytorquecurrentcommands  TRUE
#define EXP_getgravitytorquecurrentcommands  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getgravitytorquecurrentcommands", (RTS_UINTPTR)getgravitytorquecurrentcommands, 1, 0x30D8A81C, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_getgravitytorquecurrentcommands  PFGETGRAVITYTORQUECURRENTCOMMANDS_IEC pfgetgravitytorquecurrentcommands;
#define EXT_getgravitytorquecurrentcommands  extern PFGETGRAVITYTORQUECURRENTCOMMANDS_IEC pfgetgravitytorquecurrentcommands;
#define GET_getgravitytorquecurrentcommands(fl)  s_pfCMGetAPI2( "getgravitytorquecurrentcommands", (RTS_VOID_FCTPTR *)&pfgetgravitytorquecurrentcommands, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x30D8A81C, 0x01000000)
#define CAL_getgravitytorquecurrentcommands  pfgetgravitytorquecurrentcommands
#define CHK_getgravitytorquecurrentcommands  (pfgetgravitytorquecurrentcommands != NULL)
#define EXP_getgravitytorquecurrentcommands   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getgravitytorquecurrentcommands", (RTS_UINTPTR)getgravitytorquecurrentcommands, 1, 0x30D8A81C, 0x01000000) 
#endif


	/**
	 * 获取当前关节的转动惯量；
	 */
	typedef struct taggetinertiaofjoint_struct
	{
		RTS_IEC_LREAL jointPosition[6];		/* VAR_INPUT */	/* joint position,deg; */
		RTS_IEC_BOOL getInertiaofJoint;		/* VAR_OUTPUT */
		RTS_IEC_LREAL jointInertia[6];		/* VAR_OUTPUT */	/* the moment of joint inertia, kg·m^2 */
	} getinertiaofjoint_struct;

	void CDECL CDECL_EXT getinertiaofjoint(getinertiaofjoint_struct* p);
	typedef void (CDECL CDECL_EXT* PFGETINERTIAOFJOINT_IEC) (getinertiaofjoint_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(GETINERTIAOFJOINT_NOTIMPLEMENTED)
#define USE_getinertiaofjoint
#define EXT_getinertiaofjoint
#define GET_getinertiaofjoint(fl)  ERR_NOTIMPLEMENTED
#define CAL_getinertiaofjoint(p0) 
#define CHK_getinertiaofjoint  FALSE
#define EXP_getinertiaofjoint  ERR_OK
#elif defined(STATIC_LINK)
#define USE_getinertiaofjoint
#define EXT_getinertiaofjoint
#define GET_getinertiaofjoint(fl)  CAL_CMGETAPI( "getinertiaofjoint" ) 
#define CAL_getinertiaofjoint  getinertiaofjoint
#define CHK_getinertiaofjoint  TRUE
#define EXP_getinertiaofjoint  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getinertiaofjoint", (RTS_UINTPTR)getinertiaofjoint, 1, 0x1FA1A115, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_getinertiaofjoint
#define EXT_getinertiaofjoint
#define GET_getinertiaofjoint(fl)  CAL_CMGETAPI( "getinertiaofjoint" ) 
#define CAL_getinertiaofjoint  getinertiaofjoint
#define CHK_getinertiaofjoint  TRUE
#define EXP_getinertiaofjoint  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getinertiaofjoint", (RTS_UINTPTR)getinertiaofjoint, 1, 0x1FA1A115, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibgetinertiaofjoint
#define EXT_ElfinCollaborativeLibgetinertiaofjoint
#define GET_ElfinCollaborativeLibgetinertiaofjoint  ERR_OK
#define CAL_ElfinCollaborativeLibgetinertiaofjoint  getinertiaofjoint
#define CHK_ElfinCollaborativeLibgetinertiaofjoint  TRUE
#define EXP_ElfinCollaborativeLibgetinertiaofjoint  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getinertiaofjoint", (RTS_UINTPTR)getinertiaofjoint, 1, 0x1FA1A115, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_getinertiaofjoint
#define EXT_getinertiaofjoint
#define GET_getinertiaofjoint(fl)  CAL_CMGETAPI( "getinertiaofjoint" ) 
#define CAL_getinertiaofjoint  getinertiaofjoint
#define CHK_getinertiaofjoint  TRUE
#define EXP_getinertiaofjoint  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getinertiaofjoint", (RTS_UINTPTR)getinertiaofjoint, 1, 0x1FA1A115, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_getinertiaofjoint  PFGETINERTIAOFJOINT_IEC pfgetinertiaofjoint;
#define EXT_getinertiaofjoint  extern PFGETINERTIAOFJOINT_IEC pfgetinertiaofjoint;
#define GET_getinertiaofjoint(fl)  s_pfCMGetAPI2( "getinertiaofjoint", (RTS_VOID_FCTPTR *)&pfgetinertiaofjoint, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x1FA1A115, 0x01000000)
#define CAL_getinertiaofjoint  pfgetinertiaofjoint
#define CHK_getinertiaofjoint  (pfgetinertiaofjoint != NULL)
#define EXP_getinertiaofjoint   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getinertiaofjoint", (RTS_UINTPTR)getinertiaofjoint, 1, 0x1FA1A115, 0x01000000) 
#endif


	/**
	 * <description>getsettingparameters</description>
	 */
	typedef struct taggetsettingparameters_struct
	{
		RTS_IEC_BOOL getSettingParameters;	/* VAR_OUTPUT */
		cobotParams cobotParams;			/* VAR_OUTPUT */
	} getsettingparameters_struct;

	void CDECL CDECL_EXT getsettingparameters(getsettingparameters_struct* p);
	typedef void (CDECL CDECL_EXT* PFGETSETTINGPARAMETERS_IEC) (getsettingparameters_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(GETSETTINGPARAMETERS_NOTIMPLEMENTED)
#define USE_getsettingparameters
#define EXT_getsettingparameters
#define GET_getsettingparameters(fl)  ERR_NOTIMPLEMENTED
#define CAL_getsettingparameters(p0) 
#define CHK_getsettingparameters  FALSE
#define EXP_getsettingparameters  ERR_OK
#elif defined(STATIC_LINK)
#define USE_getsettingparameters
#define EXT_getsettingparameters
#define GET_getsettingparameters(fl)  CAL_CMGETAPI( "getsettingparameters" ) 
#define CAL_getsettingparameters  getsettingparameters
#define CHK_getsettingparameters  TRUE
#define EXP_getsettingparameters  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getsettingparameters", (RTS_UINTPTR)getsettingparameters, 1, 0x0180F8FE, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_getsettingparameters
#define EXT_getsettingparameters
#define GET_getsettingparameters(fl)  CAL_CMGETAPI( "getsettingparameters" ) 
#define CAL_getsettingparameters  getsettingparameters
#define CHK_getsettingparameters  TRUE
#define EXP_getsettingparameters  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getsettingparameters", (RTS_UINTPTR)getsettingparameters, 1, 0x0180F8FE, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibgetsettingparameters
#define EXT_ElfinCollaborativeLibgetsettingparameters
#define GET_ElfinCollaborativeLibgetsettingparameters  ERR_OK
#define CAL_ElfinCollaborativeLibgetsettingparameters  getsettingparameters
#define CHK_ElfinCollaborativeLibgetsettingparameters  TRUE
#define EXP_ElfinCollaborativeLibgetsettingparameters  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getsettingparameters", (RTS_UINTPTR)getsettingparameters, 1, 0x0180F8FE, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_getsettingparameters
#define EXT_getsettingparameters
#define GET_getsettingparameters(fl)  CAL_CMGETAPI( "getsettingparameters" ) 
#define CAL_getsettingparameters  getsettingparameters
#define CHK_getsettingparameters  TRUE
#define EXP_getsettingparameters  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getsettingparameters", (RTS_UINTPTR)getsettingparameters, 1, 0x0180F8FE, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_getsettingparameters  PFGETSETTINGPARAMETERS_IEC pfgetsettingparameters;
#define EXT_getsettingparameters  extern PFGETSETTINGPARAMETERS_IEC pfgetsettingparameters;
#define GET_getsettingparameters(fl)  s_pfCMGetAPI2( "getsettingparameters", (RTS_VOID_FCTPTR *)&pfgetsettingparameters, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x0180F8FE, 0x01000000)
#define CAL_getsettingparameters  pfgetsettingparameters
#define CHK_getsettingparameters  (pfgetsettingparameters != NULL)
#define EXP_getsettingparameters   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"getsettingparameters", (RTS_UINTPTR)getsettingparameters, 1, 0x0180F8FE, 0x01000000) 
#endif


<<<<<<< HEAD
/**
 * Initialize the objects used for estimating manipulator state and torques
 * VAR_INPUT [in] jointPositions the current joint positions (radians)
 * VAR_INPUT [in] updatePeriod The update period in seconds used for state estimation. (0.001s)
 * VAR_INPUT [in] robotType elfin:0, UR: 1
 * VAR_INPUT [in] kinematics parameters of robot. 
 */
typedef struct taginitializecollaborativealgorithm_struct
{
	RTS_IEC_LREAL jointPosition[6];		/* VAR_INPUT */	
	RTS_IEC_LREAL updatePeriod;			/* VAR_INPUT */	
	RTS_IEC_INT robotType;				/* VAR_INPUT */	/* 0:elfin; 1:UR;, pallet: 5 */
	RTS_IEC_LREAL kinParams[10];		/* VAR_INPUT */	/* elfin[d1,d4,d6,a2]; UR[d1,d2,d3,d4,d5,d6,a2,a3]; */
	RTS_IEC_BOOL initializeCollaborativeAlgorithm;	/* VAR_OUTPUT */	
} initializecollaborativealgorithm_struct;

void CDECL CDECL_EXT initializecollaborativealgorithm(initializecollaborativealgorithm_struct *p);
typedef void (CDECL CDECL_EXT* PFINITIALIZECOLLABORATIVEALGORITHM_IEC) (initializecollaborativealgorithm_struct *p);
=======
	/**
	 * Initialize the objects used for estimating manipulator state and torques
	 * VAR_INPUT [in] jointPositions the current joint positions (radians)
	 * VAR_INPUT [in] updatePeriod The update period in seconds used for state estimation. (0.001s)
	 * VAR_INPUT [in] robotType elfin:0, UR: 1
	 * VAR_INPUT [in] kinematics parameters of robot.
	 */
	typedef struct taginitializecollaborativealgorithm_struct
	{
		RTS_IEC_LREAL jointPosition[6];		/* VAR_INPUT */
		RTS_IEC_LREAL updatePeriod;			/* VAR_INPUT */
		RTS_IEC_INT robotType;				/* VAR_INPUT */	/* 0:elfin; 1:UR;, pallet: 5 */
		RTS_IEC_LREAL kinParams[10];		/* VAR_INPUT */	/* elfin[d1,d4,d6,a2]; UR[d1,d2,d3,d4,d5,d6,a2,a3]; */
		RTS_IEC_BOOL initializeCollaborativeAlgorithm;	/* VAR_OUTPUT */
	} initializecollaborativealgorithm_struct;

	void CDECL CDECL_EXT initializecollaborativealgorithm(initializecollaborativealgorithm_struct* p);
	typedef void (CDECL CDECL_EXT* PFINITIALIZECOLLABORATIVEALGORITHM_IEC) (initializecollaborativealgorithm_struct* p);
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(INITIALIZECOLLABORATIVEALGORITHM_NOTIMPLEMENTED)
#define USE_initializecollaborativealgorithm
#define EXT_initializecollaborativealgorithm
#define GET_initializecollaborativealgorithm(fl)  ERR_NOTIMPLEMENTED
#define CAL_initializecollaborativealgorithm(p0) 
#define CHK_initializecollaborativealgorithm  FALSE
#define EXP_initializecollaborativealgorithm  ERR_OK
#elif defined(STATIC_LINK)
#define USE_initializecollaborativealgorithm
#define EXT_initializecollaborativealgorithm
#define GET_initializecollaborativealgorithm(fl)  CAL_CMGETAPI( "initializecollaborativealgorithm" ) 
#define CAL_initializecollaborativealgorithm  initializecollaborativealgorithm
#define CHK_initializecollaborativealgorithm  TRUE
#define EXP_initializecollaborativealgorithm  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"initializecollaborativealgorithm", (RTS_UINTPTR)initializecollaborativealgorithm, 1, 0xBFF655A5, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_initializecollaborativealgorithm
#define EXT_initializecollaborativealgorithm
#define GET_initializecollaborativealgorithm(fl)  CAL_CMGETAPI( "initializecollaborativealgorithm" ) 
#define CAL_initializecollaborativealgorithm  initializecollaborativealgorithm
#define CHK_initializecollaborativealgorithm  TRUE
#define EXP_initializecollaborativealgorithm  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"initializecollaborativealgorithm", (RTS_UINTPTR)initializecollaborativealgorithm, 1, 0xBFF655A5, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibinitializecollaborativealgorithm
#define EXT_ElfinCollaborativeLibinitializecollaborativealgorithm
#define GET_ElfinCollaborativeLibinitializecollaborativealgorithm  ERR_OK
#define CAL_ElfinCollaborativeLibinitializecollaborativealgorithm  initializecollaborativealgorithm
#define CHK_ElfinCollaborativeLibinitializecollaborativealgorithm  TRUE
#define EXP_ElfinCollaborativeLibinitializecollaborativealgorithm  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"initializecollaborativealgorithm", (RTS_UINTPTR)initializecollaborativealgorithm, 1, 0xBFF655A5, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_initializecollaborativealgorithm
#define EXT_initializecollaborativealgorithm
#define GET_initializecollaborativealgorithm(fl)  CAL_CMGETAPI( "initializecollaborativealgorithm" ) 
#define CAL_initializecollaborativealgorithm  initializecollaborativealgorithm
#define CHK_initializecollaborativealgorithm  TRUE
#define EXP_initializecollaborativealgorithm  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"initializecollaborativealgorithm", (RTS_UINTPTR)initializecollaborativealgorithm, 1, 0xBFF655A5, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_initializecollaborativealgorithm  PFINITIALIZECOLLABORATIVEALGORITHM_IEC pfinitializecollaborativealgorithm;
#define EXT_initializecollaborativealgorithm  extern PFINITIALIZECOLLABORATIVEALGORITHM_IEC pfinitializecollaborativealgorithm;
#define GET_initializecollaborativealgorithm(fl)  s_pfCMGetAPI2( "initializecollaborativealgorithm", (RTS_VOID_FCTPTR *)&pfinitializecollaborativealgorithm, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xBFF655A5, 0x01000000)
#define CAL_initializecollaborativealgorithm  pfinitializecollaborativealgorithm
#define CHK_initializecollaborativealgorithm  (pfinitializecollaborativealgorithm != NULL)
#define EXP_initializecollaborativealgorithm   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"initializecollaborativealgorithm", (RTS_UINTPTR)initializecollaborativealgorithm, 1, 0xBFF655A5, 0x01000000) 
#endif


	/**
	 *
	 *return:
	 *	0. constraint_normal, 机器人的运动处于正常状态
	 *	1. constraint_errorStatus, 处于错误状态，如零力示教时的负载参数错误、过电流等错误；
	 *	2. constraint_overJointVelocitiesLimit, 超出关节最大速度限制；
	 *	3. constraint_overLinearVelocitiesLimit, 超出笛卡尔最大运动速度限制（末端和肘部）；
	 *	4. constraint_overMomentumLimit, 超出本体最大动量限制；
	 *	5. constraint_overPowerLImit， 超出本体最大功率限制；
	 */
	typedef struct tagmonitormotionconstraintstatus_struct
	{
		RTS_IEC_BOOL isAssistiveMode;		/* VAR_INPUT */	/* 是否处于零力示教模式？ */
		RTS_IEC_INT monitorMotionConstraintStatus;	/* VAR_OUTPUT */
	} monitormotionconstraintstatus_struct;

	void CDECL CDECL_EXT monitormotionconstraintstatus(monitormotionconstraintstatus_struct* p);
	typedef void (CDECL CDECL_EXT* PFMONITORMOTIONCONSTRAINTSTATUS_IEC) (monitormotionconstraintstatus_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(MONITORMOTIONCONSTRAINTSTATUS_NOTIMPLEMENTED)
#define USE_monitormotionconstraintstatus
#define EXT_monitormotionconstraintstatus
#define GET_monitormotionconstraintstatus(fl)  ERR_NOTIMPLEMENTED
#define CAL_monitormotionconstraintstatus(p0) 
#define CHK_monitormotionconstraintstatus  FALSE
#define EXP_monitormotionconstraintstatus  ERR_OK
#elif defined(STATIC_LINK)
#define USE_monitormotionconstraintstatus
#define EXT_monitormotionconstraintstatus
#define GET_monitormotionconstraintstatus(fl)  CAL_CMGETAPI( "monitormotionconstraintstatus" ) 
#define CAL_monitormotionconstraintstatus  monitormotionconstraintstatus
#define CHK_monitormotionconstraintstatus  TRUE
#define EXP_monitormotionconstraintstatus  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"monitormotionconstraintstatus", (RTS_UINTPTR)monitormotionconstraintstatus, 1, 0x88062ACE, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_monitormotionconstraintstatus
#define EXT_monitormotionconstraintstatus
#define GET_monitormotionconstraintstatus(fl)  CAL_CMGETAPI( "monitormotionconstraintstatus" ) 
#define CAL_monitormotionconstraintstatus  monitormotionconstraintstatus
#define CHK_monitormotionconstraintstatus  TRUE
#define EXP_monitormotionconstraintstatus  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"monitormotionconstraintstatus", (RTS_UINTPTR)monitormotionconstraintstatus, 1, 0x88062ACE, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibmonitormotionconstraintstatus
#define EXT_ElfinCollaborativeLibmonitormotionconstraintstatus
#define GET_ElfinCollaborativeLibmonitormotionconstraintstatus  ERR_OK
#define CAL_ElfinCollaborativeLibmonitormotionconstraintstatus  monitormotionconstraintstatus
#define CHK_ElfinCollaborativeLibmonitormotionconstraintstatus  TRUE
#define EXP_ElfinCollaborativeLibmonitormotionconstraintstatus  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"monitormotionconstraintstatus", (RTS_UINTPTR)monitormotionconstraintstatus, 1, 0x88062ACE, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_monitormotionconstraintstatus
#define EXT_monitormotionconstraintstatus
#define GET_monitormotionconstraintstatus(fl)  CAL_CMGETAPI( "monitormotionconstraintstatus" ) 
#define CAL_monitormotionconstraintstatus  monitormotionconstraintstatus
#define CHK_monitormotionconstraintstatus  TRUE
#define EXP_monitormotionconstraintstatus  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"monitormotionconstraintstatus", (RTS_UINTPTR)monitormotionconstraintstatus, 1, 0x88062ACE, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_monitormotionconstraintstatus  PFMONITORMOTIONCONSTRAINTSTATUS_IEC pfmonitormotionconstraintstatus;
#define EXT_monitormotionconstraintstatus  extern PFMONITORMOTIONCONSTRAINTSTATUS_IEC pfmonitormotionconstraintstatus;
#define GET_monitormotionconstraintstatus(fl)  s_pfCMGetAPI2( "monitormotionconstraintstatus", (RTS_VOID_FCTPTR *)&pfmonitormotionconstraintstatus, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x88062ACE, 0x01000000)
#define CAL_monitormotionconstraintstatus  pfmonitormotionconstraintstatus
#define CHK_monitormotionconstraintstatus  (pfmonitormotionconstraintstatus != NULL)
#define EXP_monitormotionconstraintstatus   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"monitormotionconstraintstatus", (RTS_UINTPTR)monitormotionconstraintstatus, 1, 0x88062ACE, 0x01000000) 
#endif


	/**
	 * <description>resetcobotstatus</description>
	 */
	typedef struct tagresetcobotstatus_struct
	{
		RTS_IEC_BOOL resetCobotStatus;		/* VAR_OUTPUT */
	} resetcobotstatus_struct;

	void CDECL CDECL_EXT resetcobotstatus(resetcobotstatus_struct* p);
	typedef void (CDECL CDECL_EXT* PFRESETCOBOTSTATUS_IEC) (resetcobotstatus_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(RESETCOBOTSTATUS_NOTIMPLEMENTED)
#define USE_resetcobotstatus
#define EXT_resetcobotstatus
#define GET_resetcobotstatus(fl)  ERR_NOTIMPLEMENTED
#define CAL_resetcobotstatus(p0) 
#define CHK_resetcobotstatus  FALSE
#define EXP_resetcobotstatus  ERR_OK
#elif defined(STATIC_LINK)
#define USE_resetcobotstatus
#define EXT_resetcobotstatus
#define GET_resetcobotstatus(fl)  CAL_CMGETAPI( "resetcobotstatus" ) 
#define CAL_resetcobotstatus  resetcobotstatus
#define CHK_resetcobotstatus  TRUE
#define EXP_resetcobotstatus  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"resetcobotstatus", (RTS_UINTPTR)resetcobotstatus, 1, 0xA7FAE663, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_resetcobotstatus
#define EXT_resetcobotstatus
#define GET_resetcobotstatus(fl)  CAL_CMGETAPI( "resetcobotstatus" ) 
#define CAL_resetcobotstatus  resetcobotstatus
#define CHK_resetcobotstatus  TRUE
#define EXP_resetcobotstatus  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"resetcobotstatus", (RTS_UINTPTR)resetcobotstatus, 1, 0xA7FAE663, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibresetcobotstatus
#define EXT_ElfinCollaborativeLibresetcobotstatus
#define GET_ElfinCollaborativeLibresetcobotstatus  ERR_OK
#define CAL_ElfinCollaborativeLibresetcobotstatus  resetcobotstatus
#define CHK_ElfinCollaborativeLibresetcobotstatus  TRUE
#define EXP_ElfinCollaborativeLibresetcobotstatus  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"resetcobotstatus", (RTS_UINTPTR)resetcobotstatus, 1, 0xA7FAE663, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_resetcobotstatus
#define EXT_resetcobotstatus
#define GET_resetcobotstatus(fl)  CAL_CMGETAPI( "resetcobotstatus" ) 
#define CAL_resetcobotstatus  resetcobotstatus
#define CHK_resetcobotstatus  TRUE
#define EXP_resetcobotstatus  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"resetcobotstatus", (RTS_UINTPTR)resetcobotstatus, 1, 0xA7FAE663, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_resetcobotstatus  PFRESETCOBOTSTATUS_IEC pfresetcobotstatus;
#define EXT_resetcobotstatus  extern PFRESETCOBOTSTATUS_IEC pfresetcobotstatus;
#define GET_resetcobotstatus(fl)  s_pfCMGetAPI2( "resetcobotstatus", (RTS_VOID_FCTPTR *)&pfresetcobotstatus, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xA7FAE663, 0x01000000)
#define CAL_resetcobotstatus  pfresetcobotstatus
#define CHK_resetcobotstatus  (pfresetcobotstatus != NULL)
#define EXP_resetcobotstatus   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"resetcobotstatus", (RTS_UINTPTR)resetcobotstatus, 1, 0xA7FAE663, 0x01000000) 
#endif


	/**
	 * 是否使用15066检测标准，将会取消电流滤波,本体电流的抖动将会更容易触发碰撞；
	 */
	typedef struct tagset15066strategy_struct
	{
		RTS_IEC_BOOL enable;				/* VAR_INPUT */	/* 初始化函数之后，仅需调用一次； */
		RTS_IEC_BOOL set15066Strategy;		/* VAR_OUTPUT */
	} set15066strategy_struct;

	void CDECL CDECL_EXT set15066strategy(set15066strategy_struct* p);
	typedef void (CDECL CDECL_EXT* PFSET15066STRATEGY_IEC) (set15066strategy_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SET15066STRATEGY_NOTIMPLEMENTED)
#define USE_set15066strategy
#define EXT_set15066strategy
#define GET_set15066strategy(fl)  ERR_NOTIMPLEMENTED
#define CAL_set15066strategy(p0) 
#define CHK_set15066strategy  FALSE
#define EXP_set15066strategy  ERR_OK
#elif defined(STATIC_LINK)
#define USE_set15066strategy
#define EXT_set15066strategy
#define GET_set15066strategy(fl)  CAL_CMGETAPI( "set15066strategy" ) 
#define CAL_set15066strategy  set15066strategy
#define CHK_set15066strategy  TRUE
#define EXP_set15066strategy  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"set15066strategy", (RTS_UINTPTR)set15066strategy, 1, 0x41622766, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_set15066strategy
#define EXT_set15066strategy
#define GET_set15066strategy(fl)  CAL_CMGETAPI( "set15066strategy" ) 
#define CAL_set15066strategy  set15066strategy
#define CHK_set15066strategy  TRUE
#define EXP_set15066strategy  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"set15066strategy", (RTS_UINTPTR)set15066strategy, 1, 0x41622766, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibset15066strategy
#define EXT_ElfinCollaborativeLibset15066strategy
#define GET_ElfinCollaborativeLibset15066strategy  ERR_OK
#define CAL_ElfinCollaborativeLibset15066strategy  set15066strategy
#define CHK_ElfinCollaborativeLibset15066strategy  TRUE
#define EXP_ElfinCollaborativeLibset15066strategy  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"set15066strategy", (RTS_UINTPTR)set15066strategy, 1, 0x41622766, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_set15066strategy
#define EXT_set15066strategy
#define GET_set15066strategy(fl)  CAL_CMGETAPI( "set15066strategy" ) 
#define CAL_set15066strategy  set15066strategy
#define CHK_set15066strategy  TRUE
#define EXP_set15066strategy  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"set15066strategy", (RTS_UINTPTR)set15066strategy, 1, 0x41622766, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_set15066strategy  PFSET15066STRATEGY_IEC pfset15066strategy;
#define EXT_set15066strategy  extern PFSET15066STRATEGY_IEC pfset15066strategy;
#define GET_set15066strategy(fl)  s_pfCMGetAPI2( "set15066strategy", (RTS_VOID_FCTPTR *)&pfset15066strategy, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x41622766, 0x01000000)
#define CAL_set15066strategy  pfset15066strategy
#define CHK_set15066strategy  (pfset15066strategy != NULL)
#define EXP_set15066strategy   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"set15066strategy", (RTS_UINTPTR)set15066strategy, 1, 0x41622766, 0x01000000) 
#endif


	/**
	 * set actutor damp constant
	 */
	typedef struct tagsetactutordampconstants_struct
	{
		RTS_IEC_LREAL damp[6];				/* VAR_INPUT */
		RTS_IEC_BOOL setActutorDampConstants;	/* VAR_OUTPUT */
	} setactutordampconstants_struct;

	void CDECL CDECL_EXT setactutordampconstants(setactutordampconstants_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETACTUTORDAMPCONSTANTS_IEC) (setactutordampconstants_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETACTUTORDAMPCONSTANTS_NOTIMPLEMENTED)
#define USE_setactutordampconstants
#define EXT_setactutordampconstants
#define GET_setactutordampconstants(fl)  ERR_NOTIMPLEMENTED
#define CAL_setactutordampconstants(p0) 
#define CHK_setactutordampconstants  FALSE
#define EXP_setactutordampconstants  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setactutordampconstants
#define EXT_setactutordampconstants
#define GET_setactutordampconstants(fl)  CAL_CMGETAPI( "setactutordampconstants" ) 
#define CAL_setactutordampconstants  setactutordampconstants
#define CHK_setactutordampconstants  TRUE
#define EXP_setactutordampconstants  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutordampconstants", (RTS_UINTPTR)setactutordampconstants, 1, 0x5F20C2CB, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setactutordampconstants
#define EXT_setactutordampconstants
#define GET_setactutordampconstants(fl)  CAL_CMGETAPI( "setactutordampconstants" ) 
#define CAL_setactutordampconstants  setactutordampconstants
#define CHK_setactutordampconstants  TRUE
#define EXP_setactutordampconstants  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutordampconstants", (RTS_UINTPTR)setactutordampconstants, 1, 0x5F20C2CB, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetactutordampconstants
#define EXT_ElfinCollaborativeLibsetactutordampconstants
#define GET_ElfinCollaborativeLibsetactutordampconstants  ERR_OK
#define CAL_ElfinCollaborativeLibsetactutordampconstants  setactutordampconstants
#define CHK_ElfinCollaborativeLibsetactutordampconstants  TRUE
#define EXP_ElfinCollaborativeLibsetactutordampconstants  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutordampconstants", (RTS_UINTPTR)setactutordampconstants, 1, 0x5F20C2CB, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setactutordampconstants
#define EXT_setactutordampconstants
#define GET_setactutordampconstants(fl)  CAL_CMGETAPI( "setactutordampconstants" ) 
#define CAL_setactutordampconstants  setactutordampconstants
#define CHK_setactutordampconstants  TRUE
#define EXP_setactutordampconstants  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutordampconstants", (RTS_UINTPTR)setactutordampconstants, 1, 0x5F20C2CB, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setactutordampconstants  PFSETACTUTORDAMPCONSTANTS_IEC pfsetactutordampconstants;
#define EXT_setactutordampconstants  extern PFSETACTUTORDAMPCONSTANTS_IEC pfsetactutordampconstants;
#define GET_setactutordampconstants(fl)  s_pfCMGetAPI2( "setactutordampconstants", (RTS_VOID_FCTPTR *)&pfsetactutordampconstants, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x5F20C2CB, 0x01000000)
#define CAL_setactutordampconstants  pfsetactutordampconstants
#define CHK_setactutordampconstants  (pfsetactutordampconstants != NULL)
#define EXP_setactutordampconstants   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutordampconstants", (RTS_UINTPTR)setactutordampconstants, 1, 0x5F20C2CB, 0x01000000) 
#endif


	/**
	 * VAR_INPUT [in] maxCurrents Joints actutor max current in assistive mode.
	 */
	typedef struct tagsetactutormaxcurrentlimits_struct
	{
		RTS_IEC_LREAL maxCurrents[6];		/* VAR_INPUT */
		RTS_IEC_BOOL setActutorMaxCurrentLimits;	/* VAR_OUTPUT */
	} setactutormaxcurrentlimits_struct;

	void CDECL CDECL_EXT setactutormaxcurrentlimits(setactutormaxcurrentlimits_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETACTUTORMAXCURRENTLIMITS_IEC) (setactutormaxcurrentlimits_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETACTUTORMAXCURRENTLIMITS_NOTIMPLEMENTED)
#define USE_setactutormaxcurrentlimits
#define EXT_setactutormaxcurrentlimits
#define GET_setactutormaxcurrentlimits(fl)  ERR_NOTIMPLEMENTED
#define CAL_setactutormaxcurrentlimits(p0) 
#define CHK_setactutormaxcurrentlimits  FALSE
#define EXP_setactutormaxcurrentlimits  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setactutormaxcurrentlimits
#define EXT_setactutormaxcurrentlimits
#define GET_setactutormaxcurrentlimits(fl)  CAL_CMGETAPI( "setactutormaxcurrentlimits" ) 
#define CAL_setactutormaxcurrentlimits  setactutormaxcurrentlimits
#define CHK_setactutormaxcurrentlimits  TRUE
#define EXP_setactutormaxcurrentlimits  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutormaxcurrentlimits", (RTS_UINTPTR)setactutormaxcurrentlimits, 1, 0x51F9F45A, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setactutormaxcurrentlimits
#define EXT_setactutormaxcurrentlimits
#define GET_setactutormaxcurrentlimits(fl)  CAL_CMGETAPI( "setactutormaxcurrentlimits" ) 
#define CAL_setactutormaxcurrentlimits  setactutormaxcurrentlimits
#define CHK_setactutormaxcurrentlimits  TRUE
#define EXP_setactutormaxcurrentlimits  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutormaxcurrentlimits", (RTS_UINTPTR)setactutormaxcurrentlimits, 1, 0x51F9F45A, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetactutormaxcurrentlimits
#define EXT_ElfinCollaborativeLibsetactutormaxcurrentlimits
#define GET_ElfinCollaborativeLibsetactutormaxcurrentlimits  ERR_OK
#define CAL_ElfinCollaborativeLibsetactutormaxcurrentlimits  setactutormaxcurrentlimits
#define CHK_ElfinCollaborativeLibsetactutormaxcurrentlimits  TRUE
#define EXP_ElfinCollaborativeLibsetactutormaxcurrentlimits  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutormaxcurrentlimits", (RTS_UINTPTR)setactutormaxcurrentlimits, 1, 0x51F9F45A, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setactutormaxcurrentlimits
#define EXT_setactutormaxcurrentlimits
#define GET_setactutormaxcurrentlimits(fl)  CAL_CMGETAPI( "setactutormaxcurrentlimits" ) 
#define CAL_setactutormaxcurrentlimits  setactutormaxcurrentlimits
#define CHK_setactutormaxcurrentlimits  TRUE
#define EXP_setactutormaxcurrentlimits  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutormaxcurrentlimits", (RTS_UINTPTR)setactutormaxcurrentlimits, 1, 0x51F9F45A, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setactutormaxcurrentlimits  PFSETACTUTORMAXCURRENTLIMITS_IEC pfsetactutormaxcurrentlimits;
#define EXT_setactutormaxcurrentlimits  extern PFSETACTUTORMAXCURRENTLIMITS_IEC pfsetactutormaxcurrentlimits;
#define GET_setactutormaxcurrentlimits(fl)  s_pfCMGetAPI2( "setactutormaxcurrentlimits", (RTS_VOID_FCTPTR *)&pfsetactutormaxcurrentlimits, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x51F9F45A, 0x01000000)
#define CAL_setactutormaxcurrentlimits  pfsetactutormaxcurrentlimits
#define CHK_setactutormaxcurrentlimits  (pfsetactutormaxcurrentlimits != NULL)
#define EXP_setactutormaxcurrentlimits   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutormaxcurrentlimits", (RTS_UINTPTR)setactutormaxcurrentlimits, 1, 0x51F9F45A, 0x01000000) 
#endif


	/**
	 * VAR_INPUT [in] torqueConstant Motor torque constant
	 * VAR_INPUT [in] gearRatio joint gear ratio
	 * VAR_INPUT [in] maxEfficiency the maximum efficiency of transmission, default setting 1.0
	 */
	typedef struct tagsetactutortorqueconstants_struct
	{
		RTS_IEC_LREAL torqueConstant[6];	/* VAR_INPUT */
		RTS_IEC_LREAL gearRatio[6];			/* VAR_INPUT */
		RTS_IEC_LREAL maxEfficiency[6];		/* VAR_INPUT */
		RTS_IEC_BOOL setActutorTorqueConstants;	/* VAR_OUTPUT */
	} setactutortorqueconstants_struct;

	void CDECL CDECL_EXT setactutortorqueconstants(setactutortorqueconstants_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETACTUTORTORQUECONSTANTS_IEC) (setactutortorqueconstants_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETACTUTORTORQUECONSTANTS_NOTIMPLEMENTED)
#define USE_setactutortorqueconstants
#define EXT_setactutortorqueconstants
#define GET_setactutortorqueconstants(fl)  ERR_NOTIMPLEMENTED
#define CAL_setactutortorqueconstants(p0) 
#define CHK_setactutortorqueconstants  FALSE
#define EXP_setactutortorqueconstants  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setactutortorqueconstants
#define EXT_setactutortorqueconstants
#define GET_setactutortorqueconstants(fl)  CAL_CMGETAPI( "setactutortorqueconstants" ) 
#define CAL_setactutortorqueconstants  setactutortorqueconstants
#define CHK_setactutortorqueconstants  TRUE
#define EXP_setactutortorqueconstants  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutortorqueconstants", (RTS_UINTPTR)setactutortorqueconstants, 1, 0x995D6518, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setactutortorqueconstants
#define EXT_setactutortorqueconstants
#define GET_setactutortorqueconstants(fl)  CAL_CMGETAPI( "setactutortorqueconstants" ) 
#define CAL_setactutortorqueconstants  setactutortorqueconstants
#define CHK_setactutortorqueconstants  TRUE
#define EXP_setactutortorqueconstants  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutortorqueconstants", (RTS_UINTPTR)setactutortorqueconstants, 1, 0x995D6518, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetactutortorqueconstants
#define EXT_ElfinCollaborativeLibsetactutortorqueconstants
#define GET_ElfinCollaborativeLibsetactutortorqueconstants  ERR_OK
#define CAL_ElfinCollaborativeLibsetactutortorqueconstants  setactutortorqueconstants
#define CHK_ElfinCollaborativeLibsetactutortorqueconstants  TRUE
#define EXP_ElfinCollaborativeLibsetactutortorqueconstants  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutortorqueconstants", (RTS_UINTPTR)setactutortorqueconstants, 1, 0x995D6518, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setactutortorqueconstants
#define EXT_setactutortorqueconstants
#define GET_setactutortorqueconstants(fl)  CAL_CMGETAPI( "setactutortorqueconstants" ) 
#define CAL_setactutortorqueconstants  setactutortorqueconstants
#define CHK_setactutortorqueconstants  TRUE
#define EXP_setactutortorqueconstants  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutortorqueconstants", (RTS_UINTPTR)setactutortorqueconstants, 1, 0x995D6518, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setactutortorqueconstants  PFSETACTUTORTORQUECONSTANTS_IEC pfsetactutortorqueconstants;
#define EXT_setactutortorqueconstants  extern PFSETACTUTORTORQUECONSTANTS_IEC pfsetactutortorqueconstants;
#define GET_setactutortorqueconstants(fl)  s_pfCMGetAPI2( "setactutortorqueconstants", (RTS_VOID_FCTPTR *)&pfsetactutortorqueconstants, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x995D6518, 0x01000000)
#define CAL_setactutortorqueconstants  pfsetactutortorqueconstants
#define CHK_setactutortorqueconstants  (pfsetactutortorqueconstants != NULL)
#define EXP_setactutortorqueconstants   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setactutortorqueconstants", (RTS_UINTPTR)setactutortorqueconstants, 1, 0x995D6518, 0x01000000) 
#endif


	/**
	 * Set assistive mode torque collision stop thresholds.
	 * VAR_INPUT [in] assistiveModeThresholds The collision stop thresholds for assistive mode torque.
	 */
	typedef struct tagsetassistivemodecollisionstopthresholds_struct
	{
		RTS_IEC_LREAL assistiveModeCollisionStopThresholds[6];	/* VAR_INPUT */
		RTS_IEC_BOOL setAssistiveModeCollisionStopThresholds;	/* VAR_OUTPUT */
	} setassistivemodecollisionstopthresholds_struct;

	void CDECL CDECL_EXT setassistivemodecollisionstopthresholds(setassistivemodecollisionstopthresholds_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETASSISTIVEMODECOLLISIONSTOPTHRESHOLDS_IEC) (setassistivemodecollisionstopthresholds_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETASSISTIVEMODECOLLISIONSTOPTHRESHOLDS_NOTIMPLEMENTED)
#define USE_setassistivemodecollisionstopthresholds
#define EXT_setassistivemodecollisionstopthresholds
#define GET_setassistivemodecollisionstopthresholds(fl)  ERR_NOTIMPLEMENTED
#define CAL_setassistivemodecollisionstopthresholds(p0) 
#define CHK_setassistivemodecollisionstopthresholds  FALSE
#define EXP_setassistivemodecollisionstopthresholds  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setassistivemodecollisionstopthresholds
#define EXT_setassistivemodecollisionstopthresholds
#define GET_setassistivemodecollisionstopthresholds(fl)  CAL_CMGETAPI( "setassistivemodecollisionstopthresholds" ) 
#define CAL_setassistivemodecollisionstopthresholds  setassistivemodecollisionstopthresholds
#define CHK_setassistivemodecollisionstopthresholds  TRUE
#define EXP_setassistivemodecollisionstopthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setassistivemodecollisionstopthresholds", (RTS_UINTPTR)setassistivemodecollisionstopthresholds, 1, 0x95CB564B, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setassistivemodecollisionstopthresholds
#define EXT_setassistivemodecollisionstopthresholds
#define GET_setassistivemodecollisionstopthresholds(fl)  CAL_CMGETAPI( "setassistivemodecollisionstopthresholds" ) 
#define CAL_setassistivemodecollisionstopthresholds  setassistivemodecollisionstopthresholds
#define CHK_setassistivemodecollisionstopthresholds  TRUE
#define EXP_setassistivemodecollisionstopthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setassistivemodecollisionstopthresholds", (RTS_UINTPTR)setassistivemodecollisionstopthresholds, 1, 0x95CB564B, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetassistivemodecollisionstopthresholds
#define EXT_ElfinCollaborativeLibsetassistivemodecollisionstopthresholds
#define GET_ElfinCollaborativeLibsetassistivemodecollisionstopthresholds  ERR_OK
#define CAL_ElfinCollaborativeLibsetassistivemodecollisionstopthresholds  setassistivemodecollisionstopthresholds
#define CHK_ElfinCollaborativeLibsetassistivemodecollisionstopthresholds  TRUE
#define EXP_ElfinCollaborativeLibsetassistivemodecollisionstopthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setassistivemodecollisionstopthresholds", (RTS_UINTPTR)setassistivemodecollisionstopthresholds, 1, 0x95CB564B, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setassistivemodecollisionstopthresholds
#define EXT_setassistivemodecollisionstopthresholds
#define GET_setassistivemodecollisionstopthresholds(fl)  CAL_CMGETAPI( "setassistivemodecollisionstopthresholds" ) 
#define CAL_setassistivemodecollisionstopthresholds  setassistivemodecollisionstopthresholds
#define CHK_setassistivemodecollisionstopthresholds  TRUE
#define EXP_setassistivemodecollisionstopthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setassistivemodecollisionstopthresholds", (RTS_UINTPTR)setassistivemodecollisionstopthresholds, 1, 0x95CB564B, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setassistivemodecollisionstopthresholds  PFSETASSISTIVEMODECOLLISIONSTOPTHRESHOLDS_IEC pfsetassistivemodecollisionstopthresholds;
#define EXT_setassistivemodecollisionstopthresholds  extern PFSETASSISTIVEMODECOLLISIONSTOPTHRESHOLDS_IEC pfsetassistivemodecollisionstopthresholds;
#define GET_setassistivemodecollisionstopthresholds(fl)  s_pfCMGetAPI2( "setassistivemodecollisionstopthresholds", (RTS_VOID_FCTPTR *)&pfsetassistivemodecollisionstopthresholds, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x95CB564B, 0x01000000)
#define CAL_setassistivemodecollisionstopthresholds  pfsetassistivemodecollisionstopthresholds
#define CHK_setassistivemodecollisionstopthresholds  (pfsetassistivemodecollisionstopthresholds != NULL)
#define EXP_setassistivemodecollisionstopthresholds   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setassistivemodecollisionstopthresholds", (RTS_UINTPTR)setassistivemodecollisionstopthresholds, 1, 0x95CB564B, 0x01000000) 
#endif


	/**
	 * 设置各关节增强阈值参数
	 */
	typedef struct tagsetaugmentedthdindualencoder_struct
	{
		RTS_IEC_LREAL jointThd[6];			/* VAR_INPUT */	/* default setting:1.0 */
		RTS_IEC_BOOL setAugmentedThdInDualEncoder;	/* VAR_OUTPUT */
	} setaugmentedthdindualencoder_struct;

	void CDECL CDECL_EXT setaugmentedthdindualencoder(setaugmentedthdindualencoder_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETAUGMENTEDTHDINDUALENCODER_IEC) (setaugmentedthdindualencoder_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETAUGMENTEDTHDINDUALENCODER_NOTIMPLEMENTED)
#define USE_setaugmentedthdindualencoder
#define EXT_setaugmentedthdindualencoder
#define GET_setaugmentedthdindualencoder(fl)  ERR_NOTIMPLEMENTED
#define CAL_setaugmentedthdindualencoder(p0) 
#define CHK_setaugmentedthdindualencoder  FALSE
#define EXP_setaugmentedthdindualencoder  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setaugmentedthdindualencoder
#define EXT_setaugmentedthdindualencoder
#define GET_setaugmentedthdindualencoder(fl)  CAL_CMGETAPI( "setaugmentedthdindualencoder" ) 
#define CAL_setaugmentedthdindualencoder  setaugmentedthdindualencoder
#define CHK_setaugmentedthdindualencoder  TRUE
#define EXP_setaugmentedthdindualencoder  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setaugmentedthdindualencoder", (RTS_UINTPTR)setaugmentedthdindualencoder, 1, 0xA6973A99, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setaugmentedthdindualencoder
#define EXT_setaugmentedthdindualencoder
#define GET_setaugmentedthdindualencoder(fl)  CAL_CMGETAPI( "setaugmentedthdindualencoder" ) 
#define CAL_setaugmentedthdindualencoder  setaugmentedthdindualencoder
#define CHK_setaugmentedthdindualencoder  TRUE
#define EXP_setaugmentedthdindualencoder  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setaugmentedthdindualencoder", (RTS_UINTPTR)setaugmentedthdindualencoder, 1, 0xA6973A99, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetaugmentedthdindualencoder
#define EXT_ElfinCollaborativeLibsetaugmentedthdindualencoder
#define GET_ElfinCollaborativeLibsetaugmentedthdindualencoder  ERR_OK
#define CAL_ElfinCollaborativeLibsetaugmentedthdindualencoder  setaugmentedthdindualencoder
#define CHK_ElfinCollaborativeLibsetaugmentedthdindualencoder  TRUE
#define EXP_ElfinCollaborativeLibsetaugmentedthdindualencoder  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setaugmentedthdindualencoder", (RTS_UINTPTR)setaugmentedthdindualencoder, 1, 0xA6973A99, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setaugmentedthdindualencoder
#define EXT_setaugmentedthdindualencoder
#define GET_setaugmentedthdindualencoder(fl)  CAL_CMGETAPI( "setaugmentedthdindualencoder" ) 
#define CAL_setaugmentedthdindualencoder  setaugmentedthdindualencoder
#define CHK_setaugmentedthdindualencoder  TRUE
#define EXP_setaugmentedthdindualencoder  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setaugmentedthdindualencoder", (RTS_UINTPTR)setaugmentedthdindualencoder, 1, 0xA6973A99, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setaugmentedthdindualencoder  PFSETAUGMENTEDTHDINDUALENCODER_IEC pfsetaugmentedthdindualencoder;
#define EXT_setaugmentedthdindualencoder  extern PFSETAUGMENTEDTHDINDUALENCODER_IEC pfsetaugmentedthdindualencoder;
#define GET_setaugmentedthdindualencoder(fl)  s_pfCMGetAPI2( "setaugmentedthdindualencoder", (RTS_VOID_FCTPTR *)&pfsetaugmentedthdindualencoder, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xA6973A99, 0x01000000)
#define CAL_setaugmentedthdindualencoder  pfsetaugmentedthdindualencoder
#define CHK_setaugmentedthdindualencoder  (pfsetaugmentedthdindualencoder != NULL)
#define EXP_setaugmentedthdindualencoder   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setaugmentedthdindualencoder", (RTS_UINTPTR)setaugmentedthdindualencoder, 1, 0xA6973A99, 0x01000000) 
#endif


	/**
	 * set collaborative joint space limits
	 * VAR_INPUT [IN] upperJointLimits the upper joint limits in deg
	 * VAR_INPUT [IN] lowerJointLimits  the lower joint limits in deg
	 */
	typedef struct tagsetcollaborativejointspacelimits_struct
	{
		RTS_IEC_LREAL upperJointLimits[6];	/* VAR_INPUT */
		RTS_IEC_LREAL lowerJointLimits[6];	/* VAR_INPUT */
		RTS_IEC_BOOL setCollaborativeJointSpaceLimits;	/* VAR_OUTPUT */
	} setcollaborativejointspacelimits_struct;

	void CDECL CDECL_EXT setcollaborativejointspacelimits(setcollaborativejointspacelimits_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETCOLLABORATIVEJOINTSPACELIMITS_IEC) (setcollaborativejointspacelimits_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETCOLLABORATIVEJOINTSPACELIMITS_NOTIMPLEMENTED)
#define USE_setcollaborativejointspacelimits
#define EXT_setcollaborativejointspacelimits
#define GET_setcollaborativejointspacelimits(fl)  ERR_NOTIMPLEMENTED
#define CAL_setcollaborativejointspacelimits(p0) 
#define CHK_setcollaborativejointspacelimits  FALSE
#define EXP_setcollaborativejointspacelimits  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setcollaborativejointspacelimits
#define EXT_setcollaborativejointspacelimits
#define GET_setcollaborativejointspacelimits(fl)  CAL_CMGETAPI( "setcollaborativejointspacelimits" ) 
#define CAL_setcollaborativejointspacelimits  setcollaborativejointspacelimits
#define CHK_setcollaborativejointspacelimits  TRUE
#define EXP_setcollaborativejointspacelimits  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollaborativejointspacelimits", (RTS_UINTPTR)setcollaborativejointspacelimits, 1, 0xC7F0193C, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setcollaborativejointspacelimits
#define EXT_setcollaborativejointspacelimits
#define GET_setcollaborativejointspacelimits(fl)  CAL_CMGETAPI( "setcollaborativejointspacelimits" ) 
#define CAL_setcollaborativejointspacelimits  setcollaborativejointspacelimits
#define CHK_setcollaborativejointspacelimits  TRUE
#define EXP_setcollaborativejointspacelimits  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollaborativejointspacelimits", (RTS_UINTPTR)setcollaborativejointspacelimits, 1, 0xC7F0193C, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetcollaborativejointspacelimits
#define EXT_ElfinCollaborativeLibsetcollaborativejointspacelimits
#define GET_ElfinCollaborativeLibsetcollaborativejointspacelimits  ERR_OK
#define CAL_ElfinCollaborativeLibsetcollaborativejointspacelimits  setcollaborativejointspacelimits
#define CHK_ElfinCollaborativeLibsetcollaborativejointspacelimits  TRUE
#define EXP_ElfinCollaborativeLibsetcollaborativejointspacelimits  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollaborativejointspacelimits", (RTS_UINTPTR)setcollaborativejointspacelimits, 1, 0xC7F0193C, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setcollaborativejointspacelimits
#define EXT_setcollaborativejointspacelimits
#define GET_setcollaborativejointspacelimits(fl)  CAL_CMGETAPI( "setcollaborativejointspacelimits" ) 
#define CAL_setcollaborativejointspacelimits  setcollaborativejointspacelimits
#define CHK_setcollaborativejointspacelimits  TRUE
#define EXP_setcollaborativejointspacelimits  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollaborativejointspacelimits", (RTS_UINTPTR)setcollaborativejointspacelimits, 1, 0xC7F0193C, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setcollaborativejointspacelimits  PFSETCOLLABORATIVEJOINTSPACELIMITS_IEC pfsetcollaborativejointspacelimits;
#define EXT_setcollaborativejointspacelimits  extern PFSETCOLLABORATIVEJOINTSPACELIMITS_IEC pfsetcollaborativejointspacelimits;
#define GET_setcollaborativejointspacelimits(fl)  s_pfCMGetAPI2( "setcollaborativejointspacelimits", (RTS_VOID_FCTPTR *)&pfsetcollaborativejointspacelimits, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xC7F0193C, 0x01000000)
#define CAL_setcollaborativejointspacelimits  pfsetcollaborativejointspacelimits
#define CHK_setcollaborativejointspacelimits  (pfsetcollaborativejointspacelimits != NULL)
#define EXP_setcollaborativejointspacelimits   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollaborativejointspacelimits", (RTS_UINTPTR)setcollaborativejointspacelimits, 1, 0xC7F0193C, 0x01000000) 
#endif


	/**
	 * if you want to swtich drive back mode, when collsition happend. calls this fucntion only once before you switch to assistive mode.
	 */
	typedef struct tagsetcollisiondrivebackmode_struct
	{
		RTS_IEC_INT drivebackMode;			/* VAR_INPUT */	/* mode_PowerOff = 0,	servo off; mode_ForceFree = 1,  switch to force free control mode;	mode_LimitMotion =2, drive back mode, with limit joint motion and time duration; */
		RTS_IEC_BOOL setCollisionDriveBackMode;	/* VAR_OUTPUT */
	} setcollisiondrivebackmode_struct;

	void CDECL CDECL_EXT setcollisiondrivebackmode(setcollisiondrivebackmode_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETCOLLISIONDRIVEBACKMODE_IEC) (setcollisiondrivebackmode_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETCOLLISIONDRIVEBACKMODE_NOTIMPLEMENTED)
#define USE_setcollisiondrivebackmode
#define EXT_setcollisiondrivebackmode
#define GET_setcollisiondrivebackmode(fl)  ERR_NOTIMPLEMENTED
#define CAL_setcollisiondrivebackmode(p0) 
#define CHK_setcollisiondrivebackmode  FALSE
#define EXP_setcollisiondrivebackmode  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setcollisiondrivebackmode
#define EXT_setcollisiondrivebackmode
#define GET_setcollisiondrivebackmode(fl)  CAL_CMGETAPI( "setcollisiondrivebackmode" ) 
#define CAL_setcollisiondrivebackmode  setcollisiondrivebackmode
#define CHK_setcollisiondrivebackmode  TRUE
#define EXP_setcollisiondrivebackmode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisiondrivebackmode", (RTS_UINTPTR)setcollisiondrivebackmode, 1, 0x35465C30, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setcollisiondrivebackmode
#define EXT_setcollisiondrivebackmode
#define GET_setcollisiondrivebackmode(fl)  CAL_CMGETAPI( "setcollisiondrivebackmode" ) 
#define CAL_setcollisiondrivebackmode  setcollisiondrivebackmode
#define CHK_setcollisiondrivebackmode  TRUE
#define EXP_setcollisiondrivebackmode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisiondrivebackmode", (RTS_UINTPTR)setcollisiondrivebackmode, 1, 0x35465C30, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetcollisiondrivebackmode
#define EXT_ElfinCollaborativeLibsetcollisiondrivebackmode
#define GET_ElfinCollaborativeLibsetcollisiondrivebackmode  ERR_OK
#define CAL_ElfinCollaborativeLibsetcollisiondrivebackmode  setcollisiondrivebackmode
#define CHK_ElfinCollaborativeLibsetcollisiondrivebackmode  TRUE
#define EXP_ElfinCollaborativeLibsetcollisiondrivebackmode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisiondrivebackmode", (RTS_UINTPTR)setcollisiondrivebackmode, 1, 0x35465C30, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setcollisiondrivebackmode
#define EXT_setcollisiondrivebackmode
#define GET_setcollisiondrivebackmode(fl)  CAL_CMGETAPI( "setcollisiondrivebackmode" ) 
#define CAL_setcollisiondrivebackmode  setcollisiondrivebackmode
#define CHK_setcollisiondrivebackmode  TRUE
#define EXP_setcollisiondrivebackmode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisiondrivebackmode", (RTS_UINTPTR)setcollisiondrivebackmode, 1, 0x35465C30, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setcollisiondrivebackmode  PFSETCOLLISIONDRIVEBACKMODE_IEC pfsetcollisiondrivebackmode;
#define EXT_setcollisiondrivebackmode  extern PFSETCOLLISIONDRIVEBACKMODE_IEC pfsetcollisiondrivebackmode;
#define GET_setcollisiondrivebackmode(fl)  s_pfCMGetAPI2( "setcollisiondrivebackmode", (RTS_VOID_FCTPTR *)&pfsetcollisiondrivebackmode, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x35465C30, 0x01000000)
#define CAL_setcollisiondrivebackmode  pfsetcollisiondrivebackmode
#define CHK_setcollisiondrivebackmode  (pfsetcollisiondrivebackmode != NULL)
#define EXP_setcollisiondrivebackmode   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisiondrivebackmode", (RTS_UINTPTR)setcollisiondrivebackmode, 1, 0x35465C30, 0x01000000) 
#endif


	/**
	 * Set collision stop torque thresholds using in momentum calculation.
	 * VAR_INPUT [in] collisionStopThresholds Threshold torque for collision stops.
	 */
	typedef struct tagsetcollisionstopinmomentumthresholds_struct
	{
		RTS_IEC_LREAL collisionStopThresholds[6];	/* VAR_INPUT */	/* joint collisiton threshold; */
		RTS_IEC_BOOL setCollisionStopInMomentumThresholds;	/* VAR_OUTPUT */
	} setcollisionstopinmomentumthresholds_struct;

	void CDECL CDECL_EXT setcollisionstopinmomentumthresholds(setcollisionstopinmomentumthresholds_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETCOLLISIONSTOPINMOMENTUMTHRESHOLDS_IEC) (setcollisionstopinmomentumthresholds_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETCOLLISIONSTOPINMOMENTUMTHRESHOLDS_NOTIMPLEMENTED)
#define USE_setcollisionstopinmomentumthresholds
#define EXT_setcollisionstopinmomentumthresholds
#define GET_setcollisionstopinmomentumthresholds(fl)  ERR_NOTIMPLEMENTED
#define CAL_setcollisionstopinmomentumthresholds(p0) 
#define CHK_setcollisionstopinmomentumthresholds  FALSE
#define EXP_setcollisionstopinmomentumthresholds  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setcollisionstopinmomentumthresholds
#define EXT_setcollisionstopinmomentumthresholds
#define GET_setcollisionstopinmomentumthresholds(fl)  CAL_CMGETAPI( "setcollisionstopinmomentumthresholds" ) 
#define CAL_setcollisionstopinmomentumthresholds  setcollisionstopinmomentumthresholds
#define CHK_setcollisionstopinmomentumthresholds  TRUE
#define EXP_setcollisionstopinmomentumthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisionstopinmomentumthresholds", (RTS_UINTPTR)setcollisionstopinmomentumthresholds, 1, 0xDC15365C, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setcollisionstopinmomentumthresholds
#define EXT_setcollisionstopinmomentumthresholds
#define GET_setcollisionstopinmomentumthresholds(fl)  CAL_CMGETAPI( "setcollisionstopinmomentumthresholds" ) 
#define CAL_setcollisionstopinmomentumthresholds  setcollisionstopinmomentumthresholds
#define CHK_setcollisionstopinmomentumthresholds  TRUE
#define EXP_setcollisionstopinmomentumthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisionstopinmomentumthresholds", (RTS_UINTPTR)setcollisionstopinmomentumthresholds, 1, 0xDC15365C, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetcollisionstopinmomentumthresholds
#define EXT_ElfinCollaborativeLibsetcollisionstopinmomentumthresholds
#define GET_ElfinCollaborativeLibsetcollisionstopinmomentumthresholds  ERR_OK
#define CAL_ElfinCollaborativeLibsetcollisionstopinmomentumthresholds  setcollisionstopinmomentumthresholds
#define CHK_ElfinCollaborativeLibsetcollisionstopinmomentumthresholds  TRUE
#define EXP_ElfinCollaborativeLibsetcollisionstopinmomentumthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisionstopinmomentumthresholds", (RTS_UINTPTR)setcollisionstopinmomentumthresholds, 1, 0xDC15365C, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setcollisionstopinmomentumthresholds
#define EXT_setcollisionstopinmomentumthresholds
#define GET_setcollisionstopinmomentumthresholds(fl)  CAL_CMGETAPI( "setcollisionstopinmomentumthresholds" ) 
#define CAL_setcollisionstopinmomentumthresholds  setcollisionstopinmomentumthresholds
#define CHK_setcollisionstopinmomentumthresholds  TRUE
#define EXP_setcollisionstopinmomentumthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisionstopinmomentumthresholds", (RTS_UINTPTR)setcollisionstopinmomentumthresholds, 1, 0xDC15365C, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setcollisionstopinmomentumthresholds  PFSETCOLLISIONSTOPINMOMENTUMTHRESHOLDS_IEC pfsetcollisionstopinmomentumthresholds;
#define EXT_setcollisionstopinmomentumthresholds  extern PFSETCOLLISIONSTOPINMOMENTUMTHRESHOLDS_IEC pfsetcollisionstopinmomentumthresholds;
#define GET_setcollisionstopinmomentumthresholds(fl)  s_pfCMGetAPI2( "setcollisionstopinmomentumthresholds", (RTS_VOID_FCTPTR *)&pfsetcollisionstopinmomentumthresholds, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xDC15365C, 0x01000000)
#define CAL_setcollisionstopinmomentumthresholds  pfsetcollisionstopinmomentumthresholds
#define CHK_setcollisionstopinmomentumthresholds  (pfsetcollisionstopinmomentumthresholds != NULL)
#define EXP_setcollisionstopinmomentumthresholds   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisionstopinmomentumthresholds", (RTS_UINTPTR)setcollisionstopinmomentumthresholds, 1, 0xDC15365C, 0x01000000) 
#endif


	/**
	 * Set collision stop torque thresholds.
	 * VAR_INPUT [in] collisionStopThresholds Threshold torque for collision stops.
	 */
	typedef struct tagsetcollisionstopthresholds_struct
	{
		RTS_IEC_LREAL collisionStopThresholds[6];	/* VAR_INPUT */	/* joint collisiton threshold; */
		RTS_IEC_BOOL setCollisionStopThresholds;	/* VAR_OUTPUT */
	} setcollisionstopthresholds_struct;

	void CDECL CDECL_EXT setcollisionstopthresholds(setcollisionstopthresholds_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETCOLLISIONSTOPTHRESHOLDS_IEC) (setcollisionstopthresholds_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETCOLLISIONSTOPTHRESHOLDS_NOTIMPLEMENTED)
#define USE_setcollisionstopthresholds
#define EXT_setcollisionstopthresholds
#define GET_setcollisionstopthresholds(fl)  ERR_NOTIMPLEMENTED
#define CAL_setcollisionstopthresholds(p0) 
#define CHK_setcollisionstopthresholds  FALSE
#define EXP_setcollisionstopthresholds  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setcollisionstopthresholds
#define EXT_setcollisionstopthresholds
#define GET_setcollisionstopthresholds(fl)  CAL_CMGETAPI( "setcollisionstopthresholds" ) 
#define CAL_setcollisionstopthresholds  setcollisionstopthresholds
#define CHK_setcollisionstopthresholds  TRUE
#define EXP_setcollisionstopthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisionstopthresholds", (RTS_UINTPTR)setcollisionstopthresholds, 1, 0x6BE01113, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setcollisionstopthresholds
#define EXT_setcollisionstopthresholds
#define GET_setcollisionstopthresholds(fl)  CAL_CMGETAPI( "setcollisionstopthresholds" ) 
#define CAL_setcollisionstopthresholds  setcollisionstopthresholds
#define CHK_setcollisionstopthresholds  TRUE
#define EXP_setcollisionstopthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisionstopthresholds", (RTS_UINTPTR)setcollisionstopthresholds, 1, 0x6BE01113, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetcollisionstopthresholds
#define EXT_ElfinCollaborativeLibsetcollisionstopthresholds
#define GET_ElfinCollaborativeLibsetcollisionstopthresholds  ERR_OK
#define CAL_ElfinCollaborativeLibsetcollisionstopthresholds  setcollisionstopthresholds
#define CHK_ElfinCollaborativeLibsetcollisionstopthresholds  TRUE
#define EXP_ElfinCollaborativeLibsetcollisionstopthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisionstopthresholds", (RTS_UINTPTR)setcollisionstopthresholds, 1, 0x6BE01113, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setcollisionstopthresholds
#define EXT_setcollisionstopthresholds
#define GET_setcollisionstopthresholds(fl)  CAL_CMGETAPI( "setcollisionstopthresholds" ) 
#define CAL_setcollisionstopthresholds  setcollisionstopthresholds
#define CHK_setcollisionstopthresholds  TRUE
#define EXP_setcollisionstopthresholds  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisionstopthresholds", (RTS_UINTPTR)setcollisionstopthresholds, 1, 0x6BE01113, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setcollisionstopthresholds  PFSETCOLLISIONSTOPTHRESHOLDS_IEC pfsetcollisionstopthresholds;
#define EXT_setcollisionstopthresholds  extern PFSETCOLLISIONSTOPTHRESHOLDS_IEC pfsetcollisionstopthresholds;
#define GET_setcollisionstopthresholds(fl)  s_pfCMGetAPI2( "setcollisionstopthresholds", (RTS_VOID_FCTPTR *)&pfsetcollisionstopthresholds, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x6BE01113, 0x01000000)
#define CAL_setcollisionstopthresholds  pfsetcollisionstopthresholds
#define CHK_setcollisionstopthresholds  (pfsetcollisionstopthresholds != NULL)
#define EXP_setcollisionstopthresholds   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setcollisionstopthresholds", (RTS_UINTPTR)setcollisionstopthresholds, 1, 0x6BE01113, 0x01000000) 
#endif


	/**
	 * VAR_INPUT [in] robot dynamics parameters for Elfin.
	 */
	typedef struct tagsetdynamicslinearparameters_struct
	{
		RTS_IEC_LREAL dynParams[78];		/* VAR_INPUT */
		RTS_IEC_BOOL setDynamicsLinearParameters;	/* VAR_OUTPUT */
	} setdynamicslinearparameters_struct;

	void CDECL CDECL_EXT setdynamicslinearparameters(setdynamicslinearparameters_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETDYNAMICSLINEARPARAMETERS_IEC) (setdynamicslinearparameters_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETDYNAMICSLINEARPARAMETERS_NOTIMPLEMENTED)
#define USE_setdynamicslinearparameters
#define EXT_setdynamicslinearparameters
#define GET_setdynamicslinearparameters(fl)  ERR_NOTIMPLEMENTED
#define CAL_setdynamicslinearparameters(p0) 
#define CHK_setdynamicslinearparameters  FALSE
#define EXP_setdynamicslinearparameters  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setdynamicslinearparameters
#define EXT_setdynamicslinearparameters
#define GET_setdynamicslinearparameters(fl)  CAL_CMGETAPI( "setdynamicslinearparameters" ) 
#define CAL_setdynamicslinearparameters  setdynamicslinearparameters
#define CHK_setdynamicslinearparameters  TRUE
#define EXP_setdynamicslinearparameters  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setdynamicslinearparameters", (RTS_UINTPTR)setdynamicslinearparameters, 1, 0xC0B0FD1C, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setdynamicslinearparameters
#define EXT_setdynamicslinearparameters
#define GET_setdynamicslinearparameters(fl)  CAL_CMGETAPI( "setdynamicslinearparameters" ) 
#define CAL_setdynamicslinearparameters  setdynamicslinearparameters
#define CHK_setdynamicslinearparameters  TRUE
#define EXP_setdynamicslinearparameters  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setdynamicslinearparameters", (RTS_UINTPTR)setdynamicslinearparameters, 1, 0xC0B0FD1C, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetdynamicslinearparameters
#define EXT_ElfinCollaborativeLibsetdynamicslinearparameters
#define GET_ElfinCollaborativeLibsetdynamicslinearparameters  ERR_OK
#define CAL_ElfinCollaborativeLibsetdynamicslinearparameters  setdynamicslinearparameters
#define CHK_ElfinCollaborativeLibsetdynamicslinearparameters  TRUE
#define EXP_ElfinCollaborativeLibsetdynamicslinearparameters  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setdynamicslinearparameters", (RTS_UINTPTR)setdynamicslinearparameters, 1, 0xC0B0FD1C, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setdynamicslinearparameters
#define EXT_setdynamicslinearparameters
#define GET_setdynamicslinearparameters(fl)  CAL_CMGETAPI( "setdynamicslinearparameters" ) 
#define CAL_setdynamicslinearparameters  setdynamicslinearparameters
#define CHK_setdynamicslinearparameters  TRUE
#define EXP_setdynamicslinearparameters  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setdynamicslinearparameters", (RTS_UINTPTR)setdynamicslinearparameters, 1, 0xC0B0FD1C, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setdynamicslinearparameters  PFSETDYNAMICSLINEARPARAMETERS_IEC pfsetdynamicslinearparameters;
#define EXT_setdynamicslinearparameters  extern PFSETDYNAMICSLINEARPARAMETERS_IEC pfsetdynamicslinearparameters;
#define GET_setdynamicslinearparameters(fl)  s_pfCMGetAPI2( "setdynamicslinearparameters", (RTS_VOID_FCTPTR *)&pfsetdynamicslinearparameters, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xC0B0FD1C, 0x01000000)
#define CAL_setdynamicslinearparameters  pfsetdynamicslinearparameters
#define CHK_setdynamicslinearparameters  (pfsetdynamicslinearparameters != NULL)
#define EXP_setdynamicslinearparameters   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setdynamicslinearparameters", (RTS_UINTPTR)setdynamicslinearparameters, 1, 0xC0B0FD1C, 0x01000000) 
#endif


	/**
	 * 静态摩擦力矩的补偿系数，用于调节关节在零力示教下启动时的松紧程度；
	 */
	typedef struct tagsetfrictioncompensatoryfactor_struct
	{
		RTS_IEC_LREAL frictionCompensatoryFactor[6];	/* VAR_INPUT */	/* default setting: J1="0.8" J2="0.8" J3="0.8" J4="0.8" J5="0.8" J6="0.8" */
		RTS_IEC_BOOL setFrictionCompensatoryFactor;	/* VAR_OUTPUT */
	} setfrictioncompensatoryfactor_struct;

	void CDECL CDECL_EXT setfrictioncompensatoryfactor(setfrictioncompensatoryfactor_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETFRICTIONCOMPENSATORYFACTOR_IEC) (setfrictioncompensatoryfactor_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETFRICTIONCOMPENSATORYFACTOR_NOTIMPLEMENTED)
#define USE_setfrictioncompensatoryfactor
#define EXT_setfrictioncompensatoryfactor
#define GET_setfrictioncompensatoryfactor(fl)  ERR_NOTIMPLEMENTED
#define CAL_setfrictioncompensatoryfactor(p0) 
#define CHK_setfrictioncompensatoryfactor  FALSE
#define EXP_setfrictioncompensatoryfactor  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setfrictioncompensatoryfactor
#define EXT_setfrictioncompensatoryfactor
#define GET_setfrictioncompensatoryfactor(fl)  CAL_CMGETAPI( "setfrictioncompensatoryfactor" ) 
#define CAL_setfrictioncompensatoryfactor  setfrictioncompensatoryfactor
#define CHK_setfrictioncompensatoryfactor  TRUE
#define EXP_setfrictioncompensatoryfactor  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setfrictioncompensatoryfactor", (RTS_UINTPTR)setfrictioncompensatoryfactor, 1, 0xF0CFD1B6, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setfrictioncompensatoryfactor
#define EXT_setfrictioncompensatoryfactor
#define GET_setfrictioncompensatoryfactor(fl)  CAL_CMGETAPI( "setfrictioncompensatoryfactor" ) 
#define CAL_setfrictioncompensatoryfactor  setfrictioncompensatoryfactor
#define CHK_setfrictioncompensatoryfactor  TRUE
#define EXP_setfrictioncompensatoryfactor  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setfrictioncompensatoryfactor", (RTS_UINTPTR)setfrictioncompensatoryfactor, 1, 0xF0CFD1B6, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetfrictioncompensatoryfactor
#define EXT_ElfinCollaborativeLibsetfrictioncompensatoryfactor
#define GET_ElfinCollaborativeLibsetfrictioncompensatoryfactor  ERR_OK
#define CAL_ElfinCollaborativeLibsetfrictioncompensatoryfactor  setfrictioncompensatoryfactor
#define CHK_ElfinCollaborativeLibsetfrictioncompensatoryfactor  TRUE
#define EXP_ElfinCollaborativeLibsetfrictioncompensatoryfactor  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setfrictioncompensatoryfactor", (RTS_UINTPTR)setfrictioncompensatoryfactor, 1, 0xF0CFD1B6, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setfrictioncompensatoryfactor
#define EXT_setfrictioncompensatoryfactor
#define GET_setfrictioncompensatoryfactor(fl)  CAL_CMGETAPI( "setfrictioncompensatoryfactor" ) 
#define CAL_setfrictioncompensatoryfactor  setfrictioncompensatoryfactor
#define CHK_setfrictioncompensatoryfactor  TRUE
#define EXP_setfrictioncompensatoryfactor  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setfrictioncompensatoryfactor", (RTS_UINTPTR)setfrictioncompensatoryfactor, 1, 0xF0CFD1B6, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setfrictioncompensatoryfactor  PFSETFRICTIONCOMPENSATORYFACTOR_IEC pfsetfrictioncompensatoryfactor;
#define EXT_setfrictioncompensatoryfactor  extern PFSETFRICTIONCOMPENSATORYFACTOR_IEC pfsetfrictioncompensatoryfactor;
#define GET_setfrictioncompensatoryfactor(fl)  s_pfCMGetAPI2( "setfrictioncompensatoryfactor", (RTS_VOID_FCTPTR *)&pfsetfrictioncompensatoryfactor, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xF0CFD1B6, 0x01000000)
#define CAL_setfrictioncompensatoryfactor  pfsetfrictioncompensatoryfactor
#define CHK_setfrictioncompensatoryfactor  (pfsetfrictioncompensatoryfactor != NULL)
#define EXP_setfrictioncompensatoryfactor   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setfrictioncompensatoryfactor", (RTS_UINTPTR)setfrictioncompensatoryfactor, 1, 0xF0CFD1B6, 0x01000000) 
#endif


	/**
	 * 动态摩擦力矩的补偿系数，用于调节关节在动态运行时的程度；
	 */
	typedef struct tagsetfrictioncompensatoryfactorii_struct
	{
		RTS_IEC_LREAL frictionCompensatoryFactor[6];	/* VAR_INPUT */	/* default seting [1.0]; */
		RTS_IEC_BOOL setFrictionCompensatoryFactorII;	/* VAR_OUTPUT */
	} setfrictioncompensatoryfactorii_struct;

	void CDECL CDECL_EXT setfrictioncompensatoryfactorii(setfrictioncompensatoryfactorii_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETFRICTIONCOMPENSATORYFACTORII_IEC) (setfrictioncompensatoryfactorii_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETFRICTIONCOMPENSATORYFACTORII_NOTIMPLEMENTED)
#define USE_setfrictioncompensatoryfactorii
#define EXT_setfrictioncompensatoryfactorii
#define GET_setfrictioncompensatoryfactorii(fl)  ERR_NOTIMPLEMENTED
#define CAL_setfrictioncompensatoryfactorii(p0) 
#define CHK_setfrictioncompensatoryfactorii  FALSE
#define EXP_setfrictioncompensatoryfactorii  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setfrictioncompensatoryfactorii
#define EXT_setfrictioncompensatoryfactorii
#define GET_setfrictioncompensatoryfactorii(fl)  CAL_CMGETAPI( "setfrictioncompensatoryfactorii" ) 
#define CAL_setfrictioncompensatoryfactorii  setfrictioncompensatoryfactorii
#define CHK_setfrictioncompensatoryfactorii  TRUE
#define EXP_setfrictioncompensatoryfactorii  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setfrictioncompensatoryfactorii", (RTS_UINTPTR)setfrictioncompensatoryfactorii, 1, 0x6EA50A68, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setfrictioncompensatoryfactorii
#define EXT_setfrictioncompensatoryfactorii
#define GET_setfrictioncompensatoryfactorii(fl)  CAL_CMGETAPI( "setfrictioncompensatoryfactorii" ) 
#define CAL_setfrictioncompensatoryfactorii  setfrictioncompensatoryfactorii
#define CHK_setfrictioncompensatoryfactorii  TRUE
#define EXP_setfrictioncompensatoryfactorii  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setfrictioncompensatoryfactorii", (RTS_UINTPTR)setfrictioncompensatoryfactorii, 1, 0x6EA50A68, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetfrictioncompensatoryfactorii
#define EXT_ElfinCollaborativeLibsetfrictioncompensatoryfactorii
#define GET_ElfinCollaborativeLibsetfrictioncompensatoryfactorii  ERR_OK
#define CAL_ElfinCollaborativeLibsetfrictioncompensatoryfactorii  setfrictioncompensatoryfactorii
#define CHK_ElfinCollaborativeLibsetfrictioncompensatoryfactorii  TRUE
#define EXP_ElfinCollaborativeLibsetfrictioncompensatoryfactorii  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setfrictioncompensatoryfactorii", (RTS_UINTPTR)setfrictioncompensatoryfactorii, 1, 0x6EA50A68, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setfrictioncompensatoryfactorii
#define EXT_setfrictioncompensatoryfactorii
#define GET_setfrictioncompensatoryfactorii(fl)  CAL_CMGETAPI( "setfrictioncompensatoryfactorii" ) 
#define CAL_setfrictioncompensatoryfactorii  setfrictioncompensatoryfactorii
#define CHK_setfrictioncompensatoryfactorii  TRUE
#define EXP_setfrictioncompensatoryfactorii  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setfrictioncompensatoryfactorii", (RTS_UINTPTR)setfrictioncompensatoryfactorii, 1, 0x6EA50A68, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setfrictioncompensatoryfactorii  PFSETFRICTIONCOMPENSATORYFACTORII_IEC pfsetfrictioncompensatoryfactorii;
#define EXT_setfrictioncompensatoryfactorii  extern PFSETFRICTIONCOMPENSATORYFACTORII_IEC pfsetfrictioncompensatoryfactorii;
#define GET_setfrictioncompensatoryfactorii(fl)  s_pfCMGetAPI2( "setfrictioncompensatoryfactorii", (RTS_VOID_FCTPTR *)&pfsetfrictioncompensatoryfactorii, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x6EA50A68, 0x01000000)
#define CAL_setfrictioncompensatoryfactorii  pfsetfrictioncompensatoryfactorii
#define CHK_setfrictioncompensatoryfactorii  (pfsetfrictioncompensatoryfactorii != NULL)
#define EXP_setfrictioncompensatoryfactorii   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setfrictioncompensatoryfactorii", (RTS_UINTPTR)setfrictioncompensatoryfactorii, 1, 0x6EA50A68, 0x01000000) 
#endif


	/**
	 * set low velocity threshold which is used to calculate
	 */
	typedef struct tagsetlowvelocitythreshold_struct
	{
		RTS_IEC_LREAL velocityThreshold[6];	/* VAR_INPUT */	/* decide whether the joints are in low velocity status; */
		RTS_IEC_BOOL setLowVelocityThreshold;	/* VAR_OUTPUT */
	} setlowvelocitythreshold_struct;

	void CDECL CDECL_EXT setlowvelocitythreshold(setlowvelocitythreshold_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETLOWVELOCITYTHRESHOLD_IEC) (setlowvelocitythreshold_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETLOWVELOCITYTHRESHOLD_NOTIMPLEMENTED)
#define USE_setlowvelocitythreshold
#define EXT_setlowvelocitythreshold
#define GET_setlowvelocitythreshold(fl)  ERR_NOTIMPLEMENTED
#define CAL_setlowvelocitythreshold(p0) 
#define CHK_setlowvelocitythreshold  FALSE
#define EXP_setlowvelocitythreshold  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setlowvelocitythreshold
#define EXT_setlowvelocitythreshold
#define GET_setlowvelocitythreshold(fl)  CAL_CMGETAPI( "setlowvelocitythreshold" ) 
#define CAL_setlowvelocitythreshold  setlowvelocitythreshold
#define CHK_setlowvelocitythreshold  TRUE
#define EXP_setlowvelocitythreshold  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setlowvelocitythreshold", (RTS_UINTPTR)setlowvelocitythreshold, 1, 0x2BE2E169, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setlowvelocitythreshold
#define EXT_setlowvelocitythreshold
#define GET_setlowvelocitythreshold(fl)  CAL_CMGETAPI( "setlowvelocitythreshold" ) 
#define CAL_setlowvelocitythreshold  setlowvelocitythreshold
#define CHK_setlowvelocitythreshold  TRUE
#define EXP_setlowvelocitythreshold  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setlowvelocitythreshold", (RTS_UINTPTR)setlowvelocitythreshold, 1, 0x2BE2E169, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetlowvelocitythreshold
#define EXT_ElfinCollaborativeLibsetlowvelocitythreshold
#define GET_ElfinCollaborativeLibsetlowvelocitythreshold  ERR_OK
#define CAL_ElfinCollaborativeLibsetlowvelocitythreshold  setlowvelocitythreshold
#define CHK_ElfinCollaborativeLibsetlowvelocitythreshold  TRUE
#define EXP_ElfinCollaborativeLibsetlowvelocitythreshold  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setlowvelocitythreshold", (RTS_UINTPTR)setlowvelocitythreshold, 1, 0x2BE2E169, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setlowvelocitythreshold
#define EXT_setlowvelocitythreshold
#define GET_setlowvelocitythreshold(fl)  CAL_CMGETAPI( "setlowvelocitythreshold" ) 
#define CAL_setlowvelocitythreshold  setlowvelocitythreshold
#define CHK_setlowvelocitythreshold  TRUE
#define EXP_setlowvelocitythreshold  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setlowvelocitythreshold", (RTS_UINTPTR)setlowvelocitythreshold, 1, 0x2BE2E169, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setlowvelocitythreshold  PFSETLOWVELOCITYTHRESHOLD_IEC pfsetlowvelocitythreshold;
#define EXT_setlowvelocitythreshold  extern PFSETLOWVELOCITYTHRESHOLD_IEC pfsetlowvelocitythreshold;
#define GET_setlowvelocitythreshold(fl)  s_pfCMGetAPI2( "setlowvelocitythreshold", (RTS_VOID_FCTPTR *)&pfsetlowvelocitythreshold, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x2BE2E169, 0x01000000)
#define CAL_setlowvelocitythreshold  pfsetlowvelocitythreshold
#define CHK_setlowvelocitythreshold  (pfsetlowvelocitythreshold != NULL)
#define EXP_setlowvelocitythreshold   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setlowvelocitythreshold", (RTS_UINTPTR)setlowvelocitythreshold, 1, 0x2BE2E169, 0x01000000) 
#endif


	/**
	 * set max joint velocities, which to lilmit the motion in assistive mode;
	 */
	typedef struct tagsetmaxjointvelocity_struct
	{
		RTS_IEC_LREAL maxJointVelocity[6];	/* VAR_INPUT */
		RTS_IEC_BOOL setMaxJointVelocity;	/* VAR_OUTPUT */
	} setmaxjointvelocity_struct;

	void CDECL CDECL_EXT setmaxjointvelocity(setmaxjointvelocity_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETMAXJOINTVELOCITY_IEC) (setmaxjointvelocity_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETMAXJOINTVELOCITY_NOTIMPLEMENTED)
#define USE_setmaxjointvelocity
#define EXT_setmaxjointvelocity
#define GET_setmaxjointvelocity(fl)  ERR_NOTIMPLEMENTED
#define CAL_setmaxjointvelocity(p0) 
#define CHK_setmaxjointvelocity  FALSE
#define EXP_setmaxjointvelocity  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setmaxjointvelocity
#define EXT_setmaxjointvelocity
#define GET_setmaxjointvelocity(fl)  CAL_CMGETAPI( "setmaxjointvelocity" ) 
#define CAL_setmaxjointvelocity  setmaxjointvelocity
#define CHK_setmaxjointvelocity  TRUE
#define EXP_setmaxjointvelocity  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmaxjointvelocity", (RTS_UINTPTR)setmaxjointvelocity, 1, 0x17E70001, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setmaxjointvelocity
#define EXT_setmaxjointvelocity
#define GET_setmaxjointvelocity(fl)  CAL_CMGETAPI( "setmaxjointvelocity" ) 
#define CAL_setmaxjointvelocity  setmaxjointvelocity
#define CHK_setmaxjointvelocity  TRUE
#define EXP_setmaxjointvelocity  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmaxjointvelocity", (RTS_UINTPTR)setmaxjointvelocity, 1, 0x17E70001, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetmaxjointvelocity
#define EXT_ElfinCollaborativeLibsetmaxjointvelocity
#define GET_ElfinCollaborativeLibsetmaxjointvelocity  ERR_OK
#define CAL_ElfinCollaborativeLibsetmaxjointvelocity  setmaxjointvelocity
#define CHK_ElfinCollaborativeLibsetmaxjointvelocity  TRUE
#define EXP_ElfinCollaborativeLibsetmaxjointvelocity  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmaxjointvelocity", (RTS_UINTPTR)setmaxjointvelocity, 1, 0x17E70001, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setmaxjointvelocity
#define EXT_setmaxjointvelocity
#define GET_setmaxjointvelocity(fl)  CAL_CMGETAPI( "setmaxjointvelocity" ) 
#define CAL_setmaxjointvelocity  setmaxjointvelocity
#define CHK_setmaxjointvelocity  TRUE
#define EXP_setmaxjointvelocity  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmaxjointvelocity", (RTS_UINTPTR)setmaxjointvelocity, 1, 0x17E70001, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setmaxjointvelocity  PFSETMAXJOINTVELOCITY_IEC pfsetmaxjointvelocity;
#define EXT_setmaxjointvelocity  extern PFSETMAXJOINTVELOCITY_IEC pfsetmaxjointvelocity;
#define GET_setmaxjointvelocity(fl)  s_pfCMGetAPI2( "setmaxjointvelocity", (RTS_VOID_FCTPTR *)&pfsetmaxjointvelocity, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x17E70001, 0x01000000)
#define CAL_setmaxjointvelocity  pfsetmaxjointvelocity
#define CHK_setmaxjointvelocity  (pfsetmaxjointvelocity != NULL)
#define EXP_setmaxjointvelocity   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmaxjointvelocity", (RTS_UINTPTR)setmaxjointvelocity, 1, 0x17E70001, 0x01000000) 
#endif


	/**
	 * <description>setmaxpowerandmomentumconstraints</description>
	 */
	typedef struct tagsetmaxpowerandmomentumconstraints_struct
	{
		RTS_IEC_LREAL maxPower;				/* VAR_INPUT */	/* power */
		RTS_IEC_LREAL maxMomentum;			/* VAR_INPUT */	/* momentum */
		RTS_IEC_LREAL jointPowers[6];		/* VAR_INPUT */	/* max power of joints; */
		RTS_IEC_BOOL setMaxPowerAndMomentumConstraints;	/* VAR_OUTPUT */
	} setmaxpowerandmomentumconstraints_struct;

	void CDECL CDECL_EXT setmaxpowerandmomentumconstraints(setmaxpowerandmomentumconstraints_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETMAXPOWERANDMOMENTUMCONSTRAINTS_IEC) (setmaxpowerandmomentumconstraints_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETMAXPOWERANDMOMENTUMCONSTRAINTS_NOTIMPLEMENTED)
#define USE_setmaxpowerandmomentumconstraints
#define EXT_setmaxpowerandmomentumconstraints
#define GET_setmaxpowerandmomentumconstraints(fl)  ERR_NOTIMPLEMENTED
#define CAL_setmaxpowerandmomentumconstraints(p0) 
#define CHK_setmaxpowerandmomentumconstraints  FALSE
#define EXP_setmaxpowerandmomentumconstraints  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setmaxpowerandmomentumconstraints
#define EXT_setmaxpowerandmomentumconstraints
#define GET_setmaxpowerandmomentumconstraints(fl)  CAL_CMGETAPI( "setmaxpowerandmomentumconstraints" ) 
#define CAL_setmaxpowerandmomentumconstraints  setmaxpowerandmomentumconstraints
#define CHK_setmaxpowerandmomentumconstraints  TRUE
#define EXP_setmaxpowerandmomentumconstraints  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmaxpowerandmomentumconstraints", (RTS_UINTPTR)setmaxpowerandmomentumconstraints, 1, 0x8FC378F7, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setmaxpowerandmomentumconstraints
#define EXT_setmaxpowerandmomentumconstraints
#define GET_setmaxpowerandmomentumconstraints(fl)  CAL_CMGETAPI( "setmaxpowerandmomentumconstraints" ) 
#define CAL_setmaxpowerandmomentumconstraints  setmaxpowerandmomentumconstraints
#define CHK_setmaxpowerandmomentumconstraints  TRUE
#define EXP_setmaxpowerandmomentumconstraints  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmaxpowerandmomentumconstraints", (RTS_UINTPTR)setmaxpowerandmomentumconstraints, 1, 0x8FC378F7, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetmaxpowerandmomentumconstraints
#define EXT_ElfinCollaborativeLibsetmaxpowerandmomentumconstraints
#define GET_ElfinCollaborativeLibsetmaxpowerandmomentumconstraints  ERR_OK
#define CAL_ElfinCollaborativeLibsetmaxpowerandmomentumconstraints  setmaxpowerandmomentumconstraints
#define CHK_ElfinCollaborativeLibsetmaxpowerandmomentumconstraints  TRUE
#define EXP_ElfinCollaborativeLibsetmaxpowerandmomentumconstraints  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmaxpowerandmomentumconstraints", (RTS_UINTPTR)setmaxpowerandmomentumconstraints, 1, 0x8FC378F7, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setmaxpowerandmomentumconstraints
#define EXT_setmaxpowerandmomentumconstraints
#define GET_setmaxpowerandmomentumconstraints(fl)  CAL_CMGETAPI( "setmaxpowerandmomentumconstraints" ) 
#define CAL_setmaxpowerandmomentumconstraints  setmaxpowerandmomentumconstraints
#define CHK_setmaxpowerandmomentumconstraints  TRUE
#define EXP_setmaxpowerandmomentumconstraints  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmaxpowerandmomentumconstraints", (RTS_UINTPTR)setmaxpowerandmomentumconstraints, 1, 0x8FC378F7, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setmaxpowerandmomentumconstraints  PFSETMAXPOWERANDMOMENTUMCONSTRAINTS_IEC pfsetmaxpowerandmomentumconstraints;
#define EXT_setmaxpowerandmomentumconstraints  extern PFSETMAXPOWERANDMOMENTUMCONSTRAINTS_IEC pfsetmaxpowerandmomentumconstraints;
#define GET_setmaxpowerandmomentumconstraints(fl)  s_pfCMGetAPI2( "setmaxpowerandmomentumconstraints", (RTS_VOID_FCTPTR *)&pfsetmaxpowerandmomentumconstraints, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x8FC378F7, 0x01000000)
#define CAL_setmaxpowerandmomentumconstraints  pfsetmaxpowerandmomentumconstraints
#define CHK_setmaxpowerandmomentumconstraints  (pfsetmaxpowerandmomentumconstraints != NULL)
#define EXP_setmaxpowerandmomentumconstraints   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmaxpowerandmomentumconstraints", (RTS_UINTPTR)setmaxpowerandmomentumconstraints, 1, 0x8FC378F7, 0x01000000) 
#endif


	/**
	 * <description>setmotionlimitindriveback</description>
	 */
	typedef struct tagsetmotionlimitindriveback_struct
	{
		RTS_IEC_LREAL distanceLimit;		/* VAR_INPUT */	/* allowing joint motion(degree,  °) in drive back mode; */
		RTS_IEC_LREAL timeDuration;			/* VAR_INPUT */	/* allowing time duration(second, s) in drive back mode; */
		RTS_IEC_BOOL setMotionLimitInDriveBack;	/* VAR_OUTPUT */
	} setmotionlimitindriveback_struct;

	void CDECL CDECL_EXT setmotionlimitindriveback(setmotionlimitindriveback_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETMOTIONLIMITINDRIVEBACK_IEC) (setmotionlimitindriveback_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETMOTIONLIMITINDRIVEBACK_NOTIMPLEMENTED)
#define USE_setmotionlimitindriveback
#define EXT_setmotionlimitindriveback
#define GET_setmotionlimitindriveback(fl)  ERR_NOTIMPLEMENTED
#define CAL_setmotionlimitindriveback(p0) 
#define CHK_setmotionlimitindriveback  FALSE
#define EXP_setmotionlimitindriveback  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setmotionlimitindriveback
#define EXT_setmotionlimitindriveback
#define GET_setmotionlimitindriveback(fl)  CAL_CMGETAPI( "setmotionlimitindriveback" ) 
#define CAL_setmotionlimitindriveback  setmotionlimitindriveback
#define CHK_setmotionlimitindriveback  TRUE
#define EXP_setmotionlimitindriveback  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmotionlimitindriveback", (RTS_UINTPTR)setmotionlimitindriveback, 1, 0x2E0BC873, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setmotionlimitindriveback
#define EXT_setmotionlimitindriveback
#define GET_setmotionlimitindriveback(fl)  CAL_CMGETAPI( "setmotionlimitindriveback" ) 
#define CAL_setmotionlimitindriveback  setmotionlimitindriveback
#define CHK_setmotionlimitindriveback  TRUE
#define EXP_setmotionlimitindriveback  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmotionlimitindriveback", (RTS_UINTPTR)setmotionlimitindriveback, 1, 0x2E0BC873, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetmotionlimitindriveback
#define EXT_ElfinCollaborativeLibsetmotionlimitindriveback
#define GET_ElfinCollaborativeLibsetmotionlimitindriveback  ERR_OK
#define CAL_ElfinCollaborativeLibsetmotionlimitindriveback  setmotionlimitindriveback
#define CHK_ElfinCollaborativeLibsetmotionlimitindriveback  TRUE
#define EXP_ElfinCollaborativeLibsetmotionlimitindriveback  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmotionlimitindriveback", (RTS_UINTPTR)setmotionlimitindriveback, 1, 0x2E0BC873, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setmotionlimitindriveback
#define EXT_setmotionlimitindriveback
#define GET_setmotionlimitindriveback(fl)  CAL_CMGETAPI( "setmotionlimitindriveback" ) 
#define CAL_setmotionlimitindriveback  setmotionlimitindriveback
#define CHK_setmotionlimitindriveback  TRUE
#define EXP_setmotionlimitindriveback  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmotionlimitindriveback", (RTS_UINTPTR)setmotionlimitindriveback, 1, 0x2E0BC873, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setmotionlimitindriveback  PFSETMOTIONLIMITINDRIVEBACK_IEC pfsetmotionlimitindriveback;
#define EXT_setmotionlimitindriveback  extern PFSETMOTIONLIMITINDRIVEBACK_IEC pfsetmotionlimitindriveback;
#define GET_setmotionlimitindriveback(fl)  s_pfCMGetAPI2( "setmotionlimitindriveback", (RTS_VOID_FCTPTR *)&pfsetmotionlimitindriveback, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x2E0BC873, 0x01000000)
#define CAL_setmotionlimitindriveback  pfsetmotionlimitindriveback
#define CHK_setmotionlimitindriveback  (pfsetmotionlimitindriveback != NULL)
#define EXP_setmotionlimitindriveback   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setmotionlimitindriveback", (RTS_UINTPTR)setmotionlimitindriveback, 1, 0x2E0BC873, 0x01000000) 
#endif


	/**
	 * VAR_INPUT [in] payloadMass The mass of the payload in kg
	 * param [in] centerOfMass The center of mass for the payload in millimeter
	 */
	typedef struct tagsetpayloadmassproperties_struct
	{
		RTS_IEC_LREAL mass;					/* VAR_INPUT */
		RTS_IEC_LREAL centerofMass[3];		/* VAR_INPUT */	/* (millimeter) */
		RTS_IEC_BOOL setPayloadMassProperties;	/* VAR_OUTPUT */
	} setpayloadmassproperties_struct;

	void CDECL CDECL_EXT setpayloadmassproperties(setpayloadmassproperties_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETPAYLOADMASSPROPERTIES_IEC) (setpayloadmassproperties_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETPAYLOADMASSPROPERTIES_NOTIMPLEMENTED)
#define USE_setpayloadmassproperties
#define EXT_setpayloadmassproperties
#define GET_setpayloadmassproperties(fl)  ERR_NOTIMPLEMENTED
#define CAL_setpayloadmassproperties(p0) 
#define CHK_setpayloadmassproperties  FALSE
#define EXP_setpayloadmassproperties  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setpayloadmassproperties
#define EXT_setpayloadmassproperties
#define GET_setpayloadmassproperties(fl)  CAL_CMGETAPI( "setpayloadmassproperties" ) 
#define CAL_setpayloadmassproperties  setpayloadmassproperties
#define CHK_setpayloadmassproperties  TRUE
#define EXP_setpayloadmassproperties  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setpayloadmassproperties", (RTS_UINTPTR)setpayloadmassproperties, 1, 0xAC8C023C, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setpayloadmassproperties
#define EXT_setpayloadmassproperties
#define GET_setpayloadmassproperties(fl)  CAL_CMGETAPI( "setpayloadmassproperties" ) 
#define CAL_setpayloadmassproperties  setpayloadmassproperties
#define CHK_setpayloadmassproperties  TRUE
#define EXP_setpayloadmassproperties  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setpayloadmassproperties", (RTS_UINTPTR)setpayloadmassproperties, 1, 0xAC8C023C, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetpayloadmassproperties
#define EXT_ElfinCollaborativeLibsetpayloadmassproperties
#define GET_ElfinCollaborativeLibsetpayloadmassproperties  ERR_OK
#define CAL_ElfinCollaborativeLibsetpayloadmassproperties  setpayloadmassproperties
#define CHK_ElfinCollaborativeLibsetpayloadmassproperties  TRUE
#define EXP_ElfinCollaborativeLibsetpayloadmassproperties  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setpayloadmassproperties", (RTS_UINTPTR)setpayloadmassproperties, 1, 0xAC8C023C, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setpayloadmassproperties
#define EXT_setpayloadmassproperties
#define GET_setpayloadmassproperties(fl)  CAL_CMGETAPI( "setpayloadmassproperties" ) 
#define CAL_setpayloadmassproperties  setpayloadmassproperties
#define CHK_setpayloadmassproperties  TRUE
#define EXP_setpayloadmassproperties  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setpayloadmassproperties", (RTS_UINTPTR)setpayloadmassproperties, 1, 0xAC8C023C, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setpayloadmassproperties  PFSETPAYLOADMASSPROPERTIES_IEC pfsetpayloadmassproperties;
#define EXT_setpayloadmassproperties  extern PFSETPAYLOADMASSPROPERTIES_IEC pfsetpayloadmassproperties;
#define GET_setpayloadmassproperties(fl)  s_pfCMGetAPI2( "setpayloadmassproperties", (RTS_VOID_FCTPTR *)&pfsetpayloadmassproperties, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xAC8C023C, 0x01000000)
#define CAL_setpayloadmassproperties  pfsetpayloadmassproperties
#define CHK_setpayloadmassproperties  (pfsetpayloadmassproperties != NULL)
#define EXP_setpayloadmassproperties   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setpayloadmassproperties", (RTS_UINTPTR)setpayloadmassproperties, 1, 0xAC8C023C, 0x01000000) 
#endif


<<<<<<< HEAD
/**
 * Call this function only once before close Assistive mode.
 */
typedef struct tagsetreadytocloseassistivemode_struct
{
	RTS_IEC_BOOL setReadyToCloseAssistiveMode;	/* VAR_OUTPUT */	
} setreadytocloseassistivemode_struct;
=======
	/**
	 * Call this function only once before close Assistive mode.
	 */
	typedef struct tagsetreadytocloseassistivemode_struct
	{
		RTS_IEC_BOOL setReadyToCloseAssistiveMode;	/* VAR_OUTPUT */
	} setreadytocloseassistivemode_struct;
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117

	void CDECL CDECL_EXT setreadytocloseassistivemode(setreadytocloseassistivemode_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETREADYTOCLOSEASSISTIVEMODE_IEC) (setreadytocloseassistivemode_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETREADYTOCLOSEASSISTIVEMODE_NOTIMPLEMENTED)
#define USE_setreadytocloseassistivemode
#define EXT_setreadytocloseassistivemode
#define GET_setreadytocloseassistivemode(fl)  ERR_NOTIMPLEMENTED
#define CAL_setreadytocloseassistivemode(p0) 
#define CHK_setreadytocloseassistivemode  FALSE
#define EXP_setreadytocloseassistivemode  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setreadytocloseassistivemode
#define EXT_setreadytocloseassistivemode
#define GET_setreadytocloseassistivemode(fl)  CAL_CMGETAPI( "setreadytocloseassistivemode" ) 
#define CAL_setreadytocloseassistivemode  setreadytocloseassistivemode
#define CHK_setreadytocloseassistivemode  TRUE
#define EXP_setreadytocloseassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setreadytocloseassistivemode", (RTS_UINTPTR)setreadytocloseassistivemode, 1, 0x9BED25DA, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setreadytocloseassistivemode
#define EXT_setreadytocloseassistivemode
#define GET_setreadytocloseassistivemode(fl)  CAL_CMGETAPI( "setreadytocloseassistivemode" ) 
#define CAL_setreadytocloseassistivemode  setreadytocloseassistivemode
#define CHK_setreadytocloseassistivemode  TRUE
#define EXP_setreadytocloseassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setreadytocloseassistivemode", (RTS_UINTPTR)setreadytocloseassistivemode, 1, 0x9BED25DA, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetreadytocloseassistivemode
#define EXT_ElfinCollaborativeLibsetreadytocloseassistivemode
#define GET_ElfinCollaborativeLibsetreadytocloseassistivemode  ERR_OK
#define CAL_ElfinCollaborativeLibsetreadytocloseassistivemode  setreadytocloseassistivemode
#define CHK_ElfinCollaborativeLibsetreadytocloseassistivemode  TRUE
#define EXP_ElfinCollaborativeLibsetreadytocloseassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setreadytocloseassistivemode", (RTS_UINTPTR)setreadytocloseassistivemode, 1, 0x9BED25DA, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setreadytocloseassistivemode
#define EXT_setreadytocloseassistivemode
#define GET_setreadytocloseassistivemode(fl)  CAL_CMGETAPI( "setreadytocloseassistivemode" ) 
#define CAL_setreadytocloseassistivemode  setreadytocloseassistivemode
#define CHK_setreadytocloseassistivemode  TRUE
#define EXP_setreadytocloseassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setreadytocloseassistivemode", (RTS_UINTPTR)setreadytocloseassistivemode, 1, 0x9BED25DA, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setreadytocloseassistivemode  PFSETREADYTOCLOSEASSISTIVEMODE_IEC pfsetreadytocloseassistivemode;
#define EXT_setreadytocloseassistivemode  extern PFSETREADYTOCLOSEASSISTIVEMODE_IEC pfsetreadytocloseassistivemode;
#define GET_setreadytocloseassistivemode(fl)  s_pfCMGetAPI2( "setreadytocloseassistivemode", (RTS_VOID_FCTPTR *)&pfsetreadytocloseassistivemode, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x9BED25DA, 0x01000000)
#define CAL_setreadytocloseassistivemode  pfsetreadytocloseassistivemode
#define CHK_setreadytocloseassistivemode  (pfsetreadytocloseassistivemode != NULL)
#define EXP_setreadytocloseassistivemode   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setreadytocloseassistivemode", (RTS_UINTPTR)setreadytocloseassistivemode, 1, 0x9BED25DA, 0x01000000) 
#endif


	/**
	 * set robot mounting angles
	 */
	typedef struct tagsetrobotmountingangles_struct
	{
		RTS_IEC_LREAL rotation;				/* VAR_INPUT */	/* rotate robot base mounting angles(deg) */
		RTS_IEC_LREAL tilt;					/* VAR_INPUT */	/* tilt robot base mounting angles(deg) */
		RTS_IEC_BOOL setRobotMountingAngles;	/* VAR_OUTPUT */
	} setrobotmountingangles_struct;

	void CDECL CDECL_EXT setrobotmountingangles(setrobotmountingangles_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETROBOTMOUNTINGANGLES_IEC) (setrobotmountingangles_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETROBOTMOUNTINGANGLES_NOTIMPLEMENTED)
#define USE_setrobotmountingangles
#define EXT_setrobotmountingangles
#define GET_setrobotmountingangles(fl)  ERR_NOTIMPLEMENTED
#define CAL_setrobotmountingangles(p0) 
#define CHK_setrobotmountingangles  FALSE
#define EXP_setrobotmountingangles  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setrobotmountingangles
#define EXT_setrobotmountingangles
#define GET_setrobotmountingangles(fl)  CAL_CMGETAPI( "setrobotmountingangles" ) 
#define CAL_setrobotmountingangles  setrobotmountingangles
#define CHK_setrobotmountingangles  TRUE
#define EXP_setrobotmountingangles  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setrobotmountingangles", (RTS_UINTPTR)setrobotmountingangles, 1, 0x774360BE, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setrobotmountingangles
#define EXT_setrobotmountingangles
#define GET_setrobotmountingangles(fl)  CAL_CMGETAPI( "setrobotmountingangles" ) 
#define CAL_setrobotmountingangles  setrobotmountingangles
#define CHK_setrobotmountingangles  TRUE
#define EXP_setrobotmountingangles  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setrobotmountingangles", (RTS_UINTPTR)setrobotmountingangles, 1, 0x774360BE, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetrobotmountingangles
#define EXT_ElfinCollaborativeLibsetrobotmountingangles
#define GET_ElfinCollaborativeLibsetrobotmountingangles  ERR_OK
#define CAL_ElfinCollaborativeLibsetrobotmountingangles  setrobotmountingangles
#define CHK_ElfinCollaborativeLibsetrobotmountingangles  TRUE
#define EXP_ElfinCollaborativeLibsetrobotmountingangles  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setrobotmountingangles", (RTS_UINTPTR)setrobotmountingangles, 1, 0x774360BE, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setrobotmountingangles
#define EXT_setrobotmountingangles
#define GET_setrobotmountingangles(fl)  CAL_CMGETAPI( "setrobotmountingangles" ) 
#define CAL_setrobotmountingangles  setrobotmountingangles
#define CHK_setrobotmountingangles  TRUE
#define EXP_setrobotmountingangles  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setrobotmountingangles", (RTS_UINTPTR)setrobotmountingangles, 1, 0x774360BE, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setrobotmountingangles  PFSETROBOTMOUNTINGANGLES_IEC pfsetrobotmountingangles;
#define EXT_setrobotmountingangles  extern PFSETROBOTMOUNTINGANGLES_IEC pfsetrobotmountingangles;
#define GET_setrobotmountingangles(fl)  s_pfCMGetAPI2( "setrobotmountingangles", (RTS_VOID_FCTPTR *)&pfsetrobotmountingangles, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x774360BE, 0x01000000)
#define CAL_setrobotmountingangles  pfsetrobotmountingangles
#define CHK_setrobotmountingangles  (pfsetrobotmountingangles != NULL)
#define EXP_setrobotmountingangles   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setrobotmountingangles", (RTS_UINTPTR)setrobotmountingangles, 1, 0x774360BE, 0x01000000) 
#endif


	/**
	 * Call this function only once before start Assistive mode every times.
	 */
	typedef struct tagsetstartassistivemode_struct
	{
		RTS_IEC_BOOL setStartAssistiveMode;	/* VAR_OUTPUT */
	} setstartassistivemode_struct;

	void CDECL CDECL_EXT setstartassistivemode(setstartassistivemode_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETSTARTASSISTIVEMODE_IEC) (setstartassistivemode_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETSTARTASSISTIVEMODE_NOTIMPLEMENTED)
#define USE_setstartassistivemode
#define EXT_setstartassistivemode
#define GET_setstartassistivemode(fl)  ERR_NOTIMPLEMENTED
#define CAL_setstartassistivemode(p0) 
#define CHK_setstartassistivemode  FALSE
#define EXP_setstartassistivemode  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setstartassistivemode
#define EXT_setstartassistivemode
#define GET_setstartassistivemode(fl)  CAL_CMGETAPI( "setstartassistivemode" ) 
#define CAL_setstartassistivemode  setstartassistivemode
#define CHK_setstartassistivemode  TRUE
#define EXP_setstartassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setstartassistivemode", (RTS_UINTPTR)setstartassistivemode, 1, 0x018FD046, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setstartassistivemode
#define EXT_setstartassistivemode
#define GET_setstartassistivemode(fl)  CAL_CMGETAPI( "setstartassistivemode" ) 
#define CAL_setstartassistivemode  setstartassistivemode
#define CHK_setstartassistivemode  TRUE
#define EXP_setstartassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setstartassistivemode", (RTS_UINTPTR)setstartassistivemode, 1, 0x018FD046, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetstartassistivemode
#define EXT_ElfinCollaborativeLibsetstartassistivemode
#define GET_ElfinCollaborativeLibsetstartassistivemode  ERR_OK
#define CAL_ElfinCollaborativeLibsetstartassistivemode  setstartassistivemode
#define CHK_ElfinCollaborativeLibsetstartassistivemode  TRUE
#define EXP_ElfinCollaborativeLibsetstartassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setstartassistivemode", (RTS_UINTPTR)setstartassistivemode, 1, 0x018FD046, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setstartassistivemode
#define EXT_setstartassistivemode
#define GET_setstartassistivemode(fl)  CAL_CMGETAPI( "setstartassistivemode" ) 
#define CAL_setstartassistivemode  setstartassistivemode
#define CHK_setstartassistivemode  TRUE
#define EXP_setstartassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setstartassistivemode", (RTS_UINTPTR)setstartassistivemode, 1, 0x018FD046, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setstartassistivemode  PFSETSTARTASSISTIVEMODE_IEC pfsetstartassistivemode;
#define EXT_setstartassistivemode  extern PFSETSTARTASSISTIVEMODE_IEC pfsetstartassistivemode;
#define GET_setstartassistivemode(fl)  s_pfCMGetAPI2( "setstartassistivemode", (RTS_VOID_FCTPTR *)&pfsetstartassistivemode, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x018FD046, 0x01000000)
#define CAL_setstartassistivemode  pfsetstartassistivemode
#define CHK_setstartassistivemode  (pfsetstartassistivemode != NULL)
#define EXP_setstartassistivemode   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setstartassistivemode", (RTS_UINTPTR)setstartassistivemode, 1, 0x018FD046, 0x01000000) 
#endif


	/**
	 * <description>setstarttimeforcheckassistivemode</description>
	 */
	typedef struct tagsetstarttimeforcheckassistivemode_struct
	{
		RTS_IEC_LREAL assistiveCheckTime;	/* VAR_INPUT */	/* the start time for check robot status in assistive mode, second ,s; */
		RTS_IEC_BOOL setStartTimeForCheckAssistiveMode;	/* VAR_OUTPUT */
	} setstarttimeforcheckassistivemode_struct;

	void CDECL CDECL_EXT setstarttimeforcheckassistivemode(setstarttimeforcheckassistivemode_struct* p);
	typedef void (CDECL CDECL_EXT* PFSETSTARTTIMEFORCHECKASSISTIVEMODE_IEC) (setstarttimeforcheckassistivemode_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(SETSTARTTIMEFORCHECKASSISTIVEMODE_NOTIMPLEMENTED)
#define USE_setstarttimeforcheckassistivemode
#define EXT_setstarttimeforcheckassistivemode
#define GET_setstarttimeforcheckassistivemode(fl)  ERR_NOTIMPLEMENTED
#define CAL_setstarttimeforcheckassistivemode(p0) 
#define CHK_setstarttimeforcheckassistivemode  FALSE
#define EXP_setstarttimeforcheckassistivemode  ERR_OK
#elif defined(STATIC_LINK)
#define USE_setstarttimeforcheckassistivemode
#define EXT_setstarttimeforcheckassistivemode
#define GET_setstarttimeforcheckassistivemode(fl)  CAL_CMGETAPI( "setstarttimeforcheckassistivemode" ) 
#define CAL_setstarttimeforcheckassistivemode  setstarttimeforcheckassistivemode
#define CHK_setstarttimeforcheckassistivemode  TRUE
#define EXP_setstarttimeforcheckassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setstarttimeforcheckassistivemode", (RTS_UINTPTR)setstarttimeforcheckassistivemode, 1, 0xF95F41F2, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_setstarttimeforcheckassistivemode
#define EXT_setstarttimeforcheckassistivemode
#define GET_setstarttimeforcheckassistivemode(fl)  CAL_CMGETAPI( "setstarttimeforcheckassistivemode" ) 
#define CAL_setstarttimeforcheckassistivemode  setstarttimeforcheckassistivemode
#define CHK_setstarttimeforcheckassistivemode  TRUE
#define EXP_setstarttimeforcheckassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setstarttimeforcheckassistivemode", (RTS_UINTPTR)setstarttimeforcheckassistivemode, 1, 0xF95F41F2, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibsetstarttimeforcheckassistivemode
#define EXT_ElfinCollaborativeLibsetstarttimeforcheckassistivemode
#define GET_ElfinCollaborativeLibsetstarttimeforcheckassistivemode  ERR_OK
#define CAL_ElfinCollaborativeLibsetstarttimeforcheckassistivemode  setstarttimeforcheckassistivemode
#define CHK_ElfinCollaborativeLibsetstarttimeforcheckassistivemode  TRUE
#define EXP_ElfinCollaborativeLibsetstarttimeforcheckassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setstarttimeforcheckassistivemode", (RTS_UINTPTR)setstarttimeforcheckassistivemode, 1, 0xF95F41F2, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_setstarttimeforcheckassistivemode
#define EXT_setstarttimeforcheckassistivemode
#define GET_setstarttimeforcheckassistivemode(fl)  CAL_CMGETAPI( "setstarttimeforcheckassistivemode" ) 
#define CAL_setstarttimeforcheckassistivemode  setstarttimeforcheckassistivemode
#define CHK_setstarttimeforcheckassistivemode  TRUE
#define EXP_setstarttimeforcheckassistivemode  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setstarttimeforcheckassistivemode", (RTS_UINTPTR)setstarttimeforcheckassistivemode, 1, 0xF95F41F2, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_setstarttimeforcheckassistivemode  PFSETSTARTTIMEFORCHECKASSISTIVEMODE_IEC pfsetstarttimeforcheckassistivemode;
#define EXT_setstarttimeforcheckassistivemode  extern PFSETSTARTTIMEFORCHECKASSISTIVEMODE_IEC pfsetstarttimeforcheckassistivemode;
#define GET_setstarttimeforcheckassistivemode(fl)  s_pfCMGetAPI2( "setstarttimeforcheckassistivemode", (RTS_VOID_FCTPTR *)&pfsetstarttimeforcheckassistivemode, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xF95F41F2, 0x01000000)
#define CAL_setstarttimeforcheckassistivemode  pfsetstarttimeforcheckassistivemode
#define CHK_setstarttimeforcheckassistivemode  (pfsetstarttimeforcheckassistivemode != NULL)
#define EXP_setstarttimeforcheckassistivemode   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"setstarttimeforcheckassistivemode", (RTS_UINTPTR)setstarttimeforcheckassistivemode, 1, 0xF95F41F2, 0x01000000) 
#endif


<<<<<<< HEAD
/**
 * 实时更新外部轴的状态（命令位置、实际位置、换算后的检测力）
 */
typedef struct tagupdateexternalaxisstate_struct
{
	RTS_IEC_LREAL commandPositions[6];	/* VAR_INPUT */	/* command external axis position */
	RTS_IEC_LREAL actualPositions[6];	/* VAR_INPUT */	/* actual external axis position */
	RTS_IEC_LREAL sensedForce[6];		/* VAR_INPUT */	/* 基于电机电流、减速比和丝杆换算得到的检测力； */
	RTS_IEC_BOOL updateExternalAxisState;	/* VAR_OUTPUT */	
} updateexternalaxisstate_struct;

void CDECL CDECL_EXT updateexternalaxisstate(updateexternalaxisstate_struct *p);
typedef void (CDECL CDECL_EXT* PFUPDATEEXTERNALAXISSTATE_IEC) (updateexternalaxisstate_struct *p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(UPDATEEXTERNALAXISSTATE_NOTIMPLEMENTED)
	#define USE_updateexternalaxisstate
	#define EXT_updateexternalaxisstate
	#define GET_updateexternalaxisstate(fl)  ERR_NOTIMPLEMENTED
	#define CAL_updateexternalaxisstate(p0) 
	#define CHK_updateexternalaxisstate  FALSE
	#define EXP_updateexternalaxisstate  ERR_OK
#elif defined(STATIC_LINK)
	#define USE_updateexternalaxisstate
	#define EXT_updateexternalaxisstate
	#define GET_updateexternalaxisstate(fl)  CAL_CMGETAPI( "updateexternalaxisstate" ) 
	#define CAL_updateexternalaxisstate  updateexternalaxisstate
	#define CHK_updateexternalaxisstate  TRUE
	#define EXP_updateexternalaxisstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateexternalaxisstate", (RTS_UINTPTR)updateexternalaxisstate, 1, 0xF276A89F, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
	#define USE_updateexternalaxisstate
	#define EXT_updateexternalaxisstate
	#define GET_updateexternalaxisstate(fl)  CAL_CMGETAPI( "updateexternalaxisstate" ) 
	#define CAL_updateexternalaxisstate  updateexternalaxisstate
	#define CHK_updateexternalaxisstate  TRUE
	#define EXP_updateexternalaxisstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateexternalaxisstate", (RTS_UINTPTR)updateexternalaxisstate, 1, 0xF276A89F, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
	#define USE_ElfinCollaborativeLibupdateexternalaxisstate
	#define EXT_ElfinCollaborativeLibupdateexternalaxisstate
	#define GET_ElfinCollaborativeLibupdateexternalaxisstate  ERR_OK
	#define CAL_ElfinCollaborativeLibupdateexternalaxisstate  updateexternalaxisstate
	#define CHK_ElfinCollaborativeLibupdateexternalaxisstate  TRUE
	#define EXP_ElfinCollaborativeLibupdateexternalaxisstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateexternalaxisstate", (RTS_UINTPTR)updateexternalaxisstate, 1, 0xF276A89F, 0x01000000) 
#elif defined(CPLUSPLUS)
	#define USE_updateexternalaxisstate
	#define EXT_updateexternalaxisstate
	#define GET_updateexternalaxisstate(fl)  CAL_CMGETAPI( "updateexternalaxisstate" ) 
	#define CAL_updateexternalaxisstate  updateexternalaxisstate
	#define CHK_updateexternalaxisstate  TRUE
	#define EXP_updateexternalaxisstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateexternalaxisstate", (RTS_UINTPTR)updateexternalaxisstate, 1, 0xF276A89F, 0x01000000) 
#else /* DYNAMIC_LINK */
	#define USE_updateexternalaxisstate  PFUPDATEEXTERNALAXISSTATE_IEC pfupdateexternalaxisstate;
	#define EXT_updateexternalaxisstate  extern PFUPDATEEXTERNALAXISSTATE_IEC pfupdateexternalaxisstate;
	#define GET_updateexternalaxisstate(fl)  s_pfCMGetAPI2( "updateexternalaxisstate", (RTS_VOID_FCTPTR *)&pfupdateexternalaxisstate, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xF276A89F, 0x01000000)
	#define CAL_updateexternalaxisstate  pfupdateexternalaxisstate
	#define CHK_updateexternalaxisstate  (pfupdateexternalaxisstate != NULL)
	#define EXP_updateexternalaxisstate   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateexternalaxisstate", (RTS_UINTPTR)updateexternalaxisstate, 1, 0xF276A89F, 0x01000000) 
#endif


/**
 * <description>updateforcesensordata</description>
 */
typedef struct tagupdateforcesensordata_struct
{
	RTS_IEC_BOOL enable;				/* VAR_INPUT */	/* 是否开启末端力传感器的碰撞检测功能； */
	RTS_IEC_LREAL calibedForce[6];		/* VAR_INPUT */	/* 获取标定后的力传感器数据，当前为：GetFTValueInSelectedFrame */
	RTS_IEC_BOOL updateForceSensorData;	/* VAR_OUTPUT */	
} updateforcesensordata_struct;

void CDECL CDECL_EXT updateforcesensordata(updateforcesensordata_struct *p);
typedef void (CDECL CDECL_EXT* PFUPDATEFORCESENSORDATA_IEC) (updateforcesensordata_struct *p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(UPDATEFORCESENSORDATA_NOTIMPLEMENTED)
	#define USE_updateforcesensordata
	#define EXT_updateforcesensordata
	#define GET_updateforcesensordata(fl)  ERR_NOTIMPLEMENTED
	#define CAL_updateforcesensordata(p0) 
	#define CHK_updateforcesensordata  FALSE
	#define EXP_updateforcesensordata  ERR_OK
#elif defined(STATIC_LINK)
	#define USE_updateforcesensordata
	#define EXT_updateforcesensordata
	#define GET_updateforcesensordata(fl)  CAL_CMGETAPI( "updateforcesensordata" ) 
	#define CAL_updateforcesensordata  updateforcesensordata
	#define CHK_updateforcesensordata  TRUE
	#define EXP_updateforcesensordata  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensordata", (RTS_UINTPTR)updateforcesensordata, 1, 0x853A9D07, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
	#define USE_updateforcesensordata
	#define EXT_updateforcesensordata
	#define GET_updateforcesensordata(fl)  CAL_CMGETAPI( "updateforcesensordata" ) 
	#define CAL_updateforcesensordata  updateforcesensordata
	#define CHK_updateforcesensordata  TRUE
	#define EXP_updateforcesensordata  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensordata", (RTS_UINTPTR)updateforcesensordata, 1, 0x853A9D07, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
	#define USE_ElfinCollaborativeLibupdateforcesensordata
	#define EXT_ElfinCollaborativeLibupdateforcesensordata
	#define GET_ElfinCollaborativeLibupdateforcesensordata  ERR_OK
	#define CAL_ElfinCollaborativeLibupdateforcesensordata  updateforcesensordata
	#define CHK_ElfinCollaborativeLibupdateforcesensordata  TRUE
	#define EXP_ElfinCollaborativeLibupdateforcesensordata  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensordata", (RTS_UINTPTR)updateforcesensordata, 1, 0x853A9D07, 0x01000000) 
#elif defined(CPLUSPLUS)
	#define USE_updateforcesensordata
	#define EXT_updateforcesensordata
	#define GET_updateforcesensordata(fl)  CAL_CMGETAPI( "updateforcesensordata" ) 
	#define CAL_updateforcesensordata  updateforcesensordata
	#define CHK_updateforcesensordata  TRUE
	#define EXP_updateforcesensordata  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensordata", (RTS_UINTPTR)updateforcesensordata, 1, 0x853A9D07, 0x01000000) 
#else /* DYNAMIC_LINK */
	#define USE_updateforcesensordata  PFUPDATEFORCESENSORDATA_IEC pfupdateforcesensordata;
	#define EXT_updateforcesensordata  extern PFUPDATEFORCESENSORDATA_IEC pfupdateforcesensordata;
	#define GET_updateforcesensordata(fl)  s_pfCMGetAPI2( "updateforcesensordata", (RTS_VOID_FCTPTR *)&pfupdateforcesensordata, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x853A9D07, 0x01000000)
	#define CAL_updateforcesensordata  pfupdateforcesensordata
	#define CHK_updateforcesensordata  (pfupdateforcesensordata != NULL)
	#define EXP_updateforcesensordata   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensordata", (RTS_UINTPTR)updateforcesensordata, 1, 0x853A9D07, 0x01000000) 
#endif


/**
 * <description>updatej3j5accelerometer</description>
 */
typedef struct tagupdatej3j5accelerometer_struct
{
	RTS_IEC_LREAL J3Accelerometer[3];	/* VAR_INPUT */	
	RTS_IEC_LREAL J5Accelerometer[3];	/* VAR_INPUT */	
	RTS_IEC_BOOL updateJ3J5Accelerometer;	/* VAR_OUTPUT */	
} updatej3j5accelerometer_struct;

void CDECL CDECL_EXT updatej3j5accelerometer(updatej3j5accelerometer_struct *p);
typedef void (CDECL CDECL_EXT* PFUPDATEJ3J5ACCELEROMETER_IEC) (updatej3j5accelerometer_struct *p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(UPDATEJ3J5ACCELEROMETER_NOTIMPLEMENTED)
	#define USE_updatej3j5accelerometer
	#define EXT_updatej3j5accelerometer
	#define GET_updatej3j5accelerometer(fl)  ERR_NOTIMPLEMENTED
	#define CAL_updatej3j5accelerometer(p0) 
	#define CHK_updatej3j5accelerometer  FALSE
	#define EXP_updatej3j5accelerometer  ERR_OK
#elif defined(STATIC_LINK)
	#define USE_updatej3j5accelerometer
	#define EXT_updatej3j5accelerometer
	#define GET_updatej3j5accelerometer(fl)  CAL_CMGETAPI( "updatej3j5accelerometer" ) 
	#define CAL_updatej3j5accelerometer  updatej3j5accelerometer
	#define CHK_updatej3j5accelerometer  TRUE
	#define EXP_updatej3j5accelerometer  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatej3j5accelerometer", (RTS_UINTPTR)updatej3j5accelerometer, 1, 0xFEC26347, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
	#define USE_updatej3j5accelerometer
	#define EXT_updatej3j5accelerometer
	#define GET_updatej3j5accelerometer(fl)  CAL_CMGETAPI( "updatej3j5accelerometer" ) 
	#define CAL_updatej3j5accelerometer  updatej3j5accelerometer
	#define CHK_updatej3j5accelerometer  TRUE
	#define EXP_updatej3j5accelerometer  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatej3j5accelerometer", (RTS_UINTPTR)updatej3j5accelerometer, 1, 0xFEC26347, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
	#define USE_ElfinCollaborativeLibupdatej3j5accelerometer
	#define EXT_ElfinCollaborativeLibupdatej3j5accelerometer
	#define GET_ElfinCollaborativeLibupdatej3j5accelerometer  ERR_OK
	#define CAL_ElfinCollaborativeLibupdatej3j5accelerometer  updatej3j5accelerometer
	#define CHK_ElfinCollaborativeLibupdatej3j5accelerometer  TRUE
	#define EXP_ElfinCollaborativeLibupdatej3j5accelerometer  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatej3j5accelerometer", (RTS_UINTPTR)updatej3j5accelerometer, 1, 0xFEC26347, 0x01000000) 
#elif defined(CPLUSPLUS)
	#define USE_updatej3j5accelerometer
	#define EXT_updatej3j5accelerometer
	#define GET_updatej3j5accelerometer(fl)  CAL_CMGETAPI( "updatej3j5accelerometer" ) 
	#define CAL_updatej3j5accelerometer  updatej3j5accelerometer
	#define CHK_updatej3j5accelerometer  TRUE
	#define EXP_updatej3j5accelerometer  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatej3j5accelerometer", (RTS_UINTPTR)updatej3j5accelerometer, 1, 0xFEC26347, 0x01000000) 
#else /* DYNAMIC_LINK */
	#define USE_updatej3j5accelerometer  PFUPDATEJ3J5ACCELEROMETER_IEC pfupdatej3j5accelerometer;
	#define EXT_updatej3j5accelerometer  extern PFUPDATEJ3J5ACCELEROMETER_IEC pfupdatej3j5accelerometer;
	#define GET_updatej3j5accelerometer(fl)  s_pfCMGetAPI2( "updatej3j5accelerometer", (RTS_VOID_FCTPTR *)&pfupdatej3j5accelerometer, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xFEC26347, 0x01000000)
	#define CAL_updatej3j5accelerometer  pfupdatej3j5accelerometer
	#define CHK_updatej3j5accelerometer  (pfupdatej3j5accelerometer != NULL)
	#define EXP_updatej3j5accelerometer   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatej3j5accelerometer", (RTS_UINTPTR)updatej3j5accelerometer, 1, 0xFEC26347, 0x01000000) 
#endif


/**
 * Update the manipulator joint state estimates.
 * Call this method at the update period defined on construction of the class instance.
 * VAR_INPUT [in] commandJointPositions The current command joint positions (radians)
 * VAR_INPUT [in] jointPositions The current joint positions (radians)
 * VAR_INPUT [in] motorCurrents The motor currents (amperes)
 * return True if successful and False if the class has not been properly intialized
 */
typedef struct tagupdatestateestimates_struct
{
	RTS_IEC_LREAL commandJointPositions[6];	/* VAR_INPUT */	/* command joint position(deg) */
	RTS_IEC_LREAL jointPositions[6];	/* VAR_INPUT */	/* current joint actual position(deg) */
	RTS_IEC_LREAL motorCurrents[6];		/* VAR_INPUT */	/* joint current(A) */
	RTS_IEC_LREAL currentTime;			/* VAR_INPUT */	/* current system run time； */
	RTS_IEC_BOOL updateStateEstimates;	/* VAR_OUTPUT */	
} updatestateestimates_struct;

void CDECL CDECL_EXT updatestateestimates(updatestateestimates_struct *p);
typedef void (CDECL CDECL_EXT* PFUPDATESTATEESTIMATES_IEC) (updatestateestimates_struct *p);
=======
	/**
	 * 更新关节侧和电机侧对应的关节角度
	 */
	typedef struct tagupdatedualencoderjointposition_struct
	{
		RTS_IEC_LREAL motorSideJointPosition[6];	/* VAR_INPUT */	/* actual motor side  joint position(deg) */
		RTS_IEC_LREAL jointSideJointPosition[6];	/* VAR_INPUT */	/* actual joint side joint position(deg) */
		RTS_IEC_BOOL updateDualEnCoderJointPosition;	/* VAR_OUTPUT */
	} updatedualencoderjointposition_struct;

	void CDECL CDECL_EXT updatedualencoderjointposition(updatedualencoderjointposition_struct* p);
	typedef void (CDECL CDECL_EXT* PFUPDATEDUALENCODERJOINTPOSITION_IEC) (updatedualencoderjointposition_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(UPDATEDUALENCODERJOINTPOSITION_NOTIMPLEMENTED)
#define USE_updatedualencoderjointposition
#define EXT_updatedualencoderjointposition
#define GET_updatedualencoderjointposition(fl)  ERR_NOTIMPLEMENTED
#define CAL_updatedualencoderjointposition(p0) 
#define CHK_updatedualencoderjointposition  FALSE
#define EXP_updatedualencoderjointposition  ERR_OK
#elif defined(STATIC_LINK)
#define USE_updatedualencoderjointposition
#define EXT_updatedualencoderjointposition
#define GET_updatedualencoderjointposition(fl)  CAL_CMGETAPI( "updatedualencoderjointposition" ) 
#define CAL_updatedualencoderjointposition  updatedualencoderjointposition
#define CHK_updatedualencoderjointposition  TRUE
#define EXP_updatedualencoderjointposition  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatedualencoderjointposition", (RTS_UINTPTR)updatedualencoderjointposition, 1, 0xDDEBD37A, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_updatedualencoderjointposition
#define EXT_updatedualencoderjointposition
#define GET_updatedualencoderjointposition(fl)  CAL_CMGETAPI( "updatedualencoderjointposition" ) 
#define CAL_updatedualencoderjointposition  updatedualencoderjointposition
#define CHK_updatedualencoderjointposition  TRUE
#define EXP_updatedualencoderjointposition  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatedualencoderjointposition", (RTS_UINTPTR)updatedualencoderjointposition, 1, 0xDDEBD37A, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibupdatedualencoderjointposition
#define EXT_ElfinCollaborativeLibupdatedualencoderjointposition
#define GET_ElfinCollaborativeLibupdatedualencoderjointposition  ERR_OK
#define CAL_ElfinCollaborativeLibupdatedualencoderjointposition  updatedualencoderjointposition
#define CHK_ElfinCollaborativeLibupdatedualencoderjointposition  TRUE
#define EXP_ElfinCollaborativeLibupdatedualencoderjointposition  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatedualencoderjointposition", (RTS_UINTPTR)updatedualencoderjointposition, 1, 0xDDEBD37A, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_updatedualencoderjointposition
#define EXT_updatedualencoderjointposition
#define GET_updatedualencoderjointposition(fl)  CAL_CMGETAPI( "updatedualencoderjointposition" ) 
#define CAL_updatedualencoderjointposition  updatedualencoderjointposition
#define CHK_updatedualencoderjointposition  TRUE
#define EXP_updatedualencoderjointposition  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatedualencoderjointposition", (RTS_UINTPTR)updatedualencoderjointposition, 1, 0xDDEBD37A, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_updatedualencoderjointposition  PFUPDATEDUALENCODERJOINTPOSITION_IEC pfupdatedualencoderjointposition;
#define EXT_updatedualencoderjointposition  extern PFUPDATEDUALENCODERJOINTPOSITION_IEC pfupdatedualencoderjointposition;
#define GET_updatedualencoderjointposition(fl)  s_pfCMGetAPI2( "updatedualencoderjointposition", (RTS_VOID_FCTPTR *)&pfupdatedualencoderjointposition, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xDDEBD37A, 0x01000000)
#define CAL_updatedualencoderjointposition  pfupdatedualencoderjointposition
#define CHK_updatedualencoderjointposition  (pfupdatedualencoderjointposition != NULL)
#define EXP_updatedualencoderjointposition   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatedualencoderjointposition", (RTS_UINTPTR)updatedualencoderjointposition, 1, 0xDDEBD37A, 0x01000000) 
#endif


	/**
	 * 实时更新外部轴的状态（命令位置、实际位置、换算后的检测力）
	 */
	typedef struct tagupdateexternalaxisstate_struct
	{
		RTS_IEC_LREAL commandPositions[6];	/* VAR_INPUT */	/* command external axis position */
		RTS_IEC_LREAL actualPositions[6];	/* VAR_INPUT */	/* actual external axis position */
		RTS_IEC_LREAL sensedForce[6];		/* VAR_INPUT */	/* 基于电机电流、减速比和丝杆换算得到的检测力； */
		RTS_IEC_BOOL updateExternalAxisState;	/* VAR_OUTPUT */
	} updateexternalaxisstate_struct;

	void CDECL CDECL_EXT updateexternalaxisstate(updateexternalaxisstate_struct* p);
	typedef void (CDECL CDECL_EXT* PFUPDATEEXTERNALAXISSTATE_IEC) (updateexternalaxisstate_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(UPDATEEXTERNALAXISSTATE_NOTIMPLEMENTED)
#define USE_updateexternalaxisstate
#define EXT_updateexternalaxisstate
#define GET_updateexternalaxisstate(fl)  ERR_NOTIMPLEMENTED
#define CAL_updateexternalaxisstate(p0) 
#define CHK_updateexternalaxisstate  FALSE
#define EXP_updateexternalaxisstate  ERR_OK
#elif defined(STATIC_LINK)
#define USE_updateexternalaxisstate
#define EXT_updateexternalaxisstate
#define GET_updateexternalaxisstate(fl)  CAL_CMGETAPI( "updateexternalaxisstate" ) 
#define CAL_updateexternalaxisstate  updateexternalaxisstate
#define CHK_updateexternalaxisstate  TRUE
#define EXP_updateexternalaxisstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateexternalaxisstate", (RTS_UINTPTR)updateexternalaxisstate, 1, 0xF276A89F, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_updateexternalaxisstate
#define EXT_updateexternalaxisstate
#define GET_updateexternalaxisstate(fl)  CAL_CMGETAPI( "updateexternalaxisstate" ) 
#define CAL_updateexternalaxisstate  updateexternalaxisstate
#define CHK_updateexternalaxisstate  TRUE
#define EXP_updateexternalaxisstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateexternalaxisstate", (RTS_UINTPTR)updateexternalaxisstate, 1, 0xF276A89F, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibupdateexternalaxisstate
#define EXT_ElfinCollaborativeLibupdateexternalaxisstate
#define GET_ElfinCollaborativeLibupdateexternalaxisstate  ERR_OK
#define CAL_ElfinCollaborativeLibupdateexternalaxisstate  updateexternalaxisstate
#define CHK_ElfinCollaborativeLibupdateexternalaxisstate  TRUE
#define EXP_ElfinCollaborativeLibupdateexternalaxisstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateexternalaxisstate", (RTS_UINTPTR)updateexternalaxisstate, 1, 0xF276A89F, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_updateexternalaxisstate
#define EXT_updateexternalaxisstate
#define GET_updateexternalaxisstate(fl)  CAL_CMGETAPI( "updateexternalaxisstate" ) 
#define CAL_updateexternalaxisstate  updateexternalaxisstate
#define CHK_updateexternalaxisstate  TRUE
#define EXP_updateexternalaxisstate  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateexternalaxisstate", (RTS_UINTPTR)updateexternalaxisstate, 1, 0xF276A89F, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_updateexternalaxisstate  PFUPDATEEXTERNALAXISSTATE_IEC pfupdateexternalaxisstate;
#define EXT_updateexternalaxisstate  extern PFUPDATEEXTERNALAXISSTATE_IEC pfupdateexternalaxisstate;
#define GET_updateexternalaxisstate(fl)  s_pfCMGetAPI2( "updateexternalaxisstate", (RTS_VOID_FCTPTR *)&pfupdateexternalaxisstate, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xF276A89F, 0x01000000)
#define CAL_updateexternalaxisstate  pfupdateexternalaxisstate
#define CHK_updateexternalaxisstate  (pfupdateexternalaxisstate != NULL)
#define EXP_updateexternalaxisstate   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateexternalaxisstate", (RTS_UINTPTR)updateexternalaxisstate, 1, 0xF276A89F, 0x01000000) 
#endif


	/**
	 * <description>updateforcesensordata</description>
	 */
	typedef struct tagupdateforcesensordata_struct
	{
		RTS_IEC_BOOL enable;				/* VAR_INPUT */	/* 是否开启末端力传感器的碰撞检测功能； */
		RTS_IEC_LREAL calibedForce[6];		/* VAR_INPUT */	/* 获取标定后的力传感器数据，当前为：GetFTValueInSelectedFrame */
		RTS_IEC_BOOL updateForceSensorData;	/* VAR_OUTPUT */
	} updateforcesensordata_struct;

	void CDECL CDECL_EXT updateforcesensordata(updateforcesensordata_struct* p);
	typedef void (CDECL CDECL_EXT* PFUPDATEFORCESENSORDATA_IEC) (updateforcesensordata_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(UPDATEFORCESENSORDATA_NOTIMPLEMENTED)
#define USE_updateforcesensordata
#define EXT_updateforcesensordata
#define GET_updateforcesensordata(fl)  ERR_NOTIMPLEMENTED
#define CAL_updateforcesensordata(p0) 
#define CHK_updateforcesensordata  FALSE
#define EXP_updateforcesensordata  ERR_OK
#elif defined(STATIC_LINK)
#define USE_updateforcesensordata
#define EXT_updateforcesensordata
#define GET_updateforcesensordata(fl)  CAL_CMGETAPI( "updateforcesensordata" ) 
#define CAL_updateforcesensordata  updateforcesensordata
#define CHK_updateforcesensordata  TRUE
#define EXP_updateforcesensordata  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensordata", (RTS_UINTPTR)updateforcesensordata, 1, 0x853A9D07, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_updateforcesensordata
#define EXT_updateforcesensordata
#define GET_updateforcesensordata(fl)  CAL_CMGETAPI( "updateforcesensordata" ) 
#define CAL_updateforcesensordata  updateforcesensordata
#define CHK_updateforcesensordata  TRUE
#define EXP_updateforcesensordata  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensordata", (RTS_UINTPTR)updateforcesensordata, 1, 0x853A9D07, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibupdateforcesensordata
#define EXT_ElfinCollaborativeLibupdateforcesensordata
#define GET_ElfinCollaborativeLibupdateforcesensordata  ERR_OK
#define CAL_ElfinCollaborativeLibupdateforcesensordata  updateforcesensordata
#define CHK_ElfinCollaborativeLibupdateforcesensordata  TRUE
#define EXP_ElfinCollaborativeLibupdateforcesensordata  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensordata", (RTS_UINTPTR)updateforcesensordata, 1, 0x853A9D07, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_updateforcesensordata
#define EXT_updateforcesensordata
#define GET_updateforcesensordata(fl)  CAL_CMGETAPI( "updateforcesensordata" ) 
#define CAL_updateforcesensordata  updateforcesensordata
#define CHK_updateforcesensordata  TRUE
#define EXP_updateforcesensordata  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensordata", (RTS_UINTPTR)updateforcesensordata, 1, 0x853A9D07, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_updateforcesensordata  PFUPDATEFORCESENSORDATA_IEC pfupdateforcesensordata;
#define EXT_updateforcesensordata  extern PFUPDATEFORCESENSORDATA_IEC pfupdateforcesensordata;
#define GET_updateforcesensordata(fl)  s_pfCMGetAPI2( "updateforcesensordata", (RTS_VOID_FCTPTR *)&pfupdateforcesensordata, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x853A9D07, 0x01000000)
#define CAL_updateforcesensordata  pfupdateforcesensordata
#define CHK_updateforcesensordata  (pfupdateforcesensordata != NULL)
#define EXP_updateforcesensordata   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensordata", (RTS_UINTPTR)updateforcesensordata, 1, 0x853A9D07, 0x01000000) 
#endif


	/**
	 * <description>updateforcesensorforfocecontrol</description>
	 */
	typedef struct tagupdateforcesensorforfocecontrol_struct
	{
		RTS_IEC_BOOL enable;				/* VAR_INPUT */	/* 在电流前馈中，引入力传感器的作用力 */
		RTS_IEC_LREAL calibedForce[6];		/* VAR_INPUT */	/* 获取标定后的力传感器数据，当前为：updateFTSensorData => processedForce; */
		RTS_IEC_BOOL updateForceSensorForFoceControl;	/* VAR_OUTPUT */
	} updateforcesensorforfocecontrol_struct;

	void CDECL CDECL_EXT updateforcesensorforfocecontrol(updateforcesensorforfocecontrol_struct* p);
	typedef void (CDECL CDECL_EXT* PFUPDATEFORCESENSORFORFOCECONTROL_IEC) (updateforcesensorforfocecontrol_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(UPDATEFORCESENSORFORFOCECONTROL_NOTIMPLEMENTED)
#define USE_updateforcesensorforfocecontrol
#define EXT_updateforcesensorforfocecontrol
#define GET_updateforcesensorforfocecontrol(fl)  ERR_NOTIMPLEMENTED
#define CAL_updateforcesensorforfocecontrol(p0) 
#define CHK_updateforcesensorforfocecontrol  FALSE
#define EXP_updateforcesensorforfocecontrol  ERR_OK
#elif defined(STATIC_LINK)
#define USE_updateforcesensorforfocecontrol
#define EXT_updateforcesensorforfocecontrol
#define GET_updateforcesensorforfocecontrol(fl)  CAL_CMGETAPI( "updateforcesensorforfocecontrol" ) 
#define CAL_updateforcesensorforfocecontrol  updateforcesensorforfocecontrol
#define CHK_updateforcesensorforfocecontrol  TRUE
#define EXP_updateforcesensorforfocecontrol  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensorforfocecontrol", (RTS_UINTPTR)updateforcesensorforfocecontrol, 1, 0x7DF51220, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_updateforcesensorforfocecontrol
#define EXT_updateforcesensorforfocecontrol
#define GET_updateforcesensorforfocecontrol(fl)  CAL_CMGETAPI( "updateforcesensorforfocecontrol" ) 
#define CAL_updateforcesensorforfocecontrol  updateforcesensorforfocecontrol
#define CHK_updateforcesensorforfocecontrol  TRUE
#define EXP_updateforcesensorforfocecontrol  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensorforfocecontrol", (RTS_UINTPTR)updateforcesensorforfocecontrol, 1, 0x7DF51220, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibupdateforcesensorforfocecontrol
#define EXT_ElfinCollaborativeLibupdateforcesensorforfocecontrol
#define GET_ElfinCollaborativeLibupdateforcesensorforfocecontrol  ERR_OK
#define CAL_ElfinCollaborativeLibupdateforcesensorforfocecontrol  updateforcesensorforfocecontrol
#define CHK_ElfinCollaborativeLibupdateforcesensorforfocecontrol  TRUE
#define EXP_ElfinCollaborativeLibupdateforcesensorforfocecontrol  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensorforfocecontrol", (RTS_UINTPTR)updateforcesensorforfocecontrol, 1, 0x7DF51220, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_updateforcesensorforfocecontrol
#define EXT_updateforcesensorforfocecontrol
#define GET_updateforcesensorforfocecontrol(fl)  CAL_CMGETAPI( "updateforcesensorforfocecontrol" ) 
#define CAL_updateforcesensorforfocecontrol  updateforcesensorforfocecontrol
#define CHK_updateforcesensorforfocecontrol  TRUE
#define EXP_updateforcesensorforfocecontrol  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensorforfocecontrol", (RTS_UINTPTR)updateforcesensorforfocecontrol, 1, 0x7DF51220, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_updateforcesensorforfocecontrol  PFUPDATEFORCESENSORFORFOCECONTROL_IEC pfupdateforcesensorforfocecontrol;
#define EXT_updateforcesensorforfocecontrol  extern PFUPDATEFORCESENSORFORFOCECONTROL_IEC pfupdateforcesensorforfocecontrol;
#define GET_updateforcesensorforfocecontrol(fl)  s_pfCMGetAPI2( "updateforcesensorforfocecontrol", (RTS_VOID_FCTPTR *)&pfupdateforcesensorforfocecontrol, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x7DF51220, 0x01000000)
#define CAL_updateforcesensorforfocecontrol  pfupdateforcesensorforfocecontrol
#define CHK_updateforcesensorforfocecontrol  (pfupdateforcesensorforfocecontrol != NULL)
#define EXP_updateforcesensorforfocecontrol   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updateforcesensorforfocecontrol", (RTS_UINTPTR)updateforcesensorforfocecontrol, 1, 0x7DF51220, 0x01000000) 
#endif


	/**
	 * <description>updatej3j5accelerometer</description>
	 */
	typedef struct tagupdatej3j5accelerometer_struct
	{
		RTS_IEC_LREAL J3Accelerometer[3];	/* VAR_INPUT */
		RTS_IEC_LREAL J5Accelerometer[3];	/* VAR_INPUT */
		RTS_IEC_BOOL updateJ3J5Accelerometer;	/* VAR_OUTPUT */
	} updatej3j5accelerometer_struct;

	void CDECL CDECL_EXT updatej3j5accelerometer(updatej3j5accelerometer_struct* p);
	typedef void (CDECL CDECL_EXT* PFUPDATEJ3J5ACCELEROMETER_IEC) (updatej3j5accelerometer_struct* p);
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(UPDATEJ3J5ACCELEROMETER_NOTIMPLEMENTED)
#define USE_updatej3j5accelerometer
#define EXT_updatej3j5accelerometer
#define GET_updatej3j5accelerometer(fl)  ERR_NOTIMPLEMENTED
#define CAL_updatej3j5accelerometer(p0) 
#define CHK_updatej3j5accelerometer  FALSE
#define EXP_updatej3j5accelerometer  ERR_OK
#elif defined(STATIC_LINK)
#define USE_updatej3j5accelerometer
#define EXT_updatej3j5accelerometer
#define GET_updatej3j5accelerometer(fl)  CAL_CMGETAPI( "updatej3j5accelerometer" ) 
#define CAL_updatej3j5accelerometer  updatej3j5accelerometer
#define CHK_updatej3j5accelerometer  TRUE
#define EXP_updatej3j5accelerometer  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatej3j5accelerometer", (RTS_UINTPTR)updatej3j5accelerometer, 1, 0xFEC26347, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_updatej3j5accelerometer
#define EXT_updatej3j5accelerometer
#define GET_updatej3j5accelerometer(fl)  CAL_CMGETAPI( "updatej3j5accelerometer" ) 
#define CAL_updatej3j5accelerometer  updatej3j5accelerometer
#define CHK_updatej3j5accelerometer  TRUE
#define EXP_updatej3j5accelerometer  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatej3j5accelerometer", (RTS_UINTPTR)updatej3j5accelerometer, 1, 0xFEC26347, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibupdatej3j5accelerometer
#define EXT_ElfinCollaborativeLibupdatej3j5accelerometer
#define GET_ElfinCollaborativeLibupdatej3j5accelerometer  ERR_OK
#define CAL_ElfinCollaborativeLibupdatej3j5accelerometer  updatej3j5accelerometer
#define CHK_ElfinCollaborativeLibupdatej3j5accelerometer  TRUE
#define EXP_ElfinCollaborativeLibupdatej3j5accelerometer  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatej3j5accelerometer", (RTS_UINTPTR)updatej3j5accelerometer, 1, 0xFEC26347, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_updatej3j5accelerometer
#define EXT_updatej3j5accelerometer
#define GET_updatej3j5accelerometer(fl)  CAL_CMGETAPI( "updatej3j5accelerometer" ) 
#define CAL_updatej3j5accelerometer  updatej3j5accelerometer
#define CHK_updatej3j5accelerometer  TRUE
#define EXP_updatej3j5accelerometer  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatej3j5accelerometer", (RTS_UINTPTR)updatej3j5accelerometer, 1, 0xFEC26347, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_updatej3j5accelerometer  PFUPDATEJ3J5ACCELEROMETER_IEC pfupdatej3j5accelerometer;
#define EXT_updatej3j5accelerometer  extern PFUPDATEJ3J5ACCELEROMETER_IEC pfupdatej3j5accelerometer;
#define GET_updatej3j5accelerometer(fl)  s_pfCMGetAPI2( "updatej3j5accelerometer", (RTS_VOID_FCTPTR *)&pfupdatej3j5accelerometer, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0xFEC26347, 0x01000000)
#define CAL_updatej3j5accelerometer  pfupdatej3j5accelerometer
#define CHK_updatej3j5accelerometer  (pfupdatej3j5accelerometer != NULL)
#define EXP_updatej3j5accelerometer   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatej3j5accelerometer", (RTS_UINTPTR)updatej3j5accelerometer, 1, 0xFEC26347, 0x01000000) 
#endif


	/**
	 * Update the manipulator joint state estimates.
	 * Call this method at the update period defined on construction of the class instance.
	 * VAR_INPUT [in] commandJointPositions The current command joint positions (radians)
	 * VAR_INPUT [in] jointPositions The current joint positions (radians)
	 * VAR_INPUT [in] motorCurrents The motor currents (amperes)
	 * return True if successful and False if the class has not been properly intialized
	 */
	typedef struct tagupdatestateestimates_struct
	{
		RTS_IEC_LREAL commandJointPositions[6];	/* VAR_INPUT */	/* command joint position(deg) */
		RTS_IEC_LREAL jointPositions[6];	/* VAR_INPUT */	/* current joint actual position(deg) */
		RTS_IEC_LREAL motorCurrents[6];		/* VAR_INPUT */	/* joint current(A) */
		RTS_IEC_LREAL currentTime;			/* VAR_INPUT */	/* current system run time； */
		RTS_IEC_BOOL updateStateEstimates;	/* VAR_OUTPUT */
	} updatestateestimates_struct;

	void CDECL CDECL_EXT updatestateestimates(updatestateestimates_struct* p);
	typedef void (CDECL CDECL_EXT* PFUPDATESTATEESTIMATES_IEC) (updatestateestimates_struct* p);
>>>>>>> 40f7afc7711530af2c9319aaedf0d2aa15dee117
#if defined(ELFINCOLLABORATIVELIB_NOTIMPLEMENTED) || defined(UPDATESTATEESTIMATES_NOTIMPLEMENTED)
#define USE_updatestateestimates
#define EXT_updatestateestimates
#define GET_updatestateestimates(fl)  ERR_NOTIMPLEMENTED
#define CAL_updatestateestimates(p0) 
#define CHK_updatestateestimates  FALSE
#define EXP_updatestateestimates  ERR_OK
#elif defined(STATIC_LINK)
#define USE_updatestateestimates
#define EXT_updatestateestimates
#define GET_updatestateestimates(fl)  CAL_CMGETAPI( "updatestateestimates" ) 
#define CAL_updatestateestimates  updatestateestimates
#define CHK_updatestateestimates  TRUE
#define EXP_updatestateestimates  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatestateestimates", (RTS_UINTPTR)updatestateestimates, 1, 0x0D0F1BB6, 0x01000000) 
#elif defined(MIXED_LINK) && !defined(ELFINCOLLABORATIVELIB_EXTERNAL)
#define USE_updatestateestimates
#define EXT_updatestateestimates
#define GET_updatestateestimates(fl)  CAL_CMGETAPI( "updatestateestimates" ) 
#define CAL_updatestateestimates  updatestateestimates
#define CHK_updatestateestimates  TRUE
#define EXP_updatestateestimates  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatestateestimates", (RTS_UINTPTR)updatestateestimates, 1, 0x0D0F1BB6, 0x01000000) 
#elif defined(CPLUSPLUS_ONLY)
#define USE_ElfinCollaborativeLibupdatestateestimates
#define EXT_ElfinCollaborativeLibupdatestateestimates
#define GET_ElfinCollaborativeLibupdatestateestimates  ERR_OK
#define CAL_ElfinCollaborativeLibupdatestateestimates  updatestateestimates
#define CHK_ElfinCollaborativeLibupdatestateestimates  TRUE
#define EXP_ElfinCollaborativeLibupdatestateestimates  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatestateestimates", (RTS_UINTPTR)updatestateestimates, 1, 0x0D0F1BB6, 0x01000000) 
#elif defined(CPLUSPLUS)
#define USE_updatestateestimates
#define EXT_updatestateestimates
#define GET_updatestateestimates(fl)  CAL_CMGETAPI( "updatestateestimates" ) 
#define CAL_updatestateestimates  updatestateestimates
#define CHK_updatestateestimates  TRUE
#define EXP_updatestateestimates  s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatestateestimates", (RTS_UINTPTR)updatestateestimates, 1, 0x0D0F1BB6, 0x01000000) 
#else /* DYNAMIC_LINK */
#define USE_updatestateestimates  PFUPDATESTATEESTIMATES_IEC pfupdatestateestimates;
#define EXT_updatestateestimates  extern PFUPDATESTATEESTIMATES_IEC pfupdatestateestimates;
#define GET_updatestateestimates(fl)  s_pfCMGetAPI2( "updatestateestimates", (RTS_VOID_FCTPTR *)&pfupdatestateestimates, (fl) | CM_IMPORT_EXTERNAL_LIB_FUNCTION, 0x0D0F1BB6, 0x01000000)
#define CAL_updatestateestimates  pfupdatestateestimates
#define CHK_updatestateestimates  (pfupdatestateestimates != NULL)
#define EXP_updatestateestimates   s_pfCMRegisterAPI2( (const CMP_EXT_FUNCTION_REF*)"updatestateestimates", (RTS_UINTPTR)updatestateestimates, 1, 0x0D0F1BB6, 0x01000000) 
#endif


#ifdef __cplusplus
}
#endif

/** EXTERN LIB SECTION END **/




typedef struct
{
	IBase_C* pBase;
} IElfinCollaborativeLib_C;

#ifdef CPLUSPLUS
class IElfinCollaborativeLib : public IBase
{
public:
};
#ifndef ITF_ElfinCollaborativeLib
#define ITF_ElfinCollaborativeLib static IElfinCollaborativeLib *pIElfinCollaborativeLib = NULL;
#endif
#define EXTITF_ElfinCollaborativeLib
#else	/*CPLUSPLUS*/
typedef IElfinCollaborativeLib_C		IElfinCollaborativeLib;
#ifndef ITF_ElfinCollaborativeLib
#define ITF_ElfinCollaborativeLib
#endif
#define EXTITF_ElfinCollaborativeLib
#endif

#ifdef CPLUSPLUS_ONLY
#undef CPLUSPLUS_ONLY
#endif

#endif /*_ELFINCOLLABORATIVELIBITF_H_*/
