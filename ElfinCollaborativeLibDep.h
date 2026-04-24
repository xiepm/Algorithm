/***********************************************************************
 * DO NOT MODIFY!
 * This is a generated file. Do not modify it's contents directly
 ***********************************************************************/

/**
 *  <name>Component Template</name>
 *  <description> 
 *  An example on how to implement a component.
 *  This component does no usefull work and it exports no functions
 *  which are intended to be used for anything. Use at your own risk.
 *  </description>
 *  <copyright>
 *  (c) 2003-2017 3S-Smart Software Solutions
 *  </copyright>
 */
#ifndef _ELFINCOLLABORATIVELIBDEP_H_
#define _ELFINCOLLABORATIVELIBDEP_H_

#define COMPONENT_NAME "ElfinCollaborativeLib"COMPONENT_NAME_POSTFIX
#define COMPONENT_ID    ADDVENDORID(CMP_VENDORID, CMPID_ElfinCollaborativeLib)
#define COMPONENT_NAME_UNQUOTED ElfinCollaborativeLib






#define CMP_VERSION         UINT32_C(0x01000000)
#define CMP_VERSION_STRING "1.0.0.0"
#define CMP_VERSION_RC      1,0,0,0

/* NOTE: REPLACE 0x0001 BY YOUR VENDORID */
#define CMP_VENDORID       0x0001

#ifndef WIN32_RESOURCES

#include "CmpLogItf.h"
#include "CMUtilsItf.h"				

#define CMPID_ElfinCollaborativeLib		0x2102								/* NOTE: START HERE WITH YOUR COMPONENTIDS (see CmpItf.h */
#define CLASSID_CElfinCollaborativeLib	ADDVENDORID(CMP_VENDORID, 0x2102)	/* NOTE: START HERE WITH YOUR CLASSIDS (see CmpItf.h */
#define ITFID_IElfinCollaborativeLib		ADDVENDORID(CMP_VENDORID, 0x2102)	/* NOTE: START HERE WITH YOUR INTERFACEIDS (see CmpItf.h */


/*Obsolete include: CMUtilsItf.m4*/


#include "SysFileItf.h"







#include "ElfinCollaborativeLibItf.h"







    
     


    



      



     


#ifdef CPLUSPLUS
    #define INIT_STMT \
    {\
        IBase *pIBase;\
        RTS_RESULT initResult;\
        if (pICmpLog == NULL && s_pfCMCreateInstance != NULL) \
        { \
            pIBase = (IBase *)s_pfCMCreateInstance(CLASSID_CCmpLog, &initResult); \
            if (pIBase != NULL) \
            { \
                pICmpLog = (ICmpLog *)pIBase->QueryInterface(pIBase, ITFID_ICmpLog, &initResult); \
                pIBase->Release(pIBase); \
            } \
        } \
        if (pICMUtils == NULL && s_pfCMCreateInstance != NULL) \
        { \
            pIBase = (IBase *)s_pfCMCreateInstance(CLASSID_CCMUtils, &initResult); \
            if (pIBase != NULL) \
            { \
                pICMUtils = (ICMUtils *)pIBase->QueryInterface(pIBase, ITFID_ICMUtils, &initResult); \
                pIBase->Release(pIBase); \
            } \
        } \
        if (pISysFile == NULL && s_pfCMCreateInstance != NULL) \
        { \
            pIBase = (IBase *)s_pfCMCreateInstance(CLASSID_CSysFile, &initResult); \
            if (pIBase != NULL) \
            { \
                pISysFile = (ISysFile *)pIBase->QueryInterface(pIBase, ITFID_ISysFile, &initResult); \
                pIBase->Release(pIBase); \
            } \
        } \
          /*Obsolete include CMUtils*/ \
		   \
    }
    #define INIT_LOCALS_STMT \
    {\
        pICmpLog = NULL; \
        pICMUtils = NULL; \
        pISysFile = NULL; \
          /*Obsolete include CMUtils*/ \
		   \
    }
    #define EXIT_STMT \
    {\
        IBase *pIBase;\
        RTS_RESULT exitResult;\
        if (pICmpLog != NULL) \
        { \
            pIBase = (IBase *)pICmpLog->QueryInterface(pICmpLog, ITFID_IBase, &exitResult); \
            if (pIBase != NULL) \
            { \
                 pIBase->Release(pIBase); \
                 if (pIBase->Release(pIBase) == 0) /* The object will be deleted here! */ \
                    pICmpLog = NULL; \
            } \
        } \
        if (pICMUtils != NULL) \
        { \
            pIBase = (IBase *)pICMUtils->QueryInterface(pICMUtils, ITFID_IBase, &exitResult); \
            if (pIBase != NULL) \
            { \
                 pIBase->Release(pIBase); \
                 if (pIBase->Release(pIBase) == 0) /* The object will be deleted here! */ \
                    pICMUtils = NULL; \
            } \
        } \
        if (pISysFile != NULL) \
        { \
            pIBase = (IBase *)pISysFile->QueryInterface(pISysFile, ITFID_IBase, &exitResult); \
            if (pIBase != NULL) \
            { \
                 pIBase->Release(pIBase); \
                 if (pIBase->Release(pIBase) == 0) /* The object will be deleted here! */ \
                    pISysFile = NULL; \
            } \
        } \
          /*Obsolete include CMUtils*/ \
		   \
    }
#else
    #define INIT_STMT
    #define INIT_LOCALS_STMT
    #define EXIT_STMT
#endif



#if defined(STATIC_LINK)
    #define IMPORT_STMT
#else
    #define IMPORT_STMT \
    {\
        RTS_RESULT importResult = ERR_OK;\
        RTS_RESULT TempResult = ERR_OK;\
        INIT_STMT   \
        TempResult = GET_LogAdd(CM_IMPORT_OPTIONAL_FUNCTION); \
        TempResult = GET_CMUtlMemCpy(CM_IMPORT_OPTIONAL_FUNCTION); \
        if (ERR_OK == importResult ) TempResult = GET_SysFileWrite(CM_IMPORT_OPTIONAL_FUNCTION);\
          if (ERR_OK == importResult ) TempResult = GET_SysFileRead(CM_IMPORT_OPTIONAL_FUNCTION);\
          if (ERR_OK == importResult ) TempResult = GET_SysFileClose(CM_IMPORT_OPTIONAL_FUNCTION);\
          if (ERR_OK == importResult ) TempResult = GET_SysFileOpen(CM_IMPORT_OPTIONAL_FUNCTION);\
          if (ERR_OK == importResult ) importResult = GET_CMUtlSafeStrCpy(0);\
           \
        /* To make LINT happy */\
        TempResult = TempResult;\
        if (ERR_OK != importResult) return importResult;\
    }
#endif



#ifndef ELFINCOLLABORATIVELIB_DISABLE_EXTREF
#define EXPORT_EXTREF_STMT \
        { (RTS_VOID_FCTPTR)updatestateestimates, "updatestateestimates", 0x0D0F1BB6, 0x01000000 },\
          { (RTS_VOID_FCTPTR)updatej3j5accelerometer, "updatej3j5accelerometer", 0xFEC26347, 0x01000000 },\
          { (RTS_VOID_FCTPTR)updateforcesensordata, "updateforcesensordata", 0x853A9D07, 0x01000000 },\
          { (RTS_VOID_FCTPTR)updateexternalaxisstate, "updateexternalaxisstate", 0xF276A89F, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setstarttimeforcheckassistivemode, "setstarttimeforcheckassistivemode", 0xF95F41F2, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setstartassistivemode, "setstartassistivemode", 0x018FD046, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setrobotmountingangles, "setrobotmountingangles", 0x774360BE, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setreadytocloseassistivemode, "setreadytocloseassistivemode", 0x9BED25DA, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setpayloadmassproperties, "setpayloadmassproperties", 0xAC8C023C, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setmotionlimitindriveback, "setmotionlimitindriveback", 0x2E0BC873, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setmaxpowerandmomentumconstraints, "setmaxpowerandmomentumconstraints", 0x8FC378F7, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setmaxjointvelocity, "setmaxjointvelocity", 0x17E70001, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setlowvelocitythreshold, "setlowvelocitythreshold", 0x2BE2E169, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setfrictioncompensatoryfactorii, "setfrictioncompensatoryfactorii", 0x6EA50A68, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setfrictioncompensatoryfactor, "setfrictioncompensatoryfactor", 0xF0CFD1B6, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setdynamicslinearparameters, "setdynamicslinearparameters", 0xC0B0FD1C, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setcollisionstopthresholds, "setcollisionstopthresholds", 0x6BE01113, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setcollisionstopinmomentumthresholds, "setcollisionstopinmomentumthresholds", 0xDC15365C, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setcollisiondrivebackmode, "setcollisiondrivebackmode", 0x35465C30, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setcollaborativejointspacelimits, "setcollaborativejointspacelimits", 0xC7F0193C, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setassistivemodecollisionstopthresholds, "setassistivemodecollisionstopthresholds", 0x95CB564B, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setactutortorqueconstants, "setactutortorqueconstants", 0x995D6518, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setactutormaxcurrentlimits, "setactutormaxcurrentlimits", 0x51F9F45A, 0x01000000 },\
          { (RTS_VOID_FCTPTR)setactutordampconstants, "setactutordampconstants", 0x5F20C2CB, 0x01000000 },\
          { (RTS_VOID_FCTPTR)resetcobotstatus, "resetcobotstatus", 0xA7FAE663, 0x01000000 },\
          { (RTS_VOID_FCTPTR)monitormotionconstraintstatus, "monitormotionconstraintstatus", 0x88062ACE, 0x01000000 },\
          { (RTS_VOID_FCTPTR)initializecollaborativealgorithm, "initializecollaborativealgorithm", 0xBFF655A5, 0x01000000 },\
          { (RTS_VOID_FCTPTR)getsettingparameters, "getsettingparameters", 0x0180F8FE, 0x01000000 },\
          { (RTS_VOID_FCTPTR)getgravitytorquecurrentcommands, "getgravitytorquecurrentcommands", 0x30D8A81C, 0x01000000 },\
          { (RTS_VOID_FCTPTR)getflexiblecompensatetorque, "getflexiblecompensatetorque", 0x27D3DE2B, 0x01000000 },\
          { (RTS_VOID_FCTPTR)getestimatedstate, "getestimatedstate", 0x49A2CB2A, 0x01000000 },\
          { (RTS_VOID_FCTPTR)getcomputetorquecurrentcommands, "getcomputetorquecurrentcommands", 0x047F1D78, 0x01000000 },\
          { (RTS_VOID_FCTPTR)getassistivemodemotorcurrentcommands, "getassistivemodemotorcurrentcommands", 0x35D3E6EC, 0x01000000 },\
          { (RTS_VOID_FCTPTR)checkforcollision, "checkforcollision", 0xE9568F15, 0x01000000 },\
          { (RTS_VOID_FCTPTR)calculatepowerandmomentumconstraints, "calculatepowerandmomentumconstraints", 0x4FA4BE9E, 0x01000000 },\
          
#else
#define EXPORT_EXTREF_STMT
#endif
#ifndef ELFINCOLLABORATIVELIB_DISABLE_EXTREF2
#define EXPORT_EXTREF2_STMT \
                                                                              
#else
#define EXPORT_EXTREF2_STMT
#endif
#if !defined(STATIC_LINK) && !defined(CPLUSPLUS) && !defined(CPLUSPLUS_ONLY)
#define EXPORT_CMPITF_STMT \
    {\
                                                                              \
        { ((RTS_VOID_FCTPTR)(void *)0), "", 0, 0 }\
    }
#else
#define EXPORT_CMPITF_STMT \
    {\
        { ((RTS_VOID_FCTPTR)(void *)0), "", 0, 0 }\
    }
#endif
#define EXPORT_CPP_STMT


#if defined(STATIC_LINK)
    #define EXPORT_STMT\
    {\
        RTS_RESULT ExpResult;\
        if (NULL == s_pfCMRegisterAPI)\
            return ERR_NOTINITIALIZED;\
        ExpResult = s_pfCMRegisterAPI(s_ExternalsTable, 0, 1, COMPONENT_ID);\
        if (ERR_OK != ExpResult)\
            return ExpResult;\
    }
#else
    #define EXPORT_STMT\
    {\
        RTS_RESULT ExpResult;\
        if (NULL == s_pfCMRegisterAPI)\
            return ERR_NOTINITIALIZED;\
        ExpResult = s_pfCMRegisterAPI(s_ExternalsTable, 0, 1, COMPONENT_ID);\
        if (ERR_OK != ExpResult)\
            return ExpResult;\
        ExpResult = s_pfCMRegisterAPI(s_ItfTable, 0, 0, COMPONENT_ID);\
        if (ERR_OK != ExpResult)\
            return ExpResult;\
    }
#endif

#define USE_STMT \
    /*lint -save --e{528} --e{551} */ \
    static volatile PF_REGISTER_API s_pfCMRegisterAPI; \
    static volatile PF_REGISTER_API2 s_pfCMRegisterAPI2; \
    static volatile PF_GET_API s_pfCMGetAPI; \
    static volatile PF_GET_API2 s_pfCMGetAPI2; \
    static volatile PF_REGISTER_CLASS s_pfCMRegisterClass; \
    static volatile PF_CREATEINSTANCE s_pfCMCreateInstance; \
    static volatile PF_CALL_HOOK s_pfCMCallHook; \
    static const CMP_EXT_FUNCTION_REF s_ExternalsTable[] =\
    {\
        EXPORT_EXTREF_STMT\
        EXPORT_EXTREF2_STMT\
        { ((RTS_VOID_FCTPTR)(void *)0), "", 0, 0 }\
    };\
    static const CMP_EXT_FUNCTION_REF s_ItfTable[] = EXPORT_CMPITF_STMT; \
    /*lint -restore */  \
    static int CDECL ExportFunctions(void); \
    static int CDECL ImportFunctions(void); \
    static IBase* CDECL CreateInstance(CLASSID cid, RTS_RESULT *pResult); \
    static RTS_RESULT CDECL DeleteInstance(IBase *pIBase); \
    static RTS_UI32 CDECL CmpGetVersion(void); \
    static RTS_RESULT CDECL HookFunction(RTS_UI32 ulHook, RTS_UINTPTR ulParam1, RTS_UINTPTR ulParam2); \
    ITF_CmpLog   \
    ITF_CMUtils  \
    USE_CMUtlMemCpy  \
    USE_LogAdd \
	/*obsolete entry ITF_CMUtils*/      \
	ITF_SysFile      \
    USE_CMUtlSafeStrCpy      \
    USE_SysFileOpen      \
    USE_SysFileClose      \
    USE_SysFileRead      \
    USE_SysFileWrite     
#define USEIMPORT_STMT \
    /*lint -save --e{551} */ \
    static volatile PF_REGISTER_API s_pfCMRegisterAPI; \
    static volatile PF_REGISTER_API2 s_pfCMRegisterAPI2; \
    static volatile PF_GET_API s_pfCMGetAPI; \
    static volatile PF_GET_API2 s_pfCMGetAPI2; \
    static volatile PF_REGISTER_CLASS s_pfCMRegisterClass; \
    static volatile PF_CREATEINSTANCE s_pfCMCreateInstance; \
    static volatile PF_CALL_HOOK s_pfCMCallHook; \
    /*lint -restore */  \
    ITF_CmpLog   \
    ITF_CMUtils  \
    USE_CMUtlMemCpy  \
    USE_LogAdd \
	/*obsolete entry ITF_CMUtils*/     \
	ITF_SysFile     \
    USE_CMUtlSafeStrCpy      \
    USE_SysFileOpen      \
    USE_SysFileClose      \
    USE_SysFileRead      \
    USE_SysFileWrite     
#define USEEXTERN_STMT \
    EXT_CMUtlMemCpy  \
    EXT_LogAdd \
	/*obsolete entry EXTITF_CMUtils*/     \
	EXTITF_SysFile     \
    EXT_CMUtlSafeStrCpy  \
    EXT_SysFileOpen  \
    EXT_SysFileClose  \
    EXT_SysFileRead  \
    EXT_SysFileWrite 
#ifndef COMPONENT_NAME
    #error COMPONENT_NAME is not defined. This prevents the component from being linked statically. Use SET_COMPONENT_NAME(<name_of_your_component>) to set the name of the component in your .m4 component description.
#endif




#if defined(STATIC_LINK) || defined(MIXED_LINK) || defined(DYNAMIC_LINK) || defined(CPLUSPLUS_STATIC_LINK)
    #define ComponentEntry ElfinCollaborativeLib__Entry
#endif


#ifdef CPLUSPLUS

class CElfinCollaborativeLib : public IElfinCollaborativeLib 
{
    public:
        CElfinCollaborativeLib() : hElfinCollaborativeLib(RTS_INVALID_HANDLE), iRefCount(0)
        {
        }
        virtual ~CElfinCollaborativeLib()
        {
        }
        virtual unsigned long AddRef(IBase *pIBase = NULL)
        {
            iRefCount++;
            return iRefCount;
        }
        virtual unsigned long Release(IBase *pIBase = NULL)
        {
            iRefCount--;
            if (iRefCount == 0)
            {
                delete this;
                return 0;
            }
            return iRefCount;
        }

        
        virtual void* QueryInterface(IBase *pIBase, ITFID iid, RTS_RESULT *pResult)
        {
            void *pItf;
            if (iid == ITFID_IBase)
                pItf = dynamic_cast<IBase *>((IElfinCollaborativeLib *)this);            
            else if (iid == ITFID_IElfinCollaborativeLib)
                pItf = dynamic_cast<IElfinCollaborativeLib *>(this); 
            else
            {
                if (pResult != NULL)
                    *pResult = ERR_NOTIMPLEMENTED;
                return NULL;
            }
            if (pResult != (RTS_RESULT *)1)
                (reinterpret_cast<IBase *>(pItf))->AddRef();
            if (pResult != NULL && pResult != (RTS_RESULT *)1)
                *pResult = ERR_OK;
            return pItf;
        }

    public:
        RTS_HANDLE hElfinCollaborativeLib;
        int iRefCount;
};

#endif /*CPLUSPLUS*/
#ifdef RTS_COMPACT_MICRO
/* ElfinCollaborativeLib Declarations for uRTS */
/* This header is included only to have the CMP_EXT_FUNCTION_REF type definition. */
/* In final version this type could be moved to CmpItf.h, if we will use */
/* the same structure for representing an export entry. */
#include <_OptionalIncludes/Profiles/uRTS/CMMicro.h>

/* Undefine these macros if they were previously defined */

#ifdef USE_HOOK_FUNCTION
#undef USE_HOOK_FUNCTION
#endif

#ifdef USE_CMEXPORTFUNCTIONS
#undef USE_CMEXPORTFUNCTIONS
#endif

#ifdef USE_EXPORT_EXTREF
#undef USE_EXPORT_EXTREF
#endif

#ifdef USE_EXPORT_CMPITF
#undef USE_EXPORT_CMPITF
#endif

#ifdef USE_ENTRY_LINKAGE
#undef USE_ENTRY_LINKAGE
#endif

#ifdef IMPLEMENT_EXPORT_FUNCTIONS
#undef IMPLEMENT_EXPORT_FUNCTIONS
#endif

#ifdef IMPLEMENT_IMPORT_FUNCTIONS
#undef IMPLEMENT_IMPORT_FUNCTIONS
#endif

#ifdef IMPLEMENT_GETVERSION_FUNCTIONS
#undef IMPLEMENT_GETVERSION_FUNCTIONS
#endif

#ifdef EXPORT_STMT
#undef EXPORT_STMT
#endif

#ifdef USEEXTERN_STMT
#undef USEEXTERN_STMT
#endif

#ifdef USE_STMT
#undef USE_STMT
#endif

#ifdef USEIMPORT_STMT
#undef USEIMPORT_STMT
#endif

/* This wrapper is only required in uRTS */
#define USE_HOOK_FUNCTION	RTS_RESULT ElfinCollaborativeLib_HookFunction(RTS_UI32 ulHook, RTS_UINTPTR ulParam1, RTS_UINTPTR ulParam2)\
{\
	return HookFunction(ulHook, ulParam1, ulParam2);\
}

/******************************************************************************/
/* No s_pfCMExportFunctions in uRTS */
#define USE_CMEXPORTFUNCTIONS

/* Exports table to IEC */
#define USE_EXPORT_EXTREF const CMP_EXT_FUNCTION_REF ElfinCollaborativeLib_ExternalsTable[] = \
	{\
		EXPORT_EXTREF_STMT\
		EXPORT_EXTREF2_STMT\
		{ ((RTS_VOID_FCTPTR)(void *)0), "", 0 }\
	};

/* No component interface exports in uRTS */
#define USE_EXPORT_CMPITF

/* No dynamic linkage in uRTS */
#define USE_ENTRY_LINKAGE

/* No ExportFunctions() routine in uRTS */
#define IMPLEMENT_EXPORT_FUNCTIONS

/* No ImportFunctions() routine in uRTS */
#define IMPLEMENT_IMPORT_FUNCTIONS

/* No CmpGetVersion() routine in uRTS */
#define IMPLEMENT_GETVERSION_FUNCTIONS

/* No usage of externals from the component secondary C-modules in uRTS */
#define USEEXTERN_STMT

/* No need for export calling to the Component Manager in uRTS */
#define EXPORT_STMT

/* No need for importing from secondary C-modules */
#define USEIMPORT_STMT

/* Instantiation macro for a root component C-module */
#define USE_STMT\
	static RTS_RESULT CDECL HookFunction(RTS_UI32 ulHook, RTS_UINTPTR ulParam1, RTS_UINTPTR ulParam2);\
		USE_EXPORT_EXTREF;\
		USE_EXPORT_CMPITF;\
		USE_HOOK_FUNCTION;\
		USE_CMEXPORTFUNCTIONS;\


/******************************************************************************/
#endif

#endif /*WIN32_RESOURCES*/
#endif /*_DEP_H_*/
