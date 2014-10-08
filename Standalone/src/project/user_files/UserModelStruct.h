//---------------------------
// C-code automatically generated from Gen_mds_user project
//
//
// Last update : Tue Sep 30 17:59:16 2014
//---------------------------



#ifndef USERMODELSTRUCT_h
#define USERMODELSTRUCT_h

#include "lut.h"
#include "useful_functions.h"
#include "mbs_xml_reader.h"

// ============================================================ //


typedef struct UserModelStruct 
{
    struct Motor{
        int* motorStates;
        // index of the corresponding values in MBSdataStruct->ux/uxd/ux0
    } Motor;
 
} UserModelStruct;

UserModelStruct* init_UserModelStruct();
void free_UserModelStruct(UserModelStruct* ums);
void load_UserModelStruct(MDS_gen_strct* gen, UserModelStruct* ums);
// ============================================================ //
 
# endif
