//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2008
// Last update : 24/10/2008
//---------------------------

#include "simu_def.h"

int i;
// User parameters initialization
#ifndef CMEX
void user_initialization(SimStruct *S, MBSdataStruct *MBSdata, LocalDataStruct *lds)
#else
// Returns 0 if no problem
int user_initialization(MBSdataStruct *MBSdata, LocalDataStruct *lds)
#endif
{

    for (i=0; i<=29; i++)
         MBSdata->ux[i]= MBSdata->q[i+6];

	// inputs of the controller
	controller_inputs(MBSdata);

    // init GCM (Ground Contact Model)
    init_GCM(MBSdata);

	//  controller initialization
    controller_init(MBSdata->user_IO->cvs);
    
    #ifdef CMEX
    return 0;
    #endif
}
