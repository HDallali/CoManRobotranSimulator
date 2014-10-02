//---------------------------
// Creation : ../../2013
// Last update : ../../2013
//---------------------------

#ifndef simulation_def_h
#define simulation_def_h
//--------------------*/

#include <math.h>
#include <stdlib.h>

#include "user_sf_IO.h"
#include "MBSdef.h"
#include "MBSfun.h"
#include "MBSdataStruct.h"
#include "ControllersStruct.h"
#include "nrutil.h"

#ifdef SIMBODY
#include "simbody_cpp_functions.h"
#include "simbody_functions.h"
#endif


// ---- Constants & Macros ---- //

// Actuator Options:
#define Act_order 1 // 1st (electric), 2nd (mechanical) or 3rd (electrical, mechanical)
#define Act_type 1  // 0-> SEA-Small, 1 ->SEA-Med, 2->SEA-Big, 3-> PEA-default

// Control Type
#define IDLE_CTRL 0   //maybe using an enum would be cleaner
#define TORQUE_CTRL 1
#define POSITION_DIRECT_CTRL 2
#define POSITION_CTRL 3
#define VELOCITY_CTRL 4
#define IMPEDANCE_POS_CTRL 5
#define IMPEDANCE_VEL_CTRL 6



// motor indices
#define  Waist_m 1
#define  DWL_m   2
#define  DWS_m 	 3
#define  M_RL 4

#define  M_FR Waist_m
#define  M_FL DWL_m
#define  M_RR DWS_m

//
#define Waist_id 19
#define DWL_id	 20
#define DWS_id 21

// actuated joints (4 legs)
#define R2_FR Waist_id
#define R2_FL DWL_id
#define R2_RR DWS_id
#define R2_RL 22

//spring joints
#define Spring_FR 8
#define Spring_FL 10
#define Spring_RR 12
#define Spring_RL 14

// ---- Custom Functions ---- //

void controller_init(ControllerStruct *cvs);
void controller_loop(ControllerStruct *cvs);

double limit_angle(double angle);

void controller_inputs(MBSdataStruct *MBSdata);
void controller_outputs(MBSdataStruct *MBSdata);

void simu_outputs(MBSdataStruct *MBSdata);

void simu_controller_loop(MBSdataStruct *MBSdata);

/*--------------------*/
#endif 
