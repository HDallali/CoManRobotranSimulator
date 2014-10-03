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


// ---- Motor Indices ---- //

// Torso
#define  Waist_m 13
#define  DWL_m   14
#define  DWS_m 	 15

// ---- Actuated Joints Indices ---- //

// Torso
#define Waist_id 19
#define DWL_id	 20
#define DWS_id 21




//GCM
#define RFOOT_FSENS_ID 1
#define LFOOT_FSENS_ID 6

// limiting external forces
#define MAX_EXT_FORCES	5000.0
#define MAX_EXT_MOMENTS	5000.0

// GCM (Ground Contact Model)
#define K_GCM 10000.0    // stiffness coeff
#define D_GCM 10.0       // damping coeff
#define MU_GCM 0.9       // friction coeff (mu) [-]
#define K_GZ_GCM 27818.0 // k_gz           (HG: 27818)
#define V_GZ_MAX_GCM 0.3 // v_gz_max [m/s] (HG: 0.03)

// ---- Custom Functions ---- //

void controller_init(ControllerStruct *cvs);
void controller_loop(ControllerStruct *cvs);

double limit_angle(double angle);

void controller_inputs(MBSdataStruct *MBSdata);
void controller_outputs(MBSdataStruct *MBSdata);

void simu_outputs(MBSdataStruct *MBSdata);

void simu_controller_loop(MBSdataStruct *MBSdata);

// GCM (Ground Contact Model)
double get_ground_height(double x, double y, double tsim, MBSdataStruct *MBSdata);
void ground_mesh_model(double PxF[4], double RxF[4][4],
					   double VxF[4], double OMxF[4],
					   MBSdataStruct *MBSdata, double tsim,
					   int ixF, double *dxF, double *SWr);
void init_GCM(MBSdataStruct *MBSdata);
double z_left_foot(double x, double y);
double z_right_foot(double x, double y);

/*--------------------*/
#endif 
