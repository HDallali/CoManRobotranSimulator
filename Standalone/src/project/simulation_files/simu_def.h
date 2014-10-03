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

// Right Arm
#define RShSag_m 16
#define RShLat_m 17
#define RShYaw_m 18
#define RElbj_m  19
#define RForearmPlate_m 20
#define RWrj1_m  21
#define RWrj2_m  22

// Left Arm
#define LShSag_m 23
#define LShLat_m 24
#define LShYaw_m 25
#define LElbj_m  26
#define LForearmPlate_m 27
#define LWrj1_m  28
#define LWrj2_m  29

// Right Leg
#define RHipSag_m 1
#define RHipLat_m 2
#define RHipYaw_m 3
#define RKneeSag_m 4
#define RAnkLat_m 5
#define RAnkSag_m 6

// Left Leg
#define LHipSag_m 7
#define LHipLat_m 8
#define LHipYaw_m 9
#define LKneeSag_m 10
#define LAnkLat_m 11
#define LAnkSag_m 12

// ---- Actuated Joints Indices ---- //

// Torso
#define Waist_id 19
#define DWL_id	 20
#define DWS_id   21

// Right Arm
#define RShSag_id 22
#define RShLat_id 23
#define RShYaw_id 24
#define RElbj_id  25
#define RForearmPlate_id 26
#define RWrj1_id  27
#define RWrj2_id  28

// Left Arm
#define LShSag_id 29
#define LShLat_id 30
#define LShYaw_id 31
#define LElbj_id  32
#define LForearmPlate_id 33
#define LWrj1_id  34
#define LWrj2_id  35

// Right Leg
#define RHipSag_id 7
#define RHipLat_id 8
#define RHipYaw_id 9
#define RKneeSag_id 10
#define RAnkLat_id 11
#define RAnkSag_id 12

// Left Leg
#define LHipSag_id 13
#define LHipLat_id 14
#define LHipYaw_id 15
#define LKneeSag_id 16
#define LAnkLat_id 17
#define LAnkSag_id 18



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
