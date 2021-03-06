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
#include "ActuatorsDefinitions.h"
#ifdef SIMBODY
#include "simbody_cpp_functions.h"
#include "simbody_functions.h"
#endif

// ---- Constants & Macros ---- //

// Control Type
#define IDLE_CTRL 0   //maybe using an enum would be cleaner
#define TORQUE_CTRL 1
#define POSITION_DIRECT_CTRL 2
#define POSITION_CTRL 3
#define VELOCITY_CTRL 4
#define IMPEDANCE_POS_CTRL 5
#define IMPEDANCE_VEL_CTRL 6


typedef enum{
RHipSag_m =1,
RHipLat_m = 2,
RHipYaw_m = 3,
RKneeSag_m = 4,
RAnkLat_m = 5,
RAnkSag_m = 6,

LHipSag_m = 7,
LHipLat_m = 8,
LHipYaw_m = 9,
LKneeSag_m = 10,
LAnkLat_m = 11,
LAnkSag_m = 12,

Waist_m = 13,
DWL_m  = 14,
DWS_m 	= 15,

RShSag_m = 16,
RShLat_m = 17,
RShYaw_m = 18,
RElbj_m  = 19,
RForearmPlate_m = 20,
RWrj1_m  = 21,
RWrj2_m  = 22,
LShSag_m = 23,
LShLat_m = 24,
LShYaw_m = 25,
LElbj_m  = 26,
LForearmPlate_m = 27,
LWrj1_m  = 28,
LWrj2_m  = 29
} motorIds;

// ---- Actuated Joints Indices ---- //
typedef enum{
    // Right Leg
    RHipSag_id =  7,
      RHipLat_id =  8,
      RHipYaw_id =  9,
      RKneeSag_id =  10,
      RAnkLat_id =  11,
      RAnkSag_id =  12,

    // Left Leg
      LHipSag_id =  13,
      LHipLat_id =  14,
      LHipYaw_id =  15,
      LKneeSag_id =  16,
      LAnkLat_id =  17,
      LAnkSag_id =  18,

    // Torso
      Waist_id =  19,
      DWL_id = 20,
      DWS_id =    21,

    // Right Arm
      RShSag_id =  22,
      RShLat_id =  23,
      RShYaw_id =  24,
      RElbj_id =   25,
      RForearmPlate_id =  26,
      RWrj1_id =   27,
      RWrj2_id =   28,

    // Left Arm
      LShSag_id =  29,
      LShLat_id =  30,
      LShYaw_id =  31,
      LElbj_id =   32,
      LForearmPlate_id =  33,
      LWrj1_id =   34,
      LWrj2_id =   35
} jointIds;


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
