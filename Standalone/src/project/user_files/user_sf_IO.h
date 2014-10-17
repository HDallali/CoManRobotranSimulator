/*===========================================================================*
 *
 *  user_sf_IO.h
 * 
 *  Generation date: Thu Oct  2 15:02:23 2014

 * 
 *  (c) Universite catholique de Louvain
 *      Departement de Mecanique 
 *      Unite de Production Mecanique et Machines 
 *      2, Place du Levant 
 *      1348 Louvain-la-Neuve 
 *  http://www.robotran.be// 
 *  
/*===========================================================================*/

#ifndef UsersfIO_h
#define UsersfIO_h
/*--------------------*/
 
#include "userDef.h"
#include "ControllersStruct.h"
#include "ActuatorsDefinitions.h"

typedef struct UserIOStruct 
{
    double GRF_r[3+1];
    double GRF_l[3+1];
    double GRM_r[3+1];
    double GRM_l[3+1];
    double GRF_r_dist[3+1];
    double GRF_l_dist[3+1];
    double GRM_r_dist[3+1];
    double GRM_l_dist[3+1];
    double mu_grf;
    double F_left_leg;
    double F_right_leg;
    int Msize_GCM;
    int Msize_GCM_prox;
    int Msize_GCM_dist;
    double rn_left_x[200+1];
    double rn_left_y[200+1];
    double rn_left_z[200+1];
    double rn_right_x[200+1];
    double rn_right_y[200+1];
    double rn_right_z[200+1];
    double temp_grfx_left[200+1];
    double temp_grfy_left[200+1];
    double temp_grfx_right[200+1];
    double temp_grfy_right[200+1];
    int flag_grfx_left[200+1];
    int flag_grfy_left[200+1];
    int flag_grfx_right[200+1];
    int flag_grfy_right[200+1];
    double rn_left_prox_x[150+1];
    double rn_left_prox_y[150+1];
    double rn_left_prox_z[150+1];
    double rn_right_prox_x[150+1];
    double rn_right_prox_y[150+1];
    double rn_right_prox_z[150+1];
    double temp_grfx_left_prox[150+1];
    double temp_grfy_left_prox[150+1];
    double temp_grfx_right_prox[150+1];
    double temp_grfy_right_prox[150+1];
    int flag_grfx_left_prox[150+1];
    int flag_grfy_left_prox[150+1];
    int flag_grfx_right_prox[150+1];
    int flag_grfy_right_prox[150+1];
    double rn_left_dist_x[60+1];
    double rn_left_dist_y[60+1];
    double rn_left_dist_z[60+1];
    double rn_right_dist_x[60+1];
    double rn_right_dist_y[60+1];
    double rn_right_dist_z[60+1];
    double temp_grfx_left_dist[60+1];
    double temp_grfy_left_dist[60+1];
    double temp_grfx_right_dist[60+1];
    double temp_grfy_right_dist[60+1];
    int flag_grfx_left_dist[60+1];
    int flag_grfy_left_dist[60+1];
    int flag_grfx_right_dist[60+1];
    int flag_grfy_right_dist[60+1];
    double tsim_out1;
    double output1[29+1];
    double output2[29+1];
    double Voltage[29+1];
    double refs[29+1];
    double *currents;
    int servo_type[29+1];
    ControllerStruct *cvs;
    SimbodyStruct *simbodyStruct;
    ActuatorsStruct *actuatorsStruct;

} UserIOStruct;

/*--------------------*/
#endif
