/*===========================================================================*
 *
 *  user_sf_IO.c
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

#include "MBSfun.h" 
#include "user_sf_IO.h" 
#include "sfdef.h" 
#include "userDef.h"
#include "ControllersStruct.h"


UserIOStruct * initUserIO(MBSdataStruct *s)
{
    UserIOStruct *uvs;
    int i;
    //
    uvs = (UserIOStruct*) malloc(sizeof(UserIOStruct));


    // GRF_r //
    for (i=1;i<=3;i++)
    {
        uvs->GRF_r[i] = 0.0;
    }

    // GRF_l //
    for (i=1;i<=3;i++)
    {
        uvs->GRF_l[i] = 0.0;
    }

    // GRM_r //
    for (i=1;i<=3;i++)
    {
        uvs->GRM_r[i] = 0.0;
    }

    // GRM_l //
    for (i=1;i<=3;i++)
    {
        uvs->GRM_l[i] = 0.0;
    }

    // GRF_r_dist //
    for (i=1;i<=3;i++)
    {
        uvs->GRF_r_dist[i] = 0.0;
    }

    // GRF_l_dist //
    for (i=1;i<=3;i++)
    {
        uvs->GRF_l_dist[i] = 0.0;
    }

    // GRM_r_dist //
    for (i=1;i<=3;i++)
    {
        uvs->GRM_r_dist[i] = 0.0;
    }

    // GRM_l_dist //
    for (i=1;i<=3;i++)
    {
        uvs->GRM_l_dist[i] = 0.0;
    }

    // mu_grf //
    uvs->mu_grf = 0.0;

    // F_left_leg //
    uvs->F_left_leg = 0.0;

    // F_right_leg //
    uvs->F_right_leg = 0.0;

    // Msize_GCM //
    uvs->Msize_GCM = 0;

    // Msize_GCM_prox //
    uvs->Msize_GCM_prox = 0;

    // Msize_GCM_dist //
    uvs->Msize_GCM_dist = 0;

    // rn_left_x //
    for (i=1;i<=200;i++)
    {
        uvs->rn_left_x[i] = 0.0;
    }

    // rn_left_y //
    for (i=1;i<=200;i++)
    {
        uvs->rn_left_y[i] = 0.0;
    }

    // rn_left_z //
    for (i=1;i<=200;i++)
    {
        uvs->rn_left_z[i] = 0.0;
    }

    // rn_right_x //
    for (i=1;i<=200;i++)
    {
        uvs->rn_right_x[i] = 0.0;
    }

    // rn_right_y //
    for (i=1;i<=200;i++)
    {
        uvs->rn_right_y[i] = 0.0;
    }

    // rn_right_z //
    for (i=1;i<=200;i++)
    {
        uvs->rn_right_z[i] = 0.0;
    }

    // temp_grfx_left //
    for (i=1;i<=200;i++)
    {
        uvs->temp_grfx_left[i] = 0.0;
    }

    // temp_grfy_left //
    for (i=1;i<=200;i++)
    {
        uvs->temp_grfy_left[i] = 0.0;
    }

    // temp_grfx_right //
    for (i=1;i<=200;i++)
    {
        uvs->temp_grfx_right[i] = 0.0;
    }

    // temp_grfy_right //
    for (i=1;i<=200;i++)
    {
        uvs->temp_grfy_right[i] = 0.0;
    }

    // flag_grfx_left //
    for (i=1;i<=200;i++)
    {
        uvs->flag_grfx_left[i] = 0;
    }

    // flag_grfy_left //
    for (i=1;i<=200;i++)
    {
        uvs->flag_grfy_left[i] = 0;
    }

    // flag_grfx_right //
    for (i=1;i<=200;i++)
    {
        uvs->flag_grfx_right[i] = 0;
    }

    // flag_grfy_right //
    for (i=1;i<=200;i++)
    {
        uvs->flag_grfy_right[i] = 0;
    }

    // rn_left_prox_x //
    for (i=1;i<=150;i++)
    {
        uvs->rn_left_prox_x[i] = 0.0;
    }

    // rn_left_prox_y //
    for (i=1;i<=150;i++)
    {
        uvs->rn_left_prox_y[i] = 0.0;
    }

    // rn_left_prox_z //
    for (i=1;i<=150;i++)
    {
        uvs->rn_left_prox_z[i] = 0.0;
    }

    // rn_right_prox_x //
    for (i=1;i<=150;i++)
    {
        uvs->rn_right_prox_x[i] = 0.0;
    }

    // rn_right_prox_y //
    for (i=1;i<=150;i++)
    {
        uvs->rn_right_prox_y[i] = 0.0;
    }

    // rn_right_prox_z //
    for (i=1;i<=150;i++)
    {
        uvs->rn_right_prox_z[i] = 0.0;
    }

    // temp_grfx_left_prox //
    for (i=1;i<=150;i++)
    {
        uvs->temp_grfx_left_prox[i] = 0.0;
    }

    // temp_grfy_left_prox //
    for (i=1;i<=150;i++)
    {
        uvs->temp_grfy_left_prox[i] = 0.0;
    }

    // temp_grfx_right_prox //
    for (i=1;i<=150;i++)
    {
        uvs->temp_grfx_right_prox[i] = 0.0;
    }

    // temp_grfy_right_prox //
    for (i=1;i<=150;i++)
    {
        uvs->temp_grfy_right_prox[i] = 0.0;
    }

    // flag_grfx_left_prox //
    for (i=1;i<=150;i++)
    {
        uvs->flag_grfx_left_prox[i] = 0;
    }

    // flag_grfy_left_prox //
    for (i=1;i<=150;i++)
    {
        uvs->flag_grfy_left_prox[i] = 0;
    }

    // flag_grfx_right_prox //
    for (i=1;i<=150;i++)
    {
        uvs->flag_grfx_right_prox[i] = 0;
    }

    // flag_grfy_right_prox //
    for (i=1;i<=150;i++)
    {
        uvs->flag_grfy_right_prox[i] = 0;
    }

    // rn_left_dist_x //
    for (i=1;i<=60;i++)
    {
        uvs->rn_left_dist_x[i] = 0.0;
    }

    // rn_left_dist_y //
    for (i=1;i<=60;i++)
    {
        uvs->rn_left_dist_y[i] = 0.0;
    }

    // rn_left_dist_z //
    for (i=1;i<=60;i++)
    {
        uvs->rn_left_dist_z[i] = 0.0;
    }

    // rn_right_dist_x //
    for (i=1;i<=60;i++)
    {
        uvs->rn_right_dist_x[i] = 0.0;
    }

    // rn_right_dist_y //
    for (i=1;i<=60;i++)
    {
        uvs->rn_right_dist_y[i] = 0.0;
    }

    // rn_right_dist_z //
    for (i=1;i<=60;i++)
    {
        uvs->rn_right_dist_z[i] = 0.0;
    }

    // temp_grfx_left_dist //
    for (i=1;i<=60;i++)
    {
        uvs->temp_grfx_left_dist[i] = 0.0;
    }

    // temp_grfy_left_dist //
    for (i=1;i<=60;i++)
    {
        uvs->temp_grfy_left_dist[i] = 0.0;
    }

    // temp_grfx_right_dist //
    for (i=1;i<=60;i++)
    {
        uvs->temp_grfx_right_dist[i] = 0.0;
    }

    // temp_grfy_right_dist //
    for (i=1;i<=60;i++)
    {
        uvs->temp_grfy_right_dist[i] = 0.0;
    }

    // flag_grfx_left_dist //
    for (i=1;i<=60;i++)
    {
        uvs->flag_grfx_left_dist[i] = 0;
    }

    // flag_grfy_left_dist //
    for (i=1;i<=60;i++)
    {
        uvs->flag_grfy_left_dist[i] = 0;
    }

    // flag_grfx_right_dist //
    for (i=1;i<=60;i++)
    {
        uvs->flag_grfx_right_dist[i] = 0;
    }

    // flag_grfy_right_dist //
    for (i=1;i<=60;i++)
    {
        uvs->flag_grfy_right_dist[i] = 0;
    }

    // tsim_out1 //
    uvs->tsim_out1 = 0.0;

    // output1 //
    for (i=1;i<=29;i++)
    {
        uvs->output1[i] = 0.0;
    }

    // output2 //
    for (i=1;i<=29;i++)
    {
        uvs->output2[i] = 0.0;
    }

    // Voltage //
    for (i=1;i<=29;i++)
    {
        uvs->Voltage[i] = 0.0;
    }

    // refs //
    for (i=1;i<=29;i++)
    {
        uvs->refs[i] = 0.0;
    }

    // servo_type //
    for (i=1;i<=29;i++)
    {
        uvs->servo_type[i] = 0;
    }

    // cvs //
    uvs->cvs = init_ControllerStruct();

    // simbodyStruct //
    uvs->simbodyStruct = init_SimbodyStruct();

    // actuatorsStruct //
    uvs->actuatorsStruct = init_ActuatorsStruct();

    return uvs;
}


void freeUserIO(UserIOStruct *uvs, MBSdataStruct *s)
{

    // ControllerStruct: cvs //
    free_ControllerStruct(uvs->cvs);

    // SimbodyStruct: simbodyStruct //
    free_SimbodyStruct(uvs->simbodyStruct);

    // ActuatorsStruct: actuatorsStruct //
    free_ActuatorsStruct(uvs->actuatorsStruct);

    free(uvs);
}

