//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "simu_def.h"

double* user_JointForces(MBSdataStruct *MBSdata, double tsim)
{
    
    int i;
    double rho = MBSdata->user_IO->actuatorsStruct->acs[0]->GearRatio;
    double KT = MBSdata->user_IO->actuatorsStruct->acs[0]->TrqConst;
    // Transmission stiffness, damping
    double Ks = MBSdata->user_IO->actuatorsStruct->acs[0]->SeriesSpring;
    double Ds = MBSdata->user_IO->actuatorsStruct->acs[0]->SeriesDamping;

    double *ref = MBSdata->user_IO->refs;
    const int n = NB_ACTUATED_JOINTS;

    // order is important:
    int MotorIds[NB_ACTUATED_JOINTS]={ RHipSag_m, RHipLat_m, RHipYaw_m, RKneeSag_m, RAnkLat_m, RAnkSag_m,
                                       LHipSag_m,  LHipLat_m,  LHipYaw_m,  LKneeSag_m, LAnkLat_m, LAnkSag_m,
                                       Waist_m ,        DWL_m  ,      DWS_m ,
                                       RShSag_m, RShLat_m, RShYaw_m , RElbj_m, RForearmPlate_m, RWrj1_m,  RWrj2_m,
                                       LShSag_m, LShLat_m, LShYaw_m,  LElbj_m, LForearmPlate_m, LWrj1_m,  LWrj2_m };

    int JointIds[NB_ACTUATED_JOINTS]={ RHipSag_id, RHipLat_id, RHipYaw_id, RKneeSag_id, RAnkLat_id, RAnkSag_id,
                                       LHipSag_id,  LHipLat_id,  LHipYaw_id,  LKneeSag_id, LAnkLat_id, LAnkSag_id,
                                       Waist_id ,        DWL_id  ,      DWS_id ,
                                       RShSag_id, RShLat_id, RShYaw_id , RElbj_id, RForearmPlate_id, RWrj1_id,  RWrj2_id,
                                       LShSag_id, LShLat_id, LShYaw_id,  LElbj_id, LForearmPlate_id, LWrj1_id,  LWrj2_id };



    // Actuated Joints:
    if (Act_type==1) //SEA
    {
        switch (Act_order) {
          case 1:
            justElectrical:
            //iterating over motor and joint ids
            for (i=0; i<n; i++)
            {   rho = MBSdata->user_IO->actuatorsStruct->acs[MotorIds[i]-1]->GearRatio;
                KT = MBSdata->user_IO->actuatorsStruct->acs[MotorIds[i]-1]->TrqConst;
                MBSdata->Qq[JointIds[i]]   = rho* KT * MBSdata->ux[MotorIds[i]];
            }
          break;
          case 2:
            justMechanical:
            for (i=0; i<n; i++)
            {
                MBSdata->Qq[JointIds[i]]=Ks*(MBSdata->ux[MotorIds[i]]-MBSdata->q[JointIds[i]]);
                MBSdata->Qq[JointIds[i]]+=Ds*(MBSdata->uxd[MotorIds[i]]-MBSdata->qd[JointIds[i]]);
            }

          break;
          case 3:
            goto justMechanical;
          break;

          default:
            printf("detault actuator order (1) selected \n");
            goto justElectrical;
          break;
        }
    }

   	return MBSdata->Qq;

}

