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

    // default no actuator model
    for (i=1; i<=29; i++)
        MBSdata->Qq[i+6]=-200*(MBSdata->q[i+6]-0.0)-10*MBSdata->qd[i+6];

    MBSdata->Qq[30]=-200*(MBSdata->q[30]+1.4) -10*MBSdata->qd[30];
    MBSdata->Qq[23]=-200*(MBSdata->q[23]-1.4) -10*MBSdata->qd[23];


    // Actuated Joints:
    if (Act_type==1) //SEA
    {

        switch (Act_order) {
          case 1:
            justElectrical:
            MBSdata->Qq[Waist_id] = rho* KT * MBSdata->ux[Waist_m];
            MBSdata->Qq[DWL_id] = rho* KT * MBSdata->ux[DWL_m];
            MBSdata->Qq[DWS_id] = rho* KT * MBSdata->ux[DWS_m];
          break;

          case 2:
            justMechanical:

            MBSdata->Qq[Waist_id]=Ks*(MBSdata->ux[Waist_m]-MBSdata->q[Waist_id])+Ds*(MBSdata->uxd[Waist_m]-MBSdata->qd[Waist_id]);
            MBSdata->Qq[DWL_id]=Ks*(MBSdata->ux[DWL_m]-MBSdata->q[DWL_id])+Ds*(MBSdata->uxd[DWL_m]-MBSdata->qd[DWL_id]);
            MBSdata->Qq[DWS_id]=Ks*(MBSdata->ux[DWS_m]-MBSdata->q[DWS_id])+Ds*(MBSdata->uxd[DWS_m]-MBSdata->qd[DWS_id]);

            
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

