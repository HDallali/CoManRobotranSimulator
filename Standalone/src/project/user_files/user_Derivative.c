//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "simu_def.h"

void user_Derivative(MBSdataStruct *MBSdata)
{

    UserIOStruct *uvs;

    int i;

    double rho ;
    double K_W ;
    double L_M ;
    double R_M ;
    double KT  ;

    double Ks ;
    double Ds ;
    double J_M;
    // voltage to torque gain (used in 2nd order dynamics)
    double VT ; //
    double D_M;

    const int n = NB_ACTUATED_JOINTS;

    // PD control !!! the gains should change depending on the actuator order and servo_type
    double Kp=20;
    double Kd=1;

    double voltage[NB_ACTUATED_JOINTS]={0.0};
    double Cpl[NB_ACTUATED_JOINTS]={0.0};

    double *ref = MBSdata->user_IO->refs;

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

    uvs = MBSdata->user_IO;

    if (Act_type==1) //SEA
    {
        // PD control law
        // NEED Different values for various Joints (TO BE done by the config.ini files)

        switch (Act_order) {
          case 1:
            Kp=5;
            Kd=0.01;
          break;
        case 2:
            Kp=10;
            Kd=0.1;
          break;
        case 3:
            Kp=150;
            Kd=1;
          break;
        }

        for (i=0; i<n; i++)
        {
            voltage[MotorIds[i]]    = Kp*(ref[MotorIds[i]]-MBSdata->q[JointIds[i]])-Kd*MBSdata->qd[JointIds[i]];
        }

        switch (Act_order) {
          case 1:
            justElectrical:
            // Motor (electrical) ODE
            // need a map from index i=0:4 to real joint indices
            // ux:current, uxd: current derivatives:

            // All robot joints
            for (i=0; i<n; i++)
            {
                rho = uvs->actuatorsStruct->acs[MotorIds[i]-1]->GearRatio;
                R_M = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Resistance;
                K_W = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Kbemf;
                L_M = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Inductance;
                MBSdata->uxd[MotorIds[i]]   = (1.0/L_M)*(voltage[MotorIds[i]] -R_M*MBSdata->ux[MotorIds[i]]-K_W*rho* MBSdata->qd[JointIds[i]]);
            }


           break;
           case 2:
            // Motor (Mechanical) ODE
            // ux:motor position, velocity, uxd: motor velocity, acceleration
            for (i=0; i<n; i++)
            {
                //update motor velocities:
                MBSdata->uxd[MotorIds[i]]=MBSdata->ux[MotorIds[i]+n];

                rho = uvs->actuatorsStruct->acs[MotorIds[i]-1]->GearRatio;
                R_M = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Resistance;
                KT = uvs->actuatorsStruct->acs[MotorIds[i]-1]->TrqConst;
                J_M = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Inertia;
                VT  = rho*(KT)/R_M;
                D_M = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Damping;
                Ks  = uvs->actuatorsStruct->acs[MotorIds[i]-1]->SeriesSpring;
                Ds  = uvs->actuatorsStruct->acs[MotorIds[i]-1]->SeriesDamping;

                // computing the transmission torque (coupling between motor and load)
                Cpl[MotorIds[i]] = Ks*(MBSdata->ux[MotorIds[i]]-MBSdata->q[JointIds[i]]);
                Cpl[MotorIds[i]]+= Ds*(MBSdata->uxd[MotorIds[i]]-MBSdata->qd[JointIds[i]]);

                //update motor accelerations:
                MBSdata->uxd[MotorIds[i]]= (1.0/J_M)*(VT*voltage[MotorIds[i]] -D_M*MBSdata->ux[n+MotorIds[i]]-Cpl[MotorIds[i]]);
            }

           break;
            case 3:
            // Motor (Electrical+Mechanical) ODE
            // ux:motor position, velocity, current uxd: motor velocity, acceleration, current derivative
            //update motor velocities:
            for (i=0; i<n; i++)
            {
                MBSdata->uxd[MotorIds[i]]=MBSdata->ux[MotorIds[i]+n];

                rho = uvs->actuatorsStruct->acs[MotorIds[i]-1]->GearRatio;
                R_M = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Resistance;
                K_W = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Kbemf;
                L_M = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Inductance;
                KT = K_W;

                J_M = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Inertia;
                VT  = rho*(KT)/R_M;
                D_M = uvs->actuatorsStruct->acs[MotorIds[i]-1]->Damping;
                Ks  = uvs->actuatorsStruct->acs[MotorIds[i]-1]->SeriesSpring;
                Ds  = uvs->actuatorsStruct->acs[MotorIds[i]-1]->SeriesDamping;

                // computing the transmission torque (coupling between motor and load)
                Cpl[MotorIds[i]]  = Ks*(MBSdata->ux[MotorIds[i]]-MBSdata->q[JointIds[i]]);
                Cpl[MotorIds[i]] += Ds*(MBSdata->uxd[MotorIds[i]]-MBSdata->qd[JointIds[i]]);
                // update motor acceleration:
                MBSdata->uxd[MotorIds[i]+n]= (1.0/J_M)*(KT*MBSdata->ux[2*n+MotorIds[i]] -D_M*MBSdata->ux[n+MotorIds[i]]-Cpl[MotorIds[i]]);

                // update current derivative:
                MBSdata->uxd[MotorIds[i]+2*n]=(1.0/L_M)*(voltage[MotorIds[i]] -R_M*MBSdata->ux[2*n+MotorIds[i]]-K_W*rho* MBSdata->qd[JointIds[i]]);
            }
           break;
            default:
            printf("detault actuator order (1) selected \n");
            goto justElectrical;
           break;
            }
    }

}

