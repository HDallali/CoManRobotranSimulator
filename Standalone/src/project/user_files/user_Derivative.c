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


    uvs = MBSdata->user_IO;

    if (Act_type==1) //SEA
    {
        // PD control law

        // Torso

        Kp=20;
        Kd=1;

        voltage[Waist_m]    = Kp*(ref[Waist_m]-MBSdata->q[Waist_id])-Kd*MBSdata->qd[Waist_id];
        voltage[DWL_m]      = Kp*(ref[DWL_m]-MBSdata->q[DWL_id])-Kd*MBSdata->qd[DWL_id];
        voltage[DWS_m]      = Kp*(ref[DWS_m]-MBSdata->q[DWS_id])-Kd*MBSdata->qd[DWS_id];

        // Right Arm

        Kp=5;
        Kd=0.0; //0.1;

        voltage[RShSag_m]           = Kp*(ref[RShSag_m]-MBSdata->q[RShSag_id])-Kd*MBSdata->qd[RShSag_id];
        voltage[RShLat_m]           = Kp*(ref[RShLat_m]-MBSdata->q[RShLat_id])-Kd*MBSdata->qd[RShLat_id];
        voltage[RShYaw_m]           = Kp*(ref[RShYaw_m]-MBSdata->q[RShYaw_id])-Kd*MBSdata->qd[RShYaw_id];
        voltage[RElbj_m]            = Kp*(ref[RElbj_m]-MBSdata->q[RElbj_id])-Kd*MBSdata->qd[RElbj_id];
        voltage[RForearmPlate_m]    = Kp*(ref[RForearmPlate_m]-MBSdata->q[RForearmPlate_id])-Kd*MBSdata->qd[RForearmPlate_id];
        voltage[RWrj1_m]            = Kp*(ref[RWrj1_m]-MBSdata->q[RWrj1_id])-Kd*MBSdata->qd[RWrj1_id];
        voltage[RWrj2_m]            = Kp*(ref[RWrj2_m]-MBSdata->q[RWrj2_id])-Kd*MBSdata->qd[RWrj2_id];

        // Left Arm

        Kp=5;
        Kd=0.0; //0.1;

        voltage[LShSag_m]           = Kp*(ref[LShSag_m]-MBSdata->q[LShSag_id])-Kd*MBSdata->qd[LShSag_id];
        voltage[LShLat_m]           = Kp*(ref[LShLat_m]-MBSdata->q[LShLat_id])-Kd*MBSdata->qd[LShLat_id];
        voltage[LShYaw_m]           = Kp*(ref[LShYaw_m]-MBSdata->q[LShYaw_id])-Kd*MBSdata->qd[LShYaw_id];
        voltage[LElbj_m]            = Kp*(ref[LElbj_m]-MBSdata->q[LElbj_id])-Kd*MBSdata->qd[LElbj_id];
        voltage[LForearmPlate_m]    = Kp*(ref[LForearmPlate_m]-MBSdata->q[LForearmPlate_id])-Kd*MBSdata->qd[LForearmPlate_id];
        voltage[LWrj1_m]            = Kp*(ref[LWrj1_m]-MBSdata->q[LWrj1_id])-Kd*MBSdata->qd[LWrj1_id];
        voltage[LWrj2_m]            = Kp*(ref[LWrj2_m]-MBSdata->q[LWrj2_id])-Kd*MBSdata->qd[LWrj2_id];




        switch (Act_order) {
          case 1:
            justElectrical:
            // Motor (electrical) ODE
            // need a map from index i=0:4 to real joint indices
            // ux:current, uxd: current derivatives:

            // Torso

            rho = 100;// MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->GearRatio;
            R_M = 1;// MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->Resistance;
            K_W = 0.04; //MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->Kbemf;
            L_M = 0.0001; //MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->Inductance;

            MBSdata->uxd[Waist_m]   = (1.0/L_M)*(voltage[Waist_m] -R_M*MBSdata->ux[Waist_m]-K_W*rho* MBSdata->qd[Waist_id]);
            MBSdata->uxd[DWL_m]     = (1.0/L_M)*(voltage[DWL_m] -R_M*MBSdata->ux[DWL_m]-K_W*rho* MBSdata->qd[DWL_id]);
            MBSdata->uxd[DWS_m]     = (1.0/L_M)*(voltage[DWS_m] - R_M*MBSdata->ux[DWS_m]-K_W*rho* MBSdata->qd[DWS_id]);

            // Right Arm

            rho = 100;// MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->GearRatio;
            R_M = 1;// MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->Resistance;
            K_W = 0.004; //MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->Kbemf;
            L_M = 0.0001; //MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->Inductance;

            MBSdata->uxd[RShSag_m]  = (1.0/L_M)*(voltage[RShSag_m] - R_M*MBSdata->ux[RShSag_m]-K_W*rho*MBSdata->qd[RShSag_id]);
            MBSdata->uxd[RShLat_m]  = (1.0/L_M)*(voltage[RShLat_m] - R_M*MBSdata->ux[RShLat_m]-K_W*rho*MBSdata->qd[RShLat_id]);
            MBSdata->uxd[RShYaw_m]  = (1.0/L_M)*(voltage[RShYaw_m] - R_M*MBSdata->ux[RShYaw_m]-K_W*rho*MBSdata->qd[RShYaw_id]);
            MBSdata->uxd[RElbj_m]   = (1.0/L_M)*(voltage[RElbj_m] - R_M*MBSdata->ux[RElbj_m]-K_W*rho*MBSdata->qd[RElbj_id]);
            MBSdata->uxd[RForearmPlate_m]   = (1.0/L_M)*(voltage[RForearmPlate_m] - R_M*MBSdata->ux[RForearmPlate_m]-K_W*rho*MBSdata->qd[RForearmPlate_id]);
            MBSdata->uxd[RWrj1_m]   = (1.0/L_M)*(voltage[RWrj1_m] - R_M*MBSdata->ux[RWrj1_m]-K_W*rho*MBSdata->qd[RWrj1_id]);
            MBSdata->uxd[RWrj2_m]   = (1.0/L_M)*(voltage[RWrj2_m] - R_M*MBSdata->ux[RWrj2_m]-K_W*rho*MBSdata->qd[RWrj2_id]);


            // Left Arm

            rho = 100;// MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->GearRatio;
            R_M = 1;// MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->Resistance;
            K_W = 0.004; //MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->Kbemf;
            L_M = 0.0001; //MBSdata->user_IO->actuatorsStruct->acs[Waist_m]->Inductance;

            MBSdata->uxd[LShSag_m]  = (1.0/L_M)*(voltage[LShSag_m] - R_M*MBSdata->ux[LShSag_m]-K_W*rho*MBSdata->qd[LShSag_id]);
            MBSdata->uxd[LShLat_m]  = (1.0/L_M)*(voltage[LShLat_m] - R_M*MBSdata->ux[LShLat_m]-K_W*rho*MBSdata->qd[LShLat_id]);
            MBSdata->uxd[LShYaw_m]  = (1.0/L_M)*(voltage[LShYaw_m] - R_M*MBSdata->ux[LShYaw_m]-K_W*rho*MBSdata->qd[LShYaw_id]);
            MBSdata->uxd[LElbj_m]   = (1.0/L_M)*(voltage[LElbj_m] - R_M*MBSdata->ux[LElbj_m]-K_W*rho*MBSdata->qd[RElbj_id]);
            MBSdata->uxd[LForearmPlate_m]   = (1.0/L_M)*(voltage[LForearmPlate_m] - R_M*MBSdata->ux[LForearmPlate_m]-K_W*rho*MBSdata->qd[LForearmPlate_id]);
            MBSdata->uxd[LWrj1_m]   = (1.0/L_M)*(voltage[LWrj1_m] - R_M*MBSdata->ux[LWrj1_m]-K_W*rho*MBSdata->qd[LWrj1_id]);
            MBSdata->uxd[LWrj2_m]   = (1.0/L_M)*(voltage[LWrj2_m] - R_M*MBSdata->ux[LWrj2_m]-K_W*rho*MBSdata->qd[LWrj2_id]);

           break;
//            case 2:
//            // Motor (Mechanical) ODE
//            // ux:motor position, velocity, uxd: motor velocity, acceleration
//            //update motor velocities:
//            for (i=0; i<n; i++)
//            {
//                MBSdata->uxd[i]=MBSdata->ux[i+n];
//            }

//            J_M = MBSdata->user_IO->actuatorsStruct->acs[M_RR]->Inertia;
//            VT  = rho*(KT)/R_M;
//            D_M = MBSdata->user_IO->actuatorsStruct->acs[M_RR]->Damping;
//            Ks  = MBSdata->user_IO->actuatorsStruct->acs[M_RR]->SeriesSpring;
//            Ds  = MBSdata->user_IO->actuatorsStruct->acs[M_RR]->SeriesDamping;

//            // computing the transmission torque (coupling between motor and load)
//            Cpl[M_FR]=Ks*(MBSdata->ux[M_FR]-MBSdata->q[R2_FR])+Ds*(MBSdata->uxd[M_FR]-MBSdata->qd[R2_FR]);
//            Cpl[M_FL]=Ks*(MBSdata->ux[M_FL]-MBSdata->q[R2_FL])+Ds*(MBSdata->uxd[M_FL]-MBSdata->qd[R2_FL]);
//            Cpl[M_RR]=Ks*(MBSdata->ux[M_RR]-MBSdata->q[R2_RR])+Ds*(MBSdata->uxd[M_RR]-MBSdata->qd[R2_RR]);
//            //Cpl[M_RL]=Ks*(MBSdata->ux[M_RL]-MBSdata->q[R2_RL])+Ds*(MBSdata->uxd[M_RL]-MBSdata->qd[R2_RL]);

//            //update motor accelerations:
//            MBSdata->uxd[M_FR]= (1.0/J_M)*(VT*voltage[M_FR] -D_M*MBSdata->ux[n+M_FR]-Cpl[M_FR]);
//            MBSdata->uxd[M_FL]= (1.0/J_M)*(VT*voltage[M_FL] -D_M*MBSdata->ux[n+M_FL]-Cpl[M_FL]);
//            MBSdata->uxd[M_RR]= (1.0/J_M)*(VT*voltage[M_RR] -D_M*MBSdata->ux[n+M_RR]-Cpl[M_RR]);
//            //MBSdata->uxd[M_RL]= (1.0/J_M)*(VT*voltage[M_RL] -D_M*MBSdata->ux[n+M_RL]-Cpl[M_RL]);
//           break;
//            case 3:
//            // Motor (Electrical+Mechanical) ODE
//            // ux:motor position, velocity, current uxd: motor velocity, acceleration, current derivative
//            //update motor velocities:
//            for (i=0; i<n; i++)
//            {
//                MBSdata->uxd[i]=MBSdata->ux[i+n];
//            }

//            rho = MBSdata->user_IO->actuatorsStruct->acs[M_FR]->GearRatio;
//            R_M = MBSdata->user_IO->actuatorsStruct->acs[M_FR]->Resistance;
//            K_W = MBSdata->user_IO->actuatorsStruct->acs[M_FR]->Kbemf;
//            L_M = MBSdata->user_IO->actuatorsStruct->acs[M_FR]->Inductance;
//            KT = K_W;

//            J_M = MBSdata->user_IO->actuatorsStruct->acs[M_RR]->Inertia;
//            VT  = rho*(KT)/R_M;
//            D_M = MBSdata->user_IO->actuatorsStruct->acs[M_RR]->Damping;
//            Ks  = MBSdata->user_IO->actuatorsStruct->acs[M_RR]->SeriesSpring;
//            Ds  = MBSdata->user_IO->actuatorsStruct->acs[M_RR]->SeriesDamping;

//            // computing the transmission torque (coupling between motor and load)
//            Cpl[M_FR]=Ks*(MBSdata->ux[M_FR]-MBSdata->q[R2_FR])+Ds*(MBSdata->uxd[M_FR]-MBSdata->qd[R2_FR]);
//            Cpl[M_FL]=Ks*(MBSdata->ux[M_FL]-MBSdata->q[R2_FL])+Ds*(MBSdata->uxd[M_FL]-MBSdata->qd[R2_FL]);
//            Cpl[M_RR]=Ks*(MBSdata->ux[M_RR]-MBSdata->q[R2_RR])+Ds*(MBSdata->uxd[M_RR]-MBSdata->qd[R2_RR]);
//            //Cpl[M_RL]=Ks*(MBSdata->ux[M_RL]-MBSdata->q[R2_RL])+Ds*(MBSdata->uxd[M_RL]-MBSdata->qd[R2_RL]);
//            // update motor acceleration:
//            MBSdata->uxd[M_FR+n]= (1.0/J_M)*(KT*MBSdata->ux[2*n+R2_FR] -D_M*MBSdata->ux[n+M_FR]-Cpl[M_FR]);
//            MBSdata->uxd[M_FL+n]= (1.0/J_M)*(KT*MBSdata->ux[2*n+R2_FL] -D_M*MBSdata->ux[n+M_FL]-Cpl[M_FL]);
//            MBSdata->uxd[M_RR+n]= (1.0/J_M)*(KT*MBSdata->ux[2*n+R2_RR] -D_M*MBSdata->ux[n+M_RR]-Cpl[M_RR]);
//            //MBSdata->uxd[M_RL+n]= (1.0/J_M)*(KT*MBSdata->ux[2*n+R2_RL] -D_M*MBSdata->ux[n+M_RL]-Cpl[M_RL]);

//            // update current derivative:
//            MBSdata->uxd[M_FR+2*n]=(1.0/L_M)*(voltage[M_FR] -R_M*MBSdata->ux[2*n+M_FR]-K_W*rho* MBSdata->qd[R2_FR]);
//            MBSdata->uxd[M_FL+2*n]=(1.0/L_M)*(voltage[M_FL] -R_M*MBSdata->ux[2*n+M_FL]-K_W*rho* MBSdata->qd[R2_FL]);
//            MBSdata->uxd[M_RR+2*n]=(1.0/L_M)*(voltage[M_RR] -R_M*MBSdata->ux[2*n+M_RR]-K_W*rho* MBSdata->qd[R2_RR]);
//            //MBSdata->uxd[M_RL+2*n]=(1.0/L_M)*(voltage[M_RL] -R_M*MBSdata->ux[2*n+M_RL]-K_W*rho* MBSdata->qd[R2_RL]);
//           break;
//            default:
//            printf("detault actuator order (1) selected \n");
//            goto justElectrical;
//           break;
            }
    }

}

