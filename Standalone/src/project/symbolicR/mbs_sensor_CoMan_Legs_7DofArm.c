//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Fri Jul 11 12:29:51 2014
//
//	==> Project name : CoMan_Legs_7DofArm
//	==> using XML input file 
//
//	==> Number of joints : 35
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 43538
//
//	==> All Parameter Symbols included
//	==> Generation Time :  0.750 seconds
//	==> Post-Processing :  1.050 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void  sensor(MBSsensorStruct *sens, 
              MBSdataStruct *s,
              int isens)
{ 
 
#include "mbs_sensor_CoMan_Legs_7DofArm.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C4 = cos(q[4]);
  S4 = sin(q[4]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);

// = = Block_0_0_0_0_0_2 = = 
 
// Trigonometric Variables  

  C7 = cos(q[7]);
  S7 = sin(q[7]);
  C8 = cos(q[8]);
  S8 = sin(q[8]);
  C9 = cos(q[9]);
  S9 = sin(q[9]);
  C10 = cos(q[10]);
  S10 = sin(q[10]);
  C11 = cos(q[11]);
  S11 = sin(q[11]);
  C12 = cos(q[12]);
  S12 = sin(q[12]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);
  C15 = cos(q[15]);
  S15 = sin(q[15]);
  C16 = cos(q[16]);
  S16 = sin(q[16]);
  C17 = cos(q[17]);
  S17 = sin(q[17]);
  C18 = cos(q[18]);
  S18 = sin(q[18]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C19 = cos(q[19]);
  S19 = sin(q[19]);
  C20 = cos(q[20]);
  S20 = sin(q[20]);
  C21 = cos(q[21]);
  S21 = sin(q[21]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C22 = cos(q[22]);
  S22 = sin(q[22]);
  C23 = cos(q[23]);
  S23 = sin(q[23]);
  C24 = cos(q[24]);
  S24 = sin(q[24]);
  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);
  C27 = cos(q[27]);
  S27 = sin(q[27]);
  C28 = cos(q[28]);
  S28 = sin(q[28]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C29 = cos(q[29]);
  S29 = sin(q[29]);
  C30 = cos(q[30]);
  S30 = sin(q[30]);
  C31 = cos(q[31]);
  S31 = sin(q[31]);
  C32 = cos(q[32]);
  S32 = sin(q[32]);
  C33 = cos(q[33]);
  S33 = sin(q[33]);
  C34 = cos(q[34]);
  S34 = sin(q[34]);
  C35 = cos(q[35]);
  S35 = sin(q[35]);

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_0_1 = = 
 
// Sensor Kinematics 


    ROcp0_25 = S4*S5;
    ROcp0_35 = -C4*S5;
    ROcp0_85 = -S4*C5;
    ROcp0_95 = C4*C5;
    ROcp0_16 = C5*C6;
    ROcp0_26 = ROcp0_25*C6+C4*S6;
    ROcp0_36 = ROcp0_35*C6+S4*S6;
    ROcp0_46 = -C5*S6;
    ROcp0_56 = -(ROcp0_25*S6-C4*C6);
    ROcp0_66 = -(ROcp0_35*S6-S4*C6);
    OMcp0_25 = qd[5]*C4;
    OMcp0_35 = qd[5]*S4;
    OMcp0_16 = qd[4]+qd[6]*S5;
    OMcp0_26 = OMcp0_25+ROcp0_85*qd[6];
    OMcp0_36 = OMcp0_35+ROcp0_95*qd[6];
    OPcp0_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp0_26 = ROcp0_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp0_35*S5-ROcp0_95*qd[4]);
    OPcp0_36 = ROcp0_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp0_25*S5-ROcp0_85*qd[4]);
    RLcp0_136 = s->dpt[1][1]*ROcp0_16+s->dpt[3][1]*S5+ROcp0_46*s->dpt[2][1];
    RLcp0_236 = s->dpt[1][1]*ROcp0_26+s->dpt[3][1]*ROcp0_85+ROcp0_56*s->dpt[2][1];
    RLcp0_336 = s->dpt[1][1]*ROcp0_36+s->dpt[3][1]*ROcp0_95+ROcp0_66*s->dpt[2][1];
    POcp0_136 = RLcp0_136+q[1];
    POcp0_236 = RLcp0_236+q[2];
    POcp0_336 = RLcp0_336+q[3];
    JTcp0_136_5 = -(RLcp0_236*S4-RLcp0_336*C4);
    JTcp0_236_5 = RLcp0_136*S4;
    JTcp0_336_5 = -RLcp0_136*C4;
    JTcp0_136_6 = -(RLcp0_236*ROcp0_95-RLcp0_336*ROcp0_85);
    JTcp0_236_6 = RLcp0_136*ROcp0_95-RLcp0_336*S5;
    JTcp0_336_6 = -(RLcp0_136*ROcp0_85-RLcp0_236*S5);
    ORcp0_136 = OMcp0_26*RLcp0_336-OMcp0_36*RLcp0_236;
    ORcp0_236 = -(OMcp0_16*RLcp0_336-OMcp0_36*RLcp0_136);
    ORcp0_336 = OMcp0_16*RLcp0_236-OMcp0_26*RLcp0_136;
    VIcp0_136 = ORcp0_136+qd[1];
    VIcp0_236 = ORcp0_236+qd[2];
    VIcp0_336 = ORcp0_336+qd[3];
    ACcp0_136 = qdd[1]+OMcp0_26*ORcp0_336-OMcp0_36*ORcp0_236+OPcp0_26*RLcp0_336-OPcp0_36*RLcp0_236;
    ACcp0_236 = qdd[2]-OMcp0_16*ORcp0_336+OMcp0_36*ORcp0_136-OPcp0_16*RLcp0_336+OPcp0_36*RLcp0_136;
    ACcp0_336 = qdd[3]+OMcp0_16*ORcp0_236-OMcp0_26*ORcp0_136+OPcp0_16*RLcp0_236-OPcp0_26*RLcp0_136;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp0_136;
    sens->P[2] = POcp0_236;
    sens->P[3] = POcp0_336;
    sens->R[1][1] = ROcp0_16;
    sens->R[1][2] = ROcp0_26;
    sens->R[1][3] = ROcp0_36;
    sens->R[2][1] = ROcp0_46;
    sens->R[2][2] = ROcp0_56;
    sens->R[2][3] = ROcp0_66;
    sens->R[3][1] = S5;
    sens->R[3][2] = ROcp0_85;
    sens->R[3][3] = ROcp0_95;
    sens->V[1] = VIcp0_136;
    sens->V[2] = VIcp0_236;
    sens->V[3] = VIcp0_336;
    sens->OM[1] = OMcp0_16;
    sens->OM[2] = OMcp0_26;
    sens->OM[3] = OMcp0_36;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp0_136_5;
    sens->J[1][6] = JTcp0_136_6;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = -RLcp0_336;
    sens->J[2][5] = JTcp0_236_5;
    sens->J[2][6] = JTcp0_236_6;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = RLcp0_236;
    sens->J[3][5] = JTcp0_336_5;
    sens->J[3][6] = JTcp0_336_6;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp0_85;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp0_95;
    sens->A[1] = ACcp0_136;
    sens->A[2] = ACcp0_236;
    sens->A[3] = ACcp0_336;
    sens->OMP[1] = OPcp0_16;
    sens->OMP[2] = OPcp0_26;
    sens->OMP[3] = OPcp0_36;
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_0_1 = = 
 
// Sensor Kinematics 


    ROcp1_25 = S4*S5;
    ROcp1_35 = -C4*S5;
    ROcp1_85 = -S4*C5;
    ROcp1_95 = C4*C5;
    ROcp1_16 = C5*C6;
    ROcp1_26 = ROcp1_25*C6+C4*S6;
    ROcp1_36 = ROcp1_35*C6+S4*S6;
    ROcp1_46 = -C5*S6;
    ROcp1_56 = -(ROcp1_25*S6-C4*C6);
    ROcp1_66 = -(ROcp1_35*S6-S4*C6);
    OMcp1_25 = qd[5]*C4;
    OMcp1_35 = qd[5]*S4;
    OMcp1_16 = qd[4]+qd[6]*S5;
    OMcp1_26 = OMcp1_25+ROcp1_85*qd[6];
    OMcp1_36 = OMcp1_35+ROcp1_95*qd[6];
    OPcp1_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp1_26 = ROcp1_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp1_35*S5-ROcp1_95*qd[4]);
    OPcp1_36 = ROcp1_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp1_25*S5-ROcp1_85*qd[4]);
    RLcp1_137 = s->dpt[1][2]*ROcp1_16+s->dpt[3][2]*S5+ROcp1_46*s->dpt[2][2];
    RLcp1_237 = s->dpt[1][2]*ROcp1_26+s->dpt[3][2]*ROcp1_85+ROcp1_56*s->dpt[2][2];
    RLcp1_337 = s->dpt[1][2]*ROcp1_36+s->dpt[3][2]*ROcp1_95+ROcp1_66*s->dpt[2][2];
    POcp1_137 = RLcp1_137+q[1];
    POcp1_237 = RLcp1_237+q[2];
    POcp1_337 = RLcp1_337+q[3];
    JTcp1_137_5 = -(RLcp1_237*S4-RLcp1_337*C4);
    JTcp1_237_5 = RLcp1_137*S4;
    JTcp1_337_5 = -RLcp1_137*C4;
    JTcp1_137_6 = -(RLcp1_237*ROcp1_95-RLcp1_337*ROcp1_85);
    JTcp1_237_6 = RLcp1_137*ROcp1_95-RLcp1_337*S5;
    JTcp1_337_6 = -(RLcp1_137*ROcp1_85-RLcp1_237*S5);
    ORcp1_137 = OMcp1_26*RLcp1_337-OMcp1_36*RLcp1_237;
    ORcp1_237 = -(OMcp1_16*RLcp1_337-OMcp1_36*RLcp1_137);
    ORcp1_337 = OMcp1_16*RLcp1_237-OMcp1_26*RLcp1_137;
    VIcp1_137 = ORcp1_137+qd[1];
    VIcp1_237 = ORcp1_237+qd[2];
    VIcp1_337 = ORcp1_337+qd[3];
    ACcp1_137 = qdd[1]+OMcp1_26*ORcp1_337-OMcp1_36*ORcp1_237+OPcp1_26*RLcp1_337-OPcp1_36*RLcp1_237;
    ACcp1_237 = qdd[2]-OMcp1_16*ORcp1_337+OMcp1_36*ORcp1_137-OPcp1_16*RLcp1_337+OPcp1_36*RLcp1_137;
    ACcp1_337 = qdd[3]+OMcp1_16*ORcp1_237-OMcp1_26*ORcp1_137+OPcp1_16*RLcp1_237-OPcp1_26*RLcp1_137;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_137;
    sens->P[2] = POcp1_237;
    sens->P[3] = POcp1_337;
    sens->R[1][1] = ROcp1_16;
    sens->R[1][2] = ROcp1_26;
    sens->R[1][3] = ROcp1_36;
    sens->R[2][1] = ROcp1_46;
    sens->R[2][2] = ROcp1_56;
    sens->R[2][3] = ROcp1_66;
    sens->R[3][1] = S5;
    sens->R[3][2] = ROcp1_85;
    sens->R[3][3] = ROcp1_95;
    sens->V[1] = VIcp1_137;
    sens->V[2] = VIcp1_237;
    sens->V[3] = VIcp1_337;
    sens->OM[1] = OMcp1_16;
    sens->OM[2] = OMcp1_26;
    sens->OM[3] = OMcp1_36;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp1_137_5;
    sens->J[1][6] = JTcp1_137_6;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = -RLcp1_337;
    sens->J[2][5] = JTcp1_237_5;
    sens->J[2][6] = JTcp1_237_6;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = RLcp1_237;
    sens->J[3][5] = JTcp1_337_5;
    sens->J[3][6] = JTcp1_337_6;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp1_85;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp1_95;
    sens->A[1] = ACcp1_137;
    sens->A[2] = ACcp1_237;
    sens->A[3] = ACcp1_337;
    sens->OMP[1] = OPcp1_16;
    sens->OMP[2] = OPcp1_26;
    sens->OMP[3] = OPcp1_36;
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_0_1 = = 
 
// Sensor Kinematics 


    ROcp2_25 = S4*S5;
    ROcp2_35 = -C4*S5;
    ROcp2_85 = -S4*C5;
    ROcp2_95 = C4*C5;
    ROcp2_16 = C5*C6;
    ROcp2_26 = ROcp2_25*C6+C4*S6;
    ROcp2_36 = ROcp2_35*C6+S4*S6;
    ROcp2_46 = -C5*S6;
    ROcp2_56 = -(ROcp2_25*S6-C4*C6);
    ROcp2_66 = -(ROcp2_35*S6-S4*C6);
    OMcp2_25 = qd[5]*C4;
    OMcp2_35 = qd[5]*S4;
    OMcp2_16 = qd[4]+qd[6]*S5;
    OMcp2_26 = OMcp2_25+ROcp2_85*qd[6];
    OMcp2_36 = OMcp2_35+ROcp2_95*qd[6];
    OPcp2_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp2_26 = ROcp2_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp2_35*S5-ROcp2_95*qd[4]);
    OPcp2_36 = ROcp2_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp2_25*S5-ROcp2_85*qd[4]);
    RLcp2_138 = s->dpt[2][3]*ROcp2_46+ROcp2_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp2_238 = s->dpt[2][3]*ROcp2_56+ROcp2_26*s->dpt[1][3]+ROcp2_85*s->dpt[3][3];
    RLcp2_338 = s->dpt[2][3]*ROcp2_66+ROcp2_36*s->dpt[1][3]+ROcp2_95*s->dpt[3][3];
    POcp2_138 = RLcp2_138+q[1];
    POcp2_238 = RLcp2_238+q[2];
    POcp2_338 = RLcp2_338+q[3];
    JTcp2_138_5 = -(RLcp2_238*S4-RLcp2_338*C4);
    JTcp2_238_5 = RLcp2_138*S4;
    JTcp2_338_5 = -RLcp2_138*C4;
    JTcp2_138_6 = -(RLcp2_238*ROcp2_95-RLcp2_338*ROcp2_85);
    JTcp2_238_6 = RLcp2_138*ROcp2_95-RLcp2_338*S5;
    JTcp2_338_6 = -(RLcp2_138*ROcp2_85-RLcp2_238*S5);
    ORcp2_138 = OMcp2_26*RLcp2_338-OMcp2_36*RLcp2_238;
    ORcp2_238 = -(OMcp2_16*RLcp2_338-OMcp2_36*RLcp2_138);
    ORcp2_338 = OMcp2_16*RLcp2_238-OMcp2_26*RLcp2_138;
    VIcp2_138 = ORcp2_138+qd[1];
    VIcp2_238 = ORcp2_238+qd[2];
    VIcp2_338 = ORcp2_338+qd[3];
    ACcp2_138 = qdd[1]+OMcp2_26*ORcp2_338-OMcp2_36*ORcp2_238+OPcp2_26*RLcp2_338-OPcp2_36*RLcp2_238;
    ACcp2_238 = qdd[2]-OMcp2_16*ORcp2_338+OMcp2_36*ORcp2_138-OPcp2_16*RLcp2_338+OPcp2_36*RLcp2_138;
    ACcp2_338 = qdd[3]+OMcp2_16*ORcp2_238-OMcp2_26*ORcp2_138+OPcp2_16*RLcp2_238-OPcp2_26*RLcp2_138;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_138;
    sens->P[2] = POcp2_238;
    sens->P[3] = POcp2_338;
    sens->R[1][1] = ROcp2_16;
    sens->R[1][2] = ROcp2_26;
    sens->R[1][3] = ROcp2_36;
    sens->R[2][1] = ROcp2_46;
    sens->R[2][2] = ROcp2_56;
    sens->R[2][3] = ROcp2_66;
    sens->R[3][1] = S5;
    sens->R[3][2] = ROcp2_85;
    sens->R[3][3] = ROcp2_95;
    sens->V[1] = VIcp2_138;
    sens->V[2] = VIcp2_238;
    sens->V[3] = VIcp2_338;
    sens->OM[1] = OMcp2_16;
    sens->OM[2] = OMcp2_26;
    sens->OM[3] = OMcp2_36;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp2_138_5;
    sens->J[1][6] = JTcp2_138_6;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = -RLcp2_338;
    sens->J[2][5] = JTcp2_238_5;
    sens->J[2][6] = JTcp2_238_6;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = RLcp2_238;
    sens->J[3][5] = JTcp2_338_5;
    sens->J[3][6] = JTcp2_338_6;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp2_85;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp2_95;
    sens->A[1] = ACcp2_138;
    sens->A[2] = ACcp2_238;
    sens->A[3] = ACcp2_338;
    sens->OMP[1] = OPcp2_16;
    sens->OMP[2] = OPcp2_26;
    sens->OMP[3] = OPcp2_36;
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_0_1 = = 
 
// Sensor Kinematics 


    ROcp3_25 = S4*S5;
    ROcp3_35 = -C4*S5;
    ROcp3_85 = -S4*C5;
    ROcp3_95 = C4*C5;
    ROcp3_16 = C5*C6;
    ROcp3_26 = ROcp3_25*C6+C4*S6;
    ROcp3_36 = ROcp3_35*C6+S4*S6;
    ROcp3_46 = -C5*S6;
    ROcp3_56 = -(ROcp3_25*S6-C4*C6);
    ROcp3_66 = -(ROcp3_35*S6-S4*C6);
    OMcp3_25 = qd[5]*C4;
    OMcp3_35 = qd[5]*S4;
    OMcp3_16 = qd[4]+qd[6]*S5;
    OMcp3_26 = OMcp3_25+ROcp3_85*qd[6];
    OMcp3_36 = OMcp3_35+ROcp3_95*qd[6];
    OPcp3_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp3_26 = ROcp3_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp3_35*S5-ROcp3_95*qd[4]);
    OPcp3_36 = ROcp3_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp3_25*S5-ROcp3_85*qd[4]);
    RLcp3_139 = ROcp3_16*s->dpt[1][4]+ROcp3_46*s->dpt[2][4]+s->dpt[3][4]*S5;
    RLcp3_239 = ROcp3_26*s->dpt[1][4]+ROcp3_56*s->dpt[2][4]+ROcp3_85*s->dpt[3][4];
    RLcp3_339 = ROcp3_36*s->dpt[1][4]+ROcp3_66*s->dpt[2][4]+ROcp3_95*s->dpt[3][4];
    POcp3_139 = RLcp3_139+q[1];
    POcp3_239 = RLcp3_239+q[2];
    POcp3_339 = RLcp3_339+q[3];
    JTcp3_139_5 = -(RLcp3_239*S4-RLcp3_339*C4);
    JTcp3_239_5 = RLcp3_139*S4;
    JTcp3_339_5 = -RLcp3_139*C4;
    JTcp3_139_6 = -(RLcp3_239*ROcp3_95-RLcp3_339*ROcp3_85);
    JTcp3_239_6 = RLcp3_139*ROcp3_95-RLcp3_339*S5;
    JTcp3_339_6 = -(RLcp3_139*ROcp3_85-RLcp3_239*S5);
    ORcp3_139 = OMcp3_26*RLcp3_339-OMcp3_36*RLcp3_239;
    ORcp3_239 = -(OMcp3_16*RLcp3_339-OMcp3_36*RLcp3_139);
    ORcp3_339 = OMcp3_16*RLcp3_239-OMcp3_26*RLcp3_139;
    VIcp3_139 = ORcp3_139+qd[1];
    VIcp3_239 = ORcp3_239+qd[2];
    VIcp3_339 = ORcp3_339+qd[3];
    ACcp3_139 = qdd[1]+OMcp3_26*ORcp3_339-OMcp3_36*ORcp3_239+OPcp3_26*RLcp3_339-OPcp3_36*RLcp3_239;
    ACcp3_239 = qdd[2]-OMcp3_16*ORcp3_339+OMcp3_36*ORcp3_139-OPcp3_16*RLcp3_339+OPcp3_36*RLcp3_139;
    ACcp3_339 = qdd[3]+OMcp3_16*ORcp3_239-OMcp3_26*ORcp3_139+OPcp3_16*RLcp3_239-OPcp3_26*RLcp3_139;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_139;
    sens->P[2] = POcp3_239;
    sens->P[3] = POcp3_339;
    sens->R[1][1] = ROcp3_16;
    sens->R[1][2] = ROcp3_26;
    sens->R[1][3] = ROcp3_36;
    sens->R[2][1] = ROcp3_46;
    sens->R[2][2] = ROcp3_56;
    sens->R[2][3] = ROcp3_66;
    sens->R[3][1] = S5;
    sens->R[3][2] = ROcp3_85;
    sens->R[3][3] = ROcp3_95;
    sens->V[1] = VIcp3_139;
    sens->V[2] = VIcp3_239;
    sens->V[3] = VIcp3_339;
    sens->OM[1] = OMcp3_16;
    sens->OM[2] = OMcp3_26;
    sens->OM[3] = OMcp3_36;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp3_139_5;
    sens->J[1][6] = JTcp3_139_6;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = -RLcp3_339;
    sens->J[2][5] = JTcp3_239_5;
    sens->J[2][6] = JTcp3_239_6;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = RLcp3_239;
    sens->J[3][5] = JTcp3_339_5;
    sens->J[3][6] = JTcp3_339_6;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp3_85;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp3_95;
    sens->A[1] = ACcp3_139;
    sens->A[2] = ACcp3_239;
    sens->A[3] = ACcp3_339;
    sens->OMP[1] = OPcp3_16;
    sens->OMP[2] = OPcp3_26;
    sens->OMP[3] = OPcp3_36;
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_1 = = 
 
// Sensor Kinematics 


    ROcp4_25 = S4*S5;
    ROcp4_35 = -C4*S5;
    ROcp4_85 = -S4*C5;
    ROcp4_95 = C4*C5;
    ROcp4_16 = C5*C6;
    ROcp4_26 = ROcp4_25*C6+C4*S6;
    ROcp4_36 = ROcp4_35*C6+S4*S6;
    ROcp4_46 = -C5*S6;
    ROcp4_56 = -(ROcp4_25*S6-C4*C6);
    ROcp4_66 = -(ROcp4_35*S6-S4*C6);
    OMcp4_25 = qd[5]*C4;
    OMcp4_35 = qd[5]*S4;
    OMcp4_16 = qd[4]+qd[6]*S5;
    OMcp4_26 = OMcp4_25+ROcp4_85*qd[6];
    OMcp4_36 = OMcp4_35+ROcp4_95*qd[6];
    OPcp4_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp4_26 = ROcp4_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp4_35*S5-ROcp4_95*qd[4]);
    OPcp4_36 = ROcp4_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp4_25*S5-ROcp4_85*qd[4]);
    RLcp4_140 = s->dpt[1][5]*ROcp4_16+s->dpt[2][5]*ROcp4_46+s->dpt[3][5]*S5;
    RLcp4_240 = s->dpt[1][5]*ROcp4_26+s->dpt[2][5]*ROcp4_56+s->dpt[3][5]*ROcp4_85;
    RLcp4_340 = s->dpt[1][5]*ROcp4_36+s->dpt[2][5]*ROcp4_66+s->dpt[3][5]*ROcp4_95;
    POcp4_140 = RLcp4_140+q[1];
    POcp4_240 = RLcp4_240+q[2];
    POcp4_340 = RLcp4_340+q[3];
    JTcp4_140_5 = -(RLcp4_240*S4-RLcp4_340*C4);
    JTcp4_240_5 = RLcp4_140*S4;
    JTcp4_340_5 = -RLcp4_140*C4;
    JTcp4_140_6 = -(RLcp4_240*ROcp4_95-RLcp4_340*ROcp4_85);
    JTcp4_240_6 = RLcp4_140*ROcp4_95-RLcp4_340*S5;
    JTcp4_340_6 = -(RLcp4_140*ROcp4_85-RLcp4_240*S5);
    ORcp4_140 = OMcp4_26*RLcp4_340-OMcp4_36*RLcp4_240;
    ORcp4_240 = -(OMcp4_16*RLcp4_340-OMcp4_36*RLcp4_140);
    ORcp4_340 = OMcp4_16*RLcp4_240-OMcp4_26*RLcp4_140;
    VIcp4_140 = ORcp4_140+qd[1];
    VIcp4_240 = ORcp4_240+qd[2];
    VIcp4_340 = ORcp4_340+qd[3];
    ACcp4_140 = qdd[1]+OMcp4_26*ORcp4_340-OMcp4_36*ORcp4_240+OPcp4_26*RLcp4_340-OPcp4_36*RLcp4_240;
    ACcp4_240 = qdd[2]-OMcp4_16*ORcp4_340+OMcp4_36*ORcp4_140-OPcp4_16*RLcp4_340+OPcp4_36*RLcp4_140;
    ACcp4_340 = qdd[3]+OMcp4_16*ORcp4_240-OMcp4_26*ORcp4_140+OPcp4_16*RLcp4_240-OPcp4_26*RLcp4_140;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_140;
    sens->P[2] = POcp4_240;
    sens->P[3] = POcp4_340;
    sens->R[1][1] = ROcp4_16;
    sens->R[1][2] = ROcp4_26;
    sens->R[1][3] = ROcp4_36;
    sens->R[2][1] = ROcp4_46;
    sens->R[2][2] = ROcp4_56;
    sens->R[2][3] = ROcp4_66;
    sens->R[3][1] = S5;
    sens->R[3][2] = ROcp4_85;
    sens->R[3][3] = ROcp4_95;
    sens->V[1] = VIcp4_140;
    sens->V[2] = VIcp4_240;
    sens->V[3] = VIcp4_340;
    sens->OM[1] = OMcp4_16;
    sens->OM[2] = OMcp4_26;
    sens->OM[3] = OMcp4_36;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp4_140_5;
    sens->J[1][6] = JTcp4_140_6;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = -RLcp4_340;
    sens->J[2][5] = JTcp4_240_5;
    sens->J[2][6] = JTcp4_240_6;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = RLcp4_240;
    sens->J[3][5] = JTcp4_340_5;
    sens->J[3][6] = JTcp4_340_6;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp4_85;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp4_95;
    sens->A[1] = ACcp4_140;
    sens->A[2] = ACcp4_240;
    sens->A[3] = ACcp4_340;
    sens->OMP[1] = OPcp4_16;
    sens->OMP[2] = OPcp4_26;
    sens->OMP[3] = OPcp4_36;
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_1 = = 
 
// Sensor Kinematics 


    ROcp5_25 = S4*S5;
    ROcp5_35 = -C4*S5;
    ROcp5_85 = -S4*C5;
    ROcp5_95 = C4*C5;
    ROcp5_16 = C5*C6;
    ROcp5_26 = ROcp5_25*C6+C4*S6;
    ROcp5_36 = ROcp5_35*C6+S4*S6;
    ROcp5_46 = -C5*S6;
    ROcp5_56 = -(ROcp5_25*S6-C4*C6);
    ROcp5_66 = -(ROcp5_35*S6-S4*C6);
    OMcp5_25 = qd[5]*C4;
    OMcp5_35 = qd[5]*S4;
    OMcp5_16 = qd[4]+qd[6]*S5;
    OMcp5_26 = OMcp5_25+ROcp5_85*qd[6];
    OMcp5_36 = OMcp5_35+ROcp5_95*qd[6];
    OPcp5_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp5_26 = ROcp5_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp5_35*S5-ROcp5_95*qd[4]);
    OPcp5_36 = ROcp5_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp5_25*S5-ROcp5_85*qd[4]);

// = = Block_1_0_0_6_0_2 = = 
 
// Sensor Kinematics 


    ROcp5_17 = ROcp5_16*C7-S5*S7;
    ROcp5_27 = ROcp5_26*C7-ROcp5_85*S7;
    ROcp5_37 = ROcp5_36*C7-ROcp5_95*S7;
    ROcp5_77 = ROcp5_16*S7+S5*C7;
    ROcp5_87 = ROcp5_26*S7+ROcp5_85*C7;
    ROcp5_97 = ROcp5_36*S7+ROcp5_95*C7;
    RLcp5_17 = s->dpt[1][1]*ROcp5_16+s->dpt[3][1]*S5+ROcp5_46*s->dpt[2][1];
    RLcp5_27 = s->dpt[1][1]*ROcp5_26+s->dpt[3][1]*ROcp5_85+ROcp5_56*s->dpt[2][1];
    RLcp5_37 = s->dpt[1][1]*ROcp5_36+s->dpt[3][1]*ROcp5_95+ROcp5_66*s->dpt[2][1];
    OMcp5_17 = OMcp5_16+ROcp5_46*qd[7];
    OMcp5_27 = OMcp5_26+ROcp5_56*qd[7];
    OMcp5_37 = OMcp5_36+ROcp5_66*qd[7];
    ORcp5_17 = OMcp5_26*RLcp5_37-OMcp5_36*RLcp5_27;
    ORcp5_27 = -(OMcp5_16*RLcp5_37-OMcp5_36*RLcp5_17);
    ORcp5_37 = OMcp5_16*RLcp5_27-OMcp5_26*RLcp5_17;
    OPcp5_17 = OPcp5_16+ROcp5_46*qdd[7]+qd[7]*(OMcp5_26*ROcp5_66-OMcp5_36*ROcp5_56);
    OPcp5_27 = OPcp5_26+ROcp5_56*qdd[7]-qd[7]*(OMcp5_16*ROcp5_66-OMcp5_36*ROcp5_46);
    OPcp5_37 = OPcp5_36+ROcp5_66*qdd[7]+qd[7]*(OMcp5_16*ROcp5_56-OMcp5_26*ROcp5_46);
    RLcp5_141 = s->dpt[1][6]*ROcp5_17+s->dpt[3][6]*ROcp5_77+ROcp5_46*s->dpt[2][6];
    RLcp5_241 = s->dpt[1][6]*ROcp5_27+s->dpt[3][6]*ROcp5_87+ROcp5_56*s->dpt[2][6];
    RLcp5_341 = s->dpt[1][6]*ROcp5_37+s->dpt[3][6]*ROcp5_97+ROcp5_66*s->dpt[2][6];
    POcp5_141 = RLcp5_141+RLcp5_17+q[1];
    POcp5_241 = RLcp5_241+RLcp5_27+q[2];
    POcp5_341 = RLcp5_341+RLcp5_37+q[3];
    JTcp5_241_4 = -(RLcp5_341+RLcp5_37);
    JTcp5_341_4 = RLcp5_241+RLcp5_27;
    JTcp5_141_5 = C4*(RLcp5_341+RLcp5_37)-S4*(RLcp5_241+RLcp5_27);
    JTcp5_241_5 = S4*(RLcp5_141+RLcp5_17);
    JTcp5_341_5 = -C4*(RLcp5_141+RLcp5_17);
    JTcp5_141_6 = ROcp5_85*(RLcp5_341+RLcp5_37)-ROcp5_95*(RLcp5_241+RLcp5_27);
    JTcp5_241_6 = ROcp5_95*(RLcp5_141+RLcp5_17)-S5*(RLcp5_341+RLcp5_37);
    JTcp5_341_6 = -(ROcp5_85*(RLcp5_141+RLcp5_17)-S5*(RLcp5_241+RLcp5_27));
    JTcp5_141_7 = -(RLcp5_241*ROcp5_66-RLcp5_341*ROcp5_56);
    JTcp5_241_7 = RLcp5_141*ROcp5_66-RLcp5_341*ROcp5_46;
    JTcp5_341_7 = -(RLcp5_141*ROcp5_56-RLcp5_241*ROcp5_46);
    ORcp5_141 = OMcp5_27*RLcp5_341-OMcp5_37*RLcp5_241;
    ORcp5_241 = -(OMcp5_17*RLcp5_341-OMcp5_37*RLcp5_141);
    ORcp5_341 = OMcp5_17*RLcp5_241-OMcp5_27*RLcp5_141;
    VIcp5_141 = ORcp5_141+ORcp5_17+qd[1];
    VIcp5_241 = ORcp5_241+ORcp5_27+qd[2];
    VIcp5_341 = ORcp5_341+ORcp5_37+qd[3];
    ACcp5_141 = qdd[1]+OMcp5_26*ORcp5_37+OMcp5_27*ORcp5_341-OMcp5_36*ORcp5_27-OMcp5_37*ORcp5_241+OPcp5_26*RLcp5_37+
 OPcp5_27*RLcp5_341-OPcp5_36*RLcp5_27-OPcp5_37*RLcp5_241;
    ACcp5_241 = qdd[2]-OMcp5_16*ORcp5_37-OMcp5_17*ORcp5_341+OMcp5_36*ORcp5_17+OMcp5_37*ORcp5_141-OPcp5_16*RLcp5_37-
 OPcp5_17*RLcp5_341+OPcp5_36*RLcp5_17+OPcp5_37*RLcp5_141;
    ACcp5_341 = qdd[3]+OMcp5_16*ORcp5_27+OMcp5_17*ORcp5_241-OMcp5_26*ORcp5_17-OMcp5_27*ORcp5_141+OPcp5_16*RLcp5_27+
 OPcp5_17*RLcp5_241-OPcp5_26*RLcp5_17-OPcp5_27*RLcp5_141;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_141;
    sens->P[2] = POcp5_241;
    sens->P[3] = POcp5_341;
    sens->R[1][1] = ROcp5_17;
    sens->R[1][2] = ROcp5_27;
    sens->R[1][3] = ROcp5_37;
    sens->R[2][1] = ROcp5_46;
    sens->R[2][2] = ROcp5_56;
    sens->R[2][3] = ROcp5_66;
    sens->R[3][1] = ROcp5_77;
    sens->R[3][2] = ROcp5_87;
    sens->R[3][3] = ROcp5_97;
    sens->V[1] = VIcp5_141;
    sens->V[2] = VIcp5_241;
    sens->V[3] = VIcp5_341;
    sens->OM[1] = OMcp5_17;
    sens->OM[2] = OMcp5_27;
    sens->OM[3] = OMcp5_37;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp5_141_5;
    sens->J[1][6] = JTcp5_141_6;
    sens->J[1][7] = JTcp5_141_7;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp5_241_4;
    sens->J[2][5] = JTcp5_241_5;
    sens->J[2][6] = JTcp5_241_6;
    sens->J[2][7] = JTcp5_241_7;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp5_341_4;
    sens->J[3][5] = JTcp5_341_5;
    sens->J[3][6] = JTcp5_341_6;
    sens->J[3][7] = JTcp5_341_7;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp5_46;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp5_85;
    sens->J[5][7] = ROcp5_56;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp5_95;
    sens->J[6][7] = ROcp5_66;
    sens->A[1] = ACcp5_141;
    sens->A[2] = ACcp5_241;
    sens->A[3] = ACcp5_341;
    sens->OMP[1] = OPcp5_17;
    sens->OMP[2] = OPcp5_27;
    sens->OMP[3] = OPcp5_37;
 
// 
break;
case 7:
 


// = = Block_1_0_0_7_0_1 = = 
 
// Sensor Kinematics 


    ROcp6_25 = S4*S5;
    ROcp6_35 = -C4*S5;
    ROcp6_85 = -S4*C5;
    ROcp6_95 = C4*C5;
    ROcp6_16 = C5*C6;
    ROcp6_26 = ROcp6_25*C6+C4*S6;
    ROcp6_36 = ROcp6_35*C6+S4*S6;
    ROcp6_46 = -C5*S6;
    ROcp6_56 = -(ROcp6_25*S6-C4*C6);
    ROcp6_66 = -(ROcp6_35*S6-S4*C6);
    OMcp6_25 = qd[5]*C4;
    OMcp6_35 = qd[5]*S4;
    OMcp6_16 = qd[4]+qd[6]*S5;
    OMcp6_26 = OMcp6_25+ROcp6_85*qd[6];
    OMcp6_36 = OMcp6_35+ROcp6_95*qd[6];
    OPcp6_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp6_26 = ROcp6_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp6_35*S5-ROcp6_95*qd[4]);
    OPcp6_36 = ROcp6_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp6_25*S5-ROcp6_85*qd[4]);

// = = Block_1_0_0_7_0_2 = = 
 
// Sensor Kinematics 


    ROcp6_17 = ROcp6_16*C7-S5*S7;
    ROcp6_27 = ROcp6_26*C7-ROcp6_85*S7;
    ROcp6_37 = ROcp6_36*C7-ROcp6_95*S7;
    ROcp6_77 = ROcp6_16*S7+S5*C7;
    ROcp6_87 = ROcp6_26*S7+ROcp6_85*C7;
    ROcp6_97 = ROcp6_36*S7+ROcp6_95*C7;
    RLcp6_17 = s->dpt[1][1]*ROcp6_16+s->dpt[3][1]*S5+ROcp6_46*s->dpt[2][1];
    RLcp6_27 = s->dpt[1][1]*ROcp6_26+s->dpt[3][1]*ROcp6_85+ROcp6_56*s->dpt[2][1];
    RLcp6_37 = s->dpt[1][1]*ROcp6_36+s->dpt[3][1]*ROcp6_95+ROcp6_66*s->dpt[2][1];
    OMcp6_17 = OMcp6_16+ROcp6_46*qd[7];
    OMcp6_27 = OMcp6_26+ROcp6_56*qd[7];
    OMcp6_37 = OMcp6_36+ROcp6_66*qd[7];
    ORcp6_17 = OMcp6_26*RLcp6_37-OMcp6_36*RLcp6_27;
    ORcp6_27 = -(OMcp6_16*RLcp6_37-OMcp6_36*RLcp6_17);
    ORcp6_37 = OMcp6_16*RLcp6_27-OMcp6_26*RLcp6_17;
    OPcp6_17 = OPcp6_16+ROcp6_46*qdd[7]+qd[7]*(OMcp6_26*ROcp6_66-OMcp6_36*ROcp6_56);
    OPcp6_27 = OPcp6_26+ROcp6_56*qdd[7]-qd[7]*(OMcp6_16*ROcp6_66-OMcp6_36*ROcp6_46);
    OPcp6_37 = OPcp6_36+ROcp6_66*qdd[7]+qd[7]*(OMcp6_16*ROcp6_56-OMcp6_26*ROcp6_46);
    RLcp6_142 = ROcp6_17*s->dpt[1][7]+ROcp6_46*s->dpt[2][7]+ROcp6_77*s->dpt[3][7];
    RLcp6_242 = ROcp6_27*s->dpt[1][7]+ROcp6_56*s->dpt[2][7]+ROcp6_87*s->dpt[3][7];
    RLcp6_342 = ROcp6_37*s->dpt[1][7]+ROcp6_66*s->dpt[2][7]+ROcp6_97*s->dpt[3][7];
    POcp6_142 = RLcp6_142+RLcp6_17+q[1];
    POcp6_242 = RLcp6_242+RLcp6_27+q[2];
    POcp6_342 = RLcp6_342+RLcp6_37+q[3];
    JTcp6_242_4 = -(RLcp6_342+RLcp6_37);
    JTcp6_342_4 = RLcp6_242+RLcp6_27;
    JTcp6_142_5 = C4*(RLcp6_342+RLcp6_37)-S4*(RLcp6_242+RLcp6_27);
    JTcp6_242_5 = S4*(RLcp6_142+RLcp6_17);
    JTcp6_342_5 = -C4*(RLcp6_142+RLcp6_17);
    JTcp6_142_6 = ROcp6_85*(RLcp6_342+RLcp6_37)-ROcp6_95*(RLcp6_242+RLcp6_27);
    JTcp6_242_6 = ROcp6_95*(RLcp6_142+RLcp6_17)-S5*(RLcp6_342+RLcp6_37);
    JTcp6_342_6 = -(ROcp6_85*(RLcp6_142+RLcp6_17)-S5*(RLcp6_242+RLcp6_27));
    JTcp6_142_7 = -(RLcp6_242*ROcp6_66-RLcp6_342*ROcp6_56);
    JTcp6_242_7 = RLcp6_142*ROcp6_66-RLcp6_342*ROcp6_46;
    JTcp6_342_7 = -(RLcp6_142*ROcp6_56-RLcp6_242*ROcp6_46);
    ORcp6_142 = OMcp6_27*RLcp6_342-OMcp6_37*RLcp6_242;
    ORcp6_242 = -(OMcp6_17*RLcp6_342-OMcp6_37*RLcp6_142);
    ORcp6_342 = OMcp6_17*RLcp6_242-OMcp6_27*RLcp6_142;
    VIcp6_142 = ORcp6_142+ORcp6_17+qd[1];
    VIcp6_242 = ORcp6_242+ORcp6_27+qd[2];
    VIcp6_342 = ORcp6_342+ORcp6_37+qd[3];
    ACcp6_142 = qdd[1]+OMcp6_26*ORcp6_37+OMcp6_27*ORcp6_342-OMcp6_36*ORcp6_27-OMcp6_37*ORcp6_242+OPcp6_26*RLcp6_37+
 OPcp6_27*RLcp6_342-OPcp6_36*RLcp6_27-OPcp6_37*RLcp6_242;
    ACcp6_242 = qdd[2]-OMcp6_16*ORcp6_37-OMcp6_17*ORcp6_342+OMcp6_36*ORcp6_17+OMcp6_37*ORcp6_142-OPcp6_16*RLcp6_37-
 OPcp6_17*RLcp6_342+OPcp6_36*RLcp6_17+OPcp6_37*RLcp6_142;
    ACcp6_342 = qdd[3]+OMcp6_16*ORcp6_27+OMcp6_17*ORcp6_242-OMcp6_26*ORcp6_17-OMcp6_27*ORcp6_142+OPcp6_16*RLcp6_27+
 OPcp6_17*RLcp6_242-OPcp6_26*RLcp6_17-OPcp6_27*RLcp6_142;

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_142;
    sens->P[2] = POcp6_242;
    sens->P[3] = POcp6_342;
    sens->R[1][1] = ROcp6_17;
    sens->R[1][2] = ROcp6_27;
    sens->R[1][3] = ROcp6_37;
    sens->R[2][1] = ROcp6_46;
    sens->R[2][2] = ROcp6_56;
    sens->R[2][3] = ROcp6_66;
    sens->R[3][1] = ROcp6_77;
    sens->R[3][2] = ROcp6_87;
    sens->R[3][3] = ROcp6_97;
    sens->V[1] = VIcp6_142;
    sens->V[2] = VIcp6_242;
    sens->V[3] = VIcp6_342;
    sens->OM[1] = OMcp6_17;
    sens->OM[2] = OMcp6_27;
    sens->OM[3] = OMcp6_37;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp6_142_5;
    sens->J[1][6] = JTcp6_142_6;
    sens->J[1][7] = JTcp6_142_7;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp6_242_4;
    sens->J[2][5] = JTcp6_242_5;
    sens->J[2][6] = JTcp6_242_6;
    sens->J[2][7] = JTcp6_242_7;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp6_342_4;
    sens->J[3][5] = JTcp6_342_5;
    sens->J[3][6] = JTcp6_342_6;
    sens->J[3][7] = JTcp6_342_7;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp6_46;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp6_85;
    sens->J[5][7] = ROcp6_56;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp6_95;
    sens->J[6][7] = ROcp6_66;
    sens->A[1] = ACcp6_142;
    sens->A[2] = ACcp6_242;
    sens->A[3] = ACcp6_342;
    sens->OMP[1] = OPcp6_17;
    sens->OMP[2] = OPcp6_27;
    sens->OMP[3] = OPcp6_37;
 
// 
break;
case 8:
 


// = = Block_1_0_0_8_0_1 = = 
 
// Sensor Kinematics 


    ROcp7_25 = S4*S5;
    ROcp7_35 = -C4*S5;
    ROcp7_85 = -S4*C5;
    ROcp7_95 = C4*C5;
    ROcp7_16 = C5*C6;
    ROcp7_26 = ROcp7_25*C6+C4*S6;
    ROcp7_36 = ROcp7_35*C6+S4*S6;
    ROcp7_46 = -C5*S6;
    ROcp7_56 = -(ROcp7_25*S6-C4*C6);
    ROcp7_66 = -(ROcp7_35*S6-S4*C6);
    OMcp7_25 = qd[5]*C4;
    OMcp7_35 = qd[5]*S4;
    OMcp7_16 = qd[4]+qd[6]*S5;
    OMcp7_26 = OMcp7_25+ROcp7_85*qd[6];
    OMcp7_36 = OMcp7_35+ROcp7_95*qd[6];
    OPcp7_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp7_26 = ROcp7_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp7_35*S5-ROcp7_95*qd[4]);
    OPcp7_36 = ROcp7_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp7_25*S5-ROcp7_85*qd[4]);

// = = Block_1_0_0_8_0_2 = = 
 
// Sensor Kinematics 


    ROcp7_17 = ROcp7_16*C7-S5*S7;
    ROcp7_27 = ROcp7_26*C7-ROcp7_85*S7;
    ROcp7_37 = ROcp7_36*C7-ROcp7_95*S7;
    ROcp7_77 = ROcp7_16*S7+S5*C7;
    ROcp7_87 = ROcp7_26*S7+ROcp7_85*C7;
    ROcp7_97 = ROcp7_36*S7+ROcp7_95*C7;
    ROcp7_48 = ROcp7_46*C8+ROcp7_77*S8;
    ROcp7_58 = ROcp7_56*C8+ROcp7_87*S8;
    ROcp7_68 = ROcp7_66*C8+ROcp7_97*S8;
    ROcp7_78 = -(ROcp7_46*S8-ROcp7_77*C8);
    ROcp7_88 = -(ROcp7_56*S8-ROcp7_87*C8);
    ROcp7_98 = -(ROcp7_66*S8-ROcp7_97*C8);
    RLcp7_17 = s->dpt[1][1]*ROcp7_16+s->dpt[3][1]*S5+ROcp7_46*s->dpt[2][1];
    RLcp7_27 = s->dpt[1][1]*ROcp7_26+s->dpt[3][1]*ROcp7_85+ROcp7_56*s->dpt[2][1];
    RLcp7_37 = s->dpt[1][1]*ROcp7_36+s->dpt[3][1]*ROcp7_95+ROcp7_66*s->dpt[2][1];
    OMcp7_17 = OMcp7_16+ROcp7_46*qd[7];
    OMcp7_27 = OMcp7_26+ROcp7_56*qd[7];
    OMcp7_37 = OMcp7_36+ROcp7_66*qd[7];
    ORcp7_17 = OMcp7_26*RLcp7_37-OMcp7_36*RLcp7_27;
    ORcp7_27 = -(OMcp7_16*RLcp7_37-OMcp7_36*RLcp7_17);
    ORcp7_37 = OMcp7_16*RLcp7_27-OMcp7_26*RLcp7_17;
    OPcp7_17 = OPcp7_16+ROcp7_46*qdd[7]+qd[7]*(OMcp7_26*ROcp7_66-OMcp7_36*ROcp7_56);
    OPcp7_27 = OPcp7_26+ROcp7_56*qdd[7]-qd[7]*(OMcp7_16*ROcp7_66-OMcp7_36*ROcp7_46);
    OPcp7_37 = OPcp7_36+ROcp7_66*qdd[7]+qd[7]*(OMcp7_16*ROcp7_56-OMcp7_26*ROcp7_46);
    RLcp7_18 = s->dpt[1][6]*ROcp7_17+s->dpt[3][6]*ROcp7_77+ROcp7_46*s->dpt[2][6];
    RLcp7_28 = s->dpt[1][6]*ROcp7_27+s->dpt[3][6]*ROcp7_87+ROcp7_56*s->dpt[2][6];
    RLcp7_38 = s->dpt[1][6]*ROcp7_37+s->dpt[3][6]*ROcp7_97+ROcp7_66*s->dpt[2][6];
    OMcp7_18 = OMcp7_17+ROcp7_17*qd[8];
    OMcp7_28 = OMcp7_27+ROcp7_27*qd[8];
    OMcp7_38 = OMcp7_37+ROcp7_37*qd[8];
    ORcp7_18 = OMcp7_27*RLcp7_38-OMcp7_37*RLcp7_28;
    ORcp7_28 = -(OMcp7_17*RLcp7_38-OMcp7_37*RLcp7_18);
    ORcp7_38 = OMcp7_17*RLcp7_28-OMcp7_27*RLcp7_18;
    OPcp7_18 = OPcp7_17+ROcp7_17*qdd[8]+qd[8]*(OMcp7_27*ROcp7_37-OMcp7_37*ROcp7_27);
    OPcp7_28 = OPcp7_27+ROcp7_27*qdd[8]-qd[8]*(OMcp7_17*ROcp7_37-OMcp7_37*ROcp7_17);
    OPcp7_38 = OPcp7_37+ROcp7_37*qdd[8]+qd[8]*(OMcp7_17*ROcp7_27-OMcp7_27*ROcp7_17);
    RLcp7_143 = s->dpt[1][8]*ROcp7_17+s->dpt[2][8]*ROcp7_48+ROcp7_78*s->dpt[3][8];
    RLcp7_243 = s->dpt[1][8]*ROcp7_27+s->dpt[2][8]*ROcp7_58+ROcp7_88*s->dpt[3][8];
    RLcp7_343 = s->dpt[1][8]*ROcp7_37+s->dpt[2][8]*ROcp7_68+ROcp7_98*s->dpt[3][8];
    POcp7_143 = RLcp7_143+RLcp7_17+RLcp7_18+q[1];
    POcp7_243 = RLcp7_243+RLcp7_27+RLcp7_28+q[2];
    POcp7_343 = RLcp7_343+RLcp7_37+RLcp7_38+q[3];
    JTcp7_243_4 = -(RLcp7_343+RLcp7_37+RLcp7_38);
    JTcp7_343_4 = RLcp7_243+RLcp7_27+RLcp7_28;
    JTcp7_143_5 = C4*(RLcp7_37+RLcp7_38)-S4*(RLcp7_27+RLcp7_28)-RLcp7_243*S4+RLcp7_343*C4;
    JTcp7_243_5 = S4*(RLcp7_143+RLcp7_17+RLcp7_18);
    JTcp7_343_5 = -C4*(RLcp7_143+RLcp7_17+RLcp7_18);
    JTcp7_143_6 = ROcp7_85*(RLcp7_37+RLcp7_38)-ROcp7_95*(RLcp7_27+RLcp7_28)-RLcp7_243*ROcp7_95+RLcp7_343*ROcp7_85;
    JTcp7_243_6 = -(RLcp7_343*S5-ROcp7_95*(RLcp7_143+RLcp7_17+RLcp7_18)+S5*(RLcp7_37+RLcp7_38));
    JTcp7_343_6 = RLcp7_243*S5-ROcp7_85*(RLcp7_143+RLcp7_17+RLcp7_18)+S5*(RLcp7_27+RLcp7_28);
    JTcp7_143_7 = ROcp7_56*(RLcp7_343+RLcp7_38)-ROcp7_66*(RLcp7_243+RLcp7_28);
    JTcp7_243_7 = -(ROcp7_46*(RLcp7_343+RLcp7_38)-ROcp7_66*(RLcp7_143+RLcp7_18));
    JTcp7_343_7 = ROcp7_46*(RLcp7_243+RLcp7_28)-ROcp7_56*(RLcp7_143+RLcp7_18);
    JTcp7_143_8 = -(RLcp7_243*ROcp7_37-RLcp7_343*ROcp7_27);
    JTcp7_243_8 = RLcp7_143*ROcp7_37-RLcp7_343*ROcp7_17;
    JTcp7_343_8 = -(RLcp7_143*ROcp7_27-RLcp7_243*ROcp7_17);
    ORcp7_143 = OMcp7_28*RLcp7_343-OMcp7_38*RLcp7_243;
    ORcp7_243 = -(OMcp7_18*RLcp7_343-OMcp7_38*RLcp7_143);
    ORcp7_343 = OMcp7_18*RLcp7_243-OMcp7_28*RLcp7_143;
    VIcp7_143 = ORcp7_143+ORcp7_17+ORcp7_18+qd[1];
    VIcp7_243 = ORcp7_243+ORcp7_27+ORcp7_28+qd[2];
    VIcp7_343 = ORcp7_343+ORcp7_37+ORcp7_38+qd[3];
    ACcp7_143 = qdd[1]+OMcp7_26*ORcp7_37+OMcp7_27*ORcp7_38+OMcp7_28*ORcp7_343-OMcp7_36*ORcp7_27-OMcp7_37*ORcp7_28-OMcp7_38
 *ORcp7_243+OPcp7_26*RLcp7_37+OPcp7_27*RLcp7_38+OPcp7_28*RLcp7_343-OPcp7_36*RLcp7_27-OPcp7_37*RLcp7_28-OPcp7_38*RLcp7_243;
    ACcp7_243 = qdd[2]-OMcp7_16*ORcp7_37-OMcp7_17*ORcp7_38-OMcp7_18*ORcp7_343+OMcp7_36*ORcp7_17+OMcp7_37*ORcp7_18+OMcp7_38
 *ORcp7_143-OPcp7_16*RLcp7_37-OPcp7_17*RLcp7_38-OPcp7_18*RLcp7_343+OPcp7_36*RLcp7_17+OPcp7_37*RLcp7_18+OPcp7_38*RLcp7_143;
    ACcp7_343 = qdd[3]+OMcp7_16*ORcp7_27+OMcp7_17*ORcp7_28+OMcp7_18*ORcp7_243-OMcp7_26*ORcp7_17-OMcp7_27*ORcp7_18-OMcp7_28
 *ORcp7_143+OPcp7_16*RLcp7_27+OPcp7_17*RLcp7_28+OPcp7_18*RLcp7_243-OPcp7_26*RLcp7_17-OPcp7_27*RLcp7_18-OPcp7_28*RLcp7_143;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_143;
    sens->P[2] = POcp7_243;
    sens->P[3] = POcp7_343;
    sens->R[1][1] = ROcp7_17;
    sens->R[1][2] = ROcp7_27;
    sens->R[1][3] = ROcp7_37;
    sens->R[2][1] = ROcp7_48;
    sens->R[2][2] = ROcp7_58;
    sens->R[2][3] = ROcp7_68;
    sens->R[3][1] = ROcp7_78;
    sens->R[3][2] = ROcp7_88;
    sens->R[3][3] = ROcp7_98;
    sens->V[1] = VIcp7_143;
    sens->V[2] = VIcp7_243;
    sens->V[3] = VIcp7_343;
    sens->OM[1] = OMcp7_18;
    sens->OM[2] = OMcp7_28;
    sens->OM[3] = OMcp7_38;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp7_143_5;
    sens->J[1][6] = JTcp7_143_6;
    sens->J[1][7] = JTcp7_143_7;
    sens->J[1][8] = JTcp7_143_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp7_243_4;
    sens->J[2][5] = JTcp7_243_5;
    sens->J[2][6] = JTcp7_243_6;
    sens->J[2][7] = JTcp7_243_7;
    sens->J[2][8] = JTcp7_243_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp7_343_4;
    sens->J[3][5] = JTcp7_343_5;
    sens->J[3][6] = JTcp7_343_6;
    sens->J[3][7] = JTcp7_343_7;
    sens->J[3][8] = JTcp7_343_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp7_46;
    sens->J[4][8] = ROcp7_17;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp7_85;
    sens->J[5][7] = ROcp7_56;
    sens->J[5][8] = ROcp7_27;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp7_95;
    sens->J[6][7] = ROcp7_66;
    sens->J[6][8] = ROcp7_37;
    sens->A[1] = ACcp7_143;
    sens->A[2] = ACcp7_243;
    sens->A[3] = ACcp7_343;
    sens->OMP[1] = OPcp7_18;
    sens->OMP[2] = OPcp7_28;
    sens->OMP[3] = OPcp7_38;
 
// 
break;
case 9:
 


// = = Block_1_0_0_9_0_1 = = 
 
// Sensor Kinematics 


    ROcp8_25 = S4*S5;
    ROcp8_35 = -C4*S5;
    ROcp8_85 = -S4*C5;
    ROcp8_95 = C4*C5;
    ROcp8_16 = C5*C6;
    ROcp8_26 = ROcp8_25*C6+C4*S6;
    ROcp8_36 = ROcp8_35*C6+S4*S6;
    ROcp8_46 = -C5*S6;
    ROcp8_56 = -(ROcp8_25*S6-C4*C6);
    ROcp8_66 = -(ROcp8_35*S6-S4*C6);
    OMcp8_25 = qd[5]*C4;
    OMcp8_35 = qd[5]*S4;
    OMcp8_16 = qd[4]+qd[6]*S5;
    OMcp8_26 = OMcp8_25+ROcp8_85*qd[6];
    OMcp8_36 = OMcp8_35+ROcp8_95*qd[6];
    OPcp8_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp8_26 = ROcp8_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp8_35*S5-ROcp8_95*qd[4]);
    OPcp8_36 = ROcp8_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp8_25*S5-ROcp8_85*qd[4]);

// = = Block_1_0_0_9_0_2 = = 
 
// Sensor Kinematics 


    ROcp8_17 = ROcp8_16*C7-S5*S7;
    ROcp8_27 = ROcp8_26*C7-ROcp8_85*S7;
    ROcp8_37 = ROcp8_36*C7-ROcp8_95*S7;
    ROcp8_77 = ROcp8_16*S7+S5*C7;
    ROcp8_87 = ROcp8_26*S7+ROcp8_85*C7;
    ROcp8_97 = ROcp8_36*S7+ROcp8_95*C7;
    ROcp8_48 = ROcp8_46*C8+ROcp8_77*S8;
    ROcp8_58 = ROcp8_56*C8+ROcp8_87*S8;
    ROcp8_68 = ROcp8_66*C8+ROcp8_97*S8;
    ROcp8_78 = -(ROcp8_46*S8-ROcp8_77*C8);
    ROcp8_88 = -(ROcp8_56*S8-ROcp8_87*C8);
    ROcp8_98 = -(ROcp8_66*S8-ROcp8_97*C8);
    RLcp8_17 = s->dpt[1][1]*ROcp8_16+s->dpt[3][1]*S5+ROcp8_46*s->dpt[2][1];
    RLcp8_27 = s->dpt[1][1]*ROcp8_26+s->dpt[3][1]*ROcp8_85+ROcp8_56*s->dpt[2][1];
    RLcp8_37 = s->dpt[1][1]*ROcp8_36+s->dpt[3][1]*ROcp8_95+ROcp8_66*s->dpt[2][1];
    OMcp8_17 = OMcp8_16+ROcp8_46*qd[7];
    OMcp8_27 = OMcp8_26+ROcp8_56*qd[7];
    OMcp8_37 = OMcp8_36+ROcp8_66*qd[7];
    ORcp8_17 = OMcp8_26*RLcp8_37-OMcp8_36*RLcp8_27;
    ORcp8_27 = -(OMcp8_16*RLcp8_37-OMcp8_36*RLcp8_17);
    ORcp8_37 = OMcp8_16*RLcp8_27-OMcp8_26*RLcp8_17;
    OPcp8_17 = OPcp8_16+ROcp8_46*qdd[7]+qd[7]*(OMcp8_26*ROcp8_66-OMcp8_36*ROcp8_56);
    OPcp8_27 = OPcp8_26+ROcp8_56*qdd[7]-qd[7]*(OMcp8_16*ROcp8_66-OMcp8_36*ROcp8_46);
    OPcp8_37 = OPcp8_36+ROcp8_66*qdd[7]+qd[7]*(OMcp8_16*ROcp8_56-OMcp8_26*ROcp8_46);
    RLcp8_18 = s->dpt[1][6]*ROcp8_17+s->dpt[3][6]*ROcp8_77+ROcp8_46*s->dpt[2][6];
    RLcp8_28 = s->dpt[1][6]*ROcp8_27+s->dpt[3][6]*ROcp8_87+ROcp8_56*s->dpt[2][6];
    RLcp8_38 = s->dpt[1][6]*ROcp8_37+s->dpt[3][6]*ROcp8_97+ROcp8_66*s->dpt[2][6];
    OMcp8_18 = OMcp8_17+ROcp8_17*qd[8];
    OMcp8_28 = OMcp8_27+ROcp8_27*qd[8];
    OMcp8_38 = OMcp8_37+ROcp8_37*qd[8];
    ORcp8_18 = OMcp8_27*RLcp8_38-OMcp8_37*RLcp8_28;
    ORcp8_28 = -(OMcp8_17*RLcp8_38-OMcp8_37*RLcp8_18);
    ORcp8_38 = OMcp8_17*RLcp8_28-OMcp8_27*RLcp8_18;
    OPcp8_18 = OPcp8_17+ROcp8_17*qdd[8]+qd[8]*(OMcp8_27*ROcp8_37-OMcp8_37*ROcp8_27);
    OPcp8_28 = OPcp8_27+ROcp8_27*qdd[8]-qd[8]*(OMcp8_17*ROcp8_37-OMcp8_37*ROcp8_17);
    OPcp8_38 = OPcp8_37+ROcp8_37*qdd[8]+qd[8]*(OMcp8_17*ROcp8_27-OMcp8_27*ROcp8_17);
    RLcp8_144 = ROcp8_17*s->dpt[1][9]+ROcp8_48*s->dpt[2][9]+ROcp8_78*s->dpt[3][9];
    RLcp8_244 = ROcp8_27*s->dpt[1][9]+ROcp8_58*s->dpt[2][9]+ROcp8_88*s->dpt[3][9];
    RLcp8_344 = ROcp8_37*s->dpt[1][9]+ROcp8_68*s->dpt[2][9]+ROcp8_98*s->dpt[3][9];
    POcp8_144 = RLcp8_144+RLcp8_17+RLcp8_18+q[1];
    POcp8_244 = RLcp8_244+RLcp8_27+RLcp8_28+q[2];
    POcp8_344 = RLcp8_344+RLcp8_37+RLcp8_38+q[3];
    JTcp8_244_4 = -(RLcp8_344+RLcp8_37+RLcp8_38);
    JTcp8_344_4 = RLcp8_244+RLcp8_27+RLcp8_28;
    JTcp8_144_5 = C4*(RLcp8_37+RLcp8_38)-S4*(RLcp8_27+RLcp8_28)-RLcp8_244*S4+RLcp8_344*C4;
    JTcp8_244_5 = S4*(RLcp8_144+RLcp8_17+RLcp8_18);
    JTcp8_344_5 = -C4*(RLcp8_144+RLcp8_17+RLcp8_18);
    JTcp8_144_6 = ROcp8_85*(RLcp8_37+RLcp8_38)-ROcp8_95*(RLcp8_27+RLcp8_28)-RLcp8_244*ROcp8_95+RLcp8_344*ROcp8_85;
    JTcp8_244_6 = -(RLcp8_344*S5-ROcp8_95*(RLcp8_144+RLcp8_17+RLcp8_18)+S5*(RLcp8_37+RLcp8_38));
    JTcp8_344_6 = RLcp8_244*S5-ROcp8_85*(RLcp8_144+RLcp8_17+RLcp8_18)+S5*(RLcp8_27+RLcp8_28);
    JTcp8_144_7 = ROcp8_56*(RLcp8_344+RLcp8_38)-ROcp8_66*(RLcp8_244+RLcp8_28);
    JTcp8_244_7 = -(ROcp8_46*(RLcp8_344+RLcp8_38)-ROcp8_66*(RLcp8_144+RLcp8_18));
    JTcp8_344_7 = ROcp8_46*(RLcp8_244+RLcp8_28)-ROcp8_56*(RLcp8_144+RLcp8_18);
    JTcp8_144_8 = -(RLcp8_244*ROcp8_37-RLcp8_344*ROcp8_27);
    JTcp8_244_8 = RLcp8_144*ROcp8_37-RLcp8_344*ROcp8_17;
    JTcp8_344_8 = -(RLcp8_144*ROcp8_27-RLcp8_244*ROcp8_17);
    ORcp8_144 = OMcp8_28*RLcp8_344-OMcp8_38*RLcp8_244;
    ORcp8_244 = -(OMcp8_18*RLcp8_344-OMcp8_38*RLcp8_144);
    ORcp8_344 = OMcp8_18*RLcp8_244-OMcp8_28*RLcp8_144;
    VIcp8_144 = ORcp8_144+ORcp8_17+ORcp8_18+qd[1];
    VIcp8_244 = ORcp8_244+ORcp8_27+ORcp8_28+qd[2];
    VIcp8_344 = ORcp8_344+ORcp8_37+ORcp8_38+qd[3];
    ACcp8_144 = qdd[1]+OMcp8_26*ORcp8_37+OMcp8_27*ORcp8_38+OMcp8_28*ORcp8_344-OMcp8_36*ORcp8_27-OMcp8_37*ORcp8_28-OMcp8_38
 *ORcp8_244+OPcp8_26*RLcp8_37+OPcp8_27*RLcp8_38+OPcp8_28*RLcp8_344-OPcp8_36*RLcp8_27-OPcp8_37*RLcp8_28-OPcp8_38*RLcp8_244;
    ACcp8_244 = qdd[2]-OMcp8_16*ORcp8_37-OMcp8_17*ORcp8_38-OMcp8_18*ORcp8_344+OMcp8_36*ORcp8_17+OMcp8_37*ORcp8_18+OMcp8_38
 *ORcp8_144-OPcp8_16*RLcp8_37-OPcp8_17*RLcp8_38-OPcp8_18*RLcp8_344+OPcp8_36*RLcp8_17+OPcp8_37*RLcp8_18+OPcp8_38*RLcp8_144;
    ACcp8_344 = qdd[3]+OMcp8_16*ORcp8_27+OMcp8_17*ORcp8_28+OMcp8_18*ORcp8_244-OMcp8_26*ORcp8_17-OMcp8_27*ORcp8_18-OMcp8_28
 *ORcp8_144+OPcp8_16*RLcp8_27+OPcp8_17*RLcp8_28+OPcp8_18*RLcp8_244-OPcp8_26*RLcp8_17-OPcp8_27*RLcp8_18-OPcp8_28*RLcp8_144;

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_144;
    sens->P[2] = POcp8_244;
    sens->P[3] = POcp8_344;
    sens->R[1][1] = ROcp8_17;
    sens->R[1][2] = ROcp8_27;
    sens->R[1][3] = ROcp8_37;
    sens->R[2][1] = ROcp8_48;
    sens->R[2][2] = ROcp8_58;
    sens->R[2][3] = ROcp8_68;
    sens->R[3][1] = ROcp8_78;
    sens->R[3][2] = ROcp8_88;
    sens->R[3][3] = ROcp8_98;
    sens->V[1] = VIcp8_144;
    sens->V[2] = VIcp8_244;
    sens->V[3] = VIcp8_344;
    sens->OM[1] = OMcp8_18;
    sens->OM[2] = OMcp8_28;
    sens->OM[3] = OMcp8_38;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp8_144_5;
    sens->J[1][6] = JTcp8_144_6;
    sens->J[1][7] = JTcp8_144_7;
    sens->J[1][8] = JTcp8_144_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp8_244_4;
    sens->J[2][5] = JTcp8_244_5;
    sens->J[2][6] = JTcp8_244_6;
    sens->J[2][7] = JTcp8_244_7;
    sens->J[2][8] = JTcp8_244_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp8_344_4;
    sens->J[3][5] = JTcp8_344_5;
    sens->J[3][6] = JTcp8_344_6;
    sens->J[3][7] = JTcp8_344_7;
    sens->J[3][8] = JTcp8_344_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp8_46;
    sens->J[4][8] = ROcp8_17;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp8_85;
    sens->J[5][7] = ROcp8_56;
    sens->J[5][8] = ROcp8_27;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp8_95;
    sens->J[6][7] = ROcp8_66;
    sens->J[6][8] = ROcp8_37;
    sens->A[1] = ACcp8_144;
    sens->A[2] = ACcp8_244;
    sens->A[3] = ACcp8_344;
    sens->OMP[1] = OPcp8_18;
    sens->OMP[2] = OPcp8_28;
    sens->OMP[3] = OPcp8_38;
 
// 
break;
case 10:
 


// = = Block_1_0_0_10_0_1 = = 
 
// Sensor Kinematics 


    ROcp9_25 = S4*S5;
    ROcp9_35 = -C4*S5;
    ROcp9_85 = -S4*C5;
    ROcp9_95 = C4*C5;
    ROcp9_16 = C5*C6;
    ROcp9_26 = ROcp9_25*C6+C4*S6;
    ROcp9_36 = ROcp9_35*C6+S4*S6;
    ROcp9_46 = -C5*S6;
    ROcp9_56 = -(ROcp9_25*S6-C4*C6);
    ROcp9_66 = -(ROcp9_35*S6-S4*C6);
    OMcp9_25 = qd[5]*C4;
    OMcp9_35 = qd[5]*S4;
    OMcp9_16 = qd[4]+qd[6]*S5;
    OMcp9_26 = OMcp9_25+ROcp9_85*qd[6];
    OMcp9_36 = OMcp9_35+ROcp9_95*qd[6];
    OPcp9_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp9_26 = ROcp9_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp9_35*S5-ROcp9_95*qd[4]);
    OPcp9_36 = ROcp9_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp9_25*S5-ROcp9_85*qd[4]);

// = = Block_1_0_0_10_0_2 = = 
 
// Sensor Kinematics 


    ROcp9_17 = ROcp9_16*C7-S5*S7;
    ROcp9_27 = ROcp9_26*C7-ROcp9_85*S7;
    ROcp9_37 = ROcp9_36*C7-ROcp9_95*S7;
    ROcp9_77 = ROcp9_16*S7+S5*C7;
    ROcp9_87 = ROcp9_26*S7+ROcp9_85*C7;
    ROcp9_97 = ROcp9_36*S7+ROcp9_95*C7;
    ROcp9_48 = ROcp9_46*C8+ROcp9_77*S8;
    ROcp9_58 = ROcp9_56*C8+ROcp9_87*S8;
    ROcp9_68 = ROcp9_66*C8+ROcp9_97*S8;
    ROcp9_78 = -(ROcp9_46*S8-ROcp9_77*C8);
    ROcp9_88 = -(ROcp9_56*S8-ROcp9_87*C8);
    ROcp9_98 = -(ROcp9_66*S8-ROcp9_97*C8);
    ROcp9_19 = ROcp9_17*C9+ROcp9_48*S9;
    ROcp9_29 = ROcp9_27*C9+ROcp9_58*S9;
    ROcp9_39 = ROcp9_37*C9+ROcp9_68*S9;
    ROcp9_49 = -(ROcp9_17*S9-ROcp9_48*C9);
    ROcp9_59 = -(ROcp9_27*S9-ROcp9_58*C9);
    ROcp9_69 = -(ROcp9_37*S9-ROcp9_68*C9);
    RLcp9_17 = s->dpt[1][1]*ROcp9_16+s->dpt[3][1]*S5+ROcp9_46*s->dpt[2][1];
    RLcp9_27 = s->dpt[1][1]*ROcp9_26+s->dpt[3][1]*ROcp9_85+ROcp9_56*s->dpt[2][1];
    RLcp9_37 = s->dpt[1][1]*ROcp9_36+s->dpt[3][1]*ROcp9_95+ROcp9_66*s->dpt[2][1];
    OMcp9_17 = OMcp9_16+ROcp9_46*qd[7];
    OMcp9_27 = OMcp9_26+ROcp9_56*qd[7];
    OMcp9_37 = OMcp9_36+ROcp9_66*qd[7];
    ORcp9_17 = OMcp9_26*RLcp9_37-OMcp9_36*RLcp9_27;
    ORcp9_27 = -(OMcp9_16*RLcp9_37-OMcp9_36*RLcp9_17);
    ORcp9_37 = OMcp9_16*RLcp9_27-OMcp9_26*RLcp9_17;
    OPcp9_17 = OPcp9_16+ROcp9_46*qdd[7]+qd[7]*(OMcp9_26*ROcp9_66-OMcp9_36*ROcp9_56);
    OPcp9_27 = OPcp9_26+ROcp9_56*qdd[7]-qd[7]*(OMcp9_16*ROcp9_66-OMcp9_36*ROcp9_46);
    OPcp9_37 = OPcp9_36+ROcp9_66*qdd[7]+qd[7]*(OMcp9_16*ROcp9_56-OMcp9_26*ROcp9_46);
    RLcp9_18 = s->dpt[1][6]*ROcp9_17+s->dpt[3][6]*ROcp9_77+ROcp9_46*s->dpt[2][6];
    RLcp9_28 = s->dpt[1][6]*ROcp9_27+s->dpt[3][6]*ROcp9_87+ROcp9_56*s->dpt[2][6];
    RLcp9_38 = s->dpt[1][6]*ROcp9_37+s->dpt[3][6]*ROcp9_97+ROcp9_66*s->dpt[2][6];
    OMcp9_18 = OMcp9_17+ROcp9_17*qd[8];
    OMcp9_28 = OMcp9_27+ROcp9_27*qd[8];
    OMcp9_38 = OMcp9_37+ROcp9_37*qd[8];
    ORcp9_18 = OMcp9_27*RLcp9_38-OMcp9_37*RLcp9_28;
    ORcp9_28 = -(OMcp9_17*RLcp9_38-OMcp9_37*RLcp9_18);
    ORcp9_38 = OMcp9_17*RLcp9_28-OMcp9_27*RLcp9_18;
    OPcp9_18 = OPcp9_17+ROcp9_17*qdd[8]+qd[8]*(OMcp9_27*ROcp9_37-OMcp9_37*ROcp9_27);
    OPcp9_28 = OPcp9_27+ROcp9_27*qdd[8]-qd[8]*(OMcp9_17*ROcp9_37-OMcp9_37*ROcp9_17);
    OPcp9_38 = OPcp9_37+ROcp9_37*qdd[8]+qd[8]*(OMcp9_17*ROcp9_27-OMcp9_27*ROcp9_17);
    RLcp9_19 = s->dpt[1][8]*ROcp9_17+s->dpt[2][8]*ROcp9_48+ROcp9_78*s->dpt[3][8];
    RLcp9_29 = s->dpt[1][8]*ROcp9_27+s->dpt[2][8]*ROcp9_58+ROcp9_88*s->dpt[3][8];
    RLcp9_39 = s->dpt[1][8]*ROcp9_37+s->dpt[2][8]*ROcp9_68+ROcp9_98*s->dpt[3][8];
    OMcp9_19 = OMcp9_18+ROcp9_78*qd[9];
    OMcp9_29 = OMcp9_28+ROcp9_88*qd[9];
    OMcp9_39 = OMcp9_38+ROcp9_98*qd[9];
    ORcp9_19 = OMcp9_28*RLcp9_39-OMcp9_38*RLcp9_29;
    ORcp9_29 = -(OMcp9_18*RLcp9_39-OMcp9_38*RLcp9_19);
    ORcp9_39 = OMcp9_18*RLcp9_29-OMcp9_28*RLcp9_19;
    OPcp9_19 = OPcp9_18+ROcp9_78*qdd[9]+qd[9]*(OMcp9_28*ROcp9_98-OMcp9_38*ROcp9_88);
    OPcp9_29 = OPcp9_28+ROcp9_88*qdd[9]-qd[9]*(OMcp9_18*ROcp9_98-OMcp9_38*ROcp9_78);
    OPcp9_39 = OPcp9_38+ROcp9_98*qdd[9]+qd[9]*(OMcp9_18*ROcp9_88-OMcp9_28*ROcp9_78);
    RLcp9_145 = s->dpt[1][10]*ROcp9_19+s->dpt[2][10]*ROcp9_49+ROcp9_78*s->dpt[3][10];
    RLcp9_245 = s->dpt[1][10]*ROcp9_29+s->dpt[2][10]*ROcp9_59+ROcp9_88*s->dpt[3][10];
    RLcp9_345 = s->dpt[1][10]*ROcp9_39+s->dpt[2][10]*ROcp9_69+ROcp9_98*s->dpt[3][10];
    POcp9_145 = RLcp9_145+RLcp9_17+RLcp9_18+RLcp9_19+q[1];
    POcp9_245 = RLcp9_245+RLcp9_27+RLcp9_28+RLcp9_29+q[2];
    POcp9_345 = RLcp9_345+RLcp9_37+RLcp9_38+RLcp9_39+q[3];
    JTcp9_245_4 = -(RLcp9_345+RLcp9_37+RLcp9_38+RLcp9_39);
    JTcp9_345_4 = RLcp9_245+RLcp9_27+RLcp9_28+RLcp9_29;
    JTcp9_145_5 = C4*(RLcp9_345+RLcp9_37+RLcp9_38+RLcp9_39)-S4*(RLcp9_245+RLcp9_29)-S4*(RLcp9_27+RLcp9_28);
    JTcp9_245_5 = S4*(RLcp9_145+RLcp9_17+RLcp9_18+RLcp9_19);
    JTcp9_345_5 = -C4*(RLcp9_145+RLcp9_17+RLcp9_18+RLcp9_19);
    JTcp9_145_6 = ROcp9_85*(RLcp9_345+RLcp9_37+RLcp9_38+RLcp9_39)-ROcp9_95*(RLcp9_245+RLcp9_29)-ROcp9_95*(RLcp9_27+
 RLcp9_28);
    JTcp9_245_6 = RLcp9_145*ROcp9_95-RLcp9_345*S5-RLcp9_39*S5+ROcp9_95*(RLcp9_17+RLcp9_18+RLcp9_19)-S5*(RLcp9_37+RLcp9_38);
    JTcp9_345_6 = RLcp9_29*S5-ROcp9_85*(RLcp9_17+RLcp9_18+RLcp9_19)+S5*(RLcp9_27+RLcp9_28)-RLcp9_145*ROcp9_85+RLcp9_245*S5;
    JTcp9_145_7 = ROcp9_56*(RLcp9_38+RLcp9_39)-ROcp9_66*(RLcp9_28+RLcp9_29)-RLcp9_245*ROcp9_66+RLcp9_345*ROcp9_56;
    JTcp9_245_7 = RLcp9_145*ROcp9_66-RLcp9_345*ROcp9_46-ROcp9_46*(RLcp9_38+RLcp9_39)+ROcp9_66*(RLcp9_18+RLcp9_19);
    JTcp9_345_7 = ROcp9_46*(RLcp9_28+RLcp9_29)-ROcp9_56*(RLcp9_18+RLcp9_19)-RLcp9_145*ROcp9_56+RLcp9_245*ROcp9_46;
    JTcp9_145_8 = ROcp9_27*(RLcp9_345+RLcp9_39)-ROcp9_37*(RLcp9_245+RLcp9_29);
    JTcp9_245_8 = -(ROcp9_17*(RLcp9_345+RLcp9_39)-ROcp9_37*(RLcp9_145+RLcp9_19));
    JTcp9_345_8 = ROcp9_17*(RLcp9_245+RLcp9_29)-ROcp9_27*(RLcp9_145+RLcp9_19);
    JTcp9_145_9 = -(RLcp9_245*ROcp9_98-RLcp9_345*ROcp9_88);
    JTcp9_245_9 = RLcp9_145*ROcp9_98-RLcp9_345*ROcp9_78;
    JTcp9_345_9 = -(RLcp9_145*ROcp9_88-RLcp9_245*ROcp9_78);
    ORcp9_145 = OMcp9_29*RLcp9_345-OMcp9_39*RLcp9_245;
    ORcp9_245 = -(OMcp9_19*RLcp9_345-OMcp9_39*RLcp9_145);
    ORcp9_345 = OMcp9_19*RLcp9_245-OMcp9_29*RLcp9_145;
    VIcp9_145 = ORcp9_145+ORcp9_17+ORcp9_18+ORcp9_19+qd[1];
    VIcp9_245 = ORcp9_245+ORcp9_27+ORcp9_28+ORcp9_29+qd[2];
    VIcp9_345 = ORcp9_345+ORcp9_37+ORcp9_38+ORcp9_39+qd[3];
    ACcp9_145 = qdd[1]+OMcp9_26*ORcp9_37+OMcp9_27*ORcp9_38+OMcp9_28*ORcp9_39+OMcp9_29*ORcp9_345-OMcp9_36*ORcp9_27-OMcp9_37
 *ORcp9_28-OMcp9_38*ORcp9_29-OMcp9_39*ORcp9_245+OPcp9_26*RLcp9_37+OPcp9_27*RLcp9_38+OPcp9_28*RLcp9_39+OPcp9_29*RLcp9_345-
 OPcp9_36*RLcp9_27-OPcp9_37*RLcp9_28-OPcp9_38*RLcp9_29-OPcp9_39*RLcp9_245;
    ACcp9_245 = qdd[2]-OMcp9_16*ORcp9_37-OMcp9_17*ORcp9_38-OMcp9_18*ORcp9_39-OMcp9_19*ORcp9_345+OMcp9_36*ORcp9_17+OMcp9_37
 *ORcp9_18+OMcp9_38*ORcp9_19+OMcp9_39*ORcp9_145-OPcp9_16*RLcp9_37-OPcp9_17*RLcp9_38-OPcp9_18*RLcp9_39-OPcp9_19*RLcp9_345+
 OPcp9_36*RLcp9_17+OPcp9_37*RLcp9_18+OPcp9_38*RLcp9_19+OPcp9_39*RLcp9_145;
    ACcp9_345 = qdd[3]+OMcp9_16*ORcp9_27+OMcp9_17*ORcp9_28+OMcp9_18*ORcp9_29+OMcp9_19*ORcp9_245-OMcp9_26*ORcp9_17-OMcp9_27
 *ORcp9_18-OMcp9_28*ORcp9_19-OMcp9_29*ORcp9_145+OPcp9_16*RLcp9_27+OPcp9_17*RLcp9_28+OPcp9_18*RLcp9_29+OPcp9_19*RLcp9_245-
 OPcp9_26*RLcp9_17-OPcp9_27*RLcp9_18-OPcp9_28*RLcp9_19-OPcp9_29*RLcp9_145;

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_145;
    sens->P[2] = POcp9_245;
    sens->P[3] = POcp9_345;
    sens->R[1][1] = ROcp9_19;
    sens->R[1][2] = ROcp9_29;
    sens->R[1][3] = ROcp9_39;
    sens->R[2][1] = ROcp9_49;
    sens->R[2][2] = ROcp9_59;
    sens->R[2][3] = ROcp9_69;
    sens->R[3][1] = ROcp9_78;
    sens->R[3][2] = ROcp9_88;
    sens->R[3][3] = ROcp9_98;
    sens->V[1] = VIcp9_145;
    sens->V[2] = VIcp9_245;
    sens->V[3] = VIcp9_345;
    sens->OM[1] = OMcp9_19;
    sens->OM[2] = OMcp9_29;
    sens->OM[3] = OMcp9_39;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp9_145_5;
    sens->J[1][6] = JTcp9_145_6;
    sens->J[1][7] = JTcp9_145_7;
    sens->J[1][8] = JTcp9_145_8;
    sens->J[1][9] = JTcp9_145_9;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp9_245_4;
    sens->J[2][5] = JTcp9_245_5;
    sens->J[2][6] = JTcp9_245_6;
    sens->J[2][7] = JTcp9_245_7;
    sens->J[2][8] = JTcp9_245_8;
    sens->J[2][9] = JTcp9_245_9;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp9_345_4;
    sens->J[3][5] = JTcp9_345_5;
    sens->J[3][6] = JTcp9_345_6;
    sens->J[3][7] = JTcp9_345_7;
    sens->J[3][8] = JTcp9_345_8;
    sens->J[3][9] = JTcp9_345_9;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp9_46;
    sens->J[4][8] = ROcp9_17;
    sens->J[4][9] = ROcp9_78;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp9_85;
    sens->J[5][7] = ROcp9_56;
    sens->J[5][8] = ROcp9_27;
    sens->J[5][9] = ROcp9_88;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp9_95;
    sens->J[6][7] = ROcp9_66;
    sens->J[6][8] = ROcp9_37;
    sens->J[6][9] = ROcp9_98;
    sens->A[1] = ACcp9_145;
    sens->A[2] = ACcp9_245;
    sens->A[3] = ACcp9_345;
    sens->OMP[1] = OPcp9_19;
    sens->OMP[2] = OPcp9_29;
    sens->OMP[3] = OPcp9_39;
 
// 
break;
case 11:
 


// = = Block_1_0_0_11_0_1 = = 
 
// Sensor Kinematics 


    ROcp10_25 = S4*S5;
    ROcp10_35 = -C4*S5;
    ROcp10_85 = -S4*C5;
    ROcp10_95 = C4*C5;
    ROcp10_16 = C5*C6;
    ROcp10_26 = ROcp10_25*C6+C4*S6;
    ROcp10_36 = ROcp10_35*C6+S4*S6;
    ROcp10_46 = -C5*S6;
    ROcp10_56 = -(ROcp10_25*S6-C4*C6);
    ROcp10_66 = -(ROcp10_35*S6-S4*C6);
    OMcp10_25 = qd[5]*C4;
    OMcp10_35 = qd[5]*S4;
    OMcp10_16 = qd[4]+qd[6]*S5;
    OMcp10_26 = OMcp10_25+ROcp10_85*qd[6];
    OMcp10_36 = OMcp10_35+ROcp10_95*qd[6];
    OPcp10_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp10_26 = ROcp10_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp10_35*S5-ROcp10_95*qd[4]);
    OPcp10_36 = ROcp10_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp10_25*S5-ROcp10_85*qd[4]);

// = = Block_1_0_0_11_0_2 = = 
 
// Sensor Kinematics 


    ROcp10_17 = ROcp10_16*C7-S5*S7;
    ROcp10_27 = ROcp10_26*C7-ROcp10_85*S7;
    ROcp10_37 = ROcp10_36*C7-ROcp10_95*S7;
    ROcp10_77 = ROcp10_16*S7+S5*C7;
    ROcp10_87 = ROcp10_26*S7+ROcp10_85*C7;
    ROcp10_97 = ROcp10_36*S7+ROcp10_95*C7;
    ROcp10_48 = ROcp10_46*C8+ROcp10_77*S8;
    ROcp10_58 = ROcp10_56*C8+ROcp10_87*S8;
    ROcp10_68 = ROcp10_66*C8+ROcp10_97*S8;
    ROcp10_78 = -(ROcp10_46*S8-ROcp10_77*C8);
    ROcp10_88 = -(ROcp10_56*S8-ROcp10_87*C8);
    ROcp10_98 = -(ROcp10_66*S8-ROcp10_97*C8);
    ROcp10_19 = ROcp10_17*C9+ROcp10_48*S9;
    ROcp10_29 = ROcp10_27*C9+ROcp10_58*S9;
    ROcp10_39 = ROcp10_37*C9+ROcp10_68*S9;
    ROcp10_49 = -(ROcp10_17*S9-ROcp10_48*C9);
    ROcp10_59 = -(ROcp10_27*S9-ROcp10_58*C9);
    ROcp10_69 = -(ROcp10_37*S9-ROcp10_68*C9);
    RLcp10_17 = s->dpt[1][1]*ROcp10_16+s->dpt[3][1]*S5+ROcp10_46*s->dpt[2][1];
    RLcp10_27 = s->dpt[1][1]*ROcp10_26+s->dpt[3][1]*ROcp10_85+ROcp10_56*s->dpt[2][1];
    RLcp10_37 = s->dpt[1][1]*ROcp10_36+s->dpt[3][1]*ROcp10_95+ROcp10_66*s->dpt[2][1];
    OMcp10_17 = OMcp10_16+ROcp10_46*qd[7];
    OMcp10_27 = OMcp10_26+ROcp10_56*qd[7];
    OMcp10_37 = OMcp10_36+ROcp10_66*qd[7];
    ORcp10_17 = OMcp10_26*RLcp10_37-OMcp10_36*RLcp10_27;
    ORcp10_27 = -(OMcp10_16*RLcp10_37-OMcp10_36*RLcp10_17);
    ORcp10_37 = OMcp10_16*RLcp10_27-OMcp10_26*RLcp10_17;
    OPcp10_17 = OPcp10_16+ROcp10_46*qdd[7]+qd[7]*(OMcp10_26*ROcp10_66-OMcp10_36*ROcp10_56);
    OPcp10_27 = OPcp10_26+ROcp10_56*qdd[7]-qd[7]*(OMcp10_16*ROcp10_66-OMcp10_36*ROcp10_46);
    OPcp10_37 = OPcp10_36+ROcp10_66*qdd[7]+qd[7]*(OMcp10_16*ROcp10_56-OMcp10_26*ROcp10_46);
    RLcp10_18 = s->dpt[1][6]*ROcp10_17+s->dpt[3][6]*ROcp10_77+ROcp10_46*s->dpt[2][6];
    RLcp10_28 = s->dpt[1][6]*ROcp10_27+s->dpt[3][6]*ROcp10_87+ROcp10_56*s->dpt[2][6];
    RLcp10_38 = s->dpt[1][6]*ROcp10_37+s->dpt[3][6]*ROcp10_97+ROcp10_66*s->dpt[2][6];
    OMcp10_18 = OMcp10_17+ROcp10_17*qd[8];
    OMcp10_28 = OMcp10_27+ROcp10_27*qd[8];
    OMcp10_38 = OMcp10_37+ROcp10_37*qd[8];
    ORcp10_18 = OMcp10_27*RLcp10_38-OMcp10_37*RLcp10_28;
    ORcp10_28 = -(OMcp10_17*RLcp10_38-OMcp10_37*RLcp10_18);
    ORcp10_38 = OMcp10_17*RLcp10_28-OMcp10_27*RLcp10_18;
    OPcp10_18 = OPcp10_17+ROcp10_17*qdd[8]+qd[8]*(OMcp10_27*ROcp10_37-OMcp10_37*ROcp10_27);
    OPcp10_28 = OPcp10_27+ROcp10_27*qdd[8]-qd[8]*(OMcp10_17*ROcp10_37-OMcp10_37*ROcp10_17);
    OPcp10_38 = OPcp10_37+ROcp10_37*qdd[8]+qd[8]*(OMcp10_17*ROcp10_27-OMcp10_27*ROcp10_17);
    RLcp10_19 = s->dpt[1][8]*ROcp10_17+s->dpt[2][8]*ROcp10_48+ROcp10_78*s->dpt[3][8];
    RLcp10_29 = s->dpt[1][8]*ROcp10_27+s->dpt[2][8]*ROcp10_58+ROcp10_88*s->dpt[3][8];
    RLcp10_39 = s->dpt[1][8]*ROcp10_37+s->dpt[2][8]*ROcp10_68+ROcp10_98*s->dpt[3][8];
    OMcp10_19 = OMcp10_18+ROcp10_78*qd[9];
    OMcp10_29 = OMcp10_28+ROcp10_88*qd[9];
    OMcp10_39 = OMcp10_38+ROcp10_98*qd[9];
    ORcp10_19 = OMcp10_28*RLcp10_39-OMcp10_38*RLcp10_29;
    ORcp10_29 = -(OMcp10_18*RLcp10_39-OMcp10_38*RLcp10_19);
    ORcp10_39 = OMcp10_18*RLcp10_29-OMcp10_28*RLcp10_19;
    OPcp10_19 = OPcp10_18+ROcp10_78*qdd[9]+qd[9]*(OMcp10_28*ROcp10_98-OMcp10_38*ROcp10_88);
    OPcp10_29 = OPcp10_28+ROcp10_88*qdd[9]-qd[9]*(OMcp10_18*ROcp10_98-OMcp10_38*ROcp10_78);
    OPcp10_39 = OPcp10_38+ROcp10_98*qdd[9]+qd[9]*(OMcp10_18*ROcp10_88-OMcp10_28*ROcp10_78);
    RLcp10_146 = ROcp10_19*s->dpt[1][11]+ROcp10_49*s->dpt[2][11]+ROcp10_78*s->dpt[3][11];
    RLcp10_246 = ROcp10_29*s->dpt[1][11]+ROcp10_59*s->dpt[2][11]+ROcp10_88*s->dpt[3][11];
    RLcp10_346 = ROcp10_39*s->dpt[1][11]+ROcp10_69*s->dpt[2][11]+ROcp10_98*s->dpt[3][11];
    POcp10_146 = RLcp10_146+RLcp10_17+RLcp10_18+RLcp10_19+q[1];
    POcp10_246 = RLcp10_246+RLcp10_27+RLcp10_28+RLcp10_29+q[2];
    POcp10_346 = RLcp10_346+RLcp10_37+RLcp10_38+RLcp10_39+q[3];
    JTcp10_246_4 = -(RLcp10_346+RLcp10_37+RLcp10_38+RLcp10_39);
    JTcp10_346_4 = RLcp10_246+RLcp10_27+RLcp10_28+RLcp10_29;
    JTcp10_146_5 = C4*(RLcp10_346+RLcp10_37+RLcp10_38+RLcp10_39)-S4*(RLcp10_246+RLcp10_29)-S4*(RLcp10_27+RLcp10_28);
    JTcp10_246_5 = S4*(RLcp10_146+RLcp10_17+RLcp10_18+RLcp10_19);
    JTcp10_346_5 = -C4*(RLcp10_146+RLcp10_17+RLcp10_18+RLcp10_19);
    JTcp10_146_6 = ROcp10_85*(RLcp10_346+RLcp10_37+RLcp10_38+RLcp10_39)-ROcp10_95*(RLcp10_246+RLcp10_29)-ROcp10_95*(
 RLcp10_27+RLcp10_28);
    JTcp10_246_6 = RLcp10_146*ROcp10_95-RLcp10_346*S5-RLcp10_39*S5+ROcp10_95*(RLcp10_17+RLcp10_18+RLcp10_19)-S5*(RLcp10_37
 +RLcp10_38);
    JTcp10_346_6 = RLcp10_29*S5-ROcp10_85*(RLcp10_17+RLcp10_18+RLcp10_19)+S5*(RLcp10_27+RLcp10_28)-RLcp10_146*ROcp10_85+
 RLcp10_246*S5;
    JTcp10_146_7 = ROcp10_56*(RLcp10_38+RLcp10_39)-ROcp10_66*(RLcp10_28+RLcp10_29)-RLcp10_246*ROcp10_66+RLcp10_346*
 ROcp10_56;
    JTcp10_246_7 = RLcp10_146*ROcp10_66-RLcp10_346*ROcp10_46-ROcp10_46*(RLcp10_38+RLcp10_39)+ROcp10_66*(RLcp10_18+
 RLcp10_19);
    JTcp10_346_7 = ROcp10_46*(RLcp10_28+RLcp10_29)-ROcp10_56*(RLcp10_18+RLcp10_19)-RLcp10_146*ROcp10_56+RLcp10_246*
 ROcp10_46;
    JTcp10_146_8 = ROcp10_27*(RLcp10_346+RLcp10_39)-ROcp10_37*(RLcp10_246+RLcp10_29);
    JTcp10_246_8 = -(ROcp10_17*(RLcp10_346+RLcp10_39)-ROcp10_37*(RLcp10_146+RLcp10_19));
    JTcp10_346_8 = ROcp10_17*(RLcp10_246+RLcp10_29)-ROcp10_27*(RLcp10_146+RLcp10_19);
    JTcp10_146_9 = -(RLcp10_246*ROcp10_98-RLcp10_346*ROcp10_88);
    JTcp10_246_9 = RLcp10_146*ROcp10_98-RLcp10_346*ROcp10_78;
    JTcp10_346_9 = -(RLcp10_146*ROcp10_88-RLcp10_246*ROcp10_78);
    ORcp10_146 = OMcp10_29*RLcp10_346-OMcp10_39*RLcp10_246;
    ORcp10_246 = -(OMcp10_19*RLcp10_346-OMcp10_39*RLcp10_146);
    ORcp10_346 = OMcp10_19*RLcp10_246-OMcp10_29*RLcp10_146;
    VIcp10_146 = ORcp10_146+ORcp10_17+ORcp10_18+ORcp10_19+qd[1];
    VIcp10_246 = ORcp10_246+ORcp10_27+ORcp10_28+ORcp10_29+qd[2];
    VIcp10_346 = ORcp10_346+ORcp10_37+ORcp10_38+ORcp10_39+qd[3];
    ACcp10_146 = qdd[1]+OMcp10_26*ORcp10_37+OMcp10_27*ORcp10_38+OMcp10_28*ORcp10_39+OMcp10_29*ORcp10_346-OMcp10_36*
 ORcp10_27-OMcp10_37*ORcp10_28-OMcp10_38*ORcp10_29-OMcp10_39*ORcp10_246+OPcp10_26*RLcp10_37+OPcp10_27*RLcp10_38+OPcp10_28*
 RLcp10_39+OPcp10_29*RLcp10_346-OPcp10_36*RLcp10_27-OPcp10_37*RLcp10_28-OPcp10_38*RLcp10_29-OPcp10_39*RLcp10_246;
    ACcp10_246 = qdd[2]-OMcp10_16*ORcp10_37-OMcp10_17*ORcp10_38-OMcp10_18*ORcp10_39-OMcp10_19*ORcp10_346+OMcp10_36*
 ORcp10_17+OMcp10_37*ORcp10_18+OMcp10_38*ORcp10_19+OMcp10_39*ORcp10_146-OPcp10_16*RLcp10_37-OPcp10_17*RLcp10_38-OPcp10_18*
 RLcp10_39-OPcp10_19*RLcp10_346+OPcp10_36*RLcp10_17+OPcp10_37*RLcp10_18+OPcp10_38*RLcp10_19+OPcp10_39*RLcp10_146;
    ACcp10_346 = qdd[3]+OMcp10_16*ORcp10_27+OMcp10_17*ORcp10_28+OMcp10_18*ORcp10_29+OMcp10_19*ORcp10_246-OMcp10_26*
 ORcp10_17-OMcp10_27*ORcp10_18-OMcp10_28*ORcp10_19-OMcp10_29*ORcp10_146+OPcp10_16*RLcp10_27+OPcp10_17*RLcp10_28+OPcp10_18*
 RLcp10_29+OPcp10_19*RLcp10_246-OPcp10_26*RLcp10_17-OPcp10_27*RLcp10_18-OPcp10_28*RLcp10_19-OPcp10_29*RLcp10_146;

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_146;
    sens->P[2] = POcp10_246;
    sens->P[3] = POcp10_346;
    sens->R[1][1] = ROcp10_19;
    sens->R[1][2] = ROcp10_29;
    sens->R[1][3] = ROcp10_39;
    sens->R[2][1] = ROcp10_49;
    sens->R[2][2] = ROcp10_59;
    sens->R[2][3] = ROcp10_69;
    sens->R[3][1] = ROcp10_78;
    sens->R[3][2] = ROcp10_88;
    sens->R[3][3] = ROcp10_98;
    sens->V[1] = VIcp10_146;
    sens->V[2] = VIcp10_246;
    sens->V[3] = VIcp10_346;
    sens->OM[1] = OMcp10_19;
    sens->OM[2] = OMcp10_29;
    sens->OM[3] = OMcp10_39;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp10_146_5;
    sens->J[1][6] = JTcp10_146_6;
    sens->J[1][7] = JTcp10_146_7;
    sens->J[1][8] = JTcp10_146_8;
    sens->J[1][9] = JTcp10_146_9;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp10_246_4;
    sens->J[2][5] = JTcp10_246_5;
    sens->J[2][6] = JTcp10_246_6;
    sens->J[2][7] = JTcp10_246_7;
    sens->J[2][8] = JTcp10_246_8;
    sens->J[2][9] = JTcp10_246_9;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp10_346_4;
    sens->J[3][5] = JTcp10_346_5;
    sens->J[3][6] = JTcp10_346_6;
    sens->J[3][7] = JTcp10_346_7;
    sens->J[3][8] = JTcp10_346_8;
    sens->J[3][9] = JTcp10_346_9;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp10_46;
    sens->J[4][8] = ROcp10_17;
    sens->J[4][9] = ROcp10_78;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp10_85;
    sens->J[5][7] = ROcp10_56;
    sens->J[5][8] = ROcp10_27;
    sens->J[5][9] = ROcp10_88;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp10_95;
    sens->J[6][7] = ROcp10_66;
    sens->J[6][8] = ROcp10_37;
    sens->J[6][9] = ROcp10_98;
    sens->A[1] = ACcp10_146;
    sens->A[2] = ACcp10_246;
    sens->A[3] = ACcp10_346;
    sens->OMP[1] = OPcp10_19;
    sens->OMP[2] = OPcp10_29;
    sens->OMP[3] = OPcp10_39;
 
// 
break;
case 12:
 


// = = Block_1_0_0_12_0_1 = = 
 
// Sensor Kinematics 


    ROcp11_25 = S4*S5;
    ROcp11_35 = -C4*S5;
    ROcp11_85 = -S4*C5;
    ROcp11_95 = C4*C5;
    ROcp11_16 = C5*C6;
    ROcp11_26 = ROcp11_25*C6+C4*S6;
    ROcp11_36 = ROcp11_35*C6+S4*S6;
    ROcp11_46 = -C5*S6;
    ROcp11_56 = -(ROcp11_25*S6-C4*C6);
    ROcp11_66 = -(ROcp11_35*S6-S4*C6);
    OMcp11_25 = qd[5]*C4;
    OMcp11_35 = qd[5]*S4;
    OMcp11_16 = qd[4]+qd[6]*S5;
    OMcp11_26 = OMcp11_25+ROcp11_85*qd[6];
    OMcp11_36 = OMcp11_35+ROcp11_95*qd[6];
    OPcp11_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp11_26 = ROcp11_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp11_35*S5-ROcp11_95*qd[4]);
    OPcp11_36 = ROcp11_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp11_25*S5-ROcp11_85*qd[4]);

// = = Block_1_0_0_12_0_2 = = 
 
// Sensor Kinematics 


    ROcp11_17 = ROcp11_16*C7-S5*S7;
    ROcp11_27 = ROcp11_26*C7-ROcp11_85*S7;
    ROcp11_37 = ROcp11_36*C7-ROcp11_95*S7;
    ROcp11_77 = ROcp11_16*S7+S5*C7;
    ROcp11_87 = ROcp11_26*S7+ROcp11_85*C7;
    ROcp11_97 = ROcp11_36*S7+ROcp11_95*C7;
    ROcp11_48 = ROcp11_46*C8+ROcp11_77*S8;
    ROcp11_58 = ROcp11_56*C8+ROcp11_87*S8;
    ROcp11_68 = ROcp11_66*C8+ROcp11_97*S8;
    ROcp11_78 = -(ROcp11_46*S8-ROcp11_77*C8);
    ROcp11_88 = -(ROcp11_56*S8-ROcp11_87*C8);
    ROcp11_98 = -(ROcp11_66*S8-ROcp11_97*C8);
    ROcp11_19 = ROcp11_17*C9+ROcp11_48*S9;
    ROcp11_29 = ROcp11_27*C9+ROcp11_58*S9;
    ROcp11_39 = ROcp11_37*C9+ROcp11_68*S9;
    ROcp11_49 = -(ROcp11_17*S9-ROcp11_48*C9);
    ROcp11_59 = -(ROcp11_27*S9-ROcp11_58*C9);
    ROcp11_69 = -(ROcp11_37*S9-ROcp11_68*C9);
    ROcp11_110 = ROcp11_19*C10-ROcp11_78*S10;
    ROcp11_210 = ROcp11_29*C10-ROcp11_88*S10;
    ROcp11_310 = ROcp11_39*C10-ROcp11_98*S10;
    ROcp11_710 = ROcp11_19*S10+ROcp11_78*C10;
    ROcp11_810 = ROcp11_29*S10+ROcp11_88*C10;
    ROcp11_910 = ROcp11_39*S10+ROcp11_98*C10;
    RLcp11_17 = s->dpt[1][1]*ROcp11_16+s->dpt[3][1]*S5+ROcp11_46*s->dpt[2][1];
    RLcp11_27 = s->dpt[1][1]*ROcp11_26+s->dpt[3][1]*ROcp11_85+ROcp11_56*s->dpt[2][1];
    RLcp11_37 = s->dpt[1][1]*ROcp11_36+s->dpt[3][1]*ROcp11_95+ROcp11_66*s->dpt[2][1];
    OMcp11_17 = OMcp11_16+ROcp11_46*qd[7];
    OMcp11_27 = OMcp11_26+ROcp11_56*qd[7];
    OMcp11_37 = OMcp11_36+ROcp11_66*qd[7];
    ORcp11_17 = OMcp11_26*RLcp11_37-OMcp11_36*RLcp11_27;
    ORcp11_27 = -(OMcp11_16*RLcp11_37-OMcp11_36*RLcp11_17);
    ORcp11_37 = OMcp11_16*RLcp11_27-OMcp11_26*RLcp11_17;
    OPcp11_17 = OPcp11_16+ROcp11_46*qdd[7]+qd[7]*(OMcp11_26*ROcp11_66-OMcp11_36*ROcp11_56);
    OPcp11_27 = OPcp11_26+ROcp11_56*qdd[7]-qd[7]*(OMcp11_16*ROcp11_66-OMcp11_36*ROcp11_46);
    OPcp11_37 = OPcp11_36+ROcp11_66*qdd[7]+qd[7]*(OMcp11_16*ROcp11_56-OMcp11_26*ROcp11_46);
    RLcp11_18 = s->dpt[1][6]*ROcp11_17+s->dpt[3][6]*ROcp11_77+ROcp11_46*s->dpt[2][6];
    RLcp11_28 = s->dpt[1][6]*ROcp11_27+s->dpt[3][6]*ROcp11_87+ROcp11_56*s->dpt[2][6];
    RLcp11_38 = s->dpt[1][6]*ROcp11_37+s->dpt[3][6]*ROcp11_97+ROcp11_66*s->dpt[2][6];
    OMcp11_18 = OMcp11_17+ROcp11_17*qd[8];
    OMcp11_28 = OMcp11_27+ROcp11_27*qd[8];
    OMcp11_38 = OMcp11_37+ROcp11_37*qd[8];
    ORcp11_18 = OMcp11_27*RLcp11_38-OMcp11_37*RLcp11_28;
    ORcp11_28 = -(OMcp11_17*RLcp11_38-OMcp11_37*RLcp11_18);
    ORcp11_38 = OMcp11_17*RLcp11_28-OMcp11_27*RLcp11_18;
    OPcp11_18 = OPcp11_17+ROcp11_17*qdd[8]+qd[8]*(OMcp11_27*ROcp11_37-OMcp11_37*ROcp11_27);
    OPcp11_28 = OPcp11_27+ROcp11_27*qdd[8]-qd[8]*(OMcp11_17*ROcp11_37-OMcp11_37*ROcp11_17);
    OPcp11_38 = OPcp11_37+ROcp11_37*qdd[8]+qd[8]*(OMcp11_17*ROcp11_27-OMcp11_27*ROcp11_17);
    RLcp11_19 = s->dpt[1][8]*ROcp11_17+s->dpt[2][8]*ROcp11_48+ROcp11_78*s->dpt[3][8];
    RLcp11_29 = s->dpt[1][8]*ROcp11_27+s->dpt[2][8]*ROcp11_58+ROcp11_88*s->dpt[3][8];
    RLcp11_39 = s->dpt[1][8]*ROcp11_37+s->dpt[2][8]*ROcp11_68+ROcp11_98*s->dpt[3][8];
    OMcp11_19 = OMcp11_18+ROcp11_78*qd[9];
    OMcp11_29 = OMcp11_28+ROcp11_88*qd[9];
    OMcp11_39 = OMcp11_38+ROcp11_98*qd[9];
    ORcp11_19 = OMcp11_28*RLcp11_39-OMcp11_38*RLcp11_29;
    ORcp11_29 = -(OMcp11_18*RLcp11_39-OMcp11_38*RLcp11_19);
    ORcp11_39 = OMcp11_18*RLcp11_29-OMcp11_28*RLcp11_19;
    OPcp11_19 = OPcp11_18+ROcp11_78*qdd[9]+qd[9]*(OMcp11_28*ROcp11_98-OMcp11_38*ROcp11_88);
    OPcp11_29 = OPcp11_28+ROcp11_88*qdd[9]-qd[9]*(OMcp11_18*ROcp11_98-OMcp11_38*ROcp11_78);
    OPcp11_39 = OPcp11_38+ROcp11_98*qdd[9]+qd[9]*(OMcp11_18*ROcp11_88-OMcp11_28*ROcp11_78);
    RLcp11_110 = s->dpt[1][10]*ROcp11_19+s->dpt[2][10]*ROcp11_49+ROcp11_78*s->dpt[3][10];
    RLcp11_210 = s->dpt[1][10]*ROcp11_29+s->dpt[2][10]*ROcp11_59+ROcp11_88*s->dpt[3][10];
    RLcp11_310 = s->dpt[1][10]*ROcp11_39+s->dpt[2][10]*ROcp11_69+ROcp11_98*s->dpt[3][10];
    OMcp11_110 = OMcp11_19+ROcp11_49*qd[10];
    OMcp11_210 = OMcp11_29+ROcp11_59*qd[10];
    OMcp11_310 = OMcp11_39+ROcp11_69*qd[10];
    ORcp11_110 = OMcp11_29*RLcp11_310-OMcp11_39*RLcp11_210;
    ORcp11_210 = -(OMcp11_19*RLcp11_310-OMcp11_39*RLcp11_110);
    ORcp11_310 = OMcp11_19*RLcp11_210-OMcp11_29*RLcp11_110;
    OPcp11_110 = OPcp11_19+ROcp11_49*qdd[10]+qd[10]*(OMcp11_29*ROcp11_69-OMcp11_39*ROcp11_59);
    OPcp11_210 = OPcp11_29+ROcp11_59*qdd[10]-qd[10]*(OMcp11_19*ROcp11_69-OMcp11_39*ROcp11_49);
    OPcp11_310 = OPcp11_39+ROcp11_69*qdd[10]+qd[10]*(OMcp11_19*ROcp11_59-OMcp11_29*ROcp11_49);
    RLcp11_147 = s->dpt[1][12]*ROcp11_110+s->dpt[2][12]*ROcp11_49+ROcp11_710*s->dpt[3][12];
    RLcp11_247 = s->dpt[1][12]*ROcp11_210+s->dpt[2][12]*ROcp11_59+ROcp11_810*s->dpt[3][12];
    RLcp11_347 = s->dpt[1][12]*ROcp11_310+s->dpt[2][12]*ROcp11_69+ROcp11_910*s->dpt[3][12];
    POcp11_147 = RLcp11_110+RLcp11_147+RLcp11_17+RLcp11_18+RLcp11_19+q[1];
    POcp11_247 = RLcp11_210+RLcp11_247+RLcp11_27+RLcp11_28+RLcp11_29+q[2];
    POcp11_347 = RLcp11_310+RLcp11_347+RLcp11_37+RLcp11_38+RLcp11_39+q[3];
    JTcp11_247_4 = -(RLcp11_310+RLcp11_347+RLcp11_37+RLcp11_38+RLcp11_39);
    JTcp11_347_4 = RLcp11_210+RLcp11_247+RLcp11_27+RLcp11_28+RLcp11_29;
    JTcp11_147_5 = C4*(RLcp11_310+RLcp11_37+RLcp11_38+RLcp11_39)-S4*(RLcp11_210+RLcp11_29)-S4*(RLcp11_27+RLcp11_28)-
 RLcp11_247*S4+RLcp11_347*C4;
    JTcp11_247_5 = S4*(RLcp11_110+RLcp11_147+RLcp11_17+RLcp11_18+RLcp11_19);
    JTcp11_347_5 = -C4*(RLcp11_110+RLcp11_147+RLcp11_17+RLcp11_18+RLcp11_19);
    JTcp11_147_6 = ROcp11_85*(RLcp11_310+RLcp11_37+RLcp11_38+RLcp11_39)-ROcp11_95*(RLcp11_210+RLcp11_29)-ROcp11_95*(
 RLcp11_27+RLcp11_28)-RLcp11_247*ROcp11_95+RLcp11_347*ROcp11_85;
    JTcp11_247_6 = -(RLcp11_347*S5-ROcp11_95*(RLcp11_110+RLcp11_147+RLcp11_17+RLcp11_18+RLcp11_19)+S5*(RLcp11_310+
 RLcp11_39)+S5*(RLcp11_37+RLcp11_38));
    JTcp11_347_6 = RLcp11_247*S5-ROcp11_85*(RLcp11_110+RLcp11_147+RLcp11_17+RLcp11_18+RLcp11_19)+S5*(RLcp11_210+RLcp11_29)
 +S5*(RLcp11_27+RLcp11_28);
    JTcp11_147_7 = ROcp11_56*(RLcp11_310+RLcp11_347+RLcp11_38+RLcp11_39)-ROcp11_66*(RLcp11_210+RLcp11_247)-ROcp11_66*(
 RLcp11_28+RLcp11_29);
    JTcp11_247_7 = -(ROcp11_46*(RLcp11_310+RLcp11_347+RLcp11_38+RLcp11_39)-ROcp11_66*(RLcp11_110+RLcp11_147)-ROcp11_66*(
 RLcp11_18+RLcp11_19));
    JTcp11_347_7 = ROcp11_46*(RLcp11_210+RLcp11_247+RLcp11_28+RLcp11_29)-ROcp11_56*(RLcp11_110+RLcp11_147)-ROcp11_56*(
 RLcp11_18+RLcp11_19);
    JTcp11_147_8 = ROcp11_27*(RLcp11_310+RLcp11_39)-ROcp11_37*(RLcp11_210+RLcp11_29)-RLcp11_247*ROcp11_37+RLcp11_347*
 ROcp11_27;
    JTcp11_247_8 = RLcp11_147*ROcp11_37-RLcp11_347*ROcp11_17-ROcp11_17*(RLcp11_310+RLcp11_39)+ROcp11_37*(RLcp11_110+
 RLcp11_19);
    JTcp11_347_8 = ROcp11_17*(RLcp11_210+RLcp11_29)-ROcp11_27*(RLcp11_110+RLcp11_19)-RLcp11_147*ROcp11_27+RLcp11_247*
 ROcp11_17;
    JTcp11_147_9 = ROcp11_88*(RLcp11_310+RLcp11_347)-ROcp11_98*(RLcp11_210+RLcp11_247);
    JTcp11_247_9 = -(ROcp11_78*(RLcp11_310+RLcp11_347)-ROcp11_98*(RLcp11_110+RLcp11_147));
    JTcp11_347_9 = ROcp11_78*(RLcp11_210+RLcp11_247)-ROcp11_88*(RLcp11_110+RLcp11_147);
    JTcp11_147_10 = -(RLcp11_247*ROcp11_69-RLcp11_347*ROcp11_59);
    JTcp11_247_10 = RLcp11_147*ROcp11_69-RLcp11_347*ROcp11_49;
    JTcp11_347_10 = -(RLcp11_147*ROcp11_59-RLcp11_247*ROcp11_49);
    ORcp11_147 = OMcp11_210*RLcp11_347-OMcp11_310*RLcp11_247;
    ORcp11_247 = -(OMcp11_110*RLcp11_347-OMcp11_310*RLcp11_147);
    ORcp11_347 = OMcp11_110*RLcp11_247-OMcp11_210*RLcp11_147;
    VIcp11_147 = ORcp11_110+ORcp11_147+ORcp11_17+ORcp11_18+ORcp11_19+qd[1];
    VIcp11_247 = ORcp11_210+ORcp11_247+ORcp11_27+ORcp11_28+ORcp11_29+qd[2];
    VIcp11_347 = ORcp11_310+ORcp11_347+ORcp11_37+ORcp11_38+ORcp11_39+qd[3];
    ACcp11_147 = qdd[1]+OMcp11_210*ORcp11_347+OMcp11_26*ORcp11_37+OMcp11_27*ORcp11_38+OMcp11_28*ORcp11_39+OMcp11_29*
 ORcp11_310-OMcp11_310*ORcp11_247-OMcp11_36*ORcp11_27-OMcp11_37*ORcp11_28-OMcp11_38*ORcp11_29-OMcp11_39*ORcp11_210+OPcp11_210
 *RLcp11_347+OPcp11_26*RLcp11_37+OPcp11_27*RLcp11_38+OPcp11_28*RLcp11_39+OPcp11_29*RLcp11_310-OPcp11_310*RLcp11_247-OPcp11_36
 *RLcp11_27-OPcp11_37*RLcp11_28-OPcp11_38*RLcp11_29-OPcp11_39*RLcp11_210;
    ACcp11_247 = qdd[2]-OMcp11_110*ORcp11_347-OMcp11_16*ORcp11_37-OMcp11_17*ORcp11_38-OMcp11_18*ORcp11_39-OMcp11_19*
 ORcp11_310+OMcp11_310*ORcp11_147+OMcp11_36*ORcp11_17+OMcp11_37*ORcp11_18+OMcp11_38*ORcp11_19+OMcp11_39*ORcp11_110-OPcp11_110
 *RLcp11_347-OPcp11_16*RLcp11_37-OPcp11_17*RLcp11_38-OPcp11_18*RLcp11_39-OPcp11_19*RLcp11_310+OPcp11_310*RLcp11_147+OPcp11_36
 *RLcp11_17+OPcp11_37*RLcp11_18+OPcp11_38*RLcp11_19+OPcp11_39*RLcp11_110;
    ACcp11_347 = qdd[3]+OMcp11_110*ORcp11_247+OMcp11_16*ORcp11_27+OMcp11_17*ORcp11_28+OMcp11_18*ORcp11_29+OMcp11_19*
 ORcp11_210-OMcp11_210*ORcp11_147-OMcp11_26*ORcp11_17-OMcp11_27*ORcp11_18-OMcp11_28*ORcp11_19-OMcp11_29*ORcp11_110+OPcp11_110
 *RLcp11_247+OPcp11_16*RLcp11_27+OPcp11_17*RLcp11_28+OPcp11_18*RLcp11_29+OPcp11_19*RLcp11_210-OPcp11_210*RLcp11_147-OPcp11_26
 *RLcp11_17-OPcp11_27*RLcp11_18-OPcp11_28*RLcp11_19-OPcp11_29*RLcp11_110;

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_147;
    sens->P[2] = POcp11_247;
    sens->P[3] = POcp11_347;
    sens->R[1][1] = ROcp11_110;
    sens->R[1][2] = ROcp11_210;
    sens->R[1][3] = ROcp11_310;
    sens->R[2][1] = ROcp11_49;
    sens->R[2][2] = ROcp11_59;
    sens->R[2][3] = ROcp11_69;
    sens->R[3][1] = ROcp11_710;
    sens->R[3][2] = ROcp11_810;
    sens->R[3][3] = ROcp11_910;
    sens->V[1] = VIcp11_147;
    sens->V[2] = VIcp11_247;
    sens->V[3] = VIcp11_347;
    sens->OM[1] = OMcp11_110;
    sens->OM[2] = OMcp11_210;
    sens->OM[3] = OMcp11_310;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp11_147_5;
    sens->J[1][6] = JTcp11_147_6;
    sens->J[1][7] = JTcp11_147_7;
    sens->J[1][8] = JTcp11_147_8;
    sens->J[1][9] = JTcp11_147_9;
    sens->J[1][10] = JTcp11_147_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp11_247_4;
    sens->J[2][5] = JTcp11_247_5;
    sens->J[2][6] = JTcp11_247_6;
    sens->J[2][7] = JTcp11_247_7;
    sens->J[2][8] = JTcp11_247_8;
    sens->J[2][9] = JTcp11_247_9;
    sens->J[2][10] = JTcp11_247_10;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp11_347_4;
    sens->J[3][5] = JTcp11_347_5;
    sens->J[3][6] = JTcp11_347_6;
    sens->J[3][7] = JTcp11_347_7;
    sens->J[3][8] = JTcp11_347_8;
    sens->J[3][9] = JTcp11_347_9;
    sens->J[3][10] = JTcp11_347_10;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp11_46;
    sens->J[4][8] = ROcp11_17;
    sens->J[4][9] = ROcp11_78;
    sens->J[4][10] = ROcp11_49;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp11_85;
    sens->J[5][7] = ROcp11_56;
    sens->J[5][8] = ROcp11_27;
    sens->J[5][9] = ROcp11_88;
    sens->J[5][10] = ROcp11_59;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp11_95;
    sens->J[6][7] = ROcp11_66;
    sens->J[6][8] = ROcp11_37;
    sens->J[6][9] = ROcp11_98;
    sens->J[6][10] = ROcp11_69;
    sens->A[1] = ACcp11_147;
    sens->A[2] = ACcp11_247;
    sens->A[3] = ACcp11_347;
    sens->OMP[1] = OPcp11_110;
    sens->OMP[2] = OPcp11_210;
    sens->OMP[3] = OPcp11_310;
 
// 
break;
case 13:
 


// = = Block_1_0_0_13_0_1 = = 
 
// Sensor Kinematics 


    ROcp12_25 = S4*S5;
    ROcp12_35 = -C4*S5;
    ROcp12_85 = -S4*C5;
    ROcp12_95 = C4*C5;
    ROcp12_16 = C5*C6;
    ROcp12_26 = ROcp12_25*C6+C4*S6;
    ROcp12_36 = ROcp12_35*C6+S4*S6;
    ROcp12_46 = -C5*S6;
    ROcp12_56 = -(ROcp12_25*S6-C4*C6);
    ROcp12_66 = -(ROcp12_35*S6-S4*C6);
    OMcp12_25 = qd[5]*C4;
    OMcp12_35 = qd[5]*S4;
    OMcp12_16 = qd[4]+qd[6]*S5;
    OMcp12_26 = OMcp12_25+ROcp12_85*qd[6];
    OMcp12_36 = OMcp12_35+ROcp12_95*qd[6];
    OPcp12_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp12_26 = ROcp12_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp12_35*S5-ROcp12_95*qd[4]);
    OPcp12_36 = ROcp12_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp12_25*S5-ROcp12_85*qd[4]);

// = = Block_1_0_0_13_0_2 = = 
 
// Sensor Kinematics 


    ROcp12_17 = ROcp12_16*C7-S5*S7;
    ROcp12_27 = ROcp12_26*C7-ROcp12_85*S7;
    ROcp12_37 = ROcp12_36*C7-ROcp12_95*S7;
    ROcp12_77 = ROcp12_16*S7+S5*C7;
    ROcp12_87 = ROcp12_26*S7+ROcp12_85*C7;
    ROcp12_97 = ROcp12_36*S7+ROcp12_95*C7;
    ROcp12_48 = ROcp12_46*C8+ROcp12_77*S8;
    ROcp12_58 = ROcp12_56*C8+ROcp12_87*S8;
    ROcp12_68 = ROcp12_66*C8+ROcp12_97*S8;
    ROcp12_78 = -(ROcp12_46*S8-ROcp12_77*C8);
    ROcp12_88 = -(ROcp12_56*S8-ROcp12_87*C8);
    ROcp12_98 = -(ROcp12_66*S8-ROcp12_97*C8);
    ROcp12_19 = ROcp12_17*C9+ROcp12_48*S9;
    ROcp12_29 = ROcp12_27*C9+ROcp12_58*S9;
    ROcp12_39 = ROcp12_37*C9+ROcp12_68*S9;
    ROcp12_49 = -(ROcp12_17*S9-ROcp12_48*C9);
    ROcp12_59 = -(ROcp12_27*S9-ROcp12_58*C9);
    ROcp12_69 = -(ROcp12_37*S9-ROcp12_68*C9);
    ROcp12_110 = ROcp12_19*C10-ROcp12_78*S10;
    ROcp12_210 = ROcp12_29*C10-ROcp12_88*S10;
    ROcp12_310 = ROcp12_39*C10-ROcp12_98*S10;
    ROcp12_710 = ROcp12_19*S10+ROcp12_78*C10;
    ROcp12_810 = ROcp12_29*S10+ROcp12_88*C10;
    ROcp12_910 = ROcp12_39*S10+ROcp12_98*C10;
    RLcp12_17 = s->dpt[1][1]*ROcp12_16+s->dpt[3][1]*S5+ROcp12_46*s->dpt[2][1];
    RLcp12_27 = s->dpt[1][1]*ROcp12_26+s->dpt[3][1]*ROcp12_85+ROcp12_56*s->dpt[2][1];
    RLcp12_37 = s->dpt[1][1]*ROcp12_36+s->dpt[3][1]*ROcp12_95+ROcp12_66*s->dpt[2][1];
    OMcp12_17 = OMcp12_16+ROcp12_46*qd[7];
    OMcp12_27 = OMcp12_26+ROcp12_56*qd[7];
    OMcp12_37 = OMcp12_36+ROcp12_66*qd[7];
    ORcp12_17 = OMcp12_26*RLcp12_37-OMcp12_36*RLcp12_27;
    ORcp12_27 = -(OMcp12_16*RLcp12_37-OMcp12_36*RLcp12_17);
    ORcp12_37 = OMcp12_16*RLcp12_27-OMcp12_26*RLcp12_17;
    OPcp12_17 = OPcp12_16+ROcp12_46*qdd[7]+qd[7]*(OMcp12_26*ROcp12_66-OMcp12_36*ROcp12_56);
    OPcp12_27 = OPcp12_26+ROcp12_56*qdd[7]-qd[7]*(OMcp12_16*ROcp12_66-OMcp12_36*ROcp12_46);
    OPcp12_37 = OPcp12_36+ROcp12_66*qdd[7]+qd[7]*(OMcp12_16*ROcp12_56-OMcp12_26*ROcp12_46);
    RLcp12_18 = s->dpt[1][6]*ROcp12_17+s->dpt[3][6]*ROcp12_77+ROcp12_46*s->dpt[2][6];
    RLcp12_28 = s->dpt[1][6]*ROcp12_27+s->dpt[3][6]*ROcp12_87+ROcp12_56*s->dpt[2][6];
    RLcp12_38 = s->dpt[1][6]*ROcp12_37+s->dpt[3][6]*ROcp12_97+ROcp12_66*s->dpt[2][6];
    OMcp12_18 = OMcp12_17+ROcp12_17*qd[8];
    OMcp12_28 = OMcp12_27+ROcp12_27*qd[8];
    OMcp12_38 = OMcp12_37+ROcp12_37*qd[8];
    ORcp12_18 = OMcp12_27*RLcp12_38-OMcp12_37*RLcp12_28;
    ORcp12_28 = -(OMcp12_17*RLcp12_38-OMcp12_37*RLcp12_18);
    ORcp12_38 = OMcp12_17*RLcp12_28-OMcp12_27*RLcp12_18;
    OPcp12_18 = OPcp12_17+ROcp12_17*qdd[8]+qd[8]*(OMcp12_27*ROcp12_37-OMcp12_37*ROcp12_27);
    OPcp12_28 = OPcp12_27+ROcp12_27*qdd[8]-qd[8]*(OMcp12_17*ROcp12_37-OMcp12_37*ROcp12_17);
    OPcp12_38 = OPcp12_37+ROcp12_37*qdd[8]+qd[8]*(OMcp12_17*ROcp12_27-OMcp12_27*ROcp12_17);
    RLcp12_19 = s->dpt[1][8]*ROcp12_17+s->dpt[2][8]*ROcp12_48+ROcp12_78*s->dpt[3][8];
    RLcp12_29 = s->dpt[1][8]*ROcp12_27+s->dpt[2][8]*ROcp12_58+ROcp12_88*s->dpt[3][8];
    RLcp12_39 = s->dpt[1][8]*ROcp12_37+s->dpt[2][8]*ROcp12_68+ROcp12_98*s->dpt[3][8];
    OMcp12_19 = OMcp12_18+ROcp12_78*qd[9];
    OMcp12_29 = OMcp12_28+ROcp12_88*qd[9];
    OMcp12_39 = OMcp12_38+ROcp12_98*qd[9];
    ORcp12_19 = OMcp12_28*RLcp12_39-OMcp12_38*RLcp12_29;
    ORcp12_29 = -(OMcp12_18*RLcp12_39-OMcp12_38*RLcp12_19);
    ORcp12_39 = OMcp12_18*RLcp12_29-OMcp12_28*RLcp12_19;
    OPcp12_19 = OPcp12_18+ROcp12_78*qdd[9]+qd[9]*(OMcp12_28*ROcp12_98-OMcp12_38*ROcp12_88);
    OPcp12_29 = OPcp12_28+ROcp12_88*qdd[9]-qd[9]*(OMcp12_18*ROcp12_98-OMcp12_38*ROcp12_78);
    OPcp12_39 = OPcp12_38+ROcp12_98*qdd[9]+qd[9]*(OMcp12_18*ROcp12_88-OMcp12_28*ROcp12_78);
    RLcp12_110 = s->dpt[1][10]*ROcp12_19+s->dpt[2][10]*ROcp12_49+ROcp12_78*s->dpt[3][10];
    RLcp12_210 = s->dpt[1][10]*ROcp12_29+s->dpt[2][10]*ROcp12_59+ROcp12_88*s->dpt[3][10];
    RLcp12_310 = s->dpt[1][10]*ROcp12_39+s->dpt[2][10]*ROcp12_69+ROcp12_98*s->dpt[3][10];
    OMcp12_110 = OMcp12_19+ROcp12_49*qd[10];
    OMcp12_210 = OMcp12_29+ROcp12_59*qd[10];
    OMcp12_310 = OMcp12_39+ROcp12_69*qd[10];
    ORcp12_110 = OMcp12_29*RLcp12_310-OMcp12_39*RLcp12_210;
    ORcp12_210 = -(OMcp12_19*RLcp12_310-OMcp12_39*RLcp12_110);
    ORcp12_310 = OMcp12_19*RLcp12_210-OMcp12_29*RLcp12_110;
    OPcp12_110 = OPcp12_19+ROcp12_49*qdd[10]+qd[10]*(OMcp12_29*ROcp12_69-OMcp12_39*ROcp12_59);
    OPcp12_210 = OPcp12_29+ROcp12_59*qdd[10]-qd[10]*(OMcp12_19*ROcp12_69-OMcp12_39*ROcp12_49);
    OPcp12_310 = OPcp12_39+ROcp12_69*qdd[10]+qd[10]*(OMcp12_19*ROcp12_59-OMcp12_29*ROcp12_49);
    RLcp12_148 = ROcp12_110*s->dpt[1][13]+ROcp12_49*s->dpt[2][13]+ROcp12_710*s->dpt[3][13];
    RLcp12_248 = ROcp12_210*s->dpt[1][13]+ROcp12_59*s->dpt[2][13]+ROcp12_810*s->dpt[3][13];
    RLcp12_348 = ROcp12_310*s->dpt[1][13]+ROcp12_69*s->dpt[2][13]+ROcp12_910*s->dpt[3][13];
    POcp12_148 = RLcp12_110+RLcp12_148+RLcp12_17+RLcp12_18+RLcp12_19+q[1];
    POcp12_248 = RLcp12_210+RLcp12_248+RLcp12_27+RLcp12_28+RLcp12_29+q[2];
    POcp12_348 = RLcp12_310+RLcp12_348+RLcp12_37+RLcp12_38+RLcp12_39+q[3];
    JTcp12_248_4 = -(RLcp12_310+RLcp12_348+RLcp12_37+RLcp12_38+RLcp12_39);
    JTcp12_348_4 = RLcp12_210+RLcp12_248+RLcp12_27+RLcp12_28+RLcp12_29;
    JTcp12_148_5 = C4*(RLcp12_310+RLcp12_37+RLcp12_38+RLcp12_39)-S4*(RLcp12_210+RLcp12_29)-S4*(RLcp12_27+RLcp12_28)-
 RLcp12_248*S4+RLcp12_348*C4;
    JTcp12_248_5 = S4*(RLcp12_110+RLcp12_148+RLcp12_17+RLcp12_18+RLcp12_19);
    JTcp12_348_5 = -C4*(RLcp12_110+RLcp12_148+RLcp12_17+RLcp12_18+RLcp12_19);
    JTcp12_148_6 = ROcp12_85*(RLcp12_310+RLcp12_37+RLcp12_38+RLcp12_39)-ROcp12_95*(RLcp12_210+RLcp12_29)-ROcp12_95*(
 RLcp12_27+RLcp12_28)-RLcp12_248*ROcp12_95+RLcp12_348*ROcp12_85;
    JTcp12_248_6 = -(RLcp12_348*S5-ROcp12_95*(RLcp12_110+RLcp12_148+RLcp12_17+RLcp12_18+RLcp12_19)+S5*(RLcp12_310+
 RLcp12_39)+S5*(RLcp12_37+RLcp12_38));
    JTcp12_348_6 = RLcp12_248*S5-ROcp12_85*(RLcp12_110+RLcp12_148+RLcp12_17+RLcp12_18+RLcp12_19)+S5*(RLcp12_210+RLcp12_29)
 +S5*(RLcp12_27+RLcp12_28);
    JTcp12_148_7 = ROcp12_56*(RLcp12_310+RLcp12_348+RLcp12_38+RLcp12_39)-ROcp12_66*(RLcp12_210+RLcp12_248)-ROcp12_66*(
 RLcp12_28+RLcp12_29);
    JTcp12_248_7 = -(ROcp12_46*(RLcp12_310+RLcp12_348+RLcp12_38+RLcp12_39)-ROcp12_66*(RLcp12_110+RLcp12_148)-ROcp12_66*(
 RLcp12_18+RLcp12_19));
    JTcp12_348_7 = ROcp12_46*(RLcp12_210+RLcp12_248+RLcp12_28+RLcp12_29)-ROcp12_56*(RLcp12_110+RLcp12_148)-ROcp12_56*(
 RLcp12_18+RLcp12_19);
    JTcp12_148_8 = ROcp12_27*(RLcp12_310+RLcp12_39)-ROcp12_37*(RLcp12_210+RLcp12_29)-RLcp12_248*ROcp12_37+RLcp12_348*
 ROcp12_27;
    JTcp12_248_8 = RLcp12_148*ROcp12_37-RLcp12_348*ROcp12_17-ROcp12_17*(RLcp12_310+RLcp12_39)+ROcp12_37*(RLcp12_110+
 RLcp12_19);
    JTcp12_348_8 = ROcp12_17*(RLcp12_210+RLcp12_29)-ROcp12_27*(RLcp12_110+RLcp12_19)-RLcp12_148*ROcp12_27+RLcp12_248*
 ROcp12_17;
    JTcp12_148_9 = ROcp12_88*(RLcp12_310+RLcp12_348)-ROcp12_98*(RLcp12_210+RLcp12_248);
    JTcp12_248_9 = -(ROcp12_78*(RLcp12_310+RLcp12_348)-ROcp12_98*(RLcp12_110+RLcp12_148));
    JTcp12_348_9 = ROcp12_78*(RLcp12_210+RLcp12_248)-ROcp12_88*(RLcp12_110+RLcp12_148);
    JTcp12_148_10 = -(RLcp12_248*ROcp12_69-RLcp12_348*ROcp12_59);
    JTcp12_248_10 = RLcp12_148*ROcp12_69-RLcp12_348*ROcp12_49;
    JTcp12_348_10 = -(RLcp12_148*ROcp12_59-RLcp12_248*ROcp12_49);
    ORcp12_148 = OMcp12_210*RLcp12_348-OMcp12_310*RLcp12_248;
    ORcp12_248 = -(OMcp12_110*RLcp12_348-OMcp12_310*RLcp12_148);
    ORcp12_348 = OMcp12_110*RLcp12_248-OMcp12_210*RLcp12_148;
    VIcp12_148 = ORcp12_110+ORcp12_148+ORcp12_17+ORcp12_18+ORcp12_19+qd[1];
    VIcp12_248 = ORcp12_210+ORcp12_248+ORcp12_27+ORcp12_28+ORcp12_29+qd[2];
    VIcp12_348 = ORcp12_310+ORcp12_348+ORcp12_37+ORcp12_38+ORcp12_39+qd[3];
    ACcp12_148 = qdd[1]+OMcp12_210*ORcp12_348+OMcp12_26*ORcp12_37+OMcp12_27*ORcp12_38+OMcp12_28*ORcp12_39+OMcp12_29*
 ORcp12_310-OMcp12_310*ORcp12_248-OMcp12_36*ORcp12_27-OMcp12_37*ORcp12_28-OMcp12_38*ORcp12_29-OMcp12_39*ORcp12_210+OPcp12_210
 *RLcp12_348+OPcp12_26*RLcp12_37+OPcp12_27*RLcp12_38+OPcp12_28*RLcp12_39+OPcp12_29*RLcp12_310-OPcp12_310*RLcp12_248-OPcp12_36
 *RLcp12_27-OPcp12_37*RLcp12_28-OPcp12_38*RLcp12_29-OPcp12_39*RLcp12_210;
    ACcp12_248 = qdd[2]-OMcp12_110*ORcp12_348-OMcp12_16*ORcp12_37-OMcp12_17*ORcp12_38-OMcp12_18*ORcp12_39-OMcp12_19*
 ORcp12_310+OMcp12_310*ORcp12_148+OMcp12_36*ORcp12_17+OMcp12_37*ORcp12_18+OMcp12_38*ORcp12_19+OMcp12_39*ORcp12_110-OPcp12_110
 *RLcp12_348-OPcp12_16*RLcp12_37-OPcp12_17*RLcp12_38-OPcp12_18*RLcp12_39-OPcp12_19*RLcp12_310+OPcp12_310*RLcp12_148+OPcp12_36
 *RLcp12_17+OPcp12_37*RLcp12_18+OPcp12_38*RLcp12_19+OPcp12_39*RLcp12_110;
    ACcp12_348 = qdd[3]+OMcp12_110*ORcp12_248+OMcp12_16*ORcp12_27+OMcp12_17*ORcp12_28+OMcp12_18*ORcp12_29+OMcp12_19*
 ORcp12_210-OMcp12_210*ORcp12_148-OMcp12_26*ORcp12_17-OMcp12_27*ORcp12_18-OMcp12_28*ORcp12_19-OMcp12_29*ORcp12_110+OPcp12_110
 *RLcp12_248+OPcp12_16*RLcp12_27+OPcp12_17*RLcp12_28+OPcp12_18*RLcp12_29+OPcp12_19*RLcp12_210-OPcp12_210*RLcp12_148-OPcp12_26
 *RLcp12_17-OPcp12_27*RLcp12_18-OPcp12_28*RLcp12_19-OPcp12_29*RLcp12_110;

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_148;
    sens->P[2] = POcp12_248;
    sens->P[3] = POcp12_348;
    sens->R[1][1] = ROcp12_110;
    sens->R[1][2] = ROcp12_210;
    sens->R[1][3] = ROcp12_310;
    sens->R[2][1] = ROcp12_49;
    sens->R[2][2] = ROcp12_59;
    sens->R[2][3] = ROcp12_69;
    sens->R[3][1] = ROcp12_710;
    sens->R[3][2] = ROcp12_810;
    sens->R[3][3] = ROcp12_910;
    sens->V[1] = VIcp12_148;
    sens->V[2] = VIcp12_248;
    sens->V[3] = VIcp12_348;
    sens->OM[1] = OMcp12_110;
    sens->OM[2] = OMcp12_210;
    sens->OM[3] = OMcp12_310;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp12_148_5;
    sens->J[1][6] = JTcp12_148_6;
    sens->J[1][7] = JTcp12_148_7;
    sens->J[1][8] = JTcp12_148_8;
    sens->J[1][9] = JTcp12_148_9;
    sens->J[1][10] = JTcp12_148_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp12_248_4;
    sens->J[2][5] = JTcp12_248_5;
    sens->J[2][6] = JTcp12_248_6;
    sens->J[2][7] = JTcp12_248_7;
    sens->J[2][8] = JTcp12_248_8;
    sens->J[2][9] = JTcp12_248_9;
    sens->J[2][10] = JTcp12_248_10;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp12_348_4;
    sens->J[3][5] = JTcp12_348_5;
    sens->J[3][6] = JTcp12_348_6;
    sens->J[3][7] = JTcp12_348_7;
    sens->J[3][8] = JTcp12_348_8;
    sens->J[3][9] = JTcp12_348_9;
    sens->J[3][10] = JTcp12_348_10;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp12_46;
    sens->J[4][8] = ROcp12_17;
    sens->J[4][9] = ROcp12_78;
    sens->J[4][10] = ROcp12_49;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp12_85;
    sens->J[5][7] = ROcp12_56;
    sens->J[5][8] = ROcp12_27;
    sens->J[5][9] = ROcp12_88;
    sens->J[5][10] = ROcp12_59;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp12_95;
    sens->J[6][7] = ROcp12_66;
    sens->J[6][8] = ROcp12_37;
    sens->J[6][9] = ROcp12_98;
    sens->J[6][10] = ROcp12_69;
    sens->A[1] = ACcp12_148;
    sens->A[2] = ACcp12_248;
    sens->A[3] = ACcp12_348;
    sens->OMP[1] = OPcp12_110;
    sens->OMP[2] = OPcp12_210;
    sens->OMP[3] = OPcp12_310;
 
// 
break;
case 14:
 


// = = Block_1_0_0_14_0_1 = = 
 
// Sensor Kinematics 


    ROcp13_25 = S4*S5;
    ROcp13_35 = -C4*S5;
    ROcp13_85 = -S4*C5;
    ROcp13_95 = C4*C5;
    ROcp13_16 = C5*C6;
    ROcp13_26 = ROcp13_25*C6+C4*S6;
    ROcp13_36 = ROcp13_35*C6+S4*S6;
    ROcp13_46 = -C5*S6;
    ROcp13_56 = -(ROcp13_25*S6-C4*C6);
    ROcp13_66 = -(ROcp13_35*S6-S4*C6);
    OMcp13_25 = qd[5]*C4;
    OMcp13_35 = qd[5]*S4;
    OMcp13_16 = qd[4]+qd[6]*S5;
    OMcp13_26 = OMcp13_25+ROcp13_85*qd[6];
    OMcp13_36 = OMcp13_35+ROcp13_95*qd[6];
    OPcp13_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp13_26 = ROcp13_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp13_35*S5-ROcp13_95*qd[4]);
    OPcp13_36 = ROcp13_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp13_25*S5-ROcp13_85*qd[4]);

// = = Block_1_0_0_14_0_2 = = 
 
// Sensor Kinematics 


    ROcp13_17 = ROcp13_16*C7-S5*S7;
    ROcp13_27 = ROcp13_26*C7-ROcp13_85*S7;
    ROcp13_37 = ROcp13_36*C7-ROcp13_95*S7;
    ROcp13_77 = ROcp13_16*S7+S5*C7;
    ROcp13_87 = ROcp13_26*S7+ROcp13_85*C7;
    ROcp13_97 = ROcp13_36*S7+ROcp13_95*C7;
    ROcp13_48 = ROcp13_46*C8+ROcp13_77*S8;
    ROcp13_58 = ROcp13_56*C8+ROcp13_87*S8;
    ROcp13_68 = ROcp13_66*C8+ROcp13_97*S8;
    ROcp13_78 = -(ROcp13_46*S8-ROcp13_77*C8);
    ROcp13_88 = -(ROcp13_56*S8-ROcp13_87*C8);
    ROcp13_98 = -(ROcp13_66*S8-ROcp13_97*C8);
    ROcp13_19 = ROcp13_17*C9+ROcp13_48*S9;
    ROcp13_29 = ROcp13_27*C9+ROcp13_58*S9;
    ROcp13_39 = ROcp13_37*C9+ROcp13_68*S9;
    ROcp13_49 = -(ROcp13_17*S9-ROcp13_48*C9);
    ROcp13_59 = -(ROcp13_27*S9-ROcp13_58*C9);
    ROcp13_69 = -(ROcp13_37*S9-ROcp13_68*C9);
    ROcp13_110 = ROcp13_19*C10-ROcp13_78*S10;
    ROcp13_210 = ROcp13_29*C10-ROcp13_88*S10;
    ROcp13_310 = ROcp13_39*C10-ROcp13_98*S10;
    ROcp13_710 = ROcp13_19*S10+ROcp13_78*C10;
    ROcp13_810 = ROcp13_29*S10+ROcp13_88*C10;
    ROcp13_910 = ROcp13_39*S10+ROcp13_98*C10;
    ROcp13_411 = ROcp13_49*C11+ROcp13_710*S11;
    ROcp13_511 = ROcp13_59*C11+ROcp13_810*S11;
    ROcp13_611 = ROcp13_69*C11+ROcp13_910*S11;
    ROcp13_711 = -(ROcp13_49*S11-ROcp13_710*C11);
    ROcp13_811 = -(ROcp13_59*S11-ROcp13_810*C11);
    ROcp13_911 = -(ROcp13_69*S11-ROcp13_910*C11);
    RLcp13_17 = s->dpt[1][1]*ROcp13_16+s->dpt[3][1]*S5+ROcp13_46*s->dpt[2][1];
    RLcp13_27 = s->dpt[1][1]*ROcp13_26+s->dpt[3][1]*ROcp13_85+ROcp13_56*s->dpt[2][1];
    RLcp13_37 = s->dpt[1][1]*ROcp13_36+s->dpt[3][1]*ROcp13_95+ROcp13_66*s->dpt[2][1];
    OMcp13_17 = OMcp13_16+ROcp13_46*qd[7];
    OMcp13_27 = OMcp13_26+ROcp13_56*qd[7];
    OMcp13_37 = OMcp13_36+ROcp13_66*qd[7];
    ORcp13_17 = OMcp13_26*RLcp13_37-OMcp13_36*RLcp13_27;
    ORcp13_27 = -(OMcp13_16*RLcp13_37-OMcp13_36*RLcp13_17);
    ORcp13_37 = OMcp13_16*RLcp13_27-OMcp13_26*RLcp13_17;
    OPcp13_17 = OPcp13_16+ROcp13_46*qdd[7]+qd[7]*(OMcp13_26*ROcp13_66-OMcp13_36*ROcp13_56);
    OPcp13_27 = OPcp13_26+ROcp13_56*qdd[7]-qd[7]*(OMcp13_16*ROcp13_66-OMcp13_36*ROcp13_46);
    OPcp13_37 = OPcp13_36+ROcp13_66*qdd[7]+qd[7]*(OMcp13_16*ROcp13_56-OMcp13_26*ROcp13_46);
    RLcp13_18 = s->dpt[1][6]*ROcp13_17+s->dpt[3][6]*ROcp13_77+ROcp13_46*s->dpt[2][6];
    RLcp13_28 = s->dpt[1][6]*ROcp13_27+s->dpt[3][6]*ROcp13_87+ROcp13_56*s->dpt[2][6];
    RLcp13_38 = s->dpt[1][6]*ROcp13_37+s->dpt[3][6]*ROcp13_97+ROcp13_66*s->dpt[2][6];
    OMcp13_18 = OMcp13_17+ROcp13_17*qd[8];
    OMcp13_28 = OMcp13_27+ROcp13_27*qd[8];
    OMcp13_38 = OMcp13_37+ROcp13_37*qd[8];
    ORcp13_18 = OMcp13_27*RLcp13_38-OMcp13_37*RLcp13_28;
    ORcp13_28 = -(OMcp13_17*RLcp13_38-OMcp13_37*RLcp13_18);
    ORcp13_38 = OMcp13_17*RLcp13_28-OMcp13_27*RLcp13_18;
    OPcp13_18 = OPcp13_17+ROcp13_17*qdd[8]+qd[8]*(OMcp13_27*ROcp13_37-OMcp13_37*ROcp13_27);
    OPcp13_28 = OPcp13_27+ROcp13_27*qdd[8]-qd[8]*(OMcp13_17*ROcp13_37-OMcp13_37*ROcp13_17);
    OPcp13_38 = OPcp13_37+ROcp13_37*qdd[8]+qd[8]*(OMcp13_17*ROcp13_27-OMcp13_27*ROcp13_17);
    RLcp13_19 = s->dpt[1][8]*ROcp13_17+s->dpt[2][8]*ROcp13_48+ROcp13_78*s->dpt[3][8];
    RLcp13_29 = s->dpt[1][8]*ROcp13_27+s->dpt[2][8]*ROcp13_58+ROcp13_88*s->dpt[3][8];
    RLcp13_39 = s->dpt[1][8]*ROcp13_37+s->dpt[2][8]*ROcp13_68+ROcp13_98*s->dpt[3][8];
    OMcp13_19 = OMcp13_18+ROcp13_78*qd[9];
    OMcp13_29 = OMcp13_28+ROcp13_88*qd[9];
    OMcp13_39 = OMcp13_38+ROcp13_98*qd[9];
    ORcp13_19 = OMcp13_28*RLcp13_39-OMcp13_38*RLcp13_29;
    ORcp13_29 = -(OMcp13_18*RLcp13_39-OMcp13_38*RLcp13_19);
    ORcp13_39 = OMcp13_18*RLcp13_29-OMcp13_28*RLcp13_19;
    OPcp13_19 = OPcp13_18+ROcp13_78*qdd[9]+qd[9]*(OMcp13_28*ROcp13_98-OMcp13_38*ROcp13_88);
    OPcp13_29 = OPcp13_28+ROcp13_88*qdd[9]-qd[9]*(OMcp13_18*ROcp13_98-OMcp13_38*ROcp13_78);
    OPcp13_39 = OPcp13_38+ROcp13_98*qdd[9]+qd[9]*(OMcp13_18*ROcp13_88-OMcp13_28*ROcp13_78);
    RLcp13_110 = s->dpt[1][10]*ROcp13_19+s->dpt[2][10]*ROcp13_49+ROcp13_78*s->dpt[3][10];
    RLcp13_210 = s->dpt[1][10]*ROcp13_29+s->dpt[2][10]*ROcp13_59+ROcp13_88*s->dpt[3][10];
    RLcp13_310 = s->dpt[1][10]*ROcp13_39+s->dpt[2][10]*ROcp13_69+ROcp13_98*s->dpt[3][10];
    OMcp13_110 = OMcp13_19+ROcp13_49*qd[10];
    OMcp13_210 = OMcp13_29+ROcp13_59*qd[10];
    OMcp13_310 = OMcp13_39+ROcp13_69*qd[10];
    ORcp13_110 = OMcp13_29*RLcp13_310-OMcp13_39*RLcp13_210;
    ORcp13_210 = -(OMcp13_19*RLcp13_310-OMcp13_39*RLcp13_110);
    ORcp13_310 = OMcp13_19*RLcp13_210-OMcp13_29*RLcp13_110;
    OPcp13_110 = OPcp13_19+ROcp13_49*qdd[10]+qd[10]*(OMcp13_29*ROcp13_69-OMcp13_39*ROcp13_59);
    OPcp13_210 = OPcp13_29+ROcp13_59*qdd[10]-qd[10]*(OMcp13_19*ROcp13_69-OMcp13_39*ROcp13_49);
    OPcp13_310 = OPcp13_39+ROcp13_69*qdd[10]+qd[10]*(OMcp13_19*ROcp13_59-OMcp13_29*ROcp13_49);
    RLcp13_111 = s->dpt[1][12]*ROcp13_110+s->dpt[2][12]*ROcp13_49+ROcp13_710*s->dpt[3][12];
    RLcp13_211 = s->dpt[1][12]*ROcp13_210+s->dpt[2][12]*ROcp13_59+ROcp13_810*s->dpt[3][12];
    RLcp13_311 = s->dpt[1][12]*ROcp13_310+s->dpt[2][12]*ROcp13_69+ROcp13_910*s->dpt[3][12];
    OMcp13_111 = OMcp13_110+ROcp13_110*qd[11];
    OMcp13_211 = OMcp13_210+ROcp13_210*qd[11];
    OMcp13_311 = OMcp13_310+ROcp13_310*qd[11];
    ORcp13_111 = OMcp13_210*RLcp13_311-OMcp13_310*RLcp13_211;
    ORcp13_211 = -(OMcp13_110*RLcp13_311-OMcp13_310*RLcp13_111);
    ORcp13_311 = OMcp13_110*RLcp13_211-OMcp13_210*RLcp13_111;
    OPcp13_111 = OPcp13_110+ROcp13_110*qdd[11]+qd[11]*(OMcp13_210*ROcp13_310-OMcp13_310*ROcp13_210);
    OPcp13_211 = OPcp13_210+ROcp13_210*qdd[11]-qd[11]*(OMcp13_110*ROcp13_310-OMcp13_310*ROcp13_110);
    OPcp13_311 = OPcp13_310+ROcp13_310*qdd[11]+qd[11]*(OMcp13_110*ROcp13_210-OMcp13_210*ROcp13_110);
    RLcp13_149 = ROcp13_110*s->dpt[1][15]+ROcp13_411*s->dpt[2][15]+ROcp13_711*s->dpt[3][15];
    RLcp13_249 = ROcp13_210*s->dpt[1][15]+ROcp13_511*s->dpt[2][15]+ROcp13_811*s->dpt[3][15];
    RLcp13_349 = ROcp13_310*s->dpt[1][15]+ROcp13_611*s->dpt[2][15]+ROcp13_911*s->dpt[3][15];
    POcp13_149 = RLcp13_110+RLcp13_111+RLcp13_149+RLcp13_17+RLcp13_18+RLcp13_19+q[1];
    POcp13_249 = RLcp13_210+RLcp13_211+RLcp13_249+RLcp13_27+RLcp13_28+RLcp13_29+q[2];
    POcp13_349 = RLcp13_310+RLcp13_311+RLcp13_349+RLcp13_37+RLcp13_38+RLcp13_39+q[3];
    JTcp13_249_4 = -(RLcp13_310+RLcp13_311+RLcp13_349+RLcp13_37+RLcp13_38+RLcp13_39);
    JTcp13_349_4 = RLcp13_210+RLcp13_211+RLcp13_249+RLcp13_27+RLcp13_28+RLcp13_29;
    JTcp13_149_5 = C4*(RLcp13_310+RLcp13_311+RLcp13_349+RLcp13_37+RLcp13_38+RLcp13_39)-S4*(RLcp13_210+RLcp13_29)-S4*(
 RLcp13_211+RLcp13_249)-S4*(RLcp13_27+RLcp13_28);
    JTcp13_249_5 = S4*(RLcp13_110+RLcp13_111+RLcp13_149+RLcp13_17+RLcp13_18+RLcp13_19);
    JTcp13_349_5 = -C4*(RLcp13_110+RLcp13_111+RLcp13_149+RLcp13_17+RLcp13_18+RLcp13_19);
    JTcp13_149_6 = ROcp13_85*(RLcp13_310+RLcp13_311+RLcp13_349+RLcp13_37+RLcp13_38+RLcp13_39)-ROcp13_95*(RLcp13_210+
 RLcp13_29)-ROcp13_95*(RLcp13_211+RLcp13_249)-ROcp13_95*(RLcp13_27+RLcp13_28);
    JTcp13_249_6 = RLcp13_149*ROcp13_95-RLcp13_311*S5-RLcp13_349*S5+ROcp13_95*(RLcp13_110+RLcp13_111+RLcp13_17+RLcp13_18+
 RLcp13_19)-S5*(RLcp13_310+RLcp13_39)-S5*(RLcp13_37+RLcp13_38);
    JTcp13_349_6 = RLcp13_211*S5-ROcp13_85*(RLcp13_110+RLcp13_111+RLcp13_17+RLcp13_18+RLcp13_19)+S5*(RLcp13_210+RLcp13_29)
 +S5*(RLcp13_27+RLcp13_28)-RLcp13_149*ROcp13_85+RLcp13_249*S5;
    JTcp13_149_7 = ROcp13_56*(RLcp13_310+RLcp13_311+RLcp13_38+RLcp13_39)-ROcp13_66*(RLcp13_210+RLcp13_211)-ROcp13_66*(
 RLcp13_28+RLcp13_29)-RLcp13_249*ROcp13_66+RLcp13_349*ROcp13_56;
    JTcp13_249_7 = RLcp13_149*ROcp13_66-RLcp13_349*ROcp13_46-ROcp13_46*(RLcp13_310+RLcp13_311+RLcp13_38+RLcp13_39)+
 ROcp13_66*(RLcp13_110+RLcp13_111)+ROcp13_66*(RLcp13_18+RLcp13_19);
    JTcp13_349_7 = ROcp13_46*(RLcp13_210+RLcp13_211+RLcp13_28+RLcp13_29)-ROcp13_56*(RLcp13_110+RLcp13_111)-ROcp13_56*(
 RLcp13_18+RLcp13_19)-RLcp13_149*ROcp13_56+RLcp13_249*ROcp13_46;
    JTcp13_149_8 = ROcp13_27*(RLcp13_310+RLcp13_311+RLcp13_349+RLcp13_39)-ROcp13_37*(RLcp13_210+RLcp13_29)-ROcp13_37*(
 RLcp13_211+RLcp13_249);
    JTcp13_249_8 = -(ROcp13_17*(RLcp13_310+RLcp13_311+RLcp13_349+RLcp13_39)-ROcp13_37*(RLcp13_110+RLcp13_19)-ROcp13_37*(
 RLcp13_111+RLcp13_149));
    JTcp13_349_8 = ROcp13_17*(RLcp13_210+RLcp13_211+RLcp13_249+RLcp13_29)-ROcp13_27*(RLcp13_110+RLcp13_19)-ROcp13_27*(
 RLcp13_111+RLcp13_149);
    JTcp13_149_9 = ROcp13_88*(RLcp13_310+RLcp13_311)-ROcp13_98*(RLcp13_210+RLcp13_211)-RLcp13_249*ROcp13_98+RLcp13_349*
 ROcp13_88;
    JTcp13_249_9 = RLcp13_149*ROcp13_98-RLcp13_349*ROcp13_78-ROcp13_78*(RLcp13_310+RLcp13_311)+ROcp13_98*(RLcp13_110+
 RLcp13_111);
    JTcp13_349_9 = ROcp13_78*(RLcp13_210+RLcp13_211)-ROcp13_88*(RLcp13_110+RLcp13_111)-RLcp13_149*ROcp13_88+RLcp13_249*
 ROcp13_78;
    JTcp13_149_10 = ROcp13_59*(RLcp13_311+RLcp13_349)-ROcp13_69*(RLcp13_211+RLcp13_249);
    JTcp13_249_10 = -(ROcp13_49*(RLcp13_311+RLcp13_349)-ROcp13_69*(RLcp13_111+RLcp13_149));
    JTcp13_349_10 = ROcp13_49*(RLcp13_211+RLcp13_249)-ROcp13_59*(RLcp13_111+RLcp13_149);
    JTcp13_149_11 = -(RLcp13_249*ROcp13_310-RLcp13_349*ROcp13_210);
    JTcp13_249_11 = RLcp13_149*ROcp13_310-RLcp13_349*ROcp13_110;
    JTcp13_349_11 = -(RLcp13_149*ROcp13_210-RLcp13_249*ROcp13_110);
    ORcp13_149 = OMcp13_211*RLcp13_349-OMcp13_311*RLcp13_249;
    ORcp13_249 = -(OMcp13_111*RLcp13_349-OMcp13_311*RLcp13_149);
    ORcp13_349 = OMcp13_111*RLcp13_249-OMcp13_211*RLcp13_149;
    VIcp13_149 = ORcp13_110+ORcp13_111+ORcp13_149+ORcp13_17+ORcp13_18+ORcp13_19+qd[1];
    VIcp13_249 = ORcp13_210+ORcp13_211+ORcp13_249+ORcp13_27+ORcp13_28+ORcp13_29+qd[2];
    VIcp13_349 = ORcp13_310+ORcp13_311+ORcp13_349+ORcp13_37+ORcp13_38+ORcp13_39+qd[3];
    ACcp13_149 = qdd[1]+OMcp13_210*ORcp13_311+OMcp13_211*ORcp13_349+OMcp13_26*ORcp13_37+OMcp13_27*ORcp13_38+OMcp13_28*
 ORcp13_39+OMcp13_29*ORcp13_310-OMcp13_310*ORcp13_211-OMcp13_311*ORcp13_249-OMcp13_36*ORcp13_27-OMcp13_37*ORcp13_28-OMcp13_38
 *ORcp13_29-OMcp13_39*ORcp13_210+OPcp13_210*RLcp13_311+OPcp13_211*RLcp13_349+OPcp13_26*RLcp13_37+OPcp13_27*RLcp13_38+
 OPcp13_28*RLcp13_39+OPcp13_29*RLcp13_310-OPcp13_310*RLcp13_211-OPcp13_311*RLcp13_249-OPcp13_36*RLcp13_27-OPcp13_37*RLcp13_28
 -OPcp13_38*RLcp13_29-OPcp13_39*RLcp13_210;
    ACcp13_249 = qdd[2]-OMcp13_110*ORcp13_311-OMcp13_111*ORcp13_349-OMcp13_16*ORcp13_37-OMcp13_17*ORcp13_38-OMcp13_18*
 ORcp13_39-OMcp13_19*ORcp13_310+OMcp13_310*ORcp13_111+OMcp13_311*ORcp13_149+OMcp13_36*ORcp13_17+OMcp13_37*ORcp13_18+OMcp13_38
 *ORcp13_19+OMcp13_39*ORcp13_110-OPcp13_110*RLcp13_311-OPcp13_111*RLcp13_349-OPcp13_16*RLcp13_37-OPcp13_17*RLcp13_38-
 OPcp13_18*RLcp13_39-OPcp13_19*RLcp13_310+OPcp13_310*RLcp13_111+OPcp13_311*RLcp13_149+OPcp13_36*RLcp13_17+OPcp13_37*RLcp13_18
 +OPcp13_38*RLcp13_19+OPcp13_39*RLcp13_110;
    ACcp13_349 = qdd[3]+OMcp13_110*ORcp13_211+OMcp13_111*ORcp13_249+OMcp13_16*ORcp13_27+OMcp13_17*ORcp13_28+OMcp13_18*
 ORcp13_29+OMcp13_19*ORcp13_210-OMcp13_210*ORcp13_111-OMcp13_211*ORcp13_149-OMcp13_26*ORcp13_17-OMcp13_27*ORcp13_18-OMcp13_28
 *ORcp13_19-OMcp13_29*ORcp13_110+OPcp13_110*RLcp13_211+OPcp13_111*RLcp13_249+OPcp13_16*RLcp13_27+OPcp13_17*RLcp13_28+
 OPcp13_18*RLcp13_29+OPcp13_19*RLcp13_210-OPcp13_210*RLcp13_111-OPcp13_211*RLcp13_149-OPcp13_26*RLcp13_17-OPcp13_27*RLcp13_18
 -OPcp13_28*RLcp13_19-OPcp13_29*RLcp13_110;

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_149;
    sens->P[2] = POcp13_249;
    sens->P[3] = POcp13_349;
    sens->R[1][1] = ROcp13_110;
    sens->R[1][2] = ROcp13_210;
    sens->R[1][3] = ROcp13_310;
    sens->R[2][1] = ROcp13_411;
    sens->R[2][2] = ROcp13_511;
    sens->R[2][3] = ROcp13_611;
    sens->R[3][1] = ROcp13_711;
    sens->R[3][2] = ROcp13_811;
    sens->R[3][3] = ROcp13_911;
    sens->V[1] = VIcp13_149;
    sens->V[2] = VIcp13_249;
    sens->V[3] = VIcp13_349;
    sens->OM[1] = OMcp13_111;
    sens->OM[2] = OMcp13_211;
    sens->OM[3] = OMcp13_311;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp13_149_5;
    sens->J[1][6] = JTcp13_149_6;
    sens->J[1][7] = JTcp13_149_7;
    sens->J[1][8] = JTcp13_149_8;
    sens->J[1][9] = JTcp13_149_9;
    sens->J[1][10] = JTcp13_149_10;
    sens->J[1][11] = JTcp13_149_11;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp13_249_4;
    sens->J[2][5] = JTcp13_249_5;
    sens->J[2][6] = JTcp13_249_6;
    sens->J[2][7] = JTcp13_249_7;
    sens->J[2][8] = JTcp13_249_8;
    sens->J[2][9] = JTcp13_249_9;
    sens->J[2][10] = JTcp13_249_10;
    sens->J[2][11] = JTcp13_249_11;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp13_349_4;
    sens->J[3][5] = JTcp13_349_5;
    sens->J[3][6] = JTcp13_349_6;
    sens->J[3][7] = JTcp13_349_7;
    sens->J[3][8] = JTcp13_349_8;
    sens->J[3][9] = JTcp13_349_9;
    sens->J[3][10] = JTcp13_349_10;
    sens->J[3][11] = JTcp13_349_11;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp13_46;
    sens->J[4][8] = ROcp13_17;
    sens->J[4][9] = ROcp13_78;
    sens->J[4][10] = ROcp13_49;
    sens->J[4][11] = ROcp13_110;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp13_85;
    sens->J[5][7] = ROcp13_56;
    sens->J[5][8] = ROcp13_27;
    sens->J[5][9] = ROcp13_88;
    sens->J[5][10] = ROcp13_59;
    sens->J[5][11] = ROcp13_210;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp13_95;
    sens->J[6][7] = ROcp13_66;
    sens->J[6][8] = ROcp13_37;
    sens->J[6][9] = ROcp13_98;
    sens->J[6][10] = ROcp13_69;
    sens->J[6][11] = ROcp13_310;
    sens->A[1] = ACcp13_149;
    sens->A[2] = ACcp13_249;
    sens->A[3] = ACcp13_349;
    sens->OMP[1] = OPcp13_111;
    sens->OMP[2] = OPcp13_211;
    sens->OMP[3] = OPcp13_311;
 
// 
break;
case 15:
 


// = = Block_1_0_0_15_0_1 = = 
 
// Sensor Kinematics 


    ROcp14_25 = S4*S5;
    ROcp14_35 = -C4*S5;
    ROcp14_85 = -S4*C5;
    ROcp14_95 = C4*C5;
    ROcp14_16 = C5*C6;
    ROcp14_26 = ROcp14_25*C6+C4*S6;
    ROcp14_36 = ROcp14_35*C6+S4*S6;
    ROcp14_46 = -C5*S6;
    ROcp14_56 = -(ROcp14_25*S6-C4*C6);
    ROcp14_66 = -(ROcp14_35*S6-S4*C6);
    OMcp14_25 = qd[5]*C4;
    OMcp14_35 = qd[5]*S4;
    OMcp14_16 = qd[4]+qd[6]*S5;
    OMcp14_26 = OMcp14_25+ROcp14_85*qd[6];
    OMcp14_36 = OMcp14_35+ROcp14_95*qd[6];
    OPcp14_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp14_26 = ROcp14_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp14_35*S5-ROcp14_95*qd[4]);
    OPcp14_36 = ROcp14_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp14_25*S5-ROcp14_85*qd[4]);

// = = Block_1_0_0_15_0_2 = = 
 
// Sensor Kinematics 


    ROcp14_17 = ROcp14_16*C7-S5*S7;
    ROcp14_27 = ROcp14_26*C7-ROcp14_85*S7;
    ROcp14_37 = ROcp14_36*C7-ROcp14_95*S7;
    ROcp14_77 = ROcp14_16*S7+S5*C7;
    ROcp14_87 = ROcp14_26*S7+ROcp14_85*C7;
    ROcp14_97 = ROcp14_36*S7+ROcp14_95*C7;
    ROcp14_48 = ROcp14_46*C8+ROcp14_77*S8;
    ROcp14_58 = ROcp14_56*C8+ROcp14_87*S8;
    ROcp14_68 = ROcp14_66*C8+ROcp14_97*S8;
    ROcp14_78 = -(ROcp14_46*S8-ROcp14_77*C8);
    ROcp14_88 = -(ROcp14_56*S8-ROcp14_87*C8);
    ROcp14_98 = -(ROcp14_66*S8-ROcp14_97*C8);
    ROcp14_19 = ROcp14_17*C9+ROcp14_48*S9;
    ROcp14_29 = ROcp14_27*C9+ROcp14_58*S9;
    ROcp14_39 = ROcp14_37*C9+ROcp14_68*S9;
    ROcp14_49 = -(ROcp14_17*S9-ROcp14_48*C9);
    ROcp14_59 = -(ROcp14_27*S9-ROcp14_58*C9);
    ROcp14_69 = -(ROcp14_37*S9-ROcp14_68*C9);
    ROcp14_110 = ROcp14_19*C10-ROcp14_78*S10;
    ROcp14_210 = ROcp14_29*C10-ROcp14_88*S10;
    ROcp14_310 = ROcp14_39*C10-ROcp14_98*S10;
    ROcp14_710 = ROcp14_19*S10+ROcp14_78*C10;
    ROcp14_810 = ROcp14_29*S10+ROcp14_88*C10;
    ROcp14_910 = ROcp14_39*S10+ROcp14_98*C10;
    ROcp14_411 = ROcp14_49*C11+ROcp14_710*S11;
    ROcp14_511 = ROcp14_59*C11+ROcp14_810*S11;
    ROcp14_611 = ROcp14_69*C11+ROcp14_910*S11;
    ROcp14_711 = -(ROcp14_49*S11-ROcp14_710*C11);
    ROcp14_811 = -(ROcp14_59*S11-ROcp14_810*C11);
    ROcp14_911 = -(ROcp14_69*S11-ROcp14_910*C11);
    ROcp14_112 = ROcp14_110*C12-ROcp14_711*S12;
    ROcp14_212 = ROcp14_210*C12-ROcp14_811*S12;
    ROcp14_312 = ROcp14_310*C12-ROcp14_911*S12;
    ROcp14_712 = ROcp14_110*S12+ROcp14_711*C12;
    ROcp14_812 = ROcp14_210*S12+ROcp14_811*C12;
    ROcp14_912 = ROcp14_310*S12+ROcp14_911*C12;
    RLcp14_17 = s->dpt[1][1]*ROcp14_16+s->dpt[3][1]*S5+ROcp14_46*s->dpt[2][1];
    RLcp14_27 = s->dpt[1][1]*ROcp14_26+s->dpt[3][1]*ROcp14_85+ROcp14_56*s->dpt[2][1];
    RLcp14_37 = s->dpt[1][1]*ROcp14_36+s->dpt[3][1]*ROcp14_95+ROcp14_66*s->dpt[2][1];
    OMcp14_17 = OMcp14_16+ROcp14_46*qd[7];
    OMcp14_27 = OMcp14_26+ROcp14_56*qd[7];
    OMcp14_37 = OMcp14_36+ROcp14_66*qd[7];
    ORcp14_17 = OMcp14_26*RLcp14_37-OMcp14_36*RLcp14_27;
    ORcp14_27 = -(OMcp14_16*RLcp14_37-OMcp14_36*RLcp14_17);
    ORcp14_37 = OMcp14_16*RLcp14_27-OMcp14_26*RLcp14_17;
    OPcp14_17 = OPcp14_16+ROcp14_46*qdd[7]+qd[7]*(OMcp14_26*ROcp14_66-OMcp14_36*ROcp14_56);
    OPcp14_27 = OPcp14_26+ROcp14_56*qdd[7]-qd[7]*(OMcp14_16*ROcp14_66-OMcp14_36*ROcp14_46);
    OPcp14_37 = OPcp14_36+ROcp14_66*qdd[7]+qd[7]*(OMcp14_16*ROcp14_56-OMcp14_26*ROcp14_46);
    RLcp14_18 = s->dpt[1][6]*ROcp14_17+s->dpt[3][6]*ROcp14_77+ROcp14_46*s->dpt[2][6];
    RLcp14_28 = s->dpt[1][6]*ROcp14_27+s->dpt[3][6]*ROcp14_87+ROcp14_56*s->dpt[2][6];
    RLcp14_38 = s->dpt[1][6]*ROcp14_37+s->dpt[3][6]*ROcp14_97+ROcp14_66*s->dpt[2][6];
    OMcp14_18 = OMcp14_17+ROcp14_17*qd[8];
    OMcp14_28 = OMcp14_27+ROcp14_27*qd[8];
    OMcp14_38 = OMcp14_37+ROcp14_37*qd[8];
    ORcp14_18 = OMcp14_27*RLcp14_38-OMcp14_37*RLcp14_28;
    ORcp14_28 = -(OMcp14_17*RLcp14_38-OMcp14_37*RLcp14_18);
    ORcp14_38 = OMcp14_17*RLcp14_28-OMcp14_27*RLcp14_18;
    OPcp14_18 = OPcp14_17+ROcp14_17*qdd[8]+qd[8]*(OMcp14_27*ROcp14_37-OMcp14_37*ROcp14_27);
    OPcp14_28 = OPcp14_27+ROcp14_27*qdd[8]-qd[8]*(OMcp14_17*ROcp14_37-OMcp14_37*ROcp14_17);
    OPcp14_38 = OPcp14_37+ROcp14_37*qdd[8]+qd[8]*(OMcp14_17*ROcp14_27-OMcp14_27*ROcp14_17);
    RLcp14_19 = s->dpt[1][8]*ROcp14_17+s->dpt[2][8]*ROcp14_48+ROcp14_78*s->dpt[3][8];
    RLcp14_29 = s->dpt[1][8]*ROcp14_27+s->dpt[2][8]*ROcp14_58+ROcp14_88*s->dpt[3][8];
    RLcp14_39 = s->dpt[1][8]*ROcp14_37+s->dpt[2][8]*ROcp14_68+ROcp14_98*s->dpt[3][8];
    OMcp14_19 = OMcp14_18+ROcp14_78*qd[9];
    OMcp14_29 = OMcp14_28+ROcp14_88*qd[9];
    OMcp14_39 = OMcp14_38+ROcp14_98*qd[9];
    ORcp14_19 = OMcp14_28*RLcp14_39-OMcp14_38*RLcp14_29;
    ORcp14_29 = -(OMcp14_18*RLcp14_39-OMcp14_38*RLcp14_19);
    ORcp14_39 = OMcp14_18*RLcp14_29-OMcp14_28*RLcp14_19;
    OPcp14_19 = OPcp14_18+ROcp14_78*qdd[9]+qd[9]*(OMcp14_28*ROcp14_98-OMcp14_38*ROcp14_88);
    OPcp14_29 = OPcp14_28+ROcp14_88*qdd[9]-qd[9]*(OMcp14_18*ROcp14_98-OMcp14_38*ROcp14_78);
    OPcp14_39 = OPcp14_38+ROcp14_98*qdd[9]+qd[9]*(OMcp14_18*ROcp14_88-OMcp14_28*ROcp14_78);
    RLcp14_110 = s->dpt[1][10]*ROcp14_19+s->dpt[2][10]*ROcp14_49+ROcp14_78*s->dpt[3][10];
    RLcp14_210 = s->dpt[1][10]*ROcp14_29+s->dpt[2][10]*ROcp14_59+ROcp14_88*s->dpt[3][10];
    RLcp14_310 = s->dpt[1][10]*ROcp14_39+s->dpt[2][10]*ROcp14_69+ROcp14_98*s->dpt[3][10];
    OMcp14_110 = OMcp14_19+ROcp14_49*qd[10];
    OMcp14_210 = OMcp14_29+ROcp14_59*qd[10];
    OMcp14_310 = OMcp14_39+ROcp14_69*qd[10];
    ORcp14_110 = OMcp14_29*RLcp14_310-OMcp14_39*RLcp14_210;
    ORcp14_210 = -(OMcp14_19*RLcp14_310-OMcp14_39*RLcp14_110);
    ORcp14_310 = OMcp14_19*RLcp14_210-OMcp14_29*RLcp14_110;
    OPcp14_110 = OPcp14_19+ROcp14_49*qdd[10]+qd[10]*(OMcp14_29*ROcp14_69-OMcp14_39*ROcp14_59);
    OPcp14_210 = OPcp14_29+ROcp14_59*qdd[10]-qd[10]*(OMcp14_19*ROcp14_69-OMcp14_39*ROcp14_49);
    OPcp14_310 = OPcp14_39+ROcp14_69*qdd[10]+qd[10]*(OMcp14_19*ROcp14_59-OMcp14_29*ROcp14_49);
    RLcp14_111 = s->dpt[1][12]*ROcp14_110+s->dpt[2][12]*ROcp14_49+ROcp14_710*s->dpt[3][12];
    RLcp14_211 = s->dpt[1][12]*ROcp14_210+s->dpt[2][12]*ROcp14_59+ROcp14_810*s->dpt[3][12];
    RLcp14_311 = s->dpt[1][12]*ROcp14_310+s->dpt[2][12]*ROcp14_69+ROcp14_910*s->dpt[3][12];
    OMcp14_111 = OMcp14_110+ROcp14_110*qd[11];
    OMcp14_211 = OMcp14_210+ROcp14_210*qd[11];
    OMcp14_311 = OMcp14_310+ROcp14_310*qd[11];
    ORcp14_111 = OMcp14_210*RLcp14_311-OMcp14_310*RLcp14_211;
    ORcp14_211 = -(OMcp14_110*RLcp14_311-OMcp14_310*RLcp14_111);
    ORcp14_311 = OMcp14_110*RLcp14_211-OMcp14_210*RLcp14_111;
    OPcp14_111 = OPcp14_110+ROcp14_110*qdd[11]+qd[11]*(OMcp14_210*ROcp14_310-OMcp14_310*ROcp14_210);
    OPcp14_211 = OPcp14_210+ROcp14_210*qdd[11]-qd[11]*(OMcp14_110*ROcp14_310-OMcp14_310*ROcp14_110);
    OPcp14_311 = OPcp14_310+ROcp14_310*qdd[11]+qd[11]*(OMcp14_110*ROcp14_210-OMcp14_210*ROcp14_110);
    RLcp14_112 = s->dpt[1][14]*ROcp14_110+s->dpt[2][14]*ROcp14_411+s->dpt[3][14]*ROcp14_711;
    RLcp14_212 = s->dpt[1][14]*ROcp14_210+s->dpt[2][14]*ROcp14_511+s->dpt[3][14]*ROcp14_811;
    RLcp14_312 = s->dpt[1][14]*ROcp14_310+s->dpt[2][14]*ROcp14_611+s->dpt[3][14]*ROcp14_911;
    OMcp14_112 = OMcp14_111+ROcp14_411*qd[12];
    OMcp14_212 = OMcp14_211+ROcp14_511*qd[12];
    OMcp14_312 = OMcp14_311+ROcp14_611*qd[12];
    ORcp14_112 = OMcp14_211*RLcp14_312-OMcp14_311*RLcp14_212;
    ORcp14_212 = -(OMcp14_111*RLcp14_312-OMcp14_311*RLcp14_112);
    ORcp14_312 = OMcp14_111*RLcp14_212-OMcp14_211*RLcp14_112;
    OPcp14_112 = OPcp14_111+ROcp14_411*qdd[12]+qd[12]*(OMcp14_211*ROcp14_611-OMcp14_311*ROcp14_511);
    OPcp14_212 = OPcp14_211+ROcp14_511*qdd[12]-qd[12]*(OMcp14_111*ROcp14_611-OMcp14_311*ROcp14_411);
    OPcp14_312 = OPcp14_311+ROcp14_611*qdd[12]+qd[12]*(OMcp14_111*ROcp14_511-OMcp14_211*ROcp14_411);
    RLcp14_150 = s->dpt[1][16]*ROcp14_112+s->dpt[2][16]*ROcp14_411+ROcp14_712*s->dpt[3][16];
    RLcp14_250 = s->dpt[1][16]*ROcp14_212+s->dpt[2][16]*ROcp14_511+ROcp14_812*s->dpt[3][16];
    RLcp14_350 = s->dpt[1][16]*ROcp14_312+s->dpt[2][16]*ROcp14_611+ROcp14_912*s->dpt[3][16];
    POcp14_150 = RLcp14_110+RLcp14_111+RLcp14_112+RLcp14_150+RLcp14_17+RLcp14_18+RLcp14_19+q[1];
    POcp14_250 = RLcp14_210+RLcp14_211+RLcp14_212+RLcp14_250+RLcp14_27+RLcp14_28+RLcp14_29+q[2];
    POcp14_350 = RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_350+RLcp14_37+RLcp14_38+RLcp14_39+q[3];
    JTcp14_250_4 = -(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_350+RLcp14_37+RLcp14_38+RLcp14_39);
    JTcp14_350_4 = RLcp14_210+RLcp14_211+RLcp14_212+RLcp14_250+RLcp14_27+RLcp14_28+RLcp14_29;
    JTcp14_150_5 = C4*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_37+RLcp14_38+RLcp14_39)-S4*(RLcp14_210+RLcp14_29)-S4*(
 RLcp14_211+RLcp14_212)-S4*(RLcp14_27+RLcp14_28)-RLcp14_250*S4+RLcp14_350*C4;
    JTcp14_250_5 = S4*(RLcp14_110+RLcp14_111+RLcp14_112+RLcp14_150+RLcp14_17+RLcp14_18+RLcp14_19);
    JTcp14_350_5 = -C4*(RLcp14_110+RLcp14_111+RLcp14_112+RLcp14_150+RLcp14_17+RLcp14_18+RLcp14_19);
    JTcp14_150_6 = ROcp14_85*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_37+RLcp14_38+RLcp14_39)-ROcp14_95*(RLcp14_210+
 RLcp14_29)-ROcp14_95*(RLcp14_211+RLcp14_212)-ROcp14_95*(RLcp14_27+RLcp14_28)-RLcp14_250*ROcp14_95+RLcp14_350*ROcp14_85;
    JTcp14_250_6 = -(RLcp14_350*S5-ROcp14_95*(RLcp14_110+RLcp14_111+RLcp14_112+RLcp14_150+RLcp14_17+RLcp14_18+RLcp14_19)+
 S5*(RLcp14_310+RLcp14_39)+S5*(RLcp14_311+RLcp14_312)+S5*(RLcp14_37+RLcp14_38));
    JTcp14_350_6 = RLcp14_250*S5-ROcp14_85*(RLcp14_110+RLcp14_111+RLcp14_112+RLcp14_150+RLcp14_17+RLcp14_18+RLcp14_19)+S5*
 (RLcp14_210+RLcp14_29)+S5*(RLcp14_211+RLcp14_212)+S5*(RLcp14_27+RLcp14_28);
    JTcp14_150_7 = ROcp14_56*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_350+RLcp14_38+RLcp14_39)-ROcp14_66*(RLcp14_210+
 RLcp14_211)-ROcp14_66*(RLcp14_212+RLcp14_250)-ROcp14_66*(RLcp14_28+RLcp14_29);
    JTcp14_250_7 = -(ROcp14_46*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_350+RLcp14_38+RLcp14_39)-ROcp14_66*(RLcp14_110+
 RLcp14_111)-ROcp14_66*(RLcp14_112+RLcp14_150)-ROcp14_66*(RLcp14_18+RLcp14_19));
    JTcp14_350_7 = ROcp14_46*(RLcp14_210+RLcp14_211+RLcp14_212+RLcp14_250+RLcp14_28+RLcp14_29)-ROcp14_56*(RLcp14_110+
 RLcp14_111)-ROcp14_56*(RLcp14_112+RLcp14_150)-ROcp14_56*(RLcp14_18+RLcp14_19);
    JTcp14_150_8 = ROcp14_27*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_39)-ROcp14_37*(RLcp14_210+RLcp14_29)-ROcp14_37*(
 RLcp14_211+RLcp14_212)-RLcp14_250*ROcp14_37+RLcp14_350*ROcp14_27;
    JTcp14_250_8 = RLcp14_150*ROcp14_37-RLcp14_350*ROcp14_17-ROcp14_17*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_39)+
 ROcp14_37*(RLcp14_110+RLcp14_19)+ROcp14_37*(RLcp14_111+RLcp14_112);
    JTcp14_350_8 = ROcp14_17*(RLcp14_210+RLcp14_211+RLcp14_212+RLcp14_29)-ROcp14_27*(RLcp14_110+RLcp14_19)-ROcp14_27*(
 RLcp14_111+RLcp14_112)-RLcp14_150*ROcp14_27+RLcp14_250*ROcp14_17;
    JTcp14_150_9 = ROcp14_88*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_350)-ROcp14_98*(RLcp14_210+RLcp14_211)-ROcp14_98*(
 RLcp14_212+RLcp14_250);
    JTcp14_250_9 = -(ROcp14_78*(RLcp14_310+RLcp14_311+RLcp14_312+RLcp14_350)-ROcp14_98*(RLcp14_110+RLcp14_111)-ROcp14_98*(
 RLcp14_112+RLcp14_150));
    JTcp14_350_9 = ROcp14_78*(RLcp14_210+RLcp14_211+RLcp14_212+RLcp14_250)-ROcp14_88*(RLcp14_110+RLcp14_111)-ROcp14_88*(
 RLcp14_112+RLcp14_150);
    JTcp14_150_10 = ROcp14_59*(RLcp14_311+RLcp14_312)-ROcp14_69*(RLcp14_211+RLcp14_212)-RLcp14_250*ROcp14_69+RLcp14_350*
 ROcp14_59;
    JTcp14_250_10 = RLcp14_150*ROcp14_69-RLcp14_350*ROcp14_49-ROcp14_49*(RLcp14_311+RLcp14_312)+ROcp14_69*(RLcp14_111+
 RLcp14_112);
    JTcp14_350_10 = ROcp14_49*(RLcp14_211+RLcp14_212)-ROcp14_59*(RLcp14_111+RLcp14_112)-RLcp14_150*ROcp14_59+RLcp14_250*
 ROcp14_49;
    JTcp14_150_11 = ROcp14_210*(RLcp14_312+RLcp14_350)-ROcp14_310*(RLcp14_212+RLcp14_250);
    JTcp14_250_11 = -(ROcp14_110*(RLcp14_312+RLcp14_350)-ROcp14_310*(RLcp14_112+RLcp14_150));
    JTcp14_350_11 = ROcp14_110*(RLcp14_212+RLcp14_250)-ROcp14_210*(RLcp14_112+RLcp14_150);
    JTcp14_150_12 = -(RLcp14_250*ROcp14_611-RLcp14_350*ROcp14_511);
    JTcp14_250_12 = RLcp14_150*ROcp14_611-RLcp14_350*ROcp14_411;
    JTcp14_350_12 = -(RLcp14_150*ROcp14_511-RLcp14_250*ROcp14_411);
    ORcp14_150 = OMcp14_212*RLcp14_350-OMcp14_312*RLcp14_250;
    ORcp14_250 = -(OMcp14_112*RLcp14_350-OMcp14_312*RLcp14_150);
    ORcp14_350 = OMcp14_112*RLcp14_250-OMcp14_212*RLcp14_150;
    VIcp14_150 = ORcp14_110+ORcp14_111+ORcp14_112+ORcp14_150+ORcp14_17+ORcp14_18+ORcp14_19+qd[1];
    VIcp14_250 = ORcp14_210+ORcp14_211+ORcp14_212+ORcp14_250+ORcp14_27+ORcp14_28+ORcp14_29+qd[2];
    VIcp14_350 = ORcp14_310+ORcp14_311+ORcp14_312+ORcp14_350+ORcp14_37+ORcp14_38+ORcp14_39+qd[3];
    ACcp14_150 = qdd[1]+OMcp14_210*ORcp14_311+OMcp14_211*ORcp14_312+OMcp14_212*ORcp14_350+OMcp14_26*ORcp14_37+OMcp14_27*
 ORcp14_38+OMcp14_28*ORcp14_39+OMcp14_29*ORcp14_310-OMcp14_310*ORcp14_211-OMcp14_311*ORcp14_212-OMcp14_312*ORcp14_250-
 OMcp14_36*ORcp14_27-OMcp14_37*ORcp14_28-OMcp14_38*ORcp14_29-OMcp14_39*ORcp14_210+OPcp14_210*RLcp14_311+OPcp14_211*RLcp14_312
 +OPcp14_212*RLcp14_350+OPcp14_26*RLcp14_37+OPcp14_27*RLcp14_38+OPcp14_28*RLcp14_39+OPcp14_29*RLcp14_310-OPcp14_310*
 RLcp14_211-OPcp14_311*RLcp14_212-OPcp14_312*RLcp14_250-OPcp14_36*RLcp14_27-OPcp14_37*RLcp14_28-OPcp14_38*RLcp14_29-OPcp14_39
 *RLcp14_210;
    ACcp14_250 = qdd[2]-OMcp14_110*ORcp14_311-OMcp14_111*ORcp14_312-OMcp14_112*ORcp14_350-OMcp14_16*ORcp14_37-OMcp14_17*
 ORcp14_38-OMcp14_18*ORcp14_39-OMcp14_19*ORcp14_310+OMcp14_310*ORcp14_111+OMcp14_311*ORcp14_112+OMcp14_312*ORcp14_150+
 OMcp14_36*ORcp14_17+OMcp14_37*ORcp14_18+OMcp14_38*ORcp14_19+OMcp14_39*ORcp14_110-OPcp14_110*RLcp14_311-OPcp14_111*RLcp14_312
 -OPcp14_112*RLcp14_350-OPcp14_16*RLcp14_37-OPcp14_17*RLcp14_38-OPcp14_18*RLcp14_39-OPcp14_19*RLcp14_310+OPcp14_310*
 RLcp14_111+OPcp14_311*RLcp14_112+OPcp14_312*RLcp14_150+OPcp14_36*RLcp14_17+OPcp14_37*RLcp14_18+OPcp14_38*RLcp14_19+OPcp14_39
 *RLcp14_110;
    ACcp14_350 = qdd[3]+OMcp14_110*ORcp14_211+OMcp14_111*ORcp14_212+OMcp14_112*ORcp14_250+OMcp14_16*ORcp14_27+OMcp14_17*
 ORcp14_28+OMcp14_18*ORcp14_29+OMcp14_19*ORcp14_210-OMcp14_210*ORcp14_111-OMcp14_211*ORcp14_112-OMcp14_212*ORcp14_150-
 OMcp14_26*ORcp14_17-OMcp14_27*ORcp14_18-OMcp14_28*ORcp14_19-OMcp14_29*ORcp14_110+OPcp14_110*RLcp14_211+OPcp14_111*RLcp14_212
 +OPcp14_112*RLcp14_250+OPcp14_16*RLcp14_27+OPcp14_17*RLcp14_28+OPcp14_18*RLcp14_29+OPcp14_19*RLcp14_210-OPcp14_210*
 RLcp14_111-OPcp14_211*RLcp14_112-OPcp14_212*RLcp14_150-OPcp14_26*RLcp14_17-OPcp14_27*RLcp14_18-OPcp14_28*RLcp14_19-OPcp14_29
 *RLcp14_110;

// = = Block_1_0_0_15_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp14_150;
    sens->P[2] = POcp14_250;
    sens->P[3] = POcp14_350;
    sens->R[1][1] = ROcp14_112;
    sens->R[1][2] = ROcp14_212;
    sens->R[1][3] = ROcp14_312;
    sens->R[2][1] = ROcp14_411;
    sens->R[2][2] = ROcp14_511;
    sens->R[2][3] = ROcp14_611;
    sens->R[3][1] = ROcp14_712;
    sens->R[3][2] = ROcp14_812;
    sens->R[3][3] = ROcp14_912;
    sens->V[1] = VIcp14_150;
    sens->V[2] = VIcp14_250;
    sens->V[3] = VIcp14_350;
    sens->OM[1] = OMcp14_112;
    sens->OM[2] = OMcp14_212;
    sens->OM[3] = OMcp14_312;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp14_150_5;
    sens->J[1][6] = JTcp14_150_6;
    sens->J[1][7] = JTcp14_150_7;
    sens->J[1][8] = JTcp14_150_8;
    sens->J[1][9] = JTcp14_150_9;
    sens->J[1][10] = JTcp14_150_10;
    sens->J[1][11] = JTcp14_150_11;
    sens->J[1][12] = JTcp14_150_12;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp14_250_4;
    sens->J[2][5] = JTcp14_250_5;
    sens->J[2][6] = JTcp14_250_6;
    sens->J[2][7] = JTcp14_250_7;
    sens->J[2][8] = JTcp14_250_8;
    sens->J[2][9] = JTcp14_250_9;
    sens->J[2][10] = JTcp14_250_10;
    sens->J[2][11] = JTcp14_250_11;
    sens->J[2][12] = JTcp14_250_12;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp14_350_4;
    sens->J[3][5] = JTcp14_350_5;
    sens->J[3][6] = JTcp14_350_6;
    sens->J[3][7] = JTcp14_350_7;
    sens->J[3][8] = JTcp14_350_8;
    sens->J[3][9] = JTcp14_350_9;
    sens->J[3][10] = JTcp14_350_10;
    sens->J[3][11] = JTcp14_350_11;
    sens->J[3][12] = JTcp14_350_12;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp14_46;
    sens->J[4][8] = ROcp14_17;
    sens->J[4][9] = ROcp14_78;
    sens->J[4][10] = ROcp14_49;
    sens->J[4][11] = ROcp14_110;
    sens->J[4][12] = ROcp14_411;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp14_85;
    sens->J[5][7] = ROcp14_56;
    sens->J[5][8] = ROcp14_27;
    sens->J[5][9] = ROcp14_88;
    sens->J[5][10] = ROcp14_59;
    sens->J[5][11] = ROcp14_210;
    sens->J[5][12] = ROcp14_511;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp14_95;
    sens->J[6][7] = ROcp14_66;
    sens->J[6][8] = ROcp14_37;
    sens->J[6][9] = ROcp14_98;
    sens->J[6][10] = ROcp14_69;
    sens->J[6][11] = ROcp14_310;
    sens->J[6][12] = ROcp14_611;
    sens->A[1] = ACcp14_150;
    sens->A[2] = ACcp14_250;
    sens->A[3] = ACcp14_350;
    sens->OMP[1] = OPcp14_112;
    sens->OMP[2] = OPcp14_212;
    sens->OMP[3] = OPcp14_312;
 
// 
break;
case 16:
 


// = = Block_1_0_0_16_0_1 = = 
 
// Sensor Kinematics 


    ROcp15_25 = S4*S5;
    ROcp15_35 = -C4*S5;
    ROcp15_85 = -S4*C5;
    ROcp15_95 = C4*C5;
    ROcp15_16 = C5*C6;
    ROcp15_26 = ROcp15_25*C6+C4*S6;
    ROcp15_36 = ROcp15_35*C6+S4*S6;
    ROcp15_46 = -C5*S6;
    ROcp15_56 = -(ROcp15_25*S6-C4*C6);
    ROcp15_66 = -(ROcp15_35*S6-S4*C6);
    OMcp15_25 = qd[5]*C4;
    OMcp15_35 = qd[5]*S4;
    OMcp15_16 = qd[4]+qd[6]*S5;
    OMcp15_26 = OMcp15_25+ROcp15_85*qd[6];
    OMcp15_36 = OMcp15_35+ROcp15_95*qd[6];
    OPcp15_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp15_26 = ROcp15_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp15_35*S5-ROcp15_95*qd[4]);
    OPcp15_36 = ROcp15_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp15_25*S5-ROcp15_85*qd[4]);

// = = Block_1_0_0_16_0_2 = = 
 
// Sensor Kinematics 


    ROcp15_17 = ROcp15_16*C7-S5*S7;
    ROcp15_27 = ROcp15_26*C7-ROcp15_85*S7;
    ROcp15_37 = ROcp15_36*C7-ROcp15_95*S7;
    ROcp15_77 = ROcp15_16*S7+S5*C7;
    ROcp15_87 = ROcp15_26*S7+ROcp15_85*C7;
    ROcp15_97 = ROcp15_36*S7+ROcp15_95*C7;
    ROcp15_48 = ROcp15_46*C8+ROcp15_77*S8;
    ROcp15_58 = ROcp15_56*C8+ROcp15_87*S8;
    ROcp15_68 = ROcp15_66*C8+ROcp15_97*S8;
    ROcp15_78 = -(ROcp15_46*S8-ROcp15_77*C8);
    ROcp15_88 = -(ROcp15_56*S8-ROcp15_87*C8);
    ROcp15_98 = -(ROcp15_66*S8-ROcp15_97*C8);
    ROcp15_19 = ROcp15_17*C9+ROcp15_48*S9;
    ROcp15_29 = ROcp15_27*C9+ROcp15_58*S9;
    ROcp15_39 = ROcp15_37*C9+ROcp15_68*S9;
    ROcp15_49 = -(ROcp15_17*S9-ROcp15_48*C9);
    ROcp15_59 = -(ROcp15_27*S9-ROcp15_58*C9);
    ROcp15_69 = -(ROcp15_37*S9-ROcp15_68*C9);
    ROcp15_110 = ROcp15_19*C10-ROcp15_78*S10;
    ROcp15_210 = ROcp15_29*C10-ROcp15_88*S10;
    ROcp15_310 = ROcp15_39*C10-ROcp15_98*S10;
    ROcp15_710 = ROcp15_19*S10+ROcp15_78*C10;
    ROcp15_810 = ROcp15_29*S10+ROcp15_88*C10;
    ROcp15_910 = ROcp15_39*S10+ROcp15_98*C10;
    ROcp15_411 = ROcp15_49*C11+ROcp15_710*S11;
    ROcp15_511 = ROcp15_59*C11+ROcp15_810*S11;
    ROcp15_611 = ROcp15_69*C11+ROcp15_910*S11;
    ROcp15_711 = -(ROcp15_49*S11-ROcp15_710*C11);
    ROcp15_811 = -(ROcp15_59*S11-ROcp15_810*C11);
    ROcp15_911 = -(ROcp15_69*S11-ROcp15_910*C11);
    ROcp15_112 = ROcp15_110*C12-ROcp15_711*S12;
    ROcp15_212 = ROcp15_210*C12-ROcp15_811*S12;
    ROcp15_312 = ROcp15_310*C12-ROcp15_911*S12;
    ROcp15_712 = ROcp15_110*S12+ROcp15_711*C12;
    ROcp15_812 = ROcp15_210*S12+ROcp15_811*C12;
    ROcp15_912 = ROcp15_310*S12+ROcp15_911*C12;
    RLcp15_17 = s->dpt[1][1]*ROcp15_16+s->dpt[3][1]*S5+ROcp15_46*s->dpt[2][1];
    RLcp15_27 = s->dpt[1][1]*ROcp15_26+s->dpt[3][1]*ROcp15_85+ROcp15_56*s->dpt[2][1];
    RLcp15_37 = s->dpt[1][1]*ROcp15_36+s->dpt[3][1]*ROcp15_95+ROcp15_66*s->dpt[2][1];
    OMcp15_17 = OMcp15_16+ROcp15_46*qd[7];
    OMcp15_27 = OMcp15_26+ROcp15_56*qd[7];
    OMcp15_37 = OMcp15_36+ROcp15_66*qd[7];
    ORcp15_17 = OMcp15_26*RLcp15_37-OMcp15_36*RLcp15_27;
    ORcp15_27 = -(OMcp15_16*RLcp15_37-OMcp15_36*RLcp15_17);
    ORcp15_37 = OMcp15_16*RLcp15_27-OMcp15_26*RLcp15_17;
    OPcp15_17 = OPcp15_16+ROcp15_46*qdd[7]+qd[7]*(OMcp15_26*ROcp15_66-OMcp15_36*ROcp15_56);
    OPcp15_27 = OPcp15_26+ROcp15_56*qdd[7]-qd[7]*(OMcp15_16*ROcp15_66-OMcp15_36*ROcp15_46);
    OPcp15_37 = OPcp15_36+ROcp15_66*qdd[7]+qd[7]*(OMcp15_16*ROcp15_56-OMcp15_26*ROcp15_46);
    RLcp15_18 = s->dpt[1][6]*ROcp15_17+s->dpt[3][6]*ROcp15_77+ROcp15_46*s->dpt[2][6];
    RLcp15_28 = s->dpt[1][6]*ROcp15_27+s->dpt[3][6]*ROcp15_87+ROcp15_56*s->dpt[2][6];
    RLcp15_38 = s->dpt[1][6]*ROcp15_37+s->dpt[3][6]*ROcp15_97+ROcp15_66*s->dpt[2][6];
    OMcp15_18 = OMcp15_17+ROcp15_17*qd[8];
    OMcp15_28 = OMcp15_27+ROcp15_27*qd[8];
    OMcp15_38 = OMcp15_37+ROcp15_37*qd[8];
    ORcp15_18 = OMcp15_27*RLcp15_38-OMcp15_37*RLcp15_28;
    ORcp15_28 = -(OMcp15_17*RLcp15_38-OMcp15_37*RLcp15_18);
    ORcp15_38 = OMcp15_17*RLcp15_28-OMcp15_27*RLcp15_18;
    OPcp15_18 = OPcp15_17+ROcp15_17*qdd[8]+qd[8]*(OMcp15_27*ROcp15_37-OMcp15_37*ROcp15_27);
    OPcp15_28 = OPcp15_27+ROcp15_27*qdd[8]-qd[8]*(OMcp15_17*ROcp15_37-OMcp15_37*ROcp15_17);
    OPcp15_38 = OPcp15_37+ROcp15_37*qdd[8]+qd[8]*(OMcp15_17*ROcp15_27-OMcp15_27*ROcp15_17);
    RLcp15_19 = s->dpt[1][8]*ROcp15_17+s->dpt[2][8]*ROcp15_48+ROcp15_78*s->dpt[3][8];
    RLcp15_29 = s->dpt[1][8]*ROcp15_27+s->dpt[2][8]*ROcp15_58+ROcp15_88*s->dpt[3][8];
    RLcp15_39 = s->dpt[1][8]*ROcp15_37+s->dpt[2][8]*ROcp15_68+ROcp15_98*s->dpt[3][8];
    OMcp15_19 = OMcp15_18+ROcp15_78*qd[9];
    OMcp15_29 = OMcp15_28+ROcp15_88*qd[9];
    OMcp15_39 = OMcp15_38+ROcp15_98*qd[9];
    ORcp15_19 = OMcp15_28*RLcp15_39-OMcp15_38*RLcp15_29;
    ORcp15_29 = -(OMcp15_18*RLcp15_39-OMcp15_38*RLcp15_19);
    ORcp15_39 = OMcp15_18*RLcp15_29-OMcp15_28*RLcp15_19;
    OPcp15_19 = OPcp15_18+ROcp15_78*qdd[9]+qd[9]*(OMcp15_28*ROcp15_98-OMcp15_38*ROcp15_88);
    OPcp15_29 = OPcp15_28+ROcp15_88*qdd[9]-qd[9]*(OMcp15_18*ROcp15_98-OMcp15_38*ROcp15_78);
    OPcp15_39 = OPcp15_38+ROcp15_98*qdd[9]+qd[9]*(OMcp15_18*ROcp15_88-OMcp15_28*ROcp15_78);
    RLcp15_110 = s->dpt[1][10]*ROcp15_19+s->dpt[2][10]*ROcp15_49+ROcp15_78*s->dpt[3][10];
    RLcp15_210 = s->dpt[1][10]*ROcp15_29+s->dpt[2][10]*ROcp15_59+ROcp15_88*s->dpt[3][10];
    RLcp15_310 = s->dpt[1][10]*ROcp15_39+s->dpt[2][10]*ROcp15_69+ROcp15_98*s->dpt[3][10];
    OMcp15_110 = OMcp15_19+ROcp15_49*qd[10];
    OMcp15_210 = OMcp15_29+ROcp15_59*qd[10];
    OMcp15_310 = OMcp15_39+ROcp15_69*qd[10];
    ORcp15_110 = OMcp15_29*RLcp15_310-OMcp15_39*RLcp15_210;
    ORcp15_210 = -(OMcp15_19*RLcp15_310-OMcp15_39*RLcp15_110);
    ORcp15_310 = OMcp15_19*RLcp15_210-OMcp15_29*RLcp15_110;
    OPcp15_110 = OPcp15_19+ROcp15_49*qdd[10]+qd[10]*(OMcp15_29*ROcp15_69-OMcp15_39*ROcp15_59);
    OPcp15_210 = OPcp15_29+ROcp15_59*qdd[10]-qd[10]*(OMcp15_19*ROcp15_69-OMcp15_39*ROcp15_49);
    OPcp15_310 = OPcp15_39+ROcp15_69*qdd[10]+qd[10]*(OMcp15_19*ROcp15_59-OMcp15_29*ROcp15_49);
    RLcp15_111 = s->dpt[1][12]*ROcp15_110+s->dpt[2][12]*ROcp15_49+ROcp15_710*s->dpt[3][12];
    RLcp15_211 = s->dpt[1][12]*ROcp15_210+s->dpt[2][12]*ROcp15_59+ROcp15_810*s->dpt[3][12];
    RLcp15_311 = s->dpt[1][12]*ROcp15_310+s->dpt[2][12]*ROcp15_69+ROcp15_910*s->dpt[3][12];
    OMcp15_111 = OMcp15_110+ROcp15_110*qd[11];
    OMcp15_211 = OMcp15_210+ROcp15_210*qd[11];
    OMcp15_311 = OMcp15_310+ROcp15_310*qd[11];
    ORcp15_111 = OMcp15_210*RLcp15_311-OMcp15_310*RLcp15_211;
    ORcp15_211 = -(OMcp15_110*RLcp15_311-OMcp15_310*RLcp15_111);
    ORcp15_311 = OMcp15_110*RLcp15_211-OMcp15_210*RLcp15_111;
    OPcp15_111 = OPcp15_110+ROcp15_110*qdd[11]+qd[11]*(OMcp15_210*ROcp15_310-OMcp15_310*ROcp15_210);
    OPcp15_211 = OPcp15_210+ROcp15_210*qdd[11]-qd[11]*(OMcp15_110*ROcp15_310-OMcp15_310*ROcp15_110);
    OPcp15_311 = OPcp15_310+ROcp15_310*qdd[11]+qd[11]*(OMcp15_110*ROcp15_210-OMcp15_210*ROcp15_110);
    RLcp15_112 = s->dpt[1][14]*ROcp15_110+s->dpt[2][14]*ROcp15_411+s->dpt[3][14]*ROcp15_711;
    RLcp15_212 = s->dpt[1][14]*ROcp15_210+s->dpt[2][14]*ROcp15_511+s->dpt[3][14]*ROcp15_811;
    RLcp15_312 = s->dpt[1][14]*ROcp15_310+s->dpt[2][14]*ROcp15_611+s->dpt[3][14]*ROcp15_911;
    OMcp15_112 = OMcp15_111+ROcp15_411*qd[12];
    OMcp15_212 = OMcp15_211+ROcp15_511*qd[12];
    OMcp15_312 = OMcp15_311+ROcp15_611*qd[12];
    ORcp15_112 = OMcp15_211*RLcp15_312-OMcp15_311*RLcp15_212;
    ORcp15_212 = -(OMcp15_111*RLcp15_312-OMcp15_311*RLcp15_112);
    ORcp15_312 = OMcp15_111*RLcp15_212-OMcp15_211*RLcp15_112;
    OPcp15_112 = OPcp15_111+ROcp15_411*qdd[12]+qd[12]*(OMcp15_211*ROcp15_611-OMcp15_311*ROcp15_511);
    OPcp15_212 = OPcp15_211+ROcp15_511*qdd[12]-qd[12]*(OMcp15_111*ROcp15_611-OMcp15_311*ROcp15_411);
    OPcp15_312 = OPcp15_311+ROcp15_611*qdd[12]+qd[12]*(OMcp15_111*ROcp15_511-OMcp15_211*ROcp15_411);
    RLcp15_151 = ROcp15_112*s->dpt[1][17]+ROcp15_411*s->dpt[2][17]+ROcp15_712*s->dpt[3][17];
    RLcp15_251 = ROcp15_212*s->dpt[1][17]+ROcp15_511*s->dpt[2][17]+ROcp15_812*s->dpt[3][17];
    RLcp15_351 = ROcp15_312*s->dpt[1][17]+ROcp15_611*s->dpt[2][17]+ROcp15_912*s->dpt[3][17];
    POcp15_151 = RLcp15_110+RLcp15_111+RLcp15_112+RLcp15_151+RLcp15_17+RLcp15_18+RLcp15_19+q[1];
    POcp15_251 = RLcp15_210+RLcp15_211+RLcp15_212+RLcp15_251+RLcp15_27+RLcp15_28+RLcp15_29+q[2];
    POcp15_351 = RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_351+RLcp15_37+RLcp15_38+RLcp15_39+q[3];
    JTcp15_251_4 = -(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_351+RLcp15_37+RLcp15_38+RLcp15_39);
    JTcp15_351_4 = RLcp15_210+RLcp15_211+RLcp15_212+RLcp15_251+RLcp15_27+RLcp15_28+RLcp15_29;
    JTcp15_151_5 = C4*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_37+RLcp15_38+RLcp15_39)-S4*(RLcp15_210+RLcp15_29)-S4*(
 RLcp15_211+RLcp15_212)-S4*(RLcp15_27+RLcp15_28)-RLcp15_251*S4+RLcp15_351*C4;
    JTcp15_251_5 = S4*(RLcp15_110+RLcp15_111+RLcp15_112+RLcp15_151+RLcp15_17+RLcp15_18+RLcp15_19);
    JTcp15_351_5 = -C4*(RLcp15_110+RLcp15_111+RLcp15_112+RLcp15_151+RLcp15_17+RLcp15_18+RLcp15_19);
    JTcp15_151_6 = ROcp15_85*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_37+RLcp15_38+RLcp15_39)-ROcp15_95*(RLcp15_210+
 RLcp15_29)-ROcp15_95*(RLcp15_211+RLcp15_212)-ROcp15_95*(RLcp15_27+RLcp15_28)-RLcp15_251*ROcp15_95+RLcp15_351*ROcp15_85;
    JTcp15_251_6 = -(RLcp15_351*S5-ROcp15_95*(RLcp15_110+RLcp15_111+RLcp15_112+RLcp15_151+RLcp15_17+RLcp15_18+RLcp15_19)+
 S5*(RLcp15_310+RLcp15_39)+S5*(RLcp15_311+RLcp15_312)+S5*(RLcp15_37+RLcp15_38));
    JTcp15_351_6 = RLcp15_251*S5-ROcp15_85*(RLcp15_110+RLcp15_111+RLcp15_112+RLcp15_151+RLcp15_17+RLcp15_18+RLcp15_19)+S5*
 (RLcp15_210+RLcp15_29)+S5*(RLcp15_211+RLcp15_212)+S5*(RLcp15_27+RLcp15_28);
    JTcp15_151_7 = ROcp15_56*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_351+RLcp15_38+RLcp15_39)-ROcp15_66*(RLcp15_210+
 RLcp15_211)-ROcp15_66*(RLcp15_212+RLcp15_251)-ROcp15_66*(RLcp15_28+RLcp15_29);
    JTcp15_251_7 = -(ROcp15_46*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_351+RLcp15_38+RLcp15_39)-ROcp15_66*(RLcp15_110+
 RLcp15_111)-ROcp15_66*(RLcp15_112+RLcp15_151)-ROcp15_66*(RLcp15_18+RLcp15_19));
    JTcp15_351_7 = ROcp15_46*(RLcp15_210+RLcp15_211+RLcp15_212+RLcp15_251+RLcp15_28+RLcp15_29)-ROcp15_56*(RLcp15_110+
 RLcp15_111)-ROcp15_56*(RLcp15_112+RLcp15_151)-ROcp15_56*(RLcp15_18+RLcp15_19);
    JTcp15_151_8 = ROcp15_27*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_39)-ROcp15_37*(RLcp15_210+RLcp15_29)-ROcp15_37*(
 RLcp15_211+RLcp15_212)-RLcp15_251*ROcp15_37+RLcp15_351*ROcp15_27;
    JTcp15_251_8 = RLcp15_151*ROcp15_37-RLcp15_351*ROcp15_17-ROcp15_17*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_39)+
 ROcp15_37*(RLcp15_110+RLcp15_19)+ROcp15_37*(RLcp15_111+RLcp15_112);
    JTcp15_351_8 = ROcp15_17*(RLcp15_210+RLcp15_211+RLcp15_212+RLcp15_29)-ROcp15_27*(RLcp15_110+RLcp15_19)-ROcp15_27*(
 RLcp15_111+RLcp15_112)-RLcp15_151*ROcp15_27+RLcp15_251*ROcp15_17;
    JTcp15_151_9 = ROcp15_88*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_351)-ROcp15_98*(RLcp15_210+RLcp15_211)-ROcp15_98*(
 RLcp15_212+RLcp15_251);
    JTcp15_251_9 = -(ROcp15_78*(RLcp15_310+RLcp15_311+RLcp15_312+RLcp15_351)-ROcp15_98*(RLcp15_110+RLcp15_111)-ROcp15_98*(
 RLcp15_112+RLcp15_151));
    JTcp15_351_9 = ROcp15_78*(RLcp15_210+RLcp15_211+RLcp15_212+RLcp15_251)-ROcp15_88*(RLcp15_110+RLcp15_111)-ROcp15_88*(
 RLcp15_112+RLcp15_151);
    JTcp15_151_10 = ROcp15_59*(RLcp15_311+RLcp15_312)-ROcp15_69*(RLcp15_211+RLcp15_212)-RLcp15_251*ROcp15_69+RLcp15_351*
 ROcp15_59;
    JTcp15_251_10 = RLcp15_151*ROcp15_69-RLcp15_351*ROcp15_49-ROcp15_49*(RLcp15_311+RLcp15_312)+ROcp15_69*(RLcp15_111+
 RLcp15_112);
    JTcp15_351_10 = ROcp15_49*(RLcp15_211+RLcp15_212)-ROcp15_59*(RLcp15_111+RLcp15_112)-RLcp15_151*ROcp15_59+RLcp15_251*
 ROcp15_49;
    JTcp15_151_11 = ROcp15_210*(RLcp15_312+RLcp15_351)-ROcp15_310*(RLcp15_212+RLcp15_251);
    JTcp15_251_11 = -(ROcp15_110*(RLcp15_312+RLcp15_351)-ROcp15_310*(RLcp15_112+RLcp15_151));
    JTcp15_351_11 = ROcp15_110*(RLcp15_212+RLcp15_251)-ROcp15_210*(RLcp15_112+RLcp15_151);
    JTcp15_151_12 = -(RLcp15_251*ROcp15_611-RLcp15_351*ROcp15_511);
    JTcp15_251_12 = RLcp15_151*ROcp15_611-RLcp15_351*ROcp15_411;
    JTcp15_351_12 = -(RLcp15_151*ROcp15_511-RLcp15_251*ROcp15_411);
    ORcp15_151 = OMcp15_212*RLcp15_351-OMcp15_312*RLcp15_251;
    ORcp15_251 = -(OMcp15_112*RLcp15_351-OMcp15_312*RLcp15_151);
    ORcp15_351 = OMcp15_112*RLcp15_251-OMcp15_212*RLcp15_151;
    VIcp15_151 = ORcp15_110+ORcp15_111+ORcp15_112+ORcp15_151+ORcp15_17+ORcp15_18+ORcp15_19+qd[1];
    VIcp15_251 = ORcp15_210+ORcp15_211+ORcp15_212+ORcp15_251+ORcp15_27+ORcp15_28+ORcp15_29+qd[2];
    VIcp15_351 = ORcp15_310+ORcp15_311+ORcp15_312+ORcp15_351+ORcp15_37+ORcp15_38+ORcp15_39+qd[3];
    ACcp15_151 = qdd[1]+OMcp15_210*ORcp15_311+OMcp15_211*ORcp15_312+OMcp15_212*ORcp15_351+OMcp15_26*ORcp15_37+OMcp15_27*
 ORcp15_38+OMcp15_28*ORcp15_39+OMcp15_29*ORcp15_310-OMcp15_310*ORcp15_211-OMcp15_311*ORcp15_212-OMcp15_312*ORcp15_251-
 OMcp15_36*ORcp15_27-OMcp15_37*ORcp15_28-OMcp15_38*ORcp15_29-OMcp15_39*ORcp15_210+OPcp15_210*RLcp15_311+OPcp15_211*RLcp15_312
 +OPcp15_212*RLcp15_351+OPcp15_26*RLcp15_37+OPcp15_27*RLcp15_38+OPcp15_28*RLcp15_39+OPcp15_29*RLcp15_310-OPcp15_310*
 RLcp15_211-OPcp15_311*RLcp15_212-OPcp15_312*RLcp15_251-OPcp15_36*RLcp15_27-OPcp15_37*RLcp15_28-OPcp15_38*RLcp15_29-OPcp15_39
 *RLcp15_210;
    ACcp15_251 = qdd[2]-OMcp15_110*ORcp15_311-OMcp15_111*ORcp15_312-OMcp15_112*ORcp15_351-OMcp15_16*ORcp15_37-OMcp15_17*
 ORcp15_38-OMcp15_18*ORcp15_39-OMcp15_19*ORcp15_310+OMcp15_310*ORcp15_111+OMcp15_311*ORcp15_112+OMcp15_312*ORcp15_151+
 OMcp15_36*ORcp15_17+OMcp15_37*ORcp15_18+OMcp15_38*ORcp15_19+OMcp15_39*ORcp15_110-OPcp15_110*RLcp15_311-OPcp15_111*RLcp15_312
 -OPcp15_112*RLcp15_351-OPcp15_16*RLcp15_37-OPcp15_17*RLcp15_38-OPcp15_18*RLcp15_39-OPcp15_19*RLcp15_310+OPcp15_310*
 RLcp15_111+OPcp15_311*RLcp15_112+OPcp15_312*RLcp15_151+OPcp15_36*RLcp15_17+OPcp15_37*RLcp15_18+OPcp15_38*RLcp15_19+OPcp15_39
 *RLcp15_110;
    ACcp15_351 = qdd[3]+OMcp15_110*ORcp15_211+OMcp15_111*ORcp15_212+OMcp15_112*ORcp15_251+OMcp15_16*ORcp15_27+OMcp15_17*
 ORcp15_28+OMcp15_18*ORcp15_29+OMcp15_19*ORcp15_210-OMcp15_210*ORcp15_111-OMcp15_211*ORcp15_112-OMcp15_212*ORcp15_151-
 OMcp15_26*ORcp15_17-OMcp15_27*ORcp15_18-OMcp15_28*ORcp15_19-OMcp15_29*ORcp15_110+OPcp15_110*RLcp15_211+OPcp15_111*RLcp15_212
 +OPcp15_112*RLcp15_251+OPcp15_16*RLcp15_27+OPcp15_17*RLcp15_28+OPcp15_18*RLcp15_29+OPcp15_19*RLcp15_210-OPcp15_210*
 RLcp15_111-OPcp15_211*RLcp15_112-OPcp15_212*RLcp15_151-OPcp15_26*RLcp15_17-OPcp15_27*RLcp15_18-OPcp15_28*RLcp15_19-OPcp15_29
 *RLcp15_110;

// = = Block_1_0_0_16_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp15_151;
    sens->P[2] = POcp15_251;
    sens->P[3] = POcp15_351;
    sens->R[1][1] = ROcp15_112;
    sens->R[1][2] = ROcp15_212;
    sens->R[1][3] = ROcp15_312;
    sens->R[2][1] = ROcp15_411;
    sens->R[2][2] = ROcp15_511;
    sens->R[2][3] = ROcp15_611;
    sens->R[3][1] = ROcp15_712;
    sens->R[3][2] = ROcp15_812;
    sens->R[3][3] = ROcp15_912;
    sens->V[1] = VIcp15_151;
    sens->V[2] = VIcp15_251;
    sens->V[3] = VIcp15_351;
    sens->OM[1] = OMcp15_112;
    sens->OM[2] = OMcp15_212;
    sens->OM[3] = OMcp15_312;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp15_151_5;
    sens->J[1][6] = JTcp15_151_6;
    sens->J[1][7] = JTcp15_151_7;
    sens->J[1][8] = JTcp15_151_8;
    sens->J[1][9] = JTcp15_151_9;
    sens->J[1][10] = JTcp15_151_10;
    sens->J[1][11] = JTcp15_151_11;
    sens->J[1][12] = JTcp15_151_12;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp15_251_4;
    sens->J[2][5] = JTcp15_251_5;
    sens->J[2][6] = JTcp15_251_6;
    sens->J[2][7] = JTcp15_251_7;
    sens->J[2][8] = JTcp15_251_8;
    sens->J[2][9] = JTcp15_251_9;
    sens->J[2][10] = JTcp15_251_10;
    sens->J[2][11] = JTcp15_251_11;
    sens->J[2][12] = JTcp15_251_12;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp15_351_4;
    sens->J[3][5] = JTcp15_351_5;
    sens->J[3][6] = JTcp15_351_6;
    sens->J[3][7] = JTcp15_351_7;
    sens->J[3][8] = JTcp15_351_8;
    sens->J[3][9] = JTcp15_351_9;
    sens->J[3][10] = JTcp15_351_10;
    sens->J[3][11] = JTcp15_351_11;
    sens->J[3][12] = JTcp15_351_12;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp15_46;
    sens->J[4][8] = ROcp15_17;
    sens->J[4][9] = ROcp15_78;
    sens->J[4][10] = ROcp15_49;
    sens->J[4][11] = ROcp15_110;
    sens->J[4][12] = ROcp15_411;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp15_85;
    sens->J[5][7] = ROcp15_56;
    sens->J[5][8] = ROcp15_27;
    sens->J[5][9] = ROcp15_88;
    sens->J[5][10] = ROcp15_59;
    sens->J[5][11] = ROcp15_210;
    sens->J[5][12] = ROcp15_511;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp15_95;
    sens->J[6][7] = ROcp15_66;
    sens->J[6][8] = ROcp15_37;
    sens->J[6][9] = ROcp15_98;
    sens->J[6][10] = ROcp15_69;
    sens->J[6][11] = ROcp15_310;
    sens->J[6][12] = ROcp15_611;
    sens->A[1] = ACcp15_151;
    sens->A[2] = ACcp15_251;
    sens->A[3] = ACcp15_351;
    sens->OMP[1] = OPcp15_112;
    sens->OMP[2] = OPcp15_212;
    sens->OMP[3] = OPcp15_312;
 
// 
break;
case 17:
 


// = = Block_1_0_0_17_0_1 = = 
 
// Sensor Kinematics 


    ROcp16_25 = S4*S5;
    ROcp16_35 = -C4*S5;
    ROcp16_85 = -S4*C5;
    ROcp16_95 = C4*C5;
    ROcp16_16 = C5*C6;
    ROcp16_26 = ROcp16_25*C6+C4*S6;
    ROcp16_36 = ROcp16_35*C6+S4*S6;
    ROcp16_46 = -C5*S6;
    ROcp16_56 = -(ROcp16_25*S6-C4*C6);
    ROcp16_66 = -(ROcp16_35*S6-S4*C6);
    OMcp16_25 = qd[5]*C4;
    OMcp16_35 = qd[5]*S4;
    OMcp16_16 = qd[4]+qd[6]*S5;
    OMcp16_26 = OMcp16_25+ROcp16_85*qd[6];
    OMcp16_36 = OMcp16_35+ROcp16_95*qd[6];
    OPcp16_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp16_26 = ROcp16_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp16_35*S5-ROcp16_95*qd[4]);
    OPcp16_36 = ROcp16_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp16_25*S5-ROcp16_85*qd[4]);

// = = Block_1_0_0_17_0_3 = = 
 
// Sensor Kinematics 


    ROcp16_113 = ROcp16_16*C13-S13*S5;
    ROcp16_213 = ROcp16_26*C13-ROcp16_85*S13;
    ROcp16_313 = ROcp16_36*C13-ROcp16_95*S13;
    ROcp16_713 = ROcp16_16*S13+C13*S5;
    ROcp16_813 = ROcp16_26*S13+ROcp16_85*C13;
    ROcp16_913 = ROcp16_36*S13+ROcp16_95*C13;
    RLcp16_113 = s->dpt[1][2]*ROcp16_16+s->dpt[3][2]*S5+ROcp16_46*s->dpt[2][2];
    RLcp16_213 = s->dpt[1][2]*ROcp16_26+s->dpt[3][2]*ROcp16_85+ROcp16_56*s->dpt[2][2];
    RLcp16_313 = s->dpt[1][2]*ROcp16_36+s->dpt[3][2]*ROcp16_95+ROcp16_66*s->dpt[2][2];
    OMcp16_113 = OMcp16_16+ROcp16_46*qd[13];
    OMcp16_213 = OMcp16_26+ROcp16_56*qd[13];
    OMcp16_313 = OMcp16_36+ROcp16_66*qd[13];
    ORcp16_113 = OMcp16_26*RLcp16_313-OMcp16_36*RLcp16_213;
    ORcp16_213 = -(OMcp16_16*RLcp16_313-OMcp16_36*RLcp16_113);
    ORcp16_313 = OMcp16_16*RLcp16_213-OMcp16_26*RLcp16_113;
    OPcp16_113 = OPcp16_16+ROcp16_46*qdd[13]+qd[13]*(OMcp16_26*ROcp16_66-OMcp16_36*ROcp16_56);
    OPcp16_213 = OPcp16_26+ROcp16_56*qdd[13]-qd[13]*(OMcp16_16*ROcp16_66-OMcp16_36*ROcp16_46);
    OPcp16_313 = OPcp16_36+ROcp16_66*qdd[13]+qd[13]*(OMcp16_16*ROcp16_56-OMcp16_26*ROcp16_46);
    RLcp16_152 = s->dpt[1][22]*ROcp16_113+s->dpt[3][22]*ROcp16_713+ROcp16_46*s->dpt[2][22];
    RLcp16_252 = s->dpt[1][22]*ROcp16_213+s->dpt[3][22]*ROcp16_813+ROcp16_56*s->dpt[2][22];
    RLcp16_352 = s->dpt[1][22]*ROcp16_313+s->dpt[3][22]*ROcp16_913+ROcp16_66*s->dpt[2][22];
    POcp16_152 = RLcp16_113+RLcp16_152+q[1];
    POcp16_252 = RLcp16_213+RLcp16_252+q[2];
    POcp16_352 = RLcp16_313+RLcp16_352+q[3];
    JTcp16_252_4 = -(RLcp16_313+RLcp16_352);
    JTcp16_352_4 = RLcp16_213+RLcp16_252;
    JTcp16_152_5 = C4*(RLcp16_313+RLcp16_352)-S4*(RLcp16_213+RLcp16_252);
    JTcp16_252_5 = S4*(RLcp16_113+RLcp16_152);
    JTcp16_352_5 = -C4*(RLcp16_113+RLcp16_152);
    JTcp16_152_6 = ROcp16_85*(RLcp16_313+RLcp16_352)-ROcp16_95*(RLcp16_213+RLcp16_252);
    JTcp16_252_6 = ROcp16_95*(RLcp16_113+RLcp16_152)-S5*(RLcp16_313+RLcp16_352);
    JTcp16_352_6 = -(ROcp16_85*(RLcp16_113+RLcp16_152)-S5*(RLcp16_213+RLcp16_252));
    JTcp16_152_7 = -(RLcp16_252*ROcp16_66-RLcp16_352*ROcp16_56);
    JTcp16_252_7 = RLcp16_152*ROcp16_66-RLcp16_352*ROcp16_46;
    JTcp16_352_7 = -(RLcp16_152*ROcp16_56-RLcp16_252*ROcp16_46);
    ORcp16_152 = OMcp16_213*RLcp16_352-OMcp16_313*RLcp16_252;
    ORcp16_252 = -(OMcp16_113*RLcp16_352-OMcp16_313*RLcp16_152);
    ORcp16_352 = OMcp16_113*RLcp16_252-OMcp16_213*RLcp16_152;
    VIcp16_152 = ORcp16_113+ORcp16_152+qd[1];
    VIcp16_252 = ORcp16_213+ORcp16_252+qd[2];
    VIcp16_352 = ORcp16_313+ORcp16_352+qd[3];
    ACcp16_152 = qdd[1]+OMcp16_213*ORcp16_352+OMcp16_26*ORcp16_313-OMcp16_313*ORcp16_252-OMcp16_36*ORcp16_213+OPcp16_213*
 RLcp16_352+OPcp16_26*RLcp16_313-OPcp16_313*RLcp16_252-OPcp16_36*RLcp16_213;
    ACcp16_252 = qdd[2]-OMcp16_113*ORcp16_352-OMcp16_16*ORcp16_313+OMcp16_313*ORcp16_152+OMcp16_36*ORcp16_113-OPcp16_113*
 RLcp16_352-OPcp16_16*RLcp16_313+OPcp16_313*RLcp16_152+OPcp16_36*RLcp16_113;
    ACcp16_352 = qdd[3]+OMcp16_113*ORcp16_252+OMcp16_16*ORcp16_213-OMcp16_213*ORcp16_152-OMcp16_26*ORcp16_113+OPcp16_113*
 RLcp16_252+OPcp16_16*RLcp16_213-OPcp16_213*RLcp16_152-OPcp16_26*RLcp16_113;

// = = Block_1_0_0_17_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp16_152;
    sens->P[2] = POcp16_252;
    sens->P[3] = POcp16_352;
    sens->R[1][1] = ROcp16_113;
    sens->R[1][2] = ROcp16_213;
    sens->R[1][3] = ROcp16_313;
    sens->R[2][1] = ROcp16_46;
    sens->R[2][2] = ROcp16_56;
    sens->R[2][3] = ROcp16_66;
    sens->R[3][1] = ROcp16_713;
    sens->R[3][2] = ROcp16_813;
    sens->R[3][3] = ROcp16_913;
    sens->V[1] = VIcp16_152;
    sens->V[2] = VIcp16_252;
    sens->V[3] = VIcp16_352;
    sens->OM[1] = OMcp16_113;
    sens->OM[2] = OMcp16_213;
    sens->OM[3] = OMcp16_313;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp16_152_5;
    sens->J[1][6] = JTcp16_152_6;
    sens->J[1][13] = JTcp16_152_7;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp16_252_4;
    sens->J[2][5] = JTcp16_252_5;
    sens->J[2][6] = JTcp16_252_6;
    sens->J[2][13] = JTcp16_252_7;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp16_352_4;
    sens->J[3][5] = JTcp16_352_5;
    sens->J[3][6] = JTcp16_352_6;
    sens->J[3][13] = JTcp16_352_7;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp16_46;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp16_85;
    sens->J[5][13] = ROcp16_56;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp16_95;
    sens->J[6][13] = ROcp16_66;
    sens->A[1] = ACcp16_152;
    sens->A[2] = ACcp16_252;
    sens->A[3] = ACcp16_352;
    sens->OMP[1] = OPcp16_113;
    sens->OMP[2] = OPcp16_213;
    sens->OMP[3] = OPcp16_313;
 
// 
break;
case 18:
 


// = = Block_1_0_0_18_0_1 = = 
 
// Sensor Kinematics 


    ROcp17_25 = S4*S5;
    ROcp17_35 = -C4*S5;
    ROcp17_85 = -S4*C5;
    ROcp17_95 = C4*C5;
    ROcp17_16 = C5*C6;
    ROcp17_26 = ROcp17_25*C6+C4*S6;
    ROcp17_36 = ROcp17_35*C6+S4*S6;
    ROcp17_46 = -C5*S6;
    ROcp17_56 = -(ROcp17_25*S6-C4*C6);
    ROcp17_66 = -(ROcp17_35*S6-S4*C6);
    OMcp17_25 = qd[5]*C4;
    OMcp17_35 = qd[5]*S4;
    OMcp17_16 = qd[4]+qd[6]*S5;
    OMcp17_26 = OMcp17_25+ROcp17_85*qd[6];
    OMcp17_36 = OMcp17_35+ROcp17_95*qd[6];
    OPcp17_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp17_26 = ROcp17_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp17_35*S5-ROcp17_95*qd[4]);
    OPcp17_36 = ROcp17_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp17_25*S5-ROcp17_85*qd[4]);

// = = Block_1_0_0_18_0_3 = = 
 
// Sensor Kinematics 


    ROcp17_113 = ROcp17_16*C13-S13*S5;
    ROcp17_213 = ROcp17_26*C13-ROcp17_85*S13;
    ROcp17_313 = ROcp17_36*C13-ROcp17_95*S13;
    ROcp17_713 = ROcp17_16*S13+C13*S5;
    ROcp17_813 = ROcp17_26*S13+ROcp17_85*C13;
    ROcp17_913 = ROcp17_36*S13+ROcp17_95*C13;
    RLcp17_113 = s->dpt[1][2]*ROcp17_16+s->dpt[3][2]*S5+ROcp17_46*s->dpt[2][2];
    RLcp17_213 = s->dpt[1][2]*ROcp17_26+s->dpt[3][2]*ROcp17_85+ROcp17_56*s->dpt[2][2];
    RLcp17_313 = s->dpt[1][2]*ROcp17_36+s->dpt[3][2]*ROcp17_95+ROcp17_66*s->dpt[2][2];
    OMcp17_113 = OMcp17_16+ROcp17_46*qd[13];
    OMcp17_213 = OMcp17_26+ROcp17_56*qd[13];
    OMcp17_313 = OMcp17_36+ROcp17_66*qd[13];
    ORcp17_113 = OMcp17_26*RLcp17_313-OMcp17_36*RLcp17_213;
    ORcp17_213 = -(OMcp17_16*RLcp17_313-OMcp17_36*RLcp17_113);
    ORcp17_313 = OMcp17_16*RLcp17_213-OMcp17_26*RLcp17_113;
    OPcp17_113 = OPcp17_16+ROcp17_46*qdd[13]+qd[13]*(OMcp17_26*ROcp17_66-OMcp17_36*ROcp17_56);
    OPcp17_213 = OPcp17_26+ROcp17_56*qdd[13]-qd[13]*(OMcp17_16*ROcp17_66-OMcp17_36*ROcp17_46);
    OPcp17_313 = OPcp17_36+ROcp17_66*qdd[13]+qd[13]*(OMcp17_16*ROcp17_56-OMcp17_26*ROcp17_46);
    RLcp17_153 = ROcp17_113*s->dpt[1][23]+ROcp17_46*s->dpt[2][23]+ROcp17_713*s->dpt[3][23];
    RLcp17_253 = ROcp17_213*s->dpt[1][23]+ROcp17_56*s->dpt[2][23]+ROcp17_813*s->dpt[3][23];
    RLcp17_353 = ROcp17_313*s->dpt[1][23]+ROcp17_66*s->dpt[2][23]+ROcp17_913*s->dpt[3][23];
    POcp17_153 = RLcp17_113+RLcp17_153+q[1];
    POcp17_253 = RLcp17_213+RLcp17_253+q[2];
    POcp17_353 = RLcp17_313+RLcp17_353+q[3];
    JTcp17_253_4 = -(RLcp17_313+RLcp17_353);
    JTcp17_353_4 = RLcp17_213+RLcp17_253;
    JTcp17_153_5 = C4*(RLcp17_313+RLcp17_353)-S4*(RLcp17_213+RLcp17_253);
    JTcp17_253_5 = S4*(RLcp17_113+RLcp17_153);
    JTcp17_353_5 = -C4*(RLcp17_113+RLcp17_153);
    JTcp17_153_6 = ROcp17_85*(RLcp17_313+RLcp17_353)-ROcp17_95*(RLcp17_213+RLcp17_253);
    JTcp17_253_6 = ROcp17_95*(RLcp17_113+RLcp17_153)-S5*(RLcp17_313+RLcp17_353);
    JTcp17_353_6 = -(ROcp17_85*(RLcp17_113+RLcp17_153)-S5*(RLcp17_213+RLcp17_253));
    JTcp17_153_7 = -(RLcp17_253*ROcp17_66-RLcp17_353*ROcp17_56);
    JTcp17_253_7 = RLcp17_153*ROcp17_66-RLcp17_353*ROcp17_46;
    JTcp17_353_7 = -(RLcp17_153*ROcp17_56-RLcp17_253*ROcp17_46);
    ORcp17_153 = OMcp17_213*RLcp17_353-OMcp17_313*RLcp17_253;
    ORcp17_253 = -(OMcp17_113*RLcp17_353-OMcp17_313*RLcp17_153);
    ORcp17_353 = OMcp17_113*RLcp17_253-OMcp17_213*RLcp17_153;
    VIcp17_153 = ORcp17_113+ORcp17_153+qd[1];
    VIcp17_253 = ORcp17_213+ORcp17_253+qd[2];
    VIcp17_353 = ORcp17_313+ORcp17_353+qd[3];
    ACcp17_153 = qdd[1]+OMcp17_213*ORcp17_353+OMcp17_26*ORcp17_313-OMcp17_313*ORcp17_253-OMcp17_36*ORcp17_213+OPcp17_213*
 RLcp17_353+OPcp17_26*RLcp17_313-OPcp17_313*RLcp17_253-OPcp17_36*RLcp17_213;
    ACcp17_253 = qdd[2]-OMcp17_113*ORcp17_353-OMcp17_16*ORcp17_313+OMcp17_313*ORcp17_153+OMcp17_36*ORcp17_113-OPcp17_113*
 RLcp17_353-OPcp17_16*RLcp17_313+OPcp17_313*RLcp17_153+OPcp17_36*RLcp17_113;
    ACcp17_353 = qdd[3]+OMcp17_113*ORcp17_253+OMcp17_16*ORcp17_213-OMcp17_213*ORcp17_153-OMcp17_26*ORcp17_113+OPcp17_113*
 RLcp17_253+OPcp17_16*RLcp17_213-OPcp17_213*RLcp17_153-OPcp17_26*RLcp17_113;

// = = Block_1_0_0_18_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp17_153;
    sens->P[2] = POcp17_253;
    sens->P[3] = POcp17_353;
    sens->R[1][1] = ROcp17_113;
    sens->R[1][2] = ROcp17_213;
    sens->R[1][3] = ROcp17_313;
    sens->R[2][1] = ROcp17_46;
    sens->R[2][2] = ROcp17_56;
    sens->R[2][3] = ROcp17_66;
    sens->R[3][1] = ROcp17_713;
    sens->R[3][2] = ROcp17_813;
    sens->R[3][3] = ROcp17_913;
    sens->V[1] = VIcp17_153;
    sens->V[2] = VIcp17_253;
    sens->V[3] = VIcp17_353;
    sens->OM[1] = OMcp17_113;
    sens->OM[2] = OMcp17_213;
    sens->OM[3] = OMcp17_313;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp17_153_5;
    sens->J[1][6] = JTcp17_153_6;
    sens->J[1][13] = JTcp17_153_7;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp17_253_4;
    sens->J[2][5] = JTcp17_253_5;
    sens->J[2][6] = JTcp17_253_6;
    sens->J[2][13] = JTcp17_253_7;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp17_353_4;
    sens->J[3][5] = JTcp17_353_5;
    sens->J[3][6] = JTcp17_353_6;
    sens->J[3][13] = JTcp17_353_7;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp17_46;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp17_85;
    sens->J[5][13] = ROcp17_56;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp17_95;
    sens->J[6][13] = ROcp17_66;
    sens->A[1] = ACcp17_153;
    sens->A[2] = ACcp17_253;
    sens->A[3] = ACcp17_353;
    sens->OMP[1] = OPcp17_113;
    sens->OMP[2] = OPcp17_213;
    sens->OMP[3] = OPcp17_313;
 
// 
break;
case 19:
 


// = = Block_1_0_0_19_0_1 = = 
 
// Sensor Kinematics 


    ROcp18_25 = S4*S5;
    ROcp18_35 = -C4*S5;
    ROcp18_85 = -S4*C5;
    ROcp18_95 = C4*C5;
    ROcp18_16 = C5*C6;
    ROcp18_26 = ROcp18_25*C6+C4*S6;
    ROcp18_36 = ROcp18_35*C6+S4*S6;
    ROcp18_46 = -C5*S6;
    ROcp18_56 = -(ROcp18_25*S6-C4*C6);
    ROcp18_66 = -(ROcp18_35*S6-S4*C6);
    OMcp18_25 = qd[5]*C4;
    OMcp18_35 = qd[5]*S4;
    OMcp18_16 = qd[4]+qd[6]*S5;
    OMcp18_26 = OMcp18_25+ROcp18_85*qd[6];
    OMcp18_36 = OMcp18_35+ROcp18_95*qd[6];
    OPcp18_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp18_26 = ROcp18_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp18_35*S5-ROcp18_95*qd[4]);
    OPcp18_36 = ROcp18_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp18_25*S5-ROcp18_85*qd[4]);

// = = Block_1_0_0_19_0_3 = = 
 
// Sensor Kinematics 


    ROcp18_113 = ROcp18_16*C13-S13*S5;
    ROcp18_213 = ROcp18_26*C13-ROcp18_85*S13;
    ROcp18_313 = ROcp18_36*C13-ROcp18_95*S13;
    ROcp18_713 = ROcp18_16*S13+C13*S5;
    ROcp18_813 = ROcp18_26*S13+ROcp18_85*C13;
    ROcp18_913 = ROcp18_36*S13+ROcp18_95*C13;
    ROcp18_414 = ROcp18_46*C14+ROcp18_713*S14;
    ROcp18_514 = ROcp18_56*C14+ROcp18_813*S14;
    ROcp18_614 = ROcp18_66*C14+ROcp18_913*S14;
    ROcp18_714 = -(ROcp18_46*S14-ROcp18_713*C14);
    ROcp18_814 = -(ROcp18_56*S14-ROcp18_813*C14);
    ROcp18_914 = -(ROcp18_66*S14-ROcp18_913*C14);
    RLcp18_113 = s->dpt[1][2]*ROcp18_16+s->dpt[3][2]*S5+ROcp18_46*s->dpt[2][2];
    RLcp18_213 = s->dpt[1][2]*ROcp18_26+s->dpt[3][2]*ROcp18_85+ROcp18_56*s->dpt[2][2];
    RLcp18_313 = s->dpt[1][2]*ROcp18_36+s->dpt[3][2]*ROcp18_95+ROcp18_66*s->dpt[2][2];
    OMcp18_113 = OMcp18_16+ROcp18_46*qd[13];
    OMcp18_213 = OMcp18_26+ROcp18_56*qd[13];
    OMcp18_313 = OMcp18_36+ROcp18_66*qd[13];
    ORcp18_113 = OMcp18_26*RLcp18_313-OMcp18_36*RLcp18_213;
    ORcp18_213 = -(OMcp18_16*RLcp18_313-OMcp18_36*RLcp18_113);
    ORcp18_313 = OMcp18_16*RLcp18_213-OMcp18_26*RLcp18_113;
    OPcp18_113 = OPcp18_16+ROcp18_46*qdd[13]+qd[13]*(OMcp18_26*ROcp18_66-OMcp18_36*ROcp18_56);
    OPcp18_213 = OPcp18_26+ROcp18_56*qdd[13]-qd[13]*(OMcp18_16*ROcp18_66-OMcp18_36*ROcp18_46);
    OPcp18_313 = OPcp18_36+ROcp18_66*qdd[13]+qd[13]*(OMcp18_16*ROcp18_56-OMcp18_26*ROcp18_46);
    RLcp18_114 = s->dpt[1][22]*ROcp18_113+s->dpt[3][22]*ROcp18_713+ROcp18_46*s->dpt[2][22];
    RLcp18_214 = s->dpt[1][22]*ROcp18_213+s->dpt[3][22]*ROcp18_813+ROcp18_56*s->dpt[2][22];
    RLcp18_314 = s->dpt[1][22]*ROcp18_313+s->dpt[3][22]*ROcp18_913+ROcp18_66*s->dpt[2][22];
    OMcp18_114 = OMcp18_113+ROcp18_113*qd[14];
    OMcp18_214 = OMcp18_213+ROcp18_213*qd[14];
    OMcp18_314 = OMcp18_313+ROcp18_313*qd[14];
    ORcp18_114 = OMcp18_213*RLcp18_314-OMcp18_313*RLcp18_214;
    ORcp18_214 = -(OMcp18_113*RLcp18_314-OMcp18_313*RLcp18_114);
    ORcp18_314 = OMcp18_113*RLcp18_214-OMcp18_213*RLcp18_114;
    OPcp18_114 = OPcp18_113+ROcp18_113*qdd[14]+qd[14]*(OMcp18_213*ROcp18_313-OMcp18_313*ROcp18_213);
    OPcp18_214 = OPcp18_213+ROcp18_213*qdd[14]-qd[14]*(OMcp18_113*ROcp18_313-OMcp18_313*ROcp18_113);
    OPcp18_314 = OPcp18_313+ROcp18_313*qdd[14]+qd[14]*(OMcp18_113*ROcp18_213-OMcp18_213*ROcp18_113);
    RLcp18_154 = s->dpt[1][24]*ROcp18_113+s->dpt[2][24]*ROcp18_414+ROcp18_714*s->dpt[3][24];
    RLcp18_254 = s->dpt[1][24]*ROcp18_213+s->dpt[2][24]*ROcp18_514+ROcp18_814*s->dpt[3][24];
    RLcp18_354 = s->dpt[1][24]*ROcp18_313+s->dpt[2][24]*ROcp18_614+ROcp18_914*s->dpt[3][24];
    POcp18_154 = RLcp18_113+RLcp18_114+RLcp18_154+q[1];
    POcp18_254 = RLcp18_213+RLcp18_214+RLcp18_254+q[2];
    POcp18_354 = RLcp18_313+RLcp18_314+RLcp18_354+q[3];
    JTcp18_254_4 = -(RLcp18_313+RLcp18_314+RLcp18_354);
    JTcp18_354_4 = RLcp18_213+RLcp18_214+RLcp18_254;
    JTcp18_154_5 = C4*(RLcp18_313+RLcp18_314)-S4*(RLcp18_213+RLcp18_214)-RLcp18_254*S4+RLcp18_354*C4;
    JTcp18_254_5 = S4*(RLcp18_113+RLcp18_114+RLcp18_154);
    JTcp18_354_5 = -C4*(RLcp18_113+RLcp18_114+RLcp18_154);
    JTcp18_154_6 = ROcp18_85*(RLcp18_313+RLcp18_314)-ROcp18_95*(RLcp18_213+RLcp18_214)-RLcp18_254*ROcp18_95+RLcp18_354*
 ROcp18_85;
    JTcp18_254_6 = -(RLcp18_354*S5-ROcp18_95*(RLcp18_113+RLcp18_114+RLcp18_154)+S5*(RLcp18_313+RLcp18_314));
    JTcp18_354_6 = RLcp18_254*S5-ROcp18_85*(RLcp18_113+RLcp18_114+RLcp18_154)+S5*(RLcp18_213+RLcp18_214);
    JTcp18_154_7 = ROcp18_56*(RLcp18_314+RLcp18_354)-ROcp18_66*(RLcp18_214+RLcp18_254);
    JTcp18_254_7 = -(ROcp18_46*(RLcp18_314+RLcp18_354)-ROcp18_66*(RLcp18_114+RLcp18_154));
    JTcp18_354_7 = ROcp18_46*(RLcp18_214+RLcp18_254)-ROcp18_56*(RLcp18_114+RLcp18_154);
    JTcp18_154_8 = -(RLcp18_254*ROcp18_313-RLcp18_354*ROcp18_213);
    JTcp18_254_8 = RLcp18_154*ROcp18_313-RLcp18_354*ROcp18_113;
    JTcp18_354_8 = -(RLcp18_154*ROcp18_213-RLcp18_254*ROcp18_113);
    ORcp18_154 = OMcp18_214*RLcp18_354-OMcp18_314*RLcp18_254;
    ORcp18_254 = -(OMcp18_114*RLcp18_354-OMcp18_314*RLcp18_154);
    ORcp18_354 = OMcp18_114*RLcp18_254-OMcp18_214*RLcp18_154;
    VIcp18_154 = ORcp18_113+ORcp18_114+ORcp18_154+qd[1];
    VIcp18_254 = ORcp18_213+ORcp18_214+ORcp18_254+qd[2];
    VIcp18_354 = ORcp18_313+ORcp18_314+ORcp18_354+qd[3];
    ACcp18_154 = qdd[1]+OMcp18_213*ORcp18_314+OMcp18_214*ORcp18_354+OMcp18_26*ORcp18_313-OMcp18_313*ORcp18_214-OMcp18_314*
 ORcp18_254-OMcp18_36*ORcp18_213+OPcp18_213*RLcp18_314+OPcp18_214*RLcp18_354+OPcp18_26*RLcp18_313-OPcp18_313*RLcp18_214-
 OPcp18_314*RLcp18_254-OPcp18_36*RLcp18_213;
    ACcp18_254 = qdd[2]-OMcp18_113*ORcp18_314-OMcp18_114*ORcp18_354-OMcp18_16*ORcp18_313+OMcp18_313*ORcp18_114+OMcp18_314*
 ORcp18_154+OMcp18_36*ORcp18_113-OPcp18_113*RLcp18_314-OPcp18_114*RLcp18_354-OPcp18_16*RLcp18_313+OPcp18_313*RLcp18_114+
 OPcp18_314*RLcp18_154+OPcp18_36*RLcp18_113;
    ACcp18_354 = qdd[3]+OMcp18_113*ORcp18_214+OMcp18_114*ORcp18_254+OMcp18_16*ORcp18_213-OMcp18_213*ORcp18_114-OMcp18_214*
 ORcp18_154-OMcp18_26*ORcp18_113+OPcp18_113*RLcp18_214+OPcp18_114*RLcp18_254+OPcp18_16*RLcp18_213-OPcp18_213*RLcp18_114-
 OPcp18_214*RLcp18_154-OPcp18_26*RLcp18_113;

// = = Block_1_0_0_19_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp18_154;
    sens->P[2] = POcp18_254;
    sens->P[3] = POcp18_354;
    sens->R[1][1] = ROcp18_113;
    sens->R[1][2] = ROcp18_213;
    sens->R[1][3] = ROcp18_313;
    sens->R[2][1] = ROcp18_414;
    sens->R[2][2] = ROcp18_514;
    sens->R[2][3] = ROcp18_614;
    sens->R[3][1] = ROcp18_714;
    sens->R[3][2] = ROcp18_814;
    sens->R[3][3] = ROcp18_914;
    sens->V[1] = VIcp18_154;
    sens->V[2] = VIcp18_254;
    sens->V[3] = VIcp18_354;
    sens->OM[1] = OMcp18_114;
    sens->OM[2] = OMcp18_214;
    sens->OM[3] = OMcp18_314;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp18_154_5;
    sens->J[1][6] = JTcp18_154_6;
    sens->J[1][13] = JTcp18_154_7;
    sens->J[1][14] = JTcp18_154_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp18_254_4;
    sens->J[2][5] = JTcp18_254_5;
    sens->J[2][6] = JTcp18_254_6;
    sens->J[2][13] = JTcp18_254_7;
    sens->J[2][14] = JTcp18_254_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp18_354_4;
    sens->J[3][5] = JTcp18_354_5;
    sens->J[3][6] = JTcp18_354_6;
    sens->J[3][13] = JTcp18_354_7;
    sens->J[3][14] = JTcp18_354_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp18_46;
    sens->J[4][14] = ROcp18_113;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp18_85;
    sens->J[5][13] = ROcp18_56;
    sens->J[5][14] = ROcp18_213;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp18_95;
    sens->J[6][13] = ROcp18_66;
    sens->J[6][14] = ROcp18_313;
    sens->A[1] = ACcp18_154;
    sens->A[2] = ACcp18_254;
    sens->A[3] = ACcp18_354;
    sens->OMP[1] = OPcp18_114;
    sens->OMP[2] = OPcp18_214;
    sens->OMP[3] = OPcp18_314;
 
// 
break;
case 20:
 


// = = Block_1_0_0_20_0_1 = = 
 
// Sensor Kinematics 


    ROcp19_25 = S4*S5;
    ROcp19_35 = -C4*S5;
    ROcp19_85 = -S4*C5;
    ROcp19_95 = C4*C5;
    ROcp19_16 = C5*C6;
    ROcp19_26 = ROcp19_25*C6+C4*S6;
    ROcp19_36 = ROcp19_35*C6+S4*S6;
    ROcp19_46 = -C5*S6;
    ROcp19_56 = -(ROcp19_25*S6-C4*C6);
    ROcp19_66 = -(ROcp19_35*S6-S4*C6);
    OMcp19_25 = qd[5]*C4;
    OMcp19_35 = qd[5]*S4;
    OMcp19_16 = qd[4]+qd[6]*S5;
    OMcp19_26 = OMcp19_25+ROcp19_85*qd[6];
    OMcp19_36 = OMcp19_35+ROcp19_95*qd[6];
    OPcp19_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp19_26 = ROcp19_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp19_35*S5-ROcp19_95*qd[4]);
    OPcp19_36 = ROcp19_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp19_25*S5-ROcp19_85*qd[4]);

// = = Block_1_0_0_20_0_3 = = 
 
// Sensor Kinematics 


    ROcp19_113 = ROcp19_16*C13-S13*S5;
    ROcp19_213 = ROcp19_26*C13-ROcp19_85*S13;
    ROcp19_313 = ROcp19_36*C13-ROcp19_95*S13;
    ROcp19_713 = ROcp19_16*S13+C13*S5;
    ROcp19_813 = ROcp19_26*S13+ROcp19_85*C13;
    ROcp19_913 = ROcp19_36*S13+ROcp19_95*C13;
    ROcp19_414 = ROcp19_46*C14+ROcp19_713*S14;
    ROcp19_514 = ROcp19_56*C14+ROcp19_813*S14;
    ROcp19_614 = ROcp19_66*C14+ROcp19_913*S14;
    ROcp19_714 = -(ROcp19_46*S14-ROcp19_713*C14);
    ROcp19_814 = -(ROcp19_56*S14-ROcp19_813*C14);
    ROcp19_914 = -(ROcp19_66*S14-ROcp19_913*C14);
    RLcp19_113 = s->dpt[1][2]*ROcp19_16+s->dpt[3][2]*S5+ROcp19_46*s->dpt[2][2];
    RLcp19_213 = s->dpt[1][2]*ROcp19_26+s->dpt[3][2]*ROcp19_85+ROcp19_56*s->dpt[2][2];
    RLcp19_313 = s->dpt[1][2]*ROcp19_36+s->dpt[3][2]*ROcp19_95+ROcp19_66*s->dpt[2][2];
    OMcp19_113 = OMcp19_16+ROcp19_46*qd[13];
    OMcp19_213 = OMcp19_26+ROcp19_56*qd[13];
    OMcp19_313 = OMcp19_36+ROcp19_66*qd[13];
    ORcp19_113 = OMcp19_26*RLcp19_313-OMcp19_36*RLcp19_213;
    ORcp19_213 = -(OMcp19_16*RLcp19_313-OMcp19_36*RLcp19_113);
    ORcp19_313 = OMcp19_16*RLcp19_213-OMcp19_26*RLcp19_113;
    OPcp19_113 = OPcp19_16+ROcp19_46*qdd[13]+qd[13]*(OMcp19_26*ROcp19_66-OMcp19_36*ROcp19_56);
    OPcp19_213 = OPcp19_26+ROcp19_56*qdd[13]-qd[13]*(OMcp19_16*ROcp19_66-OMcp19_36*ROcp19_46);
    OPcp19_313 = OPcp19_36+ROcp19_66*qdd[13]+qd[13]*(OMcp19_16*ROcp19_56-OMcp19_26*ROcp19_46);
    RLcp19_114 = s->dpt[1][22]*ROcp19_113+s->dpt[3][22]*ROcp19_713+ROcp19_46*s->dpt[2][22];
    RLcp19_214 = s->dpt[1][22]*ROcp19_213+s->dpt[3][22]*ROcp19_813+ROcp19_56*s->dpt[2][22];
    RLcp19_314 = s->dpt[1][22]*ROcp19_313+s->dpt[3][22]*ROcp19_913+ROcp19_66*s->dpt[2][22];
    OMcp19_114 = OMcp19_113+ROcp19_113*qd[14];
    OMcp19_214 = OMcp19_213+ROcp19_213*qd[14];
    OMcp19_314 = OMcp19_313+ROcp19_313*qd[14];
    ORcp19_114 = OMcp19_213*RLcp19_314-OMcp19_313*RLcp19_214;
    ORcp19_214 = -(OMcp19_113*RLcp19_314-OMcp19_313*RLcp19_114);
    ORcp19_314 = OMcp19_113*RLcp19_214-OMcp19_213*RLcp19_114;
    OPcp19_114 = OPcp19_113+ROcp19_113*qdd[14]+qd[14]*(OMcp19_213*ROcp19_313-OMcp19_313*ROcp19_213);
    OPcp19_214 = OPcp19_213+ROcp19_213*qdd[14]-qd[14]*(OMcp19_113*ROcp19_313-OMcp19_313*ROcp19_113);
    OPcp19_314 = OPcp19_313+ROcp19_313*qdd[14]+qd[14]*(OMcp19_113*ROcp19_213-OMcp19_213*ROcp19_113);
    RLcp19_155 = ROcp19_113*s->dpt[1][25]+ROcp19_414*s->dpt[2][25]+ROcp19_714*s->dpt[3][25];
    RLcp19_255 = ROcp19_213*s->dpt[1][25]+ROcp19_514*s->dpt[2][25]+ROcp19_814*s->dpt[3][25];
    RLcp19_355 = ROcp19_313*s->dpt[1][25]+ROcp19_614*s->dpt[2][25]+ROcp19_914*s->dpt[3][25];
    POcp19_155 = RLcp19_113+RLcp19_114+RLcp19_155+q[1];
    POcp19_255 = RLcp19_213+RLcp19_214+RLcp19_255+q[2];
    POcp19_355 = RLcp19_313+RLcp19_314+RLcp19_355+q[3];
    JTcp19_255_4 = -(RLcp19_313+RLcp19_314+RLcp19_355);
    JTcp19_355_4 = RLcp19_213+RLcp19_214+RLcp19_255;
    JTcp19_155_5 = C4*(RLcp19_313+RLcp19_314)-S4*(RLcp19_213+RLcp19_214)-RLcp19_255*S4+RLcp19_355*C4;
    JTcp19_255_5 = S4*(RLcp19_113+RLcp19_114+RLcp19_155);
    JTcp19_355_5 = -C4*(RLcp19_113+RLcp19_114+RLcp19_155);
    JTcp19_155_6 = ROcp19_85*(RLcp19_313+RLcp19_314)-ROcp19_95*(RLcp19_213+RLcp19_214)-RLcp19_255*ROcp19_95+RLcp19_355*
 ROcp19_85;
    JTcp19_255_6 = -(RLcp19_355*S5-ROcp19_95*(RLcp19_113+RLcp19_114+RLcp19_155)+S5*(RLcp19_313+RLcp19_314));
    JTcp19_355_6 = RLcp19_255*S5-ROcp19_85*(RLcp19_113+RLcp19_114+RLcp19_155)+S5*(RLcp19_213+RLcp19_214);
    JTcp19_155_7 = ROcp19_56*(RLcp19_314+RLcp19_355)-ROcp19_66*(RLcp19_214+RLcp19_255);
    JTcp19_255_7 = -(ROcp19_46*(RLcp19_314+RLcp19_355)-ROcp19_66*(RLcp19_114+RLcp19_155));
    JTcp19_355_7 = ROcp19_46*(RLcp19_214+RLcp19_255)-ROcp19_56*(RLcp19_114+RLcp19_155);
    JTcp19_155_8 = -(RLcp19_255*ROcp19_313-RLcp19_355*ROcp19_213);
    JTcp19_255_8 = RLcp19_155*ROcp19_313-RLcp19_355*ROcp19_113;
    JTcp19_355_8 = -(RLcp19_155*ROcp19_213-RLcp19_255*ROcp19_113);
    ORcp19_155 = OMcp19_214*RLcp19_355-OMcp19_314*RLcp19_255;
    ORcp19_255 = -(OMcp19_114*RLcp19_355-OMcp19_314*RLcp19_155);
    ORcp19_355 = OMcp19_114*RLcp19_255-OMcp19_214*RLcp19_155;
    VIcp19_155 = ORcp19_113+ORcp19_114+ORcp19_155+qd[1];
    VIcp19_255 = ORcp19_213+ORcp19_214+ORcp19_255+qd[2];
    VIcp19_355 = ORcp19_313+ORcp19_314+ORcp19_355+qd[3];
    ACcp19_155 = qdd[1]+OMcp19_213*ORcp19_314+OMcp19_214*ORcp19_355+OMcp19_26*ORcp19_313-OMcp19_313*ORcp19_214-OMcp19_314*
 ORcp19_255-OMcp19_36*ORcp19_213+OPcp19_213*RLcp19_314+OPcp19_214*RLcp19_355+OPcp19_26*RLcp19_313-OPcp19_313*RLcp19_214-
 OPcp19_314*RLcp19_255-OPcp19_36*RLcp19_213;
    ACcp19_255 = qdd[2]-OMcp19_113*ORcp19_314-OMcp19_114*ORcp19_355-OMcp19_16*ORcp19_313+OMcp19_313*ORcp19_114+OMcp19_314*
 ORcp19_155+OMcp19_36*ORcp19_113-OPcp19_113*RLcp19_314-OPcp19_114*RLcp19_355-OPcp19_16*RLcp19_313+OPcp19_313*RLcp19_114+
 OPcp19_314*RLcp19_155+OPcp19_36*RLcp19_113;
    ACcp19_355 = qdd[3]+OMcp19_113*ORcp19_214+OMcp19_114*ORcp19_255+OMcp19_16*ORcp19_213-OMcp19_213*ORcp19_114-OMcp19_214*
 ORcp19_155-OMcp19_26*ORcp19_113+OPcp19_113*RLcp19_214+OPcp19_114*RLcp19_255+OPcp19_16*RLcp19_213-OPcp19_213*RLcp19_114-
 OPcp19_214*RLcp19_155-OPcp19_26*RLcp19_113;

// = = Block_1_0_0_20_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp19_155;
    sens->P[2] = POcp19_255;
    sens->P[3] = POcp19_355;
    sens->R[1][1] = ROcp19_113;
    sens->R[1][2] = ROcp19_213;
    sens->R[1][3] = ROcp19_313;
    sens->R[2][1] = ROcp19_414;
    sens->R[2][2] = ROcp19_514;
    sens->R[2][3] = ROcp19_614;
    sens->R[3][1] = ROcp19_714;
    sens->R[3][2] = ROcp19_814;
    sens->R[3][3] = ROcp19_914;
    sens->V[1] = VIcp19_155;
    sens->V[2] = VIcp19_255;
    sens->V[3] = VIcp19_355;
    sens->OM[1] = OMcp19_114;
    sens->OM[2] = OMcp19_214;
    sens->OM[3] = OMcp19_314;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp19_155_5;
    sens->J[1][6] = JTcp19_155_6;
    sens->J[1][13] = JTcp19_155_7;
    sens->J[1][14] = JTcp19_155_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp19_255_4;
    sens->J[2][5] = JTcp19_255_5;
    sens->J[2][6] = JTcp19_255_6;
    sens->J[2][13] = JTcp19_255_7;
    sens->J[2][14] = JTcp19_255_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp19_355_4;
    sens->J[3][5] = JTcp19_355_5;
    sens->J[3][6] = JTcp19_355_6;
    sens->J[3][13] = JTcp19_355_7;
    sens->J[3][14] = JTcp19_355_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp19_46;
    sens->J[4][14] = ROcp19_113;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp19_85;
    sens->J[5][13] = ROcp19_56;
    sens->J[5][14] = ROcp19_213;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp19_95;
    sens->J[6][13] = ROcp19_66;
    sens->J[6][14] = ROcp19_313;
    sens->A[1] = ACcp19_155;
    sens->A[2] = ACcp19_255;
    sens->A[3] = ACcp19_355;
    sens->OMP[1] = OPcp19_114;
    sens->OMP[2] = OPcp19_214;
    sens->OMP[3] = OPcp19_314;
 
// 
break;
case 21:
 


// = = Block_1_0_0_21_0_1 = = 
 
// Sensor Kinematics 


    ROcp20_25 = S4*S5;
    ROcp20_35 = -C4*S5;
    ROcp20_85 = -S4*C5;
    ROcp20_95 = C4*C5;
    ROcp20_16 = C5*C6;
    ROcp20_26 = ROcp20_25*C6+C4*S6;
    ROcp20_36 = ROcp20_35*C6+S4*S6;
    ROcp20_46 = -C5*S6;
    ROcp20_56 = -(ROcp20_25*S6-C4*C6);
    ROcp20_66 = -(ROcp20_35*S6-S4*C6);
    OMcp20_25 = qd[5]*C4;
    OMcp20_35 = qd[5]*S4;
    OMcp20_16 = qd[4]+qd[6]*S5;
    OMcp20_26 = OMcp20_25+ROcp20_85*qd[6];
    OMcp20_36 = OMcp20_35+ROcp20_95*qd[6];
    OPcp20_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp20_26 = ROcp20_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp20_35*S5-ROcp20_95*qd[4]);
    OPcp20_36 = ROcp20_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp20_25*S5-ROcp20_85*qd[4]);

// = = Block_1_0_0_21_0_3 = = 
 
// Sensor Kinematics 


    ROcp20_113 = ROcp20_16*C13-S13*S5;
    ROcp20_213 = ROcp20_26*C13-ROcp20_85*S13;
    ROcp20_313 = ROcp20_36*C13-ROcp20_95*S13;
    ROcp20_713 = ROcp20_16*S13+C13*S5;
    ROcp20_813 = ROcp20_26*S13+ROcp20_85*C13;
    ROcp20_913 = ROcp20_36*S13+ROcp20_95*C13;
    ROcp20_414 = ROcp20_46*C14+ROcp20_713*S14;
    ROcp20_514 = ROcp20_56*C14+ROcp20_813*S14;
    ROcp20_614 = ROcp20_66*C14+ROcp20_913*S14;
    ROcp20_714 = -(ROcp20_46*S14-ROcp20_713*C14);
    ROcp20_814 = -(ROcp20_56*S14-ROcp20_813*C14);
    ROcp20_914 = -(ROcp20_66*S14-ROcp20_913*C14);
    ROcp20_115 = ROcp20_113*C15+ROcp20_414*S15;
    ROcp20_215 = ROcp20_213*C15+ROcp20_514*S15;
    ROcp20_315 = ROcp20_313*C15+ROcp20_614*S15;
    ROcp20_415 = -(ROcp20_113*S15-ROcp20_414*C15);
    ROcp20_515 = -(ROcp20_213*S15-ROcp20_514*C15);
    ROcp20_615 = -(ROcp20_313*S15-ROcp20_614*C15);
    RLcp20_113 = s->dpt[1][2]*ROcp20_16+s->dpt[3][2]*S5+ROcp20_46*s->dpt[2][2];
    RLcp20_213 = s->dpt[1][2]*ROcp20_26+s->dpt[3][2]*ROcp20_85+ROcp20_56*s->dpt[2][2];
    RLcp20_313 = s->dpt[1][2]*ROcp20_36+s->dpt[3][2]*ROcp20_95+ROcp20_66*s->dpt[2][2];
    OMcp20_113 = OMcp20_16+ROcp20_46*qd[13];
    OMcp20_213 = OMcp20_26+ROcp20_56*qd[13];
    OMcp20_313 = OMcp20_36+ROcp20_66*qd[13];
    ORcp20_113 = OMcp20_26*RLcp20_313-OMcp20_36*RLcp20_213;
    ORcp20_213 = -(OMcp20_16*RLcp20_313-OMcp20_36*RLcp20_113);
    ORcp20_313 = OMcp20_16*RLcp20_213-OMcp20_26*RLcp20_113;
    OPcp20_113 = OPcp20_16+ROcp20_46*qdd[13]+qd[13]*(OMcp20_26*ROcp20_66-OMcp20_36*ROcp20_56);
    OPcp20_213 = OPcp20_26+ROcp20_56*qdd[13]-qd[13]*(OMcp20_16*ROcp20_66-OMcp20_36*ROcp20_46);
    OPcp20_313 = OPcp20_36+ROcp20_66*qdd[13]+qd[13]*(OMcp20_16*ROcp20_56-OMcp20_26*ROcp20_46);
    RLcp20_114 = s->dpt[1][22]*ROcp20_113+s->dpt[3][22]*ROcp20_713+ROcp20_46*s->dpt[2][22];
    RLcp20_214 = s->dpt[1][22]*ROcp20_213+s->dpt[3][22]*ROcp20_813+ROcp20_56*s->dpt[2][22];
    RLcp20_314 = s->dpt[1][22]*ROcp20_313+s->dpt[3][22]*ROcp20_913+ROcp20_66*s->dpt[2][22];
    OMcp20_114 = OMcp20_113+ROcp20_113*qd[14];
    OMcp20_214 = OMcp20_213+ROcp20_213*qd[14];
    OMcp20_314 = OMcp20_313+ROcp20_313*qd[14];
    ORcp20_114 = OMcp20_213*RLcp20_314-OMcp20_313*RLcp20_214;
    ORcp20_214 = -(OMcp20_113*RLcp20_314-OMcp20_313*RLcp20_114);
    ORcp20_314 = OMcp20_113*RLcp20_214-OMcp20_213*RLcp20_114;
    OPcp20_114 = OPcp20_113+ROcp20_113*qdd[14]+qd[14]*(OMcp20_213*ROcp20_313-OMcp20_313*ROcp20_213);
    OPcp20_214 = OPcp20_213+ROcp20_213*qdd[14]-qd[14]*(OMcp20_113*ROcp20_313-OMcp20_313*ROcp20_113);
    OPcp20_314 = OPcp20_313+ROcp20_313*qdd[14]+qd[14]*(OMcp20_113*ROcp20_213-OMcp20_213*ROcp20_113);
    RLcp20_115 = s->dpt[1][24]*ROcp20_113+s->dpt[2][24]*ROcp20_414+ROcp20_714*s->dpt[3][24];
    RLcp20_215 = s->dpt[1][24]*ROcp20_213+s->dpt[2][24]*ROcp20_514+ROcp20_814*s->dpt[3][24];
    RLcp20_315 = s->dpt[1][24]*ROcp20_313+s->dpt[2][24]*ROcp20_614+ROcp20_914*s->dpt[3][24];
    OMcp20_115 = OMcp20_114+ROcp20_714*qd[15];
    OMcp20_215 = OMcp20_214+ROcp20_814*qd[15];
    OMcp20_315 = OMcp20_314+ROcp20_914*qd[15];
    ORcp20_115 = OMcp20_214*RLcp20_315-OMcp20_314*RLcp20_215;
    ORcp20_215 = -(OMcp20_114*RLcp20_315-OMcp20_314*RLcp20_115);
    ORcp20_315 = OMcp20_114*RLcp20_215-OMcp20_214*RLcp20_115;
    OPcp20_115 = OPcp20_114+ROcp20_714*qdd[15]+qd[15]*(OMcp20_214*ROcp20_914-OMcp20_314*ROcp20_814);
    OPcp20_215 = OPcp20_214+ROcp20_814*qdd[15]-qd[15]*(OMcp20_114*ROcp20_914-OMcp20_314*ROcp20_714);
    OPcp20_315 = OPcp20_314+ROcp20_914*qdd[15]+qd[15]*(OMcp20_114*ROcp20_814-OMcp20_214*ROcp20_714);
    RLcp20_156 = s->dpt[1][26]*ROcp20_115+s->dpt[2][26]*ROcp20_415+ROcp20_714*s->dpt[3][26];
    RLcp20_256 = s->dpt[1][26]*ROcp20_215+s->dpt[2][26]*ROcp20_515+ROcp20_814*s->dpt[3][26];
    RLcp20_356 = s->dpt[1][26]*ROcp20_315+s->dpt[2][26]*ROcp20_615+ROcp20_914*s->dpt[3][26];
    POcp20_156 = RLcp20_113+RLcp20_114+RLcp20_115+RLcp20_156+q[1];
    POcp20_256 = RLcp20_213+RLcp20_214+RLcp20_215+RLcp20_256+q[2];
    POcp20_356 = RLcp20_313+RLcp20_314+RLcp20_315+RLcp20_356+q[3];
    JTcp20_256_4 = -(RLcp20_313+RLcp20_314+RLcp20_315+RLcp20_356);
    JTcp20_356_4 = RLcp20_213+RLcp20_214+RLcp20_215+RLcp20_256;
    JTcp20_156_5 = C4*(RLcp20_313+RLcp20_314+RLcp20_315+RLcp20_356)-S4*(RLcp20_213+RLcp20_214)-S4*(RLcp20_215+RLcp20_256);
    JTcp20_256_5 = S4*(RLcp20_113+RLcp20_114+RLcp20_115+RLcp20_156);
    JTcp20_356_5 = -C4*(RLcp20_113+RLcp20_114+RLcp20_115+RLcp20_156);
    JTcp20_156_6 = ROcp20_85*(RLcp20_313+RLcp20_314+RLcp20_315+RLcp20_356)-ROcp20_95*(RLcp20_213+RLcp20_214)-ROcp20_95*(
 RLcp20_215+RLcp20_256);
    JTcp20_256_6 = RLcp20_156*ROcp20_95-RLcp20_315*S5-RLcp20_356*S5+ROcp20_95*(RLcp20_113+RLcp20_114+RLcp20_115)-S5*(
 RLcp20_313+RLcp20_314);
    JTcp20_356_6 = RLcp20_215*S5-ROcp20_85*(RLcp20_113+RLcp20_114+RLcp20_115)+S5*(RLcp20_213+RLcp20_214)-RLcp20_156*
 ROcp20_85+RLcp20_256*S5;
    JTcp20_156_7 = ROcp20_56*(RLcp20_314+RLcp20_315)-ROcp20_66*(RLcp20_214+RLcp20_215)-RLcp20_256*ROcp20_66+RLcp20_356*
 ROcp20_56;
    JTcp20_256_7 = RLcp20_156*ROcp20_66-RLcp20_356*ROcp20_46-ROcp20_46*(RLcp20_314+RLcp20_315)+ROcp20_66*(RLcp20_114+
 RLcp20_115);
    JTcp20_356_7 = ROcp20_46*(RLcp20_214+RLcp20_215)-ROcp20_56*(RLcp20_114+RLcp20_115)-RLcp20_156*ROcp20_56+RLcp20_256*
 ROcp20_46;
    JTcp20_156_8 = ROcp20_213*(RLcp20_315+RLcp20_356)-ROcp20_313*(RLcp20_215+RLcp20_256);
    JTcp20_256_8 = -(ROcp20_113*(RLcp20_315+RLcp20_356)-ROcp20_313*(RLcp20_115+RLcp20_156));
    JTcp20_356_8 = ROcp20_113*(RLcp20_215+RLcp20_256)-ROcp20_213*(RLcp20_115+RLcp20_156);
    JTcp20_156_9 = -(RLcp20_256*ROcp20_914-RLcp20_356*ROcp20_814);
    JTcp20_256_9 = RLcp20_156*ROcp20_914-RLcp20_356*ROcp20_714;
    JTcp20_356_9 = -(RLcp20_156*ROcp20_814-RLcp20_256*ROcp20_714);
    ORcp20_156 = OMcp20_215*RLcp20_356-OMcp20_315*RLcp20_256;
    ORcp20_256 = -(OMcp20_115*RLcp20_356-OMcp20_315*RLcp20_156);
    ORcp20_356 = OMcp20_115*RLcp20_256-OMcp20_215*RLcp20_156;
    VIcp20_156 = ORcp20_113+ORcp20_114+ORcp20_115+ORcp20_156+qd[1];
    VIcp20_256 = ORcp20_213+ORcp20_214+ORcp20_215+ORcp20_256+qd[2];
    VIcp20_356 = ORcp20_313+ORcp20_314+ORcp20_315+ORcp20_356+qd[3];
    ACcp20_156 = qdd[1]+OMcp20_213*ORcp20_314+OMcp20_214*ORcp20_315+OMcp20_215*ORcp20_356+OMcp20_26*ORcp20_313-OMcp20_313*
 ORcp20_214-OMcp20_314*ORcp20_215-OMcp20_315*ORcp20_256-OMcp20_36*ORcp20_213+OPcp20_213*RLcp20_314+OPcp20_214*RLcp20_315+
 OPcp20_215*RLcp20_356+OPcp20_26*RLcp20_313-OPcp20_313*RLcp20_214-OPcp20_314*RLcp20_215-OPcp20_315*RLcp20_256-OPcp20_36*
 RLcp20_213;
    ACcp20_256 = qdd[2]-OMcp20_113*ORcp20_314-OMcp20_114*ORcp20_315-OMcp20_115*ORcp20_356-OMcp20_16*ORcp20_313+OMcp20_313*
 ORcp20_114+OMcp20_314*ORcp20_115+OMcp20_315*ORcp20_156+OMcp20_36*ORcp20_113-OPcp20_113*RLcp20_314-OPcp20_114*RLcp20_315-
 OPcp20_115*RLcp20_356-OPcp20_16*RLcp20_313+OPcp20_313*RLcp20_114+OPcp20_314*RLcp20_115+OPcp20_315*RLcp20_156+OPcp20_36*
 RLcp20_113;
    ACcp20_356 = qdd[3]+OMcp20_113*ORcp20_214+OMcp20_114*ORcp20_215+OMcp20_115*ORcp20_256+OMcp20_16*ORcp20_213-OMcp20_213*
 ORcp20_114-OMcp20_214*ORcp20_115-OMcp20_215*ORcp20_156-OMcp20_26*ORcp20_113+OPcp20_113*RLcp20_214+OPcp20_114*RLcp20_215+
 OPcp20_115*RLcp20_256+OPcp20_16*RLcp20_213-OPcp20_213*RLcp20_114-OPcp20_214*RLcp20_115-OPcp20_215*RLcp20_156-OPcp20_26*
 RLcp20_113;

// = = Block_1_0_0_21_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp20_156;
    sens->P[2] = POcp20_256;
    sens->P[3] = POcp20_356;
    sens->R[1][1] = ROcp20_115;
    sens->R[1][2] = ROcp20_215;
    sens->R[1][3] = ROcp20_315;
    sens->R[2][1] = ROcp20_415;
    sens->R[2][2] = ROcp20_515;
    sens->R[2][3] = ROcp20_615;
    sens->R[3][1] = ROcp20_714;
    sens->R[3][2] = ROcp20_814;
    sens->R[3][3] = ROcp20_914;
    sens->V[1] = VIcp20_156;
    sens->V[2] = VIcp20_256;
    sens->V[3] = VIcp20_356;
    sens->OM[1] = OMcp20_115;
    sens->OM[2] = OMcp20_215;
    sens->OM[3] = OMcp20_315;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp20_156_5;
    sens->J[1][6] = JTcp20_156_6;
    sens->J[1][13] = JTcp20_156_7;
    sens->J[1][14] = JTcp20_156_8;
    sens->J[1][15] = JTcp20_156_9;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp20_256_4;
    sens->J[2][5] = JTcp20_256_5;
    sens->J[2][6] = JTcp20_256_6;
    sens->J[2][13] = JTcp20_256_7;
    sens->J[2][14] = JTcp20_256_8;
    sens->J[2][15] = JTcp20_256_9;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp20_356_4;
    sens->J[3][5] = JTcp20_356_5;
    sens->J[3][6] = JTcp20_356_6;
    sens->J[3][13] = JTcp20_356_7;
    sens->J[3][14] = JTcp20_356_8;
    sens->J[3][15] = JTcp20_356_9;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp20_46;
    sens->J[4][14] = ROcp20_113;
    sens->J[4][15] = ROcp20_714;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp20_85;
    sens->J[5][13] = ROcp20_56;
    sens->J[5][14] = ROcp20_213;
    sens->J[5][15] = ROcp20_814;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp20_95;
    sens->J[6][13] = ROcp20_66;
    sens->J[6][14] = ROcp20_313;
    sens->J[6][15] = ROcp20_914;
    sens->A[1] = ACcp20_156;
    sens->A[2] = ACcp20_256;
    sens->A[3] = ACcp20_356;
    sens->OMP[1] = OPcp20_115;
    sens->OMP[2] = OPcp20_215;
    sens->OMP[3] = OPcp20_315;
 
// 
break;
case 22:
 


// = = Block_1_0_0_22_0_1 = = 
 
// Sensor Kinematics 


    ROcp21_25 = S4*S5;
    ROcp21_35 = -C4*S5;
    ROcp21_85 = -S4*C5;
    ROcp21_95 = C4*C5;
    ROcp21_16 = C5*C6;
    ROcp21_26 = ROcp21_25*C6+C4*S6;
    ROcp21_36 = ROcp21_35*C6+S4*S6;
    ROcp21_46 = -C5*S6;
    ROcp21_56 = -(ROcp21_25*S6-C4*C6);
    ROcp21_66 = -(ROcp21_35*S6-S4*C6);
    OMcp21_25 = qd[5]*C4;
    OMcp21_35 = qd[5]*S4;
    OMcp21_16 = qd[4]+qd[6]*S5;
    OMcp21_26 = OMcp21_25+ROcp21_85*qd[6];
    OMcp21_36 = OMcp21_35+ROcp21_95*qd[6];
    OPcp21_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp21_26 = ROcp21_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp21_35*S5-ROcp21_95*qd[4]);
    OPcp21_36 = ROcp21_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp21_25*S5-ROcp21_85*qd[4]);

// = = Block_1_0_0_22_0_3 = = 
 
// Sensor Kinematics 


    ROcp21_113 = ROcp21_16*C13-S13*S5;
    ROcp21_213 = ROcp21_26*C13-ROcp21_85*S13;
    ROcp21_313 = ROcp21_36*C13-ROcp21_95*S13;
    ROcp21_713 = ROcp21_16*S13+C13*S5;
    ROcp21_813 = ROcp21_26*S13+ROcp21_85*C13;
    ROcp21_913 = ROcp21_36*S13+ROcp21_95*C13;
    ROcp21_414 = ROcp21_46*C14+ROcp21_713*S14;
    ROcp21_514 = ROcp21_56*C14+ROcp21_813*S14;
    ROcp21_614 = ROcp21_66*C14+ROcp21_913*S14;
    ROcp21_714 = -(ROcp21_46*S14-ROcp21_713*C14);
    ROcp21_814 = -(ROcp21_56*S14-ROcp21_813*C14);
    ROcp21_914 = -(ROcp21_66*S14-ROcp21_913*C14);
    ROcp21_115 = ROcp21_113*C15+ROcp21_414*S15;
    ROcp21_215 = ROcp21_213*C15+ROcp21_514*S15;
    ROcp21_315 = ROcp21_313*C15+ROcp21_614*S15;
    ROcp21_415 = -(ROcp21_113*S15-ROcp21_414*C15);
    ROcp21_515 = -(ROcp21_213*S15-ROcp21_514*C15);
    ROcp21_615 = -(ROcp21_313*S15-ROcp21_614*C15);
    RLcp21_113 = s->dpt[1][2]*ROcp21_16+s->dpt[3][2]*S5+ROcp21_46*s->dpt[2][2];
    RLcp21_213 = s->dpt[1][2]*ROcp21_26+s->dpt[3][2]*ROcp21_85+ROcp21_56*s->dpt[2][2];
    RLcp21_313 = s->dpt[1][2]*ROcp21_36+s->dpt[3][2]*ROcp21_95+ROcp21_66*s->dpt[2][2];
    OMcp21_113 = OMcp21_16+ROcp21_46*qd[13];
    OMcp21_213 = OMcp21_26+ROcp21_56*qd[13];
    OMcp21_313 = OMcp21_36+ROcp21_66*qd[13];
    ORcp21_113 = OMcp21_26*RLcp21_313-OMcp21_36*RLcp21_213;
    ORcp21_213 = -(OMcp21_16*RLcp21_313-OMcp21_36*RLcp21_113);
    ORcp21_313 = OMcp21_16*RLcp21_213-OMcp21_26*RLcp21_113;
    OPcp21_113 = OPcp21_16+ROcp21_46*qdd[13]+qd[13]*(OMcp21_26*ROcp21_66-OMcp21_36*ROcp21_56);
    OPcp21_213 = OPcp21_26+ROcp21_56*qdd[13]-qd[13]*(OMcp21_16*ROcp21_66-OMcp21_36*ROcp21_46);
    OPcp21_313 = OPcp21_36+ROcp21_66*qdd[13]+qd[13]*(OMcp21_16*ROcp21_56-OMcp21_26*ROcp21_46);
    RLcp21_114 = s->dpt[1][22]*ROcp21_113+s->dpt[3][22]*ROcp21_713+ROcp21_46*s->dpt[2][22];
    RLcp21_214 = s->dpt[1][22]*ROcp21_213+s->dpt[3][22]*ROcp21_813+ROcp21_56*s->dpt[2][22];
    RLcp21_314 = s->dpt[1][22]*ROcp21_313+s->dpt[3][22]*ROcp21_913+ROcp21_66*s->dpt[2][22];
    OMcp21_114 = OMcp21_113+ROcp21_113*qd[14];
    OMcp21_214 = OMcp21_213+ROcp21_213*qd[14];
    OMcp21_314 = OMcp21_313+ROcp21_313*qd[14];
    ORcp21_114 = OMcp21_213*RLcp21_314-OMcp21_313*RLcp21_214;
    ORcp21_214 = -(OMcp21_113*RLcp21_314-OMcp21_313*RLcp21_114);
    ORcp21_314 = OMcp21_113*RLcp21_214-OMcp21_213*RLcp21_114;
    OPcp21_114 = OPcp21_113+ROcp21_113*qdd[14]+qd[14]*(OMcp21_213*ROcp21_313-OMcp21_313*ROcp21_213);
    OPcp21_214 = OPcp21_213+ROcp21_213*qdd[14]-qd[14]*(OMcp21_113*ROcp21_313-OMcp21_313*ROcp21_113);
    OPcp21_314 = OPcp21_313+ROcp21_313*qdd[14]+qd[14]*(OMcp21_113*ROcp21_213-OMcp21_213*ROcp21_113);
    RLcp21_115 = s->dpt[1][24]*ROcp21_113+s->dpt[2][24]*ROcp21_414+ROcp21_714*s->dpt[3][24];
    RLcp21_215 = s->dpt[1][24]*ROcp21_213+s->dpt[2][24]*ROcp21_514+ROcp21_814*s->dpt[3][24];
    RLcp21_315 = s->dpt[1][24]*ROcp21_313+s->dpt[2][24]*ROcp21_614+ROcp21_914*s->dpt[3][24];
    OMcp21_115 = OMcp21_114+ROcp21_714*qd[15];
    OMcp21_215 = OMcp21_214+ROcp21_814*qd[15];
    OMcp21_315 = OMcp21_314+ROcp21_914*qd[15];
    ORcp21_115 = OMcp21_214*RLcp21_315-OMcp21_314*RLcp21_215;
    ORcp21_215 = -(OMcp21_114*RLcp21_315-OMcp21_314*RLcp21_115);
    ORcp21_315 = OMcp21_114*RLcp21_215-OMcp21_214*RLcp21_115;
    OPcp21_115 = OPcp21_114+ROcp21_714*qdd[15]+qd[15]*(OMcp21_214*ROcp21_914-OMcp21_314*ROcp21_814);
    OPcp21_215 = OPcp21_214+ROcp21_814*qdd[15]-qd[15]*(OMcp21_114*ROcp21_914-OMcp21_314*ROcp21_714);
    OPcp21_315 = OPcp21_314+ROcp21_914*qdd[15]+qd[15]*(OMcp21_114*ROcp21_814-OMcp21_214*ROcp21_714);
    RLcp21_157 = ROcp21_115*s->dpt[1][27]+ROcp21_415*s->dpt[2][27]+ROcp21_714*s->dpt[3][27];
    RLcp21_257 = ROcp21_215*s->dpt[1][27]+ROcp21_515*s->dpt[2][27]+ROcp21_814*s->dpt[3][27];
    RLcp21_357 = ROcp21_315*s->dpt[1][27]+ROcp21_615*s->dpt[2][27]+ROcp21_914*s->dpt[3][27];
    POcp21_157 = RLcp21_113+RLcp21_114+RLcp21_115+RLcp21_157+q[1];
    POcp21_257 = RLcp21_213+RLcp21_214+RLcp21_215+RLcp21_257+q[2];
    POcp21_357 = RLcp21_313+RLcp21_314+RLcp21_315+RLcp21_357+q[3];
    JTcp21_257_4 = -(RLcp21_313+RLcp21_314+RLcp21_315+RLcp21_357);
    JTcp21_357_4 = RLcp21_213+RLcp21_214+RLcp21_215+RLcp21_257;
    JTcp21_157_5 = C4*(RLcp21_313+RLcp21_314+RLcp21_315+RLcp21_357)-S4*(RLcp21_213+RLcp21_214)-S4*(RLcp21_215+RLcp21_257);
    JTcp21_257_5 = S4*(RLcp21_113+RLcp21_114+RLcp21_115+RLcp21_157);
    JTcp21_357_5 = -C4*(RLcp21_113+RLcp21_114+RLcp21_115+RLcp21_157);
    JTcp21_157_6 = ROcp21_85*(RLcp21_313+RLcp21_314+RLcp21_315+RLcp21_357)-ROcp21_95*(RLcp21_213+RLcp21_214)-ROcp21_95*(
 RLcp21_215+RLcp21_257);
    JTcp21_257_6 = RLcp21_157*ROcp21_95-RLcp21_315*S5-RLcp21_357*S5+ROcp21_95*(RLcp21_113+RLcp21_114+RLcp21_115)-S5*(
 RLcp21_313+RLcp21_314);
    JTcp21_357_6 = RLcp21_215*S5-ROcp21_85*(RLcp21_113+RLcp21_114+RLcp21_115)+S5*(RLcp21_213+RLcp21_214)-RLcp21_157*
 ROcp21_85+RLcp21_257*S5;
    JTcp21_157_7 = ROcp21_56*(RLcp21_314+RLcp21_315)-ROcp21_66*(RLcp21_214+RLcp21_215)-RLcp21_257*ROcp21_66+RLcp21_357*
 ROcp21_56;
    JTcp21_257_7 = RLcp21_157*ROcp21_66-RLcp21_357*ROcp21_46-ROcp21_46*(RLcp21_314+RLcp21_315)+ROcp21_66*(RLcp21_114+
 RLcp21_115);
    JTcp21_357_7 = ROcp21_46*(RLcp21_214+RLcp21_215)-ROcp21_56*(RLcp21_114+RLcp21_115)-RLcp21_157*ROcp21_56+RLcp21_257*
 ROcp21_46;
    JTcp21_157_8 = ROcp21_213*(RLcp21_315+RLcp21_357)-ROcp21_313*(RLcp21_215+RLcp21_257);
    JTcp21_257_8 = -(ROcp21_113*(RLcp21_315+RLcp21_357)-ROcp21_313*(RLcp21_115+RLcp21_157));
    JTcp21_357_8 = ROcp21_113*(RLcp21_215+RLcp21_257)-ROcp21_213*(RLcp21_115+RLcp21_157);
    JTcp21_157_9 = -(RLcp21_257*ROcp21_914-RLcp21_357*ROcp21_814);
    JTcp21_257_9 = RLcp21_157*ROcp21_914-RLcp21_357*ROcp21_714;
    JTcp21_357_9 = -(RLcp21_157*ROcp21_814-RLcp21_257*ROcp21_714);
    ORcp21_157 = OMcp21_215*RLcp21_357-OMcp21_315*RLcp21_257;
    ORcp21_257 = -(OMcp21_115*RLcp21_357-OMcp21_315*RLcp21_157);
    ORcp21_357 = OMcp21_115*RLcp21_257-OMcp21_215*RLcp21_157;
    VIcp21_157 = ORcp21_113+ORcp21_114+ORcp21_115+ORcp21_157+qd[1];
    VIcp21_257 = ORcp21_213+ORcp21_214+ORcp21_215+ORcp21_257+qd[2];
    VIcp21_357 = ORcp21_313+ORcp21_314+ORcp21_315+ORcp21_357+qd[3];
    ACcp21_157 = qdd[1]+OMcp21_213*ORcp21_314+OMcp21_214*ORcp21_315+OMcp21_215*ORcp21_357+OMcp21_26*ORcp21_313-OMcp21_313*
 ORcp21_214-OMcp21_314*ORcp21_215-OMcp21_315*ORcp21_257-OMcp21_36*ORcp21_213+OPcp21_213*RLcp21_314+OPcp21_214*RLcp21_315+
 OPcp21_215*RLcp21_357+OPcp21_26*RLcp21_313-OPcp21_313*RLcp21_214-OPcp21_314*RLcp21_215-OPcp21_315*RLcp21_257-OPcp21_36*
 RLcp21_213;
    ACcp21_257 = qdd[2]-OMcp21_113*ORcp21_314-OMcp21_114*ORcp21_315-OMcp21_115*ORcp21_357-OMcp21_16*ORcp21_313+OMcp21_313*
 ORcp21_114+OMcp21_314*ORcp21_115+OMcp21_315*ORcp21_157+OMcp21_36*ORcp21_113-OPcp21_113*RLcp21_314-OPcp21_114*RLcp21_315-
 OPcp21_115*RLcp21_357-OPcp21_16*RLcp21_313+OPcp21_313*RLcp21_114+OPcp21_314*RLcp21_115+OPcp21_315*RLcp21_157+OPcp21_36*
 RLcp21_113;
    ACcp21_357 = qdd[3]+OMcp21_113*ORcp21_214+OMcp21_114*ORcp21_215+OMcp21_115*ORcp21_257+OMcp21_16*ORcp21_213-OMcp21_213*
 ORcp21_114-OMcp21_214*ORcp21_115-OMcp21_215*ORcp21_157-OMcp21_26*ORcp21_113+OPcp21_113*RLcp21_214+OPcp21_114*RLcp21_215+
 OPcp21_115*RLcp21_257+OPcp21_16*RLcp21_213-OPcp21_213*RLcp21_114-OPcp21_214*RLcp21_115-OPcp21_215*RLcp21_157-OPcp21_26*
 RLcp21_113;

// = = Block_1_0_0_22_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp21_157;
    sens->P[2] = POcp21_257;
    sens->P[3] = POcp21_357;
    sens->R[1][1] = ROcp21_115;
    sens->R[1][2] = ROcp21_215;
    sens->R[1][3] = ROcp21_315;
    sens->R[2][1] = ROcp21_415;
    sens->R[2][2] = ROcp21_515;
    sens->R[2][3] = ROcp21_615;
    sens->R[3][1] = ROcp21_714;
    sens->R[3][2] = ROcp21_814;
    sens->R[3][3] = ROcp21_914;
    sens->V[1] = VIcp21_157;
    sens->V[2] = VIcp21_257;
    sens->V[3] = VIcp21_357;
    sens->OM[1] = OMcp21_115;
    sens->OM[2] = OMcp21_215;
    sens->OM[3] = OMcp21_315;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp21_157_5;
    sens->J[1][6] = JTcp21_157_6;
    sens->J[1][13] = JTcp21_157_7;
    sens->J[1][14] = JTcp21_157_8;
    sens->J[1][15] = JTcp21_157_9;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp21_257_4;
    sens->J[2][5] = JTcp21_257_5;
    sens->J[2][6] = JTcp21_257_6;
    sens->J[2][13] = JTcp21_257_7;
    sens->J[2][14] = JTcp21_257_8;
    sens->J[2][15] = JTcp21_257_9;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp21_357_4;
    sens->J[3][5] = JTcp21_357_5;
    sens->J[3][6] = JTcp21_357_6;
    sens->J[3][13] = JTcp21_357_7;
    sens->J[3][14] = JTcp21_357_8;
    sens->J[3][15] = JTcp21_357_9;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp21_46;
    sens->J[4][14] = ROcp21_113;
    sens->J[4][15] = ROcp21_714;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp21_85;
    sens->J[5][13] = ROcp21_56;
    sens->J[5][14] = ROcp21_213;
    sens->J[5][15] = ROcp21_814;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp21_95;
    sens->J[6][13] = ROcp21_66;
    sens->J[6][14] = ROcp21_313;
    sens->J[6][15] = ROcp21_914;
    sens->A[1] = ACcp21_157;
    sens->A[2] = ACcp21_257;
    sens->A[3] = ACcp21_357;
    sens->OMP[1] = OPcp21_115;
    sens->OMP[2] = OPcp21_215;
    sens->OMP[3] = OPcp21_315;
 
// 
break;
case 23:
 


// = = Block_1_0_0_23_0_1 = = 
 
// Sensor Kinematics 


    ROcp22_25 = S4*S5;
    ROcp22_35 = -C4*S5;
    ROcp22_85 = -S4*C5;
    ROcp22_95 = C4*C5;
    ROcp22_16 = C5*C6;
    ROcp22_26 = ROcp22_25*C6+C4*S6;
    ROcp22_36 = ROcp22_35*C6+S4*S6;
    ROcp22_46 = -C5*S6;
    ROcp22_56 = -(ROcp22_25*S6-C4*C6);
    ROcp22_66 = -(ROcp22_35*S6-S4*C6);
    OMcp22_25 = qd[5]*C4;
    OMcp22_35 = qd[5]*S4;
    OMcp22_16 = qd[4]+qd[6]*S5;
    OMcp22_26 = OMcp22_25+ROcp22_85*qd[6];
    OMcp22_36 = OMcp22_35+ROcp22_95*qd[6];
    OPcp22_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp22_26 = ROcp22_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp22_35*S5-ROcp22_95*qd[4]);
    OPcp22_36 = ROcp22_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp22_25*S5-ROcp22_85*qd[4]);

// = = Block_1_0_0_23_0_3 = = 
 
// Sensor Kinematics 


    ROcp22_113 = ROcp22_16*C13-S13*S5;
    ROcp22_213 = ROcp22_26*C13-ROcp22_85*S13;
    ROcp22_313 = ROcp22_36*C13-ROcp22_95*S13;
    ROcp22_713 = ROcp22_16*S13+C13*S5;
    ROcp22_813 = ROcp22_26*S13+ROcp22_85*C13;
    ROcp22_913 = ROcp22_36*S13+ROcp22_95*C13;
    ROcp22_414 = ROcp22_46*C14+ROcp22_713*S14;
    ROcp22_514 = ROcp22_56*C14+ROcp22_813*S14;
    ROcp22_614 = ROcp22_66*C14+ROcp22_913*S14;
    ROcp22_714 = -(ROcp22_46*S14-ROcp22_713*C14);
    ROcp22_814 = -(ROcp22_56*S14-ROcp22_813*C14);
    ROcp22_914 = -(ROcp22_66*S14-ROcp22_913*C14);
    ROcp22_115 = ROcp22_113*C15+ROcp22_414*S15;
    ROcp22_215 = ROcp22_213*C15+ROcp22_514*S15;
    ROcp22_315 = ROcp22_313*C15+ROcp22_614*S15;
    ROcp22_415 = -(ROcp22_113*S15-ROcp22_414*C15);
    ROcp22_515 = -(ROcp22_213*S15-ROcp22_514*C15);
    ROcp22_615 = -(ROcp22_313*S15-ROcp22_614*C15);
    ROcp22_116 = ROcp22_115*C16-ROcp22_714*S16;
    ROcp22_216 = ROcp22_215*C16-ROcp22_814*S16;
    ROcp22_316 = ROcp22_315*C16-ROcp22_914*S16;
    ROcp22_716 = ROcp22_115*S16+ROcp22_714*C16;
    ROcp22_816 = ROcp22_215*S16+ROcp22_814*C16;
    ROcp22_916 = ROcp22_315*S16+ROcp22_914*C16;
    RLcp22_113 = s->dpt[1][2]*ROcp22_16+s->dpt[3][2]*S5+ROcp22_46*s->dpt[2][2];
    RLcp22_213 = s->dpt[1][2]*ROcp22_26+s->dpt[3][2]*ROcp22_85+ROcp22_56*s->dpt[2][2];
    RLcp22_313 = s->dpt[1][2]*ROcp22_36+s->dpt[3][2]*ROcp22_95+ROcp22_66*s->dpt[2][2];
    OMcp22_113 = OMcp22_16+ROcp22_46*qd[13];
    OMcp22_213 = OMcp22_26+ROcp22_56*qd[13];
    OMcp22_313 = OMcp22_36+ROcp22_66*qd[13];
    ORcp22_113 = OMcp22_26*RLcp22_313-OMcp22_36*RLcp22_213;
    ORcp22_213 = -(OMcp22_16*RLcp22_313-OMcp22_36*RLcp22_113);
    ORcp22_313 = OMcp22_16*RLcp22_213-OMcp22_26*RLcp22_113;
    OPcp22_113 = OPcp22_16+ROcp22_46*qdd[13]+qd[13]*(OMcp22_26*ROcp22_66-OMcp22_36*ROcp22_56);
    OPcp22_213 = OPcp22_26+ROcp22_56*qdd[13]-qd[13]*(OMcp22_16*ROcp22_66-OMcp22_36*ROcp22_46);
    OPcp22_313 = OPcp22_36+ROcp22_66*qdd[13]+qd[13]*(OMcp22_16*ROcp22_56-OMcp22_26*ROcp22_46);
    RLcp22_114 = s->dpt[1][22]*ROcp22_113+s->dpt[3][22]*ROcp22_713+ROcp22_46*s->dpt[2][22];
    RLcp22_214 = s->dpt[1][22]*ROcp22_213+s->dpt[3][22]*ROcp22_813+ROcp22_56*s->dpt[2][22];
    RLcp22_314 = s->dpt[1][22]*ROcp22_313+s->dpt[3][22]*ROcp22_913+ROcp22_66*s->dpt[2][22];
    OMcp22_114 = OMcp22_113+ROcp22_113*qd[14];
    OMcp22_214 = OMcp22_213+ROcp22_213*qd[14];
    OMcp22_314 = OMcp22_313+ROcp22_313*qd[14];
    ORcp22_114 = OMcp22_213*RLcp22_314-OMcp22_313*RLcp22_214;
    ORcp22_214 = -(OMcp22_113*RLcp22_314-OMcp22_313*RLcp22_114);
    ORcp22_314 = OMcp22_113*RLcp22_214-OMcp22_213*RLcp22_114;
    OPcp22_114 = OPcp22_113+ROcp22_113*qdd[14]+qd[14]*(OMcp22_213*ROcp22_313-OMcp22_313*ROcp22_213);
    OPcp22_214 = OPcp22_213+ROcp22_213*qdd[14]-qd[14]*(OMcp22_113*ROcp22_313-OMcp22_313*ROcp22_113);
    OPcp22_314 = OPcp22_313+ROcp22_313*qdd[14]+qd[14]*(OMcp22_113*ROcp22_213-OMcp22_213*ROcp22_113);
    RLcp22_115 = s->dpt[1][24]*ROcp22_113+s->dpt[2][24]*ROcp22_414+ROcp22_714*s->dpt[3][24];
    RLcp22_215 = s->dpt[1][24]*ROcp22_213+s->dpt[2][24]*ROcp22_514+ROcp22_814*s->dpt[3][24];
    RLcp22_315 = s->dpt[1][24]*ROcp22_313+s->dpt[2][24]*ROcp22_614+ROcp22_914*s->dpt[3][24];
    OMcp22_115 = OMcp22_114+ROcp22_714*qd[15];
    OMcp22_215 = OMcp22_214+ROcp22_814*qd[15];
    OMcp22_315 = OMcp22_314+ROcp22_914*qd[15];
    ORcp22_115 = OMcp22_214*RLcp22_315-OMcp22_314*RLcp22_215;
    ORcp22_215 = -(OMcp22_114*RLcp22_315-OMcp22_314*RLcp22_115);
    ORcp22_315 = OMcp22_114*RLcp22_215-OMcp22_214*RLcp22_115;
    OPcp22_115 = OPcp22_114+ROcp22_714*qdd[15]+qd[15]*(OMcp22_214*ROcp22_914-OMcp22_314*ROcp22_814);
    OPcp22_215 = OPcp22_214+ROcp22_814*qdd[15]-qd[15]*(OMcp22_114*ROcp22_914-OMcp22_314*ROcp22_714);
    OPcp22_315 = OPcp22_314+ROcp22_914*qdd[15]+qd[15]*(OMcp22_114*ROcp22_814-OMcp22_214*ROcp22_714);
    RLcp22_116 = s->dpt[1][26]*ROcp22_115+s->dpt[2][26]*ROcp22_415+ROcp22_714*s->dpt[3][26];
    RLcp22_216 = s->dpt[1][26]*ROcp22_215+s->dpt[2][26]*ROcp22_515+ROcp22_814*s->dpt[3][26];
    RLcp22_316 = s->dpt[1][26]*ROcp22_315+s->dpt[2][26]*ROcp22_615+ROcp22_914*s->dpt[3][26];
    OMcp22_116 = OMcp22_115+ROcp22_415*qd[16];
    OMcp22_216 = OMcp22_215+ROcp22_515*qd[16];
    OMcp22_316 = OMcp22_315+ROcp22_615*qd[16];
    ORcp22_116 = OMcp22_215*RLcp22_316-OMcp22_315*RLcp22_216;
    ORcp22_216 = -(OMcp22_115*RLcp22_316-OMcp22_315*RLcp22_116);
    ORcp22_316 = OMcp22_115*RLcp22_216-OMcp22_215*RLcp22_116;
    OPcp22_116 = OPcp22_115+ROcp22_415*qdd[16]+qd[16]*(OMcp22_215*ROcp22_615-OMcp22_315*ROcp22_515);
    OPcp22_216 = OPcp22_215+ROcp22_515*qdd[16]-qd[16]*(OMcp22_115*ROcp22_615-OMcp22_315*ROcp22_415);
    OPcp22_316 = OPcp22_315+ROcp22_615*qdd[16]+qd[16]*(OMcp22_115*ROcp22_515-OMcp22_215*ROcp22_415);
    RLcp22_158 = s->dpt[1][28]*ROcp22_116+s->dpt[2][28]*ROcp22_415+ROcp22_716*s->dpt[3][28];
    RLcp22_258 = s->dpt[1][28]*ROcp22_216+s->dpt[2][28]*ROcp22_515+ROcp22_816*s->dpt[3][28];
    RLcp22_358 = s->dpt[1][28]*ROcp22_316+s->dpt[2][28]*ROcp22_615+ROcp22_916*s->dpt[3][28];
    POcp22_158 = RLcp22_113+RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_158+q[1];
    POcp22_258 = RLcp22_213+RLcp22_214+RLcp22_215+RLcp22_216+RLcp22_258+q[2];
    POcp22_358 = RLcp22_313+RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_358+q[3];
    JTcp22_258_4 = -(RLcp22_313+RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_358);
    JTcp22_358_4 = RLcp22_213+RLcp22_214+RLcp22_215+RLcp22_216+RLcp22_258;
    JTcp22_158_5 = C4*(RLcp22_313+RLcp22_314+RLcp22_315+RLcp22_316)-S4*(RLcp22_213+RLcp22_214)-S4*(RLcp22_215+RLcp22_216)-
 RLcp22_258*S4+RLcp22_358*C4;
    JTcp22_258_5 = S4*(RLcp22_113+RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_158);
    JTcp22_358_5 = -C4*(RLcp22_113+RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_158);
    JTcp22_158_6 = ROcp22_85*(RLcp22_313+RLcp22_314+RLcp22_315+RLcp22_316)-ROcp22_95*(RLcp22_213+RLcp22_214)-ROcp22_95*(
 RLcp22_215+RLcp22_216)-RLcp22_258*ROcp22_95+RLcp22_358*ROcp22_85;
    JTcp22_258_6 = -(RLcp22_358*S5-ROcp22_95*(RLcp22_113+RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_158)+S5*(RLcp22_313+
 RLcp22_314)+S5*(RLcp22_315+RLcp22_316));
    JTcp22_358_6 = RLcp22_258*S5-ROcp22_85*(RLcp22_113+RLcp22_114+RLcp22_115+RLcp22_116+RLcp22_158)+S5*(RLcp22_213+
 RLcp22_214)+S5*(RLcp22_215+RLcp22_216);
    JTcp22_158_7 = ROcp22_56*(RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_358)-ROcp22_66*(RLcp22_214+RLcp22_215)-ROcp22_66*(
 RLcp22_216+RLcp22_258);
    JTcp22_258_7 = -(ROcp22_46*(RLcp22_314+RLcp22_315+RLcp22_316+RLcp22_358)-ROcp22_66*(RLcp22_114+RLcp22_115)-ROcp22_66*(
 RLcp22_116+RLcp22_158));
    JTcp22_358_7 = ROcp22_46*(RLcp22_214+RLcp22_215+RLcp22_216+RLcp22_258)-ROcp22_56*(RLcp22_114+RLcp22_115)-ROcp22_56*(
 RLcp22_116+RLcp22_158);
    JTcp22_158_8 = ROcp22_213*(RLcp22_315+RLcp22_316)-ROcp22_313*(RLcp22_215+RLcp22_216)-RLcp22_258*ROcp22_313+RLcp22_358*
 ROcp22_213;
    JTcp22_258_8 = RLcp22_158*ROcp22_313-RLcp22_358*ROcp22_113-ROcp22_113*(RLcp22_315+RLcp22_316)+ROcp22_313*(RLcp22_115+
 RLcp22_116);
    JTcp22_358_8 = ROcp22_113*(RLcp22_215+RLcp22_216)-ROcp22_213*(RLcp22_115+RLcp22_116)-RLcp22_158*ROcp22_213+RLcp22_258*
 ROcp22_113;
    JTcp22_158_9 = ROcp22_814*(RLcp22_316+RLcp22_358)-ROcp22_914*(RLcp22_216+RLcp22_258);
    JTcp22_258_9 = -(ROcp22_714*(RLcp22_316+RLcp22_358)-ROcp22_914*(RLcp22_116+RLcp22_158));
    JTcp22_358_9 = ROcp22_714*(RLcp22_216+RLcp22_258)-ROcp22_814*(RLcp22_116+RLcp22_158);
    JTcp22_158_10 = -(RLcp22_258*ROcp22_615-RLcp22_358*ROcp22_515);
    JTcp22_258_10 = RLcp22_158*ROcp22_615-RLcp22_358*ROcp22_415;
    JTcp22_358_10 = -(RLcp22_158*ROcp22_515-RLcp22_258*ROcp22_415);
    ORcp22_158 = OMcp22_216*RLcp22_358-OMcp22_316*RLcp22_258;
    ORcp22_258 = -(OMcp22_116*RLcp22_358-OMcp22_316*RLcp22_158);
    ORcp22_358 = OMcp22_116*RLcp22_258-OMcp22_216*RLcp22_158;
    VIcp22_158 = ORcp22_113+ORcp22_114+ORcp22_115+ORcp22_116+ORcp22_158+qd[1];
    VIcp22_258 = ORcp22_213+ORcp22_214+ORcp22_215+ORcp22_216+ORcp22_258+qd[2];
    VIcp22_358 = ORcp22_313+ORcp22_314+ORcp22_315+ORcp22_316+ORcp22_358+qd[3];
    ACcp22_158 = qdd[1]+OMcp22_213*ORcp22_314+OMcp22_214*ORcp22_315+OMcp22_215*ORcp22_316+OMcp22_216*ORcp22_358+OMcp22_26*
 ORcp22_313-OMcp22_313*ORcp22_214-OMcp22_314*ORcp22_215-OMcp22_315*ORcp22_216-OMcp22_316*ORcp22_258-OMcp22_36*ORcp22_213+
 OPcp22_213*RLcp22_314+OPcp22_214*RLcp22_315+OPcp22_215*RLcp22_316+OPcp22_216*RLcp22_358+OPcp22_26*RLcp22_313-OPcp22_313*
 RLcp22_214-OPcp22_314*RLcp22_215-OPcp22_315*RLcp22_216-OPcp22_316*RLcp22_258-OPcp22_36*RLcp22_213;
    ACcp22_258 = qdd[2]-OMcp22_113*ORcp22_314-OMcp22_114*ORcp22_315-OMcp22_115*ORcp22_316-OMcp22_116*ORcp22_358-OMcp22_16*
 ORcp22_313+OMcp22_313*ORcp22_114+OMcp22_314*ORcp22_115+OMcp22_315*ORcp22_116+OMcp22_316*ORcp22_158+OMcp22_36*ORcp22_113-
 OPcp22_113*RLcp22_314-OPcp22_114*RLcp22_315-OPcp22_115*RLcp22_316-OPcp22_116*RLcp22_358-OPcp22_16*RLcp22_313+OPcp22_313*
 RLcp22_114+OPcp22_314*RLcp22_115+OPcp22_315*RLcp22_116+OPcp22_316*RLcp22_158+OPcp22_36*RLcp22_113;
    ACcp22_358 = qdd[3]+OMcp22_113*ORcp22_214+OMcp22_114*ORcp22_215+OMcp22_115*ORcp22_216+OMcp22_116*ORcp22_258+OMcp22_16*
 ORcp22_213-OMcp22_213*ORcp22_114-OMcp22_214*ORcp22_115-OMcp22_215*ORcp22_116-OMcp22_216*ORcp22_158-OMcp22_26*ORcp22_113+
 OPcp22_113*RLcp22_214+OPcp22_114*RLcp22_215+OPcp22_115*RLcp22_216+OPcp22_116*RLcp22_258+OPcp22_16*RLcp22_213-OPcp22_213*
 RLcp22_114-OPcp22_214*RLcp22_115-OPcp22_215*RLcp22_116-OPcp22_216*RLcp22_158-OPcp22_26*RLcp22_113;

// = = Block_1_0_0_23_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp22_158;
    sens->P[2] = POcp22_258;
    sens->P[3] = POcp22_358;
    sens->R[1][1] = ROcp22_116;
    sens->R[1][2] = ROcp22_216;
    sens->R[1][3] = ROcp22_316;
    sens->R[2][1] = ROcp22_415;
    sens->R[2][2] = ROcp22_515;
    sens->R[2][3] = ROcp22_615;
    sens->R[3][1] = ROcp22_716;
    sens->R[3][2] = ROcp22_816;
    sens->R[3][3] = ROcp22_916;
    sens->V[1] = VIcp22_158;
    sens->V[2] = VIcp22_258;
    sens->V[3] = VIcp22_358;
    sens->OM[1] = OMcp22_116;
    sens->OM[2] = OMcp22_216;
    sens->OM[3] = OMcp22_316;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp22_158_5;
    sens->J[1][6] = JTcp22_158_6;
    sens->J[1][13] = JTcp22_158_7;
    sens->J[1][14] = JTcp22_158_8;
    sens->J[1][15] = JTcp22_158_9;
    sens->J[1][16] = JTcp22_158_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp22_258_4;
    sens->J[2][5] = JTcp22_258_5;
    sens->J[2][6] = JTcp22_258_6;
    sens->J[2][13] = JTcp22_258_7;
    sens->J[2][14] = JTcp22_258_8;
    sens->J[2][15] = JTcp22_258_9;
    sens->J[2][16] = JTcp22_258_10;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp22_358_4;
    sens->J[3][5] = JTcp22_358_5;
    sens->J[3][6] = JTcp22_358_6;
    sens->J[3][13] = JTcp22_358_7;
    sens->J[3][14] = JTcp22_358_8;
    sens->J[3][15] = JTcp22_358_9;
    sens->J[3][16] = JTcp22_358_10;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp22_46;
    sens->J[4][14] = ROcp22_113;
    sens->J[4][15] = ROcp22_714;
    sens->J[4][16] = ROcp22_415;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp22_85;
    sens->J[5][13] = ROcp22_56;
    sens->J[5][14] = ROcp22_213;
    sens->J[5][15] = ROcp22_814;
    sens->J[5][16] = ROcp22_515;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp22_95;
    sens->J[6][13] = ROcp22_66;
    sens->J[6][14] = ROcp22_313;
    sens->J[6][15] = ROcp22_914;
    sens->J[6][16] = ROcp22_615;
    sens->A[1] = ACcp22_158;
    sens->A[2] = ACcp22_258;
    sens->A[3] = ACcp22_358;
    sens->OMP[1] = OPcp22_116;
    sens->OMP[2] = OPcp22_216;
    sens->OMP[3] = OPcp22_316;
 
// 
break;
case 24:
 


// = = Block_1_0_0_24_0_1 = = 
 
// Sensor Kinematics 


    ROcp23_25 = S4*S5;
    ROcp23_35 = -C4*S5;
    ROcp23_85 = -S4*C5;
    ROcp23_95 = C4*C5;
    ROcp23_16 = C5*C6;
    ROcp23_26 = ROcp23_25*C6+C4*S6;
    ROcp23_36 = ROcp23_35*C6+S4*S6;
    ROcp23_46 = -C5*S6;
    ROcp23_56 = -(ROcp23_25*S6-C4*C6);
    ROcp23_66 = -(ROcp23_35*S6-S4*C6);
    OMcp23_25 = qd[5]*C4;
    OMcp23_35 = qd[5]*S4;
    OMcp23_16 = qd[4]+qd[6]*S5;
    OMcp23_26 = OMcp23_25+ROcp23_85*qd[6];
    OMcp23_36 = OMcp23_35+ROcp23_95*qd[6];
    OPcp23_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp23_26 = ROcp23_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp23_35*S5-ROcp23_95*qd[4]);
    OPcp23_36 = ROcp23_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp23_25*S5-ROcp23_85*qd[4]);

// = = Block_1_0_0_24_0_3 = = 
 
// Sensor Kinematics 


    ROcp23_113 = ROcp23_16*C13-S13*S5;
    ROcp23_213 = ROcp23_26*C13-ROcp23_85*S13;
    ROcp23_313 = ROcp23_36*C13-ROcp23_95*S13;
    ROcp23_713 = ROcp23_16*S13+C13*S5;
    ROcp23_813 = ROcp23_26*S13+ROcp23_85*C13;
    ROcp23_913 = ROcp23_36*S13+ROcp23_95*C13;
    ROcp23_414 = ROcp23_46*C14+ROcp23_713*S14;
    ROcp23_514 = ROcp23_56*C14+ROcp23_813*S14;
    ROcp23_614 = ROcp23_66*C14+ROcp23_913*S14;
    ROcp23_714 = -(ROcp23_46*S14-ROcp23_713*C14);
    ROcp23_814 = -(ROcp23_56*S14-ROcp23_813*C14);
    ROcp23_914 = -(ROcp23_66*S14-ROcp23_913*C14);
    ROcp23_115 = ROcp23_113*C15+ROcp23_414*S15;
    ROcp23_215 = ROcp23_213*C15+ROcp23_514*S15;
    ROcp23_315 = ROcp23_313*C15+ROcp23_614*S15;
    ROcp23_415 = -(ROcp23_113*S15-ROcp23_414*C15);
    ROcp23_515 = -(ROcp23_213*S15-ROcp23_514*C15);
    ROcp23_615 = -(ROcp23_313*S15-ROcp23_614*C15);
    ROcp23_116 = ROcp23_115*C16-ROcp23_714*S16;
    ROcp23_216 = ROcp23_215*C16-ROcp23_814*S16;
    ROcp23_316 = ROcp23_315*C16-ROcp23_914*S16;
    ROcp23_716 = ROcp23_115*S16+ROcp23_714*C16;
    ROcp23_816 = ROcp23_215*S16+ROcp23_814*C16;
    ROcp23_916 = ROcp23_315*S16+ROcp23_914*C16;
    RLcp23_113 = s->dpt[1][2]*ROcp23_16+s->dpt[3][2]*S5+ROcp23_46*s->dpt[2][2];
    RLcp23_213 = s->dpt[1][2]*ROcp23_26+s->dpt[3][2]*ROcp23_85+ROcp23_56*s->dpt[2][2];
    RLcp23_313 = s->dpt[1][2]*ROcp23_36+s->dpt[3][2]*ROcp23_95+ROcp23_66*s->dpt[2][2];
    OMcp23_113 = OMcp23_16+ROcp23_46*qd[13];
    OMcp23_213 = OMcp23_26+ROcp23_56*qd[13];
    OMcp23_313 = OMcp23_36+ROcp23_66*qd[13];
    ORcp23_113 = OMcp23_26*RLcp23_313-OMcp23_36*RLcp23_213;
    ORcp23_213 = -(OMcp23_16*RLcp23_313-OMcp23_36*RLcp23_113);
    ORcp23_313 = OMcp23_16*RLcp23_213-OMcp23_26*RLcp23_113;
    OPcp23_113 = OPcp23_16+ROcp23_46*qdd[13]+qd[13]*(OMcp23_26*ROcp23_66-OMcp23_36*ROcp23_56);
    OPcp23_213 = OPcp23_26+ROcp23_56*qdd[13]-qd[13]*(OMcp23_16*ROcp23_66-OMcp23_36*ROcp23_46);
    OPcp23_313 = OPcp23_36+ROcp23_66*qdd[13]+qd[13]*(OMcp23_16*ROcp23_56-OMcp23_26*ROcp23_46);
    RLcp23_114 = s->dpt[1][22]*ROcp23_113+s->dpt[3][22]*ROcp23_713+ROcp23_46*s->dpt[2][22];
    RLcp23_214 = s->dpt[1][22]*ROcp23_213+s->dpt[3][22]*ROcp23_813+ROcp23_56*s->dpt[2][22];
    RLcp23_314 = s->dpt[1][22]*ROcp23_313+s->dpt[3][22]*ROcp23_913+ROcp23_66*s->dpt[2][22];
    OMcp23_114 = OMcp23_113+ROcp23_113*qd[14];
    OMcp23_214 = OMcp23_213+ROcp23_213*qd[14];
    OMcp23_314 = OMcp23_313+ROcp23_313*qd[14];
    ORcp23_114 = OMcp23_213*RLcp23_314-OMcp23_313*RLcp23_214;
    ORcp23_214 = -(OMcp23_113*RLcp23_314-OMcp23_313*RLcp23_114);
    ORcp23_314 = OMcp23_113*RLcp23_214-OMcp23_213*RLcp23_114;
    OPcp23_114 = OPcp23_113+ROcp23_113*qdd[14]+qd[14]*(OMcp23_213*ROcp23_313-OMcp23_313*ROcp23_213);
    OPcp23_214 = OPcp23_213+ROcp23_213*qdd[14]-qd[14]*(OMcp23_113*ROcp23_313-OMcp23_313*ROcp23_113);
    OPcp23_314 = OPcp23_313+ROcp23_313*qdd[14]+qd[14]*(OMcp23_113*ROcp23_213-OMcp23_213*ROcp23_113);
    RLcp23_115 = s->dpt[1][24]*ROcp23_113+s->dpt[2][24]*ROcp23_414+ROcp23_714*s->dpt[3][24];
    RLcp23_215 = s->dpt[1][24]*ROcp23_213+s->dpt[2][24]*ROcp23_514+ROcp23_814*s->dpt[3][24];
    RLcp23_315 = s->dpt[1][24]*ROcp23_313+s->dpt[2][24]*ROcp23_614+ROcp23_914*s->dpt[3][24];
    OMcp23_115 = OMcp23_114+ROcp23_714*qd[15];
    OMcp23_215 = OMcp23_214+ROcp23_814*qd[15];
    OMcp23_315 = OMcp23_314+ROcp23_914*qd[15];
    ORcp23_115 = OMcp23_214*RLcp23_315-OMcp23_314*RLcp23_215;
    ORcp23_215 = -(OMcp23_114*RLcp23_315-OMcp23_314*RLcp23_115);
    ORcp23_315 = OMcp23_114*RLcp23_215-OMcp23_214*RLcp23_115;
    OPcp23_115 = OPcp23_114+ROcp23_714*qdd[15]+qd[15]*(OMcp23_214*ROcp23_914-OMcp23_314*ROcp23_814);
    OPcp23_215 = OPcp23_214+ROcp23_814*qdd[15]-qd[15]*(OMcp23_114*ROcp23_914-OMcp23_314*ROcp23_714);
    OPcp23_315 = OPcp23_314+ROcp23_914*qdd[15]+qd[15]*(OMcp23_114*ROcp23_814-OMcp23_214*ROcp23_714);
    RLcp23_116 = s->dpt[1][26]*ROcp23_115+s->dpt[2][26]*ROcp23_415+ROcp23_714*s->dpt[3][26];
    RLcp23_216 = s->dpt[1][26]*ROcp23_215+s->dpt[2][26]*ROcp23_515+ROcp23_814*s->dpt[3][26];
    RLcp23_316 = s->dpt[1][26]*ROcp23_315+s->dpt[2][26]*ROcp23_615+ROcp23_914*s->dpt[3][26];
    OMcp23_116 = OMcp23_115+ROcp23_415*qd[16];
    OMcp23_216 = OMcp23_215+ROcp23_515*qd[16];
    OMcp23_316 = OMcp23_315+ROcp23_615*qd[16];
    ORcp23_116 = OMcp23_215*RLcp23_316-OMcp23_315*RLcp23_216;
    ORcp23_216 = -(OMcp23_115*RLcp23_316-OMcp23_315*RLcp23_116);
    ORcp23_316 = OMcp23_115*RLcp23_216-OMcp23_215*RLcp23_116;
    OPcp23_116 = OPcp23_115+ROcp23_415*qdd[16]+qd[16]*(OMcp23_215*ROcp23_615-OMcp23_315*ROcp23_515);
    OPcp23_216 = OPcp23_215+ROcp23_515*qdd[16]-qd[16]*(OMcp23_115*ROcp23_615-OMcp23_315*ROcp23_415);
    OPcp23_316 = OPcp23_315+ROcp23_615*qdd[16]+qd[16]*(OMcp23_115*ROcp23_515-OMcp23_215*ROcp23_415);
    RLcp23_159 = ROcp23_116*s->dpt[1][29]+ROcp23_415*s->dpt[2][29]+ROcp23_716*s->dpt[3][29];
    RLcp23_259 = ROcp23_216*s->dpt[1][29]+ROcp23_515*s->dpt[2][29]+ROcp23_816*s->dpt[3][29];
    RLcp23_359 = ROcp23_316*s->dpt[1][29]+ROcp23_615*s->dpt[2][29]+ROcp23_916*s->dpt[3][29];
    POcp23_159 = RLcp23_113+RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_159+q[1];
    POcp23_259 = RLcp23_213+RLcp23_214+RLcp23_215+RLcp23_216+RLcp23_259+q[2];
    POcp23_359 = RLcp23_313+RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_359+q[3];
    JTcp23_259_4 = -(RLcp23_313+RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_359);
    JTcp23_359_4 = RLcp23_213+RLcp23_214+RLcp23_215+RLcp23_216+RLcp23_259;
    JTcp23_159_5 = C4*(RLcp23_313+RLcp23_314+RLcp23_315+RLcp23_316)-S4*(RLcp23_213+RLcp23_214)-S4*(RLcp23_215+RLcp23_216)-
 RLcp23_259*S4+RLcp23_359*C4;
    JTcp23_259_5 = S4*(RLcp23_113+RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_159);
    JTcp23_359_5 = -C4*(RLcp23_113+RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_159);
    JTcp23_159_6 = ROcp23_85*(RLcp23_313+RLcp23_314+RLcp23_315+RLcp23_316)-ROcp23_95*(RLcp23_213+RLcp23_214)-ROcp23_95*(
 RLcp23_215+RLcp23_216)-RLcp23_259*ROcp23_95+RLcp23_359*ROcp23_85;
    JTcp23_259_6 = -(RLcp23_359*S5-ROcp23_95*(RLcp23_113+RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_159)+S5*(RLcp23_313+
 RLcp23_314)+S5*(RLcp23_315+RLcp23_316));
    JTcp23_359_6 = RLcp23_259*S5-ROcp23_85*(RLcp23_113+RLcp23_114+RLcp23_115+RLcp23_116+RLcp23_159)+S5*(RLcp23_213+
 RLcp23_214)+S5*(RLcp23_215+RLcp23_216);
    JTcp23_159_7 = ROcp23_56*(RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_359)-ROcp23_66*(RLcp23_214+RLcp23_215)-ROcp23_66*(
 RLcp23_216+RLcp23_259);
    JTcp23_259_7 = -(ROcp23_46*(RLcp23_314+RLcp23_315+RLcp23_316+RLcp23_359)-ROcp23_66*(RLcp23_114+RLcp23_115)-ROcp23_66*(
 RLcp23_116+RLcp23_159));
    JTcp23_359_7 = ROcp23_46*(RLcp23_214+RLcp23_215+RLcp23_216+RLcp23_259)-ROcp23_56*(RLcp23_114+RLcp23_115)-ROcp23_56*(
 RLcp23_116+RLcp23_159);
    JTcp23_159_8 = ROcp23_213*(RLcp23_315+RLcp23_316)-ROcp23_313*(RLcp23_215+RLcp23_216)-RLcp23_259*ROcp23_313+RLcp23_359*
 ROcp23_213;
    JTcp23_259_8 = RLcp23_159*ROcp23_313-RLcp23_359*ROcp23_113-ROcp23_113*(RLcp23_315+RLcp23_316)+ROcp23_313*(RLcp23_115+
 RLcp23_116);
    JTcp23_359_8 = ROcp23_113*(RLcp23_215+RLcp23_216)-ROcp23_213*(RLcp23_115+RLcp23_116)-RLcp23_159*ROcp23_213+RLcp23_259*
 ROcp23_113;
    JTcp23_159_9 = ROcp23_814*(RLcp23_316+RLcp23_359)-ROcp23_914*(RLcp23_216+RLcp23_259);
    JTcp23_259_9 = -(ROcp23_714*(RLcp23_316+RLcp23_359)-ROcp23_914*(RLcp23_116+RLcp23_159));
    JTcp23_359_9 = ROcp23_714*(RLcp23_216+RLcp23_259)-ROcp23_814*(RLcp23_116+RLcp23_159);
    JTcp23_159_10 = -(RLcp23_259*ROcp23_615-RLcp23_359*ROcp23_515);
    JTcp23_259_10 = RLcp23_159*ROcp23_615-RLcp23_359*ROcp23_415;
    JTcp23_359_10 = -(RLcp23_159*ROcp23_515-RLcp23_259*ROcp23_415);
    ORcp23_159 = OMcp23_216*RLcp23_359-OMcp23_316*RLcp23_259;
    ORcp23_259 = -(OMcp23_116*RLcp23_359-OMcp23_316*RLcp23_159);
    ORcp23_359 = OMcp23_116*RLcp23_259-OMcp23_216*RLcp23_159;
    VIcp23_159 = ORcp23_113+ORcp23_114+ORcp23_115+ORcp23_116+ORcp23_159+qd[1];
    VIcp23_259 = ORcp23_213+ORcp23_214+ORcp23_215+ORcp23_216+ORcp23_259+qd[2];
    VIcp23_359 = ORcp23_313+ORcp23_314+ORcp23_315+ORcp23_316+ORcp23_359+qd[3];
    ACcp23_159 = qdd[1]+OMcp23_213*ORcp23_314+OMcp23_214*ORcp23_315+OMcp23_215*ORcp23_316+OMcp23_216*ORcp23_359+OMcp23_26*
 ORcp23_313-OMcp23_313*ORcp23_214-OMcp23_314*ORcp23_215-OMcp23_315*ORcp23_216-OMcp23_316*ORcp23_259-OMcp23_36*ORcp23_213+
 OPcp23_213*RLcp23_314+OPcp23_214*RLcp23_315+OPcp23_215*RLcp23_316+OPcp23_216*RLcp23_359+OPcp23_26*RLcp23_313-OPcp23_313*
 RLcp23_214-OPcp23_314*RLcp23_215-OPcp23_315*RLcp23_216-OPcp23_316*RLcp23_259-OPcp23_36*RLcp23_213;
    ACcp23_259 = qdd[2]-OMcp23_113*ORcp23_314-OMcp23_114*ORcp23_315-OMcp23_115*ORcp23_316-OMcp23_116*ORcp23_359-OMcp23_16*
 ORcp23_313+OMcp23_313*ORcp23_114+OMcp23_314*ORcp23_115+OMcp23_315*ORcp23_116+OMcp23_316*ORcp23_159+OMcp23_36*ORcp23_113-
 OPcp23_113*RLcp23_314-OPcp23_114*RLcp23_315-OPcp23_115*RLcp23_316-OPcp23_116*RLcp23_359-OPcp23_16*RLcp23_313+OPcp23_313*
 RLcp23_114+OPcp23_314*RLcp23_115+OPcp23_315*RLcp23_116+OPcp23_316*RLcp23_159+OPcp23_36*RLcp23_113;
    ACcp23_359 = qdd[3]+OMcp23_113*ORcp23_214+OMcp23_114*ORcp23_215+OMcp23_115*ORcp23_216+OMcp23_116*ORcp23_259+OMcp23_16*
 ORcp23_213-OMcp23_213*ORcp23_114-OMcp23_214*ORcp23_115-OMcp23_215*ORcp23_116-OMcp23_216*ORcp23_159-OMcp23_26*ORcp23_113+
 OPcp23_113*RLcp23_214+OPcp23_114*RLcp23_215+OPcp23_115*RLcp23_216+OPcp23_116*RLcp23_259+OPcp23_16*RLcp23_213-OPcp23_213*
 RLcp23_114-OPcp23_214*RLcp23_115-OPcp23_215*RLcp23_116-OPcp23_216*RLcp23_159-OPcp23_26*RLcp23_113;

// = = Block_1_0_0_24_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp23_159;
    sens->P[2] = POcp23_259;
    sens->P[3] = POcp23_359;
    sens->R[1][1] = ROcp23_116;
    sens->R[1][2] = ROcp23_216;
    sens->R[1][3] = ROcp23_316;
    sens->R[2][1] = ROcp23_415;
    sens->R[2][2] = ROcp23_515;
    sens->R[2][3] = ROcp23_615;
    sens->R[3][1] = ROcp23_716;
    sens->R[3][2] = ROcp23_816;
    sens->R[3][3] = ROcp23_916;
    sens->V[1] = VIcp23_159;
    sens->V[2] = VIcp23_259;
    sens->V[3] = VIcp23_359;
    sens->OM[1] = OMcp23_116;
    sens->OM[2] = OMcp23_216;
    sens->OM[3] = OMcp23_316;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp23_159_5;
    sens->J[1][6] = JTcp23_159_6;
    sens->J[1][13] = JTcp23_159_7;
    sens->J[1][14] = JTcp23_159_8;
    sens->J[1][15] = JTcp23_159_9;
    sens->J[1][16] = JTcp23_159_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp23_259_4;
    sens->J[2][5] = JTcp23_259_5;
    sens->J[2][6] = JTcp23_259_6;
    sens->J[2][13] = JTcp23_259_7;
    sens->J[2][14] = JTcp23_259_8;
    sens->J[2][15] = JTcp23_259_9;
    sens->J[2][16] = JTcp23_259_10;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp23_359_4;
    sens->J[3][5] = JTcp23_359_5;
    sens->J[3][6] = JTcp23_359_6;
    sens->J[3][13] = JTcp23_359_7;
    sens->J[3][14] = JTcp23_359_8;
    sens->J[3][15] = JTcp23_359_9;
    sens->J[3][16] = JTcp23_359_10;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp23_46;
    sens->J[4][14] = ROcp23_113;
    sens->J[4][15] = ROcp23_714;
    sens->J[4][16] = ROcp23_415;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp23_85;
    sens->J[5][13] = ROcp23_56;
    sens->J[5][14] = ROcp23_213;
    sens->J[5][15] = ROcp23_814;
    sens->J[5][16] = ROcp23_515;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp23_95;
    sens->J[6][13] = ROcp23_66;
    sens->J[6][14] = ROcp23_313;
    sens->J[6][15] = ROcp23_914;
    sens->J[6][16] = ROcp23_615;
    sens->A[1] = ACcp23_159;
    sens->A[2] = ACcp23_259;
    sens->A[3] = ACcp23_359;
    sens->OMP[1] = OPcp23_116;
    sens->OMP[2] = OPcp23_216;
    sens->OMP[3] = OPcp23_316;
 
// 
break;
case 25:
 


// = = Block_1_0_0_25_0_1 = = 
 
// Sensor Kinematics 


    ROcp24_25 = S4*S5;
    ROcp24_35 = -C4*S5;
    ROcp24_85 = -S4*C5;
    ROcp24_95 = C4*C5;
    ROcp24_16 = C5*C6;
    ROcp24_26 = ROcp24_25*C6+C4*S6;
    ROcp24_36 = ROcp24_35*C6+S4*S6;
    ROcp24_46 = -C5*S6;
    ROcp24_56 = -(ROcp24_25*S6-C4*C6);
    ROcp24_66 = -(ROcp24_35*S6-S4*C6);
    OMcp24_25 = qd[5]*C4;
    OMcp24_35 = qd[5]*S4;
    OMcp24_16 = qd[4]+qd[6]*S5;
    OMcp24_26 = OMcp24_25+ROcp24_85*qd[6];
    OMcp24_36 = OMcp24_35+ROcp24_95*qd[6];
    OPcp24_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp24_26 = ROcp24_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp24_35*S5-ROcp24_95*qd[4]);
    OPcp24_36 = ROcp24_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp24_25*S5-ROcp24_85*qd[4]);

// = = Block_1_0_0_25_0_3 = = 
 
// Sensor Kinematics 


    ROcp24_113 = ROcp24_16*C13-S13*S5;
    ROcp24_213 = ROcp24_26*C13-ROcp24_85*S13;
    ROcp24_313 = ROcp24_36*C13-ROcp24_95*S13;
    ROcp24_713 = ROcp24_16*S13+C13*S5;
    ROcp24_813 = ROcp24_26*S13+ROcp24_85*C13;
    ROcp24_913 = ROcp24_36*S13+ROcp24_95*C13;
    ROcp24_414 = ROcp24_46*C14+ROcp24_713*S14;
    ROcp24_514 = ROcp24_56*C14+ROcp24_813*S14;
    ROcp24_614 = ROcp24_66*C14+ROcp24_913*S14;
    ROcp24_714 = -(ROcp24_46*S14-ROcp24_713*C14);
    ROcp24_814 = -(ROcp24_56*S14-ROcp24_813*C14);
    ROcp24_914 = -(ROcp24_66*S14-ROcp24_913*C14);
    ROcp24_115 = ROcp24_113*C15+ROcp24_414*S15;
    ROcp24_215 = ROcp24_213*C15+ROcp24_514*S15;
    ROcp24_315 = ROcp24_313*C15+ROcp24_614*S15;
    ROcp24_415 = -(ROcp24_113*S15-ROcp24_414*C15);
    ROcp24_515 = -(ROcp24_213*S15-ROcp24_514*C15);
    ROcp24_615 = -(ROcp24_313*S15-ROcp24_614*C15);
    ROcp24_116 = ROcp24_115*C16-ROcp24_714*S16;
    ROcp24_216 = ROcp24_215*C16-ROcp24_814*S16;
    ROcp24_316 = ROcp24_315*C16-ROcp24_914*S16;
    ROcp24_716 = ROcp24_115*S16+ROcp24_714*C16;
    ROcp24_816 = ROcp24_215*S16+ROcp24_814*C16;
    ROcp24_916 = ROcp24_315*S16+ROcp24_914*C16;
    ROcp24_417 = ROcp24_415*C17+ROcp24_716*S17;
    ROcp24_517 = ROcp24_515*C17+ROcp24_816*S17;
    ROcp24_617 = ROcp24_615*C17+ROcp24_916*S17;
    ROcp24_717 = -(ROcp24_415*S17-ROcp24_716*C17);
    ROcp24_817 = -(ROcp24_515*S17-ROcp24_816*C17);
    ROcp24_917 = -(ROcp24_615*S17-ROcp24_916*C17);
    RLcp24_113 = s->dpt[1][2]*ROcp24_16+s->dpt[3][2]*S5+ROcp24_46*s->dpt[2][2];
    RLcp24_213 = s->dpt[1][2]*ROcp24_26+s->dpt[3][2]*ROcp24_85+ROcp24_56*s->dpt[2][2];
    RLcp24_313 = s->dpt[1][2]*ROcp24_36+s->dpt[3][2]*ROcp24_95+ROcp24_66*s->dpt[2][2];
    OMcp24_113 = OMcp24_16+ROcp24_46*qd[13];
    OMcp24_213 = OMcp24_26+ROcp24_56*qd[13];
    OMcp24_313 = OMcp24_36+ROcp24_66*qd[13];
    ORcp24_113 = OMcp24_26*RLcp24_313-OMcp24_36*RLcp24_213;
    ORcp24_213 = -(OMcp24_16*RLcp24_313-OMcp24_36*RLcp24_113);
    ORcp24_313 = OMcp24_16*RLcp24_213-OMcp24_26*RLcp24_113;
    OPcp24_113 = OPcp24_16+ROcp24_46*qdd[13]+qd[13]*(OMcp24_26*ROcp24_66-OMcp24_36*ROcp24_56);
    OPcp24_213 = OPcp24_26+ROcp24_56*qdd[13]-qd[13]*(OMcp24_16*ROcp24_66-OMcp24_36*ROcp24_46);
    OPcp24_313 = OPcp24_36+ROcp24_66*qdd[13]+qd[13]*(OMcp24_16*ROcp24_56-OMcp24_26*ROcp24_46);
    RLcp24_114 = s->dpt[1][22]*ROcp24_113+s->dpt[3][22]*ROcp24_713+ROcp24_46*s->dpt[2][22];
    RLcp24_214 = s->dpt[1][22]*ROcp24_213+s->dpt[3][22]*ROcp24_813+ROcp24_56*s->dpt[2][22];
    RLcp24_314 = s->dpt[1][22]*ROcp24_313+s->dpt[3][22]*ROcp24_913+ROcp24_66*s->dpt[2][22];
    OMcp24_114 = OMcp24_113+ROcp24_113*qd[14];
    OMcp24_214 = OMcp24_213+ROcp24_213*qd[14];
    OMcp24_314 = OMcp24_313+ROcp24_313*qd[14];
    ORcp24_114 = OMcp24_213*RLcp24_314-OMcp24_313*RLcp24_214;
    ORcp24_214 = -(OMcp24_113*RLcp24_314-OMcp24_313*RLcp24_114);
    ORcp24_314 = OMcp24_113*RLcp24_214-OMcp24_213*RLcp24_114;
    OPcp24_114 = OPcp24_113+ROcp24_113*qdd[14]+qd[14]*(OMcp24_213*ROcp24_313-OMcp24_313*ROcp24_213);
    OPcp24_214 = OPcp24_213+ROcp24_213*qdd[14]-qd[14]*(OMcp24_113*ROcp24_313-OMcp24_313*ROcp24_113);
    OPcp24_314 = OPcp24_313+ROcp24_313*qdd[14]+qd[14]*(OMcp24_113*ROcp24_213-OMcp24_213*ROcp24_113);
    RLcp24_115 = s->dpt[1][24]*ROcp24_113+s->dpt[2][24]*ROcp24_414+ROcp24_714*s->dpt[3][24];
    RLcp24_215 = s->dpt[1][24]*ROcp24_213+s->dpt[2][24]*ROcp24_514+ROcp24_814*s->dpt[3][24];
    RLcp24_315 = s->dpt[1][24]*ROcp24_313+s->dpt[2][24]*ROcp24_614+ROcp24_914*s->dpt[3][24];
    OMcp24_115 = OMcp24_114+ROcp24_714*qd[15];
    OMcp24_215 = OMcp24_214+ROcp24_814*qd[15];
    OMcp24_315 = OMcp24_314+ROcp24_914*qd[15];
    ORcp24_115 = OMcp24_214*RLcp24_315-OMcp24_314*RLcp24_215;
    ORcp24_215 = -(OMcp24_114*RLcp24_315-OMcp24_314*RLcp24_115);
    ORcp24_315 = OMcp24_114*RLcp24_215-OMcp24_214*RLcp24_115;
    OPcp24_115 = OPcp24_114+ROcp24_714*qdd[15]+qd[15]*(OMcp24_214*ROcp24_914-OMcp24_314*ROcp24_814);
    OPcp24_215 = OPcp24_214+ROcp24_814*qdd[15]-qd[15]*(OMcp24_114*ROcp24_914-OMcp24_314*ROcp24_714);
    OPcp24_315 = OPcp24_314+ROcp24_914*qdd[15]+qd[15]*(OMcp24_114*ROcp24_814-OMcp24_214*ROcp24_714);
    RLcp24_116 = s->dpt[1][26]*ROcp24_115+s->dpt[2][26]*ROcp24_415+ROcp24_714*s->dpt[3][26];
    RLcp24_216 = s->dpt[1][26]*ROcp24_215+s->dpt[2][26]*ROcp24_515+ROcp24_814*s->dpt[3][26];
    RLcp24_316 = s->dpt[1][26]*ROcp24_315+s->dpt[2][26]*ROcp24_615+ROcp24_914*s->dpt[3][26];
    OMcp24_116 = OMcp24_115+ROcp24_415*qd[16];
    OMcp24_216 = OMcp24_215+ROcp24_515*qd[16];
    OMcp24_316 = OMcp24_315+ROcp24_615*qd[16];
    ORcp24_116 = OMcp24_215*RLcp24_316-OMcp24_315*RLcp24_216;
    ORcp24_216 = -(OMcp24_115*RLcp24_316-OMcp24_315*RLcp24_116);
    ORcp24_316 = OMcp24_115*RLcp24_216-OMcp24_215*RLcp24_116;
    OPcp24_116 = OPcp24_115+ROcp24_415*qdd[16]+qd[16]*(OMcp24_215*ROcp24_615-OMcp24_315*ROcp24_515);
    OPcp24_216 = OPcp24_215+ROcp24_515*qdd[16]-qd[16]*(OMcp24_115*ROcp24_615-OMcp24_315*ROcp24_415);
    OPcp24_316 = OPcp24_315+ROcp24_615*qdd[16]+qd[16]*(OMcp24_115*ROcp24_515-OMcp24_215*ROcp24_415);
    RLcp24_117 = s->dpt[1][28]*ROcp24_116+s->dpt[2][28]*ROcp24_415+ROcp24_716*s->dpt[3][28];
    RLcp24_217 = s->dpt[1][28]*ROcp24_216+s->dpt[2][28]*ROcp24_515+ROcp24_816*s->dpt[3][28];
    RLcp24_317 = s->dpt[1][28]*ROcp24_316+s->dpt[2][28]*ROcp24_615+ROcp24_916*s->dpt[3][28];
    OMcp24_117 = OMcp24_116+ROcp24_116*qd[17];
    OMcp24_217 = OMcp24_216+ROcp24_216*qd[17];
    OMcp24_317 = OMcp24_316+ROcp24_316*qd[17];
    ORcp24_117 = OMcp24_216*RLcp24_317-OMcp24_316*RLcp24_217;
    ORcp24_217 = -(OMcp24_116*RLcp24_317-OMcp24_316*RLcp24_117);
    ORcp24_317 = OMcp24_116*RLcp24_217-OMcp24_216*RLcp24_117;
    OPcp24_117 = OPcp24_116+ROcp24_116*qdd[17]+qd[17]*(OMcp24_216*ROcp24_316-OMcp24_316*ROcp24_216);
    OPcp24_217 = OPcp24_216+ROcp24_216*qdd[17]-qd[17]*(OMcp24_116*ROcp24_316-OMcp24_316*ROcp24_116);
    OPcp24_317 = OPcp24_316+ROcp24_316*qdd[17]+qd[17]*(OMcp24_116*ROcp24_216-OMcp24_216*ROcp24_116);
    RLcp24_160 = ROcp24_116*s->dpt[1][31]+ROcp24_417*s->dpt[2][31]+ROcp24_717*s->dpt[3][31];
    RLcp24_260 = ROcp24_216*s->dpt[1][31]+ROcp24_517*s->dpt[2][31]+ROcp24_817*s->dpt[3][31];
    RLcp24_360 = ROcp24_316*s->dpt[1][31]+ROcp24_617*s->dpt[2][31]+ROcp24_917*s->dpt[3][31];
    POcp24_160 = RLcp24_113+RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_160+q[1];
    POcp24_260 = RLcp24_213+RLcp24_214+RLcp24_215+RLcp24_216+RLcp24_217+RLcp24_260+q[2];
    POcp24_360 = RLcp24_313+RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_360+q[3];
    JTcp24_260_4 = -(RLcp24_313+RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_360);
    JTcp24_360_4 = RLcp24_213+RLcp24_214+RLcp24_215+RLcp24_216+RLcp24_217+RLcp24_260;
    JTcp24_160_5 = C4*(RLcp24_313+RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_360)-S4*(RLcp24_213+RLcp24_214)-S4*(
 RLcp24_215+RLcp24_216)-S4*(RLcp24_217+RLcp24_260);
    JTcp24_260_5 = S4*(RLcp24_113+RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_160);
    JTcp24_360_5 = -C4*(RLcp24_113+RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117+RLcp24_160);
    JTcp24_160_6 = ROcp24_85*(RLcp24_313+RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_360)-ROcp24_95*(RLcp24_213+
 RLcp24_214)-ROcp24_95*(RLcp24_215+RLcp24_216)-ROcp24_95*(RLcp24_217+RLcp24_260);
    JTcp24_260_6 = RLcp24_160*ROcp24_95-RLcp24_317*S5-RLcp24_360*S5+ROcp24_95*(RLcp24_113+RLcp24_114+RLcp24_115+RLcp24_116
 +RLcp24_117)-S5*(RLcp24_313+RLcp24_314)-S5*(RLcp24_315+RLcp24_316);
    JTcp24_360_6 = RLcp24_217*S5-ROcp24_85*(RLcp24_113+RLcp24_114+RLcp24_115+RLcp24_116+RLcp24_117)+S5*(RLcp24_213+
 RLcp24_214)+S5*(RLcp24_215+RLcp24_216)-RLcp24_160*ROcp24_85+RLcp24_260*S5;
    JTcp24_160_7 = ROcp24_56*(RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317)-ROcp24_66*(RLcp24_214+RLcp24_215)-ROcp24_66*(
 RLcp24_216+RLcp24_217)-RLcp24_260*ROcp24_66+RLcp24_360*ROcp24_56;
    JTcp24_260_7 = RLcp24_160*ROcp24_66-RLcp24_360*ROcp24_46-ROcp24_46*(RLcp24_314+RLcp24_315+RLcp24_316+RLcp24_317)+
 ROcp24_66*(RLcp24_114+RLcp24_115)+ROcp24_66*(RLcp24_116+RLcp24_117);
    JTcp24_360_7 = ROcp24_46*(RLcp24_214+RLcp24_215+RLcp24_216+RLcp24_217)-ROcp24_56*(RLcp24_114+RLcp24_115)-ROcp24_56*(
 RLcp24_116+RLcp24_117)-RLcp24_160*ROcp24_56+RLcp24_260*ROcp24_46;
    JTcp24_160_8 = ROcp24_213*(RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_360)-ROcp24_313*(RLcp24_215+RLcp24_216)-ROcp24_313*
 (RLcp24_217+RLcp24_260);
    JTcp24_260_8 = -(ROcp24_113*(RLcp24_315+RLcp24_316+RLcp24_317+RLcp24_360)-ROcp24_313*(RLcp24_115+RLcp24_116)-
 ROcp24_313*(RLcp24_117+RLcp24_160));
    JTcp24_360_8 = ROcp24_113*(RLcp24_215+RLcp24_216+RLcp24_217+RLcp24_260)-ROcp24_213*(RLcp24_115+RLcp24_116)-ROcp24_213*
 (RLcp24_117+RLcp24_160);
    JTcp24_160_9 = ROcp24_814*(RLcp24_316+RLcp24_317)-ROcp24_914*(RLcp24_216+RLcp24_217)-RLcp24_260*ROcp24_914+RLcp24_360*
 ROcp24_814;
    JTcp24_260_9 = RLcp24_160*ROcp24_914-RLcp24_360*ROcp24_714-ROcp24_714*(RLcp24_316+RLcp24_317)+ROcp24_914*(RLcp24_116+
 RLcp24_117);
    JTcp24_360_9 = ROcp24_714*(RLcp24_216+RLcp24_217)-ROcp24_814*(RLcp24_116+RLcp24_117)-RLcp24_160*ROcp24_814+RLcp24_260*
 ROcp24_714;
    JTcp24_160_10 = ROcp24_515*(RLcp24_317+RLcp24_360)-ROcp24_615*(RLcp24_217+RLcp24_260);
    JTcp24_260_10 = -(ROcp24_415*(RLcp24_317+RLcp24_360)-ROcp24_615*(RLcp24_117+RLcp24_160));
    JTcp24_360_10 = ROcp24_415*(RLcp24_217+RLcp24_260)-ROcp24_515*(RLcp24_117+RLcp24_160);
    JTcp24_160_11 = -(RLcp24_260*ROcp24_316-RLcp24_360*ROcp24_216);
    JTcp24_260_11 = RLcp24_160*ROcp24_316-RLcp24_360*ROcp24_116;
    JTcp24_360_11 = -(RLcp24_160*ROcp24_216-RLcp24_260*ROcp24_116);
    ORcp24_160 = OMcp24_217*RLcp24_360-OMcp24_317*RLcp24_260;
    ORcp24_260 = -(OMcp24_117*RLcp24_360-OMcp24_317*RLcp24_160);
    ORcp24_360 = OMcp24_117*RLcp24_260-OMcp24_217*RLcp24_160;
    VIcp24_160 = ORcp24_113+ORcp24_114+ORcp24_115+ORcp24_116+ORcp24_117+ORcp24_160+qd[1];
    VIcp24_260 = ORcp24_213+ORcp24_214+ORcp24_215+ORcp24_216+ORcp24_217+ORcp24_260+qd[2];
    VIcp24_360 = ORcp24_313+ORcp24_314+ORcp24_315+ORcp24_316+ORcp24_317+ORcp24_360+qd[3];
    ACcp24_160 = qdd[1]+OMcp24_213*ORcp24_314+OMcp24_214*ORcp24_315+OMcp24_215*ORcp24_316+OMcp24_216*ORcp24_317+OMcp24_217
 *ORcp24_360+OMcp24_26*ORcp24_313-OMcp24_313*ORcp24_214-OMcp24_314*ORcp24_215-OMcp24_315*ORcp24_216-OMcp24_316*ORcp24_217-
 OMcp24_317*ORcp24_260-OMcp24_36*ORcp24_213+OPcp24_213*RLcp24_314+OPcp24_214*RLcp24_315+OPcp24_215*RLcp24_316+OPcp24_216*
 RLcp24_317+OPcp24_217*RLcp24_360+OPcp24_26*RLcp24_313-OPcp24_313*RLcp24_214-OPcp24_314*RLcp24_215-OPcp24_315*RLcp24_216-
 OPcp24_316*RLcp24_217-OPcp24_317*RLcp24_260-OPcp24_36*RLcp24_213;
    ACcp24_260 = qdd[2]-OMcp24_113*ORcp24_314-OMcp24_114*ORcp24_315-OMcp24_115*ORcp24_316-OMcp24_116*ORcp24_317-OMcp24_117
 *ORcp24_360-OMcp24_16*ORcp24_313+OMcp24_313*ORcp24_114+OMcp24_314*ORcp24_115+OMcp24_315*ORcp24_116+OMcp24_316*ORcp24_117+
 OMcp24_317*ORcp24_160+OMcp24_36*ORcp24_113-OPcp24_113*RLcp24_314-OPcp24_114*RLcp24_315-OPcp24_115*RLcp24_316-OPcp24_116*
 RLcp24_317-OPcp24_117*RLcp24_360-OPcp24_16*RLcp24_313+OPcp24_313*RLcp24_114+OPcp24_314*RLcp24_115+OPcp24_315*RLcp24_116+
 OPcp24_316*RLcp24_117+OPcp24_317*RLcp24_160+OPcp24_36*RLcp24_113;
    ACcp24_360 = qdd[3]+OMcp24_113*ORcp24_214+OMcp24_114*ORcp24_215+OMcp24_115*ORcp24_216+OMcp24_116*ORcp24_217+OMcp24_117
 *ORcp24_260+OMcp24_16*ORcp24_213-OMcp24_213*ORcp24_114-OMcp24_214*ORcp24_115-OMcp24_215*ORcp24_116-OMcp24_216*ORcp24_117-
 OMcp24_217*ORcp24_160-OMcp24_26*ORcp24_113+OPcp24_113*RLcp24_214+OPcp24_114*RLcp24_215+OPcp24_115*RLcp24_216+OPcp24_116*
 RLcp24_217+OPcp24_117*RLcp24_260+OPcp24_16*RLcp24_213-OPcp24_213*RLcp24_114-OPcp24_214*RLcp24_115-OPcp24_215*RLcp24_116-
 OPcp24_216*RLcp24_117-OPcp24_217*RLcp24_160-OPcp24_26*RLcp24_113;

// = = Block_1_0_0_25_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp24_160;
    sens->P[2] = POcp24_260;
    sens->P[3] = POcp24_360;
    sens->R[1][1] = ROcp24_116;
    sens->R[1][2] = ROcp24_216;
    sens->R[1][3] = ROcp24_316;
    sens->R[2][1] = ROcp24_417;
    sens->R[2][2] = ROcp24_517;
    sens->R[2][3] = ROcp24_617;
    sens->R[3][1] = ROcp24_717;
    sens->R[3][2] = ROcp24_817;
    sens->R[3][3] = ROcp24_917;
    sens->V[1] = VIcp24_160;
    sens->V[2] = VIcp24_260;
    sens->V[3] = VIcp24_360;
    sens->OM[1] = OMcp24_117;
    sens->OM[2] = OMcp24_217;
    sens->OM[3] = OMcp24_317;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp24_160_5;
    sens->J[1][6] = JTcp24_160_6;
    sens->J[1][13] = JTcp24_160_7;
    sens->J[1][14] = JTcp24_160_8;
    sens->J[1][15] = JTcp24_160_9;
    sens->J[1][16] = JTcp24_160_10;
    sens->J[1][17] = JTcp24_160_11;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp24_260_4;
    sens->J[2][5] = JTcp24_260_5;
    sens->J[2][6] = JTcp24_260_6;
    sens->J[2][13] = JTcp24_260_7;
    sens->J[2][14] = JTcp24_260_8;
    sens->J[2][15] = JTcp24_260_9;
    sens->J[2][16] = JTcp24_260_10;
    sens->J[2][17] = JTcp24_260_11;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp24_360_4;
    sens->J[3][5] = JTcp24_360_5;
    sens->J[3][6] = JTcp24_360_6;
    sens->J[3][13] = JTcp24_360_7;
    sens->J[3][14] = JTcp24_360_8;
    sens->J[3][15] = JTcp24_360_9;
    sens->J[3][16] = JTcp24_360_10;
    sens->J[3][17] = JTcp24_360_11;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp24_46;
    sens->J[4][14] = ROcp24_113;
    sens->J[4][15] = ROcp24_714;
    sens->J[4][16] = ROcp24_415;
    sens->J[4][17] = ROcp24_116;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp24_85;
    sens->J[5][13] = ROcp24_56;
    sens->J[5][14] = ROcp24_213;
    sens->J[5][15] = ROcp24_814;
    sens->J[5][16] = ROcp24_515;
    sens->J[5][17] = ROcp24_216;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp24_95;
    sens->J[6][13] = ROcp24_66;
    sens->J[6][14] = ROcp24_313;
    sens->J[6][15] = ROcp24_914;
    sens->J[6][16] = ROcp24_615;
    sens->J[6][17] = ROcp24_316;
    sens->A[1] = ACcp24_160;
    sens->A[2] = ACcp24_260;
    sens->A[3] = ACcp24_360;
    sens->OMP[1] = OPcp24_117;
    sens->OMP[2] = OPcp24_217;
    sens->OMP[3] = OPcp24_317;
 
// 
break;
case 26:
 


// = = Block_1_0_0_26_0_1 = = 
 
// Sensor Kinematics 


    ROcp25_25 = S4*S5;
    ROcp25_35 = -C4*S5;
    ROcp25_85 = -S4*C5;
    ROcp25_95 = C4*C5;
    ROcp25_16 = C5*C6;
    ROcp25_26 = ROcp25_25*C6+C4*S6;
    ROcp25_36 = ROcp25_35*C6+S4*S6;
    ROcp25_46 = -C5*S6;
    ROcp25_56 = -(ROcp25_25*S6-C4*C6);
    ROcp25_66 = -(ROcp25_35*S6-S4*C6);
    OMcp25_25 = qd[5]*C4;
    OMcp25_35 = qd[5]*S4;
    OMcp25_16 = qd[4]+qd[6]*S5;
    OMcp25_26 = OMcp25_25+ROcp25_85*qd[6];
    OMcp25_36 = OMcp25_35+ROcp25_95*qd[6];
    OPcp25_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp25_26 = ROcp25_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp25_35*S5-ROcp25_95*qd[4]);
    OPcp25_36 = ROcp25_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp25_25*S5-ROcp25_85*qd[4]);

// = = Block_1_0_0_26_0_3 = = 
 
// Sensor Kinematics 


    ROcp25_113 = ROcp25_16*C13-S13*S5;
    ROcp25_213 = ROcp25_26*C13-ROcp25_85*S13;
    ROcp25_313 = ROcp25_36*C13-ROcp25_95*S13;
    ROcp25_713 = ROcp25_16*S13+C13*S5;
    ROcp25_813 = ROcp25_26*S13+ROcp25_85*C13;
    ROcp25_913 = ROcp25_36*S13+ROcp25_95*C13;
    ROcp25_414 = ROcp25_46*C14+ROcp25_713*S14;
    ROcp25_514 = ROcp25_56*C14+ROcp25_813*S14;
    ROcp25_614 = ROcp25_66*C14+ROcp25_913*S14;
    ROcp25_714 = -(ROcp25_46*S14-ROcp25_713*C14);
    ROcp25_814 = -(ROcp25_56*S14-ROcp25_813*C14);
    ROcp25_914 = -(ROcp25_66*S14-ROcp25_913*C14);
    ROcp25_115 = ROcp25_113*C15+ROcp25_414*S15;
    ROcp25_215 = ROcp25_213*C15+ROcp25_514*S15;
    ROcp25_315 = ROcp25_313*C15+ROcp25_614*S15;
    ROcp25_415 = -(ROcp25_113*S15-ROcp25_414*C15);
    ROcp25_515 = -(ROcp25_213*S15-ROcp25_514*C15);
    ROcp25_615 = -(ROcp25_313*S15-ROcp25_614*C15);
    ROcp25_116 = ROcp25_115*C16-ROcp25_714*S16;
    ROcp25_216 = ROcp25_215*C16-ROcp25_814*S16;
    ROcp25_316 = ROcp25_315*C16-ROcp25_914*S16;
    ROcp25_716 = ROcp25_115*S16+ROcp25_714*C16;
    ROcp25_816 = ROcp25_215*S16+ROcp25_814*C16;
    ROcp25_916 = ROcp25_315*S16+ROcp25_914*C16;
    ROcp25_417 = ROcp25_415*C17+ROcp25_716*S17;
    ROcp25_517 = ROcp25_515*C17+ROcp25_816*S17;
    ROcp25_617 = ROcp25_615*C17+ROcp25_916*S17;
    ROcp25_717 = -(ROcp25_415*S17-ROcp25_716*C17);
    ROcp25_817 = -(ROcp25_515*S17-ROcp25_816*C17);
    ROcp25_917 = -(ROcp25_615*S17-ROcp25_916*C17);
    ROcp25_118 = ROcp25_116*C18-ROcp25_717*S18;
    ROcp25_218 = ROcp25_216*C18-ROcp25_817*S18;
    ROcp25_318 = ROcp25_316*C18-ROcp25_917*S18;
    ROcp25_718 = ROcp25_116*S18+ROcp25_717*C18;
    ROcp25_818 = ROcp25_216*S18+ROcp25_817*C18;
    ROcp25_918 = ROcp25_316*S18+ROcp25_917*C18;
    RLcp25_113 = s->dpt[1][2]*ROcp25_16+s->dpt[3][2]*S5+ROcp25_46*s->dpt[2][2];
    RLcp25_213 = s->dpt[1][2]*ROcp25_26+s->dpt[3][2]*ROcp25_85+ROcp25_56*s->dpt[2][2];
    RLcp25_313 = s->dpt[1][2]*ROcp25_36+s->dpt[3][2]*ROcp25_95+ROcp25_66*s->dpt[2][2];
    OMcp25_113 = OMcp25_16+ROcp25_46*qd[13];
    OMcp25_213 = OMcp25_26+ROcp25_56*qd[13];
    OMcp25_313 = OMcp25_36+ROcp25_66*qd[13];
    ORcp25_113 = OMcp25_26*RLcp25_313-OMcp25_36*RLcp25_213;
    ORcp25_213 = -(OMcp25_16*RLcp25_313-OMcp25_36*RLcp25_113);
    ORcp25_313 = OMcp25_16*RLcp25_213-OMcp25_26*RLcp25_113;
    OPcp25_113 = OPcp25_16+ROcp25_46*qdd[13]+qd[13]*(OMcp25_26*ROcp25_66-OMcp25_36*ROcp25_56);
    OPcp25_213 = OPcp25_26+ROcp25_56*qdd[13]-qd[13]*(OMcp25_16*ROcp25_66-OMcp25_36*ROcp25_46);
    OPcp25_313 = OPcp25_36+ROcp25_66*qdd[13]+qd[13]*(OMcp25_16*ROcp25_56-OMcp25_26*ROcp25_46);
    RLcp25_114 = s->dpt[1][22]*ROcp25_113+s->dpt[3][22]*ROcp25_713+ROcp25_46*s->dpt[2][22];
    RLcp25_214 = s->dpt[1][22]*ROcp25_213+s->dpt[3][22]*ROcp25_813+ROcp25_56*s->dpt[2][22];
    RLcp25_314 = s->dpt[1][22]*ROcp25_313+s->dpt[3][22]*ROcp25_913+ROcp25_66*s->dpt[2][22];
    OMcp25_114 = OMcp25_113+ROcp25_113*qd[14];
    OMcp25_214 = OMcp25_213+ROcp25_213*qd[14];
    OMcp25_314 = OMcp25_313+ROcp25_313*qd[14];
    ORcp25_114 = OMcp25_213*RLcp25_314-OMcp25_313*RLcp25_214;
    ORcp25_214 = -(OMcp25_113*RLcp25_314-OMcp25_313*RLcp25_114);
    ORcp25_314 = OMcp25_113*RLcp25_214-OMcp25_213*RLcp25_114;
    OPcp25_114 = OPcp25_113+ROcp25_113*qdd[14]+qd[14]*(OMcp25_213*ROcp25_313-OMcp25_313*ROcp25_213);
    OPcp25_214 = OPcp25_213+ROcp25_213*qdd[14]-qd[14]*(OMcp25_113*ROcp25_313-OMcp25_313*ROcp25_113);
    OPcp25_314 = OPcp25_313+ROcp25_313*qdd[14]+qd[14]*(OMcp25_113*ROcp25_213-OMcp25_213*ROcp25_113);
    RLcp25_115 = s->dpt[1][24]*ROcp25_113+s->dpt[2][24]*ROcp25_414+ROcp25_714*s->dpt[3][24];
    RLcp25_215 = s->dpt[1][24]*ROcp25_213+s->dpt[2][24]*ROcp25_514+ROcp25_814*s->dpt[3][24];
    RLcp25_315 = s->dpt[1][24]*ROcp25_313+s->dpt[2][24]*ROcp25_614+ROcp25_914*s->dpt[3][24];
    OMcp25_115 = OMcp25_114+ROcp25_714*qd[15];
    OMcp25_215 = OMcp25_214+ROcp25_814*qd[15];
    OMcp25_315 = OMcp25_314+ROcp25_914*qd[15];
    ORcp25_115 = OMcp25_214*RLcp25_315-OMcp25_314*RLcp25_215;
    ORcp25_215 = -(OMcp25_114*RLcp25_315-OMcp25_314*RLcp25_115);
    ORcp25_315 = OMcp25_114*RLcp25_215-OMcp25_214*RLcp25_115;
    OPcp25_115 = OPcp25_114+ROcp25_714*qdd[15]+qd[15]*(OMcp25_214*ROcp25_914-OMcp25_314*ROcp25_814);
    OPcp25_215 = OPcp25_214+ROcp25_814*qdd[15]-qd[15]*(OMcp25_114*ROcp25_914-OMcp25_314*ROcp25_714);
    OPcp25_315 = OPcp25_314+ROcp25_914*qdd[15]+qd[15]*(OMcp25_114*ROcp25_814-OMcp25_214*ROcp25_714);
    RLcp25_116 = s->dpt[1][26]*ROcp25_115+s->dpt[2][26]*ROcp25_415+ROcp25_714*s->dpt[3][26];
    RLcp25_216 = s->dpt[1][26]*ROcp25_215+s->dpt[2][26]*ROcp25_515+ROcp25_814*s->dpt[3][26];
    RLcp25_316 = s->dpt[1][26]*ROcp25_315+s->dpt[2][26]*ROcp25_615+ROcp25_914*s->dpt[3][26];
    OMcp25_116 = OMcp25_115+ROcp25_415*qd[16];
    OMcp25_216 = OMcp25_215+ROcp25_515*qd[16];
    OMcp25_316 = OMcp25_315+ROcp25_615*qd[16];
    ORcp25_116 = OMcp25_215*RLcp25_316-OMcp25_315*RLcp25_216;
    ORcp25_216 = -(OMcp25_115*RLcp25_316-OMcp25_315*RLcp25_116);
    ORcp25_316 = OMcp25_115*RLcp25_216-OMcp25_215*RLcp25_116;
    OPcp25_116 = OPcp25_115+ROcp25_415*qdd[16]+qd[16]*(OMcp25_215*ROcp25_615-OMcp25_315*ROcp25_515);
    OPcp25_216 = OPcp25_215+ROcp25_515*qdd[16]-qd[16]*(OMcp25_115*ROcp25_615-OMcp25_315*ROcp25_415);
    OPcp25_316 = OPcp25_315+ROcp25_615*qdd[16]+qd[16]*(OMcp25_115*ROcp25_515-OMcp25_215*ROcp25_415);
    RLcp25_117 = s->dpt[1][28]*ROcp25_116+s->dpt[2][28]*ROcp25_415+ROcp25_716*s->dpt[3][28];
    RLcp25_217 = s->dpt[1][28]*ROcp25_216+s->dpt[2][28]*ROcp25_515+ROcp25_816*s->dpt[3][28];
    RLcp25_317 = s->dpt[1][28]*ROcp25_316+s->dpt[2][28]*ROcp25_615+ROcp25_916*s->dpt[3][28];
    OMcp25_117 = OMcp25_116+ROcp25_116*qd[17];
    OMcp25_217 = OMcp25_216+ROcp25_216*qd[17];
    OMcp25_317 = OMcp25_316+ROcp25_316*qd[17];
    ORcp25_117 = OMcp25_216*RLcp25_317-OMcp25_316*RLcp25_217;
    ORcp25_217 = -(OMcp25_116*RLcp25_317-OMcp25_316*RLcp25_117);
    ORcp25_317 = OMcp25_116*RLcp25_217-OMcp25_216*RLcp25_117;
    OPcp25_117 = OPcp25_116+ROcp25_116*qdd[17]+qd[17]*(OMcp25_216*ROcp25_316-OMcp25_316*ROcp25_216);
    OPcp25_217 = OPcp25_216+ROcp25_216*qdd[17]-qd[17]*(OMcp25_116*ROcp25_316-OMcp25_316*ROcp25_116);
    OPcp25_317 = OPcp25_316+ROcp25_316*qdd[17]+qd[17]*(OMcp25_116*ROcp25_216-OMcp25_216*ROcp25_116);
    RLcp25_118 = s->dpt[1][30]*ROcp25_116+s->dpt[2][30]*ROcp25_417+s->dpt[3][30]*ROcp25_717;
    RLcp25_218 = s->dpt[1][30]*ROcp25_216+s->dpt[2][30]*ROcp25_517+s->dpt[3][30]*ROcp25_817;
    RLcp25_318 = s->dpt[1][30]*ROcp25_316+s->dpt[2][30]*ROcp25_617+s->dpt[3][30]*ROcp25_917;
    OMcp25_118 = OMcp25_117+ROcp25_417*qd[18];
    OMcp25_218 = OMcp25_217+ROcp25_517*qd[18];
    OMcp25_318 = OMcp25_317+ROcp25_617*qd[18];
    ORcp25_118 = OMcp25_217*RLcp25_318-OMcp25_317*RLcp25_218;
    ORcp25_218 = -(OMcp25_117*RLcp25_318-OMcp25_317*RLcp25_118);
    ORcp25_318 = OMcp25_117*RLcp25_218-OMcp25_217*RLcp25_118;
    OPcp25_118 = OPcp25_117+ROcp25_417*qdd[18]+qd[18]*(OMcp25_217*ROcp25_617-OMcp25_317*ROcp25_517);
    OPcp25_218 = OPcp25_217+ROcp25_517*qdd[18]-qd[18]*(OMcp25_117*ROcp25_617-OMcp25_317*ROcp25_417);
    OPcp25_318 = OPcp25_317+ROcp25_617*qdd[18]+qd[18]*(OMcp25_117*ROcp25_517-OMcp25_217*ROcp25_417);
    RLcp25_161 = ROcp25_118*s->dpt[1][32]+ROcp25_417*s->dpt[2][32]+ROcp25_718*s->dpt[3][32];
    RLcp25_261 = ROcp25_218*s->dpt[1][32]+ROcp25_517*s->dpt[2][32]+ROcp25_818*s->dpt[3][32];
    RLcp25_361 = ROcp25_318*s->dpt[1][32]+ROcp25_617*s->dpt[2][32]+ROcp25_918*s->dpt[3][32];
    POcp25_161 = RLcp25_113+RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_161+q[1];
    POcp25_261 = RLcp25_213+RLcp25_214+RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_261+q[2];
    POcp25_361 = RLcp25_313+RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_361+q[3];
    JTcp25_261_4 = -(RLcp25_313+RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_361);
    JTcp25_361_4 = RLcp25_213+RLcp25_214+RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_261;
    JTcp25_161_5 = C4*(RLcp25_313+RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318)-S4*(RLcp25_213+RLcp25_214)-S4*(
 RLcp25_215+RLcp25_216)-S4*(RLcp25_217+RLcp25_218)-RLcp25_261*S4+RLcp25_361*C4;
    JTcp25_261_5 = S4*(RLcp25_113+RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_161);
    JTcp25_361_5 = -C4*(RLcp25_113+RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_161);
    JTcp25_161_6 = ROcp25_85*(RLcp25_313+RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318)-ROcp25_95*(RLcp25_213+
 RLcp25_214)-ROcp25_95*(RLcp25_215+RLcp25_216)-ROcp25_95*(RLcp25_217+RLcp25_218)-RLcp25_261*ROcp25_95+RLcp25_361*ROcp25_85;
    JTcp25_261_6 = -(RLcp25_361*S5-ROcp25_95*(RLcp25_113+RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_161
 )+S5*(RLcp25_313+RLcp25_314)+S5*(RLcp25_315+RLcp25_316)+S5*(RLcp25_317+RLcp25_318));
    JTcp25_361_6 = RLcp25_261*S5-ROcp25_85*(RLcp25_113+RLcp25_114+RLcp25_115+RLcp25_116+RLcp25_117+RLcp25_118+RLcp25_161)+
 S5*(RLcp25_213+RLcp25_214)+S5*(RLcp25_215+RLcp25_216)+S5*(RLcp25_217+RLcp25_218);
    JTcp25_161_7 = ROcp25_56*(RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_361)-ROcp25_66*(RLcp25_214+
 RLcp25_215)-ROcp25_66*(RLcp25_216+RLcp25_217)-ROcp25_66*(RLcp25_218+RLcp25_261);
    JTcp25_261_7 = -(ROcp25_46*(RLcp25_314+RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_361)-ROcp25_66*(RLcp25_114+
 RLcp25_115)-ROcp25_66*(RLcp25_116+RLcp25_117)-ROcp25_66*(RLcp25_118+RLcp25_161));
    JTcp25_361_7 = ROcp25_46*(RLcp25_214+RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_261)-ROcp25_56*(RLcp25_114+
 RLcp25_115)-ROcp25_56*(RLcp25_116+RLcp25_117)-ROcp25_56*(RLcp25_118+RLcp25_161);
    JTcp25_161_8 = ROcp25_213*(RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318)-ROcp25_313*(RLcp25_215+RLcp25_216)-ROcp25_313*
 (RLcp25_217+RLcp25_218)-RLcp25_261*ROcp25_313+RLcp25_361*ROcp25_213;
    JTcp25_261_8 = RLcp25_161*ROcp25_313-RLcp25_361*ROcp25_113-ROcp25_113*(RLcp25_315+RLcp25_316+RLcp25_317+RLcp25_318)+
 ROcp25_313*(RLcp25_115+RLcp25_116)+ROcp25_313*(RLcp25_117+RLcp25_118);
    JTcp25_361_8 = ROcp25_113*(RLcp25_215+RLcp25_216+RLcp25_217+RLcp25_218)-ROcp25_213*(RLcp25_115+RLcp25_116)-ROcp25_213*
 (RLcp25_117+RLcp25_118)-RLcp25_161*ROcp25_213+RLcp25_261*ROcp25_113;
    JTcp25_161_9 = ROcp25_814*(RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_361)-ROcp25_914*(RLcp25_216+RLcp25_217)-ROcp25_914*
 (RLcp25_218+RLcp25_261);
    JTcp25_261_9 = -(ROcp25_714*(RLcp25_316+RLcp25_317+RLcp25_318+RLcp25_361)-ROcp25_914*(RLcp25_116+RLcp25_117)-
 ROcp25_914*(RLcp25_118+RLcp25_161));
    JTcp25_361_9 = ROcp25_714*(RLcp25_216+RLcp25_217+RLcp25_218+RLcp25_261)-ROcp25_814*(RLcp25_116+RLcp25_117)-ROcp25_814*
 (RLcp25_118+RLcp25_161);
    JTcp25_161_10 = ROcp25_515*(RLcp25_317+RLcp25_318)-ROcp25_615*(RLcp25_217+RLcp25_218)-RLcp25_261*ROcp25_615+RLcp25_361
 *ROcp25_515;
    JTcp25_261_10 = RLcp25_161*ROcp25_615-RLcp25_361*ROcp25_415-ROcp25_415*(RLcp25_317+RLcp25_318)+ROcp25_615*(RLcp25_117+
 RLcp25_118);
    JTcp25_361_10 = ROcp25_415*(RLcp25_217+RLcp25_218)-ROcp25_515*(RLcp25_117+RLcp25_118)-RLcp25_161*ROcp25_515+RLcp25_261
 *ROcp25_415;
    JTcp25_161_11 = ROcp25_216*(RLcp25_318+RLcp25_361)-ROcp25_316*(RLcp25_218+RLcp25_261);
    JTcp25_261_11 = -(ROcp25_116*(RLcp25_318+RLcp25_361)-ROcp25_316*(RLcp25_118+RLcp25_161));
    JTcp25_361_11 = ROcp25_116*(RLcp25_218+RLcp25_261)-ROcp25_216*(RLcp25_118+RLcp25_161);
    JTcp25_161_12 = -(RLcp25_261*ROcp25_617-RLcp25_361*ROcp25_517);
    JTcp25_261_12 = RLcp25_161*ROcp25_617-RLcp25_361*ROcp25_417;
    JTcp25_361_12 = -(RLcp25_161*ROcp25_517-RLcp25_261*ROcp25_417);
    ORcp25_161 = OMcp25_218*RLcp25_361-OMcp25_318*RLcp25_261;
    ORcp25_261 = -(OMcp25_118*RLcp25_361-OMcp25_318*RLcp25_161);
    ORcp25_361 = OMcp25_118*RLcp25_261-OMcp25_218*RLcp25_161;
    VIcp25_161 = ORcp25_113+ORcp25_114+ORcp25_115+ORcp25_116+ORcp25_117+ORcp25_118+ORcp25_161+qd[1];
    VIcp25_261 = ORcp25_213+ORcp25_214+ORcp25_215+ORcp25_216+ORcp25_217+ORcp25_218+ORcp25_261+qd[2];
    VIcp25_361 = ORcp25_313+ORcp25_314+ORcp25_315+ORcp25_316+ORcp25_317+ORcp25_318+ORcp25_361+qd[3];
    ACcp25_161 = qdd[1]+OMcp25_213*ORcp25_314+OMcp25_214*ORcp25_315+OMcp25_215*ORcp25_316+OMcp25_216*ORcp25_317+OMcp25_217
 *ORcp25_318+OMcp25_218*ORcp25_361+OMcp25_26*ORcp25_313-OMcp25_313*ORcp25_214-OMcp25_314*ORcp25_215-OMcp25_315*ORcp25_216-
 OMcp25_316*ORcp25_217-OMcp25_317*ORcp25_218-OMcp25_318*ORcp25_261-OMcp25_36*ORcp25_213+OPcp25_213*RLcp25_314+OPcp25_214*
 RLcp25_315+OPcp25_215*RLcp25_316+OPcp25_216*RLcp25_317+OPcp25_217*RLcp25_318+OPcp25_218*RLcp25_361+OPcp25_26*RLcp25_313-
 OPcp25_313*RLcp25_214-OPcp25_314*RLcp25_215-OPcp25_315*RLcp25_216-OPcp25_316*RLcp25_217-OPcp25_317*RLcp25_218-OPcp25_318*
 RLcp25_261-OPcp25_36*RLcp25_213;
    ACcp25_261 = qdd[2]-OMcp25_113*ORcp25_314-OMcp25_114*ORcp25_315-OMcp25_115*ORcp25_316-OMcp25_116*ORcp25_317-OMcp25_117
 *ORcp25_318-OMcp25_118*ORcp25_361-OMcp25_16*ORcp25_313+OMcp25_313*ORcp25_114+OMcp25_314*ORcp25_115+OMcp25_315*ORcp25_116+
 OMcp25_316*ORcp25_117+OMcp25_317*ORcp25_118+OMcp25_318*ORcp25_161+OMcp25_36*ORcp25_113-OPcp25_113*RLcp25_314-OPcp25_114*
 RLcp25_315-OPcp25_115*RLcp25_316-OPcp25_116*RLcp25_317-OPcp25_117*RLcp25_318-OPcp25_118*RLcp25_361-OPcp25_16*RLcp25_313+
 OPcp25_313*RLcp25_114+OPcp25_314*RLcp25_115+OPcp25_315*RLcp25_116+OPcp25_316*RLcp25_117+OPcp25_317*RLcp25_118+OPcp25_318*
 RLcp25_161+OPcp25_36*RLcp25_113;
    ACcp25_361 = qdd[3]+OMcp25_113*ORcp25_214+OMcp25_114*ORcp25_215+OMcp25_115*ORcp25_216+OMcp25_116*ORcp25_217+OMcp25_117
 *ORcp25_218+OMcp25_118*ORcp25_261+OMcp25_16*ORcp25_213-OMcp25_213*ORcp25_114-OMcp25_214*ORcp25_115-OMcp25_215*ORcp25_116-
 OMcp25_216*ORcp25_117-OMcp25_217*ORcp25_118-OMcp25_218*ORcp25_161-OMcp25_26*ORcp25_113+OPcp25_113*RLcp25_214+OPcp25_114*
 RLcp25_215+OPcp25_115*RLcp25_216+OPcp25_116*RLcp25_217+OPcp25_117*RLcp25_218+OPcp25_118*RLcp25_261+OPcp25_16*RLcp25_213-
 OPcp25_213*RLcp25_114-OPcp25_214*RLcp25_115-OPcp25_215*RLcp25_116-OPcp25_216*RLcp25_117-OPcp25_217*RLcp25_118-OPcp25_218*
 RLcp25_161-OPcp25_26*RLcp25_113;

// = = Block_1_0_0_26_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp25_161;
    sens->P[2] = POcp25_261;
    sens->P[3] = POcp25_361;
    sens->R[1][1] = ROcp25_118;
    sens->R[1][2] = ROcp25_218;
    sens->R[1][3] = ROcp25_318;
    sens->R[2][1] = ROcp25_417;
    sens->R[2][2] = ROcp25_517;
    sens->R[2][3] = ROcp25_617;
    sens->R[3][1] = ROcp25_718;
    sens->R[3][2] = ROcp25_818;
    sens->R[3][3] = ROcp25_918;
    sens->V[1] = VIcp25_161;
    sens->V[2] = VIcp25_261;
    sens->V[3] = VIcp25_361;
    sens->OM[1] = OMcp25_118;
    sens->OM[2] = OMcp25_218;
    sens->OM[3] = OMcp25_318;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp25_161_5;
    sens->J[1][6] = JTcp25_161_6;
    sens->J[1][13] = JTcp25_161_7;
    sens->J[1][14] = JTcp25_161_8;
    sens->J[1][15] = JTcp25_161_9;
    sens->J[1][16] = JTcp25_161_10;
    sens->J[1][17] = JTcp25_161_11;
    sens->J[1][18] = JTcp25_161_12;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp25_261_4;
    sens->J[2][5] = JTcp25_261_5;
    sens->J[2][6] = JTcp25_261_6;
    sens->J[2][13] = JTcp25_261_7;
    sens->J[2][14] = JTcp25_261_8;
    sens->J[2][15] = JTcp25_261_9;
    sens->J[2][16] = JTcp25_261_10;
    sens->J[2][17] = JTcp25_261_11;
    sens->J[2][18] = JTcp25_261_12;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp25_361_4;
    sens->J[3][5] = JTcp25_361_5;
    sens->J[3][6] = JTcp25_361_6;
    sens->J[3][13] = JTcp25_361_7;
    sens->J[3][14] = JTcp25_361_8;
    sens->J[3][15] = JTcp25_361_9;
    sens->J[3][16] = JTcp25_361_10;
    sens->J[3][17] = JTcp25_361_11;
    sens->J[3][18] = JTcp25_361_12;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp25_46;
    sens->J[4][14] = ROcp25_113;
    sens->J[4][15] = ROcp25_714;
    sens->J[4][16] = ROcp25_415;
    sens->J[4][17] = ROcp25_116;
    sens->J[4][18] = ROcp25_417;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp25_85;
    sens->J[5][13] = ROcp25_56;
    sens->J[5][14] = ROcp25_213;
    sens->J[5][15] = ROcp25_814;
    sens->J[5][16] = ROcp25_515;
    sens->J[5][17] = ROcp25_216;
    sens->J[5][18] = ROcp25_517;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp25_95;
    sens->J[6][13] = ROcp25_66;
    sens->J[6][14] = ROcp25_313;
    sens->J[6][15] = ROcp25_914;
    sens->J[6][16] = ROcp25_615;
    sens->J[6][17] = ROcp25_316;
    sens->J[6][18] = ROcp25_617;
    sens->A[1] = ACcp25_161;
    sens->A[2] = ACcp25_261;
    sens->A[3] = ACcp25_361;
    sens->OMP[1] = OPcp25_118;
    sens->OMP[2] = OPcp25_218;
    sens->OMP[3] = OPcp25_318;
 
// 
break;
case 27:
 


// = = Block_1_0_0_27_0_1 = = 
 
// Sensor Kinematics 


    ROcp26_25 = S4*S5;
    ROcp26_35 = -C4*S5;
    ROcp26_85 = -S4*C5;
    ROcp26_95 = C4*C5;
    ROcp26_16 = C5*C6;
    ROcp26_26 = ROcp26_25*C6+C4*S6;
    ROcp26_36 = ROcp26_35*C6+S4*S6;
    ROcp26_46 = -C5*S6;
    ROcp26_56 = -(ROcp26_25*S6-C4*C6);
    ROcp26_66 = -(ROcp26_35*S6-S4*C6);
    OMcp26_25 = qd[5]*C4;
    OMcp26_35 = qd[5]*S4;
    OMcp26_16 = qd[4]+qd[6]*S5;
    OMcp26_26 = OMcp26_25+ROcp26_85*qd[6];
    OMcp26_36 = OMcp26_35+ROcp26_95*qd[6];
    OPcp26_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp26_26 = ROcp26_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp26_35*S5-ROcp26_95*qd[4]);
    OPcp26_36 = ROcp26_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp26_25*S5-ROcp26_85*qd[4]);

// = = Block_1_0_0_27_0_3 = = 
 
// Sensor Kinematics 


    ROcp26_113 = ROcp26_16*C13-S13*S5;
    ROcp26_213 = ROcp26_26*C13-ROcp26_85*S13;
    ROcp26_313 = ROcp26_36*C13-ROcp26_95*S13;
    ROcp26_713 = ROcp26_16*S13+C13*S5;
    ROcp26_813 = ROcp26_26*S13+ROcp26_85*C13;
    ROcp26_913 = ROcp26_36*S13+ROcp26_95*C13;
    ROcp26_414 = ROcp26_46*C14+ROcp26_713*S14;
    ROcp26_514 = ROcp26_56*C14+ROcp26_813*S14;
    ROcp26_614 = ROcp26_66*C14+ROcp26_913*S14;
    ROcp26_714 = -(ROcp26_46*S14-ROcp26_713*C14);
    ROcp26_814 = -(ROcp26_56*S14-ROcp26_813*C14);
    ROcp26_914 = -(ROcp26_66*S14-ROcp26_913*C14);
    ROcp26_115 = ROcp26_113*C15+ROcp26_414*S15;
    ROcp26_215 = ROcp26_213*C15+ROcp26_514*S15;
    ROcp26_315 = ROcp26_313*C15+ROcp26_614*S15;
    ROcp26_415 = -(ROcp26_113*S15-ROcp26_414*C15);
    ROcp26_515 = -(ROcp26_213*S15-ROcp26_514*C15);
    ROcp26_615 = -(ROcp26_313*S15-ROcp26_614*C15);
    ROcp26_116 = ROcp26_115*C16-ROcp26_714*S16;
    ROcp26_216 = ROcp26_215*C16-ROcp26_814*S16;
    ROcp26_316 = ROcp26_315*C16-ROcp26_914*S16;
    ROcp26_716 = ROcp26_115*S16+ROcp26_714*C16;
    ROcp26_816 = ROcp26_215*S16+ROcp26_814*C16;
    ROcp26_916 = ROcp26_315*S16+ROcp26_914*C16;
    ROcp26_417 = ROcp26_415*C17+ROcp26_716*S17;
    ROcp26_517 = ROcp26_515*C17+ROcp26_816*S17;
    ROcp26_617 = ROcp26_615*C17+ROcp26_916*S17;
    ROcp26_717 = -(ROcp26_415*S17-ROcp26_716*C17);
    ROcp26_817 = -(ROcp26_515*S17-ROcp26_816*C17);
    ROcp26_917 = -(ROcp26_615*S17-ROcp26_916*C17);
    ROcp26_118 = ROcp26_116*C18-ROcp26_717*S18;
    ROcp26_218 = ROcp26_216*C18-ROcp26_817*S18;
    ROcp26_318 = ROcp26_316*C18-ROcp26_917*S18;
    ROcp26_718 = ROcp26_116*S18+ROcp26_717*C18;
    ROcp26_818 = ROcp26_216*S18+ROcp26_817*C18;
    ROcp26_918 = ROcp26_316*S18+ROcp26_917*C18;
    RLcp26_113 = s->dpt[1][2]*ROcp26_16+s->dpt[3][2]*S5+ROcp26_46*s->dpt[2][2];
    RLcp26_213 = s->dpt[1][2]*ROcp26_26+s->dpt[3][2]*ROcp26_85+ROcp26_56*s->dpt[2][2];
    RLcp26_313 = s->dpt[1][2]*ROcp26_36+s->dpt[3][2]*ROcp26_95+ROcp26_66*s->dpt[2][2];
    OMcp26_113 = OMcp26_16+ROcp26_46*qd[13];
    OMcp26_213 = OMcp26_26+ROcp26_56*qd[13];
    OMcp26_313 = OMcp26_36+ROcp26_66*qd[13];
    ORcp26_113 = OMcp26_26*RLcp26_313-OMcp26_36*RLcp26_213;
    ORcp26_213 = -(OMcp26_16*RLcp26_313-OMcp26_36*RLcp26_113);
    ORcp26_313 = OMcp26_16*RLcp26_213-OMcp26_26*RLcp26_113;
    OPcp26_113 = OPcp26_16+ROcp26_46*qdd[13]+qd[13]*(OMcp26_26*ROcp26_66-OMcp26_36*ROcp26_56);
    OPcp26_213 = OPcp26_26+ROcp26_56*qdd[13]-qd[13]*(OMcp26_16*ROcp26_66-OMcp26_36*ROcp26_46);
    OPcp26_313 = OPcp26_36+ROcp26_66*qdd[13]+qd[13]*(OMcp26_16*ROcp26_56-OMcp26_26*ROcp26_46);
    RLcp26_114 = s->dpt[1][22]*ROcp26_113+s->dpt[3][22]*ROcp26_713+ROcp26_46*s->dpt[2][22];
    RLcp26_214 = s->dpt[1][22]*ROcp26_213+s->dpt[3][22]*ROcp26_813+ROcp26_56*s->dpt[2][22];
    RLcp26_314 = s->dpt[1][22]*ROcp26_313+s->dpt[3][22]*ROcp26_913+ROcp26_66*s->dpt[2][22];
    OMcp26_114 = OMcp26_113+ROcp26_113*qd[14];
    OMcp26_214 = OMcp26_213+ROcp26_213*qd[14];
    OMcp26_314 = OMcp26_313+ROcp26_313*qd[14];
    ORcp26_114 = OMcp26_213*RLcp26_314-OMcp26_313*RLcp26_214;
    ORcp26_214 = -(OMcp26_113*RLcp26_314-OMcp26_313*RLcp26_114);
    ORcp26_314 = OMcp26_113*RLcp26_214-OMcp26_213*RLcp26_114;
    OPcp26_114 = OPcp26_113+ROcp26_113*qdd[14]+qd[14]*(OMcp26_213*ROcp26_313-OMcp26_313*ROcp26_213);
    OPcp26_214 = OPcp26_213+ROcp26_213*qdd[14]-qd[14]*(OMcp26_113*ROcp26_313-OMcp26_313*ROcp26_113);
    OPcp26_314 = OPcp26_313+ROcp26_313*qdd[14]+qd[14]*(OMcp26_113*ROcp26_213-OMcp26_213*ROcp26_113);
    RLcp26_115 = s->dpt[1][24]*ROcp26_113+s->dpt[2][24]*ROcp26_414+ROcp26_714*s->dpt[3][24];
    RLcp26_215 = s->dpt[1][24]*ROcp26_213+s->dpt[2][24]*ROcp26_514+ROcp26_814*s->dpt[3][24];
    RLcp26_315 = s->dpt[1][24]*ROcp26_313+s->dpt[2][24]*ROcp26_614+ROcp26_914*s->dpt[3][24];
    OMcp26_115 = OMcp26_114+ROcp26_714*qd[15];
    OMcp26_215 = OMcp26_214+ROcp26_814*qd[15];
    OMcp26_315 = OMcp26_314+ROcp26_914*qd[15];
    ORcp26_115 = OMcp26_214*RLcp26_315-OMcp26_314*RLcp26_215;
    ORcp26_215 = -(OMcp26_114*RLcp26_315-OMcp26_314*RLcp26_115);
    ORcp26_315 = OMcp26_114*RLcp26_215-OMcp26_214*RLcp26_115;
    OPcp26_115 = OPcp26_114+ROcp26_714*qdd[15]+qd[15]*(OMcp26_214*ROcp26_914-OMcp26_314*ROcp26_814);
    OPcp26_215 = OPcp26_214+ROcp26_814*qdd[15]-qd[15]*(OMcp26_114*ROcp26_914-OMcp26_314*ROcp26_714);
    OPcp26_315 = OPcp26_314+ROcp26_914*qdd[15]+qd[15]*(OMcp26_114*ROcp26_814-OMcp26_214*ROcp26_714);
    RLcp26_116 = s->dpt[1][26]*ROcp26_115+s->dpt[2][26]*ROcp26_415+ROcp26_714*s->dpt[3][26];
    RLcp26_216 = s->dpt[1][26]*ROcp26_215+s->dpt[2][26]*ROcp26_515+ROcp26_814*s->dpt[3][26];
    RLcp26_316 = s->dpt[1][26]*ROcp26_315+s->dpt[2][26]*ROcp26_615+ROcp26_914*s->dpt[3][26];
    OMcp26_116 = OMcp26_115+ROcp26_415*qd[16];
    OMcp26_216 = OMcp26_215+ROcp26_515*qd[16];
    OMcp26_316 = OMcp26_315+ROcp26_615*qd[16];
    ORcp26_116 = OMcp26_215*RLcp26_316-OMcp26_315*RLcp26_216;
    ORcp26_216 = -(OMcp26_115*RLcp26_316-OMcp26_315*RLcp26_116);
    ORcp26_316 = OMcp26_115*RLcp26_216-OMcp26_215*RLcp26_116;
    OPcp26_116 = OPcp26_115+ROcp26_415*qdd[16]+qd[16]*(OMcp26_215*ROcp26_615-OMcp26_315*ROcp26_515);
    OPcp26_216 = OPcp26_215+ROcp26_515*qdd[16]-qd[16]*(OMcp26_115*ROcp26_615-OMcp26_315*ROcp26_415);
    OPcp26_316 = OPcp26_315+ROcp26_615*qdd[16]+qd[16]*(OMcp26_115*ROcp26_515-OMcp26_215*ROcp26_415);
    RLcp26_117 = s->dpt[1][28]*ROcp26_116+s->dpt[2][28]*ROcp26_415+ROcp26_716*s->dpt[3][28];
    RLcp26_217 = s->dpt[1][28]*ROcp26_216+s->dpt[2][28]*ROcp26_515+ROcp26_816*s->dpt[3][28];
    RLcp26_317 = s->dpt[1][28]*ROcp26_316+s->dpt[2][28]*ROcp26_615+ROcp26_916*s->dpt[3][28];
    OMcp26_117 = OMcp26_116+ROcp26_116*qd[17];
    OMcp26_217 = OMcp26_216+ROcp26_216*qd[17];
    OMcp26_317 = OMcp26_316+ROcp26_316*qd[17];
    ORcp26_117 = OMcp26_216*RLcp26_317-OMcp26_316*RLcp26_217;
    ORcp26_217 = -(OMcp26_116*RLcp26_317-OMcp26_316*RLcp26_117);
    ORcp26_317 = OMcp26_116*RLcp26_217-OMcp26_216*RLcp26_117;
    OPcp26_117 = OPcp26_116+ROcp26_116*qdd[17]+qd[17]*(OMcp26_216*ROcp26_316-OMcp26_316*ROcp26_216);
    OPcp26_217 = OPcp26_216+ROcp26_216*qdd[17]-qd[17]*(OMcp26_116*ROcp26_316-OMcp26_316*ROcp26_116);
    OPcp26_317 = OPcp26_316+ROcp26_316*qdd[17]+qd[17]*(OMcp26_116*ROcp26_216-OMcp26_216*ROcp26_116);
    RLcp26_118 = s->dpt[1][30]*ROcp26_116+s->dpt[2][30]*ROcp26_417+s->dpt[3][30]*ROcp26_717;
    RLcp26_218 = s->dpt[1][30]*ROcp26_216+s->dpt[2][30]*ROcp26_517+s->dpt[3][30]*ROcp26_817;
    RLcp26_318 = s->dpt[1][30]*ROcp26_316+s->dpt[2][30]*ROcp26_617+s->dpt[3][30]*ROcp26_917;
    OMcp26_118 = OMcp26_117+ROcp26_417*qd[18];
    OMcp26_218 = OMcp26_217+ROcp26_517*qd[18];
    OMcp26_318 = OMcp26_317+ROcp26_617*qd[18];
    ORcp26_118 = OMcp26_217*RLcp26_318-OMcp26_317*RLcp26_218;
    ORcp26_218 = -(OMcp26_117*RLcp26_318-OMcp26_317*RLcp26_118);
    ORcp26_318 = OMcp26_117*RLcp26_218-OMcp26_217*RLcp26_118;
    OPcp26_118 = OPcp26_117+ROcp26_417*qdd[18]+qd[18]*(OMcp26_217*ROcp26_617-OMcp26_317*ROcp26_517);
    OPcp26_218 = OPcp26_217+ROcp26_517*qdd[18]-qd[18]*(OMcp26_117*ROcp26_617-OMcp26_317*ROcp26_417);
    OPcp26_318 = OPcp26_317+ROcp26_617*qdd[18]+qd[18]*(OMcp26_117*ROcp26_517-OMcp26_217*ROcp26_417);
    RLcp26_162 = s->dpt[1][33]*ROcp26_118+s->dpt[2][33]*ROcp26_417+ROcp26_718*s->dpt[3][33];
    RLcp26_262 = s->dpt[1][33]*ROcp26_218+s->dpt[2][33]*ROcp26_517+ROcp26_818*s->dpt[3][33];
    RLcp26_362 = s->dpt[1][33]*ROcp26_318+s->dpt[2][33]*ROcp26_617+ROcp26_918*s->dpt[3][33];
    POcp26_162 = RLcp26_113+RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_162+q[1];
    POcp26_262 = RLcp26_213+RLcp26_214+RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_262+q[2];
    POcp26_362 = RLcp26_313+RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_362+q[3];
    JTcp26_262_4 = -(RLcp26_313+RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_362);
    JTcp26_362_4 = RLcp26_213+RLcp26_214+RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_262;
    JTcp26_162_5 = C4*(RLcp26_313+RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318)-S4*(RLcp26_213+RLcp26_214)-S4*(
 RLcp26_215+RLcp26_216)-S4*(RLcp26_217+RLcp26_218)-RLcp26_262*S4+RLcp26_362*C4;
    JTcp26_262_5 = S4*(RLcp26_113+RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_162);
    JTcp26_362_5 = -C4*(RLcp26_113+RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_162);
    JTcp26_162_6 = ROcp26_85*(RLcp26_313+RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318)-ROcp26_95*(RLcp26_213+
 RLcp26_214)-ROcp26_95*(RLcp26_215+RLcp26_216)-ROcp26_95*(RLcp26_217+RLcp26_218)-RLcp26_262*ROcp26_95+RLcp26_362*ROcp26_85;
    JTcp26_262_6 = -(RLcp26_362*S5-ROcp26_95*(RLcp26_113+RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_162
 )+S5*(RLcp26_313+RLcp26_314)+S5*(RLcp26_315+RLcp26_316)+S5*(RLcp26_317+RLcp26_318));
    JTcp26_362_6 = RLcp26_262*S5-ROcp26_85*(RLcp26_113+RLcp26_114+RLcp26_115+RLcp26_116+RLcp26_117+RLcp26_118+RLcp26_162)+
 S5*(RLcp26_213+RLcp26_214)+S5*(RLcp26_215+RLcp26_216)+S5*(RLcp26_217+RLcp26_218);
    JTcp26_162_7 = ROcp26_56*(RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_362)-ROcp26_66*(RLcp26_214+
 RLcp26_215)-ROcp26_66*(RLcp26_216+RLcp26_217)-ROcp26_66*(RLcp26_218+RLcp26_262);
    JTcp26_262_7 = -(ROcp26_46*(RLcp26_314+RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_362)-ROcp26_66*(RLcp26_114+
 RLcp26_115)-ROcp26_66*(RLcp26_116+RLcp26_117)-ROcp26_66*(RLcp26_118+RLcp26_162));
    JTcp26_362_7 = ROcp26_46*(RLcp26_214+RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_262)-ROcp26_56*(RLcp26_114+
 RLcp26_115)-ROcp26_56*(RLcp26_116+RLcp26_117)-ROcp26_56*(RLcp26_118+RLcp26_162);
    JTcp26_162_8 = ROcp26_213*(RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318)-ROcp26_313*(RLcp26_215+RLcp26_216)-ROcp26_313*
 (RLcp26_217+RLcp26_218)-RLcp26_262*ROcp26_313+RLcp26_362*ROcp26_213;
    JTcp26_262_8 = RLcp26_162*ROcp26_313-RLcp26_362*ROcp26_113-ROcp26_113*(RLcp26_315+RLcp26_316+RLcp26_317+RLcp26_318)+
 ROcp26_313*(RLcp26_115+RLcp26_116)+ROcp26_313*(RLcp26_117+RLcp26_118);
    JTcp26_362_8 = ROcp26_113*(RLcp26_215+RLcp26_216+RLcp26_217+RLcp26_218)-ROcp26_213*(RLcp26_115+RLcp26_116)-ROcp26_213*
 (RLcp26_117+RLcp26_118)-RLcp26_162*ROcp26_213+RLcp26_262*ROcp26_113;
    JTcp26_162_9 = ROcp26_814*(RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_362)-ROcp26_914*(RLcp26_216+RLcp26_217)-ROcp26_914*
 (RLcp26_218+RLcp26_262);
    JTcp26_262_9 = -(ROcp26_714*(RLcp26_316+RLcp26_317+RLcp26_318+RLcp26_362)-ROcp26_914*(RLcp26_116+RLcp26_117)-
 ROcp26_914*(RLcp26_118+RLcp26_162));
    JTcp26_362_9 = ROcp26_714*(RLcp26_216+RLcp26_217+RLcp26_218+RLcp26_262)-ROcp26_814*(RLcp26_116+RLcp26_117)-ROcp26_814*
 (RLcp26_118+RLcp26_162);
    JTcp26_162_10 = ROcp26_515*(RLcp26_317+RLcp26_318)-ROcp26_615*(RLcp26_217+RLcp26_218)-RLcp26_262*ROcp26_615+RLcp26_362
 *ROcp26_515;
    JTcp26_262_10 = RLcp26_162*ROcp26_615-RLcp26_362*ROcp26_415-ROcp26_415*(RLcp26_317+RLcp26_318)+ROcp26_615*(RLcp26_117+
 RLcp26_118);
    JTcp26_362_10 = ROcp26_415*(RLcp26_217+RLcp26_218)-ROcp26_515*(RLcp26_117+RLcp26_118)-RLcp26_162*ROcp26_515+RLcp26_262
 *ROcp26_415;
    JTcp26_162_11 = ROcp26_216*(RLcp26_318+RLcp26_362)-ROcp26_316*(RLcp26_218+RLcp26_262);
    JTcp26_262_11 = -(ROcp26_116*(RLcp26_318+RLcp26_362)-ROcp26_316*(RLcp26_118+RLcp26_162));
    JTcp26_362_11 = ROcp26_116*(RLcp26_218+RLcp26_262)-ROcp26_216*(RLcp26_118+RLcp26_162);
    JTcp26_162_12 = -(RLcp26_262*ROcp26_617-RLcp26_362*ROcp26_517);
    JTcp26_262_12 = RLcp26_162*ROcp26_617-RLcp26_362*ROcp26_417;
    JTcp26_362_12 = -(RLcp26_162*ROcp26_517-RLcp26_262*ROcp26_417);
    ORcp26_162 = OMcp26_218*RLcp26_362-OMcp26_318*RLcp26_262;
    ORcp26_262 = -(OMcp26_118*RLcp26_362-OMcp26_318*RLcp26_162);
    ORcp26_362 = OMcp26_118*RLcp26_262-OMcp26_218*RLcp26_162;
    VIcp26_162 = ORcp26_113+ORcp26_114+ORcp26_115+ORcp26_116+ORcp26_117+ORcp26_118+ORcp26_162+qd[1];
    VIcp26_262 = ORcp26_213+ORcp26_214+ORcp26_215+ORcp26_216+ORcp26_217+ORcp26_218+ORcp26_262+qd[2];
    VIcp26_362 = ORcp26_313+ORcp26_314+ORcp26_315+ORcp26_316+ORcp26_317+ORcp26_318+ORcp26_362+qd[3];
    ACcp26_162 = qdd[1]+OMcp26_213*ORcp26_314+OMcp26_214*ORcp26_315+OMcp26_215*ORcp26_316+OMcp26_216*ORcp26_317+OMcp26_217
 *ORcp26_318+OMcp26_218*ORcp26_362+OMcp26_26*ORcp26_313-OMcp26_313*ORcp26_214-OMcp26_314*ORcp26_215-OMcp26_315*ORcp26_216-
 OMcp26_316*ORcp26_217-OMcp26_317*ORcp26_218-OMcp26_318*ORcp26_262-OMcp26_36*ORcp26_213+OPcp26_213*RLcp26_314+OPcp26_214*
 RLcp26_315+OPcp26_215*RLcp26_316+OPcp26_216*RLcp26_317+OPcp26_217*RLcp26_318+OPcp26_218*RLcp26_362+OPcp26_26*RLcp26_313-
 OPcp26_313*RLcp26_214-OPcp26_314*RLcp26_215-OPcp26_315*RLcp26_216-OPcp26_316*RLcp26_217-OPcp26_317*RLcp26_218-OPcp26_318*
 RLcp26_262-OPcp26_36*RLcp26_213;
    ACcp26_262 = qdd[2]-OMcp26_113*ORcp26_314-OMcp26_114*ORcp26_315-OMcp26_115*ORcp26_316-OMcp26_116*ORcp26_317-OMcp26_117
 *ORcp26_318-OMcp26_118*ORcp26_362-OMcp26_16*ORcp26_313+OMcp26_313*ORcp26_114+OMcp26_314*ORcp26_115+OMcp26_315*ORcp26_116+
 OMcp26_316*ORcp26_117+OMcp26_317*ORcp26_118+OMcp26_318*ORcp26_162+OMcp26_36*ORcp26_113-OPcp26_113*RLcp26_314-OPcp26_114*
 RLcp26_315-OPcp26_115*RLcp26_316-OPcp26_116*RLcp26_317-OPcp26_117*RLcp26_318-OPcp26_118*RLcp26_362-OPcp26_16*RLcp26_313+
 OPcp26_313*RLcp26_114+OPcp26_314*RLcp26_115+OPcp26_315*RLcp26_116+OPcp26_316*RLcp26_117+OPcp26_317*RLcp26_118+OPcp26_318*
 RLcp26_162+OPcp26_36*RLcp26_113;
    ACcp26_362 = qdd[3]+OMcp26_113*ORcp26_214+OMcp26_114*ORcp26_215+OMcp26_115*ORcp26_216+OMcp26_116*ORcp26_217+OMcp26_117
 *ORcp26_218+OMcp26_118*ORcp26_262+OMcp26_16*ORcp26_213-OMcp26_213*ORcp26_114-OMcp26_214*ORcp26_115-OMcp26_215*ORcp26_116-
 OMcp26_216*ORcp26_117-OMcp26_217*ORcp26_118-OMcp26_218*ORcp26_162-OMcp26_26*ORcp26_113+OPcp26_113*RLcp26_214+OPcp26_114*
 RLcp26_215+OPcp26_115*RLcp26_216+OPcp26_116*RLcp26_217+OPcp26_117*RLcp26_218+OPcp26_118*RLcp26_262+OPcp26_16*RLcp26_213-
 OPcp26_213*RLcp26_114-OPcp26_214*RLcp26_115-OPcp26_215*RLcp26_116-OPcp26_216*RLcp26_117-OPcp26_217*RLcp26_118-OPcp26_218*
 RLcp26_162-OPcp26_26*RLcp26_113;

// = = Block_1_0_0_27_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp26_162;
    sens->P[2] = POcp26_262;
    sens->P[3] = POcp26_362;
    sens->R[1][1] = ROcp26_118;
    sens->R[1][2] = ROcp26_218;
    sens->R[1][3] = ROcp26_318;
    sens->R[2][1] = ROcp26_417;
    sens->R[2][2] = ROcp26_517;
    sens->R[2][3] = ROcp26_617;
    sens->R[3][1] = ROcp26_718;
    sens->R[3][2] = ROcp26_818;
    sens->R[3][3] = ROcp26_918;
    sens->V[1] = VIcp26_162;
    sens->V[2] = VIcp26_262;
    sens->V[3] = VIcp26_362;
    sens->OM[1] = OMcp26_118;
    sens->OM[2] = OMcp26_218;
    sens->OM[3] = OMcp26_318;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp26_162_5;
    sens->J[1][6] = JTcp26_162_6;
    sens->J[1][13] = JTcp26_162_7;
    sens->J[1][14] = JTcp26_162_8;
    sens->J[1][15] = JTcp26_162_9;
    sens->J[1][16] = JTcp26_162_10;
    sens->J[1][17] = JTcp26_162_11;
    sens->J[1][18] = JTcp26_162_12;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp26_262_4;
    sens->J[2][5] = JTcp26_262_5;
    sens->J[2][6] = JTcp26_262_6;
    sens->J[2][13] = JTcp26_262_7;
    sens->J[2][14] = JTcp26_262_8;
    sens->J[2][15] = JTcp26_262_9;
    sens->J[2][16] = JTcp26_262_10;
    sens->J[2][17] = JTcp26_262_11;
    sens->J[2][18] = JTcp26_262_12;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp26_362_4;
    sens->J[3][5] = JTcp26_362_5;
    sens->J[3][6] = JTcp26_362_6;
    sens->J[3][13] = JTcp26_362_7;
    sens->J[3][14] = JTcp26_362_8;
    sens->J[3][15] = JTcp26_362_9;
    sens->J[3][16] = JTcp26_362_10;
    sens->J[3][17] = JTcp26_362_11;
    sens->J[3][18] = JTcp26_362_12;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp26_46;
    sens->J[4][14] = ROcp26_113;
    sens->J[4][15] = ROcp26_714;
    sens->J[4][16] = ROcp26_415;
    sens->J[4][17] = ROcp26_116;
    sens->J[4][18] = ROcp26_417;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp26_85;
    sens->J[5][13] = ROcp26_56;
    sens->J[5][14] = ROcp26_213;
    sens->J[5][15] = ROcp26_814;
    sens->J[5][16] = ROcp26_515;
    sens->J[5][17] = ROcp26_216;
    sens->J[5][18] = ROcp26_517;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp26_95;
    sens->J[6][13] = ROcp26_66;
    sens->J[6][14] = ROcp26_313;
    sens->J[6][15] = ROcp26_914;
    sens->J[6][16] = ROcp26_615;
    sens->J[6][17] = ROcp26_316;
    sens->J[6][18] = ROcp26_617;
    sens->A[1] = ACcp26_162;
    sens->A[2] = ACcp26_262;
    sens->A[3] = ACcp26_362;
    sens->OMP[1] = OPcp26_118;
    sens->OMP[2] = OPcp26_218;
    sens->OMP[3] = OPcp26_318;
 
// 
break;
case 28:
 


// = = Block_1_0_0_28_0_1 = = 
 
// Sensor Kinematics 


    ROcp27_25 = S4*S5;
    ROcp27_35 = -C4*S5;
    ROcp27_85 = -S4*C5;
    ROcp27_95 = C4*C5;
    ROcp27_16 = C5*C6;
    ROcp27_26 = ROcp27_25*C6+C4*S6;
    ROcp27_36 = ROcp27_35*C6+S4*S6;
    ROcp27_46 = -C5*S6;
    ROcp27_56 = -(ROcp27_25*S6-C4*C6);
    ROcp27_66 = -(ROcp27_35*S6-S4*C6);
    OMcp27_25 = qd[5]*C4;
    OMcp27_35 = qd[5]*S4;
    OMcp27_16 = qd[4]+qd[6]*S5;
    OMcp27_26 = OMcp27_25+ROcp27_85*qd[6];
    OMcp27_36 = OMcp27_35+ROcp27_95*qd[6];
    OPcp27_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp27_26 = ROcp27_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp27_35*S5-ROcp27_95*qd[4]);
    OPcp27_36 = ROcp27_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp27_25*S5-ROcp27_85*qd[4]);

// = = Block_1_0_0_28_0_4 = = 
 
// Sensor Kinematics 


    ROcp27_419 = ROcp27_46*C19+S19*S5;
    ROcp27_519 = ROcp27_56*C19+ROcp27_85*S19;
    ROcp27_619 = ROcp27_66*C19+ROcp27_95*S19;
    ROcp27_719 = -(ROcp27_46*S19-C19*S5);
    ROcp27_819 = -(ROcp27_56*S19-ROcp27_85*C19);
    ROcp27_919 = -(ROcp27_66*S19-ROcp27_95*C19);
    RLcp27_119 = s->dpt[2][3]*ROcp27_46+ROcp27_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp27_219 = s->dpt[2][3]*ROcp27_56+ROcp27_26*s->dpt[1][3]+ROcp27_85*s->dpt[3][3];
    RLcp27_319 = s->dpt[2][3]*ROcp27_66+ROcp27_36*s->dpt[1][3]+ROcp27_95*s->dpt[3][3];
    OMcp27_119 = OMcp27_16+ROcp27_16*qd[19];
    OMcp27_219 = OMcp27_26+ROcp27_26*qd[19];
    OMcp27_319 = OMcp27_36+ROcp27_36*qd[19];
    ORcp27_119 = OMcp27_26*RLcp27_319-OMcp27_36*RLcp27_219;
    ORcp27_219 = -(OMcp27_16*RLcp27_319-OMcp27_36*RLcp27_119);
    ORcp27_319 = OMcp27_16*RLcp27_219-OMcp27_26*RLcp27_119;
    OPcp27_119 = OPcp27_16+ROcp27_16*qdd[19]+qd[19]*(OMcp27_26*ROcp27_36-OMcp27_36*ROcp27_26);
    OPcp27_219 = OPcp27_26+ROcp27_26*qdd[19]-qd[19]*(OMcp27_16*ROcp27_36-OMcp27_36*ROcp27_16);
    OPcp27_319 = OPcp27_36+ROcp27_36*qdd[19]+qd[19]*(OMcp27_16*ROcp27_26-OMcp27_26*ROcp27_16);
    RLcp27_163 = s->dpt[1][38]*ROcp27_16+s->dpt[2][38]*ROcp27_419+s->dpt[3][38]*ROcp27_719;
    RLcp27_263 = s->dpt[1][38]*ROcp27_26+s->dpt[2][38]*ROcp27_519+s->dpt[3][38]*ROcp27_819;
    RLcp27_363 = s->dpt[1][38]*ROcp27_36+s->dpt[2][38]*ROcp27_619+s->dpt[3][38]*ROcp27_919;
    POcp27_163 = RLcp27_119+RLcp27_163+q[1];
    POcp27_263 = RLcp27_219+RLcp27_263+q[2];
    POcp27_363 = RLcp27_319+RLcp27_363+q[3];
    JTcp27_263_4 = -(RLcp27_319+RLcp27_363);
    JTcp27_363_4 = RLcp27_219+RLcp27_263;
    JTcp27_163_5 = C4*(RLcp27_319+RLcp27_363)-S4*(RLcp27_219+RLcp27_263);
    JTcp27_263_5 = S4*(RLcp27_119+RLcp27_163);
    JTcp27_363_5 = -C4*(RLcp27_119+RLcp27_163);
    JTcp27_163_6 = ROcp27_85*(RLcp27_319+RLcp27_363)-ROcp27_95*(RLcp27_219+RLcp27_263);
    JTcp27_263_6 = ROcp27_95*(RLcp27_119+RLcp27_163)-S5*(RLcp27_319+RLcp27_363);
    JTcp27_363_6 = -(ROcp27_85*(RLcp27_119+RLcp27_163)-S5*(RLcp27_219+RLcp27_263));
    JTcp27_163_7 = -(RLcp27_263*ROcp27_36-RLcp27_363*ROcp27_26);
    JTcp27_263_7 = RLcp27_163*ROcp27_36-RLcp27_363*ROcp27_16;
    JTcp27_363_7 = -(RLcp27_163*ROcp27_26-RLcp27_263*ROcp27_16);
    ORcp27_163 = OMcp27_219*RLcp27_363-OMcp27_319*RLcp27_263;
    ORcp27_263 = -(OMcp27_119*RLcp27_363-OMcp27_319*RLcp27_163);
    ORcp27_363 = OMcp27_119*RLcp27_263-OMcp27_219*RLcp27_163;
    VIcp27_163 = ORcp27_119+ORcp27_163+qd[1];
    VIcp27_263 = ORcp27_219+ORcp27_263+qd[2];
    VIcp27_363 = ORcp27_319+ORcp27_363+qd[3];
    ACcp27_163 = qdd[1]+OMcp27_219*ORcp27_363+OMcp27_26*ORcp27_319-OMcp27_319*ORcp27_263-OMcp27_36*ORcp27_219+OPcp27_219*
 RLcp27_363+OPcp27_26*RLcp27_319-OPcp27_319*RLcp27_263-OPcp27_36*RLcp27_219;
    ACcp27_263 = qdd[2]-OMcp27_119*ORcp27_363-OMcp27_16*ORcp27_319+OMcp27_319*ORcp27_163+OMcp27_36*ORcp27_119-OPcp27_119*
 RLcp27_363-OPcp27_16*RLcp27_319+OPcp27_319*RLcp27_163+OPcp27_36*RLcp27_119;
    ACcp27_363 = qdd[3]+OMcp27_119*ORcp27_263+OMcp27_16*ORcp27_219-OMcp27_219*ORcp27_163-OMcp27_26*ORcp27_119+OPcp27_119*
 RLcp27_263+OPcp27_16*RLcp27_219-OPcp27_219*RLcp27_163-OPcp27_26*RLcp27_119;

// = = Block_1_0_0_28_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp27_163;
    sens->P[2] = POcp27_263;
    sens->P[3] = POcp27_363;
    sens->R[1][1] = ROcp27_16;
    sens->R[1][2] = ROcp27_26;
    sens->R[1][3] = ROcp27_36;
    sens->R[2][1] = ROcp27_419;
    sens->R[2][2] = ROcp27_519;
    sens->R[2][3] = ROcp27_619;
    sens->R[3][1] = ROcp27_719;
    sens->R[3][2] = ROcp27_819;
    sens->R[3][3] = ROcp27_919;
    sens->V[1] = VIcp27_163;
    sens->V[2] = VIcp27_263;
    sens->V[3] = VIcp27_363;
    sens->OM[1] = OMcp27_119;
    sens->OM[2] = OMcp27_219;
    sens->OM[3] = OMcp27_319;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp27_163_5;
    sens->J[1][6] = JTcp27_163_6;
    sens->J[1][19] = JTcp27_163_7;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp27_263_4;
    sens->J[2][5] = JTcp27_263_5;
    sens->J[2][6] = JTcp27_263_6;
    sens->J[2][19] = JTcp27_263_7;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp27_363_4;
    sens->J[3][5] = JTcp27_363_5;
    sens->J[3][6] = JTcp27_363_6;
    sens->J[3][19] = JTcp27_363_7;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp27_16;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp27_85;
    sens->J[5][19] = ROcp27_26;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp27_95;
    sens->J[6][19] = ROcp27_36;
    sens->A[1] = ACcp27_163;
    sens->A[2] = ACcp27_263;
    sens->A[3] = ACcp27_363;
    sens->OMP[1] = OPcp27_119;
    sens->OMP[2] = OPcp27_219;
    sens->OMP[3] = OPcp27_319;
 
// 
break;
case 29:
 


// = = Block_1_0_0_29_0_1 = = 
 
// Sensor Kinematics 


    ROcp28_25 = S4*S5;
    ROcp28_35 = -C4*S5;
    ROcp28_85 = -S4*C5;
    ROcp28_95 = C4*C5;
    ROcp28_16 = C5*C6;
    ROcp28_26 = ROcp28_25*C6+C4*S6;
    ROcp28_36 = ROcp28_35*C6+S4*S6;
    ROcp28_46 = -C5*S6;
    ROcp28_56 = -(ROcp28_25*S6-C4*C6);
    ROcp28_66 = -(ROcp28_35*S6-S4*C6);
    OMcp28_25 = qd[5]*C4;
    OMcp28_35 = qd[5]*S4;
    OMcp28_16 = qd[4]+qd[6]*S5;
    OMcp28_26 = OMcp28_25+ROcp28_85*qd[6];
    OMcp28_36 = OMcp28_35+ROcp28_95*qd[6];
    OPcp28_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp28_26 = ROcp28_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp28_35*S5-ROcp28_95*qd[4]);
    OPcp28_36 = ROcp28_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp28_25*S5-ROcp28_85*qd[4]);

// = = Block_1_0_0_29_0_4 = = 
 
// Sensor Kinematics 


    ROcp28_419 = ROcp28_46*C19+S19*S5;
    ROcp28_519 = ROcp28_56*C19+ROcp28_85*S19;
    ROcp28_619 = ROcp28_66*C19+ROcp28_95*S19;
    ROcp28_719 = -(ROcp28_46*S19-C19*S5);
    ROcp28_819 = -(ROcp28_56*S19-ROcp28_85*C19);
    ROcp28_919 = -(ROcp28_66*S19-ROcp28_95*C19);
    RLcp28_119 = s->dpt[2][3]*ROcp28_46+ROcp28_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp28_219 = s->dpt[2][3]*ROcp28_56+ROcp28_26*s->dpt[1][3]+ROcp28_85*s->dpt[3][3];
    RLcp28_319 = s->dpt[2][3]*ROcp28_66+ROcp28_36*s->dpt[1][3]+ROcp28_95*s->dpt[3][3];
    OMcp28_119 = OMcp28_16+ROcp28_16*qd[19];
    OMcp28_219 = OMcp28_26+ROcp28_26*qd[19];
    OMcp28_319 = OMcp28_36+ROcp28_36*qd[19];
    ORcp28_119 = OMcp28_26*RLcp28_319-OMcp28_36*RLcp28_219;
    ORcp28_219 = -(OMcp28_16*RLcp28_319-OMcp28_36*RLcp28_119);
    ORcp28_319 = OMcp28_16*RLcp28_219-OMcp28_26*RLcp28_119;
    OPcp28_119 = OPcp28_16+ROcp28_16*qdd[19]+qd[19]*(OMcp28_26*ROcp28_36-OMcp28_36*ROcp28_26);
    OPcp28_219 = OPcp28_26+ROcp28_26*qdd[19]-qd[19]*(OMcp28_16*ROcp28_36-OMcp28_36*ROcp28_16);
    OPcp28_319 = OPcp28_36+ROcp28_36*qdd[19]+qd[19]*(OMcp28_16*ROcp28_26-OMcp28_26*ROcp28_16);
    RLcp28_164 = ROcp28_16*s->dpt[1][39]+ROcp28_419*s->dpt[2][39]+ROcp28_719*s->dpt[3][39];
    RLcp28_264 = ROcp28_26*s->dpt[1][39]+ROcp28_519*s->dpt[2][39]+ROcp28_819*s->dpt[3][39];
    RLcp28_364 = ROcp28_36*s->dpt[1][39]+ROcp28_619*s->dpt[2][39]+ROcp28_919*s->dpt[3][39];
    POcp28_164 = RLcp28_119+RLcp28_164+q[1];
    POcp28_264 = RLcp28_219+RLcp28_264+q[2];
    POcp28_364 = RLcp28_319+RLcp28_364+q[3];
    JTcp28_264_4 = -(RLcp28_319+RLcp28_364);
    JTcp28_364_4 = RLcp28_219+RLcp28_264;
    JTcp28_164_5 = C4*(RLcp28_319+RLcp28_364)-S4*(RLcp28_219+RLcp28_264);
    JTcp28_264_5 = S4*(RLcp28_119+RLcp28_164);
    JTcp28_364_5 = -C4*(RLcp28_119+RLcp28_164);
    JTcp28_164_6 = ROcp28_85*(RLcp28_319+RLcp28_364)-ROcp28_95*(RLcp28_219+RLcp28_264);
    JTcp28_264_6 = ROcp28_95*(RLcp28_119+RLcp28_164)-S5*(RLcp28_319+RLcp28_364);
    JTcp28_364_6 = -(ROcp28_85*(RLcp28_119+RLcp28_164)-S5*(RLcp28_219+RLcp28_264));
    JTcp28_164_7 = -(RLcp28_264*ROcp28_36-RLcp28_364*ROcp28_26);
    JTcp28_264_7 = RLcp28_164*ROcp28_36-RLcp28_364*ROcp28_16;
    JTcp28_364_7 = -(RLcp28_164*ROcp28_26-RLcp28_264*ROcp28_16);
    ORcp28_164 = OMcp28_219*RLcp28_364-OMcp28_319*RLcp28_264;
    ORcp28_264 = -(OMcp28_119*RLcp28_364-OMcp28_319*RLcp28_164);
    ORcp28_364 = OMcp28_119*RLcp28_264-OMcp28_219*RLcp28_164;
    VIcp28_164 = ORcp28_119+ORcp28_164+qd[1];
    VIcp28_264 = ORcp28_219+ORcp28_264+qd[2];
    VIcp28_364 = ORcp28_319+ORcp28_364+qd[3];
    ACcp28_164 = qdd[1]+OMcp28_219*ORcp28_364+OMcp28_26*ORcp28_319-OMcp28_319*ORcp28_264-OMcp28_36*ORcp28_219+OPcp28_219*
 RLcp28_364+OPcp28_26*RLcp28_319-OPcp28_319*RLcp28_264-OPcp28_36*RLcp28_219;
    ACcp28_264 = qdd[2]-OMcp28_119*ORcp28_364-OMcp28_16*ORcp28_319+OMcp28_319*ORcp28_164+OMcp28_36*ORcp28_119-OPcp28_119*
 RLcp28_364-OPcp28_16*RLcp28_319+OPcp28_319*RLcp28_164+OPcp28_36*RLcp28_119;
    ACcp28_364 = qdd[3]+OMcp28_119*ORcp28_264+OMcp28_16*ORcp28_219-OMcp28_219*ORcp28_164-OMcp28_26*ORcp28_119+OPcp28_119*
 RLcp28_264+OPcp28_16*RLcp28_219-OPcp28_219*RLcp28_164-OPcp28_26*RLcp28_119;

// = = Block_1_0_0_29_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp28_164;
    sens->P[2] = POcp28_264;
    sens->P[3] = POcp28_364;
    sens->R[1][1] = ROcp28_16;
    sens->R[1][2] = ROcp28_26;
    sens->R[1][3] = ROcp28_36;
    sens->R[2][1] = ROcp28_419;
    sens->R[2][2] = ROcp28_519;
    sens->R[2][3] = ROcp28_619;
    sens->R[3][1] = ROcp28_719;
    sens->R[3][2] = ROcp28_819;
    sens->R[3][3] = ROcp28_919;
    sens->V[1] = VIcp28_164;
    sens->V[2] = VIcp28_264;
    sens->V[3] = VIcp28_364;
    sens->OM[1] = OMcp28_119;
    sens->OM[2] = OMcp28_219;
    sens->OM[3] = OMcp28_319;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp28_164_5;
    sens->J[1][6] = JTcp28_164_6;
    sens->J[1][19] = JTcp28_164_7;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp28_264_4;
    sens->J[2][5] = JTcp28_264_5;
    sens->J[2][6] = JTcp28_264_6;
    sens->J[2][19] = JTcp28_264_7;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp28_364_4;
    sens->J[3][5] = JTcp28_364_5;
    sens->J[3][6] = JTcp28_364_6;
    sens->J[3][19] = JTcp28_364_7;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp28_16;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp28_85;
    sens->J[5][19] = ROcp28_26;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp28_95;
    sens->J[6][19] = ROcp28_36;
    sens->A[1] = ACcp28_164;
    sens->A[2] = ACcp28_264;
    sens->A[3] = ACcp28_364;
    sens->OMP[1] = OPcp28_119;
    sens->OMP[2] = OPcp28_219;
    sens->OMP[3] = OPcp28_319;
 
// 
break;
case 30:
 


// = = Block_1_0_0_30_0_1 = = 
 
// Sensor Kinematics 


    ROcp29_25 = S4*S5;
    ROcp29_35 = -C4*S5;
    ROcp29_85 = -S4*C5;
    ROcp29_95 = C4*C5;
    ROcp29_16 = C5*C6;
    ROcp29_26 = ROcp29_25*C6+C4*S6;
    ROcp29_36 = ROcp29_35*C6+S4*S6;
    ROcp29_46 = -C5*S6;
    ROcp29_56 = -(ROcp29_25*S6-C4*C6);
    ROcp29_66 = -(ROcp29_35*S6-S4*C6);
    OMcp29_25 = qd[5]*C4;
    OMcp29_35 = qd[5]*S4;
    OMcp29_16 = qd[4]+qd[6]*S5;
    OMcp29_26 = OMcp29_25+ROcp29_85*qd[6];
    OMcp29_36 = OMcp29_35+ROcp29_95*qd[6];
    OPcp29_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp29_26 = ROcp29_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp29_35*S5-ROcp29_95*qd[4]);
    OPcp29_36 = ROcp29_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp29_25*S5-ROcp29_85*qd[4]);

// = = Block_1_0_0_30_0_4 = = 
 
// Sensor Kinematics 


    ROcp29_419 = ROcp29_46*C19+S19*S5;
    ROcp29_519 = ROcp29_56*C19+ROcp29_85*S19;
    ROcp29_619 = ROcp29_66*C19+ROcp29_95*S19;
    ROcp29_719 = -(ROcp29_46*S19-C19*S5);
    ROcp29_819 = -(ROcp29_56*S19-ROcp29_85*C19);
    ROcp29_919 = -(ROcp29_66*S19-ROcp29_95*C19);
    ROcp29_120 = ROcp29_16*C20-ROcp29_719*S20;
    ROcp29_220 = ROcp29_26*C20-ROcp29_819*S20;
    ROcp29_320 = ROcp29_36*C20-ROcp29_919*S20;
    ROcp29_720 = ROcp29_16*S20+ROcp29_719*C20;
    ROcp29_820 = ROcp29_26*S20+ROcp29_819*C20;
    ROcp29_920 = ROcp29_36*S20+ROcp29_919*C20;
    RLcp29_119 = s->dpt[2][3]*ROcp29_46+ROcp29_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp29_219 = s->dpt[2][3]*ROcp29_56+ROcp29_26*s->dpt[1][3]+ROcp29_85*s->dpt[3][3];
    RLcp29_319 = s->dpt[2][3]*ROcp29_66+ROcp29_36*s->dpt[1][3]+ROcp29_95*s->dpt[3][3];
    OMcp29_119 = OMcp29_16+ROcp29_16*qd[19];
    OMcp29_219 = OMcp29_26+ROcp29_26*qd[19];
    OMcp29_319 = OMcp29_36+ROcp29_36*qd[19];
    ORcp29_119 = OMcp29_26*RLcp29_319-OMcp29_36*RLcp29_219;
    ORcp29_219 = -(OMcp29_16*RLcp29_319-OMcp29_36*RLcp29_119);
    ORcp29_319 = OMcp29_16*RLcp29_219-OMcp29_26*RLcp29_119;
    OPcp29_119 = OPcp29_16+ROcp29_16*qdd[19]+qd[19]*(OMcp29_26*ROcp29_36-OMcp29_36*ROcp29_26);
    OPcp29_219 = OPcp29_26+ROcp29_26*qdd[19]-qd[19]*(OMcp29_16*ROcp29_36-OMcp29_36*ROcp29_16);
    OPcp29_319 = OPcp29_36+ROcp29_36*qdd[19]+qd[19]*(OMcp29_16*ROcp29_26-OMcp29_26*ROcp29_16);
    RLcp29_120 = s->dpt[1][38]*ROcp29_16+s->dpt[2][38]*ROcp29_419+s->dpt[3][38]*ROcp29_719;
    RLcp29_220 = s->dpt[1][38]*ROcp29_26+s->dpt[2][38]*ROcp29_519+s->dpt[3][38]*ROcp29_819;
    RLcp29_320 = s->dpt[1][38]*ROcp29_36+s->dpt[2][38]*ROcp29_619+s->dpt[3][38]*ROcp29_919;
    OMcp29_120 = OMcp29_119+ROcp29_419*qd[20];
    OMcp29_220 = OMcp29_219+ROcp29_519*qd[20];
    OMcp29_320 = OMcp29_319+ROcp29_619*qd[20];
    ORcp29_120 = OMcp29_219*RLcp29_320-OMcp29_319*RLcp29_220;
    ORcp29_220 = -(OMcp29_119*RLcp29_320-OMcp29_319*RLcp29_120);
    ORcp29_320 = OMcp29_119*RLcp29_220-OMcp29_219*RLcp29_120;
    OPcp29_120 = OPcp29_119+ROcp29_419*qdd[20]+qd[20]*(OMcp29_219*ROcp29_619-OMcp29_319*ROcp29_519);
    OPcp29_220 = OPcp29_219+ROcp29_519*qdd[20]-qd[20]*(OMcp29_119*ROcp29_619-OMcp29_319*ROcp29_419);
    OPcp29_320 = OPcp29_319+ROcp29_619*qdd[20]+qd[20]*(OMcp29_119*ROcp29_519-OMcp29_219*ROcp29_419);
    RLcp29_165 = s->dpt[1][40]*ROcp29_120+s->dpt[2][40]*ROcp29_419+ROcp29_720*s->dpt[3][40];
    RLcp29_265 = s->dpt[1][40]*ROcp29_220+s->dpt[2][40]*ROcp29_519+ROcp29_820*s->dpt[3][40];
    RLcp29_365 = s->dpt[1][40]*ROcp29_320+s->dpt[2][40]*ROcp29_619+ROcp29_920*s->dpt[3][40];
    POcp29_165 = RLcp29_119+RLcp29_120+RLcp29_165+q[1];
    POcp29_265 = RLcp29_219+RLcp29_220+RLcp29_265+q[2];
    POcp29_365 = RLcp29_319+RLcp29_320+RLcp29_365+q[3];
    JTcp29_265_4 = -(RLcp29_319+RLcp29_320+RLcp29_365);
    JTcp29_365_4 = RLcp29_219+RLcp29_220+RLcp29_265;
    JTcp29_165_5 = C4*(RLcp29_319+RLcp29_320)-S4*(RLcp29_219+RLcp29_220)-RLcp29_265*S4+RLcp29_365*C4;
    JTcp29_265_5 = S4*(RLcp29_119+RLcp29_120+RLcp29_165);
    JTcp29_365_5 = -C4*(RLcp29_119+RLcp29_120+RLcp29_165);
    JTcp29_165_6 = ROcp29_85*(RLcp29_319+RLcp29_320)-ROcp29_95*(RLcp29_219+RLcp29_220)-RLcp29_265*ROcp29_95+RLcp29_365*
 ROcp29_85;
    JTcp29_265_6 = -(RLcp29_365*S5-ROcp29_95*(RLcp29_119+RLcp29_120+RLcp29_165)+S5*(RLcp29_319+RLcp29_320));
    JTcp29_365_6 = RLcp29_265*S5-ROcp29_85*(RLcp29_119+RLcp29_120+RLcp29_165)+S5*(RLcp29_219+RLcp29_220);
    JTcp29_165_7 = ROcp29_26*(RLcp29_320+RLcp29_365)-ROcp29_36*(RLcp29_220+RLcp29_265);
    JTcp29_265_7 = -(ROcp29_16*(RLcp29_320+RLcp29_365)-ROcp29_36*(RLcp29_120+RLcp29_165));
    JTcp29_365_7 = ROcp29_16*(RLcp29_220+RLcp29_265)-ROcp29_26*(RLcp29_120+RLcp29_165);
    JTcp29_165_8 = -(RLcp29_265*ROcp29_619-RLcp29_365*ROcp29_519);
    JTcp29_265_8 = RLcp29_165*ROcp29_619-RLcp29_365*ROcp29_419;
    JTcp29_365_8 = -(RLcp29_165*ROcp29_519-RLcp29_265*ROcp29_419);
    ORcp29_165 = OMcp29_220*RLcp29_365-OMcp29_320*RLcp29_265;
    ORcp29_265 = -(OMcp29_120*RLcp29_365-OMcp29_320*RLcp29_165);
    ORcp29_365 = OMcp29_120*RLcp29_265-OMcp29_220*RLcp29_165;
    VIcp29_165 = ORcp29_119+ORcp29_120+ORcp29_165+qd[1];
    VIcp29_265 = ORcp29_219+ORcp29_220+ORcp29_265+qd[2];
    VIcp29_365 = ORcp29_319+ORcp29_320+ORcp29_365+qd[3];
    ACcp29_165 = qdd[1]+OMcp29_219*ORcp29_320+OMcp29_220*ORcp29_365+OMcp29_26*ORcp29_319-OMcp29_319*ORcp29_220-OMcp29_320*
 ORcp29_265-OMcp29_36*ORcp29_219+OPcp29_219*RLcp29_320+OPcp29_220*RLcp29_365+OPcp29_26*RLcp29_319-OPcp29_319*RLcp29_220-
 OPcp29_320*RLcp29_265-OPcp29_36*RLcp29_219;
    ACcp29_265 = qdd[2]-OMcp29_119*ORcp29_320-OMcp29_120*ORcp29_365-OMcp29_16*ORcp29_319+OMcp29_319*ORcp29_120+OMcp29_320*
 ORcp29_165+OMcp29_36*ORcp29_119-OPcp29_119*RLcp29_320-OPcp29_120*RLcp29_365-OPcp29_16*RLcp29_319+OPcp29_319*RLcp29_120+
 OPcp29_320*RLcp29_165+OPcp29_36*RLcp29_119;
    ACcp29_365 = qdd[3]+OMcp29_119*ORcp29_220+OMcp29_120*ORcp29_265+OMcp29_16*ORcp29_219-OMcp29_219*ORcp29_120-OMcp29_220*
 ORcp29_165-OMcp29_26*ORcp29_119+OPcp29_119*RLcp29_220+OPcp29_120*RLcp29_265+OPcp29_16*RLcp29_219-OPcp29_219*RLcp29_120-
 OPcp29_220*RLcp29_165-OPcp29_26*RLcp29_119;

// = = Block_1_0_0_30_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp29_165;
    sens->P[2] = POcp29_265;
    sens->P[3] = POcp29_365;
    sens->R[1][1] = ROcp29_120;
    sens->R[1][2] = ROcp29_220;
    sens->R[1][3] = ROcp29_320;
    sens->R[2][1] = ROcp29_419;
    sens->R[2][2] = ROcp29_519;
    sens->R[2][3] = ROcp29_619;
    sens->R[3][1] = ROcp29_720;
    sens->R[3][2] = ROcp29_820;
    sens->R[3][3] = ROcp29_920;
    sens->V[1] = VIcp29_165;
    sens->V[2] = VIcp29_265;
    sens->V[3] = VIcp29_365;
    sens->OM[1] = OMcp29_120;
    sens->OM[2] = OMcp29_220;
    sens->OM[3] = OMcp29_320;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp29_165_5;
    sens->J[1][6] = JTcp29_165_6;
    sens->J[1][19] = JTcp29_165_7;
    sens->J[1][20] = JTcp29_165_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp29_265_4;
    sens->J[2][5] = JTcp29_265_5;
    sens->J[2][6] = JTcp29_265_6;
    sens->J[2][19] = JTcp29_265_7;
    sens->J[2][20] = JTcp29_265_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp29_365_4;
    sens->J[3][5] = JTcp29_365_5;
    sens->J[3][6] = JTcp29_365_6;
    sens->J[3][19] = JTcp29_365_7;
    sens->J[3][20] = JTcp29_365_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp29_16;
    sens->J[4][20] = ROcp29_419;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp29_85;
    sens->J[5][19] = ROcp29_26;
    sens->J[5][20] = ROcp29_519;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp29_95;
    sens->J[6][19] = ROcp29_36;
    sens->J[6][20] = ROcp29_619;
    sens->A[1] = ACcp29_165;
    sens->A[2] = ACcp29_265;
    sens->A[3] = ACcp29_365;
    sens->OMP[1] = OPcp29_120;
    sens->OMP[2] = OPcp29_220;
    sens->OMP[3] = OPcp29_320;
 
// 
break;
case 31:
 


// = = Block_1_0_0_31_0_1 = = 
 
// Sensor Kinematics 


    ROcp30_25 = S4*S5;
    ROcp30_35 = -C4*S5;
    ROcp30_85 = -S4*C5;
    ROcp30_95 = C4*C5;
    ROcp30_16 = C5*C6;
    ROcp30_26 = ROcp30_25*C6+C4*S6;
    ROcp30_36 = ROcp30_35*C6+S4*S6;
    ROcp30_46 = -C5*S6;
    ROcp30_56 = -(ROcp30_25*S6-C4*C6);
    ROcp30_66 = -(ROcp30_35*S6-S4*C6);
    OMcp30_25 = qd[5]*C4;
    OMcp30_35 = qd[5]*S4;
    OMcp30_16 = qd[4]+qd[6]*S5;
    OMcp30_26 = OMcp30_25+ROcp30_85*qd[6];
    OMcp30_36 = OMcp30_35+ROcp30_95*qd[6];
    OPcp30_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp30_26 = ROcp30_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp30_35*S5-ROcp30_95*qd[4]);
    OPcp30_36 = ROcp30_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp30_25*S5-ROcp30_85*qd[4]);

// = = Block_1_0_0_31_0_4 = = 
 
// Sensor Kinematics 


    ROcp30_419 = ROcp30_46*C19+S19*S5;
    ROcp30_519 = ROcp30_56*C19+ROcp30_85*S19;
    ROcp30_619 = ROcp30_66*C19+ROcp30_95*S19;
    ROcp30_719 = -(ROcp30_46*S19-C19*S5);
    ROcp30_819 = -(ROcp30_56*S19-ROcp30_85*C19);
    ROcp30_919 = -(ROcp30_66*S19-ROcp30_95*C19);
    ROcp30_120 = ROcp30_16*C20-ROcp30_719*S20;
    ROcp30_220 = ROcp30_26*C20-ROcp30_819*S20;
    ROcp30_320 = ROcp30_36*C20-ROcp30_919*S20;
    ROcp30_720 = ROcp30_16*S20+ROcp30_719*C20;
    ROcp30_820 = ROcp30_26*S20+ROcp30_819*C20;
    ROcp30_920 = ROcp30_36*S20+ROcp30_919*C20;
    RLcp30_119 = s->dpt[2][3]*ROcp30_46+ROcp30_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp30_219 = s->dpt[2][3]*ROcp30_56+ROcp30_26*s->dpt[1][3]+ROcp30_85*s->dpt[3][3];
    RLcp30_319 = s->dpt[2][3]*ROcp30_66+ROcp30_36*s->dpt[1][3]+ROcp30_95*s->dpt[3][3];
    OMcp30_119 = OMcp30_16+ROcp30_16*qd[19];
    OMcp30_219 = OMcp30_26+ROcp30_26*qd[19];
    OMcp30_319 = OMcp30_36+ROcp30_36*qd[19];
    ORcp30_119 = OMcp30_26*RLcp30_319-OMcp30_36*RLcp30_219;
    ORcp30_219 = -(OMcp30_16*RLcp30_319-OMcp30_36*RLcp30_119);
    ORcp30_319 = OMcp30_16*RLcp30_219-OMcp30_26*RLcp30_119;
    OPcp30_119 = OPcp30_16+ROcp30_16*qdd[19]+qd[19]*(OMcp30_26*ROcp30_36-OMcp30_36*ROcp30_26);
    OPcp30_219 = OPcp30_26+ROcp30_26*qdd[19]-qd[19]*(OMcp30_16*ROcp30_36-OMcp30_36*ROcp30_16);
    OPcp30_319 = OPcp30_36+ROcp30_36*qdd[19]+qd[19]*(OMcp30_16*ROcp30_26-OMcp30_26*ROcp30_16);
    RLcp30_120 = s->dpt[1][38]*ROcp30_16+s->dpt[2][38]*ROcp30_419+s->dpt[3][38]*ROcp30_719;
    RLcp30_220 = s->dpt[1][38]*ROcp30_26+s->dpt[2][38]*ROcp30_519+s->dpt[3][38]*ROcp30_819;
    RLcp30_320 = s->dpt[1][38]*ROcp30_36+s->dpt[2][38]*ROcp30_619+s->dpt[3][38]*ROcp30_919;
    OMcp30_120 = OMcp30_119+ROcp30_419*qd[20];
    OMcp30_220 = OMcp30_219+ROcp30_519*qd[20];
    OMcp30_320 = OMcp30_319+ROcp30_619*qd[20];
    ORcp30_120 = OMcp30_219*RLcp30_320-OMcp30_319*RLcp30_220;
    ORcp30_220 = -(OMcp30_119*RLcp30_320-OMcp30_319*RLcp30_120);
    ORcp30_320 = OMcp30_119*RLcp30_220-OMcp30_219*RLcp30_120;
    OPcp30_120 = OPcp30_119+ROcp30_419*qdd[20]+qd[20]*(OMcp30_219*ROcp30_619-OMcp30_319*ROcp30_519);
    OPcp30_220 = OPcp30_219+ROcp30_519*qdd[20]-qd[20]*(OMcp30_119*ROcp30_619-OMcp30_319*ROcp30_419);
    OPcp30_320 = OPcp30_319+ROcp30_619*qdd[20]+qd[20]*(OMcp30_119*ROcp30_519-OMcp30_219*ROcp30_419);
    RLcp30_166 = ROcp30_120*s->dpt[1][41]+ROcp30_419*s->dpt[2][41]+ROcp30_720*s->dpt[3][41];
    RLcp30_266 = ROcp30_220*s->dpt[1][41]+ROcp30_519*s->dpt[2][41]+ROcp30_820*s->dpt[3][41];
    RLcp30_366 = ROcp30_320*s->dpt[1][41]+ROcp30_619*s->dpt[2][41]+ROcp30_920*s->dpt[3][41];
    POcp30_166 = RLcp30_119+RLcp30_120+RLcp30_166+q[1];
    POcp30_266 = RLcp30_219+RLcp30_220+RLcp30_266+q[2];
    POcp30_366 = RLcp30_319+RLcp30_320+RLcp30_366+q[3];
    JTcp30_266_4 = -(RLcp30_319+RLcp30_320+RLcp30_366);
    JTcp30_366_4 = RLcp30_219+RLcp30_220+RLcp30_266;
    JTcp30_166_5 = C4*(RLcp30_319+RLcp30_320)-S4*(RLcp30_219+RLcp30_220)-RLcp30_266*S4+RLcp30_366*C4;
    JTcp30_266_5 = S4*(RLcp30_119+RLcp30_120+RLcp30_166);
    JTcp30_366_5 = -C4*(RLcp30_119+RLcp30_120+RLcp30_166);
    JTcp30_166_6 = ROcp30_85*(RLcp30_319+RLcp30_320)-ROcp30_95*(RLcp30_219+RLcp30_220)-RLcp30_266*ROcp30_95+RLcp30_366*
 ROcp30_85;
    JTcp30_266_6 = -(RLcp30_366*S5-ROcp30_95*(RLcp30_119+RLcp30_120+RLcp30_166)+S5*(RLcp30_319+RLcp30_320));
    JTcp30_366_6 = RLcp30_266*S5-ROcp30_85*(RLcp30_119+RLcp30_120+RLcp30_166)+S5*(RLcp30_219+RLcp30_220);
    JTcp30_166_7 = ROcp30_26*(RLcp30_320+RLcp30_366)-ROcp30_36*(RLcp30_220+RLcp30_266);
    JTcp30_266_7 = -(ROcp30_16*(RLcp30_320+RLcp30_366)-ROcp30_36*(RLcp30_120+RLcp30_166));
    JTcp30_366_7 = ROcp30_16*(RLcp30_220+RLcp30_266)-ROcp30_26*(RLcp30_120+RLcp30_166);
    JTcp30_166_8 = -(RLcp30_266*ROcp30_619-RLcp30_366*ROcp30_519);
    JTcp30_266_8 = RLcp30_166*ROcp30_619-RLcp30_366*ROcp30_419;
    JTcp30_366_8 = -(RLcp30_166*ROcp30_519-RLcp30_266*ROcp30_419);
    ORcp30_166 = OMcp30_220*RLcp30_366-OMcp30_320*RLcp30_266;
    ORcp30_266 = -(OMcp30_120*RLcp30_366-OMcp30_320*RLcp30_166);
    ORcp30_366 = OMcp30_120*RLcp30_266-OMcp30_220*RLcp30_166;
    VIcp30_166 = ORcp30_119+ORcp30_120+ORcp30_166+qd[1];
    VIcp30_266 = ORcp30_219+ORcp30_220+ORcp30_266+qd[2];
    VIcp30_366 = ORcp30_319+ORcp30_320+ORcp30_366+qd[3];
    ACcp30_166 = qdd[1]+OMcp30_219*ORcp30_320+OMcp30_220*ORcp30_366+OMcp30_26*ORcp30_319-OMcp30_319*ORcp30_220-OMcp30_320*
 ORcp30_266-OMcp30_36*ORcp30_219+OPcp30_219*RLcp30_320+OPcp30_220*RLcp30_366+OPcp30_26*RLcp30_319-OPcp30_319*RLcp30_220-
 OPcp30_320*RLcp30_266-OPcp30_36*RLcp30_219;
    ACcp30_266 = qdd[2]-OMcp30_119*ORcp30_320-OMcp30_120*ORcp30_366-OMcp30_16*ORcp30_319+OMcp30_319*ORcp30_120+OMcp30_320*
 ORcp30_166+OMcp30_36*ORcp30_119-OPcp30_119*RLcp30_320-OPcp30_120*RLcp30_366-OPcp30_16*RLcp30_319+OPcp30_319*RLcp30_120+
 OPcp30_320*RLcp30_166+OPcp30_36*RLcp30_119;
    ACcp30_366 = qdd[3]+OMcp30_119*ORcp30_220+OMcp30_120*ORcp30_266+OMcp30_16*ORcp30_219-OMcp30_219*ORcp30_120-OMcp30_220*
 ORcp30_166-OMcp30_26*ORcp30_119+OPcp30_119*RLcp30_220+OPcp30_120*RLcp30_266+OPcp30_16*RLcp30_219-OPcp30_219*RLcp30_120-
 OPcp30_220*RLcp30_166-OPcp30_26*RLcp30_119;

// = = Block_1_0_0_31_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp30_166;
    sens->P[2] = POcp30_266;
    sens->P[3] = POcp30_366;
    sens->R[1][1] = ROcp30_120;
    sens->R[1][2] = ROcp30_220;
    sens->R[1][3] = ROcp30_320;
    sens->R[2][1] = ROcp30_419;
    sens->R[2][2] = ROcp30_519;
    sens->R[2][3] = ROcp30_619;
    sens->R[3][1] = ROcp30_720;
    sens->R[3][2] = ROcp30_820;
    sens->R[3][3] = ROcp30_920;
    sens->V[1] = VIcp30_166;
    sens->V[2] = VIcp30_266;
    sens->V[3] = VIcp30_366;
    sens->OM[1] = OMcp30_120;
    sens->OM[2] = OMcp30_220;
    sens->OM[3] = OMcp30_320;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp30_166_5;
    sens->J[1][6] = JTcp30_166_6;
    sens->J[1][19] = JTcp30_166_7;
    sens->J[1][20] = JTcp30_166_8;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp30_266_4;
    sens->J[2][5] = JTcp30_266_5;
    sens->J[2][6] = JTcp30_266_6;
    sens->J[2][19] = JTcp30_266_7;
    sens->J[2][20] = JTcp30_266_8;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp30_366_4;
    sens->J[3][5] = JTcp30_366_5;
    sens->J[3][6] = JTcp30_366_6;
    sens->J[3][19] = JTcp30_366_7;
    sens->J[3][20] = JTcp30_366_8;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp30_16;
    sens->J[4][20] = ROcp30_419;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp30_85;
    sens->J[5][19] = ROcp30_26;
    sens->J[5][20] = ROcp30_519;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp30_95;
    sens->J[6][19] = ROcp30_36;
    sens->J[6][20] = ROcp30_619;
    sens->A[1] = ACcp30_166;
    sens->A[2] = ACcp30_266;
    sens->A[3] = ACcp30_366;
    sens->OMP[1] = OPcp30_120;
    sens->OMP[2] = OPcp30_220;
    sens->OMP[3] = OPcp30_320;
 
// 
break;
case 32:
 


// = = Block_1_0_0_32_0_1 = = 
 
// Sensor Kinematics 


    ROcp31_25 = S4*S5;
    ROcp31_35 = -C4*S5;
    ROcp31_85 = -S4*C5;
    ROcp31_95 = C4*C5;
    ROcp31_16 = C5*C6;
    ROcp31_26 = ROcp31_25*C6+C4*S6;
    ROcp31_36 = ROcp31_35*C6+S4*S6;
    ROcp31_46 = -C5*S6;
    ROcp31_56 = -(ROcp31_25*S6-C4*C6);
    ROcp31_66 = -(ROcp31_35*S6-S4*C6);
    OMcp31_25 = qd[5]*C4;
    OMcp31_35 = qd[5]*S4;
    OMcp31_16 = qd[4]+qd[6]*S5;
    OMcp31_26 = OMcp31_25+ROcp31_85*qd[6];
    OMcp31_36 = OMcp31_35+ROcp31_95*qd[6];
    OPcp31_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp31_26 = ROcp31_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp31_35*S5-ROcp31_95*qd[4]);
    OPcp31_36 = ROcp31_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp31_25*S5-ROcp31_85*qd[4]);

// = = Block_1_0_0_32_0_4 = = 
 
// Sensor Kinematics 


    ROcp31_419 = ROcp31_46*C19+S19*S5;
    ROcp31_519 = ROcp31_56*C19+ROcp31_85*S19;
    ROcp31_619 = ROcp31_66*C19+ROcp31_95*S19;
    ROcp31_719 = -(ROcp31_46*S19-C19*S5);
    ROcp31_819 = -(ROcp31_56*S19-ROcp31_85*C19);
    ROcp31_919 = -(ROcp31_66*S19-ROcp31_95*C19);
    ROcp31_120 = ROcp31_16*C20-ROcp31_719*S20;
    ROcp31_220 = ROcp31_26*C20-ROcp31_819*S20;
    ROcp31_320 = ROcp31_36*C20-ROcp31_919*S20;
    ROcp31_720 = ROcp31_16*S20+ROcp31_719*C20;
    ROcp31_820 = ROcp31_26*S20+ROcp31_819*C20;
    ROcp31_920 = ROcp31_36*S20+ROcp31_919*C20;
    ROcp31_121 = ROcp31_120*C21+ROcp31_419*S21;
    ROcp31_221 = ROcp31_220*C21+ROcp31_519*S21;
    ROcp31_321 = ROcp31_320*C21+ROcp31_619*S21;
    ROcp31_421 = -(ROcp31_120*S21-ROcp31_419*C21);
    ROcp31_521 = -(ROcp31_220*S21-ROcp31_519*C21);
    ROcp31_621 = -(ROcp31_320*S21-ROcp31_619*C21);
    RLcp31_119 = s->dpt[2][3]*ROcp31_46+ROcp31_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp31_219 = s->dpt[2][3]*ROcp31_56+ROcp31_26*s->dpt[1][3]+ROcp31_85*s->dpt[3][3];
    RLcp31_319 = s->dpt[2][3]*ROcp31_66+ROcp31_36*s->dpt[1][3]+ROcp31_95*s->dpt[3][3];
    OMcp31_119 = OMcp31_16+ROcp31_16*qd[19];
    OMcp31_219 = OMcp31_26+ROcp31_26*qd[19];
    OMcp31_319 = OMcp31_36+ROcp31_36*qd[19];
    ORcp31_119 = OMcp31_26*RLcp31_319-OMcp31_36*RLcp31_219;
    ORcp31_219 = -(OMcp31_16*RLcp31_319-OMcp31_36*RLcp31_119);
    ORcp31_319 = OMcp31_16*RLcp31_219-OMcp31_26*RLcp31_119;
    OPcp31_119 = OPcp31_16+ROcp31_16*qdd[19]+qd[19]*(OMcp31_26*ROcp31_36-OMcp31_36*ROcp31_26);
    OPcp31_219 = OPcp31_26+ROcp31_26*qdd[19]-qd[19]*(OMcp31_16*ROcp31_36-OMcp31_36*ROcp31_16);
    OPcp31_319 = OPcp31_36+ROcp31_36*qdd[19]+qd[19]*(OMcp31_16*ROcp31_26-OMcp31_26*ROcp31_16);
    RLcp31_120 = s->dpt[1][38]*ROcp31_16+s->dpt[2][38]*ROcp31_419+s->dpt[3][38]*ROcp31_719;
    RLcp31_220 = s->dpt[1][38]*ROcp31_26+s->dpt[2][38]*ROcp31_519+s->dpt[3][38]*ROcp31_819;
    RLcp31_320 = s->dpt[1][38]*ROcp31_36+s->dpt[2][38]*ROcp31_619+s->dpt[3][38]*ROcp31_919;
    OMcp31_120 = OMcp31_119+ROcp31_419*qd[20];
    OMcp31_220 = OMcp31_219+ROcp31_519*qd[20];
    OMcp31_320 = OMcp31_319+ROcp31_619*qd[20];
    ORcp31_120 = OMcp31_219*RLcp31_320-OMcp31_319*RLcp31_220;
    ORcp31_220 = -(OMcp31_119*RLcp31_320-OMcp31_319*RLcp31_120);
    ORcp31_320 = OMcp31_119*RLcp31_220-OMcp31_219*RLcp31_120;
    OPcp31_120 = OPcp31_119+ROcp31_419*qdd[20]+qd[20]*(OMcp31_219*ROcp31_619-OMcp31_319*ROcp31_519);
    OPcp31_220 = OPcp31_219+ROcp31_519*qdd[20]-qd[20]*(OMcp31_119*ROcp31_619-OMcp31_319*ROcp31_419);
    OPcp31_320 = OPcp31_319+ROcp31_619*qdd[20]+qd[20]*(OMcp31_119*ROcp31_519-OMcp31_219*ROcp31_419);
    RLcp31_121 = s->dpt[1][40]*ROcp31_120+s->dpt[2][40]*ROcp31_419+ROcp31_720*s->dpt[3][40];
    RLcp31_221 = s->dpt[1][40]*ROcp31_220+s->dpt[2][40]*ROcp31_519+ROcp31_820*s->dpt[3][40];
    RLcp31_321 = s->dpt[1][40]*ROcp31_320+s->dpt[2][40]*ROcp31_619+ROcp31_920*s->dpt[3][40];
    OMcp31_121 = OMcp31_120+ROcp31_720*qd[21];
    OMcp31_221 = OMcp31_220+ROcp31_820*qd[21];
    OMcp31_321 = OMcp31_320+ROcp31_920*qd[21];
    ORcp31_121 = OMcp31_220*RLcp31_321-OMcp31_320*RLcp31_221;
    ORcp31_221 = -(OMcp31_120*RLcp31_321-OMcp31_320*RLcp31_121);
    ORcp31_321 = OMcp31_120*RLcp31_221-OMcp31_220*RLcp31_121;
    OPcp31_121 = OPcp31_120+ROcp31_720*qdd[21]+qd[21]*(OMcp31_220*ROcp31_920-OMcp31_320*ROcp31_820);
    OPcp31_221 = OPcp31_220+ROcp31_820*qdd[21]-qd[21]*(OMcp31_120*ROcp31_920-OMcp31_320*ROcp31_720);
    OPcp31_321 = OPcp31_320+ROcp31_920*qdd[21]+qd[21]*(OMcp31_120*ROcp31_820-OMcp31_220*ROcp31_720);
    RLcp31_167 = s->dpt[1][42]*ROcp31_121+s->dpt[2][42]*ROcp31_421+ROcp31_720*s->dpt[3][42];
    RLcp31_267 = s->dpt[1][42]*ROcp31_221+s->dpt[2][42]*ROcp31_521+ROcp31_820*s->dpt[3][42];
    RLcp31_367 = s->dpt[1][42]*ROcp31_321+s->dpt[2][42]*ROcp31_621+ROcp31_920*s->dpt[3][42];
    POcp31_167 = RLcp31_119+RLcp31_120+RLcp31_121+RLcp31_167+q[1];
    POcp31_267 = RLcp31_219+RLcp31_220+RLcp31_221+RLcp31_267+q[2];
    POcp31_367 = RLcp31_319+RLcp31_320+RLcp31_321+RLcp31_367+q[3];
    JTcp31_267_4 = -(RLcp31_319+RLcp31_320+RLcp31_321+RLcp31_367);
    JTcp31_367_4 = RLcp31_219+RLcp31_220+RLcp31_221+RLcp31_267;
    JTcp31_167_5 = C4*(RLcp31_319+RLcp31_320+RLcp31_321+RLcp31_367)-S4*(RLcp31_219+RLcp31_220)-S4*(RLcp31_221+RLcp31_267);
    JTcp31_267_5 = S4*(RLcp31_119+RLcp31_120+RLcp31_121+RLcp31_167);
    JTcp31_367_5 = -C4*(RLcp31_119+RLcp31_120+RLcp31_121+RLcp31_167);
    JTcp31_167_6 = ROcp31_85*(RLcp31_319+RLcp31_320+RLcp31_321+RLcp31_367)-ROcp31_95*(RLcp31_219+RLcp31_220)-ROcp31_95*(
 RLcp31_221+RLcp31_267);
    JTcp31_267_6 = RLcp31_167*ROcp31_95-RLcp31_321*S5-RLcp31_367*S5+ROcp31_95*(RLcp31_119+RLcp31_120+RLcp31_121)-S5*(
 RLcp31_319+RLcp31_320);
    JTcp31_367_6 = RLcp31_221*S5-ROcp31_85*(RLcp31_119+RLcp31_120+RLcp31_121)+S5*(RLcp31_219+RLcp31_220)-RLcp31_167*
 ROcp31_85+RLcp31_267*S5;
    JTcp31_167_7 = ROcp31_26*(RLcp31_320+RLcp31_321)-ROcp31_36*(RLcp31_220+RLcp31_221)-RLcp31_267*ROcp31_36+RLcp31_367*
 ROcp31_26;
    JTcp31_267_7 = RLcp31_167*ROcp31_36-RLcp31_367*ROcp31_16-ROcp31_16*(RLcp31_320+RLcp31_321)+ROcp31_36*(RLcp31_120+
 RLcp31_121);
    JTcp31_367_7 = ROcp31_16*(RLcp31_220+RLcp31_221)-ROcp31_26*(RLcp31_120+RLcp31_121)-RLcp31_167*ROcp31_26+RLcp31_267*
 ROcp31_16;
    JTcp31_167_8 = ROcp31_519*(RLcp31_321+RLcp31_367)-ROcp31_619*(RLcp31_221+RLcp31_267);
    JTcp31_267_8 = -(ROcp31_419*(RLcp31_321+RLcp31_367)-ROcp31_619*(RLcp31_121+RLcp31_167));
    JTcp31_367_8 = ROcp31_419*(RLcp31_221+RLcp31_267)-ROcp31_519*(RLcp31_121+RLcp31_167);
    JTcp31_167_9 = -(RLcp31_267*ROcp31_920-RLcp31_367*ROcp31_820);
    JTcp31_267_9 = RLcp31_167*ROcp31_920-RLcp31_367*ROcp31_720;
    JTcp31_367_9 = -(RLcp31_167*ROcp31_820-RLcp31_267*ROcp31_720);
    ORcp31_167 = OMcp31_221*RLcp31_367-OMcp31_321*RLcp31_267;
    ORcp31_267 = -(OMcp31_121*RLcp31_367-OMcp31_321*RLcp31_167);
    ORcp31_367 = OMcp31_121*RLcp31_267-OMcp31_221*RLcp31_167;
    VIcp31_167 = ORcp31_119+ORcp31_120+ORcp31_121+ORcp31_167+qd[1];
    VIcp31_267 = ORcp31_219+ORcp31_220+ORcp31_221+ORcp31_267+qd[2];
    VIcp31_367 = ORcp31_319+ORcp31_320+ORcp31_321+ORcp31_367+qd[3];
    ACcp31_167 = qdd[1]+OMcp31_219*ORcp31_320+OMcp31_220*ORcp31_321+OMcp31_221*ORcp31_367+OMcp31_26*ORcp31_319-OMcp31_319*
 ORcp31_220-OMcp31_320*ORcp31_221-OMcp31_321*ORcp31_267-OMcp31_36*ORcp31_219+OPcp31_219*RLcp31_320+OPcp31_220*RLcp31_321+
 OPcp31_221*RLcp31_367+OPcp31_26*RLcp31_319-OPcp31_319*RLcp31_220-OPcp31_320*RLcp31_221-OPcp31_321*RLcp31_267-OPcp31_36*
 RLcp31_219;
    ACcp31_267 = qdd[2]-OMcp31_119*ORcp31_320-OMcp31_120*ORcp31_321-OMcp31_121*ORcp31_367-OMcp31_16*ORcp31_319+OMcp31_319*
 ORcp31_120+OMcp31_320*ORcp31_121+OMcp31_321*ORcp31_167+OMcp31_36*ORcp31_119-OPcp31_119*RLcp31_320-OPcp31_120*RLcp31_321-
 OPcp31_121*RLcp31_367-OPcp31_16*RLcp31_319+OPcp31_319*RLcp31_120+OPcp31_320*RLcp31_121+OPcp31_321*RLcp31_167+OPcp31_36*
 RLcp31_119;
    ACcp31_367 = qdd[3]+OMcp31_119*ORcp31_220+OMcp31_120*ORcp31_221+OMcp31_121*ORcp31_267+OMcp31_16*ORcp31_219-OMcp31_219*
 ORcp31_120-OMcp31_220*ORcp31_121-OMcp31_221*ORcp31_167-OMcp31_26*ORcp31_119+OPcp31_119*RLcp31_220+OPcp31_120*RLcp31_221+
 OPcp31_121*RLcp31_267+OPcp31_16*RLcp31_219-OPcp31_219*RLcp31_120-OPcp31_220*RLcp31_121-OPcp31_221*RLcp31_167-OPcp31_26*
 RLcp31_119;

// = = Block_1_0_0_32_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp31_167;
    sens->P[2] = POcp31_267;
    sens->P[3] = POcp31_367;
    sens->R[1][1] = ROcp31_121;
    sens->R[1][2] = ROcp31_221;
    sens->R[1][3] = ROcp31_321;
    sens->R[2][1] = ROcp31_421;
    sens->R[2][2] = ROcp31_521;
    sens->R[2][3] = ROcp31_621;
    sens->R[3][1] = ROcp31_720;
    sens->R[3][2] = ROcp31_820;
    sens->R[3][3] = ROcp31_920;
    sens->V[1] = VIcp31_167;
    sens->V[2] = VIcp31_267;
    sens->V[3] = VIcp31_367;
    sens->OM[1] = OMcp31_121;
    sens->OM[2] = OMcp31_221;
    sens->OM[3] = OMcp31_321;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp31_167_5;
    sens->J[1][6] = JTcp31_167_6;
    sens->J[1][19] = JTcp31_167_7;
    sens->J[1][20] = JTcp31_167_8;
    sens->J[1][21] = JTcp31_167_9;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp31_267_4;
    sens->J[2][5] = JTcp31_267_5;
    sens->J[2][6] = JTcp31_267_6;
    sens->J[2][19] = JTcp31_267_7;
    sens->J[2][20] = JTcp31_267_8;
    sens->J[2][21] = JTcp31_267_9;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp31_367_4;
    sens->J[3][5] = JTcp31_367_5;
    sens->J[3][6] = JTcp31_367_6;
    sens->J[3][19] = JTcp31_367_7;
    sens->J[3][20] = JTcp31_367_8;
    sens->J[3][21] = JTcp31_367_9;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp31_16;
    sens->J[4][20] = ROcp31_419;
    sens->J[4][21] = ROcp31_720;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp31_85;
    sens->J[5][19] = ROcp31_26;
    sens->J[5][20] = ROcp31_519;
    sens->J[5][21] = ROcp31_820;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp31_95;
    sens->J[6][19] = ROcp31_36;
    sens->J[6][20] = ROcp31_619;
    sens->J[6][21] = ROcp31_920;
    sens->A[1] = ACcp31_167;
    sens->A[2] = ACcp31_267;
    sens->A[3] = ACcp31_367;
    sens->OMP[1] = OPcp31_121;
    sens->OMP[2] = OPcp31_221;
    sens->OMP[3] = OPcp31_321;
 
// 
break;
case 33:
 


// = = Block_1_0_0_33_0_1 = = 
 
// Sensor Kinematics 


    ROcp32_25 = S4*S5;
    ROcp32_35 = -C4*S5;
    ROcp32_85 = -S4*C5;
    ROcp32_95 = C4*C5;
    ROcp32_16 = C5*C6;
    ROcp32_26 = ROcp32_25*C6+C4*S6;
    ROcp32_36 = ROcp32_35*C6+S4*S6;
    ROcp32_46 = -C5*S6;
    ROcp32_56 = -(ROcp32_25*S6-C4*C6);
    ROcp32_66 = -(ROcp32_35*S6-S4*C6);
    OMcp32_25 = qd[5]*C4;
    OMcp32_35 = qd[5]*S4;
    OMcp32_16 = qd[4]+qd[6]*S5;
    OMcp32_26 = OMcp32_25+ROcp32_85*qd[6];
    OMcp32_36 = OMcp32_35+ROcp32_95*qd[6];
    OPcp32_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp32_26 = ROcp32_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp32_35*S5-ROcp32_95*qd[4]);
    OPcp32_36 = ROcp32_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp32_25*S5-ROcp32_85*qd[4]);

// = = Block_1_0_0_33_0_4 = = 
 
// Sensor Kinematics 


    ROcp32_419 = ROcp32_46*C19+S19*S5;
    ROcp32_519 = ROcp32_56*C19+ROcp32_85*S19;
    ROcp32_619 = ROcp32_66*C19+ROcp32_95*S19;
    ROcp32_719 = -(ROcp32_46*S19-C19*S5);
    ROcp32_819 = -(ROcp32_56*S19-ROcp32_85*C19);
    ROcp32_919 = -(ROcp32_66*S19-ROcp32_95*C19);
    ROcp32_120 = ROcp32_16*C20-ROcp32_719*S20;
    ROcp32_220 = ROcp32_26*C20-ROcp32_819*S20;
    ROcp32_320 = ROcp32_36*C20-ROcp32_919*S20;
    ROcp32_720 = ROcp32_16*S20+ROcp32_719*C20;
    ROcp32_820 = ROcp32_26*S20+ROcp32_819*C20;
    ROcp32_920 = ROcp32_36*S20+ROcp32_919*C20;
    ROcp32_121 = ROcp32_120*C21+ROcp32_419*S21;
    ROcp32_221 = ROcp32_220*C21+ROcp32_519*S21;
    ROcp32_321 = ROcp32_320*C21+ROcp32_619*S21;
    ROcp32_421 = -(ROcp32_120*S21-ROcp32_419*C21);
    ROcp32_521 = -(ROcp32_220*S21-ROcp32_519*C21);
    ROcp32_621 = -(ROcp32_320*S21-ROcp32_619*C21);
    RLcp32_119 = s->dpt[2][3]*ROcp32_46+ROcp32_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp32_219 = s->dpt[2][3]*ROcp32_56+ROcp32_26*s->dpt[1][3]+ROcp32_85*s->dpt[3][3];
    RLcp32_319 = s->dpt[2][3]*ROcp32_66+ROcp32_36*s->dpt[1][3]+ROcp32_95*s->dpt[3][3];
    OMcp32_119 = OMcp32_16+ROcp32_16*qd[19];
    OMcp32_219 = OMcp32_26+ROcp32_26*qd[19];
    OMcp32_319 = OMcp32_36+ROcp32_36*qd[19];
    ORcp32_119 = OMcp32_26*RLcp32_319-OMcp32_36*RLcp32_219;
    ORcp32_219 = -(OMcp32_16*RLcp32_319-OMcp32_36*RLcp32_119);
    ORcp32_319 = OMcp32_16*RLcp32_219-OMcp32_26*RLcp32_119;
    OPcp32_119 = OPcp32_16+ROcp32_16*qdd[19]+qd[19]*(OMcp32_26*ROcp32_36-OMcp32_36*ROcp32_26);
    OPcp32_219 = OPcp32_26+ROcp32_26*qdd[19]-qd[19]*(OMcp32_16*ROcp32_36-OMcp32_36*ROcp32_16);
    OPcp32_319 = OPcp32_36+ROcp32_36*qdd[19]+qd[19]*(OMcp32_16*ROcp32_26-OMcp32_26*ROcp32_16);
    RLcp32_120 = s->dpt[1][38]*ROcp32_16+s->dpt[2][38]*ROcp32_419+s->dpt[3][38]*ROcp32_719;
    RLcp32_220 = s->dpt[1][38]*ROcp32_26+s->dpt[2][38]*ROcp32_519+s->dpt[3][38]*ROcp32_819;
    RLcp32_320 = s->dpt[1][38]*ROcp32_36+s->dpt[2][38]*ROcp32_619+s->dpt[3][38]*ROcp32_919;
    OMcp32_120 = OMcp32_119+ROcp32_419*qd[20];
    OMcp32_220 = OMcp32_219+ROcp32_519*qd[20];
    OMcp32_320 = OMcp32_319+ROcp32_619*qd[20];
    ORcp32_120 = OMcp32_219*RLcp32_320-OMcp32_319*RLcp32_220;
    ORcp32_220 = -(OMcp32_119*RLcp32_320-OMcp32_319*RLcp32_120);
    ORcp32_320 = OMcp32_119*RLcp32_220-OMcp32_219*RLcp32_120;
    OPcp32_120 = OPcp32_119+ROcp32_419*qdd[20]+qd[20]*(OMcp32_219*ROcp32_619-OMcp32_319*ROcp32_519);
    OPcp32_220 = OPcp32_219+ROcp32_519*qdd[20]-qd[20]*(OMcp32_119*ROcp32_619-OMcp32_319*ROcp32_419);
    OPcp32_320 = OPcp32_319+ROcp32_619*qdd[20]+qd[20]*(OMcp32_119*ROcp32_519-OMcp32_219*ROcp32_419);
    RLcp32_121 = s->dpt[1][40]*ROcp32_120+s->dpt[2][40]*ROcp32_419+ROcp32_720*s->dpt[3][40];
    RLcp32_221 = s->dpt[1][40]*ROcp32_220+s->dpt[2][40]*ROcp32_519+ROcp32_820*s->dpt[3][40];
    RLcp32_321 = s->dpt[1][40]*ROcp32_320+s->dpt[2][40]*ROcp32_619+ROcp32_920*s->dpt[3][40];
    OMcp32_121 = OMcp32_120+ROcp32_720*qd[21];
    OMcp32_221 = OMcp32_220+ROcp32_820*qd[21];
    OMcp32_321 = OMcp32_320+ROcp32_920*qd[21];
    ORcp32_121 = OMcp32_220*RLcp32_321-OMcp32_320*RLcp32_221;
    ORcp32_221 = -(OMcp32_120*RLcp32_321-OMcp32_320*RLcp32_121);
    ORcp32_321 = OMcp32_120*RLcp32_221-OMcp32_220*RLcp32_121;
    OPcp32_121 = OPcp32_120+ROcp32_720*qdd[21]+qd[21]*(OMcp32_220*ROcp32_920-OMcp32_320*ROcp32_820);
    OPcp32_221 = OPcp32_220+ROcp32_820*qdd[21]-qd[21]*(OMcp32_120*ROcp32_920-OMcp32_320*ROcp32_720);
    OPcp32_321 = OPcp32_320+ROcp32_920*qdd[21]+qd[21]*(OMcp32_120*ROcp32_820-OMcp32_220*ROcp32_720);
    RLcp32_168 = ROcp32_121*s->dpt[1][43]+ROcp32_421*s->dpt[2][43]+ROcp32_720*s->dpt[3][43];
    RLcp32_268 = ROcp32_221*s->dpt[1][43]+ROcp32_521*s->dpt[2][43]+ROcp32_820*s->dpt[3][43];
    RLcp32_368 = ROcp32_321*s->dpt[1][43]+ROcp32_621*s->dpt[2][43]+ROcp32_920*s->dpt[3][43];
    POcp32_168 = RLcp32_119+RLcp32_120+RLcp32_121+RLcp32_168+q[1];
    POcp32_268 = RLcp32_219+RLcp32_220+RLcp32_221+RLcp32_268+q[2];
    POcp32_368 = RLcp32_319+RLcp32_320+RLcp32_321+RLcp32_368+q[3];
    JTcp32_268_4 = -(RLcp32_319+RLcp32_320+RLcp32_321+RLcp32_368);
    JTcp32_368_4 = RLcp32_219+RLcp32_220+RLcp32_221+RLcp32_268;
    JTcp32_168_5 = C4*(RLcp32_319+RLcp32_320+RLcp32_321+RLcp32_368)-S4*(RLcp32_219+RLcp32_220)-S4*(RLcp32_221+RLcp32_268);
    JTcp32_268_5 = S4*(RLcp32_119+RLcp32_120+RLcp32_121+RLcp32_168);
    JTcp32_368_5 = -C4*(RLcp32_119+RLcp32_120+RLcp32_121+RLcp32_168);
    JTcp32_168_6 = ROcp32_85*(RLcp32_319+RLcp32_320+RLcp32_321+RLcp32_368)-ROcp32_95*(RLcp32_219+RLcp32_220)-ROcp32_95*(
 RLcp32_221+RLcp32_268);
    JTcp32_268_6 = RLcp32_168*ROcp32_95-RLcp32_321*S5-RLcp32_368*S5+ROcp32_95*(RLcp32_119+RLcp32_120+RLcp32_121)-S5*(
 RLcp32_319+RLcp32_320);
    JTcp32_368_6 = RLcp32_221*S5-ROcp32_85*(RLcp32_119+RLcp32_120+RLcp32_121)+S5*(RLcp32_219+RLcp32_220)-RLcp32_168*
 ROcp32_85+RLcp32_268*S5;
    JTcp32_168_7 = ROcp32_26*(RLcp32_320+RLcp32_321)-ROcp32_36*(RLcp32_220+RLcp32_221)-RLcp32_268*ROcp32_36+RLcp32_368*
 ROcp32_26;
    JTcp32_268_7 = RLcp32_168*ROcp32_36-RLcp32_368*ROcp32_16-ROcp32_16*(RLcp32_320+RLcp32_321)+ROcp32_36*(RLcp32_120+
 RLcp32_121);
    JTcp32_368_7 = ROcp32_16*(RLcp32_220+RLcp32_221)-ROcp32_26*(RLcp32_120+RLcp32_121)-RLcp32_168*ROcp32_26+RLcp32_268*
 ROcp32_16;
    JTcp32_168_8 = ROcp32_519*(RLcp32_321+RLcp32_368)-ROcp32_619*(RLcp32_221+RLcp32_268);
    JTcp32_268_8 = -(ROcp32_419*(RLcp32_321+RLcp32_368)-ROcp32_619*(RLcp32_121+RLcp32_168));
    JTcp32_368_8 = ROcp32_419*(RLcp32_221+RLcp32_268)-ROcp32_519*(RLcp32_121+RLcp32_168);
    JTcp32_168_9 = -(RLcp32_268*ROcp32_920-RLcp32_368*ROcp32_820);
    JTcp32_268_9 = RLcp32_168*ROcp32_920-RLcp32_368*ROcp32_720;
    JTcp32_368_9 = -(RLcp32_168*ROcp32_820-RLcp32_268*ROcp32_720);
    ORcp32_168 = OMcp32_221*RLcp32_368-OMcp32_321*RLcp32_268;
    ORcp32_268 = -(OMcp32_121*RLcp32_368-OMcp32_321*RLcp32_168);
    ORcp32_368 = OMcp32_121*RLcp32_268-OMcp32_221*RLcp32_168;
    VIcp32_168 = ORcp32_119+ORcp32_120+ORcp32_121+ORcp32_168+qd[1];
    VIcp32_268 = ORcp32_219+ORcp32_220+ORcp32_221+ORcp32_268+qd[2];
    VIcp32_368 = ORcp32_319+ORcp32_320+ORcp32_321+ORcp32_368+qd[3];
    ACcp32_168 = qdd[1]+OMcp32_219*ORcp32_320+OMcp32_220*ORcp32_321+OMcp32_221*ORcp32_368+OMcp32_26*ORcp32_319-OMcp32_319*
 ORcp32_220-OMcp32_320*ORcp32_221-OMcp32_321*ORcp32_268-OMcp32_36*ORcp32_219+OPcp32_219*RLcp32_320+OPcp32_220*RLcp32_321+
 OPcp32_221*RLcp32_368+OPcp32_26*RLcp32_319-OPcp32_319*RLcp32_220-OPcp32_320*RLcp32_221-OPcp32_321*RLcp32_268-OPcp32_36*
 RLcp32_219;
    ACcp32_268 = qdd[2]-OMcp32_119*ORcp32_320-OMcp32_120*ORcp32_321-OMcp32_121*ORcp32_368-OMcp32_16*ORcp32_319+OMcp32_319*
 ORcp32_120+OMcp32_320*ORcp32_121+OMcp32_321*ORcp32_168+OMcp32_36*ORcp32_119-OPcp32_119*RLcp32_320-OPcp32_120*RLcp32_321-
 OPcp32_121*RLcp32_368-OPcp32_16*RLcp32_319+OPcp32_319*RLcp32_120+OPcp32_320*RLcp32_121+OPcp32_321*RLcp32_168+OPcp32_36*
 RLcp32_119;
    ACcp32_368 = qdd[3]+OMcp32_119*ORcp32_220+OMcp32_120*ORcp32_221+OMcp32_121*ORcp32_268+OMcp32_16*ORcp32_219-OMcp32_219*
 ORcp32_120-OMcp32_220*ORcp32_121-OMcp32_221*ORcp32_168-OMcp32_26*ORcp32_119+OPcp32_119*RLcp32_220+OPcp32_120*RLcp32_221+
 OPcp32_121*RLcp32_268+OPcp32_16*RLcp32_219-OPcp32_219*RLcp32_120-OPcp32_220*RLcp32_121-OPcp32_221*RLcp32_168-OPcp32_26*
 RLcp32_119;

// = = Block_1_0_0_33_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp32_168;
    sens->P[2] = POcp32_268;
    sens->P[3] = POcp32_368;
    sens->R[1][1] = ROcp32_121;
    sens->R[1][2] = ROcp32_221;
    sens->R[1][3] = ROcp32_321;
    sens->R[2][1] = ROcp32_421;
    sens->R[2][2] = ROcp32_521;
    sens->R[2][3] = ROcp32_621;
    sens->R[3][1] = ROcp32_720;
    sens->R[3][2] = ROcp32_820;
    sens->R[3][3] = ROcp32_920;
    sens->V[1] = VIcp32_168;
    sens->V[2] = VIcp32_268;
    sens->V[3] = VIcp32_368;
    sens->OM[1] = OMcp32_121;
    sens->OM[2] = OMcp32_221;
    sens->OM[3] = OMcp32_321;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp32_168_5;
    sens->J[1][6] = JTcp32_168_6;
    sens->J[1][19] = JTcp32_168_7;
    sens->J[1][20] = JTcp32_168_8;
    sens->J[1][21] = JTcp32_168_9;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp32_268_4;
    sens->J[2][5] = JTcp32_268_5;
    sens->J[2][6] = JTcp32_268_6;
    sens->J[2][19] = JTcp32_268_7;
    sens->J[2][20] = JTcp32_268_8;
    sens->J[2][21] = JTcp32_268_9;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp32_368_4;
    sens->J[3][5] = JTcp32_368_5;
    sens->J[3][6] = JTcp32_368_6;
    sens->J[3][19] = JTcp32_368_7;
    sens->J[3][20] = JTcp32_368_8;
    sens->J[3][21] = JTcp32_368_9;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp32_16;
    sens->J[4][20] = ROcp32_419;
    sens->J[4][21] = ROcp32_720;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp32_85;
    sens->J[5][19] = ROcp32_26;
    sens->J[5][20] = ROcp32_519;
    sens->J[5][21] = ROcp32_820;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp32_95;
    sens->J[6][19] = ROcp32_36;
    sens->J[6][20] = ROcp32_619;
    sens->J[6][21] = ROcp32_920;
    sens->A[1] = ACcp32_168;
    sens->A[2] = ACcp32_268;
    sens->A[3] = ACcp32_368;
    sens->OMP[1] = OPcp32_121;
    sens->OMP[2] = OPcp32_221;
    sens->OMP[3] = OPcp32_321;
 
// 
break;
case 34:
 


// = = Block_1_0_0_34_0_1 = = 
 
// Sensor Kinematics 


    ROcp33_25 = S4*S5;
    ROcp33_35 = -C4*S5;
    ROcp33_85 = -S4*C5;
    ROcp33_95 = C4*C5;
    ROcp33_16 = C5*C6;
    ROcp33_26 = ROcp33_25*C6+C4*S6;
    ROcp33_36 = ROcp33_35*C6+S4*S6;
    ROcp33_46 = -C5*S6;
    ROcp33_56 = -(ROcp33_25*S6-C4*C6);
    ROcp33_66 = -(ROcp33_35*S6-S4*C6);
    OMcp33_25 = qd[5]*C4;
    OMcp33_35 = qd[5]*S4;
    OMcp33_16 = qd[4]+qd[6]*S5;
    OMcp33_26 = OMcp33_25+ROcp33_85*qd[6];
    OMcp33_36 = OMcp33_35+ROcp33_95*qd[6];
    OPcp33_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp33_26 = ROcp33_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp33_35*S5-ROcp33_95*qd[4]);
    OPcp33_36 = ROcp33_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp33_25*S5-ROcp33_85*qd[4]);

// = = Block_1_0_0_34_0_4 = = 
 
// Sensor Kinematics 


    ROcp33_419 = ROcp33_46*C19+S19*S5;
    ROcp33_519 = ROcp33_56*C19+ROcp33_85*S19;
    ROcp33_619 = ROcp33_66*C19+ROcp33_95*S19;
    ROcp33_719 = -(ROcp33_46*S19-C19*S5);
    ROcp33_819 = -(ROcp33_56*S19-ROcp33_85*C19);
    ROcp33_919 = -(ROcp33_66*S19-ROcp33_95*C19);
    ROcp33_120 = ROcp33_16*C20-ROcp33_719*S20;
    ROcp33_220 = ROcp33_26*C20-ROcp33_819*S20;
    ROcp33_320 = ROcp33_36*C20-ROcp33_919*S20;
    ROcp33_720 = ROcp33_16*S20+ROcp33_719*C20;
    ROcp33_820 = ROcp33_26*S20+ROcp33_819*C20;
    ROcp33_920 = ROcp33_36*S20+ROcp33_919*C20;
    ROcp33_121 = ROcp33_120*C21+ROcp33_419*S21;
    ROcp33_221 = ROcp33_220*C21+ROcp33_519*S21;
    ROcp33_321 = ROcp33_320*C21+ROcp33_619*S21;
    ROcp33_421 = -(ROcp33_120*S21-ROcp33_419*C21);
    ROcp33_521 = -(ROcp33_220*S21-ROcp33_519*C21);
    ROcp33_621 = -(ROcp33_320*S21-ROcp33_619*C21);
    RLcp33_119 = s->dpt[2][3]*ROcp33_46+ROcp33_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp33_219 = s->dpt[2][3]*ROcp33_56+ROcp33_26*s->dpt[1][3]+ROcp33_85*s->dpt[3][3];
    RLcp33_319 = s->dpt[2][3]*ROcp33_66+ROcp33_36*s->dpt[1][3]+ROcp33_95*s->dpt[3][3];
    OMcp33_119 = OMcp33_16+ROcp33_16*qd[19];
    OMcp33_219 = OMcp33_26+ROcp33_26*qd[19];
    OMcp33_319 = OMcp33_36+ROcp33_36*qd[19];
    ORcp33_119 = OMcp33_26*RLcp33_319-OMcp33_36*RLcp33_219;
    ORcp33_219 = -(OMcp33_16*RLcp33_319-OMcp33_36*RLcp33_119);
    ORcp33_319 = OMcp33_16*RLcp33_219-OMcp33_26*RLcp33_119;
    OPcp33_119 = OPcp33_16+ROcp33_16*qdd[19]+qd[19]*(OMcp33_26*ROcp33_36-OMcp33_36*ROcp33_26);
    OPcp33_219 = OPcp33_26+ROcp33_26*qdd[19]-qd[19]*(OMcp33_16*ROcp33_36-OMcp33_36*ROcp33_16);
    OPcp33_319 = OPcp33_36+ROcp33_36*qdd[19]+qd[19]*(OMcp33_16*ROcp33_26-OMcp33_26*ROcp33_16);
    RLcp33_120 = s->dpt[1][38]*ROcp33_16+s->dpt[2][38]*ROcp33_419+s->dpt[3][38]*ROcp33_719;
    RLcp33_220 = s->dpt[1][38]*ROcp33_26+s->dpt[2][38]*ROcp33_519+s->dpt[3][38]*ROcp33_819;
    RLcp33_320 = s->dpt[1][38]*ROcp33_36+s->dpt[2][38]*ROcp33_619+s->dpt[3][38]*ROcp33_919;
    OMcp33_120 = OMcp33_119+ROcp33_419*qd[20];
    OMcp33_220 = OMcp33_219+ROcp33_519*qd[20];
    OMcp33_320 = OMcp33_319+ROcp33_619*qd[20];
    ORcp33_120 = OMcp33_219*RLcp33_320-OMcp33_319*RLcp33_220;
    ORcp33_220 = -(OMcp33_119*RLcp33_320-OMcp33_319*RLcp33_120);
    ORcp33_320 = OMcp33_119*RLcp33_220-OMcp33_219*RLcp33_120;
    OPcp33_120 = OPcp33_119+ROcp33_419*qdd[20]+qd[20]*(OMcp33_219*ROcp33_619-OMcp33_319*ROcp33_519);
    OPcp33_220 = OPcp33_219+ROcp33_519*qdd[20]-qd[20]*(OMcp33_119*ROcp33_619-OMcp33_319*ROcp33_419);
    OPcp33_320 = OPcp33_319+ROcp33_619*qdd[20]+qd[20]*(OMcp33_119*ROcp33_519-OMcp33_219*ROcp33_419);
    RLcp33_121 = s->dpt[1][40]*ROcp33_120+s->dpt[2][40]*ROcp33_419+ROcp33_720*s->dpt[3][40];
    RLcp33_221 = s->dpt[1][40]*ROcp33_220+s->dpt[2][40]*ROcp33_519+ROcp33_820*s->dpt[3][40];
    RLcp33_321 = s->dpt[1][40]*ROcp33_320+s->dpt[2][40]*ROcp33_619+ROcp33_920*s->dpt[3][40];
    OMcp33_121 = OMcp33_120+ROcp33_720*qd[21];
    OMcp33_221 = OMcp33_220+ROcp33_820*qd[21];
    OMcp33_321 = OMcp33_320+ROcp33_920*qd[21];
    ORcp33_121 = OMcp33_220*RLcp33_321-OMcp33_320*RLcp33_221;
    ORcp33_221 = -(OMcp33_120*RLcp33_321-OMcp33_320*RLcp33_121);
    ORcp33_321 = OMcp33_120*RLcp33_221-OMcp33_220*RLcp33_121;
    OPcp33_121 = OPcp33_120+ROcp33_720*qdd[21]+qd[21]*(OMcp33_220*ROcp33_920-OMcp33_320*ROcp33_820);
    OPcp33_221 = OPcp33_220+ROcp33_820*qdd[21]-qd[21]*(OMcp33_120*ROcp33_920-OMcp33_320*ROcp33_720);
    OPcp33_321 = OPcp33_320+ROcp33_920*qdd[21]+qd[21]*(OMcp33_120*ROcp33_820-OMcp33_220*ROcp33_720);
    RLcp33_169 = ROcp33_121*s->dpt[1][44]+ROcp33_421*s->dpt[2][44]+ROcp33_720*s->dpt[3][44];
    RLcp33_269 = ROcp33_221*s->dpt[1][44]+ROcp33_521*s->dpt[2][44]+ROcp33_820*s->dpt[3][44];
    RLcp33_369 = ROcp33_321*s->dpt[1][44]+ROcp33_621*s->dpt[2][44]+ROcp33_920*s->dpt[3][44];
    POcp33_169 = RLcp33_119+RLcp33_120+RLcp33_121+RLcp33_169+q[1];
    POcp33_269 = RLcp33_219+RLcp33_220+RLcp33_221+RLcp33_269+q[2];
    POcp33_369 = RLcp33_319+RLcp33_320+RLcp33_321+RLcp33_369+q[3];
    JTcp33_269_4 = -(RLcp33_319+RLcp33_320+RLcp33_321+RLcp33_369);
    JTcp33_369_4 = RLcp33_219+RLcp33_220+RLcp33_221+RLcp33_269;
    JTcp33_169_5 = C4*(RLcp33_319+RLcp33_320+RLcp33_321+RLcp33_369)-S4*(RLcp33_219+RLcp33_220)-S4*(RLcp33_221+RLcp33_269);
    JTcp33_269_5 = S4*(RLcp33_119+RLcp33_120+RLcp33_121+RLcp33_169);
    JTcp33_369_5 = -C4*(RLcp33_119+RLcp33_120+RLcp33_121+RLcp33_169);
    JTcp33_169_6 = ROcp33_85*(RLcp33_319+RLcp33_320+RLcp33_321+RLcp33_369)-ROcp33_95*(RLcp33_219+RLcp33_220)-ROcp33_95*(
 RLcp33_221+RLcp33_269);
    JTcp33_269_6 = RLcp33_169*ROcp33_95-RLcp33_321*S5-RLcp33_369*S5+ROcp33_95*(RLcp33_119+RLcp33_120+RLcp33_121)-S5*(
 RLcp33_319+RLcp33_320);
    JTcp33_369_6 = RLcp33_221*S5-ROcp33_85*(RLcp33_119+RLcp33_120+RLcp33_121)+S5*(RLcp33_219+RLcp33_220)-RLcp33_169*
 ROcp33_85+RLcp33_269*S5;
    JTcp33_169_7 = ROcp33_26*(RLcp33_320+RLcp33_321)-ROcp33_36*(RLcp33_220+RLcp33_221)-RLcp33_269*ROcp33_36+RLcp33_369*
 ROcp33_26;
    JTcp33_269_7 = RLcp33_169*ROcp33_36-RLcp33_369*ROcp33_16-ROcp33_16*(RLcp33_320+RLcp33_321)+ROcp33_36*(RLcp33_120+
 RLcp33_121);
    JTcp33_369_7 = ROcp33_16*(RLcp33_220+RLcp33_221)-ROcp33_26*(RLcp33_120+RLcp33_121)-RLcp33_169*ROcp33_26+RLcp33_269*
 ROcp33_16;
    JTcp33_169_8 = ROcp33_519*(RLcp33_321+RLcp33_369)-ROcp33_619*(RLcp33_221+RLcp33_269);
    JTcp33_269_8 = -(ROcp33_419*(RLcp33_321+RLcp33_369)-ROcp33_619*(RLcp33_121+RLcp33_169));
    JTcp33_369_8 = ROcp33_419*(RLcp33_221+RLcp33_269)-ROcp33_519*(RLcp33_121+RLcp33_169);
    JTcp33_169_9 = -(RLcp33_269*ROcp33_920-RLcp33_369*ROcp33_820);
    JTcp33_269_9 = RLcp33_169*ROcp33_920-RLcp33_369*ROcp33_720;
    JTcp33_369_9 = -(RLcp33_169*ROcp33_820-RLcp33_269*ROcp33_720);
    ORcp33_169 = OMcp33_221*RLcp33_369-OMcp33_321*RLcp33_269;
    ORcp33_269 = -(OMcp33_121*RLcp33_369-OMcp33_321*RLcp33_169);
    ORcp33_369 = OMcp33_121*RLcp33_269-OMcp33_221*RLcp33_169;
    VIcp33_169 = ORcp33_119+ORcp33_120+ORcp33_121+ORcp33_169+qd[1];
    VIcp33_269 = ORcp33_219+ORcp33_220+ORcp33_221+ORcp33_269+qd[2];
    VIcp33_369 = ORcp33_319+ORcp33_320+ORcp33_321+ORcp33_369+qd[3];
    ACcp33_169 = qdd[1]+OMcp33_219*ORcp33_320+OMcp33_220*ORcp33_321+OMcp33_221*ORcp33_369+OMcp33_26*ORcp33_319-OMcp33_319*
 ORcp33_220-OMcp33_320*ORcp33_221-OMcp33_321*ORcp33_269-OMcp33_36*ORcp33_219+OPcp33_219*RLcp33_320+OPcp33_220*RLcp33_321+
 OPcp33_221*RLcp33_369+OPcp33_26*RLcp33_319-OPcp33_319*RLcp33_220-OPcp33_320*RLcp33_221-OPcp33_321*RLcp33_269-OPcp33_36*
 RLcp33_219;
    ACcp33_269 = qdd[2]-OMcp33_119*ORcp33_320-OMcp33_120*ORcp33_321-OMcp33_121*ORcp33_369-OMcp33_16*ORcp33_319+OMcp33_319*
 ORcp33_120+OMcp33_320*ORcp33_121+OMcp33_321*ORcp33_169+OMcp33_36*ORcp33_119-OPcp33_119*RLcp33_320-OPcp33_120*RLcp33_321-
 OPcp33_121*RLcp33_369-OPcp33_16*RLcp33_319+OPcp33_319*RLcp33_120+OPcp33_320*RLcp33_121+OPcp33_321*RLcp33_169+OPcp33_36*
 RLcp33_119;
    ACcp33_369 = qdd[3]+OMcp33_119*ORcp33_220+OMcp33_120*ORcp33_221+OMcp33_121*ORcp33_269+OMcp33_16*ORcp33_219-OMcp33_219*
 ORcp33_120-OMcp33_220*ORcp33_121-OMcp33_221*ORcp33_169-OMcp33_26*ORcp33_119+OPcp33_119*RLcp33_220+OPcp33_120*RLcp33_221+
 OPcp33_121*RLcp33_269+OPcp33_16*RLcp33_219-OPcp33_219*RLcp33_120-OPcp33_220*RLcp33_121-OPcp33_221*RLcp33_169-OPcp33_26*
 RLcp33_119;

// = = Block_1_0_0_34_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp33_169;
    sens->P[2] = POcp33_269;
    sens->P[3] = POcp33_369;
    sens->R[1][1] = ROcp33_121;
    sens->R[1][2] = ROcp33_221;
    sens->R[1][3] = ROcp33_321;
    sens->R[2][1] = ROcp33_421;
    sens->R[2][2] = ROcp33_521;
    sens->R[2][3] = ROcp33_621;
    sens->R[3][1] = ROcp33_720;
    sens->R[3][2] = ROcp33_820;
    sens->R[3][3] = ROcp33_920;
    sens->V[1] = VIcp33_169;
    sens->V[2] = VIcp33_269;
    sens->V[3] = VIcp33_369;
    sens->OM[1] = OMcp33_121;
    sens->OM[2] = OMcp33_221;
    sens->OM[3] = OMcp33_321;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp33_169_5;
    sens->J[1][6] = JTcp33_169_6;
    sens->J[1][19] = JTcp33_169_7;
    sens->J[1][20] = JTcp33_169_8;
    sens->J[1][21] = JTcp33_169_9;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp33_269_4;
    sens->J[2][5] = JTcp33_269_5;
    sens->J[2][6] = JTcp33_269_6;
    sens->J[2][19] = JTcp33_269_7;
    sens->J[2][20] = JTcp33_269_8;
    sens->J[2][21] = JTcp33_269_9;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp33_369_4;
    sens->J[3][5] = JTcp33_369_5;
    sens->J[3][6] = JTcp33_369_6;
    sens->J[3][19] = JTcp33_369_7;
    sens->J[3][20] = JTcp33_369_8;
    sens->J[3][21] = JTcp33_369_9;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp33_16;
    sens->J[4][20] = ROcp33_419;
    sens->J[4][21] = ROcp33_720;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp33_85;
    sens->J[5][19] = ROcp33_26;
    sens->J[5][20] = ROcp33_519;
    sens->J[5][21] = ROcp33_820;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp33_95;
    sens->J[6][19] = ROcp33_36;
    sens->J[6][20] = ROcp33_619;
    sens->J[6][21] = ROcp33_920;
    sens->A[1] = ACcp33_169;
    sens->A[2] = ACcp33_269;
    sens->A[3] = ACcp33_369;
    sens->OMP[1] = OPcp33_121;
    sens->OMP[2] = OPcp33_221;
    sens->OMP[3] = OPcp33_321;
 
// 
break;
case 35:
 


// = = Block_1_0_0_35_0_1 = = 
 
// Sensor Kinematics 


    ROcp34_25 = S4*S5;
    ROcp34_35 = -C4*S5;
    ROcp34_85 = -S4*C5;
    ROcp34_95 = C4*C5;
    ROcp34_16 = C5*C6;
    ROcp34_26 = ROcp34_25*C6+C4*S6;
    ROcp34_36 = ROcp34_35*C6+S4*S6;
    ROcp34_46 = -C5*S6;
    ROcp34_56 = -(ROcp34_25*S6-C4*C6);
    ROcp34_66 = -(ROcp34_35*S6-S4*C6);
    OMcp34_25 = qd[5]*C4;
    OMcp34_35 = qd[5]*S4;
    OMcp34_16 = qd[4]+qd[6]*S5;
    OMcp34_26 = OMcp34_25+ROcp34_85*qd[6];
    OMcp34_36 = OMcp34_35+ROcp34_95*qd[6];
    OPcp34_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp34_26 = ROcp34_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp34_35*S5-ROcp34_95*qd[4]);
    OPcp34_36 = ROcp34_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp34_25*S5-ROcp34_85*qd[4]);

// = = Block_1_0_0_35_0_4 = = 
 
// Sensor Kinematics 


    ROcp34_419 = ROcp34_46*C19+S19*S5;
    ROcp34_519 = ROcp34_56*C19+ROcp34_85*S19;
    ROcp34_619 = ROcp34_66*C19+ROcp34_95*S19;
    ROcp34_719 = -(ROcp34_46*S19-C19*S5);
    ROcp34_819 = -(ROcp34_56*S19-ROcp34_85*C19);
    ROcp34_919 = -(ROcp34_66*S19-ROcp34_95*C19);
    ROcp34_120 = ROcp34_16*C20-ROcp34_719*S20;
    ROcp34_220 = ROcp34_26*C20-ROcp34_819*S20;
    ROcp34_320 = ROcp34_36*C20-ROcp34_919*S20;
    ROcp34_720 = ROcp34_16*S20+ROcp34_719*C20;
    ROcp34_820 = ROcp34_26*S20+ROcp34_819*C20;
    ROcp34_920 = ROcp34_36*S20+ROcp34_919*C20;
    ROcp34_121 = ROcp34_120*C21+ROcp34_419*S21;
    ROcp34_221 = ROcp34_220*C21+ROcp34_519*S21;
    ROcp34_321 = ROcp34_320*C21+ROcp34_619*S21;
    ROcp34_421 = -(ROcp34_120*S21-ROcp34_419*C21);
    ROcp34_521 = -(ROcp34_220*S21-ROcp34_519*C21);
    ROcp34_621 = -(ROcp34_320*S21-ROcp34_619*C21);
    RLcp34_119 = s->dpt[2][3]*ROcp34_46+ROcp34_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp34_219 = s->dpt[2][3]*ROcp34_56+ROcp34_26*s->dpt[1][3]+ROcp34_85*s->dpt[3][3];
    RLcp34_319 = s->dpt[2][3]*ROcp34_66+ROcp34_36*s->dpt[1][3]+ROcp34_95*s->dpt[3][3];
    OMcp34_119 = OMcp34_16+ROcp34_16*qd[19];
    OMcp34_219 = OMcp34_26+ROcp34_26*qd[19];
    OMcp34_319 = OMcp34_36+ROcp34_36*qd[19];
    ORcp34_119 = OMcp34_26*RLcp34_319-OMcp34_36*RLcp34_219;
    ORcp34_219 = -(OMcp34_16*RLcp34_319-OMcp34_36*RLcp34_119);
    ORcp34_319 = OMcp34_16*RLcp34_219-OMcp34_26*RLcp34_119;
    OPcp34_119 = OPcp34_16+ROcp34_16*qdd[19]+qd[19]*(OMcp34_26*ROcp34_36-OMcp34_36*ROcp34_26);
    OPcp34_219 = OPcp34_26+ROcp34_26*qdd[19]-qd[19]*(OMcp34_16*ROcp34_36-OMcp34_36*ROcp34_16);
    OPcp34_319 = OPcp34_36+ROcp34_36*qdd[19]+qd[19]*(OMcp34_16*ROcp34_26-OMcp34_26*ROcp34_16);
    RLcp34_120 = s->dpt[1][38]*ROcp34_16+s->dpt[2][38]*ROcp34_419+s->dpt[3][38]*ROcp34_719;
    RLcp34_220 = s->dpt[1][38]*ROcp34_26+s->dpt[2][38]*ROcp34_519+s->dpt[3][38]*ROcp34_819;
    RLcp34_320 = s->dpt[1][38]*ROcp34_36+s->dpt[2][38]*ROcp34_619+s->dpt[3][38]*ROcp34_919;
    OMcp34_120 = OMcp34_119+ROcp34_419*qd[20];
    OMcp34_220 = OMcp34_219+ROcp34_519*qd[20];
    OMcp34_320 = OMcp34_319+ROcp34_619*qd[20];
    ORcp34_120 = OMcp34_219*RLcp34_320-OMcp34_319*RLcp34_220;
    ORcp34_220 = -(OMcp34_119*RLcp34_320-OMcp34_319*RLcp34_120);
    ORcp34_320 = OMcp34_119*RLcp34_220-OMcp34_219*RLcp34_120;
    OPcp34_120 = OPcp34_119+ROcp34_419*qdd[20]+qd[20]*(OMcp34_219*ROcp34_619-OMcp34_319*ROcp34_519);
    OPcp34_220 = OPcp34_219+ROcp34_519*qdd[20]-qd[20]*(OMcp34_119*ROcp34_619-OMcp34_319*ROcp34_419);
    OPcp34_320 = OPcp34_319+ROcp34_619*qdd[20]+qd[20]*(OMcp34_119*ROcp34_519-OMcp34_219*ROcp34_419);
    RLcp34_121 = s->dpt[1][40]*ROcp34_120+s->dpt[2][40]*ROcp34_419+ROcp34_720*s->dpt[3][40];
    RLcp34_221 = s->dpt[1][40]*ROcp34_220+s->dpt[2][40]*ROcp34_519+ROcp34_820*s->dpt[3][40];
    RLcp34_321 = s->dpt[1][40]*ROcp34_320+s->dpt[2][40]*ROcp34_619+ROcp34_920*s->dpt[3][40];
    OMcp34_121 = OMcp34_120+ROcp34_720*qd[21];
    OMcp34_221 = OMcp34_220+ROcp34_820*qd[21];
    OMcp34_321 = OMcp34_320+ROcp34_920*qd[21];
    ORcp34_121 = OMcp34_220*RLcp34_321-OMcp34_320*RLcp34_221;
    ORcp34_221 = -(OMcp34_120*RLcp34_321-OMcp34_320*RLcp34_121);
    ORcp34_321 = OMcp34_120*RLcp34_221-OMcp34_220*RLcp34_121;
    OPcp34_121 = OPcp34_120+ROcp34_720*qdd[21]+qd[21]*(OMcp34_220*ROcp34_920-OMcp34_320*ROcp34_820);
    OPcp34_221 = OPcp34_220+ROcp34_820*qdd[21]-qd[21]*(OMcp34_120*ROcp34_920-OMcp34_320*ROcp34_720);
    OPcp34_321 = OPcp34_320+ROcp34_920*qdd[21]+qd[21]*(OMcp34_120*ROcp34_820-OMcp34_220*ROcp34_720);
    RLcp34_170 = ROcp34_121*s->dpt[1][45]+ROcp34_421*s->dpt[2][45]+ROcp34_720*s->dpt[3][45];
    RLcp34_270 = ROcp34_221*s->dpt[1][45]+ROcp34_521*s->dpt[2][45]+ROcp34_820*s->dpt[3][45];
    RLcp34_370 = ROcp34_321*s->dpt[1][45]+ROcp34_621*s->dpt[2][45]+ROcp34_920*s->dpt[3][45];
    POcp34_170 = RLcp34_119+RLcp34_120+RLcp34_121+RLcp34_170+q[1];
    POcp34_270 = RLcp34_219+RLcp34_220+RLcp34_221+RLcp34_270+q[2];
    POcp34_370 = RLcp34_319+RLcp34_320+RLcp34_321+RLcp34_370+q[3];
    JTcp34_270_4 = -(RLcp34_319+RLcp34_320+RLcp34_321+RLcp34_370);
    JTcp34_370_4 = RLcp34_219+RLcp34_220+RLcp34_221+RLcp34_270;
    JTcp34_170_5 = C4*(RLcp34_319+RLcp34_320+RLcp34_321+RLcp34_370)-S4*(RLcp34_219+RLcp34_220)-S4*(RLcp34_221+RLcp34_270);
    JTcp34_270_5 = S4*(RLcp34_119+RLcp34_120+RLcp34_121+RLcp34_170);
    JTcp34_370_5 = -C4*(RLcp34_119+RLcp34_120+RLcp34_121+RLcp34_170);
    JTcp34_170_6 = ROcp34_85*(RLcp34_319+RLcp34_320+RLcp34_321+RLcp34_370)-ROcp34_95*(RLcp34_219+RLcp34_220)-ROcp34_95*(
 RLcp34_221+RLcp34_270);
    JTcp34_270_6 = RLcp34_170*ROcp34_95-RLcp34_321*S5-RLcp34_370*S5+ROcp34_95*(RLcp34_119+RLcp34_120+RLcp34_121)-S5*(
 RLcp34_319+RLcp34_320);
    JTcp34_370_6 = RLcp34_221*S5-ROcp34_85*(RLcp34_119+RLcp34_120+RLcp34_121)+S5*(RLcp34_219+RLcp34_220)-RLcp34_170*
 ROcp34_85+RLcp34_270*S5;
    JTcp34_170_7 = ROcp34_26*(RLcp34_320+RLcp34_321)-ROcp34_36*(RLcp34_220+RLcp34_221)-RLcp34_270*ROcp34_36+RLcp34_370*
 ROcp34_26;
    JTcp34_270_7 = RLcp34_170*ROcp34_36-RLcp34_370*ROcp34_16-ROcp34_16*(RLcp34_320+RLcp34_321)+ROcp34_36*(RLcp34_120+
 RLcp34_121);
    JTcp34_370_7 = ROcp34_16*(RLcp34_220+RLcp34_221)-ROcp34_26*(RLcp34_120+RLcp34_121)-RLcp34_170*ROcp34_26+RLcp34_270*
 ROcp34_16;
    JTcp34_170_8 = ROcp34_519*(RLcp34_321+RLcp34_370)-ROcp34_619*(RLcp34_221+RLcp34_270);
    JTcp34_270_8 = -(ROcp34_419*(RLcp34_321+RLcp34_370)-ROcp34_619*(RLcp34_121+RLcp34_170));
    JTcp34_370_8 = ROcp34_419*(RLcp34_221+RLcp34_270)-ROcp34_519*(RLcp34_121+RLcp34_170);
    JTcp34_170_9 = -(RLcp34_270*ROcp34_920-RLcp34_370*ROcp34_820);
    JTcp34_270_9 = RLcp34_170*ROcp34_920-RLcp34_370*ROcp34_720;
    JTcp34_370_9 = -(RLcp34_170*ROcp34_820-RLcp34_270*ROcp34_720);
    ORcp34_170 = OMcp34_221*RLcp34_370-OMcp34_321*RLcp34_270;
    ORcp34_270 = -(OMcp34_121*RLcp34_370-OMcp34_321*RLcp34_170);
    ORcp34_370 = OMcp34_121*RLcp34_270-OMcp34_221*RLcp34_170;
    VIcp34_170 = ORcp34_119+ORcp34_120+ORcp34_121+ORcp34_170+qd[1];
    VIcp34_270 = ORcp34_219+ORcp34_220+ORcp34_221+ORcp34_270+qd[2];
    VIcp34_370 = ORcp34_319+ORcp34_320+ORcp34_321+ORcp34_370+qd[3];
    ACcp34_170 = qdd[1]+OMcp34_219*ORcp34_320+OMcp34_220*ORcp34_321+OMcp34_221*ORcp34_370+OMcp34_26*ORcp34_319-OMcp34_319*
 ORcp34_220-OMcp34_320*ORcp34_221-OMcp34_321*ORcp34_270-OMcp34_36*ORcp34_219+OPcp34_219*RLcp34_320+OPcp34_220*RLcp34_321+
 OPcp34_221*RLcp34_370+OPcp34_26*RLcp34_319-OPcp34_319*RLcp34_220-OPcp34_320*RLcp34_221-OPcp34_321*RLcp34_270-OPcp34_36*
 RLcp34_219;
    ACcp34_270 = qdd[2]-OMcp34_119*ORcp34_320-OMcp34_120*ORcp34_321-OMcp34_121*ORcp34_370-OMcp34_16*ORcp34_319+OMcp34_319*
 ORcp34_120+OMcp34_320*ORcp34_121+OMcp34_321*ORcp34_170+OMcp34_36*ORcp34_119-OPcp34_119*RLcp34_320-OPcp34_120*RLcp34_321-
 OPcp34_121*RLcp34_370-OPcp34_16*RLcp34_319+OPcp34_319*RLcp34_120+OPcp34_320*RLcp34_121+OPcp34_321*RLcp34_170+OPcp34_36*
 RLcp34_119;
    ACcp34_370 = qdd[3]+OMcp34_119*ORcp34_220+OMcp34_120*ORcp34_221+OMcp34_121*ORcp34_270+OMcp34_16*ORcp34_219-OMcp34_219*
 ORcp34_120-OMcp34_220*ORcp34_121-OMcp34_221*ORcp34_170-OMcp34_26*ORcp34_119+OPcp34_119*RLcp34_220+OPcp34_120*RLcp34_221+
 OPcp34_121*RLcp34_270+OPcp34_16*RLcp34_219-OPcp34_219*RLcp34_120-OPcp34_220*RLcp34_121-OPcp34_221*RLcp34_170-OPcp34_26*
 RLcp34_119;

// = = Block_1_0_0_35_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp34_170;
    sens->P[2] = POcp34_270;
    sens->P[3] = POcp34_370;
    sens->R[1][1] = ROcp34_121;
    sens->R[1][2] = ROcp34_221;
    sens->R[1][3] = ROcp34_321;
    sens->R[2][1] = ROcp34_421;
    sens->R[2][2] = ROcp34_521;
    sens->R[2][3] = ROcp34_621;
    sens->R[3][1] = ROcp34_720;
    sens->R[3][2] = ROcp34_820;
    sens->R[3][3] = ROcp34_920;
    sens->V[1] = VIcp34_170;
    sens->V[2] = VIcp34_270;
    sens->V[3] = VIcp34_370;
    sens->OM[1] = OMcp34_121;
    sens->OM[2] = OMcp34_221;
    sens->OM[3] = OMcp34_321;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp34_170_5;
    sens->J[1][6] = JTcp34_170_6;
    sens->J[1][19] = JTcp34_170_7;
    sens->J[1][20] = JTcp34_170_8;
    sens->J[1][21] = JTcp34_170_9;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp34_270_4;
    sens->J[2][5] = JTcp34_270_5;
    sens->J[2][6] = JTcp34_270_6;
    sens->J[2][19] = JTcp34_270_7;
    sens->J[2][20] = JTcp34_270_8;
    sens->J[2][21] = JTcp34_270_9;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp34_370_4;
    sens->J[3][5] = JTcp34_370_5;
    sens->J[3][6] = JTcp34_370_6;
    sens->J[3][19] = JTcp34_370_7;
    sens->J[3][20] = JTcp34_370_8;
    sens->J[3][21] = JTcp34_370_9;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp34_16;
    sens->J[4][20] = ROcp34_419;
    sens->J[4][21] = ROcp34_720;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp34_85;
    sens->J[5][19] = ROcp34_26;
    sens->J[5][20] = ROcp34_519;
    sens->J[5][21] = ROcp34_820;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp34_95;
    sens->J[6][19] = ROcp34_36;
    sens->J[6][20] = ROcp34_619;
    sens->J[6][21] = ROcp34_920;
    sens->A[1] = ACcp34_170;
    sens->A[2] = ACcp34_270;
    sens->A[3] = ACcp34_370;
    sens->OMP[1] = OPcp34_121;
    sens->OMP[2] = OPcp34_221;
    sens->OMP[3] = OPcp34_321;
 
// 
break;
case 36:
 


// = = Block_1_0_0_36_0_1 = = 
 
// Sensor Kinematics 


    ROcp35_25 = S4*S5;
    ROcp35_35 = -C4*S5;
    ROcp35_85 = -S4*C5;
    ROcp35_95 = C4*C5;
    ROcp35_16 = C5*C6;
    ROcp35_26 = ROcp35_25*C6+C4*S6;
    ROcp35_36 = ROcp35_35*C6+S4*S6;
    ROcp35_46 = -C5*S6;
    ROcp35_56 = -(ROcp35_25*S6-C4*C6);
    ROcp35_66 = -(ROcp35_35*S6-S4*C6);
    OMcp35_25 = qd[5]*C4;
    OMcp35_35 = qd[5]*S4;
    OMcp35_16 = qd[4]+qd[6]*S5;
    OMcp35_26 = OMcp35_25+ROcp35_85*qd[6];
    OMcp35_36 = OMcp35_35+ROcp35_95*qd[6];
    OPcp35_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp35_26 = ROcp35_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp35_35*S5-ROcp35_95*qd[4]);
    OPcp35_36 = ROcp35_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp35_25*S5-ROcp35_85*qd[4]);

// = = Block_1_0_0_36_0_4 = = 
 
// Sensor Kinematics 


    ROcp35_419 = ROcp35_46*C19+S19*S5;
    ROcp35_519 = ROcp35_56*C19+ROcp35_85*S19;
    ROcp35_619 = ROcp35_66*C19+ROcp35_95*S19;
    ROcp35_719 = -(ROcp35_46*S19-C19*S5);
    ROcp35_819 = -(ROcp35_56*S19-ROcp35_85*C19);
    ROcp35_919 = -(ROcp35_66*S19-ROcp35_95*C19);
    ROcp35_120 = ROcp35_16*C20-ROcp35_719*S20;
    ROcp35_220 = ROcp35_26*C20-ROcp35_819*S20;
    ROcp35_320 = ROcp35_36*C20-ROcp35_919*S20;
    ROcp35_720 = ROcp35_16*S20+ROcp35_719*C20;
    ROcp35_820 = ROcp35_26*S20+ROcp35_819*C20;
    ROcp35_920 = ROcp35_36*S20+ROcp35_919*C20;
    ROcp35_121 = ROcp35_120*C21+ROcp35_419*S21;
    ROcp35_221 = ROcp35_220*C21+ROcp35_519*S21;
    ROcp35_321 = ROcp35_320*C21+ROcp35_619*S21;
    ROcp35_421 = -(ROcp35_120*S21-ROcp35_419*C21);
    ROcp35_521 = -(ROcp35_220*S21-ROcp35_519*C21);
    ROcp35_621 = -(ROcp35_320*S21-ROcp35_619*C21);
    RLcp35_119 = s->dpt[2][3]*ROcp35_46+ROcp35_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp35_219 = s->dpt[2][3]*ROcp35_56+ROcp35_26*s->dpt[1][3]+ROcp35_85*s->dpt[3][3];
    RLcp35_319 = s->dpt[2][3]*ROcp35_66+ROcp35_36*s->dpt[1][3]+ROcp35_95*s->dpt[3][3];
    OMcp35_119 = OMcp35_16+ROcp35_16*qd[19];
    OMcp35_219 = OMcp35_26+ROcp35_26*qd[19];
    OMcp35_319 = OMcp35_36+ROcp35_36*qd[19];
    ORcp35_119 = OMcp35_26*RLcp35_319-OMcp35_36*RLcp35_219;
    ORcp35_219 = -(OMcp35_16*RLcp35_319-OMcp35_36*RLcp35_119);
    ORcp35_319 = OMcp35_16*RLcp35_219-OMcp35_26*RLcp35_119;
    OPcp35_119 = OPcp35_16+ROcp35_16*qdd[19]+qd[19]*(OMcp35_26*ROcp35_36-OMcp35_36*ROcp35_26);
    OPcp35_219 = OPcp35_26+ROcp35_26*qdd[19]-qd[19]*(OMcp35_16*ROcp35_36-OMcp35_36*ROcp35_16);
    OPcp35_319 = OPcp35_36+ROcp35_36*qdd[19]+qd[19]*(OMcp35_16*ROcp35_26-OMcp35_26*ROcp35_16);
    RLcp35_120 = s->dpt[1][38]*ROcp35_16+s->dpt[2][38]*ROcp35_419+s->dpt[3][38]*ROcp35_719;
    RLcp35_220 = s->dpt[1][38]*ROcp35_26+s->dpt[2][38]*ROcp35_519+s->dpt[3][38]*ROcp35_819;
    RLcp35_320 = s->dpt[1][38]*ROcp35_36+s->dpt[2][38]*ROcp35_619+s->dpt[3][38]*ROcp35_919;
    OMcp35_120 = OMcp35_119+ROcp35_419*qd[20];
    OMcp35_220 = OMcp35_219+ROcp35_519*qd[20];
    OMcp35_320 = OMcp35_319+ROcp35_619*qd[20];
    ORcp35_120 = OMcp35_219*RLcp35_320-OMcp35_319*RLcp35_220;
    ORcp35_220 = -(OMcp35_119*RLcp35_320-OMcp35_319*RLcp35_120);
    ORcp35_320 = OMcp35_119*RLcp35_220-OMcp35_219*RLcp35_120;
    OPcp35_120 = OPcp35_119+ROcp35_419*qdd[20]+qd[20]*(OMcp35_219*ROcp35_619-OMcp35_319*ROcp35_519);
    OPcp35_220 = OPcp35_219+ROcp35_519*qdd[20]-qd[20]*(OMcp35_119*ROcp35_619-OMcp35_319*ROcp35_419);
    OPcp35_320 = OPcp35_319+ROcp35_619*qdd[20]+qd[20]*(OMcp35_119*ROcp35_519-OMcp35_219*ROcp35_419);
    RLcp35_121 = s->dpt[1][40]*ROcp35_120+s->dpt[2][40]*ROcp35_419+ROcp35_720*s->dpt[3][40];
    RLcp35_221 = s->dpt[1][40]*ROcp35_220+s->dpt[2][40]*ROcp35_519+ROcp35_820*s->dpt[3][40];
    RLcp35_321 = s->dpt[1][40]*ROcp35_320+s->dpt[2][40]*ROcp35_619+ROcp35_920*s->dpt[3][40];
    OMcp35_121 = OMcp35_120+ROcp35_720*qd[21];
    OMcp35_221 = OMcp35_220+ROcp35_820*qd[21];
    OMcp35_321 = OMcp35_320+ROcp35_920*qd[21];
    ORcp35_121 = OMcp35_220*RLcp35_321-OMcp35_320*RLcp35_221;
    ORcp35_221 = -(OMcp35_120*RLcp35_321-OMcp35_320*RLcp35_121);
    ORcp35_321 = OMcp35_120*RLcp35_221-OMcp35_220*RLcp35_121;
    OPcp35_121 = OPcp35_120+ROcp35_720*qdd[21]+qd[21]*(OMcp35_220*ROcp35_920-OMcp35_320*ROcp35_820);
    OPcp35_221 = OPcp35_220+ROcp35_820*qdd[21]-qd[21]*(OMcp35_120*ROcp35_920-OMcp35_320*ROcp35_720);
    OPcp35_321 = OPcp35_320+ROcp35_920*qdd[21]+qd[21]*(OMcp35_120*ROcp35_820-OMcp35_220*ROcp35_720);

// = = Block_1_0_0_36_0_5 = = 
 
// Sensor Kinematics 


    ROcp35_122 = ROcp35_121*C22-ROcp35_720*S22;
    ROcp35_222 = ROcp35_221*C22-ROcp35_820*S22;
    ROcp35_322 = ROcp35_321*C22-ROcp35_920*S22;
    ROcp35_722 = ROcp35_121*S22+ROcp35_720*C22;
    ROcp35_822 = ROcp35_221*S22+ROcp35_820*C22;
    ROcp35_922 = ROcp35_321*S22+ROcp35_920*C22;
    RLcp35_122 = ROcp35_121*s->dpt[1][44]+ROcp35_421*s->dpt[2][44]+ROcp35_720*s->dpt[3][44];
    RLcp35_222 = ROcp35_221*s->dpt[1][44]+ROcp35_521*s->dpt[2][44]+ROcp35_820*s->dpt[3][44];
    RLcp35_322 = ROcp35_321*s->dpt[1][44]+ROcp35_621*s->dpt[2][44]+ROcp35_920*s->dpt[3][44];
    OMcp35_122 = OMcp35_121+ROcp35_421*qd[22];
    OMcp35_222 = OMcp35_221+ROcp35_521*qd[22];
    OMcp35_322 = OMcp35_321+ROcp35_621*qd[22];
    ORcp35_122 = OMcp35_221*RLcp35_322-OMcp35_321*RLcp35_222;
    ORcp35_222 = -(OMcp35_121*RLcp35_322-OMcp35_321*RLcp35_122);
    ORcp35_322 = OMcp35_121*RLcp35_222-OMcp35_221*RLcp35_122;
    OPcp35_122 = OPcp35_121+ROcp35_421*qdd[22]+qd[22]*(OMcp35_221*ROcp35_621-OMcp35_321*ROcp35_521);
    OPcp35_222 = OPcp35_221+ROcp35_521*qdd[22]-qd[22]*(OMcp35_121*ROcp35_621-OMcp35_321*ROcp35_421);
    OPcp35_322 = OPcp35_321+ROcp35_621*qdd[22]+qd[22]*(OMcp35_121*ROcp35_521-OMcp35_221*ROcp35_421);
    RLcp35_171 = ROcp35_122*s->dpt[1][47]+ROcp35_421*s->dpt[2][47]+ROcp35_722*s->dpt[3][47];
    RLcp35_271 = ROcp35_222*s->dpt[1][47]+ROcp35_521*s->dpt[2][47]+ROcp35_822*s->dpt[3][47];
    RLcp35_371 = ROcp35_322*s->dpt[1][47]+ROcp35_621*s->dpt[2][47]+ROcp35_922*s->dpt[3][47];
    POcp35_171 = RLcp35_119+RLcp35_120+RLcp35_121+RLcp35_122+RLcp35_171+q[1];
    POcp35_271 = RLcp35_219+RLcp35_220+RLcp35_221+RLcp35_222+RLcp35_271+q[2];
    POcp35_371 = RLcp35_319+RLcp35_320+RLcp35_321+RLcp35_322+RLcp35_371+q[3];
    JTcp35_271_4 = -(RLcp35_319+RLcp35_320+RLcp35_321+RLcp35_322+RLcp35_371);
    JTcp35_371_4 = RLcp35_219+RLcp35_220+RLcp35_221+RLcp35_222+RLcp35_271;
    JTcp35_171_5 = C4*(RLcp35_319+RLcp35_320+RLcp35_321+RLcp35_322)-S4*(RLcp35_219+RLcp35_220)-S4*(RLcp35_221+RLcp35_222)-
 RLcp35_271*S4+RLcp35_371*C4;
    JTcp35_271_5 = S4*(RLcp35_119+RLcp35_120+RLcp35_121+RLcp35_122+RLcp35_171);
    JTcp35_371_5 = -C4*(RLcp35_119+RLcp35_120+RLcp35_121+RLcp35_122+RLcp35_171);
    JTcp35_171_6 = ROcp35_85*(RLcp35_319+RLcp35_320+RLcp35_321+RLcp35_322)-ROcp35_95*(RLcp35_219+RLcp35_220)-ROcp35_95*(
 RLcp35_221+RLcp35_222)-RLcp35_271*ROcp35_95+RLcp35_371*ROcp35_85;
    JTcp35_271_6 = -(RLcp35_371*S5-ROcp35_95*(RLcp35_119+RLcp35_120+RLcp35_121+RLcp35_122+RLcp35_171)+S5*(RLcp35_319+
 RLcp35_320)+S5*(RLcp35_321+RLcp35_322));
    JTcp35_371_6 = RLcp35_271*S5-ROcp35_85*(RLcp35_119+RLcp35_120+RLcp35_121+RLcp35_122+RLcp35_171)+S5*(RLcp35_219+
 RLcp35_220)+S5*(RLcp35_221+RLcp35_222);
    JTcp35_171_7 = ROcp35_26*(RLcp35_320+RLcp35_321+RLcp35_322+RLcp35_371)-ROcp35_36*(RLcp35_220+RLcp35_221)-ROcp35_36*(
 RLcp35_222+RLcp35_271);
    JTcp35_271_7 = -(ROcp35_16*(RLcp35_320+RLcp35_321+RLcp35_322+RLcp35_371)-ROcp35_36*(RLcp35_120+RLcp35_121)-ROcp35_36*(
 RLcp35_122+RLcp35_171));
    JTcp35_371_7 = ROcp35_16*(RLcp35_220+RLcp35_221+RLcp35_222+RLcp35_271)-ROcp35_26*(RLcp35_120+RLcp35_121)-ROcp35_26*(
 RLcp35_122+RLcp35_171);
    JTcp35_171_8 = ROcp35_519*(RLcp35_321+RLcp35_322)-ROcp35_619*(RLcp35_221+RLcp35_222)-RLcp35_271*ROcp35_619+RLcp35_371*
 ROcp35_519;
    JTcp35_271_8 = RLcp35_171*ROcp35_619-RLcp35_371*ROcp35_419-ROcp35_419*(RLcp35_321+RLcp35_322)+ROcp35_619*(RLcp35_121+
 RLcp35_122);
    JTcp35_371_8 = ROcp35_419*(RLcp35_221+RLcp35_222)-ROcp35_519*(RLcp35_121+RLcp35_122)-RLcp35_171*ROcp35_519+RLcp35_271*
 ROcp35_419;
    JTcp35_171_9 = ROcp35_820*(RLcp35_322+RLcp35_371)-ROcp35_920*(RLcp35_222+RLcp35_271);
    JTcp35_271_9 = -(ROcp35_720*(RLcp35_322+RLcp35_371)-ROcp35_920*(RLcp35_122+RLcp35_171));
    JTcp35_371_9 = ROcp35_720*(RLcp35_222+RLcp35_271)-ROcp35_820*(RLcp35_122+RLcp35_171);
    JTcp35_171_10 = -(RLcp35_271*ROcp35_621-RLcp35_371*ROcp35_521);
    JTcp35_271_10 = RLcp35_171*ROcp35_621-RLcp35_371*ROcp35_421;
    JTcp35_371_10 = -(RLcp35_171*ROcp35_521-RLcp35_271*ROcp35_421);
    ORcp35_171 = OMcp35_222*RLcp35_371-OMcp35_322*RLcp35_271;
    ORcp35_271 = -(OMcp35_122*RLcp35_371-OMcp35_322*RLcp35_171);
    ORcp35_371 = OMcp35_122*RLcp35_271-OMcp35_222*RLcp35_171;
    VIcp35_171 = ORcp35_119+ORcp35_120+ORcp35_121+ORcp35_122+ORcp35_171+qd[1];
    VIcp35_271 = ORcp35_219+ORcp35_220+ORcp35_221+ORcp35_222+ORcp35_271+qd[2];
    VIcp35_371 = ORcp35_319+ORcp35_320+ORcp35_321+ORcp35_322+ORcp35_371+qd[3];
    ACcp35_171 = qdd[1]+OMcp35_219*ORcp35_320+OMcp35_220*ORcp35_321+OMcp35_221*ORcp35_322+OMcp35_222*ORcp35_371+OMcp35_26*
 ORcp35_319-OMcp35_319*ORcp35_220-OMcp35_320*ORcp35_221-OMcp35_321*ORcp35_222-OMcp35_322*ORcp35_271-OMcp35_36*ORcp35_219+
 OPcp35_219*RLcp35_320+OPcp35_220*RLcp35_321+OPcp35_221*RLcp35_322+OPcp35_222*RLcp35_371+OPcp35_26*RLcp35_319-OPcp35_319*
 RLcp35_220-OPcp35_320*RLcp35_221-OPcp35_321*RLcp35_222-OPcp35_322*RLcp35_271-OPcp35_36*RLcp35_219;
    ACcp35_271 = qdd[2]-OMcp35_119*ORcp35_320-OMcp35_120*ORcp35_321-OMcp35_121*ORcp35_322-OMcp35_122*ORcp35_371-OMcp35_16*
 ORcp35_319+OMcp35_319*ORcp35_120+OMcp35_320*ORcp35_121+OMcp35_321*ORcp35_122+OMcp35_322*ORcp35_171+OMcp35_36*ORcp35_119-
 OPcp35_119*RLcp35_320-OPcp35_120*RLcp35_321-OPcp35_121*RLcp35_322-OPcp35_122*RLcp35_371-OPcp35_16*RLcp35_319+OPcp35_319*
 RLcp35_120+OPcp35_320*RLcp35_121+OPcp35_321*RLcp35_122+OPcp35_322*RLcp35_171+OPcp35_36*RLcp35_119;
    ACcp35_371 = qdd[3]+OMcp35_119*ORcp35_220+OMcp35_120*ORcp35_221+OMcp35_121*ORcp35_222+OMcp35_122*ORcp35_271+OMcp35_16*
 ORcp35_219-OMcp35_219*ORcp35_120-OMcp35_220*ORcp35_121-OMcp35_221*ORcp35_122-OMcp35_222*ORcp35_171-OMcp35_26*ORcp35_119+
 OPcp35_119*RLcp35_220+OPcp35_120*RLcp35_221+OPcp35_121*RLcp35_222+OPcp35_122*RLcp35_271+OPcp35_16*RLcp35_219-OPcp35_219*
 RLcp35_120-OPcp35_220*RLcp35_121-OPcp35_221*RLcp35_122-OPcp35_222*RLcp35_171-OPcp35_26*RLcp35_119;

// = = Block_1_0_0_36_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp35_171;
    sens->P[2] = POcp35_271;
    sens->P[3] = POcp35_371;
    sens->R[1][1] = ROcp35_122;
    sens->R[1][2] = ROcp35_222;
    sens->R[1][3] = ROcp35_322;
    sens->R[2][1] = ROcp35_421;
    sens->R[2][2] = ROcp35_521;
    sens->R[2][3] = ROcp35_621;
    sens->R[3][1] = ROcp35_722;
    sens->R[3][2] = ROcp35_822;
    sens->R[3][3] = ROcp35_922;
    sens->V[1] = VIcp35_171;
    sens->V[2] = VIcp35_271;
    sens->V[3] = VIcp35_371;
    sens->OM[1] = OMcp35_122;
    sens->OM[2] = OMcp35_222;
    sens->OM[3] = OMcp35_322;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp35_171_5;
    sens->J[1][6] = JTcp35_171_6;
    sens->J[1][19] = JTcp35_171_7;
    sens->J[1][20] = JTcp35_171_8;
    sens->J[1][21] = JTcp35_171_9;
    sens->J[1][22] = JTcp35_171_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp35_271_4;
    sens->J[2][5] = JTcp35_271_5;
    sens->J[2][6] = JTcp35_271_6;
    sens->J[2][19] = JTcp35_271_7;
    sens->J[2][20] = JTcp35_271_8;
    sens->J[2][21] = JTcp35_271_9;
    sens->J[2][22] = JTcp35_271_10;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp35_371_4;
    sens->J[3][5] = JTcp35_371_5;
    sens->J[3][6] = JTcp35_371_6;
    sens->J[3][19] = JTcp35_371_7;
    sens->J[3][20] = JTcp35_371_8;
    sens->J[3][21] = JTcp35_371_9;
    sens->J[3][22] = JTcp35_371_10;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp35_16;
    sens->J[4][20] = ROcp35_419;
    sens->J[4][21] = ROcp35_720;
    sens->J[4][22] = ROcp35_421;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp35_85;
    sens->J[5][19] = ROcp35_26;
    sens->J[5][20] = ROcp35_519;
    sens->J[5][21] = ROcp35_820;
    sens->J[5][22] = ROcp35_521;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp35_95;
    sens->J[6][19] = ROcp35_36;
    sens->J[6][20] = ROcp35_619;
    sens->J[6][21] = ROcp35_920;
    sens->J[6][22] = ROcp35_621;
    sens->A[1] = ACcp35_171;
    sens->A[2] = ACcp35_271;
    sens->A[3] = ACcp35_371;
    sens->OMP[1] = OPcp35_122;
    sens->OMP[2] = OPcp35_222;
    sens->OMP[3] = OPcp35_322;
 
// 
break;
case 37:
 


// = = Block_1_0_0_37_0_1 = = 
 
// Sensor Kinematics 


    ROcp36_25 = S4*S5;
    ROcp36_35 = -C4*S5;
    ROcp36_85 = -S4*C5;
    ROcp36_95 = C4*C5;
    ROcp36_16 = C5*C6;
    ROcp36_26 = ROcp36_25*C6+C4*S6;
    ROcp36_36 = ROcp36_35*C6+S4*S6;
    ROcp36_46 = -C5*S6;
    ROcp36_56 = -(ROcp36_25*S6-C4*C6);
    ROcp36_66 = -(ROcp36_35*S6-S4*C6);
    OMcp36_25 = qd[5]*C4;
    OMcp36_35 = qd[5]*S4;
    OMcp36_16 = qd[4]+qd[6]*S5;
    OMcp36_26 = OMcp36_25+ROcp36_85*qd[6];
    OMcp36_36 = OMcp36_35+ROcp36_95*qd[6];
    OPcp36_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp36_26 = ROcp36_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp36_35*S5-ROcp36_95*qd[4]);
    OPcp36_36 = ROcp36_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp36_25*S5-ROcp36_85*qd[4]);

// = = Block_1_0_0_37_0_4 = = 
 
// Sensor Kinematics 


    ROcp36_419 = ROcp36_46*C19+S19*S5;
    ROcp36_519 = ROcp36_56*C19+ROcp36_85*S19;
    ROcp36_619 = ROcp36_66*C19+ROcp36_95*S19;
    ROcp36_719 = -(ROcp36_46*S19-C19*S5);
    ROcp36_819 = -(ROcp36_56*S19-ROcp36_85*C19);
    ROcp36_919 = -(ROcp36_66*S19-ROcp36_95*C19);
    ROcp36_120 = ROcp36_16*C20-ROcp36_719*S20;
    ROcp36_220 = ROcp36_26*C20-ROcp36_819*S20;
    ROcp36_320 = ROcp36_36*C20-ROcp36_919*S20;
    ROcp36_720 = ROcp36_16*S20+ROcp36_719*C20;
    ROcp36_820 = ROcp36_26*S20+ROcp36_819*C20;
    ROcp36_920 = ROcp36_36*S20+ROcp36_919*C20;
    ROcp36_121 = ROcp36_120*C21+ROcp36_419*S21;
    ROcp36_221 = ROcp36_220*C21+ROcp36_519*S21;
    ROcp36_321 = ROcp36_320*C21+ROcp36_619*S21;
    ROcp36_421 = -(ROcp36_120*S21-ROcp36_419*C21);
    ROcp36_521 = -(ROcp36_220*S21-ROcp36_519*C21);
    ROcp36_621 = -(ROcp36_320*S21-ROcp36_619*C21);
    RLcp36_119 = s->dpt[2][3]*ROcp36_46+ROcp36_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp36_219 = s->dpt[2][3]*ROcp36_56+ROcp36_26*s->dpt[1][3]+ROcp36_85*s->dpt[3][3];
    RLcp36_319 = s->dpt[2][3]*ROcp36_66+ROcp36_36*s->dpt[1][3]+ROcp36_95*s->dpt[3][3];
    OMcp36_119 = OMcp36_16+ROcp36_16*qd[19];
    OMcp36_219 = OMcp36_26+ROcp36_26*qd[19];
    OMcp36_319 = OMcp36_36+ROcp36_36*qd[19];
    ORcp36_119 = OMcp36_26*RLcp36_319-OMcp36_36*RLcp36_219;
    ORcp36_219 = -(OMcp36_16*RLcp36_319-OMcp36_36*RLcp36_119);
    ORcp36_319 = OMcp36_16*RLcp36_219-OMcp36_26*RLcp36_119;
    OPcp36_119 = OPcp36_16+ROcp36_16*qdd[19]+qd[19]*(OMcp36_26*ROcp36_36-OMcp36_36*ROcp36_26);
    OPcp36_219 = OPcp36_26+ROcp36_26*qdd[19]-qd[19]*(OMcp36_16*ROcp36_36-OMcp36_36*ROcp36_16);
    OPcp36_319 = OPcp36_36+ROcp36_36*qdd[19]+qd[19]*(OMcp36_16*ROcp36_26-OMcp36_26*ROcp36_16);
    RLcp36_120 = s->dpt[1][38]*ROcp36_16+s->dpt[2][38]*ROcp36_419+s->dpt[3][38]*ROcp36_719;
    RLcp36_220 = s->dpt[1][38]*ROcp36_26+s->dpt[2][38]*ROcp36_519+s->dpt[3][38]*ROcp36_819;
    RLcp36_320 = s->dpt[1][38]*ROcp36_36+s->dpt[2][38]*ROcp36_619+s->dpt[3][38]*ROcp36_919;
    OMcp36_120 = OMcp36_119+ROcp36_419*qd[20];
    OMcp36_220 = OMcp36_219+ROcp36_519*qd[20];
    OMcp36_320 = OMcp36_319+ROcp36_619*qd[20];
    ORcp36_120 = OMcp36_219*RLcp36_320-OMcp36_319*RLcp36_220;
    ORcp36_220 = -(OMcp36_119*RLcp36_320-OMcp36_319*RLcp36_120);
    ORcp36_320 = OMcp36_119*RLcp36_220-OMcp36_219*RLcp36_120;
    OPcp36_120 = OPcp36_119+ROcp36_419*qdd[20]+qd[20]*(OMcp36_219*ROcp36_619-OMcp36_319*ROcp36_519);
    OPcp36_220 = OPcp36_219+ROcp36_519*qdd[20]-qd[20]*(OMcp36_119*ROcp36_619-OMcp36_319*ROcp36_419);
    OPcp36_320 = OPcp36_319+ROcp36_619*qdd[20]+qd[20]*(OMcp36_119*ROcp36_519-OMcp36_219*ROcp36_419);
    RLcp36_121 = s->dpt[1][40]*ROcp36_120+s->dpt[2][40]*ROcp36_419+ROcp36_720*s->dpt[3][40];
    RLcp36_221 = s->dpt[1][40]*ROcp36_220+s->dpt[2][40]*ROcp36_519+ROcp36_820*s->dpt[3][40];
    RLcp36_321 = s->dpt[1][40]*ROcp36_320+s->dpt[2][40]*ROcp36_619+ROcp36_920*s->dpt[3][40];
    OMcp36_121 = OMcp36_120+ROcp36_720*qd[21];
    OMcp36_221 = OMcp36_220+ROcp36_820*qd[21];
    OMcp36_321 = OMcp36_320+ROcp36_920*qd[21];
    ORcp36_121 = OMcp36_220*RLcp36_321-OMcp36_320*RLcp36_221;
    ORcp36_221 = -(OMcp36_120*RLcp36_321-OMcp36_320*RLcp36_121);
    ORcp36_321 = OMcp36_120*RLcp36_221-OMcp36_220*RLcp36_121;
    OPcp36_121 = OPcp36_120+ROcp36_720*qdd[21]+qd[21]*(OMcp36_220*ROcp36_920-OMcp36_320*ROcp36_820);
    OPcp36_221 = OPcp36_220+ROcp36_820*qdd[21]-qd[21]*(OMcp36_120*ROcp36_920-OMcp36_320*ROcp36_720);
    OPcp36_321 = OPcp36_320+ROcp36_920*qdd[21]+qd[21]*(OMcp36_120*ROcp36_820-OMcp36_220*ROcp36_720);

// = = Block_1_0_0_37_0_5 = = 
 
// Sensor Kinematics 


    ROcp36_122 = ROcp36_121*C22-ROcp36_720*S22;
    ROcp36_222 = ROcp36_221*C22-ROcp36_820*S22;
    ROcp36_322 = ROcp36_321*C22-ROcp36_920*S22;
    ROcp36_722 = ROcp36_121*S22+ROcp36_720*C22;
    ROcp36_822 = ROcp36_221*S22+ROcp36_820*C22;
    ROcp36_922 = ROcp36_321*S22+ROcp36_920*C22;
    RLcp36_122 = ROcp36_121*s->dpt[1][44]+ROcp36_421*s->dpt[2][44]+ROcp36_720*s->dpt[3][44];
    RLcp36_222 = ROcp36_221*s->dpt[1][44]+ROcp36_521*s->dpt[2][44]+ROcp36_820*s->dpt[3][44];
    RLcp36_322 = ROcp36_321*s->dpt[1][44]+ROcp36_621*s->dpt[2][44]+ROcp36_920*s->dpt[3][44];
    OMcp36_122 = OMcp36_121+ROcp36_421*qd[22];
    OMcp36_222 = OMcp36_221+ROcp36_521*qd[22];
    OMcp36_322 = OMcp36_321+ROcp36_621*qd[22];
    ORcp36_122 = OMcp36_221*RLcp36_322-OMcp36_321*RLcp36_222;
    ORcp36_222 = -(OMcp36_121*RLcp36_322-OMcp36_321*RLcp36_122);
    ORcp36_322 = OMcp36_121*RLcp36_222-OMcp36_221*RLcp36_122;
    OPcp36_122 = OPcp36_121+ROcp36_421*qdd[22]+qd[22]*(OMcp36_221*ROcp36_621-OMcp36_321*ROcp36_521);
    OPcp36_222 = OPcp36_221+ROcp36_521*qdd[22]-qd[22]*(OMcp36_121*ROcp36_621-OMcp36_321*ROcp36_421);
    OPcp36_322 = OPcp36_321+ROcp36_621*qdd[22]+qd[22]*(OMcp36_121*ROcp36_521-OMcp36_221*ROcp36_421);
    RLcp36_172 = s->dpt[1][48]*ROcp36_122+s->dpt[3][48]*ROcp36_722+ROcp36_421*s->dpt[2][48];
    RLcp36_272 = s->dpt[1][48]*ROcp36_222+s->dpt[3][48]*ROcp36_822+ROcp36_521*s->dpt[2][48];
    RLcp36_372 = s->dpt[1][48]*ROcp36_322+s->dpt[3][48]*ROcp36_922+ROcp36_621*s->dpt[2][48];
    POcp36_172 = RLcp36_119+RLcp36_120+RLcp36_121+RLcp36_122+RLcp36_172+q[1];
    POcp36_272 = RLcp36_219+RLcp36_220+RLcp36_221+RLcp36_222+RLcp36_272+q[2];
    POcp36_372 = RLcp36_319+RLcp36_320+RLcp36_321+RLcp36_322+RLcp36_372+q[3];
    JTcp36_272_4 = -(RLcp36_319+RLcp36_320+RLcp36_321+RLcp36_322+RLcp36_372);
    JTcp36_372_4 = RLcp36_219+RLcp36_220+RLcp36_221+RLcp36_222+RLcp36_272;
    JTcp36_172_5 = C4*(RLcp36_319+RLcp36_320+RLcp36_321+RLcp36_322)-S4*(RLcp36_219+RLcp36_220)-S4*(RLcp36_221+RLcp36_222)-
 RLcp36_272*S4+RLcp36_372*C4;
    JTcp36_272_5 = S4*(RLcp36_119+RLcp36_120+RLcp36_121+RLcp36_122+RLcp36_172);
    JTcp36_372_5 = -C4*(RLcp36_119+RLcp36_120+RLcp36_121+RLcp36_122+RLcp36_172);
    JTcp36_172_6 = ROcp36_85*(RLcp36_319+RLcp36_320+RLcp36_321+RLcp36_322)-ROcp36_95*(RLcp36_219+RLcp36_220)-ROcp36_95*(
 RLcp36_221+RLcp36_222)-RLcp36_272*ROcp36_95+RLcp36_372*ROcp36_85;
    JTcp36_272_6 = -(RLcp36_372*S5-ROcp36_95*(RLcp36_119+RLcp36_120+RLcp36_121+RLcp36_122+RLcp36_172)+S5*(RLcp36_319+
 RLcp36_320)+S5*(RLcp36_321+RLcp36_322));
    JTcp36_372_6 = RLcp36_272*S5-ROcp36_85*(RLcp36_119+RLcp36_120+RLcp36_121+RLcp36_122+RLcp36_172)+S5*(RLcp36_219+
 RLcp36_220)+S5*(RLcp36_221+RLcp36_222);
    JTcp36_172_7 = ROcp36_26*(RLcp36_320+RLcp36_321+RLcp36_322+RLcp36_372)-ROcp36_36*(RLcp36_220+RLcp36_221)-ROcp36_36*(
 RLcp36_222+RLcp36_272);
    JTcp36_272_7 = -(ROcp36_16*(RLcp36_320+RLcp36_321+RLcp36_322+RLcp36_372)-ROcp36_36*(RLcp36_120+RLcp36_121)-ROcp36_36*(
 RLcp36_122+RLcp36_172));
    JTcp36_372_7 = ROcp36_16*(RLcp36_220+RLcp36_221+RLcp36_222+RLcp36_272)-ROcp36_26*(RLcp36_120+RLcp36_121)-ROcp36_26*(
 RLcp36_122+RLcp36_172);
    JTcp36_172_8 = ROcp36_519*(RLcp36_321+RLcp36_322)-ROcp36_619*(RLcp36_221+RLcp36_222)-RLcp36_272*ROcp36_619+RLcp36_372*
 ROcp36_519;
    JTcp36_272_8 = RLcp36_172*ROcp36_619-RLcp36_372*ROcp36_419-ROcp36_419*(RLcp36_321+RLcp36_322)+ROcp36_619*(RLcp36_121+
 RLcp36_122);
    JTcp36_372_8 = ROcp36_419*(RLcp36_221+RLcp36_222)-ROcp36_519*(RLcp36_121+RLcp36_122)-RLcp36_172*ROcp36_519+RLcp36_272*
 ROcp36_419;
    JTcp36_172_9 = ROcp36_820*(RLcp36_322+RLcp36_372)-ROcp36_920*(RLcp36_222+RLcp36_272);
    JTcp36_272_9 = -(ROcp36_720*(RLcp36_322+RLcp36_372)-ROcp36_920*(RLcp36_122+RLcp36_172));
    JTcp36_372_9 = ROcp36_720*(RLcp36_222+RLcp36_272)-ROcp36_820*(RLcp36_122+RLcp36_172);
    JTcp36_172_10 = -(RLcp36_272*ROcp36_621-RLcp36_372*ROcp36_521);
    JTcp36_272_10 = RLcp36_172*ROcp36_621-RLcp36_372*ROcp36_421;
    JTcp36_372_10 = -(RLcp36_172*ROcp36_521-RLcp36_272*ROcp36_421);
    ORcp36_172 = OMcp36_222*RLcp36_372-OMcp36_322*RLcp36_272;
    ORcp36_272 = -(OMcp36_122*RLcp36_372-OMcp36_322*RLcp36_172);
    ORcp36_372 = OMcp36_122*RLcp36_272-OMcp36_222*RLcp36_172;
    VIcp36_172 = ORcp36_119+ORcp36_120+ORcp36_121+ORcp36_122+ORcp36_172+qd[1];
    VIcp36_272 = ORcp36_219+ORcp36_220+ORcp36_221+ORcp36_222+ORcp36_272+qd[2];
    VIcp36_372 = ORcp36_319+ORcp36_320+ORcp36_321+ORcp36_322+ORcp36_372+qd[3];
    ACcp36_172 = qdd[1]+OMcp36_219*ORcp36_320+OMcp36_220*ORcp36_321+OMcp36_221*ORcp36_322+OMcp36_222*ORcp36_372+OMcp36_26*
 ORcp36_319-OMcp36_319*ORcp36_220-OMcp36_320*ORcp36_221-OMcp36_321*ORcp36_222-OMcp36_322*ORcp36_272-OMcp36_36*ORcp36_219+
 OPcp36_219*RLcp36_320+OPcp36_220*RLcp36_321+OPcp36_221*RLcp36_322+OPcp36_222*RLcp36_372+OPcp36_26*RLcp36_319-OPcp36_319*
 RLcp36_220-OPcp36_320*RLcp36_221-OPcp36_321*RLcp36_222-OPcp36_322*RLcp36_272-OPcp36_36*RLcp36_219;
    ACcp36_272 = qdd[2]-OMcp36_119*ORcp36_320-OMcp36_120*ORcp36_321-OMcp36_121*ORcp36_322-OMcp36_122*ORcp36_372-OMcp36_16*
 ORcp36_319+OMcp36_319*ORcp36_120+OMcp36_320*ORcp36_121+OMcp36_321*ORcp36_122+OMcp36_322*ORcp36_172+OMcp36_36*ORcp36_119-
 OPcp36_119*RLcp36_320-OPcp36_120*RLcp36_321-OPcp36_121*RLcp36_322-OPcp36_122*RLcp36_372-OPcp36_16*RLcp36_319+OPcp36_319*
 RLcp36_120+OPcp36_320*RLcp36_121+OPcp36_321*RLcp36_122+OPcp36_322*RLcp36_172+OPcp36_36*RLcp36_119;
    ACcp36_372 = qdd[3]+OMcp36_119*ORcp36_220+OMcp36_120*ORcp36_221+OMcp36_121*ORcp36_222+OMcp36_122*ORcp36_272+OMcp36_16*
 ORcp36_219-OMcp36_219*ORcp36_120-OMcp36_220*ORcp36_121-OMcp36_221*ORcp36_122-OMcp36_222*ORcp36_172-OMcp36_26*ORcp36_119+
 OPcp36_119*RLcp36_220+OPcp36_120*RLcp36_221+OPcp36_121*RLcp36_222+OPcp36_122*RLcp36_272+OPcp36_16*RLcp36_219-OPcp36_219*
 RLcp36_120-OPcp36_220*RLcp36_121-OPcp36_221*RLcp36_122-OPcp36_222*RLcp36_172-OPcp36_26*RLcp36_119;

// = = Block_1_0_0_37_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp36_172;
    sens->P[2] = POcp36_272;
    sens->P[3] = POcp36_372;
    sens->R[1][1] = ROcp36_122;
    sens->R[1][2] = ROcp36_222;
    sens->R[1][3] = ROcp36_322;
    sens->R[2][1] = ROcp36_421;
    sens->R[2][2] = ROcp36_521;
    sens->R[2][3] = ROcp36_621;
    sens->R[3][1] = ROcp36_722;
    sens->R[3][2] = ROcp36_822;
    sens->R[3][3] = ROcp36_922;
    sens->V[1] = VIcp36_172;
    sens->V[2] = VIcp36_272;
    sens->V[3] = VIcp36_372;
    sens->OM[1] = OMcp36_122;
    sens->OM[2] = OMcp36_222;
    sens->OM[3] = OMcp36_322;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp36_172_5;
    sens->J[1][6] = JTcp36_172_6;
    sens->J[1][19] = JTcp36_172_7;
    sens->J[1][20] = JTcp36_172_8;
    sens->J[1][21] = JTcp36_172_9;
    sens->J[1][22] = JTcp36_172_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp36_272_4;
    sens->J[2][5] = JTcp36_272_5;
    sens->J[2][6] = JTcp36_272_6;
    sens->J[2][19] = JTcp36_272_7;
    sens->J[2][20] = JTcp36_272_8;
    sens->J[2][21] = JTcp36_272_9;
    sens->J[2][22] = JTcp36_272_10;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp36_372_4;
    sens->J[3][5] = JTcp36_372_5;
    sens->J[3][6] = JTcp36_372_6;
    sens->J[3][19] = JTcp36_372_7;
    sens->J[3][20] = JTcp36_372_8;
    sens->J[3][21] = JTcp36_372_9;
    sens->J[3][22] = JTcp36_372_10;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp36_16;
    sens->J[4][20] = ROcp36_419;
    sens->J[4][21] = ROcp36_720;
    sens->J[4][22] = ROcp36_421;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp36_85;
    sens->J[5][19] = ROcp36_26;
    sens->J[5][20] = ROcp36_519;
    sens->J[5][21] = ROcp36_820;
    sens->J[5][22] = ROcp36_521;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp36_95;
    sens->J[6][19] = ROcp36_36;
    sens->J[6][20] = ROcp36_619;
    sens->J[6][21] = ROcp36_920;
    sens->J[6][22] = ROcp36_621;
    sens->A[1] = ACcp36_172;
    sens->A[2] = ACcp36_272;
    sens->A[3] = ACcp36_372;
    sens->OMP[1] = OPcp36_122;
    sens->OMP[2] = OPcp36_222;
    sens->OMP[3] = OPcp36_322;
 
// 
break;
case 38:
 


// = = Block_1_0_0_38_0_1 = = 
 
// Sensor Kinematics 


    ROcp37_25 = S4*S5;
    ROcp37_35 = -C4*S5;
    ROcp37_85 = -S4*C5;
    ROcp37_95 = C4*C5;
    ROcp37_16 = C5*C6;
    ROcp37_26 = ROcp37_25*C6+C4*S6;
    ROcp37_36 = ROcp37_35*C6+S4*S6;
    ROcp37_46 = -C5*S6;
    ROcp37_56 = -(ROcp37_25*S6-C4*C6);
    ROcp37_66 = -(ROcp37_35*S6-S4*C6);
    OMcp37_25 = qd[5]*C4;
    OMcp37_35 = qd[5]*S4;
    OMcp37_16 = qd[4]+qd[6]*S5;
    OMcp37_26 = OMcp37_25+ROcp37_85*qd[6];
    OMcp37_36 = OMcp37_35+ROcp37_95*qd[6];
    OPcp37_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp37_26 = ROcp37_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp37_35*S5-ROcp37_95*qd[4]);
    OPcp37_36 = ROcp37_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp37_25*S5-ROcp37_85*qd[4]);

// = = Block_1_0_0_38_0_4 = = 
 
// Sensor Kinematics 


    ROcp37_419 = ROcp37_46*C19+S19*S5;
    ROcp37_519 = ROcp37_56*C19+ROcp37_85*S19;
    ROcp37_619 = ROcp37_66*C19+ROcp37_95*S19;
    ROcp37_719 = -(ROcp37_46*S19-C19*S5);
    ROcp37_819 = -(ROcp37_56*S19-ROcp37_85*C19);
    ROcp37_919 = -(ROcp37_66*S19-ROcp37_95*C19);
    ROcp37_120 = ROcp37_16*C20-ROcp37_719*S20;
    ROcp37_220 = ROcp37_26*C20-ROcp37_819*S20;
    ROcp37_320 = ROcp37_36*C20-ROcp37_919*S20;
    ROcp37_720 = ROcp37_16*S20+ROcp37_719*C20;
    ROcp37_820 = ROcp37_26*S20+ROcp37_819*C20;
    ROcp37_920 = ROcp37_36*S20+ROcp37_919*C20;
    ROcp37_121 = ROcp37_120*C21+ROcp37_419*S21;
    ROcp37_221 = ROcp37_220*C21+ROcp37_519*S21;
    ROcp37_321 = ROcp37_320*C21+ROcp37_619*S21;
    ROcp37_421 = -(ROcp37_120*S21-ROcp37_419*C21);
    ROcp37_521 = -(ROcp37_220*S21-ROcp37_519*C21);
    ROcp37_621 = -(ROcp37_320*S21-ROcp37_619*C21);
    RLcp37_119 = s->dpt[2][3]*ROcp37_46+ROcp37_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp37_219 = s->dpt[2][3]*ROcp37_56+ROcp37_26*s->dpt[1][3]+ROcp37_85*s->dpt[3][3];
    RLcp37_319 = s->dpt[2][3]*ROcp37_66+ROcp37_36*s->dpt[1][3]+ROcp37_95*s->dpt[3][3];
    OMcp37_119 = OMcp37_16+ROcp37_16*qd[19];
    OMcp37_219 = OMcp37_26+ROcp37_26*qd[19];
    OMcp37_319 = OMcp37_36+ROcp37_36*qd[19];
    ORcp37_119 = OMcp37_26*RLcp37_319-OMcp37_36*RLcp37_219;
    ORcp37_219 = -(OMcp37_16*RLcp37_319-OMcp37_36*RLcp37_119);
    ORcp37_319 = OMcp37_16*RLcp37_219-OMcp37_26*RLcp37_119;
    OPcp37_119 = OPcp37_16+ROcp37_16*qdd[19]+qd[19]*(OMcp37_26*ROcp37_36-OMcp37_36*ROcp37_26);
    OPcp37_219 = OPcp37_26+ROcp37_26*qdd[19]-qd[19]*(OMcp37_16*ROcp37_36-OMcp37_36*ROcp37_16);
    OPcp37_319 = OPcp37_36+ROcp37_36*qdd[19]+qd[19]*(OMcp37_16*ROcp37_26-OMcp37_26*ROcp37_16);
    RLcp37_120 = s->dpt[1][38]*ROcp37_16+s->dpt[2][38]*ROcp37_419+s->dpt[3][38]*ROcp37_719;
    RLcp37_220 = s->dpt[1][38]*ROcp37_26+s->dpt[2][38]*ROcp37_519+s->dpt[3][38]*ROcp37_819;
    RLcp37_320 = s->dpt[1][38]*ROcp37_36+s->dpt[2][38]*ROcp37_619+s->dpt[3][38]*ROcp37_919;
    OMcp37_120 = OMcp37_119+ROcp37_419*qd[20];
    OMcp37_220 = OMcp37_219+ROcp37_519*qd[20];
    OMcp37_320 = OMcp37_319+ROcp37_619*qd[20];
    ORcp37_120 = OMcp37_219*RLcp37_320-OMcp37_319*RLcp37_220;
    ORcp37_220 = -(OMcp37_119*RLcp37_320-OMcp37_319*RLcp37_120);
    ORcp37_320 = OMcp37_119*RLcp37_220-OMcp37_219*RLcp37_120;
    OPcp37_120 = OPcp37_119+ROcp37_419*qdd[20]+qd[20]*(OMcp37_219*ROcp37_619-OMcp37_319*ROcp37_519);
    OPcp37_220 = OPcp37_219+ROcp37_519*qdd[20]-qd[20]*(OMcp37_119*ROcp37_619-OMcp37_319*ROcp37_419);
    OPcp37_320 = OPcp37_319+ROcp37_619*qdd[20]+qd[20]*(OMcp37_119*ROcp37_519-OMcp37_219*ROcp37_419);
    RLcp37_121 = s->dpt[1][40]*ROcp37_120+s->dpt[2][40]*ROcp37_419+ROcp37_720*s->dpt[3][40];
    RLcp37_221 = s->dpt[1][40]*ROcp37_220+s->dpt[2][40]*ROcp37_519+ROcp37_820*s->dpt[3][40];
    RLcp37_321 = s->dpt[1][40]*ROcp37_320+s->dpt[2][40]*ROcp37_619+ROcp37_920*s->dpt[3][40];
    OMcp37_121 = OMcp37_120+ROcp37_720*qd[21];
    OMcp37_221 = OMcp37_220+ROcp37_820*qd[21];
    OMcp37_321 = OMcp37_320+ROcp37_920*qd[21];
    ORcp37_121 = OMcp37_220*RLcp37_321-OMcp37_320*RLcp37_221;
    ORcp37_221 = -(OMcp37_120*RLcp37_321-OMcp37_320*RLcp37_121);
    ORcp37_321 = OMcp37_120*RLcp37_221-OMcp37_220*RLcp37_121;
    OPcp37_121 = OPcp37_120+ROcp37_720*qdd[21]+qd[21]*(OMcp37_220*ROcp37_920-OMcp37_320*ROcp37_820);
    OPcp37_221 = OPcp37_220+ROcp37_820*qdd[21]-qd[21]*(OMcp37_120*ROcp37_920-OMcp37_320*ROcp37_720);
    OPcp37_321 = OPcp37_320+ROcp37_920*qdd[21]+qd[21]*(OMcp37_120*ROcp37_820-OMcp37_220*ROcp37_720);

// = = Block_1_0_0_38_0_5 = = 
 
// Sensor Kinematics 


    ROcp37_122 = ROcp37_121*C22-ROcp37_720*S22;
    ROcp37_222 = ROcp37_221*C22-ROcp37_820*S22;
    ROcp37_322 = ROcp37_321*C22-ROcp37_920*S22;
    ROcp37_722 = ROcp37_121*S22+ROcp37_720*C22;
    ROcp37_822 = ROcp37_221*S22+ROcp37_820*C22;
    ROcp37_922 = ROcp37_321*S22+ROcp37_920*C22;
    ROcp37_423 = ROcp37_421*C23+ROcp37_722*S23;
    ROcp37_523 = ROcp37_521*C23+ROcp37_822*S23;
    ROcp37_623 = ROcp37_621*C23+ROcp37_922*S23;
    ROcp37_723 = -(ROcp37_421*S23-ROcp37_722*C23);
    ROcp37_823 = -(ROcp37_521*S23-ROcp37_822*C23);
    ROcp37_923 = -(ROcp37_621*S23-ROcp37_922*C23);
    RLcp37_122 = ROcp37_121*s->dpt[1][44]+ROcp37_421*s->dpt[2][44]+ROcp37_720*s->dpt[3][44];
    RLcp37_222 = ROcp37_221*s->dpt[1][44]+ROcp37_521*s->dpt[2][44]+ROcp37_820*s->dpt[3][44];
    RLcp37_322 = ROcp37_321*s->dpt[1][44]+ROcp37_621*s->dpt[2][44]+ROcp37_920*s->dpt[3][44];
    OMcp37_122 = OMcp37_121+ROcp37_421*qd[22];
    OMcp37_222 = OMcp37_221+ROcp37_521*qd[22];
    OMcp37_322 = OMcp37_321+ROcp37_621*qd[22];
    ORcp37_122 = OMcp37_221*RLcp37_322-OMcp37_321*RLcp37_222;
    ORcp37_222 = -(OMcp37_121*RLcp37_322-OMcp37_321*RLcp37_122);
    ORcp37_322 = OMcp37_121*RLcp37_222-OMcp37_221*RLcp37_122;
    OPcp37_122 = OPcp37_121+ROcp37_421*qdd[22]+qd[22]*(OMcp37_221*ROcp37_621-OMcp37_321*ROcp37_521);
    OPcp37_222 = OPcp37_221+ROcp37_521*qdd[22]-qd[22]*(OMcp37_121*ROcp37_621-OMcp37_321*ROcp37_421);
    OPcp37_322 = OPcp37_321+ROcp37_621*qdd[22]+qd[22]*(OMcp37_121*ROcp37_521-OMcp37_221*ROcp37_421);
    RLcp37_123 = s->dpt[1][48]*ROcp37_122+s->dpt[3][48]*ROcp37_722+ROcp37_421*s->dpt[2][48];
    RLcp37_223 = s->dpt[1][48]*ROcp37_222+s->dpt[3][48]*ROcp37_822+ROcp37_521*s->dpt[2][48];
    RLcp37_323 = s->dpt[1][48]*ROcp37_322+s->dpt[3][48]*ROcp37_922+ROcp37_621*s->dpt[2][48];
    OMcp37_123 = OMcp37_122+ROcp37_122*qd[23];
    OMcp37_223 = OMcp37_222+ROcp37_222*qd[23];
    OMcp37_323 = OMcp37_322+ROcp37_322*qd[23];
    ORcp37_123 = OMcp37_222*RLcp37_323-OMcp37_322*RLcp37_223;
    ORcp37_223 = -(OMcp37_122*RLcp37_323-OMcp37_322*RLcp37_123);
    ORcp37_323 = OMcp37_122*RLcp37_223-OMcp37_222*RLcp37_123;
    OPcp37_123 = OPcp37_122+ROcp37_122*qdd[23]+qd[23]*(OMcp37_222*ROcp37_322-OMcp37_322*ROcp37_222);
    OPcp37_223 = OPcp37_222+ROcp37_222*qdd[23]-qd[23]*(OMcp37_122*ROcp37_322-OMcp37_322*ROcp37_122);
    OPcp37_323 = OPcp37_322+ROcp37_322*qdd[23]+qd[23]*(OMcp37_122*ROcp37_222-OMcp37_222*ROcp37_122);
    RLcp37_173 = s->dpt[3][49]*ROcp37_723+ROcp37_122*s->dpt[1][49]+ROcp37_423*s->dpt[2][49];
    RLcp37_273 = s->dpt[3][49]*ROcp37_823+ROcp37_222*s->dpt[1][49]+ROcp37_523*s->dpt[2][49];
    RLcp37_373 = s->dpt[3][49]*ROcp37_923+ROcp37_322*s->dpt[1][49]+ROcp37_623*s->dpt[2][49];
    POcp37_173 = RLcp37_119+RLcp37_120+RLcp37_121+RLcp37_122+RLcp37_123+RLcp37_173+q[1];
    POcp37_273 = RLcp37_219+RLcp37_220+RLcp37_221+RLcp37_222+RLcp37_223+RLcp37_273+q[2];
    POcp37_373 = RLcp37_319+RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_373+q[3];
    JTcp37_273_4 = -(RLcp37_319+RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_373);
    JTcp37_373_4 = RLcp37_219+RLcp37_220+RLcp37_221+RLcp37_222+RLcp37_223+RLcp37_273;
    JTcp37_173_5 = C4*(RLcp37_319+RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_373)-S4*(RLcp37_219+RLcp37_220)-S4*(
 RLcp37_221+RLcp37_222)-S4*(RLcp37_223+RLcp37_273);
    JTcp37_273_5 = S4*(RLcp37_119+RLcp37_120+RLcp37_121+RLcp37_122+RLcp37_123+RLcp37_173);
    JTcp37_373_5 = -C4*(RLcp37_119+RLcp37_120+RLcp37_121+RLcp37_122+RLcp37_123+RLcp37_173);
    JTcp37_173_6 = ROcp37_85*(RLcp37_319+RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_373)-ROcp37_95*(RLcp37_219+
 RLcp37_220)-ROcp37_95*(RLcp37_221+RLcp37_222)-ROcp37_95*(RLcp37_223+RLcp37_273);
    JTcp37_273_6 = RLcp37_173*ROcp37_95-RLcp37_323*S5-RLcp37_373*S5+ROcp37_95*(RLcp37_119+RLcp37_120+RLcp37_121+RLcp37_122
 +RLcp37_123)-S5*(RLcp37_319+RLcp37_320)-S5*(RLcp37_321+RLcp37_322);
    JTcp37_373_6 = RLcp37_223*S5-ROcp37_85*(RLcp37_119+RLcp37_120+RLcp37_121+RLcp37_122+RLcp37_123)+S5*(RLcp37_219+
 RLcp37_220)+S5*(RLcp37_221+RLcp37_222)-RLcp37_173*ROcp37_85+RLcp37_273*S5;
    JTcp37_173_7 = ROcp37_26*(RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323)-ROcp37_36*(RLcp37_220+RLcp37_221)-ROcp37_36*(
 RLcp37_222+RLcp37_223)-RLcp37_273*ROcp37_36+RLcp37_373*ROcp37_26;
    JTcp37_273_7 = RLcp37_173*ROcp37_36-RLcp37_373*ROcp37_16-ROcp37_16*(RLcp37_320+RLcp37_321+RLcp37_322+RLcp37_323)+
 ROcp37_36*(RLcp37_120+RLcp37_121)+ROcp37_36*(RLcp37_122+RLcp37_123);
    JTcp37_373_7 = ROcp37_16*(RLcp37_220+RLcp37_221+RLcp37_222+RLcp37_223)-ROcp37_26*(RLcp37_120+RLcp37_121)-ROcp37_26*(
 RLcp37_122+RLcp37_123)-RLcp37_173*ROcp37_26+RLcp37_273*ROcp37_16;
    JTcp37_173_8 = ROcp37_519*(RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_373)-ROcp37_619*(RLcp37_221+RLcp37_222)-ROcp37_619*
 (RLcp37_223+RLcp37_273);
    JTcp37_273_8 = -(ROcp37_419*(RLcp37_321+RLcp37_322+RLcp37_323+RLcp37_373)-ROcp37_619*(RLcp37_121+RLcp37_122)-
 ROcp37_619*(RLcp37_123+RLcp37_173));
    JTcp37_373_8 = ROcp37_419*(RLcp37_221+RLcp37_222+RLcp37_223+RLcp37_273)-ROcp37_519*(RLcp37_121+RLcp37_122)-ROcp37_519*
 (RLcp37_123+RLcp37_173);
    JTcp37_173_9 = ROcp37_820*(RLcp37_322+RLcp37_323)-ROcp37_920*(RLcp37_222+RLcp37_223)-RLcp37_273*ROcp37_920+RLcp37_373*
 ROcp37_820;
    JTcp37_273_9 = RLcp37_173*ROcp37_920-RLcp37_373*ROcp37_720-ROcp37_720*(RLcp37_322+RLcp37_323)+ROcp37_920*(RLcp37_122+
 RLcp37_123);
    JTcp37_373_9 = ROcp37_720*(RLcp37_222+RLcp37_223)-ROcp37_820*(RLcp37_122+RLcp37_123)-RLcp37_173*ROcp37_820+RLcp37_273*
 ROcp37_720;
    JTcp37_173_10 = ROcp37_521*(RLcp37_323+RLcp37_373)-ROcp37_621*(RLcp37_223+RLcp37_273);
    JTcp37_273_10 = -(ROcp37_421*(RLcp37_323+RLcp37_373)-ROcp37_621*(RLcp37_123+RLcp37_173));
    JTcp37_373_10 = ROcp37_421*(RLcp37_223+RLcp37_273)-ROcp37_521*(RLcp37_123+RLcp37_173);
    JTcp37_173_11 = -(RLcp37_273*ROcp37_322-RLcp37_373*ROcp37_222);
    JTcp37_273_11 = RLcp37_173*ROcp37_322-RLcp37_373*ROcp37_122;
    JTcp37_373_11 = -(RLcp37_173*ROcp37_222-RLcp37_273*ROcp37_122);
    ORcp37_173 = OMcp37_223*RLcp37_373-OMcp37_323*RLcp37_273;
    ORcp37_273 = -(OMcp37_123*RLcp37_373-OMcp37_323*RLcp37_173);
    ORcp37_373 = OMcp37_123*RLcp37_273-OMcp37_223*RLcp37_173;
    VIcp37_173 = ORcp37_119+ORcp37_120+ORcp37_121+ORcp37_122+ORcp37_123+ORcp37_173+qd[1];
    VIcp37_273 = ORcp37_219+ORcp37_220+ORcp37_221+ORcp37_222+ORcp37_223+ORcp37_273+qd[2];
    VIcp37_373 = ORcp37_319+ORcp37_320+ORcp37_321+ORcp37_322+ORcp37_323+ORcp37_373+qd[3];
    ACcp37_173 = qdd[1]+OMcp37_219*ORcp37_320+OMcp37_220*ORcp37_321+OMcp37_221*ORcp37_322+OMcp37_222*ORcp37_323+OMcp37_223
 *ORcp37_373+OMcp37_26*ORcp37_319-OMcp37_319*ORcp37_220-OMcp37_320*ORcp37_221-OMcp37_321*ORcp37_222-OMcp37_322*ORcp37_223-
 OMcp37_323*ORcp37_273-OMcp37_36*ORcp37_219+OPcp37_219*RLcp37_320+OPcp37_220*RLcp37_321+OPcp37_221*RLcp37_322+OPcp37_222*
 RLcp37_323+OPcp37_223*RLcp37_373+OPcp37_26*RLcp37_319-OPcp37_319*RLcp37_220-OPcp37_320*RLcp37_221-OPcp37_321*RLcp37_222-
 OPcp37_322*RLcp37_223-OPcp37_323*RLcp37_273-OPcp37_36*RLcp37_219;
    ACcp37_273 = qdd[2]-OMcp37_119*ORcp37_320-OMcp37_120*ORcp37_321-OMcp37_121*ORcp37_322-OMcp37_122*ORcp37_323-OMcp37_123
 *ORcp37_373-OMcp37_16*ORcp37_319+OMcp37_319*ORcp37_120+OMcp37_320*ORcp37_121+OMcp37_321*ORcp37_122+OMcp37_322*ORcp37_123+
 OMcp37_323*ORcp37_173+OMcp37_36*ORcp37_119-OPcp37_119*RLcp37_320-OPcp37_120*RLcp37_321-OPcp37_121*RLcp37_322-OPcp37_122*
 RLcp37_323-OPcp37_123*RLcp37_373-OPcp37_16*RLcp37_319+OPcp37_319*RLcp37_120+OPcp37_320*RLcp37_121+OPcp37_321*RLcp37_122+
 OPcp37_322*RLcp37_123+OPcp37_323*RLcp37_173+OPcp37_36*RLcp37_119;
    ACcp37_373 = qdd[3]+OMcp37_119*ORcp37_220+OMcp37_120*ORcp37_221+OMcp37_121*ORcp37_222+OMcp37_122*ORcp37_223+OMcp37_123
 *ORcp37_273+OMcp37_16*ORcp37_219-OMcp37_219*ORcp37_120-OMcp37_220*ORcp37_121-OMcp37_221*ORcp37_122-OMcp37_222*ORcp37_123-
 OMcp37_223*ORcp37_173-OMcp37_26*ORcp37_119+OPcp37_119*RLcp37_220+OPcp37_120*RLcp37_221+OPcp37_121*RLcp37_222+OPcp37_122*
 RLcp37_223+OPcp37_123*RLcp37_273+OPcp37_16*RLcp37_219-OPcp37_219*RLcp37_120-OPcp37_220*RLcp37_121-OPcp37_221*RLcp37_122-
 OPcp37_222*RLcp37_123-OPcp37_223*RLcp37_173-OPcp37_26*RLcp37_119;

// = = Block_1_0_0_38_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp37_173;
    sens->P[2] = POcp37_273;
    sens->P[3] = POcp37_373;
    sens->R[1][1] = ROcp37_122;
    sens->R[1][2] = ROcp37_222;
    sens->R[1][3] = ROcp37_322;
    sens->R[2][1] = ROcp37_423;
    sens->R[2][2] = ROcp37_523;
    sens->R[2][3] = ROcp37_623;
    sens->R[3][1] = ROcp37_723;
    sens->R[3][2] = ROcp37_823;
    sens->R[3][3] = ROcp37_923;
    sens->V[1] = VIcp37_173;
    sens->V[2] = VIcp37_273;
    sens->V[3] = VIcp37_373;
    sens->OM[1] = OMcp37_123;
    sens->OM[2] = OMcp37_223;
    sens->OM[3] = OMcp37_323;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp37_173_5;
    sens->J[1][6] = JTcp37_173_6;
    sens->J[1][19] = JTcp37_173_7;
    sens->J[1][20] = JTcp37_173_8;
    sens->J[1][21] = JTcp37_173_9;
    sens->J[1][22] = JTcp37_173_10;
    sens->J[1][23] = JTcp37_173_11;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp37_273_4;
    sens->J[2][5] = JTcp37_273_5;
    sens->J[2][6] = JTcp37_273_6;
    sens->J[2][19] = JTcp37_273_7;
    sens->J[2][20] = JTcp37_273_8;
    sens->J[2][21] = JTcp37_273_9;
    sens->J[2][22] = JTcp37_273_10;
    sens->J[2][23] = JTcp37_273_11;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp37_373_4;
    sens->J[3][5] = JTcp37_373_5;
    sens->J[3][6] = JTcp37_373_6;
    sens->J[3][19] = JTcp37_373_7;
    sens->J[3][20] = JTcp37_373_8;
    sens->J[3][21] = JTcp37_373_9;
    sens->J[3][22] = JTcp37_373_10;
    sens->J[3][23] = JTcp37_373_11;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp37_16;
    sens->J[4][20] = ROcp37_419;
    sens->J[4][21] = ROcp37_720;
    sens->J[4][22] = ROcp37_421;
    sens->J[4][23] = ROcp37_122;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp37_85;
    sens->J[5][19] = ROcp37_26;
    sens->J[5][20] = ROcp37_519;
    sens->J[5][21] = ROcp37_820;
    sens->J[5][22] = ROcp37_521;
    sens->J[5][23] = ROcp37_222;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp37_95;
    sens->J[6][19] = ROcp37_36;
    sens->J[6][20] = ROcp37_619;
    sens->J[6][21] = ROcp37_920;
    sens->J[6][22] = ROcp37_621;
    sens->J[6][23] = ROcp37_322;
    sens->A[1] = ACcp37_173;
    sens->A[2] = ACcp37_273;
    sens->A[3] = ACcp37_373;
    sens->OMP[1] = OPcp37_123;
    sens->OMP[2] = OPcp37_223;
    sens->OMP[3] = OPcp37_323;
 
// 
break;
case 39:
 


// = = Block_1_0_0_39_0_1 = = 
 
// Sensor Kinematics 


    ROcp38_25 = S4*S5;
    ROcp38_35 = -C4*S5;
    ROcp38_85 = -S4*C5;
    ROcp38_95 = C4*C5;
    ROcp38_16 = C5*C6;
    ROcp38_26 = ROcp38_25*C6+C4*S6;
    ROcp38_36 = ROcp38_35*C6+S4*S6;
    ROcp38_46 = -C5*S6;
    ROcp38_56 = -(ROcp38_25*S6-C4*C6);
    ROcp38_66 = -(ROcp38_35*S6-S4*C6);
    OMcp38_25 = qd[5]*C4;
    OMcp38_35 = qd[5]*S4;
    OMcp38_16 = qd[4]+qd[6]*S5;
    OMcp38_26 = OMcp38_25+ROcp38_85*qd[6];
    OMcp38_36 = OMcp38_35+ROcp38_95*qd[6];
    OPcp38_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp38_26 = ROcp38_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp38_35*S5-ROcp38_95*qd[4]);
    OPcp38_36 = ROcp38_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp38_25*S5-ROcp38_85*qd[4]);

// = = Block_1_0_0_39_0_4 = = 
 
// Sensor Kinematics 


    ROcp38_419 = ROcp38_46*C19+S19*S5;
    ROcp38_519 = ROcp38_56*C19+ROcp38_85*S19;
    ROcp38_619 = ROcp38_66*C19+ROcp38_95*S19;
    ROcp38_719 = -(ROcp38_46*S19-C19*S5);
    ROcp38_819 = -(ROcp38_56*S19-ROcp38_85*C19);
    ROcp38_919 = -(ROcp38_66*S19-ROcp38_95*C19);
    ROcp38_120 = ROcp38_16*C20-ROcp38_719*S20;
    ROcp38_220 = ROcp38_26*C20-ROcp38_819*S20;
    ROcp38_320 = ROcp38_36*C20-ROcp38_919*S20;
    ROcp38_720 = ROcp38_16*S20+ROcp38_719*C20;
    ROcp38_820 = ROcp38_26*S20+ROcp38_819*C20;
    ROcp38_920 = ROcp38_36*S20+ROcp38_919*C20;
    ROcp38_121 = ROcp38_120*C21+ROcp38_419*S21;
    ROcp38_221 = ROcp38_220*C21+ROcp38_519*S21;
    ROcp38_321 = ROcp38_320*C21+ROcp38_619*S21;
    ROcp38_421 = -(ROcp38_120*S21-ROcp38_419*C21);
    ROcp38_521 = -(ROcp38_220*S21-ROcp38_519*C21);
    ROcp38_621 = -(ROcp38_320*S21-ROcp38_619*C21);
    RLcp38_119 = s->dpt[2][3]*ROcp38_46+ROcp38_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp38_219 = s->dpt[2][3]*ROcp38_56+ROcp38_26*s->dpt[1][3]+ROcp38_85*s->dpt[3][3];
    RLcp38_319 = s->dpt[2][3]*ROcp38_66+ROcp38_36*s->dpt[1][3]+ROcp38_95*s->dpt[3][3];
    OMcp38_119 = OMcp38_16+ROcp38_16*qd[19];
    OMcp38_219 = OMcp38_26+ROcp38_26*qd[19];
    OMcp38_319 = OMcp38_36+ROcp38_36*qd[19];
    ORcp38_119 = OMcp38_26*RLcp38_319-OMcp38_36*RLcp38_219;
    ORcp38_219 = -(OMcp38_16*RLcp38_319-OMcp38_36*RLcp38_119);
    ORcp38_319 = OMcp38_16*RLcp38_219-OMcp38_26*RLcp38_119;
    OPcp38_119 = OPcp38_16+ROcp38_16*qdd[19]+qd[19]*(OMcp38_26*ROcp38_36-OMcp38_36*ROcp38_26);
    OPcp38_219 = OPcp38_26+ROcp38_26*qdd[19]-qd[19]*(OMcp38_16*ROcp38_36-OMcp38_36*ROcp38_16);
    OPcp38_319 = OPcp38_36+ROcp38_36*qdd[19]+qd[19]*(OMcp38_16*ROcp38_26-OMcp38_26*ROcp38_16);
    RLcp38_120 = s->dpt[1][38]*ROcp38_16+s->dpt[2][38]*ROcp38_419+s->dpt[3][38]*ROcp38_719;
    RLcp38_220 = s->dpt[1][38]*ROcp38_26+s->dpt[2][38]*ROcp38_519+s->dpt[3][38]*ROcp38_819;
    RLcp38_320 = s->dpt[1][38]*ROcp38_36+s->dpt[2][38]*ROcp38_619+s->dpt[3][38]*ROcp38_919;
    OMcp38_120 = OMcp38_119+ROcp38_419*qd[20];
    OMcp38_220 = OMcp38_219+ROcp38_519*qd[20];
    OMcp38_320 = OMcp38_319+ROcp38_619*qd[20];
    ORcp38_120 = OMcp38_219*RLcp38_320-OMcp38_319*RLcp38_220;
    ORcp38_220 = -(OMcp38_119*RLcp38_320-OMcp38_319*RLcp38_120);
    ORcp38_320 = OMcp38_119*RLcp38_220-OMcp38_219*RLcp38_120;
    OPcp38_120 = OPcp38_119+ROcp38_419*qdd[20]+qd[20]*(OMcp38_219*ROcp38_619-OMcp38_319*ROcp38_519);
    OPcp38_220 = OPcp38_219+ROcp38_519*qdd[20]-qd[20]*(OMcp38_119*ROcp38_619-OMcp38_319*ROcp38_419);
    OPcp38_320 = OPcp38_319+ROcp38_619*qdd[20]+qd[20]*(OMcp38_119*ROcp38_519-OMcp38_219*ROcp38_419);
    RLcp38_121 = s->dpt[1][40]*ROcp38_120+s->dpt[2][40]*ROcp38_419+ROcp38_720*s->dpt[3][40];
    RLcp38_221 = s->dpt[1][40]*ROcp38_220+s->dpt[2][40]*ROcp38_519+ROcp38_820*s->dpt[3][40];
    RLcp38_321 = s->dpt[1][40]*ROcp38_320+s->dpt[2][40]*ROcp38_619+ROcp38_920*s->dpt[3][40];
    OMcp38_121 = OMcp38_120+ROcp38_720*qd[21];
    OMcp38_221 = OMcp38_220+ROcp38_820*qd[21];
    OMcp38_321 = OMcp38_320+ROcp38_920*qd[21];
    ORcp38_121 = OMcp38_220*RLcp38_321-OMcp38_320*RLcp38_221;
    ORcp38_221 = -(OMcp38_120*RLcp38_321-OMcp38_320*RLcp38_121);
    ORcp38_321 = OMcp38_120*RLcp38_221-OMcp38_220*RLcp38_121;
    OPcp38_121 = OPcp38_120+ROcp38_720*qdd[21]+qd[21]*(OMcp38_220*ROcp38_920-OMcp38_320*ROcp38_820);
    OPcp38_221 = OPcp38_220+ROcp38_820*qdd[21]-qd[21]*(OMcp38_120*ROcp38_920-OMcp38_320*ROcp38_720);
    OPcp38_321 = OPcp38_320+ROcp38_920*qdd[21]+qd[21]*(OMcp38_120*ROcp38_820-OMcp38_220*ROcp38_720);

// = = Block_1_0_0_39_0_5 = = 
 
// Sensor Kinematics 


    ROcp38_122 = ROcp38_121*C22-ROcp38_720*S22;
    ROcp38_222 = ROcp38_221*C22-ROcp38_820*S22;
    ROcp38_322 = ROcp38_321*C22-ROcp38_920*S22;
    ROcp38_722 = ROcp38_121*S22+ROcp38_720*C22;
    ROcp38_822 = ROcp38_221*S22+ROcp38_820*C22;
    ROcp38_922 = ROcp38_321*S22+ROcp38_920*C22;
    ROcp38_423 = ROcp38_421*C23+ROcp38_722*S23;
    ROcp38_523 = ROcp38_521*C23+ROcp38_822*S23;
    ROcp38_623 = ROcp38_621*C23+ROcp38_922*S23;
    ROcp38_723 = -(ROcp38_421*S23-ROcp38_722*C23);
    ROcp38_823 = -(ROcp38_521*S23-ROcp38_822*C23);
    ROcp38_923 = -(ROcp38_621*S23-ROcp38_922*C23);
    RLcp38_122 = ROcp38_121*s->dpt[1][44]+ROcp38_421*s->dpt[2][44]+ROcp38_720*s->dpt[3][44];
    RLcp38_222 = ROcp38_221*s->dpt[1][44]+ROcp38_521*s->dpt[2][44]+ROcp38_820*s->dpt[3][44];
    RLcp38_322 = ROcp38_321*s->dpt[1][44]+ROcp38_621*s->dpt[2][44]+ROcp38_920*s->dpt[3][44];
    OMcp38_122 = OMcp38_121+ROcp38_421*qd[22];
    OMcp38_222 = OMcp38_221+ROcp38_521*qd[22];
    OMcp38_322 = OMcp38_321+ROcp38_621*qd[22];
    ORcp38_122 = OMcp38_221*RLcp38_322-OMcp38_321*RLcp38_222;
    ORcp38_222 = -(OMcp38_121*RLcp38_322-OMcp38_321*RLcp38_122);
    ORcp38_322 = OMcp38_121*RLcp38_222-OMcp38_221*RLcp38_122;
    OPcp38_122 = OPcp38_121+ROcp38_421*qdd[22]+qd[22]*(OMcp38_221*ROcp38_621-OMcp38_321*ROcp38_521);
    OPcp38_222 = OPcp38_221+ROcp38_521*qdd[22]-qd[22]*(OMcp38_121*ROcp38_621-OMcp38_321*ROcp38_421);
    OPcp38_322 = OPcp38_321+ROcp38_621*qdd[22]+qd[22]*(OMcp38_121*ROcp38_521-OMcp38_221*ROcp38_421);
    RLcp38_123 = s->dpt[1][48]*ROcp38_122+s->dpt[3][48]*ROcp38_722+ROcp38_421*s->dpt[2][48];
    RLcp38_223 = s->dpt[1][48]*ROcp38_222+s->dpt[3][48]*ROcp38_822+ROcp38_521*s->dpt[2][48];
    RLcp38_323 = s->dpt[1][48]*ROcp38_322+s->dpt[3][48]*ROcp38_922+ROcp38_621*s->dpt[2][48];
    OMcp38_123 = OMcp38_122+ROcp38_122*qd[23];
    OMcp38_223 = OMcp38_222+ROcp38_222*qd[23];
    OMcp38_323 = OMcp38_322+ROcp38_322*qd[23];
    ORcp38_123 = OMcp38_222*RLcp38_323-OMcp38_322*RLcp38_223;
    ORcp38_223 = -(OMcp38_122*RLcp38_323-OMcp38_322*RLcp38_123);
    ORcp38_323 = OMcp38_122*RLcp38_223-OMcp38_222*RLcp38_123;
    OPcp38_123 = OPcp38_122+ROcp38_122*qdd[23]+qd[23]*(OMcp38_222*ROcp38_322-OMcp38_322*ROcp38_222);
    OPcp38_223 = OPcp38_222+ROcp38_222*qdd[23]-qd[23]*(OMcp38_122*ROcp38_322-OMcp38_322*ROcp38_122);
    OPcp38_323 = OPcp38_322+ROcp38_322*qdd[23]+qd[23]*(OMcp38_122*ROcp38_222-OMcp38_222*ROcp38_122);
    RLcp38_174 = s->dpt[1][50]*ROcp38_122+s->dpt[3][50]*ROcp38_723+ROcp38_423*s->dpt[2][50];
    RLcp38_274 = s->dpt[1][50]*ROcp38_222+s->dpt[3][50]*ROcp38_823+ROcp38_523*s->dpt[2][50];
    RLcp38_374 = s->dpt[1][50]*ROcp38_322+s->dpt[3][50]*ROcp38_923+ROcp38_623*s->dpt[2][50];
    POcp38_174 = RLcp38_119+RLcp38_120+RLcp38_121+RLcp38_122+RLcp38_123+RLcp38_174+q[1];
    POcp38_274 = RLcp38_219+RLcp38_220+RLcp38_221+RLcp38_222+RLcp38_223+RLcp38_274+q[2];
    POcp38_374 = RLcp38_319+RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_374+q[3];
    JTcp38_274_4 = -(RLcp38_319+RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_374);
    JTcp38_374_4 = RLcp38_219+RLcp38_220+RLcp38_221+RLcp38_222+RLcp38_223+RLcp38_274;
    JTcp38_174_5 = C4*(RLcp38_319+RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_374)-S4*(RLcp38_219+RLcp38_220)-S4*(
 RLcp38_221+RLcp38_222)-S4*(RLcp38_223+RLcp38_274);
    JTcp38_274_5 = S4*(RLcp38_119+RLcp38_120+RLcp38_121+RLcp38_122+RLcp38_123+RLcp38_174);
    JTcp38_374_5 = -C4*(RLcp38_119+RLcp38_120+RLcp38_121+RLcp38_122+RLcp38_123+RLcp38_174);
    JTcp38_174_6 = ROcp38_85*(RLcp38_319+RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_374)-ROcp38_95*(RLcp38_219+
 RLcp38_220)-ROcp38_95*(RLcp38_221+RLcp38_222)-ROcp38_95*(RLcp38_223+RLcp38_274);
    JTcp38_274_6 = RLcp38_174*ROcp38_95-RLcp38_323*S5-RLcp38_374*S5+ROcp38_95*(RLcp38_119+RLcp38_120+RLcp38_121+RLcp38_122
 +RLcp38_123)-S5*(RLcp38_319+RLcp38_320)-S5*(RLcp38_321+RLcp38_322);
    JTcp38_374_6 = RLcp38_223*S5-ROcp38_85*(RLcp38_119+RLcp38_120+RLcp38_121+RLcp38_122+RLcp38_123)+S5*(RLcp38_219+
 RLcp38_220)+S5*(RLcp38_221+RLcp38_222)-RLcp38_174*ROcp38_85+RLcp38_274*S5;
    JTcp38_174_7 = ROcp38_26*(RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323)-ROcp38_36*(RLcp38_220+RLcp38_221)-ROcp38_36*(
 RLcp38_222+RLcp38_223)-RLcp38_274*ROcp38_36+RLcp38_374*ROcp38_26;
    JTcp38_274_7 = RLcp38_174*ROcp38_36-RLcp38_374*ROcp38_16-ROcp38_16*(RLcp38_320+RLcp38_321+RLcp38_322+RLcp38_323)+
 ROcp38_36*(RLcp38_120+RLcp38_121)+ROcp38_36*(RLcp38_122+RLcp38_123);
    JTcp38_374_7 = ROcp38_16*(RLcp38_220+RLcp38_221+RLcp38_222+RLcp38_223)-ROcp38_26*(RLcp38_120+RLcp38_121)-ROcp38_26*(
 RLcp38_122+RLcp38_123)-RLcp38_174*ROcp38_26+RLcp38_274*ROcp38_16;
    JTcp38_174_8 = ROcp38_519*(RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_374)-ROcp38_619*(RLcp38_221+RLcp38_222)-ROcp38_619*
 (RLcp38_223+RLcp38_274);
    JTcp38_274_8 = -(ROcp38_419*(RLcp38_321+RLcp38_322+RLcp38_323+RLcp38_374)-ROcp38_619*(RLcp38_121+RLcp38_122)-
 ROcp38_619*(RLcp38_123+RLcp38_174));
    JTcp38_374_8 = ROcp38_419*(RLcp38_221+RLcp38_222+RLcp38_223+RLcp38_274)-ROcp38_519*(RLcp38_121+RLcp38_122)-ROcp38_519*
 (RLcp38_123+RLcp38_174);
    JTcp38_174_9 = ROcp38_820*(RLcp38_322+RLcp38_323)-ROcp38_920*(RLcp38_222+RLcp38_223)-RLcp38_274*ROcp38_920+RLcp38_374*
 ROcp38_820;
    JTcp38_274_9 = RLcp38_174*ROcp38_920-RLcp38_374*ROcp38_720-ROcp38_720*(RLcp38_322+RLcp38_323)+ROcp38_920*(RLcp38_122+
 RLcp38_123);
    JTcp38_374_9 = ROcp38_720*(RLcp38_222+RLcp38_223)-ROcp38_820*(RLcp38_122+RLcp38_123)-RLcp38_174*ROcp38_820+RLcp38_274*
 ROcp38_720;
    JTcp38_174_10 = ROcp38_521*(RLcp38_323+RLcp38_374)-ROcp38_621*(RLcp38_223+RLcp38_274);
    JTcp38_274_10 = -(ROcp38_421*(RLcp38_323+RLcp38_374)-ROcp38_621*(RLcp38_123+RLcp38_174));
    JTcp38_374_10 = ROcp38_421*(RLcp38_223+RLcp38_274)-ROcp38_521*(RLcp38_123+RLcp38_174);
    JTcp38_174_11 = -(RLcp38_274*ROcp38_322-RLcp38_374*ROcp38_222);
    JTcp38_274_11 = RLcp38_174*ROcp38_322-RLcp38_374*ROcp38_122;
    JTcp38_374_11 = -(RLcp38_174*ROcp38_222-RLcp38_274*ROcp38_122);
    ORcp38_174 = OMcp38_223*RLcp38_374-OMcp38_323*RLcp38_274;
    ORcp38_274 = -(OMcp38_123*RLcp38_374-OMcp38_323*RLcp38_174);
    ORcp38_374 = OMcp38_123*RLcp38_274-OMcp38_223*RLcp38_174;
    VIcp38_174 = ORcp38_119+ORcp38_120+ORcp38_121+ORcp38_122+ORcp38_123+ORcp38_174+qd[1];
    VIcp38_274 = ORcp38_219+ORcp38_220+ORcp38_221+ORcp38_222+ORcp38_223+ORcp38_274+qd[2];
    VIcp38_374 = ORcp38_319+ORcp38_320+ORcp38_321+ORcp38_322+ORcp38_323+ORcp38_374+qd[3];
    ACcp38_174 = qdd[1]+OMcp38_219*ORcp38_320+OMcp38_220*ORcp38_321+OMcp38_221*ORcp38_322+OMcp38_222*ORcp38_323+OMcp38_223
 *ORcp38_374+OMcp38_26*ORcp38_319-OMcp38_319*ORcp38_220-OMcp38_320*ORcp38_221-OMcp38_321*ORcp38_222-OMcp38_322*ORcp38_223-
 OMcp38_323*ORcp38_274-OMcp38_36*ORcp38_219+OPcp38_219*RLcp38_320+OPcp38_220*RLcp38_321+OPcp38_221*RLcp38_322+OPcp38_222*
 RLcp38_323+OPcp38_223*RLcp38_374+OPcp38_26*RLcp38_319-OPcp38_319*RLcp38_220-OPcp38_320*RLcp38_221-OPcp38_321*RLcp38_222-
 OPcp38_322*RLcp38_223-OPcp38_323*RLcp38_274-OPcp38_36*RLcp38_219;
    ACcp38_274 = qdd[2]-OMcp38_119*ORcp38_320-OMcp38_120*ORcp38_321-OMcp38_121*ORcp38_322-OMcp38_122*ORcp38_323-OMcp38_123
 *ORcp38_374-OMcp38_16*ORcp38_319+OMcp38_319*ORcp38_120+OMcp38_320*ORcp38_121+OMcp38_321*ORcp38_122+OMcp38_322*ORcp38_123+
 OMcp38_323*ORcp38_174+OMcp38_36*ORcp38_119-OPcp38_119*RLcp38_320-OPcp38_120*RLcp38_321-OPcp38_121*RLcp38_322-OPcp38_122*
 RLcp38_323-OPcp38_123*RLcp38_374-OPcp38_16*RLcp38_319+OPcp38_319*RLcp38_120+OPcp38_320*RLcp38_121+OPcp38_321*RLcp38_122+
 OPcp38_322*RLcp38_123+OPcp38_323*RLcp38_174+OPcp38_36*RLcp38_119;
    ACcp38_374 = qdd[3]+OMcp38_119*ORcp38_220+OMcp38_120*ORcp38_221+OMcp38_121*ORcp38_222+OMcp38_122*ORcp38_223+OMcp38_123
 *ORcp38_274+OMcp38_16*ORcp38_219-OMcp38_219*ORcp38_120-OMcp38_220*ORcp38_121-OMcp38_221*ORcp38_122-OMcp38_222*ORcp38_123-
 OMcp38_223*ORcp38_174-OMcp38_26*ORcp38_119+OPcp38_119*RLcp38_220+OPcp38_120*RLcp38_221+OPcp38_121*RLcp38_222+OPcp38_122*
 RLcp38_223+OPcp38_123*RLcp38_274+OPcp38_16*RLcp38_219-OPcp38_219*RLcp38_120-OPcp38_220*RLcp38_121-OPcp38_221*RLcp38_122-
 OPcp38_222*RLcp38_123-OPcp38_223*RLcp38_174-OPcp38_26*RLcp38_119;

// = = Block_1_0_0_39_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp38_174;
    sens->P[2] = POcp38_274;
    sens->P[3] = POcp38_374;
    sens->R[1][1] = ROcp38_122;
    sens->R[1][2] = ROcp38_222;
    sens->R[1][3] = ROcp38_322;
    sens->R[2][1] = ROcp38_423;
    sens->R[2][2] = ROcp38_523;
    sens->R[2][3] = ROcp38_623;
    sens->R[3][1] = ROcp38_723;
    sens->R[3][2] = ROcp38_823;
    sens->R[3][3] = ROcp38_923;
    sens->V[1] = VIcp38_174;
    sens->V[2] = VIcp38_274;
    sens->V[3] = VIcp38_374;
    sens->OM[1] = OMcp38_123;
    sens->OM[2] = OMcp38_223;
    sens->OM[3] = OMcp38_323;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp38_174_5;
    sens->J[1][6] = JTcp38_174_6;
    sens->J[1][19] = JTcp38_174_7;
    sens->J[1][20] = JTcp38_174_8;
    sens->J[1][21] = JTcp38_174_9;
    sens->J[1][22] = JTcp38_174_10;
    sens->J[1][23] = JTcp38_174_11;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp38_274_4;
    sens->J[2][5] = JTcp38_274_5;
    sens->J[2][6] = JTcp38_274_6;
    sens->J[2][19] = JTcp38_274_7;
    sens->J[2][20] = JTcp38_274_8;
    sens->J[2][21] = JTcp38_274_9;
    sens->J[2][22] = JTcp38_274_10;
    sens->J[2][23] = JTcp38_274_11;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp38_374_4;
    sens->J[3][5] = JTcp38_374_5;
    sens->J[3][6] = JTcp38_374_6;
    sens->J[3][19] = JTcp38_374_7;
    sens->J[3][20] = JTcp38_374_8;
    sens->J[3][21] = JTcp38_374_9;
    sens->J[3][22] = JTcp38_374_10;
    sens->J[3][23] = JTcp38_374_11;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp38_16;
    sens->J[4][20] = ROcp38_419;
    sens->J[4][21] = ROcp38_720;
    sens->J[4][22] = ROcp38_421;
    sens->J[4][23] = ROcp38_122;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp38_85;
    sens->J[5][19] = ROcp38_26;
    sens->J[5][20] = ROcp38_519;
    sens->J[5][21] = ROcp38_820;
    sens->J[5][22] = ROcp38_521;
    sens->J[5][23] = ROcp38_222;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp38_95;
    sens->J[6][19] = ROcp38_36;
    sens->J[6][20] = ROcp38_619;
    sens->J[6][21] = ROcp38_920;
    sens->J[6][22] = ROcp38_621;
    sens->J[6][23] = ROcp38_322;
    sens->A[1] = ACcp38_174;
    sens->A[2] = ACcp38_274;
    sens->A[3] = ACcp38_374;
    sens->OMP[1] = OPcp38_123;
    sens->OMP[2] = OPcp38_223;
    sens->OMP[3] = OPcp38_323;
 
// 
break;
case 40:
 


// = = Block_1_0_0_40_0_1 = = 
 
// Sensor Kinematics 


    ROcp39_25 = S4*S5;
    ROcp39_35 = -C4*S5;
    ROcp39_85 = -S4*C5;
    ROcp39_95 = C4*C5;
    ROcp39_16 = C5*C6;
    ROcp39_26 = ROcp39_25*C6+C4*S6;
    ROcp39_36 = ROcp39_35*C6+S4*S6;
    ROcp39_46 = -C5*S6;
    ROcp39_56 = -(ROcp39_25*S6-C4*C6);
    ROcp39_66 = -(ROcp39_35*S6-S4*C6);
    OMcp39_25 = qd[5]*C4;
    OMcp39_35 = qd[5]*S4;
    OMcp39_16 = qd[4]+qd[6]*S5;
    OMcp39_26 = OMcp39_25+ROcp39_85*qd[6];
    OMcp39_36 = OMcp39_35+ROcp39_95*qd[6];
    OPcp39_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp39_26 = ROcp39_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp39_35*S5-ROcp39_95*qd[4]);
    OPcp39_36 = ROcp39_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp39_25*S5-ROcp39_85*qd[4]);

// = = Block_1_0_0_40_0_4 = = 
 
// Sensor Kinematics 


    ROcp39_419 = ROcp39_46*C19+S19*S5;
    ROcp39_519 = ROcp39_56*C19+ROcp39_85*S19;
    ROcp39_619 = ROcp39_66*C19+ROcp39_95*S19;
    ROcp39_719 = -(ROcp39_46*S19-C19*S5);
    ROcp39_819 = -(ROcp39_56*S19-ROcp39_85*C19);
    ROcp39_919 = -(ROcp39_66*S19-ROcp39_95*C19);
    ROcp39_120 = ROcp39_16*C20-ROcp39_719*S20;
    ROcp39_220 = ROcp39_26*C20-ROcp39_819*S20;
    ROcp39_320 = ROcp39_36*C20-ROcp39_919*S20;
    ROcp39_720 = ROcp39_16*S20+ROcp39_719*C20;
    ROcp39_820 = ROcp39_26*S20+ROcp39_819*C20;
    ROcp39_920 = ROcp39_36*S20+ROcp39_919*C20;
    ROcp39_121 = ROcp39_120*C21+ROcp39_419*S21;
    ROcp39_221 = ROcp39_220*C21+ROcp39_519*S21;
    ROcp39_321 = ROcp39_320*C21+ROcp39_619*S21;
    ROcp39_421 = -(ROcp39_120*S21-ROcp39_419*C21);
    ROcp39_521 = -(ROcp39_220*S21-ROcp39_519*C21);
    ROcp39_621 = -(ROcp39_320*S21-ROcp39_619*C21);
    RLcp39_119 = s->dpt[2][3]*ROcp39_46+ROcp39_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp39_219 = s->dpt[2][3]*ROcp39_56+ROcp39_26*s->dpt[1][3]+ROcp39_85*s->dpt[3][3];
    RLcp39_319 = s->dpt[2][3]*ROcp39_66+ROcp39_36*s->dpt[1][3]+ROcp39_95*s->dpt[3][3];
    OMcp39_119 = OMcp39_16+ROcp39_16*qd[19];
    OMcp39_219 = OMcp39_26+ROcp39_26*qd[19];
    OMcp39_319 = OMcp39_36+ROcp39_36*qd[19];
    ORcp39_119 = OMcp39_26*RLcp39_319-OMcp39_36*RLcp39_219;
    ORcp39_219 = -(OMcp39_16*RLcp39_319-OMcp39_36*RLcp39_119);
    ORcp39_319 = OMcp39_16*RLcp39_219-OMcp39_26*RLcp39_119;
    OPcp39_119 = OPcp39_16+ROcp39_16*qdd[19]+qd[19]*(OMcp39_26*ROcp39_36-OMcp39_36*ROcp39_26);
    OPcp39_219 = OPcp39_26+ROcp39_26*qdd[19]-qd[19]*(OMcp39_16*ROcp39_36-OMcp39_36*ROcp39_16);
    OPcp39_319 = OPcp39_36+ROcp39_36*qdd[19]+qd[19]*(OMcp39_16*ROcp39_26-OMcp39_26*ROcp39_16);
    RLcp39_120 = s->dpt[1][38]*ROcp39_16+s->dpt[2][38]*ROcp39_419+s->dpt[3][38]*ROcp39_719;
    RLcp39_220 = s->dpt[1][38]*ROcp39_26+s->dpt[2][38]*ROcp39_519+s->dpt[3][38]*ROcp39_819;
    RLcp39_320 = s->dpt[1][38]*ROcp39_36+s->dpt[2][38]*ROcp39_619+s->dpt[3][38]*ROcp39_919;
    OMcp39_120 = OMcp39_119+ROcp39_419*qd[20];
    OMcp39_220 = OMcp39_219+ROcp39_519*qd[20];
    OMcp39_320 = OMcp39_319+ROcp39_619*qd[20];
    ORcp39_120 = OMcp39_219*RLcp39_320-OMcp39_319*RLcp39_220;
    ORcp39_220 = -(OMcp39_119*RLcp39_320-OMcp39_319*RLcp39_120);
    ORcp39_320 = OMcp39_119*RLcp39_220-OMcp39_219*RLcp39_120;
    OPcp39_120 = OPcp39_119+ROcp39_419*qdd[20]+qd[20]*(OMcp39_219*ROcp39_619-OMcp39_319*ROcp39_519);
    OPcp39_220 = OPcp39_219+ROcp39_519*qdd[20]-qd[20]*(OMcp39_119*ROcp39_619-OMcp39_319*ROcp39_419);
    OPcp39_320 = OPcp39_319+ROcp39_619*qdd[20]+qd[20]*(OMcp39_119*ROcp39_519-OMcp39_219*ROcp39_419);
    RLcp39_121 = s->dpt[1][40]*ROcp39_120+s->dpt[2][40]*ROcp39_419+ROcp39_720*s->dpt[3][40];
    RLcp39_221 = s->dpt[1][40]*ROcp39_220+s->dpt[2][40]*ROcp39_519+ROcp39_820*s->dpt[3][40];
    RLcp39_321 = s->dpt[1][40]*ROcp39_320+s->dpt[2][40]*ROcp39_619+ROcp39_920*s->dpt[3][40];
    OMcp39_121 = OMcp39_120+ROcp39_720*qd[21];
    OMcp39_221 = OMcp39_220+ROcp39_820*qd[21];
    OMcp39_321 = OMcp39_320+ROcp39_920*qd[21];
    ORcp39_121 = OMcp39_220*RLcp39_321-OMcp39_320*RLcp39_221;
    ORcp39_221 = -(OMcp39_120*RLcp39_321-OMcp39_320*RLcp39_121);
    ORcp39_321 = OMcp39_120*RLcp39_221-OMcp39_220*RLcp39_121;
    OPcp39_121 = OPcp39_120+ROcp39_720*qdd[21]+qd[21]*(OMcp39_220*ROcp39_920-OMcp39_320*ROcp39_820);
    OPcp39_221 = OPcp39_220+ROcp39_820*qdd[21]-qd[21]*(OMcp39_120*ROcp39_920-OMcp39_320*ROcp39_720);
    OPcp39_321 = OPcp39_320+ROcp39_920*qdd[21]+qd[21]*(OMcp39_120*ROcp39_820-OMcp39_220*ROcp39_720);

// = = Block_1_0_0_40_0_5 = = 
 
// Sensor Kinematics 


    ROcp39_122 = ROcp39_121*C22-ROcp39_720*S22;
    ROcp39_222 = ROcp39_221*C22-ROcp39_820*S22;
    ROcp39_322 = ROcp39_321*C22-ROcp39_920*S22;
    ROcp39_722 = ROcp39_121*S22+ROcp39_720*C22;
    ROcp39_822 = ROcp39_221*S22+ROcp39_820*C22;
    ROcp39_922 = ROcp39_321*S22+ROcp39_920*C22;
    ROcp39_423 = ROcp39_421*C23+ROcp39_722*S23;
    ROcp39_523 = ROcp39_521*C23+ROcp39_822*S23;
    ROcp39_623 = ROcp39_621*C23+ROcp39_922*S23;
    ROcp39_723 = -(ROcp39_421*S23-ROcp39_722*C23);
    ROcp39_823 = -(ROcp39_521*S23-ROcp39_822*C23);
    ROcp39_923 = -(ROcp39_621*S23-ROcp39_922*C23);
    ROcp39_124 = ROcp39_122*C24-ROcp39_723*S24;
    ROcp39_224 = ROcp39_222*C24-ROcp39_823*S24;
    ROcp39_324 = ROcp39_322*C24-ROcp39_923*S24;
    ROcp39_724 = ROcp39_122*S24+ROcp39_723*C24;
    ROcp39_824 = ROcp39_222*S24+ROcp39_823*C24;
    ROcp39_924 = ROcp39_322*S24+ROcp39_923*C24;
    RLcp39_122 = ROcp39_121*s->dpt[1][44]+ROcp39_421*s->dpt[2][44]+ROcp39_720*s->dpt[3][44];
    RLcp39_222 = ROcp39_221*s->dpt[1][44]+ROcp39_521*s->dpt[2][44]+ROcp39_820*s->dpt[3][44];
    RLcp39_322 = ROcp39_321*s->dpt[1][44]+ROcp39_621*s->dpt[2][44]+ROcp39_920*s->dpt[3][44];
    OMcp39_122 = OMcp39_121+ROcp39_421*qd[22];
    OMcp39_222 = OMcp39_221+ROcp39_521*qd[22];
    OMcp39_322 = OMcp39_321+ROcp39_621*qd[22];
    ORcp39_122 = OMcp39_221*RLcp39_322-OMcp39_321*RLcp39_222;
    ORcp39_222 = -(OMcp39_121*RLcp39_322-OMcp39_321*RLcp39_122);
    ORcp39_322 = OMcp39_121*RLcp39_222-OMcp39_221*RLcp39_122;
    OPcp39_122 = OPcp39_121+ROcp39_421*qdd[22]+qd[22]*(OMcp39_221*ROcp39_621-OMcp39_321*ROcp39_521);
    OPcp39_222 = OPcp39_221+ROcp39_521*qdd[22]-qd[22]*(OMcp39_121*ROcp39_621-OMcp39_321*ROcp39_421);
    OPcp39_322 = OPcp39_321+ROcp39_621*qdd[22]+qd[22]*(OMcp39_121*ROcp39_521-OMcp39_221*ROcp39_421);
    RLcp39_123 = s->dpt[1][48]*ROcp39_122+s->dpt[3][48]*ROcp39_722+ROcp39_421*s->dpt[2][48];
    RLcp39_223 = s->dpt[1][48]*ROcp39_222+s->dpt[3][48]*ROcp39_822+ROcp39_521*s->dpt[2][48];
    RLcp39_323 = s->dpt[1][48]*ROcp39_322+s->dpt[3][48]*ROcp39_922+ROcp39_621*s->dpt[2][48];
    OMcp39_123 = OMcp39_122+ROcp39_122*qd[23];
    OMcp39_223 = OMcp39_222+ROcp39_222*qd[23];
    OMcp39_323 = OMcp39_322+ROcp39_322*qd[23];
    ORcp39_123 = OMcp39_222*RLcp39_323-OMcp39_322*RLcp39_223;
    ORcp39_223 = -(OMcp39_122*RLcp39_323-OMcp39_322*RLcp39_123);
    ORcp39_323 = OMcp39_122*RLcp39_223-OMcp39_222*RLcp39_123;
    OPcp39_123 = OPcp39_122+ROcp39_122*qdd[23]+qd[23]*(OMcp39_222*ROcp39_322-OMcp39_322*ROcp39_222);
    OPcp39_223 = OPcp39_222+ROcp39_222*qdd[23]-qd[23]*(OMcp39_122*ROcp39_322-OMcp39_322*ROcp39_122);
    OPcp39_323 = OPcp39_322+ROcp39_322*qdd[23]+qd[23]*(OMcp39_122*ROcp39_222-OMcp39_222*ROcp39_122);
    RLcp39_124 = s->dpt[1][50]*ROcp39_122+s->dpt[3][50]*ROcp39_723+ROcp39_423*s->dpt[2][50];
    RLcp39_224 = s->dpt[1][50]*ROcp39_222+s->dpt[3][50]*ROcp39_823+ROcp39_523*s->dpt[2][50];
    RLcp39_324 = s->dpt[1][50]*ROcp39_322+s->dpt[3][50]*ROcp39_923+ROcp39_623*s->dpt[2][50];
    OMcp39_124 = OMcp39_123+ROcp39_423*qd[24];
    OMcp39_224 = OMcp39_223+ROcp39_523*qd[24];
    OMcp39_324 = OMcp39_323+ROcp39_623*qd[24];
    ORcp39_124 = OMcp39_223*RLcp39_324-OMcp39_323*RLcp39_224;
    ORcp39_224 = -(OMcp39_123*RLcp39_324-OMcp39_323*RLcp39_124);
    ORcp39_324 = OMcp39_123*RLcp39_224-OMcp39_223*RLcp39_124;
    OPcp39_124 = OPcp39_123+ROcp39_423*qdd[24]+qd[24]*(OMcp39_223*ROcp39_623-OMcp39_323*ROcp39_523);
    OPcp39_224 = OPcp39_223+ROcp39_523*qdd[24]-qd[24]*(OMcp39_123*ROcp39_623-OMcp39_323*ROcp39_423);
    OPcp39_324 = OPcp39_323+ROcp39_623*qdd[24]+qd[24]*(OMcp39_123*ROcp39_523-OMcp39_223*ROcp39_423);
    RLcp39_175 = ROcp39_124*s->dpt[1][51]+ROcp39_423*s->dpt[2][51]+ROcp39_724*s->dpt[3][51];
    RLcp39_275 = ROcp39_224*s->dpt[1][51]+ROcp39_523*s->dpt[2][51]+ROcp39_824*s->dpt[3][51];
    RLcp39_375 = ROcp39_324*s->dpt[1][51]+ROcp39_623*s->dpt[2][51]+ROcp39_924*s->dpt[3][51];
    POcp39_175 = RLcp39_119+RLcp39_120+RLcp39_121+RLcp39_122+RLcp39_123+RLcp39_124+RLcp39_175+q[1];
    POcp39_275 = RLcp39_219+RLcp39_220+RLcp39_221+RLcp39_222+RLcp39_223+RLcp39_224+RLcp39_275+q[2];
    POcp39_375 = RLcp39_319+RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_375+q[3];
    JTcp39_275_4 = -(RLcp39_319+RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_375);
    JTcp39_375_4 = RLcp39_219+RLcp39_220+RLcp39_221+RLcp39_222+RLcp39_223+RLcp39_224+RLcp39_275;
    JTcp39_175_5 = C4*(RLcp39_319+RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324)-S4*(RLcp39_219+RLcp39_220)-S4*(
 RLcp39_221+RLcp39_222)-S4*(RLcp39_223+RLcp39_224)-RLcp39_275*S4+RLcp39_375*C4;
    JTcp39_275_5 = S4*(RLcp39_119+RLcp39_120+RLcp39_121+RLcp39_122+RLcp39_123+RLcp39_124+RLcp39_175);
    JTcp39_375_5 = -C4*(RLcp39_119+RLcp39_120+RLcp39_121+RLcp39_122+RLcp39_123+RLcp39_124+RLcp39_175);
    JTcp39_175_6 = ROcp39_85*(RLcp39_319+RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324)-ROcp39_95*(RLcp39_219+
 RLcp39_220)-ROcp39_95*(RLcp39_221+RLcp39_222)-ROcp39_95*(RLcp39_223+RLcp39_224)-RLcp39_275*ROcp39_95+RLcp39_375*ROcp39_85;
    JTcp39_275_6 = -(RLcp39_375*S5-ROcp39_95*(RLcp39_119+RLcp39_120+RLcp39_121+RLcp39_122+RLcp39_123+RLcp39_124+RLcp39_175
 )+S5*(RLcp39_319+RLcp39_320)+S5*(RLcp39_321+RLcp39_322)+S5*(RLcp39_323+RLcp39_324));
    JTcp39_375_6 = RLcp39_275*S5-ROcp39_85*(RLcp39_119+RLcp39_120+RLcp39_121+RLcp39_122+RLcp39_123+RLcp39_124+RLcp39_175)+
 S5*(RLcp39_219+RLcp39_220)+S5*(RLcp39_221+RLcp39_222)+S5*(RLcp39_223+RLcp39_224);
    JTcp39_175_7 = ROcp39_26*(RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_375)-ROcp39_36*(RLcp39_220+
 RLcp39_221)-ROcp39_36*(RLcp39_222+RLcp39_223)-ROcp39_36*(RLcp39_224+RLcp39_275);
    JTcp39_275_7 = -(ROcp39_16*(RLcp39_320+RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_375)-ROcp39_36*(RLcp39_120+
 RLcp39_121)-ROcp39_36*(RLcp39_122+RLcp39_123)-ROcp39_36*(RLcp39_124+RLcp39_175));
    JTcp39_375_7 = ROcp39_16*(RLcp39_220+RLcp39_221+RLcp39_222+RLcp39_223+RLcp39_224+RLcp39_275)-ROcp39_26*(RLcp39_120+
 RLcp39_121)-ROcp39_26*(RLcp39_122+RLcp39_123)-ROcp39_26*(RLcp39_124+RLcp39_175);
    JTcp39_175_8 = ROcp39_519*(RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324)-ROcp39_619*(RLcp39_221+RLcp39_222)-ROcp39_619*
 (RLcp39_223+RLcp39_224)-RLcp39_275*ROcp39_619+RLcp39_375*ROcp39_519;
    JTcp39_275_8 = RLcp39_175*ROcp39_619-RLcp39_375*ROcp39_419-ROcp39_419*(RLcp39_321+RLcp39_322+RLcp39_323+RLcp39_324)+
 ROcp39_619*(RLcp39_121+RLcp39_122)+ROcp39_619*(RLcp39_123+RLcp39_124);
    JTcp39_375_8 = ROcp39_419*(RLcp39_221+RLcp39_222+RLcp39_223+RLcp39_224)-ROcp39_519*(RLcp39_121+RLcp39_122)-ROcp39_519*
 (RLcp39_123+RLcp39_124)-RLcp39_175*ROcp39_519+RLcp39_275*ROcp39_419;
    JTcp39_175_9 = ROcp39_820*(RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_375)-ROcp39_920*(RLcp39_222+RLcp39_223)-ROcp39_920*
 (RLcp39_224+RLcp39_275);
    JTcp39_275_9 = -(ROcp39_720*(RLcp39_322+RLcp39_323+RLcp39_324+RLcp39_375)-ROcp39_920*(RLcp39_122+RLcp39_123)-
 ROcp39_920*(RLcp39_124+RLcp39_175));
    JTcp39_375_9 = ROcp39_720*(RLcp39_222+RLcp39_223+RLcp39_224+RLcp39_275)-ROcp39_820*(RLcp39_122+RLcp39_123)-ROcp39_820*
 (RLcp39_124+RLcp39_175);
    JTcp39_175_10 = ROcp39_521*(RLcp39_323+RLcp39_324)-ROcp39_621*(RLcp39_223+RLcp39_224)-RLcp39_275*ROcp39_621+RLcp39_375
 *ROcp39_521;
    JTcp39_275_10 = RLcp39_175*ROcp39_621-RLcp39_375*ROcp39_421-ROcp39_421*(RLcp39_323+RLcp39_324)+ROcp39_621*(RLcp39_123+
 RLcp39_124);
    JTcp39_375_10 = ROcp39_421*(RLcp39_223+RLcp39_224)-ROcp39_521*(RLcp39_123+RLcp39_124)-RLcp39_175*ROcp39_521+RLcp39_275
 *ROcp39_421;
    JTcp39_175_11 = ROcp39_222*(RLcp39_324+RLcp39_375)-ROcp39_322*(RLcp39_224+RLcp39_275);
    JTcp39_275_11 = -(ROcp39_122*(RLcp39_324+RLcp39_375)-ROcp39_322*(RLcp39_124+RLcp39_175));
    JTcp39_375_11 = ROcp39_122*(RLcp39_224+RLcp39_275)-ROcp39_222*(RLcp39_124+RLcp39_175);
    JTcp39_175_12 = -(RLcp39_275*ROcp39_623-RLcp39_375*ROcp39_523);
    JTcp39_275_12 = RLcp39_175*ROcp39_623-RLcp39_375*ROcp39_423;
    JTcp39_375_12 = -(RLcp39_175*ROcp39_523-RLcp39_275*ROcp39_423);
    ORcp39_175 = OMcp39_224*RLcp39_375-OMcp39_324*RLcp39_275;
    ORcp39_275 = -(OMcp39_124*RLcp39_375-OMcp39_324*RLcp39_175);
    ORcp39_375 = OMcp39_124*RLcp39_275-OMcp39_224*RLcp39_175;
    VIcp39_175 = ORcp39_119+ORcp39_120+ORcp39_121+ORcp39_122+ORcp39_123+ORcp39_124+ORcp39_175+qd[1];
    VIcp39_275 = ORcp39_219+ORcp39_220+ORcp39_221+ORcp39_222+ORcp39_223+ORcp39_224+ORcp39_275+qd[2];
    VIcp39_375 = ORcp39_319+ORcp39_320+ORcp39_321+ORcp39_322+ORcp39_323+ORcp39_324+ORcp39_375+qd[3];
    ACcp39_175 = qdd[1]+OMcp39_219*ORcp39_320+OMcp39_220*ORcp39_321+OMcp39_221*ORcp39_322+OMcp39_222*ORcp39_323+OMcp39_223
 *ORcp39_324+OMcp39_224*ORcp39_375+OMcp39_26*ORcp39_319-OMcp39_319*ORcp39_220-OMcp39_320*ORcp39_221-OMcp39_321*ORcp39_222-
 OMcp39_322*ORcp39_223-OMcp39_323*ORcp39_224-OMcp39_324*ORcp39_275-OMcp39_36*ORcp39_219+OPcp39_219*RLcp39_320+OPcp39_220*
 RLcp39_321+OPcp39_221*RLcp39_322+OPcp39_222*RLcp39_323+OPcp39_223*RLcp39_324+OPcp39_224*RLcp39_375+OPcp39_26*RLcp39_319-
 OPcp39_319*RLcp39_220-OPcp39_320*RLcp39_221-OPcp39_321*RLcp39_222-OPcp39_322*RLcp39_223-OPcp39_323*RLcp39_224-OPcp39_324*
 RLcp39_275-OPcp39_36*RLcp39_219;
    ACcp39_275 = qdd[2]-OMcp39_119*ORcp39_320-OMcp39_120*ORcp39_321-OMcp39_121*ORcp39_322-OMcp39_122*ORcp39_323-OMcp39_123
 *ORcp39_324-OMcp39_124*ORcp39_375-OMcp39_16*ORcp39_319+OMcp39_319*ORcp39_120+OMcp39_320*ORcp39_121+OMcp39_321*ORcp39_122+
 OMcp39_322*ORcp39_123+OMcp39_323*ORcp39_124+OMcp39_324*ORcp39_175+OMcp39_36*ORcp39_119-OPcp39_119*RLcp39_320-OPcp39_120*
 RLcp39_321-OPcp39_121*RLcp39_322-OPcp39_122*RLcp39_323-OPcp39_123*RLcp39_324-OPcp39_124*RLcp39_375-OPcp39_16*RLcp39_319+
 OPcp39_319*RLcp39_120+OPcp39_320*RLcp39_121+OPcp39_321*RLcp39_122+OPcp39_322*RLcp39_123+OPcp39_323*RLcp39_124+OPcp39_324*
 RLcp39_175+OPcp39_36*RLcp39_119;
    ACcp39_375 = qdd[3]+OMcp39_119*ORcp39_220+OMcp39_120*ORcp39_221+OMcp39_121*ORcp39_222+OMcp39_122*ORcp39_223+OMcp39_123
 *ORcp39_224+OMcp39_124*ORcp39_275+OMcp39_16*ORcp39_219-OMcp39_219*ORcp39_120-OMcp39_220*ORcp39_121-OMcp39_221*ORcp39_122-
 OMcp39_222*ORcp39_123-OMcp39_223*ORcp39_124-OMcp39_224*ORcp39_175-OMcp39_26*ORcp39_119+OPcp39_119*RLcp39_220+OPcp39_120*
 RLcp39_221+OPcp39_121*RLcp39_222+OPcp39_122*RLcp39_223+OPcp39_123*RLcp39_224+OPcp39_124*RLcp39_275+OPcp39_16*RLcp39_219-
 OPcp39_219*RLcp39_120-OPcp39_220*RLcp39_121-OPcp39_221*RLcp39_122-OPcp39_222*RLcp39_123-OPcp39_223*RLcp39_124-OPcp39_224*
 RLcp39_175-OPcp39_26*RLcp39_119;

// = = Block_1_0_0_40_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp39_175;
    sens->P[2] = POcp39_275;
    sens->P[3] = POcp39_375;
    sens->R[1][1] = ROcp39_124;
    sens->R[1][2] = ROcp39_224;
    sens->R[1][3] = ROcp39_324;
    sens->R[2][1] = ROcp39_423;
    sens->R[2][2] = ROcp39_523;
    sens->R[2][3] = ROcp39_623;
    sens->R[3][1] = ROcp39_724;
    sens->R[3][2] = ROcp39_824;
    sens->R[3][3] = ROcp39_924;
    sens->V[1] = VIcp39_175;
    sens->V[2] = VIcp39_275;
    sens->V[3] = VIcp39_375;
    sens->OM[1] = OMcp39_124;
    sens->OM[2] = OMcp39_224;
    sens->OM[3] = OMcp39_324;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp39_175_5;
    sens->J[1][6] = JTcp39_175_6;
    sens->J[1][19] = JTcp39_175_7;
    sens->J[1][20] = JTcp39_175_8;
    sens->J[1][21] = JTcp39_175_9;
    sens->J[1][22] = JTcp39_175_10;
    sens->J[1][23] = JTcp39_175_11;
    sens->J[1][24] = JTcp39_175_12;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp39_275_4;
    sens->J[2][5] = JTcp39_275_5;
    sens->J[2][6] = JTcp39_275_6;
    sens->J[2][19] = JTcp39_275_7;
    sens->J[2][20] = JTcp39_275_8;
    sens->J[2][21] = JTcp39_275_9;
    sens->J[2][22] = JTcp39_275_10;
    sens->J[2][23] = JTcp39_275_11;
    sens->J[2][24] = JTcp39_275_12;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp39_375_4;
    sens->J[3][5] = JTcp39_375_5;
    sens->J[3][6] = JTcp39_375_6;
    sens->J[3][19] = JTcp39_375_7;
    sens->J[3][20] = JTcp39_375_8;
    sens->J[3][21] = JTcp39_375_9;
    sens->J[3][22] = JTcp39_375_10;
    sens->J[3][23] = JTcp39_375_11;
    sens->J[3][24] = JTcp39_375_12;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp39_16;
    sens->J[4][20] = ROcp39_419;
    sens->J[4][21] = ROcp39_720;
    sens->J[4][22] = ROcp39_421;
    sens->J[4][23] = ROcp39_122;
    sens->J[4][24] = ROcp39_423;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp39_85;
    sens->J[5][19] = ROcp39_26;
    sens->J[5][20] = ROcp39_519;
    sens->J[5][21] = ROcp39_820;
    sens->J[5][22] = ROcp39_521;
    sens->J[5][23] = ROcp39_222;
    sens->J[5][24] = ROcp39_523;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp39_95;
    sens->J[6][19] = ROcp39_36;
    sens->J[6][20] = ROcp39_619;
    sens->J[6][21] = ROcp39_920;
    sens->J[6][22] = ROcp39_621;
    sens->J[6][23] = ROcp39_322;
    sens->J[6][24] = ROcp39_623;
    sens->A[1] = ACcp39_175;
    sens->A[2] = ACcp39_275;
    sens->A[3] = ACcp39_375;
    sens->OMP[1] = OPcp39_124;
    sens->OMP[2] = OPcp39_224;
    sens->OMP[3] = OPcp39_324;
 
// 
break;
case 41:
 


// = = Block_1_0_0_41_0_1 = = 
 
// Sensor Kinematics 


    ROcp40_25 = S4*S5;
    ROcp40_35 = -C4*S5;
    ROcp40_85 = -S4*C5;
    ROcp40_95 = C4*C5;
    ROcp40_16 = C5*C6;
    ROcp40_26 = ROcp40_25*C6+C4*S6;
    ROcp40_36 = ROcp40_35*C6+S4*S6;
    ROcp40_46 = -C5*S6;
    ROcp40_56 = -(ROcp40_25*S6-C4*C6);
    ROcp40_66 = -(ROcp40_35*S6-S4*C6);
    OMcp40_25 = qd[5]*C4;
    OMcp40_35 = qd[5]*S4;
    OMcp40_16 = qd[4]+qd[6]*S5;
    OMcp40_26 = OMcp40_25+ROcp40_85*qd[6];
    OMcp40_36 = OMcp40_35+ROcp40_95*qd[6];
    OPcp40_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp40_26 = ROcp40_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp40_35*S5-ROcp40_95*qd[4]);
    OPcp40_36 = ROcp40_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp40_25*S5-ROcp40_85*qd[4]);

// = = Block_1_0_0_41_0_4 = = 
 
// Sensor Kinematics 


    ROcp40_419 = ROcp40_46*C19+S19*S5;
    ROcp40_519 = ROcp40_56*C19+ROcp40_85*S19;
    ROcp40_619 = ROcp40_66*C19+ROcp40_95*S19;
    ROcp40_719 = -(ROcp40_46*S19-C19*S5);
    ROcp40_819 = -(ROcp40_56*S19-ROcp40_85*C19);
    ROcp40_919 = -(ROcp40_66*S19-ROcp40_95*C19);
    ROcp40_120 = ROcp40_16*C20-ROcp40_719*S20;
    ROcp40_220 = ROcp40_26*C20-ROcp40_819*S20;
    ROcp40_320 = ROcp40_36*C20-ROcp40_919*S20;
    ROcp40_720 = ROcp40_16*S20+ROcp40_719*C20;
    ROcp40_820 = ROcp40_26*S20+ROcp40_819*C20;
    ROcp40_920 = ROcp40_36*S20+ROcp40_919*C20;
    ROcp40_121 = ROcp40_120*C21+ROcp40_419*S21;
    ROcp40_221 = ROcp40_220*C21+ROcp40_519*S21;
    ROcp40_321 = ROcp40_320*C21+ROcp40_619*S21;
    ROcp40_421 = -(ROcp40_120*S21-ROcp40_419*C21);
    ROcp40_521 = -(ROcp40_220*S21-ROcp40_519*C21);
    ROcp40_621 = -(ROcp40_320*S21-ROcp40_619*C21);
    RLcp40_119 = s->dpt[2][3]*ROcp40_46+ROcp40_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp40_219 = s->dpt[2][3]*ROcp40_56+ROcp40_26*s->dpt[1][3]+ROcp40_85*s->dpt[3][3];
    RLcp40_319 = s->dpt[2][3]*ROcp40_66+ROcp40_36*s->dpt[1][3]+ROcp40_95*s->dpt[3][3];
    OMcp40_119 = OMcp40_16+ROcp40_16*qd[19];
    OMcp40_219 = OMcp40_26+ROcp40_26*qd[19];
    OMcp40_319 = OMcp40_36+ROcp40_36*qd[19];
    ORcp40_119 = OMcp40_26*RLcp40_319-OMcp40_36*RLcp40_219;
    ORcp40_219 = -(OMcp40_16*RLcp40_319-OMcp40_36*RLcp40_119);
    ORcp40_319 = OMcp40_16*RLcp40_219-OMcp40_26*RLcp40_119;
    OPcp40_119 = OPcp40_16+ROcp40_16*qdd[19]+qd[19]*(OMcp40_26*ROcp40_36-OMcp40_36*ROcp40_26);
    OPcp40_219 = OPcp40_26+ROcp40_26*qdd[19]-qd[19]*(OMcp40_16*ROcp40_36-OMcp40_36*ROcp40_16);
    OPcp40_319 = OPcp40_36+ROcp40_36*qdd[19]+qd[19]*(OMcp40_16*ROcp40_26-OMcp40_26*ROcp40_16);
    RLcp40_120 = s->dpt[1][38]*ROcp40_16+s->dpt[2][38]*ROcp40_419+s->dpt[3][38]*ROcp40_719;
    RLcp40_220 = s->dpt[1][38]*ROcp40_26+s->dpt[2][38]*ROcp40_519+s->dpt[3][38]*ROcp40_819;
    RLcp40_320 = s->dpt[1][38]*ROcp40_36+s->dpt[2][38]*ROcp40_619+s->dpt[3][38]*ROcp40_919;
    OMcp40_120 = OMcp40_119+ROcp40_419*qd[20];
    OMcp40_220 = OMcp40_219+ROcp40_519*qd[20];
    OMcp40_320 = OMcp40_319+ROcp40_619*qd[20];
    ORcp40_120 = OMcp40_219*RLcp40_320-OMcp40_319*RLcp40_220;
    ORcp40_220 = -(OMcp40_119*RLcp40_320-OMcp40_319*RLcp40_120);
    ORcp40_320 = OMcp40_119*RLcp40_220-OMcp40_219*RLcp40_120;
    OPcp40_120 = OPcp40_119+ROcp40_419*qdd[20]+qd[20]*(OMcp40_219*ROcp40_619-OMcp40_319*ROcp40_519);
    OPcp40_220 = OPcp40_219+ROcp40_519*qdd[20]-qd[20]*(OMcp40_119*ROcp40_619-OMcp40_319*ROcp40_419);
    OPcp40_320 = OPcp40_319+ROcp40_619*qdd[20]+qd[20]*(OMcp40_119*ROcp40_519-OMcp40_219*ROcp40_419);
    RLcp40_121 = s->dpt[1][40]*ROcp40_120+s->dpt[2][40]*ROcp40_419+ROcp40_720*s->dpt[3][40];
    RLcp40_221 = s->dpt[1][40]*ROcp40_220+s->dpt[2][40]*ROcp40_519+ROcp40_820*s->dpt[3][40];
    RLcp40_321 = s->dpt[1][40]*ROcp40_320+s->dpt[2][40]*ROcp40_619+ROcp40_920*s->dpt[3][40];
    OMcp40_121 = OMcp40_120+ROcp40_720*qd[21];
    OMcp40_221 = OMcp40_220+ROcp40_820*qd[21];
    OMcp40_321 = OMcp40_320+ROcp40_920*qd[21];
    ORcp40_121 = OMcp40_220*RLcp40_321-OMcp40_320*RLcp40_221;
    ORcp40_221 = -(OMcp40_120*RLcp40_321-OMcp40_320*RLcp40_121);
    ORcp40_321 = OMcp40_120*RLcp40_221-OMcp40_220*RLcp40_121;
    OPcp40_121 = OPcp40_120+ROcp40_720*qdd[21]+qd[21]*(OMcp40_220*ROcp40_920-OMcp40_320*ROcp40_820);
    OPcp40_221 = OPcp40_220+ROcp40_820*qdd[21]-qd[21]*(OMcp40_120*ROcp40_920-OMcp40_320*ROcp40_720);
    OPcp40_321 = OPcp40_320+ROcp40_920*qdd[21]+qd[21]*(OMcp40_120*ROcp40_820-OMcp40_220*ROcp40_720);

// = = Block_1_0_0_41_0_5 = = 
 
// Sensor Kinematics 


    ROcp40_122 = ROcp40_121*C22-ROcp40_720*S22;
    ROcp40_222 = ROcp40_221*C22-ROcp40_820*S22;
    ROcp40_322 = ROcp40_321*C22-ROcp40_920*S22;
    ROcp40_722 = ROcp40_121*S22+ROcp40_720*C22;
    ROcp40_822 = ROcp40_221*S22+ROcp40_820*C22;
    ROcp40_922 = ROcp40_321*S22+ROcp40_920*C22;
    ROcp40_423 = ROcp40_421*C23+ROcp40_722*S23;
    ROcp40_523 = ROcp40_521*C23+ROcp40_822*S23;
    ROcp40_623 = ROcp40_621*C23+ROcp40_922*S23;
    ROcp40_723 = -(ROcp40_421*S23-ROcp40_722*C23);
    ROcp40_823 = -(ROcp40_521*S23-ROcp40_822*C23);
    ROcp40_923 = -(ROcp40_621*S23-ROcp40_922*C23);
    ROcp40_124 = ROcp40_122*C24-ROcp40_723*S24;
    ROcp40_224 = ROcp40_222*C24-ROcp40_823*S24;
    ROcp40_324 = ROcp40_322*C24-ROcp40_923*S24;
    ROcp40_724 = ROcp40_122*S24+ROcp40_723*C24;
    ROcp40_824 = ROcp40_222*S24+ROcp40_823*C24;
    ROcp40_924 = ROcp40_322*S24+ROcp40_923*C24;
    RLcp40_122 = ROcp40_121*s->dpt[1][44]+ROcp40_421*s->dpt[2][44]+ROcp40_720*s->dpt[3][44];
    RLcp40_222 = ROcp40_221*s->dpt[1][44]+ROcp40_521*s->dpt[2][44]+ROcp40_820*s->dpt[3][44];
    RLcp40_322 = ROcp40_321*s->dpt[1][44]+ROcp40_621*s->dpt[2][44]+ROcp40_920*s->dpt[3][44];
    OMcp40_122 = OMcp40_121+ROcp40_421*qd[22];
    OMcp40_222 = OMcp40_221+ROcp40_521*qd[22];
    OMcp40_322 = OMcp40_321+ROcp40_621*qd[22];
    ORcp40_122 = OMcp40_221*RLcp40_322-OMcp40_321*RLcp40_222;
    ORcp40_222 = -(OMcp40_121*RLcp40_322-OMcp40_321*RLcp40_122);
    ORcp40_322 = OMcp40_121*RLcp40_222-OMcp40_221*RLcp40_122;
    OPcp40_122 = OPcp40_121+ROcp40_421*qdd[22]+qd[22]*(OMcp40_221*ROcp40_621-OMcp40_321*ROcp40_521);
    OPcp40_222 = OPcp40_221+ROcp40_521*qdd[22]-qd[22]*(OMcp40_121*ROcp40_621-OMcp40_321*ROcp40_421);
    OPcp40_322 = OPcp40_321+ROcp40_621*qdd[22]+qd[22]*(OMcp40_121*ROcp40_521-OMcp40_221*ROcp40_421);
    RLcp40_123 = s->dpt[1][48]*ROcp40_122+s->dpt[3][48]*ROcp40_722+ROcp40_421*s->dpt[2][48];
    RLcp40_223 = s->dpt[1][48]*ROcp40_222+s->dpt[3][48]*ROcp40_822+ROcp40_521*s->dpt[2][48];
    RLcp40_323 = s->dpt[1][48]*ROcp40_322+s->dpt[3][48]*ROcp40_922+ROcp40_621*s->dpt[2][48];
    OMcp40_123 = OMcp40_122+ROcp40_122*qd[23];
    OMcp40_223 = OMcp40_222+ROcp40_222*qd[23];
    OMcp40_323 = OMcp40_322+ROcp40_322*qd[23];
    ORcp40_123 = OMcp40_222*RLcp40_323-OMcp40_322*RLcp40_223;
    ORcp40_223 = -(OMcp40_122*RLcp40_323-OMcp40_322*RLcp40_123);
    ORcp40_323 = OMcp40_122*RLcp40_223-OMcp40_222*RLcp40_123;
    OPcp40_123 = OPcp40_122+ROcp40_122*qdd[23]+qd[23]*(OMcp40_222*ROcp40_322-OMcp40_322*ROcp40_222);
    OPcp40_223 = OPcp40_222+ROcp40_222*qdd[23]-qd[23]*(OMcp40_122*ROcp40_322-OMcp40_322*ROcp40_122);
    OPcp40_323 = OPcp40_322+ROcp40_322*qdd[23]+qd[23]*(OMcp40_122*ROcp40_222-OMcp40_222*ROcp40_122);
    RLcp40_124 = s->dpt[1][50]*ROcp40_122+s->dpt[3][50]*ROcp40_723+ROcp40_423*s->dpt[2][50];
    RLcp40_224 = s->dpt[1][50]*ROcp40_222+s->dpt[3][50]*ROcp40_823+ROcp40_523*s->dpt[2][50];
    RLcp40_324 = s->dpt[1][50]*ROcp40_322+s->dpt[3][50]*ROcp40_923+ROcp40_623*s->dpt[2][50];
    OMcp40_124 = OMcp40_123+ROcp40_423*qd[24];
    OMcp40_224 = OMcp40_223+ROcp40_523*qd[24];
    OMcp40_324 = OMcp40_323+ROcp40_623*qd[24];
    ORcp40_124 = OMcp40_223*RLcp40_324-OMcp40_323*RLcp40_224;
    ORcp40_224 = -(OMcp40_123*RLcp40_324-OMcp40_323*RLcp40_124);
    ORcp40_324 = OMcp40_123*RLcp40_224-OMcp40_223*RLcp40_124;
    OPcp40_124 = OPcp40_123+ROcp40_423*qdd[24]+qd[24]*(OMcp40_223*ROcp40_623-OMcp40_323*ROcp40_523);
    OPcp40_224 = OPcp40_223+ROcp40_523*qdd[24]-qd[24]*(OMcp40_123*ROcp40_623-OMcp40_323*ROcp40_423);
    OPcp40_324 = OPcp40_323+ROcp40_623*qdd[24]+qd[24]*(OMcp40_123*ROcp40_523-OMcp40_223*ROcp40_423);
    RLcp40_176 = s->dpt[1][52]*ROcp40_124+s->dpt[3][52]*ROcp40_724+ROcp40_423*s->dpt[2][52];
    RLcp40_276 = s->dpt[1][52]*ROcp40_224+s->dpt[3][52]*ROcp40_824+ROcp40_523*s->dpt[2][52];
    RLcp40_376 = s->dpt[1][52]*ROcp40_324+s->dpt[3][52]*ROcp40_924+ROcp40_623*s->dpt[2][52];
    POcp40_176 = RLcp40_119+RLcp40_120+RLcp40_121+RLcp40_122+RLcp40_123+RLcp40_124+RLcp40_176+q[1];
    POcp40_276 = RLcp40_219+RLcp40_220+RLcp40_221+RLcp40_222+RLcp40_223+RLcp40_224+RLcp40_276+q[2];
    POcp40_376 = RLcp40_319+RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_376+q[3];
    JTcp40_276_4 = -(RLcp40_319+RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_376);
    JTcp40_376_4 = RLcp40_219+RLcp40_220+RLcp40_221+RLcp40_222+RLcp40_223+RLcp40_224+RLcp40_276;
    JTcp40_176_5 = C4*(RLcp40_319+RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324)-S4*(RLcp40_219+RLcp40_220)-S4*(
 RLcp40_221+RLcp40_222)-S4*(RLcp40_223+RLcp40_224)-RLcp40_276*S4+RLcp40_376*C4;
    JTcp40_276_5 = S4*(RLcp40_119+RLcp40_120+RLcp40_121+RLcp40_122+RLcp40_123+RLcp40_124+RLcp40_176);
    JTcp40_376_5 = -C4*(RLcp40_119+RLcp40_120+RLcp40_121+RLcp40_122+RLcp40_123+RLcp40_124+RLcp40_176);
    JTcp40_176_6 = ROcp40_85*(RLcp40_319+RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324)-ROcp40_95*(RLcp40_219+
 RLcp40_220)-ROcp40_95*(RLcp40_221+RLcp40_222)-ROcp40_95*(RLcp40_223+RLcp40_224)-RLcp40_276*ROcp40_95+RLcp40_376*ROcp40_85;
    JTcp40_276_6 = -(RLcp40_376*S5-ROcp40_95*(RLcp40_119+RLcp40_120+RLcp40_121+RLcp40_122+RLcp40_123+RLcp40_124+RLcp40_176
 )+S5*(RLcp40_319+RLcp40_320)+S5*(RLcp40_321+RLcp40_322)+S5*(RLcp40_323+RLcp40_324));
    JTcp40_376_6 = RLcp40_276*S5-ROcp40_85*(RLcp40_119+RLcp40_120+RLcp40_121+RLcp40_122+RLcp40_123+RLcp40_124+RLcp40_176)+
 S5*(RLcp40_219+RLcp40_220)+S5*(RLcp40_221+RLcp40_222)+S5*(RLcp40_223+RLcp40_224);
    JTcp40_176_7 = ROcp40_26*(RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_376)-ROcp40_36*(RLcp40_220+
 RLcp40_221)-ROcp40_36*(RLcp40_222+RLcp40_223)-ROcp40_36*(RLcp40_224+RLcp40_276);
    JTcp40_276_7 = -(ROcp40_16*(RLcp40_320+RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_376)-ROcp40_36*(RLcp40_120+
 RLcp40_121)-ROcp40_36*(RLcp40_122+RLcp40_123)-ROcp40_36*(RLcp40_124+RLcp40_176));
    JTcp40_376_7 = ROcp40_16*(RLcp40_220+RLcp40_221+RLcp40_222+RLcp40_223+RLcp40_224+RLcp40_276)-ROcp40_26*(RLcp40_120+
 RLcp40_121)-ROcp40_26*(RLcp40_122+RLcp40_123)-ROcp40_26*(RLcp40_124+RLcp40_176);
    JTcp40_176_8 = ROcp40_519*(RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324)-ROcp40_619*(RLcp40_221+RLcp40_222)-ROcp40_619*
 (RLcp40_223+RLcp40_224)-RLcp40_276*ROcp40_619+RLcp40_376*ROcp40_519;
    JTcp40_276_8 = RLcp40_176*ROcp40_619-RLcp40_376*ROcp40_419-ROcp40_419*(RLcp40_321+RLcp40_322+RLcp40_323+RLcp40_324)+
 ROcp40_619*(RLcp40_121+RLcp40_122)+ROcp40_619*(RLcp40_123+RLcp40_124);
    JTcp40_376_8 = ROcp40_419*(RLcp40_221+RLcp40_222+RLcp40_223+RLcp40_224)-ROcp40_519*(RLcp40_121+RLcp40_122)-ROcp40_519*
 (RLcp40_123+RLcp40_124)-RLcp40_176*ROcp40_519+RLcp40_276*ROcp40_419;
    JTcp40_176_9 = ROcp40_820*(RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_376)-ROcp40_920*(RLcp40_222+RLcp40_223)-ROcp40_920*
 (RLcp40_224+RLcp40_276);
    JTcp40_276_9 = -(ROcp40_720*(RLcp40_322+RLcp40_323+RLcp40_324+RLcp40_376)-ROcp40_920*(RLcp40_122+RLcp40_123)-
 ROcp40_920*(RLcp40_124+RLcp40_176));
    JTcp40_376_9 = ROcp40_720*(RLcp40_222+RLcp40_223+RLcp40_224+RLcp40_276)-ROcp40_820*(RLcp40_122+RLcp40_123)-ROcp40_820*
 (RLcp40_124+RLcp40_176);
    JTcp40_176_10 = ROcp40_521*(RLcp40_323+RLcp40_324)-ROcp40_621*(RLcp40_223+RLcp40_224)-RLcp40_276*ROcp40_621+RLcp40_376
 *ROcp40_521;
    JTcp40_276_10 = RLcp40_176*ROcp40_621-RLcp40_376*ROcp40_421-ROcp40_421*(RLcp40_323+RLcp40_324)+ROcp40_621*(RLcp40_123+
 RLcp40_124);
    JTcp40_376_10 = ROcp40_421*(RLcp40_223+RLcp40_224)-ROcp40_521*(RLcp40_123+RLcp40_124)-RLcp40_176*ROcp40_521+RLcp40_276
 *ROcp40_421;
    JTcp40_176_11 = ROcp40_222*(RLcp40_324+RLcp40_376)-ROcp40_322*(RLcp40_224+RLcp40_276);
    JTcp40_276_11 = -(ROcp40_122*(RLcp40_324+RLcp40_376)-ROcp40_322*(RLcp40_124+RLcp40_176));
    JTcp40_376_11 = ROcp40_122*(RLcp40_224+RLcp40_276)-ROcp40_222*(RLcp40_124+RLcp40_176);
    JTcp40_176_12 = -(RLcp40_276*ROcp40_623-RLcp40_376*ROcp40_523);
    JTcp40_276_12 = RLcp40_176*ROcp40_623-RLcp40_376*ROcp40_423;
    JTcp40_376_12 = -(RLcp40_176*ROcp40_523-RLcp40_276*ROcp40_423);
    ORcp40_176 = OMcp40_224*RLcp40_376-OMcp40_324*RLcp40_276;
    ORcp40_276 = -(OMcp40_124*RLcp40_376-OMcp40_324*RLcp40_176);
    ORcp40_376 = OMcp40_124*RLcp40_276-OMcp40_224*RLcp40_176;
    VIcp40_176 = ORcp40_119+ORcp40_120+ORcp40_121+ORcp40_122+ORcp40_123+ORcp40_124+ORcp40_176+qd[1];
    VIcp40_276 = ORcp40_219+ORcp40_220+ORcp40_221+ORcp40_222+ORcp40_223+ORcp40_224+ORcp40_276+qd[2];
    VIcp40_376 = ORcp40_319+ORcp40_320+ORcp40_321+ORcp40_322+ORcp40_323+ORcp40_324+ORcp40_376+qd[3];
    ACcp40_176 = qdd[1]+OMcp40_219*ORcp40_320+OMcp40_220*ORcp40_321+OMcp40_221*ORcp40_322+OMcp40_222*ORcp40_323+OMcp40_223
 *ORcp40_324+OMcp40_224*ORcp40_376+OMcp40_26*ORcp40_319-OMcp40_319*ORcp40_220-OMcp40_320*ORcp40_221-OMcp40_321*ORcp40_222-
 OMcp40_322*ORcp40_223-OMcp40_323*ORcp40_224-OMcp40_324*ORcp40_276-OMcp40_36*ORcp40_219+OPcp40_219*RLcp40_320+OPcp40_220*
 RLcp40_321+OPcp40_221*RLcp40_322+OPcp40_222*RLcp40_323+OPcp40_223*RLcp40_324+OPcp40_224*RLcp40_376+OPcp40_26*RLcp40_319-
 OPcp40_319*RLcp40_220-OPcp40_320*RLcp40_221-OPcp40_321*RLcp40_222-OPcp40_322*RLcp40_223-OPcp40_323*RLcp40_224-OPcp40_324*
 RLcp40_276-OPcp40_36*RLcp40_219;
    ACcp40_276 = qdd[2]-OMcp40_119*ORcp40_320-OMcp40_120*ORcp40_321-OMcp40_121*ORcp40_322-OMcp40_122*ORcp40_323-OMcp40_123
 *ORcp40_324-OMcp40_124*ORcp40_376-OMcp40_16*ORcp40_319+OMcp40_319*ORcp40_120+OMcp40_320*ORcp40_121+OMcp40_321*ORcp40_122+
 OMcp40_322*ORcp40_123+OMcp40_323*ORcp40_124+OMcp40_324*ORcp40_176+OMcp40_36*ORcp40_119-OPcp40_119*RLcp40_320-OPcp40_120*
 RLcp40_321-OPcp40_121*RLcp40_322-OPcp40_122*RLcp40_323-OPcp40_123*RLcp40_324-OPcp40_124*RLcp40_376-OPcp40_16*RLcp40_319+
 OPcp40_319*RLcp40_120+OPcp40_320*RLcp40_121+OPcp40_321*RLcp40_122+OPcp40_322*RLcp40_123+OPcp40_323*RLcp40_124+OPcp40_324*
 RLcp40_176+OPcp40_36*RLcp40_119;
    ACcp40_376 = qdd[3]+OMcp40_119*ORcp40_220+OMcp40_120*ORcp40_221+OMcp40_121*ORcp40_222+OMcp40_122*ORcp40_223+OMcp40_123
 *ORcp40_224+OMcp40_124*ORcp40_276+OMcp40_16*ORcp40_219-OMcp40_219*ORcp40_120-OMcp40_220*ORcp40_121-OMcp40_221*ORcp40_122-
 OMcp40_222*ORcp40_123-OMcp40_223*ORcp40_124-OMcp40_224*ORcp40_176-OMcp40_26*ORcp40_119+OPcp40_119*RLcp40_220+OPcp40_120*
 RLcp40_221+OPcp40_121*RLcp40_222+OPcp40_122*RLcp40_223+OPcp40_123*RLcp40_224+OPcp40_124*RLcp40_276+OPcp40_16*RLcp40_219-
 OPcp40_219*RLcp40_120-OPcp40_220*RLcp40_121-OPcp40_221*RLcp40_122-OPcp40_222*RLcp40_123-OPcp40_223*RLcp40_124-OPcp40_224*
 RLcp40_176-OPcp40_26*RLcp40_119;

// = = Block_1_0_0_41_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp40_176;
    sens->P[2] = POcp40_276;
    sens->P[3] = POcp40_376;
    sens->R[1][1] = ROcp40_124;
    sens->R[1][2] = ROcp40_224;
    sens->R[1][3] = ROcp40_324;
    sens->R[2][1] = ROcp40_423;
    sens->R[2][2] = ROcp40_523;
    sens->R[2][3] = ROcp40_623;
    sens->R[3][1] = ROcp40_724;
    sens->R[3][2] = ROcp40_824;
    sens->R[3][3] = ROcp40_924;
    sens->V[1] = VIcp40_176;
    sens->V[2] = VIcp40_276;
    sens->V[3] = VIcp40_376;
    sens->OM[1] = OMcp40_124;
    sens->OM[2] = OMcp40_224;
    sens->OM[3] = OMcp40_324;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp40_176_5;
    sens->J[1][6] = JTcp40_176_6;
    sens->J[1][19] = JTcp40_176_7;
    sens->J[1][20] = JTcp40_176_8;
    sens->J[1][21] = JTcp40_176_9;
    sens->J[1][22] = JTcp40_176_10;
    sens->J[1][23] = JTcp40_176_11;
    sens->J[1][24] = JTcp40_176_12;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp40_276_4;
    sens->J[2][5] = JTcp40_276_5;
    sens->J[2][6] = JTcp40_276_6;
    sens->J[2][19] = JTcp40_276_7;
    sens->J[2][20] = JTcp40_276_8;
    sens->J[2][21] = JTcp40_276_9;
    sens->J[2][22] = JTcp40_276_10;
    sens->J[2][23] = JTcp40_276_11;
    sens->J[2][24] = JTcp40_276_12;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp40_376_4;
    sens->J[3][5] = JTcp40_376_5;
    sens->J[3][6] = JTcp40_376_6;
    sens->J[3][19] = JTcp40_376_7;
    sens->J[3][20] = JTcp40_376_8;
    sens->J[3][21] = JTcp40_376_9;
    sens->J[3][22] = JTcp40_376_10;
    sens->J[3][23] = JTcp40_376_11;
    sens->J[3][24] = JTcp40_376_12;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp40_16;
    sens->J[4][20] = ROcp40_419;
    sens->J[4][21] = ROcp40_720;
    sens->J[4][22] = ROcp40_421;
    sens->J[4][23] = ROcp40_122;
    sens->J[4][24] = ROcp40_423;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp40_85;
    sens->J[5][19] = ROcp40_26;
    sens->J[5][20] = ROcp40_519;
    sens->J[5][21] = ROcp40_820;
    sens->J[5][22] = ROcp40_521;
    sens->J[5][23] = ROcp40_222;
    sens->J[5][24] = ROcp40_523;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp40_95;
    sens->J[6][19] = ROcp40_36;
    sens->J[6][20] = ROcp40_619;
    sens->J[6][21] = ROcp40_920;
    sens->J[6][22] = ROcp40_621;
    sens->J[6][23] = ROcp40_322;
    sens->J[6][24] = ROcp40_623;
    sens->A[1] = ACcp40_176;
    sens->A[2] = ACcp40_276;
    sens->A[3] = ACcp40_376;
    sens->OMP[1] = OPcp40_124;
    sens->OMP[2] = OPcp40_224;
    sens->OMP[3] = OPcp40_324;
 
// 
break;
case 42:
 


// = = Block_1_0_0_42_0_1 = = 
 
// Sensor Kinematics 


    ROcp41_25 = S4*S5;
    ROcp41_35 = -C4*S5;
    ROcp41_85 = -S4*C5;
    ROcp41_95 = C4*C5;
    ROcp41_16 = C5*C6;
    ROcp41_26 = ROcp41_25*C6+C4*S6;
    ROcp41_36 = ROcp41_35*C6+S4*S6;
    ROcp41_46 = -C5*S6;
    ROcp41_56 = -(ROcp41_25*S6-C4*C6);
    ROcp41_66 = -(ROcp41_35*S6-S4*C6);
    OMcp41_25 = qd[5]*C4;
    OMcp41_35 = qd[5]*S4;
    OMcp41_16 = qd[4]+qd[6]*S5;
    OMcp41_26 = OMcp41_25+ROcp41_85*qd[6];
    OMcp41_36 = OMcp41_35+ROcp41_95*qd[6];
    OPcp41_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp41_26 = ROcp41_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp41_35*S5-ROcp41_95*qd[4]);
    OPcp41_36 = ROcp41_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp41_25*S5-ROcp41_85*qd[4]);

// = = Block_1_0_0_42_0_4 = = 
 
// Sensor Kinematics 


    ROcp41_419 = ROcp41_46*C19+S19*S5;
    ROcp41_519 = ROcp41_56*C19+ROcp41_85*S19;
    ROcp41_619 = ROcp41_66*C19+ROcp41_95*S19;
    ROcp41_719 = -(ROcp41_46*S19-C19*S5);
    ROcp41_819 = -(ROcp41_56*S19-ROcp41_85*C19);
    ROcp41_919 = -(ROcp41_66*S19-ROcp41_95*C19);
    ROcp41_120 = ROcp41_16*C20-ROcp41_719*S20;
    ROcp41_220 = ROcp41_26*C20-ROcp41_819*S20;
    ROcp41_320 = ROcp41_36*C20-ROcp41_919*S20;
    ROcp41_720 = ROcp41_16*S20+ROcp41_719*C20;
    ROcp41_820 = ROcp41_26*S20+ROcp41_819*C20;
    ROcp41_920 = ROcp41_36*S20+ROcp41_919*C20;
    ROcp41_121 = ROcp41_120*C21+ROcp41_419*S21;
    ROcp41_221 = ROcp41_220*C21+ROcp41_519*S21;
    ROcp41_321 = ROcp41_320*C21+ROcp41_619*S21;
    ROcp41_421 = -(ROcp41_120*S21-ROcp41_419*C21);
    ROcp41_521 = -(ROcp41_220*S21-ROcp41_519*C21);
    ROcp41_621 = -(ROcp41_320*S21-ROcp41_619*C21);
    RLcp41_119 = s->dpt[2][3]*ROcp41_46+ROcp41_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp41_219 = s->dpt[2][3]*ROcp41_56+ROcp41_26*s->dpt[1][3]+ROcp41_85*s->dpt[3][3];
    RLcp41_319 = s->dpt[2][3]*ROcp41_66+ROcp41_36*s->dpt[1][3]+ROcp41_95*s->dpt[3][3];
    OMcp41_119 = OMcp41_16+ROcp41_16*qd[19];
    OMcp41_219 = OMcp41_26+ROcp41_26*qd[19];
    OMcp41_319 = OMcp41_36+ROcp41_36*qd[19];
    ORcp41_119 = OMcp41_26*RLcp41_319-OMcp41_36*RLcp41_219;
    ORcp41_219 = -(OMcp41_16*RLcp41_319-OMcp41_36*RLcp41_119);
    ORcp41_319 = OMcp41_16*RLcp41_219-OMcp41_26*RLcp41_119;
    OPcp41_119 = OPcp41_16+ROcp41_16*qdd[19]+qd[19]*(OMcp41_26*ROcp41_36-OMcp41_36*ROcp41_26);
    OPcp41_219 = OPcp41_26+ROcp41_26*qdd[19]-qd[19]*(OMcp41_16*ROcp41_36-OMcp41_36*ROcp41_16);
    OPcp41_319 = OPcp41_36+ROcp41_36*qdd[19]+qd[19]*(OMcp41_16*ROcp41_26-OMcp41_26*ROcp41_16);
    RLcp41_120 = s->dpt[1][38]*ROcp41_16+s->dpt[2][38]*ROcp41_419+s->dpt[3][38]*ROcp41_719;
    RLcp41_220 = s->dpt[1][38]*ROcp41_26+s->dpt[2][38]*ROcp41_519+s->dpt[3][38]*ROcp41_819;
    RLcp41_320 = s->dpt[1][38]*ROcp41_36+s->dpt[2][38]*ROcp41_619+s->dpt[3][38]*ROcp41_919;
    OMcp41_120 = OMcp41_119+ROcp41_419*qd[20];
    OMcp41_220 = OMcp41_219+ROcp41_519*qd[20];
    OMcp41_320 = OMcp41_319+ROcp41_619*qd[20];
    ORcp41_120 = OMcp41_219*RLcp41_320-OMcp41_319*RLcp41_220;
    ORcp41_220 = -(OMcp41_119*RLcp41_320-OMcp41_319*RLcp41_120);
    ORcp41_320 = OMcp41_119*RLcp41_220-OMcp41_219*RLcp41_120;
    OPcp41_120 = OPcp41_119+ROcp41_419*qdd[20]+qd[20]*(OMcp41_219*ROcp41_619-OMcp41_319*ROcp41_519);
    OPcp41_220 = OPcp41_219+ROcp41_519*qdd[20]-qd[20]*(OMcp41_119*ROcp41_619-OMcp41_319*ROcp41_419);
    OPcp41_320 = OPcp41_319+ROcp41_619*qdd[20]+qd[20]*(OMcp41_119*ROcp41_519-OMcp41_219*ROcp41_419);
    RLcp41_121 = s->dpt[1][40]*ROcp41_120+s->dpt[2][40]*ROcp41_419+ROcp41_720*s->dpt[3][40];
    RLcp41_221 = s->dpt[1][40]*ROcp41_220+s->dpt[2][40]*ROcp41_519+ROcp41_820*s->dpt[3][40];
    RLcp41_321 = s->dpt[1][40]*ROcp41_320+s->dpt[2][40]*ROcp41_619+ROcp41_920*s->dpt[3][40];
    OMcp41_121 = OMcp41_120+ROcp41_720*qd[21];
    OMcp41_221 = OMcp41_220+ROcp41_820*qd[21];
    OMcp41_321 = OMcp41_320+ROcp41_920*qd[21];
    ORcp41_121 = OMcp41_220*RLcp41_321-OMcp41_320*RLcp41_221;
    ORcp41_221 = -(OMcp41_120*RLcp41_321-OMcp41_320*RLcp41_121);
    ORcp41_321 = OMcp41_120*RLcp41_221-OMcp41_220*RLcp41_121;
    OPcp41_121 = OPcp41_120+ROcp41_720*qdd[21]+qd[21]*(OMcp41_220*ROcp41_920-OMcp41_320*ROcp41_820);
    OPcp41_221 = OPcp41_220+ROcp41_820*qdd[21]-qd[21]*(OMcp41_120*ROcp41_920-OMcp41_320*ROcp41_720);
    OPcp41_321 = OPcp41_320+ROcp41_920*qdd[21]+qd[21]*(OMcp41_120*ROcp41_820-OMcp41_220*ROcp41_720);

// = = Block_1_0_0_42_0_5 = = 
 
// Sensor Kinematics 


    ROcp41_122 = ROcp41_121*C22-ROcp41_720*S22;
    ROcp41_222 = ROcp41_221*C22-ROcp41_820*S22;
    ROcp41_322 = ROcp41_321*C22-ROcp41_920*S22;
    ROcp41_722 = ROcp41_121*S22+ROcp41_720*C22;
    ROcp41_822 = ROcp41_221*S22+ROcp41_820*C22;
    ROcp41_922 = ROcp41_321*S22+ROcp41_920*C22;
    ROcp41_423 = ROcp41_421*C23+ROcp41_722*S23;
    ROcp41_523 = ROcp41_521*C23+ROcp41_822*S23;
    ROcp41_623 = ROcp41_621*C23+ROcp41_922*S23;
    ROcp41_723 = -(ROcp41_421*S23-ROcp41_722*C23);
    ROcp41_823 = -(ROcp41_521*S23-ROcp41_822*C23);
    ROcp41_923 = -(ROcp41_621*S23-ROcp41_922*C23);
    ROcp41_124 = ROcp41_122*C24-ROcp41_723*S24;
    ROcp41_224 = ROcp41_222*C24-ROcp41_823*S24;
    ROcp41_324 = ROcp41_322*C24-ROcp41_923*S24;
    ROcp41_724 = ROcp41_122*S24+ROcp41_723*C24;
    ROcp41_824 = ROcp41_222*S24+ROcp41_823*C24;
    ROcp41_924 = ROcp41_322*S24+ROcp41_923*C24;
    ROcp41_125 = ROcp41_124*C25+ROcp41_423*S25;
    ROcp41_225 = ROcp41_224*C25+ROcp41_523*S25;
    ROcp41_325 = ROcp41_324*C25+ROcp41_623*S25;
    ROcp41_425 = -(ROcp41_124*S25-ROcp41_423*C25);
    ROcp41_525 = -(ROcp41_224*S25-ROcp41_523*C25);
    ROcp41_625 = -(ROcp41_324*S25-ROcp41_623*C25);
    RLcp41_122 = ROcp41_121*s->dpt[1][44]+ROcp41_421*s->dpt[2][44]+ROcp41_720*s->dpt[3][44];
    RLcp41_222 = ROcp41_221*s->dpt[1][44]+ROcp41_521*s->dpt[2][44]+ROcp41_820*s->dpt[3][44];
    RLcp41_322 = ROcp41_321*s->dpt[1][44]+ROcp41_621*s->dpt[2][44]+ROcp41_920*s->dpt[3][44];
    OMcp41_122 = OMcp41_121+ROcp41_421*qd[22];
    OMcp41_222 = OMcp41_221+ROcp41_521*qd[22];
    OMcp41_322 = OMcp41_321+ROcp41_621*qd[22];
    ORcp41_122 = OMcp41_221*RLcp41_322-OMcp41_321*RLcp41_222;
    ORcp41_222 = -(OMcp41_121*RLcp41_322-OMcp41_321*RLcp41_122);
    ORcp41_322 = OMcp41_121*RLcp41_222-OMcp41_221*RLcp41_122;
    OPcp41_122 = OPcp41_121+ROcp41_421*qdd[22]+qd[22]*(OMcp41_221*ROcp41_621-OMcp41_321*ROcp41_521);
    OPcp41_222 = OPcp41_221+ROcp41_521*qdd[22]-qd[22]*(OMcp41_121*ROcp41_621-OMcp41_321*ROcp41_421);
    OPcp41_322 = OPcp41_321+ROcp41_621*qdd[22]+qd[22]*(OMcp41_121*ROcp41_521-OMcp41_221*ROcp41_421);
    RLcp41_123 = s->dpt[1][48]*ROcp41_122+s->dpt[3][48]*ROcp41_722+ROcp41_421*s->dpt[2][48];
    RLcp41_223 = s->dpt[1][48]*ROcp41_222+s->dpt[3][48]*ROcp41_822+ROcp41_521*s->dpt[2][48];
    RLcp41_323 = s->dpt[1][48]*ROcp41_322+s->dpt[3][48]*ROcp41_922+ROcp41_621*s->dpt[2][48];
    OMcp41_123 = OMcp41_122+ROcp41_122*qd[23];
    OMcp41_223 = OMcp41_222+ROcp41_222*qd[23];
    OMcp41_323 = OMcp41_322+ROcp41_322*qd[23];
    ORcp41_123 = OMcp41_222*RLcp41_323-OMcp41_322*RLcp41_223;
    ORcp41_223 = -(OMcp41_122*RLcp41_323-OMcp41_322*RLcp41_123);
    ORcp41_323 = OMcp41_122*RLcp41_223-OMcp41_222*RLcp41_123;
    OPcp41_123 = OPcp41_122+ROcp41_122*qdd[23]+qd[23]*(OMcp41_222*ROcp41_322-OMcp41_322*ROcp41_222);
    OPcp41_223 = OPcp41_222+ROcp41_222*qdd[23]-qd[23]*(OMcp41_122*ROcp41_322-OMcp41_322*ROcp41_122);
    OPcp41_323 = OPcp41_322+ROcp41_322*qdd[23]+qd[23]*(OMcp41_122*ROcp41_222-OMcp41_222*ROcp41_122);
    RLcp41_124 = s->dpt[1][50]*ROcp41_122+s->dpt[3][50]*ROcp41_723+ROcp41_423*s->dpt[2][50];
    RLcp41_224 = s->dpt[1][50]*ROcp41_222+s->dpt[3][50]*ROcp41_823+ROcp41_523*s->dpt[2][50];
    RLcp41_324 = s->dpt[1][50]*ROcp41_322+s->dpt[3][50]*ROcp41_923+ROcp41_623*s->dpt[2][50];
    OMcp41_124 = OMcp41_123+ROcp41_423*qd[24];
    OMcp41_224 = OMcp41_223+ROcp41_523*qd[24];
    OMcp41_324 = OMcp41_323+ROcp41_623*qd[24];
    ORcp41_124 = OMcp41_223*RLcp41_324-OMcp41_323*RLcp41_224;
    ORcp41_224 = -(OMcp41_123*RLcp41_324-OMcp41_323*RLcp41_124);
    ORcp41_324 = OMcp41_123*RLcp41_224-OMcp41_223*RLcp41_124;
    OPcp41_124 = OPcp41_123+ROcp41_423*qdd[24]+qd[24]*(OMcp41_223*ROcp41_623-OMcp41_323*ROcp41_523);
    OPcp41_224 = OPcp41_223+ROcp41_523*qdd[24]-qd[24]*(OMcp41_123*ROcp41_623-OMcp41_323*ROcp41_423);
    OPcp41_324 = OPcp41_323+ROcp41_623*qdd[24]+qd[24]*(OMcp41_123*ROcp41_523-OMcp41_223*ROcp41_423);
    RLcp41_125 = s->dpt[1][52]*ROcp41_124+s->dpt[3][52]*ROcp41_724+ROcp41_423*s->dpt[2][52];
    RLcp41_225 = s->dpt[1][52]*ROcp41_224+s->dpt[3][52]*ROcp41_824+ROcp41_523*s->dpt[2][52];
    RLcp41_325 = s->dpt[1][52]*ROcp41_324+s->dpt[3][52]*ROcp41_924+ROcp41_623*s->dpt[2][52];
    OMcp41_125 = OMcp41_124+ROcp41_724*qd[25];
    OMcp41_225 = OMcp41_224+ROcp41_824*qd[25];
    OMcp41_325 = OMcp41_324+ROcp41_924*qd[25];
    ORcp41_125 = OMcp41_224*RLcp41_325-OMcp41_324*RLcp41_225;
    ORcp41_225 = -(OMcp41_124*RLcp41_325-OMcp41_324*RLcp41_125);
    ORcp41_325 = OMcp41_124*RLcp41_225-OMcp41_224*RLcp41_125;
    OPcp41_125 = OPcp41_124+ROcp41_724*qdd[25]+qd[25]*(OMcp41_224*ROcp41_924-OMcp41_324*ROcp41_824);
    OPcp41_225 = OPcp41_224+ROcp41_824*qdd[25]-qd[25]*(OMcp41_124*ROcp41_924-OMcp41_324*ROcp41_724);
    OPcp41_325 = OPcp41_324+ROcp41_924*qdd[25]+qd[25]*(OMcp41_124*ROcp41_824-OMcp41_224*ROcp41_724);
    RLcp41_177 = ROcp41_125*s->dpt[1][53]+ROcp41_425*s->dpt[2][53]+ROcp41_724*s->dpt[3][53];
    RLcp41_277 = ROcp41_225*s->dpt[1][53]+ROcp41_525*s->dpt[2][53]+ROcp41_824*s->dpt[3][53];
    RLcp41_377 = ROcp41_325*s->dpt[1][53]+ROcp41_625*s->dpt[2][53]+ROcp41_924*s->dpt[3][53];
    POcp41_177 = RLcp41_119+RLcp41_120+RLcp41_121+RLcp41_122+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_177+q[1];
    POcp41_277 = RLcp41_219+RLcp41_220+RLcp41_221+RLcp41_222+RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_277+q[2];
    POcp41_377 = RLcp41_319+RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_377+q[3];
    JTcp41_277_4 = -(RLcp41_319+RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_377);
    JTcp41_377_4 = RLcp41_219+RLcp41_220+RLcp41_221+RLcp41_222+RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_277;
    JTcp41_177_5 = C4*(RLcp41_319+RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_377)-S4*(
 RLcp41_219+RLcp41_220)-S4*(RLcp41_221+RLcp41_222)-S4*(RLcp41_223+RLcp41_224)-S4*(RLcp41_225+RLcp41_277);
    JTcp41_277_5 = S4*(RLcp41_119+RLcp41_120+RLcp41_121+RLcp41_122+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_177);
    JTcp41_377_5 = -C4*(RLcp41_119+RLcp41_120+RLcp41_121+RLcp41_122+RLcp41_123+RLcp41_124+RLcp41_125+RLcp41_177);
    JTcp41_177_6 = ROcp41_85*(RLcp41_319+RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_377)-
 ROcp41_95*(RLcp41_219+RLcp41_220)-ROcp41_95*(RLcp41_221+RLcp41_222)-ROcp41_95*(RLcp41_223+RLcp41_224)-ROcp41_95*(RLcp41_225+
 RLcp41_277);
    JTcp41_277_6 = RLcp41_177*ROcp41_95-RLcp41_325*S5-RLcp41_377*S5+ROcp41_95*(RLcp41_119+RLcp41_120+RLcp41_121+RLcp41_122
 +RLcp41_123+RLcp41_124+RLcp41_125)-S5*(RLcp41_319+RLcp41_320)-S5*(RLcp41_321+RLcp41_322)-S5*(RLcp41_323+RLcp41_324);
    JTcp41_377_6 = RLcp41_225*S5-ROcp41_85*(RLcp41_119+RLcp41_120+RLcp41_121+RLcp41_122+RLcp41_123+RLcp41_124+RLcp41_125)+
 S5*(RLcp41_219+RLcp41_220)+S5*(RLcp41_221+RLcp41_222)+S5*(RLcp41_223+RLcp41_224)-RLcp41_177*ROcp41_85+RLcp41_277*S5;
    JTcp41_177_7 = ROcp41_26*(RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325)-ROcp41_36*(RLcp41_220+
 RLcp41_221)-ROcp41_36*(RLcp41_222+RLcp41_223)-ROcp41_36*(RLcp41_224+RLcp41_225)-RLcp41_277*ROcp41_36+RLcp41_377*ROcp41_26;
    JTcp41_277_7 = RLcp41_177*ROcp41_36-RLcp41_377*ROcp41_16-ROcp41_16*(RLcp41_320+RLcp41_321+RLcp41_322+RLcp41_323+
 RLcp41_324+RLcp41_325)+ROcp41_36*(RLcp41_120+RLcp41_121)+ROcp41_36*(RLcp41_122+RLcp41_123)+ROcp41_36*(RLcp41_124+RLcp41_125);
    JTcp41_377_7 = ROcp41_16*(RLcp41_220+RLcp41_221+RLcp41_222+RLcp41_223+RLcp41_224+RLcp41_225)-ROcp41_26*(RLcp41_120+
 RLcp41_121)-ROcp41_26*(RLcp41_122+RLcp41_123)-ROcp41_26*(RLcp41_124+RLcp41_125)-RLcp41_177*ROcp41_26+RLcp41_277*ROcp41_16;
    JTcp41_177_8 = ROcp41_519*(RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_377)-ROcp41_619*(RLcp41_221+
 RLcp41_222)-ROcp41_619*(RLcp41_223+RLcp41_224)-ROcp41_619*(RLcp41_225+RLcp41_277);
    JTcp41_277_8 = -(ROcp41_419*(RLcp41_321+RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_377)-ROcp41_619*(RLcp41_121
 +RLcp41_122)-ROcp41_619*(RLcp41_123+RLcp41_124)-ROcp41_619*(RLcp41_125+RLcp41_177));
    JTcp41_377_8 = ROcp41_419*(RLcp41_221+RLcp41_222+RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_277)-ROcp41_519*(RLcp41_121+
 RLcp41_122)-ROcp41_519*(RLcp41_123+RLcp41_124)-ROcp41_519*(RLcp41_125+RLcp41_177);
    JTcp41_177_9 = ROcp41_820*(RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325)-ROcp41_920*(RLcp41_222+RLcp41_223)-ROcp41_920*
 (RLcp41_224+RLcp41_225)-RLcp41_277*ROcp41_920+RLcp41_377*ROcp41_820;
    JTcp41_277_9 = RLcp41_177*ROcp41_920-RLcp41_377*ROcp41_720-ROcp41_720*(RLcp41_322+RLcp41_323+RLcp41_324+RLcp41_325)+
 ROcp41_920*(RLcp41_122+RLcp41_123)+ROcp41_920*(RLcp41_124+RLcp41_125);
    JTcp41_377_9 = ROcp41_720*(RLcp41_222+RLcp41_223+RLcp41_224+RLcp41_225)-ROcp41_820*(RLcp41_122+RLcp41_123)-ROcp41_820*
 (RLcp41_124+RLcp41_125)-RLcp41_177*ROcp41_820+RLcp41_277*ROcp41_720;
    JTcp41_177_10 = ROcp41_521*(RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_377)-ROcp41_621*(RLcp41_223+RLcp41_224)-ROcp41_621
 *(RLcp41_225+RLcp41_277);
    JTcp41_277_10 = -(ROcp41_421*(RLcp41_323+RLcp41_324+RLcp41_325+RLcp41_377)-ROcp41_621*(RLcp41_123+RLcp41_124)-
 ROcp41_621*(RLcp41_125+RLcp41_177));
    JTcp41_377_10 = ROcp41_421*(RLcp41_223+RLcp41_224+RLcp41_225+RLcp41_277)-ROcp41_521*(RLcp41_123+RLcp41_124)-ROcp41_521
 *(RLcp41_125+RLcp41_177);
    JTcp41_177_11 = ROcp41_222*(RLcp41_324+RLcp41_325)-ROcp41_322*(RLcp41_224+RLcp41_225)-RLcp41_277*ROcp41_322+RLcp41_377
 *ROcp41_222;
    JTcp41_277_11 = RLcp41_177*ROcp41_322-RLcp41_377*ROcp41_122-ROcp41_122*(RLcp41_324+RLcp41_325)+ROcp41_322*(RLcp41_124+
 RLcp41_125);
    JTcp41_377_11 = ROcp41_122*(RLcp41_224+RLcp41_225)-ROcp41_222*(RLcp41_124+RLcp41_125)-RLcp41_177*ROcp41_222+RLcp41_277
 *ROcp41_122;
    JTcp41_177_12 = ROcp41_523*(RLcp41_325+RLcp41_377)-ROcp41_623*(RLcp41_225+RLcp41_277);
    JTcp41_277_12 = -(ROcp41_423*(RLcp41_325+RLcp41_377)-ROcp41_623*(RLcp41_125+RLcp41_177));
    JTcp41_377_12 = ROcp41_423*(RLcp41_225+RLcp41_277)-ROcp41_523*(RLcp41_125+RLcp41_177);
    JTcp41_177_13 = -(RLcp41_277*ROcp41_924-RLcp41_377*ROcp41_824);
    JTcp41_277_13 = RLcp41_177*ROcp41_924-RLcp41_377*ROcp41_724;
    JTcp41_377_13 = -(RLcp41_177*ROcp41_824-RLcp41_277*ROcp41_724);
    ORcp41_177 = OMcp41_225*RLcp41_377-OMcp41_325*RLcp41_277;
    ORcp41_277 = -(OMcp41_125*RLcp41_377-OMcp41_325*RLcp41_177);
    ORcp41_377 = OMcp41_125*RLcp41_277-OMcp41_225*RLcp41_177;
    VIcp41_177 = ORcp41_119+ORcp41_120+ORcp41_121+ORcp41_122+ORcp41_123+ORcp41_124+ORcp41_125+ORcp41_177+qd[1];
    VIcp41_277 = ORcp41_219+ORcp41_220+ORcp41_221+ORcp41_222+ORcp41_223+ORcp41_224+ORcp41_225+ORcp41_277+qd[2];
    VIcp41_377 = ORcp41_319+ORcp41_320+ORcp41_321+ORcp41_322+ORcp41_323+ORcp41_324+ORcp41_325+ORcp41_377+qd[3];
    ACcp41_177 = qdd[1]+OMcp41_219*ORcp41_320+OMcp41_220*ORcp41_321+OMcp41_221*ORcp41_322+OMcp41_222*ORcp41_323+OMcp41_223
 *ORcp41_324+OMcp41_224*ORcp41_325+OMcp41_225*ORcp41_377+OMcp41_26*ORcp41_319-OMcp41_319*ORcp41_220-OMcp41_320*ORcp41_221-
 OMcp41_321*ORcp41_222-OMcp41_322*ORcp41_223-OMcp41_323*ORcp41_224-OMcp41_324*ORcp41_225-OMcp41_325*ORcp41_277-OMcp41_36*
 ORcp41_219+OPcp41_219*RLcp41_320+OPcp41_220*RLcp41_321+OPcp41_221*RLcp41_322+OPcp41_222*RLcp41_323+OPcp41_223*RLcp41_324+
 OPcp41_224*RLcp41_325+OPcp41_225*RLcp41_377+OPcp41_26*RLcp41_319-OPcp41_319*RLcp41_220-OPcp41_320*RLcp41_221-OPcp41_321*
 RLcp41_222-OPcp41_322*RLcp41_223-OPcp41_323*RLcp41_224-OPcp41_324*RLcp41_225-OPcp41_325*RLcp41_277-OPcp41_36*RLcp41_219;
    ACcp41_277 = qdd[2]-OMcp41_119*ORcp41_320-OMcp41_120*ORcp41_321-OMcp41_121*ORcp41_322-OMcp41_122*ORcp41_323-OMcp41_123
 *ORcp41_324-OMcp41_124*ORcp41_325-OMcp41_125*ORcp41_377-OMcp41_16*ORcp41_319+OMcp41_319*ORcp41_120+OMcp41_320*ORcp41_121+
 OMcp41_321*ORcp41_122+OMcp41_322*ORcp41_123+OMcp41_323*ORcp41_124+OMcp41_324*ORcp41_125+OMcp41_325*ORcp41_177+OMcp41_36*
 ORcp41_119-OPcp41_119*RLcp41_320-OPcp41_120*RLcp41_321-OPcp41_121*RLcp41_322-OPcp41_122*RLcp41_323-OPcp41_123*RLcp41_324-
 OPcp41_124*RLcp41_325-OPcp41_125*RLcp41_377-OPcp41_16*RLcp41_319+OPcp41_319*RLcp41_120+OPcp41_320*RLcp41_121+OPcp41_321*
 RLcp41_122+OPcp41_322*RLcp41_123+OPcp41_323*RLcp41_124+OPcp41_324*RLcp41_125+OPcp41_325*RLcp41_177+OPcp41_36*RLcp41_119;
    ACcp41_377 = qdd[3]+OMcp41_119*ORcp41_220+OMcp41_120*ORcp41_221+OMcp41_121*ORcp41_222+OMcp41_122*ORcp41_223+OMcp41_123
 *ORcp41_224+OMcp41_124*ORcp41_225+OMcp41_125*ORcp41_277+OMcp41_16*ORcp41_219-OMcp41_219*ORcp41_120-OMcp41_220*ORcp41_121-
 OMcp41_221*ORcp41_122-OMcp41_222*ORcp41_123-OMcp41_223*ORcp41_124-OMcp41_224*ORcp41_125-OMcp41_225*ORcp41_177-OMcp41_26*
 ORcp41_119+OPcp41_119*RLcp41_220+OPcp41_120*RLcp41_221+OPcp41_121*RLcp41_222+OPcp41_122*RLcp41_223+OPcp41_123*RLcp41_224+
 OPcp41_124*RLcp41_225+OPcp41_125*RLcp41_277+OPcp41_16*RLcp41_219-OPcp41_219*RLcp41_120-OPcp41_220*RLcp41_121-OPcp41_221*
 RLcp41_122-OPcp41_222*RLcp41_123-OPcp41_223*RLcp41_124-OPcp41_224*RLcp41_125-OPcp41_225*RLcp41_177-OPcp41_26*RLcp41_119;

// = = Block_1_0_0_42_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp41_177;
    sens->P[2] = POcp41_277;
    sens->P[3] = POcp41_377;
    sens->R[1][1] = ROcp41_125;
    sens->R[1][2] = ROcp41_225;
    sens->R[1][3] = ROcp41_325;
    sens->R[2][1] = ROcp41_425;
    sens->R[2][2] = ROcp41_525;
    sens->R[2][3] = ROcp41_625;
    sens->R[3][1] = ROcp41_724;
    sens->R[3][2] = ROcp41_824;
    sens->R[3][3] = ROcp41_924;
    sens->V[1] = VIcp41_177;
    sens->V[2] = VIcp41_277;
    sens->V[3] = VIcp41_377;
    sens->OM[1] = OMcp41_125;
    sens->OM[2] = OMcp41_225;
    sens->OM[3] = OMcp41_325;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp41_177_5;
    sens->J[1][6] = JTcp41_177_6;
    sens->J[1][19] = JTcp41_177_7;
    sens->J[1][20] = JTcp41_177_8;
    sens->J[1][21] = JTcp41_177_9;
    sens->J[1][22] = JTcp41_177_10;
    sens->J[1][23] = JTcp41_177_11;
    sens->J[1][24] = JTcp41_177_12;
    sens->J[1][25] = JTcp41_177_13;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp41_277_4;
    sens->J[2][5] = JTcp41_277_5;
    sens->J[2][6] = JTcp41_277_6;
    sens->J[2][19] = JTcp41_277_7;
    sens->J[2][20] = JTcp41_277_8;
    sens->J[2][21] = JTcp41_277_9;
    sens->J[2][22] = JTcp41_277_10;
    sens->J[2][23] = JTcp41_277_11;
    sens->J[2][24] = JTcp41_277_12;
    sens->J[2][25] = JTcp41_277_13;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp41_377_4;
    sens->J[3][5] = JTcp41_377_5;
    sens->J[3][6] = JTcp41_377_6;
    sens->J[3][19] = JTcp41_377_7;
    sens->J[3][20] = JTcp41_377_8;
    sens->J[3][21] = JTcp41_377_9;
    sens->J[3][22] = JTcp41_377_10;
    sens->J[3][23] = JTcp41_377_11;
    sens->J[3][24] = JTcp41_377_12;
    sens->J[3][25] = JTcp41_377_13;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp41_16;
    sens->J[4][20] = ROcp41_419;
    sens->J[4][21] = ROcp41_720;
    sens->J[4][22] = ROcp41_421;
    sens->J[4][23] = ROcp41_122;
    sens->J[4][24] = ROcp41_423;
    sens->J[4][25] = ROcp41_724;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp41_85;
    sens->J[5][19] = ROcp41_26;
    sens->J[5][20] = ROcp41_519;
    sens->J[5][21] = ROcp41_820;
    sens->J[5][22] = ROcp41_521;
    sens->J[5][23] = ROcp41_222;
    sens->J[5][24] = ROcp41_523;
    sens->J[5][25] = ROcp41_824;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp41_95;
    sens->J[6][19] = ROcp41_36;
    sens->J[6][20] = ROcp41_619;
    sens->J[6][21] = ROcp41_920;
    sens->J[6][22] = ROcp41_621;
    sens->J[6][23] = ROcp41_322;
    sens->J[6][24] = ROcp41_623;
    sens->J[6][25] = ROcp41_924;
    sens->A[1] = ACcp41_177;
    sens->A[2] = ACcp41_277;
    sens->A[3] = ACcp41_377;
    sens->OMP[1] = OPcp41_125;
    sens->OMP[2] = OPcp41_225;
    sens->OMP[3] = OPcp41_325;
 
// 
break;
case 43:
 


// = = Block_1_0_0_43_0_1 = = 
 
// Sensor Kinematics 


    ROcp42_25 = S4*S5;
    ROcp42_35 = -C4*S5;
    ROcp42_85 = -S4*C5;
    ROcp42_95 = C4*C5;
    ROcp42_16 = C5*C6;
    ROcp42_26 = ROcp42_25*C6+C4*S6;
    ROcp42_36 = ROcp42_35*C6+S4*S6;
    ROcp42_46 = -C5*S6;
    ROcp42_56 = -(ROcp42_25*S6-C4*C6);
    ROcp42_66 = -(ROcp42_35*S6-S4*C6);
    OMcp42_25 = qd[5]*C4;
    OMcp42_35 = qd[5]*S4;
    OMcp42_16 = qd[4]+qd[6]*S5;
    OMcp42_26 = OMcp42_25+ROcp42_85*qd[6];
    OMcp42_36 = OMcp42_35+ROcp42_95*qd[6];
    OPcp42_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp42_26 = ROcp42_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp42_35*S5-ROcp42_95*qd[4]);
    OPcp42_36 = ROcp42_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp42_25*S5-ROcp42_85*qd[4]);

// = = Block_1_0_0_43_0_4 = = 
 
// Sensor Kinematics 


    ROcp42_419 = ROcp42_46*C19+S19*S5;
    ROcp42_519 = ROcp42_56*C19+ROcp42_85*S19;
    ROcp42_619 = ROcp42_66*C19+ROcp42_95*S19;
    ROcp42_719 = -(ROcp42_46*S19-C19*S5);
    ROcp42_819 = -(ROcp42_56*S19-ROcp42_85*C19);
    ROcp42_919 = -(ROcp42_66*S19-ROcp42_95*C19);
    ROcp42_120 = ROcp42_16*C20-ROcp42_719*S20;
    ROcp42_220 = ROcp42_26*C20-ROcp42_819*S20;
    ROcp42_320 = ROcp42_36*C20-ROcp42_919*S20;
    ROcp42_720 = ROcp42_16*S20+ROcp42_719*C20;
    ROcp42_820 = ROcp42_26*S20+ROcp42_819*C20;
    ROcp42_920 = ROcp42_36*S20+ROcp42_919*C20;
    ROcp42_121 = ROcp42_120*C21+ROcp42_419*S21;
    ROcp42_221 = ROcp42_220*C21+ROcp42_519*S21;
    ROcp42_321 = ROcp42_320*C21+ROcp42_619*S21;
    ROcp42_421 = -(ROcp42_120*S21-ROcp42_419*C21);
    ROcp42_521 = -(ROcp42_220*S21-ROcp42_519*C21);
    ROcp42_621 = -(ROcp42_320*S21-ROcp42_619*C21);
    RLcp42_119 = s->dpt[2][3]*ROcp42_46+ROcp42_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp42_219 = s->dpt[2][3]*ROcp42_56+ROcp42_26*s->dpt[1][3]+ROcp42_85*s->dpt[3][3];
    RLcp42_319 = s->dpt[2][3]*ROcp42_66+ROcp42_36*s->dpt[1][3]+ROcp42_95*s->dpt[3][3];
    OMcp42_119 = OMcp42_16+ROcp42_16*qd[19];
    OMcp42_219 = OMcp42_26+ROcp42_26*qd[19];
    OMcp42_319 = OMcp42_36+ROcp42_36*qd[19];
    ORcp42_119 = OMcp42_26*RLcp42_319-OMcp42_36*RLcp42_219;
    ORcp42_219 = -(OMcp42_16*RLcp42_319-OMcp42_36*RLcp42_119);
    ORcp42_319 = OMcp42_16*RLcp42_219-OMcp42_26*RLcp42_119;
    OPcp42_119 = OPcp42_16+ROcp42_16*qdd[19]+qd[19]*(OMcp42_26*ROcp42_36-OMcp42_36*ROcp42_26);
    OPcp42_219 = OPcp42_26+ROcp42_26*qdd[19]-qd[19]*(OMcp42_16*ROcp42_36-OMcp42_36*ROcp42_16);
    OPcp42_319 = OPcp42_36+ROcp42_36*qdd[19]+qd[19]*(OMcp42_16*ROcp42_26-OMcp42_26*ROcp42_16);
    RLcp42_120 = s->dpt[1][38]*ROcp42_16+s->dpt[2][38]*ROcp42_419+s->dpt[3][38]*ROcp42_719;
    RLcp42_220 = s->dpt[1][38]*ROcp42_26+s->dpt[2][38]*ROcp42_519+s->dpt[3][38]*ROcp42_819;
    RLcp42_320 = s->dpt[1][38]*ROcp42_36+s->dpt[2][38]*ROcp42_619+s->dpt[3][38]*ROcp42_919;
    OMcp42_120 = OMcp42_119+ROcp42_419*qd[20];
    OMcp42_220 = OMcp42_219+ROcp42_519*qd[20];
    OMcp42_320 = OMcp42_319+ROcp42_619*qd[20];
    ORcp42_120 = OMcp42_219*RLcp42_320-OMcp42_319*RLcp42_220;
    ORcp42_220 = -(OMcp42_119*RLcp42_320-OMcp42_319*RLcp42_120);
    ORcp42_320 = OMcp42_119*RLcp42_220-OMcp42_219*RLcp42_120;
    OPcp42_120 = OPcp42_119+ROcp42_419*qdd[20]+qd[20]*(OMcp42_219*ROcp42_619-OMcp42_319*ROcp42_519);
    OPcp42_220 = OPcp42_219+ROcp42_519*qdd[20]-qd[20]*(OMcp42_119*ROcp42_619-OMcp42_319*ROcp42_419);
    OPcp42_320 = OPcp42_319+ROcp42_619*qdd[20]+qd[20]*(OMcp42_119*ROcp42_519-OMcp42_219*ROcp42_419);
    RLcp42_121 = s->dpt[1][40]*ROcp42_120+s->dpt[2][40]*ROcp42_419+ROcp42_720*s->dpt[3][40];
    RLcp42_221 = s->dpt[1][40]*ROcp42_220+s->dpt[2][40]*ROcp42_519+ROcp42_820*s->dpt[3][40];
    RLcp42_321 = s->dpt[1][40]*ROcp42_320+s->dpt[2][40]*ROcp42_619+ROcp42_920*s->dpt[3][40];
    OMcp42_121 = OMcp42_120+ROcp42_720*qd[21];
    OMcp42_221 = OMcp42_220+ROcp42_820*qd[21];
    OMcp42_321 = OMcp42_320+ROcp42_920*qd[21];
    ORcp42_121 = OMcp42_220*RLcp42_321-OMcp42_320*RLcp42_221;
    ORcp42_221 = -(OMcp42_120*RLcp42_321-OMcp42_320*RLcp42_121);
    ORcp42_321 = OMcp42_120*RLcp42_221-OMcp42_220*RLcp42_121;
    OPcp42_121 = OPcp42_120+ROcp42_720*qdd[21]+qd[21]*(OMcp42_220*ROcp42_920-OMcp42_320*ROcp42_820);
    OPcp42_221 = OPcp42_220+ROcp42_820*qdd[21]-qd[21]*(OMcp42_120*ROcp42_920-OMcp42_320*ROcp42_720);
    OPcp42_321 = OPcp42_320+ROcp42_920*qdd[21]+qd[21]*(OMcp42_120*ROcp42_820-OMcp42_220*ROcp42_720);

// = = Block_1_0_0_43_0_5 = = 
 
// Sensor Kinematics 


    ROcp42_122 = ROcp42_121*C22-ROcp42_720*S22;
    ROcp42_222 = ROcp42_221*C22-ROcp42_820*S22;
    ROcp42_322 = ROcp42_321*C22-ROcp42_920*S22;
    ROcp42_722 = ROcp42_121*S22+ROcp42_720*C22;
    ROcp42_822 = ROcp42_221*S22+ROcp42_820*C22;
    ROcp42_922 = ROcp42_321*S22+ROcp42_920*C22;
    ROcp42_423 = ROcp42_421*C23+ROcp42_722*S23;
    ROcp42_523 = ROcp42_521*C23+ROcp42_822*S23;
    ROcp42_623 = ROcp42_621*C23+ROcp42_922*S23;
    ROcp42_723 = -(ROcp42_421*S23-ROcp42_722*C23);
    ROcp42_823 = -(ROcp42_521*S23-ROcp42_822*C23);
    ROcp42_923 = -(ROcp42_621*S23-ROcp42_922*C23);
    ROcp42_124 = ROcp42_122*C24-ROcp42_723*S24;
    ROcp42_224 = ROcp42_222*C24-ROcp42_823*S24;
    ROcp42_324 = ROcp42_322*C24-ROcp42_923*S24;
    ROcp42_724 = ROcp42_122*S24+ROcp42_723*C24;
    ROcp42_824 = ROcp42_222*S24+ROcp42_823*C24;
    ROcp42_924 = ROcp42_322*S24+ROcp42_923*C24;
    ROcp42_125 = ROcp42_124*C25+ROcp42_423*S25;
    ROcp42_225 = ROcp42_224*C25+ROcp42_523*S25;
    ROcp42_325 = ROcp42_324*C25+ROcp42_623*S25;
    ROcp42_425 = -(ROcp42_124*S25-ROcp42_423*C25);
    ROcp42_525 = -(ROcp42_224*S25-ROcp42_523*C25);
    ROcp42_625 = -(ROcp42_324*S25-ROcp42_623*C25);
    ROcp42_126 = ROcp42_125*C26-ROcp42_724*S26;
    ROcp42_226 = ROcp42_225*C26-ROcp42_824*S26;
    ROcp42_326 = ROcp42_325*C26-ROcp42_924*S26;
    ROcp42_726 = ROcp42_125*S26+ROcp42_724*C26;
    ROcp42_826 = ROcp42_225*S26+ROcp42_824*C26;
    ROcp42_926 = ROcp42_325*S26+ROcp42_924*C26;
    ROcp42_127 = ROcp42_126*C27+ROcp42_425*S27;
    ROcp42_227 = ROcp42_226*C27+ROcp42_525*S27;
    ROcp42_327 = ROcp42_326*C27+ROcp42_625*S27;
    ROcp42_427 = -(ROcp42_126*S27-ROcp42_425*C27);
    ROcp42_527 = -(ROcp42_226*S27-ROcp42_525*C27);
    ROcp42_627 = -(ROcp42_326*S27-ROcp42_625*C27);
    ROcp42_428 = ROcp42_427*C28+ROcp42_726*S28;
    ROcp42_528 = ROcp42_527*C28+ROcp42_826*S28;
    ROcp42_628 = ROcp42_627*C28+ROcp42_926*S28;
    ROcp42_728 = -(ROcp42_427*S28-ROcp42_726*C28);
    ROcp42_828 = -(ROcp42_527*S28-ROcp42_826*C28);
    ROcp42_928 = -(ROcp42_627*S28-ROcp42_926*C28);
    RLcp42_122 = ROcp42_121*s->dpt[1][44]+ROcp42_421*s->dpt[2][44]+ROcp42_720*s->dpt[3][44];
    RLcp42_222 = ROcp42_221*s->dpt[1][44]+ROcp42_521*s->dpt[2][44]+ROcp42_820*s->dpt[3][44];
    RLcp42_322 = ROcp42_321*s->dpt[1][44]+ROcp42_621*s->dpt[2][44]+ROcp42_920*s->dpt[3][44];
    OMcp42_122 = OMcp42_121+ROcp42_421*qd[22];
    OMcp42_222 = OMcp42_221+ROcp42_521*qd[22];
    OMcp42_322 = OMcp42_321+ROcp42_621*qd[22];
    ORcp42_122 = OMcp42_221*RLcp42_322-OMcp42_321*RLcp42_222;
    ORcp42_222 = -(OMcp42_121*RLcp42_322-OMcp42_321*RLcp42_122);
    ORcp42_322 = OMcp42_121*RLcp42_222-OMcp42_221*RLcp42_122;
    OPcp42_122 = OPcp42_121+ROcp42_421*qdd[22]+qd[22]*(OMcp42_221*ROcp42_621-OMcp42_321*ROcp42_521);
    OPcp42_222 = OPcp42_221+ROcp42_521*qdd[22]-qd[22]*(OMcp42_121*ROcp42_621-OMcp42_321*ROcp42_421);
    OPcp42_322 = OPcp42_321+ROcp42_621*qdd[22]+qd[22]*(OMcp42_121*ROcp42_521-OMcp42_221*ROcp42_421);
    RLcp42_123 = s->dpt[1][48]*ROcp42_122+s->dpt[3][48]*ROcp42_722+ROcp42_421*s->dpt[2][48];
    RLcp42_223 = s->dpt[1][48]*ROcp42_222+s->dpt[3][48]*ROcp42_822+ROcp42_521*s->dpt[2][48];
    RLcp42_323 = s->dpt[1][48]*ROcp42_322+s->dpt[3][48]*ROcp42_922+ROcp42_621*s->dpt[2][48];
    OMcp42_123 = OMcp42_122+ROcp42_122*qd[23];
    OMcp42_223 = OMcp42_222+ROcp42_222*qd[23];
    OMcp42_323 = OMcp42_322+ROcp42_322*qd[23];
    ORcp42_123 = OMcp42_222*RLcp42_323-OMcp42_322*RLcp42_223;
    ORcp42_223 = -(OMcp42_122*RLcp42_323-OMcp42_322*RLcp42_123);
    ORcp42_323 = OMcp42_122*RLcp42_223-OMcp42_222*RLcp42_123;
    OPcp42_123 = OPcp42_122+ROcp42_122*qdd[23]+qd[23]*(OMcp42_222*ROcp42_322-OMcp42_322*ROcp42_222);
    OPcp42_223 = OPcp42_222+ROcp42_222*qdd[23]-qd[23]*(OMcp42_122*ROcp42_322-OMcp42_322*ROcp42_122);
    OPcp42_323 = OPcp42_322+ROcp42_322*qdd[23]+qd[23]*(OMcp42_122*ROcp42_222-OMcp42_222*ROcp42_122);
    RLcp42_124 = s->dpt[1][50]*ROcp42_122+s->dpt[3][50]*ROcp42_723+ROcp42_423*s->dpt[2][50];
    RLcp42_224 = s->dpt[1][50]*ROcp42_222+s->dpt[3][50]*ROcp42_823+ROcp42_523*s->dpt[2][50];
    RLcp42_324 = s->dpt[1][50]*ROcp42_322+s->dpt[3][50]*ROcp42_923+ROcp42_623*s->dpt[2][50];
    OMcp42_124 = OMcp42_123+ROcp42_423*qd[24];
    OMcp42_224 = OMcp42_223+ROcp42_523*qd[24];
    OMcp42_324 = OMcp42_323+ROcp42_623*qd[24];
    ORcp42_124 = OMcp42_223*RLcp42_324-OMcp42_323*RLcp42_224;
    ORcp42_224 = -(OMcp42_123*RLcp42_324-OMcp42_323*RLcp42_124);
    ORcp42_324 = OMcp42_123*RLcp42_224-OMcp42_223*RLcp42_124;
    OPcp42_124 = OPcp42_123+ROcp42_423*qdd[24]+qd[24]*(OMcp42_223*ROcp42_623-OMcp42_323*ROcp42_523);
    OPcp42_224 = OPcp42_223+ROcp42_523*qdd[24]-qd[24]*(OMcp42_123*ROcp42_623-OMcp42_323*ROcp42_423);
    OPcp42_324 = OPcp42_323+ROcp42_623*qdd[24]+qd[24]*(OMcp42_123*ROcp42_523-OMcp42_223*ROcp42_423);
    RLcp42_125 = s->dpt[1][52]*ROcp42_124+s->dpt[3][52]*ROcp42_724+ROcp42_423*s->dpt[2][52];
    RLcp42_225 = s->dpt[1][52]*ROcp42_224+s->dpt[3][52]*ROcp42_824+ROcp42_523*s->dpt[2][52];
    RLcp42_325 = s->dpt[1][52]*ROcp42_324+s->dpt[3][52]*ROcp42_924+ROcp42_623*s->dpt[2][52];
    OMcp42_125 = OMcp42_124+ROcp42_724*qd[25];
    OMcp42_225 = OMcp42_224+ROcp42_824*qd[25];
    OMcp42_325 = OMcp42_324+ROcp42_924*qd[25];
    ORcp42_125 = OMcp42_224*RLcp42_325-OMcp42_324*RLcp42_225;
    ORcp42_225 = -(OMcp42_124*RLcp42_325-OMcp42_324*RLcp42_125);
    ORcp42_325 = OMcp42_124*RLcp42_225-OMcp42_224*RLcp42_125;
    OPcp42_125 = OPcp42_124+ROcp42_724*qdd[25]+qd[25]*(OMcp42_224*ROcp42_924-OMcp42_324*ROcp42_824);
    OPcp42_225 = OPcp42_224+ROcp42_824*qdd[25]-qd[25]*(OMcp42_124*ROcp42_924-OMcp42_324*ROcp42_724);
    OPcp42_325 = OPcp42_324+ROcp42_924*qdd[25]+qd[25]*(OMcp42_124*ROcp42_824-OMcp42_224*ROcp42_724);
    RLcp42_126 = s->dpt[2][54]*ROcp42_425+s->dpt[3][54]*ROcp42_724+ROcp42_125*s->dpt[1][54];
    RLcp42_226 = s->dpt[2][54]*ROcp42_525+s->dpt[3][54]*ROcp42_824+ROcp42_225*s->dpt[1][54];
    RLcp42_326 = s->dpt[2][54]*ROcp42_625+s->dpt[3][54]*ROcp42_924+ROcp42_325*s->dpt[1][54];
    OMcp42_126 = OMcp42_125+ROcp42_425*qd[26];
    OMcp42_226 = OMcp42_225+ROcp42_525*qd[26];
    OMcp42_326 = OMcp42_325+ROcp42_625*qd[26];
    ORcp42_126 = OMcp42_225*RLcp42_326-OMcp42_325*RLcp42_226;
    ORcp42_226 = -(OMcp42_125*RLcp42_326-OMcp42_325*RLcp42_126);
    ORcp42_326 = OMcp42_125*RLcp42_226-OMcp42_225*RLcp42_126;
    OPcp42_126 = OPcp42_125+ROcp42_425*qdd[26]+qd[26]*(OMcp42_225*ROcp42_625-OMcp42_325*ROcp42_525);
    OPcp42_226 = OPcp42_225+ROcp42_525*qdd[26]-qd[26]*(OMcp42_125*ROcp42_625-OMcp42_325*ROcp42_425);
    OPcp42_326 = OPcp42_325+ROcp42_625*qdd[26]+qd[26]*(OMcp42_125*ROcp42_525-OMcp42_225*ROcp42_425);
    RLcp42_127 = s->dpt[1][55]*ROcp42_126+s->dpt[3][55]*ROcp42_726+ROcp42_425*s->dpt[2][55];
    RLcp42_227 = s->dpt[1][55]*ROcp42_226+s->dpt[3][55]*ROcp42_826+ROcp42_525*s->dpt[2][55];
    RLcp42_327 = s->dpt[1][55]*ROcp42_326+s->dpt[3][55]*ROcp42_926+ROcp42_625*s->dpt[2][55];
    OMcp42_127 = OMcp42_126+ROcp42_726*qd[27];
    OMcp42_227 = OMcp42_226+ROcp42_826*qd[27];
    OMcp42_327 = OMcp42_326+ROcp42_926*qd[27];
    ORcp42_127 = OMcp42_226*RLcp42_327-OMcp42_326*RLcp42_227;
    ORcp42_227 = -(OMcp42_126*RLcp42_327-OMcp42_326*RLcp42_127);
    ORcp42_327 = OMcp42_126*RLcp42_227-OMcp42_226*RLcp42_127;
    OPcp42_127 = OPcp42_126+ROcp42_726*qdd[27]+qd[27]*(OMcp42_226*ROcp42_926-OMcp42_326*ROcp42_826);
    OPcp42_227 = OPcp42_226+ROcp42_826*qdd[27]-qd[27]*(OMcp42_126*ROcp42_926-OMcp42_326*ROcp42_726);
    OPcp42_327 = OPcp42_326+ROcp42_926*qdd[27]+qd[27]*(OMcp42_126*ROcp42_826-OMcp42_226*ROcp42_726);
    RLcp42_128 = s->dpt[1][56]*ROcp42_127+s->dpt[2][56]*ROcp42_427+s->dpt[3][56]*ROcp42_726;
    RLcp42_228 = s->dpt[1][56]*ROcp42_227+s->dpt[2][56]*ROcp42_527+s->dpt[3][56]*ROcp42_826;
    RLcp42_328 = s->dpt[1][56]*ROcp42_327+s->dpt[2][56]*ROcp42_627+s->dpt[3][56]*ROcp42_926;
    OMcp42_128 = OMcp42_127+ROcp42_127*qd[28];
    OMcp42_228 = OMcp42_227+ROcp42_227*qd[28];
    OMcp42_328 = OMcp42_327+ROcp42_327*qd[28];
    ORcp42_128 = OMcp42_227*RLcp42_328-OMcp42_327*RLcp42_228;
    ORcp42_228 = -(OMcp42_127*RLcp42_328-OMcp42_327*RLcp42_128);
    ORcp42_328 = OMcp42_127*RLcp42_228-OMcp42_227*RLcp42_128;
    OPcp42_128 = OPcp42_127+ROcp42_127*qdd[28]+qd[28]*(OMcp42_227*ROcp42_327-OMcp42_327*ROcp42_227);
    OPcp42_228 = OPcp42_227+ROcp42_227*qdd[28]-qd[28]*(OMcp42_127*ROcp42_327-OMcp42_327*ROcp42_127);
    OPcp42_328 = OPcp42_327+ROcp42_327*qdd[28]+qd[28]*(OMcp42_127*ROcp42_227-OMcp42_227*ROcp42_127);
    RLcp42_178 = s->dpt[1][57]*ROcp42_127+s->dpt[3][57]*ROcp42_728+ROcp42_428*s->dpt[2][57];
    RLcp42_278 = s->dpt[1][57]*ROcp42_227+s->dpt[3][57]*ROcp42_828+ROcp42_528*s->dpt[2][57];
    RLcp42_378 = s->dpt[1][57]*ROcp42_327+s->dpt[3][57]*ROcp42_928+ROcp42_628*s->dpt[2][57];
    POcp42_178 = RLcp42_119+RLcp42_120+RLcp42_121+RLcp42_122+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_126+RLcp42_127+
 RLcp42_128+RLcp42_178+q[1];
    POcp42_278 = RLcp42_219+RLcp42_220+RLcp42_221+RLcp42_222+RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_226+RLcp42_227+
 RLcp42_228+RLcp42_278+q[2];
    POcp42_378 = RLcp42_319+RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+
 RLcp42_328+RLcp42_378+q[3];
    JTcp42_278_4 = -(RLcp42_319+RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+
 RLcp42_328+RLcp42_378);
    JTcp42_378_4 = RLcp42_219+RLcp42_220+RLcp42_221+RLcp42_222+RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_226+RLcp42_227+
 RLcp42_228+RLcp42_278;
    JTcp42_178_5 = C4*(RLcp42_319+RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+
 RLcp42_328)-S4*(RLcp42_219+RLcp42_220)-S4*(RLcp42_221+RLcp42_222)-S4*(RLcp42_223+RLcp42_224)-S4*(RLcp42_225+RLcp42_226)-S4*(
 RLcp42_227+RLcp42_228)-RLcp42_278*S4+RLcp42_378*C4;
    JTcp42_278_5 = S4*(RLcp42_119+RLcp42_120+RLcp42_121+RLcp42_122+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_126+RLcp42_127+
 RLcp42_128+RLcp42_178);
    JTcp42_378_5 = -C4*(RLcp42_119+RLcp42_120+RLcp42_121+RLcp42_122+RLcp42_123+RLcp42_124+RLcp42_125+RLcp42_126+RLcp42_127
 +RLcp42_128+RLcp42_178);
    JTcp42_178_6 = ROcp42_85*(RLcp42_319+RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+
 RLcp42_327+RLcp42_328)-ROcp42_95*(RLcp42_219+RLcp42_220)-ROcp42_95*(RLcp42_221+RLcp42_222)-ROcp42_95*(RLcp42_223+RLcp42_224)
 -ROcp42_95*(RLcp42_225+RLcp42_226)-ROcp42_95*(RLcp42_227+RLcp42_228)-RLcp42_278*ROcp42_95+RLcp42_378*ROcp42_85;
    JTcp42_278_6 = -(RLcp42_378*S5-ROcp42_95*(RLcp42_119+RLcp42_120+RLcp42_121+RLcp42_122+RLcp42_123+RLcp42_124+RLcp42_125
 +RLcp42_126+RLcp42_127+RLcp42_128+RLcp42_178)+S5*(RLcp42_319+RLcp42_320)+S5*(RLcp42_321+RLcp42_322)+S5*(RLcp42_323+
 RLcp42_324)+S5*(RLcp42_325+RLcp42_326)+S5*(RLcp42_327+RLcp42_328));
    JTcp42_378_6 = RLcp42_278*S5-ROcp42_85*(RLcp42_119+RLcp42_120+RLcp42_121+RLcp42_122+RLcp42_123+RLcp42_124+RLcp42_125+
 RLcp42_126+RLcp42_127+RLcp42_128+RLcp42_178)+S5*(RLcp42_219+RLcp42_220)+S5*(RLcp42_221+RLcp42_222)+S5*(RLcp42_223+RLcp42_224
 )+S5*(RLcp42_225+RLcp42_226)+S5*(RLcp42_227+RLcp42_228);
    JTcp42_178_7 = ROcp42_26*(RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+
 RLcp42_328+RLcp42_378)-ROcp42_36*(RLcp42_220+RLcp42_221)-ROcp42_36*(RLcp42_222+RLcp42_223)-ROcp42_36*(RLcp42_224+RLcp42_225)
 -ROcp42_36*(RLcp42_226+RLcp42_227)-ROcp42_36*(RLcp42_228+RLcp42_278);
    JTcp42_278_7 = -(ROcp42_16*(RLcp42_320+RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+
 RLcp42_328+RLcp42_378)-ROcp42_36*(RLcp42_120+RLcp42_121)-ROcp42_36*(RLcp42_122+RLcp42_123)-ROcp42_36*(RLcp42_124+RLcp42_125)
 -ROcp42_36*(RLcp42_126+RLcp42_127)-ROcp42_36*(RLcp42_128+RLcp42_178));
    JTcp42_378_7 = ROcp42_16*(RLcp42_220+RLcp42_221+RLcp42_222+RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_226+RLcp42_227+
 RLcp42_228+RLcp42_278)-ROcp42_26*(RLcp42_120+RLcp42_121)-ROcp42_26*(RLcp42_122+RLcp42_123)-ROcp42_26*(RLcp42_124+RLcp42_125)
 -ROcp42_26*(RLcp42_126+RLcp42_127)-ROcp42_26*(RLcp42_128+RLcp42_178);
    JTcp42_178_8 = ROcp42_519*(RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+RLcp42_328)-
 ROcp42_619*(RLcp42_221+RLcp42_222)-ROcp42_619*(RLcp42_223+RLcp42_224)-ROcp42_619*(RLcp42_225+RLcp42_226)-ROcp42_619*(
 RLcp42_227+RLcp42_228)-RLcp42_278*ROcp42_619+RLcp42_378*ROcp42_519;
    JTcp42_278_8 = RLcp42_178*ROcp42_619-RLcp42_378*ROcp42_419-ROcp42_419*(RLcp42_321+RLcp42_322+RLcp42_323+RLcp42_324+
 RLcp42_325+RLcp42_326+RLcp42_327+RLcp42_328)+ROcp42_619*(RLcp42_121+RLcp42_122)+ROcp42_619*(RLcp42_123+RLcp42_124)+
 ROcp42_619*(RLcp42_125+RLcp42_126)+ROcp42_619*(RLcp42_127+RLcp42_128);
    JTcp42_378_8 = ROcp42_419*(RLcp42_221+RLcp42_222+RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_226+RLcp42_227+RLcp42_228)-
 ROcp42_519*(RLcp42_121+RLcp42_122)-ROcp42_519*(RLcp42_123+RLcp42_124)-ROcp42_519*(RLcp42_125+RLcp42_126)-ROcp42_519*(
 RLcp42_127+RLcp42_128)-RLcp42_178*ROcp42_519+RLcp42_278*ROcp42_419;
    JTcp42_178_9 = ROcp42_820*(RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+RLcp42_328+RLcp42_378)-
 ROcp42_920*(RLcp42_222+RLcp42_223)-ROcp42_920*(RLcp42_224+RLcp42_225)-ROcp42_920*(RLcp42_226+RLcp42_227)-ROcp42_920*(
 RLcp42_228+RLcp42_278);
    JTcp42_278_9 = -(ROcp42_720*(RLcp42_322+RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+RLcp42_328+RLcp42_378)-
 ROcp42_920*(RLcp42_122+RLcp42_123)-ROcp42_920*(RLcp42_124+RLcp42_125)-ROcp42_920*(RLcp42_126+RLcp42_127)-ROcp42_920*(
 RLcp42_128+RLcp42_178));
    JTcp42_378_9 = ROcp42_720*(RLcp42_222+RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_226+RLcp42_227+RLcp42_228+RLcp42_278)-
 ROcp42_820*(RLcp42_122+RLcp42_123)-ROcp42_820*(RLcp42_124+RLcp42_125)-ROcp42_820*(RLcp42_126+RLcp42_127)-ROcp42_820*(
 RLcp42_128+RLcp42_178);
    JTcp42_178_10 = ROcp42_521*(RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+RLcp42_328)-ROcp42_621*(RLcp42_223+
 RLcp42_224)-ROcp42_621*(RLcp42_225+RLcp42_226)-ROcp42_621*(RLcp42_227+RLcp42_228)-RLcp42_278*ROcp42_621+RLcp42_378*
 ROcp42_521;
    JTcp42_278_10 = RLcp42_178*ROcp42_621-RLcp42_378*ROcp42_421-ROcp42_421*(RLcp42_323+RLcp42_324+RLcp42_325+RLcp42_326+
 RLcp42_327+RLcp42_328)+ROcp42_621*(RLcp42_123+RLcp42_124)+ROcp42_621*(RLcp42_125+RLcp42_126)+ROcp42_621*(RLcp42_127+
 RLcp42_128);
    JTcp42_378_10 = ROcp42_421*(RLcp42_223+RLcp42_224+RLcp42_225+RLcp42_226+RLcp42_227+RLcp42_228)-ROcp42_521*(RLcp42_123+
 RLcp42_124)-ROcp42_521*(RLcp42_125+RLcp42_126)-ROcp42_521*(RLcp42_127+RLcp42_128)-RLcp42_178*ROcp42_521+RLcp42_278*
 ROcp42_421;
    JTcp42_178_11 = ROcp42_222*(RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+RLcp42_328+RLcp42_378)-ROcp42_322*(RLcp42_224+
 RLcp42_225)-ROcp42_322*(RLcp42_226+RLcp42_227)-ROcp42_322*(RLcp42_228+RLcp42_278);
    JTcp42_278_11 = -(ROcp42_122*(RLcp42_324+RLcp42_325+RLcp42_326+RLcp42_327+RLcp42_328+RLcp42_378)-ROcp42_322*(
 RLcp42_124+RLcp42_125)-ROcp42_322*(RLcp42_126+RLcp42_127)-ROcp42_322*(RLcp42_128+RLcp42_178));
    JTcp42_378_11 = ROcp42_122*(RLcp42_224+RLcp42_225+RLcp42_226+RLcp42_227+RLcp42_228+RLcp42_278)-ROcp42_222*(RLcp42_124+
 RLcp42_125)-ROcp42_222*(RLcp42_126+RLcp42_127)-ROcp42_222*(RLcp42_128+RLcp42_178);
    JTcp42_178_12 = ROcp42_523*(RLcp42_325+RLcp42_326+RLcp42_327+RLcp42_328)-ROcp42_623*(RLcp42_225+RLcp42_226)-ROcp42_623
 *(RLcp42_227+RLcp42_228)-RLcp42_278*ROcp42_623+RLcp42_378*ROcp42_523;
    JTcp42_278_12 = RLcp42_178*ROcp42_623-RLcp42_378*ROcp42_423-ROcp42_423*(RLcp42_325+RLcp42_326+RLcp42_327+RLcp42_328)+
 ROcp42_623*(RLcp42_125+RLcp42_126)+ROcp42_623*(RLcp42_127+RLcp42_128);
    JTcp42_378_12 = ROcp42_423*(RLcp42_225+RLcp42_226+RLcp42_227+RLcp42_228)-ROcp42_523*(RLcp42_125+RLcp42_126)-ROcp42_523
 *(RLcp42_127+RLcp42_128)-RLcp42_178*ROcp42_523+RLcp42_278*ROcp42_423;
    JTcp42_178_13 = ROcp42_824*(RLcp42_326+RLcp42_327+RLcp42_328+RLcp42_378)-ROcp42_924*(RLcp42_226+RLcp42_227)-ROcp42_924
 *(RLcp42_228+RLcp42_278);
    JTcp42_278_13 = -(ROcp42_724*(RLcp42_326+RLcp42_327+RLcp42_328+RLcp42_378)-ROcp42_924*(RLcp42_126+RLcp42_127)-
 ROcp42_924*(RLcp42_128+RLcp42_178));
    JTcp42_378_13 = ROcp42_724*(RLcp42_226+RLcp42_227+RLcp42_228+RLcp42_278)-ROcp42_824*(RLcp42_126+RLcp42_127)-ROcp42_824
 *(RLcp42_128+RLcp42_178);
    JTcp42_178_14 = ROcp42_525*(RLcp42_327+RLcp42_328)-ROcp42_625*(RLcp42_227+RLcp42_228)-RLcp42_278*ROcp42_625+RLcp42_378
 *ROcp42_525;
    JTcp42_278_14 = RLcp42_178*ROcp42_625-RLcp42_378*ROcp42_425-ROcp42_425*(RLcp42_327+RLcp42_328)+ROcp42_625*(RLcp42_127+
 RLcp42_128);
    JTcp42_378_14 = ROcp42_425*(RLcp42_227+RLcp42_228)-ROcp42_525*(RLcp42_127+RLcp42_128)-RLcp42_178*ROcp42_525+RLcp42_278
 *ROcp42_425;
    JTcp42_178_15 = ROcp42_826*(RLcp42_328+RLcp42_378)-ROcp42_926*(RLcp42_228+RLcp42_278);
    JTcp42_278_15 = -(ROcp42_726*(RLcp42_328+RLcp42_378)-ROcp42_926*(RLcp42_128+RLcp42_178));
    JTcp42_378_15 = ROcp42_726*(RLcp42_228+RLcp42_278)-ROcp42_826*(RLcp42_128+RLcp42_178);
    JTcp42_178_16 = -(RLcp42_278*ROcp42_327-RLcp42_378*ROcp42_227);
    JTcp42_278_16 = RLcp42_178*ROcp42_327-RLcp42_378*ROcp42_127;
    JTcp42_378_16 = -(RLcp42_178*ROcp42_227-RLcp42_278*ROcp42_127);
    ORcp42_178 = OMcp42_228*RLcp42_378-OMcp42_328*RLcp42_278;
    ORcp42_278 = -(OMcp42_128*RLcp42_378-OMcp42_328*RLcp42_178);
    ORcp42_378 = OMcp42_128*RLcp42_278-OMcp42_228*RLcp42_178;
    VIcp42_178 = ORcp42_119+ORcp42_120+ORcp42_121+ORcp42_122+ORcp42_123+ORcp42_124+ORcp42_125+ORcp42_126+ORcp42_127+
 ORcp42_128+ORcp42_178+qd[1];
    VIcp42_278 = ORcp42_219+ORcp42_220+ORcp42_221+ORcp42_222+ORcp42_223+ORcp42_224+ORcp42_225+ORcp42_226+ORcp42_227+
 ORcp42_228+ORcp42_278+qd[2];
    VIcp42_378 = ORcp42_319+ORcp42_320+ORcp42_321+ORcp42_322+ORcp42_323+ORcp42_324+ORcp42_325+ORcp42_326+ORcp42_327+
 ORcp42_328+ORcp42_378+qd[3];
    ACcp42_178 = qdd[1]+OMcp42_219*ORcp42_320+OMcp42_220*ORcp42_321+OMcp42_221*ORcp42_322+OMcp42_222*ORcp42_323+OMcp42_223
 *ORcp42_324+OMcp42_224*ORcp42_325+OMcp42_225*ORcp42_326+OMcp42_226*ORcp42_327+OMcp42_227*ORcp42_328+OMcp42_228*ORcp42_378+
 OMcp42_26*ORcp42_319-OMcp42_319*ORcp42_220-OMcp42_320*ORcp42_221-OMcp42_321*ORcp42_222-OMcp42_322*ORcp42_223-OMcp42_323*
 ORcp42_224-OMcp42_324*ORcp42_225-OMcp42_325*ORcp42_226-OMcp42_326*ORcp42_227-OMcp42_327*ORcp42_228-OMcp42_328*ORcp42_278-
 OMcp42_36*ORcp42_219+OPcp42_219*RLcp42_320+OPcp42_220*RLcp42_321+OPcp42_221*RLcp42_322+OPcp42_222*RLcp42_323+OPcp42_223*
 RLcp42_324+OPcp42_224*RLcp42_325+OPcp42_225*RLcp42_326+OPcp42_226*RLcp42_327+OPcp42_227*RLcp42_328+OPcp42_228*RLcp42_378+
 OPcp42_26*RLcp42_319-OPcp42_319*RLcp42_220-OPcp42_320*RLcp42_221-OPcp42_321*RLcp42_222-OPcp42_322*RLcp42_223-OPcp42_323*
 RLcp42_224-OPcp42_324*RLcp42_225-OPcp42_325*RLcp42_226-OPcp42_326*RLcp42_227-OPcp42_327*RLcp42_228-OPcp42_328*RLcp42_278-
 OPcp42_36*RLcp42_219;
    ACcp42_278 = qdd[2]-OMcp42_119*ORcp42_320-OMcp42_120*ORcp42_321-OMcp42_121*ORcp42_322-OMcp42_122*ORcp42_323-OMcp42_123
 *ORcp42_324-OMcp42_124*ORcp42_325-OMcp42_125*ORcp42_326-OMcp42_126*ORcp42_327-OMcp42_127*ORcp42_328-OMcp42_128*ORcp42_378-
 OMcp42_16*ORcp42_319+OMcp42_319*ORcp42_120+OMcp42_320*ORcp42_121+OMcp42_321*ORcp42_122+OMcp42_322*ORcp42_123+OMcp42_323*
 ORcp42_124+OMcp42_324*ORcp42_125+OMcp42_325*ORcp42_126+OMcp42_326*ORcp42_127+OMcp42_327*ORcp42_128+OMcp42_328*ORcp42_178+
 OMcp42_36*ORcp42_119-OPcp42_119*RLcp42_320-OPcp42_120*RLcp42_321-OPcp42_121*RLcp42_322-OPcp42_122*RLcp42_323-OPcp42_123*
 RLcp42_324-OPcp42_124*RLcp42_325-OPcp42_125*RLcp42_326-OPcp42_126*RLcp42_327-OPcp42_127*RLcp42_328-OPcp42_128*RLcp42_378-
 OPcp42_16*RLcp42_319+OPcp42_319*RLcp42_120+OPcp42_320*RLcp42_121+OPcp42_321*RLcp42_122+OPcp42_322*RLcp42_123+OPcp42_323*
 RLcp42_124+OPcp42_324*RLcp42_125+OPcp42_325*RLcp42_126+OPcp42_326*RLcp42_127+OPcp42_327*RLcp42_128+OPcp42_328*RLcp42_178+
 OPcp42_36*RLcp42_119;
    ACcp42_378 = qdd[3]+OMcp42_119*ORcp42_220+OMcp42_120*ORcp42_221+OMcp42_121*ORcp42_222+OMcp42_122*ORcp42_223+OMcp42_123
 *ORcp42_224+OMcp42_124*ORcp42_225+OMcp42_125*ORcp42_226+OMcp42_126*ORcp42_227+OMcp42_127*ORcp42_228+OMcp42_128*ORcp42_278+
 OMcp42_16*ORcp42_219-OMcp42_219*ORcp42_120-OMcp42_220*ORcp42_121-OMcp42_221*ORcp42_122-OMcp42_222*ORcp42_123-OMcp42_223*
 ORcp42_124-OMcp42_224*ORcp42_125-OMcp42_225*ORcp42_126-OMcp42_226*ORcp42_127-OMcp42_227*ORcp42_128-OMcp42_228*ORcp42_178-
 OMcp42_26*ORcp42_119+OPcp42_119*RLcp42_220+OPcp42_120*RLcp42_221+OPcp42_121*RLcp42_222+OPcp42_122*RLcp42_223+OPcp42_123*
 RLcp42_224+OPcp42_124*RLcp42_225+OPcp42_125*RLcp42_226+OPcp42_126*RLcp42_227+OPcp42_127*RLcp42_228+OPcp42_128*RLcp42_278+
 OPcp42_16*RLcp42_219-OPcp42_219*RLcp42_120-OPcp42_220*RLcp42_121-OPcp42_221*RLcp42_122-OPcp42_222*RLcp42_123-OPcp42_223*
 RLcp42_124-OPcp42_224*RLcp42_125-OPcp42_225*RLcp42_126-OPcp42_226*RLcp42_127-OPcp42_227*RLcp42_128-OPcp42_228*RLcp42_178-
 OPcp42_26*RLcp42_119;

// = = Block_1_0_0_43_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp42_178;
    sens->P[2] = POcp42_278;
    sens->P[3] = POcp42_378;
    sens->R[1][1] = ROcp42_127;
    sens->R[1][2] = ROcp42_227;
    sens->R[1][3] = ROcp42_327;
    sens->R[2][1] = ROcp42_428;
    sens->R[2][2] = ROcp42_528;
    sens->R[2][3] = ROcp42_628;
    sens->R[3][1] = ROcp42_728;
    sens->R[3][2] = ROcp42_828;
    sens->R[3][3] = ROcp42_928;
    sens->V[1] = VIcp42_178;
    sens->V[2] = VIcp42_278;
    sens->V[3] = VIcp42_378;
    sens->OM[1] = OMcp42_128;
    sens->OM[2] = OMcp42_228;
    sens->OM[3] = OMcp42_328;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp42_178_5;
    sens->J[1][6] = JTcp42_178_6;
    sens->J[1][19] = JTcp42_178_7;
    sens->J[1][20] = JTcp42_178_8;
    sens->J[1][21] = JTcp42_178_9;
    sens->J[1][22] = JTcp42_178_10;
    sens->J[1][23] = JTcp42_178_11;
    sens->J[1][24] = JTcp42_178_12;
    sens->J[1][25] = JTcp42_178_13;
    sens->J[1][26] = JTcp42_178_14;
    sens->J[1][27] = JTcp42_178_15;
    sens->J[1][28] = JTcp42_178_16;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp42_278_4;
    sens->J[2][5] = JTcp42_278_5;
    sens->J[2][6] = JTcp42_278_6;
    sens->J[2][19] = JTcp42_278_7;
    sens->J[2][20] = JTcp42_278_8;
    sens->J[2][21] = JTcp42_278_9;
    sens->J[2][22] = JTcp42_278_10;
    sens->J[2][23] = JTcp42_278_11;
    sens->J[2][24] = JTcp42_278_12;
    sens->J[2][25] = JTcp42_278_13;
    sens->J[2][26] = JTcp42_278_14;
    sens->J[2][27] = JTcp42_278_15;
    sens->J[2][28] = JTcp42_278_16;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp42_378_4;
    sens->J[3][5] = JTcp42_378_5;
    sens->J[3][6] = JTcp42_378_6;
    sens->J[3][19] = JTcp42_378_7;
    sens->J[3][20] = JTcp42_378_8;
    sens->J[3][21] = JTcp42_378_9;
    sens->J[3][22] = JTcp42_378_10;
    sens->J[3][23] = JTcp42_378_11;
    sens->J[3][24] = JTcp42_378_12;
    sens->J[3][25] = JTcp42_378_13;
    sens->J[3][26] = JTcp42_378_14;
    sens->J[3][27] = JTcp42_378_15;
    sens->J[3][28] = JTcp42_378_16;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp42_16;
    sens->J[4][20] = ROcp42_419;
    sens->J[4][21] = ROcp42_720;
    sens->J[4][22] = ROcp42_421;
    sens->J[4][23] = ROcp42_122;
    sens->J[4][24] = ROcp42_423;
    sens->J[4][25] = ROcp42_724;
    sens->J[4][26] = ROcp42_425;
    sens->J[4][27] = ROcp42_726;
    sens->J[4][28] = ROcp42_127;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp42_85;
    sens->J[5][19] = ROcp42_26;
    sens->J[5][20] = ROcp42_519;
    sens->J[5][21] = ROcp42_820;
    sens->J[5][22] = ROcp42_521;
    sens->J[5][23] = ROcp42_222;
    sens->J[5][24] = ROcp42_523;
    sens->J[5][25] = ROcp42_824;
    sens->J[5][26] = ROcp42_525;
    sens->J[5][27] = ROcp42_826;
    sens->J[5][28] = ROcp42_227;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp42_95;
    sens->J[6][19] = ROcp42_36;
    sens->J[6][20] = ROcp42_619;
    sens->J[6][21] = ROcp42_920;
    sens->J[6][22] = ROcp42_621;
    sens->J[6][23] = ROcp42_322;
    sens->J[6][24] = ROcp42_623;
    sens->J[6][25] = ROcp42_924;
    sens->J[6][26] = ROcp42_625;
    sens->J[6][27] = ROcp42_926;
    sens->J[6][28] = ROcp42_327;
    sens->A[1] = ACcp42_178;
    sens->A[2] = ACcp42_278;
    sens->A[3] = ACcp42_378;
    sens->OMP[1] = OPcp42_128;
    sens->OMP[2] = OPcp42_228;
    sens->OMP[3] = OPcp42_328;
 
// 
break;
case 44:
 


// = = Block_1_0_0_44_0_1 = = 
 
// Sensor Kinematics 


    ROcp43_25 = S4*S5;
    ROcp43_35 = -C4*S5;
    ROcp43_85 = -S4*C5;
    ROcp43_95 = C4*C5;
    ROcp43_16 = C5*C6;
    ROcp43_26 = ROcp43_25*C6+C4*S6;
    ROcp43_36 = ROcp43_35*C6+S4*S6;
    ROcp43_46 = -C5*S6;
    ROcp43_56 = -(ROcp43_25*S6-C4*C6);
    ROcp43_66 = -(ROcp43_35*S6-S4*C6);
    OMcp43_25 = qd[5]*C4;
    OMcp43_35 = qd[5]*S4;
    OMcp43_16 = qd[4]+qd[6]*S5;
    OMcp43_26 = OMcp43_25+ROcp43_85*qd[6];
    OMcp43_36 = OMcp43_35+ROcp43_95*qd[6];
    OPcp43_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp43_26 = ROcp43_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp43_35*S5-ROcp43_95*qd[4]);
    OPcp43_36 = ROcp43_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp43_25*S5-ROcp43_85*qd[4]);

// = = Block_1_0_0_44_0_4 = = 
 
// Sensor Kinematics 


    ROcp43_419 = ROcp43_46*C19+S19*S5;
    ROcp43_519 = ROcp43_56*C19+ROcp43_85*S19;
    ROcp43_619 = ROcp43_66*C19+ROcp43_95*S19;
    ROcp43_719 = -(ROcp43_46*S19-C19*S5);
    ROcp43_819 = -(ROcp43_56*S19-ROcp43_85*C19);
    ROcp43_919 = -(ROcp43_66*S19-ROcp43_95*C19);
    ROcp43_120 = ROcp43_16*C20-ROcp43_719*S20;
    ROcp43_220 = ROcp43_26*C20-ROcp43_819*S20;
    ROcp43_320 = ROcp43_36*C20-ROcp43_919*S20;
    ROcp43_720 = ROcp43_16*S20+ROcp43_719*C20;
    ROcp43_820 = ROcp43_26*S20+ROcp43_819*C20;
    ROcp43_920 = ROcp43_36*S20+ROcp43_919*C20;
    ROcp43_121 = ROcp43_120*C21+ROcp43_419*S21;
    ROcp43_221 = ROcp43_220*C21+ROcp43_519*S21;
    ROcp43_321 = ROcp43_320*C21+ROcp43_619*S21;
    ROcp43_421 = -(ROcp43_120*S21-ROcp43_419*C21);
    ROcp43_521 = -(ROcp43_220*S21-ROcp43_519*C21);
    ROcp43_621 = -(ROcp43_320*S21-ROcp43_619*C21);
    RLcp43_119 = s->dpt[2][3]*ROcp43_46+ROcp43_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp43_219 = s->dpt[2][3]*ROcp43_56+ROcp43_26*s->dpt[1][3]+ROcp43_85*s->dpt[3][3];
    RLcp43_319 = s->dpt[2][3]*ROcp43_66+ROcp43_36*s->dpt[1][3]+ROcp43_95*s->dpt[3][3];
    OMcp43_119 = OMcp43_16+ROcp43_16*qd[19];
    OMcp43_219 = OMcp43_26+ROcp43_26*qd[19];
    OMcp43_319 = OMcp43_36+ROcp43_36*qd[19];
    ORcp43_119 = OMcp43_26*RLcp43_319-OMcp43_36*RLcp43_219;
    ORcp43_219 = -(OMcp43_16*RLcp43_319-OMcp43_36*RLcp43_119);
    ORcp43_319 = OMcp43_16*RLcp43_219-OMcp43_26*RLcp43_119;
    OPcp43_119 = OPcp43_16+ROcp43_16*qdd[19]+qd[19]*(OMcp43_26*ROcp43_36-OMcp43_36*ROcp43_26);
    OPcp43_219 = OPcp43_26+ROcp43_26*qdd[19]-qd[19]*(OMcp43_16*ROcp43_36-OMcp43_36*ROcp43_16);
    OPcp43_319 = OPcp43_36+ROcp43_36*qdd[19]+qd[19]*(OMcp43_16*ROcp43_26-OMcp43_26*ROcp43_16);
    RLcp43_120 = s->dpt[1][38]*ROcp43_16+s->dpt[2][38]*ROcp43_419+s->dpt[3][38]*ROcp43_719;
    RLcp43_220 = s->dpt[1][38]*ROcp43_26+s->dpt[2][38]*ROcp43_519+s->dpt[3][38]*ROcp43_819;
    RLcp43_320 = s->dpt[1][38]*ROcp43_36+s->dpt[2][38]*ROcp43_619+s->dpt[3][38]*ROcp43_919;
    OMcp43_120 = OMcp43_119+ROcp43_419*qd[20];
    OMcp43_220 = OMcp43_219+ROcp43_519*qd[20];
    OMcp43_320 = OMcp43_319+ROcp43_619*qd[20];
    ORcp43_120 = OMcp43_219*RLcp43_320-OMcp43_319*RLcp43_220;
    ORcp43_220 = -(OMcp43_119*RLcp43_320-OMcp43_319*RLcp43_120);
    ORcp43_320 = OMcp43_119*RLcp43_220-OMcp43_219*RLcp43_120;
    OPcp43_120 = OPcp43_119+ROcp43_419*qdd[20]+qd[20]*(OMcp43_219*ROcp43_619-OMcp43_319*ROcp43_519);
    OPcp43_220 = OPcp43_219+ROcp43_519*qdd[20]-qd[20]*(OMcp43_119*ROcp43_619-OMcp43_319*ROcp43_419);
    OPcp43_320 = OPcp43_319+ROcp43_619*qdd[20]+qd[20]*(OMcp43_119*ROcp43_519-OMcp43_219*ROcp43_419);
    RLcp43_121 = s->dpt[1][40]*ROcp43_120+s->dpt[2][40]*ROcp43_419+ROcp43_720*s->dpt[3][40];
    RLcp43_221 = s->dpt[1][40]*ROcp43_220+s->dpt[2][40]*ROcp43_519+ROcp43_820*s->dpt[3][40];
    RLcp43_321 = s->dpt[1][40]*ROcp43_320+s->dpt[2][40]*ROcp43_619+ROcp43_920*s->dpt[3][40];
    OMcp43_121 = OMcp43_120+ROcp43_720*qd[21];
    OMcp43_221 = OMcp43_220+ROcp43_820*qd[21];
    OMcp43_321 = OMcp43_320+ROcp43_920*qd[21];
    ORcp43_121 = OMcp43_220*RLcp43_321-OMcp43_320*RLcp43_221;
    ORcp43_221 = -(OMcp43_120*RLcp43_321-OMcp43_320*RLcp43_121);
    ORcp43_321 = OMcp43_120*RLcp43_221-OMcp43_220*RLcp43_121;
    OPcp43_121 = OPcp43_120+ROcp43_720*qdd[21]+qd[21]*(OMcp43_220*ROcp43_920-OMcp43_320*ROcp43_820);
    OPcp43_221 = OPcp43_220+ROcp43_820*qdd[21]-qd[21]*(OMcp43_120*ROcp43_920-OMcp43_320*ROcp43_720);
    OPcp43_321 = OPcp43_320+ROcp43_920*qdd[21]+qd[21]*(OMcp43_120*ROcp43_820-OMcp43_220*ROcp43_720);

// = = Block_1_0_0_44_0_6 = = 
 
// Sensor Kinematics 


    ROcp43_129 = ROcp43_121*C29-ROcp43_720*S29;
    ROcp43_229 = ROcp43_221*C29-ROcp43_820*S29;
    ROcp43_329 = ROcp43_321*C29-ROcp43_920*S29;
    ROcp43_729 = ROcp43_121*S29+ROcp43_720*C29;
    ROcp43_829 = ROcp43_221*S29+ROcp43_820*C29;
    ROcp43_929 = ROcp43_321*S29+ROcp43_920*C29;
    RLcp43_129 = ROcp43_121*s->dpt[1][45]+ROcp43_421*s->dpt[2][45]+ROcp43_720*s->dpt[3][45];
    RLcp43_229 = ROcp43_221*s->dpt[1][45]+ROcp43_521*s->dpt[2][45]+ROcp43_820*s->dpt[3][45];
    RLcp43_329 = ROcp43_321*s->dpt[1][45]+ROcp43_621*s->dpt[2][45]+ROcp43_920*s->dpt[3][45];
    OMcp43_129 = OMcp43_121+ROcp43_421*qd[29];
    OMcp43_229 = OMcp43_221+ROcp43_521*qd[29];
    OMcp43_329 = OMcp43_321+ROcp43_621*qd[29];
    ORcp43_129 = OMcp43_221*RLcp43_329-OMcp43_321*RLcp43_229;
    ORcp43_229 = -(OMcp43_121*RLcp43_329-OMcp43_321*RLcp43_129);
    ORcp43_329 = OMcp43_121*RLcp43_229-OMcp43_221*RLcp43_129;
    OPcp43_129 = OPcp43_121+ROcp43_421*qdd[29]+qd[29]*(OMcp43_221*ROcp43_621-OMcp43_321*ROcp43_521);
    OPcp43_229 = OPcp43_221+ROcp43_521*qdd[29]-qd[29]*(OMcp43_121*ROcp43_621-OMcp43_321*ROcp43_421);
    OPcp43_329 = OPcp43_321+ROcp43_621*qdd[29]+qd[29]*(OMcp43_121*ROcp43_521-OMcp43_221*ROcp43_421);
    RLcp43_179 = ROcp43_129*s->dpt[1][58]+ROcp43_421*s->dpt[2][58]+ROcp43_729*s->dpt[3][58];
    RLcp43_279 = ROcp43_229*s->dpt[1][58]+ROcp43_521*s->dpt[2][58]+ROcp43_829*s->dpt[3][58];
    RLcp43_379 = ROcp43_329*s->dpt[1][58]+ROcp43_621*s->dpt[2][58]+ROcp43_929*s->dpt[3][58];
    POcp43_179 = RLcp43_119+RLcp43_120+RLcp43_121+RLcp43_129+RLcp43_179+q[1];
    POcp43_279 = RLcp43_219+RLcp43_220+RLcp43_221+RLcp43_229+RLcp43_279+q[2];
    POcp43_379 = RLcp43_319+RLcp43_320+RLcp43_321+RLcp43_329+RLcp43_379+q[3];
    JTcp43_279_4 = -(RLcp43_319+RLcp43_320+RLcp43_321+RLcp43_329+RLcp43_379);
    JTcp43_379_4 = RLcp43_219+RLcp43_220+RLcp43_221+RLcp43_229+RLcp43_279;
    JTcp43_179_5 = C4*(RLcp43_319+RLcp43_320+RLcp43_321+RLcp43_329)-S4*(RLcp43_219+RLcp43_220)-S4*(RLcp43_221+RLcp43_229)-
 RLcp43_279*S4+RLcp43_379*C4;
    JTcp43_279_5 = S4*(RLcp43_119+RLcp43_120+RLcp43_121+RLcp43_129+RLcp43_179);
    JTcp43_379_5 = -C4*(RLcp43_119+RLcp43_120+RLcp43_121+RLcp43_129+RLcp43_179);
    JTcp43_179_6 = ROcp43_85*(RLcp43_319+RLcp43_320+RLcp43_321+RLcp43_329)-ROcp43_95*(RLcp43_219+RLcp43_220)-ROcp43_95*(
 RLcp43_221+RLcp43_229)-RLcp43_279*ROcp43_95+RLcp43_379*ROcp43_85;
    JTcp43_279_6 = -(RLcp43_379*S5-ROcp43_95*(RLcp43_119+RLcp43_120+RLcp43_121+RLcp43_129+RLcp43_179)+S5*(RLcp43_319+
 RLcp43_320)+S5*(RLcp43_321+RLcp43_329));
    JTcp43_379_6 = RLcp43_279*S5-ROcp43_85*(RLcp43_119+RLcp43_120+RLcp43_121+RLcp43_129+RLcp43_179)+S5*(RLcp43_219+
 RLcp43_220)+S5*(RLcp43_221+RLcp43_229);
    JTcp43_179_7 = ROcp43_26*(RLcp43_320+RLcp43_321+RLcp43_329+RLcp43_379)-ROcp43_36*(RLcp43_220+RLcp43_221)-ROcp43_36*(
 RLcp43_229+RLcp43_279);
    JTcp43_279_7 = -(ROcp43_16*(RLcp43_320+RLcp43_321+RLcp43_329+RLcp43_379)-ROcp43_36*(RLcp43_120+RLcp43_121)-ROcp43_36*(
 RLcp43_129+RLcp43_179));
    JTcp43_379_7 = ROcp43_16*(RLcp43_220+RLcp43_221+RLcp43_229+RLcp43_279)-ROcp43_26*(RLcp43_120+RLcp43_121)-ROcp43_26*(
 RLcp43_129+RLcp43_179);
    JTcp43_179_8 = ROcp43_519*(RLcp43_321+RLcp43_329)-ROcp43_619*(RLcp43_221+RLcp43_229)-RLcp43_279*ROcp43_619+RLcp43_379*
 ROcp43_519;
    JTcp43_279_8 = RLcp43_179*ROcp43_619-RLcp43_379*ROcp43_419-ROcp43_419*(RLcp43_321+RLcp43_329)+ROcp43_619*(RLcp43_121+
 RLcp43_129);
    JTcp43_379_8 = ROcp43_419*(RLcp43_221+RLcp43_229)-ROcp43_519*(RLcp43_121+RLcp43_129)-RLcp43_179*ROcp43_519+RLcp43_279*
 ROcp43_419;
    JTcp43_179_9 = ROcp43_820*(RLcp43_329+RLcp43_379)-ROcp43_920*(RLcp43_229+RLcp43_279);
    JTcp43_279_9 = -(ROcp43_720*(RLcp43_329+RLcp43_379)-ROcp43_920*(RLcp43_129+RLcp43_179));
    JTcp43_379_9 = ROcp43_720*(RLcp43_229+RLcp43_279)-ROcp43_820*(RLcp43_129+RLcp43_179);
    JTcp43_179_10 = -(RLcp43_279*ROcp43_621-RLcp43_379*ROcp43_521);
    JTcp43_279_10 = RLcp43_179*ROcp43_621-RLcp43_379*ROcp43_421;
    JTcp43_379_10 = -(RLcp43_179*ROcp43_521-RLcp43_279*ROcp43_421);
    ORcp43_179 = OMcp43_229*RLcp43_379-OMcp43_329*RLcp43_279;
    ORcp43_279 = -(OMcp43_129*RLcp43_379-OMcp43_329*RLcp43_179);
    ORcp43_379 = OMcp43_129*RLcp43_279-OMcp43_229*RLcp43_179;
    VIcp43_179 = ORcp43_119+ORcp43_120+ORcp43_121+ORcp43_129+ORcp43_179+qd[1];
    VIcp43_279 = ORcp43_219+ORcp43_220+ORcp43_221+ORcp43_229+ORcp43_279+qd[2];
    VIcp43_379 = ORcp43_319+ORcp43_320+ORcp43_321+ORcp43_329+ORcp43_379+qd[3];
    ACcp43_179 = qdd[1]+OMcp43_219*ORcp43_320+OMcp43_220*ORcp43_321+OMcp43_221*ORcp43_329+OMcp43_229*ORcp43_379+OMcp43_26*
 ORcp43_319-OMcp43_319*ORcp43_220-OMcp43_320*ORcp43_221-OMcp43_321*ORcp43_229-OMcp43_329*ORcp43_279-OMcp43_36*ORcp43_219+
 OPcp43_219*RLcp43_320+OPcp43_220*RLcp43_321+OPcp43_221*RLcp43_329+OPcp43_229*RLcp43_379+OPcp43_26*RLcp43_319-OPcp43_319*
 RLcp43_220-OPcp43_320*RLcp43_221-OPcp43_321*RLcp43_229-OPcp43_329*RLcp43_279-OPcp43_36*RLcp43_219;
    ACcp43_279 = qdd[2]-OMcp43_119*ORcp43_320-OMcp43_120*ORcp43_321-OMcp43_121*ORcp43_329-OMcp43_129*ORcp43_379-OMcp43_16*
 ORcp43_319+OMcp43_319*ORcp43_120+OMcp43_320*ORcp43_121+OMcp43_321*ORcp43_129+OMcp43_329*ORcp43_179+OMcp43_36*ORcp43_119-
 OPcp43_119*RLcp43_320-OPcp43_120*RLcp43_321-OPcp43_121*RLcp43_329-OPcp43_129*RLcp43_379-OPcp43_16*RLcp43_319+OPcp43_319*
 RLcp43_120+OPcp43_320*RLcp43_121+OPcp43_321*RLcp43_129+OPcp43_329*RLcp43_179+OPcp43_36*RLcp43_119;
    ACcp43_379 = qdd[3]+OMcp43_119*ORcp43_220+OMcp43_120*ORcp43_221+OMcp43_121*ORcp43_229+OMcp43_129*ORcp43_279+OMcp43_16*
 ORcp43_219-OMcp43_219*ORcp43_120-OMcp43_220*ORcp43_121-OMcp43_221*ORcp43_129-OMcp43_229*ORcp43_179-OMcp43_26*ORcp43_119+
 OPcp43_119*RLcp43_220+OPcp43_120*RLcp43_221+OPcp43_121*RLcp43_229+OPcp43_129*RLcp43_279+OPcp43_16*RLcp43_219-OPcp43_219*
 RLcp43_120-OPcp43_220*RLcp43_121-OPcp43_221*RLcp43_129-OPcp43_229*RLcp43_179-OPcp43_26*RLcp43_119;

// = = Block_1_0_0_44_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp43_179;
    sens->P[2] = POcp43_279;
    sens->P[3] = POcp43_379;
    sens->R[1][1] = ROcp43_129;
    sens->R[1][2] = ROcp43_229;
    sens->R[1][3] = ROcp43_329;
    sens->R[2][1] = ROcp43_421;
    sens->R[2][2] = ROcp43_521;
    sens->R[2][3] = ROcp43_621;
    sens->R[3][1] = ROcp43_729;
    sens->R[3][2] = ROcp43_829;
    sens->R[3][3] = ROcp43_929;
    sens->V[1] = VIcp43_179;
    sens->V[2] = VIcp43_279;
    sens->V[3] = VIcp43_379;
    sens->OM[1] = OMcp43_129;
    sens->OM[2] = OMcp43_229;
    sens->OM[3] = OMcp43_329;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp43_179_5;
    sens->J[1][6] = JTcp43_179_6;
    sens->J[1][19] = JTcp43_179_7;
    sens->J[1][20] = JTcp43_179_8;
    sens->J[1][21] = JTcp43_179_9;
    sens->J[1][29] = JTcp43_179_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp43_279_4;
    sens->J[2][5] = JTcp43_279_5;
    sens->J[2][6] = JTcp43_279_6;
    sens->J[2][19] = JTcp43_279_7;
    sens->J[2][20] = JTcp43_279_8;
    sens->J[2][21] = JTcp43_279_9;
    sens->J[2][29] = JTcp43_279_10;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp43_379_4;
    sens->J[3][5] = JTcp43_379_5;
    sens->J[3][6] = JTcp43_379_6;
    sens->J[3][19] = JTcp43_379_7;
    sens->J[3][20] = JTcp43_379_8;
    sens->J[3][21] = JTcp43_379_9;
    sens->J[3][29] = JTcp43_379_10;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp43_16;
    sens->J[4][20] = ROcp43_419;
    sens->J[4][21] = ROcp43_720;
    sens->J[4][29] = ROcp43_421;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp43_85;
    sens->J[5][19] = ROcp43_26;
    sens->J[5][20] = ROcp43_519;
    sens->J[5][21] = ROcp43_820;
    sens->J[5][29] = ROcp43_521;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp43_95;
    sens->J[6][19] = ROcp43_36;
    sens->J[6][20] = ROcp43_619;
    sens->J[6][21] = ROcp43_920;
    sens->J[6][29] = ROcp43_621;
    sens->A[1] = ACcp43_179;
    sens->A[2] = ACcp43_279;
    sens->A[3] = ACcp43_379;
    sens->OMP[1] = OPcp43_129;
    sens->OMP[2] = OPcp43_229;
    sens->OMP[3] = OPcp43_329;
 
// 
break;
case 45:
 


// = = Block_1_0_0_45_0_1 = = 
 
// Sensor Kinematics 


    ROcp44_25 = S4*S5;
    ROcp44_35 = -C4*S5;
    ROcp44_85 = -S4*C5;
    ROcp44_95 = C4*C5;
    ROcp44_16 = C5*C6;
    ROcp44_26 = ROcp44_25*C6+C4*S6;
    ROcp44_36 = ROcp44_35*C6+S4*S6;
    ROcp44_46 = -C5*S6;
    ROcp44_56 = -(ROcp44_25*S6-C4*C6);
    ROcp44_66 = -(ROcp44_35*S6-S4*C6);
    OMcp44_25 = qd[5]*C4;
    OMcp44_35 = qd[5]*S4;
    OMcp44_16 = qd[4]+qd[6]*S5;
    OMcp44_26 = OMcp44_25+ROcp44_85*qd[6];
    OMcp44_36 = OMcp44_35+ROcp44_95*qd[6];
    OPcp44_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp44_26 = ROcp44_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp44_35*S5-ROcp44_95*qd[4]);
    OPcp44_36 = ROcp44_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp44_25*S5-ROcp44_85*qd[4]);

// = = Block_1_0_0_45_0_4 = = 
 
// Sensor Kinematics 


    ROcp44_419 = ROcp44_46*C19+S19*S5;
    ROcp44_519 = ROcp44_56*C19+ROcp44_85*S19;
    ROcp44_619 = ROcp44_66*C19+ROcp44_95*S19;
    ROcp44_719 = -(ROcp44_46*S19-C19*S5);
    ROcp44_819 = -(ROcp44_56*S19-ROcp44_85*C19);
    ROcp44_919 = -(ROcp44_66*S19-ROcp44_95*C19);
    ROcp44_120 = ROcp44_16*C20-ROcp44_719*S20;
    ROcp44_220 = ROcp44_26*C20-ROcp44_819*S20;
    ROcp44_320 = ROcp44_36*C20-ROcp44_919*S20;
    ROcp44_720 = ROcp44_16*S20+ROcp44_719*C20;
    ROcp44_820 = ROcp44_26*S20+ROcp44_819*C20;
    ROcp44_920 = ROcp44_36*S20+ROcp44_919*C20;
    ROcp44_121 = ROcp44_120*C21+ROcp44_419*S21;
    ROcp44_221 = ROcp44_220*C21+ROcp44_519*S21;
    ROcp44_321 = ROcp44_320*C21+ROcp44_619*S21;
    ROcp44_421 = -(ROcp44_120*S21-ROcp44_419*C21);
    ROcp44_521 = -(ROcp44_220*S21-ROcp44_519*C21);
    ROcp44_621 = -(ROcp44_320*S21-ROcp44_619*C21);
    RLcp44_119 = s->dpt[2][3]*ROcp44_46+ROcp44_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp44_219 = s->dpt[2][3]*ROcp44_56+ROcp44_26*s->dpt[1][3]+ROcp44_85*s->dpt[3][3];
    RLcp44_319 = s->dpt[2][3]*ROcp44_66+ROcp44_36*s->dpt[1][3]+ROcp44_95*s->dpt[3][3];
    OMcp44_119 = OMcp44_16+ROcp44_16*qd[19];
    OMcp44_219 = OMcp44_26+ROcp44_26*qd[19];
    OMcp44_319 = OMcp44_36+ROcp44_36*qd[19];
    ORcp44_119 = OMcp44_26*RLcp44_319-OMcp44_36*RLcp44_219;
    ORcp44_219 = -(OMcp44_16*RLcp44_319-OMcp44_36*RLcp44_119);
    ORcp44_319 = OMcp44_16*RLcp44_219-OMcp44_26*RLcp44_119;
    OPcp44_119 = OPcp44_16+ROcp44_16*qdd[19]+qd[19]*(OMcp44_26*ROcp44_36-OMcp44_36*ROcp44_26);
    OPcp44_219 = OPcp44_26+ROcp44_26*qdd[19]-qd[19]*(OMcp44_16*ROcp44_36-OMcp44_36*ROcp44_16);
    OPcp44_319 = OPcp44_36+ROcp44_36*qdd[19]+qd[19]*(OMcp44_16*ROcp44_26-OMcp44_26*ROcp44_16);
    RLcp44_120 = s->dpt[1][38]*ROcp44_16+s->dpt[2][38]*ROcp44_419+s->dpt[3][38]*ROcp44_719;
    RLcp44_220 = s->dpt[1][38]*ROcp44_26+s->dpt[2][38]*ROcp44_519+s->dpt[3][38]*ROcp44_819;
    RLcp44_320 = s->dpt[1][38]*ROcp44_36+s->dpt[2][38]*ROcp44_619+s->dpt[3][38]*ROcp44_919;
    OMcp44_120 = OMcp44_119+ROcp44_419*qd[20];
    OMcp44_220 = OMcp44_219+ROcp44_519*qd[20];
    OMcp44_320 = OMcp44_319+ROcp44_619*qd[20];
    ORcp44_120 = OMcp44_219*RLcp44_320-OMcp44_319*RLcp44_220;
    ORcp44_220 = -(OMcp44_119*RLcp44_320-OMcp44_319*RLcp44_120);
    ORcp44_320 = OMcp44_119*RLcp44_220-OMcp44_219*RLcp44_120;
    OPcp44_120 = OPcp44_119+ROcp44_419*qdd[20]+qd[20]*(OMcp44_219*ROcp44_619-OMcp44_319*ROcp44_519);
    OPcp44_220 = OPcp44_219+ROcp44_519*qdd[20]-qd[20]*(OMcp44_119*ROcp44_619-OMcp44_319*ROcp44_419);
    OPcp44_320 = OPcp44_319+ROcp44_619*qdd[20]+qd[20]*(OMcp44_119*ROcp44_519-OMcp44_219*ROcp44_419);
    RLcp44_121 = s->dpt[1][40]*ROcp44_120+s->dpt[2][40]*ROcp44_419+ROcp44_720*s->dpt[3][40];
    RLcp44_221 = s->dpt[1][40]*ROcp44_220+s->dpt[2][40]*ROcp44_519+ROcp44_820*s->dpt[3][40];
    RLcp44_321 = s->dpt[1][40]*ROcp44_320+s->dpt[2][40]*ROcp44_619+ROcp44_920*s->dpt[3][40];
    OMcp44_121 = OMcp44_120+ROcp44_720*qd[21];
    OMcp44_221 = OMcp44_220+ROcp44_820*qd[21];
    OMcp44_321 = OMcp44_320+ROcp44_920*qd[21];
    ORcp44_121 = OMcp44_220*RLcp44_321-OMcp44_320*RLcp44_221;
    ORcp44_221 = -(OMcp44_120*RLcp44_321-OMcp44_320*RLcp44_121);
    ORcp44_321 = OMcp44_120*RLcp44_221-OMcp44_220*RLcp44_121;
    OPcp44_121 = OPcp44_120+ROcp44_720*qdd[21]+qd[21]*(OMcp44_220*ROcp44_920-OMcp44_320*ROcp44_820);
    OPcp44_221 = OPcp44_220+ROcp44_820*qdd[21]-qd[21]*(OMcp44_120*ROcp44_920-OMcp44_320*ROcp44_720);
    OPcp44_321 = OPcp44_320+ROcp44_920*qdd[21]+qd[21]*(OMcp44_120*ROcp44_820-OMcp44_220*ROcp44_720);

// = = Block_1_0_0_45_0_6 = = 
 
// Sensor Kinematics 


    ROcp44_129 = ROcp44_121*C29-ROcp44_720*S29;
    ROcp44_229 = ROcp44_221*C29-ROcp44_820*S29;
    ROcp44_329 = ROcp44_321*C29-ROcp44_920*S29;
    ROcp44_729 = ROcp44_121*S29+ROcp44_720*C29;
    ROcp44_829 = ROcp44_221*S29+ROcp44_820*C29;
    ROcp44_929 = ROcp44_321*S29+ROcp44_920*C29;
    RLcp44_129 = ROcp44_121*s->dpt[1][45]+ROcp44_421*s->dpt[2][45]+ROcp44_720*s->dpt[3][45];
    RLcp44_229 = ROcp44_221*s->dpt[1][45]+ROcp44_521*s->dpt[2][45]+ROcp44_820*s->dpt[3][45];
    RLcp44_329 = ROcp44_321*s->dpt[1][45]+ROcp44_621*s->dpt[2][45]+ROcp44_920*s->dpt[3][45];
    OMcp44_129 = OMcp44_121+ROcp44_421*qd[29];
    OMcp44_229 = OMcp44_221+ROcp44_521*qd[29];
    OMcp44_329 = OMcp44_321+ROcp44_621*qd[29];
    ORcp44_129 = OMcp44_221*RLcp44_329-OMcp44_321*RLcp44_229;
    ORcp44_229 = -(OMcp44_121*RLcp44_329-OMcp44_321*RLcp44_129);
    ORcp44_329 = OMcp44_121*RLcp44_229-OMcp44_221*RLcp44_129;
    OPcp44_129 = OPcp44_121+ROcp44_421*qdd[29]+qd[29]*(OMcp44_221*ROcp44_621-OMcp44_321*ROcp44_521);
    OPcp44_229 = OPcp44_221+ROcp44_521*qdd[29]-qd[29]*(OMcp44_121*ROcp44_621-OMcp44_321*ROcp44_421);
    OPcp44_329 = OPcp44_321+ROcp44_621*qdd[29]+qd[29]*(OMcp44_121*ROcp44_521-OMcp44_221*ROcp44_421);
    RLcp44_180 = s->dpt[1][59]*ROcp44_129+s->dpt[3][59]*ROcp44_729+ROcp44_421*s->dpt[2][59];
    RLcp44_280 = s->dpt[1][59]*ROcp44_229+s->dpt[3][59]*ROcp44_829+ROcp44_521*s->dpt[2][59];
    RLcp44_380 = s->dpt[1][59]*ROcp44_329+s->dpt[3][59]*ROcp44_929+ROcp44_621*s->dpt[2][59];
    POcp44_180 = RLcp44_119+RLcp44_120+RLcp44_121+RLcp44_129+RLcp44_180+q[1];
    POcp44_280 = RLcp44_219+RLcp44_220+RLcp44_221+RLcp44_229+RLcp44_280+q[2];
    POcp44_380 = RLcp44_319+RLcp44_320+RLcp44_321+RLcp44_329+RLcp44_380+q[3];
    JTcp44_280_4 = -(RLcp44_319+RLcp44_320+RLcp44_321+RLcp44_329+RLcp44_380);
    JTcp44_380_4 = RLcp44_219+RLcp44_220+RLcp44_221+RLcp44_229+RLcp44_280;
    JTcp44_180_5 = C4*(RLcp44_319+RLcp44_320+RLcp44_321+RLcp44_329)-S4*(RLcp44_219+RLcp44_220)-S4*(RLcp44_221+RLcp44_229)-
 RLcp44_280*S4+RLcp44_380*C4;
    JTcp44_280_5 = S4*(RLcp44_119+RLcp44_120+RLcp44_121+RLcp44_129+RLcp44_180);
    JTcp44_380_5 = -C4*(RLcp44_119+RLcp44_120+RLcp44_121+RLcp44_129+RLcp44_180);
    JTcp44_180_6 = ROcp44_85*(RLcp44_319+RLcp44_320+RLcp44_321+RLcp44_329)-ROcp44_95*(RLcp44_219+RLcp44_220)-ROcp44_95*(
 RLcp44_221+RLcp44_229)-RLcp44_280*ROcp44_95+RLcp44_380*ROcp44_85;
    JTcp44_280_6 = -(RLcp44_380*S5-ROcp44_95*(RLcp44_119+RLcp44_120+RLcp44_121+RLcp44_129+RLcp44_180)+S5*(RLcp44_319+
 RLcp44_320)+S5*(RLcp44_321+RLcp44_329));
    JTcp44_380_6 = RLcp44_280*S5-ROcp44_85*(RLcp44_119+RLcp44_120+RLcp44_121+RLcp44_129+RLcp44_180)+S5*(RLcp44_219+
 RLcp44_220)+S5*(RLcp44_221+RLcp44_229);
    JTcp44_180_7 = ROcp44_26*(RLcp44_320+RLcp44_321+RLcp44_329+RLcp44_380)-ROcp44_36*(RLcp44_220+RLcp44_221)-ROcp44_36*(
 RLcp44_229+RLcp44_280);
    JTcp44_280_7 = -(ROcp44_16*(RLcp44_320+RLcp44_321+RLcp44_329+RLcp44_380)-ROcp44_36*(RLcp44_120+RLcp44_121)-ROcp44_36*(
 RLcp44_129+RLcp44_180));
    JTcp44_380_7 = ROcp44_16*(RLcp44_220+RLcp44_221+RLcp44_229+RLcp44_280)-ROcp44_26*(RLcp44_120+RLcp44_121)-ROcp44_26*(
 RLcp44_129+RLcp44_180);
    JTcp44_180_8 = ROcp44_519*(RLcp44_321+RLcp44_329)-ROcp44_619*(RLcp44_221+RLcp44_229)-RLcp44_280*ROcp44_619+RLcp44_380*
 ROcp44_519;
    JTcp44_280_8 = RLcp44_180*ROcp44_619-RLcp44_380*ROcp44_419-ROcp44_419*(RLcp44_321+RLcp44_329)+ROcp44_619*(RLcp44_121+
 RLcp44_129);
    JTcp44_380_8 = ROcp44_419*(RLcp44_221+RLcp44_229)-ROcp44_519*(RLcp44_121+RLcp44_129)-RLcp44_180*ROcp44_519+RLcp44_280*
 ROcp44_419;
    JTcp44_180_9 = ROcp44_820*(RLcp44_329+RLcp44_380)-ROcp44_920*(RLcp44_229+RLcp44_280);
    JTcp44_280_9 = -(ROcp44_720*(RLcp44_329+RLcp44_380)-ROcp44_920*(RLcp44_129+RLcp44_180));
    JTcp44_380_9 = ROcp44_720*(RLcp44_229+RLcp44_280)-ROcp44_820*(RLcp44_129+RLcp44_180);
    JTcp44_180_10 = -(RLcp44_280*ROcp44_621-RLcp44_380*ROcp44_521);
    JTcp44_280_10 = RLcp44_180*ROcp44_621-RLcp44_380*ROcp44_421;
    JTcp44_380_10 = -(RLcp44_180*ROcp44_521-RLcp44_280*ROcp44_421);
    ORcp44_180 = OMcp44_229*RLcp44_380-OMcp44_329*RLcp44_280;
    ORcp44_280 = -(OMcp44_129*RLcp44_380-OMcp44_329*RLcp44_180);
    ORcp44_380 = OMcp44_129*RLcp44_280-OMcp44_229*RLcp44_180;
    VIcp44_180 = ORcp44_119+ORcp44_120+ORcp44_121+ORcp44_129+ORcp44_180+qd[1];
    VIcp44_280 = ORcp44_219+ORcp44_220+ORcp44_221+ORcp44_229+ORcp44_280+qd[2];
    VIcp44_380 = ORcp44_319+ORcp44_320+ORcp44_321+ORcp44_329+ORcp44_380+qd[3];
    ACcp44_180 = qdd[1]+OMcp44_219*ORcp44_320+OMcp44_220*ORcp44_321+OMcp44_221*ORcp44_329+OMcp44_229*ORcp44_380+OMcp44_26*
 ORcp44_319-OMcp44_319*ORcp44_220-OMcp44_320*ORcp44_221-OMcp44_321*ORcp44_229-OMcp44_329*ORcp44_280-OMcp44_36*ORcp44_219+
 OPcp44_219*RLcp44_320+OPcp44_220*RLcp44_321+OPcp44_221*RLcp44_329+OPcp44_229*RLcp44_380+OPcp44_26*RLcp44_319-OPcp44_319*
 RLcp44_220-OPcp44_320*RLcp44_221-OPcp44_321*RLcp44_229-OPcp44_329*RLcp44_280-OPcp44_36*RLcp44_219;
    ACcp44_280 = qdd[2]-OMcp44_119*ORcp44_320-OMcp44_120*ORcp44_321-OMcp44_121*ORcp44_329-OMcp44_129*ORcp44_380-OMcp44_16*
 ORcp44_319+OMcp44_319*ORcp44_120+OMcp44_320*ORcp44_121+OMcp44_321*ORcp44_129+OMcp44_329*ORcp44_180+OMcp44_36*ORcp44_119-
 OPcp44_119*RLcp44_320-OPcp44_120*RLcp44_321-OPcp44_121*RLcp44_329-OPcp44_129*RLcp44_380-OPcp44_16*RLcp44_319+OPcp44_319*
 RLcp44_120+OPcp44_320*RLcp44_121+OPcp44_321*RLcp44_129+OPcp44_329*RLcp44_180+OPcp44_36*RLcp44_119;
    ACcp44_380 = qdd[3]+OMcp44_119*ORcp44_220+OMcp44_120*ORcp44_221+OMcp44_121*ORcp44_229+OMcp44_129*ORcp44_280+OMcp44_16*
 ORcp44_219-OMcp44_219*ORcp44_120-OMcp44_220*ORcp44_121-OMcp44_221*ORcp44_129-OMcp44_229*ORcp44_180-OMcp44_26*ORcp44_119+
 OPcp44_119*RLcp44_220+OPcp44_120*RLcp44_221+OPcp44_121*RLcp44_229+OPcp44_129*RLcp44_280+OPcp44_16*RLcp44_219-OPcp44_219*
 RLcp44_120-OPcp44_220*RLcp44_121-OPcp44_221*RLcp44_129-OPcp44_229*RLcp44_180-OPcp44_26*RLcp44_119;

// = = Block_1_0_0_45_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp44_180;
    sens->P[2] = POcp44_280;
    sens->P[3] = POcp44_380;
    sens->R[1][1] = ROcp44_129;
    sens->R[1][2] = ROcp44_229;
    sens->R[1][3] = ROcp44_329;
    sens->R[2][1] = ROcp44_421;
    sens->R[2][2] = ROcp44_521;
    sens->R[2][3] = ROcp44_621;
    sens->R[3][1] = ROcp44_729;
    sens->R[3][2] = ROcp44_829;
    sens->R[3][3] = ROcp44_929;
    sens->V[1] = VIcp44_180;
    sens->V[2] = VIcp44_280;
    sens->V[3] = VIcp44_380;
    sens->OM[1] = OMcp44_129;
    sens->OM[2] = OMcp44_229;
    sens->OM[3] = OMcp44_329;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp44_180_5;
    sens->J[1][6] = JTcp44_180_6;
    sens->J[1][19] = JTcp44_180_7;
    sens->J[1][20] = JTcp44_180_8;
    sens->J[1][21] = JTcp44_180_9;
    sens->J[1][29] = JTcp44_180_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp44_280_4;
    sens->J[2][5] = JTcp44_280_5;
    sens->J[2][6] = JTcp44_280_6;
    sens->J[2][19] = JTcp44_280_7;
    sens->J[2][20] = JTcp44_280_8;
    sens->J[2][21] = JTcp44_280_9;
    sens->J[2][29] = JTcp44_280_10;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp44_380_4;
    sens->J[3][5] = JTcp44_380_5;
    sens->J[3][6] = JTcp44_380_6;
    sens->J[3][19] = JTcp44_380_7;
    sens->J[3][20] = JTcp44_380_8;
    sens->J[3][21] = JTcp44_380_9;
    sens->J[3][29] = JTcp44_380_10;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp44_16;
    sens->J[4][20] = ROcp44_419;
    sens->J[4][21] = ROcp44_720;
    sens->J[4][29] = ROcp44_421;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp44_85;
    sens->J[5][19] = ROcp44_26;
    sens->J[5][20] = ROcp44_519;
    sens->J[5][21] = ROcp44_820;
    sens->J[5][29] = ROcp44_521;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp44_95;
    sens->J[6][19] = ROcp44_36;
    sens->J[6][20] = ROcp44_619;
    sens->J[6][21] = ROcp44_920;
    sens->J[6][29] = ROcp44_621;
    sens->A[1] = ACcp44_180;
    sens->A[2] = ACcp44_280;
    sens->A[3] = ACcp44_380;
    sens->OMP[1] = OPcp44_129;
    sens->OMP[2] = OPcp44_229;
    sens->OMP[3] = OPcp44_329;
 
// 
break;
case 46:
 


// = = Block_1_0_0_46_0_1 = = 
 
// Sensor Kinematics 


    ROcp45_25 = S4*S5;
    ROcp45_35 = -C4*S5;
    ROcp45_85 = -S4*C5;
    ROcp45_95 = C4*C5;
    ROcp45_16 = C5*C6;
    ROcp45_26 = ROcp45_25*C6+C4*S6;
    ROcp45_36 = ROcp45_35*C6+S4*S6;
    ROcp45_46 = -C5*S6;
    ROcp45_56 = -(ROcp45_25*S6-C4*C6);
    ROcp45_66 = -(ROcp45_35*S6-S4*C6);
    OMcp45_25 = qd[5]*C4;
    OMcp45_35 = qd[5]*S4;
    OMcp45_16 = qd[4]+qd[6]*S5;
    OMcp45_26 = OMcp45_25+ROcp45_85*qd[6];
    OMcp45_36 = OMcp45_35+ROcp45_95*qd[6];
    OPcp45_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp45_26 = ROcp45_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp45_35*S5-ROcp45_95*qd[4]);
    OPcp45_36 = ROcp45_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp45_25*S5-ROcp45_85*qd[4]);

// = = Block_1_0_0_46_0_4 = = 
 
// Sensor Kinematics 


    ROcp45_419 = ROcp45_46*C19+S19*S5;
    ROcp45_519 = ROcp45_56*C19+ROcp45_85*S19;
    ROcp45_619 = ROcp45_66*C19+ROcp45_95*S19;
    ROcp45_719 = -(ROcp45_46*S19-C19*S5);
    ROcp45_819 = -(ROcp45_56*S19-ROcp45_85*C19);
    ROcp45_919 = -(ROcp45_66*S19-ROcp45_95*C19);
    ROcp45_120 = ROcp45_16*C20-ROcp45_719*S20;
    ROcp45_220 = ROcp45_26*C20-ROcp45_819*S20;
    ROcp45_320 = ROcp45_36*C20-ROcp45_919*S20;
    ROcp45_720 = ROcp45_16*S20+ROcp45_719*C20;
    ROcp45_820 = ROcp45_26*S20+ROcp45_819*C20;
    ROcp45_920 = ROcp45_36*S20+ROcp45_919*C20;
    ROcp45_121 = ROcp45_120*C21+ROcp45_419*S21;
    ROcp45_221 = ROcp45_220*C21+ROcp45_519*S21;
    ROcp45_321 = ROcp45_320*C21+ROcp45_619*S21;
    ROcp45_421 = -(ROcp45_120*S21-ROcp45_419*C21);
    ROcp45_521 = -(ROcp45_220*S21-ROcp45_519*C21);
    ROcp45_621 = -(ROcp45_320*S21-ROcp45_619*C21);
    RLcp45_119 = s->dpt[2][3]*ROcp45_46+ROcp45_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp45_219 = s->dpt[2][3]*ROcp45_56+ROcp45_26*s->dpt[1][3]+ROcp45_85*s->dpt[3][3];
    RLcp45_319 = s->dpt[2][3]*ROcp45_66+ROcp45_36*s->dpt[1][3]+ROcp45_95*s->dpt[3][3];
    OMcp45_119 = OMcp45_16+ROcp45_16*qd[19];
    OMcp45_219 = OMcp45_26+ROcp45_26*qd[19];
    OMcp45_319 = OMcp45_36+ROcp45_36*qd[19];
    ORcp45_119 = OMcp45_26*RLcp45_319-OMcp45_36*RLcp45_219;
    ORcp45_219 = -(OMcp45_16*RLcp45_319-OMcp45_36*RLcp45_119);
    ORcp45_319 = OMcp45_16*RLcp45_219-OMcp45_26*RLcp45_119;
    OPcp45_119 = OPcp45_16+ROcp45_16*qdd[19]+qd[19]*(OMcp45_26*ROcp45_36-OMcp45_36*ROcp45_26);
    OPcp45_219 = OPcp45_26+ROcp45_26*qdd[19]-qd[19]*(OMcp45_16*ROcp45_36-OMcp45_36*ROcp45_16);
    OPcp45_319 = OPcp45_36+ROcp45_36*qdd[19]+qd[19]*(OMcp45_16*ROcp45_26-OMcp45_26*ROcp45_16);
    RLcp45_120 = s->dpt[1][38]*ROcp45_16+s->dpt[2][38]*ROcp45_419+s->dpt[3][38]*ROcp45_719;
    RLcp45_220 = s->dpt[1][38]*ROcp45_26+s->dpt[2][38]*ROcp45_519+s->dpt[3][38]*ROcp45_819;
    RLcp45_320 = s->dpt[1][38]*ROcp45_36+s->dpt[2][38]*ROcp45_619+s->dpt[3][38]*ROcp45_919;
    OMcp45_120 = OMcp45_119+ROcp45_419*qd[20];
    OMcp45_220 = OMcp45_219+ROcp45_519*qd[20];
    OMcp45_320 = OMcp45_319+ROcp45_619*qd[20];
    ORcp45_120 = OMcp45_219*RLcp45_320-OMcp45_319*RLcp45_220;
    ORcp45_220 = -(OMcp45_119*RLcp45_320-OMcp45_319*RLcp45_120);
    ORcp45_320 = OMcp45_119*RLcp45_220-OMcp45_219*RLcp45_120;
    OPcp45_120 = OPcp45_119+ROcp45_419*qdd[20]+qd[20]*(OMcp45_219*ROcp45_619-OMcp45_319*ROcp45_519);
    OPcp45_220 = OPcp45_219+ROcp45_519*qdd[20]-qd[20]*(OMcp45_119*ROcp45_619-OMcp45_319*ROcp45_419);
    OPcp45_320 = OPcp45_319+ROcp45_619*qdd[20]+qd[20]*(OMcp45_119*ROcp45_519-OMcp45_219*ROcp45_419);
    RLcp45_121 = s->dpt[1][40]*ROcp45_120+s->dpt[2][40]*ROcp45_419+ROcp45_720*s->dpt[3][40];
    RLcp45_221 = s->dpt[1][40]*ROcp45_220+s->dpt[2][40]*ROcp45_519+ROcp45_820*s->dpt[3][40];
    RLcp45_321 = s->dpt[1][40]*ROcp45_320+s->dpt[2][40]*ROcp45_619+ROcp45_920*s->dpt[3][40];
    OMcp45_121 = OMcp45_120+ROcp45_720*qd[21];
    OMcp45_221 = OMcp45_220+ROcp45_820*qd[21];
    OMcp45_321 = OMcp45_320+ROcp45_920*qd[21];
    ORcp45_121 = OMcp45_220*RLcp45_321-OMcp45_320*RLcp45_221;
    ORcp45_221 = -(OMcp45_120*RLcp45_321-OMcp45_320*RLcp45_121);
    ORcp45_321 = OMcp45_120*RLcp45_221-OMcp45_220*RLcp45_121;
    OPcp45_121 = OPcp45_120+ROcp45_720*qdd[21]+qd[21]*(OMcp45_220*ROcp45_920-OMcp45_320*ROcp45_820);
    OPcp45_221 = OPcp45_220+ROcp45_820*qdd[21]-qd[21]*(OMcp45_120*ROcp45_920-OMcp45_320*ROcp45_720);
    OPcp45_321 = OPcp45_320+ROcp45_920*qdd[21]+qd[21]*(OMcp45_120*ROcp45_820-OMcp45_220*ROcp45_720);

// = = Block_1_0_0_46_0_6 = = 
 
// Sensor Kinematics 


    ROcp45_129 = ROcp45_121*C29-ROcp45_720*S29;
    ROcp45_229 = ROcp45_221*C29-ROcp45_820*S29;
    ROcp45_329 = ROcp45_321*C29-ROcp45_920*S29;
    ROcp45_729 = ROcp45_121*S29+ROcp45_720*C29;
    ROcp45_829 = ROcp45_221*S29+ROcp45_820*C29;
    ROcp45_929 = ROcp45_321*S29+ROcp45_920*C29;
    ROcp45_430 = ROcp45_421*C30+ROcp45_729*S30;
    ROcp45_530 = ROcp45_521*C30+ROcp45_829*S30;
    ROcp45_630 = ROcp45_621*C30+ROcp45_929*S30;
    ROcp45_730 = -(ROcp45_421*S30-ROcp45_729*C30);
    ROcp45_830 = -(ROcp45_521*S30-ROcp45_829*C30);
    ROcp45_930 = -(ROcp45_621*S30-ROcp45_929*C30);
    RLcp45_129 = ROcp45_121*s->dpt[1][45]+ROcp45_421*s->dpt[2][45]+ROcp45_720*s->dpt[3][45];
    RLcp45_229 = ROcp45_221*s->dpt[1][45]+ROcp45_521*s->dpt[2][45]+ROcp45_820*s->dpt[3][45];
    RLcp45_329 = ROcp45_321*s->dpt[1][45]+ROcp45_621*s->dpt[2][45]+ROcp45_920*s->dpt[3][45];
    OMcp45_129 = OMcp45_121+ROcp45_421*qd[29];
    OMcp45_229 = OMcp45_221+ROcp45_521*qd[29];
    OMcp45_329 = OMcp45_321+ROcp45_621*qd[29];
    ORcp45_129 = OMcp45_221*RLcp45_329-OMcp45_321*RLcp45_229;
    ORcp45_229 = -(OMcp45_121*RLcp45_329-OMcp45_321*RLcp45_129);
    ORcp45_329 = OMcp45_121*RLcp45_229-OMcp45_221*RLcp45_129;
    OPcp45_129 = OPcp45_121+ROcp45_421*qdd[29]+qd[29]*(OMcp45_221*ROcp45_621-OMcp45_321*ROcp45_521);
    OPcp45_229 = OPcp45_221+ROcp45_521*qdd[29]-qd[29]*(OMcp45_121*ROcp45_621-OMcp45_321*ROcp45_421);
    OPcp45_329 = OPcp45_321+ROcp45_621*qdd[29]+qd[29]*(OMcp45_121*ROcp45_521-OMcp45_221*ROcp45_421);
    RLcp45_130 = s->dpt[1][59]*ROcp45_129+s->dpt[3][59]*ROcp45_729+ROcp45_421*s->dpt[2][59];
    RLcp45_230 = s->dpt[1][59]*ROcp45_229+s->dpt[3][59]*ROcp45_829+ROcp45_521*s->dpt[2][59];
    RLcp45_330 = s->dpt[1][59]*ROcp45_329+s->dpt[3][59]*ROcp45_929+ROcp45_621*s->dpt[2][59];
    OMcp45_130 = OMcp45_129+ROcp45_129*qd[30];
    OMcp45_230 = OMcp45_229+ROcp45_229*qd[30];
    OMcp45_330 = OMcp45_329+ROcp45_329*qd[30];
    ORcp45_130 = OMcp45_229*RLcp45_330-OMcp45_329*RLcp45_230;
    ORcp45_230 = -(OMcp45_129*RLcp45_330-OMcp45_329*RLcp45_130);
    ORcp45_330 = OMcp45_129*RLcp45_230-OMcp45_229*RLcp45_130;
    OPcp45_130 = OPcp45_129+ROcp45_129*qdd[30]+qd[30]*(OMcp45_229*ROcp45_329-OMcp45_329*ROcp45_229);
    OPcp45_230 = OPcp45_229+ROcp45_229*qdd[30]-qd[30]*(OMcp45_129*ROcp45_329-OMcp45_329*ROcp45_129);
    OPcp45_330 = OPcp45_329+ROcp45_329*qdd[30]+qd[30]*(OMcp45_129*ROcp45_229-OMcp45_229*ROcp45_129);
    RLcp45_181 = s->dpt[3][60]*ROcp45_730+ROcp45_129*s->dpt[1][60]+ROcp45_430*s->dpt[2][60];
    RLcp45_281 = s->dpt[3][60]*ROcp45_830+ROcp45_229*s->dpt[1][60]+ROcp45_530*s->dpt[2][60];
    RLcp45_381 = s->dpt[3][60]*ROcp45_930+ROcp45_329*s->dpt[1][60]+ROcp45_630*s->dpt[2][60];
    POcp45_181 = RLcp45_119+RLcp45_120+RLcp45_121+RLcp45_129+RLcp45_130+RLcp45_181+q[1];
    POcp45_281 = RLcp45_219+RLcp45_220+RLcp45_221+RLcp45_229+RLcp45_230+RLcp45_281+q[2];
    POcp45_381 = RLcp45_319+RLcp45_320+RLcp45_321+RLcp45_329+RLcp45_330+RLcp45_381+q[3];
    JTcp45_281_4 = -(RLcp45_319+RLcp45_320+RLcp45_321+RLcp45_329+RLcp45_330+RLcp45_381);
    JTcp45_381_4 = RLcp45_219+RLcp45_220+RLcp45_221+RLcp45_229+RLcp45_230+RLcp45_281;
    JTcp45_181_5 = C4*(RLcp45_319+RLcp45_320+RLcp45_321+RLcp45_329+RLcp45_330+RLcp45_381)-S4*(RLcp45_219+RLcp45_220)-S4*(
 RLcp45_221+RLcp45_229)-S4*(RLcp45_230+RLcp45_281);
    JTcp45_281_5 = S4*(RLcp45_119+RLcp45_120+RLcp45_121+RLcp45_129+RLcp45_130+RLcp45_181);
    JTcp45_381_5 = -C4*(RLcp45_119+RLcp45_120+RLcp45_121+RLcp45_129+RLcp45_130+RLcp45_181);
    JTcp45_181_6 = ROcp45_85*(RLcp45_319+RLcp45_320+RLcp45_321+RLcp45_329+RLcp45_330+RLcp45_381)-ROcp45_95*(RLcp45_219+
 RLcp45_220)-ROcp45_95*(RLcp45_221+RLcp45_229)-ROcp45_95*(RLcp45_230+RLcp45_281);
    JTcp45_281_6 = RLcp45_181*ROcp45_95-RLcp45_330*S5-RLcp45_381*S5+ROcp45_95*(RLcp45_119+RLcp45_120+RLcp45_121+RLcp45_129
 +RLcp45_130)-S5*(RLcp45_319+RLcp45_320)-S5*(RLcp45_321+RLcp45_329);
    JTcp45_381_6 = RLcp45_230*S5-ROcp45_85*(RLcp45_119+RLcp45_120+RLcp45_121+RLcp45_129+RLcp45_130)+S5*(RLcp45_219+
 RLcp45_220)+S5*(RLcp45_221+RLcp45_229)-RLcp45_181*ROcp45_85+RLcp45_281*S5;
    JTcp45_181_7 = ROcp45_26*(RLcp45_320+RLcp45_321+RLcp45_329+RLcp45_330)-ROcp45_36*(RLcp45_220+RLcp45_221)-ROcp45_36*(
 RLcp45_229+RLcp45_230)-RLcp45_281*ROcp45_36+RLcp45_381*ROcp45_26;
    JTcp45_281_7 = RLcp45_181*ROcp45_36-RLcp45_381*ROcp45_16-ROcp45_16*(RLcp45_320+RLcp45_321+RLcp45_329+RLcp45_330)+
 ROcp45_36*(RLcp45_120+RLcp45_121)+ROcp45_36*(RLcp45_129+RLcp45_130);
    JTcp45_381_7 = ROcp45_16*(RLcp45_220+RLcp45_221+RLcp45_229+RLcp45_230)-ROcp45_26*(RLcp45_120+RLcp45_121)-ROcp45_26*(
 RLcp45_129+RLcp45_130)-RLcp45_181*ROcp45_26+RLcp45_281*ROcp45_16;
    JTcp45_181_8 = ROcp45_519*(RLcp45_321+RLcp45_329+RLcp45_330+RLcp45_381)-ROcp45_619*(RLcp45_221+RLcp45_229)-ROcp45_619*
 (RLcp45_230+RLcp45_281);
    JTcp45_281_8 = -(ROcp45_419*(RLcp45_321+RLcp45_329+RLcp45_330+RLcp45_381)-ROcp45_619*(RLcp45_121+RLcp45_129)-
 ROcp45_619*(RLcp45_130+RLcp45_181));
    JTcp45_381_8 = ROcp45_419*(RLcp45_221+RLcp45_229+RLcp45_230+RLcp45_281)-ROcp45_519*(RLcp45_121+RLcp45_129)-ROcp45_519*
 (RLcp45_130+RLcp45_181);
    JTcp45_181_9 = ROcp45_820*(RLcp45_329+RLcp45_330)-ROcp45_920*(RLcp45_229+RLcp45_230)-RLcp45_281*ROcp45_920+RLcp45_381*
 ROcp45_820;
    JTcp45_281_9 = RLcp45_181*ROcp45_920-RLcp45_381*ROcp45_720-ROcp45_720*(RLcp45_329+RLcp45_330)+ROcp45_920*(RLcp45_129+
 RLcp45_130);
    JTcp45_381_9 = ROcp45_720*(RLcp45_229+RLcp45_230)-ROcp45_820*(RLcp45_129+RLcp45_130)-RLcp45_181*ROcp45_820+RLcp45_281*
 ROcp45_720;
    JTcp45_181_10 = ROcp45_521*(RLcp45_330+RLcp45_381)-ROcp45_621*(RLcp45_230+RLcp45_281);
    JTcp45_281_10 = -(ROcp45_421*(RLcp45_330+RLcp45_381)-ROcp45_621*(RLcp45_130+RLcp45_181));
    JTcp45_381_10 = ROcp45_421*(RLcp45_230+RLcp45_281)-ROcp45_521*(RLcp45_130+RLcp45_181);
    JTcp45_181_11 = -(RLcp45_281*ROcp45_329-RLcp45_381*ROcp45_229);
    JTcp45_281_11 = RLcp45_181*ROcp45_329-RLcp45_381*ROcp45_129;
    JTcp45_381_11 = -(RLcp45_181*ROcp45_229-RLcp45_281*ROcp45_129);
    ORcp45_181 = OMcp45_230*RLcp45_381-OMcp45_330*RLcp45_281;
    ORcp45_281 = -(OMcp45_130*RLcp45_381-OMcp45_330*RLcp45_181);
    ORcp45_381 = OMcp45_130*RLcp45_281-OMcp45_230*RLcp45_181;
    VIcp45_181 = ORcp45_119+ORcp45_120+ORcp45_121+ORcp45_129+ORcp45_130+ORcp45_181+qd[1];
    VIcp45_281 = ORcp45_219+ORcp45_220+ORcp45_221+ORcp45_229+ORcp45_230+ORcp45_281+qd[2];
    VIcp45_381 = ORcp45_319+ORcp45_320+ORcp45_321+ORcp45_329+ORcp45_330+ORcp45_381+qd[3];
    ACcp45_181 = qdd[1]+OMcp45_219*ORcp45_320+OMcp45_220*ORcp45_321+OMcp45_221*ORcp45_329+OMcp45_229*ORcp45_330+OMcp45_230
 *ORcp45_381+OMcp45_26*ORcp45_319-OMcp45_319*ORcp45_220-OMcp45_320*ORcp45_221-OMcp45_321*ORcp45_229-OMcp45_329*ORcp45_230-
 OMcp45_330*ORcp45_281-OMcp45_36*ORcp45_219+OPcp45_219*RLcp45_320+OPcp45_220*RLcp45_321+OPcp45_221*RLcp45_329+OPcp45_229*
 RLcp45_330+OPcp45_230*RLcp45_381+OPcp45_26*RLcp45_319-OPcp45_319*RLcp45_220-OPcp45_320*RLcp45_221-OPcp45_321*RLcp45_229-
 OPcp45_329*RLcp45_230-OPcp45_330*RLcp45_281-OPcp45_36*RLcp45_219;
    ACcp45_281 = qdd[2]-OMcp45_119*ORcp45_320-OMcp45_120*ORcp45_321-OMcp45_121*ORcp45_329-OMcp45_129*ORcp45_330-OMcp45_130
 *ORcp45_381-OMcp45_16*ORcp45_319+OMcp45_319*ORcp45_120+OMcp45_320*ORcp45_121+OMcp45_321*ORcp45_129+OMcp45_329*ORcp45_130+
 OMcp45_330*ORcp45_181+OMcp45_36*ORcp45_119-OPcp45_119*RLcp45_320-OPcp45_120*RLcp45_321-OPcp45_121*RLcp45_329-OPcp45_129*
 RLcp45_330-OPcp45_130*RLcp45_381-OPcp45_16*RLcp45_319+OPcp45_319*RLcp45_120+OPcp45_320*RLcp45_121+OPcp45_321*RLcp45_129+
 OPcp45_329*RLcp45_130+OPcp45_330*RLcp45_181+OPcp45_36*RLcp45_119;
    ACcp45_381 = qdd[3]+OMcp45_119*ORcp45_220+OMcp45_120*ORcp45_221+OMcp45_121*ORcp45_229+OMcp45_129*ORcp45_230+OMcp45_130
 *ORcp45_281+OMcp45_16*ORcp45_219-OMcp45_219*ORcp45_120-OMcp45_220*ORcp45_121-OMcp45_221*ORcp45_129-OMcp45_229*ORcp45_130-
 OMcp45_230*ORcp45_181-OMcp45_26*ORcp45_119+OPcp45_119*RLcp45_220+OPcp45_120*RLcp45_221+OPcp45_121*RLcp45_229+OPcp45_129*
 RLcp45_230+OPcp45_130*RLcp45_281+OPcp45_16*RLcp45_219-OPcp45_219*RLcp45_120-OPcp45_220*RLcp45_121-OPcp45_221*RLcp45_129-
 OPcp45_229*RLcp45_130-OPcp45_230*RLcp45_181-OPcp45_26*RLcp45_119;

// = = Block_1_0_0_46_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp45_181;
    sens->P[2] = POcp45_281;
    sens->P[3] = POcp45_381;
    sens->R[1][1] = ROcp45_129;
    sens->R[1][2] = ROcp45_229;
    sens->R[1][3] = ROcp45_329;
    sens->R[2][1] = ROcp45_430;
    sens->R[2][2] = ROcp45_530;
    sens->R[2][3] = ROcp45_630;
    sens->R[3][1] = ROcp45_730;
    sens->R[3][2] = ROcp45_830;
    sens->R[3][3] = ROcp45_930;
    sens->V[1] = VIcp45_181;
    sens->V[2] = VIcp45_281;
    sens->V[3] = VIcp45_381;
    sens->OM[1] = OMcp45_130;
    sens->OM[2] = OMcp45_230;
    sens->OM[3] = OMcp45_330;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp45_181_5;
    sens->J[1][6] = JTcp45_181_6;
    sens->J[1][19] = JTcp45_181_7;
    sens->J[1][20] = JTcp45_181_8;
    sens->J[1][21] = JTcp45_181_9;
    sens->J[1][29] = JTcp45_181_10;
    sens->J[1][30] = JTcp45_181_11;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp45_281_4;
    sens->J[2][5] = JTcp45_281_5;
    sens->J[2][6] = JTcp45_281_6;
    sens->J[2][19] = JTcp45_281_7;
    sens->J[2][20] = JTcp45_281_8;
    sens->J[2][21] = JTcp45_281_9;
    sens->J[2][29] = JTcp45_281_10;
    sens->J[2][30] = JTcp45_281_11;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp45_381_4;
    sens->J[3][5] = JTcp45_381_5;
    sens->J[3][6] = JTcp45_381_6;
    sens->J[3][19] = JTcp45_381_7;
    sens->J[3][20] = JTcp45_381_8;
    sens->J[3][21] = JTcp45_381_9;
    sens->J[3][29] = JTcp45_381_10;
    sens->J[3][30] = JTcp45_381_11;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp45_16;
    sens->J[4][20] = ROcp45_419;
    sens->J[4][21] = ROcp45_720;
    sens->J[4][29] = ROcp45_421;
    sens->J[4][30] = ROcp45_129;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp45_85;
    sens->J[5][19] = ROcp45_26;
    sens->J[5][20] = ROcp45_519;
    sens->J[5][21] = ROcp45_820;
    sens->J[5][29] = ROcp45_521;
    sens->J[5][30] = ROcp45_229;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp45_95;
    sens->J[6][19] = ROcp45_36;
    sens->J[6][20] = ROcp45_619;
    sens->J[6][21] = ROcp45_920;
    sens->J[6][29] = ROcp45_621;
    sens->J[6][30] = ROcp45_329;
    sens->A[1] = ACcp45_181;
    sens->A[2] = ACcp45_281;
    sens->A[3] = ACcp45_381;
    sens->OMP[1] = OPcp45_130;
    sens->OMP[2] = OPcp45_230;
    sens->OMP[3] = OPcp45_330;
 
// 
break;
case 47:
 


// = = Block_1_0_0_47_0_1 = = 
 
// Sensor Kinematics 


    ROcp46_25 = S4*S5;
    ROcp46_35 = -C4*S5;
    ROcp46_85 = -S4*C5;
    ROcp46_95 = C4*C5;
    ROcp46_16 = C5*C6;
    ROcp46_26 = ROcp46_25*C6+C4*S6;
    ROcp46_36 = ROcp46_35*C6+S4*S6;
    ROcp46_46 = -C5*S6;
    ROcp46_56 = -(ROcp46_25*S6-C4*C6);
    ROcp46_66 = -(ROcp46_35*S6-S4*C6);
    OMcp46_25 = qd[5]*C4;
    OMcp46_35 = qd[5]*S4;
    OMcp46_16 = qd[4]+qd[6]*S5;
    OMcp46_26 = OMcp46_25+ROcp46_85*qd[6];
    OMcp46_36 = OMcp46_35+ROcp46_95*qd[6];
    OPcp46_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp46_26 = ROcp46_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp46_35*S5-ROcp46_95*qd[4]);
    OPcp46_36 = ROcp46_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp46_25*S5-ROcp46_85*qd[4]);

// = = Block_1_0_0_47_0_4 = = 
 
// Sensor Kinematics 


    ROcp46_419 = ROcp46_46*C19+S19*S5;
    ROcp46_519 = ROcp46_56*C19+ROcp46_85*S19;
    ROcp46_619 = ROcp46_66*C19+ROcp46_95*S19;
    ROcp46_719 = -(ROcp46_46*S19-C19*S5);
    ROcp46_819 = -(ROcp46_56*S19-ROcp46_85*C19);
    ROcp46_919 = -(ROcp46_66*S19-ROcp46_95*C19);
    ROcp46_120 = ROcp46_16*C20-ROcp46_719*S20;
    ROcp46_220 = ROcp46_26*C20-ROcp46_819*S20;
    ROcp46_320 = ROcp46_36*C20-ROcp46_919*S20;
    ROcp46_720 = ROcp46_16*S20+ROcp46_719*C20;
    ROcp46_820 = ROcp46_26*S20+ROcp46_819*C20;
    ROcp46_920 = ROcp46_36*S20+ROcp46_919*C20;
    ROcp46_121 = ROcp46_120*C21+ROcp46_419*S21;
    ROcp46_221 = ROcp46_220*C21+ROcp46_519*S21;
    ROcp46_321 = ROcp46_320*C21+ROcp46_619*S21;
    ROcp46_421 = -(ROcp46_120*S21-ROcp46_419*C21);
    ROcp46_521 = -(ROcp46_220*S21-ROcp46_519*C21);
    ROcp46_621 = -(ROcp46_320*S21-ROcp46_619*C21);
    RLcp46_119 = s->dpt[2][3]*ROcp46_46+ROcp46_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp46_219 = s->dpt[2][3]*ROcp46_56+ROcp46_26*s->dpt[1][3]+ROcp46_85*s->dpt[3][3];
    RLcp46_319 = s->dpt[2][3]*ROcp46_66+ROcp46_36*s->dpt[1][3]+ROcp46_95*s->dpt[3][3];
    OMcp46_119 = OMcp46_16+ROcp46_16*qd[19];
    OMcp46_219 = OMcp46_26+ROcp46_26*qd[19];
    OMcp46_319 = OMcp46_36+ROcp46_36*qd[19];
    ORcp46_119 = OMcp46_26*RLcp46_319-OMcp46_36*RLcp46_219;
    ORcp46_219 = -(OMcp46_16*RLcp46_319-OMcp46_36*RLcp46_119);
    ORcp46_319 = OMcp46_16*RLcp46_219-OMcp46_26*RLcp46_119;
    OPcp46_119 = OPcp46_16+ROcp46_16*qdd[19]+qd[19]*(OMcp46_26*ROcp46_36-OMcp46_36*ROcp46_26);
    OPcp46_219 = OPcp46_26+ROcp46_26*qdd[19]-qd[19]*(OMcp46_16*ROcp46_36-OMcp46_36*ROcp46_16);
    OPcp46_319 = OPcp46_36+ROcp46_36*qdd[19]+qd[19]*(OMcp46_16*ROcp46_26-OMcp46_26*ROcp46_16);
    RLcp46_120 = s->dpt[1][38]*ROcp46_16+s->dpt[2][38]*ROcp46_419+s->dpt[3][38]*ROcp46_719;
    RLcp46_220 = s->dpt[1][38]*ROcp46_26+s->dpt[2][38]*ROcp46_519+s->dpt[3][38]*ROcp46_819;
    RLcp46_320 = s->dpt[1][38]*ROcp46_36+s->dpt[2][38]*ROcp46_619+s->dpt[3][38]*ROcp46_919;
    OMcp46_120 = OMcp46_119+ROcp46_419*qd[20];
    OMcp46_220 = OMcp46_219+ROcp46_519*qd[20];
    OMcp46_320 = OMcp46_319+ROcp46_619*qd[20];
    ORcp46_120 = OMcp46_219*RLcp46_320-OMcp46_319*RLcp46_220;
    ORcp46_220 = -(OMcp46_119*RLcp46_320-OMcp46_319*RLcp46_120);
    ORcp46_320 = OMcp46_119*RLcp46_220-OMcp46_219*RLcp46_120;
    OPcp46_120 = OPcp46_119+ROcp46_419*qdd[20]+qd[20]*(OMcp46_219*ROcp46_619-OMcp46_319*ROcp46_519);
    OPcp46_220 = OPcp46_219+ROcp46_519*qdd[20]-qd[20]*(OMcp46_119*ROcp46_619-OMcp46_319*ROcp46_419);
    OPcp46_320 = OPcp46_319+ROcp46_619*qdd[20]+qd[20]*(OMcp46_119*ROcp46_519-OMcp46_219*ROcp46_419);
    RLcp46_121 = s->dpt[1][40]*ROcp46_120+s->dpt[2][40]*ROcp46_419+ROcp46_720*s->dpt[3][40];
    RLcp46_221 = s->dpt[1][40]*ROcp46_220+s->dpt[2][40]*ROcp46_519+ROcp46_820*s->dpt[3][40];
    RLcp46_321 = s->dpt[1][40]*ROcp46_320+s->dpt[2][40]*ROcp46_619+ROcp46_920*s->dpt[3][40];
    OMcp46_121 = OMcp46_120+ROcp46_720*qd[21];
    OMcp46_221 = OMcp46_220+ROcp46_820*qd[21];
    OMcp46_321 = OMcp46_320+ROcp46_920*qd[21];
    ORcp46_121 = OMcp46_220*RLcp46_321-OMcp46_320*RLcp46_221;
    ORcp46_221 = -(OMcp46_120*RLcp46_321-OMcp46_320*RLcp46_121);
    ORcp46_321 = OMcp46_120*RLcp46_221-OMcp46_220*RLcp46_121;
    OPcp46_121 = OPcp46_120+ROcp46_720*qdd[21]+qd[21]*(OMcp46_220*ROcp46_920-OMcp46_320*ROcp46_820);
    OPcp46_221 = OPcp46_220+ROcp46_820*qdd[21]-qd[21]*(OMcp46_120*ROcp46_920-OMcp46_320*ROcp46_720);
    OPcp46_321 = OPcp46_320+ROcp46_920*qdd[21]+qd[21]*(OMcp46_120*ROcp46_820-OMcp46_220*ROcp46_720);

// = = Block_1_0_0_47_0_6 = = 
 
// Sensor Kinematics 


    ROcp46_129 = ROcp46_121*C29-ROcp46_720*S29;
    ROcp46_229 = ROcp46_221*C29-ROcp46_820*S29;
    ROcp46_329 = ROcp46_321*C29-ROcp46_920*S29;
    ROcp46_729 = ROcp46_121*S29+ROcp46_720*C29;
    ROcp46_829 = ROcp46_221*S29+ROcp46_820*C29;
    ROcp46_929 = ROcp46_321*S29+ROcp46_920*C29;
    ROcp46_430 = ROcp46_421*C30+ROcp46_729*S30;
    ROcp46_530 = ROcp46_521*C30+ROcp46_829*S30;
    ROcp46_630 = ROcp46_621*C30+ROcp46_929*S30;
    ROcp46_730 = -(ROcp46_421*S30-ROcp46_729*C30);
    ROcp46_830 = -(ROcp46_521*S30-ROcp46_829*C30);
    ROcp46_930 = -(ROcp46_621*S30-ROcp46_929*C30);
    RLcp46_129 = ROcp46_121*s->dpt[1][45]+ROcp46_421*s->dpt[2][45]+ROcp46_720*s->dpt[3][45];
    RLcp46_229 = ROcp46_221*s->dpt[1][45]+ROcp46_521*s->dpt[2][45]+ROcp46_820*s->dpt[3][45];
    RLcp46_329 = ROcp46_321*s->dpt[1][45]+ROcp46_621*s->dpt[2][45]+ROcp46_920*s->dpt[3][45];
    OMcp46_129 = OMcp46_121+ROcp46_421*qd[29];
    OMcp46_229 = OMcp46_221+ROcp46_521*qd[29];
    OMcp46_329 = OMcp46_321+ROcp46_621*qd[29];
    ORcp46_129 = OMcp46_221*RLcp46_329-OMcp46_321*RLcp46_229;
    ORcp46_229 = -(OMcp46_121*RLcp46_329-OMcp46_321*RLcp46_129);
    ORcp46_329 = OMcp46_121*RLcp46_229-OMcp46_221*RLcp46_129;
    OPcp46_129 = OPcp46_121+ROcp46_421*qdd[29]+qd[29]*(OMcp46_221*ROcp46_621-OMcp46_321*ROcp46_521);
    OPcp46_229 = OPcp46_221+ROcp46_521*qdd[29]-qd[29]*(OMcp46_121*ROcp46_621-OMcp46_321*ROcp46_421);
    OPcp46_329 = OPcp46_321+ROcp46_621*qdd[29]+qd[29]*(OMcp46_121*ROcp46_521-OMcp46_221*ROcp46_421);
    RLcp46_130 = s->dpt[1][59]*ROcp46_129+s->dpt[3][59]*ROcp46_729+ROcp46_421*s->dpt[2][59];
    RLcp46_230 = s->dpt[1][59]*ROcp46_229+s->dpt[3][59]*ROcp46_829+ROcp46_521*s->dpt[2][59];
    RLcp46_330 = s->dpt[1][59]*ROcp46_329+s->dpt[3][59]*ROcp46_929+ROcp46_621*s->dpt[2][59];
    OMcp46_130 = OMcp46_129+ROcp46_129*qd[30];
    OMcp46_230 = OMcp46_229+ROcp46_229*qd[30];
    OMcp46_330 = OMcp46_329+ROcp46_329*qd[30];
    ORcp46_130 = OMcp46_229*RLcp46_330-OMcp46_329*RLcp46_230;
    ORcp46_230 = -(OMcp46_129*RLcp46_330-OMcp46_329*RLcp46_130);
    ORcp46_330 = OMcp46_129*RLcp46_230-OMcp46_229*RLcp46_130;
    OPcp46_130 = OPcp46_129+ROcp46_129*qdd[30]+qd[30]*(OMcp46_229*ROcp46_329-OMcp46_329*ROcp46_229);
    OPcp46_230 = OPcp46_229+ROcp46_229*qdd[30]-qd[30]*(OMcp46_129*ROcp46_329-OMcp46_329*ROcp46_129);
    OPcp46_330 = OPcp46_329+ROcp46_329*qdd[30]+qd[30]*(OMcp46_129*ROcp46_229-OMcp46_229*ROcp46_129);
    RLcp46_182 = s->dpt[1][61]*ROcp46_129+s->dpt[3][61]*ROcp46_730+ROcp46_430*s->dpt[2][61];
    RLcp46_282 = s->dpt[1][61]*ROcp46_229+s->dpt[3][61]*ROcp46_830+ROcp46_530*s->dpt[2][61];
    RLcp46_382 = s->dpt[1][61]*ROcp46_329+s->dpt[3][61]*ROcp46_930+ROcp46_630*s->dpt[2][61];
    POcp46_182 = RLcp46_119+RLcp46_120+RLcp46_121+RLcp46_129+RLcp46_130+RLcp46_182+q[1];
    POcp46_282 = RLcp46_219+RLcp46_220+RLcp46_221+RLcp46_229+RLcp46_230+RLcp46_282+q[2];
    POcp46_382 = RLcp46_319+RLcp46_320+RLcp46_321+RLcp46_329+RLcp46_330+RLcp46_382+q[3];
    JTcp46_282_4 = -(RLcp46_319+RLcp46_320+RLcp46_321+RLcp46_329+RLcp46_330+RLcp46_382);
    JTcp46_382_4 = RLcp46_219+RLcp46_220+RLcp46_221+RLcp46_229+RLcp46_230+RLcp46_282;
    JTcp46_182_5 = C4*(RLcp46_319+RLcp46_320+RLcp46_321+RLcp46_329+RLcp46_330+RLcp46_382)-S4*(RLcp46_219+RLcp46_220)-S4*(
 RLcp46_221+RLcp46_229)-S4*(RLcp46_230+RLcp46_282);
    JTcp46_282_5 = S4*(RLcp46_119+RLcp46_120+RLcp46_121+RLcp46_129+RLcp46_130+RLcp46_182);
    JTcp46_382_5 = -C4*(RLcp46_119+RLcp46_120+RLcp46_121+RLcp46_129+RLcp46_130+RLcp46_182);
    JTcp46_182_6 = ROcp46_85*(RLcp46_319+RLcp46_320+RLcp46_321+RLcp46_329+RLcp46_330+RLcp46_382)-ROcp46_95*(RLcp46_219+
 RLcp46_220)-ROcp46_95*(RLcp46_221+RLcp46_229)-ROcp46_95*(RLcp46_230+RLcp46_282);
    JTcp46_282_6 = RLcp46_182*ROcp46_95-RLcp46_330*S5-RLcp46_382*S5+ROcp46_95*(RLcp46_119+RLcp46_120+RLcp46_121+RLcp46_129
 +RLcp46_130)-S5*(RLcp46_319+RLcp46_320)-S5*(RLcp46_321+RLcp46_329);
    JTcp46_382_6 = RLcp46_230*S5-ROcp46_85*(RLcp46_119+RLcp46_120+RLcp46_121+RLcp46_129+RLcp46_130)+S5*(RLcp46_219+
 RLcp46_220)+S5*(RLcp46_221+RLcp46_229)-RLcp46_182*ROcp46_85+RLcp46_282*S5;
    JTcp46_182_7 = ROcp46_26*(RLcp46_320+RLcp46_321+RLcp46_329+RLcp46_330)-ROcp46_36*(RLcp46_220+RLcp46_221)-ROcp46_36*(
 RLcp46_229+RLcp46_230)-RLcp46_282*ROcp46_36+RLcp46_382*ROcp46_26;
    JTcp46_282_7 = RLcp46_182*ROcp46_36-RLcp46_382*ROcp46_16-ROcp46_16*(RLcp46_320+RLcp46_321+RLcp46_329+RLcp46_330)+
 ROcp46_36*(RLcp46_120+RLcp46_121)+ROcp46_36*(RLcp46_129+RLcp46_130);
    JTcp46_382_7 = ROcp46_16*(RLcp46_220+RLcp46_221+RLcp46_229+RLcp46_230)-ROcp46_26*(RLcp46_120+RLcp46_121)-ROcp46_26*(
 RLcp46_129+RLcp46_130)-RLcp46_182*ROcp46_26+RLcp46_282*ROcp46_16;
    JTcp46_182_8 = ROcp46_519*(RLcp46_321+RLcp46_329+RLcp46_330+RLcp46_382)-ROcp46_619*(RLcp46_221+RLcp46_229)-ROcp46_619*
 (RLcp46_230+RLcp46_282);
    JTcp46_282_8 = -(ROcp46_419*(RLcp46_321+RLcp46_329+RLcp46_330+RLcp46_382)-ROcp46_619*(RLcp46_121+RLcp46_129)-
 ROcp46_619*(RLcp46_130+RLcp46_182));
    JTcp46_382_8 = ROcp46_419*(RLcp46_221+RLcp46_229+RLcp46_230+RLcp46_282)-ROcp46_519*(RLcp46_121+RLcp46_129)-ROcp46_519*
 (RLcp46_130+RLcp46_182);
    JTcp46_182_9 = ROcp46_820*(RLcp46_329+RLcp46_330)-ROcp46_920*(RLcp46_229+RLcp46_230)-RLcp46_282*ROcp46_920+RLcp46_382*
 ROcp46_820;
    JTcp46_282_9 = RLcp46_182*ROcp46_920-RLcp46_382*ROcp46_720-ROcp46_720*(RLcp46_329+RLcp46_330)+ROcp46_920*(RLcp46_129+
 RLcp46_130);
    JTcp46_382_9 = ROcp46_720*(RLcp46_229+RLcp46_230)-ROcp46_820*(RLcp46_129+RLcp46_130)-RLcp46_182*ROcp46_820+RLcp46_282*
 ROcp46_720;
    JTcp46_182_10 = ROcp46_521*(RLcp46_330+RLcp46_382)-ROcp46_621*(RLcp46_230+RLcp46_282);
    JTcp46_282_10 = -(ROcp46_421*(RLcp46_330+RLcp46_382)-ROcp46_621*(RLcp46_130+RLcp46_182));
    JTcp46_382_10 = ROcp46_421*(RLcp46_230+RLcp46_282)-ROcp46_521*(RLcp46_130+RLcp46_182);
    JTcp46_182_11 = -(RLcp46_282*ROcp46_329-RLcp46_382*ROcp46_229);
    JTcp46_282_11 = RLcp46_182*ROcp46_329-RLcp46_382*ROcp46_129;
    JTcp46_382_11 = -(RLcp46_182*ROcp46_229-RLcp46_282*ROcp46_129);
    ORcp46_182 = OMcp46_230*RLcp46_382-OMcp46_330*RLcp46_282;
    ORcp46_282 = -(OMcp46_130*RLcp46_382-OMcp46_330*RLcp46_182);
    ORcp46_382 = OMcp46_130*RLcp46_282-OMcp46_230*RLcp46_182;
    VIcp46_182 = ORcp46_119+ORcp46_120+ORcp46_121+ORcp46_129+ORcp46_130+ORcp46_182+qd[1];
    VIcp46_282 = ORcp46_219+ORcp46_220+ORcp46_221+ORcp46_229+ORcp46_230+ORcp46_282+qd[2];
    VIcp46_382 = ORcp46_319+ORcp46_320+ORcp46_321+ORcp46_329+ORcp46_330+ORcp46_382+qd[3];
    ACcp46_182 = qdd[1]+OMcp46_219*ORcp46_320+OMcp46_220*ORcp46_321+OMcp46_221*ORcp46_329+OMcp46_229*ORcp46_330+OMcp46_230
 *ORcp46_382+OMcp46_26*ORcp46_319-OMcp46_319*ORcp46_220-OMcp46_320*ORcp46_221-OMcp46_321*ORcp46_229-OMcp46_329*ORcp46_230-
 OMcp46_330*ORcp46_282-OMcp46_36*ORcp46_219+OPcp46_219*RLcp46_320+OPcp46_220*RLcp46_321+OPcp46_221*RLcp46_329+OPcp46_229*
 RLcp46_330+OPcp46_230*RLcp46_382+OPcp46_26*RLcp46_319-OPcp46_319*RLcp46_220-OPcp46_320*RLcp46_221-OPcp46_321*RLcp46_229-
 OPcp46_329*RLcp46_230-OPcp46_330*RLcp46_282-OPcp46_36*RLcp46_219;
    ACcp46_282 = qdd[2]-OMcp46_119*ORcp46_320-OMcp46_120*ORcp46_321-OMcp46_121*ORcp46_329-OMcp46_129*ORcp46_330-OMcp46_130
 *ORcp46_382-OMcp46_16*ORcp46_319+OMcp46_319*ORcp46_120+OMcp46_320*ORcp46_121+OMcp46_321*ORcp46_129+OMcp46_329*ORcp46_130+
 OMcp46_330*ORcp46_182+OMcp46_36*ORcp46_119-OPcp46_119*RLcp46_320-OPcp46_120*RLcp46_321-OPcp46_121*RLcp46_329-OPcp46_129*
 RLcp46_330-OPcp46_130*RLcp46_382-OPcp46_16*RLcp46_319+OPcp46_319*RLcp46_120+OPcp46_320*RLcp46_121+OPcp46_321*RLcp46_129+
 OPcp46_329*RLcp46_130+OPcp46_330*RLcp46_182+OPcp46_36*RLcp46_119;
    ACcp46_382 = qdd[3]+OMcp46_119*ORcp46_220+OMcp46_120*ORcp46_221+OMcp46_121*ORcp46_229+OMcp46_129*ORcp46_230+OMcp46_130
 *ORcp46_282+OMcp46_16*ORcp46_219-OMcp46_219*ORcp46_120-OMcp46_220*ORcp46_121-OMcp46_221*ORcp46_129-OMcp46_229*ORcp46_130-
 OMcp46_230*ORcp46_182-OMcp46_26*ORcp46_119+OPcp46_119*RLcp46_220+OPcp46_120*RLcp46_221+OPcp46_121*RLcp46_229+OPcp46_129*
 RLcp46_230+OPcp46_130*RLcp46_282+OPcp46_16*RLcp46_219-OPcp46_219*RLcp46_120-OPcp46_220*RLcp46_121-OPcp46_221*RLcp46_129-
 OPcp46_229*RLcp46_130-OPcp46_230*RLcp46_182-OPcp46_26*RLcp46_119;

// = = Block_1_0_0_47_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp46_182;
    sens->P[2] = POcp46_282;
    sens->P[3] = POcp46_382;
    sens->R[1][1] = ROcp46_129;
    sens->R[1][2] = ROcp46_229;
    sens->R[1][3] = ROcp46_329;
    sens->R[2][1] = ROcp46_430;
    sens->R[2][2] = ROcp46_530;
    sens->R[2][3] = ROcp46_630;
    sens->R[3][1] = ROcp46_730;
    sens->R[3][2] = ROcp46_830;
    sens->R[3][3] = ROcp46_930;
    sens->V[1] = VIcp46_182;
    sens->V[2] = VIcp46_282;
    sens->V[3] = VIcp46_382;
    sens->OM[1] = OMcp46_130;
    sens->OM[2] = OMcp46_230;
    sens->OM[3] = OMcp46_330;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp46_182_5;
    sens->J[1][6] = JTcp46_182_6;
    sens->J[1][19] = JTcp46_182_7;
    sens->J[1][20] = JTcp46_182_8;
    sens->J[1][21] = JTcp46_182_9;
    sens->J[1][29] = JTcp46_182_10;
    sens->J[1][30] = JTcp46_182_11;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp46_282_4;
    sens->J[2][5] = JTcp46_282_5;
    sens->J[2][6] = JTcp46_282_6;
    sens->J[2][19] = JTcp46_282_7;
    sens->J[2][20] = JTcp46_282_8;
    sens->J[2][21] = JTcp46_282_9;
    sens->J[2][29] = JTcp46_282_10;
    sens->J[2][30] = JTcp46_282_11;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp46_382_4;
    sens->J[3][5] = JTcp46_382_5;
    sens->J[3][6] = JTcp46_382_6;
    sens->J[3][19] = JTcp46_382_7;
    sens->J[3][20] = JTcp46_382_8;
    sens->J[3][21] = JTcp46_382_9;
    sens->J[3][29] = JTcp46_382_10;
    sens->J[3][30] = JTcp46_382_11;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp46_16;
    sens->J[4][20] = ROcp46_419;
    sens->J[4][21] = ROcp46_720;
    sens->J[4][29] = ROcp46_421;
    sens->J[4][30] = ROcp46_129;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp46_85;
    sens->J[5][19] = ROcp46_26;
    sens->J[5][20] = ROcp46_519;
    sens->J[5][21] = ROcp46_820;
    sens->J[5][29] = ROcp46_521;
    sens->J[5][30] = ROcp46_229;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp46_95;
    sens->J[6][19] = ROcp46_36;
    sens->J[6][20] = ROcp46_619;
    sens->J[6][21] = ROcp46_920;
    sens->J[6][29] = ROcp46_621;
    sens->J[6][30] = ROcp46_329;
    sens->A[1] = ACcp46_182;
    sens->A[2] = ACcp46_282;
    sens->A[3] = ACcp46_382;
    sens->OMP[1] = OPcp46_130;
    sens->OMP[2] = OPcp46_230;
    sens->OMP[3] = OPcp46_330;
 
// 
break;
case 48:
 


// = = Block_1_0_0_48_0_1 = = 
 
// Sensor Kinematics 


    ROcp47_25 = S4*S5;
    ROcp47_35 = -C4*S5;
    ROcp47_85 = -S4*C5;
    ROcp47_95 = C4*C5;
    ROcp47_16 = C5*C6;
    ROcp47_26 = ROcp47_25*C6+C4*S6;
    ROcp47_36 = ROcp47_35*C6+S4*S6;
    ROcp47_46 = -C5*S6;
    ROcp47_56 = -(ROcp47_25*S6-C4*C6);
    ROcp47_66 = -(ROcp47_35*S6-S4*C6);
    OMcp47_25 = qd[5]*C4;
    OMcp47_35 = qd[5]*S4;
    OMcp47_16 = qd[4]+qd[6]*S5;
    OMcp47_26 = OMcp47_25+ROcp47_85*qd[6];
    OMcp47_36 = OMcp47_35+ROcp47_95*qd[6];
    OPcp47_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp47_26 = ROcp47_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp47_35*S5-ROcp47_95*qd[4]);
    OPcp47_36 = ROcp47_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp47_25*S5-ROcp47_85*qd[4]);

// = = Block_1_0_0_48_0_4 = = 
 
// Sensor Kinematics 


    ROcp47_419 = ROcp47_46*C19+S19*S5;
    ROcp47_519 = ROcp47_56*C19+ROcp47_85*S19;
    ROcp47_619 = ROcp47_66*C19+ROcp47_95*S19;
    ROcp47_719 = -(ROcp47_46*S19-C19*S5);
    ROcp47_819 = -(ROcp47_56*S19-ROcp47_85*C19);
    ROcp47_919 = -(ROcp47_66*S19-ROcp47_95*C19);
    ROcp47_120 = ROcp47_16*C20-ROcp47_719*S20;
    ROcp47_220 = ROcp47_26*C20-ROcp47_819*S20;
    ROcp47_320 = ROcp47_36*C20-ROcp47_919*S20;
    ROcp47_720 = ROcp47_16*S20+ROcp47_719*C20;
    ROcp47_820 = ROcp47_26*S20+ROcp47_819*C20;
    ROcp47_920 = ROcp47_36*S20+ROcp47_919*C20;
    ROcp47_121 = ROcp47_120*C21+ROcp47_419*S21;
    ROcp47_221 = ROcp47_220*C21+ROcp47_519*S21;
    ROcp47_321 = ROcp47_320*C21+ROcp47_619*S21;
    ROcp47_421 = -(ROcp47_120*S21-ROcp47_419*C21);
    ROcp47_521 = -(ROcp47_220*S21-ROcp47_519*C21);
    ROcp47_621 = -(ROcp47_320*S21-ROcp47_619*C21);
    RLcp47_119 = s->dpt[2][3]*ROcp47_46+ROcp47_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp47_219 = s->dpt[2][3]*ROcp47_56+ROcp47_26*s->dpt[1][3]+ROcp47_85*s->dpt[3][3];
    RLcp47_319 = s->dpt[2][3]*ROcp47_66+ROcp47_36*s->dpt[1][3]+ROcp47_95*s->dpt[3][3];
    OMcp47_119 = OMcp47_16+ROcp47_16*qd[19];
    OMcp47_219 = OMcp47_26+ROcp47_26*qd[19];
    OMcp47_319 = OMcp47_36+ROcp47_36*qd[19];
    ORcp47_119 = OMcp47_26*RLcp47_319-OMcp47_36*RLcp47_219;
    ORcp47_219 = -(OMcp47_16*RLcp47_319-OMcp47_36*RLcp47_119);
    ORcp47_319 = OMcp47_16*RLcp47_219-OMcp47_26*RLcp47_119;
    OPcp47_119 = OPcp47_16+ROcp47_16*qdd[19]+qd[19]*(OMcp47_26*ROcp47_36-OMcp47_36*ROcp47_26);
    OPcp47_219 = OPcp47_26+ROcp47_26*qdd[19]-qd[19]*(OMcp47_16*ROcp47_36-OMcp47_36*ROcp47_16);
    OPcp47_319 = OPcp47_36+ROcp47_36*qdd[19]+qd[19]*(OMcp47_16*ROcp47_26-OMcp47_26*ROcp47_16);
    RLcp47_120 = s->dpt[1][38]*ROcp47_16+s->dpt[2][38]*ROcp47_419+s->dpt[3][38]*ROcp47_719;
    RLcp47_220 = s->dpt[1][38]*ROcp47_26+s->dpt[2][38]*ROcp47_519+s->dpt[3][38]*ROcp47_819;
    RLcp47_320 = s->dpt[1][38]*ROcp47_36+s->dpt[2][38]*ROcp47_619+s->dpt[3][38]*ROcp47_919;
    OMcp47_120 = OMcp47_119+ROcp47_419*qd[20];
    OMcp47_220 = OMcp47_219+ROcp47_519*qd[20];
    OMcp47_320 = OMcp47_319+ROcp47_619*qd[20];
    ORcp47_120 = OMcp47_219*RLcp47_320-OMcp47_319*RLcp47_220;
    ORcp47_220 = -(OMcp47_119*RLcp47_320-OMcp47_319*RLcp47_120);
    ORcp47_320 = OMcp47_119*RLcp47_220-OMcp47_219*RLcp47_120;
    OPcp47_120 = OPcp47_119+ROcp47_419*qdd[20]+qd[20]*(OMcp47_219*ROcp47_619-OMcp47_319*ROcp47_519);
    OPcp47_220 = OPcp47_219+ROcp47_519*qdd[20]-qd[20]*(OMcp47_119*ROcp47_619-OMcp47_319*ROcp47_419);
    OPcp47_320 = OPcp47_319+ROcp47_619*qdd[20]+qd[20]*(OMcp47_119*ROcp47_519-OMcp47_219*ROcp47_419);
    RLcp47_121 = s->dpt[1][40]*ROcp47_120+s->dpt[2][40]*ROcp47_419+ROcp47_720*s->dpt[3][40];
    RLcp47_221 = s->dpt[1][40]*ROcp47_220+s->dpt[2][40]*ROcp47_519+ROcp47_820*s->dpt[3][40];
    RLcp47_321 = s->dpt[1][40]*ROcp47_320+s->dpt[2][40]*ROcp47_619+ROcp47_920*s->dpt[3][40];
    OMcp47_121 = OMcp47_120+ROcp47_720*qd[21];
    OMcp47_221 = OMcp47_220+ROcp47_820*qd[21];
    OMcp47_321 = OMcp47_320+ROcp47_920*qd[21];
    ORcp47_121 = OMcp47_220*RLcp47_321-OMcp47_320*RLcp47_221;
    ORcp47_221 = -(OMcp47_120*RLcp47_321-OMcp47_320*RLcp47_121);
    ORcp47_321 = OMcp47_120*RLcp47_221-OMcp47_220*RLcp47_121;
    OPcp47_121 = OPcp47_120+ROcp47_720*qdd[21]+qd[21]*(OMcp47_220*ROcp47_920-OMcp47_320*ROcp47_820);
    OPcp47_221 = OPcp47_220+ROcp47_820*qdd[21]-qd[21]*(OMcp47_120*ROcp47_920-OMcp47_320*ROcp47_720);
    OPcp47_321 = OPcp47_320+ROcp47_920*qdd[21]+qd[21]*(OMcp47_120*ROcp47_820-OMcp47_220*ROcp47_720);

// = = Block_1_0_0_48_0_6 = = 
 
// Sensor Kinematics 


    ROcp47_129 = ROcp47_121*C29-ROcp47_720*S29;
    ROcp47_229 = ROcp47_221*C29-ROcp47_820*S29;
    ROcp47_329 = ROcp47_321*C29-ROcp47_920*S29;
    ROcp47_729 = ROcp47_121*S29+ROcp47_720*C29;
    ROcp47_829 = ROcp47_221*S29+ROcp47_820*C29;
    ROcp47_929 = ROcp47_321*S29+ROcp47_920*C29;
    ROcp47_430 = ROcp47_421*C30+ROcp47_729*S30;
    ROcp47_530 = ROcp47_521*C30+ROcp47_829*S30;
    ROcp47_630 = ROcp47_621*C30+ROcp47_929*S30;
    ROcp47_730 = -(ROcp47_421*S30-ROcp47_729*C30);
    ROcp47_830 = -(ROcp47_521*S30-ROcp47_829*C30);
    ROcp47_930 = -(ROcp47_621*S30-ROcp47_929*C30);
    ROcp47_131 = ROcp47_129*C31-ROcp47_730*S31;
    ROcp47_231 = ROcp47_229*C31-ROcp47_830*S31;
    ROcp47_331 = ROcp47_329*C31-ROcp47_930*S31;
    ROcp47_731 = ROcp47_129*S31+ROcp47_730*C31;
    ROcp47_831 = ROcp47_229*S31+ROcp47_830*C31;
    ROcp47_931 = ROcp47_329*S31+ROcp47_930*C31;
    RLcp47_129 = ROcp47_121*s->dpt[1][45]+ROcp47_421*s->dpt[2][45]+ROcp47_720*s->dpt[3][45];
    RLcp47_229 = ROcp47_221*s->dpt[1][45]+ROcp47_521*s->dpt[2][45]+ROcp47_820*s->dpt[3][45];
    RLcp47_329 = ROcp47_321*s->dpt[1][45]+ROcp47_621*s->dpt[2][45]+ROcp47_920*s->dpt[3][45];
    OMcp47_129 = OMcp47_121+ROcp47_421*qd[29];
    OMcp47_229 = OMcp47_221+ROcp47_521*qd[29];
    OMcp47_329 = OMcp47_321+ROcp47_621*qd[29];
    ORcp47_129 = OMcp47_221*RLcp47_329-OMcp47_321*RLcp47_229;
    ORcp47_229 = -(OMcp47_121*RLcp47_329-OMcp47_321*RLcp47_129);
    ORcp47_329 = OMcp47_121*RLcp47_229-OMcp47_221*RLcp47_129;
    OPcp47_129 = OPcp47_121+ROcp47_421*qdd[29]+qd[29]*(OMcp47_221*ROcp47_621-OMcp47_321*ROcp47_521);
    OPcp47_229 = OPcp47_221+ROcp47_521*qdd[29]-qd[29]*(OMcp47_121*ROcp47_621-OMcp47_321*ROcp47_421);
    OPcp47_329 = OPcp47_321+ROcp47_621*qdd[29]+qd[29]*(OMcp47_121*ROcp47_521-OMcp47_221*ROcp47_421);
    RLcp47_130 = s->dpt[1][59]*ROcp47_129+s->dpt[3][59]*ROcp47_729+ROcp47_421*s->dpt[2][59];
    RLcp47_230 = s->dpt[1][59]*ROcp47_229+s->dpt[3][59]*ROcp47_829+ROcp47_521*s->dpt[2][59];
    RLcp47_330 = s->dpt[1][59]*ROcp47_329+s->dpt[3][59]*ROcp47_929+ROcp47_621*s->dpt[2][59];
    OMcp47_130 = OMcp47_129+ROcp47_129*qd[30];
    OMcp47_230 = OMcp47_229+ROcp47_229*qd[30];
    OMcp47_330 = OMcp47_329+ROcp47_329*qd[30];
    ORcp47_130 = OMcp47_229*RLcp47_330-OMcp47_329*RLcp47_230;
    ORcp47_230 = -(OMcp47_129*RLcp47_330-OMcp47_329*RLcp47_130);
    ORcp47_330 = OMcp47_129*RLcp47_230-OMcp47_229*RLcp47_130;
    OPcp47_130 = OPcp47_129+ROcp47_129*qdd[30]+qd[30]*(OMcp47_229*ROcp47_329-OMcp47_329*ROcp47_229);
    OPcp47_230 = OPcp47_229+ROcp47_229*qdd[30]-qd[30]*(OMcp47_129*ROcp47_329-OMcp47_329*ROcp47_129);
    OPcp47_330 = OPcp47_329+ROcp47_329*qdd[30]+qd[30]*(OMcp47_129*ROcp47_229-OMcp47_229*ROcp47_129);
    RLcp47_131 = s->dpt[1][61]*ROcp47_129+s->dpt[3][61]*ROcp47_730+ROcp47_430*s->dpt[2][61];
    RLcp47_231 = s->dpt[1][61]*ROcp47_229+s->dpt[3][61]*ROcp47_830+ROcp47_530*s->dpt[2][61];
    RLcp47_331 = s->dpt[1][61]*ROcp47_329+s->dpt[3][61]*ROcp47_930+ROcp47_630*s->dpt[2][61];
    OMcp47_131 = OMcp47_130+ROcp47_430*qd[31];
    OMcp47_231 = OMcp47_230+ROcp47_530*qd[31];
    OMcp47_331 = OMcp47_330+ROcp47_630*qd[31];
    ORcp47_131 = OMcp47_230*RLcp47_331-OMcp47_330*RLcp47_231;
    ORcp47_231 = -(OMcp47_130*RLcp47_331-OMcp47_330*RLcp47_131);
    ORcp47_331 = OMcp47_130*RLcp47_231-OMcp47_230*RLcp47_131;
    OPcp47_131 = OPcp47_130+ROcp47_430*qdd[31]+qd[31]*(OMcp47_230*ROcp47_630-OMcp47_330*ROcp47_530);
    OPcp47_231 = OPcp47_230+ROcp47_530*qdd[31]-qd[31]*(OMcp47_130*ROcp47_630-OMcp47_330*ROcp47_430);
    OPcp47_331 = OPcp47_330+ROcp47_630*qdd[31]+qd[31]*(OMcp47_130*ROcp47_530-OMcp47_230*ROcp47_430);
    RLcp47_183 = ROcp47_131*s->dpt[1][62]+ROcp47_430*s->dpt[2][62]+ROcp47_731*s->dpt[3][62];
    RLcp47_283 = ROcp47_231*s->dpt[1][62]+ROcp47_530*s->dpt[2][62]+ROcp47_831*s->dpt[3][62];
    RLcp47_383 = ROcp47_331*s->dpt[1][62]+ROcp47_630*s->dpt[2][62]+ROcp47_931*s->dpt[3][62];
    POcp47_183 = RLcp47_119+RLcp47_120+RLcp47_121+RLcp47_129+RLcp47_130+RLcp47_131+RLcp47_183+q[1];
    POcp47_283 = RLcp47_219+RLcp47_220+RLcp47_221+RLcp47_229+RLcp47_230+RLcp47_231+RLcp47_283+q[2];
    POcp47_383 = RLcp47_319+RLcp47_320+RLcp47_321+RLcp47_329+RLcp47_330+RLcp47_331+RLcp47_383+q[3];
    JTcp47_283_4 = -(RLcp47_319+RLcp47_320+RLcp47_321+RLcp47_329+RLcp47_330+RLcp47_331+RLcp47_383);
    JTcp47_383_4 = RLcp47_219+RLcp47_220+RLcp47_221+RLcp47_229+RLcp47_230+RLcp47_231+RLcp47_283;
    JTcp47_183_5 = C4*(RLcp47_319+RLcp47_320+RLcp47_321+RLcp47_329+RLcp47_330+RLcp47_331)-S4*(RLcp47_219+RLcp47_220)-S4*(
 RLcp47_221+RLcp47_229)-S4*(RLcp47_230+RLcp47_231)-RLcp47_283*S4+RLcp47_383*C4;
    JTcp47_283_5 = S4*(RLcp47_119+RLcp47_120+RLcp47_121+RLcp47_129+RLcp47_130+RLcp47_131+RLcp47_183);
    JTcp47_383_5 = -C4*(RLcp47_119+RLcp47_120+RLcp47_121+RLcp47_129+RLcp47_130+RLcp47_131+RLcp47_183);
    JTcp47_183_6 = ROcp47_85*(RLcp47_319+RLcp47_320+RLcp47_321+RLcp47_329+RLcp47_330+RLcp47_331)-ROcp47_95*(RLcp47_219+
 RLcp47_220)-ROcp47_95*(RLcp47_221+RLcp47_229)-ROcp47_95*(RLcp47_230+RLcp47_231)-RLcp47_283*ROcp47_95+RLcp47_383*ROcp47_85;
    JTcp47_283_6 = -(RLcp47_383*S5-ROcp47_95*(RLcp47_119+RLcp47_120+RLcp47_121+RLcp47_129+RLcp47_130+RLcp47_131+RLcp47_183
 )+S5*(RLcp47_319+RLcp47_320)+S5*(RLcp47_321+RLcp47_329)+S5*(RLcp47_330+RLcp47_331));
    JTcp47_383_6 = RLcp47_283*S5-ROcp47_85*(RLcp47_119+RLcp47_120+RLcp47_121+RLcp47_129+RLcp47_130+RLcp47_131+RLcp47_183)+
 S5*(RLcp47_219+RLcp47_220)+S5*(RLcp47_221+RLcp47_229)+S5*(RLcp47_230+RLcp47_231);
    JTcp47_183_7 = ROcp47_26*(RLcp47_320+RLcp47_321+RLcp47_329+RLcp47_330+RLcp47_331+RLcp47_383)-ROcp47_36*(RLcp47_220+
 RLcp47_221)-ROcp47_36*(RLcp47_229+RLcp47_230)-ROcp47_36*(RLcp47_231+RLcp47_283);
    JTcp47_283_7 = -(ROcp47_16*(RLcp47_320+RLcp47_321+RLcp47_329+RLcp47_330+RLcp47_331+RLcp47_383)-ROcp47_36*(RLcp47_120+
 RLcp47_121)-ROcp47_36*(RLcp47_129+RLcp47_130)-ROcp47_36*(RLcp47_131+RLcp47_183));
    JTcp47_383_7 = ROcp47_16*(RLcp47_220+RLcp47_221+RLcp47_229+RLcp47_230+RLcp47_231+RLcp47_283)-ROcp47_26*(RLcp47_120+
 RLcp47_121)-ROcp47_26*(RLcp47_129+RLcp47_130)-ROcp47_26*(RLcp47_131+RLcp47_183);
    JTcp47_183_8 = ROcp47_519*(RLcp47_321+RLcp47_329+RLcp47_330+RLcp47_331)-ROcp47_619*(RLcp47_221+RLcp47_229)-ROcp47_619*
 (RLcp47_230+RLcp47_231)-RLcp47_283*ROcp47_619+RLcp47_383*ROcp47_519;
    JTcp47_283_8 = RLcp47_183*ROcp47_619-RLcp47_383*ROcp47_419-ROcp47_419*(RLcp47_321+RLcp47_329+RLcp47_330+RLcp47_331)+
 ROcp47_619*(RLcp47_121+RLcp47_129)+ROcp47_619*(RLcp47_130+RLcp47_131);
    JTcp47_383_8 = ROcp47_419*(RLcp47_221+RLcp47_229+RLcp47_230+RLcp47_231)-ROcp47_519*(RLcp47_121+RLcp47_129)-ROcp47_519*
 (RLcp47_130+RLcp47_131)-RLcp47_183*ROcp47_519+RLcp47_283*ROcp47_419;
    JTcp47_183_9 = ROcp47_820*(RLcp47_329+RLcp47_330+RLcp47_331+RLcp47_383)-ROcp47_920*(RLcp47_229+RLcp47_230)-ROcp47_920*
 (RLcp47_231+RLcp47_283);
    JTcp47_283_9 = -(ROcp47_720*(RLcp47_329+RLcp47_330+RLcp47_331+RLcp47_383)-ROcp47_920*(RLcp47_129+RLcp47_130)-
 ROcp47_920*(RLcp47_131+RLcp47_183));
    JTcp47_383_9 = ROcp47_720*(RLcp47_229+RLcp47_230+RLcp47_231+RLcp47_283)-ROcp47_820*(RLcp47_129+RLcp47_130)-ROcp47_820*
 (RLcp47_131+RLcp47_183);
    JTcp47_183_10 = ROcp47_521*(RLcp47_330+RLcp47_331)-ROcp47_621*(RLcp47_230+RLcp47_231)-RLcp47_283*ROcp47_621+RLcp47_383
 *ROcp47_521;
    JTcp47_283_10 = RLcp47_183*ROcp47_621-RLcp47_383*ROcp47_421-ROcp47_421*(RLcp47_330+RLcp47_331)+ROcp47_621*(RLcp47_130+
 RLcp47_131);
    JTcp47_383_10 = ROcp47_421*(RLcp47_230+RLcp47_231)-ROcp47_521*(RLcp47_130+RLcp47_131)-RLcp47_183*ROcp47_521+RLcp47_283
 *ROcp47_421;
    JTcp47_183_11 = ROcp47_229*(RLcp47_331+RLcp47_383)-ROcp47_329*(RLcp47_231+RLcp47_283);
    JTcp47_283_11 = -(ROcp47_129*(RLcp47_331+RLcp47_383)-ROcp47_329*(RLcp47_131+RLcp47_183));
    JTcp47_383_11 = ROcp47_129*(RLcp47_231+RLcp47_283)-ROcp47_229*(RLcp47_131+RLcp47_183);
    JTcp47_183_12 = -(RLcp47_283*ROcp47_630-RLcp47_383*ROcp47_530);
    JTcp47_283_12 = RLcp47_183*ROcp47_630-RLcp47_383*ROcp47_430;
    JTcp47_383_12 = -(RLcp47_183*ROcp47_530-RLcp47_283*ROcp47_430);
    ORcp47_183 = OMcp47_231*RLcp47_383-OMcp47_331*RLcp47_283;
    ORcp47_283 = -(OMcp47_131*RLcp47_383-OMcp47_331*RLcp47_183);
    ORcp47_383 = OMcp47_131*RLcp47_283-OMcp47_231*RLcp47_183;
    VIcp47_183 = ORcp47_119+ORcp47_120+ORcp47_121+ORcp47_129+ORcp47_130+ORcp47_131+ORcp47_183+qd[1];
    VIcp47_283 = ORcp47_219+ORcp47_220+ORcp47_221+ORcp47_229+ORcp47_230+ORcp47_231+ORcp47_283+qd[2];
    VIcp47_383 = ORcp47_319+ORcp47_320+ORcp47_321+ORcp47_329+ORcp47_330+ORcp47_331+ORcp47_383+qd[3];
    ACcp47_183 = qdd[1]+OMcp47_219*ORcp47_320+OMcp47_220*ORcp47_321+OMcp47_221*ORcp47_329+OMcp47_229*ORcp47_330+OMcp47_230
 *ORcp47_331+OMcp47_231*ORcp47_383+OMcp47_26*ORcp47_319-OMcp47_319*ORcp47_220-OMcp47_320*ORcp47_221-OMcp47_321*ORcp47_229-
 OMcp47_329*ORcp47_230-OMcp47_330*ORcp47_231-OMcp47_331*ORcp47_283-OMcp47_36*ORcp47_219+OPcp47_219*RLcp47_320+OPcp47_220*
 RLcp47_321+OPcp47_221*RLcp47_329+OPcp47_229*RLcp47_330+OPcp47_230*RLcp47_331+OPcp47_231*RLcp47_383+OPcp47_26*RLcp47_319-
 OPcp47_319*RLcp47_220-OPcp47_320*RLcp47_221-OPcp47_321*RLcp47_229-OPcp47_329*RLcp47_230-OPcp47_330*RLcp47_231-OPcp47_331*
 RLcp47_283-OPcp47_36*RLcp47_219;
    ACcp47_283 = qdd[2]-OMcp47_119*ORcp47_320-OMcp47_120*ORcp47_321-OMcp47_121*ORcp47_329-OMcp47_129*ORcp47_330-OMcp47_130
 *ORcp47_331-OMcp47_131*ORcp47_383-OMcp47_16*ORcp47_319+OMcp47_319*ORcp47_120+OMcp47_320*ORcp47_121+OMcp47_321*ORcp47_129+
 OMcp47_329*ORcp47_130+OMcp47_330*ORcp47_131+OMcp47_331*ORcp47_183+OMcp47_36*ORcp47_119-OPcp47_119*RLcp47_320-OPcp47_120*
 RLcp47_321-OPcp47_121*RLcp47_329-OPcp47_129*RLcp47_330-OPcp47_130*RLcp47_331-OPcp47_131*RLcp47_383-OPcp47_16*RLcp47_319+
 OPcp47_319*RLcp47_120+OPcp47_320*RLcp47_121+OPcp47_321*RLcp47_129+OPcp47_329*RLcp47_130+OPcp47_330*RLcp47_131+OPcp47_331*
 RLcp47_183+OPcp47_36*RLcp47_119;
    ACcp47_383 = qdd[3]+OMcp47_119*ORcp47_220+OMcp47_120*ORcp47_221+OMcp47_121*ORcp47_229+OMcp47_129*ORcp47_230+OMcp47_130
 *ORcp47_231+OMcp47_131*ORcp47_283+OMcp47_16*ORcp47_219-OMcp47_219*ORcp47_120-OMcp47_220*ORcp47_121-OMcp47_221*ORcp47_129-
 OMcp47_229*ORcp47_130-OMcp47_230*ORcp47_131-OMcp47_231*ORcp47_183-OMcp47_26*ORcp47_119+OPcp47_119*RLcp47_220+OPcp47_120*
 RLcp47_221+OPcp47_121*RLcp47_229+OPcp47_129*RLcp47_230+OPcp47_130*RLcp47_231+OPcp47_131*RLcp47_283+OPcp47_16*RLcp47_219-
 OPcp47_219*RLcp47_120-OPcp47_220*RLcp47_121-OPcp47_221*RLcp47_129-OPcp47_229*RLcp47_130-OPcp47_230*RLcp47_131-OPcp47_231*
 RLcp47_183-OPcp47_26*RLcp47_119;

// = = Block_1_0_0_48_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp47_183;
    sens->P[2] = POcp47_283;
    sens->P[3] = POcp47_383;
    sens->R[1][1] = ROcp47_131;
    sens->R[1][2] = ROcp47_231;
    sens->R[1][3] = ROcp47_331;
    sens->R[2][1] = ROcp47_430;
    sens->R[2][2] = ROcp47_530;
    sens->R[2][3] = ROcp47_630;
    sens->R[3][1] = ROcp47_731;
    sens->R[3][2] = ROcp47_831;
    sens->R[3][3] = ROcp47_931;
    sens->V[1] = VIcp47_183;
    sens->V[2] = VIcp47_283;
    sens->V[3] = VIcp47_383;
    sens->OM[1] = OMcp47_131;
    sens->OM[2] = OMcp47_231;
    sens->OM[3] = OMcp47_331;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp47_183_5;
    sens->J[1][6] = JTcp47_183_6;
    sens->J[1][19] = JTcp47_183_7;
    sens->J[1][20] = JTcp47_183_8;
    sens->J[1][21] = JTcp47_183_9;
    sens->J[1][29] = JTcp47_183_10;
    sens->J[1][30] = JTcp47_183_11;
    sens->J[1][31] = JTcp47_183_12;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp47_283_4;
    sens->J[2][5] = JTcp47_283_5;
    sens->J[2][6] = JTcp47_283_6;
    sens->J[2][19] = JTcp47_283_7;
    sens->J[2][20] = JTcp47_283_8;
    sens->J[2][21] = JTcp47_283_9;
    sens->J[2][29] = JTcp47_283_10;
    sens->J[2][30] = JTcp47_283_11;
    sens->J[2][31] = JTcp47_283_12;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp47_383_4;
    sens->J[3][5] = JTcp47_383_5;
    sens->J[3][6] = JTcp47_383_6;
    sens->J[3][19] = JTcp47_383_7;
    sens->J[3][20] = JTcp47_383_8;
    sens->J[3][21] = JTcp47_383_9;
    sens->J[3][29] = JTcp47_383_10;
    sens->J[3][30] = JTcp47_383_11;
    sens->J[3][31] = JTcp47_383_12;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp47_16;
    sens->J[4][20] = ROcp47_419;
    sens->J[4][21] = ROcp47_720;
    sens->J[4][29] = ROcp47_421;
    sens->J[4][30] = ROcp47_129;
    sens->J[4][31] = ROcp47_430;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp47_85;
    sens->J[5][19] = ROcp47_26;
    sens->J[5][20] = ROcp47_519;
    sens->J[5][21] = ROcp47_820;
    sens->J[5][29] = ROcp47_521;
    sens->J[5][30] = ROcp47_229;
    sens->J[5][31] = ROcp47_530;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp47_95;
    sens->J[6][19] = ROcp47_36;
    sens->J[6][20] = ROcp47_619;
    sens->J[6][21] = ROcp47_920;
    sens->J[6][29] = ROcp47_621;
    sens->J[6][30] = ROcp47_329;
    sens->J[6][31] = ROcp47_630;
    sens->A[1] = ACcp47_183;
    sens->A[2] = ACcp47_283;
    sens->A[3] = ACcp47_383;
    sens->OMP[1] = OPcp47_131;
    sens->OMP[2] = OPcp47_231;
    sens->OMP[3] = OPcp47_331;
 
// 
break;
case 49:
 


// = = Block_1_0_0_49_0_1 = = 
 
// Sensor Kinematics 


    ROcp48_25 = S4*S5;
    ROcp48_35 = -C4*S5;
    ROcp48_85 = -S4*C5;
    ROcp48_95 = C4*C5;
    ROcp48_16 = C5*C6;
    ROcp48_26 = ROcp48_25*C6+C4*S6;
    ROcp48_36 = ROcp48_35*C6+S4*S6;
    ROcp48_46 = -C5*S6;
    ROcp48_56 = -(ROcp48_25*S6-C4*C6);
    ROcp48_66 = -(ROcp48_35*S6-S4*C6);
    OMcp48_25 = qd[5]*C4;
    OMcp48_35 = qd[5]*S4;
    OMcp48_16 = qd[4]+qd[6]*S5;
    OMcp48_26 = OMcp48_25+ROcp48_85*qd[6];
    OMcp48_36 = OMcp48_35+ROcp48_95*qd[6];
    OPcp48_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp48_26 = ROcp48_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp48_35*S5-ROcp48_95*qd[4]);
    OPcp48_36 = ROcp48_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp48_25*S5-ROcp48_85*qd[4]);

// = = Block_1_0_0_49_0_4 = = 
 
// Sensor Kinematics 


    ROcp48_419 = ROcp48_46*C19+S19*S5;
    ROcp48_519 = ROcp48_56*C19+ROcp48_85*S19;
    ROcp48_619 = ROcp48_66*C19+ROcp48_95*S19;
    ROcp48_719 = -(ROcp48_46*S19-C19*S5);
    ROcp48_819 = -(ROcp48_56*S19-ROcp48_85*C19);
    ROcp48_919 = -(ROcp48_66*S19-ROcp48_95*C19);
    ROcp48_120 = ROcp48_16*C20-ROcp48_719*S20;
    ROcp48_220 = ROcp48_26*C20-ROcp48_819*S20;
    ROcp48_320 = ROcp48_36*C20-ROcp48_919*S20;
    ROcp48_720 = ROcp48_16*S20+ROcp48_719*C20;
    ROcp48_820 = ROcp48_26*S20+ROcp48_819*C20;
    ROcp48_920 = ROcp48_36*S20+ROcp48_919*C20;
    ROcp48_121 = ROcp48_120*C21+ROcp48_419*S21;
    ROcp48_221 = ROcp48_220*C21+ROcp48_519*S21;
    ROcp48_321 = ROcp48_320*C21+ROcp48_619*S21;
    ROcp48_421 = -(ROcp48_120*S21-ROcp48_419*C21);
    ROcp48_521 = -(ROcp48_220*S21-ROcp48_519*C21);
    ROcp48_621 = -(ROcp48_320*S21-ROcp48_619*C21);
    RLcp48_119 = s->dpt[2][3]*ROcp48_46+ROcp48_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp48_219 = s->dpt[2][3]*ROcp48_56+ROcp48_26*s->dpt[1][3]+ROcp48_85*s->dpt[3][3];
    RLcp48_319 = s->dpt[2][3]*ROcp48_66+ROcp48_36*s->dpt[1][3]+ROcp48_95*s->dpt[3][3];
    OMcp48_119 = OMcp48_16+ROcp48_16*qd[19];
    OMcp48_219 = OMcp48_26+ROcp48_26*qd[19];
    OMcp48_319 = OMcp48_36+ROcp48_36*qd[19];
    ORcp48_119 = OMcp48_26*RLcp48_319-OMcp48_36*RLcp48_219;
    ORcp48_219 = -(OMcp48_16*RLcp48_319-OMcp48_36*RLcp48_119);
    ORcp48_319 = OMcp48_16*RLcp48_219-OMcp48_26*RLcp48_119;
    OPcp48_119 = OPcp48_16+ROcp48_16*qdd[19]+qd[19]*(OMcp48_26*ROcp48_36-OMcp48_36*ROcp48_26);
    OPcp48_219 = OPcp48_26+ROcp48_26*qdd[19]-qd[19]*(OMcp48_16*ROcp48_36-OMcp48_36*ROcp48_16);
    OPcp48_319 = OPcp48_36+ROcp48_36*qdd[19]+qd[19]*(OMcp48_16*ROcp48_26-OMcp48_26*ROcp48_16);
    RLcp48_120 = s->dpt[1][38]*ROcp48_16+s->dpt[2][38]*ROcp48_419+s->dpt[3][38]*ROcp48_719;
    RLcp48_220 = s->dpt[1][38]*ROcp48_26+s->dpt[2][38]*ROcp48_519+s->dpt[3][38]*ROcp48_819;
    RLcp48_320 = s->dpt[1][38]*ROcp48_36+s->dpt[2][38]*ROcp48_619+s->dpt[3][38]*ROcp48_919;
    OMcp48_120 = OMcp48_119+ROcp48_419*qd[20];
    OMcp48_220 = OMcp48_219+ROcp48_519*qd[20];
    OMcp48_320 = OMcp48_319+ROcp48_619*qd[20];
    ORcp48_120 = OMcp48_219*RLcp48_320-OMcp48_319*RLcp48_220;
    ORcp48_220 = -(OMcp48_119*RLcp48_320-OMcp48_319*RLcp48_120);
    ORcp48_320 = OMcp48_119*RLcp48_220-OMcp48_219*RLcp48_120;
    OPcp48_120 = OPcp48_119+ROcp48_419*qdd[20]+qd[20]*(OMcp48_219*ROcp48_619-OMcp48_319*ROcp48_519);
    OPcp48_220 = OPcp48_219+ROcp48_519*qdd[20]-qd[20]*(OMcp48_119*ROcp48_619-OMcp48_319*ROcp48_419);
    OPcp48_320 = OPcp48_319+ROcp48_619*qdd[20]+qd[20]*(OMcp48_119*ROcp48_519-OMcp48_219*ROcp48_419);
    RLcp48_121 = s->dpt[1][40]*ROcp48_120+s->dpt[2][40]*ROcp48_419+ROcp48_720*s->dpt[3][40];
    RLcp48_221 = s->dpt[1][40]*ROcp48_220+s->dpt[2][40]*ROcp48_519+ROcp48_820*s->dpt[3][40];
    RLcp48_321 = s->dpt[1][40]*ROcp48_320+s->dpt[2][40]*ROcp48_619+ROcp48_920*s->dpt[3][40];
    OMcp48_121 = OMcp48_120+ROcp48_720*qd[21];
    OMcp48_221 = OMcp48_220+ROcp48_820*qd[21];
    OMcp48_321 = OMcp48_320+ROcp48_920*qd[21];
    ORcp48_121 = OMcp48_220*RLcp48_321-OMcp48_320*RLcp48_221;
    ORcp48_221 = -(OMcp48_120*RLcp48_321-OMcp48_320*RLcp48_121);
    ORcp48_321 = OMcp48_120*RLcp48_221-OMcp48_220*RLcp48_121;
    OPcp48_121 = OPcp48_120+ROcp48_720*qdd[21]+qd[21]*(OMcp48_220*ROcp48_920-OMcp48_320*ROcp48_820);
    OPcp48_221 = OPcp48_220+ROcp48_820*qdd[21]-qd[21]*(OMcp48_120*ROcp48_920-OMcp48_320*ROcp48_720);
    OPcp48_321 = OPcp48_320+ROcp48_920*qdd[21]+qd[21]*(OMcp48_120*ROcp48_820-OMcp48_220*ROcp48_720);

// = = Block_1_0_0_49_0_6 = = 
 
// Sensor Kinematics 


    ROcp48_129 = ROcp48_121*C29-ROcp48_720*S29;
    ROcp48_229 = ROcp48_221*C29-ROcp48_820*S29;
    ROcp48_329 = ROcp48_321*C29-ROcp48_920*S29;
    ROcp48_729 = ROcp48_121*S29+ROcp48_720*C29;
    ROcp48_829 = ROcp48_221*S29+ROcp48_820*C29;
    ROcp48_929 = ROcp48_321*S29+ROcp48_920*C29;
    ROcp48_430 = ROcp48_421*C30+ROcp48_729*S30;
    ROcp48_530 = ROcp48_521*C30+ROcp48_829*S30;
    ROcp48_630 = ROcp48_621*C30+ROcp48_929*S30;
    ROcp48_730 = -(ROcp48_421*S30-ROcp48_729*C30);
    ROcp48_830 = -(ROcp48_521*S30-ROcp48_829*C30);
    ROcp48_930 = -(ROcp48_621*S30-ROcp48_929*C30);
    ROcp48_131 = ROcp48_129*C31-ROcp48_730*S31;
    ROcp48_231 = ROcp48_229*C31-ROcp48_830*S31;
    ROcp48_331 = ROcp48_329*C31-ROcp48_930*S31;
    ROcp48_731 = ROcp48_129*S31+ROcp48_730*C31;
    ROcp48_831 = ROcp48_229*S31+ROcp48_830*C31;
    ROcp48_931 = ROcp48_329*S31+ROcp48_930*C31;
    RLcp48_129 = ROcp48_121*s->dpt[1][45]+ROcp48_421*s->dpt[2][45]+ROcp48_720*s->dpt[3][45];
    RLcp48_229 = ROcp48_221*s->dpt[1][45]+ROcp48_521*s->dpt[2][45]+ROcp48_820*s->dpt[3][45];
    RLcp48_329 = ROcp48_321*s->dpt[1][45]+ROcp48_621*s->dpt[2][45]+ROcp48_920*s->dpt[3][45];
    OMcp48_129 = OMcp48_121+ROcp48_421*qd[29];
    OMcp48_229 = OMcp48_221+ROcp48_521*qd[29];
    OMcp48_329 = OMcp48_321+ROcp48_621*qd[29];
    ORcp48_129 = OMcp48_221*RLcp48_329-OMcp48_321*RLcp48_229;
    ORcp48_229 = -(OMcp48_121*RLcp48_329-OMcp48_321*RLcp48_129);
    ORcp48_329 = OMcp48_121*RLcp48_229-OMcp48_221*RLcp48_129;
    OPcp48_129 = OPcp48_121+ROcp48_421*qdd[29]+qd[29]*(OMcp48_221*ROcp48_621-OMcp48_321*ROcp48_521);
    OPcp48_229 = OPcp48_221+ROcp48_521*qdd[29]-qd[29]*(OMcp48_121*ROcp48_621-OMcp48_321*ROcp48_421);
    OPcp48_329 = OPcp48_321+ROcp48_621*qdd[29]+qd[29]*(OMcp48_121*ROcp48_521-OMcp48_221*ROcp48_421);
    RLcp48_130 = s->dpt[1][59]*ROcp48_129+s->dpt[3][59]*ROcp48_729+ROcp48_421*s->dpt[2][59];
    RLcp48_230 = s->dpt[1][59]*ROcp48_229+s->dpt[3][59]*ROcp48_829+ROcp48_521*s->dpt[2][59];
    RLcp48_330 = s->dpt[1][59]*ROcp48_329+s->dpt[3][59]*ROcp48_929+ROcp48_621*s->dpt[2][59];
    OMcp48_130 = OMcp48_129+ROcp48_129*qd[30];
    OMcp48_230 = OMcp48_229+ROcp48_229*qd[30];
    OMcp48_330 = OMcp48_329+ROcp48_329*qd[30];
    ORcp48_130 = OMcp48_229*RLcp48_330-OMcp48_329*RLcp48_230;
    ORcp48_230 = -(OMcp48_129*RLcp48_330-OMcp48_329*RLcp48_130);
    ORcp48_330 = OMcp48_129*RLcp48_230-OMcp48_229*RLcp48_130;
    OPcp48_130 = OPcp48_129+ROcp48_129*qdd[30]+qd[30]*(OMcp48_229*ROcp48_329-OMcp48_329*ROcp48_229);
    OPcp48_230 = OPcp48_229+ROcp48_229*qdd[30]-qd[30]*(OMcp48_129*ROcp48_329-OMcp48_329*ROcp48_129);
    OPcp48_330 = OPcp48_329+ROcp48_329*qdd[30]+qd[30]*(OMcp48_129*ROcp48_229-OMcp48_229*ROcp48_129);
    RLcp48_131 = s->dpt[1][61]*ROcp48_129+s->dpt[3][61]*ROcp48_730+ROcp48_430*s->dpt[2][61];
    RLcp48_231 = s->dpt[1][61]*ROcp48_229+s->dpt[3][61]*ROcp48_830+ROcp48_530*s->dpt[2][61];
    RLcp48_331 = s->dpt[1][61]*ROcp48_329+s->dpt[3][61]*ROcp48_930+ROcp48_630*s->dpt[2][61];
    OMcp48_131 = OMcp48_130+ROcp48_430*qd[31];
    OMcp48_231 = OMcp48_230+ROcp48_530*qd[31];
    OMcp48_331 = OMcp48_330+ROcp48_630*qd[31];
    ORcp48_131 = OMcp48_230*RLcp48_331-OMcp48_330*RLcp48_231;
    ORcp48_231 = -(OMcp48_130*RLcp48_331-OMcp48_330*RLcp48_131);
    ORcp48_331 = OMcp48_130*RLcp48_231-OMcp48_230*RLcp48_131;
    OPcp48_131 = OPcp48_130+ROcp48_430*qdd[31]+qd[31]*(OMcp48_230*ROcp48_630-OMcp48_330*ROcp48_530);
    OPcp48_231 = OPcp48_230+ROcp48_530*qdd[31]-qd[31]*(OMcp48_130*ROcp48_630-OMcp48_330*ROcp48_430);
    OPcp48_331 = OPcp48_330+ROcp48_630*qdd[31]+qd[31]*(OMcp48_130*ROcp48_530-OMcp48_230*ROcp48_430);
    RLcp48_184 = s->dpt[3][63]*ROcp48_731+ROcp48_131*s->dpt[1][63]+ROcp48_430*s->dpt[2][63];
    RLcp48_284 = s->dpt[3][63]*ROcp48_831+ROcp48_231*s->dpt[1][63]+ROcp48_530*s->dpt[2][63];
    RLcp48_384 = s->dpt[3][63]*ROcp48_931+ROcp48_331*s->dpt[1][63]+ROcp48_630*s->dpt[2][63];
    POcp48_184 = RLcp48_119+RLcp48_120+RLcp48_121+RLcp48_129+RLcp48_130+RLcp48_131+RLcp48_184+q[1];
    POcp48_284 = RLcp48_219+RLcp48_220+RLcp48_221+RLcp48_229+RLcp48_230+RLcp48_231+RLcp48_284+q[2];
    POcp48_384 = RLcp48_319+RLcp48_320+RLcp48_321+RLcp48_329+RLcp48_330+RLcp48_331+RLcp48_384+q[3];
    JTcp48_284_4 = -(RLcp48_319+RLcp48_320+RLcp48_321+RLcp48_329+RLcp48_330+RLcp48_331+RLcp48_384);
    JTcp48_384_4 = RLcp48_219+RLcp48_220+RLcp48_221+RLcp48_229+RLcp48_230+RLcp48_231+RLcp48_284;
    JTcp48_184_5 = C4*(RLcp48_319+RLcp48_320+RLcp48_321+RLcp48_329+RLcp48_330+RLcp48_331)-S4*(RLcp48_219+RLcp48_220)-S4*(
 RLcp48_221+RLcp48_229)-S4*(RLcp48_230+RLcp48_231)-RLcp48_284*S4+RLcp48_384*C4;
    JTcp48_284_5 = S4*(RLcp48_119+RLcp48_120+RLcp48_121+RLcp48_129+RLcp48_130+RLcp48_131+RLcp48_184);
    JTcp48_384_5 = -C4*(RLcp48_119+RLcp48_120+RLcp48_121+RLcp48_129+RLcp48_130+RLcp48_131+RLcp48_184);
    JTcp48_184_6 = ROcp48_85*(RLcp48_319+RLcp48_320+RLcp48_321+RLcp48_329+RLcp48_330+RLcp48_331)-ROcp48_95*(RLcp48_219+
 RLcp48_220)-ROcp48_95*(RLcp48_221+RLcp48_229)-ROcp48_95*(RLcp48_230+RLcp48_231)-RLcp48_284*ROcp48_95+RLcp48_384*ROcp48_85;
    JTcp48_284_6 = -(RLcp48_384*S5-ROcp48_95*(RLcp48_119+RLcp48_120+RLcp48_121+RLcp48_129+RLcp48_130+RLcp48_131+RLcp48_184
 )+S5*(RLcp48_319+RLcp48_320)+S5*(RLcp48_321+RLcp48_329)+S5*(RLcp48_330+RLcp48_331));
    JTcp48_384_6 = RLcp48_284*S5-ROcp48_85*(RLcp48_119+RLcp48_120+RLcp48_121+RLcp48_129+RLcp48_130+RLcp48_131+RLcp48_184)+
 S5*(RLcp48_219+RLcp48_220)+S5*(RLcp48_221+RLcp48_229)+S5*(RLcp48_230+RLcp48_231);
    JTcp48_184_7 = ROcp48_26*(RLcp48_320+RLcp48_321+RLcp48_329+RLcp48_330+RLcp48_331+RLcp48_384)-ROcp48_36*(RLcp48_220+
 RLcp48_221)-ROcp48_36*(RLcp48_229+RLcp48_230)-ROcp48_36*(RLcp48_231+RLcp48_284);
    JTcp48_284_7 = -(ROcp48_16*(RLcp48_320+RLcp48_321+RLcp48_329+RLcp48_330+RLcp48_331+RLcp48_384)-ROcp48_36*(RLcp48_120+
 RLcp48_121)-ROcp48_36*(RLcp48_129+RLcp48_130)-ROcp48_36*(RLcp48_131+RLcp48_184));
    JTcp48_384_7 = ROcp48_16*(RLcp48_220+RLcp48_221+RLcp48_229+RLcp48_230+RLcp48_231+RLcp48_284)-ROcp48_26*(RLcp48_120+
 RLcp48_121)-ROcp48_26*(RLcp48_129+RLcp48_130)-ROcp48_26*(RLcp48_131+RLcp48_184);
    JTcp48_184_8 = ROcp48_519*(RLcp48_321+RLcp48_329+RLcp48_330+RLcp48_331)-ROcp48_619*(RLcp48_221+RLcp48_229)-ROcp48_619*
 (RLcp48_230+RLcp48_231)-RLcp48_284*ROcp48_619+RLcp48_384*ROcp48_519;
    JTcp48_284_8 = RLcp48_184*ROcp48_619-RLcp48_384*ROcp48_419-ROcp48_419*(RLcp48_321+RLcp48_329+RLcp48_330+RLcp48_331)+
 ROcp48_619*(RLcp48_121+RLcp48_129)+ROcp48_619*(RLcp48_130+RLcp48_131);
    JTcp48_384_8 = ROcp48_419*(RLcp48_221+RLcp48_229+RLcp48_230+RLcp48_231)-ROcp48_519*(RLcp48_121+RLcp48_129)-ROcp48_519*
 (RLcp48_130+RLcp48_131)-RLcp48_184*ROcp48_519+RLcp48_284*ROcp48_419;
    JTcp48_184_9 = ROcp48_820*(RLcp48_329+RLcp48_330+RLcp48_331+RLcp48_384)-ROcp48_920*(RLcp48_229+RLcp48_230)-ROcp48_920*
 (RLcp48_231+RLcp48_284);
    JTcp48_284_9 = -(ROcp48_720*(RLcp48_329+RLcp48_330+RLcp48_331+RLcp48_384)-ROcp48_920*(RLcp48_129+RLcp48_130)-
 ROcp48_920*(RLcp48_131+RLcp48_184));
    JTcp48_384_9 = ROcp48_720*(RLcp48_229+RLcp48_230+RLcp48_231+RLcp48_284)-ROcp48_820*(RLcp48_129+RLcp48_130)-ROcp48_820*
 (RLcp48_131+RLcp48_184);
    JTcp48_184_10 = ROcp48_521*(RLcp48_330+RLcp48_331)-ROcp48_621*(RLcp48_230+RLcp48_231)-RLcp48_284*ROcp48_621+RLcp48_384
 *ROcp48_521;
    JTcp48_284_10 = RLcp48_184*ROcp48_621-RLcp48_384*ROcp48_421-ROcp48_421*(RLcp48_330+RLcp48_331)+ROcp48_621*(RLcp48_130+
 RLcp48_131);
    JTcp48_384_10 = ROcp48_421*(RLcp48_230+RLcp48_231)-ROcp48_521*(RLcp48_130+RLcp48_131)-RLcp48_184*ROcp48_521+RLcp48_284
 *ROcp48_421;
    JTcp48_184_11 = ROcp48_229*(RLcp48_331+RLcp48_384)-ROcp48_329*(RLcp48_231+RLcp48_284);
    JTcp48_284_11 = -(ROcp48_129*(RLcp48_331+RLcp48_384)-ROcp48_329*(RLcp48_131+RLcp48_184));
    JTcp48_384_11 = ROcp48_129*(RLcp48_231+RLcp48_284)-ROcp48_229*(RLcp48_131+RLcp48_184);
    JTcp48_184_12 = -(RLcp48_284*ROcp48_630-RLcp48_384*ROcp48_530);
    JTcp48_284_12 = RLcp48_184*ROcp48_630-RLcp48_384*ROcp48_430;
    JTcp48_384_12 = -(RLcp48_184*ROcp48_530-RLcp48_284*ROcp48_430);
    ORcp48_184 = OMcp48_231*RLcp48_384-OMcp48_331*RLcp48_284;
    ORcp48_284 = -(OMcp48_131*RLcp48_384-OMcp48_331*RLcp48_184);
    ORcp48_384 = OMcp48_131*RLcp48_284-OMcp48_231*RLcp48_184;
    VIcp48_184 = ORcp48_119+ORcp48_120+ORcp48_121+ORcp48_129+ORcp48_130+ORcp48_131+ORcp48_184+qd[1];
    VIcp48_284 = ORcp48_219+ORcp48_220+ORcp48_221+ORcp48_229+ORcp48_230+ORcp48_231+ORcp48_284+qd[2];
    VIcp48_384 = ORcp48_319+ORcp48_320+ORcp48_321+ORcp48_329+ORcp48_330+ORcp48_331+ORcp48_384+qd[3];
    ACcp48_184 = qdd[1]+OMcp48_219*ORcp48_320+OMcp48_220*ORcp48_321+OMcp48_221*ORcp48_329+OMcp48_229*ORcp48_330+OMcp48_230
 *ORcp48_331+OMcp48_231*ORcp48_384+OMcp48_26*ORcp48_319-OMcp48_319*ORcp48_220-OMcp48_320*ORcp48_221-OMcp48_321*ORcp48_229-
 OMcp48_329*ORcp48_230-OMcp48_330*ORcp48_231-OMcp48_331*ORcp48_284-OMcp48_36*ORcp48_219+OPcp48_219*RLcp48_320+OPcp48_220*
 RLcp48_321+OPcp48_221*RLcp48_329+OPcp48_229*RLcp48_330+OPcp48_230*RLcp48_331+OPcp48_231*RLcp48_384+OPcp48_26*RLcp48_319-
 OPcp48_319*RLcp48_220-OPcp48_320*RLcp48_221-OPcp48_321*RLcp48_229-OPcp48_329*RLcp48_230-OPcp48_330*RLcp48_231-OPcp48_331*
 RLcp48_284-OPcp48_36*RLcp48_219;
    ACcp48_284 = qdd[2]-OMcp48_119*ORcp48_320-OMcp48_120*ORcp48_321-OMcp48_121*ORcp48_329-OMcp48_129*ORcp48_330-OMcp48_130
 *ORcp48_331-OMcp48_131*ORcp48_384-OMcp48_16*ORcp48_319+OMcp48_319*ORcp48_120+OMcp48_320*ORcp48_121+OMcp48_321*ORcp48_129+
 OMcp48_329*ORcp48_130+OMcp48_330*ORcp48_131+OMcp48_331*ORcp48_184+OMcp48_36*ORcp48_119-OPcp48_119*RLcp48_320-OPcp48_120*
 RLcp48_321-OPcp48_121*RLcp48_329-OPcp48_129*RLcp48_330-OPcp48_130*RLcp48_331-OPcp48_131*RLcp48_384-OPcp48_16*RLcp48_319+
 OPcp48_319*RLcp48_120+OPcp48_320*RLcp48_121+OPcp48_321*RLcp48_129+OPcp48_329*RLcp48_130+OPcp48_330*RLcp48_131+OPcp48_331*
 RLcp48_184+OPcp48_36*RLcp48_119;
    ACcp48_384 = qdd[3]+OMcp48_119*ORcp48_220+OMcp48_120*ORcp48_221+OMcp48_121*ORcp48_229+OMcp48_129*ORcp48_230+OMcp48_130
 *ORcp48_231+OMcp48_131*ORcp48_284+OMcp48_16*ORcp48_219-OMcp48_219*ORcp48_120-OMcp48_220*ORcp48_121-OMcp48_221*ORcp48_129-
 OMcp48_229*ORcp48_130-OMcp48_230*ORcp48_131-OMcp48_231*ORcp48_184-OMcp48_26*ORcp48_119+OPcp48_119*RLcp48_220+OPcp48_120*
 RLcp48_221+OPcp48_121*RLcp48_229+OPcp48_129*RLcp48_230+OPcp48_130*RLcp48_231+OPcp48_131*RLcp48_284+OPcp48_16*RLcp48_219-
 OPcp48_219*RLcp48_120-OPcp48_220*RLcp48_121-OPcp48_221*RLcp48_129-OPcp48_229*RLcp48_130-OPcp48_230*RLcp48_131-OPcp48_231*
 RLcp48_184-OPcp48_26*RLcp48_119;

// = = Block_1_0_0_49_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp48_184;
    sens->P[2] = POcp48_284;
    sens->P[3] = POcp48_384;
    sens->R[1][1] = ROcp48_131;
    sens->R[1][2] = ROcp48_231;
    sens->R[1][3] = ROcp48_331;
    sens->R[2][1] = ROcp48_430;
    sens->R[2][2] = ROcp48_530;
    sens->R[2][3] = ROcp48_630;
    sens->R[3][1] = ROcp48_731;
    sens->R[3][2] = ROcp48_831;
    sens->R[3][3] = ROcp48_931;
    sens->V[1] = VIcp48_184;
    sens->V[2] = VIcp48_284;
    sens->V[3] = VIcp48_384;
    sens->OM[1] = OMcp48_131;
    sens->OM[2] = OMcp48_231;
    sens->OM[3] = OMcp48_331;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp48_184_5;
    sens->J[1][6] = JTcp48_184_6;
    sens->J[1][19] = JTcp48_184_7;
    sens->J[1][20] = JTcp48_184_8;
    sens->J[1][21] = JTcp48_184_9;
    sens->J[1][29] = JTcp48_184_10;
    sens->J[1][30] = JTcp48_184_11;
    sens->J[1][31] = JTcp48_184_12;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp48_284_4;
    sens->J[2][5] = JTcp48_284_5;
    sens->J[2][6] = JTcp48_284_6;
    sens->J[2][19] = JTcp48_284_7;
    sens->J[2][20] = JTcp48_284_8;
    sens->J[2][21] = JTcp48_284_9;
    sens->J[2][29] = JTcp48_284_10;
    sens->J[2][30] = JTcp48_284_11;
    sens->J[2][31] = JTcp48_284_12;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp48_384_4;
    sens->J[3][5] = JTcp48_384_5;
    sens->J[3][6] = JTcp48_384_6;
    sens->J[3][19] = JTcp48_384_7;
    sens->J[3][20] = JTcp48_384_8;
    sens->J[3][21] = JTcp48_384_9;
    sens->J[3][29] = JTcp48_384_10;
    sens->J[3][30] = JTcp48_384_11;
    sens->J[3][31] = JTcp48_384_12;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp48_16;
    sens->J[4][20] = ROcp48_419;
    sens->J[4][21] = ROcp48_720;
    sens->J[4][29] = ROcp48_421;
    sens->J[4][30] = ROcp48_129;
    sens->J[4][31] = ROcp48_430;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp48_85;
    sens->J[5][19] = ROcp48_26;
    sens->J[5][20] = ROcp48_519;
    sens->J[5][21] = ROcp48_820;
    sens->J[5][29] = ROcp48_521;
    sens->J[5][30] = ROcp48_229;
    sens->J[5][31] = ROcp48_530;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp48_95;
    sens->J[6][19] = ROcp48_36;
    sens->J[6][20] = ROcp48_619;
    sens->J[6][21] = ROcp48_920;
    sens->J[6][29] = ROcp48_621;
    sens->J[6][30] = ROcp48_329;
    sens->J[6][31] = ROcp48_630;
    sens->A[1] = ACcp48_184;
    sens->A[2] = ACcp48_284;
    sens->A[3] = ACcp48_384;
    sens->OMP[1] = OPcp48_131;
    sens->OMP[2] = OPcp48_231;
    sens->OMP[3] = OPcp48_331;
 
// 
break;
case 50:
 


// = = Block_1_0_0_50_0_1 = = 
 
// Sensor Kinematics 


    ROcp49_25 = S4*S5;
    ROcp49_35 = -C4*S5;
    ROcp49_85 = -S4*C5;
    ROcp49_95 = C4*C5;
    ROcp49_16 = C5*C6;
    ROcp49_26 = ROcp49_25*C6+C4*S6;
    ROcp49_36 = ROcp49_35*C6+S4*S6;
    ROcp49_46 = -C5*S6;
    ROcp49_56 = -(ROcp49_25*S6-C4*C6);
    ROcp49_66 = -(ROcp49_35*S6-S4*C6);
    OMcp49_25 = qd[5]*C4;
    OMcp49_35 = qd[5]*S4;
    OMcp49_16 = qd[4]+qd[6]*S5;
    OMcp49_26 = OMcp49_25+ROcp49_85*qd[6];
    OMcp49_36 = OMcp49_35+ROcp49_95*qd[6];
    OPcp49_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp49_26 = ROcp49_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp49_35*S5-ROcp49_95*qd[4]);
    OPcp49_36 = ROcp49_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp49_25*S5-ROcp49_85*qd[4]);

// = = Block_1_0_0_50_0_4 = = 
 
// Sensor Kinematics 


    ROcp49_419 = ROcp49_46*C19+S19*S5;
    ROcp49_519 = ROcp49_56*C19+ROcp49_85*S19;
    ROcp49_619 = ROcp49_66*C19+ROcp49_95*S19;
    ROcp49_719 = -(ROcp49_46*S19-C19*S5);
    ROcp49_819 = -(ROcp49_56*S19-ROcp49_85*C19);
    ROcp49_919 = -(ROcp49_66*S19-ROcp49_95*C19);
    ROcp49_120 = ROcp49_16*C20-ROcp49_719*S20;
    ROcp49_220 = ROcp49_26*C20-ROcp49_819*S20;
    ROcp49_320 = ROcp49_36*C20-ROcp49_919*S20;
    ROcp49_720 = ROcp49_16*S20+ROcp49_719*C20;
    ROcp49_820 = ROcp49_26*S20+ROcp49_819*C20;
    ROcp49_920 = ROcp49_36*S20+ROcp49_919*C20;
    ROcp49_121 = ROcp49_120*C21+ROcp49_419*S21;
    ROcp49_221 = ROcp49_220*C21+ROcp49_519*S21;
    ROcp49_321 = ROcp49_320*C21+ROcp49_619*S21;
    ROcp49_421 = -(ROcp49_120*S21-ROcp49_419*C21);
    ROcp49_521 = -(ROcp49_220*S21-ROcp49_519*C21);
    ROcp49_621 = -(ROcp49_320*S21-ROcp49_619*C21);
    RLcp49_119 = s->dpt[2][3]*ROcp49_46+ROcp49_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp49_219 = s->dpt[2][3]*ROcp49_56+ROcp49_26*s->dpt[1][3]+ROcp49_85*s->dpt[3][3];
    RLcp49_319 = s->dpt[2][3]*ROcp49_66+ROcp49_36*s->dpt[1][3]+ROcp49_95*s->dpt[3][3];
    OMcp49_119 = OMcp49_16+ROcp49_16*qd[19];
    OMcp49_219 = OMcp49_26+ROcp49_26*qd[19];
    OMcp49_319 = OMcp49_36+ROcp49_36*qd[19];
    ORcp49_119 = OMcp49_26*RLcp49_319-OMcp49_36*RLcp49_219;
    ORcp49_219 = -(OMcp49_16*RLcp49_319-OMcp49_36*RLcp49_119);
    ORcp49_319 = OMcp49_16*RLcp49_219-OMcp49_26*RLcp49_119;
    OPcp49_119 = OPcp49_16+ROcp49_16*qdd[19]+qd[19]*(OMcp49_26*ROcp49_36-OMcp49_36*ROcp49_26);
    OPcp49_219 = OPcp49_26+ROcp49_26*qdd[19]-qd[19]*(OMcp49_16*ROcp49_36-OMcp49_36*ROcp49_16);
    OPcp49_319 = OPcp49_36+ROcp49_36*qdd[19]+qd[19]*(OMcp49_16*ROcp49_26-OMcp49_26*ROcp49_16);
    RLcp49_120 = s->dpt[1][38]*ROcp49_16+s->dpt[2][38]*ROcp49_419+s->dpt[3][38]*ROcp49_719;
    RLcp49_220 = s->dpt[1][38]*ROcp49_26+s->dpt[2][38]*ROcp49_519+s->dpt[3][38]*ROcp49_819;
    RLcp49_320 = s->dpt[1][38]*ROcp49_36+s->dpt[2][38]*ROcp49_619+s->dpt[3][38]*ROcp49_919;
    OMcp49_120 = OMcp49_119+ROcp49_419*qd[20];
    OMcp49_220 = OMcp49_219+ROcp49_519*qd[20];
    OMcp49_320 = OMcp49_319+ROcp49_619*qd[20];
    ORcp49_120 = OMcp49_219*RLcp49_320-OMcp49_319*RLcp49_220;
    ORcp49_220 = -(OMcp49_119*RLcp49_320-OMcp49_319*RLcp49_120);
    ORcp49_320 = OMcp49_119*RLcp49_220-OMcp49_219*RLcp49_120;
    OPcp49_120 = OPcp49_119+ROcp49_419*qdd[20]+qd[20]*(OMcp49_219*ROcp49_619-OMcp49_319*ROcp49_519);
    OPcp49_220 = OPcp49_219+ROcp49_519*qdd[20]-qd[20]*(OMcp49_119*ROcp49_619-OMcp49_319*ROcp49_419);
    OPcp49_320 = OPcp49_319+ROcp49_619*qdd[20]+qd[20]*(OMcp49_119*ROcp49_519-OMcp49_219*ROcp49_419);
    RLcp49_121 = s->dpt[1][40]*ROcp49_120+s->dpt[2][40]*ROcp49_419+ROcp49_720*s->dpt[3][40];
    RLcp49_221 = s->dpt[1][40]*ROcp49_220+s->dpt[2][40]*ROcp49_519+ROcp49_820*s->dpt[3][40];
    RLcp49_321 = s->dpt[1][40]*ROcp49_320+s->dpt[2][40]*ROcp49_619+ROcp49_920*s->dpt[3][40];
    OMcp49_121 = OMcp49_120+ROcp49_720*qd[21];
    OMcp49_221 = OMcp49_220+ROcp49_820*qd[21];
    OMcp49_321 = OMcp49_320+ROcp49_920*qd[21];
    ORcp49_121 = OMcp49_220*RLcp49_321-OMcp49_320*RLcp49_221;
    ORcp49_221 = -(OMcp49_120*RLcp49_321-OMcp49_320*RLcp49_121);
    ORcp49_321 = OMcp49_120*RLcp49_221-OMcp49_220*RLcp49_121;
    OPcp49_121 = OPcp49_120+ROcp49_720*qdd[21]+qd[21]*(OMcp49_220*ROcp49_920-OMcp49_320*ROcp49_820);
    OPcp49_221 = OPcp49_220+ROcp49_820*qdd[21]-qd[21]*(OMcp49_120*ROcp49_920-OMcp49_320*ROcp49_720);
    OPcp49_321 = OPcp49_320+ROcp49_920*qdd[21]+qd[21]*(OMcp49_120*ROcp49_820-OMcp49_220*ROcp49_720);

// = = Block_1_0_0_50_0_6 = = 
 
// Sensor Kinematics 


    ROcp49_129 = ROcp49_121*C29-ROcp49_720*S29;
    ROcp49_229 = ROcp49_221*C29-ROcp49_820*S29;
    ROcp49_329 = ROcp49_321*C29-ROcp49_920*S29;
    ROcp49_729 = ROcp49_121*S29+ROcp49_720*C29;
    ROcp49_829 = ROcp49_221*S29+ROcp49_820*C29;
    ROcp49_929 = ROcp49_321*S29+ROcp49_920*C29;
    ROcp49_430 = ROcp49_421*C30+ROcp49_729*S30;
    ROcp49_530 = ROcp49_521*C30+ROcp49_829*S30;
    ROcp49_630 = ROcp49_621*C30+ROcp49_929*S30;
    ROcp49_730 = -(ROcp49_421*S30-ROcp49_729*C30);
    ROcp49_830 = -(ROcp49_521*S30-ROcp49_829*C30);
    ROcp49_930 = -(ROcp49_621*S30-ROcp49_929*C30);
    ROcp49_131 = ROcp49_129*C31-ROcp49_730*S31;
    ROcp49_231 = ROcp49_229*C31-ROcp49_830*S31;
    ROcp49_331 = ROcp49_329*C31-ROcp49_930*S31;
    ROcp49_731 = ROcp49_129*S31+ROcp49_730*C31;
    ROcp49_831 = ROcp49_229*S31+ROcp49_830*C31;
    ROcp49_931 = ROcp49_329*S31+ROcp49_930*C31;
    ROcp49_132 = ROcp49_131*C32+ROcp49_430*S32;
    ROcp49_232 = ROcp49_231*C32+ROcp49_530*S32;
    ROcp49_332 = ROcp49_331*C32+ROcp49_630*S32;
    ROcp49_432 = -(ROcp49_131*S32-ROcp49_430*C32);
    ROcp49_532 = -(ROcp49_231*S32-ROcp49_530*C32);
    ROcp49_632 = -(ROcp49_331*S32-ROcp49_630*C32);
    RLcp49_129 = ROcp49_121*s->dpt[1][45]+ROcp49_421*s->dpt[2][45]+ROcp49_720*s->dpt[3][45];
    RLcp49_229 = ROcp49_221*s->dpt[1][45]+ROcp49_521*s->dpt[2][45]+ROcp49_820*s->dpt[3][45];
    RLcp49_329 = ROcp49_321*s->dpt[1][45]+ROcp49_621*s->dpt[2][45]+ROcp49_920*s->dpt[3][45];
    OMcp49_129 = OMcp49_121+ROcp49_421*qd[29];
    OMcp49_229 = OMcp49_221+ROcp49_521*qd[29];
    OMcp49_329 = OMcp49_321+ROcp49_621*qd[29];
    ORcp49_129 = OMcp49_221*RLcp49_329-OMcp49_321*RLcp49_229;
    ORcp49_229 = -(OMcp49_121*RLcp49_329-OMcp49_321*RLcp49_129);
    ORcp49_329 = OMcp49_121*RLcp49_229-OMcp49_221*RLcp49_129;
    OPcp49_129 = OPcp49_121+ROcp49_421*qdd[29]+qd[29]*(OMcp49_221*ROcp49_621-OMcp49_321*ROcp49_521);
    OPcp49_229 = OPcp49_221+ROcp49_521*qdd[29]-qd[29]*(OMcp49_121*ROcp49_621-OMcp49_321*ROcp49_421);
    OPcp49_329 = OPcp49_321+ROcp49_621*qdd[29]+qd[29]*(OMcp49_121*ROcp49_521-OMcp49_221*ROcp49_421);
    RLcp49_130 = s->dpt[1][59]*ROcp49_129+s->dpt[3][59]*ROcp49_729+ROcp49_421*s->dpt[2][59];
    RLcp49_230 = s->dpt[1][59]*ROcp49_229+s->dpt[3][59]*ROcp49_829+ROcp49_521*s->dpt[2][59];
    RLcp49_330 = s->dpt[1][59]*ROcp49_329+s->dpt[3][59]*ROcp49_929+ROcp49_621*s->dpt[2][59];
    OMcp49_130 = OMcp49_129+ROcp49_129*qd[30];
    OMcp49_230 = OMcp49_229+ROcp49_229*qd[30];
    OMcp49_330 = OMcp49_329+ROcp49_329*qd[30];
    ORcp49_130 = OMcp49_229*RLcp49_330-OMcp49_329*RLcp49_230;
    ORcp49_230 = -(OMcp49_129*RLcp49_330-OMcp49_329*RLcp49_130);
    ORcp49_330 = OMcp49_129*RLcp49_230-OMcp49_229*RLcp49_130;
    OPcp49_130 = OPcp49_129+ROcp49_129*qdd[30]+qd[30]*(OMcp49_229*ROcp49_329-OMcp49_329*ROcp49_229);
    OPcp49_230 = OPcp49_229+ROcp49_229*qdd[30]-qd[30]*(OMcp49_129*ROcp49_329-OMcp49_329*ROcp49_129);
    OPcp49_330 = OPcp49_329+ROcp49_329*qdd[30]+qd[30]*(OMcp49_129*ROcp49_229-OMcp49_229*ROcp49_129);
    RLcp49_131 = s->dpt[1][61]*ROcp49_129+s->dpt[3][61]*ROcp49_730+ROcp49_430*s->dpt[2][61];
    RLcp49_231 = s->dpt[1][61]*ROcp49_229+s->dpt[3][61]*ROcp49_830+ROcp49_530*s->dpt[2][61];
    RLcp49_331 = s->dpt[1][61]*ROcp49_329+s->dpt[3][61]*ROcp49_930+ROcp49_630*s->dpt[2][61];
    OMcp49_131 = OMcp49_130+ROcp49_430*qd[31];
    OMcp49_231 = OMcp49_230+ROcp49_530*qd[31];
    OMcp49_331 = OMcp49_330+ROcp49_630*qd[31];
    ORcp49_131 = OMcp49_230*RLcp49_331-OMcp49_330*RLcp49_231;
    ORcp49_231 = -(OMcp49_130*RLcp49_331-OMcp49_330*RLcp49_131);
    ORcp49_331 = OMcp49_130*RLcp49_231-OMcp49_230*RLcp49_131;
    OPcp49_131 = OPcp49_130+ROcp49_430*qdd[31]+qd[31]*(OMcp49_230*ROcp49_630-OMcp49_330*ROcp49_530);
    OPcp49_231 = OPcp49_230+ROcp49_530*qdd[31]-qd[31]*(OMcp49_130*ROcp49_630-OMcp49_330*ROcp49_430);
    OPcp49_331 = OPcp49_330+ROcp49_630*qdd[31]+qd[31]*(OMcp49_130*ROcp49_530-OMcp49_230*ROcp49_430);
    RLcp49_132 = s->dpt[3][63]*ROcp49_731+ROcp49_131*s->dpt[1][63]+ROcp49_430*s->dpt[2][63];
    RLcp49_232 = s->dpt[3][63]*ROcp49_831+ROcp49_231*s->dpt[1][63]+ROcp49_530*s->dpt[2][63];
    RLcp49_332 = s->dpt[3][63]*ROcp49_931+ROcp49_331*s->dpt[1][63]+ROcp49_630*s->dpt[2][63];
    OMcp49_132 = OMcp49_131+ROcp49_731*qd[32];
    OMcp49_232 = OMcp49_231+ROcp49_831*qd[32];
    OMcp49_332 = OMcp49_331+ROcp49_931*qd[32];
    ORcp49_132 = OMcp49_231*RLcp49_332-OMcp49_331*RLcp49_232;
    ORcp49_232 = -(OMcp49_131*RLcp49_332-OMcp49_331*RLcp49_132);
    ORcp49_332 = OMcp49_131*RLcp49_232-OMcp49_231*RLcp49_132;
    OPcp49_132 = OPcp49_131+ROcp49_731*qdd[32]+qd[32]*(OMcp49_231*ROcp49_931-OMcp49_331*ROcp49_831);
    OPcp49_232 = OPcp49_231+ROcp49_831*qdd[32]-qd[32]*(OMcp49_131*ROcp49_931-OMcp49_331*ROcp49_731);
    OPcp49_332 = OPcp49_331+ROcp49_931*qdd[32]+qd[32]*(OMcp49_131*ROcp49_831-OMcp49_231*ROcp49_731);
    RLcp49_185 = ROcp49_132*s->dpt[1][64]+ROcp49_432*s->dpt[2][64]+ROcp49_731*s->dpt[3][64];
    RLcp49_285 = ROcp49_232*s->dpt[1][64]+ROcp49_532*s->dpt[2][64]+ROcp49_831*s->dpt[3][64];
    RLcp49_385 = ROcp49_332*s->dpt[1][64]+ROcp49_632*s->dpt[2][64]+ROcp49_931*s->dpt[3][64];
    POcp49_185 = RLcp49_119+RLcp49_120+RLcp49_121+RLcp49_129+RLcp49_130+RLcp49_131+RLcp49_132+RLcp49_185+q[1];
    POcp49_285 = RLcp49_219+RLcp49_220+RLcp49_221+RLcp49_229+RLcp49_230+RLcp49_231+RLcp49_232+RLcp49_285+q[2];
    POcp49_385 = RLcp49_319+RLcp49_320+RLcp49_321+RLcp49_329+RLcp49_330+RLcp49_331+RLcp49_332+RLcp49_385+q[3];
    JTcp49_285_4 = -(RLcp49_319+RLcp49_320+RLcp49_321+RLcp49_329+RLcp49_330+RLcp49_331+RLcp49_332+RLcp49_385);
    JTcp49_385_4 = RLcp49_219+RLcp49_220+RLcp49_221+RLcp49_229+RLcp49_230+RLcp49_231+RLcp49_232+RLcp49_285;
    JTcp49_185_5 = C4*(RLcp49_319+RLcp49_320+RLcp49_321+RLcp49_329+RLcp49_330+RLcp49_331+RLcp49_332+RLcp49_385)-S4*(
 RLcp49_219+RLcp49_220)-S4*(RLcp49_221+RLcp49_229)-S4*(RLcp49_230+RLcp49_231)-S4*(RLcp49_232+RLcp49_285);
    JTcp49_285_5 = S4*(RLcp49_119+RLcp49_120+RLcp49_121+RLcp49_129+RLcp49_130+RLcp49_131+RLcp49_132+RLcp49_185);
    JTcp49_385_5 = -C4*(RLcp49_119+RLcp49_120+RLcp49_121+RLcp49_129+RLcp49_130+RLcp49_131+RLcp49_132+RLcp49_185);
    JTcp49_185_6 = ROcp49_85*(RLcp49_319+RLcp49_320+RLcp49_321+RLcp49_329+RLcp49_330+RLcp49_331+RLcp49_332+RLcp49_385)-
 ROcp49_95*(RLcp49_219+RLcp49_220)-ROcp49_95*(RLcp49_221+RLcp49_229)-ROcp49_95*(RLcp49_230+RLcp49_231)-ROcp49_95*(RLcp49_232+
 RLcp49_285);
    JTcp49_285_6 = RLcp49_185*ROcp49_95-RLcp49_332*S5-RLcp49_385*S5+ROcp49_95*(RLcp49_119+RLcp49_120+RLcp49_121+RLcp49_129
 +RLcp49_130+RLcp49_131+RLcp49_132)-S5*(RLcp49_319+RLcp49_320)-S5*(RLcp49_321+RLcp49_329)-S5*(RLcp49_330+RLcp49_331);
    JTcp49_385_6 = RLcp49_232*S5-ROcp49_85*(RLcp49_119+RLcp49_120+RLcp49_121+RLcp49_129+RLcp49_130+RLcp49_131+RLcp49_132)+
 S5*(RLcp49_219+RLcp49_220)+S5*(RLcp49_221+RLcp49_229)+S5*(RLcp49_230+RLcp49_231)-RLcp49_185*ROcp49_85+RLcp49_285*S5;
    JTcp49_185_7 = ROcp49_26*(RLcp49_320+RLcp49_321+RLcp49_329+RLcp49_330+RLcp49_331+RLcp49_332)-ROcp49_36*(RLcp49_220+
 RLcp49_221)-ROcp49_36*(RLcp49_229+RLcp49_230)-ROcp49_36*(RLcp49_231+RLcp49_232)-RLcp49_285*ROcp49_36+RLcp49_385*ROcp49_26;
    JTcp49_285_7 = RLcp49_185*ROcp49_36-RLcp49_385*ROcp49_16-ROcp49_16*(RLcp49_320+RLcp49_321+RLcp49_329+RLcp49_330+
 RLcp49_331+RLcp49_332)+ROcp49_36*(RLcp49_120+RLcp49_121)+ROcp49_36*(RLcp49_129+RLcp49_130)+ROcp49_36*(RLcp49_131+RLcp49_132);
    JTcp49_385_7 = ROcp49_16*(RLcp49_220+RLcp49_221+RLcp49_229+RLcp49_230+RLcp49_231+RLcp49_232)-ROcp49_26*(RLcp49_120+
 RLcp49_121)-ROcp49_26*(RLcp49_129+RLcp49_130)-ROcp49_26*(RLcp49_131+RLcp49_132)-RLcp49_185*ROcp49_26+RLcp49_285*ROcp49_16;
    JTcp49_185_8 = ROcp49_519*(RLcp49_321+RLcp49_329+RLcp49_330+RLcp49_331+RLcp49_332+RLcp49_385)-ROcp49_619*(RLcp49_221+
 RLcp49_229)-ROcp49_619*(RLcp49_230+RLcp49_231)-ROcp49_619*(RLcp49_232+RLcp49_285);
    JTcp49_285_8 = -(ROcp49_419*(RLcp49_321+RLcp49_329+RLcp49_330+RLcp49_331+RLcp49_332+RLcp49_385)-ROcp49_619*(RLcp49_121
 +RLcp49_129)-ROcp49_619*(RLcp49_130+RLcp49_131)-ROcp49_619*(RLcp49_132+RLcp49_185));
    JTcp49_385_8 = ROcp49_419*(RLcp49_221+RLcp49_229+RLcp49_230+RLcp49_231+RLcp49_232+RLcp49_285)-ROcp49_519*(RLcp49_121+
 RLcp49_129)-ROcp49_519*(RLcp49_130+RLcp49_131)-ROcp49_519*(RLcp49_132+RLcp49_185);
    JTcp49_185_9 = ROcp49_820*(RLcp49_329+RLcp49_330+RLcp49_331+RLcp49_332)-ROcp49_920*(RLcp49_229+RLcp49_230)-ROcp49_920*
 (RLcp49_231+RLcp49_232)-RLcp49_285*ROcp49_920+RLcp49_385*ROcp49_820;
    JTcp49_285_9 = RLcp49_185*ROcp49_920-RLcp49_385*ROcp49_720-ROcp49_720*(RLcp49_329+RLcp49_330+RLcp49_331+RLcp49_332)+
 ROcp49_920*(RLcp49_129+RLcp49_130)+ROcp49_920*(RLcp49_131+RLcp49_132);
    JTcp49_385_9 = ROcp49_720*(RLcp49_229+RLcp49_230+RLcp49_231+RLcp49_232)-ROcp49_820*(RLcp49_129+RLcp49_130)-ROcp49_820*
 (RLcp49_131+RLcp49_132)-RLcp49_185*ROcp49_820+RLcp49_285*ROcp49_720;
    JTcp49_185_10 = ROcp49_521*(RLcp49_330+RLcp49_331+RLcp49_332+RLcp49_385)-ROcp49_621*(RLcp49_230+RLcp49_231)-ROcp49_621
 *(RLcp49_232+RLcp49_285);
    JTcp49_285_10 = -(ROcp49_421*(RLcp49_330+RLcp49_331+RLcp49_332+RLcp49_385)-ROcp49_621*(RLcp49_130+RLcp49_131)-
 ROcp49_621*(RLcp49_132+RLcp49_185));
    JTcp49_385_10 = ROcp49_421*(RLcp49_230+RLcp49_231+RLcp49_232+RLcp49_285)-ROcp49_521*(RLcp49_130+RLcp49_131)-ROcp49_521
 *(RLcp49_132+RLcp49_185);
    JTcp49_185_11 = ROcp49_229*(RLcp49_331+RLcp49_332)-ROcp49_329*(RLcp49_231+RLcp49_232)-RLcp49_285*ROcp49_329+RLcp49_385
 *ROcp49_229;
    JTcp49_285_11 = RLcp49_185*ROcp49_329-RLcp49_385*ROcp49_129-ROcp49_129*(RLcp49_331+RLcp49_332)+ROcp49_329*(RLcp49_131+
 RLcp49_132);
    JTcp49_385_11 = ROcp49_129*(RLcp49_231+RLcp49_232)-ROcp49_229*(RLcp49_131+RLcp49_132)-RLcp49_185*ROcp49_229+RLcp49_285
 *ROcp49_129;
    JTcp49_185_12 = ROcp49_530*(RLcp49_332+RLcp49_385)-ROcp49_630*(RLcp49_232+RLcp49_285);
    JTcp49_285_12 = -(ROcp49_430*(RLcp49_332+RLcp49_385)-ROcp49_630*(RLcp49_132+RLcp49_185));
    JTcp49_385_12 = ROcp49_430*(RLcp49_232+RLcp49_285)-ROcp49_530*(RLcp49_132+RLcp49_185);
    JTcp49_185_13 = -(RLcp49_285*ROcp49_931-RLcp49_385*ROcp49_831);
    JTcp49_285_13 = RLcp49_185*ROcp49_931-RLcp49_385*ROcp49_731;
    JTcp49_385_13 = -(RLcp49_185*ROcp49_831-RLcp49_285*ROcp49_731);
    ORcp49_185 = OMcp49_232*RLcp49_385-OMcp49_332*RLcp49_285;
    ORcp49_285 = -(OMcp49_132*RLcp49_385-OMcp49_332*RLcp49_185);
    ORcp49_385 = OMcp49_132*RLcp49_285-OMcp49_232*RLcp49_185;
    VIcp49_185 = ORcp49_119+ORcp49_120+ORcp49_121+ORcp49_129+ORcp49_130+ORcp49_131+ORcp49_132+ORcp49_185+qd[1];
    VIcp49_285 = ORcp49_219+ORcp49_220+ORcp49_221+ORcp49_229+ORcp49_230+ORcp49_231+ORcp49_232+ORcp49_285+qd[2];
    VIcp49_385 = ORcp49_319+ORcp49_320+ORcp49_321+ORcp49_329+ORcp49_330+ORcp49_331+ORcp49_332+ORcp49_385+qd[3];
    ACcp49_185 = qdd[1]+OMcp49_219*ORcp49_320+OMcp49_220*ORcp49_321+OMcp49_221*ORcp49_329+OMcp49_229*ORcp49_330+OMcp49_230
 *ORcp49_331+OMcp49_231*ORcp49_332+OMcp49_232*ORcp49_385+OMcp49_26*ORcp49_319-OMcp49_319*ORcp49_220-OMcp49_320*ORcp49_221-
 OMcp49_321*ORcp49_229-OMcp49_329*ORcp49_230-OMcp49_330*ORcp49_231-OMcp49_331*ORcp49_232-OMcp49_332*ORcp49_285-OMcp49_36*
 ORcp49_219+OPcp49_219*RLcp49_320+OPcp49_220*RLcp49_321+OPcp49_221*RLcp49_329+OPcp49_229*RLcp49_330+OPcp49_230*RLcp49_331+
 OPcp49_231*RLcp49_332+OPcp49_232*RLcp49_385+OPcp49_26*RLcp49_319-OPcp49_319*RLcp49_220-OPcp49_320*RLcp49_221-OPcp49_321*
 RLcp49_229-OPcp49_329*RLcp49_230-OPcp49_330*RLcp49_231-OPcp49_331*RLcp49_232-OPcp49_332*RLcp49_285-OPcp49_36*RLcp49_219;
    ACcp49_285 = qdd[2]-OMcp49_119*ORcp49_320-OMcp49_120*ORcp49_321-OMcp49_121*ORcp49_329-OMcp49_129*ORcp49_330-OMcp49_130
 *ORcp49_331-OMcp49_131*ORcp49_332-OMcp49_132*ORcp49_385-OMcp49_16*ORcp49_319+OMcp49_319*ORcp49_120+OMcp49_320*ORcp49_121+
 OMcp49_321*ORcp49_129+OMcp49_329*ORcp49_130+OMcp49_330*ORcp49_131+OMcp49_331*ORcp49_132+OMcp49_332*ORcp49_185+OMcp49_36*
 ORcp49_119-OPcp49_119*RLcp49_320-OPcp49_120*RLcp49_321-OPcp49_121*RLcp49_329-OPcp49_129*RLcp49_330-OPcp49_130*RLcp49_331-
 OPcp49_131*RLcp49_332-OPcp49_132*RLcp49_385-OPcp49_16*RLcp49_319+OPcp49_319*RLcp49_120+OPcp49_320*RLcp49_121+OPcp49_321*
 RLcp49_129+OPcp49_329*RLcp49_130+OPcp49_330*RLcp49_131+OPcp49_331*RLcp49_132+OPcp49_332*RLcp49_185+OPcp49_36*RLcp49_119;
    ACcp49_385 = qdd[3]+OMcp49_119*ORcp49_220+OMcp49_120*ORcp49_221+OMcp49_121*ORcp49_229+OMcp49_129*ORcp49_230+OMcp49_130
 *ORcp49_231+OMcp49_131*ORcp49_232+OMcp49_132*ORcp49_285+OMcp49_16*ORcp49_219-OMcp49_219*ORcp49_120-OMcp49_220*ORcp49_121-
 OMcp49_221*ORcp49_129-OMcp49_229*ORcp49_130-OMcp49_230*ORcp49_131-OMcp49_231*ORcp49_132-OMcp49_232*ORcp49_185-OMcp49_26*
 ORcp49_119+OPcp49_119*RLcp49_220+OPcp49_120*RLcp49_221+OPcp49_121*RLcp49_229+OPcp49_129*RLcp49_230+OPcp49_130*RLcp49_231+
 OPcp49_131*RLcp49_232+OPcp49_132*RLcp49_285+OPcp49_16*RLcp49_219-OPcp49_219*RLcp49_120-OPcp49_220*RLcp49_121-OPcp49_221*
 RLcp49_129-OPcp49_229*RLcp49_130-OPcp49_230*RLcp49_131-OPcp49_231*RLcp49_132-OPcp49_232*RLcp49_185-OPcp49_26*RLcp49_119;

// = = Block_1_0_0_50_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp49_185;
    sens->P[2] = POcp49_285;
    sens->P[3] = POcp49_385;
    sens->R[1][1] = ROcp49_132;
    sens->R[1][2] = ROcp49_232;
    sens->R[1][3] = ROcp49_332;
    sens->R[2][1] = ROcp49_432;
    sens->R[2][2] = ROcp49_532;
    sens->R[2][3] = ROcp49_632;
    sens->R[3][1] = ROcp49_731;
    sens->R[3][2] = ROcp49_831;
    sens->R[3][3] = ROcp49_931;
    sens->V[1] = VIcp49_185;
    sens->V[2] = VIcp49_285;
    sens->V[3] = VIcp49_385;
    sens->OM[1] = OMcp49_132;
    sens->OM[2] = OMcp49_232;
    sens->OM[3] = OMcp49_332;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp49_185_5;
    sens->J[1][6] = JTcp49_185_6;
    sens->J[1][19] = JTcp49_185_7;
    sens->J[1][20] = JTcp49_185_8;
    sens->J[1][21] = JTcp49_185_9;
    sens->J[1][29] = JTcp49_185_10;
    sens->J[1][30] = JTcp49_185_11;
    sens->J[1][31] = JTcp49_185_12;
    sens->J[1][32] = JTcp49_185_13;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp49_285_4;
    sens->J[2][5] = JTcp49_285_5;
    sens->J[2][6] = JTcp49_285_6;
    sens->J[2][19] = JTcp49_285_7;
    sens->J[2][20] = JTcp49_285_8;
    sens->J[2][21] = JTcp49_285_9;
    sens->J[2][29] = JTcp49_285_10;
    sens->J[2][30] = JTcp49_285_11;
    sens->J[2][31] = JTcp49_285_12;
    sens->J[2][32] = JTcp49_285_13;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp49_385_4;
    sens->J[3][5] = JTcp49_385_5;
    sens->J[3][6] = JTcp49_385_6;
    sens->J[3][19] = JTcp49_385_7;
    sens->J[3][20] = JTcp49_385_8;
    sens->J[3][21] = JTcp49_385_9;
    sens->J[3][29] = JTcp49_385_10;
    sens->J[3][30] = JTcp49_385_11;
    sens->J[3][31] = JTcp49_385_12;
    sens->J[3][32] = JTcp49_385_13;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp49_16;
    sens->J[4][20] = ROcp49_419;
    sens->J[4][21] = ROcp49_720;
    sens->J[4][29] = ROcp49_421;
    sens->J[4][30] = ROcp49_129;
    sens->J[4][31] = ROcp49_430;
    sens->J[4][32] = ROcp49_731;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp49_85;
    sens->J[5][19] = ROcp49_26;
    sens->J[5][20] = ROcp49_519;
    sens->J[5][21] = ROcp49_820;
    sens->J[5][29] = ROcp49_521;
    sens->J[5][30] = ROcp49_229;
    sens->J[5][31] = ROcp49_530;
    sens->J[5][32] = ROcp49_831;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp49_95;
    sens->J[6][19] = ROcp49_36;
    sens->J[6][20] = ROcp49_619;
    sens->J[6][21] = ROcp49_920;
    sens->J[6][29] = ROcp49_621;
    sens->J[6][30] = ROcp49_329;
    sens->J[6][31] = ROcp49_630;
    sens->J[6][32] = ROcp49_931;
    sens->A[1] = ACcp49_185;
    sens->A[2] = ACcp49_285;
    sens->A[3] = ACcp49_385;
    sens->OMP[1] = OPcp49_132;
    sens->OMP[2] = OPcp49_232;
    sens->OMP[3] = OPcp49_332;
 
// 
break;
case 51:
 


// = = Block_1_0_0_51_0_1 = = 
 
// Sensor Kinematics 


    ROcp50_25 = S4*S5;
    ROcp50_35 = -C4*S5;
    ROcp50_85 = -S4*C5;
    ROcp50_95 = C4*C5;
    ROcp50_16 = C5*C6;
    ROcp50_26 = ROcp50_25*C6+C4*S6;
    ROcp50_36 = ROcp50_35*C6+S4*S6;
    ROcp50_46 = -C5*S6;
    ROcp50_56 = -(ROcp50_25*S6-C4*C6);
    ROcp50_66 = -(ROcp50_35*S6-S4*C6);
    OMcp50_25 = qd[5]*C4;
    OMcp50_35 = qd[5]*S4;
    OMcp50_16 = qd[4]+qd[6]*S5;
    OMcp50_26 = OMcp50_25+ROcp50_85*qd[6];
    OMcp50_36 = OMcp50_35+ROcp50_95*qd[6];
    OPcp50_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp50_26 = ROcp50_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp50_35*S5-ROcp50_95*qd[4]);
    OPcp50_36 = ROcp50_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp50_25*S5-ROcp50_85*qd[4]);

// = = Block_1_0_0_51_0_4 = = 
 
// Sensor Kinematics 


    ROcp50_419 = ROcp50_46*C19+S19*S5;
    ROcp50_519 = ROcp50_56*C19+ROcp50_85*S19;
    ROcp50_619 = ROcp50_66*C19+ROcp50_95*S19;
    ROcp50_719 = -(ROcp50_46*S19-C19*S5);
    ROcp50_819 = -(ROcp50_56*S19-ROcp50_85*C19);
    ROcp50_919 = -(ROcp50_66*S19-ROcp50_95*C19);
    ROcp50_120 = ROcp50_16*C20-ROcp50_719*S20;
    ROcp50_220 = ROcp50_26*C20-ROcp50_819*S20;
    ROcp50_320 = ROcp50_36*C20-ROcp50_919*S20;
    ROcp50_720 = ROcp50_16*S20+ROcp50_719*C20;
    ROcp50_820 = ROcp50_26*S20+ROcp50_819*C20;
    ROcp50_920 = ROcp50_36*S20+ROcp50_919*C20;
    ROcp50_121 = ROcp50_120*C21+ROcp50_419*S21;
    ROcp50_221 = ROcp50_220*C21+ROcp50_519*S21;
    ROcp50_321 = ROcp50_320*C21+ROcp50_619*S21;
    ROcp50_421 = -(ROcp50_120*S21-ROcp50_419*C21);
    ROcp50_521 = -(ROcp50_220*S21-ROcp50_519*C21);
    ROcp50_621 = -(ROcp50_320*S21-ROcp50_619*C21);
    RLcp50_119 = s->dpt[2][3]*ROcp50_46+ROcp50_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp50_219 = s->dpt[2][3]*ROcp50_56+ROcp50_26*s->dpt[1][3]+ROcp50_85*s->dpt[3][3];
    RLcp50_319 = s->dpt[2][3]*ROcp50_66+ROcp50_36*s->dpt[1][3]+ROcp50_95*s->dpt[3][3];
    OMcp50_119 = OMcp50_16+ROcp50_16*qd[19];
    OMcp50_219 = OMcp50_26+ROcp50_26*qd[19];
    OMcp50_319 = OMcp50_36+ROcp50_36*qd[19];
    ORcp50_119 = OMcp50_26*RLcp50_319-OMcp50_36*RLcp50_219;
    ORcp50_219 = -(OMcp50_16*RLcp50_319-OMcp50_36*RLcp50_119);
    ORcp50_319 = OMcp50_16*RLcp50_219-OMcp50_26*RLcp50_119;
    OPcp50_119 = OPcp50_16+ROcp50_16*qdd[19]+qd[19]*(OMcp50_26*ROcp50_36-OMcp50_36*ROcp50_26);
    OPcp50_219 = OPcp50_26+ROcp50_26*qdd[19]-qd[19]*(OMcp50_16*ROcp50_36-OMcp50_36*ROcp50_16);
    OPcp50_319 = OPcp50_36+ROcp50_36*qdd[19]+qd[19]*(OMcp50_16*ROcp50_26-OMcp50_26*ROcp50_16);
    RLcp50_120 = s->dpt[1][38]*ROcp50_16+s->dpt[2][38]*ROcp50_419+s->dpt[3][38]*ROcp50_719;
    RLcp50_220 = s->dpt[1][38]*ROcp50_26+s->dpt[2][38]*ROcp50_519+s->dpt[3][38]*ROcp50_819;
    RLcp50_320 = s->dpt[1][38]*ROcp50_36+s->dpt[2][38]*ROcp50_619+s->dpt[3][38]*ROcp50_919;
    OMcp50_120 = OMcp50_119+ROcp50_419*qd[20];
    OMcp50_220 = OMcp50_219+ROcp50_519*qd[20];
    OMcp50_320 = OMcp50_319+ROcp50_619*qd[20];
    ORcp50_120 = OMcp50_219*RLcp50_320-OMcp50_319*RLcp50_220;
    ORcp50_220 = -(OMcp50_119*RLcp50_320-OMcp50_319*RLcp50_120);
    ORcp50_320 = OMcp50_119*RLcp50_220-OMcp50_219*RLcp50_120;
    OPcp50_120 = OPcp50_119+ROcp50_419*qdd[20]+qd[20]*(OMcp50_219*ROcp50_619-OMcp50_319*ROcp50_519);
    OPcp50_220 = OPcp50_219+ROcp50_519*qdd[20]-qd[20]*(OMcp50_119*ROcp50_619-OMcp50_319*ROcp50_419);
    OPcp50_320 = OPcp50_319+ROcp50_619*qdd[20]+qd[20]*(OMcp50_119*ROcp50_519-OMcp50_219*ROcp50_419);
    RLcp50_121 = s->dpt[1][40]*ROcp50_120+s->dpt[2][40]*ROcp50_419+ROcp50_720*s->dpt[3][40];
    RLcp50_221 = s->dpt[1][40]*ROcp50_220+s->dpt[2][40]*ROcp50_519+ROcp50_820*s->dpt[3][40];
    RLcp50_321 = s->dpt[1][40]*ROcp50_320+s->dpt[2][40]*ROcp50_619+ROcp50_920*s->dpt[3][40];
    OMcp50_121 = OMcp50_120+ROcp50_720*qd[21];
    OMcp50_221 = OMcp50_220+ROcp50_820*qd[21];
    OMcp50_321 = OMcp50_320+ROcp50_920*qd[21];
    ORcp50_121 = OMcp50_220*RLcp50_321-OMcp50_320*RLcp50_221;
    ORcp50_221 = -(OMcp50_120*RLcp50_321-OMcp50_320*RLcp50_121);
    ORcp50_321 = OMcp50_120*RLcp50_221-OMcp50_220*RLcp50_121;
    OPcp50_121 = OPcp50_120+ROcp50_720*qdd[21]+qd[21]*(OMcp50_220*ROcp50_920-OMcp50_320*ROcp50_820);
    OPcp50_221 = OPcp50_220+ROcp50_820*qdd[21]-qd[21]*(OMcp50_120*ROcp50_920-OMcp50_320*ROcp50_720);
    OPcp50_321 = OPcp50_320+ROcp50_920*qdd[21]+qd[21]*(OMcp50_120*ROcp50_820-OMcp50_220*ROcp50_720);

// = = Block_1_0_0_51_0_6 = = 
 
// Sensor Kinematics 


    ROcp50_129 = ROcp50_121*C29-ROcp50_720*S29;
    ROcp50_229 = ROcp50_221*C29-ROcp50_820*S29;
    ROcp50_329 = ROcp50_321*C29-ROcp50_920*S29;
    ROcp50_729 = ROcp50_121*S29+ROcp50_720*C29;
    ROcp50_829 = ROcp50_221*S29+ROcp50_820*C29;
    ROcp50_929 = ROcp50_321*S29+ROcp50_920*C29;
    ROcp50_430 = ROcp50_421*C30+ROcp50_729*S30;
    ROcp50_530 = ROcp50_521*C30+ROcp50_829*S30;
    ROcp50_630 = ROcp50_621*C30+ROcp50_929*S30;
    ROcp50_730 = -(ROcp50_421*S30-ROcp50_729*C30);
    ROcp50_830 = -(ROcp50_521*S30-ROcp50_829*C30);
    ROcp50_930 = -(ROcp50_621*S30-ROcp50_929*C30);
    ROcp50_131 = ROcp50_129*C31-ROcp50_730*S31;
    ROcp50_231 = ROcp50_229*C31-ROcp50_830*S31;
    ROcp50_331 = ROcp50_329*C31-ROcp50_930*S31;
    ROcp50_731 = ROcp50_129*S31+ROcp50_730*C31;
    ROcp50_831 = ROcp50_229*S31+ROcp50_830*C31;
    ROcp50_931 = ROcp50_329*S31+ROcp50_930*C31;
    ROcp50_132 = ROcp50_131*C32+ROcp50_430*S32;
    ROcp50_232 = ROcp50_231*C32+ROcp50_530*S32;
    ROcp50_332 = ROcp50_331*C32+ROcp50_630*S32;
    ROcp50_432 = -(ROcp50_131*S32-ROcp50_430*C32);
    ROcp50_532 = -(ROcp50_231*S32-ROcp50_530*C32);
    ROcp50_632 = -(ROcp50_331*S32-ROcp50_630*C32);
    ROcp50_133 = ROcp50_132*C33-ROcp50_731*S33;
    ROcp50_233 = ROcp50_232*C33-ROcp50_831*S33;
    ROcp50_333 = ROcp50_332*C33-ROcp50_931*S33;
    ROcp50_733 = ROcp50_132*S33+ROcp50_731*C33;
    ROcp50_833 = ROcp50_232*S33+ROcp50_831*C33;
    ROcp50_933 = ROcp50_332*S33+ROcp50_931*C33;
    ROcp50_134 = ROcp50_133*C34+ROcp50_432*S34;
    ROcp50_234 = ROcp50_233*C34+ROcp50_532*S34;
    ROcp50_334 = ROcp50_333*C34+ROcp50_632*S34;
    ROcp50_434 = -(ROcp50_133*S34-ROcp50_432*C34);
    ROcp50_534 = -(ROcp50_233*S34-ROcp50_532*C34);
    ROcp50_634 = -(ROcp50_333*S34-ROcp50_632*C34);
    ROcp50_435 = ROcp50_434*C35+ROcp50_733*S35;
    ROcp50_535 = ROcp50_534*C35+ROcp50_833*S35;
    ROcp50_635 = ROcp50_634*C35+ROcp50_933*S35;
    ROcp50_735 = -(ROcp50_434*S35-ROcp50_733*C35);
    ROcp50_835 = -(ROcp50_534*S35-ROcp50_833*C35);
    ROcp50_935 = -(ROcp50_634*S35-ROcp50_933*C35);
    RLcp50_129 = ROcp50_121*s->dpt[1][45]+ROcp50_421*s->dpt[2][45]+ROcp50_720*s->dpt[3][45];
    RLcp50_229 = ROcp50_221*s->dpt[1][45]+ROcp50_521*s->dpt[2][45]+ROcp50_820*s->dpt[3][45];
    RLcp50_329 = ROcp50_321*s->dpt[1][45]+ROcp50_621*s->dpt[2][45]+ROcp50_920*s->dpt[3][45];
    OMcp50_129 = OMcp50_121+ROcp50_421*qd[29];
    OMcp50_229 = OMcp50_221+ROcp50_521*qd[29];
    OMcp50_329 = OMcp50_321+ROcp50_621*qd[29];
    ORcp50_129 = OMcp50_221*RLcp50_329-OMcp50_321*RLcp50_229;
    ORcp50_229 = -(OMcp50_121*RLcp50_329-OMcp50_321*RLcp50_129);
    ORcp50_329 = OMcp50_121*RLcp50_229-OMcp50_221*RLcp50_129;
    OPcp50_129 = OPcp50_121+ROcp50_421*qdd[29]+qd[29]*(OMcp50_221*ROcp50_621-OMcp50_321*ROcp50_521);
    OPcp50_229 = OPcp50_221+ROcp50_521*qdd[29]-qd[29]*(OMcp50_121*ROcp50_621-OMcp50_321*ROcp50_421);
    OPcp50_329 = OPcp50_321+ROcp50_621*qdd[29]+qd[29]*(OMcp50_121*ROcp50_521-OMcp50_221*ROcp50_421);
    RLcp50_130 = s->dpt[1][59]*ROcp50_129+s->dpt[3][59]*ROcp50_729+ROcp50_421*s->dpt[2][59];
    RLcp50_230 = s->dpt[1][59]*ROcp50_229+s->dpt[3][59]*ROcp50_829+ROcp50_521*s->dpt[2][59];
    RLcp50_330 = s->dpt[1][59]*ROcp50_329+s->dpt[3][59]*ROcp50_929+ROcp50_621*s->dpt[2][59];
    OMcp50_130 = OMcp50_129+ROcp50_129*qd[30];
    OMcp50_230 = OMcp50_229+ROcp50_229*qd[30];
    OMcp50_330 = OMcp50_329+ROcp50_329*qd[30];
    ORcp50_130 = OMcp50_229*RLcp50_330-OMcp50_329*RLcp50_230;
    ORcp50_230 = -(OMcp50_129*RLcp50_330-OMcp50_329*RLcp50_130);
    ORcp50_330 = OMcp50_129*RLcp50_230-OMcp50_229*RLcp50_130;
    OPcp50_130 = OPcp50_129+ROcp50_129*qdd[30]+qd[30]*(OMcp50_229*ROcp50_329-OMcp50_329*ROcp50_229);
    OPcp50_230 = OPcp50_229+ROcp50_229*qdd[30]-qd[30]*(OMcp50_129*ROcp50_329-OMcp50_329*ROcp50_129);
    OPcp50_330 = OPcp50_329+ROcp50_329*qdd[30]+qd[30]*(OMcp50_129*ROcp50_229-OMcp50_229*ROcp50_129);
    RLcp50_131 = s->dpt[1][61]*ROcp50_129+s->dpt[3][61]*ROcp50_730+ROcp50_430*s->dpt[2][61];
    RLcp50_231 = s->dpt[1][61]*ROcp50_229+s->dpt[3][61]*ROcp50_830+ROcp50_530*s->dpt[2][61];
    RLcp50_331 = s->dpt[1][61]*ROcp50_329+s->dpt[3][61]*ROcp50_930+ROcp50_630*s->dpt[2][61];
    OMcp50_131 = OMcp50_130+ROcp50_430*qd[31];
    OMcp50_231 = OMcp50_230+ROcp50_530*qd[31];
    OMcp50_331 = OMcp50_330+ROcp50_630*qd[31];
    ORcp50_131 = OMcp50_230*RLcp50_331-OMcp50_330*RLcp50_231;
    ORcp50_231 = -(OMcp50_130*RLcp50_331-OMcp50_330*RLcp50_131);
    ORcp50_331 = OMcp50_130*RLcp50_231-OMcp50_230*RLcp50_131;
    OPcp50_131 = OPcp50_130+ROcp50_430*qdd[31]+qd[31]*(OMcp50_230*ROcp50_630-OMcp50_330*ROcp50_530);
    OPcp50_231 = OPcp50_230+ROcp50_530*qdd[31]-qd[31]*(OMcp50_130*ROcp50_630-OMcp50_330*ROcp50_430);
    OPcp50_331 = OPcp50_330+ROcp50_630*qdd[31]+qd[31]*(OMcp50_130*ROcp50_530-OMcp50_230*ROcp50_430);
    RLcp50_132 = s->dpt[3][63]*ROcp50_731+ROcp50_131*s->dpt[1][63]+ROcp50_430*s->dpt[2][63];
    RLcp50_232 = s->dpt[3][63]*ROcp50_831+ROcp50_231*s->dpt[1][63]+ROcp50_530*s->dpt[2][63];
    RLcp50_332 = s->dpt[3][63]*ROcp50_931+ROcp50_331*s->dpt[1][63]+ROcp50_630*s->dpt[2][63];
    OMcp50_132 = OMcp50_131+ROcp50_731*qd[32];
    OMcp50_232 = OMcp50_231+ROcp50_831*qd[32];
    OMcp50_332 = OMcp50_331+ROcp50_931*qd[32];
    ORcp50_132 = OMcp50_231*RLcp50_332-OMcp50_331*RLcp50_232;
    ORcp50_232 = -(OMcp50_131*RLcp50_332-OMcp50_331*RLcp50_132);
    ORcp50_332 = OMcp50_131*RLcp50_232-OMcp50_231*RLcp50_132;
    OPcp50_132 = OPcp50_131+ROcp50_731*qdd[32]+qd[32]*(OMcp50_231*ROcp50_931-OMcp50_331*ROcp50_831);
    OPcp50_232 = OPcp50_231+ROcp50_831*qdd[32]-qd[32]*(OMcp50_131*ROcp50_931-OMcp50_331*ROcp50_731);
    OPcp50_332 = OPcp50_331+ROcp50_931*qdd[32]+qd[32]*(OMcp50_131*ROcp50_831-OMcp50_231*ROcp50_731);
    RLcp50_133 = s->dpt[2][65]*ROcp50_432+s->dpt[3][65]*ROcp50_731+ROcp50_132*s->dpt[1][65];
    RLcp50_233 = s->dpt[2][65]*ROcp50_532+s->dpt[3][65]*ROcp50_831+ROcp50_232*s->dpt[1][65];
    RLcp50_333 = s->dpt[2][65]*ROcp50_632+s->dpt[3][65]*ROcp50_931+ROcp50_332*s->dpt[1][65];
    OMcp50_133 = OMcp50_132+ROcp50_432*qd[33];
    OMcp50_233 = OMcp50_232+ROcp50_532*qd[33];
    OMcp50_333 = OMcp50_332+ROcp50_632*qd[33];
    ORcp50_133 = OMcp50_232*RLcp50_333-OMcp50_332*RLcp50_233;
    ORcp50_233 = -(OMcp50_132*RLcp50_333-OMcp50_332*RLcp50_133);
    ORcp50_333 = OMcp50_132*RLcp50_233-OMcp50_232*RLcp50_133;
    OPcp50_133 = OPcp50_132+ROcp50_432*qdd[33]+qd[33]*(OMcp50_232*ROcp50_632-OMcp50_332*ROcp50_532);
    OPcp50_233 = OPcp50_232+ROcp50_532*qdd[33]-qd[33]*(OMcp50_132*ROcp50_632-OMcp50_332*ROcp50_432);
    OPcp50_333 = OPcp50_332+ROcp50_632*qdd[33]+qd[33]*(OMcp50_132*ROcp50_532-OMcp50_232*ROcp50_432);
    RLcp50_134 = s->dpt[1][66]*ROcp50_133+s->dpt[3][66]*ROcp50_733+ROcp50_432*s->dpt[2][66];
    RLcp50_234 = s->dpt[1][66]*ROcp50_233+s->dpt[3][66]*ROcp50_833+ROcp50_532*s->dpt[2][66];
    RLcp50_334 = s->dpt[1][66]*ROcp50_333+s->dpt[3][66]*ROcp50_933+ROcp50_632*s->dpt[2][66];
    OMcp50_134 = OMcp50_133+ROcp50_733*qd[34];
    OMcp50_234 = OMcp50_233+ROcp50_833*qd[34];
    OMcp50_334 = OMcp50_333+ROcp50_933*qd[34];
    ORcp50_134 = OMcp50_233*RLcp50_334-OMcp50_333*RLcp50_234;
    ORcp50_234 = -(OMcp50_133*RLcp50_334-OMcp50_333*RLcp50_134);
    ORcp50_334 = OMcp50_133*RLcp50_234-OMcp50_233*RLcp50_134;
    OPcp50_134 = OPcp50_133+ROcp50_733*qdd[34]+qd[34]*(OMcp50_233*ROcp50_933-OMcp50_333*ROcp50_833);
    OPcp50_234 = OPcp50_233+ROcp50_833*qdd[34]-qd[34]*(OMcp50_133*ROcp50_933-OMcp50_333*ROcp50_733);
    OPcp50_334 = OPcp50_333+ROcp50_933*qdd[34]+qd[34]*(OMcp50_133*ROcp50_833-OMcp50_233*ROcp50_733);
    RLcp50_135 = s->dpt[1][67]*ROcp50_134+s->dpt[2][67]*ROcp50_434+s->dpt[3][67]*ROcp50_733;
    RLcp50_235 = s->dpt[1][67]*ROcp50_234+s->dpt[2][67]*ROcp50_534+s->dpt[3][67]*ROcp50_833;
    RLcp50_335 = s->dpt[1][67]*ROcp50_334+s->dpt[2][67]*ROcp50_634+s->dpt[3][67]*ROcp50_933;
    OMcp50_135 = OMcp50_134+ROcp50_134*qd[35];
    OMcp50_235 = OMcp50_234+ROcp50_234*qd[35];
    OMcp50_335 = OMcp50_334+ROcp50_334*qd[35];
    ORcp50_135 = OMcp50_234*RLcp50_335-OMcp50_334*RLcp50_235;
    ORcp50_235 = -(OMcp50_134*RLcp50_335-OMcp50_334*RLcp50_135);
    ORcp50_335 = OMcp50_134*RLcp50_235-OMcp50_234*RLcp50_135;
    OPcp50_135 = OPcp50_134+ROcp50_134*qdd[35]+qd[35]*(OMcp50_234*ROcp50_334-OMcp50_334*ROcp50_234);
    OPcp50_235 = OPcp50_234+ROcp50_234*qdd[35]-qd[35]*(OMcp50_134*ROcp50_334-OMcp50_334*ROcp50_134);
    OPcp50_335 = OPcp50_334+ROcp50_334*qdd[35]+qd[35]*(OMcp50_134*ROcp50_234-OMcp50_234*ROcp50_134);
    RLcp50_186 = s->dpt[1][68]*ROcp50_134+s->dpt[3][68]*ROcp50_735+ROcp50_435*s->dpt[2][68];
    RLcp50_286 = s->dpt[1][68]*ROcp50_234+s->dpt[3][68]*ROcp50_835+ROcp50_535*s->dpt[2][68];
    RLcp50_386 = s->dpt[1][68]*ROcp50_334+s->dpt[3][68]*ROcp50_935+ROcp50_635*s->dpt[2][68];
    POcp50_186 = RLcp50_119+RLcp50_120+RLcp50_121+RLcp50_129+RLcp50_130+RLcp50_131+RLcp50_132+RLcp50_133+RLcp50_134+
 RLcp50_135+RLcp50_186+q[1];
    POcp50_286 = RLcp50_219+RLcp50_220+RLcp50_221+RLcp50_229+RLcp50_230+RLcp50_231+RLcp50_232+RLcp50_233+RLcp50_234+
 RLcp50_235+RLcp50_286+q[2];
    POcp50_386 = RLcp50_319+RLcp50_320+RLcp50_321+RLcp50_329+RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+
 RLcp50_335+RLcp50_386+q[3];
    JTcp50_286_4 = -(RLcp50_319+RLcp50_320+RLcp50_321+RLcp50_329+RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+
 RLcp50_335+RLcp50_386);
    JTcp50_386_4 = RLcp50_219+RLcp50_220+RLcp50_221+RLcp50_229+RLcp50_230+RLcp50_231+RLcp50_232+RLcp50_233+RLcp50_234+
 RLcp50_235+RLcp50_286;
    JTcp50_186_5 = C4*(RLcp50_319+RLcp50_320+RLcp50_321+RLcp50_329+RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+
 RLcp50_335)-S4*(RLcp50_219+RLcp50_220)-S4*(RLcp50_221+RLcp50_229)-S4*(RLcp50_230+RLcp50_231)-S4*(RLcp50_232+RLcp50_233)-S4*(
 RLcp50_234+RLcp50_235)-RLcp50_286*S4+RLcp50_386*C4;
    JTcp50_286_5 = S4*(RLcp50_119+RLcp50_120+RLcp50_121+RLcp50_129+RLcp50_130+RLcp50_131+RLcp50_132+RLcp50_133+RLcp50_134+
 RLcp50_135+RLcp50_186);
    JTcp50_386_5 = -C4*(RLcp50_119+RLcp50_120+RLcp50_121+RLcp50_129+RLcp50_130+RLcp50_131+RLcp50_132+RLcp50_133+RLcp50_134
 +RLcp50_135+RLcp50_186);
    JTcp50_186_6 = ROcp50_85*(RLcp50_319+RLcp50_320+RLcp50_321+RLcp50_329+RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+
 RLcp50_334+RLcp50_335)-ROcp50_95*(RLcp50_219+RLcp50_220)-ROcp50_95*(RLcp50_221+RLcp50_229)-ROcp50_95*(RLcp50_230+RLcp50_231)
 -ROcp50_95*(RLcp50_232+RLcp50_233)-ROcp50_95*(RLcp50_234+RLcp50_235)-RLcp50_286*ROcp50_95+RLcp50_386*ROcp50_85;
    JTcp50_286_6 = -(RLcp50_386*S5-ROcp50_95*(RLcp50_119+RLcp50_120+RLcp50_121+RLcp50_129+RLcp50_130+RLcp50_131+RLcp50_132
 +RLcp50_133+RLcp50_134+RLcp50_135+RLcp50_186)+S5*(RLcp50_319+RLcp50_320)+S5*(RLcp50_321+RLcp50_329)+S5*(RLcp50_330+
 RLcp50_331)+S5*(RLcp50_332+RLcp50_333)+S5*(RLcp50_334+RLcp50_335));
    JTcp50_386_6 = RLcp50_286*S5-ROcp50_85*(RLcp50_119+RLcp50_120+RLcp50_121+RLcp50_129+RLcp50_130+RLcp50_131+RLcp50_132+
 RLcp50_133+RLcp50_134+RLcp50_135+RLcp50_186)+S5*(RLcp50_219+RLcp50_220)+S5*(RLcp50_221+RLcp50_229)+S5*(RLcp50_230+RLcp50_231
 )+S5*(RLcp50_232+RLcp50_233)+S5*(RLcp50_234+RLcp50_235);
    JTcp50_186_7 = ROcp50_26*(RLcp50_320+RLcp50_321+RLcp50_329+RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+
 RLcp50_335+RLcp50_386)-ROcp50_36*(RLcp50_220+RLcp50_221)-ROcp50_36*(RLcp50_229+RLcp50_230)-ROcp50_36*(RLcp50_231+RLcp50_232)
 -ROcp50_36*(RLcp50_233+RLcp50_234)-ROcp50_36*(RLcp50_235+RLcp50_286);
    JTcp50_286_7 = -(ROcp50_16*(RLcp50_320+RLcp50_321+RLcp50_329+RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+
 RLcp50_335+RLcp50_386)-ROcp50_36*(RLcp50_120+RLcp50_121)-ROcp50_36*(RLcp50_129+RLcp50_130)-ROcp50_36*(RLcp50_131+RLcp50_132)
 -ROcp50_36*(RLcp50_133+RLcp50_134)-ROcp50_36*(RLcp50_135+RLcp50_186));
    JTcp50_386_7 = ROcp50_16*(RLcp50_220+RLcp50_221+RLcp50_229+RLcp50_230+RLcp50_231+RLcp50_232+RLcp50_233+RLcp50_234+
 RLcp50_235+RLcp50_286)-ROcp50_26*(RLcp50_120+RLcp50_121)-ROcp50_26*(RLcp50_129+RLcp50_130)-ROcp50_26*(RLcp50_131+RLcp50_132)
 -ROcp50_26*(RLcp50_133+RLcp50_134)-ROcp50_26*(RLcp50_135+RLcp50_186);
    JTcp50_186_8 = ROcp50_519*(RLcp50_321+RLcp50_329+RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+RLcp50_335)-
 ROcp50_619*(RLcp50_221+RLcp50_229)-ROcp50_619*(RLcp50_230+RLcp50_231)-ROcp50_619*(RLcp50_232+RLcp50_233)-ROcp50_619*(
 RLcp50_234+RLcp50_235)-RLcp50_286*ROcp50_619+RLcp50_386*ROcp50_519;
    JTcp50_286_8 = RLcp50_186*ROcp50_619-RLcp50_386*ROcp50_419-ROcp50_419*(RLcp50_321+RLcp50_329+RLcp50_330+RLcp50_331+
 RLcp50_332+RLcp50_333+RLcp50_334+RLcp50_335)+ROcp50_619*(RLcp50_121+RLcp50_129)+ROcp50_619*(RLcp50_130+RLcp50_131)+
 ROcp50_619*(RLcp50_132+RLcp50_133)+ROcp50_619*(RLcp50_134+RLcp50_135);
    JTcp50_386_8 = ROcp50_419*(RLcp50_221+RLcp50_229+RLcp50_230+RLcp50_231+RLcp50_232+RLcp50_233+RLcp50_234+RLcp50_235)-
 ROcp50_519*(RLcp50_121+RLcp50_129)-ROcp50_519*(RLcp50_130+RLcp50_131)-ROcp50_519*(RLcp50_132+RLcp50_133)-ROcp50_519*(
 RLcp50_134+RLcp50_135)-RLcp50_186*ROcp50_519+RLcp50_286*ROcp50_419;
    JTcp50_186_9 = ROcp50_820*(RLcp50_329+RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+RLcp50_335+RLcp50_386)-
 ROcp50_920*(RLcp50_229+RLcp50_230)-ROcp50_920*(RLcp50_231+RLcp50_232)-ROcp50_920*(RLcp50_233+RLcp50_234)-ROcp50_920*(
 RLcp50_235+RLcp50_286);
    JTcp50_286_9 = -(ROcp50_720*(RLcp50_329+RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+RLcp50_335+RLcp50_386)-
 ROcp50_920*(RLcp50_129+RLcp50_130)-ROcp50_920*(RLcp50_131+RLcp50_132)-ROcp50_920*(RLcp50_133+RLcp50_134)-ROcp50_920*(
 RLcp50_135+RLcp50_186));
    JTcp50_386_9 = ROcp50_720*(RLcp50_229+RLcp50_230+RLcp50_231+RLcp50_232+RLcp50_233+RLcp50_234+RLcp50_235+RLcp50_286)-
 ROcp50_820*(RLcp50_129+RLcp50_130)-ROcp50_820*(RLcp50_131+RLcp50_132)-ROcp50_820*(RLcp50_133+RLcp50_134)-ROcp50_820*(
 RLcp50_135+RLcp50_186);
    JTcp50_186_10 = ROcp50_521*(RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+RLcp50_335)-ROcp50_621*(RLcp50_230+
 RLcp50_231)-ROcp50_621*(RLcp50_232+RLcp50_233)-ROcp50_621*(RLcp50_234+RLcp50_235)-RLcp50_286*ROcp50_621+RLcp50_386*
 ROcp50_521;
    JTcp50_286_10 = RLcp50_186*ROcp50_621-RLcp50_386*ROcp50_421-ROcp50_421*(RLcp50_330+RLcp50_331+RLcp50_332+RLcp50_333+
 RLcp50_334+RLcp50_335)+ROcp50_621*(RLcp50_130+RLcp50_131)+ROcp50_621*(RLcp50_132+RLcp50_133)+ROcp50_621*(RLcp50_134+
 RLcp50_135);
    JTcp50_386_10 = ROcp50_421*(RLcp50_230+RLcp50_231+RLcp50_232+RLcp50_233+RLcp50_234+RLcp50_235)-ROcp50_521*(RLcp50_130+
 RLcp50_131)-ROcp50_521*(RLcp50_132+RLcp50_133)-ROcp50_521*(RLcp50_134+RLcp50_135)-RLcp50_186*ROcp50_521+RLcp50_286*
 ROcp50_421;
    JTcp50_186_11 = ROcp50_229*(RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+RLcp50_335+RLcp50_386)-ROcp50_329*(RLcp50_231+
 RLcp50_232)-ROcp50_329*(RLcp50_233+RLcp50_234)-ROcp50_329*(RLcp50_235+RLcp50_286);
    JTcp50_286_11 = -(ROcp50_129*(RLcp50_331+RLcp50_332+RLcp50_333+RLcp50_334+RLcp50_335+RLcp50_386)-ROcp50_329*(
 RLcp50_131+RLcp50_132)-ROcp50_329*(RLcp50_133+RLcp50_134)-ROcp50_329*(RLcp50_135+RLcp50_186));
    JTcp50_386_11 = ROcp50_129*(RLcp50_231+RLcp50_232+RLcp50_233+RLcp50_234+RLcp50_235+RLcp50_286)-ROcp50_229*(RLcp50_131+
 RLcp50_132)-ROcp50_229*(RLcp50_133+RLcp50_134)-ROcp50_229*(RLcp50_135+RLcp50_186);
    JTcp50_186_12 = ROcp50_530*(RLcp50_332+RLcp50_333+RLcp50_334+RLcp50_335)-ROcp50_630*(RLcp50_232+RLcp50_233)-ROcp50_630
 *(RLcp50_234+RLcp50_235)-RLcp50_286*ROcp50_630+RLcp50_386*ROcp50_530;
    JTcp50_286_12 = RLcp50_186*ROcp50_630-RLcp50_386*ROcp50_430-ROcp50_430*(RLcp50_332+RLcp50_333+RLcp50_334+RLcp50_335)+
 ROcp50_630*(RLcp50_132+RLcp50_133)+ROcp50_630*(RLcp50_134+RLcp50_135);
    JTcp50_386_12 = ROcp50_430*(RLcp50_232+RLcp50_233+RLcp50_234+RLcp50_235)-ROcp50_530*(RLcp50_132+RLcp50_133)-ROcp50_530
 *(RLcp50_134+RLcp50_135)-RLcp50_186*ROcp50_530+RLcp50_286*ROcp50_430;
    JTcp50_186_13 = ROcp50_831*(RLcp50_333+RLcp50_334+RLcp50_335+RLcp50_386)-ROcp50_931*(RLcp50_233+RLcp50_234)-ROcp50_931
 *(RLcp50_235+RLcp50_286);
    JTcp50_286_13 = -(ROcp50_731*(RLcp50_333+RLcp50_334+RLcp50_335+RLcp50_386)-ROcp50_931*(RLcp50_133+RLcp50_134)-
 ROcp50_931*(RLcp50_135+RLcp50_186));
    JTcp50_386_13 = ROcp50_731*(RLcp50_233+RLcp50_234+RLcp50_235+RLcp50_286)-ROcp50_831*(RLcp50_133+RLcp50_134)-ROcp50_831
 *(RLcp50_135+RLcp50_186);
    JTcp50_186_14 = ROcp50_532*(RLcp50_334+RLcp50_335)-ROcp50_632*(RLcp50_234+RLcp50_235)-RLcp50_286*ROcp50_632+RLcp50_386
 *ROcp50_532;
    JTcp50_286_14 = RLcp50_186*ROcp50_632-RLcp50_386*ROcp50_432-ROcp50_432*(RLcp50_334+RLcp50_335)+ROcp50_632*(RLcp50_134+
 RLcp50_135);
    JTcp50_386_14 = ROcp50_432*(RLcp50_234+RLcp50_235)-ROcp50_532*(RLcp50_134+RLcp50_135)-RLcp50_186*ROcp50_532+RLcp50_286
 *ROcp50_432;
    JTcp50_186_15 = ROcp50_833*(RLcp50_335+RLcp50_386)-ROcp50_933*(RLcp50_235+RLcp50_286);
    JTcp50_286_15 = -(ROcp50_733*(RLcp50_335+RLcp50_386)-ROcp50_933*(RLcp50_135+RLcp50_186));
    JTcp50_386_15 = ROcp50_733*(RLcp50_235+RLcp50_286)-ROcp50_833*(RLcp50_135+RLcp50_186);
    JTcp50_186_16 = -(RLcp50_286*ROcp50_334-RLcp50_386*ROcp50_234);
    JTcp50_286_16 = RLcp50_186*ROcp50_334-RLcp50_386*ROcp50_134;
    JTcp50_386_16 = -(RLcp50_186*ROcp50_234-RLcp50_286*ROcp50_134);
    ORcp50_186 = OMcp50_235*RLcp50_386-OMcp50_335*RLcp50_286;
    ORcp50_286 = -(OMcp50_135*RLcp50_386-OMcp50_335*RLcp50_186);
    ORcp50_386 = OMcp50_135*RLcp50_286-OMcp50_235*RLcp50_186;
    VIcp50_186 = ORcp50_119+ORcp50_120+ORcp50_121+ORcp50_129+ORcp50_130+ORcp50_131+ORcp50_132+ORcp50_133+ORcp50_134+
 ORcp50_135+ORcp50_186+qd[1];
    VIcp50_286 = ORcp50_219+ORcp50_220+ORcp50_221+ORcp50_229+ORcp50_230+ORcp50_231+ORcp50_232+ORcp50_233+ORcp50_234+
 ORcp50_235+ORcp50_286+qd[2];
    VIcp50_386 = ORcp50_319+ORcp50_320+ORcp50_321+ORcp50_329+ORcp50_330+ORcp50_331+ORcp50_332+ORcp50_333+ORcp50_334+
 ORcp50_335+ORcp50_386+qd[3];
    ACcp50_186 = qdd[1]+OMcp50_219*ORcp50_320+OMcp50_220*ORcp50_321+OMcp50_221*ORcp50_329+OMcp50_229*ORcp50_330+OMcp50_230
 *ORcp50_331+OMcp50_231*ORcp50_332+OMcp50_232*ORcp50_333+OMcp50_233*ORcp50_334+OMcp50_234*ORcp50_335+OMcp50_235*ORcp50_386+
 OMcp50_26*ORcp50_319-OMcp50_319*ORcp50_220-OMcp50_320*ORcp50_221-OMcp50_321*ORcp50_229-OMcp50_329*ORcp50_230-OMcp50_330*
 ORcp50_231-OMcp50_331*ORcp50_232-OMcp50_332*ORcp50_233-OMcp50_333*ORcp50_234-OMcp50_334*ORcp50_235-OMcp50_335*ORcp50_286-
 OMcp50_36*ORcp50_219+OPcp50_219*RLcp50_320+OPcp50_220*RLcp50_321+OPcp50_221*RLcp50_329+OPcp50_229*RLcp50_330+OPcp50_230*
 RLcp50_331+OPcp50_231*RLcp50_332+OPcp50_232*RLcp50_333+OPcp50_233*RLcp50_334+OPcp50_234*RLcp50_335+OPcp50_235*RLcp50_386+
 OPcp50_26*RLcp50_319-OPcp50_319*RLcp50_220-OPcp50_320*RLcp50_221-OPcp50_321*RLcp50_229-OPcp50_329*RLcp50_230-OPcp50_330*
 RLcp50_231-OPcp50_331*RLcp50_232-OPcp50_332*RLcp50_233-OPcp50_333*RLcp50_234-OPcp50_334*RLcp50_235-OPcp50_335*RLcp50_286-
 OPcp50_36*RLcp50_219;
    ACcp50_286 = qdd[2]-OMcp50_119*ORcp50_320-OMcp50_120*ORcp50_321-OMcp50_121*ORcp50_329-OMcp50_129*ORcp50_330-OMcp50_130
 *ORcp50_331-OMcp50_131*ORcp50_332-OMcp50_132*ORcp50_333-OMcp50_133*ORcp50_334-OMcp50_134*ORcp50_335-OMcp50_135*ORcp50_386-
 OMcp50_16*ORcp50_319+OMcp50_319*ORcp50_120+OMcp50_320*ORcp50_121+OMcp50_321*ORcp50_129+OMcp50_329*ORcp50_130+OMcp50_330*
 ORcp50_131+OMcp50_331*ORcp50_132+OMcp50_332*ORcp50_133+OMcp50_333*ORcp50_134+OMcp50_334*ORcp50_135+OMcp50_335*ORcp50_186+
 OMcp50_36*ORcp50_119-OPcp50_119*RLcp50_320-OPcp50_120*RLcp50_321-OPcp50_121*RLcp50_329-OPcp50_129*RLcp50_330-OPcp50_130*
 RLcp50_331-OPcp50_131*RLcp50_332-OPcp50_132*RLcp50_333-OPcp50_133*RLcp50_334-OPcp50_134*RLcp50_335-OPcp50_135*RLcp50_386-
 OPcp50_16*RLcp50_319+OPcp50_319*RLcp50_120+OPcp50_320*RLcp50_121+OPcp50_321*RLcp50_129+OPcp50_329*RLcp50_130+OPcp50_330*
 RLcp50_131+OPcp50_331*RLcp50_132+OPcp50_332*RLcp50_133+OPcp50_333*RLcp50_134+OPcp50_334*RLcp50_135+OPcp50_335*RLcp50_186+
 OPcp50_36*RLcp50_119;
    ACcp50_386 = qdd[3]+OMcp50_119*ORcp50_220+OMcp50_120*ORcp50_221+OMcp50_121*ORcp50_229+OMcp50_129*ORcp50_230+OMcp50_130
 *ORcp50_231+OMcp50_131*ORcp50_232+OMcp50_132*ORcp50_233+OMcp50_133*ORcp50_234+OMcp50_134*ORcp50_235+OMcp50_135*ORcp50_286+
 OMcp50_16*ORcp50_219-OMcp50_219*ORcp50_120-OMcp50_220*ORcp50_121-OMcp50_221*ORcp50_129-OMcp50_229*ORcp50_130-OMcp50_230*
 ORcp50_131-OMcp50_231*ORcp50_132-OMcp50_232*ORcp50_133-OMcp50_233*ORcp50_134-OMcp50_234*ORcp50_135-OMcp50_235*ORcp50_186-
 OMcp50_26*ORcp50_119+OPcp50_119*RLcp50_220+OPcp50_120*RLcp50_221+OPcp50_121*RLcp50_229+OPcp50_129*RLcp50_230+OPcp50_130*
 RLcp50_231+OPcp50_131*RLcp50_232+OPcp50_132*RLcp50_233+OPcp50_133*RLcp50_234+OPcp50_134*RLcp50_235+OPcp50_135*RLcp50_286+
 OPcp50_16*RLcp50_219-OPcp50_219*RLcp50_120-OPcp50_220*RLcp50_121-OPcp50_221*RLcp50_129-OPcp50_229*RLcp50_130-OPcp50_230*
 RLcp50_131-OPcp50_231*RLcp50_132-OPcp50_232*RLcp50_133-OPcp50_233*RLcp50_134-OPcp50_234*RLcp50_135-OPcp50_235*RLcp50_186-
 OPcp50_26*RLcp50_119;

// = = Block_1_0_0_51_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp50_186;
    sens->P[2] = POcp50_286;
    sens->P[3] = POcp50_386;
    sens->R[1][1] = ROcp50_134;
    sens->R[1][2] = ROcp50_234;
    sens->R[1][3] = ROcp50_334;
    sens->R[2][1] = ROcp50_435;
    sens->R[2][2] = ROcp50_535;
    sens->R[2][3] = ROcp50_635;
    sens->R[3][1] = ROcp50_735;
    sens->R[3][2] = ROcp50_835;
    sens->R[3][3] = ROcp50_935;
    sens->V[1] = VIcp50_186;
    sens->V[2] = VIcp50_286;
    sens->V[3] = VIcp50_386;
    sens->OM[1] = OMcp50_135;
    sens->OM[2] = OMcp50_235;
    sens->OM[3] = OMcp50_335;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp50_186_5;
    sens->J[1][6] = JTcp50_186_6;
    sens->J[1][19] = JTcp50_186_7;
    sens->J[1][20] = JTcp50_186_8;
    sens->J[1][21] = JTcp50_186_9;
    sens->J[1][29] = JTcp50_186_10;
    sens->J[1][30] = JTcp50_186_11;
    sens->J[1][31] = JTcp50_186_12;
    sens->J[1][32] = JTcp50_186_13;
    sens->J[1][33] = JTcp50_186_14;
    sens->J[1][34] = JTcp50_186_15;
    sens->J[1][35] = JTcp50_186_16;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp50_286_4;
    sens->J[2][5] = JTcp50_286_5;
    sens->J[2][6] = JTcp50_286_6;
    sens->J[2][19] = JTcp50_286_7;
    sens->J[2][20] = JTcp50_286_8;
    sens->J[2][21] = JTcp50_286_9;
    sens->J[2][29] = JTcp50_286_10;
    sens->J[2][30] = JTcp50_286_11;
    sens->J[2][31] = JTcp50_286_12;
    sens->J[2][32] = JTcp50_286_13;
    sens->J[2][33] = JTcp50_286_14;
    sens->J[2][34] = JTcp50_286_15;
    sens->J[2][35] = JTcp50_286_16;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp50_386_4;
    sens->J[3][5] = JTcp50_386_5;
    sens->J[3][6] = JTcp50_386_6;
    sens->J[3][19] = JTcp50_386_7;
    sens->J[3][20] = JTcp50_386_8;
    sens->J[3][21] = JTcp50_386_9;
    sens->J[3][29] = JTcp50_386_10;
    sens->J[3][30] = JTcp50_386_11;
    sens->J[3][31] = JTcp50_386_12;
    sens->J[3][32] = JTcp50_386_13;
    sens->J[3][33] = JTcp50_386_14;
    sens->J[3][34] = JTcp50_386_15;
    sens->J[3][35] = JTcp50_386_16;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][19] = ROcp50_16;
    sens->J[4][20] = ROcp50_419;
    sens->J[4][21] = ROcp50_720;
    sens->J[4][29] = ROcp50_421;
    sens->J[4][30] = ROcp50_129;
    sens->J[4][31] = ROcp50_430;
    sens->J[4][32] = ROcp50_731;
    sens->J[4][33] = ROcp50_432;
    sens->J[4][34] = ROcp50_733;
    sens->J[4][35] = ROcp50_134;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp50_85;
    sens->J[5][19] = ROcp50_26;
    sens->J[5][20] = ROcp50_519;
    sens->J[5][21] = ROcp50_820;
    sens->J[5][29] = ROcp50_521;
    sens->J[5][30] = ROcp50_229;
    sens->J[5][31] = ROcp50_530;
    sens->J[5][32] = ROcp50_831;
    sens->J[5][33] = ROcp50_532;
    sens->J[5][34] = ROcp50_833;
    sens->J[5][35] = ROcp50_234;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp50_95;
    sens->J[6][19] = ROcp50_36;
    sens->J[6][20] = ROcp50_619;
    sens->J[6][21] = ROcp50_920;
    sens->J[6][29] = ROcp50_621;
    sens->J[6][30] = ROcp50_329;
    sens->J[6][31] = ROcp50_630;
    sens->J[6][32] = ROcp50_931;
    sens->J[6][33] = ROcp50_632;
    sens->J[6][34] = ROcp50_933;
    sens->J[6][35] = ROcp50_334;
    sens->A[1] = ACcp50_186;
    sens->A[2] = ACcp50_286;
    sens->A[3] = ACcp50_386;
    sens->OMP[1] = OPcp50_135;
    sens->OMP[2] = OPcp50_235;
    sens->OMP[3] = OPcp50_335;
 
// 
break;
case 52:
 


// = = Block_1_0_0_52_0_1 = = 
 
// Sensor Kinematics 


    ROcp51_25 = S4*S5;
    ROcp51_35 = -C4*S5;
    ROcp51_85 = -S4*C5;
    ROcp51_95 = C4*C5;
    ROcp51_16 = C5*C6;
    ROcp51_26 = ROcp51_25*C6+C4*S6;
    ROcp51_36 = ROcp51_35*C6+S4*S6;
    ROcp51_46 = -C5*S6;
    ROcp51_56 = -(ROcp51_25*S6-C4*C6);
    ROcp51_66 = -(ROcp51_35*S6-S4*C6);
    OMcp51_25 = qd[5]*C4;
    OMcp51_35 = qd[5]*S4;
    OMcp51_16 = qd[4]+qd[6]*S5;
    OMcp51_26 = OMcp51_25+ROcp51_85*qd[6];
    OMcp51_36 = OMcp51_35+ROcp51_95*qd[6];
    OPcp51_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp51_26 = ROcp51_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp51_35*S5-ROcp51_95*qd[4]);
    OPcp51_36 = ROcp51_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp51_25*S5-ROcp51_85*qd[4]);

// = = Block_1_0_0_52_0_2 = = 
 
// Sensor Kinematics 


    ROcp51_17 = ROcp51_16*C7-S5*S7;
    ROcp51_27 = ROcp51_26*C7-ROcp51_85*S7;
    ROcp51_37 = ROcp51_36*C7-ROcp51_95*S7;
    ROcp51_77 = ROcp51_16*S7+S5*C7;
    ROcp51_87 = ROcp51_26*S7+ROcp51_85*C7;
    ROcp51_97 = ROcp51_36*S7+ROcp51_95*C7;
    ROcp51_48 = ROcp51_46*C8+ROcp51_77*S8;
    ROcp51_58 = ROcp51_56*C8+ROcp51_87*S8;
    ROcp51_68 = ROcp51_66*C8+ROcp51_97*S8;
    ROcp51_78 = -(ROcp51_46*S8-ROcp51_77*C8);
    ROcp51_88 = -(ROcp51_56*S8-ROcp51_87*C8);
    ROcp51_98 = -(ROcp51_66*S8-ROcp51_97*C8);
    ROcp51_19 = ROcp51_17*C9+ROcp51_48*S9;
    ROcp51_29 = ROcp51_27*C9+ROcp51_58*S9;
    ROcp51_39 = ROcp51_37*C9+ROcp51_68*S9;
    ROcp51_49 = -(ROcp51_17*S9-ROcp51_48*C9);
    ROcp51_59 = -(ROcp51_27*S9-ROcp51_58*C9);
    ROcp51_69 = -(ROcp51_37*S9-ROcp51_68*C9);
    ROcp51_110 = ROcp51_19*C10-ROcp51_78*S10;
    ROcp51_210 = ROcp51_29*C10-ROcp51_88*S10;
    ROcp51_310 = ROcp51_39*C10-ROcp51_98*S10;
    ROcp51_710 = ROcp51_19*S10+ROcp51_78*C10;
    ROcp51_810 = ROcp51_29*S10+ROcp51_88*C10;
    ROcp51_910 = ROcp51_39*S10+ROcp51_98*C10;
    ROcp51_411 = ROcp51_49*C11+ROcp51_710*S11;
    ROcp51_511 = ROcp51_59*C11+ROcp51_810*S11;
    ROcp51_611 = ROcp51_69*C11+ROcp51_910*S11;
    ROcp51_711 = -(ROcp51_49*S11-ROcp51_710*C11);
    ROcp51_811 = -(ROcp51_59*S11-ROcp51_810*C11);
    ROcp51_911 = -(ROcp51_69*S11-ROcp51_910*C11);
    ROcp51_112 = ROcp51_110*C12-ROcp51_711*S12;
    ROcp51_212 = ROcp51_210*C12-ROcp51_811*S12;
    ROcp51_312 = ROcp51_310*C12-ROcp51_911*S12;
    ROcp51_712 = ROcp51_110*S12+ROcp51_711*C12;
    ROcp51_812 = ROcp51_210*S12+ROcp51_811*C12;
    ROcp51_912 = ROcp51_310*S12+ROcp51_911*C12;
    RLcp51_17 = s->dpt[1][1]*ROcp51_16+s->dpt[3][1]*S5+ROcp51_46*s->dpt[2][1];
    RLcp51_27 = s->dpt[1][1]*ROcp51_26+s->dpt[3][1]*ROcp51_85+ROcp51_56*s->dpt[2][1];
    RLcp51_37 = s->dpt[1][1]*ROcp51_36+s->dpt[3][1]*ROcp51_95+ROcp51_66*s->dpt[2][1];
    OMcp51_17 = OMcp51_16+ROcp51_46*qd[7];
    OMcp51_27 = OMcp51_26+ROcp51_56*qd[7];
    OMcp51_37 = OMcp51_36+ROcp51_66*qd[7];
    ORcp51_17 = OMcp51_26*RLcp51_37-OMcp51_36*RLcp51_27;
    ORcp51_27 = -(OMcp51_16*RLcp51_37-OMcp51_36*RLcp51_17);
    ORcp51_37 = OMcp51_16*RLcp51_27-OMcp51_26*RLcp51_17;
    OPcp51_17 = OPcp51_16+ROcp51_46*qdd[7]+qd[7]*(OMcp51_26*ROcp51_66-OMcp51_36*ROcp51_56);
    OPcp51_27 = OPcp51_26+ROcp51_56*qdd[7]-qd[7]*(OMcp51_16*ROcp51_66-OMcp51_36*ROcp51_46);
    OPcp51_37 = OPcp51_36+ROcp51_66*qdd[7]+qd[7]*(OMcp51_16*ROcp51_56-OMcp51_26*ROcp51_46);
    RLcp51_18 = s->dpt[1][6]*ROcp51_17+s->dpt[3][6]*ROcp51_77+ROcp51_46*s->dpt[2][6];
    RLcp51_28 = s->dpt[1][6]*ROcp51_27+s->dpt[3][6]*ROcp51_87+ROcp51_56*s->dpt[2][6];
    RLcp51_38 = s->dpt[1][6]*ROcp51_37+s->dpt[3][6]*ROcp51_97+ROcp51_66*s->dpt[2][6];
    OMcp51_18 = OMcp51_17+ROcp51_17*qd[8];
    OMcp51_28 = OMcp51_27+ROcp51_27*qd[8];
    OMcp51_38 = OMcp51_37+ROcp51_37*qd[8];
    ORcp51_18 = OMcp51_27*RLcp51_38-OMcp51_37*RLcp51_28;
    ORcp51_28 = -(OMcp51_17*RLcp51_38-OMcp51_37*RLcp51_18);
    ORcp51_38 = OMcp51_17*RLcp51_28-OMcp51_27*RLcp51_18;
    OPcp51_18 = OPcp51_17+ROcp51_17*qdd[8]+qd[8]*(OMcp51_27*ROcp51_37-OMcp51_37*ROcp51_27);
    OPcp51_28 = OPcp51_27+ROcp51_27*qdd[8]-qd[8]*(OMcp51_17*ROcp51_37-OMcp51_37*ROcp51_17);
    OPcp51_38 = OPcp51_37+ROcp51_37*qdd[8]+qd[8]*(OMcp51_17*ROcp51_27-OMcp51_27*ROcp51_17);
    RLcp51_19 = s->dpt[1][8]*ROcp51_17+s->dpt[2][8]*ROcp51_48+ROcp51_78*s->dpt[3][8];
    RLcp51_29 = s->dpt[1][8]*ROcp51_27+s->dpt[2][8]*ROcp51_58+ROcp51_88*s->dpt[3][8];
    RLcp51_39 = s->dpt[1][8]*ROcp51_37+s->dpt[2][8]*ROcp51_68+ROcp51_98*s->dpt[3][8];
    OMcp51_19 = OMcp51_18+ROcp51_78*qd[9];
    OMcp51_29 = OMcp51_28+ROcp51_88*qd[9];
    OMcp51_39 = OMcp51_38+ROcp51_98*qd[9];
    ORcp51_19 = OMcp51_28*RLcp51_39-OMcp51_38*RLcp51_29;
    ORcp51_29 = -(OMcp51_18*RLcp51_39-OMcp51_38*RLcp51_19);
    ORcp51_39 = OMcp51_18*RLcp51_29-OMcp51_28*RLcp51_19;
    OPcp51_19 = OPcp51_18+ROcp51_78*qdd[9]+qd[9]*(OMcp51_28*ROcp51_98-OMcp51_38*ROcp51_88);
    OPcp51_29 = OPcp51_28+ROcp51_88*qdd[9]-qd[9]*(OMcp51_18*ROcp51_98-OMcp51_38*ROcp51_78);
    OPcp51_39 = OPcp51_38+ROcp51_98*qdd[9]+qd[9]*(OMcp51_18*ROcp51_88-OMcp51_28*ROcp51_78);
    RLcp51_110 = s->dpt[1][10]*ROcp51_19+s->dpt[2][10]*ROcp51_49+ROcp51_78*s->dpt[3][10];
    RLcp51_210 = s->dpt[1][10]*ROcp51_29+s->dpt[2][10]*ROcp51_59+ROcp51_88*s->dpt[3][10];
    RLcp51_310 = s->dpt[1][10]*ROcp51_39+s->dpt[2][10]*ROcp51_69+ROcp51_98*s->dpt[3][10];
    OMcp51_110 = OMcp51_19+ROcp51_49*qd[10];
    OMcp51_210 = OMcp51_29+ROcp51_59*qd[10];
    OMcp51_310 = OMcp51_39+ROcp51_69*qd[10];
    ORcp51_110 = OMcp51_29*RLcp51_310-OMcp51_39*RLcp51_210;
    ORcp51_210 = -(OMcp51_19*RLcp51_310-OMcp51_39*RLcp51_110);
    ORcp51_310 = OMcp51_19*RLcp51_210-OMcp51_29*RLcp51_110;
    OPcp51_110 = OPcp51_19+ROcp51_49*qdd[10]+qd[10]*(OMcp51_29*ROcp51_69-OMcp51_39*ROcp51_59);
    OPcp51_210 = OPcp51_29+ROcp51_59*qdd[10]-qd[10]*(OMcp51_19*ROcp51_69-OMcp51_39*ROcp51_49);
    OPcp51_310 = OPcp51_39+ROcp51_69*qdd[10]+qd[10]*(OMcp51_19*ROcp51_59-OMcp51_29*ROcp51_49);
    RLcp51_111 = s->dpt[1][12]*ROcp51_110+s->dpt[2][12]*ROcp51_49+ROcp51_710*s->dpt[3][12];
    RLcp51_211 = s->dpt[1][12]*ROcp51_210+s->dpt[2][12]*ROcp51_59+ROcp51_810*s->dpt[3][12];
    RLcp51_311 = s->dpt[1][12]*ROcp51_310+s->dpt[2][12]*ROcp51_69+ROcp51_910*s->dpt[3][12];
    OMcp51_111 = OMcp51_110+ROcp51_110*qd[11];
    OMcp51_211 = OMcp51_210+ROcp51_210*qd[11];
    OMcp51_311 = OMcp51_310+ROcp51_310*qd[11];
    ORcp51_111 = OMcp51_210*RLcp51_311-OMcp51_310*RLcp51_211;
    ORcp51_211 = -(OMcp51_110*RLcp51_311-OMcp51_310*RLcp51_111);
    ORcp51_311 = OMcp51_110*RLcp51_211-OMcp51_210*RLcp51_111;
    OPcp51_111 = OPcp51_110+ROcp51_110*qdd[11]+qd[11]*(OMcp51_210*ROcp51_310-OMcp51_310*ROcp51_210);
    OPcp51_211 = OPcp51_210+ROcp51_210*qdd[11]-qd[11]*(OMcp51_110*ROcp51_310-OMcp51_310*ROcp51_110);
    OPcp51_311 = OPcp51_310+ROcp51_310*qdd[11]+qd[11]*(OMcp51_110*ROcp51_210-OMcp51_210*ROcp51_110);
    RLcp51_112 = s->dpt[1][14]*ROcp51_110+s->dpt[2][14]*ROcp51_411+s->dpt[3][14]*ROcp51_711;
    RLcp51_212 = s->dpt[1][14]*ROcp51_210+s->dpt[2][14]*ROcp51_511+s->dpt[3][14]*ROcp51_811;
    RLcp51_312 = s->dpt[1][14]*ROcp51_310+s->dpt[2][14]*ROcp51_611+s->dpt[3][14]*ROcp51_911;
    OMcp51_112 = OMcp51_111+ROcp51_411*qd[12];
    OMcp51_212 = OMcp51_211+ROcp51_511*qd[12];
    OMcp51_312 = OMcp51_311+ROcp51_611*qd[12];
    ORcp51_112 = OMcp51_211*RLcp51_312-OMcp51_311*RLcp51_212;
    ORcp51_212 = -(OMcp51_111*RLcp51_312-OMcp51_311*RLcp51_112);
    ORcp51_312 = OMcp51_111*RLcp51_212-OMcp51_211*RLcp51_112;
    OPcp51_112 = OPcp51_111+ROcp51_411*qdd[12]+qd[12]*(OMcp51_211*ROcp51_611-OMcp51_311*ROcp51_511);
    OPcp51_212 = OPcp51_211+ROcp51_511*qdd[12]-qd[12]*(OMcp51_111*ROcp51_611-OMcp51_311*ROcp51_411);
    OPcp51_312 = OPcp51_311+ROcp51_611*qdd[12]+qd[12]*(OMcp51_111*ROcp51_511-OMcp51_211*ROcp51_411);
    RLcp51_187 = s->dpt[1][16]*ROcp51_112+s->dpt[2][16]*ROcp51_411+ROcp51_712*s->dpt[3][16];
    RLcp51_287 = s->dpt[1][16]*ROcp51_212+s->dpt[2][16]*ROcp51_511+ROcp51_812*s->dpt[3][16];
    RLcp51_387 = s->dpt[1][16]*ROcp51_312+s->dpt[2][16]*ROcp51_611+ROcp51_912*s->dpt[3][16];
    POcp51_187 = RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_187+RLcp51_19+q[1];
    POcp51_287 = RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_27+RLcp51_28+RLcp51_287+RLcp51_29+q[2];
    POcp51_387 = RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_37+RLcp51_38+RLcp51_387+RLcp51_39+q[3];
    JTcp51_287_4 = -(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_37+RLcp51_38+RLcp51_387+RLcp51_39);
    JTcp51_387_4 = RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_27+RLcp51_28+RLcp51_287+RLcp51_29;
    JTcp51_187_5 = C4*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_37+RLcp51_38+RLcp51_39)-S4*(RLcp51_210+RLcp51_29)-S4*(
 RLcp51_211+RLcp51_212)-S4*(RLcp51_27+RLcp51_28)-RLcp51_287*S4+RLcp51_387*C4;
    JTcp51_287_5 = S4*(RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_187+RLcp51_19);
    JTcp51_387_5 = -C4*(RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_187+RLcp51_19);
    JTcp51_187_6 = ROcp51_85*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_37+RLcp51_38+RLcp51_39)-ROcp51_95*(RLcp51_210+
 RLcp51_29)-ROcp51_95*(RLcp51_211+RLcp51_212)-ROcp51_95*(RLcp51_27+RLcp51_28)-RLcp51_287*ROcp51_95+RLcp51_387*ROcp51_85;
    JTcp51_287_6 = -(RLcp51_387*S5-ROcp51_95*(RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_187+RLcp51_19)+
 S5*(RLcp51_310+RLcp51_39)+S5*(RLcp51_311+RLcp51_312)+S5*(RLcp51_37+RLcp51_38));
    JTcp51_387_6 = RLcp51_287*S5-ROcp51_85*(RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_187+RLcp51_19)+S5*
 (RLcp51_210+RLcp51_29)+S5*(RLcp51_211+RLcp51_212)+S5*(RLcp51_27+RLcp51_28);
    JTcp51_187_7 = ROcp51_56*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_38+RLcp51_387+RLcp51_39)-ROcp51_66*(RLcp51_210+
 RLcp51_211)-ROcp51_66*(RLcp51_212+RLcp51_287)-ROcp51_66*(RLcp51_28+RLcp51_29);
    JTcp51_287_7 = -(ROcp51_46*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_38+RLcp51_387+RLcp51_39)-ROcp51_66*(RLcp51_110+
 RLcp51_111)-ROcp51_66*(RLcp51_112+RLcp51_187)-ROcp51_66*(RLcp51_18+RLcp51_19));
    JTcp51_387_7 = ROcp51_46*(RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_28+RLcp51_287+RLcp51_29)-ROcp51_56*(RLcp51_110+
 RLcp51_111)-ROcp51_56*(RLcp51_112+RLcp51_187)-ROcp51_56*(RLcp51_18+RLcp51_19);
    JTcp51_187_8 = ROcp51_27*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_39)-ROcp51_37*(RLcp51_210+RLcp51_29)-ROcp51_37*(
 RLcp51_211+RLcp51_212)-RLcp51_287*ROcp51_37+RLcp51_387*ROcp51_27;
    JTcp51_287_8 = RLcp51_187*ROcp51_37-RLcp51_387*ROcp51_17-ROcp51_17*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_39)+
 ROcp51_37*(RLcp51_110+RLcp51_19)+ROcp51_37*(RLcp51_111+RLcp51_112);
    JTcp51_387_8 = ROcp51_17*(RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_29)-ROcp51_27*(RLcp51_110+RLcp51_19)-ROcp51_27*(
 RLcp51_111+RLcp51_112)-RLcp51_187*ROcp51_27+RLcp51_287*ROcp51_17;
    JTcp51_187_9 = ROcp51_88*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_387)-ROcp51_98*(RLcp51_210+RLcp51_211)-ROcp51_98*(
 RLcp51_212+RLcp51_287);
    JTcp51_287_9 = -(ROcp51_78*(RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_387)-ROcp51_98*(RLcp51_110+RLcp51_111)-ROcp51_98*(
 RLcp51_112+RLcp51_187));
    JTcp51_387_9 = ROcp51_78*(RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_287)-ROcp51_88*(RLcp51_110+RLcp51_111)-ROcp51_88*(
 RLcp51_112+RLcp51_187);
    JTcp51_187_10 = ROcp51_59*(RLcp51_311+RLcp51_312)-ROcp51_69*(RLcp51_211+RLcp51_212)-RLcp51_287*ROcp51_69+RLcp51_387*
 ROcp51_59;
    JTcp51_287_10 = RLcp51_187*ROcp51_69-RLcp51_387*ROcp51_49-ROcp51_49*(RLcp51_311+RLcp51_312)+ROcp51_69*(RLcp51_111+
 RLcp51_112);
    JTcp51_387_10 = ROcp51_49*(RLcp51_211+RLcp51_212)-ROcp51_59*(RLcp51_111+RLcp51_112)-RLcp51_187*ROcp51_59+RLcp51_287*
 ROcp51_49;
    JTcp51_187_11 = ROcp51_210*(RLcp51_312+RLcp51_387)-ROcp51_310*(RLcp51_212+RLcp51_287);
    JTcp51_287_11 = -(ROcp51_110*(RLcp51_312+RLcp51_387)-ROcp51_310*(RLcp51_112+RLcp51_187));
    JTcp51_387_11 = ROcp51_110*(RLcp51_212+RLcp51_287)-ROcp51_210*(RLcp51_112+RLcp51_187);
    JTcp51_187_12 = -(RLcp51_287*ROcp51_611-RLcp51_387*ROcp51_511);
    JTcp51_287_12 = RLcp51_187*ROcp51_611-RLcp51_387*ROcp51_411;
    JTcp51_387_12 = -(RLcp51_187*ROcp51_511-RLcp51_287*ROcp51_411);
    ORcp51_187 = OMcp51_212*RLcp51_387-OMcp51_312*RLcp51_287;
    ORcp51_287 = -(OMcp51_112*RLcp51_387-OMcp51_312*RLcp51_187);
    ORcp51_387 = OMcp51_112*RLcp51_287-OMcp51_212*RLcp51_187;
    VIcp51_187 = ORcp51_110+ORcp51_111+ORcp51_112+ORcp51_17+ORcp51_18+ORcp51_187+ORcp51_19+qd[1];
    VIcp51_287 = ORcp51_210+ORcp51_211+ORcp51_212+ORcp51_27+ORcp51_28+ORcp51_287+ORcp51_29+qd[2];
    VIcp51_387 = ORcp51_310+ORcp51_311+ORcp51_312+ORcp51_37+ORcp51_38+ORcp51_387+ORcp51_39+qd[3];
    ACcp51_187 = qdd[1]+OMcp51_210*ORcp51_311+OMcp51_211*ORcp51_312+OMcp51_212*ORcp51_387+OMcp51_26*ORcp51_37+OMcp51_27*
 ORcp51_38+OMcp51_28*ORcp51_39+OMcp51_29*ORcp51_310-OMcp51_310*ORcp51_211-OMcp51_311*ORcp51_212-OMcp51_312*ORcp51_287-
 OMcp51_36*ORcp51_27-OMcp51_37*ORcp51_28-OMcp51_38*ORcp51_29-OMcp51_39*ORcp51_210+OPcp51_210*RLcp51_311+OPcp51_211*RLcp51_312
 +OPcp51_212*RLcp51_387+OPcp51_26*RLcp51_37+OPcp51_27*RLcp51_38+OPcp51_28*RLcp51_39+OPcp51_29*RLcp51_310-OPcp51_310*
 RLcp51_211-OPcp51_311*RLcp51_212-OPcp51_312*RLcp51_287-OPcp51_36*RLcp51_27-OPcp51_37*RLcp51_28-OPcp51_38*RLcp51_29-OPcp51_39
 *RLcp51_210;
    ACcp51_287 = qdd[2]-OMcp51_110*ORcp51_311-OMcp51_111*ORcp51_312-OMcp51_112*ORcp51_387-OMcp51_16*ORcp51_37-OMcp51_17*
 ORcp51_38-OMcp51_18*ORcp51_39-OMcp51_19*ORcp51_310+OMcp51_310*ORcp51_111+OMcp51_311*ORcp51_112+OMcp51_312*ORcp51_187+
 OMcp51_36*ORcp51_17+OMcp51_37*ORcp51_18+OMcp51_38*ORcp51_19+OMcp51_39*ORcp51_110-OPcp51_110*RLcp51_311-OPcp51_111*RLcp51_312
 -OPcp51_112*RLcp51_387-OPcp51_16*RLcp51_37-OPcp51_17*RLcp51_38-OPcp51_18*RLcp51_39-OPcp51_19*RLcp51_310+OPcp51_310*
 RLcp51_111+OPcp51_311*RLcp51_112+OPcp51_312*RLcp51_187+OPcp51_36*RLcp51_17+OPcp51_37*RLcp51_18+OPcp51_38*RLcp51_19+OPcp51_39
 *RLcp51_110;
    ACcp51_387 = qdd[3]+OMcp51_110*ORcp51_211+OMcp51_111*ORcp51_212+OMcp51_112*ORcp51_287+OMcp51_16*ORcp51_27+OMcp51_17*
 ORcp51_28+OMcp51_18*ORcp51_29+OMcp51_19*ORcp51_210-OMcp51_210*ORcp51_111-OMcp51_211*ORcp51_112-OMcp51_212*ORcp51_187-
 OMcp51_26*ORcp51_17-OMcp51_27*ORcp51_18-OMcp51_28*ORcp51_19-OMcp51_29*ORcp51_110+OPcp51_110*RLcp51_211+OPcp51_111*RLcp51_212
 +OPcp51_112*RLcp51_287+OPcp51_16*RLcp51_27+OPcp51_17*RLcp51_28+OPcp51_18*RLcp51_29+OPcp51_19*RLcp51_210-OPcp51_210*
 RLcp51_111-OPcp51_211*RLcp51_112-OPcp51_212*RLcp51_187-OPcp51_26*RLcp51_17-OPcp51_27*RLcp51_18-OPcp51_28*RLcp51_19-OPcp51_29
 *RLcp51_110;

// = = Block_1_0_0_52_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp51_187;
    sens->P[2] = POcp51_287;
    sens->P[3] = POcp51_387;
    sens->R[1][1] = ROcp51_112;
    sens->R[1][2] = ROcp51_212;
    sens->R[1][3] = ROcp51_312;
    sens->R[2][1] = ROcp51_411;
    sens->R[2][2] = ROcp51_511;
    sens->R[2][3] = ROcp51_611;
    sens->R[3][1] = ROcp51_712;
    sens->R[3][2] = ROcp51_812;
    sens->R[3][3] = ROcp51_912;
    sens->V[1] = VIcp51_187;
    sens->V[2] = VIcp51_287;
    sens->V[3] = VIcp51_387;
    sens->OM[1] = OMcp51_112;
    sens->OM[2] = OMcp51_212;
    sens->OM[3] = OMcp51_312;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp51_187_5;
    sens->J[1][6] = JTcp51_187_6;
    sens->J[1][7] = JTcp51_187_7;
    sens->J[1][8] = JTcp51_187_8;
    sens->J[1][9] = JTcp51_187_9;
    sens->J[1][10] = JTcp51_187_10;
    sens->J[1][11] = JTcp51_187_11;
    sens->J[1][12] = JTcp51_187_12;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp51_287_4;
    sens->J[2][5] = JTcp51_287_5;
    sens->J[2][6] = JTcp51_287_6;
    sens->J[2][7] = JTcp51_287_7;
    sens->J[2][8] = JTcp51_287_8;
    sens->J[2][9] = JTcp51_287_9;
    sens->J[2][10] = JTcp51_287_10;
    sens->J[2][11] = JTcp51_287_11;
    sens->J[2][12] = JTcp51_287_12;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp51_387_4;
    sens->J[3][5] = JTcp51_387_5;
    sens->J[3][6] = JTcp51_387_6;
    sens->J[3][7] = JTcp51_387_7;
    sens->J[3][8] = JTcp51_387_8;
    sens->J[3][9] = JTcp51_387_9;
    sens->J[3][10] = JTcp51_387_10;
    sens->J[3][11] = JTcp51_387_11;
    sens->J[3][12] = JTcp51_387_12;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][7] = ROcp51_46;
    sens->J[4][8] = ROcp51_17;
    sens->J[4][9] = ROcp51_78;
    sens->J[4][10] = ROcp51_49;
    sens->J[4][11] = ROcp51_110;
    sens->J[4][12] = ROcp51_411;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp51_85;
    sens->J[5][7] = ROcp51_56;
    sens->J[5][8] = ROcp51_27;
    sens->J[5][9] = ROcp51_88;
    sens->J[5][10] = ROcp51_59;
    sens->J[5][11] = ROcp51_210;
    sens->J[5][12] = ROcp51_511;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp51_95;
    sens->J[6][7] = ROcp51_66;
    sens->J[6][8] = ROcp51_37;
    sens->J[6][9] = ROcp51_98;
    sens->J[6][10] = ROcp51_69;
    sens->J[6][11] = ROcp51_310;
    sens->J[6][12] = ROcp51_611;
    sens->A[1] = ACcp51_187;
    sens->A[2] = ACcp51_287;
    sens->A[3] = ACcp51_387;
    sens->OMP[1] = OPcp51_112;
    sens->OMP[2] = OPcp51_212;
    sens->OMP[3] = OPcp51_312;
 
// 
break;
case 53:
 


// = = Block_1_0_0_53_0_1 = = 
 
// Sensor Kinematics 


    ROcp52_25 = S4*S5;
    ROcp52_35 = -C4*S5;
    ROcp52_85 = -S4*C5;
    ROcp52_95 = C4*C5;
    ROcp52_16 = C5*C6;
    ROcp52_26 = ROcp52_25*C6+C4*S6;
    ROcp52_36 = ROcp52_35*C6+S4*S6;
    ROcp52_46 = -C5*S6;
    ROcp52_56 = -(ROcp52_25*S6-C4*C6);
    ROcp52_66 = -(ROcp52_35*S6-S4*C6);
    OMcp52_25 = qd[5]*C4;
    OMcp52_35 = qd[5]*S4;
    OMcp52_16 = qd[4]+qd[6]*S5;
    OMcp52_26 = OMcp52_25+ROcp52_85*qd[6];
    OMcp52_36 = OMcp52_35+ROcp52_95*qd[6];
    OPcp52_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp52_26 = ROcp52_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp52_35*S5-ROcp52_95*qd[4]);
    OPcp52_36 = ROcp52_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp52_25*S5-ROcp52_85*qd[4]);

// = = Block_1_0_0_53_0_2 = = 
 
// Sensor Kinematics 


    ROcp52_17 = ROcp52_16*C7-S5*S7;
    ROcp52_27 = ROcp52_26*C7-ROcp52_85*S7;
    ROcp52_37 = ROcp52_36*C7-ROcp52_95*S7;
    ROcp52_77 = ROcp52_16*S7+S5*C7;
    ROcp52_87 = ROcp52_26*S7+ROcp52_85*C7;
    ROcp52_97 = ROcp52_36*S7+ROcp52_95*C7;
    ROcp52_48 = ROcp52_46*C8+ROcp52_77*S8;
    ROcp52_58 = ROcp52_56*C8+ROcp52_87*S8;
    ROcp52_68 = ROcp52_66*C8+ROcp52_97*S8;
    ROcp52_78 = -(ROcp52_46*S8-ROcp52_77*C8);
    ROcp52_88 = -(ROcp52_56*S8-ROcp52_87*C8);
    ROcp52_98 = -(ROcp52_66*S8-ROcp52_97*C8);
    ROcp52_19 = ROcp52_17*C9+ROcp52_48*S9;
    ROcp52_29 = ROcp52_27*C9+ROcp52_58*S9;
    ROcp52_39 = ROcp52_37*C9+ROcp52_68*S9;
    ROcp52_49 = -(ROcp52_17*S9-ROcp52_48*C9);
    ROcp52_59 = -(ROcp52_27*S9-ROcp52_58*C9);
    ROcp52_69 = -(ROcp52_37*S9-ROcp52_68*C9);
    ROcp52_110 = ROcp52_19*C10-ROcp52_78*S10;
    ROcp52_210 = ROcp52_29*C10-ROcp52_88*S10;
    ROcp52_310 = ROcp52_39*C10-ROcp52_98*S10;
    ROcp52_710 = ROcp52_19*S10+ROcp52_78*C10;
    ROcp52_810 = ROcp52_29*S10+ROcp52_88*C10;
    ROcp52_910 = ROcp52_39*S10+ROcp52_98*C10;
    ROcp52_411 = ROcp52_49*C11+ROcp52_710*S11;
    ROcp52_511 = ROcp52_59*C11+ROcp52_810*S11;
    ROcp52_611 = ROcp52_69*C11+ROcp52_910*S11;
    ROcp52_711 = -(ROcp52_49*S11-ROcp52_710*C11);
    ROcp52_811 = -(ROcp52_59*S11-ROcp52_810*C11);
    ROcp52_911 = -(ROcp52_69*S11-ROcp52_910*C11);
    ROcp52_112 = ROcp52_110*C12-ROcp52_711*S12;
    ROcp52_212 = ROcp52_210*C12-ROcp52_811*S12;
    ROcp52_312 = ROcp52_310*C12-ROcp52_911*S12;
    ROcp52_712 = ROcp52_110*S12+ROcp52_711*C12;
    ROcp52_812 = ROcp52_210*S12+ROcp52_811*C12;
    ROcp52_912 = ROcp52_310*S12+ROcp52_911*C12;
    RLcp52_17 = s->dpt[1][1]*ROcp52_16+s->dpt[3][1]*S5+ROcp52_46*s->dpt[2][1];
    RLcp52_27 = s->dpt[1][1]*ROcp52_26+s->dpt[3][1]*ROcp52_85+ROcp52_56*s->dpt[2][1];
    RLcp52_37 = s->dpt[1][1]*ROcp52_36+s->dpt[3][1]*ROcp52_95+ROcp52_66*s->dpt[2][1];
    OMcp52_17 = OMcp52_16+ROcp52_46*qd[7];
    OMcp52_27 = OMcp52_26+ROcp52_56*qd[7];
    OMcp52_37 = OMcp52_36+ROcp52_66*qd[7];
    ORcp52_17 = OMcp52_26*RLcp52_37-OMcp52_36*RLcp52_27;
    ORcp52_27 = -(OMcp52_16*RLcp52_37-OMcp52_36*RLcp52_17);
    ORcp52_37 = OMcp52_16*RLcp52_27-OMcp52_26*RLcp52_17;
    OPcp52_17 = OPcp52_16+ROcp52_46*qdd[7]+qd[7]*(OMcp52_26*ROcp52_66-OMcp52_36*ROcp52_56);
    OPcp52_27 = OPcp52_26+ROcp52_56*qdd[7]-qd[7]*(OMcp52_16*ROcp52_66-OMcp52_36*ROcp52_46);
    OPcp52_37 = OPcp52_36+ROcp52_66*qdd[7]+qd[7]*(OMcp52_16*ROcp52_56-OMcp52_26*ROcp52_46);
    RLcp52_18 = s->dpt[1][6]*ROcp52_17+s->dpt[3][6]*ROcp52_77+ROcp52_46*s->dpt[2][6];
    RLcp52_28 = s->dpt[1][6]*ROcp52_27+s->dpt[3][6]*ROcp52_87+ROcp52_56*s->dpt[2][6];
    RLcp52_38 = s->dpt[1][6]*ROcp52_37+s->dpt[3][6]*ROcp52_97+ROcp52_66*s->dpt[2][6];
    OMcp52_18 = OMcp52_17+ROcp52_17*qd[8];
    OMcp52_28 = OMcp52_27+ROcp52_27*qd[8];
    OMcp52_38 = OMcp52_37+ROcp52_37*qd[8];
    ORcp52_18 = OMcp52_27*RLcp52_38-OMcp52_37*RLcp52_28;
    ORcp52_28 = -(OMcp52_17*RLcp52_38-OMcp52_37*RLcp52_18);
    ORcp52_38 = OMcp52_17*RLcp52_28-OMcp52_27*RLcp52_18;
    OPcp52_18 = OPcp52_17+ROcp52_17*qdd[8]+qd[8]*(OMcp52_27*ROcp52_37-OMcp52_37*ROcp52_27);
    OPcp52_28 = OPcp52_27+ROcp52_27*qdd[8]-qd[8]*(OMcp52_17*ROcp52_37-OMcp52_37*ROcp52_17);
    OPcp52_38 = OPcp52_37+ROcp52_37*qdd[8]+qd[8]*(OMcp52_17*ROcp52_27-OMcp52_27*ROcp52_17);
    RLcp52_19 = s->dpt[1][8]*ROcp52_17+s->dpt[2][8]*ROcp52_48+ROcp52_78*s->dpt[3][8];
    RLcp52_29 = s->dpt[1][8]*ROcp52_27+s->dpt[2][8]*ROcp52_58+ROcp52_88*s->dpt[3][8];
    RLcp52_39 = s->dpt[1][8]*ROcp52_37+s->dpt[2][8]*ROcp52_68+ROcp52_98*s->dpt[3][8];
    OMcp52_19 = OMcp52_18+ROcp52_78*qd[9];
    OMcp52_29 = OMcp52_28+ROcp52_88*qd[9];
    OMcp52_39 = OMcp52_38+ROcp52_98*qd[9];
    ORcp52_19 = OMcp52_28*RLcp52_39-OMcp52_38*RLcp52_29;
    ORcp52_29 = -(OMcp52_18*RLcp52_39-OMcp52_38*RLcp52_19);
    ORcp52_39 = OMcp52_18*RLcp52_29-OMcp52_28*RLcp52_19;
    OPcp52_19 = OPcp52_18+ROcp52_78*qdd[9]+qd[9]*(OMcp52_28*ROcp52_98-OMcp52_38*ROcp52_88);
    OPcp52_29 = OPcp52_28+ROcp52_88*qdd[9]-qd[9]*(OMcp52_18*ROcp52_98-OMcp52_38*ROcp52_78);
    OPcp52_39 = OPcp52_38+ROcp52_98*qdd[9]+qd[9]*(OMcp52_18*ROcp52_88-OMcp52_28*ROcp52_78);
    RLcp52_110 = s->dpt[1][10]*ROcp52_19+s->dpt[2][10]*ROcp52_49+ROcp52_78*s->dpt[3][10];
    RLcp52_210 = s->dpt[1][10]*ROcp52_29+s->dpt[2][10]*ROcp52_59+ROcp52_88*s->dpt[3][10];
    RLcp52_310 = s->dpt[1][10]*ROcp52_39+s->dpt[2][10]*ROcp52_69+ROcp52_98*s->dpt[3][10];
    OMcp52_110 = OMcp52_19+ROcp52_49*qd[10];
    OMcp52_210 = OMcp52_29+ROcp52_59*qd[10];
    OMcp52_310 = OMcp52_39+ROcp52_69*qd[10];
    ORcp52_110 = OMcp52_29*RLcp52_310-OMcp52_39*RLcp52_210;
    ORcp52_210 = -(OMcp52_19*RLcp52_310-OMcp52_39*RLcp52_110);
    ORcp52_310 = OMcp52_19*RLcp52_210-OMcp52_29*RLcp52_110;
    OPcp52_110 = OPcp52_19+ROcp52_49*qdd[10]+qd[10]*(OMcp52_29*ROcp52_69-OMcp52_39*ROcp52_59);
    OPcp52_210 = OPcp52_29+ROcp52_59*qdd[10]-qd[10]*(OMcp52_19*ROcp52_69-OMcp52_39*ROcp52_49);
    OPcp52_310 = OPcp52_39+ROcp52_69*qdd[10]+qd[10]*(OMcp52_19*ROcp52_59-OMcp52_29*ROcp52_49);
    RLcp52_111 = s->dpt[1][12]*ROcp52_110+s->dpt[2][12]*ROcp52_49+ROcp52_710*s->dpt[3][12];
    RLcp52_211 = s->dpt[1][12]*ROcp52_210+s->dpt[2][12]*ROcp52_59+ROcp52_810*s->dpt[3][12];
    RLcp52_311 = s->dpt[1][12]*ROcp52_310+s->dpt[2][12]*ROcp52_69+ROcp52_910*s->dpt[3][12];
    OMcp52_111 = OMcp52_110+ROcp52_110*qd[11];
    OMcp52_211 = OMcp52_210+ROcp52_210*qd[11];
    OMcp52_311 = OMcp52_310+ROcp52_310*qd[11];
    ORcp52_111 = OMcp52_210*RLcp52_311-OMcp52_310*RLcp52_211;
    ORcp52_211 = -(OMcp52_110*RLcp52_311-OMcp52_310*RLcp52_111);
    ORcp52_311 = OMcp52_110*RLcp52_211-OMcp52_210*RLcp52_111;
    OPcp52_111 = OPcp52_110+ROcp52_110*qdd[11]+qd[11]*(OMcp52_210*ROcp52_310-OMcp52_310*ROcp52_210);
    OPcp52_211 = OPcp52_210+ROcp52_210*qdd[11]-qd[11]*(OMcp52_110*ROcp52_310-OMcp52_310*ROcp52_110);
    OPcp52_311 = OPcp52_310+ROcp52_310*qdd[11]+qd[11]*(OMcp52_110*ROcp52_210-OMcp52_210*ROcp52_110);
    RLcp52_112 = s->dpt[1][14]*ROcp52_110+s->dpt[2][14]*ROcp52_411+s->dpt[3][14]*ROcp52_711;
    RLcp52_212 = s->dpt[1][14]*ROcp52_210+s->dpt[2][14]*ROcp52_511+s->dpt[3][14]*ROcp52_811;
    RLcp52_312 = s->dpt[1][14]*ROcp52_310+s->dpt[2][14]*ROcp52_611+s->dpt[3][14]*ROcp52_911;
    OMcp52_112 = OMcp52_111+ROcp52_411*qd[12];
    OMcp52_212 = OMcp52_211+ROcp52_511*qd[12];
    OMcp52_312 = OMcp52_311+ROcp52_611*qd[12];
    ORcp52_112 = OMcp52_211*RLcp52_312-OMcp52_311*RLcp52_212;
    ORcp52_212 = -(OMcp52_111*RLcp52_312-OMcp52_311*RLcp52_112);
    ORcp52_312 = OMcp52_111*RLcp52_212-OMcp52_211*RLcp52_112;
    OPcp52_112 = OPcp52_111+ROcp52_411*qdd[12]+qd[12]*(OMcp52_211*ROcp52_611-OMcp52_311*ROcp52_511);
    OPcp52_212 = OPcp52_211+ROcp52_511*qdd[12]-qd[12]*(OMcp52_111*ROcp52_611-OMcp52_311*ROcp52_411);
    OPcp52_312 = OPcp52_311+ROcp52_611*qdd[12]+qd[12]*(OMcp52_111*ROcp52_511-OMcp52_211*ROcp52_411);
    RLcp52_188 = ROcp52_112*s->dpt[1][18]+ROcp52_411*s->dpt[2][18]+ROcp52_712*s->dpt[3][18];
    RLcp52_288 = ROcp52_212*s->dpt[1][18]+ROcp52_511*s->dpt[2][18]+ROcp52_812*s->dpt[3][18];
    RLcp52_388 = ROcp52_312*s->dpt[1][18]+ROcp52_611*s->dpt[2][18]+ROcp52_912*s->dpt[3][18];
    POcp52_188 = RLcp52_110+RLcp52_111+RLcp52_112+RLcp52_17+RLcp52_18+RLcp52_188+RLcp52_19+q[1];
    POcp52_288 = RLcp52_210+RLcp52_211+RLcp52_212+RLcp52_27+RLcp52_28+RLcp52_288+RLcp52_29+q[2];
    POcp52_388 = RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_37+RLcp52_38+RLcp52_388+RLcp52_39+q[3];
    ORcp52_188 = OMcp52_212*RLcp52_388-OMcp52_312*RLcp52_288;
    ORcp52_288 = -(OMcp52_112*RLcp52_388-OMcp52_312*RLcp52_188);
    ORcp52_388 = OMcp52_112*RLcp52_288-OMcp52_212*RLcp52_188;
    VIcp52_188 = ORcp52_110+ORcp52_111+ORcp52_112+ORcp52_17+ORcp52_18+ORcp52_188+ORcp52_19+qd[1];
    VIcp52_288 = ORcp52_210+ORcp52_211+ORcp52_212+ORcp52_27+ORcp52_28+ORcp52_288+ORcp52_29+qd[2];
    VIcp52_388 = ORcp52_310+ORcp52_311+ORcp52_312+ORcp52_37+ORcp52_38+ORcp52_388+ORcp52_39+qd[3];
    ACcp52_188 = qdd[1]+OMcp52_210*ORcp52_311+OMcp52_211*ORcp52_312+OMcp52_212*ORcp52_388+OMcp52_26*ORcp52_37+OMcp52_27*
 ORcp52_38+OMcp52_28*ORcp52_39+OMcp52_29*ORcp52_310-OMcp52_310*ORcp52_211-OMcp52_311*ORcp52_212-OMcp52_312*ORcp52_288-
 OMcp52_36*ORcp52_27-OMcp52_37*ORcp52_28-OMcp52_38*ORcp52_29-OMcp52_39*ORcp52_210+OPcp52_210*RLcp52_311+OPcp52_211*RLcp52_312
 +OPcp52_212*RLcp52_388+OPcp52_26*RLcp52_37+OPcp52_27*RLcp52_38+OPcp52_28*RLcp52_39+OPcp52_29*RLcp52_310-OPcp52_310*
 RLcp52_211-OPcp52_311*RLcp52_212-OPcp52_312*RLcp52_288-OPcp52_36*RLcp52_27-OPcp52_37*RLcp52_28-OPcp52_38*RLcp52_29-OPcp52_39
 *RLcp52_210;
    ACcp52_288 = qdd[2]-OMcp52_110*ORcp52_311-OMcp52_111*ORcp52_312-OMcp52_112*ORcp52_388-OMcp52_16*ORcp52_37-OMcp52_17*
 ORcp52_38-OMcp52_18*ORcp52_39-OMcp52_19*ORcp52_310+OMcp52_310*ORcp52_111+OMcp52_311*ORcp52_112+OMcp52_312*ORcp52_188+
 OMcp52_36*ORcp52_17+OMcp52_37*ORcp52_18+OMcp52_38*ORcp52_19+OMcp52_39*ORcp52_110-OPcp52_110*RLcp52_311-OPcp52_111*RLcp52_312
 -OPcp52_112*RLcp52_388-OPcp52_16*RLcp52_37-OPcp52_17*RLcp52_38-OPcp52_18*RLcp52_39-OPcp52_19*RLcp52_310+OPcp52_310*
 RLcp52_111+OPcp52_311*RLcp52_112+OPcp52_312*RLcp52_188+OPcp52_36*RLcp52_17+OPcp52_37*RLcp52_18+OPcp52_38*RLcp52_19+OPcp52_39
 *RLcp52_110;
    ACcp52_388 = qdd[3]+OMcp52_110*ORcp52_211+OMcp52_111*ORcp52_212+OMcp52_112*ORcp52_288+OMcp52_16*ORcp52_27+OMcp52_17*
 ORcp52_28+OMcp52_18*ORcp52_29+OMcp52_19*ORcp52_210-OMcp52_210*ORcp52_111-OMcp52_211*ORcp52_112-OMcp52_212*ORcp52_188-
 OMcp52_26*ORcp52_17-OMcp52_27*ORcp52_18-OMcp52_28*ORcp52_19-OMcp52_29*ORcp52_110+OPcp52_110*RLcp52_211+OPcp52_111*RLcp52_212
 +OPcp52_112*RLcp52_288+OPcp52_16*RLcp52_27+OPcp52_17*RLcp52_28+OPcp52_18*RLcp52_29+OPcp52_19*RLcp52_210-OPcp52_210*
 RLcp52_111-OPcp52_211*RLcp52_112-OPcp52_212*RLcp52_188-OPcp52_26*RLcp52_17-OPcp52_27*RLcp52_18-OPcp52_28*RLcp52_19-OPcp52_29
 *RLcp52_110;

// = = Block_1_0_0_53_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp52_188;
    sens->P[2] = POcp52_288;
    sens->P[3] = POcp52_388;
    sens->R[1][1] = ROcp52_112;
    sens->R[1][2] = ROcp52_212;
    sens->R[1][3] = ROcp52_312;
    sens->R[2][1] = ROcp52_411;
    sens->R[2][2] = ROcp52_511;
    sens->R[2][3] = ROcp52_611;
    sens->R[3][1] = ROcp52_712;
    sens->R[3][2] = ROcp52_812;
    sens->R[3][3] = ROcp52_912;
    sens->V[1] = VIcp52_188;
    sens->V[2] = VIcp52_288;
    sens->V[3] = VIcp52_388;
    sens->OM[1] = OMcp52_112;
    sens->OM[2] = OMcp52_212;
    sens->OM[3] = OMcp52_312;
    sens->A[1] = ACcp52_188;
    sens->A[2] = ACcp52_288;
    sens->A[3] = ACcp52_388;
    sens->OMP[1] = OPcp52_112;
    sens->OMP[2] = OPcp52_212;
    sens->OMP[3] = OPcp52_312;
 
// 
break;
case 54:
 


// = = Block_1_0_0_54_0_1 = = 
 
// Sensor Kinematics 


    ROcp53_25 = S4*S5;
    ROcp53_35 = -C4*S5;
    ROcp53_85 = -S4*C5;
    ROcp53_95 = C4*C5;
    ROcp53_16 = C5*C6;
    ROcp53_26 = ROcp53_25*C6+C4*S6;
    ROcp53_36 = ROcp53_35*C6+S4*S6;
    ROcp53_46 = -C5*S6;
    ROcp53_56 = -(ROcp53_25*S6-C4*C6);
    ROcp53_66 = -(ROcp53_35*S6-S4*C6);
    OMcp53_25 = qd[5]*C4;
    OMcp53_35 = qd[5]*S4;
    OMcp53_16 = qd[4]+qd[6]*S5;
    OMcp53_26 = OMcp53_25+ROcp53_85*qd[6];
    OMcp53_36 = OMcp53_35+ROcp53_95*qd[6];
    OPcp53_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp53_26 = ROcp53_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp53_35*S5-ROcp53_95*qd[4]);
    OPcp53_36 = ROcp53_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp53_25*S5-ROcp53_85*qd[4]);

// = = Block_1_0_0_54_0_2 = = 
 
// Sensor Kinematics 


    ROcp53_17 = ROcp53_16*C7-S5*S7;
    ROcp53_27 = ROcp53_26*C7-ROcp53_85*S7;
    ROcp53_37 = ROcp53_36*C7-ROcp53_95*S7;
    ROcp53_77 = ROcp53_16*S7+S5*C7;
    ROcp53_87 = ROcp53_26*S7+ROcp53_85*C7;
    ROcp53_97 = ROcp53_36*S7+ROcp53_95*C7;
    ROcp53_48 = ROcp53_46*C8+ROcp53_77*S8;
    ROcp53_58 = ROcp53_56*C8+ROcp53_87*S8;
    ROcp53_68 = ROcp53_66*C8+ROcp53_97*S8;
    ROcp53_78 = -(ROcp53_46*S8-ROcp53_77*C8);
    ROcp53_88 = -(ROcp53_56*S8-ROcp53_87*C8);
    ROcp53_98 = -(ROcp53_66*S8-ROcp53_97*C8);
    ROcp53_19 = ROcp53_17*C9+ROcp53_48*S9;
    ROcp53_29 = ROcp53_27*C9+ROcp53_58*S9;
    ROcp53_39 = ROcp53_37*C9+ROcp53_68*S9;
    ROcp53_49 = -(ROcp53_17*S9-ROcp53_48*C9);
    ROcp53_59 = -(ROcp53_27*S9-ROcp53_58*C9);
    ROcp53_69 = -(ROcp53_37*S9-ROcp53_68*C9);
    ROcp53_110 = ROcp53_19*C10-ROcp53_78*S10;
    ROcp53_210 = ROcp53_29*C10-ROcp53_88*S10;
    ROcp53_310 = ROcp53_39*C10-ROcp53_98*S10;
    ROcp53_710 = ROcp53_19*S10+ROcp53_78*C10;
    ROcp53_810 = ROcp53_29*S10+ROcp53_88*C10;
    ROcp53_910 = ROcp53_39*S10+ROcp53_98*C10;
    ROcp53_411 = ROcp53_49*C11+ROcp53_710*S11;
    ROcp53_511 = ROcp53_59*C11+ROcp53_810*S11;
    ROcp53_611 = ROcp53_69*C11+ROcp53_910*S11;
    ROcp53_711 = -(ROcp53_49*S11-ROcp53_710*C11);
    ROcp53_811 = -(ROcp53_59*S11-ROcp53_810*C11);
    ROcp53_911 = -(ROcp53_69*S11-ROcp53_910*C11);
    ROcp53_112 = ROcp53_110*C12-ROcp53_711*S12;
    ROcp53_212 = ROcp53_210*C12-ROcp53_811*S12;
    ROcp53_312 = ROcp53_310*C12-ROcp53_911*S12;
    ROcp53_712 = ROcp53_110*S12+ROcp53_711*C12;
    ROcp53_812 = ROcp53_210*S12+ROcp53_811*C12;
    ROcp53_912 = ROcp53_310*S12+ROcp53_911*C12;
    RLcp53_17 = s->dpt[1][1]*ROcp53_16+s->dpt[3][1]*S5+ROcp53_46*s->dpt[2][1];
    RLcp53_27 = s->dpt[1][1]*ROcp53_26+s->dpt[3][1]*ROcp53_85+ROcp53_56*s->dpt[2][1];
    RLcp53_37 = s->dpt[1][1]*ROcp53_36+s->dpt[3][1]*ROcp53_95+ROcp53_66*s->dpt[2][1];
    OMcp53_17 = OMcp53_16+ROcp53_46*qd[7];
    OMcp53_27 = OMcp53_26+ROcp53_56*qd[7];
    OMcp53_37 = OMcp53_36+ROcp53_66*qd[7];
    ORcp53_17 = OMcp53_26*RLcp53_37-OMcp53_36*RLcp53_27;
    ORcp53_27 = -(OMcp53_16*RLcp53_37-OMcp53_36*RLcp53_17);
    ORcp53_37 = OMcp53_16*RLcp53_27-OMcp53_26*RLcp53_17;
    OPcp53_17 = OPcp53_16+ROcp53_46*qdd[7]+qd[7]*(OMcp53_26*ROcp53_66-OMcp53_36*ROcp53_56);
    OPcp53_27 = OPcp53_26+ROcp53_56*qdd[7]-qd[7]*(OMcp53_16*ROcp53_66-OMcp53_36*ROcp53_46);
    OPcp53_37 = OPcp53_36+ROcp53_66*qdd[7]+qd[7]*(OMcp53_16*ROcp53_56-OMcp53_26*ROcp53_46);
    RLcp53_18 = s->dpt[1][6]*ROcp53_17+s->dpt[3][6]*ROcp53_77+ROcp53_46*s->dpt[2][6];
    RLcp53_28 = s->dpt[1][6]*ROcp53_27+s->dpt[3][6]*ROcp53_87+ROcp53_56*s->dpt[2][6];
    RLcp53_38 = s->dpt[1][6]*ROcp53_37+s->dpt[3][6]*ROcp53_97+ROcp53_66*s->dpt[2][6];
    OMcp53_18 = OMcp53_17+ROcp53_17*qd[8];
    OMcp53_28 = OMcp53_27+ROcp53_27*qd[8];
    OMcp53_38 = OMcp53_37+ROcp53_37*qd[8];
    ORcp53_18 = OMcp53_27*RLcp53_38-OMcp53_37*RLcp53_28;
    ORcp53_28 = -(OMcp53_17*RLcp53_38-OMcp53_37*RLcp53_18);
    ORcp53_38 = OMcp53_17*RLcp53_28-OMcp53_27*RLcp53_18;
    OPcp53_18 = OPcp53_17+ROcp53_17*qdd[8]+qd[8]*(OMcp53_27*ROcp53_37-OMcp53_37*ROcp53_27);
    OPcp53_28 = OPcp53_27+ROcp53_27*qdd[8]-qd[8]*(OMcp53_17*ROcp53_37-OMcp53_37*ROcp53_17);
    OPcp53_38 = OPcp53_37+ROcp53_37*qdd[8]+qd[8]*(OMcp53_17*ROcp53_27-OMcp53_27*ROcp53_17);
    RLcp53_19 = s->dpt[1][8]*ROcp53_17+s->dpt[2][8]*ROcp53_48+ROcp53_78*s->dpt[3][8];
    RLcp53_29 = s->dpt[1][8]*ROcp53_27+s->dpt[2][8]*ROcp53_58+ROcp53_88*s->dpt[3][8];
    RLcp53_39 = s->dpt[1][8]*ROcp53_37+s->dpt[2][8]*ROcp53_68+ROcp53_98*s->dpt[3][8];
    OMcp53_19 = OMcp53_18+ROcp53_78*qd[9];
    OMcp53_29 = OMcp53_28+ROcp53_88*qd[9];
    OMcp53_39 = OMcp53_38+ROcp53_98*qd[9];
    ORcp53_19 = OMcp53_28*RLcp53_39-OMcp53_38*RLcp53_29;
    ORcp53_29 = -(OMcp53_18*RLcp53_39-OMcp53_38*RLcp53_19);
    ORcp53_39 = OMcp53_18*RLcp53_29-OMcp53_28*RLcp53_19;
    OPcp53_19 = OPcp53_18+ROcp53_78*qdd[9]+qd[9]*(OMcp53_28*ROcp53_98-OMcp53_38*ROcp53_88);
    OPcp53_29 = OPcp53_28+ROcp53_88*qdd[9]-qd[9]*(OMcp53_18*ROcp53_98-OMcp53_38*ROcp53_78);
    OPcp53_39 = OPcp53_38+ROcp53_98*qdd[9]+qd[9]*(OMcp53_18*ROcp53_88-OMcp53_28*ROcp53_78);
    RLcp53_110 = s->dpt[1][10]*ROcp53_19+s->dpt[2][10]*ROcp53_49+ROcp53_78*s->dpt[3][10];
    RLcp53_210 = s->dpt[1][10]*ROcp53_29+s->dpt[2][10]*ROcp53_59+ROcp53_88*s->dpt[3][10];
    RLcp53_310 = s->dpt[1][10]*ROcp53_39+s->dpt[2][10]*ROcp53_69+ROcp53_98*s->dpt[3][10];
    OMcp53_110 = OMcp53_19+ROcp53_49*qd[10];
    OMcp53_210 = OMcp53_29+ROcp53_59*qd[10];
    OMcp53_310 = OMcp53_39+ROcp53_69*qd[10];
    ORcp53_110 = OMcp53_29*RLcp53_310-OMcp53_39*RLcp53_210;
    ORcp53_210 = -(OMcp53_19*RLcp53_310-OMcp53_39*RLcp53_110);
    ORcp53_310 = OMcp53_19*RLcp53_210-OMcp53_29*RLcp53_110;
    OPcp53_110 = OPcp53_19+ROcp53_49*qdd[10]+qd[10]*(OMcp53_29*ROcp53_69-OMcp53_39*ROcp53_59);
    OPcp53_210 = OPcp53_29+ROcp53_59*qdd[10]-qd[10]*(OMcp53_19*ROcp53_69-OMcp53_39*ROcp53_49);
    OPcp53_310 = OPcp53_39+ROcp53_69*qdd[10]+qd[10]*(OMcp53_19*ROcp53_59-OMcp53_29*ROcp53_49);
    RLcp53_111 = s->dpt[1][12]*ROcp53_110+s->dpt[2][12]*ROcp53_49+ROcp53_710*s->dpt[3][12];
    RLcp53_211 = s->dpt[1][12]*ROcp53_210+s->dpt[2][12]*ROcp53_59+ROcp53_810*s->dpt[3][12];
    RLcp53_311 = s->dpt[1][12]*ROcp53_310+s->dpt[2][12]*ROcp53_69+ROcp53_910*s->dpt[3][12];
    OMcp53_111 = OMcp53_110+ROcp53_110*qd[11];
    OMcp53_211 = OMcp53_210+ROcp53_210*qd[11];
    OMcp53_311 = OMcp53_310+ROcp53_310*qd[11];
    ORcp53_111 = OMcp53_210*RLcp53_311-OMcp53_310*RLcp53_211;
    ORcp53_211 = -(OMcp53_110*RLcp53_311-OMcp53_310*RLcp53_111);
    ORcp53_311 = OMcp53_110*RLcp53_211-OMcp53_210*RLcp53_111;
    OPcp53_111 = OPcp53_110+ROcp53_110*qdd[11]+qd[11]*(OMcp53_210*ROcp53_310-OMcp53_310*ROcp53_210);
    OPcp53_211 = OPcp53_210+ROcp53_210*qdd[11]-qd[11]*(OMcp53_110*ROcp53_310-OMcp53_310*ROcp53_110);
    OPcp53_311 = OPcp53_310+ROcp53_310*qdd[11]+qd[11]*(OMcp53_110*ROcp53_210-OMcp53_210*ROcp53_110);
    RLcp53_112 = s->dpt[1][14]*ROcp53_110+s->dpt[2][14]*ROcp53_411+s->dpt[3][14]*ROcp53_711;
    RLcp53_212 = s->dpt[1][14]*ROcp53_210+s->dpt[2][14]*ROcp53_511+s->dpt[3][14]*ROcp53_811;
    RLcp53_312 = s->dpt[1][14]*ROcp53_310+s->dpt[2][14]*ROcp53_611+s->dpt[3][14]*ROcp53_911;
    OMcp53_112 = OMcp53_111+ROcp53_411*qd[12];
    OMcp53_212 = OMcp53_211+ROcp53_511*qd[12];
    OMcp53_312 = OMcp53_311+ROcp53_611*qd[12];
    ORcp53_112 = OMcp53_211*RLcp53_312-OMcp53_311*RLcp53_212;
    ORcp53_212 = -(OMcp53_111*RLcp53_312-OMcp53_311*RLcp53_112);
    ORcp53_312 = OMcp53_111*RLcp53_212-OMcp53_211*RLcp53_112;
    OPcp53_112 = OPcp53_111+ROcp53_411*qdd[12]+qd[12]*(OMcp53_211*ROcp53_611-OMcp53_311*ROcp53_511);
    OPcp53_212 = OPcp53_211+ROcp53_511*qdd[12]-qd[12]*(OMcp53_111*ROcp53_611-OMcp53_311*ROcp53_411);
    OPcp53_312 = OPcp53_311+ROcp53_611*qdd[12]+qd[12]*(OMcp53_111*ROcp53_511-OMcp53_211*ROcp53_411);
    RLcp53_189 = ROcp53_112*s->dpt[1][19]+ROcp53_411*s->dpt[2][19]+ROcp53_712*s->dpt[3][19];
    RLcp53_289 = ROcp53_212*s->dpt[1][19]+ROcp53_511*s->dpt[2][19]+ROcp53_812*s->dpt[3][19];
    RLcp53_389 = ROcp53_312*s->dpt[1][19]+ROcp53_611*s->dpt[2][19]+ROcp53_912*s->dpt[3][19];
    POcp53_189 = RLcp53_110+RLcp53_111+RLcp53_112+RLcp53_17+RLcp53_18+RLcp53_189+RLcp53_19+q[1];
    POcp53_289 = RLcp53_210+RLcp53_211+RLcp53_212+RLcp53_27+RLcp53_28+RLcp53_289+RLcp53_29+q[2];
    POcp53_389 = RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_37+RLcp53_38+RLcp53_389+RLcp53_39+q[3];
    ORcp53_189 = OMcp53_212*RLcp53_389-OMcp53_312*RLcp53_289;
    ORcp53_289 = -(OMcp53_112*RLcp53_389-OMcp53_312*RLcp53_189);
    ORcp53_389 = OMcp53_112*RLcp53_289-OMcp53_212*RLcp53_189;
    VIcp53_189 = ORcp53_110+ORcp53_111+ORcp53_112+ORcp53_17+ORcp53_18+ORcp53_189+ORcp53_19+qd[1];
    VIcp53_289 = ORcp53_210+ORcp53_211+ORcp53_212+ORcp53_27+ORcp53_28+ORcp53_289+ORcp53_29+qd[2];
    VIcp53_389 = ORcp53_310+ORcp53_311+ORcp53_312+ORcp53_37+ORcp53_38+ORcp53_389+ORcp53_39+qd[3];
    ACcp53_189 = qdd[1]+OMcp53_210*ORcp53_311+OMcp53_211*ORcp53_312+OMcp53_212*ORcp53_389+OMcp53_26*ORcp53_37+OMcp53_27*
 ORcp53_38+OMcp53_28*ORcp53_39+OMcp53_29*ORcp53_310-OMcp53_310*ORcp53_211-OMcp53_311*ORcp53_212-OMcp53_312*ORcp53_289-
 OMcp53_36*ORcp53_27-OMcp53_37*ORcp53_28-OMcp53_38*ORcp53_29-OMcp53_39*ORcp53_210+OPcp53_210*RLcp53_311+OPcp53_211*RLcp53_312
 +OPcp53_212*RLcp53_389+OPcp53_26*RLcp53_37+OPcp53_27*RLcp53_38+OPcp53_28*RLcp53_39+OPcp53_29*RLcp53_310-OPcp53_310*
 RLcp53_211-OPcp53_311*RLcp53_212-OPcp53_312*RLcp53_289-OPcp53_36*RLcp53_27-OPcp53_37*RLcp53_28-OPcp53_38*RLcp53_29-OPcp53_39
 *RLcp53_210;
    ACcp53_289 = qdd[2]-OMcp53_110*ORcp53_311-OMcp53_111*ORcp53_312-OMcp53_112*ORcp53_389-OMcp53_16*ORcp53_37-OMcp53_17*
 ORcp53_38-OMcp53_18*ORcp53_39-OMcp53_19*ORcp53_310+OMcp53_310*ORcp53_111+OMcp53_311*ORcp53_112+OMcp53_312*ORcp53_189+
 OMcp53_36*ORcp53_17+OMcp53_37*ORcp53_18+OMcp53_38*ORcp53_19+OMcp53_39*ORcp53_110-OPcp53_110*RLcp53_311-OPcp53_111*RLcp53_312
 -OPcp53_112*RLcp53_389-OPcp53_16*RLcp53_37-OPcp53_17*RLcp53_38-OPcp53_18*RLcp53_39-OPcp53_19*RLcp53_310+OPcp53_310*
 RLcp53_111+OPcp53_311*RLcp53_112+OPcp53_312*RLcp53_189+OPcp53_36*RLcp53_17+OPcp53_37*RLcp53_18+OPcp53_38*RLcp53_19+OPcp53_39
 *RLcp53_110;
    ACcp53_389 = qdd[3]+OMcp53_110*ORcp53_211+OMcp53_111*ORcp53_212+OMcp53_112*ORcp53_289+OMcp53_16*ORcp53_27+OMcp53_17*
 ORcp53_28+OMcp53_18*ORcp53_29+OMcp53_19*ORcp53_210-OMcp53_210*ORcp53_111-OMcp53_211*ORcp53_112-OMcp53_212*ORcp53_189-
 OMcp53_26*ORcp53_17-OMcp53_27*ORcp53_18-OMcp53_28*ORcp53_19-OMcp53_29*ORcp53_110+OPcp53_110*RLcp53_211+OPcp53_111*RLcp53_212
 +OPcp53_112*RLcp53_289+OPcp53_16*RLcp53_27+OPcp53_17*RLcp53_28+OPcp53_18*RLcp53_29+OPcp53_19*RLcp53_210-OPcp53_210*
 RLcp53_111-OPcp53_211*RLcp53_112-OPcp53_212*RLcp53_189-OPcp53_26*RLcp53_17-OPcp53_27*RLcp53_18-OPcp53_28*RLcp53_19-OPcp53_29
 *RLcp53_110;

// = = Block_1_0_0_54_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp53_189;
    sens->P[2] = POcp53_289;
    sens->P[3] = POcp53_389;
    sens->R[1][1] = ROcp53_112;
    sens->R[1][2] = ROcp53_212;
    sens->R[1][3] = ROcp53_312;
    sens->R[2][1] = ROcp53_411;
    sens->R[2][2] = ROcp53_511;
    sens->R[2][3] = ROcp53_611;
    sens->R[3][1] = ROcp53_712;
    sens->R[3][2] = ROcp53_812;
    sens->R[3][3] = ROcp53_912;
    sens->V[1] = VIcp53_189;
    sens->V[2] = VIcp53_289;
    sens->V[3] = VIcp53_389;
    sens->OM[1] = OMcp53_112;
    sens->OM[2] = OMcp53_212;
    sens->OM[3] = OMcp53_312;
    sens->A[1] = ACcp53_189;
    sens->A[2] = ACcp53_289;
    sens->A[3] = ACcp53_389;
    sens->OMP[1] = OPcp53_112;
    sens->OMP[2] = OPcp53_212;
    sens->OMP[3] = OPcp53_312;
 
// 
break;
case 55:
 


// = = Block_1_0_0_55_0_1 = = 
 
// Sensor Kinematics 


    ROcp54_25 = S4*S5;
    ROcp54_35 = -C4*S5;
    ROcp54_85 = -S4*C5;
    ROcp54_95 = C4*C5;
    ROcp54_16 = C5*C6;
    ROcp54_26 = ROcp54_25*C6+C4*S6;
    ROcp54_36 = ROcp54_35*C6+S4*S6;
    ROcp54_46 = -C5*S6;
    ROcp54_56 = -(ROcp54_25*S6-C4*C6);
    ROcp54_66 = -(ROcp54_35*S6-S4*C6);
    OMcp54_25 = qd[5]*C4;
    OMcp54_35 = qd[5]*S4;
    OMcp54_16 = qd[4]+qd[6]*S5;
    OMcp54_26 = OMcp54_25+ROcp54_85*qd[6];
    OMcp54_36 = OMcp54_35+ROcp54_95*qd[6];
    OPcp54_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp54_26 = ROcp54_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp54_35*S5-ROcp54_95*qd[4]);
    OPcp54_36 = ROcp54_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp54_25*S5-ROcp54_85*qd[4]);

// = = Block_1_0_0_55_0_2 = = 
 
// Sensor Kinematics 


    ROcp54_17 = ROcp54_16*C7-S5*S7;
    ROcp54_27 = ROcp54_26*C7-ROcp54_85*S7;
    ROcp54_37 = ROcp54_36*C7-ROcp54_95*S7;
    ROcp54_77 = ROcp54_16*S7+S5*C7;
    ROcp54_87 = ROcp54_26*S7+ROcp54_85*C7;
    ROcp54_97 = ROcp54_36*S7+ROcp54_95*C7;
    ROcp54_48 = ROcp54_46*C8+ROcp54_77*S8;
    ROcp54_58 = ROcp54_56*C8+ROcp54_87*S8;
    ROcp54_68 = ROcp54_66*C8+ROcp54_97*S8;
    ROcp54_78 = -(ROcp54_46*S8-ROcp54_77*C8);
    ROcp54_88 = -(ROcp54_56*S8-ROcp54_87*C8);
    ROcp54_98 = -(ROcp54_66*S8-ROcp54_97*C8);
    ROcp54_19 = ROcp54_17*C9+ROcp54_48*S9;
    ROcp54_29 = ROcp54_27*C9+ROcp54_58*S9;
    ROcp54_39 = ROcp54_37*C9+ROcp54_68*S9;
    ROcp54_49 = -(ROcp54_17*S9-ROcp54_48*C9);
    ROcp54_59 = -(ROcp54_27*S9-ROcp54_58*C9);
    ROcp54_69 = -(ROcp54_37*S9-ROcp54_68*C9);
    ROcp54_110 = ROcp54_19*C10-ROcp54_78*S10;
    ROcp54_210 = ROcp54_29*C10-ROcp54_88*S10;
    ROcp54_310 = ROcp54_39*C10-ROcp54_98*S10;
    ROcp54_710 = ROcp54_19*S10+ROcp54_78*C10;
    ROcp54_810 = ROcp54_29*S10+ROcp54_88*C10;
    ROcp54_910 = ROcp54_39*S10+ROcp54_98*C10;
    ROcp54_411 = ROcp54_49*C11+ROcp54_710*S11;
    ROcp54_511 = ROcp54_59*C11+ROcp54_810*S11;
    ROcp54_611 = ROcp54_69*C11+ROcp54_910*S11;
    ROcp54_711 = -(ROcp54_49*S11-ROcp54_710*C11);
    ROcp54_811 = -(ROcp54_59*S11-ROcp54_810*C11);
    ROcp54_911 = -(ROcp54_69*S11-ROcp54_910*C11);
    ROcp54_112 = ROcp54_110*C12-ROcp54_711*S12;
    ROcp54_212 = ROcp54_210*C12-ROcp54_811*S12;
    ROcp54_312 = ROcp54_310*C12-ROcp54_911*S12;
    ROcp54_712 = ROcp54_110*S12+ROcp54_711*C12;
    ROcp54_812 = ROcp54_210*S12+ROcp54_811*C12;
    ROcp54_912 = ROcp54_310*S12+ROcp54_911*C12;
    RLcp54_17 = s->dpt[1][1]*ROcp54_16+s->dpt[3][1]*S5+ROcp54_46*s->dpt[2][1];
    RLcp54_27 = s->dpt[1][1]*ROcp54_26+s->dpt[3][1]*ROcp54_85+ROcp54_56*s->dpt[2][1];
    RLcp54_37 = s->dpt[1][1]*ROcp54_36+s->dpt[3][1]*ROcp54_95+ROcp54_66*s->dpt[2][1];
    OMcp54_17 = OMcp54_16+ROcp54_46*qd[7];
    OMcp54_27 = OMcp54_26+ROcp54_56*qd[7];
    OMcp54_37 = OMcp54_36+ROcp54_66*qd[7];
    ORcp54_17 = OMcp54_26*RLcp54_37-OMcp54_36*RLcp54_27;
    ORcp54_27 = -(OMcp54_16*RLcp54_37-OMcp54_36*RLcp54_17);
    ORcp54_37 = OMcp54_16*RLcp54_27-OMcp54_26*RLcp54_17;
    OPcp54_17 = OPcp54_16+ROcp54_46*qdd[7]+qd[7]*(OMcp54_26*ROcp54_66-OMcp54_36*ROcp54_56);
    OPcp54_27 = OPcp54_26+ROcp54_56*qdd[7]-qd[7]*(OMcp54_16*ROcp54_66-OMcp54_36*ROcp54_46);
    OPcp54_37 = OPcp54_36+ROcp54_66*qdd[7]+qd[7]*(OMcp54_16*ROcp54_56-OMcp54_26*ROcp54_46);
    RLcp54_18 = s->dpt[1][6]*ROcp54_17+s->dpt[3][6]*ROcp54_77+ROcp54_46*s->dpt[2][6];
    RLcp54_28 = s->dpt[1][6]*ROcp54_27+s->dpt[3][6]*ROcp54_87+ROcp54_56*s->dpt[2][6];
    RLcp54_38 = s->dpt[1][6]*ROcp54_37+s->dpt[3][6]*ROcp54_97+ROcp54_66*s->dpt[2][6];
    OMcp54_18 = OMcp54_17+ROcp54_17*qd[8];
    OMcp54_28 = OMcp54_27+ROcp54_27*qd[8];
    OMcp54_38 = OMcp54_37+ROcp54_37*qd[8];
    ORcp54_18 = OMcp54_27*RLcp54_38-OMcp54_37*RLcp54_28;
    ORcp54_28 = -(OMcp54_17*RLcp54_38-OMcp54_37*RLcp54_18);
    ORcp54_38 = OMcp54_17*RLcp54_28-OMcp54_27*RLcp54_18;
    OPcp54_18 = OPcp54_17+ROcp54_17*qdd[8]+qd[8]*(OMcp54_27*ROcp54_37-OMcp54_37*ROcp54_27);
    OPcp54_28 = OPcp54_27+ROcp54_27*qdd[8]-qd[8]*(OMcp54_17*ROcp54_37-OMcp54_37*ROcp54_17);
    OPcp54_38 = OPcp54_37+ROcp54_37*qdd[8]+qd[8]*(OMcp54_17*ROcp54_27-OMcp54_27*ROcp54_17);
    RLcp54_19 = s->dpt[1][8]*ROcp54_17+s->dpt[2][8]*ROcp54_48+ROcp54_78*s->dpt[3][8];
    RLcp54_29 = s->dpt[1][8]*ROcp54_27+s->dpt[2][8]*ROcp54_58+ROcp54_88*s->dpt[3][8];
    RLcp54_39 = s->dpt[1][8]*ROcp54_37+s->dpt[2][8]*ROcp54_68+ROcp54_98*s->dpt[3][8];
    OMcp54_19 = OMcp54_18+ROcp54_78*qd[9];
    OMcp54_29 = OMcp54_28+ROcp54_88*qd[9];
    OMcp54_39 = OMcp54_38+ROcp54_98*qd[9];
    ORcp54_19 = OMcp54_28*RLcp54_39-OMcp54_38*RLcp54_29;
    ORcp54_29 = -(OMcp54_18*RLcp54_39-OMcp54_38*RLcp54_19);
    ORcp54_39 = OMcp54_18*RLcp54_29-OMcp54_28*RLcp54_19;
    OPcp54_19 = OPcp54_18+ROcp54_78*qdd[9]+qd[9]*(OMcp54_28*ROcp54_98-OMcp54_38*ROcp54_88);
    OPcp54_29 = OPcp54_28+ROcp54_88*qdd[9]-qd[9]*(OMcp54_18*ROcp54_98-OMcp54_38*ROcp54_78);
    OPcp54_39 = OPcp54_38+ROcp54_98*qdd[9]+qd[9]*(OMcp54_18*ROcp54_88-OMcp54_28*ROcp54_78);
    RLcp54_110 = s->dpt[1][10]*ROcp54_19+s->dpt[2][10]*ROcp54_49+ROcp54_78*s->dpt[3][10];
    RLcp54_210 = s->dpt[1][10]*ROcp54_29+s->dpt[2][10]*ROcp54_59+ROcp54_88*s->dpt[3][10];
    RLcp54_310 = s->dpt[1][10]*ROcp54_39+s->dpt[2][10]*ROcp54_69+ROcp54_98*s->dpt[3][10];
    OMcp54_110 = OMcp54_19+ROcp54_49*qd[10];
    OMcp54_210 = OMcp54_29+ROcp54_59*qd[10];
    OMcp54_310 = OMcp54_39+ROcp54_69*qd[10];
    ORcp54_110 = OMcp54_29*RLcp54_310-OMcp54_39*RLcp54_210;
    ORcp54_210 = -(OMcp54_19*RLcp54_310-OMcp54_39*RLcp54_110);
    ORcp54_310 = OMcp54_19*RLcp54_210-OMcp54_29*RLcp54_110;
    OPcp54_110 = OPcp54_19+ROcp54_49*qdd[10]+qd[10]*(OMcp54_29*ROcp54_69-OMcp54_39*ROcp54_59);
    OPcp54_210 = OPcp54_29+ROcp54_59*qdd[10]-qd[10]*(OMcp54_19*ROcp54_69-OMcp54_39*ROcp54_49);
    OPcp54_310 = OPcp54_39+ROcp54_69*qdd[10]+qd[10]*(OMcp54_19*ROcp54_59-OMcp54_29*ROcp54_49);
    RLcp54_111 = s->dpt[1][12]*ROcp54_110+s->dpt[2][12]*ROcp54_49+ROcp54_710*s->dpt[3][12];
    RLcp54_211 = s->dpt[1][12]*ROcp54_210+s->dpt[2][12]*ROcp54_59+ROcp54_810*s->dpt[3][12];
    RLcp54_311 = s->dpt[1][12]*ROcp54_310+s->dpt[2][12]*ROcp54_69+ROcp54_910*s->dpt[3][12];
    OMcp54_111 = OMcp54_110+ROcp54_110*qd[11];
    OMcp54_211 = OMcp54_210+ROcp54_210*qd[11];
    OMcp54_311 = OMcp54_310+ROcp54_310*qd[11];
    ORcp54_111 = OMcp54_210*RLcp54_311-OMcp54_310*RLcp54_211;
    ORcp54_211 = -(OMcp54_110*RLcp54_311-OMcp54_310*RLcp54_111);
    ORcp54_311 = OMcp54_110*RLcp54_211-OMcp54_210*RLcp54_111;
    OPcp54_111 = OPcp54_110+ROcp54_110*qdd[11]+qd[11]*(OMcp54_210*ROcp54_310-OMcp54_310*ROcp54_210);
    OPcp54_211 = OPcp54_210+ROcp54_210*qdd[11]-qd[11]*(OMcp54_110*ROcp54_310-OMcp54_310*ROcp54_110);
    OPcp54_311 = OPcp54_310+ROcp54_310*qdd[11]+qd[11]*(OMcp54_110*ROcp54_210-OMcp54_210*ROcp54_110);
    RLcp54_112 = s->dpt[1][14]*ROcp54_110+s->dpt[2][14]*ROcp54_411+s->dpt[3][14]*ROcp54_711;
    RLcp54_212 = s->dpt[1][14]*ROcp54_210+s->dpt[2][14]*ROcp54_511+s->dpt[3][14]*ROcp54_811;
    RLcp54_312 = s->dpt[1][14]*ROcp54_310+s->dpt[2][14]*ROcp54_611+s->dpt[3][14]*ROcp54_911;
    OMcp54_112 = OMcp54_111+ROcp54_411*qd[12];
    OMcp54_212 = OMcp54_211+ROcp54_511*qd[12];
    OMcp54_312 = OMcp54_311+ROcp54_611*qd[12];
    ORcp54_112 = OMcp54_211*RLcp54_312-OMcp54_311*RLcp54_212;
    ORcp54_212 = -(OMcp54_111*RLcp54_312-OMcp54_311*RLcp54_112);
    ORcp54_312 = OMcp54_111*RLcp54_212-OMcp54_211*RLcp54_112;
    OPcp54_112 = OPcp54_111+ROcp54_411*qdd[12]+qd[12]*(OMcp54_211*ROcp54_611-OMcp54_311*ROcp54_511);
    OPcp54_212 = OPcp54_211+ROcp54_511*qdd[12]-qd[12]*(OMcp54_111*ROcp54_611-OMcp54_311*ROcp54_411);
    OPcp54_312 = OPcp54_311+ROcp54_611*qdd[12]+qd[12]*(OMcp54_111*ROcp54_511-OMcp54_211*ROcp54_411);
    RLcp54_190 = ROcp54_112*s->dpt[1][20]+ROcp54_411*s->dpt[2][20]+ROcp54_712*s->dpt[3][20];
    RLcp54_290 = ROcp54_212*s->dpt[1][20]+ROcp54_511*s->dpt[2][20]+ROcp54_812*s->dpt[3][20];
    RLcp54_390 = ROcp54_312*s->dpt[1][20]+ROcp54_611*s->dpt[2][20]+ROcp54_912*s->dpt[3][20];
    POcp54_190 = RLcp54_110+RLcp54_111+RLcp54_112+RLcp54_17+RLcp54_18+RLcp54_19+RLcp54_190+q[1];
    POcp54_290 = RLcp54_210+RLcp54_211+RLcp54_212+RLcp54_27+RLcp54_28+RLcp54_29+RLcp54_290+q[2];
    POcp54_390 = RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_37+RLcp54_38+RLcp54_39+RLcp54_390+q[3];
    ORcp54_190 = OMcp54_212*RLcp54_390-OMcp54_312*RLcp54_290;
    ORcp54_290 = -(OMcp54_112*RLcp54_390-OMcp54_312*RLcp54_190);
    ORcp54_390 = OMcp54_112*RLcp54_290-OMcp54_212*RLcp54_190;
    VIcp54_190 = ORcp54_110+ORcp54_111+ORcp54_112+ORcp54_17+ORcp54_18+ORcp54_19+ORcp54_190+qd[1];
    VIcp54_290 = ORcp54_210+ORcp54_211+ORcp54_212+ORcp54_27+ORcp54_28+ORcp54_29+ORcp54_290+qd[2];
    VIcp54_390 = ORcp54_310+ORcp54_311+ORcp54_312+ORcp54_37+ORcp54_38+ORcp54_39+ORcp54_390+qd[3];
    ACcp54_190 = qdd[1]+OMcp54_210*ORcp54_311+OMcp54_211*ORcp54_312+OMcp54_212*ORcp54_390+OMcp54_26*ORcp54_37+OMcp54_27*
 ORcp54_38+OMcp54_28*ORcp54_39+OMcp54_29*ORcp54_310-OMcp54_310*ORcp54_211-OMcp54_311*ORcp54_212-OMcp54_312*ORcp54_290-
 OMcp54_36*ORcp54_27-OMcp54_37*ORcp54_28-OMcp54_38*ORcp54_29-OMcp54_39*ORcp54_210+OPcp54_210*RLcp54_311+OPcp54_211*RLcp54_312
 +OPcp54_212*RLcp54_390+OPcp54_26*RLcp54_37+OPcp54_27*RLcp54_38+OPcp54_28*RLcp54_39+OPcp54_29*RLcp54_310-OPcp54_310*
 RLcp54_211-OPcp54_311*RLcp54_212-OPcp54_312*RLcp54_290-OPcp54_36*RLcp54_27-OPcp54_37*RLcp54_28-OPcp54_38*RLcp54_29-OPcp54_39
 *RLcp54_210;
    ACcp54_290 = qdd[2]-OMcp54_110*ORcp54_311-OMcp54_111*ORcp54_312-OMcp54_112*ORcp54_390-OMcp54_16*ORcp54_37-OMcp54_17*
 ORcp54_38-OMcp54_18*ORcp54_39-OMcp54_19*ORcp54_310+OMcp54_310*ORcp54_111+OMcp54_311*ORcp54_112+OMcp54_312*ORcp54_190+
 OMcp54_36*ORcp54_17+OMcp54_37*ORcp54_18+OMcp54_38*ORcp54_19+OMcp54_39*ORcp54_110-OPcp54_110*RLcp54_311-OPcp54_111*RLcp54_312
 -OPcp54_112*RLcp54_390-OPcp54_16*RLcp54_37-OPcp54_17*RLcp54_38-OPcp54_18*RLcp54_39-OPcp54_19*RLcp54_310+OPcp54_310*
 RLcp54_111+OPcp54_311*RLcp54_112+OPcp54_312*RLcp54_190+OPcp54_36*RLcp54_17+OPcp54_37*RLcp54_18+OPcp54_38*RLcp54_19+OPcp54_39
 *RLcp54_110;
    ACcp54_390 = qdd[3]+OMcp54_110*ORcp54_211+OMcp54_111*ORcp54_212+OMcp54_112*ORcp54_290+OMcp54_16*ORcp54_27+OMcp54_17*
 ORcp54_28+OMcp54_18*ORcp54_29+OMcp54_19*ORcp54_210-OMcp54_210*ORcp54_111-OMcp54_211*ORcp54_112-OMcp54_212*ORcp54_190-
 OMcp54_26*ORcp54_17-OMcp54_27*ORcp54_18-OMcp54_28*ORcp54_19-OMcp54_29*ORcp54_110+OPcp54_110*RLcp54_211+OPcp54_111*RLcp54_212
 +OPcp54_112*RLcp54_290+OPcp54_16*RLcp54_27+OPcp54_17*RLcp54_28+OPcp54_18*RLcp54_29+OPcp54_19*RLcp54_210-OPcp54_210*
 RLcp54_111-OPcp54_211*RLcp54_112-OPcp54_212*RLcp54_190-OPcp54_26*RLcp54_17-OPcp54_27*RLcp54_18-OPcp54_28*RLcp54_19-OPcp54_29
 *RLcp54_110;

// = = Block_1_0_0_55_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp54_190;
    sens->P[2] = POcp54_290;
    sens->P[3] = POcp54_390;
    sens->R[1][1] = ROcp54_112;
    sens->R[1][2] = ROcp54_212;
    sens->R[1][3] = ROcp54_312;
    sens->R[2][1] = ROcp54_411;
    sens->R[2][2] = ROcp54_511;
    sens->R[2][3] = ROcp54_611;
    sens->R[3][1] = ROcp54_712;
    sens->R[3][2] = ROcp54_812;
    sens->R[3][3] = ROcp54_912;
    sens->V[1] = VIcp54_190;
    sens->V[2] = VIcp54_290;
    sens->V[3] = VIcp54_390;
    sens->OM[1] = OMcp54_112;
    sens->OM[2] = OMcp54_212;
    sens->OM[3] = OMcp54_312;
    sens->A[1] = ACcp54_190;
    sens->A[2] = ACcp54_290;
    sens->A[3] = ACcp54_390;
    sens->OMP[1] = OPcp54_112;
    sens->OMP[2] = OPcp54_212;
    sens->OMP[3] = OPcp54_312;
 
// 
break;
case 56:
 


// = = Block_1_0_0_56_0_1 = = 
 
// Sensor Kinematics 


    ROcp55_25 = S4*S5;
    ROcp55_35 = -C4*S5;
    ROcp55_85 = -S4*C5;
    ROcp55_95 = C4*C5;
    ROcp55_16 = C5*C6;
    ROcp55_26 = ROcp55_25*C6+C4*S6;
    ROcp55_36 = ROcp55_35*C6+S4*S6;
    ROcp55_46 = -C5*S6;
    ROcp55_56 = -(ROcp55_25*S6-C4*C6);
    ROcp55_66 = -(ROcp55_35*S6-S4*C6);
    OMcp55_25 = qd[5]*C4;
    OMcp55_35 = qd[5]*S4;
    OMcp55_16 = qd[4]+qd[6]*S5;
    OMcp55_26 = OMcp55_25+ROcp55_85*qd[6];
    OMcp55_36 = OMcp55_35+ROcp55_95*qd[6];
    OPcp55_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp55_26 = ROcp55_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp55_35*S5-ROcp55_95*qd[4]);
    OPcp55_36 = ROcp55_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp55_25*S5-ROcp55_85*qd[4]);

// = = Block_1_0_0_56_0_2 = = 
 
// Sensor Kinematics 


    ROcp55_17 = ROcp55_16*C7-S5*S7;
    ROcp55_27 = ROcp55_26*C7-ROcp55_85*S7;
    ROcp55_37 = ROcp55_36*C7-ROcp55_95*S7;
    ROcp55_77 = ROcp55_16*S7+S5*C7;
    ROcp55_87 = ROcp55_26*S7+ROcp55_85*C7;
    ROcp55_97 = ROcp55_36*S7+ROcp55_95*C7;
    ROcp55_48 = ROcp55_46*C8+ROcp55_77*S8;
    ROcp55_58 = ROcp55_56*C8+ROcp55_87*S8;
    ROcp55_68 = ROcp55_66*C8+ROcp55_97*S8;
    ROcp55_78 = -(ROcp55_46*S8-ROcp55_77*C8);
    ROcp55_88 = -(ROcp55_56*S8-ROcp55_87*C8);
    ROcp55_98 = -(ROcp55_66*S8-ROcp55_97*C8);
    ROcp55_19 = ROcp55_17*C9+ROcp55_48*S9;
    ROcp55_29 = ROcp55_27*C9+ROcp55_58*S9;
    ROcp55_39 = ROcp55_37*C9+ROcp55_68*S9;
    ROcp55_49 = -(ROcp55_17*S9-ROcp55_48*C9);
    ROcp55_59 = -(ROcp55_27*S9-ROcp55_58*C9);
    ROcp55_69 = -(ROcp55_37*S9-ROcp55_68*C9);
    ROcp55_110 = ROcp55_19*C10-ROcp55_78*S10;
    ROcp55_210 = ROcp55_29*C10-ROcp55_88*S10;
    ROcp55_310 = ROcp55_39*C10-ROcp55_98*S10;
    ROcp55_710 = ROcp55_19*S10+ROcp55_78*C10;
    ROcp55_810 = ROcp55_29*S10+ROcp55_88*C10;
    ROcp55_910 = ROcp55_39*S10+ROcp55_98*C10;
    ROcp55_411 = ROcp55_49*C11+ROcp55_710*S11;
    ROcp55_511 = ROcp55_59*C11+ROcp55_810*S11;
    ROcp55_611 = ROcp55_69*C11+ROcp55_910*S11;
    ROcp55_711 = -(ROcp55_49*S11-ROcp55_710*C11);
    ROcp55_811 = -(ROcp55_59*S11-ROcp55_810*C11);
    ROcp55_911 = -(ROcp55_69*S11-ROcp55_910*C11);
    ROcp55_112 = ROcp55_110*C12-ROcp55_711*S12;
    ROcp55_212 = ROcp55_210*C12-ROcp55_811*S12;
    ROcp55_312 = ROcp55_310*C12-ROcp55_911*S12;
    ROcp55_712 = ROcp55_110*S12+ROcp55_711*C12;
    ROcp55_812 = ROcp55_210*S12+ROcp55_811*C12;
    ROcp55_912 = ROcp55_310*S12+ROcp55_911*C12;
    RLcp55_17 = s->dpt[1][1]*ROcp55_16+s->dpt[3][1]*S5+ROcp55_46*s->dpt[2][1];
    RLcp55_27 = s->dpt[1][1]*ROcp55_26+s->dpt[3][1]*ROcp55_85+ROcp55_56*s->dpt[2][1];
    RLcp55_37 = s->dpt[1][1]*ROcp55_36+s->dpt[3][1]*ROcp55_95+ROcp55_66*s->dpt[2][1];
    OMcp55_17 = OMcp55_16+ROcp55_46*qd[7];
    OMcp55_27 = OMcp55_26+ROcp55_56*qd[7];
    OMcp55_37 = OMcp55_36+ROcp55_66*qd[7];
    ORcp55_17 = OMcp55_26*RLcp55_37-OMcp55_36*RLcp55_27;
    ORcp55_27 = -(OMcp55_16*RLcp55_37-OMcp55_36*RLcp55_17);
    ORcp55_37 = OMcp55_16*RLcp55_27-OMcp55_26*RLcp55_17;
    OPcp55_17 = OPcp55_16+ROcp55_46*qdd[7]+qd[7]*(OMcp55_26*ROcp55_66-OMcp55_36*ROcp55_56);
    OPcp55_27 = OPcp55_26+ROcp55_56*qdd[7]-qd[7]*(OMcp55_16*ROcp55_66-OMcp55_36*ROcp55_46);
    OPcp55_37 = OPcp55_36+ROcp55_66*qdd[7]+qd[7]*(OMcp55_16*ROcp55_56-OMcp55_26*ROcp55_46);
    RLcp55_18 = s->dpt[1][6]*ROcp55_17+s->dpt[3][6]*ROcp55_77+ROcp55_46*s->dpt[2][6];
    RLcp55_28 = s->dpt[1][6]*ROcp55_27+s->dpt[3][6]*ROcp55_87+ROcp55_56*s->dpt[2][6];
    RLcp55_38 = s->dpt[1][6]*ROcp55_37+s->dpt[3][6]*ROcp55_97+ROcp55_66*s->dpt[2][6];
    OMcp55_18 = OMcp55_17+ROcp55_17*qd[8];
    OMcp55_28 = OMcp55_27+ROcp55_27*qd[8];
    OMcp55_38 = OMcp55_37+ROcp55_37*qd[8];
    ORcp55_18 = OMcp55_27*RLcp55_38-OMcp55_37*RLcp55_28;
    ORcp55_28 = -(OMcp55_17*RLcp55_38-OMcp55_37*RLcp55_18);
    ORcp55_38 = OMcp55_17*RLcp55_28-OMcp55_27*RLcp55_18;
    OPcp55_18 = OPcp55_17+ROcp55_17*qdd[8]+qd[8]*(OMcp55_27*ROcp55_37-OMcp55_37*ROcp55_27);
    OPcp55_28 = OPcp55_27+ROcp55_27*qdd[8]-qd[8]*(OMcp55_17*ROcp55_37-OMcp55_37*ROcp55_17);
    OPcp55_38 = OPcp55_37+ROcp55_37*qdd[8]+qd[8]*(OMcp55_17*ROcp55_27-OMcp55_27*ROcp55_17);
    RLcp55_19 = s->dpt[1][8]*ROcp55_17+s->dpt[2][8]*ROcp55_48+ROcp55_78*s->dpt[3][8];
    RLcp55_29 = s->dpt[1][8]*ROcp55_27+s->dpt[2][8]*ROcp55_58+ROcp55_88*s->dpt[3][8];
    RLcp55_39 = s->dpt[1][8]*ROcp55_37+s->dpt[2][8]*ROcp55_68+ROcp55_98*s->dpt[3][8];
    OMcp55_19 = OMcp55_18+ROcp55_78*qd[9];
    OMcp55_29 = OMcp55_28+ROcp55_88*qd[9];
    OMcp55_39 = OMcp55_38+ROcp55_98*qd[9];
    ORcp55_19 = OMcp55_28*RLcp55_39-OMcp55_38*RLcp55_29;
    ORcp55_29 = -(OMcp55_18*RLcp55_39-OMcp55_38*RLcp55_19);
    ORcp55_39 = OMcp55_18*RLcp55_29-OMcp55_28*RLcp55_19;
    OPcp55_19 = OPcp55_18+ROcp55_78*qdd[9]+qd[9]*(OMcp55_28*ROcp55_98-OMcp55_38*ROcp55_88);
    OPcp55_29 = OPcp55_28+ROcp55_88*qdd[9]-qd[9]*(OMcp55_18*ROcp55_98-OMcp55_38*ROcp55_78);
    OPcp55_39 = OPcp55_38+ROcp55_98*qdd[9]+qd[9]*(OMcp55_18*ROcp55_88-OMcp55_28*ROcp55_78);
    RLcp55_110 = s->dpt[1][10]*ROcp55_19+s->dpt[2][10]*ROcp55_49+ROcp55_78*s->dpt[3][10];
    RLcp55_210 = s->dpt[1][10]*ROcp55_29+s->dpt[2][10]*ROcp55_59+ROcp55_88*s->dpt[3][10];
    RLcp55_310 = s->dpt[1][10]*ROcp55_39+s->dpt[2][10]*ROcp55_69+ROcp55_98*s->dpt[3][10];
    OMcp55_110 = OMcp55_19+ROcp55_49*qd[10];
    OMcp55_210 = OMcp55_29+ROcp55_59*qd[10];
    OMcp55_310 = OMcp55_39+ROcp55_69*qd[10];
    ORcp55_110 = OMcp55_29*RLcp55_310-OMcp55_39*RLcp55_210;
    ORcp55_210 = -(OMcp55_19*RLcp55_310-OMcp55_39*RLcp55_110);
    ORcp55_310 = OMcp55_19*RLcp55_210-OMcp55_29*RLcp55_110;
    OPcp55_110 = OPcp55_19+ROcp55_49*qdd[10]+qd[10]*(OMcp55_29*ROcp55_69-OMcp55_39*ROcp55_59);
    OPcp55_210 = OPcp55_29+ROcp55_59*qdd[10]-qd[10]*(OMcp55_19*ROcp55_69-OMcp55_39*ROcp55_49);
    OPcp55_310 = OPcp55_39+ROcp55_69*qdd[10]+qd[10]*(OMcp55_19*ROcp55_59-OMcp55_29*ROcp55_49);
    RLcp55_111 = s->dpt[1][12]*ROcp55_110+s->dpt[2][12]*ROcp55_49+ROcp55_710*s->dpt[3][12];
    RLcp55_211 = s->dpt[1][12]*ROcp55_210+s->dpt[2][12]*ROcp55_59+ROcp55_810*s->dpt[3][12];
    RLcp55_311 = s->dpt[1][12]*ROcp55_310+s->dpt[2][12]*ROcp55_69+ROcp55_910*s->dpt[3][12];
    OMcp55_111 = OMcp55_110+ROcp55_110*qd[11];
    OMcp55_211 = OMcp55_210+ROcp55_210*qd[11];
    OMcp55_311 = OMcp55_310+ROcp55_310*qd[11];
    ORcp55_111 = OMcp55_210*RLcp55_311-OMcp55_310*RLcp55_211;
    ORcp55_211 = -(OMcp55_110*RLcp55_311-OMcp55_310*RLcp55_111);
    ORcp55_311 = OMcp55_110*RLcp55_211-OMcp55_210*RLcp55_111;
    OPcp55_111 = OPcp55_110+ROcp55_110*qdd[11]+qd[11]*(OMcp55_210*ROcp55_310-OMcp55_310*ROcp55_210);
    OPcp55_211 = OPcp55_210+ROcp55_210*qdd[11]-qd[11]*(OMcp55_110*ROcp55_310-OMcp55_310*ROcp55_110);
    OPcp55_311 = OPcp55_310+ROcp55_310*qdd[11]+qd[11]*(OMcp55_110*ROcp55_210-OMcp55_210*ROcp55_110);
    RLcp55_112 = s->dpt[1][14]*ROcp55_110+s->dpt[2][14]*ROcp55_411+s->dpt[3][14]*ROcp55_711;
    RLcp55_212 = s->dpt[1][14]*ROcp55_210+s->dpt[2][14]*ROcp55_511+s->dpt[3][14]*ROcp55_811;
    RLcp55_312 = s->dpt[1][14]*ROcp55_310+s->dpt[2][14]*ROcp55_611+s->dpt[3][14]*ROcp55_911;
    OMcp55_112 = OMcp55_111+ROcp55_411*qd[12];
    OMcp55_212 = OMcp55_211+ROcp55_511*qd[12];
    OMcp55_312 = OMcp55_311+ROcp55_611*qd[12];
    ORcp55_112 = OMcp55_211*RLcp55_312-OMcp55_311*RLcp55_212;
    ORcp55_212 = -(OMcp55_111*RLcp55_312-OMcp55_311*RLcp55_112);
    ORcp55_312 = OMcp55_111*RLcp55_212-OMcp55_211*RLcp55_112;
    OPcp55_112 = OPcp55_111+ROcp55_411*qdd[12]+qd[12]*(OMcp55_211*ROcp55_611-OMcp55_311*ROcp55_511);
    OPcp55_212 = OPcp55_211+ROcp55_511*qdd[12]-qd[12]*(OMcp55_111*ROcp55_611-OMcp55_311*ROcp55_411);
    OPcp55_312 = OPcp55_311+ROcp55_611*qdd[12]+qd[12]*(OMcp55_111*ROcp55_511-OMcp55_211*ROcp55_411);
    RLcp55_191 = ROcp55_112*s->dpt[1][21]+ROcp55_411*s->dpt[2][21]+ROcp55_712*s->dpt[3][21];
    RLcp55_291 = ROcp55_212*s->dpt[1][21]+ROcp55_511*s->dpt[2][21]+ROcp55_812*s->dpt[3][21];
    RLcp55_391 = ROcp55_312*s->dpt[1][21]+ROcp55_611*s->dpt[2][21]+ROcp55_912*s->dpt[3][21];
    POcp55_191 = RLcp55_110+RLcp55_111+RLcp55_112+RLcp55_17+RLcp55_18+RLcp55_19+RLcp55_191+q[1];
    POcp55_291 = RLcp55_210+RLcp55_211+RLcp55_212+RLcp55_27+RLcp55_28+RLcp55_29+RLcp55_291+q[2];
    POcp55_391 = RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_37+RLcp55_38+RLcp55_39+RLcp55_391+q[3];
    ORcp55_191 = OMcp55_212*RLcp55_391-OMcp55_312*RLcp55_291;
    ORcp55_291 = -(OMcp55_112*RLcp55_391-OMcp55_312*RLcp55_191);
    ORcp55_391 = OMcp55_112*RLcp55_291-OMcp55_212*RLcp55_191;
    VIcp55_191 = ORcp55_110+ORcp55_111+ORcp55_112+ORcp55_17+ORcp55_18+ORcp55_19+ORcp55_191+qd[1];
    VIcp55_291 = ORcp55_210+ORcp55_211+ORcp55_212+ORcp55_27+ORcp55_28+ORcp55_29+ORcp55_291+qd[2];
    VIcp55_391 = ORcp55_310+ORcp55_311+ORcp55_312+ORcp55_37+ORcp55_38+ORcp55_39+ORcp55_391+qd[3];
    ACcp55_191 = qdd[1]+OMcp55_210*ORcp55_311+OMcp55_211*ORcp55_312+OMcp55_212*ORcp55_391+OMcp55_26*ORcp55_37+OMcp55_27*
 ORcp55_38+OMcp55_28*ORcp55_39+OMcp55_29*ORcp55_310-OMcp55_310*ORcp55_211-OMcp55_311*ORcp55_212-OMcp55_312*ORcp55_291-
 OMcp55_36*ORcp55_27-OMcp55_37*ORcp55_28-OMcp55_38*ORcp55_29-OMcp55_39*ORcp55_210+OPcp55_210*RLcp55_311+OPcp55_211*RLcp55_312
 +OPcp55_212*RLcp55_391+OPcp55_26*RLcp55_37+OPcp55_27*RLcp55_38+OPcp55_28*RLcp55_39+OPcp55_29*RLcp55_310-OPcp55_310*
 RLcp55_211-OPcp55_311*RLcp55_212-OPcp55_312*RLcp55_291-OPcp55_36*RLcp55_27-OPcp55_37*RLcp55_28-OPcp55_38*RLcp55_29-OPcp55_39
 *RLcp55_210;
    ACcp55_291 = qdd[2]-OMcp55_110*ORcp55_311-OMcp55_111*ORcp55_312-OMcp55_112*ORcp55_391-OMcp55_16*ORcp55_37-OMcp55_17*
 ORcp55_38-OMcp55_18*ORcp55_39-OMcp55_19*ORcp55_310+OMcp55_310*ORcp55_111+OMcp55_311*ORcp55_112+OMcp55_312*ORcp55_191+
 OMcp55_36*ORcp55_17+OMcp55_37*ORcp55_18+OMcp55_38*ORcp55_19+OMcp55_39*ORcp55_110-OPcp55_110*RLcp55_311-OPcp55_111*RLcp55_312
 -OPcp55_112*RLcp55_391-OPcp55_16*RLcp55_37-OPcp55_17*RLcp55_38-OPcp55_18*RLcp55_39-OPcp55_19*RLcp55_310+OPcp55_310*
 RLcp55_111+OPcp55_311*RLcp55_112+OPcp55_312*RLcp55_191+OPcp55_36*RLcp55_17+OPcp55_37*RLcp55_18+OPcp55_38*RLcp55_19+OPcp55_39
 *RLcp55_110;
    ACcp55_391 = qdd[3]+OMcp55_110*ORcp55_211+OMcp55_111*ORcp55_212+OMcp55_112*ORcp55_291+OMcp55_16*ORcp55_27+OMcp55_17*
 ORcp55_28+OMcp55_18*ORcp55_29+OMcp55_19*ORcp55_210-OMcp55_210*ORcp55_111-OMcp55_211*ORcp55_112-OMcp55_212*ORcp55_191-
 OMcp55_26*ORcp55_17-OMcp55_27*ORcp55_18-OMcp55_28*ORcp55_19-OMcp55_29*ORcp55_110+OPcp55_110*RLcp55_211+OPcp55_111*RLcp55_212
 +OPcp55_112*RLcp55_291+OPcp55_16*RLcp55_27+OPcp55_17*RLcp55_28+OPcp55_18*RLcp55_29+OPcp55_19*RLcp55_210-OPcp55_210*
 RLcp55_111-OPcp55_211*RLcp55_112-OPcp55_212*RLcp55_191-OPcp55_26*RLcp55_17-OPcp55_27*RLcp55_18-OPcp55_28*RLcp55_19-OPcp55_29
 *RLcp55_110;

// = = Block_1_0_0_56_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp55_191;
    sens->P[2] = POcp55_291;
    sens->P[3] = POcp55_391;
    sens->R[1][1] = ROcp55_112;
    sens->R[1][2] = ROcp55_212;
    sens->R[1][3] = ROcp55_312;
    sens->R[2][1] = ROcp55_411;
    sens->R[2][2] = ROcp55_511;
    sens->R[2][3] = ROcp55_611;
    sens->R[3][1] = ROcp55_712;
    sens->R[3][2] = ROcp55_812;
    sens->R[3][3] = ROcp55_912;
    sens->V[1] = VIcp55_191;
    sens->V[2] = VIcp55_291;
    sens->V[3] = VIcp55_391;
    sens->OM[1] = OMcp55_112;
    sens->OM[2] = OMcp55_212;
    sens->OM[3] = OMcp55_312;
    sens->A[1] = ACcp55_191;
    sens->A[2] = ACcp55_291;
    sens->A[3] = ACcp55_391;
    sens->OMP[1] = OPcp55_112;
    sens->OMP[2] = OPcp55_212;
    sens->OMP[3] = OPcp55_312;
 
// 
break;
case 57:
 


// = = Block_1_0_0_57_0_1 = = 
 
// Sensor Kinematics 


    ROcp56_25 = S4*S5;
    ROcp56_35 = -C4*S5;
    ROcp56_85 = -S4*C5;
    ROcp56_95 = C4*C5;
    ROcp56_16 = C5*C6;
    ROcp56_26 = ROcp56_25*C6+C4*S6;
    ROcp56_36 = ROcp56_35*C6+S4*S6;
    ROcp56_46 = -C5*S6;
    ROcp56_56 = -(ROcp56_25*S6-C4*C6);
    ROcp56_66 = -(ROcp56_35*S6-S4*C6);
    OMcp56_25 = qd[5]*C4;
    OMcp56_35 = qd[5]*S4;
    OMcp56_16 = qd[4]+qd[6]*S5;
    OMcp56_26 = OMcp56_25+ROcp56_85*qd[6];
    OMcp56_36 = OMcp56_35+ROcp56_95*qd[6];
    OPcp56_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp56_26 = ROcp56_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp56_35*S5-ROcp56_95*qd[4]);
    OPcp56_36 = ROcp56_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp56_25*S5-ROcp56_85*qd[4]);

// = = Block_1_0_0_57_0_3 = = 
 
// Sensor Kinematics 


    ROcp56_113 = ROcp56_16*C13-S13*S5;
    ROcp56_213 = ROcp56_26*C13-ROcp56_85*S13;
    ROcp56_313 = ROcp56_36*C13-ROcp56_95*S13;
    ROcp56_713 = ROcp56_16*S13+C13*S5;
    ROcp56_813 = ROcp56_26*S13+ROcp56_85*C13;
    ROcp56_913 = ROcp56_36*S13+ROcp56_95*C13;
    ROcp56_414 = ROcp56_46*C14+ROcp56_713*S14;
    ROcp56_514 = ROcp56_56*C14+ROcp56_813*S14;
    ROcp56_614 = ROcp56_66*C14+ROcp56_913*S14;
    ROcp56_714 = -(ROcp56_46*S14-ROcp56_713*C14);
    ROcp56_814 = -(ROcp56_56*S14-ROcp56_813*C14);
    ROcp56_914 = -(ROcp56_66*S14-ROcp56_913*C14);
    ROcp56_115 = ROcp56_113*C15+ROcp56_414*S15;
    ROcp56_215 = ROcp56_213*C15+ROcp56_514*S15;
    ROcp56_315 = ROcp56_313*C15+ROcp56_614*S15;
    ROcp56_415 = -(ROcp56_113*S15-ROcp56_414*C15);
    ROcp56_515 = -(ROcp56_213*S15-ROcp56_514*C15);
    ROcp56_615 = -(ROcp56_313*S15-ROcp56_614*C15);
    ROcp56_116 = ROcp56_115*C16-ROcp56_714*S16;
    ROcp56_216 = ROcp56_215*C16-ROcp56_814*S16;
    ROcp56_316 = ROcp56_315*C16-ROcp56_914*S16;
    ROcp56_716 = ROcp56_115*S16+ROcp56_714*C16;
    ROcp56_816 = ROcp56_215*S16+ROcp56_814*C16;
    ROcp56_916 = ROcp56_315*S16+ROcp56_914*C16;
    ROcp56_417 = ROcp56_415*C17+ROcp56_716*S17;
    ROcp56_517 = ROcp56_515*C17+ROcp56_816*S17;
    ROcp56_617 = ROcp56_615*C17+ROcp56_916*S17;
    ROcp56_717 = -(ROcp56_415*S17-ROcp56_716*C17);
    ROcp56_817 = -(ROcp56_515*S17-ROcp56_816*C17);
    ROcp56_917 = -(ROcp56_615*S17-ROcp56_916*C17);
    ROcp56_118 = ROcp56_116*C18-ROcp56_717*S18;
    ROcp56_218 = ROcp56_216*C18-ROcp56_817*S18;
    ROcp56_318 = ROcp56_316*C18-ROcp56_917*S18;
    ROcp56_718 = ROcp56_116*S18+ROcp56_717*C18;
    ROcp56_818 = ROcp56_216*S18+ROcp56_817*C18;
    ROcp56_918 = ROcp56_316*S18+ROcp56_917*C18;
    RLcp56_113 = s->dpt[1][2]*ROcp56_16+s->dpt[3][2]*S5+ROcp56_46*s->dpt[2][2];
    RLcp56_213 = s->dpt[1][2]*ROcp56_26+s->dpt[3][2]*ROcp56_85+ROcp56_56*s->dpt[2][2];
    RLcp56_313 = s->dpt[1][2]*ROcp56_36+s->dpt[3][2]*ROcp56_95+ROcp56_66*s->dpt[2][2];
    OMcp56_113 = OMcp56_16+ROcp56_46*qd[13];
    OMcp56_213 = OMcp56_26+ROcp56_56*qd[13];
    OMcp56_313 = OMcp56_36+ROcp56_66*qd[13];
    ORcp56_113 = OMcp56_26*RLcp56_313-OMcp56_36*RLcp56_213;
    ORcp56_213 = -(OMcp56_16*RLcp56_313-OMcp56_36*RLcp56_113);
    ORcp56_313 = OMcp56_16*RLcp56_213-OMcp56_26*RLcp56_113;
    OPcp56_113 = OPcp56_16+ROcp56_46*qdd[13]+qd[13]*(OMcp56_26*ROcp56_66-OMcp56_36*ROcp56_56);
    OPcp56_213 = OPcp56_26+ROcp56_56*qdd[13]-qd[13]*(OMcp56_16*ROcp56_66-OMcp56_36*ROcp56_46);
    OPcp56_313 = OPcp56_36+ROcp56_66*qdd[13]+qd[13]*(OMcp56_16*ROcp56_56-OMcp56_26*ROcp56_46);
    RLcp56_114 = s->dpt[1][22]*ROcp56_113+s->dpt[3][22]*ROcp56_713+ROcp56_46*s->dpt[2][22];
    RLcp56_214 = s->dpt[1][22]*ROcp56_213+s->dpt[3][22]*ROcp56_813+ROcp56_56*s->dpt[2][22];
    RLcp56_314 = s->dpt[1][22]*ROcp56_313+s->dpt[3][22]*ROcp56_913+ROcp56_66*s->dpt[2][22];
    OMcp56_114 = OMcp56_113+ROcp56_113*qd[14];
    OMcp56_214 = OMcp56_213+ROcp56_213*qd[14];
    OMcp56_314 = OMcp56_313+ROcp56_313*qd[14];
    ORcp56_114 = OMcp56_213*RLcp56_314-OMcp56_313*RLcp56_214;
    ORcp56_214 = -(OMcp56_113*RLcp56_314-OMcp56_313*RLcp56_114);
    ORcp56_314 = OMcp56_113*RLcp56_214-OMcp56_213*RLcp56_114;
    OPcp56_114 = OPcp56_113+ROcp56_113*qdd[14]+qd[14]*(OMcp56_213*ROcp56_313-OMcp56_313*ROcp56_213);
    OPcp56_214 = OPcp56_213+ROcp56_213*qdd[14]-qd[14]*(OMcp56_113*ROcp56_313-OMcp56_313*ROcp56_113);
    OPcp56_314 = OPcp56_313+ROcp56_313*qdd[14]+qd[14]*(OMcp56_113*ROcp56_213-OMcp56_213*ROcp56_113);
    RLcp56_115 = s->dpt[1][24]*ROcp56_113+s->dpt[2][24]*ROcp56_414+ROcp56_714*s->dpt[3][24];
    RLcp56_215 = s->dpt[1][24]*ROcp56_213+s->dpt[2][24]*ROcp56_514+ROcp56_814*s->dpt[3][24];
    RLcp56_315 = s->dpt[1][24]*ROcp56_313+s->dpt[2][24]*ROcp56_614+ROcp56_914*s->dpt[3][24];
    OMcp56_115 = OMcp56_114+ROcp56_714*qd[15];
    OMcp56_215 = OMcp56_214+ROcp56_814*qd[15];
    OMcp56_315 = OMcp56_314+ROcp56_914*qd[15];
    ORcp56_115 = OMcp56_214*RLcp56_315-OMcp56_314*RLcp56_215;
    ORcp56_215 = -(OMcp56_114*RLcp56_315-OMcp56_314*RLcp56_115);
    ORcp56_315 = OMcp56_114*RLcp56_215-OMcp56_214*RLcp56_115;
    OPcp56_115 = OPcp56_114+ROcp56_714*qdd[15]+qd[15]*(OMcp56_214*ROcp56_914-OMcp56_314*ROcp56_814);
    OPcp56_215 = OPcp56_214+ROcp56_814*qdd[15]-qd[15]*(OMcp56_114*ROcp56_914-OMcp56_314*ROcp56_714);
    OPcp56_315 = OPcp56_314+ROcp56_914*qdd[15]+qd[15]*(OMcp56_114*ROcp56_814-OMcp56_214*ROcp56_714);
    RLcp56_116 = s->dpt[1][26]*ROcp56_115+s->dpt[2][26]*ROcp56_415+ROcp56_714*s->dpt[3][26];
    RLcp56_216 = s->dpt[1][26]*ROcp56_215+s->dpt[2][26]*ROcp56_515+ROcp56_814*s->dpt[3][26];
    RLcp56_316 = s->dpt[1][26]*ROcp56_315+s->dpt[2][26]*ROcp56_615+ROcp56_914*s->dpt[3][26];
    OMcp56_116 = OMcp56_115+ROcp56_415*qd[16];
    OMcp56_216 = OMcp56_215+ROcp56_515*qd[16];
    OMcp56_316 = OMcp56_315+ROcp56_615*qd[16];
    ORcp56_116 = OMcp56_215*RLcp56_316-OMcp56_315*RLcp56_216;
    ORcp56_216 = -(OMcp56_115*RLcp56_316-OMcp56_315*RLcp56_116);
    ORcp56_316 = OMcp56_115*RLcp56_216-OMcp56_215*RLcp56_116;
    OPcp56_116 = OPcp56_115+ROcp56_415*qdd[16]+qd[16]*(OMcp56_215*ROcp56_615-OMcp56_315*ROcp56_515);
    OPcp56_216 = OPcp56_215+ROcp56_515*qdd[16]-qd[16]*(OMcp56_115*ROcp56_615-OMcp56_315*ROcp56_415);
    OPcp56_316 = OPcp56_315+ROcp56_615*qdd[16]+qd[16]*(OMcp56_115*ROcp56_515-OMcp56_215*ROcp56_415);
    RLcp56_117 = s->dpt[1][28]*ROcp56_116+s->dpt[2][28]*ROcp56_415+ROcp56_716*s->dpt[3][28];
    RLcp56_217 = s->dpt[1][28]*ROcp56_216+s->dpt[2][28]*ROcp56_515+ROcp56_816*s->dpt[3][28];
    RLcp56_317 = s->dpt[1][28]*ROcp56_316+s->dpt[2][28]*ROcp56_615+ROcp56_916*s->dpt[3][28];
    OMcp56_117 = OMcp56_116+ROcp56_116*qd[17];
    OMcp56_217 = OMcp56_216+ROcp56_216*qd[17];
    OMcp56_317 = OMcp56_316+ROcp56_316*qd[17];
    ORcp56_117 = OMcp56_216*RLcp56_317-OMcp56_316*RLcp56_217;
    ORcp56_217 = -(OMcp56_116*RLcp56_317-OMcp56_316*RLcp56_117);
    ORcp56_317 = OMcp56_116*RLcp56_217-OMcp56_216*RLcp56_117;
    OPcp56_117 = OPcp56_116+ROcp56_116*qdd[17]+qd[17]*(OMcp56_216*ROcp56_316-OMcp56_316*ROcp56_216);
    OPcp56_217 = OPcp56_216+ROcp56_216*qdd[17]-qd[17]*(OMcp56_116*ROcp56_316-OMcp56_316*ROcp56_116);
    OPcp56_317 = OPcp56_316+ROcp56_316*qdd[17]+qd[17]*(OMcp56_116*ROcp56_216-OMcp56_216*ROcp56_116);
    RLcp56_118 = s->dpt[1][30]*ROcp56_116+s->dpt[2][30]*ROcp56_417+s->dpt[3][30]*ROcp56_717;
    RLcp56_218 = s->dpt[1][30]*ROcp56_216+s->dpt[2][30]*ROcp56_517+s->dpt[3][30]*ROcp56_817;
    RLcp56_318 = s->dpt[1][30]*ROcp56_316+s->dpt[2][30]*ROcp56_617+s->dpt[3][30]*ROcp56_917;
    OMcp56_118 = OMcp56_117+ROcp56_417*qd[18];
    OMcp56_218 = OMcp56_217+ROcp56_517*qd[18];
    OMcp56_318 = OMcp56_317+ROcp56_617*qd[18];
    ORcp56_118 = OMcp56_217*RLcp56_318-OMcp56_317*RLcp56_218;
    ORcp56_218 = -(OMcp56_117*RLcp56_318-OMcp56_317*RLcp56_118);
    ORcp56_318 = OMcp56_117*RLcp56_218-OMcp56_217*RLcp56_118;
    OPcp56_118 = OPcp56_117+ROcp56_417*qdd[18]+qd[18]*(OMcp56_217*ROcp56_617-OMcp56_317*ROcp56_517);
    OPcp56_218 = OPcp56_217+ROcp56_517*qdd[18]-qd[18]*(OMcp56_117*ROcp56_617-OMcp56_317*ROcp56_417);
    OPcp56_318 = OPcp56_317+ROcp56_617*qdd[18]+qd[18]*(OMcp56_117*ROcp56_517-OMcp56_217*ROcp56_417);
    RLcp56_192 = s->dpt[1][33]*ROcp56_118+s->dpt[2][33]*ROcp56_417+ROcp56_718*s->dpt[3][33];
    RLcp56_292 = s->dpt[1][33]*ROcp56_218+s->dpt[2][33]*ROcp56_517+ROcp56_818*s->dpt[3][33];
    RLcp56_392 = s->dpt[1][33]*ROcp56_318+s->dpt[2][33]*ROcp56_617+ROcp56_918*s->dpt[3][33];
    POcp56_192 = RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_192+q[1];
    POcp56_292 = RLcp56_213+RLcp56_214+RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_292+q[2];
    POcp56_392 = RLcp56_313+RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_392+q[3];
    JTcp56_292_4 = -(RLcp56_313+RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_392);
    JTcp56_392_4 = RLcp56_213+RLcp56_214+RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_292;
    JTcp56_192_5 = C4*(RLcp56_313+RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318)-S4*(RLcp56_213+RLcp56_214)-S4*(
 RLcp56_215+RLcp56_216)-S4*(RLcp56_217+RLcp56_218)-RLcp56_292*S4+RLcp56_392*C4;
    JTcp56_292_5 = S4*(RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_192);
    JTcp56_392_5 = -C4*(RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_192);
    JTcp56_192_6 = ROcp56_85*(RLcp56_313+RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318)-ROcp56_95*(RLcp56_213+
 RLcp56_214)-ROcp56_95*(RLcp56_215+RLcp56_216)-ROcp56_95*(RLcp56_217+RLcp56_218)-RLcp56_292*ROcp56_95+RLcp56_392*ROcp56_85;
    JTcp56_292_6 = -(RLcp56_392*S5-ROcp56_95*(RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_192
 )+S5*(RLcp56_313+RLcp56_314)+S5*(RLcp56_315+RLcp56_316)+S5*(RLcp56_317+RLcp56_318));
    JTcp56_392_6 = RLcp56_292*S5-ROcp56_85*(RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_192)+
 S5*(RLcp56_213+RLcp56_214)+S5*(RLcp56_215+RLcp56_216)+S5*(RLcp56_217+RLcp56_218);
    JTcp56_192_7 = ROcp56_56*(RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_392)-ROcp56_66*(RLcp56_214+
 RLcp56_215)-ROcp56_66*(RLcp56_216+RLcp56_217)-ROcp56_66*(RLcp56_218+RLcp56_292);
    JTcp56_292_7 = -(ROcp56_46*(RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_392)-ROcp56_66*(RLcp56_114+
 RLcp56_115)-ROcp56_66*(RLcp56_116+RLcp56_117)-ROcp56_66*(RLcp56_118+RLcp56_192));
    JTcp56_392_7 = ROcp56_46*(RLcp56_214+RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_292)-ROcp56_56*(RLcp56_114+
 RLcp56_115)-ROcp56_56*(RLcp56_116+RLcp56_117)-ROcp56_56*(RLcp56_118+RLcp56_192);
    JTcp56_192_8 = ROcp56_213*(RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318)-ROcp56_313*(RLcp56_215+RLcp56_216)-ROcp56_313*
 (RLcp56_217+RLcp56_218)-RLcp56_292*ROcp56_313+RLcp56_392*ROcp56_213;
    JTcp56_292_8 = RLcp56_192*ROcp56_313-RLcp56_392*ROcp56_113-ROcp56_113*(RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318)+
 ROcp56_313*(RLcp56_115+RLcp56_116)+ROcp56_313*(RLcp56_117+RLcp56_118);
    JTcp56_392_8 = ROcp56_113*(RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218)-ROcp56_213*(RLcp56_115+RLcp56_116)-ROcp56_213*
 (RLcp56_117+RLcp56_118)-RLcp56_192*ROcp56_213+RLcp56_292*ROcp56_113;
    JTcp56_192_9 = ROcp56_814*(RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_392)-ROcp56_914*(RLcp56_216+RLcp56_217)-ROcp56_914*
 (RLcp56_218+RLcp56_292);
    JTcp56_292_9 = -(ROcp56_714*(RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_392)-ROcp56_914*(RLcp56_116+RLcp56_117)-
 ROcp56_914*(RLcp56_118+RLcp56_192));
    JTcp56_392_9 = ROcp56_714*(RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_292)-ROcp56_814*(RLcp56_116+RLcp56_117)-ROcp56_814*
 (RLcp56_118+RLcp56_192);
    JTcp56_192_10 = ROcp56_515*(RLcp56_317+RLcp56_318)-ROcp56_615*(RLcp56_217+RLcp56_218)-RLcp56_292*ROcp56_615+RLcp56_392
 *ROcp56_515;
    JTcp56_292_10 = RLcp56_192*ROcp56_615-RLcp56_392*ROcp56_415-ROcp56_415*(RLcp56_317+RLcp56_318)+ROcp56_615*(RLcp56_117+
 RLcp56_118);
    JTcp56_392_10 = ROcp56_415*(RLcp56_217+RLcp56_218)-ROcp56_515*(RLcp56_117+RLcp56_118)-RLcp56_192*ROcp56_515+RLcp56_292
 *ROcp56_415;
    JTcp56_192_11 = ROcp56_216*(RLcp56_318+RLcp56_392)-ROcp56_316*(RLcp56_218+RLcp56_292);
    JTcp56_292_11 = -(ROcp56_116*(RLcp56_318+RLcp56_392)-ROcp56_316*(RLcp56_118+RLcp56_192));
    JTcp56_392_11 = ROcp56_116*(RLcp56_218+RLcp56_292)-ROcp56_216*(RLcp56_118+RLcp56_192);
    JTcp56_192_12 = -(RLcp56_292*ROcp56_617-RLcp56_392*ROcp56_517);
    JTcp56_292_12 = RLcp56_192*ROcp56_617-RLcp56_392*ROcp56_417;
    JTcp56_392_12 = -(RLcp56_192*ROcp56_517-RLcp56_292*ROcp56_417);
    ORcp56_192 = OMcp56_218*RLcp56_392-OMcp56_318*RLcp56_292;
    ORcp56_292 = -(OMcp56_118*RLcp56_392-OMcp56_318*RLcp56_192);
    ORcp56_392 = OMcp56_118*RLcp56_292-OMcp56_218*RLcp56_192;
    VIcp56_192 = ORcp56_113+ORcp56_114+ORcp56_115+ORcp56_116+ORcp56_117+ORcp56_118+ORcp56_192+qd[1];
    VIcp56_292 = ORcp56_213+ORcp56_214+ORcp56_215+ORcp56_216+ORcp56_217+ORcp56_218+ORcp56_292+qd[2];
    VIcp56_392 = ORcp56_313+ORcp56_314+ORcp56_315+ORcp56_316+ORcp56_317+ORcp56_318+ORcp56_392+qd[3];
    ACcp56_192 = qdd[1]+OMcp56_213*ORcp56_314+OMcp56_214*ORcp56_315+OMcp56_215*ORcp56_316+OMcp56_216*ORcp56_317+OMcp56_217
 *ORcp56_318+OMcp56_218*ORcp56_392+OMcp56_26*ORcp56_313-OMcp56_313*ORcp56_214-OMcp56_314*ORcp56_215-OMcp56_315*ORcp56_216-
 OMcp56_316*ORcp56_217-OMcp56_317*ORcp56_218-OMcp56_318*ORcp56_292-OMcp56_36*ORcp56_213+OPcp56_213*RLcp56_314+OPcp56_214*
 RLcp56_315+OPcp56_215*RLcp56_316+OPcp56_216*RLcp56_317+OPcp56_217*RLcp56_318+OPcp56_218*RLcp56_392+OPcp56_26*RLcp56_313-
 OPcp56_313*RLcp56_214-OPcp56_314*RLcp56_215-OPcp56_315*RLcp56_216-OPcp56_316*RLcp56_217-OPcp56_317*RLcp56_218-OPcp56_318*
 RLcp56_292-OPcp56_36*RLcp56_213;
    ACcp56_292 = qdd[2]-OMcp56_113*ORcp56_314-OMcp56_114*ORcp56_315-OMcp56_115*ORcp56_316-OMcp56_116*ORcp56_317-OMcp56_117
 *ORcp56_318-OMcp56_118*ORcp56_392-OMcp56_16*ORcp56_313+OMcp56_313*ORcp56_114+OMcp56_314*ORcp56_115+OMcp56_315*ORcp56_116+
 OMcp56_316*ORcp56_117+OMcp56_317*ORcp56_118+OMcp56_318*ORcp56_192+OMcp56_36*ORcp56_113-OPcp56_113*RLcp56_314-OPcp56_114*
 RLcp56_315-OPcp56_115*RLcp56_316-OPcp56_116*RLcp56_317-OPcp56_117*RLcp56_318-OPcp56_118*RLcp56_392-OPcp56_16*RLcp56_313+
 OPcp56_313*RLcp56_114+OPcp56_314*RLcp56_115+OPcp56_315*RLcp56_116+OPcp56_316*RLcp56_117+OPcp56_317*RLcp56_118+OPcp56_318*
 RLcp56_192+OPcp56_36*RLcp56_113;
    ACcp56_392 = qdd[3]+OMcp56_113*ORcp56_214+OMcp56_114*ORcp56_215+OMcp56_115*ORcp56_216+OMcp56_116*ORcp56_217+OMcp56_117
 *ORcp56_218+OMcp56_118*ORcp56_292+OMcp56_16*ORcp56_213-OMcp56_213*ORcp56_114-OMcp56_214*ORcp56_115-OMcp56_215*ORcp56_116-
 OMcp56_216*ORcp56_117-OMcp56_217*ORcp56_118-OMcp56_218*ORcp56_192-OMcp56_26*ORcp56_113+OPcp56_113*RLcp56_214+OPcp56_114*
 RLcp56_215+OPcp56_115*RLcp56_216+OPcp56_116*RLcp56_217+OPcp56_117*RLcp56_218+OPcp56_118*RLcp56_292+OPcp56_16*RLcp56_213-
 OPcp56_213*RLcp56_114-OPcp56_214*RLcp56_115-OPcp56_215*RLcp56_116-OPcp56_216*RLcp56_117-OPcp56_217*RLcp56_118-OPcp56_218*
 RLcp56_192-OPcp56_26*RLcp56_113;

// = = Block_1_0_0_57_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp56_192;
    sens->P[2] = POcp56_292;
    sens->P[3] = POcp56_392;
    sens->R[1][1] = ROcp56_118;
    sens->R[1][2] = ROcp56_218;
    sens->R[1][3] = ROcp56_318;
    sens->R[2][1] = ROcp56_417;
    sens->R[2][2] = ROcp56_517;
    sens->R[2][3] = ROcp56_617;
    sens->R[3][1] = ROcp56_718;
    sens->R[3][2] = ROcp56_818;
    sens->R[3][3] = ROcp56_918;
    sens->V[1] = VIcp56_192;
    sens->V[2] = VIcp56_292;
    sens->V[3] = VIcp56_392;
    sens->OM[1] = OMcp56_118;
    sens->OM[2] = OMcp56_218;
    sens->OM[3] = OMcp56_318;
    sens->J[1][1] = (1.0);
    sens->J[1][5] = JTcp56_192_5;
    sens->J[1][6] = JTcp56_192_6;
    sens->J[1][13] = JTcp56_192_7;
    sens->J[1][14] = JTcp56_192_8;
    sens->J[1][15] = JTcp56_192_9;
    sens->J[1][16] = JTcp56_192_10;
    sens->J[1][17] = JTcp56_192_11;
    sens->J[1][18] = JTcp56_192_12;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp56_292_4;
    sens->J[2][5] = JTcp56_292_5;
    sens->J[2][6] = JTcp56_292_6;
    sens->J[2][13] = JTcp56_292_7;
    sens->J[2][14] = JTcp56_292_8;
    sens->J[2][15] = JTcp56_292_9;
    sens->J[2][16] = JTcp56_292_10;
    sens->J[2][17] = JTcp56_292_11;
    sens->J[2][18] = JTcp56_292_12;
    sens->J[3][3] = (1.0);
    sens->J[3][4] = JTcp56_392_4;
    sens->J[3][5] = JTcp56_392_5;
    sens->J[3][6] = JTcp56_392_6;
    sens->J[3][13] = JTcp56_392_7;
    sens->J[3][14] = JTcp56_392_8;
    sens->J[3][15] = JTcp56_392_9;
    sens->J[3][16] = JTcp56_392_10;
    sens->J[3][17] = JTcp56_392_11;
    sens->J[3][18] = JTcp56_392_12;
    sens->J[4][4] = (1.0);
    sens->J[4][6] = S5;
    sens->J[4][13] = ROcp56_46;
    sens->J[4][14] = ROcp56_113;
    sens->J[4][15] = ROcp56_714;
    sens->J[4][16] = ROcp56_415;
    sens->J[4][17] = ROcp56_116;
    sens->J[4][18] = ROcp56_417;
    sens->J[5][5] = C4;
    sens->J[5][6] = ROcp56_85;
    sens->J[5][13] = ROcp56_56;
    sens->J[5][14] = ROcp56_213;
    sens->J[5][15] = ROcp56_814;
    sens->J[5][16] = ROcp56_515;
    sens->J[5][17] = ROcp56_216;
    sens->J[5][18] = ROcp56_517;
    sens->J[6][5] = S4;
    sens->J[6][6] = ROcp56_95;
    sens->J[6][13] = ROcp56_66;
    sens->J[6][14] = ROcp56_313;
    sens->J[6][15] = ROcp56_914;
    sens->J[6][16] = ROcp56_615;
    sens->J[6][17] = ROcp56_316;
    sens->J[6][18] = ROcp56_617;
    sens->A[1] = ACcp56_192;
    sens->A[2] = ACcp56_292;
    sens->A[3] = ACcp56_392;
    sens->OMP[1] = OPcp56_118;
    sens->OMP[2] = OPcp56_218;
    sens->OMP[3] = OPcp56_318;
 
// 
break;
case 58:
 


// = = Block_1_0_0_58_0_1 = = 
 
// Sensor Kinematics 


    ROcp57_25 = S4*S5;
    ROcp57_35 = -C4*S5;
    ROcp57_85 = -S4*C5;
    ROcp57_95 = C4*C5;
    ROcp57_16 = C5*C6;
    ROcp57_26 = ROcp57_25*C6+C4*S6;
    ROcp57_36 = ROcp57_35*C6+S4*S6;
    ROcp57_46 = -C5*S6;
    ROcp57_56 = -(ROcp57_25*S6-C4*C6);
    ROcp57_66 = -(ROcp57_35*S6-S4*C6);
    OMcp57_25 = qd[5]*C4;
    OMcp57_35 = qd[5]*S4;
    OMcp57_16 = qd[4]+qd[6]*S5;
    OMcp57_26 = OMcp57_25+ROcp57_85*qd[6];
    OMcp57_36 = OMcp57_35+ROcp57_95*qd[6];
    OPcp57_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp57_26 = ROcp57_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp57_35*S5-ROcp57_95*qd[4]);
    OPcp57_36 = ROcp57_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp57_25*S5-ROcp57_85*qd[4]);

// = = Block_1_0_0_58_0_3 = = 
 
// Sensor Kinematics 


    ROcp57_113 = ROcp57_16*C13-S13*S5;
    ROcp57_213 = ROcp57_26*C13-ROcp57_85*S13;
    ROcp57_313 = ROcp57_36*C13-ROcp57_95*S13;
    ROcp57_713 = ROcp57_16*S13+C13*S5;
    ROcp57_813 = ROcp57_26*S13+ROcp57_85*C13;
    ROcp57_913 = ROcp57_36*S13+ROcp57_95*C13;
    ROcp57_414 = ROcp57_46*C14+ROcp57_713*S14;
    ROcp57_514 = ROcp57_56*C14+ROcp57_813*S14;
    ROcp57_614 = ROcp57_66*C14+ROcp57_913*S14;
    ROcp57_714 = -(ROcp57_46*S14-ROcp57_713*C14);
    ROcp57_814 = -(ROcp57_56*S14-ROcp57_813*C14);
    ROcp57_914 = -(ROcp57_66*S14-ROcp57_913*C14);
    ROcp57_115 = ROcp57_113*C15+ROcp57_414*S15;
    ROcp57_215 = ROcp57_213*C15+ROcp57_514*S15;
    ROcp57_315 = ROcp57_313*C15+ROcp57_614*S15;
    ROcp57_415 = -(ROcp57_113*S15-ROcp57_414*C15);
    ROcp57_515 = -(ROcp57_213*S15-ROcp57_514*C15);
    ROcp57_615 = -(ROcp57_313*S15-ROcp57_614*C15);
    ROcp57_116 = ROcp57_115*C16-ROcp57_714*S16;
    ROcp57_216 = ROcp57_215*C16-ROcp57_814*S16;
    ROcp57_316 = ROcp57_315*C16-ROcp57_914*S16;
    ROcp57_716 = ROcp57_115*S16+ROcp57_714*C16;
    ROcp57_816 = ROcp57_215*S16+ROcp57_814*C16;
    ROcp57_916 = ROcp57_315*S16+ROcp57_914*C16;
    ROcp57_417 = ROcp57_415*C17+ROcp57_716*S17;
    ROcp57_517 = ROcp57_515*C17+ROcp57_816*S17;
    ROcp57_617 = ROcp57_615*C17+ROcp57_916*S17;
    ROcp57_717 = -(ROcp57_415*S17-ROcp57_716*C17);
    ROcp57_817 = -(ROcp57_515*S17-ROcp57_816*C17);
    ROcp57_917 = -(ROcp57_615*S17-ROcp57_916*C17);
    ROcp57_118 = ROcp57_116*C18-ROcp57_717*S18;
    ROcp57_218 = ROcp57_216*C18-ROcp57_817*S18;
    ROcp57_318 = ROcp57_316*C18-ROcp57_917*S18;
    ROcp57_718 = ROcp57_116*S18+ROcp57_717*C18;
    ROcp57_818 = ROcp57_216*S18+ROcp57_817*C18;
    ROcp57_918 = ROcp57_316*S18+ROcp57_917*C18;
    RLcp57_113 = s->dpt[1][2]*ROcp57_16+s->dpt[3][2]*S5+ROcp57_46*s->dpt[2][2];
    RLcp57_213 = s->dpt[1][2]*ROcp57_26+s->dpt[3][2]*ROcp57_85+ROcp57_56*s->dpt[2][2];
    RLcp57_313 = s->dpt[1][2]*ROcp57_36+s->dpt[3][2]*ROcp57_95+ROcp57_66*s->dpt[2][2];
    OMcp57_113 = OMcp57_16+ROcp57_46*qd[13];
    OMcp57_213 = OMcp57_26+ROcp57_56*qd[13];
    OMcp57_313 = OMcp57_36+ROcp57_66*qd[13];
    ORcp57_113 = OMcp57_26*RLcp57_313-OMcp57_36*RLcp57_213;
    ORcp57_213 = -(OMcp57_16*RLcp57_313-OMcp57_36*RLcp57_113);
    ORcp57_313 = OMcp57_16*RLcp57_213-OMcp57_26*RLcp57_113;
    OPcp57_113 = OPcp57_16+ROcp57_46*qdd[13]+qd[13]*(OMcp57_26*ROcp57_66-OMcp57_36*ROcp57_56);
    OPcp57_213 = OPcp57_26+ROcp57_56*qdd[13]-qd[13]*(OMcp57_16*ROcp57_66-OMcp57_36*ROcp57_46);
    OPcp57_313 = OPcp57_36+ROcp57_66*qdd[13]+qd[13]*(OMcp57_16*ROcp57_56-OMcp57_26*ROcp57_46);
    RLcp57_114 = s->dpt[1][22]*ROcp57_113+s->dpt[3][22]*ROcp57_713+ROcp57_46*s->dpt[2][22];
    RLcp57_214 = s->dpt[1][22]*ROcp57_213+s->dpt[3][22]*ROcp57_813+ROcp57_56*s->dpt[2][22];
    RLcp57_314 = s->dpt[1][22]*ROcp57_313+s->dpt[3][22]*ROcp57_913+ROcp57_66*s->dpt[2][22];
    OMcp57_114 = OMcp57_113+ROcp57_113*qd[14];
    OMcp57_214 = OMcp57_213+ROcp57_213*qd[14];
    OMcp57_314 = OMcp57_313+ROcp57_313*qd[14];
    ORcp57_114 = OMcp57_213*RLcp57_314-OMcp57_313*RLcp57_214;
    ORcp57_214 = -(OMcp57_113*RLcp57_314-OMcp57_313*RLcp57_114);
    ORcp57_314 = OMcp57_113*RLcp57_214-OMcp57_213*RLcp57_114;
    OPcp57_114 = OPcp57_113+ROcp57_113*qdd[14]+qd[14]*(OMcp57_213*ROcp57_313-OMcp57_313*ROcp57_213);
    OPcp57_214 = OPcp57_213+ROcp57_213*qdd[14]-qd[14]*(OMcp57_113*ROcp57_313-OMcp57_313*ROcp57_113);
    OPcp57_314 = OPcp57_313+ROcp57_313*qdd[14]+qd[14]*(OMcp57_113*ROcp57_213-OMcp57_213*ROcp57_113);
    RLcp57_115 = s->dpt[1][24]*ROcp57_113+s->dpt[2][24]*ROcp57_414+ROcp57_714*s->dpt[3][24];
    RLcp57_215 = s->dpt[1][24]*ROcp57_213+s->dpt[2][24]*ROcp57_514+ROcp57_814*s->dpt[3][24];
    RLcp57_315 = s->dpt[1][24]*ROcp57_313+s->dpt[2][24]*ROcp57_614+ROcp57_914*s->dpt[3][24];
    OMcp57_115 = OMcp57_114+ROcp57_714*qd[15];
    OMcp57_215 = OMcp57_214+ROcp57_814*qd[15];
    OMcp57_315 = OMcp57_314+ROcp57_914*qd[15];
    ORcp57_115 = OMcp57_214*RLcp57_315-OMcp57_314*RLcp57_215;
    ORcp57_215 = -(OMcp57_114*RLcp57_315-OMcp57_314*RLcp57_115);
    ORcp57_315 = OMcp57_114*RLcp57_215-OMcp57_214*RLcp57_115;
    OPcp57_115 = OPcp57_114+ROcp57_714*qdd[15]+qd[15]*(OMcp57_214*ROcp57_914-OMcp57_314*ROcp57_814);
    OPcp57_215 = OPcp57_214+ROcp57_814*qdd[15]-qd[15]*(OMcp57_114*ROcp57_914-OMcp57_314*ROcp57_714);
    OPcp57_315 = OPcp57_314+ROcp57_914*qdd[15]+qd[15]*(OMcp57_114*ROcp57_814-OMcp57_214*ROcp57_714);
    RLcp57_116 = s->dpt[1][26]*ROcp57_115+s->dpt[2][26]*ROcp57_415+ROcp57_714*s->dpt[3][26];
    RLcp57_216 = s->dpt[1][26]*ROcp57_215+s->dpt[2][26]*ROcp57_515+ROcp57_814*s->dpt[3][26];
    RLcp57_316 = s->dpt[1][26]*ROcp57_315+s->dpt[2][26]*ROcp57_615+ROcp57_914*s->dpt[3][26];
    OMcp57_116 = OMcp57_115+ROcp57_415*qd[16];
    OMcp57_216 = OMcp57_215+ROcp57_515*qd[16];
    OMcp57_316 = OMcp57_315+ROcp57_615*qd[16];
    ORcp57_116 = OMcp57_215*RLcp57_316-OMcp57_315*RLcp57_216;
    ORcp57_216 = -(OMcp57_115*RLcp57_316-OMcp57_315*RLcp57_116);
    ORcp57_316 = OMcp57_115*RLcp57_216-OMcp57_215*RLcp57_116;
    OPcp57_116 = OPcp57_115+ROcp57_415*qdd[16]+qd[16]*(OMcp57_215*ROcp57_615-OMcp57_315*ROcp57_515);
    OPcp57_216 = OPcp57_215+ROcp57_515*qdd[16]-qd[16]*(OMcp57_115*ROcp57_615-OMcp57_315*ROcp57_415);
    OPcp57_316 = OPcp57_315+ROcp57_615*qdd[16]+qd[16]*(OMcp57_115*ROcp57_515-OMcp57_215*ROcp57_415);
    RLcp57_117 = s->dpt[1][28]*ROcp57_116+s->dpt[2][28]*ROcp57_415+ROcp57_716*s->dpt[3][28];
    RLcp57_217 = s->dpt[1][28]*ROcp57_216+s->dpt[2][28]*ROcp57_515+ROcp57_816*s->dpt[3][28];
    RLcp57_317 = s->dpt[1][28]*ROcp57_316+s->dpt[2][28]*ROcp57_615+ROcp57_916*s->dpt[3][28];
    OMcp57_117 = OMcp57_116+ROcp57_116*qd[17];
    OMcp57_217 = OMcp57_216+ROcp57_216*qd[17];
    OMcp57_317 = OMcp57_316+ROcp57_316*qd[17];
    ORcp57_117 = OMcp57_216*RLcp57_317-OMcp57_316*RLcp57_217;
    ORcp57_217 = -(OMcp57_116*RLcp57_317-OMcp57_316*RLcp57_117);
    ORcp57_317 = OMcp57_116*RLcp57_217-OMcp57_216*RLcp57_117;
    OPcp57_117 = OPcp57_116+ROcp57_116*qdd[17]+qd[17]*(OMcp57_216*ROcp57_316-OMcp57_316*ROcp57_216);
    OPcp57_217 = OPcp57_216+ROcp57_216*qdd[17]-qd[17]*(OMcp57_116*ROcp57_316-OMcp57_316*ROcp57_116);
    OPcp57_317 = OPcp57_316+ROcp57_316*qdd[17]+qd[17]*(OMcp57_116*ROcp57_216-OMcp57_216*ROcp57_116);
    RLcp57_118 = s->dpt[1][30]*ROcp57_116+s->dpt[2][30]*ROcp57_417+s->dpt[3][30]*ROcp57_717;
    RLcp57_218 = s->dpt[1][30]*ROcp57_216+s->dpt[2][30]*ROcp57_517+s->dpt[3][30]*ROcp57_817;
    RLcp57_318 = s->dpt[1][30]*ROcp57_316+s->dpt[2][30]*ROcp57_617+s->dpt[3][30]*ROcp57_917;
    OMcp57_118 = OMcp57_117+ROcp57_417*qd[18];
    OMcp57_218 = OMcp57_217+ROcp57_517*qd[18];
    OMcp57_318 = OMcp57_317+ROcp57_617*qd[18];
    ORcp57_118 = OMcp57_217*RLcp57_318-OMcp57_317*RLcp57_218;
    ORcp57_218 = -(OMcp57_117*RLcp57_318-OMcp57_317*RLcp57_118);
    ORcp57_318 = OMcp57_117*RLcp57_218-OMcp57_217*RLcp57_118;
    OPcp57_118 = OPcp57_117+ROcp57_417*qdd[18]+qd[18]*(OMcp57_217*ROcp57_617-OMcp57_317*ROcp57_517);
    OPcp57_218 = OPcp57_217+ROcp57_517*qdd[18]-qd[18]*(OMcp57_117*ROcp57_617-OMcp57_317*ROcp57_417);
    OPcp57_318 = OPcp57_317+ROcp57_617*qdd[18]+qd[18]*(OMcp57_117*ROcp57_517-OMcp57_217*ROcp57_417);
    RLcp57_193 = ROcp57_118*s->dpt[1][34]+ROcp57_417*s->dpt[2][34]+ROcp57_718*s->dpt[3][34];
    RLcp57_293 = ROcp57_218*s->dpt[1][34]+ROcp57_517*s->dpt[2][34]+ROcp57_818*s->dpt[3][34];
    RLcp57_393 = ROcp57_318*s->dpt[1][34]+ROcp57_617*s->dpt[2][34]+ROcp57_918*s->dpt[3][34];
    POcp57_193 = RLcp57_113+RLcp57_114+RLcp57_115+RLcp57_116+RLcp57_117+RLcp57_118+RLcp57_193+q[1];
    POcp57_293 = RLcp57_213+RLcp57_214+RLcp57_215+RLcp57_216+RLcp57_217+RLcp57_218+RLcp57_293+q[2];
    POcp57_393 = RLcp57_313+RLcp57_314+RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318+RLcp57_393+q[3];
    ORcp57_193 = OMcp57_218*RLcp57_393-OMcp57_318*RLcp57_293;
    ORcp57_293 = -(OMcp57_118*RLcp57_393-OMcp57_318*RLcp57_193);
    ORcp57_393 = OMcp57_118*RLcp57_293-OMcp57_218*RLcp57_193;
    VIcp57_193 = ORcp57_113+ORcp57_114+ORcp57_115+ORcp57_116+ORcp57_117+ORcp57_118+ORcp57_193+qd[1];
    VIcp57_293 = ORcp57_213+ORcp57_214+ORcp57_215+ORcp57_216+ORcp57_217+ORcp57_218+ORcp57_293+qd[2];
    VIcp57_393 = ORcp57_313+ORcp57_314+ORcp57_315+ORcp57_316+ORcp57_317+ORcp57_318+ORcp57_393+qd[3];
    ACcp57_193 = qdd[1]+OMcp57_213*ORcp57_314+OMcp57_214*ORcp57_315+OMcp57_215*ORcp57_316+OMcp57_216*ORcp57_317+OMcp57_217
 *ORcp57_318+OMcp57_218*ORcp57_393+OMcp57_26*ORcp57_313-OMcp57_313*ORcp57_214-OMcp57_314*ORcp57_215-OMcp57_315*ORcp57_216-
 OMcp57_316*ORcp57_217-OMcp57_317*ORcp57_218-OMcp57_318*ORcp57_293-OMcp57_36*ORcp57_213+OPcp57_213*RLcp57_314+OPcp57_214*
 RLcp57_315+OPcp57_215*RLcp57_316+OPcp57_216*RLcp57_317+OPcp57_217*RLcp57_318+OPcp57_218*RLcp57_393+OPcp57_26*RLcp57_313-
 OPcp57_313*RLcp57_214-OPcp57_314*RLcp57_215-OPcp57_315*RLcp57_216-OPcp57_316*RLcp57_217-OPcp57_317*RLcp57_218-OPcp57_318*
 RLcp57_293-OPcp57_36*RLcp57_213;
    ACcp57_293 = qdd[2]-OMcp57_113*ORcp57_314-OMcp57_114*ORcp57_315-OMcp57_115*ORcp57_316-OMcp57_116*ORcp57_317-OMcp57_117
 *ORcp57_318-OMcp57_118*ORcp57_393-OMcp57_16*ORcp57_313+OMcp57_313*ORcp57_114+OMcp57_314*ORcp57_115+OMcp57_315*ORcp57_116+
 OMcp57_316*ORcp57_117+OMcp57_317*ORcp57_118+OMcp57_318*ORcp57_193+OMcp57_36*ORcp57_113-OPcp57_113*RLcp57_314-OPcp57_114*
 RLcp57_315-OPcp57_115*RLcp57_316-OPcp57_116*RLcp57_317-OPcp57_117*RLcp57_318-OPcp57_118*RLcp57_393-OPcp57_16*RLcp57_313+
 OPcp57_313*RLcp57_114+OPcp57_314*RLcp57_115+OPcp57_315*RLcp57_116+OPcp57_316*RLcp57_117+OPcp57_317*RLcp57_118+OPcp57_318*
 RLcp57_193+OPcp57_36*RLcp57_113;
    ACcp57_393 = qdd[3]+OMcp57_113*ORcp57_214+OMcp57_114*ORcp57_215+OMcp57_115*ORcp57_216+OMcp57_116*ORcp57_217+OMcp57_117
 *ORcp57_218+OMcp57_118*ORcp57_293+OMcp57_16*ORcp57_213-OMcp57_213*ORcp57_114-OMcp57_214*ORcp57_115-OMcp57_215*ORcp57_116-
 OMcp57_216*ORcp57_117-OMcp57_217*ORcp57_118-OMcp57_218*ORcp57_193-OMcp57_26*ORcp57_113+OPcp57_113*RLcp57_214+OPcp57_114*
 RLcp57_215+OPcp57_115*RLcp57_216+OPcp57_116*RLcp57_217+OPcp57_117*RLcp57_218+OPcp57_118*RLcp57_293+OPcp57_16*RLcp57_213-
 OPcp57_213*RLcp57_114-OPcp57_214*RLcp57_115-OPcp57_215*RLcp57_116-OPcp57_216*RLcp57_117-OPcp57_217*RLcp57_118-OPcp57_218*
 RLcp57_193-OPcp57_26*RLcp57_113;

// = = Block_1_0_0_58_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp57_193;
    sens->P[2] = POcp57_293;
    sens->P[3] = POcp57_393;
    sens->R[1][1] = ROcp57_118;
    sens->R[1][2] = ROcp57_218;
    sens->R[1][3] = ROcp57_318;
    sens->R[2][1] = ROcp57_417;
    sens->R[2][2] = ROcp57_517;
    sens->R[2][3] = ROcp57_617;
    sens->R[3][1] = ROcp57_718;
    sens->R[3][2] = ROcp57_818;
    sens->R[3][3] = ROcp57_918;
    sens->V[1] = VIcp57_193;
    sens->V[2] = VIcp57_293;
    sens->V[3] = VIcp57_393;
    sens->OM[1] = OMcp57_118;
    sens->OM[2] = OMcp57_218;
    sens->OM[3] = OMcp57_318;
    sens->A[1] = ACcp57_193;
    sens->A[2] = ACcp57_293;
    sens->A[3] = ACcp57_393;
    sens->OMP[1] = OPcp57_118;
    sens->OMP[2] = OPcp57_218;
    sens->OMP[3] = OPcp57_318;
 
// 
break;
case 59:
 


// = = Block_1_0_0_59_0_1 = = 
 
// Sensor Kinematics 


    ROcp58_25 = S4*S5;
    ROcp58_35 = -C4*S5;
    ROcp58_85 = -S4*C5;
    ROcp58_95 = C4*C5;
    ROcp58_16 = C5*C6;
    ROcp58_26 = ROcp58_25*C6+C4*S6;
    ROcp58_36 = ROcp58_35*C6+S4*S6;
    ROcp58_46 = -C5*S6;
    ROcp58_56 = -(ROcp58_25*S6-C4*C6);
    ROcp58_66 = -(ROcp58_35*S6-S4*C6);
    OMcp58_25 = qd[5]*C4;
    OMcp58_35 = qd[5]*S4;
    OMcp58_16 = qd[4]+qd[6]*S5;
    OMcp58_26 = OMcp58_25+ROcp58_85*qd[6];
    OMcp58_36 = OMcp58_35+ROcp58_95*qd[6];
    OPcp58_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp58_26 = ROcp58_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp58_35*S5-ROcp58_95*qd[4]);
    OPcp58_36 = ROcp58_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp58_25*S5-ROcp58_85*qd[4]);

// = = Block_1_0_0_59_0_3 = = 
 
// Sensor Kinematics 


    ROcp58_113 = ROcp58_16*C13-S13*S5;
    ROcp58_213 = ROcp58_26*C13-ROcp58_85*S13;
    ROcp58_313 = ROcp58_36*C13-ROcp58_95*S13;
    ROcp58_713 = ROcp58_16*S13+C13*S5;
    ROcp58_813 = ROcp58_26*S13+ROcp58_85*C13;
    ROcp58_913 = ROcp58_36*S13+ROcp58_95*C13;
    ROcp58_414 = ROcp58_46*C14+ROcp58_713*S14;
    ROcp58_514 = ROcp58_56*C14+ROcp58_813*S14;
    ROcp58_614 = ROcp58_66*C14+ROcp58_913*S14;
    ROcp58_714 = -(ROcp58_46*S14-ROcp58_713*C14);
    ROcp58_814 = -(ROcp58_56*S14-ROcp58_813*C14);
    ROcp58_914 = -(ROcp58_66*S14-ROcp58_913*C14);
    ROcp58_115 = ROcp58_113*C15+ROcp58_414*S15;
    ROcp58_215 = ROcp58_213*C15+ROcp58_514*S15;
    ROcp58_315 = ROcp58_313*C15+ROcp58_614*S15;
    ROcp58_415 = -(ROcp58_113*S15-ROcp58_414*C15);
    ROcp58_515 = -(ROcp58_213*S15-ROcp58_514*C15);
    ROcp58_615 = -(ROcp58_313*S15-ROcp58_614*C15);
    ROcp58_116 = ROcp58_115*C16-ROcp58_714*S16;
    ROcp58_216 = ROcp58_215*C16-ROcp58_814*S16;
    ROcp58_316 = ROcp58_315*C16-ROcp58_914*S16;
    ROcp58_716 = ROcp58_115*S16+ROcp58_714*C16;
    ROcp58_816 = ROcp58_215*S16+ROcp58_814*C16;
    ROcp58_916 = ROcp58_315*S16+ROcp58_914*C16;
    ROcp58_417 = ROcp58_415*C17+ROcp58_716*S17;
    ROcp58_517 = ROcp58_515*C17+ROcp58_816*S17;
    ROcp58_617 = ROcp58_615*C17+ROcp58_916*S17;
    ROcp58_717 = -(ROcp58_415*S17-ROcp58_716*C17);
    ROcp58_817 = -(ROcp58_515*S17-ROcp58_816*C17);
    ROcp58_917 = -(ROcp58_615*S17-ROcp58_916*C17);
    ROcp58_118 = ROcp58_116*C18-ROcp58_717*S18;
    ROcp58_218 = ROcp58_216*C18-ROcp58_817*S18;
    ROcp58_318 = ROcp58_316*C18-ROcp58_917*S18;
    ROcp58_718 = ROcp58_116*S18+ROcp58_717*C18;
    ROcp58_818 = ROcp58_216*S18+ROcp58_817*C18;
    ROcp58_918 = ROcp58_316*S18+ROcp58_917*C18;
    RLcp58_113 = s->dpt[1][2]*ROcp58_16+s->dpt[3][2]*S5+ROcp58_46*s->dpt[2][2];
    RLcp58_213 = s->dpt[1][2]*ROcp58_26+s->dpt[3][2]*ROcp58_85+ROcp58_56*s->dpt[2][2];
    RLcp58_313 = s->dpt[1][2]*ROcp58_36+s->dpt[3][2]*ROcp58_95+ROcp58_66*s->dpt[2][2];
    OMcp58_113 = OMcp58_16+ROcp58_46*qd[13];
    OMcp58_213 = OMcp58_26+ROcp58_56*qd[13];
    OMcp58_313 = OMcp58_36+ROcp58_66*qd[13];
    ORcp58_113 = OMcp58_26*RLcp58_313-OMcp58_36*RLcp58_213;
    ORcp58_213 = -(OMcp58_16*RLcp58_313-OMcp58_36*RLcp58_113);
    ORcp58_313 = OMcp58_16*RLcp58_213-OMcp58_26*RLcp58_113;
    OPcp58_113 = OPcp58_16+ROcp58_46*qdd[13]+qd[13]*(OMcp58_26*ROcp58_66-OMcp58_36*ROcp58_56);
    OPcp58_213 = OPcp58_26+ROcp58_56*qdd[13]-qd[13]*(OMcp58_16*ROcp58_66-OMcp58_36*ROcp58_46);
    OPcp58_313 = OPcp58_36+ROcp58_66*qdd[13]+qd[13]*(OMcp58_16*ROcp58_56-OMcp58_26*ROcp58_46);
    RLcp58_114 = s->dpt[1][22]*ROcp58_113+s->dpt[3][22]*ROcp58_713+ROcp58_46*s->dpt[2][22];
    RLcp58_214 = s->dpt[1][22]*ROcp58_213+s->dpt[3][22]*ROcp58_813+ROcp58_56*s->dpt[2][22];
    RLcp58_314 = s->dpt[1][22]*ROcp58_313+s->dpt[3][22]*ROcp58_913+ROcp58_66*s->dpt[2][22];
    OMcp58_114 = OMcp58_113+ROcp58_113*qd[14];
    OMcp58_214 = OMcp58_213+ROcp58_213*qd[14];
    OMcp58_314 = OMcp58_313+ROcp58_313*qd[14];
    ORcp58_114 = OMcp58_213*RLcp58_314-OMcp58_313*RLcp58_214;
    ORcp58_214 = -(OMcp58_113*RLcp58_314-OMcp58_313*RLcp58_114);
    ORcp58_314 = OMcp58_113*RLcp58_214-OMcp58_213*RLcp58_114;
    OPcp58_114 = OPcp58_113+ROcp58_113*qdd[14]+qd[14]*(OMcp58_213*ROcp58_313-OMcp58_313*ROcp58_213);
    OPcp58_214 = OPcp58_213+ROcp58_213*qdd[14]-qd[14]*(OMcp58_113*ROcp58_313-OMcp58_313*ROcp58_113);
    OPcp58_314 = OPcp58_313+ROcp58_313*qdd[14]+qd[14]*(OMcp58_113*ROcp58_213-OMcp58_213*ROcp58_113);
    RLcp58_115 = s->dpt[1][24]*ROcp58_113+s->dpt[2][24]*ROcp58_414+ROcp58_714*s->dpt[3][24];
    RLcp58_215 = s->dpt[1][24]*ROcp58_213+s->dpt[2][24]*ROcp58_514+ROcp58_814*s->dpt[3][24];
    RLcp58_315 = s->dpt[1][24]*ROcp58_313+s->dpt[2][24]*ROcp58_614+ROcp58_914*s->dpt[3][24];
    OMcp58_115 = OMcp58_114+ROcp58_714*qd[15];
    OMcp58_215 = OMcp58_214+ROcp58_814*qd[15];
    OMcp58_315 = OMcp58_314+ROcp58_914*qd[15];
    ORcp58_115 = OMcp58_214*RLcp58_315-OMcp58_314*RLcp58_215;
    ORcp58_215 = -(OMcp58_114*RLcp58_315-OMcp58_314*RLcp58_115);
    ORcp58_315 = OMcp58_114*RLcp58_215-OMcp58_214*RLcp58_115;
    OPcp58_115 = OPcp58_114+ROcp58_714*qdd[15]+qd[15]*(OMcp58_214*ROcp58_914-OMcp58_314*ROcp58_814);
    OPcp58_215 = OPcp58_214+ROcp58_814*qdd[15]-qd[15]*(OMcp58_114*ROcp58_914-OMcp58_314*ROcp58_714);
    OPcp58_315 = OPcp58_314+ROcp58_914*qdd[15]+qd[15]*(OMcp58_114*ROcp58_814-OMcp58_214*ROcp58_714);
    RLcp58_116 = s->dpt[1][26]*ROcp58_115+s->dpt[2][26]*ROcp58_415+ROcp58_714*s->dpt[3][26];
    RLcp58_216 = s->dpt[1][26]*ROcp58_215+s->dpt[2][26]*ROcp58_515+ROcp58_814*s->dpt[3][26];
    RLcp58_316 = s->dpt[1][26]*ROcp58_315+s->dpt[2][26]*ROcp58_615+ROcp58_914*s->dpt[3][26];
    OMcp58_116 = OMcp58_115+ROcp58_415*qd[16];
    OMcp58_216 = OMcp58_215+ROcp58_515*qd[16];
    OMcp58_316 = OMcp58_315+ROcp58_615*qd[16];
    ORcp58_116 = OMcp58_215*RLcp58_316-OMcp58_315*RLcp58_216;
    ORcp58_216 = -(OMcp58_115*RLcp58_316-OMcp58_315*RLcp58_116);
    ORcp58_316 = OMcp58_115*RLcp58_216-OMcp58_215*RLcp58_116;
    OPcp58_116 = OPcp58_115+ROcp58_415*qdd[16]+qd[16]*(OMcp58_215*ROcp58_615-OMcp58_315*ROcp58_515);
    OPcp58_216 = OPcp58_215+ROcp58_515*qdd[16]-qd[16]*(OMcp58_115*ROcp58_615-OMcp58_315*ROcp58_415);
    OPcp58_316 = OPcp58_315+ROcp58_615*qdd[16]+qd[16]*(OMcp58_115*ROcp58_515-OMcp58_215*ROcp58_415);
    RLcp58_117 = s->dpt[1][28]*ROcp58_116+s->dpt[2][28]*ROcp58_415+ROcp58_716*s->dpt[3][28];
    RLcp58_217 = s->dpt[1][28]*ROcp58_216+s->dpt[2][28]*ROcp58_515+ROcp58_816*s->dpt[3][28];
    RLcp58_317 = s->dpt[1][28]*ROcp58_316+s->dpt[2][28]*ROcp58_615+ROcp58_916*s->dpt[3][28];
    OMcp58_117 = OMcp58_116+ROcp58_116*qd[17];
    OMcp58_217 = OMcp58_216+ROcp58_216*qd[17];
    OMcp58_317 = OMcp58_316+ROcp58_316*qd[17];
    ORcp58_117 = OMcp58_216*RLcp58_317-OMcp58_316*RLcp58_217;
    ORcp58_217 = -(OMcp58_116*RLcp58_317-OMcp58_316*RLcp58_117);
    ORcp58_317 = OMcp58_116*RLcp58_217-OMcp58_216*RLcp58_117;
    OPcp58_117 = OPcp58_116+ROcp58_116*qdd[17]+qd[17]*(OMcp58_216*ROcp58_316-OMcp58_316*ROcp58_216);
    OPcp58_217 = OPcp58_216+ROcp58_216*qdd[17]-qd[17]*(OMcp58_116*ROcp58_316-OMcp58_316*ROcp58_116);
    OPcp58_317 = OPcp58_316+ROcp58_316*qdd[17]+qd[17]*(OMcp58_116*ROcp58_216-OMcp58_216*ROcp58_116);
    RLcp58_118 = s->dpt[1][30]*ROcp58_116+s->dpt[2][30]*ROcp58_417+s->dpt[3][30]*ROcp58_717;
    RLcp58_218 = s->dpt[1][30]*ROcp58_216+s->dpt[2][30]*ROcp58_517+s->dpt[3][30]*ROcp58_817;
    RLcp58_318 = s->dpt[1][30]*ROcp58_316+s->dpt[2][30]*ROcp58_617+s->dpt[3][30]*ROcp58_917;
    OMcp58_118 = OMcp58_117+ROcp58_417*qd[18];
    OMcp58_218 = OMcp58_217+ROcp58_517*qd[18];
    OMcp58_318 = OMcp58_317+ROcp58_617*qd[18];
    ORcp58_118 = OMcp58_217*RLcp58_318-OMcp58_317*RLcp58_218;
    ORcp58_218 = -(OMcp58_117*RLcp58_318-OMcp58_317*RLcp58_118);
    ORcp58_318 = OMcp58_117*RLcp58_218-OMcp58_217*RLcp58_118;
    OPcp58_118 = OPcp58_117+ROcp58_417*qdd[18]+qd[18]*(OMcp58_217*ROcp58_617-OMcp58_317*ROcp58_517);
    OPcp58_218 = OPcp58_217+ROcp58_517*qdd[18]-qd[18]*(OMcp58_117*ROcp58_617-OMcp58_317*ROcp58_417);
    OPcp58_318 = OPcp58_317+ROcp58_617*qdd[18]+qd[18]*(OMcp58_117*ROcp58_517-OMcp58_217*ROcp58_417);
    RLcp58_194 = ROcp58_118*s->dpt[1][35]+ROcp58_417*s->dpt[2][35]+ROcp58_718*s->dpt[3][35];
    RLcp58_294 = ROcp58_218*s->dpt[1][35]+ROcp58_517*s->dpt[2][35]+ROcp58_818*s->dpt[3][35];
    RLcp58_394 = ROcp58_318*s->dpt[1][35]+ROcp58_617*s->dpt[2][35]+ROcp58_918*s->dpt[3][35];
    POcp58_194 = RLcp58_113+RLcp58_114+RLcp58_115+RLcp58_116+RLcp58_117+RLcp58_118+RLcp58_194+q[1];
    POcp58_294 = RLcp58_213+RLcp58_214+RLcp58_215+RLcp58_216+RLcp58_217+RLcp58_218+RLcp58_294+q[2];
    POcp58_394 = RLcp58_313+RLcp58_314+RLcp58_315+RLcp58_316+RLcp58_317+RLcp58_318+RLcp58_394+q[3];
    ORcp58_194 = OMcp58_218*RLcp58_394-OMcp58_318*RLcp58_294;
    ORcp58_294 = -(OMcp58_118*RLcp58_394-OMcp58_318*RLcp58_194);
    ORcp58_394 = OMcp58_118*RLcp58_294-OMcp58_218*RLcp58_194;
    VIcp58_194 = ORcp58_113+ORcp58_114+ORcp58_115+ORcp58_116+ORcp58_117+ORcp58_118+ORcp58_194+qd[1];
    VIcp58_294 = ORcp58_213+ORcp58_214+ORcp58_215+ORcp58_216+ORcp58_217+ORcp58_218+ORcp58_294+qd[2];
    VIcp58_394 = ORcp58_313+ORcp58_314+ORcp58_315+ORcp58_316+ORcp58_317+ORcp58_318+ORcp58_394+qd[3];
    ACcp58_194 = qdd[1]+OMcp58_213*ORcp58_314+OMcp58_214*ORcp58_315+OMcp58_215*ORcp58_316+OMcp58_216*ORcp58_317+OMcp58_217
 *ORcp58_318+OMcp58_218*ORcp58_394+OMcp58_26*ORcp58_313-OMcp58_313*ORcp58_214-OMcp58_314*ORcp58_215-OMcp58_315*ORcp58_216-
 OMcp58_316*ORcp58_217-OMcp58_317*ORcp58_218-OMcp58_318*ORcp58_294-OMcp58_36*ORcp58_213+OPcp58_213*RLcp58_314+OPcp58_214*
 RLcp58_315+OPcp58_215*RLcp58_316+OPcp58_216*RLcp58_317+OPcp58_217*RLcp58_318+OPcp58_218*RLcp58_394+OPcp58_26*RLcp58_313-
 OPcp58_313*RLcp58_214-OPcp58_314*RLcp58_215-OPcp58_315*RLcp58_216-OPcp58_316*RLcp58_217-OPcp58_317*RLcp58_218-OPcp58_318*
 RLcp58_294-OPcp58_36*RLcp58_213;
    ACcp58_294 = qdd[2]-OMcp58_113*ORcp58_314-OMcp58_114*ORcp58_315-OMcp58_115*ORcp58_316-OMcp58_116*ORcp58_317-OMcp58_117
 *ORcp58_318-OMcp58_118*ORcp58_394-OMcp58_16*ORcp58_313+OMcp58_313*ORcp58_114+OMcp58_314*ORcp58_115+OMcp58_315*ORcp58_116+
 OMcp58_316*ORcp58_117+OMcp58_317*ORcp58_118+OMcp58_318*ORcp58_194+OMcp58_36*ORcp58_113-OPcp58_113*RLcp58_314-OPcp58_114*
 RLcp58_315-OPcp58_115*RLcp58_316-OPcp58_116*RLcp58_317-OPcp58_117*RLcp58_318-OPcp58_118*RLcp58_394-OPcp58_16*RLcp58_313+
 OPcp58_313*RLcp58_114+OPcp58_314*RLcp58_115+OPcp58_315*RLcp58_116+OPcp58_316*RLcp58_117+OPcp58_317*RLcp58_118+OPcp58_318*
 RLcp58_194+OPcp58_36*RLcp58_113;
    ACcp58_394 = qdd[3]+OMcp58_113*ORcp58_214+OMcp58_114*ORcp58_215+OMcp58_115*ORcp58_216+OMcp58_116*ORcp58_217+OMcp58_117
 *ORcp58_218+OMcp58_118*ORcp58_294+OMcp58_16*ORcp58_213-OMcp58_213*ORcp58_114-OMcp58_214*ORcp58_115-OMcp58_215*ORcp58_116-
 OMcp58_216*ORcp58_117-OMcp58_217*ORcp58_118-OMcp58_218*ORcp58_194-OMcp58_26*ORcp58_113+OPcp58_113*RLcp58_214+OPcp58_114*
 RLcp58_215+OPcp58_115*RLcp58_216+OPcp58_116*RLcp58_217+OPcp58_117*RLcp58_218+OPcp58_118*RLcp58_294+OPcp58_16*RLcp58_213-
 OPcp58_213*RLcp58_114-OPcp58_214*RLcp58_115-OPcp58_215*RLcp58_116-OPcp58_216*RLcp58_117-OPcp58_217*RLcp58_118-OPcp58_218*
 RLcp58_194-OPcp58_26*RLcp58_113;

// = = Block_1_0_0_59_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp58_194;
    sens->P[2] = POcp58_294;
    sens->P[3] = POcp58_394;
    sens->R[1][1] = ROcp58_118;
    sens->R[1][2] = ROcp58_218;
    sens->R[1][3] = ROcp58_318;
    sens->R[2][1] = ROcp58_417;
    sens->R[2][2] = ROcp58_517;
    sens->R[2][3] = ROcp58_617;
    sens->R[3][1] = ROcp58_718;
    sens->R[3][2] = ROcp58_818;
    sens->R[3][3] = ROcp58_918;
    sens->V[1] = VIcp58_194;
    sens->V[2] = VIcp58_294;
    sens->V[3] = VIcp58_394;
    sens->OM[1] = OMcp58_118;
    sens->OM[2] = OMcp58_218;
    sens->OM[3] = OMcp58_318;
    sens->A[1] = ACcp58_194;
    sens->A[2] = ACcp58_294;
    sens->A[3] = ACcp58_394;
    sens->OMP[1] = OPcp58_118;
    sens->OMP[2] = OPcp58_218;
    sens->OMP[3] = OPcp58_318;
 
// 
break;
case 60:
 


// = = Block_1_0_0_60_0_1 = = 
 
// Sensor Kinematics 


    ROcp59_25 = S4*S5;
    ROcp59_35 = -C4*S5;
    ROcp59_85 = -S4*C5;
    ROcp59_95 = C4*C5;
    ROcp59_16 = C5*C6;
    ROcp59_26 = ROcp59_25*C6+C4*S6;
    ROcp59_36 = ROcp59_35*C6+S4*S6;
    ROcp59_46 = -C5*S6;
    ROcp59_56 = -(ROcp59_25*S6-C4*C6);
    ROcp59_66 = -(ROcp59_35*S6-S4*C6);
    OMcp59_25 = qd[5]*C4;
    OMcp59_35 = qd[5]*S4;
    OMcp59_16 = qd[4]+qd[6]*S5;
    OMcp59_26 = OMcp59_25+ROcp59_85*qd[6];
    OMcp59_36 = OMcp59_35+ROcp59_95*qd[6];
    OPcp59_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp59_26 = ROcp59_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp59_35*S5-ROcp59_95*qd[4]);
    OPcp59_36 = ROcp59_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp59_25*S5-ROcp59_85*qd[4]);

// = = Block_1_0_0_60_0_3 = = 
 
// Sensor Kinematics 


    ROcp59_113 = ROcp59_16*C13-S13*S5;
    ROcp59_213 = ROcp59_26*C13-ROcp59_85*S13;
    ROcp59_313 = ROcp59_36*C13-ROcp59_95*S13;
    ROcp59_713 = ROcp59_16*S13+C13*S5;
    ROcp59_813 = ROcp59_26*S13+ROcp59_85*C13;
    ROcp59_913 = ROcp59_36*S13+ROcp59_95*C13;
    ROcp59_414 = ROcp59_46*C14+ROcp59_713*S14;
    ROcp59_514 = ROcp59_56*C14+ROcp59_813*S14;
    ROcp59_614 = ROcp59_66*C14+ROcp59_913*S14;
    ROcp59_714 = -(ROcp59_46*S14-ROcp59_713*C14);
    ROcp59_814 = -(ROcp59_56*S14-ROcp59_813*C14);
    ROcp59_914 = -(ROcp59_66*S14-ROcp59_913*C14);
    ROcp59_115 = ROcp59_113*C15+ROcp59_414*S15;
    ROcp59_215 = ROcp59_213*C15+ROcp59_514*S15;
    ROcp59_315 = ROcp59_313*C15+ROcp59_614*S15;
    ROcp59_415 = -(ROcp59_113*S15-ROcp59_414*C15);
    ROcp59_515 = -(ROcp59_213*S15-ROcp59_514*C15);
    ROcp59_615 = -(ROcp59_313*S15-ROcp59_614*C15);
    ROcp59_116 = ROcp59_115*C16-ROcp59_714*S16;
    ROcp59_216 = ROcp59_215*C16-ROcp59_814*S16;
    ROcp59_316 = ROcp59_315*C16-ROcp59_914*S16;
    ROcp59_716 = ROcp59_115*S16+ROcp59_714*C16;
    ROcp59_816 = ROcp59_215*S16+ROcp59_814*C16;
    ROcp59_916 = ROcp59_315*S16+ROcp59_914*C16;
    ROcp59_417 = ROcp59_415*C17+ROcp59_716*S17;
    ROcp59_517 = ROcp59_515*C17+ROcp59_816*S17;
    ROcp59_617 = ROcp59_615*C17+ROcp59_916*S17;
    ROcp59_717 = -(ROcp59_415*S17-ROcp59_716*C17);
    ROcp59_817 = -(ROcp59_515*S17-ROcp59_816*C17);
    ROcp59_917 = -(ROcp59_615*S17-ROcp59_916*C17);
    ROcp59_118 = ROcp59_116*C18-ROcp59_717*S18;
    ROcp59_218 = ROcp59_216*C18-ROcp59_817*S18;
    ROcp59_318 = ROcp59_316*C18-ROcp59_917*S18;
    ROcp59_718 = ROcp59_116*S18+ROcp59_717*C18;
    ROcp59_818 = ROcp59_216*S18+ROcp59_817*C18;
    ROcp59_918 = ROcp59_316*S18+ROcp59_917*C18;
    RLcp59_113 = s->dpt[1][2]*ROcp59_16+s->dpt[3][2]*S5+ROcp59_46*s->dpt[2][2];
    RLcp59_213 = s->dpt[1][2]*ROcp59_26+s->dpt[3][2]*ROcp59_85+ROcp59_56*s->dpt[2][2];
    RLcp59_313 = s->dpt[1][2]*ROcp59_36+s->dpt[3][2]*ROcp59_95+ROcp59_66*s->dpt[2][2];
    OMcp59_113 = OMcp59_16+ROcp59_46*qd[13];
    OMcp59_213 = OMcp59_26+ROcp59_56*qd[13];
    OMcp59_313 = OMcp59_36+ROcp59_66*qd[13];
    ORcp59_113 = OMcp59_26*RLcp59_313-OMcp59_36*RLcp59_213;
    ORcp59_213 = -(OMcp59_16*RLcp59_313-OMcp59_36*RLcp59_113);
    ORcp59_313 = OMcp59_16*RLcp59_213-OMcp59_26*RLcp59_113;
    OPcp59_113 = OPcp59_16+ROcp59_46*qdd[13]+qd[13]*(OMcp59_26*ROcp59_66-OMcp59_36*ROcp59_56);
    OPcp59_213 = OPcp59_26+ROcp59_56*qdd[13]-qd[13]*(OMcp59_16*ROcp59_66-OMcp59_36*ROcp59_46);
    OPcp59_313 = OPcp59_36+ROcp59_66*qdd[13]+qd[13]*(OMcp59_16*ROcp59_56-OMcp59_26*ROcp59_46);
    RLcp59_114 = s->dpt[1][22]*ROcp59_113+s->dpt[3][22]*ROcp59_713+ROcp59_46*s->dpt[2][22];
    RLcp59_214 = s->dpt[1][22]*ROcp59_213+s->dpt[3][22]*ROcp59_813+ROcp59_56*s->dpt[2][22];
    RLcp59_314 = s->dpt[1][22]*ROcp59_313+s->dpt[3][22]*ROcp59_913+ROcp59_66*s->dpt[2][22];
    OMcp59_114 = OMcp59_113+ROcp59_113*qd[14];
    OMcp59_214 = OMcp59_213+ROcp59_213*qd[14];
    OMcp59_314 = OMcp59_313+ROcp59_313*qd[14];
    ORcp59_114 = OMcp59_213*RLcp59_314-OMcp59_313*RLcp59_214;
    ORcp59_214 = -(OMcp59_113*RLcp59_314-OMcp59_313*RLcp59_114);
    ORcp59_314 = OMcp59_113*RLcp59_214-OMcp59_213*RLcp59_114;
    OPcp59_114 = OPcp59_113+ROcp59_113*qdd[14]+qd[14]*(OMcp59_213*ROcp59_313-OMcp59_313*ROcp59_213);
    OPcp59_214 = OPcp59_213+ROcp59_213*qdd[14]-qd[14]*(OMcp59_113*ROcp59_313-OMcp59_313*ROcp59_113);
    OPcp59_314 = OPcp59_313+ROcp59_313*qdd[14]+qd[14]*(OMcp59_113*ROcp59_213-OMcp59_213*ROcp59_113);
    RLcp59_115 = s->dpt[1][24]*ROcp59_113+s->dpt[2][24]*ROcp59_414+ROcp59_714*s->dpt[3][24];
    RLcp59_215 = s->dpt[1][24]*ROcp59_213+s->dpt[2][24]*ROcp59_514+ROcp59_814*s->dpt[3][24];
    RLcp59_315 = s->dpt[1][24]*ROcp59_313+s->dpt[2][24]*ROcp59_614+ROcp59_914*s->dpt[3][24];
    OMcp59_115 = OMcp59_114+ROcp59_714*qd[15];
    OMcp59_215 = OMcp59_214+ROcp59_814*qd[15];
    OMcp59_315 = OMcp59_314+ROcp59_914*qd[15];
    ORcp59_115 = OMcp59_214*RLcp59_315-OMcp59_314*RLcp59_215;
    ORcp59_215 = -(OMcp59_114*RLcp59_315-OMcp59_314*RLcp59_115);
    ORcp59_315 = OMcp59_114*RLcp59_215-OMcp59_214*RLcp59_115;
    OPcp59_115 = OPcp59_114+ROcp59_714*qdd[15]+qd[15]*(OMcp59_214*ROcp59_914-OMcp59_314*ROcp59_814);
    OPcp59_215 = OPcp59_214+ROcp59_814*qdd[15]-qd[15]*(OMcp59_114*ROcp59_914-OMcp59_314*ROcp59_714);
    OPcp59_315 = OPcp59_314+ROcp59_914*qdd[15]+qd[15]*(OMcp59_114*ROcp59_814-OMcp59_214*ROcp59_714);
    RLcp59_116 = s->dpt[1][26]*ROcp59_115+s->dpt[2][26]*ROcp59_415+ROcp59_714*s->dpt[3][26];
    RLcp59_216 = s->dpt[1][26]*ROcp59_215+s->dpt[2][26]*ROcp59_515+ROcp59_814*s->dpt[3][26];
    RLcp59_316 = s->dpt[1][26]*ROcp59_315+s->dpt[2][26]*ROcp59_615+ROcp59_914*s->dpt[3][26];
    OMcp59_116 = OMcp59_115+ROcp59_415*qd[16];
    OMcp59_216 = OMcp59_215+ROcp59_515*qd[16];
    OMcp59_316 = OMcp59_315+ROcp59_615*qd[16];
    ORcp59_116 = OMcp59_215*RLcp59_316-OMcp59_315*RLcp59_216;
    ORcp59_216 = -(OMcp59_115*RLcp59_316-OMcp59_315*RLcp59_116);
    ORcp59_316 = OMcp59_115*RLcp59_216-OMcp59_215*RLcp59_116;
    OPcp59_116 = OPcp59_115+ROcp59_415*qdd[16]+qd[16]*(OMcp59_215*ROcp59_615-OMcp59_315*ROcp59_515);
    OPcp59_216 = OPcp59_215+ROcp59_515*qdd[16]-qd[16]*(OMcp59_115*ROcp59_615-OMcp59_315*ROcp59_415);
    OPcp59_316 = OPcp59_315+ROcp59_615*qdd[16]+qd[16]*(OMcp59_115*ROcp59_515-OMcp59_215*ROcp59_415);
    RLcp59_117 = s->dpt[1][28]*ROcp59_116+s->dpt[2][28]*ROcp59_415+ROcp59_716*s->dpt[3][28];
    RLcp59_217 = s->dpt[1][28]*ROcp59_216+s->dpt[2][28]*ROcp59_515+ROcp59_816*s->dpt[3][28];
    RLcp59_317 = s->dpt[1][28]*ROcp59_316+s->dpt[2][28]*ROcp59_615+ROcp59_916*s->dpt[3][28];
    OMcp59_117 = OMcp59_116+ROcp59_116*qd[17];
    OMcp59_217 = OMcp59_216+ROcp59_216*qd[17];
    OMcp59_317 = OMcp59_316+ROcp59_316*qd[17];
    ORcp59_117 = OMcp59_216*RLcp59_317-OMcp59_316*RLcp59_217;
    ORcp59_217 = -(OMcp59_116*RLcp59_317-OMcp59_316*RLcp59_117);
    ORcp59_317 = OMcp59_116*RLcp59_217-OMcp59_216*RLcp59_117;
    OPcp59_117 = OPcp59_116+ROcp59_116*qdd[17]+qd[17]*(OMcp59_216*ROcp59_316-OMcp59_316*ROcp59_216);
    OPcp59_217 = OPcp59_216+ROcp59_216*qdd[17]-qd[17]*(OMcp59_116*ROcp59_316-OMcp59_316*ROcp59_116);
    OPcp59_317 = OPcp59_316+ROcp59_316*qdd[17]+qd[17]*(OMcp59_116*ROcp59_216-OMcp59_216*ROcp59_116);
    RLcp59_118 = s->dpt[1][30]*ROcp59_116+s->dpt[2][30]*ROcp59_417+s->dpt[3][30]*ROcp59_717;
    RLcp59_218 = s->dpt[1][30]*ROcp59_216+s->dpt[2][30]*ROcp59_517+s->dpt[3][30]*ROcp59_817;
    RLcp59_318 = s->dpt[1][30]*ROcp59_316+s->dpt[2][30]*ROcp59_617+s->dpt[3][30]*ROcp59_917;
    OMcp59_118 = OMcp59_117+ROcp59_417*qd[18];
    OMcp59_218 = OMcp59_217+ROcp59_517*qd[18];
    OMcp59_318 = OMcp59_317+ROcp59_617*qd[18];
    ORcp59_118 = OMcp59_217*RLcp59_318-OMcp59_317*RLcp59_218;
    ORcp59_218 = -(OMcp59_117*RLcp59_318-OMcp59_317*RLcp59_118);
    ORcp59_318 = OMcp59_117*RLcp59_218-OMcp59_217*RLcp59_118;
    OPcp59_118 = OPcp59_117+ROcp59_417*qdd[18]+qd[18]*(OMcp59_217*ROcp59_617-OMcp59_317*ROcp59_517);
    OPcp59_218 = OPcp59_217+ROcp59_517*qdd[18]-qd[18]*(OMcp59_117*ROcp59_617-OMcp59_317*ROcp59_417);
    OPcp59_318 = OPcp59_317+ROcp59_617*qdd[18]+qd[18]*(OMcp59_117*ROcp59_517-OMcp59_217*ROcp59_417);
    RLcp59_195 = ROcp59_118*s->dpt[1][36]+ROcp59_417*s->dpt[2][36]+ROcp59_718*s->dpt[3][36];
    RLcp59_295 = ROcp59_218*s->dpt[1][36]+ROcp59_517*s->dpt[2][36]+ROcp59_818*s->dpt[3][36];
    RLcp59_395 = ROcp59_318*s->dpt[1][36]+ROcp59_617*s->dpt[2][36]+ROcp59_918*s->dpt[3][36];
    POcp59_195 = RLcp59_113+RLcp59_114+RLcp59_115+RLcp59_116+RLcp59_117+RLcp59_118+RLcp59_195+q[1];
    POcp59_295 = RLcp59_213+RLcp59_214+RLcp59_215+RLcp59_216+RLcp59_217+RLcp59_218+RLcp59_295+q[2];
    POcp59_395 = RLcp59_313+RLcp59_314+RLcp59_315+RLcp59_316+RLcp59_317+RLcp59_318+RLcp59_395+q[3];
    ORcp59_195 = OMcp59_218*RLcp59_395-OMcp59_318*RLcp59_295;
    ORcp59_295 = -(OMcp59_118*RLcp59_395-OMcp59_318*RLcp59_195);
    ORcp59_395 = OMcp59_118*RLcp59_295-OMcp59_218*RLcp59_195;
    VIcp59_195 = ORcp59_113+ORcp59_114+ORcp59_115+ORcp59_116+ORcp59_117+ORcp59_118+ORcp59_195+qd[1];
    VIcp59_295 = ORcp59_213+ORcp59_214+ORcp59_215+ORcp59_216+ORcp59_217+ORcp59_218+ORcp59_295+qd[2];
    VIcp59_395 = ORcp59_313+ORcp59_314+ORcp59_315+ORcp59_316+ORcp59_317+ORcp59_318+ORcp59_395+qd[3];
    ACcp59_195 = qdd[1]+OMcp59_213*ORcp59_314+OMcp59_214*ORcp59_315+OMcp59_215*ORcp59_316+OMcp59_216*ORcp59_317+OMcp59_217
 *ORcp59_318+OMcp59_218*ORcp59_395+OMcp59_26*ORcp59_313-OMcp59_313*ORcp59_214-OMcp59_314*ORcp59_215-OMcp59_315*ORcp59_216-
 OMcp59_316*ORcp59_217-OMcp59_317*ORcp59_218-OMcp59_318*ORcp59_295-OMcp59_36*ORcp59_213+OPcp59_213*RLcp59_314+OPcp59_214*
 RLcp59_315+OPcp59_215*RLcp59_316+OPcp59_216*RLcp59_317+OPcp59_217*RLcp59_318+OPcp59_218*RLcp59_395+OPcp59_26*RLcp59_313-
 OPcp59_313*RLcp59_214-OPcp59_314*RLcp59_215-OPcp59_315*RLcp59_216-OPcp59_316*RLcp59_217-OPcp59_317*RLcp59_218-OPcp59_318*
 RLcp59_295-OPcp59_36*RLcp59_213;
    ACcp59_295 = qdd[2]-OMcp59_113*ORcp59_314-OMcp59_114*ORcp59_315-OMcp59_115*ORcp59_316-OMcp59_116*ORcp59_317-OMcp59_117
 *ORcp59_318-OMcp59_118*ORcp59_395-OMcp59_16*ORcp59_313+OMcp59_313*ORcp59_114+OMcp59_314*ORcp59_115+OMcp59_315*ORcp59_116+
 OMcp59_316*ORcp59_117+OMcp59_317*ORcp59_118+OMcp59_318*ORcp59_195+OMcp59_36*ORcp59_113-OPcp59_113*RLcp59_314-OPcp59_114*
 RLcp59_315-OPcp59_115*RLcp59_316-OPcp59_116*RLcp59_317-OPcp59_117*RLcp59_318-OPcp59_118*RLcp59_395-OPcp59_16*RLcp59_313+
 OPcp59_313*RLcp59_114+OPcp59_314*RLcp59_115+OPcp59_315*RLcp59_116+OPcp59_316*RLcp59_117+OPcp59_317*RLcp59_118+OPcp59_318*
 RLcp59_195+OPcp59_36*RLcp59_113;
    ACcp59_395 = qdd[3]+OMcp59_113*ORcp59_214+OMcp59_114*ORcp59_215+OMcp59_115*ORcp59_216+OMcp59_116*ORcp59_217+OMcp59_117
 *ORcp59_218+OMcp59_118*ORcp59_295+OMcp59_16*ORcp59_213-OMcp59_213*ORcp59_114-OMcp59_214*ORcp59_115-OMcp59_215*ORcp59_116-
 OMcp59_216*ORcp59_117-OMcp59_217*ORcp59_118-OMcp59_218*ORcp59_195-OMcp59_26*ORcp59_113+OPcp59_113*RLcp59_214+OPcp59_114*
 RLcp59_215+OPcp59_115*RLcp59_216+OPcp59_116*RLcp59_217+OPcp59_117*RLcp59_218+OPcp59_118*RLcp59_295+OPcp59_16*RLcp59_213-
 OPcp59_213*RLcp59_114-OPcp59_214*RLcp59_115-OPcp59_215*RLcp59_116-OPcp59_216*RLcp59_117-OPcp59_217*RLcp59_118-OPcp59_218*
 RLcp59_195-OPcp59_26*RLcp59_113;

// = = Block_1_0_0_60_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp59_195;
    sens->P[2] = POcp59_295;
    sens->P[3] = POcp59_395;
    sens->R[1][1] = ROcp59_118;
    sens->R[1][2] = ROcp59_218;
    sens->R[1][3] = ROcp59_318;
    sens->R[2][1] = ROcp59_417;
    sens->R[2][2] = ROcp59_517;
    sens->R[2][3] = ROcp59_617;
    sens->R[3][1] = ROcp59_718;
    sens->R[3][2] = ROcp59_818;
    sens->R[3][3] = ROcp59_918;
    sens->V[1] = VIcp59_195;
    sens->V[2] = VIcp59_295;
    sens->V[3] = VIcp59_395;
    sens->OM[1] = OMcp59_118;
    sens->OM[2] = OMcp59_218;
    sens->OM[3] = OMcp59_318;
    sens->A[1] = ACcp59_195;
    sens->A[2] = ACcp59_295;
    sens->A[3] = ACcp59_395;
    sens->OMP[1] = OPcp59_118;
    sens->OMP[2] = OPcp59_218;
    sens->OMP[3] = OPcp59_318;
 
// 
break;
case 61:
 


// = = Block_1_0_0_61_0_1 = = 
 
// Sensor Kinematics 


    ROcp60_25 = S4*S5;
    ROcp60_35 = -C4*S5;
    ROcp60_85 = -S4*C5;
    ROcp60_95 = C4*C5;
    ROcp60_16 = C5*C6;
    ROcp60_26 = ROcp60_25*C6+C4*S6;
    ROcp60_36 = ROcp60_35*C6+S4*S6;
    ROcp60_46 = -C5*S6;
    ROcp60_56 = -(ROcp60_25*S6-C4*C6);
    ROcp60_66 = -(ROcp60_35*S6-S4*C6);
    OMcp60_25 = qd[5]*C4;
    OMcp60_35 = qd[5]*S4;
    OMcp60_16 = qd[4]+qd[6]*S5;
    OMcp60_26 = OMcp60_25+ROcp60_85*qd[6];
    OMcp60_36 = OMcp60_35+ROcp60_95*qd[6];
    OPcp60_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp60_26 = ROcp60_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp60_35*S5-ROcp60_95*qd[4]);
    OPcp60_36 = ROcp60_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp60_25*S5-ROcp60_85*qd[4]);

// = = Block_1_0_0_61_0_3 = = 
 
// Sensor Kinematics 


    ROcp60_113 = ROcp60_16*C13-S13*S5;
    ROcp60_213 = ROcp60_26*C13-ROcp60_85*S13;
    ROcp60_313 = ROcp60_36*C13-ROcp60_95*S13;
    ROcp60_713 = ROcp60_16*S13+C13*S5;
    ROcp60_813 = ROcp60_26*S13+ROcp60_85*C13;
    ROcp60_913 = ROcp60_36*S13+ROcp60_95*C13;
    ROcp60_414 = ROcp60_46*C14+ROcp60_713*S14;
    ROcp60_514 = ROcp60_56*C14+ROcp60_813*S14;
    ROcp60_614 = ROcp60_66*C14+ROcp60_913*S14;
    ROcp60_714 = -(ROcp60_46*S14-ROcp60_713*C14);
    ROcp60_814 = -(ROcp60_56*S14-ROcp60_813*C14);
    ROcp60_914 = -(ROcp60_66*S14-ROcp60_913*C14);
    ROcp60_115 = ROcp60_113*C15+ROcp60_414*S15;
    ROcp60_215 = ROcp60_213*C15+ROcp60_514*S15;
    ROcp60_315 = ROcp60_313*C15+ROcp60_614*S15;
    ROcp60_415 = -(ROcp60_113*S15-ROcp60_414*C15);
    ROcp60_515 = -(ROcp60_213*S15-ROcp60_514*C15);
    ROcp60_615 = -(ROcp60_313*S15-ROcp60_614*C15);
    ROcp60_116 = ROcp60_115*C16-ROcp60_714*S16;
    ROcp60_216 = ROcp60_215*C16-ROcp60_814*S16;
    ROcp60_316 = ROcp60_315*C16-ROcp60_914*S16;
    ROcp60_716 = ROcp60_115*S16+ROcp60_714*C16;
    ROcp60_816 = ROcp60_215*S16+ROcp60_814*C16;
    ROcp60_916 = ROcp60_315*S16+ROcp60_914*C16;
    ROcp60_417 = ROcp60_415*C17+ROcp60_716*S17;
    ROcp60_517 = ROcp60_515*C17+ROcp60_816*S17;
    ROcp60_617 = ROcp60_615*C17+ROcp60_916*S17;
    ROcp60_717 = -(ROcp60_415*S17-ROcp60_716*C17);
    ROcp60_817 = -(ROcp60_515*S17-ROcp60_816*C17);
    ROcp60_917 = -(ROcp60_615*S17-ROcp60_916*C17);
    ROcp60_118 = ROcp60_116*C18-ROcp60_717*S18;
    ROcp60_218 = ROcp60_216*C18-ROcp60_817*S18;
    ROcp60_318 = ROcp60_316*C18-ROcp60_917*S18;
    ROcp60_718 = ROcp60_116*S18+ROcp60_717*C18;
    ROcp60_818 = ROcp60_216*S18+ROcp60_817*C18;
    ROcp60_918 = ROcp60_316*S18+ROcp60_917*C18;
    RLcp60_113 = s->dpt[1][2]*ROcp60_16+s->dpt[3][2]*S5+ROcp60_46*s->dpt[2][2];
    RLcp60_213 = s->dpt[1][2]*ROcp60_26+s->dpt[3][2]*ROcp60_85+ROcp60_56*s->dpt[2][2];
    RLcp60_313 = s->dpt[1][2]*ROcp60_36+s->dpt[3][2]*ROcp60_95+ROcp60_66*s->dpt[2][2];
    OMcp60_113 = OMcp60_16+ROcp60_46*qd[13];
    OMcp60_213 = OMcp60_26+ROcp60_56*qd[13];
    OMcp60_313 = OMcp60_36+ROcp60_66*qd[13];
    ORcp60_113 = OMcp60_26*RLcp60_313-OMcp60_36*RLcp60_213;
    ORcp60_213 = -(OMcp60_16*RLcp60_313-OMcp60_36*RLcp60_113);
    ORcp60_313 = OMcp60_16*RLcp60_213-OMcp60_26*RLcp60_113;
    OPcp60_113 = OPcp60_16+ROcp60_46*qdd[13]+qd[13]*(OMcp60_26*ROcp60_66-OMcp60_36*ROcp60_56);
    OPcp60_213 = OPcp60_26+ROcp60_56*qdd[13]-qd[13]*(OMcp60_16*ROcp60_66-OMcp60_36*ROcp60_46);
    OPcp60_313 = OPcp60_36+ROcp60_66*qdd[13]+qd[13]*(OMcp60_16*ROcp60_56-OMcp60_26*ROcp60_46);
    RLcp60_114 = s->dpt[1][22]*ROcp60_113+s->dpt[3][22]*ROcp60_713+ROcp60_46*s->dpt[2][22];
    RLcp60_214 = s->dpt[1][22]*ROcp60_213+s->dpt[3][22]*ROcp60_813+ROcp60_56*s->dpt[2][22];
    RLcp60_314 = s->dpt[1][22]*ROcp60_313+s->dpt[3][22]*ROcp60_913+ROcp60_66*s->dpt[2][22];
    OMcp60_114 = OMcp60_113+ROcp60_113*qd[14];
    OMcp60_214 = OMcp60_213+ROcp60_213*qd[14];
    OMcp60_314 = OMcp60_313+ROcp60_313*qd[14];
    ORcp60_114 = OMcp60_213*RLcp60_314-OMcp60_313*RLcp60_214;
    ORcp60_214 = -(OMcp60_113*RLcp60_314-OMcp60_313*RLcp60_114);
    ORcp60_314 = OMcp60_113*RLcp60_214-OMcp60_213*RLcp60_114;
    OPcp60_114 = OPcp60_113+ROcp60_113*qdd[14]+qd[14]*(OMcp60_213*ROcp60_313-OMcp60_313*ROcp60_213);
    OPcp60_214 = OPcp60_213+ROcp60_213*qdd[14]-qd[14]*(OMcp60_113*ROcp60_313-OMcp60_313*ROcp60_113);
    OPcp60_314 = OPcp60_313+ROcp60_313*qdd[14]+qd[14]*(OMcp60_113*ROcp60_213-OMcp60_213*ROcp60_113);
    RLcp60_115 = s->dpt[1][24]*ROcp60_113+s->dpt[2][24]*ROcp60_414+ROcp60_714*s->dpt[3][24];
    RLcp60_215 = s->dpt[1][24]*ROcp60_213+s->dpt[2][24]*ROcp60_514+ROcp60_814*s->dpt[3][24];
    RLcp60_315 = s->dpt[1][24]*ROcp60_313+s->dpt[2][24]*ROcp60_614+ROcp60_914*s->dpt[3][24];
    OMcp60_115 = OMcp60_114+ROcp60_714*qd[15];
    OMcp60_215 = OMcp60_214+ROcp60_814*qd[15];
    OMcp60_315 = OMcp60_314+ROcp60_914*qd[15];
    ORcp60_115 = OMcp60_214*RLcp60_315-OMcp60_314*RLcp60_215;
    ORcp60_215 = -(OMcp60_114*RLcp60_315-OMcp60_314*RLcp60_115);
    ORcp60_315 = OMcp60_114*RLcp60_215-OMcp60_214*RLcp60_115;
    OPcp60_115 = OPcp60_114+ROcp60_714*qdd[15]+qd[15]*(OMcp60_214*ROcp60_914-OMcp60_314*ROcp60_814);
    OPcp60_215 = OPcp60_214+ROcp60_814*qdd[15]-qd[15]*(OMcp60_114*ROcp60_914-OMcp60_314*ROcp60_714);
    OPcp60_315 = OPcp60_314+ROcp60_914*qdd[15]+qd[15]*(OMcp60_114*ROcp60_814-OMcp60_214*ROcp60_714);
    RLcp60_116 = s->dpt[1][26]*ROcp60_115+s->dpt[2][26]*ROcp60_415+ROcp60_714*s->dpt[3][26];
    RLcp60_216 = s->dpt[1][26]*ROcp60_215+s->dpt[2][26]*ROcp60_515+ROcp60_814*s->dpt[3][26];
    RLcp60_316 = s->dpt[1][26]*ROcp60_315+s->dpt[2][26]*ROcp60_615+ROcp60_914*s->dpt[3][26];
    OMcp60_116 = OMcp60_115+ROcp60_415*qd[16];
    OMcp60_216 = OMcp60_215+ROcp60_515*qd[16];
    OMcp60_316 = OMcp60_315+ROcp60_615*qd[16];
    ORcp60_116 = OMcp60_215*RLcp60_316-OMcp60_315*RLcp60_216;
    ORcp60_216 = -(OMcp60_115*RLcp60_316-OMcp60_315*RLcp60_116);
    ORcp60_316 = OMcp60_115*RLcp60_216-OMcp60_215*RLcp60_116;
    OPcp60_116 = OPcp60_115+ROcp60_415*qdd[16]+qd[16]*(OMcp60_215*ROcp60_615-OMcp60_315*ROcp60_515);
    OPcp60_216 = OPcp60_215+ROcp60_515*qdd[16]-qd[16]*(OMcp60_115*ROcp60_615-OMcp60_315*ROcp60_415);
    OPcp60_316 = OPcp60_315+ROcp60_615*qdd[16]+qd[16]*(OMcp60_115*ROcp60_515-OMcp60_215*ROcp60_415);
    RLcp60_117 = s->dpt[1][28]*ROcp60_116+s->dpt[2][28]*ROcp60_415+ROcp60_716*s->dpt[3][28];
    RLcp60_217 = s->dpt[1][28]*ROcp60_216+s->dpt[2][28]*ROcp60_515+ROcp60_816*s->dpt[3][28];
    RLcp60_317 = s->dpt[1][28]*ROcp60_316+s->dpt[2][28]*ROcp60_615+ROcp60_916*s->dpt[3][28];
    OMcp60_117 = OMcp60_116+ROcp60_116*qd[17];
    OMcp60_217 = OMcp60_216+ROcp60_216*qd[17];
    OMcp60_317 = OMcp60_316+ROcp60_316*qd[17];
    ORcp60_117 = OMcp60_216*RLcp60_317-OMcp60_316*RLcp60_217;
    ORcp60_217 = -(OMcp60_116*RLcp60_317-OMcp60_316*RLcp60_117);
    ORcp60_317 = OMcp60_116*RLcp60_217-OMcp60_216*RLcp60_117;
    OPcp60_117 = OPcp60_116+ROcp60_116*qdd[17]+qd[17]*(OMcp60_216*ROcp60_316-OMcp60_316*ROcp60_216);
    OPcp60_217 = OPcp60_216+ROcp60_216*qdd[17]-qd[17]*(OMcp60_116*ROcp60_316-OMcp60_316*ROcp60_116);
    OPcp60_317 = OPcp60_316+ROcp60_316*qdd[17]+qd[17]*(OMcp60_116*ROcp60_216-OMcp60_216*ROcp60_116);
    RLcp60_118 = s->dpt[1][30]*ROcp60_116+s->dpt[2][30]*ROcp60_417+s->dpt[3][30]*ROcp60_717;
    RLcp60_218 = s->dpt[1][30]*ROcp60_216+s->dpt[2][30]*ROcp60_517+s->dpt[3][30]*ROcp60_817;
    RLcp60_318 = s->dpt[1][30]*ROcp60_316+s->dpt[2][30]*ROcp60_617+s->dpt[3][30]*ROcp60_917;
    OMcp60_118 = OMcp60_117+ROcp60_417*qd[18];
    OMcp60_218 = OMcp60_217+ROcp60_517*qd[18];
    OMcp60_318 = OMcp60_317+ROcp60_617*qd[18];
    ORcp60_118 = OMcp60_217*RLcp60_318-OMcp60_317*RLcp60_218;
    ORcp60_218 = -(OMcp60_117*RLcp60_318-OMcp60_317*RLcp60_118);
    ORcp60_318 = OMcp60_117*RLcp60_218-OMcp60_217*RLcp60_118;
    OPcp60_118 = OPcp60_117+ROcp60_417*qdd[18]+qd[18]*(OMcp60_217*ROcp60_617-OMcp60_317*ROcp60_517);
    OPcp60_218 = OPcp60_217+ROcp60_517*qdd[18]-qd[18]*(OMcp60_117*ROcp60_617-OMcp60_317*ROcp60_417);
    OPcp60_318 = OPcp60_317+ROcp60_617*qdd[18]+qd[18]*(OMcp60_117*ROcp60_517-OMcp60_217*ROcp60_417);
    RLcp60_196 = ROcp60_118*s->dpt[1][37]+ROcp60_417*s->dpt[2][37]+ROcp60_718*s->dpt[3][37];
    RLcp60_296 = ROcp60_218*s->dpt[1][37]+ROcp60_517*s->dpt[2][37]+ROcp60_818*s->dpt[3][37];
    RLcp60_396 = ROcp60_318*s->dpt[1][37]+ROcp60_617*s->dpt[2][37]+ROcp60_918*s->dpt[3][37];
    POcp60_196 = RLcp60_113+RLcp60_114+RLcp60_115+RLcp60_116+RLcp60_117+RLcp60_118+RLcp60_196+q[1];
    POcp60_296 = RLcp60_213+RLcp60_214+RLcp60_215+RLcp60_216+RLcp60_217+RLcp60_218+RLcp60_296+q[2];
    POcp60_396 = RLcp60_313+RLcp60_314+RLcp60_315+RLcp60_316+RLcp60_317+RLcp60_318+RLcp60_396+q[3];
    ORcp60_196 = OMcp60_218*RLcp60_396-OMcp60_318*RLcp60_296;
    ORcp60_296 = -(OMcp60_118*RLcp60_396-OMcp60_318*RLcp60_196);
    ORcp60_396 = OMcp60_118*RLcp60_296-OMcp60_218*RLcp60_196;
    VIcp60_196 = ORcp60_113+ORcp60_114+ORcp60_115+ORcp60_116+ORcp60_117+ORcp60_118+ORcp60_196+qd[1];
    VIcp60_296 = ORcp60_213+ORcp60_214+ORcp60_215+ORcp60_216+ORcp60_217+ORcp60_218+ORcp60_296+qd[2];
    VIcp60_396 = ORcp60_313+ORcp60_314+ORcp60_315+ORcp60_316+ORcp60_317+ORcp60_318+ORcp60_396+qd[3];
    ACcp60_196 = qdd[1]+OMcp60_213*ORcp60_314+OMcp60_214*ORcp60_315+OMcp60_215*ORcp60_316+OMcp60_216*ORcp60_317+OMcp60_217
 *ORcp60_318+OMcp60_218*ORcp60_396+OMcp60_26*ORcp60_313-OMcp60_313*ORcp60_214-OMcp60_314*ORcp60_215-OMcp60_315*ORcp60_216-
 OMcp60_316*ORcp60_217-OMcp60_317*ORcp60_218-OMcp60_318*ORcp60_296-OMcp60_36*ORcp60_213+OPcp60_213*RLcp60_314+OPcp60_214*
 RLcp60_315+OPcp60_215*RLcp60_316+OPcp60_216*RLcp60_317+OPcp60_217*RLcp60_318+OPcp60_218*RLcp60_396+OPcp60_26*RLcp60_313-
 OPcp60_313*RLcp60_214-OPcp60_314*RLcp60_215-OPcp60_315*RLcp60_216-OPcp60_316*RLcp60_217-OPcp60_317*RLcp60_218-OPcp60_318*
 RLcp60_296-OPcp60_36*RLcp60_213;
    ACcp60_296 = qdd[2]-OMcp60_113*ORcp60_314-OMcp60_114*ORcp60_315-OMcp60_115*ORcp60_316-OMcp60_116*ORcp60_317-OMcp60_117
 *ORcp60_318-OMcp60_118*ORcp60_396-OMcp60_16*ORcp60_313+OMcp60_313*ORcp60_114+OMcp60_314*ORcp60_115+OMcp60_315*ORcp60_116+
 OMcp60_316*ORcp60_117+OMcp60_317*ORcp60_118+OMcp60_318*ORcp60_196+OMcp60_36*ORcp60_113-OPcp60_113*RLcp60_314-OPcp60_114*
 RLcp60_315-OPcp60_115*RLcp60_316-OPcp60_116*RLcp60_317-OPcp60_117*RLcp60_318-OPcp60_118*RLcp60_396-OPcp60_16*RLcp60_313+
 OPcp60_313*RLcp60_114+OPcp60_314*RLcp60_115+OPcp60_315*RLcp60_116+OPcp60_316*RLcp60_117+OPcp60_317*RLcp60_118+OPcp60_318*
 RLcp60_196+OPcp60_36*RLcp60_113;
    ACcp60_396 = qdd[3]+OMcp60_113*ORcp60_214+OMcp60_114*ORcp60_215+OMcp60_115*ORcp60_216+OMcp60_116*ORcp60_217+OMcp60_117
 *ORcp60_218+OMcp60_118*ORcp60_296+OMcp60_16*ORcp60_213-OMcp60_213*ORcp60_114-OMcp60_214*ORcp60_115-OMcp60_215*ORcp60_116-
 OMcp60_216*ORcp60_117-OMcp60_217*ORcp60_118-OMcp60_218*ORcp60_196-OMcp60_26*ORcp60_113+OPcp60_113*RLcp60_214+OPcp60_114*
 RLcp60_215+OPcp60_115*RLcp60_216+OPcp60_116*RLcp60_217+OPcp60_117*RLcp60_218+OPcp60_118*RLcp60_296+OPcp60_16*RLcp60_213-
 OPcp60_213*RLcp60_114-OPcp60_214*RLcp60_115-OPcp60_215*RLcp60_116-OPcp60_216*RLcp60_117-OPcp60_217*RLcp60_118-OPcp60_218*
 RLcp60_196-OPcp60_26*RLcp60_113;

// = = Block_1_0_0_61_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp60_196;
    sens->P[2] = POcp60_296;
    sens->P[3] = POcp60_396;
    sens->R[1][1] = ROcp60_118;
    sens->R[1][2] = ROcp60_218;
    sens->R[1][3] = ROcp60_318;
    sens->R[2][1] = ROcp60_417;
    sens->R[2][2] = ROcp60_517;
    sens->R[2][3] = ROcp60_617;
    sens->R[3][1] = ROcp60_718;
    sens->R[3][2] = ROcp60_818;
    sens->R[3][3] = ROcp60_918;
    sens->V[1] = VIcp60_196;
    sens->V[2] = VIcp60_296;
    sens->V[3] = VIcp60_396;
    sens->OM[1] = OMcp60_118;
    sens->OM[2] = OMcp60_218;
    sens->OM[3] = OMcp60_318;
    sens->A[1] = ACcp60_196;
    sens->A[2] = ACcp60_296;
    sens->A[3] = ACcp60_396;
    sens->OMP[1] = OPcp60_118;
    sens->OMP[2] = OPcp60_218;
    sens->OMP[3] = OPcp60_318;
 
// 
break;
case 62:
 


// = = Block_1_0_0_62_0_1 = = 
 
// Sensor Kinematics 


    ROcp61_25 = S4*S5;
    ROcp61_35 = -C4*S5;
    ROcp61_85 = -S4*C5;
    ROcp61_95 = C4*C5;
    ROcp61_16 = C5*C6;
    ROcp61_26 = ROcp61_25*C6+C4*S6;
    ROcp61_36 = ROcp61_35*C6+S4*S6;
    ROcp61_46 = -C5*S6;
    ROcp61_56 = -(ROcp61_25*S6-C4*C6);
    ROcp61_66 = -(ROcp61_35*S6-S4*C6);
    OMcp61_25 = qd[5]*C4;
    OMcp61_35 = qd[5]*S4;
    OMcp61_16 = qd[4]+qd[6]*S5;
    OMcp61_26 = OMcp61_25+ROcp61_85*qd[6];
    OMcp61_36 = OMcp61_35+ROcp61_95*qd[6];
    OPcp61_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp61_26 = ROcp61_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp61_35*S5-ROcp61_95*qd[4]);
    OPcp61_36 = ROcp61_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp61_25*S5-ROcp61_85*qd[4]);

// = = Block_1_0_0_62_0_4 = = 
 
// Sensor Kinematics 


    ROcp61_419 = ROcp61_46*C19+S19*S5;
    ROcp61_519 = ROcp61_56*C19+ROcp61_85*S19;
    ROcp61_619 = ROcp61_66*C19+ROcp61_95*S19;
    ROcp61_719 = -(ROcp61_46*S19-C19*S5);
    ROcp61_819 = -(ROcp61_56*S19-ROcp61_85*C19);
    ROcp61_919 = -(ROcp61_66*S19-ROcp61_95*C19);
    ROcp61_120 = ROcp61_16*C20-ROcp61_719*S20;
    ROcp61_220 = ROcp61_26*C20-ROcp61_819*S20;
    ROcp61_320 = ROcp61_36*C20-ROcp61_919*S20;
    ROcp61_720 = ROcp61_16*S20+ROcp61_719*C20;
    ROcp61_820 = ROcp61_26*S20+ROcp61_819*C20;
    ROcp61_920 = ROcp61_36*S20+ROcp61_919*C20;
    ROcp61_121 = ROcp61_120*C21+ROcp61_419*S21;
    ROcp61_221 = ROcp61_220*C21+ROcp61_519*S21;
    ROcp61_321 = ROcp61_320*C21+ROcp61_619*S21;
    ROcp61_421 = -(ROcp61_120*S21-ROcp61_419*C21);
    ROcp61_521 = -(ROcp61_220*S21-ROcp61_519*C21);
    ROcp61_621 = -(ROcp61_320*S21-ROcp61_619*C21);
    RLcp61_119 = s->dpt[2][3]*ROcp61_46+ROcp61_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp61_219 = s->dpt[2][3]*ROcp61_56+ROcp61_26*s->dpt[1][3]+ROcp61_85*s->dpt[3][3];
    RLcp61_319 = s->dpt[2][3]*ROcp61_66+ROcp61_36*s->dpt[1][3]+ROcp61_95*s->dpt[3][3];
    OMcp61_119 = OMcp61_16+ROcp61_16*qd[19];
    OMcp61_219 = OMcp61_26+ROcp61_26*qd[19];
    OMcp61_319 = OMcp61_36+ROcp61_36*qd[19];
    ORcp61_119 = OMcp61_26*RLcp61_319-OMcp61_36*RLcp61_219;
    ORcp61_219 = -(OMcp61_16*RLcp61_319-OMcp61_36*RLcp61_119);
    ORcp61_319 = OMcp61_16*RLcp61_219-OMcp61_26*RLcp61_119;
    OPcp61_119 = OPcp61_16+ROcp61_16*qdd[19]+qd[19]*(OMcp61_26*ROcp61_36-OMcp61_36*ROcp61_26);
    OPcp61_219 = OPcp61_26+ROcp61_26*qdd[19]-qd[19]*(OMcp61_16*ROcp61_36-OMcp61_36*ROcp61_16);
    OPcp61_319 = OPcp61_36+ROcp61_36*qdd[19]+qd[19]*(OMcp61_16*ROcp61_26-OMcp61_26*ROcp61_16);
    RLcp61_120 = s->dpt[1][38]*ROcp61_16+s->dpt[2][38]*ROcp61_419+s->dpt[3][38]*ROcp61_719;
    RLcp61_220 = s->dpt[1][38]*ROcp61_26+s->dpt[2][38]*ROcp61_519+s->dpt[3][38]*ROcp61_819;
    RLcp61_320 = s->dpt[1][38]*ROcp61_36+s->dpt[2][38]*ROcp61_619+s->dpt[3][38]*ROcp61_919;
    OMcp61_120 = OMcp61_119+ROcp61_419*qd[20];
    OMcp61_220 = OMcp61_219+ROcp61_519*qd[20];
    OMcp61_320 = OMcp61_319+ROcp61_619*qd[20];
    ORcp61_120 = OMcp61_219*RLcp61_320-OMcp61_319*RLcp61_220;
    ORcp61_220 = -(OMcp61_119*RLcp61_320-OMcp61_319*RLcp61_120);
    ORcp61_320 = OMcp61_119*RLcp61_220-OMcp61_219*RLcp61_120;
    OPcp61_120 = OPcp61_119+ROcp61_419*qdd[20]+qd[20]*(OMcp61_219*ROcp61_619-OMcp61_319*ROcp61_519);
    OPcp61_220 = OPcp61_219+ROcp61_519*qdd[20]-qd[20]*(OMcp61_119*ROcp61_619-OMcp61_319*ROcp61_419);
    OPcp61_320 = OPcp61_319+ROcp61_619*qdd[20]+qd[20]*(OMcp61_119*ROcp61_519-OMcp61_219*ROcp61_419);
    RLcp61_121 = s->dpt[1][40]*ROcp61_120+s->dpt[2][40]*ROcp61_419+ROcp61_720*s->dpt[3][40];
    RLcp61_221 = s->dpt[1][40]*ROcp61_220+s->dpt[2][40]*ROcp61_519+ROcp61_820*s->dpt[3][40];
    RLcp61_321 = s->dpt[1][40]*ROcp61_320+s->dpt[2][40]*ROcp61_619+ROcp61_920*s->dpt[3][40];
    OMcp61_121 = OMcp61_120+ROcp61_720*qd[21];
    OMcp61_221 = OMcp61_220+ROcp61_820*qd[21];
    OMcp61_321 = OMcp61_320+ROcp61_920*qd[21];
    ORcp61_121 = OMcp61_220*RLcp61_321-OMcp61_320*RLcp61_221;
    ORcp61_221 = -(OMcp61_120*RLcp61_321-OMcp61_320*RLcp61_121);
    ORcp61_321 = OMcp61_120*RLcp61_221-OMcp61_220*RLcp61_121;
    OPcp61_121 = OPcp61_120+ROcp61_720*qdd[21]+qd[21]*(OMcp61_220*ROcp61_920-OMcp61_320*ROcp61_820);
    OPcp61_221 = OPcp61_220+ROcp61_820*qdd[21]-qd[21]*(OMcp61_120*ROcp61_920-OMcp61_320*ROcp61_720);
    OPcp61_321 = OPcp61_320+ROcp61_920*qdd[21]+qd[21]*(OMcp61_120*ROcp61_820-OMcp61_220*ROcp61_720);
    RLcp61_197 = s->dpt[2][46]*ROcp61_421+ROcp61_121*s->dpt[1][46]+ROcp61_720*s->dpt[3][46];
    RLcp61_297 = s->dpt[2][46]*ROcp61_521+ROcp61_221*s->dpt[1][46]+ROcp61_820*s->dpt[3][46];
    RLcp61_397 = s->dpt[2][46]*ROcp61_621+ROcp61_321*s->dpt[1][46]+ROcp61_920*s->dpt[3][46];
    POcp61_197 = RLcp61_119+RLcp61_120+RLcp61_121+RLcp61_197+q[1];
    POcp61_297 = RLcp61_219+RLcp61_220+RLcp61_221+RLcp61_297+q[2];
    POcp61_397 = RLcp61_319+RLcp61_320+RLcp61_321+RLcp61_397+q[3];
    ORcp61_197 = OMcp61_221*RLcp61_397-OMcp61_321*RLcp61_297;
    ORcp61_297 = -(OMcp61_121*RLcp61_397-OMcp61_321*RLcp61_197);
    ORcp61_397 = OMcp61_121*RLcp61_297-OMcp61_221*RLcp61_197;
    VIcp61_197 = ORcp61_119+ORcp61_120+ORcp61_121+ORcp61_197+qd[1];
    VIcp61_297 = ORcp61_219+ORcp61_220+ORcp61_221+ORcp61_297+qd[2];
    VIcp61_397 = ORcp61_319+ORcp61_320+ORcp61_321+ORcp61_397+qd[3];
    ACcp61_197 = qdd[1]+OMcp61_219*ORcp61_320+OMcp61_220*ORcp61_321+OMcp61_221*ORcp61_397+OMcp61_26*ORcp61_319-OMcp61_319*
 ORcp61_220-OMcp61_320*ORcp61_221-OMcp61_321*ORcp61_297-OMcp61_36*ORcp61_219+OPcp61_219*RLcp61_320+OPcp61_220*RLcp61_321+
 OPcp61_221*RLcp61_397+OPcp61_26*RLcp61_319-OPcp61_319*RLcp61_220-OPcp61_320*RLcp61_221-OPcp61_321*RLcp61_297-OPcp61_36*
 RLcp61_219;
    ACcp61_297 = qdd[2]-OMcp61_119*ORcp61_320-OMcp61_120*ORcp61_321-OMcp61_121*ORcp61_397-OMcp61_16*ORcp61_319+OMcp61_319*
 ORcp61_120+OMcp61_320*ORcp61_121+OMcp61_321*ORcp61_197+OMcp61_36*ORcp61_119-OPcp61_119*RLcp61_320-OPcp61_120*RLcp61_321-
 OPcp61_121*RLcp61_397-OPcp61_16*RLcp61_319+OPcp61_319*RLcp61_120+OPcp61_320*RLcp61_121+OPcp61_321*RLcp61_197+OPcp61_36*
 RLcp61_119;
    ACcp61_397 = qdd[3]+OMcp61_119*ORcp61_220+OMcp61_120*ORcp61_221+OMcp61_121*ORcp61_297+OMcp61_16*ORcp61_219-OMcp61_219*
 ORcp61_120-OMcp61_220*ORcp61_121-OMcp61_221*ORcp61_197-OMcp61_26*ORcp61_119+OPcp61_119*RLcp61_220+OPcp61_120*RLcp61_221+
 OPcp61_121*RLcp61_297+OPcp61_16*RLcp61_219-OPcp61_219*RLcp61_120-OPcp61_220*RLcp61_121-OPcp61_221*RLcp61_197-OPcp61_26*
 RLcp61_119;

// = = Block_1_0_0_62_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp61_197;
    sens->P[2] = POcp61_297;
    sens->P[3] = POcp61_397;
    sens->R[1][1] = ROcp61_121;
    sens->R[1][2] = ROcp61_221;
    sens->R[1][3] = ROcp61_321;
    sens->R[2][1] = ROcp61_421;
    sens->R[2][2] = ROcp61_521;
    sens->R[2][3] = ROcp61_621;
    sens->R[3][1] = ROcp61_720;
    sens->R[3][2] = ROcp61_820;
    sens->R[3][3] = ROcp61_920;
    sens->V[1] = VIcp61_197;
    sens->V[2] = VIcp61_297;
    sens->V[3] = VIcp61_397;
    sens->OM[1] = OMcp61_121;
    sens->OM[2] = OMcp61_221;
    sens->OM[3] = OMcp61_321;
    sens->A[1] = ACcp61_197;
    sens->A[2] = ACcp61_297;
    sens->A[3] = ACcp61_397;
    sens->OMP[1] = OPcp61_121;
    sens->OMP[2] = OPcp61_221;
    sens->OMP[3] = OPcp61_321;
 
// 
break;
case 63:
 


// = = Block_1_0_0_63_0_1 = = 
 
// Sensor Kinematics 


    ROcp62_25 = S4*S5;
    ROcp62_35 = -C4*S5;
    ROcp62_85 = -S4*C5;
    ROcp62_95 = C4*C5;
    ROcp62_16 = C5*C6;
    ROcp62_26 = ROcp62_25*C6+C4*S6;
    ROcp62_36 = ROcp62_35*C6+S4*S6;
    ROcp62_46 = -C5*S6;
    ROcp62_56 = -(ROcp62_25*S6-C4*C6);
    ROcp62_66 = -(ROcp62_35*S6-S4*C6);
    OMcp62_25 = qd[5]*C4;
    OMcp62_35 = qd[5]*S4;
    OMcp62_16 = qd[4]+qd[6]*S5;
    OMcp62_26 = OMcp62_25+ROcp62_85*qd[6];
    OMcp62_36 = OMcp62_35+ROcp62_95*qd[6];
    OPcp62_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp62_26 = ROcp62_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp62_35*S5-ROcp62_95*qd[4]);
    OPcp62_36 = ROcp62_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp62_25*S5-ROcp62_85*qd[4]);

// = = Block_1_0_0_63_0_4 = = 
 
// Sensor Kinematics 


    ROcp62_419 = ROcp62_46*C19+S19*S5;
    ROcp62_519 = ROcp62_56*C19+ROcp62_85*S19;
    ROcp62_619 = ROcp62_66*C19+ROcp62_95*S19;
    ROcp62_719 = -(ROcp62_46*S19-C19*S5);
    ROcp62_819 = -(ROcp62_56*S19-ROcp62_85*C19);
    ROcp62_919 = -(ROcp62_66*S19-ROcp62_95*C19);
    ROcp62_120 = ROcp62_16*C20-ROcp62_719*S20;
    ROcp62_220 = ROcp62_26*C20-ROcp62_819*S20;
    ROcp62_320 = ROcp62_36*C20-ROcp62_919*S20;
    ROcp62_720 = ROcp62_16*S20+ROcp62_719*C20;
    ROcp62_820 = ROcp62_26*S20+ROcp62_819*C20;
    ROcp62_920 = ROcp62_36*S20+ROcp62_919*C20;
    ROcp62_121 = ROcp62_120*C21+ROcp62_419*S21;
    ROcp62_221 = ROcp62_220*C21+ROcp62_519*S21;
    ROcp62_321 = ROcp62_320*C21+ROcp62_619*S21;
    ROcp62_421 = -(ROcp62_120*S21-ROcp62_419*C21);
    ROcp62_521 = -(ROcp62_220*S21-ROcp62_519*C21);
    ROcp62_621 = -(ROcp62_320*S21-ROcp62_619*C21);
    RLcp62_119 = s->dpt[2][3]*ROcp62_46+ROcp62_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp62_219 = s->dpt[2][3]*ROcp62_56+ROcp62_26*s->dpt[1][3]+ROcp62_85*s->dpt[3][3];
    RLcp62_319 = s->dpt[2][3]*ROcp62_66+ROcp62_36*s->dpt[1][3]+ROcp62_95*s->dpt[3][3];
    OMcp62_119 = OMcp62_16+ROcp62_16*qd[19];
    OMcp62_219 = OMcp62_26+ROcp62_26*qd[19];
    OMcp62_319 = OMcp62_36+ROcp62_36*qd[19];
    ORcp62_119 = OMcp62_26*RLcp62_319-OMcp62_36*RLcp62_219;
    ORcp62_219 = -(OMcp62_16*RLcp62_319-OMcp62_36*RLcp62_119);
    ORcp62_319 = OMcp62_16*RLcp62_219-OMcp62_26*RLcp62_119;
    OPcp62_119 = OPcp62_16+ROcp62_16*qdd[19]+qd[19]*(OMcp62_26*ROcp62_36-OMcp62_36*ROcp62_26);
    OPcp62_219 = OPcp62_26+ROcp62_26*qdd[19]-qd[19]*(OMcp62_16*ROcp62_36-OMcp62_36*ROcp62_16);
    OPcp62_319 = OPcp62_36+ROcp62_36*qdd[19]+qd[19]*(OMcp62_16*ROcp62_26-OMcp62_26*ROcp62_16);
    RLcp62_120 = s->dpt[1][38]*ROcp62_16+s->dpt[2][38]*ROcp62_419+s->dpt[3][38]*ROcp62_719;
    RLcp62_220 = s->dpt[1][38]*ROcp62_26+s->dpt[2][38]*ROcp62_519+s->dpt[3][38]*ROcp62_819;
    RLcp62_320 = s->dpt[1][38]*ROcp62_36+s->dpt[2][38]*ROcp62_619+s->dpt[3][38]*ROcp62_919;
    OMcp62_120 = OMcp62_119+ROcp62_419*qd[20];
    OMcp62_220 = OMcp62_219+ROcp62_519*qd[20];
    OMcp62_320 = OMcp62_319+ROcp62_619*qd[20];
    ORcp62_120 = OMcp62_219*RLcp62_320-OMcp62_319*RLcp62_220;
    ORcp62_220 = -(OMcp62_119*RLcp62_320-OMcp62_319*RLcp62_120);
    ORcp62_320 = OMcp62_119*RLcp62_220-OMcp62_219*RLcp62_120;
    OPcp62_120 = OPcp62_119+ROcp62_419*qdd[20]+qd[20]*(OMcp62_219*ROcp62_619-OMcp62_319*ROcp62_519);
    OPcp62_220 = OPcp62_219+ROcp62_519*qdd[20]-qd[20]*(OMcp62_119*ROcp62_619-OMcp62_319*ROcp62_419);
    OPcp62_320 = OPcp62_319+ROcp62_619*qdd[20]+qd[20]*(OMcp62_119*ROcp62_519-OMcp62_219*ROcp62_419);
    RLcp62_121 = s->dpt[1][40]*ROcp62_120+s->dpt[2][40]*ROcp62_419+ROcp62_720*s->dpt[3][40];
    RLcp62_221 = s->dpt[1][40]*ROcp62_220+s->dpt[2][40]*ROcp62_519+ROcp62_820*s->dpt[3][40];
    RLcp62_321 = s->dpt[1][40]*ROcp62_320+s->dpt[2][40]*ROcp62_619+ROcp62_920*s->dpt[3][40];
    OMcp62_121 = OMcp62_120+ROcp62_720*qd[21];
    OMcp62_221 = OMcp62_220+ROcp62_820*qd[21];
    OMcp62_321 = OMcp62_320+ROcp62_920*qd[21];
    ORcp62_121 = OMcp62_220*RLcp62_321-OMcp62_320*RLcp62_221;
    ORcp62_221 = -(OMcp62_120*RLcp62_321-OMcp62_320*RLcp62_121);
    ORcp62_321 = OMcp62_120*RLcp62_221-OMcp62_220*RLcp62_121;
    OPcp62_121 = OPcp62_120+ROcp62_720*qdd[21]+qd[21]*(OMcp62_220*ROcp62_920-OMcp62_320*ROcp62_820);
    OPcp62_221 = OPcp62_220+ROcp62_820*qdd[21]-qd[21]*(OMcp62_120*ROcp62_920-OMcp62_320*ROcp62_720);
    OPcp62_321 = OPcp62_320+ROcp62_920*qdd[21]+qd[21]*(OMcp62_120*ROcp62_820-OMcp62_220*ROcp62_720);

// = = Block_1_0_0_63_0_5 = = 
 
// Sensor Kinematics 


    ROcp62_122 = ROcp62_121*C22-ROcp62_720*S22;
    ROcp62_222 = ROcp62_221*C22-ROcp62_820*S22;
    ROcp62_322 = ROcp62_321*C22-ROcp62_920*S22;
    ROcp62_722 = ROcp62_121*S22+ROcp62_720*C22;
    ROcp62_822 = ROcp62_221*S22+ROcp62_820*C22;
    ROcp62_922 = ROcp62_321*S22+ROcp62_920*C22;
    ROcp62_423 = ROcp62_421*C23+ROcp62_722*S23;
    ROcp62_523 = ROcp62_521*C23+ROcp62_822*S23;
    ROcp62_623 = ROcp62_621*C23+ROcp62_922*S23;
    ROcp62_723 = -(ROcp62_421*S23-ROcp62_722*C23);
    ROcp62_823 = -(ROcp62_521*S23-ROcp62_822*C23);
    ROcp62_923 = -(ROcp62_621*S23-ROcp62_922*C23);
    ROcp62_124 = ROcp62_122*C24-ROcp62_723*S24;
    ROcp62_224 = ROcp62_222*C24-ROcp62_823*S24;
    ROcp62_324 = ROcp62_322*C24-ROcp62_923*S24;
    ROcp62_724 = ROcp62_122*S24+ROcp62_723*C24;
    ROcp62_824 = ROcp62_222*S24+ROcp62_823*C24;
    ROcp62_924 = ROcp62_322*S24+ROcp62_923*C24;
    ROcp62_125 = ROcp62_124*C25+ROcp62_423*S25;
    ROcp62_225 = ROcp62_224*C25+ROcp62_523*S25;
    ROcp62_325 = ROcp62_324*C25+ROcp62_623*S25;
    ROcp62_425 = -(ROcp62_124*S25-ROcp62_423*C25);
    ROcp62_525 = -(ROcp62_224*S25-ROcp62_523*C25);
    ROcp62_625 = -(ROcp62_324*S25-ROcp62_623*C25);
    ROcp62_126 = ROcp62_125*C26-ROcp62_724*S26;
    ROcp62_226 = ROcp62_225*C26-ROcp62_824*S26;
    ROcp62_326 = ROcp62_325*C26-ROcp62_924*S26;
    ROcp62_726 = ROcp62_125*S26+ROcp62_724*C26;
    ROcp62_826 = ROcp62_225*S26+ROcp62_824*C26;
    ROcp62_926 = ROcp62_325*S26+ROcp62_924*C26;
    ROcp62_127 = ROcp62_126*C27+ROcp62_425*S27;
    ROcp62_227 = ROcp62_226*C27+ROcp62_525*S27;
    ROcp62_327 = ROcp62_326*C27+ROcp62_625*S27;
    ROcp62_427 = -(ROcp62_126*S27-ROcp62_425*C27);
    ROcp62_527 = -(ROcp62_226*S27-ROcp62_525*C27);
    ROcp62_627 = -(ROcp62_326*S27-ROcp62_625*C27);
    ROcp62_428 = ROcp62_427*C28+ROcp62_726*S28;
    ROcp62_528 = ROcp62_527*C28+ROcp62_826*S28;
    ROcp62_628 = ROcp62_627*C28+ROcp62_926*S28;
    ROcp62_728 = -(ROcp62_427*S28-ROcp62_726*C28);
    ROcp62_828 = -(ROcp62_527*S28-ROcp62_826*C28);
    ROcp62_928 = -(ROcp62_627*S28-ROcp62_926*C28);
    RLcp62_122 = ROcp62_121*s->dpt[1][44]+ROcp62_421*s->dpt[2][44]+ROcp62_720*s->dpt[3][44];
    RLcp62_222 = ROcp62_221*s->dpt[1][44]+ROcp62_521*s->dpt[2][44]+ROcp62_820*s->dpt[3][44];
    RLcp62_322 = ROcp62_321*s->dpt[1][44]+ROcp62_621*s->dpt[2][44]+ROcp62_920*s->dpt[3][44];
    OMcp62_122 = OMcp62_121+ROcp62_421*qd[22];
    OMcp62_222 = OMcp62_221+ROcp62_521*qd[22];
    OMcp62_322 = OMcp62_321+ROcp62_621*qd[22];
    ORcp62_122 = OMcp62_221*RLcp62_322-OMcp62_321*RLcp62_222;
    ORcp62_222 = -(OMcp62_121*RLcp62_322-OMcp62_321*RLcp62_122);
    ORcp62_322 = OMcp62_121*RLcp62_222-OMcp62_221*RLcp62_122;
    OPcp62_122 = OPcp62_121+ROcp62_421*qdd[22]+qd[22]*(OMcp62_221*ROcp62_621-OMcp62_321*ROcp62_521);
    OPcp62_222 = OPcp62_221+ROcp62_521*qdd[22]-qd[22]*(OMcp62_121*ROcp62_621-OMcp62_321*ROcp62_421);
    OPcp62_322 = OPcp62_321+ROcp62_621*qdd[22]+qd[22]*(OMcp62_121*ROcp62_521-OMcp62_221*ROcp62_421);
    RLcp62_123 = s->dpt[1][48]*ROcp62_122+s->dpt[3][48]*ROcp62_722+ROcp62_421*s->dpt[2][48];
    RLcp62_223 = s->dpt[1][48]*ROcp62_222+s->dpt[3][48]*ROcp62_822+ROcp62_521*s->dpt[2][48];
    RLcp62_323 = s->dpt[1][48]*ROcp62_322+s->dpt[3][48]*ROcp62_922+ROcp62_621*s->dpt[2][48];
    OMcp62_123 = OMcp62_122+ROcp62_122*qd[23];
    OMcp62_223 = OMcp62_222+ROcp62_222*qd[23];
    OMcp62_323 = OMcp62_322+ROcp62_322*qd[23];
    ORcp62_123 = OMcp62_222*RLcp62_323-OMcp62_322*RLcp62_223;
    ORcp62_223 = -(OMcp62_122*RLcp62_323-OMcp62_322*RLcp62_123);
    ORcp62_323 = OMcp62_122*RLcp62_223-OMcp62_222*RLcp62_123;
    OPcp62_123 = OPcp62_122+ROcp62_122*qdd[23]+qd[23]*(OMcp62_222*ROcp62_322-OMcp62_322*ROcp62_222);
    OPcp62_223 = OPcp62_222+ROcp62_222*qdd[23]-qd[23]*(OMcp62_122*ROcp62_322-OMcp62_322*ROcp62_122);
    OPcp62_323 = OPcp62_322+ROcp62_322*qdd[23]+qd[23]*(OMcp62_122*ROcp62_222-OMcp62_222*ROcp62_122);
    RLcp62_124 = s->dpt[1][50]*ROcp62_122+s->dpt[3][50]*ROcp62_723+ROcp62_423*s->dpt[2][50];
    RLcp62_224 = s->dpt[1][50]*ROcp62_222+s->dpt[3][50]*ROcp62_823+ROcp62_523*s->dpt[2][50];
    RLcp62_324 = s->dpt[1][50]*ROcp62_322+s->dpt[3][50]*ROcp62_923+ROcp62_623*s->dpt[2][50];
    OMcp62_124 = OMcp62_123+ROcp62_423*qd[24];
    OMcp62_224 = OMcp62_223+ROcp62_523*qd[24];
    OMcp62_324 = OMcp62_323+ROcp62_623*qd[24];
    ORcp62_124 = OMcp62_223*RLcp62_324-OMcp62_323*RLcp62_224;
    ORcp62_224 = -(OMcp62_123*RLcp62_324-OMcp62_323*RLcp62_124);
    ORcp62_324 = OMcp62_123*RLcp62_224-OMcp62_223*RLcp62_124;
    OPcp62_124 = OPcp62_123+ROcp62_423*qdd[24]+qd[24]*(OMcp62_223*ROcp62_623-OMcp62_323*ROcp62_523);
    OPcp62_224 = OPcp62_223+ROcp62_523*qdd[24]-qd[24]*(OMcp62_123*ROcp62_623-OMcp62_323*ROcp62_423);
    OPcp62_324 = OPcp62_323+ROcp62_623*qdd[24]+qd[24]*(OMcp62_123*ROcp62_523-OMcp62_223*ROcp62_423);
    RLcp62_125 = s->dpt[1][52]*ROcp62_124+s->dpt[3][52]*ROcp62_724+ROcp62_423*s->dpt[2][52];
    RLcp62_225 = s->dpt[1][52]*ROcp62_224+s->dpt[3][52]*ROcp62_824+ROcp62_523*s->dpt[2][52];
    RLcp62_325 = s->dpt[1][52]*ROcp62_324+s->dpt[3][52]*ROcp62_924+ROcp62_623*s->dpt[2][52];
    OMcp62_125 = OMcp62_124+ROcp62_724*qd[25];
    OMcp62_225 = OMcp62_224+ROcp62_824*qd[25];
    OMcp62_325 = OMcp62_324+ROcp62_924*qd[25];
    ORcp62_125 = OMcp62_224*RLcp62_325-OMcp62_324*RLcp62_225;
    ORcp62_225 = -(OMcp62_124*RLcp62_325-OMcp62_324*RLcp62_125);
    ORcp62_325 = OMcp62_124*RLcp62_225-OMcp62_224*RLcp62_125;
    OPcp62_125 = OPcp62_124+ROcp62_724*qdd[25]+qd[25]*(OMcp62_224*ROcp62_924-OMcp62_324*ROcp62_824);
    OPcp62_225 = OPcp62_224+ROcp62_824*qdd[25]-qd[25]*(OMcp62_124*ROcp62_924-OMcp62_324*ROcp62_724);
    OPcp62_325 = OPcp62_324+ROcp62_924*qdd[25]+qd[25]*(OMcp62_124*ROcp62_824-OMcp62_224*ROcp62_724);
    RLcp62_126 = s->dpt[2][54]*ROcp62_425+s->dpt[3][54]*ROcp62_724+ROcp62_125*s->dpt[1][54];
    RLcp62_226 = s->dpt[2][54]*ROcp62_525+s->dpt[3][54]*ROcp62_824+ROcp62_225*s->dpt[1][54];
    RLcp62_326 = s->dpt[2][54]*ROcp62_625+s->dpt[3][54]*ROcp62_924+ROcp62_325*s->dpt[1][54];
    OMcp62_126 = OMcp62_125+ROcp62_425*qd[26];
    OMcp62_226 = OMcp62_225+ROcp62_525*qd[26];
    OMcp62_326 = OMcp62_325+ROcp62_625*qd[26];
    ORcp62_126 = OMcp62_225*RLcp62_326-OMcp62_325*RLcp62_226;
    ORcp62_226 = -(OMcp62_125*RLcp62_326-OMcp62_325*RLcp62_126);
    ORcp62_326 = OMcp62_125*RLcp62_226-OMcp62_225*RLcp62_126;
    OPcp62_126 = OPcp62_125+ROcp62_425*qdd[26]+qd[26]*(OMcp62_225*ROcp62_625-OMcp62_325*ROcp62_525);
    OPcp62_226 = OPcp62_225+ROcp62_525*qdd[26]-qd[26]*(OMcp62_125*ROcp62_625-OMcp62_325*ROcp62_425);
    OPcp62_326 = OPcp62_325+ROcp62_625*qdd[26]+qd[26]*(OMcp62_125*ROcp62_525-OMcp62_225*ROcp62_425);
    RLcp62_127 = s->dpt[1][55]*ROcp62_126+s->dpt[3][55]*ROcp62_726+ROcp62_425*s->dpt[2][55];
    RLcp62_227 = s->dpt[1][55]*ROcp62_226+s->dpt[3][55]*ROcp62_826+ROcp62_525*s->dpt[2][55];
    RLcp62_327 = s->dpt[1][55]*ROcp62_326+s->dpt[3][55]*ROcp62_926+ROcp62_625*s->dpt[2][55];
    OMcp62_127 = OMcp62_126+ROcp62_726*qd[27];
    OMcp62_227 = OMcp62_226+ROcp62_826*qd[27];
    OMcp62_327 = OMcp62_326+ROcp62_926*qd[27];
    ORcp62_127 = OMcp62_226*RLcp62_327-OMcp62_326*RLcp62_227;
    ORcp62_227 = -(OMcp62_126*RLcp62_327-OMcp62_326*RLcp62_127);
    ORcp62_327 = OMcp62_126*RLcp62_227-OMcp62_226*RLcp62_127;
    OPcp62_127 = OPcp62_126+ROcp62_726*qdd[27]+qd[27]*(OMcp62_226*ROcp62_926-OMcp62_326*ROcp62_826);
    OPcp62_227 = OPcp62_226+ROcp62_826*qdd[27]-qd[27]*(OMcp62_126*ROcp62_926-OMcp62_326*ROcp62_726);
    OPcp62_327 = OPcp62_326+ROcp62_926*qdd[27]+qd[27]*(OMcp62_126*ROcp62_826-OMcp62_226*ROcp62_726);
    RLcp62_128 = s->dpt[1][56]*ROcp62_127+s->dpt[2][56]*ROcp62_427+s->dpt[3][56]*ROcp62_726;
    RLcp62_228 = s->dpt[1][56]*ROcp62_227+s->dpt[2][56]*ROcp62_527+s->dpt[3][56]*ROcp62_826;
    RLcp62_328 = s->dpt[1][56]*ROcp62_327+s->dpt[2][56]*ROcp62_627+s->dpt[3][56]*ROcp62_926;
    OMcp62_128 = OMcp62_127+ROcp62_127*qd[28];
    OMcp62_228 = OMcp62_227+ROcp62_227*qd[28];
    OMcp62_328 = OMcp62_327+ROcp62_327*qd[28];
    ORcp62_128 = OMcp62_227*RLcp62_328-OMcp62_327*RLcp62_228;
    ORcp62_228 = -(OMcp62_127*RLcp62_328-OMcp62_327*RLcp62_128);
    ORcp62_328 = OMcp62_127*RLcp62_228-OMcp62_227*RLcp62_128;
    OPcp62_128 = OPcp62_127+ROcp62_127*qdd[28]+qd[28]*(OMcp62_227*ROcp62_327-OMcp62_327*ROcp62_227);
    OPcp62_228 = OPcp62_227+ROcp62_227*qdd[28]-qd[28]*(OMcp62_127*ROcp62_327-OMcp62_327*ROcp62_127);
    OPcp62_328 = OPcp62_327+ROcp62_327*qdd[28]+qd[28]*(OMcp62_127*ROcp62_227-OMcp62_227*ROcp62_127);
    RLcp62_198 = s->dpt[1][57]*ROcp62_127+s->dpt[3][57]*ROcp62_728+ROcp62_428*s->dpt[2][57];
    RLcp62_298 = s->dpt[1][57]*ROcp62_227+s->dpt[3][57]*ROcp62_828+ROcp62_528*s->dpt[2][57];
    RLcp62_398 = s->dpt[1][57]*ROcp62_327+s->dpt[3][57]*ROcp62_928+ROcp62_628*s->dpt[2][57];
    POcp62_198 = RLcp62_119+RLcp62_120+RLcp62_121+RLcp62_122+RLcp62_123+RLcp62_124+RLcp62_125+RLcp62_126+RLcp62_127+
 RLcp62_128+RLcp62_198+q[1];
    POcp62_298 = RLcp62_219+RLcp62_220+RLcp62_221+RLcp62_222+RLcp62_223+RLcp62_224+RLcp62_225+RLcp62_226+RLcp62_227+
 RLcp62_228+RLcp62_298+q[2];
    POcp62_398 = RLcp62_319+RLcp62_320+RLcp62_321+RLcp62_322+RLcp62_323+RLcp62_324+RLcp62_325+RLcp62_326+RLcp62_327+
 RLcp62_328+RLcp62_398+q[3];
    ORcp62_198 = OMcp62_228*RLcp62_398-OMcp62_328*RLcp62_298;
    ORcp62_298 = -(OMcp62_128*RLcp62_398-OMcp62_328*RLcp62_198);
    ORcp62_398 = OMcp62_128*RLcp62_298-OMcp62_228*RLcp62_198;
    VIcp62_198 = ORcp62_119+ORcp62_120+ORcp62_121+ORcp62_122+ORcp62_123+ORcp62_124+ORcp62_125+ORcp62_126+ORcp62_127+
 ORcp62_128+ORcp62_198+qd[1];
    VIcp62_298 = ORcp62_219+ORcp62_220+ORcp62_221+ORcp62_222+ORcp62_223+ORcp62_224+ORcp62_225+ORcp62_226+ORcp62_227+
 ORcp62_228+ORcp62_298+qd[2];
    VIcp62_398 = ORcp62_319+ORcp62_320+ORcp62_321+ORcp62_322+ORcp62_323+ORcp62_324+ORcp62_325+ORcp62_326+ORcp62_327+
 ORcp62_328+ORcp62_398+qd[3];
    ACcp62_198 = qdd[1]+OMcp62_219*ORcp62_320+OMcp62_220*ORcp62_321+OMcp62_221*ORcp62_322+OMcp62_222*ORcp62_323+OMcp62_223
 *ORcp62_324+OMcp62_224*ORcp62_325+OMcp62_225*ORcp62_326+OMcp62_226*ORcp62_327+OMcp62_227*ORcp62_328+OMcp62_228*ORcp62_398+
 OMcp62_26*ORcp62_319-OMcp62_319*ORcp62_220-OMcp62_320*ORcp62_221-OMcp62_321*ORcp62_222-OMcp62_322*ORcp62_223-OMcp62_323*
 ORcp62_224-OMcp62_324*ORcp62_225-OMcp62_325*ORcp62_226-OMcp62_326*ORcp62_227-OMcp62_327*ORcp62_228-OMcp62_328*ORcp62_298-
 OMcp62_36*ORcp62_219+OPcp62_219*RLcp62_320+OPcp62_220*RLcp62_321+OPcp62_221*RLcp62_322+OPcp62_222*RLcp62_323+OPcp62_223*
 RLcp62_324+OPcp62_224*RLcp62_325+OPcp62_225*RLcp62_326+OPcp62_226*RLcp62_327+OPcp62_227*RLcp62_328+OPcp62_228*RLcp62_398+
 OPcp62_26*RLcp62_319-OPcp62_319*RLcp62_220-OPcp62_320*RLcp62_221-OPcp62_321*RLcp62_222-OPcp62_322*RLcp62_223-OPcp62_323*
 RLcp62_224-OPcp62_324*RLcp62_225-OPcp62_325*RLcp62_226-OPcp62_326*RLcp62_227-OPcp62_327*RLcp62_228-OPcp62_328*RLcp62_298-
 OPcp62_36*RLcp62_219;
    ACcp62_298 = qdd[2]-OMcp62_119*ORcp62_320-OMcp62_120*ORcp62_321-OMcp62_121*ORcp62_322-OMcp62_122*ORcp62_323-OMcp62_123
 *ORcp62_324-OMcp62_124*ORcp62_325-OMcp62_125*ORcp62_326-OMcp62_126*ORcp62_327-OMcp62_127*ORcp62_328-OMcp62_128*ORcp62_398-
 OMcp62_16*ORcp62_319+OMcp62_319*ORcp62_120+OMcp62_320*ORcp62_121+OMcp62_321*ORcp62_122+OMcp62_322*ORcp62_123+OMcp62_323*
 ORcp62_124+OMcp62_324*ORcp62_125+OMcp62_325*ORcp62_126+OMcp62_326*ORcp62_127+OMcp62_327*ORcp62_128+OMcp62_328*ORcp62_198+
 OMcp62_36*ORcp62_119-OPcp62_119*RLcp62_320-OPcp62_120*RLcp62_321-OPcp62_121*RLcp62_322-OPcp62_122*RLcp62_323-OPcp62_123*
 RLcp62_324-OPcp62_124*RLcp62_325-OPcp62_125*RLcp62_326-OPcp62_126*RLcp62_327-OPcp62_127*RLcp62_328-OPcp62_128*RLcp62_398-
 OPcp62_16*RLcp62_319+OPcp62_319*RLcp62_120+OPcp62_320*RLcp62_121+OPcp62_321*RLcp62_122+OPcp62_322*RLcp62_123+OPcp62_323*
 RLcp62_124+OPcp62_324*RLcp62_125+OPcp62_325*RLcp62_126+OPcp62_326*RLcp62_127+OPcp62_327*RLcp62_128+OPcp62_328*RLcp62_198+
 OPcp62_36*RLcp62_119;
    ACcp62_398 = qdd[3]+OMcp62_119*ORcp62_220+OMcp62_120*ORcp62_221+OMcp62_121*ORcp62_222+OMcp62_122*ORcp62_223+OMcp62_123
 *ORcp62_224+OMcp62_124*ORcp62_225+OMcp62_125*ORcp62_226+OMcp62_126*ORcp62_227+OMcp62_127*ORcp62_228+OMcp62_128*ORcp62_298+
 OMcp62_16*ORcp62_219-OMcp62_219*ORcp62_120-OMcp62_220*ORcp62_121-OMcp62_221*ORcp62_122-OMcp62_222*ORcp62_123-OMcp62_223*
 ORcp62_124-OMcp62_224*ORcp62_125-OMcp62_225*ORcp62_126-OMcp62_226*ORcp62_127-OMcp62_227*ORcp62_128-OMcp62_228*ORcp62_198-
 OMcp62_26*ORcp62_119+OPcp62_119*RLcp62_220+OPcp62_120*RLcp62_221+OPcp62_121*RLcp62_222+OPcp62_122*RLcp62_223+OPcp62_123*
 RLcp62_224+OPcp62_124*RLcp62_225+OPcp62_125*RLcp62_226+OPcp62_126*RLcp62_227+OPcp62_127*RLcp62_228+OPcp62_128*RLcp62_298+
 OPcp62_16*RLcp62_219-OPcp62_219*RLcp62_120-OPcp62_220*RLcp62_121-OPcp62_221*RLcp62_122-OPcp62_222*RLcp62_123-OPcp62_223*
 RLcp62_124-OPcp62_224*RLcp62_125-OPcp62_225*RLcp62_126-OPcp62_226*RLcp62_127-OPcp62_227*RLcp62_128-OPcp62_228*RLcp62_198-
 OPcp62_26*RLcp62_119;

// = = Block_1_0_0_63_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp62_198;
    sens->P[2] = POcp62_298;
    sens->P[3] = POcp62_398;
    sens->R[1][1] = ROcp62_127;
    sens->R[1][2] = ROcp62_227;
    sens->R[1][3] = ROcp62_327;
    sens->R[2][1] = ROcp62_428;
    sens->R[2][2] = ROcp62_528;
    sens->R[2][3] = ROcp62_628;
    sens->R[3][1] = ROcp62_728;
    sens->R[3][2] = ROcp62_828;
    sens->R[3][3] = ROcp62_928;
    sens->V[1] = VIcp62_198;
    sens->V[2] = VIcp62_298;
    sens->V[3] = VIcp62_398;
    sens->OM[1] = OMcp62_128;
    sens->OM[2] = OMcp62_228;
    sens->OM[3] = OMcp62_328;
    sens->A[1] = ACcp62_198;
    sens->A[2] = ACcp62_298;
    sens->A[3] = ACcp62_398;
    sens->OMP[1] = OPcp62_128;
    sens->OMP[2] = OPcp62_228;
    sens->OMP[3] = OPcp62_328;
 
// 
break;
case 64:
 


// = = Block_1_0_0_64_0_1 = = 
 
// Sensor Kinematics 


    ROcp63_25 = S4*S5;
    ROcp63_35 = -C4*S5;
    ROcp63_85 = -S4*C5;
    ROcp63_95 = C4*C5;
    ROcp63_16 = C5*C6;
    ROcp63_26 = ROcp63_25*C6+C4*S6;
    ROcp63_36 = ROcp63_35*C6+S4*S6;
    ROcp63_46 = -C5*S6;
    ROcp63_56 = -(ROcp63_25*S6-C4*C6);
    ROcp63_66 = -(ROcp63_35*S6-S4*C6);
    OMcp63_25 = qd[5]*C4;
    OMcp63_35 = qd[5]*S4;
    OMcp63_16 = qd[4]+qd[6]*S5;
    OMcp63_26 = OMcp63_25+ROcp63_85*qd[6];
    OMcp63_36 = OMcp63_35+ROcp63_95*qd[6];
    OPcp63_16 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    OPcp63_26 = ROcp63_85*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp63_35*S5-ROcp63_95*qd[4]);
    OPcp63_36 = ROcp63_95*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp63_25*S5-ROcp63_85*qd[4]);

// = = Block_1_0_0_64_0_4 = = 
 
// Sensor Kinematics 


    ROcp63_419 = ROcp63_46*C19+S19*S5;
    ROcp63_519 = ROcp63_56*C19+ROcp63_85*S19;
    ROcp63_619 = ROcp63_66*C19+ROcp63_95*S19;
    ROcp63_719 = -(ROcp63_46*S19-C19*S5);
    ROcp63_819 = -(ROcp63_56*S19-ROcp63_85*C19);
    ROcp63_919 = -(ROcp63_66*S19-ROcp63_95*C19);
    ROcp63_120 = ROcp63_16*C20-ROcp63_719*S20;
    ROcp63_220 = ROcp63_26*C20-ROcp63_819*S20;
    ROcp63_320 = ROcp63_36*C20-ROcp63_919*S20;
    ROcp63_720 = ROcp63_16*S20+ROcp63_719*C20;
    ROcp63_820 = ROcp63_26*S20+ROcp63_819*C20;
    ROcp63_920 = ROcp63_36*S20+ROcp63_919*C20;
    ROcp63_121 = ROcp63_120*C21+ROcp63_419*S21;
    ROcp63_221 = ROcp63_220*C21+ROcp63_519*S21;
    ROcp63_321 = ROcp63_320*C21+ROcp63_619*S21;
    ROcp63_421 = -(ROcp63_120*S21-ROcp63_419*C21);
    ROcp63_521 = -(ROcp63_220*S21-ROcp63_519*C21);
    ROcp63_621 = -(ROcp63_320*S21-ROcp63_619*C21);
    RLcp63_119 = s->dpt[2][3]*ROcp63_46+ROcp63_16*s->dpt[1][3]+s->dpt[3][3]*S5;
    RLcp63_219 = s->dpt[2][3]*ROcp63_56+ROcp63_26*s->dpt[1][3]+ROcp63_85*s->dpt[3][3];
    RLcp63_319 = s->dpt[2][3]*ROcp63_66+ROcp63_36*s->dpt[1][3]+ROcp63_95*s->dpt[3][3];
    OMcp63_119 = OMcp63_16+ROcp63_16*qd[19];
    OMcp63_219 = OMcp63_26+ROcp63_26*qd[19];
    OMcp63_319 = OMcp63_36+ROcp63_36*qd[19];
    ORcp63_119 = OMcp63_26*RLcp63_319-OMcp63_36*RLcp63_219;
    ORcp63_219 = -(OMcp63_16*RLcp63_319-OMcp63_36*RLcp63_119);
    ORcp63_319 = OMcp63_16*RLcp63_219-OMcp63_26*RLcp63_119;
    OPcp63_119 = OPcp63_16+ROcp63_16*qdd[19]+qd[19]*(OMcp63_26*ROcp63_36-OMcp63_36*ROcp63_26);
    OPcp63_219 = OPcp63_26+ROcp63_26*qdd[19]-qd[19]*(OMcp63_16*ROcp63_36-OMcp63_36*ROcp63_16);
    OPcp63_319 = OPcp63_36+ROcp63_36*qdd[19]+qd[19]*(OMcp63_16*ROcp63_26-OMcp63_26*ROcp63_16);
    RLcp63_120 = s->dpt[1][38]*ROcp63_16+s->dpt[2][38]*ROcp63_419+s->dpt[3][38]*ROcp63_719;
    RLcp63_220 = s->dpt[1][38]*ROcp63_26+s->dpt[2][38]*ROcp63_519+s->dpt[3][38]*ROcp63_819;
    RLcp63_320 = s->dpt[1][38]*ROcp63_36+s->dpt[2][38]*ROcp63_619+s->dpt[3][38]*ROcp63_919;
    OMcp63_120 = OMcp63_119+ROcp63_419*qd[20];
    OMcp63_220 = OMcp63_219+ROcp63_519*qd[20];
    OMcp63_320 = OMcp63_319+ROcp63_619*qd[20];
    ORcp63_120 = OMcp63_219*RLcp63_320-OMcp63_319*RLcp63_220;
    ORcp63_220 = -(OMcp63_119*RLcp63_320-OMcp63_319*RLcp63_120);
    ORcp63_320 = OMcp63_119*RLcp63_220-OMcp63_219*RLcp63_120;
    OPcp63_120 = OPcp63_119+ROcp63_419*qdd[20]+qd[20]*(OMcp63_219*ROcp63_619-OMcp63_319*ROcp63_519);
    OPcp63_220 = OPcp63_219+ROcp63_519*qdd[20]-qd[20]*(OMcp63_119*ROcp63_619-OMcp63_319*ROcp63_419);
    OPcp63_320 = OPcp63_319+ROcp63_619*qdd[20]+qd[20]*(OMcp63_119*ROcp63_519-OMcp63_219*ROcp63_419);
    RLcp63_121 = s->dpt[1][40]*ROcp63_120+s->dpt[2][40]*ROcp63_419+ROcp63_720*s->dpt[3][40];
    RLcp63_221 = s->dpt[1][40]*ROcp63_220+s->dpt[2][40]*ROcp63_519+ROcp63_820*s->dpt[3][40];
    RLcp63_321 = s->dpt[1][40]*ROcp63_320+s->dpt[2][40]*ROcp63_619+ROcp63_920*s->dpt[3][40];
    OMcp63_121 = OMcp63_120+ROcp63_720*qd[21];
    OMcp63_221 = OMcp63_220+ROcp63_820*qd[21];
    OMcp63_321 = OMcp63_320+ROcp63_920*qd[21];
    ORcp63_121 = OMcp63_220*RLcp63_321-OMcp63_320*RLcp63_221;
    ORcp63_221 = -(OMcp63_120*RLcp63_321-OMcp63_320*RLcp63_121);
    ORcp63_321 = OMcp63_120*RLcp63_221-OMcp63_220*RLcp63_121;
    OPcp63_121 = OPcp63_120+ROcp63_720*qdd[21]+qd[21]*(OMcp63_220*ROcp63_920-OMcp63_320*ROcp63_820);
    OPcp63_221 = OPcp63_220+ROcp63_820*qdd[21]-qd[21]*(OMcp63_120*ROcp63_920-OMcp63_320*ROcp63_720);
    OPcp63_321 = OPcp63_320+ROcp63_920*qdd[21]+qd[21]*(OMcp63_120*ROcp63_820-OMcp63_220*ROcp63_720);

// = = Block_1_0_0_64_0_6 = = 
 
// Sensor Kinematics 


    ROcp63_129 = ROcp63_121*C29-ROcp63_720*S29;
    ROcp63_229 = ROcp63_221*C29-ROcp63_820*S29;
    ROcp63_329 = ROcp63_321*C29-ROcp63_920*S29;
    ROcp63_729 = ROcp63_121*S29+ROcp63_720*C29;
    ROcp63_829 = ROcp63_221*S29+ROcp63_820*C29;
    ROcp63_929 = ROcp63_321*S29+ROcp63_920*C29;
    ROcp63_430 = ROcp63_421*C30+ROcp63_729*S30;
    ROcp63_530 = ROcp63_521*C30+ROcp63_829*S30;
    ROcp63_630 = ROcp63_621*C30+ROcp63_929*S30;
    ROcp63_730 = -(ROcp63_421*S30-ROcp63_729*C30);
    ROcp63_830 = -(ROcp63_521*S30-ROcp63_829*C30);
    ROcp63_930 = -(ROcp63_621*S30-ROcp63_929*C30);
    ROcp63_131 = ROcp63_129*C31-ROcp63_730*S31;
    ROcp63_231 = ROcp63_229*C31-ROcp63_830*S31;
    ROcp63_331 = ROcp63_329*C31-ROcp63_930*S31;
    ROcp63_731 = ROcp63_129*S31+ROcp63_730*C31;
    ROcp63_831 = ROcp63_229*S31+ROcp63_830*C31;
    ROcp63_931 = ROcp63_329*S31+ROcp63_930*C31;
    ROcp63_132 = ROcp63_131*C32+ROcp63_430*S32;
    ROcp63_232 = ROcp63_231*C32+ROcp63_530*S32;
    ROcp63_332 = ROcp63_331*C32+ROcp63_630*S32;
    ROcp63_432 = -(ROcp63_131*S32-ROcp63_430*C32);
    ROcp63_532 = -(ROcp63_231*S32-ROcp63_530*C32);
    ROcp63_632 = -(ROcp63_331*S32-ROcp63_630*C32);
    ROcp63_133 = ROcp63_132*C33-ROcp63_731*S33;
    ROcp63_233 = ROcp63_232*C33-ROcp63_831*S33;
    ROcp63_333 = ROcp63_332*C33-ROcp63_931*S33;
    ROcp63_733 = ROcp63_132*S33+ROcp63_731*C33;
    ROcp63_833 = ROcp63_232*S33+ROcp63_831*C33;
    ROcp63_933 = ROcp63_332*S33+ROcp63_931*C33;
    ROcp63_134 = ROcp63_133*C34+ROcp63_432*S34;
    ROcp63_234 = ROcp63_233*C34+ROcp63_532*S34;
    ROcp63_334 = ROcp63_333*C34+ROcp63_632*S34;
    ROcp63_434 = -(ROcp63_133*S34-ROcp63_432*C34);
    ROcp63_534 = -(ROcp63_233*S34-ROcp63_532*C34);
    ROcp63_634 = -(ROcp63_333*S34-ROcp63_632*C34);
    ROcp63_435 = ROcp63_434*C35+ROcp63_733*S35;
    ROcp63_535 = ROcp63_534*C35+ROcp63_833*S35;
    ROcp63_635 = ROcp63_634*C35+ROcp63_933*S35;
    ROcp63_735 = -(ROcp63_434*S35-ROcp63_733*C35);
    ROcp63_835 = -(ROcp63_534*S35-ROcp63_833*C35);
    ROcp63_935 = -(ROcp63_634*S35-ROcp63_933*C35);
    RLcp63_129 = ROcp63_121*s->dpt[1][45]+ROcp63_421*s->dpt[2][45]+ROcp63_720*s->dpt[3][45];
    RLcp63_229 = ROcp63_221*s->dpt[1][45]+ROcp63_521*s->dpt[2][45]+ROcp63_820*s->dpt[3][45];
    RLcp63_329 = ROcp63_321*s->dpt[1][45]+ROcp63_621*s->dpt[2][45]+ROcp63_920*s->dpt[3][45];
    OMcp63_129 = OMcp63_121+ROcp63_421*qd[29];
    OMcp63_229 = OMcp63_221+ROcp63_521*qd[29];
    OMcp63_329 = OMcp63_321+ROcp63_621*qd[29];
    ORcp63_129 = OMcp63_221*RLcp63_329-OMcp63_321*RLcp63_229;
    ORcp63_229 = -(OMcp63_121*RLcp63_329-OMcp63_321*RLcp63_129);
    ORcp63_329 = OMcp63_121*RLcp63_229-OMcp63_221*RLcp63_129;
    OPcp63_129 = OPcp63_121+ROcp63_421*qdd[29]+qd[29]*(OMcp63_221*ROcp63_621-OMcp63_321*ROcp63_521);
    OPcp63_229 = OPcp63_221+ROcp63_521*qdd[29]-qd[29]*(OMcp63_121*ROcp63_621-OMcp63_321*ROcp63_421);
    OPcp63_329 = OPcp63_321+ROcp63_621*qdd[29]+qd[29]*(OMcp63_121*ROcp63_521-OMcp63_221*ROcp63_421);
    RLcp63_130 = s->dpt[1][59]*ROcp63_129+s->dpt[3][59]*ROcp63_729+ROcp63_421*s->dpt[2][59];
    RLcp63_230 = s->dpt[1][59]*ROcp63_229+s->dpt[3][59]*ROcp63_829+ROcp63_521*s->dpt[2][59];
    RLcp63_330 = s->dpt[1][59]*ROcp63_329+s->dpt[3][59]*ROcp63_929+ROcp63_621*s->dpt[2][59];
    OMcp63_130 = OMcp63_129+ROcp63_129*qd[30];
    OMcp63_230 = OMcp63_229+ROcp63_229*qd[30];
    OMcp63_330 = OMcp63_329+ROcp63_329*qd[30];
    ORcp63_130 = OMcp63_229*RLcp63_330-OMcp63_329*RLcp63_230;
    ORcp63_230 = -(OMcp63_129*RLcp63_330-OMcp63_329*RLcp63_130);
    ORcp63_330 = OMcp63_129*RLcp63_230-OMcp63_229*RLcp63_130;
    OPcp63_130 = OPcp63_129+ROcp63_129*qdd[30]+qd[30]*(OMcp63_229*ROcp63_329-OMcp63_329*ROcp63_229);
    OPcp63_230 = OPcp63_229+ROcp63_229*qdd[30]-qd[30]*(OMcp63_129*ROcp63_329-OMcp63_329*ROcp63_129);
    OPcp63_330 = OPcp63_329+ROcp63_329*qdd[30]+qd[30]*(OMcp63_129*ROcp63_229-OMcp63_229*ROcp63_129);
    RLcp63_131 = s->dpt[1][61]*ROcp63_129+s->dpt[3][61]*ROcp63_730+ROcp63_430*s->dpt[2][61];
    RLcp63_231 = s->dpt[1][61]*ROcp63_229+s->dpt[3][61]*ROcp63_830+ROcp63_530*s->dpt[2][61];
    RLcp63_331 = s->dpt[1][61]*ROcp63_329+s->dpt[3][61]*ROcp63_930+ROcp63_630*s->dpt[2][61];
    OMcp63_131 = OMcp63_130+ROcp63_430*qd[31];
    OMcp63_231 = OMcp63_230+ROcp63_530*qd[31];
    OMcp63_331 = OMcp63_330+ROcp63_630*qd[31];
    ORcp63_131 = OMcp63_230*RLcp63_331-OMcp63_330*RLcp63_231;
    ORcp63_231 = -(OMcp63_130*RLcp63_331-OMcp63_330*RLcp63_131);
    ORcp63_331 = OMcp63_130*RLcp63_231-OMcp63_230*RLcp63_131;
    OPcp63_131 = OPcp63_130+ROcp63_430*qdd[31]+qd[31]*(OMcp63_230*ROcp63_630-OMcp63_330*ROcp63_530);
    OPcp63_231 = OPcp63_230+ROcp63_530*qdd[31]-qd[31]*(OMcp63_130*ROcp63_630-OMcp63_330*ROcp63_430);
    OPcp63_331 = OPcp63_330+ROcp63_630*qdd[31]+qd[31]*(OMcp63_130*ROcp63_530-OMcp63_230*ROcp63_430);
    RLcp63_132 = s->dpt[3][63]*ROcp63_731+ROcp63_131*s->dpt[1][63]+ROcp63_430*s->dpt[2][63];
    RLcp63_232 = s->dpt[3][63]*ROcp63_831+ROcp63_231*s->dpt[1][63]+ROcp63_530*s->dpt[2][63];
    RLcp63_332 = s->dpt[3][63]*ROcp63_931+ROcp63_331*s->dpt[1][63]+ROcp63_630*s->dpt[2][63];
    OMcp63_132 = OMcp63_131+ROcp63_731*qd[32];
    OMcp63_232 = OMcp63_231+ROcp63_831*qd[32];
    OMcp63_332 = OMcp63_331+ROcp63_931*qd[32];
    ORcp63_132 = OMcp63_231*RLcp63_332-OMcp63_331*RLcp63_232;
    ORcp63_232 = -(OMcp63_131*RLcp63_332-OMcp63_331*RLcp63_132);
    ORcp63_332 = OMcp63_131*RLcp63_232-OMcp63_231*RLcp63_132;
    OPcp63_132 = OPcp63_131+ROcp63_731*qdd[32]+qd[32]*(OMcp63_231*ROcp63_931-OMcp63_331*ROcp63_831);
    OPcp63_232 = OPcp63_231+ROcp63_831*qdd[32]-qd[32]*(OMcp63_131*ROcp63_931-OMcp63_331*ROcp63_731);
    OPcp63_332 = OPcp63_331+ROcp63_931*qdd[32]+qd[32]*(OMcp63_131*ROcp63_831-OMcp63_231*ROcp63_731);
    RLcp63_133 = s->dpt[2][65]*ROcp63_432+s->dpt[3][65]*ROcp63_731+ROcp63_132*s->dpt[1][65];
    RLcp63_233 = s->dpt[2][65]*ROcp63_532+s->dpt[3][65]*ROcp63_831+ROcp63_232*s->dpt[1][65];
    RLcp63_333 = s->dpt[2][65]*ROcp63_632+s->dpt[3][65]*ROcp63_931+ROcp63_332*s->dpt[1][65];
    OMcp63_133 = OMcp63_132+ROcp63_432*qd[33];
    OMcp63_233 = OMcp63_232+ROcp63_532*qd[33];
    OMcp63_333 = OMcp63_332+ROcp63_632*qd[33];
    ORcp63_133 = OMcp63_232*RLcp63_333-OMcp63_332*RLcp63_233;
    ORcp63_233 = -(OMcp63_132*RLcp63_333-OMcp63_332*RLcp63_133);
    ORcp63_333 = OMcp63_132*RLcp63_233-OMcp63_232*RLcp63_133;
    OPcp63_133 = OPcp63_132+ROcp63_432*qdd[33]+qd[33]*(OMcp63_232*ROcp63_632-OMcp63_332*ROcp63_532);
    OPcp63_233 = OPcp63_232+ROcp63_532*qdd[33]-qd[33]*(OMcp63_132*ROcp63_632-OMcp63_332*ROcp63_432);
    OPcp63_333 = OPcp63_332+ROcp63_632*qdd[33]+qd[33]*(OMcp63_132*ROcp63_532-OMcp63_232*ROcp63_432);
    RLcp63_134 = s->dpt[1][66]*ROcp63_133+s->dpt[3][66]*ROcp63_733+ROcp63_432*s->dpt[2][66];
    RLcp63_234 = s->dpt[1][66]*ROcp63_233+s->dpt[3][66]*ROcp63_833+ROcp63_532*s->dpt[2][66];
    RLcp63_334 = s->dpt[1][66]*ROcp63_333+s->dpt[3][66]*ROcp63_933+ROcp63_632*s->dpt[2][66];
    OMcp63_134 = OMcp63_133+ROcp63_733*qd[34];
    OMcp63_234 = OMcp63_233+ROcp63_833*qd[34];
    OMcp63_334 = OMcp63_333+ROcp63_933*qd[34];
    ORcp63_134 = OMcp63_233*RLcp63_334-OMcp63_333*RLcp63_234;
    ORcp63_234 = -(OMcp63_133*RLcp63_334-OMcp63_333*RLcp63_134);
    ORcp63_334 = OMcp63_133*RLcp63_234-OMcp63_233*RLcp63_134;
    OPcp63_134 = OPcp63_133+ROcp63_733*qdd[34]+qd[34]*(OMcp63_233*ROcp63_933-OMcp63_333*ROcp63_833);
    OPcp63_234 = OPcp63_233+ROcp63_833*qdd[34]-qd[34]*(OMcp63_133*ROcp63_933-OMcp63_333*ROcp63_733);
    OPcp63_334 = OPcp63_333+ROcp63_933*qdd[34]+qd[34]*(OMcp63_133*ROcp63_833-OMcp63_233*ROcp63_733);
    RLcp63_135 = s->dpt[1][67]*ROcp63_134+s->dpt[2][67]*ROcp63_434+s->dpt[3][67]*ROcp63_733;
    RLcp63_235 = s->dpt[1][67]*ROcp63_234+s->dpt[2][67]*ROcp63_534+s->dpt[3][67]*ROcp63_833;
    RLcp63_335 = s->dpt[1][67]*ROcp63_334+s->dpt[2][67]*ROcp63_634+s->dpt[3][67]*ROcp63_933;
    OMcp63_135 = OMcp63_134+ROcp63_134*qd[35];
    OMcp63_235 = OMcp63_234+ROcp63_234*qd[35];
    OMcp63_335 = OMcp63_334+ROcp63_334*qd[35];
    ORcp63_135 = OMcp63_234*RLcp63_335-OMcp63_334*RLcp63_235;
    ORcp63_235 = -(OMcp63_134*RLcp63_335-OMcp63_334*RLcp63_135);
    ORcp63_335 = OMcp63_134*RLcp63_235-OMcp63_234*RLcp63_135;
    OPcp63_135 = OPcp63_134+ROcp63_134*qdd[35]+qd[35]*(OMcp63_234*ROcp63_334-OMcp63_334*ROcp63_234);
    OPcp63_235 = OPcp63_234+ROcp63_234*qdd[35]-qd[35]*(OMcp63_134*ROcp63_334-OMcp63_334*ROcp63_134);
    OPcp63_335 = OPcp63_334+ROcp63_334*qdd[35]+qd[35]*(OMcp63_134*ROcp63_234-OMcp63_234*ROcp63_134);
    RLcp63_199 = s->dpt[1][68]*ROcp63_134+s->dpt[3][68]*ROcp63_735+ROcp63_435*s->dpt[2][68];
    RLcp63_299 = s->dpt[1][68]*ROcp63_234+s->dpt[3][68]*ROcp63_835+ROcp63_535*s->dpt[2][68];
    RLcp63_399 = s->dpt[1][68]*ROcp63_334+s->dpt[3][68]*ROcp63_935+ROcp63_635*s->dpt[2][68];
    POcp63_199 = RLcp63_119+RLcp63_120+RLcp63_121+RLcp63_129+RLcp63_130+RLcp63_131+RLcp63_132+RLcp63_133+RLcp63_134+
 RLcp63_135+RLcp63_199+q[1];
    POcp63_299 = RLcp63_219+RLcp63_220+RLcp63_221+RLcp63_229+RLcp63_230+RLcp63_231+RLcp63_232+RLcp63_233+RLcp63_234+
 RLcp63_235+RLcp63_299+q[2];
    POcp63_399 = RLcp63_319+RLcp63_320+RLcp63_321+RLcp63_329+RLcp63_330+RLcp63_331+RLcp63_332+RLcp63_333+RLcp63_334+
 RLcp63_335+RLcp63_399+q[3];
    ORcp63_199 = OMcp63_235*RLcp63_399-OMcp63_335*RLcp63_299;
    ORcp63_299 = -(OMcp63_135*RLcp63_399-OMcp63_335*RLcp63_199);
    ORcp63_399 = OMcp63_135*RLcp63_299-OMcp63_235*RLcp63_199;
    VIcp63_199 = ORcp63_119+ORcp63_120+ORcp63_121+ORcp63_129+ORcp63_130+ORcp63_131+ORcp63_132+ORcp63_133+ORcp63_134+
 ORcp63_135+ORcp63_199+qd[1];
    VIcp63_299 = ORcp63_219+ORcp63_220+ORcp63_221+ORcp63_229+ORcp63_230+ORcp63_231+ORcp63_232+ORcp63_233+ORcp63_234+
 ORcp63_235+ORcp63_299+qd[2];
    VIcp63_399 = ORcp63_319+ORcp63_320+ORcp63_321+ORcp63_329+ORcp63_330+ORcp63_331+ORcp63_332+ORcp63_333+ORcp63_334+
 ORcp63_335+ORcp63_399+qd[3];
    ACcp63_199 = qdd[1]+OMcp63_219*ORcp63_320+OMcp63_220*ORcp63_321+OMcp63_221*ORcp63_329+OMcp63_229*ORcp63_330+OMcp63_230
 *ORcp63_331+OMcp63_231*ORcp63_332+OMcp63_232*ORcp63_333+OMcp63_233*ORcp63_334+OMcp63_234*ORcp63_335+OMcp63_235*ORcp63_399+
 OMcp63_26*ORcp63_319-OMcp63_319*ORcp63_220-OMcp63_320*ORcp63_221-OMcp63_321*ORcp63_229-OMcp63_329*ORcp63_230-OMcp63_330*
 ORcp63_231-OMcp63_331*ORcp63_232-OMcp63_332*ORcp63_233-OMcp63_333*ORcp63_234-OMcp63_334*ORcp63_235-OMcp63_335*ORcp63_299-
 OMcp63_36*ORcp63_219+OPcp63_219*RLcp63_320+OPcp63_220*RLcp63_321+OPcp63_221*RLcp63_329+OPcp63_229*RLcp63_330+OPcp63_230*
 RLcp63_331+OPcp63_231*RLcp63_332+OPcp63_232*RLcp63_333+OPcp63_233*RLcp63_334+OPcp63_234*RLcp63_335+OPcp63_235*RLcp63_399+
 OPcp63_26*RLcp63_319-OPcp63_319*RLcp63_220-OPcp63_320*RLcp63_221-OPcp63_321*RLcp63_229-OPcp63_329*RLcp63_230-OPcp63_330*
 RLcp63_231-OPcp63_331*RLcp63_232-OPcp63_332*RLcp63_233-OPcp63_333*RLcp63_234-OPcp63_334*RLcp63_235-OPcp63_335*RLcp63_299-
 OPcp63_36*RLcp63_219;
    ACcp63_299 = qdd[2]-OMcp63_119*ORcp63_320-OMcp63_120*ORcp63_321-OMcp63_121*ORcp63_329-OMcp63_129*ORcp63_330-OMcp63_130
 *ORcp63_331-OMcp63_131*ORcp63_332-OMcp63_132*ORcp63_333-OMcp63_133*ORcp63_334-OMcp63_134*ORcp63_335-OMcp63_135*ORcp63_399-
 OMcp63_16*ORcp63_319+OMcp63_319*ORcp63_120+OMcp63_320*ORcp63_121+OMcp63_321*ORcp63_129+OMcp63_329*ORcp63_130+OMcp63_330*
 ORcp63_131+OMcp63_331*ORcp63_132+OMcp63_332*ORcp63_133+OMcp63_333*ORcp63_134+OMcp63_334*ORcp63_135+OMcp63_335*ORcp63_199+
 OMcp63_36*ORcp63_119-OPcp63_119*RLcp63_320-OPcp63_120*RLcp63_321-OPcp63_121*RLcp63_329-OPcp63_129*RLcp63_330-OPcp63_130*
 RLcp63_331-OPcp63_131*RLcp63_332-OPcp63_132*RLcp63_333-OPcp63_133*RLcp63_334-OPcp63_134*RLcp63_335-OPcp63_135*RLcp63_399-
 OPcp63_16*RLcp63_319+OPcp63_319*RLcp63_120+OPcp63_320*RLcp63_121+OPcp63_321*RLcp63_129+OPcp63_329*RLcp63_130+OPcp63_330*
 RLcp63_131+OPcp63_331*RLcp63_132+OPcp63_332*RLcp63_133+OPcp63_333*RLcp63_134+OPcp63_334*RLcp63_135+OPcp63_335*RLcp63_199+
 OPcp63_36*RLcp63_119;
    ACcp63_399 = qdd[3]+OMcp63_119*ORcp63_220+OMcp63_120*ORcp63_221+OMcp63_121*ORcp63_229+OMcp63_129*ORcp63_230+OMcp63_130
 *ORcp63_231+OMcp63_131*ORcp63_232+OMcp63_132*ORcp63_233+OMcp63_133*ORcp63_234+OMcp63_134*ORcp63_235+OMcp63_135*ORcp63_299+
 OMcp63_16*ORcp63_219-OMcp63_219*ORcp63_120-OMcp63_220*ORcp63_121-OMcp63_221*ORcp63_129-OMcp63_229*ORcp63_130-OMcp63_230*
 ORcp63_131-OMcp63_231*ORcp63_132-OMcp63_232*ORcp63_133-OMcp63_233*ORcp63_134-OMcp63_234*ORcp63_135-OMcp63_235*ORcp63_199-
 OMcp63_26*ORcp63_119+OPcp63_119*RLcp63_220+OPcp63_120*RLcp63_221+OPcp63_121*RLcp63_229+OPcp63_129*RLcp63_230+OPcp63_130*
 RLcp63_231+OPcp63_131*RLcp63_232+OPcp63_132*RLcp63_233+OPcp63_133*RLcp63_234+OPcp63_134*RLcp63_235+OPcp63_135*RLcp63_299+
 OPcp63_16*RLcp63_219-OPcp63_219*RLcp63_120-OPcp63_220*RLcp63_121-OPcp63_221*RLcp63_129-OPcp63_229*RLcp63_130-OPcp63_230*
 RLcp63_131-OPcp63_231*RLcp63_132-OPcp63_232*RLcp63_133-OPcp63_233*RLcp63_134-OPcp63_234*RLcp63_135-OPcp63_235*RLcp63_199-
 OPcp63_26*RLcp63_119;

// = = Block_1_0_0_64_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp63_199;
    sens->P[2] = POcp63_299;
    sens->P[3] = POcp63_399;
    sens->R[1][1] = ROcp63_134;
    sens->R[1][2] = ROcp63_234;
    sens->R[1][3] = ROcp63_334;
    sens->R[2][1] = ROcp63_435;
    sens->R[2][2] = ROcp63_535;
    sens->R[2][3] = ROcp63_635;
    sens->R[3][1] = ROcp63_735;
    sens->R[3][2] = ROcp63_835;
    sens->R[3][3] = ROcp63_935;
    sens->V[1] = VIcp63_199;
    sens->V[2] = VIcp63_299;
    sens->V[3] = VIcp63_399;
    sens->OM[1] = OMcp63_135;
    sens->OM[2] = OMcp63_235;
    sens->OM[3] = OMcp63_335;
    sens->A[1] = ACcp63_199;
    sens->A[2] = ACcp63_299;
    sens->A[3] = ACcp63_399;
    sens->OMP[1] = OPcp63_135;
    sens->OMP[2] = OPcp63_235;
    sens->OMP[3] = OPcp63_335;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

