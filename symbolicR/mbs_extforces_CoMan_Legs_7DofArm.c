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
//	==> Generation Date : Fri Jul 11 12:29:59 2014
//
//	==> Project name : CoMan_Legs_7DofArm
//	==> using XML input file 
//
//	==> Number of joints : 35
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 10381
//
//	==> All Parameter Symbols included
//	==> Generation Time :  0.230 seconds
//	==> Post-Processing :  0.190 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void extforces(double **frc,double **trq,
MBSdataStruct *s, double tsim)

// double frc[3][35];
// double trq[3][35];
{ 
double PxF1[4]; 
double RxF1[4][4]; 
double VxF1[4]; 
double OMxF1[4]; 
double AxF1[4]; 
double OMPxF1[4]; 
double *SWr1; 
double PxF2[4]; 
double RxF2[4][4]; 
double VxF2[4]; 
double OMxF2[4]; 
double AxF2[4]; 
double OMPxF2[4]; 
double *SWr2; 
double PxF3[4]; 
double RxF3[4][4]; 
double VxF3[4]; 
double OMxF3[4]; 
double AxF3[4]; 
double OMPxF3[4]; 
double *SWr3; 
double PxF4[4]; 
double RxF4[4][4]; 
double VxF4[4]; 
double OMxF4[4]; 
double AxF4[4]; 
double OMPxF4[4]; 
double *SWr4; 
double PxF5[4]; 
double RxF5[4][4]; 
double VxF5[4]; 
double OMxF5[4]; 
double AxF5[4]; 
double OMPxF5[4]; 
double *SWr5; 
double PxF6[4]; 
double RxF6[4][4]; 
double VxF6[4]; 
double OMxF6[4]; 
double AxF6[4]; 
double OMPxF6[4]; 
double *SWr6; 
double PxF7[4]; 
double RxF7[4][4]; 
double VxF7[4]; 
double OMxF7[4]; 
double AxF7[4]; 
double OMPxF7[4]; 
double *SWr7; 
double PxF8[4]; 
double RxF8[4][4]; 
double VxF8[4]; 
double OMxF8[4]; 
double AxF8[4]; 
double OMPxF8[4]; 
double *SWr8; 
double PxF9[4]; 
double RxF9[4][4]; 
double VxF9[4]; 
double OMxF9[4]; 
double AxF9[4]; 
double OMPxF9[4]; 
double *SWr9; 
double PxF10[4]; 
double RxF10[4][4]; 
double VxF10[4]; 
double OMxF10[4]; 
double AxF10[4]; 
double OMPxF10[4]; 
double *SWr10; 
double PxF11[4]; 
double RxF11[4][4]; 
double VxF11[4]; 
double OMxF11[4]; 
double AxF11[4]; 
double OMPxF11[4]; 
double *SWr11; 
double PxF12[4]; 
double RxF12[4][4]; 
double VxF12[4]; 
double OMxF12[4]; 
double AxF12[4]; 
double OMPxF12[4]; 
double *SWr12; 
double PxF13[4]; 
double RxF13[4][4]; 
double VxF13[4]; 
double OMxF13[4]; 
double AxF13[4]; 
double OMPxF13[4]; 
double *SWr13; 
 
#include "mbs_extforces_CoMan_Legs_7DofArm.h" 
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

// = = Block_0_0_1_1_0_1 = = 
 
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
  OMcp51_26 = OMcp51_25+qd[6]*ROcp51_85;
  OMcp51_36 = OMcp51_35+qd[6]*ROcp51_95;
  OPcp51_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp51_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp51_95-OMcp51_35*S5)-qdd[5]*C4-qdd[6]*ROcp51_85);
  OPcp51_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp51_85-OMcp51_25*S5)+qdd[5]*S4+qdd[6]*ROcp51_95;

// = = Block_0_0_1_1_0_2 = = 
 
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
  OMcp51_17 = OMcp51_16+qd[7]*ROcp51_46;
  OMcp51_27 = OMcp51_26+qd[7]*ROcp51_56;
  OMcp51_37 = OMcp51_36+qd[7]*ROcp51_66;
  ORcp51_17 = OMcp51_26*RLcp51_37-OMcp51_36*RLcp51_27;
  ORcp51_27 = -(OMcp51_16*RLcp51_37-OMcp51_36*RLcp51_17);
  ORcp51_37 = OMcp51_16*RLcp51_27-OMcp51_26*RLcp51_17;
  OPcp51_17 = OPcp51_16+qd[7]*(OMcp51_26*ROcp51_66-OMcp51_36*ROcp51_56)+qdd[7]*ROcp51_46;
  OPcp51_27 = OPcp51_26-qd[7]*(OMcp51_16*ROcp51_66-OMcp51_36*ROcp51_46)+qdd[7]*ROcp51_56;
  OPcp51_37 = OPcp51_36+qd[7]*(OMcp51_16*ROcp51_56-OMcp51_26*ROcp51_46)+qdd[7]*ROcp51_66;
  RLcp51_18 = s->dpt[1][6]*ROcp51_17+s->dpt[3][6]*ROcp51_77+ROcp51_46*s->dpt[2][6];
  RLcp51_28 = s->dpt[1][6]*ROcp51_27+s->dpt[3][6]*ROcp51_87+ROcp51_56*s->dpt[2][6];
  RLcp51_38 = s->dpt[1][6]*ROcp51_37+s->dpt[3][6]*ROcp51_97+ROcp51_66*s->dpt[2][6];
  OMcp51_18 = OMcp51_17+qd[8]*ROcp51_17;
  OMcp51_28 = OMcp51_27+qd[8]*ROcp51_27;
  OMcp51_38 = OMcp51_37+qd[8]*ROcp51_37;
  ORcp51_18 = OMcp51_27*RLcp51_38-OMcp51_37*RLcp51_28;
  ORcp51_28 = -(OMcp51_17*RLcp51_38-OMcp51_37*RLcp51_18);
  ORcp51_38 = OMcp51_17*RLcp51_28-OMcp51_27*RLcp51_18;
  OPcp51_18 = OPcp51_17+qd[8]*(OMcp51_27*ROcp51_37-OMcp51_37*ROcp51_27)+qdd[8]*ROcp51_17;
  OPcp51_28 = OPcp51_27-qd[8]*(OMcp51_17*ROcp51_37-OMcp51_37*ROcp51_17)+qdd[8]*ROcp51_27;
  OPcp51_38 = OPcp51_37+qd[8]*(OMcp51_17*ROcp51_27-OMcp51_27*ROcp51_17)+qdd[8]*ROcp51_37;
  RLcp51_19 = s->dpt[1][8]*ROcp51_17+s->dpt[2][8]*ROcp51_48+ROcp51_78*s->dpt[3][8];
  RLcp51_29 = s->dpt[1][8]*ROcp51_27+s->dpt[2][8]*ROcp51_58+ROcp51_88*s->dpt[3][8];
  RLcp51_39 = s->dpt[1][8]*ROcp51_37+s->dpt[2][8]*ROcp51_68+ROcp51_98*s->dpt[3][8];
  OMcp51_19 = OMcp51_18+qd[9]*ROcp51_78;
  OMcp51_29 = OMcp51_28+qd[9]*ROcp51_88;
  OMcp51_39 = OMcp51_38+qd[9]*ROcp51_98;
  ORcp51_19 = OMcp51_28*RLcp51_39-OMcp51_38*RLcp51_29;
  ORcp51_29 = -(OMcp51_18*RLcp51_39-OMcp51_38*RLcp51_19);
  ORcp51_39 = OMcp51_18*RLcp51_29-OMcp51_28*RLcp51_19;
  OPcp51_19 = OPcp51_18+qd[9]*(OMcp51_28*ROcp51_98-OMcp51_38*ROcp51_88)+qdd[9]*ROcp51_78;
  OPcp51_29 = OPcp51_28-qd[9]*(OMcp51_18*ROcp51_98-OMcp51_38*ROcp51_78)+qdd[9]*ROcp51_88;
  OPcp51_39 = OPcp51_38+qd[9]*(OMcp51_18*ROcp51_88-OMcp51_28*ROcp51_78)+qdd[9]*ROcp51_98;
  RLcp51_110 = s->dpt[1][10]*ROcp51_19+s->dpt[2][10]*ROcp51_49+ROcp51_78*s->dpt[3][10];
  RLcp51_210 = s->dpt[1][10]*ROcp51_29+s->dpt[2][10]*ROcp51_59+ROcp51_88*s->dpt[3][10];
  RLcp51_310 = s->dpt[1][10]*ROcp51_39+s->dpt[2][10]*ROcp51_69+ROcp51_98*s->dpt[3][10];
  OMcp51_110 = OMcp51_19+qd[10]*ROcp51_49;
  OMcp51_210 = OMcp51_29+qd[10]*ROcp51_59;
  OMcp51_310 = OMcp51_39+qd[10]*ROcp51_69;
  ORcp51_110 = OMcp51_29*RLcp51_310-OMcp51_39*RLcp51_210;
  ORcp51_210 = -(OMcp51_19*RLcp51_310-OMcp51_39*RLcp51_110);
  ORcp51_310 = OMcp51_19*RLcp51_210-OMcp51_29*RLcp51_110;
  OPcp51_110 = OPcp51_19+qd[10]*(OMcp51_29*ROcp51_69-OMcp51_39*ROcp51_59)+qdd[10]*ROcp51_49;
  OPcp51_210 = OPcp51_29-qd[10]*(OMcp51_19*ROcp51_69-OMcp51_39*ROcp51_49)+qdd[10]*ROcp51_59;
  OPcp51_310 = OPcp51_39+qd[10]*(OMcp51_19*ROcp51_59-OMcp51_29*ROcp51_49)+qdd[10]*ROcp51_69;
  RLcp51_111 = s->dpt[1][12]*ROcp51_110+s->dpt[2][12]*ROcp51_49+ROcp51_710*s->dpt[3][12];
  RLcp51_211 = s->dpt[1][12]*ROcp51_210+s->dpt[2][12]*ROcp51_59+ROcp51_810*s->dpt[3][12];
  RLcp51_311 = s->dpt[1][12]*ROcp51_310+s->dpt[2][12]*ROcp51_69+ROcp51_910*s->dpt[3][12];
  OMcp51_111 = OMcp51_110+qd[11]*ROcp51_110;
  OMcp51_211 = OMcp51_210+qd[11]*ROcp51_210;
  OMcp51_311 = OMcp51_310+qd[11]*ROcp51_310;
  ORcp51_111 = OMcp51_210*RLcp51_311-OMcp51_310*RLcp51_211;
  ORcp51_211 = -(OMcp51_110*RLcp51_311-OMcp51_310*RLcp51_111);
  ORcp51_311 = OMcp51_110*RLcp51_211-OMcp51_210*RLcp51_111;
  OPcp51_111 = OPcp51_110+qd[11]*(OMcp51_210*ROcp51_310-OMcp51_310*ROcp51_210)+qdd[11]*ROcp51_110;
  OPcp51_211 = OPcp51_210-qd[11]*(OMcp51_110*ROcp51_310-OMcp51_310*ROcp51_110)+qdd[11]*ROcp51_210;
  OPcp51_311 = OPcp51_310+qd[11]*(OMcp51_110*ROcp51_210-OMcp51_210*ROcp51_110)+qdd[11]*ROcp51_310;
  RLcp51_112 = s->dpt[1][14]*ROcp51_110+s->dpt[2][14]*ROcp51_411+s->dpt[3][14]*ROcp51_711;
  RLcp51_212 = s->dpt[1][14]*ROcp51_210+s->dpt[2][14]*ROcp51_511+s->dpt[3][14]*ROcp51_811;
  RLcp51_312 = s->dpt[1][14]*ROcp51_310+s->dpt[2][14]*ROcp51_611+s->dpt[3][14]*ROcp51_911;
  OMcp51_112 = OMcp51_111+qd[12]*ROcp51_411;
  OMcp51_212 = OMcp51_211+qd[12]*ROcp51_511;
  OMcp51_312 = OMcp51_311+qd[12]*ROcp51_611;
  ORcp51_112 = OMcp51_211*RLcp51_312-OMcp51_311*RLcp51_212;
  ORcp51_212 = -(OMcp51_111*RLcp51_312-OMcp51_311*RLcp51_112);
  ORcp51_312 = OMcp51_111*RLcp51_212-OMcp51_211*RLcp51_112;
  OPcp51_112 = OPcp51_111+qd[12]*(OMcp51_211*ROcp51_611-OMcp51_311*ROcp51_511)+qdd[12]*ROcp51_411;
  OPcp51_212 = OPcp51_211-qd[12]*(OMcp51_111*ROcp51_611-OMcp51_311*ROcp51_411)+qdd[12]*ROcp51_511;
  OPcp51_312 = OPcp51_311+qd[12]*(OMcp51_111*ROcp51_511-OMcp51_211*ROcp51_411)+qdd[12]*ROcp51_611;
  RLcp51_187 = s->dpt[1][16]*ROcp51_112+s->dpt[2][16]*ROcp51_411+ROcp51_712*s->dpt[3][16];
  RLcp51_287 = s->dpt[1][16]*ROcp51_212+s->dpt[2][16]*ROcp51_511+ROcp51_812*s->dpt[3][16];
  RLcp51_387 = s->dpt[1][16]*ROcp51_312+s->dpt[2][16]*ROcp51_611+ROcp51_912*s->dpt[3][16];
  ORcp51_187 = OMcp51_212*RLcp51_387-OMcp51_312*RLcp51_287;
  ORcp51_287 = -(OMcp51_112*RLcp51_387-OMcp51_312*RLcp51_187);
  ORcp51_387 = OMcp51_112*RLcp51_287-OMcp51_212*RLcp51_187;
  PxF1[1] = q[1]+RLcp51_110+RLcp51_111+RLcp51_112+RLcp51_17+RLcp51_18+RLcp51_187+RLcp51_19;
  PxF1[2] = q[2]+RLcp51_210+RLcp51_211+RLcp51_212+RLcp51_27+RLcp51_28+RLcp51_287+RLcp51_29;
  PxF1[3] = q[3]+RLcp51_310+RLcp51_311+RLcp51_312+RLcp51_37+RLcp51_38+RLcp51_387+RLcp51_39;
  RxF1[1][1] = ROcp51_112;
  RxF1[1][2] = ROcp51_212;
  RxF1[1][3] = ROcp51_312;
  RxF1[2][1] = ROcp51_411;
  RxF1[2][2] = ROcp51_511;
  RxF1[2][3] = ROcp51_611;
  RxF1[3][1] = ROcp51_712;
  RxF1[3][2] = ROcp51_812;
  RxF1[3][3] = ROcp51_912;
  VxF1[1] = qd[1]+ORcp51_110+ORcp51_111+ORcp51_112+ORcp51_17+ORcp51_18+ORcp51_187+ORcp51_19;
  VxF1[2] = qd[2]+ORcp51_210+ORcp51_211+ORcp51_212+ORcp51_27+ORcp51_28+ORcp51_287+ORcp51_29;
  VxF1[3] = qd[3]+ORcp51_310+ORcp51_311+ORcp51_312+ORcp51_37+ORcp51_38+ORcp51_387+ORcp51_39;
  OMxF1[1] = OMcp51_112;
  OMxF1[2] = OMcp51_212;
  OMxF1[3] = OMcp51_312;
  AxF1[1] = qdd[1]+OMcp51_210*ORcp51_311+OMcp51_211*ORcp51_312+OMcp51_212*ORcp51_387+OMcp51_26*ORcp51_37+OMcp51_27*
 ORcp51_38+OMcp51_28*ORcp51_39+OMcp51_29*ORcp51_310-OMcp51_310*ORcp51_211-OMcp51_311*ORcp51_212-OMcp51_312*ORcp51_287-
 OMcp51_36*ORcp51_27-OMcp51_37*ORcp51_28-OMcp51_38*ORcp51_29-OMcp51_39*ORcp51_210+OPcp51_210*RLcp51_311+OPcp51_211*RLcp51_312
 +OPcp51_212*RLcp51_387+OPcp51_26*RLcp51_37+OPcp51_27*RLcp51_38+OPcp51_28*RLcp51_39+OPcp51_29*RLcp51_310-OPcp51_310*
 RLcp51_211-OPcp51_311*RLcp51_212-OPcp51_312*RLcp51_287-OPcp51_36*RLcp51_27-OPcp51_37*RLcp51_28-OPcp51_38*RLcp51_29-OPcp51_39
 *RLcp51_210;
  AxF1[2] = qdd[2]-OMcp51_110*ORcp51_311-OMcp51_111*ORcp51_312-OMcp51_112*ORcp51_387-OMcp51_16*ORcp51_37-OMcp51_17*
 ORcp51_38-OMcp51_18*ORcp51_39-OMcp51_19*ORcp51_310+OMcp51_310*ORcp51_111+OMcp51_311*ORcp51_112+OMcp51_312*ORcp51_187+
 OMcp51_36*ORcp51_17+OMcp51_37*ORcp51_18+OMcp51_38*ORcp51_19+OMcp51_39*ORcp51_110-OPcp51_110*RLcp51_311-OPcp51_111*RLcp51_312
 -OPcp51_112*RLcp51_387-OPcp51_16*RLcp51_37-OPcp51_17*RLcp51_38-OPcp51_18*RLcp51_39-OPcp51_19*RLcp51_310+OPcp51_310*
 RLcp51_111+OPcp51_311*RLcp51_112+OPcp51_312*RLcp51_187+OPcp51_36*RLcp51_17+OPcp51_37*RLcp51_18+OPcp51_38*RLcp51_19+OPcp51_39
 *RLcp51_110;
  AxF1[3] = qdd[3]+OMcp51_110*ORcp51_211+OMcp51_111*ORcp51_212+OMcp51_112*ORcp51_287+OMcp51_16*ORcp51_27+OMcp51_17*
 ORcp51_28+OMcp51_18*ORcp51_29+OMcp51_19*ORcp51_210-OMcp51_210*ORcp51_111-OMcp51_211*ORcp51_112-OMcp51_212*ORcp51_187-
 OMcp51_26*ORcp51_17-OMcp51_27*ORcp51_18-OMcp51_28*ORcp51_19-OMcp51_29*ORcp51_110+OPcp51_110*RLcp51_211+OPcp51_111*RLcp51_212
 +OPcp51_112*RLcp51_287+OPcp51_16*RLcp51_27+OPcp51_17*RLcp51_28+OPcp51_18*RLcp51_29+OPcp51_19*RLcp51_210-OPcp51_210*
 RLcp51_111-OPcp51_211*RLcp51_112-OPcp51_212*RLcp51_187-OPcp51_26*RLcp51_17-OPcp51_27*RLcp51_18-OPcp51_28*RLcp51_19-OPcp51_29
 *RLcp51_110;
  OMPxF1[1] = OPcp51_112;
  OMPxF1[2] = OPcp51_212;
  OMPxF1[3] = OPcp51_312;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc152 = ROcp51_112*SWr1[1]+ROcp51_212*SWr1[2]+ROcp51_312*SWr1[3];
  xfrc252 = ROcp51_411*SWr1[1]+ROcp51_511*SWr1[2]+ROcp51_611*SWr1[3];
  xfrc352 = ROcp51_712*SWr1[1]+ROcp51_812*SWr1[2]+ROcp51_912*SWr1[3];
  s->frc[1][12] = s->frc[1][12]+xfrc152;
  s->frc[2][12] = s->frc[2][12]+xfrc252;
  s->frc[3][12] = s->frc[3][12]+xfrc352;
  xtrq152 = ROcp51_112*SWr1[4]+ROcp51_212*SWr1[5]+ROcp51_312*SWr1[6];
  xtrq252 = ROcp51_411*SWr1[4]+ROcp51_511*SWr1[5]+ROcp51_611*SWr1[6];
  xtrq352 = ROcp51_712*SWr1[4]+ROcp51_812*SWr1[5]+ROcp51_912*SWr1[6];
  s->trq[1][12] = s->trq[1][12]+xtrq152-xfrc252*(SWr1[9]-s->l[3][12])+xfrc352*(SWr1[8]-s->l[2][12]);
  s->trq[2][12] = s->trq[2][12]+xtrq252+xfrc152*(SWr1[9]-s->l[3][12])-xfrc352*(SWr1[7]-s->l[1][12]);
  s->trq[3][12] = s->trq[3][12]+xtrq352-xfrc152*(SWr1[8]-s->l[2][12])+xfrc252*(SWr1[7]-s->l[1][12]);

// = = Block_0_0_1_2_0_1 = = 
 
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
  OMcp52_26 = OMcp52_25+qd[6]*ROcp52_85;
  OMcp52_36 = OMcp52_35+qd[6]*ROcp52_95;
  OPcp52_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp52_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp52_95-OMcp52_35*S5)-qdd[5]*C4-qdd[6]*ROcp52_85);
  OPcp52_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp52_85-OMcp52_25*S5)+qdd[5]*S4+qdd[6]*ROcp52_95;

// = = Block_0_0_1_2_0_2 = = 
 
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
  OMcp52_17 = OMcp52_16+qd[7]*ROcp52_46;
  OMcp52_27 = OMcp52_26+qd[7]*ROcp52_56;
  OMcp52_37 = OMcp52_36+qd[7]*ROcp52_66;
  ORcp52_17 = OMcp52_26*RLcp52_37-OMcp52_36*RLcp52_27;
  ORcp52_27 = -(OMcp52_16*RLcp52_37-OMcp52_36*RLcp52_17);
  ORcp52_37 = OMcp52_16*RLcp52_27-OMcp52_26*RLcp52_17;
  OPcp52_17 = OPcp52_16+qd[7]*(OMcp52_26*ROcp52_66-OMcp52_36*ROcp52_56)+qdd[7]*ROcp52_46;
  OPcp52_27 = OPcp52_26-qd[7]*(OMcp52_16*ROcp52_66-OMcp52_36*ROcp52_46)+qdd[7]*ROcp52_56;
  OPcp52_37 = OPcp52_36+qd[7]*(OMcp52_16*ROcp52_56-OMcp52_26*ROcp52_46)+qdd[7]*ROcp52_66;
  RLcp52_18 = s->dpt[1][6]*ROcp52_17+s->dpt[3][6]*ROcp52_77+ROcp52_46*s->dpt[2][6];
  RLcp52_28 = s->dpt[1][6]*ROcp52_27+s->dpt[3][6]*ROcp52_87+ROcp52_56*s->dpt[2][6];
  RLcp52_38 = s->dpt[1][6]*ROcp52_37+s->dpt[3][6]*ROcp52_97+ROcp52_66*s->dpt[2][6];
  OMcp52_18 = OMcp52_17+qd[8]*ROcp52_17;
  OMcp52_28 = OMcp52_27+qd[8]*ROcp52_27;
  OMcp52_38 = OMcp52_37+qd[8]*ROcp52_37;
  ORcp52_18 = OMcp52_27*RLcp52_38-OMcp52_37*RLcp52_28;
  ORcp52_28 = -(OMcp52_17*RLcp52_38-OMcp52_37*RLcp52_18);
  ORcp52_38 = OMcp52_17*RLcp52_28-OMcp52_27*RLcp52_18;
  OPcp52_18 = OPcp52_17+qd[8]*(OMcp52_27*ROcp52_37-OMcp52_37*ROcp52_27)+qdd[8]*ROcp52_17;
  OPcp52_28 = OPcp52_27-qd[8]*(OMcp52_17*ROcp52_37-OMcp52_37*ROcp52_17)+qdd[8]*ROcp52_27;
  OPcp52_38 = OPcp52_37+qd[8]*(OMcp52_17*ROcp52_27-OMcp52_27*ROcp52_17)+qdd[8]*ROcp52_37;
  RLcp52_19 = s->dpt[1][8]*ROcp52_17+s->dpt[2][8]*ROcp52_48+ROcp52_78*s->dpt[3][8];
  RLcp52_29 = s->dpt[1][8]*ROcp52_27+s->dpt[2][8]*ROcp52_58+ROcp52_88*s->dpt[3][8];
  RLcp52_39 = s->dpt[1][8]*ROcp52_37+s->dpt[2][8]*ROcp52_68+ROcp52_98*s->dpt[3][8];
  OMcp52_19 = OMcp52_18+qd[9]*ROcp52_78;
  OMcp52_29 = OMcp52_28+qd[9]*ROcp52_88;
  OMcp52_39 = OMcp52_38+qd[9]*ROcp52_98;
  ORcp52_19 = OMcp52_28*RLcp52_39-OMcp52_38*RLcp52_29;
  ORcp52_29 = -(OMcp52_18*RLcp52_39-OMcp52_38*RLcp52_19);
  ORcp52_39 = OMcp52_18*RLcp52_29-OMcp52_28*RLcp52_19;
  OPcp52_19 = OPcp52_18+qd[9]*(OMcp52_28*ROcp52_98-OMcp52_38*ROcp52_88)+qdd[9]*ROcp52_78;
  OPcp52_29 = OPcp52_28-qd[9]*(OMcp52_18*ROcp52_98-OMcp52_38*ROcp52_78)+qdd[9]*ROcp52_88;
  OPcp52_39 = OPcp52_38+qd[9]*(OMcp52_18*ROcp52_88-OMcp52_28*ROcp52_78)+qdd[9]*ROcp52_98;
  RLcp52_110 = s->dpt[1][10]*ROcp52_19+s->dpt[2][10]*ROcp52_49+ROcp52_78*s->dpt[3][10];
  RLcp52_210 = s->dpt[1][10]*ROcp52_29+s->dpt[2][10]*ROcp52_59+ROcp52_88*s->dpt[3][10];
  RLcp52_310 = s->dpt[1][10]*ROcp52_39+s->dpt[2][10]*ROcp52_69+ROcp52_98*s->dpt[3][10];
  OMcp52_110 = OMcp52_19+qd[10]*ROcp52_49;
  OMcp52_210 = OMcp52_29+qd[10]*ROcp52_59;
  OMcp52_310 = OMcp52_39+qd[10]*ROcp52_69;
  ORcp52_110 = OMcp52_29*RLcp52_310-OMcp52_39*RLcp52_210;
  ORcp52_210 = -(OMcp52_19*RLcp52_310-OMcp52_39*RLcp52_110);
  ORcp52_310 = OMcp52_19*RLcp52_210-OMcp52_29*RLcp52_110;
  OPcp52_110 = OPcp52_19+qd[10]*(OMcp52_29*ROcp52_69-OMcp52_39*ROcp52_59)+qdd[10]*ROcp52_49;
  OPcp52_210 = OPcp52_29-qd[10]*(OMcp52_19*ROcp52_69-OMcp52_39*ROcp52_49)+qdd[10]*ROcp52_59;
  OPcp52_310 = OPcp52_39+qd[10]*(OMcp52_19*ROcp52_59-OMcp52_29*ROcp52_49)+qdd[10]*ROcp52_69;
  RLcp52_111 = s->dpt[1][12]*ROcp52_110+s->dpt[2][12]*ROcp52_49+ROcp52_710*s->dpt[3][12];
  RLcp52_211 = s->dpt[1][12]*ROcp52_210+s->dpt[2][12]*ROcp52_59+ROcp52_810*s->dpt[3][12];
  RLcp52_311 = s->dpt[1][12]*ROcp52_310+s->dpt[2][12]*ROcp52_69+ROcp52_910*s->dpt[3][12];
  OMcp52_111 = OMcp52_110+qd[11]*ROcp52_110;
  OMcp52_211 = OMcp52_210+qd[11]*ROcp52_210;
  OMcp52_311 = OMcp52_310+qd[11]*ROcp52_310;
  ORcp52_111 = OMcp52_210*RLcp52_311-OMcp52_310*RLcp52_211;
  ORcp52_211 = -(OMcp52_110*RLcp52_311-OMcp52_310*RLcp52_111);
  ORcp52_311 = OMcp52_110*RLcp52_211-OMcp52_210*RLcp52_111;
  OPcp52_111 = OPcp52_110+qd[11]*(OMcp52_210*ROcp52_310-OMcp52_310*ROcp52_210)+qdd[11]*ROcp52_110;
  OPcp52_211 = OPcp52_210-qd[11]*(OMcp52_110*ROcp52_310-OMcp52_310*ROcp52_110)+qdd[11]*ROcp52_210;
  OPcp52_311 = OPcp52_310+qd[11]*(OMcp52_110*ROcp52_210-OMcp52_210*ROcp52_110)+qdd[11]*ROcp52_310;
  RLcp52_112 = s->dpt[1][14]*ROcp52_110+s->dpt[2][14]*ROcp52_411+s->dpt[3][14]*ROcp52_711;
  RLcp52_212 = s->dpt[1][14]*ROcp52_210+s->dpt[2][14]*ROcp52_511+s->dpt[3][14]*ROcp52_811;
  RLcp52_312 = s->dpt[1][14]*ROcp52_310+s->dpt[2][14]*ROcp52_611+s->dpt[3][14]*ROcp52_911;
  OMcp52_112 = OMcp52_111+qd[12]*ROcp52_411;
  OMcp52_212 = OMcp52_211+qd[12]*ROcp52_511;
  OMcp52_312 = OMcp52_311+qd[12]*ROcp52_611;
  ORcp52_112 = OMcp52_211*RLcp52_312-OMcp52_311*RLcp52_212;
  ORcp52_212 = -(OMcp52_111*RLcp52_312-OMcp52_311*RLcp52_112);
  ORcp52_312 = OMcp52_111*RLcp52_212-OMcp52_211*RLcp52_112;
  OPcp52_112 = OPcp52_111+qd[12]*(OMcp52_211*ROcp52_611-OMcp52_311*ROcp52_511)+qdd[12]*ROcp52_411;
  OPcp52_212 = OPcp52_211-qd[12]*(OMcp52_111*ROcp52_611-OMcp52_311*ROcp52_411)+qdd[12]*ROcp52_511;
  OPcp52_312 = OPcp52_311+qd[12]*(OMcp52_111*ROcp52_511-OMcp52_211*ROcp52_411)+qdd[12]*ROcp52_611;
  RLcp52_188 = ROcp52_112*s->dpt[1][18]+ROcp52_411*s->dpt[2][18]+ROcp52_712*s->dpt[3][18];
  RLcp52_288 = ROcp52_212*s->dpt[1][18]+ROcp52_511*s->dpt[2][18]+ROcp52_812*s->dpt[3][18];
  RLcp52_388 = ROcp52_312*s->dpt[1][18]+ROcp52_611*s->dpt[2][18]+ROcp52_912*s->dpt[3][18];
  ORcp52_188 = OMcp52_212*RLcp52_388-OMcp52_312*RLcp52_288;
  ORcp52_288 = -(OMcp52_112*RLcp52_388-OMcp52_312*RLcp52_188);
  ORcp52_388 = OMcp52_112*RLcp52_288-OMcp52_212*RLcp52_188;
  PxF2[1] = q[1]+RLcp52_110+RLcp52_111+RLcp52_112+RLcp52_17+RLcp52_18+RLcp52_188+RLcp52_19;
  PxF2[2] = q[2]+RLcp52_210+RLcp52_211+RLcp52_212+RLcp52_27+RLcp52_28+RLcp52_288+RLcp52_29;
  PxF2[3] = q[3]+RLcp52_310+RLcp52_311+RLcp52_312+RLcp52_37+RLcp52_38+RLcp52_388+RLcp52_39;
  RxF2[1][1] = ROcp52_112;
  RxF2[1][2] = ROcp52_212;
  RxF2[1][3] = ROcp52_312;
  RxF2[2][1] = ROcp52_411;
  RxF2[2][2] = ROcp52_511;
  RxF2[2][3] = ROcp52_611;
  RxF2[3][1] = ROcp52_712;
  RxF2[3][2] = ROcp52_812;
  RxF2[3][3] = ROcp52_912;
  VxF2[1] = qd[1]+ORcp52_110+ORcp52_111+ORcp52_112+ORcp52_17+ORcp52_18+ORcp52_188+ORcp52_19;
  VxF2[2] = qd[2]+ORcp52_210+ORcp52_211+ORcp52_212+ORcp52_27+ORcp52_28+ORcp52_288+ORcp52_29;
  VxF2[3] = qd[3]+ORcp52_310+ORcp52_311+ORcp52_312+ORcp52_37+ORcp52_38+ORcp52_388+ORcp52_39;
  OMxF2[1] = OMcp52_112;
  OMxF2[2] = OMcp52_212;
  OMxF2[3] = OMcp52_312;
  AxF2[1] = qdd[1]+OMcp52_210*ORcp52_311+OMcp52_211*ORcp52_312+OMcp52_212*ORcp52_388+OMcp52_26*ORcp52_37+OMcp52_27*
 ORcp52_38+OMcp52_28*ORcp52_39+OMcp52_29*ORcp52_310-OMcp52_310*ORcp52_211-OMcp52_311*ORcp52_212-OMcp52_312*ORcp52_288-
 OMcp52_36*ORcp52_27-OMcp52_37*ORcp52_28-OMcp52_38*ORcp52_29-OMcp52_39*ORcp52_210+OPcp52_210*RLcp52_311+OPcp52_211*RLcp52_312
 +OPcp52_212*RLcp52_388+OPcp52_26*RLcp52_37+OPcp52_27*RLcp52_38+OPcp52_28*RLcp52_39+OPcp52_29*RLcp52_310-OPcp52_310*
 RLcp52_211-OPcp52_311*RLcp52_212-OPcp52_312*RLcp52_288-OPcp52_36*RLcp52_27-OPcp52_37*RLcp52_28-OPcp52_38*RLcp52_29-OPcp52_39
 *RLcp52_210;
  AxF2[2] = qdd[2]-OMcp52_110*ORcp52_311-OMcp52_111*ORcp52_312-OMcp52_112*ORcp52_388-OMcp52_16*ORcp52_37-OMcp52_17*
 ORcp52_38-OMcp52_18*ORcp52_39-OMcp52_19*ORcp52_310+OMcp52_310*ORcp52_111+OMcp52_311*ORcp52_112+OMcp52_312*ORcp52_188+
 OMcp52_36*ORcp52_17+OMcp52_37*ORcp52_18+OMcp52_38*ORcp52_19+OMcp52_39*ORcp52_110-OPcp52_110*RLcp52_311-OPcp52_111*RLcp52_312
 -OPcp52_112*RLcp52_388-OPcp52_16*RLcp52_37-OPcp52_17*RLcp52_38-OPcp52_18*RLcp52_39-OPcp52_19*RLcp52_310+OPcp52_310*
 RLcp52_111+OPcp52_311*RLcp52_112+OPcp52_312*RLcp52_188+OPcp52_36*RLcp52_17+OPcp52_37*RLcp52_18+OPcp52_38*RLcp52_19+OPcp52_39
 *RLcp52_110;
  AxF2[3] = qdd[3]+OMcp52_110*ORcp52_211+OMcp52_111*ORcp52_212+OMcp52_112*ORcp52_288+OMcp52_16*ORcp52_27+OMcp52_17*
 ORcp52_28+OMcp52_18*ORcp52_29+OMcp52_19*ORcp52_210-OMcp52_210*ORcp52_111-OMcp52_211*ORcp52_112-OMcp52_212*ORcp52_188-
 OMcp52_26*ORcp52_17-OMcp52_27*ORcp52_18-OMcp52_28*ORcp52_19-OMcp52_29*ORcp52_110+OPcp52_110*RLcp52_211+OPcp52_111*RLcp52_212
 +OPcp52_112*RLcp52_288+OPcp52_16*RLcp52_27+OPcp52_17*RLcp52_28+OPcp52_18*RLcp52_29+OPcp52_19*RLcp52_210-OPcp52_210*
 RLcp52_111-OPcp52_211*RLcp52_112-OPcp52_212*RLcp52_188-OPcp52_26*RLcp52_17-OPcp52_27*RLcp52_18-OPcp52_28*RLcp52_19-OPcp52_29
 *RLcp52_110;
  OMPxF2[1] = OPcp52_112;
  OMPxF2[2] = OPcp52_212;
  OMPxF2[3] = OPcp52_312;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc153 = ROcp52_112*SWr2[1]+ROcp52_212*SWr2[2]+ROcp52_312*SWr2[3];
  xfrc253 = ROcp52_411*SWr2[1]+ROcp52_511*SWr2[2]+ROcp52_611*SWr2[3];
  xfrc353 = ROcp52_712*SWr2[1]+ROcp52_812*SWr2[2]+ROcp52_912*SWr2[3];
  s->frc[1][12] = s->frc[1][12]+xfrc153;
  s->frc[2][12] = s->frc[2][12]+xfrc253;
  s->frc[3][12] = s->frc[3][12]+xfrc353;
  xtrq153 = ROcp52_112*SWr2[4]+ROcp52_212*SWr2[5]+ROcp52_312*SWr2[6];
  xtrq253 = ROcp52_411*SWr2[4]+ROcp52_511*SWr2[5]+ROcp52_611*SWr2[6];
  xtrq353 = ROcp52_712*SWr2[4]+ROcp52_812*SWr2[5]+ROcp52_912*SWr2[6];
  s->trq[1][12] = s->trq[1][12]+xtrq153-xfrc253*(SWr2[9]-s->l[3][12])+xfrc353*(SWr2[8]-s->l[2][12]);
  s->trq[2][12] = s->trq[2][12]+xtrq253+xfrc153*(SWr2[9]-s->l[3][12])-xfrc353*(SWr2[7]-s->l[1][12]);
  s->trq[3][12] = s->trq[3][12]+xtrq353-xfrc153*(SWr2[8]-s->l[2][12])+xfrc253*(SWr2[7]-s->l[1][12]);

// = = Block_0_0_1_3_0_1 = = 
 
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
  OMcp53_26 = OMcp53_25+qd[6]*ROcp53_85;
  OMcp53_36 = OMcp53_35+qd[6]*ROcp53_95;
  OPcp53_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp53_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp53_95-OMcp53_35*S5)-qdd[5]*C4-qdd[6]*ROcp53_85);
  OPcp53_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp53_85-OMcp53_25*S5)+qdd[5]*S4+qdd[6]*ROcp53_95;

// = = Block_0_0_1_3_0_2 = = 
 
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
  OMcp53_17 = OMcp53_16+qd[7]*ROcp53_46;
  OMcp53_27 = OMcp53_26+qd[7]*ROcp53_56;
  OMcp53_37 = OMcp53_36+qd[7]*ROcp53_66;
  ORcp53_17 = OMcp53_26*RLcp53_37-OMcp53_36*RLcp53_27;
  ORcp53_27 = -(OMcp53_16*RLcp53_37-OMcp53_36*RLcp53_17);
  ORcp53_37 = OMcp53_16*RLcp53_27-OMcp53_26*RLcp53_17;
  OPcp53_17 = OPcp53_16+qd[7]*(OMcp53_26*ROcp53_66-OMcp53_36*ROcp53_56)+qdd[7]*ROcp53_46;
  OPcp53_27 = OPcp53_26-qd[7]*(OMcp53_16*ROcp53_66-OMcp53_36*ROcp53_46)+qdd[7]*ROcp53_56;
  OPcp53_37 = OPcp53_36+qd[7]*(OMcp53_16*ROcp53_56-OMcp53_26*ROcp53_46)+qdd[7]*ROcp53_66;
  RLcp53_18 = s->dpt[1][6]*ROcp53_17+s->dpt[3][6]*ROcp53_77+ROcp53_46*s->dpt[2][6];
  RLcp53_28 = s->dpt[1][6]*ROcp53_27+s->dpt[3][6]*ROcp53_87+ROcp53_56*s->dpt[2][6];
  RLcp53_38 = s->dpt[1][6]*ROcp53_37+s->dpt[3][6]*ROcp53_97+ROcp53_66*s->dpt[2][6];
  OMcp53_18 = OMcp53_17+qd[8]*ROcp53_17;
  OMcp53_28 = OMcp53_27+qd[8]*ROcp53_27;
  OMcp53_38 = OMcp53_37+qd[8]*ROcp53_37;
  ORcp53_18 = OMcp53_27*RLcp53_38-OMcp53_37*RLcp53_28;
  ORcp53_28 = -(OMcp53_17*RLcp53_38-OMcp53_37*RLcp53_18);
  ORcp53_38 = OMcp53_17*RLcp53_28-OMcp53_27*RLcp53_18;
  OPcp53_18 = OPcp53_17+qd[8]*(OMcp53_27*ROcp53_37-OMcp53_37*ROcp53_27)+qdd[8]*ROcp53_17;
  OPcp53_28 = OPcp53_27-qd[8]*(OMcp53_17*ROcp53_37-OMcp53_37*ROcp53_17)+qdd[8]*ROcp53_27;
  OPcp53_38 = OPcp53_37+qd[8]*(OMcp53_17*ROcp53_27-OMcp53_27*ROcp53_17)+qdd[8]*ROcp53_37;
  RLcp53_19 = s->dpt[1][8]*ROcp53_17+s->dpt[2][8]*ROcp53_48+ROcp53_78*s->dpt[3][8];
  RLcp53_29 = s->dpt[1][8]*ROcp53_27+s->dpt[2][8]*ROcp53_58+ROcp53_88*s->dpt[3][8];
  RLcp53_39 = s->dpt[1][8]*ROcp53_37+s->dpt[2][8]*ROcp53_68+ROcp53_98*s->dpt[3][8];
  OMcp53_19 = OMcp53_18+qd[9]*ROcp53_78;
  OMcp53_29 = OMcp53_28+qd[9]*ROcp53_88;
  OMcp53_39 = OMcp53_38+qd[9]*ROcp53_98;
  ORcp53_19 = OMcp53_28*RLcp53_39-OMcp53_38*RLcp53_29;
  ORcp53_29 = -(OMcp53_18*RLcp53_39-OMcp53_38*RLcp53_19);
  ORcp53_39 = OMcp53_18*RLcp53_29-OMcp53_28*RLcp53_19;
  OPcp53_19 = OPcp53_18+qd[9]*(OMcp53_28*ROcp53_98-OMcp53_38*ROcp53_88)+qdd[9]*ROcp53_78;
  OPcp53_29 = OPcp53_28-qd[9]*(OMcp53_18*ROcp53_98-OMcp53_38*ROcp53_78)+qdd[9]*ROcp53_88;
  OPcp53_39 = OPcp53_38+qd[9]*(OMcp53_18*ROcp53_88-OMcp53_28*ROcp53_78)+qdd[9]*ROcp53_98;
  RLcp53_110 = s->dpt[1][10]*ROcp53_19+s->dpt[2][10]*ROcp53_49+ROcp53_78*s->dpt[3][10];
  RLcp53_210 = s->dpt[1][10]*ROcp53_29+s->dpt[2][10]*ROcp53_59+ROcp53_88*s->dpt[3][10];
  RLcp53_310 = s->dpt[1][10]*ROcp53_39+s->dpt[2][10]*ROcp53_69+ROcp53_98*s->dpt[3][10];
  OMcp53_110 = OMcp53_19+qd[10]*ROcp53_49;
  OMcp53_210 = OMcp53_29+qd[10]*ROcp53_59;
  OMcp53_310 = OMcp53_39+qd[10]*ROcp53_69;
  ORcp53_110 = OMcp53_29*RLcp53_310-OMcp53_39*RLcp53_210;
  ORcp53_210 = -(OMcp53_19*RLcp53_310-OMcp53_39*RLcp53_110);
  ORcp53_310 = OMcp53_19*RLcp53_210-OMcp53_29*RLcp53_110;
  OPcp53_110 = OPcp53_19+qd[10]*(OMcp53_29*ROcp53_69-OMcp53_39*ROcp53_59)+qdd[10]*ROcp53_49;
  OPcp53_210 = OPcp53_29-qd[10]*(OMcp53_19*ROcp53_69-OMcp53_39*ROcp53_49)+qdd[10]*ROcp53_59;
  OPcp53_310 = OPcp53_39+qd[10]*(OMcp53_19*ROcp53_59-OMcp53_29*ROcp53_49)+qdd[10]*ROcp53_69;
  RLcp53_111 = s->dpt[1][12]*ROcp53_110+s->dpt[2][12]*ROcp53_49+ROcp53_710*s->dpt[3][12];
  RLcp53_211 = s->dpt[1][12]*ROcp53_210+s->dpt[2][12]*ROcp53_59+ROcp53_810*s->dpt[3][12];
  RLcp53_311 = s->dpt[1][12]*ROcp53_310+s->dpt[2][12]*ROcp53_69+ROcp53_910*s->dpt[3][12];
  OMcp53_111 = OMcp53_110+qd[11]*ROcp53_110;
  OMcp53_211 = OMcp53_210+qd[11]*ROcp53_210;
  OMcp53_311 = OMcp53_310+qd[11]*ROcp53_310;
  ORcp53_111 = OMcp53_210*RLcp53_311-OMcp53_310*RLcp53_211;
  ORcp53_211 = -(OMcp53_110*RLcp53_311-OMcp53_310*RLcp53_111);
  ORcp53_311 = OMcp53_110*RLcp53_211-OMcp53_210*RLcp53_111;
  OPcp53_111 = OPcp53_110+qd[11]*(OMcp53_210*ROcp53_310-OMcp53_310*ROcp53_210)+qdd[11]*ROcp53_110;
  OPcp53_211 = OPcp53_210-qd[11]*(OMcp53_110*ROcp53_310-OMcp53_310*ROcp53_110)+qdd[11]*ROcp53_210;
  OPcp53_311 = OPcp53_310+qd[11]*(OMcp53_110*ROcp53_210-OMcp53_210*ROcp53_110)+qdd[11]*ROcp53_310;
  RLcp53_112 = s->dpt[1][14]*ROcp53_110+s->dpt[2][14]*ROcp53_411+s->dpt[3][14]*ROcp53_711;
  RLcp53_212 = s->dpt[1][14]*ROcp53_210+s->dpt[2][14]*ROcp53_511+s->dpt[3][14]*ROcp53_811;
  RLcp53_312 = s->dpt[1][14]*ROcp53_310+s->dpt[2][14]*ROcp53_611+s->dpt[3][14]*ROcp53_911;
  OMcp53_112 = OMcp53_111+qd[12]*ROcp53_411;
  OMcp53_212 = OMcp53_211+qd[12]*ROcp53_511;
  OMcp53_312 = OMcp53_311+qd[12]*ROcp53_611;
  ORcp53_112 = OMcp53_211*RLcp53_312-OMcp53_311*RLcp53_212;
  ORcp53_212 = -(OMcp53_111*RLcp53_312-OMcp53_311*RLcp53_112);
  ORcp53_312 = OMcp53_111*RLcp53_212-OMcp53_211*RLcp53_112;
  OPcp53_112 = OPcp53_111+qd[12]*(OMcp53_211*ROcp53_611-OMcp53_311*ROcp53_511)+qdd[12]*ROcp53_411;
  OPcp53_212 = OPcp53_211-qd[12]*(OMcp53_111*ROcp53_611-OMcp53_311*ROcp53_411)+qdd[12]*ROcp53_511;
  OPcp53_312 = OPcp53_311+qd[12]*(OMcp53_111*ROcp53_511-OMcp53_211*ROcp53_411)+qdd[12]*ROcp53_611;
  RLcp53_189 = ROcp53_112*s->dpt[1][19]+ROcp53_411*s->dpt[2][19]+ROcp53_712*s->dpt[3][19];
  RLcp53_289 = ROcp53_212*s->dpt[1][19]+ROcp53_511*s->dpt[2][19]+ROcp53_812*s->dpt[3][19];
  RLcp53_389 = ROcp53_312*s->dpt[1][19]+ROcp53_611*s->dpt[2][19]+ROcp53_912*s->dpt[3][19];
  ORcp53_189 = OMcp53_212*RLcp53_389-OMcp53_312*RLcp53_289;
  ORcp53_289 = -(OMcp53_112*RLcp53_389-OMcp53_312*RLcp53_189);
  ORcp53_389 = OMcp53_112*RLcp53_289-OMcp53_212*RLcp53_189;
  PxF3[1] = q[1]+RLcp53_110+RLcp53_111+RLcp53_112+RLcp53_17+RLcp53_18+RLcp53_189+RLcp53_19;
  PxF3[2] = q[2]+RLcp53_210+RLcp53_211+RLcp53_212+RLcp53_27+RLcp53_28+RLcp53_289+RLcp53_29;
  PxF3[3] = q[3]+RLcp53_310+RLcp53_311+RLcp53_312+RLcp53_37+RLcp53_38+RLcp53_389+RLcp53_39;
  RxF3[1][1] = ROcp53_112;
  RxF3[1][2] = ROcp53_212;
  RxF3[1][3] = ROcp53_312;
  RxF3[2][1] = ROcp53_411;
  RxF3[2][2] = ROcp53_511;
  RxF3[2][3] = ROcp53_611;
  RxF3[3][1] = ROcp53_712;
  RxF3[3][2] = ROcp53_812;
  RxF3[3][3] = ROcp53_912;
  VxF3[1] = qd[1]+ORcp53_110+ORcp53_111+ORcp53_112+ORcp53_17+ORcp53_18+ORcp53_189+ORcp53_19;
  VxF3[2] = qd[2]+ORcp53_210+ORcp53_211+ORcp53_212+ORcp53_27+ORcp53_28+ORcp53_289+ORcp53_29;
  VxF3[3] = qd[3]+ORcp53_310+ORcp53_311+ORcp53_312+ORcp53_37+ORcp53_38+ORcp53_389+ORcp53_39;
  OMxF3[1] = OMcp53_112;
  OMxF3[2] = OMcp53_212;
  OMxF3[3] = OMcp53_312;
  AxF3[1] = qdd[1]+OMcp53_210*ORcp53_311+OMcp53_211*ORcp53_312+OMcp53_212*ORcp53_389+OMcp53_26*ORcp53_37+OMcp53_27*
 ORcp53_38+OMcp53_28*ORcp53_39+OMcp53_29*ORcp53_310-OMcp53_310*ORcp53_211-OMcp53_311*ORcp53_212-OMcp53_312*ORcp53_289-
 OMcp53_36*ORcp53_27-OMcp53_37*ORcp53_28-OMcp53_38*ORcp53_29-OMcp53_39*ORcp53_210+OPcp53_210*RLcp53_311+OPcp53_211*RLcp53_312
 +OPcp53_212*RLcp53_389+OPcp53_26*RLcp53_37+OPcp53_27*RLcp53_38+OPcp53_28*RLcp53_39+OPcp53_29*RLcp53_310-OPcp53_310*
 RLcp53_211-OPcp53_311*RLcp53_212-OPcp53_312*RLcp53_289-OPcp53_36*RLcp53_27-OPcp53_37*RLcp53_28-OPcp53_38*RLcp53_29-OPcp53_39
 *RLcp53_210;
  AxF3[2] = qdd[2]-OMcp53_110*ORcp53_311-OMcp53_111*ORcp53_312-OMcp53_112*ORcp53_389-OMcp53_16*ORcp53_37-OMcp53_17*
 ORcp53_38-OMcp53_18*ORcp53_39-OMcp53_19*ORcp53_310+OMcp53_310*ORcp53_111+OMcp53_311*ORcp53_112+OMcp53_312*ORcp53_189+
 OMcp53_36*ORcp53_17+OMcp53_37*ORcp53_18+OMcp53_38*ORcp53_19+OMcp53_39*ORcp53_110-OPcp53_110*RLcp53_311-OPcp53_111*RLcp53_312
 -OPcp53_112*RLcp53_389-OPcp53_16*RLcp53_37-OPcp53_17*RLcp53_38-OPcp53_18*RLcp53_39-OPcp53_19*RLcp53_310+OPcp53_310*
 RLcp53_111+OPcp53_311*RLcp53_112+OPcp53_312*RLcp53_189+OPcp53_36*RLcp53_17+OPcp53_37*RLcp53_18+OPcp53_38*RLcp53_19+OPcp53_39
 *RLcp53_110;
  AxF3[3] = qdd[3]+OMcp53_110*ORcp53_211+OMcp53_111*ORcp53_212+OMcp53_112*ORcp53_289+OMcp53_16*ORcp53_27+OMcp53_17*
 ORcp53_28+OMcp53_18*ORcp53_29+OMcp53_19*ORcp53_210-OMcp53_210*ORcp53_111-OMcp53_211*ORcp53_112-OMcp53_212*ORcp53_189-
 OMcp53_26*ORcp53_17-OMcp53_27*ORcp53_18-OMcp53_28*ORcp53_19-OMcp53_29*ORcp53_110+OPcp53_110*RLcp53_211+OPcp53_111*RLcp53_212
 +OPcp53_112*RLcp53_289+OPcp53_16*RLcp53_27+OPcp53_17*RLcp53_28+OPcp53_18*RLcp53_29+OPcp53_19*RLcp53_210-OPcp53_210*
 RLcp53_111-OPcp53_211*RLcp53_112-OPcp53_212*RLcp53_189-OPcp53_26*RLcp53_17-OPcp53_27*RLcp53_18-OPcp53_28*RLcp53_19-OPcp53_29
 *RLcp53_110;
  OMPxF3[1] = OPcp53_112;
  OMPxF3[2] = OPcp53_212;
  OMPxF3[3] = OPcp53_312;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc154 = ROcp53_112*SWr3[1]+ROcp53_212*SWr3[2]+ROcp53_312*SWr3[3];
  xfrc254 = ROcp53_411*SWr3[1]+ROcp53_511*SWr3[2]+ROcp53_611*SWr3[3];
  xfrc354 = ROcp53_712*SWr3[1]+ROcp53_812*SWr3[2]+ROcp53_912*SWr3[3];
  s->frc[1][12] = s->frc[1][12]+xfrc154;
  s->frc[2][12] = s->frc[2][12]+xfrc254;
  s->frc[3][12] = s->frc[3][12]+xfrc354;
  xtrq154 = ROcp53_112*SWr3[4]+ROcp53_212*SWr3[5]+ROcp53_312*SWr3[6];
  xtrq254 = ROcp53_411*SWr3[4]+ROcp53_511*SWr3[5]+ROcp53_611*SWr3[6];
  xtrq354 = ROcp53_712*SWr3[4]+ROcp53_812*SWr3[5]+ROcp53_912*SWr3[6];
  s->trq[1][12] = s->trq[1][12]+xtrq154-xfrc254*(SWr3[9]-s->l[3][12])+xfrc354*(SWr3[8]-s->l[2][12]);
  s->trq[2][12] = s->trq[2][12]+xtrq254+xfrc154*(SWr3[9]-s->l[3][12])-xfrc354*(SWr3[7]-s->l[1][12]);
  s->trq[3][12] = s->trq[3][12]+xtrq354-xfrc154*(SWr3[8]-s->l[2][12])+xfrc254*(SWr3[7]-s->l[1][12]);

// = = Block_0_0_1_4_0_1 = = 
 
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
  OMcp54_26 = OMcp54_25+qd[6]*ROcp54_85;
  OMcp54_36 = OMcp54_35+qd[6]*ROcp54_95;
  OPcp54_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp54_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp54_95-OMcp54_35*S5)-qdd[5]*C4-qdd[6]*ROcp54_85);
  OPcp54_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp54_85-OMcp54_25*S5)+qdd[5]*S4+qdd[6]*ROcp54_95;

// = = Block_0_0_1_4_0_2 = = 
 
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
  OMcp54_17 = OMcp54_16+qd[7]*ROcp54_46;
  OMcp54_27 = OMcp54_26+qd[7]*ROcp54_56;
  OMcp54_37 = OMcp54_36+qd[7]*ROcp54_66;
  ORcp54_17 = OMcp54_26*RLcp54_37-OMcp54_36*RLcp54_27;
  ORcp54_27 = -(OMcp54_16*RLcp54_37-OMcp54_36*RLcp54_17);
  ORcp54_37 = OMcp54_16*RLcp54_27-OMcp54_26*RLcp54_17;
  OPcp54_17 = OPcp54_16+qd[7]*(OMcp54_26*ROcp54_66-OMcp54_36*ROcp54_56)+qdd[7]*ROcp54_46;
  OPcp54_27 = OPcp54_26-qd[7]*(OMcp54_16*ROcp54_66-OMcp54_36*ROcp54_46)+qdd[7]*ROcp54_56;
  OPcp54_37 = OPcp54_36+qd[7]*(OMcp54_16*ROcp54_56-OMcp54_26*ROcp54_46)+qdd[7]*ROcp54_66;
  RLcp54_18 = s->dpt[1][6]*ROcp54_17+s->dpt[3][6]*ROcp54_77+ROcp54_46*s->dpt[2][6];
  RLcp54_28 = s->dpt[1][6]*ROcp54_27+s->dpt[3][6]*ROcp54_87+ROcp54_56*s->dpt[2][6];
  RLcp54_38 = s->dpt[1][6]*ROcp54_37+s->dpt[3][6]*ROcp54_97+ROcp54_66*s->dpt[2][6];
  OMcp54_18 = OMcp54_17+qd[8]*ROcp54_17;
  OMcp54_28 = OMcp54_27+qd[8]*ROcp54_27;
  OMcp54_38 = OMcp54_37+qd[8]*ROcp54_37;
  ORcp54_18 = OMcp54_27*RLcp54_38-OMcp54_37*RLcp54_28;
  ORcp54_28 = -(OMcp54_17*RLcp54_38-OMcp54_37*RLcp54_18);
  ORcp54_38 = OMcp54_17*RLcp54_28-OMcp54_27*RLcp54_18;
  OPcp54_18 = OPcp54_17+qd[8]*(OMcp54_27*ROcp54_37-OMcp54_37*ROcp54_27)+qdd[8]*ROcp54_17;
  OPcp54_28 = OPcp54_27-qd[8]*(OMcp54_17*ROcp54_37-OMcp54_37*ROcp54_17)+qdd[8]*ROcp54_27;
  OPcp54_38 = OPcp54_37+qd[8]*(OMcp54_17*ROcp54_27-OMcp54_27*ROcp54_17)+qdd[8]*ROcp54_37;
  RLcp54_19 = s->dpt[1][8]*ROcp54_17+s->dpt[2][8]*ROcp54_48+ROcp54_78*s->dpt[3][8];
  RLcp54_29 = s->dpt[1][8]*ROcp54_27+s->dpt[2][8]*ROcp54_58+ROcp54_88*s->dpt[3][8];
  RLcp54_39 = s->dpt[1][8]*ROcp54_37+s->dpt[2][8]*ROcp54_68+ROcp54_98*s->dpt[3][8];
  OMcp54_19 = OMcp54_18+qd[9]*ROcp54_78;
  OMcp54_29 = OMcp54_28+qd[9]*ROcp54_88;
  OMcp54_39 = OMcp54_38+qd[9]*ROcp54_98;
  ORcp54_19 = OMcp54_28*RLcp54_39-OMcp54_38*RLcp54_29;
  ORcp54_29 = -(OMcp54_18*RLcp54_39-OMcp54_38*RLcp54_19);
  ORcp54_39 = OMcp54_18*RLcp54_29-OMcp54_28*RLcp54_19;
  OPcp54_19 = OPcp54_18+qd[9]*(OMcp54_28*ROcp54_98-OMcp54_38*ROcp54_88)+qdd[9]*ROcp54_78;
  OPcp54_29 = OPcp54_28-qd[9]*(OMcp54_18*ROcp54_98-OMcp54_38*ROcp54_78)+qdd[9]*ROcp54_88;
  OPcp54_39 = OPcp54_38+qd[9]*(OMcp54_18*ROcp54_88-OMcp54_28*ROcp54_78)+qdd[9]*ROcp54_98;
  RLcp54_110 = s->dpt[1][10]*ROcp54_19+s->dpt[2][10]*ROcp54_49+ROcp54_78*s->dpt[3][10];
  RLcp54_210 = s->dpt[1][10]*ROcp54_29+s->dpt[2][10]*ROcp54_59+ROcp54_88*s->dpt[3][10];
  RLcp54_310 = s->dpt[1][10]*ROcp54_39+s->dpt[2][10]*ROcp54_69+ROcp54_98*s->dpt[3][10];
  OMcp54_110 = OMcp54_19+qd[10]*ROcp54_49;
  OMcp54_210 = OMcp54_29+qd[10]*ROcp54_59;
  OMcp54_310 = OMcp54_39+qd[10]*ROcp54_69;
  ORcp54_110 = OMcp54_29*RLcp54_310-OMcp54_39*RLcp54_210;
  ORcp54_210 = -(OMcp54_19*RLcp54_310-OMcp54_39*RLcp54_110);
  ORcp54_310 = OMcp54_19*RLcp54_210-OMcp54_29*RLcp54_110;
  OPcp54_110 = OPcp54_19+qd[10]*(OMcp54_29*ROcp54_69-OMcp54_39*ROcp54_59)+qdd[10]*ROcp54_49;
  OPcp54_210 = OPcp54_29-qd[10]*(OMcp54_19*ROcp54_69-OMcp54_39*ROcp54_49)+qdd[10]*ROcp54_59;
  OPcp54_310 = OPcp54_39+qd[10]*(OMcp54_19*ROcp54_59-OMcp54_29*ROcp54_49)+qdd[10]*ROcp54_69;
  RLcp54_111 = s->dpt[1][12]*ROcp54_110+s->dpt[2][12]*ROcp54_49+ROcp54_710*s->dpt[3][12];
  RLcp54_211 = s->dpt[1][12]*ROcp54_210+s->dpt[2][12]*ROcp54_59+ROcp54_810*s->dpt[3][12];
  RLcp54_311 = s->dpt[1][12]*ROcp54_310+s->dpt[2][12]*ROcp54_69+ROcp54_910*s->dpt[3][12];
  OMcp54_111 = OMcp54_110+qd[11]*ROcp54_110;
  OMcp54_211 = OMcp54_210+qd[11]*ROcp54_210;
  OMcp54_311 = OMcp54_310+qd[11]*ROcp54_310;
  ORcp54_111 = OMcp54_210*RLcp54_311-OMcp54_310*RLcp54_211;
  ORcp54_211 = -(OMcp54_110*RLcp54_311-OMcp54_310*RLcp54_111);
  ORcp54_311 = OMcp54_110*RLcp54_211-OMcp54_210*RLcp54_111;
  OPcp54_111 = OPcp54_110+qd[11]*(OMcp54_210*ROcp54_310-OMcp54_310*ROcp54_210)+qdd[11]*ROcp54_110;
  OPcp54_211 = OPcp54_210-qd[11]*(OMcp54_110*ROcp54_310-OMcp54_310*ROcp54_110)+qdd[11]*ROcp54_210;
  OPcp54_311 = OPcp54_310+qd[11]*(OMcp54_110*ROcp54_210-OMcp54_210*ROcp54_110)+qdd[11]*ROcp54_310;
  RLcp54_112 = s->dpt[1][14]*ROcp54_110+s->dpt[2][14]*ROcp54_411+s->dpt[3][14]*ROcp54_711;
  RLcp54_212 = s->dpt[1][14]*ROcp54_210+s->dpt[2][14]*ROcp54_511+s->dpt[3][14]*ROcp54_811;
  RLcp54_312 = s->dpt[1][14]*ROcp54_310+s->dpt[2][14]*ROcp54_611+s->dpt[3][14]*ROcp54_911;
  OMcp54_112 = OMcp54_111+qd[12]*ROcp54_411;
  OMcp54_212 = OMcp54_211+qd[12]*ROcp54_511;
  OMcp54_312 = OMcp54_311+qd[12]*ROcp54_611;
  ORcp54_112 = OMcp54_211*RLcp54_312-OMcp54_311*RLcp54_212;
  ORcp54_212 = -(OMcp54_111*RLcp54_312-OMcp54_311*RLcp54_112);
  ORcp54_312 = OMcp54_111*RLcp54_212-OMcp54_211*RLcp54_112;
  OPcp54_112 = OPcp54_111+qd[12]*(OMcp54_211*ROcp54_611-OMcp54_311*ROcp54_511)+qdd[12]*ROcp54_411;
  OPcp54_212 = OPcp54_211-qd[12]*(OMcp54_111*ROcp54_611-OMcp54_311*ROcp54_411)+qdd[12]*ROcp54_511;
  OPcp54_312 = OPcp54_311+qd[12]*(OMcp54_111*ROcp54_511-OMcp54_211*ROcp54_411)+qdd[12]*ROcp54_611;
  RLcp54_190 = ROcp54_112*s->dpt[1][20]+ROcp54_411*s->dpt[2][20]+ROcp54_712*s->dpt[3][20];
  RLcp54_290 = ROcp54_212*s->dpt[1][20]+ROcp54_511*s->dpt[2][20]+ROcp54_812*s->dpt[3][20];
  RLcp54_390 = ROcp54_312*s->dpt[1][20]+ROcp54_611*s->dpt[2][20]+ROcp54_912*s->dpt[3][20];
  ORcp54_190 = OMcp54_212*RLcp54_390-OMcp54_312*RLcp54_290;
  ORcp54_290 = -(OMcp54_112*RLcp54_390-OMcp54_312*RLcp54_190);
  ORcp54_390 = OMcp54_112*RLcp54_290-OMcp54_212*RLcp54_190;
  PxF4[1] = q[1]+RLcp54_110+RLcp54_111+RLcp54_112+RLcp54_17+RLcp54_18+RLcp54_19+RLcp54_190;
  PxF4[2] = q[2]+RLcp54_210+RLcp54_211+RLcp54_212+RLcp54_27+RLcp54_28+RLcp54_29+RLcp54_290;
  PxF4[3] = q[3]+RLcp54_310+RLcp54_311+RLcp54_312+RLcp54_37+RLcp54_38+RLcp54_39+RLcp54_390;
  RxF4[1][1] = ROcp54_112;
  RxF4[1][2] = ROcp54_212;
  RxF4[1][3] = ROcp54_312;
  RxF4[2][1] = ROcp54_411;
  RxF4[2][2] = ROcp54_511;
  RxF4[2][3] = ROcp54_611;
  RxF4[3][1] = ROcp54_712;
  RxF4[3][2] = ROcp54_812;
  RxF4[3][3] = ROcp54_912;
  VxF4[1] = qd[1]+ORcp54_110+ORcp54_111+ORcp54_112+ORcp54_17+ORcp54_18+ORcp54_19+ORcp54_190;
  VxF4[2] = qd[2]+ORcp54_210+ORcp54_211+ORcp54_212+ORcp54_27+ORcp54_28+ORcp54_29+ORcp54_290;
  VxF4[3] = qd[3]+ORcp54_310+ORcp54_311+ORcp54_312+ORcp54_37+ORcp54_38+ORcp54_39+ORcp54_390;
  OMxF4[1] = OMcp54_112;
  OMxF4[2] = OMcp54_212;
  OMxF4[3] = OMcp54_312;
  AxF4[1] = qdd[1]+OMcp54_210*ORcp54_311+OMcp54_211*ORcp54_312+OMcp54_212*ORcp54_390+OMcp54_26*ORcp54_37+OMcp54_27*
 ORcp54_38+OMcp54_28*ORcp54_39+OMcp54_29*ORcp54_310-OMcp54_310*ORcp54_211-OMcp54_311*ORcp54_212-OMcp54_312*ORcp54_290-
 OMcp54_36*ORcp54_27-OMcp54_37*ORcp54_28-OMcp54_38*ORcp54_29-OMcp54_39*ORcp54_210+OPcp54_210*RLcp54_311+OPcp54_211*RLcp54_312
 +OPcp54_212*RLcp54_390+OPcp54_26*RLcp54_37+OPcp54_27*RLcp54_38+OPcp54_28*RLcp54_39+OPcp54_29*RLcp54_310-OPcp54_310*
 RLcp54_211-OPcp54_311*RLcp54_212-OPcp54_312*RLcp54_290-OPcp54_36*RLcp54_27-OPcp54_37*RLcp54_28-OPcp54_38*RLcp54_29-OPcp54_39
 *RLcp54_210;
  AxF4[2] = qdd[2]-OMcp54_110*ORcp54_311-OMcp54_111*ORcp54_312-OMcp54_112*ORcp54_390-OMcp54_16*ORcp54_37-OMcp54_17*
 ORcp54_38-OMcp54_18*ORcp54_39-OMcp54_19*ORcp54_310+OMcp54_310*ORcp54_111+OMcp54_311*ORcp54_112+OMcp54_312*ORcp54_190+
 OMcp54_36*ORcp54_17+OMcp54_37*ORcp54_18+OMcp54_38*ORcp54_19+OMcp54_39*ORcp54_110-OPcp54_110*RLcp54_311-OPcp54_111*RLcp54_312
 -OPcp54_112*RLcp54_390-OPcp54_16*RLcp54_37-OPcp54_17*RLcp54_38-OPcp54_18*RLcp54_39-OPcp54_19*RLcp54_310+OPcp54_310*
 RLcp54_111+OPcp54_311*RLcp54_112+OPcp54_312*RLcp54_190+OPcp54_36*RLcp54_17+OPcp54_37*RLcp54_18+OPcp54_38*RLcp54_19+OPcp54_39
 *RLcp54_110;
  AxF4[3] = qdd[3]+OMcp54_110*ORcp54_211+OMcp54_111*ORcp54_212+OMcp54_112*ORcp54_290+OMcp54_16*ORcp54_27+OMcp54_17*
 ORcp54_28+OMcp54_18*ORcp54_29+OMcp54_19*ORcp54_210-OMcp54_210*ORcp54_111-OMcp54_211*ORcp54_112-OMcp54_212*ORcp54_190-
 OMcp54_26*ORcp54_17-OMcp54_27*ORcp54_18-OMcp54_28*ORcp54_19-OMcp54_29*ORcp54_110+OPcp54_110*RLcp54_211+OPcp54_111*RLcp54_212
 +OPcp54_112*RLcp54_290+OPcp54_16*RLcp54_27+OPcp54_17*RLcp54_28+OPcp54_18*RLcp54_29+OPcp54_19*RLcp54_210-OPcp54_210*
 RLcp54_111-OPcp54_211*RLcp54_112-OPcp54_212*RLcp54_190-OPcp54_26*RLcp54_17-OPcp54_27*RLcp54_18-OPcp54_28*RLcp54_19-OPcp54_29
 *RLcp54_110;
  OMPxF4[1] = OPcp54_112;
  OMPxF4[2] = OPcp54_212;
  OMPxF4[3] = OPcp54_312;
 
// Sensor Forces Computation 

  SWr4 = user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc155 = ROcp54_112*SWr4[1]+ROcp54_212*SWr4[2]+ROcp54_312*SWr4[3];
  xfrc255 = ROcp54_411*SWr4[1]+ROcp54_511*SWr4[2]+ROcp54_611*SWr4[3];
  xfrc355 = ROcp54_712*SWr4[1]+ROcp54_812*SWr4[2]+ROcp54_912*SWr4[3];
  s->frc[1][12] = s->frc[1][12]+xfrc155;
  s->frc[2][12] = s->frc[2][12]+xfrc255;
  s->frc[3][12] = s->frc[3][12]+xfrc355;
  xtrq155 = ROcp54_112*SWr4[4]+ROcp54_212*SWr4[5]+ROcp54_312*SWr4[6];
  xtrq255 = ROcp54_411*SWr4[4]+ROcp54_511*SWr4[5]+ROcp54_611*SWr4[6];
  xtrq355 = ROcp54_712*SWr4[4]+ROcp54_812*SWr4[5]+ROcp54_912*SWr4[6];
  s->trq[1][12] = s->trq[1][12]+xtrq155-xfrc255*(SWr4[9]-s->l[3][12])+xfrc355*(SWr4[8]-s->l[2][12]);
  s->trq[2][12] = s->trq[2][12]+xtrq255+xfrc155*(SWr4[9]-s->l[3][12])-xfrc355*(SWr4[7]-s->l[1][12]);
  s->trq[3][12] = s->trq[3][12]+xtrq355-xfrc155*(SWr4[8]-s->l[2][12])+xfrc255*(SWr4[7]-s->l[1][12]);

// = = Block_0_0_1_5_0_1 = = 
 
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
  OMcp55_26 = OMcp55_25+qd[6]*ROcp55_85;
  OMcp55_36 = OMcp55_35+qd[6]*ROcp55_95;
  OPcp55_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp55_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp55_95-OMcp55_35*S5)-qdd[5]*C4-qdd[6]*ROcp55_85);
  OPcp55_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp55_85-OMcp55_25*S5)+qdd[5]*S4+qdd[6]*ROcp55_95;

// = = Block_0_0_1_5_0_2 = = 
 
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
  OMcp55_17 = OMcp55_16+qd[7]*ROcp55_46;
  OMcp55_27 = OMcp55_26+qd[7]*ROcp55_56;
  OMcp55_37 = OMcp55_36+qd[7]*ROcp55_66;
  ORcp55_17 = OMcp55_26*RLcp55_37-OMcp55_36*RLcp55_27;
  ORcp55_27 = -(OMcp55_16*RLcp55_37-OMcp55_36*RLcp55_17);
  ORcp55_37 = OMcp55_16*RLcp55_27-OMcp55_26*RLcp55_17;
  OPcp55_17 = OPcp55_16+qd[7]*(OMcp55_26*ROcp55_66-OMcp55_36*ROcp55_56)+qdd[7]*ROcp55_46;
  OPcp55_27 = OPcp55_26-qd[7]*(OMcp55_16*ROcp55_66-OMcp55_36*ROcp55_46)+qdd[7]*ROcp55_56;
  OPcp55_37 = OPcp55_36+qd[7]*(OMcp55_16*ROcp55_56-OMcp55_26*ROcp55_46)+qdd[7]*ROcp55_66;
  RLcp55_18 = s->dpt[1][6]*ROcp55_17+s->dpt[3][6]*ROcp55_77+ROcp55_46*s->dpt[2][6];
  RLcp55_28 = s->dpt[1][6]*ROcp55_27+s->dpt[3][6]*ROcp55_87+ROcp55_56*s->dpt[2][6];
  RLcp55_38 = s->dpt[1][6]*ROcp55_37+s->dpt[3][6]*ROcp55_97+ROcp55_66*s->dpt[2][6];
  OMcp55_18 = OMcp55_17+qd[8]*ROcp55_17;
  OMcp55_28 = OMcp55_27+qd[8]*ROcp55_27;
  OMcp55_38 = OMcp55_37+qd[8]*ROcp55_37;
  ORcp55_18 = OMcp55_27*RLcp55_38-OMcp55_37*RLcp55_28;
  ORcp55_28 = -(OMcp55_17*RLcp55_38-OMcp55_37*RLcp55_18);
  ORcp55_38 = OMcp55_17*RLcp55_28-OMcp55_27*RLcp55_18;
  OPcp55_18 = OPcp55_17+qd[8]*(OMcp55_27*ROcp55_37-OMcp55_37*ROcp55_27)+qdd[8]*ROcp55_17;
  OPcp55_28 = OPcp55_27-qd[8]*(OMcp55_17*ROcp55_37-OMcp55_37*ROcp55_17)+qdd[8]*ROcp55_27;
  OPcp55_38 = OPcp55_37+qd[8]*(OMcp55_17*ROcp55_27-OMcp55_27*ROcp55_17)+qdd[8]*ROcp55_37;
  RLcp55_19 = s->dpt[1][8]*ROcp55_17+s->dpt[2][8]*ROcp55_48+ROcp55_78*s->dpt[3][8];
  RLcp55_29 = s->dpt[1][8]*ROcp55_27+s->dpt[2][8]*ROcp55_58+ROcp55_88*s->dpt[3][8];
  RLcp55_39 = s->dpt[1][8]*ROcp55_37+s->dpt[2][8]*ROcp55_68+ROcp55_98*s->dpt[3][8];
  OMcp55_19 = OMcp55_18+qd[9]*ROcp55_78;
  OMcp55_29 = OMcp55_28+qd[9]*ROcp55_88;
  OMcp55_39 = OMcp55_38+qd[9]*ROcp55_98;
  ORcp55_19 = OMcp55_28*RLcp55_39-OMcp55_38*RLcp55_29;
  ORcp55_29 = -(OMcp55_18*RLcp55_39-OMcp55_38*RLcp55_19);
  ORcp55_39 = OMcp55_18*RLcp55_29-OMcp55_28*RLcp55_19;
  OPcp55_19 = OPcp55_18+qd[9]*(OMcp55_28*ROcp55_98-OMcp55_38*ROcp55_88)+qdd[9]*ROcp55_78;
  OPcp55_29 = OPcp55_28-qd[9]*(OMcp55_18*ROcp55_98-OMcp55_38*ROcp55_78)+qdd[9]*ROcp55_88;
  OPcp55_39 = OPcp55_38+qd[9]*(OMcp55_18*ROcp55_88-OMcp55_28*ROcp55_78)+qdd[9]*ROcp55_98;
  RLcp55_110 = s->dpt[1][10]*ROcp55_19+s->dpt[2][10]*ROcp55_49+ROcp55_78*s->dpt[3][10];
  RLcp55_210 = s->dpt[1][10]*ROcp55_29+s->dpt[2][10]*ROcp55_59+ROcp55_88*s->dpt[3][10];
  RLcp55_310 = s->dpt[1][10]*ROcp55_39+s->dpt[2][10]*ROcp55_69+ROcp55_98*s->dpt[3][10];
  OMcp55_110 = OMcp55_19+qd[10]*ROcp55_49;
  OMcp55_210 = OMcp55_29+qd[10]*ROcp55_59;
  OMcp55_310 = OMcp55_39+qd[10]*ROcp55_69;
  ORcp55_110 = OMcp55_29*RLcp55_310-OMcp55_39*RLcp55_210;
  ORcp55_210 = -(OMcp55_19*RLcp55_310-OMcp55_39*RLcp55_110);
  ORcp55_310 = OMcp55_19*RLcp55_210-OMcp55_29*RLcp55_110;
  OPcp55_110 = OPcp55_19+qd[10]*(OMcp55_29*ROcp55_69-OMcp55_39*ROcp55_59)+qdd[10]*ROcp55_49;
  OPcp55_210 = OPcp55_29-qd[10]*(OMcp55_19*ROcp55_69-OMcp55_39*ROcp55_49)+qdd[10]*ROcp55_59;
  OPcp55_310 = OPcp55_39+qd[10]*(OMcp55_19*ROcp55_59-OMcp55_29*ROcp55_49)+qdd[10]*ROcp55_69;
  RLcp55_111 = s->dpt[1][12]*ROcp55_110+s->dpt[2][12]*ROcp55_49+ROcp55_710*s->dpt[3][12];
  RLcp55_211 = s->dpt[1][12]*ROcp55_210+s->dpt[2][12]*ROcp55_59+ROcp55_810*s->dpt[3][12];
  RLcp55_311 = s->dpt[1][12]*ROcp55_310+s->dpt[2][12]*ROcp55_69+ROcp55_910*s->dpt[3][12];
  OMcp55_111 = OMcp55_110+qd[11]*ROcp55_110;
  OMcp55_211 = OMcp55_210+qd[11]*ROcp55_210;
  OMcp55_311 = OMcp55_310+qd[11]*ROcp55_310;
  ORcp55_111 = OMcp55_210*RLcp55_311-OMcp55_310*RLcp55_211;
  ORcp55_211 = -(OMcp55_110*RLcp55_311-OMcp55_310*RLcp55_111);
  ORcp55_311 = OMcp55_110*RLcp55_211-OMcp55_210*RLcp55_111;
  OPcp55_111 = OPcp55_110+qd[11]*(OMcp55_210*ROcp55_310-OMcp55_310*ROcp55_210)+qdd[11]*ROcp55_110;
  OPcp55_211 = OPcp55_210-qd[11]*(OMcp55_110*ROcp55_310-OMcp55_310*ROcp55_110)+qdd[11]*ROcp55_210;
  OPcp55_311 = OPcp55_310+qd[11]*(OMcp55_110*ROcp55_210-OMcp55_210*ROcp55_110)+qdd[11]*ROcp55_310;
  RLcp55_112 = s->dpt[1][14]*ROcp55_110+s->dpt[2][14]*ROcp55_411+s->dpt[3][14]*ROcp55_711;
  RLcp55_212 = s->dpt[1][14]*ROcp55_210+s->dpt[2][14]*ROcp55_511+s->dpt[3][14]*ROcp55_811;
  RLcp55_312 = s->dpt[1][14]*ROcp55_310+s->dpt[2][14]*ROcp55_611+s->dpt[3][14]*ROcp55_911;
  OMcp55_112 = OMcp55_111+qd[12]*ROcp55_411;
  OMcp55_212 = OMcp55_211+qd[12]*ROcp55_511;
  OMcp55_312 = OMcp55_311+qd[12]*ROcp55_611;
  ORcp55_112 = OMcp55_211*RLcp55_312-OMcp55_311*RLcp55_212;
  ORcp55_212 = -(OMcp55_111*RLcp55_312-OMcp55_311*RLcp55_112);
  ORcp55_312 = OMcp55_111*RLcp55_212-OMcp55_211*RLcp55_112;
  OPcp55_112 = OPcp55_111+qd[12]*(OMcp55_211*ROcp55_611-OMcp55_311*ROcp55_511)+qdd[12]*ROcp55_411;
  OPcp55_212 = OPcp55_211-qd[12]*(OMcp55_111*ROcp55_611-OMcp55_311*ROcp55_411)+qdd[12]*ROcp55_511;
  OPcp55_312 = OPcp55_311+qd[12]*(OMcp55_111*ROcp55_511-OMcp55_211*ROcp55_411)+qdd[12]*ROcp55_611;
  RLcp55_191 = ROcp55_112*s->dpt[1][21]+ROcp55_411*s->dpt[2][21]+ROcp55_712*s->dpt[3][21];
  RLcp55_291 = ROcp55_212*s->dpt[1][21]+ROcp55_511*s->dpt[2][21]+ROcp55_812*s->dpt[3][21];
  RLcp55_391 = ROcp55_312*s->dpt[1][21]+ROcp55_611*s->dpt[2][21]+ROcp55_912*s->dpt[3][21];
  ORcp55_191 = OMcp55_212*RLcp55_391-OMcp55_312*RLcp55_291;
  ORcp55_291 = -(OMcp55_112*RLcp55_391-OMcp55_312*RLcp55_191);
  ORcp55_391 = OMcp55_112*RLcp55_291-OMcp55_212*RLcp55_191;
  PxF5[1] = q[1]+RLcp55_110+RLcp55_111+RLcp55_112+RLcp55_17+RLcp55_18+RLcp55_19+RLcp55_191;
  PxF5[2] = q[2]+RLcp55_210+RLcp55_211+RLcp55_212+RLcp55_27+RLcp55_28+RLcp55_29+RLcp55_291;
  PxF5[3] = q[3]+RLcp55_310+RLcp55_311+RLcp55_312+RLcp55_37+RLcp55_38+RLcp55_39+RLcp55_391;
  RxF5[1][1] = ROcp55_112;
  RxF5[1][2] = ROcp55_212;
  RxF5[1][3] = ROcp55_312;
  RxF5[2][1] = ROcp55_411;
  RxF5[2][2] = ROcp55_511;
  RxF5[2][3] = ROcp55_611;
  RxF5[3][1] = ROcp55_712;
  RxF5[3][2] = ROcp55_812;
  RxF5[3][3] = ROcp55_912;
  VxF5[1] = qd[1]+ORcp55_110+ORcp55_111+ORcp55_112+ORcp55_17+ORcp55_18+ORcp55_19+ORcp55_191;
  VxF5[2] = qd[2]+ORcp55_210+ORcp55_211+ORcp55_212+ORcp55_27+ORcp55_28+ORcp55_29+ORcp55_291;
  VxF5[3] = qd[3]+ORcp55_310+ORcp55_311+ORcp55_312+ORcp55_37+ORcp55_38+ORcp55_39+ORcp55_391;
  OMxF5[1] = OMcp55_112;
  OMxF5[2] = OMcp55_212;
  OMxF5[3] = OMcp55_312;
  AxF5[1] = qdd[1]+OMcp55_210*ORcp55_311+OMcp55_211*ORcp55_312+OMcp55_212*ORcp55_391+OMcp55_26*ORcp55_37+OMcp55_27*
 ORcp55_38+OMcp55_28*ORcp55_39+OMcp55_29*ORcp55_310-OMcp55_310*ORcp55_211-OMcp55_311*ORcp55_212-OMcp55_312*ORcp55_291-
 OMcp55_36*ORcp55_27-OMcp55_37*ORcp55_28-OMcp55_38*ORcp55_29-OMcp55_39*ORcp55_210+OPcp55_210*RLcp55_311+OPcp55_211*RLcp55_312
 +OPcp55_212*RLcp55_391+OPcp55_26*RLcp55_37+OPcp55_27*RLcp55_38+OPcp55_28*RLcp55_39+OPcp55_29*RLcp55_310-OPcp55_310*
 RLcp55_211-OPcp55_311*RLcp55_212-OPcp55_312*RLcp55_291-OPcp55_36*RLcp55_27-OPcp55_37*RLcp55_28-OPcp55_38*RLcp55_29-OPcp55_39
 *RLcp55_210;
  AxF5[2] = qdd[2]-OMcp55_110*ORcp55_311-OMcp55_111*ORcp55_312-OMcp55_112*ORcp55_391-OMcp55_16*ORcp55_37-OMcp55_17*
 ORcp55_38-OMcp55_18*ORcp55_39-OMcp55_19*ORcp55_310+OMcp55_310*ORcp55_111+OMcp55_311*ORcp55_112+OMcp55_312*ORcp55_191+
 OMcp55_36*ORcp55_17+OMcp55_37*ORcp55_18+OMcp55_38*ORcp55_19+OMcp55_39*ORcp55_110-OPcp55_110*RLcp55_311-OPcp55_111*RLcp55_312
 -OPcp55_112*RLcp55_391-OPcp55_16*RLcp55_37-OPcp55_17*RLcp55_38-OPcp55_18*RLcp55_39-OPcp55_19*RLcp55_310+OPcp55_310*
 RLcp55_111+OPcp55_311*RLcp55_112+OPcp55_312*RLcp55_191+OPcp55_36*RLcp55_17+OPcp55_37*RLcp55_18+OPcp55_38*RLcp55_19+OPcp55_39
 *RLcp55_110;
  AxF5[3] = qdd[3]+OMcp55_110*ORcp55_211+OMcp55_111*ORcp55_212+OMcp55_112*ORcp55_291+OMcp55_16*ORcp55_27+OMcp55_17*
 ORcp55_28+OMcp55_18*ORcp55_29+OMcp55_19*ORcp55_210-OMcp55_210*ORcp55_111-OMcp55_211*ORcp55_112-OMcp55_212*ORcp55_191-
 OMcp55_26*ORcp55_17-OMcp55_27*ORcp55_18-OMcp55_28*ORcp55_19-OMcp55_29*ORcp55_110+OPcp55_110*RLcp55_211+OPcp55_111*RLcp55_212
 +OPcp55_112*RLcp55_291+OPcp55_16*RLcp55_27+OPcp55_17*RLcp55_28+OPcp55_18*RLcp55_29+OPcp55_19*RLcp55_210-OPcp55_210*
 RLcp55_111-OPcp55_211*RLcp55_112-OPcp55_212*RLcp55_191-OPcp55_26*RLcp55_17-OPcp55_27*RLcp55_18-OPcp55_28*RLcp55_19-OPcp55_29
 *RLcp55_110;
  OMPxF5[1] = OPcp55_112;
  OMPxF5[2] = OPcp55_212;
  OMPxF5[3] = OPcp55_312;
 
// Sensor Forces Computation 

  SWr5 = user_ExtForces(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc156 = ROcp55_112*SWr5[1]+ROcp55_212*SWr5[2]+ROcp55_312*SWr5[3];
  xfrc256 = ROcp55_411*SWr5[1]+ROcp55_511*SWr5[2]+ROcp55_611*SWr5[3];
  xfrc356 = ROcp55_712*SWr5[1]+ROcp55_812*SWr5[2]+ROcp55_912*SWr5[3];
  frc[1][12] = s->frc[1][12]+xfrc156;
  frc[2][12] = s->frc[2][12]+xfrc256;
  frc[3][12] = s->frc[3][12]+xfrc356;
  xtrq156 = ROcp55_112*SWr5[4]+ROcp55_212*SWr5[5]+ROcp55_312*SWr5[6];
  xtrq256 = ROcp55_411*SWr5[4]+ROcp55_511*SWr5[5]+ROcp55_611*SWr5[6];
  xtrq356 = ROcp55_712*SWr5[4]+ROcp55_812*SWr5[5]+ROcp55_912*SWr5[6];
  trq[1][12] = s->trq[1][12]+xtrq156-xfrc256*(SWr5[9]-s->l[3][12])+xfrc356*(SWr5[8]-s->l[2][12]);
  trq[2][12] = s->trq[2][12]+xtrq256+xfrc156*(SWr5[9]-s->l[3][12])-xfrc356*(SWr5[7]-s->l[1][12]);
  trq[3][12] = s->trq[3][12]+xtrq356-xfrc156*(SWr5[8]-s->l[2][12])+xfrc256*(SWr5[7]-s->l[1][12]);

// = = Block_0_0_1_6_0_1 = = 
 
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
  OMcp56_26 = OMcp56_25+qd[6]*ROcp56_85;
  OMcp56_36 = OMcp56_35+qd[6]*ROcp56_95;
  OPcp56_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp56_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp56_95-OMcp56_35*S5)-qdd[5]*C4-qdd[6]*ROcp56_85);
  OPcp56_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp56_85-OMcp56_25*S5)+qdd[5]*S4+qdd[6]*ROcp56_95;

// = = Block_0_0_1_6_0_3 = = 
 
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
  OMcp56_113 = OMcp56_16+qd[13]*ROcp56_46;
  OMcp56_213 = OMcp56_26+qd[13]*ROcp56_56;
  OMcp56_313 = OMcp56_36+qd[13]*ROcp56_66;
  ORcp56_113 = OMcp56_26*RLcp56_313-OMcp56_36*RLcp56_213;
  ORcp56_213 = -(OMcp56_16*RLcp56_313-OMcp56_36*RLcp56_113);
  ORcp56_313 = OMcp56_16*RLcp56_213-OMcp56_26*RLcp56_113;
  OPcp56_113 = OPcp56_16+qd[13]*(OMcp56_26*ROcp56_66-OMcp56_36*ROcp56_56)+qdd[13]*ROcp56_46;
  OPcp56_213 = OPcp56_26-qd[13]*(OMcp56_16*ROcp56_66-OMcp56_36*ROcp56_46)+qdd[13]*ROcp56_56;
  OPcp56_313 = OPcp56_36+qd[13]*(OMcp56_16*ROcp56_56-OMcp56_26*ROcp56_46)+qdd[13]*ROcp56_66;
  RLcp56_114 = s->dpt[1][22]*ROcp56_113+s->dpt[3][22]*ROcp56_713+ROcp56_46*s->dpt[2][22];
  RLcp56_214 = s->dpt[1][22]*ROcp56_213+s->dpt[3][22]*ROcp56_813+ROcp56_56*s->dpt[2][22];
  RLcp56_314 = s->dpt[1][22]*ROcp56_313+s->dpt[3][22]*ROcp56_913+ROcp56_66*s->dpt[2][22];
  OMcp56_114 = OMcp56_113+qd[14]*ROcp56_113;
  OMcp56_214 = OMcp56_213+qd[14]*ROcp56_213;
  OMcp56_314 = OMcp56_313+qd[14]*ROcp56_313;
  ORcp56_114 = OMcp56_213*RLcp56_314-OMcp56_313*RLcp56_214;
  ORcp56_214 = -(OMcp56_113*RLcp56_314-OMcp56_313*RLcp56_114);
  ORcp56_314 = OMcp56_113*RLcp56_214-OMcp56_213*RLcp56_114;
  OPcp56_114 = OPcp56_113+qd[14]*(OMcp56_213*ROcp56_313-OMcp56_313*ROcp56_213)+qdd[14]*ROcp56_113;
  OPcp56_214 = OPcp56_213-qd[14]*(OMcp56_113*ROcp56_313-OMcp56_313*ROcp56_113)+qdd[14]*ROcp56_213;
  OPcp56_314 = OPcp56_313+qd[14]*(OMcp56_113*ROcp56_213-OMcp56_213*ROcp56_113)+qdd[14]*ROcp56_313;
  RLcp56_115 = s->dpt[1][24]*ROcp56_113+s->dpt[2][24]*ROcp56_414+ROcp56_714*s->dpt[3][24];
  RLcp56_215 = s->dpt[1][24]*ROcp56_213+s->dpt[2][24]*ROcp56_514+ROcp56_814*s->dpt[3][24];
  RLcp56_315 = s->dpt[1][24]*ROcp56_313+s->dpt[2][24]*ROcp56_614+ROcp56_914*s->dpt[3][24];
  OMcp56_115 = OMcp56_114+qd[15]*ROcp56_714;
  OMcp56_215 = OMcp56_214+qd[15]*ROcp56_814;
  OMcp56_315 = OMcp56_314+qd[15]*ROcp56_914;
  ORcp56_115 = OMcp56_214*RLcp56_315-OMcp56_314*RLcp56_215;
  ORcp56_215 = -(OMcp56_114*RLcp56_315-OMcp56_314*RLcp56_115);
  ORcp56_315 = OMcp56_114*RLcp56_215-OMcp56_214*RLcp56_115;
  OPcp56_115 = OPcp56_114+qd[15]*(OMcp56_214*ROcp56_914-OMcp56_314*ROcp56_814)+qdd[15]*ROcp56_714;
  OPcp56_215 = OPcp56_214-qd[15]*(OMcp56_114*ROcp56_914-OMcp56_314*ROcp56_714)+qdd[15]*ROcp56_814;
  OPcp56_315 = OPcp56_314+qd[15]*(OMcp56_114*ROcp56_814-OMcp56_214*ROcp56_714)+qdd[15]*ROcp56_914;
  RLcp56_116 = s->dpt[1][26]*ROcp56_115+s->dpt[2][26]*ROcp56_415+ROcp56_714*s->dpt[3][26];
  RLcp56_216 = s->dpt[1][26]*ROcp56_215+s->dpt[2][26]*ROcp56_515+ROcp56_814*s->dpt[3][26];
  RLcp56_316 = s->dpt[1][26]*ROcp56_315+s->dpt[2][26]*ROcp56_615+ROcp56_914*s->dpt[3][26];
  OMcp56_116 = OMcp56_115+qd[16]*ROcp56_415;
  OMcp56_216 = OMcp56_215+qd[16]*ROcp56_515;
  OMcp56_316 = OMcp56_315+qd[16]*ROcp56_615;
  ORcp56_116 = OMcp56_215*RLcp56_316-OMcp56_315*RLcp56_216;
  ORcp56_216 = -(OMcp56_115*RLcp56_316-OMcp56_315*RLcp56_116);
  ORcp56_316 = OMcp56_115*RLcp56_216-OMcp56_215*RLcp56_116;
  OPcp56_116 = OPcp56_115+qd[16]*(OMcp56_215*ROcp56_615-OMcp56_315*ROcp56_515)+qdd[16]*ROcp56_415;
  OPcp56_216 = OPcp56_215-qd[16]*(OMcp56_115*ROcp56_615-OMcp56_315*ROcp56_415)+qdd[16]*ROcp56_515;
  OPcp56_316 = OPcp56_315+qd[16]*(OMcp56_115*ROcp56_515-OMcp56_215*ROcp56_415)+qdd[16]*ROcp56_615;
  RLcp56_117 = s->dpt[1][28]*ROcp56_116+s->dpt[2][28]*ROcp56_415+ROcp56_716*s->dpt[3][28];
  RLcp56_217 = s->dpt[1][28]*ROcp56_216+s->dpt[2][28]*ROcp56_515+ROcp56_816*s->dpt[3][28];
  RLcp56_317 = s->dpt[1][28]*ROcp56_316+s->dpt[2][28]*ROcp56_615+ROcp56_916*s->dpt[3][28];
  OMcp56_117 = OMcp56_116+qd[17]*ROcp56_116;
  OMcp56_217 = OMcp56_216+qd[17]*ROcp56_216;
  OMcp56_317 = OMcp56_316+qd[17]*ROcp56_316;
  ORcp56_117 = OMcp56_216*RLcp56_317-OMcp56_316*RLcp56_217;
  ORcp56_217 = -(OMcp56_116*RLcp56_317-OMcp56_316*RLcp56_117);
  ORcp56_317 = OMcp56_116*RLcp56_217-OMcp56_216*RLcp56_117;
  OPcp56_117 = OPcp56_116+qd[17]*(OMcp56_216*ROcp56_316-OMcp56_316*ROcp56_216)+qdd[17]*ROcp56_116;
  OPcp56_217 = OPcp56_216-qd[17]*(OMcp56_116*ROcp56_316-OMcp56_316*ROcp56_116)+qdd[17]*ROcp56_216;
  OPcp56_317 = OPcp56_316+qd[17]*(OMcp56_116*ROcp56_216-OMcp56_216*ROcp56_116)+qdd[17]*ROcp56_316;
  RLcp56_118 = s->dpt[1][30]*ROcp56_116+s->dpt[2][30]*ROcp56_417+s->dpt[3][30]*ROcp56_717;
  RLcp56_218 = s->dpt[1][30]*ROcp56_216+s->dpt[2][30]*ROcp56_517+s->dpt[3][30]*ROcp56_817;
  RLcp56_318 = s->dpt[1][30]*ROcp56_316+s->dpt[2][30]*ROcp56_617+s->dpt[3][30]*ROcp56_917;
  OMcp56_118 = OMcp56_117+qd[18]*ROcp56_417;
  OMcp56_218 = OMcp56_217+qd[18]*ROcp56_517;
  OMcp56_318 = OMcp56_317+qd[18]*ROcp56_617;
  ORcp56_118 = OMcp56_217*RLcp56_318-OMcp56_317*RLcp56_218;
  ORcp56_218 = -(OMcp56_117*RLcp56_318-OMcp56_317*RLcp56_118);
  ORcp56_318 = OMcp56_117*RLcp56_218-OMcp56_217*RLcp56_118;
  OPcp56_118 = OPcp56_117+qd[18]*(OMcp56_217*ROcp56_617-OMcp56_317*ROcp56_517)+qdd[18]*ROcp56_417;
  OPcp56_218 = OPcp56_217-qd[18]*(OMcp56_117*ROcp56_617-OMcp56_317*ROcp56_417)+qdd[18]*ROcp56_517;
  OPcp56_318 = OPcp56_317+qd[18]*(OMcp56_117*ROcp56_517-OMcp56_217*ROcp56_417)+qdd[18]*ROcp56_617;
  RLcp56_192 = s->dpt[1][33]*ROcp56_118+s->dpt[2][33]*ROcp56_417+ROcp56_718*s->dpt[3][33];
  RLcp56_292 = s->dpt[1][33]*ROcp56_218+s->dpt[2][33]*ROcp56_517+ROcp56_818*s->dpt[3][33];
  RLcp56_392 = s->dpt[1][33]*ROcp56_318+s->dpt[2][33]*ROcp56_617+ROcp56_918*s->dpt[3][33];
  ORcp56_192 = OMcp56_218*RLcp56_392-OMcp56_318*RLcp56_292;
  ORcp56_292 = -(OMcp56_118*RLcp56_392-OMcp56_318*RLcp56_192);
  ORcp56_392 = OMcp56_118*RLcp56_292-OMcp56_218*RLcp56_192;
  PxF6[1] = q[1]+RLcp56_113+RLcp56_114+RLcp56_115+RLcp56_116+RLcp56_117+RLcp56_118+RLcp56_192;
  PxF6[2] = q[2]+RLcp56_213+RLcp56_214+RLcp56_215+RLcp56_216+RLcp56_217+RLcp56_218+RLcp56_292;
  PxF6[3] = q[3]+RLcp56_313+RLcp56_314+RLcp56_315+RLcp56_316+RLcp56_317+RLcp56_318+RLcp56_392;
  RxF6[1][1] = ROcp56_118;
  RxF6[1][2] = ROcp56_218;
  RxF6[1][3] = ROcp56_318;
  RxF6[2][1] = ROcp56_417;
  RxF6[2][2] = ROcp56_517;
  RxF6[2][3] = ROcp56_617;
  RxF6[3][1] = ROcp56_718;
  RxF6[3][2] = ROcp56_818;
  RxF6[3][3] = ROcp56_918;
  VxF6[1] = qd[1]+ORcp56_113+ORcp56_114+ORcp56_115+ORcp56_116+ORcp56_117+ORcp56_118+ORcp56_192;
  VxF6[2] = qd[2]+ORcp56_213+ORcp56_214+ORcp56_215+ORcp56_216+ORcp56_217+ORcp56_218+ORcp56_292;
  VxF6[3] = qd[3]+ORcp56_313+ORcp56_314+ORcp56_315+ORcp56_316+ORcp56_317+ORcp56_318+ORcp56_392;
  OMxF6[1] = OMcp56_118;
  OMxF6[2] = OMcp56_218;
  OMxF6[3] = OMcp56_318;
  AxF6[1] = qdd[1]+OMcp56_213*ORcp56_314+OMcp56_214*ORcp56_315+OMcp56_215*ORcp56_316+OMcp56_216*ORcp56_317+OMcp56_217*
 ORcp56_318+OMcp56_218*ORcp56_392+OMcp56_26*ORcp56_313-OMcp56_313*ORcp56_214-OMcp56_314*ORcp56_215-OMcp56_315*ORcp56_216-
 OMcp56_316*ORcp56_217-OMcp56_317*ORcp56_218-OMcp56_318*ORcp56_292-OMcp56_36*ORcp56_213+OPcp56_213*RLcp56_314+OPcp56_214*
 RLcp56_315+OPcp56_215*RLcp56_316+OPcp56_216*RLcp56_317+OPcp56_217*RLcp56_318+OPcp56_218*RLcp56_392+OPcp56_26*RLcp56_313-
 OPcp56_313*RLcp56_214-OPcp56_314*RLcp56_215-OPcp56_315*RLcp56_216-OPcp56_316*RLcp56_217-OPcp56_317*RLcp56_218-OPcp56_318*
 RLcp56_292-OPcp56_36*RLcp56_213;
  AxF6[2] = qdd[2]-OMcp56_113*ORcp56_314-OMcp56_114*ORcp56_315-OMcp56_115*ORcp56_316-OMcp56_116*ORcp56_317-OMcp56_117*
 ORcp56_318-OMcp56_118*ORcp56_392-OMcp56_16*ORcp56_313+OMcp56_313*ORcp56_114+OMcp56_314*ORcp56_115+OMcp56_315*ORcp56_116+
 OMcp56_316*ORcp56_117+OMcp56_317*ORcp56_118+OMcp56_318*ORcp56_192+OMcp56_36*ORcp56_113-OPcp56_113*RLcp56_314-OPcp56_114*
 RLcp56_315-OPcp56_115*RLcp56_316-OPcp56_116*RLcp56_317-OPcp56_117*RLcp56_318-OPcp56_118*RLcp56_392-OPcp56_16*RLcp56_313+
 OPcp56_313*RLcp56_114+OPcp56_314*RLcp56_115+OPcp56_315*RLcp56_116+OPcp56_316*RLcp56_117+OPcp56_317*RLcp56_118+OPcp56_318*
 RLcp56_192+OPcp56_36*RLcp56_113;
  AxF6[3] = qdd[3]+OMcp56_113*ORcp56_214+OMcp56_114*ORcp56_215+OMcp56_115*ORcp56_216+OMcp56_116*ORcp56_217+OMcp56_117*
 ORcp56_218+OMcp56_118*ORcp56_292+OMcp56_16*ORcp56_213-OMcp56_213*ORcp56_114-OMcp56_214*ORcp56_115-OMcp56_215*ORcp56_116-
 OMcp56_216*ORcp56_117-OMcp56_217*ORcp56_118-OMcp56_218*ORcp56_192-OMcp56_26*ORcp56_113+OPcp56_113*RLcp56_214+OPcp56_114*
 RLcp56_215+OPcp56_115*RLcp56_216+OPcp56_116*RLcp56_217+OPcp56_117*RLcp56_218+OPcp56_118*RLcp56_292+OPcp56_16*RLcp56_213-
 OPcp56_213*RLcp56_114-OPcp56_214*RLcp56_115-OPcp56_215*RLcp56_116-OPcp56_216*RLcp56_117-OPcp56_217*RLcp56_118-OPcp56_218*
 RLcp56_192-OPcp56_26*RLcp56_113;
  OMPxF6[1] = OPcp56_118;
  OMPxF6[2] = OPcp56_218;
  OMPxF6[3] = OPcp56_318;
 
// Sensor Forces Computation 

  SWr6 = user_ExtForces(PxF6,RxF6,VxF6,OMxF6,AxF6,OMPxF6,s,tsim,6);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc157 = ROcp56_118*SWr6[1]+ROcp56_218*SWr6[2]+ROcp56_318*SWr6[3];
  xfrc257 = ROcp56_417*SWr6[1]+ROcp56_517*SWr6[2]+ROcp56_617*SWr6[3];
  xfrc357 = ROcp56_718*SWr6[1]+ROcp56_818*SWr6[2]+ROcp56_918*SWr6[3];
  s->frc[1][18] = s->frc[1][18]+xfrc157;
  s->frc[2][18] = s->frc[2][18]+xfrc257;
  s->frc[3][18] = s->frc[3][18]+xfrc357;
  xtrq157 = ROcp56_118*SWr6[4]+ROcp56_218*SWr6[5]+ROcp56_318*SWr6[6];
  xtrq257 = ROcp56_417*SWr6[4]+ROcp56_517*SWr6[5]+ROcp56_617*SWr6[6];
  xtrq357 = ROcp56_718*SWr6[4]+ROcp56_818*SWr6[5]+ROcp56_918*SWr6[6];
  s->trq[1][18] = s->trq[1][18]+xtrq157-xfrc257*(SWr6[9]-s->l[3][18])+xfrc357*(SWr6[8]-s->l[2][18]);
  s->trq[2][18] = s->trq[2][18]+xtrq257+xfrc157*(SWr6[9]-s->l[3][18])-xfrc357*(SWr6[7]-s->l[1][18]);
  s->trq[3][18] = s->trq[3][18]+xtrq357-xfrc157*(SWr6[8]-s->l[2][18])+xfrc257*(SWr6[7]-s->l[1][18]);

// = = Block_0_0_1_7_0_1 = = 
 
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
  OMcp57_26 = OMcp57_25+qd[6]*ROcp57_85;
  OMcp57_36 = OMcp57_35+qd[6]*ROcp57_95;
  OPcp57_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp57_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp57_95-OMcp57_35*S5)-qdd[5]*C4-qdd[6]*ROcp57_85);
  OPcp57_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp57_85-OMcp57_25*S5)+qdd[5]*S4+qdd[6]*ROcp57_95;

// = = Block_0_0_1_7_0_3 = = 
 
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
  OMcp57_113 = OMcp57_16+qd[13]*ROcp57_46;
  OMcp57_213 = OMcp57_26+qd[13]*ROcp57_56;
  OMcp57_313 = OMcp57_36+qd[13]*ROcp57_66;
  ORcp57_113 = OMcp57_26*RLcp57_313-OMcp57_36*RLcp57_213;
  ORcp57_213 = -(OMcp57_16*RLcp57_313-OMcp57_36*RLcp57_113);
  ORcp57_313 = OMcp57_16*RLcp57_213-OMcp57_26*RLcp57_113;
  OPcp57_113 = OPcp57_16+qd[13]*(OMcp57_26*ROcp57_66-OMcp57_36*ROcp57_56)+qdd[13]*ROcp57_46;
  OPcp57_213 = OPcp57_26-qd[13]*(OMcp57_16*ROcp57_66-OMcp57_36*ROcp57_46)+qdd[13]*ROcp57_56;
  OPcp57_313 = OPcp57_36+qd[13]*(OMcp57_16*ROcp57_56-OMcp57_26*ROcp57_46)+qdd[13]*ROcp57_66;
  RLcp57_114 = s->dpt[1][22]*ROcp57_113+s->dpt[3][22]*ROcp57_713+ROcp57_46*s->dpt[2][22];
  RLcp57_214 = s->dpt[1][22]*ROcp57_213+s->dpt[3][22]*ROcp57_813+ROcp57_56*s->dpt[2][22];
  RLcp57_314 = s->dpt[1][22]*ROcp57_313+s->dpt[3][22]*ROcp57_913+ROcp57_66*s->dpt[2][22];
  OMcp57_114 = OMcp57_113+qd[14]*ROcp57_113;
  OMcp57_214 = OMcp57_213+qd[14]*ROcp57_213;
  OMcp57_314 = OMcp57_313+qd[14]*ROcp57_313;
  ORcp57_114 = OMcp57_213*RLcp57_314-OMcp57_313*RLcp57_214;
  ORcp57_214 = -(OMcp57_113*RLcp57_314-OMcp57_313*RLcp57_114);
  ORcp57_314 = OMcp57_113*RLcp57_214-OMcp57_213*RLcp57_114;
  OPcp57_114 = OPcp57_113+qd[14]*(OMcp57_213*ROcp57_313-OMcp57_313*ROcp57_213)+qdd[14]*ROcp57_113;
  OPcp57_214 = OPcp57_213-qd[14]*(OMcp57_113*ROcp57_313-OMcp57_313*ROcp57_113)+qdd[14]*ROcp57_213;
  OPcp57_314 = OPcp57_313+qd[14]*(OMcp57_113*ROcp57_213-OMcp57_213*ROcp57_113)+qdd[14]*ROcp57_313;
  RLcp57_115 = s->dpt[1][24]*ROcp57_113+s->dpt[2][24]*ROcp57_414+ROcp57_714*s->dpt[3][24];
  RLcp57_215 = s->dpt[1][24]*ROcp57_213+s->dpt[2][24]*ROcp57_514+ROcp57_814*s->dpt[3][24];
  RLcp57_315 = s->dpt[1][24]*ROcp57_313+s->dpt[2][24]*ROcp57_614+ROcp57_914*s->dpt[3][24];
  OMcp57_115 = OMcp57_114+qd[15]*ROcp57_714;
  OMcp57_215 = OMcp57_214+qd[15]*ROcp57_814;
  OMcp57_315 = OMcp57_314+qd[15]*ROcp57_914;
  ORcp57_115 = OMcp57_214*RLcp57_315-OMcp57_314*RLcp57_215;
  ORcp57_215 = -(OMcp57_114*RLcp57_315-OMcp57_314*RLcp57_115);
  ORcp57_315 = OMcp57_114*RLcp57_215-OMcp57_214*RLcp57_115;
  OPcp57_115 = OPcp57_114+qd[15]*(OMcp57_214*ROcp57_914-OMcp57_314*ROcp57_814)+qdd[15]*ROcp57_714;
  OPcp57_215 = OPcp57_214-qd[15]*(OMcp57_114*ROcp57_914-OMcp57_314*ROcp57_714)+qdd[15]*ROcp57_814;
  OPcp57_315 = OPcp57_314+qd[15]*(OMcp57_114*ROcp57_814-OMcp57_214*ROcp57_714)+qdd[15]*ROcp57_914;
  RLcp57_116 = s->dpt[1][26]*ROcp57_115+s->dpt[2][26]*ROcp57_415+ROcp57_714*s->dpt[3][26];
  RLcp57_216 = s->dpt[1][26]*ROcp57_215+s->dpt[2][26]*ROcp57_515+ROcp57_814*s->dpt[3][26];
  RLcp57_316 = s->dpt[1][26]*ROcp57_315+s->dpt[2][26]*ROcp57_615+ROcp57_914*s->dpt[3][26];
  OMcp57_116 = OMcp57_115+qd[16]*ROcp57_415;
  OMcp57_216 = OMcp57_215+qd[16]*ROcp57_515;
  OMcp57_316 = OMcp57_315+qd[16]*ROcp57_615;
  ORcp57_116 = OMcp57_215*RLcp57_316-OMcp57_315*RLcp57_216;
  ORcp57_216 = -(OMcp57_115*RLcp57_316-OMcp57_315*RLcp57_116);
  ORcp57_316 = OMcp57_115*RLcp57_216-OMcp57_215*RLcp57_116;
  OPcp57_116 = OPcp57_115+qd[16]*(OMcp57_215*ROcp57_615-OMcp57_315*ROcp57_515)+qdd[16]*ROcp57_415;
  OPcp57_216 = OPcp57_215-qd[16]*(OMcp57_115*ROcp57_615-OMcp57_315*ROcp57_415)+qdd[16]*ROcp57_515;
  OPcp57_316 = OPcp57_315+qd[16]*(OMcp57_115*ROcp57_515-OMcp57_215*ROcp57_415)+qdd[16]*ROcp57_615;
  RLcp57_117 = s->dpt[1][28]*ROcp57_116+s->dpt[2][28]*ROcp57_415+ROcp57_716*s->dpt[3][28];
  RLcp57_217 = s->dpt[1][28]*ROcp57_216+s->dpt[2][28]*ROcp57_515+ROcp57_816*s->dpt[3][28];
  RLcp57_317 = s->dpt[1][28]*ROcp57_316+s->dpt[2][28]*ROcp57_615+ROcp57_916*s->dpt[3][28];
  OMcp57_117 = OMcp57_116+qd[17]*ROcp57_116;
  OMcp57_217 = OMcp57_216+qd[17]*ROcp57_216;
  OMcp57_317 = OMcp57_316+qd[17]*ROcp57_316;
  ORcp57_117 = OMcp57_216*RLcp57_317-OMcp57_316*RLcp57_217;
  ORcp57_217 = -(OMcp57_116*RLcp57_317-OMcp57_316*RLcp57_117);
  ORcp57_317 = OMcp57_116*RLcp57_217-OMcp57_216*RLcp57_117;
  OPcp57_117 = OPcp57_116+qd[17]*(OMcp57_216*ROcp57_316-OMcp57_316*ROcp57_216)+qdd[17]*ROcp57_116;
  OPcp57_217 = OPcp57_216-qd[17]*(OMcp57_116*ROcp57_316-OMcp57_316*ROcp57_116)+qdd[17]*ROcp57_216;
  OPcp57_317 = OPcp57_316+qd[17]*(OMcp57_116*ROcp57_216-OMcp57_216*ROcp57_116)+qdd[17]*ROcp57_316;
  RLcp57_118 = s->dpt[1][30]*ROcp57_116+s->dpt[2][30]*ROcp57_417+s->dpt[3][30]*ROcp57_717;
  RLcp57_218 = s->dpt[1][30]*ROcp57_216+s->dpt[2][30]*ROcp57_517+s->dpt[3][30]*ROcp57_817;
  RLcp57_318 = s->dpt[1][30]*ROcp57_316+s->dpt[2][30]*ROcp57_617+s->dpt[3][30]*ROcp57_917;
  OMcp57_118 = OMcp57_117+qd[18]*ROcp57_417;
  OMcp57_218 = OMcp57_217+qd[18]*ROcp57_517;
  OMcp57_318 = OMcp57_317+qd[18]*ROcp57_617;
  ORcp57_118 = OMcp57_217*RLcp57_318-OMcp57_317*RLcp57_218;
  ORcp57_218 = -(OMcp57_117*RLcp57_318-OMcp57_317*RLcp57_118);
  ORcp57_318 = OMcp57_117*RLcp57_218-OMcp57_217*RLcp57_118;
  OPcp57_118 = OPcp57_117+qd[18]*(OMcp57_217*ROcp57_617-OMcp57_317*ROcp57_517)+qdd[18]*ROcp57_417;
  OPcp57_218 = OPcp57_217-qd[18]*(OMcp57_117*ROcp57_617-OMcp57_317*ROcp57_417)+qdd[18]*ROcp57_517;
  OPcp57_318 = OPcp57_317+qd[18]*(OMcp57_117*ROcp57_517-OMcp57_217*ROcp57_417)+qdd[18]*ROcp57_617;
  RLcp57_193 = ROcp57_118*s->dpt[1][34]+ROcp57_417*s->dpt[2][34]+ROcp57_718*s->dpt[3][34];
  RLcp57_293 = ROcp57_218*s->dpt[1][34]+ROcp57_517*s->dpt[2][34]+ROcp57_818*s->dpt[3][34];
  RLcp57_393 = ROcp57_318*s->dpt[1][34]+ROcp57_617*s->dpt[2][34]+ROcp57_918*s->dpt[3][34];
  ORcp57_193 = OMcp57_218*RLcp57_393-OMcp57_318*RLcp57_293;
  ORcp57_293 = -(OMcp57_118*RLcp57_393-OMcp57_318*RLcp57_193);
  ORcp57_393 = OMcp57_118*RLcp57_293-OMcp57_218*RLcp57_193;
  PxF7[1] = q[1]+RLcp57_113+RLcp57_114+RLcp57_115+RLcp57_116+RLcp57_117+RLcp57_118+RLcp57_193;
  PxF7[2] = q[2]+RLcp57_213+RLcp57_214+RLcp57_215+RLcp57_216+RLcp57_217+RLcp57_218+RLcp57_293;
  PxF7[3] = q[3]+RLcp57_313+RLcp57_314+RLcp57_315+RLcp57_316+RLcp57_317+RLcp57_318+RLcp57_393;
  RxF7[1][1] = ROcp57_118;
  RxF7[1][2] = ROcp57_218;
  RxF7[1][3] = ROcp57_318;
  RxF7[2][1] = ROcp57_417;
  RxF7[2][2] = ROcp57_517;
  RxF7[2][3] = ROcp57_617;
  RxF7[3][1] = ROcp57_718;
  RxF7[3][2] = ROcp57_818;
  RxF7[3][3] = ROcp57_918;
  VxF7[1] = qd[1]+ORcp57_113+ORcp57_114+ORcp57_115+ORcp57_116+ORcp57_117+ORcp57_118+ORcp57_193;
  VxF7[2] = qd[2]+ORcp57_213+ORcp57_214+ORcp57_215+ORcp57_216+ORcp57_217+ORcp57_218+ORcp57_293;
  VxF7[3] = qd[3]+ORcp57_313+ORcp57_314+ORcp57_315+ORcp57_316+ORcp57_317+ORcp57_318+ORcp57_393;
  OMxF7[1] = OMcp57_118;
  OMxF7[2] = OMcp57_218;
  OMxF7[3] = OMcp57_318;
  AxF7[1] = qdd[1]+OMcp57_213*ORcp57_314+OMcp57_214*ORcp57_315+OMcp57_215*ORcp57_316+OMcp57_216*ORcp57_317+OMcp57_217*
 ORcp57_318+OMcp57_218*ORcp57_393+OMcp57_26*ORcp57_313-OMcp57_313*ORcp57_214-OMcp57_314*ORcp57_215-OMcp57_315*ORcp57_216-
 OMcp57_316*ORcp57_217-OMcp57_317*ORcp57_218-OMcp57_318*ORcp57_293-OMcp57_36*ORcp57_213+OPcp57_213*RLcp57_314+OPcp57_214*
 RLcp57_315+OPcp57_215*RLcp57_316+OPcp57_216*RLcp57_317+OPcp57_217*RLcp57_318+OPcp57_218*RLcp57_393+OPcp57_26*RLcp57_313-
 OPcp57_313*RLcp57_214-OPcp57_314*RLcp57_215-OPcp57_315*RLcp57_216-OPcp57_316*RLcp57_217-OPcp57_317*RLcp57_218-OPcp57_318*
 RLcp57_293-OPcp57_36*RLcp57_213;
  AxF7[2] = qdd[2]-OMcp57_113*ORcp57_314-OMcp57_114*ORcp57_315-OMcp57_115*ORcp57_316-OMcp57_116*ORcp57_317-OMcp57_117*
 ORcp57_318-OMcp57_118*ORcp57_393-OMcp57_16*ORcp57_313+OMcp57_313*ORcp57_114+OMcp57_314*ORcp57_115+OMcp57_315*ORcp57_116+
 OMcp57_316*ORcp57_117+OMcp57_317*ORcp57_118+OMcp57_318*ORcp57_193+OMcp57_36*ORcp57_113-OPcp57_113*RLcp57_314-OPcp57_114*
 RLcp57_315-OPcp57_115*RLcp57_316-OPcp57_116*RLcp57_317-OPcp57_117*RLcp57_318-OPcp57_118*RLcp57_393-OPcp57_16*RLcp57_313+
 OPcp57_313*RLcp57_114+OPcp57_314*RLcp57_115+OPcp57_315*RLcp57_116+OPcp57_316*RLcp57_117+OPcp57_317*RLcp57_118+OPcp57_318*
 RLcp57_193+OPcp57_36*RLcp57_113;
  AxF7[3] = qdd[3]+OMcp57_113*ORcp57_214+OMcp57_114*ORcp57_215+OMcp57_115*ORcp57_216+OMcp57_116*ORcp57_217+OMcp57_117*
 ORcp57_218+OMcp57_118*ORcp57_293+OMcp57_16*ORcp57_213-OMcp57_213*ORcp57_114-OMcp57_214*ORcp57_115-OMcp57_215*ORcp57_116-
 OMcp57_216*ORcp57_117-OMcp57_217*ORcp57_118-OMcp57_218*ORcp57_193-OMcp57_26*ORcp57_113+OPcp57_113*RLcp57_214+OPcp57_114*
 RLcp57_215+OPcp57_115*RLcp57_216+OPcp57_116*RLcp57_217+OPcp57_117*RLcp57_218+OPcp57_118*RLcp57_293+OPcp57_16*RLcp57_213-
 OPcp57_213*RLcp57_114-OPcp57_214*RLcp57_115-OPcp57_215*RLcp57_116-OPcp57_216*RLcp57_117-OPcp57_217*RLcp57_118-OPcp57_218*
 RLcp57_193-OPcp57_26*RLcp57_113;
  OMPxF7[1] = OPcp57_118;
  OMPxF7[2] = OPcp57_218;
  OMPxF7[3] = OPcp57_318;
 
// Sensor Forces Computation 

  SWr7 = user_ExtForces(PxF7,RxF7,VxF7,OMxF7,AxF7,OMPxF7,s,tsim,7);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc158 = ROcp57_118*SWr7[1]+ROcp57_218*SWr7[2]+ROcp57_318*SWr7[3];
  xfrc258 = ROcp57_417*SWr7[1]+ROcp57_517*SWr7[2]+ROcp57_617*SWr7[3];
  xfrc358 = ROcp57_718*SWr7[1]+ROcp57_818*SWr7[2]+ROcp57_918*SWr7[3];
  s->frc[1][18] = s->frc[1][18]+xfrc158;
  s->frc[2][18] = s->frc[2][18]+xfrc258;
  s->frc[3][18] = s->frc[3][18]+xfrc358;
  xtrq158 = ROcp57_118*SWr7[4]+ROcp57_218*SWr7[5]+ROcp57_318*SWr7[6];
  xtrq258 = ROcp57_417*SWr7[4]+ROcp57_517*SWr7[5]+ROcp57_617*SWr7[6];
  xtrq358 = ROcp57_718*SWr7[4]+ROcp57_818*SWr7[5]+ROcp57_918*SWr7[6];
  s->trq[1][18] = s->trq[1][18]+xtrq158-xfrc258*(SWr7[9]-s->l[3][18])+xfrc358*(SWr7[8]-s->l[2][18]);
  s->trq[2][18] = s->trq[2][18]+xtrq258+xfrc158*(SWr7[9]-s->l[3][18])-xfrc358*(SWr7[7]-s->l[1][18]);
  s->trq[3][18] = s->trq[3][18]+xtrq358-xfrc158*(SWr7[8]-s->l[2][18])+xfrc258*(SWr7[7]-s->l[1][18]);

// = = Block_0_0_1_8_0_1 = = 
 
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
  OMcp58_26 = OMcp58_25+qd[6]*ROcp58_85;
  OMcp58_36 = OMcp58_35+qd[6]*ROcp58_95;
  OPcp58_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp58_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp58_95-OMcp58_35*S5)-qdd[5]*C4-qdd[6]*ROcp58_85);
  OPcp58_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp58_85-OMcp58_25*S5)+qdd[5]*S4+qdd[6]*ROcp58_95;

// = = Block_0_0_1_8_0_3 = = 
 
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
  OMcp58_113 = OMcp58_16+qd[13]*ROcp58_46;
  OMcp58_213 = OMcp58_26+qd[13]*ROcp58_56;
  OMcp58_313 = OMcp58_36+qd[13]*ROcp58_66;
  ORcp58_113 = OMcp58_26*RLcp58_313-OMcp58_36*RLcp58_213;
  ORcp58_213 = -(OMcp58_16*RLcp58_313-OMcp58_36*RLcp58_113);
  ORcp58_313 = OMcp58_16*RLcp58_213-OMcp58_26*RLcp58_113;
  OPcp58_113 = OPcp58_16+qd[13]*(OMcp58_26*ROcp58_66-OMcp58_36*ROcp58_56)+qdd[13]*ROcp58_46;
  OPcp58_213 = OPcp58_26-qd[13]*(OMcp58_16*ROcp58_66-OMcp58_36*ROcp58_46)+qdd[13]*ROcp58_56;
  OPcp58_313 = OPcp58_36+qd[13]*(OMcp58_16*ROcp58_56-OMcp58_26*ROcp58_46)+qdd[13]*ROcp58_66;
  RLcp58_114 = s->dpt[1][22]*ROcp58_113+s->dpt[3][22]*ROcp58_713+ROcp58_46*s->dpt[2][22];
  RLcp58_214 = s->dpt[1][22]*ROcp58_213+s->dpt[3][22]*ROcp58_813+ROcp58_56*s->dpt[2][22];
  RLcp58_314 = s->dpt[1][22]*ROcp58_313+s->dpt[3][22]*ROcp58_913+ROcp58_66*s->dpt[2][22];
  OMcp58_114 = OMcp58_113+qd[14]*ROcp58_113;
  OMcp58_214 = OMcp58_213+qd[14]*ROcp58_213;
  OMcp58_314 = OMcp58_313+qd[14]*ROcp58_313;
  ORcp58_114 = OMcp58_213*RLcp58_314-OMcp58_313*RLcp58_214;
  ORcp58_214 = -(OMcp58_113*RLcp58_314-OMcp58_313*RLcp58_114);
  ORcp58_314 = OMcp58_113*RLcp58_214-OMcp58_213*RLcp58_114;
  OPcp58_114 = OPcp58_113+qd[14]*(OMcp58_213*ROcp58_313-OMcp58_313*ROcp58_213)+qdd[14]*ROcp58_113;
  OPcp58_214 = OPcp58_213-qd[14]*(OMcp58_113*ROcp58_313-OMcp58_313*ROcp58_113)+qdd[14]*ROcp58_213;
  OPcp58_314 = OPcp58_313+qd[14]*(OMcp58_113*ROcp58_213-OMcp58_213*ROcp58_113)+qdd[14]*ROcp58_313;
  RLcp58_115 = s->dpt[1][24]*ROcp58_113+s->dpt[2][24]*ROcp58_414+ROcp58_714*s->dpt[3][24];
  RLcp58_215 = s->dpt[1][24]*ROcp58_213+s->dpt[2][24]*ROcp58_514+ROcp58_814*s->dpt[3][24];
  RLcp58_315 = s->dpt[1][24]*ROcp58_313+s->dpt[2][24]*ROcp58_614+ROcp58_914*s->dpt[3][24];
  OMcp58_115 = OMcp58_114+qd[15]*ROcp58_714;
  OMcp58_215 = OMcp58_214+qd[15]*ROcp58_814;
  OMcp58_315 = OMcp58_314+qd[15]*ROcp58_914;
  ORcp58_115 = OMcp58_214*RLcp58_315-OMcp58_314*RLcp58_215;
  ORcp58_215 = -(OMcp58_114*RLcp58_315-OMcp58_314*RLcp58_115);
  ORcp58_315 = OMcp58_114*RLcp58_215-OMcp58_214*RLcp58_115;
  OPcp58_115 = OPcp58_114+qd[15]*(OMcp58_214*ROcp58_914-OMcp58_314*ROcp58_814)+qdd[15]*ROcp58_714;
  OPcp58_215 = OPcp58_214-qd[15]*(OMcp58_114*ROcp58_914-OMcp58_314*ROcp58_714)+qdd[15]*ROcp58_814;
  OPcp58_315 = OPcp58_314+qd[15]*(OMcp58_114*ROcp58_814-OMcp58_214*ROcp58_714)+qdd[15]*ROcp58_914;
  RLcp58_116 = s->dpt[1][26]*ROcp58_115+s->dpt[2][26]*ROcp58_415+ROcp58_714*s->dpt[3][26];
  RLcp58_216 = s->dpt[1][26]*ROcp58_215+s->dpt[2][26]*ROcp58_515+ROcp58_814*s->dpt[3][26];
  RLcp58_316 = s->dpt[1][26]*ROcp58_315+s->dpt[2][26]*ROcp58_615+ROcp58_914*s->dpt[3][26];
  OMcp58_116 = OMcp58_115+qd[16]*ROcp58_415;
  OMcp58_216 = OMcp58_215+qd[16]*ROcp58_515;
  OMcp58_316 = OMcp58_315+qd[16]*ROcp58_615;
  ORcp58_116 = OMcp58_215*RLcp58_316-OMcp58_315*RLcp58_216;
  ORcp58_216 = -(OMcp58_115*RLcp58_316-OMcp58_315*RLcp58_116);
  ORcp58_316 = OMcp58_115*RLcp58_216-OMcp58_215*RLcp58_116;
  OPcp58_116 = OPcp58_115+qd[16]*(OMcp58_215*ROcp58_615-OMcp58_315*ROcp58_515)+qdd[16]*ROcp58_415;
  OPcp58_216 = OPcp58_215-qd[16]*(OMcp58_115*ROcp58_615-OMcp58_315*ROcp58_415)+qdd[16]*ROcp58_515;
  OPcp58_316 = OPcp58_315+qd[16]*(OMcp58_115*ROcp58_515-OMcp58_215*ROcp58_415)+qdd[16]*ROcp58_615;
  RLcp58_117 = s->dpt[1][28]*ROcp58_116+s->dpt[2][28]*ROcp58_415+ROcp58_716*s->dpt[3][28];
  RLcp58_217 = s->dpt[1][28]*ROcp58_216+s->dpt[2][28]*ROcp58_515+ROcp58_816*s->dpt[3][28];
  RLcp58_317 = s->dpt[1][28]*ROcp58_316+s->dpt[2][28]*ROcp58_615+ROcp58_916*s->dpt[3][28];
  OMcp58_117 = OMcp58_116+qd[17]*ROcp58_116;
  OMcp58_217 = OMcp58_216+qd[17]*ROcp58_216;
  OMcp58_317 = OMcp58_316+qd[17]*ROcp58_316;
  ORcp58_117 = OMcp58_216*RLcp58_317-OMcp58_316*RLcp58_217;
  ORcp58_217 = -(OMcp58_116*RLcp58_317-OMcp58_316*RLcp58_117);
  ORcp58_317 = OMcp58_116*RLcp58_217-OMcp58_216*RLcp58_117;
  OPcp58_117 = OPcp58_116+qd[17]*(OMcp58_216*ROcp58_316-OMcp58_316*ROcp58_216)+qdd[17]*ROcp58_116;
  OPcp58_217 = OPcp58_216-qd[17]*(OMcp58_116*ROcp58_316-OMcp58_316*ROcp58_116)+qdd[17]*ROcp58_216;
  OPcp58_317 = OPcp58_316+qd[17]*(OMcp58_116*ROcp58_216-OMcp58_216*ROcp58_116)+qdd[17]*ROcp58_316;
  RLcp58_118 = s->dpt[1][30]*ROcp58_116+s->dpt[2][30]*ROcp58_417+s->dpt[3][30]*ROcp58_717;
  RLcp58_218 = s->dpt[1][30]*ROcp58_216+s->dpt[2][30]*ROcp58_517+s->dpt[3][30]*ROcp58_817;
  RLcp58_318 = s->dpt[1][30]*ROcp58_316+s->dpt[2][30]*ROcp58_617+s->dpt[3][30]*ROcp58_917;
  OMcp58_118 = OMcp58_117+qd[18]*ROcp58_417;
  OMcp58_218 = OMcp58_217+qd[18]*ROcp58_517;
  OMcp58_318 = OMcp58_317+qd[18]*ROcp58_617;
  ORcp58_118 = OMcp58_217*RLcp58_318-OMcp58_317*RLcp58_218;
  ORcp58_218 = -(OMcp58_117*RLcp58_318-OMcp58_317*RLcp58_118);
  ORcp58_318 = OMcp58_117*RLcp58_218-OMcp58_217*RLcp58_118;
  OPcp58_118 = OPcp58_117+qd[18]*(OMcp58_217*ROcp58_617-OMcp58_317*ROcp58_517)+qdd[18]*ROcp58_417;
  OPcp58_218 = OPcp58_217-qd[18]*(OMcp58_117*ROcp58_617-OMcp58_317*ROcp58_417)+qdd[18]*ROcp58_517;
  OPcp58_318 = OPcp58_317+qd[18]*(OMcp58_117*ROcp58_517-OMcp58_217*ROcp58_417)+qdd[18]*ROcp58_617;
  RLcp58_194 = ROcp58_118*s->dpt[1][35]+ROcp58_417*s->dpt[2][35]+ROcp58_718*s->dpt[3][35];
  RLcp58_294 = ROcp58_218*s->dpt[1][35]+ROcp58_517*s->dpt[2][35]+ROcp58_818*s->dpt[3][35];
  RLcp58_394 = ROcp58_318*s->dpt[1][35]+ROcp58_617*s->dpt[2][35]+ROcp58_918*s->dpt[3][35];
  ORcp58_194 = OMcp58_218*RLcp58_394-OMcp58_318*RLcp58_294;
  ORcp58_294 = -(OMcp58_118*RLcp58_394-OMcp58_318*RLcp58_194);
  ORcp58_394 = OMcp58_118*RLcp58_294-OMcp58_218*RLcp58_194;
  PxF8[1] = q[1]+RLcp58_113+RLcp58_114+RLcp58_115+RLcp58_116+RLcp58_117+RLcp58_118+RLcp58_194;
  PxF8[2] = q[2]+RLcp58_213+RLcp58_214+RLcp58_215+RLcp58_216+RLcp58_217+RLcp58_218+RLcp58_294;
  PxF8[3] = q[3]+RLcp58_313+RLcp58_314+RLcp58_315+RLcp58_316+RLcp58_317+RLcp58_318+RLcp58_394;
  RxF8[1][1] = ROcp58_118;
  RxF8[1][2] = ROcp58_218;
  RxF8[1][3] = ROcp58_318;
  RxF8[2][1] = ROcp58_417;
  RxF8[2][2] = ROcp58_517;
  RxF8[2][3] = ROcp58_617;
  RxF8[3][1] = ROcp58_718;
  RxF8[3][2] = ROcp58_818;
  RxF8[3][3] = ROcp58_918;
  VxF8[1] = qd[1]+ORcp58_113+ORcp58_114+ORcp58_115+ORcp58_116+ORcp58_117+ORcp58_118+ORcp58_194;
  VxF8[2] = qd[2]+ORcp58_213+ORcp58_214+ORcp58_215+ORcp58_216+ORcp58_217+ORcp58_218+ORcp58_294;
  VxF8[3] = qd[3]+ORcp58_313+ORcp58_314+ORcp58_315+ORcp58_316+ORcp58_317+ORcp58_318+ORcp58_394;
  OMxF8[1] = OMcp58_118;
  OMxF8[2] = OMcp58_218;
  OMxF8[3] = OMcp58_318;
  AxF8[1] = qdd[1]+OMcp58_213*ORcp58_314+OMcp58_214*ORcp58_315+OMcp58_215*ORcp58_316+OMcp58_216*ORcp58_317+OMcp58_217*
 ORcp58_318+OMcp58_218*ORcp58_394+OMcp58_26*ORcp58_313-OMcp58_313*ORcp58_214-OMcp58_314*ORcp58_215-OMcp58_315*ORcp58_216-
 OMcp58_316*ORcp58_217-OMcp58_317*ORcp58_218-OMcp58_318*ORcp58_294-OMcp58_36*ORcp58_213+OPcp58_213*RLcp58_314+OPcp58_214*
 RLcp58_315+OPcp58_215*RLcp58_316+OPcp58_216*RLcp58_317+OPcp58_217*RLcp58_318+OPcp58_218*RLcp58_394+OPcp58_26*RLcp58_313-
 OPcp58_313*RLcp58_214-OPcp58_314*RLcp58_215-OPcp58_315*RLcp58_216-OPcp58_316*RLcp58_217-OPcp58_317*RLcp58_218-OPcp58_318*
 RLcp58_294-OPcp58_36*RLcp58_213;
  AxF8[2] = qdd[2]-OMcp58_113*ORcp58_314-OMcp58_114*ORcp58_315-OMcp58_115*ORcp58_316-OMcp58_116*ORcp58_317-OMcp58_117*
 ORcp58_318-OMcp58_118*ORcp58_394-OMcp58_16*ORcp58_313+OMcp58_313*ORcp58_114+OMcp58_314*ORcp58_115+OMcp58_315*ORcp58_116+
 OMcp58_316*ORcp58_117+OMcp58_317*ORcp58_118+OMcp58_318*ORcp58_194+OMcp58_36*ORcp58_113-OPcp58_113*RLcp58_314-OPcp58_114*
 RLcp58_315-OPcp58_115*RLcp58_316-OPcp58_116*RLcp58_317-OPcp58_117*RLcp58_318-OPcp58_118*RLcp58_394-OPcp58_16*RLcp58_313+
 OPcp58_313*RLcp58_114+OPcp58_314*RLcp58_115+OPcp58_315*RLcp58_116+OPcp58_316*RLcp58_117+OPcp58_317*RLcp58_118+OPcp58_318*
 RLcp58_194+OPcp58_36*RLcp58_113;
  AxF8[3] = qdd[3]+OMcp58_113*ORcp58_214+OMcp58_114*ORcp58_215+OMcp58_115*ORcp58_216+OMcp58_116*ORcp58_217+OMcp58_117*
 ORcp58_218+OMcp58_118*ORcp58_294+OMcp58_16*ORcp58_213-OMcp58_213*ORcp58_114-OMcp58_214*ORcp58_115-OMcp58_215*ORcp58_116-
 OMcp58_216*ORcp58_117-OMcp58_217*ORcp58_118-OMcp58_218*ORcp58_194-OMcp58_26*ORcp58_113+OPcp58_113*RLcp58_214+OPcp58_114*
 RLcp58_215+OPcp58_115*RLcp58_216+OPcp58_116*RLcp58_217+OPcp58_117*RLcp58_218+OPcp58_118*RLcp58_294+OPcp58_16*RLcp58_213-
 OPcp58_213*RLcp58_114-OPcp58_214*RLcp58_115-OPcp58_215*RLcp58_116-OPcp58_216*RLcp58_117-OPcp58_217*RLcp58_118-OPcp58_218*
 RLcp58_194-OPcp58_26*RLcp58_113;
  OMPxF8[1] = OPcp58_118;
  OMPxF8[2] = OPcp58_218;
  OMPxF8[3] = OPcp58_318;
 
// Sensor Forces Computation 

  SWr8 = user_ExtForces(PxF8,RxF8,VxF8,OMxF8,AxF8,OMPxF8,s,tsim,8);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc159 = ROcp58_118*SWr8[1]+ROcp58_218*SWr8[2]+ROcp58_318*SWr8[3];
  xfrc259 = ROcp58_417*SWr8[1]+ROcp58_517*SWr8[2]+ROcp58_617*SWr8[3];
  xfrc359 = ROcp58_718*SWr8[1]+ROcp58_818*SWr8[2]+ROcp58_918*SWr8[3];
  s->frc[1][18] = s->frc[1][18]+xfrc159;
  s->frc[2][18] = s->frc[2][18]+xfrc259;
  s->frc[3][18] = s->frc[3][18]+xfrc359;
  xtrq159 = ROcp58_118*SWr8[4]+ROcp58_218*SWr8[5]+ROcp58_318*SWr8[6];
  xtrq259 = ROcp58_417*SWr8[4]+ROcp58_517*SWr8[5]+ROcp58_617*SWr8[6];
  xtrq359 = ROcp58_718*SWr8[4]+ROcp58_818*SWr8[5]+ROcp58_918*SWr8[6];
  s->trq[1][18] = s->trq[1][18]+xtrq159-xfrc259*(SWr8[9]-s->l[3][18])+xfrc359*(SWr8[8]-s->l[2][18]);
  s->trq[2][18] = s->trq[2][18]+xtrq259+xfrc159*(SWr8[9]-s->l[3][18])-xfrc359*(SWr8[7]-s->l[1][18]);
  s->trq[3][18] = s->trq[3][18]+xtrq359-xfrc159*(SWr8[8]-s->l[2][18])+xfrc259*(SWr8[7]-s->l[1][18]);

// = = Block_0_0_1_9_0_1 = = 
 
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
  OMcp59_26 = OMcp59_25+qd[6]*ROcp59_85;
  OMcp59_36 = OMcp59_35+qd[6]*ROcp59_95;
  OPcp59_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp59_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp59_95-OMcp59_35*S5)-qdd[5]*C4-qdd[6]*ROcp59_85);
  OPcp59_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp59_85-OMcp59_25*S5)+qdd[5]*S4+qdd[6]*ROcp59_95;

// = = Block_0_0_1_9_0_3 = = 
 
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
  OMcp59_113 = OMcp59_16+qd[13]*ROcp59_46;
  OMcp59_213 = OMcp59_26+qd[13]*ROcp59_56;
  OMcp59_313 = OMcp59_36+qd[13]*ROcp59_66;
  ORcp59_113 = OMcp59_26*RLcp59_313-OMcp59_36*RLcp59_213;
  ORcp59_213 = -(OMcp59_16*RLcp59_313-OMcp59_36*RLcp59_113);
  ORcp59_313 = OMcp59_16*RLcp59_213-OMcp59_26*RLcp59_113;
  OPcp59_113 = OPcp59_16+qd[13]*(OMcp59_26*ROcp59_66-OMcp59_36*ROcp59_56)+qdd[13]*ROcp59_46;
  OPcp59_213 = OPcp59_26-qd[13]*(OMcp59_16*ROcp59_66-OMcp59_36*ROcp59_46)+qdd[13]*ROcp59_56;
  OPcp59_313 = OPcp59_36+qd[13]*(OMcp59_16*ROcp59_56-OMcp59_26*ROcp59_46)+qdd[13]*ROcp59_66;
  RLcp59_114 = s->dpt[1][22]*ROcp59_113+s->dpt[3][22]*ROcp59_713+ROcp59_46*s->dpt[2][22];
  RLcp59_214 = s->dpt[1][22]*ROcp59_213+s->dpt[3][22]*ROcp59_813+ROcp59_56*s->dpt[2][22];
  RLcp59_314 = s->dpt[1][22]*ROcp59_313+s->dpt[3][22]*ROcp59_913+ROcp59_66*s->dpt[2][22];
  OMcp59_114 = OMcp59_113+qd[14]*ROcp59_113;
  OMcp59_214 = OMcp59_213+qd[14]*ROcp59_213;
  OMcp59_314 = OMcp59_313+qd[14]*ROcp59_313;
  ORcp59_114 = OMcp59_213*RLcp59_314-OMcp59_313*RLcp59_214;
  ORcp59_214 = -(OMcp59_113*RLcp59_314-OMcp59_313*RLcp59_114);
  ORcp59_314 = OMcp59_113*RLcp59_214-OMcp59_213*RLcp59_114;
  OPcp59_114 = OPcp59_113+qd[14]*(OMcp59_213*ROcp59_313-OMcp59_313*ROcp59_213)+qdd[14]*ROcp59_113;
  OPcp59_214 = OPcp59_213-qd[14]*(OMcp59_113*ROcp59_313-OMcp59_313*ROcp59_113)+qdd[14]*ROcp59_213;
  OPcp59_314 = OPcp59_313+qd[14]*(OMcp59_113*ROcp59_213-OMcp59_213*ROcp59_113)+qdd[14]*ROcp59_313;
  RLcp59_115 = s->dpt[1][24]*ROcp59_113+s->dpt[2][24]*ROcp59_414+ROcp59_714*s->dpt[3][24];
  RLcp59_215 = s->dpt[1][24]*ROcp59_213+s->dpt[2][24]*ROcp59_514+ROcp59_814*s->dpt[3][24];
  RLcp59_315 = s->dpt[1][24]*ROcp59_313+s->dpt[2][24]*ROcp59_614+ROcp59_914*s->dpt[3][24];
  OMcp59_115 = OMcp59_114+qd[15]*ROcp59_714;
  OMcp59_215 = OMcp59_214+qd[15]*ROcp59_814;
  OMcp59_315 = OMcp59_314+qd[15]*ROcp59_914;
  ORcp59_115 = OMcp59_214*RLcp59_315-OMcp59_314*RLcp59_215;
  ORcp59_215 = -(OMcp59_114*RLcp59_315-OMcp59_314*RLcp59_115);
  ORcp59_315 = OMcp59_114*RLcp59_215-OMcp59_214*RLcp59_115;
  OPcp59_115 = OPcp59_114+qd[15]*(OMcp59_214*ROcp59_914-OMcp59_314*ROcp59_814)+qdd[15]*ROcp59_714;
  OPcp59_215 = OPcp59_214-qd[15]*(OMcp59_114*ROcp59_914-OMcp59_314*ROcp59_714)+qdd[15]*ROcp59_814;
  OPcp59_315 = OPcp59_314+qd[15]*(OMcp59_114*ROcp59_814-OMcp59_214*ROcp59_714)+qdd[15]*ROcp59_914;
  RLcp59_116 = s->dpt[1][26]*ROcp59_115+s->dpt[2][26]*ROcp59_415+ROcp59_714*s->dpt[3][26];
  RLcp59_216 = s->dpt[1][26]*ROcp59_215+s->dpt[2][26]*ROcp59_515+ROcp59_814*s->dpt[3][26];
  RLcp59_316 = s->dpt[1][26]*ROcp59_315+s->dpt[2][26]*ROcp59_615+ROcp59_914*s->dpt[3][26];
  OMcp59_116 = OMcp59_115+qd[16]*ROcp59_415;
  OMcp59_216 = OMcp59_215+qd[16]*ROcp59_515;
  OMcp59_316 = OMcp59_315+qd[16]*ROcp59_615;
  ORcp59_116 = OMcp59_215*RLcp59_316-OMcp59_315*RLcp59_216;
  ORcp59_216 = -(OMcp59_115*RLcp59_316-OMcp59_315*RLcp59_116);
  ORcp59_316 = OMcp59_115*RLcp59_216-OMcp59_215*RLcp59_116;
  OPcp59_116 = OPcp59_115+qd[16]*(OMcp59_215*ROcp59_615-OMcp59_315*ROcp59_515)+qdd[16]*ROcp59_415;
  OPcp59_216 = OPcp59_215-qd[16]*(OMcp59_115*ROcp59_615-OMcp59_315*ROcp59_415)+qdd[16]*ROcp59_515;
  OPcp59_316 = OPcp59_315+qd[16]*(OMcp59_115*ROcp59_515-OMcp59_215*ROcp59_415)+qdd[16]*ROcp59_615;
  RLcp59_117 = s->dpt[1][28]*ROcp59_116+s->dpt[2][28]*ROcp59_415+ROcp59_716*s->dpt[3][28];
  RLcp59_217 = s->dpt[1][28]*ROcp59_216+s->dpt[2][28]*ROcp59_515+ROcp59_816*s->dpt[3][28];
  RLcp59_317 = s->dpt[1][28]*ROcp59_316+s->dpt[2][28]*ROcp59_615+ROcp59_916*s->dpt[3][28];
  OMcp59_117 = OMcp59_116+qd[17]*ROcp59_116;
  OMcp59_217 = OMcp59_216+qd[17]*ROcp59_216;
  OMcp59_317 = OMcp59_316+qd[17]*ROcp59_316;
  ORcp59_117 = OMcp59_216*RLcp59_317-OMcp59_316*RLcp59_217;
  ORcp59_217 = -(OMcp59_116*RLcp59_317-OMcp59_316*RLcp59_117);
  ORcp59_317 = OMcp59_116*RLcp59_217-OMcp59_216*RLcp59_117;
  OPcp59_117 = OPcp59_116+qd[17]*(OMcp59_216*ROcp59_316-OMcp59_316*ROcp59_216)+qdd[17]*ROcp59_116;
  OPcp59_217 = OPcp59_216-qd[17]*(OMcp59_116*ROcp59_316-OMcp59_316*ROcp59_116)+qdd[17]*ROcp59_216;
  OPcp59_317 = OPcp59_316+qd[17]*(OMcp59_116*ROcp59_216-OMcp59_216*ROcp59_116)+qdd[17]*ROcp59_316;
  RLcp59_118 = s->dpt[1][30]*ROcp59_116+s->dpt[2][30]*ROcp59_417+s->dpt[3][30]*ROcp59_717;
  RLcp59_218 = s->dpt[1][30]*ROcp59_216+s->dpt[2][30]*ROcp59_517+s->dpt[3][30]*ROcp59_817;
  RLcp59_318 = s->dpt[1][30]*ROcp59_316+s->dpt[2][30]*ROcp59_617+s->dpt[3][30]*ROcp59_917;
  OMcp59_118 = OMcp59_117+qd[18]*ROcp59_417;
  OMcp59_218 = OMcp59_217+qd[18]*ROcp59_517;
  OMcp59_318 = OMcp59_317+qd[18]*ROcp59_617;
  ORcp59_118 = OMcp59_217*RLcp59_318-OMcp59_317*RLcp59_218;
  ORcp59_218 = -(OMcp59_117*RLcp59_318-OMcp59_317*RLcp59_118);
  ORcp59_318 = OMcp59_117*RLcp59_218-OMcp59_217*RLcp59_118;
  OPcp59_118 = OPcp59_117+qd[18]*(OMcp59_217*ROcp59_617-OMcp59_317*ROcp59_517)+qdd[18]*ROcp59_417;
  OPcp59_218 = OPcp59_217-qd[18]*(OMcp59_117*ROcp59_617-OMcp59_317*ROcp59_417)+qdd[18]*ROcp59_517;
  OPcp59_318 = OPcp59_317+qd[18]*(OMcp59_117*ROcp59_517-OMcp59_217*ROcp59_417)+qdd[18]*ROcp59_617;
  RLcp59_195 = ROcp59_118*s->dpt[1][36]+ROcp59_417*s->dpt[2][36]+ROcp59_718*s->dpt[3][36];
  RLcp59_295 = ROcp59_218*s->dpt[1][36]+ROcp59_517*s->dpt[2][36]+ROcp59_818*s->dpt[3][36];
  RLcp59_395 = ROcp59_318*s->dpt[1][36]+ROcp59_617*s->dpt[2][36]+ROcp59_918*s->dpt[3][36];
  ORcp59_195 = OMcp59_218*RLcp59_395-OMcp59_318*RLcp59_295;
  ORcp59_295 = -(OMcp59_118*RLcp59_395-OMcp59_318*RLcp59_195);
  ORcp59_395 = OMcp59_118*RLcp59_295-OMcp59_218*RLcp59_195;
  PxF9[1] = q[1]+RLcp59_113+RLcp59_114+RLcp59_115+RLcp59_116+RLcp59_117+RLcp59_118+RLcp59_195;
  PxF9[2] = q[2]+RLcp59_213+RLcp59_214+RLcp59_215+RLcp59_216+RLcp59_217+RLcp59_218+RLcp59_295;
  PxF9[3] = q[3]+RLcp59_313+RLcp59_314+RLcp59_315+RLcp59_316+RLcp59_317+RLcp59_318+RLcp59_395;
  RxF9[1][1] = ROcp59_118;
  RxF9[1][2] = ROcp59_218;
  RxF9[1][3] = ROcp59_318;
  RxF9[2][1] = ROcp59_417;
  RxF9[2][2] = ROcp59_517;
  RxF9[2][3] = ROcp59_617;
  RxF9[3][1] = ROcp59_718;
  RxF9[3][2] = ROcp59_818;
  RxF9[3][3] = ROcp59_918;
  VxF9[1] = qd[1]+ORcp59_113+ORcp59_114+ORcp59_115+ORcp59_116+ORcp59_117+ORcp59_118+ORcp59_195;
  VxF9[2] = qd[2]+ORcp59_213+ORcp59_214+ORcp59_215+ORcp59_216+ORcp59_217+ORcp59_218+ORcp59_295;
  VxF9[3] = qd[3]+ORcp59_313+ORcp59_314+ORcp59_315+ORcp59_316+ORcp59_317+ORcp59_318+ORcp59_395;
  OMxF9[1] = OMcp59_118;
  OMxF9[2] = OMcp59_218;
  OMxF9[3] = OMcp59_318;
  AxF9[1] = qdd[1]+OMcp59_213*ORcp59_314+OMcp59_214*ORcp59_315+OMcp59_215*ORcp59_316+OMcp59_216*ORcp59_317+OMcp59_217*
 ORcp59_318+OMcp59_218*ORcp59_395+OMcp59_26*ORcp59_313-OMcp59_313*ORcp59_214-OMcp59_314*ORcp59_215-OMcp59_315*ORcp59_216-
 OMcp59_316*ORcp59_217-OMcp59_317*ORcp59_218-OMcp59_318*ORcp59_295-OMcp59_36*ORcp59_213+OPcp59_213*RLcp59_314+OPcp59_214*
 RLcp59_315+OPcp59_215*RLcp59_316+OPcp59_216*RLcp59_317+OPcp59_217*RLcp59_318+OPcp59_218*RLcp59_395+OPcp59_26*RLcp59_313-
 OPcp59_313*RLcp59_214-OPcp59_314*RLcp59_215-OPcp59_315*RLcp59_216-OPcp59_316*RLcp59_217-OPcp59_317*RLcp59_218-OPcp59_318*
 RLcp59_295-OPcp59_36*RLcp59_213;
  AxF9[2] = qdd[2]-OMcp59_113*ORcp59_314-OMcp59_114*ORcp59_315-OMcp59_115*ORcp59_316-OMcp59_116*ORcp59_317-OMcp59_117*
 ORcp59_318-OMcp59_118*ORcp59_395-OMcp59_16*ORcp59_313+OMcp59_313*ORcp59_114+OMcp59_314*ORcp59_115+OMcp59_315*ORcp59_116+
 OMcp59_316*ORcp59_117+OMcp59_317*ORcp59_118+OMcp59_318*ORcp59_195+OMcp59_36*ORcp59_113-OPcp59_113*RLcp59_314-OPcp59_114*
 RLcp59_315-OPcp59_115*RLcp59_316-OPcp59_116*RLcp59_317-OPcp59_117*RLcp59_318-OPcp59_118*RLcp59_395-OPcp59_16*RLcp59_313+
 OPcp59_313*RLcp59_114+OPcp59_314*RLcp59_115+OPcp59_315*RLcp59_116+OPcp59_316*RLcp59_117+OPcp59_317*RLcp59_118+OPcp59_318*
 RLcp59_195+OPcp59_36*RLcp59_113;
  AxF9[3] = qdd[3]+OMcp59_113*ORcp59_214+OMcp59_114*ORcp59_215+OMcp59_115*ORcp59_216+OMcp59_116*ORcp59_217+OMcp59_117*
 ORcp59_218+OMcp59_118*ORcp59_295+OMcp59_16*ORcp59_213-OMcp59_213*ORcp59_114-OMcp59_214*ORcp59_115-OMcp59_215*ORcp59_116-
 OMcp59_216*ORcp59_117-OMcp59_217*ORcp59_118-OMcp59_218*ORcp59_195-OMcp59_26*ORcp59_113+OPcp59_113*RLcp59_214+OPcp59_114*
 RLcp59_215+OPcp59_115*RLcp59_216+OPcp59_116*RLcp59_217+OPcp59_117*RLcp59_218+OPcp59_118*RLcp59_295+OPcp59_16*RLcp59_213-
 OPcp59_213*RLcp59_114-OPcp59_214*RLcp59_115-OPcp59_215*RLcp59_116-OPcp59_216*RLcp59_117-OPcp59_217*RLcp59_118-OPcp59_218*
 RLcp59_195-OPcp59_26*RLcp59_113;
  OMPxF9[1] = OPcp59_118;
  OMPxF9[2] = OPcp59_218;
  OMPxF9[3] = OPcp59_318;
 
// Sensor Forces Computation 

  SWr9 = user_ExtForces(PxF9,RxF9,VxF9,OMxF9,AxF9,OMPxF9,s,tsim,9);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc160 = ROcp59_118*SWr9[1]+ROcp59_218*SWr9[2]+ROcp59_318*SWr9[3];
  xfrc260 = ROcp59_417*SWr9[1]+ROcp59_517*SWr9[2]+ROcp59_617*SWr9[3];
  xfrc360 = ROcp59_718*SWr9[1]+ROcp59_818*SWr9[2]+ROcp59_918*SWr9[3];
  s->frc[1][18] = s->frc[1][18]+xfrc160;
  s->frc[2][18] = s->frc[2][18]+xfrc260;
  s->frc[3][18] = s->frc[3][18]+xfrc360;
  xtrq160 = ROcp59_118*SWr9[4]+ROcp59_218*SWr9[5]+ROcp59_318*SWr9[6];
  xtrq260 = ROcp59_417*SWr9[4]+ROcp59_517*SWr9[5]+ROcp59_617*SWr9[6];
  xtrq360 = ROcp59_718*SWr9[4]+ROcp59_818*SWr9[5]+ROcp59_918*SWr9[6];
  s->trq[1][18] = s->trq[1][18]+xtrq160-xfrc260*(SWr9[9]-s->l[3][18])+xfrc360*(SWr9[8]-s->l[2][18]);
  s->trq[2][18] = s->trq[2][18]+xtrq260+xfrc160*(SWr9[9]-s->l[3][18])-xfrc360*(SWr9[7]-s->l[1][18]);
  s->trq[3][18] = s->trq[3][18]+xtrq360-xfrc160*(SWr9[8]-s->l[2][18])+xfrc260*(SWr9[7]-s->l[1][18]);

// = = Block_0_0_1_10_0_1 = = 
 
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
  OMcp60_26 = OMcp60_25+qd[6]*ROcp60_85;
  OMcp60_36 = OMcp60_35+qd[6]*ROcp60_95;
  OPcp60_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp60_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp60_95-OMcp60_35*S5)-qdd[5]*C4-qdd[6]*ROcp60_85);
  OPcp60_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp60_85-OMcp60_25*S5)+qdd[5]*S4+qdd[6]*ROcp60_95;

// = = Block_0_0_1_10_0_3 = = 
 
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
  OMcp60_113 = OMcp60_16+qd[13]*ROcp60_46;
  OMcp60_213 = OMcp60_26+qd[13]*ROcp60_56;
  OMcp60_313 = OMcp60_36+qd[13]*ROcp60_66;
  ORcp60_113 = OMcp60_26*RLcp60_313-OMcp60_36*RLcp60_213;
  ORcp60_213 = -(OMcp60_16*RLcp60_313-OMcp60_36*RLcp60_113);
  ORcp60_313 = OMcp60_16*RLcp60_213-OMcp60_26*RLcp60_113;
  OPcp60_113 = OPcp60_16+qd[13]*(OMcp60_26*ROcp60_66-OMcp60_36*ROcp60_56)+qdd[13]*ROcp60_46;
  OPcp60_213 = OPcp60_26-qd[13]*(OMcp60_16*ROcp60_66-OMcp60_36*ROcp60_46)+qdd[13]*ROcp60_56;
  OPcp60_313 = OPcp60_36+qd[13]*(OMcp60_16*ROcp60_56-OMcp60_26*ROcp60_46)+qdd[13]*ROcp60_66;
  RLcp60_114 = s->dpt[1][22]*ROcp60_113+s->dpt[3][22]*ROcp60_713+ROcp60_46*s->dpt[2][22];
  RLcp60_214 = s->dpt[1][22]*ROcp60_213+s->dpt[3][22]*ROcp60_813+ROcp60_56*s->dpt[2][22];
  RLcp60_314 = s->dpt[1][22]*ROcp60_313+s->dpt[3][22]*ROcp60_913+ROcp60_66*s->dpt[2][22];
  OMcp60_114 = OMcp60_113+qd[14]*ROcp60_113;
  OMcp60_214 = OMcp60_213+qd[14]*ROcp60_213;
  OMcp60_314 = OMcp60_313+qd[14]*ROcp60_313;
  ORcp60_114 = OMcp60_213*RLcp60_314-OMcp60_313*RLcp60_214;
  ORcp60_214 = -(OMcp60_113*RLcp60_314-OMcp60_313*RLcp60_114);
  ORcp60_314 = OMcp60_113*RLcp60_214-OMcp60_213*RLcp60_114;
  OPcp60_114 = OPcp60_113+qd[14]*(OMcp60_213*ROcp60_313-OMcp60_313*ROcp60_213)+qdd[14]*ROcp60_113;
  OPcp60_214 = OPcp60_213-qd[14]*(OMcp60_113*ROcp60_313-OMcp60_313*ROcp60_113)+qdd[14]*ROcp60_213;
  OPcp60_314 = OPcp60_313+qd[14]*(OMcp60_113*ROcp60_213-OMcp60_213*ROcp60_113)+qdd[14]*ROcp60_313;
  RLcp60_115 = s->dpt[1][24]*ROcp60_113+s->dpt[2][24]*ROcp60_414+ROcp60_714*s->dpt[3][24];
  RLcp60_215 = s->dpt[1][24]*ROcp60_213+s->dpt[2][24]*ROcp60_514+ROcp60_814*s->dpt[3][24];
  RLcp60_315 = s->dpt[1][24]*ROcp60_313+s->dpt[2][24]*ROcp60_614+ROcp60_914*s->dpt[3][24];
  OMcp60_115 = OMcp60_114+qd[15]*ROcp60_714;
  OMcp60_215 = OMcp60_214+qd[15]*ROcp60_814;
  OMcp60_315 = OMcp60_314+qd[15]*ROcp60_914;
  ORcp60_115 = OMcp60_214*RLcp60_315-OMcp60_314*RLcp60_215;
  ORcp60_215 = -(OMcp60_114*RLcp60_315-OMcp60_314*RLcp60_115);
  ORcp60_315 = OMcp60_114*RLcp60_215-OMcp60_214*RLcp60_115;
  OPcp60_115 = OPcp60_114+qd[15]*(OMcp60_214*ROcp60_914-OMcp60_314*ROcp60_814)+qdd[15]*ROcp60_714;
  OPcp60_215 = OPcp60_214-qd[15]*(OMcp60_114*ROcp60_914-OMcp60_314*ROcp60_714)+qdd[15]*ROcp60_814;
  OPcp60_315 = OPcp60_314+qd[15]*(OMcp60_114*ROcp60_814-OMcp60_214*ROcp60_714)+qdd[15]*ROcp60_914;
  RLcp60_116 = s->dpt[1][26]*ROcp60_115+s->dpt[2][26]*ROcp60_415+ROcp60_714*s->dpt[3][26];
  RLcp60_216 = s->dpt[1][26]*ROcp60_215+s->dpt[2][26]*ROcp60_515+ROcp60_814*s->dpt[3][26];
  RLcp60_316 = s->dpt[1][26]*ROcp60_315+s->dpt[2][26]*ROcp60_615+ROcp60_914*s->dpt[3][26];
  OMcp60_116 = OMcp60_115+qd[16]*ROcp60_415;
  OMcp60_216 = OMcp60_215+qd[16]*ROcp60_515;
  OMcp60_316 = OMcp60_315+qd[16]*ROcp60_615;
  ORcp60_116 = OMcp60_215*RLcp60_316-OMcp60_315*RLcp60_216;
  ORcp60_216 = -(OMcp60_115*RLcp60_316-OMcp60_315*RLcp60_116);
  ORcp60_316 = OMcp60_115*RLcp60_216-OMcp60_215*RLcp60_116;
  OPcp60_116 = OPcp60_115+qd[16]*(OMcp60_215*ROcp60_615-OMcp60_315*ROcp60_515)+qdd[16]*ROcp60_415;
  OPcp60_216 = OPcp60_215-qd[16]*(OMcp60_115*ROcp60_615-OMcp60_315*ROcp60_415)+qdd[16]*ROcp60_515;
  OPcp60_316 = OPcp60_315+qd[16]*(OMcp60_115*ROcp60_515-OMcp60_215*ROcp60_415)+qdd[16]*ROcp60_615;
  RLcp60_117 = s->dpt[1][28]*ROcp60_116+s->dpt[2][28]*ROcp60_415+ROcp60_716*s->dpt[3][28];
  RLcp60_217 = s->dpt[1][28]*ROcp60_216+s->dpt[2][28]*ROcp60_515+ROcp60_816*s->dpt[3][28];
  RLcp60_317 = s->dpt[1][28]*ROcp60_316+s->dpt[2][28]*ROcp60_615+ROcp60_916*s->dpt[3][28];
  OMcp60_117 = OMcp60_116+qd[17]*ROcp60_116;
  OMcp60_217 = OMcp60_216+qd[17]*ROcp60_216;
  OMcp60_317 = OMcp60_316+qd[17]*ROcp60_316;
  ORcp60_117 = OMcp60_216*RLcp60_317-OMcp60_316*RLcp60_217;
  ORcp60_217 = -(OMcp60_116*RLcp60_317-OMcp60_316*RLcp60_117);
  ORcp60_317 = OMcp60_116*RLcp60_217-OMcp60_216*RLcp60_117;
  OPcp60_117 = OPcp60_116+qd[17]*(OMcp60_216*ROcp60_316-OMcp60_316*ROcp60_216)+qdd[17]*ROcp60_116;
  OPcp60_217 = OPcp60_216-qd[17]*(OMcp60_116*ROcp60_316-OMcp60_316*ROcp60_116)+qdd[17]*ROcp60_216;
  OPcp60_317 = OPcp60_316+qd[17]*(OMcp60_116*ROcp60_216-OMcp60_216*ROcp60_116)+qdd[17]*ROcp60_316;
  RLcp60_118 = s->dpt[1][30]*ROcp60_116+s->dpt[2][30]*ROcp60_417+s->dpt[3][30]*ROcp60_717;
  RLcp60_218 = s->dpt[1][30]*ROcp60_216+s->dpt[2][30]*ROcp60_517+s->dpt[3][30]*ROcp60_817;
  RLcp60_318 = s->dpt[1][30]*ROcp60_316+s->dpt[2][30]*ROcp60_617+s->dpt[3][30]*ROcp60_917;
  OMcp60_118 = OMcp60_117+qd[18]*ROcp60_417;
  OMcp60_218 = OMcp60_217+qd[18]*ROcp60_517;
  OMcp60_318 = OMcp60_317+qd[18]*ROcp60_617;
  ORcp60_118 = OMcp60_217*RLcp60_318-OMcp60_317*RLcp60_218;
  ORcp60_218 = -(OMcp60_117*RLcp60_318-OMcp60_317*RLcp60_118);
  ORcp60_318 = OMcp60_117*RLcp60_218-OMcp60_217*RLcp60_118;
  OPcp60_118 = OPcp60_117+qd[18]*(OMcp60_217*ROcp60_617-OMcp60_317*ROcp60_517)+qdd[18]*ROcp60_417;
  OPcp60_218 = OPcp60_217-qd[18]*(OMcp60_117*ROcp60_617-OMcp60_317*ROcp60_417)+qdd[18]*ROcp60_517;
  OPcp60_318 = OPcp60_317+qd[18]*(OMcp60_117*ROcp60_517-OMcp60_217*ROcp60_417)+qdd[18]*ROcp60_617;
  RLcp60_196 = ROcp60_118*s->dpt[1][37]+ROcp60_417*s->dpt[2][37]+ROcp60_718*s->dpt[3][37];
  RLcp60_296 = ROcp60_218*s->dpt[1][37]+ROcp60_517*s->dpt[2][37]+ROcp60_818*s->dpt[3][37];
  RLcp60_396 = ROcp60_318*s->dpt[1][37]+ROcp60_617*s->dpt[2][37]+ROcp60_918*s->dpt[3][37];
  ORcp60_196 = OMcp60_218*RLcp60_396-OMcp60_318*RLcp60_296;
  ORcp60_296 = -(OMcp60_118*RLcp60_396-OMcp60_318*RLcp60_196);
  ORcp60_396 = OMcp60_118*RLcp60_296-OMcp60_218*RLcp60_196;
  PxF10[1] = q[1]+RLcp60_113+RLcp60_114+RLcp60_115+RLcp60_116+RLcp60_117+RLcp60_118+RLcp60_196;
  PxF10[2] = q[2]+RLcp60_213+RLcp60_214+RLcp60_215+RLcp60_216+RLcp60_217+RLcp60_218+RLcp60_296;
  PxF10[3] = q[3]+RLcp60_313+RLcp60_314+RLcp60_315+RLcp60_316+RLcp60_317+RLcp60_318+RLcp60_396;
  RxF10[1][1] = ROcp60_118;
  RxF10[1][2] = ROcp60_218;
  RxF10[1][3] = ROcp60_318;
  RxF10[2][1] = ROcp60_417;
  RxF10[2][2] = ROcp60_517;
  RxF10[2][3] = ROcp60_617;
  RxF10[3][1] = ROcp60_718;
  RxF10[3][2] = ROcp60_818;
  RxF10[3][3] = ROcp60_918;
  VxF10[1] = qd[1]+ORcp60_113+ORcp60_114+ORcp60_115+ORcp60_116+ORcp60_117+ORcp60_118+ORcp60_196;
  VxF10[2] = qd[2]+ORcp60_213+ORcp60_214+ORcp60_215+ORcp60_216+ORcp60_217+ORcp60_218+ORcp60_296;
  VxF10[3] = qd[3]+ORcp60_313+ORcp60_314+ORcp60_315+ORcp60_316+ORcp60_317+ORcp60_318+ORcp60_396;
  OMxF10[1] = OMcp60_118;
  OMxF10[2] = OMcp60_218;
  OMxF10[3] = OMcp60_318;
  AxF10[1] = qdd[1]+OMcp60_213*ORcp60_314+OMcp60_214*ORcp60_315+OMcp60_215*ORcp60_316+OMcp60_216*ORcp60_317+OMcp60_217*
 ORcp60_318+OMcp60_218*ORcp60_396+OMcp60_26*ORcp60_313-OMcp60_313*ORcp60_214-OMcp60_314*ORcp60_215-OMcp60_315*ORcp60_216-
 OMcp60_316*ORcp60_217-OMcp60_317*ORcp60_218-OMcp60_318*ORcp60_296-OMcp60_36*ORcp60_213+OPcp60_213*RLcp60_314+OPcp60_214*
 RLcp60_315+OPcp60_215*RLcp60_316+OPcp60_216*RLcp60_317+OPcp60_217*RLcp60_318+OPcp60_218*RLcp60_396+OPcp60_26*RLcp60_313-
 OPcp60_313*RLcp60_214-OPcp60_314*RLcp60_215-OPcp60_315*RLcp60_216-OPcp60_316*RLcp60_217-OPcp60_317*RLcp60_218-OPcp60_318*
 RLcp60_296-OPcp60_36*RLcp60_213;
  AxF10[2] = qdd[2]-OMcp60_113*ORcp60_314-OMcp60_114*ORcp60_315-OMcp60_115*ORcp60_316-OMcp60_116*ORcp60_317-OMcp60_117*
 ORcp60_318-OMcp60_118*ORcp60_396-OMcp60_16*ORcp60_313+OMcp60_313*ORcp60_114+OMcp60_314*ORcp60_115+OMcp60_315*ORcp60_116+
 OMcp60_316*ORcp60_117+OMcp60_317*ORcp60_118+OMcp60_318*ORcp60_196+OMcp60_36*ORcp60_113-OPcp60_113*RLcp60_314-OPcp60_114*
 RLcp60_315-OPcp60_115*RLcp60_316-OPcp60_116*RLcp60_317-OPcp60_117*RLcp60_318-OPcp60_118*RLcp60_396-OPcp60_16*RLcp60_313+
 OPcp60_313*RLcp60_114+OPcp60_314*RLcp60_115+OPcp60_315*RLcp60_116+OPcp60_316*RLcp60_117+OPcp60_317*RLcp60_118+OPcp60_318*
 RLcp60_196+OPcp60_36*RLcp60_113;
  AxF10[3] = qdd[3]+OMcp60_113*ORcp60_214+OMcp60_114*ORcp60_215+OMcp60_115*ORcp60_216+OMcp60_116*ORcp60_217+OMcp60_117*
 ORcp60_218+OMcp60_118*ORcp60_296+OMcp60_16*ORcp60_213-OMcp60_213*ORcp60_114-OMcp60_214*ORcp60_115-OMcp60_215*ORcp60_116-
 OMcp60_216*ORcp60_117-OMcp60_217*ORcp60_118-OMcp60_218*ORcp60_196-OMcp60_26*ORcp60_113+OPcp60_113*RLcp60_214+OPcp60_114*
 RLcp60_215+OPcp60_115*RLcp60_216+OPcp60_116*RLcp60_217+OPcp60_117*RLcp60_218+OPcp60_118*RLcp60_296+OPcp60_16*RLcp60_213-
 OPcp60_213*RLcp60_114-OPcp60_214*RLcp60_115-OPcp60_215*RLcp60_116-OPcp60_216*RLcp60_117-OPcp60_217*RLcp60_118-OPcp60_218*
 RLcp60_196-OPcp60_26*RLcp60_113;
  OMPxF10[1] = OPcp60_118;
  OMPxF10[2] = OPcp60_218;
  OMPxF10[3] = OPcp60_318;
 
// Sensor Forces Computation 

  SWr10 = user_ExtForces(PxF10,RxF10,VxF10,OMxF10,AxF10,OMPxF10,s,tsim,10);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc161 = ROcp60_118*SWr10[1]+ROcp60_218*SWr10[2]+ROcp60_318*SWr10[3];
  xfrc261 = ROcp60_417*SWr10[1]+ROcp60_517*SWr10[2]+ROcp60_617*SWr10[3];
  xfrc361 = ROcp60_718*SWr10[1]+ROcp60_818*SWr10[2]+ROcp60_918*SWr10[3];
  frc[1][18] = s->frc[1][18]+xfrc161;
  frc[2][18] = s->frc[2][18]+xfrc261;
  frc[3][18] = s->frc[3][18]+xfrc361;
  xtrq161 = ROcp60_118*SWr10[4]+ROcp60_218*SWr10[5]+ROcp60_318*SWr10[6];
  xtrq261 = ROcp60_417*SWr10[4]+ROcp60_517*SWr10[5]+ROcp60_617*SWr10[6];
  xtrq361 = ROcp60_718*SWr10[4]+ROcp60_818*SWr10[5]+ROcp60_918*SWr10[6];
  trq[1][18] = s->trq[1][18]+xtrq161-xfrc261*(SWr10[9]-s->l[3][18])+xfrc361*(SWr10[8]-s->l[2][18]);
  trq[2][18] = s->trq[2][18]+xtrq261+xfrc161*(SWr10[9]-s->l[3][18])-xfrc361*(SWr10[7]-s->l[1][18]);
  trq[3][18] = s->trq[3][18]+xtrq361-xfrc161*(SWr10[8]-s->l[2][18])+xfrc261*(SWr10[7]-s->l[1][18]);

// = = Block_0_0_1_11_0_1 = = 
 
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
  OMcp61_26 = OMcp61_25+qd[6]*ROcp61_85;
  OMcp61_36 = OMcp61_35+qd[6]*ROcp61_95;
  OPcp61_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp61_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp61_95-OMcp61_35*S5)-qdd[5]*C4-qdd[6]*ROcp61_85);
  OPcp61_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp61_85-OMcp61_25*S5)+qdd[5]*S4+qdd[6]*ROcp61_95;

// = = Block_0_0_1_11_0_4 = = 
 
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
  OMcp61_119 = OMcp61_16+qd[19]*ROcp61_16;
  OMcp61_219 = OMcp61_26+qd[19]*ROcp61_26;
  OMcp61_319 = OMcp61_36+qd[19]*ROcp61_36;
  ORcp61_119 = OMcp61_26*RLcp61_319-OMcp61_36*RLcp61_219;
  ORcp61_219 = -(OMcp61_16*RLcp61_319-OMcp61_36*RLcp61_119);
  ORcp61_319 = OMcp61_16*RLcp61_219-OMcp61_26*RLcp61_119;
  OPcp61_119 = OPcp61_16+qd[19]*(OMcp61_26*ROcp61_36-OMcp61_36*ROcp61_26)+qdd[19]*ROcp61_16;
  OPcp61_219 = OPcp61_26-qd[19]*(OMcp61_16*ROcp61_36-OMcp61_36*ROcp61_16)+qdd[19]*ROcp61_26;
  OPcp61_319 = OPcp61_36+qd[19]*(OMcp61_16*ROcp61_26-OMcp61_26*ROcp61_16)+qdd[19]*ROcp61_36;
  RLcp61_120 = s->dpt[1][38]*ROcp61_16+s->dpt[2][38]*ROcp61_419+s->dpt[3][38]*ROcp61_719;
  RLcp61_220 = s->dpt[1][38]*ROcp61_26+s->dpt[2][38]*ROcp61_519+s->dpt[3][38]*ROcp61_819;
  RLcp61_320 = s->dpt[1][38]*ROcp61_36+s->dpt[2][38]*ROcp61_619+s->dpt[3][38]*ROcp61_919;
  OMcp61_120 = OMcp61_119+qd[20]*ROcp61_419;
  OMcp61_220 = OMcp61_219+qd[20]*ROcp61_519;
  OMcp61_320 = OMcp61_319+qd[20]*ROcp61_619;
  ORcp61_120 = OMcp61_219*RLcp61_320-OMcp61_319*RLcp61_220;
  ORcp61_220 = -(OMcp61_119*RLcp61_320-OMcp61_319*RLcp61_120);
  ORcp61_320 = OMcp61_119*RLcp61_220-OMcp61_219*RLcp61_120;
  OPcp61_120 = OPcp61_119+qd[20]*(OMcp61_219*ROcp61_619-OMcp61_319*ROcp61_519)+qdd[20]*ROcp61_419;
  OPcp61_220 = OPcp61_219-qd[20]*(OMcp61_119*ROcp61_619-OMcp61_319*ROcp61_419)+qdd[20]*ROcp61_519;
  OPcp61_320 = OPcp61_319+qd[20]*(OMcp61_119*ROcp61_519-OMcp61_219*ROcp61_419)+qdd[20]*ROcp61_619;
  RLcp61_121 = s->dpt[1][40]*ROcp61_120+s->dpt[2][40]*ROcp61_419+ROcp61_720*s->dpt[3][40];
  RLcp61_221 = s->dpt[1][40]*ROcp61_220+s->dpt[2][40]*ROcp61_519+ROcp61_820*s->dpt[3][40];
  RLcp61_321 = s->dpt[1][40]*ROcp61_320+s->dpt[2][40]*ROcp61_619+ROcp61_920*s->dpt[3][40];
  OMcp61_121 = OMcp61_120+qd[21]*ROcp61_720;
  OMcp61_221 = OMcp61_220+qd[21]*ROcp61_820;
  OMcp61_321 = OMcp61_320+qd[21]*ROcp61_920;
  ORcp61_121 = OMcp61_220*RLcp61_321-OMcp61_320*RLcp61_221;
  ORcp61_221 = -(OMcp61_120*RLcp61_321-OMcp61_320*RLcp61_121);
  ORcp61_321 = OMcp61_120*RLcp61_221-OMcp61_220*RLcp61_121;
  OPcp61_121 = OPcp61_120+qd[21]*(OMcp61_220*ROcp61_920-OMcp61_320*ROcp61_820)+qdd[21]*ROcp61_720;
  OPcp61_221 = OPcp61_220-qd[21]*(OMcp61_120*ROcp61_920-OMcp61_320*ROcp61_720)+qdd[21]*ROcp61_820;
  OPcp61_321 = OPcp61_320+qd[21]*(OMcp61_120*ROcp61_820-OMcp61_220*ROcp61_720)+qdd[21]*ROcp61_920;
  RLcp61_197 = s->dpt[2][46]*ROcp61_421+ROcp61_121*s->dpt[1][46]+ROcp61_720*s->dpt[3][46];
  RLcp61_297 = s->dpt[2][46]*ROcp61_521+ROcp61_221*s->dpt[1][46]+ROcp61_820*s->dpt[3][46];
  RLcp61_397 = s->dpt[2][46]*ROcp61_621+ROcp61_321*s->dpt[1][46]+ROcp61_920*s->dpt[3][46];
  ORcp61_197 = OMcp61_221*RLcp61_397-OMcp61_321*RLcp61_297;
  ORcp61_297 = -(OMcp61_121*RLcp61_397-OMcp61_321*RLcp61_197);
  ORcp61_397 = OMcp61_121*RLcp61_297-OMcp61_221*RLcp61_197;
  PxF11[1] = q[1]+RLcp61_119+RLcp61_120+RLcp61_121+RLcp61_197;
  PxF11[2] = q[2]+RLcp61_219+RLcp61_220+RLcp61_221+RLcp61_297;
  PxF11[3] = q[3]+RLcp61_319+RLcp61_320+RLcp61_321+RLcp61_397;
  RxF11[1][1] = ROcp61_121;
  RxF11[1][2] = ROcp61_221;
  RxF11[1][3] = ROcp61_321;
  RxF11[2][1] = ROcp61_421;
  RxF11[2][2] = ROcp61_521;
  RxF11[2][3] = ROcp61_621;
  RxF11[3][1] = ROcp61_720;
  RxF11[3][2] = ROcp61_820;
  RxF11[3][3] = ROcp61_920;
  VxF11[1] = qd[1]+ORcp61_119+ORcp61_120+ORcp61_121+ORcp61_197;
  VxF11[2] = qd[2]+ORcp61_219+ORcp61_220+ORcp61_221+ORcp61_297;
  VxF11[3] = qd[3]+ORcp61_319+ORcp61_320+ORcp61_321+ORcp61_397;
  OMxF11[1] = OMcp61_121;
  OMxF11[2] = OMcp61_221;
  OMxF11[3] = OMcp61_321;
  AxF11[1] = qdd[1]+OMcp61_219*ORcp61_320+OMcp61_220*ORcp61_321+OMcp61_221*ORcp61_397+OMcp61_26*ORcp61_319-OMcp61_319*
 ORcp61_220-OMcp61_320*ORcp61_221-OMcp61_321*ORcp61_297-OMcp61_36*ORcp61_219+OPcp61_219*RLcp61_320+OPcp61_220*RLcp61_321+
 OPcp61_221*RLcp61_397+OPcp61_26*RLcp61_319-OPcp61_319*RLcp61_220-OPcp61_320*RLcp61_221-OPcp61_321*RLcp61_297-OPcp61_36*
 RLcp61_219;
  AxF11[2] = qdd[2]-OMcp61_119*ORcp61_320-OMcp61_120*ORcp61_321-OMcp61_121*ORcp61_397-OMcp61_16*ORcp61_319+OMcp61_319*
 ORcp61_120+OMcp61_320*ORcp61_121+OMcp61_321*ORcp61_197+OMcp61_36*ORcp61_119-OPcp61_119*RLcp61_320-OPcp61_120*RLcp61_321-
 OPcp61_121*RLcp61_397-OPcp61_16*RLcp61_319+OPcp61_319*RLcp61_120+OPcp61_320*RLcp61_121+OPcp61_321*RLcp61_197+OPcp61_36*
 RLcp61_119;
  AxF11[3] = qdd[3]+OMcp61_119*ORcp61_220+OMcp61_120*ORcp61_221+OMcp61_121*ORcp61_297+OMcp61_16*ORcp61_219-OMcp61_219*
 ORcp61_120-OMcp61_220*ORcp61_121-OMcp61_221*ORcp61_197-OMcp61_26*ORcp61_119+OPcp61_119*RLcp61_220+OPcp61_120*RLcp61_221+
 OPcp61_121*RLcp61_297+OPcp61_16*RLcp61_219-OPcp61_219*RLcp61_120-OPcp61_220*RLcp61_121-OPcp61_221*RLcp61_197-OPcp61_26*
 RLcp61_119;
  OMPxF11[1] = OPcp61_121;
  OMPxF11[2] = OPcp61_221;
  OMPxF11[3] = OPcp61_321;
 
// Sensor Forces Computation 

  SWr11 = user_ExtForces(PxF11,RxF11,VxF11,OMxF11,AxF11,OMPxF11,s,tsim,11);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc162 = ROcp61_121*SWr11[1]+ROcp61_221*SWr11[2]+ROcp61_321*SWr11[3];
  xfrc262 = ROcp61_421*SWr11[1]+ROcp61_521*SWr11[2]+ROcp61_621*SWr11[3];
  xfrc362 = ROcp61_720*SWr11[1]+ROcp61_820*SWr11[2]+ROcp61_920*SWr11[3];
  frc[1][21] = s->frc[1][21]+xfrc162;
  frc[2][21] = s->frc[2][21]+xfrc262;
  frc[3][21] = s->frc[3][21]+xfrc362;
  xtrq162 = ROcp61_121*SWr11[4]+ROcp61_221*SWr11[5]+ROcp61_321*SWr11[6];
  xtrq262 = ROcp61_421*SWr11[4]+ROcp61_521*SWr11[5]+ROcp61_621*SWr11[6];
  xtrq362 = ROcp61_720*SWr11[4]+ROcp61_820*SWr11[5]+ROcp61_920*SWr11[6];
  trq[1][21] = s->trq[1][21]+xtrq162-xfrc262*(SWr11[9]-s->l[3][21])+xfrc362*(SWr11[8]-s->l[2][21]);
  trq[2][21] = s->trq[2][21]+xtrq262+xfrc162*(SWr11[9]-s->l[3][21])-xfrc362*(SWr11[7]-s->l[1][21]);
  trq[3][21] = s->trq[3][21]+xtrq362-xfrc162*(SWr11[8]-s->l[2][21])+xfrc262*(SWr11[7]-s->l[1][21]);

// = = Block_0_0_1_12_0_1 = = 
 
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
  OMcp62_26 = OMcp62_25+qd[6]*ROcp62_85;
  OMcp62_36 = OMcp62_35+qd[6]*ROcp62_95;
  OPcp62_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp62_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp62_95-OMcp62_35*S5)-qdd[5]*C4-qdd[6]*ROcp62_85);
  OPcp62_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp62_85-OMcp62_25*S5)+qdd[5]*S4+qdd[6]*ROcp62_95;

// = = Block_0_0_1_12_0_4 = = 
 
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
  OMcp62_119 = OMcp62_16+qd[19]*ROcp62_16;
  OMcp62_219 = OMcp62_26+qd[19]*ROcp62_26;
  OMcp62_319 = OMcp62_36+qd[19]*ROcp62_36;
  ORcp62_119 = OMcp62_26*RLcp62_319-OMcp62_36*RLcp62_219;
  ORcp62_219 = -(OMcp62_16*RLcp62_319-OMcp62_36*RLcp62_119);
  ORcp62_319 = OMcp62_16*RLcp62_219-OMcp62_26*RLcp62_119;
  OPcp62_119 = OPcp62_16+qd[19]*(OMcp62_26*ROcp62_36-OMcp62_36*ROcp62_26)+qdd[19]*ROcp62_16;
  OPcp62_219 = OPcp62_26-qd[19]*(OMcp62_16*ROcp62_36-OMcp62_36*ROcp62_16)+qdd[19]*ROcp62_26;
  OPcp62_319 = OPcp62_36+qd[19]*(OMcp62_16*ROcp62_26-OMcp62_26*ROcp62_16)+qdd[19]*ROcp62_36;
  RLcp62_120 = s->dpt[1][38]*ROcp62_16+s->dpt[2][38]*ROcp62_419+s->dpt[3][38]*ROcp62_719;
  RLcp62_220 = s->dpt[1][38]*ROcp62_26+s->dpt[2][38]*ROcp62_519+s->dpt[3][38]*ROcp62_819;
  RLcp62_320 = s->dpt[1][38]*ROcp62_36+s->dpt[2][38]*ROcp62_619+s->dpt[3][38]*ROcp62_919;
  OMcp62_120 = OMcp62_119+qd[20]*ROcp62_419;
  OMcp62_220 = OMcp62_219+qd[20]*ROcp62_519;
  OMcp62_320 = OMcp62_319+qd[20]*ROcp62_619;
  ORcp62_120 = OMcp62_219*RLcp62_320-OMcp62_319*RLcp62_220;
  ORcp62_220 = -(OMcp62_119*RLcp62_320-OMcp62_319*RLcp62_120);
  ORcp62_320 = OMcp62_119*RLcp62_220-OMcp62_219*RLcp62_120;
  OPcp62_120 = OPcp62_119+qd[20]*(OMcp62_219*ROcp62_619-OMcp62_319*ROcp62_519)+qdd[20]*ROcp62_419;
  OPcp62_220 = OPcp62_219-qd[20]*(OMcp62_119*ROcp62_619-OMcp62_319*ROcp62_419)+qdd[20]*ROcp62_519;
  OPcp62_320 = OPcp62_319+qd[20]*(OMcp62_119*ROcp62_519-OMcp62_219*ROcp62_419)+qdd[20]*ROcp62_619;
  RLcp62_121 = s->dpt[1][40]*ROcp62_120+s->dpt[2][40]*ROcp62_419+ROcp62_720*s->dpt[3][40];
  RLcp62_221 = s->dpt[1][40]*ROcp62_220+s->dpt[2][40]*ROcp62_519+ROcp62_820*s->dpt[3][40];
  RLcp62_321 = s->dpt[1][40]*ROcp62_320+s->dpt[2][40]*ROcp62_619+ROcp62_920*s->dpt[3][40];
  OMcp62_121 = OMcp62_120+qd[21]*ROcp62_720;
  OMcp62_221 = OMcp62_220+qd[21]*ROcp62_820;
  OMcp62_321 = OMcp62_320+qd[21]*ROcp62_920;
  ORcp62_121 = OMcp62_220*RLcp62_321-OMcp62_320*RLcp62_221;
  ORcp62_221 = -(OMcp62_120*RLcp62_321-OMcp62_320*RLcp62_121);
  ORcp62_321 = OMcp62_120*RLcp62_221-OMcp62_220*RLcp62_121;
  OPcp62_121 = OPcp62_120+qd[21]*(OMcp62_220*ROcp62_920-OMcp62_320*ROcp62_820)+qdd[21]*ROcp62_720;
  OPcp62_221 = OPcp62_220-qd[21]*(OMcp62_120*ROcp62_920-OMcp62_320*ROcp62_720)+qdd[21]*ROcp62_820;
  OPcp62_321 = OPcp62_320+qd[21]*(OMcp62_120*ROcp62_820-OMcp62_220*ROcp62_720)+qdd[21]*ROcp62_920;

// = = Block_0_0_1_12_0_5 = = 
 
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
  OMcp62_122 = OMcp62_121+qd[22]*ROcp62_421;
  OMcp62_222 = OMcp62_221+qd[22]*ROcp62_521;
  OMcp62_322 = OMcp62_321+qd[22]*ROcp62_621;
  ORcp62_122 = OMcp62_221*RLcp62_322-OMcp62_321*RLcp62_222;
  ORcp62_222 = -(OMcp62_121*RLcp62_322-OMcp62_321*RLcp62_122);
  ORcp62_322 = OMcp62_121*RLcp62_222-OMcp62_221*RLcp62_122;
  OPcp62_122 = OPcp62_121+qd[22]*(OMcp62_221*ROcp62_621-OMcp62_321*ROcp62_521)+qdd[22]*ROcp62_421;
  OPcp62_222 = OPcp62_221-qd[22]*(OMcp62_121*ROcp62_621-OMcp62_321*ROcp62_421)+qdd[22]*ROcp62_521;
  OPcp62_322 = OPcp62_321+qd[22]*(OMcp62_121*ROcp62_521-OMcp62_221*ROcp62_421)+qdd[22]*ROcp62_621;
  RLcp62_123 = s->dpt[1][48]*ROcp62_122+s->dpt[3][48]*ROcp62_722+ROcp62_421*s->dpt[2][48];
  RLcp62_223 = s->dpt[1][48]*ROcp62_222+s->dpt[3][48]*ROcp62_822+ROcp62_521*s->dpt[2][48];
  RLcp62_323 = s->dpt[1][48]*ROcp62_322+s->dpt[3][48]*ROcp62_922+ROcp62_621*s->dpt[2][48];
  OMcp62_123 = OMcp62_122+qd[23]*ROcp62_122;
  OMcp62_223 = OMcp62_222+qd[23]*ROcp62_222;
  OMcp62_323 = OMcp62_322+qd[23]*ROcp62_322;
  ORcp62_123 = OMcp62_222*RLcp62_323-OMcp62_322*RLcp62_223;
  ORcp62_223 = -(OMcp62_122*RLcp62_323-OMcp62_322*RLcp62_123);
  ORcp62_323 = OMcp62_122*RLcp62_223-OMcp62_222*RLcp62_123;
  OPcp62_123 = OPcp62_122+qd[23]*(OMcp62_222*ROcp62_322-OMcp62_322*ROcp62_222)+qdd[23]*ROcp62_122;
  OPcp62_223 = OPcp62_222-qd[23]*(OMcp62_122*ROcp62_322-OMcp62_322*ROcp62_122)+qdd[23]*ROcp62_222;
  OPcp62_323 = OPcp62_322+qd[23]*(OMcp62_122*ROcp62_222-OMcp62_222*ROcp62_122)+qdd[23]*ROcp62_322;
  RLcp62_124 = s->dpt[1][50]*ROcp62_122+s->dpt[3][50]*ROcp62_723+ROcp62_423*s->dpt[2][50];
  RLcp62_224 = s->dpt[1][50]*ROcp62_222+s->dpt[3][50]*ROcp62_823+ROcp62_523*s->dpt[2][50];
  RLcp62_324 = s->dpt[1][50]*ROcp62_322+s->dpt[3][50]*ROcp62_923+ROcp62_623*s->dpt[2][50];
  OMcp62_124 = OMcp62_123+qd[24]*ROcp62_423;
  OMcp62_224 = OMcp62_223+qd[24]*ROcp62_523;
  OMcp62_324 = OMcp62_323+qd[24]*ROcp62_623;
  ORcp62_124 = OMcp62_223*RLcp62_324-OMcp62_323*RLcp62_224;
  ORcp62_224 = -(OMcp62_123*RLcp62_324-OMcp62_323*RLcp62_124);
  ORcp62_324 = OMcp62_123*RLcp62_224-OMcp62_223*RLcp62_124;
  OPcp62_124 = OPcp62_123+qd[24]*(OMcp62_223*ROcp62_623-OMcp62_323*ROcp62_523)+qdd[24]*ROcp62_423;
  OPcp62_224 = OPcp62_223-qd[24]*(OMcp62_123*ROcp62_623-OMcp62_323*ROcp62_423)+qdd[24]*ROcp62_523;
  OPcp62_324 = OPcp62_323+qd[24]*(OMcp62_123*ROcp62_523-OMcp62_223*ROcp62_423)+qdd[24]*ROcp62_623;
  RLcp62_125 = s->dpt[1][52]*ROcp62_124+s->dpt[3][52]*ROcp62_724+ROcp62_423*s->dpt[2][52];
  RLcp62_225 = s->dpt[1][52]*ROcp62_224+s->dpt[3][52]*ROcp62_824+ROcp62_523*s->dpt[2][52];
  RLcp62_325 = s->dpt[1][52]*ROcp62_324+s->dpt[3][52]*ROcp62_924+ROcp62_623*s->dpt[2][52];
  OMcp62_125 = OMcp62_124+qd[25]*ROcp62_724;
  OMcp62_225 = OMcp62_224+qd[25]*ROcp62_824;
  OMcp62_325 = OMcp62_324+qd[25]*ROcp62_924;
  ORcp62_125 = OMcp62_224*RLcp62_325-OMcp62_324*RLcp62_225;
  ORcp62_225 = -(OMcp62_124*RLcp62_325-OMcp62_324*RLcp62_125);
  ORcp62_325 = OMcp62_124*RLcp62_225-OMcp62_224*RLcp62_125;
  OPcp62_125 = OPcp62_124+qd[25]*(OMcp62_224*ROcp62_924-OMcp62_324*ROcp62_824)+qdd[25]*ROcp62_724;
  OPcp62_225 = OPcp62_224-qd[25]*(OMcp62_124*ROcp62_924-OMcp62_324*ROcp62_724)+qdd[25]*ROcp62_824;
  OPcp62_325 = OPcp62_324+qd[25]*(OMcp62_124*ROcp62_824-OMcp62_224*ROcp62_724)+qdd[25]*ROcp62_924;
  RLcp62_126 = s->dpt[2][54]*ROcp62_425+s->dpt[3][54]*ROcp62_724+ROcp62_125*s->dpt[1][54];
  RLcp62_226 = s->dpt[2][54]*ROcp62_525+s->dpt[3][54]*ROcp62_824+ROcp62_225*s->dpt[1][54];
  RLcp62_326 = s->dpt[2][54]*ROcp62_625+s->dpt[3][54]*ROcp62_924+ROcp62_325*s->dpt[1][54];
  OMcp62_126 = OMcp62_125+qd[26]*ROcp62_425;
  OMcp62_226 = OMcp62_225+qd[26]*ROcp62_525;
  OMcp62_326 = OMcp62_325+qd[26]*ROcp62_625;
  ORcp62_126 = OMcp62_225*RLcp62_326-OMcp62_325*RLcp62_226;
  ORcp62_226 = -(OMcp62_125*RLcp62_326-OMcp62_325*RLcp62_126);
  ORcp62_326 = OMcp62_125*RLcp62_226-OMcp62_225*RLcp62_126;
  OPcp62_126 = OPcp62_125+qd[26]*(OMcp62_225*ROcp62_625-OMcp62_325*ROcp62_525)+qdd[26]*ROcp62_425;
  OPcp62_226 = OPcp62_225-qd[26]*(OMcp62_125*ROcp62_625-OMcp62_325*ROcp62_425)+qdd[26]*ROcp62_525;
  OPcp62_326 = OPcp62_325+qd[26]*(OMcp62_125*ROcp62_525-OMcp62_225*ROcp62_425)+qdd[26]*ROcp62_625;
  RLcp62_127 = s->dpt[1][55]*ROcp62_126+s->dpt[3][55]*ROcp62_726+ROcp62_425*s->dpt[2][55];
  RLcp62_227 = s->dpt[1][55]*ROcp62_226+s->dpt[3][55]*ROcp62_826+ROcp62_525*s->dpt[2][55];
  RLcp62_327 = s->dpt[1][55]*ROcp62_326+s->dpt[3][55]*ROcp62_926+ROcp62_625*s->dpt[2][55];
  OMcp62_127 = OMcp62_126+qd[27]*ROcp62_726;
  OMcp62_227 = OMcp62_226+qd[27]*ROcp62_826;
  OMcp62_327 = OMcp62_326+qd[27]*ROcp62_926;
  ORcp62_127 = OMcp62_226*RLcp62_327-OMcp62_326*RLcp62_227;
  ORcp62_227 = -(OMcp62_126*RLcp62_327-OMcp62_326*RLcp62_127);
  ORcp62_327 = OMcp62_126*RLcp62_227-OMcp62_226*RLcp62_127;
  OPcp62_127 = OPcp62_126+qd[27]*(OMcp62_226*ROcp62_926-OMcp62_326*ROcp62_826)+qdd[27]*ROcp62_726;
  OPcp62_227 = OPcp62_226-qd[27]*(OMcp62_126*ROcp62_926-OMcp62_326*ROcp62_726)+qdd[27]*ROcp62_826;
  OPcp62_327 = OPcp62_326+qd[27]*(OMcp62_126*ROcp62_826-OMcp62_226*ROcp62_726)+qdd[27]*ROcp62_926;
  RLcp62_128 = s->dpt[1][56]*ROcp62_127+s->dpt[2][56]*ROcp62_427+s->dpt[3][56]*ROcp62_726;
  RLcp62_228 = s->dpt[1][56]*ROcp62_227+s->dpt[2][56]*ROcp62_527+s->dpt[3][56]*ROcp62_826;
  RLcp62_328 = s->dpt[1][56]*ROcp62_327+s->dpt[2][56]*ROcp62_627+s->dpt[3][56]*ROcp62_926;
  OMcp62_128 = OMcp62_127+qd[28]*ROcp62_127;
  OMcp62_228 = OMcp62_227+qd[28]*ROcp62_227;
  OMcp62_328 = OMcp62_327+qd[28]*ROcp62_327;
  ORcp62_128 = OMcp62_227*RLcp62_328-OMcp62_327*RLcp62_228;
  ORcp62_228 = -(OMcp62_127*RLcp62_328-OMcp62_327*RLcp62_128);
  ORcp62_328 = OMcp62_127*RLcp62_228-OMcp62_227*RLcp62_128;
  OPcp62_128 = OPcp62_127+qd[28]*(OMcp62_227*ROcp62_327-OMcp62_327*ROcp62_227)+qdd[28]*ROcp62_127;
  OPcp62_228 = OPcp62_227-qd[28]*(OMcp62_127*ROcp62_327-OMcp62_327*ROcp62_127)+qdd[28]*ROcp62_227;
  OPcp62_328 = OPcp62_327+qd[28]*(OMcp62_127*ROcp62_227-OMcp62_227*ROcp62_127)+qdd[28]*ROcp62_327;
  RLcp62_198 = s->dpt[1][57]*ROcp62_127+s->dpt[3][57]*ROcp62_728+ROcp62_428*s->dpt[2][57];
  RLcp62_298 = s->dpt[1][57]*ROcp62_227+s->dpt[3][57]*ROcp62_828+ROcp62_528*s->dpt[2][57];
  RLcp62_398 = s->dpt[1][57]*ROcp62_327+s->dpt[3][57]*ROcp62_928+ROcp62_628*s->dpt[2][57];
  ORcp62_198 = OMcp62_228*RLcp62_398-OMcp62_328*RLcp62_298;
  ORcp62_298 = -(OMcp62_128*RLcp62_398-OMcp62_328*RLcp62_198);
  ORcp62_398 = OMcp62_128*RLcp62_298-OMcp62_228*RLcp62_198;
  PxF12[1] = q[1]+RLcp62_119+RLcp62_120+RLcp62_121+RLcp62_122+RLcp62_123+RLcp62_124+RLcp62_125+RLcp62_126+RLcp62_127+
 RLcp62_128+RLcp62_198;
  PxF12[2] = q[2]+RLcp62_219+RLcp62_220+RLcp62_221+RLcp62_222+RLcp62_223+RLcp62_224+RLcp62_225+RLcp62_226+RLcp62_227+
 RLcp62_228+RLcp62_298;
  PxF12[3] = q[3]+RLcp62_319+RLcp62_320+RLcp62_321+RLcp62_322+RLcp62_323+RLcp62_324+RLcp62_325+RLcp62_326+RLcp62_327+
 RLcp62_328+RLcp62_398;
  RxF12[1][1] = ROcp62_127;
  RxF12[1][2] = ROcp62_227;
  RxF12[1][3] = ROcp62_327;
  RxF12[2][1] = ROcp62_428;
  RxF12[2][2] = ROcp62_528;
  RxF12[2][3] = ROcp62_628;
  RxF12[3][1] = ROcp62_728;
  RxF12[3][2] = ROcp62_828;
  RxF12[3][3] = ROcp62_928;
  VxF12[1] = qd[1]+ORcp62_119+ORcp62_120+ORcp62_121+ORcp62_122+ORcp62_123+ORcp62_124+ORcp62_125+ORcp62_126+ORcp62_127+
 ORcp62_128+ORcp62_198;
  VxF12[2] = qd[2]+ORcp62_219+ORcp62_220+ORcp62_221+ORcp62_222+ORcp62_223+ORcp62_224+ORcp62_225+ORcp62_226+ORcp62_227+
 ORcp62_228+ORcp62_298;
  VxF12[3] = qd[3]+ORcp62_319+ORcp62_320+ORcp62_321+ORcp62_322+ORcp62_323+ORcp62_324+ORcp62_325+ORcp62_326+ORcp62_327+
 ORcp62_328+ORcp62_398;
  OMxF12[1] = OMcp62_128;
  OMxF12[2] = OMcp62_228;
  OMxF12[3] = OMcp62_328;
  AxF12[1] = qdd[1]+OMcp62_219*ORcp62_320+OMcp62_220*ORcp62_321+OMcp62_221*ORcp62_322+OMcp62_222*ORcp62_323+OMcp62_223*
 ORcp62_324+OMcp62_224*ORcp62_325+OMcp62_225*ORcp62_326+OMcp62_226*ORcp62_327+OMcp62_227*ORcp62_328+OMcp62_228*ORcp62_398+
 OMcp62_26*ORcp62_319-OMcp62_319*ORcp62_220-OMcp62_320*ORcp62_221-OMcp62_321*ORcp62_222-OMcp62_322*ORcp62_223-OMcp62_323*
 ORcp62_224-OMcp62_324*ORcp62_225-OMcp62_325*ORcp62_226-OMcp62_326*ORcp62_227-OMcp62_327*ORcp62_228-OMcp62_328*ORcp62_298-
 OMcp62_36*ORcp62_219+OPcp62_219*RLcp62_320+OPcp62_220*RLcp62_321+OPcp62_221*RLcp62_322+OPcp62_222*RLcp62_323+OPcp62_223*
 RLcp62_324+OPcp62_224*RLcp62_325+OPcp62_225*RLcp62_326+OPcp62_226*RLcp62_327+OPcp62_227*RLcp62_328+OPcp62_228*RLcp62_398+
 OPcp62_26*RLcp62_319-OPcp62_319*RLcp62_220-OPcp62_320*RLcp62_221-OPcp62_321*RLcp62_222-OPcp62_322*RLcp62_223-OPcp62_323*
 RLcp62_224-OPcp62_324*RLcp62_225-OPcp62_325*RLcp62_226-OPcp62_326*RLcp62_227-OPcp62_327*RLcp62_228-OPcp62_328*RLcp62_298-
 OPcp62_36*RLcp62_219;
  AxF12[2] = qdd[2]-OMcp62_119*ORcp62_320-OMcp62_120*ORcp62_321-OMcp62_121*ORcp62_322-OMcp62_122*ORcp62_323-OMcp62_123*
 ORcp62_324-OMcp62_124*ORcp62_325-OMcp62_125*ORcp62_326-OMcp62_126*ORcp62_327-OMcp62_127*ORcp62_328-OMcp62_128*ORcp62_398-
 OMcp62_16*ORcp62_319+OMcp62_319*ORcp62_120+OMcp62_320*ORcp62_121+OMcp62_321*ORcp62_122+OMcp62_322*ORcp62_123+OMcp62_323*
 ORcp62_124+OMcp62_324*ORcp62_125+OMcp62_325*ORcp62_126+OMcp62_326*ORcp62_127+OMcp62_327*ORcp62_128+OMcp62_328*ORcp62_198+
 OMcp62_36*ORcp62_119-OPcp62_119*RLcp62_320-OPcp62_120*RLcp62_321-OPcp62_121*RLcp62_322-OPcp62_122*RLcp62_323-OPcp62_123*
 RLcp62_324-OPcp62_124*RLcp62_325-OPcp62_125*RLcp62_326-OPcp62_126*RLcp62_327-OPcp62_127*RLcp62_328-OPcp62_128*RLcp62_398-
 OPcp62_16*RLcp62_319+OPcp62_319*RLcp62_120+OPcp62_320*RLcp62_121+OPcp62_321*RLcp62_122+OPcp62_322*RLcp62_123+OPcp62_323*
 RLcp62_124+OPcp62_324*RLcp62_125+OPcp62_325*RLcp62_126+OPcp62_326*RLcp62_127+OPcp62_327*RLcp62_128+OPcp62_328*RLcp62_198+
 OPcp62_36*RLcp62_119;
  AxF12[3] = qdd[3]+OMcp62_119*ORcp62_220+OMcp62_120*ORcp62_221+OMcp62_121*ORcp62_222+OMcp62_122*ORcp62_223+OMcp62_123*
 ORcp62_224+OMcp62_124*ORcp62_225+OMcp62_125*ORcp62_226+OMcp62_126*ORcp62_227+OMcp62_127*ORcp62_228+OMcp62_128*ORcp62_298+
 OMcp62_16*ORcp62_219-OMcp62_219*ORcp62_120-OMcp62_220*ORcp62_121-OMcp62_221*ORcp62_122-OMcp62_222*ORcp62_123-OMcp62_223*
 ORcp62_124-OMcp62_224*ORcp62_125-OMcp62_225*ORcp62_126-OMcp62_226*ORcp62_127-OMcp62_227*ORcp62_128-OMcp62_228*ORcp62_198-
 OMcp62_26*ORcp62_119+OPcp62_119*RLcp62_220+OPcp62_120*RLcp62_221+OPcp62_121*RLcp62_222+OPcp62_122*RLcp62_223+OPcp62_123*
 RLcp62_224+OPcp62_124*RLcp62_225+OPcp62_125*RLcp62_226+OPcp62_126*RLcp62_227+OPcp62_127*RLcp62_228+OPcp62_128*RLcp62_298+
 OPcp62_16*RLcp62_219-OPcp62_219*RLcp62_120-OPcp62_220*RLcp62_121-OPcp62_221*RLcp62_122-OPcp62_222*RLcp62_123-OPcp62_223*
 RLcp62_124-OPcp62_224*RLcp62_125-OPcp62_225*RLcp62_126-OPcp62_226*RLcp62_127-OPcp62_227*RLcp62_128-OPcp62_228*RLcp62_198-
 OPcp62_26*RLcp62_119;
  OMPxF12[1] = OPcp62_128;
  OMPxF12[2] = OPcp62_228;
  OMPxF12[3] = OPcp62_328;
 
// Sensor Forces Computation 

  SWr12 = user_ExtForces(PxF12,RxF12,VxF12,OMxF12,AxF12,OMPxF12,s,tsim,12);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc163 = ROcp62_127*SWr12[1]+ROcp62_227*SWr12[2]+ROcp62_327*SWr12[3];
  xfrc263 = ROcp62_428*SWr12[1]+ROcp62_528*SWr12[2]+ROcp62_628*SWr12[3];
  xfrc363 = ROcp62_728*SWr12[1]+ROcp62_828*SWr12[2]+ROcp62_928*SWr12[3];
  frc[1][28] = s->frc[1][28]+xfrc163;
  frc[2][28] = s->frc[2][28]+xfrc263;
  frc[3][28] = s->frc[3][28]+xfrc363;
  xtrq163 = ROcp62_127*SWr12[4]+ROcp62_227*SWr12[5]+ROcp62_327*SWr12[6];
  xtrq263 = ROcp62_428*SWr12[4]+ROcp62_528*SWr12[5]+ROcp62_628*SWr12[6];
  xtrq363 = ROcp62_728*SWr12[4]+ROcp62_828*SWr12[5]+ROcp62_928*SWr12[6];
  trq[1][28] = s->trq[1][28]+xtrq163+xfrc263*(s->l[3][28]-SWr12[9])-xfrc363*(s->l[2][28]-SWr12[8]);
  trq[2][28] = s->trq[2][28]+xtrq263-xfrc163*(s->l[3][28]-SWr12[9])+xfrc363*(s->l[1][28]-SWr12[7]);
  trq[3][28] = s->trq[3][28]+xtrq363+xfrc163*(s->l[2][28]-SWr12[8])-xfrc263*(s->l[1][28]-SWr12[7]);

// = = Block_0_0_1_13_0_1 = = 
 
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
  OMcp63_26 = OMcp63_25+qd[6]*ROcp63_85;
  OMcp63_36 = OMcp63_35+qd[6]*ROcp63_95;
  OPcp63_16 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;
  OPcp63_26 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp63_95-OMcp63_35*S5)-qdd[5]*C4-qdd[6]*ROcp63_85);
  OPcp63_36 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp63_85-OMcp63_25*S5)+qdd[5]*S4+qdd[6]*ROcp63_95;

// = = Block_0_0_1_13_0_4 = = 
 
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
  OMcp63_119 = OMcp63_16+qd[19]*ROcp63_16;
  OMcp63_219 = OMcp63_26+qd[19]*ROcp63_26;
  OMcp63_319 = OMcp63_36+qd[19]*ROcp63_36;
  ORcp63_119 = OMcp63_26*RLcp63_319-OMcp63_36*RLcp63_219;
  ORcp63_219 = -(OMcp63_16*RLcp63_319-OMcp63_36*RLcp63_119);
  ORcp63_319 = OMcp63_16*RLcp63_219-OMcp63_26*RLcp63_119;
  OPcp63_119 = OPcp63_16+qd[19]*(OMcp63_26*ROcp63_36-OMcp63_36*ROcp63_26)+qdd[19]*ROcp63_16;
  OPcp63_219 = OPcp63_26-qd[19]*(OMcp63_16*ROcp63_36-OMcp63_36*ROcp63_16)+qdd[19]*ROcp63_26;
  OPcp63_319 = OPcp63_36+qd[19]*(OMcp63_16*ROcp63_26-OMcp63_26*ROcp63_16)+qdd[19]*ROcp63_36;
  RLcp63_120 = s->dpt[1][38]*ROcp63_16+s->dpt[2][38]*ROcp63_419+s->dpt[3][38]*ROcp63_719;
  RLcp63_220 = s->dpt[1][38]*ROcp63_26+s->dpt[2][38]*ROcp63_519+s->dpt[3][38]*ROcp63_819;
  RLcp63_320 = s->dpt[1][38]*ROcp63_36+s->dpt[2][38]*ROcp63_619+s->dpt[3][38]*ROcp63_919;
  OMcp63_120 = OMcp63_119+qd[20]*ROcp63_419;
  OMcp63_220 = OMcp63_219+qd[20]*ROcp63_519;
  OMcp63_320 = OMcp63_319+qd[20]*ROcp63_619;
  ORcp63_120 = OMcp63_219*RLcp63_320-OMcp63_319*RLcp63_220;
  ORcp63_220 = -(OMcp63_119*RLcp63_320-OMcp63_319*RLcp63_120);
  ORcp63_320 = OMcp63_119*RLcp63_220-OMcp63_219*RLcp63_120;
  OPcp63_120 = OPcp63_119+qd[20]*(OMcp63_219*ROcp63_619-OMcp63_319*ROcp63_519)+qdd[20]*ROcp63_419;
  OPcp63_220 = OPcp63_219-qd[20]*(OMcp63_119*ROcp63_619-OMcp63_319*ROcp63_419)+qdd[20]*ROcp63_519;
  OPcp63_320 = OPcp63_319+qd[20]*(OMcp63_119*ROcp63_519-OMcp63_219*ROcp63_419)+qdd[20]*ROcp63_619;
  RLcp63_121 = s->dpt[1][40]*ROcp63_120+s->dpt[2][40]*ROcp63_419+ROcp63_720*s->dpt[3][40];
  RLcp63_221 = s->dpt[1][40]*ROcp63_220+s->dpt[2][40]*ROcp63_519+ROcp63_820*s->dpt[3][40];
  RLcp63_321 = s->dpt[1][40]*ROcp63_320+s->dpt[2][40]*ROcp63_619+ROcp63_920*s->dpt[3][40];
  OMcp63_121 = OMcp63_120+qd[21]*ROcp63_720;
  OMcp63_221 = OMcp63_220+qd[21]*ROcp63_820;
  OMcp63_321 = OMcp63_320+qd[21]*ROcp63_920;
  ORcp63_121 = OMcp63_220*RLcp63_321-OMcp63_320*RLcp63_221;
  ORcp63_221 = -(OMcp63_120*RLcp63_321-OMcp63_320*RLcp63_121);
  ORcp63_321 = OMcp63_120*RLcp63_221-OMcp63_220*RLcp63_121;
  OPcp63_121 = OPcp63_120+qd[21]*(OMcp63_220*ROcp63_920-OMcp63_320*ROcp63_820)+qdd[21]*ROcp63_720;
  OPcp63_221 = OPcp63_220-qd[21]*(OMcp63_120*ROcp63_920-OMcp63_320*ROcp63_720)+qdd[21]*ROcp63_820;
  OPcp63_321 = OPcp63_320+qd[21]*(OMcp63_120*ROcp63_820-OMcp63_220*ROcp63_720)+qdd[21]*ROcp63_920;

// = = Block_0_0_1_13_0_6 = = 
 
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
  OMcp63_129 = OMcp63_121+qd[29]*ROcp63_421;
  OMcp63_229 = OMcp63_221+qd[29]*ROcp63_521;
  OMcp63_329 = OMcp63_321+qd[29]*ROcp63_621;
  ORcp63_129 = OMcp63_221*RLcp63_329-OMcp63_321*RLcp63_229;
  ORcp63_229 = -(OMcp63_121*RLcp63_329-OMcp63_321*RLcp63_129);
  ORcp63_329 = OMcp63_121*RLcp63_229-OMcp63_221*RLcp63_129;
  OPcp63_129 = OPcp63_121+qd[29]*(OMcp63_221*ROcp63_621-OMcp63_321*ROcp63_521)+qdd[29]*ROcp63_421;
  OPcp63_229 = OPcp63_221-qd[29]*(OMcp63_121*ROcp63_621-OMcp63_321*ROcp63_421)+qdd[29]*ROcp63_521;
  OPcp63_329 = OPcp63_321+qd[29]*(OMcp63_121*ROcp63_521-OMcp63_221*ROcp63_421)+qdd[29]*ROcp63_621;
  RLcp63_130 = s->dpt[1][59]*ROcp63_129+s->dpt[3][59]*ROcp63_729+ROcp63_421*s->dpt[2][59];
  RLcp63_230 = s->dpt[1][59]*ROcp63_229+s->dpt[3][59]*ROcp63_829+ROcp63_521*s->dpt[2][59];
  RLcp63_330 = s->dpt[1][59]*ROcp63_329+s->dpt[3][59]*ROcp63_929+ROcp63_621*s->dpt[2][59];
  OMcp63_130 = OMcp63_129+qd[30]*ROcp63_129;
  OMcp63_230 = OMcp63_229+qd[30]*ROcp63_229;
  OMcp63_330 = OMcp63_329+qd[30]*ROcp63_329;
  ORcp63_130 = OMcp63_229*RLcp63_330-OMcp63_329*RLcp63_230;
  ORcp63_230 = -(OMcp63_129*RLcp63_330-OMcp63_329*RLcp63_130);
  ORcp63_330 = OMcp63_129*RLcp63_230-OMcp63_229*RLcp63_130;
  OPcp63_130 = OPcp63_129+qd[30]*(OMcp63_229*ROcp63_329-OMcp63_329*ROcp63_229)+qdd[30]*ROcp63_129;
  OPcp63_230 = OPcp63_229-qd[30]*(OMcp63_129*ROcp63_329-OMcp63_329*ROcp63_129)+qdd[30]*ROcp63_229;
  OPcp63_330 = OPcp63_329+qd[30]*(OMcp63_129*ROcp63_229-OMcp63_229*ROcp63_129)+qdd[30]*ROcp63_329;
  RLcp63_131 = s->dpt[1][61]*ROcp63_129+s->dpt[3][61]*ROcp63_730+ROcp63_430*s->dpt[2][61];
  RLcp63_231 = s->dpt[1][61]*ROcp63_229+s->dpt[3][61]*ROcp63_830+ROcp63_530*s->dpt[2][61];
  RLcp63_331 = s->dpt[1][61]*ROcp63_329+s->dpt[3][61]*ROcp63_930+ROcp63_630*s->dpt[2][61];
  OMcp63_131 = OMcp63_130+qd[31]*ROcp63_430;
  OMcp63_231 = OMcp63_230+qd[31]*ROcp63_530;
  OMcp63_331 = OMcp63_330+qd[31]*ROcp63_630;
  ORcp63_131 = OMcp63_230*RLcp63_331-OMcp63_330*RLcp63_231;
  ORcp63_231 = -(OMcp63_130*RLcp63_331-OMcp63_330*RLcp63_131);
  ORcp63_331 = OMcp63_130*RLcp63_231-OMcp63_230*RLcp63_131;
  OPcp63_131 = OPcp63_130+qd[31]*(OMcp63_230*ROcp63_630-OMcp63_330*ROcp63_530)+qdd[31]*ROcp63_430;
  OPcp63_231 = OPcp63_230-qd[31]*(OMcp63_130*ROcp63_630-OMcp63_330*ROcp63_430)+qdd[31]*ROcp63_530;
  OPcp63_331 = OPcp63_330+qd[31]*(OMcp63_130*ROcp63_530-OMcp63_230*ROcp63_430)+qdd[31]*ROcp63_630;
  RLcp63_132 = s->dpt[3][63]*ROcp63_731+ROcp63_131*s->dpt[1][63]+ROcp63_430*s->dpt[2][63];
  RLcp63_232 = s->dpt[3][63]*ROcp63_831+ROcp63_231*s->dpt[1][63]+ROcp63_530*s->dpt[2][63];
  RLcp63_332 = s->dpt[3][63]*ROcp63_931+ROcp63_331*s->dpt[1][63]+ROcp63_630*s->dpt[2][63];
  OMcp63_132 = OMcp63_131+qd[32]*ROcp63_731;
  OMcp63_232 = OMcp63_231+qd[32]*ROcp63_831;
  OMcp63_332 = OMcp63_331+qd[32]*ROcp63_931;
  ORcp63_132 = OMcp63_231*RLcp63_332-OMcp63_331*RLcp63_232;
  ORcp63_232 = -(OMcp63_131*RLcp63_332-OMcp63_331*RLcp63_132);
  ORcp63_332 = OMcp63_131*RLcp63_232-OMcp63_231*RLcp63_132;
  OPcp63_132 = OPcp63_131+qd[32]*(OMcp63_231*ROcp63_931-OMcp63_331*ROcp63_831)+qdd[32]*ROcp63_731;
  OPcp63_232 = OPcp63_231-qd[32]*(OMcp63_131*ROcp63_931-OMcp63_331*ROcp63_731)+qdd[32]*ROcp63_831;
  OPcp63_332 = OPcp63_331+qd[32]*(OMcp63_131*ROcp63_831-OMcp63_231*ROcp63_731)+qdd[32]*ROcp63_931;
  RLcp63_133 = s->dpt[2][65]*ROcp63_432+s->dpt[3][65]*ROcp63_731+ROcp63_132*s->dpt[1][65];
  RLcp63_233 = s->dpt[2][65]*ROcp63_532+s->dpt[3][65]*ROcp63_831+ROcp63_232*s->dpt[1][65];
  RLcp63_333 = s->dpt[2][65]*ROcp63_632+s->dpt[3][65]*ROcp63_931+ROcp63_332*s->dpt[1][65];
  OMcp63_133 = OMcp63_132+qd[33]*ROcp63_432;
  OMcp63_233 = OMcp63_232+qd[33]*ROcp63_532;
  OMcp63_333 = OMcp63_332+qd[33]*ROcp63_632;
  ORcp63_133 = OMcp63_232*RLcp63_333-OMcp63_332*RLcp63_233;
  ORcp63_233 = -(OMcp63_132*RLcp63_333-OMcp63_332*RLcp63_133);
  ORcp63_333 = OMcp63_132*RLcp63_233-OMcp63_232*RLcp63_133;
  OPcp63_133 = OPcp63_132+qd[33]*(OMcp63_232*ROcp63_632-OMcp63_332*ROcp63_532)+qdd[33]*ROcp63_432;
  OPcp63_233 = OPcp63_232-qd[33]*(OMcp63_132*ROcp63_632-OMcp63_332*ROcp63_432)+qdd[33]*ROcp63_532;
  OPcp63_333 = OPcp63_332+qd[33]*(OMcp63_132*ROcp63_532-OMcp63_232*ROcp63_432)+qdd[33]*ROcp63_632;
  RLcp63_134 = s->dpt[1][66]*ROcp63_133+s->dpt[3][66]*ROcp63_733+ROcp63_432*s->dpt[2][66];
  RLcp63_234 = s->dpt[1][66]*ROcp63_233+s->dpt[3][66]*ROcp63_833+ROcp63_532*s->dpt[2][66];
  RLcp63_334 = s->dpt[1][66]*ROcp63_333+s->dpt[3][66]*ROcp63_933+ROcp63_632*s->dpt[2][66];
  OMcp63_134 = OMcp63_133+qd[34]*ROcp63_733;
  OMcp63_234 = OMcp63_233+qd[34]*ROcp63_833;
  OMcp63_334 = OMcp63_333+qd[34]*ROcp63_933;
  ORcp63_134 = OMcp63_233*RLcp63_334-OMcp63_333*RLcp63_234;
  ORcp63_234 = -(OMcp63_133*RLcp63_334-OMcp63_333*RLcp63_134);
  ORcp63_334 = OMcp63_133*RLcp63_234-OMcp63_233*RLcp63_134;
  OPcp63_134 = OPcp63_133+qd[34]*(OMcp63_233*ROcp63_933-OMcp63_333*ROcp63_833)+qdd[34]*ROcp63_733;
  OPcp63_234 = OPcp63_233-qd[34]*(OMcp63_133*ROcp63_933-OMcp63_333*ROcp63_733)+qdd[34]*ROcp63_833;
  OPcp63_334 = OPcp63_333+qd[34]*(OMcp63_133*ROcp63_833-OMcp63_233*ROcp63_733)+qdd[34]*ROcp63_933;
  RLcp63_135 = s->dpt[1][67]*ROcp63_134+s->dpt[2][67]*ROcp63_434+s->dpt[3][67]*ROcp63_733;
  RLcp63_235 = s->dpt[1][67]*ROcp63_234+s->dpt[2][67]*ROcp63_534+s->dpt[3][67]*ROcp63_833;
  RLcp63_335 = s->dpt[1][67]*ROcp63_334+s->dpt[2][67]*ROcp63_634+s->dpt[3][67]*ROcp63_933;
  OMcp63_135 = OMcp63_134+qd[35]*ROcp63_134;
  OMcp63_235 = OMcp63_234+qd[35]*ROcp63_234;
  OMcp63_335 = OMcp63_334+qd[35]*ROcp63_334;
  ORcp63_135 = OMcp63_234*RLcp63_335-OMcp63_334*RLcp63_235;
  ORcp63_235 = -(OMcp63_134*RLcp63_335-OMcp63_334*RLcp63_135);
  ORcp63_335 = OMcp63_134*RLcp63_235-OMcp63_234*RLcp63_135;
  OPcp63_135 = OPcp63_134+qd[35]*(OMcp63_234*ROcp63_334-OMcp63_334*ROcp63_234)+qdd[35]*ROcp63_134;
  OPcp63_235 = OPcp63_234-qd[35]*(OMcp63_134*ROcp63_334-OMcp63_334*ROcp63_134)+qdd[35]*ROcp63_234;
  OPcp63_335 = OPcp63_334+qd[35]*(OMcp63_134*ROcp63_234-OMcp63_234*ROcp63_134)+qdd[35]*ROcp63_334;
  RLcp63_199 = s->dpt[1][68]*ROcp63_134+s->dpt[3][68]*ROcp63_735+ROcp63_435*s->dpt[2][68];
  RLcp63_299 = s->dpt[1][68]*ROcp63_234+s->dpt[3][68]*ROcp63_835+ROcp63_535*s->dpt[2][68];
  RLcp63_399 = s->dpt[1][68]*ROcp63_334+s->dpt[3][68]*ROcp63_935+ROcp63_635*s->dpt[2][68];
  ORcp63_199 = OMcp63_235*RLcp63_399-OMcp63_335*RLcp63_299;
  ORcp63_299 = -(OMcp63_135*RLcp63_399-OMcp63_335*RLcp63_199);
  ORcp63_399 = OMcp63_135*RLcp63_299-OMcp63_235*RLcp63_199;
  PxF13[1] = q[1]+RLcp63_119+RLcp63_120+RLcp63_121+RLcp63_129+RLcp63_130+RLcp63_131+RLcp63_132+RLcp63_133+RLcp63_134+
 RLcp63_135+RLcp63_199;
  PxF13[2] = q[2]+RLcp63_219+RLcp63_220+RLcp63_221+RLcp63_229+RLcp63_230+RLcp63_231+RLcp63_232+RLcp63_233+RLcp63_234+
 RLcp63_235+RLcp63_299;
  PxF13[3] = q[3]+RLcp63_319+RLcp63_320+RLcp63_321+RLcp63_329+RLcp63_330+RLcp63_331+RLcp63_332+RLcp63_333+RLcp63_334+
 RLcp63_335+RLcp63_399;
  RxF13[1][1] = ROcp63_134;
  RxF13[1][2] = ROcp63_234;
  RxF13[1][3] = ROcp63_334;
  RxF13[2][1] = ROcp63_435;
  RxF13[2][2] = ROcp63_535;
  RxF13[2][3] = ROcp63_635;
  RxF13[3][1] = ROcp63_735;
  RxF13[3][2] = ROcp63_835;
  RxF13[3][3] = ROcp63_935;
  VxF13[1] = qd[1]+ORcp63_119+ORcp63_120+ORcp63_121+ORcp63_129+ORcp63_130+ORcp63_131+ORcp63_132+ORcp63_133+ORcp63_134+
 ORcp63_135+ORcp63_199;
  VxF13[2] = qd[2]+ORcp63_219+ORcp63_220+ORcp63_221+ORcp63_229+ORcp63_230+ORcp63_231+ORcp63_232+ORcp63_233+ORcp63_234+
 ORcp63_235+ORcp63_299;
  VxF13[3] = qd[3]+ORcp63_319+ORcp63_320+ORcp63_321+ORcp63_329+ORcp63_330+ORcp63_331+ORcp63_332+ORcp63_333+ORcp63_334+
 ORcp63_335+ORcp63_399;
  OMxF13[1] = OMcp63_135;
  OMxF13[2] = OMcp63_235;
  OMxF13[3] = OMcp63_335;
  AxF13[1] = qdd[1]+OMcp63_219*ORcp63_320+OMcp63_220*ORcp63_321+OMcp63_221*ORcp63_329+OMcp63_229*ORcp63_330+OMcp63_230*
 ORcp63_331+OMcp63_231*ORcp63_332+OMcp63_232*ORcp63_333+OMcp63_233*ORcp63_334+OMcp63_234*ORcp63_335+OMcp63_235*ORcp63_399+
 OMcp63_26*ORcp63_319-OMcp63_319*ORcp63_220-OMcp63_320*ORcp63_221-OMcp63_321*ORcp63_229-OMcp63_329*ORcp63_230-OMcp63_330*
 ORcp63_231-OMcp63_331*ORcp63_232-OMcp63_332*ORcp63_233-OMcp63_333*ORcp63_234-OMcp63_334*ORcp63_235-OMcp63_335*ORcp63_299-
 OMcp63_36*ORcp63_219+OPcp63_219*RLcp63_320+OPcp63_220*RLcp63_321+OPcp63_221*RLcp63_329+OPcp63_229*RLcp63_330+OPcp63_230*
 RLcp63_331+OPcp63_231*RLcp63_332+OPcp63_232*RLcp63_333+OPcp63_233*RLcp63_334+OPcp63_234*RLcp63_335+OPcp63_235*RLcp63_399+
 OPcp63_26*RLcp63_319-OPcp63_319*RLcp63_220-OPcp63_320*RLcp63_221-OPcp63_321*RLcp63_229-OPcp63_329*RLcp63_230-OPcp63_330*
 RLcp63_231-OPcp63_331*RLcp63_232-OPcp63_332*RLcp63_233-OPcp63_333*RLcp63_234-OPcp63_334*RLcp63_235-OPcp63_335*RLcp63_299-
 OPcp63_36*RLcp63_219;
  AxF13[2] = qdd[2]-OMcp63_119*ORcp63_320-OMcp63_120*ORcp63_321-OMcp63_121*ORcp63_329-OMcp63_129*ORcp63_330-OMcp63_130*
 ORcp63_331-OMcp63_131*ORcp63_332-OMcp63_132*ORcp63_333-OMcp63_133*ORcp63_334-OMcp63_134*ORcp63_335-OMcp63_135*ORcp63_399-
 OMcp63_16*ORcp63_319+OMcp63_319*ORcp63_120+OMcp63_320*ORcp63_121+OMcp63_321*ORcp63_129+OMcp63_329*ORcp63_130+OMcp63_330*
 ORcp63_131+OMcp63_331*ORcp63_132+OMcp63_332*ORcp63_133+OMcp63_333*ORcp63_134+OMcp63_334*ORcp63_135+OMcp63_335*ORcp63_199+
 OMcp63_36*ORcp63_119-OPcp63_119*RLcp63_320-OPcp63_120*RLcp63_321-OPcp63_121*RLcp63_329-OPcp63_129*RLcp63_330-OPcp63_130*
 RLcp63_331-OPcp63_131*RLcp63_332-OPcp63_132*RLcp63_333-OPcp63_133*RLcp63_334-OPcp63_134*RLcp63_335-OPcp63_135*RLcp63_399-
 OPcp63_16*RLcp63_319+OPcp63_319*RLcp63_120+OPcp63_320*RLcp63_121+OPcp63_321*RLcp63_129+OPcp63_329*RLcp63_130+OPcp63_330*
 RLcp63_131+OPcp63_331*RLcp63_132+OPcp63_332*RLcp63_133+OPcp63_333*RLcp63_134+OPcp63_334*RLcp63_135+OPcp63_335*RLcp63_199+
 OPcp63_36*RLcp63_119;
  AxF13[3] = qdd[3]+OMcp63_119*ORcp63_220+OMcp63_120*ORcp63_221+OMcp63_121*ORcp63_229+OMcp63_129*ORcp63_230+OMcp63_130*
 ORcp63_231+OMcp63_131*ORcp63_232+OMcp63_132*ORcp63_233+OMcp63_133*ORcp63_234+OMcp63_134*ORcp63_235+OMcp63_135*ORcp63_299+
 OMcp63_16*ORcp63_219-OMcp63_219*ORcp63_120-OMcp63_220*ORcp63_121-OMcp63_221*ORcp63_129-OMcp63_229*ORcp63_130-OMcp63_230*
 ORcp63_131-OMcp63_231*ORcp63_132-OMcp63_232*ORcp63_133-OMcp63_233*ORcp63_134-OMcp63_234*ORcp63_135-OMcp63_235*ORcp63_199-
 OMcp63_26*ORcp63_119+OPcp63_119*RLcp63_220+OPcp63_120*RLcp63_221+OPcp63_121*RLcp63_229+OPcp63_129*RLcp63_230+OPcp63_130*
 RLcp63_231+OPcp63_131*RLcp63_232+OPcp63_132*RLcp63_233+OPcp63_133*RLcp63_234+OPcp63_134*RLcp63_235+OPcp63_135*RLcp63_299+
 OPcp63_16*RLcp63_219-OPcp63_219*RLcp63_120-OPcp63_220*RLcp63_121-OPcp63_221*RLcp63_129-OPcp63_229*RLcp63_130-OPcp63_230*
 RLcp63_131-OPcp63_231*RLcp63_132-OPcp63_232*RLcp63_133-OPcp63_233*RLcp63_134-OPcp63_234*RLcp63_135-OPcp63_235*RLcp63_199-
 OPcp63_26*RLcp63_119;
  OMPxF13[1] = OPcp63_135;
  OMPxF13[2] = OPcp63_235;
  OMPxF13[3] = OPcp63_335;
 
// Sensor Forces Computation 

  SWr13 = user_ExtForces(PxF13,RxF13,VxF13,OMxF13,AxF13,OMPxF13,s,tsim,13);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc164 = ROcp63_134*SWr13[1]+ROcp63_234*SWr13[2]+ROcp63_334*SWr13[3];
  xfrc264 = ROcp63_435*SWr13[1]+ROcp63_535*SWr13[2]+ROcp63_635*SWr13[3];
  xfrc364 = ROcp63_735*SWr13[1]+ROcp63_835*SWr13[2]+ROcp63_935*SWr13[3];
  frc[1][35] = s->frc[1][35]+xfrc164;
  frc[2][35] = s->frc[2][35]+xfrc264;
  frc[3][35] = s->frc[3][35]+xfrc364;
  xtrq164 = ROcp63_134*SWr13[4]+ROcp63_234*SWr13[5]+ROcp63_334*SWr13[6];
  xtrq264 = ROcp63_435*SWr13[4]+ROcp63_535*SWr13[5]+ROcp63_635*SWr13[6];
  xtrq364 = ROcp63_735*SWr13[4]+ROcp63_835*SWr13[5]+ROcp63_935*SWr13[6];
  trq[1][35] = s->trq[1][35]+xtrq164+xfrc264*(s->l[3][35]-SWr13[9])-xfrc364*(s->l[2][35]-SWr13[8]);
  trq[2][35] = s->trq[2][35]+xtrq264-xfrc164*(s->l[3][35]-SWr13[9])+xfrc364*(s->l[1][35]-SWr13[7]);
  trq[3][35] = s->trq[3][35]+xtrq364+xfrc164*(s->l[2][35]-SWr13[8])-xfrc264*(s->l[1][35]-SWr13[7]);

// = = Block_0_0_1_13_1_0 = = 
 
// Symbolic Outputs  

  frc[1][1] = s->frc[1][1];
  frc[2][1] = s->frc[2][1];
  frc[3][1] = s->frc[3][1];
  frc[1][2] = s->frc[1][2];
  frc[2][2] = s->frc[2][2];
  frc[3][2] = s->frc[3][2];
  frc[1][3] = s->frc[1][3];
  frc[2][3] = s->frc[2][3];
  frc[3][3] = s->frc[3][3];
  frc[1][4] = s->frc[1][4];
  frc[2][4] = s->frc[2][4];
  frc[3][4] = s->frc[3][4];
  frc[1][5] = s->frc[1][5];
  frc[2][5] = s->frc[2][5];
  frc[3][5] = s->frc[3][5];
  frc[1][6] = s->frc[1][6];
  frc[2][6] = s->frc[2][6];
  frc[3][6] = s->frc[3][6];
  frc[1][7] = s->frc[1][7];
  frc[2][7] = s->frc[2][7];
  frc[3][7] = s->frc[3][7];
  frc[1][8] = s->frc[1][8];
  frc[2][8] = s->frc[2][8];
  frc[3][8] = s->frc[3][8];
  frc[1][9] = s->frc[1][9];
  frc[2][9] = s->frc[2][9];
  frc[3][9] = s->frc[3][9];
  frc[1][10] = s->frc[1][10];
  frc[2][10] = s->frc[2][10];
  frc[3][10] = s->frc[3][10];
  frc[1][11] = s->frc[1][11];
  frc[2][11] = s->frc[2][11];
  frc[3][11] = s->frc[3][11];
  frc[1][13] = s->frc[1][13];
  frc[2][13] = s->frc[2][13];
  frc[3][13] = s->frc[3][13];
  frc[1][14] = s->frc[1][14];
  frc[2][14] = s->frc[2][14];
  frc[3][14] = s->frc[3][14];
  frc[1][15] = s->frc[1][15];
  frc[2][15] = s->frc[2][15];
  frc[3][15] = s->frc[3][15];
  frc[1][16] = s->frc[1][16];
  frc[2][16] = s->frc[2][16];
  frc[3][16] = s->frc[3][16];
  frc[1][17] = s->frc[1][17];
  frc[2][17] = s->frc[2][17];
  frc[3][17] = s->frc[3][17];
  frc[1][19] = s->frc[1][19];
  frc[2][19] = s->frc[2][19];
  frc[3][19] = s->frc[3][19];
  frc[1][20] = s->frc[1][20];
  frc[2][20] = s->frc[2][20];
  frc[3][20] = s->frc[3][20];
  frc[1][22] = s->frc[1][22];
  frc[2][22] = s->frc[2][22];
  frc[3][22] = s->frc[3][22];
  frc[1][23] = s->frc[1][23];
  frc[2][23] = s->frc[2][23];
  frc[3][23] = s->frc[3][23];
  frc[1][24] = s->frc[1][24];
  frc[2][24] = s->frc[2][24];
  frc[3][24] = s->frc[3][24];
  frc[1][25] = s->frc[1][25];
  frc[2][25] = s->frc[2][25];
  frc[3][25] = s->frc[3][25];
  frc[1][26] = s->frc[1][26];
  frc[2][26] = s->frc[2][26];
  frc[3][26] = s->frc[3][26];
  frc[1][27] = s->frc[1][27];
  frc[2][27] = s->frc[2][27];
  frc[3][27] = s->frc[3][27];
  frc[1][29] = s->frc[1][29];
  frc[2][29] = s->frc[2][29];
  frc[3][29] = s->frc[3][29];
  frc[1][30] = s->frc[1][30];
  frc[2][30] = s->frc[2][30];
  frc[3][30] = s->frc[3][30];
  frc[1][31] = s->frc[1][31];
  frc[2][31] = s->frc[2][31];
  frc[3][31] = s->frc[3][31];
  frc[1][32] = s->frc[1][32];
  frc[2][32] = s->frc[2][32];
  frc[3][32] = s->frc[3][32];
  frc[1][33] = s->frc[1][33];
  frc[2][33] = s->frc[2][33];
  frc[3][33] = s->frc[3][33];
  frc[1][34] = s->frc[1][34];
  frc[2][34] = s->frc[2][34];
  frc[3][34] = s->frc[3][34];
  trq[1][1] = s->trq[1][1];
  trq[2][1] = s->trq[2][1];
  trq[3][1] = s->trq[3][1];
  trq[1][2] = s->trq[1][2];
  trq[2][2] = s->trq[2][2];
  trq[3][2] = s->trq[3][2];
  trq[1][3] = s->trq[1][3];
  trq[2][3] = s->trq[2][3];
  trq[3][3] = s->trq[3][3];
  trq[1][4] = s->trq[1][4];
  trq[2][4] = s->trq[2][4];
  trq[3][4] = s->trq[3][4];
  trq[1][5] = s->trq[1][5];
  trq[2][5] = s->trq[2][5];
  trq[3][5] = s->trq[3][5];
  trq[1][6] = s->trq[1][6];
  trq[2][6] = s->trq[2][6];
  trq[3][6] = s->trq[3][6];
  trq[1][7] = s->trq[1][7];
  trq[2][7] = s->trq[2][7];
  trq[3][7] = s->trq[3][7];
  trq[1][8] = s->trq[1][8];
  trq[2][8] = s->trq[2][8];
  trq[3][8] = s->trq[3][8];
  trq[1][9] = s->trq[1][9];
  trq[2][9] = s->trq[2][9];
  trq[3][9] = s->trq[3][9];
  trq[1][10] = s->trq[1][10];
  trq[2][10] = s->trq[2][10];
  trq[3][10] = s->trq[3][10];
  trq[1][11] = s->trq[1][11];
  trq[2][11] = s->trq[2][11];
  trq[3][11] = s->trq[3][11];
  trq[1][13] = s->trq[1][13];
  trq[2][13] = s->trq[2][13];
  trq[3][13] = s->trq[3][13];
  trq[1][14] = s->trq[1][14];
  trq[2][14] = s->trq[2][14];
  trq[3][14] = s->trq[3][14];
  trq[1][15] = s->trq[1][15];
  trq[2][15] = s->trq[2][15];
  trq[3][15] = s->trq[3][15];
  trq[1][16] = s->trq[1][16];
  trq[2][16] = s->trq[2][16];
  trq[3][16] = s->trq[3][16];
  trq[1][17] = s->trq[1][17];
  trq[2][17] = s->trq[2][17];
  trq[3][17] = s->trq[3][17];
  trq[1][19] = s->trq[1][19];
  trq[2][19] = s->trq[2][19];
  trq[3][19] = s->trq[3][19];
  trq[1][20] = s->trq[1][20];
  trq[2][20] = s->trq[2][20];
  trq[3][20] = s->trq[3][20];
  trq[1][22] = s->trq[1][22];
  trq[2][22] = s->trq[2][22];
  trq[3][22] = s->trq[3][22];
  trq[1][23] = s->trq[1][23];
  trq[2][23] = s->trq[2][23];
  trq[3][23] = s->trq[3][23];
  trq[1][24] = s->trq[1][24];
  trq[2][24] = s->trq[2][24];
  trq[3][24] = s->trq[3][24];
  trq[1][25] = s->trq[1][25];
  trq[2][25] = s->trq[2][25];
  trq[3][25] = s->trq[3][25];
  trq[1][26] = s->trq[1][26];
  trq[2][26] = s->trq[2][26];
  trq[3][26] = s->trq[3][26];
  trq[1][27] = s->trq[1][27];
  trq[2][27] = s->trq[2][27];
  trq[3][27] = s->trq[3][27];
  trq[1][29] = s->trq[1][29];
  trq[2][29] = s->trq[2][29];
  trq[3][29] = s->trq[3][29];
  trq[1][30] = s->trq[1][30];
  trq[2][30] = s->trq[2][30];
  trq[3][30] = s->trq[3][30];
  trq[1][31] = s->trq[1][31];
  trq[2][31] = s->trq[2][31];
  trq[3][31] = s->trq[3][31];
  trq[1][32] = s->trq[1][32];
  trq[2][32] = s->trq[2][32];
  trq[3][32] = s->trq[3][32];
  trq[1][33] = s->trq[1][33];
  trq[2][33] = s->trq[2][33];
  trq[3][33] = s->trq[3][33];
  trq[1][34] = s->trq[1][34];
  trq[2][34] = s->trq[2][34];
  trq[3][34] = s->trq[3][34];

// ====== END Task 0 ====== 


}
 

