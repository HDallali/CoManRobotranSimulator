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
//	==> Generation Date : Thu Oct 16 13:29:37 2014
//
//	==> Project name : CoMan_Legs_7DofArm
//	==> using XML input file 
//
//	==> Number of joints : 35
//
//	==> Function : F 2 : Inverse Dynamics : RNEA
//	==> Flops complexity : 5114
//
//	==> Generation Time :  0.060 seconds
//	==> Post-Processing :  0.050 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "MBSdataStructR7.h"
#include "MBSfunR7.h"
 
void invdyna(double *Qq,
MBSdataStruct *s, double tsim)

// double Qq[35];
{ 
 
#include "mbs_invdyna_CoMan_Legs_7DofArm.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

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

// = = Block_0_1_0_0_0_0 = = 
 
// Forward Kinematics 

  ALPHA33 = qdd[3]-s->g[3];
  ALPHA24 = qdd[2]*C4+ALPHA33*S4;
  ALPHA34 = -(qdd[2]*S4-ALPHA33*C4);
  OM15 = qd[4]*C5;
  OMp15 = -(qd[4]*qd[5]*S5-qdd[4]*C5);
  ALPHA15 = qdd[1]*C5-ALPHA34*S5;
  ALPHA35 = qdd[1]*S5+ALPHA34*C5;
  OM16 = qd[5]*S6+OM15*C6;
  OM26 = qd[5]*C6-OM15*S6;
  OM36 = qd[6]+qd[4]*S5;
  OMp16 = C6*(OMp15+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM15);
  OMp26 = C6*(qdd[5]-qd[6]*OM15)-S6*(OMp15+qd[5]*qd[6]);
  OMp36 = qdd[6]+qd[4]*qd[5]*C5+qdd[4]*S5;
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BS96 = -(OM16*OM16+OM26*OM26);
  BETA26 = BS26-OMp36;
  BETA36 = BS36+OMp26;
  BETA46 = BS26+OMp36;
  BETA66 = BS66-OMp16;
  BETA76 = BS36-OMp26;
  BETA86 = BS66+OMp16;
  ALPHA16 = ALPHA15*C6+ALPHA24*S6;
  ALPHA26 = -(ALPHA15*S6-ALPHA24*C6);
  OM17 = OM16*C7-OM36*S7;
  OM27 = qd[7]+OM26;
  OM37 = OM16*S7+OM36*C7;
  OMp17 = C7*(OMp16-qd[7]*OM36)-S7*(OMp36+qd[7]*OM16);
  OMp27 = qdd[7]+OMp26;
  OMp37 = C7*(OMp36+qd[7]*OM16)+S7*(OMp16-qd[7]*OM36);
  BS27 = OM17*OM27;
  BS37 = OM17*OM37;
  BS57 = -(OM17*OM17+OM37*OM37);
  BS67 = OM27*OM37;
  BETA27 = BS27-OMp37;
  BETA87 = BS67+OMp17;
  ALPHA17 = C7*(ALPHA16+BETA26*s->dpt[2][1])-S7*(ALPHA35+BETA86*s->dpt[2][1]);
  ALPHA27 = ALPHA26+BS56*s->dpt[2][1];
  ALPHA37 = C7*(ALPHA35+BETA86*s->dpt[2][1])+S7*(ALPHA16+BETA26*s->dpt[2][1]);
  OM18 = qd[8]+OM17;
  OM28 = OM27*C8+OM37*S8;
  OM38 = -(OM27*S8-OM37*C8);
  OMp18 = qdd[8]+OMp17;
  OMp28 = C8*(OMp27+qd[8]*OM37)+S8*(OMp37-qd[8]*OM27);
  OMp38 = C8*(OMp37-qd[8]*OM27)-S8*(OMp27+qd[8]*OM37);
  BS28 = OM18*OM28;
  BS38 = OM18*OM38;
  BS68 = OM28*OM38;
  BS98 = -(OM18*OM18+OM28*OM28);
  BETA38 = BS38+OMp28;
  BETA68 = BS68-OMp18;
  ALPHA18 = ALPHA17+BETA27*s->dpt[2][6];
  ALPHA28 = C8*(ALPHA27+BS57*s->dpt[2][6])+S8*(ALPHA37+BETA87*s->dpt[2][6]);
  ALPHA38 = C8*(ALPHA37+BETA87*s->dpt[2][6])-S8*(ALPHA27+BS57*s->dpt[2][6]);
  OM19 = OM18*C9+OM28*S9;
  OM29 = -(OM18*S9-OM28*C9);
  OM39 = qd[9]+OM38;
  OMp19 = C9*(OMp18+qd[9]*OM28)+S9*(OMp28-qd[9]*OM18);
  OMp29 = C9*(OMp28-qd[9]*OM18)-S9*(OMp18+qd[9]*OM28);
  OMp39 = qdd[9]+OMp38;
  BS29 = OM19*OM29;
  BS39 = OM19*OM39;
  BS69 = OM29*OM39;
  BS99 = -(OM19*OM19+OM29*OM29);
  BETA39 = BS39+OMp29;
  BETA69 = BS69-OMp19;
  ALPHA19 = C9*(ALPHA18+BETA38*s->dpt[3][8])+S9*(ALPHA28+BETA68*s->dpt[3][8]);
  ALPHA29 = C9*(ALPHA28+BETA68*s->dpt[3][8])-S9*(ALPHA18+BETA38*s->dpt[3][8]);
  ALPHA39 = ALPHA38+BS98*s->dpt[3][8];
  OM110 = OM19*C10-OM39*S10;
  OM210 = qd[10]+OM29;
  OM310 = OM19*S10+OM39*C10;
  OMp110 = C10*(OMp19-qd[10]*OM39)-S10*(OMp39+qd[10]*OM19);
  OMp210 = qdd[10]+OMp29;
  OMp310 = C10*(OMp39+qd[10]*OM19)+S10*(OMp19-qd[10]*OM39);
  BS210 = OM110*OM210;
  BS310 = OM110*OM310;
  BS610 = OM210*OM310;
  BS910 = -(OM110*OM110+OM210*OM210);
  BETA310 = BS310+OMp210;
  BETA610 = BS610-OMp110;
  ALPHA110 = C10*(ALPHA19+BETA39*s->dpt[3][10])-S10*(ALPHA39+BS99*s->dpt[3][10]);
  ALPHA210 = ALPHA29+BETA69*s->dpt[3][10];
  ALPHA310 = C10*(ALPHA39+BS99*s->dpt[3][10])+S10*(ALPHA19+BETA39*s->dpt[3][10]);
  OM111 = qd[11]+OM110;
  OM211 = OM210*C11+OM310*S11;
  OM311 = -(OM210*S11-OM310*C11);
  OMp111 = qdd[11]+OMp110;
  OMp211 = C11*(OMp210+qd[11]*OM310)+S11*(OMp310-qd[11]*OM210);
  OMp311 = C11*(OMp310-qd[11]*OM210)-S11*(OMp210+qd[11]*OM310);
  BS211 = OM111*OM211;
  BS311 = OM111*OM311;
  BS611 = OM211*OM311;
  ALPHA111 = ALPHA110+BETA310*s->dpt[3][12];
  ALPHA211 = C11*(ALPHA210+BETA610*s->dpt[3][12])+S11*(ALPHA310+BS910*s->dpt[3][12]);
  ALPHA311 = C11*(ALPHA310+BS910*s->dpt[3][12])-S11*(ALPHA210+BETA610*s->dpt[3][12]);
  OM112 = OM111*C12-OM311*S12;
  OM212 = qd[12]+OM211;
  OM312 = OM111*S12+OM311*C12;
  OMp112 = C12*(OMp111-qd[12]*OM311)-S12*(OMp311+qd[12]*OM111);
  OMp212 = qdd[12]+OMp211;
  OMp312 = C12*(OMp311+qd[12]*OM111)+S12*(OMp111-qd[12]*OM311);
  BS212 = OM112*OM212;
  BS312 = OM112*OM312;
  BS612 = OM212*OM312;
  OM113 = OM16*C13-OM36*S13;
  OM213 = qd[13]+OM26;
  OM313 = OM16*S13+OM36*C13;
  OMp113 = C13*(OMp16-qd[13]*OM36)-S13*(OMp36+qd[13]*OM16);
  OMp213 = qdd[13]+OMp26;
  OMp313 = C13*(OMp36+qd[13]*OM16)+S13*(OMp16-qd[13]*OM36);
  BS213 = OM113*OM213;
  BS313 = OM113*OM313;
  BS513 = -(OM113*OM113+OM313*OM313);
  BS613 = OM213*OM313;
  BETA213 = BS213-OMp313;
  BETA813 = BS613+OMp113;
  ALPHA113 = C13*(ALPHA16+BETA26*s->dpt[2][2])-S13*(ALPHA35+BETA86*s->dpt[2][2]);
  ALPHA213 = ALPHA26+BS56*s->dpt[2][2];
  ALPHA313 = C13*(ALPHA35+BETA86*s->dpt[2][2])+S13*(ALPHA16+BETA26*s->dpt[2][2]);
  OM114 = qd[14]+OM113;
  OM214 = OM213*C14+OM313*S14;
  OM314 = -(OM213*S14-OM313*C14);
  OMp114 = qdd[14]+OMp113;
  OMp214 = C14*(OMp213+qd[14]*OM313)+S14*(OMp313-qd[14]*OM213);
  OMp314 = C14*(OMp313-qd[14]*OM213)-S14*(OMp213+qd[14]*OM313);
  BS214 = OM114*OM214;
  BS314 = OM114*OM314;
  BS614 = OM214*OM314;
  BS914 = -(OM114*OM114+OM214*OM214);
  BETA314 = BS314+OMp214;
  BETA614 = BS614-OMp114;
  ALPHA114 = ALPHA113+BETA213*s->dpt[2][22];
  ALPHA214 = C14*(ALPHA213+BS513*s->dpt[2][22])+S14*(ALPHA313+BETA813*s->dpt[2][22]);
  ALPHA314 = C14*(ALPHA313+BETA813*s->dpt[2][22])-S14*(ALPHA213+BS513*s->dpt[2][22]);
  OM115 = OM114*C15+OM214*S15;
  OM215 = -(OM114*S15-OM214*C15);
  OM315 = qd[15]+OM314;
  OMp115 = C15*(OMp114+qd[15]*OM214)+S15*(OMp214-qd[15]*OM114);
  OMp215 = C15*(OMp214-qd[15]*OM114)-S15*(OMp114+qd[15]*OM214);
  OMp315 = qdd[15]+OMp314;
  BS215 = OM115*OM215;
  BS315 = OM115*OM315;
  BS615 = OM215*OM315;
  BS915 = -(OM115*OM115+OM215*OM215);
  BETA315 = BS315+OMp215;
  BETA615 = BS615-OMp115;
  ALPHA115 = C15*(ALPHA114+BETA314*s->dpt[3][24])+S15*(ALPHA214+BETA614*s->dpt[3][24]);
  ALPHA215 = C15*(ALPHA214+BETA614*s->dpt[3][24])-S15*(ALPHA114+BETA314*s->dpt[3][24]);
  ALPHA315 = ALPHA314+BS914*s->dpt[3][24];
  OM116 = OM115*C16-OM315*S16;
  OM216 = qd[16]+OM215;
  OM316 = OM115*S16+OM315*C16;
  OMp116 = C16*(OMp115-qd[16]*OM315)-S16*(OMp315+qd[16]*OM115);
  OMp216 = qdd[16]+OMp215;
  OMp316 = C16*(OMp315+qd[16]*OM115)+S16*(OMp115-qd[16]*OM315);
  BS216 = OM116*OM216;
  BS316 = OM116*OM316;
  BS616 = OM216*OM316;
  BS916 = -(OM116*OM116+OM216*OM216);
  BETA316 = BS316+OMp216;
  BETA616 = BS616-OMp116;
  ALPHA116 = C16*(ALPHA115+BETA315*s->dpt[3][26])-S16*(ALPHA315+BS915*s->dpt[3][26]);
  ALPHA216 = ALPHA215+BETA615*s->dpt[3][26];
  ALPHA316 = C16*(ALPHA315+BS915*s->dpt[3][26])+S16*(ALPHA115+BETA315*s->dpt[3][26]);
  OM117 = qd[17]+OM116;
  OM217 = OM216*C17+OM316*S17;
  OM317 = -(OM216*S17-OM316*C17);
  OMp117 = qdd[17]+OMp116;
  OMp217 = C17*(OMp216+qd[17]*OM316)+S17*(OMp316-qd[17]*OM216);
  OMp317 = C17*(OMp316-qd[17]*OM216)-S17*(OMp216+qd[17]*OM316);
  BS217 = OM117*OM217;
  BS317 = OM117*OM317;
  BS617 = OM217*OM317;
  ALPHA117 = ALPHA116+BETA316*s->dpt[3][28];
  ALPHA217 = C17*(ALPHA216+BETA616*s->dpt[3][28])+S17*(ALPHA316+BS916*s->dpt[3][28]);
  ALPHA317 = C17*(ALPHA316+BS916*s->dpt[3][28])-S17*(ALPHA216+BETA616*s->dpt[3][28]);
  OM118 = OM117*C18-OM317*S18;
  OM218 = qd[18]+OM217;
  OM318 = OM117*S18+OM317*C18;
  OMp118 = C18*(OMp117-qd[18]*OM317)-S18*(OMp317+qd[18]*OM117);
  OMp218 = qdd[18]+OMp217;
  OMp318 = C18*(OMp317+qd[18]*OM117)+S18*(OMp117-qd[18]*OM317);
  BS218 = OM118*OM218;
  BS318 = OM118*OM318;
  BS618 = OM218*OM318;
  OM119 = qd[19]+OM16;
  OM219 = OM26*C19+OM36*S19;
  OM319 = -(OM26*S19-OM36*C19);
  OMp119 = qdd[19]+OMp16;
  OMp219 = C19*(OMp26+qd[19]*OM36)+S19*(OMp36-qd[19]*OM26);
  OMp319 = C19*(OMp36-qd[19]*OM26)-S19*(OMp26+qd[19]*OM36);
  BS219 = OM119*OM219;
  BS319 = OM119*OM319;
  BS619 = OM219*OM319;
  ALPHA119 = ALPHA16+BETA36*s->dpt[3][3]+BS16*s->dpt[1][3];
  ALPHA219 = C19*(ALPHA26+BETA46*s->dpt[1][3]+BETA66*s->dpt[3][3])+S19*(ALPHA35+BETA76*s->dpt[1][3]+BS96*s->dpt[3][3]);
  ALPHA319 = C19*(ALPHA35+BETA76*s->dpt[1][3]+BS96*s->dpt[3][3])-S19*(ALPHA26+BETA46*s->dpt[1][3]+BETA66*s->dpt[3][3]);
  OM120 = OM119*C20-OM319*S20;
  OM220 = qd[20]+OM219;
  OM320 = OM119*S20+OM319*C20;
  OMp120 = C20*(OMp119-qd[20]*OM319)-S20*(OMp319+qd[20]*OM119);
  OMp220 = qdd[20]+OMp219;
  OMp320 = C20*(OMp319+qd[20]*OM119)+S20*(OMp119-qd[20]*OM319);
  BS220 = OM120*OM220;
  BS320 = OM120*OM320;
  BS620 = OM220*OM320;
  BS920 = -(OM120*OM120+OM220*OM220);
  BETA320 = BS320+OMp220;
  BETA620 = BS620-OMp120;
  ALPHA120 = ALPHA119*C20-ALPHA319*S20;
  ALPHA320 = ALPHA119*S20+ALPHA319*C20;
  OM121 = OM120*C21+OM220*S21;
  OM221 = -(OM120*S21-OM220*C21);
  OM321 = qd[21]+OM320;
  OMp121 = C21*(OMp120+qd[21]*OM220)+S21*(OMp220-qd[21]*OM120);
  OMp221 = C21*(OMp220-qd[21]*OM120)-S21*(OMp120+qd[21]*OM220);
  OMp321 = qdd[21]+OMp320;
  BS121 = -(OM221*OM221+OM321*OM321);
  BS221 = OM121*OM221;
  BS321 = OM121*OM321;
  BS521 = -(OM121*OM121+OM321*OM321);
  BS621 = OM221*OM321;
  BS921 = -(OM121*OM121+OM221*OM221);
  BETA221 = BS221-OMp321;
  BETA321 = BS321+OMp221;
  BETA421 = BS221+OMp321;
  BETA621 = BS621-OMp121;
  BETA721 = BS321-OMp221;
  BETA821 = BS621+OMp121;
  ALPHA121 = C21*(ALPHA120+BETA320*s->dpt[3][40])+S21*(ALPHA219+BETA620*s->dpt[3][40]);
  ALPHA221 = C21*(ALPHA219+BETA620*s->dpt[3][40])-S21*(ALPHA120+BETA320*s->dpt[3][40]);
  ALPHA321 = ALPHA320+BS920*s->dpt[3][40];
  OM122 = OM121*C22-OM321*S22;
  OM222 = qd[22]+OM221;
  OM322 = OM121*S22+OM321*C22;
  OMp122 = C22*(OMp121-qd[22]*OM321)-S22*(OMp321+qd[22]*OM121);
  OMp222 = qdd[22]+OMp221;
  OMp322 = C22*(OMp321+qd[22]*OM121)+S22*(OMp121-qd[22]*OM321);
  BS222 = OM122*OM222;
  BS322 = OM122*OM322;
  BS522 = -(OM122*OM122+OM322*OM322);
  BS622 = OM222*OM322;
  BETA222 = BS222-OMp322;
  BETA822 = BS622+OMp122;
  ALPHA122 = C22*(ALPHA121+BETA221*s->dpt[2][44]+BETA321*s->dpt[3][44]+BS121*s->dpt[1][44])-S22*(ALPHA321+BETA721*
 s->dpt[1][44]+BETA821*s->dpt[2][44]+BS921*s->dpt[3][44]);
  ALPHA222 = ALPHA221+BETA421*s->dpt[1][44]+BETA621*s->dpt[3][44]+BS521*s->dpt[2][44];
  ALPHA322 = C22*(ALPHA321+BETA721*s->dpt[1][44]+BETA821*s->dpt[2][44]+BS921*s->dpt[3][44])+S22*(ALPHA121+BETA221*
 s->dpt[2][44]+BETA321*s->dpt[3][44]+BS121*s->dpt[1][44]);
  OM123 = qd[23]+OM122;
  OM223 = OM222*C23+OM322*S23;
  OM323 = -(OM222*S23-OM322*C23);
  OMp123 = qdd[23]+OMp122;
  OMp223 = C23*(OMp222+qd[23]*OM322)+S23*(OMp322-qd[23]*OM222);
  OMp323 = C23*(OMp322-qd[23]*OM222)-S23*(OMp222+qd[23]*OM322);
  BS223 = OM123*OM223;
  BS523 = -(OM123*OM123+OM323*OM323);
  BETA223 = BS223-OMp323;
  BETA823 = OMp123+OM223*OM323;
  ALPHA123 = ALPHA122+BETA222*s->dpt[2][48];
  ALPHA223 = C23*(ALPHA222+BS522*s->dpt[2][48])+S23*(ALPHA322+BETA822*s->dpt[2][48]);
  ALPHA323 = C23*(ALPHA322+BETA822*s->dpt[2][48])-S23*(ALPHA222+BS522*s->dpt[2][48]);
  OM124 = OM123*C24-OM323*S24;
  OM224 = qd[24]+OM223;
  OM324 = OM123*S24+OM323*C24;
  OMp124 = C24*(OMp123-qd[24]*OM323)-S24*(OMp323+qd[24]*OM123);
  OMp224 = qdd[24]+OMp223;
  OMp324 = C24*(OMp323+qd[24]*OM123)+S24*(OMp123-qd[24]*OM323);
  BS224 = OM124*OM224;
  BS324 = OM124*OM324;
  BS524 = -(OM124*OM124+OM324*OM324);
  BS624 = OM224*OM324;
  BETA224 = BS224-OMp324;
  BETA824 = BS624+OMp124;
  ALPHA124 = C24*(ALPHA123+BETA223*s->dpt[2][50])-S24*(ALPHA323+BETA823*s->dpt[2][50]);
  ALPHA224 = ALPHA223+BS523*s->dpt[2][50];
  ALPHA324 = C24*(ALPHA323+BETA823*s->dpt[2][50])+S24*(ALPHA123+BETA223*s->dpt[2][50]);
  OM125 = OM124*C25+OM224*S25;
  OM225 = -(OM124*S25-OM224*C25);
  OM325 = qd[25]+OM324;
  OMp125 = C25*(OMp124+qd[25]*OM224)+S25*(OMp224-qd[25]*OM124);
  OMp225 = C25*(OMp224-qd[25]*OM124)-S25*(OMp124+qd[25]*OM224);
  OMp325 = qdd[25]+OMp324;
  BS125 = -(OM225*OM225+OM325*OM325);
  BS225 = OM125*OM225;
  BS325 = OM125*OM325;
  BS625 = OM225*OM325;
  BETA425 = BS225+OMp325;
  BETA725 = BS325-OMp225;
  ALPHA125 = C25*(ALPHA124+BETA224*s->dpt[2][52])+S25*(ALPHA224+BS524*s->dpt[2][52]);
  ALPHA225 = C25*(ALPHA224+BS524*s->dpt[2][52])-S25*(ALPHA124+BETA224*s->dpt[2][52]);
  ALPHA325 = ALPHA324+BETA824*s->dpt[2][52];
  OM126 = OM125*C26-OM325*S26;
  OM226 = qd[26]+OM225;
  OM326 = OM125*S26+OM325*C26;
  OMp126 = C26*(OMp125-qd[26]*OM325)-S26*(OMp325+qd[26]*OM125);
  OMp226 = qdd[26]+OMp225;
  OMp326 = C26*(OMp325+qd[26]*OM125)+S26*(OMp125-qd[26]*OM325);
  BS526 = -(OM126*OM126+OM326*OM326);
  BETA226 = OM126*OM226-OMp326;
  BETA826 = OMp126+OM226*OM326;
  ALPHA126 = C26*(ALPHA125+BS125*s->dpt[1][54])-S26*(ALPHA325+BETA725*s->dpt[1][54]);
  ALPHA226 = ALPHA225+BETA425*s->dpt[1][54];
  ALPHA326 = C26*(ALPHA325+BETA725*s->dpt[1][54])+S26*(ALPHA125+BS125*s->dpt[1][54]);
  OM127 = OM126*C27+OM226*S27;
  OM227 = -(OM126*S27-OM226*C27);
  OM327 = qd[27]+OM326;
  OMp127 = C27*(OMp126+qd[27]*OM226)+S27*(OMp226-qd[27]*OM126);
  OMp227 = C27*(OMp226-qd[27]*OM126)-S27*(OMp126+qd[27]*OM226);
  OMp327 = qdd[27]+OMp326;
  ALPHA127 = C27*(ALPHA126+BETA226*s->dpt[2][55])+S27*(ALPHA226+BS526*s->dpt[2][55]);
  ALPHA227 = C27*(ALPHA226+BS526*s->dpt[2][55])-S27*(ALPHA126+BETA226*s->dpt[2][55]);
  ALPHA327 = ALPHA326+BETA826*s->dpt[2][55];
  OM128 = qd[28]+OM127;
  OM228 = OM227*C28+OM327*S28;
  OM328 = -(OM227*S28-OM327*C28);
  OM129 = OM121*C29-OM321*S29;
  OM229 = qd[29]+OM221;
  OM329 = OM121*S29+OM321*C29;
  OMp129 = C29*(OMp121-qd[29]*OM321)-S29*(OMp321+qd[29]*OM121);
  OMp229 = qdd[29]+OMp221;
  OMp329 = C29*(OMp321+qd[29]*OM121)+S29*(OMp121-qd[29]*OM321);
  BS229 = OM129*OM229;
  BS329 = OM129*OM329;
  BS529 = -(OM129*OM129+OM329*OM329);
  BS629 = OM229*OM329;
  BETA229 = BS229-OMp329;
  BETA829 = BS629+OMp129;
  ALPHA129 = C29*(ALPHA121+BETA221*s->dpt[2][45]+BETA321*s->dpt[3][45]+BS121*s->dpt[1][45])-S29*(ALPHA321+BETA721*
 s->dpt[1][45]+BETA821*s->dpt[2][45]+BS921*s->dpt[3][45]);
  ALPHA229 = ALPHA221+BETA421*s->dpt[1][45]+BETA621*s->dpt[3][45]+BS521*s->dpt[2][45];
  ALPHA329 = C29*(ALPHA321+BETA721*s->dpt[1][45]+BETA821*s->dpt[2][45]+BS921*s->dpt[3][45])+S29*(ALPHA121+BETA221*
 s->dpt[2][45]+BETA321*s->dpt[3][45]+BS121*s->dpt[1][45]);
  OM130 = qd[30]+OM129;
  OM230 = OM229*C30+OM329*S30;
  OM330 = -(OM229*S30-OM329*C30);
  OMp130 = qdd[30]+OMp129;
  OMp230 = C30*(OMp229+qd[30]*OM329)+S30*(OMp329-qd[30]*OM229);
  OMp330 = C30*(OMp329-qd[30]*OM229)-S30*(OMp229+qd[30]*OM329);
  BS230 = OM130*OM230;
  BS330 = OM130*OM330;
  BS530 = -(OM130*OM130+OM330*OM330);
  BS630 = OM230*OM330;
  BETA230 = BS230-OMp330;
  BETA830 = BS630+OMp130;
  ALPHA130 = ALPHA129+BETA229*s->dpt[2][59];
  ALPHA230 = C30*(ALPHA229+BS529*s->dpt[2][59])+S30*(ALPHA329+BETA829*s->dpt[2][59]);
  ALPHA330 = C30*(ALPHA329+BETA829*s->dpt[2][59])-S30*(ALPHA229+BS529*s->dpt[2][59]);
  OM131 = OM130*C31-OM330*S31;
  OM231 = qd[31]+OM230;
  OM331 = OM130*S31+OM330*C31;
  OMp131 = C31*(OMp130-qd[31]*OM330)-S31*(OMp330+qd[31]*OM130);
  OMp231 = qdd[31]+OMp230;
  OMp331 = C31*(OMp330+qd[31]*OM130)+S31*(OMp130-qd[31]*OM330);
  BS131 = -(OM231*OM231+OM331*OM331);
  BS231 = OM131*OM231;
  BS331 = OM131*OM331;
  BS531 = -(OM131*OM131+OM331*OM331);
  BS631 = OM231*OM331;
  BETA231 = BS231-OMp331;
  BETA431 = BS231+OMp331;
  BETA731 = BS331-OMp231;
  BETA831 = BS631+OMp131;
  ALPHA131 = C31*(ALPHA130+BETA230*s->dpt[2][61])-S31*(ALPHA330+BETA830*s->dpt[2][61]);
  ALPHA231 = ALPHA230+BS530*s->dpt[2][61];
  ALPHA331 = C31*(ALPHA330+BETA830*s->dpt[2][61])+S31*(ALPHA130+BETA230*s->dpt[2][61]);
  OM132 = OM131*C32+OM231*S32;
  OM232 = -(OM131*S32-OM231*C32);
  OM332 = qd[32]+OM331;
  OMp132 = C32*(OMp131+qd[32]*OM231)+S32*(OMp231-qd[32]*OM131);
  OMp232 = C32*(OMp231-qd[32]*OM131)-S32*(OMp131+qd[32]*OM231);
  OMp332 = qdd[32]+OMp331;
  BS132 = -(OM232*OM232+OM332*OM332);
  BS232 = OM132*OM232;
  BETA732 = OM132*OM332-OMp232;
  ALPHA132 = C32*(ALPHA131+BETA231*s->dpt[2][63]+BS131*s->dpt[1][63])+S32*(ALPHA231+BETA431*s->dpt[1][63]+BS531*
 s->dpt[2][63]);
  ALPHA232 = C32*(ALPHA231+BETA431*s->dpt[1][63]+BS531*s->dpt[2][63])-S32*(ALPHA131+BETA231*s->dpt[2][63]+BS131*
 s->dpt[1][63]);
  ALPHA332 = ALPHA331+BETA731*s->dpt[1][63]+BETA831*s->dpt[2][63];
  OM133 = OM132*C33-OM332*S33;
  OM233 = qd[33]+OM232;
  OM333 = OM132*S33+OM332*C33;
  OMp133 = C33*(OMp132-qd[33]*OM332)-S33*(OMp332+qd[33]*OM132);
  OMp233 = qdd[33]+OMp232;
  OMp333 = C33*(OMp332+qd[33]*OM132)+S33*(OMp132-qd[33]*OM332);
  BS533 = -(OM133*OM133+OM333*OM333);
  BETA233 = OM133*OM233-OMp333;
  BETA833 = OMp133+OM233*OM333;
  ALPHA133 = C33*(ALPHA132+BS132*s->dpt[1][65])-S33*(ALPHA332+BETA732*s->dpt[1][65]);
  ALPHA233 = ALPHA232+s->dpt[1][65]*(BS232+OMp332);
  ALPHA333 = C33*(ALPHA332+BETA732*s->dpt[1][65])+S33*(ALPHA132+BS132*s->dpt[1][65]);
  OM134 = OM133*C34+OM233*S34;
  OM234 = -(OM133*S34-OM233*C34);
  OM334 = qd[34]+OM333;
  OMp134 = C34*(OMp133+qd[34]*OM233)+S34*(OMp233-qd[34]*OM133);
  OMp234 = C34*(OMp233-qd[34]*OM133)-S34*(OMp133+qd[34]*OM233);
  OMp334 = qdd[34]+OMp333;
  ALPHA134 = C34*(ALPHA133+BETA233*s->dpt[2][66])+S34*(ALPHA233+BS533*s->dpt[2][66]);
  ALPHA234 = C34*(ALPHA233+BS533*s->dpt[2][66])-S34*(ALPHA133+BETA233*s->dpt[2][66]);
  ALPHA334 = ALPHA333+BETA833*s->dpt[2][66];
  OM135 = qd[35]+OM134;
  OM235 = OM234*C35+OM334*S35;
  OM335 = -(OM234*S35-OM334*C35);
 
// Backward Dynamics 

  Fs235 = -(s->frc[2][35]-s->m[35]*(ALPHA234*C35+ALPHA334*S35));
  Fs335 = -(s->frc[3][35]+s->m[35]*(ALPHA234*S35-ALPHA334*C35));
  Cq135 = -(s->trq[1][35]-s->In[1][35]*(qdd[35]+OMp134)+OM235*OM335*(s->In[5][35]-s->In[9][35]));
  Cq235 = -(s->trq[2][35]-s->In[5][35]*(C35*(OMp234+qd[35]*OM334)+S35*(OMp334-qd[35]*OM234))-OM135*OM335*(s->In[1][35]-
 s->In[9][35]));
  Cq335 = -(s->trq[3][35]-s->In[9][35]*(C35*(OMp334-qd[35]*OM234)-S35*(OMp234+qd[35]*OM334))+OM135*OM235*(s->In[1][35]-
 s->In[5][35]));
  Fq134 = -(s->frc[1][34]+s->frc[1][35]-s->m[34]*ALPHA134-s->m[35]*ALPHA134);
  Fq234 = -(s->frc[2][34]-s->m[34]*ALPHA234-Fs235*C35+Fs335*S35);
  Fq334 = -(s->frc[3][34]-s->m[34]*ALPHA334-Fs235*S35-Fs335*C35);
  Cq134 = -(s->trq[1][34]-Cq135-s->In[1][34]*OMp134+OM234*OM334*(s->In[5][34]-s->In[9][34]));
  Cq234 = -(s->trq[2][34]-s->In[5][34]*OMp234-Cq235*C35+Cq335*S35-OM134*OM334*(s->In[1][34]-s->In[9][34]));
  Cq334 = -(s->trq[3][34]-s->In[9][34]*OMp334-Cq235*S35-Cq335*C35+OM134*OM234*(s->In[1][34]-s->In[5][34]));
  Fs133 = -(s->frc[1][33]-s->m[33]*(ALPHA133+BETA233*s->l[2][33]));
  Fs333 = -(s->frc[3][33]-s->m[33]*(ALPHA333+BETA833*s->l[2][33]));
  Fq133 = Fs133+Fq134*C34-Fq234*S34;
  Fq233 = -(s->frc[2][33]-s->m[33]*(ALPHA233+BS533*s->l[2][33])-Fq134*S34-Fq234*C34);
  Fq333 = Fq334+Fs333;
  Cq133 = -(s->trq[1][33]-s->In[1][33]*OMp133-Cq134*C34+Cq234*S34-Fq334*s->dpt[2][66]-Fs333*s->l[2][33]+OM233*OM333*(
 s->In[5][33]-s->In[9][33]));
  Cq233 = -(s->trq[2][33]-s->In[5][33]*OMp233-Cq134*S34-Cq234*C34-OM133*OM333*(s->In[1][33]-s->In[9][33]));
  Cq333 = -(s->trq[3][33]-Cq334-s->In[9][33]*OMp333+Fs133*s->l[2][33]+OM133*OM233*(s->In[1][33]-s->In[5][33])+
 s->dpt[2][66]*(Fq134*C34-Fq234*S34));
  Fs132 = -(s->frc[1][32]-s->m[32]*(ALPHA132+s->l[2][32]*(BS232-OMp332)));
  Fs332 = -(s->frc[3][32]-s->m[32]*(ALPHA332+s->l[2][32]*(OMp132+OM232*OM332)));
  Fq132 = Fs132+Fq133*C33+Fq333*S33;
  Fq232 = -(s->frc[2][32]-Fq233-s->m[32]*(ALPHA232-s->l[2][32]*(OM132*OM132+OM332*OM332)));
  Fq332 = Fs332-Fq133*S33+Fq333*C33;
  Cq132 = -(s->trq[1][32]-s->In[1][32]*OMp132-s->In[2][32]*OMp232-s->In[3][32]*OMp332-Cq133*C33-Cq333*S33-Fs332*
 s->l[2][32]-OM232*(s->In[3][32]*OM132+s->In[6][32]*OM232+s->In[9][32]*OM332)+OM332*(s->In[2][32]*OM132+s->In[5][32]*OM232+
 s->In[6][32]*OM332));
  Cq232 = -(s->trq[2][32]-Cq233-s->In[2][32]*OMp132-s->In[5][32]*OMp232-s->In[6][32]*OMp332+OM132*(s->In[3][32]*OM132+
 s->In[6][32]*OM232+s->In[9][32]*OM332)-OM332*(s->In[1][32]*OM132+s->In[2][32]*OM232+s->In[3][32]*OM332)-s->dpt[1][65]*(Fq133
 *S33-Fq333*C33));
  Cq332 = -(s->trq[3][32]-s->In[3][32]*OMp132-s->In[6][32]*OMp232-s->In[9][32]*OMp332+Cq133*S33-Cq333*C33-Fq233*
 s->dpt[1][65]+Fs132*s->l[2][32]-OM132*(s->In[2][32]*OM132+s->In[5][32]*OM232+s->In[6][32]*OM332)+OM232*(s->In[1][32]*OM132+
 s->In[2][32]*OM232+s->In[3][32]*OM332));
  Fs131 = -(s->frc[1][31]-s->m[31]*(ALPHA131+BETA231*s->l[2][31]+BS131*s->l[1][31]+s->l[3][31]*(BS331+OMp231)));
  Fs231 = -(s->frc[2][31]-s->m[31]*(ALPHA231+BETA431*s->l[1][31]+BS531*s->l[2][31]+s->l[3][31]*(BS631-OMp131)));
  Fs331 = -(s->frc[3][31]-s->m[31]*(ALPHA331+BETA731*s->l[1][31]+BETA831*s->l[2][31]-s->l[3][31]*(OM131*OM131+OM231*
 OM231)));
  Fq131 = Fs131+Fq132*C32-Fq232*S32;
  Fq331 = Fq332+Fs331;
  Cq131 = -(s->trq[1][31]-s->In[1][31]*OMp131-s->In[2][31]*OMp231-s->In[3][31]*OMp331-Cq132*C32+Cq232*S32-Fq332*
 s->dpt[2][63]+Fs231*s->l[3][31]-Fs331*s->l[2][31]-OM231*(s->In[3][31]*OM131+s->In[6][31]*OM231+s->In[9][31]*OM331)+OM331*(
 s->In[2][31]*OM131+s->In[5][31]*OM231+s->In[6][31]*OM331));
  Cq231 = -(s->trq[2][31]-s->In[2][31]*OMp131-s->In[5][31]*OMp231-s->In[6][31]*OMp331-Cq132*S32-Cq232*C32+Fq332*
 s->dpt[1][63]-Fs131*s->l[3][31]+Fs331*s->l[1][31]+OM131*(s->In[3][31]*OM131+s->In[6][31]*OM231+s->In[9][31]*OM331)-OM331*(
 s->In[1][31]*OM131+s->In[2][31]*OM231+s->In[3][31]*OM331));
  Cq331 = -(s->trq[3][31]-Cq332-s->In[3][31]*OMp131-s->In[6][31]*OMp231-s->In[9][31]*OMp331+Fs131*s->l[2][31]-Fs231*
 s->l[1][31]-OM131*(s->In[2][31]*OM131+s->In[5][31]*OM231+s->In[6][31]*OM331)+OM231*(s->In[1][31]*OM131+s->In[2][31]*OM231+
 s->In[3][31]*OM331)-s->dpt[1][63]*(Fq132*S32+Fq232*C32)+s->dpt[2][63]*(Fq132*C32-Fq232*S32));
  Fs130 = -(s->frc[1][30]-s->m[30]*(ALPHA130+BETA230*s->l[2][30]-s->l[1][30]*(OM230*OM230+OM330*OM330)+s->l[3][30]*(
 BS330+OMp230)));
  Fs230 = -(s->frc[2][30]-s->m[30]*(ALPHA230+BS530*s->l[2][30]+s->l[1][30]*(BS230+OMp330)+s->l[3][30]*(BS630-OMp130)));
  Fs330 = -(s->frc[3][30]-s->m[30]*(ALPHA330+BETA830*s->l[2][30]+s->l[1][30]*(BS330-OMp230)-s->l[3][30]*(OM130*OM130+
 OM230*OM230)));
  Fq130 = Fs130+Fq131*C31+Fq331*S31;
  Fq230 = Fs230+Fs231+Fq132*S32+Fq232*C32;
  Fq330 = Fs330-Fq131*S31+Fq331*C31;
  Cq130 = -(s->trq[1][30]-s->In[1][30]*OMp130-s->In[2][30]*OMp230-s->In[3][30]*OMp330-Cq131*C31-Cq331*S31+Fs230*
 s->l[3][30]-Fs330*s->l[2][30]-OM230*(s->In[3][30]*OM130+s->In[6][30]*OM230+s->In[9][30]*OM330)+OM330*(s->In[2][30]*OM130+
 s->In[5][30]*OM230+s->In[6][30]*OM330)+s->dpt[2][61]*(Fq131*S31-Fq331*C31));
  Cq230 = -(s->trq[2][30]-Cq231-s->In[2][30]*OMp130-s->In[5][30]*OMp230-s->In[6][30]*OMp330-Fs130*s->l[3][30]+Fs330*
 s->l[1][30]+OM130*(s->In[3][30]*OM130+s->In[6][30]*OM230+s->In[9][30]*OM330)-OM330*(s->In[1][30]*OM130+s->In[2][30]*OM230+
 s->In[3][30]*OM330));
  Cq330 = -(s->trq[3][30]-s->In[3][30]*OMp130-s->In[6][30]*OMp230-s->In[9][30]*OMp330+Cq131*S31-Cq331*C31+Fs130*
 s->l[2][30]-Fs230*s->l[1][30]-OM130*(s->In[2][30]*OM130+s->In[5][30]*OM230+s->In[6][30]*OM330)+OM230*(s->In[1][30]*OM130+
 s->In[2][30]*OM230+s->In[3][30]*OM330)+s->dpt[2][61]*(Fq131*C31+Fq331*S31));
  Fs129 = -(s->frc[1][29]-s->m[29]*(ALPHA129+BETA229*s->l[2][29]-s->l[1][29]*(OM229*OM229+OM329*OM329)+s->l[3][29]*(
 BS329+OMp229)));
  Fs229 = -(s->frc[2][29]-s->m[29]*(ALPHA229+BS529*s->l[2][29]+s->l[1][29]*(BS229+OMp329)+s->l[3][29]*(BS629-OMp129)));
  Fs329 = -(s->frc[3][29]-s->m[29]*(ALPHA329+BETA829*s->l[2][29]+s->l[1][29]*(BS329-OMp229)-s->l[3][29]*(OM129*OM129+
 OM229*OM229)));
  Fq129 = Fq130+Fs129;
  Fq229 = Fs229+Fq230*C30-Fq330*S30;
  Fq329 = Fs329+Fq230*S30+Fq330*C30;
  Cq129 = -(s->trq[1][29]-Cq130-s->In[1][29]*OMp129-s->In[2][29]*OMp229-s->In[3][29]*OMp329+Fs229*s->l[3][29]-Fs329*
 s->l[2][29]-OM229*(s->In[3][29]*OM129+s->In[6][29]*OM229+s->In[9][29]*OM329)+OM329*(s->In[2][29]*OM129+s->In[5][29]*OM229+
 s->In[6][29]*OM329)-s->dpt[2][59]*(Fq230*S30+Fq330*C30));
  Cq229 = -(s->trq[2][29]-s->In[2][29]*OMp129-s->In[5][29]*OMp229-s->In[6][29]*OMp329-Cq230*C30+Cq330*S30-Fs129*
 s->l[3][29]+Fs329*s->l[1][29]+OM129*(s->In[3][29]*OM129+s->In[6][29]*OM229+s->In[9][29]*OM329)-OM329*(s->In[1][29]*OM129+
 s->In[2][29]*OM229+s->In[3][29]*OM329));
  Cq329 = -(s->trq[3][29]-s->In[3][29]*OMp129-s->In[6][29]*OMp229-s->In[9][29]*OMp329-Cq230*S30-Cq330*C30+Fq130*
 s->dpt[2][59]+Fs129*s->l[2][29]-Fs229*s->l[1][29]-OM129*(s->In[2][29]*OM129+s->In[5][29]*OM229+s->In[6][29]*OM329)+OM229*(
 s->In[1][29]*OM129+s->In[2][29]*OM229+s->In[3][29]*OM329));
  Fs228 = -(s->frc[2][28]-s->m[28]*(ALPHA227*C28+ALPHA327*S28));
  Fs328 = -(s->frc[3][28]+s->m[28]*(ALPHA227*S28-ALPHA327*C28));
  Cq128 = -(s->trq[1][28]-s->In[1][28]*(qdd[28]+OMp127)+OM228*OM328*(s->In[5][28]-s->In[9][28]));
  Cq228 = -(s->trq[2][28]-s->In[5][28]*(C28*(OMp227+qd[28]*OM327)+S28*(OMp327-qd[28]*OM227))-OM128*OM328*(s->In[1][28]-
 s->In[9][28]));
  Cq328 = -(s->trq[3][28]-s->In[9][28]*(C28*(OMp327-qd[28]*OM227)-S28*(OMp227+qd[28]*OM327))+OM128*OM228*(s->In[1][28]-
 s->In[5][28]));
  Fq127 = -(s->frc[1][27]+s->frc[1][28]-s->m[27]*ALPHA127-s->m[28]*ALPHA127);
  Fq227 = -(s->frc[2][27]-s->m[27]*ALPHA227-Fs228*C28+Fs328*S28);
  Fq327 = -(s->frc[3][27]-s->m[27]*ALPHA327-Fs228*S28-Fs328*C28);
  Cq127 = -(s->trq[1][27]-Cq128-s->In[1][27]*OMp127+OM227*OM327*(s->In[5][27]-s->In[9][27]));
  Cq227 = -(s->trq[2][27]-s->In[5][27]*OMp227-Cq228*C28+Cq328*S28-OM127*OM327*(s->In[1][27]-s->In[9][27]));
  Cq327 = -(s->trq[3][27]-s->In[9][27]*OMp327-Cq228*S28-Cq328*C28+OM127*OM227*(s->In[1][27]-s->In[5][27]));
  Fs126 = -(s->frc[1][26]-s->m[26]*(ALPHA126+BETA226*s->l[2][26]));
  Fs326 = -(s->frc[3][26]-s->m[26]*(ALPHA326+BETA826*s->l[2][26]));
  Fq126 = Fs126+Fq127*C27-Fq227*S27;
  Fq226 = -(s->frc[2][26]-s->m[26]*(ALPHA226+BS526*s->l[2][26])-Fq127*S27-Fq227*C27);
  Fq326 = Fq327+Fs326;
  Cq126 = -(s->trq[1][26]-s->In[1][26]*OMp126-Cq127*C27+Cq227*S27-Fq327*s->dpt[2][55]-Fs326*s->l[2][26]+OM226*OM326*(
 s->In[5][26]-s->In[9][26]));
  Cq226 = -(s->trq[2][26]-s->In[5][26]*OMp226-Cq127*S27-Cq227*C27-OM126*OM326*(s->In[1][26]-s->In[9][26]));
  Cq326 = -(s->trq[3][26]-Cq327-s->In[9][26]*OMp326+Fs126*s->l[2][26]+OM126*OM226*(s->In[1][26]-s->In[5][26])+
 s->dpt[2][55]*(Fq127*C27-Fq227*S27));
  Fs125 = -(s->frc[1][25]-s->m[25]*(ALPHA125+BS125*s->l[1][25]+s->l[2][25]*(BS225-OMp325)+s->l[3][25]*(BS325+OMp225)));
  Fs225 = -(s->frc[2][25]-s->m[25]*(ALPHA225+BETA425*s->l[1][25]-s->l[2][25]*(OM125*OM125+OM325*OM325)+s->l[3][25]*(
 BS625-OMp125)));
  Fs325 = -(s->frc[3][25]-s->m[25]*(ALPHA325+BETA725*s->l[1][25]+s->l[2][25]*(BS625+OMp125)-s->l[3][25]*(OM125*OM125+
 OM225*OM225)));
  Fq125 = Fs125+Fq126*C26+Fq326*S26;
  Fq225 = Fq226+Fs225;
  Fq325 = Fs325-Fq126*S26+Fq326*C26;
  Cq125 = -(s->trq[1][25]-s->In[1][25]*OMp125-s->In[2][25]*OMp225-s->In[3][25]*OMp325-Cq126*C26-Cq326*S26+Fs225*
 s->l[3][25]-Fs325*s->l[2][25]-OM225*(s->In[3][25]*OM125+s->In[6][25]*OM225+s->In[9][25]*OM325)+OM325*(s->In[2][25]*OM125+
 s->In[5][25]*OM225+s->In[6][25]*OM325));
  Cq225 = -(s->trq[2][25]-Cq226-s->In[2][25]*OMp125-s->In[5][25]*OMp225-s->In[6][25]*OMp325-Fs125*s->l[3][25]+Fs325*
 s->l[1][25]+OM125*(s->In[3][25]*OM125+s->In[6][25]*OM225+s->In[9][25]*OM325)-OM325*(s->In[1][25]*OM125+s->In[2][25]*OM225+
 s->In[3][25]*OM325)-s->dpt[1][54]*(Fq126*S26-Fq326*C26));
  Cq325 = -(s->trq[3][25]-s->In[3][25]*OMp125-s->In[6][25]*OMp225-s->In[9][25]*OMp325+Cq126*S26-Cq326*C26-Fq226*
 s->dpt[1][54]+Fs125*s->l[2][25]-Fs225*s->l[1][25]-OM125*(s->In[2][25]*OM125+s->In[5][25]*OM225+s->In[6][25]*OM325)+OM225*(
 s->In[1][25]*OM125+s->In[2][25]*OM225+s->In[3][25]*OM325));
  Fs124 = -(s->frc[1][24]-s->m[24]*(ALPHA124+BETA224*s->l[2][24]-s->l[1][24]*(OM224*OM224+OM324*OM324)+s->l[3][24]*(
 BS324+OMp224)));
  Fs224 = -(s->frc[2][24]-s->m[24]*(ALPHA224+BS524*s->l[2][24]+s->l[1][24]*(BS224+OMp324)+s->l[3][24]*(BS624-OMp124)));
  Fs324 = -(s->frc[3][24]-s->m[24]*(ALPHA324+BETA824*s->l[2][24]+s->l[1][24]*(BS324-OMp224)-s->l[3][24]*(OM124*OM124+
 OM224*OM224)));
  Fq124 = Fs124+Fq125*C25-Fq225*S25;
  Fq324 = Fq325+Fs324;
  Cq124 = -(s->trq[1][24]-s->In[1][24]*OMp124-s->In[2][24]*OMp224-s->In[3][24]*OMp324-Cq125*C25+Cq225*S25-Fq325*
 s->dpt[2][52]+Fs224*s->l[3][24]-Fs324*s->l[2][24]-OM224*(s->In[3][24]*OM124+s->In[6][24]*OM224+s->In[9][24]*OM324)+OM324*(
 s->In[2][24]*OM124+s->In[5][24]*OM224+s->In[6][24]*OM324));
  Cq224 = -(s->trq[2][24]-s->In[2][24]*OMp124-s->In[5][24]*OMp224-s->In[6][24]*OMp324-Cq125*S25-Cq225*C25-Fs124*
 s->l[3][24]+Fs324*s->l[1][24]+OM124*(s->In[3][24]*OM124+s->In[6][24]*OM224+s->In[9][24]*OM324)-OM324*(s->In[1][24]*OM124+
 s->In[2][24]*OM224+s->In[3][24]*OM324));
  Cq324 = -(s->trq[3][24]-Cq325-s->In[3][24]*OMp124-s->In[6][24]*OMp224-s->In[9][24]*OMp324+Fs124*s->l[2][24]-Fs224*
 s->l[1][24]-OM124*(s->In[2][24]*OM124+s->In[5][24]*OM224+s->In[6][24]*OM324)+OM224*(s->In[1][24]*OM124+s->In[2][24]*OM224+
 s->In[3][24]*OM324)+s->dpt[2][52]*(Fq125*C25-Fq225*S25));
  Fs123 = -(s->frc[1][23]-s->m[23]*(ALPHA123+BETA223*s->l[2][23]-s->l[1][23]*(OM223*OM223+OM323*OM323)));
  Fs223 = -(s->frc[2][23]-s->m[23]*(ALPHA223+BS523*s->l[2][23]+s->l[1][23]*(BS223+OMp323)));
  Fs323 = -(s->frc[3][23]-s->m[23]*(ALPHA323+BETA823*s->l[2][23]-s->l[1][23]*(OMp223-OM123*OM323)));
  Fq123 = Fs123+Fq124*C24+Fq324*S24;
  Fq223 = Fs223+Fs224+Fq125*S25+Fq225*C25;
  Fq323 = Fs323-Fq124*S24+Fq324*C24;
  Cq123 = -(s->trq[1][23]-s->In[1][23]*OMp123-s->In[2][23]*OMp223-s->In[3][23]*OMp323-Cq124*C24-Cq324*S24-Fs323*
 s->l[2][23]-OM223*(s->In[3][23]*OM123+s->In[6][23]*OM223+s->In[9][23]*OM323)+OM323*(s->In[2][23]*OM123+s->In[5][23]*OM223+
 s->In[6][23]*OM323)+s->dpt[2][50]*(Fq124*S24-Fq324*C24));
  Cq223 = -(s->trq[2][23]-Cq224-s->In[2][23]*OMp123-s->In[5][23]*OMp223-s->In[6][23]*OMp323+Fs323*s->l[1][23]+OM123*(
 s->In[3][23]*OM123+s->In[6][23]*OM223+s->In[9][23]*OM323)-OM323*(s->In[1][23]*OM123+s->In[2][23]*OM223+s->In[3][23]*OM323));
  Cq323 = -(s->trq[3][23]-s->In[3][23]*OMp123-s->In[6][23]*OMp223-s->In[9][23]*OMp323+Cq124*S24-Cq324*C24+Fs123*
 s->l[2][23]-Fs223*s->l[1][23]-OM123*(s->In[2][23]*OM123+s->In[5][23]*OM223+s->In[6][23]*OM323)+OM223*(s->In[1][23]*OM123+
 s->In[2][23]*OM223+s->In[3][23]*OM323)+s->dpt[2][50]*(Fq124*C24+Fq324*S24));
  Fs122 = -(s->frc[1][22]-s->m[22]*(ALPHA122+BETA222*s->l[2][22]-s->l[1][22]*(OM222*OM222+OM322*OM322)+s->l[3][22]*(
 BS322+OMp222)));
  Fs222 = -(s->frc[2][22]-s->m[22]*(ALPHA222+BS522*s->l[2][22]+s->l[1][22]*(BS222+OMp322)+s->l[3][22]*(BS622-OMp122)));
  Fs322 = -(s->frc[3][22]-s->m[22]*(ALPHA322+BETA822*s->l[2][22]+s->l[1][22]*(BS322-OMp222)-s->l[3][22]*(OM122*OM122+
 OM222*OM222)));
  Fq122 = Fq123+Fs122;
  Fq222 = Fs222+Fq223*C23-Fq323*S23;
  Fq322 = Fs322+Fq223*S23+Fq323*C23;
  Cq122 = -(s->trq[1][22]-Cq123-s->In[1][22]*OMp122-s->In[2][22]*OMp222-s->In[3][22]*OMp322+Fs222*s->l[3][22]-Fs322*
 s->l[2][22]-OM222*(s->In[3][22]*OM122+s->In[6][22]*OM222+s->In[9][22]*OM322)+OM322*(s->In[2][22]*OM122+s->In[5][22]*OM222+
 s->In[6][22]*OM322)-s->dpt[2][48]*(Fq223*S23+Fq323*C23));
  Cq222 = -(s->trq[2][22]-s->In[2][22]*OMp122-s->In[5][22]*OMp222-s->In[6][22]*OMp322-Cq223*C23+Cq323*S23-Fs122*
 s->l[3][22]+Fs322*s->l[1][22]+OM122*(s->In[3][22]*OM122+s->In[6][22]*OM222+s->In[9][22]*OM322)-OM322*(s->In[1][22]*OM122+
 s->In[2][22]*OM222+s->In[3][22]*OM322));
  Cq322 = -(s->trq[3][22]-s->In[3][22]*OMp122-s->In[6][22]*OMp222-s->In[9][22]*OMp322-Cq223*S23-Cq323*C23+Fq123*
 s->dpt[2][48]+Fs122*s->l[2][22]-Fs222*s->l[1][22]-OM122*(s->In[2][22]*OM122+s->In[5][22]*OM222+s->In[6][22]*OM322)+OM222*(
 s->In[1][22]*OM122+s->In[2][22]*OM222+s->In[3][22]*OM322));
  Fs121 = -(s->frc[1][21]-s->m[21]*(ALPHA121+BETA221*s->l[2][21]+BETA321*s->l[3][21]+BS121*s->l[1][21]));
  Fs221 = -(s->frc[2][21]-s->m[21]*(ALPHA221+BETA421*s->l[1][21]+BETA621*s->l[3][21]+BS521*s->l[2][21]));
  Fs321 = -(s->frc[3][21]-s->m[21]*(ALPHA321+BETA721*s->l[1][21]+BETA821*s->l[2][21]+BS921*s->l[3][21]));
  Fq121 = Fs121+Fq122*C22+Fq129*C29+Fq322*S22+Fq329*S29;
  Fq221 = Fq222+Fq229+Fs221;
  Cq121 = -(s->trq[1][21]-s->In[1][21]*OMp121-s->In[2][21]*OMp221-s->In[3][21]*OMp321-Cq122*C22-Cq129*C29-Cq322*S22-
 Cq329*S29+Fq222*s->dpt[3][44]+Fq229*s->dpt[3][45]+Fs221*s->l[3][21]-Fs321*s->l[2][21]-OM221*(s->In[3][21]*OM121+s->In[6][21]
 *OM221+s->In[9][21]*OM321)+OM321*(s->In[2][21]*OM121+s->In[5][21]*OM221+s->In[6][21]*OM321)+s->dpt[2][44]*(Fq122*S22-Fq322*
 C22)+s->dpt[2][45]*(Fq129*S29-Fq329*C29));
  Cq221 = -(s->trq[2][21]-Cq222-Cq229-s->In[2][21]*OMp121-s->In[5][21]*OMp221-s->In[6][21]*OMp321-Fs121*s->l[3][21]+
 Fs321*s->l[1][21]+OM121*(s->In[3][21]*OM121+s->In[6][21]*OM221+s->In[9][21]*OM321)-OM321*(s->In[1][21]*OM121+s->In[2][21]*
 OM221+s->In[3][21]*OM321)-s->dpt[1][44]*(Fq122*S22-Fq322*C22)-s->dpt[1][45]*(Fq129*S29-Fq329*C29)-s->dpt[3][44]*(Fq122*C22+
 Fq322*S22)-s->dpt[3][45]*(Fq129*C29+Fq329*S29));
  Cq321 = -(s->trq[3][21]-s->In[3][21]*OMp121-s->In[6][21]*OMp221-s->In[9][21]*OMp321+Cq122*S22+Cq129*S29-Cq322*C22-
 Cq329*C29-Fq222*s->dpt[1][44]-Fq229*s->dpt[1][45]+Fs121*s->l[2][21]-Fs221*s->l[1][21]-OM121*(s->In[2][21]*OM121+s->In[5][21]
 *OM221+s->In[6][21]*OM321)+OM221*(s->In[1][21]*OM121+s->In[2][21]*OM221+s->In[3][21]*OM321)+s->dpt[2][44]*(Fq122*C22+Fq322*
 S22)+s->dpt[2][45]*(Fq129*C29+Fq329*S29));
  Fs120 = -(s->frc[1][20]-s->m[20]*(ALPHA120+BETA320*s->l[3][20]-s->l[1][20]*(OM220*OM220+OM320*OM320)+s->l[2][20]*(
 BS220-OMp320)));
  Fs220 = -(s->frc[2][20]-s->m[20]*(ALPHA219+BETA620*s->l[3][20]+s->l[1][20]*(BS220+OMp320)-s->l[2][20]*(OM120*OM120+
 OM320*OM320)));
  Fs320 = -(s->frc[3][20]-s->m[20]*(ALPHA320+BS920*s->l[3][20]+s->l[1][20]*(BS320-OMp220)+s->l[2][20]*(BS620+OMp120)));
  Fq120 = Fs120+Fq121*C21-Fq221*S21;
  Fq320 = Fs320+Fs321-Fq122*S22-Fq129*S29+Fq322*C22+Fq329*C29;
  Cq120 = -(s->trq[1][20]-s->In[1][20]*OMp120-s->In[2][20]*OMp220-s->In[3][20]*OMp320-Cq121*C21+Cq221*S21+Fs220*
 s->l[3][20]-Fs320*s->l[2][20]-OM220*(s->In[3][20]*OM120+s->In[6][20]*OM220+s->In[9][20]*OM320)+OM320*(s->In[2][20]*OM120+
 s->In[5][20]*OM220+s->In[6][20]*OM320)+s->dpt[3][40]*(Fq121*S21+Fq221*C21));
  Cq220 = -(s->trq[2][20]-s->In[2][20]*OMp120-s->In[5][20]*OMp220-s->In[6][20]*OMp320-Cq121*S21-Cq221*C21-Fs120*
 s->l[3][20]+Fs320*s->l[1][20]+OM120*(s->In[3][20]*OM120+s->In[6][20]*OM220+s->In[9][20]*OM320)-OM320*(s->In[1][20]*OM120+
 s->In[2][20]*OM220+s->In[3][20]*OM320)-s->dpt[3][40]*(Fq121*C21-Fq221*S21));
  Cq320 = -(s->trq[3][20]-Cq321-s->In[3][20]*OMp120-s->In[6][20]*OMp220-s->In[9][20]*OMp320+Fs120*s->l[2][20]-Fs220*
 s->l[1][20]-OM120*(s->In[2][20]*OM120+s->In[5][20]*OM220+s->In[6][20]*OM320)+OM220*(s->In[1][20]*OM120+s->In[2][20]*OM220+
 s->In[3][20]*OM320));
  Fs119 = -(s->frc[1][19]-s->m[19]*(ALPHA119-s->l[1][19]*(OM219*OM219+OM319*OM319)+s->l[2][19]*(BS219-OMp319)+
 s->l[3][19]*(BS319+OMp219)));
  Fs219 = -(s->frc[2][19]-s->m[19]*(ALPHA219+s->l[1][19]*(BS219+OMp319)-s->l[2][19]*(OM119*OM119+OM319*OM319)+
 s->l[3][19]*(BS619-OMp119)));
  Fs319 = -(s->frc[3][19]-s->m[19]*(ALPHA319+s->l[1][19]*(BS319-OMp219)+s->l[2][19]*(BS619+OMp119)-s->l[3][19]*(OM119*
 OM119+OM219*OM219)));
  Fq119 = Fs119+Fq120*C20+Fq320*S20;
  Fq219 = Fs219+Fs220+Fq121*S21+Fq221*C21;
  Fq319 = Fs319-Fq120*S20+Fq320*C20;
  Cq119 = -(s->trq[1][19]-s->In[1][19]*OMp119-s->In[2][19]*OMp219-s->In[3][19]*OMp319-Cq120*C20-Cq320*S20+Fs219*
 s->l[3][19]-Fs319*s->l[2][19]-OM219*(s->In[3][19]*OM119+s->In[6][19]*OM219+s->In[9][19]*OM319)+OM319*(s->In[2][19]*OM119+
 s->In[5][19]*OM219+s->In[6][19]*OM319));
  Cq219 = -(s->trq[2][19]-Cq220-s->In[2][19]*OMp119-s->In[5][19]*OMp219-s->In[6][19]*OMp319-Fs119*s->l[3][19]+Fs319*
 s->l[1][19]+OM119*(s->In[3][19]*OM119+s->In[6][19]*OM219+s->In[9][19]*OM319)-OM319*(s->In[1][19]*OM119+s->In[2][19]*OM219+
 s->In[3][19]*OM319));
  Cq319 = -(s->trq[3][19]-s->In[3][19]*OMp119-s->In[6][19]*OMp219-s->In[9][19]*OMp319+Cq120*S20-Cq320*C20+Fs119*
 s->l[2][19]-Fs219*s->l[1][19]-OM119*(s->In[2][19]*OM119+s->In[5][19]*OM219+s->In[6][19]*OM319)+OM219*(s->In[1][19]*OM119+
 s->In[2][19]*OM219+s->In[3][19]*OM319));
  Fs118 = -(s->frc[1][18]-s->m[18]*(ALPHA117*C18-ALPHA317*S18-s->l[1][18]*(OM218*OM218+OM318*OM318)+s->l[2][18]*(BS218-
 OMp318)+s->l[3][18]*(BS318+OMp218)));
  Fs218 = -(s->frc[2][18]-s->m[18]*(ALPHA217+s->l[1][18]*(BS218+OMp318)-s->l[2][18]*(OM118*OM118+OM318*OM318)+
 s->l[3][18]*(BS618-OMp118)));
  Fs318 = -(s->frc[3][18]-s->m[18]*(ALPHA117*S18+ALPHA317*C18+s->l[1][18]*(BS318-OMp218)+s->l[2][18]*(BS618+OMp118)-
 s->l[3][18]*(OM118*OM118+OM218*OM218)));
  Cq118 = -(s->trq[1][18]-s->In[1][18]*OMp118-s->In[2][18]*OMp218-s->In[3][18]*OMp318+Fs218*s->l[3][18]-Fs318*
 s->l[2][18]-OM218*(s->In[3][18]*OM118+s->In[6][18]*OM218+s->In[9][18]*OM318)+OM318*(s->In[2][18]*OM118+s->In[5][18]*OM218+
 s->In[6][18]*OM318));
  Cq218 = -(s->trq[2][18]-s->In[2][18]*OMp118-s->In[5][18]*OMp218-s->In[6][18]*OMp318-Fs118*s->l[3][18]+Fs318*
 s->l[1][18]+OM118*(s->In[3][18]*OM118+s->In[6][18]*OM218+s->In[9][18]*OM318)-OM318*(s->In[1][18]*OM118+s->In[2][18]*OM218+
 s->In[3][18]*OM318));
  Cq318 = -(s->trq[3][18]-s->In[3][18]*OMp118-s->In[6][18]*OMp218-s->In[9][18]*OMp318+Fs118*s->l[2][18]-Fs218*
 s->l[1][18]-OM118*(s->In[2][18]*OM118+s->In[5][18]*OM218+s->In[6][18]*OM318)+OM218*(s->In[1][18]*OM118+s->In[2][18]*OM218+
 s->In[3][18]*OM318));
  Fs117 = -(s->frc[1][17]-s->m[17]*(ALPHA117-s->l[1][17]*(OM217*OM217+OM317*OM317)+s->l[2][17]*(BS217-OMp317)+
 s->l[3][17]*(BS317+OMp217)));
  Fs217 = -(s->frc[2][17]-s->m[17]*(ALPHA217+s->l[1][17]*(BS217+OMp317)-s->l[2][17]*(OM117*OM117+OM317*OM317)+
 s->l[3][17]*(BS617-OMp117)));
  Fs317 = -(s->frc[3][17]-s->m[17]*(ALPHA317+s->l[1][17]*(BS317-OMp217)+s->l[2][17]*(BS617+OMp117)-s->l[3][17]*(OM117*
 OM117+OM217*OM217)));
  Fq117 = Fs117+Fs118*C18+Fs318*S18;
  Fq217 = Fs217+Fs218;
  Fq317 = Fs317-Fs118*S18+Fs318*C18;
  Cq117 = -(s->trq[1][17]-s->In[1][17]*OMp117-s->In[2][17]*OMp217-s->In[3][17]*OMp317-Cq118*C18-Cq318*S18+Fs217*
 s->l[3][17]-Fs317*s->l[2][17]-OM217*(s->In[3][17]*OM117+s->In[6][17]*OM217+s->In[9][17]*OM317)+OM317*(s->In[2][17]*OM117+
 s->In[5][17]*OM217+s->In[6][17]*OM317));
  Cq217 = -(s->trq[2][17]-Cq218-s->In[2][17]*OMp117-s->In[5][17]*OMp217-s->In[6][17]*OMp317-Fs117*s->l[3][17]+Fs317*
 s->l[1][17]+OM117*(s->In[3][17]*OM117+s->In[6][17]*OM217+s->In[9][17]*OM317)-OM317*(s->In[1][17]*OM117+s->In[2][17]*OM217+
 s->In[3][17]*OM317));
  Cq317 = -(s->trq[3][17]-s->In[3][17]*OMp117-s->In[6][17]*OMp217-s->In[9][17]*OMp317+Cq118*S18-Cq318*C18+Fs117*
 s->l[2][17]-Fs217*s->l[1][17]-OM117*(s->In[2][17]*OM117+s->In[5][17]*OM217+s->In[6][17]*OM317)+OM217*(s->In[1][17]*OM117+
 s->In[2][17]*OM217+s->In[3][17]*OM317));
  Fs116 = -(s->frc[1][16]-s->m[16]*(ALPHA116+BETA316*s->l[3][16]-s->l[1][16]*(OM216*OM216+OM316*OM316)+s->l[2][16]*(
 BS216-OMp316)));
  Fs216 = -(s->frc[2][16]-s->m[16]*(ALPHA216+BETA616*s->l[3][16]+s->l[1][16]*(BS216+OMp316)-s->l[2][16]*(OM116*OM116+
 OM316*OM316)));
  Fs316 = -(s->frc[3][16]-s->m[16]*(ALPHA316+BS916*s->l[3][16]+s->l[1][16]*(BS316-OMp216)+s->l[2][16]*(BS616+OMp116)));
  Fq116 = Fq117+Fs116;
  Fq216 = Fs216+Fq217*C17-Fq317*S17;
  Fq316 = Fs316+Fq217*S17+Fq317*C17;
  Cq116 = -(s->trq[1][16]-Cq117-s->In[1][16]*OMp116-s->In[2][16]*OMp216-s->In[3][16]*OMp316+Fs216*s->l[3][16]-Fs316*
 s->l[2][16]-OM216*(s->In[3][16]*OM116+s->In[6][16]*OM216+s->In[9][16]*OM316)+OM316*(s->In[2][16]*OM116+s->In[5][16]*OM216+
 s->In[6][16]*OM316)+s->dpt[3][28]*(Fq217*C17-Fq317*S17));
  Cq216 = -(s->trq[2][16]-s->In[2][16]*OMp116-s->In[5][16]*OMp216-s->In[6][16]*OMp316-Cq217*C17+Cq317*S17-Fq117*
 s->dpt[3][28]-Fs116*s->l[3][16]+Fs316*s->l[1][16]+OM116*(s->In[3][16]*OM116+s->In[6][16]*OM216+s->In[9][16]*OM316)-OM316*(
 s->In[1][16]*OM116+s->In[2][16]*OM216+s->In[3][16]*OM316));
  Cq316 = -(s->trq[3][16]-s->In[3][16]*OMp116-s->In[6][16]*OMp216-s->In[9][16]*OMp316-Cq217*S17-Cq317*C17+Fs116*
 s->l[2][16]-Fs216*s->l[1][16]-OM116*(s->In[2][16]*OM116+s->In[5][16]*OM216+s->In[6][16]*OM316)+OM216*(s->In[1][16]*OM116+
 s->In[2][16]*OM216+s->In[3][16]*OM316));
  Fs115 = -(s->frc[1][15]-s->m[15]*(ALPHA115+BETA315*s->l[3][15]-s->l[1][15]*(OM215*OM215+OM315*OM315)+s->l[2][15]*(
 BS215-OMp315)));
  Fs215 = -(s->frc[2][15]-s->m[15]*(ALPHA215+BETA615*s->l[3][15]+s->l[1][15]*(BS215+OMp315)-s->l[2][15]*(OM115*OM115+
 OM315*OM315)));
  Fs315 = -(s->frc[3][15]-s->m[15]*(ALPHA315+BS915*s->l[3][15]+s->l[1][15]*(BS315-OMp215)+s->l[2][15]*(BS615+OMp115)));
  Fq115 = Fs115+Fq116*C16+Fq316*S16;
  Fq215 = Fq216+Fs215;
  Cq115 = -(s->trq[1][15]-s->In[1][15]*OMp115-s->In[2][15]*OMp215-s->In[3][15]*OMp315-Cq116*C16-Cq316*S16+Fq216*
 s->dpt[3][26]+Fs215*s->l[3][15]-Fs315*s->l[2][15]-OM215*(s->In[3][15]*OM115+s->In[6][15]*OM215+s->In[9][15]*OM315)+OM315*(
 s->In[2][15]*OM115+s->In[5][15]*OM215+s->In[6][15]*OM315));
  Cq215 = -(s->trq[2][15]-Cq216-s->In[2][15]*OMp115-s->In[5][15]*OMp215-s->In[6][15]*OMp315-Fs115*s->l[3][15]+Fs315*
 s->l[1][15]+OM115*(s->In[3][15]*OM115+s->In[6][15]*OM215+s->In[9][15]*OM315)-OM315*(s->In[1][15]*OM115+s->In[2][15]*OM215+
 s->In[3][15]*OM315)-s->dpt[3][26]*(Fq116*C16+Fq316*S16));
  Cq315 = -(s->trq[3][15]-s->In[3][15]*OMp115-s->In[6][15]*OMp215-s->In[9][15]*OMp315+Cq116*S16-Cq316*C16+Fs115*
 s->l[2][15]-Fs215*s->l[1][15]-OM115*(s->In[2][15]*OM115+s->In[5][15]*OM215+s->In[6][15]*OM315)+OM215*(s->In[1][15]*OM115+
 s->In[2][15]*OM215+s->In[3][15]*OM315));
  Fs114 = -(s->frc[1][14]-s->m[14]*(ALPHA114+BETA314*s->l[3][14]-s->l[1][14]*(OM214*OM214+OM314*OM314)+s->l[2][14]*(
 BS214-OMp314)));
  Fs214 = -(s->frc[2][14]-s->m[14]*(ALPHA214+BETA614*s->l[3][14]+s->l[1][14]*(BS214+OMp314)-s->l[2][14]*(OM114*OM114+
 OM314*OM314)));
  Fs314 = -(s->frc[3][14]-s->m[14]*(ALPHA314+BS914*s->l[3][14]+s->l[1][14]*(BS314-OMp214)+s->l[2][14]*(BS614+OMp114)));
  Fq114 = Fs114+Fq115*C15-Fq215*S15;
  Fq214 = Fs214+Fq115*S15+Fq215*C15;
  Fq314 = Fs314+Fs315-Fq116*S16+Fq316*C16;
  Cq114 = -(s->trq[1][14]-s->In[1][14]*OMp114-s->In[2][14]*OMp214-s->In[3][14]*OMp314-Cq115*C15+Cq215*S15+Fs214*
 s->l[3][14]-Fs314*s->l[2][14]-OM214*(s->In[3][14]*OM114+s->In[6][14]*OM214+s->In[9][14]*OM314)+OM314*(s->In[2][14]*OM114+
 s->In[5][14]*OM214+s->In[6][14]*OM314)+s->dpt[3][24]*(Fq115*S15+Fq215*C15));
  Cq214 = -(s->trq[2][14]-s->In[2][14]*OMp114-s->In[5][14]*OMp214-s->In[6][14]*OMp314-Cq115*S15-Cq215*C15-Fs114*
 s->l[3][14]+Fs314*s->l[1][14]+OM114*(s->In[3][14]*OM114+s->In[6][14]*OM214+s->In[9][14]*OM314)-OM314*(s->In[1][14]*OM114+
 s->In[2][14]*OM214+s->In[3][14]*OM314)-s->dpt[3][24]*(Fq115*C15-Fq215*S15));
  Cq314 = -(s->trq[3][14]-Cq315-s->In[3][14]*OMp114-s->In[6][14]*OMp214-s->In[9][14]*OMp314+Fs114*s->l[2][14]-Fs214*
 s->l[1][14]-OM114*(s->In[2][14]*OM114+s->In[5][14]*OM214+s->In[6][14]*OM314)+OM214*(s->In[1][14]*OM114+s->In[2][14]*OM214+
 s->In[3][14]*OM314));
  Fs113 = -(s->frc[1][13]-s->m[13]*(ALPHA113+BETA213*s->l[2][13]-s->l[1][13]*(OM213*OM213+OM313*OM313)+s->l[3][13]*(
 BS313+OMp213)));
  Fs213 = -(s->frc[2][13]-s->m[13]*(ALPHA213+BS513*s->l[2][13]+s->l[1][13]*(BS213+OMp313)+s->l[3][13]*(BS613-OMp113)));
  Fs313 = -(s->frc[3][13]-s->m[13]*(ALPHA313+BETA813*s->l[2][13]+s->l[1][13]*(BS313-OMp213)-s->l[3][13]*(OM113*OM113+
 OM213*OM213)));
  Fq113 = Fq114+Fs113;
  Fq313 = Fs313+Fq214*S14+Fq314*C14;
  Cq113 = -(s->trq[1][13]-Cq114-s->In[1][13]*OMp113-s->In[2][13]*OMp213-s->In[3][13]*OMp313+Fs213*s->l[3][13]-Fs313*
 s->l[2][13]-OM213*(s->In[3][13]*OM113+s->In[6][13]*OM213+s->In[9][13]*OM313)+OM313*(s->In[2][13]*OM113+s->In[5][13]*OM213+
 s->In[6][13]*OM313)-s->dpt[2][22]*(Fq214*S14+Fq314*C14));
  Cq213 = -(s->trq[2][13]-s->In[2][13]*OMp113-s->In[5][13]*OMp213-s->In[6][13]*OMp313-Cq214*C14+Cq314*S14-Fs113*
 s->l[3][13]+Fs313*s->l[1][13]+OM113*(s->In[3][13]*OM113+s->In[6][13]*OM213+s->In[9][13]*OM313)-OM313*(s->In[1][13]*OM113+
 s->In[2][13]*OM213+s->In[3][13]*OM313));
  Cq313 = -(s->trq[3][13]-s->In[3][13]*OMp113-s->In[6][13]*OMp213-s->In[9][13]*OMp313-Cq214*S14-Cq314*C14+Fq114*
 s->dpt[2][22]+Fs113*s->l[2][13]-Fs213*s->l[1][13]-OM113*(s->In[2][13]*OM113+s->In[5][13]*OM213+s->In[6][13]*OM313)+OM213*(
 s->In[1][13]*OM113+s->In[2][13]*OM213+s->In[3][13]*OM313));
  Fs112 = -(s->frc[1][12]-s->m[12]*(ALPHA111*C12-ALPHA311*S12-s->l[1][12]*(OM212*OM212+OM312*OM312)+s->l[2][12]*(BS212-
 OMp312)+s->l[3][12]*(BS312+OMp212)));
  Fs212 = -(s->frc[2][12]-s->m[12]*(ALPHA211+s->l[1][12]*(BS212+OMp312)-s->l[2][12]*(OM112*OM112+OM312*OM312)+
 s->l[3][12]*(BS612-OMp112)));
  Fs312 = -(s->frc[3][12]-s->m[12]*(ALPHA111*S12+ALPHA311*C12+s->l[1][12]*(BS312-OMp212)+s->l[2][12]*(BS612+OMp112)-
 s->l[3][12]*(OM112*OM112+OM212*OM212)));
  Cq112 = -(s->trq[1][12]-s->In[1][12]*OMp112-s->In[2][12]*OMp212-s->In[3][12]*OMp312+Fs212*s->l[3][12]-Fs312*
 s->l[2][12]-OM212*(s->In[3][12]*OM112+s->In[6][12]*OM212+s->In[9][12]*OM312)+OM312*(s->In[2][12]*OM112+s->In[5][12]*OM212+
 s->In[6][12]*OM312));
  Cq212 = -(s->trq[2][12]-s->In[2][12]*OMp112-s->In[5][12]*OMp212-s->In[6][12]*OMp312-Fs112*s->l[3][12]+Fs312*
 s->l[1][12]+OM112*(s->In[3][12]*OM112+s->In[6][12]*OM212+s->In[9][12]*OM312)-OM312*(s->In[1][12]*OM112+s->In[2][12]*OM212+
 s->In[3][12]*OM312));
  Cq312 = -(s->trq[3][12]-s->In[3][12]*OMp112-s->In[6][12]*OMp212-s->In[9][12]*OMp312+Fs112*s->l[2][12]-Fs212*
 s->l[1][12]-OM112*(s->In[2][12]*OM112+s->In[5][12]*OM212+s->In[6][12]*OM312)+OM212*(s->In[1][12]*OM112+s->In[2][12]*OM212+
 s->In[3][12]*OM312));
  Fs111 = -(s->frc[1][11]-s->m[11]*(ALPHA111-s->l[1][11]*(OM211*OM211+OM311*OM311)+s->l[2][11]*(BS211-OMp311)+
 s->l[3][11]*(BS311+OMp211)));
  Fs211 = -(s->frc[2][11]-s->m[11]*(ALPHA211+s->l[1][11]*(BS211+OMp311)-s->l[2][11]*(OM111*OM111+OM311*OM311)+
 s->l[3][11]*(BS611-OMp111)));
  Fs311 = -(s->frc[3][11]-s->m[11]*(ALPHA311+s->l[1][11]*(BS311-OMp211)+s->l[2][11]*(BS611+OMp111)-s->l[3][11]*(OM111*
 OM111+OM211*OM211)));
  Fq111 = Fs111+Fs112*C12+Fs312*S12;
  Fq211 = Fs211+Fs212;
  Fq311 = Fs311-Fs112*S12+Fs312*C12;
  Cq111 = -(s->trq[1][11]-s->In[1][11]*OMp111-s->In[2][11]*OMp211-s->In[3][11]*OMp311-Cq112*C12-Cq312*S12+Fs211*
 s->l[3][11]-Fs311*s->l[2][11]-OM211*(s->In[3][11]*OM111+s->In[6][11]*OM211+s->In[9][11]*OM311)+OM311*(s->In[2][11]*OM111+
 s->In[5][11]*OM211+s->In[6][11]*OM311));
  Cq211 = -(s->trq[2][11]-Cq212-s->In[2][11]*OMp111-s->In[5][11]*OMp211-s->In[6][11]*OMp311-Fs111*s->l[3][11]+Fs311*
 s->l[1][11]+OM111*(s->In[3][11]*OM111+s->In[6][11]*OM211+s->In[9][11]*OM311)-OM311*(s->In[1][11]*OM111+s->In[2][11]*OM211+
 s->In[3][11]*OM311));
  Cq311 = -(s->trq[3][11]-s->In[3][11]*OMp111-s->In[6][11]*OMp211-s->In[9][11]*OMp311+Cq112*S12-Cq312*C12+Fs111*
 s->l[2][11]-Fs211*s->l[1][11]-OM111*(s->In[2][11]*OM111+s->In[5][11]*OM211+s->In[6][11]*OM311)+OM211*(s->In[1][11]*OM111+
 s->In[2][11]*OM211+s->In[3][11]*OM311));
  Fs110 = -(s->frc[1][10]-s->m[10]*(ALPHA110+BETA310*s->l[3][10]-s->l[1][10]*(OM210*OM210+OM310*OM310)+s->l[2][10]*(
 BS210-OMp310)));
  Fs210 = -(s->frc[2][10]-s->m[10]*(ALPHA210+BETA610*s->l[3][10]+s->l[1][10]*(BS210+OMp310)-s->l[2][10]*(OM110*OM110+
 OM310*OM310)));
  Fs310 = -(s->frc[3][10]-s->m[10]*(ALPHA310+BS910*s->l[3][10]+s->l[1][10]*(BS310-OMp210)+s->l[2][10]*(BS610+OMp110)));
  Fq110 = Fq111+Fs110;
  Fq210 = Fs210+Fq211*C11-Fq311*S11;
  Fq310 = Fs310+Fq211*S11+Fq311*C11;
  Cq110 = -(s->trq[1][10]-Cq111-s->In[1][10]*OMp110-s->In[2][10]*OMp210-s->In[3][10]*OMp310+Fs210*s->l[3][10]-Fs310*
 s->l[2][10]-OM210*(s->In[3][10]*OM110+s->In[6][10]*OM210+s->In[9][10]*OM310)+OM310*(s->In[2][10]*OM110+s->In[5][10]*OM210+
 s->In[6][10]*OM310)+s->dpt[3][12]*(Fq211*C11-Fq311*S11));
  Cq210 = -(s->trq[2][10]-s->In[2][10]*OMp110-s->In[5][10]*OMp210-s->In[6][10]*OMp310-Cq211*C11+Cq311*S11-Fq111*
 s->dpt[3][12]-Fs110*s->l[3][10]+Fs310*s->l[1][10]+OM110*(s->In[3][10]*OM110+s->In[6][10]*OM210+s->In[9][10]*OM310)-OM310*(
 s->In[1][10]*OM110+s->In[2][10]*OM210+s->In[3][10]*OM310));
  Cq310 = -(s->trq[3][10]-s->In[3][10]*OMp110-s->In[6][10]*OMp210-s->In[9][10]*OMp310-Cq211*S11-Cq311*C11+Fs110*
 s->l[2][10]-Fs210*s->l[1][10]-OM110*(s->In[2][10]*OM110+s->In[5][10]*OM210+s->In[6][10]*OM310)+OM210*(s->In[1][10]*OM110+
 s->In[2][10]*OM210+s->In[3][10]*OM310));
  Fs19 = -(s->frc[1][9]-s->m[9]*(ALPHA19+BETA39*s->l[3][9]-s->l[1][9]*(OM29*OM29+OM39*OM39)+s->l[2][9]*(BS29-OMp39)));
  Fs29 = -(s->frc[2][9]-s->m[9]*(ALPHA29+BETA69*s->l[3][9]+s->l[1][9]*(BS29+OMp39)-s->l[2][9]*(OM19*OM19+OM39*OM39)));
  Fs39 = -(s->frc[3][9]-s->m[9]*(ALPHA39+BS99*s->l[3][9]+s->l[1][9]*(BS39-OMp29)+s->l[2][9]*(BS69+OMp19)));
  Fq19 = Fs19+Fq110*C10+Fq310*S10;
  Fq29 = Fq210+Fs29;
  Cq19 = -(s->trq[1][9]-s->In[1][9]*OMp19-s->In[2][9]*OMp29-s->In[3][9]*OMp39-Cq110*C10-Cq310*S10+Fq210*s->dpt[3][10]+
 Fs29*s->l[3][9]-Fs39*s->l[2][9]-OM29*(s->In[3][9]*OM19+s->In[6][9]*OM29+s->In[9][9]*OM39)+OM39*(s->In[2][9]*OM19+s->In[5][9]
 *OM29+s->In[6][9]*OM39));
  Cq29 = -(s->trq[2][9]-Cq210-s->In[2][9]*OMp19-s->In[5][9]*OMp29-s->In[6][9]*OMp39-Fs19*s->l[3][9]+Fs39*s->l[1][9]+OM19
 *(s->In[3][9]*OM19+s->In[6][9]*OM29+s->In[9][9]*OM39)-OM39*(s->In[1][9]*OM19+s->In[2][9]*OM29+s->In[3][9]*OM39)-
 s->dpt[3][10]*(Fq110*C10+Fq310*S10));
  Cq39 = -(s->trq[3][9]-s->In[3][9]*OMp19-s->In[6][9]*OMp29-s->In[9][9]*OMp39+Cq110*S10-Cq310*C10+Fs19*s->l[2][9]-Fs29*
 s->l[1][9]-OM19*(s->In[2][9]*OM19+s->In[5][9]*OM29+s->In[6][9]*OM39)+OM29*(s->In[1][9]*OM19+s->In[2][9]*OM29+s->In[3][9]*
 OM39));
  Fs18 = -(s->frc[1][8]-s->m[8]*(ALPHA18+BETA38*s->l[3][8]-s->l[1][8]*(OM28*OM28+OM38*OM38)+s->l[2][8]*(BS28-OMp38)));
  Fs28 = -(s->frc[2][8]-s->m[8]*(ALPHA28+BETA68*s->l[3][8]+s->l[1][8]*(BS28+OMp38)-s->l[2][8]*(OM18*OM18+OM38*OM38)));
  Fs38 = -(s->frc[3][8]-s->m[8]*(ALPHA38+BS98*s->l[3][8]+s->l[1][8]*(BS38-OMp28)+s->l[2][8]*(BS68+OMp18)));
  Fq18 = Fs18+Fq19*C9-Fq29*S9;
  Fq28 = Fs28+Fq19*S9+Fq29*C9;
  Fq38 = Fs38+Fs39-Fq110*S10+Fq310*C10;
  Cq18 = -(s->trq[1][8]-s->In[1][8]*OMp18-s->In[2][8]*OMp28-s->In[3][8]*OMp38-Cq19*C9+Cq29*S9+Fs28*s->l[3][8]-Fs38*
 s->l[2][8]-OM28*(s->In[3][8]*OM18+s->In[6][8]*OM28+s->In[9][8]*OM38)+OM38*(s->In[2][8]*OM18+s->In[5][8]*OM28+s->In[6][8]*
 OM38)+s->dpt[3][8]*(Fq19*S9+Fq29*C9));
  Cq28 = -(s->trq[2][8]-s->In[2][8]*OMp18-s->In[5][8]*OMp28-s->In[6][8]*OMp38-Cq19*S9-Cq29*C9-Fs18*s->l[3][8]+Fs38*
 s->l[1][8]+OM18*(s->In[3][8]*OM18+s->In[6][8]*OM28+s->In[9][8]*OM38)-OM38*(s->In[1][8]*OM18+s->In[2][8]*OM28+s->In[3][8]*
 OM38)-s->dpt[3][8]*(Fq19*C9-Fq29*S9));
  Cq38 = -(s->trq[3][8]-Cq39-s->In[3][8]*OMp18-s->In[6][8]*OMp28-s->In[9][8]*OMp38+Fs18*s->l[2][8]-Fs28*s->l[1][8]-OM18*
 (s->In[2][8]*OM18+s->In[5][8]*OM28+s->In[6][8]*OM38)+OM28*(s->In[1][8]*OM18+s->In[2][8]*OM28+s->In[3][8]*OM38));
  Fs17 = -(s->frc[1][7]-s->m[7]*(ALPHA17+BETA27*s->l[2][7]-s->l[1][7]*(OM27*OM27+OM37*OM37)+s->l[3][7]*(BS37+OMp27)));
  Fs27 = -(s->frc[2][7]-s->m[7]*(ALPHA27+BS57*s->l[2][7]+s->l[1][7]*(BS27+OMp37)+s->l[3][7]*(BS67-OMp17)));
  Fs37 = -(s->frc[3][7]-s->m[7]*(ALPHA37+BETA87*s->l[2][7]+s->l[1][7]*(BS37-OMp27)-s->l[3][7]*(OM17*OM17+OM27*OM27)));
  Fq17 = Fq18+Fs17;
  Fq37 = Fs37+Fq28*S8+Fq38*C8;
  Cq17 = -(s->trq[1][7]-Cq18-s->In[1][7]*OMp17-s->In[2][7]*OMp27-s->In[3][7]*OMp37+Fs27*s->l[3][7]-Fs37*s->l[2][7]-OM27*
 (s->In[3][7]*OM17+s->In[6][7]*OM27+s->In[9][7]*OM37)+OM37*(s->In[2][7]*OM17+s->In[5][7]*OM27+s->In[6][7]*OM37)-s->dpt[2][6]*
 (Fq28*S8+Fq38*C8));
  Cq27 = -(s->trq[2][7]-s->In[2][7]*OMp17-s->In[5][7]*OMp27-s->In[6][7]*OMp37-Cq28*C8+Cq38*S8-Fs17*s->l[3][7]+Fs37*
 s->l[1][7]+OM17*(s->In[3][7]*OM17+s->In[6][7]*OM27+s->In[9][7]*OM37)-OM37*(s->In[1][7]*OM17+s->In[2][7]*OM27+s->In[3][7]*
 OM37));
  Cq37 = -(s->trq[3][7]-s->In[3][7]*OMp17-s->In[6][7]*OMp27-s->In[9][7]*OMp37-Cq28*S8-Cq38*C8+Fq18*s->dpt[2][6]+Fs17*
 s->l[2][7]-Fs27*s->l[1][7]-OM17*(s->In[2][7]*OM17+s->In[5][7]*OM27+s->In[6][7]*OM37)+OM27*(s->In[1][7]*OM17+s->In[2][7]*OM27
 +s->In[3][7]*OM37));
  Fs16 = -(s->frc[1][6]-s->m[6]*(ALPHA16+BETA26*s->l[2][6]+BETA36*s->l[3][6]+BS16*s->l[1][6]));
  Fs26 = -(s->frc[2][6]-s->m[6]*(ALPHA26+BETA46*s->l[1][6]+BETA66*s->l[3][6]+BS56*s->l[2][6]));
  Fs36 = -(s->frc[3][6]-s->m[6]*(ALPHA35+BETA76*s->l[1][6]+BETA86*s->l[2][6]+BS96*s->l[3][6]));
  Fq16 = Fq119+Fs16+Fq113*C13+Fq17*C7+Fq313*S13+Fq37*S7;
  Fq26 = Fs213+Fs26+Fs27+Fq214*C14+Fq219*C19+Fq28*C8-Fq314*S14-Fq319*S19-Fq38*S8;
  Fq36 = Fs36-Fq113*S13-Fq17*S7+Fq219*S19+Fq313*C13+Fq319*C19+Fq37*C7;
  Cq16 = -(s->trq[1][6]-Cq119-s->In[1][6]*OMp16-s->In[2][6]*OMp26-s->In[3][6]*OMp36-Cq113*C13-Cq17*C7-Cq313*S13-Cq37*S7+
 Fs26*s->l[3][6]-Fs36*s->l[2][6]-OM26*(s->In[3][6]*OM16+s->In[6][6]*OM26+s->In[9][6]*OM36)+OM36*(s->In[2][6]*OM16+s->In[5][6]
 *OM26+s->In[6][6]*OM36)+s->dpt[2][1]*(Fq17*S7-Fq37*C7)+s->dpt[2][2]*(Fq113*S13-Fq313*C13)+s->dpt[3][3]*(Fq219*C19-Fq319*S19)
 );
  Cq26 = -(s->trq[2][6]-Cq213-Cq27-s->In[2][6]*OMp16-s->In[5][6]*OMp26-s->In[6][6]*OMp36-Cq219*C19+Cq319*S19-Fq119*
 s->dpt[3][3]-Fs16*s->l[3][6]+Fs36*s->l[1][6]+OM16*(s->In[3][6]*OM16+s->In[6][6]*OM26+s->In[9][6]*OM36)-OM36*(s->In[1][6]*
 OM16+s->In[2][6]*OM26+s->In[3][6]*OM36)+s->dpt[1][3]*(Fq219*S19+Fq319*C19));
  Cq36 = -(s->trq[3][6]-s->In[3][6]*OMp16-s->In[6][6]*OMp26-s->In[9][6]*OMp36+Cq113*S13+Cq17*S7-Cq219*S19-Cq313*C13-
 Cq319*C19-Cq37*C7+Fs16*s->l[2][6]-Fs26*s->l[1][6]-OM16*(s->In[2][6]*OM16+s->In[5][6]*OM26+s->In[6][6]*OM36)+OM26*(
 s->In[1][6]*OM16+s->In[2][6]*OM26+s->In[3][6]*OM36)-s->dpt[1][3]*(Fq219*C19-Fq319*S19)+s->dpt[2][1]*(Fq17*C7+Fq37*S7)+
 s->dpt[2][2]*(Fq113*C13+Fq313*S13));
  Fq15 = Fq16*C6-Fq26*S6;
  Fq25 = Fq16*S6+Fq26*C6;
  Cq25 = Cq16*S6+Cq26*C6;
  Fq14 = Fq15*C5+Fq36*S5;
  Fq34 = -(Fq15*S5-Fq36*C5);
  Cq14 = Cq36*S5+C5*(Cq16*C6-Cq26*S6);
  Fq23 = Fq25*C4-Fq34*S4;
  Fq33 = Fq25*S4+Fq34*C4;

// = = Block_0_2_0_0_0_0 = = 
 
// Symbolic Outputs  

  Qq[1] = Fq14;
  Qq[2] = Fq23;
  Qq[3] = Fq33;
  Qq[4] = Cq14;
  Qq[5] = Cq25;
  Qq[6] = Cq36;
  Qq[7] = Cq27;
  Qq[8] = Cq18;
  Qq[9] = Cq39;
  Qq[10] = Cq210;
  Qq[11] = Cq111;
  Qq[12] = Cq212;
  Qq[13] = Cq213;
  Qq[14] = Cq114;
  Qq[15] = Cq315;
  Qq[16] = Cq216;
  Qq[17] = Cq117;
  Qq[18] = Cq218;
  Qq[19] = Cq119;
  Qq[20] = Cq220;
  Qq[21] = Cq321;
  Qq[22] = Cq222;
  Qq[23] = Cq123;
  Qq[24] = Cq224;
  Qq[25] = Cq325;
  Qq[26] = Cq226;
  Qq[27] = Cq327;
  Qq[28] = Cq128;
  Qq[29] = Cq229;
  Qq[30] = Cq130;
  Qq[31] = Cq231;
  Qq[32] = Cq332;
  Qq[33] = Cq233;
  Qq[34] = Cq334;
  Qq[35] = Cq135;

// ====== END Task 0 ====== 


}
 

