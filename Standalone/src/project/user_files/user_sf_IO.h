/*===========================================================================*
 *
 *  user_sf_IO.h
 * 
 *  Generation date: Fri Jul 11 10:52:32 2014

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
 
typedef struct UserIOStruct 
{
    double tsim_out1;
    double output1[29+1];
    double output2[29+1];
    double Voltage[6+1];
    double refs[6+1];
    int servo_type[6+1];
    ControllerStruct *cvs;
    SimbodyStruct *simbodyStruct;
    ActuatorsStruct *actuatorsStruct;

} UserIOStruct;

/*--------------------*/
#endif
