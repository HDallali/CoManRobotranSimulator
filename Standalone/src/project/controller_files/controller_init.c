//---------------------------
// Creation : ../../2013
// Last update : ../../2013
//---------------------------

#include "controller_def.h"

void controller_init(ControllerStruct *cvs)
{
    int i;

    cvs->PIDs->p[0]=0.0;
    cvs->PIDs->d[0]=0.0;
    cvs->PIDs->i[0]=0.0;

    #ifndef YARP
    for (i=1; i<30; i++)
        cvs->PIDs->p[i]=10.0;
        cvs->PIDs->d[i]=0.1;
        cvs->PIDs->i[i]=0.0;
    #else
    for (i=1; i<30; i++)
        cvs->PIDs->p[i]=0;
        cvs->PIDs->d[i]=0;
        cvs->PIDs->i[i]=0;
    #endif

}
