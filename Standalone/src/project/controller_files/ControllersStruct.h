//---------------------------
// Nicolas Van der Noot
//
// Creation : 19-Sep-2013
// Last update : Wed May 21 16:40:42 2014
//---------------------------

#ifndef ControllerStruct_h
#define ControllerStruct_h


// ---- Structures definitions (typedef) ---- //

// ControllerInputsStructStruc
typedef struct ControllerInputsStruct
{
    double tsim;

} ControllerInputsStruct;


// ControllerOutputsStructStruc
typedef struct ControllerOutputsStruct
{
    double q_ref[4];
    double qd_ref[4];
    double qdd_ref[4];

} ControllerOutputsStruct;

// Defining PID struct (Joint Specific)
/* PID structures used with/without YARP in Simulation*/
typedef struct ControllerPIDs
{
    double p[30];
    double i[30];
    double d[30];
    double maxInt[30];
    double maxOut[30];
} ControllerPIDs ;

// ControllerStructStruc
typedef struct ControllerStruct
{
    struct ControllerInputsStruct *Inputs;
    struct ControllerOutputsStruct *Outputs;
    struct ControllerPIDs *PIDs;

} ControllerStruct;


// ---- Init and free functions: declarations ---- //

ControllerInputsStruct * init_ControllerInputsStruct(void);
void free_ControllerInputsStruct(ControllerInputsStruct *cvs);

ControllerOutputsStruct * init_ControllerOutputsStruct(void);
void free_ControllerOutputsStruct(ControllerOutputsStruct *cvs);

ControllerPIDs *init_ControllerPIDs(void);
void free_ControllerPIDs(ControllerPIDs *cvs);

ControllerStruct * init_ControllerStruct(void);
void free_ControllerStruct(ControllerStruct *cvs);

/*--------------------*/
#endif

