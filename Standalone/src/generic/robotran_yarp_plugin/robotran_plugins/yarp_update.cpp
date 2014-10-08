#if defined(YARP) & defined(__cplusplus)

#include "yarp_files.h"
#include <iostream>

using namespace std;

void updateDataFromYarp(void* RobotranYarp_interface, MBSdataStruct * MBSdata)
{
	// here should come the update from yarp to the simulator
	// write the new references (torque, position, speed ...) desired by the ControlInterface
	// - get references
	// - feed low leved PID controllers with the ref

	//cout << "update data from yarp " << endl;
    yarp::dev::PolyDriverList *controlBoardList = (yarp::dev::PolyDriverList*)RobotranYarp_interface;  // convert back into object
    robotran::IRobotran                         *RobotranPlugin          	= NULL;


    if(controlBoardList == NULL)
        return;

    for(int i=0; i < controlBoardList->size(); i++)
    {
        (*controlBoardList)[i]->poly->view(RobotranPlugin);
        if(RobotranPlugin)
        {
//            printf("I found a valid plugin at %d \n\n", i);
            RobotranPlugin->updateFromYarp(MBSdata);
        }

        // for controller too?
    }
}

// here should come the update from simulator to yarp
// write the new sensor state computed by simulation
// - sensor_driver.Update()
void updateDataToYarp(void* RobotranYarp_interface, const MBSdataStruct * MBSdata)
{
	yarp::dev::PolyDriverList *controlBoardList = (yarp::dev::PolyDriverList*)RobotranYarp_interface;  // convert back into object
    robotran::IRobotran                         *RobotranPlugin          	= NULL;

    if(controlBoardList == NULL)
        return;

	for(int i=0; i < controlBoardList->size(); i++)
    {
        (*controlBoardList)[i]->poly->view(RobotranPlugin);
        if(RobotranPlugin)
        {
//            printf("I found a valid plugin at %d \n\n", i);
            RobotranPlugin->updateToYarp(MBSdata);
        }
    }
}

#endif
