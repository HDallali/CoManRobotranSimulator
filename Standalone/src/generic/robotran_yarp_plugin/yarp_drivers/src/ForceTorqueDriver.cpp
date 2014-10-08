/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifdef YARP

/// general purpose stuff.

#include <yarp/os/Time.h>
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>

#include <string>
#include <iostream>
#include <iterator>
#include <string.h>


/// specific to this device driver.
#include <ForceTorqueDriver.h>


#ifdef WIN32
#pragma warning(once:4355)
#endif

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    return false;
}

//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
static inline bool validate(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;  // size includes also the name of the parameter
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        fprintf(stderr, "%s not found\n", key1.c_str());
        return false;
    }

    if( tmp.size() != size)
    {
        fprintf(stderr, "%s incorrect number of entries\n", key1.c_str());
        return false;
    }

    out=tmp;

    return true;
}


RobotranYarpForceTorqueDriver::RobotranYarpForceTorqueDriver() : _verbose(false),
                                                                 _useCalibration(0),
                                                                 _channels(0)
{

    status=IAnalogSensor::AS_OK;
}

RobotranYarpForceTorqueDriver::~RobotranYarpForceTorqueDriver()
{

}

bool RobotranYarpForceTorqueDriver::open(yarp::os::Searchable &config)
{
    if(config.check("verbose"))
    {
        std::cout << "INFO: verbose output enabled" << std::endl;
        _verbose = true;
    }

    if(_verbose)
        std::cout << "RobotranYarpForceTorqueDriver::open with parameters \n " << config.toString().c_str() << std::endl;


    if(!fromConfig(config))
        return false;


     // opening Wrapper, if needed
     if(!config.check("useWrapper"))
     {
         std::cout << "\nNOT USING WRAPPERS!\n" << std::endl;
         return true;
     }

     std::cout << "\nForceTorque WRAPPERS!\n" << std::endl;

     yarp::dev::IMultipleWrapper* iWrap;
     wrap = new yarp::dev::PolyDriver();
     yarp::os::Property wrapProp;
     wrapProp.fromString(config.toString());
     yarp::os::ConstString partName, robotName, wholeName;
     partName = config.find("name").asString();
     robotName = config.find("robot").asString();
     wrapProp.unput("device");
     wrapProp.put("device", "analogServer");
     wrapProp.unput("name");

     wholeName = robotName + "/" + partName;
     wrapProp.put("name", wholeName);


     std::cout << "robotName is " << robotName << "; partName is " << partName << "; wholeName is " << wholeName << std::endl;

     std::cout << "\n*********\n before wrapper " << wrapProp.toString() << "\n***********\n" << std::endl;

     wrap->open(wrapProp);
     if (!wrap->isValid())
         fprintf(stderr, "RobotranYarpForceTorqueDriver: wrapper did not open\n");
     else
         fprintf(stderr, "RobotranYarpForceTorqueDriver: wrapper opened correctly\n");

     if (!wrap->view(iWrap)) {
         printf("RobotranYarpForceTorqueDriver Wrapper interface not found\n");
     }

    yarp::dev::PolyDriverList polyList;
    yarp::os::Bottle *netList = config.find("networks").asList();
    if (netList == NULL) {
        printf("RobotranYarpForceTorqueDriver ERROR, net list to attach to was not found, exiting\n");
        wrap->close();
        return false;
    }
    std::cout << "Analog list found " << netList->toString() << "!!" << std::endl;

    polyList.push((yarp::dev::PolyDriver*) this, netList->get(0).asString().c_str() );
    if(!iWrap->attachAll(polyList) )
    {
        std::cout << "\n\n\nERROR while attching\n\n\n" << std::endl;
        return false;
     }
    else
    {
        std::cout << "\n ATTACH WAS OK\n" << std::endl;
    }

    return true;
}


bool RobotranYarpForceTorqueDriver::fromConfig(yarp::os::Searchable &_config)
{
//    if(!_config.check("channels"))
//    {
//        std::cout << "ERROR: the number of channels is missing" << std::endl;
//        return false;
//    }
//    _channels = _config.find("channels").asInt();

    _channels = 6;
    data.resize(_channels); // number of DoFs

//    // Get joints id
//    if(!_config.check("robotran_joint_id"))
//    {
//        std::cout << "robotran joints id not specified in config file " << std::endl;
//        return false;
//    }
//    yarp::os::Bottle & jointID = _config.findGroup("robotran_joint_id");
//    jointID_map.resize(_channels);
//    for(int i=0; i< jointID.size()-1; i++)
//    {
//        jointID_map[i] = jointID.get(i+1).asInt();
//        printf("jointID_map[%d] = %d \n", i, jointID_map[i]);
//    }

//    // Get motor id
//    if(!_config.check("robotran_motor_id"))
//    {
//        std::cout << "robotran_motor_id parameter is not specified in config file " << std::endl;
//        return false;
//    }
//    yarp::os::Bottle & motorID = _config.findGroup("robotran_motor_id");
//    motorID_map.resize(_channels);
//    for(int i=0; i< motorID.size()-1; i++)
//    {
//        motorID_map[i] = motorID.get(i+1).asInt();
//        printf("motorID_map[%d] = %d \n", i, motorID_map[i]);
//    }

    if(!_config.check("deviceId"))
    {
        std::cout << "ERROR: please insert deviceId as left_leg or right_leg" << std::endl;
        return false;
    }
    partName = _config.find("deviceId").asString();

    if( partName == ("left_leg"))
    {
        part = LEFT_LEG;
    }
    else if(partName == ("right_leg"))
    {
        part = LEFT_LEG;
    }
    else
    {
        std::cout << "part " << partName << " provided is not supported yet" << std::endl;
        return false;
    }


    // newtonsToSensor scaling factor
    newtonToSensor.resize(_channels);
    if(!_config.check("newtonsToSensor"))
    {
        for(int i=0; i< _channels; i++)
        {
            std::cout << "warning: robotran newtonsToSensor joints values not specified in config file, using '+1' as a default value " << std::endl;
            newtonToSensor[i] = 1.0;
            printf("newtonToSensor[%d] = %d \n", i, newtonToSensor[i]);
        }
    }
    else
    {
        yarp::os::Bottle & newtonToSensorBottle = _config.findGroup("newtonsToSensor");
        for(int i=0; i< newtonToSensorBottle.size()-1; i++)
        {
            newtonToSensor[i] = newtonToSensorBottle.get(i+1).asInt();
            printf("encoder[%d] = %d \n", i, newtonToSensor[i]);
        }
    }
    return true;
}

bool RobotranYarpForceTorqueDriver::close()
{
    return true;
}

/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/
int RobotranYarpForceTorqueDriver::read(yarp::sig::Vector &out)
{
    // This method gives data to the analogServer

    mutex.wait();
    status = AS_OK;

    out.resize(data.size());

    out = data;
    mutex.post();
    return status;
}

int RobotranYarpForceTorqueDriver::getState(int ch)
{
    printf("getstate\n");
    return AS_OK;
}

int RobotranYarpForceTorqueDriver::getChannels()
{
     return _channels;
}

int RobotranYarpForceTorqueDriver::calibrateSensor()
{
    return AS_OK;
}

int RobotranYarpForceTorqueDriver::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int RobotranYarpForceTorqueDriver::calibrateChannel(int ch)
{
    return AS_OK;
}

int RobotranYarpForceTorqueDriver::calibrateChannel(int ch, double v)
{
    return AS_OK;
}


void RobotranYarpForceTorqueDriver::updateToYarp(const MBSdataStruct * MBSdata)
{
    switch(part)
    {
        case LEFT_LEG:
        {
            for (int i=0; i<3; i++)
                data[i] = MBSdata->user_IO->GRF_l[i+1];

            for (int i=0; i<3; i++)
                data[3+i] = MBSdata->user_IO->GRM_l[i+1];

            break;
        }

        case RIGHT_LEG:
        {
            for (int i=0; i<3; i++)
                data[i] = MBSdata->user_IO->GRF_r[i+1];

            for (int i=0; i<3; i++)
                data[3+i] = MBSdata->user_IO->GRM_r[i+1];

            break;
        }

        default:
        {
            std::cerr << "Wrong part name " << partName << ". Check your config file" << std::endl;
            break;
        }
    }
}

void RobotranYarpForceTorqueDriver::updateFromYarp(MBSdataStruct *MBSdata)
{

}



#endif
