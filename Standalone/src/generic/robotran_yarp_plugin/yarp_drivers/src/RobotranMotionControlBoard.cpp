/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifdef YARP

#include <RobotranMotionControlBoard.h>


using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool RobotranYarpMotionControl::extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        //yError () << key1.c_str() << " not found\n"; //yError() will be included in the next version of yarp.
        std::cout << key1.c_str() << " not found\n";
        return false;
    }

    if(tmp.size()!=size)
    {
       // yError () << key1.c_str() << " incorrect number of entries";
        std::cout << key1.c_str() << " incorrect number of entries";
        return false;
    }

    out=tmp;
    return true;
}

static inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    std::cout << txt << " is not yet implemented for robotran yarp plugin";
    return false;
}

/**
 * robotran stuff
 */
bool RobotranYarpMotionControl::robotran_init()
{
    std::cout << "robotran_init : is this function really necessary?" << std::endl;
    return true;
}

void RobotranYarpMotionControl::updateToYarp(const MBSdataStruct * MBSdata)
{
    //update the vector pos
    for(unsigned int i=0; i<pos.size();i++)
    {
        pos[i] = zero[i] + (encoder[i] * convertRadiansToDegrees(MBSdata->q[jointID_map[i]]) );
        torque[i] = MBSdata->Qq[jointID_map[i]];
        current[i] = MBSdata->user_IO->currents[jointID_map[i]];
    }

    //update time
    simu_time = MBSdata->tsim;

}

void RobotranYarpMotionControl::updateFromYarp(MBSdataStruct *MBSdata)
{

    for(unsigned int i=0; i<numberOfJoints; i++)
    {
        if((!initialized) && _initialPidConfigFound)
        {
            //initialized the pid simulation structure with the yarp read conf.
            // converting to SI units/sign used in the user_Derivative.c for low level PD control
            MBSdata->user_IO->cvs->PIDs->p[motorID_map[i]]= -(_posPids[i].kp/1000);
            MBSdata->user_IO->cvs->PIDs->d[motorID_map[i]]= -(_posPids[i].kd/1000);
            MBSdata->user_IO->cvs->PIDs->i[motorID_map[i]]= -(_posPids[i].ki/1000);
            std::cout<< "PID intiliazed from YARP"<< std::endl;
        }
        //std::cout << "index " << i << " ref " << desiredPosition[i] << std::endl;
        MBSdata->user_IO->refs[motorID_map[i]]  = (encoder[i] * convertDegreesToRadians(desiredPosition[i]- zero[i]));
        MBSdata->user_IO->servo_type[motorID_map[i]] = controlMode[i];
    }

    if (_initialPidConfigFound)
    {
        initialized = true;
    }

}

/////////////////////////////////////
// DEVICE DRIVER
/////////////////////////////////////

RobotranYarpMotionControl::RobotranYarpMotionControl()
{

}

RobotranYarpMotionControl::~RobotranYarpMotionControl()
{

}

bool RobotranYarpMotionControl::open(yarp::os::Searchable& config)
{

    std::cout << "\n*********\nrobotran motionControl parameters are " << config.toString() << "\n***********\n" << std::endl;

    // Get joints names
    if(!config.check("jointNames"))
    {
        std::cout << "joints names not specified in config file " << std::endl;
        return false;
    }

    yarp::os::Bottle & jointNames = config.findGroup("jointNames");
    numberOfJoints = jointNames.size()-1;

    std::cout << "nbr joints = " << numberOfJoints << std::endl;

    pos.resize(numberOfJoints);
    pos.zero();

    desiredPosition.resize(numberOfJoints);
    desiredPosition.zero();

    torque.resize(numberOfJoints);
    torque.zero();

    current.resize(numberOfJoints);
    current.zero();

    _posPids.resize(numberOfJoints);

    // Get joints id
    if(!config.check("robotran_joint_id"))
    {
        std::cout << "robotran joints id not specified in config file " << std::endl;
        return false;
    }
    yarp::os::Bottle & jointID = config.findGroup("robotran_joint_id");
    jointID_map.resize(numberOfJoints);
    for(int i=0; i< jointID.size()-1; i++)
    {
        jointID_map[i] = jointID.get(i+1).asInt();
        printf("jointID_map[%d] = %d \n", i, jointID_map[i]);
    }

    // Get motor id
    if(!config.check("robotran_motor_id"))
    {
        std::cout << "robotran_motor_id parameter is not specified in config file " << std::endl;
        return false;
    }
    yarp::os::Bottle & motorID = config.findGroup("robotran_motor_id");
    motorID_map.resize(numberOfJoints);
    for(int i=0; i< motorID.size()-1; i++)
    {
        motorID_map[i] = motorID.get(i+1).asInt();
        printf("motorID_map[%d] = %d \n", i, motorID_map[i]);
    }

    // encoder sign and value
    if(!config.check("encoder"))
    {
        std::cout << "robotran encoder joints values not specified in config file " << std::endl;
        return false;
    }
    yarp::os::Bottle & encoderBottle = config.findGroup("encoder");
    encoder.resize(numberOfJoints);
    for(int i=0; i< encoderBottle.size()-1; i++)
    {
        encoder[i] = encoderBottle.get(i+1).asInt();
        printf("encoder[%d] = %d \n", i, encoder[i]);
    }

    // zeros value
    if(!config.check("zero"))
    {
        std::cout << "robotran zero joints values not specified in config file " << std::endl;
        return false;
    }
    yarp::os::Bottle & zeroBottle = config.findGroup("zero");
    zero.resize(numberOfJoints);
    for(int i=0; i< zeroBottle.size()-1; i++)
    {
        zero[i] = zeroBottle.get(i+1).asDouble();
        printf("zero[%d] = %d \n", i, zero[i]);
    }

    // Get max/min joints limits
    if(!config.check("max") || !config.check("min"))
    {
        std::cout << "robotran max /min joints positions not specified in config file " << std::endl;
        return false;
    }
    yarp::os::Bottle & max = config.findGroup("max");
    yarp::os::Bottle & min = config.findGroup("min");
    max_pos.resize(numberOfJoints);
    min_pos.resize(numberOfJoints);
    for(int i=0; i< max.size()-1; i++)  // we suppose that max and min have same size (it should be check!)
    {
        max_pos[i] = max.get(i+1).asDouble();
        printf("max pos[%d] = %f \n", i, max_pos[i]);

        min_pos[i] = min.get(i+1).asDouble();
        printf("min pos[%d] = %f \n", i, min_pos[i]);
    }

    controlMode.resize(numberOfJoints);
    for(int i=0; i< numberOfJoints; i++)
    {
        controlMode[i] = VOCAB_CM_POSITION;  //by default init to pos control
    }

      ////// POSITION PIDS  - optional
      yarp::os::Bottle posPidsGroup;
      posPidsGroup=config.findGroup("POS_PIDS", "Position Pid parameters");
      if (posPidsGroup.isNull()==false)
      {
          if (!parsePidsGroup(posPidsGroup, _posPids.data() ))
         {
            // yError() << "POS_PIDS section: error detected in parameters syntax\n";
             std::cout << "POS_PIDS section: error detected in parameters syntax\n";
             return false;
         }
         else
         {
           //  yDebug() << "Position Pids successfully loaded\n";
             std::cout << "Position Pids successfully loaded\n";
             std::cout  << "Using the following values for Position PID (all joints)" << std::endl;
             for (int i=0; i< numberOfJoints; i++)
             {
                 std::cout  << "PID Joint " << i << std::endl;
                 std::cout  << "\tkp is " << _posPids[i].kp << std::endl;
                 std::cout  << "\tki is " << _posPids[i].ki << std::endl;
                 std::cout  << "\tkd is " << _posPids[i].kd << "\n" << std::endl;
             }
             _initialPidConfigFound = true;
         }
      }
      else
      {
        //  yWarning() << "POS_PIDS group not found, using defaults from boards!!\n";
            std::cout << "POS_PIDS group not found, using defaults from boards!!\n";
      }

//    yarp::os::Property wrapProp;
////  yarp::os::Property &mmm =wrapProp.addGroup();

//    wrapProp.put("device","controlboardwrapper2");
//    yarp::os::Property  &net = wrapProp.addGroup(networks);
//    net.
//    wrapProp.put("networks","myself");
//    wrapProp.put("joints", numberOfJoints);
//    char str[100];
//    sprintf(str, "0 %d 0 %d", numberOfJoints, numberOfJoints);
//    wrapProp.put("myself", str);

//    std::cout << "$$$$$$$$$$$$$$$$$ " << wrapProp.toString() << std::endl;

    if(!config.check("useWrapper"))
    {
        std::cout << "\nNOT USING WRAPPERS!\n" << std::endl;
        return true;
    }

    std::cout << "\nUSING WRAPPERS!\n" << std::endl;

    yarp::dev::IMultipleWrapper* iWrap;
    wrap = new yarp::dev::PolyDriver();
    yarp::os::Property wrapProp;
    wrapProp.fromString(config.toString());
    yarp::os::ConstString partName, robotName, wholeName;
    partName = config.find("name").asString();
    robotName = config.find("robot").asString();
    wrapProp.unput("device");
    wrapProp.put("device", "controlboardwrapper2");
    wrapProp.unput("name");

    wholeName = robotName + "/" + partName;
    wrapProp.put("name", wholeName);


    std::cout << "robotName is " << robotName << "; partName is " << partName << "; wholeName is " << wholeName << std::endl;

    std::cout << "\n*********\n before wrapper " << wrapProp.toString() << "\n***********\n" << std::endl;

    wrap->open(wrapProp);
    if (!wrap->isValid())
        fprintf(stderr, "RobotranYarpMotionControl: wrapper did not open\n");
    else
        fprintf(stderr, "RobotranYarpMotionControl: wrapper opened correctly\n");

    if (!wrap->view(iWrap)) {
        printf("RobotranYarpMotionControl Wrapper interface not found\n");
    }

   yarp::dev::PolyDriverList polyList;
   yarp::os::Bottle *netList = config.find("networks").asList();
   if (netList->isNull()) {
       printf("RobotranYarpMotionControl ERROR, net list to attach to was not found, exiting\n");
       wrap->close();
       // m_controlBoard.close();
       return false;
   }
   std::cout << "NNNNNNNNNNNNNNNNNNNNetwork list found !!" << netList->toString() << std::endl;

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

bool RobotranYarpMotionControl::close()
{
    std::cout << "RobotranYarpMotionControl::close" << std::endl;
    if(wrap)
        wrap->close();

    return true;
}


/////////////////////////////////////
// POSITION CONTROL
/////////////////////////////////////

bool RobotranYarpMotionControl::positionMove(int j, double ref) //TESTING
{

   // std::cout << "robotran motionControl: positionMove " << std::endl;
    if (j >= 0 && j < (int) numberOfJoints) {
        desiredPosition[j] = ref; //we will use this ref_pos in the next simulation onUpdate call to ask Robotran to set PIDs ref_pos to this value
        return true;
    }
    return false;
}

bool RobotranYarpMotionControl::stop(int j) //TO BE DONE
{
    std::cout << "robotran motionControl: stop " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::stop() //TO BE DONE
{
    std::cout << "robotran motionControl: stop " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::positionMove(const double *refs) //TO BE DONE
{
    std::cout << "robotran motionControl: positionMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getAxes(int *ax) //WORKS
{
    std::cout << "robotran motionControl: getAxes " << std::endl;
    *ax = numberOfJoints;
    return true;
}

bool RobotranYarpMotionControl::setRefSpeed(int j, double sp) //TO BE DONE
{
    std::cout << "robotran motionControl: setRefSpeed " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getRefSpeed(int j, double *ref) //TO BE DONE
{
    std::cout << "robotran motionControl: getRefSpeed " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getRefSpeeds(double *spds) //TO BE DONE
{
    std::cout << "robotran motionControl: getRefSpeeds " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::relativeMove(int j, double delta) //NOT TESTED
{
    std::cout << "robotran motionControl: relativeMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::relativeMove(const double *deltas) //NOT TESTED
{
    std::cout << "robotran motionControl: relativeMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::checkMotionDone(int j, bool *flag) //NOT TESTED
{
    std::cout << "robotran motionControl: checkMotionDone " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::checkMotionDone(bool *flag) //NOT TESTED
{
    std::cout << "robotran motionControl: checkMotionDone " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setPositionMode() //NOT TESTED
{
    std::cout << "robotran motionControl: setPositionMode " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setRefSpeeds(const double *spds) //NOT TESTED
{
    std::cout << "robotran motionControl: setRefSpeeds " << std::endl;
    return false;
}


bool RobotranYarpMotionControl::setRefAcceleration(int j, double acc) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: setRefAcceleration " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setRefAccelerations(const double *accs) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: setRefAccelerations " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getRefAcceleration(int j, double *acc) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: getRefAcceleration " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getRefAccelerations(double *accs) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: getRefAccelerations " << std::endl;
    return false;
}

// IPositionControl2

bool RobotranYarpMotionControl::positionMove(const int n_joint, const int *joints, const double *refs) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: positionMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::relativeMove(const int n_joint, const int *joints, const double *deltas) //NOT IMPLEMENTED
{
    return false;
}


bool RobotranYarpMotionControl::checkMotionDone(const int n_joint, const int *joints, bool *flags) //NOT IMPLEMENTED
{
    return false;
}

bool RobotranYarpMotionControl::setRefSpeeds(const int n_joint, const int *joints, const double *spds) //NOT IMPLEMENTED
{
    return false;
}


bool RobotranYarpMotionControl::setRefAccelerations(const int n_joint, const int *joints, const double *accs) //NOT IMPLEMENTED
{
    return false;
}


bool RobotranYarpMotionControl::getRefSpeeds(const int n_joint, const int *joints, double *spds) //NOT IMPLEMENTED
{
    return false;
}


bool RobotranYarpMotionControl::getRefAccelerations(const int n_joint, const int *joints, double *accs) //NOT IMPLEMENTED
{
    return false;
}


bool RobotranYarpMotionControl::stop(const int n_joint, const int *joints) //NOT IMPLEMENTED
{
    return false;
}

/////////////////////////////////////
// POSITION DIRECT
/////////////////////////////////////

bool RobotranYarpMotionControl::setPositionDirectMode() //NOT IMPLEMENTED -> Is it the same as setPositionMode?
{
    return false;
}

bool RobotranYarpMotionControl::setPosition(int j, double ref)
{
    return positionMove(j, ref);
}

bool RobotranYarpMotionControl::setPositions(const int n_joint, const int *joints, double *refs)
{
    bool ret = true;
    for(int i=0; i<n_joint; i++)
    {
        ret = ret && positionMove(joints[i], refs[i]);
    }
    return ret;
}

bool RobotranYarpMotionControl::setPositions(const double *refs)
{
    bool ret = true;
    for(int i=0; i<numberOfJoints; i++)
    {
        ret = ret && positionMove(i, refs[i]);
    }
    return ret;
}

/////////////////////////////////////
// VELOCITY CONTROL
/////////////////////////////////////


bool RobotranYarpMotionControl::setVelocityMode() //NOT TESTED
{
    std::cout << "robotran motionControl: setVelocityMode " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::velocityMove(int j, double sp) //NOT TESTED
{
    std::cout << "robotran motionControl: velocityMove " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::velocityMove(const double *sp) //NOT TESTED
{
    std::cout << "robotran motionControl: velocityMove " << std::endl;
    return false;
}

/////////////////////////////////////
// CONTROL MODE
/////////////////////////////////////

bool RobotranYarpMotionControl::setPositionMode(int j)  //TO BE TESTED
{
    controlMode[j] = VOCAB_CM_POSITION;// POSITION_CTRL;
}

/////////////////////////////////////
// ENCODER
/////////////////////////////////////

bool RobotranYarpMotionControl::getEncoder(int j, double *v) //TO BE TESTED
{
    if (v && j >= 0 && j < (int)numberOfJoints) {
        *v = pos[j];
        return true;
    }
    return false;
}

bool RobotranYarpMotionControl::getEncoders(double *encs) //TO BE TESTED
{
    if (!encs) return false;
    for (unsigned int i = 0; i < numberOfJoints; ++i)
    {
        encs[i] = pos[i];  //should we just use memcopy here?
    }
    return true;
}

bool RobotranYarpMotionControl::getEncodersTimed(double *encs, double *time) //TO BE TESTED
{
    if (!encs) return false;
    for (unsigned int i = 0; i <numberOfJoints; ++i) {
        encs[i] = pos[i];  //should we just use memcopy here?
        time[i] = simu_time;
    }    
    return true;
}


bool RobotranYarpMotionControl::getEncoderTimed(int j, double *enc, double *time) //TO BE TESTED
{
    if (time && enc && j >= 0 && j < (int)numberOfJoints)
    {
        *enc = pos[j];
        *time = simu_time;
        return true;
    }
    return false;
}


bool RobotranYarpMotionControl::resetEncoder(int j) //TO BE DONE
{
    std::cout << "robotran motionControl: resetEncoder " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::resetEncoders() //TO BE DONE
{
    std::cout << "robotran motionControl: resetEncoders " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setEncoder(int j, double val) //TO BE DONE
{
    std::cout << "robotran motionControl: setEncoder " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::setEncoders(const double *vals) //TO BE DONE
{
    std::cout << "robotran motionControl: setEncoders " << std::endl;
    return false;
}


bool RobotranYarpMotionControl::getEncoderSpeed(int j, double *sp) //NOT TESTED
{
    std::cout << "robotran motionControl: getEncoderSpeed " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getEncoderSpeeds(double *spds) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: getEncoderSpeeds " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getEncoderAcceleration(int j, double *spds) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: getEncoderAcceleration " << std::endl;
    return false;
}

bool RobotranYarpMotionControl::getEncoderAccelerations(double *accs) //NOT IMPLEMENTED
{
    std::cout << "robotran motionControl: getEncoderAccelerations " << std::endl;
    return false;
}

// ICONTROLLIMITS2

bool  RobotranYarpMotionControl::getLimits (int axis, double *min, double *max)  //TO BE TESTED
{
    if (!min || !max) return false;
    *min = min_pos[axis];
    *max = max_pos[axis];
    return true;
}

bool RobotranYarpMotionControl::setTorqueMode()
{
    return NOT_YET_IMPLEMENTED("setTorqueMode");
}

bool RobotranYarpMotionControl::getRefTorques(double *t)
{
    return NOT_YET_IMPLEMENTED("getRefTorques");
}

bool RobotranYarpMotionControl::getRefTorque(int j, double *t)
{
    return NOT_YET_IMPLEMENTED("getRefTorque");
}

bool RobotranYarpMotionControl::setRefTorques(const double *t)
{
    return NOT_YET_IMPLEMENTED("setRefTorques");
}

bool RobotranYarpMotionControl::setRefTorque(int j, double t)
{
    return NOT_YET_IMPLEMENTED("setRefTorque");
}

bool RobotranYarpMotionControl::getBemfParam(int j, double *bemf)
{
    return NOT_YET_IMPLEMENTED("getBemfParam");
}

bool RobotranYarpMotionControl::setBemfParam(int j, double bemf)
{
    return NOT_YET_IMPLEMENTED("setBemfParam");
}

bool RobotranYarpMotionControl::setTorquePid(int j, const Pid &pid)
{
    return NOT_YET_IMPLEMENTED("setTorquePid");
}

bool RobotranYarpMotionControl::getTorque(int j, double *t)
{
    *t = torque[j];
    return true;
}

bool RobotranYarpMotionControl::getTorques(double *t)
{
    bool ret = true;
    for(int j=0; j<numberOfJoints; j++)
        ret = ret && getTorque(j, &t[j]);
    return true;
}

bool RobotranYarpMotionControl::getTorqueRange(int j, double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getTorqueRange");
}

bool RobotranYarpMotionControl::getTorqueRanges(double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getTorqueRanges");
}

bool RobotranYarpMotionControl::setTorquePids(const Pid *pids)
{
    return NOT_YET_IMPLEMENTED("setTorquePids");
}

bool RobotranYarpMotionControl::setTorqueErrorLimit(int j, double limit)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimit");
}

bool RobotranYarpMotionControl::setTorqueErrorLimits(const double *limits)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimits");
}

bool RobotranYarpMotionControl::getTorqueError(int j, double *err)
{
    return NOT_YET_IMPLEMENTED("getTorqueError");
}

bool RobotranYarpMotionControl::getTorqueErrors(double *errs)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrors");
}

bool RobotranYarpMotionControl::getTorquePidOutput(int j, double *out)
{
    return NOT_YET_IMPLEMENTED("getTorquePidOutput");
}

bool RobotranYarpMotionControl::getTorquePidOutputs(double *outs)
{
    return NOT_YET_IMPLEMENTED("getTorquePidOutputs");
}

bool RobotranYarpMotionControl::getTorquePid(int j, Pid *pid)
{
    return NOT_YET_IMPLEMENTED("getTorquePid");
}

bool RobotranYarpMotionControl::getTorquePids(Pid *pids)
{
    return NOT_YET_IMPLEMENTED("getTorquePids");
}

bool RobotranYarpMotionControl::getTorqueErrorLimit(int j, double *limit)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimit");
}

bool RobotranYarpMotionControl::getTorqueErrorLimits(double *limits)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimits");
}

bool RobotranYarpMotionControl::resetTorquePid(int j)
{
    return NOT_YET_IMPLEMENTED("resetTorquePid");
}

bool RobotranYarpMotionControl::disableTorquePid(int j)
{
    return NOT_YET_IMPLEMENTED("disableTorquePid");
}

bool RobotranYarpMotionControl::enableTorquePid(int j)
{
    return NOT_YET_IMPLEMENTED("enableTorquePid");
}

bool RobotranYarpMotionControl::setTorqueOffset(int j, double v)
{
    return NOT_YET_IMPLEMENTED("setTorqueOffset");
}



bool RobotranYarpMotionControl::enableAmp(int j)
{
    return NOT_YET_IMPLEMENTED("enableAmp");
}

bool RobotranYarpMotionControl::disableAmp(int j)
{
    return NOT_YET_IMPLEMENTED("disableAmp");
}

bool RobotranYarpMotionControl::getCurrents(double *vals)
{
    bool ret = true;
    for(int j=0; j<numberOfJoints; j++)
        ret = ret && getCurrent(j, &vals[j]);
    return true;
}

bool RobotranYarpMotionControl::getCurrent(int j, double *val)
{
        *val = current[j];
        return true;
}

bool RobotranYarpMotionControl::setMaxCurrent(int j, double v)
{
    return NOT_YET_IMPLEMENTED("setMaxCurrent");
}

bool RobotranYarpMotionControl::getAmpStatus(int *st)
{
    return NOT_YET_IMPLEMENTED("getAmpStatus");
}

bool RobotranYarpMotionControl::getAmpStatus(int j, int *st)
{
    return NOT_YET_IMPLEMENTED("getAmpStatus");
}



bool RobotranYarpMotionControl::setControlMode(const int j, const int mode)
{
    controlMode[j] = mode;
}
bool RobotranYarpMotionControl::setControlModes(const int n_joint, const int *joints, int *modes)
{
    for(int j=0; j<n_joint; j++)
        setControlMode(joints[j], modes[j]);
    return true;
}

bool RobotranYarpMotionControl::setControlModes(int *modes)
{
    for(int j=0; j<numberOfJoints; j++)
        setControlMode(j, modes[j]);
    // some control modes such as torque control, velocity control are not implemented yet.
    return true;
}

bool RobotranYarpMotionControl::getControlMode(int j, int *mode)
{
    *mode =controlMode[j];
    return true;
}

bool RobotranYarpMotionControl::getControlModes(int *modes)
{
    for(int j=0; j<numberOfJoints; j++)
        getControlMode(j, &modes[j]);
    return true;
}

bool RobotranYarpMotionControl::getControlModes(const int n_joint, const int *joints, int *modes)
{
    for(int j=0; j<n_joint; j++)
        getControlMode(joints[j], &modes[j]);
    // some control modes such as torque control, velocity control are not implemented yet.
    return true;
}

 bool RobotranYarpMotionControl::setPid (int j, const Pid &pid)
 {
     return NOT_YET_IMPLEMENTED("setPid");
 }
 bool RobotranYarpMotionControl::setPids (const Pid *pids)
 {
     return NOT_YET_IMPLEMENTED("setPids");
 }
 bool RobotranYarpMotionControl::setReference (int j, double ref)
 {
     return NOT_YET_IMPLEMENTED("setReference");
 }
 bool RobotranYarpMotionControl::setReferences (const double *refs){
     return NOT_YET_IMPLEMENTED("setReferences");
 }
 bool RobotranYarpMotionControl::setErrorLimit (int j, double limit)
 {
     return NOT_YET_IMPLEMENTED("setErrorLimit");
 }
 bool RobotranYarpMotionControl::setErrorLimits (const double *limits)
 {
     return NOT_YET_IMPLEMENTED("setErrorLimits");
 }
 bool RobotranYarpMotionControl::getError (int j, double *err)
 {
     return NOT_YET_IMPLEMENTED("getError");
 }
 bool RobotranYarpMotionControl::getErrors (double *errs)
 {
     return NOT_YET_IMPLEMENTED("getErrors");
 }
 bool RobotranYarpMotionControl::getPid (int j, Pid *pid)
 {
     return NOT_YET_IMPLEMENTED("getPid");
 }
 bool RobotranYarpMotionControl::getPids (Pid *pids)
 {
     return NOT_YET_IMPLEMENTED("getPids");
 }
 bool RobotranYarpMotionControl::getReference (int j, double *ref)
 {
     return NOT_YET_IMPLEMENTED("getReference");
 }
 bool RobotranYarpMotionControl::getReferences (double *refs)
 {
     return NOT_YET_IMPLEMENTED("getReferences");
 }
 bool RobotranYarpMotionControl::getErrorLimit (int j, double *limit)
 {
     return NOT_YET_IMPLEMENTED("getErrorLimit");
 }
 bool RobotranYarpMotionControl::getErrorLimits (double *limits)
 {
     return NOT_YET_IMPLEMENTED("getErrorLimits");
 }
 bool RobotranYarpMotionControl::resetPid (int j)
 {
     return NOT_YET_IMPLEMENTED("resetPid");
 }
 bool RobotranYarpMotionControl::disablePid (int j)
 {
     return NOT_YET_IMPLEMENTED("disablePid");
 }
 bool RobotranYarpMotionControl::enablePid (int j)
 {
     return NOT_YET_IMPLEMENTED("enablePid");
 }
 bool RobotranYarpMotionControl::setOffset (int j, double v)
 {
     return NOT_YET_IMPLEMENTED("setOffset");
 }

 bool RobotranYarpMotionControl::getOutput(int j, double *out)
 {
     return NOT_YET_IMPLEMENTED("getOutput");
 }

 bool RobotranYarpMotionControl::getOutputs(double *outs)
 {
     return NOT_YET_IMPLEMENTED("getOutputs");
 }

bool RobotranYarpMotionControl::parsePidsGroup(yarp::os::Bottle& pidsGroup , yarp::dev::Pid myPid[])
{
    int j=0;
    yarp::os::Bottle xtmp;

    if (!extractGroup(pidsGroup, xtmp, "kp", "kp parameter", numberOfJoints+1))  return false;
    else
    {
        for (j=0; j<numberOfJoints; j++)
            myPid[j].kp = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "kd", "kd parameter", numberOfJoints+1))  return false;
    else
    {
        for (j=0; j<numberOfJoints; j++)
            myPid[j].kd = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "ki", "ki parameter", numberOfJoints+1))  return false;
    else
    {
        for (j=0; j<numberOfJoints; j++)
            myPid[j].ki = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "maxInt", "maxInt parameter", numberOfJoints+1))  return false;
    else
    {
        for (j=0; j<numberOfJoints; j++)
            myPid[j].max_int = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "maxPwm", "maxPwm parameter", numberOfJoints+1))   return false;
    else
    {
        for (j=0; j<numberOfJoints; j++)
            myPid[j].max_output = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "shift", "scale parameter", numberOfJoints+1))     return false;
    else
    {
        for (j=0; j<numberOfJoints; j++)
            myPid[j].scale = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "ko", "pid offset parameter", numberOfJoints+1))   return false;
    else
    {
        for (j=0; j<numberOfJoints; j++)
            myPid[j].offset = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "stictionUp", "stictionUp parameter", numberOfJoints+1))    return false;
    else
    {
        for (j=0; j<numberOfJoints; j++)
            myPid[j].stiction_up_val = xtmp.get(j+1).asDouble();
    }

    if (!extractGroup(pidsGroup, xtmp, "stictionDwn", "stictionDwn parameter", numberOfJoints+1))  return false;
    else
    {
        for (j=0; j<numberOfJoints; j++)
            myPid[j].stiction_down_val = xtmp.get(j+1).asDouble();
    }

    return true;
}


#endif
