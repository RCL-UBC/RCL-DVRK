/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Angelica Ruszkowski
  Created on: 2015-03-10

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


/*!
  \file
  \brief Basic interface to Simulink Controller
  \ingroup sawControllers

  \todo Add dError filtering
*/


#ifndef _mtsSimulinkController_h
#define _mtsSimulinkController_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmJointType.h>
#include <cisstParameterTypes/prmTwoWaySocket.h>
#include <cisstParameterTypes/prmSimulinkDataPacket.h>

#include <sawControllers/sawControllersRevision.h>

//! Always include last
#include <sawControllers/sawControllersExport.h>
#include <queue>

class CISST_EXPORT mtsSimulinkController : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:

    /**
     * @brief Main constructor
     *
     * @param taskname   The name of the MTS periodic task
     * @param period     The period of the task
     */
    mtsSimulinkController(const std::string & taskname, double period, const unsigned int NumJoints);
    mtsSimulinkController(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsSimulinkController(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:
    void SetupInterfaces(void);
    void Enable(const bool & enable);

    /******************Interfaces******************/
    // Required interface
    struct InterfaceRobotTorque {
        //! Read joint position from robot
        mtsFunctionRead GetFeedbackPosition;
        //! Read joint velocity from robot
        mtsFunctionRead GetFeedbackVelocity;
        //! Write the joint torques
        mtsFunctionWrite SetTorque;
    } Robot;

    // Required interface
    struct TopLevelWidgetSimulinkStruct {
        mtsFunctionWrite SignalSimulinkDone;
    } TopLevelWidgetSimulink;

    struct RobotArmStruct {
        mtsFunctionRead GetRobotCartVelFromJacobian;
        mtsFunctionRead GetJointTorqueFromForceTorques;
        mtsFunctionRead GetPositionCartesian;
    } RobotArm;

    /******************Control Members******************/
    //! Counter for internal use
    int  Counter;

    //! Enable mtsSimulink controller
    bool Enabled;

    //! Component name
    std::string taskName;

    //! Number of Joints
    unsigned int NumberOfJoints;

    //! Flag to track trajectory status
    bool trajRunning;

    //! Flag to track type of controller used
    bool usingJointController; //versus Cartesian

    //! Feedback joint positions
    vctDoubleVec FeedbackPosition;
    //! Desired joint positions
    vctDoubleVec DesiredPosition;
    //! Feedback joint velocities
    vctDoubleVec FeedbackVelocity;
    //! Feedback joint velocities
    vctDoubleVec FeedbackVelocityPrev;
    //! Cartesian current position
    vctDoubleVec CartesianCurrentPos;
    //! Cartesian desired position
    vctDoubleVec CartesianDesiredPos;
    //! Calculated cartesian velocity
    vctDoubleVec CartesianVelocity;
    //! Calculated cartesian force torque
    vctDoubleVec ForceTorque;
    //! Torque set to robot
    vctDoubleVec Torque;

    //! prm type feedback positoin
    prmPositionJointGet FeedbackPositionParam;
    prmPositionJointGet FeedbackPositionPreviousParam;
    //! prm type desired position
    prmPositionJointSet DesiredPositionParam;
    //! prm type desired position cartesian
    prmPositionCartesianSet CartesianSetParam;
    //! prm type current position cartesian
    prmPositionCartesianGet CartesianGetParam;
    //! prm type feedback velocity
    prmVelocityJointGet FeedbackVelocityParam;
    //! prm type set torque
    prmForceTorqueJointSet TorqueParam;

    /******************Interface Functions******************/
    //! Signal start/end of trajectory
    void SignalTrajectoryEvent(const mtsBool & trajInProgress);

    void SetControllerType(const mtsBool & isJointControl);
    void GetControllerType(mtsBool & isJointControl) const;

    void SetDesiredPositions(const prmPositionJointSet & prmPos);
    void SetDesiredCartPosition(const prmPositionCartesianSet &newPosition);

    void SetHostIP(const mtsStdString & hostIP);
    void SetKinematicsPort(const mtsUShort & kinematicsPort);
    void SetTorquePort(const mtsUShort & torquePort);

    /******************Simulink Communication Members and Functions******************/
    int tryingToInitializeSim;
    int initializingMaxAttempts;

    prmTwoWaySocket       * socketComm;
    prmSimulinkDataPacket * dataPacket;

    struct communicationInformation {
        mtsStdString hostIP;
        mtsUShort    kinematicsPort;
        mtsUShort    torquePort;

        bool isInitialized() { return !(hostIP.Data == "" || kinematicsPort.Data == 0 || torquePort.Data == 0); }
    } SocketConnectionInfo;

    bool simulinkIsReady;
    bool disablePIDNextReceivedPacket; //after initialization, need to disable PID
    struct timespec startLoop, endLoop, watchDogTime;
    double watchDogTimeOutMS; //in milliseconds

    void SendDataToSimulink();
    void ReceiveFromSimulink();
    vctDoubleVec ParseMatlabServerData(int * exitVal, char * bufferTorque, int bytesRead);
    void DoneCommunicationWithSimulink(bool done);

    /******************Logging******************/
    //! Log file
    std::ofstream SimulinkControllerLogFile;
    //! Flag to remember if logs are enabled
    bool logsEnabled;
    //! Keeps track of which trajectory
    unsigned int logEntryIndex;

    //! Enable logs
    void EnableLogs(const mtsBool & enable);
    //! Function to close logs or end entry
    void CloseLogs(bool closingTime);
    //! Function to check if a file already exists, to not overwrite a file
    void CheckFileExists(std::string * fname);
    //! Log a single entry
    void LogEntry(bool all);

    /******************Helper Functions******************/
    void UpdateStateData();
    vctDoubleVec GetCartesianPosRot(vctFrm4x4 position);
    double GetElapsedTime(struct timespec time0, struct timespec time1);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSimulinkController);

#endif  // _mtsSimulinkController_h
