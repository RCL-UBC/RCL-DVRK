/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitPSM_h
#define _mtsIntuitiveResearchKitPSM_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robLSPB.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSMTypes.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitOptimizer.h>
#include <cisstRobot/robQuintic.h>
//#include <cisstCommon/cmnUnits.h>

// temporary
#include <cisstOSAbstraction/osaStopwatch.h>

class mtsIntuitiveResearchKitPSM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    static const size_t NumberOfJoints = 7;

    mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds, const bool usingSimulinkControl = false);
    mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitPSM() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:

    // PSM Optimizer
    mtsIntuitiveResearchKitOptimizer * Optimizer;

    void Init(void);

    /*! Get data from the PID level based on current state. */
    void GetRobotData(void);

    /*! Verify that the state transition is possible, initialize global
      variables for the desired state and finally set the state. */
    void SetState(const mtsIntuitiveResearchKitPSMTypes::RobotStateType & newState);

    /*! Homing procedure, will check the homing state and call the required method. */
    void RunHoming(void);

    /*! Homing procedure, power the robot and initial current and encoder calibration. */
    void RunHomingPower(void);

    /*! Homing procedure, home all joints except last one using potentiometers as reference. */
    void RunHomingCalibrateArm(void);

    /*! Engaging adapter procedure. */
    void RunEngagingAdapter(void);

    /*! Engaging tool procedure. */
    void RunEngagingTool(void);

    /*! Cartesian state. */
    void RunPositionCartesian(void);

    /*! Following Trajectory and Need FeedForward Commands. */
    void RunFollowingTrajectory(void);

    /*! Cartesian constraint controller. */
    void RunConstraintControllerCartesian(void);

    /*! Wrapper to convert vector of 7 values to prmPositionJointSet and send to PID */
    void SetPositionJointLocal(const vctDoubleVec & newPosition);

    void EventHandlerAdapter(const prmEventButton & button);
    void EventHandlerTool(const prmEventButton & button);
    void EventHandlerManipClutch(const prmEventButton & button);
    void EventHandlerSUJClutch(const prmEventButton & button);

    void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    void SetTorqueJointArm(const prmForceTorqueJointSet & newTorque);
    void GetTorqueJointArm(vctDoubleVec & currentTorque) const;
    void GetRobotCartVelFromJacobian(vctDoubleVec & slaveVelAndPos) const;
    void GetJointTorqueFromForceTorque(vctDoubleVec & torqueValues) const;
    void SetOpenAngle(const double & openAngle);
    void SetTrajectoryOnOrOff(const mtsBool & trajectoryRunning);
    void SetRobotControlState(const std::string & state);
    void GetRobotControlState(std::string & state) const;

    //! Enable logs
    void EnableLogs(const mtsBool & enable);

    //! Function to close logs or end entry
    void CloseLogs(bool closingTime);

    //! Function to check if a file already exists, to not overwrite a file
    void CheckFileExists(std::string * fname);

    struct {
        mtsFunctionWrite Enable;
        mtsFunctionRead  GetPositionJoint;
        mtsFunctionRead  GetPositionJointDesired;
        mtsFunctionRead  GetTorqueJoint;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetTorqueJoint;
        mtsFunctionWrite SetCheckJointLimit;
    } PID;

    struct {
        mtsFunctionWrite SignalTrajectoryRunning;
        mtsFunctionWrite SetDesiredJointPosition;
        mtsFunctionWrite SetDesiredCartesianPosition;
        mtsFunctionRead  GetControllerTypeIsJoint;

        mtsBool          UsingJointControl;
    } SimulinkController;

    // Required interface
    struct InterfaceRobotTorque {
        //! Enable Robot Power
        mtsFunctionVoid  EnablePower;
        mtsFunctionVoid  DisablePower;
        mtsFunctionRead  GetActuatorAmpStatus;
        mtsFunctionVoid  BiasEncoder;
        mtsFunctionWrite SetActuatorCurrent;
        mtsFunctionWrite UsePotsForSafetyCheck;
        mtsFunctionWrite SetPotsToEncodersTolerance;
    } RobotIO;

    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
    } Adapter;

    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
    } Tool;

    struct {
        mtsFunctionRead GetButton;
        bool IsPressed;
    } ManipClutch;

    struct {
        mtsFunctionRead GetButton;
        bool IsPressed;
    } SUJClutch;

    // Functions for events
    struct {
        mtsFunctionWrite RobotStatusMsg;
        mtsFunctionWrite RobotErrorMsg;
        mtsFunctionWrite ManipClutch;
        mtsIntuitiveResearchKitPSMTypes::RobotStateType ManipClutchPreviousState;
        mtsFunctionWrite SUJClutch;
    } EventTriggers;

    // Cache cartesian goal position
    prmPositionCartesianSet CartesianSetParam;
    bool IsCartesianGoalSet;

    prmPositionCartesianGet CartesianGetParam;
    vctFrm4x4               CartesianGet;
    prmPositionCartesianGet CartesianGetDesiredParam;
    vctFrm4x4               CartesianGetDesired;
    prmPositionJointGet     JointGetParamPrev;
    prmPositionJointGet     JointGetParam;
    vctDoubleVec            JointGet; //current
    vctDoubleVec            JointGetDesired;
    prmPositionJointSet     JointSetParam;
    vctDoubleVec            JointSet; //desired
    vctDoubleVec            FeedbackVelocity; //AR_MINE
    robManipulator          Manipulator;
    robManipulator *        ToolOffset;
    vctFrm4x4               ToolOffsetTransformation;

    //! desired torque for feedforward
    prmForceTorqueJointSet TorqueDesired;

    vctFrm4x4 CartesianPositionFrm;
    double DesiredOpenAngle;
    mtsIntuitiveResearchKitPSMTypes::RobotStateType RobotState;

    struct {
        robLSPB LSPB;
        vctDoubleVec Velocity;
        vctDoubleVec Acceleration;
        vctDoubleVec Start;
        vctDoubleVec Goal;
        vctDoubleVec GoalError;
        vctDoubleVec GoalTolerance;
        double EndTime;
    } JointTrajectory;

    // Home Action
    double HomingTimer;
    bool HomingPowerRequested, HomingPowerCurrentBiasRequested;
    bool HomingCalibrateArmStarted;
    bool EngagingAdapterStarted;
    bool EngagingToolStarted;

    // temporary
    osaStopwatch EngagingStopwatch;
    vctDoubleVec EngagingJointSet;

    int Counter;
    std::string taskName;

    std::ofstream PSMLogFile;
    bool logsEnabled; //manually set in code, not through UI!!
    int  logEntryIndex;

    bool usingSimulink;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitPSM);

#endif // _mtsIntuitiveResearchKitPSM_h
