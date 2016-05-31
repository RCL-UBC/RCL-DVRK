/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitPSM_h
#define _mtsIntuitiveResearchKitPSM_h

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitOptimizer.h>

// temporary
#include <cisstOSAbstraction/osaStopwatch.h>

class mtsIntuitiveResearchKitPSM: public mtsIntuitiveResearchKitArm
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:

    mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds, const bool usingSimulink);
    mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitPSM() {}

    void Configure(const std::string & filename);

protected:

    // PSM Optimizer
    mtsIntuitiveResearchKitOptimizer * Optimizer;

    /*! Configuration methods */
    inline size_t NumberOfAxes(void) const {
        return 7;
    }

    inline size_t NumberOfJoints(void) const {
        return 7;
    }

    inline size_t NumberOfJointsKinematics(void) const {
        return 6;
    }

    inline size_t NumberOfBrakes(void) const {
        return 0;
    }

    inline bool UsePIDTrackingError(void) const {
        return false;
    }

    robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                            const vctFrm4x4 & cartesianGoal);

    void Init(void);

    /*! Verify that the state transition is possible, initialize global
      variables for the desired state and finally set the state. */
    void SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState);

    void SetRobotControlState(const std::string & state);

    /*! Switch case for user mode. */
    void RunArmSpecific(void);

    /*! Homing procedure, home all joints except last one using potentiometers as reference. */
    void RunHomingCalibrateArm(void);

    /*! Engaging adapter procedure. */
    void RunEngagingAdapter(void);

    /*! Engaging tool procedure. */
    void RunEngagingTool(void);

    /*! Cartesian constraint controller. */
    virtual void RunConstraintControllerCartesian(void);

    void EventHandlerAdapter(const prmEventButton & button);
    void EventHandlerTool(const prmEventButton & button);
    void EventHandlerManipClutch(const prmEventButton & button);
    void EventHandlerSUJClutch(const prmEventButton & button);

    void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    void SetJawPosition(const double & openAngle);

    /*! Event handlers for tools */
    //@{
    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
    } Adapter;

    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
    } Tool;
    //@}

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
        mtsFunctionWrite ManipClutch;
        mtsFunctionWrite SUJClutch;
        mtsIntuitiveResearchKitArmTypes::RobotStateType ManipClutchPreviousState;
    } ClutchEvents;

    robManipulator *        ToolOffset;
    vctFrm4x4               ToolOffsetTransformation;

    bool EngagingAdapterStarted;
    bool EngagingToolStarted;

    // temporary
    osaStopwatch EngagingStopwatch;
    vctDoubleVec EngagingJointSet;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitPSM);

#endif // _mtsIntuitiveResearchKitPSM_h
