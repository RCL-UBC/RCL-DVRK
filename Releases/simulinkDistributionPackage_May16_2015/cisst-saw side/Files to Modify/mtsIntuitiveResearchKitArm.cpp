/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>
#include <time.h>

#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaSocket.h>
#include <cisstOSAbstraction/osaSocketServer.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitArm, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitArm::mtsIntuitiveResearchKitArm(const std::string & componentName, const double periodInSeconds, const bool usingSimulinkControl):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    usingSimulink = usingSimulinkControl;
}

mtsIntuitiveResearchKitArm::mtsIntuitiveResearchKitArm(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
}

void mtsIntuitiveResearchKitArm::Init(void)
{
    IsGoalSet = false;
    logsEnabled = false;
    logEntryIndex = 0;

    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);

    // initialize trajectory data
    JointGet.SetSize(NumberOfJoints());
    JointSet.SetSize(NumberOfJoints());
    JointSetParam.Goal().SetSize(NumberOfAxes());
    JointTrajectory.Velocity.SetSize(NumberOfJoints());
    JointTrajectory.Acceleration.SetSize(NumberOfJoints());
    JointTrajectory.Start.SetSize(NumberOfJoints());
    JointTrajectory.Goal.SetSize(NumberOfJoints());
    JointTrajectory.GoalError.SetSize(NumberOfJoints());
    JointTrajectory.GoalTolerance.SetSize(NumberOfJoints());
    JointTrajectory.EndTime = 0.0;
    PotsToEncodersTolerance.SetSize(NumberOfAxes());

    // initialize velocity
    CartesianVelocityGetParam.SetVelocityLinear(vct3(0.0));
    CartesianVelocityGetParam.SetVelocityAngular(vct3(0.0));
    CartesianVelocityGetParam.SetTimestamp(0.0);
    CartesianVelocityGetParam.SetValid(false);

    // cartesian position are timestamped using timestamps provided by PID
    CartesianGetParam.SetAutomaticTimestamp(false);
    CartesianGetDesiredParam.SetAutomaticTimestamp(false);
    this->StateTable.AddData(CartesianGetParam, "CartesianPosition");
    this->StateTable.AddData(CartesianGetDesiredParam, "CartesianPositionDesired");
    this->StateTable.AddData(JointGetParam, "JointPosition");
    this->StateTable.AddData(JointGetDesired, "JointPositionDesired");

    // setup CISST Interface
    PIDInterface = AddInterfaceRequired("PID");
    if (PIDInterface) {
        PIDInterface->AddFunction("Enable", PID.Enable);
        PIDInterface->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        PIDInterface->AddFunction("GetPositionJointDesired", PID.GetPositionJointDesired);
        PIDInterface->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        PIDInterface->AddFunction("SetCheckJointLimit", PID.SetCheckJointLimit);
        PIDInterface->AddFunction("EnableTorqueMode", PID.EnableTorqueMode);
        PIDInterface->AddFunction("SetTorqueJoint", PID.SetTorqueJoint);
        PIDInterface->AddFunction("GetTorqueJoint",  PID.GetTorqueJoint);
        PIDInterface->AddFunction("SetTorqueOffset", PID.SetTorqueOffset);
        PIDInterface->AddFunction("EnableTrackingError", PID.EnableTrackingError);
        PIDInterface->AddFunction("SetTrackingErrorTolerances", PID.SetTrackingErrorTolerance);
        PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::JointLimitEventHandler, this, "JointLimit");
        PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::ErrorEventHandler, this, "Error");
    }

    if(usingSimulink) {
        mtsInterfaceRequired * simulinkInterfaceRequired = AddInterfaceRequired("SimulinkControlCommand");
        if (simulinkInterfaceRequired) {
            simulinkInterfaceRequired->AddFunction("SignalTrajEvent",            SimulinkController.SignalTrajectoryRunning);
            simulinkInterfaceRequired->AddFunction("SetPositionJoint",           SimulinkController.SetDesiredJointPosition);
            simulinkInterfaceRequired->AddFunction("SetPositionCartDes",         SimulinkController.SetDesiredCartesianPosition);
            simulinkInterfaceRequired->AddFunction("GetSimControllerType",       SimulinkController.GetControllerTypeIsJoint);
         }
    }

    // Robot IO
    IOInterface = AddInterfaceRequired("RobotIO");
    if (IOInterface) {
        IOInterface->AddFunction("EnablePower", RobotIO.EnablePower);
        IOInterface->AddFunction("DisablePower", RobotIO.DisablePower);
        IOInterface->AddFunction("GetActuatorAmpStatus", RobotIO.GetActuatorAmpStatus);
        IOInterface->AddFunction("GetBrakeAmpStatus", RobotIO.GetBrakeAmpStatus);
        IOInterface->AddFunction("BiasEncoder", RobotIO.BiasEncoder);
        IOInterface->AddFunction("ResetSingleEncoder", RobotIO.ResetSingleEncoder);
        IOInterface->AddFunction("GetAnalogInputPosSI", RobotIO.GetAnalogInputPosSI);
        IOInterface->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
        IOInterface->AddFunction("UsePotsForSafetyCheck", RobotIO.UsePotsForSafetyCheck);
        IOInterface->AddFunction("SetPotsToEncodersTolerance", RobotIO.SetPotsToEncodersTolerance);
        IOInterface->AddFunction("BrakeRelease", RobotIO.BrakeRelease);
        IOInterface->AddFunction("BrakeEngage", RobotIO.BrakeEngage);
    }

    RobotInterface = AddInterfaceProvided("Robot");
    if (RobotInterface) {
        // Get
        RobotInterface->AddCommandReadState(this->StateTable, JointGetParam, "GetPositionJoint");
        RobotInterface->AddCommandReadState(this->StateTable, JointGetDesired, "GetPositionJointDesired");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetParam, "GetPositionCartesian");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetDesiredParam, "GetPositionCartesianDesired");
        // Set
        RobotInterface->AddCommandRead(&mtsIntuitiveResearchKitArm::GetTorqueJointArm, this, "GetEffortJoint");
        RobotInterface->AddCommandRead(&mtsIntuitiveResearchKitArm::GetRobotCartVelFromJacobian, this, "GetRobotCartVelFromJacobian");
        RobotInterface->AddCommandRead(&mtsIntuitiveResearchKitArm::GetJointTorqueFromForceTorque,this, "GetJointTorqueFromForceTorque");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionJoint, this, "SetPositionJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionGoalJoint, this, "SetPositionGoalJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionCartesian, this, "SetPositionCartesian");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetTorqueJointArm,  this, "SetTorqueJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionGoalCartesian, this, "SetPositionGoalCartesian");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetTrajectoryOnOrOff, this, "SetTrajectoryRunning", mtsBool());
        // Trajectory events
        RobotInterface->AddEventWrite(JointTrajectory.GoalReachedEvent, "GoalReached", bool());
        // Robot State
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetRobotControlState,
                                        this, "SetRobotControlState", std::string(""));
        RobotInterface->AddCommandRead(&mtsIntuitiveResearchKitArm::GetRobotControlState,
                                       this, "GetRobotControlState", std::string(""));
        // Human readable messages
        RobotInterface->AddEventWrite(MessageEvents.Status, "Status", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.RobotState, "RobotState", std::string(""));

        // Stats
        RobotInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                            "GetPeriodStatistics");
    }
}

void mtsIntuitiveResearchKitArm::Configure(const std::string & filename)
{
    robManipulator::Errno result;
    result = this->Manipulator.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        CMN_LOG_CLASS_INIT_ERROR << GetName() << ": Configure: failed to load manipulator configuration file \""
                                 << filename << "\"" << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::Startup(void)
{
    this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
}

void mtsIntuitiveResearchKitArm::Run(void)
{
    ProcessQueuedEvents();
    GetRobotData();

    switch (RobotState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED:
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING:
        RunHomingPower();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ARM:
        RunHomingCalibrateArm();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED:
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_READY:
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT:
        RunPositionJoint();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT:
        RunPositionGoalJoint();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN:
        RunPositionCartesian();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN:
        RunPositionGoalCartesian();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL:
        break;
    default:
        RunArmSpecific();
        break;
    }

    RunEvent();
    ProcessQueuedCommands();

    CartesianGetPreviousParam = CartesianGetParam;
}

void mtsIntuitiveResearchKitArm::Cleanup(void)
{
    if (NumberOfBrakes() > 0) {
        RobotIO.BrakeEngage();
    }
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitArm::GetRobotControlState(std::string & state) const
{
    state = mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(this->RobotState);
}

bool mtsIntuitiveResearchKitArm::CurrentStateIs(const mtsIntuitiveResearchKitArmTypes::RobotStateType & state)
{
    if (RobotState == state) {
        return true;
    }
    CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionGoalJoint: arm not in "
                              << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN)
                              << ", current state is " << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(RobotState) << std::endl;
    return false;
}

void mtsIntuitiveResearchKitArm::GetRobotData(void)
{
    // we can start reporting some joint values after the robot is powered
    if (this->RobotState > mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING) {
        mtsExecutionResult executionResult;
        executionResult = PID.GetPositionJoint(JointGetParam);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointPosition failed \""
                                    << executionResult << "\"" << std::endl;
        }
        // assign to a more convenient vctDoubleVec
        JointGet.Assign(JointGetParam.Position(), NumberOfJoints());

        // desired joints
        executionResult = PID.GetPositionJointDesired(JointGetDesired);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointPositionDesired failed \""
                                    << executionResult << "\"" << std::endl;
        }

        // when the robot is ready, we can compute cartesian position
        if (this->RobotState >= mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            // update cartesian position
            CartesianGet = Manipulator.ForwardKinematics(JointGet);
            CartesianGet.Rotation().NormalizedSelf();
            CartesianGetParam.SetTimestamp(JointGetParam.Timestamp());
            CartesianGetParam.SetValid(true);
            // update cartesian velocity (caveat, velocities are not updated)
            const double dt = CartesianGetParam.Timestamp() - CartesianGetPreviousParam.Timestamp();
            if (dt > 0.0) {
                vct3 linearVelocity;
                linearVelocity.DifferenceOf(CartesianGetParam.Position().Translation(),
                                            CartesianGetPreviousParam.Position().Translation());
                linearVelocity.Divide(dt);
                CartesianVelocityGetParam.SetVelocityLinear(linearVelocity);
                CartesianVelocityGetParam.SetVelocityAngular(vct3(0.0));
                CartesianVelocityGetParam.SetTimestamp(CartesianGetParam.Timestamp());
                CartesianVelocityGetParam.SetValid(true);
            } else {
                CartesianVelocityGetParam.SetValid(false);
            }
            // update cartesian position desired based on joint desired
            CartesianGetDesired = Manipulator.ForwardKinematics(JointGetDesired);
            CartesianGetDesired.Rotation().NormalizedSelf();
            CartesianGetDesiredParam.SetTimestamp(JointGetParam.Timestamp());
            CartesianGetDesiredParam.SetValid(true);
        } else {
            // update cartesian position
            CartesianGet.Assign(vctFrm4x4::Identity());
            CartesianGetParam.SetValid(false);
            // update cartesian position desired
            CartesianGetDesired.Assign(vctFrm4x4::Identity());
            CartesianGetDesiredParam.SetValid(false);
        }
        CartesianGetParam.Position().From(CartesianGet);
        CartesianGetDesiredParam.Position().From(CartesianGetDesired);
    } else {
        // set joint to zeros
        JointGet.Zeros();
        JointGetParam.Position().Zeros();
        JointGetParam.SetValid(false);
    }
}

void mtsIntuitiveResearchKitArm::RunHomingPower(void)
{
    const double timeToPower = 3.0 * cmn_s;

    const double currentTime = this->StateTable.GetTic();
    // first, request power to be turned on
    if (!HomingPowerRequested) {
        // in case we still have power but brakes are not engaged
        if (NumberOfBrakes() > 0) {
            RobotIO.BrakeEngage();
        }
        // bias encoders based on pots
        RobotIO.BiasEncoder();
        // use pots for redundancy
        if (UsePotsForSafetyCheck()) {
            RobotIO.SetPotsToEncodersTolerance(PotsToEncodersTolerance);
            RobotIO.UsePotsForSafetyCheck(true);
        }
        HomingTimer = currentTime;
        // make sure the PID is not sending currents
        PID.Enable(false);
        // pre-load the boards with zero current
        RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
        // enable power and set a flags to move to next step
        RobotIO.EnablePower();
        HomingPowerRequested = true;
        MessageEvents.Status(this->GetName() + " power requested");
        return;
    }

    // second, check status
    if (HomingPowerRequested
        && ((currentTime - HomingTimer) > timeToPower)) {

        // pre-load PID to make sure desired position has some reasonable values
        PID.GetPositionJoint(JointGetParam);
        // assign to a more convenient vctDoubleVec
        JointGet.Assign(JointGetParam.Position(), NumberOfJoints());
        SetPositionJointLocal(JointGet);

        // check power status
        vctBoolVec actuatorAmplifiersStatus(NumberOfJoints());
        RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        vctBoolVec brakeAmplifiersStatus(NumberOfBrakes());
        if (NumberOfBrakes() > 0) {
            RobotIO.GetBrakeAmpStatus(brakeAmplifiersStatus);
        }
        if (actuatorAmplifiersStatus.All() && brakeAmplifiersStatus.All()) {
            MessageEvents.Status(this->GetName() + " power on");
            if (NumberOfBrakes() > 0) {
                RobotIO.BrakeRelease();
            }
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ARM);
        } else {
            // make sure the PID is not sending currents
            PID.Enable(false);
            RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
            RobotIO.DisablePower();
            MessageEvents.Error(this->GetName() + " failed to enable power.");
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
        }
    }
}

void mtsIntuitiveResearchKitArm::RunPositionJoint(void)
{
    if (IsGoalSet) {
        SetPositionJointLocal(JointSet);
    }
}

void mtsIntuitiveResearchKitArm::RunPositionGoalJoint(void)
{
    const double currentTime = this->StateTable.GetTic();
    if (currentTime <= JointTrajectory.EndTime) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
    } else {
        if (JointTrajectory.EndTime != 0.0) {
            JointTrajectory.GoalReachedEvent(true);
            JointTrajectory.EndTime = 0.0;
        }
    }
}

void mtsIntuitiveResearchKitArm::RunPositionCartesian(void)
{
    if (IsGoalSet) {
        // copy current position
        vctDoubleVec jointSet(JointGet.Ref(NumberOfJointsKinematics()));

        // compute desired slave position
        CartesianPositionFrm.From(CartesianSetParam.Goal());
        if (this->InverseKinematics(jointSet, CartesianPositionFrm) == robManipulator::ESUCCESS) {
            // assign to joints used for kinematics
            JointSet.Ref(NumberOfJointsKinematics()).Assign(jointSet);
            // finally send new joint values
            SetPositionJointLocal(JointSet);
        } else {
            MessageEvents.Error(this->GetName() + " unable to solve inverse kinematics.");
        }
        // reset flag
        IsGoalSet = false;
    }
}

void mtsIntuitiveResearchKitArm::RunPositionGoalCartesian(void)
{
    // trajectory are computed in joint space for now
    RunPositionGoalJoint();
}

void mtsIntuitiveResearchKitArm::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    JointSetParam.Goal().Zeros();
    JointSetParam.Goal().Assign(newPosition, NumberOfJoints());

    //Send it to both, logic to turn PID off when Simulink is on is handled through the QT Widget
    mtsExecutionResult executionResult;
    executionResult = PID.SetPositionJoint(JointSetParam);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": Call to PID.SetJointPosition failed \""
                                  << executionResult << "\"" << std::endl;
    }

    if(usingSimulink) {
        SimulinkController.GetControllerTypeIsJoint(SimulinkController.UsingJointControl);
        if(SimulinkController.UsingJointControl) {
            SimulinkController.SetDesiredJointPosition(JointSetParam);
        } else { //using Cartesian Control
            SimulinkController.SetDesiredCartesianPosition(CartesianSetParam);
        }
    }
}

void mtsIntuitiveResearchKitArm::SetPositionJoint(const prmPositionJointSet & newPosition)
{
    if (CurrentStateIs(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT)) {
        JointSet.Assign(newPosition.Goal(), NumberOfJoints());
        IsGoalSet = true;
    }
}

void mtsIntuitiveResearchKitArm::SetPositionGoalJoint(const prmPositionJointSet & newPosition)
{
    if (CurrentStateIs(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT)) {
        const double currentTime = this->StateTable.GetTic();
        // starting point is last requested to PID component
        JointTrajectory.Start.Assign(JointGetDesired, NumberOfJoints());
        JointTrajectory.Goal.Assign(newPosition.Goal(), NumberOfJoints());
        JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime, robLSPB::LSPB_DURATION);
        JointTrajectory.EndTime = currentTime + JointTrajectory.LSPB.Duration();
    }
}

void mtsIntuitiveResearchKitArm::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if ((RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN)
        || (RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_CONSTRAINT_CONTROLLER_CARTESIAN)) {
        CartesianSetParam = newPosition;
        IsGoalSet = true;
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionCartesian: Arm not ready" << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition)
{
    if (CurrentStateIs(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN)) {
        // copy current position
        vctDoubleVec jointSet(JointGet.Ref(NumberOfJointsKinematics()));

        // compute desired slave position
        CartesianPositionFrm.From(newPosition.Goal());

        if (this->InverseKinematics(jointSet, CartesianPositionFrm) == robManipulator::ESUCCESS) {
            // resize to proper number of joints
            jointSet.resize(NumberOfJoints());

            const double currentTime = this->StateTable.GetTic();
            // starting point is last requested to PID component
            JointTrajectory.Start.Assign(JointGetDesired, NumberOfJoints());
            JointTrajectory.Goal.Assign(jointSet);
            JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                     JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                     currentTime, robLSPB::LSPB_DURATION);
            JointTrajectory.EndTime = currentTime + JointTrajectory.LSPB.Duration();
        } else {
            MessageEvents.Error(this->GetName() + " unable to solve inverse kinematics.");
            JointTrajectory.GoalReachedEvent(false);
        }
    }
}


void mtsIntuitiveResearchKitArm::SetTorqueJointArm(const prmForceTorqueJointSet & newTorque)
{
    //just pass it on to PID, command coming from followTrajectory layer
    PID.SetTorqueJoint(newTorque);
}

void mtsIntuitiveResearchKitArm::GetTorqueJointArm( vctDoubleVec & currentTorque) const
{
    PID.GetTorqueJoint(currentTorque);
}

/*original was supposed to just return Jacobian, but couldn't pass double** through the provided interface,
so instead the calculation of the cartesian space velocity will be done here. Joint space velocity will be
passed in, this will use the available Jacobian and return the cartesian space velocity in the same vector*/
void mtsIntuitiveResearchKitArm::GetRobotCartVelFromJacobian( vctDoubleVec & slaveVelAndPos ) const
{
    //From Chapter 4 of "A mathematical introduction to robotic manipulation" by Murray

    vctDoubleVec jointVel, pos_t0, cartVel;
    jointVel.SetSize(NumberOfJoints()); //rad and m
    pos_t0.SetSize(3);
    cartVel.SetSize(6);

    for (unsigned int i=0; i < slaveVelAndPos.size(); i++)
    {
        if(i < jointVel.size())
            jointVel.at(i) = slaveVelAndPos.at(i);
        else
            pos_t0.at(i - jointVel.size()) = slaveVelAndPos.at(i);
    }

    //From Zhan-Fan Quek, Stanford
    vctDoubleFrm3 cartesianPosRotCurrent;
    vctFrame4x4<double> t_FKResults;
    vctDoubleRot3 t_FKResultsRotation;

    t_FKResults = Manipulator.ForwardKinematics(JointGet);
    Manipulator.JacobianSpatial(JointGet);

    // Update the rotation matrix of the robot end effector
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            t_FKResultsRotation[i][j] = (t_FKResults.Rotation())[i][j];
    }

    cartesianPosRotCurrent.From(t_FKResults);

    double** robotJacobian = Manipulator.Js; //spatial Jacobian
    vctDynamicMatrix<double> manipulatorJacobian;
    manipulatorJacobian.SetSize(6,NumberOfJoints());

    // Determine the manipulator jacobian
    for (int j = 0; j < 6; j++)
    {
        //NOTE the robotJacobian is the spatial Jacobian; columns are NOT associated with Joints
        //The first three rows are linear velocities; so it's the spatial Jacobian linear velocity plus the cross product of the position and the angular velocities (v = theta_dot x p)
        //And the last three rows are angular velocities. So your final manipulator Jacobian contains information about the linear and angular velocities of the end effector WITH RESPECT
        //TO the base frame. Or, http://summerschool2009.robotik-bs.de/downloads/lecture_notes/summerschool09_mmr_part09.pdf
        manipulatorJacobian[0][j] = robotJacobian[j][0] + robotJacobian[j][4] * cartesianPosRotCurrent.Translation()(2) - robotJacobian[j][5] * cartesianPosRotCurrent.Translation()(1);
        manipulatorJacobian[1][j] = robotJacobian[j][1] + robotJacobian[j][5] * cartesianPosRotCurrent.Translation()(0) - robotJacobian[j][3] * cartesianPosRotCurrent.Translation()(2);
        manipulatorJacobian[2][j] = robotJacobian[j][2] + robotJacobian[j][3] * cartesianPosRotCurrent.Translation()(1) - robotJacobian[j][4] * cartesianPosRotCurrent.Translation()(0);
        manipulatorJacobian[3][j] = robotJacobian[j][3];
        manipulatorJacobian[4][j] = robotJacobian[j][4];
        manipulatorJacobian[5][j] = robotJacobian[j][5];
    }

    // Calculate the end effector velocities
    for (unsigned int i = 0; i < 6; i++)
    {
        cartVel.at(i) = 0;

        for (unsigned int j = 0; j < NumberOfJoints()-1; j++)
            cartVel(i)  = cartVel(i)  + manipulatorJacobian(i,j)  * jointVel(j);
    }

    //put calculated cartesian velocity in return vector
    for (unsigned int i = 0; i < slaveVelAndPos.size(); i++)
        slaveVelAndPos.at(i) = (i < cartVel.size()) ? cartVel.at(i) : 0;
}

/*Similar to GetRobotCartVelFromJacobian, original was supposed to just return Jacobian, but couldn't pass
double** through the provided interface, so instead the calculation of the joint torques from force torque
will be done here. ForceTorque (size = 6) will be passed in size 7 (NumberOfJoints) vector torqeValues and
return joint torques in the same vector. Note, grip force is ASSUMED ZERO */
void mtsIntuitiveResearchKitArm::GetJointTorqueFromForceTorque(vctDoubleVec &torqueValues) const
{
    vctDoubleVec cartesianPIDCalculatedForceTorque, cartesianForceJointTorque, cartesianTorqueJointTorque, cartesianJointTorque;
    cartesianPIDCalculatedForceTorque.SetSize(6);
    cartesianPIDCalculatedForceTorque.SetAll(0.0);
    cartesianForceJointTorque.SetSize(NumberOfJoints());
    cartesianForceJointTorque.SetAll(0.0);
    cartesianTorqueJointTorque.SetSize(NumberOfJoints());
    cartesianTorqueJointTorque.SetAll(0.0);
    cartesianJointTorque.SetSize(NumberOfJoints()-1);
    cartesianJointTorque.SetAll(0.0);

    vct6 forcePID, torquePID;
    forcePID.SetAll(0.0);
    torquePID.SetAll(0.0);

    //Populate force/torque vector calculated by cartesian controller
    for (unsigned int i=0; i < cartesianPIDCalculatedForceTorque.size(); i++)
        cartesianPIDCalculatedForceTorque.at(i) = torqueValues.at(i);

    //From Zhan-Fan Quek, Stanford
    vctDoubleFrm3 cartesianPosRotCurrent;
    vctFrame4x4<double> t_FKResults;
    vctDoubleRot3 t_FKResultsRotation;

    t_FKResults = Manipulator.ForwardKinematics(JointGet);
    Manipulator.JacobianSpatial(JointGet);

    // Update the rotation matrix of the robot end effector
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            t_FKResultsRotation[i][j] = (t_FKResults.Rotation())[i][j];
    }

    cartesianPosRotCurrent.From(t_FKResults);

    double** robotJacobian = Manipulator.Js; //spatial Jacobian
    vctDynamicMatrix<double> manipulatorJacobian;
    manipulatorJacobian.SetSize(6,NumberOfJoints());

    // Determine the manipulator jacobian
    for (int j = 0; j < 6; j++)
    {
        //NOTE the robotJacobian is the spatial Jacobian; columns are NOT associated with Joints
        //The first three rows are linear velocities; so it's the spatial Jacobian linear velocity plus the cross product of the position and the angular velocities (v = theta_dot x p)
        //And the last three rows are angular velocities. So your final manipulator Jacobian contains information about the linear and angular velocities of the end effector WITH RESPECT
        //TO the base frame. Or, http://summerschool2009.robotik-bs.de/downloads/lecture_notes/summerschool09_mmr_part09.pdf
        manipulatorJacobian[0][j] = robotJacobian[j][0] + robotJacobian[j][4] * cartesianPosRotCurrent.Translation()(2) - robotJacobian[j][5] * cartesianPosRotCurrent.Translation()(1);
        manipulatorJacobian[1][j] = robotJacobian[j][1] + robotJacobian[j][5] * cartesianPosRotCurrent.Translation()(0) - robotJacobian[j][3] * cartesianPosRotCurrent.Translation()(2);
        manipulatorJacobian[2][j] = robotJacobian[j][2] + robotJacobian[j][3] * cartesianPosRotCurrent.Translation()(1) - robotJacobian[j][4] * cartesianPosRotCurrent.Translation()(0);
        manipulatorJacobian[3][j] = robotJacobian[j][3];
        manipulatorJacobian[4][j] = robotJacobian[j][4];
        manipulatorJacobian[5][j] = robotJacobian[j][5];
    }

    //assume UpdateManipulatorJacobian has been run recently
    // Calculate the respective joint torques from the commanded cartesian force/torque using the jacobian transform
    vctDynamicMatrix<double> manipulatorJacobianTranspose;
    manipulatorJacobianTranspose.SetSize(NumberOfJoints(), 6);
    manipulatorJacobianTranspose = manipulatorJacobian.Transpose();

    for (unsigned int i=0; i<cartesianPIDCalculatedForceTorque.size(); i++) {
        if(i < 3)
            forcePID(i)  = cartesianPIDCalculatedForceTorque(i);
        else
            torquePID(i) = cartesianPIDCalculatedForceTorque(i);
    }

    for (unsigned int i=0; i < NumberOfJoints()-1; i++) {
        for (unsigned int j=0; j < 6; j++) {
            cartesianForceJointTorque[i] = cartesianForceJointTorque[i] + manipulatorJacobianTranspose[i][j] * forcePID(j);
        }
    }

    for (unsigned int i=0; i < NumberOfJoints(); i++) {
        for (unsigned int j=0; j < 6; j++) {
            cartesianTorqueJointTorque[i] = cartesianTorqueJointTorque[i] + manipulatorJacobianTranspose[i][j] * torquePID(j);
        }
    }

    for (unsigned int i=0; i < cartesianJointTorque.size(); i++) {
        if(i < 3)
            cartesianJointTorque(i) = cartesianForceJointTorque(i);
        else
            cartesianJointTorque(i) = cartesianTorqueJointTorque(i);
    }

    for (unsigned int i=0; i < cartesianJointTorque.size(); i++)
        torqueValues.at(i) = cartesianJointTorque.at(i);
}

void mtsIntuitiveResearchKitArm::SetTrajectoryOnOrOff(const mtsBool & trajectoryRunning)
{
    if(usingSimulink)
    {
        mtsExecutionResult executionResult = SimulinkController.SignalTrajectoryRunning(trajectoryRunning);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_WARNING << GetName() << ": Call to SimulinkController.SignalTrajectoryRunning failed \""
                                      << executionResult << "\"" << std::endl;
        }
    }

    if(!trajectoryRunning)
    {
        IsGoalSet = trajectoryRunning; //turn it off properly
        logFile << "Stopping trajectory" << std::endl << std::endl;
    }
    else
    {
        logFile << "Starting trajectory" << std::endl << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::ErrorEventHandler(const std::string & message)
{
    RobotIO.DisablePower();
    // in case there was a trajectory going on
    if (JointTrajectory.EndTime != 0.0) {
        JointTrajectory.GoalReachedEvent(false);
        JointTrajectory.EndTime = 0.0;
    }
    MessageEvents.Error(this->GetName() + ": received [" + message + "]");
    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
}

void mtsIntuitiveResearchKitArm::JointLimitEventHandler(const vctBoolVec & flags)
{
    // in case there was a trajectory going on
    if (JointTrajectory.EndTime != 0.0) {
        JointTrajectory.GoalReachedEvent(false);
        JointTrajectory.EndTime = 0.0;
    }
    MessageEvents.Warning(this->GetName() + ": PID joint limit");
}

void mtsIntuitiveResearchKitArm::EnableLogs(const mtsBool &enable)
{
    logsEnabled = enable;
    if(logsEnabled)
    {
        //CMN_LOG_RUN_WARNING << "mtsIntuitiveResearchKitARM: logs enabled" << std::endl;

        // UBC open output file
        if(!logFile.is_open())
        {
            std::string fname = componentName;
            fname.append("ArmMDebug.log");
            CheckFileExists(&fname);

            logFile.open(fname.c_str(), std::ofstream::out | std::ofstream::app);
            if(!logFile.is_open())
                CMN_LOG_CLASS_RUN_WARNING << "Failed to open the PSM log file!" << std::endl;
        }

    }
    else {
        CloseLogs(false);
        //CMN_LOG_RUN_WARNING << "mtsIntuitiveResearchKitARM: logs disabled" << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::CloseLogs(bool closingTime)
{
    if(closingTime) {
        if(logsEnabled)
        {
            logFile << std::endl << "Ending now " << std::endl << std::endl << std::endl << std::endl << std::endl;
            logFile.close();
        }
    } else {
        if(logFile.is_open())
            logFile << "*********end entry " << logEntryIndex << std::endl << std::endl;
        logEntryIndex++;
    }
}

void mtsIntuitiveResearchKitArm::CheckFileExists(std::string *fname)
{
    unsigned int safety = 0;
    while(access(fname->c_str(), F_OK ) != -1) //while the file already exists
    {
        fname->insert(fname->find_last_of("."), "_");
        safety++;

        if(safety > 100) //this would be ridiculous
            break;
    }
}
