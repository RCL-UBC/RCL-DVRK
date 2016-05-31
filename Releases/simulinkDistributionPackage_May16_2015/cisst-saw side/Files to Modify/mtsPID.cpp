/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen
  Created on: 2013-02-22

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnXMLPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstCommonXML.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <sawControllers/mtsPID.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsPID, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);


mtsPID::mtsPID(const std::string &componentName, double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    Counter(0),
    CheckJointLimit(true),
    Enabled(false),
    EnabledIO(true),
    ConfigurationStateTable(100, "Configuration")
{
    taskName = componentName;
    AddStateTable(&ConfigurationStateTable);
    ConfigurationStateTable.SetAutomaticAdvance(false);
}


mtsPID::mtsPID(const mtsTaskPeriodicConstructorArg &arg):
    mtsTaskPeriodic(arg),
    Counter(0),
    CheckJointLimit(true),
    Enabled(false),
    EnabledIO(true),
    ConfigurationStateTable(100, "Configuration")
{
    AddStateTable(&ConfigurationStateTable);
    ConfigurationStateTable.SetAutomaticAdvance(false);
}


void mtsPID::SetupInterfaces(void)
{

    // require RobotJointTorque interface
    mtsInterfaceRequired * req = AddInterfaceRequired("RobotJointTorqueInterface");
    if (req) {
        req->AddFunction("GetJointType",     Robot.GetJointType);
        req->AddFunction("GetPositionJoint", Robot.GetFeedbackPosition);
        req->AddFunction("GetTorqueJoint",   Robot.GetFeedbackTorque);
//        req->AddFunction("GetVelocityJointSTOP", Robot.GetFeedbackVelocity, MTS_OPTIONAL);
        req->AddFunction("SetTorqueJoint",   Robot.SetTorque);
    }

    // this should go in "write" state table
    StateTable.AddData(Torque, "RequestedTorque");
    // this should go in a "read" state table
    StateTable.AddData(FeedbackPositionParam, "prmFeedbackPos");
    StateTable.AddData(FeedbackVelocityParam, "prmFeedbackVel");
    StateTable.AddData(FeedbackTorque, "FeedbackTorque");
    StateTable.AddData(DesiredPosition, "DesiredPosition");
    StateTable.AddData(CheckJointLimit, "IsCheckJointLimit");
    StateTable.AddData(Offset, "TorqueOffset");

    // configuration state table with occasional start/advance
    ConfigurationStateTable.AddData(Kp, "Kp");
    ConfigurationStateTable.AddData(Kd, "Kd");
    ConfigurationStateTable.AddData(Ki, "Ki");
    ConfigurationStateTable.AddData(JointLowerLimit, "JointLowerLimit");
    ConfigurationStateTable.AddData(JointUpperLimit, "JointUpperLimit");
    ConfigurationStateTable.AddData(JointType, "JointType");
    ConfigurationStateTable.AddData(mEnableTrackingError, "EnableTrackingError");
    ConfigurationStateTable.AddData(mTrackingErrorTolerances, "TrackingErrorTolerances");

    // provide SetDesiredPositions
    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Controller");
    if (interfaceProvided) {
        interfaceProvided->AddCommandVoid(&mtsPID::ResetController, this,         "ResetController");
        interfaceProvided->AddCommandWrite(&mtsPID::Enable, this,                 "Enable", false);
        interfaceProvided->AddCommandWrite(&mtsPID::EnableIO, this,               "EnableIO", false); //simulink too over
        interfaceProvided->AddCommandWrite(&mtsPID::EnableTorqueMode, this,       "EnableTorqueMode", TorqueMode);
        interfaceProvided->AddCommandRead(&mtsPID::GetTorqueModeEnabled, this,    "GetTorqueEnabled");
        interfaceProvided->AddCommandWrite(&mtsPID::SetDesiredTorques, this,      "SetTorqueJoint",   prmDesiredTrq);
        interfaceProvided->AddCommandWrite(&mtsPID::SetDesiredPositions, this,    "SetPositionJoint", DesiredPositionParam);
        interfaceProvided->AddCommandReadState(StateTable, FeedbackPositionParam, "GetPositionJoint");
        interfaceProvided->AddCommandReadState(StateTable, FeedbackVelocityParam, "GetVelocityJoint");
        interfaceProvided->AddCommandReadState(StateTable, FeedbackTorque,        "GetTorqueJoint");
        interfaceProvided->AddCommandReadState(StateTable, DesiredPosition,       "GetPositionJointDesired");
        interfaceProvided->AddCommandReadState(StateTable, Torque,                "GetEffortJointDesired");
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,"GetPeriodStatistics"); // mtsIntervalStatistics
        // Set check limits
        interfaceProvided->AddCommandWriteState(StateTable, CheckJointLimit, "SetCheckJointLimit");
        interfaceProvided->AddCommandWriteState(StateTable, Offset, "SetTorqueOffset");

        // Get PID gains
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, Kp, "GetPGain");
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, Kd, "GetDGain");
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, Ki, "GetIGain");
        // Get joint limits
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, JointLowerLimit, "GetJointLowerLimit");
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, JointUpperLimit, "GetJointUpperLimit");
        interfaceProvided->AddCommandReadState(ConfigurationStateTable, JointType, "GetJointType");

        // Error tracking
        interfaceProvided->AddCommandWriteState(ConfigurationStateTable, mEnableTrackingError, "EnableTrackingError");
        interfaceProvided->AddCommandWrite(&mtsPID::SetTrackingErrorTolerances, this, "SetTrackingErrorTolerances");

        // Set PID gains
        interfaceProvided->AddCommandWrite(&mtsPID::SetPGain, this, "SetPGain", Kp);
        interfaceProvided->AddCommandWrite(&mtsPID::SetDGain, this, "SetDGain", Kd);
        interfaceProvided->AddCommandWrite(&mtsPID::SetIGain, this, "SetIGain", Ki);
        // Set joint limits
        interfaceProvided->AddCommandWrite(&mtsPID::SetJointLowerLimit, this, "SetJointLowerLimit", JointLowerLimit);
        interfaceProvided->AddCommandWrite(&mtsPID::SetJointUpperLimit, this, "SetJointUpperLimit", JointUpperLimit);

        // Events
        interfaceProvided->AddEventWrite(this->EventPIDEnable, "EventPIDEnable", false);
        // For logging
        interfaceProvided->AddCommandWrite(&mtsPID::SignalTrajectoryEvent, this, "SignalTrajEvent", mtsBool());
        interfaceProvided->AddCommandWrite(&mtsPID::EnableLogs, this, "EnableLogs", mtsBool());

        interfaceProvided->AddEventVoid(this->EventTrackingError, "TrackingError");
    }
}

void mtsPID::Configure(const std::string & filename)
{
    ConfigurationStateTable.Start();

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;
    cmnXMLPath config;
    config.SetInputSource(filename);

    // check type, interface and number of joints
    std::string type, interface;
    int numJoints = -1;
    config.GetXMLValue("/controller", "@type", type, "");
    config.GetXMLValue("/controller", "@interface", interface, "");
    config.GetXMLValue("/controller", "@numofjoints", numJoints, -1);
    if (type != "PID") {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: wrong controller type" << std::endl;
        ConfigurationStateTable.Advance();
        return;
    } else if (interface != "JointTorqueInterface") {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: wrong interface. Require JointTorqueInterface" << std::endl;
        ConfigurationStateTable.Advance();
        return;
    } else if (numJoints < 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: invalid number of joints" << std::endl;
        ConfigurationStateTable.Advance();
        return;
    }

    // set dynamic var size
    Kp.SetSize(numJoints);
    Kd.SetSize(numJoints);
    Ki.SetSize(numJoints);
    Offset.SetSize(numJoints);
    Offset.SetAll(0.0);
    JointLowerLimit.SetSize(numJoints);
    JointLowerLimit.SetAll(0.0);
    JointUpperLimit.SetSize(numJoints);
    JointUpperLimit.SetAll(0.0);

    // feedback
    FeedbackPosition.SetSize(numJoints);
    DesiredPosition.SetSize(numJoints);
    DesiredPosition.SetAll(0.0);
    FeedbackTorque.SetSize(numJoints);
    DesiredTorque.SetSize(numJoints);
    DesiredTorque.SetAll(0.0);
    FeedbackVelocity.SetSize(numJoints);
    DesiredVelocity.SetSize(numJoints);
    Torque.SetSize(numJoints);
    Torque.SetAll(0.0);

    FeedbackPositionParam.SetSize(numJoints);
    FeedbackPositionPreviousParam.SetSize(numJoints);
    DesiredPositionParam.SetSize(numJoints);
    FeedbackVelocityParam.SetSize(numJoints);
    TorqueParam.SetSize(numJoints);

    // errors
    Error.SetSize(numJoints);
    ErrorAbsolute.SetSize(numJoints);
    dError.SetSize(numJoints);
    iError.SetSize(numJoints);
    oldError.SetSize(numJoints);
    oldDesiredPos.SetSize(numJoints);
    ResetController();

    minIErrorLimit.SetSize(numJoints);
    minIErrorLimit.SetAll(-100.0);
    maxIErrorLimit.SetSize(numJoints);
    maxIErrorLimit.SetAll(100.0);

    // default 1.0: no effect
    forgetIError.SetSize(numJoints);
    forgetIError.SetAll(1.0);

    // default: use regular PID
    nonlinear.SetSize(numJoints);
    nonlinear.SetAll(0.0);

    DeadBand.SetSize(numJoints);
    DeadBand.SetAll(0.0);

    // torque mode
    TorqueMode.SetSize(numJoints);
    TorqueMode.SetAll(false);

    // tracking error
    mEnableTrackingError = false;
    mTrackingErrorTolerances.SetSize(numJoints);
    mTrackingErrorTolerances.SetAll(0.0);

    // read data from xml file
    char context[64];
    for (int i = 0; i < numJoints; i++) {
        // joint
        sprintf(context, "controller/joints/joint[%d]", i+1);
        // pid
        config.GetXMLValue(context, "pid/@PGain", Kp[i]);
        config.GetXMLValue(context, "pid/@DGain", Kd[i]);
        config.GetXMLValue(context, "pid/@IGain", Ki[i]);
        config.GetXMLValue(context, "pid/@OffsetTorque", Offset[i]);
        config.GetXMLValue(context, "pid/@Forget", forgetIError[i]);
        config.GetXMLValue(context, "pid/@Nonlinear", nonlinear[i]);

        // limit
        config.GetXMLValue(context, "limit/@MinILimit", minIErrorLimit[i]);
        config.GetXMLValue(context, "limit/@MaxILimit", maxIErrorLimit[i]);
        config.GetXMLValue(context, "limit/@ErrorLimit", mTrackingErrorTolerances[i]);
        config.GetXMLValue(context, "limit/@Deadband", DeadBand[i]);

        // joint limit
        CheckJointLimit = true;
        std::string tmpUnits;
        bool ret = false;
        ret = config.GetXMLValue(context, "pos/@Units", tmpUnits);
        if (ret) {
            config.GetXMLValue(context, "pos/@LowerLimit", JointLowerLimit[i]);
            config.GetXMLValue(context, "pos/@UpperLimit", JointUpperLimit[i]);
            if (tmpUnits == "deg") {
                JointLowerLimit[i] *= cmnPI_180;
                JointUpperLimit[i] *= cmnPI_180;
            } else if (tmpUnits == "mm") {
                JointLowerLimit[i] *= cmn_mm;
                JointUpperLimit[i] *= cmn_mm;
            }
        } else {
            CheckJointLimit = false;
        }
    }


    // Convert from degrees to radians
    // TODO: Decide whether to use degrees or radians in XML file
    // TODO: Also do this for other parameters (not just Deadband)
    // TODO: Only do this for revolute joints
    DeadBand.Multiply(cmnPI_180);

    CMN_LOG_CLASS_INIT_VERBOSE << "Kp: " << Kp << std::endl
                               << "Kd: " << Kd << std::endl
                               << "Ki: " << Ki << std::endl
                               << "Offset: " << Offset << std::endl
                               << "JntLowerLimit" << JointLowerLimit << std::endl
                               << "JntUpperLimit" << JointUpperLimit << std::endl
                               << "Deadband: " << DeadBand << std::endl
                               << "minLimit: " << minIErrorLimit << std::endl
                               << "maxLimit: " << maxIErrorLimit << std::endl
                               << "elimit: " << mTrackingErrorTolerances << std::endl
                               << "forget: " << forgetIError << std::endl;

    ConfigurationStateTable.Advance();

    logsEnabled                 = false;
    trajRunning                 = false;

    // now that we know the sizes of vectors, create interfaces
    this->SetupInterfaces();
}

void mtsPID::Startup(void)
{
    // get joint type and store in configuration state table
    mtsExecutionResult result;
    ConfigurationStateTable.Start();
    result = Robot.GetJointType(JointType);
    ConfigurationStateTable.Advance();
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                 << result << std::endl;
    }
}

void mtsPID::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    // increment counter
    Counter++;

    // update position, insertion is in mm, all others in radians!!!
    Robot.GetFeedbackPosition(FeedbackPositionParam);
    FeedbackPositionParam.GetPosition(FeedbackPosition);
    Robot.GetFeedbackTorque(FeedbackTorque);
    // compute error
    Error.DifferenceOf(DesiredPosition, FeedbackPosition);
    for (size_t i = 0; i < Error.size(); i++) {
        if ((Error[i] <= DeadBand[i]) && (Error[i] >= -DeadBand[i]))
            Error[i] = 0.0;
    }

    // update velocities
    if (Robot.GetFeedbackVelocity.IsValid()) {
        Robot.GetFeedbackVelocity(FeedbackVelocityParam);
        FeedbackVelocityParam.GetVelocity(FeedbackVelocity);
    } else {
#if 0
        double dt = StateTable.Period;
        if (dt > 0) {
            FeedbackVelocity.DifferenceOf(oldError, Error);
            FeedbackVelocity.Divide(dt);
        } else {
            FeedbackVelocity.SetAll(0.0);
        }
        // set param so data in state table is accurate
        FeedbackVelocityParam.SetVelocity(FeedbackVelocity);
#endif
#if 1
        double dt = FeedbackPositionParam.Timestamp() - FeedbackPositionPreviousParam.Timestamp();
        if (dt > 0) {
            FeedbackVelocity.DifferenceOf(FeedbackPositionParam.Position(),
                                          FeedbackPositionPreviousParam.Position());

            FeedbackVelocity.Divide(dt);
        } else {
            FeedbackVelocity.SetAll(0.0);
        }
        // set param so data in state table is accurate
        FeedbackVelocityParam.SetVelocity(FeedbackVelocity);
#endif
    }

    // compute torque
    if (Enabled) {
        // check for tracking error
        if (mEnableTrackingError) {
            ErrorAbsolute.AbsOf(Error);
            if (!ErrorAbsolute.LesserOrEqual(mTrackingErrorTolerances)) {
                EventTrackingError();
                Enable(false);
                std::string errorMessage = this->Name + ": tracking error, errors: ";
                errorMessage.append(Error.ToString());
                errorMessage.append(", tolerances: ");
                errorMessage.append(mTrackingErrorTolerances.ToString());
                errorMessage.append(", mask: ");
                errorMessage.append(ErrorAbsolute.ElementwiseLesserOrEqual(mTrackingErrorTolerances).ToString());
                cmnThrow(errorMessage);
            }
        }

        dError.Assign(FeedbackVelocity);
        dError.Multiply(-1.0);

        // compute error integral
        iError.ElementwiseMultiply(forgetIError);
        iError.Add(Error);

        // check error limit & clamp iError
        for (size_t i = 0; i < iError.size(); i++) {
            // iError clamping
            if (iError.at(i) > maxIErrorLimit.at(i)) 
                iError.at(i) = maxIErrorLimit.at(i);
            else if (iError.at(i) < minIErrorLimit.at(i))
                iError.at(i) = minIErrorLimit.at(i);
        }

        // save Error to oldError
        oldError.Assign(Error);
        oldDesiredPos.Assign(DesiredPosition);

        // compute torque
        Torque.ElementwiseProductOf(Kp, Error);
        Torque.AddElementwiseProductOf(Kd, dError);
        Torque.AddElementwiseProductOf(Ki, iError);


        // nonlinear control mode
        // UBC: some non-linear control for MTMs, not PSMDesiredPosition
        for (size_t i = 0; i < Error.size(); i++) {
            if ((nonlinear[i] > 0)
                && (fabs(Error[i]) < nonlinear[i])) {
                Torque[i] = Torque[i] * fabs(Error[i]) / nonlinear[i];
            }
        }

        // Set Torque to DesiredTorque if Torque Mode enabled (e.g. RunGravityCompensation from mtsIntuitiveResearchKitMTM.cpp)
        for (size_t i = 0; i < Torque.size(); i++) {
            if (TorqueMode[i]) {
                Torque[i] = DesiredTorque[i];
            }
        }

        // Add torque (e.g. gravity compensation)
        Torque.Add(Offset);

        if(logsEnabled)
        {
            struct timespec spec;
            clock_gettime(CLOCK_REALTIME, &spec);
            time_t s  = spec.tv_sec;
            long ms = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
            if(ms == 1000)
            {
                s = s+1;
                ms = 0;
            }
            // pretty gross hack, but didn't want to get into converting long to string then padding it
            std::string zeros = "";
            if(ms < 10) {
                zeros = "00";
            } else if(ms < 100) {
                zeros = "0";
            }

            PIDLogFile << "t = " << (intmax_t)s << "." << zeros << ms << std::endl
                       << "Desired Pos = "  << std::setprecision(5) << DesiredPosition  << std::endl
                       << "Current Pos = "  << std::setprecision(5) << FeedbackPosition << std::endl
                       << "Feedback Vel = " << std::setprecision(5) << FeedbackVelocity << std::endl
                       << "oError  = "      << std::setprecision(5) << Error            << std::endl
                       << "dError = "       << std::setprecision(5) << dError           << std::endl
                       << "iError = "       << std::setprecision(5) << iError           << std::endl
                       << "Torque = "       << std::setprecision(5) << Torque           << std::endl << std::endl;
        }

        // write torque to robot
        TorqueParam.SetForceTorque(Torque);

        if(EnabledIO) {
            Robot.SetTorque(TorqueParam);
            //CMN_LOG_RUN_WARNING << "RunPID: " << GetName() << std::setprecision(5) << Torque << std::endl;
        }

        if (Counter % 1000 == 0) {
            //CMN_LOG_RUN_WARNING << "RunPID: " << GetName() << std::setprecision(5) << Torque << std::endl;
            CMN_LOG_CLASS_RUN_DEBUG  << "Run: " << GetName() << std::setprecision(5) << Torque << std::endl;
        }
    }
    else {
        if (Counter % 100 == 0) {
            CMN_LOG_CLASS_RUN_DEBUG << "Run: " << GetName() << " disabled  "
                                    << Torque << std::endl;
        }
        Torque.SetAll(0.0);
        TorqueParam.SetForceTorque(Torque);
        Robot.SetTorque(TorqueParam);
    }

    // save previous position
    FeedbackPositionPreviousParam = FeedbackPositionParam;
}

void mtsPID::Cleanup(void)
{
    CloseLogs(true);
    // cleanup
    Torque.SetAll(0.0);
    TorqueParam.SetForceTorque(Torque);
    Robot.SetTorque(TorqueParam);
}

void mtsPID::SetPGain(const vctDoubleVec & pgain)
{
    if (pgain.size() != Kp.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetPGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        Kp.Assign(pgain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetDGain(const vctDoubleVec & dgain)
{
    if (dgain.size() != Kd.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetDGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        Kd.Assign(dgain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetIGain(const vctDoubleVec & igain)
{
    if (igain.size() != Ki.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetIGain: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        Ki.Assign(igain);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetJointLowerLimit(const vctDoubleVec &lowerLimit)
{
    if (lowerLimit.size() != JointLowerLimit.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetJointLowerLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        JointLowerLimit.Assign(lowerLimit);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetJointUpperLimit(const vctDoubleVec &upperLimit)
{
    if (upperLimit.size() != JointUpperLimit.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetJointUpperLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        JointUpperLimit.Assign(upperLimit);
        ConfigurationStateTable.Advance();
    }
}


void mtsPID::SetMinIErrorLimit(const vctDoubleVec & iminlim)
{
    if (iminlim.size() != minIErrorLimit.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetMinIErrorLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        minIErrorLimit.Assign(iminlim);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetMaxIErrorLimit(const vctDoubleVec & imaxlim)
{
    if (imaxlim.size() != maxIErrorLimit.size()) {
        CMN_LOG_CLASS_INIT_ERROR << "SetMaxIErrorLimit: size mismatch" << std::endl;
    } else {
        ConfigurationStateTable.Start();
        maxIErrorLimit.Assign(imaxlim);
        ConfigurationStateTable.Advance();
    }
}

void mtsPID::SetForgetIError(const double & forget)
{
    forgetIError = forget;
}


//------------- Protected Method ---------------

void mtsPID::ResetController(void)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "Reset Controller" << std::endl;

    Error.SetAll(0.0);
    oldError.SetAll(0.0);
    dError.SetAll(0.0);
    iError.SetAll(0.0);

    prmPositionJointSet setPrmPos;
    setPrmPos.SetSize(Error.size());
    setPrmPos.SetGoal(FeedbackPosition);
    SetDesiredPositions(setPrmPos);

    DesiredVelocity.SetAll(0.0);
}

void mtsPID::SetDesiredTorques(const prmForceTorqueJointSet & prmTrq)
{
    prmDesiredTrq = prmTrq;
    prmDesiredTrq.GetForceTorque( DesiredTorque );
    //above takes torque values from prmTrq and populates DesiredTorque with them
    //the specified desiredTorque is held/commanded to robot until a new torque is provided
}

void mtsPID::SetDesiredPositions(const prmPositionJointSet & positionParam)
{
    DesiredPositionParam = positionParam;
    DesiredPositionParam.GetGoal(DesiredPosition);

    if (CheckJointLimit) {
        // limit check: clip the desired position
        DesiredPosition.ElementwiseMin(JointUpperLimit);
        DesiredPosition.ElementwiseMax(JointLowerLimit);
    }

    double dt = StateTable.Period;
    DesiredVelocity = (DesiredPosition - oldDesiredPos) / dt;
}


void mtsPID::Enable(const bool & enable)
{
    Enabled = enable;

    // set torque to 0
    Torque.SetAll(0.0);

    // write torque to robot
    TorqueParam.SetForceTorque(Torque);
    Robot.SetTorque(TorqueParam);

    // trigger EventPIDEnable
    EventPIDEnable(Enabled);
}

void mtsPID::EnableIO(const bool & enable)
{
    EnabledIO = enable;
}

void mtsPID::EnableTorqueMode(const vctBoolVec &ena)
{
    if (TorqueMode.size() != ena.size()) {
        CMN_LOG_CLASS_RUN_ERROR << "EnableTorqueMode size mismatch" << std::endl;
        return;
    } else {
        TorqueMode.Assign(ena);
    }

    // set torque to 0
    Torque.SetAll(0.0);

    // write torque to robot
    TorqueParam.SetForceTorque(Torque);
    Robot.SetTorque(TorqueParam);
}

void mtsPID::SignalTrajectoryEvent(const mtsBool & trajInProgress)
{
    trajRunning = trajInProgress;
    if(trajRunning)
    {
        if(logsEnabled)
            PIDLogFile << "*********Start Trajectory" << std::endl;
    }
    else //if (!trajRunning)
    {
        //denote in log that it's over
        if(logsEnabled)
            PIDLogFile << "*********End Trajectory" << std::endl;
    }
}

void mtsPID::GetTorqueModeEnabled(vctBoolVec &torqueEnabled) const
{
    torqueEnabled = TorqueMode;
}

void mtsPID::EnableLogs(const mtsBool &enable)
{
    logsEnabled = enable;
    if(logsEnabled) // !logsEnabled && enable) //logs previously disabled
    {
        //CMN_LOG_RUN_WARNING << "mtsPID: logs enabled" << std::endl;

        // UBC open output file
        if(!PIDLogFile.is_open())
        {
            std::string fname = taskName;
            fname.append("mtsPID.log");
            checkFileExists(&fname);

            PIDLogFile.open(fname.c_str(), std::ofstream::out | std::ofstream::app);
            if(!PIDLogFile.is_open())
                CMN_LOG_CLASS_RUN_WARNING << "Failed to open the mtsPID log file!" << std::endl;

        }

        PIDLogFile << "*********Log number:   " << logEntryIndex << std::endl;
        PIDLogFile << "PID Params:" << std::endl
                   << "J0: Kp = " << Kp(0) << " Kd = " << Kd(0) << " Ki = " << Ki(0) << std::endl
                   << "J1: Kp = " << Kp(1) << " Kd = " << Kd(1) << " Ki = " << Ki(1) << std::endl
                   << "J2: Kp = " << Kp(2) << " Kd = " << Kd(2) << " Ki = " << Ki(2) << std::endl
                   << "J3: Kp = " << Kp(3) << " Kd = " << Kd(3) << " Ki = " << Ki(3) << std::endl
                   << "J4: Kp = " << Kp(4) << " Kd = " << Kd(4) << " Ki = " << Ki(4) << std::endl
                   << "J5: Kp = " << Kp(5) << " Kd = " << Kd(5) << " Ki = " << Ki(5) << std::endl;

    }
    else {
        CloseLogs(false);
        //CMN_LOG_RUN_WARNING << "mtsPID: logs disabled" << std::endl;
    }
}

void mtsPID::CloseLogs(bool closingTime)
{
    if(closingTime) {
        if(logsEnabled)
        {
            PIDLogFile << std::endl << "*********************************END OF TEST, APPLICATION CLOSED" << std::endl;
            PIDLogFile.close();
        }
    } else {
        if(PIDLogFile.is_open())
            PIDLogFile << "*********end entry " << logEntryIndex << std::endl << std::endl;
        logEntryIndex++;
    }
}

void mtsPID::checkFileExists(std::string *fname)
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

void mtsPID::SetTrackingErrorTolerances(const vctDoubleVec & tolerances)
{
    if (tolerances.size() == mTrackingErrorTolerances.size()) {
        mTrackingErrorTolerances.Assign(tolerances);
    } else {
        std::string errorMessage = this->Name + ": incorrect vector size for SetTrackingErrorTolerances";
        cmnThrow(errorMessage);
    }
}
