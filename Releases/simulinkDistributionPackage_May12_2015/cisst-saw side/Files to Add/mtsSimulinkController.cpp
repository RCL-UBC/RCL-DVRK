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

#include <cisstCommon/cmnXMLPath.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstCommonXML.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <sawControllers/mtsSimulinkController.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsSimulinkController, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

/******************Public Functions******************/
mtsSimulinkController::mtsSimulinkController(const std::string &componentName, double periodInSeconds, const unsigned int NumJoints):
    mtsTaskPeriodic(componentName, periodInSeconds),
    Counter(0),
    Enabled(false),
    usingJointController(true)
{
    NumberOfJoints = NumJoints;
    taskName = componentName;
    tryingToInitializeSim = -1;
}

mtsSimulinkController::mtsSimulinkController(const mtsTaskPeriodicConstructorArg &arg):
    mtsTaskPeriodic(arg),
    Counter(0),
    Enabled(false),
    usingJointController(true)
{ }

void mtsSimulinkController::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsSimulinkController::Configure: " << filename << std::endl;

    logsEnabled                   = false;
    trajRunning                   = false;
    simulinkIsReady               = false;
    disablePIDNextReceivedPacket  = false;

    watchDogTimeOutMS             = 300;
    tryingToInitializeSim         = -1;
    initializingMaxAttempts       = 8000; //8 seconds

    socketComm = 0;
    dataPacket = 0;

    SocketConnectionInfo.hostIP          = mtsStdString("");
    SocketConnectionInfo.kinematicsPort  = mtsUShort(0);
    SocketConnectionInfo.torquePort      = mtsUShort(0);

    // feedback
    FeedbackPosition.SetSize(NumberOfJoints);
    DesiredPosition.SetSize(NumberOfJoints);
    DesiredPosition.SetAll(0.0);
    FeedbackVelocity.SetSize(NumberOfJoints);
    FeedbackVelocityPrev.SetSize(NumberOfJoints);
    FeedbackVelocityPrev.SetAll(0.0);
    CartesianCurrentPos.SetSize(12); //[t] (3x1), [R] (3x3)
    CartesianCurrentPos.SetAll(0.0);
    CartesianDesiredPos.SetSize(12);
    CartesianDesiredPos.SetAll(0.0);
    CartesianVelocity.SetSize(6.0);
    CartesianVelocity.SetAll(0.0);
    ForceTorque.SetSize(6.0);
    ForceTorque.SetAll(0.0);
    Torque.SetSize(NumberOfJoints);
    Torque.SetAll(0.0);

    DesiredPositionParam.SetSize(NumberOfJoints);
    FeedbackPositionParam.SetSize(NumberOfJoints);
    FeedbackPositionPreviousParam.SetSize(NumberOfJoints);
    FeedbackVelocityParam.SetSize(NumberOfJoints);
    TorqueParam.SetSize(NumberOfJoints);

    SetupInterfaces();
}

void mtsSimulinkController::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsSimulinkController::Startup" << std::endl;
}

void mtsSimulinkController::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    // increment counter
    Counter++;

    // compute torque
    if (Enabled) {
        SendDataToSimulink();
        ReceiveFromSimulink();

        //if don't actually receive anything from Simulink, just sent old torque value
        if(!disablePIDNextReceivedPacket) { //just not for first wait
            //write torque to robot
            TorqueParam.SetForceTorque(Torque);
            Robot.SetTorque(TorqueParam);
        }

        if(simulinkIsReady) { //successfully received something
            LogEntry(true);
        } else {
            LogEntry(false);
        }
  }

    //try to connect socket
    if(tryingToInitializeSim >= 0)
    {
        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 100; //got .1ms to try

        //assume the worst
        bool recvConnected = socketComm->IsRecvSocketConnected();
        bool sendConnected = socketComm->IsSendSocketConnected();

        if(!recvConnected) { //try receive first
            recvConnected = socketComm->ConnectRecvSocket(tv);
        }

        if(!sendConnected) { //try send next
            sendConnected = socketComm->ConnectSendSocket(tv);
        }

        tryingToInitializeSim++;

        if(recvConnected && sendConnected) {
            Enabled = true;
            disablePIDNextReceivedPacket = true;
            simulinkIsReady = true;
            tryingToInitializeSim = -1;
        }
        else {
            if((tryingToInitializeSim > initializingMaxAttempts)) {
                CMN_LOG_RUN_WARNING << "Too many initialization attempts; Simulink controller is NOT enabled" << std::endl;
                DoneCommunicationWithSimulink(true);
                tryingToInitializeSim = -1;
            }
        }
    }
}

void mtsSimulinkController::Cleanup(void)
{
    CloseLogs(true);
    // cleanup
    Torque.SetAll(0.0);
    TorqueParam.SetForceTorque(Torque);
    Robot.SetTorque(TorqueParam);
}

/******************Protected Functions: General******************/
void mtsSimulinkController::SetupInterfaces(void)
{
    // require RobotJointTorque interface
    mtsInterfaceRequired * req = AddInterfaceRequired("RobotJointTorqueInterface");
    if (req) {
        req->AddFunction("GetPositionJoint",    Robot.GetFeedbackPosition);
        req->AddFunction("SetTorqueJoint",      Robot.SetTorque);
    }

    // this should go in "write" state table
    StateTable.AddData(Torque,                "RequestedTorque");
    // this should go in a "read" state table
    StateTable.AddData(FeedbackPositionParam, "prmFeedbackPos");
    StateTable.AddData(FeedbackVelocityParam, "prmFeedbackVel");
    StateTable.AddData(DesiredPosition,       "DesiredPosition");

    // require to uncheck/disable simulink control in higher level (should make this a signal)
    mtsInterfaceRequired * reqSimulinkControl = AddInterfaceRequired("SignalSimulinkSocketsDone");
    if (reqSimulinkControl) {
        reqSimulinkControl->AddFunction("SignalSimulinkDone",  TopLevelWidgetSimulink.SignalSimulinkDone);
    }

    mtsInterfaceRequired * reqRobotPSM = AddInterfaceRequired("RobotPSM");
    if(reqRobotPSM) {
        reqRobotPSM->AddFunction("GetRobotCartVelFromJacobian",   RobotArm.GetRobotCartVelFromJacobian);
        reqRobotPSM->AddFunction("GetJointTorqueFromForceTorque", RobotArm.GetJointTorqueFromForceTorques);
        reqRobotPSM->AddFunction("GetPositionCartesian",          RobotArm.GetPositionCartesian);
    }

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("SimulinkController");
    if (interfaceProvided) {
        interfaceProvided->AddCommandWrite(&mtsSimulinkController::Enable,                   this,  "Enable",                   false);
        interfaceProvided->AddCommandWrite(&mtsSimulinkController::EnableLogs,               this,  "EnableLogs",               mtsBool());
        interfaceProvided->AddCommandWrite(&mtsSimulinkController::SetHostIP,                this,  "SetHostIP",                mtsStdString());
        interfaceProvided->AddCommandWrite(&mtsSimulinkController::SetKinematicsPort,        this,  "SetKinematicsPortNum",     mtsUShort());
        interfaceProvided->AddCommandWrite(&mtsSimulinkController::SetTorquePort,            this,  "SetTorquePortNum",         mtsUShort());
        interfaceProvided->AddCommandWrite(&mtsSimulinkController::SetDesiredPositions,      this,  "SetPositionJoint",         DesiredPositionParam);
        interfaceProvided->AddCommandWrite(&mtsSimulinkController::SetDesiredCartPosition,   this,  "SetPositionCartDes",       CartesianSetParam);
        interfaceProvided->AddCommandWrite(&mtsSimulinkController::SetControllerType,        this,  "SetSimControllerType",     mtsBool());
        interfaceProvided->AddCommandRead(&mtsSimulinkController::GetControllerType,         this,  "GetSimControllerType");

        interfaceProvided->AddCommandReadState(StateTable, DesiredPosition,                         "GetSimPositionJointDesired");
        interfaceProvided->AddCommandReadState(StateTable, Torque,                                  "GetSimEffortJointDesired");
        interfaceProvided->AddCommandReadState(StateTable, FeedbackPositionParam,                   "GetSimPositionJoint");
        interfaceProvided->AddCommandReadState(StateTable, FeedbackVelocityParam,                   "GetSimVelocityJoint");
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,                  "GetPeriodStatistics"); // mtsIntervalStatistics

        //only from arm
        interfaceProvided->AddCommandWrite(&mtsSimulinkController::SignalTrajectoryEvent,    this,  "SignalTrajEvent",          mtsBool());
    }
}

void mtsSimulinkController::Enable(const bool & enable)
{
    //if turning off, turn off right away. Otherwise, enable at the end so don't try anything funny while setting up
    if(!enable)
        Enabled = enable;

    if(enable)
    {
        if(!SocketConnectionInfo.isInitialized())
        {
            CMN_LOG_RUN_WARNING << "Please select valid socket connection parameters (hostIP, kinematics port number, torque port number)" << std::endl;
            Enabled = !enable;
            return;
        }

        //setup sockets
        if(usingJointController) {
            //socketComm = new prmTwoWaySocket(54321,12345,"137.82.56.245", 69, 326);
            socketComm = new prmTwoWaySocket(SocketConnectionInfo.torquePort,
                                             SocketConnectionInfo.kinematicsPort,
                                             SocketConnectionInfo.hostIP, 69, 326);
        } else {
            //socketComm = new prmTwoWaySocket(54321,12345,"137.82.56.245", 54, 452);
            socketComm = new prmTwoWaySocket(SocketConnectionInfo.torquePort,
                                             SocketConnectionInfo.kinematicsPort,
                                             SocketConnectionInfo.hostIP, 54, 452);
        }

        tryingToInitializeSim = 0;
    }
    else {
        if(socketComm == 0) {
            //CMN_LOG_RUN_WARNING << "What are you doing? You already disabled the simulink controller!" << std::endl;
            return;
        }

        //CMN_LOG_RUN_WARNING << "Disabling Simulink Controller" << std::endl;
        socketComm->CloseRecvSocket();
        socketComm->CloseSendSocket();

        delete socketComm;
        socketComm = 0;

        //should already be false, but just in case
        simulinkIsReady              = false;
        disablePIDNextReceivedPacket = false;

        tryingToInitializeSim = -1;
    }
}

/******************Protected Functions: Interfaces******************/
void mtsSimulinkController::SignalTrajectoryEvent(const mtsBool & trajInProgress)
{
    trajRunning = trajInProgress;
    if(trajRunning)
    {
        if(logsEnabled)
            SimulinkControllerLogFile << "*********Start Trajectory" << std::endl;
    }
    else //if (!trajRunning)
    {
        //denote in log that it's over
        if(logsEnabled)
            SimulinkControllerLogFile << "*********End Trajectory" << std::endl;
    }
}

void mtsSimulinkController::SetControllerType(const mtsBool &isJointControl)
{
    usingJointController = isJointControl;
}

void mtsSimulinkController::GetControllerType(mtsBool &isJointControl) const
{
    isJointControl = usingJointController;
}

void mtsSimulinkController::SetDesiredPositions(const prmPositionJointSet & positionParam)
{
    if(!usingJointController) {
        CMN_LOG_RUN_WARNING << "mtsSimulinkController::SetDesiredPositions COMMANDING JOINT POSITION WHEN RUNNING CARTESIAN CONTROLLER. NOT GOOD" << std::endl;
        CMN_LOG_RUN_ERROR << "mtsSimulinkController::SetDesiredPositions COMMANDING JOINT POSITION WHEN RUNNING CARTESIAN CONTROLLER. NOT GOOD" << std::endl;
    }

    DesiredPositionParam = positionParam;
    DesiredPositionParam.GetGoal(DesiredPosition);
}

void mtsSimulinkController::SetDesiredCartPosition(const prmPositionCartesianSet &newPosition)
{
    if(usingJointController){
        CMN_LOG_RUN_WARNING << "mtsSimulinkController::SetDesiredPositions COMMANDING CARTESIAN POSITION WHEN RUNNING JOINT CONTROLLER. NOT GOOD" << std::endl;
        CMN_LOG_RUN_ERROR << "mtsSimulinkController::SetDesiredPositions COMMANDING CARTESIAN POSITION WHEN RUNNING JOINT CONTROLLER. NOT GOOD" << std::endl;
    }

    CartesianSetParam = newPosition;
    vctFrm4x4 desPosition(CartesianSetParam.Goal());
    CartesianDesiredPos = GetCartesianPosRot(desPosition);
}

void mtsSimulinkController::SetHostIP(const mtsStdString &hostIP)
{
    SocketConnectionInfo.hostIP = hostIP;
}

void mtsSimulinkController::SetKinematicsPort(const mtsUShort &kinematicsPort)
{
    SocketConnectionInfo.kinematicsPort = kinematicsPort;
}

void mtsSimulinkController::SetTorquePort(const mtsUShort &torquePort)
{
    SocketConnectionInfo.torquePort = torquePort;
}

/******************Protected Functions: Simulink Communication******************/
void mtsSimulinkController::SendDataToSimulink()
{
    UpdateStateData();
    if(socketComm->IsSendSocketConnected() && simulinkIsReady) {
        struct timespec temp = startLoop;
        clock_gettime(CLOCK_REALTIME, &startLoop);

        if(logsEnabled) {
            SimulinkControllerLogFile << "BetweenSends = " << GetElapsedTime(temp, startLoop) << std::endl;
        }

        if(usingJointController) {
            dataPacket->SetPositionData(FeedbackPosition, DesiredPosition);
            dataPacket->SetVelocityData(FeedbackVelocity);
        } else {
            dataPacket->SetPositionData(CartesianCurrentPos, CartesianDesiredPos);
            dataPacket->SetVelocityData(CartesianVelocity);
        }

        struct timespec before, after;
        clock_gettime(CLOCK_REALTIME, &before);

        int socketCommSendReturn = socketComm->SendData(dataPacket);
        clock_gettime(CLOCK_REALTIME, &after);

        if(logsEnabled) {
            SimulinkControllerLogFile << "SendTime = " << GetElapsedTime(before, after) << std::endl;

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

            SimulinkControllerLogFile << "SentTime = " << (intmax_t)s << "." << zeros << ms << std::endl;
        }

        if(socketCommSendReturn == -1)
            CMN_LOG_RUN_WARNING << "Send returned -1" << std::endl;
        else if (socketCommSendReturn == 0)
            CMN_LOG_RUN_WARNING << "Send returned 0..." << std::endl;
        else {
            simulinkIsReady = false; //success, wait until response to send another packet
        }
    } else if(socketComm->IsSendSocketConnected() && !simulinkIsReady) {
        if(!disablePIDNextReceivedPacket) { //don't do this on first try
            clock_gettime(CLOCK_REALTIME, &watchDogTime);

            if(GetElapsedTime(startLoop, watchDogTime) > watchDogTimeOutMS) { //if waiting too long for Simulink to respond
                CMN_LOG_RUN_WARNING << "Simulink is taking too long to respond; watchdog timer of " << watchDogTimeOutMS << "ms has expired. Terminating communication" << std::endl;
                DoneCommunicationWithSimulink(true);
            }
        }
    } else { //!socketComm->IsSendSocketConnected()
        CMN_LOG_RUN_WARNING << "SEND IS CLOSED!! Shutting down!" << std::endl;

        if(Enabled)
            DoneCommunicationWithSimulink(true);
    }

    //regardless, clean up packet that was made
    delete dataPacket;
    dataPacket = 0;
}

void mtsSimulinkController::ReceiveFromSimulink()
{
    if(socketComm->IsRecvSocketConnected() && !simulinkIsReady) {

        int maxAttempts = 100;
        //try a couple times to receive
        for(int count = 1; count < maxAttempts; count++) {
            char * bufferTorque = new char[socketComm->GetBufferRecvSize()];

            struct timespec before, after;
            clock_gettime(CLOCK_REALTIME, &before);

            int bytesRead = socketComm->RecvData(bufferTorque, true);

            clock_gettime(CLOCK_REALTIME, &after);

            if(logsEnabled) {
                SimulinkControllerLogFile << "ReceiveTime = " << GetElapsedTime(before, after) << std::endl;
            }

            if(bytesRead > 0) { //got something good
                int ret = 0;
                vctDoubleVec torqueData = ParseMatlabServerData(&ret, bufferTorque, bytesRead);
                delete bufferTorque;

                if(ret == -1) { //it's done!
                    DoneCommunicationWithSimulink(true);
                    Torque.SetAll(0.0);
                }
                else {
                    if(disablePIDNextReceivedPacket)
                    {
                        DoneCommunicationWithSimulink(false); //disable PID when first transmission received
                        disablePIDNextReceivedPacket = false;
                    }

                    Torque.Assign(torqueData);

                    simulinkIsReady = true;
                    clock_gettime(CLOCK_REALTIME, &endLoop);

                    if(logsEnabled) {
                        SimulinkControllerLogFile << "LoopTime = " << GetElapsedTime(startLoop, endLoop) << std::endl
                                                  << "NumRecvAttempt = " << count << std::endl;
                    }
                }

                count = maxAttempts + 1;
            }
            else if (bytesRead == -1) {
                CMN_LOG_RUN_WARNING << "Error reading" << std::endl;
                count = maxAttempts + 1;
            } else {
                //CMN_LOG_RUN_WARNING << "Didn't read anything" << std::endl;
            }
        }
    } else if (socketComm->IsRecvSocketConnected() && !simulinkIsReady) {
        CMN_LOG_RUN_WARNING << "Waiting on Simulink" << std::endl;
    } else { //!socketComm->IsRecvSocketConnected()
        CMN_LOG_RUN_WARNING << "RECEIVE IS CLOSED!! Shutting down!" << std::endl;

        if(Enabled)
            DoneCommunicationWithSimulink(true);
    }
}

/*
 * Sets exitVal = -2 if error
 * Sets exitVal = -1 if Simulink finished
 * Sets exitVal =  0 if received joint torque values
 * Sets exitVal =  1 if received force-torque values
 */
vctDoubleVec mtsSimulinkController::ParseMatlabServerData(int * exitVal, char * bufferTorque, int bytesRead)
{
    vctDoubleVec jointTorque(NumberOfJoints, 0.0);
    *exitVal       = -2;

    if(usingJointController) {
        double j1,j2,j3,j4,j5,j6,j7;
        int ret = sscanf(bufferTorque, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &j1, &j2, &j3, &j4, &j5, &j6, &j7);

        if(ret != 7)
            CMN_LOG_RUN_WARNING << "Incorrect data packet format; missing data. Ret = " << ret << "Numbytes = " << bytesRead << ", Got " << bufferTorque << std::endl;
        else {
            if(NumberOfJoints == 7) //PSM
                jointTorque.Assign(j1,j2,j3,j4,j5,j6,j7);
            else if(NumberOfJoints == 8) //MTM
                jointTorque.Assign(j1,j2,j3,j4,j5,j6,j7, 0.0);
            else
                CMN_LOG_RUN_WARNING << "Incorrect arm in use!" << std::endl;

            if(j1 == -1 && j2 == -1 && j3 == -1 && j4 == -1 && j5 == -1 && j6 == -1 && j7 == -1)
                *exitVal = -1;
            else
                *exitVal = 0;
        }
    } else {
        double f1,f2,f3,t1,t2,t3;
        int ret = sscanf(bufferTorque, "%lf,%lf,%lf,%lf,%lf,%lf", &f1, &f2, &f3, &t1, &t2, &t3);

        if(ret != 6)
            CMN_LOG_RUN_WARNING << "Incorrect data packet format; missing data. Ret = " << ret << "Numbytes = " << bytesRead << ", Got " << bufferTorque << std::endl;
        else {
            ForceTorque.Assign(f1,f2,f3,t1,t2,t3);

            if(f1 == -1 && f2 == -1 && f3 == -1 && t1 == -1 && t2 == -1 && t3 == -1)
                *exitVal = -1;
            else
                *exitVal = 1;

            for (unsigned int i = 0; i < ForceTorque.size(); i++)
                jointTorque.at(i) = ForceTorque.at(i);

            //Calculate joint-level torques
            RobotArm.GetJointTorqueFromForceTorques(jointTorque); //jointTorque gets modified in PSM.cpp to torque values
        }
    }

    return jointTorque;
}

//Send signal up to top level so then shutdown can be propagate through all levels of control properly
void mtsSimulinkController::DoneCommunicationWithSimulink(bool done)
{
    if(done)
    {
        Enabled = false; //stop anything else from running (could take a ms or two for the top level to filter down, receive might still work then)

        simulinkIsReady              = false;
        disablePIDNextReceivedPacket = false;
    }

    TopLevelWidgetSimulink.SignalSimulinkDone(done);
}

/******************Protected Functions: Logging******************/
void mtsSimulinkController::EnableLogs(const mtsBool &enable)
{
    logsEnabled = enable;
    if(logsEnabled)
    {
        //CMN_LOG_RUN_WARNING << "mtsSimulinkController: logs enabled" << std::endl;

        // UBC open output file
        if(!SimulinkControllerLogFile.is_open())
        {
            std::string fname = taskName;
            fname.append("mtsSimulinkController.log");
            CheckFileExists(&fname);

            SimulinkControllerLogFile.open(fname.c_str(), std::ofstream::out | std::ofstream::app);
            if(!SimulinkControllerLogFile.is_open())
                CMN_LOG_CLASS_RUN_WARNING << "Failed to open the mtsSimulinkController log file!" << std::endl;

        }

        SimulinkControllerLogFile << "*********Log number:   " << logEntryIndex << std::endl;
    }
    else {
        CloseLogs(false);
        //CMN_LOG_RUN_WARNING << "mtsSimulinkController: logs disabled" << std::endl;
    }
}

void mtsSimulinkController::CloseLogs(bool closingTime)
{
    if(closingTime) {
        if(logsEnabled)
        {
            SimulinkControllerLogFile << std::endl << "*********************************END OF TEST, APPLICATION CLOSED" << std::endl;
            SimulinkControllerLogFile.close();
        }
    } else {
        SimulinkControllerLogFile << "*********end entry " << logEntryIndex << std::endl << std::endl;
        logEntryIndex++;
    }
}

void mtsSimulinkController::CheckFileExists(std::string *fname)
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

void mtsSimulinkController::LogEntry(bool all)
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

    if(all)
    {
        if(usingJointController)
        {
            SimulinkControllerLogFile << "t1 = " << (intmax_t)s << "." << zeros << ms << std::endl
                       << "Desired Pos = "  << std::setprecision(5)  << DesiredPosition  << std::endl
                       << "Current Pos = "  << std::setprecision(5)  << FeedbackPosition << std::endl
                       << "Feedback Vel = " << std::setprecision(5)  << FeedbackVelocity << std::endl
                       << "Torque1 = "      << std::setprecision(5)  << Torque           << std::endl << std::endl;
        }
        else
        {
            SimulinkControllerLogFile << "t1 = " << (intmax_t)s << "." << zeros << ms << std::endl
                       << "Desired Pos  = "  << std::setprecision(5)  << CartesianDesiredPos << std::endl
                       << "Current Pos  = "  << std::setprecision(5)  << CartesianCurrentPos << std::endl
                       << "Feedback Vel = "  << std::setprecision(5)  << CartesianVelocity   << std::endl
                       << "ForceTorque  = "  << std::setprecision(5)  << ForceTorque         << std::endl
                       << "Torque1 = "       << std::setprecision(5)  << Torque              << std::endl << std::endl;
        }
    }
    else
        SimulinkControllerLogFile << "t2 = " << (intmax_t)s << "." << zeros << ms << std::endl
                   << "Torque2 = "       << std::setprecision(5)  << Torque           << std::endl << std::endl;
}

/******************Protected Functions: Helpers******************/
void mtsSimulinkController::UpdateStateData()
{
    // update position, insertion is in mm, all others in radians!!!
    mtsExecutionResult executionResult = Robot.GetFeedbackPosition(FeedbackPositionParam);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_WARNING << "UpdateStateData: Call to mtsSimulinkController::Robot.GetFeedbackPosition failed \""
                                  << executionResult << "\"" << std::endl;
    }
    FeedbackPositionParam.GetPosition(FeedbackPosition);

    double dt = -1;
    // update velocities
    if (Robot.GetFeedbackVelocity.IsValid()) {
        Robot.GetFeedbackVelocity(FeedbackVelocityParam);
        FeedbackVelocityParam.GetVelocity(FeedbackVelocity);
    }
    else {
        dt = FeedbackPositionParam.Timestamp() - FeedbackPositionPreviousParam.Timestamp();

        if (dt > 0) {
            FeedbackVelocity.DifferenceOf(FeedbackPositionParam.Position(),
                                          FeedbackPositionPreviousParam.Position());
            FeedbackVelocity.Divide(dt);
        } else { //== 0
            //FeedbackVelocity.SetAll(0.0);
            FeedbackVelocity.Assign(FeedbackVelocityPrev);
        }

        // set param so data in state table is accurate
        FeedbackVelocityParam.SetVelocity(FeedbackVelocity);
    }
    if(!usingJointController) {
        RobotArm.GetPositionCartesian(CartesianGetParam); //update current position (only gets updated every 3ms)
        vctFrm4x4 curPosition(CartesianGetParam.Position());
        CartesianCurrentPos = GetCartesianPosRot(curPosition);

        //Calculate Cartesian Velocity
        //Calculation of cartesian velocity will be done in PSM class (couldn't pass Jacobian easily)
        vctDoubleVec velocityDataForPSM, cartesianVelocity;
        velocityDataForPSM.SetSize(NumberOfJoints + 3);
        velocityDataForPSM.SetAll(0.0);
        cartesianVelocity.SetSize(6); //first 3 = linear velocities; last 3 = angular velocities

        for (unsigned int i = 0; i < NumberOfJoints; i++)
            velocityDataForPSM.at(i) = FeedbackVelocity.at(i); //rest are 0s (pos_t0, not used here)

        RobotArm.GetRobotCartVelFromJacobian(velocityDataForPSM);
        for (unsigned int i = 0; i < cartesianVelocity.size(); i++)
            cartesianVelocity(i) = velocityDataForPSM.at(i);

        CartesianVelocity.Assign(cartesianVelocity);
    }

    // Prepare Data packet
    dataPacket = new prmSimulinkDataPacket(dt, FeedbackPositionParam.Timestamp());

    // save previous position
    FeedbackPositionPreviousParam = FeedbackPositionParam;
    FeedbackVelocityPrev          = FeedbackVelocity;
}

vctDoubleVec mtsSimulinkController::GetCartesianPosRot(vctFrm4x4 position)
{
    vctDoubleVec posRot(12,0.0);
    for(unsigned int i = 0; i < 3; i++)
        posRot.at(i) = position.Translation().at(i)*cmn_m; //convert to mm

    vctMatRot3  rotTemp;
    rotTemp.Assign(vctMatRot3::Identity() * position.Rotation());

    unsigned int index = 3;
    for(unsigned int j = 0; j < 3; j++) {
        for(unsigned int k  = 0; k < 3; k++) {
            posRot.at(index) = rotTemp[j][k];
            index++;
        }
    }

    return posRot;
}

double mtsSimulinkController::GetElapsedTime(timespec time0, timespec time1)
{
    return (time1.tv_sec  - time0.tv_sec) * 1000.0 +
           (time1.tv_nsec - time0.tv_nsec) / 1.0e6;
}
