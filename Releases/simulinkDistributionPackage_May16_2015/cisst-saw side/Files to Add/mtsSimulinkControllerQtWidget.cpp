/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Angelica Ruszkowski
  Created on: 2015-04-06

  Based off mtsSimulinkControllerQtWidget.cpp

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>
#include <unistd.h>
#include <time.h>

// Qt include
#include <QMessageBox>
#include <QGridLayout>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QCloseEvent>
#include <QCoreApplication>

//to check robot state
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmJointType.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstCommon/cmnUnits.h>

#include <sawControllers/mtsSimulinkControllerQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsSimulinkControllerQtWidget, mtsComponent, mtsComponentConstructorNameAndUInt)

/******************Public Functions******************/
mtsSimulinkControllerQtWidget::mtsSimulinkControllerQtWidget(const std::string & componentName,
                               unsigned int numberOfAxis, double periodInSeconds,
                               int kinPortNum, int torquePortNum):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000), // Qt timer are in milliseconds
    NumberOfJoints(numberOfAxis)
{
    Init();
    component_name = componentName;

    std::ostringstream ssk, sst;
    ssk << kinPortNum;
    LEkinematicPort->setText((ssk.str()).c_str());

    sst << torquePortNum;
    LEtorquePort->setText((sst.str()).c_str());
}

mtsSimulinkControllerQtWidget::mtsSimulinkControllerQtWidget(const mtsComponentConstructorNameAndUInt & arg):
    mtsComponent(arg.Name),
    TimerPeriodInMilliseconds(50),
    NumberOfJoints(arg.Arg)
{
    Init();
}

void mtsSimulinkControllerQtWidget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsSimulinkControllerQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsSimulinkControllerQtWidget::Startup" << std::endl;

    mtsExecutionResult result;
    prmJointTypeVec jointType;
    result = PID.GetJointType(jointType);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                 << result << std::endl;
        UnitFactor.SetAll(0.0);
    } else {
        // set unitFactor;
        for (size_t i = 0; i < this->NumberOfJoints; i++) {
            if (jointType[i] == PRM_REVOLUTE) {
                UnitFactor[i] = cmn180_PI;
            } else if (jointType[i] == PRM_PRISMATIC) {
                UnitFactor[i] = cmn_mm;
            } else {
                cmnThrow("mtsRobotIO1394QtWidget: Unknown joint type");
            }
        }
    }

    // Show the GUI
    if (!parent()) {
        show();
    }
}

void mtsSimulinkControllerQtWidget::Cleanup(void)
{
    SimulinkController.Enable(false);
    //UBC log file for PID joint efforts
    closeLogs(true);
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsSimulinkControllerQtWidget::Cleanup" << std::endl;
}

/******************Protected Functions******************/
void mtsSimulinkControllerQtWidget::Init()
{
    SimulinkActuallyInControl = false;

    SimulinkController.PositionJointGetParam.Position().SetSize(NumberOfJoints);
    SimulinkController.PositionJointGetDesired.SetSize(NumberOfJoints);
    SimulinkController.VelocityJointGetParam.Velocity().SetSize(NumberOfJoints);
    SimulinkController.EffortJoint.SetSize(NumberOfJoints);

    DesiredPositionParam.SetSize(NumberOfJoints);
    DesiredPosition.SetSize(NumberOfJoints);
    DesiredPosition.SetAll(0.0);
    UnitFactor.SetSize(NumberOfJoints);
    UnitFactor.SetAll(1.0);

    PlotIndex = 0;

    logsEnabled = false;
    logEntryIndex = 0;

    // Setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("PIDController");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetJointType", PID.GetJointType);
        interfaceRequired->AddFunction("GetPositionJointDesired", PID.GetPositionJointDesired);

    }

    mtsInterfaceRequired * robotArmInterfaceRequired = AddInterfaceRequired("RobotArmSimGUI");
    if (robotArmInterfaceRequired) {
        robotArmInterfaceRequired->AddFunction("GetPositionCartesian", RobotArm.GetPositionCartesian);
        robotArmInterfaceRequired->AddFunction("GetRobotControlState", RobotArm.GetRobotControlState);
    }

    mtsInterfaceRequired * simulinkInterfaceRequired = AddInterfaceRequired("SimulinkControllerPIDGUI");
    if(simulinkInterfaceRequired) {
        simulinkInterfaceRequired->AddFunction("Enable",                    SimulinkController.Enable);
        simulinkInterfaceRequired->AddFunction("EnableLogs",                SimulinkController.EnableLogs);
        simulinkInterfaceRequired->AddFunction("SetHostIP",                 SimulinkController.SetHostIP);
        simulinkInterfaceRequired->AddFunction("SetKinematicsPortNum",      SimulinkController.SetKinematicsPortNum);
        simulinkInterfaceRequired->AddFunction("SetTorquePortNum",          SimulinkController.SetTorquePortNum);
        simulinkInterfaceRequired->AddFunction("SetPositionJoint",          SimulinkController.SetPositionJoint);
        simulinkInterfaceRequired->AddFunction("SetPositionCartDes",        SimulinkController.SetPositionCartDesired);
        simulinkInterfaceRequired->AddFunction("SetSimControllerType",      SimulinkController.SetControllerTypeToJoint);
        simulinkInterfaceRequired->AddFunction("GetSimControllerType",      SimulinkController.GetControllerTypeIsJoint);

        simulinkInterfaceRequired->AddFunction("GetSimPositionJointDesired",SimulinkController.GetPositionJointDesired);
        simulinkInterfaceRequired->AddFunction("GetSimEffortJointDesired",  SimulinkController.GetEffortJointDesired);
        simulinkInterfaceRequired->AddFunction("GetSimPositionJoint",       SimulinkController.GetPositionJoint);
        simulinkInterfaceRequired->AddFunction("GetSimVelocityJoint",       SimulinkController.GetVelocityJoint);

        simulinkInterfaceRequired->AddFunction("GetPeriodStatistics",       SimulinkController.GetPeriodStatistics);
    }

    // provide SetDesiredPositions
    mtsInterfaceProvided * simulinkInterfaceProvided = AddInterfaceProvided("SignalSimulinkDoneHighLevel");
    if (simulinkInterfaceProvided) {
        simulinkInterfaceProvided->AddCommandWrite(&mtsSimulinkControllerQtWidget::ChangeSimulinkStatusFromBelow, this,  "SignalSimulinkDone", mtsBool());
    }

    mtsInterfaceProvided * pidQtInterfaceProvided = AddInterfaceProvided("PidQtInterfacePIDCommand");
    if (pidQtInterfaceProvided) {
        pidQtInterfaceProvided->AddCommandWrite(&mtsSimulinkControllerQtWidget::EnableGUIFromPIDQt ,      this,  "EnableSimulinkWidgetFromPID", mtsBool());
        pidQtInterfaceProvided->AddCommandWrite(&mtsSimulinkControllerQtWidget::EnableSimulinkFromPIDQt , this,  "EnableSimulinkFromPID",       mtsBool());
        pidQtInterfaceProvided->AddCommandWrite(&mtsSimulinkControllerQtWidget::EnableLogsFromPIDQt ,     this,  "EnableSimulinkLogsFromPID",   mtsBool());
        pidQtInterfaceProvided->AddCommandRead(&mtsSimulinkControllerQtWidget::GetSimulinkInControl ,     this,  "IsSimulinkEnabled");
    }

    mtsInterfaceRequired * pidQtInterfaceRequired = AddInterfaceRequired("PidQtInterfaceSimulinkCommand");
    if (pidQtInterfaceRequired) {
        pidQtInterfaceRequired->AddFunction("EnablePIDFromSimulink",      PIDQtWidget.Enable);
        pidQtInterfaceRequired->AddFunction("EnablePIDLogsFromSimulink",  PIDQtWidget.EnableLogs);


    }

    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms

    EnableGUIFromPIDQt(false);
}

void mtsSimulinkControllerQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsSimulinkControllerQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

/******************Slot Functions******************/
void mtsSimulinkControllerQtWidget::SlotEnableSimulink(bool toggle)
{
    //Configure simulink TCP/IP Communication
    if(toggle) {
        SimulinkController.SetHostIP(mtsStdString((LEhostIP->text()).toStdString()));
        SimulinkController.SetKinematicsPortNum(mtsUShort((LEkinematicPort->text()).toUShort()));
        SimulinkController.SetTorquePortNum(mtsUShort((LEtorquePort->text()).toUShort()));
    }

    SimulinkActuallyInControl = false; //only in control once get signal from ChangeSimulinkStatusFromBelow

    if(!toggle) //user turned Simulink off from this widget, turn PID back on! Only turn PID off once receive signal from Below (ChangeSimulinkStatusFromBelow)
    {
        PIDQtWidget.Enable(!toggle); //enable PID
        CMN_LOG_RUN_WARNING << "PID controller being turned " << (!toggle ? "ON" : "OFF") << std::endl;
    }

    //Update desired information
    mtsExecutionResult executionResult;
    if(toggle) //Assign initial desired position to be current (ok, most recent set) position
    {
        SimulinkController.GetControllerTypeIsJoint(SimulinkController.UsingJointControl);

        if(SimulinkController.UsingJointControl) {
            executionResult = PID.GetPositionJointDesired(DesiredPosition);
            DesiredPositionParam.Goal().Assign(DesiredPosition);
            SimulinkController.SetPositionJoint(DesiredPositionParam);
        } else { //using Cartesian Control
            //do sanity check that in cartesian mode
            std::string state;
            RobotArm.GetRobotControlState(state);
            if(mtsIntuitiveResearchKitArmTypes::RobotStateTypeFromString(state) != mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN)
                CMN_LOG_RUN_WARNING << "WARNING!! Robot not in Cartesian mode! About to run Cartesian controller!" << std::endl;

            RobotArm.GetPositionCartesian(RobotArm.PositionCartesianCurrent);
            prmPositionCartesianSet positionCartCur;
            positionCartCur.Goal().Assign(RobotArm.PositionCartesianCurrent.Position());
            SimulinkController.SetPositionCartDesired(positionCartCur);
        }
    }

    //Actually enable
    CMN_LOG_RUN_WARNING << "Simulink controller being turned " << (toggle ? "ON" : "OFF") << std::endl;
    SimulinkController.Enable(toggle);

    //Update GUI
    QVWDesiredPositionWidget->setEnabled(toggle);
    QVRCurrentEffortWidget->setEnabled(toggle);
}

void mtsSimulinkControllerQtWidget::SlotSetSimulinkJointControl(bool toggle)
{
    SimulinkController.SetControllerTypeToJoint(toggle);
}

void mtsSimulinkControllerQtWidget::ChangeSimulinkStatusFromBelow(const mtsBool & socketsDone)
{
    SimulinkActuallyInControl = !socketsDone; //only case in which Simulink is actually doing control already; it has commenced

    PIDQtWidget.Enable(socketsDone); //enable PID

    if(socketsDone) { //can only turn Simulink off from here
        CMN_LOG_RUN_WARNING << "Simulink controller being turned " << (!socketsDone ? "ON" : "OFF") << std::endl;
        SimulinkController.Enable(!socketsDone);
    }

    QCBEnableSimulink->setChecked(SimulinkActuallyInControl); //will only ever be turned off
    QVWDesiredPositionWidget->setEnabled(SimulinkActuallyInControl);
    QVRCurrentEffortWidget->setEnabled(SimulinkActuallyInControl);
}

// UBC slot to enable printing PID joint effort data to LOG files
void mtsSimulinkControllerQtWidget::SlotEnableLOG(bool toggle)
{
    EnableLog(true,toggle);
}

void mtsSimulinkControllerQtWidget::SlotPositionChanged(void)
{
    DesiredPosition.SetAll(0.0);
    QVWDesiredPositionWidget->GetValue(DesiredPosition);
    DesiredPositionParam.SetGoal(DesiredPosition);
    DesiredPositionParam.Goal().ElementwiseDivide(UnitFactor);

    if(QCBEnableSimulink->isChecked() && QCBSimControlType->isChecked()) //joint level control
        SimulinkController.SetPositionJoint(DesiredPositionParam);
}

void mtsSimulinkControllerQtWidget::SlotPlotIndex(int newAxis)
{
    PlotIndex = newAxis;
    QVPlot->SetContinuousExpandYResetSlot();
}

void mtsSimulinkControllerQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // get data from the PID
    SimulinkController.GetPeriodStatistics(IntervalStatistics);

    SimulinkController.GetPositionJoint(SimulinkController.PositionJointGetParam);
    SimulinkController.PositionJointGetParam.Position().ElementwiseMultiply(UnitFactor);
    SimulinkController.GetPositionJointDesired(SimulinkController.PositionJointGetDesired);
    SimulinkController.PositionJointGetDesired.ElementwiseMultiply(UnitFactor);
    SimulinkController.GetVelocityJoint(SimulinkController.VelocityJointGetParam);
    SimulinkController.VelocityJointGetParam.Velocity().ElementwiseMultiply(UnitFactor);
    SimulinkController.GetEffortJointDesired(SimulinkController.EffortJoint);
    //CMN_LOG_RUN_WARNING << "SimQt: effortJoint = " << SimulinkRobot.EffortJoint << std::endl;

    // update GUI
    QMIntervalStatistics->SetValue(IntervalStatistics);

    QVRCurrentPositionWidget->SetValue(SimulinkController.PositionJointGetParam.Position());
    QVWDesiredPositionWidget->SetValue(SimulinkController.PositionJointGetDesired);
    QVRCurrentEffortWidget->SetValue(SimulinkController.EffortJoint);

    // plot
    CurrentPositionSignal->AppendPoint(vctDouble2(SimulinkController.PositionJointGetParam.Timestamp(),
                                                  SimulinkController.PositionJointGetParam.Position().Element(PlotIndex)));
    DesiredPositionSignal->AppendPoint(vctDouble2(SimulinkController.PositionJointGetParam.Timestamp(),
                                                  SimulinkController.PositionJointGetDesired.Element(PlotIndex)));
    CurrentVelocitySignal->AppendPoint(vctDouble2(SimulinkController.VelocityJointGetParam.Timestamp(),
                                                  SimulinkController.VelocityJointGetParam.Velocity().Element(PlotIndex)));
    DesiredEffortSignal->AppendPoint(vctDouble2(SimulinkController.PositionJointGetParam.Timestamp(),
                                                -SimulinkController.EffortJoint.Element(PlotIndex))); // negate current to plot the same direction

    // UBC log file for simulink joint efforts
    if(simulinkLogFile.is_open())
    {
        struct timespec spec;
        clock_gettime(CLOCK_REALTIME, &spec);
        time_t s  = spec.tv_sec;
        long ms = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
        if(ms >= 1000) // this is the grossest conversion ever!!
        {
            s = s+1;
            ms = ms-1000;
        }
        // pretty gross hack, but didn't want to get into converting long to string then padding it
        std::string zeros = "";
        if(ms < 10) {
            zeros = "00";
        } else if(ms < 100) {
            zeros = "0";
        }

        simulinkLogFile << "Simulink effort: t = " << (intmax_t)s << "." << zeros << ms << std::endl;
        simulinkLogFile << "effort: " << SimulinkController.EffortJoint.Element(0) << " ";
        simulinkLogFile << SimulinkController.EffortJoint.Element(1) << " ";
        simulinkLogFile << SimulinkController.EffortJoint.Element(2) << " ";
        simulinkLogFile << SimulinkController.EffortJoint.Element(3) << " ";
        simulinkLogFile << SimulinkController.EffortJoint.Element(4) << " ";
        simulinkLogFile << SimulinkController.EffortJoint.Element(5) << " ";
        simulinkLogFile << SimulinkController.EffortJoint.Element(6) << std::endl;
    }

    QVPlot->updateGL();
}

/******************Private Functions******************/
void mtsSimulinkControllerQtWidget::setupUi(void)
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    QGridLayout * gridLayout = new QGridLayout();

    int row = 0;
    QLabel * currentPosLabel = new QLabel("Current position (deg)");
    currentPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(currentPosLabel, row, 0);
    QVRCurrentPositionWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVRCurrentPositionWidget->SetPrecision(3);
    gridLayout->addWidget(QVRCurrentPositionWidget, row, 1);
    row++;

    QLabel * desiredPosLabel = new QLabel("Desired position (deg)");
    desiredPosLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(desiredPosLabel, row, 0);
    QVWDesiredPositionWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    // DesiredPositionWidget->SetDecimals(2);
    QVWDesiredPositionWidget->SetStep(1); //0.1);
    QVWDesiredPositionWidget->SetRange(-360.0, 360.0);
    QVWDesiredPositionWidget->setEnabled(false);
    gridLayout->addWidget(QVWDesiredPositionWidget, row, 1);
    row++;

    QLabel * currentEffortLabel = new QLabel("Current effort (Nm)");
    currentEffortLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(currentEffortLabel, row, 0);
    QVRCurrentEffortWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVRCurrentEffortWidget->SetPrecision(5);
    QVRCurrentEffortWidget->setEnabled(false);
    gridLayout->addWidget(QVRCurrentEffortWidget, row, 1);
    row++;

    // plot
    QHBoxLayout * plotLayout = new QHBoxLayout;
    // plot control
    QVBoxLayout * plotButtonsLayout = new QVBoxLayout;
    // - pick axis to display
    QLabel * plotIndexLabel = new QLabel("Index");
    plotButtonsLayout->addWidget(plotIndexLabel);
    QSBPlotIndex = new QSpinBox();
    QSBPlotIndex->setRange(0, NumberOfJoints);
    plotButtonsLayout->addWidget(QSBPlotIndex);
    plotButtonsLayout->addStretch();
    plotLayout->addLayout(plotButtonsLayout);
    // plotting area
    QVPlot = new vctPlot2DOpenGLQtWidget();
    vctPlot2DBase::Scale * scalePosition = QVPlot->AddScale("positions");
    CurrentPositionSignal = scalePosition->AddSignal("current");
    CurrentPositionSignal->SetColor(vctDouble3(1.0, 0.0, 0.0));
    vctPlot2DBase::Scale * scalePosition2 = QVPlot->AddScale("positions2");
    DesiredPositionSignal = scalePosition2->AddSignal("desired");
    DesiredPositionSignal->SetColor(vctDouble3(0.0, 1.0, 0.0));
    vctPlot2DBase::Scale * scaleVelocity = QVPlot->AddScale("velocities");
    CurrentVelocitySignal = scaleVelocity->AddSignal("current");
    CurrentVelocitySignal->SetColor(vctDouble3(0.0, 0.75, 1.0));
    vctPlot2DBase::Scale * scaleEffort = QVPlot->AddScale("efforts");
    DesiredEffortSignal = scaleEffort->AddSignal("-desired");
    DesiredEffortSignal->SetColor(vctDouble3(1.0, 1.0, 1.0));
    QVPlot->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    plotLayout->addWidget(QVPlot);

    // control
    QCBEnableSimulink = new QCheckBox("Enable Simulink Control");
    QCBSimControlType = new QCheckBox("Simulink Joint Control");
    QCBSimControlType->setChecked(true);
    QCBEnableLOG = new QCheckBox("Enable LOG"); // UBC for enable LOG button
    QHBoxLayout * testLayout = new QHBoxLayout;
    testLayout->addWidget(QCBEnableSimulink);
    testLayout->addWidget(QCBSimControlType);
    testLayout->addWidget(QCBEnableLOG); // UBC for enable LOG button
    testLayout->addStretch();
    QGroupBox * testGroupBox = new QGroupBox("Control");
    testGroupBox->setLayout(testLayout);

    //simulink configuration
    QLabel * hostIpLabel = new QLabel(tr("Host IP:"));
    LEhostIP = new QLineEdit;
    LEhostIP->setInputMask("000.000.000.000");
    //LEhostIP->setInputMask("000.000.000.000;_");
    LEhostIP->setCursorPosition(0);
    LEhostIP->setText("137.82.56.245");

    QLabel * kinematicPortLabel = new QLabel(tr("Kinematic port:"));
    LEkinematicPort = new QLineEdit;
    LEkinematicPort->setValidator(new QIntValidator(0, 99999, LEkinematicPort));
    LEkinematicPort->setText("12345");

    QLabel * torquePortLabel = new QLabel(tr("Torque port:"));
    LEtorquePort = new QLineEdit;
    LEtorquePort->setValidator(new QIntValidator(0, 99999, LEtorquePort));
    LEtorquePort->setText("54321");

    QHBoxLayout * configurationLayout = new QHBoxLayout;
    configurationLayout->addWidget(hostIpLabel);
    configurationLayout->addWidget(LEhostIP);
    configurationLayout->addWidget(kinematicPortLabel);
    configurationLayout->addWidget(LEkinematicPort);
    configurationLayout->addWidget(torquePortLabel);
    configurationLayout->addWidget(LEtorquePort);
    configurationLayout->addStretch();
    QGroupBox * configurationGroupBox = new QGroupBox(tr("Simulink Configuration"));
    configurationGroupBox->setLayout(configurationLayout);

    // Timing
    QVBoxLayout * timingLayout = new QVBoxLayout;
    QFrame * timingFrame = new QFrame;
    QLabel * timingTitle = new QLabel("Timing");
    timingTitle->setFont(font);
    timingTitle->setAlignment(Qt::AlignCenter);
    timingLayout->addWidget(timingTitle);
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    timingLayout->addWidget(QMIntervalStatistics);
    timingLayout->addStretch();
    timingFrame->setLayout(timingLayout);
    timingFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

    connect(QCBEnableSimulink, SIGNAL(clicked(bool)),      this, SLOT(SlotEnableSimulink(bool)));
    connect(QCBSimControlType, SIGNAL(clicked(bool)),      this, SLOT(SlotSetSimulinkJointControl(bool)));
    connect(QCBEnableLOG,      SIGNAL(clicked(bool)),      this, SLOT(SlotEnableLOG(bool))); // UBC for enable LOG button
    connect(QSBPlotIndex,      SIGNAL(valueChanged(int)),  this, SLOT(SlotPlotIndex(int)));

    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(gridLayout);
    mainLayout->addLayout(plotLayout);
    mainLayout->addWidget(testGroupBox);
    mainLayout->addWidget(configurationGroupBox);
    mainLayout->addWidget(timingFrame);

    setLayout(mainLayout);

    setWindowTitle(this->GetName().c_str());
    setMinimumWidth(750);
    resize(sizeHint());

    // connect signals & slots
    connect(QVWDesiredPositionWidget, SIGNAL(valueChanged()), this, SLOT(SlotPositionChanged()));
}

void mtsSimulinkControllerQtWidget::EnableLog(bool localButtonUsed, bool enable)
{
    logsEnabled = enable;
    if(logsEnabled)
    {
        //CMN_LOG_RUN_WARNING << "mtsSimulinkControllerQtWidget: logs enabled" << std::endl;

        // UBC open output file
        if(!simulinkLogFile.is_open())
        {
            std::string fname = component_name;
            fname.append("SimulinkWidget.log");
            checkFileExists(&fname);
            simulinkLogFile.open(fname.c_str(), std::ofstream::out | std::ofstream::app);
            if(!simulinkLogFile.is_open())
                CMN_LOG_CLASS_RUN_ERROR << "Failed to open the log file!" << std::endl;
        }

        if(simulinkLogFile.is_open())
            simulinkLogFile << "*********Log number:   " << logEntryIndex << std::endl;
    }
    else {
        closeLogs(false);
        //CMN_LOG_RUN_WARNING << "mtsSimulinkControllerQtWidget: logs disabled" << std::endl;
    }
    SimulinkController.EnableLogs(logsEnabled);

    if(localButtonUsed) {
        PIDQtWidget.EnableLogs(logsEnabled);
    } else {
        QCBEnableLOG->setChecked(enable);
    }
}

/******************Interface Functions******************/
void mtsSimulinkControllerQtWidget::EnableSimulinkFromPIDQt(const mtsBool &enable)
{
    //will only ever be sent a false
    if(enable)
        CMN_LOG_RUN_WARNING << "Something's wrong! PID should not be commanded Simulink controller to start!" << std::endl;
    else {
        CMN_LOG_RUN_WARNING << "Simulink controller being turned " << (enable ? "ON" : "OFF") << std::endl;
        SimulinkController.Enable(enable);

        SimulinkActuallyInControl = enable;

        QCBEnableSimulink->setChecked(enable); //will only ever be turned off
        QVWDesiredPositionWidget->setEnabled(enable);
        QVRCurrentEffortWidget->setEnabled(enable);
    }
}

void mtsSimulinkControllerQtWidget::EnableLogsFromPIDQt(const mtsBool &enable)
{
    EnableLog(false, enable);
}

void mtsSimulinkControllerQtWidget::EnableGUIFromPIDQt(const mtsBool &enable)
{
    QCBEnableSimulink->setEnabled(enable);
    QCBSimControlType->setEnabled(enable);
}

void mtsSimulinkControllerQtWidget::GetSimulinkInControl(mtsBool &inControl) const
{
    inControl = SimulinkActuallyInControl;
}

/******************Logging******************/
void mtsSimulinkControllerQtWidget::closeLogs(bool appClosing)
{
    if(simulinkLogFile.is_open())
        simulinkLogFile << "*********end entry " << logEntryIndex << std::endl << std::endl;

    if(appClosing)
        simulinkLogFile.close();

    logEntryIndex++;
}

void mtsSimulinkControllerQtWidget::checkFileExists(std::string * fname)
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
