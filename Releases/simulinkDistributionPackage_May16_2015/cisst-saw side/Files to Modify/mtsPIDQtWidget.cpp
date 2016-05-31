/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

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

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmJointType.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstCommon/cmnUnits.h>

#include <sawControllers/mtsPIDQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsPIDQtWidget, mtsComponent, mtsComponentConstructorNameAndUInt)

mtsPIDQtWidget::mtsPIDQtWidget(const std::string & componentName, unsigned int numberOfAxis,
                               double periodInSeconds, const bool usingSimulinkControl):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000), // Qt timer are in milliseconds
    NumberOfAxis(numberOfAxis)
{
    usingSimulink = usingSimulinkControl;
    Init();
    component_name = componentName;
}

mtsPIDQtWidget::mtsPIDQtWidget(const mtsComponentConstructorNameAndUInt & arg):
    mtsComponent(arg.Name),
    TimerPeriodInMilliseconds(50),
    NumberOfAxis(arg.Arg)
{
    Init();
}

void mtsPIDQtWidget::Init()
{
    PID.PositionJointGetParam.Position().SetSize(NumberOfAxis);
    PID.PositionJointGetDesired.SetSize(NumberOfAxis);
    PID.VelocityJointGetParam.Velocity().SetSize(NumberOfAxis);
    PID.EffortJoint.SetSize(NumberOfAxis);

    DesiredPosition.SetSize(NumberOfAxis);
    DesiredPosition.SetAll(0.0);
    DesiredEffort.SetSize(NumberOfAxis);
    DesiredEffort.SetAll(0.0);
    UnitFactor.SetSize(NumberOfAxis);
    UnitFactor.SetAll(1.0);

    DirectControl = false;
    PlotIndex = 0;

    logsEnabled = false;
    logEntryIndex = 0;

    // Setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Controller");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("ResetController",           PID.ResetController);
        interfaceRequired->AddFunction("Enable",                    PID.Enable);
        interfaceRequired->AddFunction("EnableIO",                  PID.EnableIO);
        interfaceRequired->AddFunction("EnableLogs",                PID.EnableLogs);
        interfaceRequired->AddFunction("EnableTorqueMode",          PID.EnableTorqueMode);
        interfaceRequired->AddFunction("SetPositionJoint",          PID.SetPositionJoint);
        interfaceRequired->AddFunction("SetTorqueJoint",            PID.SetEffortJoint);
        interfaceRequired->AddFunction("GetPositionJoint",          PID.GetPositionJoint);
        interfaceRequired->AddFunction("GetVelocityJoint",          PID.GetVelocityJoint);
        interfaceRequired->AddFunction("GetPositionJointDesired",   PID.GetPositionJointDesired);
        interfaceRequired->AddFunction("GetEffortJointDesired",     PID.GetEffortJointDesired);
        interfaceRequired->AddFunction("GetPeriodStatistics",       PID.GetPeriodStatistics);
        interfaceRequired->AddFunction("GetJointType",              PID.GetJointType);
        interfaceRequired->AddFunction("GetPGain",                  PID.GetPGain);
        interfaceRequired->AddFunction("GetDGain",                  PID.GetDGain);
        interfaceRequired->AddFunction("GetIGain",                  PID.GetIGain);
        interfaceRequired->AddFunction("SetPGain",                  PID.SetPGain);
        interfaceRequired->AddFunction("SetDGain",                  PID.SetDGain);
        interfaceRequired->AddFunction("SetIGain",                  PID.SetIGain);
        // Events
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::JointLimitEventHandler, this, "JointLimit");
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::ErrorEventHandler, this, "Error");
        interfaceRequired->AddEventHandlerWrite(&mtsPIDQtWidget::EnableEventHandler, this, "Enabled");
    }

    if(usingSimulink) {
        mtsInterfaceRequired * simulinkQtInterfaceRequired = AddInterfaceRequired("SimulinkQtInterfacePIDCommand");
        if (simulinkQtInterfaceRequired) {
            simulinkQtInterfaceRequired->AddFunction("EnableSimulinkWidgetFromPID", SimulinkQtWidget.EnableWidget);
            simulinkQtInterfaceRequired->AddFunction("EnableSimulinkFromPID",       SimulinkQtWidget.Enable);
            simulinkQtInterfaceRequired->AddFunction("EnableSimulinkLogsFromPID",   SimulinkQtWidget.EnableLogs);
            //the below means COMMENCED, as in, is Simulink in control?
            simulinkQtInterfaceRequired->AddFunction("IsSimulinkEnabled",           SimulinkQtWidget.IsEnabled);
        }

        mtsInterfaceProvided * simulinkQtInterfaceProvided = AddInterfaceProvided("SimulinkQtInterfaceSimulinkCommand");
        if (simulinkQtInterfaceProvided) {
            simulinkQtInterfaceProvided->AddCommandWrite(&mtsPIDQtWidget::EnablePIDFromSimulinkQt , this,  "EnablePIDFromSimulink",     mtsBool());
            simulinkQtInterfaceProvided->AddCommandWrite(&mtsPIDQtWidget::EnableLogsFromSimulinkQt , this,  "EnablePIDLogsFromSimulink", mtsBool());
            }
    }

    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsPIDQtWidget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsPIDQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQtWidget::Startup" << std::endl;
    // Set desired pos to cur pos
    SlotResetPIDGain();
    SlotMaintainPosition();

    mtsExecutionResult result;
    prmJointTypeVec jointType;
    result = PID.GetJointType(jointType);
    if (!result) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: Robot interface isn't connected properly, unable to get joint type.  Function call returned: "
                                 << result << std::endl;
        UnitFactor.SetAll(0.0);
    } else {
        // set unitFactor;
        for (size_t i = 0; i < this->NumberOfAxis; i++) {
            if (jointType[i] == PRM_REVOLUTE) {
                UnitFactor[i] = cmn180_PI;
            } else if (jointType[i] == PRM_PRISMATIC) {
                UnitFactor[i] = 1.0 / cmn_mm;
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

void mtsPIDQtWidget::Cleanup(void)
{
    //UBC log file for PID joint efforts
    closeLogs(true);
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQtWidget::Cleanup" << std::endl;
}

//---------- Protected --------------------------
void mtsPIDQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsPIDQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsPIDQtWidget::SlotEnablePID(bool toggle)
{
    EnablePID(true, toggle);
}

void mtsPIDQtWidget::SlotEnableTorqueMode(bool toggle)
{
    vctBoolVec torqueMode(NumberOfAxis, toggle);
    PID.EnableTorqueMode(torqueMode);

    QVWDesiredEffortWidget->setEnabled(toggle);
    if(!toggle)
    {
        QVWDesiredEffortWidget->setStyleSheet(QString::fromUtf8("QPushButton:disabled"
                                                    "{ color: gray }"
                                                    ));
    } else {
        vctDoubleVec initTorque;
        initTorque.SetSize(NumberOfAxis);
        initTorque.SetAll(0.0);
        QVWDesiredEffortWidget->SetValue(initTorque);
    }
}

// UBC slot to enable printing PID joint effort data to LOG files
void mtsPIDQtWidget::SlotEnableLOG(bool toggle)
{
    EnableLog(true, toggle);
}

void mtsPIDQtWidget::checkFileExists(std::string * fname)
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

void mtsPIDQtWidget::closeLogs(bool appClosing)
{
    if(effortLogFile.is_open())
        effortLogFile << "*********end entry " << logEntryIndex << std::endl << std::endl;

    if(appClosing)
        effortLogFile.close();

    logEntryIndex++;
}

void mtsPIDQtWidget::SlotPositionChanged(void)
{
    DesiredPosition.SetAll(0.0);
    QVWDesiredPositionWidget->GetValue(DesiredPosition);
    DesiredPositionParam.SetGoal(DesiredPosition);
    DesiredPositionParam.Goal().ElementwiseDivide(UnitFactor);
    PID.SetPositionJoint(DesiredPositionParam);
}

void mtsPIDQtWidget::SlotEffortChanged(void)
{
    if(QCBEnableTorqueMode->isChecked()) //shouldn't be enabled otherwise
    {
        DesiredEffort.SetAll(0.0);
        QVWDesiredEffortWidget->GetValue(DesiredEffort);
        DesiredEffortParam.SetForceTorque(DesiredEffort);
        PID.SetEffortJoint(DesiredEffortParam);
    }
}

void mtsPIDQtWidget::SlotPGainChanged(void)
{
    vctDoubleVec pgain(NumberOfAxis, 0.0);
    QVWPGainWidget->GetValue(pgain);
    PID.SetPGain(pgain);
}

void mtsPIDQtWidget::SlotDGainChanged(void)
{
    vctDoubleVec dgain(NumberOfAxis, 0.0);
    QVWDGainWidget->GetValue(dgain);
    PID.SetDGain(dgain);
}

void mtsPIDQtWidget::SlotIGainChanged(void)
{
    vctDoubleVec igain(NumberOfAxis, 0.0);
    QVWIGainWidget->GetValue(igain);
    PID.SetIGain(igain);
}

void mtsPIDQtWidget::SlotMaintainPosition(void)
{
    // reset desired position
    prmPositionJointGet prmFeedbackPos;
    prmFeedbackPos.SetSize(NumberOfAxis);
    PID.GetPositionJoint(prmFeedbackPos);
    prmFeedbackPos.Position().ElementwiseMultiply(UnitFactor);
    QVWDesiredPositionWidget->SetValue(prmFeedbackPos.Position());
    PID.ResetController();
    SlotPositionChanged();
}

void mtsPIDQtWidget::SlotZeroPosition(void)
{
    // reset desired position
    DesiredPosition.SetAll(0.0);
    QVWDesiredPositionWidget->SetValue(DesiredPosition);
    PID.ResetController();
    SlotPositionChanged();
}

void mtsPIDQtWidget::SlotResetPIDGain(void)
{
    // get gains
    vctDoubleVec gain;
    gain.SetSize(NumberOfAxis);
    // PGain
    PID.GetPGain(gain);
    QVWPGainWidget->SetValue(gain);
    // DGain
    PID.GetDGain(gain);
    QVWDGainWidget->SetValue(gain);
    // IGain
    PID.GetIGain(gain);
    QVWIGainWidget->SetValue(gain);
}

void mtsPIDQtWidget::SlotPlotIndex(int newAxis)
{
    PlotIndex = newAxis;
    QVPlot->SetContinuousExpandYResetSlot();
}

void mtsPIDQtWidget::SlotEnableEventHandler(bool enable)
{
    QCBEnablePID->setChecked(enable);
}

void mtsPIDQtWidget::SlotEnableDirectControl(bool toggle)
{
    DirectControl = toggle;
    // if checked in DIRECT_CONTROL mode
    QVWDesiredPositionWidget->setEnabled(toggle);
    QVWPGainWidget->setEnabled(toggle);
    QVWIGainWidget->setEnabled(toggle);
    QVWDGainWidget->setEnabled(toggle);
    QCBEnablePID->setEnabled(toggle);
    QCBEnableTorqueMode->setEnabled(toggle);
    QPBMaintainPosition->setEnabled(toggle);
    QPBZeroPosition->setEnabled(toggle);
    QPBResetPIDGain->setEnabled(toggle);
    if(usingSimulink)
        SimulinkQtWidget.EnableWidget(toggle);
}

void mtsPIDQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // get data from the PID
    PID.GetPeriodStatistics(IntervalStatistics);

    PID.GetPositionJoint(PID.PositionJointGetParam);
    PID.PositionJointGetParam.Position().ElementwiseMultiply(UnitFactor);
    PID.GetPositionJointDesired(PID.PositionJointGetDesired);
    PID.PositionJointGetDesired.ElementwiseMultiply(UnitFactor);
    PID.GetVelocityJoint(PID.VelocityJointGetParam);
    PID.VelocityJointGetParam.Velocity().ElementwiseMultiply(UnitFactor);
    PID.GetEffortJointDesired(PID.EffortJoint);

    // update GUI
    QMIntervalStatistics->SetValue(IntervalStatistics);

    QVRCurrentPositionWidget->SetValue(PID.PositionJointGetParam.Position());
    QVRCurrentEffortWidget->SetValue(PID.EffortJoint);

    // display requested joint positions when we are not trying to set it using GUI
    if (!DirectControl) {
        QVWDesiredPositionWidget->SetValue(PID.PositionJointGetDesired);
    }

    // plot
    CurrentPositionSignal->AppendPoint(vctDouble2(PID.PositionJointGetParam.Timestamp(),
                                                  PID.PositionJointGetParam.Position().Element(PlotIndex)));
    DesiredPositionSignal->AppendPoint(vctDouble2(PID.PositionJointGetParam.Timestamp(),
                                                  PID.PositionJointGetDesired.Element(PlotIndex)));
    CurrentVelocitySignal->AppendPoint(vctDouble2(PID.VelocityJointGetParam.Timestamp(),
                                                  PID.VelocityJointGetParam.Velocity().Element(PlotIndex)));
    DesiredEffortSignal->AppendPoint(vctDouble2(PID.PositionJointGetParam.Timestamp(),
                                                -PID.EffortJoint.Element(PlotIndex))); // negate current to plot the same direction

    // UBC log file for PID joint efforts
    if(effortLogFile.is_open())
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

        effortLogFile << "PID effort: t = " << (intmax_t)s << "." << zeros << ms << std::endl;
        effortLogFile << "effort: " << PID.EffortJoint.Element(0) << " ";
        effortLogFile << PID.EffortJoint.Element(1) << " ";
        effortLogFile << PID.EffortJoint.Element(2) << " ";
        effortLogFile << PID.EffortJoint.Element(3) << " ";
        effortLogFile << PID.EffortJoint.Element(4) << " ";
        effortLogFile << PID.EffortJoint.Element(5) << " ";
        effortLogFile << PID.EffortJoint.Element(6) << std::endl;

        /*
        effortLogFile << "Joint Pos (deg): " << std::endl;
        effortLogFile << "t:   " << PID.PositionJointGetParam.Timestamp() << std::endl;
        effortLogFile << PID.PositionJointGetParam.Position() << std::endl;
        */
    }

    QVPlot->updateGL();
}

////------------ Private Methods ----------------
void mtsPIDQtWidget::setupUi(void)
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    const double maximum = 30000;

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
    gridLayout->addWidget(QVWDesiredPositionWidget, row, 1);
    row++;

    QLabel * pLabel = new QLabel("PGain");
    pLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(pLabel);
    QVWPGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWPGainWidget->SetStep(0.01);
    QVWPGainWidget->SetPrecision(3);
    QVWPGainWidget->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWPGainWidget, row, 1);
    row++;

    QLabel * dLabel = new QLabel("DGain");
    dLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(dLabel);
    QVWDGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWDGainWidget->SetStep(0.01);
    QVWDGainWidget->SetPrecision(3);
    QVWDGainWidget->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWDGainWidget, row, 1);
    row++;

    QLabel * iLabel = new QLabel("IGain");
    iLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(iLabel);
    QVWIGainWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    QVWIGainWidget->SetStep(0.001);
    QVWIGainWidget->SetPrecision(5);
    QVWIGainWidget->SetRange(-maximum, maximum);
    gridLayout->addWidget(QVWIGainWidget, row, 1);
    row++;

    QLabel * currentEffortLabel = new QLabel("Current effort (Nm)");
    currentEffortLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(currentEffortLabel, row, 0);
    QVRCurrentEffortWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVRCurrentEffortWidget->SetPrecision(5);
    gridLayout->addWidget(QVRCurrentEffortWidget, row, 1);
    row++;

    QLabel * desiredEffortLabel = new QLabel("Desired effort (Nm)");
    desiredEffortLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(desiredEffortLabel, row, 0);
    QVWDesiredEffortWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::SPINBOX_WIDGET);
    // DesiredPositionWidget->SetDecimals(2);
    QVWDesiredEffortWidget->SetStep(0.1);
    QVWDesiredEffortWidget->SetRange(-8.0, 8.0);
    QVWDesiredEffortWidget->SetValue(DesiredEffort);
    QVWDesiredEffortWidget->setEnabled(false);
    QVWDesiredEffortWidget->setStyleSheet(QString::fromUtf8("QPushButton:disabled"
                                                "{ color: gray }"
                                                ));
    gridLayout->addWidget(QVWDesiredEffortWidget, row, 1);
    row++;

    // plot
    QHBoxLayout * plotLayout = new QHBoxLayout;
    // plot control
    QVBoxLayout * plotButtonsLayout = new QVBoxLayout;
    // - pick axis to display
    QLabel * plotIndexLabel = new QLabel("Index");
    plotButtonsLayout->addWidget(plotIndexLabel);
    QSBPlotIndex = new QSpinBox();
    QSBPlotIndex->setRange(0, NumberOfAxis);
    plotButtonsLayout->addWidget(QSBPlotIndex);
    // legend
    QLabel * label;
    QPalette palette;
    palette.setColor(QPalette::Window, Qt::black);
    label = new QLabel("Current");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::red);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    label = new QLabel("Desired");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::green);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    label = new QLabel("Velocity");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::blue);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
    label = new QLabel("Effort");
    label->setAutoFillBackground(true);
    palette.setColor(QPalette::WindowText, Qt::white);
    label->setPalette(palette);
    plotButtonsLayout->addWidget(label);
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
    QCBEnableDirectControl = new QCheckBox("Direct control");
    QCBEnablePID = new QCheckBox("Enable PID");
    QCBEnableLOG = new QCheckBox("Enable LOG"); // UBC for enable LOG button
    QCBEnableTorqueMode = new QCheckBox("Enable torque mode");
    QPBMaintainPosition = new QPushButton("Maintain position");
    QPBZeroPosition = new QPushButton("Zero position");
    QPBResetPIDGain = new QPushButton("Reset PID gains");
    QHBoxLayout * testLayout = new QHBoxLayout;
    testLayout->addWidget(QCBEnableDirectControl);
    testLayout->addWidget(QCBEnablePID);
    testLayout->addWidget(QCBEnableTorqueMode);
    testLayout->addWidget(QCBEnableLOG); // UBC for enable LOG button
    testLayout->addWidget(QPBMaintainPosition);
    testLayout->addWidget(QPBZeroPosition);
    testLayout->addWidget(QPBResetPIDGain);
    testLayout->addStretch();
    QGroupBox * testGroupBox = new QGroupBox("Control");
    testGroupBox->setLayout(testLayout);

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

    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)), this, SLOT(SlotEnableDirectControl(bool)));
    connect(QCBEnablePID, SIGNAL(clicked(bool)), this, SLOT(SlotEnablePID(bool)));
    connect(this, SIGNAL(SignalEnablePID(bool)), this, SLOT(SlotEnableEventHandler(bool)));
    connect(QCBEnableTorqueMode, SIGNAL(toggled(bool)), this, SLOT(SlotEnableTorqueMode(bool)));
    connect(QCBEnableLOG, SIGNAL(toggled(bool)), this, SLOT(SlotEnableLOG(bool))); // UBC for enable LOG button
    connect(QPBMaintainPosition, SIGNAL(clicked()), this, SLOT(SlotMaintainPosition()));
    connect(QPBZeroPosition, SIGNAL(clicked()), this, SLOT(SlotZeroPosition()));
    connect(QPBResetPIDGain, SIGNAL(clicked()), this, SLOT(SlotResetPIDGain()));
    connect(QSBPlotIndex, SIGNAL(valueChanged(int)), this, SLOT(SlotPlotIndex(int)));

    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(gridLayout);
    mainLayout->addLayout(plotLayout);
    mainLayout->addWidget(testGroupBox);
    mainLayout->addWidget(timingFrame);

    setLayout(mainLayout);

    setWindowTitle(this->GetName().c_str());
    setMinimumWidth(750);
    resize(sizeHint());

    // connect signals & slots
    connect(QVWDesiredPositionWidget, SIGNAL(valueChanged()), this, SLOT(SlotPositionChanged()));
    connect(QVWDesiredEffortWidget, SIGNAL(valueChanged()), this, SLOT(SlotEffortChanged()));
    connect(QVWPGainWidget, SIGNAL(valueChanged()), this, SLOT(SlotPGainChanged()));
    connect(QVWDGainWidget, SIGNAL(valueChanged()), this, SLOT(SlotDGainChanged()));
    connect(QVWIGainWidget, SIGNAL(valueChanged()), this, SLOT(SlotIGainChanged()));

    // set initial values
    QCBEnableDirectControl->setChecked(DirectControl);
    SlotEnableDirectControl(DirectControl);
}

void mtsPIDQtWidget::JointLimitEventHandler(const vctBoolVec & flags)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "JointLimitEventHandler: " << flags << std::endl;
}

void mtsPIDQtWidget::ErrorEventHandler(const std::string & message)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "ErrorEventHandler: " << message << std::endl;
}

void mtsPIDQtWidget::EnableEventHandler(const bool & enable)
{
    emit SignalEnablePID(enable);
}

void mtsPIDQtWidget::EnablePIDFromSimulinkQt(const mtsBool &enable)
{
    EnablePID(false, enable);
}

void mtsPIDQtWidget::EnableLogsFromSimulinkQt(const mtsBool &enable)
{
    EnableLog(false, enable);
}

void mtsPIDQtWidget::EnablePID(bool localButtonUsed, bool enable)
{
    if(enable) {
        PID.EnableIO(enable); //make sure to re-enable IO, disabling can only happen if Simulink used
    }

    if(localButtonUsed) {
        PID.Enable(enable);
        CMN_LOG_RUN_WARNING << "PID controller being turned " << (enable ? "ON" : "OFF") << std::endl;
    } else {
        PID.EnableIO(enable);
        CMN_LOG_RUN_WARNING << "PID IO being turned " << (enable ? "ON" : "OFF") << std::endl;
        QCBEnablePID->setChecked(enable);
    }

    //update GUI
    QVWDesiredPositionWidget->setEnabled(enable);
    QVWPGainWidget->setEnabled(enable);
    QVWIGainWidget->setEnabled(enable);
    QVWDGainWidget->setEnabled(enable);
    QVWDesiredEffortWidget->setEnabled(enable);
    QVRCurrentEffortWidget->setEnabled(enable);
}

void mtsPIDQtWidget::EnableLog(bool localButtonUsed, bool enable)
{
    logsEnabled = enable;
    if(logsEnabled)
    {
        //CMN_LOG_RUN_WARNING << "mtsPIDQtWidget: logs enabled" << std::endl;

        // UBC open output file
        if(!effortLogFile.is_open())
        {
            std::string fname = component_name;
            fname.append("PIDWidgetJointEffort.log");
            checkFileExists(&fname);
            effortLogFile.open(fname.c_str(), std::ofstream::out | std::ofstream::app);
            if(!effortLogFile.is_open())
                CMN_LOG_CLASS_RUN_ERROR << "Failed to open the log file!" << std::endl;
        }

        if(effortLogFile.is_open())
            effortLogFile << "*********Log number:   " << logEntryIndex << std::endl;
    }
    else {
        closeLogs(false);
        //CMN_LOG_RUN_WARNING << "mtsPIDQtWidget: logs disabled" << std::endl;
    }

    PID.EnableLogs(logsEnabled);

    if(!localButtonUsed) {
        QCBEnableLOG->setChecked(enable);
    }
}
