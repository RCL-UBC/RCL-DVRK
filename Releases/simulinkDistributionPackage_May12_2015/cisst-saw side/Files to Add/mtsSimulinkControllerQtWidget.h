/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Angelica Ruszkowski
  Created on: 2015-04-06

  Based off mtsPIDQtWidget.cpp

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsSimulinkControllerQtWidget_h
#define _mtsSimulinkControllerQtWidget_h

#include <cisstCommon/cmnXMLPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstVector/vctQtWidgetDynamicVector.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>

#include <QCheckBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QPushButton>
#include <sawControllers/sawControllersQtExport.h>

class CISST_EXPORT mtsSimulinkControllerQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsSimulinkControllerQtWidget(const std::string & componentName, unsigned int numberOfAxis,
                   double periodInSeconds = 50.0 * cmn_ms, int kinPortNum = 12345, int torquePortNum = 54321);
    mtsSimulinkControllerQtWidget(const mtsComponentConstructorNameAndUInt &arg);
    ~mtsSimulinkControllerQtWidget(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    void Init();
    virtual void closeEvent(QCloseEvent * event);

    /******************Interfaces******************/
    struct ControllerPIDStruct {
        mtsFunctionRead  GetJointType;
        mtsFunctionRead  GetPositionJointDesired;
    } PID;

    struct RobotArmStruct {
        mtsFunctionRead         GetRobotControlState;
        mtsFunctionRead         GetPositionCartesian;
        prmPositionCartesianGet PositionCartesianCurrent;
    } RobotArm;

    struct PIDQtWidgetStruct {
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableLogs;
    } PIDQtWidget;

    struct SimulinkRobotStruct {
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableLogs;
        mtsFunctionWrite SetHostIP;
        mtsFunctionWrite SetTorquePortNum;
        mtsFunctionWrite SetKinematicsPortNum;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetPositionCartDesired;
        mtsFunctionWrite SetControllerTypeToJoint;

        mtsFunctionRead  GetControllerTypeIsJoint;
        mtsFunctionRead  GetPositionJointDesired;
        mtsFunctionRead  GetEffortJointDesired;
        mtsFunctionRead  GetPositionJoint;
        mtsFunctionRead  GetVelocityJoint;
        mtsFunctionRead  GetPeriodStatistics;

        vctDoubleVec     PositionJointGetDesired;
        vctDoubleVec     EffortJoint;

        mtsBool          UsingJointControl;

        prmPositionJointGet PositionJointGetParam;
        prmVelocityJointGet VelocityJointGetParam;
    } SimulinkController;

private slots:
    //! slot enable/disable replacing mtsSimulinkController controller with simulink controller
    void SlotEnableSimulink(bool toggle);
    void SlotSetSimulinkJointControl(bool toggle);
    void ChangeSimulinkStatusFromBelow(const mtsBool &socketsDone);
    void SlotEnableLOG(bool toggle);
    //! slot send desired pos when input changed
    void SlotPositionChanged(void);
    //! slot to select which axis to plot
    void SlotPlotIndex(int newAxis);
    //! timer event to update GUI
    void timerEvent(QTimerEvent * event);

private:
    int  TimerPeriodInMilliseconds;
    std::string component_name;
    bool SimulinkActuallyInControl;
    size_t NumberOfJoints;
    mtsIntervalStatistics IntervalStatistics;

    /******************Kinematics Members******************/
    //! SetPosition
    vctDoubleVec            DesiredPosition;
    prmPositionJointSet     DesiredPositionParam;
    vctDoubleVec            UnitFactor;

    /******************GUI Members******************/
    // GUI: Commands
    QCheckBox * QCBEnableSimulink;
    QCheckBox * QCBSimControlType;
    QCheckBox * QCBEnableLOG;
    vctQtWidgetDynamicVectorDoubleWrite * QVWDesiredPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead  * QVRCurrentPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead  * QVRCurrentEffortWidget;
    QLineEdit * LEhostIP;
    QLineEdit * LEtorquePort;
    QLineEdit * LEkinematicPort;

    // GUI: plot
    vctPlot2DOpenGLQtWidget * QVPlot;
    vctPlot2DBase::Signal * CurrentPositionSignal;  //RED
    vctPlot2DBase::Signal * DesiredPositionSignal;  //GREEN
    vctPlot2DBase::Signal * CurrentVelocitySignal;  //LIGHT BLUE
    vctPlot2DBase::Signal * DesiredEffortSignal;    //WHITE
    QSpinBox * QSBPlotIndex;
    int PlotIndex;

    // GUI: timing
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;

    /******************General Functions******************/
    void setupUi(void);
    void EnableLog(bool localButtonUsed, bool enable);

    /******************Interface Functions******************/
    void EnableSimulinkFromPIDQt(const mtsBool & enable);
    void EnableLogsFromPIDQt(const mtsBool & enable);
    void EnableGUIFromPIDQt(const mtsBool & enable);
    void GetSimulinkInControl(mtsBool & inControl) const;

    /******************Logging******************/
    std::ofstream simulinkLogFile;
    int  logEntryIndex;
    bool logsEnabled;
    void closeLogs(bool appClosing);
    void checkFileExists(std::string * fname);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSimulinkControllerQtWidget);

#endif // _mtsSimulinkControllerQtWidget_h
