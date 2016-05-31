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


#ifndef _mtsPIDQtWidget_h
#define _mtsPIDQtWidget_h

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

#include <QCheckBox>
#include <QSpinBox>
#include <QPushButton>
#include <sawControllers/sawControllersQtExport.h>

class CISST_EXPORT mtsPIDQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsPIDQtWidget(const std::string & componentName, unsigned int numberOfAxis,
                   double periodInSeconds = 50.0 * cmn_ms, const bool usingSimulinkControl = false);
    mtsPIDQtWidget(const mtsComponentConstructorNameAndUInt &arg);
    ~mtsPIDQtWidget(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    void Init();
    virtual void closeEvent(QCloseEvent * event);

signals:
    void SignalEnablePID(bool enable);

private slots:
    //! slot enable/disable mtsPID controller
    void SlotEnablePID(bool toggle);
    void SlotEnableTorqueMode(bool toggle);
    void SlotEnableLOG(bool toggle);
    //! slot send desired pos when input changed
    void SlotPositionChanged(void);
    //! slot send desired effort when input changed
    void SlotEffortChanged(void);
    void SlotPGainChanged(void);
    void SlotDGainChanged(void);
    void SlotIGainChanged(void);
    //! slot reset desired pos to current pos
    void SlotMaintainPosition(void);
    //! go to zero position
    void SlotZeroPosition(void);
    //! slot reset pid gain to current gain
    void SlotResetPIDGain(void);
    //! slot to select which axis to plot
    void SlotPlotIndex(int newAxis);
    //! slot to change Enable Checkbox
    void SlotEnableEventHandler(bool enable);
    void SlotEnableDirectControl(bool toggle);

    //! timer event to update GUI
    void timerEvent(QTimerEvent * event);

private:
    //! setup PID controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;
    mtsIntervalStatistics IntervalStatistics;

    void JointLimitEventHandler(const vctBoolVec & flags);
    void ErrorEventHandler(const std::string & message);
    void EnableEventHandler(const bool & enable);

protected:

    struct ControllerPIDStruct {
        mtsFunctionVoid  ResetController;
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableIO;
        mtsFunctionWrite EnableTorqueMode;
        mtsFunctionWrite EnableLogs;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetEffortJoint;
        mtsFunctionRead  GetPositionJoint;
        mtsFunctionRead  GetVelocityJoint;
        mtsFunctionRead  GetPositionJointDesired;
        mtsFunctionRead  GetEffortJointDesired;
        mtsFunctionRead  GetPeriodStatistics;

        prmPositionJointGet PositionJointGetParam;
        vctDoubleVec        PositionJointGetDesired;
        prmVelocityJointGet VelocityJointGetParam;
        vctDoubleVec        EffortJoint;

        mtsFunctionRead  GetJointType;
        mtsFunctionRead  GetPGain;
        mtsFunctionRead  GetDGain;
        mtsFunctionRead  GetIGain;

        mtsFunctionWrite SetPGain;
        mtsFunctionWrite SetDGain;
        mtsFunctionWrite SetIGain;
    } PID;

    struct SimulinkQtWidgetStruct {
        mtsFunctionWrite EnableWidget;
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableLogs;
        mtsFunctionRead  IsEnabled;
    } SimulinkQtWidget;

private:
    // UBC log file for PID joint efforts
    std::ofstream effortLogFile;
    int  logEntryIndex;
    bool logsEnabled;
    void closeLogs(bool appClosing);
    void checkFileExists(std::string * fname);

    void EnablePIDFromSimulinkQt(const mtsBool &enable);
    void EnableLogsFromSimulinkQt(const mtsBool &enable);

    void EnableLog(bool localButtonUsed, bool enable);
    void EnablePID(bool localButtonUsed, bool enable);

    std::string component_name;
    bool usingSimulink;

    //! SetPosition
    vctDoubleVec            DesiredPosition;
    vctDoubleVec            DesiredPositionFromPID;
    prmPositionJointSet     DesiredPositionParam;
    vctDoubleVec            UnitFactor;
    bool DirectControl;

    size_t NumberOfAxis;

    //! Set Effort/Torque
    vctDoubleVec            DesiredEffort;
    prmForceTorqueJointSet  DesiredEffortParam;

    // GUI: Commands
    QCheckBox * QCBEnableDirectControl;
    QCheckBox * QCBEnablePID;
    QCheckBox * QCBEnableTorqueMode;
    QCheckBox * QCBEnableLOG;
    QPushButton * QPBMaintainPosition;
    QPushButton * QPBZeroPosition;
    QPushButton * QPBResetPIDGain;
    vctQtWidgetDynamicVectorDoubleWrite * QVWDesiredPositionWidget;
    vctQtWidgetDynamicVectorDoubleWrite * QVWDesiredEffortWidget;
    vctQtWidgetDynamicVectorDoubleWrite * QVWPGainWidget;
    vctQtWidgetDynamicVectorDoubleWrite * QVWDGainWidget;
    vctQtWidgetDynamicVectorDoubleWrite * QVWIGainWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRCurrentPositionWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRCurrentEffortWidget;

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
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsPIDQtWidget);

#endif // _mtsPIDQtWidget_h
