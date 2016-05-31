/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
Author(s):	Angelica Ruszkowski
Created on:   2014-11-12

(C) Copyright 2008 Johns Hopkins University (JHU), All Rights
Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


/*!
\file
\brief Struct containing information that will be based over an osaSocket to Simulink for controller design; i.e. struct to pass controller inputs.
*/


#ifndef _prmSimulinkDataPacket_h
#define _prmSimulinkDataPacket_h


//basic includes
#include <cisstVector.h>

// Always include last
#include <cisstParameterTypes/prmExport.h>

class CISST_EXPORT prmSimulinkDataPacket
{
    //CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:

    /*! Constructor */
    prmSimulinkDataPacket();
    prmSimulinkDataPacket(double dt, double ts);

    /*! Destructor */
    ~prmSimulinkDataPacket();

    /*! Setters */
    void SetPositionData(vctDoubleVec currentPos, vctDoubleVec desiredPos);
    void SetVelocityData(vctDoubleVec currentVel);

    /*! Serialization */
    std::string SerializeData(unsigned int buffSize);

    struct KinematicData
    {
        vctDoubleVec currentPosition;
        vctDoubleVec desiredPosition;
        vctDoubleVec currentVelocity;

        KinematicData()
        { }

        void ResetAll()
        {
            currentPosition.SetAll(0.0);
            desiredPosition.SetAll(0.0);
            currentVelocity.SetAll(0.0);
        }
    };

private:
    double          dt;
    double          timeStamp;
    KinematicData   data;

}; // _prmSimulinkDataPacket_h


//CMN_DECLARE_SERVICES_INSTANTIATION(prmSimulinkDataPacket);


#endif
