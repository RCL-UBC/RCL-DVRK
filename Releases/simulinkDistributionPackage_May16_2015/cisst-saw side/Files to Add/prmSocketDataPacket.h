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


#ifndef _prmSocketDataPacket_h
#define _prmSocketDataPacket_h


//basic includes
#include <cisstVector.h>

// Always include last
#include <cisstParameterTypes/prmExport.h>

class CISST_EXPORT prmSocketDataPacket
{
    //CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:

    /*! Constructor */
    prmSocketDataPacket(const char * flag, const char * delim, vctDoubleVec data);

    /*! Destructor */
    ~prmSocketDataPacket();


    /*! Serialization */
    std::string SerializeData(const char * format, unsigned int bufferSize);

private:
    const char * flag;
    const char * delim;
    vctDoubleVec data;

}; // _prmSocketDataPacket_h


//CMN_DECLARE_SERVICES_INSTANTIATION(prmSocketDataPacket);


#endif
