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
#include <sstream>
#include <string>
#include <stdio.h>
#include <cisstParameterTypes/prmSocketDataPacket.h>

prmSocketDataPacket::prmSocketDataPacket(const char * flag, const char * delim, vctDoubleVec data)
{
    this->flag = flag;
    this->delim = delim;
    this->data = data;
}

prmSocketDataPacket::~prmSocketDataPacket() { }

//write own because it has to be in a format with which Matlab will play nicely
std::string prmSocketDataPacket::SerializeData(const char * format, unsigned int bufferSize)
{
    char s[bufferSize+1];
    s[0] = '\0';

    strcat(s,flag);
    strcat(s,delim);

    char val[bufferSize/(data.size())];
    for(unsigned int i = 0; i < data.size(); i++) {
        snprintf(val, bufferSize, format, data.at(i));

        strcat(val, delim);
        strcat(s,val);
    }

    s[bufferSize] = '\0';
    return std::string(s);
}

