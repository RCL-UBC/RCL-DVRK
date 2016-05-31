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
#include <cisstParameterTypes/prmSimulinkDataPacket.h>

prmSimulinkDataPacket::prmSimulinkDataPacket()
{
    prmSimulinkDataPacket(-1, -1);
}

prmSimulinkDataPacket::prmSimulinkDataPacket(double dt, double ts)
{
    this->dt          = dt;
    this->timeStamp   = ts;
}

prmSimulinkDataPacket::~prmSimulinkDataPacket() { }

void prmSimulinkDataPacket::SetPositionData(vctDoubleVec currentPos, vctDoubleVec desiredPos)
{
    data.currentPosition = currentPos;
    data.desiredPosition = desiredPos;
}

void prmSimulinkDataPacket::SetVelocityData(vctDoubleVec currentVel)
{
    data.currentVelocity = currentVel;
}

//write own because it has to be in a format with which Matlab will play nicely
std::string prmSimulinkDataPacket::SerializeData(unsigned int buffSize)
{
    char bigBuffer[buffSize];
    if((data.currentPosition).size() == 7 || (data.currentPosition).size() == 8) {  //joint level on PSM or MTM (last joint is gripper for MTM, can ignore
        sprintf(bigBuffer, "%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%016.8f$",
                    dt,
                    data.currentPosition(0), data.currentPosition(1), data.currentPosition(2), data.currentPosition(3), data.currentPosition(4), data.currentPosition(5), data.currentPosition(6),
                    data.desiredPosition(0), data.desiredPosition(1), data.desiredPosition(2), data.desiredPosition(3), data.desiredPosition(4), data.desiredPosition(5), data.desiredPosition(6),
                    data.currentVelocity(0), data.currentVelocity(1), data.currentVelocity(2), data.currentVelocity(3), data.currentVelocity(4), data.currentVelocity(5), data.currentVelocity(6),
                    timeStamp
                );
    } else  if((data.currentPosition).size() == 12) { //cartesian control
        sprintf(bigBuffer, "%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%013.8f,%016.8f$",
                    dt,
                    data.currentPosition(0), data.currentPosition(1), data.currentPosition(2), data.currentPosition(3), data.currentPosition(4), data.currentPosition(5), data.currentPosition(6), data.currentPosition(7), data.currentPosition(8), data.currentPosition(9), data.currentPosition(10), data.currentPosition(11),
                    data.desiredPosition(0), data.desiredPosition(1), data.desiredPosition(2), data.desiredPosition(3), data.desiredPosition(4), data.desiredPosition(5), data.desiredPosition(6), data.desiredPosition(7), data.desiredPosition(8), data.desiredPosition(9), data.desiredPosition(10), data.desiredPosition(11),
                    data.currentVelocity(0), data.currentVelocity(1), data.currentVelocity(2), data.currentVelocity(3), data.currentVelocity(4), data.currentVelocity(5),
                    timeStamp
                );
    } else {
        CMN_LOG_RUN_WARNING << "prmSimulinkDataPacket::SerializeData unknown data format" << std::endl;
        CMN_LOG_RUN_ERROR << "prmSimulinkDataPacket::SerializeData unknown data format" << std::endl;
    }

    return std::string(bigBuffer);
}

