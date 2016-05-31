/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
Author(s):	Angelica Ruszkowski
Created on:   2015-01-26

Based off prmSocketDataPacket by Angelica Ruszkowski

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
\brief Classes that houses most logic for communicating between the dVRK and something else. Two way communication achieved using two sockets, one listening, one sending
*/

#ifndef _prmTwoWaySocket_h
#define _prmTwoWaySocket_h


//basic includes
#include <cisstCommon/cmnGenericObject.h>
#include <cisstVector.h>
#include <cisstParameterTypes/prmSocketDataPacket.h>
#include <cisstParameterTypes/prmSimulinkDataPacket.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaSocket.h>
#include <cisstOSAbstraction/osaSocketServer.h>

// Always include last
#include <cisstParameterTypes/prmExport.h>

class CISST_EXPORT prmTwoWaySocket : public cmnGenericObject
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:

    /*! Constructor */
    prmTwoWaySocket(unsigned short portRecv = 12345,
                    unsigned short portSend = 54321,
                    std::string host = "127.0.0.1",
                    unsigned int bufferRecvSize = 255,
                    unsigned int bufferSendSize = 255);

    /*! Destructor */
    ~prmTwoWaySocket(void);

    bool ConnectRecvSocket(struct timeval timeout);
    bool ConnectSendSocket(struct timeval timeout);

    int RecvData(char* receivedData, bool changeToUpperEndian);
    int SendData(prmSocketDataPacket* data, const char * format);
    int SendData(prmSimulinkDataPacket* data);

    bool IsRecvSocketConnected();
    bool IsSendSocketConnected();

    void CloseRecvSocket();
    void CloseSendSocket();

    unsigned int GetBufferRecvSize() { return bufferRecvSize; }
    unsigned int GetBufferSendSize() { return bufferSendSize; }

private:
    unsigned short  portRecv;
    unsigned short  portSend;
    std::string     host;

    osaSocketServer serverRecv;
    osaSocketServer serverSend; //should be just a client, but Matlab won't open the port, so open it here
    int             serverRecvFD;
    int             serverSendFD;
    osaSocket *     socketRecv;
    osaSocket *     socketSend;
    bool            isServerRecvConnected;
    bool            isServerSendConnected;
    unsigned int    bufferRecvSize;
    unsigned int    bufferSendSize;
};

CMN_DECLARE_SERVICES_INSTANTIATION(prmTwoWaySocket)

#endif // _prmTwoWaySocket_h
