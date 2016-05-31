/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
Author(s):	Angelica Ruszkowski
Created on:   2015-01-26

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
#include <cisstParameterTypes/prmTwoWaySocket.h>

CMN_IMPLEMENT_SERVICES(prmTwoWaySocket);

prmTwoWaySocket::prmTwoWaySocket(unsigned short portRecv,     unsigned short portSend, std::string host,
                                 unsigned int bufferRecvSize, unsigned int bufferSendSize):
    portRecv(portRecv),
    portSend(portSend),
    host(host),
    bufferRecvSize(bufferRecvSize),
    bufferSendSize(bufferSendSize)
{
    socketRecv            = 0;
    socketSend            = 0;
    serverRecvFD          = 0;
    serverSendFD          = 0;
    isServerRecvConnected = false;
    isServerSendConnected = false;

    // Set up sockets
    while (!serverRecv.AssignPort(this->portRecv)) {
        CMN_LOG_CLASS_RUN_WARNING << "prmTwoWaySocket::Trying to assign portRecv; Will try again in 5 seconds" << std::endl;
        osaSleep(2.0 * cmn_s);
    }
    while (!serverSend.AssignPort(this->portSend)) {
        CMN_LOG_CLASS_RUN_WARNING << "prmTwoWaySocket::Trying to assign portSend; Will try again in 5 seconds" << std::endl;
        osaSleep(2.0 * cmn_s);
    }

    isServerRecvConnected = serverRecv.Listen();
    isServerSendConnected = serverSend.Listen();

    if (!isServerRecvConnected) {
        CMN_LOG_CLASS_RUN_ERROR << "prmTwoWaySocket::Could not set serverRecv to listen mode" << std::endl;
    }
    if (!isServerSendConnected) {
        CMN_LOG_CLASS_RUN_ERROR << "prmTwoWaySocket::Could not set serverSend to listen mode" << std::endl;
    }

    CMN_LOG_CLASS_RUN_WARNING << "prmTwoWaySocket::Started serverRecv on " << host << ": " << this->portRecv << std::endl;
    CMN_LOG_CLASS_RUN_WARNING << "prmTwoWaySocket::Started serverSend on " << host << ": " << this->portSend << std::endl;

    serverRecvFD  = serverRecv.GetIdentifier();
    serverSendFD  = serverSend.GetIdentifier();
}

prmTwoWaySocket::~prmTwoWaySocket(void)
{
    delete socketRecv;
    delete socketSend;
}

//Returns false if not connected; true if connected
bool prmTwoWaySocket::ConnectRecvSocket(struct timeval timeout)
{
    //for select() for sockets
    fd_set fds;

    //Set up server which sends data from C++ to whomever
    if(!(socketRecv != 0 && socketRecv->IsConnected())) //not accepted and not connected
    {
        //CMN_LOG_RUN_WARNING << "socketRecv: try to connect"<< std::endl;
        if(isServerRecvConnected) //try to connect if server connected
        {
            FD_ZERO(&fds); //clear the set
            FD_SET(serverRecvFD, &fds); //add descriptor to the set

            //writefds
            int rv = select(serverRecvFD+1, &fds, NULL, NULL, &timeout);
            if (rv == -1) {
                CMN_LOG_RUN_WARNING << "prmTwoWaySocket::serverRecv: Error in select" << std::endl;
            } else if (rv == 0) {
                //CMN_LOG_RUN_WARNING << "prmTwoWaySocket::serverRecv: Timeout occurred!" << std::endl;
            } else { //something's there!
                socketRecv = serverRecv.Accept(); //socketRecv == 0 if not connected
                CMN_LOG_RUN_WARNING << "prmTwoWaySocket::serverRecv: Accepted" << std::endl;
            }
        }
        //else, would have been reported in initialization, something is wrong. Assume it's right :) Optimism!

        if(socketRecv != 0) //check if accepted
        {
            //CMN_LOG_RUN_WARNING << "socketRecv Accepted!" << std::endl;
            return true;
        }
    }
    return false;
}

//Returns false if not connected; true if connected
bool prmTwoWaySocket::ConnectSendSocket(struct timeval timeout)
{
    //for select() for sockets
    fd_set fds;

    //Set up server which sends data from C++ to whomever
    if(!(socketSend != 0 && socketSend->IsConnected())) //not accepted and not connected
    {
        //CMN_LOG_RUN_WARNING << "socketSend: try to connect"<< std::endl;
        if(isServerSendConnected) //try to connect if server connected
        {
            FD_ZERO(&fds); //clear the set
            FD_SET(serverSendFD, &fds); //add descriptor to the set

            //writefds
            int rv = select(serverSendFD+1, &fds, NULL, NULL, &timeout);
            if (rv == -1) {
                CMN_LOG_RUN_WARNING << "prmTwoWaySocket::serverSend: Error in select" << std::endl;
           }  else if (rv == 0) {
                //CMN_LOG_RUN_WARNING << "prmTwoWaySocket::serverSend: Timeout occurred!" << std::endl;
            } else { //something's there!
                socketSend = serverSend.Accept(); //socketSend == 0 if not connected
                CMN_LOG_RUN_WARNING << "prmTwoWaySocket::serverSend: Accepted" << std::endl;
            }
        }
        //else, would have been reported in initialization, something is wrong. Assume it's right :) Optimism!

        if(socketSend != 0) //check if accepted
        {
            //CMN_LOG_RUN_WARNING << "socketSend Accepted!" << std::endl;
            return true;
        }
    }
    return false;
}

int prmTwoWaySocket::RecvData(char* receivedData, bool changeToUpperEndian)
{
    int bytesRead = socketRecv->Receive(receivedData,bufferRecvSize);
    if(bytesRead == -1) {
        receivedData[0] = '\0';
    }
    else if (bytesRead > 0) {
        if(changeToUpperEndian) {
            for (int i = 0; i < bytesRead; i++)
                receivedData[i] = toupper(receivedData[i]);
        }
        receivedData[bytesRead] = 0;
    }
    else {
        //CMN_LOG_RUN_WARNING << "No dice" << std::endl;
        /*buffer empty, do nothing*/
        receivedData[0] = '\0';
    }
    return bytesRead;
}

int prmTwoWaySocket::SendData(prmSocketDataPacket* data, const char * format)
{
    std::string dataString  = data->SerializeData(format, bufferSendSize);

    char bufferSend[bufferSendSize];
    sprintf(bufferSend, "%s", dataString.c_str());

    //Other error handling?
    return socketSend->Send(bufferSend);
}

int prmTwoWaySocket::SendData(prmSimulinkDataPacket* data)
{
    std::string dataString  = data->SerializeData(bufferSendSize);

    char bufferSend[bufferSendSize];
    sprintf(bufferSend, "%s", dataString.c_str());

    //Other error handling?
    return socketSend->Send(bufferSend);
}

bool prmTwoWaySocket::IsRecvSocketConnected()
{
    return (socketRecv != 0 && socketRecv->IsConnected());
}

bool prmTwoWaySocket::IsSendSocketConnected()
{
    return (socketSend != 0 && socketSend->IsConnected());
}

void prmTwoWaySocket::CloseRecvSocket()
{
    if(IsRecvSocketConnected())
        socketRecv->Close();
    isServerRecvConnected = false;
}

void prmTwoWaySocket::CloseSendSocket()
{
    if(IsSendSocketConnected())
        socketSend->Close();
    isServerSendConnected = false;
}
