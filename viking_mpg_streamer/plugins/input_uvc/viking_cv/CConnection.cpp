/*******************************************************************************************/
/* The MIT License (MIT)                                                                   */
/*                                                                                         */
/* Copyright (c) 2014 - Marina High School FIRST Robotics Team 4276 (Huntington Beach, CA) */
/*                                                                                         */
/* Permission is hereby granted, free of charge, to any person obtaining a copy            */
/* of this software and associated documentation files (the "Software"), to deal           */
/* in the Software without restriction, including without limitation the rights            */
/* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell               */
/* copies of the Software, and to permit persons to whom the Software is                   */
/* furnished to do so, subject to the following conditions:                                */
/*                                                                                         */
/* The above copyright notice and this permission notice shall be included in              */
/* all copies or substantial portions of the Software.                                     */
/*                                                                                         */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR              */
/* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,                */
/* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE             */
/* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER                  */
/* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,           */
/* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN               */
/* THE SOFTWARE.                                                                           */
/*******************************************************************************************/

/*******************************************************************************************/
/* We are a high school robotics team and always in need of financial support.             */
/* If you use this software for commercial purposes please return the favor and donate     */
/* (tax free) to "Marina High School Educational Foundation"  (Huntington Beach, CA)       */
/*******************************************************************************************/

#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <string>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#define SOCKET_ERROR        -1

#include "CLedStrip.h"
#include "CConnection.h"
#include "dbgMsg.h"

CConnection::CConnection()
{
}

CConnection::~CConnection()
{
}

void CConnection::init(int port, char* pName)
{
    m_port = port;
    m_sock = -1;
    m_client = -1;
    m_state = CONNECTION_STATE_INIT;
    m_threadSender = -1;
    m_threadListener = -1;
    strcpy(m_name, pName);
}

std::string CConnection::displayText() const
{
    char buf[256];
    sprintf(buf, "%s Connection port: %d  sock: %d  client: %d  state: %s", m_name, m_port, m_sock, m_client, connectionStateToText(m_state).c_str());
    return buf;
}

void CConnection::showConnectionContext(const char* func, const char* msg) const
{
    char buf[256];
    buf[0] = 0;
    strcat(buf, func);
    strcat(buf, " - ");
    strcat(buf, msg);
    dbgMsg(buf);
}

std::string CConnection::connectionStateToText(CONNECTION_STATE eState)
{
    std::string sRet;
    switch (eState)
    {
    case CONNECTION_STATE_RESET:
        sRet = "CONNECTION_STATE_RESET";
        break;

    case CONNECTION_STATE_INIT:
        sRet = "CONNECTION_STATE_INIT";
        break;

    case CONNECTION_STATE_BIND_OK:
        sRet = "CONNECTION_STATE_BIND_OK";
        break;

    case CONNECTION_STATE_ACCEPT_OK:
        sRet = "CONNECTION_STATE_ACCEPT_OK";
        break;

    default:
        sRet = "****";
        break;
    }
    return sRet;
}

int CConnection::serverBind()
{
    char buf[256];
    int tempSock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    struct sockaddr_in address;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_family = AF_INET;
    address.sin_port = htons(m_port);

    if (::bind(tempSock, (struct sockaddr*) & address, sizeof (struct sockaddr_in)) == SOCKET_ERROR)
    {
#ifdef CV_DEBUG_CONNECT
        sprintf(buf, "error %d : couldn't bind ", errno);
        showConnectionContext("serverBind", buf);
#endif
        close(tempSock);
        tempSock = -1;
    }
    else
    {
        if (::listen(tempSock, 10) == SOCKET_ERROR)
        {
#ifdef CV_DEBUG_CONNECT
            sprintf(buf, "error %d : couldn't listen ", errno);
            showConnectionContext("serverBind", buf);
#endif
            close(tempSock);
            tempSock = -1;
        }
        return tempSock;
    }
}

int CConnection::acceptConnection()
{
    //
    // Accept will block
    //
    char buf[256];
    unsigned int addrlen = sizeof (struct sockaddr);
    struct sockaddr_in address = {0};
    int client = ::accept(m_sock, (struct sockaddr*) & address, &addrlen);
    if (client == SOCKET_ERROR)
    {
#ifdef CV_DEBUG_CONNECT

        sprintf(buf, "error %d : couldn't accept ", errno);
        showConnectionContext("acceptConnection", buf);
#endif
        client = -1;
    }
    return client;
}

void CConnection::resetConnection()
{
#ifdef CV_DEBUG_CONNECT
    showConnectionContext("resetConnection", "Before reset connection - ");
#endif
    if ((m_state == CONNECTION_STATE_BIND_OK)
            || (m_state == CONNECTION_STATE_ACCEPT_OK))
    {
        m_state = CONNECTION_STATE_RESET;
#ifdef CV_DEBUG_CONNECT
        showConnectionContext("resetConnection", "Resetting connection");
#endif
        fflush(NULL);
        close(m_client);
        m_client = -1;
        close(m_sock);
        m_sock = -1;

        m_state = CONNECTION_STATE_INIT;
    }
}

void CConnection::tryToBind()
{
    if (m_sock != -1)
    {
        close(m_sock);
        m_sock = -1;
    }
    m_sock = serverBind();
    if (m_sock == -1)
    {
        m_state = CONNECTION_STATE_INIT;
    }
    else
    {
        m_state = CONNECTION_STATE_BIND_OK;
#ifdef CV_DEBUG_CONNECT
        showConnectionContext("tryToBind", "Bind OK");
#endif  
    }
}

void CConnection::tryToAccept()
{
    if (m_state == CONNECTION_STATE_BIND_OK)
    {
#ifdef CV_DEBUG_CONNECT
        showConnectionContext("tryToAccept", "Trying to accept");
#endif
        int tempClient = acceptConnection();
        if (tempClient == -1)
        {
#ifdef CV_DEBUG_CONNECT
            showConnectionContext("tryToAccept", "Accept failed");
#endif
        }
        else
        {
            int browserBytes = 0;
            char browser_request[1024];
            browserBytes = readClient(browser_request, 1023);
            m_client = tempClient;
            calledOnceAfterAccept();
            m_state = CONNECTION_STATE_ACCEPT_OK;
#ifdef CV_DEBUG_CONNECT
            showConnectionContext("tryToAccept", "Accept OK");
#endif
        }
    }
}

int CConnection::writeClient(char* pBuf, unsigned int siz)
{
    return write(m_client, pBuf, siz);
}

int CConnection::readClient(char* pBuf, unsigned int siz)
{
    return read(m_client, pBuf, siz);
}
