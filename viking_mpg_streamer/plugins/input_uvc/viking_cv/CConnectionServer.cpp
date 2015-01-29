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


#include <stdio.h>
#include <string>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "../v4l2uvc.h" // this header will includes the ../../mjpg_streamer.h

#include "CLedStrip.h"
#include "CConnection.h"
#include "CToteRectangle.h"
#include "CTargetInfo.h"
#include "CVideoFrame.h"
#include "CVideoFrameQueue.h"
#include "CConnectionServer.h"
#include "CGpioLed.h"
#include "CToteDetector.h"
#include "CTestMonitor.h"
#include "CFrameGrinder.h"
#include "CMessageFromClient.h"
#include "dbgMsg.h"

// Global shutdown flag is set when user typed Ctrl-C
bool g_isShutdown = false;

extern globals *pglobal;
extern context cams[MAX_INPUT_PLUGINS];

static CBrowserConnection static_browserConnection;
static CTextConnection static_textConnection;

void sigPipeHandler(int sig_num)
{
    static_browserConnection.resetConnection();
    static_textConnection.resetConnection();
    fflush(NULL);
}

void sigCtrlCHandler(int sig_num)
{
    dbgMsg_s("shutting down...\n");
    g_isShutdown = true;
    static_browserConnection.resetConnection();
    static_textConnection.resetConnection();
    fflush(NULL);
}

CConnectionServer::CConnectionServer()
{
    m_textServerThread = -1;
    m_browserServerThread = -1;
}

CConnectionServer::CConnectionServer(const CConnectionServer& orig)
{
}

CConnectionServer::~CConnectionServer()
{
}

void* connect_thread(void* pVoid)
{
    CConnection* pCtx = (CConnection*) pVoid;
    pCtx->resetConnection();
    while (!g_isShutdown)
    {
        if (pCtx->m_state == CConnection::CONNECTION_STATE_INIT)
        {
#ifdef CV_DEBUG_CONNECT
            pCtx->showConnectionContext("connect_thread", "Before TryToBind()  ");
#endif
            pCtx->tryToBind();
        }
        if (pCtx->m_state == CConnection::CONNECTION_STATE_BIND_OK)
        {
#ifdef CV_DEBUG_CONNECT
            pCtx->showConnectionContext("connect_thread", "bind OK, try to accept  ");
#endif
            CConnection::CONNECTION_STATE eTempBrowserState = CConnection::CONNECTION_STATE_INIT;
            pCtx->tryToAccept();
        }
        usleep(2000 * 1000);
    }
    pthread_exit(NULL);
}

void* listener_thread(void* pVoid)
{
    CTextConnection* pCtx = (CTextConnection*) pVoid;
    CMessageFromClient messageFromClient;
    char rcvBuffer[1024];
    int bytesRead;
    std::string sMsg;
    while (!g_isShutdown)
    {
        try
        {
            if (pCtx->m_state != CConnection::CONNECTION_STATE_ACCEPT_OK)
            {
                usleep(2000 * 1000);
            }
            else
            {
                rcvBuffer[0] = 0;
                bytesRead = pCtx->readClient(rcvBuffer, 1024);
                if (bytesRead > 0)
                {
                    messageFromClient.initFromText(rcvBuffer, 1, (int)CLedStrip::NUMBER_OF_LED_STRIP_MODES);
                    if (!messageFromClient.m_commandFromClient)
                    {
                        sMsg = "listener_thread received an unrecognized command = \"";
                        sMsg += rcvBuffer;
                        sMsg += "\"\n";
                        dbgMsg_s(sMsg);
                    }
                    else
                    {
                        pCtx->m_ledStrip.setMode((CLedStrip::LED_STRIP_MODE)messageFromClient.m_commandFromClient);
                    }
                }
            }
        }
        catch (...)
        {
        }
    }

    pthread_exit(NULL);
}

void* text_server_thread(void* pVoid)
{
    int iRet = 0;
    std::string sMsg;
    CFrameGrinder* pFrameGrinder = (CFrameGrinder*) pVoid;
    CVideoFrame* pFrame = 0;
    while (!g_isShutdown)
    {
        if (pFrameGrinder->safeBlockingRemoveHead(&pFrame, CVideoFrame::FRAME_QUEUE_WAIT_FOR_TEXT_CLIENT))
        {
            /*
            if (pFrameGrinder->m_connectionServer.isTextConnectionReadyToReceive())
            {
                pFrame->m_targetInfo.initFormattedTextFromTargetInfo();
                sMsg = pFrame->m_targetInfo.displayText();
                sMsg += ",";
                sMsg += static_textConnection.m_ledStrip.displayText();
                sMsg += "\n";
                iRet = static_textConnection.writeClient((char*) sMsg.c_str(), sMsg.size());
                fflush(NULL);
                pFrameGrinder->m_testMonitor.m_nTasksDone[CTestMonitor::TASK_DONE_TEXT]++;
            }
             */
            
            
            pFrameGrinder->safeAddTail(pFrame, CVideoFrame::FRAME_QUEUE_WAIT_FOR_BROWSER_CLIENT);
        }
    }
}

void* browser_server_thread(void* pVoid)
{
    context* pcontext = &cams[0];  // All code not relating to our camera in and http out has been removed
    static int iCount = 0;
    unsigned int i = 0;
    CFrameGrinder* pFrameGrinder = (CFrameGrinder*) pVoid;
    CVideoFrame* pFrame = 0;
    char head[1024];
 
    std::vector<unsigned char> buf;
    std::vector<int> qualityType;
    qualityType.push_back(CV_IMWRITE_JPEG_QUALITY);
    qualityType.push_back(50);

    while (!g_isShutdown)
    {
        if (pFrameGrinder->safeBlockingRemoveHead(&pFrame, CVideoFrame::FRAME_QUEUE_WAIT_FOR_BROWSER_CLIENT))
        {            
           /* copy JPG picture to global buffer */
            pthread_mutex_lock(&pglobal->in[pcontext->id].db);
            
            pFrame->annotate();
             
            cv::imencode(".jpg", pFrame->m_frame, buf, qualityType);  

            DBG("copying frame from input: %d\n", (int)pcontext->id);
            pglobal->in[pcontext->id].size = memcpy_picture(pglobal->in[pcontext->id].buf, buf.data(), buf.size());


            /* copy this frame's timestamp to user space */
            pglobal->in[pcontext->id].timestamp = pcontext->videoIn->buf.timestamp;

            /* signal fresh_frame */
            pthread_cond_broadcast(&pglobal->in[pcontext->id].db_update);
            pthread_mutex_unlock(&pglobal->in[pcontext->id].db);

            pFrameGrinder->m_testMonitor.m_nTasksDone[CTestMonitor::TASK_DONE_BROWSER]++;

            pFrameGrinder->m_testMonitor.monitorQueueTimesBeforeReturnToFreeQueue(pFrame, pFrameGrinder);
            pFrameGrinder->safeAddTail(pFrame, CVideoFrame::FRAME_QUEUE_FREE);
        }
    }
}

void CConnectionServer::init(CFrameGrinder* pFrameGrinder)
{
    dbgInit();
    static_browserConnection.init(7777, (char*) "browser");

    // Port 1180 is legal to use per FRC rules. The port is bidirectional but usually used to relay camera 
    // video from the cRIO to the driver station
    static_textConnection.init(5801, (char*) "text");

    m_old_pipeHandler = signal(SIGPIPE, sigPipeHandler);
    if (m_old_pipeHandler == SIG_ERR)
    {
        dbgMsg_s("We have an error\n");
    }

    m_old_ctrlcHandler = signal(SIGINT, sigCtrlCHandler);
    if (m_old_ctrlcHandler == SIG_ERR)
    {
        dbgMsg_s("We have an error\n");
    }
    int iRet = pthread_create(&static_browserConnection.m_threadSender, NULL, connect_thread, &static_browserConnection);
    if (iRet != 0)
    {
        dbgMsg_s("We have an error\n");
    }
    iRet = pthread_create(&static_textConnection.m_threadSender, NULL, connect_thread, &static_textConnection);
    if (iRet != 0)
    {
        dbgMsg_s("We have an error\n");
    }
    iRet = pthread_create(&static_textConnection.m_threadListener, NULL, listener_thread, &static_textConnection);
    if (iRet != 0)
    {
        dbgMsg_s("We have an error\n");
    }
    iRet = pthread_create(&m_textServerThread, NULL, text_server_thread, pFrameGrinder);
    if (iRet != 0)
    {
        dbgMsg_s("We have an error\n");
    }
    iRet = pthread_create(&m_browserServerThread, NULL, browser_server_thread, pFrameGrinder);
    if (iRet != 0)
    {

        dbgMsg_s("We have an error\n");
    }
}

bool CConnectionServer::isTextConnectionReadyToReceive() const
{

    return (static_textConnection.m_state == CConnection::CONNECTION_STATE_ACCEPT_OK);
}

bool CConnectionServer::isBrowserConnectionReadyToReceive() const
{
    return (static_browserConnection.m_state == CConnection::CONNECTION_STATE_ACCEPT_OK);
}
