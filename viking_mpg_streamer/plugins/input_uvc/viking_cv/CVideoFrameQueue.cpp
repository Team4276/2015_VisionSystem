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
#include <signal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "CToteRectangle.h"
#include "CTargetInfo.h"
#include "CVideoFrame.h"
#include "CVideoFrameQueue.h"
#include "CConnectionServer.h"
#include "CGpioLed.h"
#include "CToteDetector.h"
#include "CTestMonitor.h"
#include "CFrameGrinder.h"
#include "dbgMsg.h"

CVideoFrameQueue::CVideoFrameQueue()
{
    m_eQueueType = CVideoFrame::FRAME_QUEUE_TYPE_UNKNOWN;
    memset(&m_condThread, 0, sizeof (pthread_cond_t));
}

CVideoFrameQueue::~CVideoFrameQueue()
{
    while (m_queue.size() != 0)
    {
        CVideoFrame* pFrame = m_queue.back();
        m_queue.pop_back();
        delete pFrame;
    }
}

void CVideoFrameQueue::init(CVideoFrame::FRAME_QUEUE_TYPE eQueueType)
{
    m_eQueueType = eQueueType;
}

void CVideoFrameQueue::addTail(CVideoFrame* pFrame, pthread_mutex_t& mutexQueue)
{
    pthread_mutex_lock(&mutexQueue);
    nolockAddTail(pFrame);
    pthread_mutex_unlock(&mutexQueue);
    int rc = pthread_cond_broadcast(&m_condThread);
}

bool CVideoFrameQueue::removeHead(CVideoFrame** ppFrame, pthread_mutex_t& mutexQueue)
{
    pthread_mutex_lock(&mutexQueue);
    bool bRet = nolockRemoveHead(ppFrame);
    pthread_mutex_unlock(&mutexQueue);
    return bRet;
}

bool CVideoFrameQueue::blockingRemoveHead(CVideoFrame** ppFrame, pthread_mutex_t& mutexQueue)
{
    if (!removeHead(ppFrame, mutexQueue))
    {
        pthread_mutex_lock(&m_mutexThread);
        int rc = pthread_cond_wait(&m_condThread, &m_mutexThread);
        pthread_mutex_unlock(&m_mutexThread);
        if (!removeHead(ppFrame, mutexQueue))
        {
            dbgMsg_s("Woke up after blocking remove, but nothing in the queue\n");
            *ppFrame = 0;
            return false;
        }
    }
    return true;
}

void CVideoFrameQueue::nolockAddTail(CVideoFrame* pFrame)
{
    CTestMonitor::getTicks(&pFrame->m_timeAddedToQueue[(int) m_eQueueType]);
    m_queue.push_back(pFrame);
}

bool CVideoFrameQueue::nolockRemoveHead(CVideoFrame** ppFrame)
{
    if (m_queue.size() == 0)
    {
        *ppFrame = 0;
        return false;
    }
    *ppFrame = m_queue.back();
    m_queue.pop_back();
    CTestMonitor::getTicks(&((*ppFrame)->m_timeRemovedFromQueue[(int) m_eQueueType]));
    return true;
}
