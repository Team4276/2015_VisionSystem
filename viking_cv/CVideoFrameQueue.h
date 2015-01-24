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


class CVideoFrameQueue
{
public:

    CVideoFrameQueue();
    ~CVideoFrameQueue();

    void init(CVideoFrame::FRAME_QUEUE_TYPE eQueueType);
    void addTail(CVideoFrame* pFrame, pthread_mutex_t& mutexQueue);
    bool removeHead(CVideoFrame** ppFrame, pthread_mutex_t& mutexQueue); // Returns 'false' if queue is empty
    bool blockingRemoveHead(CVideoFrame** ppFrame, pthread_mutex_t& mutexQueue); // Calling thread will sleep if nothing in the queue
    void nolockAddTail(CVideoFrame* pFrame);
    bool nolockRemoveHead(CVideoFrame** ppFrame); // Returns 'false' if queue is empty

    unsigned int size() const
    {
        return m_queue.size();
    }

public:
    unsigned int m_droppedFrames;

private:
    CVideoFrame::FRAME_QUEUE_TYPE m_eQueueType;
    std::vector<CVideoFrame*> m_queue;
    pthread_mutex_t m_mutexThread;
    pthread_mutexattr_t m_mutexattrThread;
    pthread_cond_t m_condThread; // Block thread when remove fails
};