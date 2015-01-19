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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "CToteRectangle.h"
#include "CTargetInfo.h"
#include "CVideoFrame.h"
#include "dbgMsg.h"

CVideoFrame::CVideoFrame()
{
    init();
}

CVideoFrame::~CVideoFrame()
{
}

void CVideoFrame::init()
{
    m_targetInfo.init();
    m_toteRectangleGray.init();
    memset(m_timeAddedToQueue, 0, sizeof (struct timespec)*CVideoFrame::NUMBER_OF_FRAME_QUEUES);
    memset(m_timeRemovedFromQueue, 0, sizeof (struct timespec)*CVideoFrame::NUMBER_OF_FRAME_QUEUES);
    m_outbuf.clear();
    m_params.clear();
}

void CVideoFrame::updateAnnotationInfo(const CToteRectangle& ToteRectangleGray)
{
    m_toteRectangleGray = ToteRectangleGray;
}

void CVideoFrame::annotate()
{
    const cv::Scalar colorOrange = cv::Scalar(0, 128, 255);

    if (m_targetInfo.isGrayToteFound())
    {
        cv::Point2f rect_points[4];
        m_toteRectangleGray.points(rect_points);
        for (int j = 0; j < 4; j++)
        {
            cv::line(m_frame, rect_points[j], rect_points[(j + 1) % 4], colorOrange, 3, 8);
        }
    }
}

void CVideoFrame::compress()
{
    m_outbuf.clear();
    m_params.clear();
    m_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    m_params.push_back(8);
    if (!cv::imencode(".jpg", m_frame, m_outbuf, m_params))
    {
        dbgMsg_s("Failed to encode frame to JPG format\n");
    }
}
