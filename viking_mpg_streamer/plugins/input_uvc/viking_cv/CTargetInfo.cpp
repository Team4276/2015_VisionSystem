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


#include <string>
#include <stdio.h>
#include "opencv2/core/core.hpp"

#include "CToteRectangle.h"
#include "CTargetInfo.h"

CTargetInfo::CTargetInfo()
{
    init();
}

CTargetInfo::CTargetInfo(const CTargetInfo& orig)
{
    init();
}

CTargetInfo::~CTargetInfo()
{
    init();
}

void CTargetInfo::init()
{
    m_targetInfoText = "";
    m_timeSinceLastCameraFrameMilliseconds = 0;
    m_timeLatencyThisCameraFrameMilliseconds = 0;
    m_isGrayToteFound = 0;
    m_toteDirectionDegrees = -999.0;
    m_toteAngleDegrees = -999.0;
    m_distanceToToteCenterInches = -1.0;
}

void CTargetInfo::updateTargetInfo(
        int timeSinceLastCameraFrameMilliseconds,
        int timeLatencyThisCameraFrameMilliseconds,
        bool isGrayToteFound,
        float toteDirectionDegrees,
        float toteAngleDegrees,
        float distanceToToteCenterInches)
{
    init();

    m_timeSinceLastCameraFrameMilliseconds = timeSinceLastCameraFrameMilliseconds;
    m_timeLatencyThisCameraFrameMilliseconds = timeLatencyThisCameraFrameMilliseconds;

    m_isGrayToteFound = isGrayToteFound;

    if (isGrayToteFound)
    {
        m_toteDirectionDegrees = toteDirectionDegrees;
        m_toteAngleDegrees = toteAngleDegrees;
        m_distanceToToteCenterInches = distanceToToteCenterInches;
    }
    else
    {
        m_toteDirectionDegrees = -999.0;
        m_toteAngleDegrees = -999.0;
        m_distanceToToteCenterInches = -1.0;
    }
}

void CTargetInfo::initTargetInfoFromText(const std::string& targetInfoText)
{
    // Transmit on;y - no need for this conversion 
}

void CTargetInfo::initFormattedTextFromTargetInfo()
{
    char buf[128];
    int iTemp1 = (int) (m_toteDirectionDegrees * 10.0);
    int iTemp2 = (int) (m_toteAngleDegrees * 10.0);
    int iTemp3 = (int) (m_distanceToToteCenterInches * 10.0);

    // Format text for transmission to the cRio
    sprintf(buf, "%d,%d,%d,%d,%d,%d",
            m_timeSinceLastCameraFrameMilliseconds,
            m_timeLatencyThisCameraFrameMilliseconds,
            m_isGrayToteFound,
            iTemp1,
            iTemp2,
            iTemp3);
    m_targetInfoText = buf;
}
