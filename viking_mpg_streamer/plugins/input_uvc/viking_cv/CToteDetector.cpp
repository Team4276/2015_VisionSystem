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
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

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
#include "viking_cv_version.h"


CToteDetector::CToteDetector()
{
}

CToteDetector::CToteDetector(const CToteDetector& orig)
{
}

CToteDetector::~CToteDetector()
{
}

void CToteDetector::init()
{
    m_tolerancePercentForRadius = 0.20;
}

void CToteDetector::detectBlobs(CVideoFrame * pFrame, CFrameGrinder* pFrameGrinder)
{
    try
    {
        static struct timespec timeLastCameraFrame = {0};
        static struct timespec timeNow = {0};
        cv::Mat img_hsv, gray_blob, binary_blob, binary_blob_8u;
        static int iCount = 0;

        int timeSinceLastCameraFrameMilliseconds = (int) CTestMonitor::getDeltaTimeMilliseconds(
                timeLastCameraFrame,
                pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT]);
        timeLastCameraFrame = pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT];
       

        // RBG is flawed as a way to filter based on color because the brightness is combined 
        // with the color info. 
        // Not so with HSV, where Hue and Saturation are maintained separately
        // OpenCV has a handy conversion from RGB to HSV
        cv::cvtColor(pFrame->m_frame, img_hsv, CV_BGR2HSV);

        // Filter out all but Gray hue
        
#ifdef COLORFILTER_ZOOKS_CLASSROOM_LIFECAM
        cv::inRange(img_hsv, cv::Scalar(30, 15, 32), cv::Scalar(80, 220, 220), gray_blob);
#endif
        
#ifdef COLORFILTER_MARINA_HALLWAY_LIFECAM
        cv::inRange(img_hsv, cv::Scalar(30, 0, 90), cv::Scalar(220, 80, 160), gray_blob);
#endif
            
        iCount++;
        if ((iCount % 17) == 0)
        {
            pFrameGrinder->m_testMonitor.saveFrameToJpeg(gray_blob);
        }
      

        // make a kernel of 4x4, all 1's
        cv::Mat kernel = cv::Mat::ones(cv::Size(2, 2), CV_8U);
        cv::erode(gray_blob, gray_blob, kernel);

         //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
        std::vector<std::vector<cv::Point> > grayContours;
        try
        {
            cv::findContours(gray_blob, grayContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );
        }
        catch( cv::Exception& e )
        {
            const char* err_msg = e.what();
            dbgMsg_s(err_msg);
        }
        
        //const cv::Scalar colorGreen = cv::Scalar(0, 255, 0);
        //for (int i = 0; i < grayContours.size(); i++){
        //    cv::drawContours(pFrame->m_frame, grayContours, i, colorGreen, 1, 8);
        //}
        //pFrameGrinder->m_testMonitor.saveFrameToJpeg(pFrame->m_frame);

        CToteRectangle bestToteRectangleGray;
        float angleToBlueToteDegrees = 0.0;
        float offsetFromCenterlineToToteCenterToteFeet = 0.0;
        bool isGrayToteFound = false;
#ifdef DETECT_LARGEST_BLOB_NO_FILTER_BASED_ON_SIZE
        isGrayToteFound = filterContoursToFindLargestBlob(grayContours, bestToteRectangleGray, angleToBlueToteDegrees, offsetFromCenterlineToToteCenterToteFeet);

#ifdef DISPLAY_CALIBRATION_INFO
        printf("viking_cv version %d.%d.%d", VERSION_YEAR, VERSION_INTERFACE, VERSION_BUILD);
        if (isGrayToteFound)
        {
            printf("   NearFar_X: %d    LeftRight_Y: %d    Radius  %02f\r",
                    (int) bestToteRectangleGray.m_ptCenter.x,
                    (int) bestToteRectangleGray.m_ptCenter.y,
                    bestToteRectangleGray.m_radius);
        }
        else
        {
            printf("   Gray rectangle *NOT* found\r");
        }
#endif

#else
        isGrayToteFound = filterContoursToFindToteBySize(grayContours, bestToteRectangleGray, angleToBlueToteDegrees, offsetFromCenterlineToToteCenterToteFeet);
#endif

        CTestMonitor::getTicks(&timeNow);
        int timeLatencyThisCameraFrameMilliseconds = (int) CTestMonitor::getDeltaTimeMilliseconds(
                pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT],
                timeNow);

        pFrame->m_targetInfo.updateTargetInfo(
                timeSinceLastCameraFrameMilliseconds, timeLatencyThisCameraFrameMilliseconds, 
                isGrayToteFound, angleToBlueToteDegrees, offsetFromCenterlineToToteCenterToteFeet);

        pFrame->updateAnnotationInfo(bestToteRectangleGray);

        m_gpioLed.setGreenLED(isGrayToteFound, pFrame->m_timeRemovedFromQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT]);
    }
    catch (...)
    {
    }
}

bool CToteDetector::filterContoursToFindLargestBlob(
        const std::vector<std::vector<cv::Point> >& listContours,
        CToteRectangle& bestToteRectangle,
        float& toteAngleDegrees,
        float& distanceToToteFeet)
{
    bool isToteFound = false;
    bestToteRectangle.init();
    toteAngleDegrees = -999.0;
    distanceToToteFeet = -1.0;

    unsigned int i = 0;
    CToteRectangle tempToteRectangle;
    double aspectRatio, diff, area, predict, max;
    cv::RotatedRect tempRect, vertRect, horizRect;
    std::vector<CToteRectangle> listPossibleToteRectangle;
    std::vector<cv::Point> contours_poly;
    max = 0.0;
    for (i = 0; i < listContours.size(); i++)
    {
        tempRect = cv::minAreaRect(cv::Mat(listContours[i]));
        area = tempRect.size.width * tempRect.size.height;
        //if ((area > 10000.0) && (area < 15000.0))
        {
            if(max < area)
            {
                max = area;
                isToteFound = true;
                bestToteRectangle = tempRect;
            }
        }
    }
    if (isToteFound)
    {
        //toteAngleDegrees = m_lookupTable[(int) bestToteRectangle.m_ptCenter.x][(int) bestToteRectangle.m_ptCenter.y].toteAngleDegrees;
        //distanceToToteFeet = m_lookupTable[(int) bestToteRectangle.m_ptCenter.x][(int) bestToteRectangle.m_ptCenter.y].distanceToToteFeet;
    }
    return isToteFound;
}
