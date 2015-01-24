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
        cv::Mat img_hsv, red_blob, blue_blob, green_blob, dstA, dstB;
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

        // Filter out all but Blue hue
        cv::inRange(img_hsv, cv::Scalar(100, 75, 75), cv::Scalar(105, 255, 255), blue_blob);
            
        pFrameGrinder->m_testMonitor.saveFrameToJpeg(blue_blob);

        // Filter out all but red
        cv::inRange(img_hsv, cv::Scalar(0, 75, 75), cv::Scalar(5, 255, 255), dstA);
        cv::inRange(img_hsv, cv::Scalar(175, 75, 75), cv::Scalar(180, 255, 255), dstB);
        cv::bitwise_or(dstA, dstB, red_blob);

        // Filter out all but Green hue
        //cv::inRange(img_hsv, cv::Scalar(74, 50, 40), cv::Scalar(94, 200, 245), green_blob);
        cv::inRange(img_hsv, cv::Scalar(74, 50, 40), cv::Scalar(94, 200, 245), green_blob);

        // make a kernel of 4x4, all 1's
        cv::Mat kernel = cv::Mat::ones(cv::Size(2, 2), CV_8U);
        cv::erode(green_blob, green_blob, kernel);
        cv::erode(green_blob, green_blob, kernel);
        cv::erode(green_blob, green_blob, kernel);
        cv::dilate(green_blob, green_blob, kernel);
        cv::dilate(green_blob, green_blob, kernel);
        cv::dilate(green_blob, green_blob, kernel);
        ///cv::GaussianBlur(green_blob,green_blob,cv::Size(5,5),1.5);

        //cv::imshow("green_blob", green_blob);
        //cv::waitKey(0);
        iCount++;
        if ((iCount % 17) == 0)
        {
            pFrameGrinder->m_testMonitor.saveFrameToJpeg(green_blob);
        }


        //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
        std::vector<std::vector<cv::Point> > blueContours, redContours, greenContours;
        cv::findContours(blue_blob, blueContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        cv::findContours(red_blob, redContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        cv::findContours(green_blob, greenContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

        CToteRectangle bestToteRectangleBlue;
        float angleToBlueToteDegrees = 0.0;
        float offsetFromCenterlineToToteCenterToteFeet = 0.0;
        bool isGrayToteFound = false;
#ifdef DETECT_LARGEST_BLOB_NO_FILTER_BASED_ON_SIZE
        isGrayToteFound = filterContoursToFindLargestBlob(blueContours, bestToteRectangleBlue, angleToBlueToteDegrees, offsetFromCenterlineToToteCenterToteFeet);

        isGrayToteFound = true;
#ifdef DISPLAY_CALIBRATION_INFO
        printf("viking_cv version %d.%d.%d", VERSION_YEAR, VERSION_INTERFACE, VERSION_BUILD);
        if (isGrayToteFound)
        {
            printf("   NearFar_X: %d    LeftRight_Y: %d    Radius  %02f\r",
                    (int) bestToteRectangleBlue.m_ptCenter.x,
                    (int) bestToteRectangleBlue.m_ptCenter.y,
                    bestToteRectangleBlue.m_radius);
        }
        else
        {
            printf("   Gray rectangle *NOT* found\r");
        }
#endif

#else
        isGrayToteFound = filterContoursToFindToteBySize(blueContours, bestToteRectangleBlue, angleToBlueToteDegrees, offsetFromCenterlineToToteCenterToteFeet);
#endif

        CToteRectangle bestToteRectangleRed;
        float angleToRedToteDegrees = 0.0;
        float distanceToRedToteFeet = 0.0;
        bool isRedFound = false;
#ifdef DETECT_LARGEST_BLOB_NO_FILTER_BASED_ON_SIZE
        isRedFound = filterContoursToFindLargestBlob(redContours, bestToteRectangleRed, angleToRedToteDegrees, distanceToRedToteFeet);

#else    
        isRedFound = filterContoursToFindToteBySize(redContours, bestToteRectangleRed, angleToRedToteDegrees, distanceToRedToteFeet);
#endif

        bool isRightGreenTargetLit = false;
        bool isLeftGreenTargetLit = false;
        double idealAspectRatioStatic = 32.0 / 4.0; // Vertical target is 2' 8" x 4"
        double idealAspectRatioDynamic = 4.0 / 23.5; // Horizontal target is 4" x 1' 11.5"
        double aspectRatio, diff, area, predict;
        cv::RotatedRect tempRect, vertRect, horizRect;
        bool isFoundVertRect = false;
        bool isFoundHorizRect = false;
        for (int i = 0; i < greenContours.size(); i++)
        {
            tempRect = cv::minAreaRect(cv::Mat(greenContours[i]));
            area = tempRect.boundingRect().width * tempRect.boundingRect().height;
            if ((area > 100.0) && (area < 400))
            {
                // Test to see if width and height look like vertical (static) target
                // Note "width" is vertical due to orientation of camera
                if (tempRect.boundingRect().width > (tempRect.boundingRect().height * 2.5))
                {
                    isFoundVertRect = true;
                    vertRect = tempRect;
                }

                // Test to see if width and height look like horizontal (dynamic) target
                if (tempRect.boundingRect().height > (tempRect.boundingRect().width * 2.5))
                {
                    isFoundHorizRect = true;
                    horizRect = tempRect;
                }
            }
        }

        if ((!isFoundHorizRect) || (!isFoundVertRect))
        {
            isRightGreenTargetLit = false;
            isLeftGreenTargetLit = false;
        }
        else
        {
            // The dynamic target is to the left of both static targets or to the right of both static targets, so only need to test one of them
            // Note that due to camera on its side "X" is Near/Far and "Y" is Left/Right
            if (horizRect.center.y < vertRect.center.y)
            {
                isLeftGreenTargetLit = true;
            }
            else
            {
                isRightGreenTargetLit = true;
            }
        }

        CTestMonitor::getTicks(&timeNow);
        int timeLatencyThisCameraFrameMilliseconds = (int) CTestMonitor::getDeltaTimeMilliseconds(
                pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT],
                timeNow);

        pFrame->m_targetInfo.updateTargetInfo(
                timeSinceLastCameraFrameMilliseconds, timeLatencyThisCameraFrameMilliseconds, isRightGreenTargetLit, isLeftGreenTargetLit,
                isGrayToteFound, angleToBlueToteDegrees, offsetFromCenterlineToToteCenterToteFeet,
                isRedFound, angleToRedToteDegrees, distanceToRedToteFeet);

        pFrame->updateAnnotationInfo(
                bestToteRectangleBlue,
                bestToteRectangleRed);

        m_gpioLed.setGreenLED((isGrayToteFound || isRedFound), pFrame->m_timeRemovedFromQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT]);
    }
    catch (...)
    {
    }
}

bool CToteDetector::filterContoursToFindLargestBlob(
        const std::vector<std::vector<cv::Point> >& listContours,
        CToteRectangle& bestToteRectangle,
        float& angleToToteDegrees,
        float& distanceToToteFeet)
{
    bool isToteFound = false;
    bestToteRectangle.init();
    angleToToteDegrees = -999.0;
    distanceToToteFeet = -1.0;

    unsigned int i = 0;
    CToteRectangle tempToteRectangle;
    std::vector<CToteRectangle> listPossibleToteRectangle;
    std::vector<cv::Point> contours_poly;
    for (i = 0; i < listContours.size(); i++)
    {
        //approxPolyDP(cv::Mat(listContours[i]), contours_poly, 3, true);
        //minEnclosingCircle((cv::Mat) contours_poly, tempToteRectangle.m_ptCenter, tempToteRectangle.m_radius);
        if (bestToteRectangle.m_radius < tempToteRectangle.m_radius)
        {
            isToteFound = true;
            bestToteRectangle = tempToteRectangle;
        }
    }
    if (isToteFound)
    {
        //angleToToteDegrees = m_lookupTable[(int) bestToteRectangle.m_ptCenter.x][(int) bestToteRectangle.m_ptCenter.y].angleToToteDegrees;
        //distanceToToteFeet = m_lookupTable[(int) bestToteRectangle.m_ptCenter.x][(int) bestToteRectangle.m_ptCenter.y].distanceToToteFeet;
    }
    return isToteFound;
}

bool CToteDetector::filterContoursToFindToteBySize(
        const std::vector<std::vector<cv::Point> >& listContours,
        CToteRectangle& bestToteRectangle,
        float& angleToToteDegrees,
        float& distanceToToteFeet)
{
    bool isToteFound = false;
    bestToteRectangle.init();
    angleToToteDegrees = -999.0;
    distanceToToteFeet = -1.0;

    unsigned int i = 0;
    CToteRectangle tempToteRectangle;
    std::vector<CToteRectangle> listPossibleToteRectangle;
    std::vector<cv::Point> contours_poly;
    for (i = 0; i < listContours.size(); i++)
    {
        //approxPolyDP(cv::Mat(listContours[i]), contours_poly, 3, true);
        //minEnclosingCircle((cv::Mat) contours_poly, tempToteRectangle.m_ptCenter, tempToteRectangle.m_radius);
        if (!isTooSmallToBeATote(tempToteRectangle))
        {
            listPossibleToteRectangle.push_back(tempToteRectangle);
        }
    }
    if (listPossibleToteRectangle.size() == 1)
    {
        // Perfect - Found exactly one candidate 
        isToteFound = true;
        bestToteRectangle = listPossibleToteRectangle.at(0);
    }
    else if (listPossibleToteRectangle.size() > 1)
    {
        // Take first candidate that is about the right size
        for (i = 0; i < listPossibleToteRectangle.size(); i++)
        {
            if (!isTooBigToBeATote(listPossibleToteRectangle.at(i)))
            {
                isToteFound = true;
                bestToteRectangle = listPossibleToteRectangle.at(i);
            }
        }
    }
    if (isToteFound)
    {
        //angleToToteDegrees = m_lookupTable[(int) bestToteRectangle.m_ptCenter.x][(int) bestToteRectangle.m_ptCenter.y].angleToToteDegrees;
        //distanceToToteFeet = m_lookupTable[(int) bestToteRectangle.m_ptCenter.x][(int) bestToteRectangle.m_ptCenter.y].distanceToToteFeet;
    }
    return isToteFound;
}

bool CToteDetector::isTooSmallToBeATote(const CToteRectangle & toteRectangle) const
{
    unsigned int pixelX = (unsigned int) toteRectangle.m_ptCenter.x;
    unsigned int pixelY = (unsigned int) toteRectangle.m_ptCenter.y;
    float pixelRadius = 100.0;  //m_lookupTable[pixelX][pixelY].pixelRadius;

    return (toteRectangle.m_radius < ((1.0 - m_tolerancePercentForRadius) * pixelRadius));
}

bool CToteDetector::isTooBigToBeATote(const CToteRectangle & toteRectangle) const
{
    unsigned int pixelX = (unsigned int) toteRectangle.m_ptCenter.x;
    unsigned int pixelY = (unsigned int) toteRectangle.m_ptCenter.y;
    float pixelRadius = 100.0;  //m_lookupTable[pixelX][pixelY].pixelRadius;
    return (toteRectangle.m_radius > ((1.0 + m_tolerancePercentForRadius) * pixelRadius));
}
