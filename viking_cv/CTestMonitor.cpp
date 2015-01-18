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
#include <signal.h>
#include <time.h> 
#include <dirent.h> 
#include <sys/stat.h>
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

CTestMonitor::CTestMonitor()
{
    init();
}

CTestMonitor::CTestMonitor(const CTestMonitor& orig)
{
}

CTestMonitor::~CTestMonitor()
{
    if (m_isVideoRecording)
    {
        m_outVideo.release();
    }
}

void CTestMonitor::init()
{
    time_t rawtime;
    struct tm * timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    m_isMonitorEnabled = true; // if 'false' nothing is saved 
    m_isPrintEnabled = true; // if 'false' nothing is output to the console

    m_nNextFile = 0;
    m_nMaxFileNumber = 1000;

    m_sBaseFileName = numberToText00(timeinfo->tm_year + 1900);
    m_sBaseFileName += numberToText00(timeinfo->tm_mon + 1);
    m_sBaseFileName += numberToText00(timeinfo->tm_mday + 1);
    m_sBaseFileName += "-";
    m_sBaseFileName += numberToText00(timeinfo->tm_hour);
    m_sBaseFileName += "-";
    m_sBaseFileName += numberToText00(timeinfo->tm_min);
    m_sBaseFileName += "-";
    m_sBaseFileName += numberToText00(timeinfo->tm_sec);

    m_sLogFolder = "/home/";
    m_sLogFolder += HOME_NAME;
    m_sLogFolder += "/log";
    m_sLogVideoFolder = "/home/";
    m_sLogVideoFolder += HOME_NAME;
    m_sLogVideoFolder += "/logVideo";
    m_nNextFile = getNextFileNumber(m_sLogFolder);


    m_nIntervalisGrayToteFound = 0;
    m_avgGrayToteRectangle.init();
    m_nCountTestInterval = 0;
    memset(&m_avgElapsedSeconds, 0, sizeof (double)*NUMBER_OF_TIME_IN_TASK);
    memset(&m_savedElapsedSeconds, 0, sizeof (double)*NUMBER_OF_TIME_IN_TASK);
    m_avgTimeBetweenCameraFramesMilliseconds = 0;
    m_avgLatencyForProcessingFrameMilliseconds = 0;
}

void CTestMonitor::initVideo(int framesPerSec, unsigned int height, unsigned int width, int codec)
{
    m_isVideoRecording = false; //enableVideoCollection(true, framesPerSec, height, width, codec);
}

std::string CTestMonitor::numberToText(unsigned int n)
{
    char buf[128];
    sprintf(buf, "%d", n);
    return buf;
}

std::string CTestMonitor::numberToText00(unsigned int n)
{
    char buf[128];
    sprintf(buf, "%02d", n);
    return buf;
}

std::string CTestMonitor::numberToText0000(unsigned int n)
{
    char buf[128];
    sprintf(buf, "%04d", n);
    return buf;
}

unsigned int CTestMonitor::getNextFileNumber(const std::string& sFolderPath)
{
    unsigned int uiRet = 0;
    bool bFound = false;
    std::string sTempFilePath;
    std::string sNewestFileName;
    time_t newestTime = {0};
    struct stat sb;
    DIR *d;
    struct dirent *dir;
    d = opendir(m_sLogFolder.c_str());
    if (d)
    {
        while ((dir = readdir(d)) != NULL)
        {
            if (*dir->d_name != 'l')
            {
                sTempFilePath = m_sLogFolder;
                sTempFilePath += "/";
                sTempFilePath += dir->d_name;
                if (stat(sTempFilePath.c_str(), &sb) != -1)
                {
                    if (newestTime < sb.st_mtim.tv_sec)
                    {
                        bFound = true;
                        newestTime = sb.st_mtim.tv_sec;
                        sNewestFileName = dir->d_name;
                    }
                }
            }
        }
        closedir(d);
    }
    if (bFound)
    {
        if (sNewestFileName.size() > 4)
        {
            char buf[128];
            strcpy(buf, sNewestFileName.c_str());
            buf[4] = 0;
            uiRet = atoi(buf);
            if (uiRet >= m_nMaxFileNumber)
            {
                uiRet = 0;
            }
        }
    }
    return uiRet;
}

std::string CTestMonitor::getNextFilePath(const std::string& sFolderPath)
{
    std::string sRet = sFolderPath;
    sRet += "/";
    sRet += numberToText0000(m_nNextFile++);
    sRet += "-";
    sRet += m_sBaseFileName;
    if (m_nNextFile >= m_nMaxFileNumber)
    {
        m_nNextFile = 0;
    }
    return sRet;
}

std::string CTestMonitor::getLogFilePath()
{
    std::string sRet = m_sLogFolder;
    sRet += "/log-";
    sRet += m_sBaseFileName;
    sRet += ".txt";
    return sRet;
}

void CTestMonitor::deleteFileByNumberIfExists(unsigned int nFile, const std::string& sFolderPath)
{
    std::string sCmd = "rm -f ";
    sCmd += sFolderPath;
    sCmd += "/";
    sCmd += numberToText0000(nFile);
    sCmd += "*.*";
    int iRet = system(sCmd.c_str());
}

bool CTestMonitor::logWrite(const std::string& sLine)
{
    if (m_isPrintEnabled)
    {
        dbgMsg_s(sLine);
    }
    if (m_isMonitorEnabled)
    {
        FILE* fp = NULL;
        std::string sFilePath = getLogFilePath();
        if ((fp = fopen(sFilePath.c_str(), "a")) == NULL)
        {
            dbgMsg_s("Problem opening log file\n");
            return false;
        }
        unsigned int bytesWritten = fwrite(sLine.c_str(), sizeof (char), sLine.size(), fp);
        fclose(fp);
    }
    return true;
}

bool CTestMonitor::enableVideoCollection(bool bEnable, int framesPerSec, unsigned int height, unsigned int width, int codec)
{
    int nColor = 1;
    cv::Size mySize = cv::Size(width, height);

    std::string sFilePath = getNextFilePath(m_sLogVideoFolder);
    sFilePath += ".avi";

    //make output video file  
    m_outVideo.open(sFilePath, codec, framesPerSec, mySize, nColor);
    if (!m_outVideo.isOpened())
    {
        return false;
    }
    return true;
}

bool CTestMonitor::saveVideoFrame(cv::Mat frame)
{
    if (m_isMonitorEnabled && m_isVideoRecording)
    {
        m_outVideo << frame;
    }
}

bool CTestMonitor::saveFrameToJpeg(cv::Mat frame)
{
    if (m_isMonitorEnabled)
    {
        std::vector<int> qualityType;
        qualityType.push_back(CV_IMWRITE_JPEG_QUALITY);
        qualityType.push_back(90);

        deleteFileByNumberIfExists(m_nNextFile, m_sLogFolder);
        std::string sFilePath = getNextFilePath(m_sLogFolder);
        sFilePath += ".jpg";
        cv::imwrite(sFilePath.c_str(), frame, qualityType);
    }
}

void CTestMonitor::getTicks(struct timespec* pTime)
{
    if (clock_gettime(CLOCK_MONOTONIC, pTime) != 0)
    {
        dbgMsg_s("Problem getting timestamp\n");
    }
}

double CTestMonitor::getDeltaTimeSeconds(struct timespec timeStart, struct timespec timeEnd)
{
    double dTemp = getDeltaTimeMilliseconds(timeStart, timeEnd);
    dTemp /= 1000.0;
    return dTemp;
}

double CTestMonitor::getDeltaTimeMilliseconds(struct timespec timeStart, struct timespec timeEnd)
{
    double dTemp2 = (timeEnd.tv_sec - timeStart.tv_sec);
    double dTemp = (timeEnd.tv_nsec - timeStart.tv_nsec);
    if (dTemp < 0)
    {
        dTemp += 1000000000.0;
        if (dTemp2 > 0.0)
        {
            dTemp2 -= 1.0;
        }
    }
    dTemp /= (1000.0 * 1000.0);
    dTemp += (dTemp2 * 1000.0);
    return dTemp;
}

void CTestMonitor::monitorQueueTimesBeforeReturnToFreeQueue(CVideoFrame* pFrame, CFrameGrinder* pFrameGrinder)
{
#ifdef DISPLAY_CALIBRATION_INFO
    return;
#endif

    std::string sLine;
    static struct timespec timeAtStartOfInterval = {0};
    std::string sMsg;
    unsigned int i = 0;
    double timeIntervalSeconds = 0.0;
    getTicks(&pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_FREE]);
    if (m_nTasksDone[TASK_DONE_CAMERA] >= NUMBER_OF_SAMPLES_PER_TEST_INTERVAL)
    {
        timeIntervalSeconds = getDeltaTimeSeconds(
                timeAtStartOfInterval,
                pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_FREE]);
        timeAtStartOfInterval = pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_FREE];
        if (m_nIntervalisGrayToteFound == 0)
        {
            sLine += "Gray rectangle not found\n";
        }
        else
        {
            m_avgGrayToteRectangle.m_ptCenter.x /= m_nIntervalisGrayToteFound;
            m_avgGrayToteRectangle.m_ptCenter.y /= m_nIntervalisGrayToteFound;
            m_avgGrayToteRectangle.m_radius /= m_nIntervalisGrayToteFound;
            sLine += "Gray rectangle (";
            sLine += numberToText(m_nIntervalisGrayToteFound);
            sLine += " in this interval) avg ";
            sLine += m_avgGrayToteRectangle.displayText();
            sLine += "\n";
        }

        for (i = 0; i < NUMBER_OF_TIME_IN_TASK; i++)
        {
            m_avgElapsedSeconds[i] /= NUMBER_OF_SAMPLES_PER_TEST_INTERVAL;
        }
        m_avgTimeBetweenCameraFramesMilliseconds /= NUMBER_OF_SAMPLES_PER_TEST_INTERVAL;
        m_avgLatencyForProcessingFrameMilliseconds /= NUMBER_OF_SAMPLES_PER_TEST_INTERVAL;
        memcpy(&m_savedElapsedSeconds, &m_avgElapsedSeconds, sizeof (double)*NUMBER_OF_TIME_IN_TASK);
        sLine += displayQueueTimes(timeIntervalSeconds, pFrameGrinder);

        logWrite(sLine);
        sLine = "";

        for (i = 0; i < NUMBER_OF_TASK_DONE_TYPES; i++)
        {
            m_nTasksDone[i] = 0;
        }

        m_nIntervalisGrayToteFound = 0;
        m_avgGrayToteRectangle.init();

        m_nCountTestInterval = 0;
        memset(&m_avgElapsedSeconds, 0, sizeof (unsigned int)*NUMBER_OF_TIME_IN_TASK);
        m_avgTimeBetweenCameraFramesMilliseconds = 0;
        m_avgLatencyForProcessingFrameMilliseconds = 0;

        // 'true' is returned when the end of the interval is reached
        for (i = 0; i < CVideoFrame::NUMBER_OF_FRAME_QUEUES; i++)
        {
            pFrameGrinder->m_frameQueueList[i].m_droppedFrames = 0;
        }
    }
    if (pFrame->m_targetInfo.isGrayToteFound())
    {
        m_nIntervalisGrayToteFound++;
        m_avgGrayToteRectangle.m_ptCenter.x += pFrame->m_toteRectangleGray.m_ptCenter.x;
        m_avgGrayToteRectangle.m_ptCenter.y += pFrame->m_toteRectangleGray.m_ptCenter.y;
        m_avgGrayToteRectangle.m_radius += pFrame->m_toteRectangleGray.m_radius;
    }
    m_avgElapsedSeconds[TIME_IN_TASK_CAMERA] += getDeltaTimeSeconds(
            pFrame->m_timeRemovedFromQueue[(int) CVideoFrame::FRAME_QUEUE_FREE], // earlier time
            pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT]); // later time

    m_avgElapsedSeconds[TIME_IN_TASK_WAIT_FOR_BLOB_DETECT] += getDeltaTimeSeconds(
            pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT],
            pFrame->m_timeRemovedFromQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT]);

    m_avgElapsedSeconds[TIME_IN_TASK_BLOB_DETECT] += getDeltaTimeSeconds(
            pFrame->m_timeRemovedFromQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT],
            pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_TEXT_CLIENT]);

    m_avgElapsedSeconds[TIME_IN_TASK_WAIT_FOR_TEXT_CLIENT] += getDeltaTimeSeconds(
            pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_TEXT_CLIENT],
            pFrame->m_timeRemovedFromQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_TEXT_CLIENT]);

    m_avgElapsedSeconds[TIME_IN_TASK_TEXT_CLIENT] += getDeltaTimeSeconds(
            pFrame->m_timeRemovedFromQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_TEXT_CLIENT],
            pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BROWSER_CLIENT]);

    m_avgElapsedSeconds[TIME_IN_TASK_WAIT_FOR_BROWSER_CLIENT] += getDeltaTimeSeconds(
            pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BROWSER_CLIENT],
            pFrame->m_timeRemovedFromQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BROWSER_CLIENT]);

    m_avgElapsedSeconds[TIME_IN_TASK_BROWSER_CLIENT] += getDeltaTimeSeconds(
            pFrame->m_timeRemovedFromQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BROWSER_CLIENT],
            pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_FREE]);

    m_avgElapsedSeconds[TIME_TOTAL_CAMDONE_TO_TEXTDONE] += getDeltaTimeSeconds(
            pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT],
            pFrame->m_timeAddedToQueue[(int) CVideoFrame::FRAME_QUEUE_WAIT_FOR_BROWSER_CLIENT]);
    m_avgTimeBetweenCameraFramesMilliseconds += pFrame->m_targetInfo.getTimeSinceLastCameraFrameMilliseconds();
    m_avgLatencyForProcessingFrameMilliseconds += pFrame->m_targetInfo.getTimeLatencyThisCameraFrameMilliseconds();
}

void CTestMonitor::padString(std::string& str, int desiredLength)
{
    while (str.size() < desiredLength)
    {

        str += " ";
    }
}

std::string CTestMonitor::displayQueueTimes(double timeIntervalSeconds, CFrameGrinder* pFrameGrinder) const
{
    double dTemp;
    char buf[256];
    static std::string initTitles[] = {"Camera", "BlobDetect", "Text", "Browser", "Total", " "};
    std::string sLine, sRet;

    sLine = "";
    unsigned int i = 0;
    unsigned int nColSize = 20;
    while (0 != initTitles[i].compare(" "))
    {
        padString(sLine, i * nColSize);
        sLine += initTitles[i];
        i++;
    }
    sRet += sLine;
    sRet += "\n";

    sLine = "";
    nColSize = 10;
    for (i = 0; i < NUMBER_OF_TIME_IN_TASK; i++)
    {
        padString(sLine, i * nColSize);
        dTemp = m_savedElapsedSeconds[i];
        dTemp *= 1000.0;
        sprintf(buf, "%0.2fms", dTemp);
        sLine += buf;
    }
    sRet += sLine;
    sRet += "\n";

    sLine = "Done: ";
    nColSize = 20;
    for (i = 0; i < NUMBER_OF_TASK_DONE_TYPES; i++)
    {
        padString(sLine, 10 + (i * nColSize));
        sprintf(buf, "%d", m_nTasksDone[i]);
        sLine += buf;
    }
    sRet += sLine;
    sRet += "\n";

    sLine = "Drop: ";
    nColSize = 20;
    for (i = 1; i < CVideoFrame::NUMBER_OF_FRAME_QUEUES - 1; i++)
    {
        padString(sLine, 10 + ((i - 1) * nColSize));
        sprintf(buf, "%d", pFrameGrinder->m_frameQueueList[i].m_droppedFrames);
        sLine += buf;
    }
    sRet += sLine;
    sRet += "\n";

    unsigned int framesPerSecTextComplete = 0;
    if (timeIntervalSeconds > 0)
    {
        framesPerSecTextComplete = m_nTasksDone[TASK_DONE_TEXT] / timeIntervalSeconds;
    }
    sprintf(buf, "viking_cv Version %d.%d.%d    Interval:  %02f sec", VERSION_YEAR, VERSION_INTERFACE, VERSION_BUILD, timeIntervalSeconds);
    sRet += buf;
    sRet += "\n";

    sprintf(buf, "avgTimeBetweenCameraFrames: %0.2fms    avgLatencyForProcessingFrame: %0.2fms", m_avgTimeBetweenCameraFramesMilliseconds, m_avgLatencyForProcessingFrameMilliseconds);
    sRet += buf;
    sRet += "\n\n";

    return sRet;
}

