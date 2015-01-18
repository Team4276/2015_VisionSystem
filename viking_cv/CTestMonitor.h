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

#ifndef CTESTMONITOR_H
#define	CTESTMONITOR_H

#define NUMBER_OF_SAMPLES_PER_TEST_INTERVAL 50 

class CTestMonitor
{
public:
    CTestMonitor();
    CTestMonitor(const CTestMonitor& orig);
    virtual ~CTestMonitor();

    void init();
    void initVideo(int framesPerSec, unsigned int height, unsigned int width, int codec);
    void monitorQueueTimesBeforeReturnToFreeQueue(CVideoFrame* pFrame, CFrameGrinder* pFrameGrinder);
    std::string displayQueueTimes(double timeIntervalSeconds, CFrameGrinder* pFrameGrinder) const;
    bool logWrite(const std::string& sLine);
    bool saveVideoFrame(cv::Mat frame);
    bool saveFrameToJpeg(cv::Mat frame);

    static void padString(std::string& str, int desiredLength);
    static double getDeltaTimeMilliseconds(struct timespec timeStart, struct timespec timeEnd);
    static double getDeltaTimeSeconds(struct timespec timeStart, struct timespec timeEnd);
    static void getTicks(struct timespec* pTime);

    typedef enum
    {
        TASK_DONE_CAMERA,
        TASK_DONE_BLOB_DETECT,
        TASK_DONE_TEXT,
        TASK_DONE_BROWSER,
        NUMBER_OF_TASK_DONE_TYPES
    } TASK_DONE_TYPE;

    // Running totals for current test interval
    unsigned int m_nTasksDone[NUMBER_OF_TASK_DONE_TYPES];

private:
    bool enableVideoCollection(bool bEnable, int framesPerSec, unsigned int height, unsigned int width, int codec);
    unsigned int getNextFileNumber(const std::string& sFolderPath);
    std::string getNextFilePath(const std::string& sFolderPath);
    std::string getLogFilePath();
    void deleteFileByNumberIfExists(unsigned int nFile, const std::string& sFolderPath);
    static std::string numberToText(unsigned int n);
    static std::string numberToText00(unsigned int n);
    static std::string numberToText0000(unsigned int n);


private:

    typedef enum
    {
        TIME_IN_TASK_CAMERA,
        TIME_IN_TASK_WAIT_FOR_BLOB_DETECT,
        TIME_IN_TASK_BLOB_DETECT,
        TIME_IN_TASK_WAIT_FOR_TEXT_CLIENT,
        TIME_IN_TASK_TEXT_CLIENT,
        TIME_IN_TASK_WAIT_FOR_BROWSER_CLIENT,
        TIME_IN_TASK_BROWSER_CLIENT,
        TIME_TOTAL_CAMDONE_TO_TEXTDONE,
        NUMBER_OF_TIME_IN_TASK
    } TIME_IN_TASK_TYPE;

    bool m_isMonitorEnabled; // if 'false' nothing is saved
    bool m_isPrintEnabled; // if 'false' nothing is output to the console
    bool m_isVideoRecording; // even if 'true' nothing is saved unless m_isMonitorEnabled is also 'true'

    unsigned int m_nNextFile;
    unsigned int m_nMaxFileNumber;
    std::string m_sBaseFileName;
    std::string m_sLogFolder;
    std::string m_sLogVideoFolder;
    cv::VideoWriter m_outVideo;

    // Running totals for current test interval
    unsigned int m_nIntervalisGrayToteFound;
    CToteRectangle m_avgGrayToteRectangle;
    double m_avgElapsedSeconds[NUMBER_OF_TIME_IN_TASK];
    double m_avgTimeBetweenCameraFramesMilliseconds;
    double m_avgLatencyForProcessingFrameMilliseconds;
    unsigned int m_nCountTestInterval;

    // Averages from last test interval
    double m_savedElapsedSeconds[NUMBER_OF_TIME_IN_TASK];

};

#endif	/* CTESTMONITOR_H */

