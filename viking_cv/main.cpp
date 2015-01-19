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
#include <stdlib.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
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

CFrameGrinder frameGrinder;

// Global shutdown flag is set when user typed Ctrl-C
extern bool g_isShutdown;

int main(int, char**) {
    CVideoFrame* pFrame = NULL;

    frameGrinder.init();

#ifndef TEST_USE_JPEGS_NOT_CAMERA
    cv::VideoCapture camera;
#endif

    while (!g_isShutdown) {

#ifndef TEST_USE_JPEGS_NOT_CAMERA
        //open the video stream and make sure it's opened
        if ((!camera.open(0) || (!camera.isOpened()))) // "0" --> Use default camera (local USB webcam))
        {
            dbgMsg_d1("Error opening video stream or file, err = %d\n", errno);
            exit(0);
        } else
#endif
        {
#ifdef TEST_USE_JPEGS_NOT_CAMERA
            std::string sBasePath = "/home/";
            sBasePath += HOME_NAME;
            std::string sPath = sBasePath;
            sPath += "/IMG_0727.JPG";
            cv::Mat frame1 = cv::imread(sPath.c_str(), CV_LOAD_IMAGE_COLOR);
            sPath = sBasePath;
            sPath += "/IMG_0729.JPG";
            cv::Mat frame2 = cv::imread(sPath.c_str(), CV_LOAD_IMAGE_COLOR);
            if (frame2.empty()) {
                dbgMsg_s("Failed to read image data from a file\n");
            }
            int toggle = 0;

#else 
            camera.set(CV_CAP_PROP_FRAME_WIDTH, 424); //640    );  //1920); //640);
            camera.set(CV_CAP_PROP_FRAME_HEIGHT, 240); //380  // 1080); //360);
            camera.set(CV_CAP_PROP_FPS, 30);

            int codec = CV_FOURCC('M', 'J', 'P', 'G');
            unsigned int width = camera.get(CV_CAP_PROP_FRAME_WIDTH);
            unsigned int height = camera.get(CV_CAP_PROP_FRAME_HEIGHT);
            unsigned int fps = 10;
            frameGrinder.initVideo(fps, height, width, codec);
#endif
            while (!g_isShutdown) {
#ifdef TEST_USE_JPEGS_NOT_CAMERA
                usleep(10 * 1000);
#endif
                if (frameGrinder.safeGetFreeFrame(&pFrame)) {
#ifdef TEST_USE_JPEGS_NOT_CAMERA
                    if (toggle == 0) {
                        toggle = 1;
                        pFrame->m_frame = frame1.clone();
                    } else {
                        toggle = 0;
                        pFrame->m_frame = frame2.clone();
                    }
#else
                    camera >> pFrame->m_frame;
#endif
                    if (!pFrame->m_frame.empty()) {
                        frameGrinder.safeAddTail(pFrame, CVideoFrame::FRAME_QUEUE_WAIT_FOR_BLOB_DETECT);
                    } else {
                        dbgMsg_s("Frame is empty\n");
                        frameGrinder.safeAddTail(pFrame, CVideoFrame::FRAME_QUEUE_FREE);
                    }
                    frameGrinder.m_testMonitor.m_nTasksDone[CTestMonitor::TASK_DONE_CAMERA]++;
                }
            }
        }
    }
}
