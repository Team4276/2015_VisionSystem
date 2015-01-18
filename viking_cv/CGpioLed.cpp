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
#include <string.h>
#include <string>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
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
#include "dbgMsg.h"

CGpioLed::CGpioLed()
{
    //Don't adjust the state of the LED more often than this interval
    m_updateLedIntervalSeconds = 0.5;
}

CGpioLed::CGpioLed(const CGpioLed& orig)
{
    local_setGreenLED(false);
    m_enableStateCurrent = false;
    CTestMonitor::getTicks(&m_timeStamp);
    m_initOK = false;
    if (!local_setGreenLED(false))
    {
        dbgMsg_s("GPIO LED init failed\n");
    }
}

CGpioLed::~CGpioLed()
{
}

void CGpioLed::setGreenLED(bool turnLedOn, struct timespec timeStamp)
{
    if (turnLedOn != m_enableStateCurrent)
    {
        double elapsedSecs = CTestMonitor::getDeltaTimeSeconds(m_timeStamp, timeStamp);
        if (elapsedSecs > m_updateLedIntervalSeconds)
        {
            if (local_setGreenLED(turnLedOn))
            {
                m_enableStateCurrent = turnLedOn;
            }
            m_timeStamp = timeStamp; // Even if the attempt fails, don't try to do this more often than the specified interval
        }
    }
}

bool CGpioLed::local_initGreenLED()
{
    try
    {
        FILE* hLED = NULL;
        char setValue[16];

        m_initOK = false;

        // Export the GPIO pin    -- BeagleBone Black P9 pin 12 --> GPIO #60
        // Details:   BeagleBone Black pin 12 is gpio_1[28]
        //            There are 4 controllers for 32 GPIOs each, so (1*32)+28 = 60
        if ((hLED = fopen("/sys/class/gpio/export", "ab")) == NULL)
        {
            // This is normal when running on a non-BBB laptop - just ignore and continue
            return false;
        }
        strcpy(setValue, "60");
        fwrite(&setValue, sizeof (char), strlen(setValue), hLED);
        fflush(hLED);
        fclose(hLED);

        // Set the direction of the pin to "out"
        if ((hLED = fopen("/sys/class/gpio/gpio60/direction", "rb+")) == NULL)
        {
            dbgMsg_s("Problem opening handle to export GPIO pin 60\n");
            return false;
        }
        strcpy(setValue, "high");
        fwrite(&setValue, sizeof (char), strlen(setValue), hLED);
        fflush(hLED);
        fclose(hLED);

        m_initOK = true;
        return true;
    }
    catch (...)
    {
    }
    dbgMsg_s("local_initVisionOkLED() terminated unexpectedly\n");
    return false;
}

bool CGpioLed::local_setGreenLED(bool turnLedOn)
{
    if (!m_initOK)
    {
        if (!local_initGreenLED())
        {
            return false;
        }
    }
    try
    {
        FILE* hLED = NULL;
        char setValue[4];

        // Turn the LED on or off depending on the input
        if ((hLED = fopen("/sys/class/gpio/gpio60/value", "rb+")) == NULL)
        {
            dbgMsg_s("Problem opening handle to export GPIO pin 60\n");
            return false;
        }
        if (turnLedOn)
        {
            strcpy(setValue, "1");
        }
        else
        {
            strcpy(setValue, "0");
        }
        fwrite(&setValue, sizeof (char), strlen(setValue), hLED);
        fflush(hLED);
        fclose(hLED);

        return true;
    }
    catch (...)
    {
    }
    dbgMsg_s("local_setGreenLED() terminated unexpectedly\n");
    return false;
}

