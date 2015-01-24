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
#include <string.h>

static pthread_mutex_t static_dbgMutex;
static pthread_mutexattr_t static_dbgMutexattr;

void dbgInit()
{
    pthread_mutex_init(&static_dbgMutex, &static_dbgMutexattr);
    FILE* fp = fopen("/home/ubuntu/dbg.log", "w");
    if (fp != NULL)
    {
        fclose(fp);
    }
}

void dbgMsg(const char* msg)
{
    pthread_mutex_lock(&static_dbgMutex);
    try
    {
        FILE* fp = fopen("/home/ubuntu/dbg.log", "a");
        if (fp != NULL)
        {
            int bytesWritten = fwrite(msg, sizeof (char), strlen(msg), fp);
            bytesWritten = fwrite("\n", sizeof (char), 1, fp);
            fflush(NULL);
            fclose(fp);
        }
    }
    catch (...)
    {
    }
    pthread_mutex_unlock(&static_dbgMutex);
}

void dbgMsg_s(const std::string& str)
{
    printf("%s", str.c_str());
    dbgMsg(str.c_str());
}

void dbgMsg_d1(const char* fmt, unsigned int val_1)
{
    char buf[128];
    sprintf(buf, fmt, val_1);
    printf("%s", buf);
    dbgMsg(buf);
}

void dbgMsg_f1(const char* fmt, float val_1)
{
    char buf[128];
    sprintf(buf, fmt, val_1);
    printf("%s", buf);
    dbgMsg(buf);
}


