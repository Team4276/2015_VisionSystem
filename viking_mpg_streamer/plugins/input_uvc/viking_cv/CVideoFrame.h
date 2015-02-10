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


#define VIEW_PIXEL_X_WIDTH 416
#define VIEW_PIXEL_Y_HEIGHT 240

#define PIXEL_OFFSET_FROM_CENTERLINE_TO_CAMERA 60
#define PIXEL_OFFSET_FROM_BOTTOM_OF_VIEW_TO_COLLECTOR_WHEELS 110
#define PIXEL_OFFSET_FROM_ROBOT_CENTER_TO_BOTTOM_OF_VIEW 150
#define INCHES_PER_PIXEL 0.2
#define DEGREES_PER_RADIAN (180.0/3.141592658979323846264)

class CVideoFrame
{
public:

    typedef enum
    {
        FRAME_QUEUE_TYPE_UNKNOWN,
        FRAME_QUEUE_WAIT_FOR_BLOB_DETECT,
        FRAME_QUEUE_WAIT_FOR_TEXT_CLIENT,
        FRAME_QUEUE_WAIT_FOR_BROWSER_CLIENT,
        FRAME_QUEUE_FREE,
        NUMBER_OF_FRAME_QUEUES
    } FRAME_QUEUE_TYPE;

    CVideoFrame();
    ~CVideoFrame();

    void init();
    void updateAnnotationInfo(
            const CToteRectangle& ToteRectangleGray);
    void annotate();
    void compress();

    cv::Mat m_frame;
    CTargetInfo m_targetInfo;

    CToteRectangle m_toteRectangleGray;

    struct timespec m_timeAddedToQueue[CVideoFrame::NUMBER_OF_FRAME_QUEUES];
    struct timespec m_timeRemovedFromQueue[CVideoFrame::NUMBER_OF_FRAME_QUEUES];

    std::vector<uchar> m_outbuf;
    std::vector<int> m_params;
};