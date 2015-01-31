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


#ifndef CTARGETINFO_H
#define	CTARGETINFO_H

class CTargetInfo
{
public:
    CTargetInfo();
    CTargetInfo(const CTargetInfo& orig);
    virtual ~CTargetInfo();

    void init();

    std::string displayText() const
    {
        return m_targetInfoText;
    }
    void updateTargetInfo(
            int timeSinceLastCameraFrameMilliseconds,
            int timeLatencyThisCameraFrameMilliseconds, 
            bool isGrayToteFound,
            float angleFromStraightAheadToTote,
            float offsetFromCenterlineToToteCenter);

    void initTargetInfoFromText(const std::string& targetInfoText);

    /* Not absolute time -- use for relative time between samples */
    int getTimeSinceLastCameraFrameMilliseconds()
    {
        return m_timeSinceLastCameraFrameMilliseconds;
    }

    int getTimeLatencyThisCameraFrameMilliseconds()
    {
        return m_timeLatencyThisCameraFrameMilliseconds;
    }

    /* 0 == not found */
    int isGrayToteFound() const
    {
        return m_isGrayToteFound;
    }

    /* degrees, + == to the right */
    float angleFromStraightAheadToTote() const
    {
        return m_angleFromStraightAheadToTote;
    }

    /* feet from back of robot, should always be positive */
    float offsetFromCenterlineToToteCenter() const
    {
        return m_offsetFromCenterlineToToteCenter;
    }

    void initFormattedTextFromTargetInfo();

private:


    std::string m_targetInfoText;
    int m_timeSinceLastCameraFrameMilliseconds;
    int m_timeLatencyThisCameraFrameMilliseconds;
    int m_isGrayToteFound;
    float m_angleFromStraightAheadToTote;    // Range +-90 where plus is to the right and zero would make the long side of the tote parallel to the camera centerline
    float m_offsetFromCenterlineToToteCenter;    // In inches.  Positive (+) means the centerline is to the right of the robot  
    float m_distanceToToteCenterInches;   // As measured in a straight line from the center of where the collector wheels would optimally make first contact 
    
    // When the angle and both distances are zero, the robot is perfectly aligned and positioned to collect the tote
    
    // Automated drive strategy would be to take a small control option 10 times a second
    // Always try to rotate the robot to make the m_angleFromStraightAheadToTote be zero
    // If less than "X" inches to the tote  (Need to experiment to find best value for X))
    //     If "close" to lined up on the centerline   (Also need to find out much misalignment the tote collector can comfortably handle))
    //         Run the collector and drive forward to help collection
    //     Else 
    //         Go sideways toward the centerline
    //     Endif
    // Else   (Still far away)
    //     If closer to the centerline than the tote
    //         Go towards the tote
    //     Else
    //         Go sideways toward the centerline
    //     Endif
    // Endif


    //     
    
    // Drive the robot until both offset and angle are zero and the tote should 
    // be perfectly positioned so you can drive straight forward for pickup.
    // After that drive forward until the distance is also zero and 
};

#endif	/* CTARGETINFO_H */

