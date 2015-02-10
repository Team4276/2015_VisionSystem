/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package test_streamer;

/**
 *
 * @author Jesse Lozano
 */
public class JTargetInfo
{

    int m_timeSinceLastCameraFrameMilliseconds;
    int m_timeLatencyThisCameraFrameMilliseconds;
    int m_isGrayToteFound;  // zero == false,  nonzero == true
    
    // If this is zero the robot is pointed at the tote center. 
    // Positive (+) means tote is to the right, and the robot needs to rotate clockwise to reduce the angle to zero
    double m_toteDirectionDegrees;
    
    // Rotation of the tote around its center, relative to a straight line from the robot to the tote 
    // Positive (+) means tote is rotated clockwise and the robot needs to move left to reduce the angle to zero
    double m_toteAngleDegrees;
    
    // As measured in a straight line from the center of the robot to the center of the tote
    // For autonomous operation want to arrive at a position where both angles are zero and this distance is a fixed offset "X"
    // "X" is TBD, depending on ideal distance from the tote to begin autonomous tote pickup
    double m_distanceToToteCenterInches;
    
    int commaPos;
    String word;
    int num;
    double fnum;
    String s;

    public void JTargetInfo()
    {
        init();
    }

    public void init()
    {
        m_timeSinceLastCameraFrameMilliseconds = 0;
        m_timeLatencyThisCameraFrameMilliseconds = 0;
        m_isGrayToteFound = 0;
        m_toteDirectionDegrees = 0;
        m_toteAngleDegrees = 0;
        m_distanceToToteCenterInches = 0;

        commaPos = 0;
        word = "";
        num = 0;
        s = "";
        fnum = 0.0;
    }

    public void initTargetInfoFromText(String Text)
    {
        int ctr = 0;
        String targetInfoText = Text;
        while (ctr < 3)
        {
            //this if-statement removes commas in the beginning of the text
            if (targetInfoText.indexOf(",") == 0)
            {
                targetInfoText = targetInfoText.substring(1);
            }
            commaPos = targetInfoText.indexOf(",");//find position of the comma
            word = targetInfoText.substring(0, commaPos);//returns a piece of the string
            num = Integer.parseInt(word);//turns that piece into its appropriate number type
            targetInfoText = targetInfoText.substring(commaPos);//removes that piece from the text
            if (ctr == 0)
            {
                m_timeSinceLastCameraFrameMilliseconds = num;
            }//end if
            if (ctr == 1)
            {
                m_timeLatencyThisCameraFrameMilliseconds = num;
            }//end if
            if (ctr == 2)
            {
                m_isGrayToteFound = num;
            }//end if
            ctr++;
        }//end while
        while (ctr < 6 && ctr > 2)
        {
            if (targetInfoText.indexOf(",") == 0)
            {
                targetInfoText = targetInfoText.substring(1);
            }
            if (ctr > 2 && ctr < 5)
            {
                commaPos = targetInfoText.indexOf(",");
                word = targetInfoText.substring(0, commaPos);//
                fnum = Double.parseDouble(word);
                targetInfoText = targetInfoText.substring(commaPos);

                if (ctr == 3)
                {
                    m_toteDirectionDegrees = fnum;
                    m_toteDirectionDegrees /= 10;
                }
                if (ctr == 4)
                {
                    m_toteAngleDegrees = fnum;
                    m_toteAngleDegrees /= 10;
               }
            }// end if 2 < ctr < 6
            if (ctr == 5)
            {
                //targetInfoText = targetInfoText.substring(1);
                fnum = Double.parseDouble(targetInfoText);
                m_distanceToToteCenterInches = fnum;
                m_distanceToToteCenterInches /= 10;
           }

            ctr++;
        }//end while
    }

    public String displayText()
    {
        String str;
        if(m_isGrayToteFound == 0)
        {
            str = "No Gray Tote found\n";
        }
        else
        {
            str = "Direction: " + m_toteDirectionDegrees 
                    + "(deg),  Angle: " + m_toteAngleDegrees 
                    + "(deg),  Distance: " + m_distanceToToteCenterInches 
                    + " (inches),   Latency: " + m_timeLatencyThisCameraFrameMilliseconds
                    + " (ms.)\n";
        }
        return str;
    }
}
