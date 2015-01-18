/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package display_cv;

/**
 *
 * @author Jesse Lozano
 */
public class JTargetInfo
{

    double ballRadiusNear;
    double ballRadiusFar;
    double distanceToFarEdgeOfView;

    int timeSinceLastCameraFrameMilliseconds;
    int timeLatencyThisCameraFrameMilliseconds;
    int isRightGreenTargetLit;
    int isLeftGreenTargetLit;
    int isBlueFound;
    int isRedFound;
    double angleFromStraightAheadToBlue;
    double angleFromStraightAheadToRed;
    double distanceToBlue;
    double distanceToRed;

    int commaPos;
    String word;
    int num;
    double fnum;
    String s;

    public void JTargetInfo()
    {
        init();
    }

    /*
     public void JTargetInfo(const JTargetInfo & orig)
     {
     init();
     }
     */
    public void init()
    {
        timeSinceLastCameraFrameMilliseconds = 0;
        timeLatencyThisCameraFrameMilliseconds = 0;
        isRightGreenTargetLit = 0;
        isLeftGreenTargetLit = 0;
        isBlueFound = 0;
        isRedFound = 0;
        angleFromStraightAheadToBlue = 0.0;
        angleFromStraightAheadToRed = 0.0;
        distanceToBlue = 0.0;
        distanceToRed = 0.0;

        commaPos = 0;
        word = "";
        num = 0;
        s = "";
        double fnum = 0.0;
    }

    public void settimeSinceLastCameraFrameMilliseconds(int num)
    {
        timeSinceLastCameraFrameMilliseconds = num;
    }

    public int getTimeSinceLastCameraFrameMilliseconds()
    {
        return timeSinceLastCameraFrameMilliseconds;
    }

   public void settimeLatencyThisCameraFrameMilliseconds(int num)
    {
        timeLatencyThisCameraFrameMilliseconds = num;
    }

    public int getTimeLatencyThisCameraFrameMilliseconds()
    {
        return timeLatencyThisCameraFrameMilliseconds;
    }

   public void setisRightGreenTargetLit(int num)
    {
        isRightGreenTargetLit = num;
    }

    public int getIsRightGreenTargetLit()
    {
        return isRightGreenTargetLit;
    }

   public void setisLeftGreenTargetLit(int num)
    {
        isLeftGreenTargetLit = num;
    }

    public int getIsLeftGreenTargetLit()
    {
        return isLeftGreenTargetLit;
    }

    public void setisBlueFound(int num)
    {
        isBlueFound = num;
    }

    public int getIsBlueFound()
    {
        return isBlueFound;
    }

    public void setisRedFound(int num)
    {
        isRedFound = num;
    }

    public int getIsRedFound()
    {
        return isRedFound;
    }

    public void setAngleFromStraightAheadToBlue(int num)
    {
        angleFromStraightAheadToBlue = num;
        angleFromStraightAheadToBlue /= 10.0;  // degrees
    }

    public double getAngleFromStraightAheadToBlue()
    {
        return angleFromStraightAheadToBlue;
    }

    public void setAngleFromStraightAheadToRed(int num)
    {
        angleFromStraightAheadToRed = num;
        angleFromStraightAheadToRed /= 10.0;  // degrees
    }

    public double getAngleFromStraightAheadToRed()
    {
        return angleFromStraightAheadToRed;
    }

    public void setDistanceToBlue(int num)
    {
        distanceToBlue = num;
        distanceToBlue /= 12.0;   // feet
    }

    public double getDistanceToBlue()
    {
        return distanceToBlue;
    }

    public void setDistanceToRed(int num)
    {
        distanceToRed = num;
        distanceToRed /= 12.0;   // feet
    }

    public double getDistanceToRed()
    {
        return distanceToRed;
    }

    public void initTargetInfoFromText(String Text)
    {
        int ctr = 0;
        String targetInfoText = Text;
        while (ctr < 10)
        {
            //this if-statement removes commas in the beginning of the text
            if (targetInfoText.indexOf(",") == 0)
            {
                targetInfoText = targetInfoText.substring(1);
            }
            commaPos = targetInfoText.indexOf(",");//find position of the comma
            if (commaPos == -1)
            {
                word = targetInfoText;
                num = Integer.parseInt(word);//turns that piece into its appropriate number type
            } else
            {
                word = targetInfoText.substring(0, commaPos);//returns a piece of the string
                num = Integer.parseInt(word);//turns that piece into its appropriate number type
                targetInfoText = targetInfoText.substring(commaPos);//removes that piece from the text
            }
            switch (ctr)
            {
                case 0:
                    settimeSinceLastCameraFrameMilliseconds(num);
                    break;

                case 1:
                    settimeLatencyThisCameraFrameMilliseconds(num);
                    break;

                case 2:
                    setisRightGreenTargetLit(num);
                    break;

                case 3:
                    setisLeftGreenTargetLit(num);
                    break;

                case 4:
                    setisBlueFound(num);
                    break;

                case 5:
                    setAngleFromStraightAheadToBlue(num);
                    break;

                case 6:
                    setDistanceToBlue(num);
                    break;

                case 7:
                    setisRedFound(num);
                    break;

                case 8:
                    setAngleFromStraightAheadToRed(num);
                    break;

                case 9:
                    setDistanceToRed(num);
                    break;

                default:
                    System.out.println("Unexpected text received from BeagleBone");
                    break;
            }
            ctr++;
        }//end while
    }

    public String NumberToText()
    {
        s = Integer.toString(getTimeSinceLastCameraFrameMilliseconds()) + ",";
        s += Integer.toString(getTimeLatencyThisCameraFrameMilliseconds()) + ",";
        s += Integer.toString(getIsRightGreenTargetLit()) + ",";
        s += Integer.toString(getIsLeftGreenTargetLit()) + ",";
        s += Integer.toString(getIsBlueFound()) + ",";
        s += Integer.toString(getIsRedFound()) + ",";
        s += Double.toString(getAngleFromStraightAheadToBlue()) + ",";
        s += Double.toString(getAngleFromStraightAheadToRed()) + ",";
        s += Double.toString(getDistanceToBlue()) + ",";
        s += Double.toString(getDistanceToRed());
        return s;
    }

    public String displayText()
    {
        boolean blue = false, red = false;
        boolean right_lit = false, left_lit = false;

        if (isBlueFound != 0)
        {
            blue = true;
        }
        if (isRedFound != 0)
        {
            red = true;
        }
        if (isRightGreenTargetLit != 0)
        {
            right_lit = true;
        }
        if (isLeftGreenTargetLit != 0)
        {
            left_lit = true;
        }

        String str = ("Time Since Last Frame: " + timeSinceLastCameraFrameMilliseconds + "ms.\n"
                + "Latency This Frame: " + timeSinceLastCameraFrameMilliseconds + "ms.\n"
                + "Right Green Target Lit: " + right_lit + "\n"
                + "Left Green Target Lit: " + left_lit + "\n"
                + "Blue Found: " + blue + "\n"
                + "Red Found: " + red + "\n"
                + "Angle to blue: " + angleFromStraightAheadToBlue + "\n"
                + "Angle to red: " + angleFromStraightAheadToRed + "\n"
                + "Distance to Blue: " + distanceToBlue + "\n"
                + "Distance to Red: " + distanceToRed);
        return str;
    }

    public void initFormattedTextFromTargetInfo()
    {
    //targetInfoText = Text;
        // TODO:  Assign values to target info data members by parsing the text string from initFormattedTextFromTargetInfo() 

        // Format text for transmission to the cRio
        initFormattedTextFromTargetInfo();
    }

}
