/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package test_cv;

/**
 *
 * @author Jesse Lozano
 */
public class JTargetInfo {

double ballRadiusNear;
double ballRadiusFar;
double distanceToFarEdgeOfView;
int isVisionOriented;
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
        

    public void JTargetInfo(){
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
    isVisionOriented = 0;
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

public void setisVisonOriented(int num)
{
    isVisionOriented = num;
}

public int getIsVisonOriented()
{
    return isVisionOriented;
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

public void setAngleFromStraightAheadToBlue(double fnum)
{
    angleFromStraightAheadToBlue = fnum;
}

public double getAngleFromStraightAheadToBlue()
{
    return angleFromStraightAheadToBlue;
}

public void setAngleFromStraightAheadToRed(double fnum)
{
    angleFromStraightAheadToRed = fnum;
}

public double getAngleFromStraightAheadToRed()
{
    return angleFromStraightAheadToRed;
}

public void setDistanceToBlue(double fnum)
{
    distanceToBlue = fnum;
}

public double getDistanceToBlue()
{
    return distanceToBlue;
}

public void setDistanceToRed(double fnum)
{
    distanceToRed = fnum;
}

public double getDistanceToRed()
{
    return distanceToRed;
}

public void initTargetInfoFromText(String Text)
{
    int ctr = 0;
    String targetInfoText = Text;
    while(ctr < 3)
    {
        //this if-statement removes commas in the beginning of the text
        if(targetInfoText.indexOf(",") == 0)
        {
            targetInfoText = targetInfoText.substring(1);
        }
        commaPos = targetInfoText.indexOf(",");//find position of the comma
        word = targetInfoText.substring(0, commaPos);//returns a piece of the string
        num = Integer.parseInt(word);//turns that piece into its appropriate number type
        targetInfoText = targetInfoText.substring(commaPos);//removes that piece from the text
        if(ctr == 0)
        {
            setisVisonOriented(num);
        }//end if
        if(ctr == 1)
        {
            setisBlueFound(num);
        }//end if
        if(ctr == 2)
        {
            setisRedFound(num);
        }//end if
        ctr++;
    }//end while
    while(ctr < 7 && ctr > 2)
    {
        if(targetInfoText.indexOf(",") == 0)
        {
            targetInfoText = targetInfoText.substring(1);
        }
        if(ctr > 2 && ctr < 6)
        {
        commaPos = targetInfoText.indexOf(",");
        word = targetInfoText.substring(0, commaPos);//
        fnum = Double.parseDouble(word);
        targetInfoText = targetInfoText.substring(commaPos);
        
        if(ctr == 3)
        {
            setAngleFromStraightAheadToBlue(fnum);
        }
        if(ctr == 4)
        {
            setAngleFromStraightAheadToRed(fnum);
        }
        if(ctr == 5)
        {
            setDistanceToBlue(fnum);
        }
        }// end if 2 < ctr < 6
        if(ctr == 6 )
        {
            //targetInfoText = targetInfoText.substring(1);
            fnum = Double.parseDouble(targetInfoText);
            setDistanceToRed(fnum);
        }
       
        
        ctr++;
    }//end while
}

public String NumberToText()
{
    s = Integer.toString(getIsVisonOriented()) + ",";
    s += Integer.toString(getIsBlueFound() ) + ",";
    s += Integer.toString(getIsRedFound() ) + ",";
    s += Double.toString(getAngleFromStraightAheadToBlue() ) + ",";
    s += Double.toString(getAngleFromStraightAheadToRed() ) + ",";
    s += Double.toString(getDistanceToBlue() ) + ",";
    s += Double.toString(getDistanceToRed() );
    return s;
}

public String displayText()
{
    boolean blue = false, red = false; 
    
    if(isBlueFound != 0)
    {
        blue = true;
    }
    if(isRedFound != 0)
    {
        red = true;
    }
    
    String str = ("Is Vision Oriented: " + isVisionOriented + "\n" +
                  "Blue Found: " + blue + "\n" +
                  "Red Found: " + red + "\n" +
                  "Angle to blue: " + angleFromStraightAheadToBlue + "\n" +
                  "Angle to red: " + angleFromStraightAheadToRed + "\n" + 
                  "Distance to Blue: " + distanceToBlue + "\n" +
                  "Distance to Red: " + distanceToRed);
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
