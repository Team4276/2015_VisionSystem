package test_streamer;

/*
public class GoalieMode {
    double angleBlue;
    double angleRed;
    double distance;
    
    boolean redButton;
    boolean blueButton;
    
    JTargetInfo object; 
    MecanumDrive move;
    
    double distanceConstant;
    double angleConstant;

    public void GoalieMode()
    {
        move = new MecanumDrive(0,0,0,0,0);
        object = new JTargetInfo();
        angleBlue = 0.0;
        angleRed = 0.0;
        redButton = false;
        blueButton = false;
        moveRobot();
        angleConstant = (.0111111111111111111);
        distanceConstant = (.05);
    }
    
    
    
    public void findRed()
    {
       redButton = true;
    }
    
    public void findBlue()
    {
       blueButton = true;
    }
        
    public void setColorAngle()
    {
        if(blueButton == true)
        {
            angleBlue = object.getAngleFromStraightAheadToBlue();
            
        }
        if(redButton == true)
        {
            angleRed = object.getAngleFromStraightAheadToRed();
            
        }
        
    }

    public double getDistance()
    {
         if(blueButton == true)
        {
            distance = object.getDistanceToBlue();
            
        }
        if(redButton == true)
        {
            distance = object.getDistanceToRed();
            
        }
         return distance;
    }
    
    public double getColorAngle()
    {
        //this.setColorAngle();
        if(blueButton == true)
        {
            return angleBlue;
        }
        else if(redButton == true)
        {
            return angleRed;
        }
    }

    public double Distance();
    {
        double distance = 0.0;
        if(blueButton == true)
        {
           distance = object.getDistanceToBlue()* cos(object.getAngleFromStraightAheadToBlue()); 
        }
        else if(redButton == true)
        {
            distance = object.getDistanceToRed()* cos(object.getAngleFromStraightAheadToRed()); 
        }
        return distance;

    }
    
    public void moveRobot()
    {
        while(getColorAngle() < 0)
        {
            move.driveBodyFrameWithSearch(-1,-1,0);
            if(distance<=1)
            {
            move.driveBodyFrameWithSearch(angleConstant * getColorAngle(), 
                                          distanceConstant * getDistance(), 0);
            }
        }

        while(getColorAngle() > 0)
        {
            move.driveBodyFrameWithSearch(1,1,0);
            if(distance<=1)
            {
            move.driveBodyFrameWithSearch(angleConstant * getColorAngle(), 
                                          distanceConstant * getDistance(), 0);
            }
        }
        
        if(getColorAngle() == 0)
        {
            move.driveBodyFrameWithSearch(0,0,0);
        }
        
    }
}

*/