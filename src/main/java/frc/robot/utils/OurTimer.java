package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

public class OurTimer {
    double startTime;

    public OurTimer()
    {
        initTimer();
    }

    public void initTimer()
    {
        startTime = Timer.getFPGATimestamp(); // get sys time in seconds
    }

    public boolean timerTest(double maxTime)
    {                
        boolean timerExpired = false;
        double currentTime;
        currentTime = Timer.getFPGATimestamp(); // get sys time in seconds
        if ((currentTime - startTime) > maxTime)
        {
            timerExpired = true;
        }
        return timerExpired;
    }       
}
