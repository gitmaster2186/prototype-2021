package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class LimeLightSupport {
    
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable limeLightTable = inst.getTable("limelight");
    boolean ledsOn = false;

    public LimeLightSupport()
    {
        ledsSet(false);
    }

    // this forces the limelight LEDs on or off
    public void ledsSet(boolean inLedsOn)
    {
        Number dir;
        if (inLedsOn == true)
        {
            dir = Constants.LIMELIGHT_LEDS_ON;
            ledsOn = true;
        }
        else
        {
            ledsOn = false;
            dir = Constants.LIMELIGHT_LEDS_OFF;
        }
        limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(dir);
        SmartDashboard.putBoolean("ledsOnxx", ledsOn);
    }

    // this toggles the limelight LEDs on or off
    public void ledsToggle()
    {
        if (ledsOn == false)
        {
            ledsSet(true);
        }
        else
        {
            ledsSet(false);
        }
    }
    
}
