package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class LoaderRollers {
    private CANSparkMax neo550ShooterLoaderRollerFront;
    private CANSparkMax neo550ShooterLoaderRollerBack;
    //private RelativeEncoder neo550ShooterLoaderRollerEncoder;
    private SlewRateLimiter loaderFilter = new SlewRateLimiter(Constants.LOADER_RAMP_UP_POWER);
    int dbgCount = 0;
    // private boolean leftLimitSwitchTrippedValue = false;
    // private boolean rightLimitSwitchTrippedValue = false;
    // private DigitalInput leftLimitSwitch = new DigitalInput(Constants.DIOleftLimitSwitch);
    // private DigitalInput rightLimitSwitch = new DigitalInput(Constants.DIOrightLimitSwitch);
  
    public LoaderRollers(CANSparkMax inneo550ShooterLoaderRollerFront,
                         CANSparkMax inneo550ShooterLoaderRollerBack)
    {
        // we need the fly wheels because we need to check it is up to speed
        // before turning on the load roller.
        neo550ShooterLoaderRollerFront = inneo550ShooterLoaderRollerFront;
        neo550ShooterLoaderRollerBack = inneo550ShooterLoaderRollerBack;

        neo550ShooterLoaderRollerFront.restoreFactoryDefaults();
        neo550ShooterLoaderRollerBack.restoreFactoryDefaults();

        // neo550 motor specs: Hall-Sensor Encoder Resolution: 42 counts per rev.
        // neo550ShooterLoaderRollerEncoder  = neo550ShooterLoaderRollerFront.getEncoder();   
    }
    /*
     * toggle the loader rollers on/off
     * run while button is pushed to load ball into firing position
     * do not load ball if flywheels are not up to speed 
     * 
     * objects used: falcon500ShooterFlyWheel1, 
     *               falcon500ShooterFlyWheel2,
     *               neo550ShooterLoaderRoller,
     *               neo550ShooterLoaderRollerEncoder
     * 
    
    
     */
    
    public void toggle(boolean toggleOn)
    {
        dbgCount += 1;
        System.out.println("loaderToggle " + dbgCount);
        double speed = Constants.LOADER_SPEED_ON;
        double fltSpeed = 0.0;

        if (toggleOn == true)
        {
            fltSpeed = loaderFilter.calculate(speed);
        }
        
        neo550ShooterLoaderRollerFront.set(fltSpeed);
        neo550ShooterLoaderRollerBack.set(fltSpeed);

        SmartDashboard.putNumber("FrontLoader speed", 
                                 speed);
    }   

    //  public void toggle(boolean buttonPressed)
    // {
    //     double speed = Constants.LOADER_SPEED_ON;
    //     if (buttonPressed == false)  
    //     {
    //          speed = Constants.LOADER_SPEED_OFF;
    //     }    
    //     double fltSpeed = loaderFilter.calculate(speed);

    //     neo550ShooterLoaderRollerFront.set(fltSpeed);
    //     neo550ShooterLoaderRollerBack.set(fltSpeed);
        
    //     // if we want RPS instead of RPM?
    //     // neo550ShooterLoaderRollerEncoder.setVelocityConversionFactor(60);
        
    //     // put up the rpm number
    //     SmartDashboard.putNumber("LoadRoller Velocity", 
    //                              neo550ShooterLoaderRollerEncoder.getVelocity());
    // }

    public boolean ballLoaded() {
        return true;
    }

}
