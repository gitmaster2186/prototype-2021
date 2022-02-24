package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class LoaderRollers {
    private WPI_TalonFX falcon500ShooterFlyWheel1;
    private WPI_TalonFX falcon500ShooterFlyWheel2;
    private CANSparkMax neo550ShooterLoadRollerFront;
    private CANSparkMax neo550ShooterLoadRollerBack;
    private RelativeEncoder neo550ShooterLoadRollerEncoder;
    private SlewRateLimiter loaderFilter = new SlewRateLimiter(Constants.LOADER_RAMP_UP_POWER);

    // private boolean leftLimitSwitchTrippedValue = false;
    // private boolean rightLimitSwitchTrippedValue = false;
    // private DigitalInput leftLimitSwitch = new DigitalInput(Constants.DIOleftLimitSwitch);
    // private DigitalInput rightLimitSwitch = new DigitalInput(Constants.DIOrightLimitSwitch);
  
    public LoaderRollers(WPI_TalonFX inFalcon500ShooterFlyWheel1,
                         WPI_TalonFX inFalcon500ShooterFlyWheel2,
                         CANSparkMax inneo550ShooterLoadRollerFront,
                         CANSparkMax inneo550ShooterLoadRollerBack)
    {
        // we need the fly wheels because we need to check it is up to speed
        // before turning on the load roller.
        falcon500ShooterFlyWheel1 = inFalcon500ShooterFlyWheel1;
        falcon500ShooterFlyWheel2 = inFalcon500ShooterFlyWheel2;
        neo550ShooterLoadRollerFront = inneo550ShooterLoadRollerFront;
        neo550ShooterLoadRollerBack = inneo550ShooterLoadRollerBack;

        neo550ShooterLoadRollerFront.restoreFactoryDefaults();
        neo550ShooterLoadRollerBack.restoreFactoryDefaults();

        // neo550 motor specs: Hall-Sensor Encoder Resolution: 42 counts per rev.
        neo550ShooterLoadRollerEncoder  = neo550ShooterLoadRollerFront.getEncoder();   
    }
    /*
     * toggle the loader rollers on/off
     * run while button is pushed to load ball into firing position
     * do not load ball if flywheels are not up to speed 
     * 
     * objects used: falcon500ShooterFlyWheel1, 
     *               falcon500ShooterFlyWheel2,
     *               neo550ShooterLoadRoller,
     *               neo550ShooterLoadRollerEncoder
     * 
     */
    public void toggle(boolean buttonPressed)
    {
        double speed = Constants.LOADER_SPEED_ON;
        if (buttonPressed == false)  
        {
             speed = Constants.LOADER_SPEED_OFF;
        }    
        double fltSpeed = loaderFilter.calculate(speed);

        neo550ShooterLoadRollerFront.set(fltSpeed);
        neo550ShooterLoadRollerBack.set(fltSpeed);
        
        // if we want RPS instead of RPM?
        // neo550ShooterLoadRollerEncoder.setVelocityConversionFactor(60);
        
        // put up the rpm number
        SmartDashboard.putNumber("LoadRoller Velocity", 
                                 neo550ShooterLoadRollerEncoder.getVelocity());
    }

    public boolean ballLoaded() {
        return true;
    }

}
