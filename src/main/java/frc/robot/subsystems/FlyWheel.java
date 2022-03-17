package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class FlyWheel {

    private WPI_TalonFX falcon500ShooterFlyWheel1;
    private WPI_TalonFX falcon500ShooterFlyWheel2;
    
    // private double startTime = 0;
    // private int startballShootTimedCounter = 0;
    // private int stopballShootTimedCounter = 0;
    // private boolean shooterActive = false;
    //private int startFireBallCounter = 0;
    //private int stopFireBallCounter = 0;
    // private SlewRateLimiter leftFlyfilter = new SlewRateLimiter(Constants.FLY_WHEEL_RAMP_UP_POWER);
    // private SlewRateLimiter rightFlyfilter = new SlewRateLimiter(Constants.FLY_WHEEL_RAMP_UP_POWER);
                                      
    public FlyWheel(WPI_TalonFX inFalcon500ShooterFlyWheel1,
                    WPI_TalonFX inFalcon500ShooterFlyWheel2)
                   
    {
        falcon500ShooterFlyWheel1 = inFalcon500ShooterFlyWheel1;
        falcon500ShooterFlyWheel2 = inFalcon500ShooterFlyWheel2;

        falcon500ShooterFlyWheel1.configFactoryDefault();
        falcon500ShooterFlyWheel2.configFactoryDefault();

        falcon500ShooterFlyWheel1.setInverted(true);
        falcon500ShooterFlyWheel2.setInverted(true);        
    }



    /*
     * toggle the ball shooter flywheels on/off
     * 
     * objects used: falcon500ShooterFlyWheel1, falcon500ShooterFlyWheel2
     */
    public void setFlyWheelSpeed(double inFlySpeed)
    {
        double speed;
        double xl;
        double xr;
        if (inFlySpeed == Constants.FLYWHEEL_OFF) 
        {
            xl = 0.0;
            xr = 0.0;
            // shooterActive = false;
            //stopFireBallCounter += 1;
            // SmartDashboard.putNumber("stopFireBallCounter", stopFireBallCounter);    
        }
        else
        {
            speed = inFlySpeed;
            // shooterActive = true;
            //startFireBallCounter += 1;

            // !!!SID!!! XXX - reenabled the ramp filters?
            //xl = leftFlyfilter.calculate(speed);
            // xr = rightFlyfilter.calculate(speed);

            xl = speed;
            xr = speed;
            // SmartDashboard.putNumber("startFireBallCounter", startFireBallCounter);    
        }

        // set the left flywheel speed
        falcon500ShooterFlyWheel1.set(xl);
        // set the right flywheel speed
        falcon500ShooterFlyWheel2.set(xr);

        // measure the velocity of each flywheel and display the data
        // on the dashboard for debug.

        double vel2 = falcon500ShooterFlyWheel2.getSelectedSensorVelocity();
        SmartDashboard.putNumber("flyW2vel", vel2);

    }

    // return true if the flywheels are up to speed otherwise false.
    public boolean upToSpeed() 
    {
        boolean ret = false;
        double flyVel1 = falcon500ShooterFlyWheel1.getSelectedSensorVelocity();
        double flyVel2 = falcon500ShooterFlyWheel2.getSelectedSensorVelocity();
        if ((flyVel1 >= Constants.FLYWHEEL_MIN_VEL) &&
            (flyVel2 >= Constants.FLYWHEEL_MIN_VEL))
        {
            ret = true;
        }

        return ret;
    }
    

    // !!!SID!!! - this ain't right!!!

    /*
     * a timer based shooter
     * 
     * objects used: 
     */
    // public void timed(double fireTurrentTimeSeconds)
    // {
    //     // are we starting a new shot?
    //     if (shooterActive == false)
    //     {
    //         startTime =  Timer.getFPGATimestamp(); // get sys time in seconds
    //         activate(true);
    //         startballShootTimedCounter += 1;
    //         SmartDashboard.putNumber("startballShootTimedCounter", startballShootTimedCounter);
    //     }
    //     else
    //     {
    //         double nowTime = Timer.getFPGATimestamp();

    //         // are we done (timer expired)?
    //         if ((nowTime - startTime) >= fireTurrentTimeSeconds)
    //         {
    //             activate(false);

    //             stopballShootTimedCounter += 1;
    //             SmartDashboard.putNumber("stopballShootTimedCounter", stopballShootTimedCounter);
    //         }
    //     }
    // }

}
