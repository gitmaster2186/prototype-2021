package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class FlyWheel {

    private WPI_TalonFX falcon500ShooterFlyWheelFront;
    private WPI_TalonFX falcon500ShooterFlyWheelBack;
    
    // private double startTime = 0;
    // private int startballShootTimedCounter = 0;
    // private int stopballShootTimedCounter = 0;
    // private boolean shooterActive = false;
    private int startFireBallCounter = 0;
    private int stopFireBallCounter = 0;
    // private SlewRateLimiter leftFlyfilter = new SlewRateLimiter(Constants.FLY_WHEEL_RAMP_UP_POWER);
    // private SlewRateLimiter rightFlyfilter = new SlewRateLimiter(Constants.FLY_WHEEL_RAMP_UP_POWER);
                                      
    public FlyWheel(WPI_TalonFX inFalcon500ShooterFlyWheelFront,
                    WPI_TalonFX inFalcon500ShooterFlyWheelBack)
                   
    {
        falcon500ShooterFlyWheelFront = inFalcon500ShooterFlyWheelFront;
        falcon500ShooterFlyWheelBack = inFalcon500ShooterFlyWheelBack;

        falcon500ShooterFlyWheelFront.configFactoryDefault();
        falcon500ShooterFlyWheelBack.configFactoryDefault();

        falcon500ShooterFlyWheelFront.setInverted(true);
        falcon500ShooterFlyWheelBack.setInverted(true); 

        // !!!SID!!! XXX - 3/31/22 - try making\one flywheel follow the other
        // falcon500ShooterFlyWheelFront.follow(falcon500ShooterFlyWheelBack);
        // falcon500ShooterFlyWheelBack.follow(falcon500ShooterFlyWheelFront);
    }



    /*
     * toggle the ball shooter flywheels on/off
     * 
     * objects used: falcon500ShooterFlyWheelFront, falcon500ShooterFlyWheelBack
     */
    public void setFlyWheelSpeed(double inFlySpeed)
    {
        double xl;
        double xr;

        if (inFlySpeed == Constants.FLYWHEEL_OFF) 
        {
            // shooterActive = false;

            xl = 0.0;
            xr = 0.0;
            stopFireBallCounter += 1;
            SmartDashboard.putNumber("stopFireBallCounter", stopFireBallCounter);    
        }
        else
        {
            // shooterActive = true;

            // !!!SID!!! XXX - reenabled the ramp filters?
            // double speed;
            // speed = inFlySpeed;
            // xl = leftFlyfilter.calculate(speed);
            // xr = rightFlyfilter.calculate(speed);

            xl = inFlySpeed;
            xr = inFlySpeed;
            startFireBallCounter += 1;
            SmartDashboard.putNumber("startFireBallCounter", startFireBallCounter);    
        }

        // set the left flywheel speed
//        falcon500ShooterFlyWheelFront.set(xl * 0.9825); // equal speed
//        falcon500ShooterFlyWheelFront.set(xl * 0.8); // backspin test
        falcon500ShooterFlyWheelFront.set(xl);
        // set the right flywheel speed
        falcon500ShooterFlyWheelBack.set(xr);
        // falcon500ShooterFlyWheelBack.set(1);

        // measure the velocity of each flywheel and display the data
        // on the dashboard for debug.

        double velFr = falcon500ShooterFlyWheelFront.getSelectedSensorVelocity();
        double velBa = falcon500ShooterFlyWheelBack.getSelectedSensorVelocity();
        double velDiff = velFr - velBa;
        double velRatio = velFr/velBa;
        SmartDashboard.putNumber("flyWFrVel", velFr);
        SmartDashboard.putNumber("flyWBaVel", velBa);
        SmartDashboard.putNumber("velDiff", velDiff);
        SmartDashboard.putNumber("velRatio", velRatio);


    }

    // return true if the flywheels are up to speed otherwise false.
    public boolean upToSpeed() 
    {
        boolean ret = false;
        double flyVel1 = falcon500ShooterFlyWheelFront.getSelectedSensorVelocity();
        double flyVel2 = falcon500ShooterFlyWheelBack.getSelectedSensorVelocity();
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
