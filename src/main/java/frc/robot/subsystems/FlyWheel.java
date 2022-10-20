package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class FlyWheel {

    private CANSparkMax shooterFlyWheelFront;
    private CANSparkMax shooterFlyWheelBack;
    RelativeEncoder frontEncoder;
    RelativeEncoder backEncoder;
        
    // private double startTime = 0;
    // private int startballShootTimedCounter = 0;
    // private int stopballShootTimedCounter = 0;
    // private boolean shooterActive = false;
    private int startFireBallCounter = 0;
    private int stopFireBallCounter = 0;
    // private SlewRateLimiter leftFlyfilter = new SlewRateLimiter(Constants.FLY_WHEEL_RAMP_UP_POWER);
    // private SlewRateLimiter rightFlyfilter = new SlewRateLimiter(Constants.FLY_WHEEL_RAMP_UP_POWER);
    public FlyWheel()
    {
        // 10/18/22 moved instanciation of motor objects to here
//        shooterFlyWheelFront = new WPI_TalonFX(Constants.FALCON500_FRONT_FLYWHEEL_CAN_ID);
 //       shooterFlyWheelBack = new WPI_TalonFX(Constants.FALCON500_BACK_FLYWHEEL_CAN_ID);
        shooterFlyWheelFront = new CANSparkMax(Constants.FRONT_FLYWHEEL_CAN_ID, MotorType.kBrushless);
        shooterFlyWheelBack = new CANSparkMax(Constants.BACK_FLYWHEEL_CAN_ID, MotorType.kBrushless);
        frontEncoder = shooterFlyWheelFront.getEncoder();
        backEncoder = shooterFlyWheelBack.getEncoder();
    
        shooterFlyWheelFront.restoreFactoryDefaults();
        shooterFlyWheelBack.restoreFactoryDefaults();

        //shooterFlyWheelFront.setInverted(true);
        shooterFlyWheelBack.setInverted(true); 

        // !!!SID!!! XXX - 3/31/22 - try making\one flywheel follow the other
        // shooterFlyWheelFront.follow(shooterFlyWheelBack);
        // shooterFlyWheelBack.follow(shooterFlyWheelFront);
    }



    /*
     * toggle the ball shooter flywheels on/off
     * 
     * objects used: shooterFlyWheelFront, shooterFlyWheelBack
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
//        shooterFlyWheelFront.set(xl * 0.9825); // equal speed
        shooterFlyWheelFront.set(xl * Constants.FLY_WHEEL_SCALING); // backspin test
        //shooterFlyWheelFront.set(xl);
        // set the right flywheel speed
        // shooterFlyWheelBack.set(xr);
        shooterFlyWheelBack.set(xr * Constants.FLY_WHEEL_SCALING);
        // shooterFlyWheelBack.set(1);

        // measure the velocity of each flywheel and display the data
        // on the dashboard for debug.
/*
        double velFr = shooterFlyWheelFront.getSelectedSensorVelocity();
        double velBa = shooterFlyWheelBack.getSelectedSensorVelocity();
        double velDiff = velFr - velBa;
        double velRatio = velFr/velBa;
        SmartDashboard.putNumber("flyWFrVel", velFr);
        SmartDashboard.putNumber("flyWBaVel", velBa);
        SmartDashboard.putNumber("velDiff", velDiff);
        SmartDashboard.putNumber("velRatio", velRatio);
*/

    }

    // return true if the flywheels are up to speed otherwise false.
    public boolean upToSpeed() 
    {
        boolean ret = false;
        double flyVel1 = frontEncoder.getVelocity();
        double flyVel2 = backEncoder.getVelocity();

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
