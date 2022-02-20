package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class BallShooter {

    private WPI_TalonFX falcon500ShooterFlyWheel1;
    private WPI_TalonFX falcon500ShooterFlyWheel2;
    
    private double startTime = 0;
    private boolean shooterActive = false;
    private int startballShootTimedCounter = 0;
    private int stopballShootTimedCounter = 0;
    private int startFireBallCounter = 0;
    private int stopFireBallCounter = 0;

    public BallShooter(WPI_TalonFX inFalcon500ShooterFlyWheel1,
                       WPI_TalonFX inFalcon500ShooterFlyWheel2)
    {
        falcon500ShooterFlyWheel1 = inFalcon500ShooterFlyWheel1;
        falcon500ShooterFlyWheel2 = inFalcon500ShooterFlyWheel2;
    }


    // !!!SID!!! - this ain't right!!!

    /*
     * a timer based shooter
     * 
     * objects used: 
     */
    public void timed(double fireTurrentTimeSeconds)
    {
        // are we starting a new shot?
        if (shooterActive == false)
        {
            startTime =  Timer.getFPGATimestamp(); // get sys time in seconds
            toggle(true);
            startballShootTimedCounter += 1;
            SmartDashboard.putNumber("startballShootTimedCounter", startballShootTimedCounter);
        }
        else
        {
            double nowTime = Timer.getFPGATimestamp();

            // are we done (timer expired)?
            if ((nowTime - startTime) >= fireTurrentTimeSeconds)
            {
                toggle(false);

                stopballShootTimedCounter += 1;
                SmartDashboard.putNumber("stopballShootTimedCounter", stopballShootTimedCounter);
            }
        }
    }

    /*
     * toggle the ball shooter on/off
     * 
     * objects used: falcon500ShooterFlyWheel1, falcon500ShooterFlyWheel2
     */
    public void toggle(boolean buttonPressed)
    {
        double speed;
        if (buttonPressed == false)
        {
            speed = Constants.FLYWHEEL_OFF;
            shooterActive = false;
            stopFireBallCounter += 1;
            SmartDashboard.putNumber("stopFireBallCounter", stopFireBallCounter);    
        }
        else
        {
            speed = Constants.FLYWHEEL_ON;
            shooterActive = true;
            startFireBallCounter += 1;
            SmartDashboard.putNumber("startFireBallCounter", startFireBallCounter);    
        }

        falcon500ShooterFlyWheel1.set(speed);
        double vel1 = falcon500ShooterFlyWheel1.getSelectedSensorVelocity();
        SmartDashboard.putNumber("flyWheel 1 velocity", vel1);

        falcon500ShooterFlyWheel2.set(speed);
        double vel2 = falcon500ShooterFlyWheel2.getSelectedSensorVelocity();
        SmartDashboard.putNumber("flyWheel 2 velocity", vel2);
    }
    

}
