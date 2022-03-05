package frc.robot;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.OurTimer;
import frc.robot.utils.TankSpeeds;

public class AutonomousMode {

    public enum AutonomousStateEnum {
        AUTO_STATE_INIT,
        AUTO_STATE_MOVE,
        AUTO_STATE_AIM,
        AUTO_STATE_SHOOT,
        AUTO_STATE_WAIT
    }
    AutonomousStateEnum autonomousStateVar = AutonomousStateEnum.AUTO_STATE_INIT;
    boolean stateInit = true;
    DriveTrain driveTrain;
    BallShooter shooter;
    OurTimer stateTimer;

    // !!!SID!!! XXX - these values must be tuned!!!
    private static final double AUTO_MOVE_TIME = 2.0;
    //private static final double AUTO_MOVE_DISTANCE = 2.0;
    private static final double AUTO_AIM_TIME = 2.0;
    private static final double AUTO_SHOOT_TIME = 2.0;
    private static final double AUTO_MOVE_SPEED = 0.5;


    public AutonomousMode(DriveTrain dt, BallShooter inShooter)
    {
        nextState("Init", AutonomousStateEnum.AUTO_STATE_INIT, false);
        driveTrain = dt;
        shooter = inShooter;
        stateTimer = new OurTimer();
    }

    private void nextState(String stateString, 
                           AutonomousStateEnum nextState, 
                           boolean initTimer)
    {
        autonomousStateVar = nextState;
        setStateInit(true);    
        SmartDashboard.putString("autonomousStateVar", stateString);
        if (initTimer)
        {
            stateTimer.initTimer();
        }
    }                    

    private boolean getStateInit()
    {
        return stateInit;
    }            
    private void setStateInit(boolean initVar)
    {
        stateInit = initVar;
    }            
        
    public void autoModePeriodic()
    {
        switch(autonomousStateVar)
        {
            case AUTO_STATE_INIT:
                // immediately transition to the MOVE state
                nextState("Move", AutonomousStateEnum.AUTO_STATE_MOVE, true);
                break;
            
            case AUTO_STATE_MOVE:
                // move off the tarmac
                
                if (getStateInit() == true)
                {
                    setStateInit(false);
                    driveTrain.timerInit();
                }

                // for now do a dumb timed based move
                // !!!SID!!! XXX - this should be a distance based move 
                //                 with encoders and gyro to move straight
                if ((driveTrain.timedMove(AUTO_MOVE_TIME, AUTO_MOVE_SPEED) == true) ||
                    (stateTimer.timerTest(AUTO_MOVE_TIME) == true))
                {
                    nextState("Aim", AutonomousStateEnum.AUTO_STATE_AIM, true);
                }
                break;
    
            case AUTO_STATE_AIM:
                // use vision to get into shooting position but don't do it forever...
                TankSpeeds ts = new TankSpeeds(0, 0);
                if ((driveTrain.aimAssist(ts)  == true) ||
                    (stateTimer.timerTest(AUTO_AIM_TIME)  == true))
                {
                    nextState("Shoot", AutonomousStateEnum.AUTO_STATE_SHOOT, true);
                }
                break;
    
            case AUTO_STATE_SHOOT:
                // try to shoot the ball
                if ((shooter.manualShoot() == true) ||
                    (stateTimer.timerTest(AUTO_SHOOT_TIME) == true))
                {
                    nextState("Wait", AutonomousStateEnum.AUTO_STATE_WAIT, false);
                }
                break;
    
            case AUTO_STATE_WAIT:
                // autonomous is done. 
                // nothing else to do while waiting for telop mode to start
                break;
    
            default:
                System.out.println("---BAD State---");
        }
    }
}
