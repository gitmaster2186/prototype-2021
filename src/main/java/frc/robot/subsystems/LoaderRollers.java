package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class LoaderRollers {
    private WPI_TalonFX falcon500ShooterFlyWheel1;
    private WPI_TalonFX falcon500ShooterFlyWheel2;
    private CANSparkMax neo550ShooterLoadRoller;
    private RelativeEncoder neo550ShooterLoadRollerEncoder;   

    public LoaderRollers(WPI_TalonFX inFalcon500ShooterFlyWheel1,
                         WPI_TalonFX inFalcon500ShooterFlyWheel2,
                         CANSparkMax inNeo550ShooterLoadRoller,
                         RelativeEncoder inNeo550ShooterLoadRollerEncoder)
    {
        falcon500ShooterFlyWheel1 = inFalcon500ShooterFlyWheel1;
        falcon500ShooterFlyWheel2 = inFalcon500ShooterFlyWheel2;
        neo550ShooterLoadRoller = inNeo550ShooterLoadRoller;
        neo550ShooterLoadRollerEncoder = inNeo550ShooterLoadRollerEncoder;
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
        // get the motor velocity of the flywheels
        double flyVel1 = falcon500ShooterFlyWheel1.getSelectedSensorVelocity();
        SmartDashboard.putNumber("flyWheel 2 velocity", flyVel1);
        double flyVel2 = falcon500ShooterFlyWheel2.getSelectedSensorVelocity();
        SmartDashboard.putNumber("flyWheel 2 velocity", flyVel2);

        // don't allow the loader to load the ball until 
        // the flywheels are up to speed?
        if ((flyVel1 >= Constants.MIN_FLYWHEEL_VEL) &&
            (flyVel2 >= Constants.MIN_FLYWHEEL_VEL))
        {
            neo550ShooterLoadRoller.set(speed);
        }

        SmartDashboard.putNumber("LoadRoller Velocity", 
                                 neo550ShooterLoadRollerEncoder.getVelocity());
    }

}
