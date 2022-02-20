package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Climber {
    private WPI_TalonFX falcon500Climber1;
    private WPI_TalonFX falcon500Climber2;

    public Climber(WPI_TalonFX inFalcon500Climber1,
                   WPI_TalonFX inFalcon500Climber2)
    {
        falcon500Climber1 = inFalcon500Climber1;
        falcon500Climber2 = inFalcon500Climber2;
    }
     
    /*
     * turn climbers on or off
     * 
     * objects used: falcon500Climber1, 
     *               falcon500Climber2
     */
    public void toggle(boolean buttonPressed)
    {
       double speed = Constants.CLIMBER_SPEED_OFF;
   
        if (buttonPressed == true)
        {
           speed = Constants.CLIMBER_SPEED_ON;
        }
        falcon500Climber1.set(speed);
        falcon500Climber2.set(speed);
   
        SmartDashboard.putNumber("climber 1 Velocity", 
                                  falcon500Climber1.getSelectedSensorVelocity());
        SmartDashboard.putNumber("climber 2 Velocity", 
                                  falcon500Climber2.getSelectedSensorVelocity());
        System.out.println("climbers " + buttonPressed);
    }
}
