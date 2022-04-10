package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Climber {
    private final DoubleSolenoid m_doubleSolenoidLeft;
    private final DoubleSolenoid m_doubleSolenoidRight;
    
    public Climber(DoubleSolenoid doubleSolenoidLeft,
                   DoubleSolenoid doubleSolenoidRight)
    {
        m_doubleSolenoidLeft = doubleSolenoidLeft;
        m_doubleSolenoidRight = doubleSolenoidRight;
    }
     
    /*
     * climb up or down
     * 
     * objects used: m_doubleSolenoidLeft, 
     *               m_doubleSolenoidRight
     */
    public void climb(int direction)
    {  
       Value value = DoubleSolenoid.Value.kReverse;
       if (direction == Constants.CLIMBER_DOWN_DIRECTION)
       {
         value = DoubleSolenoid.Value.kForward;
       }
       SmartDashboard.putString("left  climber", value.toString());
       SmartDashboard.putString("right climber", value.toString());

       m_doubleSolenoidLeft.set(value);
       m_doubleSolenoidRight.set(value);
    }
}
