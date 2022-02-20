package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake {
    private CANSparkMax neo550ShooterFrontIntake;
    private CANSparkMax neo550ShooterRearIntake;
    private RelativeEncoder neo550ShooterFrontIntakeEncoder;

    public Intake(CANSparkMax inNeo550ShooterFrontIntake,
                  CANSparkMax inNeo550ShooterRearIntake,
                  RelativeEncoder inNeo550ShooterFrontIntakeEncoder)
    {
        neo550ShooterFrontIntake = inNeo550ShooterFrontIntake;
        neo550ShooterRearIntake  = inNeo550ShooterRearIntake;
        neo550ShooterFrontIntakeEncoder = inNeo550ShooterFrontIntakeEncoder;
    }

    /*
     * toggle the intakes on or off
     * 
     * objects used: neo550ShooterFrontIntake, 
     *               neo550ShooterRearIntake, 
     *               neo550ShooterFrontIntakeEncoder
     */
    public void toggle(boolean toggleOn)
    {
        System.out.println("intakeToggle");
        double speed = Constants.INTAKE_ON;

        if (toggleOn == false)
        {
            speed = Constants.INTAKE_OFF;
        }

        neo550ShooterFrontIntake.set(speed);
        neo550ShooterRearIntake.set(speed);

        SmartDashboard.putNumber("FrontIntake Velocity", 
                                 neo550ShooterFrontIntakeEncoder.getVelocity());
    }   

}
