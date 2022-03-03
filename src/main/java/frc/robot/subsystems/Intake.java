package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake {
    private CANSparkMax neo550ShooterFrontIntake;
    private CANSparkMax neo550ShooterRearIntake;
    private RelativeEncoder neo550ShooterFrontIntakeEncoder;   

    private SlewRateLimiter intakeFilter = new SlewRateLimiter(Constants.INTAKE_RAMP_UP_POWER);
    int calledCount = 0;
    public Intake(CANSparkMax inNeo550ShooterFrontIntake,
                  CANSparkMax inNeo550ShooterRearIntake)
    {
        neo550ShooterFrontIntake = inNeo550ShooterFrontIntake;
        neo550ShooterRearIntake  = inNeo550ShooterRearIntake;
        neo550ShooterFrontIntakeEncoder = neo550ShooterFrontIntake.getEncoder();   
        neo550ShooterFrontIntake.restoreFactoryDefaults();
        neo550ShooterRearIntake.restoreFactoryDefaults();

        
        SmartDashboard.putNumber("FrontIntake position",
                                 neo550ShooterFrontIntakeEncoder.getPosition());

        intakeFilter.reset(0.25);
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
        double fltSpeed = 0.0;
        calledCount += 1;
        if (calledCount == 1)
        {
            intakeFilter.reset(0.25);
        }

        if (toggleOn == true)
        {
            fltSpeed = intakeFilter.calculate(speed);
        }
        else
        {
            // intakeFilter.reset(0.25);
            calledCount = 0;
        }
        neo550ShooterFrontIntake.set(fltSpeed);
        neo550ShooterRearIntake.set(fltSpeed);

        SmartDashboard.putNumber("FrontIntake called", 
                                 calledCount);
        SmartDashboard.putNumber("FrontIntake speed", 
                                 fltSpeed);
        SmartDashboard.putNumber("FrontIntake Velocity", 
                                 neo550ShooterFrontIntakeEncoder.getVelocity());
        SmartDashboard.putNumber("FrontIntake position",
                                 neo550ShooterFrontIntakeEncoder.getPosition());
    }   

}
