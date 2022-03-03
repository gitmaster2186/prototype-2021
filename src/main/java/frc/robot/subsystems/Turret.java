package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants;

public class Turret {
    private CANSparkMax neo550ShooterTurret;
    private SlewRateLimiter turretfilter = new SlewRateLimiter(Constants.TURRET_RAMP_UP_POWER);
    private RelativeEncoder neo550ShooterTurretEncoder;   

    /*
     * rotate the turret at the specified speed
     * 
     * objects used: neo550ShooterTurret
     */
    public Turret(CANSparkMax inNeo550ShooterTurret)
    {
        neo550ShooterTurret = inNeo550ShooterTurret;

        neo550ShooterTurret.restoreFactoryDefaults();

        // try to limit the number of times the turret can turn
        neo550ShooterTurret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        neo550ShooterTurret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        neo550ShooterTurret.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 1);
        neo550ShooterTurret.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1);

        neo550ShooterTurret.setIdleMode(CANSparkMax.IdleMode.kBrake);
       
        neo550ShooterTurretEncoder  = neo550ShooterTurret.getEncoder(); 
        neo550ShooterTurretEncoder.setPosition(0);
        // gear ratios -- 28:1, 12.5

    }

    public void rotate(double speed)
    {
        double fltSpeed = 0.0;

        System.out.println("turretRotate");
        if (speed != 0.0)
        {
            fltSpeed = turretfilter.calculate(speed);
        }
        neo550ShooterTurret.set(fltSpeed);
        
        if (speed == 0.0)
        {
            neo550ShooterTurretEncoder.setPosition(0);
        }
    }
}
