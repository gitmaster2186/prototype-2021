package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants;

public class Turret {
    private CANSparkMax neo550ShooterTurret;
    private SlewRateLimiter turretfilter = new SlewRateLimiter(Constants.TURRET_RAMP_UP_POWER);

    /*
     * rotate the turret at the specified speed
     * 
     * objects used: neo550ShooterTurret
     */
    public Turret(CANSparkMax inNeo550ShooterTurret)
    {
        neo550ShooterTurret = inNeo550ShooterTurret;

        neo550ShooterTurret.restoreFactoryDefaults();
    }

    public void rotate(double speed)
    {
        System.out.println("turretRotate");
        double fltSpeed = turretfilter.calculate(speed);
        neo550ShooterTurret.set(fltSpeed);
    }
}
