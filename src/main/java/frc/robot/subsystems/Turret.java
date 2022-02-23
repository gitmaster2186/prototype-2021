package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;

public class Turret {
    private CANSparkMax neo550ShooterTurret;

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
        neo550ShooterTurret.set(speed);
    }
}
