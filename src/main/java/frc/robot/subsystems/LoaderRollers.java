package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;

// import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;

public class LoaderRollers {
    private CANSparkMax neo550ShooterLoaderRollerFront;
    private CANSparkMax neo550ShooterLoaderRollerBack;
    FlyWheel flyWheel;

    //private RelativeEncoder neo550ShooterLoaderRollerEncoder;
    //private SlewRateLimiter loaderFilter = new SlewRateLimiter(Constants.LOADER_RAMP_UP_POWER);
    int dbgCount = 0;
  
    public LoaderRollers(CANSparkMax inneo550ShooterLoaderRollerFront,
                         CANSparkMax inneo550ShooterLoaderRollerBack,
                         FlyWheel fly)
    {
        // we need the fly wheels because we need to check it is up to speed
        // before turning on the load roller.
        neo550ShooterLoaderRollerFront = inneo550ShooterLoaderRollerFront;
        neo550ShooterLoaderRollerBack = inneo550ShooterLoaderRollerBack;

        neo550ShooterLoaderRollerFront.restoreFactoryDefaults();
        neo550ShooterLoaderRollerBack.restoreFactoryDefaults();
        flyWheel = fly;
        // neo550 motor specs: Hall-Sensor Encoder Resolution: 42 counts per rev.
        // neo550ShooterLoaderRollerEncoder  = neo550ShooterLoaderRollerFront.getEncoder();   
        neo550ShooterLoaderRollerFront.setInverted(true);

    }

    public void setRollerSpeed(double speed)
    {
    //    double fltSpeed = loaderFilter.calculate(speed);

   // neo550ShooterLoaderRollerFront.set(fltSpeed);
   // neo550ShooterLoaderRollerBack.set(fltSpeed);
    neo550ShooterLoaderRollerFront.set(speed);
    neo550ShooterLoaderRollerBack.set(speed);
    }


}
