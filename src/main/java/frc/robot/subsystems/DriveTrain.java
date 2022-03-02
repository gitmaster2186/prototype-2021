package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.TankSpeeds;

public class DriveTrain {
    private CANSparkMax neoDriveTrainFrontLeft;
    private CANSparkMax neoDriveTrainFrontRight;
    private CANSparkMax neoDriveTrainRearLeft;
    private CANSparkMax neoDriveTrainRearRight;
    private DifferentialDrive drive;
    private RelativeEncoder neoDtFrontLeftEncoder;
    private RelativeEncoder neoDtFrontRightEncoder;
    //private RelativeEncoder neo550ShooterRearIntakeEncoder  = neo550ShooterRearIntake.getEncoder();   
    //private RelativeEncoder neo550ShooterTurretEncoder      = neo550ShooterTurret.getEncoder();   
   

    // speed rate limiters for drive train
    // !!!SID!!! XXX - is this needed??                             
    private SlewRateLimiter leftDtfilter;
    private SlewRateLimiter rightDtfilter;
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable limeLightTable = inst.getTable("limelight");

    public DriveTrain(CANSparkMax inNeoDriveTrainFrontLeft,
                      CANSparkMax inNeoDriveTrainFrontRight,
                      CANSparkMax inNeoDriveTrainRearLeft,
                      CANSparkMax inNeoDriveTrainRearRight)
    {
        neoDriveTrainFrontLeft  = inNeoDriveTrainFrontLeft;
        neoDriveTrainFrontRight = inNeoDriveTrainFrontRight;
        neoDriveTrainRearLeft   = inNeoDriveTrainRearLeft;
        neoDriveTrainRearRight  = inNeoDriveTrainRearRight;

        neoDriveTrainFrontLeft.restoreFactoryDefaults();
        neoDriveTrainFrontRight.restoreFactoryDefaults();
        neoDriveTrainRearLeft.restoreFactoryDefaults();
        neoDriveTrainRearRight.restoreFactoryDefaults();
                              
        // the follower will mirror the leader. The follower will spin in the
        // same direction as the leader unless changed by adding an inversion
        // parameter to the follow method.
        REVLibError rc = neoDriveTrainRearLeft.follow(neoDriveTrainFrontLeft);
        if (rc != REVLibError.kOk)
        {
            System.out.println("Error set follow mode rear left");
        }
        rc = neoDriveTrainRearRight.follow(neoDriveTrainFrontRight);
        if (rc != REVLibError.kOk)
        {
            System.out.println("Error set follow mode rear right");
        }

        // NEO motor specs: Hall-Sensor Encoder Resolution: 42 counts per rev
        neoDtFrontLeftEncoder  = neoDriveTrainFrontLeft.getEncoder();   
        neoDtFrontRightEncoder  = neoDriveTrainFrontRight.getEncoder();   

        drive = new DifferentialDrive(neoDriveTrainFrontLeft, 
                                      neoDriveTrainFrontRight);                              
        leftDtfilter = new SlewRateLimiter(Constants.DRIVE_TRAIN_RAMP_UP_POWER);
        rightDtfilter = new SlewRateLimiter(Constants.DRIVE_TRAIN_RAMP_UP_POWER);

    }

        // Constants such as camera and target height stored. Change per robot and goal!
        final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
        // Angle between horizontal and the camera.
        final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    
        // How far from the target we want to be
        final double GOAL_RANGE_METERS = Units.feetToMeters(3);
    
        // PID constants should be tuned per robot
        final double LINEAR_P = 0.1;
        final double LINEAR_D = 0.0;
        PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    
        final double ANGULAR_P = 0.1;
        final double ANGULAR_D = 0.0;
    
    
        final double INCREMENT = 0.25;
        final double LOW_IN_LIMIT = -1.0;
        final double HIGH_IN_LIMIT = 1.0;
        
        final double LOW_OUT_LIMIT_1 = 0.0;
        final double HIGH_OUT_LIMIT_1 = 360.0;
        
        final double LOW_OUT_LIMIT_2 = -180.0;
        final double HIGH_OUT_LIMIT_2 = 180.0;
        PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    
    // Vision-alignment mode
    private void aimAssist(TankSpeeds tankSpeed)
    {
        double x = 0.0;
        double y = 0.0;
        double area = 0.0;

        System.out.println("assist mode");

        NetworkTableEntry tv = limeLightTable.getEntry(Constants.LIMELIGHT_VALID_TARGETS);
        tankSpeed.targets = tv.getDouble(0.0);
        SmartDashboard.putNumber("targets", tankSpeed.targets);

        if (tankSpeed.targets > 0)
        {
            NetworkTableEntry tx = limeLightTable.getEntry(Constants.LIMELIGHT_HORIZONTAL_OFFSET);
            NetworkTableEntry ty = limeLightTable.getEntry(Constants.LIMELIGHT_VERTICAL_OFFSET);
            NetworkTableEntry ta = limeLightTable.getEntry(Constants.LIMELIGHT_TARGET_AREA);
        
            //read values periodically
            x    = tx.getDouble(0.0);
            y    = ty.getDouble(0.0);
            area = ta.getDouble(0.0);
        
            double heading_error = -x;
            double distance_error = -y;
            double steering_adjust = 0.0f;

            // !!!SID!!! XXX - these 3 constantsneed to be tuned to our robot

            // Beware, if you set KpAim or min_aim_command too high, your 
            // robot can become unstable and can oscillate back and forth 
            // as it overshoots the target
            double KpAim = -0.1f;
            // minimum amount of power needed for the robot to actually move 
            // (you actually want to use a little bit less than this).
            double min_aim_command = 0.05f;
            double KpDistance = -0.1f;

            if (x > 1.0)
            {
                steering_adjust = KpAim*heading_error - min_aim_command;
            }
            else if (x < 1.0)
            {
                steering_adjust = KpAim*heading_error + min_aim_command;
            }
    
            double distance_adjust = KpDistance * distance_error;
    
            tankSpeed.leftSpeed  += (steering_adjust + distance_adjust);
            tankSpeed.rightSpeed -= (steering_adjust + distance_adjust);
        }
    
        //post to smart dashboard
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightTargets", tankSpeed.targets);
    }

    public void tankDrive(TankSpeeds tankSpeed, boolean driverAssistMode)
    {
        double xl;
        double xr;

        if (driverAssistMode == true)
        {
            limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_ON); // leds on

            aimAssist(tankSpeed);
            xl = tankSpeed.leftSpeed;
            xr = tankSpeed.rightSpeed;
        }
        else
        {
            // if we're not aiming the turret then make sure the LEDs are off
            //limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF);

            xl = leftDtfilter.calculate(tankSpeed.leftSpeed);
            xr = rightDtfilter.calculate(tankSpeed.rightSpeed);
        }


        SmartDashboard.putNumber("xl", xl);
        SmartDashboard.putNumber("xr", xr);

        SmartDashboard.putNumber("leftPos", 
                                 neoDtFrontLeftEncoder.getPosition());
        SmartDashboard.putNumber("rightPos", 
                                neoDtFrontRightEncoder.getPosition());

        drive.tankDrive(xl, xr);
    }

}
