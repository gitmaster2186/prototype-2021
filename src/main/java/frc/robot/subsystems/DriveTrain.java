package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
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
                              
        neoDriveTrainRearLeft.follow(neoDriveTrainFrontLeft);
        neoDriveTrainRearRight.follow(neoDriveTrainFrontRight);

        drive = new DifferentialDrive(neoDriveTrainFrontLeft, 
                                      neoDriveTrainFrontRight);                              
        leftDtfilter = new SlewRateLimiter(0.25);
        rightDtfilter = new SlewRateLimiter(0.25);

    }

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
            x = tx.getDouble(0.0);
            y = ty.getDouble(0.0);
            area = ta.getDouble(0.0);

            double heading_error = -x;
            double distance_error = -y;
            double steering_adjust = 0.0f;
            double KpAim = -0.1f;
            double KpDistance = -0.1f;
            double min_aim_command = 0.05f;

            if (x > 1.0)
            {
                steering_adjust = KpAim*heading_error - min_aim_command;
            }
            else if (x < 1.0)
            {
                steering_adjust = KpAim*heading_error + min_aim_command;
            }
    
            double distance_adjust = KpDistance * distance_error;
    
            tankSpeed.leftSpeed += steering_adjust + distance_adjust;
            tankSpeed.rightSpeed -= steering_adjust + distance_adjust;
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
            limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF);

            xl = leftDtfilter.calculate(tankSpeed.leftSpeed);
            xr = rightDtfilter.calculate(tankSpeed.rightSpeed);
        }


        SmartDashboard.putNumber("xl", xl);
        SmartDashboard.putNumber("xr", xr);

        drive.tankDrive(xl, xr);
    }

}
