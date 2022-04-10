package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.OurTimer;
import frc.robot.utils.TankSpeeds;

public class DriveTrain {

    // !!!SID!!! XXX - TBD - tune distance and steering limits
    //private static final double TARGET_DISTANCE_LIMIT = 1.0;
    //private static final double TARGET_STEERING_LIMIT = 1.0;

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
    //private SlewRateLimiter leftDtfilter;
    //private SlewRateLimiter rightDtfilter;
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable limeLightTable = inst.getTable("limelight");
    private OurTimer dtTimer;
    private int limeAreaTooSmall = 0;
    private int limeAreaOk = 0;
    private int limeNoTarget = 0;
    double prevLeft = 0;
    double prevRight = 0;
    boolean flipped = false;

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

        // !!!SID!!! - XXX - 03/19/22 - old way of getting robot in correct direction
        // neoDriveTrainFrontRight.setInverted(true);
        // neoDriveTrainRearRight.setInverted(true);
        // !!!SID!!! - XXX - 03/19/22 - now go the other way
        neoDriveTrainFrontLeft.setInverted(true);
        neoDriveTrainRearLeft.setInverted(true);

        // NEO motor specs: Hall-Sensor Encoder Resolution: 42 counts per rev
        neoDtFrontLeftEncoder  = neoDriveTrainFrontLeft.getEncoder();   
        neoDtFrontRightEncoder  = neoDriveTrainFrontRight.getEncoder();   
        drive = new DifferentialDrive(neoDriveTrainFrontLeft, 
                                      neoDriveTrainFrontRight); 
        
        // this value (0.2) is actually the default for the deadband.
        // uncomment the next line to change the default deadband
        // drive.setDeadband(0.02);
                             
        //leftDtfilter = new SlewRateLimiter(Constants.DRIVE_TRAIN_RAMP_UP_POWER);
        //rightDtfilter = new SlewRateLimiter(Constants.DRIVE_TRAIN_RAMP_UP_POWER);
        dtTimer = new OurTimer();
        prevLeft = 0;
        prevRight = 0;
    
    }
/*
 *      tx	Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
 *      ta	Target Area (0% of image to 100% of image)  
 */

    public boolean aimAssist(TankSpeeds tankSpeed)
    {
        boolean gotTarget = false;
//        double Kp = -0.03;
//       double Kp = -0.04;
       double Kp = 0.04;
//        double Kp = -0.1;
        //double min_command = 0.05;
        double min_command = 0.11;

        NetworkTableEntry tv = limeLightTable.getEntry(Constants.LIMELIGHT_VALID_TARGETS);
        tankSpeed.targets = tv.getDouble(0.0);

        // System.out.println("assist mode");
        if (tankSpeed.targets <= 0)
        {
            limeNoTarget += 1;
            SmartDashboard.putNumber("limeNoTarget", limeNoTarget);

            tankSpeed.leftSpeed  = prevLeft;
            tankSpeed.rightSpeed = prevRight;
        }
        else
        {
            NetworkTableEntry ta = limeLightTable.getEntry(Constants.LIMELIGHT_TARGET_AREA);
            double area = ta.getDouble(0.0);

            if (area <=  Constants.VISION_MIN_AREA)
            {
                limeAreaTooSmall += 1;
                SmartDashboard.putNumber("limeAreaTooSmall", limeAreaTooSmall);

                tankSpeed.leftSpeed  = prevLeft;
                tankSpeed.rightSpeed = prevRight;
            }
            else
            {
                gotTarget = true;
                limeAreaOk += 1;
                SmartDashboard.putNumber("limeAreaOk", limeAreaOk);

                //read values periodically  
                // !!!SID!!! XXX - should we be using raw values instead?      
                NetworkTableEntry tx = limeLightTable.getEntry(Constants.LIMELIGHT_HORIZONTAL_OFFSET); 
                double x = tx.getDouble(0.0); 
                
                // -29.8 to 29.8 degrees
                /*
                    29.8 * 0.1 - 0.05 = 2.93 // this can't be right
                    29.8 * 0.035 - 0.05 = 0.993

                    1.0  * 0.1 - 0.05 = 0.05 OR 1.0 * 0.1 + 0.05 = 0.15
                */
                double heading_error = -x;
                double steering_adjust = 0.0f;
                //double minAdj = 1.0;
                //double minAdj = 0.5;
                double minAdj = 0.3;

                SmartDashboard.putNumber("limX", x);
                if (x > minAdj)
                {
                    steering_adjust = Kp*heading_error - min_command;
                }
                else if (x < minAdj)
                {
                    steering_adjust = Kp*heading_error + min_command;
                }
                SmartDashboard.putNumber("leftBefore", tankSpeed.leftSpeed);
                SmartDashboard.putNumber("rightBefore", tankSpeed.rightSpeed);
                SmartDashboard.putNumber("steering_adjust", steering_adjust);

                tankSpeed.leftSpeed  += steering_adjust;
                tankSpeed.rightSpeed -= steering_adjust;
                prevLeft = tankSpeed.leftSpeed;
                prevRight = tankSpeed.rightSpeed;
                SmartDashboard.putNumber("leftAfter", tankSpeed.leftSpeed);
                SmartDashboard.putNumber("rightAfter", tankSpeed.rightSpeed);

                // !!!SID!!! - XXX - debug
                NetworkTableEntry txx = limeLightTable.getEntry("tx"); 
                double xx = txx.getDouble(0.0);
                SmartDashboard.putNumber("tx", xx);
                NetworkTableEntry tx0 = limeLightTable.getEntry("tx0"); 
                double x0 = tx0.getDouble(0.0);
                SmartDashboard.putNumber("tx0", x0);
                NetworkTableEntry tx1 = limeLightTable.getEntry("tx1"); 
                double x1 = tx1.getDouble(0.0);
                SmartDashboard.putNumber("tx1", x1);
            }
        }
        SmartDashboard.putBoolean("gotTarget", gotTarget);
        SmartDashboard.putNumber("tx", tankSpeed.leftSpeed);

        return gotTarget;
    }

    // Vision-alignment mode
    public boolean aimAndDistance(TankSpeeds tankSpeed)
    {
        boolean gotTarget = false;

        // System.out.println("assist mode");

        //read values periodically  
        // !!!SID!!! XXX - should we be using raw values instead?      
        NetworkTableEntry ta = limeLightTable.getEntry(Constants.LIMELIGHT_TARGET_AREA);
        NetworkTableEntry tx = limeLightTable.getEntry(Constants.LIMELIGHT_HORIZONTAL_OFFSET);
        NetworkTableEntry ty = limeLightTable.getEntry(Constants.LIMELIGHT_VERTICAL_OFFSET);
        NetworkTableEntry tv = limeLightTable.getEntry(Constants.LIMELIGHT_VALID_TARGETS);

        double area = ta.getDouble(0.0);
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        tankSpeed.targets = tv.getDouble(0.0);

        if (tankSpeed.targets <= 0)
        {
            limeNoTarget += 1;
            tankSpeed.leftSpeed  = prevLeft;
            tankSpeed.rightSpeed = prevRight;
        }
        else
        {
        
            if (area <=  Constants.VISION_MIN_AREA)
            {
                limeAreaTooSmall += 1;
                tankSpeed.leftSpeed  = prevLeft;
                tankSpeed.rightSpeed = prevRight;
            }
            else
            {
                double heading_error = -x;
                double distance_error = -y;
                double steering_adjust = 0.0f;

                limeAreaOk += 1;

                // !!!SID!!! XXX - these 3 constants need to be tuned to our robot

                // Beware, if you set KpAim or min_aim_command too high, your 
                // robot can become unstable and can oscillate back and forth 
                // as it overshoots the target.
                // What happens if you set them too low? 
                //    Changes too slowly or not at all?
                //
                double KpAim = -0.05f; // original value
                // double KpAim = -0.01f; // 3/5/22 15:00

                double KpDistance = -0.1f; // original value

                // minimum amount of power needed for the robot to actually move 
                // (you actually want to use a little bit less than this).
                double min_aim_command = 0.05f; // original value

                if (x > 1.0)
                {
                    steering_adjust = KpAim*heading_error - min_aim_command;
                }
                else if (x < 1.0)
                {
                    steering_adjust = KpAim*heading_error + min_aim_command;
                }
        
                double distance_adjust = KpDistance * distance_error;
                
                // are we close enough to the target?
                // !!!SID!!! XXX - reenable distance test
                // if ((Math.abs(distance_adjust) < TARGET_DISTANCE_LIMIT) && 
                //     (Math.abs(steering_adjust) < TARGET_STEERING_LIMIT)) 
                {
                    gotTarget = true;
                }

                tankSpeed.leftSpeed  += (steering_adjust + distance_adjust);
                tankSpeed.rightSpeed -= (steering_adjust + distance_adjust);

                // !!!SID!!! XXX - 3/5/22 - hack - stop running away?
                // tankSpeed.leftSpeed  = -tankSpeed.leftSpeed;
                // tankSpeed.rightSpeed = -tankSpeed.rightSpeed;

                // remember the new values
                prevLeft = tankSpeed.leftSpeed;
                prevRight = tankSpeed.rightSpeed;

            }
        }
    
        //post to smart dashboard
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightTargets", tankSpeed.targets);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("limeAreaOk", limeAreaOk);
        SmartDashboard.putNumber("limeAreaTooSmall", limeAreaTooSmall);
        SmartDashboard.putNumber("limeNoTarget", limeNoTarget);

        return (gotTarget);
    }


    // move a specified distance.
    // return true if done or false if not done traveling.
    // We need gearbox ratios, encoder values for distance of one foot (), and diameter of wheels.
    /*
    public boolean distanceMove(double distanceToTravel)
    {
        TankSpeeds ts = new TankSpeeds(0.5, 0.5);
        double lPos = neoDtFrontLeftEncoder.getPosition() * kDriveTick2Feet;   
        double rPos = neoDtFrontRightEncoder.getPosition() * kDriveTick2Feet;   
        double calcDist = (lPos + rPos) / 2;

        if (calcDist >= distanceToTravel)
        {
            ts.leftSpeed = 0;
            ts.rightSpeed = 0;
        }

        tankDrive(ts, false);

        return false;
    }
*/

    public void timerInit()
    {
        dtTimer.initTimer();
    }

    // drive for the specified time the specified speed
    // timer must be initialized before the 1st call with timerInit.
    // return timer expired state
    public boolean timedMove(double time, double speedLeft, double speedRight, boolean autoMode)
    {
        boolean ret = dtTimer.timerTest(time);

        // has the timer expired?
        if (ret == false)
        {
            TankSpeeds ts = new TankSpeeds(speedLeft, speedRight);
            
            // go forward/backward
            tankDrive(ts, false, autoMode);
        }
        return ret;
    }

    // Flip the driver flipped boolean.
    // if flipped is true then drive reversed.
    public void flip()
    {
        flipped = !flipped;
    }

    /*
     * !!!SID!!! XXX - HACK - experimenting with adjusting drivetrain speeds side-to-side
     * 
     * problem - the robot seems faster on one side. This causes problems driving straight.
     *   - initial thought this is a drivetrain issue
     *   - have we eliminated the joysticks as the source of the problem?
     *   - which side needs to be adjusted?
     *     hack from yesterday: drive.tankDrive(xl, (xr+0.2)); -- right side
     * 
     * Adjustment:
     *   - 1st version will add or subtract a constant from one side
     *   - can we make a smarter version that can measure the motor speeds and 
     *     adjust accordingly?
     * 
     * cases to handle:
     * - 0 speed - don't adjust if near zero - like a deadband
     *             If we don't handle this the robot will move when it should have stopped.
     * - forward and backward have to be calculated differently i.e. a simple subtract
     *   might make forward better but makes backwards worse.
     * - initially hardcode the side to be changed and the constant
     * - make it adjust left or right based on input parameters
     * 
     */
    // private void alignSpeeds(TankSpeeds tankSpeed)
    // {
    //     // equiv to yesterday's dumb hack
    //     tankSpeed.rightSpeed += 0.2;
    // }

    // !!!SID!!! XXX - 4/1/22 - debug - smooth drivetrain speeds
    private void smoothSpeeds(TankSpeeds tankSpeed)
    {
        boolean posSign = true;

        // don't do ramp-up up/down
        // tankSpeed.leftSpeed = leftDtfilter.calculate(tankSpeed.leftSpeed);
        // tankSpeed.rightSpeed = rightDtfilter.calculate(tankSpeed.rightSpeed);
        
        if (tankSpeed.leftSpeed < 0)
        {
            posSign= false;
        }
        tankSpeed.leftSpeed *= tankSpeed.leftSpeed;
        if (posSign == false)
        {
            tankSpeed.leftSpeed = -tankSpeed.leftSpeed;
        }
        posSign = true;
        if (tankSpeed.rightSpeed < 0)
        {
            posSign= false;
        }
        tankSpeed.rightSpeed *= tankSpeed.rightSpeed;
        if (posSign == false)
        {
            tankSpeed.rightSpeed = -tankSpeed.rightSpeed;
        }
    }

    private static final double MIN_ADJ_SPEED = 0.2;
    private static final double ADJ_SPEED_RATIO = 0.875;
    private void alignSpeeds(TankSpeeds tankSpeed, boolean adjRight)
    {
        // only adjust if not stopped or moving slow
        if ((Math.abs(tankSpeed.leftSpeed) > MIN_ADJ_SPEED) &&
            (Math.abs(tankSpeed.rightSpeed) > MIN_ADJ_SPEED))
        {

            // which side are we adjusting?
            if (adjRight == true)
            {
                tankSpeed.rightSpeed *= ADJ_SPEED_RATIO;
            }
            else
            {
                tankSpeed.leftSpeed *= ADJ_SPEED_RATIO;
            }
        }

    }

    /* 
     * Our version of tankdrive. Different modes to modify the tank drive left and right speeds.
     * It calls the real (wpilib) version of tank drive when we done messing with the left 
     * and right speeds.
     * 
     * Modes:
     *  driver assist mode - use the limelight to align with the target, no aligned sides
     *  jflip mode - driver controlled, drive backward like forward  - flip aligned sides
     *  autonomous mode - not driver controlled, no aligned sides
     *  no mode - driver controlled, not autonomous, not jflip, not driver assist - just align sides
     */
    public void tankDrive(TankSpeeds tankSpeed, 
                          boolean driverAssistMode,
                          boolean autonomousMode)
    {
        double xl = 0;
        double xr = 0;

        SmartDashboard.putNumber("raw Right", tankSpeed.rightSpeed);
        SmartDashboard.putNumber("raw Left", tankSpeed.leftSpeed);

        if (driverAssistMode == true)
        {
            //limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_ON); // leds on

            // we're intentionally ignoring the return value

            // !!!SID!!! XXX - adjust slower
            tankSpeed.leftSpeed = 0;
            tankSpeed.rightSpeed = 0;

            // only adjust the aim (not distance)
            aimAssist(tankSpeed);
            xl = tankSpeed.leftSpeed;
            xr = tankSpeed.rightSpeed;
        }
        else
        {
            // not driver assist mode driving

            // if we're not aiming the turret then make sure the LEDs are off
            //limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF);


            // flip only makes sense when the driver is in control
            if (flipped == true)
            {
                // drive like the back is the front when in flipped mode

                double tmp = tankSpeed.leftSpeed;
                tankSpeed.leftSpeed = -tankSpeed.rightSpeed;
                tankSpeed.rightSpeed = -tmp;

                // !!!SID!!! XXX - disabled 4/2/22 
                // smoothSpeeds(tankSpeed);        

                // adjust the speed if necessary 
                // 2nd parameter is true if right side, false if left
                // !!!SID!!! XXX - left or right adjustment needed?
                alignSpeeds(tankSpeed, false); 
            }
            else
            {
                // only mess with speeds if we're not in autonomous mode
                if (autonomousMode == false)
                {
                    // !!!SID!!! XXX - smoothSpeeds disabled 4/2/22
                    // smoothSpeeds(tankSpeed);        

                    alignSpeeds(tankSpeed, true); 
                }
            }

            xl = tankSpeed.leftSpeed;
            xr = tankSpeed.rightSpeed;
        }
        
        SmartDashboard.putBoolean("isFlipped", flipped);

        SmartDashboard.putNumber("xl", xl);
        SmartDashboard.putNumber("xr", xr);
        SmartDashboard.putNumber("x Diff", (xr-xl));
        
        drive.tankDrive(xl, xr);
    }
}
        // Constants such as camera and target height stored. Change per robot and goal!
        // final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        // final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
        // // Angle between horizontal and the camera.
        // final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    
        // // How far from the target we want to be
        // final double GOAL_RANGE_METERS = Units.feetToMeters(3);
    
        // // PID constants should be tuned per robot
        // final double LINEAR_P = 0.1;
        // final double LINEAR_D = 0.0;
        // PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    
        // final double ANGULAR_P = 0.1;
        // final double ANGULAR_D = 0.0;
    
    
        // final double INCREMENT = 0.25;
        // final double LOW_IN_LIMIT = -1.0;
        // final double HIGH_IN_LIMIT = 1.0;
        
        // final double LOW_OUT_LIMIT_1 = 0.0;
        // final double HIGH_OUT_LIMIT_1 = 360.0;
        
        // final double LOW_OUT_LIMIT_2 = -180.0;
        // final double HIGH_OUT_LIMIT_2 = 180.0;
        // PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
        // SmartDashboard.putNumber("leftPos", 
        //                          neoDtFrontLeftEncoder.getPosition());
        // SmartDashboard.putNumber("rightPos", 
        //                         neoDtFrontRightEncoder.getPosition());
