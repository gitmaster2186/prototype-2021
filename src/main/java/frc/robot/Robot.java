/*
 * TBD:
 *    - change drivetrain motors to NEOs
 *    - verify the vision alignment code
 *    - vision alighnment mode 
 *      - just advise driver with dashboard info
 *      - implement angle alignment mode
 *      - add distance adjustment mode
 *    - put actual code in stub routines
 *    - get agreement on button assignments
 *
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

// import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
// import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
//import org.photonvision.PhotonCamera;
// import org.photonvision.common.hardware.VisionLEDMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    class TankSpeeds
    {
        double leftSpeed;
        double rightSpeed;
        double targets;
        // calculate tankspeed based on input
        TankSpeeds(double x, double y)
        {
            this.leftSpeed = x;
            this.rightSpeed = y;
            this.targets = 0;
        }
    }

    public TankSpeeds getTankSpeeds(double x, double y, double z)
    {
        return new TankSpeeds(x, y);
    }

    public TankSpeeds getManualTankSpeed()
    {
        return getTankSpeeds(-leftJoystick.getY(),
                             -rightJoystick.getY(),
                             0);
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
      
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable limeLightTable = inst.getTable("limelight");
    
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK);
    Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK);
    Joystick altJoystick = new Joystick(Constants.ALT_JOYSTICK);
    
    // XboxController xboxController = new XboxController(Constants.DRIVER_XBOX_CONTROLLER);

    // Drive motors
    private CANSparkMax neoDriveTrainFrontLeft    = new CANSparkMax(Constants.SPARK_NEO_1_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neoDriveTrainFrontRight   = new CANSparkMax(Constants.SPARK_NEO_2_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neoDriveTrainRearLeft     = new CANSparkMax(Constants.SPARK_NEO_3_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neoDriveTrainRearRight    = new CANSparkMax(Constants.SPARK_NEO_4_CAN_ID, MotorType.kBrushless);
    DifferentialDrive drive;

    private CANSparkMax neo550ShooterFrontIntake  = new CANSparkMax(Constants.SPARK_NEO550_1_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterRearIntake   = new CANSparkMax(Constants.SPARK_NEO550_2_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterLoadRoller   = new CANSparkMax(Constants.SPARK_NEO550_3_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterTurret       = new CANSparkMax(Constants.SPARK_NEO550_4_CAN_ID, MotorType.kBrushless);
    private WPI_TalonFX falcon500ShooterFlyWheel1 = new WPI_TalonFX(Constants.FALCON500_1_CAN_ID);
    private WPI_TalonFX falcon500ShooterFlyWheel2 = new WPI_TalonFX(Constants.FALCON500_2_CAN_ID);
  
    private WPI_TalonFX falcon500Climber1         = new WPI_TalonFX(Constants.FALCON500_3_CAN_ID);
    private WPI_TalonFX falcon500Climber2         = new WPI_TalonFX(Constants.FALCON500_4_CAN_ID);
  
    //private Timer flyWheelFireTimer = new Timer();
    //private boolean flyWheelFireActive = false;
    private boolean shooterActive = false;
    private double startTime = 0;
    private int startFireTurretTimedCounter = 0;
    private int stopFireTurretTimedCounter = 0;
    private int startFireTurretCounter = 0;
    private int stopFireTurretCounter = 0;

    @Override
    public void robotInit()
    {
        /* set the motors to factory default values */

        neo550ShooterFrontIntake.restoreFactoryDefaults();
        neo550ShooterRearIntake.restoreFactoryDefaults();
        neo550ShooterLoadRoller.restoreFactoryDefaults();
        neo550ShooterTurret.restoreFactoryDefaults();

        falcon500ShooterFlyWheel1.configFactoryDefault();
        falcon500ShooterFlyWheel2.configFactoryDefault();
        falcon500Climber1.configFactoryDefault();
        falcon500Climber2.configFactoryDefault();

        neoDriveTrainFrontLeft.restoreFactoryDefaults();
        neoDriveTrainFrontRight.restoreFactoryDefaults();
        neoDriveTrainRearLeft.restoreFactoryDefaults();
        neoDriveTrainRearRight.restoreFactoryDefaults();

        neoDriveTrainRearLeft.follow(neoDriveTrainFrontLeft);
        neoDriveTrainRearRight.follow(neoDriveTrainFrontRight);
        drive = new DifferentialDrive(neoDriveTrainFrontLeft, 
                                      neoDriveTrainFrontRight);

    }
     
    @Override
    public void teleopInit() {    
        limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF); // leds off
    }

/*
    // remap inVal from in min and max to out in min max ranges
    double map(double inVal, double in_min, double in_max, double out_min, double out_max)
    {
      return (inVal - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
*/

    private void startIntake()
    {
        System.out.println("startIntake");
        neo550ShooterFrontIntake.set(Constants.INTAKE_ON);
        neo550ShooterRearIntake.set(Constants.INTAKE_ON);
    }

    private void stopIntake()
    {
      System.out.println("stopIntake");
      neo550ShooterFrontIntake.set(Constants.INTAKE_OFF);
      neo550ShooterRearIntake.set(Constants.INTAKE_OFF);
  }

    private void RotateTurretLeft()
    {
        System.out.println("RotateTurretLeft");
    }

    private void RotateTurretRight()
    {
        System.out.println("RotateTurretRight");
    }

    // !!!SID!!! - this ain't right!!!

    // a timer based shooter 
    private void fireTurretTimed(double fireTurrentTimeSeconds)
    {
        // are we starting a new shot?
        if (shooterActive == false)
        {
            startTime =  Timer.getFPGATimestamp(); // get sys time in seconds
            startFireTurret();
            startFireTurretTimedCounter += 1;
            SmartDashboard.putNumber("startFireTurretTimedCounter", startFireTurretTimedCounter);
        }

        double nowTime = Timer.getFPGATimestamp();

        // are we done (timer expired)?
        if ((nowTime - startTime) >= fireTurrentTimeSeconds)
        {
            stopFireTurret();
            stopFireTurretTimedCounter += 1;
            SmartDashboard.putNumber("stopFireTurretTimedCounter", stopFireTurretTimedCounter);
        }
    }

    // !!!SID!!! TBD - don't fire until flywheels are up to speed
    private void startFireTurret()
    {
        //System.out.println("######FireTurret");
        falcon500ShooterFlyWheel1.set(Constants.FLYWHEEL_ON);
        falcon500ShooterFlyWheel2.set(Constants.FLYWHEEL_ON);
        shooterActive = true;
        startFireTurretCounter += 1;
        SmartDashboard.putNumber("startFireTurretCounter", startFireTurretCounter);
    }
    
    private void stopFireTurret()
    {
        if (shooterActive == true)
        {
            //System.out.println("@@@@@@@@stopFireTurret");
            falcon500ShooterFlyWheel1.set(Constants.FLYWHEEL_OFF);
            falcon500ShooterFlyWheel2.set(Constants.FLYWHEEL_OFF);
            shooterActive = false;
            stopFireTurretCounter += 1;
            SmartDashboard.putNumber("stopFireTurretCounter", stopFireTurretCounter);
        }
    }

    private void Deployintake()
    {
        System.out.println("DeployIntake");
    }

    // run for a specified time to load ball into firing position
    // do not load ball if flywheels are not up to speed
    // 
    private void InternalRollers(boolean buttonPressed)
    {
        
        // !!!SID!!! FIX ME!!!! how do we get the motor speed of a falcon500?
        if (buttonPressed == true){
            neo550ShooterLoadRoller.set(Constants.LOADER_SPEED_ON);
            // shooterActive = true;
            System.out.println("InternalRollers True");
        }
        else {
            // shooterActive = false;
            //  !!!SID!!! FIX ME!!!! make stop faster
            neo550ShooterLoadRoller.set(Constants.LOADER_SPEED_OFF);
            System.out.println("InternalRollers False");
        }


    }

    // Vision-alignment mode
    void aimAssist(TankSpeeds tankSpeed)
    {
        double x = 0.0;
        double y = 0.0;
        double area = 0.0;

        System.out.println("assist mode");

        limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_ON); // leds on
        NetworkTableEntry tv = limeLightTable.getEntry(Constants.LIMELIGHT_VALID_TARGETS);
        tankSpeed.targets = tv.getDouble(0.0);

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
        } else {
            // If we have no targets, don't do anything
            if (altJoystick.getRawButton(Constants.DEBUG_BUTTON)) {
                System.out.println("visionAlignmentTarget -- no targets -- ");
            }
        }

        //post to smart dashboard
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightTargets", tankSpeed.targets);

    }

    @Override
    // gets called 50 times a second
    public void teleopPeriodic() {
        TankSpeeds tankSpeed = getManualTankSpeed();

        /* !!!SID!!! - review each of these. Do we want to call
         * getRawButton, getRawButtonPressed or getRawButtonReleased?
         */


        if (altJoystick.getRawButtonPressed(Constants.TOGGLE_INTAKE)){
            startIntake();
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_INTAKE)){
            stopIntake();
        }

        if (altJoystick.getRawButton(Constants.TURN_TURRET_LEFT)) {
            RotateTurretLeft();
        }

        if (altJoystick.getRawButton(Constants.TURN_TURRET_RIGHT)){
            RotateTurretRight();
        }

        // as long as the trigger is pushed keep firing
        if(altJoystick.getRawButton(Constants.FIRE_TURRET))
        {
            startFireTurret();
        }
        else
        {
            stopFireTurret();
        }

        // returns true if the button is being held down
        // at the time that this method is being called
        if(altJoystick.getRawButton(Constants.FIRE_TURRET_TIMED))
        {
            fireTurretTimed(1.0);
        }

        if(altJoystick.getRawButton(Constants.DEPLOY_INTAKE)){
            Deployintake();
        }

        if (altJoystick.getRawButtonPressed(Constants.TOGGLE_INTERNAL_ROLLER)){
            InternalRollers(true);
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_INTERNAL_ROLLER))
        {
            InternalRollers(false);
        }

        if (altJoystick.getRawButton(Constants.TOGGLE_TURRET_AIM_ASSIST)) {
            aimAssist(tankSpeed);
        } 
        else
        {
            // if we're not aiming the turret then make sure the LEDs are off
            limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF);
        }

        if (altJoystick.getRawButton(Constants.DEBUG_BUTTON)) {
            System.out.println("manual -- leftSpeed: " + 
                                tankSpeed.leftSpeed + 
                                " rightSpeed: " + 
                                tankSpeed.rightSpeed);
        }

        SmartDashboard.putNumber("leftSpeed", tankSpeed.leftSpeed);
        SmartDashboard.putNumber("rightSpeed", tankSpeed.rightSpeed);
        SmartDashboard.putNumber("targets", tankSpeed.targets);

        drive.tankDrive(tankSpeed.leftSpeed, tankSpeed.rightSpeed);
    }
}
