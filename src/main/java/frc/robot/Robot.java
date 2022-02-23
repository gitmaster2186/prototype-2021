/*
 * TBD:
 *    - use limit switch to determine when ball is in position to load
 *    - cameras?
 *      - vision camera(s)
 *      - do we need a driver camera?
 *    - vision alignment mode phases
 *      1. just advise driver with dashboard info
 *      2. implement angle alignment mode
 *      3. add distance adjustment mode
 *    - don't allow shooting until flywheel is up to speed
 *    - The robot cannot move when we are shooting.
 *      suppress drive when the targeting sequence is activated.
 *    - use color sensors at the input to reject wrong balls.
 *
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LoaderRollers;
import frc.robot.subsystems.Turret;
import frc.robot.utils.TankSpeeds;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{

    public TankSpeeds getManualTankSpeed()
    {
        return new TankSpeeds(-leftJoystick.getY(),
                             -rightJoystick.getY());
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

    Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_USB_PORT);
    Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_USB_PORT);
    Joystick altJoystick = new Joystick(Constants.ALT_JOYSTICK_USB_PORT);
    
    // XboxController xboxController = new XboxController(Constants.DRIVER_XBOX_CONTROLLER);

    // Drive motors
    private CANSparkMax neoDriveTrainFrontLeft    = new CANSparkMax(Constants.SPARK_NEO_DRIVETRAIN_1_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neoDriveTrainFrontRight   = new CANSparkMax(Constants.SPARK_NEO_DRIVETRAIN_2_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neoDriveTrainRearLeft     = new CANSparkMax(Constants.SPARK_NEO_DRIVETRAIN_3_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neoDriveTrainRearRight    = new CANSparkMax(Constants.SPARK_NEO_DRIVETRAIN_4_CAN_ID, MotorType.kBrushless);
    DriveTrain driveTrain  = new DriveTrain(neoDriveTrainFrontLeft,
                                            neoDriveTrainFrontRight,
                                            neoDriveTrainRearLeft,
                                            neoDriveTrainRearRight);

    private CANSparkMax neo550ShooterFrontIntake  = new CANSparkMax(Constants.SPARK_NEO550_INTAKE_1_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterRearIntake   = new CANSparkMax(Constants.SPARK_NEO550_INTAKE_2_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterLoadRoller   = new CANSparkMax(Constants.SPARK_NEO550_ROLLER_1_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterTurret       = new CANSparkMax(Constants.SPARK_NEO550_TURRET_CAN_ID, MotorType.kBrushless);

    private RelativeEncoder neo550ShooterFrontIntakeEncoder = neo550ShooterFrontIntake.getEncoder();   
    //private RelativeEncoder neo550ShooterRearIntakeEncoder  = neo550ShooterRearIntake.getEncoder();   
    private RelativeEncoder neo550ShooterLoadRollerEncoder  = neo550ShooterLoadRoller.getEncoder();   
    //private RelativeEncoder neo550ShooterTurretEncoder      = neo550ShooterTurret.getEncoder();   
   
    private WPI_TalonFX falcon500ShooterFlyWheel1 = new WPI_TalonFX(Constants.FALCON500_SHOOTER_1_CAN_ID);
    private WPI_TalonFX falcon500ShooterFlyWheel2 = new WPI_TalonFX(Constants.FALCON500_SHOOTER_2_CAN_ID);
  
    private WPI_TalonFX falcon500Climber1         = new WPI_TalonFX(Constants.FALCON500_CLIMBER_1_CAN_ID);
    private WPI_TalonFX falcon500Climber2         = new WPI_TalonFX(Constants.FALCON500_CLIMBER_2_CAN_ID);
  
    private Intake intake = new Intake(neo550ShooterFrontIntake, 
                                       neo550ShooterRearIntake, 
                                       neo550ShooterFrontIntakeEncoder);
    private Turret turret = new Turret(neo550ShooterTurret);
    private Climber climber = new Climber(falcon500Climber1,
                                          falcon500Climber2);

    private SlewRateLimiter leftFlyfilter = new SlewRateLimiter(0.25);
    private SlewRateLimiter rightFlyfilter = new SlewRateLimiter(0.25);
                                      

    private BallShooter ballShooter = new BallShooter(falcon500ShooterFlyWheel1, 
                                                      falcon500ShooterFlyWheel2,
                                                      leftFlyfilter,
                                                      rightFlyfilter);

     private LoaderRollers loaderRollers = new LoaderRollers (falcon500ShooterFlyWheel1,
                                                              falcon500ShooterFlyWheel2,
                                                              neo550ShooterLoadRoller,
                                                              neo550ShooterLoadRollerEncoder);

  
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


        // put some motor velocity numbers on dashboard
        SmartDashboard.putNumber("FrontIntake Velocity", 
                                 neo550ShooterFrontIntakeEncoder.getVelocity());
        SmartDashboard.putNumber("LoadRoller Velocity", 
                                 neo550ShooterLoadRollerEncoder.getVelocity());
        SmartDashboard.putNumber("flyWheel 1 velocity", 
                                 falcon500ShooterFlyWheel1.getSelectedSensorVelocity());                         
 }


    @Override
    public void teleopInit() 
    {    
        limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF); // leds off
    }

/*
    // remap inVal from in min and max to out in min max ranges
    double map(double inVal, double in_min, double in_max, double out_min, double out_max)
    {
      return (inVal - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
*/



    // not sure what this is
    // private void Deployintake()
    // {
    //     System.out.println("DeployIntake");
    // }

    @Override
    // gets called 50 times a second
    public void teleopPeriodic() {
        boolean driverAssistMode;
        TankSpeeds tankSpeed = getManualTankSpeed();

        /* !!!SID!!! - review each of these. Do we want to call
         * getRawButton, getRawButtonPressed or getRawButtonReleased?
         */

        // !!!SID!!! - Should we change the turret to use the joystick y axis?
        if (altJoystick.getRawButton(Constants.TURN_TURRET_RIGHT))
        {
            turret.rotate(-Constants.TURRET_ON);
        }
        else if (altJoystick.getRawButtonReleased(Constants.TURN_TURRET_RIGHT))
        {
            turret.rotate(Constants.TURRET_OFF);
        }

        if (altJoystick.getRawButton(Constants.TURN_TURRET_LEFT))
        {
            turret.rotate(Constants.TURRET_ON);
        }
        else if (altJoystick.getRawButtonReleased(Constants.TURN_TURRET_LEFT))
        {
            turret.rotate(Constants.TURRET_OFF);
        }

        // tested - working
        if (altJoystick.getRawButtonPressed(Constants.TOGGLE_INTAKE))
        {
            intake.toggle(true); 
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_INTAKE))
        {
            intake.toggle(false);
        }

        if (altJoystick.getRawButtonPressed(Constants.TOGGLE_LOADER_ROLLERS))
        {
            loaderRollers.toggle(true);
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_LOADER_ROLLERS))
        {
            loaderRollers.toggle(false);
        }

        // as long as the trigger is pushed keep firing
        if(altJoystick.getRawButton(Constants.FIRE_TURRET))
        {
            ballShooter.toggle(true);
        }
        else
        {
            ballShooter.toggle(false);
        }

        // returns true if the button is being held down
        // at the time that this method is being called
        if(altJoystick.getRawButton(Constants.FIRE_TURRET_TIMED))
        {
            // activate the shooter for 1 second
            ballShooter.timed(1.0);
        }

        // !!!SID!!! - this will probably need more than this
        //             for the climber
        if (altJoystick.getRawButtonPressed(Constants.TOGGLE_CLIMBER))
        {
            climber.toggle(true);
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_CLIMBER))
        {
            climber.toggle(false);
        }


        // if(altJoystick.getRawButton(Constants.DEPLOY_INTAKE)){
        //     Deployintake();
        // }


        if (altJoystick.getRawButton(Constants.AIM_ASSIST_BUTTON)) 
        {
            driverAssistMode = true;
        } 
        else
        {
            driverAssistMode = false;
        }

        if (altJoystick.getRawButton(Constants.DEBUG_BUTTON)) 
        {
            System.out.println("leftSpeed: " + 
                                tankSpeed.leftSpeed + 
                                " rightSpeed: " + 
                                tankSpeed.rightSpeed +
                                " driverAssist: " +
                                driverAssistMode);
        }
        SmartDashboard.putNumber("leftSpeed", tankSpeed.leftSpeed);
        SmartDashboard.putNumber("rightSpeed", tankSpeed.rightSpeed);
        SmartDashboard.putBoolean("driverAssistMode", driverAssistMode);

        // move the robot
        driveTrain.tankDrive(tankSpeed, driverAssistMode);
    }
}
