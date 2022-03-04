/*
 * TBD:
 *    - find the flywheel velocity that we want to shoot the ball
 *      - code in FlyWheel.java
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
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.BallShooter;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LoaderRollers;
//import frc.robot.subsystems.Turret;
import frc.robot.utils.TankSpeeds;

import edu.wpi.first.wpilibj.SPI;



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

    AHRS ahrs; // navx 
      
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable limeLightTable = inst.getTable("limelight");
    

    Joystick leftJoystick = new Joystick(Constants.DRIVER_LEFT_JOYSTICK_USB_PORT);
    Joystick rightJoystick = new Joystick(Constants.DRIVER_RIGHT_JOYSTICK_USB_PORT);
    Joystick altJoystick = new Joystick(Constants.ALT_JOYSTICK_USB_PORT);
    
    // XboxController xboxController = new XboxController(Constants.DRIVER_XBOX_CONTROLLER);

    // Drive motors
    private CANSparkMax neoDriveTrainFrontLeft    = new CANSparkMax(Constants.SPARK_NEO_DRIVETRAIN_FRONT_LEFT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neoDriveTrainFrontRight   = new CANSparkMax(Constants.SPARK_NEO_DRIVETRAIN_FRONT_RIGHT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neoDriveTrainRearLeft     = new CANSparkMax(Constants.SPARK_NEO_DRIVETRAIN_BACK_LEFT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neoDriveTrainRearRight    = new CANSparkMax(Constants.SPARK_NEO_DRIVETRAIN_BACK_RIGHT_CAN_ID, MotorType.kBrushless);
    DriveTrain driveTrain  = new DriveTrain(neoDriveTrainFrontLeft,
                                            neoDriveTrainFrontRight,
                                            neoDriveTrainRearLeft,
                                            neoDriveTrainRearRight);

    private CANSparkMax neo550ShooterFrontIntake  = new CANSparkMax(Constants.SPARK_NEO550_FRONT_INTAKE_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterRearIntake   = new CANSparkMax(Constants.SPARK_NEO550_BACK_INTAKE_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterLoaderRollerFront  = new CANSparkMax(Constants.SPARK_NEO550_FRONT_LOADER_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterLoaderRollerBack  = new CANSparkMax(Constants.SPARK_NEO550_BACK_LOADER_CAN_ID, MotorType.kBrushless);
    // private CANSparkMax neo550ShooterTurret       = new CANSparkMax(Constants.SPARK_NEO550_TURRET_CAN_ID, MotorType.kBrushless);

    private WPI_TalonFX falcon500ShooterFlyWheel1 = new WPI_TalonFX(Constants.FALCON500_FRONT_FLYWHEEL_CAN_ID);
    private WPI_TalonFX falcon500ShooterFlyWheel2 = new WPI_TalonFX(Constants.FALCON500_BACK_FLYWHEEL_CAN_ID);
  
    // private WPI_TalonFX falcon500Climber1         = new WPI_TalonFX(Constants.FALCON500_LEFT_CLIMBER_CAN_ID);
    // private WPI_TalonFX falcon500Climber2         = new WPI_TalonFX(Constants.FALCON500_RIGHT_CLIMBER_CAN_ID);
  
    private Intake intake = new Intake(neo550ShooterFrontIntake, 
                                       neo550ShooterRearIntake); 
    // private Turret turret = new Turret(neo550ShooterTurret);
    // private Climber climber = new Climber(falcon500Climber1,
    //                                       falcon500Climber2);


    private FlyWheel flyWheel = new FlyWheel(falcon500ShooterFlyWheel1, 
                                             falcon500ShooterFlyWheel2);

     private LoaderRollers loaderRollers = new LoaderRollers (neo550ShooterLoaderRollerFront,
                                                              neo550ShooterLoaderRollerBack);
    // !!!SID!!! - XXX - TBD
    private BallShooter ballShooter = new BallShooter(intake, loaderRollers, flyWheel, driveTrain);

    private AutonomousMode autoMode;


    int intakeOnCount = 0;
    boolean ledsOn = false;

    @Override
    public void robotInit()
    {
        try {
            /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
            /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
            /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
            // RoboRIO MXP I2C Interface
            ahrs  = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
        ahrs.reset();
        String wpiVerStr = WPILibVersion.Version;
        System.out.println("WPI version " + wpiVerStr);

        // limelight leds off
        limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF); // leds off

        // turn the driver camera on
        CameraServer.startAutomaticCapture();
    }


    @Override
  /**
   * This function is called once at the start of autonomous.
   */
  public void autonomousInit() {
        autoMode = new AutonomousMode(driveTrain, ballShooter);
  }

  /**
   * This function is called periodically (50/sec) during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    autoMode.autoModePeriodic();
  }

    @Override
    public void teleopInit() 
    {    
        limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF); // leds off
//        limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_ON); // leds on
}

/*
    // remap inVal from in min and max to out in min max ranges
    double map(double inVal, double in_min, double in_max, double out_min, double out_max)
    {
      return (inVal - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
*/


    @Override
    // gets called 50 times a second
    public void teleopPeriodic() {
        boolean driverAssistMode;
        
        /*
         * get speed values from joysticks
         * get them now because some cmds/buttons will modify the
         * speed input from the joysticks
         */
        TankSpeeds tankSpeed = getManualTankSpeed(); 

        /* 
         * !!!SID!!! XXX - review each of these. Do we want to call
         * getRawButton, getRawButtonPressed or getRawButtonReleased?
         */

         // toggle limelight leds on/off
         if (altJoystick.getRawButtonPressed(Constants.LIMELIGHT_LEDS_BUTTON))
        {
            if (ledsOn == false)
            {
                limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_ON); // leds on
                ledsOn = true;
            }
            else
            {
                limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF); // leds on
                ledsOn = false;
            }
        }

        // toggle intake motors on/off
        if (altJoystick.getRawButton(Constants.TOGGLE_INTAKE_BUTTON))
        {
            intakeOnCount += 1;
            intake.toggle(true); 
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_INTAKE_BUTTON))
        {
            intake.toggle(false);
        }

        // toggle loader motors on/off
        if (altJoystick.getRawButton(Constants.TOGGLE_LOADER_ROLLERS_BUTTON))
        {
            loaderRollers.toggle(true);
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_LOADER_ROLLERS_BUTTON))
        {
            loaderRollers.toggle(false);
        }

        // toggle flywheel motors on/off
        if (altJoystick.getRawButton(Constants.TOGGLE_FLYWHEELS_BUTTON))
        {
            flyWheel.toggle(true);
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_FLYWHEELS_BUTTON))
        {
            flyWheel.toggle(false);
        }

        /* 
         * toggle aim assist mode on/off
         * This will be input to our tankDrive method
         * to determine if ew're using vision to position
         * the robot.
         */
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

        SmartDashboard.putBoolean("IMU_Connected",        ahrs.isConnected());
        SmartDashboard.putBoolean("IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber("IMU_Yaw",              ahrs.getYaw());
        SmartDashboard.putNumber("IMU_Pitch",            ahrs.getPitch());
        SmartDashboard.putNumber("IMU_Roll",             ahrs.getRoll());        
        SmartDashboard.putNumber("IMU_Angle",            ahrs.getAngle());

        SmartDashboard.putNumber("leftSpeed", tankSpeed.leftSpeed);
        SmartDashboard.putNumber("rightSpeed", tankSpeed.rightSpeed);
        SmartDashboard.putBoolean("driverAssistMode", driverAssistMode);
        SmartDashboard.putNumber("intakeOnCount", intakeOnCount);


        /*
         * don't delete this commented-out code yet.
         * we may use this once the other sw/hw is verified.
         */

        // !!!SID!!! - Should we change the turret to use the joystick y axis?
        // if (altJoystick.getRawButton(Constants.TURN_TURRET_RIGHT_BUTTON))
        // {
        //     turret.rotate(-Constants.TURRET_ON);
        // }
        // else if (altJoystick.getRawButtonReleased(Constants.TURN_TURRET_RIGHT_BUTTON))
        // {
        //     turret.rotate(Constants.TURRET_OFF);
        // }
        // if (altJoystick.getRawButton(Constants.TURN_TURRET_LEFT_BUTTON))
        // {
        //     turret.rotate(Constants.TURRET_ON);
        // }
        // else if (altJoystick.getRawButtonReleased(Constants.TURN_TURRET_LEFT_BUTTON))
        // {
        //     turret.rotate(Constants.TURRET_OFF);
        // }


        // returns true if the button is being held down
        // at the time that this method is being called
        // if(altJoystick.getRawButton(Constants.ACTIVATE_SHOOTER_TIMED_BUTTON))
        // {
        //     // activate the shooter for 1 second
        //     flyWheel.timed(1.0);
        // }

        // as long as the trigger is pushed keep firing
        // if(altJoystick.getRawButton(Constants.ACTIVATE_SHOOTER_BUTTON))
        // {
        //     ballShooter.manualShoot();
        // }
        // else
        // {
        //     ballShooter.manualStop();
        // }

        // !!!SID!!! - this will probably need more than this
        //             for the climber
        // if (altJoystick.getRawButtonPressed(Constants.TOGGLE_CLIMBER_BUTTON))
        // {
        //     climber.toggle(true);
        // }
        // else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_CLIMBER_BUTTON))
        // {
        //     climber.toggle(false);
        // }

        // move the robot -- our driveTrain object
        driveTrain.tankDrive(tankSpeed, driverAssistMode);
    }
}
