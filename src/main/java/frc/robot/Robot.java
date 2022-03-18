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

/* USB order for the drivers
 *  Left driver joystick - 0-
 *  Right driver joystick - 1
 *  Shooter joystick - 2
 *  Xbox controler - 4
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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.Climber;
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

private final DoubleSolenoid m_doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                            Constants.LEFT_FIRST_PCM_PORT,  // PCM port 
                                            Constants.LEFT_SECOND_PCM_PORT); // PCM port 
private final DoubleSolenoid m_doubleSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                                            Constants.RIGHT_FIRST_PCM_PORT,  // PCM port 
                                            Constants.RIGHT_SECOND_PCM_PORT); // PCM port 

    private CANSparkMax neo550ShooterFrontIntake  = new CANSparkMax(Constants.SPARK_NEO550_FRONT_INTAKE_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterRearIntake   = new CANSparkMax(Constants.SPARK_NEO550_BACK_INTAKE_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterLoaderRollerFront  = new CANSparkMax(Constants.SPARK_NEO550_FRONT_LOADER_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterLoaderRollerBack  = new CANSparkMax(Constants.SPARK_NEO550_BACK_LOADER_CAN_ID, MotorType.kBrushless);
    // private CANSparkMax neo550ShooterTurret       = new CANSparkMax(Constants.SPARK_NEO550_TURRET_CAN_ID, MotorType.kBrushless);

    private WPI_TalonFX falcon500ShooterFlyWheel1 = new WPI_TalonFX(Constants.FALCON500_FRONT_FLYWHEEL_CAN_ID);
    private WPI_TalonFX falcon500ShooterFlyWheel2 = new WPI_TalonFX(Constants.FALCON500_BACK_FLYWHEEL_CAN_ID);
  
    private Intake intake = new Intake(neo550ShooterFrontIntake, 
                                       neo550ShooterRearIntake); 
    // private Turret turret = new Turret(neo550ShooterTurret);


    private FlyWheel flyWheel = new FlyWheel(falcon500ShooterFlyWheel1, 
                                             falcon500ShooterFlyWheel2);

     private LoaderRollers loaderRollers = new LoaderRollers (neo550ShooterLoaderRollerFront,
                                                              neo550ShooterLoaderRollerBack,
                                                              flyWheel);
    // !!!SID!!! - XXX - TBD
    private BallShooter ballShooter = new BallShooter(intake, loaderRollers, flyWheel, driveTrain);

    private AutonomousMode autoMode;
    private Climber climber;

    int intakeOnCount = 0;
    boolean ledsOn = false;

    @Override
    public void robotInit()
    {
        climber = new Climber(m_doubleSolenoid1, m_doubleSolenoid2);
        climber.climb(Constants.CLIMBER_DOWN_DIRECTION);

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

        // !!!SID!!! - XXX - 3/17/22 does this make us stop quicker?
        neoDriveTrainFrontLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        neoDriveTrainFrontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
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
        //limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF); // leds off
        //limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_ON); // leds on

        // !!!SID!!! - XXX - 3/17/22 does this allow our drive train to coast?
        neoDriveTrainFrontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        neoDriveTrainFrontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);

    }

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

        /*
         * to manually shoot:
         * 1. intake ball
         * 2. hit the trigger
         */
        // as long as the trigger is pushed keep firing
               
               

        //roller and flywheel individual cmds are only for debug
        if (altJoystick.getRawButton(Constants.LOADER_ONLY_BUTTON))
        {
            loaderRollers.setRollerSpeed(Constants.LOADER_SPEED_ON);
        }
        else if (altJoystick.getRawButtonReleased(Constants.LOADER_ONLY_BUTTON))
        {
            loaderRollers.setRollerSpeed(Constants.LOADER_SPEED_OFF);
        }
        
        if (altJoystick.getRawButton(Constants.FLYWHEEL_ONLY_BUTTON))
        {
            flyWheel.setFlyWheelSpeed(Constants.FLYWHEEL_ON);
        }
        else if (altJoystick.getRawButtonReleased(Constants.FLYWHEEL_ONLY_BUTTON))
        {
            flyWheel.setFlyWheelSpeed(Constants.FLYWHEEL_OFF);
        }

        // climb stage 1
        if (altJoystick.getRawButtonPressed(Constants.CLIMBER_UP_BUTTON))
        {
          System.out.println("up");
          climber.climb(Constants.CLIMBER_UP_DIRECTION);
        } 
        else if (altJoystick.getRawButtonPressed(Constants.CLIMBER_DOWN_BUTTON)) 
        {
          System.out.println("down");
          climber.climb(Constants.CLIMBER_DOWN_DIRECTION);
        }

        /* 
         * set aim assist mode on/off
         * This will be input to our tankDrive method
         * to determine if we're using vision to position
         * the robot.
         */
        if (altJoystick.getRawButton(Constants.AIM_ASSIST_BUTTON)) 
        {
            driverAssistMode = true;
        } 
        else
        {
            // limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF); // leds on
            // ledsOn = false;
            driverAssistMode = false;
        }

        //System.out.println("ledOn: " + ledsOn);

        //limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_ON); // leds on
        // toggle limelight leds on/off
        if (altJoystick.getRawButtonPressed(Constants.LIMELIGHT_LEDS_BUTTON))
        {
            System.out.println("pressed: " + ledsOn);
            if (ledsOn == false)
            {
                limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_ON); // leds on
                ledsOn = true;
                System.out.println("on?: " + ledsOn);
            }
            else
            {
                limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(Constants.LIMELIGHT_LEDS_OFF); // leds on
                ledsOn = false;
                System.out.println("off?: " + ledsOn);
            }
            System.out.println("test?: " + ledsOn);
            SmartDashboard.putBoolean("ledsOnxx", ledsOn);
        }

        // check the driver right joystick
        if (rightJoystick.getRawButton(Constants.DRIVER_DEBUG_BUTTON)) 
        {
            // 
            if (rightJoystick.getRawButtonPressed(Constants.DRIVER_FLIP_DRIVETRAIN_BUTTON))
            {
                driveTrain.flip();
            }
        }

        // to use shooter button also hold the debug button
        if (altJoystick.getRawButton(Constants.DEBUG_BUTTON)) 
        {
            //System.out.println("leftSpeed: " + 
            //                   tankSpeed.leftSpeed + 
            //                   " rightSpeed: " + 
            //                   tankSpeed.rightSpeed +
            //                   " driverAssist: " +
            //                   driverAssistMode);

            if (altJoystick.getRawButton(Constants.ACTIVATE_SHOOTER_BUTTON))
            {
                ballShooter.activate(Constants.LOADER_SPEED_ON, 
                                    Constants.FLYWHEEL_ON);
            }
            else if (altJoystick.getRawButtonReleased(Constants.ACTIVATE_SHOOTER_BUTTON))
            {
                ballShooter.manualStop();
            }

            // if have the wrong ball in the intake reject it
            if (altJoystick.getRawButton(Constants.REJECT_BALL_BUTTON))
            {
                ballShooter.manualReject();
            }
            else if (altJoystick.getRawButtonReleased(Constants.REJECT_BALL_BUTTON))
            {
                ballShooter.manualStop();
            }
        }

        /*
         * disabled: turret, flywheel.timed, 
         *           
         * 
         * The commented-out code was moved to the bottom of the file.
         * don't delete the commented-out code yet.
         * we may use this once the other sw/hw is verified.
         */

         // Testing to see if Smartdashboard is lagging the system
        // SmartDashboard.putBoolean("IMU_Connected",        ahrs.isConnected());
        // SmartDashboard.putBoolean("IMU_IsCalibrating",    ahrs.isCalibrating());
        // SmartDashboard.putNumber("IMU_Yaw",              ahrs.getYaw());
        // SmartDashboard.putNumber("IMU_Pitch",            ahrs.getPitch());
        // SmartDashboard.putNumber("IMU_Roll",             ahrs.getRoll());        
        // SmartDashboard.putNumber("IMU_Angle",            ahrs.getAngle());

        // SmartDashboard.putNumber("leftSpeed", tankSpeed.leftSpeed);
        // SmartDashboard.putNumber("rightSpeed", tankSpeed.rightSpeed);
        // SmartDashboard.putBoolean("driverAssistMode", driverAssistMode);
        // SmartDashboard.putNumber("intakeOnCount", intakeOnCount);

        // move the robot -- our driveTrain object
        driveTrain.tankDrive(tankSpeed, driverAssistMode);
    }
}


    // old code -- don't delete this. We will reenable this later.
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



    // remap inVal from in min and max to out in min max ranges
    // double map(double inVal, double in_min, double in_max, double out_min, double out_max)
    // {
    //   return (inVal - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // }


