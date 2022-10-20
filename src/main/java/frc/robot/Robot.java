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

//import javax.print.attribute.standard.Compression;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.BallShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LoaderRollers;
import frc.robot.utils.LimeLightSupport;
import frc.robot.utils.TankSpeeds;

//import frc.robot.subsystems.Climber;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.SPI;
//import frc.robot.subsystems.Turret;



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

    // AHRS ahrs; // navx 
      
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

    //private CANSparkMax neo550ShooterFrontIntake  = new CANSparkMax(Constants.SPARK_NEO550_FRONT_INTAKE_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterRearIntake   = new CANSparkMax(Constants.SPARK_NEO550_BACK_INTAKE_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterLoaderRollerFront  = new CANSparkMax(Constants.SPARK_NEO550_FRONT_LOADER_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterLoaderRollerBack  = new CANSparkMax(Constants.SPARK_NEO550_BACK_LOADER_CAN_ID, MotorType.kBrushless);

    private Intake intake = new Intake(neo550ShooterRearIntake); 

    // private CANSparkMax neo550ShooterTurret = new CANSparkMax(Constants.SPARK_NEO550_TURRET_CAN_ID, MotorType.kBrushless);
    // private Turret turret = new Turret(neo550ShooterTurret);

    /*
    !!!old way!!!
    private WPI_TalonFX shooterFlyWheelFront = new WPI_TalonFX(Constants.FALCON500_FRONT_FLYWHEEL_CAN_ID);
    private WPI_TalonFX shooterFlyWheelBack = new WPI_TalonFX(Constants.FALCON500_BACK_FLYWHEEL_CAN_ID);
  
    private FlyWheel flyWheel = new FlyWheel(shooterFlyWheelFront, 
                                             shooterFlyWheelBack);
*/
     // !!!NEW WAY!!!
     private FlyWheel flyWheel = new FlyWheel();

     private LoaderRollers loaderRollers = new LoaderRollers (neo550ShooterLoaderRollerFront,
                                                              neo550ShooterLoaderRollerBack,
                                                              flyWheel);

    private DigitalInput loaderLimitSwitch = new DigitalInput(Constants.LOADER_LIMIT_SWITCH);
    private DigitalInput leftPistonLimitSwitch = new DigitalInput(Constants.LEFT_PISTON_LIMIT_SWITCH);
    private DigitalInput rightPistonLimitSwitch = new DigitalInput(Constants.RIGHT_PISTON_LIMIT_SWITCH);
    private BallShooter ballShooter = new BallShooter(intake, loaderRollers, flyWheel, driveTrain, loaderLimitSwitch);

    private AutonomousMode autoMode;
    private Climber climber;
    private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    //private PowerDistribution examplePdp = new PowerDistribution(1, ModuleType.kRev);
    LimeLightSupport limeLeds;

    int intakeOnCount = 0;
    int intakeOffCount = 0;

    double vx = 0.0;

    //boolean ledsOn = true; // the limelight lights defaults to on?

    // this forces the limelight LEDs on or off
    // private void limeLeds.ledsSet(boolean inLedsOn)
    // {
    //     Number dir;
    //     if (inLedsOn == true)
    //     {
    //         dir = Constants.LIMELIGHT_LEDS_ON;
    //         ledsOn = true;
    //     }
    //     else
    //     {
    //         ledsOn = false;
    //         dir = Constants.LIMELIGHT_LEDS_OFF;
    //     }
    //     limeLightTable.getEntry(Constants.LIMELIGHT_LEDMODE).setNumber(dir);
    //     SmartDashboard.putBoolean("ledsOnxx", ledsOn);
    // }

    // // this toggles the limelight LEDs on or off
    // private void limeLightLEDsToggle()
    // {
    //     if (ledsOn == false)
    //     {
    //         limeLeds.ledsSet(true);
    //     }
    //     else
    //     {
    //         limeLeds.ledsSet(false);
    //     }
    // }

    @Override
    public void robotInit()
    {
        climber = new Climber(m_doubleSolenoid1, m_doubleSolenoid2);
        limeLeds = new LimeLightSupport();

        // make sure the arms are in (lowest position)
        climber.climb(Constants.CLIMBER_DOWN_DIRECTION);

        // try {
        //     /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
        //     /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
        //     /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
        //     // RoboRIO MXP I2C Interface
        //     ahrs  = new AHRS(SPI.Port.kMXP); 
        // } catch (RuntimeException ex ) {
        //     DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        // }
        // ahrs.reset();
        String wpiVerStr = WPILibVersion.Version;
        System.out.println("WPI version " + wpiVerStr);

        // limelight leds off
        limeLeds.ledsSet(false);

        // turn the driver camera on
        CameraServer.startAutomaticCapture();

        double comprAir = compressor.getPressure();
        SmartDashboard.putNumber("comprAir", comprAir);

        //double volts = examplePdp.getVoltage();
        //SmartDashboard.putNumber("pdpVolts", volts);

    }


    @Override
  /**
   * This function is called once at the start of autonomous.
   */
  public void autonomousInit() {
        // limelight leds on
        limeLeds.ledsSet(true);

        autoMode = new AutonomousMode(driveTrain, ballShooter);

        // !!!SID!!! - XXX - 3/17/22 does this make us stop quicker?
        //             This is important in autonomous mode
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
        // limelight leds off
        limeLeds.ledsSet(false);

        // put drivetrain back in coast mode
        neoDriveTrainFrontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        neoDriveTrainFrontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        SmartDashboard.putNumber("vxIn", 0.0);

    }

    @Override
    // gets called 50 times a second
    public void teleopPeriodic() {
        boolean driverAssistMode;
        
        // !!!SID!!! XXX - debug - look at the air pressure and battery voltage
        double comprAir = compressor.getPressure();
        SmartDashboard.putNumber("comprAir", comprAir);
        //double volts = examplePdp.getVoltage();
        //SmartDashboard.putNumber("pdpVolts", volts);

        /*
         * get speed values from joysticks
         * get them now because some cmds/buttons will modify the
         * speed input from the joysticks
         */
        TankSpeeds tankSpeed = getManualTankSpeed(); 

        /*
         * test the limit switches on the pistons.
         * if hit and moving forward then stop forward
         * forward motion of the motor on that side
         * This might allow us to align the robot to the 
         * obstacle we're hitting.
         */ 
        SmartDashboard.putBoolean("leftLimit", leftPistonLimitSwitch.get());
        if ((leftPistonLimitSwitch.get() == true) &&
            (tankSpeed.leftSpeed > 0))
        {
            tankSpeed.leftSpeed = 0;
        }

        SmartDashboard.putBoolean("rightLimit", rightPistonLimitSwitch.get());
        if ((rightPistonLimitSwitch.get() == true) && 
            (tankSpeed.rightSpeed > 0))
        {
            tankSpeed.rightSpeed = 0;
        }
    
        // toggle intake motors on/off
        if (altJoystick.getRawButton(Constants.TOGGLE_INTAKE_BUTTON))
        {
            intakeOnCount += 1;
            intake.toggle(true); 
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_INTAKE_BUTTON))
        {
            intakeOffCount += 1;
            intake.toggle(false);
        }

        if (altJoystick.getRawButtonPressed(Constants.CLIMBER_UP_BUTTON))
        {
            // climb -- extend piston
            climber.climb(Constants.CLIMBER_UP_DIRECTION);
        } 
        else if (altJoystick.getRawButtonPressed(Constants.CLIMBER_DOWN_BUTTON)) 
        {
            // climb -- retract piston
            climber.climb(Constants.CLIMBER_DOWN_DIRECTION);
        }

        /* 
         * set aim assist mode on/off
         * This will be input to our tankDrive method
         * to determine if we're using vision to position
         * the robot.
         */
        if ((altJoystick.getRawButton(Constants.AIM_ASSIST_BUTTON)) || 
            (rightJoystick.getRawButton(Constants.AIM_ASSIST_BUTTON)))
        {
            driverAssistMode = true;
        } 
        else
        {
            driverAssistMode = false;
        }

        // toggle limelight leds on/off
        if (altJoystick.getRawButtonPressed(Constants.LIMELIGHT_LEDS_BUTTON))
        {
            //System.out.println("ledOn: " + ledsOn);
            limeLeds.ledsToggle();
        }

        // check the driver right joystick button is pressed now
        if (rightJoystick.getRawButton(Constants.DRIVER_DEBUG_BUTTON)) 
        {
            // pressed since last time checked
            if (rightJoystick.getRawButtonPressed(Constants.DRIVER_FLIP_DRIVETRAIN_BUTTON))
            {
                // flip the drivetrain variable
                driveTrain.flip();
            }
        }
        
        // hold the debug button and shooter button to shoot 
        if (altJoystick.getRawButton(Constants.ALT_DEBUG_BUTTON)) 
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
                                    Constants.FLYWHEEL_ON,
                                    true, false); // use the limit switch
            }
            else if (altJoystick.getRawButtonReleased(Constants.ACTIVATE_SHOOTER_BUTTON))
            {
                ballShooter.manualStop();
                SmartDashboard.putNumber("flyWFrVel", 0);
                SmartDashboard.putNumber("flyWBaVel", 0);
        
            }

            // if we have the wrong ball in the intake reject it
            if (altJoystick.getRawButton(Constants.REJECT_BALL_BUTTON))
            {
                ballShooter.manualReject();
            }
            else if (altJoystick.getRawButtonReleased(Constants.REJECT_BALL_BUTTON))
            {
                ballShooter.manualStop();
                SmartDashboard.putNumber("flyWFrVel", 0);
                SmartDashboard.putNumber("flyWBaVel", 0);
            }
        }
        else
        {
            // debug button on alt stick is off to allow manual shooting

            /*
            * to manually shoot:
            * 1. intake ball
            * 2. hit the trigger
            * as long as the trigger is pushed keep firing
            */
                                
            //roller and flywheel individual cmds
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
            else if (altJoystick.getRawButton(Constants.FLYWHEEL_ONLY_BUTTON) == false)
            {
                // !!!SID!!! XXX - debug - 3/26/22 - test only remove after debug
                // double velFr = shooterFlyWheelFront.getSelectedSensorVelocity();
                // double velBa = shooterFlyWheelBack.getSelectedSensorVelocity();
                // SmartDashboard.putNumber("flyWFrVel", velFr);
                // SmartDashboard.putNumber("flyWBaVel", velBa);  

                SmartDashboard.getNumber("SmartDashboard/vxIn", vx);
                SmartDashboard.putNumber("vxOut", vx);

            }
        }


        SmartDashboard.putBoolean("driverAssistMode", driverAssistMode);
        // move the robot using our driveTrain object
        driveTrain.tankDrive(tankSpeed, driverAssistMode, false);

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


