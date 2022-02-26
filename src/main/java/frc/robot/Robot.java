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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.FlyWheel;
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


      
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable limeLightTable = inst.getTable("limelight");
    

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
    private CANSparkMax neo550ShooterLoaderRollerFront  = new CANSparkMax(Constants.SPARK_NEO550_LOADER_1_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterLoaderRollerBack  = new CANSparkMax(Constants.SPARK_NEO550_LOADER_2_CAN_ID, MotorType.kBrushless);
    private CANSparkMax neo550ShooterTurret       = new CANSparkMax(Constants.SPARK_NEO550_TURRET_CAN_ID, MotorType.kBrushless);

    private WPI_TalonFX falcon500ShooterFlyWheel1 = new WPI_TalonFX(Constants.FALCON500_SHOOTER_1_CAN_ID);
    private WPI_TalonFX falcon500ShooterFlyWheel2 = new WPI_TalonFX(Constants.FALCON500_SHOOTER_2_CAN_ID);
  
    private WPI_TalonFX falcon500Climber1         = new WPI_TalonFX(Constants.FALCON500_CLIMBER_1_CAN_ID);
    private WPI_TalonFX falcon500Climber2         = new WPI_TalonFX(Constants.FALCON500_CLIMBER_2_CAN_ID);
  
    private Intake intake = new Intake(neo550ShooterFrontIntake, 
                                       neo550ShooterRearIntake); 
    private Turret turret = new Turret(neo550ShooterTurret);
    private Climber climber = new Climber(falcon500Climber1,
                                          falcon500Climber2);


    private FlyWheel flyWheel = new FlyWheel(falcon500ShooterFlyWheel1, 
                                             falcon500ShooterFlyWheel2);

     private LoaderRollers loaderRollers = new LoaderRollers (neo550ShooterLoaderRollerFront,
                                                              neo550ShooterLoaderRollerBack);
    // !!!SID!!! - XXX - TBD
    private BallShooter ballShooter = new BallShooter(intake, loaderRollers, flyWheel, driveTrain);
    int intakeCount = 0;
    @Override
    public void robotInit()
    {}


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
        TankSpeeds tankSpeed = getManualTankSpeed(); // get speed values from joysticks

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
        if (altJoystick.getRawButton(Constants.TOGGLE_INTAKE))
        {
            intakeCount += 1;
            intake.toggle(true); 
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_INTAKE))
        {
            intake.toggle(false);
        }
        SmartDashboard.putNumber("intakeCount", intakeCount);

        if (altJoystick.getRawButton(Constants.TOGGLE_LOADER_ROLLERS))
        {
            System.out.println("LON");
            loaderRollers.toggle(true);
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_LOADER_ROLLERS))
        {
            System.out.println("LOFF");
            loaderRollers.toggle(false);
        }

        if (altJoystick.getRawButton(Constants.TOGGLE_FLYWHEELS))
        {
            flyWheel.toggle(true);
        }
        else if (altJoystick.getRawButtonReleased(Constants.TOGGLE_FLYWHEELS))
        {
            flyWheel.toggle(false);
        }

        // returns true if the button is being held down
        // at the time that this method is being called
        if(altJoystick.getRawButton(Constants.ACTIVATE_SHOOTER_TIMED))
        {
            // activate the shooter for 1 second
            flyWheel.timed(1.0);
        }

        // as long as the trigger is pushed keep firing
        if(altJoystick.getRawButton(Constants.ACTIVATE_SHOOTER))
        {
            ballShooter.manualShoot();
        }
        else
        {
            ballShooter.manualStop();
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
