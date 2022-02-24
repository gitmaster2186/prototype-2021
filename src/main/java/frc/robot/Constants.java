package frc.robot;

public class Constants {
    public static final int LEFT_JOYSTICK_USB_PORT = 0;
    public static final int RIGHT_JOYSTICK_USB_PORT = 1;
    public static final int ALT_JOYSTICK_USB_PORT = 2;


    /*
     * commands:
     *   1.  activate shooter
     *   2.  aim assist
     *   3.  toggle intake
     *   4.  toggle loader
     *   5.  toggle flywheels
     *   6.  unassigned
     *   7.  turn turret left
     *   8.  turn turret right
     *   9.  activate shooter timed
     *  10.  toggle climber
     *  11.  unassigned
     *  12.  debug
     */

    // alt joystick -- start -- extreme 3dpro joystick button mapping
    public static final int ACTIVATE_SHOOTER          = 1;       // stick, trigger                          -- vision assisted driving and shooting
    public static final int AIM_ASSIST_BUTTON         = 2;       // stick, thumb button                     -- vision assisted driving

    public static final int TOGGLE_INTAKE             = 3;       // stick, below, left of central switch    -- manual intake control
    public static final int TOGGLE_LOADER_ROLLERS     = 4;       // stick, below, right of central switch   -- manual loader control
    public static final int TOGGLE_FLYWHEELS          = 5;       // stick, even,  left of central switch    -- manual flywheels control

    public static final int AVAILABLE_1_BUTTON        = 6;       // stick, even,  right of central switch   -- available, no function

    // !!!SID!!! XXX - Should we change the turret to use the joystick y axis?
    public static final int TURN_TURRET_LEFT          = 7;       // base, left, top,    outside             -- manual turn turret left
    public static final int TURN_TURRET_RIGHT         = 8;       // base, left, top,    inside              -- manual turn turret right

    public static final int ACTIVATE_SHOOTER_TIMED    = 9;       // base, left, middle, outside
    public static final int TOGGLE_CLIMBER            = 10;      // base, left, middle, inside

    public static final int AVAILABLE_2_BUTTON        = 11;      // base, left, bottom, outside             -- available, no function

    public static final int DEBUG_BUTTON              = 12;      // base, left, bottom, inside              -- activate debug prints to console
    // alt joystick -- end   -- extreme 3dpro joystick button mapping

    public static final int SPARK_NEO550_INTAKE_1_CAN_ID = 6;
    public static final int SPARK_NEO550_INTAKE_2_CAN_ID = 1;
    public static final int SPARK_NEO550_TURRET_CAN_ID = 8;
    public static final int SPARK_NEO550_ROLLER_1_CAN_ID = 9;
    public static final int SPARK_NEO550_ROLLER_2_CAN_ID = 10;

    // climber
    public static final int FALCON500_CLIMBER_1_CAN_ID = 3;
    public static final int FALCON500_CLIMBER_2_CAN_ID = 6;

    // shooter
    public static final int FALCON500_SHOOTER_1_CAN_ID = 2;
    public static final int FALCON500_SHOOTER_2_CAN_ID = 7;

    // drivetrain
    public static final int SPARK_NEO_DRIVETRAIN_1_CAN_ID = 3;
    public static final int SPARK_NEO_DRIVETRAIN_2_CAN_ID = 5;
    public static final int SPARK_NEO_DRIVETRAIN_3_CAN_ID = 2;
    public static final int SPARK_NEO_DRIVETRAIN_4_CAN_ID = 7;

    public static final double INTAKE_ON  =  0.5;
    public static final double INTAKE_OFF =  0.0;
    public static final double FLYWHEEL_ON = 1.0;
    public static final double FLYWHEEL_OFF = 0.0;
    public static final double FLYWHEEL_ACTIVE_TIME = 1.0;
    public static final double LOADER_SPEED_ON = 0.5;
    public static final double LOADER_SPEED_OFF = 0;

    public static final double TURRET_ON = 0.1;
    public static final double TURRET_OFF = 0.0;
    public static final double CLIMBER_SPEED_ON = 0.25;
    public static final double CLIMBER_SPEED_OFF = 0.0;

    // for flywheel up to speed test
    // !!!SID!!! XXX - change this to a real value.
    public static final double MIN_FLYWHEEL_VEL = 0.0;

    // roborio DIO assignments
    public static final int DIOleftLimitSwitch = 0;
    public static final int DIOrightLimitSwitch = 1;

    // Slew Rate Limiter Constants    
    public static final double FLY_WHEEL_RAMP_UP_POWER = 0.75;
    public static final double DRIVE_TRAIN_RAMP_UP_POWER = 0.25;
    public static final double TURRET_RAMP_UP_POWER = 0.75;
    public static final double INTAKE_RAMP_UP_POWER = 0.75;
    public static final double LOADER_RAMP_UP_POWER = 0.75;
    
    // vision constants
    public static final double VISION_MIN_AREA = 0.5;
    public static final double VISION_MIN_ALIGN_SPEED = 0.1;

    public static final Number LIMELIGHT_LEDS_OFF = 1;
    public static final Number LIMELIGHT_LEDS_ON = 3;
    public static final String LIMELIGHT_VALID_TARGETS = "tv";
    public static final String LIMELIGHT_HORIZONTAL_OFFSET =  "tx";
    public static final String LIMELIGHT_VERTICAL_OFFSET =  "ty";
    public static final String LIMELIGHT_TARGET_AREA =  "ta";
    public static final String LIMELIGHT_LEDMODE = "ledMode";
}
