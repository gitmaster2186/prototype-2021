package frc.robot;

public class Constants {
    public static final int LEFT_JOYSTICK_USB_PORT = 0;
    public static final int RIGHT_JOYSTICK_USB_PORT = 1;
    public static final int ALT_JOYSTICK_USB_PORT = 2;

    public static final double VISION_MIN_AREA = 0.5;
    public static final double VISION_MIN_ALIGN_SPEED = 0.1;

    public static final Number LIMELIGHT_LEDS_OFF = 1;
    public static final Number LIMELIGHT_LEDS_ON = 3;
    public static final String LIMELIGHT_VALID_TARGETS = "tv";
    public static final String LIMELIGHT_HORIZONTAL_OFFSET =  "tx";
    public static final String LIMELIGHT_VERTICAL_OFFSET =  "ty";
    public static final String LIMELIGHT_TARGET_AREA =  "ta";
    public static final String LIMELIGHT_LEDMODE = "ledMode";

    /*
     * commands:
     *   1.  fire turret
     *   2.  aim assist
     *   3.  turn turret left
     *   4.  turn turret right
     *   5.  toggle intake
     *   6.  toggle loader
     *   7.  deploy intake
     *   8.  fire turret timed
     *   9.  toggle climber
     *  11.  debug
     */

    // alt joystick -- extreme 3dpro joystick button mapping
    public static int FIRE_TURRET = 1;          // trigger
    public static int AIM_ASSIST_BUTTON = 2;    // thumb button

    // !!!SID!!! - Should we change the turret to use the joystick y axis?
    public static int TURN_TURRET_LEFT = 3;     // below and left of central switch
    public static int TURN_TURRET_RIGHT = 4;    // below and right of central switch

    public static int TOGGLE_INTAKE = 5;        // left of central switch
    public static int TOGGLE_LOADER_ROLLERS = 6; // right of central switch

    public static int DEPLOY_INTAKE = 7;        // top left outside on base
    public static int FIRE_TURRET_TIMED = 8;    // top left inside on base

    public static final int TOGGLE_CLIMBER = 9; // 

    public static int DEBUG_BUTTON = 11;        // bottom left outside on base


    public static final int SPARK_NEO550_INTAKE_1_CAN_ID = 6;
    public static final int SPARK_NEO550_INTAKE_2_CAN_ID = 1;
    public static final int SPARK_NEO550_TURRET_CAN_ID = 8;
    public static final int SPARK_NEO550_ROLLER_1_CAN_ID = 9;

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
    public static final double CLIMBER_SPEED_OFF = 0;
    public static final double MIN_FLYWHEEL_VEL = 0;
    

}
