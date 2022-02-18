package frc.robot;

public class DriverUI {

    public enum ControllerType {
        XBOX_CONTROLLER, 
        JOYSTICK_CONTROLLER
    }

    ControllerType driverControllers = ControllerType.JOYSTICK_CONTROLLER;
    ControllerType shooterControllers = ControllerType.XBOX_CONTROLLER;

    public DriverUI(ControllerType drvCtrls, int drvUsbPort,
                    ControllerType shootCtrl, int shootUsbPort)
    {
        System.out.println("init driver UI");
        driverControllers = drvCtrls;
        shooterControllers = shootCtrl;
    }
}
