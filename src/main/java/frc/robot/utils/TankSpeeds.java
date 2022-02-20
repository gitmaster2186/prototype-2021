package frc.robot.utils;

public class TankSpeeds
{
    public double leftSpeed;
    public double rightSpeed;
    public double targets;

    // calculate tankspeed based on input
    public TankSpeeds(double x, double y)
    {
            leftSpeed = x;
            rightSpeed = y;
            targets = 0;
    }
}
