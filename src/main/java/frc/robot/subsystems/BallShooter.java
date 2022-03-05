package frc.robot.subsystems;

import frc.robot.utils.TankSpeeds;

public class BallShooter {
    Intake intake;
    LoaderRollers loaderRollers;
    FlyWheel flyWheel;
    DriveTrain driveTrain;

   public BallShooter(Intake inIntake, LoaderRollers inLoaderRollers, FlyWheel inFlyWheel, DriveTrain inDriveTrain)
   {
    intake = inIntake;
    loaderRollers = inLoaderRollers;
    flyWheel = inFlyWheel;
    driveTrain = inDriveTrain;
    // setting this to 1 because we start with 1 loaded for autonomous mode
    loaderRollers.setBallCount(1);
   } 
   
   public void manualStop()
   {
//       loaderRollers.toggle(false);

       // !!!SID!!! XXX - do we have to stop the flywheel also?
   }

   public boolean manualShoot()
   {
       boolean ballShot = false;
       // is there a ball is loadable position?
        if (loaderRollers.ballLoaded())
        {
            // set flywheel speed
            flyWheel.toggle(true);

            // yes. are the flywheels up to speed?
            if (flyWheel.upToSpeed())
            {
               // yes. send the ball up to the flywheels
               loaderRollers.toggle(true);
               ballShot = true;
            }
        }
        return ballShot;
   }

   public void autoShoot(TankSpeeds tankSpeed)
   {
        driveTrain.tankDrive(tankSpeed, true);

       // do we have the target?
       if (tankSpeed.targets > 0)
        {
            // yes. shoot the ball
            manualShoot();
        }
   }

}
