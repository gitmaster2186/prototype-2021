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
   } 
   
   public void manualStop()
   {
//       loaderRollers.toggle(false);

       // !!!SID!!! XXX - do we have to stop the flywheel also?
   }

   public void manualShoot()
   {
    //    // is there a ball is loadable position?
    //     if (loaderRollers.ballLoaded())
    //     {
    //        // yes. are the flywheels up to speed?
    //         if (flyWheel.upToSpeed())
    //         {
    //            // yes. send the ball up to the flywheels
    //            loaderRollers.toggle(true);
    //         }
    //     }

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
