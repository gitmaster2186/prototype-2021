package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.TankSpeeds;

public class BallShooter {
    Intake intake;
    LoaderRollers loaderRollers;
    FlyWheel flyWheel;
    DriveTrain driveTrain;
    int ballCount = 0;

   public BallShooter(Intake inIntake, LoaderRollers inLoaderRollers, FlyWheel inFlyWheel, DriveTrain inDriveTrain)
   {
    intake = inIntake;
    loaderRollers = inLoaderRollers;
    flyWheel = inFlyWheel;
    driveTrain = inDriveTrain;
    // setting this to 1 because we start with 1 loaded for autonomous mode
    setBallCount(1);
   } 
   
   public boolean manualReject()
   {
        return activate(Constants.LOADER_SPEED_ON,
                        Constants.FLYWHEEL_REJECT_SPEED);
   }

   public boolean  manualStop()
   {
        return activate(Constants.LOADER_SPEED_OFF,
                        Constants.FLYWHEEL_OFF);
   }

    /*
     * activate/deactivate the loader rollers and the flywheels.
     * run while button is pushed to load ball into firing position
     * do not load ball if flywheels are not up to speed 
     * update but ignore the ball count.
     * 
     * Return true of the loaders where turned on (ball shot)
     * 
     */
   public boolean activate(double inLoaderSpeed, double inFlySpeed)
   {
       boolean ret = false; 
       // System.out.println("activate " + dbgCount);

       if (inLoaderSpeed > Constants.LOADER_SPEED_OFF)
       {
           // make sure the flywheel is on
           flyWheel.setFlyWheelSpeed(inFlySpeed);

           // if the flywheels are already up to speed then turn on the loaders
           if (flyWheel.upToSpeed() == true)
           {

               // turn the loader motors on
               loaderRollers.setRollerSpeed(inLoaderSpeed);

               // the ball has been shot
               ret = true;
           }
       }
       else
       {
           // done shooting. turn the loader and flywheel motors off
           loaderRollers.setRollerSpeed(Constants.LOADER_SPEED_OFF);
           flyWheel.setFlyWheelSpeed(Constants.FLYWHEEL_OFF);
       }

       // add -1 to ballCount
       // incBallCount(-1);

       return ret;
   }   


   public void autoShoot(TankSpeeds tankSpeed)
   {
        driveTrain.tankDrive(tankSpeed, true);

       // do we have the target?
       if (tankSpeed.targets > 0)
        {
            // yes. shoot the ball
            activate(Constants.LOADER_SPEED_ON, Constants.FLYWHEEL_ON);
        }
   }

   public void setBallCount(int count)
   {
       ballCount = count;
   }

   public void incBallCount(int count)
   {
       ballCount += count;
   }

   public boolean ballLoaded() 
   {
       boolean ret = false;
       if (ballCount > 0)
       {
           ret = true;
       }
       return ret;
   }
}
