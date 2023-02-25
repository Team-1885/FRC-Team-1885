package us.ilite.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import us.ilite.common.Data;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.NeoDriveModule;

public class AutoBalance extends CommandBase {
    private Data db = Robot.DATA;
    private NetworkTable mTable;
    // set a range where the robot is considered balanced. You will probably never get it at exactly zero degrees.
    private double mErrorRange = 6; // total degrees in range
    private double mErrorCenterPoint = 0; // degree considered the center of the range
    private double mErrorLowBound;
    private double mErrorHighBound;
    // keep track of the current robot pitch
    private double mCurrentPitch;

    @Override
    public void initialize() {
        mTable = NetworkTableInstance.getDefault().getTable("AutoBalance");

        // calculate the low and high bound of the error range
        mErrorLowBound = mErrorCenterPoint - (mErrorRange / 2);
        mErrorHighBound = mErrorCenterPoint + (mErrorRange / 2);


        //mCommand = commandGenerator.generateCommand(mTrajectoryName);
//        mCommand.beforeStarting(() -> mNeoDrive.resetOdometry(commandGenerator.getTrajInitPose()));
        //mTable.getEntry("initial pose").setString(commandGenerator.getTrajInitPose().toString());
        //commandGenerator.generateCommand(mTrajectoryName).schedule(false);
    }
    public void autoBalance()
    {
        /* TODO If holding a button, move by the pitch scaled to a number
        If pitch is 30 move forward, if pitch is 15 move forawrd a little less. If pitch is -30 move backward,
        if pitch is -15 move backward a little less.
        do these while a button is pressed and while the pitch is not within a range near 0 degrees.
         */



        // get current left and right velocities
        //double currentLeftVelocity = db.drivetrain.get(EDriveData.L_DESIRED_VEL_FT_s);
        //double currentRightVelocity = db.drivetrain.get(EDriveData.R_DESIRED_VEL_FT_s);

        double currentPitch = db.imu.get(EGyro.ROOL_DEGREES); // db.imu.get(EGyro.PITCH_DEGREES); pitch and roll are switched
        if (currentPitch < errorLowBound)
        {
            // move robot backwards by a factor of angle between pitch and horizontal
            double factor = -1*Math.abs(currentPitch - errorLowBound); // corrects faster if there is a larger angle

            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, factor/180);
            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, factor/180);

        }
        else if (currentPitch > errorHighBound)
        {
            // move robot forwards by a factor of angle between pitch and horizontal
            double factor = Math.abs(currentPitch - errorHighBound); // corrects faster if there is a larger angle

            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, factor/180);
            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, factor/180);
        }
        else // when within error range
        {
            // stop robot
            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, 0);
            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, 0);
        }
    }

    @Override
    public void execute() { // continuously executes until the command is finished
        // update current pitch
        mCurrentPitch = db.imu.get(EGyro.PITCH_DEGREES);
    }

    @Override
    public void end(boolean interrupted) { // executes once after the command ends
//        double desiredHeading = mTrajectory.sample(mTrajectory.getTotalTimeSeconds()).poseMeters.getRotation().getDegrees();
//        if(Robot.DATA.imu.get(EGyro.HEADING_DEGREES) < desiredHeading)  {
//            Robot.DATA.drivetrain.set(EDriveData.DESIRED_TURN_PCT, (Robot.DATA.drivetrain.get(EDriveData.DESIRED_TURN_PCT) + 0.1));
//        } else {
//            Robot.DATA.drivetrain.set(EDriveData.DESIRED_TURN_PCT, (Robot.DATA.drivetrain.get(EDriveData.DESIRED_TURN_PCT) - 0.1));
//        }

        // if finished, stop robot

    }

    @Override
    public boolean isFinished() { // command scheduler continually checks if the command is finished
        mTable.getEntry("Current Pitch").setNumber(mCurrentPitch);


        if(mCurrentPitch > mErrorHighBound) { // pitch too positive - drive forward to balance
            mTable.getEntry("AutoBalance Status").setString("Greater than high bound");

            //

            // drive forward
            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, );
            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, );

            return false;

        }
        else if (mCurrentPitch < mErrorLowBound) { // pitch too negative - drive backward to balance
            mTable.getEntry("AutoBalance Status").setString("Lower than low bound");

            // drive backward
            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, );
            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, );

            return false;
        }
        else { // in between bounds
            return true;
        }


    }

}
