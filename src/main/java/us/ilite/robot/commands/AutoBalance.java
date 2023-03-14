package us.ilite.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import us.ilite.common.Data;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.NeoDriveModule;

import static us.ilite.common.types.drive.EDriveData.*;
// this command will need to start before the robot drives up onto the charge station
// trajectory should end at the foot of the charge station
// gyro should zero
// get on top of the charge station and start the pid loop
/** Balance the robot on top of the charge station game element:
 *
 * The AutoBalance Command is run once the robot is on top of the charge station.
 * It will continuously check the robots pitch and determine if it considers a pitch balanced.
 *      If the robot is not balanced, it will drive forward or backward to correct the balance.
 *      If the robot is balanced, the robot will stop its motion.
 */

// what if this utility function belonged to neodrive, and neodrive set the database
public class AutoBalance extends CommandBase {
    private Data db = Robot.DATA;
    private NetworkTable mTable;
    // set a range of roll values where the robot is considered balanced. A range is needed because the robot will never be exactly horizontal
    private double mBalancedRange = 6; // total degrees in range
    private double mCenterDegreeOfRange = 0; // degree considered the center of the range
    private double mBalancedLowBound; // lowest degree considered balanced
    private double mBalancedHighBound; // highest degree considered balanced
    // keep track of the current robot roll (the gyro is oriented so roll and pitch are swapped)
    private double mCurrentRoll;
    private NeoDriveModule mDrive;

    private double newVelocity;

    @Override
    public void initialize() {
        mTable = NetworkTableInstance.getDefault().getTable("AutoBalance");

        // calculate the low and high bound of the range of degrees where the robot is considered balanced
        mBalancedLowBound = mCenterDegreeOfRange - (mBalancedRange / 2);
        mBalancedHighBound = mCenterDegreeOfRange + (mBalancedRange / 2);

        mDrive = NeoDriveModule.getInstance();
    }

    @Override
    public void execute() { // continuously executes until the command is finished
        // update current roll
        mCurrentRoll = db.imu.get(EGyro.ROLL_DEGREES);

        //mDrive.setValueInDrivetrainDatabase(Enums.EDriveState.AUTOBALANCE);
        mDrive.setValueInDrivetrainDatabase(DESIRED_THROTTLE_PCT, newVelocity);

        mTable.getEntry("Robotmode").setString("" + Enums.EDriveState.AUTOBALANCE);
        mTable.getEntry("newVelocity").setDouble(newVelocity);
    }

    @Override
    public void end(boolean interrupted) { // executes once after the command ends
        // robot is balanced, stop robot
        //mDrive.setValueInDrivetrainDatabase(Enums.EDriveState.AUTOBALANCE);
        mDrive.setValueInDrivetrainDatabase(DESIRED_THROTTLE_PCT, 0);
    }

    @Override
    public boolean isFinished() { // command scheduler continually checks if the command is finished
        double errorAngle; // angle away from balanced
        // the greater the errorAngle, the faster the robot should correct its balance

        if(mCurrentRoll > mBalancedHighBound) { // Roll too positive - drive forward to balance
            mTable.getEntry("AutoBalance Status").setString("Greater than high bound");

            // identify the angle between the current roll and the high bound
            errorAngle = mCurrentRoll - mBalancedHighBound; // always positive
            // the greater the errorAngle, the faster the robot should correct its balance
            newVelocity = errorAngle/20; // make angle magnitude between 0 and 1

            mTable.getEntry("finshed").setString("not finished");

            return false; // it is running a single command. its only this. just a command no group
        }
        else if (mCurrentRoll < mBalancedLowBound) { // roll too negative - drive backward to balance
            mTable.getEntry("AutoBalance Status").setString("Lower than low bound");

            // identify the angle between the current roll and the low bound
            errorAngle = mCurrentRoll - mBalancedLowBound; // always negative
            // the greater the errorAngle, the faster the robot should correct its balance
            newVelocity = errorAngle/20; // make angle magnitude between 0 and 1

            mTable.getEntry("finshed").setString("not finished");
            return false;
        }
        else { // in between bounds
            mTable.getEntry("finshed").setString("finished");
            return true;
        }
    }
}




//public class AutoBalance extends CommandBase {
//    private Data db = Robot.DATA;
//    private NetworkTable mTable;
//    // set a range of roll values where the robot is considered balanced. A range is needed because the robot will never be exactly horizontal
//    private double mBalancedRange = 6; // total degrees in range
//    private double mCenterDegreeOfRange = 0; // degree considered the center of the range
//    private double mBalancedLowBound; // lowest degree considered balanced
//    private double mBalancedHighBound; // highest degree considered balanced
//    // keep track of the current robot roll (the gyro is oriented so roll and pitch are swapped)
//    private double mCurrentRoll;
//    private NeoDriveModule mDrive;
//
//    private double newVelocity;
//
//    @Override
//    public void initialize() {
//        mTable = NetworkTableInstance.getDefault().getTable("AutoBalance");
//
//        // calculate the low and high bound of the range of degrees where the robot is considered balanced
//        mBalancedLowBound = mCenterDegreeOfRange - (mBalancedRange / 2);
//        mBalancedHighBound = mCenterDegreeOfRange + (mBalancedRange / 2);
//
//        mDrive = NeoDriveModule.getInstance();
//    }
//
//    @Override
//    public void execute() { // continuously executes until the command is finished
//        // update current roll
//        mCurrentRoll = db.imu.get(EGyro.ROLL_DEGREES);
////        Robot.DATA.drivetrain.set(EDriveData.STATE, Enums.EDriveState.AUTOBALANCE);
////        Robot.DATA.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, newVelocity);
//
//        //mDrive.setThrottlePct(newVelocity);
//
//        mTable.getEntry("Robotmode").setString("" + Enums.EDriveState.AUTOBALANCE);
//        mTable.getEntry("newVelocity").setDouble(newVelocity);
//
////        mTable.getEntry("Execute: database drivetrain instance").setString("" + db.drivetrain.hashCode());
////        mTable.getEntry("Exevute: database instance").setString("" + db.hashCode());
//
//
//
//        // i was originally using db
//        // maybe db is a local reference?
//    }
//
//    @Override
//    public void end(boolean interrupted) { // executes once after the command ends
//        // robot is balanced, stop robot
////        db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.AUTOBALANCE);
////        db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0);
//        //mDrive.setThrottlePct(0);
//        // what do i revert the drive state to when im done with it? it gets called at the very end but it never finishes, so it is never being called
//        // the end is called when it is balanced.
//    }
//
//    @Override
//    public boolean isFinished() { // command scheduler continually checks if the command is finished
//        double errorAngle;
//        // the greater the errorAngle, the faster the robot should correct its balance
//
//
////        mTable.getEntry("Current Roll").setNumber(mCurrentRoll);
////        mTable.getEntry("throttle").setString("" + db.drivetrain.get(EDriveData.DESIRED_THROTTLE_PCT) + " at " + System.currentTimeMillis()%10000);
////
////        mTable.getEntry("IsFinished: database drivetrain instance").setString("" + db.drivetrain.hashCode());
////        mTable.getEntry("IsFinished: database instance").setString("" + db.hashCode());
//
//
//        if(mCurrentRoll > mBalancedHighBound) { // Roll too positive - drive forward to balance
//            mTable.getEntry("AutoBalance Status").setString("Greater than high bound");
//
//            // identify the angle between the current roll and the high bound
////            double errorAngle = mCurrentRoll - mBalancedHighBound; // always positive
////            // the greater the errorAngle, the faster the robot should correct its balance
////            double newVelocity = errorAngle/20; // make angle magnitude between 0 and 1
//            errorAngle = mCurrentRoll - mBalancedHighBound; // always positive
//            // the greater the errorAngle, the faster the robot should correct its balance
//            newVelocity = errorAngle/20; // make angle magnitude between 0 and 1
////            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.AUTOBALANCE);
////            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, newVelocity);
//            // ^^^^^^^^^ im setting the desired throttle right here, but the database is pretending like these changes were never made
//            mTable.getEntry("throttleInIf").setString("" + db.drivetrain.get(EDriveData.DESIRED_THROTTLE_PCT) + " at " + System.currentTimeMillis()%10000);
//            //mTable.getEntry("throttle").setString("" + db.drivetrain.get(EDriveData.DESIRED_THROTTLE_PCT) + " at " + System.currentTimeMillis()); // when printed in this class,
//            // the thing is, ^^^^^ this is updating the throttle, and it works. It is getting stored in the database. but when neodrive looks in the database
//            // it doesn't see the changes
//            // it recognizes it. but when printed in the neodrivemodule, it doesn't change
//            mTable.getEntry("finshed").setString("not finished");
//
//            return false; // it is running a single command. its only this. just a command no group
//        }
//        else if (mCurrentRoll < mBalancedLowBound) { // roll too negative - drive backward to balance
//            mTable.getEntry("AutoBalance Status").setString("Lower than low bound");
//
//            // identify the angle between the current roll and the low bound
//            errorAngle = mCurrentRoll - mBalancedLowBound; // always negative
//            // the greater the errorAngle, the faster the robot should correct its balance
//            newVelocity = errorAngle/20; // make angle magnitude between 0 and 1
////            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.AUTOBALANCE);
////            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, newVelocity);
//
//            mTable.getEntry("throttleInIf").setString("" + db.drivetrain.get(EDriveData.DESIRED_THROTTLE_PCT) + " at " + System.currentTimeMillis()%10000);
//            //mTable.getEntry("throttle").setString("" + db.drivetrain.get(EDriveData.DESIRED_THROTTLE_PCT) + " at " + System.currentTimeMillis());
//
//            mTable.getEntry("finshed").setString("not finished");
//            return false;
//        }
//        else { // in between bounds
//            mTable.getEntry("finshed").setString("finished");
//            return true;
//        }
//    }
//}













//        double desiredHeading = mTrajectory.sample(mTrajectory.getTotalTimeSeconds()).poseMeters.getRotation().getDegrees();
//        if(Robot.DATA.imu.get(EGyro.HEADING_DEGREES) < desiredHeading)  {
//            Robot.DATA.drivetrain.set(EDriveData.DESIRED_TURN_PCT, (Robot.DATA.drivetrain.get(EDriveData.DESIRED_TURN_PCT) + 0.1));
//        } else {
//            Robot.DATA.drivetrain.set(EDriveData.DESIRED_TURN_PCT, (Robot.DATA.drivetrain.get(EDriveData.DESIRED_TURN_PCT) - 0.1));
//        }

//    public void autoBalance()
//    {
//        /* TODO If holding a button, move by the pitch scaled to a number
//        If pitch is 30 move forward, if pitch is 15 move forawrd a little less. If pitch is -30 move backward,
//        if pitch is -15 move backward a little less.
//        do these while a button is pressed and while the pitch is not within a range near 0 degrees.
//         */
//
//
//
//        // get current left and right velocities
//        //double currentLeftVelocity = db.drivetrain.get(EDriveData.L_DESIRED_VEL_FT_s);
//        //double currentRightVelocity = db.drivetrain.get(EDriveData.R_DESIRED_VEL_FT_s);
//
//        double currentPitch = db.imu.get(EGyro.ROOL_DEGREES); // db.imu.get(EGyro.PITCH_DEGREES); pitch and roll are switched
//        if (currentPitch < errorLowBound)
//        {
//            // move robot backwards by a factor of angle between pitch and horizontal
//            double factor = -1*Math.abs(currentPitch - errorLowBound); // corrects faster if there is a larger angle
//
//            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, factor/180);
//            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, factor/180);
//
//        }
//        else if (currentPitch > errorHighBound)
//        {
//            // move robot forwards by a factor of angle between pitch and horizontal
//            double factor = Math.abs(currentPitch - errorHighBound); // corrects faster if there is a larger angle
//
//            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, factor/180);
//            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, factor/180);
//        }
//        else // when within error range
//        {
//            // stop robot
//            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, 0);
//            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, 0);
//        }
//    }

//mCommand = commandGenerator.generateCommand(mTrajectoryName);
//        mCommand.beforeStarting(() -> mNeoDrive.resetOdometry(commandGenerator.getTrajInitPose()));
//mTable.getEntry("initial pose").setString(commandGenerator.getTrajInitPose().toString());
//commandGenerator.generateCommand(mTrajectoryName).schedule(false);











// Megans code from the autobalance rework ------------------------------------------

//
//package us.ilite.robot.commands;
//
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
//import us.ilite.robot.modules.NeoDriveModule;
//
//public class AutoBalance extends PIDCommand {
//        NeoDriveModule driveSubsystem;
//
//        public AutoBalance(NeoDriveModule pDriveSubsystem, double pSetpoint) { // im trying to figure out how this gets the robot to move
//            // while what i was doing before doesn't. And i tried that yesterday. We went through a whole lot of tests after you left and
//            // there doesn't seem to be a rhyme or reaso
//            // yeah. we werent using pid
//            // we were logging it and it wasn't
//
//            // one of the problems was that you could set the database in the command, but when you looked at the database in teleo
//            super(new PIDController(0.1, 0, 0), pDriveSubsystem::getGyroRollDeg, pSetpoint, //1.5 is about the base roll in deg
//                output -> {
//                    pDriveSubsystem.setThrottlePct(-output); //-output * Settings.kMaxSpeedMetersPerSecond
//                },
//                    pDriveSubsystem);
//            driveSubsystem = pDriveSubsystem;
//
//        }
//
//    @Override
//    public boolean isFinished() {
//            System.out.println("position error " + m_controller.getPositionError());
//        return Math.abs(m_controller.getPositionError()) < 2.5;
//    }
//    @Override
//    public void end(boolean interrupted) {
//        super.end(interrupted);
//        System.out.println("Robot Balanced!");
//        driveSubsystem.setThrottlePct(0);
//    }
//}



//    private class Balance extends PIDCommand{
//        NeoDriveModule driveSubsystem;
//        private Balance(NeoDriveModule driveSubsystem, double setpoint) {
//            super(new PIDController(0.1, 0, 0), driveSubsystem::getGyroRollDeg, setpoint,
//                    output -> {
//                        driveSubsystem.setThrottlePct(output); //-output * Settings.kMaxSpeedMetersPerSecond
//                    },
//            driveSubsystem);
//            this.driveSubsystem = driveSubsystem;
//
//        }
//
//        @Override
//        public boolean isFinished() {
//            return Math.abs(m_controller.getPositionError()) < 1;
//        }
//        @Override
//        public void end(boolean interrupted) {
//            super.end(interrupted);
//            System.out.println("Robot Balanced!");
//            driveSubsystem.setThrottlePct(0);
//        }
//    }


