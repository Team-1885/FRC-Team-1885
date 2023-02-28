package us.ilite.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.Pigeon;
import us.ilite.robot.modules.NeoDriveModule;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class AutoBalancePID extends SequentialCommandGroup {
    private final double kP = Settings.kP;
    private Data db = Robot.DATA;
    private Pigeon mGyro;




    private class Balance extends PIDCommand {
        NeoDriveModule driveSubsystem;

        private NetworkTable mTable = NetworkTableInstance.getDefault().getTable("AutoBalance");


        public Balance(NeoDriveModule driveSubsystem, double setpoint) {
            super(new PIDController(kP, 0, 0), driveSubsystem.getGyroRoll()::getDegrees, setpoint, output -> {
                db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, -output * Settings.kMaxSpeedMetersPerSecond);
                }, driveSubsystem);
            this.driveSubsystem = driveSubsystem;

            mTable.getEntry("Balance").setString("Created Balance Class");
        }

        @Override
        public boolean isFinished() {
            return Math.abs(m_controller.getPositionError()) < 1;
        }
        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            System.out.println("Robot Balanced!");
            db.drivetrain.set(EDriveData.DESIRED_THROTTLE_PCT, 0);
        }
    }

    public AutoBalancePID() {
        NetworkTable mTable = NetworkTableInstance.getDefault().getTable("AutoBalance");

        NeoDriveModule driveSubsystem = NeoDriveModule.getInstance();
        // Since we cannot zero the ROLL of the gyro, take the initial roll
        // Before we balance (the roll when the robot is flat) and make this our setpoint
        final double initialRoll = driveSubsystem.getGyro().getRoll().getDegrees();
        addCommands(
                // drive on top of the charge station
                //db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, 1),
                // TODO create path that puts the robot on chargestation
//                new FollowTrajectory("Drive onto charge station"),
//                new DriveStraight(10)
                new Balance(driveSubsystem, initialRoll)
        );
        mTable.getEntry("AutoBalanceCommands").setString("Added Commands");
    }
}









//
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import us.ilite.common.Data;
//import us.ilite.common.types.drive.EDriveData;
//import us.ilite.common.types.sensor.EGyro;
//import us.ilite.robot.Robot;
//
//// this command will need to start before the robot drives up onto the charge station
//// trajectory should end at the foot of the charge station
//// gyro should zero
//// get on top of the charge station and start the pid loop
///** Balance the robot on top of the charge station game element:
// *
// * The AutoBalance Command is run once the robot is on top of the charge station.
// * It will continuously check the robots pitch and determine if it considers a pitch balanced.
// *      If the robot is not balanced, it will drive forward or backward to correct the balance.
// *      If the robot is balanced, the robot will stop its motion.
// */
//public class AutoBalancePID extends CommandBase {
//    private Data db = Robot.DATA;
//    private NetworkTable mTable;
//    // set a range of pitch values where the robot is considered balanced. A range is needed because the robot will never be exactly horizontal
//    private double mBalancedRange = 6; // total degrees in range
//    private double mCenterDegreeOfRange = 0; // degree considered the center of the range
//    private double mBalancedLowBound; // lowest degree considered balanced
//    private double mBalancedHighBound; // highest degree considered balanced
//    // keep track of the current robot pitch
//    private double mCurrentPitch;
//
//    @Override
//    public void initialize() {
//        mTable = NetworkTableInstance.getDefault().getTable("AutoBalance");
//
//        // calculate the low and high bound of the range of degrees where the robot is considered balanced
//        mBalancedLowBound = mCenterDegreeOfRange - (mBalancedRange / 2);
//        mBalancedHighBound = mCenterDegreeOfRange + (mBalancedRange / 2);
//    }
//
//    @Override
//    public void execute() { // continuously executes until the command is finished
//        // update current pitch
//        mCurrentPitch = db.imu.get(EGyro.PITCH_DEGREES);
//    }
//
//    @Override
//    public void end(boolean interrupted) { // executes once after the command ends
//        // robot is balanced, stop robot
//        db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, 0.0);
//        db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, 0.0);
//    }
//
//    @Override
//    public boolean isFinished() { // command scheduler continually checks if the command is finished
//        mTable.getEntry("Current Pitch").setNumber(mCurrentPitch);
//
//
//        if(mCurrentPitch > mBalancedHighBound) { // pitch too positive - drive forward to balance
//            mTable.getEntry("AutoBalance Status").setString("Greater than high bound");
//
//            // identify the angle between the current pitch and the high bound
//            double errorAngle = mCurrentPitch - mBalancedHighBound; // always positive
//            // the greater the errorAngle, the faster the robot should correct its balance
//            double newVelocity = errorAngle/360; // make angle magnitude between 0 and 1
//
//            // drive forward
//            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, newVelocity);
//            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, newVelocity);
//
//            return false;
//        }
//        else if (mCurrentPitch < mBalancedLowBound) { // pitch too negative - drive backward to balance
//            mTable.getEntry("AutoBalance Status").setString("Lower than low bound");
//
//            // identify the angle between the current pitch and the low bound
//            double errorAngle = mCurrentPitch - mBalancedLowBound; // always negative
//            // the greater the errorAngle, the faster the robot should correct its balance
//            double newVelocity = errorAngle/360; // make angle magnitude between 0 and 1
//
//            // drive backward
//            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, newVelocity);
//            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, newVelocity);
//
//            return false;
//        }
//        else { // in between bounds
//            return true;
//        }
//
//
//    }
//
//}