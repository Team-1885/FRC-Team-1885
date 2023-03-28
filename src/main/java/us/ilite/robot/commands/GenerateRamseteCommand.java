package us.ilite.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Units;
import us.ilite.robot.modules.NeoDriveModule;

public class GenerateRamseteCommand {

    private PPRamseteCommand mRamseteCommand;
    private NeoDriveModule mRobotDrive; // get singleton instance
    private DifferentialDriveKinematics mDriveKinematics; // save instance kDriveKinematics for reuse
    private PIDController mLeftDrivePID;
    private PIDController mRightDrivePID;
    private SimpleMotorFeedforward mFeedForward;
    private RamseteController mRamseteController;
    private NetworkTable mTable;

    public GenerateRamseteCommand() {
        mTable = NetworkTableInstance.getDefault().getTable("ramsete command");

        mRamseteController= new  RamseteController(
                Settings.kRamseteB, // kRamseteB
                Settings.kRamseteZeta // kRamseteZeta
        );
        mRobotDrive = NeoDriveModule.getInstance();
        mDriveKinematics = new DifferentialDriveKinematics(Units.feet_to_meters(NeoDriveModule.kTrackWidthFeet));
        mLeftDrivePID = new PIDController(Settings.kP, 0, 0);
        mRightDrivePID = new PIDController(Settings.kP, 0, 0);
        mFeedForward = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);
    }
    private PathPlannerTrajectory desiredTrajectory;
    private double trajectoryTime;
    public PPRamseteCommand generateCommand(PathPlannerTrajectory trajectory) { // TODO implement with path weaver such that one may pass in the .json with the trajectory info
        // Create a voltage constraint to ensure we don't accelerate too fast
        TrajectoryConstraint autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        mFeedForward,
                        mDriveKinematics,
                        10);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Settings.kMaxSpeedMetersPerSecond, // kMaxSpeedMetersPerSecond
                        Settings.kMaxAccelerationMetersPerSecondSquared // kMaxAccelerationMetersPerSecoondSquared
                )
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(mDriveKinematics); // kDriveKinematics

        // An example trajectory to follow.  All units in meters.
//        Trajectory exampleTrajectory =
//                TrajectoryGenerator.generateTrajectory(
//                        // Start at the origin facing the +X direction
//                        new Pose2d(0, 0, new Rotation2d(0)),
//                        // Pass through these two interior waypoints, making an 's' curve path
//                        List.of(new Translation2d(0.25, 0.25), new Translation2d(0.5, -0.25)),
//                        // End 3 meters straight ahead of where we started, facing forward
//                        new Pose2d(2, 0, new Rotation2d(0)),
//                        // Pass config
//                        config);
//        Trajectory DriveStraight = TrajectoryCommandUtils.getJSONTrajectory("DriveStraight");
//        Trajectory LeftPiece = TrajectoryCommandUtils.getJSONTrajectory("LeftPiece");
//        Trajectory LeftOrigin = TrajectoryCommandUtils.getJSONTrajectory("LeftOrigin");
//        Trajectory LeftScore = TrajectoryCommandUtils.getJSONTrajectory("LeftScore");
//        Trajectory MiddlePieceDock = TrajectoryCommandUtils.getJSONTrajectory("MiddlePieceDock");
//        Trajectory RightOrigin = TrajectoryCommandUtils.getJSONTrajectory("RightOrigin");
//        Trajectory RightPiece = TrajectoryCommandUtils.getJSONTrajectory("RightPiece");
//        Trajectory RightScore = TrajectoryCommandUtils.getJSONTrajectory("RightScore");
//          desiredTrajectory = TrajectoryCommandUtils.getJSONTrajectory(trajectory);
//        desiredTrajectory = PathPlanner.loadPath(trajectory, new PathConstraints(2,1));
//        trajectoryTime = desiredTrajectory.getTotalTimeSeconds();
//        mTable.getEntry("state").setString(desiredTrajectory.sample(trajectoryTime + 1).toString());
        desiredTrajectory = trajectory;
        mRamseteCommand =
                new PPRamseteCommand(
                        desiredTrajectory,
                        mRobotDrive::getPose,
                        mRamseteController,
                        mFeedForward,
                        mDriveKinematics,
                        mRobotDrive::getWheelSpeeds,
                        mLeftDrivePID, // left controller
                        mRightDrivePID, // right controller
                        // RamseteCommand passes volts to the callback
                        mRobotDrive::setVolts,
                        mRobotDrive
                );
        // Reset odometry to the starting pose of the trajectory.
//        mRobotDrive.resetOdometry(desiredTrajectory.getInitialPose());
//        mTable.getEntry("initial pose").setString((desiredTrajectory.getInitialPose()).toString());
        return mRamseteCommand;
    }
    public double getTotalTimeSeconds(){
        return trajectoryTime;
    }

    public Pose2d getTrajInitPose() {
        return desiredTrajectory.getInitialPose();
    }
}
