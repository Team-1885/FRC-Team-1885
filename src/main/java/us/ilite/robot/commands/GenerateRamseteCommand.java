package us.ilite.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Units;
import us.ilite.robot.modules.NeoDriveModule;

import javax.swing.*;
import java.util.List;

public class GenerateRamseteCommand {

    private RamseteCommand mRamseteCommand;
    private NeoDriveModule mRobotDrive; // get singleton instance
    private DifferentialDriveKinematics mDriveKinematics; // save instance kDriveKinematics for reuse
    private PIDController mLeftDrivePID;
    private PIDController mRightDrivePID;
    private SimpleMotorFeedforward mFeedForward;

    public GenerateRamseteCommand() {
        mRobotDrive = NeoDriveModule.getInstance();
        mDriveKinematics = new DifferentialDriveKinematics(Units.feet_to_meters(NeoDriveModule.kTrackWidthFeet));
        mLeftDrivePID = new PIDController(Settings.kP, 0, 0);
        mRightDrivePID = new PIDController(Settings.kP, 0, 0);
        mFeedForward = new SimpleMotorFeedforward(Settings.kS, Settings.kV, Settings.kA);
    }

    public Command generateCommand() { // TODO implement with path weaver such that one may pass in the .json with the trajectory info
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
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, new Rotation2d(0))),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(2, 0, new Rotation2d(0)),
                        // Pass config
                        config);

        mRamseteCommand =
                new RamseteCommand(
                        exampleTrajectory,
                        mRobotDrive::getPose, //(Supplier<Pose2d>) getRobotPose(), //m_robotDrive::getPose,
                        new RamseteController(
                                Settings.kRamseteB, // kRamseteB
                                Settings.kRamseteZeta // kRamseteZeta
                        ),
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
        mRobotDrive.resetOdometry(exampleTrajectory.getInitialPose());
        return mRamseteCommand;
    }

}
