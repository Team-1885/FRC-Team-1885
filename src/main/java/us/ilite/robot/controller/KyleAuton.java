package us.ilite.robot.controller;


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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import us.ilite.common.Data;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.NeoDriveModule;

import java.util.List;
import java.util.function.Supplier;
// NOT SURE IF THIS CODE ACTUALLY GETS THE KINEMATIC CONSTANT OF THE ROBOT
// new DifferentialDriveKinematics(Units.feet_to_meters(NeoDriveModule.kTrackWidthFeet)

public class KyleAuton extends BaseAutonController {
    protected final Data db = Robot.DATA;
    private DifferentialDriveKinematics mDriveKinematics; // save instance kDriveKinematics for reuse
    private NeoDriveModule mRobotDrive; // get singleton instance
    private Command mAuton; // save command instance for reuse

    @Override
    public void initialize() { // called once
        mDriveKinematics = new DifferentialDriveKinematics(Units.feet_to_meters(NeoDriveModule.kTrackWidthFeet));
        mRobotDrive = NeoDriveModule.getInstance();
        mAuton = getTrajectoryInstructions();
        mAuton.schedule();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    @Override
    public void updateImpl() { // called periodically
        if (mAuton != null) // make sure command exists
        {
            mAuton.execute(); // this follows the actual trajectory
            mAuton.cancel();
        }
    }
    public Command getTrajectoryInstructions() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        TrajectoryConstraint autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                Settings.kS, // Volts
                                Settings.kV, // VoltSecondsPerMeter
                                Settings.kA), // VoltSecondsSquaredPerMeter
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
                        //new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2, 0, new Rotation2d(0))),
                        // End 3 meters straight ahead of where we started, facing forward
                        //new Pose2d(3, 0, new Rotation2d(0)),
                        // Pass config
                        config);

        RamseteCommand ramseteCommand =

                new RamseteCommand(
                        exampleTrajectory,
                        mRobotDrive::getRobotPose, //(Supplier<Pose2d>) getRobotPose(), //m_robotDrive::getPose,
                        new RamseteController(
                                Settings.kRamseteB, // kRamseteB
                                Settings.kRamseteZeta // kRamseteZeta
                        ),
                        new SimpleMotorFeedforward(
                                Settings.kS, // Volts
                                Settings.kV, // VoltSecondsPerMeter
                                Settings.kA // VoltSecondsSquaredPerMeter
                        ),
                        mDriveKinematics,
                        mRobotDrive::getWheelSpeeds,
//                                mDriveKinematics.toWheelSpeeds(
//                                        new ChassisSpeeds(
//                                                initialState.velocityMetersPerSecond,
//                                                0,
//                                                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond)),  //::getWheelSpeeds,
                        new PIDController(0.1, 0, 0), // left controller
                        new PIDController(0.1, 0, 0), // right controller
                        // RamseteCommand passes volts to the callback
                        mRobotDrive::tankDriveVolts, //BiConsumer<Double, Double> outputVolts = new BiConsumer<Double, Double> = m_robotDrive::tankDriveVolts, //(leftVolts, rightVolts) -> m_robotDrive.tankDriveVolts(leftVolts, rightVolts),
                        mRobotDrive
                );

        // Reset odometry to the starting pose of the trajectory.
        mRobotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> mRobotDrive.tankDriveVolts(0, 0));
    }
    // TODO --------------------------
    // take a set of points and create a trajectory between them
    // create command method that sets up trajectories with ramsete
}

