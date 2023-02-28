package us.ilite.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team319.trajectory.Path;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Robot;
import us.ilite.robot.TrajectoryCommandUtils;
import us.ilite.robot.modules.NeoDriveModule;

public class FollowTrajectory extends CommandBase {
    Timer mTimer;
    GenerateRamseteCommand commandGenerator;
    private NetworkTable mTable;
    private String mTrajectoryName;
    private Command mCommand;
    private NeoDriveModule mNeoDrive;

    private PathPlannerTrajectory mTrajectory;

    public FollowTrajectory(String pTrajectoryName) {
        mTimer = new Timer();
        mTable = NetworkTableInstance.getDefault().getTable("follow traj");

        commandGenerator = new GenerateRamseteCommand();
        mNeoDrive = NeoDriveModule.getInstance();
        mTrajectoryName = pTrajectoryName;
        mTrajectory = PathPlanner.loadPath(pTrajectoryName, 2, 1);
//        mTrajectory = TrajectoryCommandUtils.getJSONTrajectory(pTrajectoryName);

        addRequirements(mNeoDrive);
    }

    @Override
    public void initialize() {
        mTimer.start();
        mCommand = commandGenerator.generateCommand(mTrajectoryName);
        mTable.getEntry("initial pose").setString(commandGenerator.getTrajInitPose().toString());
        mCommand.schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
//        double desiredHeading = mTrajectory.sample(mTrajectory.getTotalTimeSeconds()).poseMeters.getRotation().getDegrees();
//        if(Robot.DATA.imu.get(EGyro.HEADING_DEGREES) < desiredHeading)  {
//            Robot.DATA.drivetrain.set(EDriveData.DESIRED_TURN_PCT, (Robot.DATA.drivetrain.get(EDriveData.DESIRED_TURN_PCT) + 0.1));
//        } else {
//            Robot.DATA.drivetrain.set(EDriveData.DESIRED_TURN_PCT, (Robot.DATA.drivetrain.get(EDriveData.DESIRED_TURN_PCT) - 0.1));
//        }
    }

    @Override
    public boolean isFinished() {
        mTable.getEntry("trajectory timer").setNumber(mTimer.get());
        if(mTimer.get()>commandGenerator.getTotalTimeSeconds()){
            mTable.getEntry("follow trajectory").setString("finished");
            return true;
        }
        return false;
    }
}
