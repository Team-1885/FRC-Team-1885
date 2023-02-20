package us.ilite.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import us.ilite.robot.modules.NeoDriveModule;

public class FollowTrajectory extends CommandBase {
    Timer mTimer;
    GenerateRamseteCommand commandGenerator;
    private NetworkTable mTable;
    private String mTrajectoryName;

    public FollowTrajectory(String pTrajectoryName) {
        mTimer = new Timer();
        commandGenerator = new GenerateRamseteCommand();
        addRequirements(NeoDriveModule.getInstance());
        mTable = NetworkTableInstance.getDefault().getTable(pTrajectoryName);
        mTrajectoryName = pTrajectoryName;

    }

    @Override
    public void initialize() {
        mTimer.start();
        commandGenerator.generateCommand(mTrajectoryName).schedule(false);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
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
