package us.ilite.robot.controller;

import edu.wpi.first.wpilibj.Timer;
import us.ilite.robot.commands.FollowRamseteCommand;

public class FollowRamseteTrajectory extends BaseAutonController{

    private FollowRamseteCommand mRamseteCommand;
    private Timer mTimer;


    public void initialize() {
        mRamseteCommand = new FollowRamseteCommand();
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();

        mRamseteCommand.init(mTimer.get());
    }
    public void updateImpl() {

    }

}
