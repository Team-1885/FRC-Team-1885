package us.ilite.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import us.ilite.robot.modules.NeoDriveModule;

public class LeftPiece extends CommandBase {
    private Timer mTimer;

    public LeftPiece() {
        mTimer = new Timer();
        commandGenerator = new GenerateRamseteCommand();
        addRequirements(NeoDriveModule.getInstance());
    }

    @Override
    public void initialize() {
        mTimer.start();
        commandGenerator.generateCommand("LeftPiece").schedule(false);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if(mTimer.get()>commandGenerator.getTotalTimeSeconds()){

            System.out.println("PIECE FINISHED");
            return true;
        }
        return false;
    }

    private GenerateRamseteCommand commandGenerator;
}
