package us.ilite.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import us.ilite.robot.modules.NeoDriveModule;

public class LeftPiece extends CommandBase {
    private Timer mTimer;
    private NetworkTable mTable;

    public LeftPiece() {
        mTimer = new Timer();
        commandGenerator = new GenerateRamseteCommand();
        addRequirements(NeoDriveModule.getInstance());
        mTable = NetworkTableInstance.getDefault().getTable("left piece");
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
        mTable.getEntry("left piece timer").setNumber(mTimer.get());
        if(mTimer.get()>commandGenerator.getTotalTimeSeconds()){
            System.out.println("PIECE FINISHED");
            return true;
        }
        return false;
    }

    private GenerateRamseteCommand commandGenerator;
}
