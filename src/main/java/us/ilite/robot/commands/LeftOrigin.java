package us.ilite.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import us.ilite.robot.modules.NeoDriveModule;

public class LeftOrigin extends CommandBase {
    Timer mTimer;
    GenerateRamseteCommand commandGenerator;
    private NetworkTable mTable;

    public LeftOrigin() {
        mTimer = new Timer();
        commandGenerator = new GenerateRamseteCommand();
        addRequirements(NeoDriveModule.getInstance());
        mTable = NetworkTableInstance.getDefault().getTable("left origin");

    }

    @Override
    public void initialize() {
        mTimer.start();
        commandGenerator.generateCommand("LeftOrigin").schedule(false);
        mTable.getEntry("left origin default").setNumber(1);
    }

    @Override
    public void execute() {
        mTable.getEntry("left origin default").setNumber(2);
    }

    @Override
    public void end(boolean interrupted) {
        mTable.getEntry("left origin timer").setString("left origin end");
    }

    @Override
    public boolean isFinished() {
        mTable.getEntry("left origin timer").setNumber(mTimer.get());
        if(mTimer.get()>commandGenerator.getTotalTimeSeconds()){

            System.out.println("ORIGIN FINISHED");
            return true;
        }
        return false;
    }
}
