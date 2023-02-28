package us.ilite.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.NeoDriveModule;

public class StopMoving extends CommandBase {
    private NeoDriveModule mNeoDrive;

    public StopMoving() {
        mNeoDrive = NeoDriveModule.getInstance();
        addRequirements(mNeoDrive);
    }

    public void initialize() {
        mNeoDrive.setVolts(0,0);
    }

    public void execute() {}

    public void end(boolean interrupted) {}

    public boolean isFinished() {
        if (Robot.DATA.drivetrain.get(EDriveData.LEFT_VOLTAGE) == 0
        && Robot.DATA.drivetrain.get(EDriveData.LEFT_VOLTAGE) == 0) {
            return true;
        }
        return false;
    }
}
