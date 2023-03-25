package us.ilite.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import us.ilite.common.types.sensor.EClawData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.modules.ClawModule;

public class SpinIntake extends CommandBase {
    private ClawModule intake;

    @Override
    public void initialize() {
        intake = ClawModule.getInstance();
    }
    @Override
    public void execute() {
        Robot.DATA.claw.set(EClawData.ROLLER_STATE, Enums.EClawMode.PERCENT_OUTPUT);
        // CONSTANT
        Robot.DATA.claw.set(EClawData.DESIRED_ROLLER_pct, 0.5);
    }
    @Override
    public boolean isFinished() {
        if(intake.getmClawSensor().isBroken()) {
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Robot.DATA.claw.set(EClawData.DESIRED_ROLLER_pct, 0.0);
    }
}
