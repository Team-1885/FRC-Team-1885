package us.ilite.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import us.ilite.robot.modules.ClawModule;

public class SpinIntake extends CommandBase {
    private ClawModule intake;

    @Override
    public void initialize() {
        intake = ClawModule.getInstance();
    }
    @Override
    public void execute() {
        intake.setMotors(0.5);
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
        intake.setMotors(0.0);
    }
}
