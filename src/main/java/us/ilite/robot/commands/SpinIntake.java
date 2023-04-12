package us.ilite.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import us.ilite.robot.modules.ClawModule;
import edu.wpi.first.wpilibj.Timer;

public class SpinIntake extends CommandBase {
    private ClawModule intake;
    private NetworkTable mTable;
    private Timer mTimer;

    @Override
    public void initialize() {
        intake = ClawModule.getInstance();
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
        mTable = intake.getGlass();
    }
    @Override
    public void execute() {
        intake.setMotors(0.2);
        mTable.getEntry("PercentOutput").setNumber(ClawModule.getInstance().getMotors());
    }
    @Override
    public boolean isFinished() {

        if (mTimer.get() > 2.0) {
            return true;
        }
        return false;

    /*
        if(intake.getmClawSensor().isBroken()) {
            System.out.println("finish true finish start finish start finish start");
            return false;
//          For some reason, the value is being set to true even when the bream break is not broken, so it is set to false for both, thus fixing this problem temporarily, however the weird noise is still made.
        }
        System.out.println("finish false finish false finish false");
        return false;
    */

    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        intake.setMotors(0.0);
        mTable.getEntry("PercentOutput").setNumber(0.0);
        System.out.println("end end end end end end end end end end end");
    }
}
