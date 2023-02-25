package us.ilite.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import us.ilite.common.types.EIntakeData;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
//import us.ilite.robot.modules.IntakeModule;

public class SpinIntake extends CommandBase {

    Timer mTimer;
    GenerateRamseteCommand commandGenerator;
//    IntakeModule intake;


    public SpinIntake() {
        mTimer = new Timer();
        commandGenerator = new GenerateRamseteCommand();
//        intake = IntakeModule.getInstance();
//        addRequirements(IntakeModule.getInstance());
    }

    @Override
    public void initialize() {
        mTimer.start();
    }

    @Override
    public void execute() {
        System.out.println("INTAKE EXECUTING");
        Robot.DATA.intake.set(EIntakeData.PNEUMATIC_STATE, Enums.EArmState.EXTEND);
        Robot.DATA.intake.set(EIntakeData.DESIRED_ROLLER_pct, 0.5);
    }

    @Override
    public boolean isFinished() {
        if(mTimer.get()>5){

            return true;
        }
        return false;
    }
}
