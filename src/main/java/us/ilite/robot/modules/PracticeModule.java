package us.ilite.robot.modules;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import us.ilite.common.types.EIntakeData;
import us.ilite.robot.hardware.TalonSRXFactory;
import us.ilite.robot.modules.FalconDriveModule;

public class PracticeModule  extends Module {

    private TalonFX mMotor;
    private double speed;


    public void PracticeModule() {
        TalonFX mMotor = new TalonFX(0);

    }

    @Override
    public void modeInit(){

    }

    @Override
    public void readInputs() {
        db.intake.set(EIntakeData.ROLLER_PCT, mMotor.);

    }

    @Override
    public void setOutputs(){


    }


}
