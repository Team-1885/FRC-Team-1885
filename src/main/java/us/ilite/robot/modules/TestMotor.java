package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import us.ilite.common.types.EIntakeData;

public class TestMotor extends Module {
    private final TalonFX mIntakeMotor;
    public TestMotor(){
        mIntakeMotor = new TalonFX(9);
    }
    public void modeInIt(){

    }
    public void setOutputs(){
        mIntakeMotor.set(ControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));

    }
    public void readInputs(){
        db.intake.set(EIntakeData.ROLLER_PCT, mIntakeMotor.getMotorOutputPercent());
    }
}
