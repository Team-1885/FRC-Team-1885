package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;

public class SpinIntakeMotor extends Module {
    private TalonFX mTalonFX;
    public SpinIntakeMotor() {
        mTalonFX = new TalonFX(9);
    }
    @Override
    public void modeInit(EMatchMode pMode) {

    }

    @Override
    public void readInputs() {
        db.intake.set(EIntakeData.ROLLER_PCT, mTalonFX.getMotorOutputPercent());
    }
    @Override
    public void setOutputs() {
        mTalonFX.set(TalonFXControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
    }

}
