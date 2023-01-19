package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;

import static us.ilite.common.types.EIntakeData.ARM_STATE;
import static us.ilite.common.types.EIntakeData.ROLLER_VEL_ft_s;

public class SpinIntakeMotor extends Module {
    private TalonFX mTalonFX;
    private final NetworkTable mTable;
    public SpinIntakeMotor() {
        mTalonFX = new TalonFX(9);
        mTable = NetworkTableInstance.getDefault().getTable("intake");
    }
    @Override
    public void modeInit(EMatchMode pMode) {

    }

    @Override
    public void readInputs() {

        db.intake.set(EIntakeData.ROLLER_PCT, mTalonFX.getMotorOutputPercent());
        db.intake.set(ROLLER_VEL_ft_s, mTalonFX.getSelectedSensorVelocity());
        db.intake.set(EIntakeData.ARM_STATE, mTalonFX.getSelectedSensorPosition());
    }
    @Override
    public void setOutputs() {
        mTalonFX.set(TalonFXControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
        mTalonFX.set(TalonFXControlMode.Position, db.intake.get(ARM_STATE));
        mTalonFX.set(TalonFXControlMode.Velocity,db.intake.get(EIntakeData.ROLLER_VEL_ft_s));
        mTable.getEntry("Roller_Pct").setNumber(db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
    }

}
