package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;

import static us.ilite.common.types.EIntakeData.ROLLER_VEL_ft_s;

public class falconMotor extends Module {
    private TalonFX mMotor;
    private final NetworkTable mTable;
    public falconMotor() {
        TalonFX mMotor = new TalonFX(9);
        mTable = NetworkTableInstance.getDefault().getTable("falconMotor");
    }
    public void modeInit(EMatchMode mode) {

    }
    public void readInputs() {
        db.intake.set(EIntakeData.ROLLER_PCT, mMotor.getMotorOutputPercent());
        db.intake.set(ROLLER_VEL_ft_s, mMotor.getSelectedSensorVelocity());
        db.intake.set(EIntakeData.ARM_STATE, mMotor.getSelectedSensorPosition());
    }
    public void setOutputs() {
        mMotor.set(TalonFXControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
        mMotor.set(TalonFXControlMode.Velocity, db.intake.get(EIntakeData.ROLLER_VEL_ft_s));
        mMotor.set(TalonFXControlMode.Position, db.intake.get(EIntakeData.ARM_STATE));
        mTable.getEntry(" DESIRED_ROLLER_pct" ).setNumber(db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
    }
}
