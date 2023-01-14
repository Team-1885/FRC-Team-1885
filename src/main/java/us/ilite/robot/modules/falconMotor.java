package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;

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
    }
    public void setOutputs() {
        mMotor.set(TalonFXControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));

        mTable.getEntry(" DESIRED_ROLLER_pct" ).setNumber(db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
    }
}
