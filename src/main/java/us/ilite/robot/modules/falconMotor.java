package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EIntakeData.*;

public class falconMotor extends Module {
    private TalonFX mMotor;
    private final NetworkTable mTable;
    public falconMotor() {
        mMotor = new TalonFX(9);
        mTable = NetworkTableInstance.getDefault().getTable("falconMotor");
    }
    public void modeInit(EMatchMode mode) {

    }
    public void readInputs() {
        db.intake.set(EIntakeData.ROLLER_PCT, mMotor.getMotorOutputPercent());
        db.intake.set(ROLLER_VEL_ft_s, mMotor.getSelectedSensorVelocity() * IntakeModule.kFeetSpeedConversion);

    }
    public void setOutputs() {
        setRollerState();
        mMotor.set(TalonFXControlMode.Position, db.intake.get(EIntakeData.DESIRED_ARM_STATE));
        mTable.getEntry(" Roller_Pct" ).setNumber(db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
        mTable.getEntry("Velocity").setNumber(db.intake.get(EIntakeData.SET_ROLLER_VEL_ft_s));
        mTable.getEntry("Position").setNumber(db.intake.get(EIntakeData.ARM_STATE));
    }
    public void setRollerState() {
        Enums.ERollerState mode = db.intake.get(ROLLER_STATE, Enums.ERollerState.class);
        if (mode == null) {
            mode = Enums.ERollerState.PERCENT_OUTPUT;
        }
        switch (mode) {
            case PERCENT_OUTPUT:
                mMotor.set(TalonFXControlMode.PercentOutput, db.intake.get(DESIRED_ROLLER_pct));
                break;
            case VELOCITY:
                mMotor.set(TalonFXControlMode.Velocity, db.intake.get(EIntakeData.SET_ROLLER_VEL_ft_s));
                break;
        }
    }
    public  void idk() {
        Enums.ear
    }
}
