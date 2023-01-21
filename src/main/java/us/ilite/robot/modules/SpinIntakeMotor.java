package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EIntakeData.*;
import static us.ilite.common.types.drive.EDriveData.STATE;

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

        db.intake.set(ROLLER_PCT, (mTalonFX.getSelectedSensorVelocity() * IntakeModule.kScaledUnitsToRPM) / IntakeModule.kMaxFalconSpeed);
        db.intake.set(ROLLER_VEL_ft_s, mTalonFX.getSelectedSensorVelocity() * IntakeModule.kFeetSpeedConversion);
        db.intake.set(EIntakeData.ARM_STATE, mTalonFX.getSelectedSensorPosition());
    }
    @Override
    public void setOutputs() {
        setRollerState();
        mTalonFX.set(TalonFXControlMode.Position, db.intake.get(DESIRED_ARM_STATE));
        mTable.getEntry("Roller_Pct").setNumber(db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
        mTable.getEntry("Velocity").setNumber(db.intake.get(SET_ROLLER_VEL_ft_s));
        mTable.getEntry("Position").setNumber(db.intake.get(DESIRED_ARM_STATE));
    }
    public void setRollerState() {
        Enums.ERollerState state = db.intake.get(ROLLER_STATE,Enums.ERollerState.class);
        switch(state) {
            case PERCENT_OUTPUT:
                mTalonFX.set(TalonFXControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
                break;
            case VELOCITY:
                mTalonFX.set(TalonFXControlMode.Velocity, db.intake.get(EIntakeData.SET_ROLLER_VEL_ft_s));
                break;
        }
    }

}
