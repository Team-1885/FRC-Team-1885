package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EIntakeData.*;

public class falconMotor extends Module {
    private TalonFX mMotor;
    private DoubleSolenoid mThingy;
    private final NetworkTable mTable;
    public falconMotor() {
        mMotor = new TalonFX(9);
        mTable = NetworkTableInstance.getDefault().getTable("falconMotor");
        mThingy = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, Settings.HW.PCH.kINPNIntakeForward, Settings.HW.PCH.kINPNIntakeReverse);
        mThingy.set(DoubleSolenoid.Value.kForward);
        db.intake.set(PNEUMATIC_STATE, 1.0);
    }
    public void modeInit(EMatchMode mode) {

    }
    public void readInputs() {
        db.intake.set(EIntakeData.ROLLER_PCT, mMotor.getMotorOutputPercent());
        db.intake.set(ROLLER_VEL_ft_s, mMotor.getSelectedSensorVelocity() * IntakeModule.kFeetSpeedConversion);

    }
    public void setOutputs() {
        setRollerState();
        idk();
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
                db.intake.set(ROLLER_STATE, 0.2);
                break;
            case VELOCITY:
                mMotor.set(TalonFXControlMode.Velocity, db.intake.get(EIntakeData.SET_ROLLER_VEL_ft_s));
                db.intake.set(ROLLER_STATE, 0.2);
                break;
        }
    }
    public void idk() {
        Enums.EArmState mode = db.intake.get(PNEUMATIC_STATE, Enums.EArmState.class);
        if (mode == null) {
            return;
        }
        switch (mode) {
            case DEFAULT:
            case EXTEND:
                mThingy.set(DoubleSolenoid.Value.kReverse);
                db.intake.set(PNEUMATIC_STATE, 1.0);
                break;
            case RETRACT:
                mThingy.set(DoubleSolenoid.Value.kForward);
                db.intake.set(PNEUMATIC_STATE, 2.0);
                break;
        }
    }
}
