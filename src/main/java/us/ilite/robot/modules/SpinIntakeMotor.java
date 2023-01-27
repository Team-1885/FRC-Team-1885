package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.Settings;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EIntakeData.*;
import static us.ilite.common.types.drive.EDriveData.STATE;

public class SpinIntakeMotor extends Module {
    private TalonFX mTalonFX;
    private DoubleSolenoid mThingy;
    private final NetworkTable mTable;
    public SpinIntakeMotor() {
        mTalonFX = new TalonFX(9);
        mThingy = new DoubleSolenoid(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH, Settings.HW.PCH.kINPNIntakeForward, Settings.HW.PCH.kINPNIntakeReverse);
        mThingy.set(DoubleSolenoid.Value.kForward);
        db.intake.set(PNEUMATIC_STATE, 1.0);
        mTable = NetworkTableInstance.getDefault().getTable("intake");
    }
    @Override
    public void modeInit(EMatchMode pMode) {

    }

    @Override
    public void readInputs() {
        db.intake.set(ROLLER_PCT, (mTalonFX.getSelectedSensorVelocity() * IntakeModule.kScaledUnitsToRPM) / IntakeModule.kMaxFalconSpeed);
        db.intake.set(ROLLER_VEL_ft_s, mTalonFX.getSelectedSensorVelocity() * IntakeModule.kFeetSpeedConversion);
    }
    @Override
    public void setOutputs() {
        setRollerState();
        idk();
        mTable.getEntry("Roller_Pct").setNumber(db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
        mTable.getEntry("Velocity").setNumber(db.intake.get(SET_ROLLER_VEL_ft_s));
        mTable.getEntry("Position").setNumber(db.intake.get(PNEUMATIC_STATE));
    }
    public void setRollerState() {
        Enums.ERollerState state = db.intake.get(ROLLER_STATE,Enums.ERollerState.class);
        if (state == null) {
            //state = Enums.ERollerState.PERCENT_OUTPUT;
            return;
        }
        switch(state) {
            case PERCENT_OUTPUT:
                System.out.println("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
                mTalonFX.set(TalonFXControlMode.PercentOutput, db.intake.get(EIntakeData.DESIRED_ROLLER_pct));
                //db.intake.set(ROLLER_STATE,0.2);
                break;
            case VELOCITY:
                System.out.println("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
                mTalonFX.set(TalonFXControlMode.Velocity, db.intake.get(EIntakeData.SET_ROLLER_VEL_ft_s));
                //db.intake.set(ROLLER_STATE,0.2);
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
