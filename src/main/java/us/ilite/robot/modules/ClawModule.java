package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import us.ilite.common.config.Settings;
import us.ilite.common.types.sensor.EClawData;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EClimberData.DESIRED_POS_deg;
import static us.ilite.common.types.EIntakeData.*;

public class ClawModule extends Module {
    private TalonFX mIntakeRoller;
    private TalonFX mActuateIntake;
    private Compressor mCompressor;
    private boolean mIntakeState = false;
    private NetworkTable mTable;

    // ========================================
    // DO NOT MODIFY THESE CONSTANTS
    // ========================================
    public static final double kIntakeRollerRatio = (1.0 / 4.0) * (24.0 / 32.0);
    public static final double kMaxFalconSpeed = 6380 * kIntakeRollerRatio;
    public static final double kWheelDiameter = 2.0 / 12.0;
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kScaledUnitsToRPM = (600.0 / 2048.0) * kIntakeRollerRatio;
    public static final double kFeetSpeedConversion = (kScaledUnitsToRPM * kWheelCircumference) / 60.0;

    // ========================================
    // PHYSICAL CONTANTS /!\ DO NOT MODIFY /!\
    // ========================================
    public static final double kActuateRatio = (12.0 / 72.0) * (20.0 / 80.0) * (20.0 / 80.0) * (16.0 / 42.0);

    public ClawModule() {
        mIntakeRoller = new TalonFX(Settings.HW.CAN.kINRoller);
        mIntakeRoller.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 14, 15, 0.01));
        mIntakeRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        mIntakeRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        mIntakeRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
        mIntakeRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
        mIntakeRoller.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255);

        mActuateIntake = new TalonFX(0);


        mCompressor = new Compressor(Settings.HW.PCH.kPCHCompressorModule, PneumaticsModuleType.REVPH);
        mCompressor.enableAnalog(100, 110);


        mTable = NetworkTableInstance.getDefault().getTable("rollerintake");
    }

    @Override
    protected void readInputs() {
        db.claw.set(EClawData.ROLLER_VEL_ft_s, mIntakeRoller.getSelectedSensorVelocity() * kFeetSpeedConversion);
        db.claw.set(EClawData.ROLLER_STATE, (mIntakeRoller.getSelectedSensorVelocity() * kScaledUnitsToRPM) / kMaxFalconSpeed);
        db.claw.set(EClawData.CURRENT_ROLLER_RPM, mIntakeRoller.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.claw.set(EClawData.INTAKE_SUPPLY_CURRENT, mIntakeRoller.getSupplyCurrent());
        db.claw.set(EClawData.INTAKE_STATOR_CURRENT, mIntakeRoller.getSupplyCurrent());
        db.claw.set(EClawData.COMPRESSOR_PSI, mCompressor.getPressure());
    }

    @Override
    protected void setOutputs() {
        setPneumaticState();
        setRollerState();
        mTable.getEntry("PNEUMATIC_STATE").setNumber(db.claw.get(EClawData.PNEUMATIC_STATE));
        mTable.getEntry("DESIRED_VEL_ft_s").setNumber(db.claw.get(EClawData.DESIRED_VEL_ft_s));
    }

    public void setPneumaticState() {
        Enums.EClawMode mode = db.claw.get(EClawData.ARM_STATE, Enums.EClawMode.class);
        if (mode == null) {
            return;
        }
        switch (mode) {
            case DEFAULT:
            case EXTEND:
                //  #'s arent final
                mActuateIntake.set(ControlMode.Position, clawDegreesToTicks(180));
                db.claw.set(EClawData.ACTUATED_STATE, 1.0);
                break;
            case RETRACT:
                // #'s arent final
                mActuateIntake.set(ControlMode.Position, clawDegreesToTicks(45));
                db.claw.set(EClawData.ACTUATED_STATE, 2.0);
                break;
        }
    }

    public void setRollerState() {
        Enums.EClawMode mode = db.claw.get(EClawData.ROLLER_STATE, Enums.EClawMode.class);
        if (mode == null) {
            mode = Enums.EClawMode.PERCENT_OUTPUT;
        }
        switch (mode) {
            case PERCENT_OUTPUT:
                mIntakeRoller.set(TalonFXControlMode.PercentOutput, db.claw.get(EClawData.DESIRED_ROLLER_pct));
                break;
        }
    }
    private double ticksToClimberDegrees(double pTicks) {
        return pTicks / 2048 * kActuateRatio * 360;
    }
    private double clawDegreesToTicks(double pDegrees) {
        return pDegrees * 2048 / kActuateRatio / 360;
    }
}
