package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EClimberData;
import us.ilite.common.types.EIntakeData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.HardwareUtils;

import static us.ilite.common.types.EClimberData.*;
import static us.ilite.common.types.EClimberData.IS_SINGLE_CLAMPED;

public class PositionControl extends Module {
    private TalonFX mPositionMotor;
    private PIDController mPositionPID;
    private PIDController mVelocityPID;
    private NetworkTable mTable;

    // ========================================
    // PID CONSTANTS /!\ DO NOT MODIFY /!\
    // ========================================

    private final int POSITION_SLOT = 0;
    private ProfileGains kVelocityGains = new ProfileGains().p(0.0).f(0.0001);
    private ProfileGains kPositionGains = new ProfileGains().p(0.035).f(0.0010).slot(POSITION_SLOT);

    // ========================================
    // PHYSICAL CONTANTS /!\ DO NOT MODIFY /!\
    // ========================================
    public static final double kClimberRatio = (12.0 / 72.0) * (20.0 / 80.0) * (20.0 / 80.0) * (16.0 / 42.0);
    public static final double kMaxClimberSpeed = 6380 * kClimberRatio;
    public static final double kScaledUnitsToRPM = (600.0 / 2048.0) * kClimberRatio;

    public PositionControl() {
        mPositionMotor = new TalonFX(12);
        mPositionPID = new PIDController(kPositionGains, -kMaxClimberSpeed, kMaxClimberSpeed, Settings.kControlLoopPeriod);
        mVelocityPID = new PIDController(kVelocityGains,-kMaxClimberSpeed,kMaxClimberSpeed,Settings.kControlLoopPeriod);
        mTable = NetworkTableInstance.getDefault().getTable("pidposition");
    }

    @Override
    public void modeInit(EMatchMode mode) {
        if (mode == EMatchMode.TELEOPERATED) {
            db.climber.set(ACTUAL_POSITION_deg, ticksToClimberDegrees(mPositionMotor.getSelectedSensorPosition()));
        }
    }
    @Override
    public void readInputs() {
        db.climber.set(ACTUAL_POSITION_deg,ticksToClimberDegrees(mPositionMotor.getSelectedSensorPosition()));
        db.climber.set(ACTUAL_CLIMBER_PCT, (mPositionMotor.getSelectedSensorVelocity() * kScaledUnitsToRPM) / (6380 * kClimberRatio));

    }

    @Override
    public void setOutputs() {
        Enums.EClimberMode mode = db.climber.get(EClimberData.HANGER_STATE, Enums.EClimberMode.class);
        if (mode == null) return;

        if (db.climber.isSet(EClimberData.SET_COAST)) {
            mPositionMotor.setNeutralMode(NeutralMode.Coast);
        } else {
            mPositionMotor.setNeutralMode(NeutralMode.Brake);
        }

        switch(mode) {
            case PERCENT_OUTPUT:
                mPositionMotor.set(ControlMode.PercentOutput, db.climber.get(EClimberData.DESIRED_VEL_pct));
                break;
            case VELOCITY:
                double desiredVel = mVelocityPID.calculate(db.climber.get(EClimberData.ACTUAL_VEL_rpm), clock.getCurrentTimeInMillis());
                mPositionMotor.set(ControlMode.Velocity, desiredVel);
                break;
            case POSITION:
                double desiredPos = mPositionPID.calculate(db.climber.get(DESIRED_POS_deg), clock.getCurrentTimeInMillis());
                mPositionMotor.set(ControlMode.Position, desiredPos);
                break;
        }
        mTable.getEntry("PositionPIDHelpMePlease").setNumber(db.climber.get(DESIRED_POS_deg));
    }
    private double ticksToClimberDegrees(double pTicks) {
        return pTicks / 2048 * kClimberRatio * 360;
    }
    private double climberDegreesToTicks(double pDegrees) {
        return pDegrees * 2048 / kClimberRatio / 360;
    }
}
