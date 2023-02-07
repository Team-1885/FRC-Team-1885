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
import org.apache.commons.math3.ml.clustering.Cluster;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EClimberData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;
import us.ilite.robot.controller.TeleopController;
import us.ilite.robot.hardware.DigitalBeamSensor;
import us.ilite.robot.hardware.HardwareUtils;

import static us.ilite.common.types.EClimberData.*;
import static us.ilite.common.types.EClimberData.IS_SINGLE_CLAMPED;

public class positionControl extends Module {
    private final TalonFX mCL12;
    private final TalonFX mCLMR11;

    private PIDController mVelocityPID;
    private PIDController mPositionPID;
    private ProfileGains kPositionGains = new ProfileGains().p(0.035).f(0.0010).slot(0);

    private ProfileGains kVelocityGains = new ProfileGains().p(0.0).f(0.0001);
    public positionControl() {
        mCLMR11 = new TalonFX(11);
        mCL12 = new TalonFX(12);
        mCLMR11.setNeutralMode(NeutralMode.Brake);
        mCL12.setNeutralMode(NeutralMode.Brake);
        mCL12.configClosedloopRamp(0.5);
        mCLMR11.configClosedloopRamp(0.5);

        //Add 10 ms for current limit
        mCL12.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 21, 0.01));
        mCLMR11.configGetSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 21, 0.01));


        mCL12.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);
        mCLMR11.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0, 20);

        mCLMR11.configAllowableClosedloopError(1, climberDegreesToTicks(2));
        mCL12.configAllowableClosedloopError(1, climberDegreesToTicks(2));

        mVelocityPID = new PIDController(kVelocityGains, -ClimberModule.kMaxClimberSpeed, ClimberModule.kMaxClimberSpeed, Settings.kControlLoopPeriod);

        mCL12.configNominalOutputForward(0, 20);
        mCLMR11.configNominalOutputForward(0, 20);

        mCL12.configNominalOutputReverse(0, 20);
        mCLMR11.configNominalOutputReverse(0, 20);

        mCL12.configPeakOutputForward(1, 20);
        mCLMR11.configPeakOutputForward(1, 20);

        mCL12.configPeakOutputReverse(-1, 20);
        mCLMR11.configPeakOutputReverse(-1, 20);

        mCL12.configMotionAcceleration(10 / ClimberModule.kScaledUnitsToRPM, 20);
        mCLMR11.configMotionAcceleration(10 / ClimberModule.kScaledUnitsToRPM, 20);

        HardwareUtils.setGains(mCL12, kPositionGains);
        HardwareUtils.setGains(mCLMR11, kPositionGains);

    }
    private double ticksToClimberDegrees(double pTicks) {
        return pTicks / 2048 * ClimberModule.kClimberRatio * 360;
    }


    public void modeInit(EMatchMode mode) {
        if (mode == EMatchMode.TELEOPERATED) {
            mCLMR11.configClearPositionOnQuadIdx(true, 20);
            db.climber.set(ACTUAL_POSITION_deg, ticksToClimberDegrees(mCL12.getSelectedSensorPosition()));
        }
    }
    public void readInputs() {
        db.climber.set(ACTUAL_VEL_rpm, mCL12.getSelectedSensorVelocity() * ClimberModule.kScaledUnitsToRPM);
        db.climber.set(ACTUAL_POSITION_deg, ticksToClimberDegrees(mCL12.getSelectedSensorPosition()));
        db.climber.set(ACTUAL_POSITION_TARGET, ticksToClimberDegrees(mCL12.getClosedLoopTarget()));
        db.climber.set(ACTUAL_POSITION_ERROR, ticksToClimberDegrees(mCL12.getClosedLoopError()));
        db.climber.set(ACTUAL_OUTPUT_CURRENT_12, mCL12.getStatorCurrent());
        db.climber.set(ACTUAL_OUTPUT_CURRENT_11, mCLMR11.getStatorCurrent());
        db.climber.set(ACTUAL_BUS_VOLTAGE, mCL12.getMotorOutputVoltage());
        db.climber.set(ACTUAL_CLIMBER_PCT, (mCL12.getSelectedSensorVelocity() * ClimberModule.kScaledUnitsToRPM) / (6380 * ClimberModule.kClimberRatio));
    }
    public void setOutputs() {
        Enums.EClimberMode mode = db.climber.get(EClimberData.HANGER_STATE, Enums.EClimberMode.class);
        if (mode == null) return;

        if (db.climber.isSet(EClimberData.SET_COAST)) {
            mCL12.setNeutralMode(NeutralMode.Coast);
            mCLMR11.setNeutralMode(NeutralMode.Coast);
        } else {
            mCL12.setNeutralMode(NeutralMode.Brake);
            mCLMR11.setNeutralMode(NeutralMode.Brake);
        }

        switch(mode) {
            case PERCENT_OUTPUT:
                mCL12.set(ControlMode.PercentOutput, db.climber.get(EClimberData.DESIRED_VEL_pct));
                mCLMR11.set(ControlMode.PercentOutput, db.climber.get(EClimberData.DESIRED_VEL_pct));
                break;
            case VELOCITY:
                double desiredVel = mVelocityPID.calculate(db.climber.get(EClimberData.ACTUAL_VEL_rpm), clock.getCurrentTimeInMillis());
                mCL12.set(ControlMode.Velocity, desiredVel);
                mCLMR11.set(ControlMode.Velocity, desiredVel);
                break;
            case POSITION:

                mCL12.set(ControlMode.Position, climberDegreesToTicks(db.climber.get(DESIRED_POS_deg)));
                mCLMR11.set(ControlMode.Position, climberDegreesToTicks(db.climber.get(DESIRED_POS_deg)));
                break;
        }




    }
    private double climberDegreesToTicks(double pDegrees) {
        return pDegrees * 2048 / ClimberModule.kClimberRatio / 360;
    }



   
}