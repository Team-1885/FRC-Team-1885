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

    // =======================================================
    // DECLARES THREE FIELD VARIABLES FOR PID: kP, kI, and kD
    // =======================================================

    private double kP;
    private double kI;
    private double kD;

    // =============================================
    // DECLARES TALONFX MOTORS. WITH ID'S 11 AND 12
    // =============================================
    private TalonFX mMotorID12;
    private TalonFX mMotorID11;

    // ==============================================
    // DECLARES PIDCONTROLLERS POSITION AND VELOCITY
    // ==============================================
    private final PIDController mPositionPID;
    private final PIDController mVelocityPID;

    // ============================================================
    // DECLARES NETWORK TABLE TO BE USED FOR GLASS TROUBLESHOOTING
    // ============================================================
    private NetworkTable mTable;

    // ====================================
    // PID CONSTANTS /!\ DO NOT MODIFY /!\
    // ====================================

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
        // ===================
        // SETTING MOTOR ID'S
        // ===================

        mMotorID11 = new TalonFX(11);
        mMotorID12 = new TalonFX(12);

        // ================================================================================================================================
        // CONSTRUCTING TWO PID CONTROLLERS (POSITION AND VELOCITY), SETTING INPUT AND OUTPUT RANGE TO #'S, AND SETTING CONTINUOUS TO TRUE
        // ================================================================================================================================

        mVelocityPID = new PIDController(kVelocityGains, -kMaxClimberSpeed, kMaxClimberSpeed, Settings.kControlLoopPeriod);
        mPositionPID = new PIDController(kPositionGains, -kMaxClimberSpeed, kMaxClimberSpeed, Settings.kControlLoopPeriod);

        mPositionPID.setInputRange(-180.0f,180.0f);
        mPositionPID.setOutputRange(-90,90);
        mPositionPID.setContinuous(true);

        // ========================================================
        //  CREATES A TABLE FOR GLASS, CAN BE USED TO TROUBLESHOOT
        // ========================================================

        mTable = NetworkTableInstance.getDefault().getTable("pidposition");
    }

    @Override
    public void modeInit(EMatchMode mode) {

        // ===================
        // IDK WHAT THIS DOES
        // ===================

        if (mode == EMatchMode.TELEOPERATED) {
            db.climber.set(ACTUAL_POSITION_deg, ticksToClimberDegrees(mMotorID12.getSelectedSensorPosition()));
        }
    }
    @Override
    public void readInputs() {
        // ===================================================
        //  HONESTLY I HAVE NO IDEA WHAT THIS STUFF DOES BRUH
        // ===================================================
        db.climber.set(ACTUAL_VEL_rpm, mMotorID12.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.climber.set(ACTUAL_POSITION_deg,ticksToClimberDegrees(mMotorID12.getSelectedSensorPosition()));
        db.climber.set(ACTUAL_CLIMBER_PCT, (mMotorID12.getSelectedSensorVelocity() * kScaledUnitsToRPM) / (6380 * kClimberRatio));

        db.climber.set(ACTUAL_POSITION_deg,ticksToClimberDegrees(mMotorID11.getSelectedSensorPosition()));
        db.climber.set(ACTUAL_CLIMBER_PCT, (mMotorID11.getSelectedSensorVelocity() * kScaledUnitsToRPM) / (6380 * kClimberRatio));

    }

    @Override
    public void setOutputs() {
        // ==========================================================================================
        // SETS VARIABLE MODE TO HANGER STATE FROM ECLIMBERMODE CLASS, IDK WHAT THE OTHER STUFF DOES
        // ==========================================================================================

        Enums.EClimberMode mode = db.climber.get(EClimberData.HANGER_STATE, Enums.EClimberMode.class);
        if (mode == null) return;

        if (db.climber.isSet(EClimberData.SET_COAST)) {
            mMotorID12.setNeutralMode(NeutralMode.Coast);
            mMotorID11.setNeutralMode(NeutralMode.Coast);
        } else {
            mMotorID12.setNeutralMode(NeutralMode.Brake);
            mMotorID11.setNeutralMode(NeutralMode.Brake);
        }

        // ===========================================================================================================================================================================
        // SWITCH STATEMENT: PERCENT OUTPUT SETS BOTH MOTORS TO DESIRED VEL PCT, VELOCITY SETS BOTH MOTORS TO PID VALUE DESIREDVEL, POSITION SETS BOTH MOTORS TO PID VALUE DESIREDPOS
        // ===========================================================================================================================================================================

        switch(mode) {
            case PERCENT_OUTPUT:
                mMotorID12.set(ControlMode.PercentOutput, db.climber.get(EClimberData.DESIRED_VEL_pct));
                mMotorID11.set(ControlMode.PercentOutput, db.climber.get(EClimberData.DESIRED_VEL_pct));
                break;
            case VELOCITY:
                double desiredVel = mVelocityPID.calculate(db.climber.get(EClimberData.ACTUAL_VEL_rpm), clock.getCurrentTimeInMillis());
                mMotorID12.set(ControlMode.Velocity, desiredVel);
                mMotorID11.set(ControlMode.Velocity, desiredVel);
                break;
            case POSITION:
                double desiredPos = mPositionPID.calculate(db.climber.get(ACTUAL_POSITION_deg), clock.getCurrentTimeInMillis());
                mMotorID12.set(ControlMode.Position, desiredPos);
                mMotorID11.set(ControlMode.Position, desiredPos);
                break;
        }

        // ====================================================================================================
        // CREATES TWO TABLE ENTRIES FOR GLASS, ID11POS AND ID12POS, SETTING THEM TO GETSELECTEDSENSORPOSITION
        // ====================================================================================================

        mTable.getEntry("ID11pos").setNumber(mMotorID11.getSelectedSensorPosition());
        mTable.getEntry("ID12pos").setNumber(mMotorID12.getSelectedSensorPosition());
    }

    // =========================
    // AGAIN IDK WHAT THIS DOES
    // =========================

    private double ticksToClimberDegrees(double pTicks) {
        return pTicks / 2048 * kClimberRatio * 360;
    }
    private double climberDegreesToTicks(double pDegrees) {
        return pDegrees * 2048 / kClimberRatio / 360;
    }
}
