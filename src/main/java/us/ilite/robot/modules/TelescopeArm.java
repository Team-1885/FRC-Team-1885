package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EClimberData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;

import static us.ilite.common.types.EClimberData.*;
import static us.ilite.common.types.EClimberData.ACTUAL_CLIMBER_PCT;

public class TelescopeArm extends Module{

    // ============================================================
    // DECLARES MOTOR, TO BE CHANGED WHEN WE KNOW WHAT MOTOR IT IS
    // ============================================================

    private TalonFX mMotorID;

    // ==============================================
    // DECLARES PIDCONTROLLERS POSITION AND VELOCITY
    // ==============================================

    private  PIDController mPositionPID;

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

    public TelescopeArm() {
        // ===================
        // SETTING MOTOR ID'S
        // ===================

        mMotorID = new TalonFX(0);

        // =========================================================
        // CONSTRUCTING TWO PID CONTROLLERS (POSITION AND VELOCITY)
        // =========================================================

        mPositionPID = new PIDController(kPositionGains, -kMaxClimberSpeed, kMaxClimberSpeed, Settings.kControlLoopPeriod);

        // ========================================================
        //  CREATES A TABLE FOR GLASS, CAN BE USED TO TROUBLESHOOT
        // ========================================================

        mTable = NetworkTableInstance.getDefault().getTable("pidposition");
    }
    @Override
    public void modeInit(EMatchMode mode) {
        // ===================
        // SETS THE ACTUAL POSITIN DEGREES TO MMOTORIDGETSELECTEDSENSORPOSITION
        // ===================

        if (mode == EMatchMode.TELEOPERATED) {
            db.climber.set(ACTUAL_POSITION_deg, ticksToClimberDegrees(mMotorID.getSelectedSensorPosition()));
        }
    }
    @Override
    public void readInputs() {
        // ==============================================
        //  HONESTLY I HAVE NO IDEA WHAT THIS STUFF DOES
        // ==============================================
        db.climber.set(ACTUAL_VEL_rpm, mMotorID.getSelectedSensorVelocity() * kScaledUnitsToRPM);
        db.climber.set(ACTUAL_POSITION_deg,ticksToClimberDegrees(mMotorID.getSelectedSensorPosition()));
        db.climber.set(ACTUAL_CLIMBER_PCT, (mMotorID.getSelectedSensorVelocity() * kScaledUnitsToRPM) / (6380 * kClimberRatio));
    }
    @Override
    public void setOutputs() {
        // ==========================================================================================
        // SETS VARIABLE MODE TO HANGER STATE FROM ECLIMBERMODE CLASS, IDK WHAT THE OTHER STUFF DOES
        // ==========================================================================================

        Enums.EClimberMode mode = db.climber.get(EClimberData.HANGER_STATE, Enums.EClimberMode.class);
        if (mode == null) return;

        if (db.climber.isSet(EClimberData.SET_COAST)) {
            mMotorID.setNeutralMode(NeutralMode.Coast);
        } else {
            mMotorID.setNeutralMode(NeutralMode.Brake);
        }

        // ===========================================================================================================================================================================
        // SWITCH STATEMENT: PERCENT OUTPUT SETS BOTH MOTORS TO DESIRED VEL PCT, VELOCITY SETS BOTH MOTORS TO PID VALUE DESIREDVEL, POSITION SETS BOTH MOTORS TO PID VALUE DESIREDPOS
        // ===========================================================================================================================================================================

        switch(mode) {

            case POSITION:
                System.out.print("ccccccccccccccccccccccc");
                double desiredPos = mPositionPID.calculate(db.climber.get(ACTUAL_POSITION_deg), clock.getCurrentTimeInMillis());
                mMotorID.set(ControlMode.Position, climberDegreesToTicks(db.climber.get(DESIRED_POS_deg)));
                //mMotorID11.set(ControlMode.PercentOutput,0.25);
                mTable.getEntry("ID11pos").setNumber(mMotorID.getSelectedSensorPosition());
                mTable.getEntry("DESIRED_POS_deg").setNumber(db.climber.get(DESIRED_POS_deg));
                mTable.getEntry("climberDegreesToTicks").setNumber(climberDegreesToTicks(db.climber.get(DESIRED_POS_deg)));
                break;
        }
    }
    private double ticksToClimberDegrees(double pTicks) {
        return pTicks / 2048 * kClimberRatio * 360;
    }
    private double climberDegreesToTicks(double pDegrees) {
        return pDegrees * 2048 / kClimberRatio / 360;
    }
}
