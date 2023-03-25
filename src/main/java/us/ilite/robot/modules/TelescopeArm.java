package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EArmData;
import us.ilite.common.types.EMatchMode;
import us.ilite.robot.Enums;


public class TelescopeArm extends Module {

    // ============================================================
    // DECLARES MOTOR, TO BE CHANGED WHEN WE KNOW WHAT MOTOR IT IS
    // ============================================================

    private TalonFX mLevel;
    private TalonFX mMoveArm;

    // ==============================================
    // DECLARES PIDCONTROLLERS POSITION
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
    public static final double kArmRatio = (12.0 / 72.0) * (20.0 / 80.0) * (20.0 / 80.0) * (16.0 / 42.0);
    public static final double kMaxArmSpeed = 6380 * kArmRatio;
    public static final double kScaledUnitsToRPM = (600.0 / 2048.0) * kArmRatio;

    public TelescopeArm() {
        // ===================
        // SETTING MOTOR ID'S
        // ===================

        mLevel = new TalonFX(15);
        mMoveArm = new TalonFX(16);

        // =========================================================
        // CONSTRUCTING PID CONTROLLER (POSITION)
        // =========================================================

        mPositionPID = new PIDController(kPositionGains, -kMaxArmSpeed, kMaxArmSpeed, Settings.kControlLoopPeriod);

        // ========================================================
        //  CREATES A TABLE FOR GLASS, CAN BE USED TO TROUBLESHOOT
        // ========================================================

        mTable = NetworkTableInstance.getDefault().getTable("armposition");
    }
    @Override
    public void modeInit(EMatchMode mode) {
        // =================================================
        // SETS THE ACTUAL POSITION DEGREES TO SECOND THING
        // =================================================

        if (mode == EMatchMode.TELEOPERATED) {
            db.arm.set(EArmData.ACTUAL_LEVEL_POS_deg, ticksToClimberDegrees(mLevel.getSelectedSensorPosition()));
        }
    }
    @Override
    public void readInputs() {
        // ==============================================
        //  HONESTLY I HAVE NO IDEA WHAT THIS STUFF DOES
        // ==============================================
        db.arm.set(EArmData.ACTUAL_ARM_POSITION_deg, ticksToClimberDegrees(mMoveArm.getSelectedSensorPosition()));
        db.arm.set(EArmData.ACTUAL_LEVEL_POS_deg,ticksToClimberDegrees(mLevel.getSelectedSensorPosition()));
        db.arm.set(EArmData.ACTUAL_PERCENT_OUTPUT, mLevel.getMotorOutputPercent());
    }
    @Override
    public void setOutputs() {
        // ==========================================================================================
        // SETS VARIABLE MODE TO HANGER STATE FROM ECLIMBERMODE CLASS, IDK WHAT THE OTHER STUFF DOES
        // ==========================================================================================

        Enums.EArmMode mode = db.arm.get(EArmData.ARM_STATE, Enums.EArmMode.class);
        if (mode == null) return;

        // ===========================================================================================================================================================================
        // SWITCH STATEMENT: PERCENT OUTPUT SETS BOTH MOTORS TO DESIRED VEL PCT, VELOCITY SETS BOTH MOTORS TO PID VALUE DESIREDVEL, POSITION SETS BOTH MOTORS TO PID VALUE DESIREDPOS
        // ===========================================================================================================================================================================

        switch(mode) {
            case LEVEL_POSITION:
                mLevel.set(ControlMode.PercentOutput,climberDegreesToTicks(db.arm.get(EArmData.DESIRED_PERCENT_OUTPUT)));
                mTable.getEntry("DESIRED_PERCENT_OUTPUT").setNumber(db.arm.get(EArmData.DESIRED_PERCENT_OUTPUT));
                mTable.getEntry("climberDegreesToTicks").setNumber(climberDegreesToTicks(db.arm.get(EArmData.DESIRED_LEVEL_POS_deg)));
                break;
            case ARM_POSITION:
                double desiredPos = mPositionPID.calculate(db.arm.get(EArmData.DESIRED_LEVEL_POS_deg), clock.getCurrentTimeInMillis());
                mMoveArm.set(ControlMode.Position, climberDegreesToTicks(desiredPos));
                mTable.getEntry("ArmPos").setNumber(mLevel.getSelectedSensorPosition());
                break;
        }
    }
    private double ticksToClimberDegrees(double pTicks) {

        return pTicks / 2048 * kArmRatio * 360;
    }
    private double climberDegreesToTicks(double pDegrees)
    {
        return pDegrees * 2048 / kArmRatio / 360;
    }
}