package us.ilite.robot.modules;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.EMatchMode;

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
    public void modeInit(EMatchMode pMode) {

    }
    @Override
    public void setOutputs() {

    }
    @Override
    public void readInputs() {
        //test
    }
}
