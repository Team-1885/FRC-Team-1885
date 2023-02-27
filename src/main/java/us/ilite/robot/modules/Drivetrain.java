package us.ilite.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
<<<<<<< HEAD
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Robot;
=======
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EPixyData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.ECommonNeutralMode;
import us.ilite.robot.hardware.HardwareUtils;
import us.ilite.robot.hardware.Pigeon;
import us.ilite.robot.hardware.SparkMaxFactory;

import static us.ilite.common.types.drive.EDriveData.*;
>>>>>>> Telescope-Arm

import static us.ilite.common.types.drive.EDriveData.*;

public class Drivetrain extends Module {

    // ================================================
    // DECLARES NEO MOTORS FOR THE DRIVETRAIN PLS WORK
    // ================================================
    private CANSparkMax mFrontLeft;
    private CANSparkMax mFrontRight;
    private CANSparkMax mBackLeft;
    private CANSparkMax mBackRight;

    // ========================================================================
    // DECLARES TWO VARIABLES FOR LEFT AND RIGHT SPARK MAX/NEO PID CONTROLLERS
    // ========================================================================
    private SparkMaxPIDController mLeftCtrl;
    private SparkMaxPIDController mRightCtrl;

    // ======================================================
    // DECLARES TWO ENCODERS: LEFT ENCODER AND RIGHT ENCODER
    // ======================================================
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    // ===========================================================
    // GROUPS MOTORS TOGETHER IN LEFT AND RIGHT CONTROLLER GROUPS
    // ===========================================================
    MotorControllerGroup leftControllerGroup;
    MotorControllerGroup rightControllerGroup;

    // ==================================
    // DECLARES DIFFERENTIAL DRIVE THING
    // ==================================
    DifferentialDrive differentialDrive;

    // ==========================================================================
    // DECLARES 4 PID CONTROLLERS FOR DIFFERENT SWITCH CASES LATER IN THE MODULE
    // ==========================================================================
    private PIDController mTurnToDegreePID;
    private PIDController mRightPositionPID;
    private PIDController mLeftPositionPID;
    private PIDController mTargetLockPID;

    private Pigeon mGyro;

    // =========================================================================
    // DO NOT MODIFY THESE PHYSICAL CONSTANTS (VALUES WILL BE CHANGED POSSIBLY)
    // =========================================================================
    public static final double kGearboxRatio = (12.0 / 40.0) * (18.0 / 36.0);
    public static final double kWheelDiameterFeet = 3.9 / 12.0;
    public static final double kWheelCircumferenceFeet = kWheelDiameterFeet * Math.PI;
    public static final double kDriveNEOPositionFactor = kGearboxRatio * kWheelCircumferenceFeet;
    public static final double kDriveNEOVelocityFactor = kDriveNEOPositionFactor / 60.0;
    public static final double kMaxVelocityRPM = 5676;
    public static final double kPulsesPerRotation = 256.0;
    public static final double kCurrentLimitAmps = 60.0;
    public static final double kTrackWidthFeet = 22.0 / 12.0;
    public static final int kMaxLimelightFOV = 22;


    // ====================================================================
    // DO NOT MODIFY THESE PID CONSTANTS (VALUES WILL BE CHANGED POSSIBLY)
    // ====================================================================
    private static final int VELOCITY_PID_SLOT = 1;
    private static final int SMART_MOTION_PID_SLOT = 2;
    public static ProfileGains kSmartMotionGains = new ProfileGains()
            .p(0.25)
            .f(0.00015)
            .maxVelocity(kMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
            .maxAccel(5676d)
            .slot(SMART_MOTION_PID_SLOT)
            .velocityConversion(kDriveNEOPositionFactor);
    public static ProfileGains kPositionGains = new ProfileGains()
            .maxVelocity(kMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier * 12)
            .maxAccel(31*12d) // just under 1g
            .p(0.0275)
            .tolerance(0.2);
    public static ProfileGains kVelocityGains = new ProfileGains()
            .f(0.00015)
            .p(0.00000025)
            .maxVelocity(kMaxVelocityRPM * Settings.Input.kMaxAllowedVelocityMultiplier)
            .slot(VELOCITY_PID_SLOT)
            .velocityConversion(kDriveNEOVelocityFactor);
    public static ProfileGains kTurnToProfileGains = new ProfileGains().p(0.02).f(0.1);
    //Old value is 0.0075
    public static ProfileGains kTargetAngleLockGains = new ProfileGains().p(0.01);

    // ======================================================================
    // DO NOT MODIFY THESE OTHER CONSTANTS (VALUES WILL BE CHANGED POSSIBLY)
    // ======================================================================
    public static double kTurnSensitivity = 0.85;
    private DifferentialDriveOdometry mOdometry;

    public Drivetrain() {

        // =====================================================================================================
        // CONSTRUCTS THE CANSparkMax MOTORS ESTABLISHED IN THE FIELD AS NEW CANSparkMax TYPE kBrushless MOTORS
        // =====================================================================================================

        mFrontLeft = SparkMaxFactory.createDefaultSparkMax(0);
        mBackLeft = SparkMaxFactory.createDefaultSparkMax(0);
        mFrontRight = SparkMaxFactory.createDefaultSparkMax(0);
        mBackRight = SparkMaxFactory.createDefaultSparkMax(0);

        // ====================================================================
        // MAKES BACK LEFT AND RIGHT MOTORS FOLLOW FRONT LEFT AND RIGHT MOTORS
        // ====================================================================

        mBackLeft.follow(mFrontLeft);
        mBackRight.follow(mFrontRight);

        // ===============================================================================
        // CONSTRUCTS A NEW PIGEON (AS THE GYRO) TO GIVE VALUES SUCH AS ANGLE, SPEED, ETC
        // ===============================================================================

        mGyro = new Pigeon(Robot.CLOCK, Settings.HW.CAN.kDTGyro);

        // ==============================================================================
        // SETS THE IDLE MODE OF THE MOTORS TO KCOAST SETS THE FRONT RIGHT MOTOR TO TRUE
        // ==============================================================================

        mFrontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mFrontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mBackLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mBackRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mFrontRight.setInverted(true);

        // =======================================================================================================
        // SETS THE SMART CURRENT LIMIT FOR ALL MOTORS TO 65 (CTRL+CLICK setSmartCurrentLimit TO GET MORE DETAIL)
        // =======================================================================================================

        mFrontRight.setSmartCurrentLimit(65);
        mBackRight.setSmartCurrentLimit(65);
        mFrontLeft.setSmartCurrentLimit(65);
        mBackLeft.setSmartCurrentLimit(65);

        // =========================================================================================================
        // DECLARES TWO ENCODERS: LEFT, AND RIGHT. THESE ARE SET TO THE ENCODERS OF THE FRONT LEFT AND RIGHT MOTORS
        // =========================================================================================================


        leftEncoder = mFrontLeft.getEncoder();
        rightEncoder = mFrontRight.getEncoder();

        // ==========================================
        // DECLARING LEFT AND RIGHT PID CONTROLLERS
        // ==========================================

        mRightCtrl = mFrontRight.getPIDController();
        mLeftCtrl = mFrontLeft.getPIDController();

        // =================================================================
        // SETS OUTPUT RANGE FOR RIGHT AND LEFT CONTROL USING MAX VELOCITY
        // =================================================================

        mRightCtrl.setOutputRange(-kMaxVelocityRPM, kMaxVelocityRPM);
        mLeftCtrl.setOutputRange(-kMaxVelocityRPM, kMaxVelocityRPM);

        // =======================================
        // SETS UP THE PID CONTROLLER FOR TURNING
        // =======================================

        mTurnToDegreePID = new PIDController(kTurnToProfileGains, -180, 180, Settings.kControlLoopPeriod);
        mTurnToDegreePID.setContinuous(true);
        mTurnToDegreePID.setOutputRange(-1, 1);

        // =========================================================
        // SETS UP POSITION PID FOR LEFT AND RIGHT AND OUTPUT RANGE
        // =========================================================

        mRightPositionPID = new PIDController(kPositionGains,0, 10, Settings.kControlLoopPeriod);
        mRightPositionPID.setOutputRange(-1 , 1);
        mLeftPositionPID = new PIDController(kPositionGains,0, 10, Settings.kControlLoopPeriod);
        mLeftPositionPID.setOutputRange(-1 , 1);

        // =====================================
        // SETS UP POSITION PID FOR TARGET LOCK
        // =====================================

        mTargetLockPID = new PIDController(kTargetAngleLockGains, -180, 180, Settings.kControlLoopPeriod);
        mTargetLockPID.setOutputRange(Settings.kTargetAngleLockMinPower, Settings.kTargetAngleLockMaxPower);

        // =========================================
        // SETS THE GAINS FOR LEFT AND RIGHT CONTROL
        // ==========================================

        HardwareUtils.setGains(mLeftCtrl, kVelocityGains);
        HardwareUtils.setGains(mRightCtrl, kVelocityGains);
        HardwareUtils.setGains(mLeftCtrl, kSmartMotionGains);
        HardwareUtils.setGains(mRightCtrl, kSmartMotionGains);

        // =================================
        // SETS THE ODOMETRY USING THE GYRO
        // =================================

        mOdometry = new DifferentialDriveOdometry(mGyro.getHeading());
        // ===========================
        // BURNS FLASH FOR ALL MOTORS
        // ===========================

        mFrontLeft.burnFlash();
        mBackLeft.burnFlash();
        mFrontRight.burnFlash();
        mBackRight.burnFlash();


        // =================================================================
        // GROUPS MOTORS TOGETHER IN PAIRS THE TWO PAIRS ARE LEFT AND RIGHT
        // =================================================================

        leftControllerGroup = new MotorControllerGroup(mFrontLeft, mBackLeft);
        rightControllerGroup = new MotorControllerGroup(mFrontRight, mBackRight);

        //====================================================================
        // MAKES NEW DIFFERENTIAL DRIVE USING LEFT AND RIGHT CONTROLLER GROUP
        //====================================================================


        differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);
    }
    @Override
    public void modeInit(EMatchMode pMode) {
        mGyro.zeroAll();
        reset();
        if(pMode == EMatchMode.AUTONOMOUS) {
            resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
            mFrontLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
            mFrontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
            mBackLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
            mBackRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
        } else {
            mFrontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
            mFrontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
            mBackLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
            mBackRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
    }

    public void resetOdometry(Pose2d pose) {
        reset();
        mGyro.resetAngle(pose.getRotation());
        mOdometry.resetPosition(pose, Rotation2d.fromDegrees(-mGyro.getHeading().getDegrees()));
    }

    @Override
    public void readInputs() {
        mGyro.update();
        db.drivetrain.set(ACTUAL_HEADING_RADIANS, -mGyro.getHeading().getRadians());
        db.drivetrain.set(ACTUAL_HEADING_DEGREES, -mGyro.getHeading().getDegrees());
        db.drivetrain.set(LEFT_VOLTAGE, mFrontLeft.getVoltageCompensationNominalVoltage());
        db.drivetrain.set(RIGHT_VOLTAGE, mFrontRight.getVoltageCompensationNominalVoltage());
        db.drivetrain.set(LEFT_CURRENT, mFrontLeft.getOutputCurrent());
        db.drivetrain.set(RIGHT_CURRENT, mFrontRight.getOutputCurrent());
        db.drivetrain.set(L_ACTUAL_POS_FT, leftEncoder.getPosition() * kDriveNEOPositionFactor);
        db.drivetrain.set(L_ACTUAL_VEL_FT_s, leftEncoder.getVelocity() * kDriveNEOVelocityFactor);
        db.drivetrain.set(R_ACTUAL_VEL_RPM, rightEncoder.getVelocity() * kGearboxRatio);
        db.drivetrain.set(L_ACTUAL_VEL_RPM, leftEncoder.getVelocity() * kGearboxRatio);
        db.drivetrain.set(R_ACTUAL_POS_FT, rightEncoder.getPosition() * kDriveNEOPositionFactor);
        db.drivetrain.set(R_ACTUAL_VEL_FT_s, rightEncoder.getVelocity() * kDriveNEOVelocityFactor);

        db.imu.set(EGyro.ACCEL_X, mGyro.getAccelX());
        db.imu.set(EGyro.ACCEL_Y, mGyro.getAccelY());
        db.imu.set(EGyro.PITCH_DEGREES, mGyro.getPitch().getDegrees());
        db.imu.set(EGyro.ROLL_DEGREES, mGyro.getRoll().getDegrees());
        db.imu.set(EGyro.YAW_DEGREES, -mGyro.getYaw().getDegrees());
        db.imu.set(EGyro.YAW_OMEGA_DEGREES, -mGyro.getYawRate().getDegrees());

        db.drivetrain.set(X_ACTUAL_ODOMETRY_METERS, mOdometry.getPoseMeters().getX());
        db.drivetrain.set(Y_ACTuAL_ODOMETRY_METERS, mOdometry.getPoseMeters().getY());
        mOdometry.update(new Rotation2d(db.drivetrain.get(ACTUAL_HEADING_RADIANS)),
                Units.feet_to_meters(db.drivetrain.get(L_ACTUAL_POS_FT)),
                Units.feet_to_meters( db.drivetrain.get(R_ACTUAL_POS_FT)));
        Robot.FIELD.setRobotPose(mOdometry.getPoseMeters());
    }
    @Override
    public void setOutputs() {
        Enums.EDriveState state = db.drivetrain.get(STATE, Enums.EDriveState.class);
        double throttle = db.drivetrain.safeGet(DESIRED_THROTTLE_PCT, 0.0);
        double turn = db.drivetrain.safeGet(DESIRED_TURN_PCT, 0.0);
        double left = throttle + turn;
        double right = throttle - turn;
        ECommonNeutralMode neutralMode = db.drivetrain.get(NEUTRAL_MODE, ECommonNeutralMode.class);
        if (state == null) return;
        switch (state) {
            case RESET:
                mGyro.zeroAll();
                reset();
                break;
            case RESET_ODOMETRY:
                double x = db.drivetrain.get(X_DESIRED_ODOMETRY_METERS);
                double y = db.drivetrain.get(X_DESIRED_ODOMETRY_METERS);
                mGyro.zeroAll();
                resetOdometry(new Pose2d(x, y, new Rotation2d(-mGyro.getYaw().getRadians())));
                break;
            case PERCENT_OUTPUT:
                if (db.limelight.isSet(ELimelightData.TARGET_ID)) {
                    double targetLockOutput = 0;
                    if (db.limelight.isSet(ELimelightData.TV)) {
                        targetLockOutput = mTargetLockPID.calculate(-db.limelight.get(ELimelightData.TX), clock.dt());
                        turn = targetLockOutput;
                    }
                    turn += 0.1 * Math.signum(turn);
                    turn *= (1/(1-throttle)) * 0.5;
                } else if (db.pixydata.isSet(EPixyData.SIGNATURE)) {
                    double targetLockOutput = 0;
                    if (db.pixydata.get(EPixyData.TARGET_VALID) == 1) {
                        targetLockOutput = mTargetLockPID.calculate(-db.pixydata.get(EPixyData.LARGEST_ANGLE_FROM_CAMERA), clock.dt());
                        turn = targetLockOutput;
                    }
                    turn += 0.1 * Math.signum(turn);
                    turn *= (1/(1-throttle)) * 0.5;
                }

                mFrontLeft.set(throttle+turn);
                mFrontRight.set(throttle-turn);
                break;
            case VELOCITY:
                mLeftCtrl.setReference(left * kMaxVelocityRPM, CANSparkMax.ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
                mRightCtrl.setReference(right * kMaxVelocityRPM, CANSparkMax.ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
                break;
            case TURN_TO:
                mTurnToDegreePID.setSetpoint(db.drivetrain.get(DESIRED_TURN_ANGLE_deg));
                double turnOutput = mTurnToDegreePID.calculate(db.drivetrain.get(ACTUAL_HEADING_DEGREES), clock.getCurrentTimeInMillis());
                db.drivetrain.set(DESIRED_TURN_PCT, turnOutput);
                mFrontLeft.set(turnOutput);
                mFrontRight.set(-turnOutput);
                break;
            case SMART_MOTION:
                mLeftCtrl.setReference(db.drivetrain.get(L_DESIRED_POS_FT) / kDriveNEOPositionFactor,
                        CANSparkMax.ControlType.kSmartMotion, SMART_MOTION_PID_SLOT, 0 );
                mRightCtrl.setReference(db.drivetrain.get(R_DESIRED_POS_FT) / kDriveNEOPositionFactor,
                        CANSparkMax.ControlType.kSmartMotion, SMART_MOTION_PID_SLOT, 0 );
                break;
            case PATH_FOLLOWING_BASIC:
                mLeftPositionPID.setSetpoint(db.drivetrain.get(L_DESIRED_POS_FT));
                mRightPositionPID.setSetpoint(db.drivetrain.get(R_DESIRED_POS_FT));
                double lMeasurement = db.drivetrain.get(L_ACTUAL_POS_FT);
                double rMeasurement = db.drivetrain.get(R_ACTUAL_POS_FT);
                double leftPathOutput = mLeftPositionPID.calculate(lMeasurement, clock.getCurrentTimeInMillis());
                double rightPathOutput = mRightPositionPID.calculate(rMeasurement, clock.getCurrentTimeInMillis());
                mFrontLeft.set(leftPathOutput);
                mFrontRight.set(rightPathOutput);
                break;
            case PATH_FOLLOWING_RAMSETE:
                double vleft = db.drivetrain.get(L_DESIRED_VEL_FT_s);
                double vright = db.drivetrain.get(R_DESIRED_VEL_FT_s);
                mLeftCtrl.setReference((vleft / kWheelCircumferenceFeet) * 60, CANSparkMax.ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
                mRightCtrl.setReference((vright / kWheelCircumferenceFeet) * 60, CANSparkMax.ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
                break;
        }

    }

    public void reset() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
        mFrontLeft.set(0.0);
        mFrontRight.set(0.0);
    }
    public void readInputs() {

    }

}
