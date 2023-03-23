package us.ilite.robot.modules;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import us.ilite.common.config.Settings;
import us.ilite.common.lib.control.PIDController;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.ELimelightData;
import us.ilite.common.types.EMatchMode;
import us.ilite.common.types.EPixyData;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.TrajectoryCommandUtils;
import us.ilite.robot.hardware.*;

import static us.ilite.common.types.drive.EDriveData.*;

public class NeoDriveModule extends Module implements Subsystem {
    private CANSparkMax mRightMaster;
    private CANSparkMax mRightFollower;
    private CANSparkMax mLeftMaster;
    private CANSparkMax mLeftFollower;
    private RelativeEncoder mRightEncoder;
    private RelativeEncoder mLeftEncoder;
    private SparkMaxPIDController mRightCtrl;
    private SparkMaxPIDController mLeftCtrl;
    private PIDController mTurnToDegreePID;
    private PIDController mRightPositionPID;
    private PIDController mLeftPositionPID;
    private PIDController mTargetLockPID;
    private Pigeon mGyro;
    private DifferentialDriveKinematics mKinematics;
    private Pose2d mPose2d;
    private final NetworkTable mTable;
    private double angleSum;

    // ========================================
    // DO NOT MODIFY THESE PHYSICAL CONSTANTS
    // ========================================

    public static final double kGearboxRatio = (12.0 / 40.0) * (16.0 / 38.0);
    public static final double kWheelDiameterFeet = 0.5;
    public static final double kWheelDiameterInches = 6;
    public static final double kTrackWidthFeet = 1.8;

    public static final double kWheelCircumferenceFeet = kWheelDiameterFeet * Math.PI;
    public static final double kDriveNEOPositionFactor = kGearboxRatio * kWheelCircumferenceFeet;
    public static final double kDriveNEOVelocityFactor = kDriveNEOPositionFactor / 60.0;
    public static final double kMaxVelocityRPM = 5676;
    public static final double kPulsesPerRotation = 256.0;
    public static final double kCurrentLimitAmps = 60.0;
    public static final int kMaxLimelightFOV = 22;


    // ========================================
    // DO NOT MODIFY THESE PID CONSTANTS
    // ========================================
    private static final int VELOCITY_PID_SLOT = 1;
    private static final int SMART_MOTION_PID_SLOT = 2;
    //TODO change the smart motion gains once we have tuned it
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

    // ========================================
    // DO NOT MODIFY THESE OTHER CONSTANTS
    // ========================================
    public static double kTurnSensitivity = 0.85;
    private DifferentialDriveOdometry mOdometry;
    private static final NeoDriveModule instance = new NeoDriveModule(); // create singleton (only instance)

    public static NeoDriveModule getInstance() { // create method so anyone can get access of NeoDriveModule's only instance
        return instance;
    }

    public double mCurrentDeg;

    private NeoDriveModule() {
        angleSum = 0;
        mTable = NetworkTableInstance.getDefault().getTable("drivetrain");

        mLeftMaster = SparkMaxFactory.createDefaultSparkMax(Settings.HW.CAN.kDTML2);
        mLeftFollower = SparkMaxFactory.createDefaultSparkMax(Settings.HW.CAN.kDTL4);

        mRightMaster = SparkMaxFactory.createDefaultSparkMax(Settings.HW.CAN.kDTMR1);
        mRightFollower = SparkMaxFactory.createDefaultSparkMax(Settings.HW.CAN.kDTR3);

        
        mLeftFollower.follow(mLeftMaster);
        mRightFollower.follow(mRightMaster);
        mGyro = new Pigeon(Robot.CLOCK, Settings.HW.CAN.kDTGyro);

        mLeftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightMaster.setInverted(true);

        mRightMaster.setSmartCurrentLimit(65);
        mRightFollower.setSmartCurrentLimit(65);
        mLeftMaster.setSmartCurrentLimit(65);
        mLeftFollower.setSmartCurrentLimit(65);

        mRightMaster.setClosedLoopRampRate(0.5);
        mRightFollower.setClosedLoopRampRate(0.5);
        mLeftMaster.setClosedLoopRampRate(0.5);
        mLeftFollower.setClosedLoopRampRate(0.5);

        mRightEncoder = mRightMaster.getEncoder();
        mLeftEncoder = mLeftMaster.getEncoder();

        mRightCtrl = mRightMaster.getPIDController();
        mLeftCtrl = mLeftMaster.getPIDController();
        mRightCtrl.setOutputRange(-kMaxVelocityRPM, kMaxVelocityRPM);
        mLeftCtrl.setOutputRange(-kMaxVelocityRPM, kMaxVelocityRPM);

        mTurnToDegreePID = new PIDController(kTurnToProfileGains, -180, 180, Settings.kControlLoopPeriod);
        mTurnToDegreePID.setContinuous(true);
        mTurnToDegreePID.setOutputRange(-1, 1);

        mRightPositionPID = new PIDController(kPositionGains,0, 10, Settings.kControlLoopPeriod);
        mRightPositionPID.setOutputRange(-1 , 1);
        mLeftPositionPID = new PIDController(kPositionGains,0, 10, Settings.kControlLoopPeriod);
        mLeftPositionPID.setOutputRange(-1 , 1);

        mTargetLockPID = new PIDController(kTargetAngleLockGains, -180, 180, Settings.kControlLoopPeriod);
        mTargetLockPID.setOutputRange(Settings.kTargetAngleLockMinPower, Settings.kTargetAngleLockMaxPower);

        HardwareUtils.setGains(mLeftCtrl, kVelocityGains);
        HardwareUtils.setGains(mRightCtrl, kVelocityGains);
        HardwareUtils.setGains(mLeftCtrl, kSmartMotionGains);
        HardwareUtils.setGains(mRightCtrl, kSmartMotionGains);

        mOdometry = new DifferentialDriveOdometry(mGyro.getHeading(), //new Rotation2d(db.drivetrain.get(ACTUAL_HEADING_RADIANS))
                Units.feet_to_meters(db.drivetrain.get(L_ACTUAL_POS_FT)),
                Units.feet_to_meters(db.drivetrain.get(R_ACTUAL_POS_FT)));

        mLeftMaster.burnFlash();
        mLeftFollower.burnFlash();
        mRightMaster.burnFlash();
        mRightFollower.burnFlash();
        mGyro.zeroAll(); //moved from mode init
        mTable.getEntry("INIT HEADING DEG").setNumber(mGyro.getHeading().getDegrees());

    }
    @Override
    public void modeInit(EMatchMode pMode) {
        reset();
//        mCurrentDeg = 80;
        if(pMode == EMatchMode.AUTONOMOUS) {
//            resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0))); //commented out 2:19, updating odom with with a pose that is not the initial position of the ramsete command
//            mLeftMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
//            mRightMaster.setIdleMode(CANSparkMax.IdleMode.kBrake);
//            mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
//            mRightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
            mLeftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
            mRightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
            mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
            mRightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        } else {
            mLeftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
            mRightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
            mLeftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
            mRightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        }
    }
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        reset();
        mGyro.zeroAll();
//        angleSum = pose.getRotation().getDegrees();
//        mTable.getEntry("angle sum").setNumber(angleSum);
//        mGyro.resetAngle(pose.getRotation()); //potentially need to zero gyro instead of doing this
//        mOdometry.resetPosition(pose, Rotation2d.fromDegrees(-mGyro.getHeading().getRadians())); // was .degrees
        mOdometry.resetPosition(mGyro.getHeading(), //new Rotation2d(db.drivetrain.get(ACTUAL_HEADING_RADIANS))
                Units.feet_to_meters(db.drivetrain.get(L_ACTUAL_POS_FT)),
                Units.feet_to_meters(db.drivetrain.get(R_ACTUAL_POS_FT)),
                pose);
        mTable.getEntry("pose rotation").setString(pose.getRotation().toString());
//        mTable.getEntry("gyro heading").setString(pose.getRotation().toString());
    }
    @Override
    public void readInputs() {
        mGyro.update();
        db.drivetrain.set(ACTUAL_HEADING_RADIANS, -mGyro.getHeading().getRadians());
        db.drivetrain.set(ACTUAL_HEADING_DEGREES, -mGyro.getHeading().getDegrees());
        db.drivetrain.set(LEFT_VOLTAGE, mLeftMaster.getVoltageCompensationNominalVoltage());
        db.drivetrain.set(RIGHT_VOLTAGE, mRightMaster.getVoltageCompensationNominalVoltage());
        db.drivetrain.set(LEFT_CURRENT, mLeftMaster.getOutputCurrent());
        db.drivetrain.set(RIGHT_CURRENT, mRightMaster.getOutputCurrent());
        db.drivetrain.set(L_ACTUAL_POS_FT, mLeftEncoder.getPosition() * kDriveNEOPositionFactor);
        db.drivetrain.set(L_ACTUAL_VEL_FT_s, mLeftEncoder.getVelocity() * kDriveNEOVelocityFactor);
        db.drivetrain.set(R_ACTUAL_VEL_RPM, mRightEncoder.getVelocity() * kGearboxRatio);
        db.drivetrain.set(L_ACTUAL_VEL_RPM, mLeftEncoder.getVelocity() * kGearboxRatio);
        db.drivetrain.set(R_ACTUAL_POS_FT, mRightEncoder.getPosition() * kDriveNEOPositionFactor);
        db.drivetrain.set(R_ACTUAL_VEL_FT_s, mRightEncoder.getVelocity() * kDriveNEOVelocityFactor);
        db.imu.set(EGyro.ACCEL_X, mGyro.getAccelX());
        db.imu.set(EGyro.ACCEL_Y, mGyro.getAccelY());
        db.imu.set(EGyro.PITCH_DEGREES, mGyro.getPitch().getDegrees());
        db.imu.set(EGyro.ROLL_DEGREES, mGyro.getRoll().getDegrees());
        db.imu.set(EGyro.YAW_DEGREES, -mGyro.getYaw().getDegrees());
        db.imu.set(EGyro.YAW_OMEGA_DEGREES, -mGyro.getYawRate().getDegrees());
        db.drivetrain.set(X_ACTUAL_ODOMETRY_METERS, mOdometry.getPoseMeters().getX());
        db.drivetrain.set(Y_ACTuAL_ODOMETRY_METERS, mOdometry.getPoseMeters().getY());


//        Robot.FIELD.setRobotPose(mOdometry.getPoseMeters());
    }

    @Override
    public void setOutputs() {
        mPose2d = mOdometry.update(mGyro.getHeading(), //new Rotation2d(db.drivetrain.get(ACTUAL_HEADING_RADIANS))
                Units.feet_to_meters(db.drivetrain.get(L_ACTUAL_POS_FT)),
                Units.feet_to_meters(db.drivetrain.get(R_ACTUAL_POS_FT)));
        angleSum += mGyro.getHeading().getDegrees();
        mTable.getEntry("angleSum").setNumber(angleSum);

        mTable.getEntry("CURRENT HEADING DEG").setNumber(mGyro.getHeading().getDegrees());
        Enums.EDriveState state = db.drivetrain.get(STATE, Enums.EDriveState.class);
        double throttle = db.drivetrain.safeGet(DESIRED_THROTTLE_PCT, 0.0);
        double turn = db.drivetrain.safeGet(DESIRED_TURN_PCT, 0.0);
        double left = throttle + turn;
        double right = throttle - turn;

//        mTable.getEntry("left").setNumber(left);
//        mTable.getEntry("right").setNumber(right);
//
//        mTable.getEntry("left m_s").setNumber(Units.feet_to_meters(db.drivetrain.get(L_ACTUAL_VEL_FT_s)));
//        mTable.getEntry("right m_s").setNumber(Units.feet_to_meters(db.drivetrain.get(R_ACTUAL_VEL_FT_s)));
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
//                resetOdometry(new Pose2d(x, y, new Rotation2d(-mGyro.getYaw().getRadians()))); //commented out 2/19: shouldn't matter, wasnt being called anywhere
                break;
            case PERCENT_OUTPUT:
                if (db.limelight.isSet(ELimelightData.TARGET_ID)) {
                    double targetLockOutput = 0;
                    if (db.limelight.isSet(ELimelightData.TV)) {
                        targetLockOutput = mTargetLockPID.calculate(-db.limelight.get(ELimelightData.TX), clock.dt());
                        turn = targetLockOutput;
                    }
//                    turn += 0.1 * Math.signum(turn);
//                    turn *= (1/(1-throttle)) * 0.5;
                }
                mLeftMaster.set(throttle+turn);
                mRightMaster.set(throttle-turn);
                break;
            case VELOCITY:
                mLeftCtrl.setReference(left * kMaxVelocityRPM, CANSparkMax.ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
                mRightCtrl.setReference(right * kMaxVelocityRPM, CANSparkMax.ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
                break;
//            case TURN_TO:
//                mTurnToDegreePID.setSetpoint(db.drivetrain.get(DESIRED_TURN_ANGLE_deg));
//                double turnOutput = mTurnToDegreePID.calculate(db.drivetrain.get(ACTUAL_HEADING_DEGREES), clock.getCurrentTimeInMillis());
//                db.drivetrain.set(DESIRED_TURN_PCT, turnOutput);
//                mLeftMaster.set(turnOutput);
//                mRightMaster.set(-turnOutput);
//                break;
//            case SMART_MOTION:
//                mLeftCtrl.setReference(db.drivetrain.get(L_DESIRED_POS_FT) / kDriveNEOPositionFactor,
//                        CANSparkMax.ControlType.kSmartMotion, SMART_MOTION_PID_SLOT, 0 );
//                mRightCtrl.setReference(db.drivetrain.get(R_DESIRED_POS_FT) / kDriveNEOPositionFactor,
//                        CANSparkMax.ControlType.kSmartMotion, SMART_MOTION_PID_SLOT, 0 );
//                break;
//            case PATH_FOLLOWING_BASIC:
//                mLeftPositionPID.setSetpoint(db.drivetrain.get(L_DESIRED_POS_FT));
//                mRightPositionPID.setSetpoint(db.drivetrain.get(R_DESIRED_POS_FT));
//                double lMeasurement = db.drivetrain.get(L_ACTUAL_POS_FT);
//                double rMeasurement = db.drivetrain.get(R_ACTUAL_POS_FT);
//                double leftPathOutput = mLeftPositionPID.calculate(lMeasurement, clock.getCurrentTimeInMillis());
//                double rightPathOutput = mRightPositionPID.calculate(rMeasurement, clock.getCurrentTimeInMillis());
//                mLeftMaster.set(leftPathOutput);
//                mRightMaster.set(rightPathOutput);
//                break;
//            case PATH_FOLLOWING_RAMSETE:
//                //Divide by 6.56 since that is the max velocity in ft/s
//                double vleft = db.drivetrain.get(L_DESIRED_VEL_FT_s);
//                double vright = db.drivetrain.get(R_DESIRED_VEL_FT_s);
////                mLeftMaster.set(db.drivetrain.get(L_DESIRED_VEL_FT_s) / Units.meters_to_feet(0.5));
////                mRightMaster.set(db.drivetrain.get(R_DESIRED_VEL_FT_s) / Units.meters_to_feet(0.5));
////                mRightFollower.set(db.drivetrain.get(R_DESIRED_VEL_FT_s) / Units.meters_to_feet(0.5));
////                mDrive.feed();
//                mLeftCtrl.setReference((vleft / kWheelCircumferenceFeet) * 60, CANSparkMax.ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
//                mRightCtrl.setReference((vright / kWheelCircumferenceFeet) * 60, CANSparkMax.ControlType.kVelocity, VELOCITY_PID_SLOT, 0);
//                break;
        }
    }

    public void setThrottlePct(double throttle) {
        throttle = throttle/3; //divided by 15 to limit accel
        mRightMaster.set(throttle);
        mLeftMaster.set(throttle);
        mTable.getEntry("THROTTLE PCT").setNumber(throttle);
    }

    public double getGyroRollDeg() {
        mTable.getEntry("ROLL DEG").setNumber(mGyro.getRoll().getDegrees());
        System.out.println("getting deg");
//        mCurrentDeg--;
//        return mCurrentDeg;
        return mGyro.getRoll().getDegrees();
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                Units.feet_to_meters(db.drivetrain.get(R_ACTUAL_VEL_FT_s)),
                Units.feet_to_meters(db.drivetrain.get(R_ACTUAL_VEL_FT_s))
        );
    }

    public void setVolts(double leftVolts, double rightVolts) {
        //safety check, only if the desired .set() value is less than one should it be set to the motors
        if (Math.abs(leftVolts/12) < 1 && Math.abs(rightVolts/12) < 1) {
            mRightMaster.set(rightVolts / 12);
            mLeftMaster.set(leftVolts / 12);
        }
        mTable.getEntry("left volts").setNumber(leftVolts/12);
        mTable.getEntry("right volts").setNumber(rightVolts/12);
//        mRightMaster.set(-rightVolts / 12);
//        mLeftMaster.set(-leftVolts / 12);
    }
    public void reset() {
        mLeftEncoder.setPosition(0.0);
        mRightEncoder.setPosition(0.0);
        mLeftMaster.set(0.0);
        mRightMaster.set(0.0);
    }


}
