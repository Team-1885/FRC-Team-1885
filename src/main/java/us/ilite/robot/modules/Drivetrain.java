package us.ilite.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import us.ilite.common.config.Settings;

public class Drivetrain extends Module{

    // ================================================
    // DECLARES NEO MOTORS FOR THE DRIVETRAIN PLS WORK
    // ================================================
    private CANSparkMax mFrontLeft;
    private CANSparkMax mFrontRight;
    private CANSparkMax mBackLeft;
    private CANSparkMax mBackRight;

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

    public Drivetrain() {
        mFrontLeft.restoreFactoryDefaults();
        mBackLeft.restoreFactoryDefaults();
        mFrontRight.restoreFactoryDefaults();
        mBackRight.restoreFactoryDefaults();

        mFrontLeft = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        mFrontRight = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        mBackLeft = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        mBackRight = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftEncoder = mFrontLeft.getEncoder();
        rightEncoder = mFrontRight.getEncoder();

        leftControllerGroup = new MotorControllerGroup(mFrontLeft, mBackLeft);
        rightControllerGroup = new MotorControllerGroup(mFrontRight, mBackRight);

        differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        mBackLeft.follow(mFrontLeft);
        mBackRight.follow(mFrontRight);
    }

}
