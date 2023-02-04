package us.ilite.robot.commands;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Angle;
import us.ilite.common.Distance;
import us.ilite.common.lib.control.ProfileGains;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.hardware.ECommonNeutralMode;
import us.ilite.robot.hardware.SparkMaxFactory;
import us.ilite.robot.modules.NeoDriveModule;

import static us.ilite.common.types.drive.EDriveData.*;
import static us.ilite.common.types.drive.EDriveData.R_ACTUAL_POS_FT;

public class PigeonTest {
//    // only controller need an initialize with no params
//    //private Data db = Robot.DATA;
//    private NeoDriveModule mDrive;
//    private WPI_PigeonIMU gyro;
//    private CANSparkMax mLeftMaster;
//    private CANSparkMax mLeftFollower;
//    private CANSparkMax mRightMaster;
//    private CANSparkMax mRightFollower;
//    //private MotorControllerGroup mLeftMotorControllerGroup;
//    //private MotorControllerGroup mRightMotorControllerGroup;
//    private DifferentialDrive mDifferentialDrive;
//    //Robot.DATA.imu.get(EGyro.HEADING_DEGREES);
//
//    public PigeonTest() {
//        gyro = new WPI_PigeonIMU(30); // Pigeon is on CAN Bus with device ID 30
//        mDrive = NeoDriveModule.getInstance();
//
//        // pair motors with CAN ids
//        mLeftMaster = SparkMaxFactory.createDefaultSparkMax(1); // LeftMaster is on CAN Bus with device ID 1
//        mLeftFollower = SparkMaxFactory.createDefaultSparkMax(3); // LeftFollower is on CAN Bus with device ID 3
//        mRightMaster = SparkMaxFactory.createDefaultSparkMax(2); // RightMaster is on CAN Bus with device ID 2
//        mRightFollower = SparkMaxFactory.createDefaultSparkMax(4); // RightFollower is on CAN Bus with device ID 4
//        // put left motors in a group
//        //mLeftMotorControllerGroup = new MotorControllerGroup();
//        // put right motors in a group
//        // put left and right motors in a differential drive
//    }
//
//    public static void go()
//    {
//        // spin motors
//        Robot.DATA.drivetrain.set(R_DESIRED_VEL_FT_s, 0.5);
//        Robot.DATA.drivetrain.set(L_DESIRED_VEL_FT_s, 0.5);
//        // stop spinning motors when the angle has turned
//        turnLeft(90);
//    }
//
//    // turn robot left until it reaches the desired angle in degrees
//    public static void turnLeft(double desiredAngleInDegrees)
//    {
//        // when the angle has been reached, stop turning
//        if (gyro.getAngle() >= desiredAngleInDegrees)
//        {
//            Robot.DATA.drivetrain.set(R_DESIRED_VEL_FT_s, 0);
//            Robot.DATA.drivetrain.set(L_DESIRED_VEL_FT_s, 0);
//        }
//    }
//
//    // turn robot right until it reaches the desired angle in degrees
//    public void turnRight(double degrees)
//    {
//
//    }








//    @Override
//    public boolean update(double pNow) {
//
//
//        double turn = mHeadingController.calculate(getHeading().degrees());
////        // TODO - the units here are probably incorrect
//        double throttle = mDistanceController.calculate(getAverageDriveDistance().inches());
//        SmartDashboard.putNumber("Distance Error", mDistanceController.getPositionError());
////
//        SmartDashboard.putNumber("AUTON Turn Output", turn);
//        SmartDashboard.putNumber("AUTON Heading Degrees", Robot.DATA.drivetrain.get( ACTUAL_HEADING_DEGREES ));
//        SmartDashboard.putNumber("AUTON Drive Error", mDistanceController.getPositionError() );
//        if(mDistanceController.atSetpoint()) {
//            return true;
//        } else {
//            Robot.DATA.drivetrain.set(NEUTRAL_MODE, ECommonNeutralMode.BRAKE);
//            Robot.DATA.drivetrain.set(STATE, Enums.EDriveState.PERCENT_OUTPUT);
//            Robot.DATA.drivetrain.set(DESIRED_THROTTLE_PCT, throttle);
//            Robot.DATA.drivetrain.set(DESIRED_TURN_PCT, turn);
//            mLastTime = pNow;
//            return false;
//        }
//    }

//    protected Angle getHeading() {
//        return Angle.fromDegrees(Robot.DATA.drivetrain.get(ACTUAL_HEADING_DEGREES));
//    }
//
//    @Override
//    public void shutdown(double pNow) {
//
//    }
//
//    private Distance getAverageDriveDistance() {
//        return Distance.fromFeet(
//                (Robot.DATA.drivetrain.get(L_ACTUAL_POS_FT) +
//                        Robot.DATA.drivetrain.get(R_ACTUAL_POS_FT)) / 2.0);
//    }
//
//    private Distance getAverageDistanceTraveled() {
//        return Distance.fromInches(getAverageDriveDistance().inches() - mInitialDistance.inches());
//    }
//
//    public DriveStraight setDistanceToDrive(Distance pDistanceToDrive) {
//        mDistanceToDrive = pDistanceToDrive;
//        return this;
//    }
//
//    public DriveStraight setTargetHeading(Rotation2d pTargetHeading) {
//        mTargetHeading = pTargetHeading;
//        mHeadingController.setSetpoint(mTargetHeading.getDegrees());
//        return this;
//    }
//
//    public DriveStraight setHeadingGains(ProfileGains pHeadingControlGains) {
//        mHeadingController.setPID(pHeadingControlGains.P, pHeadingControlGains.I, pHeadingControlGains.D);
//        return this;
//    }
//
//    public DriveStraight setDrivePercentOutput(double pDrivePercentOutput) {
//        mDrivePercentOutput = pDrivePercentOutput;
//        return this;
//    }


}
