package us.ilite.robot.modules;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.ilite.common.lib.util.Units;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Robot;


public class DriveSubsystem extends SubsystemBase  {
    private MotorControllerGroup mLeftMotors;
    private MotorControllerGroup mRightMotors;

    public DriveSubsystem(MotorControllerGroup p_leftMotors, MotorControllerGroup p_rightMotors) {
        mLeftMotors = p_leftMotors;
        mRightMotors = p_rightMotors;
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        NeoDriveModule.(leftVolts, rightVolts);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double leftMetersPerSecond = Units.feet_to_meters((Robot.DATA.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s)));
        double rightMetersPerSecond = Units.feet_to_meters((Robot.DATA.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s)));
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }
}
