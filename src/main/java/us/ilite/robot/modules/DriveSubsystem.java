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

@Deprecated
public class DriveSubsystem extends SubsystemBase  {
    //private db = Robot.DATA;
    private DifferentialDrive mDifferentialDrive;

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        //TO-DO: make sure the motors are being set to these voltage values (in path following ramsete case)
        Robot.DATA.drivetrain.set(EDriveData.LEFT_VOLTAGE, leftVolts);
        Robot.DATA.drivetrain.set(EDriveData.RIGHT_VOLTAGE, rightVolts);
        Robot.DATA.drivetrain.set(EDriveData.FEED, 1);
    }
//    private DifferentialDrive m_drive = new DifferentialDrive(mLeftMotors, mRightMotors);
//    //auton methods:
//    public void tankDriveVolts(double leftVolts, double rightVolts) {
//        mLeftMotors.setVoltage(leftVolts);
//        mRightMotors.setVoltage(rightVolts);
//        m_drive.feed();
//    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double leftMetersPerSecond = Units.feet_to_meters((Robot.DATA.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s)));
        double rightMetersPerSecond = Units.feet_to_meters((Robot.DATA.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s)));
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }
}
