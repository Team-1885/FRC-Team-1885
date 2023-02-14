package us.ilite.robot.modules;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.Data;
import us.ilite.common.config.InputMap;
import us.ilite.robot.Robot;



import static us.ilite.common.types.drive.EDriveData.L_DESIRED_VEL_FT_s;
import static us.ilite.common.types.drive.EDriveData.R_DESIRED_VEL_FT_s;

public class WpilibPigeonModule extends Module
{
    private WPI_PigeonIMU gyro;
    private Data db;
    private NetworkTable mTable;

    public WpilibPigeonModule()
    {
        gyro = new WPI_PigeonIMU(30);
        db = Robot.DATA;
        mTable = NetworkTableInstance.getDefault().getTable("angle"); // glass
    }

    // Turn degrees clockwise
    public void turnClockwiseDegrees(double desiredDegrees)
    {
        // save original yaw degrees value
        // set motors to turn until the difference between the original degrees and the current degrees is equal to desiredDegrees
    }
    // Turn degrees counter clockwise
    public void turnCounterClockwiseDegrees(double desiredDegrees)
    {
        // save original yaw degrees value
        // set motors to turn until the difference between the original degrees and the current degrees is equal to desiredDegrees
    }
    // Face toward the given degree
    public void turnToDegrees(double desiredYawValueInDegrees)
    {

    }

}



// in teleop controller
// while B and Y are pressed
//        if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_ROLLERS) && db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_FEEDER)) {
//                mTable.getEntry("test").setNumber(gyro.getAngle());
//                // spin while the angle turned is less than 90 degress
//                if (gyro.getAngle() >= 90)
//                {
//
//                Robot.DATA.drivetrain.set(R_DESIRED_VEL_FT_s, 0);
//                Robot.DATA.drivetrain.set(L_DESIRED_VEL_FT_s, 0);
//                }
//                else
//                {
//                Robot.DATA.drivetrain.set(R_DESIRED_VEL_FT_s, 0.5);
//                Robot.DATA.drivetrain.set(L_DESIRED_VEL_FT_s, 0.5);
//                }
//                }
