package us.ilite.robot.modules;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.Data;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.sensor.EGyro;
import us.ilite.robot.Robot;



import static us.ilite.common.types.drive.EDriveData.L_DESIRED_VEL_FT_s;
import static us.ilite.common.types.drive.EDriveData.R_DESIRED_VEL_FT_s;

public class WpilibPigeonModule extends Module
{
    // ----- field variables ----- //
    private WPI_PigeonIMU gyro;
    private Data db;
    private NetworkTable mTable;

    private double desiredYaw;
    private double desiredPitch;
    private double desiredRoll;
    //private double desiredHeading; // do i need?

    // ----- Make WpilibPigeonModule a Singleton ----- //
    private WpilibPigeonModule instance;
    private WpilibPigeonModule() {
        gyro = new WPI_PigeonIMU(30); // initializes and zeros gyro
        db = Robot.DATA;
        mTable = NetworkTableInstance.getDefault().getTable("angle"); // glass
    }
    public WpilibPigeonModule getInstance(int deviceNumber)
    {
        return instance;
    }
    // ----- End Singleton ----- //

    /**
     * Create a Pigeon object that communicates with Pigeon on CAN Bus.
     *
     * @param deviceNumber
     *            CAN Device Id of Pigeon [0,62]
     * @param version Version of the PigeonIMU
     * @param canbus Name of the CANbus; can be a SocketCAN interface (on Linux),
     *               or a CANivore device name or serial number
     */

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
        db.drivetrain
    }

    // Auto Balance
    public void autoBalance()
    {
        /* TODO If holding a button, move by the pitch scaled to a number
        If pitch is 30 move forward, if pitch is 15 move forawrd a little less. If pitch is -30 move backward,
        if pitch is -15 move backward a little less.
        do these while a button is pressed and while the pitch is not within a range near 0 degrees.
         */
        // TODO implement this wpilibpigeonclass all throughout code (there is one in neodrive) use singleton
    }

    protected void setOutputs() // use values from codex to do actions
    {
        if (db.imu.get(EGyro.YAW_DEGREES) != desiredYaw)
        {
            // correct problem
        }
        if (db.imu.get(EGyro.PITCH_DEGREES) != desiredPitch)
        {
            // correct problem
        }
        if (db.imu.get(EGyro.YAW_DEGREES) != desiredRoll)
        {
            // correct problem
        }
        // I do not believe we need to set heading, that is handled automatically in WPI_PigeonIMU
    }
    protected void readInputs() // take values from encoders and log into codex
    {
        // set actual values in codex
        db.imu.set(EGyro.YAW_DEGREES, gyro.getYaw());
        db.imu.set(EGyro.PITCH_DEGREES, gyro.getPitch());
        db.imu.set(EGyro.ROLL_DEGREES, gyro.getRoll());
        // TODO what values does do codex want? FUSED HEADING? ABSOLUTE HEADING? WHAT RANGE DOES IT WANT [0, 360)
        db.imu.set(EGyro.HEADING_DEGREES, gyro.getAbsoluteCompassHeading()); // set heading to direction over [0, 360) degrees
    }
    // in methods, check desired values in codex

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
