package us.ilite.robot.modules;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import us.ilite.common.Data;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.drive.EDriveData;
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

    Mode mCurrentMode = None;
    enum Mode
    {
        None,
        Turning,
        StraigtDriving,
        AutoBalancing
    }

    // ----- Make WpilibPigeonModule a Singleton ----- //
    private static final WpilibPigeonModule instance = new WpilibPigeonModule();
    public static WpilibPigeonModule getInstance() { return instance; }
    private WpilibPigeonModule() {
        gyro = new WPI_PigeonIMU(30); // initializes and zeros gyro
        db = Robot.DATA;
        mTable = NetworkTableInstance.getDefault().getTable("angle"); // glass
    }
    // ----- End Singleton ----- //


    public void stayAtYaw(double pDesiredYaw)
    {
        mDesiredYaw = pDesiredYaw;

        mCurrentMode = Mode.StraigtDriving;
    }
    public void correctYaw()
    {
        // Get the angle between the actual direction and desired direction of the robot's front
        // If the angle is positive, the robot is facing too far left
        // So, the robot needs to turn right
        // If the angle is negative, the robot is facing too far right
        // So, the robot needs to turn left
        // The scale of the angle should affect how fast the robot corrects its direction
        // A larger angle would result in a fast correction
        // take the previous left and right speeds, and adjust them by the scale calculated to ensure the robot points in the correct direction
        // based on the difference between the target velocities, figure out if one side needs to be sped up, slowed down, or one side each
        // i think its better to slow down one side because a double an get infinitely smaller without going below zero, but
        // there would be extra calculations needed to side up one side, especially if the velocities are already at the maximum

        // get the values the velocity is looking to be set to
        double desiredLeftVelocity = db.drivetrain.get(EDriveData.L_DESIRED_VEL_FT_s);
        double desiredRightVelocity = db.drivetrain.get(EDriveData.R_DESIRED_VEL_FT_s);

        // get the current speed of the wheels so we can do calculations around them
        double leftCurrentVelocity = db.drivetrain.get(EDriveData.L_ACTUAL_VEL_FT_s);
        double rightCurrentVelocity = db.drivetrain.get(EDriveData.R_ACTUAL_VEL_FT_s);

        double angleError = db.imu.get(EGyro.YAW_DEGREES) - desiredAngle;
        double factorDecrease = (1 / angleError); // bigger angle of error, bigger decrease

        // correct direction
        if (angleError > 0) // too left, turn right
        {
            desiredLeftVelocity = leftCurrentVelocity;
            desiredRightVelocity = rightCurrentVelocity*factorDecrease;
        }
        else if (angleError < 0) // too right, turn left
        {
            desiredLeftVelocity = leftCurrentVelocity*factorDecrease;
            desiredRightVelocity = rightCurrentVelocity;
        }
        db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, desiredLeftVelocity);
        db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, desiredRightVelocity);
    }

    public void turnByDegrees(double pDesiredDegrees)
    {
        mDesiredDegrees = pDesiredDegrees;

        mCurrentMode = Mode.Turning;
    }

    public void turnToDegrees(double pDesieredDegrees)
    {
        mDesiredDegrees = db.imu.get(EGyro.YAW_DEGREES) + pDesiredDegrees; // new angle will be current plus amount to turn

        mCurrentMode = Mode.Turning;
    }

    private void updateActualDegreesToDesiredDegrees()
    {
        // Turn in place in the direction with the smallest angle

        // calculate a number in [-1, 1]
        // positive angles will accelerate the robot forward, and negative backward
        // the bigger the angle, the faster the turn

        double leftMotorVelocity;
        double rightMotorVelocity;

        double angleError = db.imu.get(EGyro.YAW_DEGREES) - mDesiredDegrees;
        double factorDecrease = (1 / angleError); // bigger angle of error, bigger decrease

        if (db.imu.get(angleError > 180) // it's faster to turn left
        {
            // left motor speed < right motor speed
            leftMotorVelocity = -Math.abs(angleError)/360; // divides by 360 to scale the speed between 0 and 1
            rightMotorVelocity = Math.abs(angleError)/360; // divides by 360 to scale the speed between 0 and 1
        }
        	else if (angleError < 180) // it's faster to turn right
    {
        // left motor speed > right motor speed
        leftMotorVelocity = Math.abs(db.imu.get(angleError)/360; // divides by 360 to scale the speed between 0 and 1
        rightMotorVelocity = -Math.abs(db.imu.get(angleError)/360; // divides by 360 to scale the speed between 0 and 1
    }
    else // keep the velocities the same
    {
        leftMotorVelocity = db.drivetrain.get(EDriveData.L_DESIRED_VEL_FT_s);
        rightMotorVelocity = db.drivetrain.get(EDriveData.R_DESIRED_VEL_FT_s);
    }

        // set new velocities to turn the robot in the most efficient direction
        db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, leftMotorVelocity);
        db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, rightMotorVelocity);
    }

    public void autoBalance()
    {
        /* TODO If holding a button, move by the pitch scaled to a number
        If pitch is 30 move forward, if pitch is 15 move forawrd a little less. If pitch is -30 move backward,
        if pitch is -15 move backward a little less.
        do these while a button is pressed and while the pitch is not within a range near 0 degrees.
         */

        // set a range to try to get the yaw in. You will probably never get it at exactly zero degrees.
        double errorRange = 0.1;
        double errorCenterPoint = 0;
        double errorLowBound = errorCenterPoint - (errorRange / 2);
        double errorHighBound = errorCenterPoint + (errorRange / 2);

        // get current left and right velocities
        //double currentLeftVelocity = db.drivetrain.get(EDriveData.L_DESIRED_VEL_FT_s);
        //double currentRightVelocity = db.drivetrain.get(EDriveData.R_DESIRED_VEL_FT_s);

        double currentPitch = db.imu.get(EGyro.ROOL_DEGREES); // db.imu.get(EGyro.PITCH_DEGREES); pitch and roll are switched
        if (currentPitch < errorLowBound)
        {
            // move robot backwards by a factor of angle between pitch and horizontal
            double factor = -1*Math.abs(currentPitch - errorLowBound); // corrects faster if there is a larger angle

            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, factor/180);
            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, factor/180);

        }
        else if (currentPitch > errorHighBound)
        {
            // move robot forwards by a factor of angle between pitch and horizontal
            double factor = Math.abs(currentPitch - errorHighBound); // corrects faster if there is a larger angle

            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, factor/180);
            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, factor/180);
        }
        else // when within error range
        {
            // stop robot
            db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, 0);
            db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, 0);
        }


        // TODO implement this wpilibpigeonclass all throughout code (there is one in neodrive) use singleton
    }

    protected void setOutputs() // use values from codex to do actions
    {
        if (mCurrentMode == Mode.Turning)
        {
            updateActualDegreesToDesiredDegrees();
        }
        else if (mCurrentMode == Mode.StraightDriving)
        {
            correctYaw();
        }
        else if (mCurrentMode == Mode.AutoBalancing)
        {
            autoBalance();
        }
        // otherwise the mode is currently set to None

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













//package us.ilite.robot.modules;
//
//import com.ctre.phoenix.sensors.WPI_PigeonIMU;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import us.ilite.common.Data;
//import us.ilite.common.config.InputMap;
//import us.ilite.common.types.drive.EDriveData;
//import us.ilite.common.types.sensor.EGyro;
//import us.ilite.robot.Robot;
//
//
//
//import static us.ilite.common.types.drive.EDriveData.L_DESIRED_VEL_FT_s;
//import static us.ilite.common.types.drive.EDriveData.R_DESIRED_VEL_FT_s;
//
//public class WpilibPigeonModule extends Module
//{
//    // ----- field variables ----- //
//    private WPI_PigeonIMU gyro;
//    private Data db;
//    private NetworkTable mTable;
//
//    private double desiredYaw;
//    private double desiredPitch;
//    private double desiredRoll;
//    //private double desiredHeading; // do i need?
//
//    // ----- Make WpilibPigeonModule a Singleton ----- //
//    private static final WpilibPigeonModule instance = new WpilibPigeonModule();
//    public static WpilibPigeonModule getInstance() { return instance; }
//    private WpilibPigeonModule() {
//        gyro = new WPI_PigeonIMU(30); // initializes and zeros gyro
//        db = Robot.DATA;
//        mTable = NetworkTableInstance.getDefault().getTable("angle"); // glass
//    }
//    // ----- End Singleton ----- //
//
//    // drive straight, robot should re-align with given angle when bumped
//    // TODO figure out how to keep checking the angle in setOutputs or readInputs and keep setting the velocities based
//    // TODO on the desired action, turnByDegree, turnToDegree, driveStraightForward. driveStraightForward would keep
//    // TODO going until turned off. turnByDegree and turnToDegree will keep going until the desired angle is reached.
//    public void driveStraightForward(double desiredAngle)
//    {
//        desiredYaw = desiredAngle;
//
//        double leftMotorVelocity;
//        double rightMotorVelocity;
//
//        // keep the robot going in a straight line forward
//        if (db.imu.get(EGyro.YAW_DEGREES) < desiredYaw) // needs to curve left to align robot with angle
//        {
//            // left motor speed < right motor speed
//
//            // how fast do the left motors need to spin to bring the robot back into a straight line without being jerky
//            // the greater difference between the Desired Angle and Actual Angle, the faster the robot will correct
//            // (the faster the left motors will spin)
//            leftMotorVelocity = (desiredYaw - db.imu.get(EGyro.YAW_DEGREES))/360; // divides by 360 to scale the speed between 0 and 1
//            // keep right velocity same as before
//            rightMotorVelocity = (db.drivetrain.get(EDriveData.R_DESIRED_VEL_FT_s));
//        }
//        else if (db.imu.get(EGyro.YAW_DEGREES) > desiredYaw) // needs to curve right to align robot with angle
//        {
//            // left motor speed > right motor speed
//
//            // keep right velocity same as before
//            leftMotorVelocity = (db.drivetrain.get(EDriveData.L_DESIRED_VEL_FT_s));
//            // how fast do the right motors need to spin to bring the robot back into a straight line without being jerky
//            // the greater difference between the Actual Angle and Desired Angle, the faster the robot will correct
//            // (the faster the right motors will spin)
//            rightMotorVelocity = (db.imu.get(EGyro.YAW_DEGREES) - desiredYaw)/360; // divides by 360 to scale the speed between 0 and 1
//        }
//        else // keep the velocities the same
//        {
//            leftMotorVelocity = db.drivetrain.get(EDriveData.L_DESIRED_VEL_FT_s);
//            rightMotorVelocity = db.drivetrain.get(EDriveData.R_DESIRED_VEL_FT_s);
//        }
//
//        // set new velocities to keep robot moving forward in a straight line
//        db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, leftMotorVelocity);
//        db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, rightMotorVelocity);
//    }
//
//    // Turn in place in the direction with the smallest angle
//    public void turnByDegreesEfficient(double desiredDegrees)
//    {
//        // calculate a number in [-1, 1]
//        // positive angles will accelerate the robot forward, and negative backward
//        // the bigger the angle, the faster the turn
//
//        desiredYaw = db.imu.get(EGyro.YAW_DEGREES) + desiredDegrees; // new angle will be current plus amount to turn
//
//        double leftMotorVelocity;
//        double rightMotorVelocity;
//
//        if (db.imu.get(EGyro.YAW_DEGREES) - desiredYaw > 180) // it's faster to turn left
//        {
//            // left motor speed < right motor speed
//            leftMotorVelocity = -Math.abs(db.imu.get(EGyro.YAW_DEGREES) - desiredYaw)/360; // divides by 360 to scale the speed between 0 and 1
//            rightMotorVelocity = Math.abs(db.imu.get(EGyro.YAW_DEGREES) - desiredYaw)/360; // divides by 360 to scale the speed between 0 and 1
//        }
//        else if (db.imu.get(EGyro.YAW_DEGREES) - desiredYaw < 180) // it's faster to turn right
//        {
//            // left motor speed > right motor speed
//            leftMotorVelocity = Math.abs(db.imu.get(EGyro.YAW_DEGREES) - desiredYaw)/360; // divides by 360 to scale the speed between 0 and 1
//            rightMotorVelocity = -Math.abs(db.imu.get(EGyro.YAW_DEGREES) - desiredYaw)/360; // divides by 360 to scale the speed between 0 and 1
//        }
//        else // keep the velocities the same
//        {
//            leftMotorVelocity = db.drivetrain.get(EDriveData.L_DESIRED_VEL_FT_s);
//            rightMotorVelocity = db.drivetrain.get(EDriveData.R_DESIRED_VEL_FT_s);
//        }
//
//        // set new velocities to turn the robot in the most efficient direction
//        db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, leftMotorVelocity);
//        db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, rightMotorVelocity);
//    }
//
//    // Turn to the specific degrees
//    public void turnToDegreesEfficient(double desiredDegrees)
//    {
//        // calculate a number in [-1, 1]
//        // positive angles will accelerate the robot forward, and negative backward
//        // the bigger the angle, the faster the turn
//
//        desiredYaw = desiredDegrees;
//
//        double leftMotorVelocity;
//        double rightMotorVelocity;
//
//        if (db.imu.get(EGyro.YAW_DEGREES) - desiredYaw > 180) // it's faster to turn left
//        {
//            // left motor speed < right motor speed
//            leftMotorVelocity = -Math.abs(db.imu.get(EGyro.YAW_DEGREES) - desiredYaw)/360; // divides by 360 to scale the speed between 0 and 1
//            rightMotorVelocity = Math.abs(db.imu.get(EGyro.YAW_DEGREES) - desiredYaw)/360; // divides by 360 to scale the speed between 0 and 1
//        }
//        else if (db.imu.get(EGyro.YAW_DEGREES) - desiredYaw < 180) // it's faster to turn right
//        {
//            // left motor speed > right motor speed
//            leftMotorVelocity = Math.abs(db.imu.get(EGyro.YAW_DEGREES) - desiredYaw)/360; // divides by 360 to scale the speed between 0 and 1
//            rightMotorVelocity = -Math.abs(db.imu.get(EGyro.YAW_DEGREES) - desiredYaw)/360; // divides by 360 to scale the speed between 0 and 1
//        }
//        else // keep the velocities the same
//        {
//            leftMotorVelocity = db.drivetrain.get(EDriveData.L_DESIRED_VEL_FT_s);
//            rightMotorVelocity = db.drivetrain.get(EDriveData.R_DESIRED_VEL_FT_s);
//        }
//
//        // set new velocities to turn the robot in the most efficient direction
//        db.drivetrain.set(EDriveData.L_DESIRED_VEL_FT_s, leftMotorVelocity);
//        db.drivetrain.set(EDriveData.R_DESIRED_VEL_FT_s, rightMotorVelocity);
//    }
//
//    // Turn degrees clockwise
//    public void turnClockwiseDegrees(double desiredDegrees)
//    {
//        // save original yaw degrees value
//        // set motors to turn until the difference between the original degrees and the current degrees is equal to desiredDegrees
//    }
//    // Turn degrees counter clockwise
//    public void turnCounterClockwiseDegrees(double desiredDegrees)
//    {
//        // save original yaw degrees value
//        // set motors to turn until the difference between the original degrees and the current degrees is equal to desiredDegrees
//    }
//    // Face toward the given degree
//    public void turnToDegrees(double desiredYawValueInDegrees)
//    {
//        db.drivetrain
//    }
//
//    // Auto Balance
//    public void autoBalance()
//    {
//        /* TODO If holding a button, move by the pitch scaled to a number
//        If pitch is 30 move forward, if pitch is 15 move forawrd a little less. If pitch is -30 move backward,
//        if pitch is -15 move backward a little less.
//        do these while a button is pressed and while the pitch is not within a range near 0 degrees.
//         */
//        // TODO implement this wpilibpigeonclass all throughout code (there is one in neodrive) use singleton
//    }
//
//    protected void setOutputs() // use values from codex to do actions
//    {
//        if (db.imu.get(EGyro.YAW_DEGREES) != desiredYaw)
//        {
//            // should the robot turn left or right
////            if (180 - desiredYaw > 0) // turn left
////            {
////
////            }
////            else // turn right
////            {
////
////            }
//
//
//        }
//        if (db.imu.get(EGyro.PITCH_DEGREES) != desiredPitch)
//        {
//            // correct problem
//        }
//        if (db.imu.get(EGyro.ROLL_DEGREES) != desiredRoll)
//        {
//            // correct problem
//        }
//        // I do not believe we need to set heading, that is handled automatically in WPI_PigeonIMU
//    }
//    protected void readInputs() // take values from encoders and log into codex
//    {
//        // set actual values in codex
//        db.imu.set(EGyro.YAW_DEGREES, gyro.getYaw());
//        db.imu.set(EGyro.PITCH_DEGREES, gyro.getPitch());
//        db.imu.set(EGyro.ROLL_DEGREES, gyro.getRoll());
//        // TODO what values does do codex want? FUSED HEADING? ABSOLUTE HEADING? WHAT RANGE DOES IT WANT [0, 360)
//        db.imu.set(EGyro.HEADING_DEGREES, gyro.getAbsoluteCompassHeading()); // set heading to direction over [0, 360) degrees
//    }
//    // in methods, check desired values in codex
//
//}
//
//
//
//// in teleop controller
//// while B and Y are pressed
////        if (db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_ROLLERS) && db.operatorinput.isSet(InputMap.OPERATOR.REVERSE_FEEDER)) {
////                mTable.getEntry("test").setNumber(gyro.getAngle());
////                // spin while the angle turned is less than 90 degress
////                if (gyro.getAngle() >= 90)
////                {
////
////                Robot.DATA.drivetrain.set(R_DESIRED_VEL_FT_s, 0);
////                Robot.DATA.drivetrain.set(L_DESIRED_VEL_FT_s, 0);
////                }
////                else
////                {
////                Robot.DATA.drivetrain.set(R_DESIRED_VEL_FT_s, 0.5);
////                Robot.DATA.drivetrain.set(L_DESIRED_VEL_FT_s, 0.5);
////                }
////                }
