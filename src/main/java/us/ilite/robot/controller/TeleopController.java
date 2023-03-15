package us.ilite.robot.controller;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import us.ilite.common.Field2022;
import us.ilite.common.config.InputMap;
import us.ilite.common.types.*;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.AutoBalance;
import us.ilite.robot.modules.NeoDriveModule;


public class TeleopController extends BaseManualController {

    private static TeleopController INSTANCE;
    private boolean mPressed = false, mPrevPressed = false;
    private Timer mClimbTimer;
    private Timer moveToTraversalTimer = new Timer();
    private NeoDriveModule mRobotDrive = NeoDriveModule.getInstance();
    private AutoBalance mAutoBalance = new AutoBalance(mRobotDrive, mRobotDrive.getGyroRollDeg());
    private NetworkTable mTable = NetworkTableInstance.getDefault().getTable("Target_Lock_Info");




    public static TeleopController getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new TeleopController();
        }
        return INSTANCE;
    }

    private TeleopController() {
        mClimbTimer = new Timer();
        mClimbTimer.reset();
        mClimbTimer.start();
    }

    @Override
    protected void updateImpl() {
        // ========================================
        // DO NOT COMMENT OUT THESE METHOD CALLS
        // ========================================
        super.updateDrivetrain();
//        updateCargo();
        super.updateBallCount();
        //Makes sure that we call the right methods for the climber mode
        if (Robot.CLIMB_MODE.equals("WCMP")) {
            if (db.operatorinput.isSet(ELogitech310.START)) {
//                updateHangerManual();
//                mLastRungState = Enums.ERungState.NULL;
            } else {
//                updateRungState();
            }
        } else {
            //Add in methods from DCMP
//            updateHangerMotors();
//            updateHangerPneumatics();
        }
//        updateAutoBalance();

//        updateIntake();
        updateTargetLock();

    }
    private void updateTargetLock() {
        if (Robot.mode() == EMatchMode.TELEOPERATED) {
            /**
             * Upon Specific Buttons Pressed, adjust the
             * limelight pipeline to target different objects
             *
             * NOTE: More tracking stuff in BaseManualController line 43
             *
             * To add a new pipeline, you need to add a Field Element with the dimensions and pipeline id
             * in the Field2022 class. You also need to add the button used for the new element
             * to an if statement in BaseManual Controller.
             *
             * We get the pipeline we want to track from the field element we want to track
             * Not from a random constant in Settings.java
             *
             * Make sure normal vision pipeline doesn't target anything to avoid weird bugs
             */
            // if the reflective tape button is pressed
            if (db.driverinput.isSet(ELogitech310.RIGHT_TRIGGER_AXIS)) {
                System.out.println("right trigger");
            }

            if (db.driverinput.isSet(InputMap.DRIVER.REFLECTIVE_TAPE_TRACKING)) {
                // adjust limelight pipeline so the robot targets reflective tape
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.REFLECTIVE_TAPE);
                // let target lock take over
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
                // set LED
                //db.ledcontrol.set(ELEDControlData.DESIRED_COLOR, 20);

                // log
                mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
                mTable.getEntry("Tracking Object").setString("Tracking Reflective Tape");

                db.addressableled.set(EAddressableLEDData.DESIREDCOLOR, Enums.EAddressableLEDState.RED);
            }
            // if the cone tracking button is pressed
            else if (db.driverinput.isSet(InputMap.DRIVER.CONE_TRACKING)) {
                // adjust limelight pipeline so the robot targets cones
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CONE);
                // let target lock take over
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);

                // log
                mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
                mTable.getEntry("Tracking Object").setString("Tracking Cones");
                db.addressableled.set(EAddressableLEDData.DESIREDCOLOR, Enums.EAddressableLEDState.YELLOW);
            }
            // if the cube tracking button is pressed
//            else if (db.driverinput.isSet(InputMap.DRIVER.CUBE_TRACKING)) {
//                // adjust limelight pipeline so the robot targets cubes
//                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CUBE);
//                // let target lock take over
//                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
//
//                // log
//                mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
//                mTable.getEntry("Tracking Object").setString("Tracking Cubes");
//            }
//            // track enemy robots' left corner
            else if (db.driverinput.isSet(InputMap.DRIVER.OPPONENT_ROBOT_LEFT_TRACKING)) {
                System.out.println("left tracking");
                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
                {
                    System.out.println("blue alliance");
                    db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.RED_ROBOT_LEFT/*red left corner field element*/);
                }
                else // if our team is red, target blue robots
                {
                    db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.BLUE_ROBOT_LEFT/*blue left corner field element*/);
                }
                // let target lock take over
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);

                // log
                mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
                mTable.getEntry("Tracking Object").setString("Tracking Cubes");
            }
            // track enemy robots' right
            else if (db.driverinput.isSet(InputMap.DRIVER.OPPONENT_ROBOT_RIGHT_TRACKING)) {
                // if our team is blue, target red robots
                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
                {
                    db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.RED_ROBOT_RIGHT/*red right corner field element*/);
                }
                else // if our team is red, target blue robots
                {
                    db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.BLUE_ROBOT_RIGHT/*blue right corner field element*/);
                }
                // let target lock take over
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);

                // log
                mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
                mTable.getEntry("Tracking Object").setString("Tracking Cubes");
            }
            else { // No targetting button is pressed
                // set limelight pipeline back to base camera without crazy filters
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CAMERA.id());
                // give turning control back to driver
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.VELOCITY);

                // log
                mTable.getEntry("Current Pipeline").setString("" + db.limelight.get(ELimelightData.PIPELINE));
                mTable.getEntry("Tracking Object").setString("Not Tracking");
                db.addressableled.set(EAddressableLEDData.DESIREDCOLOR, Enums.EAddressableLEDState.BATTLEFIElD_COLOR);
            }
        }
//        else // if the robot is not in teleoperated mode
//        {
//            if (db.driverinput.isSet(InputMap.DRIVER.TARGET_LOCK))
//            {
//                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
//                {
//                    db.pixydata.set(EPixyData.SIGNATURE, Pixy2CCC.CCC_SIG1);
//                } else {
//                    db.pixydata.set(EPixyData.SIGNATURE, Pixy2CCC.CCC_SIG2);
//                }
//                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
//            } else {
//                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CAMERA.id());
//                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.VELOCITY);
//            }
//        }
    }

}
