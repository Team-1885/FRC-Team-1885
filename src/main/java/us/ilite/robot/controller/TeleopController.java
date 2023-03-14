package us.ilite.robot.controller;

import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import us.ilite.common.Field2022;
import us.ilite.common.config.InputMap;
import us.ilite.common.config.Settings;
import us.ilite.common.types.*;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.common.types.input.ELogitech310;
import us.ilite.robot.Enums;
import us.ilite.robot.Robot;
import us.ilite.robot.commands.AutoBalance;
import us.ilite.robot.modules.NeoDriveModule;

import static us.ilite.common.types.EIntakeData.*;
import static us.ilite.common.types.EFeederData.*;


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
//    private void updateHangerMotors() {
//        db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
//
//        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
//            if (db.operatorinput.isSet(InputMap.HANGER.SPIN_SINGLE)) {
//                db.climber.set(EClimberData.DESIRED_VEL_pct, 0.45);
//            } else if (db.operatorinput.isSet(InputMap.HANGER.SPIN_DOUBLE)) {
//                db.climber.set(EClimberData.DESIRED_VEL_pct, -0.45);
//            } else if (db.driverinput.isSet(InputMap.DRIVER.MID_RUNG)) {
//                db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                db.climber.set(EClimberData.DESIRED_POS_deg, -90);
//            }
//            else if (db.operatorinput.isSet(InputMap.HANGER.HIGH_RUNG)) {
//                db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                db.climber.set(EClimberData.DESIRED_POS_deg, 90);
//            }
//            else if (db.operatorinput.isSet(InputMap.HANGER.TRAVERSAL_RUNG)) {
//                db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                db.climber.set(EClimberData.DESIRED_POS_deg, 287.5);
//            }
//            else {
//                db.climber.set(EClimberData.DESIRED_VEL_pct, 0);
//            }
//        }
//    }
//    private void updateHangerPneumatics() {
//        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
//            if (db.operatorinput.isSet(InputMap.HANGER.CLAMP_DOUBLE)) {
//                db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
//            } if (db.operatorinput.isSet(InputMap.HANGER.RELEASE_DOUBLE)) {
//                db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
//            }
//
//            if (db.operatorinput.isSet(InputMap.HANGER.CLAMP_SINGLE)) {
//                db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
//            } if (db.operatorinput.isSet(InputMap.HANGER.RELEASE_SINGLE)) {
//                db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
//            }
//        }
//    }


//    private void updateAutoBalance() {
//        if (Robot.mode() == EMatchMode.TELEOPERATED) {
//            if (db.driverinput.isSet(InputMap.DRIVER.MID_RUNG)) { // Left button
//                // schedule command, if already scheduled dont schedule again
//                if(!mAutoBalance.isScheduled()) {
//                    mAutoBalance.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming).schedule();
//                    System.out.println(mAutoBalance.isScheduled());
//                }
//            }
////            else {
////                mAutoBalance.end(true);
////            }
//        }
//    }
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
            }
            // if the cube tracking button is pressed
            else if (db.driverinput.isSet(InputMap.DRIVER.CUBE_TRACKING)) {
                // adjust limelight pipeline so the robot targets cubes
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CUBE);
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









//                // if the limelight is targeting reflective tape
//                if(db.limelight.get(ELimelightData.PIPELINE) == Settings.kReflectiveTapePipelineID)
//                {
//
//                }
//                // if the limelight is targeting cones
//                else if (db.limelight.get(ELimelightData.PIPELINE) == Settings.kConePipelineID)
//                {
//                    // if ___ button is pressed, make adjust limelight pipeline so the robot targets cones
//                }
//                // if the limelight is targeting cubes
//                else if (db.limelight.get(ELimelightData.PIPELINE) == Settings.kCubePipelineID)
//                {
//                    // if ___ button is pressed, make adjust limelight pipeline so the robot targets cubes
//                }
//                if (db.limelight.isSet(ELimelightData.TV)) {
//                    setLED(Enums.LEDColorMode.GREEN, Enums.LEDState.SOLID);
//                } else {
//                    setLED(Enums.LEDColorMode.RED, Enums.LEDState.SOLID);
//                }
//                db.limelight.set(ELimelightData.TARGET_ID, 1);

//                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
//            } else {
//                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CAMERA.id());
//                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.VELOCITY);
//            }
//        } else {
//            if (db.driverinput.isSet(InputMap.DRIVER.TARGET_LOCK)) {
//                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
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





//        if (Robot.mode() == EMatchMode.TELEOPERATED) {
//            if (db.driverinput.isSet(InputMap.DRIVER.TARGET_LOCK)) {
////                if (db.limelight.isSet(ELimelightData.TV)) {
////                    setLED(Enums.LEDColorMode.GREEN, Enums.LEDState.SOLID);
////                } else {
////                    setLED(Enums.LEDColorMode.RED, Enums.LEDState.SOLID);
////                }
//                db.limelight.set(ELimelightData.TARGET_ID, 1);
//
//                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
//            } else {
//                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CAMERA.id());
//                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.VELOCITY);
//            }
//        } else {
//            if (db.driverinput.isSet(InputMap.DRIVER.TARGET_LOCK)) {
//                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
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
//    }
//    private void updateHangerManual() {
//        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
//            if (db.operatorinput.isSet(InputMap.HANGER.SPIN_SINGLE)) {
//                db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
//                db.climber.set(EClimberData.DESIRED_VEL_pct, 0.45);
//            } else if (db.operatorinput.isSet(InputMap.HANGER.SPIN_DOUBLE)) {
//                db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
//                db.climber.set(EClimberData.DESIRED_VEL_pct, -0.45);
//            } else {
//                db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
//                db.climber.set(EClimberData.DESIRED_VEL_pct, 0.0);
//            }
//            if (db.operatorinput.isSet(InputMap.HANGER.CLAMP_DOUBLE)) {
//                db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
//            }
//            else if (db.operatorinput.isSet(InputMap.HANGER.TRAVERSAL_RUNG)) {
//                db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                setIntakeArmEnabled(true);
//                db.climber.set(EClimberData.DESIRED_POS_deg, 287.5);
//            }
//            if (db.operatorinput.isSet(InputMap.HANGER.RELEASE_DOUBLE)) {
//                db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
//            }
//            if (db.operatorinput.isSet(InputMap.HANGER.CLAMP_SINGLE)) {
//                db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
//            }
//            if (db.operatorinput.isSet(InputMap.HANGER.RELEASE_SINGLE)) {
//                db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
//            }
//        } else {
//            db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
//            db.climber.set(EClimberData.DESIRED_VEL_pct, 0.0);
//        }
//    }

//    private Enums.ERungState mLastRungState = Enums.ERungState.NULL;
//    protected void updateRungState() {
//        Enums.ERungState newState = mLastRungState;
//        if (db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
//
//            double angle = db.climber.get(EClimberData.ACTUAL_POSITION_deg);
//            // Determine Rung State
//            switch (mLastRungState) {
//                case NULL:
//                    if (db.driverinput.isSet(InputMap.DRIVER.MID_RUNG)) {
//                        newState = Enums.ERungState.GROUND;
//                    }
//                    break;
//                case GROUND:
//                    if (db.operatorinput.isSet(InputMap.HANGER.GRAB_MID)) {
//                        newState = Enums.ERungState.TRAVEL_TILL_HIT_HIGH;
//                    } else {
//                        newState = Enums.ERungState.GROUND;
//                    }
//                    break;
////                case GO_TO_HIGH_BAR:
////                    if (climberWithinTolerance(1d, angle, Enums.EClimberAngle.HIGH)) {
////                        newState = Enums.ERungState.TRAVEL_TILL_HIT_HIGH;
////                    } else {
////                        newState = Enums.ERungState.GO_TO_HIGH_BAR;
////                    }
////                    break;
//                case TRAVEL_TILL_HIT_HIGH:
//                    if (db.climber.isSet(EClimberData.SINGLE_BEAM_BROKEN)) {
//                        newState = Enums.ERungState.GRAB_HIGH_BAR;
//                    } else {
//                        newState = Enums.ERungState.TRAVEL_TILL_HIT_HIGH;
//                    }
//                    break;
//                case GRAB_HIGH_BAR:
//                    if (db.operatorinput.isSet(InputMap.HANGER.CONFIRM_CLAMPED_ON_HIGH_BAR)) {
//                        newState = Enums.ERungState.BALANCING;
//                    } else {
//                        newState = Enums.ERungState.GRAB_HIGH_BAR;
//                    }
//                    break;
//                case BALANCING:
//                    if (climberWithinTolerance(2, angle, Enums.EClimberAngle.BALANCED)) {
//                        newState = Enums.ERungState.RELEASING_MID;
//                        moveToTraversalTimer.start();
//                    } else {
//                        newState = Enums.ERungState.BALANCING;
//                    }
//                    break;
//                    // If angle = balanced angle, then go to RELEASE_MID state
//                case RELEASING_MID:
//                    //Instantaneously move to next stage to traversal once released
//                    //newState = Enums.ERungState.MOVE_TO_TRAVERSAL;
//                    if(moveToTraversalTimer.get() > 1.0) {
//                         newState = Enums.ERungState.MOVE_TO_TRAVERSAL;
//                    }
//                    break;
//                case MOVE_TO_TRAVERSAL:
//                    if (climberWithinTolerance(5, angle, Enums.EClimberAngle.TRAVERSAL)) {
//                        newState = Enums.ERungState.GRAB_TRAVERSAL;
//                    } else {
//                        newState = Enums.ERungState.MOVE_TO_TRAVERSAL;
//                    }
//                    break;
//                case GRAB_TRAVERSAL:
//                    if (db.operatorinput.isSet(InputMap.HANGER.CONFIRM_CLAMPED_ON_TRAVERSAL_RELEASE_HIGH)) {
//                        newState = Enums.ERungState.RELEASE_HIGH;
//                    }
//                    else {
//                        newState = Enums.ERungState.GRAB_TRAVERSAL;
//                    }
//                    break;
//                case RELEASE_HIGH:
//                    newState = Enums.ERungState.FINAL_LIFT;
//                    break;
//                case FINAL_LIFT:
//                    newState = Enums.ERungState.FINAL_LIFT;
//                    break;
//                default:
//                    newState = Enums.ERungState.NULL;
//
//            }
//
//            switch (newState) {
//                case GROUND:
//                    db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                    db.climber.set(EClimberData.DESIRED_POS_deg, Enums.EClimberAngle.MID.getAngle());
//                    db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
//                    db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
//                    break;
//                case GO_TO_HIGH_BAR:
//                    db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                    db.climber.set(EClimberData.DESIRED_POS_deg, Enums.EClimberAngle.HIGH.getAngle());
//                    db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
//                    break;
//                case TRAVEL_TILL_HIT_HIGH:
//                    db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
//                    db.climber.set(EClimberData.DESIRED_VEL_pct, 0.6);
//                    db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
//                    break;
//                case GRAB_HIGH_BAR:
//                    db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.PERCENT_OUTPUT);
//                    //Should this be in percent output and 0%?
//                    db.climber.set(EClimberData.DESIRED_VEL_pct, 0d);
//                    db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    break;
//                case BALANCING:
//                    db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                    db.climber.set(EClimberData.DESIRED_POS_deg, Enums.EClimberAngle.BALANCED.getAngle());
//                    db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    break;
//                case RELEASING_MID:
//                    db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                    db.climber.set(EClimberData.DESIRED_POS_deg, Enums.EClimberAngle.BALANCED.getAngle());
//                    db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
//                    break;
//                case MOVE_TO_TRAVERSAL:
//                    db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                    db.climber.set(EClimberData.DESIRED_POS_deg, Enums.EClimberAngle.TRAVERSAL.getAngle());
//                    db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.RELEASED);
//                    break;
//                case GRAB_TRAVERSAL:
//                    //dont let it go from this state to the go to traversal until a timer has hit one second
//                    setIntakeArmEnabled(true);
//                    db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                    db.climber.set(EClimberData.DESIRED_POS_deg, Enums.EClimberAngle.TRAVERSAL.getAngle());
//                    db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    break;
//                case RELEASE_HIGH:
//                    setIntakeArmEnabled(true);
//                    db.climber.set(EClimberData.HANGER_STATE, Enums.EClimberMode.POSITION);
//                    db.climber.set(EClimberData.DESIRED_POS_deg, Enums.EClimberAngle.TRAVERSAL.getAngle());
//                    db.climber.set(EClimberData.IS_SINGLE_CLAMPED, Enums.EClampMode.RELEASED);
//                    db.climber.set(EClimberData.IS_DOUBLE_CLAMPED, Enums.EClampMode.CLAMPED);
//                    break;
//                case FINAL_LIFT:
//                    updateHangerManual();
//                    break;
//            }
//        }

        // Set outputs based upon new state
//        mLastRungState = newState;
//        db.climber.set(EClimberData.RUNG_STATE, newState);
//    }

//    private void updateCargo() {
//        db.feeder.set(ADJUSTABLE_FEEDER_PCT, mFeederFireSpeed);
//        if (!db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
//            db.intake.set(ROLLER_STATE, Enums.ERollerState.PERCENT_OUTPUT);
//            if (db.operatorinput.isSet(InputMap.OPERATOR.INCREASE_FEEDER_SPEED) ||
//                    db.operatorinput.isSet(InputMap.OPERATOR.DECREASE_FEEDER_SPEED)) {
//                mPressed = true;
//            } else {
//                mPressed = false;
//            }
//            if (mPressed && !mPrevPressed && db.operatorinput.isSet(InputMap.OPERATOR.INCREASE_FEEDER_SPEED)) {
//                mFeederFireSpeed += 500d;
//            } else if (mPressed && !mPrevPressed && db.operatorinput.isSet(InputMap.OPERATOR.DECREASE_FEEDER_SPEED)) {
//                mFeederFireSpeed -= 500d;
//            }
//            if (db.operatorinput.isSet(InputMap.OPERATOR.SHOOT_CARGO)) {
//                fireCargo();
//            } else if (db.operatorinput.isSet(InputMap.OPERATOR.SPIN_FEEDER)) {
//                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
//                intakeCargo();
//            } else if (db.operatorinput.isSet(InputMap.OPERATOR.PLACE_CARGO)) {
//                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
//                placeCargo();
//            } else if (db.operatorinput.isSet(InputMap.OPERATOR.RELEASE_BALLS)) {
//                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
//                reverseCargo();
//            } else if (db.operatorinput.isSet(InputMap.OPERATOR.STAGE_BALLS)) {
//                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
//                stageBalls();
//            } else {
//                db.feeder.set(STATE, Enums.EFeederState.PERCENT_OUTPUT);
//                mBallsShot = 0;
//                mShotTimer.reset();
//                db.feeder.set(SET_FEEDER_pct, 0d);
//                db.intake.set(DESIRED_ROLLER_pct, 0d);
//            }
//        }
//        mPrevPressed = mPressed;
//    }
//    private void updateIntake() {
//        if (!db.driverinput.isSet(InputMap.DRIVER.ACTIVATE_CLIMB)) {
//            if (db.operatorinput.isSet(InputMap.OPERATOR.EXTEND_INTAKE)) {
//                setIntakeArmEnabled(true);
//            } else if (db.operatorinput.isSet(InputMap.OPERATOR.RETRACT_INTAKE)) {
//                setIntakeArmEnabled(false);
//            }
//        }
//    }
}
