package us.ilite.robot.controller;

import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import us.ilite.common.Field2022;
import us.ilite.common.config.InputMap;
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
        super.updateBallCount();
        if (Robot.CLIMB_MODE.equals("WCMP")) {
            if (db.operatorinput.isSet(ELogitech310.START)) {
            } else {
            }
        } else {
        }
        updateTargetLock();
        updateAddressableLEDS();
    }

    private void updateTargetLock() {
        if (Robot.mode() == EMatchMode.TELEOPERATED) {
            if (db.driverinput.isSet(InputMap.DRIVER.TARGET_LOCK)) {

                db.limelight.set(ELimelightData.TARGET_ID, 1);

                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
            } else {
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CAMERA.id());
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.VELOCITY);
            }
        } else {
            if (db.driverinput.isSet(InputMap.DRIVER.TARGET_LOCK)) {
                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                    db.pixydata.set(EPixyData.SIGNATURE, Pixy2CCC.CCC_SIG1);
                } else {
                    db.pixydata.set(EPixyData.SIGNATURE, Pixy2CCC.CCC_SIG2);
                }
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.PERCENT_OUTPUT);
            } else {
                db.limelight.set(ELimelightData.TARGET_ID, Field2022.FieldElement.CAMERA.id());
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.VELOCITY);
            }
        }
    }
    private void updateAddressableLEDS() {
        if(db.operatorinput.isSet(ELogitech310.A_BTN) && db.operatorinput.isSet(ELogitech310.B_BTN)) {
            //db.addressableled.get();
        }
    }



}
