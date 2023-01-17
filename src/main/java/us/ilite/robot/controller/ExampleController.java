package us.ilite.robot.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import us.ilite.common.Distance;
import us.ilite.common.types.drive.EDriveData;
import us.ilite.robot.Enums;
import us.ilite.robot.commands.DriveStraight;
import us.ilite.robot.commands.TurnToDegree;



public class ExampleController extends BaseAutonController {
    private DriveStraight mFirstLeg = new DriveStraight(Distance.fromFeet(5));
    private boolean mFirstLegComplete = false;
    private TurnToDegree mFirstTurn = new TurnToDegree(Rotation2d.fromDegrees(180), 2);
    private boolean mFirstTurnComplete = false;
    private DriveStraight mSecondLeg = new DriveStraight(Distance.fromFeet(5));

    private boolean mSecondLegComplete = false;

    private Timer mTimer;
    private double kSecondTurnTimeEnd;

    public void initialize() {
        mTimer = new Timer();
        mTimer.reset();
        mTimer.start();
    }

    //First leg time was 3.0 seconds shaved to 1.5 (1.5 second taken off)
    private static double
            kFirstLegTimeEnd = 1.5,
            kFirstTurnTimeEnd = kFirstLegTimeEnd + 1,
            kSecondLegTimeEnd = kFirstTurnTimeEnd + 1.5;


        public void updateImpl() {
            double time = mTimer.get();
            if (time < 1.5) {
                db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);

                mFirstTurn.init(time);
                SmartDashboard.putString("Auton State", "Intake Out");
            }
            else if (time < kFirstTurnTimeEnd) {

                mFirstTurnComplete = mFirstTurn.update(time) || time > kFirstTurnTimeEnd;
                if(mFirstTurnComplete) {
                    db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
                }
                SmartDashboard.putString("Auton State", "First turn " + mFirstTurnComplete);
            }
            else if (time < kFirstTurnTimeEnd + 0.1) {

                mFirstLeg.init(time);
            }
            else if (time < kFirstLegTimeEnd) {

                SmartDashboard.putString("Auton State", "First Leg " + mFirstLegComplete);
                mFirstLegComplete = mFirstLeg.update(time) || time > kFirstLegTimeEnd;
                if(mFirstTurnComplete) {
                    db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
                }
            }
            else if (time < kFirstLegTimeEnd + 0.1) {
                mSecondLeg.init(time);

            }
            else if (time < kSecondLegTimeEnd) {

                SmartDashboard.putString("Auton State", "Second Leg " + mSecondLegComplete);
                mSecondLegComplete = mSecondLeg.update(time) || time > kSecondLegTimeEnd;
                if (mSecondLegComplete) {
                    db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
                }
            }
            else {

                setIntakeArmEnabled(false);
            }
//        else if (time < kSecondTurnTimeEnd + 1.1) {
//            db.feeder.set(EFeederData.STATE, Enums.EFeederState.PERCENT_OUTPUT);
//            db.feeder.set(EFeederData.SET_FEEDER_pct, 0.0);
//            db.drivetrain.set(EDriveData.STATE, Enums.EDriveState.RESET);
//            mLeaveTarmac.init(time);
//        }
//        else {
//            mLeaveTarmac.update(time);
//        }

        }
    }


