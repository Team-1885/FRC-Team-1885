package us.ilite.robot.controller;

import org.junit.Test;

import org.junit.experimental.categories.Category;
import us.ilite.CriticalTest;
import us.ilite.robot.BaseTest;

@Category(CriticalTest.class)
public class DriveTrainUnitTest extends BaseTest {
//    private IntakeModule mIntake;

    @Test
    @Category(CriticalTest.class)
    public void testDrivetrainControl() {
//        db.driverinput.set(InputMap.DRIVER.THROTTLE_AXIS, 0.0);
//        db.driverinput.set(InputMap.DRIVER.TURN_AXIS, 1.0);
//        ctrl.update();
//        assertNormalizedInputs("0% throttle 100% turn");
//
//
//        db.driverinput.set(InputMap.DRIVER.THROTTLE_AXIS, 1.0);
//        db.driverinput.set(InputMap.DRIVER.TURN_AXIS, 1.0);
//        ctrl.update();
//        assertNormalizedInputs("100% throttle 100% turn");
//
//        for(int i = 0; i < 100; i++) {
//            randomizeAllInputs();
//            ctrl.update();
//            assertNormalizedInputs(
//                    "THROTTLE-" + nf.format(db.driverinput.get(DRIVER.THROTTLE_AXIS)) +
//                            "\tTURN-" + nf.format(db.driverinput.get(DRIVER.TURN_AXIS))
//            );
//        }
//
//
//        randomizeAllInputs();
//        db.driverinput.set(InputMap.DRIVER.THROTTLE_AXIS, 0.55);
//        db.driverinput.set(InputMap.DRIVER.TURN_AXIS, 0.5);
//        ctrl.update();
//        Assert.assertTrue(db.drivetrain.isSet(DESIRED_TURN_PCT));
//        Assert.assertTrue(db.drivetrain.isSet(DESIRED_THROTTLE_PCT));
//        // Removing this line since it is possible for the limelight to target lock with the drive train at the moment
////        Assert.assertTrue(db.drivetrain.get(STATE, EDriveState.class) == EDriveState.VELOCITY);
//        System.out.println(db.drivetrain.toVerboseString());
    }

    private void assertNormalizedInputs(String pMessage) {
//        Assert.assertTrue(
//                "Normalized Driver Inputs not <= 1.0, values are THROTTLE= " +
//                        abs(db.drivetrain.get(DESIRED_THROTTLE_PCT))+
//                        ", TURN=" + abs(db.drivetrain.get(DESIRED_TURN_PCT)) + " " +
//                        pMessage,
//                abs(db.drivetrain.get(DESIRED_THROTTLE_PCT)) + abs(db.drivetrain.get(DESIRED_TURN_PCT)) <= 1.0
//        );
    }

}