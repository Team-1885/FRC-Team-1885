package us.ilite.robot.auto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import us.ilite.robot.commands.FollowTrajectory;
import us.ilite.robot.controller.*;

import java.lang.reflect.InvocationTargetException;

public class AutonSelection {

    public static ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Pre-Match Configuration");
    public static int mDelaySeconds;
    private SendableChooser<Command> mSendableAutonControllers = new SendableChooser<>();
    private FollowTrajectory leftPiece;
    private FollowTrajectory leftOrigin;
    private FollowTrajectory DriveStraight;
    private FollowTrajectory TurnTest;
    private FollowTrajectory Left;
    private FollowTrajectory Right;
    private FollowTrajectory CenterLeft;
    private FollowTrajectory CenterRight;
    private SequentialCommandGroup mCommandGroup;

    public AutonSelection() {
        leftPiece = new FollowTrajectory("LeftPiece");
        leftOrigin = new FollowTrajectory("LeftOrigin");
//        DriveStraight = new FollowTrajectory("DriveStraight");
        TurnTest = new FollowTrajectory("TurnTest");
        Left = new FollowTrajectory("Left");
        Right = new FollowTrajectory("Right");
        CenterLeft = new FollowTrajectory("CenterLeft");
        CenterRight = new FollowTrajectory("CenterRight");
//        mCommandGroup = new SequentialCommandGroup(leftPiece, leftOrigin, DriveStraight);

        mSendableAutonControllers.addOption("left piece", leftPiece);
        mSendableAutonControllers.addOption("left origin", leftOrigin);
        mSendableAutonControllers.addOption("group", mCommandGroup);
        mSendableAutonControllers.addOption("TurnTest", TurnTest);
        mSendableAutonControllers.addOption("DriveStraight", DriveStraight);
        mSendableAutonControllers.addOption("LeftBall", Left);
        mSendableAutonControllers.addOption("RightBall", Right);
        mSendableAutonControllers.addOption("LeftToCharge", CenterLeft);
        mSendableAutonControllers.addOption("RightToCharge", CenterRight);
        SmartDashboard.putData("Autonomous Mode", mSendableAutonControllers);
    }
    public Command getSelectedAutonController() {
        return mSendableAutonControllers.getSelected();
    }

//    public AutonSelection() {
//       mDelaySeconds = ((Double) (mAutonConfiguration.add("Path Delay Seconds", 0)
//               .withPosition(2, 0)
//               .withSize(2, 1)
//               .getEntry()
//               .getDouble(0.0)))
//               .intValue();
//
//        mSendableAutonControllers.setDefaultOption("Default - Two ball", TwoBallController.class);
//        for (Class<?> c : mAutonControllers) {
//            mSendableAutonControllers.addOption(c.getSimpleName(), c);
//        }
//
//        mAutonConfiguration.add("Choose Auton Controller", mSendableAutonControllers)
//            .withPosition(3, 3)
//            .withSize(4, 2);
//    }
//
//    /**
//     * Update these Auton Controllers whenever new ones are added
//     */
//    private Class<?>[] mAutonControllers = {
//            ShootMoveController.class,
//            TwoBallController.class,
//            ThreeBallController.class,
//            TexasSwitchController.class,
////            FiveBallController.class,
////            FourBallController.class,
////            TwoBallTrajectoryController.class,
////            FourBallTrajectoryAuton.class,
////            ThreeBallTrajectoryController.class
//    };
//
//    public BaseAutonController getSelectedAutonController() {
//        try {
//            return (BaseAutonController) mSendableAutonControllers.getSelected().getDeclaredConstructor().newInstance();
//        } catch (NoSuchMethodException nsme) {
//            nsme.printStackTrace();
//        } catch (InstantiationException ie) {
//            ie.printStackTrace();
//        } catch (IllegalAccessException iae) {
//            iae.printStackTrace();
//        } catch (InvocationTargetException ite) {
//            ite.printStackTrace();
//        }
//        // THIS SHOULD NEVER BE REACHED
//        return null;
//    }
}
