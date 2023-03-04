package us.ilite.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import us.ilite.common.config.Settings;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
import us.ilite.robot.commands.AutoBalance;
import us.ilite.robot.commands.GenerateRamseteCommand;
import us.ilite.robot.controller.*;
import us.ilite.robot.modules.NeoDriveModule;

import java.lang.reflect.InvocationTargetException;
import java.util.Set;

public class AutonSelection {

    public static ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Autonomous Mode");
    public static int mDelaySeconds;
    private SendableChooser<Command> mSendableAutonControllers = new SendableChooser<>();
    private PathPlannerTrajectory leftPiece;
    private PathPlannerTrajectory leftOrigin;
    private PathPlannerTrajectory mScoringAutomation;
    private PathPlannerTrajectory mGoForward;

    private GenerateRamseteCommand mGenerateRamseteCommand;


    private PathPlannerTrajectory DriveStraight;
    private PathPlannerTrajectory TurnTest;
    private PathPlannerTrajectory Left;
    private PathPlannerTrajectory Right;
    private PathPlannerTrajectory DriveOntoChargeStation;

//    private PathPlannerTrajectory DriveStraight;
    private PathPlannerTrajectory CenterLeft;
    private PathPlannerTrajectory CenterRight;
    private SequentialCommandGroup mCommandGroup;
//    private PathPlannerTrajectory
    private int kMAX_ACCELERATON = 1;
    private int kMAX_VELOCITY = 2;
    private NeoDriveModule mDrive;
    private PPRamseteCommand mRamseteCommand;
    private NeoDriveModule mRobotDrive; // get singleton instance
    private DifferentialDriveKinematics mDriveKinematics; // save instance kDriveKinematics for reuse
    private PIDController mLeftDrivePID;
    private PIDController mRightDrivePID;
    private SimpleMotorFeedforward mFeedForward;
    private PPRamseteCommand mGoForwardCommand;
    private PPRamseteCommand DriveStraightCommand;
    private PPRamseteCommand DriveOntoChargeStationCommand;
    GenerateRamseteCommand commandGenerator;

    private PathPlannerTrajectory DriveOutOfCommunity;
    private PPRamseteCommand DriveOutOfCommunityCommand;

    private AutoBalance mAutoBalance;


    public AutonSelection() {
        commandGenerator = new GenerateRamseteCommand();
        mDrive = NeoDriveModule.getInstance();

//        DriveStraight = PathPlanner.loadPath("DriveStraight", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));
        mGoForward = PathPlanner.loadPath("GoForward", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));
        DriveOntoChargeStation = PathPlanner.loadPath("DriveOntoChargeStation", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));
        DriveOutOfCommunity = PathPlanner.loadPath("DriveOutOfCommunity", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));



        mGoForwardCommand = commandGenerator.generateCommand(mGoForward);
        DriveOutOfCommunityCommand = commandGenerator.generateCommand(DriveOutOfCommunity);
//        DriveStraightCommand = commandGenerator.generateCommand(DriveStraight);
        DriveOntoChargeStationCommand = commandGenerator.generateCommand(DriveOntoChargeStation);

        mRobotDrive = NeoDriveModule.getInstance();
        mAutoBalance = new AutoBalance(mRobotDrive, mRobotDrive.getGyroRollDeg());

        SequentialCommandGroup totalDrive = new SequentialCommandGroup(mGoForwardCommand, mAutoBalance);
//
        mSendableAutonControllers.addOption("totalAutoBalance", totalDrive);
        mSendableAutonControllers.addOption("DriveOutOfCommunity", DriveOutOfCommunityCommand);
//        mSendableAutonControllers.addOption("DriveStraight", DriveStraightCommand);
        mSendableAutonControllers.addOption("DriveOntoChargeStation", DriveOntoChargeStationCommand);

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


//package us.ilite.robot.auto;
//
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import us.ilite.robot.controller.*;
//
//import java.lang.reflect.InvocationTargetException;
//
//public class AutonSelection {
//    public static ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Pre-Match Configuration");
//    public static int mDelaySeconds;
//
//    private SendableChooser<Class<?>> mSendableAutonControllers = new SendableChooser<>();
//
//    /**
//     * Update these Auton Controllers whenever new ones are added
//     */
//    private Class<?>[] mAutonControllers = {
//            ShootMoveController.class,
////            TwoBallController.class,
////            ThreeBallController.class,
////            TexasSwitchController.class,
////            FiveBallController.class,
////            FourBallController.class,
////            TwoBallTrajectoryController.class,
////            FourBallTrajectoryAuton.class,
////            ThreeBallTrajectoryController.class
//    };
//
//    public AutonSelection() {
//       mDelaySeconds = ((Double) (mAutonConfiguration.add("Path Delay Seconds", 0)
//               .withPosition(2, 0)
//               .withSize(2, 1)
//               .getEntry()
//               .getDouble(0.0)))
//               .intValue();
//
////        mSendableAutonControllers.setDefaultOption("Default - Two ball", TwoBallController.class);
//        for (Class<?> c : mAutonControllers) {
//            mSendableAutonControllers.addOption(c.getSimpleName(), c);
//        }
//
//        mAutonConfiguration.add("Choose Auton Controller", mSendableAutonControllers)
//            .withPosition(3, 3)
//            .withSize(4, 2);
//    }
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
//}
