package us.ilite.robot.auto;

import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
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
import us.ilite.robot.commands.AutoBalancePID;
import us.ilite.robot.commands.GenerateRamseteCommand;
import us.ilite.robot.controller.*;
import us.ilite.robot.modules.NeoDriveModule;

import java.lang.reflect.InvocationTargetException;
import java.util.List;

public class AutonSelection {
    private RamseteController mRamseteController;

    public static ShuffleboardTab mAutonConfiguration = Shuffleboard.getTab("Pre-Match Configuration");
    public static int mDelaySeconds;
    private SendableChooser<Command> mSendableAutonControllers = new SendableChooser<>();
    private PathPlannerTrajectory leftPiece;
    private PathPlannerTrajectory leftOrigin;
    private PathPlannerTrajectory scoringAutomation;

    private PPRamseteCommand leftPieceCommand;
    private PPRamseteCommand leftOriginCommand;
    private PPRamseteCommand driveStraightCommand;
    private PPRamseteCommand scoringAutomationCommand;


    private PathPlannerTrajectory DriveStraight;

    private AutoBalance autoBalance;
    private AutoBalancePID autoBalancePID;
    private PathPlannerTrajectory TurnTest;
    private PathPlannerTrajectory Left;
    private PathPlannerTrajectory Right;
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
    GenerateRamseteCommand commandGenerator;


    public AutonSelection() {
        commandGenerator = new GenerateRamseteCommand();
        mDrive = NeoDriveModule.getInstance();
        leftPiece = PathPlanner.loadPath("LeftPiece", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));
        leftOrigin = PathPlanner.loadPath("LeftOrigin", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));
        DriveStraight = PathPlanner.loadPath("DriveStraight", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));
        scoringAutomation = PathPlanner.loadPath("ScoringAutomation", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));

//        leftOrigin = PathPlanner.loadPath("LeftOrigin", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));
//        List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("CenterLeft", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));

        leftPieceCommand = commandGenerator.generateCommand(leftPiece);
        leftOriginCommand = commandGenerator.generateCommand(leftOrigin);
        driveStraightCommand = commandGenerator.generateCommand(DriveStraight);
        scoringAutomationCommand = commandGenerator.generateCommand(scoringAutomation);
//        RamseteAutoBuilder RAutoBuilder = new RamseteAutoBuilder(
//                mRobotDrive::getPose,
//                mRamseteController,
//                mFeedForward,
//                mDriveKinematics,
//                mRobotDrive::getWheelSpeeds,
//                mLeftDrivePID, // left controller
//                mRightDrivePID, // right controller
//                // RamseteCommand passes volts to the callback
//                mRobotDrive::setVolts,
//                mRobotDrive);
//        RAutoBuilder.followPath(leftPiece);
//        RAutoBuilder.followPath(leftOrigin);
//        RAutoBuilder.followPathGroup(pathGroup1);

//        leftPiece = new FollowTrajectory("LeftPiece");
//        leftOrigin = new FollowTrajectory("LeftOrigin");
//        DriveStraight = new FollowTrajectory("DriveStraight");
//        TurnTest = new FollowTrajectory("TurnTest");
//        Left = new FollowTrajectory("Left");
//        Right = new FollowTrajectory("Right");
//        CenterLeft = new FollowTrajectory("CenterLeft");
//        CenterRight = new FollowTrajectory("CenterRight");
//        mCommandGroup = new SequentialCommandGroup(leftPiece, leftOrigin, DriveStraight);
        autoBalance = new AutoBalance();
        autoBalancePID = new AutoBalancePID();

        mSendableAutonControllers.addOption("left origin", leftOriginCommand);
        mSendableAutonControllers.addOption("left piece", leftPieceCommand);
        mSendableAutonControllers.addOption("drive straight", driveStraightCommand);
        mSendableAutonControllers.addOption("ScoringAutomation", scoringAutomationCommand);
        mSendableAutonControllers.addOption("auto balance w/pid", autoBalancePID);
        mSendableAutonControllers.addOption("auto balance", autoBalancePID);
//        mSendableAutonControllers.addOption("path group", pathGroup1);
//        mSendableAutonControllers.addOption("group", mCommandGroup);
//        mSendableAutonControllers.addOption("TurnTest", TurnTest);
//        mSendableAutonControllers.addOption("DriveStraight", DriveStraight);
//        mSendableAutonControllers.addOption("LeftBall", Left);
//        mSendableAutonControllers.addOption("RightBall", Right);
//        mSendableAutonControllers.addOption("LeftToCharge", CenterLeft);
//        mSendableAutonControllers.addOption("RightToCharge", CenterRight);
        SmartDashboard.putData("Autonomous Mode", mSendableAutonControllers);
        mRamseteController = new  RamseteController(
                Settings.kRamseteB, // kRamseteB
                Settings.kRamseteZeta // kRamseteZeta
        );
    }
    public Command getSelectedAutonController() {
        return mSendableAutonControllers.getSelected();
    }

//    private PathPlannerRamseteCommand generatePathPlannerRamseteCommand(PathPlannerTrajectory pPathPlannerTrajectory) {
//        PathPlannerRamseteCommand command = new PathPlannerRamseteCommand (
//            pPathPlannerTrajectory, mDrive::getPose, mRamseteController
//        )
//    }

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
