package us.ilite.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import us.ilite.robot.commands.AutoBalance;
import us.ilite.robot.commands.GenerateRamseteCommand;
import us.ilite.robot.commands.SpinIntake;
import us.ilite.robot.modules.NeoDriveModule;


public class AutonSelection {

    private SendableChooser<Command> mSendableAutonControllers = new SendableChooser<>();

    private int kMAX_ACCELERATON = 1;
    private int kMAX_VELOCITY = 2;

    private NeoDriveModule mRobotDrive; // get singleton instance
    private GenerateRamseteCommand commandGenerator;

    private PathPlannerTrajectory ScorePreloadNODOCK;
    private PPRamseteCommand ScorePreloadNODOCKCommand;

    private PathPlannerTrajectory ScorePreloadWITHDOCK;
    private PPRamseteCommand ScorePreloadWITHDOCKCommand;


    private PathPlannerTrajectory ScorePreloadWithTAXI;
    private PPRamseteCommand ScorePreloadWithTAXICommand;

    private PathPlannerTrajectory BalanceTest;
    private PPRamseteCommand BalanceTestCommand;

    // Score Preload, dock onto charge station, autobalance
    private PathPlannerTrajectory DockWithScoringTrajectory;
    private PPRamseteCommand DockWithScoringCommand;
    private AutoBalance mBalance;
    private SequentialCommandGroup mScoreWithBalance;
    private SpinIntake mSpinIntake;


    public AutonSelection() {
        mRobotDrive = NeoDriveModule.getInstance();

        commandGenerator = new GenerateRamseteCommand();

        ScorePreloadNODOCK = PathPlanner.loadPath("ScorePreloadNODOCK", new PathConstraints(kMAX_VELOCITY, kMAX_VELOCITY));
        ScorePreloadWITHDOCK = PathPlanner.loadPath("ScorePreloadWITHDOCK", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));
        ScorePreloadWithTAXI = PathPlanner.loadPath("ScorePreloadWithTAXI", new PathConstraints(kMAX_VELOCITY, kMAX_VELOCITY));


        ScorePreloadNODOCKCommand = commandGenerator.generateCommand(ScorePreloadNODOCK);
        ScorePreloadWITHDOCKCommand = commandGenerator.generateCommand(ScorePreloadWITHDOCK);
        ScorePreloadWithTAXICommand = commandGenerator.generateCommand(ScorePreloadWithTAXI);

        // scores preload, docks, then autobalances
        DockWithScoringTrajectory = PathPlanner.loadPath("ScorePreload Balance", new PathConstraints(kMAX_VELOCITY, kMAX_VELOCITY));
        DockWithScoringCommand = commandGenerator.generateCommand(DockWithScoringTrajectory); // Dock on the charge station
        mBalance = new AutoBalance(mRobotDrive, mRobotDrive.getGyroRollDeg()); // AutoBalance once docked; setpoint is set to the current roll instead of zero in order to account for gyro drift
        mScoreWithBalance = new SequentialCommandGroup(DockWithScoringCommand, mBalance); // group commands


//        mSendableAutonControllers.setDefaultOption("Score Preload WITH TAXI", ScorePreloadWithTAXICommand);

        mSendableAutonControllers.addOption("ScorePreload NO DOCK", ScorePreloadNODOCKCommand);
        //mSendableAutonControllers.addOption("ScorePreload WITH DOCK", ScorePreloadWITHDOCKCommand);
        mSendableAutonControllers.addOption("ScorePreload WITH TAXI", ScorePreloadWithTAXICommand);
        mSendableAutonControllers.addOption("ScorePreload WITH BALANCE", mScoreWithBalance);

        mSendableAutonControllers.addOption("SPIN INTAKE", mSpinIntake);

        SmartDashboard.putData("Autonomous Mode", mSendableAutonControllers);
    }
    public Command getSelectedAutonController() {
        return mSendableAutonControllers.getSelected();
    }

}

//        /*
//        The balance test command will have the robot starting against the scoring station and will drive
//        forwards until it is *hopefully* balanced/engaged. This command should be tuned first at practice
//        fields to find the end position that will balance. This end position should then be modified in the
//        balance command with scoring.
//         */
//        mSendableAutonControllers.addOption("BalanceTest", BalanceTestCommand);
//        /*
//        The balance test with scoring command will have the bot start against the charge station and facing the
//        Scoring station. make sure the bot is not in contact with the station (this is against the rules).
//        The bot will drive forward to score the cone, and then back up onto the charge station to hopefully engage
//         */
////        mSendableAutonControllers.addOption("BalanceTestWithScoring", BalanceTestWithScoringCommand);
