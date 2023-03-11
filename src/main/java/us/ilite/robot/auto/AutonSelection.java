package us.ilite.robot.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import us.ilite.robot.commands.GenerateRamseteCommand;
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

    private PathPlannerTrajectory AutobalanceREVERSEFIRST;
    private PPRamseteCommand AutobalanceREVERSECommand;

    private PathPlannerTrajectory AutobalanceFRONTFIRST;
    private PPRamseteCommand AutobalanceFRONTCommand;


    public AutonSelection() {
        commandGenerator = new GenerateRamseteCommand();

        ScorePreloadNODOCK = PathPlanner.loadPath("ScorePreloadNODOCK", new PathConstraints(kMAX_VELOCITY, kMAX_VELOCITY));
        ScorePreloadWITHDOCK = PathPlanner.loadPath("ScorePreloadWITHDOCK", new PathConstraints(kMAX_VELOCITY, kMAX_ACCELERATON));
        ScorePreloadWithTAXI = PathPlanner.loadPath("ScorePreloadWithTAXI", new PathConstraints(kMAX_VELOCITY, kMAX_VELOCITY));
        AutobalanceREVERSEFIRST = PathPlanner.loadPath("AutobalanceREVERSEFIRST", new PathConstraints(kMAX_VELOCITY, kMAX_VELOCITY));
        AutobalanceFRONTFIRST = PathPlanner.loadPath("AutobalanceFRONTFIRST", new PathConstraints(kMAX_VELOCITY, kMAX_VELOCITY));

        ScorePreloadNODOCKCommand = commandGenerator.generateCommand(ScorePreloadNODOCK);
        ScorePreloadWITHDOCKCommand = commandGenerator.generateCommand(ScorePreloadWITHDOCK);
        ScorePreloadWithTAXICommand = commandGenerator.generateCommand(ScorePreloadWithTAXI);
        AutobalanceREVERSECommand = commandGenerator.generateCommand(AutobalanceREVERSEFIRST);
        AutobalanceFRONTCommand = commandGenerator.generateCommand(AutobalanceFRONTFIRST);

        mRobotDrive = NeoDriveModule.getInstance();

        mSendableAutonControllers.addOption("ScorePreload NO DOCK", ScorePreloadNODOCKCommand);
        mSendableAutonControllers.addOption("ScorePreload WITH DOCK", ScorePreloadWITHDOCKCommand);
        mSendableAutonControllers.addOption("ScorePreload WITH TAXI", ScorePreloadWithTAXICommand);
        mSendableAutonControllers.addOption("Autobalance REVERSE", AutobalanceREVERSECommand);
        mSendableAutonControllers.addOption("Autobalance FRONT", AutobalanceFRONTCommand);

        SmartDashboard.putData("Autonomous Mode", mSendableAutonControllers);
    }
    public Command getSelectedAutonController() {
        return mSendableAutonControllers.getSelected();
    }

}
