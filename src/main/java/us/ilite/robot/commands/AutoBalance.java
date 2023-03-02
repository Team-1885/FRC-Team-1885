package us.ilite.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import us.ilite.robot.modules.NeoDriveModule;

public class AutoBalance extends CommandBase {


    private class Balance() extends PIDCommand {
        public Balance(NeoDriveModule driveSubsystem, double setpoint) {
            super(new PIDController(0.1, 0, 0), driveSubsystem.mGyro::getRoll, setpoint,
                    driveSubsystem::setThrottlePct,
                    driveSubsystem);
//            super(new PIDController(0.1, 0, 0), driveSubsystem.mGyro::getRoll, setpoint,
////                    output -> driveSubsystem.setThrottlePct(output),
//                    driveSubsystem::setThrottlePct,
//////                driveSubsystem.setTurnPct(-output * Settings.kMaxSpeedMetersPerSecond),
////                        driveSubsystem.setThrottlePct(output); //-output * Settings.kMaxSpeedMetersPerSecond
////                    },
//                    driveSubsystem);
//            this.driveSubsystem = driveSubsystem;

        }
//        public Balance(NeoDriveModule driveSubsystem, double setpoint) {
//            super(new PIDController(0.1, 0, 0), driveSubsystem.mGyro::getRoll, setpoint,
//                    driveSubsystem::setThrottlePct,
////                    output -> {
//////                driveSubsystem.setTurnPct(-output * Settings.kMaxSpeedMetersPerSecond),
////                        driveSubsystem.setThrottlePct(output); //-output * Settings.kMaxSpeedMetersPerSecond
////                    },
//                    driveSubsystem);
//            this.driveSubsystem = driveSubsystem;
//
//            mTable.getEntry("Balance").setString("Created Balance Class");
//        }
    }
}
