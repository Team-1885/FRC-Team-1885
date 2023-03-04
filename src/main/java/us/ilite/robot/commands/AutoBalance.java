package us.ilite.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import us.ilite.robot.modules.NeoDriveModule;

public class AutoBalance extends PIDCommand {
        NeoDriveModule driveSubsystem;

        public AutoBalance(NeoDriveModule pDriveSubsystem, double setpoint) {
            super(new PIDController(0.1, 0, 0), pDriveSubsystem::getGyroRollDeg, 1.5, //1.5 is about the base roll in deg
                output -> {
                    pDriveSubsystem.setThrottlePct(-output); //-output * Settings.kMaxSpeedMetersPerSecond
                },
                    pDriveSubsystem);
            driveSubsystem = pDriveSubsystem;
        }

    @Override
    public boolean isFinished() {
            System.out.println("position error " + m_controller.getPositionError());
        return Math.abs(m_controller.getPositionError()) < 2.5;
    }
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        System.out.println("Robot Balanced!");
        driveSubsystem.setThrottlePct(0);
    }
}



//    private class Balance extends PIDCommand{
//        NeoDriveModule driveSubsystem;
//        private Balance(NeoDriveModule driveSubsystem, double setpoint) {
//            super(new PIDController(0.1, 0, 0), driveSubsystem::getGyroRollDeg, setpoint,
//                    output -> {
//                        driveSubsystem.setThrottlePct(output); //-output * Settings.kMaxSpeedMetersPerSecond
//                    },
//            driveSubsystem);
//            this.driveSubsystem = driveSubsystem;
//
//        }
//
//        @Override
//        public boolean isFinished() {
//            return Math.abs(m_controller.getPositionError()) < 1;
//        }
//        @Override
//        public void end(boolean interrupted) {
//            super.end(interrupted);
//            System.out.println("Robot Balanced!");
//            driveSubsystem.setThrottlePct(0);
//        }
//    }


