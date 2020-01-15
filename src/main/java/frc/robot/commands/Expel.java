package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class Expel extends CommandBase {

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.intake.expel();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void cancel() {

    }
}
