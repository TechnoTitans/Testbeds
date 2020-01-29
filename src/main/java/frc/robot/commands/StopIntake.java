package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntake extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    public StopIntake(IntakeSubsystem intake) {
        intakeSubsystem = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSubsystem.stop();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void cancel() {

    }
}

