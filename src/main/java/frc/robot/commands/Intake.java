package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private double speed;

    public Intake(IntakeSubsystem intake, double speed) {
        this.speed = speed;
        intakeSubsystem = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSubsystem.setSpeed(speed);
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
