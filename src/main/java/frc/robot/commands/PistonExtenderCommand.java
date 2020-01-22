package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;


public class PistonExtenderCommand extends CommandBase {
    private final ClimbSubsystem climbSubsystem;

    public PistonExtenderCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        climbSubsystem.openPistons();
    }

    @Override
    public boolean isFinished() {
        if (climbSubsystem.isPistonOpen = true) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
