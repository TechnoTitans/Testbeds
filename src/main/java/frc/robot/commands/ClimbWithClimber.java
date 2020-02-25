package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbWithClimber extends CommandBase {


    private final ClimbSubsystem climb;

    public ClimbWithClimber(ClimbSubsystem climbSubsystem) {
        super();
        this.climb = climbSubsystem;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute() {
        this.climb.pullUp();
    }

    @Override
    public void end(boolean interrupted) {
        this.climb.stopMotor();
    }
}
