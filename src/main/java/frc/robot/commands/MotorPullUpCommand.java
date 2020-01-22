package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;


public class MotorPullUpCommand extends CommandBase {
    private final ClimbSubsystem climbSubsystem;

    public MotorPullUpCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //Check the voltage level

        //Limit stuff
        //Climb
        climbSubsystem.pullUp();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
