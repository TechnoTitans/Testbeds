package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.TurretSubsystem;


public class FeedBall extends CommandBase {

    private FeederSubsystem feederSubsystem;

    public FeedBall(FeederSubsystem feeder) {
        this.feederSubsystem = feeder;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {

    }

    // extract constant
    @Override
    public void execute() {
        feederSubsystem.setBelt(.5);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        // when sensor detects ball
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.setBelt(0);
    }
}
