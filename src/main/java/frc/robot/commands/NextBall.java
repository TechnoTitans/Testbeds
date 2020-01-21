package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class NextBall extends CommandBase {
    private double speed;
    public NextBall(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.turret.setBelt(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        // when sensor detects ball
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turret.setBelt(0);
    }
}
