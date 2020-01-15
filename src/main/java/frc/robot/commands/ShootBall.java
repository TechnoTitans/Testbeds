package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class ShootBall extends CommandBase {
    private double speed;
    public ShootBall(double speed) {
        addRequirements(RobotContainer.turret);
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.turret.setShooter(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turret.setShooter(0);
    }
}
