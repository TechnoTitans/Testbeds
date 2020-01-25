package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;


public class FeedBall extends CommandBase {
    private double speed;
    private TurretSubsystem turret;
    public FeedBall(double speed, TurretSubsystem turret) {
        this.speed = speed;
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        turret.setBelt(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        // when sensor detects ball
        return turret.getBeltLimitSwitch().get();
    }

    @Override
    public void end(boolean interrupted) {
        turret.setBelt(0);
    }
}
