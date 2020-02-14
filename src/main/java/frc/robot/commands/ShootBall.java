package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;


public class ShootBall extends CommandBase {
    private double speed;
    private Timer timer = new Timer();
    private TurretSubsystem turret;
    public ShootBall(double speed, TurretSubsystem turret) {
        addRequirements(turret);
        this.turret = turret;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() { turret.setShooter(speed); }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return timer.get() >= 3000; //change later
    }

    @Override
    public void end(boolean interrupted) { turret.setShooter(0); }
}
