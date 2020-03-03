package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;


public class ShootBall extends CommandBase {

    private double speed;
    private TurretSubsystem turret;


    public ShootBall(TurretSubsystem turret, double speed) {
        addRequirements(turret);
        this.turret = turret;
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() { turret.setShooter(speed); }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false; //change later
    }

    @Override
    public void end(boolean interrupted) { turret.setShooter(0); }
}
