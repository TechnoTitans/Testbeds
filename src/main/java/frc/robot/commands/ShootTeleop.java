package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;


public class ShootTeleop extends CommandBase {
    TurretSubsystem turret;

    public ShootTeleop(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        turret.setShooter(turret.getSpeedSetpoint());
        SmartDashboard.putNumber("Turret Shooter Speed", turret.getSpeedSetpoint());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setShooter(0);
    }
}
