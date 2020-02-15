package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;



public class TurretAutonomous extends CommandBase {
    private double turretAngle;
    private double hoodAngle;
    private double shooterSpeed;
    private TurretSubsystem turret;
    private Timer shootTime; // time that shooter gets to full speed
    public TurretAutonomous(double turretAngle, double hoodAngle, double shooterSpeed, TurretSubsystem turret) {
        this.turret = turret;
        this.turretAngle = turretAngle; // calculate to encoder
        this.hoodAngle = hoodAngle; // calculate to encoder
        this.shooterSpeed = shooterSpeed;
        shootTime = new Timer();
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.getHoodPID().setSetpoint(hoodAngle);
        turret.getZMotorPID().setSetpoint(turretAngle);
        shootTime.reset();
    }

    @Override
    public void execute() {
        turret.setHood(turret.getHoodPID().calculate(turret.getHoodEncoder().getDistance()));
        turret.setZMotor(turret.getZMotorPID().calculate(turret.getZMotorEncoder().getDistance()));
        turret.setShooter(shooterSpeed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return turret.getZMotorPID().atSetpoint() && turret.getHoodPID().atSetpoint() && shootTime.get() > 3000;
        // may need to change the 3000 ms
        // current pid tolerance is default 0.05
    }

    @Override
    public void end(boolean interrupted) {
        turret.setZMotor(0);
        turret.setHood(0);
    }
}
