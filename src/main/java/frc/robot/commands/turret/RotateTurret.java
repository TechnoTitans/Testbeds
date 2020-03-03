package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;


public class RotateTurret extends CommandBase {

    private double angle;
    private TurretSubsystem turret;

    public RotateTurret(TurretSubsystem turret, double angle) {
        addRequirements(turret);
        this.turret = turret;
        this.angle = angle; //todo calculations here to convert angle into encoder distance
    }

    @Override
    public void initialize() {

//        turret.getZMotorPID().setSetpoint(angle);
    }

    @Override
    public void execute() {

//        turret.setZMotor(turret.getZMotorPID().calculate(turret.getZMotorEncoder().getDistance()));

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
//        return turret.getZMotorPID().atSetpoint(); // some calculations will be made here
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setZMotor(0);
    }
}
