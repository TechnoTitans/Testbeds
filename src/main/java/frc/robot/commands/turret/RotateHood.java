package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;


public class RotateHood extends CommandBase {

    private double angle;
    private TurretSubsystem turret;

    public RotateHood(TurretSubsystem turret, double angle) {
        this.angle = angle; // todo calculations here to convert angle into encoders
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
//        turret.getHoodPID().setSetpoint(angle);
    }

    @Override
    public void execute() {

//        turret.setHood(turret.getHoodPID().calculate(turret.getHoodEncoder().getDistance()));
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()

//        return turret.getHoodPID().atSetpoint();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.setHood(0);
    }
}
