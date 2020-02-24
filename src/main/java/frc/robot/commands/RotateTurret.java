package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;


public class RotateTurret extends CommandBase {
    private double angle;
    private TurretSubsystem turret;
    public RotateTurret(double angle, TurretSubsystem turret) {
        addRequirements(turret);
        this.turret = turret;
        this.angle = angle; //todo calculations here to convert angle into encoder distance
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false; // some calculations will be made here
    }

    @Override
    public void end(boolean interrupted) {
        turret.setZMotor(0);
    }
}
