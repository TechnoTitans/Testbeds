package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class RotateTurret extends CommandBase {
    private double angle;
    private double speed;
    public RotateTurret(double angle, double speed) {
        addRequirements(RobotContainer.turret);
        this.angle = angle;
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotContainer.turret.setZMotor(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return RobotContainer.turret.getZMotorEncoder().getDistance() > angle; // some calculations will be made here
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turret.setZMotor(0);
    }
}
