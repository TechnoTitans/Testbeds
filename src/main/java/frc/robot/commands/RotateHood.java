package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class RotateHood extends CommandBase {
    private double speed;
    private double angle;
    public RotateHood(double speed, double angle) {
        this.speed = speed;
        this.angle = angle; // calculations here to convert angle into encoders
        addRequirements(RobotContainer.turret);
    }

    @Override
    public void initialize() {
        RobotContainer.turret.getHoodPID().setSetpoint(angle);
    }

    @Override
    public void execute() {
        RobotContainer.turret.setHood(RobotContainer.turret.getHoodPID().calculate(RobotContainer.turret.getHoodEncoder().getDistance()));
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return RobotContainer.turret.getHoodPID().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turret.setHood(0);
    }
}
